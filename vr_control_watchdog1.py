#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Receive test1 Unreal body-tracking UDP JSON and control both 5-DOF arms of
Berkeley Humanoid Lite.

Unreal sends to 192.168.0.65:5000. This receiver binds to 0.0.0.0:5000 by
default so it can receive packets addressed to the robot's 192.168.0.65 NIC.

Controls:
  c: save the current VR pose as neutral
  p: enter POSITION mode from the robot's current calibrated pose
  h: hold the current targets
  x: enter DAMPING mode
  Ctrl+C: stop and enter IDLE

Start with --dry-run. Verify every joint direction before using real motors.

This version adds:
  - a runtime divergence watchdog: every loop, the measured position
    returned by write_read_pdo_2() is compared against the command that
    was just sent. If they differ by more than
    DEFAULT_DIVERGENCE_THRESHOLD_RAD, all motors are switched to DAMPING
    immediately, without waiting for a human to react. The threshold is a
    code constant (not a CLI flag) by design.
  - automatic position_offset cleanup at startup (some other scripts in
    this repo write a stray POSITION_CONTROLLER_POSITION_OFFSET; if it's
    left non-zero, enter_position_from_current()'s raw SDO read and the
    PDO_2 path disagree by exactly that offset, which looks like runaway
    divergence).
  - a per-joint scale factor in RETARGET_CONFIG (separate from the
    max_motion clamp). The clamp only caps the worst case; scale
    proportionally shrinks the whole signal, which is what's needed when
    a joint picks up cross-talk from a neighboring motion.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

try:
    from loop_rate_limiters import RateLimiter
except ImportError:
    class RateLimiter:  # Minimal fallback for dry-run and deployment images.
        def __init__(self, frequency: float):
            self.period = 1.0 / float(frequency)
            self.next_tick = time.monotonic()

        def sleep(self) -> None:
            self.next_tick += self.period
            delay = self.next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                self.next_tick = time.monotonic()


try:
    import berkeley_humanoid_lite_lowlevel.recoil as recoil
except ImportError:
    recoil = None


SIDES = ("left", "right")
JOINT_ORDER = (
    "shoulder_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow_pitch",
    "elbow_roll",
)

DEFAULT_ARM_IDS = {
    "left": (1, 3, 5, 7, 9),
    "right": (2, 4, 6, 8, 10),
}

VR_JOINTS = {
    "left": {
        "upper": "L_UpperArm",
        "lower": "L_LowerArm",
        "wrist": "L_WristTwist",
    },
    "right": {
        "upper": "R_UpperArm",
        "lower": "R_LowerArm",
        "wrist": "R_WristTwist",
    },
}

URDF_JOINT_NAMES = {
    "left": {
        "shoulder_pitch": "arm_left_shoulder_pitch_joint",
        "shoulder_roll": "arm_left_shoulder_roll_joint",
        "shoulder_yaw": "arm_left_shoulder_yaw_joint",
        "elbow_pitch": "arm_left_elbow_pitch_joint",
        "elbow_roll": "arm_left_elbow_roll_joint",
    },
    "right": {
        "shoulder_pitch": "arm_right_shoulder_pitch_joint",
        "shoulder_roll": "arm_right_shoulder_roll_joint",
        "shoulder_yaw": "arm_right_shoulder_yaw_joint",
        "elbow_pitch": "arm_right_elbow_pitch_joint",
        "elbow_roll": "arm_right_elbow_roll_joint",
    },
}

# Embedded from berkeley_humanoid_lite.urdf. --urdf can override these values.
DEFAULT_URDF_LIMITS = {
    "left": {
        "shoulder_pitch": (-0.651, 0.651),
        "shoulder_roll":  (-0.17, 0.602),
        "shoulder_yaw":   (-1.514, 1.606),
        "elbow_pitch":    (-1.601, 0.147),
        "elbow_roll":     (-1.561, 1.575),   # yaml의 wrist_yaw(id 9)
    },
    "right": {
        "shoulder_pitch": (-0.651, 0.651),
        "shoulder_roll":  (-0.602, 0.0),
        "shoulder_yaw":   (-1.561, 1.468),
        "elbow_pitch":    (-0.199, 1.490),
        "elbow_roll":     (-1.467, 1.556),   # yaml의 wrist_yaw(id 10)
    },
}

# Unreal quaternion delta to robot joint delta.
# Tuple format: (source, axis, sign, scale, max_motion)
#   source     : which relative quaternion to read (upper_delta / elbow_delta / wrist_delta)
#   axis       : which Euler component of that quaternion (roll / pitch / yaw)
#   sign       : +1.0 or -1.0, direction correction (verified empirically per joint)
#   scale      : proportional gain applied BEFORE clamping. 1.0 = no attenuation.
#                Use < 1.0 to dampen a joint that picks up cross-talk from a
#                neighboring motion (this shrinks the whole signal, unlike
#                max_motion which only caps the worst case).
#   max_motion : final clamp, radians, applied after scale.
RETARGET_CONFIG = {
    "left": {
        "shoulder_pitch": ("upper_delta", "roll",  +1.0, 1.0, 0.65),
        "shoulder_roll":  ("upper_delta", "yaw",   -1.0, 0.5, 0.55),
        "shoulder_yaw":   ("upper_delta", "pitch", +1.0, 0.5, 0.55),
        "elbow_pitch":    ("elbow_delta", "roll",  -1.0, 1.0, 0.85),
        "elbow_roll":     ("wrist_delta", "pitch", +1.0, 1.0, 0.65),
    },
    "right": {
        "shoulder_pitch": ("upper_delta", "roll",  -1.0, 1.0, 0.65),
        "shoulder_roll":  ("upper_delta", "yaw",   +1.0, 0.5, 0.55),
        "shoulder_yaw":   ("upper_delta", "pitch", -1.0, 0.5, 0.55),
        "elbow_pitch":    ("elbow_delta", "roll",  +1.0, 1.0, 0.85),
        "elbow_roll":     ("wrist_delta", "pitch", -1.0, 1.0, 0.65),
    },
}

DEFAULT_KP = 20.0
DEFAULT_KD = 2.0
DEFAULT_TORQUE_LIMIT = 2.0
DEFAULT_UDP_PORT = 5000
LOOP_HZ = 200.0
UDP_TIMEOUT_SEC = 0.50
MAX_TARGET_SPEED_RAD_S = 0.8

# Position-mode runtime watchdog: if |measured - command| exceeds this many
# radians, all motors are switched to DAMPING immediately. This is a code
# constant by design (not exposed as a CLI flag) so it can't be loosened
# accidentally from the command line.
DEFAULT_DIVERGENCE_THRESHOLD_RAD = 0.3

START_LIMIT_TOLERANCE_RAD = 0.25


def clamp(value: float, low: float, high: float) -> float:
    return min(max(float(value), float(low)), float(high))


def selected_sides(side_arg: str) -> Tuple[str, ...]:
    return SIDES if side_arg == "both" else (side_arg,)


def zero_targets(sides: Iterable[str]) -> Dict[str, Dict[str, float]]:
    return {
        side: {joint: 0.0 for joint in JOINT_ORDER}
        for side in sides
    }


def parse_id_list(value: str) -> Tuple[int, ...]:
    try:
        result = tuple(int(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("motor IDs must be comma-separated integers") from exc
    if len(result) != 5 or len(set(result)) != 5:
        raise argparse.ArgumentTypeError("exactly five unique motor IDs are required")
    return result


def load_urdf_limits(path: Optional[str]) -> Dict[str, Dict[str, Tuple[float, float]]]:
    limits = {
        side: dict(DEFAULT_URDF_LIMITS[side])
        for side in SIDES
    }
    if not path:
        return limits

    urdf_path = Path(path)
    root = ET.parse(urdf_path).getroot()
    joints_by_name = {
        joint.get("name"): joint
        for joint in root.findall("joint")
        if joint.get("name")
    }

    for side in SIDES:
        for logical_name, urdf_name in URDF_JOINT_NAMES[side].items():
            joint = joints_by_name.get(urdf_name)
            limit = None if joint is None else joint.find("limit")
            if limit is None or limit.get("lower") is None or limit.get("upper") is None:
                raise ValueError(f"URDF limit missing for {urdf_name}")
            limits[side][logical_name] = (
                float(limit.get("lower")),
                float(limit.get("upper")),
            )
    return limits


def quat_normalize(q: Sequence[float]) -> np.ndarray:
    arr = np.asarray(q, dtype=np.float64)
    if arr.shape != (4,) or not np.all(np.isfinite(arr)):
        raise ValueError("quaternion must contain four finite numbers")
    norm = float(np.linalg.norm(arr))
    if norm < 1e-9:
        raise ValueError("zero-length quaternion")
    return arr / norm


def quat_inverse(q: np.ndarray) -> np.ndarray:
    x, y, z, w = quat_normalize(q)
    return np.array([-x, -y, -z, w], dtype=np.float64)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = quat_normalize(q1)
    x2, y2, z2, w2 = quat_normalize(q2)
    return quat_normalize((
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ))


def quat_relative(q_ref: np.ndarray, q_cur: np.ndarray) -> np.ndarray:
    return quat_multiply(quat_inverse(q_ref), q_cur)


def quat_to_euler_xyz(q: np.ndarray) -> Tuple[float, float, float]:
    x, y, z, w = quat_normalize(q)
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sin_pitch = clamp(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


def euler_component(q: np.ndarray, axis: str) -> float:
    roll, pitch, yaw = quat_to_euler_xyz(q)
    return {"roll": roll, "pitch": pitch, "yaw": yaw}[axis]


@dataclass(frozen=True)
class BodyTrackingFrame:
    active: bool
    joints: Dict[str, Dict[str, np.ndarray]]
    received_at: float
    source: Tuple[str, int]


class BodyTrackingUdpReceiver:
    def __init__(self, listen_ip: str, listen_port: int):
        self.lock = threading.Lock()
        self.latest: Optional[BodyTrackingFrame] = None
        self.quit = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((listen_ip, listen_port))
        self.sock.settimeout(0.1)
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.quit = True
        try:
            self.sock.close()
        except OSError:
            pass
        self.thread.join(timeout=0.3)

    def get_latest(self) -> Optional[BodyTrackingFrame]:
        with self.lock:
            return self.latest

    def _run(self) -> None:
        while not self.quit:
            try:
                data, source = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                message = json.loads(data.decode("utf-8"))
                frame = self._parse_message(message, source)
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
                print(f"\n[UDP] ignored invalid packet from {source}: {exc}")
                continue

            with self.lock:
                self.latest = frame

    @staticmethod
    def _parse_message(message: dict, source: Tuple[str, int]) -> BodyTrackingFrame:
        if not isinstance(message, dict):
            raise ValueError("JSON root must be an object")

        raw_joints = message.get("joints", [])
        if not isinstance(raw_joints, list):
            raise ValueError("'joints' must be a list")

        joints: Dict[str, Dict[str, np.ndarray]] = {}
        for item in raw_joints:
            if not isinstance(item, dict):
                continue
            name = item.get("name")
            pos = item.get("pos")
            rot = item.get("rot")
            if not isinstance(name, str) or pos is None or rot is None:
                continue
            position = np.asarray(pos, dtype=np.float64)
            if position.shape != (3,) or not np.all(np.isfinite(position)):
                continue
            try:
                rotation = quat_normalize(rot)
            except ValueError:
                continue
            joints[name] = {"pos": position, "rot": rotation}

        return BodyTrackingFrame(
            active=bool(message.get("active", False)),
            joints=joints,
            received_at=time.monotonic(),
            source=source,
        )


class ArmRetargeter:
    def __init__(self, side: str):
        self.side = side
        self.names = VR_JOINTS[side]
        self.neutral: Optional[Dict[str, np.ndarray]] = None

    def has_required_joints(self, frame: Optional[BodyTrackingFrame]) -> bool:
        return bool(
            frame
            and frame.active
            and all(name in frame.joints for name in self.names.values())
        )

    def calibrate(self, frame: BodyTrackingFrame) -> bool:
        if not self.has_required_joints(frame):
            return False
        upper = frame.joints[self.names["upper"]]["rot"]
        lower = frame.joints[self.names["lower"]]["rot"]
        wrist = frame.joints[self.names["wrist"]]["rot"]
        self.neutral = {
            "upper": upper.copy(),
            "elbow_rel": quat_relative(upper, lower),
            "wrist_rel": quat_relative(lower, wrist),
        }
        return True

    def compute_deltas(
        self,
        frame: BodyTrackingFrame,
    ) -> Optional[Dict[str, float]]:
        if self.neutral is None or not self.has_required_joints(frame):
            return None

        upper = frame.joints[self.names["upper"]]["rot"]
        lower = frame.joints[self.names["lower"]]["rot"]
        wrist = frame.joints[self.names["wrist"]]["rot"]
        sources = {
            "upper_delta": quat_relative(self.neutral["upper"], upper),
            "elbow_delta": quat_relative(
                self.neutral["elbow_rel"],
                quat_relative(upper, lower),
            ),
            "wrist_delta": quat_relative(
                self.neutral["wrist_rel"],
                quat_relative(lower, wrist),
            ),
        }

        result: Dict[str, float] = {}
        for joint in JOINT_ORDER:
            source, axis, sign, scale, max_motion = RETARGET_CONFIG[self.side][joint]
            value = euler_component(sources[source], axis) * sign * scale
            result[joint] = clamp(value, -max_motion, max_motion)
        return result


class DualArmMotorController:
    def __init__(
        self,
        channel: str,
        sides: Sequence[str],
        motor_ids: Dict[str, Sequence[int]],
        urdf_limits: Dict[str, Dict[str, Tuple[float, float]]],
        kp: float,
        kd: float,
        torque_limit: float,
        max_speed: float,
        dry_run: bool,
    ):
        self.channel = channel
        self.sides = tuple(sides)
        self.motor_ids = {side: tuple(motor_ids[side]) for side in self.sides}
        self.urdf_limits = urdf_limits
        self.kp = kp
        self.kd = kd
        self.torque_limit = torque_limit
        self.max_speed = max_speed
        self.dry_run = dry_run
        self.position_mode = False
        self.bus = None
        self.base_target = zero_targets(self.sides)
        self.command_target = zero_targets(self.sides)
        self.previous_target = zero_targets(self.sides)

        all_ids = [dev for side in self.sides for dev in self.motor_ids[side]]
        if len(all_ids) != len(set(all_ids)):
            raise ValueError("left and right motor IDs must be unique")
        if not dry_run:
            if recoil is None:
                raise RuntimeError(
                    "recoil import failed; run on the robot or use --dry-run"
                )
            self.bus = recoil.Bus(channel=channel, bitrate=1_000_000)

    def _iter_motors(self):
        for side in self.sides:
            for index, joint in enumerate(JOINT_ORDER):
                yield side, joint, self.motor_ids[side][index]

    def initialize(self) -> None:
        if self.dry_run:
            print("[DRY-RUN] motor initialization skipped")
            return
        assert self.bus is not None
        for side, joint, device_id in self._iter_motors():
            self.bus.set_mode(device_id, recoil.Mode.IDLE)
            time.sleep(0.002)

            # position_offset이 0이 아니면(다른 스크립트가 남긴 잔재) 정리한다.
            # PDO_2 getter/setter가 이 값을 반영하는데 enter_position_from_current()는
            # SDO로 raw 값을 읽으므로, offset이 남아있으면 그 크기만큼 발산이 발생한다.
            offset = self.bus._read_parameter_f32(
                device_id,
                recoil.Parameter.POSITION_CONTROLLER_POSITION_OFFSET,
                timeout=0.5,
            )
            if offset is None:
                print(f"  [{side[0].upper()} {joint:14s}] motor {device_id:2d} offset read failed (timeout), 건너뜀")
            elif abs(offset) > 1e-3:
                print(
                    f"  [{side[0].upper()} {joint:14s}] motor {device_id:2d} "
                    f"position_offset={offset:.4f} -> 0 (flash 저장)"
                )
                self.bus.write_position_offset(device_id, 0.0)
                self.bus.store_settings_to_flash(device_id)
                time.sleep(0.01)

            self.bus.write_position_kp(device_id, self.kp)
            time.sleep(0.002)
            self.bus.write_position_kd(device_id, self.kd)
            time.sleep(0.002)
            self.bus.write_torque_limit(device_id, self.torque_limit)
            time.sleep(0.002)
            self.bus.feed(device_id)
            self.bus.set_mode(device_id, recoil.Mode.DAMPING)
            print(f"  [{side[0].upper()} {joint:14s}] motor {device_id:2d} -> DAMPING")

    def enter_position_from_current(self) -> bool:
        measured = zero_targets(self.sides)
        if self.dry_run:
            print("[DRY-RUN] using URDF zero pose as the motor base")
        else:
            assert self.bus is not None
            for side, joint, device_id in self._iter_motors():
                value = self.bus._read_parameter_f32(
                    device_id,
                    recoil.Parameter.POSITION_CONTROLLER_POSITION_MEASURED,
                    timeout=0.5,
                )
                if value is None or not math.isfinite(float(value)):
                    print(f"\n[MOTOR] position read failed for motor {device_id}")
                    self.enter_damping()
                    return False
                measured[side][joint] = float(value)

        invalid = []
        for side in self.sides:
            for joint in JOINT_ORDER:
                value = measured[side][joint]
                low, high = self.urdf_limits[side][joint]
                if not (low - START_LIMIT_TOLERANCE_RAD <= value <= high + START_LIMIT_TOLERANCE_RAD):
                    invalid.append(
                        f"{side}.{joint}={value:+.3f} outside [{low:+.3f},{high:+.3f}]"
                    )
        if invalid:
            print("\n[SAFETY] POSITION blocked; calibrated pose disagrees with URDF:")
            for message in invalid:
                print(f"  {message}")
            self.enter_damping()
            return False

        self.base_target = {
            side: dict(measured[side])
            for side in self.sides
        }
        self.command_target = {
            side: dict(measured[side])
            for side in self.sides
        }
        self.previous_target = {
            side: dict(measured[side])
            for side in self.sides
        }

        if not self.dry_run:
            assert self.bus is not None
            for _side, _joint, device_id in self._iter_motors():
                self.bus.feed(device_id)
                self.bus.set_mode(device_id, recoil.Mode.POSITION)
                time.sleep(0.002)
        self.position_mode = True
        print("[MOTOR] POSITION mode; current calibrated pose is the neutral base")
        return True

    def enter_damping(self) -> None:
        was_position = self.position_mode
        self.position_mode = False
        if not self.dry_run and self.bus is not None:
            for _side, _joint, device_id in self._iter_motors():
                self.bus.set_mode(device_id, recoil.Mode.DAMPING)
        if was_position or self.dry_run:
            print("[MOTOR] DAMPING mode")

    def enter_idle(self) -> None:
        self.position_mode = False
        if not self.dry_run and self.bus is not None:
            for _side, _joint, device_id in self._iter_motors():
                self.bus.set_mode(device_id, recoil.Mode.IDLE)
        print("[MOTOR] IDLE mode")

    def stop(self) -> None:
        if self.dry_run:
            return
        if self.bus is not None:
            self.enter_idle()
            self.bus.stop()

    def update_targets(
        self,
        deltas: Dict[str, Dict[str, float]],
        dt: float,
    ) -> Dict[str, Dict[str, float]]:
        max_step = self.max_speed * clamp(dt, 0.0, 0.05)
        for side in self.sides:
            for joint in JOINT_ORDER:
                low, high = self.urdf_limits[side][joint]
                desired = clamp(
                    self.base_target[side][joint] + deltas[side][joint],
                    low,
                    high,
                )
                previous = self.previous_target[side][joint]
                target = previous + clamp(desired - previous, -max_step, max_step)
                self.command_target[side][joint] = target
                self.previous_target[side][joint] = target
        return self.command_target

    def send_targets(self) -> Optional[Tuple[str, str, int, float, float]]:
        """Send position commands and watch for divergence.

        Every loop, this compares the measured position (already returned
        by write_read_pdo_2(), previously discarded) against the command
        just sent. If any joint's |measured - command| exceeds
        DEFAULT_DIVERGENCE_THRESHOLD_RAD, all motors are switched to
        DAMPING immediately and the offending joint is returned so the
        caller can log it.

        Returns:
            (side, joint, device_id, command, measured) for the first
            joint that diverged, or None if every joint tracked normally
            (or we are not in position mode / dry-run).
        """
        if not self.position_mode:
            if not self.dry_run and self.bus is not None:
                for _side, _joint, device_id in self._iter_motors():
                    self.bus.feed(device_id)
            return None

        if self.dry_run:
            return None

        assert self.bus is not None
        for side, joint, device_id in self._iter_motors():
            command = float(self.command_target[side][joint])
            pos, _vel = self.bus.write_read_pdo_2(device_id, command, 0.0)

            if pos is not None and abs(pos - command) > DEFAULT_DIVERGENCE_THRESHOLD_RAD:
                self.enter_damping()
                return (side, joint, device_id, command, pos)

        return None


def read_key() -> str:
    if os.name == "nt":
        import msvcrt
        return msvcrt.getwch()

    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def keyboard_worker(state: dict) -> None:
    try:
        while not state["quit"]:
            key = read_key().lower()
            if key == "\x03":
                state["quit"] = True
            elif key == "c":
                state["calibrate"] = True
            elif key == "p":
                state["position"] = True
            elif key == "h":
                state["hold_request"] = True
            elif key == "x":
                state["damping"] = True
    except (EOFError, OSError):
        print("\n[KEYBOARD] interactive keys unavailable; Ctrl+C still works")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="test1 Unreal UDP to Berkeley Humanoid Lite dual-arm control"
    )
    parser.add_argument("--side", choices=("both", "left", "right"), default="both")
    parser.add_argument("--channel", default="can0", help="SocketCAN channel")
    parser.add_argument(
        "--left-ids",
        type=parse_id_list,
        default=DEFAULT_ARM_IDS["left"],
        help="five IDs: shoulder pitch, roll, yaw, elbow pitch, elbow roll",
    )
    parser.add_argument(
        "--right-ids",
        type=parse_id_list,
        default=DEFAULT_ARM_IDS["right"],
        help="five IDs: shoulder pitch, roll, yaw, elbow pitch, elbow roll",
    )
    parser.add_argument(
        "--udp-ip",
        default="0.0.0.0",
        help="local bind address; 0.0.0.0 receives Unreal traffic sent to 192.168.0.65",
    )
    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    parser.add_argument(
        "--urdf",
        default=None,
        help="optional berkeley_humanoid_lite.urdf path; embedded limits are the default",
    )
    parser.add_argument("--kp", type=float, default=DEFAULT_KP)
    parser.add_argument("--kd", type=float, default=DEFAULT_KD)
    parser.add_argument("--torque-limit", type=float, default=DEFAULT_TORQUE_LIMIT)
    parser.add_argument("--max-speed", type=float, default=MAX_TARGET_SPEED_RAD_S)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def print_header(
    args: argparse.Namespace,
    sides: Sequence[str],
    motor_ids: Dict[str, Sequence[int]],
    limits: Dict[str, Dict[str, Tuple[float, float]]],
) -> None:
    print("=" * 84)
    print("test1 -> Berkeley Humanoid Lite dual-arm controller (+ divergence watchdog)")
    print(f"Unreal target : 192.168.0.65:{args.udp_port}")
    print(f"UDP bind      : {args.udp_ip}:{args.udp_port}")
    print(f"CAN / mode    : {args.channel} / {'DRY-RUN' if args.dry_run else 'REAL'}")
    print(f"Arms          : {', '.join(sides)}")
    print(f"Divergence    : {DEFAULT_DIVERGENCE_THRESHOLD_RAD:.3f} rad (code constant, not a CLI flag)")
    for side in sides:
        print(f"[{side.upper()}]")
        for joint, device_id in zip(JOINT_ORDER, motor_ids[side]):
            low, high = limits[side][joint]
            source, axis, sign, scale, motion = RETARGET_CONFIG[side][joint]
            print(
                f"  {joint:14s} motor={device_id:2d} "
                f"URDF=[{low:+.3f},{high:+.3f}] "
                f"VR={source}.{axis} sign={sign:+.0f} scale={scale:.2f} delta=+/-{motion:.2f}"
            )
    print("Keys: c=VR neutral, p=POSITION, h=hold, x=DAMPING, Ctrl+C=quit")
    print("=" * 84)


def main() -> None:
    args = parse_args()
    sides = selected_sides(args.side)
    motor_ids = {
        "left": args.left_ids,
        "right": args.right_ids,
    }
    limits = load_urdf_limits(args.urdf)
    print_header(args, sides, motor_ids, limits)

    state = {
        "quit": False,
        "calibrate": False,
        "position": False,
        "hold_request": False,
        "damping": False,
        "hold": True,
    }

    receiver = BodyTrackingUdpReceiver(args.udp_ip, args.udp_port)
    retargeters = {side: ArmRetargeter(side) for side in sides}
    motor = DualArmMotorController(
        channel=args.channel,
        sides=sides,
        motor_ids=motor_ids,
        urdf_limits=limits,
        kp=args.kp,
        kd=args.kd,
        torque_limit=args.torque_limit,
        max_speed=args.max_speed,
        dry_run=args.dry_run,
    )

    receiver.start()
    motor.initialize()
    threading.Thread(target=keyboard_worker, args=(state,), daemon=True).start()

    rate = RateLimiter(frequency=LOOP_HZ)
    previous_time = time.monotonic()
    last_print = 0.0
    timed_out = False

    try:
        while not state["quit"]:
            now = time.monotonic()
            dt = now - previous_time
            previous_time = now
            frame = receiver.get_latest()
            frame_age = math.inf if frame is None else now - frame.received_at
            tracking_ok = bool(
                frame
                and frame.active
                and frame_age <= UDP_TIMEOUT_SEC
                and all(retargeters[side].has_required_joints(frame) for side in sides)
            )

            if state["calibrate"]:
                state["calibrate"] = False
                calibrated = bool(frame) and all(
                    retargeters[side].calibrate(frame)
                    for side in sides
                )
                if calibrated:
                    state["hold"] = True
                    print("\n[CALIBRATION] both selected arms saved as VR neutral")
                else:
                    print("\n[CALIBRATION] failed; active frame or arm joints are missing")

            if state["position"]:
                state["position"] = False
                if not all(retargeters[side].neutral is not None for side in sides):
                    print("\n[POSITION] blocked; press c with valid tracking first")
                elif not tracking_ok:
                    print("\n[POSITION] blocked; no fresh complete Unreal frame")
                elif motor.enter_position_from_current():
                    state["hold"] = False
                    timed_out = False

            if state["hold_request"]:
                state["hold_request"] = False
                state["hold"] = True
                print("\n[HOLD] current motor targets held")

            if state["damping"]:
                state["damping"] = False
                state["hold"] = True
                motor.enter_damping()

            if tracking_ok and not state["hold"] and motor.position_mode:
                assert frame is not None
                deltas = {
                    side: retargeters[side].compute_deltas(frame)
                    for side in sides
                }
                if all(value is not None for value in deltas.values()):
                    motor.update_targets(deltas, dt)  # type: ignore[arg-type]
                    timed_out = False
            elif frame_age > UDP_TIMEOUT_SEC and motor.position_mode and not timed_out:
                print("\n[SAFETY] Unreal UDP timeout -> DAMPING")
                state["hold"] = True
                motor.enter_damping()
                timed_out = True

            divergence = motor.send_targets()
            if divergence is not None:
                d_side, d_joint, d_device, d_cmd, d_meas = divergence
                print(
                    f"\n[SAFETY] divergence: {d_side}.{d_joint} (motor {d_device}) "
                    f"cmd={d_cmd:+.3f} measured={d_meas:+.3f} "
                    f"diff={abs(d_meas - d_cmd):.3f}rad -> DAMPING"
                )
                state["hold"] = True

            if now - last_print >= 0.2:
                last_print = now
                mode = "POS" if motor.position_mode else "DAMP/IDLE"
                track = "VR_OK" if tracking_ok else "VR_WAIT"
                hold = "HOLD" if state["hold"] else "RUN"
                source = "-" if frame is None else frame.source[0]
                summaries = []
                for side in sides:
                    values = motor.command_target[side]
                    summaries.append(
                        f"{side[0].upper()}:"
                        + ",".join(f"{values[joint]:+.2f}" for joint in JOINT_ORDER)
                    )
                age_text = "inf" if math.isinf(frame_age) else f"{frame_age:.2f}"
                print(
                    f"[{mode} {track} {hold}] src={source} age={age_text}s "
                    + " ".join(summaries),
                    end="\r",
                    flush=True,
                )

            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        state["quit"] = True
        receiver.stop()
        motor.stop()
        print("\nStopped.")


if __name__ == "__main__":
    main()