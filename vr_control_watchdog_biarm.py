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
  - dual-CAN-bus support: left and right arms are on physically separate
    SocketCAN interfaces (can0 / can1 by default), so the motor
    controller now keeps one Bus per side instead of a single shared one.
    This is what makes --side both actually work.
"""

from __future__ import annotations

import argparse
import csv
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

DEFAULT_CHANNELS = {
    "left": "can0",
    "right": "can1",
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

# arm_config.yaml의 실측값(안전 여유 5% 적용됨)으로 교체.
# 기존엔 일반 URDF 모델의 이론값이 들어있었는데, 이 로봇 실측 범위와
# 전혀 일치하지 않았다(부호가 반대인 관절도 있었음). --urdf로 override 가능.
# yaml의 "wrist_yaw"(id 9/10)는 이 코드의 elbow_roll과 같은 모터다.
DEFAULT_URDF_LIMITS = {
    "left": {
        "shoulder_pitch": (-10.0, 10.0),
        "shoulder_roll":  (-10.0, 10.0),
        "shoulder_yaw":   (-10.0, 10.0),
        "elbow_pitch":    (-10.0, 10.0),
        "elbow_roll":     (-10.0, 10.0),
    },
    "right": {
        "shoulder_pitch": (-10.0, 10.0),
        "shoulder_roll":  (-10.0, 10.0),
        "shoulder_yaw":   (-10.0, 10.0),
        "elbow_pitch":    (-10.0, 10.0),
        "elbow_roll":     (-10.0, 10.0),
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
        "shoulder_pitch": ("upper_delta", "roll",  +1.0, 1.0, 2.5),   # ≈ 143°
        "shoulder_roll":  ("upper_delta", "yaw",   -1.0, 1.0, 2.5),
        "shoulder_yaw":   ("upper_delta", "pitch", +1.0, 1.0, 2.5),
        "elbow_pitch":    ("elbow_delta", "roll",  -1.0, 1.0, 2.5),
        "elbow_roll":     ("wrist_delta", "pitch", +1.0, 1.0, 2.5),
    },
    "right": {
        "shoulder_pitch": ("upper_delta", "roll",  -1.0, 1.0, 2.5),
        "shoulder_roll":  ("upper_delta", "yaw",   +1.0, 1.0, 2.5),
        "shoulder_yaw":   ("upper_delta", "pitch", -1.0, 1.0, 2.5),
        "elbow_pitch":    ("elbow_delta", "roll",  +1.0, 1.0, 2.5),
        "elbow_roll":     ("wrist_delta", "pitch", -1.0, 1.0, 2.5),
    },
}
DEFAULT_KP = 20.0
DEFAULT_KD = 2.0
DEFAULT_TORQUE_LIMIT = 5.0
DEFAULT_UDP_PORT = 5000
LOOP_HZ = 200.0
UDP_TIMEOUT_SEC = 0.50
MAX_TARGET_SPEED_RAD_S = 2.5

# EMA smoothing applied to raw VR joint quaternions, per ARM, only when a
# NEW Unreal packet arrives (Unreal sends at ~15Hz; the robot loop runs at
# 200Hz, so gating on frame.received_at is required -- otherwise the same
# stale quaternion would get blended into the filter ~13 times per new
# packet and the filter would just snap to it, doing nothing).
# Lower alpha = more smoothing / more lag. 1.0 = filter disabled.
# euler_probe_log_left_5.csv showed 'up' flipping sign every ~0.4s while
# the operator held a single slow yaw motion -- that's tracking jitter,
# not intentional motion, and this filter targets exactly that.
DEFAULT_EMA_ALPHA = 0.35

# Twist axis (in the upper_delta quaternion's own xyz, NOT world space) used
# for the shoulder's twist-swing decomposition. This is the Y component,
# because empirical axis calibration already established that upper_delta's
# "pitch" (Euler Y-rotation) is shoulder_yaw. See shoulder_axis_components().
SHOULDER_TWIST_AXIS = np.array([0.0, 1.0, 0.0])

# Position-mode runtime watchdog: if |measured - command| exceeds this many
# radians, all motors are switched to DAMPING immediately. This is a code
# constant by design (not exposed as a CLI flag) so it can't be loosened
# accidentally from the command line.
DEFAULT_DIVERGENCE_THRESHOLD_RAD = 1.0

START_LIMIT_TOLERANCE_RAD = 0.25

# Delay between pressing 'c' and actually capturing the VR neutral pose.
# The operator needs to move their hand to the keyboard to press 'c',
# which disturbs the pose they're trying to calibrate as neutral -- this
# gives them time to get back into position before the snapshot is taken.
CALIBRATION_DELAY_SEC = 2.0


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


def shoulder_axis_components(upper_delta: np.ndarray) -> Dict[str, float]:
    """Twist-swing decomposition of the shoulder's upper_delta quaternion.

    Sequential Euler-angle extraction (quat_to_euler_xyz) shares cross
    terms between roll/pitch/yaw whenever more than one axis is active at
    once, which is normal for a 3-DOF ball joint. euler_probe logs
    confirmed this: a shoulder_yaw-only motion produced a strongly
    correlated, sustained (non-jitter) signal on the roll axis
    (corr up<->ur up to -0.82, ur reaching 0.56rad -- almost half of
    shoulder_pitch's whole +/-0.651rad range) and on the yaw axis
    (corr uy<->ur = 0.955 during the shoulder_roll test).

    This decomposition instead projects out the rotation about
    SHOULDER_TWIST_AXIS (Y) first -- that projection is exact for any
    angle, not a small-angle approximation -- and only the remaining
    "swing" (which by construction has zero component along Y) is used
    for the other two axes. This removes the shared-Y cross terms that
    were causing the coupling above, for any rotation magnitude.

    Returns a dict keyed the same as quat_to_euler_xyz's axis names
    ("roll", "pitch", "yaw") so RETARGET_CONFIG entries for upper_delta
    don't need to change: "pitch" is still shoulder_yaw (now the exact
    twist angle), "roll"/"yaw" are still shoulder_pitch/shoulder_roll
    (now swing components with the twist projected out).

    Re-measured after this fix (euler_probe_log_left_{1,3,5}.csv):
    the twist (shoulder_yaw / "pitch") is now cleanly isolated -- e.g.
    corr(sp, sr) = -0.04 during a pure shoulder_yaw motion. But the
    swing plane itself (shoulder_pitch "roll" vs shoulder_roll "yaw")
    still leaks in BOTH directions with DIFFERENT ratios (sy/sr=-0.187
    during a pure-pitch motion, sr/sy=+0.304 during a pure-roll motion).
    Since a single fixed misalignment angle would give the same ratio
    both ways, this isn't just an axis-calibration offset -- it's at
    least partly genuine shoulder anatomy (humeral rotation naturally
    accompanies elevation/abduction to different degrees). See
    suppress_minor_swing_axis() for the practical mitigation applied to
    this remaining pair.
    """
    x, y, z, w = quat_normalize(upper_delta)
    n = SHOULDER_TWIST_AXIS  # already unit length
    proj = float(np.dot((x, y, z), n))  # signed scalar along the twist axis
    twist = quat_normalize(np.array([proj * n[0], proj * n[1], proj * n[2], w]))
    swing = quat_multiply(np.array([x, y, z, w]), quat_inverse(twist))
    sx, sy, sz, sw = quat_normalize(swing)
    return {
        "pitch": 2.0 * math.atan2(proj, w),   # shoulder_yaw (twist, exact)
        "roll": 2.0 * math.atan2(sx, sw),     # shoulder_pitch (swing component)
        "yaw": 2.0 * math.atan2(sz, sw),      # shoulder_roll (swing component)
    }


# How aggressively suppress_minor_swing_axis() damps the smaller of the two
# swing-plane values. 1.0 = proportional damping, higher = more aggressive
# ("winner takes most"). Diagonal reaches (both axes genuinely intended)
# still move, just less than a single-axis motion of the same magnitude.
SWING_SUPPRESSION_EXPONENT = 1.5

# Suppression works well at moderate swing magnitudes but backfires near
# the end of the shoulder's real range of motion, where genuine anatomical
# coupling grows large enough that the "leak" axis can exceed the intended
# axis in raw magnitude (see suppress_minor_swing_axis() docstring).
# Measured clustering: moderate reaches/motions land around 86-93 degrees
# total (|roll|+|yaw|); deliberately extreme motions (e.g. hand-to-head,
# ~90 degrees of pure shoulder_pitch) land around 116-159 degrees total.
# Taper suppression down across that gap so moderate motion keeps the full
# base exponent (unchanged behavior) while extreme motion gets little to
# no suppression (so it isn't crushed the way the fixed exponent=1.5 case
# cut a 90-degree reach down to 25 degrees).
SWING_SUPPRESSION_TAPER_START_RAD = 1.75   # ~100 degrees total; below this, full base exponent
SWING_SUPPRESSION_TAPER_END_RAD = 2.60     # ~149 degrees total; at/above this, exponent -> 0


def effective_swing_suppression_exponent(
    total_magnitude: float,
    base_exponent: float,
    taper_start: float = SWING_SUPPRESSION_TAPER_START_RAD,
    taper_end: float = SWING_SUPPRESSION_TAPER_END_RAD,
) -> float:
    """Scale base_exponent down to 0 as total_magnitude (|roll|+|yaw|,
    BEFORE suppression) crosses from taper_start to taper_end.

    total_magnitude <= taper_start -> base_exponent unchanged
    total_magnitude >= taper_end   -> 0.0 (suppression fully disabled)
    in between                     -> linear interpolation
    """
    if taper_end <= taper_start or total_magnitude <= taper_start:
        return base_exponent
    if total_magnitude >= taper_end:
        return 0.0
    frac = (total_magnitude - taper_start) / (taper_end - taper_start)
    return base_exponent * (1.0 - frac)

# How much inertia the "which swing axis is intended" decision has.
# motor_log6.csv: a shoulder_pitch-only motion produced a raw yaw leak
# that briefly exceeded the raw roll signal itself (see
# suppress_minor_swing_axis() docstring). Comparing instantaneous
# magnitudes got fooled by that. Smoothing the magnitudes used for the
# comparison (NOT the values actually sent to the motor) with a slower
# alpha than DEFAULT_EMA_ALPHA adds hysteresis: the leak has to
# dominate for a sustained stretch, not just one noisy instant, before
# suppression treats it as the intended axis.
SWING_DOMINANCE_EMA_ALPHA = 0.08


def suppress_minor_swing_axis(
    roll_val: float,
    yaw_val: float,
    exponent: float = SWING_SUPPRESSION_EXPONENT,
    dominance_roll: Optional[float] = None,
    dominance_yaw: Optional[float] = None,
) -> Tuple[float, float]:
    """Damp whichever of shoulder_pitch("roll")/shoulder_roll("yaw") is
    smaller in magnitude, relative to the other, at this instant.

    euler_probe measurements show the swing plane leaks in both
    directions with DIFFERENT ratios depending on which axis is the
    intended one (see shoulder_axis_components() docstring) -- a fixed
    geometric correction can't remove this because it isn't a single
    consistent misalignment. This is a pragmatic runtime mitigation
    instead: assume whichever axis is moving the most right now is the
    one the operator intends, and shrink the other axis's contribution
    smoothly rather than zeroing it outright (so genuine diagonal
    motion -- both axes truly intended -- still comes through, just
    reduced versus a pure single-axis motion of the same size).

    exponent=0 disables suppression entirely (weight is always 1.0).

    dominance_roll/dominance_yaw let the caller decide "who's intended"
    using different (typically smoothed) magnitudes than the values
    actually being weighted -- see ArmRetargeter's swing_mag_ema.
    motor_log6.csv showed the raw yaw LEAK briefly exceeding the raw
    roll SIGNAL in magnitude (-1.88 vs -1.06rad) during one direction of
    a shoulder_pitch-only motion; comparing instantaneous |roll_val| vs
    |yaw_val| got fooled by that and suppressed the intended axis
    instead of the leak. Comparing EMA-smoothed magnitudes adds inertia
    so a momentary (or even sustained-but-secondary) crossover doesn't
    immediately flip which axis is treated as intended.
    """
    mag_roll = abs(roll_val) if dominance_roll is None else abs(dominance_roll)
    mag_yaw = abs(yaw_val) if dominance_yaw is None else abs(dominance_yaw)
    total = mag_roll + mag_yaw
    if total < 1e-6 or exponent <= 0.0:
        return roll_val, yaw_val
    weight_roll = (mag_roll / total) ** exponent
    weight_yaw = (mag_yaw / total) ** exponent
    return roll_val * weight_roll, yaw_val * weight_yaw


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
    def __init__(
        self,
        side: str,
        ema_alpha: float = DEFAULT_EMA_ALPHA,
        swing_suppression_exponent: float = SWING_SUPPRESSION_EXPONENT,
        swing_taper_start: float = SWING_SUPPRESSION_TAPER_START_RAD,
        swing_taper_end: float = SWING_SUPPRESSION_TAPER_END_RAD,
    ):
        self.side = side
        self.names = VR_JOINTS[side]
        self.neutral: Optional[Dict[str, np.ndarray]] = None
        self.ema_alpha = float(ema_alpha)
        self.swing_suppression_exponent = float(swing_suppression_exponent)
        self.swing_taper_start = float(swing_taper_start)
        self.swing_taper_end = float(swing_taper_end)
        # Debug snapshot of the shoulder swing plane, before vs after
        # suppress_minor_swing_axis(), for --log-csv to expose. Lets a
        # single test session separate "VR/EMA gave us this raw value"
        # from "suppression then shrank it to this".
        self.last_swing_raw = {"roll": 0.0, "yaw": 0.0}
        self.last_swing_suppressed = {"roll": 0.0, "yaw": 0.0}
        # Slow-moving magnitude estimate used ONLY to decide which swing
        # axis is "intended" (see suppress_minor_swing_axis()). Kept
        # separate from the actual roll/yaw values so the motor still
        # gets the real, unsmoothed signal -- only the suppression
        # weighting decision is stabilized.
        self.swing_mag_ema = {"roll": 0.0, "yaw": 0.0}
        self.filtered: Optional[Dict[str, np.ndarray]] = None
        self._last_frame_at: Optional[float] = None

    def has_required_joints(self, frame: Optional[BodyTrackingFrame]) -> bool:
        return bool(
            frame
            and frame.active
            and all(name in frame.joints for name in self.names.values())
        )

    def _smoothed_joints(self, frame: BodyTrackingFrame) -> Dict[str, np.ndarray]:
        """Return this arm's upper/lower/wrist quaternions, EMA-smoothed.

        Only blends in a new sample when frame.received_at changes, so
        the filter tracks actual new Unreal packets (~15Hz) instead of
        being re-fed the same stale frame on every 200Hz robot tick
        (which would make the filter converge to the raw value
        immediately and defeat the smoothing).
        """
        raw = {
            "upper": frame.joints[self.names["upper"]]["rot"],
            "lower": frame.joints[self.names["lower"]]["rot"],
            "wrist": frame.joints[self.names["wrist"]]["rot"],
        }
        if self.filtered is None:
            self.filtered = {key: value.copy() for key, value in raw.items()}
            self._last_frame_at = frame.received_at
            return self.filtered

        if frame.received_at != self._last_frame_at:
            self._last_frame_at = frame.received_at
            for key, q_new in raw.items():
                q_prev = self.filtered[key]
                if float(np.dot(q_prev, q_new)) < 0.0:
                    q_new = -q_new  # same rotation, opposite quaternion sign; align before blending
                blended = self.ema_alpha * q_new + (1.0 - self.ema_alpha) * q_prev
                self.filtered[key] = quat_normalize(blended)
        return self.filtered

    def calibrate(self, frame: BodyTrackingFrame) -> bool:
        if not self.has_required_joints(frame):
            return False
        joints = self._smoothed_joints(frame)
        upper = joints["upper"]
        lower = joints["lower"]
        wrist = joints["wrist"]
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

        joints = self._smoothed_joints(frame)
        upper = joints["upper"]
        lower = joints["lower"]
        wrist = joints["wrist"]
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
        shoulder_components = shoulder_axis_components(sources["upper_delta"])
        self.last_swing_raw = {
            "roll": shoulder_components["roll"],
            "yaw": shoulder_components["yaw"],
        }
        self.swing_mag_ema["roll"] = (
            SWING_DOMINANCE_EMA_ALPHA * abs(shoulder_components["roll"])
            + (1.0 - SWING_DOMINANCE_EMA_ALPHA) * self.swing_mag_ema["roll"]
        )
        self.swing_mag_ema["yaw"] = (
            SWING_DOMINANCE_EMA_ALPHA * abs(shoulder_components["yaw"])
            + (1.0 - SWING_DOMINANCE_EMA_ALPHA) * self.swing_mag_ema["yaw"]
        )
        shoulder_components["roll"], shoulder_components["yaw"] = suppress_minor_swing_axis(
            shoulder_components["roll"], shoulder_components["yaw"],
            exponent=effective_swing_suppression_exponent(
                abs(self.last_swing_raw["roll"]) + abs(self.last_swing_raw["yaw"]),
                self.swing_suppression_exponent,
                self.swing_taper_start,
                self.swing_taper_end,
            ),
            dominance_roll=self.swing_mag_ema["roll"],
            dominance_yaw=self.swing_mag_ema["yaw"],
        )
        self.last_swing_suppressed = {
            "roll": shoulder_components["roll"],
            "yaw": shoulder_components["yaw"],
        }

        result: Dict[str, float] = {}
        for joint in JOINT_ORDER:
            source, axis, sign, scale, max_motion = RETARGET_CONFIG[self.side][joint]
            if source == "upper_delta":
                value = shoulder_components[axis] * sign * scale
            else:
                value = euler_component(sources[source], axis) * sign * scale
            result[joint] = clamp(value, -max_motion, max_motion)
        return result


class DualArmMotorController:
    """Controls both arms, each potentially on a different CAN bus.

    self.buses maps side -> recoil.Bus. If two sides happen to share the
    same channel string, they share the same Bus object (one SocketCAN
    interface isn't opened twice). self._unique_buses holds the deduped
    list used for shutdown.
    """

    def __init__(
        self,
        channels: Dict[str, str],
        sides: Sequence[str],
        motor_ids: Dict[str, Sequence[int]],
        urdf_limits: Dict[str, Dict[str, Tuple[float, float]]],
        kp: float,
        kd: float,
        torque_limit: float,
        max_speed: float,
        dry_run: bool,
    ):
        self.channels = dict(channels)
        self.sides = tuple(sides)
        self.motor_ids = {side: tuple(motor_ids[side]) for side in self.sides}
        self.urdf_limits = urdf_limits
        self.kp = kp
        self.kd = kd
        self.torque_limit = torque_limit
        self.max_speed = max_speed
        self.dry_run = dry_run
        self.position_mode = False
        self.buses: Dict[str, "recoil.Bus"] = {}
        self._unique_buses: List["recoil.Bus"] = []
        self.base_target = zero_targets(self.sides)
        self.command_target = zero_targets(self.sides)
        self.previous_target = zero_targets(self.sides)
        self.last_measured = zero_targets(self.sides)  # populated by send_targets(); NaN until real data arrives
        self.last_desired = zero_targets(self.sides)  # pre-rate-limit target, for measuring how much max_speed lags behind

        all_ids = [dev for side in self.sides for dev in self.motor_ids[side]]
        if len(all_ids) != len(set(all_ids)):
            raise ValueError("left and right motor IDs must be unique")

        if not dry_run:
            if recoil is None:
                raise RuntimeError(
                    "recoil import failed; run on the robot or use --dry-run"
                )
            opened: Dict[str, "recoil.Bus"] = {}
            for side in self.sides:
                channel = self.channels[side]
                if channel not in opened:
                    opened[channel] = recoil.Bus(channel=channel, bitrate=1_000_000)
                self.buses[side] = opened[channel]
            self._unique_buses = list(opened.values())

    def _bus_for(self, side: str) -> "recoil.Bus":
        return self.buses[side]

    def _iter_motors(self):
        for side in self.sides:
            for index, joint in enumerate(JOINT_ORDER):
                yield side, joint, self.motor_ids[side][index]

    def initialize(self) -> None:
        if self.dry_run:
            print("[DRY-RUN] motor initialization skipped")
            return
        for side, joint, device_id in self._iter_motors():
            bus = self._bus_for(side)
            bus.set_mode(device_id, recoil.Mode.IDLE)
            time.sleep(0.002)

            # position_offset이 0이 아니면(다른 스크립트가 남긴 잔재) 정리한다.
            # PDO_2 getter/setter가 이 값을 반영하는데 enter_position_from_current()는
            # SDO로 raw 값을 읽으므로, offset이 남아있으면 그 크기만큼 발산이 발생한다.
            offset = bus._read_parameter_f32(
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
                bus.write_position_offset(device_id, 0.0)
                bus.store_settings_to_flash(device_id)
                time.sleep(0.01)

            bus.write_position_kp(device_id, self.kp)
            time.sleep(0.002)
            bus.write_position_kd(device_id, self.kd)
            time.sleep(0.002)
            bus.write_torque_limit(device_id, self.torque_limit)
            time.sleep(0.002)
            bus.feed(device_id)
            bus.set_mode(device_id, recoil.Mode.DAMPING)
            print(f"  [{side[0].upper()} {joint:14s}] motor {device_id:2d} -> DAMPING")

    def enter_position_from_current(self) -> bool:
        measured = zero_targets(self.sides)
        if self.dry_run:
            print("[DRY-RUN] using URDF zero pose as the motor base")
        else:
            for side, joint, device_id in self._iter_motors():
                bus = self._bus_for(side)
                value = bus._read_parameter_f32(
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
            for _side, _joint, device_id in self._iter_motors():
                bus = self._bus_for(_side)
                bus.feed(device_id)
                bus.set_mode(device_id, recoil.Mode.POSITION)
                time.sleep(0.002)
        self.position_mode = True
        print("[MOTOR] POSITION mode; current calibrated pose is the neutral base")
        return True

    def enter_damping(self) -> None:
        was_position = self.position_mode
        self.position_mode = False
        if not self.dry_run:
            for _side, _joint, device_id in self._iter_motors():
                self._bus_for(_side).set_mode(device_id, recoil.Mode.DAMPING)
        if was_position or self.dry_run:
            print("[MOTOR] DAMPING mode")

    def enter_idle(self) -> None:
        self.position_mode = False
        if not self.dry_run:
            for _side, _joint, device_id in self._iter_motors():
                self._bus_for(_side).set_mode(device_id, recoil.Mode.IDLE)
        print("[MOTOR] IDLE mode")

    def stop(self) -> None:
        if self.dry_run:
            return
        self.enter_idle()
        for bus in self._unique_buses:
            bus.stop()

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
                self.last_desired[side][joint] = desired
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
            if not self.dry_run:
                for _side, _joint, device_id in self._iter_motors():
                    self._bus_for(_side).feed(device_id)
            return None

        if self.dry_run:
            return None

        for side, joint, device_id in self._iter_motors():
            bus = self._bus_for(side)
            command = float(self.command_target[side][joint])
            pos, _vel = bus.write_read_pdo_2(device_id, command, 0.0)
            if pos is not None:
                self.last_measured[side][joint] = float(pos)

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
                delay = state.get("calibrate_delay", CALIBRATION_DELAY_SEC)
                state["calibrate_requested_at"] = time.monotonic()
                print(f"\n[CALIBRATE] hold your pose -- capturing in {delay:.1f}s")
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
    parser.add_argument(
        "--left-channel",
        default=DEFAULT_CHANNELS["left"],
        help="SocketCAN channel for the left arm (default can0)",
    )
    parser.add_argument(
        "--right-channel",
        default=DEFAULT_CHANNELS["right"],
        help="SocketCAN channel for the right arm (default can1)",
    )
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
    parser.add_argument(
        "--ema-alpha",
        type=float,
        default=DEFAULT_EMA_ALPHA,
        help="VR quaternion smoothing, applied per new Unreal packet. "
             "Lower = smoother/more lag, 1.0 = off (default %(default)s)",
    )
    parser.add_argument(
        "--swing-suppression-exponent",
        type=float,
        default=0.0,
        help="Base suppress_minor_swing_axis() strength at moderate swing "
             "magnitudes (below --swing-taper-start). Default is 0 "
             "(disabled) -- raw shoulder_pitch/shoulder_roll pass through "
             "unmodified. Set > 0 (e.g. 1.5) to re-enable magnitude-based "
             "cross-talk suppression; see SWING_SUPPRESSION_EXPONENT and "
             "effective_swing_suppression_exponent() for what it does and "
             "its known trade-offs.",
    )
    parser.add_argument(
        "--swing-taper-start",
        type=float,
        default=SWING_SUPPRESSION_TAPER_START_RAD,
        help="Total |roll|+|yaw| (rad) below which suppression stays at "
             "full strength (default %(default)s)",
    )
    parser.add_argument(
        "--swing-taper-end",
        type=float,
        default=SWING_SUPPRESSION_TAPER_END_RAD,
        help="Total |roll|+|yaw| (rad) at/above which suppression is fully "
             "disabled -- large, near-end-of-range motions have genuine "
             "anatomical coupling that can exceed the intended axis, which "
             "fools magnitude-based suppression into crushing the real "
             "signal (default %(default)s)",
    )
    parser.add_argument(
        "--calibrate-delay",
        type=float,
        default=CALIBRATION_DELAY_SEC,
        help="Seconds between pressing 'c' and actually capturing the VR "
             "neutral pose, so moving your hand to the keyboard doesn't "
             "disturb the pose (default %(default)s)",
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--log-csv",
        type=str,
        default=None,
        help="Write command_target vs last_measured for every joint, ~20Hz, "
             "to this CSV path. Works in --dry-run too (measured columns "
             "stay 0.0 there since no real motor feedback exists).",
    )
    return parser.parse_args()


def print_header(
    args: argparse.Namespace,
    sides: Sequence[str],
    channels: Dict[str, str],
    motor_ids: Dict[str, Sequence[int]],
    limits: Dict[str, Dict[str, Tuple[float, float]]],
) -> None:
    print("=" * 84)
    print("test1 -> Berkeley Humanoid Lite dual-arm controller (+ divergence watchdog)")
    print(f"Unreal target : 192.168.0.65:{args.udp_port}")
    print(f"UDP bind      : {args.udp_ip}:{args.udp_port}")
    print(f"Mode          : {'DRY-RUN' if args.dry_run else 'REAL'}")
    print(f"Arms          : {', '.join(sides)}")
    print(f"Divergence    : {DEFAULT_DIVERGENCE_THRESHOLD_RAD:.3f} rad (code constant, not a CLI flag)")
    print(f"EMA alpha     : {args.ema_alpha:.2f} (--ema-alpha; 1.0 = off)")
    print(f"Calibrate delay: {args.calibrate_delay:.1f}s (--calibrate-delay; time to get into pose after 'c')")
    print(
        f"Swing suppress: base_exponent={args.swing_suppression_exponent} "
        f"taper=[{args.swing_taper_start:.2f},{args.swing_taper_end:.2f}]rad "
        f"(full strength below taper_start, 0 at/above taper_end)"
    )
    for side in sides:
        print(f"[{side.upper()}] channel={channels[side]}")
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
    channels = {
        "left": args.left_channel,
        "right": args.right_channel,
    }
    motor_ids = {
        "left": args.left_ids,
        "right": args.right_ids,
    }
    limits = load_urdf_limits(args.urdf)
    print_header(args, sides, channels, motor_ids, limits)

    state = {
        "quit": False,
        "calibrate_requested_at": None,
        "calibrate_delay": args.calibrate_delay,
        "position": False,
        "hold_request": False,
        "damping": False,
        "hold": True,
    }

    receiver = BodyTrackingUdpReceiver(args.udp_ip, args.udp_port)
    retargeters = {
        side: ArmRetargeter(
            side,
            ema_alpha=args.ema_alpha,
            swing_suppression_exponent=args.swing_suppression_exponent,
            swing_taper_start=args.swing_taper_start,
            swing_taper_end=args.swing_taper_end,
        )
        for side in sides
    }
    motor = DualArmMotorController(
        channels=channels,
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
    last_log = 0.0
    timed_out = False

    log_writer = None
    log_file = None
    if args.log_csv:
        log_file = open(args.log_csv, "w", newline="")
        log_writer = csv.writer(log_file)
        header = ["t", "mode", "tracking", "hold"]
        for side in sides:
            header += [
                f"{side}_swing_roll_raw", f"{side}_swing_yaw_raw",
                f"{side}_swing_roll_suppressed", f"{side}_swing_yaw_suppressed",
            ]
            for joint in JOINT_ORDER:
                header += [f"{side}_{joint}_cmd", f"{side}_{joint}_meas", f"{side}_{joint}_desired"]
        log_writer.writerow(header)
        print(f"Logging to    : {args.log_csv} (~20Hz)")
        print(f"Swing suppress: base_exponent={args.swing_suppression_exponent} taper=[{args.swing_taper_start:.2f},{args.swing_taper_end:.2f}]rad")

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

            if state["calibrate_requested_at"] is not None:
                elapsed = now - state["calibrate_requested_at"]
                remaining = state["calibrate_delay"] - elapsed
                if remaining <= 0.0:
                    state["calibrate_requested_at"] = None
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

            if log_writer is not None and now - last_log >= 0.05:
                last_log = now
                row = [
                    f"{now:.3f}",
                    "POS" if motor.position_mode else "DAMP/IDLE",
                    "1" if tracking_ok else "0",
                    "1" if state["hold"] else "0",
                ]
                for side in sides:
                    swing_raw = retargeters[side].last_swing_raw
                    swing_supp = retargeters[side].last_swing_suppressed
                    row += [
                        f"{swing_raw['roll']:.4f}", f"{swing_raw['yaw']:.4f}",
                        f"{swing_supp['roll']:.4f}", f"{swing_supp['yaw']:.4f}",
                    ]
                    for joint in JOINT_ORDER:
                        row.append(f"{motor.command_target[side][joint]:.4f}")
                        row.append(f"{motor.last_measured[side][joint]:.4f}")
                        row.append(f"{motor.last_desired[side][joint]:.4f}")
                log_writer.writerow(row)
                log_file.flush()

            if now - last_print >= 0.2:
                last_print = now
                if state["calibrate_requested_at"] is not None:
                    remaining = max(0.0, state["calibrate_delay"] - (now - state["calibrate_requested_at"]))
                    print(f"[CALIBRATE] hold still... {remaining:.1f}s          ", end="\r", flush=True)
                else:
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
        if log_file is not None:
            log_file.close()
            print(f"\nLog saved: {args.log_csv}")
        print("\nStopped.")


if __name__ == "__main__":
    main()
