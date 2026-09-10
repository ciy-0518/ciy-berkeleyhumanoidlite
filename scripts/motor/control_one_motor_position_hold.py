#!/usr/bin/env python3
"""
Single-motor keyboard controller that keeps POSITION mode active while armed.

Behavior
- DISARMED: motor stays in DAMPING.
- ARMED: motor switches to POSITION once and keeps sending the latest target.
- q / w: nudge the held target by one step while staying in POSITION.

This is intended to answer the case where the previous jog script kept returning
immediately to DAMPING after each key press.
"""

from __future__ import annotations

import argparse
import math
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Optional

import berkeley_humanoid_lite_lowlevel.recoil as recoil


MODE_NAME_BY_VALUE = {
    recoil.Mode.DISABLED: "DISABLED",
    recoil.Mode.IDLE: "IDLE",
    recoil.Mode.DAMPING: "DAMPING",
    recoil.Mode.CALIBRATION: "CALIBRATION",
    recoil.Mode.CURRENT: "CURRENT",
    recoil.Mode.TORQUE: "TORQUE",
    recoil.Mode.VELOCITY: "VELOCITY",
    recoil.Mode.POSITION: "POSITION",
    recoil.Mode.VABC_OVERRIDE: "VABC_OVERRIDE",
    recoil.Mode.VALPHABETA_OVERRIDE: "VALPHABETA_OVERRIDE",
    recoil.Mode.VQD_OVERRIDE: "VQD_OVERRIDE",
    recoil.Mode.DEBUG: "DEBUG",
}

ERROR_BITS = []
for _name in dir(recoil.ErrorCode):
    if _name.startswith("_"):
        continue
    _value = getattr(recoil.ErrorCode, _name)
    if isinstance(_value, int):
        ERROR_BITS.append((_name, _value))
ERROR_BITS.sort(key=lambda item: item[1])


class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


@dataclass
class Snapshot:
    mode: Optional[int]
    error: Optional[int]
    upd: Optional[int]
    raw: Optional[int]
    enc_pos: Optional[float]
    enc_vel: Optional[float]
    pos_meas: Optional[float]
    vel_meas: Optional[float]


@dataclass
class State:
    armed: bool = False
    local_fault_latched: bool = False
    hold_target: Optional[float] = None
    last_pdo_pos: Optional[float] = None
    last_pdo_vel: Optional[float] = None
    position_mode_entered: bool = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Single-motor position-hold controller")
    parser.add_argument("-c", "--channel", default="can0", help="CAN channel")
    parser.add_argument("-i", "--id", type=int, required=True, help="Motor CAN device id")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate")
    parser.add_argument("--kp", type=float, default=1.0, help="Position Kp")
    parser.add_argument("--kd", type=float, default=0.05, help="Position Kd (= velocity Kp in this API)")
    parser.add_argument("--torque-limit", type=float, default=0.10, help="Torque limit")
    parser.add_argument("--step", type=float, default=0.005, help="Jog step in rad")
    parser.add_argument("--hold-hz", type=float, default=20.0, help="Rate for sending held POSITION target")
    parser.add_argument("--status-hz", type=float, default=5.0, help="Background status refresh rate")
    parser.add_argument("--command-interval", type=float, default=0.04, help="Minimum interval between successive hold PDO sends")
    return parser.parse_args()


def get_key_nonblocking(timeout: float) -> Optional[str]:
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.read(1)
    return None


def fmt_float(value: Optional[float]) -> str:
    if value is None:
        return "None"
    if math.isnan(value):
        return "nan"
    return f"{value:+.6f}"


def fmt_int(value: Optional[int]) -> str:
    return "None" if value is None else str(value)


def fmt_mode(mode_value: Optional[int]) -> str:
    if mode_value is None:
        return "None"
    return f"{mode_value}({MODE_NAME_BY_VALUE.get(mode_value, 'UNKNOWN')})"


def decode_error(error_value: Optional[int]) -> str:
    if error_value is None:
        return "None"
    if error_value == 0:
        return "0(NO_ERROR)"
    parts = [name for name, bit in ERROR_BITS if bit != 0 and (error_value & bit)]
    if not parts:
        return f"0x{error_value:08X}"
    return f"0x{error_value:08X}({'|'.join(parts)})"


def print_help() -> None:
    print()
    print("=== control_one_motor_position_hold.py ===")
    print("a : arm / disarm (armed = keep POSITION mode active)")
    print("q : decrement held target by one step")
    print("w : increment held target by one step")
    print("s : print compact status")
    print("e : print full diagnostics snapshot")
    print("r : clear local fault latch (does not reset motor)")
    print("z : halve step")
    print("x : double step")
    print("d : force DAMPING mode and disarm")
    print("i : force IDLE mode and disarm")
    print("t : recapture current measured position as hold target")
    print("h : show help")
    print("Ctrl+C or ESC : exit")
    print()


class SingleMotorPositionHoldController:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.bus = recoil.Bus(channel=args.channel, bitrate=args.bitrate)
        self.state = State()
        self.step = float(args.step)
        self.status_period = 1.0 / max(float(args.status_hz), 0.2)
        self.command_period = 1.0 / max(float(args.hold_hz), 1.0)
        self.command_interval = max(float(args.command_interval), 0.0)
        self.last_status_print = 0.0
        self.last_command_time = 0.0

    def close(self) -> None:
        try:
            self.bus.set_mode(self.args.id, recoil.Mode.IDLE)
            print(f"[MODE] id={self.args.id} -> IDLE")
        except Exception as exc:
            print(f"[WARN] Failed to set IDLE on exit: {exc}")
        try:
            self.bus.stop()
            print("[CLOSE] CAN bus closed")
        except Exception as exc:
            print(f"[WARN] Failed to close bus cleanly: {exc}")

    def feed(self) -> None:
        try:
            self.bus.feed(self.args.id)
        except Exception:
            pass

    def set_mode(self, mode: int, label: str) -> None:
        self.feed()
        self.bus.set_mode(self.args.id, mode)
        time.sleep(0.003)
        print(f"[MODE] id={self.args.id} -> {label}")

    def configure(self) -> None:
        ok = self.bus.ping(self.args.id)
        print(f"[PING] id={self.args.id}: {'online' if ok else 'offline'}")
        if not ok:
            raise RuntimeError(f"Motor {self.args.id} did not respond on {self.args.channel}")

        self.bus.set_mode(self.args.id, recoil.Mode.IDLE)
        time.sleep(0.01)
        self.bus.write_position_kp(self.args.id, float(self.args.kp))
        time.sleep(0.01)
        self.bus.write_position_kd(self.args.id, float(self.args.kd))
        time.sleep(0.01)
        self.bus.write_torque_limit(self.args.id, float(self.args.torque_limit))
        time.sleep(0.01)
        self.set_mode(recoil.Mode.DAMPING, "DAMPING")

        snap = self.read_snapshot()
        self.state.hold_target = snap.pos_meas
        print(
            f"[INIT] mode={fmt_mode(snap.mode)}  error={decode_error(snap.error)}  upd={fmt_int(snap.upd)}  "
            f"raw={fmt_int(snap.raw)}  pos={fmt_float(snap.pos_meas)} rad  vel={fmt_float(snap.vel_meas)} rad/s | "
            f"kp={self.args.kp} kd={self.args.kd} torque_limit={self.args.torque_limit} step={self.step}"
        )

    def read_snapshot(self) -> Snapshot:
        def safe(callable_):
            try:
                return callable_()
            except Exception:
                return None

        return Snapshot(
            mode=safe(lambda: self.bus._read_parameter_u32(self.args.id, recoil.Parameter.MODE)),
            error=safe(lambda: self.bus._read_parameter_u32(self.args.id, recoil.Parameter.ERROR)),
            upd=safe(lambda: self.bus._read_parameter_u32(self.args.id, recoil.Parameter.ENCODER_I2C_UPDATE_COUNTER)),
            raw=safe(lambda: self.bus._read_parameter_i32(self.args.id, recoil.Parameter.ENCODER_POSITION_RAW)),
            enc_pos=safe(lambda: self.bus._read_parameter_f32(self.args.id, recoil.Parameter.ENCODER_POSITION)),
            enc_vel=safe(lambda: self.bus._read_parameter_f32(self.args.id, recoil.Parameter.ENCODER_VELOCITY)),
            pos_meas=safe(lambda: self.bus._read_parameter_f32(self.args.id, recoil.Parameter.POSITION_CONTROLLER_POSITION_MEASURED)),
            vel_meas=safe(lambda: self.bus._read_parameter_f32(self.args.id, recoil.Parameter.POSITION_CONTROLLER_VELOCITY_MEASURED)),
        )

    def print_snapshot(self, snap: Snapshot, prefix: str = "[DIAG]") -> None:
        print(
            f"{prefix} mode={fmt_mode(snap.mode)}  error={decode_error(snap.error)}  upd={fmt_int(snap.upd):>8}  "
            f"raw={fmt_int(snap.raw):>8}  enc_pos={fmt_float(snap.enc_pos)}  enc_vel={fmt_float(snap.enc_vel)}  "
            f"pos_meas={fmt_float(snap.pos_meas)}  vel_meas={fmt_float(snap.vel_meas)}  "
            f"target={fmt_float(self.state.hold_target)}  armed={'ON' if self.state.armed else 'OFF'}  "
            f"fault={'ON' if self.state.local_fault_latched else 'OFF'}"
        )

    def print_compact_status(self, prefix: str = "[LIVE]") -> None:
        snap = self.read_snapshot()
        print(
            f"{prefix} raw={fmt_int(snap.raw):>6}  pos={fmt_float(snap.pos_meas)} rad  vel={fmt_float(snap.vel_meas)} rad/s  "
            f"mode={fmt_mode(snap.mode)}  err={decode_error(snap.error)}  upd={fmt_int(snap.upd):>8}  "
            f"target={fmt_float(self.state.hold_target)}  pdo_pos={fmt_float(self.state.last_pdo_pos)}  "
            f"pdo_vel={fmt_float(self.state.last_pdo_vel)}  armed={'ON' if self.state.armed else 'OFF'}  "
            f"fault={'ON' if self.state.local_fault_latched else 'OFF'}  step={self.step:.6f}"
        )

    def clear_local_fault_latch(self) -> None:
        self.state.local_fault_latched = False
        print("[FAULT] Local latch cleared. This does not reset the motor itself.")
        self.print_snapshot(self.read_snapshot(), prefix="[AFTER-CLR]")

    def arm(self) -> None:
        if self.state.local_fault_latched:
            print("[WARN] Fault is latched. Clear only after checking motor state.")
            return

        snap = self.read_snapshot()
        if snap.pos_meas is None:
            print("[ERROR] Cannot arm because current measured position could not be read.")
            self.print_snapshot(snap, prefix="[ARM?]")
            return

        self.state.hold_target = float(snap.pos_meas)
        self.set_mode(recoil.Mode.POSITION, "POSITION")
        self.state.position_mode_entered = True
        self.state.armed = True
        print(f"[ARM] ON  -> holding POSITION at target={self.state.hold_target:+.6f} rad")

    def disarm_to_damping(self) -> None:
        self.state.armed = False
        self.state.position_mode_entered = False
        try:
            self.set_mode(recoil.Mode.DAMPING, "DAMPING")
        except Exception as exc:
            print(f"[WARN] Failed to force DAMPING: {exc}")
        print("[ARM] OFF -> returned to DAMPING")

    def arm_toggle(self) -> None:
        if self.state.armed:
            self.disarm_to_damping()
        else:
            self.arm()

    def recapture_target(self) -> None:
        snap = self.read_snapshot()
        if snap.pos_meas is None:
            print("[WARN] Failed to recapture target because position_measured is unavailable")
            return
        self.state.hold_target = float(snap.pos_meas)
        print(f"[TARGET] recaptured from measured position -> {self.state.hold_target:+.6f} rad")

    def nudge_target(self, delta: float) -> None:
        if self.state.local_fault_latched:
            print("[WARN] Fault is latched. Clear only after checking motor state.")
            return
        if not self.state.armed:
            print("[WARN] Press 'a' first to arm POSITION hold")
            return
        if self.state.hold_target is None:
            snap = self.read_snapshot()
            if snap.pos_meas is None:
                print("[ERROR] Cannot update target because measured position is unavailable")
                return
            self.state.hold_target = float(snap.pos_meas)
        self.state.hold_target += float(delta)
        print(f"[TARGET] delta={delta:+.6f} -> target={self.state.hold_target:+.6f} rad")
        self.send_hold_command(force=True)

    def fault_and_disarm(self, reason: str) -> None:
        self.state.local_fault_latched = True
        print(f"[FAULT] {reason}")
        self.print_snapshot(self.read_snapshot(), prefix="[FAULT-SNAP]")
        self.disarm_to_damping()

    def send_hold_command(self, force: bool = False) -> None:
        if not self.state.armed or self.state.hold_target is None or self.state.local_fault_latched:
            return

        now = time.time()
        if not force and now - self.last_command_time < max(self.command_interval, self.command_period):
            return

        try:
            pdo_pos, pdo_vel = self.bus.write_read_pdo_2(self.args.id, float(self.state.hold_target), 0.0)
            self.state.last_pdo_pos = pdo_pos
            self.state.last_pdo_vel = pdo_vel
            self.last_command_time = now
        except Exception as exc:
            self.state.last_pdo_pos = None
            self.state.last_pdo_vel = None
            self.fault_and_disarm(f"PDO write/read failed in POSITION hold: {exc}")
            return

        snap = self.read_snapshot()
        if snap.error not in (None, 0):
            self.fault_and_disarm(f"Motor ERROR became non-zero in hold mode: {decode_error(snap.error)}")
            return

    def background_tick(self) -> None:
        self.feed()
        if self.state.armed and not self.state.local_fault_latched:
            self.send_hold_command(force=False)

        now = time.time()
        if now - self.last_status_print >= self.status_period:
            self.last_status_print = now
            self.print_compact_status(prefix="[LIVE]")

    def run(self) -> int:
        self.configure()
        print_help()

        with RawTerminal():
            while True:
                key = get_key_nonblocking(timeout=0.05)
                if key is None:
                    self.background_tick()
                    continue

                if key == "a":
                    self.arm_toggle()
                elif key == "q":
                    self.nudge_target(-self.step)
                elif key == "w":
                    self.nudge_target(+self.step)
                elif key == "s":
                    self.print_compact_status(prefix="[STAT]")
                elif key == "e":
                    self.print_snapshot(self.read_snapshot(), prefix="[DIAG]")
                elif key == "r":
                    self.clear_local_fault_latch()
                elif key == "z":
                    self.step *= 0.5
                    print(f"[STEP] -> {self.step:.6f} rad")
                elif key == "x":
                    self.step *= 2.0
                    print(f"[STEP] -> {self.step:.6f} rad")
                elif key == "d":
                    self.disarm_to_damping()
                elif key == "i":
                    self.state.armed = False
                    self.state.position_mode_entered = False
                    self.set_mode(recoil.Mode.IDLE, "IDLE")
                elif key == "t":
                    self.recapture_target()
                elif key == "h":
                    print_help()
                elif key == "\x1b":
                    print("[STOP] ESC received")
                    break

        return 0


def main() -> int:
    args = parse_args()
    controller = SingleMotorPositionHoldController(args)
    try:
        return controller.run()
    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C received")
        return 0
    finally:
        controller.close()


if __name__ == "__main__":
    raise SystemExit(main())
