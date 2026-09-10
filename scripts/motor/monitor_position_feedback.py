#!/usr/bin/env python3
"""
Simple monitor for actuator position feedback.

Purpose:
- Open a CAN bus
- Ping one motor
- Put it into a non-position-holding mode (default: IDLE)
- Repeatedly read measured position / velocity
- Let the user rotate the motor by hand and verify whether feedback changes

Example:
    uv run python ./scripts/motor/monitor_position_feedback.py -c can0 -i 7

Safer examples:
    uv run python ./scripts/motor/monitor_position_feedback.py -c can0 -i 7 --mode idle
    uv run python ./scripts/motor/monitor_position_feedback.py -c can0 -i 7 --mode damping
"""

import argparse
import math
import time

import berkeley_humanoid_lite_lowlevel.recoil as recoil


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--channel", type=str, default="can0", help="CAN channel")
    parser.add_argument("-i", "--id", type=int, required=True, help="Actuator device id")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate")
    parser.add_argument(
        "--mode",
        type=str,
        default="idle",
        choices=["idle", "damping"],
        help="Mode to set before monitoring. Default: idle",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=10.0,
        help="Read rate in Hz. Default: 10",
    )
    parser.add_argument(
        "--change-threshold",
        type=float,
        default=0.001,
        help="Print 'changed' marker when delta from last printed sample exceeds this rad value",
    )
    return parser.parse_args()


def set_safe_mode(bus: recoil.Bus, device_id: int, mode_name: str) -> None:
    if mode_name == "idle":
        bus.set_mode(device_id, recoil.Mode.IDLE)
        print(f"[MODE] id={device_id} -> IDLE")
    elif mode_name == "damping":
        bus.feed(device_id)
        bus.set_mode(device_id, recoil.Mode.DAMPING)
        print(f"[MODE] id={device_id} -> DAMPING")


def fmt_raw(v) -> str:
    if v is None:
        return "None"
    return str(v)


def fmt_float(v) -> str:
    if v is None:
        return "None"
    if math.isnan(v):
        return "nan"
    return f"{v:+.6f}"


def main() -> int:
    args = parse_args()
    period = 1.0 / max(args.hz, 0.1)

    print(f"[OPEN] channel={args.channel} bitrate={args.bitrate} id={args.id}")
    bus = recoil.Bus(channel=args.channel, bitrate=args.bitrate)

    try:
        ok = bus.ping(args.id)
        print(f"[PING] id={args.id}: {'online' if ok else 'offline'}")
        if not ok:
            print("[ERROR] Motor did not respond to ping.")
            return 1

        set_safe_mode(bus, args.id, args.mode)

        raw0 = bus._read_parameter_i32(args.id, recoil.Parameter.ENCODER_POSITION_RAW)
        pos0 = bus.read_position_measured(args.id)
        vel0 = bus.read_velocity_measured(args.id)

        if pos0 is None:
            print("[ERROR] Failed to read initial position.")
            return 1

        if vel0 is None:
            vel0 = float("nan")

        print()
        print("=== Position Feedback Monitor ===")
        print("Rotate the motor by hand and watch whether measured position changes.")
        print("Ctrl+C to stop.")
        print(f"Initial raw:      {fmt_raw(raw0)}")
        print(f"Initial position: {pos0:+.6f} rad")
        print(f"Initial velocity: {vel0:+.6f} rad/s")
        print()

        last_printed_pos = pos0
        sample_count = 0

        while True:
            t0 = time.time()

            raw = bus._read_parameter_i32(args.id, recoil.Parameter.ENCODER_POSITION_RAW)
            pos = bus.read_position_measured(args.id)
            vel = bus.read_velocity_measured(args.id)

            sample_count += 1

            if pos is None:
                print(
                    f"[{sample_count:06d}] "
                    f"raw={fmt_raw(raw):>8}  "
                    f"pos=None  "
                    f"vel={fmt_float(vel)}  <-- read timeout / no data"
                )
            else:
                delta_from_start = pos - pos0
                delta_from_last = pos - last_printed_pos
                changed = abs(delta_from_last) >= args.change_threshold

                marker = " <== changed" if changed else ""
                print(
                    f"[{sample_count:06d}] "
                    f"raw={fmt_raw(raw):>8}  "
                    f"pos={pos:+.6f} rad  "
                    f"vel={fmt_float(vel)} rad/s  "
                    f"delta_start={delta_from_start:+.6f} rad"
                    f"{marker}"
                )

                if changed:
                    last_printed_pos = pos

            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)

    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C received.")
    finally:
        try:
            bus.set_mode(args.id, recoil.Mode.IDLE)
            print(f"[MODE] id={args.id} -> IDLE")
        except Exception as exc:
            print(f"[WARN] Failed to set IDLE on exit: {exc}")

        try:
            bus.stop()
            print("[CLOSE] CAN bus closed.")
        except Exception as exc:
            print(f"[WARN] Failed to close bus cleanly: {exc}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())