#!/usr/bin/env python3
"""
Monitor encoder chain for one actuator over CAN.

Reads:
- ENCODER_I2C_UPDATE_COUNTER
- ENCODER_POSITION_RAW
- ENCODER_N_ROTATIONS
- ENCODER_POSITION
- ENCODER_VELOCITY
- POSITION_CONTROLLER_POSITION_MEASURED
- POSITION_CONTROLLER_VELOCITY_MEASURED

Use this while rotating the motor by hand to see which stage changes.
"""

import argparse
import time
import math

import berkeley_humanoid_lite_lowlevel.recoil as recoil


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("-c", "--channel", default="can0")
    p.add_argument("-i", "--id", type=int, required=True)
    p.add_argument("--bitrate", type=int, default=1_000_000)
    p.add_argument("--hz", type=float, default=5.0)
    p.add_argument("--mode", choices=["idle", "damping"], default="idle")
    return p.parse_args()


def fmt(v):
    if v is None:
        return "None"
    if isinstance(v, float):
        if math.isnan(v):
            return "nan"
        return f"{v:+.6f}"
    return str(v)


def main():
    args = parse_args()
    bus = recoil.Bus(channel=args.channel, bitrate=args.bitrate)

    try:
        ok = bus.ping(args.id)
        print(f"[PING] id={args.id}: {'online' if ok else 'offline'}")
        if not ok:
            return 1

        if args.mode == "idle":
            bus.set_mode(args.id, recoil.Mode.IDLE)
            print(f"[MODE] id={args.id} -> IDLE")
        else:
            bus.feed(args.id)
            bus.set_mode(args.id, recoil.Mode.DAMPING)
            print(f"[MODE] id={args.id} -> DAMPING")

        period = 1.0 / max(args.hz, 0.1)
        n = 0
        print()
        print("Rotate the motor by hand and watch which fields change.")
        print("Ctrl+C to stop.")
        print()

        while True:
            n += 1
            t0 = time.time()

            upd = bus._read_parameter_u32(args.id, recoil.Parameter.ENCODER_I2C_UPDATE_COUNTER)
            raw = bus._read_parameter_i32(args.id, recoil.Parameter.ENCODER_POSITION_RAW)
            nrot = bus._read_parameter_i32(args.id, recoil.Parameter.ENCODER_N_ROTATIONS)
            enc_pos = bus._read_parameter_f32(args.id, recoil.Parameter.ENCODER_POSITION)
            enc_vel = bus._read_parameter_f32(args.id, recoil.Parameter.ENCODER_VELOCITY)
            pos_meas = bus.read_position_measured(args.id)
            vel_meas = bus.read_velocity_measured(args.id)

            print(
                f"[{n:05d}] "
                f"upd={fmt(upd):>8}  "
                f"raw={fmt(raw):>12}  "
                f"nrot={fmt(nrot):>8}  "
                f"enc_pos={fmt(enc_pos):>12}  "
                f"enc_vel={fmt(enc_vel):>12}  "
                f"pos_meas={fmt(pos_meas):>12}  "
                f"vel_meas={fmt(vel_meas):>12}"
            )

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
