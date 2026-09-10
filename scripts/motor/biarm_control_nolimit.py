# 양팔 동시 제어 스크립트 (리미트 없음)
# 왼팔 (can0): a/q  s/w  d/e  f/r  g/t  (정/역)
# 오른팔(can1): h/y  j/u  k/i  l/o  ;/p  (정/역)
# ESC 또는 Ctrl+C: 종료
#
# 실행 예시:
#   uv run python biarm_control_nolimit.py \
#       --left_ids  1 3 5 7 9 \
#       --right_ids 2 4 6 8 10

import argparse
import threading

from pynput import keyboard
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# ── 설정 ───────────────────────────────────────────────
KP           = 20.0
KD           = 2.0
TORQUE_LIMIT = 5.0
LOOP_HZ      = 200
MOVE_SPEED   = 1.0
KEY_STEP     = MOVE_SPEED / LOOP_HZ
# ───────────────────────────────────────────────────────

LEFT_KEYS  = [('a','q'), ('s','w'), ('d','e'), ('f','r'), ('g','t')]
RIGHT_KEYS = [('h','y'), ('j','u'), ('k','i'), ('l','o'), (';','p')]


def get_args():
    parser = argparse.ArgumentParser(description="양팔 동시 제어 (리미트 없음)")
    parser.add_argument('--left_chan',  default='can0')
    parser.add_argument('--right_chan', default='can1')
    parser.add_argument('--left_ids',  type=int, nargs=5, required=True,
                        metavar=('ID1','ID2','ID3','ID4','ID5'))
    parser.add_argument('--right_ids', type=int, nargs=5, required=True,
                        metavar=('ID1','ID2','ID3','ID4','ID5'))
    return parser.parse_args()


def init_motor(bus, dev, kp, kd, torque_limit):
    bus.write_position_kp(dev, kp)
    bus.write_position_kd(dev, kd)
    bus.write_torque_limit(dev, torque_limit)

    cur = bus._read_parameter_f32(
        dev,
        recoil.Parameter.POSITION_CONTROLLER_POSITION_MEASURED,
        timeout=0.5
    )
    if cur is not None:
        bus.write_position_offset(dev, cur)

    bus.set_mode(dev, recoil.Mode.POSITION)
    bus.feed(dev)
    print(f"  [모터 {dev:2d}] 초기화 완료")


def main():
    args = get_args()

    left_bus  = recoil.Bus(channel=args.left_chan,  bitrate=1000000)
    right_bus = recoil.Bus(channel=args.right_chan, bitrate=1000000)

    targets      = [0.0] * 10
    pressed_keys = set()
    quit_flag    = threading.Event()

    # 왼팔: 방향 반전 (-1), 오른팔: 정방향 (+1)
    LEFT_DIR  = -1
    RIGHT_DIR = +1

    key_action = {}
    for i, (fwd, rev) in enumerate(LEFT_KEYS):
        key_action[fwd] = (i,     +1)
        key_action[rev] = (i,     -1)
    for i, (fwd, rev) in enumerate(RIGHT_KEYS):
        key_action[fwd] = (5 + i, +1)
        key_action[rev] = (5 + i, -1)

    def on_press(key):
        try:
            pressed_keys.add(key.char)
        except AttributeError:
            pass

    def on_release(key):
        try:
            pressed_keys.discard(key.char)
        except AttributeError:
            if key == keyboard.Key.esc:
                quit_flag.set()
                return False

    print("=" * 65)
    print(f"  왼팔  ({args.left_chan})  :", ' '.join(f"{a}/{b}" for a, b in LEFT_KEYS))
    print(f"  오른팔({args.right_chan}) :", ' '.join(f"{a}/{b}" for a, b in RIGHT_KEYS))
    print(f"  KP={KP}  KD={KD}  TORQUE_LIMIT={TORQUE_LIMIT}")
    print(f"  MOVE_SPEED={MOVE_SPEED} rad/s  |  리미트 없음")
    print("  ESC 또는 Ctrl+C : 종료")
    print("=" * 65)

    print("\n왼팔 초기화:")
    for dev in args.left_ids:
        init_motor(left_bus, dev, KP, KD, TORQUE_LIMIT)

    print("\n오른팔 초기화:")
    for dev in args.right_ids:
        init_motor(right_bus, dev, KP, KD, TORQUE_LIMIT)

    print("\n제어 시작\n")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    rate = RateLimiter(frequency=LOOP_HZ)

    try:
        while not quit_flag.is_set():

            for ch in list(pressed_keys):
                if ch in key_action:
                    idx, direction = key_action[ch]
                    axis_dir = LEFT_DIR if idx < 5 else RIGHT_DIR
                    targets[idx] += KEY_STEP * direction * axis_dir

            # 왼팔
            left_results = []
            for i, dev in enumerate(args.left_ids):
                pos, vel = left_bus.write_read_pdo_2(dev, targets[i], 0.0)
                left_results.append((dev, pos, vel))

            # 오른팔
            right_results = []
            for i, dev in enumerate(args.right_ids):
                pos, vel = right_bus.write_read_pdo_2(dev, targets[5 + i], 0.0)
                right_results.append((dev, pos, vel))

            left_str  = '  '.join(
                f"[{d}]{p:+.2f}" for d, p, v in left_results  if p is not None)
            right_str = '  '.join(
                f"[{d}]{p:+.2f}" for d, p, v in right_results if p is not None)
            print(f"  L: {left_str}  |  R: {right_str}", end='\r')

            rate.sleep()

    except KeyboardInterrupt:
        pass

    finally:
        quit_flag.set()
        listener.stop()

        for dev in args.left_ids:
            left_bus.set_mode(dev, recoil.Mode.IDLE)
        for dev in args.right_ids:
            right_bus.set_mode(dev, recoil.Mode.IDLE)

        left_bus.stop()
        right_bus.stop()
        print("\n\n종료.")


if __name__ == "__main__":
    main()
