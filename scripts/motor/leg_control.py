# 단일 다리 제어 스크립트 (모터 6개 동시 제어, yaml 설정 없이 동작)
# 왼쪽/오른쪽 구분은 이 스크립트 안에서 하지 않고, 실행 시 --chan/--ids
# 인자를 다르게 줘서 각각 별도 터미널에서 실행한다.
#
# 키 배치 (정방향키/역방향키):
#   joint1  a/q   joint2  s/w   joint3  d/e
#   joint4  f/r   joint5  g/t   joint6  h/y
#
# 방향(direction)은 코드 상단 DIRECTION 상수로 고정 (기본값 1)
#
# ESC 또는 Ctrl+C: 종료
#
# 실행 예시 (왼쪽 다리):
#   uv run python leg_control.py \
#       --chan can2 \
#       --ids 1 3 5 7 11 13
#
# 실행 예시 (오른쪽 다리):
#   uv run python leg_control.py \
#       --chan can3 \
#       --ids 2 4 6 8 12 14

import argparse
import threading

from pynput import keyboard
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# ── 설정 ───────────────────────────────────────────────
KP           = 20.0
KD           = 2.0
TORQUE_LIMIT = 2.0
LOOP_HZ      = 200
MOVE_SPEED   = 1.0                   # rad/s
KEY_STEP     = MOVE_SPEED / LOOP_HZ  # = 0.005 rad/loop
DIRECTION    = 1                     # 전체 관절 방향 (1 또는 -1, 반대쪽 다리 미러링용)
# ───────────────────────────────────────────────────────

# (정방향키, 역방향키) — 6개 관절
KEYS = [('a', 'q'), ('s', 'w'), ('d', 'e'), ('f', 'r'), ('g', 't'), ('h', 'y')]


def get_args():
    parser = argparse.ArgumentParser(description="단일 다리 제어 (모터 6개, yaml 불필요)")
    parser.add_argument('--chan', default='can2')
    parser.add_argument('--ids',  type=int, nargs=6, required=True,
                        metavar=('ID1', 'ID2', 'ID3', 'ID4', 'ID5', 'ID6'))
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

    bus = recoil.Bus(channel=args.chan, bitrate=1000000)

    ids     = args.ids            # 6개
    targets = [0.0] * 6
    pressed_keys = set()
    quit_flag    = threading.Event()

    # 키맵 구성
    key_action = {}
    for i, (fwd, rev) in enumerate(KEYS):
        key_action[fwd] = (i, +1)
        key_action[rev] = (i, -1)

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
    print(f"  다리 ({args.chan})  direction={DIRECTION} :",
          ' '.join(f"{a}/{b}" for a, b in KEYS))
    print(f"  KP={KP}  KD={KD}  TORQUE_LIMIT={TORQUE_LIMIT}")
    print(f"  MOVE_SPEED={MOVE_SPEED} rad/s  |  KEY_STEP={KEY_STEP:.4f} rad/loop")
    print("  ESC 또는 Ctrl+C : 종료")
    print("=" * 65)

    print("\n초기화:")
    for dev in ids:
        init_motor(bus, dev, KP, KD, TORQUE_LIMIT)

    print("\n제어 시작\n")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    rate = RateLimiter(frequency=LOOP_HZ)

    try:
        while not quit_flag.is_set():

            # 눌린 키 처리 — 방향(direction) 적용
            for ch in list(pressed_keys):
                if ch in key_action:
                    idx, key_dir = key_action[ch]
                    step = KEY_STEP * key_dir * DIRECTION
                    targets[idx] += step

            # 모터 제어
            results = []
            for i, dev in enumerate(ids):
                pos, vel = bus.write_read_pdo_2(dev, targets[i], 0.0)
                results.append((dev, pos, vel))

            # 출력
            status = '  '.join(
                f"[{d}]{p:+.2f}" for d, p, v in results if p is not None)
            print(f"  {status}", end='\r')

            rate.sleep()

    except KeyboardInterrupt:
        pass

    finally:
        quit_flag.set()
        listener.stop()

        for dev in ids:
            bus.set_mode(dev, recoil.Mode.IDLE)

        bus.stop()
        print("\n\n종료.")


if __name__ == "__main__":
    main()
