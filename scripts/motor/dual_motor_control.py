# Dual Motor Control Script
#
# 모터 1 (--id1): a=정방향, q=역방향
# 모터 2 (--id2): s=정방향, w=역방향
# Ctrl+C: 종료
#
# 실행 예시:
#   uv run python dual_motor_control.py -c can0 --id1 14 --id2 11

import sys
import tty
import termios
import threading
import argparse

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# ── 설정 ───────────────────────────────────────────────
KP           = 0.2
KD           = 0.005
TORQUE_LIMIT = 1.0
STEP         = 0.2    # 한 번 누를 때 이동량 (rad)
MAX_POS      = 6.28
MIN_POS      = -6.28
LOOP_HZ      = 200
# ───────────────────────────────────────────────────────


def get_args():
    parser = argparse.ArgumentParser(description="Dual motor keyboard control")
    parser.add_argument('-c', '--channel', default='can0',  help='CAN 채널 (기본: can0)')
    parser.add_argument('--id1', type=int, required=True,   help='모터 1 CAN ID (a/q 제어)')
    parser.add_argument('--id2', type=int, required=True,   help='모터 2 CAN ID (s/w 제어)')
    return parser.parse_args()


def getch_blocking():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def keyboard_thread(state, dev1, dev2):
    while not state['quit']:
        key = getch_blocking()

        if key == '\x03':  # Ctrl+C
            state['quit'] = True
            break

        elif key == 'a':
            state['target_1'] = min(state['target_1'] + STEP, MAX_POS)
            print(f"\n[모터{dev1} 정방향] target → {state['target_1']:+.3f} rad")

        elif key == 'q':
            state['target_1'] = max(state['target_1'] - STEP, MIN_POS)
            print(f"\n[모터{dev1} 역방향] target → {state['target_1']:+.3f} rad")

        elif key == 's':
            state['target_2'] = min(state['target_2'] + STEP, MAX_POS)
            print(f"\n[모터{dev2} 정방향] target → {state['target_2']:+.3f} rad")

        elif key == 'w':
            state['target_2'] = max(state['target_2'] - STEP, MIN_POS)
            print(f"\n[모터{dev2} 역방향] target → {state['target_2']:+.3f} rad")


def init_motor(bus, dev):
    bus.write_position_kp(dev, KP)
    bus.write_position_kd(dev, KD)
    bus.write_torque_limit(dev, TORQUE_LIMIT)

    cur = bus._read_parameter_f32(
        dev,
        recoil.Parameter.POSITION_CONTROLLER_POSITION_MEASURED,
        timeout=0.5
    )
    if cur is not None:
        bus.write_position_offset(dev, cur)

    bus.set_mode(dev, recoil.Mode.POSITION)
    bus.feed(dev)
    print(f"  [모터 {dev}] 초기화 완료 — 현재 위치를 0으로 설정")


def main():
    args = get_args()
    dev1 = args.id1
    dev2 = args.id2

    bus = recoil.Bus(channel=args.channel, bitrate=1000000)

    print("=" * 55)
    print(f"  채널  : {args.channel}")
    print(f"  모터 {dev1} : a=정방향  q=역방향")
    print(f"  모터 {dev2} : s=정방향  w=역방향")
    print(f"  KP={KP}  KD={KD}  TORQUE_LIMIT={TORQUE_LIMIT}")
    print(f"  STEP={STEP} rad  |  범위 [{MIN_POS}, {MAX_POS}] rad")
    print("  Ctrl+C : 종료")
    print("=" * 55)

    init_motor(bus, dev1)
    init_motor(bus, dev2)

    state = {
        'target_1': 0.0,
        'target_2': 0.0,
        'quit':     False,
    }

    t = threading.Thread(target=keyboard_thread, args=(state, dev1, dev2), daemon=True)
    t.start()

    rate = RateLimiter(frequency=LOOP_HZ)

    try:
        while not state['quit']:
            pos1, vel1 = bus.write_read_pdo_2(dev1, state['target_1'], 0.0)
            pos2, vel2 = bus.write_read_pdo_2(dev2, state['target_2'], 0.0)

            if pos1 is not None and pos2 is not None:
                print(
                    f"  [{dev1}] pos:{pos1:+.3f} vel:{vel1:+.3f}"
                    f"  |  [{dev2}] pos:{pos2:+.3f} vel:{vel2:+.3f}"
                    f"  target:[{state['target_1']:+.3f}, {state['target_2']:+.3f}]",
                    end='\r'
                )

            rate.sleep()

    except KeyboardInterrupt:
        pass

    finally:
        state['quit'] = True
        bus.set_mode(dev1, recoil.Mode.IDLE)
        bus.set_mode(dev2, recoil.Mode.IDLE)
        bus.stop()
        print("\n\n종료.")


if __name__ == "__main__":
    main()
