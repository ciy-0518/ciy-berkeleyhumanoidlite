# 5-Motor Control Script
#
# 모터 1 (--id1): a=정방향  q=역방향
# 모터 2 (--id2): s=정방향  w=역방향
# 모터 3 (--id3): d=정방향  e=역방향
# 모터 4 (--id4): f=정방향  r=역방향
# 모터 5 (--id5): g=정방향  t=역방향
# Ctrl+C: 종료
#
# 실행 예시:
#   uv run python five_motor_control.py -c can0 \
#       --id1 11 --id2 12 --id3 13 --id4 14 --id5 15

import sys
import tty
import termios
import threading
import argparse

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# ── 설정 ───────────────────────────────────────────────
KP           = 20
KD           = 2
TORQUE_LIMIT = 2.0
STEP         = 0.2
MAX_POS      = 6.28
MIN_POS      = -6.28
LOOP_HZ      = 200
# ───────────────────────────────────────────────────────

KEY_MAP = {
    1: ('a', 'q'),
    2: ('s', 'w'),
    3: ('d', 'e'),
    4: ('f', 'r'),
    5: ('g', 't'),
}


def get_args():
    parser = argparse.ArgumentParser(description="5-motor keyboard control")
    parser.add_argument('-c', '--channel', default='can0')
    parser.add_argument('--id1', type=int, required=True, help='모터 1 CAN ID (a/q)')
    parser.add_argument('--id2', type=int, required=True, help='모터 2 CAN ID (s/w)')
    parser.add_argument('--id3', type=int, required=True, help='모터 3 CAN ID (d/e)')
    parser.add_argument('--id4', type=int, required=True, help='모터 4 CAN ID (f/r)')
    parser.add_argument('--id5', type=int, required=True, help='모터 5 CAN ID (g/t)')
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


def keyboard_thread(state, dev_ids):
    key_action = {}
    for idx, dev_id in enumerate(dev_ids, start=1):
        fwd_key, rev_key = KEY_MAP[idx]
        key_action[fwd_key] = (idx, +1)
        key_action[rev_key] = (idx, -1)

    while not state['quit']:
        key = getch_blocking()

        if key == '\x03':
            state['quit'] = True
            break

        if key in key_action:
            idx, direction = key_action[key]
            t_key = f'target_{idx}'
            dev_id = dev_ids[idx - 1]

            if direction == +1:
                state[t_key] = min(state[t_key] + STEP, MAX_POS)
                label = "정방향"
            else:
                state[t_key] = max(state[t_key] - STEP, MIN_POS)
                label = "역방향"

            print(f"\n[모터{dev_id} {label}] target → {state[t_key]:+.3f} rad")


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
    print(f"  [모터 {dev:2d}] 초기화 완료 — 현재 위치를 0으로 설정")


def main():
    args    = get_args()
    dev_ids = [args.id1, args.id2, args.id3, args.id4, args.id5]

    bus = recoil.Bus(channel=args.channel, bitrate=1000000)

    print("=" * 60)
    print(f"  채널 : {args.channel}")
    for idx, dev_id in enumerate(dev_ids, start=1):
        fwd, rev = KEY_MAP[idx]
        print(f"  모터 {dev_id:2d} : {fwd}=정방향  {rev}=역방향")
    print(f"  KP={KP}  KD={KD}  TORQUE_LIMIT={TORQUE_LIMIT}")
    print(f"  STEP={STEP} rad  |  범위 [{MIN_POS}, {MAX_POS}] rad")
    print("  Ctrl+C : 종료")
    print("=" * 60)

    for dev_id in dev_ids:
        init_motor(bus, dev_id)

    state = {f'target_{i}': 0.0 for i in range(1, 6)}
    state['quit'] = False

    t = threading.Thread(target=keyboard_thread, args=(state, dev_ids), daemon=True)
    t.start()

    rate = RateLimiter(frequency=LOOP_HZ)

    try:
        while not state['quit']:
            results = []
            for idx, dev_id in enumerate(dev_ids, start=1):
                pos, vel = bus.write_read_pdo_2(dev_id, state[f'target_{idx}'], 0.0)
                results.append((dev_id, pos, vel))

            if all(r[1] is not None for r in results):
                line = "  " + "  |  ".join(
                    f"[{dev_id}] {pos:+.2f}rad {vel:+.1f}rad/s"
                    for dev_id, pos, vel in results
                )
                print(line, end='\r')

            rate.sleep()

    except KeyboardInterrupt:
        pass

    finally:
        state['quit'] = True
        for dev_id in dev_ids:
            bus.set_mode(dev_id, recoil.Mode.IDLE)
        bus.stop()
        print("\n\n종료.")


if __name__ == "__main__":
    main()
