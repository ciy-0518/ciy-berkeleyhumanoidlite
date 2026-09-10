# Motor control script
# a: 활성/비활성 토글
# q: 정방향 회전 (position +step)
# w: 역방향 회전 (position -step)
# Ctrl+C: 종료

import sys
import tty
import termios
import threading

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# ── 설정 ───────────────────────────────────────────────
KP           = 0.2
KD           = 0.005
TORQUE_LIMIT = 1.0
STEP         = 0.2    # q/w 한 번 누를 때 이동량 (rad)
MAX_POS      = 23.0   # 위치 상한 (rad)
MIN_POS      = -23.0  # 위치 하한 (rad)
LOOP_HZ      = 200
# ───────────────────────────────────────────────────────


def getch_blocking():
    """블로킹 1문자 입력 (별도 스레드용)"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def keyboard_thread(state):
    """키 입력을 별도 스레드에서 처리 — 제어 루프를 막지 않음"""
    while not state['quit']:
        key = getch_blocking()

        if key == '\x03':  # Ctrl+C
            state['quit'] = True
            break

        elif key == 'a':
            state['toggle'] = True  # 메인 루프에서 CAN 처리

        elif key == 'q':
            if state['active']:
                state['target'] = min(state['target'] + STEP, MAX_POS)
                print(f"\n[정방향] target → {state['target']:+.3f} rad")
            else:
                print("\n[비활성 상태] 먼저 'a'로 활성화하세요")

        elif key == 'w':
            if state['active']:
                state['target'] = max(state['target'] - STEP, MIN_POS)
                print(f"\n[역방향] target → {state['target']:+.3f} rad")
            else:
                print("\n[비활성 상태] 먼저 'a'로 활성화하세요")


def main():
    args = recoil.util.get_args()
    bus  = recoil.Bus(channel=args.channel, bitrate=1000000)
    dev  = args.id

    bus.write_position_kp(dev, KP)
    bus.write_position_kd(dev, KD)
    bus.write_torque_limit(dev, TORQUE_LIMIT)
    bus.set_mode(dev, recoil.Mode.IDLE)

    state = {
        'active': False,
        'target': 0.0,
        'toggle': False,
        'quit':   False,
    }

    # 키보드 스레드 시작
    t = threading.Thread(target=keyboard_thread, args=(state,), daemon=True)
    t.start()

    print("=" * 45)
    print(f"  Device ID : {dev}")
    print(f"  KP={KP}  KD={KD}  TORQUE_LIMIT={TORQUE_LIMIT}")
    print(f"  STEP={STEP} rad  |  범위 [{MIN_POS}, {MAX_POS}] rad")
    print("=" * 45)
    print("  a       : 활성 / 비활성 토글")
    print(f"  q       : 정방향 +{STEP} rad")
    print(f"  w       : 역방향 -{STEP} rad")
    print("  Ctrl+C  : 종료")
    print("=" * 45)
    print("상태: 비활성\n")

    rate = RateLimiter(frequency=LOOP_HZ)

    try:
        while not state['quit']:

            # 활성/비활성 토글 처리 (CAN은 메인 루프에서만)
            if state['toggle']:
                state['toggle'] = False
                state['active'] = not state['active']

                if state['active']:
                    cur = bus._read_parameter_f32(
                        dev,
                        recoil.Parameter.POSITION_CONTROLLER_POSITION_MEASURED,
                        timeout=0.5
                    )
                    if cur is not None:
                        bus.write_position_offset(dev, cur)
                    state['target'] = 0.0
                    bus.set_mode(dev, recoil.Mode.POSITION)
                    bus.feed(dev)
                    print(f"\n[활성화] 현재 위치를 0으로 설정")
                else:
                    bus.set_mode(dev, recoil.Mode.IDLE)
                    print(f"\n[비활성화] IDLE 모드")

            # 제어 루프
            if state['active']:
                measured_pos, measured_vel = bus.write_read_pdo_2(
                    dev, state['target'], 0.0
                )
                if measured_pos is not None and measured_vel is not None:
                    print(
                        f"  pos: {measured_pos:+.3f} rad"
                        f"  vel: {measured_vel:+.3f} rad/s"
                        f"  target: {state['target']:+.3f} rad",
                        end='\r'
                    )
            else:
                bus.feed(dev)

            rate.sleep()

    except KeyboardInterrupt:
        pass

    finally:
        state['quit'] = True
        bus.set_mode(dev, recoil.Mode.IDLE)
        bus.stop()
        print("\n\n종료.")


if __name__ == "__main__":
    main()
