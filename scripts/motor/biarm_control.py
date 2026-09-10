# 양팔 동시 제어 스크립트 (yaml 설정 적용)
# 왼팔 (can0): a/q  s/w  d/e  f/r  g/t  (정/역)
# 오른팔(can1): h/y  j/u  k/i  l/o  ;/p  (정/역)
# ESC 또는 Ctrl+C: 종료
#
# 실행 예시:
#   uv run python biarm_control.py \
#       --left_ids  1 3 5 7 9 \
#       --right_ids 2 4 6 8 10 \
#       --config arm_config.yaml

import argparse
import threading

import yaml
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
# ───────────────────────────────────────────────────────

LEFT_KEYS  = [('a','q'), ('s','w'), ('d','e'), ('f','r'), ('g','t')]
RIGHT_KEYS = [('h','y'), ('j','u'), ('k','i'), ('l','o'), (';','p')]


def get_args():
    parser = argparse.ArgumentParser(description="양팔 동시 제어")
    parser.add_argument('--left_chan',  default='can0')
    parser.add_argument('--right_chan', default='can1')
    parser.add_argument('--left_ids',  type=int, nargs=5, required=True,
                        metavar=('ID1','ID2','ID3','ID4','ID5'))
    parser.add_argument('--right_ids', type=int, nargs=5, required=True,
                        metavar=('ID1','ID2','ID3','ID4','ID5'))
    parser.add_argument('--config',    default='arm_config.yaml',
                        help='관절 설정 yaml 파일 경로')
    return parser.parse_args()


def load_config(path):
    """yaml에서 방향(direction)과 리미트(limit) 로드"""
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)

    result = {}
    for arm_key in ('left_arm', 'right_arm'):
        arm = cfg[arm_key]
        direction = arm.get('direction', 1)
        for joint in arm['joints']:
            result[joint['id']] = {
                'direction':   direction,
                'limit_upper': joint['limit_upper'],
                'limit_lower': joint['limit_lower'],
            }
    return result


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
    args    = get_args()
    cfg     = load_config(args.config)

    left_bus  = recoil.Bus(channel=args.left_chan,  bitrate=1000000)
    right_bus = recoil.Bus(channel=args.right_chan, bitrate=1000000)

    all_ids   = args.left_ids + args.right_ids   # 총 10개
    targets   = [0.0] * 10
    pressed_keys = set()
    quit_flag    = threading.Event()

    # 키맵 구성 (인덱스 0~4: 왼팔, 5~9: 오른팔)
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
    print(f"  MOVE_SPEED={MOVE_SPEED} rad/s  |  KEY_STEP={KEY_STEP:.4f} rad/loop")
    print(f"  config: {args.config}")
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

            # 눌린 키 처리 — 방향 반전 + 리미트 클램핑 적용
            for ch in list(pressed_keys):
                if ch in key_action:
                    idx, direction = key_action[ch]
                    dev_id = all_ids[idx]
                    joint  = cfg.get(dev_id, {})

                    # yaml direction 적용 (왼팔 -1이면 방향 반전)
                    axis_dir = joint.get('direction', 1)
                    step     = KEY_STEP * direction * axis_dir

                    new_target = targets[idx] + step

                    # 리미트 클램핑
                    upper = joint.get('limit_upper',  99.0)
                    lower = joint.get('limit_lower', -99.0)
                    targets[idx] = max(lower, min(upper, new_target))

            # 왼팔 제어
            left_results = []
            for i, dev in enumerate(args.left_ids):
                pos, vel = left_bus.write_read_pdo_2(dev, targets[i], 0.0)
                left_results.append((dev, pos, vel))

            # 오른팔 제어
            right_results = []
            for i, dev in enumerate(args.right_ids):
                pos, vel = right_bus.write_read_pdo_2(dev, targets[5 + i], 0.0)
                right_results.append((dev, pos, vel))

            # 출력
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
