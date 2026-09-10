# 양다리 동시 제어 스크립트 (yaml 설정 적용)
# 다리 1개당 6개 관절, 총 12개 모터
#
# 키보드 배치 (대각선 컬럼 방식):
#   왼쪽 다리(can0) : 숫자행(정/역 아님, 역) + 상단행(정)
#     joint1  q/1   joint2  w/2   joint3  e/3
#     joint4  r/4   joint5  t/5   joint6  y/6
#   오른쪽 다리(can1): 홈행(역) + 하단행(정)
#     joint1  z/a   joint2  x/s   joint3  c/d
#     joint4  v/f   joint5  b/g   joint6  n/h
#   (표기는 "정방향키/역방향키" 순서)
#
# ESC 또는 Ctrl+C: 종료
#
# 실행 예시:
#   uv run python biped_leg_control.py \
#       --left_chan  can2 --right_chan can3 \
#       --left_ids  1 3 5 7 11 13 \
#       --right_ids 2 4 6 8 12 14 \
#       --config leg_config.yaml

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

# (정방향키, 역방향키) — 대각선 컬럼: 숫자행/상단행 = 왼쪽 다리, 홈행/하단행 = 오른쪽 다리
LEFT_KEYS  = [('q', '1'), ('w', '2'), ('e', '3'), ('r', '4'), ('t', '5'), ('y', '6')]
RIGHT_KEYS = [('z', 'a'), ('x', 's'), ('c', 'd'), ('v', 'f'), ('b', 'g'), ('n', 'h')]


def get_args():
    parser = argparse.ArgumentParser(description="양다리 동시 제어")
    parser.add_argument('--left_chan',  default='can2')
    parser.add_argument('--right_chan', default='can3')
    parser.add_argument('--left_ids',  type=int, nargs=6, required=True,
                        metavar=('ID1', 'ID2', 'ID3', 'ID4', 'ID5', 'ID6'))
    parser.add_argument('--right_ids', type=int, nargs=6, required=True,
                        metavar=('ID1', 'ID2', 'ID3', 'ID4', 'ID5', 'ID6'))
    parser.add_argument('--config',    default='leg_config.yaml',
                        help='관절 설정 yaml 파일 경로')
    return parser.parse_args()


def load_config(path):
    """yaml에서 방향(direction)과 리미트(limit) 로드"""
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)

    result = {}
    for leg_key in ('left_leg', 'right_leg'):
        leg = cfg[leg_key]
        direction = leg.get('direction', 1)
        for joint in leg['joints']:
            result[joint['id']] = {
                'direction':   direction,
                'limit_upper': joint.get('limit_upper',  99.0),
                'limit_lower': joint.get('limit_lower', -99.0),
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

    all_ids   = args.left_ids + args.right_ids   # 총 12개
    targets   = [0.0] * 12
    pressed_keys = set()
    quit_flag    = threading.Event()

    # 키맵 구성 (인덱스 0~5: 왼쪽 다리, 6~11: 오른쪽 다리)
    key_action = {}
    for i, (fwd, rev) in enumerate(LEFT_KEYS):
        key_action[fwd] = (i,     +1)
        key_action[rev] = (i,     -1)
    for i, (fwd, rev) in enumerate(RIGHT_KEYS):
        key_action[fwd] = (6 + i, +1)
        key_action[rev] = (6 + i, -1)

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
    print(f"  왼쪽 다리 ({args.left_chan}) :", ' '.join(f"{a}/{b}" for a, b in LEFT_KEYS))
    print(f"  오른쪽다리({args.right_chan}) :", ' '.join(f"{a}/{b}" for a, b in RIGHT_KEYS))
    print(f"  KP={KP}  KD={KD}  TORQUE_LIMIT={TORQUE_LIMIT}")
    print(f"  MOVE_SPEED={MOVE_SPEED} rad/s  |  KEY_STEP={KEY_STEP:.4f} rad/loop")
    print(f"  config: {args.config}")
    print("  ESC 또는 Ctrl+C : 종료")
    print("=" * 65)

    print("\n왼쪽 다리 초기화:")
    for dev in args.left_ids:
        init_motor(left_bus, dev, KP, KD, TORQUE_LIMIT)

    print("\n오른쪽 다리 초기화:")
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

                    # yaml direction 적용 (다리 -1이면 방향 반전)
                    axis_dir = joint.get('direction', 1)
                    step     = KEY_STEP * direction * axis_dir

                    new_target = targets[idx] + step

                    # 리미트 클램핑
                    upper = joint.get('limit_upper',  99.0)
                    lower = joint.get('limit_lower', -99.0)
                    targets[idx] = max(lower, min(upper, new_target))

            # 왼쪽 다리 제어
            left_results = []
            for i, dev in enumerate(args.left_ids):
                pos, vel = left_bus.write_read_pdo_2(dev, targets[i], 0.0)
                left_results.append((dev, pos, vel))

            # 오른쪽 다리 제어
            right_results = []
            for i, dev in enumerate(args.right_ids):
                pos, vel = right_bus.write_read_pdo_2(dev, targets[6 + i], 0.0)
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
