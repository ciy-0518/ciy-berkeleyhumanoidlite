# 감속비 설정 스크립트
# 실행 예시:
#   uv run python set_gear_ratio.py -c can0 -i 9 -g -15.0

import argparse
import berkeley_humanoid_lite_lowlevel.recoil as recoil


def get_args():
    parser = argparse.ArgumentParser(description="모터 gear_ratio 설정")
    parser.add_argument('-c', '--channel', default='can0', help='CAN 채널')
    parser.add_argument('-i', '--id',      type=int,   required=True, help='모터 CAN ID')
    parser.add_argument('-g', '--gear',    type=float, required=True, help='감속비 (예: -15.0)')
    return parser.parse_args()


def main():
    args = get_args()

    bus = recoil.Bus(channel=args.channel, bitrate=1000000)

    before = bus.read_gear_ratio(args.id)
    bus.write_gear_ratio(args.id, args.gear)
    after  = bus.read_gear_ratio(args.id)

    print(f"device     : {args.id}")
    print(f"gear_ratio : {before} → {after}")
    print("※ 변경 후 반드시 재캘리브레이션 필요")

    bus.stop()


if __name__ == "__main__":
    main()
