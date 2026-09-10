# 엔코더 CPR 부호 반전 스크립트
# 실행 예시:
#   uv run python reverse_encoder.py -c can0 -i 9

import berkeley_humanoid_lite_lowlevel.recoil as recoil
import argparse

parser = argparse.ArgumentParser(description="엔코더 CPR 부호 반전")
parser.add_argument('-c', '--channel', default='can0', help='CAN 채널')
parser.add_argument('-i', '--id',      type=int, required=True, help='모터 CAN ID')
args = parser.parse_args()

bus = recoil.Bus(channel=args.channel, bitrate=1000000)

cpr = bus.read_encoder_cpr(args.id)
print(f"device    : {args.id}")
print(f"현재 cpr  : {cpr}")

try:
    bus.write_encoder_cpr(args.id, -cpr)
    after = bus.read_encoder_cpr(args.id)
    print(f"변경 후 cpr: {after}")
    print("※ 변경 후 반드시 재캘리브레이션 필요")
except AssertionError:
    print("ERROR: write_encoder_cpr이 unsigned 제약으로 막혀 있습니다.")
    print("       ENCODER_DIRECTION은 펌웨어 수정으로만 변경 가능합니다.")

bus.stop()
