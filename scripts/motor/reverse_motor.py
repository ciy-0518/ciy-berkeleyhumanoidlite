# 모터 상 방향 반전 스크립트
# 실행 예시:
#   uv run python reverse_motor.py -c can0 -i 9 -p -1

import berkeley_humanoid_lite_lowlevel.recoil as recoil
import argparse

parser = argparse.ArgumentParser(description="모터 phase_order 설정")
parser.add_argument('-c', '--channel', default='can0', help='CAN 채널')
parser.add_argument('-i', '--id',      type=int, required=True, help='모터 CAN ID')
parser.add_argument('-p', '--phase',   type=int, choices=[1, -1], required=True, help='상 방향 (1 또는 -1)')
args = parser.parse_args()

bus = recoil.Bus(channel=args.channel, bitrate=1000000)

before = bus.read_motor_phase_order(args.id)
bus.write_motor_phase_order(args.id, args.phase)
after  = bus.read_motor_phase_order(args.id)

print(f"device    : {args.id}")
print(f"phase_order: {before} → {after}")
print("※ 변경 후 반드시 재캘리브레이션 필요")

bus.stop()
