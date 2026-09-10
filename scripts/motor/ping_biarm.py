# 전체 모터 연결 상태 확인
# 실행: uv run python ping_all.py

import berkeley_humanoid_lite_lowlevel.recoil as recoil

LEFT_CHAN  = "can0"
LEFT_IDS  = [1, 3, 5, 7, 9]

RIGHT_CHAN = "can1"
RIGHT_IDS  = [2, 4, 6, 8, 10]

left_bus  = recoil.Bus(channel=LEFT_CHAN,  bitrate=1000000)
right_bus = recoil.Bus(channel=RIGHT_CHAN, bitrate=1000000)

print("=" * 40)
print(f"  왼팔 ({LEFT_CHAN})")
print("=" * 40)
for dev_id in LEFT_IDS:
    status = left_bus.ping(dev_id)
    mark = "✅" if status else "❌"
    print(f"  {mark} 모터 {dev_id:2d} : {'online' if status else 'offline'}")

print()
print("=" * 40)
print(f"  오른팔 ({RIGHT_CHAN})")
print("=" * 40)
for dev_id in RIGHT_IDS:
    status = right_bus.ping(dev_id)
    mark = "✅" if status else "❌"
    print(f"  {mark} 모터 {dev_id:2d} : {'online' if status else 'offline'}")

left_bus.stop()
right_bus.stop()
print()
