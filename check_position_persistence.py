#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터 자체 encoder 기준점("0")이 전원 재시작/CAN 재연결 후에도 유지되는지
확인하는 도구.

사용법 (팔을 전혀 안 움직인 채로):
    1) python3 check_position_persistence.py --save before.json
    2) 모터 컨트롤러 전원을 껐다 켠다 (또는 최소한 이 스크립트를 종료하고
       CAN 인터페이스를 내렸다 올린다: sudo ip link set can0 down && sudo ip link set can0 up
       can1도 동일)
    3) python3 check_position_persistence.py --compare before.json

주의: 이 확인 동안 팔을 절대 움직이면 안 됨 -- 움직이면 값이 달라지는 게
"0점이 안 맞아서"인지 "그냥 팔이 움직여서"인지 구분이 안 됨.
"""

import argparse
import json
import time

try:
    import berkeley_humanoid_lite_lowlevel.recoil as recoil
except ImportError:
    recoil = None


JOINT_ORDER = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "elbow_roll")
DEFAULT_IDS = {"left": (1, 3, 5, 7, 9), "right": (2, 4, 6, 8, 10)}
DEFAULT_CHANNELS = {"left": "can0", "right": "can1"}


def read_all():
    buses = {}
    result = {}
    for side, channel in DEFAULT_CHANNELS.items():
        if channel not in buses:
            buses[channel] = recoil.Bus(channel=channel, bitrate=1_000_000)
        bus = buses[channel]
        for joint, device_id in zip(JOINT_ORDER, DEFAULT_IDS[side]):
            pos = bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_MEASURED, timeout=0.5)
            result[f"{side}_{joint}"] = pos
    for bus in buses.values():
        bus.stop()
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--save", default=None, help="지금 위치를 이 파일에 저장")
    parser.add_argument("--compare", default=None, help="저장된 파일과 지금 위치를 비교")
    parser.add_argument("--tolerance", type=float, default=0.02, help="이 이상 차이나면 경고 (rad, 기본 0.02≈1.1도)")
    args = parser.parse_args()

    if recoil is None:
        print("[에러] recoil 모듈을 import할 수 없습니다. 로봇 PC에서 실행해주세요.")
        return

    print("측정 중 (팔을 움직이지 마세요)...")
    current = read_all()
    for key, val in current.items():
        print(f"  {key:22s} {val:+.4f}" if val is not None else f"  {key:22s} 읽기 실패")

    if args.save:
        with open(args.save, "w") as f:
            json.dump({"t": time.time(), "positions": current}, f, indent=2)
        print(f"\n저장됨: {args.save}")
        print("이제 모터 전원을 껐다 켜거나 CAN을 재연결한 뒤, --compare로 다시 실행하세요.")

    if args.compare:
        with open(args.compare) as f:
            saved = json.load(f)
        print(f"\n비교 대상: {args.compare} (측정 시각: {time.ctime(saved['t'])})\n")
        print(f"{'관절':22s} {'이전':>10s} {'지금':>10s} {'차이':>10s}")
        worst = 0.0
        any_fail = False
        for key in current:
            before = saved["positions"].get(key)
            after = current[key]
            if before is None or after is None:
                print(f"  {key:22s} 읽기 실패로 비교 불가")
                any_fail = True
                continue
            diff = abs(after - before)
            worst = max(worst, diff)
            flag = "  <-- 차이 큼!" if diff > args.tolerance else ""
            print(f"  {key:22s} {before:+10.4f} {after:+10.4f} {diff:10.4f}{flag}")

        print()
        if any_fail:
            print("[결과 불명] 일부 관절 읽기 실패 -- 재시도 필요")
        elif worst <= args.tolerance:
            print(f"[결과] 최대 차이 {worst:.4f}rad -- 허용오차({args.tolerance}) 이내. 0점 유지되는 것으로 보임.")
        else:
            print(f"[결과] 최대 차이 {worst:.4f}rad -- 허용오차({args.tolerance}) 초과! 0점이 전원 재시작마다 바뀔 수 있음.")
            print("       이 경우 오늘 잰 URDF_LIMITS를 그대로 재사용하면 안 되고,")
            print("       매 세션 시작 시 다시 홈잉/영점조정하는 절차가 필요합니다.")


if __name__ == "__main__":
    main()
