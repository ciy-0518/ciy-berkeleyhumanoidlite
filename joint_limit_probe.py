#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
관절 실측 한계값(URDF_LIMITS) 재측정용 도구.

모터를 IDLE(손으로 자유롭게 움직일 수 있는, 저항 없는 상태)로 두고, 매 순간의
측정 위치를 계속 읽어서 각 관절의 최소/최대값을 실시간으로 추적한다.
손으로 각 관절을 안전한 범위 끝까지 천천히 움직여보면 되고, 순서는
아무렇게나 해도 된다(10개 관절을 동시에 계속 추적하고 있음).

(IDLE이 맞는 모드임을 실측으로 확인함: DAMPING은 예상과 달리 뻣뻣했고,
IDLE에서는 제어 명령이 없을 때처럼 가볍게 움직임 -- Mode enum 주석에서도
DISABLED/IDLE만 "safe modes"로 따로 분류되어 있었던 것과 일치.)

주의: 그래도 로봇 자체 무게로 인해 갑자기 축 처질 수 있으니 처음 모드
전환 시에는 팔을 받치고 있는 상태에서 시작할 것.

사용법:
    python3 joint_limit_probe.py
    (Ctrl+C로 종료하면 최종 결과와 DEFAULT_URDF_LIMITS용 코드 블록을 출력)

옵션:
    --margin 0.05          # 실측값에서 안전 여유를 얼마나 뺄지 (기본 5%, 문서에 있던 관행과 동일)
    --left-channel can0 --right-channel can1
    --left-ids 1,3,5,7,9 --right-ids 2,4,6,8,10
"""

import argparse
import csv
import time

try:
    import berkeley_humanoid_lite_lowlevel.recoil as recoil
except ImportError:
    recoil = None


JOINT_ORDER = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "elbow_roll")


def parse_ids(value):
    result = tuple(int(x.strip()) for x in value.split(","))
    if len(result) != 5:
        raise argparse.ArgumentTypeError("5개 ID가 필요합니다 (shoulder_pitch,roll,yaw,elbow_pitch,elbow_roll)")
    return result


def parse_args():
    p = argparse.ArgumentParser(description="관절 실측 한계값(min/max) 실시간 추적 도구")
    p.add_argument("--left-channel", default="can0")
    p.add_argument("--right-channel", default="can1")
    p.add_argument("--left-ids", type=parse_ids, default=(1, 3, 5, 7, 9))
    p.add_argument("--right-ids", type=parse_ids, default=(2, 4, 6, 8, 10))
    p.add_argument("--margin", type=float, default=0.05,
                   help="실측 범위에서 양쪽으로 뺄 안전 여유 비율 (기본 0.05 = 5%%)")
    p.add_argument("--log-csv", default=None, help="매 샘플을 다 기록하고 싶으면 CSV 경로 지정 (선택)")
    return p.parse_args()


def main():
    args = parse_args()
    if recoil is None:
        print("[에러] recoil 모듈을 import할 수 없습니다. 로봇 PC에서 실행해주세요.")
        return

    motors = []  # (side, joint, device_id, bus)
    channels = {"left": args.left_channel, "right": args.right_channel}
    ids = {"left": args.left_ids, "right": args.right_ids}
    buses = {}
    for side in ("left", "right"):
        channel = channels[side]
        if channel not in buses:
            buses[channel] = recoil.Bus(channel=channel, bitrate=1_000_000)
        bus = buses[channel]
        for joint, device_id in zip(JOINT_ORDER, ids[side]):
            motors.append((side, joint, device_id, bus))

    print("초기화 중 (position_offset 정리 포함)...")
    for side, joint, device_id, bus in motors:
        bus.set_mode(device_id, recoil.Mode.IDLE)
        time.sleep(0.002)
        offset = bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_OFFSET, timeout=0.5)
        if offset is not None and abs(offset) > 1e-3:
            print(f"  [{side[0].upper()} {joint:14s}] motor {device_id:2d} offset={offset:.4f} -> 0")
            bus.write_position_offset(device_id, 0.0)
            bus.store_settings_to_flash(device_id)
            time.sleep(0.01)
        bus.set_mode(device_id, recoil.Mode.IDLE)
        print(f"  [{side[0].upper()} {joint:14s}] motor {device_id:2d} -> IDLE (손으로 자유롭게 움직여도 됨)")

    print("\n각 관절을 손으로 안전한 범위 끝까지 천천히 움직여보세요 (순서 무관, 양쪽 방향 다).")
    print("끝나면 Ctrl+C.\n")

    running_min = {(s, j): None for s, j, _, _ in motors}
    running_max = {(s, j): None for s, j, _, _ in motors}

    log_writer = None
    log_file = None
    if args.log_csv:
        log_file = open(args.log_csv, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow(["t"] + [f"{s}_{j}" for s, j, _, _ in motors])

    t0 = time.time()
    try:
        while True:
            row = [f"{time.time()-t0:.2f}"]
            lines = []
            for side, joint, device_id, bus in motors:
                pos = bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_MEASURED, timeout=0.2)
                key = (side, joint)
                if pos is not None:
                    if running_min[key] is None or pos < running_min[key]:
                        running_min[key] = pos
                    if running_max[key] is None or pos > running_max[key]:
                        running_max[key] = pos
                    row.append(f"{pos:.4f}")
                else:
                    row.append("")
                lo = running_min[key]
                hi = running_max[key]
                cur = f"{pos:+.3f}" if pos is not None else "  ??? "
                lo_s = f"{lo:+.3f}" if lo is not None else "  ??? "
                hi_s = f"{hi:+.3f}" if hi is not None else "  ??? "
                lines.append(f"[{side[0].upper()} {joint:14s}] 현재={cur}  min={lo_s}  max={hi_s}")

            if log_writer is not None:
                log_writer.writerow(row)
                log_file.flush()

            print("\n".join(lines))
            print(f"\033[{len(lines)}A", end="")  # 커서를 위로 올려서 덮어쓰기
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n" * (len(motors) + 1))
        print("=" * 70)
        print("측정 종료. 결과:\n")

        limits = {"left": {}, "right": {}}
        for side, joint, device_id, bus in motors:
            key = (side, joint)
            lo, hi = running_min[key], running_max[key]
            if lo is None or hi is None:
                print(f"  [{side[0].upper()} {joint:14s}] 데이터 없음 (안 움직여봄)")
                continue
            span = hi - lo
            margin = span * args.margin
            safe_lo, safe_hi = lo + margin, hi - margin
            limits[side][joint] = (safe_lo, safe_hi)
            print(f"  [{side[0].upper()} {joint:14s}] 실측=[{lo:+.4f}, {hi:+.4f}]  "
                  f"-> 여유{args.margin*100:.0f}% 적용=[{safe_lo:+.4f}, {safe_hi:+.4f}]")
            bus.set_mode(device_id, recoil.Mode.IDLE)

        print("\n--- DEFAULT_URDF_LIMITS에 붙여넣을 코드 ---\n")
        print("DEFAULT_URDF_LIMITS = {")
        for side in ("left", "right"):
            print(f'    "{side}": {{')
            for joint in JOINT_ORDER:
                if joint in limits[side]:
                    lo, hi = limits[side][joint]
                    print(f'        "{joint}": ({lo:.4f}, {hi:.4f}),')
                else:
                    print(f'        "{joint}": (-10.0, 10.0),  # 측정 안 됨, 기존값 유지')
            print("    },")
        print("}")

        if log_file is not None:
            log_file.close()
            print(f"\n전체 샘플 로그 저장됨: {args.log_csv}")


if __name__ == "__main__":
    main()
