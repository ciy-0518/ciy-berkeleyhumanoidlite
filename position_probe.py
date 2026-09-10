#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
순수 진단용: 모터/CAN 전혀 안 씀. VR의 pos(위치, cm 추정) 필드를 받아서
    1) 언리얼 좌표축이 실제로 어느 방향(앞/옆/위)을 의미하는지
    2) 사람 팔 길이(어깨-손목 거리)
    3) 팔꿈치 위치(L/R_LowerArm) 트래킹 노이즈 수준
을 한 번에 확인하기 위한 스크립트.

방법 B(IK 기반 리타게팅)에 필요한 "좌표축 확인 테스트".

사용법:
    python3 position_probe.py

진행 방식:
    화면에 안내되는 순서대로 자세를 취하고 스페이스바를 누르면,
    3초 카운트다운 후 약 1초간 샘플을 모아 평균+표준편차를 계산해서
    화면에 출력하고 CSV에 한 줄 기록합니다.

    자세 순서 (총 7개, 양팔이 있어도 한쪽씩 순서대로):
      1. 중립 (양팔 편하게 내리기)          -- 기준점
      2. 오른팔 옆으로 수평 뻗기
      3. 오른팔 위로 (머리 위, 최대한)
      4. 오른팔 앞으로 수평 뻗기
      5. 왼팔 옆으로 수평 뻗기
      6. 왼팔 위로 (머리 위, 최대한)
      7. 왼팔 앞으로 수평 뻗기

    Ctrl+C: 언제든 종료 (그때까지 캡처된 자세는 CSV에 이미 저장되어 있음)
"""

import csv
import json
import socket
import statistics
import sys
import termios
import threading
import time
import tty

import numpy as np


JOINT_NAMES = [
    "Head",
    "L_UpperArm", "L_LowerArm", "L_WristTwist",
    "R_UpperArm", "R_LowerArm", "R_WristTwist",
]

POSES = [
    ("1_neutral", "중립 (양팔 편하게 내리기) -- 기준점"),
    ("2_right_side", "오른팔 옆으로 수평 뻗기"),
    ("3_right_up", "오른팔 위로 (머리 위, 최대한)"),
    ("4_right_forward", "오른팔 앞으로 수평 뻗기"),
    ("5_left_side", "왼팔 옆으로 수평 뻗기"),
    ("6_left_up", "왼팔 위로 (머리 위, 최대한)"),
    ("7_left_forward", "왼팔 앞으로 수평 뻗기"),
]

COUNTDOWN_SEC = 3.0
CAPTURE_SEC = 1.0  # 카운트다운 이후 이만큼 샘플을 모아서 평균/표준편차 계산


def read_key_blocking():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", 5000))
    sock.settimeout(0.1)

    latest = {"joints": {}, "active": False}
    lock = threading.Lock()
    quit_flag = {"q": False}

    def udp_thread():
        while not quit_flag["q"]:
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                continue
            try:
                msg = json.loads(data.decode("utf-8"))
                joints = {}
                for item in msg.get("joints", []):
                    name = item.get("name")
                    pos = item.get("pos")
                    if name and pos and len(pos) == 3:
                        joints[name] = np.asarray(pos, dtype=np.float64)
                with lock:
                    latest["joints"] = joints
                    latest["active"] = bool(msg.get("active", False))
            except Exception:
                continue

    threading.Thread(target=udp_thread, daemon=True).start()

    log_path = f"position_probe_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_file = open(log_path, "w", newline="")
    log_writer = csv.writer(log_file)
    header = ["pose_id", "pose_label", "n_samples"]
    for name in JOINT_NAMES:
        header += [f"{name}_x", f"{name}_y", f"{name}_z",
                   f"{name}_std_x", f"{name}_std_y", f"{name}_std_z"]
    header += ["TorsoCenter_computed_x", "TorsoCenter_computed_y", "TorsoCenter_computed_z"]  # (L_UpperArm+R_UpperArm)/2, 실측 아님
    # 파생값: 손목-어깨 상대 벡터 (양팔), 팔 길이
    for side in ("L", "R"):
        header += [f"{side}_wrist_rel_x", f"{side}_wrist_rel_y", f"{side}_wrist_rel_z",
                   f"{side}_arm_length_cm"]
    log_writer.writerow(header)

    print(f"로그 저장 위치: {log_path}")
    print("각 자세를 취하고 준비되면 스페이스바를 누르세요. (Ctrl+C: 종료)\n")

    try:
        for pose_id, pose_label in POSES:
            print(f"\n[다음 자세] {pose_label}")
            print("  준비되면 스페이스바를 누르세요...")
            while True:
                key = read_key_blocking()
                if key == "\x03":
                    raise KeyboardInterrupt
                if key == " ":
                    break

            for remaining in range(int(COUNTDOWN_SEC), 0, -1):
                print(f"  {remaining}...", end="\r", flush=True)
                time.sleep(1.0)
            print("  캡처 중...       ", end="\r", flush=True)

            samples = {name: [] for name in JOINT_NAMES}
            t_end = time.time() + CAPTURE_SEC
            while time.time() < t_end:
                with lock:
                    joints = dict(latest["joints"])
                    active = latest["active"]
                if active:
                    for name in JOINT_NAMES:
                        if name in joints:
                            samples[name].append(joints[name])
                time.sleep(0.02)

            missing = [name for name in JOINT_NAMES if len(samples[name]) == 0]
            if missing:
                print(f"  [실패] 트래킹 안 잡힌 관절: {missing} -- 이 자세 건너뜀, 다시 시도하려면 스크립트 재실행")
                continue

            means = {}
            stds = {}
            for name in JOINT_NAMES:
                arr = np.array(samples[name])
                means[name] = arr.mean(axis=0)
                stds[name] = arr.std(axis=0)

            n = len(samples["Head"])
            row = [pose_id, pose_label, n]
            for name in JOINT_NAMES:
                row += list(means[name]) + list(stds[name])
            torso_computed = (means["L_UpperArm"] + means["R_UpperArm"]) / 2.0
            row += list(torso_computed)

            rel_info = {}
            for side, upper_name, wrist_name in (
                ("L", "L_UpperArm", "L_WristTwist"),
                ("R", "R_UpperArm", "R_WristTwist"),
            ):
                rel = means[wrist_name] - means[upper_name]
                length = float(np.linalg.norm(rel))
                rel_info[side] = (rel, length)
                row += list(rel) + [length]

            log_writer.writerow(row)
            log_file.flush()

            print(f"  [캡처 완료] n={n}샘플")
            for name in JOINT_NAMES:
                std_mag = float(np.linalg.norm(stds[name]))
                flag = "  <-- 노이즈 큼" if std_mag > 1.0 else ""
                print(f"    {name:14s} pos={means[name]}  (표준편차 크기={std_mag:.3f}){flag}")
            print(f"    {'TorsoCenter(계산)':14s} pos={torso_computed}  (실측 아님, 양 어깨 중점)")
            for side in ("L", "R"):
                rel, length = rel_info[side]
                print(f"    {side}_손목-어깨 벡터={rel}  거리={length:.1f}")

        print("\n모든 자세 캡처 완료.")

    except KeyboardInterrupt:
        print("\n중단됨. 지금까지 캡처된 자세는 CSV에 저장되어 있습니다.")
    finally:
        quit_flag["q"] = True
        sock.close()
        log_file.close()
        print(f"\n로그 저장됨: {log_path}")


if __name__ == "__main__":
    main()
