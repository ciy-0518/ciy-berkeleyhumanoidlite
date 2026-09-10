#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
순수 진단용: 모터/CAN 전혀 안 씀. VR 데이터만 받아서
upper/elbow/wrist delta의 raw roll/pitch/yaw 9개 값을 전부 출력한다.

c : 현재 자세를 neutral로 저장
Ctrl+C : 종료

화면엔 한 줄씩 새로 출력(스크롤백 가능)하고,
동시에 CSV 파일로도 기록한다.
"""

import csv
import json
import math
import socket
import sys
import termios
import threading
import time
import tty

import numpy as np


VR_JOINTS = {
    "left":  {"upper": "L_UpperArm", "lower": "L_LowerArm", "wrist": "L_WristTwist"},
    "right": {"upper": "R_UpperArm", "lower": "R_LowerArm", "wrist": "R_WristTwist"},
}

PRINT_INTERVAL_SEC = 0.10  # 화면/로그 출력 주기 (10Hz, 너무 빠르면 읽기 힘들어서 줄임)


def quat_normalize(q):
    arr = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(arr)
    if n < 1e-9:
        raise ValueError("zero-length quaternion")
    return arr / n


def quat_inverse(q):
    x, y, z, w = quat_normalize(q)
    return np.array([-x, -y, -z, w])


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = quat_normalize(q1)
    x2, y2, z2, w2 = quat_normalize(q2)
    return quat_normalize((
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ))


def quat_relative(q_ref, q_cur):
    return quat_multiply(quat_inverse(q_ref), q_cur)


def quat_to_euler_xyz(q):
    x, y, z, w = quat_normalize(q)
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y-z*x))))
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return roll, pitch, yaw


SHOULDER_TWIST_AXIS = np.array([0.0, 1.0, 0.0])  # vr_control_watchdog_biarm.py와 동일


def shoulder_axis_components(upper_delta):
    """vr_control_watchdog_biarm.py의 shoulder_axis_components()와 동일한 twist-swing 분해.
    실제 컨트롤 코드가 지금 무엇을 보고 있는지 그대로 재현해서 로그로 남기기 위함.
    """
    x, y, z, w = quat_normalize(upper_delta)
    n = SHOULDER_TWIST_AXIS
    proj = float(np.dot((x, y, z), n))
    twist = quat_normalize(np.array([proj*n[0], proj*n[1], proj*n[2], w]))
    swing = quat_multiply(np.array([x, y, z, w]), quat_inverse(twist))
    sx, sy, sz, sw = quat_normalize(swing)
    return {
        "pitch": 2.0 * math.atan2(proj, w),   # shoulder_yaw (twist, exact)
        "roll": 2.0 * math.atan2(sx, sw),     # shoulder_pitch (swing)
        "yaw": 2.0 * math.atan2(sz, sw),      # shoulder_roll (swing)
    }


EMA_ALPHA = 0.35  # vr_control_watchdog_biarm.py 기본값과 동일


def main():
    side = sys.argv[1] if len(sys.argv) > 1 else "right"
    names = VR_JOINTS[side]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", 5000))
    sock.settimeout(0.1)

    latest = {"joints": {}, "active": False, "seq": 0}
    lock = threading.Lock()
    neutral = {"upper": None, "elbow_rel": None, "wrist_rel": None}
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
                    rot = item.get("rot")
                    if name and rot and len(rot) == 4:
                        joints[name] = quat_normalize(rot)
                with lock:
                    latest["joints"] = joints
                    latest["active"] = bool(msg.get("active", False))
                    latest["seq"] += 1  # 새 UDP 패킷마다 증가 (EMA는 이게 바뀔 때만 블렌딩)
            except Exception:
                continue

    filt = {"upper": None, "lower": None, "wrist": None, "seq": None}

    def get_smoothed_joints():
        """latest에서 side에 필요한 upper/lower/wrist를 읽어 EMA로 스무딩.
        latest["seq"]가 바뀐(=새 UDP 패킷 도착) 경우에만 블렌딩한다.
        메인 루프는 10Hz로 도는데 VR은 그보다 느릴 수 있어서, 같은 패킷을
        여러 번 블렌딩해 필터가 무의미해지는 걸 막기 위함.
        """
        with lock:
            joints = dict(latest["joints"])
            seq = latest["seq"]
        if not all(n in joints for n in names.values()):
            return None
        raw = {
            "upper": joints[names["upper"]],
            "lower": joints[names["lower"]],
            "wrist": joints[names["wrist"]],
        }
        if filt["upper"] is None:
            filt.update({k: v.copy() for k, v in raw.items()})
            filt["seq"] = seq
            return dict(raw)
        if seq != filt["seq"]:
            filt["seq"] = seq
            for key, q_new in raw.items():
                q_prev = filt[key]
                if float(np.dot(q_prev, q_new)) < 0.0:
                    q_new = -q_new
                blended = EMA_ALPHA * q_new + (1.0 - EMA_ALPHA) * q_prev
                filt[key] = quat_normalize(blended)
        return {k: filt[k] for k in ("upper", "lower", "wrist")}

    def key_thread():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            while not quit_flag["q"]:
                tty.setraw(fd)
                ch = sys.stdin.read(1)
                if ch == "\x03":
                    quit_flag["q"] = True
                elif ch == "c":
                    joints = get_smoothed_joints()
                    if joints is not None:
                        upper = joints["upper"]
                        lower = joints["lower"]
                        wrist = joints["wrist"]
                        neutral["upper"] = upper.copy()
                        neutral["elbow_rel"] = quat_relative(upper, lower)
                        neutral["wrist_rel"] = quat_relative(lower, wrist)
                        print("\n[CALIBRATED]")
                    else:
                        print("\n[CALIBRATION FAILED] joints missing")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    threading.Thread(target=udp_thread, daemon=True).start()
    threading.Thread(target=key_thread, daemon=True).start()

    log_path = f"euler_probe_log_{side}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_file = open(log_path, "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow([
        "t", "ur", "up", "uy", "sr", "sp", "sy", "er", "ep", "ey", "wr", "wp", "wy"
    ])

    print(f"side={side}  |  c=calibrate, Ctrl+C=quit")
    print(f"로그 저장 위치: {log_path}")
    print("ur/up/uy = 기존 순차 Euler 분해 (참고용, 여전히 축간 누출 있음)")
    print("sr/sp/sy = 새 twist-swing 분해 (vr_control_watchdog_biarm.py가 실제로 쓰는 값)")
    print("  sr = shoulder_pitch(swing)  sp = shoulder_yaw(twist, 정확)  sy = shoulder_roll(swing)")
    print("동작 하나씩 해보면서 어느 값이 반응하는지 보세요.\n")

    t0 = time.time()
    last_print = 0.0

    try:
        while not quit_flag["q"]:
            with lock:
                active = latest["active"]
            joints = get_smoothed_joints()

            if neutral["upper"] is None or not active or joints is None:
                time.sleep(0.02)
                continue

            now = time.time()
            if now - last_print < PRINT_INTERVAL_SEC:
                time.sleep(0.01)
                continue
            last_print = now

            upper = joints["upper"]
            lower = joints["lower"]
            wrist = joints["wrist"]

            upper_delta = quat_relative(neutral["upper"], upper)
            elbow_delta = quat_relative(neutral["elbow_rel"], quat_relative(upper, lower))
            wrist_delta = quat_relative(neutral["wrist_rel"], quat_relative(lower, wrist))

            ur, up, uy = quat_to_euler_xyz(upper_delta)
            shoulder = shoulder_axis_components(upper_delta)
            sr, sp, sy = shoulder["roll"], shoulder["pitch"], shoulder["yaw"]
            er, ep, ey = quat_to_euler_xyz(elbow_delta)
            wr, wp, wy = quat_to_euler_xyz(wrist_delta)

            elapsed = now - t0
            line = (
                f"[{elapsed:7.2f}s] "
                f"upper_euler[r={ur:+.2f} p={up:+.2f} y={uy:+.2f}]  "
                f"upper_swing[r={sr:+.2f} p={sp:+.2f} y={sy:+.2f}]  "
                f"elbow[r={er:+.2f} p={ep:+.2f} y={ey:+.2f}]  "
                f"wrist[r={wr:+.2f} p={wp:+.2f} y={wy:+.2f}]"
            )
            print(line)

            log_writer.writerow([
                f"{elapsed:.3f}",
                f"{ur:.4f}", f"{up:.4f}", f"{uy:.4f}",
                f"{sr:.4f}", f"{sp:.4f}", f"{sy:.4f}",
                f"{er:.4f}", f"{ep:.4f}", f"{ey:.4f}",
                f"{wr:.4f}", f"{wp:.4f}", f"{wy:.4f}",
            ])
            log_file.flush()
    except KeyboardInterrupt:
        pass
    finally:
        quit_flag["q"] = True
        sock.close()
        log_file.close()
        print(f"\n종료. 로그 저장됨: {log_path}")


if __name__ == "__main__":
    main()