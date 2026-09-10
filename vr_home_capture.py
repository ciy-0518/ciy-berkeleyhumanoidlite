#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VR 홈 자세(팔을 자연스럽게 내린 상태)의 쿼터니언을 측정해서 yaml로 저장한다.

모터/CAN을 전혀 쓰지 않는다. Unreal에서 오는 UDP만 받는다.

Keys:
  c      : 현재 VR 자세를 홈으로 캡처 (여러 번 눌러 다시 잡을 수 있음)
  s      : 캡처된 값을 yaml로 저장
  Ctrl+C : 종료

사용 순서:
  1. Unreal Play 모드 실행, VR 헤드셋 착용
  2. 이 스크립트 실행
  3. VR_OK 확인 후, 로봇 홈과 같은 자세(팔을 자연스럽게 내림)를 취함
  4. c 눌러 캡처 -> 화면에 값 확인
  5. s 눌러 저장

캡처는 CAPTURE_SAMPLES 프레임을 모아 평균 내므로, c를 누른 뒤
약 1~2초간 자세를 유지해야 한다(15Hz 송신 기준).
"""

import argparse
import json
import math
import os
import socket
import sys
import threading
import time
from typing import Dict, List, Optional

import numpy as np


VR_JOINTS = {
    "left": {
        "upper": "L_UpperArm",
        "lower": "L_LowerArm",
        "wrist": "L_WristTwist",
    },
    "right": {
        "upper": "R_UpperArm",
        "lower": "R_LowerArm",
        "wrist": "R_WristTwist",
    },
}

SEGMENTS = ("upper", "lower", "wrist")
SIDES = ("left", "right")

DEFAULT_UDP_PORT = 5000
UDP_TIMEOUT_SEC = 0.5

# c를 누른 뒤 이만큼의 프레임을 모아 평균낸다. Unreal이 15Hz로 보내므로
# 20프레임이면 약 1.3초. 트래킹 지터를 줄이기 위한 것.
CAPTURE_SAMPLES = 20

# 로봇 홈 자세(이미 측정 완료). yaml에 같이 기록해서 한 파일로 관리한다.
ROBOT_HOME = {
    "left": {
        "shoulder_pitch": 0.0573,
        "shoulder_roll": -0.1965,
        "shoulder_yaw": -0.1673,
        "elbow_pitch": 0.0117,
        "elbow_roll": 0.0672,
    },
    "right": {
        "shoulder_pitch": 0.0103,
        "shoulder_roll": -0.1533,
        "shoulder_yaw": -0.0556,
        "elbow_pitch": -0.0865,
        "elbow_roll": 0.0945,
    },
}


def quat_normalize(q) -> np.ndarray:
    arr = np.asarray(q, dtype=np.float64)
    if arr.shape != (4,) or not np.all(np.isfinite(arr)):
        raise ValueError("quaternion must contain four finite numbers")
    norm = float(np.linalg.norm(arr))
    if norm < 1e-9:
        raise ValueError("zero-length quaternion")
    return arr / norm


def quat_average(quats: List[np.ndarray]) -> np.ndarray:
    """여러 쿼터니언의 평균.

    q와 -q는 같은 회전이므로, 단순 산술평균 전에 첫 샘플과 부호를 맞춘다
    (내적이 음수면 뒤집는다). 작은 각도 범위의 샘플에 대해서는 이 방식이
    충분히 정확하다.
    """
    if not quats:
        raise ValueError("no samples to average")

    reference = quats[0]
    aligned = []
    for q in quats:
        aligned.append(-q if float(np.dot(reference, q)) < 0.0 else q)

    mean = np.mean(np.stack(aligned, axis=0), axis=0)
    return quat_normalize(mean)


def quat_max_deviation_deg(quats: List[np.ndarray], mean: np.ndarray) -> float:
    """평균 대비 각 샘플의 최대 각도 편차(도). 캡처 안정성 확인용."""
    worst = 0.0
    for q in quats:
        dot = abs(float(np.dot(mean, q)))
        dot = min(1.0, max(-1.0, dot))
        angle = 2.0 * math.acos(dot)
        worst = max(worst, angle)
    return math.degrees(worst)


class BodyTrackingUdpReceiver:
    def __init__(self, listen_ip: str, listen_port: int):
        self.lock = threading.Lock()
        self.joints: Dict[str, np.ndarray] = {}
        self.active = False
        self.received_at = 0.0
        self.source = None
        self.quit = False

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((listen_ip, listen_port))
        self.sock.settimeout(0.1)
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.quit = True
        try:
            self.sock.close()
        except OSError:
            pass

    def snapshot(self):
        with self.lock:
            return dict(self.joints), self.active, self.received_at, self.source

    def _run(self) -> None:
        while not self.quit:
            try:
                data, source = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                message = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue

            joints: Dict[str, np.ndarray] = {}
            for item in message.get("joints", []):
                if not isinstance(item, dict):
                    continue
                name = item.get("name")
                rot = item.get("rot")
                if not isinstance(name, str) or rot is None:
                    continue
                try:
                    joints[name] = quat_normalize(rot)
                except ValueError:
                    continue

            with self.lock:
                self.joints = joints
                self.active = bool(message.get("active", False))
                self.received_at = time.monotonic()
                self.source = source


def read_key() -> str:
    if os.name == "nt":
        import msvcrt
        return msvcrt.getwch()

    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def keyboard_worker(state: dict) -> None:
    try:
        while not state["quit"]:
            key = read_key().lower()
            if key == "\x03":
                state["quit"] = True
            elif key == "c":
                state["capture"] = True
            elif key == "s":
                state["save"] = True
    except (EOFError, OSError):
        print("\n[KEYBOARD] interactive keys unavailable; Ctrl+C still works")


def required_names(sides) -> List[str]:
    names = []
    for side in sides:
        for segment in SEGMENTS:
            names.append(VR_JOINTS[side][segment])
    return names


def format_quat(q: np.ndarray) -> str:
    return "[" + ", ".join(f"{v:+.6f}" for v in q) + "]"


def write_yaml(path: str, captured: Dict[str, Dict[str, np.ndarray]]) -> None:
    """의존성을 늘리지 않기 위해 yaml을 직접 쓴다(구조가 단순하므로)."""
    lines = []
    lines.append("# home_pose.yaml")
    lines.append("# 팔을 자연스럽게 내린 뉴트럴 자세 기준")
    lines.append(f"# 생성: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("")
    lines.append("# 로봇 홈 (모터 절대 각도, rad)")
    lines.append("robot:")
    for side in SIDES:
        lines.append(f"  {side}:")
        for joint, value in ROBOT_HOME[side].items():
            lines.append(f"    {joint}: {value}")
    lines.append("")
    lines.append("# VR 홈 (쿼터니언 [x, y, z, w])")
    lines.append("vr:")
    for side in SIDES:
        if side not in captured:
            continue
        lines.append(f"  {side}:")
        for segment in SEGMENTS:
            q = captured[side][segment]
            values = ", ".join(f"{v:.6f}" for v in q)
            lines.append(f"    {segment}: [{values}]")
    lines.append("")

    with open(path, "w") as f:
        f.write("\n".join(lines))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="VR 홈 자세 쿼터니언 측정 (모터/CAN 사용 안 함)"
    )
    parser.add_argument("--side", choices=("both", "left", "right"), default="both")
    parser.add_argument("--udp-ip", default="0.0.0.0")
    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    parser.add_argument("--output", default="home_pose.yaml", help="저장할 yaml 경로")
    parser.add_argument(
        "--samples",
        type=int,
        default=CAPTURE_SAMPLES,
        help="캡처 시 평균낼 프레임 수 (기본 20, 15Hz 기준 약 1.3초)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    sides = SIDES if args.side == "both" else (args.side,)
    needed = required_names(sides)

    receiver = BodyTrackingUdpReceiver(args.udp_ip, args.udp_port)
    receiver.start()

    state = {"quit": False, "capture": False, "save": False}
    threading.Thread(target=keyboard_worker, args=(state,), daemon=True).start()

    captured: Dict[str, Dict[str, np.ndarray]] = {}

    print("=" * 78)
    print("VR 홈 자세 캡처 (모터/CAN 미사용)")
    print(f"UDP bind : {args.udp_ip}:{args.udp_port}")
    print(f"Arms     : {', '.join(sides)}")
    print(f"Samples  : {args.samples} 프레임 평균")
    print(f"Output   : {args.output}")
    print("-" * 78)
    print("  c : 현재 자세를 홈으로 캡처 (누른 뒤 1~2초간 자세 유지)")
    print("  s : 캡처된 값을 yaml로 저장")
    print("  Ctrl+C : 종료")
    print("=" * 78)
    print()

    # 캡처 진행 중일 때 모으는 버퍼
    capturing = False
    buffers: Dict[str, List[np.ndarray]] = {}
    last_frame_time = 0.0
    last_print = 0.0

    try:
        while not state["quit"]:
            joints, active, received_at, source = receiver.snapshot()
            now = time.monotonic()
            age = math.inf if received_at == 0.0 else now - received_at
            tracking_ok = (
                active
                and age <= UDP_TIMEOUT_SEC
                and all(name in joints for name in needed)
            )

            if state["capture"]:
                state["capture"] = False
                if not tracking_ok:
                    print("\n[CAPTURE] 실패: VR 트래킹이 유효하지 않습니다")
                else:
                    capturing = True
                    buffers = {name: [] for name in needed}
                    last_frame_time = 0.0
                    print(f"\n[CAPTURE] 시작 - {args.samples}프레임 수집 중, 자세 유지하세요...")

            if capturing:
                if not tracking_ok:
                    capturing = False
                    print("[CAPTURE] 중단: 트래킹이 끊겼습니다. 다시 시도하세요")
                elif received_at != last_frame_time:
                    # 새 프레임이 도착했을 때만 수집(같은 프레임 중복 방지)
                    last_frame_time = received_at
                    for name in needed:
                        buffers[name].append(joints[name].copy())

                    collected = len(buffers[needed[0]])
                    print(f"  수집 {collected}/{args.samples}", end="\r", flush=True)

                    if collected >= args.samples:
                        capturing = False
                        print(" " * 40, end="\r")
                        print("[CAPTURE] 완료\n")

                        for side in sides:
                            captured[side] = {}
                            print(f"[{side.upper()}]")
                            for segment in SEGMENTS:
                                name = VR_JOINTS[side][segment]
                                samples = buffers[name]
                                mean = quat_average(samples)
                                spread = quat_max_deviation_deg(samples, mean)
                                captured[side][segment] = mean
                                flag = "  <- 흔들림 큼" if spread > 5.0 else ""
                                print(
                                    f"  {segment:6s} {format_quat(mean)} "
                                    f"(편차 {spread:.1f}deg){flag}"
                                )
                            print()
                        print("문제 없으면 s 를 눌러 저장하세요.\n")

            if state["save"]:
                state["save"] = False
                if not captured:
                    print("\n[SAVE] 실패: 먼저 c 로 캡처하세요")
                else:
                    missing = [s for s in sides if s not in captured]
                    if missing:
                        print(f"\n[SAVE] 실패: 캡처되지 않은 arm: {', '.join(missing)}")
                    else:
                        write_yaml(args.output, captured)
                        print(f"\n[SAVE] 저장 완료: {args.output}\n")

            if not capturing and now - last_print >= 0.2:
                last_print = now
                status = "VR_OK" if tracking_ok else "VR_WAIT"
                src = "-" if source is None else source[0]
                age_text = "inf" if math.isinf(age) else f"{age:.2f}"
                has = "CAPTURED" if captured else "no capture"
                print(
                    f"[{status}] src={src} age={age_text}s  {has}",
                    end="\r",
                    flush=True,
                )

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        state["quit"] = True
        receiver.stop()
        print("\n종료")


if __name__ == "__main__":
    main()
