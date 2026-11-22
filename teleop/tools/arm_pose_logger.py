#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""팔 10모터 값 실시간 표시 + 키를 누른 순간의 값을 라벨과 함께 CSV 로 남기는 **읽기 전용** 도구.

왜 필요한가 (ROBOT_JOINTS.md §6, SESSION_20260921_robot_ref.md)
----------------------------------------------------------------
팔의 절대 영점은 전원마다 다를 수 있다. 그래서 "물리적으로 알려진 기준 자세"에서 매 세션 모터 값을 잡는다.
그 기준 자세의 반복성(A), 관절별 부호(B), 기구 한계(C), 전원 재시작 후 값(D)을 잰다.

🔴 읽기 전용 — 이 도구는 모터에 **파라미터 읽기 요청(SDO read) 외에는 아무것도 보내지 않는다.**
   모드 변경(set_mode) · 파라미터 쓰기 · 플래시 저장 · 하트비트 · ping 모두 **전송 직전에 막힌다** (`ReadOnlyBus`).
   `joint_limit_probe.py` 와 다른 점: 그 도구는 시작 시 모든 모터를 IDLE 로 **바꾸고**,
   position_offset 이 0 이 아니면 **0 으로 써서 플래시에 저장**한다. 이 도구는 둘 다 하지 않는다.
   ⇒ 모터가 IDLE/DISABLED 가 아니면 **바꾸지 않고 종료**한다 (전원을 껐다 켜서 기본 모드로 돌아오게 할 것).

사용법 (로봇 PC, 워치독이 꺼져 있을 때 — 같은 CAN 을 쓰면 응답이 섞인다)
    cd ~/code_workspace/Berkeley-Humanoid-Lite-main/Berkeley-Humanoid-Lite-Lowlevel
    python3 ~/teleop-docs/_teleop/tools/arm_pose_logger.py --part A      # A / B / C / D / all
    python3 ~/teleop-docs/_teleop/tools/arm_pose_logger.py --self-test   # 로봇 없이 읽기 전용 가드 검사
    python3 ~/teleop-docs/_teleop/tools/arm_pose_logger.py --part G      # 그리퍼 맞닿음 (2초 가만히면 자동 기록, REPORT_54)
    python3 ~/teleop-docs/_teleop/tools/arm_pose_logger.py --part G --manual   # 같은 순서, space 로만 기록
    python3 ~/teleop-docs/_teleop/tools/arm_pose_logger.py --part Q      # 주차 자세 반복 오차 (읽기만, 작업 지시 11-b)
    python3 ~/teleop-docs/_teleop/tools/arm_pose_logger.py --part M      # 마스터 기준 robot_master.json (판을 뗀 상태로 시작)
    python3 ~/teleop-docs/_teleop/tools/arm_pose_logger.py --part P      # 매 전원: 주차 자세 → robot_ref_*.json
    (예전 --part M 만세 기구 한계는 --part MAX 로 옮겼다)
    python3 ~/teleop-docs/_teleop/tools/arm_pose_logger.py --part T      # 집게 위치·방향 실측 (14-r, tools/calib_tip.py). 그리퍼만 연다/닫는다
    (예전 --part T 비틀기 두 관절 확인은 --part TW 로 옮겼다. 단계·기록은 그대로)

키
    space  지금 값을 기록하고 다음 단계로 (0.4 초 평균)
    r      직전 기록을 지우고 그 단계로 돌아가기
    s      이 단계 건너뛰기 (기록에 skip 으로 남김)
    m      단계와 무관한 표시 기록 (label = mark)
    q      종료 (Ctrl+C 도 같음)

출력 (기본 ~/teleop-docs/_teleop/logs/)
    armpose_<시각>_<part>_meta.json    시작 시 읽은 모터 설정 (모드·offset·기어비·상순서·토크한계·엔코더)
    armpose_<시각>_<part>_snap.csv     키를 누른 순간 기록 (단계 id·라벨·10모터 평균/표준편차 + 엔코더 회전수)
    armpose_<시각>_<part>_stream.csv   모든 샘플 (≈10 Hz)
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import select
import struct
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path

LOWLEVEL = Path.home() / "code_workspace/Berkeley-Humanoid-Lite-main/Berkeley-Humanoid-Lite-Lowlevel"
sys.path.insert(0, str(LOWLEVEL))
try:
    import berkeley_humanoid_lite_lowlevel.recoil as recoil
except ImportError:
    recoil = None

LOGS = Path(__file__).resolve().parent.parent / "logs"
JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "elbow_roll")
KOR = {"shoulder_pitch": "어깨 pitch", "shoulder_roll": "어깨 roll", "shoulder_yaw": "어깨 yaw",
       "elbow_pitch": "팔꿈치", "elbow_roll": "아래팔 비틀기"}
IDS = {"left": (1, 3, 5, 7, 9), "right": (2, 4, 6, 8, 10)}
CHANNELS = {"left": "can0", "right": "can1"}
SAFE_MODES = {0: "DISABLED", 1: "IDLE"}
READ_TIMEOUT = 0.05


# --------------------------------------------------------------------------- 읽기 전용 가드

class WriteBlocked(RuntimeError):
    pass


def _is_sdo_read(func_id: int, data: bytes) -> bool:
    """recoil.Bus._read_parameter 가 보내는 프레임만 참: RECEIVE_SDO, 3바이트, 첫 바이트 0x02<<5."""
    return (func_id == recoil.Function.RECEIVE_SDO and len(data) == 3 and data[0] == (0x02 << 5))


def make_readonly_bus_class():
    class ReadOnlyBus(recoil.Bus):
        """모든 전송이 지나가는 transmit() 에서 SDO 읽기 요청이 아니면 **보내기 전에** 예외."""
        sent_reads = 0

        def transmit(self, frame):
            data = bytes(frame.data) if frame.data is not None else b""
            if not _is_sdo_read(frame.func_id, data):
                raise WriteBlocked(f"읽기 전용: func={frame.func_id:#06b} dev={frame.device_id} data={data.hex()} 전송 차단")
            self.sent_reads += 1
            return super().transmit(frame)

        # 명시적으로도 막는다 (transmit 가드의 이중 안전)
        def set_mode(self, *a, **k):
            raise WriteBlocked("읽기 전용: set_mode 금지")

        def store_settings_to_flash(self, *a, **k):
            raise WriteBlocked("읽기 전용: 플래시 저장 금지")

        def load_settings_from_flash(self, *a, **k):
            raise WriteBlocked("읽기 전용: 플래시 로드 금지")

        def _write_parameter(self, *a, **k):
            raise WriteBlocked("읽기 전용: 파라미터 쓰기 금지")

        def feed(self, *a, **k):
            raise WriteBlocked("읽기 전용: heartbeat 금지")

        def ping(self, *a, **k):
            raise WriteBlocked("읽기 전용: ping 금지 (PDO 전송)")

    return ReadOnlyBus


def self_test() -> int:
    """로봇 없이: 가짜 CAN 으로 가드가 읽기만 통과시키는지 확인."""
    if recoil is None:
        print("recoil import 실패 — 로봇 PC 에서 실행하라"); return 1
    RO = make_readonly_bus_class()
    sent = []

    class FakeCan:
        def send(self, msg):
            sent.append(msg)

        def recv(self, timeout=None):
            return None

        def shutdown(self):
            pass

    bus = object.__new__(RO)
    bus.channel, bus.bitrate = "fake", 1_000_000
    bus._Bus__bus = FakeCan()
    ok = True

    def expect_block(name, fn):
        nonlocal ok
        n0 = len(sent)
        try:
            fn()
            print(f"  ❌ {name}: 막히지 않았다"); ok = False
        except WriteBlocked:
            if len(sent) != n0:
                print(f"  ❌ {name}: 막혔지만 프레임이 나갔다"); ok = False
            else:
                print(f"  ✅ {name}: 차단, 전송 0")

    r = bus._read_parameter_f32(1, recoil.Parameter.POSITION_CONTROLLER_POSITION_MEASURED, timeout=0)
    print(f"  {'✅' if len(sent) == 1 and r is None else '❌'} 읽기 요청 1개 전송 (응답 없음 → None)")
    ok &= len(sent) == 1
    expect_block("set_mode(IDLE)", lambda: bus.set_mode(1, recoil.Mode.IDLE))
    expect_block("set_mode(POSITION)", lambda: bus.set_mode(1, recoil.Mode.POSITION))
    expect_block("write_position_offset", lambda: bus.write_position_offset(1, 0.0))
    expect_block("_write_parameter_f32", lambda: bus._write_parameter_f32(1, 0x040, 0.0))
    expect_block("store_settings_to_flash", lambda: bus.store_settings_to_flash(1))
    expect_block("feed", lambda: bus.feed(1))
    expect_block("ping", lambda: bus.ping(1))
    expect_block("write_read_pdo_2", lambda: bus.write_read_pdo_2(1, 0.0, 0.0))
    # 가드를 우회해 base transmit 을 부르는 경로가 있는가 — 가짜 프레임을 직접 넣어 본다
    expect_block("transmit(NMT 프레임 직접)", lambda: bus.transmit(
        recoil.CANFrame(1, recoil.Function.NMT, size=2, data=struct.pack("<BB", 0x13, 1))))
    expect_block("transmit(SDO 쓰기 직접)", lambda: bus.transmit(
        recoil.CANFrame(1, recoil.Function.RECEIVE_SDO, size=8, data=struct.pack("<BHB", 0x01 << 5, 0x40, 0) + b"\0\0\0\0")))
    for m in sent:
        if not _is_sdo_read(m.arbitration_id >> recoil.CANFrame.FUNC_ID_POS, bytes(m.data)):
            print("  ❌ 읽기가 아닌 프레임이 나갔다"); ok = False
    print("자체 검사:", "통과" if ok else "실패")
    return 0 if ok else 1


# --------------------------------------------------------------------------- 절차 (SESSION_20260921_robot_ref.md 와 같은 순서)

HOLD = "🤚 손으로 받친 채"


def steps_A():
    s = []
    for k in range(1, 5):
        s.append(("A", f"A{2*k-1:02d}", f"hang_{k}",
                  f"[A-① 늘어뜨림 #{k}/4] {HOLD} 양팔을 앞뒤·옆으로 크게 흔든 뒤, 받친 손을 천천히 빼서 "
                  f"팔이 중력으로 늘어지게 둔다. 비틀기 관절은 만지지 않는다. 팔이 완전히 멈추면 space."))
        s.append(("A", f"A{2*k:02d}", f"stop_{k}",
                  f"[A-② 멈춤점 #{k}/4] {HOLD} 양팔을 흔든 뒤, 위팔을 몸 쪽으로 밀어 어깨 roll 이 몸통 쪽 "
                  f"멈춤점에 닿게 하고 팔꿈치는 끝까지 편다. 가볍게 댄 채(힘주어 누르지 않는다) space."))
    for k in range(1, 5):
        s.append(("A", f"A{8+k:02d}", f"twiststop_{k}",
                  f"[A-③ 비틀기 멈춤점 #{k}/4 · 선택] 팔 늘어뜨림. {HOLD} 어깨 yaw 를 바깥 비틀기 끝, "
                  f"아래팔 비틀기를 뒤침 끝 멈춤점까지 천천히 돌려 가볍게 댄 채 space. 끝을 모르겠으면 s."))
    return s


B_MOTIONS = [
    ("sp", "어깨 앞으로 들기", "팔 늘어뜨림, 팔꿈치 편 채", "위팔을 앞으로 약 30° 든다"),
    ("sr", "어깨 옆으로 벌리기", "팔 늘어뜨림, 팔꿈치 편 채", "위팔을 옆으로 약 30° 벌린다"),
    ("sy", "상완 안쪽 비틀기", "위팔 늘어뜨림 + 팔꿈치 90° (아래팔이 정면을 향함)",
     "위팔은 제자리, 아래팔 끝이 배 쪽으로 약 30° 오게 비튼다"),
    ("ep", "팔꿈치 굽히기", "팔 늘어뜨림, 팔꿈치 편 채", "팔꿈치를 약 30° 굽힌다 (아래팔 끝이 앞으로)"),
    ("er", "아래팔 비틀기 (엎침)", "위팔 늘어뜨림 + 팔꿈치 90° (아래팔 정면), 아래팔 윗면 테이프 표시가 위",
     "아래팔을 제 축으로, 윗면 표시가 몸 안쪽으로 약 30° 굴러가게 돌린다"),
]


def steps_B():
    s = []
    for side, kor in (("left", "왼팔"), ("right", "오른팔")):
        for key, name, start, move in B_MOTIONS:
            s.append(("B", f"B-{side[0].upper()}-{key}-0", f"{side}_{key}_start",
                      f"[B {kor} · {name} — 시작] {HOLD} {start}. 정지하면 space."))
            s.append(("B", f"B-{side[0].upper()}-{key}-1", f"{side}_{key}_end",
                      f"[B {kor} · {name} — 끝] {HOLD} {move}. 다른 관절은 되도록 그대로. 정지하면 space."))
    return s


C_ENDS = [
    ("sp", "어깨 pitch", "앞으로 드는 끝", "뒤로 젖히는 끝", "팔꿈치 편 채"),
    ("sr", "어깨 roll", "옆으로 벌리는 끝", "몸 쪽 멈춤점", "팔꿈치 편 채"),
    ("sy", "어깨 yaw", "안쪽 비틀기 끝 (아래팔이 배 쪽)", "바깥 비틀기 끝", "팔꿈치 90°"),
    ("ep", "팔꿈치", "굽힘 끝", "폄 끝", "위팔 늘어뜨림"),
    ("er", "아래팔 비틀기", "엎침 끝 (윗면 표시가 몸 안쪽)", "뒤침 끝", "팔꿈치 90°"),
]


def steps_C():
    s = []
    for side, kor in (("left", "왼팔"), ("right", "오른팔")):
        for key, name, end1, end2, pose in C_ENDS:
            for i, end in enumerate((end1, end2)):
                s.append(("C", f"C-{side[0].upper()}-{key}-{i+1}", f"{side}_{key}_end{i+1}",
                          f"[C {kor} · {name} — {end}] {HOLD} ({pose}) 천천히 돌려 처음 저항이 느껴지는 곳에서 멈춘다. "
                          f"힘으로 밀지 않는다. 배선이 당기면 거기서 멈춘다. 멈춤점이 없으면 안전한 곳에서 멈추고 m 후 space."))
    return s


def steps_D():
    s = []
    for k in range(1, 3):
        s.append(("D", f"D{2*k-1:02d}", f"hang_{k}", f"[D 늘어뜨림 #{k}/2] A-① 과 똑같이. 멈추면 space."))
        s.append(("D", f"D{2*k:02d}", f"stop_{k}", f"[D 멈춤점 #{k}/2] A-② 와 똑같이. 가볍게 댄 채 space."))
    return s


def steps_T():
    """비틀기 두 관절(어깨 yaw, 아래팔)의 기준과 부호 — 한 팔에 4번 기록."""
    s = []
    for side, kor in (("left", "왼팔"), ("right", "오른팔")):
        S = side[0].upper()
        s += [("T", f"T-{S}-1", f"{side}_front_1", f"[{kor}] 팔꿈치 90°, 아래팔이 정면, 테이프가 위. 받친 채 space."),
              ("T", f"T-{S}-2", f"{side}_sy_in", f"[{kor}] 그대로 손만 배 쪽으로 조금 (문 닫듯). space."),
              ("T", f"T-{S}-3", f"{side}_front_2", f"[{kor}] 다시 정면, 테이프 위. space."),
              ("T", f"T-{S}-4", f"{side}_er_in", f"[{kor}] 문손잡이 돌리듯 테이프를 몸 안쪽으로 조금. space.")]
    return s


def steps_E():
    """아래팔 비틀기만 다시 (T 에서 어깨 yaw 가 대신 돌았다) — 한 팔에 2번 기록."""
    s = []
    for side, kor in (("left", "왼팔"), ("right", "오른팔")):
        S = side[0].upper()
        s += [("E", f"E-{S}-1", f"{side}_front", f"[{kor}] 팔꿈치 90°, 손목 끝(그리퍼 붙는 면)이 몸 쪽. space."),
              ("E", f"E-{S}-2", f"{side}_er_down",
               f"[{kor}] 팔꿈치 위를 다른 손으로 잡고, 손목만 돌려 그리퍼 면을 바닥 쪽으로. "
               f"화면 '아래팔 비틀기' 숫자가 바뀌면 space. 안 바뀌면 반대로 돌리고 m 후 space.")]
    return s


PARTS = {"A": steps_A, "B": steps_B, "C": steps_C, "D": steps_D, "TW": steps_T, "E": steps_E}   # TW = 예전 T (14-r 에 이름만 옮김)


# --------------------------------------------------------------------------- 본체

META_PARAMS = [
    ("mode", "MODE", "u32"),
    ("position_offset", "POSITION_CONTROLLER_POSITION_OFFSET", "f32"),
    ("gear_ratio", "POSITION_CONTROLLER_GEAR_RATIO", "f32"),
    ("torque_limit", "POSITION_CONTROLLER_TORQUE_LIMIT", "f32"),
    ("position_limit_lower", "POSITION_CONTROLLER_POSITION_LIMIT_LOWER", "f32"),
    ("position_limit_upper", "POSITION_CONTROLLER_POSITION_LIMIT_UPPER", "f32"),
    ("phase_order", "MOTOR_PHASE_ORDER", "i32"),
    ("encoder_cpr", "ENCODER_CPR", "u32"),
    ("encoder_position_offset", "ENCODER_POSITION_OFFSET", "f32"),
    ("encoder_n_rotations", "ENCODER_N_ROTATIONS", "i32"),
    ("encoder_position_raw", "ENCODER_POSITION_RAW", "i32"),
    ("encoder_position", "ENCODER_POSITION", "f32"),
    ("position_measured", "POSITION_CONTROLLER_POSITION_MEASURED", "f32"),
]


def rd(bus, dev, pname, typ, timeout=READ_TIMEOUT):
    pid = getattr(recoil.Parameter, pname)
    fn = {"f32": bus._read_parameter_f32, "i32": bus._read_parameter_i32, "u32": bus._read_parameter_u32}[typ]
    return fn(dev, pid, timeout=timeout)


def motors_list(buses):
    return [(side, j, dev, buses[CHANNELS[side]]) for side in ("left", "right") for j, dev in zip(JOINTS, IDS[side])]


def read_positions(motors):
    return [rd(b, d, "POSITION_CONTROLLER_POSITION_MEASURED", "f32") for _, _, d, b in motors]


def read_encoders(motors):
    # 늦게 온 위치 응답이 엔코더 읽기에 잘못 잡히지 않도록 넉넉히 기다린다 (응답 프레임에 파라미터 id 가 없다)
    return [(rd(b, d, "ENCODER_N_ROTATIONS", "i32", timeout=0.3), rd(b, d, "ENCODER_POSITION_RAW", "i32", timeout=0.3))
            for _, _, d, b in motors]


class Keys:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *a):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def get(self):
        r, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if r else None


def fmt(v, w=7):
    return f"{v:+{w}.3f}" if isinstance(v, float) and math.isfinite(v) else " " * (w - 3) + "???"


# --------------------------------------------------------------------------- H: "내 팔로 먼저 → 로봇을 똑같이" (2026-09-22, 문구 사용자 승인)

H_NAME = {"shoulder_pitch": "어깨 앞뒤", "shoulder_roll": "어깨 옆", "shoulder_yaw": "어깨 비틀기",
          "elbow_pitch": "팔꿈치", "elbow_roll": "손목 비틀기"}
H_MIN_DEG = 15.0
E90 = "다시 엄지가 위"
# (제목, 안내, 바뀌어야 할 관절, 분석용 라벨)
H_STEPS = [
    ("차렷", "차렷. 팔꿈치 쭉 펴고, 손바닥은 허벅지, 엄지는 앞", None, "rest"),
    ("어깨 앞", "팔꿈치는 쭉 편 채로, 팔 전체를 어깨에서부터 앞으로 든다. 손이 배꼽 높이쯤 오게. 팔꿈치는 굽히지 않는다",
     "shoulder_pitch", "sp"),
    ("차렷", "다시 차렷", "shoulder_pitch", "rest"),
    ("어깨 옆", "팔꿈치는 쭉 편 채로, 팔 전체를 어깨에서부터 옆으로 든다. 손이 배꼽 높이쯤 오게. 팔꿈치는 굽히지 않는다",
     "shoulder_roll", "sr"),
    ("차렷", "다시 차렷", "shoulder_roll", "rest"),
    ("팔꿈치 90°", "위팔은 그대로, 팔꿈치만 90° 굽힌다. 엄지가 위", "elbow_pitch", "e90"),
    ("어깨 비틀기", "팔꿈치 90° 그대로, 팔짱 끼듯 손을 배 쪽으로", "shoulder_yaw", "sy"),
    ("팔꿈치 90°", "다시 팔꿈치 90°, 손은 앞, 엄지가 위", "shoulder_yaw", "e90"),
    ("손등 위", "팔꿈치 90° 그대로, 손목만 돌려 손등이 위로", "elbow_roll", "er_back"),
    ("팔꿈치 90°", E90, "elbow_roll", "e90"),
    ("손바닥 위", "팔꿈치 90° 그대로, 손목만 돌려 손바닥이 위로", "elbow_roll", "er_palm"),
    ("팔꿈치 90°", E90, "elbow_roll", "e90"),
    ("차렷", None, "elbow_pitch", "rest"),
]
H_START = """팔 자세 기록 (읽기 전용 — 모터에 힘을 주지 않습니다)

준비
- 로봇 옆에, 로봇과 같은 방향을 보고 서세요. 내 왼팔 = 로봇 왼팔입니다.
- 로봇 팔은 힘이 없어서 놓으면 떨어집니다. 항상 손으로 받쳐 주세요.
- 한 팔씩 합니다. 쉬는 팔은 차렷으로 둡니다.

매 단계
  ① 내 팔로 그 자세를 만든다
  ② 로봇 팔을 내 팔과 똑같이 맞춘다
  ③ space

키: space 기록 · r 한 단계 되돌리기 · f 경고 무시하고 기록 · q 끝내기
아무 키나 누르면 시작합니다."""


def h_steps():
    out = []
    for side, kor in (("left", "왼팔"), ("right", "오른팔")):
        for n, (title, text, joint, label) in enumerate(H_STEPS, 1):
            if text is None:
                text = "팔꿈치를 펴서 차렷. 이제 오른팔로 넘어갑니다" if side == "left" else "팔꿈치를 펴서 차렷"
            out.append({"side": side, "kor": kor, "n": n, "title": title, "text": text, "joint": joint,
                        "label": f"{side}_{n:02d}_{label}", "step_id": f"H-{side[0].upper()}-{n:02d}"})
    return out


def h_check(step, prev, cur, motors):
    """경고 문구 또는 None. prev/cur = 10개 모터 값(rad)."""
    if step["joint"] is None or prev is None:
        return None
    d = {k: abs(cur[k] - prev[k]) * 180 / math.pi for k in range(len(motors)) if cur[k] is not None and prev[k] is not None}
    idx = {(s, j): k for k, (s, j, _, _) in enumerate(motors)}
    t = idx[(step["side"], step["joint"])]
    tdev = motors[t][2]
    tname = f"{tdev}번({H_NAME[step['joint']]})"
    other = max((k for k in d if k != t), key=lambda k: d[k])
    if d[other] > d.get(t, 0.0):
        o = motors[other]
        return (f"⚠ 이번엔 {tname}이 바뀌어야 하는데 {o[2]}번({H_NAME[o[1]]})이 더 많이 바뀌었습니다. "
                f"로봇 팔을 다시 맞추고 space. (그래도 기록하려면 f)")
    if d.get(t, 0.0) < H_MIN_DEG:
        return (f"⚠ {tname}이 거의 안 바뀌었습니다 ({d.get(t, 0.0):.0f}°). "
                f"로봇 팔을 다시 맞추고 space. (그래도 기록하려면 f)")
    return None


def run_H(args, buses, motors, stem, steps=None, per_arm=None, on_done=None, start_text=H_START, redo_done=False) -> int:
    fields = [f"{s}_{j}" for s, j, _, _ in motors]
    snap_f = open(args.out_dir / f"{stem}_snap.csv", "w", newline="", encoding="utf-8")
    stream_f = open(args.out_dir / f"{stem}_stream.csv", "w", newline="", encoding="utf-8")
    snap = csv.writer(snap_f); stream = csv.writer(stream_f)
    snap.writerow(["t", "t_wall", "part", "step_id", "label", "status", "n"] + fields
                  + [f"{f}_std" for f in fields] + [f"{f}_nrot" for f in fields] + [f"{f}_raw" for f in fields])
    stream.writerow(["t", "step_id"] + fields)
    steps = steps or h_steps()
    per_arm = per_arm or len(H_STEPS)
    t0 = time.monotonic()
    i, recs, buf, msg = 0, [], [], ""
    done_msg = None   # recs: [(step 인덱스, 10개 값)]

    def mean_now():
        good = [r for r in buf[-4:] if all(v is not None for v in r)]
        if not good:
            return read_positions(motors), 1, [None] * len(motors)
        n = len(good)
        m = [sum(r[k] for r in good) / n for k in range(len(motors))]
        sd = [math.sqrt(sum((r[k] - m[k]) ** 2 for r in good) / n) for k in range(len(motors))]
        return m, n, sd

    def write(step, status, m, n, sd):
        enc = read_encoders(motors)
        snap.writerow([f"{time.monotonic()-t0:.2f}", f"{time.time():.3f}", step["step_id"][0], step["step_id"], step["label"], status, n]
                      + [f"{v:.5f}" if v is not None else "" for v in m]
                      + [f"{v:.5f}" if v is not None else "" for v in sd]
                      + [e[0] if e[0] is not None else "" for e in enc]
                      + [e[1] if e[1] is not None else "" for e in enc])
        snap_f.flush()

    try:
        with Keys() as keys:
            sys.stdout.write("\033[H\033[J" + start_text); sys.stdout.flush()
            while keys.get() is None:
                time.sleep(0.05)
            while True:
                pos = read_positions(motors)
                buf.append(pos); buf[:] = buf[-10:]
                cur_step = steps[i] if i < len(steps) else None
                stream.writerow([f"{time.monotonic()-t0:.3f}", cur_step["step_id"] if cur_step else "done"]
                                + [f"{v:.5f}" if v is not None else "" for v in pos])
                prev = recs[-1][1] if recs else None
                out = ["\033[H\033[J"]
                if cur_step is None:
                    if done_msg is None:
                        done_msg = on_done({steps[j]["label"]: v for j, v in recs}) if on_done else ""
                    out += ["끝났습니다. q로 종료하세요."] + ([done_msg] if done_msg else [])
                    side = "right"
                else:
                    st = cur_step; side = st["side"]
                    out += [f"[{st['kor']} {st['n']}/{per_arm}] {st['title']}",
                            f"  내 팔: {st['text']}",
                            "  로봇 팔을 똑같이 맞추고 space", ""]
                    if st["joint"] is None:
                        out.append("  (첫 기록 — 확인 없음)")
                    else:
                        dev = IDS[side][JOINTS.index(st["joint"])]
                        out.append(f"  이번에 바뀌어야 할 모터: {dev}번 ({H_NAME[st['joint']]})")
                out += ["", f"  {'모터':<18}{'지금 값':>8}   지난 기록부터 바뀐 양"]
                for k, (s_, j, dev, _) in enumerate(motors):
                    if side != "both" and s_ != side:
                        continue
                    mark = "◀" if cur_step and cur_step["joint"] == j else " "
                    dd = (f"{(pos[k] - prev[k]) * 180 / math.pi:+6.0f}°"
                          if prev is not None and pos[k] is not None and prev[k] is not None else "     —")
                    out.append(f"  {dev:>2}번 {H_NAME[j]:<10} {mark}  {fmt(pos[k], 8)}      {dd}")
                out += ["", msg]
                sys.stdout.write("\n".join(out)); sys.stdout.flush()

                key = keys.get()
                if key in (" ", "f") and cur_step is not None:
                    m, n, sd = mean_now()
                    warn = None if key == "f" else h_check(cur_step, prev, m, motors)
                    if warn:
                        msg = warn
                    else:
                        write(cur_step, "forced" if key == "f" else "ok", m, n, sd)
                        recs.append((i, m)); i += 1
                        msg = "✔ 기록했습니다."
                elif key == "r" and recs:
                    j_, _ = recs.pop(); i = j_
                    if redo_done:
                        done_msg = None
                    snap.writerow([f"{time.monotonic()-t0:.2f}", f"{time.time():.3f}", steps[i]["step_id"][0], steps[i]["step_id"],
                                   steps[i]["label"], "undo", 0] + [""] * (4 * len(motors)))
                    snap_f.flush()
                    msg = "↺ 한 단계 되돌렸습니다. 이 단계를 다시 하세요."
                elif key == "q":
                    break
                time.sleep(0.08)
    except KeyboardInterrupt:
        pass
    finally:
        snap_f.close(); stream_f.close()
        for b in buses.values():
            b.stop()
        print(f"\n\n저장: {args.out_dir}/{stem}_{{meta.json,snap.csv,stream.csv}}")
    return 0



# --------------------------------------------------------------------------- R: 로봇을 켤 때마다 기준값 잡기 (pos_retarget.RobotRef)

R_STEPS = [
    ("차렷", "차렷. 팔꿈치 쭉 펴고, 손바닥은 허벅지, 엄지는 앞", None, "rest"),
    ("팔꿈치 90°", "위팔은 그대로, 팔꿈치만 90° 굽힌다. 엄지가 위, 손은 정면", "elbow_pitch", "e90"),
]
WINDOW = 2 * math.pi / 15          # 모터 한 바퀴 = 관절 24° (REPORT_18 §2)


# --------------------------------------------------------------------------- M: 만세용 기구 한계 (2026-09-30, REPORT_29)
# 읽기 전용. 모터 힘 없는 상태에서 손으로 팔을 들어 올려 "더 못 가는 곳" 을 잰다. 한 번에 한 관절.
M_STOP = "멈출 때: 뻑뻑해진다 · 선이 당긴다 · 어디에 닿는다 · 소리가 난다 — 힘으로 밀지 않는다"
M_STEPS = [
    ("차렷", "차렷. 팔꿈치 쭉 펴고, 손바닥은 허벅지", None, "rest"),
    ("앞으로 끝까지", "팔꿈치 쭉 편 채로 팔 전체를 앞으로 천천히 들어 올린다 (머리 위까지 가도 된다). "
     "멈춘 그 자리에서 space.  " + M_STOP, "shoulder_pitch", "sp_max"),
    ("차렷", "천천히 내려 차렷", "shoulder_pitch", "rest"),
    ("옆으로 끝까지", "팔꿈치 쭉 편 채로 팔 전체를 옆으로 천천히 들어 올린다 (머리 위까지 가도 된다). "
     "멈춘 그 자리에서 space.  " + M_STOP, "shoulder_roll", "sr_max"),
    ("차렷", "천천히 내려 차렷", "shoulder_roll", "rest"),
]


# --------------------------------------------------------------------------- Y: 어깨 비틀기(5·6번) 기구 끝 (2026-10-02, REPORT_36)
# 읽기 전용. 힘 뺀 상태에서 팔꿈치 90° 로 위팔을 안쪽(손이 배 쪽)·바깥쪽 끝까지 돌려 "더 못 가는 곳" 을 잰다.
# 기준 = "팔꿈치 90°, 아래팔 정면" (IK 의 어깨 비틀기 0). 안쪽 끝은 위팔을 몸에 붙인 채 / 앞으로 약 30° 든 채 두 번
# (붙인 채로는 아래팔이 배에 먼저 닿을 수 있다 — 그러면 그게 멈춘 이유다).
Y_STOP = M_STOP + " · 배·옆구리에 닿으면 거기서 멈춘다 (멈춘 이유를 기억해 두었다가 알려 주세요)"
Y_STEPS = [
    ("정면 ①", "위팔 몸에 붙이고 팔꿈치 90°, 아래팔이 정면, 엄지가 위. 받친 채 space", "shoulder_yaw", "front1"),
    ("안쪽 끝 (위팔 붙임)", "위팔은 몸에 붙인 채, 팔꿈치 90° 그대로 손을 배 쪽으로 (문 닫듯) 천천히 끝까지. "
     "멈춘 그 자리에서 space.  " + Y_STOP, "shoulder_yaw", "in_rest"),
    ("정면 ②", "다시 아래팔 정면, 엄지 위. space", "shoulder_yaw", "front2"),
    ("안쪽 끝 (위팔 앞 30°)", "위팔을 앞으로 조금(약 30°, 주먹 하나 반쯤) 든 채, 팔꿈치 90° 그대로 손을 몸 앞 가운데 쪽으로 천천히 끝까지. "
     "멈춘 그 자리에서 space.  " + Y_STOP, "shoulder_yaw", "in_fwd30"),
    ("정면 ③", "위팔을 몸에 붙이고 다시 아래팔 정면, 엄지 위. space", "shoulder_yaw", "front3"),
    ("바깥쪽 끝", "위팔은 몸에 붙인 채, 팔꿈치 90° 그대로 손을 바깥쪽으로 (문 열듯) 천천히 끝까지. "
     "멈춘 그 자리에서 space.  " + Y_STOP, "shoulder_yaw", "out_rest"),
    ("차렷", "천천히 팔을 내려 차렷", None, "rest"),
]


def y_steps():
    out = []
    for side, kor in (("left", "왼팔"), ("right", "오른팔")):
        for n, (title, text, joint, label) in enumerate(Y_STEPS, 1):
            out.append({"side": side, "kor": kor, "n": n, "title": title, "text": text, "joint": joint,
                        "label": f"{side}_{n:02d}_{label}", "step_id": f"Y-{side[0].upper()}-{n:02d}"})
    return out


def m_steps():
    out = []
    for side, kor in (("left", "왼팔"), ("right", "오른팔")):
        for n, (title, text, joint, label) in enumerate(M_STEPS, 1):
            out.append({"side": side, "kor": kor, "n": n, "title": title, "text": text, "joint": joint,
                        "label": f"{side}_{n:02d}_{label}", "step_id": f"M-{side[0].upper()}-{n:02d}"})
    return out


def r_steps():
    out = []
    for side, kor in (("left", "왼팔"), ("right", "오른팔")):
        for n, (title, text, joint, label) in enumerate(R_STEPS, 1):
            out.append({"side": side, "kor": kor, "n": n, "title": title, "text": text, "joint": joint,
                        "label": f"{side}_{label}", "step_id": f"R-{side[0].upper()}-{n}"})
    return out


def make_r_done(motors, out_dir: Path, ts: str):
    def done(vals):
        ref = {"time": datetime.now().isoformat(timespec="seconds"), "rest": {}, "e90": {}}
        for k, (s_, j, dev, _) in enumerate(motors):
            ref["rest"].setdefault(s_, {})[j] = vals[f"{s_}_rest"][k]
            ref["e90"].setdefault(s_, {})[j] = vals[f"{s_}_e90"][k]
        prev = sorted(out_dir.glob("robot_ref_*.json"))
        path = out_dir / f"robot_ref_{ts}.json"
        path.write_text(json.dumps(ref, ensure_ascii=False, indent=1), encoding="utf-8")
        lines = [f"기준값 저장: {path.name}"]
        if prev:
            old = json.loads(prev[-1].read_text(encoding="utf-8"))
            jumps = []
            for pose, name in (("rest", "차렷"), ("e90", "팔꿈치 90°")):
                for s_, jd in ref[pose].items():
                    for j, v in jd.items():
                        d = v - old[pose][s_][j]
                        n = round(d / WINDOW)
                        if n != 0 and abs(d - n * WINDOW) < math.radians(8):
                            dev = IDS[s_][JOINTS.index(j)]
                            jumps.append(f"{dev}번 {n * 24:+d}° ({name})")
            lines.append("지난번 대비 영점 점프: " + (", ".join(sorted(set(jumps))) if jumps else "없음")
                         + f"  (비교: {prev[-1].name})")
        return "\n".join(lines)
    return done

# --------------------------------------------------------------------------- Q·M·P: 주차 자세 기준값 (2026-11-02, 작업 지시 11-b)
# 판이 있으면 차렷(R)을 잡을 수 없다. 모터 한 바퀴 = 관절 24° 이고 그 안의 값(raw)은 전원을 꺼도 같으므로 (REPORT_table_a §6),
# 판을 뗀 상태에서 한 번 잰 rest·e90 (마스터) 에 "주차 자세 값의 차이를 24° 배수로 고른 것" 만 더하면 그 전원의 rest·e90 이 된다.
# 주차 자세의 손 오차는 칸을 고를 때만 쓰이고 결과에 섞이지 않는다.
MASTER_NAME = "robot_master.json"
PARK_TOL_DEG = 8.0        # 기본 허용 폭 (--park-tol-deg, Q 결과로 바꾼다)
PARK_EDGE_DEG = 10.0      # 칸 경계 (±12°) 근처: 이 값 이상이면 --park-tol-deg 와 상관없이 거부
PARK_POSE = "위팔을 몸통에 붙이고, 그리퍼를 판 위에 올려 둔다"
QMP_START = """주차 자세 기록 (읽기 전용 — 모터에 힘을 주지 않습니다)

- 주차 자세 = 양팔 """ + PARK_POSE + """.
- 로봇 팔은 힘이 없어서 판에서 들면 떨어집니다. 들 때는 손으로 받쳐 주세요.

키: space 기록 · r 한 단계 되돌리기 · f 경고 무시하고 기록 · q 끝내기
아무 키나 누르면 시작합니다."""


def _both_step(n, total, title, text, label, part):
    return {"side": "both", "kor": "양팔", "n": n, "title": title, "text": text, "joint": None,
            "label": label, "step_id": f"{part}-{n:02d}"}


def q_steps(repeat: int = 3):
    """같은 전원에서 주차 자세를 repeat 번. 사이에 팔을 한 번 들어 다시 놓는다 (같은 자세를 두 번 재지 않게)."""
    out, n = [], 0
    for k in range(1, repeat + 1):
        n += 1
        out.append(_both_step(n, 0, f"주차 {k}/{repeat}", f"양팔 {PARK_POSE}. 가만히 놓고 space", f"park_{k}", "Q"))
        if k < repeat:
            n += 1
            out.append(_both_step(n, 0, "한 번 들기", "양팔을 판에서 들어 위팔도 몸통에서 뗀 채 받치고 space", f"away_{k}", "Q"))
    return out


def m2_steps():
    """--part M: 판을 뗀 상태에서 R 과 같은 rest·e90 (팔꿈치는 멈춤점까지) → 판을 붙이고 주차 자세."""
    out = []
    for side, kor in (("left", "왼팔"), ("right", "오른팔")):
        out.append({"side": side, "kor": kor, "n": 1, "title": "차렷", "joint": None, "label": f"{side}_rest",
                    "text": "판을 뗀 상태. 차렷, 손바닥은 허벅지, 엄지는 앞. 팔꿈치는 펴지는 멈춤점까지 살짝 눌러서",
                    "step_id": f"M-{side[0].upper()}-1"})
        out.append({"side": side, "kor": kor, "n": 2, "title": "팔꿈치 90°", "joint": "elbow_pitch", "label": f"{side}_e90",
                    "text": "위팔은 그대로, 팔꿈치만 90° 굽힌다. 엄지가 위, 손은 정면", "step_id": f"M-{side[0].upper()}-2"})
    out.append(_both_step(3, 0, "판 붙이고 주차", f"🔴 전원을 끄지 마세요. 판을 붙인 뒤 양팔 {PARK_POSE}. 가만히 놓고 space", "park", "M"))
    return out


def p_steps():
    return [_both_step(1, 0, "주차", f"양팔 {PARK_POSE}. 가만히 놓고 space", "park", "P")]


def read_snap(path: Path) -> dict:
    """snap.csv → {라벨: {"pos"|"nrot"|"raw": {side: {joint: 값}}}}. undo 는 그 라벨을 지우고, 같은 라벨이면 마지막 기록을 쓴다."""
    out: dict = {}
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r["status"] == "undo":
                out.pop(r["label"], None)
            elif r["status"] in ("ok", "forced"):
                d = {"pos": {}, "nrot": {}, "raw": {}}
                for side in ("left", "right"):
                    for key in d:
                        d[key][side] = {}
                    for j in JOINTS:
                        k = f"{side}_{j}"
                        v = r.get(k, "")
                        d["pos"][side][j] = float(v) if v not in ("", None) else None
                        d["nrot"][side][j] = int(r[f"{k}_nrot"]) if r.get(f"{k}_nrot") else None
                        d["raw"][side][j] = int(r[f"{k}_raw"]) if r.get(f"{k}_raw") else None
                out[r["label"]] = d
    return out


def q_summary(parks, aways=()):
    """parks = [{side: {joint: rad}}, …]. 관절별 흩어짐 (폭, 평균에서 가장 먼 값, °) + 다음 번 허용 폭 제안."""
    rows, worst = {}, 0.0
    for side in ("left", "right"):
        rows[side] = {}
        for j in JOINTS:
            v = [p[side][j] * 180 / math.pi for p in parks]
            m = sum(v) / len(v)
            rng = max(v) - min(v)
            rows[side][j] = {"values_deg": [round(x, 3) for x in v], "range_deg": round(rng, 3), "max_dev_deg": round(max(abs(x - m) for x in v), 3)}
            worst = max(worst, rng)
    moved = None
    if aways:                       # 사이에 정말 움직였나: 들기 ↔ 앞뒤 주차 자세에서 어느 관절이든 10° 넘게
        moved = all(max(abs(a[s][j] - p[s][j]) * 180 / math.pi for s in ("left", "right") for j in JOINTS) >= 10.0
                    for a, p in zip(aways, parks))
    # 두 번 주차한 값의 차이가 칸을 고르는 잣대라 폭을 그대로 쓰고, 재지 않은 흩어짐 몫으로 2° 를 더한다
    suggest = round(worst + 2.0, 1)
    return {"joints": rows, "worst_range_deg": round(worst, 3), "moved_between": moved, "suggest_tol_deg": min(suggest, PARK_EDGE_DEG - 0.1),
            "usable": suggest < PARK_EDGE_DEG}


def park_cells(master: dict, park: dict, tol_deg: float = PARK_TOL_DEG) -> dict:
    """관절마다 k = round((지금 주차 − 마스터 주차) / 24°), 나머지 = 손 오차. 허용 폭·칸 경계를 넘거나 값이 없으면 bad."""
    cells: dict = {}
    bad = []
    for side in ("left", "right"):
        cells[side] = {}
        for j in JOINTS:
            now, m0 = park[side][j], master["park"][side][j]
            dev = IDS[side][JOINTS.index(j)]
            if now is None or m0 is None or not (math.isfinite(now) and math.isfinite(m0)):
                cells[side][j] = {"k": None, "resid_deg": None}
                bad.append({"side": side, "joint": j, "device": dev, "why": "값 없음", "resid_deg": None})
                continue
            d = now - m0
            k = round(d / WINDOW)
            res = (d - k * WINDOW) * 180 / math.pi
            cells[side][j] = {"k": k, "resid_deg": round(res, 3)}
            if abs(res) >= PARK_EDGE_DEG:
                bad.append({"side": side, "joint": j, "device": dev, "why": f"칸 경계 근처 (±{PARK_EDGE_DEG:.0f}°)", "resid_deg": round(res, 2)})
            elif abs(res) > tol_deg:
                bad.append({"side": side, "joint": j, "device": dev, "why": f"허용 폭 ±{tol_deg:g}° 넘음", "resid_deg": round(res, 2)})
    return {"cells": cells, "bad": bad, "tol_deg": tol_deg, "edge_deg": PARK_EDGE_DEG}


def refs_from_master(master: dict, cells: dict, when: str) -> dict:
    """make_r_done 과 같은 형식 (time, rest, e90). 값 = 마스터 값 + k × 24°."""
    ref = {"time": when, "rest": {}, "e90": {}}
    for pose in ("rest", "e90"):
        for side in ("left", "right"):
            ref[pose][side] = {j: master[pose][side][j] + cells[side][j]["k"] * WINDOW for j in JOINTS}
    return ref


def _name(side, j, dev):
    return f"{dev}번 {'왼' if side == 'left' else '오른'} {H_NAME[j]}"


def make_q_done(snap_path: Path, out_dir: Path, ts: str, repeat: int):
    def done(_vals):
        rec = read_snap(snap_path)
        parks = [rec[f"park_{k}"]["pos"] for k in range(1, repeat + 1)]
        aways = [rec[f"away_{k}"]["pos"] for k in range(1, repeat)]
        s = q_summary(parks, aways)
        s.update(time=datetime.now().isoformat(timespec="seconds"), repeat=repeat, snap=snap_path.name)
        path = out_dir / f"park_repeat_{ts}.json"
        path.write_text(json.dumps(s, ensure_ascii=False, indent=1), encoding="utf-8")
        lines = [f"저장: {path.name}", f"  {'관절':<18}{'폭 왼':>8}{'폭 오른':>8}  (°, 같은 전원에서 {repeat}번)"]
        for j in JOINTS:
            lines.append(f"  {H_NAME[j]:<16}{s['joints']['left'][j]['range_deg']:8.2f}{s['joints']['right'][j]['range_deg']:8.2f}")
        lines.append(f"가장 큰 폭 {s['worst_range_deg']:.2f}° → --part P 허용 폭 제안 --park-tol-deg {s['suggest_tol_deg']:g}")
        if not s["usable"]:
            lines.append(f"⚠ 흩어짐이 칸 경계 ±{PARK_EDGE_DEG:.0f}° 에 가깝다. 주차 자세를 더 일정하게 잡을 방법을 먼저 찾으세요")
        if s["moved_between"] is False:
            lines.append("⚠ 사이에 팔을 거의 안 움직인 것 같다. 이 결과는 작게 나온다 — 팔을 들었다 놓으며 다시 하세요")
        return "\n".join(lines)
    return done


def make_m_done(snap_path: Path, out_dir: Path, ts: str):
    def done(_vals):
        rec = read_snap(snap_path)
        m = {"time": datetime.now().isoformat(timespec="seconds"), "snap": snap_path.name, "rest": {}, "e90": {}, "park": {}, "enc": {}}
        for side in ("left", "right"):
            m["rest"][side] = dict(rec[f"{side}_rest"]["pos"][side])
            m["e90"][side] = dict(rec[f"{side}_e90"]["pos"][side])
            m["park"][side] = dict(rec["park"]["pos"][side])
        for pose, lab in (("rest", "{}_rest"), ("e90", "{}_e90")):
            m["enc"][pose] = {s: {j: {"nrot": rec[lab.format(s)]["nrot"][s][j], "raw": rec[lab.format(s)]["raw"][s][j]} for j in JOINTS}
                              for s in ("left", "right")}
        m["enc"]["park"] = {s: {j: {"nrot": rec["park"]["nrot"][s][j], "raw": rec["park"]["raw"][s][j]} for j in JOINTS} for s in ("left", "right")}
        path = out_dir / MASTER_NAME
        lines = []
        if path.exists():
            old = out_dir / f"robot_master_old_{ts}.json"
            path.replace(old)
            lines.append(f"이전 마스터는 {old.name} 로 옮겼다")
        path.write_text(json.dumps(m, ensure_ascii=False, indent=1), encoding="utf-8")
        lines.append(f"마스터 저장: {path.name}")
        for s in ("left", "right"):
            d = (m["e90"][s]["elbow_pitch"] - m["rest"][s]["elbow_pitch"]) * 180 / math.pi
            lines.append(f"  {'왼' if s == 'left' else '오른'} 팔꿈치 e90 − 차렷 = {d:+.1f}°" + ("" if 70 <= abs(d) <= 110 else "  ⚠ 90° 와 많이 다르다. r 로 다시 재세요"))
        lines.append("이제 전원을 끄지 말고 --part P 를 돌려 이 전원의 기준값을 만드세요")
        return "\n".join(lines)
    return done


def make_p_done(snap_path: Path, out_dir: Path, ts: str, master_path: Path, tol_deg: float):
    def done(_vals):
        rec = read_snap(snap_path)
        master = json.loads(master_path.read_text(encoding="utf-8"))
        chk = park_cells(master, rec["park"]["pos"], tol_deg)
        chk.update(time=datetime.now().isoformat(timespec="seconds"), master=master_path.name, master_time=master.get("time"), snap=snap_path.name)
        (out_dir / f"park_check_{ts}.json").write_text(json.dumps(chk, ensure_ascii=False, indent=1), encoding="utf-8")
        tab = [f"  {'관절':<16}{'왼 차이':>9}{'칸':>5}{'오른 차이':>10}{'칸':>5}  (°, 마스터 주차 대비)"]
        for j in JOINTS:
            a, b = chk["cells"]["left"][j], chk["cells"]["right"][j]
            f = lambda c: (f"{c['resid_deg']:+9.2f}{c['k']:+5d}" if c["k"] is not None else f"{'?':>9}{'?':>5}")
            tab.append(f"  {H_NAME[j]:<14}{f(a)} {f(b)}")
        if chk["bad"]:
            return "\n".join(["🔴 기준값을 만들지 않았다 — 칸을 믿을 수 없는 관절:"]
                             + [f"  {_name(b['side'], b['joint'], b['device'])}: {b['why']}"
                                + (f" ({b['resid_deg']:+.1f}°)" if b["resid_deg"] is not None else "") for b in chk["bad"]]
                             + tab + ["주차 자세를 다시 잡고 r 을 누른 뒤 다시 space (또는 q 로 끝내고 다시 실행)"])
        ref = refs_from_master(master, chk["cells"], datetime.now().isoformat(timespec="seconds"))
        path = out_dir / f"robot_ref_{ts}.json"
        path.write_text(json.dumps(ref, ensure_ascii=False, indent=1), encoding="utf-8")
        return "\n".join([f"기준값 저장: {path.name}  (마스터 {master.get('time')}, 허용 ±{tol_deg:g}°)"] + tab)
    return done


# --------------------------------------------------------------------------- G: 그리퍼 맞닿음 · 몸통 쪽 끝 (2026-10-13, REPORT_54)
# 읽기 전용. 모델(URDF 손 원기둥)과 실제 손 간격을 대조한다. 혼자 두 팔을 잡고 하므로 키 대신
# "모든 관절이 G_STILL_SEC 초 동안 G_STILL_DEG 안에서 가만히" 면 저절로 기록한다 (space 도 된다).
# 같은 자세를 두 번 연달아 기록하지 않도록, 직전 기록에서 어느 관절이든 G_MOVE_DEG 넘게 움직인 뒤에만 자동 기록한다.
G_STILL_SEC = 2.0
G_STILL_DEG = 0.5
G_MOVE_DEG = 5.0
G_SHAKE_SEC = 1.0      # --manual: space 를 누른 순간 앞 1초 동안
G_SHAKE_DEG = 1.5      # 어느 관절이든 이만큼 넘게 움직였으면 "흔들림" 으로 거절
G_STEPS = [
    ("both", "가슴 높이 ①", "두 팔을 들어 두 그리퍼 끝이 몸 앞 가슴 높이에서 '막' 맞닿게 (살짝 닿을 뿐, 누르지 않는다). "
     "그대로 2초 가만히 → 삑", "chest_1"),
    ("both", "가슴 높이 ②", "팔을 한 번 내렸다가, 다시 가슴 높이에서 그리퍼 끝이 막 맞닿게. 2초 가만히 → 삑", "chest_2"),
    ("both", "배 높이 ①", "두 그리퍼 끝이 배(배꼽) 높이에서 막 맞닿게. 2초 가만히 → 삑", "belly_1"),
    ("both", "배 높이 ②", "팔을 한 번 내렸다가, 다시 배 높이에서 막 맞닿게. 2초 가만히 → 삑", "belly_2"),
    ("both", "컵 붓기", "양쪽 그리퍼에 컵을 끼우고 두 컵 입구가 맞닿게 (붓는 자세). 컵이 없으면 s 로 건너뜀. 2초 가만히 → 삑",
     "cup"),
    ("left", "왼팔 안쪽 끝", "오른팔은 내려 둔다. 왼팔 위팔은 몸에 붙이고 팔꿈치 90°, 손을 배 쪽으로 (문 닫듯) "
     "몸통 부품에 닿기 직전까지. 2초 가만히 → 삑", "in_left"),
    ("right", "오른팔 안쪽 끝", "왼팔은 내려 둔다. 오른팔 위팔은 몸에 붙이고 팔꿈치 90°, 손을 배 쪽으로 "
     "몸통 부품에 닿기 직전까지. 2초 가만히 → 삑", "in_right"),
]
G_START = """그리퍼 맞닿음 기록 (읽기 전용 — 모터에 힘을 주지 않습니다)

- 로봇 팔은 힘이 없어서 놓으면 떨어집니다. 기록이 끝날 때까지 손으로 받치고 있으세요.
- 자세를 잡고 2초 가만히 있으면 저절로 기록하고 삑 소리를 냅니다 (space 도 됩니다).
- 같은 자세가 두 번 기록되지 않게, 다음 기록은 팔을 한 번 움직인 뒤에만 됩니다.

키: space 지금 기록 · s 건너뛰기 · r 한 단계 되돌리기 · q 끝내기
아무 키나 누르면 시작합니다."""


G_START_MANUAL = """그리퍼 맞닿음 기록 — 손으로 기록 (--manual, 읽기 전용 — 모터에 힘을 주지 않습니다)

- 로봇 팔은 힘이 없어서 놓으면 떨어집니다. 받친 채로 하세요.
- 자세를 잡고 1초 가만히 둔 뒤 space. 그 1초 동안 크게 흔들렸으면 "흔들림 — 다시 space" 가 나옵니다.

키: space 기록 · s 건너뛰기 · r 한 단계 되돌리기 · q 끝내기
아무 키나 누르면 시작합니다."""


def g_steps():
    return [{"side": side, "kor": {"both": "양팔", "left": "왼팔", "right": "오른팔"}[side], "n": n, "title": title,
             "text": text, "joint": None, "label": label, "step_id": f"G-{n:02d}"}
            for n, (side, title, text, label) in enumerate(G_STEPS, 1)]


def run_G(args, buses, motors, stem) -> int:
    """--manual 이면 자동 기록을 끄고 space 로만 기록한다 (앞 1초 흔들림이 크면 거절)."""
    import socket
    manual = bool(getattr(args, "manual", False))
    fields = [f"{s}_{j}" for s, j, _, _ in motors]
    snap_f = open(args.out_dir / f"{stem}_snap.csv", "w", newline="", encoding="utf-8")
    stream_f = open(args.out_dir / f"{stem}_stream.csv", "w", newline="", encoding="utf-8")
    snap = csv.writer(snap_f); stream = csv.writer(stream_f)
    snap.writerow(["t", "t_wall", "part", "step_id", "label", "status", "n"] + fields
                  + [f"{f}_std" for f in fields] + [f"{f}_nrot" for f in fields] + [f"{f}_raw" for f in fields])
    stream.writerow(["t", "step_id"] + fields)
    cue = None
    if args.cue_sound:                                  # 서버 PC cue_player 로 삑 (선택)
        host, _, port = args.cue_sound.partition(":")
        cue = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (host, int(port or 5021)))

    def beep(sound):
        sys.stdout.write("\a"); sys.stdout.flush()      # 로봇 PC 터미널 벨
        if cue is not None:
            try:
                cue[0].sendto(json.dumps({"sound": sound, "phase": "G", "text": ""}).encode("utf-8"), cue[1])
            except OSError:
                pass

    steps = g_steps()
    t0 = time.monotonic()
    i, recs, hist, msg = 0, [], [], ""        # hist: (시각, 10개 값)
    acts = []                                  # 기록·건너뛰기 순서 (단계 번호, "rec"|"skip") — r 은 마지막 하나만 되돌린다

    def still_mean(sec=G_STILL_SEC, deg=G_STILL_DEG):
        """마지막 sec 초가 모두 deg 안이면 (평균, 개수, 표준편차), 아니면 None."""
        now = time.monotonic()
        win = [v for t, v in hist if now - t <= sec and all(x is not None for x in v)]
        if not hist or now - hist[0][0] < sec or len(win) < 3:
            return None
        for k in range(len(motors)):
            vals = [v[k] for v in win]
            if (max(vals) - min(vals)) * 180 / math.pi > deg:
                return None
        n = len(win)
        m = [sum(v[k] for v in win) / n for k in range(len(motors))]
        sd = [math.sqrt(sum((v[k] - m[k]) ** 2 for v in win) / n) for k in range(len(motors))]
        return m, n, sd

    def write(step, status, m, n, sd):
        enc = read_encoders(motors)
        snap.writerow([f"{time.monotonic()-t0:.2f}", f"{time.time():.3f}", "G", step["step_id"], step["label"], status, n]
                      + [f"{v:.5f}" if v is not None else "" for v in m]
                      + [f"{v:.5f}" if v is not None else "" for v in sd]
                      + [e[0] if e[0] is not None else "" for e in enc]
                      + [e[1] if e[1] is not None else "" for e in enc])
        snap_f.flush()

    try:
        with Keys() as keys:
            sys.stdout.write("\033[H\033[J" + (G_START_MANUAL if manual else G_START)); sys.stdout.flush()
            while keys.get() is None:
                time.sleep(0.05)
            start_pose = read_positions(motors)       # 시작 자세(보통 차렷)도 "직전 기록" 으로 — 바로 기록되지 않게
            beep("ready")
            while True:
                pos = read_positions(motors)
                now = time.monotonic()
                hist.append((now, pos)); hist[:] = [h for h in hist if now - h[0] <= G_STILL_SEC + 0.5]
                cur = steps[i] if i < len(steps) else None
                stream.writerow([f"{now-t0:.3f}", cur["step_id"] if cur else "done"]
                                + [f"{v:.5f}" if v is not None else "" for v in pos])
                prev = recs[-1][1] if recs else start_pose
                moved = prev is None or any(p is not None and q is not None and abs(p - q) * 180 / math.pi > G_MOVE_DEG
                                            for p, q in zip(pos, prev))
                out = ["\033[H\033[J"]
                if cur is None:
                    out += ["끝났습니다. q 로 종료하세요."]
                else:
                    if manual:
                        out += [f"▶▶ [{cur['n']}/{len(steps)}] {cur['kor']} {cur['title']}: {cur['text']}", "",
                                "  자세를 잡고 space (s 건너뛰기 · r 되돌리기 · q 끝)"]
                    else:
                        out += [f"[{cur['n']}/{len(steps)}] {cur['kor']} — {cur['title']}", f"  {cur['text']}", "",
                                "  " + ("가만히 2초면 기록합니다" if moved else "팔을 한 번 움직인 뒤 다시 자세를 잡으세요 (같은 자세 연속 기록 방지)")]
                out += ["", f"  {'모터':<18}{'지금 값':>8}"]
                for k, (s_, j, dev, _) in enumerate(motors):
                    out.append(f"  {dev:>2}번 {s_[0].upper()} {H_NAME[j]:<10}  {fmt(pos[k], 8)}")
                out += ["", msg]
                sys.stdout.write("\n".join(out)); sys.stdout.flush()

                key = keys.get()
                st = still_mean() if (cur is not None and moved and not manual) else None
                if manual and cur is not None and key == " ":
                    st = still_mean(G_SHAKE_SEC, G_SHAKE_DEG)
                    if st is None:                        # 앞 1초에 크게 움직였거나 아직 1초가 안 됨
                        msg = "✖ 흔들림 — 다시 space (팔을 1초 가만히 둔 뒤)"
                        beep("tick")
                        time.sleep(0.08)
                        continue
                    nxt = steps[i + 1]["title"] if i + 1 < len(steps) else None
                    write(cur, "ok", *st)
                    recs.append((i, st[0])); acts.append((i, "rec")); i += 1
                    beep("stop" if nxt else "done")
                    msg = f"✔ 기록 — 다음: {steps[i]['kor']} {nxt}" if nxt else "✔ 기록 — 모두 끝 (q)"
                elif cur is not None and not manual and (key == " " or st is not None):
                    m, n, sd = st if st is not None else (read_positions(motors), 1, [None] * len(motors))
                    write(cur, "auto" if key != " " else "ok", m, n, sd)
                    recs.append((i, m)); acts.append((i, "rec")); i += 1
                    beep("stop" if i < len(steps) else "done")
                    msg = f"✔ 기록: {cur['title']}" + ("" if i < len(steps) else " — 모두 끝")
                elif key == "s" and cur is not None:
                    write(cur, "skip", [None] * len(motors), 0, [None] * len(motors))
                    acts.append((i, "skip")); i += 1
                    msg = f"… 건너뜀: {cur['title']}" + (f" — 다음: {steps[i]['kor']} {steps[i]['title']}" if i < len(steps) else "")
                elif key == "r" and acts:
                    i, kind = acts.pop()
                    if kind == "rec":
                        recs.pop()
                    snap.writerow([f"{time.monotonic()-t0:.2f}", f"{time.time():.3f}", "G", steps[i]["step_id"],
                                   steps[i]["label"], "undo", 0] + [""] * (4 * len(motors)))
                    snap_f.flush()
                    msg = "↺ 한 단계 되돌렸습니다. 이 자세를 다시 하세요."
                elif key == "q":
                    break
                time.sleep(0.08)
    except KeyboardInterrupt:
        pass
    finally:
        snap_f.close(); stream_f.close()
        for b in buses.values():
            b.stop()
        print(f"\n\n저장: {args.out_dir}/{stem}_{{meta.json,snap.csv,stream.csv}}")
    return 0


# --------------------------------------------------------------------------- T: 집게 위치·방향 실측 (2026-11-12, 작업 지시 14-r)
# CAN 은 그대로 읽기만 (ReadOnlyBus). 그리퍼 (아두이노 직렬) 만 연다/닫는다. 계산·판단·출력은 tools/calib_tip.py.

def run_T(args, buses, motors, stem, ts) -> int:
    import calib_tip as CT          # 워치독 모델을 읽어 오므로 T 일 때만
    import gripper_link as GL
    ref_path = CT.W.newest_robot_ref(None)
    if ref_path is None:
        print("[에러] logs/robot_ref_*.json 이 없다 — 먼저 ~/run.sh P"); return 1
    ref = CT.W.PR.RobotRef.load(ref_path)
    names = [(s, j) for s, j, _, _ in motors]
    grip_deg = GL.grip_deg()
    gripper = CT.NoGripper() if args.no_gripper else GL.GripperLink(args.gripper_port, log=lambda s: None)
    fields = [f"{s}_{j}" for s, j in names]
    snap_f = open(args.out_dir / f"{stem}_snap.csv", "w", newline="", encoding="utf-8")
    snap = csv.writer(snap_f)
    snap.writerow(["t_wall", "part", "step_id", "label", "status", "n", "angle_deg"] + fields + [f"{f}_std_deg" for f in fields])

    def snap_row(d):
        st, rec = d["step"], d["rec"]
        snap.writerow([f"{time.time():.3f}", "T", st["id"], st["kind"], "angle" if d.get("angle_only") else "ok", rec["n"],
                       rec.get("angle", "")] + [f"{v:.5f}" for v in rec["pos"]] + [f"{v:.3f}" for v in rec["std_deg"]])
        snap_f.flush()
    print(f"robot_ref: {ref_path.name}  그리퍼: {'없음' if args.no_gripper else args.gripper_port}  "
          f"열림 {grip_deg[0]} 닫힘 {grip_deg[1]}  — 아무 키나 누르면 시작")
    try:
        with Keys() as keys:
            while keys.get() is None:
                time.sleep(0.05)
            CT.run_interactive(lambda: read_positions(motors), names, keys, ref, ref_path, gripper, grip_deg, ts=ts, snap=snap_row)
    except KeyboardInterrupt:
        pass
    finally:
        snap_f.close()
        gripper.close()
        for b in buses.values():
            b.stop()
        print(f"\n기록: {args.out_dir}/{stem}_{{meta.json,snap.csv}}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--part", choices=["A", "B", "C", "D", "T", "TW", "E", "H", "R", "M", "MAX", "Y", "G", "Q", "P", "all"],
                    help="절차 구간 (G = 그리퍼 맞닿음·몸통 쪽 끝, 2초 가만히면 자동 기록. Q = 주차 자세 반복 오차, "
                         "M = 마스터 기준 robot_master.json, P = 매 전원 주차 자세 → robot_ref. MAX = 예전 --part M 만세 기구 한계. "
                         "T = 집게 위치·방향 실측 (14-r, 그리퍼를 연다/닫는다), TW = 예전 T 비틀기 확인)")
    ap.add_argument("--gripper-port", default="/dev/ttyUSB0", help="T: 그리퍼 아두이노 직렬 (기본 /dev/ttyUSB0)")
    ap.add_argument("--no-gripper", action="store_true", help="T: 그리퍼를 쓰지 않는다 (사람이 손으로 열고 닫는다)")
    ap.add_argument("--repeat", type=int, default=3, help="Q: 주차 자세를 몇 번 잡나 (기본 3)")
    ap.add_argument("--park-tol-deg", type=float, default=PARK_TOL_DEG,
                    help=f"P: 마스터 주차 자세와의 허용 폭 (°, 기본 {PARK_TOL_DEG:g}, Q 결과로 정한다). "
                         f"±{PARK_EDGE_DEG:g}° 이상은 칸 경계 근처라 이 값과 상관없이 거부한다")
    ap.add_argument("--master", type=Path, default=None, help=f"P: 마스터 파일 (기본 --out-dir 의 {MASTER_NAME})")
    ap.add_argument("--cue-sound", default=None, metavar="IP:PORT",
                    help="G: 기록할 때 서버 PC cue_player 로도 삑 (없으면 로봇 PC 터미널 벨만)")
    ap.add_argument("--manual", action="store_true",
                    help="G: 자동 기록(2초 가만히)을 끄고 space 로만 기록. 누른 순간 앞 1초에 1.5° 넘게 흔들렸으면 거절")
    ap.add_argument("--out-dir", type=Path, default=LOGS)
    ap.add_argument("--tag", default="", help="파일 이름에 붙일 표시 (예: D 의 before / after)")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        return self_test()
    if not args.part:
        ap.error("--part 가 필요하다")
    if args.part == "Q" and args.repeat < 2:
        ap.error("--repeat 은 2 이상")
    master_path = args.master or (args.out_dir / MASTER_NAME)
    if args.part == "P":
        try:
            if "park" not in json.loads(master_path.read_text(encoding="utf-8")):
                print(f"[에러] {master_path} 에 주차 자세가 없다 — --part M 을 끝까지 하라"); return 1
        except (OSError, ValueError) as exc:
            print(f"[에러] 마스터를 못 읽음 ({master_path}): {exc} — 판을 떼고 --part M 부터"); return 1
    if recoil is None:
        print("[에러] recoil import 실패 — 로봇 PC 에서 실행하라"); return 1
    if not sys.stdin.isatty():
        print("[에러] 키 입력이 필요하다 — TTY 에서 실행하라"); return 1

    RO = make_readonly_bus_class()
    buses = {ch: RO(channel=ch, bitrate=1_000_000) for ch in sorted(set(CHANNELS.values()))}
    motors = motors_list(buses)

    # ---- 시작 점검 (읽기만)
    meta = {"time": datetime.now().isoformat(timespec="seconds"), "part": args.part, "tag": args.tag, "motors": {}}
    print("모터 설정 읽는 중 (읽기만 한다)...")
    bad = []
    for side, j, dev, bus in motors:
        m = {p: rd(bus, dev, pn, t, timeout=0.3) for p, pn, t in META_PARAMS}
        meta["motors"][f"{side}_{j}"] = {"device_id": dev, "channel": CHANNELS[side], **m}
        mode = m["mode"]
        mname = SAFE_MODES.get(mode, f"{mode}" if mode is not None else "응답 없음")
        print(f"  [{side[0].upper()} {KOR[j]:<8}] id {dev:2d}  모드 {mname:<9} offset {fmt(m['position_offset'])}  "
              f"기어 {fmt(m['gear_ratio'])}  상순서 {m['phase_order']}  토크한계 {fmt(m['torque_limit'])}  "
              f"회전수 {m['encoder_n_rotations']}  현재 {fmt(m['position_measured'])}")
        if mode not in SAFE_MODES:
            bad.append((side, j, dev, mname))
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = f"armpose_{ts}_{args.part}{('_' + args.tag) if args.tag else ''}"
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / f"{stem}_meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")
    if bad and args.part in ("H", "R", "M", "MAX", "G", "Q", "P", "T"):
        print("\n모터가 힘이 들어간 상태라 멈춥니다. 전원을 껐다 켜고 다시 실행하세요.")
        return 2
    if bad:
        print("\n🔴 IDLE/DISABLED 가 아니거나 응답이 없는 모터가 있다 — 이 도구는 모드를 바꾸지 않으므로 여기서 멈춘다:")
        for side, j, dev, mname in bad:
            print(f"   {side} {j} (id {dev}): {mname}")
        print("   토크가 걸린 모드라면 팔을 받친 채 모터 전원을 껐다 켜서 기본 모드로 돌아오게 한 뒤 다시 실행하라.")
        print(f"   (설정 기록은 저장됨: {stem}_meta.json)")
        return 2

    if args.part == "H":
        return run_H(args, buses, motors, stem)
    if args.part == "R":
        return run_H(args, buses, motors, stem, steps=r_steps(), per_arm=len(R_STEPS),
                     on_done=make_r_done(motors, args.out_dir, ts))
    if args.part == "MAX":
        return run_H(args, buses, motors, stem, steps=m_steps(), per_arm=len(M_STEPS))
    snap_path = args.out_dir / f"{stem}_snap.csv"
    if args.part == "Q":
        st = q_steps(args.repeat)
        return run_H(args, buses, motors, stem, steps=st, per_arm=len(st), on_done=make_q_done(snap_path, args.out_dir, ts, args.repeat),
                     start_text=QMP_START, redo_done=True)
    if args.part == "M":
        return run_H(args, buses, motors, stem, steps=m2_steps(), per_arm=3, on_done=make_m_done(snap_path, args.out_dir, ts),
                     start_text=QMP_START, redo_done=True)
    if args.part == "P":
        return run_H(args, buses, motors, stem, steps=p_steps(), per_arm=1,
                     on_done=make_p_done(snap_path, args.out_dir, ts, master_path, args.park_tol_deg), start_text=QMP_START, redo_done=True)
    if args.part == "Y":
        return run_H(args, buses, motors, stem, steps=y_steps(), per_arm=len(Y_STEPS))
    if args.part == "G":
        return run_G(args, buses, motors, stem)
    if args.part == "T":
        return run_T(args, buses, motors, stem, ts)
    steps = [s for p in (("A", "B", "C", "D") if args.part == "all" else (args.part,)) for s in PARTS[p]()]
    fields = [f"{s}_{j}" for s, j, _, _ in motors]
    snap_f = open(args.out_dir / f"{stem}_snap.csv", "w", newline="", encoding="utf-8")
    stream_f = open(args.out_dir / f"{stem}_stream.csv", "w", newline="", encoding="utf-8")
    snap = csv.writer(snap_f); stream = csv.writer(stream_f)
    snap.writerow(["t", "t_wall", "part", "step_id", "label", "status", "n"] + fields
                  + [f"{f}_std" for f in fields] + [f"{f}_nrot" for f in fields] + [f"{f}_raw" for f in fields])
    stream.writerow(["t", "step_id"] + fields)

    t0 = time.monotonic()
    i = 0
    history = []        # 기록한 snap 행의 step 인덱스 (r 로 되돌리기)
    lo = [None] * len(motors); hi = [None] * len(motors)
    buf = []
    msg = ""

    def record(status, label, step_id, part):
        samples = [r for r in buf[-4:] if all(v is not None for v in r)] or [read_positions(motors)]
        n = len(samples)
        mean = [sum(r[k] for r in samples) / n if all(r[k] is not None for r in samples) else None for k in range(len(motors))]
        std = [math.sqrt(sum((r[k] - mean[k]) ** 2 for r in samples) / n) if mean[k] is not None else None
               for k in range(len(motors))]
        enc = read_encoders(motors)
        snap.writerow([f"{time.monotonic()-t0:.2f}", f"{time.time():.3f}", part, step_id, label, status, n]
                      + [f"{v:.5f}" if v is not None else "" for v in mean]
                      + [f"{v:.5f}" if v is not None else "" for v in std]
                      + [e[0] if e[0] is not None else "" for e in enc]
                      + [e[1] if e[1] is not None else "" for e in enc])
        snap_f.flush()
        return mean

    try:
        with Keys() as keys:
            while True:
                pos = read_positions(motors)
                buf.append(pos); buf[:] = buf[-10:]
                t = time.monotonic() - t0
                cur = steps[i] if i < len(steps) else ("-", "done", "done", "모든 단계를 끝냈다. q 로 종료.")
                stream.writerow([f"{t:.3f}", cur[1]] + [f"{v:.5f}" if v is not None else "" for v in pos])
                for k, v in enumerate(pos):
                    if v is not None:
                        lo[k] = v if lo[k] is None or v < lo[k] else lo[k]
                        hi[k] = v if hi[k] is None or v > hi[k] else hi[k]

                out = ["\033[H\033[J",
                       f"arm_pose_logger (읽기 전용)  part={args.part}  단계 {min(i+1, len(steps))}/{len(steps)}  "
                       f"읽기요청 {sum(b.sent_reads for b in buses.values())}",
                       "", f"▶ {cur[3]}", ""]
                out.append(f"{'':16}{'현재':>8}{'min':>9}{'max':>9}")
                for k, (side, j, dev, _) in enumerate(motors):
                    out.append(f"[{side[0].upper()} {KOR[j]:<8}] {dev:2d} {fmt(pos[k], 8)} {fmt(lo[k], 8)} {fmt(hi[k], 8)}")
                out += ["", "space 기록·다음   r 되돌리기   s 건너뛰기   m 표시   q 종료", msg]
                sys.stdout.write("\n".join(out)); sys.stdout.flush()

                key = keys.get()
                if key == " " and i < len(steps):
                    record("ok", cur[2], cur[1], cur[0]); history.append(i); i += 1
                    msg = f"✔ 기록: {cur[1]} {cur[2]}"
                elif key == "s" and i < len(steps):
                    record("skip", cur[2], cur[1], cur[0]); history.append(i); i += 1
                    msg = f"… 건너뜀: {cur[1]}"
                elif key == "m":
                    record("mark", "mark", cur[1], cur[0]); msg = f"◆ 표시 기록 (단계 {cur[1]})"
                elif key == "r" and history:
                    i = history.pop()
                    snap.writerow([f"{time.monotonic()-t0:.2f}", f"{time.time():.3f}", steps[i][0], steps[i][1],
                                   steps[i][2], "undo", 0] + [""] * (4 * len(motors)))
                    snap_f.flush()
                    msg = f"↺ 되돌림: {steps[i][1]} 을 다시 한다 (앞 기록은 undo 로 표시됨)"
                elif key == "q":
                    break
                time.sleep(0.08)
    except KeyboardInterrupt:
        pass
    finally:
        snap_f.close(); stream_f.close()
        for b in buses.values():
            b.stop()
        print(f"\n\n저장: {args.out_dir}/{stem}_{{meta.json,snap.csv,stream.csv}}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
