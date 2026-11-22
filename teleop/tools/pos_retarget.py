#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""위치 기반 팔 변환 — 사람 관절 위치 → 로봇 관절각 (워치독과 별개 파일, 워치독은 아직 안 건드림).

근거: REPORT_17 (위치로 구한 팔 방향은 캘리브레이션과 무관하게 깨끗하다),
      REPORT_18 §10 (관절별 부호·기준 자세, 영점 24° 점프), ROBOT_JOINTS §2.1 (URDF 구조).

무엇을 하나
-----------
  어깨 앞뒤·옆·비틀기·팔꿈치  ← 사람 관절 **위치** (어깨·팔꿈치·손목 세 점)
      위팔 방향 u = LowerArm − UpperArm,  아래팔 방향 f = WristTwist − LowerArm
      → URDF 팔 구조로 역산 (해석해, 반복 계산 없음) → q_URDF
      → 모터값 = 모터 기준값 + (q_URDF − q_URDF@기준)          (REPORT_18 §10.4, k = +1)
  손목 비틀기                 ← 뼈 **회전** (LowerArm 대비 WristTwist 의 긴축 비틀림, 캘리브레이션 대비)
      → 모터값 = 모터 기준값(팔꿈치 90° 엄지 위) + 손목부호 × 엎침각     (손등 위로 = 왼 +, 오른 −)

로봇 쪽 "팔 방향"의 정의
  위팔 방향 = 어깨 비틀기(shoulder_yaw) 축,  아래팔 방향 = 손목(elbow_roll) 축.
  URDF 영점에서 두 축은 같은 방향이고, 팔꿈치(elbow_pitch) 축은 두 축에 수직이다 (자체 검사에서 확인).
  어깨 옆 −15°(왼 −0.262 / 오른 +0.262) 에서 위팔 축이 정확히 수직이 된다 = 차렷 (9/22 로봇 정면 사진: 위팔 수직).

팔을 폈을 때 어깨 비틀기
  세 점이 일직선이면 "팔이 접히는 면"이 없어 어깨 비틀기가 정해지지 않는다.
  Meta Quest 는 편 팔도 약 15° 굽은 것으로 보고한다 (REPORT_17 §3.2) → 굽힘 30° 미만이면 **직전 값 유지**,
  30~50° 사이는 **조금씩 따라감**, 50° 이상이면 **그대로 따라감** (ELBOW_HOLD_LO/HI).

좌표
  사람: 언리얼 트래킹 공간 (x 앞, y 오른쪽, z 위, cm). 캘리브레이션 때의 몸 틀(두 어깨 관절을 이은 선 + 수직)로 옮긴다.
  로봇: URDF base (x 앞, y 왼쪽, z 위). 몸 틀 → 로봇 틀은 y 부호만 뒤집는다.

시작 절차 (영점이 전원마다 24° 씩 뛸 수 있어서 — REPORT_18 §10.3)
  1) 로봇을 켤 때마다: `arm_pose_logger.py --part R` — 로봇 팔을 **차렷**, **팔꿈치 90° 엄지 위**로 맞추고 모터값 기록
     → logs/robot_ref_<시각>.json (이 파일의 `RobotRef` 가 모터 기준값이다)
  2) VR 캘리브레이션: 사람이 차렷 (손바닥은 허벅지, 엄지는 앞) → 몸 틀 + 손목 비틀기 0 을 잡는다 (`HumanCalib`)
  3) 매 프레임: `PosRetargeter.step(bones)` → 모터 목표값

    python3 pos_retarget.py --self-test
"""

from __future__ import annotations

import argparse
import json
import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

URDF = (Path.home() / "code_workspace/Berkeley-Humanoid-Lite-Assets-main/data/robots/berkeley_humanoid/"
        "berkeley_humanoid_lite/urdf/berkeley_humanoid_lite.urdf")
JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "elbow_roll")
PREFIX = {"left": "arm_left_", "right": "arm_right_"}
BONES = {"left": ("L_UpperArm", "L_LowerArm", "L_WristTwist"), "right": ("R_UpperArm", "R_LowerArm", "R_WristTwist")}
SIGMA = {"left": +1.0, "right": -1.0}          # 팔꿈치 굽힘 부호 (URDF = 실측, REPORT_18 §10.1)
WRIST_MOTOR_SIGN = {"left": +1.0, "right": -1.0}  # 손등 위로(엎침) 돌릴 때 모터 부호 (실측, REPORT_18 §10.1)
# 사람 쪽: 뼈 twist(+) 가 엎침(손등 위)인가. ✅ VR 로 확인됨 — 오른팔 9/22, 왼팔 9/23 (REPORT_21 §3).
HUMAN_PRONATION_SIGN = {"left": +1.0, "right": +1.0}
Q_REF_REST = {"left": {"shoulder_pitch": 0.0, "shoulder_roll": -0.261799, "elbow_pitch": 0.0},
              "right": {"shoulder_pitch": 0.0, "shoulder_roll": +0.261799, "elbow_pitch": 0.0}}
# 팔이 펴졌을 때 어깨 비틀기를 어떻게 할 것인가 (REPORT_26, REPORT_27)
#   C(hold, 현행) 직전 값 유지 · A(none) 계산값 그대로 · B(recenter) 기준값으로 천천히 복귀
#   D(bone) 뼈 긴축 회전으로 대신 추정 (굽힌 구간에서 계산값과의 차이를 계속 학습해 보정)
SY_POLICIES = ("hold", "none", "recenter", "bone")
ARM_TWIST_SIGN = {"left": +1.0, "right": -1.0}   # 뼈 긴축 회전 → 어깨 비틀기 부호 (9/19·9/23 녹화 적합)
BONE_OFFSET_ALPHA = 0.02                          # 굽힌 구간에서 오프셋을 배우는 속도 (프레임당)
RECENTER_RATE = 0.02
# 어깨 앞뒤·옆이 가질 수 있는 값의 창 (URDF 기준, 라디안). 만세(±180°)를 넘어 조금 여유. REPORT_29/30
BRANCH_PENALTY = 2.5   # 9/30 1.0 → 2.5 (REPORT_31: 1.0 이면 떨림 2° 재생에서 머리 위로 넘긴 해를 고른다)
# 🔴 REPORT_31: IK 가 로봇 관절 한계를 안다. 값 = 워치독 limits_from_robot_ref 와 같은 것 (차렷 기준 상대각, 도).
# 워치독은 자기 계산값을 limits_rel 로 넘긴다. 이 표는 오프라인 도구(검증·재생)용 기본값.
ROBOT_LIMITS_REL_DEG = {
    "left": {"shoulder_pitch": (-165.0, 33.25), "shoulder_roll": (-7.95, 154.0), "shoulder_yaw": (-54.0, 54.0)},
    "right": {"shoulder_pitch": (-45.4, 167.0), "shoulder_roll": (-155.0, 10.25), "shoulder_yaw": (-58.0, 54.0)},
}
# 🔴 어깨 비틀기 한계 (2026-10-03 사용자 승인, REPORT_36 §5). e90(팔꿈치 90°·아래팔 정면) 기준, 안쪽(손이 배 쪽) = 왼 +, 오른 −.
#   안쪽: 위팔을 앞으로 든 각(어깨 앞뒤, 차렷 대비)이 0° 이하 → 30°, 25° 이상 → 왼 54° / 오른 58°, 사이는 직선.
#   바깥쪽: 왼 54°, 오른 54°.   위 표의 shoulder_yaw 는 이 둘을 다 담는 겉 범위 (워치독 정적 한계와 같다).
#   측정: armpose_20261003_005119_Y — 안쪽 붙임 40/41°, 위팔 28/23° 들고 64/68°, 바깥 64/77° (안쪽은 전면 하드웨어에 닿음)
YAW_IN_AT_REST_DEG = 30.0
YAW_IN_RAISED_DEG = {"left": 54.0, "right": 58.0}
YAW_RAISE_FULL_DEG = 25.0
YAW_OUT_DEG = {"left": 54.0, "right": 54.0}
YAW_IN_SIGN = {"left": +1.0, "right": -1.0}


def upper_arm_forward_deg(side: str, sp_rel_deg: float) -> float:
    """어깨 앞뒤(차렷 대비, 도) → 위팔을 앞으로 든 각 (+ = 앞). 왼팔은 앞이 −, 오른팔은 앞이 + (REPORT_18)."""
    return -sp_rel_deg if side == "left" else sp_rel_deg


# REPORT_48 (2026-10-04 사용자 결정): 안쪽 한계를 위팔 든 각과 상관없이 직접 지정 (워치독 --yaw-in-limit-deg 왼 오른).
# None 이면 위 규칙 그대로. set_yaw_in_limit() 로만 바꾼다 — 범위 규칙·ROBOT_LIMITS_REL_DEG 를 함께 바꾼다.
YAW_IN_OVERRIDE_DEG: Optional[Dict[str, float]] = None


def set_yaw_in_limit(left_deg: Optional[float], right_deg: Optional[float] = None) -> None:
    """안쪽 한계 직접 지정 (도, 안쪽 + 크기). None 이면 원래 규칙으로 되돌린다.
    yaw_window_deg (IK·워치독 매 루프 자르기·hand 다시 맞춤이 쓰는 식) 와 ROBOT_LIMITS_REL_DEG 비틀기 안쪽 끝을 같이 바꾼다.
    워치독의 SAFE_YAW_ENVELOPE_DEG 는 워치독이 따로 바꾼다."""
    global YAW_IN_OVERRIDE_DEG
    if left_deg is None:
        YAW_IN_OVERRIDE_DEG = None
        ROBOT_LIMITS_REL_DEG["left"]["shoulder_yaw"] = (-YAW_OUT_DEG["left"], YAW_IN_RAISED_DEG["left"])
        ROBOT_LIMITS_REL_DEG["right"]["shoulder_yaw"] = (-YAW_IN_RAISED_DEG["right"], YAW_OUT_DEG["right"])
        return
    right_deg = left_deg if right_deg is None else right_deg
    YAW_IN_OVERRIDE_DEG = {"left": float(left_deg), "right": float(right_deg)}
    ROBOT_LIMITS_REL_DEG["left"]["shoulder_yaw"] = (-YAW_OUT_DEG["left"], float(left_deg))
    ROBOT_LIMITS_REL_DEG["right"]["shoulder_yaw"] = (-float(right_deg), YAW_OUT_DEG["right"])


def yaw_window_deg(side: str, fwd_deg: float) -> Tuple[float, float]:
    """위팔을 앞으로 든 각에 따른 어깨 비틀기 허용 범위 (e90 기준, 도) — (아래, 위). 직접 지정(REPORT_48)이 있으면 그 값."""
    a = min(max(fwd_deg / YAW_RAISE_FULL_DEG, 0.0), 1.0)
    inward = YAW_IN_AT_REST_DEG + a * (YAW_IN_RAISED_DEG[side] - YAW_IN_AT_REST_DEG)
    if YAW_IN_OVERRIDE_DEG is not None:
        inward = YAW_IN_OVERRIDE_DEG[side]
    out = YAW_OUT_DEG[side]
    return (-out, inward) if YAW_IN_SIGN[side] > 0 else (-inward, out)
# REPORT_35: "자세는 한 번 정하면 유지" — 옆 수평 축 KEEP_ZONE_DEG 안에서는 들어올 때의 어깨 앞뒤 ± KEEP_BAND_DEG 안에서만
# 움직이고, 그 띠 안 자세가 사람 팔과 KEEP_ERR_DEG 넘게 어긋날 때만 자세를 바꾼다.
KEEP_ZONE_DEG = 55.0
KEEP_ERR_DEG = 30.0
KEEP_BAND_DEG = 35.0     # 들어올 때 어깨 앞뒤 ± 이만큼 = "같은 자세" (check_2018 재생으로 25/35, 오차 30/40, 구역 45/55 비교 — REPORT_35)
SY_STATE_MARGIN = math.radians(5.0)   # 어깨 비틀기 상태를 한계 + 이만큼 안에 둔다 (±180° 넘김 방지, REPORT_35)
# REPORT_42 R1 (2026-10-03 승인): 위팔이 어깨 높이 아래인데 "머리 위로 넘긴 해"(차렷 대비 |앞뒤| > 90°)로 한 번에
# FLIP_HOLD_JUMP_DEG 넘게 뛰어 들어가려 하면 그 프레임은 어깨 앞뒤·옆을 직전 값으로 둔다 (사람 팔이 등 뒤 — 로봇이 갈 수 없는 곳).
# 옆 흔들기는 앞뒤가 연속으로 90° 를 지나 뜀이 작고, 앞으로 만세는 위팔이 수평 이상이라 걸리지 않는다 (기존 로그 전부 0).
FLIP_HOLD_JUMP_DEG = 45.0
# REPORT_47 (2026-10-04 승인, REPORT_46 안 A): hand 에서 어깨 옆이 몸 쪽 한계를 넘으려 하면 한계로 자르고, 남는 어깨 비틀기·
# 팔꿈치로 손목 관절을 손 목표에 다시 맞춘다 (비틀기 범위 규칙·팔꿈치 한계 안). 가우스-뉴턴 이만큼 반복.
HAND_REFIT_ITERS = 6
FLIP_HOLD_BELOW = True      # True: 위팔이 어깨 높이 아래(u_z < 0)일 때만
SWITCH_ERR_DEG = 30.0   # 지금 해가 한계에 막혀 사람 팔과 이만큼 넘게 어긋나면 그때만 다른 해로 (REPORT_31)
SP_HOLD_DEG = 20.0   # 위팔이 옆 수평 축에서 이 각 안이면 어깨 앞뒤를 멈춘다 (REPORT_31)
SP_FULL_DEG = 35.0   # 이 각 밖이면 어깨 앞뒤를 그대로 따라간다 (사이는 비율)
ELBOW_HOLD_LO = math.radians(30.0)
ELBOW_HOLD_HI = math.radians(50.0)
# REPORT_32/33 — 사람 입력(Meta 추정)이 믿기 어려운 곳을 거른다. 셋 다 PosRetargeter 인자로 켜고 끈다 (기본 켬).
WRIST_HOLD_LO = math.radians(40.0)     # 손목 A: 팔꿈치(머리 위 A 뒤 값)가 이보다 작으면 손목 유지 …
WRIST_HOLD_BAND = math.radians(15.0)   # … 이만큼 위에서 그대로 따라감 (사이는 비율). 차렷 팔꿈치+15° 가 더 크면 그쪽
OVERHEAD_LO = math.radians(100.0)      # 머리 위 A: 위팔 높이(아래에서 잰 각)가 이보다 높으면 팔꿈치를 덜 믿기 시작 …
OVERHEAD_HI = math.radians(130.0)      # … 이보다 높으면 들어가기 직전 값으로 완전히 유지
#   (check_2025 왼팔: 가짜 굽힘 85~96° 가 높이 121~128° 에서 시작 → 110/140 으로는 반 이상 따라갔다, REPORT_33 §2)
ELBOW_BASE_MAX = math.radians(45.0)    # 머리 위 B: 차렷 팔꿈치 값이 이보다 크면 믿지 않고 이 값으로 자른다
UP_UE = np.array([0.0, 0.0, 1.0])
# REPORT_38 손 위치 기준 방식 (--retarget hand, REPORT_37 안 2). 관절 한계·비틀기 규칙·충돌 방지는 pos 와 같은 것을 그대로 건다.
HAND_TOUCH_CM = 25.0       # 사람 두 손목 3차원 거리가 이 아래면 "손이 닿음" → 로봇은 최소 간격 (REPORT_39: 주먹 맞댐 22.8/24.0,
                           # 한 팔 동작 최소 29.7/35.1 cm — 2026-10-03 승인)
HAND_BLEND_CM = 5.0        # 문턱 ~ 문턱 + 이만큼 사이는 비율로 섞는다 (REPORT_39 C — 예전: 문턱 ~ 사람 어깨 폭)
HAND_TOUCH_MODES = ("3d", "lateral")   # 닿음 판정 거리: 3d = 두 손목 3차원 거리 (기본), lateral = 몸 틀 좌우 간격 (REPORT_38 방식)
HAND_MIN_GAP_CM = 14.0     # 로봇 양 손목 관절 좌우 간격 최소. 손 원기둥이 안쪽으로 5 cm 나와서 14 cm 일 때 가슴 앞 두 아래팔 ≈ 4 cm (충돌 방지 문턱)
HAND_SWIVEL_TAU = 0.15     # 팔꿈치 돌림(swivel) 을 사람 팔꿈치 쪽으로 따라가는 시간 상수 (초)
HAND_SWIVEL_PERP_CM = (3.0, 7.0)   # 사람 팔꿈치가 어깨–손목 선에서 이만큼(cm) 떨어져야 돌림을 믿는다 (사이는 비율)
HAND_MAX_SPEED = 1.0       # 로봇 손목 목표가 움직일 수 있는 최대 속도 (m/s) — 추적이 튀어도 로봇이 튀지 않게
# REPORT_56 G3 하이브리드 (2026-10-14 승인): 양손이 가까울 때만 "잡는 점 쌍 매핑" — 기본 끔 (끄면 아래 코드가 돌지 않는다)
#   사람 잡는 점 = 손목 + HAND_GRIP_OFFSET_CM · 아래팔 방향 (컨트롤러 앞끝, 19:02 녹화 "모으기" 에서 1.8/2.0 cm 가 되는 값)
#   로봇 잡는 점 = 손목 관절 + ROBOT_GRIP_M · 손 축 (CAD 집게 끝, 팔꿈치 축에서 25.6 cm)
#   사람 두 잡는 점 사이 ≤ near → 두 로봇 잡는 점의 가운데 = 지금 매핑 가운데, 두 점 사이 벡터 = 사람 것 1:1 (물체 폭 보존, 닿음 규칙 효과 없음)
#                        ≥ far  → 지금과 같음. 사이는 끝점을 비율로 섞는다 (§9: 끝점 = 손목 + w·13.3 cm 손 축, 직전 손 방향을 안 씀)
HAND_PAIR_NEAR_CM = 25.0
HAND_PAIR_FAR_CM = 33.0
HAND_GRIP_OFFSET_CM = 15.0
HAND_TABLE_MAX_LIFT = 0.30       # 작업 지시 11-b: 손 목표를 올리는 최대 (m)
HAND_TABLE_X_MIN = 0.14          # 목표를 올릴 때 손목을 이 x (m) 까지 앞으로 민다 (팔꿈치 90° 의 손목 12.3 cm 근처)
HAND_TABLE_X_RAMP = 0.05         # 이만큼 (m) 올리는 동안 다 민다
HAND_PAIR_CENTERS = ("grip", "robot-hand")   # 작업 지시 10-c: 쌍 매핑 가운데점 (HandTargeter pair_center)
ROBOT_GRIP_M = 0.133
HAND_GAP_GOAL_CM = 5.0     # 손 목표를 정할 때 두 아래팔(모델) 사이를 이만큼 이상으로 — 충돌 방지(4 cm, 비틀기·어깨 옆만 막음)
                           # 에 기대지 않게 1 cm 여유. 모자라면 양손 목표를 좌우로 벌린다 (충돌 방지 값은 그대로)


# --------------------------------------------------------------------------- 기하

def unit(v):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def rot(axis, ang):
    """축-각 → 회전행렬 (오른손 법칙)."""
    a = unit(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def ang(a, b):
    return math.acos(float(np.clip(np.dot(unit(a), unit(b)), -1, 1)))


def _origin(j):
    o = j.find("origin")
    xyz = np.array(o.get("xyz").split(), float)
    r, p, y = np.array(o.get("rpy").split(), float)
    R = rot([0, 0, 1], y) @ rot([0, 1, 0], p) @ rot([1, 0, 0], r)
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = xyz
    return M


class ArmModel:
    """URDF 팔 한쪽. 영점 자세에서 각 관절 축(base 틀)을 읽어 두고, 방향만 다룬다 (PoE 회전부)."""

    def __init__(self, side: str, urdf: Path = URDF):
        self.side = side
        root = ET.parse(urdf).getroot()
        J = {j.get("name"): j for j in root.findall("joint")}
        self._J = [J[PREFIX[side] + n + "_joint"] for n in JOINTS]
        M = np.eye(4)
        self.w = {}
        for n, j in zip(JOINTS, self._J):
            M = M @ _origin(j)
            self.w[n] = unit(M[:3, :3] @ np.array([0, 0, 1.0]))   # 모든 팔 관절이 axis 0 0 1
        self.lower = {n: float(j.find("limit").get("lower")) for n, j in zip(JOINTS, self._J)}
        self.upper = {n: float(j.find("limit").get("upper")) for n, j in zip(JOINTS, self._J)}

    # ---- 순기구학 (방향만)
    def fk_dirs(self, q: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        w = self.w
        R1 = rot(w["shoulder_pitch"], q["shoulder_pitch"]) @ rot(w["shoulder_roll"], q["shoulder_roll"])
        R3 = R1 @ rot(w["shoulder_yaw"], q["shoulder_yaw"])
        u = R1 @ w["shoulder_yaw"]
        f = R3 @ rot(w["elbow_pitch"], q["elbow_pitch"]) @ w["elbow_roll"]
        return u, f

    def forearm_segment(self, q: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        """base 틀에서 아래팔 선분 (팔꿈치, 손끝) [m]. 손끝 = 손목 관절 + 손 원기둥(URDF 충돌, 길이 0.12·중심 −0.01) 먼 끝.
        양팔이 같은 base 틀이라 두 팔 사이 거리를 바로 잴 수 있다 (REPORT_36 충돌 방지).
        REPORT_54 P1: 관절 원점을 매번 다시 읽지 않고 points() 의 저장값을 쓴다 (같은 계산, 약 5배 빠름 — 충돌 방지가 루프마다 여러 번 부른다)."""
        return self.segment_of(*self.points(q))

    def points(self, q: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """base 틀에서 (팔꿈치 관절, 손목 관절, 손목 축) [m] (REPORT_38 손 위치 방식). 관절 원점은 한 번만 읽어 둔다."""
        if not hasattr(self, "_O"):
            self._O = [_origin(j) for j in self._J]
        M = np.eye(4)
        el = None
        for n, O in zip(JOINTS, self._O):
            c_, s_ = math.cos(q[n]), math.sin(q[n])
            Q = np.array([[c_, -s_, 0, 0], [s_, c_, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]])   # z 축 회전 (= rot)
            M = M @ O @ Q
            if n == "elbow_pitch":
                el = M[:3, 3].copy()
        return el, M[:3, 3].copy(), M[:3, 2].copy()

    def jaw0(self, q: Dict[str, float]) -> np.ndarray:
        """작업 지시 12-c: 끝 프레임 y 축 (q 의 손목 0 이면 집게 벌어지는 축, robot_viz.fk 와 같다)."""
        self.points(q)
        M = np.eye(4)
        for n, O in zip(JOINTS, self._O):
            c_, s_ = math.cos(q[n]), math.sin(q[n])
            M = M @ O @ np.array([[c_, -s_, 0, 0], [s_, c_, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]])
        return M[:3, 1].copy()

    def segment_of(self, el: np.ndarray, wr: np.ndarray, ax: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """forearm_segment 와 같은 선분 (팔꿈치, 손끝) — points() 결과로."""
        ends = (wr + ax * (-HAND_BACK_M), wr + ax * HAND_TIP_M)
        return el, max(ends, key=lambda e: float(np.linalg.norm(e - el)))

    def shoulder_center(self) -> np.ndarray:
        """어깨 앞뒤 축과 어깨 옆 축이 만나는 점 (base 틀, m) — 어깨 세 축이 지나는 공 관절 중심."""
        M1 = _origin(self._J[0])
        M2 = M1 @ _origin(self._J[1])
        p1, d1 = M1[:3, 3], M1[:3, :3] @ np.array([0, 0, 1.0])
        p2, d2 = M2[:3, 3], M2[:3, :3] @ np.array([0, 0, 1.0])
        # 두 직선의 가장 가까운 두 점의 가운데
        r = p1 - p2
        a, b, c, d, e = d1 @ d1, d1 @ d2, d2 @ d2, d1 @ r, d2 @ r
        den = a * c - b * b
        s = (b * e - c * d) / den if abs(den) > 1e-12 else 0.0
        t = (a * e - b * d) / den if abs(den) > 1e-12 else 0.0
        return 0.5 * ((p1 + s * d1) + (p2 + t * d2))

    def q_from_dirs(self, u: np.ndarray, f: np.ndarray) -> Dict[str, float]:
        """방향 (u, f) → 관절값 하나 (상태 없음, 한계 무시). 위치 계산용 — 위팔 링크 자세는 두 해 어느 쪽이든 같다."""
        for sp, sr, _ in self.ik_shoulder_candidates(u):
            if math.isfinite(sp):
                break
        else:
            sp, sr = 0.0, 0.0
        sy, ep = self.ik_twist(sp, sr, u, f)
        return {"shoulder_pitch": sp, "shoulder_roll": sr, "shoulder_yaw": sy if math.isfinite(sy) else 0.0,
                "elbow_pitch": ep, "elbow_roll": 0.0}

    def fk_dirs_chain(self, q: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        """검사용: URDF 변환을 그대로 곱해서 얻은 비틀기·손목 축."""
        M = np.eye(4); out = {}
        for n, j in zip(JOINTS, self._J):
            Q = np.eye(4); Q[:3, :3] = rot([0, 0, 1], q[n])
            M = M @ _origin(j) @ Q
            out[n] = M[:3, 2].copy()
        return out["shoulder_yaw"], out["elbow_roll"]

    # ---- 역기구학 (해석해)
    def ik_shoulder(self, u: np.ndarray, sr_prev: float) -> Tuple[float, float, float]:
        """위팔 방향 u(로봇 틀) → (어깨 앞뒤, 어깨 옆, 어깨 앞뒤 신뢰도 0~1). 어깨 옆은 두 해 중 sr_prev 에 가까운 쪽."""
        a = self.w["shoulder_yaw"]
        # 어깨 옆 축은 영점에서 +X, 어깨 앞뒤 축은 ±Y (y 성분 보존) → v_y = u_y 로 어깨 옆이 정해진다
        assert abs(abs(self.w["shoulder_roll"][0]) - 1) < 1e-6 and abs(abs(self.w["shoulder_pitch"][1]) - 1) < 1e-6
        sx = self.w["shoulder_roll"][0]
        phi = math.atan2(a[2], a[1])
        c = math.acos(float(np.clip(u[1], -1, 1)))
        cands = [wrap(sx * (s * c - phi)) for s in (+1, -1)]
        best = min(cands, key=lambda t: abs(wrap(t - sr_prev)))
        # 🔴 이어 붙이기 (REPORT_29): ±180° 에서 값이 뒤집히지 않게 직전 값에 가장 가까운 연속값으로 둔다
        sr = sr_prev + wrap(best - sr_prev)
        v = rot(self.w["shoulder_roll"], sr) @ a
        sy_ = self.w["shoulder_pitch"][1]
        # 위팔이 옆으로 수평 근처면 어깨 앞뒤 축과 위팔이 거의 나란해서 어깨 앞뒤가 불안정하다.
        # 반환값 셋째 = 신뢰도 (0 = 수평 옆, 1 = 10° 이상 벗어남)
        conf = float(np.clip(abs(v[2]) / math.sin(math.radians(10)), 0, 1))
        g = 1.0 if v[2] >= 0 else -1.0
        sp = sy_ * math.atan2(u[0] * g, u[2] * g) if abs(v[2]) > 1e-9 else float("nan")
        return sp, sr, conf

    def ik_shoulder_candidates(self, u: np.ndarray):
        """위팔 방향 u 를 만드는 (어깨 앞뒤, 어깨 옆, 신뢰도) 두 해. 어깨 옆은 wrap 된 값."""
        a = self.w["shoulder_yaw"]
        sx = self.w["shoulder_roll"][0]
        sy_ = self.w["shoulder_pitch"][1]
        phi = math.atan2(a[2], a[1])
        c = math.acos(float(np.clip(u[1], -1, 1)))
        out = []
        for s in (+1, -1):
            sr = wrap(sx * (s * c - phi))
            v = rot(self.w["shoulder_roll"], sr) @ a
            conf = float(np.clip(abs(v[2]) / math.sin(math.radians(10)), 0, 1))
            g = 1.0 if v[2] >= 0 else -1.0
            sp = sy_ * math.atan2(u[0] * g, u[2] * g) if abs(v[2]) > 1e-9 else float("nan")
            out.append((sp, sr, conf))
        return out

    def ik_twist(self, q_sp: float, q_sr: float, u: np.ndarray, f: np.ndarray) -> Tuple[float, float]:
        """(어깨 비틀기, 팔꿈치). 팔꿈치 축이 u×f 방향이 되도록 어깨 비틀기를 고른다."""
        sig = SIGMA[self.side]
        el = ang(u, f)
        n = np.cross(u, f)
        if np.linalg.norm(n) < 1e-9:
            return float("nan"), sig * el
        e_t = sig * unit(n)
        R1 = rot(self.w["shoulder_pitch"], q_sp) @ rot(self.w["shoulder_roll"], q_sr)
        e_b = R1 @ self.w["elbow_pitch"]
        u_r = R1 @ self.w["shoulder_yaw"]
        sy = math.atan2(float(np.dot(u_r, np.cross(e_b, e_t))), float(np.dot(e_b, e_t)))
        return sy, sig * el


# --------------------------------------------------------------------------- 사람 쪽

FOREARM_RADIUS = 0.03     # URDF 손 원기둥 반지름 (m)
# 🔴 REPORT_54 P1 (2026-10-13 승인): 모델 손 길이. 아래팔+손 = 팔꿈치 → 손끝 선분 (반지름 FOREARM_RADIUS).
#   손끝 = 손목 관절(팔꿈치 축에서 아래팔 쪽 12.3 cm)에서 손 방향으로 HAND_TIP_M.
#   cad    = 0.133 m → 팔꿈치 축에서 25.6 cm = CAD 그리퍼 집게 끝 (bhl_assembly.step). 기본.
#   legacy = 0.05 m  → 17.3 cm = 원래 URDF 손 원기둥 끝 (그리퍼 달기 전 모델, 실제보다 약 8 cm 짧다)
#   측정 (armpose_20261013_210455_G): 실제 맞닿음에서 cad 모델 겉면 −1.4~−3.3 cm (모델이 1~3 cm 일찍 닿았다고 봄 = 안전 쪽)
HAND_MODELS = {"cad": 0.133, "legacy": 0.05}
HAND_BACK_M = 0.07         # 손목 관절에서 팔꿈치 쪽 끝 (선분은 팔꿈치부터라 쓰이지 않는다)
HAND_MODEL = "cad"
HAND_TIP_M = HAND_MODELS[HAND_MODEL]


def set_hand_model(name: str) -> None:
    """모델 손 길이를 고른다 (cad / legacy). forearm_segment·segment_of·두 팔 벌림·충돌 방지가 모두 이 값을 쓴다."""
    global HAND_MODEL, HAND_TIP_M
    if name not in HAND_MODELS:
        raise ValueError(f"hand model {name!r} — {sorted(HAND_MODELS)}")
    HAND_MODEL, HAND_TIP_M = name, HAND_MODELS[name]


HAND_FINGER_M = 0.055     # 집게 길이 (REPORT_54:387). 인계 예외(--handover-exempt)가 판정에서 빼는 손 끝 구간


def handover_gaps(pa, pb, ext_m: float) -> Tuple[float, float, float]:
    """인계 예외용 두 팔 거리 [m]. pa, pb = ArmModel.points() 결과 (팔꿈치, 손목, 손 축).
    반환 (full, body, finger):
      full   = 지금 충돌 모델 겉면 거리 (segment_of 선분, 반지름 FOREARM_RADIUS) — forearm_segment 와 같은 값
      body   = 선분 끝을 손 끝에서 ext_m 만큼 팔꿈치 쪽으로 줄인 겉면 거리 (ext_m = 0 이면 full)
      finger = 집게 구간(손 끝에서 HAND_FINGER_M)끼리: 한 팔 집게 끝 → 다른 팔 집게 구간 중심선 거리 중 작은 것 (반지름 빼지 않음).
               정면으로 끝끼리 맞닿으면 0, 손 축 ~90° 로 엇갈려 같은 작은 물체를 물면 무는 깊이만큼 남는다."""
    segs, tips, dirs = [], [], []
    for el, wr, ax in (pa, pb):
        e0, e1 = segment_of_points(el, wr, ax)
        d = e1 - wr
        n = float(np.linalg.norm(d))
        d = d / n if n > 1e-9 else np.asarray(ax, float)
        segs.append((e0, e1)); tips.append(e1); dirs.append(d)
    full = segment_gap(segs[0][0], segs[0][1], segs[1][0], segs[1][1]) - 2 * FOREARM_RADIUS
    if ext_m > 0.0:
        a1, b1 = tips[0] - dirs[0] * ext_m, tips[1] - dirs[1] * ext_m
        body = segment_gap(segs[0][0], a1, segs[1][0], b1) - 2 * FOREARM_RADIUS
    else:
        body = full
    r0, r1 = tips[0] - dirs[0] * HAND_FINGER_M, tips[1] - dirs[1] * HAND_FINGER_M
    finger = min(segment_gap(tips[0], tips[0], r1, tips[1]), segment_gap(tips[1], tips[1], r0, tips[0]))
    return full, body, finger


def segment_of_points(el, wr, ax) -> Tuple[np.ndarray, np.ndarray]:
    """ArmModel.segment_of 와 같은 선분 (모델 객체 없이)."""
    ends = (wr + ax * (-HAND_BACK_M), wr + ax * HAND_TIP_M)
    return el, max(ends, key=lambda e: float(np.linalg.norm(e - el)))


def segment_gap(a0, a1, b0, b1) -> float:
    """두 선분 사이 가장 가까운 거리 (m, 닫힌 식)."""
    d1, d2, r = a1 - a0, b1 - b0, a0 - b0
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    if a < 1e-12 and e < 1e-12:
        return float(np.linalg.norm(r))
    if a < 1e-12:
        s, t = 0.0, min(max(f / e, 0.0), 1.0)
    else:
        c = float(d1 @ r)
        if e < 1e-12:
            t, s = 0.0, min(max(-c / a, 0.0), 1.0)
        else:
            b = float(d1 @ d2)
            den = a * e - b * b
            s = min(max((b * f - c * e) / den, 0.0), 1.0) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, min(max(-c / a, 0.0), 1.0)
            elif t > 1.0:
                t, s = 1.0, min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm((a0 + s * d1) - (b0 + t * d2)))


def qrot(q, v):
    x, y, z, w = q
    u = np.array([x, y, z])
    v = np.asarray(v, float)
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


def qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def qinv(q):
    return np.array([-q[0], -q[1], -q[2], q[3]]) / float(np.dot(q, q))


def twist_y(q) -> float:
    """쿼터니언의 로컬 y(뼈 긴축) 둘레 비틀림 각 (swing-twist)."""
    x, y, z, w = q
    return wrap(2 * math.atan2(y, w))


@dataclass
class HumanCalib:
    """VR 캘리브레이션 (사람 차렷). 몸 틀과 손목 비틀기 0 을 잡는다."""
    fwd: np.ndarray
    right: np.ndarray
    wrist_rel0: Dict[str, np.ndarray] = field(default_factory=dict)
    up: np.ndarray = field(default_factory=lambda: UP_UE.copy())
    upper_ref: Dict[str, np.ndarray] = field(default_factory=dict)   # 위팔 뼈 회전 (긴축 비틀림 기준)
    elbow_rest: Dict[str, float] = field(default_factory=dict)       # 차렷 때 사람 팔꿈치 입력 (rad, 중앙값) — 머리 위 B
    # REPORT_38 손 위치 방식: 차렷 때 사람 팔 길이 (cm, 중앙값). reach = 어깨→손목 직선 거리 (Meta 의 가짜 굽힘 포함)
    upper_len: Dict[str, float] = field(default_factory=dict)
    fore_len: Dict[str, float] = field(default_factory=dict)
    reach_rest: Dict[str, float] = field(default_factory=dict)
    shoulder_width: float = float("nan")                              # 두 어깨 관절 수평 거리 (cm)

    @staticmethod
    def from_bones(bones_list) -> "HumanCalib":
        """bones_list: [{name: (pos, quat)}] — 차렷 동안의 프레임들 (평균)."""
        r = np.mean([b["R_UpperArm"][0] - b["L_UpperArm"][0] for b in bones_list], axis=0)
        r = unit(r - np.dot(r, UP_UE) * UP_UE)
        fwd = unit(np.cross(r, UP_UE))           # 언리얼(왼손 좌표계)에서 오른쪽 × 위 = 앞 (REPORT_17 §0)
        rel0, upper, elbow = {}, {}, {}
        lu, lf, reach = {}, {}, {}
        for side, (U, L, W) in BONES.items():
            elbow[side] = float(np.median([ang(b[L][0] - b[U][0], b[W][0] - b[L][0]) for b in bones_list]))
            lu[side] = float(np.median([np.linalg.norm(b[L][0] - b[U][0]) for b in bones_list]))
            lf[side] = float(np.median([np.linalg.norm(b[W][0] - b[L][0]) for b in bones_list]))
            reach[side] = float(np.median([np.linalg.norm(b[W][0] - b[U][0]) for b in bones_list]))
            qs = [qmul(qinv(b[L][1]), b[W][1]) for b in bones_list]
            m = np.mean([q if np.dot(q, qs[0]) >= 0 else -q for q in qs], axis=0)
            rel0[side] = m / np.linalg.norm(m)
            us = [b[U][1] for b in bones_list]
            mu = np.mean([q if np.dot(q, us[0]) >= 0 else -q for q in us], axis=0)
            upper[side] = mu / np.linalg.norm(mu)
        sw = float(np.median([np.linalg.norm((d := b["R_UpperArm"][0] - b["L_UpperArm"][0]) - np.dot(d, UP_UE) * UP_UE)
                              for b in bones_list]))
        return HumanCalib(fwd=fwd, right=r, wrist_rel0=rel0, upper_ref=upper, elbow_rest=elbow,
                          upper_len=lu, fore_len=lf, reach_rest=reach, shoulder_width=sw)

    def set_arm_lengths(self, bones_list) -> None:
        """손 위치 방식용 팔 길이·어깨 폭만 이 프레임들(차렷)의 중앙값으로 다시 잡는다."""
        c = HumanCalib.from_bones(bones_list)
        self.upper_len, self.fore_len, self.reach_rest, self.shoulder_width = (
            c.upper_len, c.fore_len, c.reach_rest, c.shoulder_width)

    def set_wrist_zero(self, bones_list, sides=("left", "right")) -> None:
        """손목 0 을 이 프레임들(예: 팔꿈치 90° 엄지 위)로 다시 잡는다 (REPORT_29 A). 제자리에서 바꾼다."""
        for side in sides:
            U, L, W = BONES[side]
            qs = [qmul(qinv(b[L][1]), b[W][1]) for b in bones_list if L in b and W in b]
            if not qs:
                continue
            m = np.mean([q if np.dot(q, qs[0]) >= 0 else -q for q in qs], axis=0)
            self.wrist_rel0[side] = m / np.linalg.norm(m)

    def to_robot(self, v_ue: np.ndarray) -> np.ndarray:
        """언리얼 벡터 → 로봇 틀 (x 앞, y 왼쪽, z 위)."""
        return np.array([np.dot(v_ue, self.fwd), -np.dot(v_ue, self.right), np.dot(v_ue, self.up)])


HEADING_FOLLOW_TAU_S = 0.15       # 작업 지시 12-c: 부드러운 경계에서 다 따라갈 때의 시간 상수 (초)


class HeadingDeadband:
    """작업 지시 12-b --heading-deadband-deg: 몸 추적의 가슴 방향이 머리를 따라 도는 것을 손 목표에서 뺀다 (REPORT_shoulder_jitter 안 라).
    y = 쓰는 몸 방향, a = 가슴 방향. 프레임마다
      |a − y| > 무감대 이면 y 를 넘친 만큼 옮긴다 → 그 뒤 y += (a − y)(1 − exp(−dt/τ))
    어깨 = 가슴 위치 + (차렷 때 가슴→어깨, y − 차렷 방향 만큼 수직축으로 돌림). 같은 시각 프레임은 다시 계산하지 않는다.
    작업 지시 12-c --heading-soft-deg S: 경계를 넘친 만큼 바로 옮기는 대신, 무감대 − S ~ 무감대 + S 에서 따라가는 비율 w 를 0 → 1 로 (매끄럽게)
      y += (a − y)(1 − exp(−dt (1/τ + w/τ빠름))).  τ빠름 = HEADING_FOLLOW_TAU_S"""

    def __init__(self, deadband_deg: float, tau_s: float, cal: "HumanCalib", chest_ref, bones_list, soft_deg: Optional[float] = None,
                 follow_tau_s: float = HEADING_FOLLOW_TAU_S):
        self.db, self.tau = math.radians(deadband_deg), float(tau_s)
        self.soft = None if soft_deg is None else math.radians(soft_deg)
        self.follow_tau = float(follow_tau_s)
        self.fwd0, self.q_ref = np.array(cal.fwd, float), chest_ref
        self.a0 = self.a = self.y = math.atan2(self.fwd0[1], self.fwd0[0])
        self.off = {side: np.median([b[BONES[side][0]][0] - b["Chest"][0] for b in bones_list], axis=0) for side in BONES}
        self.t: Optional[float] = None
        self.shoulder: Dict[str, np.ndarray] = {}

    def _rz(self, v) -> np.ndarray:
        d = self.y - self.a0
        c, s = math.cos(d), math.sin(d)
        return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])

    def fwd(self) -> np.ndarray:
        return unit(self._rz(self.fwd0))

    def deg(self, used: bool) -> float:
        return math.degrees(wrap((self.y if used else self.a) - self.a0))   # body_heading_deg 와 같은 범위

    def view(self, bones, t: float):
        """bones 에서 어깨만 바꾼 사본. 이 시각 프레임을 처음 보면 방향을 한 번 나아간다."""
        if "Chest" not in bones or any(BONES[side][0] not in bones for side in BONES):
            return bones
        if t != self.t:
            pc, qc = bones["Chest"]
            v = qrot(qmul(qc, qinv(self.q_ref)), self.fwd0)
            self.a += wrap(math.atan2(v[1], v[0]) - self.a)
            dt = 0.0 if self.t is None else max(t - self.t, 0.0)
            err = self.a - self.y
            if self.soft is not None:
                x = min(max((abs(err) - self.db + self.soft) / (2.0 * self.soft), 0.0), 1.0)
                self.y += err * (1.0 - math.exp(-dt * (1.0 / self.tau + x * x * (3.0 - 2.0 * x) / self.follow_tau)))
            else:
                if abs(err) > self.db:
                    self.y += math.copysign(abs(err) - self.db, err)
                    err = self.a - self.y
                self.y += err * (1.0 - math.exp(-dt / self.tau))
            self.t = t
            self.shoulder = {side: pc + self._rz(self.off[side]) for side in BONES}
        out = dict(bones)
        for side in BONES:
            U = BONES[side][0]
            out[U] = (self.shoulder[side], bones[U][1])
        return out


@dataclass
class RobotRef:
    """로봇을 켤 때마다 잡는 모터 기준값 (arm_pose_logger --part R). 값은 모터 단위(rad)."""
    rest: Dict[str, Dict[str, float]]      # 차렷: side → joint → 모터값
    e90: Dict[str, Dict[str, float]]       # 팔꿈치 90° 엄지 위

    @staticmethod
    def load(path: Path) -> "RobotRef":
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        return RobotRef(rest=d["rest"], e90=d["e90"])


# --------------------------------------------------------------------------- 변환기

class PosRetargeter:
    def __init__(self, side: str, calib: HumanCalib, ref: Optional[RobotRef] = None, model: Optional[ArmModel] = None,
                 elbow_offset: float = 0.0, sy_policy: str = "hold", wrist_median: bool = False,
                 limits_rel: Optional[Dict[str, Tuple[float, float]]] = None,
                 wrist_hold: bool = True, overhead_elbow_hold: bool = True, elbow_base: bool = True,
                 flip_hold: bool = True):
        self.side = side
        self.flip_hold = flip_hold                       # R1 (REPORT_42): 등 뒤 팔에서 머리 위로 넘긴 해로 뛰지 않음
        self.flip_holds = 0                              # 그래서 유지한 횟수 (누적, 로그용)
        self.wrist_hold = wrist_hold                     # 손목 A (REPORT_32): 팔 편 동안 손목 유지
        self.overhead_elbow_hold = overhead_elbow_hold   # 머리 위 A: 위팔이 높으면 팔꿈치 유지
        self.elbow_base = elbow_base                     # 머리 위 B: 팔꿈치에서 차렷 값을 뺀다 (사람별)
        self.hand_mode = False                           # REPORT_38: True 면 elbow_zero = 0 (HandTargeter 가 dirs 를 준다)
        self.el_prev: Optional[float] = None
        self.el_entry: Optional[float] = None   # 머리 위 A: 위팔이 OVERHEAD_LO 아래였던 마지막 팔꿈치 입력
        self.pron_prev = 0.0
        # 어깨 앞뒤·옆 한계 (차렷 기준 상대각, rad). 없으면 ROBOT_LIMITS_REL_DEG.
        self.limits_rel = dict({j: (math.radians(a), math.radians(b))
                                for j, (a, b) in ROBOT_LIMITS_REL_DEG[side].items()}, **(limits_rel or {}))
        self.last_conf, self.last_sp_ik = 1.0, float("nan")
        self.sp_anchor: Optional[float] = None   # REPORT_35: 옆 수평 근처에 들어올 때의 어깨 앞뒤
        self.m = model or ArmModel(side)
        self.cal = calib
        self.ref = ref
        self.elbow_offset = elbow_offset
        # 어깨 비틀기 기준: 위팔 수직(차렷 어깨) + 팔꿈치 90° + 아래팔 정면 일 때의 URDF 값
        q = Q_REF_REST[side]
        self.sy_ref = self.m.ik_twist(q["shoulder_pitch"], q["shoulder_roll"],
                                      np.array([0, 0, -1.0]), np.array([1.0, 0, 0]))[0]
        self.q_ref = dict(q, shoulder_yaw=self.sy_ref)
        self.sy_policy = sy_policy if sy_policy in SY_POLICIES else "hold"
        self.bone_offset = None      # D: 어깨 비틀기 − 부호×(뼈 긴축 회전)
        self.last_sy_ik = float("nan")
        self.last_arm_twist = float("nan")
        self.wrist_median = wrist_median   # C: 손목 엎침각 3프레임 중앙값 (REPORT_29)
        self._pron_hist: list = []
        self.sy_prev = self.sy_ref
        self.sr_prev = q["shoulder_roll"]
        self.sp_prev = q["shoulder_pitch"]
        self.last_w = 0.0

    def dirs(self, bones) -> Tuple[np.ndarray, np.ndarray]:
        U, L, W = BONES[self.side]
        u = unit(self.cal.to_robot(bones[L][0] - bones[U][0]))
        f = unit(self.cal.to_robot(bones[W][0] - bones[L][0]))
        return u, f

    def windows(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """로봇 어깨 앞뒤·옆 한계 (URDF 값) — 워치독이 명령을 자르는 범위와 같다 (REPORT_31)."""
        q0 = self.q_ref
        (plo, phi), (rlo, rhi) = self.limits_rel["shoulder_pitch"], self.limits_rel["shoulder_roll"]
        return ((q0["shoulder_pitch"] + plo, q0["shoulder_pitch"] + phi),
                (q0["shoulder_roll"] + rlo, q0["shoulder_roll"] + rhi))

    def shoulder(self, u: np.ndarray) -> Tuple[float, float]:
        """위팔 방향 u → (어깨 앞뒤, 어깨 옆). 두 해 중 하나를 고르고 이어 붙인다. 상태(sp_prev, sr_prev) 갱신은 step 이 한다.

        🔴 REPORT_31: 팔을 옆으로 수평을 지나 올릴 때 두 해("어깨 옆 105°" / "어깨 옆 75° + 어깨 앞뒤 180° 뒤집힘")가
        겹친다. 예전에는 ① 어깨 앞뒤 변화량에 신뢰도(수평 근처 ≈0)를 곱해서 뒤집힌 해도 비용이 거의 0 이었고
        ② 범위 창이 ±225° 로 넓어서 로봇이 갈 수 없는 해(차렷인데 앞뒤 −180°·옆 +180°)에 머물 수 있었다.
        규칙:
          a. 로봇 관절 한계를 안다: 고른 해를 한계로 자른 자세가 사람 팔과 SWITCH_ERR_DEG 넘게 어긋날 때만
             한계 안에서 가장 잘 맞는 해로 바꾼다. (한계 밖 해를 딱 잘라 빼면 만세 끝·팔을 뒤로 뺀 옆 수평에서
             오히려 머리 위로 도는 해가 남는다 — REPORT_31 §3)
          b. 어깨 앞뒤 변화량은 신뢰도를 곱하지 않고 그대로 센다 → 두 해의 어깨 앞뒤는 늘 180° 차이라
             직전 값에서 90° 안쪽 해가 이긴다. 동점 근처는 "앞뒤·옆 둘 다 크게 꺾인" 벌점 — 로봇이 실제로 갈
             자세(한계로 자른 값)로 센다 (머리 위로 넘긴 자세 vs 뒤쪽 한계에 멈춘 자연스러운 자세).
          c. 위팔이 옆 수평 축 SP_HOLD_DEG 안이면 어깨 앞뒤를 멈추고, 어깨 옆은 그 앞뒤에서 가장 잘 맞는 값.
             → 수평을 지나는 동안 어깨 앞뒤가 돌지 않고, 어깨 옆은 한 방향으로 계속 간다."""
        (sp_lo, sp_hi), (sr_lo, sr_hi) = self.windows()
        sr0 = Q_REF_REST[self.side]["shoulder_roll"]
        anchor0 = self.sp_anchor
        dist = math.degrees(math.acos(float(np.clip(abs(u[1]), 0.0, 1.0))))     # 옆 수평 축과의 각
        # ① 목표 어깨 앞뒤: 두 해(와 ±360° 사본) 중 직전 값에서 덜 움직이는 해 (변화량에 신뢰도를 곱하지 않는다)
        #    + "앞뒤·옆 둘 다 크게 꺾인" 벌점 — 로봇이 실제로 갈 자세(한계로 자른 값)로 센다
        cands = []
        for sp_c, sr_c, _ in self.m.ik_shoulder_candidates(u):
            if not math.isfinite(sp_c):
                continue
            for kr in (-1, 0, 1):
                sr_u = sr_c + 2 * math.pi * kr
                for k in (-1, 0, 1):
                    sp_u = sp_c + 2 * math.pi * k
                    sp_k = min(max(sp_u, sp_lo), sp_hi)
                    sr_k = min(max(sr_u, sr_lo), sr_hi)
                    cost = (abs(sp_u - self.sp_prev) + abs(sr_u - self.sr_prev)
                            + BRANCH_PENALTY * min(abs(sp_k), abs(sr_k - sr0)))
                    cands.append((cost, sp_u, sp_k, sr_k))
        best = min(cands) if cands else None
        # ①' 그 해가 로봇 한계에 막혀 사람 팔과 SWITCH_ERR_DEG 넘게 어긋나면, 한계 안에서 가장 잘 맞는 해로 바꾼다
        #    (바꾼 뒤에는 새 해가 "덜 움직이는 해" 가 되므로 되돌아가지 않는다)
        if best is not None:
            def err(c):
                a_k, _ = self.m.fk_dirs({"shoulder_pitch": c[2], "shoulder_roll": c[3],
                                         "shoulder_yaw": 0.0, "elbow_pitch": 0.0, "elbow_roll": 0.0})
                return ang(a_k, u)
            if err(best) > math.radians(SWITCH_ERR_DEG):
                best = min(cands, key=lambda c: (round(err(c), 3), c[0]))
        # ② 위팔이 어깨 앞뒤 축(옆 수평)에 가까우면 어깨 앞뒤는 멈춘다: 이 근처에서는 앞뒤로 조금만 흔들려도
        #    정확한 해의 어깨 앞뒤가 수십~180° 돈다 (특이점). 방향 오차를 조금 받아들이고 돌지 않는다.
        g = float(np.clip((dist - SP_HOLD_DEG) / (SP_FULL_DEG - SP_HOLD_DEG), 0.0, 1.0))
        sp_t = best[1] if best is not None else self.sp_prev
        sp = self.sp_prev + g * (sp_t - self.sp_prev)
        # ②' REPORT_35: "자세는 한 번 정하면 유지". 위팔이 옆 수평 축 KEEP_ZONE_DEG 안에 들어오면, 들어올 때의 어깨 앞뒤를
        #     기준(sp_anchor)으로 잡고 그 ±KEEP_BAND_DEG 띠 안에서만 움직인다 (두 해 각각을 띠로 자른 것 중 가장 잘 맞는 것).
        #     띠 안 자세로 사람 팔과 KEEP_ERR_DEG 넘게 어긋나야 비로소 띠를 풀고 자세를 바꾼다 (그때 기준도 새로 잡음).
        #     → 옆 만세(옆 자세로 들어와 앞으로 조금 기울어도 옆 자세)·옆 흔들기(앞 돌린 자세로 들어오면 계속 그 자세)
        if dist < KEEP_ZONE_DEG and self.sp_anchor is not None:
            band_lo = max(self.sp_anchor - math.radians(KEEP_BAND_DEG), sp_lo)
            band_hi = min(self.sp_anchor + math.radians(KEEP_BAND_DEG), sp_hi)
            opts = []
            for c in cands:
                spb = min(max(c[1], band_lo), band_hi)
                # 팔이 수평 아래인데 어깨 앞뒤 |값| > 90° (머리 위로 넘긴 자세) 는 유지하지 않는다 — 내리는 길에 그 자세에
                # 붙어 있으면 차렷에서 크게 돌아야 한다 (check_2212 118.6초: 앞 머리 위에서 옆으로 빠르게 내림)
                if u[2] < -0.2 and abs(spb - self.q_ref["shoulder_pitch"]) > math.pi / 2:
                    continue
                opts.append((self._fit_roll(spb, u)[1], abs(spb - self.sp_prev), spb))
            if not cands:                       # 축 위 (해 없음) → 유지
                opts = [(0.0, 0.0, self.sp_prev)]
            e_b, _, sp_b = min(opts) if opts else (math.pi, 0.0, sp)
            if e_b <= math.radians(KEEP_ERR_DEG):
                sp = self.sp_prev + g * (sp_b - self.sp_prev)
            else:
                self.sp_anchor = sp
        else:
            self.sp_anchor = sp
        # ③ 어깨 옆: 정한 어깨 앞뒤에서 방향이 가장 잘 맞는 값 (한 가지로 정해진다 → 두 해 사이를 뛰지 않는다)
        sr_fit, _ = self._fit_roll(sp, u)
        sr = self.sr_prev + wrap(sr_fit - self.sr_prev)
        # ④ REPORT_40 D1 (2026-10-03 승인): 상태값을 360° 사본 중 로봇 한계 창(± SY_STATE_MARGIN)에 가장 가까운 것으로.
        #    같은 자세를 다르게 적을 뿐이다. 안 하면 어깨 옆이 ±180° 를 지날 때 +350°(= −10°) 같은 사본을 계속 들고 다니고
        #    (③ 이 직전 값에서 이어 붙이므로), 워치독이 그것을 한계로 잘라 만세 쪽 끝(+154°)을 끝까지 명령한다.
        sp2, sr2 = self._in_window(sp, sp_lo, sp_hi), self._in_window(sr, sr_lo, sr_hi)
        # ⑤ R1 (REPORT_42): 등 뒤 팔 → 머리 위로 넘긴 해로 한 번에 뛰지 않는다. 이 프레임은 직전 값, 다음 프레임에 다시 판단
        s0 = self.q_ref["shoulder_pitch"]
        if (self.flip_hold and abs(sp2 - s0) > math.pi / 2 and abs(self.sp_prev - s0) <= math.pi / 2
                and abs(sp2 - self.sp_prev) > math.radians(FLIP_HOLD_JUMP_DEG) and (u[2] < 0.0 or not FLIP_HOLD_BELOW)):
            self.flip_holds += 1
            self.sp_anchor = anchor0
            self.last_conf, self.last_sp_ik = g, sp_t
            return self.sp_prev, self.sr_prev
        if sp2 != sp and self.sp_anchor is not None:
            self.sp_anchor += sp2 - sp
        self.last_conf, self.last_sp_ik = g, sp_t
        return sp2, sr2

    def hand_refit(self, q: Dict[str, float], target: np.ndarray,
                   elbow_lim: Optional[Tuple[float, float]] = None) -> Optional[Dict[str, float]]:
        """REPORT_47 (REPORT_46 안 A): 어깨 옆이 몸 쪽 한계(왼 아래 끝·오른 위 끝)를 넘으려 하면 한계로 자르고,
        어깨 앞뒤는 그대로 둔 채 어깨 비틀기·팔꿈치로 손목 관절을 target(m, base 틀)에 맞춘다.
        비틀기는 위팔 든 각의 범위 규칙(yaw_window_deg)과 한계 안, 팔꿈치는 URDF·elbow_lim(차렷 기준 rad) 안.
        넘지 않으면 None. 반환: {shoulder_roll, shoulder_yaw, elbow_pitch, err0_cm, err_cm} (자르기만 했을 때 / 맞춘 뒤 손목 오차)."""
        (_, _), (rlo, rhi) = self.windows()
        inward_lo = self.side == "left"
        sr = q["shoulder_roll"]
        if (inward_lo and sr >= rlo) or ((not inward_lo) and sr <= rhi):
            return None
        sr_c = rlo if inward_lo else rhi
        fwd = upper_arm_forward_deg(self.side, math.degrees(q["shoulder_pitch"] - self.q_ref["shoulder_pitch"]))
        wlo, whi = yaw_window_deg(self.side, fwd)
        ylo = self.sy_ref + max(math.radians(wlo), self.limits_rel["shoulder_yaw"][0])
        yhi = self.sy_ref + min(math.radians(whi), self.limits_rel["shoulder_yaw"][1])
        elo, ehi = self.m.lower["elbow_pitch"], self.m.upper["elbow_pitch"]
        if elbow_lim is not None:
            elo, ehi = max(elo, elbow_lim[0]), min(ehi, elbow_lim[1])

        def wrist(v):
            return self.m.points({"shoulder_pitch": q["shoulder_pitch"], "shoulder_roll": sr_c, "shoulder_yaw": v[0],
                                  "elbow_pitch": v[1], "elbow_roll": 0.0})[1]
        x = np.array([min(max(q["shoulder_yaw"], ylo), yhi), min(max(q["elbow_pitch"], elo), ehi)])
        e0 = float(np.linalg.norm(wrist(x) - target))
        for _ in range(HAND_REFIT_ITERS):
            w0 = wrist(x)
            r = w0 - target
            Jm = np.zeros((3, 2))
            for k in range(2):
                d = np.zeros(2); d[k] = 1e-4
                Jm[:, k] = (wrist(x + d) - w0) / 1e-4
            dx = np.clip(np.linalg.lstsq(Jm, -r, rcond=None)[0], -0.2, 0.2)
            x = np.array([min(max(x[0] + dx[0], ylo), yhi), min(max(x[1] + dx[1], elo), ehi)])
        return {"shoulder_roll": sr_c, "shoulder_yaw": float(x[0]), "elbow_pitch": float(x[1]),
                "err0_cm": e0 * 100, "err_cm": float(np.linalg.norm(wrist(x) - target)) * 100}

    @staticmethod
    def _in_window(x: float, lo: float, hi: float) -> float:
        """x 의 360° 사본 (x, x±2π) 중 [lo − 5°, hi + 5°] 에서 가장 가까운 것. 같은 거리면 x 그대로."""
        def out(y):
            return round(max(lo - SY_STATE_MARGIN - y, 0.0, y - hi - SY_STATE_MARGIN), 9)
        return min((x, x - 2 * math.pi, x + 2 * math.pi), key=out)

    def _fit_roll(self, sp: float, u: np.ndarray) -> Tuple[float, float]:
        """어깨 앞뒤 sp 에서 방향이 가장 잘 맞는 어깨 옆 값과, 로봇 한계로 자른 그 자세의 방향 오차(rad)."""
        up = rot(self.m.w["shoulder_pitch"], sp).T @ u
        a = self.m.w["shoulder_yaw"]
        sx = self.m.w["shoulder_roll"][0]
        sr_fit = sx * (math.atan2(up[2], up[1]) - math.atan2(a[2], a[1]))
        (sp_lo, sp_hi), (sr_lo, sr_hi) = self.windows()
        sr_u = self.sr_prev + wrap(sr_fit - self.sr_prev)
        a_k, _ = self.m.fk_dirs({"shoulder_pitch": min(max(sp, sp_lo), sp_hi),
                                 "shoulder_roll": min(max(sr_u, sr_lo), sr_hi),
                                 "shoulder_yaw": 0.0, "elbow_pitch": 0.0, "elbow_roll": 0.0})
        return sr_fit, ang(a_k, u)

    def step(self, bones, dirs: Optional[Tuple[np.ndarray, np.ndarray]] = None) -> Dict[str, float]:
        """한 프레임 → q_URDF (+ 손목은 사람 엎침각). 반환: joint → 값, 'wrist_pron', 'hold_w'.
        dirs = (u, f) 를 주면 사람 팔 방향 대신 그것을 쓴다 (REPORT_38 손 위치 방식 — HandTargeter)."""
        u, f = self.dirs(bones) if dirs is None else dirs
        sp, sr = self.shoulder(u)
        sy_ik, ep = self.m.ik_twist(sp, sr, u, f)
        el = self.elbow_in(u, abs(ep))
        w = self.bend_weight(el)
        U = BONES[self.side][0]
        arm_tw = float("nan")
        if U in bones and self.cal.upper_ref.get(self.side) is not None:
            arm_tw = ARM_TWIST_SIGN[self.side] * twist_y(qmul(qinv(self.cal.upper_ref[self.side]), bones[U][1]))
        self.last_sy_ik, self.last_arm_twist = sy_ik, arm_tw
        sy = self.shoulder_yaw(sy_ik, arm_tw, w, sp=sp)
        ep = SIGMA[self.side] * max(0.0, el - self.elbow_zero())
        self.sp_prev, self.sr_prev, self.sy_prev, self.last_w = sp, sr, sy, w
        U, L, W = BONES[self.side]
        rel = qmul(qinv(bones[L][1]), bones[W][1])
        pron_raw = HUMAN_PRONATION_SIGN[self.side] * twist_y(qmul(qinv(self.cal.wrist_rel0[self.side]), rel))
        pron = self.wrist_out(pron_raw, el)
        return {"shoulder_pitch": sp, "shoulder_roll": sr, "shoulder_yaw": sy, "elbow_pitch": ep,
                "wrist_pron": pron, "wrist_pron_raw": pron_raw, "elbow_used": el,
                "hold_w": w, "sy_ik": sy_ik, "arm_twist": arm_tw}

    def shoulder_yaw(self, sy_ik: float, arm_tw: float, w: float, sp: Optional[float] = None) -> float:
        """어깨 비틀기: 정책(hold/none/recenter/bone)에 따라 계산값 sy_ik 를 얼마나 따라갈지. w = 팔꿈치 굽힘 가중치."""
        if math.isfinite(sy_ik) and w > 0.99 and math.isfinite(arm_tw):
            # 팔을 굽힌 동안에는 위치 계산값이 정답이다 → 뼈 긴축 회전과의 차이를 배워 둔다 (D 에서 쓴다)
            d = wrap(sy_ik - arm_tw)
            self.bone_offset = d if self.bone_offset is None else wrap(
                self.bone_offset + BONE_OFFSET_ALPHA * wrap(d - self.bone_offset))
        if not math.isfinite(sy_ik):
            sy = self.sy_prev
        elif self.sy_policy == "none":
            sy = sy_ik
        elif self.sy_policy == "recenter":
            tgt = sy_ik if w > 0 else self.sy_ref
            sy = wrap(self.sy_prev + (w if w > 0 else RECENTER_RATE) * wrap(tgt - self.sy_prev))
        elif self.sy_policy == "bone" and math.isfinite(arm_tw) and self.bone_offset is not None:
            tgt = sy_ik if w >= 1.0 else wrap(arm_tw + self.bone_offset)
            sy = wrap(self.sy_prev + max(w, 0.5) * wrap(tgt - self.sy_prev))
        else:                                    # hold (현행)
            sy = self.sy_prev + w * wrap(sy_ik - self.sy_prev)
        # REPORT_35: ±180° 에서 값을 감지 않는다 (감으면 −179 → +179 로 넘어가 워치독이 반대쪽 한계로 자른다 —
        # 왼팔 옆 흔들기의 −44 ↔ +44 튐). 직전 값에서 이어 붙이고, 로봇 한계 + 5° 안에 둔다.
        sy = self.sy_prev + wrap(sy - self.sy_prev)
        lo, hi = self.limits_rel["shoulder_yaw"]
        # REPORT_36 §5: 안쪽 한계는 위팔을 앞으로 든 각에 따라 (워치독 최종 자르기와 같은 식)
        sp_now = self.sp_prev if sp is None else sp
        fwd = upper_arm_forward_deg(self.side, math.degrees(sp_now - self.q_ref["shoulder_pitch"]))
        wlo, whi = yaw_window_deg(self.side, fwd)
        lo, hi = max(lo, math.radians(wlo)), min(hi, math.radians(whi))
        return float(np.clip(sy, self.sy_ref + lo - SY_STATE_MARGIN, self.sy_ref + hi + SY_STATE_MARGIN))

    # ---- 사람 입력 거르기 (REPORT_32/33). 오프라인 재생 도구도 이 함수들을 그대로 쓴다.
    def elbow_zero(self) -> float:
        """로봇 팔꿈치 0 에 해당하는 사람 팔꿈치 입력 (rad). elbow_offset 을 직접 주면 그 값."""
        if self.elbow_offset:
            return self.elbow_offset
        if self.hand_mode:      # 손 위치 방식: 팔꿈치는 이미 로봇 팔 기준 (차렷 거리로 나눔) — 빼지 않는다
            return 0.0
        if self.elbow_base and self.side in self.cal.elbow_rest:
            return min(max(self.cal.elbow_rest[self.side], 0.0), ELBOW_BASE_MAX)
        return 0.0

    def elbow_in(self, u: np.ndarray, el_raw: float) -> float:
        """머리 위 A: 위팔이 높을수록 팔꿈치 입력을 덜 믿는다 (머리 위는 헤드셋 카메라 시야 밖 — Meta 추정값).
        높이 OVERHEAD_LO 아래에서의 마지막 입력(el_entry)과 지금 입력을 섞는다: 믿음 c = 1 (LO 이하) → 0 (HI 이상).
        섞는 값이 매 프레임 쌓이지 않으므로(저역 통과가 아님) 머리 위에 오래 있어도 따라가지 않는다."""
        if not math.isfinite(el_raw):
            return self.el_prev if self.el_prev is not None else 0.0
        elev = math.acos(float(np.clip(-u[2], -1.0, 1.0)))
        if not self.overhead_elbow_hold or self.el_entry is None or elev <= OVERHEAD_LO:
            self.el_entry = el_raw
            self.el_prev = el_raw
            return el_raw
        c = float(np.clip((OVERHEAD_HI - elev) / (OVERHEAD_HI - OVERHEAD_LO), 0.0, 1.0))
        self.el_prev = self.el_entry + c * (el_raw - self.el_entry)
        return self.el_prev

    def bend_weight(self, el: float) -> float:
        """어깨 비틀기를 계산값대로 따라갈 비율 (팔꿈치 굽힘). 머리 위 B 가 켜져 있으면 시작점을 차렷 팔꿈치+15° 이상으로 —
        Meta 는 편 팔도 20~30° 로 주므로, 30° 에서 시작하면 편 팔인데 비틀기가 엉뚱한 계산값을 따라간다 (REPORT_33 §2)."""
        lo = ELBOW_HOLD_LO
        if self.elbow_base:
            lo = max(lo, self.elbow_zero() + WRIST_HOLD_BAND)
        return float(np.clip((el - lo) / (ELBOW_HOLD_HI - ELBOW_HOLD_LO), 0, 1))

    def wrist_out(self, pron_raw: float, el: float) -> float:
        """손목 A: 팔을 편 동안(팔꿈치 < 40°, 또는 차렷 팔꿈치+15°) 손목을 유지한다 — 팔 흔들기·반대 팔 간섭이
        손목 입력에 섞여 들어오는 것을 막는다 (REPORT_32 §1·2). 굽히면 그대로 따라간다."""
        pron = pron_raw
        if self.wrist_median:
            self._pron_hist = (self._pron_hist + [pron])[-3:]
            pron = float(np.median(self._pron_hist))
        if self.wrist_hold:
            lo = max(WRIST_HOLD_LO, self.elbow_zero() + WRIST_HOLD_BAND)
            g = float(np.clip((el - lo) / WRIST_HOLD_BAND, 0.0, 1.0))
            pron = self.pron_prev + g * (pron - self.pron_prev)
        self.pron_prev = pron
        return pron

    def motor_targets(self, q: Dict[str, float]) -> Dict[str, float]:
        """q_URDF → 모터 목표값. 🔴 RobotRef 필요 (로봇을 켤 때마다 새로 잡은 값)."""
        if self.ref is None:
            raise RuntimeError("RobotRef 가 없다 — 로봇 기준값부터 잡아라 (arm_pose_logger --part R)")
        s = self.side
        out = {}
        for j in ("shoulder_pitch", "shoulder_roll", "elbow_pitch"):
            out[j] = self.ref.rest[s][j] + (q[j] - self.q_ref[j])           # k = +1 (REPORT_18 §10.1)
        out["shoulder_yaw"] = self.ref.e90[s]["shoulder_yaw"] + (q["shoulder_yaw"] - self.q_ref["shoulder_yaw"])
        out["elbow_roll"] = self.ref.e90[s]["elbow_roll"] + WRIST_MOTOR_SIGN[s] * q["wrist_pron"]
        return out


# --------------------------------------------------------------------------- 손 위치 기준 방식 (REPORT_38)

class HandTargeter:
    """사람 손목 **위치** → 로봇 손목 목표 위치 → 로봇 팔 방향 (u, f). 양팔을 함께 본다 (좌우 간격 때문에).

    1) 사람 손목 − 사람 어깨 (몸 틀) × k,  k = 로봇 팔 뻗은 길이 / 사람 차렷 때 어깨→손목 거리
       (차렷 거리로 나누면 Meta 가 차렷에서 주는 가짜 굽힘(맨손 15~30°, 컨트롤러 약 60°)이 로봇에서는 "편 팔" 이 된다
        — pos 방식의 "차렷 팔꿈치 빼기"(머리 위 B)를 위치로 한 것. 그래서 hand 방식에서는 머리 위 B 를 끈다)
    2) 양손 좌우 간격만 따로: 로봇 어깨 폭(35 cm)이 사람 비율보다 넓어서 1) 만으로는 양손이 모이지 않는다 (REPORT_37 §D).
       판정 거리(기본: 사람 두 손목 3차원 거리) ≤ touch → 로봇 좌우 간격 = min_gap, ≥ touch + blend → 1) 그대로, 사이는 비율.
       (REPORT_39: 좌우 간격으로 판정하면 한 팔을 몸 앞 가운데로 가져갈 때(16.8~19.6 cm)가 주먹 맞댐(22.7~23.9)보다 작다.
        touch_mode="lateral" 이면 REPORT_38 방식 — 좌우 간격, 섞는 구간은 문턱 ~ 사람 어깨 폭)
    3) 손목 목표 속도를 max_speed 로 자른다 (추적 튐 → 로봇 튐 방지).
    4) 로봇 팔꿈치 = 어깨–손목 목표를 축으로 한 원 위. 원 위 어디인지(돌림)는 사람 팔꿈치 쪽을 천천히 따라간다.
       사람 팔이 펴져서(팔꿈치가 어깨–손목 선에 붙어서) 돌림을 모르면 직전 값 유지.
    5) 로봇 팔꿈치·손목 관절의 옆 어긋남(2.5 cm)은 순기구학으로 3번 고쳐 맞춘다.
    6) 그 자세(모델)에서 두 아래팔 사이가 gap_goal(5 cm) 보다 가까우면 양손 목표를 좌우로 벌린다
       — 워치독 충돌 방지(4 cm)는 비틀기·어깨 옆만 막아서, 손 목표가 안으로 밀면 팔꿈치·어깨 앞뒤로 더 붙을 수 있다.
    결과 (u, f) 를 PosRetargeter.step(bones, dirs=(u, f)) 에 넣는다 → 관절 한계·비틀기 규칙·거르기는 pos 와 같다."""

    def __init__(self, calib: HumanCalib, models: Dict[str, ArmModel], touch_cm: float = HAND_TOUCH_CM,
                 min_gap_cm: float = HAND_MIN_GAP_CM, swivel_tau: float = HAND_SWIVEL_TAU,
                 max_speed: float = HAND_MAX_SPEED, gap_goal_cm: float = HAND_GAP_GOAL_CM,
                 blend_cm: float = HAND_BLEND_CM, touch_mode: str = "3d",
                 elbow_max: Optional[Dict[str, float]] = None,
                 pair: bool = False, pair_near_cm: float = HAND_PAIR_NEAR_CM, pair_far_cm: float = HAND_PAIR_FAR_CM,
                 grip_offset_cm: float = HAND_GRIP_OFFSET_CM, pair_center: str = "grip"):
        self.cal = calib
        # 작업 지시 10 인계 예외 (워치독 --handover-exempt 가 프레임마다 넣는다. 0 이면 예전과 같은 계산)
        self.handover_ext = 0.0                                   # 판정에서 뺄 손 끝 길이 (m, 0 ~ HAND_FINGER_M)
        self.handover_finger_min = 0.01                           # 집게끼리 거리 문턱 (m)
        self._pts: Dict[str, Tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        # 작업 지시 11-b 가대 면 제한 (워치독 --table 이 넣는다. None 이면 예전과 같은 계산)
        self.table = None                                         # table_limit.TableLimit
        self.table_servo_sign: Dict[str, float] = {}               # 작업 지시 12-c body real: 칼리브레이션에서 채운다
        self.table_roll: Dict[str, float] = {s: 0.0 for s in models}   # 손목 각 (rad, 모터 − e90, 직전 프레임)
        self.table_goal = 0.003                                   # 손 목표 쪽 여유 (m): 판 여유 위로 이만큼 더 → 모터 쪽은 받침으로만
        self.table_band = 0.005                                   # 부드럽게 올리기 시작하는 폭 (m)
        self.pair = bool(pair)                                    # REPORT_56 G3 (기본 끔)
        self.pair_near, self.pair_far = float(pair_near_cm), max(float(pair_far_cm), float(pair_near_cm) + 0.1)
        self.grip_offset = float(grip_offset_cm)
        # 작업 지시 10-c: 쌍 매핑 가운데점의 팔마다 잡는 점. grip = 사람 잡는 점 × 배율 (예전), robot-hand = 로봇 손목 목표 + 13.3 cm × 사람 손 축
        self.pair_center = pair_center if pair_center in HAND_PAIR_CENTERS else "grip"
        self._ext: Dict[str, float] = {}                         # REPORT_56 §9: 끝점을 손목에서 손 축으로 늘린 길이 (m, 쌍 매핑 때만)
        self._eff_cache: Dict[Tuple[str, float], Tuple[float, float, float]] = {}
        self.m = models
        self.touch = touch_cm
        self.blend = max(blend_cm, 0.1)
        self.touch_mode = touch_mode if touch_mode in HAND_TOUCH_MODES else "3d"
        self.min_gap = min_gap_cm / 100.0
        self.tau = swivel_tau
        self.vmax = max_speed
        self.gap_goal = gap_goal_cm / 100.0
        self.Ps, self.La, self.Lb, self.k, self.rmax, self.rmin = {}, {}, {}, {}, {}, {}
        for side, m in models.items():
            c = m.shoulder_center()
            q0 = dict(Q_REF_REST[side], shoulder_yaw=0.0, elbow_roll=0.0)
            e, w, _ = m.points(q0)
            self.Ps[side] = c
            self.La[side], self.Lb[side] = float(np.linalg.norm(e - c)), float(np.linalg.norm(w - e))
            self.rmax[side] = float(np.linalg.norm(w - c))           # 편 팔 (차렷 사람 → 로봇 편 팔)
            # 가장 가까운 손 = 팔꿈치 한계까지 굽힌 거리 (워치독 관절 한계 — 없으면 135°)
            bend = min(abs((elbow_max or {}).get(side, math.radians(135.0))), math.radians(135.0))
            self._bend = getattr(self, "_bend", {}); self._bend[side] = bend
            self.rmin[side] = math.sqrt(self.La[side] ** 2 + self.Lb[side] ** 2
                                        + 2 * self.La[side] * self.Lb[side] * math.cos(bend))
            rr = calib.reach_rest.get(side, float("nan"))
            self.k[side] = (float(np.linalg.norm(w - c)) / rr) if rr and math.isfinite(rr) and rr > 1 else 0.0055
        self.p_prev: Dict[str, np.ndarray] = {}
        self.n_sw: Dict[str, np.ndarray] = {s: np.array([-1.0, 0.0, 0.0]) for s in models}  # 처음: 팔꿈치 뒤쪽
        self.t_prev: Optional[float] = None
        self.off_prev: Dict[str, np.ndarray] = {}
        self.fit_target: Dict[str, np.ndarray] = {}
        self.clock = None            # 시각 함수 (없으면 time.monotonic) — 오프라인 재생은 재생 시각을 준다
        self.last: Dict[str, float] = {}

    def lateral_gap(self, g_h_cm: float, g_plain: float) -> float:
        """판정 거리(cm, touch_mode 에 따라 3차원 또는 좌우) → 로봇 손목 좌우 간격(m). g_plain = 1) 만 했을 때의 로봇 간격."""
        if self.touch_mode == "lateral":
            sw = self.cal.shoulder_width if math.isfinite(self.cal.shoulder_width) else 38.0
            far = max(sw, self.touch + 5.0)
        else:
            far = self.touch + self.blend
        if g_h_cm >= far:
            return g_plain
        if g_h_cm <= self.touch:
            return self.min_gap
        a = (g_h_cm - self.touch) / (far - self.touch)
        return self.min_gap + a * max(g_plain - self.min_gap, 0.0)

    def _solve(self, side: str, p: np.ndarray, n: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """어깨 중심 → p 를 두 링크(La, Lb)로, 팔꿈치를 n 쪽에 두어 (u, f).
        REPORT_56 §9: 쌍 매핑 중이면 끝점 = 손목 + ext · 손 축 → 둘째 링크 = 팔꿈치 → 그 끝점."""
        c, a, b = self.Ps[side], self.La[side], self.Lb[side]
        rmin = self.rmin[side]
        ext = self._ext.get(side, 0.0)
        if ext:
            b, rmin, _ = self._eff_geom(side, ext)
        d = p - c
        r = float(np.linalg.norm(d))
        dh = d / r if r > 1e-9 else np.array([0.0, 0.0, -1.0])
        r = min(max(r, rmin), (a + b) * 0.9999)
        x = (r * r + a * a - b * b) / (2 * r)
        h = math.sqrt(max(a * a - x * x, 0.0))
        nn = n - np.dot(n, dh) * dh
        if np.linalg.norm(nn) < 0.2:          # 돌림 방향이 축과 거의 나란 → 뒤쪽/아래쪽 중 축에 수직인 것
            for alt in (np.array([-1.0, 0, 0]), np.array([0, 0, -1.0]), np.array([0, 1.0, 0])):
                nn = alt - np.dot(alt, dh) * dh
                if np.linalg.norm(nn) > 0.2:
                    break
        nn = unit(nn)
        E = c + x * dh + h * nn
        P = c + r * dh
        return unit(E - c), unit(P - E)

    def dirs(self, bones, cal: Optional[HumanCalib] = None, t: Optional[float] = None,
             sides=("left", "right")) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
        cal = cal or self.cal
        if t is None:
            t = self.clock() if self.clock is not None else time.monotonic()
        first = self.t_prev is None
        dt = 0.0 if first else max(t - self.t_prev, 0.0)
        self.t_prev = t
        h, e = {}, {}
        for side in sides:
            U, L, W = BONES[side]
            h[side] = cal.to_robot(bones[W][0] - bones[U][0])          # cm, 몸 틀 (x 앞, y 왼, z 위)
            e[side] = cal.to_robot(bones[L][0] - bones[U][0])
        p = {s: self.Ps[s] + self.k[s] * h[s] for s in sides}
        self.last = {}
        g_h = g_r = d3 = float("nan")
        if "left" in sides and "right" in sides:
            dv = bones["L_WristTwist"][0] - bones["R_WristTwist"][0]
            g_h = float(cal.to_robot(dv)[1])
            d3 = float(np.linalg.norm(dv))
            g_plain = float(p["left"][1] - p["right"][1])
            g_r = self.lateral_gap(d3 if self.touch_mode == "3d" else g_h, g_plain)
            dlt = 0.5 * (g_r - g_plain)
            p["left"] = p["left"] + np.array([0.0, dlt, 0.0])
            p["right"] = p["right"] - np.array([0.0, dlt, 0.0])
        self.last = {"human_gap_cm": g_h, "human_dist_cm": d3,
                     "robot_gap_target_cm": g_r * 100 if math.isfinite(g_r) else g_r}
        if self.pair and "left" in sides and "right" in sides:
            self._pair_mix(bones, cal, h, p)
        out, P = {}, {}
        for side in sides:
            # 3) 속도 제한
            if side in self.p_prev:
                step = p[side] - self.p_prev[side]
                lim = self.vmax * dt
                nrm = float(np.linalg.norm(step))
                if nrm > lim:
                    p[side] = self.p_prev[side] + step * (lim / nrm)
            self.p_prev[side] = p[side]
            # 4) 돌림: 사람 팔꿈치가 어깨–손목 선에서 벗어난 방향
            hs = h[side]
            hh = unit(hs)
            perp = e[side] - np.dot(e[side], hh) * hh
            dp = float(np.linalg.norm(perp))
            lo, hi = HAND_SWIVEL_PERP_CM
            wt = float(np.clip((dp - lo) / (hi - lo), 0.0, 1.0))
            if wt > 0:
                al = 1.0 if first else wt * (1.0 - math.exp(-dt / max(self.tau, 1e-3)))
                self.n_sw[side] = unit(self.n_sw[side] + al * (perp / dp - self.n_sw[side]))
            P[side] = p[side]
        if self.table is not None:             # 작업 지시 11-b: 판 위에서 멈추도록 목표를 올린다 (벌림 계산 전에 — 벌림이 올린 자세를 보게)
            for side in sides:
                P[side] = self._table_lift(side, P[side])
        # 6) 두 아래팔(모델) 사이가 gap_goal 보다 가까우면 양손 목표를 좌우로 벌린다 (몇 번 고쳐 본다)
        # 먼저 벌리지 않은 목표로 제대로 푼다 (옆 어긋남 고침 포함). 벌릴 때는 그 고침 양을 그대로 쓰는 가벼운 계산으로
        # 벌릴 양을 찾고(할선법 몇 번), 마지막에 한 번만 제대로 푼다 — 200 Hz 루프를 밀지 않게 (REPORT_38 dry-run).
        segs, off = {}, {}
        for side in sides:
            out[side], segs[side], off[side] = self._fit(side, P[side])
            self.off_prev[side] = off[side]
        widen = 0.0
        if len(segs) == 2:
            gap = self._arm_gap(segs, self._pts)
            if gap < self.gap_goal:
                def cheap(w):
                    sg, pt = {}, {}
                    for side in sides:
                        sgn = 1.0 if side == "left" else -1.0
                        tgt = self._reach(side, P[side] + np.array([0.0, sgn * w, 0.0]))
                        u, f = self._solve(side, tgt + off[side], self.n_sw[side])
                        m = self.m[side]
                        pt[side] = m.points(m.q_from_dirs(u, f))
                        sg[side] = m.segment_of(*pt[side])
                    return self._arm_gap(sg, pt)
                w0, g0 = 0.0, gap
                w1 = (self.gap_goal - gap) / 1.6 + 0.002
                g1 = cheap(w1)
                for _ in range(3):
                    if self.gap_goal <= g1 <= self.gap_goal + 0.005:
                        break
                    slope = (g1 - g0) / (w1 - w0) if abs(w1 - w0) > 1e-6 else 1.6
                    slope = min(max(slope, 0.3), 4.0)
                    w0, g0, w1 = w1, g1, max(w1 + (self.gap_goal + 0.002 - g1) / slope, 0.0)
                    g1 = cheap(w1)
                if g1 < self.gap_goal:
                    w1 += (self.gap_goal - g1) / 0.3     # 그래도 모자라면 넉넉히
                widen = w1
                for side in sides:
                    sgn = 1.0 if side == "left" else -1.0
                    out[side], _, _ = self._fit(side, P[side] + np.array([0.0, sgn * widen, 0.0]))
        self.last["widen_cm"] = widen * 100
        for side in sides:
            self.last[f"{side}_target"] = P[side]
        return out

    def _table_lift(self, side: str, target: np.ndarray) -> np.ndarray:
        """작업 지시 11-b ①: 손 목표를 위로 올려 판 면 제한을 지킨다 (table_limit.soft_lift, 끊김 없음). 자세에 따라 오르는 양이 달라 몇 번 고쳐 푼다."""
        # 판 위(x 큼)는 곧게 올린다. 몸 쪽 손(차렷으로 내린 손, x 작음)은 올려도 아래팔이 가팔라 판에 닿으므로 올리면서 앞으로도 민다 → 팔꿈치 90° 근처에서 멈춘다
        x_min = HAND_TABLE_X_MIN + self._ext.get(side, 0.0)
        behind = max(x_min - float(target[0]), 0.0)
        lift = 0.0
        for _ in range(4):
            t = target + np.array([behind * min(lift / HAND_TABLE_X_RAMP, 1.0), 0.0, lift])
            uf = self._fit(side, t)[0]
            el, w, ax = self._pts[side]
            hx = ax if np.dot(ax, w - el) > 0 else -ax
            g = self.table.floor_g(el, w, hx, self._table_servo(side, uf, hx))
            new = min(self.table.soft_lift(g - lift, self.table_goal, self.table_band), HAND_TABLE_MAX_LIFT)
            done = abs(new - lift) < 1e-4
            lift = new
            if done:
                break
        self.last[f"{side}_table_lift_cm"] = lift * 100
        return target + np.array([behind * min(lift / HAND_TABLE_X_RAMP, 1.0), 0.0, lift])

    def _table_servo(self, side: str, uf, hx):
        """body real 일 때만 서보 방향 (table_limit.servo_frame). 그 밖은 None."""
        if side not in self.table_servo_sign:
            return None
        m = self.m[side]
        return self.table.servo_frame(hx, m.jaw0(m.q_from_dirs(*uf)), self.table_servo_sign[side], self.table_roll[side])

    def table_preview(self, bones, cal: Optional[HumanCalib] = None, sides=("left", "right")) -> Dict[str, float]:
        """작업 지시 11-b 시작 순서: 사람 지금 자세 그대로의 손 목표가 판 면에서 얼마나 떠 있는지 (m, 여유 뺀 값). 상태를 바꾸지 않는다."""
        cal = cal or self.cal
        out = {}
        for side in sides:
            U, L, W = BONES[side]
            h, e = cal.to_robot(bones[W][0] - bones[U][0]), cal.to_robot(bones[L][0] - bones[U][0])
            hh = unit(h)
            perp = e - np.dot(e, hh) * hh
            n = perp if float(np.linalg.norm(perp)) > HAND_SWIVEL_PERP_CM[0] else self.n_sw[side]
            u, f = self._solve(side, self.Ps[side] + self.k[side] * h, n)
            m = self.m[side]
            el, w, ax = m.points(m.q_from_dirs(u, f))
            hx = ax if np.dot(ax, w - el) > 0 else -ax
            out[side] = self.table.floor_g(el, w, hx, self._table_servo(side, (u, f), hx))
        return out

    def _arm_gap(self, segs, pts) -> float:
        """6) 의 두 팔 거리 (m). 인계 예외가 꺼져 있으면(handover_ext = 0) 예전 식 그대로.
        켜져 있으면 손 끝 handover_ext 를 뺀 겉면 거리와 '집게끼리 거리 − 문턱' 중 작은 것 (gap_goal 과 비교되게 옮김)."""
        if self.handover_ext <= 0.0:
            return segment_gap(*segs["left"], *segs["right"]) - 2 * FOREARM_RADIUS
        _, body, finger = handover_gaps(pts["left"], pts["right"], self.handover_ext)
        return min(body, self.gap_goal + finger - self.handover_finger_min)

    def _pair_mix(self, bones, cal, h, p) -> None:
        """REPORT_56 G3: p (지금 방식의 손목 목표, 닿음 규칙 적용 뒤) 를 잡는 점 쌍 매핑과 섞는다 (제자리에서 바꿈).
        REPORT_56 §9 (떨림 고침): 직전 프레임 로봇 손 방향으로 잡는 점 → 손목을 바꾸던 되먹임(이득 13.3/12.6 > 1 → 번갈아 튐)을 없앴다.
        대신 풀이의 끝점 자체를 손목 + ext · 손 축 (ext = w · 13.3 cm) 으로 늘리고, 그 끝점의 목표 = (1 − w)·손목 목표 + w·잡는 점 목표.
        w = 0 → 지금과 같은 손목 풀이, w = 1 → 로봇 잡는 점을 사람 쌍 매핑 목표에 바로 맞춤. 손 방향 추정이 필요 없다."""
        G = {}
        for side in ("left", "right"):
            U, L, W = BONES[side]
            G[side] = bones[W][0] + self.grip_offset * unit(bones[W][0] - bones[L][0])     # 사람 잡는 점 (언리얼, cm)
        Dh = cal.to_robot(G["left"] - G["right"]) / 100.0                           # 사람 두 잡는 점 사이 (몸 틀, m) — 1:1
        dist = float(np.linalg.norm(Dh)) * 100
        w = float(np.clip((self.pair_far - dist) / (self.pair_far - self.pair_near), 0.0, 1.0))
        self.last["pair_dist_cm"], self.last["pair_w"] = dist, w
        self._ext = {"left": 0.0, "right": 0.0}
        if w <= 0.0:
            return
        q = {}
        for side in ("left", "right"):                                              # 지금 매핑으로 옮긴 잡는 점
            U, L, W = BONES[side]
            if self.pair_center == "robot-hand":
                # 작업 지시 10-c: 사람 손 길이(15 cm)를 배율로 줄이면 8.6 cm 라 로봇 손 13.3 cm 보다 짧아 다 펴도 rmax 의 0.89 에 멈췄다.
                # 손목은 a·b 단계 목표 p 그대로, 손 부분만 로봇 크기로: q = p + 13.3 cm × 사람 손 축 (몸 틀)
                q[side] = p[side] + ROBOT_GRIP_M * unit(cal.to_robot(bones[W][0] - bones[L][0]))
            else:
                q[side] = self.Ps[side] + self.k[side] * cal.to_robot(G[side] - bones[U][0])
        c = 0.5 * (q["left"] + q["right"])
        tgt = {"left": c + 0.5 * Dh, "right": c - 0.5 * Dh}                         # 로봇 잡는 점 목표
        for side in ("left", "right"):
            self._ext[side] = w * ROBOT_GRIP_M
            p[side] = (1.0 - w) * p[side] + w * tgt[side]                           # 끝점 목표

    def _fit(self, side: str, target: np.ndarray):
        """5) 옆 어긋남 고쳐 맞추기: 손목 관절(순기구학)이 target 에 오도록 (u, f) 를 3번 고친다.
        반환 (u, f), 아래팔 선분, 고친 양(풀이 목표 − 목표)."""
        m = self.m[side]
        # 먼저 닿을 수 있는 거리(팔꿈치 한계 ~ 편 팔)로 자른다 — 안 자르면 너무 가까운 목표를 쫓아 고치는 점이
        # 어깨 중심을 넘어가 위팔 방향이 뒤로 뒤집힌다 (yaw_0418 양손 모으기 131.7초에서 확인)
        c = self.Ps[side]
        target = self._reach(side, target)
        self.fit_target[side] = target            # REPORT_47: 마지막으로 푼 목표 (벌림 포함) — 다시 맞춤(A)의 목표
        # 직전 프레임의 고친 양에서 시작하면 한 번만 고쳐도 된다 (처음은 3번)
        warm = self.off_prev.get(side)
        pc = target + warm if warm is not None else target.copy()
        ext = self._ext.get(side, 0.0)
        for _ in range(1 if warm is not None else 3):
            u, f = self._solve(side, pc, self.n_sw[side])
            el_, w, ax_ = m.points(m.q_from_dirs(u, f))
            if ext:                                # REPORT_56 §9: 끝점 = 손목 + ext · 손 축 (팔꿈치에서 멀어지는 쪽)
                w = w + ext * (ax_ if np.dot(ax_, w - el_) > 0 else -ax_)
            nxt = pc + (target - w)
            if np.dot(nxt - c, target - c) <= 0.5 * float(np.dot(target - c, target - c)):
                break                              # 고치는 점이 어깨 쪽으로 너무 들어가면 멈춘다
            pc = nxt
        u, f = self._solve(side, pc, self.n_sw[side])
        el, w, ax = m.points(m.q_from_dirs(u, f))
        self._pts[side] = (el, w, ax)              # 인계 예외 거리용 (6)
        if ext:                                    # 다시 맞춤(REPORT_47)은 손목 목표를 쓴다 → 이 풀이의 손 축으로 손목 목표로 돌려 둔다
            self.fit_target[side] = target - ext * (ax if np.dot(ax, w - el) > 0 else -ax)
        return (u, f), m.segment_of(el, w, ax), pc - target

    def _eff_geom(self, side: str, ext: float) -> Tuple[float, float, float]:
        """끝점을 손목에서 ext(m) 늘렸을 때 (팔꿈치 → 끝점 길이, 가장 가까운 거리, 편 팔 거리). 영점 자세에서 잰다."""
        key = (side, round(ext, 4))
        if key not in self._eff_cache:
            m = self.m[side]
            c = self.Ps[side]
            e, w, ax = m.points(dict(Q_REF_REST[side], shoulder_yaw=0.0, elbow_roll=0.0))
            tip = w + ext * (ax if np.dot(ax, w - e) > 0 else -ax)
            lb = float(np.linalg.norm(tip - e))
            rmin = math.sqrt(self.La[side] ** 2 + lb ** 2 + 2 * self.La[side] * lb * math.cos(self._bend[side]))
            self._eff_cache[key] = (lb, rmin, float(np.linalg.norm(tip - c)))
        return self._eff_cache[key]

    def _reach(self, side: str, target: np.ndarray) -> np.ndarray:
        """목표를 어깨 중심에서 닿을 수 있는 거리(팔꿈치 한계까지 굽힘 ~ 편 팔)로 자른다."""
        c = self.Ps[side]
        d = target - c
        r = float(np.linalg.norm(d))
        if r < 1e-9:
            return target
        ext = self._ext.get(side, 0.0)
        if ext:                                    # REPORT_56 §9: 끝점이 늘어난 만큼 닿는 거리도
            _, rmin, rmax = self._eff_geom(side, ext)
            if r < rmin and d[1] ** 2 + d[2] ** 2 < rmin ** 2:
                # 너무 가까우면 어깨에서 바깥(방사)이 아니라 **앞으로** 민다 — 좌우·높이를 지켜야 두 잡는 점 사이 (1:1) 가 안 줄어든다
                # (방사로 밀면 몸 가운데 쪽 목표가 더 안쪽으로 가서 사람 20 cm → 로봇 12.9 cm). w = 0 쪽은 방사 → 섞는 비율로 이어 붙임 (튐 없음)
                fwd = np.array([math.sqrt(rmin ** 2 - d[1] ** 2 - d[2] ** 2), d[1], d[2]])
                m = d / r * rmin + min(ext / ROBOT_GRIP_M, 1.0) * (fwd - d / r * rmin)
                return c + m / float(np.linalg.norm(m)) * rmin
            return c + d / r * min(max(r, rmin), rmax)
        return c + d / r * min(max(r, self.rmin[side]), self.rmax[side])


# --------------------------------------------------------------------------- 자체 검사

def self_test() -> int:
    rng = np.random.default_rng(0)
    ok = True
    for side in ("left", "right"):
        m = ArmModel(side)
        w = m.w
        c1 = abs(np.dot(w["shoulder_yaw"], w["elbow_roll"]) - 1) < 1e-6
        c2 = abs(np.dot(w["elbow_pitch"], w["shoulder_yaw"])) < 1e-4
        print(f"  {side}: 영점에서 위팔 축 = 손목 축 {'✅' if c1 else '❌'}   팔꿈치 축 ⟂ 위팔 축 {'✅' if c2 else '❌'}")
        ok &= c1 and c2
        u0, _ = m.fk_dirs(dict(Q_REF_REST[side], shoulder_yaw=0.0, elbow_roll=0.0))
        c3 = ang(u0, [0, 0, -1]) < 1e-4
        print(f"  {side}: 어깨 옆 ∓0.262 에서 위팔이 수직 {'✅' if c3 else '❌'} ({math.degrees(ang(u0, [0, 0, -1])):.4f}°)")
        ok &= c3
        emax = 0.0; rmax = 0.0
        for _ in range(500):
            q = {"shoulder_pitch": rng.uniform(-1.4, 1.4), "shoulder_roll": rng.uniform(-0.3, 1.3) * SIGMA[side],
                 "shoulder_yaw": rng.uniform(-1.2, 1.2), "elbow_pitch": SIGMA[side] * rng.uniform(0.3, 1.8),
                 "elbow_roll": rng.uniform(-1, 1)}
            a, b = m.fk_dirs(q); a2, b2 = m.fk_dirs_chain(q)
            emax = max(emax, ang(a, a2), ang(b, b2))
            sp, sr, conf = m.ik_shoulder(a, q["shoulder_roll"])
            if conf < 1:
                continue
            sy, ep = m.ik_twist(sp, sr, a, b)
            a3, b3 = m.fk_dirs({"shoulder_pitch": sp, "shoulder_roll": sr, "shoulder_yaw": sy, "elbow_pitch": ep,
                                "elbow_roll": 0.0})
            rmax = max(rmax, ang(a, a3), ang(b, b3))
        c4, c5 = emax < math.radians(1e-3), rmax < math.radians(1e-2)
        print(f"  {side}: 방향 FK = URDF 체인 {'✅' if c4 else '❌'} (최대 {math.degrees(emax):.1e}°)   "
              f"IK→FK 되돌림 {'✅' if c5 else '❌'} (최대 {math.degrees(rmax):.1e}°)")
        ok &= c4 and c5
        # 부호: 앞으로 들기 / 옆으로 들기 / 팔꿈치 굽힘 / 안쪽 비틀기 가 REPORT_18 §10.1 과 같은가
        cal = HumanCalib(fwd=np.array([1.0, 0, 0]), right=np.array([0, 1.0, 0]))
        r = PosRetargeter(side, cal, model=m)
        out = {}
        for name, uu, ff in (("앞", [0.5, 0, -0.866], [0.5, 0, -0.866]),
                             ("옆", [0, 0.5 * SIGMA[side], -0.866], [0, 0.5 * SIGMA[side], -0.866])):
            sp, sr, _ = m.ik_shoulder(unit(uu), r.sr_prev)
            out[name] = (sp - r.q_ref["shoulder_pitch"], sr - r.q_ref["shoulder_roll"])
        _, ep = m.ik_twist(0, r.q_ref["shoulder_roll"], np.array([0, 0, -1.0]), np.array([1.0, 0, 0]))
        inward = np.array([0.8, -0.6 * SIGMA[side], 0])      # 손이 몸 쪽(왼팔은 −y, 오른팔은 +y)
        sy_in, _ = m.ik_twist(0, r.q_ref["shoulder_roll"], np.array([0, 0, -1.0]), unit(inward))
        exp = {"left": (-1, +1, +1, +1), "right": (+1, -1, -1, -1)}[side]
        got = (np.sign(out["앞"][0]), np.sign(out["옆"][1]), np.sign(ep), np.sign(sy_in - r.sy_ref))
        c6 = tuple(int(g) for g in got) == exp
        print(f"  {side}: 부호 (앞·옆·굽힘·안쪽 비틀기) = {tuple(int(g) for g in got)} 기대 {exp} {'✅' if c6 else '❌'}"
              f"   어깨 비틀기 기준 {r.sy_ref:+.3f} rad")
        ok &= c6
    print("자체 검사:", "통과" if ok else "실패")
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    raise SystemExit(self_test() if a.self_test else ap.print_help())
