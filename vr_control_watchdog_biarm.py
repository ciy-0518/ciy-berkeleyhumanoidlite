#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Receive test1 Unreal body-tracking UDP JSON and control both 5-DOF arms of
Berkeley Humanoid Lite.

Unreal sends to 192.168.0.65:5000. This receiver binds to 0.0.0.0:5000 by
default so it can receive packets addressed to the robot's 192.168.0.65 NIC.

Controls:
  c: save the current VR pose as neutral
  p: enter POSITION mode from the robot's current calibrated pose
  h: hold the current targets
  x: enter DAMPING mode
  Ctrl+C: stop and enter IDLE

Start with --dry-run. Verify every joint direction before using real motors.

This version adds:
  - a runtime divergence watchdog: every loop, the measured position
    returned by write_read_pdo_2() is compared against the command that
    was just sent. If they differ by more than
    DEFAULT_DIVERGENCE_THRESHOLD_RAD, all motors are switched to DAMPING
    immediately, without waiting for a human to react. The threshold is a
    code constant (not a CLI flag) by design.
  - automatic position_offset cleanup at startup (some other scripts in
    this repo write a stray POSITION_CONTROLLER_POSITION_OFFSET; if it's
    left non-zero, enter_position_from_current()'s raw SDO read and the
    PDO_2 path disagree by exactly that offset, which looks like runaway
    divergence).
  - a per-joint scale factor in RETARGET_CONFIG (separate from the
    max_motion clamp). The clamp only caps the worst case; scale
    proportionally shrinks the whole signal, which is what's needed when
    a joint picks up cross-talk from a neighboring motion.
  - dual-CAN-bus support: left and right arms are on physically separate
    SocketCAN interfaces (can0 / can1 by default), so the motor
    controller now keeps one Bus per side instead of a single shared one.
    This is what makes --side both actually work.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import os
import socket
import sys
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

try:
    from loop_rate_limiters import RateLimiter
except ImportError:
    class RateLimiter:  # Minimal fallback for dry-run and deployment images.
        def __init__(self, frequency: float):
            self.period = 1.0 / float(frequency)
            self.next_tick = time.monotonic()

        def sleep(self) -> None:
            self.next_tick += self.period
            delay = self.next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                self.next_tick = time.monotonic()


try:
    import berkeley_humanoid_lite_lowlevel.recoil as recoil
except ImportError:
    recoil = None


SIDES = ("left", "right")
# --------------------------------------------------------------------------- 패치 2 (2026-09-23)
# 위치 기반 변환 (REPORT_19). 워치독과 별개 파일(`_teleop/tools/pos_retarget.py`)을 그대로 가져다 쓴다.
# 몸 틀은 가슴 본의 수직축 회전만 (REPORT_22 에서 가슴/어깨선이 ① 을 모든 세션에서 통과, ② 평균 38%).
POS_TOOLS = Path.home() / "teleop-docs/_teleop/tools"
try:
    if str(POS_TOOLS) not in sys.path:
        sys.path.insert(0, str(POS_TOOLS))
    import pos_retarget as PR  # type: ignore
except Exception as _exc:      # 도구가 없으면 기존(뼈 회전) 방식만 쓴다
    PR = None
    _POS_IMPORT_ERROR = _exc
try:                           # 손목 회전 측정 순서 (REPORT_49, --auto-session wrist) — rx_probe --script wrist 와 같은 정의
    import wrist_session as WSESS  # type: ignore
except Exception:
    WSESS = None
try:                           # 그리퍼 입력 측정 문구 (REPORT_50, rx_probe --script grip 전용) — cue_player 대조용으로만
    import grip_session as GSESS  # type: ignore
except Exception:
    GSESS = None
try:                           # 위팔 측정 문구 (REPORT_52, rx_probe --script upper 전용) — cue_player 대조용으로만
    import upper_session as USESS  # type: ignore
except Exception:
    USESS = None
try:                           # 양손 장면 문구 (REPORT_54 P2, rx_probe --script both 전용) — cue_player 대조용으로만
    import both_session as BSESS  # type: ignore
except Exception:
    BSESS = None

# REPORT_18 §4 안전 범위 — IDLE 에서 손으로 "편하게 간 곳" (기구 한계가 아니다).
# 차렷 기준값(로봇 기준값 파일의 rest)에 대한 **상대각(도)**.
SAFE_RANGE_DEG = {
    "left": {"shoulder_pitch": (-95.0, 40.0), "shoulder_roll": (-14.0, 107.0),
             "shoulder_yaw": (-49.0, 49.0), "elbow_pitch": (-14.0, 92.0), "elbow_roll": (-79.0, 97.0)},
    "right": {"shoulder_pitch": (-53.0, 99.0), "shoulder_roll": (-99.0, 16.0),
              "shoulder_yaw": (-46.0, 59.0), "elbow_pitch": (-95.0, 19.0), "elbow_roll": (-81.0, 92.0)},
}
SAFE_MARGIN = 0.05       # 양쪽으로 폭의 5% 씩 안으로 줄인다

# 만세용 — 들어 올리는 쪽 끝만 실측 기구 한계로 넓힌다 (2026-09-30 사용자 승인, REPORT_30 제안 B).
# 값 = --part M 연속 기록 최대 − 15° (멈춘 이유 미기재 → 전 방향 −15°). 차렷 기준 상대각(도), 5% 여유는 더 안 뺀다.
#   왼팔 앞 180.2° → −165, 왼팔 옆 169.3° → +154, 오른팔 앞 182.0° → +167, 오른팔 옆 170.5° → −155
SAFE_END_OVERRIDE_DEG = {
    "left": {"shoulder_pitch": ("lo", -165.0), "shoulder_roll": ("hi", 154.0)},
    "right": {"shoulder_pitch": ("hi", 167.0), "shoulder_roll": ("lo", -155.0)},
}


# 어깨 비틀기 겉 범위 (2026-10-03 사용자 승인, REPORT_36 §5). 차렷 기준 상대각(도), 5% 여유를 더 빼지 않는다.
# 안쪽은 위팔을 앞으로 든 각에 따라 30° ~ 왼 54 / 오른 58° 로 바뀐다 — 그 부분은 update_targets 가 매 루프 자른다
# (식은 pos_retarget.yaw_window_deg, IK 와 같다). 여기 값은 그 겉 범위.
SAFE_YAW_ENVELOPE_DEG = {"left": (-54.0, 54.0), "right": (-58.0, 54.0)}


def apply_yaw_in_limit(left_deg: Optional[float], right_deg: Optional[float]) -> None:
    """REPORT_48 (--yaw-in-limit-deg): 비틀기 안쪽 한계를 위팔 든 각과 상관없이 이 값으로. 자르는 곳 모두에 같이 건다:
    ① pos_retarget 범위 규칙 yaw_window_deg (IK · 워치독 매 루프 _apply_yaw_window · hand 다시 맞춤이 모두 이 식을 쓴다)
    ② pos_retarget ROBOT_LIMITS_REL_DEG (오프라인 도구 기본값)  ③ 여기 SAFE_YAW_ENVELOPE_DEG (정적 관절 한계 → IK limits_rel).
    안쪽 = 왼 +, 오른 −. 바깥(54°)은 그대로. 반드시 limits_from_robot_ref() 보다 먼저 부른다."""
    if left_deg is None:
        return
    if PR is not None:
        PR.set_yaw_in_limit(left_deg, right_deg)
    SAFE_YAW_ENVELOPE_DEG["left"] = (SAFE_YAW_ENVELOPE_DEG["left"][0], float(left_deg))
    SAFE_YAW_ENVELOPE_DEG["right"] = (-float(right_deg), SAFE_YAW_ENVELOPE_DEG["right"][1])


# REPORT_45 §4 시작 확인 (2026-10-04 승인) — 중립 직후, 모터를 켜기 전
HEADING_TAU_SOFT_S = 2.0          # 작업 지시 12-c: --heading-soft-deg 를 줄 때 --heading-tau-s 기본값
START_GAP_MIN_CM = 30.0          # 중립 때 두 손목 좌우 간격(왼 − 오른, 왼쪽 +)이 이보다 작으면 바꿔 쥔 것으로 본다 (정상 +46, 바꿔 쥠 −6)
START_SCALE_RANGE = (0.50, 0.70)  # hand 배율 (로봇 편 팔 ÷ 사람 차렷 어깨→손목) 이 밖이면 차렷이 아니었던 것으로 본다


def limits_from_robot_ref(ref, sides) -> Dict[str, Dict[str, Tuple[float, float]]]:
    """REPORT_18 안전 범위를 이번 전원 사이클의 차렷 기준값에 붙인다 (영점이 24° 씩 뛰므로).
    들어 올리는 쪽 끝은 SAFE_END_OVERRIDE_DEG (만세용 실측) 로 바꾼다."""
    out: Dict[str, Dict[str, Tuple[float, float]]] = {}
    for side in sides:
        out[side] = {}
        for joint in JOINT_ORDER:
            lo_d, hi_d = SAFE_RANGE_DEG[side][joint]
            span = hi_d - lo_d
            lo = math.radians(lo_d + span * SAFE_MARGIN)
            hi = math.radians(hi_d - span * SAFE_MARGIN)
            if joint == "shoulder_yaw" and side in SAFE_YAW_ENVELOPE_DEG:
                lo, hi = (math.radians(v) for v in SAFE_YAW_ENVELOPE_DEG[side])
            end = SAFE_END_OVERRIDE_DEG.get(side, {}).get(joint)
            if end is not None:
                which, deg = end
                if which == "lo":
                    lo = math.radians(deg)
                else:
                    hi = math.radians(deg)
            base = float(ref.rest[side][joint])
            out[side][joint] = (base + lo, base + hi)
    return out


# 관절 한계 좌우 통일 (--mirror-limits on, 2026-10-14 사용자 승인). 다섯 관절 모두 URDF 축이 좌우 거울이라
# 차렷 기준 상대각에서 "오른 = −왼" 이 같은 방향이다. 방향마다 더 넓은 쪽(절댓값이 큰 쪽)으로 양팔을 맞춘다.
# 이름은 왼팔 부호 기준 (아래 끝, 위 끝). 오른팔은 부호를 뒤집어 읽는다.
MIRROR_END_NAMES = {"shoulder_pitch": ("앞으로 들기", "뒤로"), "shoulder_roll": ("몸 쪽", "옆으로 벌리기"),
                    "shoulder_yaw": ("바깥", "안쪽"), "elbow_pitch": ("편 끝 너머", "굽힘"),
                    "elbow_roll": ("뒤침(손바닥 위)", "엎침(손등 위)")}


def mirror_limits(limits, ref, sides):
    """양팔 관절 한계를 방향마다 넓은 쪽으로 통일한다. 비틀기 범위 규칙(pos_retarget yaw_window_deg 의 안쪽·바깥 값)도 같이.
    반드시 apply_yaw_in_limit() · limits_from_robot_ref() 뒤에 부른다. 반환 (새 한계, 기록용 dict). 한 팔만이면 그대로."""
    if set(sides) != {"left", "right"}:
        return limits, {"on": True, "note": "한 팔만 — 그대로"}
    yaw_rule = {}
    for name in ("YAW_IN_RAISED_DEG", "YAW_OUT_DEG"):
        d = getattr(PR, name)
        yaw_rule[name] = {"from": dict(d), "to": max(d.values())}
        d["left"] = d["right"] = max(d.values())
    if PR.YAW_IN_OVERRIDE_DEG is not None:
        v = max(PR.YAW_IN_OVERRIDE_DEG.values())
        yaw_rule["YAW_IN_OVERRIDE_DEG"] = {"from": dict(PR.YAW_IN_OVERRIDE_DEG), "to": v}
        PR.set_yaw_in_limit(v, v)
    else:
        PR.set_yaw_in_limit(None)                 # 오프라인 기본표(ROBOT_LIMITS_REL_DEG) 비틀기도 새 값으로
    out = {s: dict(limits[s]) for s in sides}
    rel = {s: {} for s in sides}
    widened = []
    for j in JOINT_ORDER:
        rl, rr = float(ref.rest["left"][j]), float(ref.rest["right"][j])
        L = (limits["left"][j][0] - rl, limits["left"][j][1] - rl)
        R = (limits["right"][j][0] - rr, limits["right"][j][1] - rr)
        lo, hi = min(L[0], -R[1]), max(L[1], -R[0])           # 왼팔 부호 기준
        new = {"left": (limits["left"][j][0] if lo == L[0] else rl + lo, limits["left"][j][1] if hi == L[1] else rl + hi),
               "right": (limits["right"][j][0] if -hi == R[0] else rr - hi, limits["right"][j][1] if -lo == R[1] else rr - lo)}
        for side, old in (("left", L), ("right", (-R[1], -R[0]))):
            for k, (a, b) in enumerate(((old[0], lo), (old[1], hi))):
                if a != b:
                    widened.append({"side": side, "joint": j, "end": MIRROR_END_NAMES[j][k],
                                    "from_deg": round(math.degrees(a), 2), "to_deg": round(math.degrees(b), 2),
                                    "wider_deg": round(abs(math.degrees(b - a)), 2)})
        out["left"][j], out["right"][j] = new["left"], new["right"]
        rel["left"][j] = [round(math.degrees(lo), 2), round(math.degrees(hi), 2)]
        rel["right"][j] = [round(math.degrees(-hi), 2), round(math.degrees(-lo), 2)]
    return out, {"on": True, "rel_deg": rel, "widened": widened, "yaw_rule": yaw_rule}


def newest_robot_ref(explicit: Optional[str]) -> Optional[Path]:
    if explicit:
        path = Path(explicit).expanduser()
        return path if path.exists() else None
    found = sorted((Path.home() / "teleop-docs/_teleop/logs").glob("robot_ref_*.json"))
    return found[-1] if found else None


def motor_to_urdf_fn(ref, sides, models=None):
    """모터값(command_target 꼴) → URDF 관절각 함수 q_of(side, c). 손목(elbow_roll)은 0 (위치에 영향 없음).
    어깨 앞뒤·옆·팔꿈치 = Q_REF_REST + (모터 − rest), 비틀기 = sy_ref + (모터 − e90) (motor_targets P:792 의 역).
    충돌 방지(make_arm_gap_fn)와 VR 표시(--robot-viz, tools/robot_viz.py)가 같이 쓴다."""
    models = models or {s: PR.ArmModel(s) for s in sides}
    q0 = {s: dict(PR.Q_REF_REST[s]) for s in sides}
    sy_ref = {s: models[s].ik_twist(q0[s]["shoulder_pitch"], q0[s]["shoulder_roll"], np.array([0, 0, -1.0]),
                                    np.array([1.0, 0, 0]))[0] for s in sides}

    def q_of(side, c):
        r, e = ref.rest[side], ref.e90[side]
        return {"shoulder_pitch": q0[side]["shoulder_pitch"] + (c["shoulder_pitch"] - r["shoulder_pitch"]),
                "shoulder_roll": q0[side]["shoulder_roll"] + (c["shoulder_roll"] - r["shoulder_roll"]),
                "shoulder_yaw": sy_ref[side] + (c["shoulder_yaw"] - e["shoulder_yaw"]),
                "elbow_pitch": q0[side]["elbow_pitch"] + (c["elbow_pitch"] - r["elbow_pitch"]),
                "elbow_roll": 0.0}
    return q_of


def make_arm_gap_fn(ref, sides):
    """모터 명령(command_target 꼴) → 두 아래팔(팔꿈치→손끝) 겉면 거리 [m] (REPORT_36). URDF 모델 + 로봇 기준값."""
    models = {s: PR.ArmModel(s) for s in sides}
    q_of = motor_to_urdf_fn(ref, sides, models)

    def gap(cmd):
        a = models["left"].forearm_segment(q_of("left", cmd["left"]))
        b = models["right"].forearm_segment(q_of("right", cmd["right"]))
        return PR.segment_gap(a[0], a[1], b[0], b[1]) - 2 * PR.FOREARM_RADIUS
    return gap


def make_handover_gap_fn(ref, sides):
    """작업 지시 10 인계 예외: 모터 명령 → (full, body, finger) [m] (pos_retarget.handover_gaps). full 은 make_arm_gap_fn 과 같은 값."""
    models = {s: PR.ArmModel(s) for s in sides}
    q_of = motor_to_urdf_fn(ref, sides, models)

    def gaps(cmd, ext_m):
        return PR.handover_gaps(models["left"].points(q_of("left", cmd["left"])),
                                models["right"].points(q_of("right", cmd["right"])), ext_m)
    return gaps


def table_servo_signs(ref, sides) -> Dict[str, float]:
    """작업 지시 12-c body real: e90 에서 서보가 위 (사진 2026-11-02) 가 되는 servo_frame 부호. 팔마다."""
    models = {s: PR.ArmModel(s) for s in sides}
    q_of = motor_to_urdf_fn(ref, sides, models)
    out = {}
    for s in sides:
        q = q_of(s, ref.e90[s])
        el, wr, ax = models[s].points(q)
        hx = ax if float(np.dot(ax, wr - el)) > 0 else -ax
        out[s] = 1.0 if float(np.cross(hx, models[s].jaw0(q))[2]) > 0 else -1.0
    return out


def make_table_clear_fn(ref, sides, table):
    """작업 지시 11-b: 팔 하나의 모터 명령 → 판까지 거리 dict (table_limit.TableLimit.clear). 손목은 쓰지 않는다 (몸체 outer 는 손목과 무관)."""
    models = {s: PR.ArmModel(s) for s in sides}
    q_of = motor_to_urdf_fn(ref, sides, models)
    signs = table_servo_signs(ref, sides) if table.body == "real" else None

    def clear(side, c):
        q = q_of(side, c)
        el, wr, ax = models[side].points(q)
        hx = ax if float(np.dot(ax, wr - el)) > 0 else -ax
        if signs is None:
            return table.clear(el, wr, hx)
        return table.clear(el, wr, hx, table.servo_frame(hx, models[side].jaw0(q), signs[side], float(c["elbow_roll"]) - float(ref.e90[side]["elbow_roll"])))
    return clear


def model_park_pose(ref, sides, table_clear):
    """dry-run·시험용 주차 자세 (모델): e90 에서 팔꿈치만 펴 내려 판까지 거리가 0 이 되는 곳 (그리퍼를 판에 올려 둔 자세). 실기동은 엔코더 값을 쓴다."""
    out = {}
    for s in sides:
        pose = dict(ref.e90[s])
        d = float(ref.rest[s]["elbow_pitch"]) - float(ref.e90[s]["elbow_pitch"])
        for t in np.linspace(0.0, 1.0, 201):
            test = dict(pose, elbow_pitch=float(ref.e90[s]["elbow_pitch"]) + float(t) * d)
            if table_clear(s, test)["min"] <= 0.0:
                break
            pose = test
        out[s] = pose
    return out


class HandoverGate:
    """작업 지시 10 인계 예외 (--handover-exempt, 기본 끔): 충돌 판정에서 뺄 손 끝 길이 ext (m) 를 정한다.
    켜짐 조건 = 쌍 매핑 w ≥ pair_min 이고 그리퍼 조건 (any-closed: 하나 이상 닫힘 / both-closed: 둘 다 / pair: 그리퍼 안 봄).
    ext 는 0 ↔ full_m 을 ramp_s 에 걸쳐 바꾼다. 조건이 꺼져도 두 팔 (예외 없는) 거리가 release_m 아래면 유지한다 (밀어내지 않기)."""

    WHEN = ("any-closed", "both-closed", "pair")

    def __init__(self, when: str = "any-closed", pair_min: float = 0.5, ramp_s: float = 0.3, full_m: float = 0.055):
        self.when, self.pair_min, self.ramp_s, self.full_m = when, float(pair_min), float(ramp_s), float(full_m)
        self.ext = 0.0
        self.cond = False
        self.held = False
        self.t_prev: Optional[float] = None
        self.on_count = 0                       # 꺼짐 → 켜짐 횟수 (로그용)

    def update(self, now: float, pair_w: float, grip: Optional[Dict[str, int]], gap_full_m: float, release_m: float) -> float:
        dt = 0.0 if self.t_prev is None else min(max(now - self.t_prev, 0.0), 0.1)
        self.t_prev = now
        closed = [v == 1 for v in (grip or {}).values()]
        if self.when == "pair":
            g_ok = True
        elif self.when == "both-closed":
            g_ok = len(closed) == 2 and all(closed)
        else:
            g_ok = any(closed)
        cond = bool(math.isfinite(pair_w) and pair_w >= self.pair_min and g_ok)
        if cond and not self.cond and self.ext <= 0.0:
            self.on_count += 1
        self.cond = cond
        self.held = (not cond) and self.ext > 0.0 and math.isfinite(gap_full_m) and gap_full_m < release_m
        target = self.full_m if (cond or self.held) else 0.0
        step = self.full_m / max(self.ramp_s, 1e-3) * dt
        self.ext = min(self.ext + step, target) if self.ext < target else max(self.ext - step, target)
        return self.ext


def frame_bones(frame) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    return {name: (item["pos"], item["rot"]) for name, item in frame.joints.items()}


class PosRetargetRunner:
    """사람 관절 **위치** → 로봇 관절각 → 모터 목표값 (절대값). 기존 뼈 회전 방식과 나란히 둔다."""

    def __init__(self, sides: Sequence[str], ref, body_frame: str = "chest_heading",
                 sy_policy: str = "hold", wrist_median: bool = False, limits=None,
                 wrist_hold: bool = True, overhead_elbow_hold: bool = True, elbow_base: bool = True,
                 hand_cfg: Optional[Dict[str, float]] = None, flip_hold: bool = True, hand_refit: bool = True):
        self.sides = tuple(sides)
        # REPORT_38 손 위치 방식 (--retarget hand): HandTargeter 설정 (touch_cm, min_gap_cm, swivel_tau, max_speed)
        self.hand_cfg = dict(hand_cfg or {})
        self.ht = None
        self.table = None           # 작업 지시 11-b --table (calibrate 에서 ht 에 넣는다)
        self.heading_cfg = None     # 작업 지시 12-b --heading-deadband-deg: (무감대 도, 시간 상수 초)
        self.heading = None         # calibrate 에서 만든다 (PR.HeadingDeadband)
        self._hand_key, self._hand_dirs = None, None
        # REPORT_47 (안 A): hand 에서 어깨 옆이 몸 쪽 한계에 걸리면 비틀기·팔꿈치로 손목 목표에 다시 맞춘다
        self.hand_refit = hand_refit
        self._refit: Dict[str, Optional[Dict[str, float]]] = {}   # 프레임마다 계산한 결과 (다음 계산까지 다시 씀)
        self._refit_due: Dict[str, bool] = {}                      # 새 프레임이 와서 아직 계산 안 함
        self.refits = 0                                            # 손질한 루프 수 (양팔 합, 누적, 로그용)
        self.refit_err: Dict[str, float] = {}                      # 마지막 손질 뒤 손목 오차 (cm)
        self.hand_info: Dict[str, float] = {}   # 로그용: 사람 양 손목 거리·좌우 간격, 로봇 좌우 간격 목표 (cm)
        # REPORT_33: 손목 A (팔 편 동안 손목 유지), 머리 위 A (위팔 높으면 팔꿈치 유지), 머리 위 B (차렷 팔꿈치 빼기)
        self.filters = dict(wrist_hold=wrist_hold, overhead_elbow_hold=overhead_elbow_hold, elbow_base=elbow_base,
                            flip_hold=flip_hold)   # flip_hold = R1 (REPORT_42): 등 뒤 팔에서 머리 위로 넘긴 해로 뛰지 않음
        self.ref = ref
        # 🔴 REPORT_31: IK 가 로봇 관절 한계를 안다 — 모터 한계(절대값)를 차렷 기준 상대각으로 바꿔 넘긴다
        # 비틀기(REPORT_35)는 모터 명령이 e90 기준이라 e90 값을 뺀다 (motor_targets 와 같은 기준)
        self.limits_rel = ({side: {j: (limits[side][j][0] - float((ref.e90 if j == "shoulder_yaw" else ref.rest)[side][j]),
                                       limits[side][j][1] - float((ref.e90 if j == "shoulder_yaw" else ref.rest)[side][j]))
                                   for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw")}
                                   for side in self.sides}
                           if limits is not None else {})
        self.limits_abs = limits
        self.body_frame = body_frame
        self.sy_policy = sy_policy
        self.wrist_median = wrist_median   # C (REPORT_29): 손목 엎침각 3프레임 중앙값
        self.extra: Dict[str, Dict[str, float]] = {}   # 로그용: 손목 엎침각·손목 목표·어깨비틀기 계산값·팔 긴축 회전
        self.cal = None
        self.chest_ref = None
        self.rts: Dict[str, object] = {}
        # 로그용 관측값: side -> (u_x, u_y, u_z, 팔꿈치도, 유지가중치), 그리고 몸 틀 heading(도)
        self.last: Dict[str, Tuple[float, float, float, float, float]] = {}
        self.last_heading = float("nan")

    def ready(self) -> bool:
        return self.cal is not None and len(self.rts) == len(self.sides)

    def start_check(self, frames, hand: bool, gap_min_cm: float = START_GAP_MIN_CM,
                    scale_range: Tuple[float, float] = START_SCALE_RANGE) -> Dict[str, object]:
        """REPORT_45 §4: 중립 직후, 모터를 켜기 전 검사. frames = 중립 동안 모은 프레임 (없으면 기준 프레임 하나).
        ① 두 손목 좌우 간격 (몸 틀, 왼 − 오른, 왼쪽 +) 중앙값 < gap_min_cm → 컨트롤러 바꿔 쥠
        ② hand 일 때 배율이 scale_range 밖 → 차렷이 아니었음.  반환 {ok, gap_cm, scale, reason, phrase}"""
        out: Dict[str, object] = {"ok": True, "gap_cm": float("nan"), "scale": {}, "reason": "", "phrase": ""}
        if self.cal is None:
            return out
        gaps = []
        for f in frames:
            b = frame_bones(f)
            if "L_WristTwist" in b and "R_WristTwist" in b:
                gaps.append(float(self.cal.to_robot(b["L_WristTwist"][0] - b["R_WristTwist"][0])[1]))
        if gaps:
            out["gap_cm"] = float(np.median(gaps))
        if hand and self.ht is not None:
            out["scale"] = {s: float(self.ht.k[s] * 100) for s in self.sides}
        if gaps and out["gap_cm"] < gap_min_cm:
            out.update(ok=False, phrase=VOICE_START_SWAP,
                       reason=f"두 손목 좌우 간격 {out['gap_cm']:+.0f} cm (정상 약 +46, 기준 +{gap_min_cm:.0f} 이상) — "
                              "컨트롤러를 좌우 바꿔 쥐었거나 손이 몸 앞에서 엇갈렸다")
        elif hand and out["scale"]:
            bad = {s: k for s, k in out["scale"].items() if not (scale_range[0] <= k <= scale_range[1])}
            if bad:
                out.update(ok=False, phrase=VOICE_START_POSE,
                           reason="배율 " + ", ".join(f"{'왼' if s == 'left' else '오른'} {k:.2f}" for s, k in bad.items())
                                  + f" ({scale_range[0]:.2f}~{scale_range[1]:.2f} 밖) — 중립 때 팔을 쭉 편 차렷이 아니었다")
        return out

    def flip_holds(self) -> int:
        """R1 로 어깨 앞뒤·옆을 유지한 횟수 (양팔 합, 누적). 캘리브레이션을 다시 하면 0 부터."""
        return sum(getattr(rt, "flip_holds", 0) for rt in self.rts.values())

    def calibrate(self, frame, frames=None) -> bool:
        """frame = 기준 프레임. frames = 차렷 동안 모은 프레임들 — 있으면 차렷 팔꿈치 값(머리 위 B)을 그 중앙값으로 잡는다."""
        bones = frame_bones(frame)
        needed = [n for side in self.sides for n in PR.BONES[side]]
        if not all(n in bones for n in needed) or "L_UpperArm" not in bones or "R_UpperArm" not in bones:
            return False
        needed_all = needed + ["L_UpperArm", "R_UpperArm"]
        more = [frame_bones(f) for f in (frames or []) if f is not None]
        more = [b for b in more if all(n in b for n in needed_all)]
        self.cal = PR.HumanCalib.from_bones([bones])
        if more:   # 차렷 팔꿈치 값만 모은 프레임들의 중앙값으로 (몸 틀·손목 0 등 나머지는 기존대로 한 프레임)
            self.cal.elbow_rest = PR.HumanCalib.from_bones(more).elbow_rest
            self.cal.set_arm_lengths(more)          # REPORT_38: 팔 길이·어깨 폭도 차렷 중앙값
        self.chest_ref = bones["Chest"][1] if "Chest" in bones else None
        if self.heading_cfg is not None:      # 작업 지시 12-b: 보정 때 기준을 새로 잡는다 (보정 프레임은 항상 원시 뼈)
            seed = [b for b in (more or [bones]) if "Chest" in b and "L_UpperArm" in b and "R_UpperArm" in b]
            self.heading = (PR.HeadingDeadband(self.heading_cfg[0], self.heading_cfg[1], self.cal, self.chest_ref, seed, *self.heading_cfg[2:])
                            if seed and self.chest_ref is not None else None)
            if self.heading is None:
                print("⚠ 몸 방향 무감대: 가슴 본이 없어 끈다 (지금 방식으로 돈다)")
        self.rts = {side: PR.PosRetargeter(side, self.cal, self.ref, sy_policy=self.sy_policy,
                                           wrist_median=self.wrist_median,
                                           limits_rel=self.limits_rel.get(side), **self.filters)
                    for side in self.sides}
        # 팔꿈치 한계(차렷 기준 굽힘 쪽 끝) → 손이 어깨에 가장 가까이 올 수 있는 거리
        el_max = {}
        if self.limits_abs is not None:
            for s in self.sides:
                lo, hi = self.limits_abs[s]["elbow_pitch"]
                r0 = float(self.ref.rest[s]["elbow_pitch"])
                el_max[s] = (hi - r0) if PR.SIGMA[s] > 0 else (r0 - lo)
        self.ht = PR.HandTargeter(self.cal, {s: rt.m for s, rt in self.rts.items()}, elbow_max=el_max or None,
                                  **self.hand_cfg)
        self.ht.table = self.table
        if self.table is not None and self.table.body == "real":    # 작업 지시 12-c
            self.ht.table_servo_sign = table_servo_signs(self.ref, self.sides)
            self.ht.table_goal = 0.010                               # 집게는 손목에서 멀어 모터가 조금만 늦어도 막힌다 → 손 목표 쪽 여유를 넓힌다 (막힘 1,124 → 0, 재생)
        self._hand_key, self._hand_dirs = None, None
        return True

    def _apply_refit(self, side: str, rt, q: Dict[str, float], new_frame: bool) -> Dict[str, float]:
        """REPORT_47: 프레임마다 한 번만 다시 맞춤을 계산하고, 같은 프레임 동안은 그 결과를 다시 쓴다 (루프 부담 ↓).
        계산은 새 프레임이 온 **다음 루프**에서 한다 — 새 프레임 루프는 손 목표 풀이(1~6 ms)가 이미 무거워서, 같은 루프에서
        하면 루프가 늦었다 (dry-run 늦음 5 → 18번). 새 프레임 루프에서는 직전 결과를 그대로 쓴다.
        어깨 옆이 몸 쪽 한계 안이면 손대지 않는다."""
        due = self._refit_due.get(side, False)
        if new_frame and not due:
            self._refit_due[side] = True               # 다음 루프에서 계산
            compute = False
        else:
            compute = due                              # 미뤄 둔 계산 (프레임이 매 루프 오면 그 루프에서 바로)
            if new_frame:
                self._refit_due[side] = True
        if compute:
            if not new_frame:
                self._refit_due[side] = False
            tgt = self.ht.fit_target.get(side) if self.ht is not None else None
            el_lim = None
            if self.limits_abs is not None:
                lo, hi = self.limits_abs[side]["elbow_pitch"]
                r0 = float(self.ref.rest[side]["elbow_pitch"])
                el_lim = (lo - r0, hi - r0)          # 팔꿈치 q = 모터 − 차렷 (k = +1, q_ref 팔꿈치 0)
            self._refit[side] = rt.hand_refit(q, tgt, el_lim) if tgt is not None else None
        res = self._refit.get(side)
        if res is None:
            return q
        (_, _), (rlo, rhi) = rt.windows()
        if (side == "left" and q["shoulder_roll"] >= rlo) or (side != "left" and q["shoulder_roll"] <= rhi):
            return q                                 # 이 루프에서는 한계 안 — 손대지 않는다
        self.refits += 1
        self.refit_err[side] = res["err_cm"]
        return dict(q, shoulder_roll=res["shoulder_roll"], shoulder_yaw=res["shoulder_yaw"],
                    elbow_pitch=res["elbow_pitch"])

    def hand_calib_text(self) -> str:
        """시작 때 찍는 손 위치 방식 기준 (사람 팔 길이·어깨 폭, 배율)."""
        if self.cal is None or self.ht is None:
            return "-"
        c = self.cal
        return (" / ".join(f"{s[0].upper()} 위팔 {c.upper_len.get(s, float('nan')):.0f}·아래팔 {c.fore_len.get(s, float('nan')):.0f}"
                           f"·어깨→손목 {c.reach_rest.get(s, float('nan')):.0f} cm (배율 {self.ht.k[s] * 100:.2f})"
                           for s in self.sides)
                + f" · 어깨 폭 {c.shoulder_width:.0f} cm")

    def _hand_obs(self, bones, cal) -> None:
        """로그용: 사람 양 손목 거리(3D)·좌우 간격 (cm). 어느 방식이든 남긴다."""
        try:
            d = bones["L_WristTwist"][0] - bones["R_WristTwist"][0]
            self.hand_info = {"human_wrist_dist_cm": float(np.linalg.norm(d)),
                              "human_gap_cm": float(cal.to_robot(d)[1]),
                              "robot_gap_target_cm": float("nan")}
        except Exception:
            self.hand_info = {}

    def elbow_deg(self, frame, side: str) -> float:
        """이 프레임의 사람 팔꿈치 각 (°). 계산 못 하면 nan."""
        try:
            u, f = self.rts[side].dirs(frame_bones(frame))
            return math.degrees(PR.ang(u, f))
        except Exception:
            return float("nan")

    def calibrate_wrist(self, frames) -> Dict[str, float]:
        """A (REPORT_29): 사람 손목 0 을 "팔꿈치 90° 엄지 위" 프레임들로 다시 잡는다.
        반환: side → 차렷 때 잡은 0 대비 바뀐 양(°)."""
        old = {s: self.cal.wrist_rel0[s].copy() for s in self.sides}
        self.cal.set_wrist_zero([frame_bones(f) for f in frames], self.sides)
        shift = {}
        for s in self.sides:
            shift[s] = math.degrees(PR.HUMAN_PRONATION_SIGN[s]
                                    * PR.twist_y(PR.qmul(PR.qinv(old[s]), self.cal.wrist_rel0[s])))
            rt = self.rts.get(s)
            if rt is not None:
                rt._pron_hist = []
        return shift

    def elbow_base_deg(self) -> Dict[str, float]:
        """머리 위 B: 이번 중립에서 잡은 사람 차렷 팔꿈치 값(°)."""
        return {s: math.degrees(self.cal.elbow_rest.get(s, float("nan"))) for s in self.sides} if self.cal else {}

    def set_policy(self, policy: str) -> None:
        self.sy_policy = policy
        for rt in self.rts.values():
            rt.sy_policy = policy

    def observe(self, frame) -> None:
        """뼈 방식으로 도는 중에도 사람 팔 방향을 로그에 남기기 위해 계산만 한다 (상태를 건드리지 않는다)."""
        if not self.ready():
            return
        bones = self.view_bones(frame)
        cal = self._frame_cal(bones)
        self._record_heading(cal)
        self._hand_obs(bones, cal)
        for side in self.sides:
            rt = self.rts[side]
            try:
                saved = rt.cal
                rt.cal = cal
                u, f = rt.dirs(bones)
                rt.cal = saved
            except Exception:
                continue
            self.last[side] = (float(u[0]), float(u[1]), float(u[2]),
                               math.degrees(PR.ang(u, f)), float("nan"))

    def _record_heading(self, cal) -> None:
        if self.cal is None:
            return
        self.last_heading = math.degrees(math.atan2(float(np.dot(cal.fwd, self.cal.right)),
                                                    float(np.dot(cal.fwd, self.cal.fwd))))

    def view_bones(self, frame):
        """손 목표·표시가 쓰는 뼈. --heading-deadband-deg 를 켜면 어깨만 그 값으로 바뀐다 (끄면 frame_bones 그대로)."""
        bones = frame_bones(frame)
        return bones if self.heading is None else self.heading.view(bones, frame.received_at)

    def _frame_cal(self, bones):
        """몸 틀: 가슴 본의 **수직축 회전만** 캘리브레이션 대비로 돌린다."""
        if self.heading is not None:
            cal = copy.copy(self.cal)
            cal.fwd = self.heading.fwd()
            cal.right, cal.up = np.cross(PR.UP_UE, cal.fwd), PR.UP_UE
            return cal
        if self.body_frame == "fixed" or self.chest_ref is None or "Chest" not in bones:
            return self.cal
        rel = PR.qmul(bones["Chest"][1], PR.qinv(self.chest_ref))
        fwd = PR.qrot(rel, self.cal.fwd)
        fwd = PR.unit(fwd - float(np.dot(fwd, PR.UP_UE)) * PR.UP_UE)
        cal = copy.copy(self.cal)
        cal.fwd, cal.right, cal.up = fwd, np.cross(PR.UP_UE, fwd), PR.UP_UE
        return cal

    def targets(self, frame, hand: bool = False) -> Optional[Dict[str, Dict[str, float]]]:
        """hand = True 면 손 위치 방식 (REPORT_38): HandTargeter 가 만든 로봇 팔 방향을 같은 IK·한계에 넣는다."""
        bones = self.view_bones(frame)
        cal = self._frame_cal(bones)
        self._record_heading(cal)
        self._hand_obs(bones, cal)
        hd = None
        new_frame = False
        if hand:
            # 손 목표는 새 프레임(약 15 Hz)일 때만 다시 푼다 (한 번에 1~6 ms — 200 Hz 루프마다 풀면 루프가 밀린다)
            key = (id(frame), getattr(frame, "received_at", None))
            try:
                if self._hand_key != key or self._hand_dirs is None:
                    self._hand_dirs = self.ht.dirs(bones, cal, sides=self.sides)
                    self._hand_key = key
                    new_frame = True
                hd = self._hand_dirs
                self.hand_info["robot_gap_target_cm"] = self.ht.last.get("robot_gap_target_cm", float("nan"))
                self.hand_info["hand_widen_cm"] = self.ht.last.get("widen_cm", float("nan"))
            except Exception:
                return None
        out: Dict[str, Dict[str, float]] = {}
        for side in self.sides:
            rt = self.rts[side]
            rt.cal = cal
            rt.hand_mode = hand
            try:
                u, f = rt.dirs(bones) if hd is None else hd[side]
                q = rt.step(bones, dirs=None if hd is None else hd[side])
                if hd is not None and self.hand_refit:
                    q = self._apply_refit(side, rt, q, new_frame)
                out[side] = rt.motor_targets(q)
                if self.ht is not None and side in self.ht.table_servo_sign:   # 작업 지시 12-c body real: 다음 프레임 손 목표에 쓸 손목 각
                    self.ht.table_roll[side] = float(out[side]["elbow_roll"]) - float(self.ref.e90[side]["elbow_roll"])
                self.last[side] = (float(u[0]), float(u[1]), float(u[2]),
                                   math.degrees(PR.ang(u, f)), float(q["hold_w"]))
                self.extra[side] = {
                    "wrist_pron_deg": math.degrees(q["wrist_pron_raw"]),     # 사람 입력 (거르기 전)
                    "wrist_target": out[side]["elbow_roll"],
                    "sy_ik": math.degrees(q["sy_ik"]) if math.isfinite(q["sy_ik"]) else float("nan"),
                    "arm_twist_deg": math.degrees(q["arm_twist"]) if math.isfinite(q["arm_twist"]) else float("nan"),
                }
            except Exception:
                return None
        return out


JOINT_ORDER = (
    "shoulder_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow_pitch",
    "elbow_roll",
)

DEFAULT_ARM_IDS = {
    "left": (1, 3, 5, 7, 9),
    "right": (2, 4, 6, 8, 10),
}

DEFAULT_CHANNELS = {
    "left": "can0",
    "right": "can1",
}

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

# 가슴(몸통) 기준 본. 좌우 공통이라 VR_JOINTS 와 분리해 둔다.
# 🔴 VR_JOINTS 에 넣으면 안 된다 — has_required_joints() 가 그 이름들을 전부 요구하므로,
# Chest 를 보내지 않는 구 언리얼 빌드에서 팔이 통째로 멈춘다.
CHEST_JOINT_NAME = "Chest"

URDF_JOINT_NAMES = {
    "left": {
        "shoulder_pitch": "arm_left_shoulder_pitch_joint",
        "shoulder_roll": "arm_left_shoulder_roll_joint",
        "shoulder_yaw": "arm_left_shoulder_yaw_joint",
        "elbow_pitch": "arm_left_elbow_pitch_joint",
        "elbow_roll": "arm_left_elbow_roll_joint",
    },
    "right": {
        "shoulder_pitch": "arm_right_shoulder_pitch_joint",
        "shoulder_roll": "arm_right_shoulder_roll_joint",
        "shoulder_yaw": "arm_right_shoulder_yaw_joint",
        "elbow_pitch": "arm_right_elbow_pitch_joint",
        "elbow_roll": "arm_right_elbow_roll_joint",
    },
}

# arm_config.yaml의 실측값(안전 여유 5% 적용됨)으로 교체.
# 기존엔 일반 URDF 모델의 이론값이 들어있었는데, 이 로봇 실측 범위와
# 전혀 일치하지 않았다(부호가 반대인 관절도 있었음). --urdf로 override 가능.
# yaml의 "wrist_yaw"(id 9/10)는 이 코드의 elbow_roll과 같은 모터다.
DEFAULT_URDF_LIMITS = {
    "left": {
        "shoulder_pitch": (-10.0, 10.0),
        "shoulder_roll":  (-10.0, 10.0),
        "shoulder_yaw":   (-10.0, 10.0),
        "elbow_pitch":    (-10.0, 10.0),
        "elbow_roll":     (-10.0, 10.0),
    },
    "right": {
        "shoulder_pitch": (-10.0, 10.0),
        "shoulder_roll":  (-10.0, 10.0),
        "shoulder_yaw":   (-10.0, 10.0),
        "elbow_pitch":    (-10.0, 10.0),
        "elbow_roll":     (-10.0, 10.0),
    },
}

# Unreal quaternion delta to robot joint delta.
# Tuple format: (source, axis, sign, scale, max_motion)
#   source     : which relative quaternion to read (upper_delta / elbow_delta / wrist_delta)
#   axis       : which Euler component of that quaternion (roll / pitch / yaw)
#   sign       : +1.0 or -1.0, direction correction (verified empirically per joint)
#   scale      : proportional gain applied BEFORE clamping. 1.0 = no attenuation.
#                Use < 1.0 to dampen a joint that picks up cross-talk from a
#                neighboring motion (this shrinks the whole signal, unlike
#                max_motion which only caps the worst case).
#   max_motion : final clamp, radians, applied after scale.
RETARGET_CONFIG = {
    "left": {
        "shoulder_pitch": ("upper_delta", "roll",  +1.0, 1.0, 2.5),   # ≈ 143°
        "shoulder_roll":  ("upper_delta", "yaw",   -1.0, 1.0, 2.5),
        "shoulder_yaw":   ("upper_delta", "pitch", +1.0, 1.0, 2.5),
        "elbow_pitch":    ("elbow_delta", "roll",  -1.0, 1.0, 2.5),
        "elbow_roll":     ("wrist_delta", "pitch", +1.0, 1.0, 2.5),
    },
    "right": {
        "shoulder_pitch": ("upper_delta", "roll",  -1.0, 1.0, 2.5),
        "shoulder_roll":  ("upper_delta", "yaw",   +1.0, 1.0, 2.5),
        "shoulder_yaw":   ("upper_delta", "pitch", -1.0, 1.0, 2.5),
        "elbow_pitch":    ("elbow_delta", "roll",  +1.0, 1.0, 2.5),
        "elbow_roll":     ("wrist_delta", "pitch", -1.0, 1.0, 2.5),
    },
}
DEFAULT_KP = 20.0
DEFAULT_KD = 2.0
DEFAULT_TORQUE_LIMIT = 5.0
DEFAULT_UDP_PORT = 5000
LOOP_HZ = 200.0
# 작업 지시 10: 인계 예외 중 집게끼리 거리 문턱 (cm, 한 팔 집게 끝 → 다른 팔 집게 중심선). 근거 ~/cc_analysis/REPORT_handover_*.md:
#   모터 쪽 0.8 = 집게 끝 폭 0.8 (두 끝 중심이 0.8 이면 가장자리가 막 닿음). 손 목표 쪽은 + 0.2 (= 1.0) 로 먼저 멈춰 모터 쪽은 받침.
#   REPORT_reach 주사위 인계 자세: 손 목표 문턱 1.0 에서 2·3 cm 큐브 97.7%, 모터 0.8 에서 100% 통과
HANDOVER_FINGER_GAP_CM = 0.8
HANDOVER_FINGER_CUSHION_CM = 0.2
UDP_TIMEOUT_SEC = 0.50
MAX_TARGET_SPEED_RAD_S = 2.5

# EMA smoothing applied to raw VR joint quaternions, per ARM, only when a
# NEW Unreal packet arrives (Unreal sends at ~15Hz; the robot loop runs at
# 200Hz, so gating on frame.received_at is required -- otherwise the same
# stale quaternion would get blended into the filter ~13 times per new
# packet and the filter would just snap to it, doing nothing).
# Lower alpha = more smoothing / more lag. 1.0 = filter disabled.
# euler_probe_log_left_5.csv showed 'up' flipping sign every ~0.4s while
# the operator held a single slow yaw motion -- that's tracking jitter,
# not intentional motion, and this filter targets exactly that.
DEFAULT_EMA_ALPHA = 0.35

# Twist axis (in the upper_delta quaternion's own xyz, NOT world space) used
# for the shoulder's twist-swing decomposition. This is the Y component,
# because empirical axis calibration already established that upper_delta's
# "pitch" (Euler Y-rotation) is shoulder_yaw. See shoulder_axis_components().
SHOULDER_TWIST_AXIS = np.array([0.0, 1.0, 0.0])

# Position-mode runtime watchdog: if |measured - command| exceeds this many
# radians, all motors are switched to DAMPING immediately. This is a code
# constant by design (not exposed as a CLI flag) so it can't be loosened
# accidentally from the command line.
DEFAULT_DIVERGENCE_THRESHOLD_RAD = 1.0

START_LIMIT_TOLERANCE_RAD = 0.25

# Delay between pressing 'c' and actually capturing the VR neutral pose.
# The operator needs to move their hand to the keyboard to press 'c',
# which disturbs the pose they're trying to calibrate as neutral -- this
# gives them time to get back into position before the snapshot is taken.
CALIBRATION_DELAY_SEC = 2.0


def clamp(value: float, low: float, high: float) -> float:
    return min(max(float(value), float(low)), float(high))


def selected_sides(side_arg: str) -> Tuple[str, ...]:
    return SIDES if side_arg == "both" else (side_arg,)


def zero_targets(sides: Iterable[str]) -> Dict[str, Dict[str, float]]:
    return {
        side: {joint: 0.0 for joint in JOINT_ORDER}
        for side in sides
    }


def parse_id_list(value: str) -> Tuple[int, ...]:
    try:
        result = tuple(int(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("motor IDs must be comma-separated integers") from exc
    if len(result) != 5 or len(set(result)) != 5:
        raise argparse.ArgumentTypeError("exactly five unique motor IDs are required")
    return result


def load_urdf_limits(path: Optional[str]) -> Dict[str, Dict[str, Tuple[float, float]]]:
    limits = {
        side: dict(DEFAULT_URDF_LIMITS[side])
        for side in SIDES
    }
    if not path:
        return limits

    urdf_path = Path(path)
    root = ET.parse(urdf_path).getroot()
    joints_by_name = {
        joint.get("name"): joint
        for joint in root.findall("joint")
        if joint.get("name")
    }

    for side in SIDES:
        for logical_name, urdf_name in URDF_JOINT_NAMES[side].items():
            joint = joints_by_name.get(urdf_name)
            limit = None if joint is None else joint.find("limit")
            if limit is None or limit.get("lower") is None or limit.get("upper") is None:
                raise ValueError(f"URDF limit missing for {urdf_name}")
            limits[side][logical_name] = (
                float(limit.get("lower")),
                float(limit.get("upper")),
            )
    return limits


def quat_normalize(q: Sequence[float]) -> np.ndarray:
    arr = np.asarray(q, dtype=np.float64)
    if arr.shape != (4,) or not np.all(np.isfinite(arr)):
        raise ValueError("quaternion must contain four finite numbers")
    norm = float(np.linalg.norm(arr))
    if norm < 1e-9:
        raise ValueError("zero-length quaternion")
    return arr / norm


def quat_inverse(q: np.ndarray) -> np.ndarray:
    x, y, z, w = quat_normalize(q)
    return np.array([-x, -y, -z, w], dtype=np.float64)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = quat_normalize(q1)
    x2, y2, z2, w2 = quat_normalize(q2)
    return quat_normalize((
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ))


def quat_relative(q_ref: np.ndarray, q_cur: np.ndarray) -> np.ndarray:
    return quat_multiply(quat_inverse(q_ref), q_cur)


def quat_to_euler_xyz(q: np.ndarray) -> Tuple[float, float, float]:
    x, y, z, w = quat_normalize(q)
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sin_pitch = clamp(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


def euler_component(q: np.ndarray, axis: str) -> float:
    roll, pitch, yaw = quat_to_euler_xyz(q)
    return {"roll": roll, "pitch": pitch, "yaw": yaw}[axis]


def shoulder_axis_components(upper_delta: np.ndarray) -> Dict[str, float]:
    """Twist-swing decomposition of the shoulder's upper_delta quaternion.

    Sequential Euler-angle extraction (quat_to_euler_xyz) shares cross
    terms between roll/pitch/yaw whenever more than one axis is active at
    once, which is normal for a 3-DOF ball joint. euler_probe logs
    confirmed this: a shoulder_yaw-only motion produced a strongly
    correlated, sustained (non-jitter) signal on the roll axis
    (corr up<->ur up to -0.82, ur reaching 0.56rad -- almost half of
    shoulder_pitch's whole +/-0.651rad range) and on the yaw axis
    (corr uy<->ur = 0.955 during the shoulder_roll test).

    This decomposition instead projects out the rotation about
    SHOULDER_TWIST_AXIS (Y) first -- that projection is exact for any
    angle, not a small-angle approximation -- and only the remaining
    "swing" (which by construction has zero component along Y) is used
    for the other two axes. This removes the shared-Y cross terms that
    were causing the coupling above, for any rotation magnitude.

    Returns a dict keyed the same as quat_to_euler_xyz's axis names
    ("roll", "pitch", "yaw") so RETARGET_CONFIG entries for upper_delta
    don't need to change: "pitch" is still shoulder_yaw (now the exact
    twist angle), "roll"/"yaw" are still shoulder_pitch/shoulder_roll
    (now swing components with the twist projected out).

    Re-measured after this fix (euler_probe_log_left_{1,3,5}.csv):
    the twist (shoulder_yaw / "pitch") is now cleanly isolated -- e.g.
    corr(sp, sr) = -0.04 during a pure shoulder_yaw motion. But the
    swing plane itself (shoulder_pitch "roll" vs shoulder_roll "yaw")
    still leaks in BOTH directions with DIFFERENT ratios (sy/sr=-0.187
    during a pure-pitch motion, sr/sy=+0.304 during a pure-roll motion).
    Since a single fixed misalignment angle would give the same ratio
    both ways, this isn't just an axis-calibration offset -- it's at
    least partly genuine shoulder anatomy (humeral rotation naturally
    accompanies elevation/abduction to different degrees). See
    suppress_minor_swing_axis() for the practical mitigation applied to
    this remaining pair.
    """
    x, y, z, w = quat_normalize(upper_delta)
    n = SHOULDER_TWIST_AXIS  # already unit length
    proj = float(np.dot((x, y, z), n))  # signed scalar along the twist axis
    twist = quat_normalize(np.array([proj * n[0], proj * n[1], proj * n[2], w]))
    swing = quat_multiply(np.array([x, y, z, w]), quat_inverse(twist))
    sx, sy, sz, sw = quat_normalize(swing)
    return {
        "pitch": 2.0 * math.atan2(proj, w),   # shoulder_yaw (twist, exact)
        "roll": 2.0 * math.atan2(sx, sw),     # shoulder_pitch (swing component)
        "yaw": 2.0 * math.atan2(sz, sw),      # shoulder_roll (swing component)
    }


# How aggressively suppress_minor_swing_axis() damps the smaller of the two
# swing-plane values. 1.0 = proportional damping, higher = more aggressive
# ("winner takes most"). Diagonal reaches (both axes genuinely intended)
# still move, just less than a single-axis motion of the same magnitude.
SWING_SUPPRESSION_EXPONENT = 1.5

# Suppression works well at moderate swing magnitudes but backfires near
# the end of the shoulder's real range of motion, where genuine anatomical
# coupling grows large enough that the "leak" axis can exceed the intended
# axis in raw magnitude (see suppress_minor_swing_axis() docstring).
# Measured clustering: moderate reaches/motions land around 86-93 degrees
# total (|roll|+|yaw|); deliberately extreme motions (e.g. hand-to-head,
# ~90 degrees of pure shoulder_pitch) land around 116-159 degrees total.
# Taper suppression down across that gap so moderate motion keeps the full
# base exponent (unchanged behavior) while extreme motion gets little to
# no suppression (so it isn't crushed the way the fixed exponent=1.5 case
# cut a 90-degree reach down to 25 degrees).
SWING_SUPPRESSION_TAPER_START_RAD = 1.75   # ~100 degrees total; below this, full base exponent
SWING_SUPPRESSION_TAPER_END_RAD = 2.60     # ~149 degrees total; at/above this, exponent -> 0


def effective_swing_suppression_exponent(
    total_magnitude: float,
    base_exponent: float,
    taper_start: float = SWING_SUPPRESSION_TAPER_START_RAD,
    taper_end: float = SWING_SUPPRESSION_TAPER_END_RAD,
) -> float:
    """Scale base_exponent down to 0 as total_magnitude (|roll|+|yaw|,
    BEFORE suppression) crosses from taper_start to taper_end.

    total_magnitude <= taper_start -> base_exponent unchanged
    total_magnitude >= taper_end   -> 0.0 (suppression fully disabled)
    in between                     -> linear interpolation
    """
    if taper_end <= taper_start or total_magnitude <= taper_start:
        return base_exponent
    if total_magnitude >= taper_end:
        return 0.0
    frac = (total_magnitude - taper_start) / (taper_end - taper_start)
    return base_exponent * (1.0 - frac)

# How much inertia the "which swing axis is intended" decision has.
# motor_log6.csv: a shoulder_pitch-only motion produced a raw yaw leak
# that briefly exceeded the raw roll signal itself (see
# suppress_minor_swing_axis() docstring). Comparing instantaneous
# magnitudes got fooled by that. Smoothing the magnitudes used for the
# comparison (NOT the values actually sent to the motor) with a slower
# alpha than DEFAULT_EMA_ALPHA adds hysteresis: the leak has to
# dominate for a sustained stretch, not just one noisy instant, before
# suppression treats it as the intended axis.
SWING_DOMINANCE_EMA_ALPHA = 0.08


def suppress_minor_swing_axis(
    roll_val: float,
    yaw_val: float,
    exponent: float = SWING_SUPPRESSION_EXPONENT,
    dominance_roll: Optional[float] = None,
    dominance_yaw: Optional[float] = None,
) -> Tuple[float, float]:
    """Damp whichever of shoulder_pitch("roll")/shoulder_roll("yaw") is
    smaller in magnitude, relative to the other, at this instant.

    euler_probe measurements show the swing plane leaks in both
    directions with DIFFERENT ratios depending on which axis is the
    intended one (see shoulder_axis_components() docstring) -- a fixed
    geometric correction can't remove this because it isn't a single
    consistent misalignment. This is a pragmatic runtime mitigation
    instead: assume whichever axis is moving the most right now is the
    one the operator intends, and shrink the other axis's contribution
    smoothly rather than zeroing it outright (so genuine diagonal
    motion -- both axes truly intended -- still comes through, just
    reduced versus a pure single-axis motion of the same size).

    exponent=0 disables suppression entirely (weight is always 1.0).

    dominance_roll/dominance_yaw let the caller decide "who's intended"
    using different (typically smoothed) magnitudes than the values
    actually being weighted -- see ArmRetargeter's swing_mag_ema.
    motor_log6.csv showed the raw yaw LEAK briefly exceeding the raw
    roll SIGNAL in magnitude (-1.88 vs -1.06rad) during one direction of
    a shoulder_pitch-only motion; comparing instantaneous |roll_val| vs
    |yaw_val| got fooled by that and suppressed the intended axis
    instead of the leak. Comparing EMA-smoothed magnitudes adds inertia
    so a momentary (or even sustained-but-secondary) crossover doesn't
    immediately flip which axis is treated as intended.
    """
    mag_roll = abs(roll_val) if dominance_roll is None else abs(dominance_roll)
    mag_yaw = abs(yaw_val) if dominance_yaw is None else abs(dominance_yaw)
    total = mag_roll + mag_yaw
    if total < 1e-6 or exponent <= 0.0:
        return roll_val, yaw_val
    weight_roll = (mag_roll / total) ** exponent
    weight_yaw = (mag_yaw / total) ** exponent
    return roll_val * weight_roll, yaw_val * weight_yaw


@dataclass(frozen=True)
class BodyTrackingFrame:
    active: bool
    joints: Dict[str, Dict[str, np.ndarray]]
    received_at: float
    source: Tuple[str, int]
    grip: Optional[dict] = None     # REPORT_51: 언리얼 "grip" 키 그대로 (없으면 None). 읽는 곳은 gripper_link.parse_grip


class RxLogger:
    """REPORT_52 (--log-rx): 받은 원시 패킷을 그대로 jsonl 로 (rx_probe 녹화와 같은 꼴 + t_mono·phase).
    기록만 한다 — 제어에 쓰는 값은 건드리지 않는다. 수신 스레드는 큐에 넣기만 하고, 파싱·쓰기는 이 스레드가 한다.
    줄: {"t_rx_ns": perf_counter_ns, "t_mono": time.monotonic (로그 CSV 의 t·frame.received_at 과 같은 시계),
         "t_wall", "size", "raw": 받은 JSON (못 읽으면 null + raw_text), "phase": 자동 세션 구간 (없으면 null)}"""

    def __init__(self, path: str):
        import queue
        self.path = path
        self.q: "queue.Queue" = queue.Queue()
        self.file = open(path, "w", encoding="utf-8", newline="\n")
        self.phase_fn = None          # main 이 자동 세션 구간 이름을 읽는 함수를 넣는다
        self.rows = 0
        self.dropped = 0
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def put(self, data: bytes, t_rx_ns: int, t_mono: float, t_wall: float) -> None:
        self.q.put((data, t_rx_ns, t_mono, t_wall))

    def _run(self) -> None:
        while True:
            item = self.q.get()
            if item is None:
                break
            data, t_rx_ns, t_mono, t_wall = item
            row = {"t_rx_ns": t_rx_ns, "t_mono": round(t_mono, 6), "t_wall": t_wall, "size": len(data)}
            try:
                msg = json.loads(data.decode("utf-8"))
                row["raw"] = msg if isinstance(msg, dict) else None
            except (UnicodeDecodeError, ValueError):
                row["raw"] = None
                row["raw_text"] = data.decode("utf-8", errors="replace")
            try:
                row["phase"] = self.phase_fn() if self.phase_fn is not None else None
            except Exception:
                row["phase"] = None
            try:
                self.file.write(json.dumps(row, ensure_ascii=False) + "\n")
                self.rows += 1
            except (OSError, ValueError):
                self.dropped += 1
        try:
            self.file.close()
        except OSError:
            pass

    def close(self) -> None:
        self.q.put(None)
        self.thread.join(timeout=2.0)


class BodyTrackingUdpReceiver:
    def __init__(self, listen_ip: str, listen_port: int, rx_log: Optional[RxLogger] = None):
        self.lock = threading.Lock()
        self.latest: Optional[BodyTrackingFrame] = None
        self.quit = False
        self.rx_log = rx_log          # REPORT_52: 있으면 받은 패킷을 그대로 기록 (기본 없음)
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
        self.thread.join(timeout=0.3)
        if self.rx_log is not None:
            self.rx_log.close()

    def get_latest(self) -> Optional[BodyTrackingFrame]:
        with self.lock:
            return self.latest

    def _run(self) -> None:
        while not self.quit:
            try:
                data, source = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if self.rx_log is not None:
                self.rx_log.put(data, time.perf_counter_ns(), time.monotonic(), time.time())

            try:
                message = json.loads(data.decode("utf-8"))
                frame = self._parse_message(message, source)
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
                print(f"\n[UDP] ignored invalid packet from {source}: {exc}")
                continue

            with self.lock:
                self.latest = frame

    @staticmethod
    def _parse_message(message: dict, source: Tuple[str, int]) -> BodyTrackingFrame:
        if not isinstance(message, dict):
            raise ValueError("JSON root must be an object")

        raw_joints = message.get("joints", [])
        if not isinstance(raw_joints, list):
            raise ValueError("'joints' must be a list")

        joints: Dict[str, Dict[str, np.ndarray]] = {}
        for item in raw_joints:
            if not isinstance(item, dict):
                continue
            name = item.get("name")
            pos = item.get("pos")
            rot = item.get("rot")
            if not isinstance(name, str) or pos is None or rot is None:
                continue
            position = np.asarray(pos, dtype=np.float64)
            if position.shape != (3,) or not np.all(np.isfinite(position)):
                continue
            try:
                rotation = quat_normalize(rot)
            except ValueError:
                continue
            joints[name] = {"pos": position, "rot": rotation}

        return BodyTrackingFrame(
            active=bool(message.get("active", False)),
            joints=joints,
            received_at=time.monotonic(),
            source=source,
            grip=message.get("grip") if isinstance(message.get("grip"), dict) else None,
        )


class ArmRetargeter:
    def __init__(
        self,
        side: str,
        ema_alpha: float = DEFAULT_EMA_ALPHA,
        swing_suppression_exponent: float = SWING_SUPPRESSION_EXPONENT,
        swing_taper_start: float = SWING_SUPPRESSION_TAPER_START_RAD,
        swing_taper_end: float = SWING_SUPPRESSION_TAPER_END_RAD,
        chest_relative: bool = True,
    ):
        self.side = side
        self.names = VR_JOINTS[side]
        self.neutral: Optional[Dict[str, np.ndarray]] = None
        self.ema_alpha = float(ema_alpha)
        # 가슴(몸통) 기준 상대화. 몸통을 돌리면 팔을 가만히 둬도 어깨 명령이 따라 움직이는
        # 문제를 없앤다. 실측(_teleop/REPORT_11): 팔을 몸에 고정하고 몸통만 48도 돌렸을 때
        # 헛명령이 53.6도 -> 19.2도 로 줄었고, shoulder_roll 의 몸통 상관이 0.591 -> 0.119.
        # 런타임에 'b' 키로 켜고 끌 수 있다 (같은 세션에서 켬/끔을 비교하려고).
        self.chest_relative = bool(chest_relative)
        self.chest_seen = False      # Chest 본을 한 번이라도 받았나 (구 빌드 판정)
        self.chest_valid = False     # 이번 프레임에 Chest 가 들어왔나 (상태 표시용)
        # 비교 로깅용: 같은 프레임을 두 기준으로 모두 계산해 둔다.
        # 명령에 쓰는 것은 chest_relative 가 정하고, 여기 둘 다 남는다.
        self.last_swing_world = {"roll": 0.0, "yaw": 0.0, "pitch": 0.0}
        self.last_swing_chest = {"roll": 0.0, "yaw": 0.0, "pitch": 0.0}
        self.swing_suppression_exponent = float(swing_suppression_exponent)
        self.swing_taper_start = float(swing_taper_start)
        self.swing_taper_end = float(swing_taper_end)
        # Debug snapshot of the shoulder swing plane, before vs after
        # suppress_minor_swing_axis(), for --log-csv to expose. Lets a
        # single test session separate "VR/EMA gave us this raw value"
        # from "suppression then shrank it to this".
        self.last_swing_raw = {"roll": 0.0, "yaw": 0.0}
        self.last_swing_suppressed = {"roll": 0.0, "yaw": 0.0}
        # Slow-moving magnitude estimate used ONLY to decide which swing
        # axis is "intended" (see suppress_minor_swing_axis()). Kept
        # separate from the actual roll/yaw values so the motor still
        # gets the real, unsmoothed signal -- only the suppression
        # weighting decision is stabilized.
        self.swing_mag_ema = {"roll": 0.0, "yaw": 0.0}
        self.filtered: Optional[Dict[str, np.ndarray]] = None
        self._last_frame_at: Optional[float] = None

    def has_required_joints(self, frame: Optional[BodyTrackingFrame]) -> bool:
        return bool(
            frame
            and frame.active
            and all(name in frame.joints for name in self.names.values())
        )

    def _smoothed_joints(self, frame: BodyTrackingFrame) -> Dict[str, np.ndarray]:
        """Return this arm's upper/lower/wrist quaternions, EMA-smoothed.

        Only blends in a new sample when frame.received_at changes, so
        the filter tracks actual new Unreal packets (~15Hz) instead of
        being re-fed the same stale frame on every 200Hz robot tick
        (which would make the filter converge to the raw value
        immediately and defeat the smoothing).
        """
        raw = {
            "upper": frame.joints[self.names["upper"]]["rot"],
            "lower": frame.joints[self.names["lower"]]["rot"],
            "wrist": frame.joints[self.names["wrist"]]["rot"],
        }
        # 가슴도 같은 경로로 스무딩한다. 🔴 순서가 중요하다: **먼저 필터, 그다음 상대화**.
        # 언리얼은 Chest 에 필터를 걸지 않는다(실측: 머리가 정지한 구간에서 가슴이 프레임당
        # 0.67도, 최대 15.9도 튄다 — REPORT_11 §4). 필터 없이 상대화하면 그 노이즈가
        # 그대로 어깨 명령이 된다 (Phase 3 에서 정지 구간 std 가 0.91 -> 8.62도 로 악화된 전례).
        if CHEST_JOINT_NAME in frame.joints:
            raw["chest"] = frame.joints[CHEST_JOINT_NAME]["rot"]
            self.chest_seen = True
            self.chest_valid = True
        else:
            # 키를 넣지 않으면 self.filtered["chest"] 가 그대로 유지된다(freeze) =
            # 마지막 유효 가슴으로 계속 상대화한다. 🔴 월드 기준으로 되돌아가지 않는다 —
            # 한 스트림에 두 좌표 규약이 섞이면 이 패치가 고치려는 문제가 되살아난다.
            self.chest_valid = False
        if self.filtered is None:
            self.filtered = {key: value.copy() for key, value in raw.items()}
            self._last_frame_at = frame.received_at
            return self.filtered

        if frame.received_at != self._last_frame_at:
            self._last_frame_at = frame.received_at
            for key, q_new in raw.items():
                q_prev = self.filtered[key]
                if float(np.dot(q_prev, q_new)) < 0.0:
                    q_new = -q_new  # same rotation, opposite quaternion sign; align before blending
                blended = self.ema_alpha * q_new + (1.0 - self.ema_alpha) * q_prev
                self.filtered[key] = quat_normalize(blended)
        return self.filtered

    def _upper_reference(self, joints: Dict[str, np.ndarray]) -> np.ndarray:
        """명령 계산에 쓸 위팔 회전. 가슴 기준이면 몸통 회전을 빼고 돌려준다.

        가슴이 아직 한 번도 안 왔으면(구 언리얼 빌드) 월드 기준 그대로 쓴다.
        오다가 끊긴 경우에는 filtered["chest"] 가 마지막 유효값으로 남아 있으므로
        여기서 자동으로 그 값이 쓰인다.
        """
        chest = joints.get("chest")
        if chest is None or not self.chest_relative:
            return joints["upper"]
        return quat_relative(chest, joints["upper"])

    def chest_status(self) -> str:
        if not self.chest_relative:
            return "OFF"
        if self.filtered is None:
            return "WAIT"        # 아직 프레임을 한 번도 처리하지 않았다 (c 를 누르기 전)
        if not self.chest_seen:
            return "NONE"
        return "OK" if self.chest_valid else "STALE"

    def calibrate(self, frame: BodyTrackingFrame) -> bool:
        if not self.has_required_joints(frame):
            return False
        joints = self._smoothed_joints(frame)
        upper = joints["upper"]
        lower = joints["lower"]
        wrist = joints["wrist"]
        # 가슴 기준과 월드 기준 중립을 **둘 다** 저장한다.
        # 그래야 'b' 키로 켬/끔을 전환할 때 재캘리브레이션이 필요 없고,
        # 같은 프레임을 두 기준으로 동시에 계산해 로그에 남길 수 있다.
        chest = joints.get("chest")
        self.neutral = {
            "upper_world": upper.copy(),
            "upper_chest": (quat_relative(chest, upper).copy() if chest is not None
                            else upper.copy()),
            "elbow_rel": quat_relative(upper, lower),
            "wrist_rel": quat_relative(lower, wrist),
        }
        return True

    def compute_deltas(
        self,
        frame: BodyTrackingFrame,
    ) -> Optional[Dict[str, float]]:
        if self.neutral is None or not self.has_required_joints(frame):
            return None

        joints = self._smoothed_joints(frame)
        upper = joints["upper"]
        lower = joints["lower"]
        wrist = joints["wrist"]

        # 🔴 가슴 기준 상대화는 shoulder_axis_components() **앞**에 들어간다.
        # twist-swing 분해는 입력이 이미 몸통 오염을 담고 있으면 그 오염을 세 축에 나눠 줄 뿐이다.
        chest = joints.get("chest")
        use_chest = bool(self.chest_relative and chest is not None)

        # 🔴 두 기준을 **매 프레임 모두** 계산한다. 명령에는 한쪽만 쓰고, 나머지는 로그로 남긴다.
        # 같은 프레임의 대조군이라 조건이 완전히 같고, 토글을 쓰지 않아도 효과를 잴 수 있다.
        delta_world = quat_relative(self.neutral["upper_world"], upper)
        delta_chest = (quat_relative(self.neutral["upper_chest"], quat_relative(chest, upper))
                       if chest is not None else delta_world)

        sources = {
            "upper_delta": delta_chest if use_chest else delta_world,
            # 팔꿈치/손목은 이미 부모 본 기준 상대값이라 몸통 회전이 수학적으로 상쇄된다.
            # 건드리지 않는다 (REPORT_05 §4.2.1).
            "elbow_delta": quat_relative(
                self.neutral["elbow_rel"],
                quat_relative(upper, lower),
            ),
            "wrist_delta": quat_relative(
                self.neutral["wrist_rel"],
                quat_relative(lower, wrist),
            ),
        }
        self.last_swing_world = shoulder_axis_components(delta_world)
        self.last_swing_chest = shoulder_axis_components(delta_chest)
        shoulder_components = shoulder_axis_components(sources["upper_delta"])
        self.last_swing_raw = {
            "roll": shoulder_components["roll"],
            "yaw": shoulder_components["yaw"],
        }
        self.swing_mag_ema["roll"] = (
            SWING_DOMINANCE_EMA_ALPHA * abs(shoulder_components["roll"])
            + (1.0 - SWING_DOMINANCE_EMA_ALPHA) * self.swing_mag_ema["roll"]
        )
        self.swing_mag_ema["yaw"] = (
            SWING_DOMINANCE_EMA_ALPHA * abs(shoulder_components["yaw"])
            + (1.0 - SWING_DOMINANCE_EMA_ALPHA) * self.swing_mag_ema["yaw"]
        )
        shoulder_components["roll"], shoulder_components["yaw"] = suppress_minor_swing_axis(
            shoulder_components["roll"], shoulder_components["yaw"],
            exponent=effective_swing_suppression_exponent(
                abs(self.last_swing_raw["roll"]) + abs(self.last_swing_raw["yaw"]),
                self.swing_suppression_exponent,
                self.swing_taper_start,
                self.swing_taper_end,
            ),
            dominance_roll=self.swing_mag_ema["roll"],
            dominance_yaw=self.swing_mag_ema["yaw"],
        )
        self.last_swing_suppressed = {
            "roll": shoulder_components["roll"],
            "yaw": shoulder_components["yaw"],
        }

        result: Dict[str, float] = {}
        for joint in JOINT_ORDER:
            source, axis, sign, scale, max_motion = RETARGET_CONFIG[self.side][joint]
            if source == "upper_delta":
                value = shoulder_components[axis] * sign * scale
            else:
                value = euler_component(sources[source], axis) * sign * scale
            result[joint] = clamp(value, -max_motion, max_motion)
        return result


class DualArmMotorController:
    """Controls both arms, each potentially on a different CAN bus.

    self.buses maps side -> recoil.Bus. If two sides happen to share the
    same channel string, they share the same Bus object (one SocketCAN
    interface isn't opened twice). self._unique_buses holds the deduped
    list used for shutdown.
    """

    def __init__(
        self,
        channels: Dict[str, str],
        sides: Sequence[str],
        motor_ids: Dict[str, Sequence[int]],
        urdf_limits: Dict[str, Dict[str, Tuple[float, float]]],
        kp: float,
        kd: float,
        torque_limit: float,
        max_speed: float,
        dry_run: bool,
    ):
        self.channels = dict(channels)
        self.sides = tuple(sides)
        self.motor_ids = {side: tuple(motor_ids[side]) for side in self.sides}
        self.urdf_limits = urdf_limits
        self.kp = kp
        self.kd = kd
        self.torque_limit = torque_limit
        self.max_speed = max_speed
        self.dry_run = dry_run
        self.position_mode = False
        self.buses: Dict[str, "recoil.Bus"] = {}
        self._unique_buses: List["recoil.Bus"] = []
        self.base_target = zero_targets(self.sides)
        self.command_target = zero_targets(self.sides)
        self.previous_target = zero_targets(self.sides)
        self.last_measured = zero_targets(self.sides)  # populated by send_targets(); NaN until real data arrives
        self.last_desired = zero_targets(self.sides)  # pre-rate-limit target, for measuring how much max_speed lags behind
        # 속도 피드포워드 (REPORT_28). PDO_2 의 두 번째 값 = 모터 목표 속도 (velocity_target).
        # 기본 끔 → 기존처럼 0 을 보낸다. 켜면 명령의 변화율(저역 통과, ±max_speed 로 제한)을 보낸다.
        self.vel_ff = False
        self.vel_ff_alpha = 0.5
        self.vel_target = zero_targets(self.sides)
        self._vel_last: Dict[Tuple[str, str], Tuple[float, float]] = {}
        self._vel_fresh = False
        # REPORT_36: 위팔 든 각에 따른 비틀기 안쪽 한계. side → (차렷 어깨 앞뒤 모터값, e90 비틀기 모터값). None 이면 끔
        self.yaw_dyn: Optional[Dict[str, Tuple[float, float]]] = None
        # REPORT_36: 양팔 충돌 방지. arm_gap_fn(command_target 같은 꼴) → 두 아래팔 겉면 거리(m). None 이면 끔
        self.arm_gap_fn = None
        self.arm_gap_min = 0.04
        self.last_arm_gap = float("nan")
        # 작업 지시 10 인계 예외 (--handover-exempt). handover_gap_fn 이 None 이거나 handover_ext 가 0 이면 예전 판정 그대로
        self.handover_gap_fn = None
        self.handover_ext = 0.0                 # 판정에서 뺄 손 끝 길이 (m)
        self.handover_finger_min = 0.01         # 집게끼리 거리 문턱 (m)
        self.last_arm_gap_full = float("nan")   # 예외 없는 거리 (로그·예외 유지 판단)
        self.last_arm_gap_finger = float("nan")
        # 작업 지시 11-b 가대 면 제한 (--table). table_fn(side, 명령) → 판까지 거리 dict. None 이면 끔
        self.table_fn = None
        self.table_blocks = 0                # 막은 횟수 (누적)
        self.table_blocks_by = {side: 0 for side in self.sides}
        # 작업 지시 11-b --start-pose table: 모터를 켠 뒤 start_ramp_s 초 동안 속도 제한을 start_ramp_lo 배 → 1 배로. 0 이면 끔
        self.start_ramp_s = 0.0
        self.start_ramp_lo = 0.2
        self.start_ramp_t = 0.0
        self.gap_blocks = 0                 # 막은 횟수 (로그용, 누적)
        self.yaw_holds = 0                  # 비틀기가 못 돌아와 어깨 앞뒤를 멈춘 횟수 (누적)

        all_ids = [dev for side in self.sides for dev in self.motor_ids[side]]
        if len(all_ids) != len(set(all_ids)):
            raise ValueError("left and right motor IDs must be unique")

        if not dry_run:
            if recoil is None:
                raise RuntimeError(
                    "recoil import failed; run on the robot or use --dry-run"
                )
            opened: Dict[str, "recoil.Bus"] = {}
            for side in self.sides:
                channel = self.channels[side]
                if channel not in opened:
                    opened[channel] = recoil.Bus(channel=channel, bitrate=1_000_000)
                self.buses[side] = opened[channel]
            self._unique_buses = list(opened.values())

    def _bus_for(self, side: str) -> "recoil.Bus":
        return self.buses[side]

    def _iter_motors(self):
        for side in self.sides:
            for index, joint in enumerate(JOINT_ORDER):
                yield side, joint, self.motor_ids[side][index]

    def initialize(self) -> None:
        if self.dry_run:
            print("[DRY-RUN] motor initialization skipped")
            return
        for side, joint, device_id in self._iter_motors():
            bus = self._bus_for(side)
            bus.set_mode(device_id, recoil.Mode.IDLE)
            time.sleep(0.002)

            # position_offset이 0이 아니면(다른 스크립트가 남긴 잔재) 정리한다.
            # PDO_2 getter/setter가 이 값을 반영하는데 enter_position_from_current()는
            # SDO로 raw 값을 읽으므로, offset이 남아있으면 그 크기만큼 발산이 발생한다.
            offset = bus._read_parameter_f32(
                device_id,
                recoil.Parameter.POSITION_CONTROLLER_POSITION_OFFSET,
                timeout=0.5,
            )
            if offset is None:
                print(f"  [{side[0].upper()} {joint:14s}] motor {device_id:2d} offset read failed (timeout), 건너뜀")
            elif abs(offset) > 1e-3:
                print(
                    f"  [{side[0].upper()} {joint:14s}] motor {device_id:2d} "
                    f"position_offset={offset:.4f} -> 0 (flash 저장)"
                )
                bus.write_position_offset(device_id, 0.0)
                bus.store_settings_to_flash(device_id)
                time.sleep(0.01)

            bus.write_position_kp(device_id, self.kp)
            time.sleep(0.002)
            bus.write_position_kd(device_id, self.kd)
            time.sleep(0.002)
            bus.write_torque_limit(device_id, self.torque_limit)
            time.sleep(0.002)
            bus.feed(device_id)
            bus.set_mode(device_id, recoil.Mode.DAMPING)
            print(f"  [{side[0].upper()} {joint:14s}] motor {device_id:2d} -> DAMPING")

    def enter_position_from_current(self) -> bool:
        measured = zero_targets(self.sides)
        if self.dry_run:
            base = getattr(self, "dry_run_base", None)
            if base is not None:
                for side in self.sides:
                    for joint in JOINT_ORDER:
                        measured[side][joint] = float(base[side][joint])
                print("[DRY-RUN] 주차 자세(모델)를 모터 기준 자세로 쓴다" if self.start_ramp_s > 0.0 else "[DRY-RUN] 로봇 기준값(차렷)을 모터 기준 자세로 쓴다")
            else:
                print("[DRY-RUN] using URDF zero pose as the motor base")
        else:
            for side, joint, device_id in self._iter_motors():
                bus = self._bus_for(side)
                value = bus._read_parameter_f32(
                    device_id,
                    recoil.Parameter.POSITION_CONTROLLER_POSITION_MEASURED,
                    timeout=0.5,
                )
                if value is None or not math.isfinite(float(value)):
                    print(f"\n[MOTOR] position read failed for motor {device_id}")
                    self.enter_damping()
                    return False
                measured[side][joint] = float(value)

        invalid = []
        for side in self.sides:
            for joint in JOINT_ORDER:
                value = measured[side][joint]
                low, high = self.urdf_limits[side][joint]
                if not (low - START_LIMIT_TOLERANCE_RAD <= value <= high + START_LIMIT_TOLERANCE_RAD):
                    invalid.append(
                        f"{side}.{joint}={value:+.3f} outside [{low:+.3f},{high:+.3f}]"
                    )
        if invalid:
            print("\n[SAFETY] POSITION blocked; calibrated pose disagrees with URDF:")
            for message in invalid:
                print(f"  {message}")
            self.enter_damping()
            return False

        self.base_target = {
            side: dict(measured[side])
            for side in self.sides
        }
        self.command_target = {
            side: dict(measured[side])
            for side in self.sides
        }
        self.previous_target = {
            side: dict(measured[side])
            for side in self.sides
        }
        self.reset_velocity()
        self.start_ramp_t = 0.0

        if not self.dry_run:
            for _side, _joint, device_id in self._iter_motors():
                bus = self._bus_for(_side)
                bus.feed(device_id)
                bus.set_mode(device_id, recoil.Mode.POSITION)
                time.sleep(0.002)
        self.position_mode = True
        print("[MOTOR] POSITION mode; current calibrated pose is the neutral base")
        return True

    def enter_damping(self) -> None:
        was_position = self.position_mode
        self.position_mode = False
        self.reset_velocity()
        if not self.dry_run:
            for _side, _joint, device_id in self._iter_motors():
                self._bus_for(_side).set_mode(device_id, recoil.Mode.DAMPING)
        if was_position or self.dry_run:
            print("[MOTOR] DAMPING mode")

    def enter_idle(self) -> None:
        self.position_mode = False
        if not self.dry_run:
            for _side, _joint, device_id in self._iter_motors():
                self._bus_for(_side).set_mode(device_id, recoil.Mode.IDLE)
        print("[MOTOR] IDLE mode")

    def stop(self) -> None:
        if self.dry_run:
            return
        self.enter_idle()
        for bus in self._unique_buses:
            bus.stop()

    def update_targets(
        self,
        deltas: Dict[str, Dict[str, float]],
        dt: float,
    ) -> Dict[str, Dict[str, float]]:
        max_step = self.max_speed * clamp(dt, 0.0, 0.05)
        if self.start_ramp_s > 0.0 and self.start_ramp_t < self.start_ramp_s:
            self.start_ramp_t += clamp(dt, 0.0, 0.05)
            max_step *= self.start_ramp_lo + (1.0 - self.start_ramp_lo) * min(self.start_ramp_t / self.start_ramp_s, 1.0)
        prev_all = {side: dict(self.previous_target[side]) for side in self.sides}
        for side in self.sides:
            desired_all = {}
            for joint in JOINT_ORDER:
                low, high = self.urdf_limits[side][joint]
                desired = clamp(
                    self.base_target[side][joint] + deltas[side][joint],
                    low,
                    high,
                )
                previous = self.previous_target[side][joint]
                target = previous + clamp(desired - previous, -max_step, max_step)
                self.command_target[side][joint] = target
                desired_all[joint] = desired
            if self.yaw_dyn is not None and side in self.yaw_dyn:
                desired_all["shoulder_yaw"] = self._apply_yaw_window(side, desired_all["shoulder_yaw"], max_step)
            for joint in JOINT_ORDER:
                self.previous_target[side][joint] = self.command_target[side][joint]
                self.last_desired[side][joint] = desired_all[joint]
                self._update_velocity(side, joint, desired_all[joint])
        if self.arm_gap_fn is not None and len(self.sides) == 2:
            self._block_arm_collision(prev_all)
        if self.table_fn is not None and self._block_table(prev_all):
            if self.arm_gap_fn is not None and len(self.sides) == 2:     # 되돌린 팔이 두 팔 거리를 줄였으면 충돌 방지가 다시 본다
                self._block_arm_collision(prev_all)
                self._block_table(prev_all)
        self._vel_fresh = True       # 이번 루프에 새 목표가 나왔다 (HOLD 중에는 안 나와서 속도 0 을 보낸다)
        return self.command_target

    def _yaw_window(self, side: str, sp_motor: float) -> Tuple[float, float]:
        sp_rest, sy0 = self.yaw_dyn[side]
        fwd = PR.upper_arm_forward_deg(side, math.degrees(sp_motor - sp_rest))
        lo, hi = PR.yaw_window_deg(side, fwd)
        return sy0 + math.radians(lo), sy0 + math.radians(hi)

    def _apply_yaw_window(self, side: str, desired: float, max_step: float) -> float:
        """REPORT_36: 어깨 앞뒤 명령(이번 루프, 속도 제한 뒤)에서의 비틀기 허용 범위로 목표를 자르고 속도 제한 안에서 따라간다.
        위팔을 내려 범위가 줄었는데 비틀기가 아직 못 돌아왔으면, 어깨 앞뒤를 내리지 않고 그 자리에 둔다 (전면 하드웨어 보호)."""
        cmd, prev = self.command_target[side], self.previous_target[side]
        lo, hi = self._yaw_window(side, cmd["shoulder_pitch"])
        desired = clamp(desired, lo, hi)
        target = prev["shoulder_yaw"] + clamp(desired - prev["shoulder_yaw"], -max_step, max_step)
        if not (lo - 1e-6 <= target <= hi + 1e-6):
            plo, phi = self._yaw_window(side, prev["shoulder_pitch"])
            if plo - 1e-6 <= target <= phi + 1e-6:
                cmd["shoulder_pitch"] = prev["shoulder_pitch"]      # 비틀기가 돌아올 때까지 위팔을 안 내린다
                self.yaw_holds += 1
        cmd["shoulder_yaw"] = target
        return desired

    COLLISION_STAGES = (("shoulder_yaw", "shoulder_roll"),                                   # REPORT_36
                        ("shoulder_yaw", "shoulder_roll", "shoulder_pitch", "elbow_pitch"))   # REPORT_54 P1

    def _block_arm_collision(self, prev_all) -> None:
        """REPORT_36: 두 아래팔(+모델 손) 겉면 거리가 arm_gap_min 보다 가깝고 더 가까워지면, 가까워지게 만든 팔의 비틀기·어깨 옆
        변화는 받지 않는다 (이전 값 유지). 멀어지는 쪽 변화·다른 관절은 그대로.
        REPORT_54 P1: 그래도 문턱 밑에서 더 가까워지면 (어깨 앞뒤·팔꿈치가 붙인 경우) 그 팔의 어깨 앞뒤·팔꿈치 변화까지 받지 않는다."""
        gap_fn = self.arm_gap_fn
        if self.handover_gap_fn is not None and self.handover_ext > 0.0:
            gap_fn = self._handover_gap         # 작업 지시 10: 인계 중에는 손 끝을 뺀 거리 + 집게끼리 문턱
        try:
            g_prev = gap_fn(prev_all)
            g_new = gap_fn(self.command_target)
        except Exception:
            return
        for joints in self.COLLISION_STAGES:
            if not (g_new < self.arm_gap_min and g_new < g_prev):
                break
            for side in self.sides:
                test = {s: dict(self.command_target[s]) for s in self.sides}
                for j in joints:
                    test[side][j] = prev_all[side][j]
                if gap_fn(test) > g_new + 1e-6:                   # 이 팔의 이 관절들 변화가 가깝게 만들었다
                    for j in joints:
                        self.command_target[side][j] = prev_all[side][j]
                        self.previous_target[side][j] = prev_all[side][j]
                    self.gap_blocks += 1
            g_new = gap_fn(self.command_target)
        self.last_arm_gap = g_new
        if self.handover_gap_fn is not None:     # 로그·예외 유지 판단용 (예외 없는 거리, 집게끼리 거리)
            try:
                full, _, finger = self.handover_gap_fn(self.command_target, 0.0)
                self.last_arm_gap_full, self.last_arm_gap_finger = full, finger
            except Exception:
                pass

    # 작업 지시 11-b ②: 판 쪽으로 더 들어가게 한 관절 변화를 받지 않는다. 팔꿈치 → 어깨 앞뒤 → 둘 다 → 손목 빼고 전부 순서로, 되돌려서 나빠지지 않는 가장 적은 관절만.
    TABLE_STAGES = (("elbow_pitch",), ("shoulder_pitch",), ("elbow_pitch", "shoulder_pitch"),
                    ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch"))
    TABLE_JOINTS = TABLE_STAGES[-1]
    # 2227 재생: 목적지가 판 위인데 가는 길 첫걸음이 살짝 내려간다고 막으면 팔이 g = 0 에 묶여 사람을 다시 못 따라갔다.
    # 그래서 목적지(desired)가 여유를 지키면 여유 안쪽으로 이만큼(m)까지는 지나간다. 다만 판까지 실제 거리는 TABLE_TRANSIT_MIN 을 남긴다
    # (집게는 여유가 0 이라 g 만 보면 판을 뚫는다 — 몸체 m2 재생에서 잡았다)
    TABLE_TRANSIT_SLACK = 0.01
    TABLE_TRANSIT_MIN = 0.003

    def _block_table(self, prev_all) -> bool:
        """판까지 값 g (여유 뺌) 가 0 아래이고 이번 변화로 더 작아지면 그 변화를 되돌린다. 이미 걸려 있어도 (주차 자세) 멀어지는 쪽은 받는다. 되돌렸으면 True."""
        blocked = False
        for side in self.sides:
            cmd, prev = self.command_target[side], prev_all[side]
            if all(cmd[j] == prev[j] for j in self.TABLE_JOINTS):
                continue
            try:
                c_new = self.table_fn(side, cmd)
                g_new = c_new["g"]
                if g_new >= 0.0:
                    continue
                g_prev = self.table_fn(side, prev)["g"]
                if g_new >= g_prev:
                    continue
                if (g_new >= -self.TABLE_TRANSIT_SLACK and c_new["min"] >= self.TABLE_TRANSIT_MIN
                        and self.table_fn(side, self.last_desired[side])["g"] >= 0.0):
                    continue
                for joints in self.TABLE_STAGES:
                    test = dict(cmd)
                    for j in joints:
                        test[j] = prev[j]
                    if joints is self.TABLE_JOINTS or self.table_fn(side, test)["g"] >= min(g_prev, 0.0):
                        for j in joints:
                            cmd[j] = prev[j]
                            self.previous_target[side][j] = prev[j]
                        self.table_blocks += 1
                        self.table_blocks_by[side] += 1
                        blocked = True
                        break
            except Exception:
                continue
        return blocked

    def _handover_gap(self, cmd) -> float:
        """인계 예외 중 판정 거리: min(손 끝 handover_ext 를 뺀 겉면 거리, arm_gap_min + 집게끼리 거리 − 문턱)."""
        _, body, finger = self.handover_gap_fn(cmd, self.handover_ext)
        return min(body, self.arm_gap_min + finger - self.handover_finger_min)

    def _update_velocity(self, side: str, joint: str, desired: float) -> None:
        """목표 속도 추정. 🔴 루프(200 Hz)마다가 아니라 **목표가 바뀔 때마다**(VR 프레임, ≈14.5 Hz) 잰다.
        루프마다 재면 속도 제한 크기의 펄스가 된다. 프레임 사이에는 값을 유지하고, 0.2 초 넘게 안 바뀌면 0 으로."""
        now = time.monotonic()
        key = (side, joint)
        last = self._vel_last.get(key)
        if last is None:
            self._vel_last[key] = (desired, now)
            return
        last_value, last_t = last
        if abs(desired - last_value) > 1e-9 and now - last_t > 1e-3:
            raw = (desired - last_value) / (now - last_t)
            a = self.vel_ff_alpha
            v = a * raw + (1.0 - a) * self.vel_target[side][joint]
            self.vel_target[side][joint] = clamp(v, -self.max_speed, self.max_speed)
            self._vel_last[key] = (desired, now)
        elif now - last_t > 0.2:
            self.vel_target[side][joint] = 0.0

    def reset_velocity(self) -> None:
        self.vel_target = zero_targets(self.sides)
        self._vel_last = {}
        self._vel_fresh = False

    def send_targets(self) -> Optional[Tuple[str, str, int, float, float]]:
        """Send position commands and watch for divergence.

        Every loop, this compares the measured position (already returned
        by write_read_pdo_2(), previously discarded) against the command
        just sent. If any joint's |measured - command| exceeds
        DEFAULT_DIVERGENCE_THRESHOLD_RAD, all motors are switched to
        DAMPING immediately and the offending joint is returned so the
        caller can log it.

        Returns:
            (side, joint, device_id, command, measured) for the first
            joint that diverged, or None if every joint tracked normally
            (or we are not in position mode / dry-run).
        """
        if not self.position_mode:
            if not self.dry_run:
                for _side, _joint, device_id in self._iter_motors():
                    self._bus_for(_side).feed(device_id)
            return None

        if self.dry_run:
            return None

        use_ff = self.vel_ff and self._vel_fresh
        self._vel_fresh = False
        for side, joint, device_id in self._iter_motors():
            bus = self._bus_for(side)
            command = float(self.command_target[side][joint])
            velocity = float(self.vel_target[side][joint]) if use_ff else 0.0
            pos, _vel = bus.write_read_pdo_2(device_id, command, velocity)
            if pos is not None:
                self.last_measured[side][joint] = float(pos)

            if pos is not None and abs(pos - command) > DEFAULT_DIVERGENCE_THRESHOLD_RAD:
                self.enter_damping()
                return (side, joint, device_id, command, pos)

        return None


# --------------------------------------------------------------------------- 자동 측정 세션 (2026-09-24)
# 사람이 키를 안 누르고 VR 안에서 동작만 하면 되게, 워치독이 진행을 직접 잡는다.
# 소리는 **서버 PC** 의 cue_player.py 가 낸다 (VR 헤드셋으로 들린다) — 여기서는 UDP 신호만 보낸다.

AUTO_PHASES = [                       # (구간 이름, 화면 문구, 길이초, 소리)
    ("L_shoulder_pitch", "왼팔 — 팔꿈치 편 채로 앞뒤로만", 10.0, "go"),
    ("rest", "팔을 완전히 내리고 멈춤", 4.0, "stop"),
    ("L_shoulder_roll", "왼팔 — 팔꿈치 편 채로 옆으로 벌렸다 모으기", 10.0, "go"),
    ("rest", "팔을 완전히 내리고 멈춤", 4.0, "stop"),
    ("L_shoulder_yaw", "왼팔 — 팔 내리고 상완만 비틀기", 10.0, "go"),
    ("rest", "팔을 완전히 내리고 멈춤", 4.0, "stop"),
    ("switch_arm", "이제 오른팔. 왼팔은 내리고 가만히", 10.0, "stop"),
    ("R_shoulder_pitch", "오른팔 — 팔꿈치 편 채로 앞뒤로만", 10.0, "go"),
    ("rest", "팔을 완전히 내리고 멈춤", 4.0, "stop"),
    ("R_shoulder_roll", "오른팔 — 팔꿈치 편 채로 옆으로 벌렸다 모으기", 10.0, "go"),
    ("rest", "팔을 완전히 내리고 멈춤", 4.0, "stop"),
    ("R_shoulder_yaw", "오른팔 — 팔 내리고 상완만 비틀기", 10.0, "go"),
    ("rest", "팔을 완전히 내리고 멈춤", 4.0, "stop"),
    ("torso_turn", "몸통만 돌리기 — 팔은 몸에 붙이고 팔꿈치 90°", 15.0, "go"),
    ("rest", "정면으로 돌아와 정지", 4.0, "stop"),
    ("torso_turn_gaze", "몸통만 돌리기 — 시선은 정면 한 점 고정", 15.0, "go"),
    ("rest", "팔을 내리고 정면으로", 4.0, "stop"),
]
AUTO_TRACKING_GRACE_SEC = 5.0         # 트래킹이 이만큼 끊기면 중단
AUTO_RETURN_TOL_RAD = 0.02            # 시작 자세 복귀 완료 판정
AUTO_RETURN_TIMEOUT_SEC = 15.0


class CueSound:
    """서버 PC 의 cue_player.py 로 소리 신호만 보낸다 (UDP, 실패해도 진행을 막지 않는다)."""

    def __init__(self, target: Optional[str], verbose: bool = False, voice: bool = True):
        self.addr = None
        self.sock = None
        self.verbose = verbose
        self.voice = voice          # 음성 안내 (cue_player 가 "say:문구" 를 말한다, 2026-10-02)
        self.said: List[Tuple[float, str]] = []   # (monotonic, 문구) — dry-run 확인용
        self.sent = 0
        self.error = None
        if not target:
            return
        host, _, port = target.partition(":")
        self.addr = (host, int(port or 5021))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, sound: str, text: str = "", phase: str = "", left: Optional[float] = None) -> None:
        if self.sock is None:
            return
        msg = {"sound": sound, "text": text, "phase": phase}
        if left is not None:
            msg["left"] = left
        try:
            payload = json.dumps(msg, ensure_ascii=False).encode("utf-8")
            self.sock.sendto(payload, self.addr)
            self.sent += 1
            if self.verbose:
                print(f"[소리→] {self.addr[0]}:{self.addr[1]} {sound} ({len(payload)}바이트, 누적 {self.sent})")
        except OSError as exc:
            if self.error != str(exc):
                self.error = str(exc)
                print(f"\n🔴 [소리] 신호를 못 보낸다: {exc}  (IP·포트·방화벽 확인)")

    def say(self, text: str, phase: str = "") -> None:
        """음성 안내. 삑 소리와 별개 — 구간 시각에는 영향 없음."""
        if not self.voice or not text:
            return
        self.said.append((time.monotonic(), text))
        self.send("say:" + text, text=text, phase=phase)


# 음성 문구 (2026-10-02). cue_player.VOICE_PHRASES 와 같은 문구여야 미리 만든 wav 로 바로 나온다.
ARM_KO = {"L": "왼팔", "R": "오른팔"}
VOICE_START = {"lead": "차렷", "calib": "차렷", "wrist": "양팔 팔꿈치 구십도, 엄지 위로",
               "back": "차렷. 곧 모터가 켜집니다"}
VOICE_REST = "차렷"
VOICE_LOWER = "내리세요"
VOICE_SWITCH_ARM = "오른팔로 바꿉니다"
VOICE_NEXT_LAP = "차렷. 다음 바퀴로 바꿉니다"
VOICE_DONE = "끝났습니다. 힘이 빠집니다"
VOICE_ABORT = "중단합니다. 힘이 빠집니다"
# REPORT_45 §4 시작 확인 (2026-10-04 승인): 모터를 켜기 전이라 "힘이 빠집니다" 대신 따로 둔다.
# 서버 PC cue_player 가 이 문구를 아직 모르면(말 길이 표에 없으면) 말하지 않고 화면 + 경보음만.
VOICE_START_SWAP = "컨트롤러를 좌우 바꿔 쥐었습니다. 시작하지 않습니다"
VOICE_START_POSE = "차렷 자세가 맞지 않습니다. 시작하지 않습니다"
VOICE_TABLE_HIGH = "손을 수평보다 조금 높게"     # 작업 지시 11-b --start-pose table (서버 PC 가 모르면 "팔꿈치 구십도" 문구로 대신, SERVER_PC.md §6)
VOICE_SEP = "\n"             # 한 예고 안에서 차례로 따로 말할 문구 사이 (REPORT_49: "왼손" → "컵을 똑바로")
VOICE_PART_GAP = 0.3         # 그 문구들 사이 쉼 (초)
VOICE_PREVIEW_DELAY = 0.8      # 쉬는 구간 시작 "차렷" 뒤 이만큼 있다가 다음 동작 예고 (겹치면 앞 문구가 버려진다)
_VOICE_ARM_TEXT = {            # 구간 이름 뒷부분 → 말 (앞에 "왼팔. " / "오른팔. ")
    "still": "차렷으로 가만히", "wave": "옆으로 벌리고 앞뒤로 흔들기", "wrist": "팔꿈치 구십도, 손목 돌리기",
    "bend": "팔꿈치 굽혔다 펴기", "side_up": "옆으로 만세", "front_up": "앞으로 만세",            # check
    "shoulder_pitch": "편 채로 앞뒤로", "shoulder_roll": "옆으로 벌렸다 모으기",                   # live
    "shoulder_yaw": "팔 내리고 비틀기",
    "swing_fast": "앞뒤로 빠르게",                                                                # speed
    "in_rest": "팔꿈치 구십도, 손을 배 쪽으로 천천히", "in_raised": "위팔을 조금 들고, 손을 가운데로 천천히",  # yaw
    "lower": "그대로 위팔만 천천히 내리기", "out": "팔꿈치 구십도, 손을 바깥으로 천천히",
}
_VOICE_OTHER = {"torso_turn": "몸통만 돌리기", "torso_turn_gaze": "몸통 돌리기. 시선은 정면",
                "C_warmup": "양팔. 팔꿈치 굽혔다 펴기", "D_learn": "양팔. 팔꿈치 굽혔다 펴기",
                "elbow_fast": "양팔. 팔꿈치 빠르게 굽혔다 펴기",
                "both_in": "양팔. 위팔을 조금 들고, 양손을 가운데로 천천히"}
if WSESS is not None:                       # REPORT_49 손목 회전 측정
    _VOICE_OTHER.update(WSESS.voice_map())


# 말 길이 (2026-10-02): "말 → 삑 → 측정" 규칙. 서버 PC cue_player 가 처음 띄울 때 실제 wav 길이를
# tools/voice_durations.json 에 남긴다 (사용자가 커밋해 올림). 없으면 글자 수로 넉넉히 어림한다.
VOICE_MARGIN = 0.5            # 말이 끝날 시간 = 말 길이 + 이것
VOICE_GAP_BEFORE_MOVE = 1.5   # 동작 구간 삑은 말이 끝난(여유 포함) 뒤 최소 이만큼 더 있다가
_VOICE_DUR: Dict[str, float] = {}
_VOICE_DUR_SRC = "어림 (글자 수)"


def load_voice_durations(path: Optional[Path] = None) -> str:
    global _VOICE_DUR, _VOICE_DUR_SRC
    path = path or POS_TOOLS / "voice_durations.json"
    try:
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        _VOICE_DUR = {k: float(v) for k, v in d.get("durations", d).items() if isinstance(v, (int, float))}
        _VOICE_DUR_SRC = f"{Path(path).name} ({d.get('voice', '?')}, {len(_VOICE_DUR)}개)"
    except Exception:
        _VOICE_DUR, _VOICE_DUR_SRC = {}, "어림 (글자 수 — voice_durations.json 없음)"
    return _VOICE_DUR_SRC


def voice_known(text: str) -> bool:
    """서버 PC cue_player 가 이 문구의 wav 를 가지고 있나 (말 길이 표 voice_durations.json 에 있으면 그렇다)."""
    return text in _VOICE_DUR


def voice_seconds(text: str) -> float:
    """문구 말 길이(초). 표에 없으면 넉넉한 어림: 0.3 + 글자당 0.2 + 쉼표·마침표당 0.25."""
    if VOICE_SEP in text:
        parts = [t for t in text.split(VOICE_SEP) if t]
        return sum(voice_seconds(t) for t in parts) + VOICE_PART_GAP * (len(parts) - 1)
    if text in _VOICE_DUR:
        return _VOICE_DUR[text]
    letters = sum(1 for ch in text if ch.isalnum())
    pauses = sum(1 for ch in text if ch in ".,")
    return 0.3 + 0.2 * letters + 0.25 * pauses


def voice_for(phase: str) -> str:
    """구간 이름 → 예고 문구. 모르면 빈 문자열 (말 안 함)."""
    if phase in _VOICE_OTHER:
        return _VOICE_OTHER[phase]
    for tag in ("C", "D"):                                       # twist: C_twist_L 등
        for A in ("L", "R"):
            if phase == f"{tag}_twist_{A}":
                return f"{ARM_KO[A]}. 편 채로 크게 돌리기"
    if phase[:2] in ("L_", "R_") and phase[2:] in _VOICE_ARM_TEXT:
        return f"{ARM_KO[phase[0]]}. {_VOICE_ARM_TEXT[phase[2:]]}"
    return ""


def voice_phrases() -> set:
    """워치독이 보낼 수 있는 모든 고정 문구 (cue_player.VOICE_PHRASES 와 대조용)."""
    out = set(VOICE_START.values()) | {VOICE_REST, VOICE_LOWER, VOICE_SWITCH_ARM, VOICE_NEXT_LAP, VOICE_DONE,
                                       VOICE_ABORT, "소리 시험입니다", VOICE_START_SWAP, VOICE_START_POSE, VOICE_TABLE_HIGH}
    out |= {"그대로 멈추기"}
    names = [p[0] for p in AUTO_PHASES + SPEED_PHASES + twist_phases("C") + twist_phases("D") + CHECK_PHASES
             + YAW_PHASES]
    names += [p[0] for p in WRIST_PHASES]
    out |= {part for n in names if voice_for(n) for part in voice_for(n).split(VOICE_SEP)}
    if GSESS is not None:                    # REPORT_50: 워치독은 말하지 않지만 cue_player 문구 목록과 맞춰 둔다
        out |= set(GSESS.phrases())
    if USESS is not None:                    # REPORT_52: 워치독은 말하지 않지만 cue_player 문구 목록과 맞춰 둔다
        out |= set(USESS.phrases())
    if BSESS is not None:                    # REPORT_54 P2: 같은 이유
        out |= set(BSESS.phrases())
    return out


# twist 프로토콜 (2026-09-28): 팔을 편 채 어깨를 돌릴 때 C(유지) 와 D(뼈 긴축 회전) 비교 (REPORT_27)
TWIST_ARM_L = "왼팔을 쭉 편 채 어깨부터 팔 전체를 크게 돌리기 — 손바닥이 앞 ↔ 뒤 (오른팔은 차렷)"
TWIST_ARM_R = "오른팔을 쭉 편 채 어깨부터 팔 전체를 크게 돌리기 — 손바닥이 앞 ↔ 뒤 (왼팔은 차렷)"
TWIST_BEND = "양팔 — 팔꿈치를 90° 로 굽혔다 펴기를 반복"


def twist_phases(tag: str) -> list:
    first = ("C_warmup", TWIST_BEND) if tag == "C" else ("D_learn", TWIST_BEND + " (D 가 보정값을 배우는 구간)")
    return [
        (first[0], first[1], 15.0, "learn"),
        ("rest", "차렷으로 멈춤", 4.0, "stop"),
        (f"{tag}_twist_L", TWIST_ARM_L, 12.0, "twist"),
        ("rest", "차렷으로 멈춤", 4.0, "stop"),
        (f"{tag}_twist_R", TWIST_ARM_R, 12.0, "twist"),
        ("rest", "차렷으로 멈춤", 4.0, "stop"),
    ]


# speed 프로토콜 (2026-09-28): 같은 빠른 동작을 속도 피드포워드 끔 → 켬 으로 한 바퀴씩 (REPORT_28)
SPEED_PHASES = [
    ("L_swing_fast", "왼팔을 쭉 편 채 앞뒤로 빠르게 흔들기 (오른팔은 차렷)", 10.0, "go"),
    ("rest", "차렷으로 멈춤", 4.0, "stop"),
    ("R_swing_fast", "오른팔을 쭉 편 채 앞뒤로 빠르게 흔들기 (왼팔은 차렷)", 10.0, "go"),
    ("rest", "차렷으로 멈춤", 4.0, "stop"),
    ("elbow_fast", "양팔 — 팔꿈치를 빠르게 굽혔다 펴기 반복", 10.0, "learn"),
    ("rest", "차렷으로 멈춤", 4.0, "stop"),
]

# check 프로토콜 (2026-10-02): 거르기(REPORT_33)·만세 확인. 팔마다 한 바퀴, 구간 사이 차렷 4초 (SESSION_20261002_check_auto)
# 구간 = (이름, 화면 문구, 초, 시작 소리, [추가 소리]). 추가 소리 = [(구간 시작부터 초, 소리, 화면 문구)].
# 소리는 cue_player 가 비동기로 내서 바로 겹쳐 보내면 앞 소리가 끊긴다 → 추가 소리는 시간을 띄운다.
CHECK_REST = ("rest", "차렷 — 가만히", 4.0, "stop")


def check_phases(side: str) -> list:
    A = "L" if side == "left" else "R"
    me, other = ("왼팔", "오른팔") if side == "left" else ("오른팔", "왼팔")
    lower = [(6.0, "tick", "▼ 이제 천천히 내리기", "내리세요"), (6.3, "tick", "")]   # 12초 구간의 절반: 짧게 두 번 + 음성
    return [
        (f"{A}_still", f"차렷으로 가만히 (양팔)", 5.0, "stop"),
        CHECK_REST,
        (f"{A}_wave", f"{me} — 옆으로 어깨 높이, 팔꿈치 편 채 앞뒤로 천천히 흔들기 ({other} 차렷)", 10.0, "go"),
        CHECK_REST,
        (f"{A}_wrist", f"{me} — 팔꿈치 90°: 엄지 위 → 손등 위 → 엄지 위 → 손바닥 위 ({other} 차렷)", 10.0, "twist"),
        CHECK_REST,
        (f"{A}_bend", f"{me} — 팔꿈치 끝까지 굽혔다 펴기 반복 ({other} 차렷)", 8.0, "learn"),
        CHECK_REST,
        (f"{A}_side_up", f"{me} — 옆으로 머리 위까지 천천히 올려 멈춤 → (짧은 소리 두 번) 내리기", 12.0, "switch",
         lower),
        CHECK_REST,
        (f"{A}_front_up", f"{me} — 앞으로 머리 위까지 천천히 올려 멈춤 → (짧은 소리 두 번) 내리기", 12.0, "switch",
         [(0.8, "switch", "")] + lower),
        CHECK_REST,
    ]


# yaw 프로토콜 (2026-10-03): 어깨 비틀기 한계·충돌 방지 확인 (REPORT_36 §5 실기동 절차를 자동 세션으로)
# 구간 중간 "그대로 멈추기"(짧은 소리 + 음성). 안쪽(위팔 듦) → 내리기 는 쉬는 구간 없이 바로 이어진다 (자세 유지한 채 내림).
def _hold_at(sec):
    return [(sec, "tick", "■ 그대로 멈추기", "그대로 멈추기")]


def yaw_phases(side: str) -> list:
    A = "L" if side == "left" else "R"
    me, other = ("왼팔", "오른팔") if side == "left" else ("오른팔", "왼팔")
    return [
        (f"{A}_in_rest", f"{me} — 위팔 붙이고 팔꿈치 90°, 손을 배 쪽으로 천천히 → 멈춤 ({other} 차렷)", 8.0, "go", _hold_at(4.0)),
        CHECK_REST,
        (f"{A}_in_raised", f"{me} — 위팔을 조금(약 30°) 들고 손을 몸 앞 가운데로 천천히 → 멈춤", 8.0, "go", _hold_at(4.0)),
        (f"{A}_lower", f"{me} — 그 자세 그대로 위팔만 천천히 내리기 → 멈춤", 8.0, "go", _hold_at(4.0)),
        CHECK_REST,
        (f"{A}_out", f"{me} — 위팔 붙이고 팔꿈치 90°, 손을 바깥으로 천천히 → 멈춤", 8.0, "go", _hold_at(4.0)),
        CHECK_REST,
    ]


YAW_PHASES = (yaw_phases("left")
              + [("switch_arm", "이제 오른팔 차례 — 양팔 차렷으로 가만히", 6.0, "ready")]
              + yaw_phases("right")
              + [("both_in", "양팔 — 위팔을 조금 들고 팔꿈치 90°, 양손을 몸 앞 가운데로 천천히 → 멈춤", 10.0, "go",
                  _hold_at(5.0)),
                 CHECK_REST])

def wrist_phases() -> list:
    """REPORT_49: 손목 회전 측정 (tools/wrist_session.py). 동작·자리 잡기 구간은 멈춤(길이)이 끝난 뒤에 다음 예고를 한다
    (6번째 원소 preview_after) — 멈춤 동안 말하지 않게."""
    if WSESS is None:
        return []
    out = []
    for name, _voice, text, dur, kind in WSESS.steps():
        if kind == "rest":
            out.append(("rest", text, dur, "stop"))
        else:
            out.append((name, text, dur, "go", [], {"preview_after": dur}))
    return out


WRIST_PHASES = wrist_phases()

CHECK_PHASES = (check_phases("left")
                + [("switch_arm", "이제 오른팔 차례 — 양팔 차렷으로 가만히", 6.0, "ready")]
                + check_phases("right"))

POLICY_LABEL = {None: "", "hold": "C 유지", "none": "A 계산값", "recenter": "B 기준 복귀", "bone": "D 뼈 긴축"}


def make_laps(kind: str, retarget: str = "pos") -> list:
    """자동 세션의 바퀴 목록. 각 바퀴 = 방식(retarget) + 팔 편 상태 비틀기 정책 + 구간들."""
    if kind == "live":
        return [{"retarget": "pos", "sy_policy": None, "phases": AUTO_PHASES, "label": "위치 방식"},
                {"retarget": "bone", "sy_policy": None, "phases": AUTO_PHASES, "label": "뼈 방식"}]
    if kind == "speed":
        return [{"retarget": "pos", "sy_policy": None, "vel_ff": False, "phases": SPEED_PHASES,
                 "label": "피드포워드 끔", "tag": "FF 끔"},
                {"retarget": "pos", "sy_policy": None, "vel_ff": True, "phases": SPEED_PHASES,
                 "label": "피드포워드 켬", "tag": "FF 켬"}]
    if kind == "yaw":
        return [{"retarget": "pos", "sy_policy": None, "phases": YAW_PHASES, "label": "비틀기 한계·충돌 확인",
                 "big": True}]
    if kind == "wrist":
        return [{"retarget": "pos", "sy_policy": None, "phases": WRIST_PHASES, "label": "손목 회전 측정",
                 "big": True}]
    if kind == "check":
        return [{"retarget": "pos", "sy_policy": None, "phases": CHECK_PHASES, "label": "거르기·만세 확인",
                 "big": True}]
    if kind == "twist":
        return [{"retarget": "pos", "sy_policy": "hold", "phases": twist_phases("C"), "label": "C 유지(현행)"},
                {"retarget": "pos", "sy_policy": "bone", "phases": twist_phases("D"), "label": "D 뼈 긴축 회전"}]
    return [{"retarget": retarget, "sy_policy": None, "phases": [],
             "label": "위치 방식" if retarget == "pos" else "뼈 방식"}]


class AutoSession:
    """정해진 순서를 워치독이 직접 진행한다. 매 루프에서 step() 을 한 번 부른다 (막지 않는다)."""

    def __init__(self, lead: float, laps: Sequence, cue: CueSound, phase_csv: Optional[Path],
                 speed: float = 1.0, free: bool = False, start_json: Optional[Path] = None):
        self.free = free          # True 면 구간을 진행하지 않고 계속 따라가기만 한다
        self.speed = max(0.01, float(speed))
        self.lead = lead / self.speed
        self.laps = [lap if isinstance(lap, dict) else make_laps("free", lap)[0] for lap in laps]
        self.cue = cue
        self.stage = "lead"              # lead → calib → position → run → ret → position … → done/abort
        self.lap = 0
        self.index = -1
        self.t_stage = None
        self.t_phase = None
        self.last_tick = None
        self.lost_since = None
        self.rows: List[dict] = []
        self.phase = "-"
        self.phase_csv = phase_csv
        self.finished = False
        self.message = ""
        self.warnings: List[str] = []
        self.wrist_e90 = True               # A (REPORT_29): 손목 0 을 팔꿈치 90° 엄지 위에서 (--no-wrist-e90 로 끔)
        self.table_start = False            # 작업 지시 11-b --start-pose table: 손목 기준 뒤 차렷으로 안 돌아가고 팔꿈치 90° 로 모터를 켠다
        self.table_start_clear = 0.01       # 손 목표가 판 여유 위로 이만큼 (m) 떠 있어야 모터를 켠다
        self.table_start_hold = 0.5         # 그 상태가 이만큼 (초) 이어져야
        self._wrist_frames: list = []
        self.start_json = start_json        # 시작 단계 결과 (손목 기준·차렷 팔꿈치 값) — 화면에만 나오던 것을 파일로
        self.start_info: dict = {}
        self.start_check_cfg: dict = {"on": True}   # REPORT_45 §4 (main 이 옵션으로 채운다)
        self.run_info: dict = {}                    # _start.json 에 같이 남길 실행 설정 (충돌 문턱 등, REPORT_45 §5)
        self._extra: list = []              # 지금 구간의 추가 소리 [(초, 소리, 문구[, 음성])]
        self._previewed: set = set()        # 음성 예고를 이미 한 (바퀴, 구간 번호)
        self.late_previews: list = []       # 앞에 쉬는 구간이 없어 예고 시간을 따로 넣은 구간
        self.voice_on = bool(cue.voice and cue.sock is not None)   # 음성이 꺼져 있으면 시각은 예전 그대로
        self._voice_end = -1e9              # 지금 말이 끝날 것으로 보는 시각 (monotonic, 여유 제외)
        self._say_queue: list = []          # (보낼 시각, 문구, 구간) — 한 예고 안의 둘째 문구부터
        self._gate = None                   # "말 → 삑" 대기: (다음 단계, 화면 문구, 소리, 띠, 여유초)
        self.cur_dur = 0.0                  # 지금 구간 길이 (쉬는 구간은 말 때문에 늘 수 있다)
        self.voice_log: list = []           # (monotonic, 문구, 예상 끝) — dry-run 표용
        self.phase_text = ""

    # -- 현재 바퀴
    @property
    def cur(self) -> dict:
        return self.laps[min(self.lap, len(self.laps) - 1)]

    @property
    def phases(self) -> list:
        return self.cur["phases"]

    @property
    def policy(self) -> str:
        return self.cur.get("sy_policy") or ""

    def status_text(self, now: float) -> str:
        """상태줄 앞머리: [정책] 구간 남은시간."""
        tag = self.cur.get("tag") or POLICY_LABEL.get(self.cur.get("sy_policy"), "")
        head = f"<{tag}> " if tag else ""
        if self.stage == "run" and 0 <= self.index < len(self.phases):
            left = max(0.0, self.cur_dur / self.speed - (now - self.t_phase))
            if self.cur.get("big"):
                return f"▶▶ {self.phase} 남은 {left:4.1f}초 ◀◀ {self.phase_text[:28]} | "
            return f"{head}{self.phase} 남은 {left:4.1f}s | "
        return f"{head}{self.stage} | "

    # -- 표시/기록
    def _announce(self, text: str, sound: str = "", phase: str = "", banner: bool = False) -> None:
        self.message = text
        if banner:
            bar = "═" * 72
            print(f"\n{bar}\n  {text}\n{bar}")
        else:
            print(f"\n{text}")
        if sound:
            self.cue.send(sound, text=text, phase=phase)

    def _close_row(self, now: float) -> None:
        if self.rows and self.rows[-1]["t_end_mono"] is None:
            self.rows[-1]["t_end_mono"] = now
            self.rows[-1]["t_end_epoch"] = time.time()

    def _open_row(self, now: float, phase: str, note: str) -> None:
        self._close_row(now)
        self.rows.append({"index": len(self.rows), "phase": phase, "note": note,
                          "retarget": self.cur["retarget"], "sy_policy": self.policy,
                          "vel_ff": int(bool(self.cur.get("vel_ff", False))),
                          "t_start_mono": now, "t_end_mono": None,
                          "t_start_epoch": time.time(), "t_end_epoch": None})
        self.phase = phase

    def write_csv(self) -> Optional[Path]:
        if not self.phase_csv or not self.rows:
            return None
        t0 = self.rows[0]["t_start_mono"]
        with open(self.phase_csv, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["index", "phase", "note", "retarget", "sy_policy", "vel_ff", "t_start_mono", "t_end_mono",
                        "t_start_epoch", "t_end_epoch", "t_start_rel", "t_end_rel"])
            for r in self.rows:
                end = r["t_end_mono"]
                w.writerow([r["index"], r["phase"], r["note"], r["retarget"], r["sy_policy"], r["vel_ff"],
                            f"{r['t_start_mono']:.3f}", "" if end is None else f"{end:.3f}",
                            f"{r['t_start_epoch']:.3f}",
                            "" if r["t_end_epoch"] is None else f"{r['t_end_epoch']:.3f}",
                            f"{r['t_start_mono'] - t0:.3f}",
                            "" if end is None else f"{end - t0:.3f}"])
        return self.phase_csv

    def write_start(self) -> None:
        """시작 단계 결과를 파일로 (손목 기준 성공/실패·그때 팔꿈치각·차렷 대비 이동, 차렷 팔꿈치 값)."""
        if not self.start_json:
            return
        try:
            info = dict(self.start_info, time=time.strftime("%Y-%m-%dT%H:%M:%S"))
            Path(self.start_json).write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding="utf-8")
        except OSError as exc:
            print(f"\n[AUTO] 시작 단계 결과를 못 남겼다: {exc}")

    def reject_start(self, reason: str, phrase: str, now: float) -> None:
        """REPORT_45 §4: 모터를 켜기 전에 시작하지 않는다. 화면 + 경보음, cue_player 가 문구를 알면 음성도."""
        self._close_row(now)
        self.stage = "abort"
        self.finished = True
        self._announce(f"[시작 확인] 시작하지 않습니다 — {reason}", sound="alarm", banner=True)
        if phrase and voice_known(phrase):
            self._say(phrase)
        elif phrase and self.voice_on:
            print(f"[시작 확인] (음성 생략 — 서버 PC cue_player 가 아직 이 문구를 모른다: '{phrase}')")

    def abort(self, reason: str, now: float) -> None:
        self._close_row(now)
        self.stage = "abort"
        self.finished = True
        self._announce(f"[중단] {reason}", sound="alarm")
        self._say(VOICE_ABORT)

    # -- 진행
    def _say(self, text: str, phase: str = "") -> None:
        """음성 + 끝날 시각 계산 (cue_player 는 한 번에 하나씩 말하므로 앞 말이 끝난 뒤부터 센다)."""
        if not self.voice_on or not text:
            return
        now = time.monotonic()
        start = max(now, self._voice_end)
        if VOICE_SEP in text:                # REPORT_49: 문구 여러 개 — 첫 문구는 지금 보내고 나머지는 앞 문구가 끝난 뒤에
            parts = [t for t in text.split(VOICE_SEP) if t]
            for i, part in enumerate(parts):
                if i == 0:
                    self.cue.say(part, phase=phase)
                else:
                    self._say_queue.append((start, part, phase))
                start += voice_seconds(part) / self.speed
                self.voice_log.append((now, part, start))
                if i < len(parts) - 1:
                    start += VOICE_PART_GAP / self.speed
            self._voice_end = start
            return
        self._voice_end = start + voice_seconds(text) / self.speed
        self.voice_log.append((now, text, self._voice_end))
        self.cue.say(text, phase=phase)

    def _pump_voice(self, now: float) -> None:
        """줄 세운 문구를 때가 되면 보낸다 (매 루프)."""
        while self._say_queue and now >= self._say_queue[0][0]:
            _t, part, phase = self._say_queue.pop(0)
            self.cue.say(part, phase=phase)

    def _voice_ready(self, now: float, extra: float) -> bool:
        return (not self.voice_on) or now >= self._voice_end + extra / self.speed

    def _gate_to(self, now, next_stage, phrase, text, sound, banner=False, margin=VOICE_MARGIN) -> None:
        """말 → (말 길이 + 여유) → 삑 → next_stage 시작 (그때부터 측정). 음성이 꺼져 있으면 바로."""
        self._say(phrase)
        self._gate = (next_stage, text, sound, banner, margin)
        self.stage = "gate"
        self._stage_gate(now, None)

    def _stage_gate(self, now, ctx):
        nxt, text, sound, banner, margin = self._gate
        if not self._voice_ready(now, margin):
            return
        self._announce(text, sound=sound, banner=banner)
        self.stage, self.t_stage, self.last_tick = nxt, now, None
        if nxt == "calib":
            self._calib_frames = []
        elif nxt == "wrist":
            self._wrist_frames = []

    def step(self, now: float, ctx: dict) -> None:
        """ctx: tracking_ok, frame, state, motor, retargeters, pos_runner, robot_ref, dt"""
        self._pump_voice(now)
        if self.finished:
            return
        if self.t_stage is None:
            self.t_stage = now
            self._gate_to(now, "lead", VOICE_START["lead"], "[준비] VR 착용하고 차렷으로 서세요", "ready")

        # 트래킹이 끊기면 진행을 멈추고 기다린다 (복귀하면 이어서 간다)
        if self.stage in ("run", "free") and not ctx["tracking_ok"]:
            if self.lost_since is None:
                self.lost_since = now
                self._announce("[대기] VR 트래킹이 끊겼습니다. 진행을 멈추고 기다립니다 (5초 넘으면 중단)",
                               sound="alarm")
            elif now - self.lost_since > AUTO_TRACKING_GRACE_SEC:
                self.abort("트래킹이 5초 넘게 끊겼습니다 → 힘을 빼고 종료합니다", now)
            return
        if self.lost_since is not None and ctx["tracking_ok"]:
            self.lost_since = None
            self._announce("[재개] 트래킹 복구. 이어서 진행합니다", sound="go")

        getattr(self, f"_stage_{self.stage}")(now, ctx)

    def _stage_lead(self, now, ctx):
        left = self.lead - (now - self.t_stage)
        if left <= 3.0 and (self.last_tick is None or now - self.last_tick >= 1.0) and left > 0:
            self.last_tick = now
            self.cue.send("tick", left=left)
        if left <= 0:
            self._gate_to(now, "calib", VOICE_START["calib"], "[중립] 차렷 그대로 2초 멈추세요", "ready")

    def _stage_calib(self, now, ctx):
        if now - self.t_stage < CALIBRATION_DELAY_SEC / self.speed:
            # 마지막 1초의 차렷 프레임을 모은다 (머리 위 B: 사람 차렷 팔꿈치 값을 사람별로 잡는다)
            if (now - self.t_stage >= (CALIBRATION_DELAY_SEC - 1.0) / self.speed and ctx["tracking_ok"]
                    and ctx["frame"] is not None):
                self._calib_frames = (getattr(self, "_calib_frames", []) + [ctx["frame"]])[-60:]
            return
        frame = ctx["frame"]
        ok = bool(frame) and ctx["tracking_ok"] and all(r.calibrate(frame) for r in ctx["retargeters"].values())
        if ok and not ctx["pos_runner"].calibrate(frame, getattr(self, "_calib_frames", None)):
            ok = False
        if not ok:
            self.abort("중립 저장 실패 — 팔 본이 안 보입니다. VR 트래킹을 확인하고 다시 실행하세요", now)
            return
        self._announce("[중립] 저장했습니다 (위치 방식 기준 포함)")
        runner = ctx["pos_runner"]
        hand = any(lap.get("retarget") == "hand" for lap in self.laps)
        self.start_info["retarget"] = "hand" if hand else str(self.laps[0].get("retarget", "pos")) if self.laps else "pos"
        self.start_info["elbow_rest_deg"] = {s: round(v, 1) for s, v in runner.elbow_base_deg().items()}
        # hand 는 차렷 팔꿈치를 빼지 않는다 (배율로 대신) — 실제 적용값을 적는다 (REPORT_45 §5)
        self.start_info["elbow_base_on"] = bool(runner.filters.get("elbow_base")) and not hand
        self.start_info["calib_frames"] = len(getattr(self, "_calib_frames", []) or [])
        self.start_info.update(self.run_info)
        if hand and runner.cal is not None and runner.ht is not None:   # REPORT_38
            c = runner.cal
            self.start_info["hand"] = {
                "upper_len_cm": {s: round(c.upper_len.get(s, float("nan")), 1) for s in runner.sides},
                "fore_len_cm": {s: round(c.fore_len.get(s, float("nan")), 1) for s in runner.sides},
                "reach_rest_cm": {s: round(c.reach_rest.get(s, float("nan")), 1) for s in runner.sides},
                "shoulder_width_cm": round(c.shoulder_width, 1),
                "scale": {s: round(runner.ht.k[s] * 100, 4) for s in runner.sides},
                "cfg": runner.hand_cfg}
        sc = self.start_check_cfg
        if sc.get("on", True):
            chk = runner.start_check(getattr(self, "_calib_frames", None) or [frame], hand,
                                     sc.get("gap_min_cm", START_GAP_MIN_CM), sc.get("scale_range", START_SCALE_RANGE))
            self.start_info["start_check"] = {"ok": chk["ok"], "gap_cm": round(float(chk["gap_cm"]), 1),
                                              "scale": {k: round(v, 3) for k, v in chk["scale"].items()},
                                              "reason": chk["reason"]}
            if not chk["ok"]:
                self.write_start()
                self.reject_start(str(chk["reason"]), str(chk["phrase"]), now)
                return
            self._announce(f"[시작 확인] 통과 — 두 손목 좌우 간격 {chk['gap_cm']:+.0f} cm"
                           + (" · 배율 " + " / ".join(f"{v:.2f}" for v in chk["scale"].values()) if chk["scale"] else ""))
        else:
            self.start_info["start_check"] = {"ok": None, "reason": "끔 (--no-start-check)"}
        self.write_start()
        if hand:
            self._announce("[손 위치] 차렷 기준 — " + runner.hand_calib_text()
                           + " (차렷 팔꿈치 빼기는 쓰지 않는다)")
        elif runner.filters.get("elbow_base"):
            base = runner.elbow_base_deg()
            self._announce("[팔꿈치] 차렷 값 — " + ", ".join(
                f"{'왼' if s == 'left' else '오른'} {base[s]:.0f}°" for s in runner.sides)
                + " (로봇은 이 값을 0° 로 본다)")
            high = [s for s, v in base.items() if not (v <= math.degrees(PR.ELBOW_BASE_MAX))]
            if high:
                msg = (f"⚠ 차렷 팔꿈치 값이 {math.degrees(PR.ELBOW_BASE_MAX):.0f}° 를 넘는다 ("
                       + ", ".join("왼팔" if s == "left" else "오른팔" for s in high)
                       + ") — 팔을 굽히고 있었나? 그 값까지만 뺀다")
                self.warnings.append(msg)
                self._announce(msg, sound="alarm", banner=True)
        if self.wrist_e90:
            self._gate_to(now, "wrist", VOICE_START["wrist"],
                          "[손목] 양팔 팔꿈치를 90° 로 굽히고, 엄지가 위 — 3초 멈추세요", "learn", banner=True)
        elif self.table_start:
            self._enter_table_wait(now)
        else:
            self.stage, self.t_stage = "position", now

    def _enter_table_wait(self, now) -> None:
        """작업 지시 11-b: 팔꿈치 90° 를 유지한 채 기다린다. "손을 수평보다 조금 높게" 는 서버 PC 가 모르면 있는 문구로 대신한다."""
        self._table_ok_since = self._table_last_say = None
        if voice_known(VOICE_TABLE_HIGH) or not self.voice_on:
            phrase = VOICE_TABLE_HIGH
        else:
            phrase = VOICE_START["wrist"]
            print(f"[시작 순서] (음성 대신 '{phrase}' — 서버 PC cue_player 가 아직 '{VOICE_TABLE_HIGH}' 를 모른다)")
        self._gate_to(now, "table_wait", phrase, "[시작 순서] 팔꿈치 90° 그대로, 손을 수평보다 조금 높게 — 판 위에 있으면 모터가 켜집니다", "ready", banner=True)

    def _stage_table_wait(self, now, ctx):
        """모터를 켜기 전: 사람 손 그대로의 목표가 판 면 제한을 여유 있게 지키는지 본다. 아니면 기다린다 (시간 제한 없음, Ctrl+C 로 끝낼 수 있다)."""
        runner = ctx["pos_runner"]
        ok, gs = False, {}
        if ctx["tracking_ok"] and ctx["frame"] is not None and runner.ht is not None and runner.ht.table is not None:
            bones = runner.view_bones(ctx["frame"])
            gs = runner.ht.table_preview(bones, runner._frame_cal(bones), sides=runner.sides)
            ok = all(v >= self.table_start_clear for v in gs.values())
        if ok:
            if self._table_ok_since is None:
                self._table_ok_since = now
            if now - self._table_ok_since >= self.table_start_hold / self.speed:
                self.start_info["table_start"] = {"clear_cm": {s: round(v * 100, 1) for s, v in gs.items()}, "waited_s": round(now - self.t_stage, 1)}
                self.write_start()
                self._announce("[시작 순서] 손 목표 판 위 여유 — " + ", ".join(f"{'왼' if s == 'left' else '오른'} {v * 100:+.1f} cm" for s, v in gs.items()))
                self.stage, self.t_stage = "position", now
            return
        self._table_ok_since = None
        if self._table_last_say is None or now - self._table_last_say >= 6.0 / self.speed:
            if self._table_last_say is not None:
                self._announce("[시작 순서] 기다리는 중 — 손을 조금 더 높게"
                               + (" (" + ", ".join(f"{'왼' if s == 'left' else '오른'} {v * 100:+.1f} cm" for s, v in gs.items()) + ")" if gs else " (추적 없음)"), sound="tick")
            self._table_last_say = now

    def _stage_wrist(self, now, ctx):
        """A (REPORT_29): 사람 손목 0 을 로봇 기준과 같은 자세(팔꿈치 90° 엄지 위)에서 잡는다.
        실패해도 중단하지 않고 차렷 때 잡은 0 을 그대로 쓴다 (경고)."""
        hold = 3.0 / self.speed
        el = now - self.t_stage
        if el >= hold - 1.0 / self.speed and ctx["tracking_ok"] and ctx["frame"] is not None:
            self._wrist_frames.append(ctx["frame"])       # 마지막 1초 동안의 프레임
        if el < hold:
            return
        runner = ctx["pos_runner"]
        frames = self._wrist_frames[-30:]
        elbows = {s: runner.elbow_deg(frames[-1], s) if frames else float("nan") for s in runner.sides}
        bad = [s for s, e in elbows.items() if not (e >= 60.0)]
        if not frames or bad:
            names = ", ".join("왼팔" if s == "left" else "오른팔" for s in (bad or runner.sides))
            msg = (f"⚠ 손목 기준을 못 잡았다 ({names} 팔꿈치 "
                   + ", ".join(f"{elbows[s]:.0f}°" for s in runner.sides) + " — 60° 이상 굽혀야 한다). "
                   "차렷 때 잡은 손목 0 을 그대로 쓴다")
            self.warnings.append(msg)
            self._announce(msg, sound="alarm", banner=True)
            self.start_info["wrist_ref"] = {"ok": False, "elbow_deg": {s: round(e, 1) for s, e in elbows.items()},
                                            "frames": len(frames)}
        else:
            shift = runner.calibrate_wrist(frames)
            self.start_info["wrist_ref"] = {"ok": True, "elbow_deg": {s: round(e, 1) for s, e in elbows.items()},
                                            "frames": len(frames),
                                            "shift_from_rest_deg": {s: round(v, 1) for s, v in shift.items()}}
            self._announce("[손목] 기준을 잡았다 — 차렷 대비 "
                           + ", ".join(f"{'왼' if s == 'left' else '오른'} {shift[s]:+.0f}°" for s in runner.sides))
        self.write_start()
        if self.table_start:
            self._enter_table_wait(now)
            return
        self._gate_to(now, "back", VOICE_START["back"], "[중립] 차렷으로 돌아오세요 — 3초 뒤 모터가 켜집니다", "ready")

    def _stage_back(self, now, ctx):
        if now - self.t_stage >= 3.0 / self.speed:
            self.stage, self.t_stage = "position", now

    def _stage_position(self, now, ctx):
        if now - self.t_stage < 1.0 / self.speed:
            return
        self._announce("[모터] 힘이 들어갑니다. 로봇이 살짝 움직일 수 있습니다", sound="go")
        if not ctx["motor"].enter_position_from_current():
            self.abort("POSITION 진입 실패 — 자세나 한계를 확인하세요", now)
            return
        ctx["state"]["hold"] = False
        ctx["state"]["retarget"] = self.cur["retarget"]
        if self.cur.get("sy_policy"):
            ctx["pos_runner"].set_policy(self.cur["sy_policy"])
        if "vel_ff" in self.cur:
            ctx["motor"].vel_ff = bool(self.cur["vel_ff"])
        if self.free:
            self.phase = "free"
            self._announce(f"[자유] 따라가기 시작 — {self.cur['label']}. 끝내려면 h(멈춤) → x(힘 빼기) 또는 Ctrl+C")
            self.stage, self.t_phase = "free", now
            return
        self._announce(f"[모터] 따라가기 시작 — {self.cur['label']}", banner=True)
        self.stage, self.index, self.t_phase = "run", -1, now

    def _check_offsets(self, ctx) -> None:
        """D 학습 구간이 끝났을 때 양팔 모두 보정값을 배웠는지 확인한다. 못 배웠으면 경고 (중단은 안 한다)."""
        runner = ctx["pos_runner"]
        missing = [side for side in runner.sides
                   if getattr(runner.rts.get(side), "bone_offset", None) is None]
        if missing:
            names = ", ".join("왼팔" if s == "left" else "오른팔" for s in missing)
            msg = (f"⚠ D 보정값을 못 배웠다: {names}. 팔꿈치를 충분히(50° 이상) 굽히지 않은 것 같다. "
                   "이 팔의 D 결과는 믿기 어렵다")
            self.warnings.append(msg)
            self._announce(msg, sound="alarm", banner=True)
        else:
            vals = ", ".join(f"{'왼' if s == 'left' else '오른'} {math.degrees(runner.rts[s].bone_offset):+.0f}°"
                             for s in runner.sides)
            self._announce(f"[D] 보정값 학습 확인 — {vals}")

    def _stage_run(self, now, ctx):
        ph = self.phases
        if self.index < 0 or now - self.t_phase >= self.cur_dur / self.speed:
            # 말 → 삑: 다음이 동작 구간이면 예고가 끝나고 (여유 0.5 + 1.5초) 뒤에 넘어간다 (지금 구간이 그만큼 늘어난다)
            nxt = self.index + 1
            if self.voice_on and nxt < len(ph) and ph[nxt][0] not in ("rest", "switch_arm"):
                if (self.lap, nxt) not in self._previewed:
                    self._say(voice_for(ph[nxt][0]), phase=ph[nxt][0])
                    self._previewed.add((self.lap, nxt))
                    self.late_previews.append(f"{self.lap + 1}바퀴 {ph[nxt][0]}")
                if not self._voice_ready(now, VOICE_MARGIN + VOICE_GAP_BEFORE_MOVE):
                    return
            if 0 <= self.index < len(ph) and ph[self.index][0].endswith("_learn"):
                self._check_offsets(ctx)
            self.index += 1
            if self.index >= len(ph):
                self._close_row(now)
                self.lap += 1
                if self.lap >= len(self.laps):
                    self.stage = "done"
                    return
                self.stage, self.t_stage = "ret", now
                self._announce("[전환] 차렷으로 서서 기다리세요. 로봇이 시작 자세로 돌아갑니다",
                               sound="switch", banner=True)
                self._say(VOICE_NEXT_LAP)
                nxt = self.laps[self.lap]["phases"]
                if self.voice_on and nxt and voice_for(nxt[0][0]):
                    # 복귀하는 동안 "다음 바퀴" 가 끝나면 첫 구간 예고
                    self._extra = [(voice_seconds(VOICE_NEXT_LAP) + VOICE_MARGIN, None, "", voice_for(nxt[0][0]))]
                    self.t_phase = now
                    self._previewed.add((self.lap, 0))
                ctx["state"]["hold"] = True
                return
            name, text, dur, sound = ph[self.index][:4]
            for item in self._extra:            # 못 낸 예고(복귀가 짧았던 경우 등)는 지금 말한다
                if len(item) > 3 and item[3]:
                    self._say(item[3])
            self._extra = list(ph[self.index][4]) if len(ph[self.index]) > 4 else []
            self.phase_text = text
            self.cur_dur = dur
            self.t_phase, self.last_tick = now, None
            moves = [p for p in ph if p[0] not in ("rest", "switch_arm")]
            n = sum(1 for p in ph[:self.index + 1] if p[0] not in ("rest", "switch_arm"))
            if name == "switch_arm":
                head = "[교대]"
            elif name == "rest":
                head = "[정지]"
            else:
                head = f"[{n}/{len(moves)}]"
            tag = self.cur.get("tag") or POLICY_LABEL.get(self.cur.get("sy_policy"), "")
            self._open_row(now, name, text)
            self._announce(f"{('<' + tag + '> ') if tag else ''}{head} {text}   {dur:.0f}초",
                           sound=sound, phase=name, banner=(sound != "stop" or bool(self.cur.get("big"))))
            self._voice_on_open(ph, name)
            return
        self._fire_extra(now)
        left = self.cur_dur / self.speed - (now - self.t_phase)
        speaking = self.voice_on and now < self._voice_end + VOICE_MARGIN / self.speed   # 말하는 동안은 초읽기 생략
        if left <= 3.0 and not speaking and (self.last_tick is None or now - self.last_tick >= 1.0):
            self.last_tick = now
            self.cue.send("tick", left=left)

    def _fire_extra(self, now) -> None:
        """구간 안 추가 소리·음성 (구간 시작부터 정한 초에)."""
        while self._extra and now - self.t_phase >= self._extra[0][0] / self.speed:
            item = self._extra.pop(0)
            extra_sound, extra_text = item[1], item[2]
            extra_say = item[3] if len(item) > 3 else ""
            if extra_text:
                self.phase_text = extra_text
                self._announce(f"{extra_text}", sound=extra_sound or "", phase=self.phase, banner=True)
            elif extra_sound:
                self.cue.send(extra_sound, phase=self.phase)
            if extra_say:
                self._say(extra_say, phase=self.phase)

    def _voice_on_open(self, ph, name) -> None:
        """음성: 쉬는 구간이 시작되면 "차렷"(교대면 "오른팔로 바꿉니다") → 그 말이 끝나면(+0.5초) 다음 동작 예고.
        쉬는 구간 길이 = max(정한 길이, 말들 + 여유 + 1.5초). 동작 구간은 예고를 앞에서 이미 했다 (_stage_run)."""
        if not self.voice_on:
            return
        nxt = self.index + 1
        upcoming = voice_for(ph[nxt][0]) if nxt < len(ph) else ""
        if name not in ("rest", "switch_arm"):
            # 동작 구간 뒤에 쉬는 구간 없이 또 동작이 오면 (yaw: 위팔 든 채 → 내리기) 이 구간 안에서 다음 동작을 예고한다:
            # 구간 안 마지막 음성("그대로 멈추기" 등)이 끝난 뒤(+0.5초). 예고 + 여유가 구간 끝을 넘으면 이 구간이 그만큼 늘어난다.
            if upcoming and ph[nxt][0] not in ("rest", "switch_arm"):
                t_last = max([e[0] + voice_seconds(e[3]) for e in self._extra if len(e) > 3 and e[3]] or [0.0])
                opts = ph[self.index][5] if len(ph[self.index]) > 5 else {}
                t_last = max(t_last, float(opts.get("preview_after", 0.0)))   # REPORT_49: 멈춤이 끝난 뒤에 예고
                self._extra.append((t_last + VOICE_MARGIN, None, "", upcoming))
                self._extra.sort(key=lambda e: e[0])
                self._previewed.add((self.lap, nxt))
            return
        first = VOICE_SWITCH_ARM if name == "switch_arm" else VOICE_REST
        self._say(first, phase=name)
        if upcoming:
            t_prev = voice_seconds(first) + VOICE_MARGIN
            self._extra.append((t_prev, None, "", upcoming))
            self._previewed.add((self.lap, nxt))
            need = t_prev + voice_seconds(upcoming) + VOICE_MARGIN + VOICE_GAP_BEFORE_MOVE
            self.cur_dur = max(self.cur_dur, need)

    def _stage_ret(self, now, ctx):
        """시작 자세(차렷 기준값)로 속도 제한 안에서 천천히 복귀. 이 동안 사람 입력은 안 받는다."""
        self._fire_extra(now)
        motor, ref = ctx["motor"], ctx["robot_ref"]
        deltas = {side: {joint: ref.rest[side][joint] - motor.base_target[side][joint]
                         for joint in JOINT_ORDER}
                  for side in motor.sides}
        motor.update_targets(deltas, ctx["dt"])
        done = all(abs(motor.command_target[s][j] - ref.rest[s][j]) < AUTO_RETURN_TOL_RAD
                   for s in motor.sides for j in JOINT_ORDER)
        if now - self.t_stage > AUTO_RETURN_TIMEOUT_SEC:
            self.abort("시작 자세 복귀가 끝나지 않습니다", now)
            return
        if done:
            ctx["state"]["retarget"] = self.cur["retarget"]
            self._announce(f"[전환] 돌아왔습니다. {self.cur['label']} 으로 바꿉니다", sound="switch")
            self.stage, self.t_stage = "position", now

    def _stage_free(self, now, ctx):
        """구간 없이 계속 따라간다. 트래킹이 끊기면 위쪽 공통 처리가 잡는다."""
        return

    def _stage_done(self, now, ctx):
        self._close_row(now)
        self.finished = True
        self._announce(f"[완료] {len(self.laps)}바퀴 끝났습니다. 힘을 뺐습니다", sound="done", banner=True)
        self._say(VOICE_DONE)
        if self.voice_on and self.late_previews:
            print("[음성] 앞에 쉬는 구간이 없어 예고 시간을 따로 넣은 구간: " + ", ".join(self.late_previews))
        for w in self.warnings:
            print(w)

    def _stage_abort(self, now, ctx):
        self.finished = True


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
                delay = state.get("calibrate_delay", CALIBRATION_DELAY_SEC)
                state["calibrate_requested_at"] = time.monotonic()
                print(f"\n[CALIBRATE] hold your pose -- capturing in {delay:.1f}s")
            elif key == "p":
                state["position"] = True
            elif key == "h":
                state["hold_request"] = True
            elif key == "x":
                state["damping"] = True
            elif key == "y":
                state["cycle_sy_policy"] = True
            elif key == "m":
                state["cycle_speed"] = True
            elif key == "n" and state.get("episodes"):         # 작업 지시 13 --episodes (끄면 n·s 는 아무것도 안 하고 f 는 예전 그대로)
                state["ep_new"] = True
            elif key == "s" and state.get("episodes"):
                state["ep_success"] = True
            elif key == "f" and state.get("episodes"):
                state["ep_fail"] = True
            elif key == "f":
                state["toggle_vel_ff"] = True
            elif key == "g" and state.get("replay"):          # 작업 지시 14 --replay (끄면 g 는 아무것도 안 한다)
                state["replay_go"] = True
            elif key == "v":
                state["toggle_retarget"] = True
            elif key == "b":
                # 가슴 기준 상대화 켜고/끄기. 중립을 두 형태로 저장해 두므로
                # 재캘리브레이션 없이 바로 전환된다 (전환 직후 목표가 조금 튈 수 있으나
                # 속도 제한이 완충한다. 불안하면 h 로 hold 한 뒤 누르면 된다).
                state["toggle_chest"] = True
    except (EOFError, OSError):
        print("\n[KEYBOARD] interactive keys unavailable; Ctrl+C still works")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="test1 Unreal UDP to Berkeley Humanoid Lite dual-arm control"
    )
    parser.add_argument("--side", choices=("both", "left", "right"), default="both")
    parser.add_argument(
        "--left-channel",
        default=DEFAULT_CHANNELS["left"],
        help="SocketCAN channel for the left arm (default can0)",
    )
    parser.add_argument(
        "--right-channel",
        default=DEFAULT_CHANNELS["right"],
        help="SocketCAN channel for the right arm (default can1)",
    )
    parser.add_argument(
        "--left-ids",
        type=parse_id_list,
        default=DEFAULT_ARM_IDS["left"],
        help="five IDs: shoulder pitch, roll, yaw, elbow pitch, elbow roll",
    )
    parser.add_argument(
        "--right-ids",
        type=parse_id_list,
        default=DEFAULT_ARM_IDS["right"],
        help="five IDs: shoulder pitch, roll, yaw, elbow pitch, elbow roll",
    )
    parser.add_argument(
        "--udp-ip",
        default="0.0.0.0",
        help="local bind address; 0.0.0.0 receives Unreal traffic sent to 192.168.0.65",
    )
    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    parser.add_argument(
        "--urdf",
        default=None,
        help="optional berkeley_humanoid_lite.urdf path; embedded limits are the default",
    )
    parser.add_argument("--kp", type=float, default=DEFAULT_KP)
    parser.add_argument("--kd", type=float, default=DEFAULT_KD)
    parser.add_argument("--torque-limit", type=float, default=DEFAULT_TORQUE_LIMIT)
    parser.add_argument("--max-speed", type=float, default=MAX_TARGET_SPEED_RAD_S)
    parser.add_argument(
        "--ema-alpha",
        type=float,
        default=DEFAULT_EMA_ALPHA,
        help="VR quaternion smoothing, applied per new Unreal packet. "
             "Lower = smoother/more lag, 1.0 = off (default %(default)s)",
    )
    parser.add_argument(
        "--swing-suppression-exponent",
        type=float,
        default=0.0,
        help="Base suppress_minor_swing_axis() strength at moderate swing "
             "magnitudes (below --swing-taper-start). Default is 0 "
             "(disabled) -- raw shoulder_pitch/shoulder_roll pass through "
             "unmodified. Set > 0 (e.g. 1.5) to re-enable magnitude-based "
             "cross-talk suppression; see SWING_SUPPRESSION_EXPONENT and "
             "effective_swing_suppression_exponent() for what it does and "
             "its known trade-offs.",
    )
    parser.add_argument(
        "--swing-taper-start",
        type=float,
        default=SWING_SUPPRESSION_TAPER_START_RAD,
        help="Total |roll|+|yaw| (rad) below which suppression stays at "
             "full strength (default %(default)s)",
    )
    parser.add_argument(
        "--swing-taper-end",
        type=float,
        default=SWING_SUPPRESSION_TAPER_END_RAD,
        help="Total |roll|+|yaw| (rad) at/above which suppression is fully "
             "disabled -- large, near-end-of-range motions have genuine "
             "anatomical coupling that can exceed the intended axis, which "
             "fools magnitude-based suppression into crushing the real "
             "signal (default %(default)s)",
    )
    parser.add_argument(
        "--calibrate-delay",
        type=float,
        default=CALIBRATION_DELAY_SEC,
        help="Seconds between pressing 'c' and actually capturing the VR "
             "neutral pose, so moving your hand to the keyboard doesn't "
             "disturb the pose (default %(default)s)",
    )
    parser.add_argument(
        "--chest-relative",
        dest="chest_relative",
        action="store_true",
        default=True,
        help="Chest 본 기준으로 어깨를 상대화한다 (기본 켜짐). 몸통 회전이 어깨 명령으로 "
             "새는 것을 없앤다",
    )
    parser.add_argument(
        "--no-chest-relative",
        dest="chest_relative",
        action="store_false",
        help="월드 기준(패치 전 동작)으로 돌린다. 같은 세션에서 비교할 때는 실행 중 'b' 키로도 "
             "전환할 수 있다",
    )
    parser.add_argument(
        "--retarget", choices=("bone", "pos", "hand"), default="bone",
        help="bone = 기존 뼈 회전 방식(기본), pos = 관절 위치 기반 (REPORT_19), hand = 손 위치 기준 (REPORT_38, "
             "자동 세션도 pos 대신 hand 로 돈다). 실행 중 v 로 bone 과 전환")
    parser.add_argument("--gripper-port", default=None,
                        help="그리퍼 아두이노 직렬 포트 (예: /dev/ttyUSB0, ardgripper_rt). 없으면 그리퍼를 안 쓴다 (REPORT_51)")
    parser.add_argument("--gripper-open-deg", type=float, nargs=2, default=[0.0, 0.0], metavar=("A", "B"),
                        help="열림 각도 (A=왼, B=오른, 기본 0 0)")
    parser.add_argument("--gripper-close-deg", type=float, nargs=2, default=[90.0, 90.0], metavar=("A", "B"),
                        help="닫힘 각도 (기본 90 90)")
    parser.add_argument("--grip-close-cm", type=float, default=3.0, help="엄지–검지 거리가 이하면 바로 닫힘 (기본 3.0)")
    parser.add_argument("--grip-open-cm", type=float, default=4.5, help="이 이상이 --grip-open-hold-s 이어지면 열림 (기본 4.5)")
    parser.add_argument("--grip-open-hold-s", type=float, default=0.4,
                        help="열림 전에 기다리는 시간 (기본 0.4 — 방아쇠 당긴 채 엄지 뗄 때 0.25초 튐을 무시)")
    # 작업 지시 14-p (기본 끔 = pinch: 끄면 명령·csv·패킷이 예전과 비트 단위로 같다)
    parser.add_argument("--grip-open-rule", choices=("pinch", "finger"), default="pinch",
                        help="그리퍼 열림 판정. pinch = 지금 그대로 (기본, ≥ --grip-open-cm 이 --grip-open-hold-s 이어지면 열림). "
                             "finger = 출처가 컨트롤러(ctrl)일 때 ≥ 7.0 (손가락 뗌) 은 --grip-open-hold-s 뒤 열림, 4.5~7 (손가락이 트리거에 얹힘) 은 "
                             "--grip-mid-open-s 이어져야 열림, 반대 손이 열린 뒤 --grip-handover-guard-s 동안은 중간값으로 안 열림. 손 추적(hand)은 그대로. "
                             "CSV grip_rule_L/R (0 그대로, 1 손가락 뗌, 2 중간 오래, 3 인계 보호로 막음)")
    parser.add_argument("--grip-mid-open-s", type=float, default=1.5,
                        help="--grip-open-rule finger: 중간값 (4.5~7) 만으로 열리는 데 걸리는 시간 (기본 1.5)")
    parser.add_argument("--grip-handover-guard-s", type=float, default=3.0,
                        help="--grip-open-rule finger: 반대 손이 닫힘 → 열림으로 바뀐 뒤 이 동안은 중간값으로 열리지 않음 (기본 3.0)")
    parser.add_argument("--no-start-check", action="store_true",
                        help="중립 직후 시작 확인(컨트롤러 바꿔 쥠·hand 배율, REPORT_45 §4)을 끈다")
    parser.add_argument("--start-gap-min-cm", type=float, default=START_GAP_MIN_CM,
                        help="시작 확인: 중립 때 두 손목 좌우 간격(왼쪽 +)이 이보다 작으면 시작 안 함 (기본 %(default).0f)")
    parser.add_argument("--start-scale-range", type=float, nargs=2, default=list(START_SCALE_RANGE),
                        metavar=("LO", "HI"), help="시작 확인: hand 배율이 이 범위 밖이면 시작 안 함 (기본 0.50 0.70)")
    parser.add_argument("--yaw-in-limit-deg", type=float, nargs=2, default=None, metavar=("LEFT", "RIGHT"),
                        help="어깨 비틀기 안쪽 한계(도, e90 기준 안쪽 크기)를 위팔 든 각과 상관없이 이 값으로 (REPORT_48). "
                             "없으면 지금 규칙: 위팔 붙임 30°, 25° 이상 든 때 왼 54 / 오른 58°. 🔴 실측 닿음은 붙임 40/41°, 든 때 64/68°")
    parser.add_argument("--no-hand-refit", action="store_true",
                        help="hand: 어깨 옆이 몸 쪽 한계에 걸리면 어깨 비틀기·팔꿈치로 손목 목표에 다시 맞추는 것"
                             "(REPORT_47, 기본 켬)을 끈다")
    parser.add_argument("--no-flip-hold", action="store_true",
                        help="pos/hand: 사람 위팔이 등 뒤(어깨 높이 아래)일 때 '머리 위로 넘긴 해'로 한 번에 45° 넘게 "
                             "뛰지 않고 직전 자세를 유지하는 것(R1, REPORT_42, 기본 켬)을 끈다")
    parser.add_argument("--hand-touch-cm", type=float, default=PR.HAND_TOUCH_CM if PR else 25.0,
                        help="hand: 사람 두 손목 거리가 이 아래면 손이 닿은 것으로 보고 로봇은 최소 간격 (기본 %(default).0f, REPORT_39)")
    parser.add_argument("--hand-blend-cm", type=float, default=PR.HAND_BLEND_CM if PR else 5.0,
                        help="hand: 문턱 ~ 문턱+이만큼 사이는 비율로 섞는다 (기본 %(default).0f). --hand-touch-mode lateral 에서는 안 씀")
    parser.add_argument("--hand-touch-mode", choices=("3d", "lateral"), default="3d",
                        help="hand: 닿음 판정 거리. 3d = 두 손목 3차원 거리 (기본), lateral = 몸 틀 좌우 간격 (REPORT_38 방식)")
    parser.add_argument("--hand-min-gap-cm", type=float, default=PR.HAND_MIN_GAP_CM if PR else 14.0,
                        help="hand: 로봇 양 손목 관절 좌우 간격 최소 (기본 %(default).0f — 가슴 앞에서 두 아래팔 약 4 cm)")
    parser.add_argument("--hand-max-speed", type=float, default=PR.HAND_MAX_SPEED if PR else 1.0,
                        help="hand: 로봇 손목 목표 최대 속도 m/s (추적 튐 막기, 기본 %(default).1f)")
    parser.add_argument("--hand-swivel-tau", type=float, default=PR.HAND_SWIVEL_TAU if PR else 0.15,
                        help="hand: 팔꿈치 돌림이 사람 팔꿈치를 따라가는 시간 상수 초 (기본 %(default).2f)")
    parser.add_argument(
        "--robot-ref", default=None,
        help="로봇 기준값 파일 (arm_pose_logger.py --part R). 없으면 logs/robot_ref_*.json 중 최신")
    parser.add_argument(
        "--body-frame", choices=("chest_heading", "fixed"), default="chest_heading",
        help="pos 방식의 몸 틀. chest_heading = 가슴 본의 수직축 회전만 (기본, REPORT_22)")
    parser.add_argument(
        "--auto-session", choices=("live", "twist", "speed", "check", "yaw", "wrist"), default=None,
        help="자동 측정. 준비 카운트다운 → 중립 → POSITION → 구간 진행 → 방식 전환 → 한 번 더 → 종료")
    parser.add_argument(
        "--auto-start", action="store_true",
        help="자유 트래킹. 카운트다운 → 자동 중립 → 자동 POSITION 후 구간 없이 계속 따라간다 "
             "(종료는 h/x/Ctrl+C)")
    parser.add_argument("--auto-lead", type=float, default=None,
                        help="준비 시간 (live 40초, twist 25초, --auto-start 15초)")
    parser.add_argument("--auto-speed", type=float, default=1.0,
                        help="🔴 검증용. 구간 길이를 이 배수로 줄인다 (기본 1.0 = 실제 시간). dry-run 확인에만 쓴다")
    parser.add_argument("--auto-log", default=None, help="자동 측정 로그 경로 (--log-csv 대신 써도 된다)")
    parser.add_argument(
        "--cue-sound", default=None, metavar="IP:PORT",
        help="구간 소리를 낼 서버 PC (cue_player.py). 예: 192.168.0.65:5021")
    parser.add_argument("--sound-test", action="store_true", help="구간 소리만 시험하고 끝낸다")
    parser.add_argument("--sy-policy", choices=("hold", "none", "recenter", "bone"), default="hold",
                        help="팔을 편 상태의 어깨 비틀기 처리. hold=현행(유지), none=계산값, "
                             "recenter=기준 복귀, bone=뼈 긴축 회전으로 추정 (실행 중 y 로 전환)")
    parser.add_argument("--no-wrist-e90", action="store_true",
                        help="자동 시작의 '팔꿈치 90° 엄지 위' 손목 기준 단계를 끈다 (기본: 켬, REPORT_29 A)")
    parser.add_argument("--no-arm-collision", action="store_true",
                        help="양팔 충돌 방지 끄기 — 기본: 두 아래팔 겉면 거리가 --arm-gap-cm 보다 가까워지면 "
                             "가까워지게 하는 비틀기·어깨 옆 변화를 받지 않는다 (REPORT_36)")
    parser.add_argument("--arm-gap-cm", type=float, default=None,
                        help="양팔 충돌 방지 최소 거리 (cm). 기본: --hand-model cad 면 0, legacy 면 4. "
                             "음수 = 그만큼 겹침 허용 (모델 cad 는 실제 맞닿음을 −1.4~−3.3 cm 로 본다, REPORT_54)")
    parser.add_argument("--hand-model", choices=("cad", "legacy"), default="cad",
                        help="충돌 방지·두 팔 벌림이 쓰는 모델 손 길이 (REPORT_54 P1). cad = CAD 그리퍼 끝 (팔꿈치 축에서 25.6 cm, 기본), "
                             "legacy = 그리퍼 달기 전 URDF 손 (17.3 cm, 실제보다 약 8 cm 짧다)")
    parser.add_argument("--hand-pair", choices=("off", "on"), default="off",
                        help="hand: 양손이 가까울 때만 '잡는 점 쌍 매핑' (REPORT_56 G3) — 두 잡는 점의 가운데는 배율, 두 점 사이는 사람 그대로 1:1 "
                             "(물체 폭 보존, 그 동안 닿음 규칙 효과 없음). 기본 off = 예전과 완전히 같음")
    parser.add_argument("--mirror-limits", choices=("off", "on"), default="off",
                        help="관절 한계를 양팔 같게: 관절·방향마다 (좌우 거울 부호를 맞춘 뒤) 더 넓은 쪽으로 통일 "
                             "(비틀기 범위 규칙 포함). 기본 off = 예전과 완전히 같음. 켜면 시작 화면·_start.json 에 통일한 한계")
    parser.add_argument("--hand-pair-near-cm", type=float, default=PR.HAND_PAIR_NEAR_CM if PR else 25.0,
                        help="hand-pair: 사람 두 잡는 점 사이가 이 아래면 완전히 쌍 매핑 (기본 %(default).0f cm)")
    parser.add_argument("--hand-pair-far-cm", type=float, default=PR.HAND_PAIR_FAR_CM if PR else 33.0,
                        help="hand-pair: 이 위면 지금 방식 그대로, near~far 사이는 섞음 (기본 %(default).0f cm)")
    parser.add_argument("--grip-offset-cm", type=float, default=PR.HAND_GRIP_OFFSET_CM if PR else 15.0,
                        help="hand-pair: 사람 잡는 점 = 손목 + 이만큼 아래팔 방향 (컨트롤러 앞끝, 기본 %(default).0f cm, REPORT_56)")
    parser.add_argument("--hand-pair-center", choices=("grip", "robot-hand"), default="grip",
                        help="hand-pair: 가운데점의 팔마다 잡는 점 (작업 지시 10-c). grip = 사람 잡는 점(손목 + --grip-offset-cm) × 배율 (기본, 예전과 같음). "
                             "robot-hand = 로봇 손목 목표 + 13.3 cm × 사람 손 축 — 손 부분을 로봇 크기로 옮겨 쌍 매핑 중에도 끝까지 뻗고 가슴 앞이 덜 접힘")
    parser.add_argument("--hand-gap-goal-cm", type=float, default=None,
                        help="hand: 두 아래팔(모델) 사이가 이보다 가까우면 양손 목표를 좌우로 벌린다 (cm). "
                             "기본: cad 면 0.5, legacy 면 --arm-gap-cm + 1 (예전과 같음). 음수 = 겹침 허용")
    parser.add_argument("--no-wrist-hold", action="store_true",
                        help="손목 A 끄기 — 기본: 팔을 편 동안(팔꿈치 40° 미만) 손목 유지 (REPORT_32/33)")
    parser.add_argument("--no-overhead-elbow-hold", action="store_true",
                        help="머리 위 A 끄기 — 기본: 위팔이 110° 넘게 올라가면 팔꿈치를 붙잡고 140° 이상은 유지")
    parser.add_argument("--no-elbow-base", action="store_true",
                        help="머리 위 B 끄기 — 기본: 중립 때 잡은 사람 차렷 팔꿈치 값을 빼서 편 팔 = 로봇 0°")
    parser.add_argument("--wrist-median", action="store_true",
                        help="손목 엎침각 3프레임 중앙값 거르기를 켠다 (기본: 끔 — REPORT_30: 효과 없고 약 70ms 늦춘다)")
    parser.add_argument("--vel-ff", action="store_true",
                        help="속도 피드포워드 켬 (기본 끔). 명령의 변화율을 모터 목표 속도로 보낸다. 실행 중 f 로 전환")
    parser.add_argument("--vel-ff-alpha", type=float, default=0.5,
                        help="피드포워드 속도의 저역 통과 계수 (0~1, 클수록 빠르고 거칠다)")
    parser.add_argument("--cue-voice", action=argparse.BooleanOptionalAction, default=True,
                        help="음성 안내 (기본 켬, --no-cue-voice 로 끔). 서버 PC cue_player 가 다음 동작을 말한다")
    parser.add_argument("--cue-verbose", action="store_true",
                        help="소리 신호를 보낼 때마다 '보냈다' 를 찍는다 (신호가 나가는지 확인)")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--log-csv",
        type=str,
        default=None,
        help="Write command_target vs last_measured for every joint, ~20Hz, "
             "to this CSV path. Works in --dry-run too (measured columns "
             "stay 0.0 there since no real motor feedback exists).",
    )
    parser.add_argument("--log-rx", type=str, default=None, metavar="PATH.jsonl",
                        help="받은 원시 UDP 패킷(사람 뼈 전부)을 그대로 jsonl 로 저장 (REPORT_52, 기본 끔). 기록만 하고 "
                             "제어는 바꾸지 않는다. 줄 꼴은 rx_probe 녹화와 같고 t_mono(로그 CSV 의 t 와 같은 시계)·phase 가 더 있다")
    # VR 표시 (2026-10-21, 설계 REPORT_vr_reach_design §8 1·2단계) — 기본 끔. 끄면 import·계산·소켓 모두 없다
    parser.add_argument("--robot-viz", default=None, metavar="IP:PORT",
                        help="로봇 집게 위치·한계 상태를 서버 PC 언리얼(RobotVizProbe, UDP 5022)로 보낸다 (기본 끔). "
                             "예: 100.85.65.19:5022. 읽기만 하고 제어는 바꾸지 않는다. 보내기 실패는 무시")
    parser.add_argument("--robot-viz-hz", type=float, default=30.0,
                        help="--robot-viz 보내는 주기 (Hz, 기본 30)")
    parser.add_argument("--robot-viz-real-size", action="store_true",
                        help="--robot-viz: 언리얼이 손 길이를 실제 크기(13.3 cm)로 그리게 표시만 한다 (기본: 배율 크기 ÷k)")
    parser.add_argument("--robot-viz-jaw-sign", type=float, default=1.0,
                        help="--robot-viz: 집게 축을 손목 모터 (모터 − e90) 방향으로 돌리는 부호 (기본 1, 사진 확인 전 임시)")
    parser.add_argument("--robot-viz-jaw-offset-deg", type=float, default=0.0,
                        help="--robot-viz: 집게 축 보정각 (°, 기본 0 = e90 에서 손목 URDF 0 으로 봄)")
    # 작업 지시 10 (2026-10-30) — 모두 기본 끔. 끄면 명령·csv·패킷이 예전과 비트 단위로 같다
    parser.add_argument("--grip-virtual-timing", choices=("instant", "real"), default="instant",
                        help="작업 지시 14-j --grip-virtual 의 그리퍼 속도. instant = 판단이 바뀌면 바로 (기본, 지금 그대로), real = 아두이노 "
                             "(ardgripper_rt 14-i) 와 같은 시간 (한 번에 한 손, 닫는 쪽 먼저, 1°/10 ms, 닿으면 0.4 초). 가상 캔은 다 닫힌 뒤에만 잡고 "
                             "다 열린 뒤에만 놓는다. 다 닫힘·다 열림 때 --cue-sound 로 tick / switch 소리")
    parser.add_argument("--grip-virtual", action="store_true",
                        help="--gripper-port 가 없을 때 그리퍼 판단(GripDecider)을 직렬 없이 돌린다 (dry-run 시험·시연 기록용). "
                             "판단·부르는 시점은 실제 그리퍼와 같고, 로그 grip_L/R·pinch_L/R, robot-viz grip, 인계 조건에 쓴다. "
                             "--gripper-port 와 같이 주면 실제 그리퍼를 쓴다")
    parser.add_argument("--handover-exempt", action="store_true",
                        help="인계 예외: 쌍 매핑 w ≥ 0.5 이고 그리퍼 조건(--handover-when)이면 두 팔 충돌 판정(손 목표 벌림·모터 충돌 방지)에서 "
                             "손 끝 5.5 cm(집게)를 뺀다. 집게끼리는 --handover-finger-gap-cm 로 따로 막는다. 0.3초에 걸쳐 켜고 끈다")
    parser.add_argument("--handover-when", choices=HandoverGate.WHEN, default="any-closed",
                        help="--handover-exempt 켜짐 조건 (기본 any-closed = 그리퍼 하나 이상 닫힘, both-closed = 둘 다, pair = 그리퍼 안 봄)")
    # 작업 지시 11-b (2026-11-02) — 모두 기본 끔. 끄면 명령·csv·패킷이 예전과 비트 단위로 같다
    parser.add_argument("--table", default=None, metavar="PATH.json",
                        help="가대 면 제한 (예: ~/teleop-docs/_teleop/config/table.json): 손 목표를 판 위로 올리고, 모터 쪽에서 판 쪽으로 더 들어가는 변화를 받지 않는다. "
                             "로그 끝 열 table_clear_cm·table_blocks, robot-viz 비트 13·RVT1 패킷")
    parser.add_argument("--table-body", choices=("outer", "m2", "real"), default="outer",
                        help="--table 그리퍼 몸체 모델 (outer = 손목과 무관한 바깥 반지름 6.1, 기본. m2 = 평판이 위로 있다고 봄. "
                             "real = 평판을 손목 각에 따른 실제 방향으로 (작업 지시 12-c))")
    parser.add_argument("--table-margin-body-cm", type=float, default=None, help="--table 몸체·아래팔 여유 (cm, 기본 table.json 의 1.5)")
    parser.add_argument("--table-margin-tip-cm", type=float, default=None, help="--table 집게 끝 여유 (cm, 기본 table.json 의 0.5)")
    parser.add_argument("--start-pose", choices=("rest", "table"), default="rest",
                        help="시작 순서. table: 손목 기준 뒤 차렷으로 돌아가지 않고 팔꿈치 90° 를 유지한 채, 손 목표가 판 위 여유를 지키면 모터를 켠다. "
                             "로봇은 주차 자세에서 시작하고 켠 뒤 4초 동안 속도 제한을 0.2 배부터 올린다. --auto-start --retarget hand --table 필요")
    parser.add_argument("--heading-deadband-deg", type=float, default=None, metavar="도",
                        help="몸 방향 무감대 (작업 지시 12-b, 추천 20): 고개를 돌릴 때 몸 추적 방향이 따라 돌아 손 목표·판이 움직이는 것을 뺀다. "
                             "이 각도 안의 변화는 무시하고 --heading-tau-s 로 천천히 맞춘다. 표시(range_*)도 같은 값. --body-frame chest_heading 일 때만. "
                             "로그 끝 열 heading_raw_deg·heading_used_deg (기본: 끔)")
    parser.add_argument("--heading-tau-s", type=float, default=None, metavar="초",
                        help=f"--heading-deadband-deg 안에서 몸 방향을 맞추는 시간 상수 (초). 기본: --heading-soft-deg 가 있으면 {HEADING_TAU_SOFT_S:g}, 없으면 5")
    parser.add_argument("--heading-soft-deg", type=float, default=None, metavar="도",
                        help="몸 방향 부드러운 경계 (작업 지시 12-c): 무감대 ± 이 값에서 따라가는 비율을 0 → 1 로. 기본: 없음 (경계를 넘친 만큼 바로 따라감)")
    parser.add_argument("--virtual-can", default=None, metavar="자리",
                        help="VR 가상 캔 (작업 지시 13, 기본 끔): 시작 자리 (L1~L4, R1~R4, H1, H2). --table 과 --grip-virtual 또는 --gripper-port 가 필요. "
                             "로그 끝 열 can_x/y/z·can_qx/qy/qz/qw·can_state·episode, --robot-viz 가 있으면 RVC1 패킷")
    parser.add_argument("--virtual-can-goal", default=None, metavar="자리",
                        help="캔이 이 자리 허용 범위 (--virtual-can-goal-tol-cm, 기본 5 cm) 안에 서고 두 팔이 다 놓으면 에피소드 자동 성공 (--episodes 와 함께)")
    parser.add_argument("--virtual-can-goal-tol-cm", type=float, default=None, metavar="cm",
                        help="목표 자리 허용 범위 (캔 밑면과 목표의 수평 거리, cm, 기본 5, tools/virtual_can.py GOAL_TOL_CM)")
    parser.add_argument("--virtual-can-margin-cm", type=float, default=None, metavar="cm",
                        help="가상 캔 잡힘 판정 여유 (cm, 기본 1.0, tools/virtual_can.py GRIP_MARGIN_CM)")
    parser.add_argument("--virtual-can-place-cm", type=float, default=None, metavar="cm",
                        help="가상 캔을 놓을 때 밑면이 판 위 이 높이 이하면 선다 (cm, 기본 4, 기울기 20°·속도 0.3 m/s 도 봄, tools/virtual_can.py PLACE_*)")
    parser.add_argument("--virtual-can-handover-extra-cm", type=float, default=None, metavar="cm",
                        help="인계 때 둘째 손의 잡는 높이 범위를 양쪽으로 넓히는 양 (cm, 기본 0.85 = 집게 반폭, tools/virtual_can.py HANDOVER_HEIGHT_EXTRA_CM)")
    # 작업 지시 14-l (기본 끔): 가상 캔 잡기 정렬 조건. 노란 테두리 (지금 닫으면 잡힘) 도 같은 조건
    parser.add_argument("--grasp-align", action="store_true",
                        help="가상 캔 잡기에 정렬 조건을 더한다 (14-s): 먼저 잡는 손은 집게 축 ⟂ 캔 축 어긋남 ≤ --grasp-align-deg 첫째 (숙임 = 셋째 축 ↔ 캔은 45° 까지 허용), "
                             "둘째 손은 셋째 축 ↔ 쥔 캔의 축 ≤ 둘째 각 + 집게 가운데 ↔ 캔 축 ≤ --grasp-align-axis-cm. 둘째 손 집게가 쥔 캔을 밀면 화면·소리로 알림. "
                             "노란 테두리도 이 조건까지 맞을 때만. 못 잡으면 각·거리를 화면에 (--virtual-can 과 함께)")
    parser.add_argument("--grasp-align-deg", type=float, nargs=2, default=None, metavar=("먼저", "둘째"),
                        help="정렬 각 한도 (도): 먼저 잡는 손, 둘째 손 (기본 15 10, tools/virtual_can.py ALIGN_DEG_*)")
    parser.add_argument("--grasp-align-axis-cm", type=float, default=None, metavar="cm",
                        help="둘째 손 집게 가운데 ↔ 캔 축 한도 (cm, 기본 2.0 — 14-s 실측 T4 1.7 cm, tools/virtual_can.py ALIGN_AXIS_CM)")
    # 작업 지시 16-a (기본 끔): 조합 순환 시연, 엄격 판정
    parser.add_argument("--can-combos", default=None, metavar="시작:목표,…",
                        help="가상 캔 조합 순환 (16-a, 예 L1:R1,L1:R2,L2:R1,L2:R2): 에피소드가 끝날 때마다 다음 조합으로 (차례로), 새 시작 자리에 캔을 세운다. "
                             "VR 은 목표 원·캔 위치로, 소리는 ready (세 번 삐삐삐). 에피소드 기록에 자리 좌표·조합별 수 (--virtual-can·--episodes 와 함께, --virtual-can 자리 대신 첫 조합)")
    parser.add_argument("--can-pairs", default=None, metavar="파일",
                        help="가상 캔 좌표 쌍 순환 (19-b, 기본 끔): 한 줄에 '시작 x, y, 도착 x, y' (cm, base). 자리 목록에 움직이는 자리 S0 · G0 를 실행 때 더하고 "
                             "에피소드가 끝날 때마다 다음 쌍의 좌표로 갈아 끼운다 (차례로, 소리 ready). 기록은 --can-combos 와 같은 칸 (시작 · 도착 xy). "
                             "--episodes 와 함께, --can-combos 와는 같이 쓰지 않는다. --virtual-can · --virtual-can-goal 은 안 줘도 된다 (S0 · G0)")
    parser.add_argument("--grasp-strict", action="store_true",
                        help="가상 캔 엄격 판정 (16-a, tools/virtual_can.py STRICT_*): 닫기 직전 열린 날·몸체가 캔에 들어감·집게 가운데가 캔 축에서 덜 들어감/비킴 (실패), "
                             "두 그리퍼 간격·놓은 자리 − 목표 (경고). 화면·소리, 에피소드 기록 strict_* (결과는 그대로) (--virtual-can 과 함께)")
    # 작업 지시 19-x (기본 끔): 위쪽 잡기 · 실물 기준 판정 · 시작 자세 (tools/virtual_can.py GRASP_MODES · REAL_* · START_GATE_*)
    parser.add_argument("--grasp-mode", choices=("bottom", "top"), default="bottom",
                        help="가상 캔 잡기 모드 (19-x, 기본 bottom = 지금). top: L 이 서 있는 캔 위쪽 (캔 밑면에서 10 ~ 14 cm) 을 먼저 잡고 R 이 아래쪽 (3 ~ 5 cm) 으로 받는다. "
                             "VR 띠: 서 있는 캔 12 cm, 시작 · 목표 자리 L 10 · 14 / R 3 · 5 (--virtual-can 과 함께)")
    parser.add_argument("--grasp-top-band", action="store_true",
                        help="19-x3 (기본 끔): --grasp-mode top 에서 VR 잡는 높이 띠를 바꾼다 (판 머리 12 cm, 자리 L 10 · 14 / R 3 · 5). "
                             "🔴 지금 언리얼에서는 목표 원이 판 속에 묻힌다 (REPORT_top19x3) — 언리얼을 고친 뒤에만 켠다")
    parser.add_argument("--grasp-real", action="store_true",
                        help="가상 캔 실물 기준 판정 (19-x, 수집 때 켬): 인계 깊이 (19-h 가정 B, 논리) ≥ --grasp-real-depth-cm, 잡기 비킴 ≤ 1 cm, "
                             "놓을 때 판 면 누름 ≤ 0.5 cm, 잡기 전 부딪힘 없음. 어기면 까닭별 음성 (인계 얕음 · 비킴 · 누름 · 부딪힘), 에피소드 기록 real_* (결과는 그대로)")
    parser.add_argument("--grasp-real-depth-cm", type=float, default=None, metavar="CM",
                        help="실물 기준 인계 깊이 하한 (논리, 기본 1.70 = 엔코더 상당 3.0 − Δ 1.30, 19-j)")
    parser.add_argument("--start-gate", action="store_true",
                        help="시작 자세 (19-x): 두 그리퍼를 닫고 두 집게 끝을 판 위 --start-gate-h-cm 이상 · 캔에서 떨어뜨려 0.5 초 두어야 에피소드가 시작된다 (--episodes 와 함께)")
    parser.add_argument("--start-gate-h-cm", type=float, default=None, metavar="CM", help="시작 자세 높이 (기본 12)")
    # 작업 지시 16-d (기본 끔): VR 그림 ↔ 판정 맞추기 (REPORT_yellow_20261113_2036.md 안 B)
    parser.add_argument("--viz-pair-after-grasp", action="store_true",
                        help="VR 표시 (16-d): 가상 캔을 쥔 손이 없을 때 (첫 잡기 전) 는 robot-viz 가 보내는 쌍 매핑 비율 pair_w 를 0 으로 (그림만, 0.3 초에 걸쳐 바꿈). "
                             "로봇 명령 쪽 쌍 매핑은 그대로. 쥔 뒤·인계 중은 지금처럼 (--virtual-can·--robot-viz 와 함께)")
    parser.add_argument("--replay", default=None, nargs="+", metavar="에피소드.npz",
                        help="녹화 재생 (작업 지시 14 C, tools/demo_replay.py): demo_export.py 의 에피소드 관절 궤적을 사람 입력 대신 목표로 쓴다. "
                             "지금 전원 robot_ref 로 모터 값을 다시 계산하고, 관절 한계·속도·충돌·가대 면·발산 감시를 그대로 거친다. "
                             "p = 지금 자세에서 첫 자세로 천천히, g = 재생, h = 멈춤 (g 로 이어서), x / Ctrl+C 그대로. 추적 패킷은 명령에 안 쓴다. "
                             "파일 여러 개는 --replay-auto 에서만 (차례로 재생)")
    parser.add_argument("--replay-speed", type=float, default=1.0, metavar="배율",
                        help="--replay 재생 속도 배율 (기본 1.0, 처음엔 0.5 권장)")
    parser.add_argument("--replay-move-s", type=float, default=5.0, metavar="초",
                        help="--replay 지금 자세 → 첫 자세로 옮기는 시간 (기본 5, 4 이상. --replay-auto 는 5 이상, 주차 복귀도 이 시간)")
    parser.add_argument("--replay-auto", action="store_true",
                        help="작업 지시 14-b: 키 없이 끝까지 (tools/auto_target.py). 삐 + 5 초 → 지금 자세에서 모터 켬 → 첫 자세로 → 캔 놓을 시간 "
                             "→ 재생 → 끝 자세 2 초 → 주차로 → 감쇠. 걸림·발산·위치 모드 풀림·h 는 비상 멈춤 (종료 코드 3), 다 끝나면 0")
    parser.add_argument("--replay-wait-s", type=float, default=15.0, metavar="초",
                        help="--replay-auto 캔 놓을 시간 (기본 15, 5 이상. 마지막 5 초는 1 초마다 삐). 첫 회차는 첫 자세에서, 다음 회차는 주차 자세에서")
    parser.add_argument("--replay-repeat", type=int, default=1, metavar="N",
                        help="--replay-auto 에피소드마다 N 번 (기본 1). 파일 여러 개면 파일마다 N 번씩 차례로")
    parser.add_argument("--stall-stop", action="store_true",
                        help="작업 지시 14-b 걸림 감시: 실제 − 명령 관절각 차이가 문턱 (관절별, tools/auto_target.py STALL_MOVE·STALL_STILL) 을 넘은 채 "
                             "이어지면 실제 자세에 멈춤 → 삐 → 0.5 초 뒤 감쇠 → 종료 (코드 3). 재생·정책 실행에서 켜기 권장. dry-run 은 실제 각이 없어 돌지 않는다")
    parser.add_argument("--auto-end", action="store_true",
                        help="작업 지시 14-b VR 실기동 종료 동작 (--start-pose table 과 함께): 로봇 두 집게 끝을 처음 (모터 켠) 자리 8 cm 안으로 내리고 "
                             "3 초 가만히 (그리퍼는 열림) → 삐 → 주차로 4 초 → 감쇠 → 정상 종료 (코드 0). 13 cm 넘게 나가 있던 시간이 합쳐 10 초 된 뒤부터 본다")
    parser.add_argument("--park-table-relax", action="store_true",
                        help="작업 지시 14-m (기본 끔): --auto-end·--vr-loss-park 주차의 뒤 절반에서만 판 면 제한을 주차 자세 (모터를 켤 때 자세 — 판에 얹힘) 의 "
                             "판 거리 (−0.2 cm 더) 까지 허용한다. 끄면 예전처럼 판 위 0.5 cm 에서 막혀 남은 차를 둔 채 감쇠한다")
    parser.add_argument("--vr-loss-park", action="store_true",
                        help="작업 지시 14-c 원격 조종 VR 끊김 (패킷 안 옴·active false·값 그대로·머리 벗어남 = 헤드셋 벗음): 바로 그 자리 유지 + 삐 → "
                             "5 초 안에 돌아오면 이어서, 넘으면 사람 입력을 끊고 주차로 4 초 → 감쇠 → 종료 (코드 3). 0.5 초 바로 감쇠·자동 흐름 5 초 중단 대신")
    parser.add_argument("--vr-guard", action="store_true",
                        help="작업 지시 14-c --replay-auto 에 VR 감시: VR 이 3 초 이어서 들어와야 시작 (그전엔 모터를 안 켠다), 어디서든 끊기면 "
                             "유지 → 5 초 안에 오면 이어서, 넘으면 주차 → 감쇠 → 종료 (코드 3). VR 패킷은 명령에 쓰지 않는다")
    parser.add_argument("--beep-local", action=argparse.BooleanOptionalAction, default=True,
                        help="--replay-auto·--stall-stop·--auto-end 의 로봇 PC 삐 소리 (기본 켬, 장치는 PULSE_SINK 환경 변수). --cue-sound 가 있으면 서버 PC 소리도 같이")
    parser.add_argument("--episodes", action="store_true",
                        help="에피소드 기록 (--virtual-can·--log-csv 와 함께). 키보드 없이 돈다: 위치 모드가 되면 시작, 성공·넘어짐·시간 초과 뒤 캔을 다시 세우고 새 에피소드. "
                             "키 n 새 에피소드, s 성공, f 실패도 쓸 수 있다 (이때 f 의 속도 피드포워드 전환은 꺼진다). 파일 <log 이름>_episodes.csv")
    parser.add_argument("--episode-end", choices=("place", "release"), default="place",
                        help="작업 지시 14-h --episodes 성공 정의: place = 놓아서 목표에 선 순간 (기본, 13-d 그대로), "
                             "release = 그 뒤 두 그리퍼가 열리고 두 집게 끝이 캔에서 3 cm 넘게 0.5 초 (손 빼기까지 기록). 쥔 채 목표 안은 '놓기 대기'")
    parser.add_argument("--episode-reset-s", type=float, default=None, metavar="초",
                        help="--episodes: 에피소드가 끝나고 캔을 다시 세우기까지 (초, 기본 3)")
    parser.add_argument("--episode-timeout-s", type=float, default=None, metavar="초",
                        help="--episodes: 이 시간을 넘기면 실패 (timeout) 로 끝낸다 (초, 기본 90, 0 이면 끔)")
    parser.add_argument("--handover-finger-gap-cm", type=float, default=HANDOVER_FINGER_GAP_CM,
                        help=f"--handover-exempt 중 집게끼리 거리 문턱 (한 팔 집게 끝 → 다른 팔 집게 중심선, cm, 기본 {HANDOVER_FINGER_GAP_CM})")
    args = parser.parse_args()
    if args.heading_tau_s is None:
        args.heading_tau_s = HEADING_TAU_SOFT_S if args.heading_soft_deg is not None else 5.0
    return args


def print_header(
    args: argparse.Namespace,
    sides: Sequence[str],
    channels: Dict[str, str],
    motor_ids: Dict[str, Sequence[int]],
    limits: Dict[str, Dict[str, Tuple[float, float]]],
) -> None:
    print("=" * 84)
    print("test1 -> Berkeley Humanoid Lite dual-arm controller (+ divergence watchdog)")
    print(f"Unreal target : 192.168.0.65:{args.udp_port}")
    print(f"UDP bind      : {args.udp_ip}:{args.udp_port}")
    print(f"Mode          : {'DRY-RUN' if args.dry_run else 'REAL'}")
    print(f"Arms          : {', '.join(sides)}")
    print(f"Divergence    : {DEFAULT_DIVERGENCE_THRESHOLD_RAD:.3f} rad (code constant, not a CLI flag)")
    print(f"EMA alpha     : {args.ema_alpha:.2f} (--ema-alpha; 1.0 = off)")
    # 🔴 상태를 반드시 눈에 보이게 찍는다. suppress_minor_swing_axis 는 CLI 기본값이 0.0 이라
    # 한 번도 동작한 적이 없었는데 아무도 몰랐다 (REPORT_07 §3.3). 같은 일을 되풀이하지 않는다.
    print(f"Chest relative: {'ON' if args.chest_relative else 'OFF'} "
          f"(--no-chest-relative 로 끄고, 실행 중 'b' 키로 전환)")
    print(f"Calibrate delay: {args.calibrate_delay:.1f}s (--calibrate-delay; time to get into pose after 'c')")
    print(
        f"Swing suppress: base_exponent={args.swing_suppression_exponent} "
        f"taper=[{args.swing_taper_start:.2f},{args.swing_taper_end:.2f}]rad "
        f"(full strength below taper_start, 0 at/above taper_end)"
    )
    for side in sides:
        print(f"[{side.upper()}] channel={channels[side]}")
        for joint, device_id in zip(JOINT_ORDER, motor_ids[side]):
            low, high = limits[side][joint]
            source, axis, sign, scale, motion = RETARGET_CONFIG[side][joint]
            print(
                f"  {joint:14s} motor={device_id:2d} "
                f"URDF=[{low:+.3f},{high:+.3f}] "
                f"VR={source}.{axis} sign={sign:+.0f} scale={scale:.2f} delta=+/-{motion:.2f}"
            )
    print(f"Retarget      : {args.retarget.upper()} "
          f"({'관절 위치 기반, 몸 틀=' + args.body_frame if args.retarget == 'pos' else '손 위치 기준 (REPORT_38), 몸 틀=' + args.body_frame if args.retarget == 'hand' else '기존 뼈 회전'}) "
          "— 실행 중 'v' 로 전환")
    if args.retarget == "hand":
        print(f"손 위치 (38/39)   : 닿음 = 두 손목 {'3차원 거리' if args.hand_touch_mode == '3d' else '좌우 간격'} "
              f"≤ {args.hand_touch_cm:.0f} cm"
              + (f" (섞기 ~{args.hand_touch_cm + args.hand_blend_cm:.0f} cm)" if args.hand_touch_mode == "3d" else " (섞기 ~어깨 폭)")
              + f" → 로봇 최소 간격 {args.hand_min_gap_cm:.0f} cm, "
              f"손목 목표 속도 ≤ {args.hand_max_speed:.1f} m/s, 팔꿈치 돌림 {args.hand_swivel_tau:.2f} s, "
              f"두 아래팔(모델 손 {args.hand_model}) {args.hand_gap_goal_cm:+.1f} cm 안 되면 손 목표를 벌림, "
              "차렷 팔꿈치 빼기 대신 차렷 어깨→손목 거리로 배율")
        print(f"양손 쌍 매핑 (56)  : " + ("끔 (--hand-pair off)" if args.hand_pair != "on" else
              f"🔵 켬 — 사람 두 잡는 점(손목 + {args.grip_offset_cm:.0f} cm) 사이 ≤ {args.hand_pair_near_cm:.0f} cm 면 가운데는 배율·사이는 1:1, "
              f"≥ {args.hand_pair_far_cm:.0f} cm 면 지금 방식, 사이는 섞음 (로그 hand_pair_w)"))
        if args.hand_pair == "on" and args.hand_pair_center == "robot-hand":
            print("  🔵 가운데점 robot-hand — 팔마다 로봇 손목 목표 + 13.3 cm × 사람 손 축 (작업 지시 10-c, 기본 grip = 사람 잡는 점 × 배율)")
        print(f"손 다시 맞춤 (47)  : {'켬' if not args.no_hand_refit else '끔'} "
              "(어깨 옆이 몸 쪽 한계에 걸리면 비틀기·팔꿈치로 손목 목표에 맞춤, 새 프레임마다 계산)")
    print(f"팔 편 상태 비틀기 : {args.sy_policy}  (y 로 hold→none→recenter→bone 순환)")
    print(f"목표 속도 제한    : {args.max_speed:.1f} rad/s  (m 으로 2.5→3.5→4.5 순환)")
    print(f"속도 피드포워드   : {'켬' if args.vel_ff else '끔'}  (f 로 전환, 저역 통과 {args.vel_ff_alpha:.2f})")
    print(f"손목 기준 / 거르기 : 자동 시작 때 팔꿈치 90° 엄지 위 {'켬' if not args.no_wrist_e90 else '끔'}, "
          f"3프레임 중앙값 {'켬' if args.wrist_median else '끔'}")
    print(f"비틀기·충돌 (36)  : 안쪽 한계 "
          + (f"🔴 직접 지정 왼 {PR.YAW_IN_OVERRIDE_DEG['left']:.0f}° / 오른 {PR.YAW_IN_OVERRIDE_DEG['right']:.0f}° (위팔 든 각 무관, REPORT_48)"
             if args.yaw_in_limit_deg is not None else
             f"위팔 든 각 0°→30°, 25°↑→왼{PR.YAW_IN_RAISED_DEG['left']:.0f}/오른{PR.YAW_IN_RAISED_DEG['right']:.0f}°")
          + ", 바깥 54° · "
          f"양팔 충돌 방지 {'끔' if args.no_arm_collision else f'켬 ({args.arm_gap_cm:+.1f} cm, 모델 손 {args.hand_model} — 손끝까지 팔꿈치 축에서 {(0.123 + (PR.HAND_TIP_M if PR else 0.05)) * 100:.1f} cm, 비틀기·어깨 옆 → 그래도 가까워지면 어깨 앞뒤·팔꿈치도)'}")
    print(f"입력 거르기 (33)  : 손목 A {'켬' if not args.no_wrist_hold else '끔'}, "
          f"머리 위 팔꿈치 유지 {'켬' if not args.no_overhead_elbow_hold else '끔'}, "
          f"차렷 팔꿈치 빼기 {'켬' if not args.no_elbow_base else '끔'}")
    mi = getattr(args, "mirror_info", {"on": False})
    print("관절 한계 좌우     : " + ("같지 않음 (--mirror-limits off, 팔마다 실측 범위)" if not mi.get("on") else
          "🔴 통일 (--mirror-limits on) — 관절·방향마다 넓은 쪽. 넓힌 칸: "
          + (", ".join(f"{'왼' if w['side'] == 'left' else '오른'} {w['joint']} {w['end']} +{w['wider_deg']:.1f}°"
                       for w in mi.get("widened", [])) or "없음")))
    if mi.get("rel_deg"):
        for side in ("left", "right"):
            print(f"  {side:5s} 차렷 기준 (°): " + "  ".join(f"{j} [{a:+.1f},{b:+.1f}]" for j, (a, b) in mi["rel_deg"][side].items()))
    print(f"등 뒤 넘김 막기 (42): {'켬' if not args.no_flip_hold else '끔'} "
          f"(위팔 어깨 높이 아래 + 머리 위로 넘긴 해로 {PR.FLIP_HOLD_JUMP_DEG if PR else 45:.0f}° 넘게 뜀 → 직전 자세 유지)")
    if args.robot_viz:
        print(f"VR 표시 (robot-viz): 🔵 {args.robot_viz} 로 {args.robot_viz_hz:.0f} Hz — 집게 위치·한계 색 "
              f"(손 길이 {'실제 크기' if args.robot_viz_real_size else '배율 크기 ÷k'}, "
              f"집게 축 부호 {args.robot_viz_jaw_sign:+.0f} 보정 {args.robot_viz_jaw_offset_deg:+.1f}°)")
    if args.grip_virtual:
        print("가상 그리퍼: " + ("🔵 켬 — 직렬 없이 손마다 열림/닫힘 판단 (로그 grip_L/R·pinch_L/R, robot-viz grip, 인계 조건)"
                                if not args.gripper_port else "--gripper-port 가 있어 실제 그리퍼 판단을 쓴다 (가상 그리퍼 안 씀)"))
    if args.table:
        print(f"가대 면 제한: 🔵 켬 — {args.table} (몸체 {args.table_body}). 판 위 여유 아래로 안 내려간다 (로그 table_clear_cm·table_blocks)")
    if args.heading_deadband_deg is not None:
        print(f"몸 방향 무감대: 🔵 켬 — {args.heading_deadband_deg:g}° 안의 변화는 무시, 시간 상수 {args.heading_tau_s:g} 초"
              + (f", 부드러운 경계 ±{args.heading_soft_deg:g}°" if args.heading_soft_deg is not None else "") + " (로그 heading_raw_deg·heading_used_deg)")
    if args.start_pose == "table":
        print("시작 순서: 🔵 table — 팔꿈치 90° 를 유지한 채 모터를 켠다 (주차 자세에서 시작, 4초 동안 천천히)")
    if args.handover_exempt:
        print(f"인계 예외: 🔵 켬 — 쌍 매핑 w ≥ 0.5 + {args.handover_when} 이면 충돌 판정에서 손 끝 5.5 cm 제외 (0.3초에 걸쳐), "
              f"집게끼리 모터 {args.handover_finger_gap_cm:.1f} / 손 목표 {args.handover_finger_gap_cm + HANDOVER_FINGER_CUSHION_CM:.1f} cm "
              "(로그 handover_on·arm_gap_full_cm·arm_gap_finger_cm)")
        if args.handover_when != "pair" and not (args.gripper_port or args.grip_virtual):
            print("  ⚠ 그리퍼가 없어 그리퍼 조건이 켜지지 않는다 — --grip-virtual 이나 --handover-when pair 를 쓴다")
        if args.hand_pair != "on":
            print("  ⚠ --hand-pair on 이 아니라 쌍 매핑 w 가 없어 켜지지 않는다")
        if args.no_arm_collision:
            print("  ⚠ --no-arm-collision 이라 충돌 방지가 없어 예외도 쓰지 않는다")
    print("Keys: c=VR neutral, p=POSITION, h=hold, x=DAMPING, v=pos/bone 전환, y=비틀기 방식, "
          "m=속도 제한, f=속도 피드포워드, b=chest-relative on/off, Ctrl+C=quit")
    print("=" * 84)


def main() -> None:
    args = parse_args()
    # REPORT_54 P1: 모델 손 길이와, 그에 맞춘 충돌 방지·두 팔 벌림 기본값 (legacy = 예전 그대로)
    if PR is not None:
        PR.set_hand_model(args.hand_model)
    if args.arm_gap_cm is None:
        args.arm_gap_cm = 0.0 if args.hand_model == "cad" else 4.0
    if args.hand_gap_goal_cm is None:
        args.hand_gap_goal_cm = 0.5 if args.hand_model == "cad" else args.arm_gap_cm + 1.0
    if args.yaw_in_limit_deg is not None:                       # REPORT_48 — 관절 한계 계산 전에
        apply_yaw_in_limit(*args.yaw_in_limit_deg)
    sides = selected_sides(args.side)
    channels = {
        "left": args.left_channel,
        "right": args.right_channel,
    }
    motor_ids = {
        "left": args.left_ids,
        "right": args.right_ids,
    }
    if args.sound_test:
        cue = CueSound(args.cue_sound, verbose=True, voice=args.cue_voice)
        if cue.sock is None:
            print("[소리] --cue-sound IP:PORT 를 주어야 한다 (서버 PC 의 cue_player.py)")
            raise SystemExit(2)
        print(f"[소리] {args.cue_sound} 로 신호를 보낸다 — 서버 PC 에서 소리가 나는지 확인하라")
        print("       서버 PC 의 cue_player.py 화면에 줄이 찍히는지 같이 보라:")
        print("         찍힌다 + 소리 없음 → 재생 장치 문제 (cue_player --list-devices / --device)")
        print("         안 찍힌다        → 신호가 못 간다 (IP·포트·방화벽)")
        for name in ("ready", "go", "learn", "twist", "stop", "tick", "switch", "done", "alarm"):
            cue.send(name, text=f"소리 시험: {name}")
            time.sleep(1.0)
        if args.cue_voice:
            print("[소리] 음성 시험 — 서버 PC 에서 한국어로 말해야 한다 (말하는 중 높은음도 같이)")
            for text in ("소리 시험입니다", "왼팔. 옆으로 만세", "끝났습니다. 힘이 빠집니다"):
                cue.say(text)
                time.sleep(2.5)
            cue.say("오른팔. 앞으로 만세")
            time.sleep(0.5)
            cue.send("go", text="음성 중 삑")
            time.sleep(2.0)
        print(f"[소리] 보낸 신호 {cue.sent}개" + ("" if cue.sent else " — 하나도 못 보냈다"))
        raise SystemExit(0)

    limits = load_urdf_limits(args.urdf)

    # 🔴 로봇 기준값 파일이 없으면 시작하지 않는다 — 영점이 전원마다 24° 씩 뛴다 (REPORT_18 §10.3).
    ref_path = newest_robot_ref(args.robot_ref)
    if ref_path is None:
        print("[STOP] 로봇 기준값 파일이 없다 (logs/robot_ref_*.json).")
        print("       로봇을 켠 뒤 먼저 실행하라:")
        print("         python3 ~/teleop-docs/_teleop/tools/arm_pose_logger.py --part R")
        raise SystemExit(2)
    if PR is None:
        print(f"[STOP] pos_retarget 를 불러올 수 없다: {_POS_IMPORT_ERROR}")
        raise SystemExit(2)
    robot_ref = PR.RobotRef.load(ref_path)
    limits = limits_from_robot_ref(robot_ref, sides)
    args.mirror_info = {"on": False}
    if args.mirror_limits == "on":                              # 2026-10-14 — 비틀기 규칙까지, 러너·모터를 만들기 전에
        limits, args.mirror_info = mirror_limits(limits, robot_ref, sides)
    print(f"[REF] 로봇 기준값: {ref_path.name}")
    print("[LIMIT] REPORT_18 안전 범위(여유 5%)를 차렷 기준값에 붙여 관절 한계로 쓴다")

    print_header(args, sides, channels, motor_ids, limits)

    state = {
        "quit": False,
        "calibrate_requested_at": None,
        "calibrate_delay": args.calibrate_delay,
        "position": False,
        "hold_request": False,
        "damping": False,
        "hold": True,
        "toggle_chest": False,
        "toggle_retarget": False,
        "retarget": args.retarget,
        "cycle_sy_policy": False,
        "cycle_speed": False,
        "toggle_vel_ff": False,
        "start_block": "",          # REPORT_45 §4: 수동 c 뒤 시작 확인 실패 이유 (있으면 p 를 막는다)
    }
    warned_no_chest = False

    if args.auto_log:
        args.log_csv = args.auto_log
    if args.heading_deadband_deg is not None and (args.heading_deadband_deg <= 0 or args.heading_tau_s <= 0 or args.body_frame != "chest_heading"):
        raise SystemExit("--heading-deadband-deg 는 0 보다 큰 값, --heading-tau-s 도 0 보다 큰 값, --body-frame chest_heading 이어야 한다")
    if args.heading_soft_deg is not None and (args.heading_deadband_deg is None or not 0 < args.heading_soft_deg <= args.heading_deadband_deg):
        raise SystemExit("--heading-soft-deg 는 --heading-deadband-deg 와 함께, 0 보다 크고 무감대 이하로 준다")
    if args.replay and (args.auto_start or args.virtual_can or args.episodes or args.auto_session):
        print("[STOP] --replay 는 --auto-start·--auto-session·--virtual-can·--episodes 와 같이 쓰지 않는다 (재생은 사람 입력·가상 캔 판정을 쓰지 않는다)")
        raise SystemExit(2)
    if args.replay and (args.replay_move_s < 4.0 or not 0.05 <= args.replay_speed <= 2.0):
        print("[STOP] --replay-move-s 는 4 이상, --replay-speed 는 0.05~2.0")
        raise SystemExit(2)
    if args.replay and len(args.replay) > 1 and not args.replay_auto:     # 작업 지시 14-b
        print("[STOP] --replay 파일 여러 개는 --replay-auto 와 같이 쓴다 (수동 p/g 는 한 파일)")
        raise SystemExit(2)
    if args.replay_auto and (not args.replay or args.replay_move_s < 5.0 or args.replay_wait_s < 5.0 or args.replay_repeat < 1):
        print("[STOP] --replay-auto 는 --replay 와 같이, --replay-move-s 5 이상, --replay-wait-s 5 이상, --replay-repeat 1 이상")
        raise SystemExit(2)
    if args.vr_loss_park and args.replay:                               # 작업 지시 14-c
        print("[STOP] --vr-loss-park 는 원격 조종용이다. 재생은 --replay-auto --vr-guard 를 쓴다")
        raise SystemExit(2)
    if args.vr_guard and not args.replay_auto:
        print("[STOP] --vr-guard 는 --replay-auto 와 같이 쓴다")
        raise SystemExit(2)
    if args.park_table_relax and not (args.auto_end or args.vr_loss_park):
        print("[STOP] --park-table-relax 는 --auto-end 또는 --vr-loss-park 와 같이 쓴다")
        raise SystemExit(2)
    if args.auto_end and (args.replay or args.start_pose != "table"):
        print("[STOP] --auto-end 는 --start-pose table (--auto-start --retarget hand --table) 과 같이 쓴다 (주차 = 판 위에서 모터를 켠 자세). --replay 와는 쓰지 않는다")
        raise SystemExit(2)
    if args.can_pairs:                                                          # 19-b: 시작 · 목표 자리는 움직이는 자리 S0 · G0
        if args.can_combos or not args.episodes:
            print("[STOP] --can-pairs 는 --episodes 와 같이 쓰고, --can-combos 와는 같이 쓰지 않는다")
            raise SystemExit(2)
        args.virtual_can, args.virtual_can_goal = "S0", "G0"
    if args.virtual_can and not args.table:
        print("[STOP] --virtual-can 은 --table 과 같이 쓴다 (자리·판 높이를 거기서 읽는다)")
        raise SystemExit(2)
    if args.grip_virtual_timing != "instant" and not (args.grip_virtual and not args.gripper_port):     # 작업 지시 14-j
        print("[STOP] --grip-virtual-timing real 은 --grip-virtual 과 같이 (--gripper-port 없이) 쓴다 — 실제 그리퍼는 보드가 시간을 정한다")
        raise SystemExit(2)
    if args.virtual_can and not (args.grip_virtual or args.gripper_port):
        print("[STOP] --virtual-can 은 그리퍼 판단이 있어야 한다: --grip-virtual 또는 --gripper-port")
        raise SystemExit(2)
    if args.grip_open_rule == "finger" and (args.grip_mid_open_s <= 0 or args.grip_handover_guard_s < 0):     # 작업 지시 14-p
        print("[STOP] --grip-mid-open-s 는 0 보다 크고 --grip-handover-guard-s 는 0 이상이어야 한다")
        raise SystemExit(2)
    if args.episodes and not (args.virtual_can and args.log_csv):
        print("[STOP] --episodes 는 --virtual-can 과 --log-csv 와 같이 쓴다")
        raise SystemExit(2)
    if args.episode_end != "place" and not args.episodes:                # 작업 지시 14-h
        print("[STOP] --episode-end release 는 --episodes 와 같이 쓴다")
        raise SystemExit(2)
    if (args.episode_reset_s is not None or args.episode_timeout_s is not None) and not args.episodes:
        print("[STOP] --episode-reset-s·--episode-timeout-s 는 --episodes 와 같이 쓴다")
        raise SystemExit(2)
    if args.virtual_can_goal_tol_cm is not None and not args.virtual_can_goal:
        print("[STOP] --virtual-can-goal-tol-cm 는 --virtual-can-goal 과 같이 쓴다")
        raise SystemExit(2)
    if args.virtual_can_goal and not args.episodes:
        print("[STOP] --virtual-can-goal 은 --episodes 와 같이 쓴다")
        raise SystemExit(2)
    if args.can_combos and not (args.virtual_can and args.episodes):          # 16-a
        print("[STOP] --can-combos 는 --virtual-can 과 --episodes 와 같이 쓴다")
        raise SystemExit(2)
    if args.grasp_strict and not args.virtual_can:
        print("[STOP] --grasp-strict 는 --virtual-can 과 같이 쓴다")
        raise SystemExit(2)
    if args.grasp_top_band and args.grasp_mode != "top":                                                                      # 19-x3
        print("[STOP] --grasp-top-band 는 --grasp-mode top 과 같이 쓴다")
        raise SystemExit(2)
    if (args.grasp_mode != "bottom" or args.grasp_real or args.grasp_real_depth_cm is not None) and not args.virtual_can:   # 19-x
        print("[STOP] --grasp-mode · --grasp-real 은 --virtual-can 과 같이 쓴다")
        raise SystemExit(2)
    if (args.start_gate or args.start_gate_h_cm is not None) and not (args.virtual_can and args.episodes):                  # 19-x
        print("[STOP] --start-gate 는 --virtual-can 과 --episodes 와 같이 쓴다")
        raise SystemExit(2)
    if args.viz_pair_after_grasp and not (args.virtual_can and args.robot_viz):   # 16-d
        print("[STOP] --viz-pair-after-grasp 는 --virtual-can 과 --robot-viz 와 같이 쓴다")
        raise SystemExit(2)
    if (args.grasp_align or args.grasp_align_deg is not None or args.grasp_align_axis_cm is not None) and not args.virtual_can:
        print("[STOP] --grasp-align 은 --virtual-can 과 같이 쓴다")
        raise SystemExit(2)
    if (args.grasp_align_deg is not None or args.grasp_align_axis_cm is not None) and not args.grasp_align:
        print("[STOP] --grasp-align-deg·--grasp-align-axis-cm 는 --grasp-align 과 같이 쓴다")
        raise SystemExit(2)
    if args.start_pose == "table" and not (args.auto_start and args.retarget == "hand" and args.table):
        print("[STOP] --start-pose table 은 --auto-start --retarget hand --table 과 같이 쓴다")
        raise SystemExit(2)
    if args.auto_session and args.auto_start:
        print("[STOP] --auto-session 과 --auto-start 는 같이 쓸 수 없다")
        raise SystemExit(2)
    if (args.auto_session or args.auto_start) and args.cue_voice and args.cue_sound:
        print(f"[AUTO] 음성 안내: 말 → 삑 → 측정. 말 길이 표: {load_voice_durations()}")
    auto_lead = (args.auto_lead if args.auto_lead is not None else
                 15.0 if args.auto_start else 25.0 if args.auto_session in ("twist", "speed", "check", "yaw", "wrist") else 40.0)
    auto = None
    if args.auto_start:
        auto = AutoSession(auto_lead, make_laps("free", args.retarget), CueSound(args.cue_sound, args.cue_verbose, voice=args.cue_voice),
                           None, speed=args.auto_speed, free=True,
                           start_json=(Path(str(args.log_csv)).with_name(Path(str(args.log_csv)).stem + "_start.json")
                                       if args.log_csv else None))
        mode = {"pos": "위치 방식", "hand": "손 위치 방식"}.get(args.retarget, "뼈 방식")
        print(f"[AUTO] 자유 트래킹: 준비 {auto_lead:.0f}초 → 중립 → POSITION → 계속 따라가기 ({mode})")
        print("[AUTO] 끝낼 때: h(멈춤) → x(힘 빼기), 또는 Ctrl+C")
    elif args.auto_session:
        phase_csv = Path(str(args.log_csv)).with_name(
            Path(str(args.log_csv)).stem + "_phases.csv") if args.log_csv else None
        auto = AutoSession(auto_lead, make_laps(args.auto_session), CueSound(args.cue_sound, args.cue_verbose, voice=args.cue_voice),
                           phase_csv, speed=args.auto_speed,
                           start_json=(Path(str(args.log_csv)).with_name(Path(str(args.log_csv)).stem + "_start.json")
                                       if args.log_csv else None))
        print(f"[AUTO] 자동 측정: 준비 {auto_lead:.0f}초 → 중립 → POSITION → "
              + " → ".join(f"{lap['label']}({len(lap['phases'])}구간)" for lap in auto.laps))
        if args.cue_sound:
            print(f"[AUTO] 구간 소리: {args.cue_sound} (서버 PC 의 cue_player.py)")
        else:
            print("[AUTO] 🔴 소리 없음 — --cue-sound IP:PORT 를 주면 서버 PC 에서 소리가 난다")
        if phase_csv:
            print(f"[AUTO] 구간 표시 CSV: {phase_csv}")

    rx_log = None
    if args.log_rx:                                   # REPORT_52: 원시 패킷 기록 (제어와 무관)
        rx_log = RxLogger(str(Path(args.log_rx).expanduser()))
        if auto is not None:
            rx_log.phase_fn = lambda: auto.phase
        print(f"원시 패킷 기록  : {rx_log.path} (--log-rx)")
    receiver = BodyTrackingUdpReceiver(args.udp_ip, args.udp_port, rx_log=rx_log)
    retargeters = {
        side: ArmRetargeter(
            side,
            ema_alpha=args.ema_alpha,
            swing_suppression_exponent=args.swing_suppression_exponent,
            swing_taper_start=args.swing_taper_start,
            swing_taper_end=args.swing_taper_end,
            chest_relative=args.chest_relative,
        )
        for side in sides
    }
    pos_runner = PosRetargetRunner(sides, robot_ref, body_frame=args.body_frame, sy_policy=args.sy_policy,
                                   wrist_median=args.wrist_median, limits=limits,
                                   wrist_hold=not args.no_wrist_hold,
                                   overhead_elbow_hold=not args.no_overhead_elbow_hold,
                                   elbow_base=not args.no_elbow_base,
                                   flip_hold=not args.no_flip_hold,
                                   hand_refit=not args.no_hand_refit,
                                   hand_cfg=dict(touch_cm=args.hand_touch_cm, min_gap_cm=args.hand_min_gap_cm,
                                                 max_speed=args.hand_max_speed, swivel_tau=args.hand_swivel_tau,
                                                 gap_goal_cm=args.hand_gap_goal_cm, blend_cm=args.hand_blend_cm,
                                                 pair=args.hand_pair == "on", pair_near_cm=args.hand_pair_near_cm,
                                                 pair_far_cm=args.hand_pair_far_cm, grip_offset_cm=args.grip_offset_cm,
                                                 touch_mode=args.hand_touch_mode,
                                                 **({"pair_center": "robot-hand"} if args.hand_pair_center == "robot-hand" else {})))
    if args.heading_deadband_deg is not None:        # 작업 지시 12-b (calibrate 에서 기준을 잡는다)
        pos_runner.heading_cfg = (args.heading_deadband_deg, args.heading_tau_s) + ((args.heading_soft_deg,) if args.heading_soft_deg is not None else ())
    if auto is not None and args.retarget == "hand":     # REPORT_38: 자동 세션의 위치 방식 바퀴를 손 위치 방식으로
        for lap in auto.laps:
            if lap.get("retarget") == "pos":
                lap["retarget"] = "hand"
                lap["label"] = str(lap.get("label", "")).replace("위치 방식", "손 위치 방식") + " [hand]"
    if auto is not None:
        auto.wrist_e90 = not args.no_wrist_e90
        auto.start_check_cfg = {"on": not args.no_start_check, "gap_min_cm": args.start_gap_min_cm,
                                "scale_range": tuple(args.start_scale_range)}
        auto.run_info = {"arm_gap_cm": args.arm_gap_cm, "arm_collision": not args.no_arm_collision,
                         "hand_model": args.hand_model, "hand_gap_goal_cm": args.hand_gap_goal_cm,
                         "hand_pair": ({"on": True, "near_cm": args.hand_pair_near_cm, "far_cm": args.hand_pair_far_cm,
                                        "grip_offset_cm": args.grip_offset_cm,
                                        **({"center": "robot-hand"} if args.hand_pair_center == "robot-hand" else {})}
                                       if args.hand_pair == "on" else {"on": False}),
                         "mirror_limits": args.mirror_info,
                         "flip_hold": not args.no_flip_hold, "hand_touch_mode": args.hand_touch_mode,
                         "hand_refit": not args.no_hand_refit,
                         "yaw_in_limit_deg": list(args.yaw_in_limit_deg) if args.yaw_in_limit_deg is not None else None,
                         "max_speed": args.max_speed,
                         "gripper": ({"port": args.gripper_port, "open_deg": list(args.gripper_open_deg),
                                      "close_deg": list(args.gripper_close_deg), "close_cm": args.grip_close_cm,
                                      "open_cm": args.grip_open_cm, "open_hold_s": args.grip_open_hold_s}
                                     if args.gripper_port else None)}
        if args.grip_open_rule == "finger":                 # 작업 지시 14-p (켰을 때만)
            auto.run_info["grip_open_rule"] = {"rule": "finger", "off_cm": 7.0, "mid_open_s": args.grip_mid_open_s,
                                               "handover_guard_s": args.grip_handover_guard_s}
        if args.grip_virtual and not args.gripper_port:     # 작업 지시 10 (켰을 때만)
            auto.run_info["grip_virtual"] = {"close_cm": args.grip_close_cm, "open_cm": args.grip_open_cm,
                                             "open_hold_s": args.grip_open_hold_s}
            if args.grip_virtual_timing != "instant":           # 작업 지시 14-j (켰을 때만)
                auto.run_info["grip_virtual"]["timing"] = {"mode": args.grip_virtual_timing, "step_ms": 10, "settle_ms": 400, "one_at_a_time": True}
        if args.handover_exempt:
            auto.run_info["handover_exempt"] = {"when": args.handover_when, "pair_w_min": 0.5, "ramp_s": 0.3,
                                                "finger_cut_cm": PR.HAND_FINGER_M * 100, "finger_gap_cm": args.handover_finger_gap_cm,
                                                "finger_gap_hand_cm": args.handover_finger_gap_cm + HANDOVER_FINGER_CUSHION_CM}
        if args.table:                                      # 작업 지시 11-b (켰을 때만)
            auto.run_info["table"] = {"path": args.table, "body": args.table_body, "start_pose": args.start_pose}
        if args.start_pose == "table":
            auto.table_start = True
        if args.heading_deadband_deg is not None:           # 작업 지시 12-b (켰을 때만)
            auto.run_info["heading_deadband"] = {"deg": args.heading_deadband_deg, "tau_s": args.heading_tau_s}
            if args.heading_soft_deg is not None:
                auto.run_info["heading_deadband"]["soft_deg"] = args.heading_soft_deg
        if args.robot_viz:                                  # 켰을 때만 (끄면 _start.json 그대로)
            auto.run_info["robot_viz"] = {"target": args.robot_viz, "hz": args.robot_viz_hz,
                                          "real_size": bool(args.robot_viz_real_size),
                                          "jaw_sign": args.robot_viz_jaw_sign, "jaw_offset_deg": args.robot_viz_jaw_offset_deg}
    motor = DualArmMotorController(
        channels=channels,
        sides=sides,
        motor_ids=motor_ids,
        urdf_limits=limits,
        kp=args.kp,
        kd=args.kd,
        torque_limit=args.torque_limit,
        max_speed=args.max_speed,
        dry_run=args.dry_run,
    )

    motor.dry_run_base = {side: dict(robot_ref.rest[side]) for side in sides}
    # REPORT_36: 비틀기 안쪽 한계(위팔 든 각) + 양팔 충돌 방지
    motor.yaw_dyn = {side: (float(robot_ref.rest[side]["shoulder_pitch"]), float(robot_ref.e90[side]["shoulder_yaw"]))
                     for side in sides}
    if not args.no_arm_collision and len(sides) == 2:
        motor.arm_gap_fn = make_arm_gap_fn(robot_ref, sides)
        motor.arm_gap_min = args.arm_gap_cm / 100.0
    # 작업 지시 11-b 가대 면 제한 (옵션). 설정 파일을 못 읽으면 판을 모른 채 움직이지 않도록 멈춘다
    table = None
    if args.table:
        import table_limit as TLIM  # type: ignore  (POS_TOOLS 가 sys.path 에 있다)
        try:
            table = TLIM.TableLimit(Path(args.table).expanduser(), body=args.table_body, margin_body_cm=args.table_margin_body_cm,
                                    margin_tip_cm=args.table_margin_tip_cm)
        except (OSError, ValueError, KeyError) as exc:
            print(f"[STOP] --table 설정을 못 읽음: {exc}")
            raise SystemExit(2)
        pos_runner.table = table
        motor.table_fn = make_table_clear_fn(robot_ref, sides, table)
        if args.start_pose == "table":
            motor.start_ramp_s = 4.0
            motor.dry_run_base = model_park_pose(robot_ref, sides, motor.table_fn)     # dry-run 만 쓴다 (실기동은 엔코더 값)
    # REPORT_51: 그리퍼 (옵션). 직렬은 별도 스레드 — 포트가 없거나 끊겨도 팔 제어는 그대로 돈다
    handover = None
    if args.handover_exempt and motor.arm_gap_fn is not None:   # 작업 지시 10 (충돌 방지를 끄면 예외도 없다)
        motor.handover_gap_fn = make_handover_gap_fn(robot_ref, sides)
        motor.handover_finger_min = args.handover_finger_gap_cm / 100.0
        handover = HandoverGate(args.handover_when, full_m=PR.HAND_FINGER_M)
    gripper = None
    gripper_last_t = None
    grip_rule_kw = {}                                   # 작업 지시 14-p (켰을 때만 넘긴다)
    if args.grip_open_rule == "finger":
        grip_rule_kw = {"open_rule": "finger", "mid_open_s": args.grip_mid_open_s, "handover_guard_s": args.grip_handover_guard_s}
    grip_rule_prev = {"L": 0, "R": 0}
    if args.grip_virtual and not args.gripper_port:     # 작업 지시 10: 직렬 없는 판단만 (GripperLink(None) 은 아무것도 안 함)
        import gripper_link as GLINK  # type: ignore  (POS_TOOLS 가 sys.path 에 있다)
        gripper = GLINK.GripperController(
            None, open_deg=[int(round(v)) for v in args.gripper_open_deg],
            close_deg=[int(round(v)) for v in args.gripper_close_deg], close_cm=args.grip_close_cm,
            open_cm=args.grip_open_cm, open_hold_s=args.grip_open_hold_s, **grip_rule_kw)
        print(f"그리퍼 (가상)    : 직렬 없음 — 닫힘 ≤ {args.grip_close_cm:.1f} cm, 열림 ≥ {args.grip_open_cm:.1f} cm "
              f"{args.grip_open_hold_s:.1f}초, 사람을 따라갈 때(POS)만 판단")
    vgrip = None
    if gripper is not None and args.grip_virtual_timing == "real":     # 작업 지시 14-j (켰을 때만)
        vcue = CueSound(args.cue_sound, args.cue_verbose, voice=False) if args.cue_sound else None

        def _reach(hand, what):
            if vcue is not None:
                vcue.send("tick" if what == "closed" else "switch", text=f"그리퍼 {hand} 다 {'닫힘' if what == 'closed' else '열림'}")
        vgrip = GLINK.TimedVirtualGrip(gripper, on_reach=_reach)
        print("그리퍼 속도 (14-j): 🔵 real — 아두이노와 같은 시간 (한 손씩, 닫는 쪽 먼저, 1°/10 ms, 닿으면 0.4 초). 가상 캔은 다 닫힌 뒤 잡고 다 열린 뒤 놓는다"
              + (f", 다 닫힘 tick / 다 열림 switch 소리 → {args.cue_sound}" if vcue is not None else ""))
    if args.gripper_port:
        try:
            import gripper_link as GLINK  # type: ignore  (POS_TOOLS 가 sys.path 에 있다)
            gripper = GLINK.GripperController(
                args.gripper_port, open_deg=[int(round(v)) for v in args.gripper_open_deg],
                close_deg=[int(round(v)) for v in args.gripper_close_deg], close_cm=args.grip_close_cm,
                open_cm=args.grip_open_cm, open_hold_s=args.grip_open_hold_s, **grip_rule_kw)
            print(f"그리퍼 (51)      : {args.gripper_port} — 닫힘 ≤ {args.grip_close_cm:.1f} cm, 열림 ≥ {args.grip_open_cm:.1f} cm "
                  f"{args.grip_open_hold_s:.1f}초, 열림 {args.gripper_open_deg} / 닫힘 {args.gripper_close_deg}°, "
                  "사람을 따라갈 때(POS)만 움직임, 끝낼 때 그대로 둠")
        except Exception as exc:
            print(f"[그리퍼] 🔴 시작 못 함: {exc} — 그리퍼 없이 계속한다")
            gripper = None
    if args.grip_open_rule == "finger":                 # 작업 지시 14-p
        if gripper is not None:
            print(f"그리퍼 열림 판정 (14-p): 🔵 finger — 컨트롤러는 ≥ 7.0 (손가락 뗌) {args.grip_open_hold_s:.1f}초 뒤 열림, "
                  f"4.5~7 (트리거에 얹힘) 은 {args.grip_mid_open_s:.1f}초 이어져야 열림, 반대 손이 연 뒤 {args.grip_handover_guard_s:.1f}초는 "
                  "중간값으로 안 엶. 손 추적은 그대로")
        else:
            print("그리퍼 열림 판정 (14-p): --grip-open-rule finger 이지만 그리퍼 판단이 없어 쓰이지 않음")
    # VR 표시 (옵션, 기본 끔). 읽기만 하는 송신 — 실패해도 팔 제어는 그대로 돈다
    robot_viz = None
    if args.robot_viz:
        try:
            import robot_viz as RVIZ  # type: ignore  (POS_TOOLS 가 sys.path 에 있다)
            robot_viz = RVIZ.RobotViz(args.robot_viz, robot_ref, sides, motor_to_urdf_fn(robot_ref, sides),
                                      hz=args.robot_viz_hz, real_size=args.robot_viz_real_size,
                                      jaw_sign=args.robot_viz_jaw_sign, jaw_offset_deg=args.robot_viz_jaw_offset_deg,
                                      dry_run=args.dry_run, table=table)
            print(f"VR 표시 (robot-viz): {args.robot_viz} 로 보낸다 ({RVIZ.PACKET_SIZE} 바이트, {args.robot_viz_hz:.0f} Hz)")
        except Exception as exc:
            print(f"[robot-viz] 🔴 시작 못 함: {exc} — VR 표시 없이 계속한다")
            robot_viz = None
    # 작업 지시 13 가상 캔 (옵션, 기본 끔). 읽기만 한다. 자리 이름이 틀리면 모터를 켜기 전에 멈춘다
    vcan = None
    if args.virtual_can:
        import virtual_can as VCAN  # type: ignore  (POS_TOOLS 가 sys.path 에 있다)
        can_cue = CueSound(args.cue_sound, args.cue_verbose, voice=False)
        margin = VCAN.GRIP_MARGIN_CM if args.virtual_can_margin_cm is None else args.virtual_can_margin_cm
        episodes_path = (Path(str(args.log_csv)).with_name(Path(str(args.log_csv)).stem + "_episodes.csv") if args.episodes else None)
        place_cm = VCAN.PLACE_HEIGHT_CM if args.virtual_can_place_cm is None else args.virtual_can_place_cm
        goal_tol = VCAN.GOAL_TOL_CM if args.virtual_can_goal_tol_cm is None else args.virtual_can_goal_tol_cm
        handover_extra = VCAN.HANDOVER_HEIGHT_EXTRA_CM if args.virtual_can_handover_extra_cm is None else args.virtual_can_handover_extra_cm
        reset_s = (VCAN.EPISODE_RESET_S if args.episode_reset_s is None else args.episode_reset_s) if args.episodes else None
        timeout_s = VCAN.EPISODE_TIMEOUT_S if args.episode_timeout_s is None else args.episode_timeout_s
        try:
            can_pairs = VCAN.load_pairs(args.can_pairs) if args.can_pairs else None      # 19-b (없으면 None = 지금 그대로)
            vcan = VCAN.VirtualCan(table, args.virtual_can, robot_ref, sides, motor_to_urdf_fn(robot_ref, sides), goal=args.virtual_can_goal,
                                   margin_cm=margin, jaw_sign=args.robot_viz_jaw_sign, jaw_offset_deg=args.robot_viz_jaw_offset_deg,
                                   servo_sign=table_servo_signs(robot_ref, sides), target=args.robot_viz, episodes_path=episodes_path, sound=can_cue.send,
                                   handover_extra_cm=handover_extra, reset_s=reset_s, timeout_s=timeout_s, place_cm=place_cm, goal_tol_cm=goal_tol,
                                   **({"episode_end": args.episode_end} if args.episode_end != "place" else {}),
                                   **({"align": (*(args.grasp_align_deg or (VCAN.ALIGN_DEG_FIRST, VCAN.ALIGN_DEG_SECOND)),
                                                 VCAN.ALIGN_AXIS_CM if args.grasp_align_axis_cm is None else args.grasp_align_axis_cm)}
                                      if args.grasp_align else {}),
                                   **({"combos": [tuple(c.strip().split(":")) for c in args.can_combos.split(",") if c.strip()]} if args.can_combos else {}),
                                   **({"pairs": can_pairs} if can_pairs else {}),
                                   **({"strict": True, "ready_strict": True} if args.grasp_strict else {}),
                                   **({"grasp_mode": args.grasp_mode} if args.grasp_mode != "bottom" else {}),                       # 19-x (켤 때만 넘김)
                                   **({"top_band": True} if args.grasp_top_band else {}),                                           # 19-x3
                                   **({"real": True, "real_depth_cm": (VCAN.REAL_DEPTH_LOGIC_CM if args.grasp_real_depth_cm is None else args.grasp_real_depth_cm)}
                                      if args.grasp_real else {}),
                                   **({"start_gate": VCAN.START_GATE_H_CM if args.start_gate_h_cm is None else args.start_gate_h_cm} if args.start_gate else {}))
        except (OSError, ValueError) as exc:
            print(f"[STOP] --virtual-can: {exc}")
            raise SystemExit(2)
        if auto is not None:
            auto.run_info["virtual_can"] = vcan.run_info()
            auto.run_info["episodes"] = {"on": bool(args.episodes)}
        print(f"가상 캔          : 시작 {vcan.start_name}" + (f", 목표 {vcan.goal_name} (허용 {goal_tol:g} cm)" if vcan.goal_name else "") + f", 판정 여유 {margin:.1f} cm"
              + (f", RVC1 {args.robot_viz} 30 Hz" if args.robot_viz else ", VR 표시 없음 (--robot-viz 없음)"))
        if args.episodes:
            state["episodes"] = True
            print(f"에피소드 기록    : {episodes_path} — 키보드 없이 돈다: 위치 모드가 되면 시작, 끝나면 {reset_s:.0f}초 뒤 다시 세움, 시간 초과 {timeout_s:.0f}초. 키 n·s·f 도 쓸 수 있다 (f 의 속도 피드포워드 전환은 꺼짐)")
            if args.episode_end != "place":                  # 작업 지시 14-h (켤 때만)
                print(f"에피소드 끝 (14-h): 🔵 release — 목표에 놓은 뒤 두 그리퍼 열림 + 두 집게 끝이 캔에서 {VCAN.RELEASE_CLEAR_CM:g} cm 넘게 "
                      f"{VCAN.RELEASE_CLEAR_S:g} 초여야 성공 (그때까지 기록). 쥔 채 목표 안 = 놓기 대기")
        if vcan.pairs is not None:                             # 작업 지시 19-b (켤 때만)
            print(f"좌표 쌍 순환 (19-b): 🔵 {args.can_pairs} — {len(vcan.pairs)} 쌍, 차례로. 첫 쌍 시작 ({vcan.pairs[0][0]:g}, {vcan.pairs[0][1]:g}) → 도착 ({vcan.pairs[0][2]:g}, {vcan.pairs[0][3]:g}). "
                  f"자리 {len(table.cfg['can_slots'])} 개 (언리얼 한도 12)")
        if vcan.combos is not None:                            # 작업 지시 16-a (켤 때만)
            print(f"조합 순환 (16-a)  : 🔵 " + " → ".join(f"{a}:{b}" for a, b in vcan.combos) + " → 처음으로. 에피소드가 끝날 때마다 다음 조합 (VR 목표 원·캔 자리, 소리 ready)")
        if vcan.strict:
            print(f"엄격 판정 (16-a)  : 🔵 실패 = 열린 날·몸체가 캔에 {VCAN.STRICT_OPEN_CM:g} cm 넘게 들어감, 집게 가운데가 캔 축보다 {VCAN.STRICT_DEPTH_CM:g} cm 넘게 덜 들어감 "
                  f"또는 닫히는 축으로 {VCAN.STRICT_SIDE_CM:g} cm 넘게 비킴 / 경고 = 두 그리퍼 간격 < {VCAN.STRICT_GAP_CM:g} cm, 놓은 자리 − 목표 > {VCAN.STRICT_GOAL_CM:g} cm "
                  "(결과는 그대로, 에피소드 기록 strict_*)")
        if vcan.grasp_mode == "top":                           # 작업 지시 19-x (켤 때만)
            print(f"잡기 모드 (19-x)  : 🔵 위쪽 잡기 — L 이 먼저 캔 밑면에서 {VCAN.TOP_FIRST_T_CM[0]:g} ~ {VCAN.TOP_FIRST_T_CM[1]:g} cm, "
                  f"R 이 {VCAN.TOP_SECOND_T_CM[0]:g} ~ {VCAN.TOP_SECOND_T_CM[1]:g} cm 로 받음 (순서 · 높이가 다르면 못 잡음). "
                  + (f"VR 띠 {VCAN.TOP_BAND_HDR_CM:g} cm (--grasp-top-band)" if vcan.top_band else "VR 띠 · 원은 지금 방식 그대로 (19-x3)"))
        if vcan.real:                                          # 작업 지시 19-x (켤 때만)
            print(f"실물 기준 (19-x)  : 🔵 실패 = 인계 깊이 < {vcan.real_depth:.2f} cm (논리, 엔코더 상당 {vcan.real_depth + 1.30:.1f}), 잡기 비킴 > {VCAN.REAL_SIDE_CM:g} cm, "
                  f"놓을 때 누름 > {VCAN.REAL_PRESS_CM:g} cm, 잡기 전 부딪힘 — 까닭별 음성, 에피소드 기록 real_* (결과는 그대로)"
                  + (f". 인계 각도 (19-x2): R 기울기 {VCAN.TOP_REAL_TILT_DEG[0]:g} ~ {VCAN.TOP_REAL_TILT_DEG[1]:g}° (< {VCAN.TOP_REAL_FLAT_DEG:g}° = 인계 수평), "
                     f"두 손 축 {VCAN.TOP_REAL_AXES_DEG[0]:g} ~ {VCAN.TOP_REAL_AXES_DEG[1]:g}°" if vcan.grasp_mode == "top" else ""))
        if vcan.start_gate is not None:                        # 작업 지시 19-x (켤 때만)
            print(f"시작 자세 (19-x)  : 🔵 두 그리퍼 닫고 두 집게 끝을 판 위 {vcan.start_gate:g} cm 넘게 · 캔에서 떨어뜨려 {VCAN.START_GATE_HOLD_S:g} 초 → 에피소드 시작 "
                  f"(기다리는 동안 {VCAN.START_GATE_SAY_S:g} 초마다 음성 \"{VCAN.START_GATE_SAY}\" · 화면에 안 맞는 조건, 통과하면 음성 \"시작\")")
        if vcan.ready_strict:                                  # 작업 지시 16-d (--grasp-strict 와 함께)
            print(f"노란 원 (16-d)    : 🔵 잡힘에 더해 집게 가운데 덜 들어감 ≤ {VCAN.STRICT_DEPTH_CM:g} cm · 비킴 ≤ {VCAN.STRICT_SIDE_CM:g} cm 일 때만 (엄격 판정과 같은 문턱)")
        if args.viz_pair_after_grasp and robot_viz is not None:   # 작업 지시 16-d (켤 때만)
            robot_viz.pair_gate = lambda: bool(vcan.holders)
            if auto is not None and "robot_viz" in auto.run_info:
                auto.run_info["robot_viz"]["pair_after_grasp"] = {"on": True, "ramp_s": RVIZ.PAIR_GATE_RAMP_S}
            print(f"VR 쌍 매핑 그림 (16-d): 🔵 가상 캔을 쥔 손이 없으면 그림 w = 0 ({RVIZ.PAIR_GATE_RAMP_S:g} 초에 걸쳐 바꿈), 로봇 명령 쌍 매핑은 그대로")
        if vcan.align is not None:                             # 작업 지시 14-l (켤 때만)
            print(f"잡기 정렬 (14-s)  : 🔵 먼저 잡는 손 집게 축 ⟂ 캔 축 어긋남 ≤ {vcan.align[0]:g}° (숙임 ≤ {VCAN.ALIGN_N_MAX_DEG:g}°), "
                  f"둘째 손 셋째 축 ↔ 쥔 캔 축 ≤ {vcan.align[1]:g}° + 집게 가운데 ↔ 캔 축 ≤ {vcan.align[2]:g} cm (노란 테두리도 같은 조건, 둘째 손이 쥔 캔을 밀면 알림)")
    # 작업 지시 14 C 녹화 재생 (옵션, 기본 끔). 파일·기준값이 틀리면 모터를 켜기 전에 멈춘다
    replay = None
    if args.replay:
        import demo_replay as DREP  # type: ignore  (POS_TOOLS 가 sys.path 에 있다)
        try:
            replay = DREP.DemoReplay(args.replay[0], robot_ref, sides, speed=args.replay_speed, move_s=args.replay_move_s,
                                     can_target=args.robot_viz)
        except (OSError, ValueError, KeyError) as exc:
            print(f"[STOP] --replay: {exc}")
            raise SystemExit(2)
        state["replay"] = True
        if motor.table_fn is not None:     # dry-run 만: 차렷은 판에 붙어 있어 가대 면 제한이 첫 자세로 가는 길을 막는다. 실기동은 엔코더 값 (판 위 주차)
            motor.dry_run_base = model_park_pose(robot_ref, sides, motor.table_fn)
        ri = replay.info()
        print(f"녹화 재생        : 🔵 {ri['record']} 에피소드 {ri['episode']} ({ri['result']}, {ri['duration_s']:.1f} 초, {ri['frames']} 프레임), "
              f"배율 {ri['speed']:g}, 첫 자세까지 {ri['move_s']:.0f} 초")
        print(f"                   기록 기준값 {ri['recorded_ref']} — 지금 기준값과 {'같다' if ri['same_ref'] else '다르다 (URDF 각에서 다시 계산)'}. "
              + ("p = 첫 자세로, g = 재생, h = 멈춤, x = 감쇠" if not args.replay_auto else "키 없이 자동 순서 (아래)") + ". 추적 패킷은 명령에 안 쓴다"
              + ("" if gripper is not None else ". 그리퍼 없음 (--grip-virtual 또는 --gripper-port 를 주면 그리퍼도 재생)"))
    # 작업 지시 14-b 키보드 없는 재생·걸림 감시·종료 동작 (옵션, 기본 끔. 끄면 import·계산 모두 없다)
    seq = stall = end_gesture = beeper = end_ret = None
    stall_at = None
    exit_code = 0
    vr_link = vr_loss = None
    vr_pos_on = False
    if args.replay_auto or args.stall_stop or args.auto_end or args.vr_loss_park or args.vr_guard:
        import auto_target as AT  # type: ignore  (POS_TOOLS 가 sys.path 에 있다)
        beeper = AT.Beeper(local=args.beep_local, cue=CueSound(args.cue_sound, args.cue_verbose, voice=False) if args.cue_sound else None)
        print(f"삐 소리 (14-b)   : {beeper.where}" + ("" if beeper.cmd or not args.beep_local else " — 🔴 로봇 PC 재생 명령 (paplay·aplay) 없음"))
    if args.vr_loss_park:                  # 작업 지시 14-c (켰을 때만)
        vr_link = AT.VrLink()
        print(f"VR 끊김 (14-c)   : 🔵 켬 — 패킷 {AT.VR_NOPKT_S:g} 초 없음·active false·값 그대로 {AT.VR_FROZEN_S:g} 초·머리가 모터 켤 때 자리에서 "
              f"수평 {AT.VR_HEAD_HORIZ_CM:g} / 아래 {AT.VR_HEAD_DROP_CM:g} cm 넘게 {AT.VR_HEAD_S:g} 초 → 그 자리 유지 + 삐, "
              f"{AT.VR_GRACE_S:g} 초 안에 오면 이어서, 넘으면 주차 {AT.VR_PARK_S:g} 초 → 감쇠 → 종료 (코드 3)")
    if args.vr_guard:
        vr_link = AT.VrLink(None, AT.VR_HEAD_DROP_GUARD_CM, back=AT.VR_HEAD_BACK_GUARD)
        print(f"VR 감시 (14-c)   : 🔵 켬 — VR 이 {AT.VR_ALIVE_S:g} 초 이어서 들어와야 시작. 끊김 (패킷·active·값 그대로·머리 아래 {AT.VR_HEAD_DROP_GUARD_CM:g} cm) "
              f"→ 유지, {AT.VR_GRACE_S:g} 초 넘으면 주차 → 감쇠 → 종료 (코드 3)")
    if args.replay_auto:
        tracks = []
        for i, path in enumerate(args.replay):
            try:
                rep = replay if i == 0 else DREP.DemoReplay(path, robot_ref, sides, speed=args.replay_speed, move_s=args.replay_move_s,
                                                            can_target=args.robot_viz)
            except (OSError, ValueError, KeyError) as exc:
                print(f"[STOP] --replay: {exc}")
                raise SystemExit(2)
            tracks += [rep] * args.replay_repeat
        state.pop("replay", None)          # 자동 순서에서는 p / g 를 쓰지 않는다 (h / x / Ctrl+C 는 비상 멈춤으로 그대로)
        seq = AT.AutoRun(tracks, beeper, move_s=args.replay_move_s, wait_s=args.replay_wait_s, vr_guard=vr_link if args.vr_guard else None)
        print(f"자동 재생 (14-b) : 🔵 {len(tracks)} 회차 — {seq.plan_text()}. 삐 + {AT.LEAD_S:.0f} 초 → 모터 켬 → 첫 자세 ({args.replay_move_s:g} 초) "
              f"→ 캔 놓을 시간 {args.replay_wait_s:g} 초 → 재생 → 끝 자세 {AT.FINAL_HOLD_S:g} 초 → 주차 → 감쇠. 키 없이 끝까지 (종료 코드 0, 비상 멈춤 3)")
    if args.stall_stop:
        stall = AT.StallGuard(sides)
        print("걸림 감시 (14-b) : 🔵 켬 — 움직이는 중 " + ", ".join(f"{j} {d:g}°·{t:g}초" for j, (d, t) in AT.STALL_MOVE.items())
              + f" / 명령이 멈춘 때 {AT.STALL_STILL[0]:g}°·{AT.STALL_STILL[1]:g}초 넘으면 멈춤 → 감쇠 → 종료 (코드 3)"
              + (". dry-run 은 실제 각이 없어 보지 않는다" if args.dry_run else ""))
    if args.auto_end:
        import demo_export as DE  # type: ignore  (POS_TOOLS 가 sys.path 에 있다)
        _jm = DE.JointMap(robot_ref)

        def _tips(pose):
            arr = np.array([[[float((pose[s] if s in pose else robot_ref.rest[s])[j]) for j in DE.JOINTS] for s in DE.SIDES]])
            t = _jm.tips(arr)[0]
            return {s: t[i] for i, s in enumerate(DE.SIDES) if s in sides}
        end_gesture = AT.EndGesture(_tips)
        print(f"자동 종료 (14-b) : 🔵 켬 — 로봇 집게 끝을 처음 자리 {AT.END_RADIUS_CM:g} cm 안으로 내리고 {AT.END_HOLD_S:g} 초 가만히 "
              f"({AT.END_SPEED_CM_S:g} cm/s 이하, 그리퍼 열림) → 주차로 {AT.END_RETURN_S:g} 초 → 감쇠 → 종료 (코드 0). "
              f"처음 자리에서 {AT.END_RADIUS_CM + AT.END_ARM_CM:g} cm 넘게 나가 있던 시간이 합쳐 {AT.END_ARM_S:g} 초가 된 뒤부터 본다")
    park_relax = None
    if args.park_table_relax and motor.table_fn is not None:    # 작업 지시 14-m (켤 때만): 주차 뒤 절반에서 판 면 허용
        park_relax = {"allow": None}
        _table_core = motor.table_fn

        def _table_relaxed(side, c, f=_table_core, st=park_relax):
            out = f(side, c)
            al = st["allow"]
            if al is None or al.get(side) is None:
                return out
            out = dict(out)
            out["g"] = out["g"] - al[side]
            return out
        motor.table_fn = _table_relaxed
        print("주차 판 면 (14-m): 🔵 켬 — 주차 뒤 절반에서만 판 면 제한을 주차 자세의 판 거리까지 허용 (일반 제한은 그대로)")

    def _park_allow():
        """주차 자세 (motor.base_target = 모터를 켤 때 자세) 의 판 거리 (0 이하, −0.2 cm 더). 14-m"""
        return {sd: min(0.0, _table_core(sd, motor.base_target[sd])["g"]) - 0.002 for sd in sides}
    meas_ok = not motor.dry_run or bool(getattr(motor, "sim_meas", False))     # 실제 관절각이 있나 (dry-run 은 없다. 시험 도구가 sim_meas 를 세운다)
    motor.vel_ff = bool(args.vel_ff)
    motor.vel_ff_alpha = float(clamp(args.vel_ff_alpha, 0.05, 1.0))
    receiver.start()
    motor.initialize()
    threading.Thread(target=keyboard_worker, args=(state,), daemon=True).start()

    rate = RateLimiter(frequency=LOOP_HZ)
    previous_time = time.monotonic()
    last_print = 0.0
    last_log = 0.0
    timed_out = False

    log_writer = None
    log_file = None
    if args.log_csv:
        log_file = open(args.log_csv, "w", newline="")
        log_writer = csv.writer(log_file)
        header = ["t", "mode", "tracking", "hold", "retarget", "body_heading_deg", "phase", "sy_policy", "vel_ff"]
        for side in sides:
            header += [
                f"{side}_swing_roll_raw", f"{side}_swing_yaw_raw",
                f"{side}_swing_roll_suppressed", f"{side}_swing_yaw_suppressed",
                # 가슴 기준 상대화 전/후 비교용. world_* 는 같은 프레임을 월드 기준으로
                # 계산한 값이라 조건이 완전히 같은 대조군이 된다.
                f"{side}_chest_mode",
                f"{side}_swing_roll_world", f"{side}_swing_yaw_world", f"{side}_twist_world",
                f"{side}_swing_roll_chest", f"{side}_swing_yaw_chest", f"{side}_twist_chest",
            ]
            header += [f"{side}_u_x", f"{side}_u_y", f"{side}_u_z",
                       f"{side}_elbow_deg", f"{side}_hold_w",
                       f"{side}_wrist_pron_deg", f"{side}_wrist_target",
                       f"{side}_sy_ik", f"{side}_arm_twist_deg", f"{side}_bone_offset"]
            for joint in JOINT_ORDER:
                header += [f"{side}_{joint}_cmd", f"{side}_{joint}_meas", f"{side}_{joint}_desired",
                           f"{side}_{joint}_vff"]
        header += ["arm_gap_cm", "gap_blocks", "yaw_holds"]       # REPORT_36
        header += ["human_wrist_dist_cm", "human_gap_cm", "robot_gap_target_cm", "hand_widen_cm"]   # REPORT_38
        header += ["flip_holds"]                                                                     # REPORT_42
        header += ["hand_refits"]                                                                    # REPORT_47
        if args.hand_pair == "on":
            header += ["hand_pair_w", "hand_pair_dist_cm"]                                           # REPORT_56 (켤 때만)
        if gripper is not None:
            header += ["grip_L", "grip_R", "pinch_L", "pinch_R"]                                     # REPORT_51
            if grip_rule_kw:
                header += ["grip_rule_L", "grip_rule_R"]                                             # 작업 지시 14-p (켤 때만)
        if vgrip is not None:                                                                        # 작업 지시 14-j (켤 때만)
            header += ["grip_vA_deg", "grip_vB_deg", "grip_effL", "grip_effR"]
        if robot_viz is not None:                                                                    # --robot-viz (켤 때만)
            for side in sides:
                header += [f"{side}_tip_cmd_x", f"{side}_tip_cmd_y", f"{side}_tip_cmd_z",
                           f"{side}_tip_meas_x", f"{side}_tip_meas_y", f"{side}_tip_meas_z", f"{side}_viz_flags"]
        if handover is not None:                                                                     # 작업 지시 10 (켤 때만)
            header += ["handover_on", "arm_gap_full_cm", "arm_gap_finger_cm"]
        if table is not None:                                                                        # 작업 지시 11-b (켤 때만)
            header += ["table_clear_cm", "table_blocks"]
        if args.heading_deadband_deg is not None:                                                    # 작업 지시 12-b (켤 때만)
            header += ["heading_raw_deg", "heading_used_deg"]
        if vcan is not None:                                                                         # 작업 지시 13 (켤 때만)
            header += VCAN.CSV_COLUMNS
            if args.episode_end != "place":                                                          # 작업 지시 14-h (켤 때만)
                header += VCAN.CSV_COLUMNS_RELEASE
        if replay is not None:                                                                       # 작업 지시 14 C (켤 때만)
            header += ["replay_phase", "replay_t", "replay_frame"]
        if seq is not None:                                                                          # 작업 지시 14-b (켤 때만)
            header += ["auto_round", "auto_stage"]
        if stall is not None:
            header += ["stall_level"]
        if vr_link is not None:                                                                      # 작업 지시 14-c (켤 때만)
            header += ["vr_state", "vr_head_horiz_cm", "vr_head_drop_cm", "vr_loss"]
        if end_gesture is not None:
            header += ["end_dist_cm", "end_speed_cm_s", "end_armed", "end_still_s", "end_return"]
        log_writer.writerow(header)
        print(f"Logging to    : {args.log_csv} (~20Hz)")
        print(f"Swing suppress: base_exponent={args.swing_suppression_exponent} taper=[{args.swing_taper_start:.2f},{args.swing_taper_end:.2f}]rad")

    try:
        while not state["quit"]:
            now = time.monotonic()
            dt = now - previous_time
            previous_time = now
            frame = receiver.get_latest()
            if pos_runner.heading is not None and frame is not None:      # 작업 지시 12-b: 모터를 켜기 전에도 프레임마다 나아간다
                pos_runner.view_bones(frame)
            frame_age = math.inf if frame is None else now - frame.received_at
            tracking_ok = bool(
                frame
                and frame.active
                and frame_age <= UDP_TIMEOUT_SEC
                and all(retargeters[side].has_required_joints(frame) for side in sides)
            )

            if state["calibrate_requested_at"] is not None:
                elapsed = now - state["calibrate_requested_at"]
                remaining = state["calibrate_delay"] - elapsed
                if remaining <= 0.0:
                    state["calibrate_requested_at"] = None
                    calibrated = bool(frame) and all(
                        retargeters[side].calibrate(frame)
                        for side in sides
                    )
                    if calibrated:
                        state["hold"] = True
                        print("\n[CALIBRATION] both selected arms saved as VR neutral")
                        if frame is not None and pos_runner.calibrate(frame):
                            print("[CALIBRATION] 위치 방식 기준도 저장 (몸 틀 + 손목 0)")
                            state["start_block"] = ""
                            if not args.no_start_check and state["retarget"] in ("pos", "hand"):
                                chk = pos_runner.start_check([frame], state["retarget"] == "hand",
                                                             args.start_gap_min_cm, tuple(args.start_scale_range))
                                if not chk["ok"]:
                                    state["start_block"] = str(chk["reason"])
                                    print(f"[시작 확인] 🔴 {chk['reason']} — p(POSITION) 를 막는다. 바르게 하고 c 를 다시")
                                else:
                                    print(f"[시작 확인] 통과 — 두 손목 좌우 간격 {chk['gap_cm']:+.0f} cm")
                        else:
                            print("[CALIBRATION] 🔴 위치 방식 기준 실패 — 본이 모자란다. v 로 전환해도 pos 는 안 돈다")
                    else:
                        print("\n[CALIBRATION] failed; active frame or arm joints are missing")

            if replay is not None and seq is None and state["position"]:       # 작업 지시 14 C: 재생은 추적·보정 없이 위치 모드로 (첫 자세로 천천히)
                state["position"] = False
                replay.on_position(motor)
                if motor.position_mode:
                    state["hold"] = False
            if replay is not None and seq is None and state.pop("replay_go", False):
                if replay.phase in ("ready", "paused"):
                    state["hold"] = False
                replay.on_go()

            if state["position"]:
                state["position"] = False
                if not all(retargeters[side].neutral is not None for side in sides):
                    print("\n[POSITION] blocked; press c with valid tracking first")
                elif not tracking_ok:
                    print("\n[POSITION] blocked; no fresh complete Unreal frame")
                elif state.get("start_block"):
                    print(f"\n[POSITION] 막음 (시작 확인): {state['start_block']} — 바르게 하고 c 를 다시")
                elif motor.enter_position_from_current():
                    state["hold"] = False
                    timed_out = False

            if state["hold_request"]:
                state["hold_request"] = False
                state["hold"] = True
                print("\n[HOLD] current motor targets held")

            if state["damping"]:
                state["damping"] = False
                state["hold"] = True
                motor.enter_damping()

            if state["cycle_sy_policy"]:
                state["cycle_sy_policy"] = False
                order = list(PR.SY_POLICIES) if PR is not None else ["hold"]
                nxt = order[(order.index(pos_runner.sy_policy) + 1) % len(order)]
                pos_runner.set_policy(nxt)
                names = {"hold": "C 유지(현행)", "none": "A 계산값 그대로", "recenter": "B 기준 복귀",
                         "bone": "D 뼈 긴축 회전"}
                print(f"\n[비틀기] 팔 편 상태 처리 → {names.get(nxt, nxt)}")

            if state["cycle_speed"]:
                state["cycle_speed"] = False
                steps = [2.5, 3.5, 4.5]
                cur = min(steps, key=lambda v: abs(v - motor.max_speed))
                motor.max_speed = steps[(steps.index(cur) + 1) % len(steps)]
                print(f"\n[속도] 목표 속도 제한 {motor.max_speed:.1f} rad/s "
                      f"(빠를수록 따라오지만 발산 위험도 커진다. h 로 언제든 멈춘다)")

            if state["toggle_vel_ff"]:
                state["toggle_vel_ff"] = False
                motor.vel_ff = not motor.vel_ff
                motor.reset_velocity()
                print(f"\n[FF] 속도 피드포워드 {'켬' if motor.vel_ff else '끔'} "
                      f"(목표 속도 = 명령 변화율, ±{motor.max_speed:.1f} rad/s 로 제한)")

            if state["toggle_retarget"]:
                state["toggle_retarget"] = False
                if state["retarget"] == "bone" and not pos_runner.ready():
                    print("\n[RETARGET] pos 로 못 바꾼다 — c 로 캘리브레이션부터 (위치 방식 기준 없음)")
                else:
                    state["retarget"] = (("hand" if args.retarget == "hand" else "pos")
                                         if state["retarget"] == "bone" else "bone")
                    state["hold"] = True
                    print(f"\n[RETARGET] {state['retarget'].upper()} 방식으로 전환. "
                          f"목표가 튀지 않도록 HOLD 했다 — 자세를 확인하고 h 를 풀거나 p 를 다시 눌러라")

            if state["toggle_chest"]:
                state["toggle_chest"] = False
                new_mode = not next(iter(retargeters.values())).chest_relative
                for retargeter in retargeters.values():
                    retargeter.chest_relative = new_mode
                print(f"\n[CHEST] 가슴 기준 상대화 {'ON' if new_mode else 'OFF'} "
                      f"(중립은 두 기준 모두 저장되어 있어 재캘리브레이션이 필요 없다)")

            # Chest 가 한 번도 안 온 경우(구 언리얼 빌드)를 조용히 넘기지 않는다.
            # 조용히 폴백하면 "패치했는데 왜 그대로지?" 가 된다.
            # 🔴 프레임을 실제로 **처리한 뒤에만** 판정한다. chest_seen 은 _smoothed_joints()
            # 에서 세워지므로, c 를 누르기 전에는 항상 False 라 오경보가 난다.
            if (not warned_no_chest and tracking_ok
                    and any(r.chest_relative and r.filtered is not None and not r.chest_seen
                            for r in retargeters.values())
                    and frame is not None and now - frame.received_at < UDP_TIMEOUT_SEC):
                warned_no_chest = True
                print(f"\n[WARN] '{CHEST_JOINT_NAME}' 본이 스트림에 없다. 월드 기준으로 동작한다 "
                      "(어깨가 몸통 회전을 따라간다). 언리얼을 2026-09-11 이후 빌드로 갱신하라.")

            vr_kind = None
            if vr_link is not None:        # 작업 지시 14-c VR 끊김 감시 (명령에는 안 쓴다)
                vr_kind = vr_link.update(now, frame, frame is not None and all(retargeters[side].has_required_joints(frame) for side in sides))
                if seq is None:
                    if motor.position_mode != vr_pos_on:
                        vr_pos_on = motor.position_mode
                        if motor.position_mode:
                            vr_link.set_ref(now)   # 머리 기준 = 모터를 켤 때
                    if vr_loss is None and vr_kind and motor.position_mode and end_ret is None and stall_at is None:
                        vr_loss = {"t0": now, "park": None}
                        beeper.play("alarm", text="VR 끊김: " + vr_link.text())
                        print(f"\n[VR 끊김] ⏸ {vr_link.text()} — 그 자리 유지. {AT.VR_GRACE_S:g} 초 안에 돌아오면 이어서, 넘으면 주차 → 감쇠 → 종료")

            if auto is not None and not auto.finished and end_ret is None and vr_loss is None:
                auto.step(now, {"tracking_ok": tracking_ok, "frame": frame, "state": state,
                                "motor": motor, "retargeters": retargeters,
                                "pos_runner": pos_runner, "robot_ref": robot_ref, "dt": dt})
                if auto.finished:
                    state["hold"] = True
                    motor.enter_damping()
                    written = auto.write_csv()
                    if written:
                        print(f"[AUTO] 구간 표시 저장: {written}")
                    state["quit"] = True

            if end_gesture is not None and end_ret is None and vr_loss is None:      # 작업 지시 14-b 종료 동작: 사람을 따라가는 동안에만 본다
                if motor.position_mode != end_gesture.pos_on:
                    end_gesture.pos_on = motor.position_mode
                    if motor.position_mode:
                        end_gesture.set_park(motor.base_target)  # 주차 = 모터를 켠 자세 (--start-pose table: 판 위)
                if (motor.position_mode and tracking_ok and not state["hold"]
                        and end_gesture.update(now, motor.command_target, gripper.state() if gripper is not None else None)):
                    end_ret = {"t0": now, "from": {side: dict(motor.command_target[side]) for side in sides}}
                    beeper.play("gesture")
                    print(f"\n[자동 종료] 종료 동작 — 처음 자리 {end_gesture.dist:.1f} cm, {AT.END_HOLD_S:g} 초 가만히. 사람 입력을 끊고 주차로 {AT.END_RETURN_S:g} 초 → 감쇠")

            if seq is not None:            # 작업 지시 14-b: 자동 순서가 목표 (안전장치는 update_targets·send_targets 그대로)
                seq_target = seq.step(now, dt, motor, state, meas_ok, vr_kind)
                if seq_target is not None:
                    motor.update_targets({side: {joint: seq_target[side][joint] - motor.base_target[side][joint]
                                                 for joint in JOINT_ORDER} for side in sides}, dt)
                    seq.apply_grip(gripper)
                if seq.finished:
                    exit_code = seq.exit_code
                    state["quit"] = True
            elif replay is not None:       # 작업 지시 14 C: 녹화 궤적이 목표 (안전장치는 update_targets·send_targets 그대로)
                replay_target = replay.step(dt, motor, state["hold"])
                if replay_target is not None:
                    motor.update_targets({side: {joint: replay_target[side][joint] - motor.base_target[side][joint]
                                                 for joint in JOINT_ORDER} for side in sides}, dt)
                    replay.apply_grip(gripper)
            elif end_ret is not None:      # 작업 지시 14-b 종료 동작: 모터를 켠 자세로 (사람 입력 안 씀)
                if not motor.position_mode:
                    print("\n[자동 종료] 🔴 복귀 중 위치 모드가 풀렸다 — 끝낸다")
                    exit_code = 3
                    state["quit"] = True
                else:
                    a_end = min((now - end_ret["t0"]) / AT.END_RETURN_S, 1.0)
                    k_end = 1.0 - a_end * a_end * (3.0 - 2.0 * a_end)
                    if park_relax is not None:                     # 14-m (켤 때만)
                        park_relax["allow"] = _park_allow() if a_end >= 0.5 else None
                    motor.update_targets({side: {joint: (end_ret["from"][side][joint] - motor.base_target[side][joint]) * k_end
                                                 for joint in JOINT_ORDER} for side in sides}, dt)
                    if a_end >= 1.0:
                        state["hold"] = True
                        motor.enter_damping()
                        beeper.play("done")
                        print("\n[자동 종료] ✅ 주차로 돌아와 힘을 뺐다 — 워치독을 끝낸다 (정상 종료). 다시 하려면 같은 명령을 다시 실행")
                        state["quit"] = True
            elif vr_loss is not None:      # 작업 지시 14-c: 끊김 — 목표를 안 바꿔 그 자리 유지 → 5 초 → 주차 → 감쇠 (걸림 감시가 먼저)
                if stall_at is not None:
                    pass
                elif not motor.position_mode:
                    print("\n[VR 끊김] 🔴 위치 모드가 풀렸다 (발산 감시 등) — 끝낸다")
                    exit_code = 3
                    state["quit"] = True
                elif vr_loss["park"] is None:
                    if vr_kind is None:
                        print(f"\n[VR 끊김] ▶ 돌아옴 ({now - vr_loss['t0']:.1f} 초) — 이어서 따라간다 (속도 제한 안에서 손으로 다시 맞춘다)")
                        vr_loss = None
                    elif now - vr_loss["t0"] >= AT.VR_GRACE_S:
                        vr_loss["park"] = {"t0": now, "from": {side: dict(motor.command_target[side]) for side in sides}}
                        print(f"\n[VR 끊김] 🔴 {AT.VR_GRACE_S:g} 초 넘음 ({vr_link.text()}) — 사람 입력을 끊고 주차로 {AT.VR_PARK_S:g} 초 → 감쇠 → 종료")
                else:
                    a_vr = min((now - vr_loss["park"]["t0"]) / AT.VR_PARK_S, 1.0)
                    k_vr = 1.0 - a_vr * a_vr * (3.0 - 2.0 * a_vr)
                    if park_relax is not None:                     # 14-m (켤 때만)
                        park_relax["allow"] = _park_allow() if a_vr >= 0.5 else None
                    motor.update_targets({side: {joint: (vr_loss["park"]["from"][side][joint] - motor.base_target[side][joint]) * k_vr
                                                 for joint in JOINT_ORDER} for side in sides}, dt)
                    if a_vr >= 1.0:
                        state["hold"] = True
                        motor.enter_damping()
                        beeper.play("done")
                        print("\n[VR 끊김] 주차로 돌아와 힘을 뺐다 — 종료 (코드 3)")
                        exit_code = 3
                        state["quit"] = True
            elif auto is not None and auto.stage == "ret":
                pass                       # 복귀 중에는 사람 입력을 쓰지 않는다 (_stage_ret 이 목표를 만든다)
            elif tracking_ok and not state["hold"] and motor.position_mode:
                assert frame is not None
                if handover is not None:      # 작업 지시 10: 직전 프레임의 쌍 매핑 w·그리퍼·두 팔 거리로 예외 길이를 정한다
                    hl = pos_runner.ht.last if pos_runner.ht is not None else {}
                    ext = handover.update(now, float(hl.get("pair_w", float("nan"))),
                                          gripper.state() if gripper is not None else None, motor.last_arm_gap_full,
                                          max(motor.arm_gap_min, args.hand_gap_goal_cm / 100.0))
                    motor.handover_ext = ext
                    if pos_runner.ht is not None:   # 손 목표 쪽 집게 문턱은 여유만큼 크게 → 모터 쪽은 받침으로만
                        pos_runner.ht.handover_ext = ext
                        pos_runner.ht.handover_finger_min = motor.handover_finger_min + HANDOVER_FINGER_CUSHION_CM / 100.0
                if state["retarget"] in ("pos", "hand") and pos_runner.ready():
                    for side in sides:
                        retargeters[side].compute_deltas(frame)   # 로그 대조군 (명령에는 안 쓴다)
                    targets = pos_runner.targets(frame, hand=state["retarget"] == "hand")
                    deltas = (None if targets is None else
                              {side: {joint: targets[side][joint] - motor.base_target[side][joint]
                                      for joint in JOINT_ORDER}
                               for side in sides})
                    deltas = deltas if deltas is not None else {side: None for side in sides}
                else:
                    pos_runner.observe(frame)                     # 로그용 (명령에는 안 쓴다)
                    deltas = {
                        side: retargeters[side].compute_deltas(frame)
                        for side in sides
                    }
                if all(value is not None for value in deltas.values()):
                    motor.update_targets(deltas, dt)  # type: ignore[arg-type]
                    timed_out = False
                # REPORT_51: 그리퍼는 팔이 사람을 따라가는 동안에만, 새 패킷마다 판단 (그 밖에는 직전 상태 유지)
                if gripper is not None and frame.received_at != gripper_last_t:
                    gripper_last_t = frame.received_at
                    gripper.update({"grip": frame.grip}, frame.received_at)
                    if grip_rule_kw:                    # 작업 지시 14-p: 판정 이유가 바뀔 때 한 줄
                        grule = gripper.rules()
                        for gh in ("L", "R"):
                            if grule[gh] != grip_rule_prev[gh] and grule[gh] != 0:
                                gp = gripper.pinch()[gh]
                                gtxt = {1: "열림 — 손가락 뗌", 2: f"열림 — 중간값 {args.grip_mid_open_s:.1f}초 이어짐",
                                        3: f"인계 보호 — 반대 손이 연 뒤 {args.grip_handover_guard_s:.0f}초 안이라 중간값으로 안 엶"}[grule[gh]]
                                print(f"\n[그리퍼] {gh} {gtxt} (값 {gp if gp is not None else '-'})")
                        grip_rule_prev = grule
            elif frame_age > UDP_TIMEOUT_SEC and motor.position_mode and not timed_out:
                print("\n[SAFETY] Unreal UDP timeout -> DAMPING")
                state["hold"] = True
                motor.enter_damping()
                timed_out = True

            divergence = motor.send_targets()
            if divergence is not None:
                d_side, d_joint, d_device, d_cmd, d_meas = divergence
                print(
                    f"\n[SAFETY] divergence: {d_side}.{d_joint} (motor {d_device}) "
                    f"cmd={d_cmd:+.3f} measured={d_meas:+.3f} "
                    f"diff={abs(d_meas - d_cmd):.3f}rad -> DAMPING"
                )
                state["hold"] = True

            if stall is not None:          # 작업 지시 14-b 걸림 감시 (실제 각이 있을 때만 — dry-run 은 없다)
                if not motor.position_mode or not meas_ok:
                    stall.reset()
                elif stall_at is None and (seq is None or seq.stage not in ("stopping", "end")):
                    hit = stall.update(now, motor.command_target, motor.last_measured)
                    if hit is not None:
                        if seq is not None:
                            seq.abort(now, "걸림 — " + AT.stall_text(hit), motor, meas_ok)
                        else:
                            AT.hold_at_measured(motor)
                            state["hold"] = True
                            stall_at = now
                            beeper.play("alarm", text="걸림: " + AT.stall_text(hit))
                            print(f"\n[걸림] 🔴 {AT.stall_text(hit)} → 실제 자세에 멈춤, {AT.STALL_DAMP_AFTER_S:g} 초 뒤 감쇠·종료")
                if stall_at is not None and now - stall_at >= AT.STALL_DAMP_AFTER_S:
                    motor.enter_damping()
                    exit_code = 3
                    state["quit"] = True

            if robot_viz is not None:      # VR 표시: 명령·발산 감시가 끝난 뒤 읽기만 (주기는 RobotViz 가 판단)
                robot_viz.tick(now, motor, pos_runner, frame, frame_bones if pos_runner.heading is None else pos_runner.view_bones,
                               hold=state["hold"], tracking_ok=tracking_ok, gripper=gripper)
            if seq is not None:            # 기록된 캔 자세를 RVC1 로 (--robot-viz 가 있을 때만, 읽기만)
                seq.tick_can(now, motor)
            elif replay is not None:
                replay.tick_can(now, motor)
            if vgrip is not None:          # 14-j: 실제 속도의 가상 그리퍼 (다 닫힘·다 열림 소리, 가상 캔이 본다)
                vgrip.update(now)
            if vcan is not None:           # 가상 캔: 위와 같이 읽기만 (실패해도 루프를 멈추지 않는다)
                vcan.tick(now, motor.command_target, vgrip if vgrip is not None else gripper, state, running=motor.position_mode and not state["hold"])

            if log_writer is not None and now - last_log >= 0.05:
                last_log = now
                row = [
                    f"{now:.3f}",
                    "POS" if motor.position_mode else "DAMP/IDLE",
                    "1" if tracking_ok else "0",
                    "1" if state["hold"] else "0",
                    state["retarget"],
                    f"{pos_runner.last_heading:.2f}",
                    (auto.phase if auto is not None else "-"),
                    pos_runner.sy_policy,
                    "1" if motor.vel_ff else "0",
                ]
                for side in sides:
                    swing_raw = retargeters[side].last_swing_raw
                    swing_supp = retargeters[side].last_swing_suppressed
                    swing_world = retargeters[side].last_swing_world
                    swing_chest = retargeters[side].last_swing_chest
                    row += [
                        f"{swing_raw['roll']:.4f}", f"{swing_raw['yaw']:.4f}",
                        f"{swing_supp['roll']:.4f}", f"{swing_supp['yaw']:.4f}",
                        retargeters[side].chest_status(),
                        f"{swing_world['roll']:.4f}", f"{swing_world['yaw']:.4f}",
                        f"{swing_world['pitch']:.4f}",
                        f"{swing_chest['roll']:.4f}", f"{swing_chest['yaw']:.4f}",
                        f"{swing_chest['pitch']:.4f}",
                    ]
                    obs = pos_runner.last.get(side, (float("nan"),) * 5)
                    ex = pos_runner.extra.get(side, {})
                    row += [f"{obs[0]:.4f}", f"{obs[1]:.4f}", f"{obs[2]:.4f}",
                            f"{obs[3]:.1f}", f"{obs[4]:.2f}",
                            f"{ex.get('wrist_pron_deg', float('nan')):.1f}",
                            f"{ex.get('wrist_target', float('nan')):.4f}",
                            f"{ex.get('sy_ik', float('nan')):.1f}",
                            f"{ex.get('arm_twist_deg', float('nan')):.1f}",
                            (f"{math.degrees(pos_runner.rts[side].bone_offset):.1f}"
                             if side in pos_runner.rts and pos_runner.rts[side].bone_offset is not None
                             else "nan")]
                    for joint in JOINT_ORDER:
                        row.append(f"{motor.command_target[side][joint]:.4f}")
                        row.append(f"{motor.last_measured[side][joint]:.4f}")
                        row.append(f"{motor.last_desired[side][joint]:.4f}")
                        row.append(f"{(motor.vel_target[side][joint] if motor.vel_ff else 0.0):.4f}")
                row += [f"{motor.last_arm_gap * 100:.1f}", str(motor.gap_blocks), str(motor.yaw_holds)]
                row += [f"{pos_runner.hand_info.get(k, float('nan')):.1f}"
                        for k in ("human_wrist_dist_cm", "human_gap_cm", "robot_gap_target_cm", "hand_widen_cm")]
                row += [str(pos_runner.flip_holds())]
                row += [str(pos_runner.refits)]
                if args.hand_pair == "on":
                    hl = pos_runner.ht.last if pos_runner.ht is not None else {}
                    row += [f"{hl.get('pair_w', float('nan')):.3f}", f"{hl.get('pair_dist_cm', float('nan')):.1f}"]
                if gripper is not None:
                    gst, gpi = gripper.state(), gripper.pinch()
                    row += [str(gst["L"]), str(gst["R"])] + [
                        (f"{gpi[h]:.2f}" if gpi[h] is not None else "nan") for h in ("L", "R")]
                    if grip_rule_kw:
                        grl = gripper.rules()
                        row += [str(grl["L"]), str(grl["R"])]
                if vgrip is not None:
                    va, vb = vgrip.angles_now(now)
                    ve = vgrip.state()
                    row += [str(va), str(vb), str(ve["L"]), str(ve["R"])]
                if robot_viz is not None:            # 로봇 base cm, 마지막으로 보낸 패킷 값 (아직 없으면 nan)
                    for side in sides:
                        lv = robot_viz.last.get(side)
                        row += ([f"{v:.2f}" for v in lv["tip_cmd_cm"]] + [f"{v:.2f}" for v in lv["tip_meas_cm"]]
                                + [str(lv["flags"])]) if lv else ["nan"] * 6 + ["0"]
                if handover is not None:
                    row += [f"{motor.handover_ext / handover.full_m:.2f}", f"{motor.last_arm_gap_full * 100:.1f}",
                            f"{motor.last_arm_gap_finger * 100:.1f}"]
                if table is not None:                # 두 팔 중 가까운 쪽, 판까지 거리 (여유 빼지 않음)
                    row += [f"{min(motor.table_fn(s, motor.command_target[s])['min'] for s in sides) * 100:.2f}", str(motor.table_blocks)]
                if args.heading_deadband_deg is not None:
                    hb = pos_runner.heading
                    row += [f"{float('nan') if hb is None else hb.deg(False):.2f}", f"{float('nan') if hb is None else hb.deg(True):.2f}"]
                if vcan is not None:
                    row += vcan.csv_row()
                if replay is not None:
                    rp = seq.track if seq is not None else replay
                    row += [rp.phase, f"{rp.tau:.3f}", str(rp.frame_at(rp.tau))]
                if seq is not None:
                    row += seq.csv_row()
                if stall is not None:
                    row += [f"{stall.level:.3f}"]
                if vr_link is not None:
                    row += [vr_kind or "ok", f"{vr_link.head_horiz_now:.1f}", f"{vr_link.head_drop_now:.1f}",
                            "-" if vr_loss is None else ("park" if vr_loss["park"] else "hold")]
                if end_gesture is not None:
                    row += [f"{end_gesture.dist:.2f}", f"{end_gesture.vel:.2f}", "1" if end_gesture.armed else "0",
                            f"{0.0 if end_gesture.since is None else now - end_gesture.since:.2f}", "0" if end_ret is None else "1"]
                log_writer.writerow(row)
                log_file.flush()

            if now - last_print >= 0.2:
                last_print = now
                if state["calibrate_requested_at"] is not None:
                    remaining = max(0.0, state["calibrate_delay"] - (now - state["calibrate_requested_at"]))
                    print(f"[CALIBRATE] hold still... {remaining:.1f}s          ", end="\r", flush=True)
                else:
                    mode = "POS" if motor.position_mode else "DAMP/IDLE"
                    track = "VR_OK" if tracking_ok else "VR_WAIT"
                    hold = "HOLD" if state["hold"] else "RUN"
                    source = "-" if frame is None else frame.source[0]
                    summaries = []
                    for side in sides:
                        values = motor.command_target[side]
                        summaries.append(
                            f"{side[0].upper()}:"
                            + ",".join(f"{values[joint]:+.2f}" for joint in JOINT_ORDER)
                        )
                    age_text = "inf" if math.isinf(frame_age) else f"{frame_age:.2f}"
                    # chest=OK 가슴 유효 / STALE 마지막 유효값 사용 중 / NONE 본이 온 적 없음 / OFF 꺼 둠
                    chest_text = "/".join(
                        sorted({retargeters[side].chest_status() for side in sides})
                    )
                    auto_text = auto.status_text(now) if (auto is not None and not auto.finished) else ""
                    print(
                        auto_text +
                        f"[{mode} {track} {hold}] chest={chest_text} src={source} age={age_text}s "
                        + " ".join(summaries),
                        end="\r",
                        flush=True,
                    )

            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        state["quit"] = True
        receiver.stop()
        motor.stop()
        if vcan is not None:
            vcan.close(time.monotonic())     # 진행 중 에피소드는 abort 로 남긴다
        if gripper is not None:
            gripper.close()          # 그리퍼는 그대로 둔다 (열지 않는다)
        if log_file is not None:
            log_file.close()
            print(f"\nLog saved: {args.log_csv}")
        if rx_log is not None:
            print(f"\nRaw rx saved: {rx_log.path} ({rx_log.rows} lines, dropped {rx_log.dropped})")
        if robot_viz is not None:
            print(f"\nVR 표시 패킷 {robot_viz.sent}개 보냄" + (f" (마지막 오류: {robot_viz.error})" if robot_viz.error else ""))
        if vcan is not None:
            print(f"\n가상 캔: RVC1 {vcan.sent}개 보냄, 에피소드 {len(vcan.results)}개, bump {vcan.bump_total}" + (f" (마지막 오류: {vcan.error})" if vcan.error else ""))
        print("\nStopped.")
    if exit_code:                    # 작업 지시 14-b: 비상 멈춤 (걸림 등) 은 3. 옵션을 끄면 늘 0 그대로
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
