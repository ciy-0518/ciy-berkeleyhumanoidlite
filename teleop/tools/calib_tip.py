#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""집게 위치·방향 실측 (작업 지시 14-r, `arm_pose_logger.py --part T`).

왜: 14-q (REPORT_real_handover) 에서 실기 성공 자세를 모델로 보면 집게 끝이 L1·R1 보다 앞으로 2~4 cm,
    인계 때 R ↔ L 셋째 축이 26°·4~5 cm 어긋나 보였다. 기록만으로는 원인 (집게 길이? 자리 좌표? 잡는 깊이? 집게 영점? 캔 돎?) 을 못 가른다.
어떻게: 모터는 힘 뺀 채 (IDLE·DISABLED, 로거는 CAN 에 읽기만) 사람이 팔을 옮기고, 그리퍼만 이 도구가 연다/닫는다 (config/gripper.json).
    단계마다 2 초 평균 관절값 → 워치독·robot_viz 와 같은 모델 (motor_to_urdf_fn + virtual_can 집게 틀, 지금 robot_ref) 로 집게 끝·축을 계산해 바로 보여 준다.

단계 (캔 Ø5.75, 판 위 십자 표시. 모두 2 회):
  T1 L1 잡기      캔을 L1 십자에 세움 → L 집게를 평소처럼 대고 space → L 닫고 기록 → L 엶
  T2 R1 잡기      같은 방법으로 R
  T3 끝 대기      집게를 닫은 채 집게 끝 가운데를 십자 중심에 살짝 댐. L1 (L), R1 (R): 평소 잡는 방향으로 비스듬히,
                  H1 (판 가운데 23, 0 — L·R 각각): 되도록 똑바로 아래로. 손 축 방향이 달라야 "집게 길이" 와 "자리 좌표" 가 갈린다
  T4 인계         a) L1 의 캔을 L 로 잡음 (캔 윗면의 지름 선을 L 손 축 방향과 나란히 돌려 둔 뒤) → space → L 닫고 기록
                  b) 가운데로 들어 R 집게를 같은 축으로 대고 space → R 닫고 기록
                  c) 옆에서 (L 집게 면을 정면으로) 볼 때 캔 윗면 선과 L 집게 긴 변이 이루는 각 → 0/1/2/3 키 (0/15/30/45°)
                     (캔이 L 집게 안에서 집게 축 둘레로 돈 각. 처음에 나란히 두었으니 지금 각 = 돈 양)
                  d) 캔을 손으로 잡고 space → 두 집게 엶

계산 (cm, base 틀 x 앞 y 왼 z 위):
  집게 틀 h 손 축, j 집게가 닫히는 축, n 셋째 축. 가운데 = 끝에서 h 뒤로 1 cm (virtual_can GRIP_CENTER_CM).
  T1·T2: 캔 축 (자리 좌표) 과 가운데의 앞뒤·좌우·높이, 끝에서 캔 축까지 깊이 s (집게 5.5 − 반지름 2.9 = 2.6 보다 크면 물리적으로 불가능),
         n ↔ 캔 축, j ⟂ 캔 어긋남, "집게 영점" = j 가 캔 축에 직각이 되도록 손 축 둘레로 돌릴 각 (손별 후보).
  T3: 집게 끝 (가로 = 끝 가운데, 높이 = 닫힌 끝면의 가장 낮은 모서리) − 십자 중심.
      e_k = d + ΔL_손·h_k 로 맞춤 → d = 자리 좌표 차 (점마다 같은 방향), ΔL = 집게 길이 차 (손 축 방향).
  T4: R ↔ L 셋째 축 각·축 거리, R 집게 영점 후보 (n_R 을 n_L 에 맞추는 손 축 둘레 각),
      캔이 L 안에서 돈 양 (T4a 에 붙인 캔 축 vs L 이 잡은 점 → R 가운데 선), 사용자 눈대중 각.
판단 (judge): §4 작업 지시 그대로 — T3 ≈ 0 인데 T1 앞으로 → 잡는 깊이, T3 도 → 좌표 또는 길이 (맞춤으로 가름),
  T4 26° 재현 + 눈대중 0 → R 집게 영점, 눈대중 ≈ 모델 돈 양 → 캔 돎.
설정 파일은 바꾸지 않는다 (값만 제안). 출력 ~/cc_analysis/calib_T/T_<시각>.json·.md
"""
from __future__ import annotations

import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

TOOLS = Path(__file__).resolve().parent
LOWLEVEL = Path.home() / "code_workspace/Berkeley-Humanoid-Lite-main/Berkeley-Humanoid-Lite-Lowlevel"
for p in (str(TOOLS), str(LOWLEVEL)):
    if p not in sys.path:
        sys.path.insert(0, p)

import vr_control_watchdog_biarm as W      # noqa: E402  (모델: motor_to_urdf_fn, RobotRef)
import virtual_can as VCAN                 # noqa: E402  (집게 틀·잡기 판정)
import table_limit as TLIM                 # noqa: E402

SIDES = ("left", "right")
JOINTS = VCAN.JOINTS
HAND = {"left": "L", "right": "R"}
KOR = {"left": "왼팔 (L)", "right": "오른팔 (R)"}
TABLE = TOOLS.parent / "config/table.json"
OUT_DIR = Path.home() / "cc_analysis/calib_T"
GRIP_CM = VCAN.GRIP_M * 100.0               # 손목 관절 → 집게 끝 (모델)
END_HALF_J = VCAN.JAW_HALF_CLOSED_CM + VCAN.JAW_THICK_CM   # 닫힌 끝면 반폭 (집게 축 방향) 1.3
END_HALF_N = VCAN.JAW_HALF_WIDTH_CM                         # 끝면 반폭 (셋째 축) 0.85
SETTLE_S = 1.6           # 닫기 명령 → 다 닫힘 (57°: 1°/10 ms + 쉼 0.4 초 + 여유)
AVG_S = 2.0              # 기록 평균 시간
SHAKE_DEG = 0.6          # 평균 동안 관절 표준편차가 이보다 크면 "흔들림" 표시
ANGLE_KEYS = {"0": 0, "1": 15, "2": 30, "3": 45}
NEAR_CM = 1.0            # T3 ≈ 0 의 기준, 보정 후보로 내는 최소 크기
DEPTH_MAX_CM = VCAN.JAW_LEN_CM - 0.5 * 5.75   # 캔 축이 집게 끝에서 들어갈 수 있는 최대 깊이 2.6
JAW_MIN_DEG = 8.0        # 집게 영점 후보로 내는 최소 크기
AXIS_OK_DEG = 10.0       # T4 셋째 축 각이 이 안이면 "모델과 실제가 맞음"
ROT_SAME_DEG = 15.0      # 눈대중 각과 모델 돈 양이 이 안이면 "캔 돎"


# --------------------------------------------------------------------------- 모델
class Model:
    """워치독·robot_viz·virtual_can 과 같은 체인. grip_dcm (집게 길이 차), jaw_off_deg (손별 집게 영점), slot_shift (자리 좌표 차) 는
    흉내 시험·보정 후 다시 보기용 (기본 0 = 지금 모델 그대로)."""

    def __init__(self, ref, table_path=TABLE, grip_dcm=None, jaw_off_deg=None, slot_shift=(0.0, 0.0, 0.0), jaw_sign=1.0):
        self.ref = ref
        self.table = TLIM.TableLimit(Path(table_path), body="real")
        self.vc = VCAN.VirtualCan(self.table, "L1", ref, SIDES, W.motor_to_urdf_fn(ref, SIDES), margin_cm=VCAN.GRIP_MARGIN_CM,
                                  log=lambda s: None)
        self.grip_dcm = {s: float((grip_dcm or {}).get(s, 0.0)) for s in SIDES}
        self.jaw_off = {s: math.radians(float((jaw_off_deg or {}).get(s, 0.0))) for s in SIDES}
        self.slot_shift = np.array(slot_shift, dtype=float)
        self.jaw_sign = float(jaw_sign)
        self.radius = self.vc.radius

    def hand(self, side: str, c: Dict[str, float]) -> dict:
        """모터값 (rad, 로거·워치독 meas 와 같은 값) → 집게 틀. virtual_can.hand_of 와 같은 식 (+ 길이·영점 차)."""
        vc = self.vc
        q = vc.q_of(side, c)
        M = np.eye(4)
        el = None
        for n, O in zip(JOINTS, vc.models[side]._O):
            c_, s_ = math.cos(q[n]), math.sin(q[n])
            M = M @ O @ np.array([[c_, -s_, 0, 0], [s_, c_, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]])
            if n == "elbow_pitch":
                el = M[:3, 3].copy()
        wr, ax, jaw0 = M[:3, 3].copy(), M[:3, 2].copy(), M[:3, 1].copy()
        h = ax if float(np.dot(ax, wr - el)) > 0 else -ax
        a = self.jaw_sign * (c["elbow_roll"] - self.ref.e90[side]["elbow_roll"]) + self.jaw_off[side]
        j = jaw0 * math.cos(a) + np.cross(h, jaw0) * math.sin(a)
        n = np.cross(h, j)
        tip = wr * 100.0 + (GRIP_CM + self.grip_dcm[side]) * h
        return {"tip": tip, "h": h, "j": j, "n": n, "R": np.column_stack((h, j, n)), "wrist": wr * 100.0,
                "centre": tip - VCAN.GRIP_CENTER_CM * h}

    def slot(self, name: str) -> np.ndarray:
        """자리 밑면 가운데 (판 윗면 높이). 이름: L1 … R4, H1."""
        return self.vc._slot_base(name) + self.slot_shift


def contact_point(hd: dict) -> np.ndarray:
    """닫힌 끝면 네 모서리 중 가장 낮은 점 (판에 닿는 점)."""
    corners = [hd["tip"] + sj * END_HALF_J * hd["j"] + sn * END_HALF_N * hd["n"] for sj in (-1, 1) for sn in (-1, 1)]
    return min(corners, key=lambda p: p[2])


def ang(a, b) -> float:
    c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(min(abs(c), 1.0)))


def wrap90(a: float) -> float:
    """각 (°) 을 −90~90 으로 (집게는 180° 돌려도 같다)."""
    return (a + 90.0) % 180.0 - 90.0


def jaw_zero_vertical(hd: dict) -> float:
    """j 가 수직 캔 축에 직각이 되도록 손 축 둘레로 더할 각 (°). j(a) = j cos a + n sin a."""
    z = np.array([0, 0, 1.0])
    return wrap90(math.degrees(math.atan2(-float(hd["j"] @ z), float(hd["n"] @ z))))


def jaw_fit_to(hd: dict, target: np.ndarray):
    """n 을 target 과 나란하게 (부호 무관) 만드는 손 축 둘레 각 (°) 과 그때 남는 각. n(a) = n cos a − j sin a."""
    a = wrap90(math.degrees(math.atan2(-float(hd["j"] @ target), float(hd["n"] @ target))))
    r = math.radians(a)
    return a, ang(hd["n"] * math.cos(r) - hd["j"] * math.sin(r), target)


def line_dist(p, a, d) -> float:
    d = d / np.linalg.norm(d)
    v = p - a
    return float(np.linalg.norm(v - float(v @ d) * d))


# --------------------------------------------------------------------------- 단계
def steps() -> List[dict]:
    """grip = 기록 전에 닫을 손, pre_close = 단계 시작에 닫아 둘 손, open_after = 기록 뒤 열 손, angle = 눈대중 각을 받는다."""
    out = []
    for k in (1, 2):
        out.append(dict(id=f"T1-{k}", kind="grasp", side="left", slot="L1", grip=["left"], open_after=["left"], pre_open=True,
                        title=f"T1 L1 잡기 {k}/2",
                        text="캔을 L1 십자 한가운데에 세운다. 왼팔을 받쳐 L 집게를 평소 잡는 높이·방향으로 캔에 댄다 (집게 사이에 캔). "
                             "팔을 받친 채 space → L 이 닫히고 2 초 기록 (삑 소리까지 움직이지 않는다) → 기록 뒤 L 이 열린다. "
                             + ("다음 회는 팔을 한 번 뗐다가 다시 댄다." if k == 1 else "")))
    for k in (1, 2):
        out.append(dict(id=f"T2-{k}", kind="grasp", side="right", slot="R1", grip=["right"], open_after=["right"], pre_open=True,
                        title=f"T2 R1 잡기 {k}/2",
                        text="캔을 R1 십자 한가운데에 세운다. 오른팔로 T1 과 같게. space → R 닫힘·기록 → R 열림."))
    for side, slot, how in (("left", "L1", "평소 잡는 방향으로 비스듬히 (T1 처럼 손이 앞·안쪽을 향하게)"),
                            ("right", "R1", "평소 잡는 방향으로 비스듬히 (T2 처럼)"),
                            ("left", "H1", "되도록 똑바로 아래로 (집게가 판에 수직)"),
                            ("right", "H1", "되도록 똑바로 아래로")):
        for k in (1, 2):
            out.append(dict(id=f"T3-{slot}-{HAND[side]}-{k}", kind="touch", side=side, slot=slot, grip=[], pre_close=[side],
                            open_after=[], title=f"T3 집게 끝 대기 {KOR[side]} @ {slot} {k}/2",
                            text=f"캔은 치운다. {HAND[side]} 집게는 닫혀 있다. 닫힌 집게 끝 가운데를 {slot} 십자 중심에 살짝 댄다 — "
                                 f"{how}. 누르지 말고 닿기만. 팔을 받친 채 space → 2 초 기록."))
    for k in (1, 2):
        out.append(dict(id=f"T4-{k}a", kind="handover_a", side="left", slot="L1", grip=["left"], open_after=[], pre_open=True,
                        title=f"T4 인계 {k}/2 — a) L 로 잡기",
                        text="캔 윗면에 지름 선을 펜으로 그어 둔다 (처음 한 번). 캔을 L1 에 세우고, 윗면 선이 L 손 축 (손목 → 집게 끝 방향) 과 "
                             "나란하도록 캔을 돌린다. L 집게를 대고 space → L 닫힘·기록 (L 은 닫힌 채)."))
        out.append(dict(id=f"T4-{k}b", kind="handover_b", side="right", slot=None, grip=["right"], open_after=[], angle=True,
                        title=f"T4 인계 {k}/2 — b) R 로 마주 잡기",
                        text="L 로 캔을 쥔 채 가운데로 들어 올린다. R 집게를 캔의 같은 축으로 대고, 보기에 '제대로 마주 봄' 이 되면 "
                             "두 팔을 받친 채 space → R 닫힘·기록. 그다음 옆에서 (L 집게 면을 정면으로) 캔 윗면 선과 L 집게 긴 변 사이 각을 "
                             "0 (0°) / 1 (15°) / 2 (30°) / 3 (45°) 키로 고른다."))
        out.append(dict(id=f"T4-{k}c", kind="release", side="both", slot=None, grip=[], open_after=["left", "right"], record=False,
                        title=f"T4 인계 {k}/2 — c) 놓기",
                        text="캔을 손으로 잡는다. space → 두 집게가 열린다 (기록 없음)."))
    return out


# --------------------------------------------------------------------------- 계산
def joints_of(vals: Sequence[float], names) -> Dict[str, Dict[str, float]]:
    out = {s: {} for s in SIDES}
    for v, (s, j) in zip(vals, names):
        out[s][j] = float(v)
    return out


def eval_grasp(model: Model, q: dict, side: str, slot: str) -> dict:
    hd = model.hand(side, q[side])
    base = model.slot(slot)
    vc = model.vc
    vc.pos, vc.rot = base.copy(), np.eye(3)
    chk = vc.grip_check((hd["tip"], hd["R"]))
    c = hd["centre"]
    return {"centre": c.tolist(), "tip": hd["tip"].tolist(), "h": hd["h"].tolist(), "n": hd["n"].tolist(), "j": hd["j"].tolist(),
            "dx": float(c[0] - base[0]), "dy": float(c[1] - base[1]), "grip_h": float(c[2] - base[2]),
            "depth_s": float(chk["s"]), "j_off": float(chk["j"]), "t": float(chk["t"]),
            "depth_over": float(chk["s"] - DEPTH_MAX_CM),
            "n_vs_axis": ang(hd["n"], np.array([0, 0, 1.0])), "j_tilt": float(chk["tilt_deg"]),
            "jaw_zero": jaw_zero_vertical(hd), "check_ok": bool(chk["ok"]), "check_reason": chk["reason"]}


def eval_touch(model: Model, q: dict, side: str, slot: str) -> dict:
    hd = model.hand(side, q[side])
    base = model.slot(slot)
    cp = contact_point(hd)
    e = np.array([hd["tip"][0] - base[0], hd["tip"][1] - base[1], cp[2] - base[2]])
    return {"e": e.tolist(), "h": hd["h"].tolist(), "tip": hd["tip"].tolist(), "contact": cp.tolist(),
            "h_vs_down": ang(hd["h"], np.array([0, 0, -1.0]))}


def eval_handover(model: Model, qa: dict, qb: dict, angle_user: Optional[float]) -> dict:
    """qa = T4a (L 이 L1 의 선 캔을 잡은 때), qb = T4b (두 손)."""
    vc = model.vc
    ha = model.hand("left", qa["left"])
    vc.pos, vc.rot = model.slot("L1").copy(), np.eye(3)
    chk = vc.grip_check((ha["tip"], ha["R"]))
    z0 = np.array([0, 0, 1.0])
    P0 = vc.pos + chk["t"] * z0
    rel_p, rel_z = ha["R"].T @ (P0 - ha["tip"]), ha["R"].T @ z0
    L, R = model.hand("left", qb["left"]), model.hand("right", qb["right"])
    PL = L["tip"] + L["R"] @ rel_p
    zr = L["R"] @ rel_z
    line = R["centre"] - PL
    lh = line / np.linalg.norm(line)
    if lh @ zr < 0:
        lh = -lh
    rot = math.degrees(math.acos(min(1.0, float(lh @ zr))))
    a_fit, a_res = jaw_fit_to(R, zr)
    return {"axis_angle": ang(L["n"], R["n"]), "axis_dist": line_dist(R["centre"], L["centre"], L["n"]),
            "nR_vs_can": ang(R["n"], zr), "nL_vs_can": ang(L["n"], zr), "nL_vs_can_a": ang(ha["n"], z0),
            "ctr_R_minus_L": (R["centre"] - L["centre"]).tolist(), "R_jaw_fit": a_fit, "R_jaw_fit_left": a_res,
            "can_rot_model": rot, "can_rot_user": angle_user,
            "R_dist_to_rigid_axis": line_dist(R["centre"], PL, zr), "line_len": float(np.linalg.norm(line))}


def analyse(model: Model, recs: Dict[str, dict], names) -> dict:
    """recs: 단계 id → {"pos": [10], "std": [10], "angle": …}. 모든 단계 결과."""
    out = {"grasp": {}, "touch": {}, "handover": {}}
    by = {s["id"]: s for s in steps()}
    for sid, r in recs.items():
        st = by.get(sid)
        if st is None or r.get("pos") is None:
            continue
        q = joints_of(r["pos"], names)
        if st["kind"] == "grasp":
            out["grasp"][sid] = eval_grasp(model, q, st["side"], st["slot"])
        elif st["kind"] == "touch":
            out["touch"][sid] = dict(eval_touch(model, q, st["side"], st["slot"]), side=st["side"], slot=st["slot"])
        elif st["kind"] == "handover_b":
            a = recs.get(sid[:-1] + "a")
            if a and a.get("pos") is not None:
                out["handover"][sid[:-1]] = eval_handover(model, joints_of(a["pos"], names), q, r.get("angle"))
    return out


def fit_touch(touch: Dict[str, dict]) -> Optional[dict]:
    """e_k = d + ΔL_손 · h_k (최소제곱). 미지수 d (3) + ΔL_L·ΔL_R (기록이 있는 손만)."""
    if len(touch) < 2:
        return None
    sides = [s for s in SIDES if any(v["side"] == s for v in touch.values())]
    rows, rhs = [], []
    for v in touch.values():
        h = np.array(v["h"])
        for ax in range(3):
            row = [1.0 if k == ax else 0.0 for k in range(3)] + [h[ax] if v["side"] == s else 0.0 for s in sides]
            rows.append(row); rhs.append(v["e"][ax])
    A, b = np.array(rows), np.array(rhs)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    sv = np.linalg.svd(A, compute_uv=False)
    res = b - A @ x
    out = {"d": x[:3].tolist(), "dL": {s: float(x[3 + i]) for i, s in enumerate(sides)}, "rms": float(np.sqrt(np.mean(res ** 2))),
           "cond": float(sv[0] / max(sv[-1], 1e-9)), "n": len(touch)}
    # 길이만 / 좌표만 으로 맞춘 잔차 (어느 쪽이 더 설명하나)
    xa, *_ = np.linalg.lstsq(A[:, :3], b, rcond=None)
    xb, *_ = np.linalg.lstsq(A[:, 3:], b, rcond=None)
    out["rms_shift_only"] = float(np.sqrt(np.mean((b - A[:, :3] @ xa) ** 2)))
    out["rms_length_only"] = float(np.sqrt(np.mean((b - A[:, 3:] @ xb) ** 2)))
    return out


def _mean(vals):
    return float(np.mean(vals)) if vals else math.nan


def _spread(vals):
    return float(np.max(vals) - np.min(vals)) if len(vals) > 1 else 0.0


def judge(ref, recs: Dict[str, dict], names, table_path=TABLE) -> dict:
    """작업 지시 14-r §4 판단과 보정 후보. 반환 {"base": analyse(지금 모델), "fit", "cand", "corrected", "lines": [...]}"""
    base_model = Model(ref, table_path)
    A = analyse(base_model, recs, names)
    lines, cand = [], {}
    # ---- T3: 좌표 vs 길이
    T = A["touch"]
    fit = fit_touch(T)
    t3_max = max((float(np.linalg.norm(v["e"])) for v in T.values()), default=math.nan)
    t3_zero = bool(T) and t3_max < NEAR_CM
    if T:
        lines.append(f"T3 집게 끝 − 십자 중심: 가장 큰 차 {t3_max:.1f} cm ({len(T)} 회)")
        for sid, v in sorted(T.items()):
            e = v["e"]
            lines.append(f"  {sid}: 앞뒤 {e[0]:+.1f} 좌우 {e[1]:+.1f} 높이 {e[2]:+.1f} (손 축 ↔ 아래 {v['h_vs_down']:.0f}°)")
    shift = np.zeros(3)
    dL = {s: 0.0 for s in SIDES}
    if t3_zero:
        lines.append("→ T3 ≈ 0: 자리 좌표·집게 길이는 맞다")
    elif fit:
        d = np.array(fit["d"])
        lines.append(f"맞춤 (e = d + ΔL·h): d = 앞뒤 {d[0]:+.1f} 좌우 {d[1]:+.1f} 높이 {d[2]:+.1f} cm, "
                     + ", ".join(f"ΔL_{HAND[s]} {v:+.1f}" for s, v in fit["dL"].items())
                     + f" cm, 남는 차 {fit['rms']:.2f} cm (좌표만 {fit['rms_shift_only']:.2f}, 길이만 {fit['rms_length_only']:.2f}), 조건수 {fit['cond']:.0f}")
        if fit["cond"] > 50:
            lines.append("  ⚠ 손 축 방향이 비슷해 길이와 좌표를 잘 못 가른다 — H1 을 더 똑바로 아래로 다시 하면 좋다")
        if float(np.linalg.norm(d[:2])) >= NEAR_CM or abs(d[2]) >= NEAR_CM:
            shift = d.copy()
            cand["slot_shift_cm"] = {"dx": round(float(d[0]), 1), "dy": round(float(d[1]), 1), "dz": round(float(d[2]), 1)}
            lines.append(f"→ 자리 좌표: 모델 집게가 십자보다 (앞뒤 {d[0]:+.1f}, 좌우 {d[1]:+.1f}, 높이 {d[2]:+.1f}) 에 있다 — "
                         f"table.json 자리 (판) 를 이만큼 옮기면 모델과 맞는다 (점마다 같은 방향)")
        for s, v in fit["dL"].items():
            if abs(v) >= NEAR_CM:
                dL[s] = -v
                cand.setdefault("grip_len_cm", {})[HAND[s]] = round(-v, 1)
                lines.append(f"→ 집게 길이 {HAND[s]}: 모델이 {v:+.1f} cm {'길다' if v > 0 else '짧다'} (손 축 방향) — 집게 끝 {-v:+.1f} cm 후보")
    # ---- T1·T2: 잡기
    G = A["grasp"]
    jaw = {}
    for side, tag in (("left", "T1"), ("right", "T2")):
        g = [v for k, v in G.items() if k.startswith(tag)]
        if not g:
            continue
        dx, dy, gh, ds = [v["dx"] for v in g], [v["dy"] for v in g], [v["grip_h"] for v in g], [v["depth_s"] for v in g]
        jz = [v["jaw_zero"] for v in g]
        lines.append(f"{tag} {HAND[side]} 잡기 ({len(g)} 회): 가운데 − 캔 축 앞뒤 {_mean(dx):+.1f} (폭 {_spread(dx):.1f}) "
                     f"좌우 {_mean(dy):+.1f} (폭 {_spread(dy):.1f}), 잡는 높이 {_mean(gh):.1f}, 끝에서 캔 축 깊이 {_mean(ds):.1f} "
                     f"(가능 최대 {DEPTH_MAX_CM:.1f}), n ↔ 캔 {_mean([v['n_vs_axis'] for v in g]):.0f}°, "
                     f"j ⟂ 캔 어긋남 {_mean([v['j_tilt'] for v in g]):.0f}°, 집게 영점 {_mean(jz):+.0f}° (폭 {_spread(jz):.0f}°)")
        if abs(_mean(jz)) >= JAW_MIN_DEG and _spread(jz) < abs(_mean(jz)):
            jaw[side] = _mean(jz)
            cand.setdefault("jaw_zero_deg", {})[HAND[side]] = round(_mean(jz), 0)
            lines.append(f"→ {HAND[side]} 집게 영점 후보 {_mean(jz):+.0f}° (j 가 선 캔에 직각이 되게; robot_viz jaw_offset 처럼 더할 값)")
        if t3_zero and (_mean(ds) - DEPTH_MAX_CM >= NEAR_CM or _mean(dx) >= 2.0):
            cand.setdefault("grasp_depth_cm", {})[HAND[side]] = round(_mean(ds), 1)
            lines.append(f"→ T3 ≈ 0 인데 {tag} 가운데가 앞으로 {_mean(dx):+.1f} cm: 집게가 캔을 무는 깊이 문제 "
                         f"(모델 잡기 기준은 끝에서 {VCAN.GRIP_CENTER_CM:.0f} cm, 실제 {_mean(ds):.1f} cm 로 보임)")
    # ---- 보정 모델로 다시
    corr = None
    if cand.get("slot_shift_cm") or cand.get("grip_len_cm") or jaw:
        cm = Model(ref, table_path, grip_dcm=dL, jaw_off_deg=jaw, slot_shift=shift)
        corr = analyse(cm, recs, names)
        for side, tag in (("left", "T1"), ("right", "T2")):
            g = [v for k, v in corr["grasp"].items() if k.startswith(tag)]
            if g:
                lines.append(f"  보정 후 {tag}: 앞뒤 {_mean([v['dx'] for v in g]):+.1f} 좌우 {_mean([v['dy'] for v in g]):+.1f} "
                             f"깊이 {_mean([v['depth_s'] for v in g]):.1f} j ⟂ 캔 {_mean([v['j_tilt'] for v in g]):.0f}°")
    # ---- T4
    # 셋째 축 각 R ↔ L 은 14-q 와 같은 값이다. 다만 L 이 손을 숙여 선 캔을 잡으면 n_L 자체가 캔 축과 어긋나 있으므로 (14-q 실기 19~27°),
    # R 은 "L 이 쥔 캔 축" (T4a 에 붙인 캔을 L 과 함께 옮긴 축) 과 비교한다 — virtual_can 둘째 손 정렬과 같은 기준.
    H = A["handover"]
    if H:
        M_ = lambda key, src=H: _mean([v[key] for v in src.values()])
        users = [v["can_rot_user"] for v in H.values() if v["can_rot_user"] is not None]
        lines.append(f"T4 인계 ({len(H)} 회): R ↔ L 셋째 축 {M_('axis_angle'):.1f}° (폭 {_spread([v['axis_angle'] for v in H.values()]):.1f}, 14-q 실기 26°) "
                     f"— 그중 L 이 잡을 때 이미 숙인 각 (n_L ↔ 캔, T4a) {M_('nL_vs_can_a'):.0f}°")
        lines.append(f"  R 셋째 축 ↔ L 이 쥔 캔 축 {M_('nR_vs_can'):.1f}°, R 가운데 ↔ 그 축 {M_('R_dist_to_rigid_axis'):.1f} cm, "
                     f"모델에서 캔이 L 안에서 돈 양 {M_('can_rot_model'):.0f}°, 눈대중 {(_mean(users) if users else math.nan):.0f}°, "
                     f"R 집게 영점 맞춤 {M_('R_jaw_fit'):+.0f}°")
        if corr and corr["handover"]:
            C = corr["handover"]
            lines.append(f"  보정 후 T4: R ↔ 캔 축 {M_('nR_vs_can', C):.1f}°, R 가운데 ↔ 축 {M_('R_dist_to_rigid_axis', C):.1f} cm")
        u = _mean(users) if users else None
        if u is not None and u >= ROT_SAME_DEG and abs(M_("can_rot_model") - u) <= ROT_SAME_DEG:
            cand["can_rotates_in_L_deg"] = round(u, 0)
            lines.append(f"→ T4: 캔이 L 안에서 돎 (눈대중 {u:.0f}° ≈ 모델 {M_('can_rot_model'):.0f}°) — 집게 영점이 아니라 캔이 집게 축 둘레로 돈 것. "
                         f"판정 기준을 '집게 축 ⟂ 캔' 으로 바꾸는 쪽 (14-q §6)")
        elif M_("nR_vs_can") < AXIS_OK_DEG:
            why = (f"R ↔ L {M_('axis_angle'):.0f}° 는 L 이 캔을 숙여 잡은 각 ({M_('nL_vs_can_a'):.0f}°) 이다 — "
                   "모델과 실제가 맞고, 'n_L 을 캔 축으로 보는' 비교가 어긋나 보이게 한 것" if M_("axis_angle") >= AXIS_OK_DEG
                   else "R ↔ L 도 작다 — 14-q 의 26° 가 재현되지 않는다")
            lines.append(f"→ T4: R 이 L 이 쥔 캔 축과 맞다 ({M_('nR_vs_can'):.0f}°, {M_('R_dist_to_rigid_axis'):.1f} cm). {why}")
            if M_("R_dist_to_rigid_axis") >= 1.5:
                lines.append(f"  다만 R 가운데가 축에서 {M_('R_dist_to_rigid_axis'):.1f} cm — 위 위치 보정 (T3) 후보와 함께 본다")
        elif u is not None and u < ROT_SAME_DEG:
            cand.setdefault("jaw_zero_deg_T4", {})["R"] = round(M_("R_jaw_fit"), 0)
            lines.append(f"→ T4: R 셋째 축이 캔 축과 {M_('nR_vs_can'):.0f}° 어긋나고 캔은 돌지 않았다 (눈대중 {u:.0f}°) — "
                         f"R 집게 영점 후보 {M_('R_jaw_fit'):+.0f}° (T2 영점 {cand.get('jaw_zero_deg', {}).get('R', '없음')} 과 비교)")
        else:
            lines.append("→ T4: 눈대중 각과 모델 돈 양이 맞지 않는다 — 판단 보류 (다시 하거나 사진으로)")
    return {"base": A, "fit": fit, "cand": cand, "corrected": corr, "lines": lines}


def step_result_lines(model: Model, st: dict, recs: Dict[str, dict], names) -> List[str]:
    """한 단계를 기록한 직후 화면에 보일 줄."""
    r = recs.get(st["id"])
    if not r or r.get("pos") is None:
        return []
    q = joints_of(r["pos"], names)
    shake = max(r.get("std_deg") or [0.0])
    tail = f"   (2 초 동안 흔들림 {shake:.2f}°{' ⚠ 크다 — r 로 다시' if shake > SHAKE_DEG else ''})"
    if st["kind"] == "grasp":
        v = eval_grasp(model, q, st["side"], st["slot"])
        return [f"결과 {st['id']}: 집게 가운데 − 캔 축  앞뒤 {v['dx']:+.1f}  좌우 {v['dy']:+.1f}  잡는 높이 {v['grip_h']:.1f} cm" + tail,
                f"          끝에서 캔 축 깊이 {v['depth_s']:.1f} cm (가능 최대 {DEPTH_MAX_CM:.1f})  n ↔ 캔 {v['n_vs_axis']:.0f}°  "
                f"j ⟂ 캔 어긋남 {v['j_tilt']:.0f}°  집게 영점 {v['jaw_zero']:+.0f}°  가상 판정 {'통과' if v['check_ok'] else v['check_reason']}"]
    if st["kind"] == "touch":
        v = eval_touch(model, q, st["side"], st["slot"])
        e = v["e"]
        return [f"결과 {st['id']}: 집게 끝 − 십자 중심  앞뒤 {e[0]:+.1f}  좌우 {e[1]:+.1f}  높이 {e[2]:+.1f} cm  "
                f"(손 축 ↔ 아래 {v['h_vs_down']:.0f}°)" + tail]
    if st["kind"] == "handover_a":
        v = eval_grasp(model, q, "left", "L1")
        return [f"결과 {st['id']}: L 잡기 앞뒤 {v['dx']:+.1f} 좌우 {v['dy']:+.1f} 높이 {v['grip_h']:.1f} cm" + tail]
    if st["kind"] == "handover_b":
        a = recs.get(st["id"][:-1] + "a")
        if not a:
            return []
        v = eval_handover(model, joints_of(a["pos"], names), q, r.get("angle"))
        return [f"결과 {st['id']}: R ↔ L 셋째 축 {v['axis_angle']:.1f}° (L 이 잡을 때 숙인 각 {v['nL_vs_can_a']:.0f}°)  "
                f"R ↔ L 이 쥔 캔 축 {v['nR_vs_can']:.1f}°  R 가운데 ↔ 그 축 {v['R_dist_to_rigid_axis']:.1f} cm" + tail,
                f"          모델에서 캔이 L 안에서 돈 양 {v['can_rot_model']:.0f}°  R 집게 영점 맞춤 {v['R_jaw_fit']:+.0f}°"
                + ("" if r.get("angle") is None else f"  눈대중 {r['angle']:.0f}°")]
    return []


def write_outputs(out_dir: Path, ts: str, ref_path: Path, recs: Dict[str, dict], names, res: dict, meta: dict) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    js = out_dir / f"T_{ts}.json"
    md = out_dir / f"T_{ts}.md"

    def clean(o):
        if isinstance(o, dict):
            return {k: clean(v) for k, v in o.items()}
        if isinstance(o, (list, tuple)):
            return [clean(v) for v in o]
        if isinstance(o, (np.floating, np.integer)):
            return float(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
        return o
    data = {"time": datetime.now().isoformat(timespec="seconds"), "robot_ref": str(ref_path), "meta": meta,
            "joint_names": [list(n) for n in names], "records": recs, "result": res}
    js.write_text(json.dumps(clean(data), ensure_ascii=False, indent=1), encoding="utf-8")
    L = [f"# 집게 실측 T ({ts})", "", f"- robot_ref: `{Path(ref_path).name}`",
         f"- 그리퍼: {meta.get('gripper', '-')}", f"- 기록 {sum(1 for r in recs.values() if r.get('pos') is not None)} 단계", "",
         "## 판단", ""] + [f"- {x}" if not x.startswith("  ") else f"  - {x.strip()}" for x in res["lines"]] + [
         "", "## 보정 후보 (설정 파일은 바꾸지 않았다)", "", "```", json.dumps(res["cand"], ensure_ascii=False, indent=1), "```", "",
         "## 단계별 값", ""]
    A = res["base"]
    if A["grasp"]:
        L += ["| 단계 | 앞뒤 | 좌우 | 잡는 높이 | 깊이 s | n↔캔 | j⟂캔 | 집게 영점 | 가상 판정 |", "|---|---|---|---|---|---|---|---|---|"]
        for k, v in sorted(A["grasp"].items()):
            L.append(f"| {k} | {v['dx']:+.1f} | {v['dy']:+.1f} | {v['grip_h']:.1f} | {v['depth_s']:.1f} | {v['n_vs_axis']:.0f}° | "
                     f"{v['j_tilt']:.0f}° | {v['jaw_zero']:+.0f}° | {'통과' if v['check_ok'] else v['check_reason']} |")
        L.append("")
    if A["touch"]:
        L += ["| 단계 | 앞뒤 | 좌우 | 높이 | 손 축↔아래 |", "|---|---|---|---|---|"]
        for k, v in sorted(A["touch"].items()):
            e = v["e"]
            L.append(f"| {k} | {e[0]:+.1f} | {e[1]:+.1f} | {e[2]:+.1f} | {v['h_vs_down']:.0f}° |")
        L.append("")
    if A["handover"]:
        L += ["| 단계 | R↔L 셋째 축 | L 숙임 (T4a) | R↔캔 축 | R↔축 거리 | R−L 가운데 | 모델 돈 양 | 눈대중 | R 영점 맞춤 |", "|---|---|---|---|---|---|---|---|---|"]
        for k, v in sorted(A["handover"].items()):
            user = "-" if v["can_rot_user"] is None else f"{v['can_rot_user']:.0f}°"
            L.append(f"| {k} | {v['axis_angle']:.1f}° | {v['nL_vs_can_a']:.0f}° | {v['nR_vs_can']:.1f}° | {v['R_dist_to_rigid_axis']:.1f} | {np.round(v['ctr_R_minus_L'], 1).tolist()} | "
                     f"{v['can_rot_model']:.0f}° | {user} | {v['R_jaw_fit']:+.0f}° |")
        L.append("")
    md.write_text("\n".join(L) + "\n", encoding="utf-8")
    return md


# --------------------------------------------------------------------------- 대화형 실행
class NoGripper:
    def __init__(self):
        self.sent = []

    def send_angles(self, a, b):
        self.sent.append((time.monotonic(), a, b))

    def close(self):
        pass


def run_interactive(read: Callable[[], List[Optional[float]]], names, keys, ref, ref_path: Path, gripper, grip_deg,
                    out_dir: Path = OUT_DIR, ts: Optional[str] = None, clock=time.monotonic, sleep=time.sleep,
                    write=sys.stdout.write, snap=None, settle_s: float = SETTLE_S, avg_s: float = AVG_S) -> Path:
    """read() → 10 모터값 (names 차례), keys.get() → 글자 또는 None. gripper.send_angles(A, B) (A = L, B = R).
    grip_deg = ((열림 L, R), (닫힘 L, R)). snap(row: dict) = 기록 줄 (로거 snap.csv). 반환 = 보고 .md 경로."""
    ts = ts or datetime.now().strftime("%Y%m%d_%H%M%S")
    model = Model(ref)
    st_list = steps()
    (o_l, o_r), (c_l, c_r) = grip_deg
    closed = {"left": False, "right": False}

    def grip(side_state):
        closed.update(side_state)
        gripper.send_angles(c_l if closed["left"] else o_l, c_r if closed["right"] else o_r)

    recs: Dict[str, dict] = {}
    i, msg, result_lines = 0, "", []
    phase, t_phase, buf = "wait", clock(), []
    history: List[int] = []
    started = False
    while True:
        st = st_list[i] if i < len(st_list) else None
        if st is not None and not started:
            if st.get("pre_open"):
                grip({"left": False, "right": False})
            if st.get("pre_close"):
                grip({s: True for s in st["pre_close"]})
            started = True
        pos = read()
        now = clock()
        # ---- 기록 상태 기계: wait → settle (닫는 중) → avg (평균) → (angle) → wait
        if phase == "settle" and now - t_phase >= settle_s:
            phase, t_phase, buf = "avg", now, []
        if phase == "avg":
            if all(v is not None for v in pos):
                buf.append(pos)
            if now - t_phase >= avg_s and buf:
                arr = np.array(buf)
                rec = {"pos": arr.mean(0).tolist(), "std_deg": np.degrees(arr.std(0)).tolist(), "n": len(buf),
                       "t_wall": time.time(), "closed": dict(closed)}
                recs[st["id"]] = rec
                if snap:
                    snap({"step": st, "rec": rec})
                write("\a")
                if st.get("angle"):
                    phase = "angle"
                else:
                    result_lines = step_result_lines(model, st, recs, names)
                    if st.get("open_after"):
                        grip({s: False for s in st["open_after"]})
                    history.append(i); i += 1; started = False; phase = "wait"
                    msg = f"✔ 기록 {st['id']} ({len(buf)} 샘플)"
        # ---- 화면
        out = ["\033[H\033[J", f"집게 실측 T (모터 힘 없음 — 팔을 항상 받친다)   robot_ref {Path(ref_path).name}   "
               f"단계 {min(i + 1, len(st_list))}/{len(st_list)}", ""]
        if st is None:
            out += ["모든 단계를 끝냈다. q 로 끝내면 결과를 저장한다."]
        else:
            out += [f"▶ {st['title']}", f"  {st['text']}", ""]
            out.append({"wait": "  준비되면 space", "settle": "  … 집게 닫는 중 (움직이지 마세요)", "avg": "  … 기록 중 2 초 (움직이지 마세요)",
                        "angle": "  캔 윗면 선 ↔ L 집게 긴 변 각: 0 = 0°  1 = 15°  2 = 30°  3 = 45°"}[phase])
            if all(v is not None for v in pos) and st["side"] in SIDES:
                hd = model.hand(st["side"], joints_of(pos, names)[st["side"]])
                out.append(f"  지금 {HAND[st['side']]} 집게 끝 (모델) x {hd['tip'][0]:5.1f}  y {hd['tip'][1]:+5.1f}  z {hd['tip'][2]:5.1f} cm")
        out += ["", *result_lines, "", "space 진행 · r 앞 기록 지우고 다시 · s 건너뛰기 · q 끝내고 저장", msg]
        write("\n".join(out))
        key = keys.get()
        if key == "q":
            break
        if st is not None and phase == "wait" and key == " ":
            if st.get("record", True) is False:
                if st.get("open_after"):
                    grip({s: False for s in st["open_after"]})
                history.append(i); i += 1; started = False; msg = f"✔ {st['id']}: 집게를 열었다"
            elif st["grip"]:
                grip({s: True for s in st["grip"]}); phase, t_phase = "settle", now
            else:
                phase, t_phase, buf = "avg", now, []
        elif st is not None and phase == "angle" and key in ANGLE_KEYS:
            recs[st["id"]]["angle"] = float(ANGLE_KEYS[key])
            if snap:
                snap({"step": st, "rec": recs[st["id"]], "angle_only": True})
            result_lines = step_result_lines(model, st, recs, names)
            history.append(i); i += 1; started = False; phase = "wait"
            msg = f"✔ 기록 {st['id']} + 눈대중 {ANGLE_KEYS[key]}°"
        elif st is not None and phase == "wait" and key == "s":
            history.append(i); i += 1; started = False; msg = f"… 건너뜀 {st['id']}"
        elif phase == "wait" and key == "r" and history:
            i = history.pop()
            recs.pop(st_list[i]["id"], None)
            started = False
            msg = f"↺ {st_list[i]['id']} 을 다시 한다"
            result_lines = []
        sleep(0.05)
    res = judge(ref, recs, names)
    md = write_outputs(out_dir, ts, ref_path, recs, names, res,
                       {"gripper": "직렬" if not isinstance(gripper, NoGripper) else "없음 (--no-gripper)",
                        "grip_deg": grip_deg, "settle_s": settle_s, "avg_s": avg_s})
    write("\n\n" + "\n".join(res["lines"]) + f"\n\n저장: {md} (.json 같은 이름)\n")
    return md
