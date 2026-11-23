#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""위쪽 잡기 (top) 인계 보정 규칙 + L 여는 안전 규칙 (작업 지시 19-x21). 기본 끔 — policy_sim (eval_pairs --handover-assist) 과
replay_run --policy --handover-assist 가 켤 때만 쓴다. numpy 만 (시스템 파이썬 · venv 둘 다).

목표 자세: 통과 시연에서 R 이 닫는 순간 (닫기 명령 + CLOSE_S) 의 'L 이 쥔 캔 기준 R 집게 자세' 의 중앙값.
  캔 틀 F: z = 캔 축, x = L 손 축을 z 에 수직으로 내린 방향 (L 이 캔을 잡은 쪽), y = z × x. 원점 = 캔 밑면 가운데 (19-h 가정 B — L 집게 축 가운데로 옮긴 자리).
  R 위치 p = Fᵀ (R 끝 − 원점), R 방향 = Fᵀ R_R. 판 · 시연마다 캔이 어느 쪽으로 돌았든 같은 값이 된다.
규칙 (한 에피소드에 한 번 켜진다):
  켬: L 이 캔을 쥐고 (시뮬 held_L, 실기 L 닫힘 + 붙인 캔) R 집게 끝 ↔ 캔 축 거리가 near_cm 안.
  다가감: R 명령 = 정책 R + α (IK(목표) − 정책 R), α 는 ramp_s 에 걸쳐 0 → 1. 관절 속도 · roll 멈춤 · 판 · 두 팔 간격 안전장치는 부르는 쪽 그대로.
    목표 IK 는 위치 + 방향 (방향은 W_ROT_CM cm/rad 로 섞음), roll 멈춤 · 관절 한계 안. 정책의 R 닫기는 목표 안일 때만 따른다 (그 전에는 열림 유지).
  닫기: α = 1 이고 지금 R 이 기준 안 (논리 깊이 ≥ depth_cm, |비킴| ≤ side_cm, 기울기 tilt_deg 안, 몸체 침범 ≤ body_cm) 이 settle_s 이어지면 R 을 닫는다.
    close_after_s 동안 기준에 못 들면 그냥 닫는다 (forced). 닫힌 뒤에도 R 은 캔 틀 목표를 따라간다 (L 이 움직이면 같이).
  L 여는 안전: 정책이 L 을 열려 하면, R 이 다 닫혔고 닫힌 R 이 기준 안 (실기: 엔코더 깊이 ≥ enc_depth_cm · 비킴) 일 때만 따른다.
    아니면 L 을 닫은 채 두고 R 을 다시 열어 다가감부터 다시 한다 (retries 번까지, 그 뒤에는 L 을 닫은 채 둔다 — 떨어뜨리지 않는다).
  끝: L 이 연 뒤 α 를 ramp_s 에 걸쳐 1 → 0 (R 을 정책에 돌려줌), R 그리퍼는 그때부터 정책을 따른다.
기록 (info): 켠 시각 · 처음 R 끝 거리 · 규칙이 옮긴 양 (정책 R ↔ 보낸 R 끝 거리 최대 cm, 관절 최대 °) · 닫은 시각 · 닫을 때 기준 값 · 다시 시도 · L 막은 횟수."""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Callable, Dict, Optional, Tuple

import numpy as np

JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "elbow_roll")
CLOSE_S = 0.55            # 닫기 명령 → 다 닫힘 (시연 · 19-x17 분석과 같은 값, 20 Hz 11 프레임)
W_ROT_CM = 30.0           # IK 방향 오차 무게 (1 rad = 30 cm). 19-x21: 25 판에 목표를 놓아 기준 안 5 → 15 / 25, 15 → 17, 30 → 22 (IK 위치 차 최대 1.7 cm)
IK_ITERS = 12
IK_LAMBDA = 1.0
EPS = 1e-4
DEFAULTS = {"w_rot_cm": W_ROT_CM, "near_cm": 8.0, "ramp_s": 1.0, "settle_s": 0.2, "close_after_s": 4.0, "retries": 2,
            "depth_cm": 1.7, "side_cm": 1.0, "tilt_deg": [15.0, 22.0], "body_cm": 0.3, "enc_depth_cm": 2.35}


# --------------------------------------------------------------------------- 기하
def can_frame(can_rot: np.ndarray, L_rot: np.ndarray) -> np.ndarray:
    z = can_rot[:, 2] / np.linalg.norm(can_rot[:, 2])
    x = L_rot[:, 0] - float(L_rot[:, 0] @ z) * z
    n = float(np.linalg.norm(x))
    if n < 1e-6:                              # L 손 축이 캔 축과 나란함 (일어나지 않음) — 캔 x 축
        x = can_rot[:, 0] - float(can_rot[:, 0] @ z) * z
        n = float(np.linalg.norm(x))
    x = x / n
    return np.column_stack((x, np.cross(z, x), z))


def rotvec(Rm: np.ndarray) -> np.ndarray:
    """회전 행렬 → 회전 벡터 (rad)."""
    c = max(-1.0, min(1.0, (np.trace(Rm) - 1.0) / 2.0))
    a = math.acos(c)
    if a < 1e-9:
        return np.zeros(3)
    v = np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]])
    s = math.sin(a)
    if abs(s) < 1e-6:                         # 180° 근처
        w, V = np.linalg.eigh((Rm + Rm.T) / 2.0)
        return a * V[:, int(np.argmax(w))]
    return a * v / (2.0 * s)


def mean_rot(Rs) -> np.ndarray:
    """회전들의 (현) 평균: Σ R 을 SVD 로 가장 가까운 회전에."""
    U, _, Vt = np.linalg.svd(np.sum(Rs, axis=0))
    D = np.diag([1.0, 1.0, float(np.sign(np.linalg.det(U @ Vt)))])
    return U @ D @ Vt


def check_r(vc, hand_R, can_pos: np.ndarray, can_rot: np.ndarray, extra_cm: float) -> dict:
    """R 둘째 손 판정 값 (가정 B 자리의 캔 = can_pos). virtual_can 의 실물 기준과 같은 식: 깊이 s (grip_check), 비킴 (strict_center),
    기울기 (R 셋째 축 ↔ 캔 축), 몸체 침범 (닫힌 집게의 링크 · 몸체 상자 ↔ 캔 몸통, 음수 = 들어감)."""
    keep = (vc.pos, vc.rot, list(vc.holders), vc.align)
    try:
        vc.pos, vc.rot, vc.holders, vc.align = np.asarray(can_pos, float), np.asarray(can_rot, float), ["left"], None
        c = vc.grip_check(hand_R, extra_cm, True, side="right")
        _, side = vc.strict_center(hand_R)
        tip, R = hand_R
        z = vc.rot[:, 2]
        tilt = math.degrees(math.acos(min(1.0, abs(float(R[:, 2] @ z)))))
        rel = (vc.pos - tip)[None, :] + vc._bump_t[:, None] * z[None, :]
        s, j, n = -(rel @ R[:, 0]), rel @ R[:, 1], rel @ R[:, 2]
        boxes = vc._hand_boxes("right", vc.radius)[2:]          # 링크 · 몸체 (집게 날 둘은 뺌)
        body = float(min(vc._box_dist(s, j, n, b).min() for b in boxes) - vc.radius)
        return {"s": float(c["s"]), "j": float(c["j"]), "t": float(c["t"]), "side": float(side), "tilt": tilt, "body": body, "grip_ok": bool(c["ok"]),
                "why": c["reason"]}
    finally:
        vc.pos, vc.rot, vc.holders, vc.align = keep


def check_grab(vc, hand_R, can_pos: np.ndarray, can_rot: np.ndarray, extra_cm: float, align) -> dict:
    """시뮬 가상 캔이 R 닫기에서 실제로 보는 잡기 판정 (캔은 붙은 그대로의 자리, 정렬 조건 align 포함)."""
    keep = (vc.pos, vc.rot, list(vc.holders), vc.align)
    try:
        vc.pos, vc.rot, vc.holders, vc.align = np.asarray(can_pos, float), np.asarray(can_rot, float), ["left"], align
        c = vc.grip_check(hand_R, extra_cm, True, side="right")
        return {"ok": bool(c["ok"]), "why": c["reason"], "axis_cm": float(c.get("axis_cm", float("nan"))), "align_deg": float(c.get("align_deg", float("nan")))}
    finally:
        vc.pos, vc.rot, vc.holders, vc.align = keep


def within(chk: dict, cfg: dict, depth_key: str = "depth_cm") -> bool:
    t0, t1 = cfg["tilt_deg"]
    return (chk["s"] >= cfg[depth_key] and abs(chk["side"]) <= cfg["side_cm"] and t0 <= chk["tilt"] <= t1 and chk["body"] >= -cfg["body_cm"])


def ik_pose(hand_of: Callable, side: str, q0: Dict[str, float], p_star: np.ndarray, R_star: np.ndarray,
            limits: Dict[str, Tuple[float, float]], stop: Optional[Tuple[float, float]] = None, iters: int = IK_ITERS,
            w_rot: Optional[float] = None) -> Tuple[Dict[str, float], dict]:
    """5 관절 감쇠 최소제곱으로 집게 끝 위치 · 방향 (방향은 W_ROT_CM 무게). 모터 값. roll 멈춤 · 관절 한계 안."""
    q = {j: float(q0[j]) for j in JOINTS}
    w = W_ROT_CM if w_rot is None else float(w_rot)

    def err(qq):
        t, R = hand_of(side, qq)
        return np.concatenate([p_star - t, w * rotvec(R_star @ R.T)])
    e = err(q)
    for _ in range(iters):
        Jm = np.empty((6, len(JOINTS)))
        for k, j in enumerate(JOINTS):
            q2 = dict(q)
            q2[j] += EPS
            Jm[:, k] = (e - err(q2)) / EPS          # e = 목표 − 지금 → de/dq = −J
        dq = Jm.T @ np.linalg.solve(Jm @ Jm.T + IK_LAMBDA ** 2 * np.eye(6), e)
        for k, j in enumerate(JOINTS):
            lo, hi = limits[j]
            if stop is not None and j == "shoulder_roll":
                lo, hi = max(lo, stop[0]), min(hi, stop[1])
            q[j] = min(max(q[j] + float(dq[k]), lo), hi)
        e = err(q)
    return q, {"pos_cm": float(np.linalg.norm(e[:3])), "rot_deg": math.degrees(float(np.linalg.norm(e[3:]) / w))}


# --------------------------------------------------------------------------- 목표 읽기
def load_target(path) -> dict:
    d = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    cfg = dict(DEFAULTS)
    cfg.update(d.get("rule", {}))
    return {"p": np.asarray(d["target"]["p_can_cm"], float), "R": np.asarray(d["target"]["R_can"], float), "cfg": cfg, "src": str(path)}


# --------------------------------------------------------------------------- 규칙
class HandoverAssist:
    """한 에피소드 상태. 부르는 쪽이 매 정책 걸음 step() 을 부른다 (모터 값 사전 · 그리퍼 0/1)."""

    def __init__(self, target: dict, vc, limits_R: Dict[str, Tuple[float, float]], roll_stop_R: Optional[Tuple[float, float]] = None,
                 extra_cm: float = 0.0, log: Callable[[str], None] = lambda s: None):
        self.p_rel, self.R_rel, self.cfg = target["p"], target["R"], target["cfg"]
        self.vc, self.limits, self.stop, self.extra, self.log = vc, limits_R, roll_stop_R, float(extra_cm), log
        self.reset()

    def reset(self) -> None:
        self.st = "off"               # off → approach → closing → held → release → done
        self.alpha = 0.0
        self.t_in = None
        self.t_full = None
        self.q_ik = None
        self.info = {"on_t": None, "first_dist_cm": None, "moved_tip_cm_max": 0.0, "moved_joint_deg_max": 0.0, "close_t": [], "close_chk": [],
                     "forced_close": 0, "retries": 0, "L_blocked_steps": 0, "ik_pos_cm_max": 0.0, "ik_rot_deg_max": 0.0, "done_t": None, "gave_up": False}
        self.shift = 0.0              # 19-x26 A (rule enc_target_cm 일 때만): R 목표를 R 손 축으로 더 깊게 (cm)
        self.abort_d = 0.0            # 19-x26 B (rule abort_retreat_cm 일 때만): 다시 시도를 다 쓴 뒤 R 을 열고 물러난 양 (cm)
        if self.cfg.get("enc_target_cm") is not None:
            self.info["shift_cm_max"] = 0.0
        if self.cfg.get("abort_retreat_cm") is not None:
            self.info["aborted"] = False

    def _goal(self, can_pos, can_rot, L_rot):
        F = can_frame(can_rot, L_rot)
        return can_pos + F @ self.p_rel, F @ self.R_rel

    def step(self, now: float, qL: Dict[str, float], qR_pol: Dict[str, float], qR_cur: Dict[str, float], want: Dict[str, int],
             L_held: bool, R_closed: bool, can_pos: Optional[np.ndarray], can_rot: Optional[np.ndarray], dt: float,
             qR_check: Optional[Dict[str, float]] = None, enc_check: Optional[Callable[[], Optional[dict]]] = None,
             grab: Optional[dict] = None) -> Tuple[Dict[str, float], Dict[str, int]]:
        """qL · qR_cur = 지금 (판정용) 자세, qR_pol = 정책 R 목표. can_pos = 가정 B 자리의 캔 밑면 (L 이 쥘 때만). 반환 (보낼 R 목표, 그리퍼 0/1).
        qR_check: R 기준 판정 자세 (없으면 qR_cur). enc_check: 실기 L 여는 안전용 — 엔코더 판정 값 (None 이면 논리 판정).
        grab (시뮬만): {"can_pos", "can_rot", "align", "held_both"} — 가상 캔이 실제로 둔 캔 (붙은 그대로) 과 정렬 조건, 두 손이 다 쥐었나.
          주면 목표는 이 캔 기준, 목표 안 = 실물 기준 (가정 B) + 가상 캔 잡기 판정, L 열기 = held_both + 실물 깊이 · 비킴."""
        cfg, hand = self.cfg, self.vc.hand_of
        out_w = dict(want)
        if self.st == "done":
            return qR_pol, out_w
        tL, RL = hand("left", qL)
        tR, RR = hand("right", qR_cur)
        if self.st == "off":
            if not (L_held and can_pos is not None):
                return qR_pol, out_w
            z = can_rot[:, 2]
            v = tR - (grab["can_pos"] if grab is not None else can_pos)
            d = float(np.linalg.norm(v - float(v @ z) * z))
            if float(v @ z) < -2.0 or float(v @ z) > self.vc.height + 2.0 or d > cfg["near_cm"]:
                out_w["R"] = 0 if not R_closed else out_w["R"]         # 켜기 전: 정책 R 닫기는 목표 안일 때만 → 아직은 열림
                return qR_pol, out_w
            self.st = "approach"
            self.info["on_t"], self.info["first_dist_cm"] = round(now, 2), round(d, 2)
            self.q_ik = dict(qR_cur)
            self.log(f"[인계 보정] 켬 — R 끝 ↔ 캔 축 {d:.1f} cm")
        # 목표 IK (L · 캔이 움직이면 같이)
        if self.st == "abort_back":                                   # 19-x26 B: R 이 다 열린 뒤 R 손 축 뒤로 물러남
            self.abort_d = min(float(cfg["abort_retreat_cm"]), self.abort_d + dt * float(cfg.get("abort_rate_cms", 3.0)))
            if self.abort_d >= float(cfg["abort_retreat_cm"]) - 1e-9:
                self.st = "abort_done"
                self.info["abort_done_t"] = round(now, 2)
                self.log(f"[인계 보정] R 을 열고 {self.abort_d:g} cm 물러남 — 끝낸다 (L 은 쥔 채)")
        if can_pos is not None:
            p_star, R_star = self._goal(grab["can_pos"], grab["can_rot"], RL) if grab is not None else self._goal(can_pos, can_rot, RL)
            if self.shift or self.abort_d:                            # 19-x26 A · B (켤 때만 0 이 아님): 목표를 R 손 축 (+ 깊게) 으로
                p_star = p_star + (self.shift - self.abort_d) * R_star[:, 0]
            self.q_ik, ik = ik_pose(hand, "right", self.q_ik or qR_cur, p_star, R_star, self.limits, self.stop, w_rot=cfg["w_rot_cm"])
            self.info["ik_pos_cm_max"] = max(self.info["ik_pos_cm_max"], round(ik["pos_cm"], 2))
            self.info["ik_rot_deg_max"] = max(self.info["ik_rot_deg_max"], round(ik["rot_deg"], 1))
        if self.st == "release":
            self.alpha = max(0.0, self.alpha - dt / cfg["ramp_s"])
            if self.alpha <= 0.0:
                self.st = "done"
                self.info["done_t"] = round(now, 2)
        else:
            self.alpha = min(1.0, self.alpha + dt / cfg["ramp_s"])
        q_out = {j: float(qR_pol[j]) + self.alpha * (float(self.q_ik[j]) - float(qR_pol[j])) for j in JOINTS} if self.q_ik is not None else dict(qR_pol)
        dtip = float(np.linalg.norm(hand("right", q_out)[0] - hand("right", qR_pol)[0]))
        self.info["moved_tip_cm_max"] = max(self.info["moved_tip_cm_max"], round(dtip, 2))
        self.info["moved_joint_deg_max"] = max(self.info["moved_joint_deg_max"],
                                              round(max(abs(math.degrees(q_out[j] - float(qR_pol[j]))) for j in JOINTS), 1))
        if self.st == "release":
            return q_out, out_w
        if self.st in ("abort", "abort_back", "abort_done"):         # 19-x26 B: R 열기 → 물러남 → 끝 (L 은 쥔 채)
            out_w["L"], out_w["R"] = 1, 0
            if self.st == "abort" and not R_closed:
                self.st = "abort_back"
            return q_out, out_w
        hR_chk = hand("right", qR_check or qR_cur)
        chk = check_r(self.vc, hR_chk, can_pos, can_rot, self.extra) if can_pos is not None else None
        ok = chk is not None and within(chk, cfg if not self.shift else dict(cfg, body_cm=cfg["body_cm"] + self.shift))
        if cfg.get("enc_target_cm") is not None and self.st == "approach" and self.alpha >= 1.0 and enc_check is not None:
            e_ = enc_check()                                           # 19-x26 A: 엔코더 판정 깊이가 enc_target_cm 이 되도록 목표를 깊게 (0 ~ shift_max_cm)
            if e_ is not None:
                r_ = float(cfg.get("shift_rate_cms", 2.0)) * dt
                self.shift = min(float(cfg.get("shift_max_cm", 2.5)), max(0.0, self.shift + max(-r_, min(r_, float(cfg["enc_target_cm"]) - e_["s"]))))
                self.info["shift_cm_max"] = max(self.info["shift_cm_max"], round(self.shift, 2))
                ok = ok and e_["s"] >= cfg["enc_depth_cm"]
        if ok and grab is not None:
            gk = check_grab(self.vc, hR_chk, grab["can_pos"], grab["can_rot"], self.extra, grab.get("align"))
            ok = gk["ok"]
            chk = dict(chk, grab_ok=gk["ok"], grab_why=gk["why"], axis_cm=gk["axis_cm"])
        if self.st == "approach":
            out_w["R"] = 0
            if self.alpha >= 1.0:
                self.t_full = now if self.t_full is None else self.t_full
                self.t_in = (now if self.t_in is None else self.t_in) if ok else None
                forced = now - self.t_full >= cfg["close_after_s"]
                if (self.t_in is not None and now - self.t_in >= cfg["settle_s"]) or forced:
                    self.st = "closing"
                    self.info["close_t"].append(round(now, 2))
                    self.info["close_chk"].append({k: (round(v, 2) if isinstance(v, float) else v) for k, v in (chk or {}).items()})
                    self.info["forced_close"] += int(forced and self.t_in is None)
                    self.log(f"[인계 보정] R 닫음{' (시간, 기준 밖)' if forced and self.t_in is None else ''}"
                             + (f" — 깊이 {chk['s']:.2f} 비킴 {chk['side']:+.2f} 기울기 {chk['tilt']:.1f}°" if chk is not None else " — 캔 없음"))
            out_w["L"] = 1
            return q_out, out_w
        if self.st in ("closing", "held"):
            out_w["R"] = 1
            if self.st == "closing" and R_closed:
                self.st = "held"
            if not want.get("L", 1):                                   # 정책이 L 을 열려 함
                good = False
                if self.st == "held":
                    e = enc_check() if enc_check is not None else None
                    if grab is not None:                               # 시뮬: 두 손이 다 쥐었고 (held_both) 실물 깊이 · 비킴이 기준 안
                        good = bool(grab.get("held_both")) and chk is not None and chk["s"] >= cfg["depth_cm"] and abs(chk["side"]) <= cfg["side_cm"]
                    elif e is not None:
                        good = e["s"] >= cfg["enc_depth_cm"] and abs(e["side"]) <= cfg["side_cm"]
                    else:
                        good = ok or (chk is not None and chk["s"] >= cfg["depth_cm"] and abs(chk["side"]) <= cfg["side_cm"])
                if good:
                    self.st = "release"
                    out_w["L"] = 0
                    self.log("[인계 보정] L 엶 허락 — R 이 기준 안")
                    return q_out, out_w
                out_w["L"] = 1
                self.info["L_blocked_steps"] += 1
                if self.st == "held" and not self.info["gave_up"]:
                    if self.info["retries"] < cfg["retries"]:
                        self.info["retries"] += 1
                        self.st, self.t_in, self.t_full = "approach", None, None
                        out_w["R"] = 0
                        self.log(f"[인계 보정] R 이 기준 밖 — L 을 닫은 채 R 을 다시 연다 ({self.info['retries']} 번째)")
                    else:
                        self.info["gave_up"] = True
                        if cfg.get("abort_retreat_cm") is not None:           # 19-x26 B: R 을 열고 물러난 뒤 끝낸다 (맞물림 방지)
                            self.st = "abort"
                            self.info["aborted"] = True
                            out_w["R"] = 0
                            self.log(f"[인계 보정] 다시 시도 다 씀 — R 을 열고 {float(cfg['abort_retreat_cm']):g} cm 물러난 뒤 끝낸다 (L 은 쥔 채)")
                        else:
                            self.log("[인계 보정] 다시 시도 다 씀 — L 을 닫은 채 둔다")
            else:
                out_w["L"] = 1
            return q_out, out_w
        return q_out, out_w


# --------------------------------------------------------------------------- 목표 만들기 (FK 만, 시뮬 없음)
def build(files, out_path, ref_logs: Path, W, VC, TL, DE, AD, L) -> dict:
    """통과 시연 목록 → 목표 json. 시연마다 R 닫기 + CLOSE_S 의 캔 틀 R 자세와 그 판 L 자세에서 목표를 IK 로 냈을 때의 기준 값."""
    S = ("left", "right")
    rows, Ps, Rs = [], [], []
    tab = TL.TableLimit(ref_logs.parent / "config/table.json", body="real")
    cache = {}
    for f in files:
        arr = L.arrays(f)
        meta = arr["meta"]
        rp = ref_logs / meta["robot_ref"]
        if rp not in cache:
            ref = W.PR.RobotRef.load(rp)
            vc = VC.VirtualCan(tab, "L1", ref, S, W.motor_to_urdf_fn(ref, S), log=lambda s: None, grasp_mode="top", real=True)
            lim, _ = W.mirror_limits(W.limits_from_robot_ref(ref, S), ref, S)
            cache[rp] = (vc, DE.JointMap(ref), lim)
        vc, jm, lim = cache[rp]
        act = np.asarray(arr["action"], float)
        iLc, iRc, iLo, iRo = AD.segments(act)
        (sx, sy), _ = AD.demo_xy(meta)
        can0 = np.array([sx, sy, tab.top(sx / 100.0) * 100.0])
        q = lambda k, s: jm.to_motor(s, dict(zip(JOINTS, act[min(k, len(act) - 1), 5 * S.index(s):5 * S.index(s) + 5])))   # noqa: E731
        hL = vc.hand_of("left", q(iLc + 11, "left"))
        vc.pos, vc.rot, vc.holders, vc.align = can0.copy(), np.eye(3), [], None
        c = vc.grip_check(hL, 0.0, False, side="left")
        canB = can0 - float(c["j"]) * hL[1][:, 1]
        rel = (hL[1].T, hL[1].T @ (canB - hL[0]))
        k = iRc + 11
        hL2, hR2 = vc.hand_of("left", q(k, "left")), vc.hand_of("right", q(k, "right"))
        can_rot, can_pos = hL2[1] @ rel[0], hL2[0] + hL2[1] @ rel[1]
        F = can_frame(can_rot, hL2[1])
        p, Rr = F.T @ (hR2[0] - can_pos), F.T @ hR2[1]
        chk = check_r(vc, hR2, can_pos, can_rot, VC.HANDOVER_HEIGHT_EXTRA_CM)
        Ps.append(p); Rs.append(Rr)
        rows.append({"name": Path(f).stem, "p_can_cm": p.round(3).tolist(), "own": {k2: (round(v, 2) if isinstance(v, float) else v) for k2, v in chk.items()},
                     "_ctx": (rp, q(k, "left"), q(k, "right"), can_pos, can_rot)})
    Pm = np.median(np.array(Ps), 0)
    Rm = mean_rot(np.array(Rs))
    dev = [math.degrees(float(np.linalg.norm(rotvec(Rm.T @ r)))) for r in Rs]
    stop = (-math.inf, math.radians(W.ROLL_STOP_DEG["right"]))
    cfg = dict(DEFAULTS)
    for r in rows:                                          # 그 판 L 자세 · 캔에서 목표를 IK 로 냈을 때
        rp, qLk, qRk, can_pos, can_rot = r.pop("_ctx")
        vc, jm, lim = cache[rp]
        F = can_frame(can_rot, vc.hand_of("left", qLk)[1])
        qi, ik = ik_pose(vc.hand_of, "right", qRk, can_pos + F @ Pm, F @ Rm, lim["right"], stop)
        chk = check_r(vc, vc.hand_of("right", qi), can_pos, can_rot, VC.HANDOVER_HEIGHT_EXTRA_CM)
        r["target_on_this"] = {**{k2: (round(v, 2) if isinstance(v, float) else v) for k2, v in chk.items()}, "ik_pos_cm": round(ik["pos_cm"], 2),
                               "ik_rot_deg": round(ik["rot_deg"], 1), "R_roll_deg": round(math.degrees(qi["shoulder_roll"]), 2), "within": within(chk, cfg)}
        r["own_within"] = within(r["own"], cfg)
    out = {"made_by": "handover_assist.build (19-x21)", "files": [str(f) for f in files], "n": len(rows),
           "target": {"p_can_cm": Pm.round(3).tolist(), "R_can": Rm.round(6).tolist()},
           "spread": {"p_std_cm": np.std(np.array(Ps), 0).round(3).tolist(), "p_iqr_cm": (np.percentile(Ps, 75, 0) - np.percentile(Ps, 25, 0)).round(3).tolist(),
                      "rot_dev_deg_median": round(float(np.median(dev)), 2), "rot_dev_deg_max": round(float(max(dev)), 2)},
           "frame": "캔 틀: z = 캔 축, x = L 손 축을 z 에 수직으로 내린 방향, 원점 = 캔 밑면 (가정 B). R 은 집게 끝 (cm) 과 집게 틀 (열 h j n)",
           "rule": {k2: v for k2, v in DEFAULTS.items()}, "rows": rows}
    Path(out_path).expanduser().write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    return out



# =========================================================================== 19-x22 L 잡기 보정 (기본 끔)
GRASP_DEFAULTS = {"w_rot_cm": W_ROT_CM, "near_cm": 8.0, "ramp_s": 1.0, "settle_s": 0.2, "give_up_s": 4.0, "side_cm": 1.0}


def grasp_frame(can_pos: np.ndarray, shoulder: np.ndarray) -> np.ndarray:
    """L 잡기 틀: z = 위 (판 위에 선 캔), x = L 어깨 → 캔 수평 방향. 39 판 방향 퍼짐 중앙 3.1° (로봇 틀 그대로는 13.2°)."""
    az = math.atan2(float(can_pos[1] - shoulder[1]), float(can_pos[0] - shoulder[0]))
    return np.array([[math.cos(az), -math.sin(az), 0.0], [math.sin(az), math.cos(az), 0.0], [0.0, 0.0, 1.0]])


def shoulder_of(vc, side: str = "left") -> np.ndarray:
    return vc.models[side]._O[0][:3, 3] * 100.0


def check_l(vc, hand_L, can_pos: np.ndarray, align=None) -> dict:
    """L 먼저 잡기 판정 (아무도 안 쥔 캔, 판 위에 섬): grip_check (top 띠 10 ~ 14 · 정렬 align), 집게 가운데 비킴 (실물 기준 L 비킴과 같은 값)."""
    keep = (vc.pos, vc.rot, list(vc.holders), vc.align)
    try:
        vc.pos, vc.rot, vc.holders, vc.align = np.asarray(can_pos, float), np.eye(3), [], align
        c = vc.grip_check(hand_L, 0.0, False, side="left")
        _, side = vc.strict_center(hand_L)
        return {"ok": bool(c["ok"]), "why": c["reason"], "t": float(c["t"]), "s": float(c["s"]), "side": float(side)}
    finally:
        vc.pos, vc.rot, vc.holders, vc.align = keep


def load_grasp_target(path) -> dict:
    d = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    cfg = dict(GRASP_DEFAULTS)
    cfg.update(d.get("rule", {}))
    return {"p": np.asarray(d["target"]["p_can_cm"], float), "R": np.asarray(d["target"]["R_can"], float), "cfg": cfg, "src": str(path)}


class GraspAssist:
    """L 이 아직 안 쥐었고 L 끝이 목표 끝에서 near_cm 안이면: L 명령을 목표 쪽으로 (α 0 → 1, ramp_s). 목표 안 (잡기 판정 ok · |비킴| ≤ side_cm) 이
    settle_s 이어질 때만 정책의 L 닫기를 따른다. give_up_s 안에 못 들면 정책에 돌려준다 (α → 0, 정책 그대로). 쥔 뒤 ramp_s 에 걸쳐 정책에 돌려준다."""

    def __init__(self, target: dict, vc, limits_L: Dict[str, Tuple[float, float]], roll_stop_L: Optional[Tuple[float, float]] = None,
                 log: Callable[[str], None] = lambda s: None):
        self.p_rel, self.R_rel, self.cfg = target["p"], target["R"], target["cfg"]
        self.vc, self.limits, self.stop, self.log = vc, limits_L, roll_stop_L, log
        self.sh = shoulder_of(vc, "left")
        self.reset()

    def reset(self) -> None:
        self.st = "off"               # off → approach → closing → back → done ; approach → giveup → done
        self.alpha = 0.0
        self.t_on = self.t_in = None
        self.q_ik = None
        self.info = {"on_t": None, "first_dist_cm": None, "moved_tip_cm_max": 0.0, "moved_joint_deg_max": 0.0, "close_t": None, "close_chk": None,
                     "gave_up": False, "held_t": None, "ik_pos_cm_max": 0.0}

    def step(self, now: float, qL_pol: Dict[str, float], qL_cur: Dict[str, float], want_L: int, L_held: bool, can_pos: Optional[np.ndarray],
             dt: float, align=None, tip_err: Optional[float] = None) -> Tuple[Dict[str, float], int]:
        """tip_err (19-x26 C, 실기만): 지금 L 집게 끝 오프셋 IK 가 못 낸 차 (cm, replay_run tip_info err_cm). rule tip_err_max_cm 일 때만 쓴다."""
        cfg, hand = self.cfg, self.vc.hand_of
        if self.st == "out":                                          # 19-x26 C: 도달 밖 — L 을 연 채 정책을 멈추게 한다 (부르는 쪽)
            return dict(self.q_ik) if self.q_ik is not None else qL_pol, 0
        if self.st == "done" or can_pos is None:
            return qL_pol, want_L
        F = grasp_frame(can_pos, self.sh)
        p_star, R_star = can_pos + F @ self.p_rel, F @ self.R_rel
        tL = hand("left", qL_cur)[0]
        if self.st == "off":
            d = float(np.linalg.norm(tL - p_star))
            if L_held or d > cfg["near_cm"]:
                if L_held:
                    self.st = "done"
                return qL_pol, want_L
            self.st, self.t_on = "approach", now
            self.info["on_t"], self.info["first_dist_cm"] = round(now, 2), round(d, 2)
            self.q_ik = dict(qL_cur)
            self.log(f"[L 잡기 보정] 켬 — L 끝 ↔ 목표 {d:.1f} cm")
        if self.st in ("approach", "closing"):
            self.q_ik, ik = ik_pose(hand, "left", self.q_ik, p_star, R_star, self.limits, self.stop, w_rot=cfg["w_rot_cm"])
            self.info["ik_pos_cm_max"] = max(self.info["ik_pos_cm_max"], round(ik["pos_cm"], 2))
            self.alpha = min(1.0, self.alpha + dt / cfg["ramp_s"])
        else:                                                         # back · giveup: 정책에 돌려줌
            self.alpha = max(0.0, self.alpha - dt / cfg["ramp_s"])
            if self.alpha <= 0.0:
                self.st = "done"
        q_out = {j: float(qL_pol[j]) + self.alpha * (float(self.q_ik[j]) - float(qL_pol[j])) for j in JOINTS} if self.q_ik is not None else dict(qL_pol)
        self.info["moved_tip_cm_max"] = max(self.info["moved_tip_cm_max"], round(float(np.linalg.norm(hand("left", q_out)[0] - hand("left", qL_pol)[0])), 2))
        self.info["moved_joint_deg_max"] = max(self.info["moved_joint_deg_max"],
                                              round(max(abs(math.degrees(q_out[j] - float(qL_pol[j]))) for j in JOINTS), 1))
        if self.st == "approach":
            c = check_l(self.vc, hand("left", qL_cur), can_pos, align)
            ok = c["ok"] and abs(c["side"]) <= cfg["side_cm"]
            if ok and cfg.get("close_dz_cm") is not None:            # 19-x26 C: 목표 높이 ± close_dz_cm 안
                ok = abs(float(tL[2] - p_star[2])) <= float(cfg["close_dz_cm"])
            if ok and cfg.get("close_depth_min_cm") is not None:     # 19-x26 C: 실물 깊이 (논리 + phys_shift_cm) ≥ close_depth_min_cm
                ok = c["s"] + float(cfg.get("phys_shift_cm", 0.0)) >= float(cfg["close_depth_min_cm"])
            self.t_in = (now if self.t_in is None else self.t_in) if ok else None
            if self.t_in is not None and now - self.t_in >= cfg["settle_s"] and want_L and cfg.get("tip_err_max_cm") is not None \
                    and tip_err is not None and tip_err > float(cfg["tip_err_max_cm"]):
                self.st = "out"                                       # 19-x26 C: 오프셋 IK 가 끝을 못 냄 (tip-cap) — 닫지 않고 끝낸다
                self.info["out_why"] = f"끝 오차 {tip_err:.2f} cm"
                self.log(f"[L 잡기 보정] 도달 밖 — 끝 오차 {tip_err:.2f} cm > {float(cfg['tip_err_max_cm']):g}, 닫지 않고 끝낸다")
                return q_out, 0
            if self.t_in is not None and now - self.t_in >= cfg["settle_s"] and want_L:
                self.st = "closing"
                self.info["close_t"], self.info["close_chk"] = round(now, 2), {k: (round(v, 2) if isinstance(v, float) else v) for k, v in c.items()}
                self.log(f"[L 잡기 보정] L 닫음 — 높이 {c['t']:.1f} 비킴 {c['side']:+.2f}")
                return q_out, 1
            if now - self.t_on >= cfg["give_up_s"]:
                self.st = "giveup"
                self.info["gave_up"] = True
                if cfg.get("stop_on_giveup"):                         # 19-x26 C: 정책에 돌려주지 않고 끝낸다 (L 연 채)
                    self.st = "out"
                    self.info["out_why"] = "4 초 안에 기준 못 채움"
                    self.log("[L 잡기 보정] 4 초 안에 기준 (높이 · 깊이) 못 채움 — 닫지 않고 끝낸다")
                    return q_out, 0
                self.log("[L 잡기 보정] 4 초 안에 목표에 못 듦 — 정책에 돌려줌")
                return q_out, want_L
            return q_out, 0
        if self.st == "closing":
            if L_held:
                self.st = "back"
                self.info["held_t"] = round(now, 2)
            return q_out, 1
        return q_out, want_L


def build_grasp(files, out_path, ref_logs: Path, W, VC, TL, DE, AD, L) -> dict:
    """L 잡기 목표: 시연마다 L 닫힘 끝 (L 닫기 + CLOSE_S) 의 L 집게 자세를 잡기 틀 (grasp_frame) 로, 중앙 (위치) · 현 평균 (방향).
    그 판 캔 자리에 목표를 IK (roll 멈춤 L ≥ -19°) 로 놓고 잡기 판정 (top 띠 · 정렬 15/10/2 · |비킴| ≤ 1) 을 본다."""
    S = ("left", "right")
    tab = TL.TableLimit(ref_logs.parent / "config/table.json", body="real")
    cache, rows, Ps, Rs = {}, [], [], []
    for f in files:
        arr = L.arrays(f)
        rp = ref_logs / arr["meta"]["robot_ref"]
        if rp not in cache:
            ref = W.PR.RobotRef.load(rp)
            vc = VC.VirtualCan(tab, "L1", ref, S, W.motor_to_urdf_fn(ref, S), log=lambda s: None, grasp_mode="top", real=True)
            lim, _ = W.mirror_limits(W.limits_from_robot_ref(ref, S), ref, S)
            cache[rp] = (vc, DE.JointMap(ref), lim)
        vc, jm, lim = cache[rp]
        act = np.asarray(arr["action"], float)
        iLc, iRc, iLo, iRo = AD.segments(act)
        (sx, sy), _ = AD.demo_xy(arr["meta"])
        can0 = np.array([sx, sy, tab.top(sx / 100.0) * 100.0])
        qL = jm.to_motor("left", dict(zip(JOINTS, act[min(iLc + 11, len(act) - 1), 0:5])))
        tL, RL = vc.hand_of("left", qL)
        F = grasp_frame(can0, shoulder_of(vc))
        Ps.append(F.T @ (tL - can0)); Rs.append(F.T @ RL)
        rows.append({"name": Path(f).stem, "p_can_cm": (F.T @ (tL - can0)).round(3).tolist(), "own": check_l(vc, (tL, RL), can0, (15.0, 10.0, 2.0)),
                     "_ctx": (rp, qL, can0)})
    Pm, Rm = np.median(np.array(Ps), 0), mean_rot(np.array(Rs))
    dev = [math.degrees(float(np.linalg.norm(rotvec(Rm.T @ r)))) for r in Rs]
    stop = (math.radians(W.ROLL_STOP_DEG["left"]), math.inf)
    for r in rows:
        rp, qL, can0 = r.pop("_ctx")
        vc, jm, lim = cache[rp]
        F = grasp_frame(can0, shoulder_of(vc))
        qi, ik = ik_pose(vc.hand_of, "left", qL, can0 + F @ Pm, F @ Rm, lim["left"], stop)
        c = check_l(vc, vc.hand_of("left", qi), can0, (15.0, 10.0, 2.0))
        r["target_on_this"] = {**{k: (round(v, 2) if isinstance(v, float) else v) for k, v in c.items()}, "ik_pos_cm": round(ik["pos_cm"], 2),
                               "ik_rot_deg": round(ik["rot_deg"], 1), "L_roll_deg": round(math.degrees(qi["shoulder_roll"]), 2),
                               "within": bool(c["ok"] and abs(c["side"]) <= 1.0)}
        r["own"] = {k: (round(v, 2) if isinstance(v, float) else v) for k, v in r["own"].items()}
    out = {"made_by": "handover_assist.build_grasp (19-x22)", "files": [str(f) for f in files], "n": len(rows),
           "target": {"p_can_cm": Pm.round(3).tolist(), "R_can": Rm.round(6).tolist()},
           "spread": {"p_std_cm": np.std(np.array(Ps), 0).round(3).tolist(), "rot_dev_deg_median": round(float(np.median(dev)), 2), "rot_dev_deg_max": round(float(max(dev)), 2)},
           "frame": "잡기 틀: z = 위, x = L 어깨 → 캔 수평 방향, 원점 = 판 위 캔 밑면 가운데. L 은 집게 끝 (cm) 과 집게 틀 (열 h j n)",
           "rule": dict(GRASP_DEFAULTS), "rows": rows}
    Path(out_path).expanduser().write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    return out


if __name__ == "__main__":
    import argparse
    import sys
    ap = argparse.ArgumentParser(description="19-x21 인계 보정 목표 · 19-x22 L 잡기 목표 만들기 (FK 만)")
    ap.add_argument("--files", required=True, help="통과 시연 npz 목록")
    ap.add_argument("--out", required=True)
    ap.add_argument("--grasp", action="store_true", help="19-x22: L 잡기 목표 (기본 = R 인계 목표)")
    a = ap.parse_args()
    here = Path(__file__).resolve().parent
    sys.path[:0] = [str(here), str(Path.home() / "code_workspace/Berkeley-Humanoid-Lite-main/Berkeley-Humanoid-Lite-Lowlevel")]
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        import vr_control_watchdog_biarm as W
    import virtual_can as VC
    import table_limit as TL
    import demo_export as DE
    import aug_demo as AD
    import learn_data as L
    fl = [x.strip() for x in Path(a.files).expanduser().read_text(encoding="utf-8").splitlines() if x.strip()]
    if a.grasp:
        o = build_grasp(fl, a.out, here.parent / "logs", W, VC, TL, DE, AD, L)
        print("L 잡기 목표 p (잡기 틀 cm)", o["target"]["p_can_cm"], "| 퍼짐", o["spread"])
        for r in o["rows"]:
            t = r["target_on_this"]
            print(f"{r['name']}: 시연 높이 {r['own']['t']} 비킴 {r['own']['side']} ({'ok' if r['own']['ok'] else r['own']['why']}) | 목표를 이 판에: 높이 {t['t']} 비킴 {t['side']} IK {t['ik_pos_cm']} cm {t['ik_rot_deg']}° L roll {t['L_roll_deg']} ({'안' if t['within'] else '밖 ' + t['why']})")
        print("목표 안", sum(r["target_on_this"]["within"] for r in o["rows"]), "/", len(o["rows"]))
        raise SystemExit(0)
    o = build(fl, a.out, here.parent / "logs", W, VC, TL, DE, AD, L)
    print("목표 p (캔 틀 cm)", o["target"]["p_can_cm"], "| 퍼짐", o["spread"])
    for r in o["rows"]:
        t = r["target_on_this"]
        print(f"{r['name']}: 그 판 자체 깊이 {r['own']['s']} 비킴 {r['own']['side']} 기울기 {r['own']['tilt']} 몸체 {r['own']['body']} ({'안' if r['own_within'] else '밖'})"
              f" | 목표를 이 판에: 깊이 {t['s']} 비킴 {t['side']} 기울기 {t['tilt']} 몸체 {t['body']} IK {t['ik_pos_cm']} cm {t['ik_rot_deg']}° R roll {t['R_roll_deg']} ({'안' if t['within'] else '밖'})")
