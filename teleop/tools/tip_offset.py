#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""집게 끝 오프셋 (작업 지시 14-w). replay_run.py --tip-offset-cm 이 쓴다 (기본 꺼짐).

목적: 실물 집게가 엔코더 FK 보다 짧게 가는 것 (부하 처짐 등, 14-u·14-x) 을 팔마다 일정한 끝 위치 오프셋으로 메운다.
값은 그 순간 '논리 목표 자세' 의 집게 틀 3 축 (cm):
  h = 손 축 (손목 → 집게 끝, + = 더 깊게), j = 집게 닫히는 축 (virtual_can.hand_of 의 j, 14-v 의 +j), n = h × j.
계산: 논리 목표 q 의 FK 끝 p·틀 R → 원하는 끝 p' = p + gain · R·off.
  위치 4 관절 (어깨 3 + 팔꿈치) 로 감쇠 최소제곱 한 걸음 Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ (p' − p(q)) 를 ITERS 번 (J = 수치 미분).
  관절마다 |q − 목표| ≤ cap (기본 5°) 로 자르고, 잘린 관절 이름을 돌려준다 (화면 알림은 부르는 쪽).
FK 는 virtual_can.VirtualCan.hand_of (robot_viz 와 같은 체인·같은 집게 축) 를 그대로 쓴다. 모든 값은 모터 값 (rad) 이다."""
from __future__ import annotations

import math
from typing import Callable, Dict, List, Sequence, Tuple

import numpy as np

POS_JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch")
CAP_DEG = 5.0
LAMBDA_CM = 3.0          # 감쇠 (cm/rad 단위의 J 에 대해, J 의 특이값 20~40 보다 작게)
ITERS = 2
EPS = 1e-4               # 수치 미분 (rad)
ITERS_STOP = 8           # 19-x19 roll_stop 을 켰을 때 반복 수 (roll 이 멈춤에 붙으면 3 관절로 풂 — 오프라인 20 판 인계 끝 오차 19 판 ≤ 0.5 cm, 2 번은 모자람)


def parse_spec(tokens: Sequence[str]) -> Dict[str, np.ndarray]:
    """['L', h, j, n, 'R', h, j, n] (팔 하나만 줘도 된다) → {'left': [h,j,n], 'right': [...]} cm."""
    out: Dict[str, np.ndarray] = {}
    toks = list(tokens)
    i = 0
    while i < len(toks):
        k = str(toks[i]).upper()
        if k not in ("L", "R") or i + 3 >= len(toks):
            raise ValueError("형식: L <h> <j> <n> R <h> <j> <n> (cm, 집게 틀)")
        side = "left" if k == "L" else "right"
        if side in out:
            raise ValueError(f"{k} 를 두 번 줌")
        out[side] = np.array([float(toks[i + 1]), float(toks[i + 2]), float(toks[i + 3])])
        i += 4
    if not out:
        raise ValueError("값이 없음")
    return out


class TipOffset:
    def __init__(self, hand_of: Callable, offsets: Dict[str, np.ndarray], cap_deg: float = CAP_DEG,
                 roll_stop: Dict[str, Tuple[float, float]] = None):
        self.hand_of = hand_of                     # (side, motor dict) → (tip cm, R 3×3 열 h j n)
        self.off = {s: np.asarray(v, float) for s, v in offsets.items()}
        self.cap = math.radians(cap_deg)
        self.cap_deg = cap_deg
        # 19-x19 (기본 None = 예전 그대로): side → (아래, 위) 모터 rad. 켜면 오프셋을 낼 때 어깨 roll 을 이 안에 묶고 (논리 목표 roll 도 먼저 묶음)
        #   roll 이 멈춤에 붙으면 그 열을 빼고 나머지 관절 (pitch · yaw · 팔꿈치) 로 끝 위치를 맞춘다. 손 방향은 맞추지 않는다 (위치만, 지금과 같음)
        self.roll_stop = roll_stop

    def _stop_goal(self, side: str, goal: Dict[str, float]) -> Dict[str, float]:
        if not self.roll_stop or side not in self.roll_stop:
            return goal
        lo, hi = self.roll_stop[side]
        g = dict(goal)
        g["shoulder_roll"] = min(max(float(g["shoulder_roll"]), lo), hi)
        return g

    def active(self, side: str) -> bool:
        return side in self.off and bool(np.any(self.off[side] != 0.0))

    def apply(self, side: str, goal: Dict[str, float], gain: float, extra=None, extra_base=None) -> Tuple[Dict[str, float], dict]:
        """goal (논리 목표, 모터 값) → (보낼 목표, 정보). 정보: dq (관절별 rad), capped (관절 이름), moved_hjn (끝 이동, 목표 틀 cm), err_cm (남은 차).
        extra (18-b --place-tip-offset-cm, 기본 None = 예전 그대로): 이 팔 오프셋에 더할 값 (같은 집게 틀 h·j·n cm). 같은 IK·같은 상한.
        extra_base (18-e --carry-lift-cm, 기본 None = 예전 그대로): 더할 값을 base 틀 벡터 (cm) 로. 같은 IK·같은 상한."""
        q = {j: float(v) for j, v in goal.items()}
        zero = {"dq": {j: 0.0 for j in POS_JOINTS}, "capped": [], "moved_hjn": [0.0, 0.0, 0.0], "err_cm": 0.0}
        if extra_base is not None:
            off = self.off.get(side, np.zeros(3)) + (0.0 if extra is None else np.asarray(extra, float))
            vb = np.asarray(extra_base, float)
            if gain <= 0.0 or not (bool(np.any(off != 0.0)) or bool(np.any(vb != 0.0))):
                return q, zero
            g0, goal = goal, self._stop_goal(side, goal)
            q = {j: float(v) for j, v in goal.items()}
            tip0, R = self.hand_of(side, q)
            return self._solve(side, goal, q, tip0, R, tip0 + gain * (R @ off + vb), g0)
        if extra is None:
            if gain <= 0.0 or not self.active(side):
                return q, zero
            off = self.off[side]
        else:
            off = self.off.get(side, np.zeros(3)) + np.asarray(extra, float)
            if gain <= 0.0 or not bool(np.any(off != 0.0)):
                return q, zero
        g0, goal = goal, self._stop_goal(side, goal)
        q = {j: float(v) for j, v in goal.items()}
        tip0, R = self.hand_of(side, q)
        want = tip0 + gain * (R @ off)
        return self._solve(side, goal, q, tip0, R, want, g0)

    def apply_vec(self, side: str, goal: Dict[str, float], vec_cm) -> Tuple[Dict[str, float], dict]:
        """14-ab: base 틀 벡터 (cm) 만큼 끝을 옮긴다 (--place-lower-cm 이 (0, 0, −d) 로 쓴다). 같은 IK·같은 상한."""
        q = {j: float(v) for j, v in goal.items()}
        v = np.asarray(vec_cm, float)
        if not np.any(v != 0.0):
            return q, {"dq": {j: 0.0 for j in POS_JOINTS}, "capped": [], "moved_hjn": [0.0, 0.0, 0.0], "err_cm": 0.0}
        g0, goal = goal, self._stop_goal(side, goal)
        q = {j: float(v) for j, v in goal.items()}
        tip0, R = self.hand_of(side, q)
        return self._solve(side, goal, q, tip0, R, tip0 + v, g0)

    def _solve(self, side, goal, q, tip0, R, want, goal0=None):
        """goal0 (19-x19): roll 을 묶기 전 논리 목표 — dq 는 이것 기준 (보낼 목표 − 논리 목표). None 이면 goal."""
        lam2 = LAMBDA_CM * LAMBDA_CM
        stop = self.roll_stop.get(side) if self.roll_stop else None       # 19-x19 (켤 때만)
        for _ in range(ITERS if stop is None else ITERS_STOP):
            tip, _ = self.hand_of(side, q)
            e = want - tip
            Jm = np.empty((3, len(POS_JOINTS)))
            for k, j in enumerate(POS_JOINTS):
                q2 = dict(q)
                q2[j] += EPS
                Jm[:, k] = (self.hand_of(side, q2)[0] - tip) / EPS
            if stop is not None:                   # roll 이 멈춤에 붙어 있으면 그 열을 뺀다 (멈춤 밖으로 못 가니 나머지로)
                r_ = q["shoulder_roll"]
                if r_ <= stop[0] + 1e-6 or r_ >= stop[1] - 1e-6:
                    Jm[:, POS_JOINTS.index("shoulder_roll")] = 0.0
            dq = Jm.T @ np.linalg.solve(Jm @ Jm.T + lam2 * np.eye(3), e)
            for k, j in enumerate(POS_JOINTS):
                v = q[j] + float(dq[k])
                lo, hi = goal[j] - self.cap, goal[j] + self.cap
                if stop is not None and j == "shoulder_roll":
                    lo, hi = max(lo, stop[0]), min(hi, stop[1])
                q[j] = min(max(v, lo), hi)
        capped = [j for j in POS_JOINTS if abs(q[j] - float(goal[j])) >= self.cap - 1e-9]
        tip1, _ = self.hand_of(side, q)
        gd = goal if goal0 is None else goal0
        return q, {"dq": {j: q[j] - float(gd[j]) for j in POS_JOINTS}, "capped": capped,
                   "moved_hjn": (R.T @ (tip1 - tip0)).tolist(), "err_cm": float(np.linalg.norm(want - tip1))}
