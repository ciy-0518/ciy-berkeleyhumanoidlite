#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""인계 목표 깊이 규칙 (작업 지시 19-i, 19-h 해결안 1). replay_run.py --handover-depth-cm 이 부른다 (기본 꺼짐). 시스템 파이썬 (numpy).

19-h: 실기 인계 성공 · 실패를 가르는 것은 R 이 캔에 손 축 방향으로 들어간 깊이 s 하나였다 (엔코더 기준 실패 ≤ 1.65, 성공 ≥ 2.35 cm,
'닫히면 캔이 집게 가운데로 밀린다' 가정 B 로 실패 ≤ 1.48, 성공 3.24 ~ 3.47 cm). 가상 캔의 잡힘 경계 (s ≥ −1) 는 실물보다 느슨했다.
규칙 (모든 좌표 · 재생 · 정책에 같음):
  1. L 이 다 닫힌 때: 그 순간 L 자세 (엔코더, 없으면 명령) 와 캔 시작 자리 (판 위에 선 캔) 로 캔이 L 손에 놓인 자세를 정한다.
     B 가정 — 닫히는 집게가 캔을 집게 축 (j) 가운데로 민다: 캔 = 시작 자리 − j · 집게 축.
  2. R 이 닫으려 할 때 (그리퍼 명령 0 → 1, 아직 안 보냄): 지금 L 자세로 캔 축을 놓고 R 깊이 s (virtual_can.grip_check, 둘째 손) 를 잰다.
     s < s* 면 채울 양 = min(s* − s, 상한). R 집게 끝 오프셋 h (손 축) 에만 더한다. 이미 깊으면 0.
  3. 안전: 채운 자세에서 (a) 오프셋 IK 가 상한에 걸리거나 관절 한계로 잘림, (b) 판 면, (c) 두 그리퍼 선분 간격 < 하한 (채우기 전이 이미 낮으면 그보다 줄면) 이면
     0.25 cm 씩 줄인다. 줄인 까닭은 로그에 남긴다.
반환 값은 replay_run 이 grip_hold (그리퍼 명령 붙잡기) 와 tip_offset.apply(extra=) 로 쓴다."""
from __future__ import annotations

import math
from typing import Callable, Dict, Optional

import numpy as np

STEP_CM = 0.25
RAMP_S = 0.5          # 채우기 · 빼기에 걸리는 시간
SETTLE_S = 0.3        # 다 채운 뒤 닫기 전에 기다리는 시간 (모터가 따라오게)
GAP_MIN_CM = 5.0      # 두 그리퍼 선분 (손목 → 집게 끝) 최단 거리 하한. 19-i 오프라인 (실기 17 판): 성공 8.0 ~ 8.4, 실패 6.0 ~ 8.1 cm, 채우기로 바뀌는 양 ≤ 0.05 cm.
                      # virtual_can.strict_gap_cm (상자) 는 실물보다 커서 성공 판도 0 이라 쓰지 않는다
GRIP_LEN_CM = 13.3    # 손목 → 집게 끝 (virtual_can.GRIP_M)


def seg_dist(p1, q1, p2, q2) -> float:
    """선분 p1q1 · p2q2 최단 거리."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    c, b = float(d1 @ r), float(d1 @ d2)
    den = a * e - b * b
    s = min(max((b * f - c * e) / den, 0.0), 1.0) if den > 1e-9 else 0.0
    t = (b * s + f) / e
    if t < 0.0:
        t, s = 0.0, min(max(-c / a, 0.0), 1.0)
    elif t > 1.0:
        t, s = 1.0, min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


class HandoverDepth:
    def __init__(self, vc, can0: np.ndarray, s_star: float, max_cm: float = 3.0, gap_min_cm: float = GAP_MIN_CM):
        """vc = virtual_can.VirtualCan (이 계산 전용 — pos · rot 를 바꾼다), can0 = 캔 시작 자리 밑면 가운데 (cm, base)."""
        self.vc, self.can0 = vc, np.asarray(can0, float)
        self.s_star, self.max_cm, self.gap_min = float(s_star), float(max_cm), float(gap_min_cm)
        self.rel = None            # (회전, 위치) 캔 ← L 손 틀
        self.L_j = None

    def on_L_closed(self, L_motor: Dict[str, float]) -> dict:
        """L 이 다 닫힌 때 한 번. 반환: L 잡기 정보 (s, j, t)."""
        vc = self.vc
        tL, RL = vc.hand_of("left", L_motor)
        keep = (vc.pos.copy(), vc.rot.copy(), list(vc.holders), vc.align)
        try:
            vc.pos, vc.rot, vc.holders, vc.align = self.can0.copy(), np.eye(3), [], None
            c = vc.grip_check((tL, RL), 0.0, False)
        finally:
            vc.pos, vc.rot, vc.holders, vc.align = keep
        can = self.can0 - float(c["j"]) * RL[:, 1]            # B: 집게 축 가운데로
        self.rel = (RL.T.copy(), RL.T @ (can - tL))
        self.L_j = float(c["j"])
        return {"s": float(c["s"]), "j": float(c["j"]), "t": float(c["t"])}

    def can_pose(self, L_motor):
        tL, RL = self.vc.hand_of("left", L_motor)
        return tL + RL @ self.rel[1], RL @ self.rel[0]

    def depth(self, L_motor, R_motor) -> dict:
        """지금 L · R 자세에서 R 판정 (둘째 손, 높이 여유, 정렬 끔). s · j · t · ok · 간격."""
        vc = self.vc
        pos, rot = self.can_pose(L_motor)
        hL, hR = vc.hand_of("left", L_motor), vc.hand_of("right", R_motor)
        keep = (vc.pos.copy(), vc.rot.copy(), list(vc.holders), vc.align)
        try:
            vc.pos, vc.rot, vc.holders, vc.align = pos, rot, ["left"], None
            import virtual_can as VCAN  # type: ignore
            c = vc.grip_check(hR, VCAN.HANDOVER_HEIGHT_EXTRA_CM, True)
            gap = seg_dist(hL[0] - GRIP_LEN_CM * hL[1][:, 0], hL[0], hR[0] - GRIP_LEN_CM * hR[1][:, 0], hR[0])
        finally:
            vc.pos, vc.rot, vc.holders, vc.align = keep
        return {"s": float(c["s"]), "j": float(c["j"]), "t": float(c["t"]), "ok": bool(c["ok"]), "gap": float(gap)}

    def plan(self, L_motor, R_motor, R_apply: Callable[[float], tuple], limits_R: Dict[str, tuple], table_g: Optional[Callable] = None) -> dict:
        """채울 양 정하기. R_apply(fill) → (보낼 R 목표 (오프셋 IK 뒤, 모터 값), tip 정보) — fill 0 과의 차를 지금 R 자세에 더해 본다.
        limits_R = {관절: (lo, hi)}, table_g(R 모터 자세) → 판 면 거리 cm (없으면 안 봄)."""
        d0 = self.depth(L_motor, R_motor)
        want = min(max(0.0, self.s_star - d0["s"]), self.max_cm)
        out = {"s": d0["s"], "j": d0["j"], "gap0": d0["gap"], "want": want, "fill": 0.0, "s_after": d0["s"], "gap": d0["gap"], "why": ""}
        if want <= 0.0:
            return out
        base, _ = R_apply(0.0)
        f = want
        why = []
        while f > 1e-6:
            tq, inf = R_apply(f)
            dq = {j: tq[j] - base[j] for j in base}
            Rf = {j: float(R_motor[j]) + dq.get(j, 0.0) for j in R_motor}
            bad = []
            if inf.get("capped") or float(inf.get("err_cm", 0.0)) > 0.3:
                bad.append("오프셋 상한")
            if any(not (limits_R[j][0] - 1e-9 <= tq[j] <= limits_R[j][1] + 1e-9) for j in limits_R if j in tq):
                bad.append("관절 한계")
            if table_g is not None and table_g(Rf) < 0.0:
                bad.append("판 면")
            d = self.depth(L_motor, Rf)
            if d["gap"] < min(self.gap_min, d0["gap"]) - 1e-6:
                bad.append("두 그리퍼 간격")
            if not bad:
                out.update(fill=round(f, 3), s_after=d["s"], gap=d["gap"], why=", ".join(why))
                return out
            for b in bad:
                if b not in why:
                    why.append(b)
            f = max(0.0, f - STEP_CM)
        out.update(fill=0.0, why=", ".join(why))
        return out
