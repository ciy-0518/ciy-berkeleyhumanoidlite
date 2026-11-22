#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""물체 기준 이동 증강 (작업 지시 19-c1, 시험판 cc_analysis/anypos/aug_proto.py 를 도구로). ~/venvs/learn 에서 돌린다.

시연 한 판의 시작 · 도착 좌표를 조금 (기본 원 반지름 3 cm 안) 옮긴 새 판을 만든다.
  구간 (그리퍼 기록): L 닫기 iLc / L 이 3 cm 든 때 / R 닫기 iRc / L 열기 (인계) iLo / R 열기 (놓기) iRo.
  L 팔: 시작 옮김을 접근 중에 0 → 1, 잡고 들 때까지 1, 그 뒤 인계 (R 닫기 0.5 초 전) 까지 1 → 0. 인계는 그대로.
  R 팔: 도착 옮김을 인계 뒤 0 → 1 (놓기 1 초 전까지), 놓고 2 초 (손 빼기) 까지 1, 그 뒤 끝까지 1 → 0.
  IK: 위치 4 관절로 집게 끝을 옮긴 자리에, 남는 자유도로 손 숙임을 원래 프레임과 같게 (캔이 손 안에서 기울지 않게),
      손목 돌림으로 집게 기울기를 원래대로. 프레임마다 앞 프레임 해에서만 시작한다 (시험판은 못 풀면 원래 자세에서 다시 풀어
      해가 뛰었다 = 관절 속도 튐). 그 뒤 옮긴 몫 (새 관절 − 원래 관절) 에 짧은 이동 평균을 걸어 남은 튐을 없앤다.
걸러내기 (모두 통과해야 남긴다):
  1. 새 시작 · 도착이 19-a 범위 지도 (config/reach_19a.json) 의 "됨" 칸   — 아니면 시뮬레이션도 안 한다
  2. 잡는 앞뒤 · 놓는 앞뒤 구간: 집게 끝 오차 ≤ 0.3 cm, 손 숙임 차 ≤ 3°, 관절 한계에 붙은 프레임 0
  3. 관절 속도 ≤ 원래 최대의 1.5 배 + 30 °/s
  4. 실기 보정 여유: L 잡는 순간 (집게 끝 오프셋 h +3) · R 놓는 순간 (h +3 − 2) 에 오프셋 IK 상한 18° 안 (남은 차 ≤ 0.5 cm),
     처짐 보정을 관절 한계로 2° 넘게 덜 하지 않음, 판 면 세 검사 통과 (재생기와 같은 계산)
  4'. (19-g, --handover-margin-deg 를 줄 때만) 인계 구간 [R 닫기, L 열기] 에서 같은 보정 사슬 (집게 끝 L 3 · R 3) 잘림 0, L shoulder_roll 여유 ≥ 그 값
  4''. (19-j2, --handover-check-j 를 줄 때만) 인계 구간 깊이 · 잘림 · 관절 여유 · 간격 (handover_swap.aug_check_j)
  5. 시뮬레이션 (policy_sim, 새 좌표의 가상 캔, 엄격 판정): 성공 + 엄격 통과 + 안전장치 깎임 ≤ 5°
학습 자료는 시뮬레이션을 다시 돌린 결과 (관절 = 안전장치 뒤 명령, 캔 관측 = 새 자리의 가상 캔) 를 쓴다.
  npz 는 demo_export 꼴, meta: record = aug_<tag>, aug = {원래 판, 옮김, 새 좌표}, can_start_xy_cm · can_goal_xy_cm = 새 좌표 (조건 입력),
  options = 원래 판 것 (그리퍼 닫힘 각). learn_data.episode_files 는 meta.aug 판을 기록 이름을 줄 때만 쓴다.

    aug_demo.py --records dry_vcan_1758 … --copies 9 --out DIR [--tag x10] [--shift-cm 3] [--seed 0] [--tries 4]
    aug_demo.py --files 목록.txt …                      (한 줄에 npz 경로)
출력 (DIR): aug_<tag>_NNN.npz, tries.json (시도마다 결과 · 까닭), summary.json, files_x1.txt (입력 원본),
            files_x<F>.txt (F = --factors, 원본 + 판마다 통과한 앞의 F − 1 개).
다른 스크립트에서: import aug_demo as AD; ctx = AD.Ctx(); act, info = AD.augment(ctx, arr, ds, dg) (평가의 "옮김 + IK")."""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import learn_data as L  # noqa: E402  (sys.path 에 Lowlevel 도 넣는다)
import demo_export as DE  # noqa: E402
import policy_sim as PS  # noqa: E402
import vr_control_watchdog_biarm as W  # noqa: E402
import tip_offset as TOF  # noqa: E402
import sag_comp as SAG  # noqa: E402

P4 = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch")
JT = DE.JOINTS
REACH_MAP = TOOLS.parent / "config/reach_19a.json"
HOFF = {"left": 3.0, "right": 1.0}          # 실기 통일안: --tip-offset-cm L 3 · R 3, 놓을 때 --place-tip-offset-cm R -2 → R 은 3 − 2
KEY_ERR_CM, KEY_PITCH_DEG = 0.3, 3.0
SPEED_GAIN, SPEED_ADD = 1.5, 30.0           # 관절 속도 한도 = 원래 최대 × 1.5 + 30 °/s
CLIP_MAX_DEG = 5.0
SMOOTH_W = 5                                # 옮긴 몫 이동 평균 (프레임, 0.25 초)
HO_MARGIN_DEG = None                        # 19-g --handover-margin-deg (기본 None = 끔): 인계 구간 보정 사슬 거르기
HO_CHECK_J = False                          # 19-j2 --handover-check-j (기본 끔): 인계 깊이 · 잘림 · 관절 여유 · 간격 (handover_swap.aug_check_j)


class Fixed:
    """기록된 행동을 프레임 차례로 (관측은 무시)."""
    cond = None
    records = None
    first = None

    def __init__(self, act):
        self.a, self.i = np.asarray(act, float), 0

    def reset(self):
        self.i = 0

    def act(self, state, env):
        a = self.a[min(self.i, len(self.a) - 1)]
        self.i += 1
        return a


class Ctx:
    """시뮬레이션 환경 하나 (FK · 관절 한계 · 판), 처짐 모델, 범위 지도."""

    def __init__(self, reach_map: Path = REACH_MAP):
        import contextlib
        import io
        with contextlib.redirect_stdout(io.StringIO()):
            self.env = PS.SimEnv(strict=True)
        self.jm, self.vc, self.lim, self.table = self.env.jm, self.env.vcan, self.env.limits, self.env.table
        models = {s: W.PR.ArmModel(s) for s in PS.SIDES}
        q_of = W.motor_to_urdf_fn(self.env.ref, PS.SIDES, models)
        self.sag = SAG.SagModel.load(q_of, models)
        self.tplain = W.make_table_clear_fn(self.env.ref, PS.SIDES, self.table)
        m = json.loads(Path(reach_map).read_text(encoding="utf-8"))
        self.mxs, self.mys = np.array(m["xs"], float), np.array(m["ys"], float)
        self.mcls = {s: np.array(m[s], int) for s in ("left", "right")}

    def top(self, x_cm: float) -> float:
        return self.table.top(x_cm / 100.0) * 100.0

    def on_plate(self, x: float, y: float) -> bool:
        """19-c2a: 캔이 판 안에 다 들어오는 자리인지 (virtual_can._pair_check 와 같은 기준 — 밖이면 가상 캔이 오류를 낸다)."""
        t, r = self.table, 0.5 * float(self.table.cfg["objects"]["can"]["diameter"])
        return t.x0 * 100.0 + r <= x <= t.x1 * 100.0 - r and t.y0 * 100.0 + r <= y <= t.y1 * 100.0 - r

    def map_cls(self, side: str, x: float, y: float) -> int:
        i, j = int(np.argmin(abs(self.mxs - x))), int(np.argmin(abs(self.mys - y)))
        if abs(self.mxs[i] - x) > 0.5 + 1e-9 or abs(self.mys[j] - y) > 0.5 + 1e-9:
            return 0
        return int(self.mcls[side][i, j])


def smooth01(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def demo_xy(meta: dict):
    c = L.can_slots(meta)
    return tuple(float(v) for v in c["start_xy_cm"]), tuple(float(v) for v in c["goal_xy_cm"])


def segments(act: np.ndarray):
    """(iLc, iRc, iLo, iRo) — 그리퍼 기록에서. L 닫기 → R 닫기 → L 열기 → R 열기 차례가 아니면 None."""
    g = act[:, 10:12] > 0.5
    if not g[:, 0].any() or not g[:, 1].any():
        return None
    iLc = int(np.argmax(g[:, 0]))
    iRc = int(np.argmax(g[:, 1]))
    rest_L = ~g[iLc:, 0]
    rest_R = ~g[iRc:, 1]
    if not rest_L.any() or not rest_R.any():
        return None
    iLo, iRo = iLc + int(np.argmax(rest_L)), iRc + int(np.argmax(rest_R))
    if not (iLc < iRc < iLo < iRo):
        return None
    return iLc, iRc, iLo, iRo


def _ik(ctx: Ctx, side: str, q_prev: dict, q_orig: dict, p: np.ndarray, hz: float):
    """모터 값. 앞 프레임 해 q_prev 에서 시작해 위치 4 관절로 집게 끝 → p, 남는 자유도로 손 축 숙임 (z 성분) → hz,
    손목 돌림으로 집게 축 z 를 원래 프레임 (q_orig) 과 같게. 반환 (q, 위치 오차 cm, 숙임 차 °)."""
    vc, lim = ctx.vc, ctx.lim[side]
    q = dict(q_prev)
    for _ in range(12):
        tip, R = vc.hand_of(side, q)
        e = p - tip
        h0 = float(R[2, 0])
        if float(np.linalg.norm(e)) < 0.02 and abs(hz - h0) < 0.003:
            break
        Jm = np.empty((3, 4))
        g = np.empty(4)
        for k, j in enumerate(P4):
            q2 = dict(q)
            q2[j] += 1e-4
            t2, R2 = vc.hand_of(side, q2)
            Jm[:, k] = (t2 - tip) / 1e-4
            g[k] = (float(R2[2, 0]) - h0) / 1e-4
        dq = Jm.T @ np.linalg.solve(Jm @ Jm.T + 0.05 * np.eye(3), e)
        n = np.linalg.svd(Jm)[2][-1]                     # 위치를 안 바꾸는 방향
        a_ = float(g @ n)
        if abs(a_) > 1e-3:
            dq = dq + n * max(-0.1, min(0.1, (hz - h0) / a_))
        m = float(np.abs(dq).max())
        if m > 0.2:
            dq *= 0.2 / m
        for k, j in enumerate(P4):
            q[j] = min(max(q[j] + float(dq[k]), lim[j][0]), lim[j][1])
    jz = float(vc.hand_of(side, q_orig)[1][2, 1])
    for _ in range(4):
        z0 = float(vc.hand_of(side, q)[1][2, 1])
        q2 = dict(q)
        q2["elbow_roll"] += 1e-3
        d = (float(vc.hand_of(side, q2)[1][2, 1]) - z0) / 1e-3
        if abs(d) < 1e-6:
            break
        q["elbow_roll"] = min(max(q["elbow_roll"] + max(-0.2, min(0.2, (jz - z0) / d)), lim["elbow_roll"][0]), lim["elbow_roll"][1])
    tip, R = vc.hand_of(side, q)
    asd = lambda x: math.degrees(math.asin(max(-1.0, min(1.0, float(x)))))   # noqa: E731
    return q, float(np.linalg.norm(p - tip)), abs(asd(R[2, 0]) - asd(hz))


def augment(ctx: Ctx, arr: dict, ds, dg):
    """arr = learn_data.arrays (action URDF (N,12)). ds · dg = 시작 · 도착 옮김 (cm, 3: x, y, 판 높이 차). 반환 (새 행동 (N,12), 정보)."""
    act = np.asarray(arr["action"], float)
    N, fps = len(act), L.FPS
    seg = segments(act)
    if seg is None:
        return None, {"why": "그리퍼 차례가 L 닫기 → R 닫기 → L 열기 → R 열기 가 아님"}
    iLc, iRc, iLo, iRo = seg
    ds, dg = np.asarray(ds, float), np.asarray(dg, float)
    jm, vc = ctx.jm, ctx.vc
    qm = [[jm.to_motor(s, dict(zip(JT, act[k, 5 * i:5 * i + 5]))) for i, s in enumerate(PS.SIDES)] for k in range(N)]
    hands = [[vc.hand_of(s, qm[k][i]) for i, s in enumerate(PS.SIDES)] for k in range(N)]
    tipL = np.array([h[0][0] for h in hands])
    tipR = np.array([h[1][0] for h in hands])
    hzL = np.array([h[0][1][2, 0] for h in hands])
    hzR = np.array([h[1][1][2, 0] for h in hands])
    lift = next((k for k in range(iLc, iRc) if tipL[k, 2] - tipL[iLc, 2] >= 3.0), min(iLc + fps, iRc - 1))
    end_L = max(lift + 1, iRc - fps // 2)
    wL = np.array([smooth01(k / max(1.0, 0.6 * iLc)) if k <= lift else 1.0 - smooth01((k - lift) / max(1, end_L - lift)) for k in range(N)])
    k0 = iLo + fps // 4
    k1 = max(iLo + fps // 2, iRo - fps)
    k2 = min(N - 1, iRo + 2 * fps)
    wR = np.array([0.0 if k <= k0 else smooth01((k - k0) / max(1, k1 - k0)) if k <= k2 else 1.0 - smooth01((k - k2) / max(1, N - 1 - k2)) for k in range(N)])
    out = act.copy()
    q_prev = [dict(qm[0][0]), dict(qm[0][1])]
    err = np.zeros((N, 2))
    perr = np.zeros((N, 2))
    for k in range(N):
        for i, (s, tip, w, d, hz) in enumerate((("left", tipL, wL, ds, hzL), ("right", tipR, wR, dg, hzR))):
            if w[k] <= 0.0:
                q = dict(qm[k][i])
            else:
                q0 = dict(q_prev[i])
                q0["elbow_roll"] = qm[k][i]["elbow_roll"]
                q, err[k, i], perr[k, i] = _ik(ctx, s, q0, qm[k][i], tip[k] + w[k] * d, float(hz[k]))
            q_prev[i] = q
            u = jm.to_urdf(s, q)
            out[k, 5 * i:5 * i + 5] = [u[j] for j in JT]
    # 옮긴 몫에 이동 평균 (옮김이 있는 구간만) → 남은 튐을 없앤다. 그 뒤 실제 집게 끝 · 숙임 오차를 다시 잰다
    delta = out[:, :10] - act[:, :10]
    w_all = np.c_[np.repeat(wL[:, None], 5, 1), np.repeat(wR[:, None], 5, 1)] > 0.0
    pad = SMOOTH_W // 2
    dp = np.pad(delta, ((pad, pad), (0, 0)), mode="edge")
    sm = np.stack([dp[k:k + N] for k in range(SMOOTH_W)], 0).mean(0)
    delta = np.where(w_all, sm, 0.0)
    out[:, :10] = act[:, :10] + delta
    lims = {s: ctx.lim[s] for s in PS.SIDES}
    key = {0: (max(0, iLc - fps // 2), lift + fps // 4), 1: (max(0, iRo - fps), min(N - 1, iRo + fps // 2))}
    kerr, kperr, klim = 0.0, 0.0, 0
    asd = lambda x: math.degrees(math.asin(max(-1.0, min(1.0, float(x)))))   # noqa: E731
    for i, s in enumerate(PS.SIDES):
        a, b = key[i]
        d = ds if i == 0 else dg
        w = wL if i == 0 else wR
        hz = hzL if i == 0 else hzR
        tip0 = tipL if i == 0 else tipR
        for k in range(a, b + 1):
            q = jm.to_motor(s, dict(zip(JT, out[k, 5 * i:5 * i + 5])))
            tip, R = vc.hand_of(s, q)
            kerr = max(kerr, float(np.linalg.norm(tip - (tip0[k] + w[k] * d))))
            kperr = max(kperr, abs(asd(R[2, 0]) - asd(hz[k])))
            klim += int(any(min(q[j] - lims[s][j][0], lims[s][j][1] - q[j]) < 1e-6 for j in P4))
    sp = lambda x: float(np.degrees(np.abs(np.diff(x[:, :10], axis=0)).max()) * fps)   # noqa: E731
    info = {"seg": [iLc, lift, iRc, iLo, iRo], "frames": N, "key_ik_err_cm": round(kerr, 3), "key_pitch_err_deg": round(kperr, 2), "key_lim_frames": klim,
            "ik_err_max_cm": round(float(err.max()), 3), "max_speed_deg_s": round(sp(out), 1), "orig_speed_deg_s": round(sp(act), 1)}
    return out, info


def corr_check(ctx: Ctx, side: str, q_urdf_row: np.ndarray) -> dict:
    """실기 보정 여유: 그 자세 (URDF 5) 에 집게 끝 오프셋 (HOFF) → 처짐 보정 → 관절 한계 (재생기와 같은 차례)."""
    q = ctx.jm.to_motor(side, dict(zip(JT, q_urdf_row)))
    tip = TOF.TipOffset(ctx.vc.hand_of, {side: np.array([HOFF[side], 0.0, 0.0])}, cap_deg=18.0)
    tq, inf = tip.apply(side, q, 1.0)
    raw = ctx.sag.command_for(side, tq, 1.0)
    lim = ctx.lim[side]
    sent = {j: min(max(raw[j], lim[j][0]), lim[j][1]) for j in W.JOINT_ORDER}
    cut = math.degrees(max(abs(raw[j] - sent[j]) for j in W.JOINT_ORDER))
    act = ctx.sag.actual(side, sent)
    dq = {j: tq[j] - q[j] for j in W.JOINT_ORDER}
    g = min(c["g"] for c in (ctx.tplain(side, sent), ctx.tplain(side, act), ctx.tplain(side, {j: float(act[j]) - dq[j] for j in W.JOINT_ORDER}))) * 100.0
    return {"err_cm": round(float(inf["err_cm"]), 3), "capped": list(inf["capped"]), "sag_cut_deg": round(cut, 2), "table_g_cm": round(g, 2),
            "ok": inf["err_cm"] <= 0.5 and cut <= 2.0 and g >= 0.0}


def shift_for(ctx: Ctx, src_xy, dst_xy) -> np.ndarray:
    """자리 src → dst 옮김 (cm, 3). z = 판 높이 차."""
    return np.array([dst_xy[0] - src_xy[0], dst_xy[1] - src_xy[1], ctx.top(dst_xy[0]) - ctx.top(src_xy[0])])


def try_one(ctx: Ctx, arr: dict, src: tuple, dst: tuple, name: str, tmp: Path, sim: bool = True) -> dict:
    """한 번 시도. src · dst = ((시작 x, y), (도착 x, y)). 반환 dict: ok, why, info, summary (시뮬레이션), act."""
    r = {"name": name, "start_xy": [round(v, 2) for v in dst[0]], "goal_xy": [round(v, 2) for v in dst[1]],
         "ds": [round(dst[0][0] - src[0][0], 2), round(dst[0][1] - src[0][1], 2)], "dg": [round(dst[1][0] - src[1][0], 2), round(dst[1][1] - src[1][1], 2)]}
    moved = any(abs(v) > 1e-9 for v in r["ds"] + r["dg"])
    if moved and not (ctx.on_plate(*dst[0]) and ctx.on_plate(*dst[1])):
        return {**r, "ok": False, "why": "판 밖 (캔 반지름만큼 안쪽이 아님)"}
    if moved and (ctx.map_cls("left", *dst[0]) != 2 or ctx.map_cls("right", *dst[1]) != 2):
        return {**r, "ok": False, "why": "범위 밖 (19-a 지도의 됨 칸이 아님)"}
    if moved:
        act, info = augment(ctx, arr, shift_for(ctx, src[0], dst[0]), shift_for(ctx, src[1], dst[1]))
    else:
        act = np.asarray(arr["action"], float)
        info = {"key_ik_err_cm": 0.0, "key_pitch_err_deg": 0.0, "key_lim_frames": 0}
        if segments(act) is None:
            act, info = None, {"why": "그리퍼 차례가 L 닫기 → R 닫기 → L 열기 → R 열기 가 아님"}
    if act is None:
        return {**r, "ok": False, "why": info["why"], "info": info}
    r["info"] = info
    if moved:
        if info["key_ik_err_cm"] > KEY_ERR_CM or info["key_pitch_err_deg"] > KEY_PITCH_DEG or info["key_lim_frames"] > 0:
            return {**r, "ok": False, "why": "잡는 · 놓는 자세를 못 만듦 (IK · 관절 한계 · 손 숙임)", "act": act}
        if info["max_speed_deg_s"] > SPEED_GAIN * info["orig_speed_deg_s"] + SPEED_ADD:
            return {**r, "ok": False, "why": "관절 속도 튐", "act": act}
    iLc, _, _, iRo = segments(act)
    cL, cR = corr_check(ctx, "left", act[iLc, 0:5]), corr_check(ctx, "right", act[max(0, iRo - 1), 5:10])
    r["corr"] = {"L": cL, "R": cR}
    if moved and not (cL["ok"] and cR["ok"]):          # 그대로 재생 (옮김 없음) 은 적기만 한다
        return {**r, "ok": False, "why": "실기 보정 여유 없음 (오프셋 상한 · 처짐 보정 · 판 면)", "act": act}
    if moved and HO_MARGIN_DEG is not None:            # 19-g (켤 때만): 인계 구간 [R 닫기, L 열기] 보정 잘림 0 · L shoulder_roll 여유
        import handover_swap as HS
        hc = HS.handover_check(ctx, act, HO_MARGIN_DEG)
        r["handover"] = hc
        if not hc["ok"]:
            return {**r, "ok": False, "why": f"인계 보정 여유 없음 (잘림 {hc['cut_deg']}°, L shoulder_roll 여유 {hc['L_sr_margin_deg']}°)", "act": act}
    if moved and HO_CHECK_J:                           # 19-j2 (켤 때만)
        import handover_swap as HS
        hj = HS.aug_check_j(ctx, act, dst[0])
        r["handover_j"] = hj
        if not hj["ok"]:
            return {**r, "ok": False, "why": f"인계 검사 (19-j2): {hj['why']}", "act": act}
    if not sim:
        return {**r, "ok": True, "why": "", "act": act}
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        s = PS.rollout(Fixed(act), name, max_s=float(arr["t"][-1] - arr["t"][0]) + 6.0, out_dir=tmp, start=dst[0], goal=dst[1], strict=True,
                       first=act[0, :10].reshape(2, 5))
    st = s.get("strict") or {}
    r["sim"] = {k: s.get(k) for k in ("result", "why", "goal_dist_cm", "table_blocks", "gap_blocks", "safety_clip_max_deg", "sim_s")}
    r["sim"]["strict_flags"], r["sim"]["strict_fail"] = st.get("strict_flags"), st.get("strict_fail")
    r["act"] = act
    if s["result"] != "auto_success":
        return {**r, "ok": False, "why": f"시뮬레이션 실패 ({s['why']})"}
    if str(st.get("strict_fail", "0")) not in ("0", "", "None"):
        return {**r, "ok": False, "why": f"엄격 판정 실패 ({st.get('strict_flags')})"}
    if s.get("real") is not None:                      # 19-x (--grasp-real 일 때만): 실물 기준 판정
        r["sim"]["real"] = s["real"]
        if str(s["real"].get("real_fail", "0")) not in ("0", "", "None"):
            return {**r, "ok": False, "why": f"실물 기준 실패 ({s['real'].get('real_flags')})"}
    if s["safety_clip_max_deg"] > CLIP_MAX_DEG:
        return {**r, "ok": False, "why": f"안전장치 깎임 {s['safety_clip_max_deg']}°"}
    return {**r, "ok": True, "why": ""}


def save_aug(tmp_npz: Path, out_npz: Path, src_meta: dict, src_file: str, tag: str, idx: int, r: dict) -> None:
    """시뮬레이션이 쓴 npz → 학습용 (meta 에 aug 표시 · 원래 판 options)."""
    z = np.load(tmp_npz, allow_pickle=True)
    meta = json.loads(str(z["meta"]))
    meta.update({"record": f"aug_{tag}", "episode": idx, "success": True, "result": "auto_success", "options": src_meta.get("options"),
                 "quality": {"policy_rollout": False, "no_release": False, "aug": True,
                             "strict": {"fail": False, "flags": (r["sim"].get("strict_flags") or "").split()}},
                 "aug": {"src_record": src_meta.get("record"), "src_episode": src_meta.get("episode"), "src_file": src_file,
                         "ds_cm": r["ds"], "dg_cm": r["dg"], "start_xy_cm": r["start_xy"], "goal_xy_cm": r["goal_xy"],
                         "info": {k: v for k, v in (r.get("info") or {}).items() if k != "seg"}, "corr": r.get("corr"), "sim": r.get("sim")}})
    arrs = {k: z[k] for k in z.files if k != "meta"}
    np.savez_compressed(out_npz, meta=np.array(json.dumps(meta, ensure_ascii=False, default=float)), **arrs)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--records", nargs="+", help="기록 이름 (learn_data base 묶음 중 그 기록)")
    src.add_argument("--files", help="npz 경로 목록 파일 (한 줄에 하나)")
    ap.add_argument("--copies", type=int, default=9, help="판마다 만들 증강 수 (기본 9 = ×10)")
    ap.add_argument("--factors", default="5,10", help="목록 파일을 낼 배율들 (기본 5,10). ×1 은 늘 낸다")
    ap.add_argument("--shift-cm", type=float, default=3.0, help="시작 · 도착 옮김 원 반지름 (cm, 기본 3)")
    ap.add_argument("--tries", type=int, default=4, help="증강 하나에 쓸 시도 수 (판마다 copies × tries 까지, 기본 4)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tag", default=None, help="record 이름 aug_<tag> (기본: 출력 폴더 이름)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--reach-map", default=str(REACH_MAP))
    ap.add_argument("--handover-margin-deg", type=float, default=None,
                    help="19-g (기본 끔): 옮긴 판의 인계 구간 [R 닫기, L 열기] 에서 실기 보정 사슬 (집게 끝 L 3 · R 3 → 처짐 보정 → 관절 한계) "
                         "잘림 0 · L shoulder_roll 한계 여유 ≥ 이 값 (°) 인 판만 남긴다 (tools/handover_swap.py handover_check)")
    ap.add_argument("--handover-check-j", action="store_true",
                    help="19-j2 (기본 끔): 옮긴 판의 인계 구간이 handover_swap make2 와 같은 기준 (엔코더 상당 깊이 ≥ 3.0 cm, 보정 잘림 ≤ 0.25°, "
                         "R elbow_pitch 여유 ≥ 0.5° · 나머지 관절 ≥ 8°, 두 그리퍼 간격 ≥ 5 cm) 을 지키는 판만 남긴다")
    ap.add_argument("--grasp-mode", choices=("bottom", "top"), default="bottom",
                    help="19-x (기본 bottom): 시뮬 가상 캔 잡기 모드. top = L 위쪽 (10 ~ 14 cm) 먼저, R 아래쪽 (3 ~ 5 cm). 범위 지도는 --reach-map 으로 (config/reach_top_19x.json)")
    ap.add_argument("--grasp-real", action="store_true",
                    help="19-x (기본 끔): 시뮬 가상 캔 실물 기준 판정 (인계 깊이 · 비킴 · 누름 · 부딪힘, virtual_can REAL_*) 을 어긴 판을 버린다")
    a = ap.parse_args()
    if a.grasp_mode != "bottom" or a.grasp_real:       # 19-x (켤 때만): Ctx · 시뮬의 가상 캔에 넘긴다
        PS.VCAN_EXTRA.update({**({"grasp_mode": a.grasp_mode} if a.grasp_mode != "bottom" else {}), **({"real": True} if a.grasp_real else {})})
    global HO_MARGIN_DEG, HO_CHECK_J
    HO_MARGIN_DEG = a.handover_margin_deg
    HO_CHECK_J = bool(a.handover_check_j)
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    tmp = out / "_sim"
    tmp.mkdir(exist_ok=True)
    tag = a.tag or out.name
    if a.files:
        paths = [p.strip() for p in Path(a.files).expanduser().read_text(encoding="utf-8").splitlines() if p.strip() and not p.startswith("#")]
        eps = [{"path": p, "meta": DE.load_episode(p)["meta"]} for p in paths]
    else:
        eps = L.episode_files("base", records=a.records)
    ctx = Ctx(Path(a.reach_map))
    rng = np.random.default_rng(a.seed)
    tries, accepted, t0 = [], {}, time.time()
    idx = 0
    for n, e in enumerate(eps):
        arr = L.arrays(e["path"])
        m = e["meta"]
        name = f"{m['record']}_ep{int(m['episode']):02d}"
        src_xy = demo_xy(m)
        base = try_one(ctx, arr, src_xy, src_xy, f"{name}_k00", tmp)            # 그대로 재생 (확인)
        tries.append({k: v for k, v in base.items() if k != "act"} | {"demo": name, "k": 0})
        accepted[name] = {"file": e["path"], "replay_ok": base["ok"], "replay_why": base["why"], "aug": []}
        if not base["ok"]:
            print(f"[{n + 1}/{len(eps)}] {name}: 그대로 재생이 통과 못 함 ({base['why']}) — 증강하지 않는다", flush=True)
            continue
        k = 0
        while len(accepted[name]["aug"]) < a.copies and k < a.copies * a.tries:
            k += 1
            def disc():
                r_, t_ = a.shift_cm * math.sqrt(rng.uniform()), rng.uniform(0, 2 * math.pi)
                return r_ * math.cos(t_), r_ * math.sin(t_)
            d1, d2 = disc(), disc()
            dst = ((src_xy[0][0] + d1[0], src_xy[0][1] + d1[1]), (src_xy[1][0] + d2[0], src_xy[1][1] + d2[1]))
            r = try_one(ctx, arr, src_xy, dst, f"{name}_k{k:02d}", tmp)
            tries.append({kk: v for kk, v in r.items() if kk != "act"} | {"demo": name, "k": k})
            if r["ok"]:
                idx += 1
                f = out / f"aug_{tag}_{idx:04d}.npz"
                save_aug(tmp / f"{name}_k{k:02d}.npz", f, m, e["path"], tag, idx, r)
                accepted[name]["aug"].append(str(f))
        print(f"[{n + 1}/{len(eps)}] {name}: 증강 {len(accepted[name]['aug'])} / {a.copies} (시도 {k}) — {round(time.time() - t0)} 초", flush=True)
        (out / "tries.json").write_text(json.dumps(tries, ensure_ascii=False, indent=0, default=str), encoding="utf-8")
    for p in tmp.glob("*"):
        p.unlink()
    tmp.rmdir()
    # 목록 파일
    lists = {"x1": [v["file"] for v in accepted.values()]}
    for F in [int(x) for x in a.factors.split(",") if x.strip()]:
        lists[f"x{F}"] = [v["file"] for v in accepted.values()] + [f for v in accepted.values() for f in v["aug"][:F - 1]]
    for nm, fl in lists.items():
        (out / f"files_{nm}.txt").write_text("\n".join(fl) + "\n", encoding="utf-8")
    moved = [t for t in tries if t["k"] > 0]
    why = {}
    for t in moved:
        why[t["why"] or "통과"] = why.get(t["why"] or "통과", 0) + 1
    summ = {"tag": tag, "demos": len(eps), "replay_ok": sum(v["replay_ok"] for v in accepted.values()), "copies": a.copies, "shift_cm": a.shift_cm,
            "tries": len(moved), "accepted": sum(len(v["aug"]) for v in accepted.values()), "pass_rate": round(sum(t["ok"] for t in moved) / max(1, len(moved)), 3),
            "why": dict(sorted(why.items(), key=lambda x: -x[1])), "short": {k: len(v["aug"]) for k, v in accepted.items() if len(v["aug"]) < a.copies},
            "lists": {k: len(v) for k, v in lists.items()}, "wall_s": round(time.time() - t0, 1), "demos_detail": accepted}
    (out / "summary.json").write_text(json.dumps(summ, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    print(json.dumps({k: v for k, v in summ.items() if k != "demos_detail"}, ensure_ascii=False, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
