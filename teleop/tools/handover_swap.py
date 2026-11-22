#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""인계 구간 바꾸기 (작업 지시 19-g, 19-e 방안 가). ~/venvs/learn 에서 돌린다.

19-b 시연은 인계 때 L shoulder_roll 이 안전 한계에 가까워 (평균 3.5°) 실기 보정 (처짐 · 집게 끝) 이 한계에서 잘린다 (19-f).
시연 한 판에서 잡기 구간 (L 이 3 cm 들 때까지) 과 놓기 구간 (L 이 연 뒤 R) 은 그대로 두고, 그 사이 인계 구간만
옛 시연 (act_multi_xy 학습 판) 의 인계 궤적 하나 (기준) 로 바꾼다.

  구간 (그리퍼 기록, aug_demo.segments): L 닫기 iLc · L 3 cm 듦 lift · R 닫기 iRc · L 열기 iLo · R 열기 iRo.
  핵심 = [iRc − 1 초, iLo + 0.5 초]. 기준의 같은 구간을 시간 맞춤 (닻: 핵심 시작 · R 닫기 · L 열기 · 핵심 끝, 사이는 선형) 으로 얹는다
    → 그리퍼가 닫히고 여는 프레임은 원래 그대로 (인계 구간 안의 상대 위치 유지), 그 순간의 팔 자세는 기준의 같은 순간.
  이음: 관절 공간. q = 원래 + w · (기준 − 원래), w 는 lift → 핵심 시작 0 → 1, 핵심 끝 → (iLo + 1.5 초, 놓기 1 초 전까지) 1 → 0 (smoothstep).
  관절 속도가 원래 시연 최대를 넘으면 [lift, 이음 끝] 을 고르게 늘인다 (1.0 ~ 2.5 배, 그리퍼 사건의 상대 위치 그대로).
  결과 행동을 policy_sim 으로 다시 돌려 (가상 캔, 엄격 판정) 학습 자료 꼴 npz 를 만든다 (aug_demo 와 같은 방식).

검사 (handover_check · check_episode):
  가상 캔 재판정 (성공 · 엄격 · 안전장치 깎임 ≤ 5°), 인계 깊이 s (R 닫는 순간, 명령 자세),
  실기 보정 사슬 모의 (집게 끝 오프셋 L 3 · R 3 → 기본 처짐 보정 → 관절 한계, 재생기와 같은 차례): 인계 구간 잘림 0, L shoulder_roll 여유 ≥ 8°,
  처짐 모델 예측 자세로 본 R 깊이 s ≥ 1.5 cm, 판 면 (보낸 명령 · 실제 예측 · 오프셋 뺀 실제, aug_demo.corr_check 와 같은 셋) ≥ 0.

    handover_swap.py pick --out DIR                       기준 고르기 (옛 시연 목록 = multi_xy_meas source_index)
    handover_swap.py make --files 목록.txt (--ref <npz> | --refs DIR/pick.json) --out DIR [--tag ho19g]
      --refs: 시연마다 기준을 고른다. 옛 시연 중 보정 사슬 조건 (잘림 0 · L 여유 ≥ 10° · 예측 s ≥ 2 · 판 면) 을 만족하는 판 가운데,
              그 시연의 L 잡기 (캔이 L 손에 놓인 자세) 로 옮겨도 기준의 R 자세가 가상 캔 판정을 통과하는 판을 실기 L1R1 거리 차례로 (8 개까지) 해 본다.
다른 스크립트에서: import handover_swap as HS; HS.handover_check(ctx, act, margin_deg) (aug_demo --handover-margin-deg)."""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import aug_demo as AD  # noqa: E402  (learn_data · policy_sim · 워치독을 먼저 부른다)
import learn_data as L  # noqa: E402
import demo_export as DE  # noqa: E402
import policy_sim as PS  # noqa: E402
import virtual_can as VCAN  # noqa: E402
import tip_offset as TOF  # noqa: E402
import vr_control_watchdog_biarm as W  # noqa: E402

JT = DE.JOINTS
FPS = L.FPS
PRE_S, POST_S, OUT_S = 1.0, 0.5, 1.5          # 핵심 = [R 닫기 − PRE, L 열기 + POST], 이음 끝 = L 열기 + OUT (놓기 1 초 전까지)
HOFF = {"left": 3.0, "right": 3.0}           # 인계 때 실기 오프셋: --tip-offset-cm L 3 0 0 R 3 0 0 (놓기 오프셋 R −2 는 인계 뒤라 보수적으로 뺌)
MARGIN_DEG = 8.0
S_MIN_CM = 1.5
CUT_TOL_DEG = 0.01
STRETCH = [1.0 + 0.1 * k for k in range(16)]
OLD_SOURCE = Path.home() / "cc_analysis/learn/lerobot/multi_xy_meas/source_index.json"
OLD_DIR = Path.home() / "cc_analysis/learn/data/multi_20261113"
L1R1_REAL = {"left": np.array([28.47, -0.25, 64.52]), "right": np.array([24.43, 1.00, 73.46])}   # 19-e 실기 L1R1 (성공) 논리 자세, R 닫는 순간


def smooth01(x: float) -> float:
    return AD.smooth01(x)


def lift_of(ctx, act: np.ndarray, iLc: int, iRc: int) -> int:
    """aug_demo.augment 와 같은 'L 이 3 cm 든 때'."""
    z = [ctx.vc.hand_of("left", ctx.jm.to_motor("left", dict(zip(JT, act[k, 0:5]))))[0][2] for k in range(iLc, iRc)]
    return next((iLc + i for i, v in enumerate(z) if v - z[0] >= 3.0), min(iLc + FPS, iRc - 1))


def speed(act: np.ndarray) -> float:
    return float(np.degrees(np.abs(np.diff(act[:, :10], axis=0)).max()) * FPS)


def motor_rows(ctx, act: np.ndarray, side: str):
    i = 0 if side == "left" else 1
    return [ctx.jm.to_motor(side, dict(zip(JT, act[k, 5 * i:5 * i + 5]))) for k in range(len(act))]


# --------------------------------------------------------------------------- 실기 보정 사슬 모의
_tof = {}


def chain(ctx, side: str, q: dict) -> dict:
    """모터 자세 q → 집게 끝 오프셋 (HOFF, 상한 18°) → 처짐 보정 (기본 모델) → 관절 한계 (재생기 차례).
    반환: cut (한계로 덜 한 보정 °), sr_margin (논리 자세의 shoulder_roll 한계까지 °), sr_margin_raw (보정한 명령의), pred (실제 예측 모터), g (판 면 cm)."""
    if side not in _tof:
        _tof[side] = TOF.TipOffset(ctx.vc.hand_of, {side: np.array([HOFF[side], 0.0, 0.0])}, cap_deg=18.0)
    tq, inf = _tof[side].apply(side, q, 1.0)
    raw = ctx.sag.command_for(side, tq, 1.0)
    lim = ctx.lim[side]
    sent = {j: min(max(raw[j], lim[j][0]), lim[j][1]) for j in W.JOINT_ORDER}
    cut = math.degrees(max(abs(raw[j] - sent[j]) for j in W.JOINT_ORDER))
    lo, hi = lim["shoulder_roll"]
    pred = ctx.sag.actual(side, sent)
    dq = {j: tq[j] - q[j] for j in W.JOINT_ORDER}
    g = min(c["g"] for c in (ctx.tplain(side, sent), ctx.tplain(side, pred), ctx.tplain(side, {j: float(pred[j]) - dq[j] for j in W.JOINT_ORDER}))) * 100.0
    return {"cut": cut, "sr_margin": math.degrees(min(q["shoulder_roll"] - lo, hi - q["shoulder_roll"])),
            "sr_margin_raw": math.degrees(min(raw["shoulder_roll"] - lo, hi - raw["shoulder_roll"])), "pred": pred, "g": g,
            "tip_err_cm": float(inf["err_cm"])}


def handover_check(ctx, act: np.ndarray, margin_deg: float = MARGIN_DEG) -> dict:
    """인계 구간 [R 닫기, L 열기] 보정 사슬: 두 팔 잘림 0 · L shoulder_roll 여유 ≥ margin. aug_demo --handover-margin-deg 가 부른다."""
    seg = AD.segments(act)
    if seg is None:
        return {"ok": False, "why": "그리퍼 차례"}
    _, iRc, iLo, _ = seg
    cut, marg, g = 0.0, 1e9, 1e9
    for k in range(iRc, iLo + 1):
        for i, s in enumerate(PS.SIDES):
            c = chain(ctx, s, ctx.jm.to_motor(s, dict(zip(JT, act[k, 5 * i:5 * i + 5]))))
            cut = max(cut, c["cut"])
            g = min(g, c["g"])
            if s == "left":
                marg = min(marg, c["sr_margin"])
    ok = cut <= CUT_TOL_DEG and marg >= margin_deg
    return {"ok": ok, "cut_deg": round(cut, 3), "L_sr_margin_deg": round(marg, 2), "table_g_cm": round(g, 2)}


# --------------------------------------------------------------------------- 인계 깊이 s (19-e 판정과 같음)
def quat_R(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def depth(ctx, Lm: list, Rm: list, iLc: int, iRc: int, can_pos: np.ndarray, can_rot: np.ndarray) -> dict:
    """L 닫는 순간의 캔 (can_pos · can_rot) 을 L 손에 붙이고 R 닫는 순간 R 판정 (둘째 손, 높이 여유, 정렬 끔 = 정책 실행과 같음)."""
    vc = ctx.vc
    keep = (vc.pos.copy(), vc.rot.copy(), list(vc.holders), vc.align)
    try:
        tL, RL = vc.hand_of("left", Lm[iLc])
        rel_rot, rel_pos = RL.T @ can_rot, RL.T @ (can_pos - tL)
        t2, R2 = vc.hand_of("left", Lm[iRc])
        vc.rot, vc.pos, vc.holders, vc.align = R2 @ rel_rot, t2 + R2 @ rel_pos, ["left"], None
        c = vc.grip_check(vc.hand_of("right", Rm[iRc]), VCAN.HANDOVER_HEIGHT_EXTRA_CM, True)
        return {"s": round(float(c["s"]), 2), "j": round(float(c["j"]), 2), "t": round(float(c["t"]), 2), "ok": bool(c["ok"]), "why": c["reason"],
                "can_tilt": round(float(np.degrees(np.arccos(np.clip(vc.rot[2, 2], -1, 1)))), 1)}
    finally:
        vc.pos, vc.rot, vc.holders, vc.align = keep


def check_episode(ctx, npz: Path, win=None) -> dict:
    """시뮬레이션 결과 npz 하나 검사. win = (a, b) 보정 사슬 · 판 면을 볼 프레임 (기본 [R 닫기, L 열기])."""
    z = np.load(npz, allow_pickle=True)
    g = np.asarray(z["grip"], float)
    act = np.concatenate([np.asarray(z["q_cmd"], float).reshape(len(g), 10), g], 1)
    seg = AD.segments(act)
    if seg is None:
        return {"ok": False, "why": "그리퍼 차례"}
    iLc, iRc, iLo, iRo = seg
    Lm, Rm = motor_rows(ctx, act, "left"), motor_rows(ctx, act, "right")
    can_pos, can_rot = np.asarray(z["can_pos"][max(0, iLc - 1)], float), quat_R(z["can_quat"][max(0, iLc - 1)])
    d_cmd = depth(ctx, Lm, Rm, iLc, iRc, can_pos, can_rot)
    a, b = win if win is not None else (iRc, iLo)
    cut, marg, marg_raw, gmin = 0.0, 1e9, 1e9, 1e9
    predL, predR = list(Lm), list(Rm)
    for k in sorted(set(range(a, b + 1)) | {iLc, iRc}):
        cl, cr = chain(ctx, "left", Lm[k]), chain(ctx, "right", Rm[k])
        predL[k], predR[k] = cl["pred"], cr["pred"]
        if a <= k <= b:
            cut = max(cut, cl["cut"], cr["cut"])
            marg, marg_raw = min(marg, cl["sr_margin"]), min(marg_raw, cl["sr_margin_raw"])
            gmin = min(gmin, cl["g"], cr["g"])
    d_pred = depth(ctx, predL, predR, iLc, iRc, can_pos, can_rot)
    tl, tr = ctx.vc.hand_of("left", Lm[iRc])[0], ctx.vc.hand_of("right", Rm[iRc])[0]
    return {"seg": [iLc, iRc, iLo, iRo], "s_cmd": d_cmd["s"], "s_pred": d_pred["s"], "R_ok_pred": d_pred["ok"], "can_tilt": d_cmd["can_tilt"],
            "cut_deg": round(cut, 3), "L_sr_margin_deg": round(marg, 2), "L_sr_margin_raw_deg": round(marg_raw, 2), "table_g_cm": round(gmin, 2),
            "L_tip": [round(float(v), 2) for v in tl], "R_tip": [round(float(v), 2) for v in tr],
            "d_L1R1_cm": round(float(np.linalg.norm(tl - L1R1_REAL["left"]) + np.linalg.norm(tr - L1R1_REAL["right"])), 2)}


# --------------------------------------------------------------------------- 바꾸기
def build(ctx, act: np.ndarray, ref: np.ndarray, stretch: float = 1.0):
    """반환 (새 행동, 정보) 또는 (None, {why})."""
    seg, rseg = AD.segments(act), AD.segments(ref)
    if seg is None or rseg is None:
        return None, {"why": "그리퍼 차례"}
    iLc, iRc, iLo, iRo = seg
    jLc, jRc, jLo, jRo = rseg
    lift = lift_of(ctx, act, iLc, iRc)
    P, Q, O = int(PRE_S * FPS), int(POST_S * FPS), int(OUT_S * FPS)
    cs, ce = max(iRc - P, lift + FPS // 2), iLo + Q
    e = min(iLo + O, iRo - FPS)
    jcs, jce = max(jRc - P, jLc + 1), min(jLo + Q, len(ref) - 1)
    if not (lift < cs < iRc < iLo < ce < e):
        return None, {"why": f"구간이 짧음 (lift {lift}, 핵심 {cs}~{ce}, 끝 {e})"}
    xs, ys = [cs, iRc, iLo, ce], [jcs, jRc, jLo, jce]
    fr = np.arange(len(act))
    rf = np.arange(len(ref))

    def at(arr, idx, u):
        return np.array([np.interp(u, idx, arr[:, c]) for c in range(10)])

    M = int(round((e - lift) * stretch))
    us = lift + np.arange(M + 1) * (e - lift) / M
    mid = np.zeros((M + 1, 12))
    for n, u in enumerate(us):
        v = float(np.interp(min(max(u, cs), ce), xs, ys))
        tgt, org = at(ref, rf, v), at(act, fr, u)
        w = smooth01((u - lift) / (cs - lift)) if u < cs else (1.0 if u <= ce else 1.0 - smooth01((u - ce) / (e - ce)))
        mid[n, :10] = org + w * (tgt - org)
        mid[n, 10:] = act[min(int(math.floor(u + 1e-9)), len(act) - 1), 10:]
    new = np.concatenate([act[:lift], mid, act[e + 1:]], 0)
    orig_at = np.concatenate([act[:lift], np.array([np.r_[at(act, fr, u), act[min(int(math.floor(u + 1e-9)), len(act) - 1), 10:]] for u in us]), act[e + 1:]], 0)
    changed = int((np.degrees(np.abs(new[:, :10] - orig_at[:, :10])).max(1) > 0.1).sum())
    k = lambda i: lift + int(round((i - lift) * stretch))    # noqa: E731  원래 프레임 → 새 프레임 (구간 안)
    info = {"stretch": round(stretch, 2), "frames_orig": len(act), "frames_new": len(new), "region": [lift, e], "region_new": [lift, lift + M],
            "core_orig": [cs, ce], "ref_core": [jcs, jce], "seg_new": [iLc, k(iRc), k(iLo), iRo - e + lift + M],
            "changed_frames": changed, "changed_pct": round(100.0 * changed / len(new), 1),
            "max_speed_deg_s": round(speed(new), 1), "orig_speed_deg_s": round(speed(act), 1)}
    return new, info


def make_one(ctx, path: str, ref: np.ndarray, out: Path, tmp: Path, tag: str, ref_info: dict = None) -> dict:
    arr = L.arrays(path)
    m = arr["meta"]
    act = np.asarray(arr["action"], float)
    name = f"{m['record']}_ep{int(m['episode']):02d}"
    r = {"name": name, "src": path}
    new = info = None
    for st in STRETCH:
        new, info = build(ctx, act, ref, st)
        if new is None:
            return {**r, "ok": False, "why": info["why"]}
        if info["max_speed_deg_s"] <= info["orig_speed_deg_s"] + 1e-6:
            break
    r["info"] = info
    if info["max_speed_deg_s"] > info["orig_speed_deg_s"] + 1e-6:
        return {**r, "ok": False, "why": f"관절 속도 (늘여도 {info['max_speed_deg_s']} > {info['orig_speed_deg_s']} °/s)"}
    sxy, gxy = AD.demo_xy(m)
    nm = f"{tag}_{name}"
    with contextlib.redirect_stdout(io.StringIO()):
        s = PS.rollout(AD.Fixed(new), nm, max_s=len(new) / FPS + 6.0, out_dir=tmp, start=sxy, goal=gxy, strict=True, first=new[0, :10].reshape(2, 5))
    stv = s.get("strict") or {}
    r["sim"] = {"result": s.get("result"), "why": s.get("why"), "goal_dist_cm": s.get("goal_dist_cm"), "clip_max_deg": s.get("safety_clip_max_deg"),
                "table_blocks": s.get("table_blocks"), "gap_blocks": s.get("gap_blocks"), "strict_flags": stv.get("strict_flags"), "strict_fail": stv.get("strict_fail")}
    sim_ok = s.get("result") == "auto_success" and str(stv.get("strict_fail", "0")) in ("0", "", "None") and float(s.get("safety_clip_max_deg") or 0) <= AD.CLIP_MAX_DEG
    c = check_episode(ctx, tmp / f"{nm}.npz")
    r["check"] = c
    fails = []
    if not sim_ok:
        fails.append(f"가상 캔 ({s.get('why')}, 엄격 {stv.get('strict_flags')}, 깎임 {s.get('safety_clip_max_deg')}°)")
    if "cut_deg" not in c:
        fails.append(f"검사 못 함 ({c.get('why')})")
    elif c["cut_deg"] > CUT_TOL_DEG:
        fails.append(f"보정 잘림 {c['cut_deg']}°")
    if "cut_deg" in c and c["L_sr_margin_deg"] < MARGIN_DEG:
        fails.append(f"L shoulder_roll 여유 {c['L_sr_margin_deg']}°")
    if "cut_deg" in c and c["s_pred"] < S_MIN_CM:
        fails.append(f"예측 깊이 s {c['s_pred']} cm")
    if "cut_deg" in c and c["table_g_cm"] < 0:
        fails.append(f"판 면 {c['table_g_cm']} cm")
    r["ok"], r["why"] = not fails, "; ".join(fails)
    z = np.load(tmp / f"{nm}.npz", allow_pickle=True)
    meta = json.loads(str(z["meta"]))
    meta.update({"record": f"{tag}_{m['record']}", "episode": int(m["episode"]), "success": sim_ok, "result": s.get("result"), "options": m.get("options"),
                 "quality": {"policy_rollout": False, "no_release": False, "handover_swap": True,
                             "strict": {"fail": not sim_ok, "flags": (stv.get("strict_flags") or "").split()}},
                 "handover_swap": {"src_record": m.get("record"), "src_episode": m.get("episode"), "src_file": path, "ref": ref_info or REF_INFO,
                                   "info": info, "check": c, "sim": r["sim"]}})
    f = out / f"{nm}.npz"
    np.savez_compressed(f, meta=np.array(json.dumps(meta, ensure_ascii=False, default=float)), **{k: z[k] for k in z.files if k != "meta"})
    r["file"] = str(f)
    return r


REF_INFO: dict = {}
MAX_REF_TRIES = 8


def good_refs(pick_rows: list) -> list:
    """기준 후보: 보정 사슬 잘림 0 · L 여유 ≥ 10° · 예측 s ≥ 2.0 · 판 면 ≥ 0 (pick 과 같은 조건)."""
    return [r for r in pick_rows if r.get("cut_deg", 99) <= CUT_TOL_DEG and r.get("L_sr_margin_deg", 0) >= MARGIN_DEG + 2
            and r.get("s_pred", 0) >= S_MIN_CM + 0.5 and r.get("table_g_cm", -1) >= 0]


def feasible(ctx, src_path: str, refs: list) -> list:
    """시연의 L 잡기 (원래 npz 의 L 닫는 순간 자세 · 가상 캔) 를 기준 판의 인계 L 자세에 옮겨 놓고, 기준 판의 R 닫는 자세가
    가상 캔 판정 (시뮬레이션과 같은 정렬 조건) 을 통과하는 기준만, 실기 L1R1 거리 차례로."""
    vc = ctx.vc
    z = np.load(src_path, allow_pickle=True)
    q = np.asarray(z["q_cmd"], float).reshape(len(z["t"]), 10)
    g = np.asarray(z["grip"], float)
    iLc = AD.segments(np.concatenate([q, g], 1))[0]
    tL, RL = vc.hand_of("left", ctx.jm.to_motor("left", dict(zip(JT, q[iLc, 0:5]))))
    cp, cr = np.asarray(z["can_pos"][iLc - 1], float), quat_R(z["can_quat"][iLc - 1])
    rel_rot, rel_pos = RL.T @ cr, RL.T @ (cp - tL)
    out = []
    for r in refs:
        ra = r["_act"]
        jRc = AD.segments(ra)[1]
        t2, R2 = vc.hand_of("left", ctx.jm.to_motor("left", dict(zip(JT, ra[jRc, 0:5]))))
        keep = (vc.pos.copy(), vc.rot.copy(), list(vc.holders))
        try:
            vc.rot, vc.pos, vc.holders = R2 @ rel_rot, t2 + R2 @ rel_pos, ["left"]
            c = vc.grip_check(vc.hand_of("right", ctx.jm.to_motor("right", dict(zip(JT, ra[jRc, 5:10])))), VCAN.HANDOVER_HEIGHT_EXTRA_CM, True)
        finally:
            vc.pos, vc.rot, vc.holders = keep
        if c["ok"]:
            out.append({**r, "pre_s": round(float(c["s"]), 2), "pre_align_deg": round(float(c.get("align_deg", -1)), 1)})
    return sorted(out, key=lambda x: x["d_L1R1_cm"])


def cmd_pick(a) -> int:
    ctx = AD.Ctx()
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    files = sorted({x["file"] for x in json.loads(OLD_SOURCE.read_text(encoding="utf-8"))})
    rows = []
    for f in files:
        p = OLD_DIR / f
        meta = DE.load_episode(str(p))["meta"]
        c = check_episode(ctx, p)
        rows.append({"file": str(p), "name": p.stem, "slot": f"{meta.get('start_slot')}>{meta.get('goal_slot')}", **c})
        print(f"{p.stem} {rows[-1]['slot']}: L1R1 거리 {c.get('d_L1R1_cm')} cm, 잘림 {c.get('cut_deg')}°, L 여유 {c.get('L_sr_margin_deg')}°, "
              f"s 명령 {c.get('s_cmd')} · 예측 {c.get('s_pred')}, 판 {c.get('table_g_cm')}", flush=True)
    good = [r for r in rows if r.get("cut_deg", 99) <= CUT_TOL_DEG and r.get("L_sr_margin_deg", 0) >= MARGIN_DEG + 2 and r.get("s_pred", 0) >= S_MIN_CM + 0.5
            and r.get("table_g_cm", -1) >= 0]
    good.sort(key=lambda r: r["d_L1R1_cm"])
    pick = good[0] if good else None
    (out / "pick.json").write_text(json.dumps({"rule": "보정 사슬 잘림 0 · L 여유 ≥ 10° · 예측 s ≥ 2.0 · 판 면 ≥ 0 인 판 중 실기 L1R1 (성공) 논리 자세에 가장 가까운 판 "
                                                       "(R 닫는 순간 L · R 집게 끝 거리 합)", "pick": pick, "rows": rows}, ensure_ascii=False, indent=1, default=float),
                                    encoding="utf-8")
    print("고름:", pick and pick["name"], pick and pick["d_L1R1_cm"])
    return 0 if pick else 1


def cmd_make(a) -> int:
    global REF_INFO
    ctx = AD.Ctx()
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    tmp = out / "_sim"
    tmp.mkdir(exist_ok=True)
    refs = None
    if a.ref:
        ref_arr = L.arrays(a.ref)
        ref = np.asarray(ref_arr["action"], float)
        REF_INFO = {"file": str(a.ref), "record": ref_arr["meta"].get("record"), "episode": ref_arr["meta"].get("episode"), "seg": list(AD.segments(ref))}
    else:
        refs = good_refs(json.loads(Path(a.refs).expanduser().read_text(encoding="utf-8"))["rows"])
        for r in refs:
            r["_act"] = np.asarray(L.arrays(r["file"])["action"], float)
        REF_INFO = {"mode": "시연마다", "candidates": [r["name"] for r in refs]}
    paths = [p.strip() for p in Path(a.files).expanduser().read_text(encoding="utf-8").splitlines() if p.strip() and not p.startswith("#")]
    res, t0 = [], time.time()
    for n, p in enumerate(paths):
        if refs is None:
            r = make_one(ctx, p, ref, out, tmp, a.tag)
        else:
            fe = feasible(ctx, p, refs)
            r = {"name": Path(p).stem, "src": p, "ok": False, "why": "R 기준 자세가 이 시연의 캔 (L 잡기) 에 맞는 기준 판이 없음", "tried": []}
            for cand in fe[:MAX_REF_TRIES]:
                ri = {"file": cand["file"], "name": cand["name"], "slot": cand["slot"], "d_L1R1_cm": cand["d_L1R1_cm"], "pre_s": cand["pre_s"],
                      "seg": list(AD.segments(cand["_act"]))}
                r = make_one(ctx, p, cand["_act"], out, tmp, a.tag, ri)
                r["ref"] = ri
                r["tried"] = [t["name"] for t in fe[:MAX_REF_TRIES]]
                if r["ok"]:
                    break
            r["n_feasible"] = len(fe)
        res.append(r)
        c = r.get("check") or {}
        print(f"[{n + 1}/{len(paths)}] {r['name']}: {'통과' if r['ok'] else '실패 — ' + r['why']} | 기준 {(r.get('ref') or {}).get('name')} 늘임 {(r.get('info') or {}).get('stretch')} "
              f"바뀐 {(r.get('info') or {}).get('changed_pct')} % · s 명령 {c.get('s_cmd')} 예측 {c.get('s_pred')} · 잘림 {c.get('cut_deg')}° · "
              f"L 여유 {c.get('L_sr_margin_deg')}° · 판 {c.get('table_g_cm')} — {round(time.time() - t0)} 초", flush=True)
        (out / "make.json").write_text(json.dumps({"ref": REF_INFO, "rows": res}, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    for q in tmp.glob("*"):
        q.unlink()
    tmp.rmdir()
    ok = [r for r in res if r["ok"]]
    (out / "files_ho.txt").write_text("\n".join(r["file"] for r in ok) + "\n", encoding="utf-8")
    print(f"통과 {len(ok)} / {len(res)} → {out / 'files_ho.txt'}")
    return 0 if len(ok) == len(res) else 1


# --------------------------------------------------------------------------- 19-j make2 (기본 동작과 무관한 새 모드)
J_DELTA_CM = 1.30          # 엔코더 상당 깊이 = 논리 깊이 + Δ. 19-j: 실기 L1R1 자세 8 판 (오늘 L1R1 · 11-19 ~ 11-20 L1→R1) 1.30 ± 0.08 cm
J_TARGET_ENC_CM = 3.3      # 목표 (엔코더 상당) — 19-h 성공 판 (B) 3.24 ~ 3.47 의 가운데
J_MIN_ENC_CM = 3.0         # 검사 하한 (엔코더 상당)
J_MARGIN_DEG = 8.0         # 인계 구간 관절 한계 여유 (두 팔 모든 관절, 논리 자세)
J_GAP_MIN_CM = 5.0         # 두 그리퍼 선분 간격 (handover_depth.GAP_MIN_CM 과 같음)
J_GRASP_T_CM = 12.9        # R 이 캔을 잡는 높이 (밑면에서, 19-h 실기 성공 판 중앙 12.97 cm) — 기준 판 높이를 그대로 쓰면 L 이 낮게 쥔 시연에서 곧은 구간 밖
J_REF_LOG = Path.home() / "teleop-docs/_teleop/logs/real19d_f_L1R1_multi.csv"
J_REF_REF = "robot_ref_20261122_010107.json"


def ref_from_log(path: Path, ref_name: str) -> np.ndarray:
    """실기 로그 → (N,12) URDF 행동 (20 Hz). 관절 = 논리 목표 (cmd + sag 열 = 정책이 낸 목표, 집게 끝 오프셋 · 처짐 보정 전 —
    재생기가 sag 열에 '논리 − 보낸' 을 적으므로 엔코더를 거꾸로 풀 필요 없이 그대로 되돌린다), 시간 = 정책 시간 replay_t (grip_wait 동안 멈춤),
    그리퍼 = grip_L · grip_R 명령."""
    import csv as _csv
    ref = W.PR.RobotRef.load(DE.LOGS / ref_name)
    jm = DE.JointMap(ref)
    tau, Q, G = [], [], []
    for r in _csv.DictReader(Path(path).open(encoding="utf-8", errors="ignore")):
        if r.get("auto_stage") not in ("play", "grip_wait"):
            continue
        try:
            tt = float(r["replay_t"])
            q = []
            for s in PS.SIDES:
                m = {j: float(r[f"{s}_{j}_cmd"]) + (float(r[f"{s}_{j}_sag"]) if (r.get(f"{s}_{j}_sag") or "").strip() else 0.0) for j in JT}
                u = jm.to_urdf(s, m)
                q += [u[j] for j in JT]
        except (KeyError, ValueError):
            continue
        if tau and tt <= tau[-1] + 1e-9:
            continue                                           # grip_wait 동안 같은 시각 — 처음 것만
        tau.append(tt)
        Q.append(q)
        G.append([1.0 if r.get("grip_L") == "1" else 0.0, 1.0 if r.get("grip_R") == "1" else 0.0])
    tau, Q, G = np.array(tau), np.array(Q), np.array(G)
    grid = np.arange(0.0, tau[-1], 1.0 / FPS)
    out = np.zeros((len(grid), 12))
    for c in range(10):
        out[:, c] = np.interp(grid, tau, Q[:, c])
    idx = np.clip(np.searchsorted(tau, grid, side="right") - 1, 0, len(tau) - 1)
    out[:, 10:] = G[idx]
    return out


def _hdm(ctx, start_xy, s_target):
    import handover_depth as HDM
    x, y = float(start_xy[0]), float(start_xy[1])
    return HDM.HandoverDepth(ctx.vc, np.array([x, y, ctx.top(x)]), s_target, 3.0, J_GAP_MIN_CM)


def depth_B(ctx, act: np.ndarray, start_xy, seg=None) -> dict:
    """논리 자세의 R 깊이 (19-h B 가정, handover_depth 와 같은 계산): L 닫기 + 0.5 초의 L 자세로 캔 (시작 자리 → 집게 축 가운데) 을 붙이고, R 닫기 프레임에서 잰다."""
    seg = seg or AD.segments(act)
    iLc, iRc = seg[0], seg[1]
    lcd = min(iLc + FPS // 2, iRc)
    hd = _hdm(ctx, start_xy, J_TARGET_ENC_CM - J_DELTA_CM)
    hd.on_L_closed(ctx.jm.to_motor("left", dict(zip(JT, act[lcd, 0:5]))))
    return hd.depth(ctx.jm.to_motor("left", dict(zip(JT, act[iRc, 0:5]))), ctx.jm.to_motor("right", dict(zip(JT, act[iRc, 5:10]))))


def _ik_align(ctx, side: str, q0: dict, p: np.ndarray, zc: np.ndarray, w: float = 10.0, iters: int = 40) -> tuple:
    """5 관절 (위치 4 + 손목) IK: 집게 끝 → p, 손 축 · 집게 축이 캔 축 zc 에 수직 (= 셋째 축이 캔 축과 나란 — 가상 캔 정렬 판정).
    감쇠 최소 제곱, 관절 한계 안. 반환 (q, 위치 오차 cm, 정렬 어긋남 °)."""
    vc, lim = ctx.vc, ctx.lim[side]
    q = dict(q0)

    def res(qq):
        tip, R = vc.hand_of(side, qq)
        return np.r_[p - tip, -w * float(R[:, 0] @ zc), -w * float(R[:, 1] @ zc)]
    for _ in range(iters):
        r0 = res(q)
        if np.linalg.norm(r0[:3]) < 0.01 and abs(r0[3]) < 0.01 * w and abs(r0[4]) < 0.01 * w:
            break
        Jm = np.empty((5, 5))
        for k, j in enumerate(JT):
            q2 = dict(q)
            q2[j] += 1e-4
            Jm[:, k] = -(res(q2) - r0) / 1e-4
        dq = Jm.T @ np.linalg.solve(Jm @ Jm.T + 0.02 * np.eye(5), r0)
        m = float(np.abs(dq).max())
        if m > 0.15:
            dq *= 0.15 / m
        for k, j in enumerate(JT):
            q[j] = min(max(q[j] + float(dq[k]), lim[j][0]), lim[j][1])
    tip, R = vc.hand_of(side, q)
    mis = math.degrees(math.asin(min(1.0, math.hypot(float(R[:, 0] @ zc), float(R[:, 1] @ zc)))))
    return q, float(np.linalg.norm(p - tip)), mis


J_WRIST_MAX_DEG = 15.0     # L 손목 (elbow_roll) 을 돌려 캔을 R 쪽으로 가져가는 양 상한 — 실기 · 옛 시연 성공 인계의 L 손목 범위 (−37 ~ −4.5°, 약 30°) 의 절반
J_WRIST_STEP_DEG = 1.0
J_R_ELBOW_MIN_DEG = 0.5    # R elbow_pitch 한계 여유 하한 (실기 성공 판 0.7 ~ 0.9°), 나머지 관절은 J_MARGIN_DEG
J_CUT_MAX_DEG = 0.25       # 인계 구간 보정 잘림 상한 (기준 판 0.23°)
J_AXIS_A_MAX_CM = 1.3      # 가상 캔 (A) 축까지 R 집게 가운데 거리 (시뮬레이션 정렬 거리 한도 1.5 cm 안쪽)


def _relA(ctx, demo_npz, dseg):
    """가상 캔 (시뮬레이션) 이 L 손에 붙인 캔 자세 (A): L 이 처음 held_L 인 프레임의 L 자세 기준."""
    z = np.load(demo_npz, allow_pickle=True)
    st = np.asarray(z["can_state"])
    k1 = next((k for k in range(dseg[0], dseg[1]) if int(st[k]) == 1), None)
    if k1 is None:
        return None
    tL, RL = ctx.vc.hand_of("left", ctx.jm.to_motor("left", dict(zip(JT, np.asarray(z["q_cmd"], float)[k1, 0]))))
    return RL.T @ quat_R(z["can_quat"][k1]), RL.T @ (np.asarray(z["can_pos"][k1], float) - tL)


def _approach_dist(ctx, act, iRc, n_back):
    """R 닫기 프레임의 손 축 기준으로, 그 앞 n_back 프레임 동안 R 집게 끝이 손 축 뒤로 떨어져 있던 거리 (cm, 0 이상) — 프레임마다."""
    qg = ctx.jm.to_motor("right", dict(zip(JT, act[iRc, 5:10])))
    tg, Rg = ctx.vc.hand_of("right", qg)
    out = {}
    for k in range(max(0, iRc - n_back), iRc + 1):
        tk = ctx.vc.hand_of("right", ctx.jm.to_motor("right", dict(zip(JT, act[k, 5:10]))))[0]
        out[k] = max(0.0, -float((tk - tg) @ Rg[:, 0]))
    return out


def _plan_R(ctx, ref, refL, relA, hd, jRc, jLo, a0, a1, s_log, a_prof, scale):
    """한 L 손목 각에서 R 프레임마다: 닫기 전 = 캔 틀에 고정한 잡는 점에서 손 축 뒤로 a(k) (기준 다가가기 × scale), 닫힌 뒤 L 열기까지 = 잡는 점 유지,
    그 뒤 = L 열기 프레임 관절 차 유지. 셋째 축 ∥ 캔 축. 반환 (R URDF (a1−a0+1, 5), 정보)."""
    Lm = lambda k: ctx.jm.to_motor("left", dict(zip(JT, refL[k, 0:5])))     # noqa: E731
    Rm = lambda k: ctx.jm.to_motor("right", dict(zip(JT, ref[k, 5:10])))      # noqa: E731

    def can(k):
        tL, RL = ctx.vc.hand_of("left", Lm(k))
        return tL + RL @ relA[1], RL @ relA[0]
    pos_g, rot_g = can(jRc)
    zc = rot_g[:, 2]
    axp = pos_g + J_GRASP_T_CM * zc
    q = dict(Rm(jRc))
    s_aim, err, mis = s_log, 0.0, 0.0
    for _ in range(3):                                         # 잡는 자세: 옆 · 정렬은 A 캔, 깊이는 B 로 s_log
        for _ in range(3):
            tip, R = ctx.vc.hand_of("right", q)
            hx = R[:, 0] - float(R[:, 0] @ zc) * zc
            hx /= max(np.linalg.norm(hx), 1e-9)
            q, err, mis = _ik_align(ctx, "right", q, axp + s_aim * hx, zc)
        sb = hd.depth(Lm(jRc), q)["s"]
        if abs(sb - s_log) < 0.03:
            break
        s_aim += s_log - sb
    for _ in range(4):                                         # 가상 캔 (A) 축까지 집게 가운데 거리가 한도를 넘으면 손 축 뒤로 물린다 (깊이는 조금 준다)
        tip, R = ctx.vc.hand_of("right", q)
        cen = tip - VCAN.GRIP_CENTER_CM * R[:, 0] - pos_g
        ax_ = float(np.linalg.norm(cen - float(cen @ zc) * zc))
        if ax_ <= J_AXIS_A_MAX_CM - 0.05:
            break
        hx = R[:, 0] - float(R[:, 0] @ zc) * zc
        hx /= max(np.linalg.norm(hx), 1e-9)
        along = float(cen @ hx)
        if abs(along) < 1e-3:
            break
        s_aim -= np.sign(along) * min(abs(along), ax_ - (J_AXIS_A_MAX_CM - 0.1))
        q, err, mis = _ik_align(ctx, "right", q, axp + s_aim * hx, zc)
    qg = dict(q)
    tg, Rg = ctx.vc.hand_of("right", qg)
    g_tip_c, g_hx_c = rot_g.T @ (tg - pos_g), rot_g.T @ Rg[:, 0]          # 캔 틀의 잡는 점 · 손 축
    dq_g = {j: qg[j] - Rm(jRc)[j] for j in JT}
    out = np.zeros((a1 - a0 + 1, 5))
    q_prev = {j: Rm(a0)[j] + dq_g[j] for j in JT}
    lim = ctx.lim["right"]
    worst = 0.0
    q_lo = None
    for k in range(a0, a1 + 1):
        if k <= jLo:
            pc, rc = can(k)
            p = pc + rc @ g_tip_c - (scale * a_prof.get(k, 0.0) if k < jRc else 0.0) * (rc @ g_hx_c)
            qk, e, _ = _ik_align(ctx, "right", q_prev, p, rc[:, 2], iters=25)
            worst = max(worst, e)
            if k == jLo:
                q_lo = {j: qk[j] - Rm(k)[j] for j in JT}
        else:
            qk = {j: min(max(Rm(k)[j] + q_lo[j], lim[j][0]), lim[j][1]) for j in JT}
        q_prev = qk
        u = ctx.jm.to_urdf("right", qk)
        out[k - a0] = [u[j] for j in JT]
    cen = tg - VCAN.GRIP_CENTER_CM * Rg[:, 0] - pos_g                 # 가상 캔 (A) 축까지 R 집게 가운데 거리 — 시뮬레이션 둘째 손 정렬 거리 한도 1.5 cm
    axis_cm = float(np.linalg.norm(cen - float(cen @ zc) * zc))
    return out, {"s_B": round(hd.depth(Lm(jRc), qg)["s"], 3), "grasp_ik_err_cm": round(err, 3), "align_mis_deg": round(mis, 2), "track_ik_err_cm": round(worst, 3),
                 "axis_A_cm": round(axis_cm, 2)}


def _quick_chain(ctx, refL, rq, a0, jRc, jLo, step: int = 2) -> dict:
    """후보 (L 손목 각) 하나의 인계 구간 보정 사슬: 잘림 최대, R elbow_pitch 밖 관절 여유 최소, R elbow_pitch 여유 (논리 자세, 2 프레임마다)."""
    cut, marg, mrep = 0.0, 1e9, 1e9
    for k in range(jRc, jLo + 1, step):
        for s, row in (("left", refL[k, 0:5]), ("right", rq[k - a0])):
            q = ctx.jm.to_motor(s, dict(zip(JT, row)))
            cut = max(cut, chain(ctx, s, q)["cut"])
            lim = ctx.lim[s]
            for j in JT:
                m_ = math.degrees(min(q[j] - lim[j][0], lim[j][1] - q[j]))
                if s == "right" and j == "elbow_pitch":
                    mrep = min(mrep, m_)
                else:
                    marg = min(marg, m_)
    return {"cut_deg": round(cut, 3), "margin_deg": round(marg, 2), "R_elbow_margin_deg": round(mrep, 2)}


def adjust_R(ctx, ref: np.ndarray, demo_act: np.ndarray, start_xy, demo_npz=None):
    """19-j2: 기준 R 을 그 시연의 캔에 프레임마다 맞춘다 (손 축 방향으로 다가감, 다가가는 시작 거리 ≤ 원래 시연). 깊이 (B) 가 R 관절 한계로 모자라면
    L 손목 (elbow_roll) 을 0 부터 1° 씩 ±J_WRIST_MAX_DEG 까지 돌려 처음 채워지는 각을 쓴다. 반환 (바꾼 기준 (L 손목 · R), 정보)."""
    rseg, dseg = AD.segments(ref), AD.segments(demo_act)
    lcd = min(dseg[0] + FPS // 2, dseg[1])
    s_log = J_TARGET_ENC_CM - J_DELTA_CM
    jRc, jLo = rseg[1], rseg[2]
    a0, a1 = max(0, jRc - int(PRE_S * FPS) - FPS // 2), min(len(ref) - 1, jLo + int(POST_S * FPS) + FPS // 2)
    relA = _relA(ctx, demo_npz, dseg) if demo_npz is not None else None
    hd = _hdm(ctx, start_xy, s_log)
    hd.on_L_closed(ctx.jm.to_motor("left", dict(zip(JT, demo_act[lcd, 0:5]))))
    if relA is None:
        relA = (hd.rel[0], hd.rel[1])
    a_ref = _approach_dist(ctx, ref, jRc, jRc - a0)
    a_dem = _approach_dist(ctx, demo_act, dseg[1], jRc - a0)
    mr, md = max(a_ref.values()), max(a_dem.values())
    scale = min(1.0, md / mr) if mr > 1e-6 else 1.0
    s0 = hd.depth(ctx.jm.to_motor("left", dict(zip(JT, ref[jRc, 0:5]))), ctx.jm.to_motor("right", dict(zip(JT, ref[jRc, 5:10]))))["s"]
    best = None
    phis = [0.0] + [sg * k * J_WRIST_STEP_DEG for k in range(1, int(J_WRIST_MAX_DEG / J_WRIST_STEP_DEG) + 1) for sg in (-1.0, 1.0)]
    for phi in phis:
        refL = ref.copy()
        refL[a0:a1 + 1, 4] += math.radians(phi)
        rq, inf = _plan_R(ctx, ref, refL, relA, hd, jRc, jLo, a0, a1, s_log, a_ref, scale)
        ch = _quick_chain(ctx, refL, rq, a0, jRc, jLo)
        inf.update(ch)
        ik_ok = inf["grasp_ik_err_cm"] <= 0.1 and inf["axis_A_cm"] <= J_AXIS_A_MAX_CM
        ok_c = ch["cut_deg"] <= J_CUT_MAX_DEG and ch["margin_deg"] >= J_MARGIN_DEG and ch["R_elbow_margin_deg"] >= J_R_ELBOW_MIN_DEG
        full = ik_ok and ok_c and inf["s_B"] >= s_log - 0.05
        mini = ik_ok and ok_c and inf["s_B"] >= J_MIN_ENC_CM - J_DELTA_CM
        # 차례: 목표 깊이까지 다 되는 가장 작은 손목 각 → 검사 하한 (엔코더 상당 3.0) 이 되는 가장 작은 각 → IK 가 되는 것 중 축 거리 가장 작은 것
        score = (full, mini, inf["grasp_ik_err_cm"] <= 0.1, -inf["axis_A_cm"] if not mini else -abs(phi))
        cand = (phi, refL, rq, inf, score)
        if best is None or score > best[4]:
            best = cand
        if full:
            break
    phi, refL, rq, inf, _ = best
    out = refL.copy()
    out[a0:a1 + 1, 5:10] = rq
    return out, {"s_log_before": round(s0, 2), "s_log_after": round(inf["s_B"], 2), "L_wrist_deg": phi, "approach_scale": round(scale, 2),
                 "approach_ref_cm": round(mr, 2), "approach_demo_cm": round(md, 2), **inf, "frames": [a0, a1]}


PAD_END_S = 2.0           # 놓기 옮김을 되돌리는 끝 구간을 멈춘 자세 위에서 하도록 끝에 붙이는 시간


def place_shift(ctx, act: np.ndarray, dp: np.ndarray):
    """R 놓기 궤적을 dp (cm, base) 만큼 옮긴다 — aug_demo.augment 의 도착 옮김 그대로 (R: L 열기 + 0.25 초 → 놓기 1 초 전 0 → 1, 놓고 2 초 유지,
    그 뒤 끝까지 1 → 0, 앞 프레임 해에서 이어 푸는 IK + 이동 평균). 끝의 빠른 물러나기 위에서 되돌리면 관절 속도가 튀므로, 끝에 마지막 자세를
    PAD_END_S 초 붙여 그 멈춘 자세 위에서 되돌린다. L 은 ds = 0 이라 원래 값 그대로 되돌려 놓는다."""
    n_pad = int(PAD_END_S * FPS)
    padded = np.concatenate([act, np.repeat(act[-1:], n_pad, 0)], 0)
    out, ainf = AD.augment(ctx, {"action": padded}, np.zeros(3), np.asarray(dp, float))
    out[:, 0:5] = padded[:, 0:5]
    out = out[:-2]                                             # 마지막 프레임은 augment 가 w = 0 이라 원래 자세로 바로 돌아가 튄다 — 뺀다
    return out, {"dp_cm": [round(float(v), 2) for v in dp], "key_ik_err_cm": ainf.get("key_ik_err_cm"), "key_pitch_err_deg": ainf.get("key_pitch_err_deg"),
                 "key_lim_frames": ainf.get("key_lim_frames"), "pad_frames": n_pad}


def check_j(ctx, npz: Path, start_xy) -> dict:
    """19-j 검사: 엔코더 상당 깊이 (논리 B + Δ), 인계 구간 [R 닫기, L 열기] 보정 사슬 잘림 · 두 팔 관절 한계 여유 (논리 자세) · 판 면, R 닫기 순간 그리퍼 간격."""
    z = np.load(npz, allow_pickle=True)
    g = np.asarray(z["grip"], float)
    act = np.concatenate([np.asarray(z["q_cmd"], float).reshape(len(g), 10), g], 1)
    seg = AD.segments(act)
    if seg is None:
        return {"why": "그리퍼 차례"}
    iLc, iRc, iLo, iRo = seg
    d = depth_B(ctx, act, start_xy, seg)
    cut, marg, marg_rep, gmin = 0.0, 1e9, 1e9, 1e9
    for k in range(iRc, iLo + 1):
        for i, s in enumerate(PS.SIDES):
            q = ctx.jm.to_motor(s, dict(zip(JT, act[k, 5 * i:5 * i + 5])))
            c = chain(ctx, s, q)
            cut, gmin = max(cut, c["cut"]), min(gmin, c["g"])
            lim = ctx.lim[s]
            for j in JT:
                m_ = math.degrees(min(q[j] - lim[j][0], lim[j][1] - q[j]))
                if s == "right" and j == "elbow_pitch":
                    marg_rep = min(marg_rep, m_)
                else:
                    marg = min(marg, m_)
    return {"seg": [iLc, iRc, iLo, iRo], "s_log": round(d["s"], 2), "s_enc_eq": round(d["s"] + J_DELTA_CM, 2), "j": round(d["j"], 2), "gap_cm": round(d["gap"], 2),
            "cut_deg": round(cut, 3), "margin_deg": round(marg, 2), "R_elbow_margin_deg": round(marg_rep, 2), "table_g_cm": round(gmin, 2),
            "can_tilt_deg": None}


def aug_check_j(ctx, act: np.ndarray, start_xy) -> dict:
    """aug_demo --handover-check-j (19-j2) 가 부른다: 증강한 행동 (시뮬레이션 전) 의 인계 구간을 make2 와 같은 기준으로.
    엔코더 상당 깊이 (논리 B + Δ) ≥ J_MIN_ENC_CM, 보정 잘림 ≤ J_CUT_MAX_DEG, 관절 여유 (R elbow_pitch ≥ J_R_ELBOW_MIN_DEG, 나머지 ≥ J_MARGIN_DEG), 그리퍼 간격 ≥ J_GAP_MIN_CM."""
    seg = AD.segments(act)
    if seg is None:
        return {"ok": False, "why": "그리퍼 차례"}
    iLc, iRc, iLo, iRo = seg
    d = depth_B(ctx, act, start_xy, seg)
    ch = _quick_chain(ctx, act, act[:, 5:10], 0, iRc, iLo, step=1)
    s_enc = d["s"] + J_DELTA_CM
    bad = []
    if s_enc < J_MIN_ENC_CM:
        bad.append(f"깊이 {s_enc:.2f}")
    if ch["cut_deg"] > J_CUT_MAX_DEG:
        bad.append(f"잘림 {ch['cut_deg']}°")
    if ch["margin_deg"] < J_MARGIN_DEG or ch["R_elbow_margin_deg"] < J_R_ELBOW_MIN_DEG:
        bad.append(f"관절 여유 {ch['margin_deg']}° / R 팔꿈치 {ch['R_elbow_margin_deg']}°")
    if d["gap"] < J_GAP_MIN_CM:
        bad.append(f"간격 {d['gap']:.1f}")
    return {"ok": not bad, "why": ", ".join(bad), "s_enc_eq": round(s_enc, 2), "gap_cm": round(d["gap"], 2), **ch}


def make2_one(ctx, path: str, ref: np.ndarray, out: Path, tmp: Path, tag: str) -> dict:
    arr = L.arrays(path)
    m = arr["meta"]
    act = np.asarray(arr["action"], float)
    name = f"{m['record']}_ep{int(m['episode']):02d}"
    sxy, gxy = AD.demo_xy(m)
    r = {"name": name, "src": path}
    src = np.load(path, allow_pickle=True)
    sg = AD.segments(np.concatenate([np.asarray(src["q_cmd"], float).reshape(len(src["t"]), 10), np.asarray(src["grip"], float)], 1))
    land0 = np.asarray(src["can_pos"][max(0, sg[3] - 1)], float) if sg else None     # 원래 시연에서 R 이 열기 직전 캔 자리
    refa, ra = adjust_R(ctx, ref, act, sxy, path)
    r["adjust_R"] = ra
    new = info = None
    for st in STRETCH:
        new, info = build(ctx, act, refa, st)
        if new is None:
            return {**r, "ok": False, "why": info["why"]}
        if info["max_speed_deg_s"] <= info["orig_speed_deg_s"] + 1e-6:
            break
    if info["max_speed_deg_s"] > info["orig_speed_deg_s"] + 1e-6:
        return {**r, "info": info, "ok": False, "why": f"관절 속도 (늘여도 {info['max_speed_deg_s']} > {info['orig_speed_deg_s']} °/s)"}

    def sim(a_, nm):
        with contextlib.redirect_stdout(io.StringIO()):
            s_ = PS.rollout(AD.Fixed(a_), nm, max_s=len(a_) / FPS + 6.0, out_dir=tmp, start=sxy, goal=gxy, strict=True, first=a_[0, :10].reshape(2, 5))
        return s_
    nm0 = f"{tag}_{name}_p0"
    sim(new, nm0)
    z0 = np.load(tmp / f"{nm0}.npz", allow_pickle=True)
    g0 = np.concatenate([np.asarray(z0["q_cmd"], float).reshape(len(z0["t"]), 10), np.asarray(z0["grip"], float)], 1)
    s0 = AD.segments(g0)
    dp = np.zeros(3)
    if s0 is not None and land0 is not None:
        land1 = np.asarray(z0["can_pos"][max(0, s0[3] - 1)], float)
        dp = land0 - land1
        dp[2] = 0.0                                             # 높이는 판이 정한다 (xy 만)
    if float(np.linalg.norm(dp)) > 0.05:
        new2, pinfo = place_shift(ctx, new, dp)
    else:
        new2, pinfo = new, {"dp_cm": [0.0, 0.0, 0.0]}
    if float(np.linalg.norm(dp)) > 0.05 and land0 is not None:      # 한 번 더: 옮긴 뒤 남은 차가 0.5 cm 넘으면 더해서 다시
        nm1 = f"{tag}_{name}_p1"
        sim(new2, nm1)
        z1 = np.load(tmp / f"{nm1}.npz", allow_pickle=True)
        s1 = AD.segments(np.concatenate([np.asarray(z1["q_cmd"], float).reshape(len(z1["t"]), 10), np.asarray(z1["grip"], float)], 1))
        if s1 is not None:
            res_ = land0 - np.asarray(z1["can_pos"][max(0, s1[3] - 1)], float)
            res_[2] = 0.0
            if float(np.linalg.norm(res_)) > 0.5:
                dp = dp + res_
                new2, pinfo = place_shift(ctx, new, dp)
                pinfo["second_pass_residual_cm"] = [round(float(v), 2) for v in res_]
    r["place_shift"] = pinfo
    sp2 = speed(new2)
    info["max_speed_deg_s_after_place"] = round(sp2, 1)
    nm = f"{tag}_{name}"
    s = sim(new2, nm)
    stv = s.get("strict") or {}
    r["sim"] = {"result": s.get("result"), "why": s.get("why"), "goal_dist_cm": s.get("goal_dist_cm"), "clip_max_deg": s.get("safety_clip_max_deg"),
                "strict_flags": stv.get("strict_flags"), "strict_fail": stv.get("strict_fail")}
    # 바뀐 프레임 (원래 시연 시간에 맞춘 비교, build 와 같은 대응)
    lift, e = info["region"]
    M = info["region_new"][1] - lift
    us = lift + np.arange(M + 1) * (e - lift) / M
    fr = np.arange(len(act))
    orig_at = np.concatenate([act[:lift, :10], np.array([[np.interp(u, fr, act[:, c]) for c in range(10)] for u in us]), act[e + 1:, :10]], 0)
    if len(new2) > len(orig_at):                               # 놓기 옮김이 끝에 멈춘 자세를 붙인 만큼
        orig_at = np.concatenate([orig_at, np.repeat(orig_at[-1:], len(new2) - len(orig_at), 0)], 0)
    changed = int((np.degrees(np.abs(new2[:, :10] - orig_at)).max(1) > 0.1).sum())
    info.update(changed_frames=changed, changed_pct=round(100.0 * changed / len(new2), 1))
    r["info"] = info
    sim_ok = s.get("result") == "auto_success" and str(stv.get("strict_fail", "0")) in ("0", "", "None") and float(s.get("safety_clip_max_deg") or 0) <= AD.CLIP_MAX_DEG
    c = check_j(ctx, tmp / f"{nm}.npz", sxy)
    r["check"] = c
    fails = []
    if not sim_ok:
        fails.append(f"가상 캔 ({s.get('why')}, 엄격 {stv.get('strict_flags')}, 깎임 {s.get('safety_clip_max_deg')}°)")
    if "s_enc_eq" not in c:
        fails.append(f"검사 못 함 ({c.get('why')})")
    else:
        if c["s_enc_eq"] < J_MIN_ENC_CM:
            fails.append(f"엔코더 상당 깊이 {c['s_enc_eq']} cm")
        if c["cut_deg"] > J_CUT_MAX_DEG:
            fails.append(f"보정 잘림 {c['cut_deg']}°")
        if c["margin_deg"] < J_MARGIN_DEG:
            fails.append(f"관절 한계 여유 {c['margin_deg']}° (R elbow_pitch 밖)")
        if c["R_elbow_margin_deg"] < J_R_ELBOW_MIN_DEG:
            fails.append(f"R elbow_pitch 여유 {c['R_elbow_margin_deg']}°")
        if c["gap_cm"] < J_GAP_MIN_CM:
            fails.append(f"그리퍼 간격 {c['gap_cm']} cm")
        if c["table_g_cm"] < 0:
            fails.append(f"판 면 {c['table_g_cm']} cm")
    spL = float(np.degrees(np.abs(np.diff(new2[:, 0:5], axis=0)).max()) * FPS)
    info["max_speed_L_deg_s"] = round(spL, 1)
    if spL > info["orig_speed_deg_s"] + 0.1:
        fails.append(f"L 관절 속도 {spL:.1f} > 원래 최대 {info['orig_speed_deg_s']} °/s")
    e_new = info["region_new"][1]                              # 인계 (다가가기 · 이음) 끝 프레임
    spR_h = float(np.degrees(np.abs(np.diff(new2[:e_new + 1, 5:10], axis=0)).max()) * FPS)
    spR = float(np.degrees(np.abs(np.diff(new2[:, 5:10], axis=0)).max()) * FPS)
    info["max_speed_R_handover_deg_s"], info["max_speed_R_deg_s"] = round(spR_h, 1), round(spR, 1)
    if spR_h > info["orig_speed_deg_s"] + 0.1:                 # 다가가기 · 인계: 원래 시연 최대 이하
        fails.append(f"R 관절 속도 (인계) {spR_h:.1f} > 원래 최대 {info['orig_speed_deg_s']} °/s")
    if spR > AD.SPEED_GAIN * info["orig_speed_deg_s"] + AD.SPEED_ADD:     # 놓기 옮김: 증강의 도착 옮김과 같은 한도
        fails.append(f"R 관절 속도 (놓기 옮김) {spR:.1f} > 증강 한도 {AD.SPEED_GAIN * info['orig_speed_deg_s'] + AD.SPEED_ADD:.1f} °/s")
    r["ok"], r["why"] = not fails, "; ".join(fails)
    z = np.load(tmp / f"{nm}.npz", allow_pickle=True)
    meta = json.loads(str(z["meta"]))
    meta.update({"record": f"{tag}_{m['record']}", "episode": int(m["episode"]), "success": sim_ok, "result": s.get("result"), "options": m.get("options"),
                 "quality": {"policy_rollout": False, "no_release": False, "handover_swap": True,
                             "strict": {"fail": not sim_ok, "flags": (stv.get("strict_flags") or "").split()}},
                 "handover_swap": {"mode": "make2 (19-j)", "src_record": m.get("record"), "src_episode": m.get("episode"), "src_file": path,
                                   "ref": {"log": str(J_REF_LOG), "robot_ref": J_REF_REF, "seg": list(AD.segments(ref))}, "delta_cm": J_DELTA_CM,
                                   "target_enc_cm": J_TARGET_ENC_CM, "adjust_R": ra, "place_shift": pinfo, "info": info, "check": c, "sim": r["sim"]}})
    f = out / f"{nm}.npz"
    np.savez_compressed(f, meta=np.array(json.dumps(meta, ensure_ascii=False, default=float)), **{k: z[k] for k in z.files if k != "meta"})
    r["file"] = str(f)
    return r


def cmd_make2(a) -> int:
    ctx = AD.Ctx()
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    tmp = out / "_sim"
    tmp.mkdir(exist_ok=True)
    ref = ref_from_log(Path(a.ref_log).expanduser(), a.ref_robot_ref)
    paths = [p.strip() for p in Path(a.files).expanduser().read_text(encoding="utf-8").splitlines() if p.strip() and not p.startswith("#")]
    res, t0 = [], time.time()
    for n, p in enumerate(paths):
        r = make2_one(ctx, p, ref, out, tmp, a.tag)
        res.append(r)
        c, i = r.get("check") or {}, r.get("info") or {}
        print(f"[{n + 1}/{len(paths)}] {r['name']}: {'통과' if r['ok'] else '실패 — ' + r['why']} | R 옮김 {(r.get('adjust_R') or {}).get('d_cm')} cm, "
              f"놓기 옮김 {(r.get('place_shift') or {}).get('dp_cm')}, 늘임 {i.get('stretch')}, 바뀐 {i.get('changed_pct')} % · 깊이 (엔코더 상당) {c.get('s_enc_eq')} · "
              f"잘림 {c.get('cut_deg')}° · 여유 {c.get('margin_deg')}° · 간격 {c.get('gap_cm')} · 판 {c.get('table_g_cm')} · 목표 {(r.get('sim') or {}).get('goal_dist_cm')} — "
              f"{round(time.time() - t0)} 초", flush=True)
        (out / "make2.json").write_text(json.dumps({"ref_log": str(a.ref_log), "delta_cm": J_DELTA_CM, "rows": res}, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    for q in tmp.glob("*"):
        q.unlink()
    tmp.rmdir()
    ok = [r for r in res if r["ok"]]
    (out / "files_ho.txt").write_text("\n".join(r["file"] for r in ok) + "\n", encoding="utf-8")
    print(f"통과 {len(ok)} / {len(res)} → {out / 'files_ho.txt'}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p3 = sub.add_parser("make2", help="19-j: 실기 성공 판 하나의 인계 자세로 통일 + R 깊이 맞춤 + 놓기 옮김")
    p3.add_argument("--files", required=True)
    p3.add_argument("--ref-log", default=str(J_REF_LOG))
    p3.add_argument("--ref-robot-ref", default=J_REF_REF)
    p3.add_argument("--out", required=True)
    p3.add_argument("--tag", default="ho19j")
    p1 = sub.add_parser("pick")
    p1.add_argument("--out", required=True)
    p2 = sub.add_parser("make")
    p2.add_argument("--files", required=True)
    g2 = p2.add_mutually_exclusive_group(required=True)
    g2.add_argument("--ref", help="기준 판 하나 (npz)")
    g2.add_argument("--refs", help="pick.json — 시연마다 기준을 고른다 (feasible)")
    p2.add_argument("--out", required=True)
    p2.add_argument("--tag", default="ho19g")
    a = ap.parse_args()
    if a.cmd == "make2":
        return cmd_make2(a)
    return cmd_pick(a) if a.cmd == "pick" else cmd_make(a)


if __name__ == "__main__":
    sys.exit(main())
