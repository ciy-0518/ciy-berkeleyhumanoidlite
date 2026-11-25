#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""실기 추가 실험 r2: 판 로그 하나 → 받는 손 (R) 엔코더 깊이 s · 인계 깊이 규칙이 채운 양 (읽기만, CPU).

정의 (사용자 승인, 2026-11-24):
  s = L 을 열기 전 마지막 R 닫힘 끝의 R 깊이 (엔코더 FK, 가정 A = 캔을 L 닫기 명령 순간의 L 손에 붙임, 19-h).
      L 을 안 열었으면 L 잡기 뒤 마지막 R 닫힘. R 닫힘이 없으면 비움.
  아래쪽 잡기 (E1 ~ E3): 19-h 분석 (ana19h.py) 과 같은 계산 (VirtualCan grip_check 둘째 손, 닫힘 끝 = R 닫기 + 0.5 초,
      robot_ref · table.json 은 로그 시각 것; table.json 이력이 없으면 지금 config/table.json). 표 캐시는 --cache.
  위쪽 잡기 (E4): 19-x27 분석 (ana27.py) 의 "엔코더 A" 와 같은 계산 (handover_assist.check_r, 닫힘 끝 = grip_wait 끝,
      L 잡기 = 캔이 처음 held 가 된 때 직전의 L 닫기). 로그 · 자세 함수는 ana17.py. 참고로 R 닫힘들 중 최댓값 (s_max) 도 낸다.
  rule_fill_cm = 로그 열 hd_fill_cm 최댓값 (인계 깊이 규칙, 아래쪽 잡기만; 열이 없으면 비움). hd_cut (줄인 까닭) 도 낸다.
출력: JSON 한 줄 (stdout).
  python3 depth_r2.py <log.csv> --mode bottom|top --start x,y --goal x,y [--cache DIR]"""
from __future__ import annotations

import argparse
import contextlib
import csv
import datetime as dt
import io
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

H = Path.home()
TOOLS = Path(__file__).resolve().parent                      # 공개 저장소: teleop/tools
LOWLEVEL = TOOLS.parents[1]                                   # 저장소 맨 위 (vr_control_watchdog_biarm.py)
for p in (str(TOOLS), str(LOWLEVEL)):
    if p not in sys.path:
        sys.path.insert(0, p)
with contextlib.redirect_stdout(io.StringIO()):
    import vr_control_watchdog_biarm as W  # noqa: E402  (워치독 먼저 — teleop-check-pitfalls)
import virtual_can as VC  # noqa: E402
import table_limit as TL  # noqa: E402
import demo_export as DE  # noqa: E402

SIDES = ("left", "right")
J = list(W.JOINT_ORDER)
CLOSE_S = 0.5


def fnum(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else float("nan")
    except (TypeError, ValueError):
        return float("nan")


def rule_fill(R):
    fills = [fnum(r.get("hd_fill_cm")) for r in R if r.get("hd_fill_cm") not in (None, "")]
    fills = [f for f in fills if f == f]
    cuts = sorted({r.get("hd_cut", "") for r in R if r.get("hd_cut")})
    return (round(max(fills), 2) if fills else None), " / ".join(cuts)


# --------------------------------------------------------------------------- 아래쪽 잡기 (19-h 식)
def log_time(path: Path) -> str:
    return dt.datetime.fromtimestamp(path.stat().st_mtime - 60).isoformat()


def table_at(when: str, cache: Path):
    """로그 시각의 table.json (teleop-docs git 이력). 이력이 없으면 (공개 저장소) 지금 config/table.json."""
    out = subprocess.run(["git", "-C", str(H / "teleop-docs"), "log", "--format=%h %cI", "--", "_teleop/config/table.json"],
                         capture_output=True, text=True).stdout.split("\n")
    t = dt.datetime.fromisoformat(when)
    for ln in out:
        if not ln.strip():
            continue
        h, ci = ln.split()
        if dt.datetime.fromisoformat(ci).replace(tzinfo=None) <= t:
            p = cache / f"table_{h}.json"
            if not p.exists():
                txt = subprocess.run(["git", "-C", str(H / "teleop-docs"), "show", f"{h}:_teleop/config/table.json"],
                                     capture_output=True, text=True).stdout
                cache.mkdir(parents=True, exist_ok=True)
                p.write_text(txt, encoding="utf-8")
            return h, TL.TableLimit(p, body="real")
    return "current", TL.TableLimit(TOOLS.parent / "config/table.json", body="real")


def bottom(path: Path, start_xy, cache: Path) -> dict:
    W.PR.set_hand_model("cad")
    W.apply_yaw_in_limit(100.0, 100.0)
    R = list(csv.DictReader(path.open(encoding="utf-8", errors="ignore")))
    when = log_time(path)
    ref_path = DE.ref_path_for(when)
    th, table = table_at(when, cache)
    ref = W.PR.RobotRef.load(ref_path)
    vc = VC.VirtualCan(table, "L1", ref, SIDES, W.motor_to_urdf_fn(ref, SIDES), log=lambda s: None)
    t = np.array([fnum(r["t"]) for r in R])
    meas = []
    for r in R:
        q = {s: {j: fnum(r.get(f"{s}_{j}_meas")) for j in J} for s in SIDES}
        meas.append(q if all(v == v for s in SIDES for v in q[s].values()) else None)
    out = {"ref": ref_path.name, "table": th}
    if sum(m is not None for m in meas) < 50 or len({r.get("right_elbow_pitch_meas") for r in R}) < 5:
        return dict(out, skip="엔코더 없음 (dry-run)")
    gL = [r.get("grip_L") == "1" for r in R]
    gR = [r.get("grip_R") == "1" for r in R]
    lc = next((i for i in range(1, len(R)) if gL[i] and not gL[i - 1]), None)
    if lc is None:
        return dict(out, skip="L 닫기 없음")
    lo = next((i for i in range(lc + 1, len(R)) if gL[i - 1] and not gL[i]), None)
    end = lo if lo is not None else len(R)
    rcs = [i for i in range(lc + 1, end) if gR[i] and not gR[i - 1]]
    out.update(events=f"Lc {t[lc] - t[0]:.1f}" + "".join(f" Rc {t[i] - t[0]:.1f}" for i in rcs) + (f" Lo {t[lo] - t[0]:.1f}" if lo else " (L 안 엶)"))
    if not rcs:
        return dict(out, skip="R 닫기 없음")

    def at(i0, dts):
        k = i0
        while k < len(t) - 1 and (t[k] - t[i0] < dts - 1e-6 or meas[k] is None):
            k += 1
        return k

    hand = lambda k, s: vc.hand_of(s, meas[k][s])     # noqa: E731
    tL0, RL0 = hand(lc, "left")
    xy = start_xy
    can0 = np.array([xy[0], xy[1], table.top(xy[0] / 100.0) * 100.0])
    rot0 = np.eye(3)
    relA = (RL0.T @ rot0, RL0.T @ (can0 - tL0))
    ss = []
    for rc in rcs:
        rcd = at(rc, CLOSE_S)
        tLh, RLh = hand(rcd, "left")
        tRh, RRh = hand(rcd, "right")
        vc.rot, vc.pos, vc.holders, vc.align = RLh @ relA[0], tLh + RLh @ relA[1], ["left"], None
        c = vc.grip_check((tRh, RRh), VC.HANDOVER_HEIGHT_EXTRA_CM, True)
        ss.append(round(float(c["s"]), 2))
    out.update(s=ss[-1], s_first=ss[0], s_all=ss, s_max=max(ss))
    return out


# --------------------------------------------------------------------------- 위쪽 잡기 (19-x27 엔코더 A 식)
def top(path: Path, start_xy, goal_xy, use_log_ref: bool = True) -> dict:
    with contextlib.redirect_stdout(io.StringIO()):
        import ana17 as A
        import handover_assist as HA
    R = list(csv.DictReader(path.open(encoding="utf-8", errors="ignore")))
    # robot_ref: 로그 시각의 것 (아래쪽 잡기와 같게). ana17.Geo → policy_sim.SimEnv 는 "가장 새 것" 을 쓰므로 그때만 바꿔 끼운다
    ref_path = DE.ref_path_for(log_time(path)) if use_log_ref else None
    orig = W.newest_robot_ref
    if ref_path is not None:
        W.newest_robot_ref = lambda *_a, **_k: ref_path
    try:
        g = A.Geo(tuple(start_xy), tuple(goal_xy))
    finally:
        W.newest_robot_ref = orig
    vc = g.vc
    P = A.poses(R)
    ev = A.events(R)
    t0 = float(R[0]["t"])
    out = {"ref": g.env.ref_path.name, "events": " ".join(f"{e['h']}{e['k']} {e['t'] - t0:.1f}" for e in ev)}
    if sum(q is not None for q in P["E"]) < 50:
        return dict(out, skip="엔코더 없음 (dry-run)")
    # L 잡기 닫기: 캔이 처음 held 가 된 때 직전의 L 닫기 (19-x29 tilt.py 와 같음 — 공중 닫힘을 건너뜀). held 가 없으면 19-x27 식 (첫 L 열기 뒤 첫 L 닫기)
    ih = next((i for i, r in enumerate(R) if str(r.get("pol_can_state", "")).startswith("held")), None)
    lcs = [e for e in ev if e["h"] == "L" and e["k"] == "c" and ih is not None and e["i"] <= ih]
    if lcs:
        lc = lcs[-1]
        out["lc_rule"] = "held 직전 L 닫기"
    else:
        lo1 = next((e for e in ev if e["h"] == "L" and e["k"] == "o"), None)
        lc = next((e for e in ev if lo1 and e["h"] == "L" and e["k"] == "c" and e["i"] > lo1["i"]), None)
        out["lc_rule"] = "첫 L 열기 뒤 L 닫기"
    if lc is None:
        return dict(out, skip="L 잡기 닫기 없음")
    qL = P["E"][lc["i"]]
    if qL is None:
        return dict(out, skip="L 닫기 순간 엔코더 없음")
    hL0 = vc.hand_of("left", qL["left"])
    relEA = (hL0[1].T, hL0[1].T @ (g.can0 - hL0[0]))
    lo = next((e for e in ev if e["h"] == "L" and e["k"] == "o" and e["i"] > lc["i"]), None)
    rcs = [e for e in ev if e["h"] == "R" and e["k"] == "c" and e["i"] > lc["i"] and (lo is None or e["i"] < lo["i"])]
    if not rcs:
        return dict(out, skip="R 닫기 없음")
    ss = []
    for e in rcs:
        k = A.settle_end(R, e["i"])
        if P["E"][k] is None:
            continue
        hL, hR = vc.hand_of("left", P["E"][k]["left"]), vc.hand_of("right", P["E"][k]["right"])
        cp, cr = hL[0] + hL[1] @ relEA[1], hL[1] @ relEA[0]
        c = HA.check_r(vc, hR, cp, cr, VC.HANDOVER_HEIGHT_EXTRA_CM)
        ss.append(round(float(c["s"]), 2))
    if not ss:
        return dict(out, skip="R 닫힘 끝 엔코더 없음")
    out.update(s=ss[-1], s_first=ss[0], s_all=ss, s_max=max(ss), L_open=lo is not None)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--mode", choices=("bottom", "top"), required=True)
    ap.add_argument("--start", required=True, help="캔 시작 x,y (로봇 cm, 명령의 --can-start)")
    ap.add_argument("--goal", required=True, help="목적지 x,y (로봇 cm)")
    ap.add_argument("--cache", default=str(TOOLS.parent / "logs/depth_r2_cache"))
    ap.add_argument("--top-newest-ref", action="store_true", help="위쪽 잡기에서 19-x27 처럼 가장 새 robot_ref 를 씀 (비교용)")
    a = ap.parse_args()
    path = Path(a.log).expanduser()
    st = [float(v) for v in a.start.split(",")]
    go = [float(v) for v in a.goal.split(",")]
    out = {"log": path.name, "mode": a.mode}
    try:
        R = list(csv.DictReader(path.open(encoding="utf-8", errors="ignore")))
        fill, cut = rule_fill(R)
        out.update(rule_fill_cm=fill, hd_cut=cut)
        with contextlib.redirect_stdout(io.StringIO()):
            res = bottom(path, st, Path(a.cache)) if a.mode == "bottom" else top(path, st, go, use_log_ref=not a.top_newest_ref)
        out.update(res)
    except Exception as e:      # 화면은 빈 칸으로 두고 나중에 일괄 계산
        out["error"] = f"{type(e).__name__}: {e}"
    print(json.dumps(out, ensure_ascii=False, default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())
