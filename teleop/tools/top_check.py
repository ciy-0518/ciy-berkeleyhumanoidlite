#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""위쪽 잡기 시연 점검 (작업 지시 19-x). dry-run 기록의 에피소드 파일 (<로그>_episodes.csv) 을 읽어 에피소드마다 한 줄:
  결과 · 잡은 차례 · 인계 · 인계 깊이 (논리, 엔코더 상당 = 논리 + 1.30) · L / R 비킴 · 누름 · 잡기 전 부딪힘 · 시작 자세 · 엄격 판정 → 통과 여부.
통과 = 결과 auto_success · 잡은 차례 L>R · 인계 1 · real_fail 0 · strict_fail 0 (엄격 판정을 켠 기록이면) · 시작 자세 기록이 있음 (--start-gate 를 켠 기록이면).
  top_check.py <로그 이름 또는 경로> [...] [--list-out 통과목록.txt]
  예: top_check.py dry_vcan_2130 dry_vcan_2215 --list-out ~/cc_analysis/top19x/pass.txt
통과 목록은 demo_export 가 만드는 에피소드 이름 (<로그 이름>_epNN) 한 줄씩 — 증강 · 학습 단계가 이 목록만 쓴다."""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

LOGS = Path(__file__).resolve().parent.parent / "logs"
DELTA_CM = 1.30          # 엔코더 상당 = 논리 + Δ (19-j delta.py, virtual_can REAL_DEPTH_LOGIC_CM 근거)


def find(name: str) -> Path:
    p = Path(name).expanduser()
    if p.suffix == ".csv" and p.exists():
        return p if p.name.endswith("_episodes.csv") else p.with_name(p.stem + "_episodes.csv")
    q = LOGS / f"{name}_episodes.csv"
    if q.exists():
        return q
    raise FileNotFoundError(f"에피소드 파일 없음: {name} ({q})")


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def check(path: Path):
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    record = path.name[:-len("_episodes.csv")]
    out = []
    for r in rows:
        why = []
        if r.get("result") != "auto_success":
            why.append(f"결과 {r.get('result')}")
        if r.get("grip_order") != "L>R":
            why.append(f"차례 {r.get('grip_order') or '-'}")
        if r.get("handover") != "1":
            why.append("인계 없음")
        has_real = "real_fail" in r
        if has_real and r.get("real_fail") == "1":
            why.append("실물 기준 " + (r.get("real_flags") or "?"))
        if "strict_fail" in r and r.get("strict_fail") == "1":
            why.append("엄격 " + (r.get("strict_flags") or "?"))
        if not has_real:
            why.append("실물 기준 판정 없이 모은 기록 (--grasp-real)")
        gate = fnum(r.get("start_gate_wait_s")) if "start_gate_wait_s" in r else None
        if "start_gate_wait_s" not in r:
            why.append("시작 자세 없이 모은 기록 (--start-gate)")
        d = fnum(r.get("real_depth_cm"))
        out.append({"name": f"{record}_ep{int(r['episode']):02d}", "episode": r["episode"], "pair": r.get("combo_index", ""),
                    "start": (r.get("start_x_cm"), r.get("start_y_cm")), "goal": (r.get("goal_x_cm"), r.get("goal_y_cm")), "result": r.get("result"),
                    "order": r.get("grip_order"), "depth": d, "depth_enc": None if d is None else d + DELTA_CM,
                    "L_side": fnum(r.get("real_L_side_cm")), "R_side": fnum(r.get("real_R_side_cm")), "press": fnum(r.get("real_press_cm")),
                    "bump": r.get("real_bump"), "gate_s": gate, "goal_cm": fnum(r.get("goal_dist_cm")), "ok": not why, "why": why})
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter, epilog="\n".join(__doc__.split("\n")[1:]))
    ap.add_argument("logs", nargs="+", help="dry-run 로그 이름 (예 dry_vcan_2130) 또는 csv 경로")
    ap.add_argument("--list-out", help="통과한 에피소드 이름 목록을 쓸 파일")
    a = ap.parse_args()
    allr = []
    f = lambda v, n=2: "-" if v is None else f"{v:.{n}f}"
    for name in a.logs:
        try:
            p = find(name)
        except FileNotFoundError as e:
            print(f"🔴 {e}")
            return 2
        rs = check(p)
        allr += rs
        print(f"== {p.name} ({len(rs)} 에피소드)")
        print("   에피소드 | 쌍 | 결과 | 차례 | 인계 깊이 논리 (엔코더 상당) | 비킴 L · R | 누름 | 부딪힘 | 시작 자세 대기 s | 목표 cm | 판정")
        for r in rs:
            print(f"   {r['name']} | {r['pair'] or '-'} | {r['result']} | {r['order'] or '-'} | {f(r['depth'])} ({f(r['depth_enc'], 1)}) | "
                  f"{f(r['L_side'])} · {f(r['R_side'])} | {f(r['press'])} | {r['bump'] or '-'} | {f(r['gate_s'], 1)} | {f(r['goal_cm'], 1)} | "
                  + ("✅ 통과" if r["ok"] else "❌ " + ", ".join(r["why"])))
    n_ok = sum(r["ok"] for r in allr)
    print(f"\n합: 통과 {n_ok} / {len(allr)}")
    pairs = {}
    for r in allr:
        pairs.setdefault(r["pair"], [0, 0])
        pairs[r["pair"]][1] += 1
        pairs[r["pair"]][0] += int(r["ok"])
    if any(k for k in pairs):
        print("쌍별 통과 (쌍 번호는 0 부터): " + ", ".join(f"{k} {v[0]}/{v[1]}" for k, v in sorted(pairs.items(), key=lambda kv: int(kv[0] or 0))))
    if a.list_out:
        Path(a.list_out).expanduser().write_text("".join(r["name"] + "\n" for r in allr if r["ok"]), encoding="utf-8")
        print(f"통과 목록: {a.list_out} ({n_ok} 줄)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
