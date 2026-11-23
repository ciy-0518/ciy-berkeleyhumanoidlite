#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""좌표 쌍 평가 (작업 지시 19-c1). ~/venvs/learn 에서 돌린다. 시뮬레이션 (policy_sim, 가상 캔 엄격 판정) 으로
좌표 쌍 파일 × 방법 × 반복을 돌리고, 단계별 결과와 목표 거리를 낸다.

방법 (--method 이름=종류, 여러 번):
  nearest           가장 가까운 시연 재생: 시작 · 도착 거리 합이 가장 작은 시연을 그대로 (관측 무시)
  shift             옮김 + IK (학습 없음): 그 시연을 aug_demo.augment 로 평가 좌표에 옮겨 재생 (거르지 않는다)
  act:<폴더> · mlp:<model.pt>   학습한 정책 (조건 입력 정책이면 조건 = 평가 좌표). --meas-sim 이면 관측 관절에 처짐 모델 + 잡음
반복: 씨앗 0 .. n−1 마다 캔 시작 자리 ±--can-jitter-cm, 첫 자세 관절 ±--pose-jitter-deg.
판정:
  단계  잡기 실패 (L 이 못 잡음) / 인계 실패 (R 이 못 받음) / 놓기 실패 (R 이 받았으나 목표에 못 세움 · 넘어짐) / 성공
  성공 (5 cm)   가상 캔 자동 성공 = 목표 5 cm 안에 서고 두 손을 뺌
  성공 (1.5 cm) 위 + 놓은 자리가 목표에서 1.5 cm 안 (엄격 판정의 목표 기준)
  엄격 실패      표에 따로 센다 (열린 날이 캔을 밂 · 덜 들어감 · 옆 비킴 — 실기라면 캔을 밀거나 놓칠 판)

    eval_pairs.py --pairs config/pairs_eval_19b.txt --demos files_x1.txt --method nearest=nearest --method shift=shift \\
                  --method x1=act:<…/pretrained_model> --n 3 --out DIR [--meas-sim] [--limit-pairs K] [--max-s 45]
출력 (DIR): <방법>/<쌍>_s<씨앗>.json · .npz (policy_sim 꼴), results.json (줄 모음), summary.md (표). 그림은 eval_plot.py (시스템 python3)."""
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
import learn_data as L  # noqa: E402
import demo_export as DE  # noqa: E402
import policy_sim as PS  # noqa: E402
import virtual_can as VC  # noqa: E402
import aug_demo as AD  # noqa: E402

NARROW_CM = 1.5


def stage_of(s: dict) -> str:
    seen = set(s.get("states_seen") or [])
    if s.get("result") == "auto_success":
        return "성공"
    if not seen & {"held_L", "held_both"}:
        return "잡기 실패"
    if not seen & {"held_R"}:
        return "인계 실패"
    return "놓기 실패"


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load_demos(path: str):
    out = []
    for p in [x.strip() for x in Path(path).expanduser().read_text(encoding="utf-8").splitlines() if x.strip() and not x.startswith("#")]:
        ep = DE.load_episode(p)
        st, go = AD.demo_xy(ep["meta"])
        out.append({"path": p, "meta": ep["meta"], "start": st, "goal": go, "name": f"{ep['meta']['record']}_ep{int(ep['meta']['episode']):02d}"})
    return out


def nearest(demos, st, go):
    d = [math.dist(st, x["start"]) + math.dist(go, x["goal"]) for x in demos]
    k = int(np.argmin(d))
    return demos[k], round(math.dist(st, demos[k]["start"]), 2), round(math.dist(go, demos[k]["goal"]), 2)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--pairs", required=True)
    ap.add_argument("--demos", default=None, help="nearest · shift 가 고를 시연 npz 목록 (한 줄에 하나)")
    ap.add_argument("--method", action="append", required=True, help="이름=종류 (nearest · shift · act:<폴더> · mlp:<pt>)")
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--out", required=True)
    ap.add_argument("--meas-sim", action="store_true", help="정책 관측에 처짐 모델 + 잡음 (학습을 --meas-sim 으로 했으면)")
    ap.add_argument("--max-s", type=float, default=45.0, help="정책 최대 시간 (초)")
    ap.add_argument("--can-jitter-cm", type=float, default=0.3)
    ap.add_argument("--pose-jitter-deg", type=float, default=1.0)
    ap.add_argument("--limit-pairs", type=int, default=None, help="앞의 K 쌍만 (시험용)")
    ap.add_argument("--grasp-mode", choices=("bottom", "top"), default="bottom", help="19-x (기본 bottom): 시뮬 가상 캔 잡기 모드 (top = L 위쪽 먼저, R 아래쪽)")
    ap.add_argument("--grasp-real", action="store_true",
                    help="19-x (기본 끔): 시뮬 가상 캔 실물 기준 판정 (인계 깊이 · 비킴 · 누름 · 부딪힘). 켜면 성공 (ok5 · ok15) 은 real_fail 0 일 때만")
    ap.add_argument("--handover-assist", default=None, metavar="JSON",
                    help="19-x21 (기본 끔): 정책 방법 (act · mlp) 에만 top 인계 보정 + L 여는 안전 규칙 (tools/handover_assist.py, 목표 json 예: config/handover_assist_top_19x21.json)")
    ap.add_argument("--grasp-assist", default=None, metavar="JSON",
                    help="19-x22 (기본 끔): 정책 방법에만 top L 잡기 보정 (tools/handover_assist.GraspAssist, 목표 json 예: config/grasp_assist_top_19x22.json). --handover-assist 와 같이 켤 수 있다")
    ap.add_argument("--roll-stop", action="store_true", help="19-x20 (기본 끔): 시뮬 모터 명령의 어깨 roll 을 실물 멈춤 (워치독 ROLL_STOP_DEG) 으로 묶는다")
    ap.add_argument("--first-from", default=None,
                    help="19-x17 (기본 끔 = 정책 기록 이름으로 학습 데이터 폴더에서 찾음): 정책 시작 자세 = 이 목록 (npz 한 줄에 하나) 의 첫 행동 평균. "
                         "다듬은 기록 (trimB_*) 처럼 학습 데이터 폴더에 없는 기록으로 학습한 정책용")
    a = ap.parse_args()
    if a.roll_stop:                                    # 19-x20 (켤 때만)
        PS.ROLL_STOP = PS.roll_stop_watchdog()
    if a.grasp_mode != "bottom" or a.grasp_real:       # 19-x (켤 때만)
        PS.VCAN_EXTRA.update({**({"grasp_mode": a.grasp_mode} if a.grasp_mode != "bottom" else {}), **({"real": True} if a.grasp_real else {})})
    args_out = {k: v for k, v in vars(a).items() if not ((k == "grasp_mode" and v == "bottom") or (k == "grasp_real" and not v) or (k == "first_from" and v is None) or (k == "roll_stop" and not v) or (k == "handover_assist" and v is None) or (k == "grasp_assist" and v is None))}   # 끄면 기록이 지금과 같게
    ha_target = None
    if a.handover_assist:                              # 19-x21 (켤 때만)
        import handover_assist as HA
        ha_target = HA.load_target(a.handover_assist)
    ga_target = None
    if a.grasp_assist:                                 # 19-x22 (켤 때만)
        import handover_assist as HA
        ga_target = HA.load_grasp_target(a.grasp_assist)
    pairs = VC.load_pairs(a.pairs)[:a.limit_pairs]
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    demos = load_demos(a.demos) if a.demos else []
    ctx = None
    rows, t00 = [], time.time()
    for spec in a.method:
        label, _, kind = spec.partition("=")
        kind = kind or label
        md = out / label
        md.mkdir(exist_ok=True)
        pol = None
        if kind not in ("nearest", "shift"):
            pol = PS.load_policy(kind)
            if a.first_from:                           # 19-x17 (켤 때만)
                fl = [ln.strip() for ln in Path(a.first_from).expanduser().read_text(encoding="utf-8").splitlines() if ln.strip()]
                pol.first = np.mean([np.asarray(L.arrays(f)["action"], float)[0, :10] for f in fl], 0).reshape(2, 5)
        elif not demos:
            raise SystemExit(f"{label}: --demos 가 필요하다")
        elif kind == "shift" and ctx is None:
            ctx = AD.Ctx()
        t0 = time.time()
        for pi, (sx, sy, gx, gy) in enumerate(pairs):
            st, go = (sx, sy), (gx, gy)
            extra = {}
            if pol is None:
                dm, dst, dgo = nearest(demos, st, go)
                arr = L.arrays(dm["path"])
                extra = {"demo": dm["name"], "demo_start": list(dm["start"]), "demo_goal": list(dm["goal"]), "demo_dist_start_cm": dst, "demo_dist_goal_cm": dgo}
                if kind == "nearest":
                    act = np.asarray(arr["action"], float)
                else:
                    act, inf = AD.augment(ctx, arr, AD.shift_for(ctx, dm["start"], st), AD.shift_for(ctx, dm["goal"], go))
                    extra["shift_info"] = {k: v for k, v in (inf or {}).items() if k != "seg"}
                max_s = float(arr["t"][-1] - arr["t"][0]) + 6.0
            for seed in range(a.n):
                nm = f"p{pi + 1:02d}_s{seed}"
                if pol is None and act is None:
                    r = {"result": "no_episode", "why": "옮김 + IK 를 못 만듦 (그리퍼 차례)", "states_seen": []}
                else:
                    with contextlib.redirect_stdout(io.StringIO()):
                        if pol is None:
                            r = PS.rollout(AD.Fixed(act), nm, max_s=max_s, out_dir=md, start=st, goal=go, strict=True, seed=seed,
                                           can_jitter_cm=a.can_jitter_cm, pose_jitter_deg=a.pose_jitter_deg, first=act[0, :10].reshape(2, 5), align_axis_cm=2.0)
                        else:
                            r = PS.rollout(pol, nm, max_s=a.max_s, out_dir=md, start=st, goal=go, strict=True, seed=seed, meas_sim=a.meas_sim,
                                           can_jitter_cm=a.can_jitter_cm, pose_jitter_deg=a.pose_jitter_deg, align_axis_cm=2.0,
                                           **({"assist": ha_target} if ha_target is not None else {}),
                                           **({"grasp_assist": ga_target} if ga_target is not None else {}))
                gd = fnum(r.get("goal_dist_cm"))
                stage = stage_of(r)
                row = {"method": label, "kind": kind if pol is None else kind.split(":")[0], "pair": pi + 1, "start": list(st), "goal": list(go), "seed": seed,
                       "result": r.get("result"), "stage": stage, "goal_dist_cm": gd, "ok5": stage == "성공", "ok15": stage == "성공" and gd is not None and gd <= NARROW_CM,
                       "strict_flags": (r.get("strict") or {}).get("strict_flags"), "strict_fail": (r.get("strict") or {}).get("strict_fail"),
                       "final_minus_goal_cm": r.get("final_minus_goal_cm"), "sim_s": r.get("sim_s"), "why": r.get("why"), **extra}
                if r.get("assist") is not None:         # 19-x21 (켤 때만): 규칙이 한 일
                    row["assist"] = r["assist"]
                if r.get("grasp_assist") is not None:   # 19-x22 (켤 때만)
                    row["grasp_assist"] = r["grasp_assist"]
                if a.grasp_real:                       # 19-x (켤 때만): 실물 기준 실패면 성공이 아니다
                    rl = r.get("real") or {}
                    row.update({"real_fail": rl.get("real_fail"), "real_flags": rl.get("real_flags"), "real_depth_cm": rl.get("real_depth_cm")})
                    if str(rl.get("real_fail", "0")) not in ("0", "", "None"):
                        row["ok5"] = row["ok15"] = False
                rows.append(row)
                print(f"{label:10s} 쌍 {pi + 1:2d} {st} → {go} 씨앗 {seed}: {stage:6s} 목표 {gd} cm {row['strict_flags'] or ''}", flush=True)
            (out / "results.json").write_text(json.dumps({"pairs": [list(p) for p in pairs], "args": args_out, "rows": rows}, ensure_ascii=False, indent=0, default=str), encoding="utf-8")
        print(f"{label}: {round(time.time() - t0)} 초", flush=True)
    # 표
    labels = [s.partition("=")[0] for s in a.method]
    stages = ("성공", "놓기 실패", "인계 실패", "잡기 실패")
    lines = [f"# 좌표 쌍 평가 — {Path(a.pairs).name} 앞 {len(pairs)} 쌍 × {a.n} 번 (캔 ±{a.can_jitter_cm:g} cm, 첫 자세 ±{a.pose_jitter_deg:g}°)", "",
             "| 방법 | 성공 (5 cm) | 성공 (1.5 cm) | 성공 중 엄격 실패 | " + " | ".join(stages[1:]) + " | 목표 거리 중앙 (성공만) |", "|---|---|---|---|---|---|---|---|"]
    for lb in labels:
        rr = [r for r in rows if r["method"] == lb]
        n = len(rr)
        gds = [r["goal_dist_cm"] for r in rr if r["ok5"] and r["goal_dist_cm"] is not None]
        sf = sum(1 for r in rr if r["ok5"] and str(r.get("strict_fail")) not in ("0", "", "None"))
        lines.append(f"| {lb} | {sum(r['ok5'] for r in rr)} / {n} | {sum(r['ok15'] for r in rr)} / {n} | {sf} | "
                     + " | ".join(str(sum(r['stage'] == s for r in rr)) for s in stages[1:]) + f" | {round(float(np.median(gds)), 2) if gds else '-'} cm |")
    lines += ["", "쌍마다 성공 (1.5 cm) 수 / 반복:", "", "| 쌍 | 시작 → 도착 | " + " | ".join(labels) + " |", "|---|---|" + "---|" * len(labels)]
    for pi, p in enumerate(pairs):
        lines.append(f"| {pi + 1} | ({p[0]:g}, {p[1]:g}) → ({p[2]:g}, {p[3]:g}) | "
                     + " | ".join(f"{sum(r['ok15'] for r in rows if r['method'] == lb and r['pair'] == pi + 1)} / {sum(1 for r in rows if r['method'] == lb and r['pair'] == pi + 1)}" for lb in labels) + " |")
    lines += ["", f"걸린 시간 {round(time.time() - t00)} 초"]
    (out / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines[:4 + len(labels)]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
