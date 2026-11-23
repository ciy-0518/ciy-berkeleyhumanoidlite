#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""위쪽 잡기 실기 계획 (작업 지시 19-x 6). 평가 좌표 쌍마다 (1) 위쪽 잡기 정책 명령 (can_go 와 같은 옵션 · 로그 이름 real19x_bNN_pNN_top),
(2) 비교로 지금 방식 (anypos_demo19j_x5) 점검 결과 — 도달 불가면 그 까닭을 적고 실행하지 않는다.
  top19x_plan.py [--pairs config/pairs_top_eval_19x.txt] [--policy-top …] [--demos-top …] [--out ~/cc_analysis/real19x]
출력: runbook.md, plan.json, results.csv (사용자 칸 + real_eval_fill 칸, 이미 있으면 새로 쓰지 않음)."""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import can_go as CG  # noqa: E402
import virtual_can as VC  # noqa: E402

FIELDS_USER = ["판", "쌍", "시작", "도착", "방법", "로그", "결과", "목적지_앞뒤_cm", "목적지_좌우_cm", "메모"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--pairs", default=str(CG.TELEOP / "config/pairs_top_eval_19x.txt"))
    ap.add_argument("--policy-top", default=CG.POLICY_TOP)
    ap.add_argument("--demos-top", default=str(CG.DEMOS_TOP))
    ap.add_argument("--out", default=str(Path.home() / "cc_analysis/real19x"))
    a = ap.parse_args()
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    ck = CG.Checker(Path(CG.DEMOS))
    demos_top, note = ck._load_demos(Path(a.demos_top).expanduser())
    names = []
    for line in Path(a.pairs).read_text(encoding="utf-8").splitlines():
        if line.split("#", 1)[0].strip():
            c = line.split("#", 1)[1] if "#" in line else ""
            names.append([w.strip() for w in c.replace("→", " ").split()][:2])
    pairs = VC.load_pairs(a.pairs)
    boards = []
    pol = a.policy_top.replace(str(Path.home()), "~")
    for i, (sx, sy, gx, gy) in enumerate(pairs, 1):
        st, go = (sx, sy), (gx, gy)
        sn, gn = (names[i - 1] + ["", ""])[:2] if i - 1 < len(names) else (f"t{i}", f"u{i}")
        bb, _, bi = CG.run_checks(ck, st, go)
        bi.pop("_kinds", None)
        tb, tw, ti = CG.run_checks(ck, st, go, mode="top", demos=demos_top)
        ti.pop("_kinds", None)
        log = f"real19x_b{i:02d}_p{i:02d}_top"
        cmd, _ = CG.build_cmd(pol, st, go, f"~/teleop-docs/_teleop/logs/{log}.csv", top=True)
        boards.append({"board": i, "pair": i, "sname": sn, "gname": gn, "start": list(st), "goal": list(go), "start_disp": CG.disp(*st), "goal_disp": CG.disp(*go),
                       "method": "top", "log": log, "cmd": cmd, "top_blocks": tb, "top_warns": tw, "top_demo": ti.get("demo"),
                       "bottom_blocks": bb, "bottom_reachable": not bb})
    plan = {"tag": "real19x", "pairs_file": a.pairs, "policy_top": a.policy_top, "demos_top": a.demos_top, "demos_top_note": note,
            "options": {"unified": CG.UNIFIED, "handover_depth": CG.HANDOVER_DEPTH, "policy_only": CG.POLICY_ONLY, "place_settle": CG.PLACE_SETTLE, "tip_roll_stop": CG.TIP_ROLL_STOP}, "boards": boards}
    (out / "plan.json").write_text(json.dumps(plan, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    rc = out / "results.csv"
    if not rc.exists():
        import real_eval_prep as RP
        with rc.open("w", encoding="utf-8-sig", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(FIELDS_USER + RP.FIELDS_AUTO)
            for b in boards:
                w.writerow([b["board"], b["pair"], f"{b['sname']} ({b['start'][0]:g}, {b['start'][1]:g})", f"{b['gname']} ({b['goal'][0]:g}, {b['goal'][1]:g})",
                            "top", b["log"], "", "", "", ""] + [""] * len(RP.FIELDS_AUTO))
    lines = ["# 실기 평가 실행표 (real19x — 위쪽 잡기: L 이 캔 위쪽, R 이 아래쪽)", "",
             "판 표시: 로봇 쪽 모서리에서 앞으로 · 판 왼쪽 끝 (로봇 왼쪽) 에서 오른쪽으로 (cm). 비교: 지금 방식 (anypos_demo19j_x5) 은 이 좌표에서 도달 불가라 실행하지 않는다.", ""]
    for b in boards:
        lines += [f"## 판 {b['board']} — {b['sname']} (모서리 {b['start_disp'][0]:g} · 왼쪽 끝 {b['start_disp'][1]:g}) → {b['gname']} "
                  f"(모서리 {b['goal_disp'][0]:g} · 왼쪽 끝 {b['goal_disp'][1]:g})", "- " + CG.RUN_STEP, "```", b["cmd"], "```"]
        lines.append("- 지금 방식: " + ("도달 가능 (비교 판으로 돌릴 수 있음)" if b["bottom_reachable"] else "🔴 도달 불가 — " + " / ".join(b["bottom_blocks"])))
        if b["top_blocks"]:
            lines.append("- 🔴 위쪽 잡기 점검: " + " / ".join(b["top_blocks"]))
        if b["top_warns"]:
            lines.append("- 🟡 위쪽 잡기 경고: " + " / ".join(b["top_warns"]))
        lines.append("")
    (out / "runbook.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"판 {len(boards)} → {out} (지금 방식 도달 가능 {sum(b['bottom_reachable'] for b in boards)} 판, 위쪽 잡기 점검 막힘 {sum(bool(b['top_blocks']) for b in boards)} 판)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
