#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""실기 평가 준비 (작업 지시 19-d). ~/venvs/learn 에서 돌린다. 로봇 · CAN · 시리얼은 쓰지 않는다 (시뮬레이션만).
좌표 쌍 파일에서 고른 쌍마다
  1. "옮김 + IK" (shift) 재생 파일: eval_pairs 의 shift 와 같은 궤적 (가장 가까운 시연 → aug_demo.augment 로 그 좌표에 옮김, 거르지 않음).
     흔들기 없이 (캔 · 첫 자세 그대로) 시뮬레이션으로 한 번 돌려 그 npz (demo_export 꼴, meta 에 좌표 쌍 S0 · G0) 를 저장한다 → replay_run 이 읽는다.
     거르기 검사 (자세 IK · 관절 한계 · 속도 튐 · 실기 보정 여유) 는 막지 않고 적기만 한다.
  2. 실행표: 쌍마다 정책 → shift 차례로 판 둘, 판마다 한 줄 명령 (통일안 보정, 로그 이름 고정), 판 표시 거리.
  3. 결과 기록표 (csv, 사용자가 채울 칸 + real_eval_fill.py 가 로그로 채울 칸).

    real_eval_prep.py --select 1,3,5,7,9,11,13,15,17,19,21,23 --out ~/cc_analysis/real19d
출력 (OUT): shift/<tag>_pNN_shift.npz, plan.json, runbook.md, results.csv"""
from __future__ import annotations

import argparse
import contextlib
import csv
import io
import json
import shutil
import sys
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import learn_data as L  # noqa: E402
import policy_sim as PS  # noqa: E402
import virtual_can as VC  # noqa: E402
import aug_demo as AD  # noqa: E402
import eval_pairs as EP  # noqa: E402

TELEOP = TOOLS.parent
HOME = Path.home()
# 통일안 (18-b · 18-e · 18-e 추가, 작업 지시 19-d §0). 두 방법 모두에 같은 몫 + 방법마다 그 방법에만 뜻이 있는 몫.
UNIFIED = "--sag-comp --no-grip-warm --grip-open-first --tip-cap-deg 18 --tip-offset-cm L 3 0 0 R 3 0 0 --place-tip-offset-cm R -2 0 0 --carry-lift-cm R 2"
POLICY_ONLY = "--policy-end-after-release 2 --policy-max-s 40 --policy-obs sag-model --policy-can-basis encoder-nocorr"
REPLAY_ONLY = "--speed 1"            # 재생 기본은 0.5 배 — 정책 · 시뮬레이션 평가 · 시연은 1 배
X_EDGE, Y_EDGE = 7.0, 30.25          # 판 표시: 로봇 쪽 모서리에서 = x − 7, 왼쪽 끝에서 = 30.25 − y
FIELDS_USER = ["판", "쌍", "시작", "도착", "방법", "로그", "결과", "목적지_앞뒤_cm", "목적지_좌우_cm", "메모"]
FIELDS_AUTO = ["로그_파일", "L_닫힘_s", "R_닫힘_s", "L_열림_s", "R_열림_s", "실행_s", "끝_까닭", "그리퍼_순서",
               "엔코더_R집게_앞뒤_cm", "엔코더_R집게_좌우_cm", "논리_R집게_앞뒤_cm", "논리_R집게_좌우_cm", "논리_R집게_높이_cm"]


def read_pairs(path: Path):
    """좌표 쌍 + 줄 끝 설명의 자리 이름 (# s1 → g1)."""
    out = []
    nums = VC.load_pairs(path)
    k = 0
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.split("#", 1)[0].strip():
            continue
        c = line.split("#", 1)[1] if "#" in line else ""
        nm = [w.strip() for w in c.replace("→", " ").split()][:2]
        out.append({"xy": nums[k], "sname": nm[0] if len(nm) > 0 else f"S{k + 1}", "gname": nm[1] if len(nm) > 1 else f"G{k + 1}"})
        k += 1
    return out


def disp(x, y):
    return round(x - X_EDGE, 2), round(Y_EDGE - y, 2)


def sim_ref(eval_json: Path):
    """19-c2 시뮬레이션 평가 결과 (방법 · 쌍 → 성공 5 cm · 1.5 cm 수 / 반복)."""
    if not eval_json.exists():
        return {}
    d = json.loads(eval_json.read_text(encoding="utf-8"))
    out = {}
    for r in d["rows"]:
        k = (r["method"], int(r["pair"]))
        o = out.setdefault(k, [0, 0, 0])
        o[0] += int(r["ok5"])
        o[1] += int(r["ok15"])
        o[2] += 1
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--pairs", default=str(TELEOP / "config/pairs_eval_19b.txt"))
    ap.add_argument("--select", default="1,3,5,7,9,11,13,15,17,19,21,23", help="쌍 번호 (1 부터)")
    ap.add_argument("--demos", default=str(HOME / "cc_analysis/anypos_c2/demo19b/aug/files_x1.txt"), help="shift 가 고를 시연 (19-c2 전수 검사 통과 판)")
    ap.add_argument("--policy", default=str(HOME / "cc_analysis/learn/runs/anypos_demo19b_x5/checkpoints/last/pretrained_model"))
    ap.add_argument("--policy-label", default="x5")
    ap.add_argument("--eval", default=str(HOME / "cc_analysis/anypos_c2/demo19b/eval/results.json"), help="시뮬레이션 평가 (참고 칸)")
    ap.add_argument("--tag", default="real19d")
    ap.add_argument("--out", default=str(HOME / "cc_analysis/real19d"))
    ap.add_argument("--fixed-policy", default=str(HOME / "cc_analysis/learn/runs/act_multi_xy/checkpoints/last/pretrained_model"),
                    help="시간이 남을 때: 고정 네 조합 정책")
    a = ap.parse_args()
    out = Path(a.out).expanduser()
    (out / "shift").mkdir(parents=True, exist_ok=True)
    tmp = out / "_tmp"
    tmp.mkdir(exist_ok=True)
    pairs = read_pairs(Path(a.pairs).expanduser())
    sel = [int(v) for v in a.select.split(",") if v.strip()]
    demos = EP.load_demos(a.demos)
    ctx = AD.Ctx()
    ref = sim_ref(Path(a.eval).expanduser())
    logs = TELEOP / "logs"
    boards, shifts = [], {}
    for pi in sel:
        p = pairs[pi - 1]
        sx, sy, gx, gy = p["xy"]
        st, go = (sx, sy), (gx, gy)
        dm, dst, dgo = EP.nearest(demos, st, go)
        arr = L.arrays(dm["path"])
        act, inf = AD.augment(ctx, arr, AD.shift_for(ctx, dm["start"], st), AD.shift_for(ctx, dm["goal"], go))
        sh = {"pair": pi, "demo": dm["name"], "demo_file": dm["path"], "demo_start": list(dm["start"]), "demo_goal": list(dm["goal"]),
              "demo_dist_start_cm": dst, "demo_dist_goal_cm": dgo}
        if act is None:
            sh.update(ok=False, why=(inf or {}).get("why", "옮김 + IK 를 못 만듦"))
            shifts[pi] = sh
            print(f"쌍 {pi}: shift 를 못 만듦 — {sh['why']}", flush=True)
        else:
            flags = []
            if inf["key_ik_err_cm"] > AD.KEY_ERR_CM or inf["key_pitch_err_deg"] > AD.KEY_PITCH_DEG or inf["key_lim_frames"] > 0:
                flags.append("자세 (IK · 한계 · 숙임)")
            if inf["max_speed_deg_s"] > AD.SPEED_GAIN * inf["orig_speed_deg_s"] + AD.SPEED_ADD:
                flags.append("속도 튐")
            iLc, _, _, iRo = AD.segments(act)
            cL, cR = AD.corr_check(ctx, "left", act[iLc, 0:5]), AD.corr_check(ctx, "right", act[max(0, iRo - 1), 5:10])
            if not (cL["ok"] and cR["ok"]):
                flags.append("실기 보정 여유")
            nm = f"{a.tag}_p{pi:02d}_shift"
            with contextlib.redirect_stdout(io.StringIO()):
                s = PS.rollout(AD.Fixed(act), nm, max_s=float(arr["t"][-1] - arr["t"][0]) + 6.0, out_dir=tmp, start=st, goal=go, strict=True,
                               first=act[0, :10].reshape(2, 5), align_axis_cm=2.0)
            z = np.load(tmp / f"{nm}.npz", allow_pickle=True)
            meta = json.loads(str(z["meta"]))
            stv = s.get("strict") or {}
            meta.update({"record": f"{a.tag}_shift", "episode": pi, "options": dm["meta"].get("options"),
                         "shift": {"pair": pi, "start_xy_cm": list(st), "goal_xy_cm": list(go), "src_record": dm["meta"].get("record"),
                                   "src_episode": dm["meta"].get("episode"), "src_file": dm["path"],
                                   "info": {k: v for k, v in inf.items() if k != "seg"}, "corr": {"L": cL, "R": cR}, "flags": flags}})
            f = out / "shift" / f"{nm}.npz"
            np.savez_compressed(f, meta=np.array(json.dumps(meta, ensure_ascii=False, default=float)), **{k: z[k] for k in z.files if k != "meta"})
            sh.update(ok=True, file=str(f), flags=flags, sim_result=s.get("result"), sim_goal_cm=EP.fnum(s.get("goal_dist_cm")),
                      sim_stage=EP.stage_of(s), sim_strict=stv.get("strict_flags"), sim_strict_fail=stv.get("strict_fail"),
                      duration_s=round(float(z["t"][-1] - z["t"][0]), 1), key_ik_err_cm=inf["key_ik_err_cm"], max_speed_deg_s=inf["max_speed_deg_s"],
                      orig_speed_deg_s=inf["orig_speed_deg_s"], clip_max_deg=s.get("safety_clip_max_deg"))
            shifts[pi] = sh
            print(f"쌍 {pi} {p['sname']}→{p['gname']}: 시연 {dm['name']} (시작 {dst} · 도착 {dgo} cm 옮김) → 시뮬 {sh['sim_stage']} "
                  f"목표 {sh['sim_goal_cm']} cm {sh['sim_strict'] or ''} {'검사: ' + ', '.join(flags) if flags else ''}", flush=True)
        for method in (a.policy_label, "shift"):
            b = len(boards) + 1
            log = f"{a.tag}_b{b:02d}_p{pi:02d}_{method}"
            logp = f"~/teleop-docs/_teleop/logs/{log}.csv"
            if method == "shift":
                core = f"~/run.sh replay {sh['file'].replace(str(HOME), '~')} {REPLAY_ONLY} {UNIFIED}" if sh.get("ok") else None
            else:
                core = f"~/run.sh policy {a.policy.replace(str(HOME), '~')} --can-start {sx:g},{sy:g} --can-goal {gx:g},{gy:g} {UNIFIED} {POLICY_ONLY}"
            rr = ref.get(("shift" if method == "shift" else a.policy_label, pi))
            boards.append({"board": b, "pair": pi, "method": method, "start": [sx, sy], "goal": [gx, gy], "sname": p["sname"], "gname": p["gname"],
                           "start_disp": disp(sx, sy), "goal_disp": disp(gx, gy), "log": log,
                           "cmd": (core + f" --log-csv {logp}") if core else None, "core": core,
                           "sim_ok5": rr and f"{rr[0]}/{rr[2]}", "sim_ok15": rr and f"{rr[1]}/{rr[2]}"})
    shutil.rmtree(tmp, ignore_errors=True)
    # 고정 네 조합 (시간이 남을 때)
    fixed = []
    for s_, g_ in (("L1", "R1"), ("L1", "R2"), ("L2", "R1"), ("L2", "R2")):
        log = f"{a.tag}_f_{s_}{g_}_multi"
        fixed.append({"start": s_, "goal": g_, "log": log,
                      "core": f"~/run.sh policy {a.fixed_policy.replace(str(HOME), '~')} --can-start {s_} --can-goal {g_} {UNIFIED} {POLICY_ONLY}",
                      "cmd": f"~/run.sh policy {a.fixed_policy.replace(str(HOME), '~')} --can-start {s_} --can-goal {g_} {UNIFIED} {POLICY_ONLY} "
                             f"--log-csv ~/teleop-docs/_teleop/logs/{log}.csv"})
    plan = {"tag": a.tag, "pairs_file": a.pairs, "select": sel, "policy": a.policy, "policy_label": a.policy_label, "unified": UNIFIED,
            "policy_only": POLICY_ONLY, "replay_only": REPLAY_ONLY, "boards": boards, "shift": {str(k): v for k, v in shifts.items()}, "fixed": fixed,
            "slots": {}}
    for b in boards:
        plan["slots"][b["sname"]] = {"xy": b["start"], "disp": b["start_disp"], "kind": "시작"}
        plan["slots"][b["gname"]] = {"xy": b["goal"], "disp": b["goal_disp"], "kind": "목적지"}
    (out / "plan.json").write_text(json.dumps(plan, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    # 결과 기록표 (이미 있으면 사용자 칸을 지키려고 새로 쓰지 않는다)
    rc = out / "results.csv"
    if not rc.exists():
        with rc.open("w", encoding="utf-8-sig", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(FIELDS_USER + FIELDS_AUTO)
            for b in boards:
                w.writerow([b["board"], b["pair"], f"{b['sname']} ({b['start'][0]:g}, {b['start'][1]:g})", f"{b['gname']} ({b['goal'][0]:g}, {b['goal'][1]:g})",
                            b["method"], b["log"], "", "", "", ""] + [""] * len(FIELDS_AUTO))
            for f_ in fixed:
                w.writerow(["", f"{f_['start']}→{f_['goal']}", f_["start"], f_["goal"], "multi", f_["log"], "", "", "", ""] + [""] * len(FIELDS_AUTO))
    # 실행표
    lines = [f"# 실기 평가 실행표 ({a.tag})", "", "판 표시: 로봇 쪽 모서리에서 앞으로 · 판 왼쪽 끝 (로봇 왼쪽) 에서 오른쪽으로 (cm).", ""]
    for b in boards:
        lines += [f"## 판 {b['board']} — 쌍 {b['pair']} {b['method']}: 캔 {b['sname']} (모서리 {b['start_disp'][0]:g} · 왼쪽 끝 {b['start_disp'][1]:g}) → "
                  f"{b['gname']} (모서리 {b['goal_disp'][0]:g} · 왼쪽 끝 {b['goal_disp'][1]:g})", "```", b["cmd"] or "(shift 파일 없음 — 건너뜀)", "```", ""]
    lines += ["## 시간이 남으면 — 고정 네 조합 (act_multi_xy)", ""]
    for f_ in fixed:
        lines += [f"{f_['start']} → {f_['goal']}", "```", f_["cmd"], "```", ""]
    (out / "runbook.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"판 {len(boards)} · shift {sum(1 for v in shifts.values() if v.get('ok'))} / {len(shifts)} → {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
