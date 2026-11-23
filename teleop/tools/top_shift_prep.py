#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""위쪽 잡기 shift 실기 준비 (작업 지시 19-x16). ~/venvs/learn 에서 돌린다. 로봇 · CAN · 시리얼은 쓰지 않는다 (CPU 시뮬레이션만).
real_eval_prep.py 의 shift 와 같은 꼴을 top 으로: 판마다
  1. 가장 가까운 top 시연 (eval_pairs nearest 와 같은 식) → aug_demo.augment 로 그 좌표에 옮김 (거르지 않음, 검사는 적기만).
     가상 캔 grasp_mode top · 실물 기준으로 흔들기 없이 한 번 시뮬레이션 → 그 npz (meta 에 좌표 쌍) 를 저장 → ~/run.sh replay 가 읽는다.
  2. 명령 한 줄: ~/run.sh replay <npz> --speed 1 + can_go.UNIFIED (real19x 와 같은 보정). --handover-depth-cm 은 넣지 않는다 (아래쪽 잡기 규칙, top 근거 없음).
     정책 전용 옵션 (can_go.POLICY_ONLY) 은 재생에 뜻이 없어 넣지 않는다 (real19j shift 판과 같음).
  3. 점검 (실행 안 함): can_go.run_checks(mode=top) 막힘 · 경고, 명령 인자를 replay_run.parse 로 읽어 봄, npz 를 demo_export 로 읽어 봄.
    top_shift_prep.py --boards boards.json --demos ~/cc_analysis/top19x/run/aug/files_x1.txt --eval a/results.json --eval b/results.json --out ~/cc_analysis/real19x16
  boards.json = [{"label", "start": [x, y], "goal": [x, y], "trained": bool}] (순서 = 판 순서)
출력 (OUT): shift/<tag>_bNN_shift.npz, plan16.json, runbook16.md, results16.csv (있으면 새로 쓰지 않음)."""
from __future__ import annotations

import argparse
import contextlib
import csv
import io
import json
import shlex
import shutil
import sys
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import learn_data as L  # noqa: E402
import policy_sim as PS  # noqa: E402
import aug_demo as AD  # noqa: E402
import eval_pairs as EP  # noqa: E402
import demo_export as DE  # noqa: E402
with contextlib.redirect_stdout(io.StringIO()):
    import can_go as CG  # noqa: E402

HOME = Path.home()
REPLAY_ONLY = "--speed 1"
FIELDS = ["판", "쌍", "자리", "캔 (모서리 · 왼쪽 끝)", "목적지 (모서리 · 왼쪽 끝)", "방법", "시연", "로그", "잡기", "인계", "놓기", "목적지_앞뒤_cm", "목적지_좌우_cm", "메모"]


def sim_ref(paths):
    """eval_pairs results.json 들에서 shift 결과를 좌표로 찾는다 → {(sx, sy, gx, gy): [성공 1.5 cm, 반복, 단계들]}."""
    out = {}
    for p in paths:
        d = json.loads(Path(p).expanduser().read_text(encoding="utf-8"))
        for r in d["rows"]:
            if r["method"] != "shift":
                continue
            k = tuple(float(v) for v in (*r["start"], *r["goal"]))
            o = out.setdefault(k, [0, 0, []])
            o[0] += int(r["ok15"]); o[1] += 1; o[2].append(r["stage"] if r["ok15"] or r["stage"] != "성공" else "실물 기준 실패")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--boards", required=True)
    ap.add_argument("--demos", default=str(HOME / "cc_analysis/top19x/run/aug/files_x1.txt"), help="shift 가 고를 시연 (top19x.sh 점검 통과 판)")
    ap.add_argument("--eval", action="append", default=[], help="시뮬 평가 results.json (참고 칸, 여러 번)")
    ap.add_argument("--tag", default="real19x16")
    ap.add_argument("--out", default=str(HOME / "cc_analysis/real19x16"))
    a = ap.parse_args()
    PS.VCAN_EXTRA.update({"grasp_mode": "top", "real": True})          # eval_pairs --grasp-mode top --grasp-real 과 같게
    out = Path(a.out).expanduser()
    (out / "shift").mkdir(parents=True, exist_ok=True)
    tmp = out / "_tmp"
    tmp.mkdir(exist_ok=True)
    boards = json.loads(Path(a.boards).expanduser().read_text(encoding="utf-8"))
    demos = EP.load_demos(a.demos)
    ctx = AD.Ctx()
    ref = sim_ref(a.eval)
    ck = CG.Checker(Path(CG.DEMOS))
    demos_top, _ = ck._load_demos(Path(a.demos).expanduser())
    with contextlib.redirect_stdout(io.StringIO()):
        import replay_run as RR
    res = []
    for i, b in enumerate(boards, 1):
        st, go = tuple(float(v) for v in b["start"]), tuple(float(v) for v in b["goal"])
        dm, dst, dgo = EP.nearest(demos, st, go)
        arr = L.arrays(dm["path"])
        act, inf = AD.augment(ctx, arr, AD.shift_for(ctx, dm["start"], st), AD.shift_for(ctx, dm["goal"], go))
        r = {"board": i, "label": b["label"], "trained": bool(b["trained"]), "start": list(st), "goal": list(go), "start_disp": CG.disp(*st), "goal_disp": CG.disp(*go),
             "demo": dm["name"], "demo_start": list(dm["start"]), "demo_goal": list(dm["goal"]), "demo_dist_start_cm": dst, "demo_dist_goal_cm": dgo,
             "log": f"{a.tag}_b{i:02d}_shift"}
        rr = ref.get((*st, *go))
        r["sim_eval"] = f"{rr[0]}/{rr[1]} ({', '.join(rr[2])})" if rr else "-"
        if act is None:
            r.update(ok=False, why=(inf or {}).get("why", "옮김 + IK 를 못 만듦"), cmd=None)
            res.append(r)
            print(f"판 {i}: shift 를 못 만듦 — {r['why']}", flush=True)
            continue
        flags = []
        if inf["key_ik_err_cm"] > AD.KEY_ERR_CM or inf["key_pitch_err_deg"] > AD.KEY_PITCH_DEG or inf["key_lim_frames"] > 0:
            flags.append("자세 (IK · 한계 · 숙임)")
        if inf["max_speed_deg_s"] > AD.SPEED_GAIN * inf["orig_speed_deg_s"] + AD.SPEED_ADD:
            flags.append("속도 튐")
        iLc, _, _, iRo = AD.segments(act)
        cL, cR = AD.corr_check(ctx, "left", act[iLc, 0:5]), AD.corr_check(ctx, "right", act[max(0, iRo - 1), 5:10])
        if not (cL["ok"] and cR["ok"]):
            flags.append("실기 보정 여유")
        ft = []                                                             # runbook 에 쓸 쉬운 말
        if "자세 (IK · 한계 · 숙임)" in flags:
            ft.append(("자세", f"옮긴 잡기 · 놓기 자세의 끝 차 {inf['key_ik_err_cm']:g} cm · 손 숙임 차 {inf['key_pitch_err_deg']:g}° (기준 {AD.KEY_ERR_CM:g} cm · {AD.KEY_PITCH_DEG:g}°)"))
        if "속도 튐" in flags:
            ft.append(("속도", f"관절 최대 속도 {inf['max_speed_deg_s']:g}°/s (원본 {inf['orig_speed_deg_s']:g}, 한도 {AD.SPEED_GAIN * inf['orig_speed_deg_s'] + AD.SPEED_ADD:g})"))
        for sd, c in (("L", cL), ("R", cR)):
            if not c["ok"]:
                ft.append(("보정", f"{sd} 집게 끝 보정 (3 cm) 이 {' · '.join(c['capped']) or '-'} 한계에 걸려 {sd} 끝이 최대 {c['err_cm']:.2f} cm 어긋날 수 있음 "
                                   f"({'잡는' if sd == 'L' else '놓는'} 순간, 처짐 깎임 {c['sag_cut_deg']:g}° · 판 면 여유 {c['table_g_cm']:g} cm)"))
        nm = f"{a.tag}_b{i:02d}_shift"
        with contextlib.redirect_stdout(io.StringIO()):
            s = PS.rollout(AD.Fixed(act), nm, max_s=float(arr["t"][-1] - arr["t"][0]) + 6.0, out_dir=tmp, start=st, goal=go, strict=True,
                           first=act[0, :10].reshape(2, 5), align_axis_cm=2.0)
        z = np.load(tmp / f"{nm}.npz", allow_pickle=True)
        meta = json.loads(str(z["meta"]))
        meta.update({"record": f"{a.tag}_shift", "episode": i, "options": dm["meta"].get("options"),
                     "shift": {"board": i, "start_xy_cm": list(st), "goal_xy_cm": list(go), "src_record": dm["meta"].get("record"),
                               "src_episode": dm["meta"].get("episode"), "src_file": dm["path"], "grasp_mode": "top",
                               "info": {k: v for k, v in inf.items() if k != "seg"}, "corr": {"L": cL, "R": cR}, "flags": flags}})
        f = out / "shift" / f"{nm}.npz"
        np.savez_compressed(f, meta=np.array(json.dumps(meta, ensure_ascii=False, default=float)), **{k: z[k] for k in z.files if k != "meta"})
        real = s.get("real") or {}
        cmd = f"~/run.sh replay {str(f).replace(str(HOME), '~')} {REPLAY_ONLY} {CG.UNIFIED} {CG.PLACE_SETTLE} {CG.TIP_ROLL_STOP} --log-csv ~/teleop-docs/_teleop/logs/{r['log']}.csv"
        tb, tw, _ = CG.run_checks(ck, st, go, mode="top", demos=demos_top)
        argv = [CG.os.path.expanduser(w) if w.startswith("~/") else w for w in shlex.split(cmd)][2:]       # run.sh replay 뒤 = replay_run 인자
        old = sys.argv
        try:
            sys.argv = ["replay_run.py"] + argv
            pa = RR.parse()
            parse_ok, parse_why = True, ""
            bad = [k for k in ("handover_depth_cm",) if getattr(pa, k, None) is not None]
            if bad:
                parse_ok, parse_why = False, "들어가면 안 되는 옵션: " + " ".join(bad)
        except SystemExit as e:
            parse_ok, parse_why = False, f"인자 해석 실패 ({e.code})"
        finally:
            sys.argv = old
        try:
            DE.load_episode(str(f)); load_ok = True
        except Exception as e:      # noqa: BLE001
            load_ok = False; parse_why += f" npz 읽기 실패 {e}"
        r.update(ok=True, file=str(f), cmd=cmd, flags=flags, flag_text=ft, sim_stage=EP.stage_of(s), sim_result=s.get("result"), sim_goal_cm=EP.fnum(s.get("goal_dist_cm")),
                 sim_real_flags=real.get("real_flags"), sim_real_fail=real.get("real_fail"), sim_tilt=real.get("real_tilt_deg"), sim_depth=real.get("real_depth_cm"),
                 duration_s=round(float(z["t"][-1] - z["t"][0]), 1), checks_block=tb, checks_warn=tw, parse_ok=parse_ok and load_ok, parse_why=parse_why.strip())
        res.append(r)
        print(f"판 {i} {b['label']}: 시연 {dm['name']} (옮김 {dst} · {dgo} cm) → 시뮬 {r['sim_stage']} 목표 {r['sim_goal_cm']} cm 실물 {r['sim_real_flags'] or '-'}"
              f"{' (실패)' if r['sim_real_fail'] == '1' else ''} | 평가 {r['sim_eval']} | 점검 막힘 {len(tb)} 경고 {len(tw)} | 인자 {'ok' if r['parse_ok'] else r['parse_why']}"
              + (f" | 검사: {', '.join(flags)}" if flags else ""), flush=True)
    shutil.rmtree(tmp, ignore_errors=True)
    (out / "plan16.json").write_text(json.dumps({"tag": a.tag, "demos": a.demos, "unified": CG.UNIFIED, "place_settle": CG.PLACE_SETTLE, "tip_roll_stop": CG.TIP_ROLL_STOP, "replay_only": REPLAY_ONLY, "boards": res},
                                                ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    rc = out / "results16.csv"
    if not rc.exists():
        with rc.open("w", encoding="utf-8-sig", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(FIELDS)
            for r in res:
                w.writerow([r["board"], r["label"], "학습" if r["trained"] else "새 자리", f"{r['start_disp'][0]:g} · {r['start_disp'][1]:g}",
                            f"{r['goal_disp'][0]:g} · {r['goal_disp'][1]:g}", "shift", r["demo"], r["log"], "", "", "", "", "", ""])
    L_ = ["# 실기 실행표 19-x16 — 위쪽 잡기 shift (가장 가까운 top 시연을 그 좌표로 옮겨 재생, 학습 정책 아님)", "",
          "- 캔 놓는 자리 = 판 기준 cm: **로봇 쪽 모서리에서 앞으로 · 판 왼쪽 끝 (로봇 왼쪽) 에서 오른쪽으로**. 로봇 좌표로는 x = 모서리 + 7, y = 30.25 − 왼쪽 끝.",
          "- **판 1 · 2 는 손을 비상정지 위에 두고** 시작합니다. 이상하면 바로 누르고, 그 판은 '멈춤' 으로 적습니다.",
          "- 판마다 적을 것: 잡기 (L 이 캔을 들었나) · 인계 (R 이 받았나) · 놓기 (목적지에 섰나) 성공 / 실패, 목적지 오차 (캔 가운데 ↔ 목적지 표시, 앞뒤 · 좌우 cm). `results16.csv` 칸에 적습니다.",
          "- 명령에는 `--handover-depth-cm` 이 없습니다 (아래쪽 잡기용 규칙). 다른 보정은 real19x 와 같습니다.", ""]
    for r in res:
        L_ += [f"## 판 {r['board']} — {r['label']} · {'학습에 쓴 자리' if r['trained'] else '학습에 안 쓴 자리'}", "- " + CG.RUN_STEP,
               f"- 캔: 모서리에서 **{r['start_disp'][0]:g}** · 왼쪽 끝에서 **{r['start_disp'][1]:g}** (로봇 {r['start'][0]:g}, {r['start'][1]:g}) → "
               f"목적지: 모서리에서 **{r['goal_disp'][0]:g}** · 왼쪽 끝에서 **{r['goal_disp'][1]:g}** (로봇 {r['goal'][0]:g}, {r['goal'][1]:g})",
               f"- shift 시연: `{r['demo']}` (시작 {r['demo_dist_start_cm']:g} · 도착 {r['demo_dist_goal_cm']:g} cm 옮김)"]
        if not r["ok"]:
            L_ += [f"- 🔴 shift 를 못 만듦 — {r['why']} (건너뜀)", ""]
            continue
        L_ += [f"- 시뮬 shift: 평가 {r['sim_eval']} · 이 재생 파일 {r['sim_stage']} (목적지 {r['sim_goal_cm']} cm, 인계 기울기 {r['sim_tilt'] or '-'}° · 깊이 {r['sim_depth'] or '-'} cm"
               f"{', 실물 기준 ' + (r['sim_real_flags'] or '') + ' 실패' if r['sim_real_fail'] == '1' else ''})", "```", r["cmd"], "```"]
        if r["checks_block"]:
            L_.append("- 🔴 점검 막힘: " + " / ".join(r["checks_block"]))
        if r["checks_warn"]:
            L_.append("- 🟡 점검 경고: " + " / ".join(r["checks_warn"]))
        for fl, txt in r.get("flag_text", []):
            L_.append(f"- 🟡 shift 검사 ({fl}): {txt}")
        L_.append("")
    (out / "runbook16.md").write_text("\n".join(L_), encoding="utf-8")
    print(f"판 {len(res)} · shift {sum(r['ok'] for r in res)} → {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
