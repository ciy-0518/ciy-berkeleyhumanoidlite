#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""위쪽 잡기 (top) 정책 실기 실행표 (작업 지시 19-x23). 판마다:
  1. 명령 한 줄: ~/run.sh policy <체크포인트> --can-start · --can-goal + 실기 보정 사슬 + 정책 옵션 + 규칙
     (--grasp-assist L 잡기 보정, --handover-assist R 보정 · L 여는 안전) + --policy-server-args "--first-from <목록>" (다듬은 기록 정책의 시작 자세).
     사슬 = can_go.UNIFIED 에서 끝 오프셋만 --offset-cm 로 + --tip-roll-stop + --place-sag-elbow-scale (--sag-scale) + can_go.PLACE_SETTLE + can_go.POLICY_ONLY.
     --handover-depth-cm 은 넣지 않는다 (아래쪽 잡기 규칙, R 보정과 같은 R 깊이를 다르게 고친다 — top_shift_prep 과 같음).
  2. 점검 (실행 안 함): can_go.run_checks(mode=top) 막힘 · 경고, 명령 인자를 replay_run.parse 로 읽어 봄 (들어가야 할 옵션 · 빠져야 할 옵션).
  3. 시뮬 참고 칸: eval_pairs results.json (--sim, 그 판 좌표의 x5 결과) 의 목적지 5 cm · 1.5 cm 성공 수.
출력 (OUT): runbook<NN>.md, plan<NN>.json, results<NN>.csv (NN = --suffix, 기본 23; results 는 있으면 새로 쓰지 않음).
19-x24 (모두 기본 = 19-x23 그대로): --offset-left-cm (L 만 다른 끝 오프셋), --can-first (캔을 먼저 놓고 시작하는 순서 줄), --suffix.
19-x30 (기본 끔): --tip-cap-deg X (사슬의 --tip-cap-deg 18 을 X 로), --grasp-held-on-grasp (명령에 --grasp-assist-held-on-grasp).
19-x29 (기본 끔): --grasp-release-on-grasp → 사슬 E 에 replay_run --grasp-tip-release-on-grasp (캔을 잡는 닫힘에서만 잡기 오프셋을 뺌).
19-x26 (기본 끔): --end-after-held → 명령에 replay_run --policy-end-after-held (캔을 쥔 뒤의 놓음만 셈).
19-x25 (기본 끔): --grasp-extra-left-cm X → 사슬에 --grasp-tip-offset-cm L X 0 0 --grasp-tip-skip-start-grip (L 이 캔을 쥘 때까지만 X 더함, 사슬 E).
  top_policy_prep.py --boards ~/cc_analysis/top19x15/boards15.json --policy <pretrained_model> --first-from <files_trim.txt> --sim <results.json> --out ~/cc_analysis/real19x23"""
from __future__ import annotations

import argparse
import contextlib
import csv
import io
import json
import shlex
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
sys.path.insert(1, str(Path.home() / "code_workspace/Berkeley-Humanoid-Lite-main/Berkeley-Humanoid-Lite-Lowlevel"))
with contextlib.redirect_stdout(io.StringIO()):
    import can_go as CG  # noqa: E402

HOME = Path.home()
CAN_FIRST = ("**순서: 캔을 십자선에 먼저 놓기 → 명령 실행 → 첫 자세 도착 뒤 15 초 기다림 (그동안 캔을 건드리지 않음).** "
             "캔은 먼저 놓고 시작합니다 (19-x24 사용자 결정, 19-x23 판 1 · 2 와 같은 순서). 첫 자세로 올라가는 팔이 캔에 가까우면 비상정지")
FIELDS = ["판", "쌍", "자리", "캔 (모서리 · 왼쪽 끝)", "목적지 (모서리 · 왼쪽 끝)", "방법", "로그", "시뮬 5 cm", "시뮬 1.5 cm",
          "잡기", "인계", "놓기", "목적지_앞뒤_cm", "목적지_좌우_cm", "L보정_옮긴_cm", "R보정_옮긴_cm", "메모"]


def tilde(p) -> str:
    return str(Path(p).expanduser()).replace(str(HOME), "~")


def chain_text(offset_cm: float, sag_scale, offset_left_cm=None, grasp_extra_left_cm=None, release_on_grasp=False) -> str:
    ol = offset_cm if offset_left_cm is None else offset_left_cm
    base = CG.UNIFIED.replace("--tip-offset-cm L 3 0 0 R 3 0 0", f"--tip-offset-cm L {ol:g} 0 0 R {offset_cm:g} 0 0")
    if "--tip-offset-cm L" not in base:
        raise SystemExit("can_go.UNIFIED 의 끝 오프셋 꼴이 바뀌었다 — 고치지 않고 멈춘다")
    out = f"{base} {CG.TIP_ROLL_STOP} {CG.PLACE_SETTLE}"
    if grasp_extra_left_cm is not None:              # 19-x25 사슬 E
        out += f" --grasp-tip-offset-cm L {grasp_extra_left_cm:g} 0 0 --grasp-tip-skip-start-grip" + (" --grasp-tip-release-on-grasp" if release_on_grasp else "")
    if sag_scale is not None:
        out += f" --place-sag-elbow-scale {sag_scale:g}"
    return out


def sim_ref(path):
    if not path:
        return {}
    d = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    out = {}
    for r in (d["rows"] if isinstance(d, dict) else d):
        if r["method"] == "shift":
            continue
        k = tuple(float(v) for v in (*r["start"], *r["goal"]))
        o = out.setdefault(k, [0, 0, 0])
        o[0] += int(bool(r.get("ok5"))); o[1] += int(bool(r.get("ok15"))); o[2] += 1
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--boards", required=True)
    ap.add_argument("--policy", required=True, help="pretrained_model 폴더")
    ap.add_argument("--first-from", required=True, help="정책 시작 자세 목록 (다듬은 시연 files_trim.txt)")
    ap.add_argument("--grasp-assist", default=str(TOOLS.parent / "config/grasp_assist_top_19x22.json"))
    ap.add_argument("--handover-assist", default=str(TOOLS.parent / "config/handover_assist_top_19x21.json"))
    ap.add_argument("--offset-cm", type=float, default=1.5, help="끝 오프셋 L · R (cm, 19-x23 사슬 C)")
    ap.add_argument("--offset-left-cm", type=float, default=None, help="19-x24: L 끝 오프셋만 따로 (cm, 기본 = --offset-cm)")
    ap.add_argument("--grasp-extra-left-cm", type=float, default=None, help="19-x25: L 이 캔을 쥘 때까지만 더할 끝 오프셋 (cm, 기본 없음). 사슬 E = --offset-cm 1.5 --grasp-extra-left-cm 1.5")
    ap.add_argument("--tip-cap-deg", type=float, default=None, help="19-x30: 사슬의 끝 오프셋 관절 상한 (도, 기본 = can_go.UNIFIED 의 18)")
    ap.add_argument("--grasp-held-on-grasp", action="store_true", help="19-x30: 명령에 --grasp-assist-held-on-grasp 를 넣는다 (기본 없음)")
    ap.add_argument("--grasp-release-on-grasp", action="store_true", help="19-x29: --grasp-extra-left-cm 과 같이, 명령에 --grasp-tip-release-on-grasp 를 넣는다 (기본 없음)")
    ap.add_argument("--end-after-held", action="store_true", help="19-x26: 명령에 --policy-end-after-held 를 넣는다 (기본 없음)")
    ap.add_argument("--can-first", action="store_true", help="19-x24: 판마다 순서 줄을 '캔을 먼저 놓고 시작' 으로 (기본 = can_go.RUN_STEP)")
    ap.add_argument("--suffix", default="23", help="19-x24: 출력 파일 이름 번호 (runbook<NN>.md 등, 기본 23)")
    ap.add_argument("--sag-scale", type=float, default=0.0, help="--place-sag-elbow-scale (19-x23: 0). 음수면 넣지 않음")
    ap.add_argument("--sim", default=None, help="eval_pairs results.json (그 판 좌표의 정책 결과)")
    ap.add_argument("--tag", default="real19x23")
    ap.add_argument("--out", default=str(HOME / "cc_analysis/real19x23"))
    a = ap.parse_args()
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    boards = json.loads(Path(a.boards).expanduser().read_text(encoding="utf-8"))
    sag = None if a.sag_scale < 0 else a.sag_scale
    chain = chain_text(a.offset_cm, sag, a.offset_left_cm, a.grasp_extra_left_cm, a.grasp_release_on_grasp)
    if a.tip_cap_deg is not None:                       # 19-x30
        if chain.count("--tip-cap-deg 18 ") != 1:
            raise SystemExit("사슬의 --tip-cap-deg 18 꼴이 바뀌었다 — 고치지 않고 멈춘다")
        chain = chain.replace("--tip-cap-deg 18 ", f"--tip-cap-deg {a.tip_cap_deg:g} ")
    if a.grasp_held_on_grasp:
        chain += " --grasp-assist-held-on-grasp"
    ol = a.offset_cm if a.offset_left_cm is None else a.offset_left_cm
    step = CAN_FIRST if a.can_first else CG.RUN_STEP
    ref = sim_ref(a.sim)
    ck = CG.Checker(Path(CG.DEMOS))
    demos_top, _ = ck._load_demos(CG.DEMOS_TOP)
    with contextlib.redirect_stdout(io.StringIO()):
        import replay_run as RR
    res = []
    for i, b in enumerate(boards, 1):
        st, go = tuple(float(v) for v in b["start"]), tuple(float(v) for v in b["goal"])
        log = f"{a.tag}_b{i:02d}_policy"
        cmd = (f"~/run.sh policy {tilde(a.policy)} --can-start {CG.fmt(st[0])},{CG.fmt(st[1])} --can-goal {CG.fmt(go[0])},{CG.fmt(go[1])} "
               f"{chain} {CG.POLICY_ONLY} --policy-end-skip-start-grip {'--policy-end-after-held ' if a.end_after_held else ''}--grasp-assist {tilde(a.grasp_assist)} --handover-assist {tilde(a.handover_assist)} "
               f"--policy-server-args \"--first-from {tilde(a.first_from)}\" --log-csv ~/teleop-docs/_teleop/logs/{log}.csv")
        tb, tw, _ = CG.run_checks(ck, st, go, mode="top", demos=demos_top)
        argv = [CG.os.path.expanduser(w) if w.startswith("~/") else w for w in shlex.split(cmd)]
        rr_argv = ["--policy", argv[2]] + argv[3:]                      # run.sh policy <ck> 뒤 = replay_run --policy <ck> 인자
        old = sys.argv
        why = []
        try:
            sys.argv = ["replay_run.py"] + rr_argv
            pa = RR.parse()
            if pa.handover_depth_cm is not None:
                why.append("들어가면 안 되는 옵션: --handover-depth-cm")
            for k, want in (("tip_roll_stop", True), ("grasp_assist", True), ("handover_assist", True), ("policy_end_skip_start_grip", True),
                            *((("grasp_tip_offset_cm", True), ("grasp_tip_skip_start_grip", True)) if a.grasp_extra_left_cm is not None else ()),
                            *((("policy_end_after_held", True),) if a.end_after_held else ()),
                            *((("grasp_tip_release_on_grasp", True),) if a.grasp_release_on_grasp and a.grasp_extra_left_cm is not None else ()),
                            *((("grasp_assist_held_on_grasp", True),) if a.grasp_held_on_grasp else ())):
                if bool(getattr(pa, k, None)) != want:
                    why.append(f"빠진 옵션: {k}")
            if "--first-from" not in (pa.policy_server_args or ""):
                why.append("빠진 옵션: --policy-server-args --first-from")
        except SystemExit as e:
            why.append(f"인자 해석 실패 ({e.code})")
        finally:
            sys.argv = old
        sm = ref.get((*st, *go))
        r = {"board": i, "label": b["label"], "trained": bool(b["trained"]), "start": list(st), "goal": list(go), "start_disp": CG.disp(*st), "goal_disp": CG.disp(*go),
             "log": log, "cmd": cmd, "rr_argv": rr_argv, "checks_block": tb, "checks_warn": tw, "parse_ok": not why, "parse_why": "; ".join(why),
             "sim5": f"{sm[0]}/{sm[2]}" if sm else "-", "sim15": f"{sm[1]}/{sm[2]}" if sm else "-"}
        res.append(r)
        print(f"판 {i} {b['label']}: 시뮬 5 cm {r['sim5']} · 1.5 cm {r['sim15']} | 점검 막힘 {len(tb)} 경고 {len(tw)} | 인자 {'ok' if r['parse_ok'] else r['parse_why']}", flush=True)
    (out / f"plan{a.suffix}.json").write_text(json.dumps({"tag": a.tag, "policy": a.policy, "first_from": a.first_from, "chain": chain, "policy_only": CG.POLICY_ONLY,
                                                 "grasp_assist": a.grasp_assist, "handover_assist": a.handover_assist, "boards": res},
                                                ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    rc = out / f"results{a.suffix}.csv"
    if not rc.exists():
        with rc.open("w", encoding="utf-8-sig", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(FIELDS)
            for r in res:
                w.writerow([r["board"], r["label"], "학습" if r["trained"] else "새 자리", f"{r['start_disp'][0]:g} · {r['start_disp'][1]:g}",
                            f"{r['goal_disp'][0]:g} · {r['goal_disp'][1]:g}", "정책 + 규칙", r["log"], r["sim5"], r["sim15"]] + [""] * 8)
    L_ = [f"# 실기 실행표 19-x{a.suffix} — 위쪽 잡기 정책 + 규칙 ({Path(a.policy).parent.parent.parent.name})", "",
          "- 캔 놓는 자리 = 판 기준 cm: **로봇 쪽 모서리에서 앞으로 · 판 왼쪽 끝 (로봇 왼쪽) 에서 오른쪽으로**. 로봇 좌표로는 x = 모서리 + 7, y = 30.25 − 왼쪽 끝.",
          "- **판 1 · 2 는 손을 비상정지 위에 두고** 시작합니다. 이상하면 바로 누르고, 그 판은 '멈춤' 으로 적습니다.",
          "- 판마다 적을 것: 잡기 (L 이 캔을 들었나) · 인계 (R 이 받았나) · 놓기 (목적지에 섰나) 성공 / 실패, 목적지 오차 (캔 가운데 ↔ 목적지 표시, 앞뒤 · 좌우 cm). "
          f"`results{a.suffix}.csv` 칸에 적습니다. 규칙이 옮긴 양 (L · R) 은 로그에서 나중에 채웁니다.",
          (f"- 사슬: 끝 오프셋 L · R {a.offset_cm:g} cm" if ol == a.offset_cm else f"- 사슬: 끝 오프셋 L {ol:g} cm · R {a.offset_cm:g} cm") + (f" (L 이 캔을 쥘 때까지는 L 에 {a.grasp_extra_left_cm:g} cm 더 — `--grasp-tip-offset-cm` + `--grasp-tip-skip-start-grip`, 쥔 뒤 0.5 초에 걸쳐 뺌)" if a.grasp_extra_left_cm is not None else "")
          + " + `--tip-roll-stop` (어깨 roll 실물 멈춤 안에서 오프셋)"
          + (f" + `--place-sag-elbow-scale {sag:g}` (R 만 쥔 동안 R 팔꿈치 처짐 보정 {sag:g} 배)" if sag is not None else "") + ". `--handover-depth-cm` 은 넣지 않았습니다.",
          "- 정책 끝: `--policy-end-after-release 2` 에 `--policy-end-skip-start-grip` 을 같이 넣었습니다 (시작 자세의 닫힌 그리퍼를 쥔 것으로 치지 않음 — 없으면 캔을 잡기 전에 정책이 끝남, 19-x23 dry-run).",
          "- 규칙: `--grasp-assist` (L 잡기 보정 — 캔 자리는 `--can-start` 입력 좌표), `--handover-assist` (R 보정 + L 여는 안전 — R 이 기준 안에서 쥐어야 L 을 엶).", ""]
    for r in res:
        L_ += [f"## 판 {r['board']} — {r['label']} · {'학습에 쓴 자리' if r['trained'] else '학습에 안 쓴 자리'}", "- " + step]
        if r["board"] in (1, 2):
            L_.append("- **손을 비상정지 위에 두고** 시작합니다.")
        L_ += [f"- 캔: 모서리에서 **{r['start_disp'][0]:g}** · 왼쪽 끝에서 **{r['start_disp'][1]:g}** (로봇 {r['start'][0]:g}, {r['start'][1]:g}) → "
               f"목적지: 모서리에서 **{r['goal_disp'][0]:g}** · 왼쪽 끝에서 **{r['goal_disp'][1]:g}** (로봇 {r['goal'][0]:g}, {r['goal'][1]:g})",
               f"- 시뮬 (정책 + 규칙, 이 좌표 3 번): 목적지 5 cm {r['sim5']} · 1.5 cm {r['sim15']}", "```", r["cmd"], "```"]
        if r["checks_block"]:
            L_.append("- 🔴 점검 막힘: " + " / ".join(r["checks_block"]))
        if r["checks_warn"]:
            L_.append("- 🟡 점검 경고: " + " / ".join(r["checks_warn"]))
        if not r["parse_ok"]:
            L_.append("- 🔴 명령 인자: " + r["parse_why"])
        L_.append("")
    (out / f"runbook{a.suffix}.md").write_text("\n".join(L_), encoding="utf-8")
    print(f"판 {len(res)} → {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
