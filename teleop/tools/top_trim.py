#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""위쪽 잡기 시연 인계 구간 다듬기 (작업 지시 19-x16). ~/venvs/learn 에서 돌린다. 로봇 · CAN · 시리얼은 쓰지 않는다.

19-x15: top ×5 정책은 R 이 캔 1.7 cm 앞까지 와서 닫지 않는다. top 시연은 R 다시 닫기 6 / 24 판 (19-j2 0 / 21), L 잡음 → 마지막 R 닫기 중앙 8.9 초 (19-j2 4.4),
마지막 닫기 전 닫는 자리 2 cm 안에서 R 이 열린 채 머문 시간 중앙 4.4 초 (19-j2 1.7). 19-j2 의 handover_swap make2 는 아래쪽 잡기 실기 궤적 · 깊이 규칙이 기준이라 top 에 쓰지 않는다.
행동 (learn_data.arrays 의 action, 관절 10 + 그리퍼 2, 20 Hz) 만 고치고, 원본 재생 (aug_demo.try_one 옮김 없음 = 시뮬 · 엄격 · 실물 기준) 으로 확인한다.
  A  놓친 R 닫기 자르기: L 이 든 동안 마지막 R 닫기 (aug_demo.segments 의 iRc) 앞의 R 닫기 = 놓친 닫기. 첫 놓친 닫기 프레임 c0 부터,
     마지막으로 R 을 연 뒤 ~ iRc 사이에서 c0 직전 관절 자세와 가장 가까운 프레임 c1 까지를 잘라 낸다. 이음매는 관절 차를 W 프레임 (0.5 → 2 초) 에 걸쳐 smoothstep 으로 줄인다
     (관절 속도가 원본 최대를 넘지 않는 가장 짧은 W). 그리퍼는 잘린 뒤 그대로 (L 쥠 · R 열림 → iRc 에 닫음).
  B  A 뒤, 접근 구간 [L 닫기 + 0.6 초, 마지막 R 닫기 − 0.3 초) 를 시간만 당긴다 (경로 그대로). 프레임마다 걸리는 시간 = max(dt / c, 관절 변화 / 원본 최대 속도) 로
     관절 속도가 원본 최대를 넘지 않고, c (최대 6) 는 'L 닫기 → 마지막 R 닫기' 가 4.2 초 + 그리퍼 닫힘 0.57 초 (19-j2 중앙) 가 되게 고른다. 이미 짧으면 그대로.
    top_trim.py plan --files 목록.txt --mode B --out DIR      CPU 만: 자를 · 당길 프레임 · 새 시간 · 속도 → DIR/trim_plan.json, DIR/act/<이름>.npy
    top_trim.py make --files 목록.txt --mode B --out DIR      plan + 원본 재생 (시뮬) → 통과 판 npz (학습 자료 꼴) · DIR/files_trim.txt
      --grasp-mode top --grasp-real 이 기본 (top19x.sh 증강 · 평가와 같은 가상 캔)."""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import sys
import time
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import learn_data as L  # noqa: E402
import policy_sim as PS  # noqa: E402
import aug_demo as AD  # noqa: E402

FPS = L.FPS
DT = 1.0 / FPS
GAP_TARGET_S = 4.2          # 19-j2 시연 L 잡음 → R 닫기 중앙 (19-x15 · 19-x16 측정 4.2 ~ 4.4)
GRIP_CLOSE_S = 0.57         # 가상 그리퍼 real 타이밍: 닫기 명령 → 다 닫힘 (19-x12)
PRE_S, POST_S = 0.6, 0.3    # B 가 건드리지 않는 L 닫기 뒤 · R 닫기 앞 구간
C_MAX = 6.0
W_STEPS = (10, 15, 20, 30, 40)


def speed(act: np.ndarray) -> float:
    return float(np.degrees(np.abs(np.diff(act[:, :10], axis=0)).max()) * FPS) if len(act) > 1 else 0.0


def ups(c: np.ndarray) -> np.ndarray:
    return np.where(c[1:] & ~c[:-1])[0] + 1


def dns(c: np.ndarray) -> np.ndarray:
    return np.where(~c[1:] & c[:-1])[0] + 1


def trim_A(act: np.ndarray, vmax: float):
    """→ (새 행동, 정보). 놓친 닫기가 없으면 그대로."""
    seg = AD.segments(act)
    if seg is None:
        return None, {"why": "그리퍼 차례가 아님"}
    iLc, iRc, iLo, iRo = seg
    gR = act[:, 11] > 0.5
    uR = ups(gR); uR = uR[(uR > iLc) & (uR < iRc)]
    if not len(uR):
        return act.copy(), {"missed": 0, "cut": None}
    c0 = int(uR[0])
    dR = dns(gR); dR = dR[(dR > c0) & (dR < iRc)]
    if not len(dR):
        return None, {"why": "놓친 닫기 뒤 R 을 연 프레임이 없음"}
    lo = int(dR[-1])
    ref = act[c0 - 1, :10]
    c1 = lo + int(np.argmin(np.abs(act[lo:iRc, :10] - ref).max(axis=1)))
    jump = float(np.degrees(np.abs(act[c1, :10] - ref).max()))
    for W in W_STEPS:
        new = np.concatenate([act[:c0], act[c1:]], 0).copy()
        d = ref - act[c1, :10]
        for k in range(min(W, len(new) - c0)):
            new[c0 + k, :10] += d * (1.0 - AD.smooth01((k + 1) / W))
        if speed(new) <= vmax + 1e-6:
            return new, {"missed": int(len(uR)), "cut": [c0, c1], "cut_s": round((c1 - c0) * DT, 2), "jump_deg": round(jump, 2), "blend_frames": W}
    return None, {"why": f"이음매 속도가 원본 최대 ({vmax:.0f}°/s) 를 넘음 (W {W_STEPS[-1]} 프레임까지, 관절 차 {jump:.1f}°)", "missed": int(len(uR)), "cut": [c0, c1]}


def warp(q: np.ndarray, vmax: float, c: float) -> np.ndarray:
    """q (n+1, 10) 경로를 시간만 당겨 20 Hz 로 다시 샘플 (끝점 q[n] 은 빼고 돌려준다 — 다음 구간 첫 프레임)."""
    step = np.degrees(np.abs(np.diff(q, axis=0)).max(axis=1))
    dur = np.maximum(DT / c, step / vmax)
    tau = np.concatenate([[0.0], np.cumsum(dur)])
    n_new = max(1, int(np.floor(tau[-1] / DT + 1e-9)))
    ts = np.arange(n_new) * DT
    return np.stack([np.interp(ts, tau, q[:, j]) for j in range(q.shape[1])], 1), float(tau[-1])


def trim_B(act: np.ndarray, vmax: float):
    seg = AD.segments(act)
    if seg is None:
        return None, {"why": "그리퍼 차례가 아님"}
    iLc, iRc, iLo, iRo = seg
    a0, a1 = iLc + int(round(PRE_S * FPS)), iRc - int(round(POST_S * FPS))
    gap0 = (iRc - iLc) * DT
    info = {"gap_cmd_s": round(gap0, 2), "window": [int(a0), int(a1)]}
    if a1 - a0 < 2 or gap0 <= GAP_TARGET_S + GRIP_CLOSE_S:
        return act.copy(), {**info, "c": 1.0, "gap_new_s": round(gap0, 2)}
    g = act[a0:a1 + 1, 10:12] > 0.5
    if not (g[:, 0].all() and not g[:, 1].any()):
        return None, {**info, "why": "접근 구간에 그리퍼 바뀜이 있음"}
    want = (a1 - a0) * DT - (gap0 - GAP_TARGET_S - GRIP_CLOSE_S)
    lo, hi = 1.0, C_MAX
    _, d_hi = warp(act[a0:a1 + 1, :10], vmax, hi)
    c = hi if d_hi >= want else None
    if c is None:
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            _, d = warp(act[a0:a1 + 1, :10], vmax, mid)
            lo, hi = (mid, hi) if d > want else (lo, mid)
        c = hi
    qw, dw = warp(act[a0:a1 + 1, :10], vmax, c)
    mid = np.concatenate([qw, np.repeat(act[a0:a0 + 1, 10:12], len(qw), 0)], 1)
    new = np.concatenate([act[:a0], mid, act[a1:]], 0)
    gap1 = (iRc - iLc - (a1 - a0) + len(qw)) * DT
    return new, {**info, "c": round(c, 3), "window_s": [round((a1 - a0) * DT, 2), round(len(qw) * DT, 2)], "gap_new_s": round(gap1, 2),
                 "speed_new": round(speed(new), 1)}


def plan_one(path: str, mode: str):
    arr = L.arrays(path)
    m = arr["meta"]
    name = f"{m['record']}_ep{int(m['episode']):02d}"
    act = np.asarray(arr["action"], float)
    vmax = speed(act)
    r = {"name": name, "src": path, "frames0": len(act), "dur0_s": round(len(act) * DT, 2), "speed0": round(vmax, 1)}
    new, ia = trim_A(act, vmax)
    r["A"] = ia
    if new is not None and mode == "B":
        new, ib = trim_B(new, vmax)
        r["B"] = ib
    if new is None:
        return r | {"ok": False, "why": (r.get("B") or {}).get("why") or ia.get("why")}, arr, None
    seg = AD.segments(new)
    r.update(ok=seg is not None, why="" if seg is not None else "다듬은 뒤 그리퍼 차례가 깨짐", frames=len(new), dur_s=round(len(new) * DT, 2),
             speed=round(speed(new), 1), gap_cmd_s=None if seg is None else round((seg[1] - seg[0]) * DT, 2),
             changed=bool(len(new) != len(act) or not np.allclose(new, act)))
    return r, arr, new


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=("plan", "make"))
    ap.add_argument("--files", required=True)
    ap.add_argument("--mode", choices=("A", "B"), default="B")
    ap.add_argument("--out", required=True)
    ap.add_argument("--tag", default=None, help="record 이름 trim<mode>_<원래 record> 의 앞 (기본 trim<mode>)")
    ap.add_argument("--grasp-mode", choices=("bottom", "top"), default="top")
    ap.add_argument("--no-grasp-real", action="store_true")
    ap.add_argument("--roll-stop", action="store_true", help="19-x20 (기본 끔): 원본 재생 시뮬 모터 명령의 어깨 roll 을 실물 멈춤 (워치독 ROLL_STOP_DEG) 으로 묶는다")
    a = ap.parse_args()
    out = Path(a.out).expanduser()
    (out / "act").mkdir(parents=True, exist_ok=True)
    tag = a.tag or f"trim{a.mode}"
    paths = [p.strip() for p in Path(a.files).expanduser().read_text(encoding="utf-8").splitlines() if p.strip() and not p.startswith("#")]
    rows, t0 = [], time.time()
    ctx = tmp = None
    if a.cmd == "make":
        PS.VCAN_EXTRA.update({**({"grasp_mode": a.grasp_mode} if a.grasp_mode != "bottom" else {}), **({} if a.no_grasp_real else {"real": True})})
        if a.roll_stop:                                  # 19-x20 (켤 때만)
            PS.ROLL_STOP = PS.roll_stop_watchdog()
        ctx = AD.Ctx()
        tmp = out / "_sim"
        tmp.mkdir(exist_ok=True)
    keep = []
    for n, p in enumerate(paths):
        r, arr, new = plan_one(p, a.mode)
        if new is not None:
            np.save(out / "act" / f"{r['name']}.npy", new.astype(np.float32))
        if a.cmd == "make" and r["ok"]:
            arr2 = dict(arr)
            arr2["action"] = new.astype(np.float32)
            arr2["t"] = float(arr["t"][0]) + np.arange(len(new)) * DT
            src = AD.demo_xy(arr["meta"])
            rr = AD.try_one(ctx, arr2, src, src, f"{tag}_{r['name']}", tmp)
            r["replay"] = {"ok": rr["ok"], "why": rr["why"], "sim": rr.get("sim")}
            if rr["ok"]:
                z = np.load(tmp / f"{tag}_{r['name']}.npz", allow_pickle=True)
                meta = json.loads(str(z["meta"]))
                m0 = arr["meta"]
                meta.update({"record": f"{tag}_{m0['record']}", "episode": m0["episode"], "success": True, "result": "auto_success", "options": m0.get("options"),
                             "quality": {"policy_rollout": False, "no_release": False, "aug": False, "trim": True,
                                         "strict": {"fail": False, "flags": ((rr.get("sim") or {}).get("strict_flags") or "").split()}},
                             "trim": {"mode": a.mode, "src_file": p, "src_record": m0["record"], "src_episode": m0["episode"],
                                      "A": r.get("A"), "B": r.get("B"), "sim": rr.get("sim")}})
                f = out / f"{tag}_{r['name']}.npz"
                np.savez_compressed(f, meta=np.array(json.dumps(meta, ensure_ascii=False, default=float)), **{k: z[k] for k in z.files if k != "meta"})
                r["file"] = str(f)
                keep.append(str(f))
        rows.append(r)
        b = r.get("B") or {}
        print(f"[{n + 1}/{len(paths)}] {r['name']}: {'ok' if r['ok'] else '실패 — ' + r['why']} | 놓친 닫기 {r['A'].get('missed')} 잘림 {r['A'].get('cut_s')} s · "
              f"이음 {r['A'].get('blend_frames')} | L→R 닫기 {b.get('gap_cmd_s', r.get('gap_cmd_s'))} → {r.get('gap_cmd_s')} s (c {b.get('c')}) | "
              f"길이 {r['dur0_s']} → {r.get('dur_s')} s · 속도 {r['speed0']} → {r.get('speed')}°/s"
              + (f" | 원본 재생 {'통과' if r['replay']['ok'] else '실패 ' + r['replay']['why']}" if "replay" in r else "") + f" — {round(time.time() - t0)} 초", flush=True)
    if tmp is not None:
        for q in tmp.glob("*"):
            q.unlink()
        tmp.rmdir()
    summ = {"cmd": a.cmd, "mode": a.mode, "files": a.files, "gap_target_s": GAP_TARGET_S, "rows": rows}
    (out / ("trim_plan.json" if a.cmd == "plan" else "trim_make.json")).write_text(json.dumps(summ, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    if a.cmd == "make":
        (out / "files_trim.txt").write_text("".join(f + "\n" for f in keep), encoding="utf-8")
        print(f"원본 재생 통과 {len(keep)} / {len(rows)} → {out / 'files_trim.txt'}")
    else:
        print(f"다듬기 가능 {sum(r['ok'] for r in rows)} / {len(rows)} → {out / 'trim_plan.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
