#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""실기 평가 결과 기록표 채우기 (작업 지시 19-d). 판이 끝난 뒤 로그로 채울 수 있는 칸만 채운다 (사용자 칸은 그대로).
  로그: <로그 폴더>/<로그 이름>*.csv 중 가장 새 것 (다시 한 판은 이름 뒤에 _2 처럼 붙이면 그것을 쓴다).
  사건 시각: 재생 · 정책 시작 (단계 play 첫 행) 부터, 그리퍼 명령 (grip_L · grip_R) 이 바뀐 때. L · R 닫힘 = 처음 0 → 1, 열림 = 마지막 1 → 0.
  실행_s: play · grip_wait 단계 길이 (정책 시간 / 재생 시간).  끝_까닭: 정책 멈춤 까닭 (pol_stop) 또는 "재생 끝".
  놓은 위치: R 이 여는 직전 행의 R 집게 자리 (virtual_can.hand_of, 가상 캔 판정과 같은 점) − 목적지 (cm, 앞뒤 + = 로봇 반대쪽, 좌우 + = 로봇 왼쪽).
    엔코더 = meas 열 (실제 관절 — 보정이 든 자세. dry-run 처럼 엔코더가 없으면 빈칸),
    논리 = cmd + sag 열 (처짐 보정 · 집게 끝 보정 전 목표 = 정책 · 재생 궤적이 뜻한 자리). 높이 = 그 점 − 목적지 판 면.
    real_eval_fill.py ~/cc_analysis/real19d/plan.json [--csv results.csv] [--log-dir ~/teleop-docs/_teleop/logs]"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
TELEOP = TOOLS.parent
LOWLEVEL = Path.home() / "code_workspace/Berkeley-Humanoid-Lite-main/Berkeley-Humanoid-Lite-Lowlevel"
for p in (str(TOOLS), str(LOWLEVEL)):
    if p not in sys.path:
        sys.path.insert(0, p)
import vr_control_watchdog_biarm as W  # noqa: E402
import virtual_can as VC  # noqa: E402
import table_limit as TL  # noqa: E402
import real_eval_prep as RP  # noqa: E402

SIDES = ("left", "right")


def fnum(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def r2(v):
    return "" if v is None else f"{v:.2f}"


class Fk:
    def __init__(self):
        W.PR.set_hand_model("cad")
        W.apply_yaw_in_limit(100.0, 100.0)
        ref_path = W.newest_robot_ref(None)
        self.ref = W.PR.RobotRef.load(ref_path)
        self.table = TL.TableLimit(TELEOP / "config/table.json", body="real")
        self.vc = VC.VirtualCan(self.table, "L1", self.ref, SIDES, W.motor_to_urdf_fn(self.ref, SIDES), log=lambda s: None)
        self.enc_ok = True
        self.slots = {s["name"]: s["base_xy_cm"] for s in self.table.cfg["can_slots"]}

    def point(self, row, kind):
        q = {}
        for j in W.JOINT_ORDER:
            if kind == "enc":
                v = fnum(row.get(f"right_{j}_meas")) if self.enc_ok else None
            else:
                c, s = fnum(row.get(f"right_{j}_cmd")), fnum(row.get(f"right_{j}_sag")) or 0.0
                v = None if c is None else c + s
            if v is None:
                return None
            q[j] = v
        return self.vc.hand_of("right", q)[0]


def analyse(path: Path, goal_xy, fk: Fk) -> dict:
    R = list(csv.DictReader(path.open(encoding="utf-8", errors="ignore")))
    run = [r for r in R if r.get("auto_stage") in ("play", "grip_wait")]
    out = {"로그_파일": path.name}
    if not run:
        out["끝_까닭"] = "재생 · 정책 단계가 없음 (" + (R[-1].get("auto_stage", "") if R else "빈 로그") + ")"
        return out
    fk.enc_ok = len({r.get("right_elbow_pitch_meas") for r in run}) > 3       # dry-run: 엔코더 열이 한 값 (0)
    t0 = float(run[0]["t"])
    out["실행_s"] = f"{float(run[-1]['t']) - t0:.1f}"
    ev = []
    for i in range(1, len(R)):
        for h in ("L", "R"):
            a, b = R[i - 1].get(f"grip_{h}"), R[i].get(f"grip_{h}")
            if a in ("0", "1") and b in ("0", "1") and a != b:
                ev.append((float(R[i]["t"]) - t0, h, "c" if b == "1" else "o", i))
    first = lambda h, k: next((e for e in ev if e[1] == h and e[2] == k), None)    # noqa: E731
    last = lambda h, k: next((e for e in reversed(ev) if e[1] == h and e[2] == k), None)    # noqa: E731
    for col, e in (("L_닫힘_s", first("L", "c")), ("R_닫힘_s", first("R", "c")), ("L_열림_s", last("L", "o")), ("R_열림_s", last("R", "o"))):
        out[col] = "" if e is None else f"{e[0]:.1f}"
    out["그리퍼_순서"] = " ".join(f"{h}{k}" for _, h, k, _ in ev)
    stops = [r.get("pol_stop") for r in R if r.get("pol_stop")]
    is_pol = any(r.get("pol_obs_t") for r in R)
    pend = [r.get("pol_end") for r in R if r.get("pol_end")]
    cans = [r.get("pol_can_state") for r in run if r.get("pol_can_state")]
    done = "정책 끝" + (f" ({pend[-1]})" if pend else (f" (가상 캔 {cans[-1]})" if cans else "")) if is_pol else "재생 끝"
    out["끝_까닭"] = stops[-1] if stops else (done if any(r.get("auto_stage") == "end_hold" for r in R) else "멈춤 (" + R[-1].get("auto_stage", "") + ")")
    ro = last("R", "o")
    if ro is not None and goal_xy is not None:
        row = R[ro[3] - 1]                                    # 열기 직전 행
        gz = fk.table.top(goal_xy[0] / 100.0) * 100.0
        for kind, pre in (("enc", "엔코더"), ("cmd", "논리")):
            p = fk.point(row, kind)
            out[f"{pre}_R집게_앞뒤_cm"] = r2(None if p is None else float(p[0] - goal_xy[0]))
            out[f"{pre}_R집게_좌우_cm"] = r2(None if p is None else float(p[1] - goal_xy[1]))
            if kind == "cmd" and p is not None:
                out["논리_R집게_높이_cm"] = r2(float(p[2] - gz))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("plan")
    ap.add_argument("--csv", default=None, help="기본: plan.json 옆 results.csv")
    ap.add_argument("--log-dir", default=str(TELEOP / "logs"))
    a = ap.parse_args()
    plan_p = Path(a.plan).expanduser()
    plan = json.loads(plan_p.read_text(encoding="utf-8"))
    csv_p = Path(a.csv).expanduser() if a.csv else plan_p.parent / "results.csv"
    goals = {b["log"]: b["goal"] for b in plan["boards"]}
    fk = Fk()
    goals.update({f["log"]: fk.slots.get(f["goal"]) for f in plan.get("fixed", [])})
    rows = list(csv.DictReader(csv_p.open(encoding="utf-8-sig")))
    fields = list(rows[0].keys()) if rows else RP.FIELDS_USER + RP.FIELDS_AUTO
    fields = [f for f in fields if f in RP.FIELDS_USER or f in RP.FIELDS_AUTO] + [f for f in RP.FIELDS_USER + RP.FIELDS_AUTO if f not in fields]
    rows = [{k: r.get(k, "") for k in fields} for r in rows]
    logdir = Path(a.log_dir).expanduser()
    n = 0
    for r in rows:
        name = (r.get("로그") or "").strip()
        if not name:
            continue
        hits = sorted(logdir.glob(f"{name}*.csv"), key=lambda p: p.stat().st_mtime)
        if not hits:
            continue
        r.update(analyse(hits[-1], goals.get(name), fk))
        n += 1
        print(f"판 {r.get('판') or '-':>2} {name}: 실행 {r.get('실행_s')} 초, {r.get('그리퍼_순서')}, {r.get('끝_까닭')}, "
              f"R − 목적지 엔코더 앞뒤 {r.get('엔코더_R집게_앞뒤_cm') or '-'} 좌우 {r.get('엔코더_R집게_좌우_cm') or '-'} · 논리 {r.get('논리_R집게_앞뒤_cm') or '-'} {r.get('논리_R집게_좌우_cm') or '-'} cm", flush=True)
    with csv_p.open("w", encoding="utf-8-sig", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    print(f"{n} 판 채움 → {csv_p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
