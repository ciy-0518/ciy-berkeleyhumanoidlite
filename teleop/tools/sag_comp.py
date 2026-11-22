#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""관절 처짐 모델과 보정 (작업 지시 14-l B). 재생기 (replay_run.py --sag-comp) 와 나중 정책 실행이 같이 쓰는 부품.

처짐 = 실제 − 명령 (모터 값, rad). 위치 모드 (kp 일정) 에서 처짐은 관절에 걸린 중력 토크에 비례한다고 본다.
  중력 토크: 팔 질량을 두 점 (팔꿈치 관절, 집게 끝) 에 모은다. 관절 i 축 a_i·원점 o_i 에서 점 p 의 질량이 만드는 토크 ∝ a_i · ((p − o_i) × (0, 0, −1)).
  관절마다: 처짐_i = k_el,i · τ_i(팔꿈치) + k_tip,i · τ_i(끝) + c_i   (팔마다 따로, 계수 3 개). 질량·강성·모터 방향 부호가 계수에 들어간다.
  τ 는 URDF 관절각 (motor_to_urdf_fn) 으로 잰다. 손목 (elbow_roll) 은 위치에 영향이 없어 0 으로 두지만, 손목 처짐도 같은 꼴로 맞춘다.
보정: 위치 4 관절만 (손목 뺌, COMP_JOINTS). 보낼 명령 c 에서 실제 ≈ c + 처짐(c). 실제가 목표 x 에 오게 c = x − 처짐(c) 를 두 번 되풀이로 푼다. 관절당 상한 (기본 10°).
모델 파일: config/sag_model.json (fit 으로 만든다). 맞추기: python3 sag_comp.py fit [--out ...] [--exclude 파일 ...]
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import sys
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

TOOLS = Path(__file__).resolve().parent
LOGS = TOOLS.parent / "logs"
MODEL_PATH = TOOLS.parent / "config/sag_model.json"
JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "elbow_roll")
CAP_DEG = 10.0             # 관절당 보정 상한
COMP_JOINTS = JOINTS[:4]   # 보정하는 관절. 손목 (elbow_roll) 은 위치에 영향이 없고 처짐이 0.8° 안팎 (되돌림 틈 꼴) 이라 모델이 거의 못 줄여 보정하지 않는다
GRIP_M = 0.133             # pos_retarget.ROBOT_GRIP_M (손목 관절 → 집게 끝)
# 맞추기 자료 고르기 (로그 20 Hz): 위치 모드가 된 뒤 START_SKIP_S 지나고, 앞뒤 STILL_ROWS 줄 동안 명령·실제가 STILL_DEG 안에서만 움직인 줄
START_SKIP_S = 2.0
STILL_ROWS = 6
STILL_DEG = 0.4
TABLE_SKIP_CM = 1.5        # 명령 자세가 판에서 이보다 가까우면 (주차·판에 기댐) 뺀다 — 판이 받치면 처짐이 중력 꼴이 아니다
LOG_GLOBS = ("hand_*.csv", "live_*.csv", "replay_*.csv")


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]])


def grav_features(origins: Sequence[np.ndarray], q: Dict[str, float]) -> np.ndarray:
    """origins = ArmModel._O (관절 원점 4×4 다섯 개), q = URDF 관절각 → (5 관절, 2 점) 중력 토크 꼴 [m]."""
    M = np.eye(4)
    axes, orgs = [], []
    el = None
    for n, O in zip(JOINTS, origins):
        M = M @ O
        axes.append(M[:3, 2].copy())
        orgs.append(M[:3, 3].copy())
        M = M @ _rz(q[n])
        if n == "elbow_pitch":
            el = M[:3, 3].copy()
    wr, ax = M[:3, 3], M[:3, 2]
    hx = ax if float(np.dot(ax, wr - el)) > 0 else -ax
    tip = wr + GRIP_M * hx
    g = np.array([0.0, 0.0, -1.0])
    out = np.zeros((len(JOINTS), 2))
    for i in range(len(JOINTS)):
        for k, p in enumerate((el, tip)):
            out[i, k] = float(np.dot(axes[i], np.cross(p - orgs[i], g)))
    return out


class SagModel:
    """params = {side: {joint: [k_el, k_tip, c]}} (rad / m). q_of = 워치독 motor_to_urdf_fn, models = {side: PR.ArmModel}."""

    def __init__(self, params: Dict[str, Dict[str, List[float]]], q_of: Callable, models: Dict, cap_deg: float = CAP_DEG, info: Optional[dict] = None):
        self.p = {s: {j: np.asarray(params[s][j], float) for j in JOINTS} for s in params}
        self.q_of, self.models = q_of, models
        for m in models.values():
            if not hasattr(m, "_O"):
                m.points({j: 0.0 for j in JOINTS})
        self.cap = math.radians(cap_deg)
        self.info = info or {}

    @classmethod
    def load(cls, q_of: Callable, models: Dict, path=MODEL_PATH, cap_deg: float = CAP_DEG) -> "SagModel":
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(d["params"], q_of, models, cap_deg=cap_deg, info=d)

    def predict(self, side: str, cmd: Dict[str, float]) -> Dict[str, float]:
        """명령 (모터 값) → 예상 처짐 (실제 − 명령, rad)."""
        F = grav_features(self.models[side]._O, self.q_of(side, cmd))
        p = self.p[side]
        return {j: float(p[j][0] * F[i, 0] + p[j][1] * F[i, 1] + p[j][2]) for i, j in enumerate(JOINTS)}

    def actual(self, side: str, cmd: Dict[str, float]) -> Dict[str, float]:
        """명령을 보내면 실제로 갈 자세 (예상)."""
        d = self.predict(side, cmd)
        return {j: float(cmd[j]) + d[j] for j in JOINTS}

    def command_for(self, side: str, goal: Dict[str, float], gain: float = 1.0, iters: int = 2) -> Dict[str, float]:
        """실제가 goal 에 오게 보낼 명령 (goal − gain·처짐, 관절당 상한)."""
        c = {j: float(goal[j]) for j in JOINTS}
        for _ in range(iters):
            d = self.predict(side, c)
            c = {j: float(goal[j]) - (max(-self.cap, min(self.cap, gain * d[j])) if j in COMP_JOINTS else 0.0) for j in JOINTS}
        return c


# --------------------------------------------------------------------------- 맞추기 (fit)
def _log_time(path: Path, first_t: float, last_t: float) -> Optional[str]:
    st = path.with_name(path.stem + "_start.json")
    if st.exists():
        try:
            return json.loads(st.read_text(encoding="utf-8"))["time"]
        except Exception:
            pass
    stem = path.stem
    if stem.startswith("replay_"):
        try:
            return dt.datetime.strptime(stem[len("replay_"):len("replay_") + 15], "%Y%m%d_%H%M%S").isoformat()
        except ValueError:
            pass
    return dt.datetime.fromtimestamp(path.stat().st_mtime - (last_t - first_t)).isoformat()   # 끝난 시각 − 길이


def collect(paths: Sequence[Path], W, DE, log=print) -> List[dict]:
    """로그마다 자료 (거의 멈춘 위치 모드 줄): {file, side, cmd (N,5), meas (N,5), F (N,5,2), tip_cmd, tip_meas}."""
    out = []
    import table_limit as TL  # noqa
    table = TL.TableLimit(TOOLS.parent / "config/table.json", body="real")
    for path in paths:
        rows = list(csv.DictReader(open(path, encoding="utf-8")))
        if not rows or "left_shoulder_pitch_meas" not in rows[0] or "mode" not in rows[0]:
            continue
        try:
            t = np.array([float(r["t"]) for r in rows])
            when = _log_time(path, t[0], t[-1])
            ref_path = DE.ref_path_for(when)
        except Exception as exc:
            log(f"  {path.name}: 건너뜀 ({exc})")
            continue
        ref = W.PR.RobotRef.load(ref_path)
        sides = tuple(s for s in ("left", "right") if f"{s}_shoulder_pitch_meas" in rows[0])
        models = {s: W.PR.ArmModel(s) for s in sides}
        for m in models.values():
            m.points({j: 0.0 for j in JOINTS})
        q_of = W.motor_to_urdf_fn(ref, sides, models)
        tfn = W.make_table_clear_fn(ref, sides, table)
        pos = np.array([r["mode"] == "POS" for r in rows])
        since = np.full(len(rows), -1.0)
        for i in range(len(rows)):
            if pos[i]:
                since[i] = (since[i - 1] + (t[i] - t[i - 1])) if i and pos[i - 1] and since[i - 1] >= 0 else 0.0
        for s in sides:
            try:
                C = np.array([[float(r[f"{s}_{j}_cmd"]) for j in JOINTS] for r in rows])
                Mm = np.array([[float(r[f"{s}_{j}_meas"]) for j in JOINTS] for r in rows])
            except (KeyError, ValueError):
                continue
            ok = np.isfinite(C).all(1) & np.isfinite(Mm).all(1) & (since >= START_SKIP_S)
            lim = math.radians(STILL_DEG)
            for i in np.nonzero(ok)[0]:
                a, b = max(0, i - STILL_ROWS), min(len(rows), i + STILL_ROWS + 1)
                if not (pos[a:b].all() and np.ptp(C[a:b], 0).max() <= lim and np.ptp(Mm[a:b], 0).max() <= lim):
                    ok[i] = False
            idx = np.nonzero(ok)[0]
            keep, F, tc, tm = [], [], [], []
            for i in idx:
                c = dict(zip(JOINTS, C[i]))
                if tfn(s, c)["min"] * 100.0 < TABLE_SKIP_CM:
                    continue
                keep.append(i)
                F.append(grav_features(models[s]._O, q_of(s, c)))
                for arr, src in ((tc, C[i]), (tm, Mm[i])):
                    el, wr, ax = models[s].points(q_of(s, dict(zip(JOINTS, src))))
                    hx = ax if float(np.dot(ax, wr - el)) > 0 else -ax
                    arr.append((wr + GRIP_M * hx) * 100.0)
            if keep:
                out.append({"file": path.name, "side": s, "ref": ref_path.name, "cmd": C[keep], "meas": Mm[keep], "F": np.array(F),
                            "tip_cmd": np.array(tc), "tip_meas": np.array(tm), "t": t[keep]})
    return out


def fit_params(chunks: List[dict], sides=("left", "right")) -> Dict[str, Dict[str, List[float]]]:
    params = {}
    for s in sides:
        cs = [c for c in chunks if c["side"] == s]
        if not cs:
            continue
        F = np.concatenate([c["F"] for c in cs])
        D = np.concatenate([c["meas"] - c["cmd"] for c in cs])
        # 로그마다 같은 무게 (긴 로그가 이기지 않게)
        w = np.concatenate([np.full(len(c["cmd"]), 1.0 / math.sqrt(len(c["cmd"]))) for c in cs])
        params[s] = {}
        for i, j in enumerate(JOINTS):
            X = np.column_stack((F[:, i, 0], F[:, i, 1], np.ones(len(F))))
            sol, *_ = np.linalg.lstsq(X * w[:, None], D[:, i] * w, rcond=None)
            params[s][j] = [float(v) for v in sol]
    return params


def main() -> int:
    ap = argparse.ArgumentParser(description="처짐 모델 맞추기")
    ap.add_argument("cmd", choices=["fit"])
    ap.add_argument("--out", default=str(MODEL_PATH))
    ap.add_argument("--exclude", nargs="*", default=[], help="뺄 로그 이름 (시험용으로 남길 것)")
    a = ap.parse_args()
    sys.path.insert(0, str(TOOLS))
    sys.path.insert(0, str(Path.home() / "code_workspace/Berkeley-Humanoid-Lite-main/Berkeley-Humanoid-Lite-Lowlevel"))
    import vr_control_watchdog_biarm as W  # type: ignore
    import demo_export as DE
    W.PR.set_hand_model("cad")
    paths = sorted({p for g in LOG_GLOBS for p in LOGS.glob(g) if not p.stem.endswith(("_dry", "_phases")) and p.name not in a.exclude})
    chunks = collect(paths, W, DE)
    params = fit_params(chunks)
    files = sorted({c["file"] for c in chunks})
    d = {"params": params, "made": dt.datetime.now().isoformat(timespec="seconds"), "files": files,
         "rows": {s: int(sum(len(c["cmd"]) for c in chunks if c["side"] == s)) for s in params}, "excluded": a.exclude,
         "model": "처짐_i = k_el·τ_i(팔꿈치) + k_tip·τ_i(집게 끝) + c (rad, τ 는 m), tools/sag_comp.py 머리 주석"}
    Path(a.out).write_text(json.dumps(d, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"저장: {a.out} (로그 {len(files)} 개, 줄 {d['rows']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
