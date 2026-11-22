#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""학습용 관측·행동 정의 (작업 지시 15a). demo_export npz → 배열. to_lerobot.py·learn_mlp.py·policy_sim.py 가 같이 쓴다.

관측 (20 Hz, demo_export 격자 그대로):
  observation.state (12)  = 관절 10 (URDF rad, 왼 5 → 오른 5, JOINTS 차례) + 그리퍼 닫힌 정도 2 (grip_deg / 닫힘 각, 0 열림 ~ 1 닫힘)
      dry-run 기록에는 엔코더가 없어 관절 = 명령값 (q_cmd). 실로봇에서는 엔코더 (meas) 가 된다 → meas_sim 옵션:
      처짐 모델 (sag_comp, config/sag_model.json) 로 명령 → 실제 예측 + 작은 잡음 (기본 0.3°) 을 관측에만 넣는다 (행동은 그대로 명령).
      그리퍼는 실로봇에서도 각도를 못 읽으므로 보낸 명령 + 서보 시간 모델 (gripper_link.ServoModel) 값이다 — dry-run 과 같다.
  observation.environment_state (8) = 캔 밑면 가운데 3 (m, 로봇 base) + 쿼터니언 4 (xyzw, 캔 → base) + can_source 1 (0 가상·1 카메라·2 수동)
      실로봇에는 캔 센서가 없다. 실행 때는 virtual_can 을 같이 돌려 (시작 자리 = 사람이 놓은 자리, can_source 2) 명령으로 캔을 따라간다 — 기록과 같은 계산.
  에피소드 진행 정도는 넣지 않는다 (지시 15a A2).
행동 (12) = 다음 프레임 관절 목표 10 (q_cmd[t+1], URDF rad) + 그리퍼 판단 2 (grip[t], 0 열림 / 1 닫힘). 마지막 프레임은 제자리.
  묶음 (ACT chunk) 은 학습 쪽에서 (기본 20 프레임 = 1 초).
URDF 각을 쓰는 까닭: 모터값은 전원마다 영점이 24° 씩 다를 수 있다 (robot_ref). 실행 때 그 전원 robot_ref 로 모터값으로 바꾼다 (demo_export.JointMap).

데이터 묶음:
  base = release 끝 (14-h) + 실제 그리퍼 속도 (14-j) + 정렬 판정 (14-l) 기록의 성공 에피소드 (지금 2336 ep1~3, 0015 ep1)
  wide = base + 11-03 옛 시연 성공 (no_release 표시, 놓고 손을 빼지 않음) — 파이프라인 시험의 "많은 데이터" 흉내만
제외 목록 (작업 지시 17-a, 16-b 제안): logs/exclude_episodes.json 의 (기록, 에피소드) 는 episode_files 가 건너뛴다 (파일이 없으면 지금과 같다).
  기록 이름 고르기: episode_files(records=[…]) 면 그 기록만 (None = 전부, 지금과 같다).
조건 입력 (작업 지시 17-a, 기본 끔): arrays(cond="xy") 면 state 뒤에 시작 자리 · 목표 자리 판 좌표 (base m, x·y 4 값, COND_NAMES) 를 붙인다 (state 16).
  cond="zero" 는 같은 자리에 0 (조건을 쓰는지 비교용). 좌표는 자리 이름의 판 좌표 (캔을 실제 놓은 위치가 아니다 — 실행 때 사용자가 고르는 값과 같은 뜻)."""
from __future__ import annotations

import glob
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

TOOLS = Path(__file__).resolve().parent
LOWLEVEL = Path.home() / "code_workspace/Berkeley-Humanoid-Lite-main/Berkeley-Humanoid-Lite-Lowlevel"
for _p in (str(TOOLS), str(LOWLEVEL)):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import demo_export as DE  # noqa: E402

DATA = Path.home() / "cc_analysis/learn/data"
SIDES = DE.SIDES
JOINTS = DE.JOINTS
STATE_NAMES = [f"{s}_{j}" for s in SIDES for j in JOINTS] + ["grip_L_closed", "grip_R_closed"]
ENV_NAMES = ["can_x_m", "can_y_m", "can_z_m", "can_qx", "can_qy", "can_qz", "can_qw", "can_source"]
ACTION_NAMES = [f"{s}_{j}_next" for s in SIDES for j in JOINTS] + ["grip_L_cmd", "grip_R_cmd"]
FPS = 20
MEAS_NOISE_DEG = 0.3
COND_NAMES = ["cond_start_x_m", "cond_start_y_m", "cond_goal_x_m", "cond_goal_y_m"]
COND_MODES = ("xy", "zero")
EXCLUDE = DE.LOGS / "exclude_episodes.json"
_excl_said = set()


def load_exclude(path: Path = EXCLUDE) -> Dict[tuple, str]:
    """{(기록, 에피소드): 이유}. 파일이 없으면 빈 dict (지금과 같다)."""
    if not Path(path).exists():
        return {}
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    return {(e["record"], int(n)): e.get("reason", "") for e in d.get("entries", []) for n in e["episodes"]}


def episode_files(kind: str = "base", include_excluded: bool = False, records: Optional[List[str]] = None) -> List[dict]:
    """[{path, meta, set}] — set = base / old (wide 에만). 제외 목록 (EXCLUDE) 에 있는 판은 빼고 (include_excluded=True 면 넣는다), records 를 주면 그 기록만."""
    out = []
    excl = {} if include_excluded else load_exclude()
    skipped: Dict[str, int] = {}
    for f in sorted(glob.glob(str(DATA / "**" / "*.npz"), recursive=True)):
        m = DE.load_episode(f)["meta"]
        if not m.get("success"):
            continue
        if records is not None and m.get("record") not in records:
            continue
        if m.get("aug") and records is None:                # 19-c1: 증강으로 만든 판은 기록 이름을 직접 줄 때만 쓴다 (안 주면 지금과 같은 목록)
            continue
        why = excl.get((m.get("record"), int(m.get("episode") or 0)))
        if why is not None:
            skipped[why] = skipped.get(why, 0) + 1
            continue
        opts = m.get("options") or {}
        vc = opts.get("virtual_can") or {}
        release = (vc.get("episode_end") or {}).get("mode") == "release"
        real_timing = ((opts.get("grip_virtual") or {}).get("timing") or {}).get("mode") == "real"
        aligned = bool(vc.get("grasp_align"))
        no_rel = bool((m.get("quality") or {}).get("no_release"))
        if release and real_timing and aligned and not no_rel:
            out.append({"path": f, "meta": m, "set": "base"})
        elif kind == "wide":
            out.append({"path": f, "meta": m, "set": "old"})
    key = (kind, tuple(records or ()), tuple(sorted(skipped.items())))
    if skipped and key not in _excl_said:                  # 같은 결과는 한 번만 알린다
        _excl_said.add(key)
        for why, n in skipped.items():
            print(f"[제외] {n} 판 — {why[:60]}", file=sys.stderr)
    return out


def excluded_list(records: Optional[List[str]] = None) -> List[str]:
    """학습 결과 정보에 남길 '빠진 판' 이름 (기록_epNN)."""
    return [f"{r}_ep{n:02d}" for (r, n) in sorted(load_exclude()) if records is None or r in records]


def xy_cond(start_xy_cm, goal_xy_cm, mode: Optional[str]) -> np.ndarray:
    """19-b 실행 · 시뮬레이션용: 좌표 (base cm) 로 조건 4 값 (cond_vector 와 같은 값 — base m)."""
    return cond_vector({"can_start_slot": "S0", "can_goal_slot": "G0", "can_start_xy_cm": [float(v) for v in start_xy_cm], "can_goal_xy_cm": [float(v) for v in goal_xy_cm]}, mode)


def slot_cond(start: str, goal: str, mode: Optional[str]) -> np.ndarray:
    """실행 · 시뮬레이션용: 자리 이름으로 조건 4 값 (cond_vector 와 같은 값)."""
    return cond_vector({"start_slot": start, "goal_slot": goal}, mode)


def checkpoint_cond(spec: str) -> dict:
    """17-a: 체크포인트가 학습한 조건 입력. {"cond": None|"xy"|"zero", "records": [...]|None}.
    act:<pretrained_model> 은 train_config.json 의 데이터셋 폴더 build_info.json, mlp:<model.pt> 는 cfg. 없으면 None (지금 꼴, state 12)."""
    kind, _, path = spec.partition(":")
    p = Path(path).expanduser()
    try:
        if kind == "act":
            root = Path(json.loads((p / "train_config.json").read_text(encoding="utf-8"))["dataset"]["root"])
            bi = root / "build_info.json"
            if bi.exists():
                d = json.loads(bi.read_text(encoding="utf-8"))
                return {"cond": d.get("cond"), "records": d.get("records")}
        elif kind == "mlp":
            import torch
            cfg = torch.load(p, map_location="cpu", weights_only=False)["cfg"]
            return {"cond": cfg.get("cond"), "records": cfg.get("records")}
    except (OSError, KeyError, ValueError):
        pass
    return {"cond": None, "records": None}


def cond_vector(meta: dict, mode: Optional[str]) -> np.ndarray:
    """조건 입력 4 값 (base m): 시작 x·y, 목표 x·y. mode "zero" 면 0."""
    if mode == "zero":
        return np.zeros(4, np.float32)
    c = can_slots(meta)
    return (np.array(list(c["start_xy_cm"]) + list(c["goal_xy_cm"]), float) / 100.0).astype(np.float32)


class MeasSim:
    """명령 (motor) → 처짐 모델 예측 실제 + 잡음. robot_ref 마다 하나."""

    def __init__(self, ref_name: str, noise_deg: float = MEAS_NOISE_DEG, seed: int = 0):
        import vr_control_watchdog_biarm as W
        import sag_comp as SAG
        self.ref = W.PR.RobotRef.load(DE.LOGS / ref_name)
        models = {s: W.PR.ArmModel(s) for s in SIDES}
        self.sag = SAG.SagModel.load(W.motor_to_urdf_fn(self.ref, SIDES, models), models)
        self.jm = DE.JointMap(self.ref)
        self.noise = math.radians(noise_deg)
        self.rng = np.random.default_rng(seed)

    def q_obs(self, motor_cmd: np.ndarray) -> np.ndarray:
        """(N,2,5) 모터 명령 → (N,2,5) URDF 관측 (예측 실제 + 잡음)."""
        out = np.empty_like(motor_cmd)
        for n in range(len(motor_cmd)):
            for i, s in enumerate(SIDES):
                act = self.sag.actual(s, dict(zip(JOINTS, motor_cmd[n, i])))
                out[n, i] = [self.jm.to_urdf(s, act)[j] for j in JOINTS]
        return out + self.rng.normal(0, self.noise, out.shape)


def grip_frac(ep: dict) -> np.ndarray:
    m = ep["meta"]
    close = ((m.get("options") or {}).get("gripper_close_deg") or None)
    g = np.asarray(ep["grip_deg"], float) if "grip_deg" in ep else np.asarray(ep["grip"], float) * 90.0
    if close is None:                       # 기록의 닫힘 각 (14-n 전은 90): 그 에피소드의 최대각
        close = [max(float(np.nanmax(g[:, k])), 1.0) for k in range(2)]
    return np.clip(g / np.asarray(close, float)[None, :], 0.0, 1.0)


def can_slots(meta: dict) -> dict:
    """작업 지시 16-a: 에피소드의 시작·목표 자리 이름과 판 좌표 (base cm). --can-combos 기록은 meta 의 can_*, 옛 기록은 start_slot·goal_slot 과 table.json."""
    if meta.get("can_start_slot"):
        return {"start": meta["can_start_slot"], "goal": meta["can_goal_slot"], "start_xy_cm": list(meta["can_start_xy_cm"]),
                "goal_xy_cm": list(meta["can_goal_xy_cm"])}
    slots = {s["name"]: s for s in json.loads((TOOLS.parent / "config/table.json").read_text(encoding="utf-8"))["can_slots"]}
    st, go = meta.get("start_slot") or "L1", meta.get("goal_slot") or "R1"
    return {"start": st, "goal": go, "start_xy_cm": list(slots[st]["base_xy_cm"]), "goal_xy_cm": list(slots[go]["base_xy_cm"])}


def arrays(path, meas_sim: Optional[MeasSim] = None, cond: Optional[str] = None) -> Dict[str, np.ndarray]:
    """한 에피소드 → state (N,12; cond 를 주면 N,16), env (N,8), action (N,12), 그리고 meta."""
    ep = DE.load_episode(path)
    q = np.asarray(ep["q_cmd"], float)                                  # (N,2,5)
    q_obs = meas_sim.q_obs(np.asarray(ep["motor_cmd"], float)) if meas_sim is not None else q
    N = len(q)
    gf = grip_frac(ep)
    state = np.concatenate([q_obs.reshape(N, 10), gf], 1)
    if cond is not None:                                                # 17-a (켤 때만)
        if cond not in COND_MODES:
            raise ValueError(f"cond 는 {COND_MODES} 중 하나")
        state = np.concatenate([state, np.repeat(cond_vector(ep["meta"], cond)[None], N, 0)], 1)
    env = np.concatenate([np.asarray(ep["can_pos"], float) / 100.0, np.asarray(ep["can_quat"], float),
                          np.asarray(ep["can_source"], float)[:, None]], 1)
    qn = np.concatenate([q[1:], q[-1:]], 0).reshape(N, 10)
    action = np.concatenate([qn, np.asarray(ep["grip"], float)], 1)
    return {"state": state.astype(np.float32), "env": env.astype(np.float32), "action": action.astype(np.float32),
            "meta": ep["meta"], "t": np.asarray(ep["t"], float)}


def chunk_targets(action: np.ndarray, k: int) -> (np.ndarray, np.ndarray):
    """(N,12) → 묶음 (N,k,12) 과 pad (N,k). 끝을 넘으면 마지막 행동을 되풀이하고 pad=1 (ACT 의 action_is_pad 와 같은 뜻)."""
    N = len(action)
    idx = np.minimum(np.arange(N)[:, None] + np.arange(k)[None, :], N - 1)
    pad = (np.arange(N)[:, None] + np.arange(k)[None, :]) >= N
    return action[idx], pad


def stats(arrs: List[Dict[str, np.ndarray]]) -> Dict[str, Dict[str, list]]:
    out = {}
    for key in ("state", "env", "action"):
        x = np.concatenate([a[key] for a in arrs], 0)
        sd = x.std(0)
        out[key] = {"mean": x.mean(0).tolist(), "std": np.where(sd < 1e-6, 1.0, sd).tolist()}
    return out


if __name__ == "__main__":
    for kind in ("base", "wide"):
        eps = episode_files(kind)
        print(kind, len(eps), "에피소드:", ", ".join(Path(e["path"]).stem + ("" if e["set"] == "base" else "*") for e in eps))
    a = arrays(episode_files("base")[0]["path"])
    print({k: v.shape for k, v in a.items() if hasattr(v, "shape")})
