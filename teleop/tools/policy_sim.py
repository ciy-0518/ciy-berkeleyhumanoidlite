#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""정책 닫힌 고리 평가 — 기구학 시뮬레이션 (작업 지시 15a C2·C3). 로봇·CAN·VR 없음. ~/venvs/learn 에서.

    ~/venvs/learn/bin/python policy_sim.py --policy act:~/cc_analysis/learn/runs/act_base/checkpoints/last/pretrained_model
    ~/venvs/learn/bin/python policy_sim.py --policy mlp:~/cc_analysis/learn/runs/mlp_base/model.pt --ensemble
    ~/venvs/learn/bin/python policy_sim.py --policy replay:dry_vcan_2336_ep02     # 고리 확인용: 정책 대신 기록 행동을 그대로

고리 (20 Hz 정책, 200 Hz 안쪽):
  관측 = learn_data 와 같은 꼴 — 관절 (명령값, URDF rad) + 그리퍼 닫힌 정도 (ServoModel 시간 모델) + 캔 자세 (virtual_can, 출처 0)
  정책 행동 → 관절 목표 (URDF) → 지금 robot_ref 로 모터값 (demo_export.JointMap) → 워치독 DualArmMotorController.update_targets
    (dry-run, 그대로 import: 관절 한계 (좌우 통일)·속도 제한·비틀기 창·양팔 충돌·판 면 — replay_run 과 같은 구성) → 다음 관측
  그리퍼 = 행동 > 0.5 면 닫음 → config/gripper.json 각도로 ServoModel (펌웨어와 같은 속도, 한 번에 하나) → 다 닫힘 = 1 (TimedVirtualGrip 과 같은 뜻)
  캔 = virtual_can.VirtualCan (dry-run 과 같은 잡기·정렬 (14-l)·인계·놓기·release 끝 (14-h) 판정), 시작 L1 · 목표 R1
시작 자세: 모델 주차 자세에서 학습 에피소드 첫 프레임 평균 자세로 --move-s 초 동안 옮긴 뒤 정책 시작 (실행 때 '시작 자세 이동' 과 같은 자리).
끝: 에피소드 결과 (auto_success / fallen / timeout) 또는 --max-s.
출력: <out>/<이름>.json (요약·실패 이유), <이름>.npz (demo_export 형식 → replay_run.py 가 읽는다).
작업 지시 17-a (모두 기본 꺼짐 — 안 주면 지금과 같다):
  --start · --goal 자리 (기본 L1 · R1). 정책이 조건 입력 (learn_data.checkpoint_cond) 으로 학습했으면 관측 state 뒤에 그 자리 조건 4 값을 붙인다.
  --pose-jitter-deg D: 시작 자세 (첫 프레임 평균) 관절마다 ±D° 균일 흔들림, --can-jitter-cm C: 시작 캔 자리 x·y 를 ±C cm 균일 흔들림, --seed.
  --meas-sim: 관측 관절 = learn_data.MeasSim (처짐 모델 + 잡음) — --meas-sim 데이터로 학습한 정책을 실로봇 관측 꼴로 본다.
  --strict: 가상 캔 엄격 판정 (16-a) 을 켜 요약에 strict 항목. --align-axis-cm: 둘째 손 거리 한도 (기본 1.5 그대로, 시연 기록은 2.0).
  첫 자세는 정책이 학습한 기록 (checkpoint_cond records) 이 있으면 그 기록의 첫 프레임 평균."""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import learn_data as L  # noqa: E402  (sys.path 에 Lowlevel 도 넣는다)
import vr_control_watchdog_biarm as W  # noqa: E402
import virtual_can as VCAN  # noqa: E402
import table_limit as TLIM  # noqa: E402
import gripper_link as GL  # noqa: E402
import demo_export as DE  # noqa: E402

SIDES = L.SIDES
JOINTS = L.JOINTS
OUT = Path.home() / "cc_analysis/learn/rollouts"
INNER = 10                    # 정책 한 걸음 (0.05 초) 에 안쪽 200 Hz 10 번
DT = 1.0 / W.LOOP_HZ


# --------------------------------------------------------------------------- 정책
class ActPolicy:
    def __init__(self, path, device="cuda", ensemble=False):
        import torch
        cc = L.checkpoint_cond(f"act:{path}")
        self.cond, self.records = cc["cond"], cc["records"]
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.factory import make_pre_post_processors
        p = Path(path).expanduser()
        self.policy = ACTPolicy.from_pretrained(str(p))
        if ensemble:                                   # ACT 의 temporal ensembling: 매 걸음 묶음을 새로 내고 겹친 것을 섞는다
            self.policy.config.temporal_ensemble_coeff = 0.01
            self.policy.config.n_action_steps = 1
            from lerobot.policies.act.modeling_act import ACTTemporalEnsembler
            self.policy.temporal_ensembler = ACTTemporalEnsembler(0.01, self.policy.config.chunk_size)
        self.policy.to(device).eval()
        self.pre, self.post = make_pre_post_processors(self.policy.config, pretrained_path=str(p))
        self.torch, self.device = torch, device

    def reset(self):
        self.policy.reset()

    def act(self, state, env):
        t = self.torch
        batch = {"observation.state": t.tensor(state, dtype=t.float32)[None], "observation.environment_state": t.tensor(env, dtype=t.float32)[None]}
        with t.no_grad():
            a = self.policy.select_action(self.pre(batch))
            a = self.post(a)
        return np.asarray(a.cpu().numpy()[0], float)


class ReplayPolicy:
    """고리 확인용: 기록된 행동을 시각대로 (관측은 무시)."""

    def __init__(self, name):
        hits = [e for e in L.episode_files("wide", include_excluded=True) if Path(e["path"]).stem.endswith(name)]
        self.arr = L.arrays(hits[0]["path"])
        self.i = 0
        self.cond, self.records = None, None

    def reset(self):
        self.i = 0

    def act(self, state, env):
        a = self.arr["action"][min(self.i, len(self.arr["action"]) - 1)]
        self.i += 1
        return np.asarray(a, float)


def load_policy(spec: str, ensemble: bool = False):
    kind, _, path = spec.partition(":")
    if kind == "act":
        return ActPolicy(path, ensemble=ensemble)
    if kind == "mlp":
        import learn_mlp as LM
        p = LM.MlpPolicy(Path(path).expanduser(), ensemble=ensemble)
        p.cond, p.records = p.cfg.get("cond"), p.cfg.get("records")
        return p
    if kind == "replay":
        return ReplayPolicy(path)
    raise SystemExit(f"--policy 는 act:<폴더> / mlp:<model.pt> / replay:<에피소드>")


# --------------------------------------------------------------------------- 그리퍼
class SimGrip:
    """ServoModel (펌웨어와 같은 속도) + 다 닫힘/다 열림 상태 (TimedVirtualGrip 과 같은 뜻). state() 는 virtual_can 이 읽는 꼴."""

    def __init__(self, open_deg, close_deg):
        self.o, self.c = open_deg, close_deg
        self.model = GL.ServoModel(open_deg[0], open_deg[1])
        self.want = (0, 0)
        self.eff = {"L": 0, "R": 0}

    def command(self, now, want):
        want = tuple(int(v) for v in want)
        if want != self.want:
            self.want = want
            self.model.command(now, self.c[0] if want[0] else self.o[0], self.c[1] if want[1] else self.o[1])

    def update(self, now):
        ang = self.model.angles(now)
        for i, h in enumerate("LR"):
            if ang[i] == self.c[i]:
                self.eff[h] = 1
            elif ang[i] == self.o[i]:
                self.eff[h] = 0

    def frac(self, now):
        a = self.model.angles(now)
        return np.array([(a[i] - self.o[i]) / max(self.c[i] - self.o[i], 1) for i in range(2)], float)

    def deg(self, now):
        return np.array(self.model.angles(now), float)

    def state(self):
        return dict(self.eff)


# --------------------------------------------------------------------------- 환경
def coord(spec):
    """19-c1: 좌표로 준 자리면 (x, y) cm, 자리 이름이면 None. 좌표 = (x, y) 묶음 또는 "x,y" 글."""
    if isinstance(spec, (tuple, list, np.ndarray)) and len(spec) == 2:
        return float(spec[0]), float(spec[1])
    if isinstance(spec, str) and "," in spec:
        a, b = spec.split(",", 1)
        return float(a), float(b)
    return None


def coord_text(xy) -> str:
    return f"{xy[0]:g},{xy[1]:g}"


# 작업 지시 19-x (기본 빈 사전 = 지금 그대로): 시뮬 가상 캔에 더 넘길 인자. aug_demo · eval_pairs 의 --grasp-mode top · --grasp-real 이 채운다
VCAN_EXTRA: dict = {}


class SimEnv:
    def __init__(self, start="L1", goal="R1", ref_path=None, align=True, timeout_s=60.0, align_axis_cm=1.5, strict=False, can_offset_cm=(0.0, 0.0)):
        W.PR.set_hand_model("cad")
        W.apply_yaw_in_limit(100.0, 100.0)                        # replay_run SAFETY_ARGS 와 같다
        self.ref_path = Path(ref_path) if ref_path else W.newest_robot_ref(None)
        self.ref = W.PR.RobotRef.load(self.ref_path)
        limits = W.limits_from_robot_ref(self.ref, SIDES)
        limits, _ = W.mirror_limits(limits, self.ref, SIDES)      # --mirror-limits on
        self.limits = limits
        self.table = TLIM.TableLimit(L.TOOLS.parent / "config/table.json", body="real")
        m = W.DualArmMotorController(channels={"left": "can0", "right": "can1"}, sides=SIDES,
                                     motor_ids={"left": (1, 3, 5, 7, 9), "right": (2, 4, 6, 8, 10)}, urdf_limits=limits,
                                     kp=0.0, kd=0.0, torque_limit=0.0, max_speed=2.5, dry_run=True)
        m.yaw_dyn = {s: (float(self.ref.rest[s]["shoulder_pitch"]), float(self.ref.e90[s]["shoulder_yaw"])) for s in SIDES}
        m.arm_gap_fn = W.make_arm_gap_fn(self.ref, SIDES)
        m.arm_gap_min = -0.02                                     # --arm-gap-cm -2
        m.table_fn = W.make_table_clear_fn(self.ref, SIDES, self.table)
        m.dry_run_base = W.model_park_pose(self.ref, SIDES, m.table_fn)
        self.motor = m
        self.jm = DE.JointMap(self.ref)
        q_of = W.motor_to_urdf_fn(self.ref, SIDES)
        (o_l, o_r), (c_l, c_r) = GL.grip_deg()
        self.grip = SimGrip((o_l, o_r), (c_l, c_r))
        self.xy = None                                            # 19-c1: 좌표로 준 자리 ((시작 x, y), (도착 x, y)) — 이름이면 None (지금 그대로)
        if coord(start) is not None or coord(goal) is not None:
            by = {s["name"]: s["base_xy_cm"] for s in self.table.cfg["can_slots"]}
            self.xy = tuple(coord(v) or (float(by[str(v).upper()][0]), float(by[str(v).upper()][1])) for v in (start, goal))
            start, goal = VCAN.PAIR_START, VCAN.PAIR_GOAL
        self.vcan = VCAN.VirtualCan(self.table, start, self.ref, SIDES, q_of, goal=goal, margin_cm=VCAN.GRIP_MARGIN_CM,
                                    servo_sign=W.table_servo_signs(self.ref, SIDES), log=lambda s: None, reset_s=3.0, timeout_s=timeout_s,
                                    episode_end="release", align=(15.0, 10.0, align_axis_cm) if align else None, **({"strict": True} if strict else {}),
                                    **({"pairs": [(*self.xy[0], *self.xy[1])]} if self.xy is not None else {}), **VCAN_EXTRA)
        if any(can_offset_cm):                                    # 17-a: 시작 캔 자리 흔들림 (그 자리 밑면을 옮긴다)
            off = np.array([can_offset_cm[0], can_offset_cm[1], 0.0])
            base_fn, st_name = self.vcan._slot_base, self.vcan.start_name
            self.vcan._slot_base = lambda name, f=base_fn: f(name) + (off if name == st_name else 0.0)
            self.vcan.pos = self.vcan._slot_base(st_name)
        self.cond = None                                          # 17-a: 관측 state 뒤 조건 (None = 지금 꼴)
        self.meas = None                                          # 17-a: learn_data.MeasSim (None = 명령값)
        self.now = 0.0

    def start(self):
        import contextlib, io
        with contextlib.redirect_stdout(io.StringIO()):
            self.motor.enter_position_from_current()
        self.now = 0.0

    def q_urdf(self) -> np.ndarray:
        return np.array([[self.jm.to_urdf(s, self.motor.command_target[s])[j] for j in JOINTS] for s in SIDES])

    def obs(self):
        if self.meas is not None:
            m = np.array([[self.motor.command_target[s][j] for j in JOINTS] for s in SIDES])[None]
            q = self.meas.q_obs(m)[0].reshape(10)
        else:
            q = self.q_urdf().reshape(10)
        state = np.concatenate([q, self.grip.frac(self.now)]).astype(np.float32)
        if self.cond is not None:
            state = np.concatenate([state, self.cond]).astype(np.float32)
        env = np.concatenate([self.vcan.pos / 100.0, self.vcan.q, [0.0]]).astype(np.float32)
        return state, env

    def step(self, q_target: np.ndarray, grip_want, running=True):
        """q_target (2,5) URDF. 안쪽 200 Hz 10 번. 반환: 안전장치가 바꾼 양 (rad, 목표 − 명령 최대)."""
        tgt = {s: self.jm.to_motor(s, dict(zip(JOINTS, q_target[i]))) for i, s in enumerate(SIDES)}
        self.grip.command(self.now, grip_want)
        for _ in range(INNER):
            self.motor.update_targets({s: {j: tgt[s][j] - self.motor.base_target[s][j] for j in W.JOINT_ORDER} for s in SIDES}, DT)
            self.now += DT
            self.grip.update(self.now)
            self.vcan.tick(self.now, self.motor.command_target, self.grip, None, running=running)
        return max(abs(tgt[s][j] - self.motor.command_target[s][j]) for s in SIDES for j in JOINTS)


def first_pose(records=None) -> np.ndarray:
    eps = [L.arrays(e["path"]) for e in L.episode_files("base", records=records)]
    return np.mean([a["action"][0, :10] for a in eps], 0).reshape(2, 5)


def rollout(policy, name: str, max_s: float = 60.0, move_s: float = 3.0, out_dir: Path = OUT, align=True, ref_path=None,
            start: str = "L1", goal: str = "R1", pose_jitter_deg: float = 0.0, can_jitter_cm: float = 0.0, seed: int = 0,
            meas_sim: bool = False, strict: bool = False, align_axis_cm: float = 1.5, cond_goal: Optional[str] = None, first=None) -> dict:
    rng = np.random.default_rng(seed)
    can_off = tuple(rng.uniform(-can_jitter_cm, can_jitter_cm, 2)) if can_jitter_cm > 0 else (0.0, 0.0)
    env = SimEnv(start=start, goal=goal, ref_path=ref_path, align=align, timeout_s=max_s, align_axis_cm=align_axis_cm, strict=strict, can_offset_cm=can_off)
    pcond = getattr(policy, "cond", None)
    if pcond is not None and env.xy is not None:                # 19-c1: 좌표로 준 자리 — 조건도 그 좌표로
        cg = coord(cond_goal) if cond_goal is not None else None
        if cond_goal is not None and cg is None:
            cg = tuple(float(v) for v in {s["name"]: s["base_xy_cm"] for s in env.table.cfg["can_slots"]}[str(cond_goal).upper()])
        env.cond = L.xy_cond(env.xy[0], cg or env.xy[1], pcond)
    elif pcond is not None:                                     # 17-a: 정책이 조건으로 학습했으면 (cond_goal = 조건에만 다른 목표를 넣는 시험)
        env.cond = L.slot_cond(start, cond_goal or goal, pcond)
    if meas_sim:
        env.meas = L.MeasSim(env.ref_path.name, seed=seed + 1000)
    env.start()
    q0 = env.q_urdf()
    q1 = first if first is not None else (getattr(policy, "first", None) if getattr(policy, "first", None) is not None
                                          else first_pose(getattr(policy, "records", None)))
    if pose_jitter_deg > 0:
        q1 = q1 + np.radians(rng.uniform(-pose_jitter_deg, pose_jitter_deg, q1.shape))
    # 시작 자세 이동 (캔 시계는 멈춤: running=False)
    n_move = int(move_s * L.FPS)
    for k in range(n_move):
        s = 0.5 - 0.5 * math.cos(math.pi * (k + 1) / n_move)
        env.step(q0 + (q1 - q0) * s, (0, 0), running=False)
    policy.reset()
    rows = []
    t0, wall = env.now, time.time()
    blocks0 = (env.motor.table_blocks, env.motor.gap_blocks)
    clip_max = 0.0
    result = None
    while env.now - t0 < max_s:
        state, envs = env.obs()
        a = policy.act(state, envs)
        q_t = np.asarray(a[:10], float).reshape(2, 5)
        want = (int(a[10] > 0.5), int(a[11] > 0.5))
        rows.append({"t": env.now - t0, "q": env.q_urdf(), "motor": np.array([[env.motor.command_target[s][j] for j in JOINTS] for s in SIDES]),
                     "grip": np.array(want, float), "grip_deg": env.grip.deg(env.now), "can_pos": env.vcan.pos.copy(), "can_quat": env.vcan.q.copy(),
                     "can_state": env.vcan.state, "q_target": q_t})
        clip_max = max(clip_max, env.step(q_t, want))
        if env.vcan.results:
            result = env.vcan.results[0]
            break
    if result is None:
        env.vcan.close(env.now)
        result = env.vcan.results[0] if env.vcan.results else {"result": "no_episode"}
    ev = [(round(t - t0, 2), k, {kk: (round(v, 2) if isinstance(v, float) else v) for kk, v in info.items()}) for t, k, info in env.vcan.events]
    states = [r["can_state"] for r in rows]
    summary = {"name": name, "result": result.get("result"), "grip_order": result.get("grip_order"), "handover": result.get("handover"),
               "fallen": result.get("fallen"), "place_why": result.get("place_why"), "goal_dist_cm": result.get("goal_dist_cm"),
               "bump": result.get("bump_count"), "align_miss": env.vcan.align_miss, "sim_s": round(env.now - t0, 2), "wall_s": round(time.time() - wall, 1),
               "table_blocks": env.motor.table_blocks - blocks0[0], "gap_blocks": env.motor.gap_blocks - blocks0[1],
               "safety_clip_max_deg": round(math.degrees(clip_max), 2), "max_state": max(states) if states else 0,
               "states_seen": sorted({VCAN.STATE_NAMES[s] for s in states}), "events": ev[:40], "robot_ref": env.ref_path.name}
    if start != "L1" or goal != "R1" or pcond is not None or strict or pose_jitter_deg or can_jitter_cm or meas_sim:   # 17-a (켤 때만)
        if env.xy is not None:                                  # 19-c1: 좌표는 글 ("x,y") 로 남기고 숫자도 따로
            gb = env.vcan._slot_base(env.vcan.goal_name)
            start, goal = coord_text(env.xy[0]), coord_text(env.xy[1])
            cond_goal = coord_text(coord(cond_goal)) if (cond_goal is not None and coord(cond_goal) is not None) else cond_goal
            summary.update({"start_xy_cm": list(env.xy[0]), "goal_xy_cm": list(env.xy[1])})
        else:
            gb = env.vcan._slot_base(goal) if goal else None
        summary.update({"start": start, "goal": goal, "cond": pcond, "cond_goal": cond_goal or goal, "seed": seed, "can_offset_cm": [round(float(v), 2) for v in can_off],
                        "final_can_cm": [round(float(v), 2) for v in env.vcan.pos], "final_minus_goal_cm": [round(float(v), 2) for v in (env.vcan.pos - gb)[:2]] if gb is not None else None,
                        "strict": {k: result.get(k) for k in result if str(k).startswith("strict_")} if strict else None})
    if VCAN_EXTRA.get("real"):                                  # 19-x (켤 때만): 실물 기준 판정
        summary["real"] = {k: result.get(k) for k in result if str(k).startswith("real_")}
    summary["why"] = why_text(summary)
    out_dir.mkdir(parents=True, exist_ok=True)
    export_npz(out_dir / f"{name}.npz", rows, summary, env)
    (out_dir / f"{name}.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    return summary


def why_text(s: dict) -> str:
    if s["result"] == "auto_success":
        return f"성공 (목표 {s.get('goal', 'R1')} 에 서고 손을 뺌)"
    seen = s["states_seen"]
    if "held_L" not in seen and "held_both" not in seen:
        miss = [e for e in s["events"] if e[1] in ("miss",)]
        return "L 이 캔을 못 잡음" + (f" (정렬 판정으로 못 잡은 닫기 {s['align_miss']} 번)" if s["align_miss"] else "")
    if "held_both" not in seen:
        return "L 은 잡았으나 R 이 마주 잡지 못함 (인계 없음)"
    if "held_R" not in seen:
        return "두 손으로 잡았으나 L 이 놓지 않음"
    if s["result"] == "fallen" or "fallen" in seen:
        return f"놓다가 넘어짐 ({s.get('place_why') or ''})"
    return f"인계 뒤 목표에 놓지 못함 ({s['result']})"


def export_npz(path: Path, rows: List[dict], summary: dict, env: SimEnv) -> None:
    """demo_export 형식 (replay_run.py·demo_replay 가 읽는 키)."""
    N = len(rows)
    q = np.array([r["q"] for r in rows])
    tips = env.jm.tips(np.array([r["motor"] for r in rows])) if hasattr(env.jm, "tips") else np.full((N, 2, 3), np.nan)
    meta = {"format_version": DE.FORMAT_VERSION, "record": f"policy_{summary['name']}", "episode": 1, "result": summary["result"],
            "success": summary["result"] == "auto_success", "start_slot": summary.get("start", "L1"), "goal_slot": summary.get("goal", "R1"), "grip_order": summary.get("grip_order") or "",
            "handover": int(summary.get("handover") or 0), "quality": {"policy_rollout": True, "no_release": False},
            "rate_hz": L.FPS, "duration_s": float(rows[-1]["t"]) if rows else 0.0, "dry_run": True, "meas_valid": False,
            "joints": list(JOINTS), "sides": list(SIDES), "state_names": list(VCAN.STATE_NAMES), "sources": list(DE.SOURCES),
            "units": {"q": "rad URDF", "motor": "rad motor", "pos": "cm robot base", "quat": "xyzw can→base"},
            "robot_ref": env.ref_path.name, "robot_ref_values": {"rest": env.ref.rest, "e90": env.ref.e90},
            "policy_summary": {k: v for k, v in summary.items() if k != "events"}}
    if summary.get("start_xy_cm"):                              # 19-c1: 좌표로 돌린 판 — 좌표 쌍 기록 (19-b) 과 같은 칸
        meta.update({"start_slot": VCAN.PAIR_START, "goal_slot": VCAN.PAIR_GOAL, "can_start_slot": VCAN.PAIR_START, "can_goal_slot": VCAN.PAIR_GOAL,
                     "can_start_xy_cm": list(summary["start_xy_cm"]), "can_goal_xy_cm": list(summary["goal_xy_cm"])})
    rec = {"t": np.array([r["t"] for r in rows]), "q_cmd": q, "q_meas": np.full_like(q, np.nan),
           "motor_cmd": np.array([r["motor"] for r in rows]), "grip": np.array([r["grip"] for r in rows]),
           "grip_deg": np.array([r["grip_deg"] for r in rows]), "tip_cmd": tips,
           "can_pos": np.array([r["can_pos"] for r in rows]), "can_quat": np.array([r["can_quat"] for r in rows]),
           "can_state": np.array([r["can_state"] for r in rows]), "can_source": np.zeros(N)}
    np.savez_compressed(path, meta=np.array(json.dumps(meta, ensure_ascii=False, default=float)), **rec)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", required=True)
    ap.add_argument("--name", default=None)
    ap.add_argument("--ensemble", action="store_true", help="temporal ensembling (기본: 묶음을 차례로)")
    ap.add_argument("--max-s", type=float, default=60.0)
    ap.add_argument("--move-s", type=float, default=3.0)
    ap.add_argument("--no-align", action="store_true", help="가상 판정에서 정렬 조건 (14-l) 을 끈다")
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--start", default="L1"); ap.add_argument("--goal", default="R1")
    ap.add_argument("--pose-jitter-deg", type=float, default=0.0); ap.add_argument("--can-jitter-cm", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0); ap.add_argument("--meas-sim", action="store_true"); ap.add_argument("--strict", action="store_true")
    ap.add_argument("--align-axis-cm", type=float, default=1.5)
    a = ap.parse_args()
    pol = load_policy(a.policy, ensemble=a.ensemble)
    name = a.name or (a.policy.split(":")[0] + ("_ens" if a.ensemble else ""))
    s = rollout(pol, name, max_s=a.max_s, move_s=a.move_s, out_dir=Path(a.out).expanduser(), align=not a.no_align, start=a.start, goal=a.goal,
                pose_jitter_deg=a.pose_jitter_deg, can_jitter_cm=a.can_jitter_cm, seed=a.seed, meas_sim=a.meas_sim, strict=a.strict,
                align_axis_cm=a.align_axis_cm)
    print(json.dumps({k: v for k, v in s.items() if k != "events"}, ensure_ascii=False, default=str))
    print("사건:", s["events"][:12])
    return 0


if __name__ == "__main__":
    sys.exit(main())
