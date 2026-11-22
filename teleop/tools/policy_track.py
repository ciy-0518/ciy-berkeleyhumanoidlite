#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""정책 궤적 (작업 지시 15b): replay_run.py --policy 가 자동 순서 (auto_target.AutoRun) 의 play 에 DemoReplay 대신 넣는다.
시스템 파이썬 (numpy 만, torch 없음). 추론은 policy_server.py (~/venvs/learn) 가 127.0.0.1 UDP 로 한다 — 이 파일이 띄우고 끈다.

DemoReplay 와 같은 것: label, meta, ep (can_pos·can_quat — 들어 올림 경유 계산용), speed (1), duration (= 최대 시간), tau, phase,
  first_pose, begin, advance, pose_at, grip_at, grip_change_between, frame_at, apply_grip, tick_can, finished, info.
관측 (20 Hz, tau 기준 — 그리퍼 동기 (grip_wait) 동안은 멈춘다): learn_data 와 같은 꼴
  관절 = 엔코더 meas → URDF (dry-run 은 엔코더가 없어 명령값), 그리퍼 = 보낸 명령 + ServoModel 각 / 닫힘 각,
  캔 = virtual_can (시작 자리 = 사용자가 놓은 자리 --can-start, 출처 2 수동) 을 명령으로 따라간다. 정책에 넣는 출처 값은 서버의 source_obs
  (학습 데이터가 한 값뿐이면 그 값, 지금 0) — 학습에 없던 값은 정규화가 터진다. 정렬 조건 (14-l) 은 켜지 않는다 (실제 캔은 잡혔는데
  가상 캔만 안 잡혀 관측이 틀어지지 않게).
  관측 출처 (작업 지시 17-b, obs_source / replay_run --policy-obs, 기본 encoder = 위 그대로): command 면 실기동에서도 관절 관측을 논리 명령
  (처짐 보정 켜면 보정 전 목표 — dry-run 과 같은 값) 으로 넣는다. 실기 모터가 명령과 2 ~ 4° 차이를 남긴 채 작은 움직임을 따라가지 않아,
  엔코더 관측으로는 묶음마다 같은 출발 묶음을 되풀이하며 맴돌았다 (logs/policy_20261115_233043, REPORT_policy_real1_*.md).
  sag-model (17-e) 이면 관절 관측 = 처짐 모델 실제 (논리 명령) — 학습 --meas-sim 과 같은 식 (잡음만 뺌). command 는 학습보다 처짐 몫만큼
  어긋나 R1 위에서 놓기 자세 앞에 멈추고 R 을 열지 않았다 (policy_20261116_000306, 같은 상태 시뮬레이션: meas 1.7 초에 엶 / command 40 초 안 엶).
  가상 캔 판정 기준 (작업 지시 17-c, can_basis / replay_run --policy-can-basis, 기본 logical = 지금 그대로): encoder 면 실기동에서 가상 캔의
  잡기 · 인계 · 놓기를 엔코더 원값 자세로 판정한다 (집게 끝 오프셋 · 처짐 보정이 실제 팔에 만든 자세). 논리 명령은 오프셋 전이라 실제 집게보다
  R 손 축 3 cm 얕게 판정해 실제로 잡은 인계를 놓침 → fallen 으로 멈췄다 (logs/policy_20261115_234405, REPORT_policy_real2_*.md).
  캔 자세는 정책 관측 (environment_state) 에도 들어가므로 encoder 면 관측 캔도 실제 팔을 따른다.
행동: 매 걸음 관측을 보내고 답 (묶음) 을 받는다 (기다리지 않는다 — 제어 루프 200 Hz 를 막지 않음).
  묶음 모드 (기본, ACT n_action_steps 와 같은 뜻): n 걸음마다 새 묶음으로 바꾼다. 답이 오기 전 (보통 수 ms) 과, 한 걸음 (50 ms) 안에 안 오면
  앞 묶음의 다음 칸으로 이어 간다 (그 걸음을 "늦음" 으로 센다). 늦게 온 답은 다음 경계에서 쓴다. 서버는 밀린 요청 중 가장 새 것만 처리한다.
  섞기 모드 (ensemble): 받은 묶음마다 그 걸음 칸을 지수 가중 평균 (ACT temporal ensembling, 오래된 것 가중 큼, 계수 0.01).
  답이 0.2 초 (DEAD_S) 동안 하나도 없으면 멈춤 이유 "정책 서버 응답 없음" → replay_run 이 그 자리 유지 → 주차.
정책 전용 멈춤 (stop_reason, replay_run 이 보고 유지 → 주차, 끝 코드 4): 응답 없음, 가상 캔 fallen·timeout, NaN·한계 밖 (LIMIT_OVER_DEG) 3 번 연속,
  최대 시간 (max_s, 기본 90). 안전장치가 깎은 양 10° 1 초는 replay_run 이 잰다 (보낸 목표와 명령을 다 아는 곳).
성공 (가상 캔 auto_success) 이면 phase done → 끝 자세 유지 → 놓기 → 들어 올림 → 주차 (재생과 같은 길)."""
from __future__ import annotations

import json
import math
import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import demo_export as DE  # noqa: E402
import gripper_link as GLINK  # noqa: E402

VENV_PY = Path.home() / "venvs/learn/bin/python"
FPS = 20.0
STEP_S = 1.0 / FPS
LATE_S = 0.05
DEAD_S = 0.2
HELLO_TIMEOUT_S = 120.0
LIMIT_OVER_DEG = 5.0          # 한계보다 이만큼 넘게 밖이면 "한계 밖" (조금 밖은 안전장치가 자른다)
BAD_STREAK = 3
ENS_K = 0.01
MAX_S = 90.0
SOURCE_MANUAL = 2.0
OBS_SOURCES = ("encoder", "command", "sag-model")   # 17-b · 17-e 관측 관절 출처
CAN_BASES = ("logical", "encoder", "encoder-nocorr")     # 17-c 가상 캔 판정 팔 자세. 18-e 추가: encoder-nocorr = 엔코더 − 보정 몫 (replay_run 이 can_shift 에 넣는다)


class PolicyTrack:
    dynamic_end = True                       # auto_target: 끝 자세가 실행 중에 정해진다 → 끝에서 들어 올림 길을 다시 계산

    def __init__(self, spec: str, ref, sides, table, *, can_start: str = "L1", can_goal: str = "R1", ensemble: bool = False,
                 max_s: float = MAX_S, limits=None, open_deg=(0, 0), close_deg=(57, 57), dry_run: bool = True, viz_target: Optional[str] = None,
                 server_args: str = "", log=print, obs_source: str = "encoder",
                 can_basis: str = "logical"):
        import virtual_can as VCAN
        import vr_control_watchdog_biarm as W
        self.spec, self.sides, self.log = spec, tuple(sides), log
        self.jm = DE.JointMap(ref)
        self.ref = ref
        self.limits = limits
        self.ensemble, self.max_s, self.dry_run = bool(ensemble), float(max_s), bool(dry_run)
        self.speed = 1.0
        if obs_source not in OBS_SOURCES:
            raise ValueError(f"obs_source 는 {OBS_SOURCES} 중 하나")
        self.obs_source = obs_source         # 17-b: 관측 관절 출처 (encoder = 지금 그대로, command = 논리 명령 — dry-run 과 같은 값)
        self._sag = None                     # 17-e: sag-model = 처짐 모델 실제(논리 명령) — 학습 --meas-sim 과 같은 식 (잡음 없음)
        if obs_source == "sag-model":
            import sag_comp as SAG
            models = {s: W.PR.ArmModel(s) for s in self.sides}          # learn_data.MeasSim 과 같은 구성
            self._sag = SAG.SagModel.load(W.motor_to_urdf_fn(ref, self.sides, models), models)
        if can_basis not in CAN_BASES:
            raise ValueError(f"can_basis 는 {CAN_BASES} 중 하나")
        self.can_shift = None                # 18-e: 가상 캔이 보는 엔코더 자세에서 뺄 관절 몫 {팔: {관절: rad}} (replay_run --carry-lift-cm 이 넣는다, 기본 None = 그대로)
        self.can_basis = can_basis           # 17-c: 가상 캔 판정 팔 자세 (logical = 지금 그대로, encoder = 실기 엔코더 그대로 — dry-run 은 logical)
        self.open_deg = tuple(int(round(v)) for v in open_deg)
        self.close_deg = tuple(int(round(v)) for v in close_deg)
        self.servo = GLINK.ServoModel(*self.open_deg)
        self.eff = {"L": 0, "R": 0}
        self.applied = {"L": 0, "R": 0}
        self.desired = {"L": 0, "R": 0}
        self.pending_t: Optional[float] = None
        can_xy = None                        # 19-c1: --can-start · --can-goal 을 "x,y" (cm) 로 주면 움직이는 자리 S0 · G0 (19-b) 를 그 좌표로. 이름이면 지금 그대로
        if "," in str(can_start) or "," in str(can_goal):
            by = {x["name"]: x["base_xy_cm"] for x in table.cfg["can_slots"]}
            can_xy = [tuple(float(t) for t in str(v).split(",", 1)) if "," in str(v) else (float(by[str(v).upper()][0]), float(by[str(v).upper()][1])) for v in (can_start, can_goal)]
            can_start, can_goal = VCAN.PAIR_START, VCAN.PAIR_GOAL
        self.vcan = VCAN.VirtualCan(table, can_start, ref, self.sides, W.motor_to_urdf_fn(ref, self.sides), goal=can_goal,
                                    margin_cm=VCAN.GRIP_MARGIN_CM, servo_sign=W.table_servo_signs(ref, self.sides), target=viz_target,
                                    log=lambda s: None, reset_s=3.0, timeout_s=max_s + 5.0, episode_end="release", align=None,
                                    **({"pairs": [(*can_xy[0], *can_xy[1])]} if can_xy is not None else {}))
        self.VCAN = VCAN
        self.can_start, self.can_goal = self.vcan.start_name, self.vcan.goal_name
        self.motor = None                    # bind() — 관측용 (명령·엔코더)
        self.table = table
        self.cond_vec = None                 # 17-a: 서버가 조건 입력 정책이라고 알리면 (hello cond) state 뒤에 붙일 4 값
        self._start_server(server_args)
        self.meta = {"record": f"정책 {self.kind}", "episode": 1, "result": "정책", "start_slot": self.can_start, "goal_slot": self.can_goal,
                     "robot_ref": "", "success": False}
        if can_xy is not None:               # 19-c1: 재생기가 좌표 쌍 기록처럼 읽는다 (캔 놓을 곳 글, 겹침 판정)
            self.meta.update({"can_start_slot": self.can_start, "can_goal_slot": self.can_goal, "can_start_xy_cm": list(can_xy[0]), "can_goal_xy_cm": list(can_xy[1])})
        self.reset()

    # ------------------------------------------------------------------ 서버
    def _start_server(self, extra: str) -> None:
        kind, _, path = self.spec.partition(":")
        if kind not in ("act", "mlp"):
            raise ValueError("--policy 는 act:<pretrained_model 폴더> 또는 mlp:<model.pt>")
        p = Path(path).expanduser()
        if not p.exists():
            raise ValueError(f"체크포인트가 없다: {p}")
        if not VENV_PY.exists():
            raise ValueError(f"학습 환경이 없다: {VENV_PY}")
        cmd = ["nice", "-n", "5", str(VENV_PY), "-u", str(TOOLS / "policy_server.py"), "--policy", f"{kind}:{p}", "--port", "0"] + extra.split()
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        t0 = time.monotonic()
        port = None
        while time.monotonic() - t0 < HELLO_TIMEOUT_S:
            line = self.proc.stdout.readline()
            if not line:
                break
            if line.startswith("PORT "):
                port = int(line.split()[1])
                break
        if port is None:
            self.close()
            raise RuntimeError("정책 서버가 시작하지 못했다 (체크포인트·학습 환경 확인)")
        self.addr = ("127.0.0.1", port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.setblocking(False)
        hello = None
        for _ in range(50):
            self.sock.sendto(b'{"op":"hello"}', self.addr)
            time.sleep(0.1)
            hello = self._recv_one("hello")
            if hello:
                break
        if not hello:
            self.close()
            raise RuntimeError("정책 서버가 hello 에 답하지 않는다")
        self.kind, self.k, self.n_steps = hello["kind"], int(hello["chunk"]), int(hello["n_action_steps"])
        self.first_urdf = np.asarray(hello["first_pose"], float)
        self.server_info = hello
        self.source_obs = hello.get("source_obs")      # 학습 데이터 출처가 한 값뿐이면 그 값을 넣는다 (실제 출처 2 는 CSV 에)
        if hello.get("cond"):                          # 17-a: 시작·목표 자리 판 좌표 (m, learn_data.cond_vector 와 같은 값), zero 면 0
            sl = {x["name"]: x for x in self.table.cfg["can_slots"]}
            v = np.array(list(sl[self.can_start]["base_xy_cm"]) + list(sl[self.can_goal]["base_xy_cm"]), float) / 100.0
            self.cond_vec = np.zeros(4) if hello["cond"] == "zero" else v
        self.sync = False                              # replay_run 이 GripHooks.sync 를 알려 준다 (14-j 그리퍼 동기)
        self.server_start_s = time.monotonic() - t0

    def _recv_one(self, op):
        while True:
            try:
                data, _ = self.sock.recvfrom(65536)
            except (BlockingIOError, OSError):
                return None
            try:
                m = json.loads(data.decode("utf-8"))
            except ValueError:
                continue
            if m.get("op") == op:
                return m
            if m.get("op") == "act":
                self._on_reply(m)

    def close(self) -> None:
        try:
            if getattr(self, "sock", None) is not None:
                self.sock.sendto(b'{"op":"bye"}', self.addr)
        except OSError:
            pass
        p = getattr(self, "proc", None)
        if p is not None and p.poll() is None:
            try:
                p.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                p.terminate()
                try:
                    p.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    p.kill()

    # ------------------------------------------------------------------ 상태
    def reset(self) -> None:
        self.phase = "wait"
        self.tau = 0.0
        self.step_n = -1                    # 마지막으로 보낸 관측 걸음
        self.next_t = 0.0
        self.sent_t: Dict[int, float] = {}  # 걸음 → 보낸 시각 (monotonic)
        self.chunks: Dict[int, np.ndarray] = {}
        self.cur_chunk: Optional[int] = None   # 묶음 모드: 쓰는 묶음의 걸음 번호
        self.action: Optional[np.ndarray] = None
        self.last_reply_t: Optional[float] = None
        self.waiting_since: Optional[float] = None
        self.stop_reason = ""
        self.bad = 0
        self.late = 0
        self.lat_ms: List[float] = []
        self.rows = 0
        self.last = {"obs_t": "", "act_t": "", "lat_ms": "", "chunk": "", "idx": "", "late": "0"}
        self.result = None
        self._fresh = False

    def bind(self, motor) -> None:
        """관측에 쓸 모터 (replay_run 의 view: 처짐 보정이면 논리 명령)."""
        self.motor = motor

    @property
    def label(self) -> str:
        return f"정책 {self.kind} ({Path(self.spec.partition(':')[2]).name}) {self.can_start}→{self.can_goal}"

    @property
    def duration(self) -> float:
        return self.max_s

    @property
    def finished(self) -> bool:
        return self.phase == "done"

    @property
    def ep(self) -> dict:
        return {"can_pos": [self.vcan.pos.tolist()], "can_quat": [self.vcan.q.tolist()]}

    def info(self) -> dict:
        return {"policy": self.spec, "kind": self.kind, "chunk": self.k, "n_action_steps": self.n_steps, "ensemble": self.ensemble,
                "can_start": self.can_start, "can_goal": self.can_goal, "max_s": self.max_s, "server_start_s": round(self.server_start_s, 1),
                **({"obs_source": self.obs_source} if self.obs_source != "encoder" else {}),
                **({"can_basis": self.can_basis} if self.can_basis != "logical" else {})}

    # ------------------------------------------------------------------ 자세
    def _to_motor(self, q_urdf: np.ndarray) -> Dict[str, Dict[str, float]]:
        out = {}
        for i, s in enumerate(DE.SIDES):
            if s in self.sides:
                out[s] = self.jm.to_motor(s, dict(zip(DE.JOINTS, (float(v) for v in q_urdf[i]))))
        return out

    def first_pose(self) -> Dict[str, Dict[str, float]]:
        return self._to_motor(self.first_urdf)

    def pose_at(self, tau: float) -> Dict[str, Dict[str, float]]:
        if self.action is None:
            return self._hold_pose()
        return self._to_motor(self.action[:10].reshape(2, 5))

    def _hold_pose(self):
        if self.motor is not None:
            return {s: dict(self.motor.command_target[s]) for s in self.sides}
        return self.first_pose()

    def frame_at(self, tau: float) -> int:
        return max(self.step_n, 0)

    # ------------------------------------------------------------------ 관측
    def _q_obs(self) -> np.ndarray:
        m = self.motor
        out = np.zeros((2, 5))
        for i, s in enumerate(DE.SIDES):
            src = m.command_target[s]
            if self._sag is not None:                   # 17-e (켤 때만): 학습 meas-sim 꼴. dry-run 에서도 같은 식
                src = self._sag.actual(s, {j: float(src[j]) for j in DE.JOINTS})
            elif not self.dry_run and self.obs_source == "encoder":
                meas = getattr(m, "last_measured", {}).get(s, {})
                if meas and all(math.isfinite(float(meas.get(j, float("nan")))) for j in DE.JOINTS):
                    src = meas
            u = self.jm.to_urdf(s, {j: float(src[j]) for j in DE.JOINTS})
            out[i] = [u[j] for j in DE.JOINTS]
        return out

    def _grip_frac(self, now) -> np.ndarray:
        a = self.servo.angles(now)
        return np.array([(a[i] - self.open_deg[i]) / max(self.close_deg[i] - self.open_deg[i], 1) for i in range(2)], float)

    def observation(self, now):
        state = np.concatenate([self._q_obs().reshape(10), self._grip_frac(now)])
        if self.cond_vec is not None:                  # 17-a (조건 입력 정책만)
            state = np.concatenate([state, self.cond_vec])
        src = SOURCE_MANUAL if self.source_obs is None else float(self.source_obs)
        env = np.concatenate([self.vcan.pos / 100.0, self.vcan.q, [src]])
        return state, env

    # ------------------------------------------------------------------ 한 걸음
    def begin(self) -> None:
        self.reset()
        self.phase = "play"
        self.vcan.new_episode(time.monotonic())  # 사용자가 이미 캔을 놓았다 → 바로 시작 (dry-run 의 "자리에 집게가 겹치면 기다림" 은 쓰지 않는다)

    def _send(self, now) -> None:
        self.step_n += 1
        state, env = self.observation(now)
        msg = {"op": "act", "id": self.step_n, "state": [round(float(v), 6) for v in state], "env": [round(float(v), 6) for v in env]}
        try:
            self.sock.sendto(json.dumps(msg).encode("utf-8"), self.addr)
        except OSError:
            pass
        self.sent_t[self.step_n] = now
        if self.waiting_since is None:
            self.waiting_since = now
        self.last.update(obs_t=f"{now:.3f}")

    def _on_reply(self, m) -> None:
        i = int(m.get("id", -1))
        if i not in self.sent_t:
            return
        now = time.monotonic()
        c = np.asarray(m["chunk"], float)
        self.chunks[i] = c
        self.last_reply_t = now
        self.waiting_since = None
        self._fresh = True
        lat = (now - self.sent_t[i]) * 1000.0
        self.lat_ms.append(lat)
        self.last.update(act_t=f"{now:.3f}", lat_ms=f"{lat:.1f}")
        for old in [k for k in self.chunks if k < i - self.k]:
            del self.chunks[old]
        for old in [k for k in self.sent_t if k < i - self.k]:
            del self.sent_t[old]

    def _pick(self, now) -> None:
        """지금 걸음 (step_n) 의 행동을 고른다."""
        n = self.step_n
        if self.ensemble:
            preds = [(m, c[n - m]) for m, c in sorted(self.chunks.items()) if 0 <= n - m < len(c)]
            if preds:
                w = np.exp(-ENS_K * np.arange(len(preds)))
                a = (w[:, None] * np.array([p for _, p in preds])).sum(0) / w.sum()
                self._set_action(a, preds[-1][0], n - preds[-1][0])
            return
        boundary = self.cur_chunk is None or n - self.cur_chunk >= self.n_steps
        newest = max(self.chunks) if self.chunks else None
        if boundary and newest is not None and (self.cur_chunk is None or newest > self.cur_chunk):
            self.cur_chunk = newest                # 경계: 받은 것 중 가장 새 묶음 (제때 왔으면 이 걸음 것, 늦었으면 그 관측 걸음부터 센 칸)
        if self.cur_chunk is not None and self.cur_chunk in self.chunks:
            c = self.chunks[self.cur_chunk]
            idx = min(n - self.cur_chunk, len(c) - 1)
            self._set_action(c[idx], self.cur_chunk, idx)

    def _set_action(self, a, chunk_no, idx) -> None:
        a = np.asarray(a, float)
        bad = not np.all(np.isfinite(a))
        if not bad and self.limits is not None:
            mot = self._to_motor(a[:10].reshape(2, 5))
            over = math.radians(LIMIT_OVER_DEG)
            bad = any(not (self.limits[s][j][0] - over <= mot[s][j] <= self.limits[s][j][1] + over) for s in self.sides for j in DE.JOINTS)
        if bad:
            self.bad += 1
            if self.bad >= BAD_STREAK and not self.stop_reason:
                self.stop_reason = f"정책 출력 NaN·한계 밖 {BAD_STREAK} 번 연속"
            return
        self.bad = 0
        self.action = a
        self.last.update(chunk=str(chunk_no), idx=str(idx))
        want = {"L": int(a[10] > 0.5), "R": int(a[11] > 0.5)}
        if want != self.desired:
            self.desired = want
            self.pending_t = None

    def advance(self, dt: float) -> Dict[str, Dict[str, float]]:
        now = time.monotonic()
        self._recv_one("__none__")
        if self.phase == "play":
            self.tau += max(dt, 0.0)
            while self.tau + 1e-9 >= self.next_t:
                if self.step_n >= 0 and self.step_n not in self.chunks:     # 앞 걸음 답이 한 걸음 (50 ms) 안에 안 왔다 → 늦음
                    self.late += 1
                    self.last["late"] = str(self.late)
                self.next_t += STEP_S
                self._send(now)
                self.rows += 1
                self._pick(now)                  # 이 걸음 답이 오기 전에는 앞 묶음의 다음 칸 (이어 감)
            if self._fresh:                      # 새 답 → 다시 고른다 (묶음 모드는 경계 걸음에서만 새 묶음)
                self._fresh = False
                self._pick(now)
            if self.waiting_since is not None and now - self.waiting_since >= DEAD_S and not self.stop_reason:
                self.stop_reason = f"정책 서버 응답 없음 ({DEAD_S:.1f} 초)"
            if self.tau >= self.max_s and not self.stop_reason:
                self.stop_reason = f"정책 최대 시간 {self.max_s:.0f} 초"
        return self.pose_at(self.tau)

    # ------------------------------------------------------------------ 그리퍼
    def grip_at(self, tau: float) -> Dict[str, int]:
        if self.pending_t is not None and tau >= self.pending_t:
            return dict(self.desired)
        return dict(self.applied)

    def grip_change_between(self, t0: float, t1: float) -> Optional[float]:
        if self.phase != "play" or self.desired == self.applied:
            return None
        self.pending_t = t0
        return t0

    def apply_grip(self, gripper, include_done: bool = False) -> None:
        if self.phase not in (("play", "done") if include_done else ("play",)):
            return
        if self.sync and self.pending_t is None:
            return                               # 동기: 자동 순서가 grip_wait 를 시작할 때 (grip_change_between) 만 바꾼다
        want = dict(self.desired)
        if want == self.applied:
            return
        self.applied = want
        self.pending_t = None
        now = time.monotonic()
        self.servo.command(now, self.close_deg[0] if want["L"] else self.open_deg[0], self.close_deg[1] if want["R"] else self.open_deg[1])
        if gripper is not None and want != gripper.state():
            for h, v in want.items():
                gripper.dec[h].state = int(v)
            gripper.changes += 1
            gripper.link.send_angles(*gripper.angles())

    def state(self) -> Dict[str, int]:
        """virtual_can 이 읽는 그리퍼 상태: 다 닫힘 1 / 다 열림 0 / 움직이는 중 앞 값 (TimedVirtualGrip 과 같은 뜻)."""
        a = self.servo.angles(time.monotonic())
        for i, h in enumerate("LR"):
            if a[i] == self.close_deg[i]:
                self.eff[h] = 1
            elif a[i] == self.open_deg[i]:
                self.eff[h] = 0
        return dict(self.eff)

    # ------------------------------------------------------------------ 캔
    def tick_can(self, now: float, motor) -> None:
        self._recv_one("__none__")               # 답은 매 루프 읽는다 (grip_wait 처럼 advance 가 멈춘 동안 지연이 부풀지 않게)
        running = self.phase == "play"
        self.vcan.tick(now, self._can_pose(motor), self, None, running=running)
        if running and self.vcan.results and self.result is None:
            self.result = dict(self.vcan.results[0])
            r = self.result.get("result")
            self.meta["result"] = r
            if r == "auto_success":
                self.phase = "done"
                self.log(f"\n[정책] ✅ 가상 캔: 목표 {self.can_goal} 에 놓고 손을 뺐다 ({self.tau:.1f} 초)")
            elif not self.stop_reason:
                self.stop_reason = f"가상 캔 {r}" + (f" ({self.result.get('place_why')})" if self.result.get("place_why") else "")

    def _can_pose(self, motor):
        """17-c: 가상 캔이 잡기 · 놓기를 판정하는 팔 자세 (모터 값). replay_run 은 SagView 를 넘긴다 (command_target = 논리 명령).
        logical = 그 논리 명령 (지금 그대로). encoder = 실기 엔코더 원값 (오프셋 · 처짐 보정이 실제로 만든 자세, MeasShift 의 오프셋 빼기 없음).
        dry-run 이나 엔코더 값이 없으면 논리 명령."""
        if self.can_basis in ("encoder", "encoder-nocorr") and not self.dry_run:
            raw = getattr(motor, "_inner", motor)
            meas = getattr(raw, "last_measured", None) or {}
            if all(s in meas and all(math.isfinite(float(meas[s].get(j, float("nan")))) for j in DE.JOINTS) for s in self.sides):
                if self.can_shift is not None:         # 18-e: 옮기는 동안 든 몫을 뺀 자세 (가상 캔 · 정책의 캔 관측은 들기 전과 같게)
                    return {s: {j: float(meas[s][j]) - float(self.can_shift.get(s, {}).get(j, 0.0)) for j in DE.JOINTS} for s in self.sides}
                return {s: {j: float(meas[s][j]) for j in DE.JOINTS} for s in self.sides}
        return motor.command_target

    def csv_row(self) -> List[str]:
        L = self.last
        return [L["obs_t"], L["act_t"], L["lat_ms"], L["chunk"], L["idx"], L["late"], self.stop_reason,
                VCANNAME(self)]

    def summary(self) -> dict:
        lat = np.asarray(self.lat_ms) if self.lat_ms else np.zeros(1)
        return {"steps": self.step_n + 1, "late": self.late, "lat_ms_median": round(float(np.median(lat)), 2),
                "lat_ms_max": round(float(lat.max()), 2), "stop_reason": self.stop_reason,
                "can_result": (self.result or {}).get("result"), "can_events": [(round(t, 2), k) for t, k, _ in self.vcan.events][-12:]}


def VCANNAME(track) -> str:
    return track.VCAN.STATE_NAMES[track.vcan.state]


CSV_COLUMNS = ["pol_obs_t", "pol_act_t", "pol_latency_ms", "pol_chunk", "pol_chunk_idx", "pol_late", "pol_stop", "pol_can_state"]
