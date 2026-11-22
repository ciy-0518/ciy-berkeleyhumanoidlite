#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""녹화 에피소드 재생 (작업 지시 14 C, 워치독 --replay <에피소드 npz>, 기본 끔).

에피소드 파일은 demo_export.py 가 만든 npz. 저장된 URDF 관절각을 **지금 전원의 robot_ref** 로 모터 값으로 바꿔 목표로 쓴다
(기록 때와 영점이 달라도 같은 자세). 목표는 워치독의 motor.update_targets 로 넣으므로 관절 한계·속도 제한·비틀기 범위·
양팔 충돌 방지·가대 면 제한·발산 감시를 모두 그대로 거친다. 추적 패킷은 명령에 쓰지 않는다.

순서 (phase):
  wait     p 를 기다림. p → 모터를 지금 자세에서 위치 모드로 (워치독 motor.enter_position_from_current) → approach
  approach 지금 명령에서 재생 위치 (처음엔 첫 프레임) 자세로 관절 공간 직선, move_s 초 (≥ 4) 에 걸쳐 (부드러운 시작·끝)
  ready    그 자세에서 멈춤. g → play
  play     기록 시각을 dt × speed 씩 나아가며 그 시각 자세 (20 Hz 사이 선형) 를 목표로, 그리퍼도 그 시각 상태로
  paused   h (HOLD) 로 멈춤. g → 이어서 play (명령이 그 자리라 튀지 않는다)
  done     끝 자세에서 멈춤. h / x / Ctrl+C 는 워치독 그대로
  x (DAMPING) 로 위치 모드가 풀리면 wait 로 돌아가고, 다시 p 를 누르면 멈춘 기록 시각의 자세로 다시 approach 한다.
VR 표시: 워치독 --robot-viz 의 364 패킷은 그대로 (사람 보정이 있으면 그려진다). 같은 곳에 RVC1 (virtual_can.py 형식) 으로
  **기록된 캔 자세** 를 그대로 보낸다 (기준 팔 = 상태가 말하는 팔, 양팔이면 캔에 더 가까운 집게 끝).
--replay-auto (작업 지시 14-b) 는 키 대신 auto_target.AutoRun 이 순서를 잡고, 여기서는 first_pose·begin·advance·finished 만 쓴다."""
from __future__ import annotations

import json
import math
import socket
import sys
from pathlib import Path
from typing import Dict, Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import demo_export as DE  # noqa: E402

MOVE_S_MIN = 4.0           # 작업 지시 14 C §2: 첫 자세로 4 초 이상
READY_TOL_RAD = 0.01       # approach 끝에 명령이 목표에서 이 안이면 ready
RVC_PERIOD = 1.0 / 30.0


class DemoReplay:
    def __init__(self, path, ref, sides, speed: float = 1.0, move_s: float = 5.0, can_target: Optional[str] = None, log=print):
        self.path = Path(path)
        self.ep = DE.load_episode(self.path)
        self.meta = self.ep["meta"]
        if tuple(self.meta["joints"]) != DE.JOINTS or tuple(self.meta["sides"]) != DE.SIDES:
            raise ValueError("에피소드 관절·팔 차례가 다르다")
        self.sides = tuple(sides)
        self.jm = DE.JointMap(ref)
        self.motor = self.jm.array_to_motor(self.ep["q_cmd"])               # (N,2,5) 지금 전원 모터 값
        self.t = np.asarray(self.ep["t"], float)
        self.grip = np.asarray(self.ep["grip"], float)
        self.speed = float(speed)
        self.move_s = max(float(move_s), MOVE_S_MIN)
        self.log = log
        rec = self.meta.get("robot_ref_values") or {}
        self.same_ref = rec.get("rest") == ref.rest and rec.get("e90") == ref.e90
        self.phase = "wait"
        self.tau = 0.0                       # 기록 시각 (초)
        self._ap_start: Optional[Dict[str, Dict[str, float]]] = None
        self._ap_t = 0.0
        self.grip_sent: Optional[Dict[str, int]] = None
        # RVC1 (기록된 캔)
        self.sock = None
        self.addr = None
        self.seq = 0
        self.last_send = -math.inf
        if can_target:
            import virtual_can as VC
            self.VC = VC
            host, _, port = can_target.partition(":")
            self.addr = (host, int(port or 5022))
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setblocking(False)

    # ------------------------------------------------------------------ 궤적
    @property
    def duration(self) -> float:
        return float(self.t[-1])

    def pose_at(self, tau: float) -> Dict[str, Dict[str, float]]:
        out = {}
        for i, s in enumerate(DE.SIDES):
            if s in self.sides:
                out[s] = {j: float(np.interp(tau, self.t, self.motor[:, i, k])) for k, j in enumerate(DE.JOINTS)}
        return out

    def grip_at(self, tau: float) -> Dict[str, int]:
        i = int(np.clip(np.searchsorted(self.t, tau, side="right") - 1, 0, len(self.t) - 1))
        return {"L": int(self.grip[i, 0]), "R": int(self.grip[i, 1])}

    def grip_change_between(self, t0: float, t1: float) -> Optional[float]:
        """작업 지시 14-j: (t0, t1] 안에서 그리퍼 상태가 grip_at(t0) 와 달라지는 첫 프레임 시각 (없으면 None)."""
        g0 = self.grip_at(t0)
        i0 = self.frame_at(t0)
        for i in range(i0 + 1, len(self.t)):
            if self.t[i] > t1 + 1e-9:
                break
            if (int(self.grip[i, 0]), int(self.grip[i, 1])) != (g0["L"], g0["R"]):
                return float(self.t[i])
        return None

    def frame_at(self, tau: float) -> int:
        return int(np.clip(np.searchsorted(self.t, tau, side="right") - 1, 0, len(self.t) - 1))

    # ------------------------------------------------------------------ 키
    def on_position(self, motor) -> None:
        """p: 위치 모드로 들어가 재생 위치 자세로 approach."""
        if motor.position_mode:
            self.log("\n[재생] 이미 위치 모드 — g 재생 / h 멈춤")
            return
        if not motor.enter_position_from_current():
            self.log("\n[재생] 🔴 위치 모드로 못 들어감 (위 [SAFETY] 참고)")
            return
        self._ap_start = {s: dict(motor.command_target[s]) for s in self.sides}
        self._ap_t = 0.0
        self.phase = "approach"
        self.log(f"\n[재생] 기록 {self.tau:.1f} 초 자세로 {self.move_s:.0f} 초에 걸쳐 옮긴다 (멈춤 h / x / Ctrl+C)")

    def on_go(self) -> None:
        if self.phase in ("ready", "paused"):
            self.phase = "play"
            self.log(f"\n[재생] ▶ {self.tau:.1f} / {self.duration:.1f} 초, 배율 {self.speed:g}")
        elif self.phase == "approach":
            self.log("\n[재생] 아직 옮기는 중 — 끝나면 g")
        elif self.phase == "done":
            self.log("\n[재생] 끝났다")
        else:
            self.log("\n[재생] 먼저 p")

    # ------------------------------------------------------------------ 한 걸음
    def step(self, dt: float, motor, hold: bool) -> Optional[Dict[str, Dict[str, float]]]:
        """이번 루프 목표 (모터 값) 또는 None (목표 없음 = 워치독이 지금 명령을 유지)."""
        if not motor.position_mode:
            if self.phase not in ("wait",):
                self.phase = "wait"
                self.log(f"\n[재생] 위치 모드가 풀렸다 — 기록 {self.tau:.1f} 초에서 멈춤. p 로 다시 옮긴 뒤 g")
            return None
        if hold:
            if self.phase == "play":
                self.phase = "paused"
                self.log(f"\n[재생] ⏸ {self.tau:.1f} 초에서 멈춤 — g 로 이어서")
            return None
        if self.phase == "approach":
            self._ap_t += max(dt, 0.0)
            a = min(self._ap_t / self.move_s, 1.0)
            s = a * a * (3.0 - 2.0 * a)                          # 시작·끝 속도 0
            goal = self.pose_at(self.tau)
            tg = {sd: {j: self._ap_start[sd][j] + (goal[sd][j] - self._ap_start[sd][j]) * s for j in goal[sd]} for sd in goal}
            if a >= 1.0:
                err = max(abs(motor.command_target[sd][j] - goal[sd][j]) for sd in goal for j in goal[sd])
                if err <= READY_TOL_RAD:
                    self.phase = "ready"
                    self.log(f"\n[재생] 재생 위치 자세 도착 (차 {math.degrees(err):.2f}°) — g 로 재생 ({self.duration:.1f} 초 × 1/{self.speed:g})")
            return tg
        if self.phase == "play":
            self.tau = min(self.tau + max(dt, 0.0) * self.speed, self.duration)
            if self.tau >= self.duration:
                self.phase = "done"
                self.log(f"\n[재생] ■ 끝 ({self.duration:.1f} 초). 끝 자세에서 멈춤 — h / x / Ctrl+C")
            return self.pose_at(self.tau)
        if self.phase in ("ready", "paused", "done"):
            return self.pose_at(self.tau)
        return None

    # ------------------------------------------------------------------ 자동 순서 (작업 지시 14-b, auto_target.AutoRun 의 목표 원본)
    @property
    def label(self) -> str:
        return f"{self.meta['record']} 에피소드 {self.meta['episode']}"

    @property
    def finished(self) -> bool:
        return self.phase == "done"

    def first_pose(self) -> Dict[str, Dict[str, float]]:
        return self.pose_at(0.0)

    def begin(self) -> None:
        self.tau = 0.0
        self.phase = "play"

    def advance(self, dt: float) -> Dict[str, Dict[str, float]]:
        if self.phase == "play":
            self.tau = min(self.tau + max(dt, 0.0) * self.speed, self.duration)
            if self.tau >= self.duration:
                self.phase = "done"
        return self.pose_at(self.tau)

    def apply_grip(self, gripper, include_done: bool = False) -> None:
        """재생 중에만 그 시각 그리퍼 상태로 (그리퍼 판단 GripDecider 상태를 바꾸고 바뀌면 각도를 보낸다).
        include_done: 끝난 뒤에도 마지막 프레임 상태로 (자동 순서 — 마지막 프레임에서 바뀐 그리퍼도 보낸다)."""
        if gripper is None or self.phase not in (("play", "done") if include_done else ("play",)):
            return
        want = self.grip_at(self.tau)
        before = gripper.state()
        if want != before:
            for h, v in want.items():
                gripper.dec[h].state = int(v)
            gripper.changes += 1
            gripper.link.send_angles(*gripper.angles())
        self.grip_sent = want

    # ------------------------------------------------------------------ VR 캔 (기록 그대로)
    def tick_can(self, now: float, motor) -> None:
        if self.sock is None or now - self.last_send < RVC_PERIOD:
            return
        self.last_send = now
        try:
            VC = self.VC
            i = self.frame_at(self.tau)
            pos = [float(v) for v in self.ep["can_pos"][i]]
            q = [float(v) for v in self.ep["can_quat"][i]]
            st = int(self.ep["can_state"][i])
            tips = self.jm.tips(np.array([[[motor.command_target[s][j] for j in DE.JOINTS] for s in DE.SIDES]]))[0]
            anchor = {1: 0, 2: 1}.get(st, VC.NO_ARM)
            if st == 3:
                c = np.array(pos)
                anchor = int(np.argmin([np.linalg.norm(tips[k] - c) for k in range(2)]))
            tip = [float(v) for v in tips[anchor]] if anchor != VC.NO_ARM else [float("nan")] * 3
            flags = VC.FLAG_ACTIVE if self.phase == "play" else 0
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            pkt = VC.RVC.pack(VC.RVC_MAGIC, self.seq, *pos, *q, *tip, st, flags, anchor, 0, int(self.meta["episode"]) & 0xFFFF, 0, 0, 0, VC.NO_DIST)
            self.sock.sendto(pkt, self.addr)
        except Exception:                                       # 표시용: 실패해도 제어 루프는 그대로
            pass

    def info(self) -> dict:
        m = self.meta
        return {"file": str(self.path), "record": m["record"], "episode": m["episode"], "result": m["result"], "frames": len(self.t),
                "duration_s": self.duration, "speed": self.speed, "move_s": self.move_s, "recorded_ref": m["robot_ref"], "same_ref": self.same_ref}
