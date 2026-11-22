#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""사람 입력이 아닌 목표 (녹화 재생 --replay, 나중의 --policy) 를 키보드 없이 돌리는 공통 부품 (작업 지시 14-b).

Beeper      로봇 PC 에서 삐 소리 패턴 (paplay → aplay, 둘 다 없으면 화면만). 출력 장치는 PULSE_SINK 환경 변수로 고른다.
            --cue-sound 가 있으면 서버 PC cue_player 로 같은 뜻의 소리 신호도 같이 보낸다 (새 음성 문구는 쓰지 않는다)
StallGuard  명령과 실제 관절각 차이 (meas − cmd) 가 문턱을 넘은 채 이어지면 알린다 (--stall-stop). 문턱은 실기동 로그에서 정했다
AutoRun     시작 안내 → 5 초 → 지금 자세에서 모터 켬 → 첫 자세로 → 캔 놓을 시간 → 재생 → 끝 자세 유지 → 주차로 복귀 → (다음 회차) → 감쇠
EndGesture  VR 실기동 종료 동작: 로봇 명령의 두 집게 끝을 주차 (모터를 켠 자세) 근처로 내리고 가만히 (--auto-end)

AutoRun 이 목표 원본 (track) 에 바라는 것 — DemoReplay 가 갖췄고, 정책 실행도 같은 이름으로 갖추면 된다:
  label, first_pose() → {side: {joint: 모터 값}}, begin(), advance(dt) → 같은 꼴, finished, apply_grip(gripper), tick_can(now, motor)
목표는 워치독이 motor.update_targets 로 넣으므로 관절 한계·속도 제한·비틀기 범위·양팔 충돌 방지·가대 면 제한·발산 감시를 모두 그대로 거친다."""
from __future__ import annotations

import math
import queue
import shutil
import subprocess
import threading
import time
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "elbow_roll")

# --------------------------------------------------------------------------- 소리
BEEP_RATE = 22050
# 이름 → (음 [(Hz, ms)], 0 Hz 는 쉼), 서버 PC cue_player 소리 이름, 뜻)
BEEP_PATTERNS = {
    "start": ([(660, 120), (0, 100), (660, 120)], "ready", "자동 순서 시작 — 5 초 뒤 모터를 켠다"),
    "motor": ([(440, 500)], "go", "모터 켬 (지금 자세 유지)"),
    "place": ([(523, 150), (0, 60), (659, 150), (0, 60), (784, 150)], "learn", "캔 놓을 시간 시작"),
    "tick": ([(880, 80)], "tick", "남은 초 (마지막 5 초, 1 초마다)"),
    "play": ([(988, 500)], "go", "재생 시작"),
    "end": ([(784, 200), (0, 60), (523, 200)], "stop", "재생 끝 → 주차로 복귀"),
    "done": ([(784, 300), (0, 80), (659, 300), (0, 80), (523, 300)], "done", "모두 끝, 힘 뺌 (감쇠)"),
    "alarm": ([(1319, 90), (0, 60)] * 6, "alarm", "비상 멈춤 (걸림·발산·위치 모드 풀림·h) → 감쇠"),
    "gesture": ([(392, 300), (0, 150), (392, 300)], "switch", "종료 동작 인식 → 주차로 복귀"),
}


def beep_pcm(name: str, volume: float = 0.4) -> bytes:
    """패턴 → s16le 모노 PCM (음 끝 5 ms 는 줄여 딸깍 소리를 막는다)."""
    out = []
    for hz, ms in BEEP_PATTERNS[name][0]:
        n = int(BEEP_RATE * ms / 1000)
        if hz <= 0:
            out.append(np.zeros(n))
            continue
        x = np.sin(2 * math.pi * hz * np.arange(n) / BEEP_RATE)
        k = min(int(BEEP_RATE * 0.005), n // 2)
        if k:
            ramp = np.linspace(0.0, 1.0, k)
            x[:k] *= ramp
            x[-k:] *= ramp[::-1]
        out.append(x)
    pcm = np.concatenate(out + [np.zeros(int(BEEP_RATE * 0.05))]) * volume
    return (pcm * 32767).astype("<i2").tobytes()


def local_player() -> Optional[List[str]]:
    """지금 로봇 PC 에서 되는 재생 명령 (raw s16le 모노를 표준 입력으로)."""
    if shutil.which("paplay"):
        return ["paplay", "--raw", f"--rate={BEEP_RATE}", "--channels=1", "--format=s16le"]
    if shutil.which("aplay"):
        return ["aplay", "-q", "-t", "raw", "-f", "S16_LE", "-r", str(BEEP_RATE), "-c", "1"]
    return None


class Beeper:
    """삐 소리. 재생은 별도 스레드에서 차례로 (제어 루프를 막지 않는다). 실패해도 진행은 그대로."""

    def __init__(self, local: bool = True, cue=None, log=print, extra_args: Sequence[str] = ()):
        self.cue = cue                      # 워치독 CueSound (없거나 sock 이 None 이면 보내지 않음)
        self.log = log
        self.cmd = (local_player() if local else None)
        if self.cmd is not None:
            self.cmd = self.cmd + list(extra_args)
        self.history: List[tuple] = []      # (monotonic, 이름) — 시험 표용
        self.error = None
        self._q: "queue.Queue" = queue.Queue()
        self._pcm = {n: beep_pcm(n) for n in BEEP_PATTERNS} if self.cmd else {}
        if self.cmd:
            threading.Thread(target=self._run, daemon=True).start()

    @property
    def where(self) -> str:
        local = self.cmd[0] if self.cmd else "없음"
        server = "서버 PC 도" if (self.cue is not None and getattr(self.cue, "sock", None) is not None) else "서버 PC 없음"
        return f"로봇 PC {local}, {server}"

    def play(self, name: str, text: str = "", left: Optional[float] = None) -> None:
        self.history.append((time.monotonic(), name))
        if self.cmd:
            self._q.put(name)
        if self.cue is not None:
            self.cue.send(BEEP_PATTERNS[name][1], text=text or BEEP_PATTERNS[name][2], left=left)

    def _run(self) -> None:
        while True:
            name = self._q.get()
            try:
                subprocess.run(self.cmd, input=self._pcm[name], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=5, check=True)
            except (OSError, subprocess.SubprocessError) as exc:
                if self.error is None:
                    self.log(f"\n[소리] 🔴 로봇 PC 소리를 못 낸다: {exc} — 화면 안내만 (서버 PC 소리는 그대로)")
                self.error = str(exc)


# --------------------------------------------------------------------------- 걸림 감시
# 근거: 실기동 로그 51 개 (2026-09-24 ~ 10-16, live_*·hand_*·check_* …, POS 줄 24 만) 에서 |meas − cmd| 가 문턱을 넘은 채 이어진 최장 시간
#   움직이는 중: 20° 넘게 → 어깨 앞뒤 0.73 s, 어깨 옆 0.67 s / 16° 넘게 → 비틀기 0.63 s, 팔꿈치 0.47 s, 손목 0.69 s
#   명령이 멈춘 때 (0.5 초 동안 명령 속도 < 0.1 rad/s): 10° 넘은 적 없음 (8° 넘게 0.96 s, 어깨 앞뒤 중력 처짐)
# 지속 시간을 정상 최장의 2 배 이상으로 잡았다 (보고서 표).
STALL_MOVE = {"shoulder_pitch": (20.0, 1.5), "shoulder_roll": (20.0, 1.5), "shoulder_yaw": (16.0, 1.5),
              "elbow_pitch": (16.0, 1.0), "elbow_roll": (16.0, 1.5)}       # 관절 → (도, 초)
STALL_STILL = (12.0, 1.0)            # 명령이 멈춘 때 모든 관절 (도, 초)
STILL_SPEED = 0.1                    # rad/s
STILL_WINDOW_S = 0.5
STALL_DAMP_AFTER_S = 0.5             # 걸림을 잡으면 실제 자세에 멈춘 채 이만큼 있다가 감쇠


class StallGuard:
    """update() 를 루프마다 (send_targets 뒤) 부른다. 걸리면 dict (side, joint, err_deg, dur_s, kind) 를 한 번 돌려준다."""

    def __init__(self, sides: Sequence[str], move: Optional[Dict[str, tuple]] = None, still: tuple = STALL_STILL):
        self.sides = tuple(sides)
        self.move = {j: (math.radians(d), s) for j, (d, s) in (move or STALL_MOVE).items()}
        self.still = (math.radians(still[0]), still[1])
        self.level = 0.0                    # 지금 가장 큰 (차이 / 움직이는 중 문턱) — 로그용
        self.hits = 0
        self.reset()

    def reset(self) -> None:
        self._over: Dict[tuple, Optional[float]] = {}
        self._prev: Dict[str, Optional[tuple]] = {s: None for s in self.sides}
        self._moved_at: Dict[str, float] = {s: -math.inf for s in self.sides}
        self.level = 0.0

    def update(self, now: float, cmd, meas) -> Optional[dict]:
        hit = None
        level = 0.0
        for s in self.sides:
            c = [float(cmd[s][j]) for j in JOINTS]
            prev = self._prev[s]
            if prev is None:
                self._moved_at[s] = now
            else:
                dt = now - prev[0]
                if dt > 1e-4 and max(abs(a - b) for a, b in zip(c, prev[1])) / dt > STILL_SPEED:
                    self._moved_at[s] = now
            self._prev[s] = (now, c)
            still = now - self._moved_at[s] >= STILL_WINDOW_S
            for k, j in enumerate(JOINTS):
                m = float(meas[s][j])
                if not math.isfinite(m):
                    continue
                e = abs(m - c[k])
                th, hold = self.move[j]
                level = max(level, e / th)
                for kind, (th_k, hold_k) in (("move", (th, hold)), ("still", self.still if still else (math.inf, 0.0))):
                    key = (s, j, kind)
                    if e > th_k:
                        t0 = self._over.get(key)
                        if t0 is None:
                            self._over[key] = now
                        elif now - t0 >= hold_k and hit is None:
                            hit = {"side": s, "joint": j, "err_deg": math.degrees(m - c[k]), "dur_s": now - t0, "kind": kind,
                                   "th_deg": math.degrees(th_k)}
                    else:
                        self._over[key] = None
        self.level = level
        if hit is not None:
            self.hits += 1
            self._over = {}
        return hit


def hold_at_measured(motor) -> None:
    """명령을 실제 관절각 (관절 한계 안) 으로 — 걸린 곳을 더 밀지 않는다. 실제 값이 없는 관절은 그대로."""
    for s in motor.sides:
        for j in JOINTS:
            m = float(motor.last_measured[s][j])
            if not math.isfinite(m):
                continue
            lo, hi = motor.urdf_limits[s][j]
            v = min(max(m, lo), hi)
            motor.command_target[s][j] = v
            motor.previous_target[s][j] = v
    motor.reset_velocity()


def stall_text(hit: dict) -> str:
    kind = "움직이는 중" if hit["kind"] == "move" else "명령이 멈춘 때"
    return (f"{hit['side']}.{hit['joint']} 실제 − 명령 {hit['err_deg']:+.1f}° 가 {hit['th_deg']:.0f}° 넘게 {hit['dur_s']:.1f} 초 ({kind})")


# --------------------------------------------------------------------------- 자동 순서
READY_TOL_RAD = 0.01        # 첫 자세 도착 판정
READY_EXTRA_S = 3.0         # 이동 시간이 지나고도 이만큼 못 가면 중단 (가대 면 제한 등에 막힘)
RETURN_TOL_RAD = 0.02       # 주차 복귀 판정
RETURN_STUCK_S = 1.0        # 이동 시간이 지난 뒤 명령이 이만큼 그대로면 (판 위 여유에 막힘) 거기서 끝낸 것으로
LEAD_S = 5.0
SETTLE_S = 1.0              # 모터를 켠 뒤 그 자세로 머무는 시간
FINAL_HOLD_S = 2.0          # 재생 끝 자세 유지
GRIP_WARM_TIMEOUT_S = 10.0  # 14-h 그리퍼 워밍업 (첫 자세에서) 을 기다리는 최대 (14-i 한 서보씩 + 도착 뒤 0.4 초 → 보통 5.2 초)
RELEASE_TIMEOUT_S = 5.0     # 14-h 끝에서 놓기: 다 열릴 때까지 기다리는 최대 (14-i: 서보 하나 1.3 초, 둘 2.6 초 + 앞 명령)
PLAN_WAIT_S = 10.0          # 14-h 그 뒤 들어 올림·경유 계산을 더 기다리는 최대 (보통 재생 중에 끝난다)
GRIP_SYNC_MAX_S = 4.0       # 14-j 그리퍼 동기: 한 번 바뀐 것을 이만큼 넘게 기다리면 걸림으로 보고 비상 멈춤 (보통 1.3 ~ 2.7 초)
LIFT_S = 2.0                # 14-h 놓은 뒤 집게 끝을 수직으로 들어 올리는 시간 (경유 자세 구간도 최소 이만큼)
LIFT_RATE = 0.5             # 14-h 경유 구간: 가장 많이 움직이는 관절이 이 속도 (rad/s) 를 넘지 않게 시간을 늘린다
USER_HOLD_S = 0.5           # 작업 지시 14-e 사용자 멈춤: 그 자리에 이만큼 둔 뒤 주차
TICK_LAST_S = 5


def _smooth(a: float) -> float:
    a = min(max(a, 0.0), 1.0)
    return a * a * (3.0 - 2.0 * a)


def _lerp(a, b, s):
    return {sd: {j: a[sd][j] + (b[sd][j] - a[sd][j]) * s for j in b[sd]} for sd in b}


def _err(motor, goal) -> float:
    return max(abs(motor.command_target[s][j] - goal[s][j]) for s in goal for j in goal[s])


class AutoRun:
    """회차 = (첫 자세로 → 캔 놓을 시간 → 재생 → 끝 자세 유지 → 주차로). 첫 회차는 첫 자세에서, 다음 회차부터는 주차 자세에서 캔 놓을 시간을 준다.
    step() 은 매 루프 한 번, 이번 루프의 목표 (모터 값) 또는 None 을 돌려준다. 끝나면 finished, exit_code (0 = 다 끝, 3 = 비상 멈춤)."""

    def __init__(self, tracks: Sequence, beeper: Beeper, move_s: float, wait_s: float, lead_s: float = LEAD_S, log=print,
                 vr_guard: Optional["VrLink"] = None, stop_keys: str = "키 h / x / Ctrl+C 도 된다", grip_hooks=None):
        self.tracks = list(tracks)
        self.stop_keys = stop_keys          # 시작 문구의 멈춤 안내 (재생기는 "아무 키나")
        self.block_text = None              # 14-m: (motor, goal) → "무엇에 막혔나" 글. None 이면 예전 글
        # 14-h (재생기가 준다, 없으면 예전 그대로): warmup_begin/done (첫 자세에서 그리퍼 워밍업), release_begin/done (끝에서 놓기),
        # lift_pose(자세) → 집게 끝을 수직으로 든 자세 또는 None
        self.hooks = grip_hooks
        # 14-j 그리퍼 동기 (hooks.sync 가 True 일 때): 재생 중 그리퍼가 바뀌는 프레임에서 재생 시간을 멈추고 팔을 그 자세에 둔 채
        # hooks.sync_done 이 될 때까지 (보드가 그 변화를 끝냄 + hooks.hold_s) 기다린다. 정책 실행도 같은 훅을 쓴다
        self.grip_wait_total = 0.0
        self.grip_waits = 0
        self._warmed = False
        self._releasing = False
        self._release_said = False
        self.lift_goal = None
        self.vr = vr_guard                  # 작업 지시 14-c --vr-guard: 살아 있어야 시작, 끊기면 유지 → 5 초 → 주차 → 감쇠
        self._alive_since = None
        self._vr_saved = None
        self._vr_since = None
        self._vr_pose = None
        self._park_why = "VR 끊김으로"     # 주차 뒤 감쇠할 때 화면 문구 (사용자 멈춤이면 바뀐다)
        self.beep = beeper
        self.move_s = float(move_s)
        self.wait_s = float(wait_s)
        self.lead_s = float(lead_s)
        self.log = log
        self.round = 0
        self.stage = "lead"
        self.t_stage: Optional[float] = None
        self.finished = False
        self.exit_code = 0
        self.reason = ""
        self.park = None
        self.goal = None
        self._from = None
        self._ticked = set()
        self._last_cmd = None
        self._still_since = None
        self.timeline: List[tuple] = []         # (monotonic, 회차, 단계) — 시험 표용

    @property
    def track(self):
        return self.tracks[min(self.round, len(self.tracks) - 1)]

    def _go(self, now: float, stage: str) -> None:
        self.stage, self.t_stage = stage, now
        self.timeline.append((now, self.round + 1, stage))
        self._ticked = set()

    def plan_text(self) -> str:
        return " → ".join(t.label for t in self.tracks)

    # -- 비상 멈춤 (걸림, 발산·x 로 위치 모드가 풀림, h)
    def abort(self, now: float, reason: str, motor, measured_ok: bool = True) -> None:
        if self.stage in ("stopping", "end"):
            return
        self.reason = reason
        self.exit_code = 3
        if motor.position_mode and measured_ok:
            hold_at_measured(motor)
        self.beep.play("alarm", text=f"비상 멈춤: {reason}")
        self.log(f"\n[자동] 🔴 비상 멈춤 — {reason}. 그 자리에 멈추고 {STALL_DAMP_AFTER_S:.1f} 초 뒤 힘을 뺀다")
        self._go(now, "stopping")

    def _begin_lead(self, now: float) -> None:
        self._go(now, "lead")
        self.beep.play("start", text=f"자동 재생 {len(self.tracks)} 회차 — {self.lead_s:.0f} 초 뒤 모터를 켭니다")
        self.log(f"\n[자동] 🔵 {len(self.tracks)} 회차: {self.plan_text()}. {self.lead_s:.0f} 초 뒤 지금 자세에서 모터를 켠다 "
                 "(멈춤: 걸림 감시·발산 감시" + (" ·VR 끊김" if self.vr is not None else "") + f" 자동, {self.stop_keys})")

    def request_stop(self, now: float, motor, reason: str = "사용자 멈춤") -> None:
        """작업 지시 14-e: 사용자 멈춤 (키·Ctrl+C). 모터를 켜기 전이면 바로 끝, 켠 뒤면 그 자리 USER_HOLD_S → 주차 → 감쇠 → 끝 (코드 0)."""
        if self.finished or self.stage in ("stopping", "end", "user_hold", "vr_park"):
            return
        self.reason = reason
        if self.t_stage is None or self.stage in ("vr_wait", "lead") or not motor.position_mode:
            self.log(f"\n[자동] {reason} — 모터를 켜기 전이라 그대로 끝낸다")
            self._go(now, "end")
            self.finished = True
            return
        self._vr_pose = {s: dict(motor.command_target[s]) for s in motor.sides}
        self._park_why = reason + " —"
        self.beep.play("end", text=reason)
        self.log(f"\n[자동] ■ {reason} — 그 자리 {USER_HOLD_S:.1f} 초 → 주차로 {VR_PARK_S:.0f} 초 → 감쇠 → 끝")
        self._go(now, "user_hold")

    def step(self, now: float, dt: float, motor, state: dict, measured_ok: bool = True, vr_kind: Optional[str] = None):
        """vr_kind: --vr-guard 일 때 VrLink.update() 결과 (None = 살아 있음)."""
        if self.finished:
            return None
        if self.t_stage is None:
            if self.vr is not None:
                self._go(now, "vr_wait")
                self.log(f"\n[자동] VR 을 기다린다 — 헤드셋을 쓰고 (패스스루) VR Preview 를 켜면, {VR_ALIVE_S:.0f} 초 이어서 들어온 뒤 시작한다. 그전에는 모터를 켜지 않는다")
            else:
                self._begin_lead(now)
        el = now - self.t_stage
        if self.stage == "vr_wait":
            if vr_kind is None:
                self._alive_since = now if self._alive_since is None else self._alive_since
                if now - self._alive_since >= VR_ALIVE_S:
                    self.vr.set_ref(now)
                    self.log(f"\n[자동] VR 들어옴 ({VR_ALIVE_S:.0f} 초) — 머리 기준 자리 저장")
                    self._begin_lead(now)
            else:
                self._alive_since = None
            return None
        if self.stage == "lead":
            if self.vr is not None and vr_kind is not None:      # 모터를 켜기 전에 끊기면 다시 기다린다
                self.log(f"\n[자동] 모터를 켜기 전에 VR 이 끊겼다 ({self.vr.text()}) — 다시 기다린다")
                self._alive_since = None
                self._go(now, "vr_wait")
                return None
            if el >= self.lead_s:
                if not motor.enter_position_from_current():
                    self.reason, self.exit_code = "모터를 켜지 못함 (위 [SAFETY] 참고)", 3
                    self.beep.play("alarm", text=self.reason)
                    self.log(f"\n[자동] 🔴 {self.reason}")
                    self._go(now, "end")
                    self.finished = True
                    return None
                state["hold"] = False
                self.park = {s: dict(motor.command_target[s]) for s in motor.sides}
                self.beep.play("motor")
                self.log("\n[자동] 모터 켬 — 지금 자세 (주차) 를 그대로 유지")
                self._go(now, "settle")
            return None
        if self.stage in ("stopping", "end"):
            if self.stage == "stopping" and el >= STALL_DAMP_AFTER_S:
                motor.enter_damping()
                state["hold"] = True
                self._go(now, "end")
                self.finished = True
            return None
        # 여기부터 모터가 켜진 단계. 위치 모드가 풀렸거나 (발산·x) h 를 눌렀으면 비상 멈춤
        if not motor.position_mode:
            self.abort(now, "위치 모드가 풀렸다 (발산 감시 또는 x)", motor, measured_ok)
            return None
        if state.get("hold"):
            self.abort(now, "h (멈춤)", motor, measured_ok)
            return None
        if self.stage == "user_hold":      # 작업 지시 14-e 사용자 멈춤: 그 자리 → 주차 (아래 vr_park 와 같은 길)
            if el >= USER_HOLD_S:
                self._from = {s: dict(motor.command_target[s]) for s in motor.sides}
                self._go(now, "vr_park")
            return self._vr_pose
        if self.vr is not None:            # 작업 지시 14-c: 끊기면 그 자리 유지 → 5 초 안에 오면 이어서 → 넘으면 주차 → 감쇠 → 종료 3
            if self.stage == "vr_hold":
                if vr_kind is None:
                    st, sel = self._vr_saved
                    self.stage, self.t_stage = st, now - sel
                    self.timeline.append((now, self.round + 1, st))
                    self.log(f"\n[자동] VR 돌아옴 ({now - self._vr_since:.1f} 초) — '{st}' 이어서")
                    el = sel
                elif now - self._vr_since >= VR_GRACE_S:
                    self._from = {s: dict(motor.command_target[s]) for s in motor.sides}
                    self.reason, self.exit_code = f"VR 끊김 {VR_GRACE_S:.0f} 초 ({self.vr.text()})", 3
                    self.log(f"\n[자동] 🔴 VR 이 {VR_GRACE_S:.0f} 초 넘게 끊겼다 — 주차로 {VR_PARK_S:.0f} 초 → 감쇠 → 종료")
                    self._go(now, "vr_park")
                    return self._vr_pose
                else:
                    return self._vr_pose
            elif self.stage != "vr_park" and vr_kind is not None:
                self._vr_saved = (self.stage, el)
                self._vr_since = now
                self._vr_pose = {s: dict(motor.command_target[s]) for s in motor.sides}
                self.beep.play("alarm", text=f"VR 끊김: {self.vr.text()}")
                self.log(f"\n[자동] ⏸ VR 끊김 — {self.vr.text()}. 그 자리 유지, {VR_GRACE_S:.0f} 초 안에 돌아오면 이어서")
                self._go(now, "vr_hold")
                return self._vr_pose
        if self.stage == "vr_park":
            a = min(el / VR_PARK_S, 1.0)
            tg = _lerp(self._from, self.park, _smooth(a))
            if a >= 1.0:
                motor.enter_damping()
                state["hold"] = True
                self.beep.play("done")
                self.log(f"\n[자동] {self._park_why} 주차 뒤 힘을 뺐다 — 종료 (코드 {self.exit_code})")
                self._go(now, "end")
                self.finished = True
                return None
            return tg

        if self.stage == "settle":
            if el >= SETTLE_S:
                self._start_round(now, motor)
            return self.park
        if self.stage == "approach":
            s = _smooth(el / self.move_s)
            tg = _lerp(self._from, self.goal, s)
            if el >= self.move_s:
                if _err(motor, self.goal) <= READY_TOL_RAD:
                    self.log(f"\n[자동] {self.round + 1} 회차 첫 자세 도착")
                    if self.round == 0 and self.hooks is not None and not self._warmed and (not getattr(self.hooks, "skip_warm", False)
                                                                                             or getattr(self.hooks, "open_first", False)):   # 14-z --no-grip-warm, 17-e --grip-open-first
                        self._warmed = True                  # 14-h: 캔 놓을 시간 전에, 판 위에서 한 번
                        self.hooks.warmup_begin(now)
                        self.log("\n[자동] " + ("그리퍼 열기 확인 (17-e) — 다 열리면 캔 놓을 시간" if getattr(self.hooks, "open_first", False)
                                                 else "그리퍼 워밍업 (열림 → 닫힘 → 열림) — 끝나면 캔 놓을 시간"))
                        self._go(now, "grip_warm")
                    elif self.round == 0:
                        self._enter_place(now, motor)
                    else:
                        self._enter_play(now)
                elif el >= self.move_s + READY_EXTRA_S:
                    # 14-m: block_text (재생기가 준다) 가 있으면 실제로 막은 것 (관절 한계 / 판 면 / 양팔 충돌 / 비틀기 범위) 을 쓴다. 없으면 예전 글 그대로
                    why = self.block_text(motor, self.goal) if self.block_text is not None else "가대 면 제한 등에 막힘"
                    self.abort(now, f"첫 자세에 못 감 (남은 차 {math.degrees(_err(motor, self.goal)):.1f}°, {why})", motor, measured_ok)
                    return None
            return tg
        if self.stage == "grip_warm":
            done = self.hooks.warmup_done(now)
            if done or el >= GRIP_WARM_TIMEOUT_S:
                self.log(f"\n[자동] 그리퍼 워밍업 {'끝' if done else '기다림 초과 — 그대로 간다'} ({el:.1f} 초)")
                self._enter_place(now, motor)
            return self.goal
        if self.stage == "place":
            left = self.wait_s - el
            sec = math.ceil(left)
            if 0 < sec <= TICK_LAST_S and sec not in self._ticked:
                self._ticked.add(sec)
                self.beep.play("tick", left=float(sec))
            if left <= 0.0:
                if self.round == 0:
                    self._enter_play(now)
                else:
                    self._enter_approach(now, motor)
            return self.goal
        if self.stage == "play" and self.hooks is not None and getattr(self.hooks, "sync", False):
            nxt = self.track.tau + max(dt, 0.0) * self.track.speed
            tc = self.track.grip_change_between(self.track.tau, min(nxt, self.track.duration))
            if tc is not None:                                 # 14-j: 그 프레임까지만 가고 멈춤
                before = self.track.grip_at(self.track.tau)
                self.track.tau = tc
                self.goal = self.track.pose_at(tc)
                self.hooks.sync_begin(now, before, self.track.grip_at(tc))
                self._go(now, "grip_wait")
                return self.goal
        if self.stage == "grip_wait":
            if self.hooks.sync_done(now):
                self.grip_wait_total += el
                self.grip_waits += 1
                self.stage, self.t_stage = "play", now           # 재생 이어 감 (_go 는 쓰지 않는다: 시각표는 grip_wait 줄로 충분)
                self.timeline.append((now, self.round + 1, "play"))
                return self.goal
            if el >= GRIP_SYNC_MAX_S and not getattr(self.hooks, "pausing", False):   # 14-v: 닫힌 뒤 멈춤 (replay_run --pause-after-close) 중에는 세지 않는다
                self.abort(now, f"그리퍼가 {GRIP_SYNC_MAX_S:.0f} 초 안에 끝나지 않음 (보드 상태 확인)", motor, measured_ok)
                return None
            return self.goal
        if self.stage == "play":
            pose = self.track.advance(dt)
            if self.track.finished:
                self.log(f"\n[자동] {self.round + 1} 회차 재생 끝 — 끝 자세 {FINAL_HOLD_S:.0f} 초 유지")
                self.goal = pose
                self._go(now, "end_hold")
                if getattr(self.track, "dynamic_end", False) and self.hooks is not None and self.park is not None:
                    self.hooks.prepare(self.track, self.park, force=True)     # 15b 정책: 끝 자세가 이제 정해졌다 → 들어 올림·경유 다시 (재생 궤적은 해당 없음)
            return pose
        if self.stage == "end_hold":
            if el >= FINAL_HOLD_S:
                self.beep.play("end")
                if self.hooks is not None:                                       # 14-h: 쥔 것을 놓는다 (없으면 경로 계산만 기다림)
                    self._releasing = self.hooks.release_begin(now)
                    if self._releasing:
                        self.log("\n[자동] 끝에서 놓기 — 닫힌 그리퍼를 연다 (다 열릴 때까지)")
                    self._go(now, "release")
                else:
                    self._enter_return(now, motor)
            return self.goal
        if self.stage == "release":
            done = (not self._releasing) or self.hooks.release_done(now)
            if self._releasing and (done or el >= RELEASE_TIMEOUT_S) and not self._release_said:
                self._release_said = True
                self.log(f"\n[자동] 그리퍼 {'다 열림' if done else '열림 확인 못 함 (기다림 초과)'} ({el:.1f} 초)")
            ready = self.hooks.plan_ready()                    # 들어 올림·경유 계산 (재생 시작 때 따로 시작했다)
            if (done or el >= RELEASE_TIMEOUT_S) and (ready or el >= RELEASE_TIMEOUT_S + PLAN_WAIT_S):
                if not ready:
                    self.log("\n[자동] 🔴 주차 길 계산이 늦다 — 들어 올리지 않고 주차로")
                self._release_said = False
                self._enter_lift(now, motor) if ready else self._enter_return(now, motor)
            return self.goal
        if self.stage == "lift":
            goal = self.lift_goal[self._lift_i]
            tg = _lerp(self._from, goal, _smooth(el / self._lift_dur))
            if el >= self._lift_dur:
                self._lift_i += 1
                if self._lift_i >= len(self.lift_goal):
                    self._enter_return(now, motor)
                else:
                    self._lift_seg(now, motor)
            return tg
        if self.stage == "return":
            tg = _lerp(self._from, self.park, _smooth(el / self.move_s))
            if el >= self.move_s:
                cur = [motor.command_target[s][j] for s in motor.sides for j in JOINTS]
                if self._last_cmd is None or max(abs(a - b) for a, b in zip(cur, self._last_cmd)) > 1e-5:
                    self._last_cmd, self._still_since = cur, now
                err = _err(motor, self.park)
                if err <= RETURN_TOL_RAD or now - self._still_since >= RETURN_STUCK_S:
                    how = "도착" if err <= RETURN_TOL_RAD else f"판 위 여유에서 멈춤 (남은 차 {math.degrees(err):.1f}°)"
                    self.log(f"\n[자동] 주차 복귀 {how}")
                    self.round += 1
                    if self.round >= len(self.tracks):
                        self.round = len(self.tracks) - 1
                        motor.enter_damping()
                        state["hold"] = True
                        self.beep.play("done")
                        self.log(f"\n[자동] ✅ {len(self.tracks)} 회차 끝 — 힘을 뺐다")
                        self._go(now, "end")
                        self.finished = True
                        return None
                    self._enter_place(now, motor)
            return tg
        return None

    def _enter_lift(self, now, motor) -> None:
        """hooks.lift_pose(끝 자세, 목표 원본, 주차) → 차례로 갈 자세 목록 (들어 올림, 필요하면 경유) 또는 None."""
        seq = self.hooks.lift_pose(self.goal, self.track, self.park) if self.hooks is not None else None
        if not seq:
            self._enter_return(now, motor)
            return
        self.lift_goal = list(seq)
        self._lift_i = 0
        self.log("\n[자동] 집게 끝을 판에서 수직으로 들어 올린다" + (f" → 경유 자세 {len(seq) - 1} 개" if len(seq) > 1 else ""))
        self._lift_seg(now, motor)

    def _lift_seg(self, now, motor) -> None:
        self._from = {s: dict(motor.command_target[s]) for s in motor.sides}
        goal = self.lift_goal[self._lift_i]
        big = max(abs(goal[s][j] - self._from[s][j]) for s in goal for j in goal[s])
        self._lift_dur = max(LIFT_S, big / LIFT_RATE)
        self._go(now, "lift")

    def _enter_return(self, now, motor) -> None:
        self.log(f"\n[자동] 주차 자세로 {self.move_s:.0f} 초에 걸쳐 돌아간다")
        self._from = {s: dict(motor.command_target[s]) for s in motor.sides}
        self._last_cmd, self._still_since = None, None
        self._go(now, "return")

    def _start_round(self, now, motor) -> None:
        if self.round == 0:
            self._enter_approach(now, motor)
        else:
            self._enter_place(now, motor)

    def _enter_approach(self, now, motor) -> None:
        self._from = {s: dict(motor.command_target[s]) for s in motor.sides}
        self.goal = self.track.first_pose()
        self.log(f"\n[자동] {self.round + 1}/{len(self.tracks)} 회차 ({self.track.label}) — 첫 자세로 {self.move_s:.0f} 초에 걸쳐")
        self._go(now, "approach")

    def _enter_place(self, now, motor) -> None:
        if self.round > 0:                  # 복귀가 멈춘 그 자리 (판 위 여유에 막혔으면 거기) — 주차 값을 계속 주면 판 쪽으로 밀기만 한다
            self.goal = {s: dict(motor.command_target[s]) for s in motor.sides}
        where = "첫 자세" if self.round == 0 else "주차 자세"
        self.beep.play("place", text=f"{self.round + 1} 회차 캔을 놓으세요 ({self.wait_s:.0f} 초)")
        self.log(f"\n[자동] {self.round + 1} 회차 캔 놓을 시간 {self.wait_s:.0f} 초 ({where}에서 기다림, 마지막 {TICK_LAST_S} 초는 1 초마다 삐)")
        self._go(now, "place")

    def _enter_play(self, now) -> None:
        if self.hooks is not None and self.park is not None:     # 14-h: 끝 자세의 들어 올림·경유를 미리 (제어 루프 밖 스레드에서)
            self.hooks.prepare(self.track, self.park)
        self.track.begin()
        self.beep.play("play")
        self.log(f"\n[자동] ▶ {self.round + 1} 회차 재생 ({self.track.label})")
        self._go(now, "play")

    def apply_grip(self, gripper) -> None:
        if self.stage in ("play", "end_hold", "grip_wait"):
            self.track.apply_grip(gripper, include_done=True)

    def tick_can(self, now: float, motor) -> None:
        self.track.tick_can(now, motor)

    def csv_row(self) -> List[str]:
        return [str(self.round + 1), self.stage]


# --------------------------------------------------------------------------- 종료 동작 (VR 실기동)
# 무장: 주차에서 13 cm 넘게 나가 있던 시간이 합쳐 10 초 (END_ARM_S)
# 근거: 판 주차로 시작한 세션 16 개 (dry_table·dry_hd·dry_tA~tC·dry_vcan, 2026-11-03) 에서 일하는 중 "주차 8 cm 안 + 3 cm/s 이하" 가
#   이어진 최장 1.25 초 (10 cm 면 3.5 초 — 그래서 8 cm). 차렷으로 시작한 세션 (주차 = 차렷) 은 쉬는 구간에 걸리므로 쓰지 않는다
END_RADIUS_CM = 8.0
END_SPEED_CM_S = 3.0
END_HOLD_S = 3.0
END_ARM_CM = 5.0            # 주차에서 (반경 + 이것) 넘게 나가 있던 시간이 합쳐 END_ARM_S 가 되어야 판정을 시작
END_ARM_S = 10.0            #   (켠 직후 손을 따라 한 번 튄 것만으로 무장하지 않게 — table_0049 재생: 3.6 초 나갔다가 가만히 서 있던 사람에 걸렸다)
END_RETURN_S = 4.0
END_SAMPLE_S = 0.05         # 20 Hz 로만 본다 (FK 비용)
END_SPEED_WINDOW_S = 0.5


class EndGesture:
    """update() → True 면 종료 동작. tips_fn({side: {joint: 모터 값}}) → {side: np.array 집게 끝 cm}."""

    def __init__(self, tips_fn: Callable, radius_cm: float = END_RADIUS_CM, speed_cm_s: float = END_SPEED_CM_S,
                 hold_s: float = END_HOLD_S, arm_cm: float = END_ARM_CM):
        self.tips_fn = tips_fn
        self.radius, self.speed, self.hold, self.arm = radius_cm, speed_cm_s, hold_s, arm_cm
        self.park = None
        self.pos_on = False         # 워치독이 위치 모드 들어갈 때를 알려고 쓴다 (들어가면 set_park)
        self.reset_state()

    def reset_state(self) -> None:
        self.armed = False
        self.away_s = 0.0
        self.since = None
        self.hist: List[tuple] = []
        self.last_t = -math.inf
        self.dist = math.nan
        self.vel = math.nan

    def set_park(self, motor_pose) -> None:
        self.park = self.tips_fn(motor_pose)
        self.reset_state()

    def update(self, now: float, motor_pose, grip: Optional[Dict[str, int]] = None) -> bool:
        if self.park is None or now - self.last_t < END_SAMPLE_S:
            return False
        step = min(now - self.last_t, 0.2)
        self.last_t = now
        tips = self.tips_fn(motor_pose)
        self.dist = max(float(np.linalg.norm(tips[s] - self.park[s])) for s in tips)
        self.hist.append((now, tips))
        while self.hist and now - self.hist[0][0] > END_SPEED_WINDOW_S + 1e-6:
            self.hist.pop(0)
        t0, p0 = self.hist[0]
        self.vel = (max(float(np.linalg.norm(tips[s] - p0[s])) for s in tips) / max(now - t0, END_SAMPLE_S)) if now > t0 else 0.0
        if self.dist > self.radius + self.arm and math.isfinite(step):
            self.away_s += step
            self.armed = self.armed or self.away_s >= END_ARM_S
        ok = (self.armed and self.dist <= self.radius and self.vel <= self.speed
              and not any(int(v) == 1 for v in (grip or {}).values()))
        if not ok:
            self.since = None
            return False
        self.since = now if self.since is None else self.since
        return now - self.since >= self.hold


# --------------------------------------------------------------------------- VR 끊김 감시 (작업 지시 14-c)
# VR 패킷 (UDP 5000 JSON) 을 명령이 아니라 "사람이 아직 VR 안에 있음" 신호로 본다. 끊김 네 종류:
#   nopkt    패킷이 안 온다 — VR Preview 끄기 (송신 컴포넌트 EndPlay 가 소켓을 닫는다), 네트워크
#   inactive active=false (관절 목록이 빔) 또는 필요한 본이 없음 — 몸 추적이 꺼짐. 기록의 세션 끝은 거의 다 이것
#   frozen   active 인데 관절 값이 그대로 — 기록 (active 패킷 전부) 에서 한 번도 없었다. 그래도 규격에 없는 상태라 막는다
#   head_off 머리가 기준 자리에서 크게 벗어남 — 헤드셋을 벗으면 (Quest Link) active 는 true 그대로이고 몸 전체가 헤드셋을 따라간다
#            (10-07 hmd_off: 아래 70 cm·옆 1 m). 기록 53 개 작업 구간 (100 분) 에서 1 초 이어진 최대 수평 29.9·아래 24.1 cm
VR_NOPKT_S = 0.5            # 워치독 UDP_TIMEOUT_SEC 와 같다
VR_FROZEN_S = 1.0           # 같은 값 14 패킷쯤
VR_HEAD_HORIZ_CM = 50.0     # 원격 조종 (사람이 판 앞에 서 있다). 재생 감시는 캔을 놓으러 걸어가므로 수평은 안 본다
VR_HEAD_DROP_CM = 40.0      # 원격 조종
VR_HEAD_DROP_GUARD_CM = 60.0  # 재생 감시 (캔을 놓으며 숙인다)
VR_HEAD_S = 1.0
VR_HEAD_BACK_S = 2.0        # 머리 벗어남은 한 번 걸리면 기준 근처 (아래 BACK 값) 로 이만큼 돌아와 있어야 풀린다 (헤드셋을 들고 오르내려도 이어지게)
VR_HEAD_BACK = (25.0, 20.0)        # 원격 조종 (수평, 아래 cm)
VR_HEAD_BACK_GUARD = (50.0, 30.0)  # 재생 감시
VR_GRACE_S = 5.0            # 이 안에 돌아오면 이어서, 넘으면 주차 뒤 감쇠
VR_PARK_S = 4.0
VR_ALIVE_S = 3.0            # --vr-guard: 이만큼 살아 있어야 시작
KIND_KO = {"nopkt": "패킷 안 옴", "inactive": "추적 꺼짐 (active false)", "frozen": "값 그대로", "head_off": "머리 벗어남 (헤드셋 벗음)"}


class VrLink:
    """update(now, frame, joints_ok) → None (살아 있음) 또는 끊김 종류. frame 은 워치독 BodyTrackingFrame (없으면 None)."""

    def __init__(self, head_horiz_cm: Optional[float] = VR_HEAD_HORIZ_CM, head_drop_cm: float = VR_HEAD_DROP_CM,
                 back: tuple = VR_HEAD_BACK):
        self.head_horiz = head_horiz_cm
        self.head_drop = head_drop_cm
        self.back = back
        self.head_latched = False
        self._head_back_since = None
        self.ref = None                     # 머리 기준 (cm)
        self.kind: Optional[str] = None
        self.detail = ""
        self._last_rx = None
        self._last_key = None
        self._changed_at = None
        self._head_hist: List[tuple] = []
        self._head_bad_since = None
        self.head_horiz_now = math.nan
        self.head_drop_now = math.nan

    @staticmethod
    def _key(frame):
        return tuple((n, tuple(np.round(j["pos"], 4)), tuple(np.round(j["rot"], 5))) for n, j in sorted(frame.joints.items()))

    def set_ref(self, now: float) -> bool:
        """지난 1 초 머리 위치 중앙값을 기준으로. 머리가 없으면 False."""
        pts = [p for t, p in self._head_hist if now - t <= 1.0]
        if not pts:
            return False
        self.ref = np.median(np.array(pts), axis=0)
        self._head_bad_since = None
        self.head_latched = False
        return True

    def update(self, now: float, frame, joints_ok: bool = True) -> Optional[str]:
        kind, detail = None, ""
        if frame is None or now - frame.received_at > VR_NOPKT_S:
            kind = "nopkt"
            detail = "처음부터 없음" if frame is None else f"{now - frame.received_at:.1f} 초째"
        else:
            if frame.received_at != self._last_rx:          # 새 패킷
                self._last_rx = frame.received_at
                key = self._key(frame) if frame.active else None
                if key is None or key != self._last_key:
                    self._changed_at = now
                self._last_key = key
                head = frame.joints.get("Head") if frame.active else None
                if head is not None:
                    self._head_hist.append((now, np.asarray(head["pos"], float)))
                    while self._head_hist and now - self._head_hist[0][0] > 2.0:
                        self._head_hist.pop(0)
            if not frame.active or not joints_ok:
                kind = "inactive"
            elif self._changed_at is not None and now - self._changed_at >= VR_FROZEN_S:
                kind, detail = "frozen", f"{now - self._changed_at:.1f} 초째"
            elif self.ref is not None and self._head_hist:
                d = self._head_hist[-1][1] - self.ref
                self.head_horiz_now = float(np.linalg.norm(d[:2]))
                self.head_drop_now = float(-d[2])
                bad = (self.head_drop_now > self.head_drop
                       or (self.head_horiz is not None and self.head_horiz_now > self.head_horiz))
                if self.head_latched:
                    near = self.head_horiz_now <= self.back[0] and self.head_drop_now <= self.back[1]
                    self._head_back_since = (now if self._head_back_since is None else self._head_back_since) if near else None
                    if near and now - self._head_back_since >= VR_HEAD_BACK_S:
                        self.head_latched = False
                        self._head_bad_since = None
                elif bad:
                    self._head_bad_since = now if self._head_bad_since is None else self._head_bad_since
                    if now - self._head_bad_since >= VR_HEAD_S:
                        self.head_latched = True
                        self._head_back_since = None
                else:
                    self._head_bad_since = None
                if self.head_latched:
                    kind = "head_off"
                    detail = f"수평 {self.head_horiz_now:.0f} cm, 아래 {self.head_drop_now:.0f} cm"
        self.kind, self.detail = kind, detail
        return kind

    def text(self) -> str:
        return KIND_KO.get(self.kind or "", "") + (f" ({self.detail})" if self.detail else "")
