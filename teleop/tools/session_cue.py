#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""구간 큐 — 측정 중 "지금 무슨 동작을 하라"를 소리로 알리고, 구간 시각을 CSV 로 남긴다.

왜 필요한가
-----------
REPORT_09 §3.2: 사용자가 감으로 움직여 **구간 분할에 실패**했다. 3초 정지가 로그에
없었고 `shoulder_pitch` 우세 구간도 없었다. 사람 탓이 아니다 — 헤드셋을 쓰면 화면이
안 보이고 타이머도 없다. 그래서 도구로 없앤다.

`vr_control_watchdog_biarm.py` 는 **고치지 않는다.** 이 스크립트는 완전히 독립이며
워치독과 나란히 띄운 뒤 사후에 시각으로 맞춘다.

🔴 시각 기준 (PLAN11 §1 조사 결론) — 다시 조사하지 마라
-------------------------------------------------------
워치독 `--log-csv` 의 `t` 열은 **`time.monotonic()`** 이다
(`vr_control_watchdog_biarm.py:1187` 에서 `now = time.monotonic()`, 1262행에서 그대로 기록).

리눅스에서 `time.monotonic()` 은 `CLOCK_MONOTONIC` 이고 **부팅 시각 기준의 시스템 공용
시계**다 (`/proc/uptime` 과 일치함을 실측 확인). 즉 **같은 PC 에서 돌린 별개의 두
프로세스가 같은 눈금을 본다.**

⇒ 이 스크립트가 `t_*_mono` 를 남기면 워치독 CSV 의 `t` 와 **오프셋 없이 그대로 정렬된다.**
   워치독을 자식 프로세스로 띄울 필요도(PLAN11 §1-a), epoch 로 근사할 필요도(§1-b) 없다.
   `split_by_cue.py --offset` 은 **보정용 예비 수단**이고 기본값 0 이면 맞는다.

   주의: 이 등식은 **두 프로세스가 같은 PC 에서 돌 때만** 성립한다. 큐를 서버 PC 에서
   돌리면 `t_*_epoch` 로 맞춰야 하고, 두 PC 의 시계 오차만큼 틀어진다.

소리
----
타이밍 루프는 **소리를 절대 기다리지 않는다.** 패턴 전체를 WAV 한 개로 미리 만들어 두고
리눅스는 `subprocess.Popen(paplay)`, 윈도우는 `winsound.PlaySound(..., SND_ASYNC)` 로
던지기만 한다. (`winsound.Beep` 은 **동기 호출이라 루프를 막는다** — 윈도우 프로브에서
구간 경계마다 4.7초를 잃은 전례가 있다. PLAN6A.)

🔴 **로봇 PC 에서는 소리가 나지 않는다** (2026-09-18 확인). 사운드 카드가 HDMI(NVidia)뿐이고
HDMI 출력 프로파일이 전부 `available: 아니요`, 유일한 싱크가 `auto_null`(더미)이다.
⇒ **큐는 서버 PC(언리얼)에서 돌린다.** 사용자가 Quest 를 Link 로 쓰고 있으므로 서버 PC 의
소리가 헤드셋으로 들린다. 그러면 시각 기준이 갈라지므로 **UDP 마커**를 쓴다:

    [서버 PC]  session_cue.py --udp-mark 192.168.0.65:5020   (소리 + 마커 송신)
    [로봇 PC]  cue_listener.py                                (마커 수신 -> 로컬 monotonic 으로 CSV)

마커는 구간 경계마다 UDP 1개다. LAN/Tailscale 왕복이 수 ms 라 10초 구간에 견줘 무시할 만하다.
로봇 PC 에 소리를 낼 수단이 생기면 마커 없이 로봇 PC 에서 그냥 돌리면 된다.

사용법
------
    python3 session_cue.py --dry                 # 소리 없이 일정만 출력
    python3 session_cue.py                       # 기본: 대기 30s, 동작 10s, 정지 4s
    python3 session_cue.py --motion-sec 12 --rest-sec 5
    python3 session_cue.py --check-audio         # 소리만 시험하고 끝낸다
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import wave
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

# (이름, 사용자에게 읽어줄 안내) — 팔 접두어(L_/R_)는 build_phases() 가 붙인다
BASE_PHASES: List[Tuple[str, str]] = [
    ("shoulder_pitch", "팔꿈치 편 채로 앞뒤로만"),
    ("shoulder_roll",  "팔꿈치 편 채로 옆으로 벌렸다 모으기"),
    ("shoulder_yaw",   "팔 내리고 상완만 비틀기"),
    ("diagonal_big",   "대각선으로 최대한 크게 (앞+옆 동시, 손이 머리 위까지!)"),
]

ARM_LABEL = {"left": "왼팔", "right": "오른팔"}


def build_phases(arms: Sequence[str]) -> List[Tuple[str, str]]:
    """양팔이면 왼팔 4구간 -> 팔 교대 -> 오른팔 4구간.

    구간 이름에 팔을 넣어야 `split_by_cue.py` 가 **어느 팔의 열을 볼지** 안다.
    한쪽을 하는 동안 **반대 팔은 가만히 두어야** 한다 — 분석기가 그것도 검사한다.
    """
    out: List[Tuple[str, str]] = []
    for i, arm in enumerate(arms):
        if i > 0:
            out.append(("switch_arm", f"🔴 팔 교대 — 이제 {ARM_LABEL[arm]}. 반대 팔은 내리고 가만히"))
        pre = "L_" if arm == "left" else "R_"
        for name, note in BASE_PHASES:
            out.append((pre + name, f"[{ARM_LABEL[arm]}] {note}  (반대 팔은 가만히)"))
    return out

# --------------------------------------------------------------------------- refs 프로토콜
# 몸통 기준 후보 비교용 (REPORT_15 §5.1). 워치독 없이 rx_probe.py 로 원시 JSON 을 녹화하고,
# 중립은 **오프라인에서 calib 구간으로** 잡는다 (c/p 를 누르지 않는다).
# 한 세션에 두 판정 기준을 같이 담는다: ① 몸통만 회전 시 헛명령  ② 앞으로 들기 1->3 누설.
PROTOCOL = ["cue"]      # main() 이 CLI 값으로 덮어쓴다

CALIB_NOTE = ("🔴 중립 고정 — 똑바로 서서 양팔을 몸 옆에 곧게 내린다. 손바닥은 허벅지를 향하고 "
              "엄지는 앞. 어깨 힘 빼고 정면 응시. 이 구간이 끝날 때까지 움직이지 않는다")
TORSO_NOTE = ("양팔을 몸통에 붙이고(겨드랑이에 대고) 팔꿈치 90°. 팔은 몸에 고정한 채 "
              "몸통만 좌우로 천천히 크게 돌린다. 고개는 몸통과 같이 돌린다")
TORSO_GAZE_NOTE = ("같은 자세로 몸통만 좌우로 돌리되, 🔴 시선은 정면의 한 점에 고정한다 "
                   "(고개는 몸통과 반대로 돌아가 정면을 유지)")


def build_schedule_refs(settle: float, motion: float, rest: float,
                        torso: float, calib: float) -> List["Segment"]:
    """settle -> calib -> 왼팔 1/3/5 -> 오른팔 2/4/6 -> 몸통 회전(고개 같이) -> 몸통 회전(시선 고정) -> calib_end."""
    plan: List[Tuple[str, str, float]] = [
        ("settle", "VR_OK 확인, 차렷 자세로 대기. 3연음이 울리면 중립 고정", settle),
        ("calib", CALIB_NOTE, calib),
    ]
    for arm, pre in (("left", "L_"), ("right", "R_")):
        for name, note in BASE_PHASES[:3]:            # pitch / roll / yaw
            plan.append((pre + name, f"[{ARM_LABEL[arm]}] {note}  (반대 팔은 가만히)", motion))
            plan.append(("rest", "팔을 완전히 내리고 정지", rest))
    plan.append(("torso_turn", TORSO_NOTE, torso))
    plan.append(("rest", "정면으로 돌아와 잠깐 정지 (팔은 몸에 붙인 채)", rest))
    # 머리 후보를 공정하게 보려면 "고개가 몸통과 따로 노는" 경우도 있어야 한다.
    # 실제 조작에서는 목표를 보면서 몸통을 돌리는 일이 흔하다.
    plan.append(("torso_turn_gaze", TORSO_GAZE_NOTE, torso))
    plan.append(("rest", "팔을 내리고 정면으로 돌아온다", rest))
    plan.append(("calib_end", "🔴 처음과 같은 중립 자세로 다시 고정 (드리프트 확인용)", calib))
    segs: List[Segment] = []
    t = 0.0
    for idx, (name, note, dur) in enumerate(plan):
        segs.append(Segment(idx, name, note, t, t + dur))
        t += dur
    return segs


# --------------------------------------------------------------------------- frames 프로토콜 (2026-09-23)
# 한 세션에 두 가지를 담는다: ① 몸 틀 후보 비교 (Hips/SpineMiddle 추가분 포함) ② 왼팔 손목 부호 재확인.
# refs 와 같은 흐름 + 끝에 왼팔 손목 구간(자세마다 2초 이상 멈춤).

WRIST_STEPS = [
    ("L_wrist_thumb1", "엄지가 위. 그대로 2초 이상 멈춘다"),
    ("L_wrist_back", "손목만 돌려 손등이 위로. 2초 이상 멈춘다"),
    ("L_wrist_thumb2", "다시 엄지가 위. 2초 이상 멈춘다"),
    ("L_wrist_palm", "손목만 돌려 손바닥이 위로. 2초 이상 멈춘다"),
    ("L_wrist_thumb3", "다시 엄지가 위. 2초 이상 멈춘다"),
]


def build_schedule_live(settle: float, motion: float, rest: float, torso: float,
                        switch: float) -> List["Segment"]:
    """실기동 측정용 (2026-09-24). c/p 신호가 있는 cue 흐름 + 몸통 구간.

    같은 순서를 **위치 방식 한 번, 뼈 방식 한 번** 돌려 같은 조건으로 비교한다.
    """
    plan: List[Tuple[str, str, float]] = [
        ("settle", "중립(차렷)으로 서 있기 → 3연음이 울리면 c, 확인되면 p", settle),
    ]
    for arm, pre, label in (("left", "L_", "왼팔"), ("right", "R_", "오른팔")):
        if pre == "R_":
            plan.append(("switch_arm", "🔴 팔 교대 — 이제 오른팔. 왼팔은 내리고 가만히", switch))
        for name, note in BASE_PHASES[:3]:
            plan.append((pre + name, f"[{label}] {note}  (반대 팔은 가만히)", motion))
            plan.append(("rest", "팔을 완전히 내리고 정지", rest))
    plan.append(("torso_turn", TORSO_NOTE, torso))
    plan.append(("rest", "정면으로 돌아와 정지 (팔은 몸에 붙인 채)", rest))
    plan.append(("torso_turn_gaze", TORSO_GAZE_NOTE, torso))
    plan.append(("rest", "팔을 내리고 정면으로. 끝나면 h 로 멈춘다", rest))
    segs: List[Segment] = []
    t = 0.0
    for idx, (name, note, dur) in enumerate(plan):
        segs.append(Segment(idx, name, note, t, t + dur)); t += dur
    return segs


def build_schedule_torso(settle: float, rest: float, torso: float, calib: float) -> List["Segment"]:
    """몸통 구간만 다시 (9/23 재측정용). 🔴 팔꿈치 90° 를 지키는 것이 핵심이다."""
    hold = ("🔴 양팔을 겨드랑이에 붙이고 팔꿈치를 90° 로 굽힌 채 (아래팔이 앞을 향함). "
            "그 자세를 유지한 채 몸통만 좌우로 천천히 크게 돌린다")
    plan = [("settle", "VR_OK 확인, 차렷 자세로 대기", settle),
            ("calib", CALIB_NOTE, calib),
            ("torso_turn", hold + ". 고개는 몸통과 같이 돌린다", torso),
            ("rest", "정면으로 돌아와 정지 (팔꿈치 90° 유지)", rest),
            ("torso_turn_gaze", hold + ". 🔴 시선은 정면의 한 점에 고정 (고개는 정면 유지)", torso),
            ("rest", "팔을 내리고 정면으로", rest),
            ("calib_end", "🔴 처음과 같은 중립 자세로 고정", calib)]
    out: List[Segment] = []
    t = 0.0
    for idx, (name, note, dur) in enumerate(plan):
        out.append(Segment(idx, name, note, t, t + dur)); t += dur
    return out


def build_schedule_frames(settle: float, motion: float, rest: float, torso: float, calib: float,
                          hold: float) -> List["Segment"]:
    segs = build_schedule_refs(settle, motion, rest, torso, calib)
    plan = [(s.phase, s.note, s.planned_end - s.planned_start) for s in segs]
    tail = plan.pop()                      # calib_end 는 맨 뒤로 보낸다
    plan.append(("wrist_ready", "🔴 왼팔만 팔꿈치 90°, 엄지가 위 (손날이 위). 오른팔은 차렷 그대로", 6.0))
    for name, note in WRIST_STEPS:
        plan.append((name, f"[왼팔 손목] {note}", hold))
    plan.append(("rest", "왼팔을 내리고 정지", rest))
    plan.append(tail)
    out: List[Segment] = []
    t = 0.0
    for idx, (name, note, dur) in enumerate(plan):
        out.append(Segment(idx, name, note, t, t + dur))
        t += dur
    return out


# 🔴 ④ 는 지난 세션(2026-09-18)에서 |roll|+|yaw| 최대가 2.37 rad 에 그쳐
# 테이퍼 끝 구간(2.60 rad 이상)을 시험하지 못했다 (REPORT_09 §6).
PHASE_NOTES = {
    "diagonal_big": "🔴 지난번보다 훨씬 크게! 팔을 끝까지 뻗어 올려야 테이퍼가 시험된다",
}


def phase_note(phase: str) -> Optional[str]:
    for key, note in PHASE_NOTES.items():
        if phase.endswith(key):
            return note
    return None

SAMPLE_RATE = 44100

# 'c' 신호를 동작 시작 몇 초 전에 줄지. main() 이 CLI 값으로 덮어쓴다.
#
# 🔴 기본 6초 -> 10초 (2026-09-18 세션 결과).
# c 누르기 -> 2초 캘리브레이션 지연 -> [CALIBRATION] 확인 -> p 누르기를 6초 안에 끝내야 했는데
# 빠듯했다. 실측: POS+RUN 진입이 첫 동작 구간 시작보다 **4.2초 / 1.0초 늦었고**,
# 그만큼 첫 구간(L_shoulder_pitch) 데이터가 통째로 비었다 (REPORT_10 §4).
READY_LEAD = [10.0]


# --------------------------------------------------------------------------- 소리

def _tone(freq: float, ms: int, volume: float = 0.6) -> bytes:
    """사인파 한 음. 클릭음이 나지 않게 양끝을 5ms 페이드한다."""
    n = int(SAMPLE_RATE * ms / 1000)
    fade = max(1, int(SAMPLE_RATE * 0.005))
    out = bytearray()
    for i in range(n):
        env = min(1.0, i / fade, (n - i) / fade)
        out += struct.pack("<h", int(32767 * volume * env * math.sin(2 * math.pi * freq * i / SAMPLE_RATE)))
    return bytes(out)


def _silence(ms: int) -> bytes:
    return b"\x00\x00" * int(SAMPLE_RATE * ms / 1000)


def _write_wav(path: Path, payload: bytes) -> None:
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(payload)


# 패턴을 WAV 한 개로 만들어 둔다 -> 재생 호출이 1회뿐이라 루프가 블로킹되지 않는다
PATTERNS = {
    "ready":  [(880, 120), (0, 80), (880, 120), (0, 80), (880, 200)],   # 중립 저장하라 (3연음)
    "start":  [(1320, 180)],                                            # 동작 시작 (높은음)
    "stop":   [(520, 150), (0, 70), (520, 150)],                        # 정지 (낮은음 2번)
    "finish": [(1046, 160), (880, 160), (698, 160), (523, 320)],        # 종료 (내려가는 4음)
}


class Beeper:
    """소리를 비동기로 낸다. 못 내면 조용히 실패하지 않고 경고한다."""

    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        self.player: Optional[List[str]] = None
        self.reason = ""
        self.files = {}
        self._procs: List[subprocess.Popen] = []
        if not enabled:
            self.reason = "--dry / --no-sound"
            return

        if os.name == "nt":
            try:
                import winsound  # noqa: F401
                self.player = ["winsound"]
            except ImportError:
                self.reason = "winsound 를 쓸 수 없다"
                return
        elif shutil.which("paplay"):
            self.player = ["paplay"]
        elif shutil.which("aplay"):
            self.player = ["aplay", "-q"]
        else:
            self.reason = "paplay / aplay 가 없다"
            return

        tmp = Path(tempfile.mkdtemp(prefix="session_cue_"))
        for name, pattern in PATTERNS.items():
            payload = b"".join(_silence(ms) if f == 0 else _tone(f, ms) for f, ms in pattern)
            path = tmp / f"{name}.wav"
            _write_wav(path, payload)
            self.files[name] = path

    # -- 실제로 들리는지 확인 (PulseAudio 가 더미 싱크로 떨어진 경우를 잡는다)
    def audio_sink_warning(self) -> Optional[str]:
        if not self.enabled:
            return None
        if self.player is None:
            return f"소리를 낼 수 없다: {self.reason}"
        if os.name == "nt":
            return None
        if shutil.which("pactl"):
            try:
                out = subprocess.run(["pactl", "list", "short", "sinks"],
                                     capture_output=True, text=True, timeout=3).stdout.strip()
            except Exception:
                return None
            if not out:
                return "PulseAudio 싱크가 하나도 없다 — 소리가 나지 않는다"
            real = [ln for ln in out.splitlines() if "auto_null" not in ln and "dummy" not in ln.lower()]
            if not real:
                return ("유일한 오디오 싱크가 **auto_null(더미)** 이다 — "
                        "소리가 어디로도 나가지 않는다. 스피커/이어폰을 연결하거나 "
                        "HDMI 출력을 활성화하라 (`pactl list short sinks` 로 확인)")
        return None

    def play(self, name: str) -> None:
        if self.player is None or name not in self.files:
            return
        try:
            if self.player[0] == "winsound":
                import winsound
                # 🔴 SND_ASYNC: 즉시 돌아온다. winsound.Beep 은 동기라 쓰면 안 된다.
                winsound.PlaySound(str(self.files[name]),
                                   winsound.SND_FILENAME | winsound.SND_ASYNC | winsound.SND_NODEFAULT)
                return
            # Popen: 기다리지 않는다. 타이밍 루프를 절대 막지 않는다.
            self._procs.append(subprocess.Popen(
                self.player + [str(self.files[name])],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        except Exception:
            pass
        self._procs = [p for p in self._procs if p.poll() is None][-8:]


class Marker:
    """구간 경계를 UDP 로 알린다 (큐를 서버 PC 에서 돌릴 때만 쓴다).

    받는 쪽(`cue_listener.py`)이 **로봇 PC 의 monotonic 으로 스탬프**하므로,
    두 PC 의 벽시계가 어긋나 있어도 상관없다. 전송은 실패해도 조용히 넘어간다 —
    측정 진행이 네트워크 때문에 멈추면 안 된다.
    """

    def __init__(self, target: Optional[str]):
        self.addr = None
        self.sock = None
        self.sent = 0
        self.failed = 0
        if not target:
            return
        host, _, port = target.partition(":")
        self.addr = (host, int(port or 5020))
        import socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def send(self, kind: str, index: int, phase: str, note: str, rel: float) -> None:
        if self.sock is None:
            return
        payload = json.dumps({"kind": kind, "index": index, "phase": phase,
                              "note": note, "rel": round(rel, 3),
                              "epoch": round(time.time(), 3)}).encode("utf-8")
        for _ in range(2):          # UDP 는 유실될 수 있다. 같은 마커를 2번 보낸다
            try:
                self.sock.sendto(payload, self.addr)
                self.sent += 1
            except OSError:
                self.failed += 1


# --------------------------------------------------------------------------- 일정

class Segment:
    __slots__ = ("index", "phase", "note", "planned_start", "planned_end",
                 "t_start_mono", "t_end_mono", "t_start_epoch", "t_end_epoch")

    def __init__(self, index: int, phase: str, note: str, planned_start: float, planned_end: float):
        self.index = index
        self.phase = phase
        self.note = note
        self.planned_start = planned_start
        self.planned_end = planned_end
        self.t_start_mono = self.t_end_mono = None
        self.t_start_epoch = self.t_end_epoch = None


def build_schedule(settle: float, motion: float, rest: float,
                   phases: Sequence[Tuple[str, str]], switch: float = 10.0) -> List[Segment]:
    """settle(=중립 저장 대기) 뒤에 [동작, 정지] 를 반복한다. 마지막 정지도 남긴다.

    `switch_arm` 은 동작 구간이 아니라 **팔을 바꾸는 시간**이라 길이가 다르고
    뒤에 정지를 따로 붙이지 않는다.
    """
    segs: List[Segment] = []
    t = 0.0
    segs.append(Segment(0, "settle", "중립 자세로 서 있기 → 신호가 울리면 c, 이어서 p", t, t + settle))
    t += settle
    idx = 1
    for name, note in phases:
        if name == "switch_arm":
            segs.append(Segment(idx, name, note, t, t + switch)); t += switch; idx += 1
            continue
        segs.append(Segment(idx, name, note, t, t + motion)); t += motion; idx += 1
        segs.append(Segment(idx, "rest", "팔을 완전히 내리고 정지", t, t + rest)); t += rest; idx += 1
    return segs


def print_schedule(segs: Sequence[Segment]) -> None:
    print("=" * 78)
    print(f"{'#':>2}  {'구간':<18} {'시작':>7} {'끝':>7}  안내")
    print("-" * 78)
    for s in segs:
        print(f"{s.index:>2}  {s.phase:<18} {s.planned_start:7.1f} {s.planned_end:7.1f}  {s.note}")
    print("-" * 78)
    print(f"총 길이: {segs[-1].planned_end:.1f}초")
    print("=" * 78)


def write_csv(path: Path, segs: Sequence[Segment], t0_mono: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["index", "phase", "note",
                    "t_start_mono", "t_end_mono",
                    "t_start_epoch", "t_end_epoch",
                    "t_start_rel", "t_end_rel"])
        for s in segs:
            if s.t_start_mono is None:
                continue
            end_mono = s.t_end_mono if s.t_end_mono is not None else ""
            end_epoch = s.t_end_epoch if s.t_end_epoch is not None else ""
            end_rel = f"{s.t_end_mono - t0_mono:.3f}" if s.t_end_mono is not None else ""
            w.writerow([s.index, s.phase, s.note,
                        f"{s.t_start_mono:.3f}", f"{end_mono:.3f}" if end_mono != "" else "",
                        f"{s.t_start_epoch:.3f}", f"{end_epoch:.3f}" if end_epoch != "" else "",
                        f"{s.t_start_mono - t0_mono:.3f}", end_rel])


# --------------------------------------------------------------------------- 실행

def run(segs: List[Segment], beeper: Beeper, csv_path: Path, marker: "Marker") -> None:
    t0_mono = time.monotonic()
    errors: List[float] = []
    print(f"\n시작. monotonic t0 = {t0_mono:.3f}  (워치독 CSV 의 t 열과 같은 눈금이다)")
    print("Ctrl+C 로 중단해도 그때까지의 구간이 CSV 에 남는다.\n")

    try:
        for i, seg in enumerate(segs):
            deadline = t0_mono + seg.planned_start
            # 마감까지 잘게 자서 대기 — 카운트다운을 찍고 Ctrl+C 도 즉시 받는다
            while True:
                left = deadline - time.monotonic()
                if left <= 0.0:
                    break
                if i > 0 and sys.stdout.isatty():
                    print(f"  다음: {seg.phase:<16} {left:4.1f}초 ", end="\r", flush=True)
                time.sleep(min(0.05, left))

            seg.t_start_mono = time.monotonic()
            seg.t_start_epoch = time.time()
            errors.append(abs(seg.t_start_mono - deadline))
            if i > 0:
                prev = segs[i - 1]
                prev.t_end_mono, prev.t_end_epoch = seg.t_start_mono, seg.t_start_epoch

            rel = seg.t_start_mono - t0_mono
            if seg.phase == "settle":
                pass                       # 대기 시작에는 소리를 내지 않는다
            elif seg.phase.startswith("calib"):
                beeper.play("ready")       # 3연음 = 지금 중립 자세로 고정
            elif seg.phase == "rest":
                beeper.play("stop")
            elif seg.phase == "switch_arm":
                beeper.play("finish")      # 팔 교대는 다른 소리로 구분한다
            else:
                beeper.play("start")
            marker.send("start", seg.index, seg.phase, seg.note, rel)
            note = phase_note(seg.phase)
            print(f"[구간 시작] {seg.phase:<18} t={rel:6.1f}s  mono={seg.t_start_mono:.3f} "
                  f"epoch={seg.t_start_epoch:.3f}   {seg.note}"
                  + (f"\n            {note}" if note else ""))

            # settle 이 끝나기 ready_lead 초 전에 "중립 저장" 신호를 준다.
            # c -> (2초 캘리브레이션 지연) -> p 를 누를 시간이 필요하다.
            if seg.phase == "settle" and PROTOCOL[0] in ("cue", "live"):
                warn_at = t0_mono + seg.planned_end - READY_LEAD[0]
                while time.monotonic() < warn_at:
                    left = warn_at - time.monotonic()
                    if sys.stdout.isatty():
                        print(f"  중립 저장(c) 신호까지 {left:4.1f}초 ", end="\r", flush=True)
                    time.sleep(min(0.05, max(left, 0.001)))
                beeper.play("ready")
                print(f"\n[신호] 3연음 — 지금 c 를 누르고, 2초 뒤 캘리브레이션이 끝나면 p 를 눌러라 "
                      f"(첫 동작까지 {READY_LEAD[0]:.0f}초)")

        # 마지막 구간의 끝
        last = segs[-1]
        deadline = t0_mono + last.planned_end
        while time.monotonic() < deadline:
            time.sleep(min(0.05, deadline - time.monotonic()))
        last.t_end_mono, last.t_end_epoch = time.monotonic(), time.time()
        beeper.play("finish")
        marker.send("end", last.index, last.phase, last.note, last.t_end_mono - t0_mono)
        print("\n[종료] 모든 구간 완료")

    except KeyboardInterrupt:
        now_m, now_e = time.monotonic(), time.time()
        for seg in segs:
            if seg.t_start_mono is not None and seg.t_end_mono is None:
                seg.t_end_mono, seg.t_end_epoch = now_m, now_e
        marker.send("end", segs[-1].index, "abort", "Ctrl+C", now_m - t0_mono)
        print("\n\n[중단] Ctrl+C — 여기까지의 구간을 저장한다")

    write_csv(csv_path, segs, t0_mono)
    done = [s for s in segs if s.t_start_mono is not None]
    print(f"\n구간 CSV: {csv_path}  ({len(done)}개 구간)")
    if errors:
        print(f"구간 경계 오차: 최대 {max(errors)*1000:.1f} ms / 평균 {sum(errors)/len(errors)*1000:.1f} ms"
              + ("   ✅ 50 ms 이내" if max(errors) <= 0.05 else "   🔴 50 ms 초과 — 결과 해석에 주의"))
    if marker.sock is not None:
        print(f"UDP 마커: {marker.sent}개 전송 (실패 {marker.failed}) → {marker.addr[0]}:{marker.addr[1]}")
        print("🔴 구간 시각의 정본은 **로봇 PC 의 cue_listener.py 가 쓴 CSV** 다. 이 파일이 아니다.")
    else:
        print("\n다음: python3 split_by_cue.py --cue "
              f"{csv_path} --log <워치독 로그>.csv")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="측정 구간을 소리로 안내하고 구간 시각을 CSV 로 남긴다")
    p.add_argument("--settle-sec", type=float, default=30.0,
                   help="시작 대기. 이 동안 워치독에서 VR_OK 를 확인하고 자세를 잡는다 (기본 %(default)s)")
    p.add_argument("--motion-sec", type=float, default=10.0, help="구간당 동작 시간 (기본 %(default)s)")
    p.add_argument("--rest-sec", type=float, default=4.0,
                   help="구간 사이 정지 시간. 🔴 팔을 완전히 내려야 분석기가 구간을 확인할 수 있다 (기본 %(default)s)")
    p.add_argument("--out-dir", default=str(Path(__file__).resolve().parent.parent / "logs"),
                   help="구간 CSV 를 쓸 폴더 (기본 _teleop/logs)")
    p.add_argument("--label", default="", help="파일 이름에 붙일 꼬리표 (예: sup_on)")
    p.add_argument("--protocol", choices=("cue", "refs", "frames", "torso", "live"), default="cue",
                   help="cue = 워치독 측정용 (c/p 신호), refs = 몸통 기준 후보 비교용 원시 녹화 "
                        "(calib 구간 + 양팔 1/3/5·2/4/6 + 몸통 회전, c/p 없음), "
                        "frames = refs + 왼팔 손목 구간, torso = 몸통 구간만 (1분 20초), "
                        "live = 실기동 측정 (c/p 신호 + 양팔 3구간 + 몸통 2구간)")
    p.add_argument("--torso-sec", type=float, default=15.0, help="refs: 몸통 회전 구간 길이")
    p.add_argument("--calib-sec", type=float, default=6.0, help="refs: 중립 고정 구간 길이")
    p.add_argument("--hold-sec", type=float, default=4.0, help="frames: 손목 자세마다 멈추는 시간 (기본 %(default)s)")
    p.add_argument("--arms", choices=("left", "right", "both"), default="both",
                   help="측정할 팔. both 면 왼팔 4구간 → 팔 교대 → 오른팔 4구간 (기본 %(default)s)")
    p.add_argument("--ready-lead", type=float, default=10.0,
                   help="첫 동작 시작 몇 초 전에 'c' 신호(3연음)를 줄지. "
                        "c → 2초 캘리브레이션 → p 를 누를 시간이다 (기본 %(default)s)")
    p.add_argument("--switch-sec", type=float, default=10.0,
                   help="팔 교대에 주는 시간 (기본 %(default)s)")
    p.add_argument("--udp-mark", default=None, metavar="IP:PORT",
                   help="구간 경계를 UDP 로 보낸다. 큐를 서버 PC 에서 돌릴 때 쓴다 "
                        "(예: 192.168.0.65:5020). 받는 쪽은 로봇 PC 의 cue_listener.py")
    p.add_argument("--dry", action="store_true", help="소리 없이 일정만 출력하고 끝낸다")
    p.add_argument("--no-sound", action="store_true", help="소리 없이 진행 (타이밍은 실제로 돈다)")
    p.add_argument("--check-audio", action="store_true", help="소리만 시험하고 끝낸다")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    arms = ("left", "right") if args.arms == "both" else (args.arms,)
    READY_LEAD[0] = max(1.0, min(args.ready_lead, max(args.settle_sec - 1.0, 1.0)))
    PROTOCOL[0] = args.protocol
    if args.protocol == "live":
        segs = build_schedule_live(args.settle_sec, args.motion_sec, args.rest_sec,
                                   args.torso_sec, args.switch_sec)
    elif args.protocol == "torso":
        segs = build_schedule_torso(args.settle_sec, args.rest_sec, args.torso_sec, args.calib_sec)
    elif args.protocol == "frames":
        segs = build_schedule_frames(args.settle_sec, args.motion_sec, args.rest_sec,
                                     args.torso_sec, args.calib_sec, args.hold_sec)
    elif args.protocol == "refs":
        segs = build_schedule_refs(args.settle_sec, args.motion_sec, args.rest_sec,
                                   args.torso_sec, args.calib_sec)
    else:
        segs = build_schedule(args.settle_sec, args.motion_sec, args.rest_sec,
                              build_phases(arms), args.switch_sec)
    print_schedule(segs)

    beeper = Beeper(enabled=not (args.dry or args.no_sound))
    warning = beeper.audio_sink_warning()
    if warning:
        print(f"\n🔴 [소리 경고] {warning}")
        print("   들리지 않으면 이 도구의 의미가 없다. 소리를 먼저 해결하라.\n")
    elif beeper.player:
        print(f"\n소리: {' '.join(beeper.player)}")

    if args.check_audio:
        for name in ("ready", "start", "stop", "finish"):
            print(f"  재생: {name}")
            beeper.play(name)
            time.sleep(1.5)
        print("네 가지 신호가 다 들렸는가? 안 들렸으면 위 경고를 확인하라.")
        return

    if args.dry:
        print("\n--dry: 일정만 출력했다. 실제 실행은 --dry 없이.")
        return

    marker = Marker(args.udp_mark)
    if marker.sock is not None:
        print(f"UDP 마커 전송: {marker.addr[0]}:{marker.addr[1]} "
              f"— 로봇 PC 에서 cue_listener.py 를 **먼저** 띄워라")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    name = f"cue_{args.label + '_' if args.label else ''}{stamp}.csv"
    run(segs, beeper, Path(args.out_dir) / name, marker)


if __name__ == "__main__":
    main()
