#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""구간 소리만 내는 상주 프로그램 (서버 PC) — 타이밍은 워치독이 잡는다.

왜 서버 PC 인가: 사용자가 VR 헤드셋을 쓰고 있어 **서버 PC 소리가 헤드셋으로 들어온다**
(2026-09-24 사용자 결정). 로봇 PC 스피커는 확실하지 않다.

    py _teleop/tools/cue_player.py                       # 띄워 두면 계속 대기 (세션이 끝나도 안 꺼진다)
    py _teleop/tools/cue_player.py --list-devices        # 출력 장치 목록 (VR 헤드셋 고르기)
    py _teleop/tools/cue_player.py --device "Rift"       # 그 장치로만 소리를 낸다
    py _teleop/tools/cue_player.py --self-test           # 소리 7종 재생
    py _teleop/tools/cue_player.py --verbose             # 받은 패킷을 전부 찍는다 (신호가 오는지 확인)
    py _teleop/tools/cue_player.py --player winsound     # 말이 안 들리고 "밀린 문구 버림" 만 나올 때 (mci 대신 별도 프로세스로 재생)

워치독이 보내는 것 (UDP, JSON 한 줄):
    {"sound": "start", "phase": "L_shoulder_pitch", "text": "왼팔 — 앞뒤로만", "left": 10.0}
    {"sound": "say:왼팔. 옆으로 만세", ...}      ← 음성 안내 (2026-10-02). JSON 이 아닌 "say:문구" 한 줄도 받는다

음성 (TTS)
    윈도우 기본 음성(SAPI, 한국어 "Microsoft Heami")으로 말한다 — 추가 설치 없음.
    PowerShell System.Speech 로 문구를 wav 로 만들어 두고(시작할 때 고정 문구 전부, 처음 한 번만 몇 초) 재생한다.
    말은 **별도 스레드·별도 재생 경로**(윈도우 MCI, --device 면 별도 스트림)로 내므로 삑 소리·UDP 수신을 막지 않는다.
    말하는 중에 새 문구가 오면 **기다리던 이전 문구는 버리고** 가장 최근 것 하나만 기다린다.
    한국어 음성이 없으면 설치 방법을 한 줄 알려 주고 삑 소리만으로 계속한다. --no-tts 로 끈다.

소리
    ready 3연음(준비/중립) · go 높은음(구간 시작) · learn 낮→높 2음(굽혔다 펴기)
    twist 높은음 두 번(팔 비틀기) · stop 낮은 2음(정지) · tick 초읽기
    switch 올라가는 3음(방식 전환) · done 내려가는 4음(끝) · alarm 3연타(중단·경고)

출력 장치
    기본: 윈도우/리눅스의 **기본 장치**로 낸다 (winsound / paplay).
    `--device` 를 주면 `sounddevice`(PortAudio)로 **그 장치에만** 낸다.
      윈도우에서 설치: py -m pip install sounddevice
    언리얼이 이미 헤드셋으로 소리를 내고 있어도, 윈도우 기본 장치가 다른 곳이면 이 프로그램 소리는
    다른 데서 난다. 그때 `--list-devices` 로 헤드셋을 찾아 `--device` 로 지정하면 된다.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import wave
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import session_cue as SC  # noqa: E402  (Beeper: 기본 장치 재생)

PATTERNS = {
    "ready":  [(880, 120), (0, 80), (880, 120), (0, 80), (880, 200)],
    "go":     [(1320, 180)],
    "learn":  [(784, 140), (0, 60), (1175, 200)],                  # 굽혔다 펴기 (낮→높 2음)
    "twist":  [(1320, 110), (0, 80), (1320, 110)],                 # 팔 비틀기 (높은음 두 번)
    "stop":   [(520, 150), (0, 70), (520, 150)],
    "tick":   [(1046, 70)],
    "switch": [(698, 140), (0, 60), (880, 140), (0, 60), (1046, 220)],
    "done":   [(1046, 160), (880, 160), (698, 160), (523, 320)],
    "alarm":  [(1568, 90), (0, 60), (1568, 90), (0, 60), (1568, 260)],
}
DEFAULT_PORT = 5021
RATE = 44100

# 워치독이 보내는 고정 문구 — 시작할 때 미리 wav 로 만들어 둔다 (vr_control_watchdog_biarm.voice_phrases() 와 같아야 한다.
# 다르면 처음 한 번만 그 자리에서 만든다 — 1~2초 늦다). 문구마다 2초 안에 끝나게 짧게.
_ARMS = ("왼팔", "오른팔")
VOICE_PHRASES = sorted({
    "차렷", "양팔 팔꿈치 구십도, 엄지 위로", "차렷. 곧 모터가 켜집니다", "내리세요", "오른팔로 바꿉니다",
    "끝났습니다. 힘이 빠집니다", "중단합니다. 힘이 빠집니다", "차렷. 다음 바퀴로 바꿉니다", "소리 시험입니다",
    "몸통만 돌리기", "몸통 돌리기. 시선은 정면", "양팔. 팔꿈치 굽혔다 펴기", "양팔. 팔꿈치 빠르게 굽혔다 펴기",
    "손을 수평보다 조금 높게",                                                             # 작업 지시 11-b --start-pose table
    "그대로 멈추기", "양팔. 위팔을 조금 들고, 양손을 가운데로 천천히",                      # yaw (10/3)
    "컨트롤러를 좌우 바꿔 쥐었습니다. 시작하지 않습니다", "차렷 자세가 맞지 않습니다. 시작하지 않습니다",  # 시작 확인 (10/4)
    *[f"{a}. {t}" for a in _ARMS for t in (
        "차렷으로 가만히", "옆으로 벌리고 앞뒤로 흔들기", "팔꿈치 구십도, 손목 돌리기", "팔꿈치 굽혔다 펴기",
        "옆으로 만세", "앞으로 만세",                                     # check
        "편 채로 앞뒤로", "옆으로 벌렸다 모으기", "팔 내리고 비틀기",        # live
        "편 채로 크게 돌리기",                                            # twist
        "앞뒤로 빠르게",                                                  # speed
        "팔꿈치 구십도, 손을 배 쪽으로 천천히", "위팔을 조금 들고, 손을 가운데로 천천히",   # yaw
        "그대로 위팔만 천천히 내리기", "팔꿈치 구십도, 손을 바깥으로 천천히")],
})
try:                                   # 손목 회전 측정 (REPORT_49) — tools/wrist_session.py 의 문구
    import wrist_session as _WS
    VOICE_PHRASES = sorted(set(VOICE_PHRASES) | set(_WS.phrases()))
except Exception as _exc:
    print(f"[음성] wrist_session 문구를 못 읽었다 ({_exc}) — 손목 측정 문구는 처음 말할 때 만든다")
try:                                   # 그리퍼 입력 측정 (REPORT_50) — tools/grip_session.py 의 문구
    import grip_session as _GS
    VOICE_PHRASES = sorted(set(VOICE_PHRASES) | set(_GS.phrases()))
except Exception as _exc:
    print(f"[음성] grip_session 문구를 못 읽었다 ({_exc}) — 그리퍼 측정 문구는 처음 말할 때 만든다")
try:                                   # 위팔 측정 (REPORT_52) — tools/upper_session.py 의 문구
    import upper_session as _US
    VOICE_PHRASES = sorted(set(VOICE_PHRASES) | set(_US.phrases()))
except Exception as _exc:
    print(f"[음성] upper_session 문구를 못 읽었다 ({_exc}) — 위팔 측정 문구는 처음 말할 때 만든다")
try:                                   # 양손 장면 (REPORT_54 P2) — tools/both_session.py 의 문구
    import both_session as _BS
    VOICE_PHRASES = sorted(set(VOICE_PHRASES) | set(_BS.phrases()))
except Exception as _exc:
    print(f"[음성] both_session 문구를 못 읽었다 ({_exc}) — 양손 장면 문구는 처음 말할 때 만든다")

TTS_RATE = 2              # SAPI 말 빠르기 (−10~10). 2 = 조금 빠르게 — 문구를 2초 안에
TTS_INSTALL_HINT = ("🔴 [음성] 한국어 음성이 없다 → 설정 › 시간 및 언어 › 언어 및 지역 › 한국어 › 언어 옵션 › "
                    "'음성' 다운로드 (Microsoft Heami) 후 다시 실행. 지금은 삑 소리만으로 계속한다")

_PS_SCRIPT = r"""param([string]$list)
Add-Type -AssemblyName System.Speech
$s = New-Object System.Speech.Synthesis.SpeechSynthesizer
$v = $s.GetInstalledVoices() | Where-Object { $_.Enabled -and $_.VoiceInfo.Culture.Name -eq 'ko-KR' } | Select-Object -First 1
if (-not $v) { Write-Output 'NOVOICE'; exit 3 }
$s.SelectVoice($v.VoiceInfo.Name)
$s.Rate = __RATE__
Write-Output ('VOICE ' + $v.VoiceInfo.Name)
if ($list) {
  $items = Get-Content -Raw -Encoding UTF8 $list | ConvertFrom-Json
  foreach ($it in $items) { $s.SetOutputToWaveFile($it.path); $s.Speak($it.text); $s.SetOutputToNull() }
}
Write-Output 'DONE'
"""


class SapiSynth:
    """윈도우 기본 음성(System.Speech)으로 wav 를 만든다. PowerShell 한 번에 여러 문구."""

    def __init__(self, cache_dir: Path):
        self.cache = cache_dir
        self.cache.mkdir(parents=True, exist_ok=True)
        self.script = self.cache / "make_voice.ps1"
        # PowerShell 5 는 BOM 이 있어야 UTF-8 로 읽는다
        self.script.write_text(_PS_SCRIPT.replace("__RATE__", str(TTS_RATE)), encoding="utf-8-sig")
        self.voice = ""

    def _run(self, list_path: str = "", timeout: float = 120.0):
        cmd = ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(self.script)]
        if list_path:
            cmd += ["-list", list_path]
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                             creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        return out.returncode, out.stdout

    def check(self):
        """(쓸 수 있나, 음성 이름 또는 이유)."""
        if os.name != "nt" or not shutil.which("powershell"):
            return False, "윈도우가 아니다 (PowerShell System.Speech 없음)"
        try:
            code, out = self._run(timeout=30)
        except Exception as exc:
            return False, f"PowerShell 실행 실패: {exc}"
        if "NOVOICE" in out or code == 3:
            return False, "NOVOICE"
        for line in out.splitlines():
            if line.startswith("VOICE "):
                self.voice = line[6:].strip()
        return (code == 0), (self.voice or out.strip()[:80])

    def path_for(self, text: str) -> Path:
        h = hashlib.sha1(f"{self.voice}|{TTS_RATE}|{text}".encode("utf-8")).hexdigest()[:16]
        return self.cache / f"say_{h}.wav"

    def make(self, texts) -> dict:
        todo = [t for t in texts if not self.path_for(t).exists()]
        if todo:
            lst = self.cache / f"list_{os.getpid()}_{threading.get_ident()}.json"
            lst.write_text(json.dumps([{"text": t, "path": str(self.path_for(t))} for t in todo],
                                      ensure_ascii=False), encoding="utf-8")
            try:
                self._run(str(lst))
            finally:
                try:
                    lst.unlink()
                except OSError:
                    pass
        return {t: self.path_for(t) for t in texts if self.path_for(t).exists()}


class FakeSynth:
    """시험용 (--fake-tts): 글자 수에 비례한 길이의 낮은 음 wav. 윈도우 없이 대기열 동작만 본다."""

    def __init__(self, cache_dir: Path):
        self.cache = cache_dir
        self.cache.mkdir(parents=True, exist_ok=True)
        self.voice = "fake"

    def check(self):
        return True, "fake (시험용 — 말 대신 낮은 음)"

    def make(self, texts) -> dict:
        out = {}
        for t in texts:
            p = self.cache / f"fake_{hashlib.sha1(t.encode()).hexdigest()[:12]}.wav"
            if not p.exists():
                n = int(16000 * min(3.0, 0.12 * len(t)))
                import struct
                frames = b"".join(struct.pack("<h", int(3000 * math.sin(2 * math.pi * 330 * i / 16000)))
                                  for i in range(n))
                with wave.open(str(p), "wb") as w:
                    w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000); w.writeframes(frames)
            out[t] = p
        return out


def wav_seconds(path: Path) -> float:
    try:
        with wave.open(str(path), "rb") as w:
            return w.getnframes() / float(w.getframerate())
    except Exception:
        return float("nan")


VOICE_TIMEOUT_MARGIN = 3.0   # 말 길이 + 이 시간 안에 재생이 안 끝나면 끊고 다음 문구로 넘어간다 (2026-10-12)

# 삑 직후에 온 음성은 삑이 끝난 뒤 말한다 (2026-10-12).
# rx_probe 스크립트는 시작할 때 삑(ready)과 첫 음성을 같은 순간(1 ms 미만 간격)에 보내는데, 그때만 첫 음성이
# 끝나지 않고 걸렸다 (워치독은 음성 → 1.5초 뒤 삑이라 걸리지 않음). 같은 장치를 두 재생이 거의 동시에 여는 것을 피한다.
BEEP_AFTER_MARGIN = 0.1      # 삑이 끝난 뒤 이만큼 더 기다렸다가 말한다 (초)


def pattern_seconds(name: str) -> float:
    """삑 소리 하나의 길이 (PATTERNS 의 ms 합)."""
    return sum(ms for _, ms in PATTERNS.get(name, ())) / 1000.0

# --player winsound 일 때 자식 프로세스가 실행할 코드. 같은 프로세스의 PlaySound 는 한 번에 소리 하나라
# 삑(winsound SND_ASYNC)이 나는 순간 말이 끊긴다 -> 별도 프로세스로 낸다. 시간 초과 시 프로세스를 끝낸다.
_WINSOUND_CHILD = ("import sys, winsound; "
                   "winsound.PlaySound(sys.argv[1], winsound.SND_FILENAME | winsound.SND_NODEFAULT)")


class VoicePlayer:
    """말 wav 재생 (끝날 때까지 기다린다 — 말하기 스레드 안에서만 부른다).
    삑 소리와 **다른 경로**로 내서 서로 끊지 않는다: 윈도우 기본 = MCI(winmm), --player winsound = 별도 프로세스의
    PlaySound, --device = 별도 sounddevice 스트림, 리눅스 = paplay/aplay.

    2026-10-12: 첫 문구 재생이 끝나지 않고 걸려서 뒤 문구가 전부 "밀린 문구 버림" 이 된 일이 있었다
    (mci "play ... wait" 가 안 돌아옴. 짧은 단독 시험에서는 재현되지 않음 — 원인 미확인).
    그래서 모든 방식에 시간 제한(말 길이 + VOICE_TIMEOUT_MARGIN)을 두고, 넘으면 끊고 다음으로 간다."""

    def __init__(self, device=None, player: str = "mci"):
        self.device = device
        self.kind = None
        if device is not None:
            try:
                import sounddevice  # noqa: F401
                self.kind = "sd"
            except ImportError:
                self.kind = None
        if self.kind is None:
            if os.name == "nt":
                self.kind = "winsound" if player == "winsound" else "mci"
            elif shutil.which("paplay"):
                self.kind = "paplay"
            elif shutil.which("aplay"):
                self.kind = "aplay"
        self._n = 0
        self.timeouts = 0

    @staticmethod
    def time_limit(path: Path) -> float:
        sec = wav_seconds(path)
        return (sec if math.isfinite(sec) else 5.0) + VOICE_TIMEOUT_MARGIN

    def play_blocking(self, path: Path) -> bool:
        """끝까지 재생했으면 True, 시간 제한을 넘겨 끊었으면 False."""
        limit = self.time_limit(path)
        if self.kind == "mci":
            import ctypes
            winmm = ctypes.windll.winmm
            self._n += 1
            alias = f"say{self._n}"
            buf = ctypes.create_unicode_buffer(128)
            rc = winmm.mciSendStringW(f'open "{path}" type waveaudio alias {alias}', buf, 127, 0)
            if rc != 0:
                err = ctypes.create_unicode_buffer(256)
                winmm.mciGetErrorStringW(rc, err, 255)
                print(f"   🔴 [음성] mci 열기 실패 ({rc}): {err.value}")
                return True
            # "play ... wait" 는 장치가 안 끝내면 영원히 기다린다 -> 재생만 걸고 상태를 보며 기다린다
            winmm.mciSendStringW(f"play {alias}", buf, 127, 0)
            deadline = time.monotonic() + limit
            finished = True
            while True:
                winmm.mciSendStringW(f"status {alias} mode", buf, 127, 0)
                if buf.value != "playing":
                    break
                if time.monotonic() > deadline:
                    winmm.mciSendStringW(f"stop {alias}", buf, 127, 0)
                    finished = False
                    break
                time.sleep(0.05)
            winmm.mciSendStringW(f"close {alias}", buf, 127, 0)
            return finished
        elif self.kind == "winsound":
            try:
                subprocess.run([sys.executable, "-c", _WINSOUND_CHILD, str(path)],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=limit,
                               creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            except subprocess.TimeoutExpired:      # run() 이 자식 프로세스를 끝내고 다시 던진다
                return False
            return True
        elif self.kind == "sd":
            import numpy as np
            import sounddevice as sd
            with wave.open(str(path), "rb") as w:
                sr, raw = w.getframerate(), w.readframes(w.getnframes())
                ch = w.getnchannels()
            data = (np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0).reshape(-1, ch)
            with sd.OutputStream(samplerate=sr, channels=ch, device=self.device, dtype="float32") as st:
                st.write(data)
        elif self.kind in ("paplay", "aplay"):
            try:
                subprocess.run([self.kind, str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               timeout=limit)
            except subprocess.TimeoutExpired:
                return False
        return True


class Speaker:
    """말하기 스레드. say() 는 바로 돌아온다 (UDP 수신을 막지 않는다).
    대기열 = 최대 1개: 말하는 중에 새 문구가 오면 기다리던 이전 문구는 버린다."""

    def __init__(self, synth, player: VoicePlayer, verbose: bool = False):
        self.synth, self.player, self.verbose = synth, player, verbose
        self.cache: dict = {}
        self.cv = threading.Condition()
        self.pending = None
        self.dropped = 0
        self.spoken = 0
        self.beep_until = 0.0          # 지금 나는 삑이 끝나는 시각 (monotonic). 수신 스레드가 note_beep 로 갱신
        threading.Thread(target=self._run, daemon=True).start()

    def note_beep(self, name: str) -> None:
        """삑을 냈다 — 그 삑이 끝날 때까지 다음 말은 기다린다 (이미 말하는 중이면 영향 없음)."""
        sec = pattern_seconds(name)
        if sec > 0:
            self.beep_until = max(self.beep_until, time.monotonic() + sec + BEEP_AFTER_MARGIN)

    def prewarm(self, texts) -> None:
        t0 = time.time()
        self.cache.update(self.synth.make(list(texts)))
        long = [(t, wav_seconds(p)) for t, p in self.cache.items() if wav_seconds(p) > 2.0]
        print(f"[음성] 고정 문구 {len(self.cache)}개 준비 ({time.time() - t0:.1f}초)")
        for t, sec in long:
            print(f"   ⚠ 2초 넘음: {sec:.1f}초 — {t}")

    def say(self, text: str) -> None:
        with self.cv:
            if self.pending is not None:
                self.dropped += 1
                if self.verbose:
                    print(f"   [음성] 밀린 문구 버림: {self.pending}")
            self.pending = text
            self.cv.notify()

    def _run(self) -> None:
        while True:
            with self.cv:
                while self.pending is None:
                    self.cv.wait()
                text, self.pending = self.pending, None
            path = self.cache.get(text)
            if path is None:                      # 고정 문구가 아니면 그 자리에서 만든다 (1~2초)
                try:
                    path = self.synth.make([text]).get(text)
                except Exception as exc:
                    print(f"   [음성] 못 만듦: {exc}")
                    path = None
                if path is not None:
                    self.cache[text] = path
                with self.cv:                     # 만드는 동안 더 새 문구가 왔으면 이건 버린다
                    if self.pending is not None:
                        self.dropped += 1
                        continue
            if path is None:
                continue
            wait = self.beep_until - time.monotonic()
            if wait > 0:                          # 삑 직후에 온 음성 — 삑이 끝난 뒤 말한다
                if self.verbose:
                    print(f"   [음성] 삑 끝날 때까지 {wait:.2f}초 기다림: {text}")
                time.sleep(wait)
            try:
                if self.player.play_blocking(path):
                    self.spoken += 1
                else:
                    self.player.timeouts += 1
                    print(f"   🔴 [음성] {self.player.time_limit(path):.1f}초 안에 재생이 안 끝나 끊었다 "
                          f"(누적 {self.player.timeouts}회): {text}"
                          + ("  — 계속 나오면 --player winsound 로 띄워 보라" if self.player.kind == "mci" else ""))
            except Exception as exc:
                print(f"   [음성] 재생 실패: {exc}")


def build_speaker(args, device):
    """음성을 켤 수 있으면 Speaker, 아니면 None (이유를 한 줄 찍는다)."""
    if args.no_tts:
        print("[음성] 끔 (--no-tts)")
        return None
    cache = Path(tempfile.gettempdir()) / "cue_player_tts"
    synth = FakeSynth(cache) if args.fake_tts else SapiSynth(cache)
    ok, info = synth.check()
    if not ok:
        print(TTS_INSTALL_HINT if info == "NOVOICE" else f"🔴 [음성] 못 쓴다: {info} — 삑 소리만으로 계속한다")
        return None
    player = VoicePlayer(device, getattr(args, "player", "mci"))
    if player.kind is None:
        print("🔴 [음성] 재생할 방법이 없다 — 삑 소리만으로 계속한다")
        return None
    print(f"[음성] {info} · 재생 {player.kind} · 빠르기 {TTS_RATE}")
    sp = Speaker(synth, player, verbose=args.verbose)
    print("[음성] 고정 문구를 wav 로 만드는 중 (처음 한 번만 몇 초)...")
    sp.prewarm(VOICE_PHRASES)
    if not args.fake_tts:
        save_durations(sp, synth.voice)
    return sp


DURATIONS_FILE = Path(__file__).resolve().parent / "voice_durations.json"


def save_durations(sp: "Speaker", voice: str) -> None:
    """문구별 실제 말 길이를 tools/voice_durations.json 에 — 워치독이 "말 → 삑 → 측정" 대기 시간으로 쓴다.
    🔴 이 파일을 커밋해 로봇 PC 에 올려야 워치독이 쓴다 (없으면 글자 수로 넉넉히 어림)."""
    dur = {t: round(wav_seconds(p), 2) for t, p in sorted(sp.cache.items()) if math.isfinite(wav_seconds(p))}
    data = {"voice": voice, "rate": TTS_RATE, "made": time.strftime("%Y-%m-%dT%H:%M:%S"), "durations": dur}
    try:
        old = json.loads(DURATIONS_FILE.read_text(encoding="utf-8")) if DURATIONS_FILE.exists() else None
    except Exception:
        old = None
    if old and old.get("durations") == dur and old.get("voice") == voice:
        print(f"[음성] 말 길이 표 그대로: {DURATIONS_FILE.name} (가장 긴 문구 {max(dur.values()):.1f}초)")
        return
    DURATIONS_FILE.write_text(json.dumps(data, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"[음성] 말 길이 표 저장: {DURATIONS_FILE} — 🔴 커밋해서 로봇 PC 로 올려 주세요 "
          f"(가장 긴 문구 {max(dur.values()):.1f}초)")


class DevicePlayer:
    """특정 출력 장치로 재생 (sounddevice 필요). 장치를 고르지 않으면 쓰지 않는다."""

    def __init__(self, device):
        import numpy as np
        import sounddevice as sd
        self.sd, self.np = sd, np
        self.device = device
        self.waves = {}
        for name, pattern in PATTERNS.items():
            chunks = []
            for freq, ms in pattern:
                n = int(RATE * ms / 1000)
                if freq == 0:
                    chunks.append(np.zeros(n, dtype=np.float32))
                else:
                    t = np.arange(n, dtype=np.float32) / RATE
                    env = np.minimum(1.0, np.minimum(t * 200, (n / RATE - t) * 200))
                    chunks.append((0.5 * env * np.sin(2 * math.pi * freq * t)).astype(np.float32))
            self.waves[name] = np.concatenate(chunks)
        sd.check_output_settings(device=device, samplerate=RATE, channels=1)

    def play(self, name: str) -> None:
        if name in self.waves:
            self.sd.play(self.waves[name], RATE, device=self.device)


def build_default_beeper() -> SC.Beeper:
    saved = SC.PATTERNS
    SC.PATTERNS = PATTERNS
    try:
        return SC.Beeper(enabled=True)
    finally:
        SC.PATTERNS = saved


def list_devices() -> int:
    try:
        import sounddevice as sd
    except ImportError:
        print("sounddevice 가 없다. 설치: py -m pip install sounddevice")
        print("(설치 전에도 기본 장치로는 소리가 난다 — 윈도우 소리 설정에서 기본 장치를 VR 헤드셋으로 바꿔도 된다)")
        return 2
    print("출력 장치 (번호 또는 이름 일부를 --device 로 준다):")
    for i, d in enumerate(sd.query_devices()):
        if d["max_output_channels"] > 0:
            mark = " ← 기본" if i == sd.default.device[1] else ""
            print(f"  [{i:2d}] {d['name']}  ({d['hostapi']}){mark}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--device", default=None, help="출력 장치 번호 또는 이름 일부 (sounddevice 필요)")
    ap.add_argument("--list-devices", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--verbose", action="store_true", help="받은 패킷을 전부 찍는다 (신호 도달 확인)")
    ap.add_argument("--no-tts", action="store_true", help="음성 안내 끄기 (삑 소리만)")
    ap.add_argument("--player", choices=("mci", "winsound"), default="mci",
                    help="음성 wav 재생 방식 (윈도우, --device 를 안 줄 때). 기본 mci. "
                         "mci 로 말이 안 들리거나 '밀린 문구 버림' 만 나오면 winsound (별도 프로세스로 재생)")
    ap.add_argument("--fake-tts", action="store_true", help=argparse.SUPPRESS)   # 시험용: 말 대신 낮은 음
    a = ap.parse_args()

    if a.list_devices:
        return list_devices()

    player = None
    if a.device is not None:
        dev = int(a.device) if a.device.lstrip("-").isdigit() else a.device
        try:
            player = DevicePlayer(dev)
            print(f"출력 장치: {dev} (sounddevice)")
        except ImportError:
            print("🔴 sounddevice 가 없어서 --device 를 못 쓴다. 설치: py -m pip install sounddevice")
            print("   기본 장치로 진행한다")
        except Exception as exc:
            print(f"🔴 그 장치로 열 수 없다: {exc}")
            print("   --list-devices 로 확인하라. 기본 장치로 진행한다")
    beeper = None if player else build_default_beeper()
    if beeper is not None:
        warning = beeper.audio_sink_warning()
        if warning:
            print(f"🔴 [소리 경고] {warning}")
        elif beeper.player:
            print(f"출력: 기본 장치 ({' '.join(beeper.player)}) — 다른 장치로 내려면 --device")

    def play(name: str) -> None:
        (player or beeper).play(name)
        if speaker is not None:            # 안 B: 이 삑이 끝나기 전에 온 음성은 기다리게 한다
            speaker.note_beep(name)

    speaker = build_speaker(a, player.device if player else None)

    if a.self_test:
        for name in PATTERNS:
            print(f"  재생: {name}")
            play(name)
            time.sleep(1.0)
        if speaker is not None:
            for text in ("소리 시험입니다", "왼팔. 옆으로 만세", "끝났습니다. 힘이 빠집니다"):
                print(f"  음성: {text}")
                speaker.say(text)
                time.sleep(2.5)
            print("  음성 + 삑 겹침: 말하는 도중 높은음이 끊기지 않고 같이 들려야 한다")
            speaker.say("오른팔. 앞으로 만세")
            time.sleep(0.5)
            play("go")
            time.sleep(2.5)
        return 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((a.bind, a.port))
    print(f"구간 소리 대기 중 — {a.bind}:{a.port}  (Ctrl+C 로 종료. 세션이 끝나도 계속 켜 둔다)")
    print("신호가 와도 소리가 안 나면 --device 로 출력 장치를 지정하라. 신호 자체가 안 오면 IP·포트·방화벽 문제다.")
    count = 0
    try:
        while True:
            data, src = sock.recvfrom(4096)
            count += 1
            try:
                msg = json.loads(data.decode("utf-8"))
            except Exception:
                try:
                    raw = data.decode("utf-8").strip()
                except UnicodeDecodeError:
                    raw = ""
                if raw.startswith("say:"):
                    msg = {"sound": raw}
                else:
                    print(f"[{count}] {src[0]} 에서 알 수 없는 패킷 {len(data)}바이트")
                    continue
            name = str(msg.get("sound", ""))
            if name.startswith("say:"):
                text = name[4:].strip()
                print(f"[{count}] {time.strftime('%H:%M:%S')} {src[0]} → 음성  {text}"
                      + ("" if speaker else "  (음성 꺼짐)"))
                if speaker is not None and text:
                    speaker.say(text)
                continue
            text = msg.get("text") or msg.get("phase") or ""
            if a.verbose or name != "tick":
                known = "" if name in PATTERNS else "  🔴 모르는 소리 이름"
                print(f"[{count}] {time.strftime('%H:%M:%S')} {src[0]} → {name:<6} {text}{known}")
            if name in PATTERNS:
                play(name)
    except KeyboardInterrupt:
        print(f"\n종료 (받은 신호 {count}개)")
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
