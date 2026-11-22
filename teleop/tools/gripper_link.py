#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""그리퍼 연결 (REPORT_51) — 언리얼 grip 값 → 손마다 열림/닫힘 → 아두이노(ardgripper_rt) 로 "g A B".

grip 형식 (언리얼이 UDP JSON 끝에 붙임, 읽는 곳은 parse_grip() 한 곳):
    "grip": {"L": [src, pinch_cm], "R": [src, pinch_cm]}
        칸 0 = src      "hand"(손 추적) | "ctrl"(컨트롤러) | "none"
        칸 1 = pinch_cm 엄지 끝–검지 끝 거리 (cm), 못 읽으면 null
    몸 추적이 끊긴 패킷({"active": false, "joints": []})에도 붙는다. 옛 언리얼은 키가 없다.

판단 (손·컨트롤러 공통, GripDecider):
    pinch ≤ close_cm (3.0)                  → 바로 닫힘
    pinch ≥ open_cm (4.5) 가 open_hold_s (0.4초) 이상 이어짐 → 열림 (그 전에 3.0 이하로 돌아오면 닫힘 유지)
    그 사이 값 · src=none · pinch=null · grip 키 없음 · 패킷 끊김 → 직전 상태 유지 (열림 기다림도 처음부터)
    시작 상태 = 열림.
    근거 (rx_20261007_213131_grip): 맨손 집기 1.0~1.9 / 펴기 6.1~9.9, 컨트롤러 당김 2.2 / 놓음 5.9~7.4.
    방아쇠를 당긴 채 엄지를 떼면 0.25초 동안 7~8 로 튐 → 0.4초 기다림으로 열리지 않게.
작업 지시 14-p (open_rule="finger", 워치독 --grip-open-rule finger, 기본은 위 그대로): 컨트롤러 값은 단계값이다
    (당김 2.2~2.3 / 트리거에 손가락 얹힘 5.9~6.5 / 손가락 뗌 7.4~8.0 / 손 폄 9.2~10.8). 얹힘 값은 쥔 채로도 나온다 (11-11 1835 인계).
    ctrl 이면 ≥ 4.5 가 0.4초 이어졌을 때 값이 ≥ 7.0 이면 열림 (지금과 같은 시각), 4.5~7 만이면 mid_open_s (1.5초) 이어져야 열림, 반대 손이 닫힘 → 열림이 된 뒤 handover_guard_s (3초)
    동안은 중간값을 세지 않는다 (FingerGripDecider). hand 출처는 그대로.

아두이노 (GripperLink): L → A(9번 핀), R → B(10번 핀). 열림 0°, 닫힘 90° (옵션으로 바꿈).
    작업 지시 14-n: 실제로 쓰는 각도는 config/gripper.json 한곳 (grip_deg() 로 읽음, 기본 닫힘 60°). 이 모듈의 기본값 90 은 옛 동작 그대로.
    상태가 바뀔 때만 보낸다. 직렬은 별도 스레드 — 부르는 쪽(20 ms 제어 루프)은 기다리지 않는다.
    포트를 열면 보드가 재시작하고 "READY" 를 보낸다 → 그것을 기다린 뒤 보낸다 (최대 READY_TIMEOUT_S). 워밍업은 send_warmup() 으로 (14-h). 포트가 없거나 끊기면 2초마다 다시 붙고,
    다시 붙으면 마지막 목표를 다시 보낸다 (보드가 재시작해 잊었으므로). 끝낼 때 그리퍼는 그대로 둔다.

작업 지시 14-f D (재생기만 켠다, 기본 꺼짐 — 원격 조종은 위 그대로):
    record=True     보낸 각도·시각을 남긴다 (sent_log: 부른 시각, write_log: 직렬에 쓴 시각·까닭)
    poll_s          이 주기로 "?" 를 보내 보드 상태 (A 지금 목표 붙음 B …) 를 읽는다 → status
                    도중에 "READY" 가 오면 보드가 재시작한 것 (서보 시동 전류 등) → resets += 1, 마지막 목표를 다시 보낸다
    repeat_s        목표를 정한 뒤 repeat_for_s 동안 (상태가 '도착' 이면 그만) 이 주기로 같은 목표를 다시 보낸다
                    ardgripper_rt 는 같은 목표를 다시 받아도 그대로 간다 (재시작으로 잊었으면 다시 붙어 간다)
"""

from __future__ import annotations

import json
import queue
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

HANDS = ("L", "R")
CLOSE_CM = 3.0
OPEN_CM = 4.5
OPEN_HOLD_S = 0.4
OPEN, CLOSED = 0, 1
GRIP_CONFIG = Path(__file__).resolve().parent.parent / "config/gripper.json"   # 작업 지시 14-n
GRIP_CLOSE_DEFAULT = (60, 60)       # 파일이 없을 때 (2026-11-10 실측 L 55° 부터 안정)


def grip_deg(path=GRIP_CONFIG) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    """작업 지시 14-n: ((열림 A, 열림 B), (닫힘 A, 닫힘 B)) — config/gripper.json. A = L, B = R."""
    try:
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        o, c = d.get("open_deg", {}), d.get("close_deg", {})
        return (int(o.get("L", 0)), int(o.get("R", 0))), (int(c.get("L", GRIP_CLOSE_DEFAULT[0])), int(c.get("R", GRIP_CLOSE_DEFAULT[1])))
    except (OSError, ValueError, AttributeError):
        return (0, 0), GRIP_CLOSE_DEFAULT


def parse_grip(msg) -> Optional[Dict[str, Tuple[Optional[str], Optional[float]]]]:
    """패킷(dict) → {"L": (src, pinch), "R": (src, pinch)}. grip 키가 없거나 모양이 다르면 None / (None, None).
    칸 순서는 [src, pinch_cm] — 이 함수 말고 다른 곳에서 grip 을 직접 읽지 않는다."""
    g = msg.get("grip") if isinstance(msg, dict) else None
    if not isinstance(g, dict):
        return None
    out = {}
    for h in HANDS:
        v = g.get(h)
        if isinstance(v, (list, tuple)) and len(v) >= 2:
            src = v[0] if isinstance(v[0], str) else None
            pinch = float(v[1]) if isinstance(v[1], (int, float)) and not isinstance(v[1], bool) else None
            out[h] = (src, pinch)
        else:
            out[h] = (None, None)
    return out


class GripDecider:
    """손 하나의 열림/닫힘."""

    def __init__(self, close_cm: float = CLOSE_CM, open_cm: float = OPEN_CM, open_hold_s: float = OPEN_HOLD_S):
        self.close_cm, self.open_cm, self.open_hold_s = close_cm, open_cm, open_hold_s
        self.state = OPEN
        self.open_since: Optional[float] = None
        self.last_pinch: Optional[float] = None

    def update(self, src: Optional[str], pinch: Optional[float], t: float) -> int:
        self.last_pinch = pinch
        if src in (None, "none") or pinch is None:
            self.open_since = None
            return self.state
        if pinch <= self.close_cm:
            self.state = CLOSED
            self.open_since = None
        elif pinch >= self.open_cm:
            if self.state == CLOSED:
                if self.open_since is None:
                    self.open_since = t
                if t - self.open_since >= self.open_hold_s:
                    self.state = OPEN
                    self.open_since = None
        else:
            self.open_since = None
        return self.state


# --------------------------------------------------------------------------- 작업 지시 14-p (켰을 때만, 기본은 위 GripDecider)
FINGER_OFF_CM = 7.0          # 컨트롤러 값 ≥ 이것 = 손가락을 트리거에서 뗌 (7.4~8.0), 그 아래 4.5~7 = 손가락이 트리거에 얹힘 (5.9~6.5)
MID_OPEN_S = 1.5             # 중간값 (4.5~7) 만으로 열리려면 이만큼 이어져야 한다 (REPORT_gripopen §A·C)
HANDOVER_GUARD_S = 3.0       # 반대 손이 열린 뒤 이 동안은 중간값으로 열리지 않는다
RULE_KEEP, RULE_FINGER, RULE_MID, RULE_GUARD = 0, 1, 2, 3


class FingerGripDecider(GripDecider):
    """작업 지시 14-p --grip-open-rule finger: 출처가 ctrl 일 때만 열림 판정을 바꾼다 (hand 는 GripDecider 그대로).
    닫힘 ≤ close_cm 그대로. 닫힌 상태에서 ≥ open_cm 이 이어진 시간을 지금처럼 센다 (open_since). 그 시간이
        open_hold_s 이상이고 지금 값이 ≥ FINGER_OFF_CM                → 열림 (rule 1 손가락 뗌, 시각은 지금 판정과 같음)
        mid_open_s 이상 (중간값 4.5~7 만이어도)                       → 열림 (rule 2 중간 오래)
    반대 손이 열린 뒤 guard_until 전에는 중간값이 mid_open_s 시계를 세지 않는다 (지난 뒤 처음부터) → rule 3 (인계 보호로 막음).
    rule 은 열린 동안 1·2 를 유지하고, 닫히면 0."""

    def __init__(self, close_cm: float = CLOSE_CM, open_cm: float = OPEN_CM, open_hold_s: float = OPEN_HOLD_S,
                 off_cm: float = FINGER_OFF_CM, mid_open_s: float = MID_OPEN_S):
        super().__init__(close_cm, open_cm, open_hold_s)
        self.off_cm, self.mid_open_s = off_cm, mid_open_s
        self.mid_since: Optional[float] = None      # 중간값 시계 (인계 보호 동안은 멈춤)
        self.rule = RULE_KEEP
        self.src: Optional[str] = None

    def update(self, src: Optional[str], pinch: Optional[float], t: float, guard_until: Optional[float] = None) -> int:
        if src != self.src:                     # 출처가 바뀌면 기다림을 처음부터
            self.open_since = self.mid_since = None
            self.src = src
        if src != "ctrl" or pinch is None:
            self.mid_since = None
            before = self.state
            st = super().update(src, pinch, t)
            if st != before or (st == CLOSED and self.rule == RULE_GUARD):
                self.rule = RULE_KEEP
            return st
        self.last_pinch = pinch
        if pinch <= self.close_cm:
            self.state = CLOSED
            self.open_since = self.mid_since = None
            self.rule = RULE_KEEP
            return self.state
        if pinch < self.open_cm:
            self.open_since = self.mid_since = None
            if self.state == CLOSED:
                self.rule = RULE_KEEP
            return self.state
        if self.open_since is None:
            self.open_since = t
        guarded = guard_until is not None and t < guard_until
        if guarded:
            self.mid_since = None               # 인계 보호: 중간값 시계를 세지 않는다
        elif self.mid_since is None:
            self.mid_since = t
        if self.state == CLOSED:
            if pinch >= self.off_cm and t - self.open_since >= self.open_hold_s:
                self.state, self.rule = OPEN, RULE_FINGER
                self.open_since = self.mid_since = None
            elif self.mid_since is not None and t - self.mid_since >= self.mid_open_s:
                self.state, self.rule = OPEN, RULE_MID
                self.open_since = self.mid_since = None
            else:
                self.rule = RULE_GUARD if (guarded and pinch < self.off_cm) else RULE_KEEP
        return self.state


# 작업 지시 14-h: 보드는 켜지면 움직이지 않고 바로 READY (포트를 열면 재시작 약 0.5 초). 14-g 의 켜질 때 워밍업 (6 초 기다림) 은 없앴다 → 다시 3 초
READY_TIMEOUT_S = 3.0


class GripperLink:
    """아두이노와 직렬 통신 (별도 스레드). port=None 이면 아무것도 안 한다."""

    def __init__(self, port: Optional[str], baud: int = 115200, ready_timeout: float = READY_TIMEOUT_S,
                 retry_s: float = 2.0, log=print, record: bool = False, poll_s: Optional[float] = None,
                 repeat_s: Optional[float] = None, repeat_for_s: float = 1.5):
        self.port, self.baud, self.ready_timeout, self.retry_s = port, baud, ready_timeout, retry_s
        self.record, self.poll_s, self.repeat_s, self.repeat_for_s = record, poll_s, repeat_s, repeat_for_s
        self.sent_log: list = []      # (monotonic, "g A B") — send_angles 를 부른 때 (record)
        self.write_log: list = []     # (monotonic, "g A B", 까닭 new/repeat/ready) — 직렬에 쓴 때 (record)
        self.status = None            # (monotonic, A지금, A목표, A붙음, B지금, B목표, B붙음) — poll_s
        self.status_qwd = None        # (대기열, 워밍업 0/1, 버린 수) — 14-g 펌웨어부터. 옛 펌웨어면 None 그대로
        self.status_ms = None         # (움직이는 서보 0/1/2, 도착 뒤 기다림 0/1) — 14-i 펌웨어부터
        self.warm_sent_t = None       # send_warmup 을 직렬에 쓴 시각 (14-h)
        self._warm_req = False
        self._warm_deg = None         # 14-n: w 에 붙일 닫힘 각도 (A, B). None 이면 "w" 만 (펌웨어 기본)
        self.resets = 0               # 도중에 받은 READY (보드 재시작)
        self.writes = 0
        self._cmd_t = None
        self._last_write = -1e9
        self._last_poll = -1e9
        self._rx = b""
        self.log = log
        self._q: "queue.Queue[str]" = queue.Queue()
        self._last_cmd: Optional[str] = None          # 마지막으로 보내려 한 명령 (다시 붙으면 다시 보냄)
        self._stop = threading.Event()
        self.connected = False
        self.sent = 0
        self.errors = 0
        self._th = None
        if port:
            self._th = threading.Thread(target=self._run, name="gripper", daemon=True)
            self._th.start()

    def send_angles(self, a: int, b: int) -> None:
        cmd = f"g {int(a)} {int(b)}"
        self._last_cmd = cmd
        if self.record:
            self._cmd_t = time.monotonic()
            self.sent_log.append((self._cmd_t, cmd))
        if self.port:
            self._q.put(cmd)

    # ---- 14-f D (record / poll_s / repeat_s 를 줄 때만)
    def _wrote(self, cmd: str, why: str) -> None:
        self.writes += 1
        self._last_write = time.monotonic()
        if self.record:
            self.write_log.append((self._last_write, cmd, why))

    def _reached(self) -> bool:
        st, cmd = self.status, self._last_cmd
        if st is None or cmd is None or time.monotonic() - st[0] > 0.5:
            return False
        a, b = (int(v) for v in cmd.split()[1:3])
        return st[1] == st[2] == min(max(a, 0), 90) and st[4] == st[5] == min(max(b, 0), 90) and st[3] == 1 and st[6] == 1

    def _extra(self, s) -> None:
        now = time.monotonic()
        if (self.repeat_s and self._last_cmd and self._cmd_t is not None and now - self._cmd_t <= self.repeat_for_s
                and now - self._last_write >= self.repeat_s and not self._reached()):
            s.write((self._last_cmd + "\n").encode())
            self._wrote(self._last_cmd, "repeat")
        if self.poll_s and now - self._last_poll >= self.poll_s:
            self._last_poll = now
            s.write(b"?\n")

    def _parse(self, data: bytes) -> None:
        self._rx = (self._rx + data)[-512:]
        while b"\n" in self._rx:
            line, self._rx = self._rx.split(b"\n", 1)
            text = line.decode("ascii", "replace").strip()
            if text == "READY":                               # 보드가 다시 켜졌다 → 서보가 풀리고 목표를 잊었다
                self.resets += 1
                self.log(f"\n[그리퍼] 🔴 보드가 재시작했다 (READY, {self.resets}번째) — 마지막 목표를 다시 보낸다: {self._last_cmd}")
                if self._last_cmd:
                    self._q.put(self._last_cmd)
                    self._cmd_t = time.monotonic()
                    if self.record:
                        self.write_log.append((self._cmd_t, self._last_cmd, "ready"))
            elif text.startswith("A ") and " B " in text:
                p = text.split()
                try:
                    self.status = (time.monotonic(), int(p[1]), int(p[2]), int(p[3]), int(p[5]), int(p[6]), int(p[7]))
                    if len(p) >= 14 and p[8] == "Q" and p[10] == "W" and p[12] == "D":     # 14-g 뒤에 붙인 칸
                        self.status_qwd = (int(p[9]), int(p[11]), int(p[13]))
                    if len(p) >= 18 and p[14] == "M" and p[16] == "S":     # 14-i 뒤에 붙인 칸
                        self.status_ms = (int(p[15]), int(p[17]))
                except (ValueError, IndexError):
                    pass

    def send_warmup(self, close_deg=None) -> None:
        """14-h: 보드에 w (워밍업: 열림 → 닫힘 → 열림) 를 보낸다. 다음 직렬 차례에 쓴다 (g 대기열과 섞이지 않게 따로).
        14-n: close_deg = (A, B) 면 "w A B" (그 각도까지 닫음). 14-n 전 펌웨어는 뒤 숫자를 무시하고 90 까지 닫는다."""
        if self.port:
            self._warm_deg = None if close_deg is None else tuple(max(0, min(90, int(v))) for v in close_deg)
            self._warm_req = True

    def close(self) -> None:
        """끝낼 때: 그리퍼는 그대로 둔다 (열지 않는다). 스레드만 멈춘다."""
        self._stop.set()
        if self._th is not None:
            self._th.join(timeout=1.0)

    # ---- 스레드
    def _open(self):
        import serial
        s = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.5)
        t0 = time.monotonic()
        buf = b""
        while time.monotonic() - t0 < self.ready_timeout and not self._stop.is_set():
            buf += s.read(64)
            if b"READY" in buf:
                break
        if b"READY" not in buf:
            self.log(f"[그리퍼] {self.port}: READY 를 못 받았다 ({self.ready_timeout:.0f}초) — 그래도 보낸다")
        return s

    def _run(self):
        s = None
        while not self._stop.is_set():
            if s is None:
                try:
                    s = self._open()
                    self.connected = True
                    self.log(f"[그리퍼] {self.port} 연결")
                    if self._last_cmd:                       # 보드가 재시작해 잊었다 → 마지막 목표를 다시
                        with self._q.mutex:
                            self._q.queue.clear()
                        self._q.put(self._last_cmd)
                except Exception as exc:                     # 포트 없음·권한 등
                    if self.connected or self.errors == 0:
                        self.log(f"[그리퍼] {self.port} 못 엶: {exc} — {self.retry_s:.0f}초마다 다시")
                    self.connected = False
                    self.errors += 1
                    self._stop.wait(self.retry_s)
                    continue
            try:
                try:
                    cmd = self._q.get(timeout=0.05)
                except queue.Empty:
                    cmd = None
                if cmd is not None:
                    while not self._q.empty():                # 밀린 것은 마지막 것만
                        cmd = self._q.get_nowait()
                    s.write((cmd + "\n").encode())
                    self.sent += 1
                    self._wrote(cmd, "new")
                if self._warm_req:                            # 14-h: w (재생기만 부른다)
                    self._warm_req = False
                    wcmd = "w" if self._warm_deg is None else f"w {self._warm_deg[0]} {self._warm_deg[1]}"
                    s.write((wcmd + "\n").encode())
                    self.warm_sent_t = time.monotonic()
                    if self.record:
                        self.write_log.append((self.warm_sent_t, wcmd, "warmup"))
                if self.repeat_s or self.poll_s:              # 14-f D (재생기만): 도착할 때까지 다시 보내기, 상태 읽기
                    self._extra(s)
                data = s.read(256)                            # 응답(OK 등)은 버린다
                if self.poll_s:
                    self._parse(data)
            except Exception as exc:
                self.log(f"[그리퍼] 연결 끊김: {exc} — 다시 붙는다")
                try:
                    s.close()
                except Exception:
                    pass
                s = None
                self.connected = False
                self.errors += 1
        if s is not None:
            try:
                s.close()
            except Exception:
                pass


class GripperController:
    """판단 두 개 + 아두이노. 워치독·도구가 쓴다. update() 는 새 패킷마다, 사람을 따라갈 때만 부른다."""

    def __init__(self, port: Optional[str], open_deg=(0, 0), close_deg=(90, 90), close_cm: float = CLOSE_CM,
                 open_cm: float = OPEN_CM, open_hold_s: float = OPEN_HOLD_S, log=print, link: Optional[GripperLink] = None,
                 open_rule: str = "pinch", mid_open_s: float = MID_OPEN_S, handover_guard_s: float = HANDOVER_GUARD_S):
        self.open_deg, self.close_deg = tuple(open_deg), tuple(close_deg)
        self.open_rule = open_rule              # 작업 지시 14-p: "pinch" (기본, 예전 그대로) | "finger"
        if open_rule == "finger":
            self.dec = {h: FingerGripDecider(close_cm, open_cm, open_hold_s, mid_open_s=mid_open_s) for h in HANDS}
        else:
            self.dec = {h: GripDecider(close_cm, open_cm, open_hold_s) for h in HANDS}
        self.handover_guard_s = handover_guard_s
        self.guard_until: Dict[str, Optional[float]] = {h: None for h in HANDS}
        self.link = link if link is not None else GripperLink(port, log=log)
        self.changes = 0

    def rules(self) -> Dict[str, int]:
        """작업 지시 14-p: 손마다 판정 이유 (0 그대로, 1 손가락 뗌, 2 중간 오래, 3 인계 보호로 막음). finger 가 아니면 늘 0."""
        return {h: getattr(self.dec[h], "rule", RULE_KEEP) for h in HANDS}

    def state(self) -> Dict[str, int]:
        return {h: self.dec[h].state for h in HANDS}

    def pinch(self) -> Dict[str, Optional[float]]:
        return {h: self.dec[h].last_pinch for h in HANDS}

    def angles(self) -> Tuple[int, int]:
        st = self.state()
        a = self.close_deg[0] if st["L"] == CLOSED else self.open_deg[0]
        b = self.close_deg[1] if st["R"] == CLOSED else self.open_deg[1]
        return a, b

    def update(self, msg, t: float) -> Dict[str, int]:
        before = self.state()
        g = parse_grip(msg)
        for h in HANDS:
            src, pinch = (g or {}).get(h, (None, None)) if g else (None, None)
            if self.open_rule == "finger":
                self.dec[h].update(src, pinch, t, self.guard_until[h])
            else:
                self.dec[h].update(src, pinch, t)
        after = self.state()
        if self.open_rule == "finger":          # 14-p 인계 보호: 한 손이 닫힘 → 열림이면 다른 손을 지킨다
            for h, o in (("L", "R"), ("R", "L")):
                if before[h] == CLOSED and after[h] == OPEN:
                    self.guard_until[o] = t + self.handover_guard_s
        if after != before:
            self.changes += 1
            self.link.send_angles(*self.angles())
        return after

    def close(self) -> None:
        self.link.close()


# --------------------------------------------------------------------------- 작업 지시 14-j: 보드 없이 펌웨어와 같은 시간
class ServoModel:
    """ardgripper_rt (14-i) 와 같은 시간 모델: 대기열 (같은 목표 연달아 합침), 한 명령에서 바뀌는 서보를 하나씩
    (닫는 쪽 = 각도가 커지는 쪽 먼저, 같은 쪽이면 A 먼저), 1° / STEP_S, 명령값이 닿으면 SETTLE_S 쉬고 다음.
    command(now, a, b) 로 넣고 angles(now) / idle(now) 로 읽는다. 펌웨어를 바꾸면 여기도 같이 바꾼다."""

    STEP_S = 0.010
    SETTLE_S = 0.400

    def __init__(self, a0: int = 0, b0: int = 0, first: str = "A"):
        self.cur = {"A": int(a0), "B": int(b0)}
        self.first = first
        self.q: list = []
        self.cmd = (int(a0), int(b0))
        self.stages: list = []        # [(서보, 목표)]
        self.settle_until = None
        self.t = None

    def command(self, now: float, a: int, b: int) -> None:
        self.update(now)
        a, b = max(0, min(90, int(a))), max(0, min(90, int(b)))
        last = self.q[-1] if self.q else self.cmd
        if (a, b) != last:
            self.q.append((a, b))

    def _start(self, a: int, b: int) -> None:
        self.cmd = (a, b)
        need = [k for k, v in (("A", a), ("B", b)) if self.cur[k] != v]
        if len(need) == 2:
            ca, cb = a > self.cur["A"], b > self.cur["B"]
            f = ("A" if ca else "B") if ca != cb else self.first
            o = "B" if f == "A" else "A"
            self.stages = [(f, a if f == "A" else b), (o, a if o == "A" else b)]
        else:
            self.stages = [(k, a if k == "A" else b) for k in need]

    def update(self, now: float) -> None:
        if self.t is None:
            self.t = now
        while self.t + 1e-9 < now:
            if self.settle_until is not None:
                if now < self.settle_until:
                    self.t = now
                    break
                self.t = self.settle_until
                self.settle_until = None
                self.stages.pop(0)
                continue
            if not self.stages:
                if not self.q:
                    self.t = now
                    break
                self._start(*self.q.pop(0))
                continue
            k, v = self.stages[0]
            if self.cur[k] == v:
                self.settle_until = self.t + self.SETTLE_S
                continue
            n = min(abs(v - self.cur[k]), int((now - self.t) / self.STEP_S + 1e-9))
            if n <= 0:
                break
            self.cur[k] += n if v > self.cur[k] else -n
            self.t += n * self.STEP_S

    def angles(self, now: float):
        self.update(now)
        return self.cur["A"], self.cur["B"]

    def idle(self, now: float) -> bool:
        self.update(now)
        return not self.q and not self.stages and self.settle_until is None

    def duration(self, a0: int, b0: int, a1: int, b1: int) -> float:
        """(a0, b0) 에서 (a1, b1) 까지 끝나는 시간 (쉼 포함) — 한 번에 하나씩."""
        return sum(abs(x1 - x0) * self.STEP_S + self.SETTLE_S for x0, x1 in ((a0, a1), (b0, b1)) if x1 != x0)


class TimedVirtualGrip:
    """작업 지시 14-j --grip-virtual-timing real: 가상 그리퍼 (보드 없음) 를 실제 속도로. gripper (GripperController) 의 판단이
    바뀌어 send_angles 가 불릴 때 ServoModel 에 넣고, state() 는 '다 닫힌 손 = 1, 다 열린 손 = 0, 움직이는 중 = 앞 값' 을 준다
    (가상 캔은 다 닫힌 뒤에만 잡고, 다 열린 뒤에만 놓는다). on_reach(손, 'closed'|'open') 를 다 닫힘·다 열림 때 부른다."""

    def __init__(self, gripper, on_reach=None, clock=time.monotonic):
        self.g, self.on_reach, self.clock = gripper, on_reach, clock
        a0, b0 = gripper.angles()
        self.model = ServoModel(a0, b0)
        self.eff = dict(gripper.state())
        link = gripper.link
        orig = link.send_angles

        def send(a, b):
            self.model.command(self.clock(), a, b)
            orig(a, b)
        link.send_angles = send

    def update(self, now: float) -> None:
        ang = dict(zip(HANDS, self.model.angles(now)))
        for h, i in zip(HANDS, range(2)):
            close, opn = self.g.close_deg[i], self.g.open_deg[i]
            new = 1 if ang[h] == close else (0 if ang[h] == opn else self.eff[h])
            if new != self.eff[h]:
                self.eff[h] = new
                if self.on_reach is not None:
                    self.on_reach(h, "closed" if new else "open")

    def state(self) -> Dict[str, int]:
        return dict(self.eff)

    def angles_now(self, now: float):
        return self.model.angles(now)
