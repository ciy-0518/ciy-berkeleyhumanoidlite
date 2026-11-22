#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""그리퍼 입력 측정 순서 (REPORT_50) — rx_probe --script grip 이 쓴다 (로봇 없이 녹화만).

언리얼이 UDP JSON 끝에 "grip":{"L":[src, pinch_cm],"R":[...]} 를 붙인다. src = hand | ctrl | none, pinch_cm = 엄지 끝–검지 끝
거리(cm, 못 읽으면 null). 컨트롤러는 방아쇠 당김 2.3 / 놓음 7.8 (예상), 맨손은 집기 0.5~1 / 펴기 11~13 (예상).
규칙: 음성 예고 → (말 길이 + 0.5 + 1.5초) → 삑 → 멈춤(측정). 멈춤 동안 말하지 않는다. 구간 이름은 jsonl 의 phase.

steps(): (구간 이름, 음성, 화면 글, 길이 초, 종류, 기대)
  종류 hold = 3초 멈춤 측정, wait = 바꾸는 시간(측정 안 함), end = 끝 안내
  기대 = {"src": hand|ctrl|None, "L": small|large|None, "R": small|large|None, "active": True|False|None}
"""

from __future__ import annotations

HOLD_SEC = 3.0
SMALL_CM = 4.0       # 집기·당기기: 이보다 작으면 맞음 (예상 맨손 0.5~1, 컨트롤러 2.3)
LARGE_CM = 6.0       # 펴기·놓기: 이보다 크면 맞음 (예상 맨손 11~13, 컨트롤러 7.8)
MTU_B = 1252         # 이보다 큰 패킷은 쪼개질 수 있다 (REPORT_37)
SEP = "\n"


def _h(name, voice, text, src, L, R, kind="hold", dur=HOLD_SEC, active=True):
    return (name, voice, text, dur, kind, {"src": src, "L": L, "R": R, "active": active})


def steps():
    S, B = "small", "large"
    out = [_h("hand_switch", "컨트롤러를 내려놓고 맨손으로 합니다", "컨트롤러를 내려놓고 맨손 — 손 추적으로 바뀔 시간",
              None, None, None, kind="wait", dur=8.0)]
    for i in (1, 2, 3):
        out.append(_h(f"hand_open_{i}", "양손 펴기", "맨손 — 양손 펴기", "hand", B, B))
        out.append(_h(f"hand_pinch_{i}", "양손 집기", "맨손 — 양손 집기 (엄지 끝과 검지 끝)", "hand", S, S))
    out += [_h("hand_L_pinch", "왼손만 집기", "맨손 — 왼손만 집기 (오른손 편 채)", "hand", S, B),
            _h("hand_open_4", "양손 펴기", "맨손 — 양손 펴기", "hand", B, B),
            _h("hand_R_pinch", "오른손만 집기", "맨손 — 오른손만 집기 (왼손 편 채)", "hand", B, S),
            _h("hand_open_5", "양손 펴기", "맨손 — 양손 펴기", "hand", B, B),
            _h("ctrl_switch", "컨트롤러를 쥐세요", "컨트롤러 쥐기 — 컨트롤러로 바뀔 시간", None, None, None,
               kind="wait", dur=10.0)]
    for i in (1, 2, 3):
        out.append(_h(f"ctrl_rel_{i}", "방아쇠 놓기", "컨트롤러 — 방아쇠 놓기", "ctrl", B, B))
        out.append(_h(f"ctrl_pull_{i}", "양손 방아쇠 당기기", "컨트롤러 — 양손 방아쇠 당기기", "ctrl", S, S))
    out += [_h("ctrl_L_pull", "왼손만 당기기", "컨트롤러 — 왼손만 방아쇠 당기기", "ctrl", S, B),
            _h("ctrl_rel_4", "방아쇠 놓기", "컨트롤러 — 방아쇠 놓기", "ctrl", B, B),
            _h("ctrl_R_pull", "오른손만 당기기", "컨트롤러 — 오른손만 방아쇠 당기기", "ctrl", B, S),
            _h("ctrl_rel_5", "방아쇠 놓기", "컨트롤러 — 방아쇠 놓기", "ctrl", B, B),
            # 엄지 영향: 엄지를 버튼에 올려도(놓은 채) 큰 값, 당기면 작은 값, 엄지를 떼도(당긴 채) 작은 값
            _h("ctrl_thumb_on", "엄지를 버튼 위에 올리기", "컨트롤러 — 양손 엄지를 버튼 위에 (방아쇠는 놓은 채)", "ctrl", B, B),
            _h("ctrl_thumb_pull", "그대로 방아쇠 당기기", "컨트롤러 — 엄지 올린 채 양손 방아쇠 당기기", "ctrl", S, S),
            _h("ctrl_thumb_off", "엄지를 떼기", "컨트롤러 — 방아쇠 당긴 채 엄지만 떼기", "ctrl", S, S),
            _h("ctrl_rel_6", "방아쇠 놓기", "컨트롤러 — 방아쇠 놓기", "ctrl", B, B),
            _h("hmd_off", "헤드셋을 벗으세요", "헤드셋 벗기 (몸 추적 끊김 → active false 에도 grip 이 붙는지)",
               None, None, None, kind="wait", dur=5.0, active=False),
            _h("hmd_on", "다시 쓰세요", "헤드셋 다시 쓰기", None, None, None, kind="wait", dur=5.0, active=None),
            _h("end", "끝났습니다", "끝", None, None, None, kind="end", dur=2.0, active=None)]
    return out


def phrases():
    return sorted({part for s in steps() for part in s[1].split(SEP)})


def info(name):
    for s in steps():
        if s[0] == name:
            return s
    return None


if __name__ == "__main__":
    for n, v, text, d, k, e in steps():
        print(f"{n:16s} {k:4s} {d:4.1f}s  음성: {v}")
    print(f"구간 {len(steps())}개, 문구 {len(phrases())}개")
