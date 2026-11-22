#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""양손 장면 녹화 순서 (REPORT_54 P2) — rx_probe --script both 가 쓴다. 2026-10-14: 18자세 → 7자세, 컨트롤러로.

로봇은 움직이지 않는다. 실행: 워치독 --dry-run --auto-start --no-start-check --log-rx (사람 뼈 + 로봇 계획값 기록)
                         + rx_probe --script both --cue-only (음성 안내만, 구간 시각 CSV)
컨트롤러를 쥐고 한다 (맨손 아님). 방아쇠는 내내 놓은 채.
규칙 (upper_session 과 같음): 문구 → (말 길이 + 0.5 + 1.5초) → 삑 → 유지 HOLD_SEC 초 (조용히) → "내리세요" → 차렷 REST_SEC 초

분석에서 보는 것 (REPORT_54 §5)
  together_chest / together_belly : 양손을 모을 때 두 손 높이 어긋남의 원인 (사람 손 높이 차 vs 로봇 계획 높이 차)
  hold_chest / hold_belly         : R1 판정용 — 두 손 마주 봄 각, 사람 손 간격 → 로봇 손목 간격·모델 손 겉면
  pour                            : 컵 붓기 때 로봇 손목 간격 (닿음 규칙 14 cm 와 비교)
  in_L / in_R                     : 한 팔 안쪽 끝 — 사람 손목 가운데선 거리 vs 로봇 (가운데선 매핑 P3)

steps(): (구간 이름, 음성, 화면 글, 유지 초, 종류)   종류 pose = 측정, intro = 안내, end = 끝
"""

from __future__ import annotations

HOLD_SEC = 5.0
REST_SEC = 4.0
SEP = "\n"

SAY_START = "양손 측정을 시작합니다. 컨트롤러를 쥐고 차렷"
SAY_REST = "차렷"                 # 기존 문구
SAY_LOWER = "내리세요"            # 기존 문구
SAY_END = "끝났습니다"            # 기존 문구

POSES = [
    ("together_chest", "양손 모으기, 가슴 높이", "컨트롤러 쥔 채 가슴 앞에서 두 컨트롤러 앞끝이 닿을락 말락"),
    ("together_belly", "양손 모으기, 배 높이", "같은 동작, 배꼽 앞"),
    ("hold_chest", "컵 하나 사이에 두고 잡기, 가슴 높이", "두 컨트롤러 앞끝을 마주 보게, 사이를 컵 하나 들어갈 만큼"),
    ("hold_belly", "컵 하나 사이에 두고 잡기, 배 높이", "같은 동작, 배꼽 앞"),
    ("pour", "컵 붓기", "평소처럼 한 손은 받는 컵, 다른 손은 기울여 붓는 자세"),
    ("in_L", "왼손만 배 쪽 끝까지", "왼팔 위팔 붙이고 팔꿈치 90°, 손을 배 쪽 끝까지 (오른팔 차렷)"),
    ("in_R", "오른손만 배 쪽 끝까지", "오른팔 위팔 붙이고 팔꿈치 90°, 손을 배 쪽 끝까지 (왼팔 차렷)"),
]


def steps():
    out = [(name, say, text, HOLD_SEC, "pose") for name, say, text in POSES]
    out.append(("end", SAY_END, "끝", 0.0, "end"))
    return out


def phrases():
    fixed = {SAY_START, SAY_REST, SAY_LOWER}
    return sorted(fixed | {part for s in steps() for part in s[1].split(SEP) if part})


def pose_names():
    return [s[0] for s in steps() if s[4] == "pose"]
