#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""위팔 측정 순서 (REPORT_52 §5.1) — rx_probe --script upper 가 쓴다 (로봇 없이 녹화만).

한 팔씩 (다른 팔은 차렷), 왼팔 → 오른팔.
  1부: 팔을 쭉 편 채 6자세 — 앞 45°, 앞 어깨 높이, 옆 45°, 옆 어깨 높이, 앞으로 머리 위, 몸 앞을 가로질러 안쪽
  2부: 팔꿈치 90° 굽힌 채 (손은 앞으로) 같은 6자세 — 위팔 각은 1부와 같게
규칙 (혼자 헤드셋을 쓰고 하므로 음성만으로 진행):
  자세 문구 → (말 길이 + 0.5 + 1.5초) → 삑 → 유지 HOLD_SEC 초 (조용히, 초읽기 없음) → "내리세요" → 차렷 REST_SEC 초 (조용히)
구간 이름 (jsonl 의 phase): <팔>_<s|b>_<자세>  예) L_s_fwd45, R_b_side90.  s = 편 팔, b = 팔꿈치 90°.
분석: analyze_upper_arm_gain.py poses <녹화>.

steps(): (구간 이름, 음성, 화면 글, 유지 초, 종류)
  종류 pose = 측정, intro = 안내(측정 안 함), switch = 팔 바꿈, end = 끝
"""

from __future__ import annotations

HOLD_SEC = 5.0      # 자세 유지 (분석은 뒤 3초)
REST_SEC = 4.0      # "내리세요" 뒤 차렷
SEP = "\n"          # 한 안내에 문구 여럿 (차례로 말함)

ARM = {"L": "왼팔", "R": "오른팔"}
POSES = ("fwd45", "fwd90", "side45", "side90", "up", "in")
POSE_TEXT = {"fwd45": "앞 45°", "fwd90": "앞 어깨 높이", "side45": "옆 45°", "side90": "옆 어깨 높이",
             "up": "위로", "in": "안쪽 (몸 앞 가로지름)"}
# 기대 사람 위팔 각 (분석 표에 같이 적는다): (앞 들기, 옆 들기, 들어 올림) — None 은 비교 안 함
POSE_EXPECT = {"fwd45": (45, 0, 45), "fwd90": (90, 0, 90), "side45": (0, 45, 45), "side90": (0, 90, 90),
               "up": (None, None, None), "in": (None, None, None)}
SAY_STRAIGHT = {"fwd45": "편 채로 앞으로 사십오도", "fwd90": "편 채로 앞으로 어깨 높이",
                "side45": "편 채로 옆으로 사십오도", "side90": "편 채로 옆으로 어깨 높이",
                "up": "편 채로 앞으로 머리 위까지", "in": "편 채로 몸 앞을 가로질러 안쪽으로"}
SAY_BENT = {"fwd45": "팔꿈치 구십도, 위팔을 앞으로 사십오도", "fwd90": "팔꿈치 구십도, 위팔을 앞으로 어깨 높이",
            "side45": "팔꿈치 구십도, 위팔을 옆으로 사십오도", "side90": "팔꿈치 구십도, 위팔을 옆으로 어깨 높이",
            "up": "팔꿈치 구십도, 위팔을 최대한 위로", "in": "팔꿈치 구십도, 위팔을 몸 앞 가운데로"}
SAY_START = "위팔 측정을 시작합니다"
SAY_STRAIGHT_INTRO = "팔을 쭉 편 채로 합니다. 다른 팔은 차렷"
SAY_BENT_INTRO = "이제 팔꿈치를 구십도로 굽힌 채 합니다. 손은 앞으로"
SAY_REST = "차렷"                 # 기존 문구
SAY_LOWER = "내리세요"            # 기존 문구
SAY_SWITCH = "오른팔로 바꿉니다"   # 기존 문구
SAY_END = "끝났습니다"            # 기존 문구


def steps():
    out = []
    for part, intro, say, tag in (("straight", SAY_STRAIGHT_INTRO, SAY_STRAIGHT, "s"),
                                  ("bent", SAY_BENT_INTRO, SAY_BENT, "b")):
        out.append((f"{part}_intro", intro, f"{'편 팔' if tag == 's' else '팔꿈치 90°'} 시작 안내", 0.0, "intro"))
        for A in ("L", "R"):
            if A == "R":
                out.append(("switch_arm", SAY_SWITCH, "오른팔로 바꿈 — 왼팔 차렷", 0.0, "switch"))
            for p in POSES:
                out.append((f"{A}_{tag}_{p}", f"{ARM[A]}. {say[p]}",
                            f"{ARM[A]} — {'쭉 편 채' if tag == 's' else '팔꿈치 90°'} {POSE_TEXT[p]} (다른 팔 차렷)",
                            HOLD_SEC, "pose"))
    out.append(("end", SAY_END, "끝", 0.0, "end"))
    return out


def phrases():
    fixed = {SAY_START, SAY_REST, SAY_LOWER}
    return sorted(fixed | {part for s in steps() for part in s[1].split(SEP) if part})


def pose_names():
    return [s[0] for s in steps() if s[4] == "pose"]
