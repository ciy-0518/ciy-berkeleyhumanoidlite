#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""손목 회전 측정 순서 (REPORT_49) — rx_probe --script wrist 와 워치독 --auto-session wrist 가 같이 쓴다.

목적: 두 손이 겹칠 때 손목 회전 입력이 이상하다는 관찰 확인. 컨트롤러를 쥐고 팔꿈치 90°.
손목 동작은 모든 자세에서 같다: 컵을 똑바로(엄지 위) → 안쪽으로 눕히기(손바닥 아래) → 똑바로 → 바깥쪽으로 눕히기(손바닥 위),
각 HOLD_SEC 멈춤. 자세: a 어깨 너비로 앞 / b 가슴 앞 나란히 / c 왼손 멀리·오른손 가까이 / d 반대 / e 한 손이 주먹 하나 위.

구간 이름: w_intro, <자세>_pos (자리 잡기), <자세>_<손>_<모양>, rest
  손 = L / R (a, b), far / near (c, d: 먼 손 / 가까운 손), top (e: 위 손)
  모양 = up1 (똑바로) · in (안쪽으로 눕히기) · up2 (다시 똑바로) · out (바깥쪽으로 눕히기)
음성 규칙 (사용자 지시): 일상 말, 한 문구에 한 가지, 위치 문구와 손목 문구를 나눔, 자리 잡을 시간 3초, 문구는 3초 안.
"""

from __future__ import annotations

HOLD_SEC = 3.0           # 손목 모양마다 멈춤 (측정 구간)
SETTLE_SEC = 3.0         # 손 위치를 바꾼 뒤 자리 잡는 시간
REST_SEC = 4.0

INTRO = "팔은 그대로 두고 손목만 돌립니다"
SHAPES = (("up1", "컵을 똑바로"), ("in", "컵을 안쪽으로 눕히기"), ("up2", "컵을 똑바로"), ("out", "컵을 바깥쪽으로 눕히기"))
SHAPES_E = (("up1", "컵을 똑바로"), ("in", "컵을 안쪽으로 눕히기"), ("up2", "컵을 똑바로"))
# 자세: (이름, 위치 문구, 화면 글, [(손 표기, 손 말, 그 손이 실제로 어느 쪽인지)], 모양들)
POSES = (
    ("a", "양손 앞으로, 어깨 너비", "양손을 어깨 너비로 벌려 앞으로 (팔꿈치 90°)",
     (("L", "왼손", "left"), ("R", "오른손", "right")), SHAPES),
    ("b", "양손을 가슴 앞에 붙이기", "양손을 가슴 앞에서 좌우로 나란히 붙임",
     (("L", "왼손", "left"), ("R", "오른손", "right")), SHAPES),
    ("c", "왼손 멀리, 오른손 가까이", "왼손을 멀리·오른손을 몸 가까이 (눈에서 보면 왼손이 오른손을 가림)",
     (("far", "먼 손", "left"), ("near", "가까운 손", "right")), SHAPES),
    ("d", "오른손 멀리, 왼손 가까이", "오른손을 멀리·왼손을 몸 가까이 (오른손이 왼손을 가림)",
     (("far", "먼 손", "right"), ("near", "가까운 손", "left")), SHAPES),
    ("e1", "왼손을 주먹 하나 위로", "왼손이 오른손보다 주먹 하나 위 (붓는 자세)",
     (("top", "위 손", "left"),), SHAPES_E),
    ("e2", "오른손을 주먹 하나 위로", "오른손이 왼손보다 주먹 하나 위 (붓는 자세)",
     (("top", "위 손", "right"),), SHAPES_E),
)


SEP = "\n"   # 한 예고 안에서 따로 말할 문구 사이 (손 고르기 → 손목 동작). 워치독·rx_probe 가 차례로 말한다


def steps():
    """[(구간 이름, 음성 예고, 화면 글, 길이 초, 종류)] — 종류: intro / pos / shape / rest.
    음성 예고는 그 구간 **앞에서** 말한다 (말 → 삑 → 그 구간 시작). rest 는 "차렷" 을 그 구간 안에서.
    손이 바뀌는 첫 모양은 "왼손" + SEP + "컵을 똑바로" 처럼 두 문구 (한 문구에 한 가지, 3초 안)."""
    out = [("w_intro", INTRO, "팔은 그대로 두고 손목만 돌립니다 (컨트롤러 쥐고 팔꿈치 90°)", 1.0, "intro")]
    for i, (p, pos_say, pos_text, hands, shapes) in enumerate(POSES):
        out.append((f"{p}_pos", pos_say, pos_text + " — 자리 잡기", SETTLE_SEC, "pos"))
        for tag, word, _side in hands:
            for j, (sh, say) in enumerate(shapes):
                voice = (word + SEP + say) if j == 0 else say
                out.append((f"{p}_{tag}_{sh}", voice, f"{word} — {say}", HOLD_SEC, "shape"))
        if i < len(POSES) - 1:
            out.append(("rest", "차렷", "차렷 — 쉬기", REST_SEC, "rest"))
    out.append(("rest", "차렷", "차렷 — 끝", REST_SEC, "rest"))
    return out


def voice_map():
    """구간 이름 → 예고 문구 (rest 제외)."""
    return {n: v for n, v, _t, _d, k in steps() if k != "rest"}


def phrases():
    """이 세션이 말하는 모든 문구 (cue_player.VOICE_PHRASES 에 넣는다)."""
    return sorted({part for _n, v, _t, _d, _k in steps() for part in v.split(SEP)})


def phase_info(name: str):
    """구간 이름 → (자세, 손 표기, 실제 쪽 left/right, 모양) — 모양 구간이 아니면 None."""
    for p, _s, _t, hands, shapes in POSES:
        for tag, _w, side in hands:
            for sh, _say in shapes:
                if name == f"{p}_{tag}_{sh}":
                    return p, tag, side, sh
    return None


if __name__ == "__main__":
    t = 0.0
    for n, v, text, d, k in steps():
        print(f"{n:14s} {k:5s} {d:4.1f}s  음성: {' / '.join(v.split(SEP))}")
    print(f"구간 {len(steps())}개, 문구 {len(phrases())}개")
