#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""큐 CSV + 워치독 `--log-csv` → **구간별** 분석.

REPORT_09 §3.2 가 못 만든 표를 이 도구가 만든다.

시각 정렬
---------
워치독 CSV 의 `t` 와 큐 CSV 의 `t_*_mono` 는 **둘 다 `time.monotonic()`** 이고,
리눅스에서 그건 시스템 공용 시계다 → **같은 PC 에서 돌렸다면 오프셋 0 으로 그대로 맞는다.**
(`session_cue.py` docstring 참조.) `--offset` 은 예비 수단이다.

사용법
------
    python3 split_by_cue.py --cue cue_xxx.csv --log sup_on.csv
    python3 split_by_cue.py --cue cue_xxx.csv --log sup_off.csv --log sup_on.csv   # 나란히 비교
    python3 split_by_cue.py --cue cue_xxx.csv --log sup_on.csv --offset -0.4
    python3 split_by_cue.py --self-test                                            # 회귀 시험

구조 (2026-09-18 재작성)
-----------------------
이 파일은 PLAN10~PLAN12 에 걸쳐 문자열 치환으로 덧대다가 변수 섀도잉 버그가 났다
(`other` 가 리스트였다가 float 로 덮어써져 `other[0]` 에서 TypeError).
그래서 `report()` 를 아래 규칙으로 다시 썼다:

1. **`analyze_phase()` 는 항상 같은 키를 가진 dict 를 돌려준다.** `empty=True` 여도
   키 구성은 같다 (값만 NaN/None). 호출부에서 키 유무를 확인할 필요가 없다.
2. **구간·팔마다 쓰는 변수는 그 스코프에서 새로 만든다.** `other_side`, `other_result` 는
   매 호출 시작에서 `None` 으로 초기화한다. `'x' in dir()` 같은 존재 확인은 쓰지 않는다.
3. **쉬는 팔 검사는 한 곳에서만** 한다 (`_check_idle_arm`). 화면 출력과 경고 목록을
   같은 결과에서 만든다 — 예전에는 두 군데에 중복 구현되어 있었다.
4. 이름이 겹칠 수 있는 지역 변수는 길게 쓴다 (`leak_span`, `basis_spans`).
"""

from __future__ import annotations

import argparse
import csv
import sys
import traceback
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# 구간 라벨 -> 그 구간에서 움직여야 하는 축 (워치독 CSV 의 논리 축 이름)
# 라벨에는 팔 접두어가 붙는다: "L_shoulder_pitch" / "R_diagonal_big"
PHASE_TARGET: Dict[str, Optional[str]] = {
    "shoulder_pitch": "shoulder_pitch",
    "shoulder_roll":  "shoulder_roll",
    "shoulder_yaw":   "shoulder_yaw",
    "diagonal_big":   None,      # 두 축을 일부러 같이 쓴다 — 우세축 판정 대상 아님
}

AXES = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw")
SKIP_PHASES = ("settle", "switch_arm")       # 분석하지 않는 구간
REST_NEUTRAL_RAD = 0.35      # 정지 구간에서 |SP|+|SR|+|SY| 가 이보다 작아야 "중립 복귀"
IDLE_ARM_SPAN_RAD = 0.5      # 쉬는 팔의 span 합이 이보다 크면 "움직였다"
TAPER_START, TAPER_END = 1.75, 2.60
MIN_SAMPLES = 5


def split_phase(phase: str) -> Tuple[Optional[str], str]:
    """'L_shoulder_pitch' -> ('left', 'shoulder_pitch'). 접두어가 없으면 팔은 None."""
    if phase.startswith("L_"):
        return "left", phase[2:]
    if phase.startswith("R_"):
        return "right", phase[2:]
    return None, phase


def robust_span(x: np.ndarray) -> float:
    if x.size < 4:
        return float("nan")
    return float(np.percentile(x, 97.5) - np.percentile(x, 2.5))


def verdict(ratio: float) -> str:
    if not np.isfinite(ratio):
        return "판정 불가"
    return "양호" if ratio < 0.3 else ("주의" if ratio < 0.5 else "누설")


# --------------------------------------------------------------------------- 입력

def load_cue(path: Path) -> List[dict]:
    rows = []
    with open(path, encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            if not r.get("t_start_mono") or not r.get("t_end_mono"):
                continue
            rows.append({"index": int(r["index"]), "phase": r["phase"],
                         "start": float(r["t_start_mono"]), "end": float(r["t_end_mono"]),
                         "note": r.get("note", "")})
    if not rows:
        raise SystemExit(f"{path}: 쓸 수 있는 구간이 없다")
    return rows


def load_log(path: Path) -> dict:
    with open(path, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit(f"{path}: 비어 있다")
    num = lambda k: np.array([float(r[k]) for r in rows])
    sides = [s for s in ("left", "right") if f"{s}_shoulder_pitch_cmd" in rows[0]]
    if not sides:
        raise SystemExit(f"{path}: 어깨 열을 찾을 수 없다 — 워치독 --log-csv 산출물이 맞는가?")

    d: dict = {
        "t": num("t"),
        "valid": np.array([r["mode"] == "POS" and r["tracking"] == "1" and r["hold"] == "0"
                           for r in rows]),
        "sides": sides,
        "name": path.name,
        # 패치 1(가슴 기준 상대화) 적용본은 같은 프레임을 두 기준으로 모두 남긴다
        "has_basis": f"{sides[0]}_swing_roll_world" in rows[0],
    }
    for side in sides:
        d[side] = {
            "roll_raw": num(f"{side}_swing_roll_raw"),
            "yaw_raw": num(f"{side}_swing_yaw_raw"),
            "roll_sup": num(f"{side}_swing_roll_suppressed"),
            "yaw_sup": num(f"{side}_swing_yaw_suppressed"),
            "shoulder_pitch": num(f"{side}_shoulder_pitch_desired"),
            "shoulder_roll": num(f"{side}_shoulder_roll_desired"),
            "shoulder_yaw": num(f"{side}_shoulder_yaw_desired"),
            "elbow_pitch": num(f"{side}_elbow_pitch_desired"),
        }
        if d["has_basis"]:
            for basis in ("world", "chest"):
                d[f"{side}_{basis}"] = {
                    "shoulder_pitch": num(f"{side}_swing_roll_{basis}"),
                    "shoulder_roll": num(f"{side}_swing_yaw_{basis}"),
                    "shoulder_yaw": num(f"{side}_twist_{basis}"),
                }
            d[f"{side}_chest_mode"] = [r[f"{side}_chest_mode"] for r in rows]
    return d


# --------------------------------------------------------------------------- 분석

def _empty_result(side: str, n: int, n_all: int, reason: str, mask: np.ndarray) -> dict:
    """정상 결과와 **같은 키 구성**을 가진 빈 결과. 호출부에서 키 유무를 따질 필요가 없다."""
    nan_axes = {axis: float("nan") for axis in AXES}
    return {
        "side": side, "n": n, "n_all": n_all, "empty": True, "reason": reason,
        "mask": mask,
        "spans": dict(nan_axes), "spans_raw": dict(nan_axes), "basis_spans": None,
        "dominant": None, "dominant_raw": None,
        "span_raw": {"roll": float("nan"), "yaw": float("nan")},
        "span_sup": {"roll": float("nan"), "yaw": float("nan")},
        "span_sum": float("nan"),
        "total_med": float("nan"), "total_p90": float("nan"), "total_max": float("nan"),
        "frac_over_start": float("nan"), "frac_over_end": float("nan"),
        "neutral_mean": float("nan"), "elbow_mean": float("nan"),
        "suppressed_active": False, "chest_modes": [],
    }


def analyze_phase(d: dict, lo: float, hi: float, side: str) -> dict:
    """한 구간·한 팔의 통계. **항상 같은 키를 가진 dict** 를 돌려준다."""
    in_window = (d["t"] >= lo) & (d["t"] < hi)
    n_all = int(in_window.sum())
    mask = in_window & d["valid"]
    n = int(mask.sum())

    if side not in d["sides"]:
        return _empty_result(side, n, n_all, f"로그에 {side}팔 열이 없다", mask)
    if n < MIN_SAMPLES:
        return _empty_result(side, n, n_all, "표본 부족", mask)

    arm = d[side]
    roll_raw, yaw_raw = arm["roll_raw"][mask], arm["yaw_raw"][mask]
    roll_sup, yaw_sup = arm["roll_sup"][mask], arm["yaw_sup"][mask]

    spans = {axis: robust_span(arm[axis][mask]) for axis in AXES}
    # 억제 **전**의 축 크기. 부호/scale 이 ±1.0 이라 크기는 명령과 같은 눈금이다.
    # twist(shoulder_yaw)는 억제 대상이 아니므로 명령값을 그대로 쓴다.
    spans_raw = {"shoulder_pitch": robust_span(roll_raw),
                 "shoulder_roll": robust_span(yaw_raw),
                 "shoulder_yaw": spans["shoulder_yaw"]}

    basis_spans = None
    chest_modes: List[str] = []
    if d["has_basis"]:
        basis_spans = {basis: {axis: robust_span(d[f"{side}_{basis}"][axis][mask])
                               for axis in AXES}
                       for basis in ("world", "chest")}
        modes = d[f"{side}_chest_mode"]
        chest_modes = sorted({modes[i] for i in np.flatnonzero(mask)})

    total = np.abs(roll_raw) + np.abs(yaw_raw)
    neutral = sum(np.abs(arm[axis][mask]) for axis in AXES)

    return {
        "side": side, "n": n, "n_all": n_all, "empty": False, "reason": "",
        "mask": mask,
        "spans": spans, "spans_raw": spans_raw, "basis_spans": basis_spans,
        "dominant": max(spans, key=lambda k: spans[k]),
        "dominant_raw": max(spans_raw, key=lambda k: spans_raw[k]),
        "span_raw": {"roll": robust_span(roll_raw), "yaw": robust_span(yaw_raw)},
        "span_sup": {"roll": robust_span(roll_sup), "yaw": robust_span(yaw_sup)},
        "span_sum": float(sum(spans.values())),
        "total_med": float(np.median(total)),
        "total_p90": float(np.percentile(total, 90)),
        "total_max": float(total.max()),
        "frac_over_start": float(np.mean(total > TAPER_START)),
        "frac_over_end": float(np.mean(total > TAPER_END)),
        "neutral_mean": float(neutral.mean()),
        "elbow_mean": float(arm["elbow_pitch"][mask].mean()),
        "suppressed_active": bool(
            float(np.max(np.abs(roll_raw - roll_sup) + np.abs(yaw_raw - yaw_sup))) > 1e-6),
        "chest_modes": chest_modes,
    }


def off_axis_ratio(spans: Dict[str, float], target: Optional[str]) -> Optional[float]:
    """목표 축 대비 가장 큰 비목표 축의 견고 span 비 (REPORT_06 §D 와 같은 정의)."""
    if target is None or target not in spans:
        return None
    target_span = spans[target]
    if not np.isfinite(target_span):
        return None
    if target_span <= 1e-6:
        return float("inf")
    leak_span = max(v for axis, v in spans.items() if axis != target)
    return leak_span / target_span


# --------------------------------------------------------------------------- 출력

def _window(seg: dict, offset: float, trim_lead: float, is_motion: bool) -> Tuple[float, float]:
    lo, hi = seg["start"] + offset, seg["end"] + offset
    if is_motion and trim_lead > 0.0:
        # 🔴 모든 동작 구간에 **일괄** 적용한다. 첫 구간만 깎으면 비교가 불공정해진다.
        lo = min(lo + trim_lead, hi - 0.5)
    return lo, hi


def _check_idle_arm(d: dict, seg: dict, lo: float, hi: float,
                    active_side: str, arm_labelled: bool) -> Optional[str]:
    """쉬는 팔이 움직였는지 **한 곳에서만** 판정한다. 경고 문자열 또는 None.

    `other_side` / `other_result` 는 호출마다 새로 만든다 — 이전 구간의 값이 남아
    다른 타입으로 쓰이던 버그(2026-09-18)를 다시 만들지 않으려는 것이다.
    """
    other_side: Optional[str] = None
    other_result: Optional[dict] = None

    if not arm_labelled:
        return None                      # 팔 접두어가 없는 구간은 판정 대상이 아니다
    candidates = [s for s in d["sides"] if s != active_side]
    if not candidates:
        return None                      # 한쪽 팔만 기록된 로그
    other_side = candidates[0]
    other_result = analyze_phase(d, lo, hi, other_side)
    if other_result["empty"]:
        return None
    if not np.isfinite(other_result["span_sum"]) or other_result["span_sum"] <= IDLE_ARM_SPAN_RAD:
        return None
    return (f"{d['name']} / 구간 {seg['index']} {seg['phase']}: 쉬는 팔({other_side})이 "
            f"움직였다 (span 합 {other_result['span_sum']:.2f} rad)")


def _report_rest(d: dict, seg: dict, lo: float, hi: float, warnings: List[str]) -> None:
    """정지 구간: 로그에 있는 모든 팔이 중립으로 돌아왔는지 본다."""
    for side in d["sides"]:
        result = analyze_phase(d, lo, hi, side)
        if result["empty"]:
            print(f"  {d['name']} [{side}]: 분석 불가 ({result['reason']}, n={result['n']})")
            continue
        returned = result["neutral_mean"] < REST_NEUTRAL_RAD
        print(f"  {d['name']} [{side}]: |SP|+|SR|+|SY| 평균 {result['neutral_mean']:.2f} rad "
              f"{'✅ 중립 복귀' if returned else '🔴 중립으로 안 돌아왔다'}   (n={result['n']})")
        if not returned:
            warnings.append(f"{d['name']} / 구간 {seg['index']} rest [{side}]: "
                            f"중립 복귀 실패 ({result['neutral_mean']:.2f} rad)")


def _report_motion(d: dict, seg: dict, base: str, arm: Optional[str],
                   lo: float, hi: float, warnings: List[str]) -> None:
    """동작 구간 하나를 출력한다. 이 함수 안의 지역 변수는 전부 여기서 새로 만든다."""
    side = arm or d["sides"][0]
    result = analyze_phase(d, lo, hi, side)
    if result["empty"]:
        print(f"  {d['name']}: 유효 표본 {result['n']}개 (구간 내 전체 {result['n_all']}개) — "
              f"분석 불가 ({result['reason']})")
        warnings.append(f"{d['name']} / {seg['phase']}: 분석 불가 "
                        f"({result['reason']}, n={result['n']})")
        return

    target = PHASE_TARGET.get(base)
    spans = result["spans"]

    print(f"  {d['name']} [{side}]  n={result['n']}  엘보 평균 {result['elbow_mean']:+.2f} rad"
          f"{'  [억제 켜짐]' if result['suppressed_active'] else '  [억제 꺼짐]'}")
    print(f"    견고 span   SP {spans['shoulder_pitch']:.2f}  SR {spans['shoulder_roll']:.2f}  "
          f"SY {spans['shoulder_yaw']:.2f}   (우세: {result['dominant']})")
    print(f"    swing raw   roll {result['span_raw']['roll']:.2f} → suppressed "
          f"{result['span_sup']['roll']:.2f}   "
          f"yaw {result['span_raw']['yaw']:.2f} → {result['span_sup']['yaw']:.2f}")

    # 패치 1 적용본: 같은 프레임의 월드(패치 전) / 가슴(패치 후) 대조
    basis_spans = result["basis_spans"]
    if basis_spans is not None and target is not None:
        parts = []
        for basis, label in (("world", "월드(패치 전)"), ("chest", "가슴(패치 후)")):
            ratio = off_axis_ratio(basis_spans[basis], target)
            ratio_text = "  n/a" if ratio is None else f"{ratio:5.2f}"
            parts.append(f"{label} ratio {ratio_text} (목표 span {basis_spans[basis][target]:.2f})")
        modes = "/".join(result["chest_modes"]) or "?"
        print(f"    몸통 상대화:  {parts[0]}   →   {parts[1]}   [명령 기준: {modes}]")
        world_target = basis_spans["world"][target]
        chest_target = basis_spans["chest"][target]
        if np.isfinite(world_target) and world_target > 1e-6 and np.isfinite(chest_target):
            print(f"                 목표 축 span {world_target:.2f} → {chest_target:.2f} "
                  f"({(chest_target / world_target - 1) * 100:+.0f}%)")

    ratio_cmd = off_axis_ratio(result["spans"], target)
    ratio_raw = off_axis_ratio(result["spans_raw"], target)
    if ratio_cmd is not None:
        if result["suppressed_active"] and ratio_raw is not None:
            # 같은 구간, 같은 프레임의 억제 전/후 — 실행 간 동작 편차와 무관한 비교다
            print(f"    off_axis_ratio  억제 전 {ratio_raw:.2f} ({verdict(ratio_raw)})"
                  f"  →  억제 후(로봇에 가는 명령) {ratio_cmd:.2f} ({verdict(ratio_cmd)})")
            target_loss = 1.0 - result["spans"][target] / max(result["spans_raw"][target], 1e-9)
            print(f"    목표 축({target}) 손실: {target_loss * 100:+.0f}%   ← 억제의 부작용")
        else:
            print(f"    off_axis_ratio = {ratio_cmd:.2f}  ({verdict(ratio_cmd)})")

    print(f"    |roll|+|yaw|  중앙 {result['total_med']:.2f}  p90 {result['total_p90']:.2f}  "
          f"최대 {result['total_max']:.2f} rad   >1.75: {result['frac_over_start']*100:.0f}%  "
          f">2.60: {result['frac_over_end']*100:.0f}%")

    # 🔴 쉬는 팔 검사 — 판정은 한 곳에서만 하고, 출력과 경고를 같은 결과에서 만든다
    idle_warning = _check_idle_arm(d, seg, lo, hi, side, arm is not None)
    if idle_warning is not None:
        print(f"    🔴 {idle_warning.split(': ', 1)[1]}")
        warnings.append(idle_warning)

    # 라벨과 실제 우세 축이 다르면 경고 (REPORT_09 §3.2 재발 방지)
    if target is not None and result["dominant_raw"] != target:
        raw = result["spans_raw"]
        message = (f"{d['name']} / 구간 {seg['index']} {seg['phase']}: 라벨과 우세 축이 다르다 "
                   f"(억제 전 기준: 우세 {result['dominant_raw']} {raw[result['dominant_raw']]:.2f} "
                   f"> 목표 {target} {raw[target]:.2f})")
        print(f"    🔴 {message}")
        warnings.append(message)

    if base == "diagonal_big" and np.isfinite(result["total_max"]) and result["total_max"] < TAPER_END:
        message = (f"{d['name']} / 구간 {seg['index']}: |roll|+|yaw| 최대가"
                   f" {result['total_max']:.2f} rad 로 {TAPER_END} rad 에 못 미쳤다"
                   f" — 테이퍼 끝 구간 미검증")
        print(f"    🔴 {message}")
        warnings.append(message)


def report(cue: List[dict], logs: List[dict], offset: float = 0.0,
           trim_lead: float = 0.0) -> List[str]:
    """구간별 분석을 출력하고 경고 목록을 돌려준다."""
    for d in logs:
        print(f"\n로그 {d['name']}  ({'+'.join(d['sides'])}팔, {len(d['t'])}행, "
              f"유효 {int(d['valid'].sum())}행, t={d['t'][0]:.1f}~{d['t'][-1]:.1f} mono"
              f"{', 패치 1 열 있음' if d['has_basis'] else ''})")
    cue_lo, cue_hi = cue[0]["start"] + offset, cue[-1]["end"] + offset
    print(f"큐 구간 범위: {cue_lo:.1f}~{cue_hi:.1f} mono (offset {offset:+.2f}s 적용)")
    for d in logs:
        cover = float(np.mean((d["t"] >= cue_lo) & (d["t"] <= cue_hi)))
        flag = "" if cover > 0.5 else "   🔴 겹침이 적다 — 시각 기준이나 --offset 을 의심하라"
        print(f"  {d['name']}: 로그 표본의 {cover*100:.0f}% 가 큐 구간 안에 있다{flag}")

    warnings: List[str] = []
    for seg in cue:
        if seg["phase"] in SKIP_PHASES:
            continue
        arm, base = split_phase(seg["phase"])
        is_motion = base != "rest"
        lo, hi = _window(seg, offset, trim_lead, is_motion)

        print("\n" + "=" * 78)
        print(f"[{seg['index']:>2}] {seg['phase']:<16} {hi - lo:4.1f}초"
              + (f" (앞 {trim_lead:.1f}초 제외)" if is_motion and trim_lead > 0 else "")
              + f"   {seg['note']}")
        print("=" * 78)

        for d in logs:
            if is_motion:
                _report_motion(d, seg, base, arm, lo, hi, warnings)
            else:
                _report_rest(d, seg, lo, hi, warnings)

    print("\n" + "=" * 78)
    if warnings:
        print(f"🔴 경고 {len(warnings)}건 — 이 구간들의 결론에는 한계를 명시하라")
        for w in warnings:
            print(f"  - {w}")
    else:
        print("✅ 경고 없음 — 모든 구간에서 라벨과 우세 축이 맞고 정지 구간도 중립으로 돌아왔다")
    print("=" * 78)
    return warnings


# --------------------------------------------------------------------------- 회귀 시험

LOGS_DIR = Path(__file__).resolve().parent.parent / "logs"
ROBOT_DIR = Path.home() / "code_workspace/Berkeley-Humanoid-Lite-main/Berkeley-Humanoid-Lite-Lowlevel"

# (설명, 큐 CSV, 워치독 CSV) — 지금까지 쓰인 조합. 없는 파일은 건너뛴다.
REGRESSION_CASES: Sequence[Tuple[str, Path, Path]] = (
    ("패치 1 적용 후 세션", LOGS_DIR / "cue_patched_20260918_222221.csv", ROBOT_DIR / "patched_cue.csv"),
    ("REPORT_10 억제 꺼짐", LOGS_DIR / "cue_sup_off_20260918_190147.csv", ROBOT_DIR / "sup_off_cue.csv"),
    ("REPORT_10 억제 켜짐", LOGS_DIR / "cue_sup_on_20260918_190540.csv", ROBOT_DIR / "sup_on_cue.csv"),
    ("가짜 큐(양팔 라벨)", LOGS_DIR / "cue_FAKE_biarm.csv", ROBOT_DIR / "sup_on_20260918.csv"),
    ("가짜 큐(패치 1 열)", LOGS_DIR / "cue_FAKE_patched.csv", ROBOT_DIR / "chest_ab2.csv"),
    ("가짜 큐(단일 구간)", LOGS_DIR / "cue_FAKE_selftest.csv", ROBOT_DIR / "sup_on_20260918.csv"),
)


def self_test(trim_leads: Sequence[float] = (0.0, 2.0)) -> int:
    """지금까지 쓰인 로그 조합을 전부 돌려 예외 없이 끝나는지 확인한다.

    출력은 버리고 **예외 발생 여부와 경고 개수만** 본다. 숫자가 맞는지는 사람이
    본 출력으로 확인한다 (회귀 비교는 REPORT_10/13 의 표와 대조).
    """
    import io
    import contextlib

    failures = 0
    ran = 0
    print("=" * 78)
    print("회귀 시험 — 디스크에 있는 모든 조합")
    print("=" * 78)
    for label, cue_path, log_path in REGRESSION_CASES:
        if not cue_path.exists() or not log_path.exists():
            missing = cue_path if not cue_path.exists() else log_path
            print(f"  [건너뜀] {label:<22} 파일 없음: {missing.name}")
            continue
        for trim in trim_leads:
            ran += 1
            buffer = io.StringIO()
            try:
                with contextlib.redirect_stdout(buffer):
                    warnings = report(load_cue(cue_path), [load_log(log_path)], 0.0, trim)
                lines = buffer.getvalue().count("\n")
                print(f"  ✅ {label:<22} trim={trim:.1f}  출력 {lines:4d}줄  경고 {len(warnings):2d}건")
            except Exception:
                failures += 1
                print(f"  🔴 {label:<22} trim={trim:.1f}  **예외 발생**")
                traceback.print_exc()
    # 두 로그를 동시에 주는 경로도 확인한다 (나란히 비교 모드)
    pair = [(c, l) for _, c, l in REGRESSION_CASES[1:3] if c.exists() and l.exists()]
    if len(pair) == 2:
        ran += 1
        buffer = io.StringIO()
        try:
            with contextlib.redirect_stdout(buffer):
                warnings = report(load_cue(pair[0][0]), [load_log(pair[0][1]), load_log(pair[1][1])], 0.0, 0.0)
            print(f"  ✅ {'두 로그 동시':<22} trim=0.0  출력 {buffer.getvalue().count(chr(10)):4d}줄  "
                  f"경고 {len(warnings):2d}건")
        except Exception:
            failures += 1
            print(f"  🔴 {'두 로그 동시':<22} **예외 발생**")
            traceback.print_exc()

    print("-" * 78)
    print(f"{ran}건 실행, 실패 {failures}건 — {'✅ 전부 통과' if failures == 0 else '🔴 실패 있음'}")
    print("=" * 78)
    return 1 if failures else 0


# --------------------------------------------------------------------------- 진입점

def main() -> None:
    p = argparse.ArgumentParser(description="큐 CSV 로 워치독 로그를 구간별로 잘라 분석한다")
    p.add_argument("--cue", type=Path)
    p.add_argument("--log", type=Path, action="append",
                   help="워치독 --log-csv 산출물. 두 번 주면 나란히 비교한다")
    p.add_argument("--offset", type=float, default=0.0,
                   help="로그 시각에 더할 보정(초). 같은 PC 에서 돌렸으면 0 이 맞는다")
    p.add_argument("--trim-lead", type=float, default=0.0, metavar="SECONDS",
                   help="각 동작 구간의 앞 SECONDS 초를 버린다 (전 구간 일괄). "
                        "신호 반응 지연과 첫 구간의 c/p 준비 시간을 제거한다 (기본 %(default)s)")
    p.add_argument("--self-test", action="store_true",
                   help="디스크에 있는 로그 조합을 전부 돌려 예외 없이 끝나는지 확인한다")
    a = p.parse_args()

    if a.self_test:
        sys.exit(self_test())
    if not a.cue or not a.log:
        p.error("--cue 와 --log 가 필요하다 (또는 --self-test)")
    report(load_cue(a.cue), [load_log(x) for x in a.log], a.offset, a.trim_lead)


if __name__ == "__main__":
    main()
