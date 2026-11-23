#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""좌표 입력 실행 도구 (작업 지시 19-u). 캔 시작 · 목적지 좌표를 숫자로 받아 점검하고, 확인을 받은 뒤 real19j 와 같은 정책 · 옵션으로
~/run.sh policy 를 부른다. 기존 명령을 감싸기만 한다 (워치독 · replay_run 은 바꾸지 않는다).

입력 (좌표는 로봇 base cm, 캔 밑면 가운데. x 앞 + (로봇에서 먼 쪽), y 왼쪽 + (로봇 왼쪽)):
  수동        --start 27,13 --goal 33,-12          (또는 자리 이름 L1 · L2 · R1 · R2 · s1~s6 · g1~g6, 없으면 대화식으로 묻는다)
  판 표시     --plate --start 20,17.25 --goal 26,42.25   (로봇 쪽 모서리에서 cm, 판 왼쪽 끝에서 cm. x = 모서리 + 7, y = 30.25 − 왼쪽 끝)
  뎁스        --start-from udp:5030 --goal 33,-12   (UDP JSON 한 줄 {"start":[x,y], "conf":0~1, "t":유닉스초}. tools/CAN_GO_INTERFACE.md)
점검: 막기 = 판 밖 · 도달 범위 밖 (19-a 지도 + 19-b2 VR 지도 + R x ≤ 34) · 주차 자세와 캔 간섭. 경고 = 학습 시연 좌표에서 3 cm 넘게 떨어짐.
실행 전에 최종 명령 · 입력 출처 · 판 위 위치 · 점검 결과를 보여 주고 Enter 로 확인받는다. --print-only 는 보여 주기만 한다.
종료 코드: 0 = 실행 (exec) 또는 print-only 통과, 1 = 사용자가 그만둠, 2 = 점검에서 막힘 · 입력 오류, 3 = 뎁스 값을 못 씀."""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import os
import select
import shlex
import socket
import subprocess
import sys
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
TELEOP = TOOLS.parent
HOME = Path.home()
LOGS = TELEOP / "logs"
LOWLEVEL = HOME / "code_workspace/Berkeley-Humanoid-Lite-main/Berkeley-Humanoid-Lite-Lowlevel"
RUN_SH = HOME / "run.sh"

# real19j 실행표와 같은 옵션 (real_eval_prep.UNIFIED · POLICY_ONLY, ~/cc_analysis/handover19j/post19j.py 의 인계 깊이 규칙). 바꾸지 않는다
POLICY = "~/cc_analysis/learn/runs/anypos_demo19j_x5/checkpoints/last/pretrained_model"
UNIFIED = "--sag-comp --no-grip-warm --grip-open-first --tip-cap-deg 18 --tip-offset-cm L 3 0 0 R 3 0 0 --place-tip-offset-cm R -2 0 0 --carry-lift-cm R 2"
HANDOVER_DEPTH = "--handover-depth-cm 3.3 --handover-depth-max-cm 3"
POLICY_ONLY = "--policy-end-after-release 2 --policy-max-s 40 --policy-obs sag-model --policy-can-basis encoder-nocorr"
# 19-x18 위쪽 잡기 (top) 명령에만: R 을 열 때 R 집게 끝 실측 높이가 의도 높이와 0.5 cm 안이 될 때까지 (최대 1 초) 열기를 늦춘다 (replay_run --open-settle-cm)
PLACE_SETTLE = "--open-settle-cm 0.5 --open-settle-max-s 1.0"
# 19-x19 위쪽 잡기 (top) 명령에만: 집게 끝 오프셋을 어깨 roll 실물 멈춤 각 안에서 나머지 관절로 낸다 (replay_run --tip-roll-stop)
TIP_ROLL_STOP = "--tip-roll-stop"
RUN_STEP = "**순서: 명령 실행 → 첫 자세 도착 → 15 초 안에 캔을 십자선에 놓기.** 캔을 먼저 놓고 시작하지 않습니다 (첫 자세로 올라가는 팔이 캔을 칠 수 있음, 19-x17 판 2 · 3)"   # 19-x18 실행표 맨 위 (판마다)
DEMOS = HOME / "cc_analysis/learn/data/vcan_demo19j/files_ho.txt"      # 19-j2 검사 통과 21 판 (anypos_demo19j_x5 학습에 들어간 시연)
# 작업 지시 19-x 모드 전환 (--mode-auto, 기본 끔): 지금 점검이 L 도달로만 막히면 위쪽 잡기 지도 (config/reach_top_19x.json) 를 보고 top 정책을 쓴다
POLICY_TOP = "~/cc_analysis/learn/runs/anypos_top19x_x5/checkpoints/last/pretrained_model"
DEMOS_TOP = HOME / "cc_analysis/top19x/run/files_demo.txt"           # tools/top19x.sh 가 만드는 학습 시연 목록 (점검 통과)

HAND = {"left": "L", "right": "R"}
X_EDGE, Y_EDGE = 7.0, 30.25          # 판 표시: 로봇 쪽 모서리에서 = x − 7, 왼쪽 끝에서 = 30.25 − y
R_X_MAX = 34.0                       # 19-b2: VR 로 R 이 놓는 자리는 x 34 까지
DEPTH_MAX_AGE_S = 2.0                # 뎁스 값이 이보다 오래되면 쓰지 않는다
DEPTH_FUTURE_S = 0.5                 # 보낸 시각이 이만큼 넘게 미래면 (두 PC 시계 어긋남) 쓰지 않는다
DEPTH_MIN_CONF = 0.5
DEPTH_MOVED_CM = 1.0                 # Enter 뒤 새로 받은 값이 이만큼 넘게 다르면 다시 확인
DEMO_IN_CM, DEMO_EDGE_CM = 3.0, 5.0  # 학습 시연까지 거리: ≤ 3 학습 범위, 3 ~ 5 가장자리, > 5 범위 밖
PARK_BLOCK_CM, PARK_WARN_CM = 0.0, 1.0   # 주차 자세 ↔ 캔 원기둥 거리: < 0 막기 (겹침), < 1 경고
PARK_LOGS = 12                       # 주차 자세를 읽을 최근 실기 로그 수 (모델 주차 자세와 함께 본다)

# 판 표시 점 (CAN_GO_INTERFACE.md 의 보정 점과 같은 목록). table.json 자리 + 19-b2 평가 좌표
NAMED = {"L1": (29.0, 13.0), "L2": (29.0, 21.0), "R1": (29.0, -14.0), "R2": (29.0, -22.0),
         "s1": (27.0, 13.0), "s2": (25.0, 23.0), "s3": (22.0, 22.0), "s4": (28.0, 18.0), "s5": (23.0, 8.0), "s6": (28.0, 15.0),
         "g1": (33.0, -12.0), "g2": (33.0, -21.0), "g3": (28.0, -15.0), "g4": (30.5, -16.5), "g5": (33.0, -15.0), "g6": (28.0, -21.0)}


def disp(x: float, y: float):
    return round(x - X_EDGE, 2), round(Y_EDGE - y, 2)


def from_plate(edge: float, left: float):
    return round(edge + X_EDGE, 3), round(Y_EDGE - left, 3)


def fmt(v: float) -> str:
    return f"{round(float(v), 1):g}"


def parse_xy(text: str, plate: bool):
    """'x,y' (또는 판 표시 '모서리,왼쪽 끝') 또는 자리 이름 → (x, y) 로봇 cm. 틀리면 ValueError."""
    t = text.strip()
    key = {k.lower(): k for k in NAMED}.get(t.lower())
    if key:
        return NAMED[key], key
    parts = t.replace(" ", ",").replace(",,", ",").split(",")
    if len(parts) != 2:
        raise ValueError(f"좌표는 x,y 두 숫자 또는 자리 이름 ({' · '.join(NAMED)}): {text!r}")
    a, b = float(parts[0]), float(parts[1])
    if not (math.isfinite(a) and math.isfinite(b)):
        raise ValueError(f"숫자가 아님: {text!r}")
    return (from_plate(a, b) if plate else (a, b)), None


# --------------------------------------------------------------------------- 뎁스 (UDP)
class DepthRx:
    """UDP JSON 한 줄 받기. 실행 직전 가장 최근 값 1 개만 쓴다 (실행 중 추적 안 함)."""

    def __init__(self, bind: str, port: int):
        self.addr = (bind, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.addr)
        self.sock.setblocking(False)
        self.history = []                     # 받은 것 전부 (json 기록용, 최근 50 개)

    def _drain(self):
        got = []
        while True:
            try:
                data, src = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            got.append((time.time(), src, data))
        return got

    BURST_S = 0.3                     # 첫 값이 온 뒤 이만큼 더 받아 그중 마지막을 쓴다 (잇달아 보낸 값)

    def latest(self, wait_s: float):
        """쌓인 것을 다 읽고 마지막 것. 없으면 wait_s 까지 첫 값을 기다리고, 온 뒤 BURST_S 동안 더 받는다. → 판정 dict 또는 None."""
        got = self._drain()
        if not got and wait_s > 0:
            r, _, _ = select.select([self.sock], [], [], wait_s)
            if r:
                end = time.time() + self.BURST_S
                got = self._drain()
                while time.time() < end:
                    r, _, _ = select.select([self.sock], [], [], max(0.0, end - time.time()))
                    if r:
                        got += self._drain()
        for rt, src, data in got:
            self.history.append(judge_msg(data, rt, src))
        self.history = self.history[-50:]
        return self.history[-1] if got else None

    def close(self):
        self.sock.close()


def judge_msg(data: bytes, recv_t: float, src=None) -> dict:
    """메시지 하나 → {ok, why, start, goal, conf, t, age_s, recv_t, raw}. 범위 점검은 여기서 하지 않는다 (공통 점검)."""
    out = {"recv_t": round(recv_t, 3), "recv_iso": dt.datetime.fromtimestamp(recv_t).isoformat(timespec="milliseconds"),
           "src": f"{src[0]}:{src[1]}" if src else None, "raw": data[:500].decode("utf-8", "replace").strip(), "ok": False, "why": []}
    try:
        m = json.loads(data.decode("utf-8").strip().splitlines()[0])
        st = [float(v) for v in m["start"]]
        conf, t = float(m["conf"]), float(m["t"])
        if len(st) != 2 or not all(math.isfinite(v) for v in st + [conf, t]):
            raise ValueError("start 는 숫자 두 개")
        goal = None
        if m.get("goal") is not None:
            goal = [float(v) for v in m["goal"]]
            if len(goal) != 2 or not all(math.isfinite(v) for v in goal):
                raise ValueError("goal 은 숫자 두 개")
    except (ValueError, KeyError, TypeError, IndexError, UnicodeDecodeError) as e:
        out["why"].append(f"형식 틀림 ({type(e).__name__}: {e})")
        return out
    age = recv_t - t
    out.update(start=st, goal=goal, conf=conf, t=t, age_s=round(age, 3))
    if age > DEPTH_MAX_AGE_S:
        out["why"].append(f"오래된 값 ({age:.1f} 초 전 > {DEPTH_MAX_AGE_S:g} 초)")
    if age < -DEPTH_FUTURE_S:
        out["why"].append(f"보낸 시각이 {-age:.1f} 초 미래 (두 PC 시계 맞추기)")
    if not 0.0 <= conf <= 1.0 or conf < DEPTH_MIN_CONF:
        out["why"].append(f"conf {conf:g} < {DEPTH_MIN_CONF:g}")
    out["ok"] = not out["why"]
    return out


# --------------------------------------------------------------------------- 점검
class Checker:
    """판 · 도달 지도 · 주차 자세 FK · 학습 시연 거리. 로봇 · CAN 은 쓰지 않는다 (계산만)."""

    def __init__(self, demos: Path, park_logs: int = PARK_LOGS):
        import numpy as np
        sys.path[:0] = [str(TOOLS), str(LOWLEVEL)]
        import contextlib, io
        with contextlib.redirect_stdout(io.StringIO()):
            import vr_control_watchdog_biarm as W
            import table_limit as TL
            import virtual_can as VC
        self.np, self.W, self.VC = np, W, VC
        W.PR.set_hand_model("cad")
        self.S = ("left", "right")
        self.tab = TL.TableLimit(TELEOP / "config/table.json", body="real")
        can = self.tab.cfg["objects"]["can"]
        self.r_can, self.h_can = 0.5 * float(can["diameter"]), float(can["height"])
        self.maps = {}
        for name, f in (("19a", "reach_19a.json"), ("vr", "reach_vr_19b2.json")):
            m = json.loads((TELEOP / "config" / f).read_text(encoding="utf-8"))
            self.maps[name] = (np.array(m["xs"], float), np.array(m["ys"], float), {s: np.array(m[s], int) for s in self.S})
        top = TELEOP / "config" / "reach_top_19x.json"           # 19-x (있으면 읽어 둔다, --mode-auto 만 쓴다)
        if top.exists():
            m = json.loads(top.read_text(encoding="utf-8"))
            self.maps["top"] = (np.array(m["xs"], float), np.array(m["ys"], float), {s: np.array(m[s], int) for s in self.S})
            self.maps["top_vr"] = (np.array(m["xs"], float), np.array(m["ys"], float), {s: np.array(m[s + "_vr"], int) for s in self.S})
        self.demos, self.demo_note = self._load_demos(demos)
        self.parks, self.park_note = self._load_parks(park_logs)

    # ---- 지도
    def map_cls(self, name: str, side: str, x: float, y: float) -> int:
        xs, ys, cl = self.maps[name]
        np = self.np
        i, j = int(np.argmin(abs(xs - x))), int(np.argmin(abs(ys - y))) if len(ys) else 0
        if abs(xs[i] - x) > 0.5 + 1e-9 or abs(ys[j] - y) > 0.5 + 1e-9:
            return 0
        return int(cl[side][i, j])

    def on_plate(self, x: float, y: float) -> bool:
        t, r = self.tab, self.r_can
        return t.x0 * 100.0 + r <= x <= t.x1 * 100.0 - r and t.y0 * 100.0 + r <= y <= t.y1 * 100.0 - r

    # ---- 학습 시연
    def _load_demos(self, path: Path):
        np = self.np
        if not path.exists():
            return [], f"시연 목록 없음: {path}"
        out = []
        for ln in path.read_text(encoding="utf-8").split():
            p = Path(ln).expanduser()
            try:
                meta = json.loads(str(np.load(p, allow_pickle=True)["meta"]))
                out.append({"name": p.stem.replace("ho19j_dry_vcan_", ""), "start": tuple(float(v) for v in meta["can_start_xy_cm"]),
                            "goal": tuple(float(v) for v in meta["can_goal_xy_cm"])})
            except (OSError, KeyError, ValueError) as e:
                return [], f"시연 좌표를 못 읽음 ({p.name}: {e})"
        return out, f"{path} ({len(out)} 판)"

    def demo_dist(self, st, go, demos=None):
        demos = self.demos if demos is None else demos
        if not demos:
            return None
        ds = min(demos, key=lambda d: math.dist(d["start"], st))
        dg = min(demos, key=lambda d: math.dist(d["goal"], go))
        dp = min(demos, key=lambda d: max(math.dist(d["start"], st), math.dist(d["goal"], go)))
        return {"start_cm": round(math.dist(ds["start"], st), 2), "start_demo": ds["name"], "start_demo_xy": ds["start"],
                "goal_cm": round(math.dist(dg["goal"], go), 2), "goal_demo": dg["name"], "goal_demo_xy": dg["goal"],
                "pair_cm": round(max(math.dist(dp["start"], st), math.dist(dp["goal"], go)), 2), "pair_demo": dp["name"]}

    # ---- 주차 자세
    def _vc(self, ref):
        W = self.W
        q_of = W.motor_to_urdf_fn(ref, self.S)
        vc = self.VC.VirtualCan(self.tab, "L1", ref, self.S, q_of, goal="R1", servo_sign=W.table_servo_signs(ref, self.S), log=lambda s: None)
        return vc, q_of

    def _load_parks(self, n_logs: int):
        """주차 자세 후보: (1) 모델 주차 자세 (dry-run 과 같은 W.model_park_pose, 최신 robot_ref),
        (2) 최근 실기 로그 n 개의 첫 settle 줄 엔코더 (모터를 켤 때 판 위에 얹힌 실제 주차 자세, 로그 시각의 robot_ref). FK 는 모두 그 로그의 ref 로."""
        W = self.W
        out, notes = [], []
        refp = W.newest_robot_ref(None)
        if refp is None:
            return [], "robot_ref 없음 — 주차 점검 못 함"
        ref = W.PR.RobotRef.load(refp)
        vc, q_of = self._vc(ref)
        mp = W.model_park_pose(ref, self.S, W.make_table_clear_fn(ref, self.S, self.tab))
        out.append({"src": f"모델 ({refp.name})", "vc": vc, "q_of": q_of, "pose": mp})
        refs = sorted(LOGS.glob("robot_ref_*.json"))
        logs = sorted([p for p in LOGS.glob("*.csv") if p.name.startswith(("real", "go_", "policy_"))], key=lambda p: p.stat().st_mtime)
        cache = {}
        for p in reversed(logs):
            if len(out) - 1 >= n_logs:
                break
            row = None
            try:
                with p.open(encoding="utf-8", errors="ignore") as fh:
                    for r in csv.DictReader(fh):
                        if r.get("auto_stage") == "settle":
                            row = r
                            break
            except OSError:
                continue
            if row is None or not row.get("left_elbow_pitch_meas"):
                continue
            t = p.stat().st_mtime - 60.0
            rp = None
            for q in refs:
                if dt.datetime.strptime(q.stem[len("robot_ref_"):], "%Y%m%d_%H%M%S").timestamp() <= t:
                    rp = q
            if rp is None:
                continue
            if rp not in cache:
                cache[rp] = self._vc(W.PR.RobotRef.load(rp))
            vc_, q_ = cache[rp]
            try:
                pose = {s: {j: float(row[f"{s}_{j}_meas"]) for j in W.JOINT_ORDER} for s in self.S}
            except (KeyError, ValueError):
                continue
            out.append({"src": f"실기 {p.stem}", "vc": vc_, "q_of": q_, "pose": pose})
        notes.append(f"모델 1 + 실기 로그 {len(out) - 1} 개 (최근 순)")
        return out, "; ".join(notes)

    def arm_points(self, cand, side):
        """팔 하나의 점 (점 cm, 반지름 cm) 두 묶음.
        rigid = 팔꿈치 구 4 · 아래팔 3 (table_limit 과 같은 값) + 링크 · 몸체 상자 (virtual_can overlap 과 같은 상자),
        jaw = 집게 날 두 상자 (닫힘 간격). 주차 때 집게는 열림 · 닫힘이 정해져 있지 않고 얇아 막기에는 쓰지 않는다 (경고만)."""
        np, VC = self.np, self.VC
        vc, q_of, c = cand["vc"], cand["q_of"], cand["pose"][side]
        el, wr, _ = vc.models[side].points(q_of(side, c))
        el, wr = np.asarray(el) * 100.0, np.asarray(wr) * 100.0
        rigid = [(el, 4.0)] + [(el + t * (wr - el), 3.0) for t in np.linspace(0.0, 1.0, 11)]
        tip, R = vc.hand_of(side, c)
        jaw = []
        for k, (s0, s1, j0, j1, n0, n1) in enumerate(vc._hand_boxes(side, VC.JAW_HALF_CLOSED_CM)):
            g = np.stack(np.meshgrid(np.linspace(s0, s1, max(2, int(math.ceil(s1 - s0)) + 1)), np.linspace(j0, j1, max(2, int(math.ceil(j1 - j0)) + 1)),
                                     np.linspace(n0, n1, max(2, int(math.ceil(n1 - n0)) + 1)), indexing="ij"), -1).reshape(-1, 3)
            P = tip[None, :] - g[:, :1] * R[:, 0][None, :] + g[:, 1:2] * R[:, 1][None, :] + g[:, 2:3] * R[:, 2][None, :]
            (jaw if k < 2 else rigid).extend((p, 0.0) for p in P)     # 앞 두 상자 = 집게 날, 뒤 = 링크 · 몸체
        return {"rigid": rigid, "jaw": jaw}

    def can_clear_cm(self, x: float, y: float, cand, side) -> dict:
        """서 있는 캔 원기둥 (밑면 가운데 x, y, 판 면에서 높이 h) ↔ 팔 점 묶음마다 가장 작은 거리 cm (음수 = 겹침)."""
        return {k: self._clear(x, y, pts) for k, pts in self.arm_points(cand, side).items()}

    def _clear(self, x: float, y: float, pts) -> float:
        z0 = self.tab.top(x / 100.0) * 100.0
        z1 = z0 + self.h_can
        best = math.inf
        for p, r in pts:
            dr = math.hypot(p[0] - x, p[1] - y) - self.r_can
            dz = max(z0 - p[2], p[2] - z1, 0.0)
            if dr <= 0.0:
                d = (dz if dz > 0.0 else dr) - r
            else:
                d = math.hypot(dr, dz) - r
            best = min(best, d)
        return best

    def park_clear(self, x: float, y: float):
        """주차 자세 후보 전부 · 두 팔 → dict: 몸체 쪽 가운데값 (보통 주차 자세, 팔마다 작은 쪽) · 가장 나쁜 값 (출처) · 겹친 후보 수 ·
        집게 날 거리의 가운데값. 실기 주차 자세는 판마다 ±2 cm 쯤 달라 (실기 로그 24 판), 가운데값으로 막고 가장 나쁜 값으로 경고한다."""
        rig = {s: [] for s in self.S}
        jaws = {s: [] for s in self.S}
        worst = (math.inf, None, None)
        for cand in self.parks:
            for s in self.S:
                d = self.can_clear_cm(x, y, cand, s)
                rig[s].append(d["rigid"])
                jaws[s].append(d["jaw"])
                if d["rigid"] < worst[0]:
                    worst = (d["rigid"], cand["src"], s)
        med = lambda v: sorted(v)[len(v) // 2]       # noqa: E731
        m_med, m_side = min((med(v), s) for s, v in rig.items())
        j_med, j_side = min((med(v), s) for s, v in jaws.items())
        return {"median_cm": round(m_med, 2), "median_arm": m_side, "min_cm": round(worst[0], 2), "min_src": worst[1], "min_arm": worst[2],
                "n_overlap": sum(v < 0.0 for v in rig[m_side]), "n_poses": len(self.parks), "jaw_median_cm": round(j_med, 2), "jaw_arm": j_side}


def run_checks(ck: Checker, st, go, mode: str = "bottom", demos=None):
    """→ (blocks, warns, info). 각 항목은 한 줄 문장. info["_kinds"] = 막은 까닭 종류 (main 이 빼서 모드 전환에 쓴다).
    mode = "top" (19-x): 도달 점검을 위쪽 잡기 지도 (L 12 cm · R 5 cm, 로봇 조건 + VR 자세) 로, 시연 거리는 demos 로."""
    blocks, warns, info, kinds = [], [], {}, []
    for what, (x, y) in (("캔 시작", st), ("목적지", go)):
        if not ck.on_plate(x, y):
            t = ck.tab
            blocks.append(f"{what} ({fmt(x)}, {fmt(y)}) 판 밖 — 캔이 판 안에 다 들어오려면 x {t.x0 * 100 + ck.r_can:.2f} ~ {t.x1 * 100 - ck.r_can:.2f}, "
                          f"y {t.y0 * 100 + ck.r_can:.2f} ~ {t.y1 * 100 - ck.r_can:.2f}")
            kinds.append("plate")
    for what, side, (x, y) in (("L 잡기 (캔 시작)", "left", st), ("R 놓기 (목적지)", "right", go)):
        if mode == "top":                                       # 19-x
            a, v = ck.map_cls("top", side, x, y), ck.map_cls("top_vr", side, x, y)
            info[f"reach_{side}"] = {"top": a, "top_vr": v}
            if a == 0:
                blocks.append(f"{what} ({fmt(x)}, {fmt(y)}) 위쪽 잡기 도달 지도 밖 (19-w, {'L 12' if side == 'left' else 'R 5'} cm)")
                kinds.append(f"reach_{HAND[side]}")
            elif v == 0:
                blocks.append(f"{what} ({fmt(x)}, {fmt(y)}) 위쪽 잡기 자세를 사람이 VR 로 내기 어려움 (굽힘 · 팔꿈치 방향)")
                kinds.append(f"reach_{HAND[side]}")
            continue
        a, v = ck.map_cls("19a", side, x, y), ck.map_cls("vr", side, x, y)
        info[f"reach_{side}"] = {"19a": a, "vr": v}
        if a == 0:
            blocks.append(f"{what} ({fmt(x)}, {fmt(y)}) 19-a 도달 지도 밖 (관절 한계 · 판 면 · 잡기 판정)")
            kinds.append(f"reach_{HAND[side]}")
        if v == 0:
            blocks.append(f"{what} ({fmt(x)}, {fmt(y)}) 19-b2 VR 도달 범위 밖")
            kinds.append(f"reach_{HAND[side]}")
        elif v == 1 and a != 0:
            warns.append(f"{what} ({fmt(x)}, {fmt(y)}) VR 도달 범위의 '여유 적음' 칸 (팔을 거의 다 펴거나 손이 많이 돎)")
    if go[0] > R_X_MAX:
        blocks.append(f"R 놓기 x {fmt(go[0])} > {R_X_MAX:g} (19-b2: VR 로 R 이 놓는 자리는 x 34 까지)")
        kinds.append("reach_R")
    if ck.parks:
        for what, (x, y) in (("캔 시작", st), ("목적지 (놓은 뒤 주차 복귀)", go)):
            pk = ck.park_clear(x, y)
            info[f"park_{'start' if what == '캔 시작' else 'goal'}"] = pk
            arm = "L" if pk["median_arm"] == "left" else "R"
            if pk["median_cm"] < PARK_BLOCK_CM:
                kinds.append("park")
                blocks.append(f"{what} ({fmt(x)}, {fmt(y)}) 주차 자세의 {arm} 손 몸체 · 팔과 캔이 겹침 (보통 {pk['median_cm']:.1f} cm, "
                              f"주차 자세 {pk['n_poses']} 개 중 {pk['n_overlap']} 개 겹침) — 캔을 놓을 수 없음")
            elif pk["min_cm"] < PARK_WARN_CM:
                warns.append(f"{what} ({fmt(x)}, {fmt(y)}) 주차 자세에 따라 {'L' if pk['min_arm'] == 'left' else 'R'} 손 몸체 · 팔과 {pk['min_cm']:.1f} cm "
                             f"(보통 {pk['median_cm']:.1f} cm, {pk['n_poses']} 개 중 {pk['n_overlap']} 개 겹침, 가장 가까운 것 {pk['min_src']}) — 캔을 놓을 때 손에 닿는지 본다")
            if pk["jaw_median_cm"] < 0.0 and pk["median_cm"] >= PARK_BLOCK_CM:
                warns.append(f"{what} ({fmt(x)}, {fmt(y)}) 주차 자세의 {'L' if pk['jaw_arm'] == 'left' else 'R'} 집게 날과 겹칠 수 있음 "
                             f"(보통 {pk['jaw_median_cm']:.1f} cm) — 집게 옆에 붙여 놓기")
    else:
        warns.append(f"주차 자세 점검 못 함 ({ck.park_note})")
    dd = ck.demo_dist(st, go, demos)
    info["demo"] = dd
    if dd is None:
        warns.append(f"학습 시연 거리 못 잼 ({ck.demo_note})")
        info["demo_level"] = None
    else:
        m = max(dd["start_cm"], dd["goal_cm"])
        lv = "학습 범위" if m <= DEMO_IN_CM else ("가장자리" if m <= DEMO_EDGE_CM else "범위 밖")
        info["demo_level"] = lv
        if lv == "가장자리":
            warns.append(f"학습 시연에서 {m:.1f} cm (3 ~ 5 cm, 가장자리) — 시작 {dd['start_cm']:g} cm ({dd['start_demo']}), 목적지 {dd['goal_cm']:g} cm ({dd['goal_demo']})")
        elif lv == "범위 밖":
            warns.append(f"학습 시연에서 {m:.1f} cm (5 cm 넘음, 범위 밖, 성공 가능성 낮음) — 시작 {dd['start_cm']:g} cm ({dd['start_demo']}), "
                         f"목적지 {dd['goal_cm']:g} cm ({dd['goal_demo']})")
    info["_kinds"] = kinds
    return blocks, warns, info


# --------------------------------------------------------------------------- 명령
def build_cmd(policy: str, st, go, log_csv: str, extra=(), top: bool = False):
    """real19j 실행표와 같은 꼴 (좌표 · 로그 이름만 다르다). 화면용 (~ 그대로) 문자열과 실행용 argv. top = 위쪽 잡기 (19-x18 PLACE_SETTLE · 19-x19 TIP_ROLL_STOP 을 붙임, 기본 끔)."""
    core = (f"~/run.sh policy {policy} --can-start {fmt(st[0])},{fmt(st[1])} --can-goal {fmt(go[0])},{fmt(go[1])} "
            f"{UNIFIED} {HANDOVER_DEPTH} {POLICY_ONLY}" + (f" {PLACE_SETTLE} {TIP_ROLL_STOP}" if top else ""))
    text = core + f" --log-csv {log_csv}" + ("".join(" " + shlex.quote(e) for e in extra))
    argv = [os.path.expanduser(w) if w.startswith("~/") else w for w in shlex.split(text)]
    return text, argv


def log_name(st, go, now: dt.datetime) -> str:
    return f"go_{now:%Y%m%d_%H%M%S}_s{fmt(st[0])}_{fmt(st[1])}_g{fmt(go[0])}_{fmt(go[1])}"


def robot_busy():
    """이미 도는 로봇 프로그램 (python 이름의 프로세스 중 replay_run · 워치독 · 정책 서버). pgrep -f 는 쓰지 않는다 (자기 셸이 걸린다)."""
    try:
        txt = subprocess.run(["ps", "-eo", "pid=,comm=,args="], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    out = []
    for ln in txt.splitlines():
        parts = ln.split(None, 2)
        if len(parts) < 3 or not parts[1].startswith("python") or int(parts[0]) == os.getpid():
            continue
        if any(k in parts[2] for k in ("replay_run.py", "vr_control_watchdog_biarm.py", "policy_server.py")):
            out.append(f"{parts[0]} {parts[2][:120]}")
    return out


def show(src: str, st, go, depth, blocks, warns, info, cmd_text, plate_in):
    p = print
    p("=" * 78)
    p(f"[can_go] 입력 출처: {src}")
    if depth is not None:
        p(f"         뎁스 값: start ({depth['start'][0]:g}, {depth['start'][1]:g}) conf {depth['conf']:g} · 보낸 시각 {depth['t']:.3f} · "
          f"받은 시각 {depth['recv_iso']} · {depth['age_s']:.2f} 초 전 · 보낸 곳 {depth['src']}")
    for what, (x, y) in (("캔", st), ("목적지", go)):
        e, l_ = disp(x, y)
        p(f"  {what:<4} 로봇 ({fmt(x)}, {fmt(y)}) cm  →  판 표시: 로봇 쪽 모서리에서 {e:g} cm · 판 왼쪽 끝에서 {l_:g} cm"
          + ("   (판 표시로 입력)" if plate_in else ""))
    dd = info.get("demo")
    if dd:
        p(f"  학습 시연까지: 시작 {dd['start_cm']:g} cm ({dd['start_demo']} {tuple(dd['start_demo_xy'])}) · 목적지 {dd['goal_cm']:g} cm "
          f"({dd['goal_demo']} {tuple(dd['goal_demo_xy'])}) → {info.get('demo_level')}")
    ps, pg = info.get("park_start"), info.get("park_goal")
    if ps:
        p(f"  주차 자세 ↔ 캔 (손 몸체 · 팔, 주차 자세 {ps['n_poses']} 개의 보통 / 가장 가까울 때): 시작 자리 {ps['median_cm']:+.1f} / {ps['min_cm']:+.1f} cm "
          f"({'L' if ps['median_arm'] == 'left' else 'R'} 팔) · 목적지 {pg['median_cm']:+.1f} / {pg['min_cm']:+.1f} cm ({'L' if pg['median_arm'] == 'left' else 'R'} 팔)")
    rl, rr = info.get("reach_left", {}), info.get("reach_right", {})
    nm = {2: "됨", 1: "여유 적음", 0: "안 됨"}
    if "top" in rl:                                            # 19-x 위쪽 잡기 모드
        vn = {1: "VR 자세 됨", 0: "VR 자세 안 됨"}
        p(f"  도달 지도 (위쪽 잡기 19-w): L 12 cm {nm.get(rl.get('top'), '?')} · {vn.get(rl.get('top_vr'), '?')} | "
          f"R 5 cm {nm.get(rr.get('top'), '?')} · {vn.get(rr.get('top_vr'), '?')} · x ≤ {R_X_MAX:g} {'예' if go[0] <= R_X_MAX else '아니오'}")
        rl, rr = {}, {}
    if rl or rr:
        p(f"  도달 지도: L 잡기 19-a {nm.get(rl.get('19a'), '?')} · VR {nm.get(rl.get('vr'), '?')} | "
          f"R 놓기 19-a {nm.get(rr.get('19a'), '?')} · VR {nm.get(rr.get('vr'), '?')} · x ≤ {R_X_MAX:g} {'예' if go[0] <= R_X_MAX else '아니오'}")
    p("  점검:")
    for b in blocks:
        p(f"    🔴 막음  {b}")
    for w in warns:
        p(f"    🟡 경고  {w}")
    if not blocks and not warns:
        p("    🟢 모두 통과")
    p("  최종 명령:")
    p(f"    {cmd_text}")
    p("=" * 78)


def ask(prompt: str) -> str:
    try:
        return input(prompt)
    except EOFError:
        return "q"


def get_depth(rx: DepthRx, wait_s: float, interactive: bool):
    """→ (판정 dict, None) 쓸 수 있는 값 / (None, '수동') 수동으로 / (None, None) 그만."""
    while True:
        d = rx.latest(wait_s)
        if d is None:
            print(f"[can_go] 뎁스: {wait_s:g} 초 동안 받은 값 없음 (UDP {rx.addr[0]}:{rx.addr[1]})")
        elif not d["ok"]:
            print(f"[can_go] 뎁스: 받은 값을 쓰지 않음 — {'; '.join(d['why'])}  (원문 {d['raw'][:120]})")
        else:
            return d, None
        if not interactive:
            return None, None
        a = ask("  Enter = 다시 받기 · m = 수동 입력 · q = 그만: ").strip().lower()
        if a == "m":
            return None, "수동"
        if a == "q":
            return None, None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog="\n".join(__doc__.split("\n")[2:]))
    ap.add_argument("--start", help="캔 시작 x,y (로봇 base cm) 또는 자리 이름. --plate 면 판 표시 (모서리,왼쪽 끝)")
    ap.add_argument("--goal", help="목적지 x,y 또는 자리 이름. --plate 면 판 표시")
    ap.add_argument("--plate", action="store_true", help="--start · --goal · 대화식 입력을 판 표시 단위로 (로봇 쪽 모서리에서 cm, 판 왼쪽 끝에서 cm)")
    ap.add_argument("--start-from", help="뎁스 입력: udp:<포트> (예 udp:5030). 받은 값의 start 를 캔 시작으로")
    ap.add_argument("--udp-bind", default="0.0.0.0", help="UDP 받는 주소 (기본 0.0.0.0 = 모든 망. 같은 PC 만이면 127.0.0.1)")
    ap.add_argument("--udp-wait-s", type=float, default=5.0, help="쌓인 값이 없을 때 첫 값을 기다리는 시간 (기본 5 초)")
    ap.add_argument("--goal-from-depth", action="store_true", help="(기본 끔) 뎁스 메시지에 goal 이 있으면 목적지로 쓴다. 끄면 목적지는 수동 입력")
    ap.add_argument("--policy", default=POLICY, help=f"체크포인트 (기본 real19j 와 같은 {POLICY})")
    ap.add_argument("--demos", default=str(DEMOS), help="학습 시연 목록 (거리 경고용, 기본 19-j2 통과 21 판)")
    ap.add_argument("--print-only", action="store_true", help="실행하지 않고 명령 · 입력 출처 · 판 위 위치 · 점검 결과만 출력")
    ap.add_argument("--mode-auto", action="store_true",
                    help="19-x (기본 끔): 지금 점검이 L 잡기 도달로만 막히면 위쪽 잡기 지도 (config/reach_top_19x.json) 를 보고, 되면 위쪽 잡기 정책 (--policy-top) 으로")
    ap.add_argument("--policy-top", default=POLICY_TOP, help=f"위쪽 잡기 정책 (기본 {POLICY_TOP})")
    ap.add_argument("--demos-top", default=str(DEMOS_TOP), help="위쪽 잡기 학습 시연 목록 (거리 경고용)")
    ap.add_argument("extra", nargs="*", help="-- 뒤에 준 것은 명령 끝에 그대로 붙인다 (예: -- --dry-run)")
    a = ap.parse_args()
    interactive = sys.stdin.isatty()

    rx = None
    if a.start_from:
        if not a.start_from.startswith("udp:"):
            print(f"[can_go] --start-from 은 udp:<포트> 꼴: {a.start_from!r}")
            return 2
        try:
            rx = DepthRx(a.udp_bind, int(a.start_from[4:]))
        except (ValueError, OSError) as e:
            print(f"[can_go] UDP 를 열 수 없음 ({a.udp_bind}:{a.start_from[4:]}): {e}")
            return 2
        print(f"[can_go] 뎁스 받는 중: UDP {a.udp_bind}:{rx.addr[1]} (지금부터 받은 값 중 가장 최근 1 개를 쓴다. "
              f"{DEPTH_MAX_AGE_S:g} 초 넘게 오래됐거나 conf < {DEPTH_MIN_CONF:g} 이면 쓰지 않음)", flush=True)
        if a.start:
            print("[can_go] --start-from 이 있어 --start 는 쓰지 않는다 (뎁스 값을 못 쓰면 대화식 수동 입력)")

    def read_manual(what: str, given):
        unit = "판 표시 모서리,왼쪽 끝" if a.plate else "로봇 x,y"
        text = given
        while True:
            if text is None:
                if not interactive:
                    print(f"[can_go] {what} 좌표가 없다 (--{'start' if what == '캔 시작' else 'goal'} 또는 대화식 터미널)")
                    return None
                text = ask(f"  {what} ({unit} cm 또는 자리 이름, q = 그만): ").strip()
                if text.lower() == "q":
                    return None
            try:
                return parse_xy(text, a.plate)
            except ValueError as e:
                print(f"[can_go] {e}")
                if not interactive or given is not None:
                    return None
                text = None

    print("[can_go] 점검 준비 (도달 지도 · 주차 자세 FK · 학습 시연 좌표) …", flush=True)
    ck = Checker(Path(a.demos).expanduser())

    depth, src_note = None, "수동"
    go_named = None
    try:
        while True:
            # 시작
            st_named = None
            if rx is not None and depth is None and src_note != "수동(뎁스 대신)":
                d, nxt = get_depth(rx, a.udp_wait_s, interactive)
                if d is None and nxt is None:
                    if a.print_only:
                        print("[can_go] --print-only: 뎁스 값을 못 써서 여기서 끝 (종료 코드 3)")
                    return 3
                if d is None:
                    src_note = "수동(뎁스 대신)"
                else:
                    depth = d
            if depth is not None:
                st = (float(depth["start"][0]), float(depth["start"][1]))
                src = f"뎁스 (UDP {rx.addr[0]}:{rx.addr[1]})"
            else:
                r_ = read_manual("캔 시작", a.start if rx is None else None)
                if r_ is None:
                    return 2 if not interactive else 1
                st, st_named = r_
                src = ("판 표시 입력" if a.plate else "수동 입력") + (" (뎁스 대신)" if rx is not None else "")
            # 목적지
            if a.goal_from_depth and depth is not None and depth.get("goal"):
                go = (float(depth["goal"][0]), float(depth["goal"][1]))
                src += " · 목적지도 뎁스"
            else:
                r_ = read_manual("목적지", a.goal)
                if r_ is None:
                    return 2 if not interactive else 1
                go, go_named = r_
            blocks, warns, info = run_checks(ck, st, go)
            kinds = info.pop("_kinds", [])
            pol = a.policy.replace(str(HOME), "~")
            mode = None
            if a.mode_auto:                                   # 19-x (켤 때만): 모드 고르기
                mode = "bottom"
                if blocks and kinds and all(k == "reach_L" for k in kinds):
                    if "top" not in ck.maps:
                        warns.append("위쪽 잡기 지도 (config/reach_top_19x.json) 가 없어 모드를 바꾸지 못함")
                    else:
                        demos_top, note_top = ck._load_demos(Path(a.demos_top).expanduser())
                        tb, tw, ti = run_checks(ck, st, go, mode="top", demos=demos_top)
                        ti.pop("_kinds", None)
                        if not tb:
                            tw = [w for w in tw if not (not demos_top and w.startswith("학습 시연 거리 못 잼"))]
                            mode, blocks, warns, info = "top", tb, tw + ([] if demos_top else [f"위쪽 잡기 시연 목록 없음 ({note_top})"]), ti
                            pol = a.policy_top.replace(str(HOME), "~")
                        else:
                            warns.append("위쪽 잡기로도 안 됨: " + " / ".join(tb))
            if not Path(os.path.expanduser(pol)).exists():
                blocks.append(f"체크포인트 없음: {pol}")
            busy = robot_busy()
            if busy:
                (warns if a.print_only else blocks).append("로봇 프로그램이 이미 돌고 있음: " + " | ".join(busy))
            now = dt.datetime.now()
            name = log_name(st, go, now)
            log_csv = f"~/teleop-docs/_teleop/logs/{name}.csv"
            cmd_text, argv = build_cmd(pol, st, go, log_csv, a.extra, top=(mode == "top"))
            names = " ".join(f"({k} = {v})" for k, v in (("캔", st_named), ("목적지", go_named)) if v)
            if mode is not None:                              # 19-x
                src = src + f" · 잡기 모드 {mode} ({'위쪽 잡기 — L 이 캔 위쪽, R 이 아래쪽' if mode == 'top' else '지금 방식 — L 아래쪽, R 위쪽'})"
            show(src + (f" {names}" if names else ""), st, go, depth, blocks, warns, info, cmd_text, a.plate)
            record = {"tool": "can_go.py (19-u)", "made": now.isoformat(timespec="seconds"), "source": src, "start_xy_cm": list(st), "goal_xy_cm": list(go),
                      "start_plate_cm": list(disp(*st)), "goal_plate_cm": list(disp(*go)), "named": {"start": st_named, "goal": go_named},
                      "depth_used": depth, "depth_received": rx.history if rx else [], "blocks": blocks, "warns": warns,
                      "checks": {k: v for k, v in info.items()}, "command": cmd_text, "log_csv": log_csv}
            if a.print_only:
                print("[can_go] --print-only: 실행하지 않음. 실행하면 남길 기록 (로그 옆 json):", f"logs/{name}_input.json")
                return 2 if blocks else 0
            if blocks:
                print("[can_go] 🔴 막힌 점검이 있어 실행하지 않는다.")
                if not interactive:
                    return 2
                if ask("  Enter = 좌표 다시 넣기 · q = 그만: ").strip().lower() == "q":
                    return 1
                depth, src_note, a.start, a.goal = None, "수동" if rx is None else src_note, None, None
                continue
            if not interactive:
                print("[can_go] 확인할 터미널이 없어 실행하지 않는다 (--print-only 로 보거나 터미널에서 실행)")
                return 2
            ans = ask(f"  {'경고 ' + str(len(warns)) + ' 개를 봤으면 ' if warns else ''}Enter = 실행 · q = 그만: ").strip().lower()
            if ans == "q":
                return 1
            if ans:
                print("[can_go] Enter 만 실행한다. 다시 묻는다.")
                continue
            if rx is not None and depth is not None:          # Enter 사이에 캔이 움직였나 (새로 받은 값만 본다)
                d2 = rx.latest(0.0)
                if d2 is not None and d2["ok"] and math.dist(d2["start"], depth["start"]) > DEPTH_MOVED_CM:
                    print(f"[can_go] 🟡 확인하는 사이 뎁스 값이 {math.dist(d2['start'], depth['start']):.1f} cm 바뀜 — 새 값으로 다시 확인")
                    depth = d2
                    continue
                record["depth_received"] = rx.history
            LOGS.mkdir(parents=True, exist_ok=True)
            (LOGS / f"{name}_input.json").write_text(json.dumps(record, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
            print(f"[can_go] 기록: logs/{name}_input.json → 실행")
            if rx is not None:
                rx.close()
            sys.stdout.flush()
            os.execv(argv[0], argv)
    except KeyboardInterrupt:
        print("\n[can_go] 그만 (Ctrl+C)")
        return 1


if __name__ == "__main__":
    sys.exit(main())
