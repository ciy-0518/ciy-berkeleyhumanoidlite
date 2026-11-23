#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""녹화 에피소드 재생기 (작업 지시 14-e). VR·UDP 없이 돈다. 사용: replay_run.py 2159_ep02   /   replay_run.py --list

워치독 (실행 위치 vr_control_watchdog_biarm.py) 을 **import** 해서 모터 제어·관절 한계·속도 제한·비틀기 범위·양팔 충돌 방지·
가대 면 제한·발산 감시·그리퍼·VR 표시를 그대로 쓴다. 순서·걸림 감시·VR 감시·소리는 auto_target.py (14-b·14-c 와 같은 것).
옵션은 워치독 parse_args 에 실기동 재생 명령과 같은 인자를 넣어 만든다 → 기본값이 워치독과 어긋나지 않는다.

멈춤: 로봇 PC 에서 **아무 키나** (스페이스·엔터 포함) 또는 Ctrl+C → 그 자리 0.5 초 → 주차 4 초 → 감쇠 → 끝 (종료 코드 0).
      Ctrl+C 를 한 번 더 누르면 바로 감쇠하고 끝. 걸림·발산·(--vr-guard) VR 끊김은 자동 (종료 코드 3).

정책 실행 (작업 지시 15b, --policy, 기본 꺼짐): 같은 자동 순서에서 play 만 정책 (tools/policy_track.py). 정책 서버 (tools/policy_server.py,
      ~/venvs/learn) 는 이 프로그램이 띄우고 끝낼 때 끈다. 사용: replay_run.py --policy act:<…/pretrained_model> [--can-start L1] [--can-goal R1]
      정책 전용 멈춤 (그 자리 → 주차 → 감쇠, 종료 코드 4): 서버 응답 없음 0.2 초, 안전장치가 깎은 양 10° 넘음이 최근 1 초의 80% 넘게, 가상 캔 넘어짐·시간 초과,
      NaN·한계 밖 3 번 연속, 최대 90 초. CSV 끝에 pol_* 열 (관측 시각, 행동 시각, 지연, 묶음 번호·칸, 늦음 수, 멈춤 이유, 가상 캔 상태, 깎은 양).
"""
from __future__ import annotations

import argparse
import csv
import glob
import math
import sys
import threading
import time
from collections.abc import MutableMapping
from pathlib import Path

HOME = Path.home()
LOWLEVEL = HOME / "code_workspace/Berkeley-Humanoid-Lite-main/Berkeley-Humanoid-Lite-Lowlevel"   # 실행 위치 워치독 + recoil
TOOLS = Path(__file__).resolve().parent
LOGS = TOOLS.parent / "logs"
DATA = HOME / "cc_analysis/learn/data"
for p in (str(TOOLS), str(LOWLEVEL)):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402
import vr_control_watchdog_biarm as W  # type: ignore  # noqa: E402
import auto_target as AT  # noqa: E402
import demo_export as DE  # noqa: E402
import demo_replay as DREP  # noqa: E402
import gripper_link as GLINK  # noqa: E402

DEFAULTS = {"table": str(TOOLS.parent / "config/table.json"), "gripper_port": "/dev/ttyUSB0", "cue_sound": "100.85.65.19:5021",
            "robot_viz": "100.85.65.19:5022", "speed": 0.5, "wait_s": 15.0, "repeat": 1}
# 워치독 원격 조종과 같은 안전 옵션 + 재생 (14-b·14-c 실기동 명령과 같다)
SAFETY_ARGS = ["--arm-gap-cm", "-2", "--yaw-in-limit-deg", "100", "100", "--mirror-limits", "on", "--table-body", "real"]

# --------------------------------------------------------------------------- 에피소드 찾기·목록
def all_episodes():
    return sorted(glob.glob(str(DATA / "**" / "*.npz"), recursive=True))

def find_episode(name: str) -> Path:
    p = Path(name).expanduser()
    if p.is_file():
        return p
    stem = p.name[:-4] if p.name.endswith(".npz") else p.name
    hits = [f for f in all_episodes() if Path(f).stem == stem or Path(f).stem.endswith("_" + stem) or Path(f).stem.endswith(stem)]
    if not hits:
        raise SystemExit(f"[STOP] 에피소드 '{name}' 를 못 찾음 — replay_run.py --list 로 이름을 본다 ({DATA})")
    if len(hits) > 1:
        raise SystemExit(f"[STOP] '{name}' 가 여러 개: " + ", ".join(Path(h).stem for h in hits) + " — 더 길게 쓴다")
    return Path(hits[0])

def short_name(path) -> str:
    s = Path(path).stem
    return s[len("dry_vcan_"):] if s.startswith("dry_vcan_") else s

def slot_text(table_json, name: str) -> str:
    for s in table_json.get("can_slots", []):
        if s.get("name") == name:
            return (f"{name} — 판의 로봇 쪽 가장자리에서 {s['from_robot_edge_cm']:g} cm, 판 왼쪽 끝에서 {s['from_left_end_cm']:g} cm "
                    f"(로봇 기준 x {s['base_xy_cm'][0]:g}, y {s['base_xy_cm'][1]:g} cm)")
    return f"{name} (table.json 에 없음)"

def list_episodes() -> None:
    rows = []
    for f in all_episodes():
        try:
            m = DE.load_episode(f)["meta"]
        except Exception as exc:
            rows.append((short_name(f), "읽기 실패", str(exc)[:30], "", "", ""))
            continue
        ok = "성공" if m.get("success") else m.get("result", "")
        rows.append((short_name(f), ok, f"{m.get('duration_s', 0):.1f}", m.get("start_slot", ""), m.get("goal_slot", "") or "-",
                     "인계" if m.get("handover") else ""))
    print(f"에피소드 {len(rows)} 개 ({DATA})")
    print(f"{'이름':14s} {'결과':14s} {'길이 s':>6s}  시작 → 목표  인계")
    for r in rows:
        print(f"{r[0]:14s} {r[1]:14s} {r[2]:>6s}  {r[3]:>4s} → {r[4]:4s}  {r[5]}")
    print("재생: replay_run.py <이름>   (예: replay_run.py 2159_ep02)")

# --------------------------------------------------------------------------- 키 (아무 키나 = 멈춤)
def key_worker(flag: dict) -> None:
    try:
        while not flag["quit"]:
            k = W.read_key()                 # 워치독과 같은 raw 한 글자 읽기 (Ctrl+C 도 한 글자로 온다)
            flag["stop"] = "Ctrl+C" if k == "\x03" else "키 " + (repr(k) if k.strip() else "(스페이스·엔터)")
    except (EOFError, OSError, Exception):
        flag["no_keys"] = True

# --------------------------------------------------------------------------- 14-h: 그리퍼 워밍업·끝에서 놓기·들어 올리기
LIFT_CM = 5.0              # 놓은 뒤 집게 끝을 판에서 수직으로 이만큼 (팔마다, 관절 한계 안에서 수직으로 갈 수 있는 만큼만 — 아래 LIFT_STEPS)
LIFT_STEPS = (5.0, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0)
LIFT_TOL_CM = 0.3          # 끝 위치 오차가 이 안이어야 "수직으로 갔다" (넘으면 한 단계 낮게 — 옆으로 밀면 캔을 칠 수 있다)
LIFT_JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch")    # 손목 돌림은 끝 위치에 영향 없음
SERVO_STEP_S = GLINK.ServoModel.STEP_S      # 보드 STEP_MS 10 ms / 1°
SERVO_SETTLE_S = GLINK.ServoModel.SETTLE_S  # 14-i 보드 SETTLE_MS: 한 서보가 닿은 뒤 다음까지
# 14-n: 가상 그리퍼 (보드 없음) 기다림은 닫힘 각도로 계산한다 (90° 면 예전 값: 워밍업 5.4 초, 열기 1.4 초)


def warm_sim_s(open_deg, close_deg) -> float:
    """워밍업 = 한 서보씩 닫고, 한 서보씩 열기 (서보마다 두 번 움직임 + 쉼) + 0.2."""
    return sum(2 * (abs(c - o) * SERVO_STEP_S + SERVO_SETTLE_S) for o, c in zip(open_deg, close_deg)) + 0.2


def open_sim_s(open_deg, close_deg, closed) -> float:
    """닫힌 서보를 하나씩 여는 시간 (+0.1). closed = (A 닫힘?, B 닫힘?)."""
    return sum(abs(c - o) * SERVO_STEP_S + SERVO_SETTLE_S + 0.1 for o, c, k in zip(open_deg, close_deg, closed) if k)
# 작업 지시 14-l --sag-comp (기본 끔): 처짐 보정 (tools/sag_comp.py). 모터가 켜진 단계에서만, 세기 0 ↔ 1 을 이 시간에 걸쳐 바꾼다
SAG_RAMP_S = 1.0
PSAG_RAMP_S = 0.5      # 19-x20 --place-sag-elbow-scale: R 팔꿈치 처짐 보정 비율을 바꾸는 시간 (초)
PT_CLIP_DEG = 10.0          # 15b --policy: 안전장치가 정책 목표를 이만큼 넘게 깎은 채
PT_CLIP_S = 1.0             #   최근 이만큼 동안
PT_CLIP_FRAC = 0.8          #   이 비율 넘게 (10° 근처에서 오르내려 끊기는 것도 잡는다 — 15b 시험: 판 면에 막혀 9.9~11° 를 오감)
SAG_CUT_SHOW_DEG = 0.1      # 14-m: 관절 한계로 덜 한 보정이 이보다 크면 CSV 에 관절 이름
SAG_CUT_PRINT_DEG = 0.5     #   화면에는 이보다 클 때만 (경계에서 깜빡이지 않게)
# 14-w --tip-offset-cm (기본 끔): 집게 끝 오프셋을 거는 단계와 세기 0 ↔ 1 시간. 첫 자세에 닿은 뒤 (grip_warm) 부터 걸고 주차 복귀 (return·vr_park) 에서 푼다
#   (approach 부터 걸면 판 위 주차에서 막 떠날 때 오프셋이 끝을 판 쪽으로 밀어 판 면 제한에 122 번 막혔다 — dry-run 0049_ep13 L 3 -2 0)
TIP_STAGES = ("grip_warm", "place", "play", "grip_wait", "end_hold", "release", "lift", "user_hold", "vr_hold")
TIP_RAMP_S = 1.0
# 18-b --place-tip-offset-cm (기본 끔): 인계가 끝난 뒤 (그 손만 쥠 = 그 손 닫힘 + 다른 손 열기 끝) 부터 이 시간에 걸쳐 집게 끝 오프셋에 더한다.
#   그 손이 연 뒤에도 유지하고, 주차 복귀에서 집게 끝 오프셋 세기 (tip_gain) 와 함께 풀린다
PLACE_TIP_RAMP_S = 0.5
# 18-c --grasp-tip-offset-cm (기본 끔): 시작부터 그 손이 처음 다 닫힐 때 (grip_wait 끝) 까지 집게 끝 오프셋에 더하고, 그 뒤 이 시간에 걸쳐 0 으로 뺀다 (캔을 쥔 채 손이 그만큼 돌아온다)
GRASP_TIP_RAMP_S = 0.5
# 18-d --grasp-tip-release lift (기본 closed = 18-c 그대로): 다 닫힌 뒤에도 유지하다가 그 손 집게 끝이 닫힐 때 높이보다 --grasp-tip-release-lift-cm 올라가면 뺀다.
#   못 오르면 다 닫힌 뒤 이 시간에 뺀다. 다른 손이 닫으려 하면 (인계 시작) 그 닫기 명령을 붙잡고 먼저 다 뺀다
GRASP_TIP_LIFT_MAX_S = 3.0
# 18-e --carry-lift-cm (기본 끔): 인계 뒤 그 손만 쥔 때부터 이 시간에 걸쳐 집게 끝을 base 위로 D cm 든다. 그 손이 열려 하면 열기 명령을 붙잡고 같은 시간에 걸쳐 내린 뒤 연다
CARRY_RAMP_S = 0.5
# 19-i --handover-depth-cm (기본 끔): 인계 목표 깊이 규칙 (tools/handover_depth.py). R 이 닫으려 할 때 L 손에 든 캔 (L 이 다 닫힐 때 캔 시작 자리를 집게 축 가운데로)
# 에 대한 R 깊이 s 를 재서 목표보다 얕으면 R 집게 끝 오프셋 h 에 모자란 만큼 (상한) 더한다. 다 채울 때까지 R 닫기를 붙잡고, R 이 판에 놓고 연 뒤 같은 시간에 뺀다
PAUSE_CENTER_CM = 1.0       # 14-v: 집게 가운데 = 끝에서 손 축 뒤로 (virtual_can.GRIP_CENTER_CM 과 같다)
PARK_RELAX_FROM = 0.5       # 14-m: 주차 복귀 (return·vr_park) 의 이 비율 뒤부터 판 면 제한을 주차 자세의 판 거리까지 풀어 준다
PARK_RELAX_MARGIN_M = 0.002 #   그보다 이만큼 더 (반올림·보정 흔들림)
SAG_STAGES = ("settle", "approach", "grip_warm", "place", "play", "grip_wait", "end_hold", "release", "lift", "return", "user_hold", "vr_hold", "vr_park")
GRIP_HOLD_S = 0.2          # 14-j --grip-hold-s: 보드가 그리퍼 변화를 끝낸 뒤 재생을 이어 가기까지 더 기다림
OLD_FW_S = 1.0             # 이만큼 지나도 상태에 W 칸이 없으면 옛 펌웨어 (w 를 모름) — 워밍업 없이 간다


LIFT_AXIS_W = 10.0         # 손 축 방향 유지 가중 (축 단위벡터 차 1 = 10 cm). 집게가 기울면 뿌리가 옆으로 밀려 캔을 칠 수 있다
LIFT_LIMIT_MARGIN = 0.01   # 관절 한계 안쪽 여유 (rad)
LIFT_ALONG_W = 0.2         # 손 축 (수평 투영) 방향 이동 벌점 — 작게 (그쪽은 캔을 치지 않는다)


def lift_pose(jm, pose, sides, dz_cm: float = LIFT_CM, axis_fn=None, limits=None, iters: int = 40, offset=None, slide_fn=None):
    """두 집게 끝을 dz_cm 위로 (수평은 그대로) 옮긴 모터 자세. 4 관절 (손목 돌림 빼고), 감쇠 최소제곱.
    axis_fn(side, 모터 dict) → 손 축 단위벡터 를 주면 그 방향도 지키고, limits 를 주면 관절 한계 안으로 자르며 푼다.
    돌려주는 것: (자세, {side: 끝 위치 남은 오차 cm}). 끝 위치는 demo_export.JointMap.tips (robot_viz 와 같은 FK)."""
    out = {s: dict(pose[s]) for s in pose}

    def tips(p):
        arr = np.array([[[float(p[s][j]) if s in p else 0.0 for j in DE.JOINTS] for s in DE.SIDES]])
        return jm.tips(arr)[0]

    def resid(p, s, i, target, ax0):
        d = tips(p)[i] - target
        if slide_fn is not None:                 # 14-h: 손 축을 따라 앞뒤는 거의 자유 (열린 집게가 캔 옆을 미끄러질 뿐), 벌어지는 쪽 수평과 높이는 맞춘다
            ah, nh = slide_fn[s]
            r = [d[2], float(d[:2] @ nh), LIFT_ALONG_W * float(d[:2] @ ah)]
        else:
            r = list(d)
        if axis_fn is not None:
            r += list(LIFT_AXIS_W * (axis_fn(s, p[s]) - ax0))
        return np.array(r)
    err_out = {}
    for s in sides:
        i = DE.SIDES.index(s)
        target = tips(out)[i] + (np.asarray(offset[s], float) if offset is not None else np.array([0.0, 0.0, dz_cm]))
        ax0 = axis_fn(s, out[s]) if axis_fn is not None else None
        for _ in range(iters):
            r = resid(out, s, i, target, ax0)
            if slide_fn is None and np.linalg.norm(r[:3]) < 0.01 and (axis_fn is None or np.linalg.norm(r[3:]) < 0.05):
                break
            J = np.zeros((len(r), len(LIFT_JOINTS)))
            for k, j in enumerate(LIFT_JOINTS):
                q = dict(out[s])
                q[j] += 1e-4
                J[:, k] = (resid({**out, s: q}, s, i, target, ax0) - r) / 1e-4
            dq = -np.linalg.solve(J.T @ J + 0.01 * np.eye(len(LIFT_JOINTS)), J.T @ r)
            for k, j in enumerate(LIFT_JOINTS):
                out[s][j] += float(dq[k])
                if limits is not None:
                    lo, hi = limits[s][j]
                    out[s][j] = min(max(out[s][j], lo + LIFT_LIMIT_MARGIN), hi - LIFT_LIMIT_MARGIN)
        if slide_fn is not None:                 # 남은 오차 = 높이 + 벌어지는 쪽 수평 (축 방향은 세지 않는다)
            d = tips(out)[i] - target
            err_out[s] = float(math.hypot(d[2], float(d[:2] @ slide_fn[s][1])))
        else:
            err_out[s] = float(np.linalg.norm(target - tips(out)[i]))
    return out, err_out


class GripHooks:
    """AutoRun 이 부른다. 보드가 있으면 상태 (? 의 W·Q·지금 각도) 로, 없으면 시간으로 끝을 안다."""

    def __init__(self, gripper, jm, sides, open_deg, axis_fn=None, limits=None, log=print, can_check=None, table_fn=None, close_deg=(90, 90)):
        self.g, self.jm, self.sides, self.open_deg, self.log = gripper, jm, sides, tuple(int(round(v)) for v in open_deg), log
        self.close_deg = tuple(int(round(v)) for v in close_deg)
        self.axis_fn, self.limits = axis_fn, limits
        self.can_check = can_check          # virtual_can.VirtualCan (겹침 판정만 쓴다). 있으면 들어 올린 뒤 주차 길이 캔을 치는지 보고 경유 자세를 고른다
        self.table_fn = table_fn
        self.plan_note = ""
        self._plan = None                   # (트랙 id, 주차 자세 id, [자세…]) — prepare() 스레드가 채운다
        self._plan_key = None
        self._plan_th = None
        self.t = None
        self.board = gripper is not None and bool(gripper.link.port)
        self.lift_err = None
        # 14-j 그리퍼 동기 (AutoRun 이 sync 를 보고 재생을 멈춘다)
        self.sync = False
        self.hold_s = GRIP_HOLD_S
        self._sync_cmd = None
        self._sync_t = None
        self._sync_idle_since = None
        self.vmodel = GLINK.ServoModel(*(int(round(v)) for v in open_deg))     # 가상 그리퍼일 때 펌웨어와 같은 시간
        # 14-v --pause-after-close (기본 0 = 꺼짐): 닫힘 동기가 끝난 직후 팔·그리퍼를 그대로 pause_s 초 더 세운다 (grip_wait 를 늘림)
        self.pause_s = 0.0
        self._sync_close = ()               # 이번 동기에서 열림 → 닫힘 된 팔 ("L"·"R")
        self.pause_until = None
        self.pause_hand = None
        self.pausing = False                # AutoRun 이 보고 그리퍼 동기 4 초 걸림 판정을 쉰다
        self.pause_count = 0
        self.on_pause = None                # 멈춤 시작 때 부름 (hand, now)
        self.held = False                   # 18-d: 그리퍼 명령을 붙잡는 중 (재생기가 apply_grip 을 미룬다) — 동기 끝 판정을 쉰다
        self._held_before = None

    # 들어 올림·경유 계산은 수 초 걸린다 (IK + 겹침 판정) → 제어 루프를 막지 않게 재생 시작 때 스레드로 미리
    def prepare(self, track, park, force=False):
        """force (15b 정책): 끝 자세가 실행 중에 정해진 궤적 — 같은 궤적이어도 다시 계산한다."""
        key = (id(track), id(park))
        if self._plan_key == key and (self._plan is not None or self._plan_th is not None) and not force:
            return
        self._plan_key, self._plan = key, None
        end = track.pose_at(track.duration)

        def run():
            t0 = time.monotonic()
            seq = self.compute(end, track, park)
            self.plan_s = time.monotonic() - t0
            self._plan = (key, seq)
        self._plan_th = threading.Thread(target=run, name="lift_plan", daemon=True)
        self._plan_th.start()

    def plan_ready(self) -> bool:
        return self._plan is not None and self._plan[0] == self._plan_key

    def _cmd_angles(self, st):
        o, c = self.open_deg, self.close_deg
        return (c[0] if int(st["L"]) == 1 else o[0], c[1] if int(st["R"]) == 1 else o[1])

    def sync_begin(self, now, before, after):
        """14-j: 재생이 그리퍼 바뀌는 프레임에서 멈췄다. after = 그 프레임 상태 (명령은 같은 루프의 apply_grip 이 보낸다)."""
        self._sync_cmd = self._cmd_angles(after)
        self._sync_t = now
        self._sync_idle_since = None
        self._sync_close = tuple(h for h in ("L", "R") if int(before[h]) == 0 and int(after[h]) == 1)
        self.pause_until, self.pausing = None, False
        if not self.board:
            self.vmodel.command(now, *self._sync_cmd)

    def sync_done(self, now) -> bool:
        """보드가 그 변화를 끝냈고 (두 서보 명령값 = 목표 = 명령, 대기열 0, 움직이는 서보 없음·쉼 끝) hold_s 더 지났다."""
        if self.held:                                                   # 18-d: 명령을 아직 안 보냈다
            self._sync_idle_since = None
            return False
        if self.board:
            lk = self.g.link
            st = lk.status
            sent = lk.sent_log[-1][0] if lk.sent_log else self._sync_t
            idle = (st is not None and st[0] > max(sent, self._sync_t) + 0.15
                    and st[1] == st[2] == self._sync_cmd[0] and st[4] == st[5] == self._sync_cmd[1]
                    and (lk.status_qwd is None or lk.status_qwd[0] == 0)
                    and (lk.status_ms is None or lk.status_ms == (0, 0)))
        else:
            idle = self.vmodel.idle(now)
        if not idle:
            self._sync_idle_since = None
            return False
        self._sync_idle_since = now if self._sync_idle_since is None else self._sync_idle_since
        done = now - self._sync_idle_since >= self.hold_s
        if not done or self.pause_s <= 0.0 or not self._sync_close:
            return done
        if self.pause_until is None:                                    # 14-v: 닫힘 동기가 끝남 → 멈춤 시작
            self.pause_until, self.pausing = now + self.pause_s, True
            self.pause_hand = self._sync_close
            self.pause_count += 1
            if self.on_pause is not None:
                self.on_pause(self.pause_hand, now)
        if now < self.pause_until:
            return False
        self.pause_until, self.pausing, self._sync_close = None, False, ()
        return True

    def hold_begin(self, now):
        """18-d: 방금 시작한 그리퍼 동기 (sync_begin 과 같은 루프) 의 명령을 붙잡는다. 가상 그리퍼는 시간 모델을 바뀌기 전 각도로 되돌려 둔다."""
        self.held = True
        if not self.board:
            self._held_before = (self.vmodel.cur["A"], self.vmodel.cur["B"])
            self.vmodel = GLINK.ServoModel(*self._held_before)

    def hold_end(self, now):
        """18-d: 붙잡기 끝 — 이 루프의 apply_grip 이 명령을 보낸다. 동기 기다림은 지금부터 센다."""
        self.held = False
        self._sync_t = now
        self._sync_idle_since = None
        if not self.board and self._sync_cmd is not None:
            self.vmodel.command(now, *self._sync_cmd)

    def warmup_begin(self, now):
        self.t = now
        if getattr(self, "open_first", False):                 # 17-e --grip-open-first: 워밍업 대신 열기만
            if self.g is not None:
                for h in self.g.dec:
                    self.g.dec[h].state = 0
                if self.board:
                    self.g.link.send_angles(*self.open_deg)
            return
        if self.board:
            self.g.link.send_warmup(self.close_deg)            # 14-n: "w A B" (닫힘 각도까지)

    def warmup_done(self, now):
        if getattr(self, "open_first", False):                 # 17-e: 다 열림 (지금 = 목표 = 열림, 움직임 없음) 이면 끝
            if not self.board:
                return now - self.t >= open_sim_s(self.open_deg, self.close_deg, (True, True))
            lk = self.g.link
            st = lk.status
            fresh = st is not None and lk.sent_log and st[0] > lk.sent_log[-1][0] + 0.15
            idle = lk.status_ms is None or lk.status_ms == (0, 0)
            return bool(fresh) and idle and st[1] == st[2] == self.open_deg[0] and st[4] == st[5] == self.open_deg[1]
        if not self.board:
            return now - self.t >= warm_sim_s(self.open_deg, self.close_deg)
        lk = self.g.link
        st, qwd = lk.status, lk.status_qwd
        if qwd is None:
            if now - self.t >= OLD_FW_S:
                self.log("\n[그리퍼] 보드 상태에 W 칸이 없다 — 14-g 전 펌웨어 (w 모름). 워밍업 없이 간다")
                return True
            return False
        fresh = st is not None and lk.warm_sent_t is not None and st[0] > lk.warm_sent_t + 0.15
        idle = lk.status_ms is None or lk.status_ms == (0, 0)          # 14-i: 움직이는 서보 없음 + 도착 뒤 기다림도 끝
        return fresh and idle and qwd[0] == 0 and qwd[1] == 0 and st[1] == st[2] == self.open_deg[0] and st[4] == st[5] == self.open_deg[1]

    def release_begin(self, now):
        if self.g is None:
            return False
        st = self.g.state()
        if not any(int(v) == 1 for v in st.values()):
            return False
        for h in self.g.dec:
            self.g.dec[h].state = 0
        self.g.changes += 1
        self.g.link.send_angles(*self.open_deg)
        self.t = now
        self.n_open = sum(int(v) == 1 for v in st.values())          # 14-i: 한 서보씩 열린다 (가상 그리퍼 기다림)
        self.open_s = open_sim_s(self.open_deg, self.close_deg, (int(st.get("L", 0)) == 1, int(st.get("R", 0)) == 1))   # 14-n
        return True

    def release_done(self, now):
        if not self.board:
            return now - self.t >= self.open_s
        lk = self.g.link
        st = lk.status
        fresh = st is not None and lk.sent_log and st[0] > lk.sent_log[-1][0] + 0.15
        qok = lk.status_qwd is None or lk.status_qwd[0] == 0
        idle = lk.status_ms is None or lk.status_ms == (0, 0)          # 14-i
        return bool(fresh) and qok and idle and st[1] == st[2] == self.open_deg[0] and st[4] == st[5] == self.open_deg[1]

    def lift_pose(self, pose, track=None, park=None):
        """AutoRun 이 부른다 (plan_ready 뒤): prepare 가 미리 계산한 [들어 올린 자세(, 경유 자세)]. 계산이 없으면 None."""
        if self.plan_ready():
            self.log("\n[자동] 들어 올림 높이 " + ", ".join(f"{'왼' if s == 'left' else '오른'} {self.lift_dz.get(s, 0.0):g} cm" for s in self.sides)
                     + f" · 주차 길 (기록된 캔 자리): {self.plan_note} (계산 {getattr(self, 'plan_s', 0.0):.1f} 초, 재생 중에)")
            return self._plan[1]
        return None

    def compute(self, pose, track=None, park=None):
        """[들어 올린 자세] 또는 [들어 올린 자세, 경유 자세] (주차 길이 기록된 캔을 치면). 화면에 찍지 않는다 (스레드)."""
        lifted = self.lift_only(pose)
        seq = [lifted]
        self.plan_note = "경유 없음"
        if self.can_check is not None and track is not None and park is not None:
            via = self.plan_via(pose, lifted, park, track)
            if via is not None:
                seq.append(via)
        return seq

    # 경유 자세 후보: 관절 하나 또는 둘을 조금씩 (작은 것부터). 오프라인 (11-03 26 에피소드): 성공 에피소드는 모두 어깨 앞뒤 −10° 하나로 풀렸다
    VIA_JOINTS = ("shoulder_pitch", "shoulder_roll", "elbow_pitch", "shoulder_yaw")
    VIA_DEG = (10, -10, 20, -20, 30, -30, 45, -45)

    def _path_bad(self, a, b, side, n):
        vc = self.can_check
        for t in np.linspace(0.0, 1.0, n)[1:]:
            q = {j: a[side][j] + (b[side][j] - a[side][j]) * t for j in DE.JOINTS}
            if vc.overlap(side, vc.hand_of(side, q), False):
                return True
            if self.table_fn is not None and self.table_fn(side, q)["min"] < -1e-3:
                return True
        return False

    def plan_via(self, end, lifted, park, track):
        import itertools
        vc = self.can_check
        vc.pos = np.asarray(track.ep["can_pos"][-1], float)
        x, y, z, w = (float(v) for v in track.ep["can_quat"][-1])
        vc.rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                           [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        via = {s: dict(lifted[s]) for s in lifted}
        notes = []
        for s in self.sides:
            if vc.overlap(s, vc.hand_of(s, end[s]), False):
                notes.append(f"{'왼' if s == 'left' else '오른'} 끝 자세부터 캔과 겹침 — 그대로")
                continue
            if not self._path_bad(lifted, park, s, 41):
                continue
            cands = [((j, d),) for j in self.VIA_JOINTS for d in self.VIA_DEG]
            cands += [((j1, d1), (j2, d2)) for j1, j2 in itertools.combinations(self.VIA_JOINTS, 2) for d1 in self.VIA_DEG[:4] for d2 in self.VIA_DEG[:4]]
            cands.sort(key=lambda c: sum(abs(d) for _, d in c))
            found = None
            for c in cands:
                q = dict(lifted[s])
                for j, d in c:
                    q[j] += math.radians(d)
                if self.limits is not None and not all(self.limits[s][j][0] <= q[j] <= self.limits[s][j][1] for j in DE.JOINTS):
                    continue
                trial = {**via, s: q}
                if not self._path_bad(lifted, trial, s, 21) and not self._path_bad(trial, park, s, 41):
                    found = (c, q)
                    break
            if found is None:
                notes.append(f"{'왼' if s == 'left' else '오른'} 🔴 캔을 피하는 경유 자세를 못 찾음 — 그대로 주차")
                continue
            via[s] = found[1]
            notes.append(f"{'왼' if s == 'left' else '오른'} 경유 " + ", ".join(f"{j} {d:+d}°" for j, d in found[0]))
        self.plan_note = "; ".join(notes) if notes else "경유 없음 (주차 길이 캔과 안 닿음)"
        return via if any(via[s] != lifted[s] for s in self.sides) else None

    def lift_only(self, pose):
        """팔마다 LIFT_STEPS 에서 관절 한계 안에서 수직으로 갈 수 있는 가장 높은 것 (못 가면 그 팔은 그대로)."""
        out = {s: dict(pose[s]) for s in pose}
        self.lift_err, self.lift_dz = {}, {}
        for s in self.sides:
            for dz in LIFT_STEPS:
                cand, err = lift_pose(self.jm, pose, (s,), dz_cm=dz, axis_fn=self.axis_fn, limits=self.limits)
                if err[s] <= LIFT_TOL_CM:
                    out[s] = cand[s]
                    self.lift_err[s], self.lift_dz[s] = err[s], dz
                    break
            else:
                self.lift_dz[s] = 0.0
        return out


class _SagSide(MutableMapping):
    """한 팔의 '논리 명령' = 보낸 명령 + 보정량. 읽기는 논리 값, 쓰기 (hold_at_measured 등) 는 보낸 명령에 그대로 쓰고 그 관절 보정량을 0 으로."""

    def __init__(self, real, off):
        self._r, self._o = real, off

    def __getitem__(self, j):
        return self._r[j] + self._o.get(j, 0.0)

    def __setitem__(self, j, v):
        self._r[j] = v
        self._o[j] = 0.0

    def __delitem__(self, j):
        raise KeyError(j)

    def __iter__(self):
        return iter(self._r)

    def __len__(self):
        return len(self._r)


class SagView:
    """작업 지시 14-l: 재생 순서 (AutoRun)·VR 표시·그리퍼 훅이 보는 모터. command_target 만 논리 명령 (= 실제로 갈 자세 예상), 나머지는 모터 그대로.
    모터 (보낸 명령) 에는 관절 한계·속도·양팔 충돌·판 면·걸림 감시가 그대로 걸린다."""

    def __init__(self, motor, off):
        object.__setattr__(self, "_m", motor)
        object.__setattr__(self, "_off", off)

    def __getattr__(self, k):
        return getattr(self._m, k)

    def __setattr__(self, k, v):
        setattr(self._m, k, v)

    @property
    def command_target(self):
        return {s: _SagSide(self._m.command_target[s], self._off[s]) for s in self._m.sides}


class MeasShift:
    """작업 지시 14-w: 정책 관측용. last_measured 에서 오프셋 몫 (보낼 목표 − 논리 목표) 을 뺀다. 나머지는 그대로 (쓰기는 하지 않는다)."""

    def __init__(self, inner, dq):
        self._inner, self._dq = inner, dq

    def __getattr__(self, k):
        return getattr(self._inner, k)

    @property
    def last_measured(self):
        src = getattr(self._inner, "last_measured", {}) or {}
        return {s: {j: float(v) - self._dq.get(s, {}).get(j, 0.0) for j, v in m.items()} for s, m in src.items()}


def banner(lines) -> None:
    bar = "█" * 78
    print("\n" + bar)
    for line in lines:
        print("  " + line)
    print(bar + "\n", flush=True)

# --------------------------------------------------------------------------- 실행
def parse() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("episode", nargs="?", help="에피소드 이름 (예 2159_ep02) 또는 npz 경로")
    ap.add_argument("--list", action="store_true", help="쓸 수 있는 에피소드 표")
    ap.add_argument("--speed", type=float, default=DEFAULTS["speed"], help="재생 배율 (기본 0.5)")
    ap.add_argument("--repeat", type=int, default=DEFAULTS["repeat"], help="같은 에피소드 몇 번 (기본 1)")
    ap.add_argument("--wait-s", type=float, default=DEFAULTS["wait_s"], help="캔 놓을 시간 초 (기본 15)")
    ap.add_argument("--dry-run", action="store_true", help="모터·CAN·그리퍼 직렬 없이 (그리퍼는 가상)")
    ap.add_argument("--vr-guard", action="store_true", help="14-c VR 감시: VR 이 3 초 들어와야 시작, 끊기면 유지 → 5 초 → 주차 → 감쇠")
    g = ap.add_argument_group("보통 바꾸지 않는 것")
    g.add_argument("--gripper-port", default=None,
                   help="그리퍼 직렬 (기본 /dev/ttyUSB0, --dry-run 이면 가상. dry-run 에서도 주면 그 포트로 그리퍼만 움직인다. 'none' 이면 그리퍼 없음)")
    g.add_argument("--cue-sound", default=DEFAULTS["cue_sound"], help="서버 PC cue_player (기본 100.85.65.19:5021, 'none' 이면 끔)")
    g.add_argument("--robot-viz", default=DEFAULTS["robot_viz"], help="VR 표시 (기본 100.85.65.19:5022, 'none' 이면 끔)")
    g.add_argument("--table", default=DEFAULTS["table"], help="판 설정 (기본 config/table.json)")
    g.add_argument("--log-csv", default=None, help="로그 (기본 logs/replay_<시각>.csv)")
    g.add_argument("--udp-port", type=int, default=W.DEFAULT_UDP_PORT, help="--vr-guard 가 VR 패킷을 받는 포트 (기본 5000)")
    g.add_argument("--no-beep-local", action="store_true", help="로봇 PC 삐 소리 끔")
    g.add_argument("--no-stall-stop", action="store_true", help="걸림 감시 끔 (권하지 않음)")
    g.add_argument("--no-grip-sync", action="store_true",
                   help="14-j 그리퍼 동기 끔 (끄면 그리퍼가 움직이는 동안에도 재생 시간이 가서 팔이 움직인다 — 14-i 까지)")
    g.add_argument("--grip-close-deg", type=float, nargs=2, default=None, metavar=("L", "R"),
                   help="그리퍼 닫힘 각도 (14-n, 기본 config/gripper.json — 지금 60 60)")
    g.add_argument("--sag-comp", action="store_true",
                   help="처짐 보정 (14-l, 기본 끔): 목표 관절각에 처짐 예상값 (config/sag_model.json) 을 반대로 더해 보낸다. 안전장치는 보낸 명령에, 판 면은 실제로 갈 자세에도")
    g.add_argument("--place-sag-elbow-scale", type=float, default=None, metavar="K",
                   help="19-x20 (기본 끔): 인계가 끝나 R 만 쥔 동안 (L 이 연 뒤 ~ R 이 열 때까지, 판 면에 다가가는 동안) R 팔꿈치 처짐 보정 양을 K 배로 (0.5 초에 걸쳐). "
                        "R 이 열면 0.5 초에 걸쳐 되돌린다. --sag-comp 와 같이 쓴다. 19-x17 실기 8 판 오프라인: K = 0 이 여는 순간 의도 대비 높이 차를 가장 줄임 (±0.5 cm 안 5 / 8)")
    g.add_argument("--sag-cap-deg", type=float, default=None, help="보정 상한 (관절당 도, 기본 10, tools/sag_comp.py CAP_DEG)")
    g.add_argument("--sag-model", default=None, help="처짐 모델 파일 (기본 config/sag_model.json)")
    g.add_argument("--sag-gain", type=float, default=1.0, metavar="G",
                   help="처짐 보정 배율 (14-t', 기본 1.0 = 모델 그대로): 모델 처짐 × G 만큼 보정. 관절 한계 자름·--sag-cap-deg 상한은 곱한 뒤 그대로")
    g.add_argument("--sag-gain-arm", type=float, nargs=2, default=None, metavar=("L", "R"),
                   help="팔마다 처짐 보정 배율 (주면 --sag-gain 대신)")
    g.add_argument("--tip-offset-cm", nargs="+", default=None, metavar="값",
                   help="14-w (기본 끔): 집게 끝 오프셋 'L <h> <j> <n> R <h> <j> <n>' cm, 그 순간 논리 목표의 집게 틀 (h 손 축 + 깊게, j 집게 닫히는 축, n 셋째 축). "
                        "위치 4 관절 IK, 관절당 상한 5°. 순서: 목표 → 오프셋 → 처짐 보정 → 관절 한계. VR 표시·정책 관측은 오프셋 전 논리 값")
    g.add_argument("--place-tip-offset-cm", nargs="+", default=None, metavar="값",
                   help="18-b (기본 끔): 인계 뒤 집게 끝 보정 'R <h> <j> <n>' cm (--tip-offset-cm 과 같은 집게 틀·같은 형식). 그 손이 닫혀 있고 다른 손이 "
                        "같이 쥐었다가 열기를 끝낸 뒤 (그 손만 쥠) 부터 0.5 초에 걸쳐 --tip-offset-cm 에 더하고, 그 손이 연 뒤에도 유지 (주차 복귀에서 함께 풀림). "
                        "같은 IK·--tip-cap-deg. 정책 관측 (sag-model·command·encoder) 은 보정 전 값 그대로")
    g.add_argument("--grasp-tip-offset-cm", nargs="+", default=None, metavar="값",
                   help="18-c (기본 끔): 잡는 동안만 집게 끝 보정 'L <h> <j> <n>' cm (--tip-offset-cm 과 같은 집게 틀·같은 형식). 시작부터 그 손이 처음 다 닫힐 때 "
                        "(grip_wait 끝) 까지 --tip-offset-cm 에 더하고, 그 뒤 0.5 초에 걸쳐 0 으로 뺀다. --place-tip-offset-cm 과 같이 쓸 수 있다. "
                        "같은 IK·--tip-cap-deg. 정책 관측은 보정 전 값 그대로")
    g.add_argument("--grasp-tip-release", choices=("closed", "lift"), default="closed",
                   help="18-d (기본 closed = 다 닫히면 바로 뺌): --grasp-tip-offset-cm 을 빼는 때. lift = 다 닫힌 뒤에도 유지하다가 그 손 집게 끝 (엔코더, 없으면 보낸 명령) 이 "
                        "닫힐 때 높이보다 --grasp-tip-release-lift-cm 올라가면 0.5 초에 걸쳐 뺀다. 3 초 안에 못 오르면 그때 뺀다. "
                        "다른 손이 닫으려 하면 그 닫기 명령을 붙잡고 먼저 다 뺀 뒤 닫는다")
    g.add_argument("--grasp-tip-release-lift-cm", type=float, default=3.0, metavar="D", help="18-d: lift 에서 빼기 시작하는 높이 (cm, 기본 3)")
    g.add_argument("--carry-lift-cm", nargs="+", default=None, metavar="값",
                   help="18-e (기본 끔): 옮기는 동안 들어 두기 'R <D>' cm (base 위쪽, 집게 끝 기준). 인계가 끝난 뒤 (그 손만 쥠, --place-tip-offset-cm 과 같은 조건) "
                        "0.5 초에 걸쳐 들고, 그 손이 열려 하면 열기 명령을 붙잡고 0.5 초에 걸쳐 내린 뒤 연다. 같은 IK·--tip-cap-deg. "
                        "정책 관측은 그대로 (가상 캔이 보는 엔코더 자세에서도 든 몫을 뺀다)")
    g.add_argument("--open-settle-cm", type=float, default=None, metavar="D",
                   help="19-x18 (기본 끔): R 을 열 때 R 집게 끝 실측 높이 (엔코더 FK) 가 의도 높이 (논리 목표 + 집게 끝 · 놓기 · 들기 오프셋, 처짐 보정 전) 와 "
                        "D cm 안이 될 때까지 R 열기 명령을 늦춘다 (--open-settle-max-s 까지). 그동안 팔 명령은 그대로 따른다. 들기 (--carry-lift-cm) 가 열기를 붙잡는 동안은 "
                        "그것이 끝난 뒤부터 잰다. 엔코더가 없으면 (dry-run) 늦추지 않는다. 집게 끝 오프셋 옵션 (--tip-offset-cm 등) 과 같이 쓴다")
    g.add_argument("--open-settle-max-s", type=float, default=1.0, metavar="S", help="19-x18: --open-settle-cm 의 최대 늦춤 (초, 기본 1.0)")
    g.add_argument("--handover-depth-cm", type=float, default=None, metavar="S",
                   help="19-i (기본 끔): 인계 목표 깊이 (cm, 19-h 기준 3.3). R 그리퍼가 닫으려 할 때 엔코더 (dry-run 은 명령) L 자세와 L 이 잡을 때 정한 캔 자세 "
                        "(캔 시작 자리를 집게 축 가운데로) 로 R 깊이 s (virtual_can 둘째 손 판정과 같은 s) 를 재서, S 보다 얕으면 R 집게 끝 오프셋 h 에 모자란 만큼 더한다 "
                        "(--handover-depth-max-cm 까지, 이미 깊으면 0). 다 채울 때까지 R 닫기를 붙잡고, 그 뒤 유지 (R 손 틀이라 놓는 자리는 그대로), R 이 연 뒤 0.5 초에 뺀다. "
                        "오프셋 상한 · 관절 한계 · 판 면 · 두 그리퍼 간격에 걸리면 0.25 cm 씩 줄인다. 재생 · 정책 같은 코드")
    g.add_argument("--handover-depth-max-cm", type=float, default=3.0, metavar="D", help="19-i: 채우는 양 상한 (cm, 기본 3)")
    g.add_argument("--handover-depth-gap-cm", type=float, default=None, metavar="G",
                   help="19-i: 두 그리퍼 선분 (손목 → 집게 끝) 최단 거리 하한 (cm, 기본 tools/handover_depth.py GAP_MIN_CM = 5)")
    g.add_argument("--policy-end-after-held", action="store_true",
                   help="19-x26 (기본 끔): --policy-end-after-release 에서 가상 캔이 쥔 상태 (held_L 등) 가 된 뒤의 닫힘 → 놓음만 센다. "
                        "없으면 캔을 쥐기 전 공중에서 닫았다 연 것도 '놓음' 으로 세어 끝난다 (19-x25 실기 판 5)")
    g.add_argument("--grasp-assist-held-on-grasp", action="store_true",
                   help="19-x30 (기본 끔): L 잡기 보정 (--grasp-assist) 이 'L 이 쥠' 으로 보고 꺼지는 것을 캔을 잡는 닫힘 (보정이 닫기를 허락함, 또는 가상 캔 held_*) "
                        "일 때만으로. 없으면 정책이 공중에서 닫았다 연 것으로도 보정이 꺼진다 (19-x28 실기 판 5)")
    g.add_argument("--grasp-tip-release-on-grasp", action="store_true",
                   help="19-x29 (기본 끔): --grasp-tip-offset-cm 을 빼는 '다 닫힘' 을 캔을 잡는 닫힘으로만 센다 — L 잡기 보정 (--grasp-assist) 이 있으면 "
                        "보정이 닫기를 허락한 뒤의 닫힘, 없으면 가상 캔이 쥔 상태 (held_*) 가 된 닫힘. 정책이 공중에서 닫았다 연 것으로는 빼지 않는다 (19-x28 실기 판 5)")
    g.add_argument("--grasp-tip-skip-start-grip", action="store_true",
                   help="19-x25 (기본 끔): --grasp-tip-offset-cm 을 빼는 '그 손이 처음 다 닫힘' 에서 재생 시작 때의 닫힘 (top 시작 관문 자세) 은 치지 않는다 — "
                        "그 손의 처음 닫힘 → 다 열림 한 번을 건너뛰고 그 뒤 닫힘 (캔 잡기) 부터 센다. 없으면 top 정책은 시작 관문에서 바로 뺀다 (19-x24)")
    g.add_argument("--policy-end-skip-start-grip", action="store_true",
                   help="19-x23 (기본 끔): --policy-end-after-release 에서 재생 시작 때 닫혀 있던 그리퍼 (top 시작 관문 자세) 는 쥔 것으로 치지 않는다 — "
                        "처음 닫힘 → 다 열림 한 번을 건너뛰고 그 뒤 닫힘부터 센다. 없으면 top 정책은 캔을 잡기 전 두 손이 열린 2 초 뒤 끝난다 (19-x23 dry-run)")
    g.add_argument("--policy-end-after-release", type=float, default=None, metavar="S",
                   help="18-e (기본 끔): 정책 실행에서 쥐었던 손이 모두 다 열린 뒤 (마지막 grip_wait 끝) S 초가 지나면 정책을 끝내고 "
                        "가상 캔 성공 때와 같은 끝 순서 (끝 자세 유지 → 수직 들기 → 주차) 로 간다")
    g.add_argument("--no-grip-warm", action="store_true",
                   help="14-z (기본 꺼짐 = 워밍업 함): 첫 자세에서 그리퍼 워밍업 (열림 → 닫힘 → 열림, grip_warm) 을 건너뛰고 바로 캔 놓을 시간. "
                        "보드 상태 읽기 (? 0.1 초마다) 는 그대로라 그리퍼 동기에는 영향 없음")
    g.add_argument("--place-lower-cm", type=float, default=0.0, metavar="D",
                   help="14-ab (기본 0 = 꺼짐): 판에 놓는 열기 (그 손만 쥐고 있을 때의 열기 — 인계로 놓는 열기는 뺌) 직전 --place-lower-s 초 동안 "
                        "그 손 집게 끝을 base 아래로 0 → D cm 내리고, 열기가 끝날 때까지 유지, 연 뒤 같은 시간에 되돌린다. 판 면 검사는 그대로")
    g.add_argument("--place-lower-s", type=float, default=1.0, metavar="T", help="14-ab: 내리기·되돌리기 시간 (초, 기본 1.0)")
    g.add_argument("--tip-roll-stop", action="store_true",
                   help="19-x19 (기본 끔): 집게 끝 오프셋 (--tip-offset-cm · --place-tip-offset-cm · --carry-lift-cm 등 같은 IK) 을 낼 때 어깨 roll 을 실물 멈춤 각 "
                        "(워치독 ROLL_STOP_DEG, 모터 각 L ≥ -19.0°, R ≤ +1.4°) 안에 묶고 나머지 관절 (pitch · yaw · 팔꿈치) 로 끝 위치를 맞춘다. 손 방향은 맞추지 않는다")
    g.add_argument("--tip-cap-deg", type=float, default=None, metavar="D",
                   help="14-y: --tip-offset-cm 관절당 변화 상한 (도, 기본 5 = tools/tip_offset.py CAP_DEG)")
    g.add_argument("--pause-after-close", type=float, default=0.0, metavar="S",
                   help="14-v (기본 0 = 꺼짐): 그리퍼가 닫히고 동기가 끝난 직후마다 팔·그리퍼를 그대로 S 초 세운다. 그동안 엔코더 FK 집게 가운데와 L1 축 차이를 띄우고 CSV 에 남긴다 (실측용). 아무 키 = 멈춤은 그대로")
    g.add_argument("--grip-hold-s", type=float, default=GRIP_HOLD_S, help=f"그리퍼가 끝난 뒤 재생을 이어 가기까지 더 기다림 (기본 {GRIP_HOLD_S:g})")
    p = ap.add_argument_group("정책 실행 (15b, 기본 꺼짐)")
    p.add_argument("--policy", default=None, metavar="체크포인트",
                   help="act:<…/pretrained_model 폴더> 또는 mlp:<…/model.pt> (앞말이 없으면 폴더면 act, .pt 면 mlp). 주면 에피소드 대신 정책")
    p.add_argument("--can-start", default="L1", help="정책: 사용자가 캔을 놓는 자리 (가상 캔 시작, 출처 수동). 기본 L1. 19-c1: 좌표로도 준다 — \"x,y\" (cm, 로봇 base. 예 --can-start 25,15)")
    p.add_argument("--can-goal", default="R1", help="정책: 목표 자리. 기본 R1. 19-c1: 좌표로도 준다 (예 --can-goal 33,-12)")
    p.add_argument("--policy-ensemble", action="store_true", help="정책: 행동 묶음 섞기 (ACT temporal ensembling). 기본은 묶음을 차례로")
    p.add_argument("--policy-max-s", type=float, default=90.0, help="정책: 최대 시간 (초, 기본 90)")
    p.add_argument("--policy-obs", choices=("encoder", "command", "sag-model"), default="encoder",
                   help="정책: 관측 관절 출처 (17-b · 17-e). encoder = 엔코더 (기본, 지금 그대로), command = 논리 명령 (dry-run 과 같은 값 — 실기 모터의 2~4° 남는 차이로 맴돌 때), "
                        "sag-model = 처짐 모델 실제(논리 명령) — 학습 --meas-sim 과 같은 식 (command 로 놓기 앞에 멈출 때)")
    p.add_argument("--grip-open-first", action="store_true",
                   help="17-e (기본 끔): --no-grip-warm 일 때도 첫 자세에서 두 그리퍼 열기 명령을 보내고 보드가 다 열림 (지금 = 목표 = 열림) 을 알릴 때까지 기다린 뒤 캔 놓을 시간 · 재생으로 간다 (보드 없으면 시간). 워밍업을 하면 이미 열림으로 끝나므로 영향 없음")
    p.add_argument("--handover-assist", default=None, metavar="JSON",
                   help="19-x21 (기본 끔, 정책일 때만, top): 인계 보정 + L 여는 안전 규칙 (tools/handover_assist.py, 목표 json 예: config/handover_assist_top_19x21.json). "
                        "L 이 닫힐 때 논리 L 자세로 캔을 붙이고 (가정 B), R 이 캔에서 가까워지면 R 을 목표 자세로 옮겨 닫는다. L 열기는 닫힌 R 의 엔코더 깊이 · 비킴이 기준 안일 때만")
    p.add_argument("--grasp-assist", default=None, metavar="JSON",
                   help="19-x22 (기본 끔, 정책일 때만, top): L 잡기 보정 (tools/handover_assist.GraspAssist, 목표 json 예: config/grasp_assist_top_19x22.json). "
                        "캔 자리는 --can-start 입력 좌표 (엔코더 · 뎁스 어림 없음). L 이 아직 안 쥐었고 목표 끝에서 8 cm 안이면 L 을 목표로 옮기고, 목표 안에서만 L 닫기를 따른다")
    p.add_argument("--policy-can-basis", choices=("logical", "encoder", "encoder-nocorr"), default="logical",
                   help="정책: 가상 캔 판정 팔 자세 (17-c). logical = 논리 명령 (기본, 지금 그대로), encoder = 실기 엔코더 (오프셋 · 처짐 보정이 만든 실제 자세, dry-run 은 logical), "
                        "encoder-nocorr (18-e 추가) = 엔코더 − 보정 몫 (--grasp-tip-offset-cm · --place-tip-offset-cm · --carry-lift-cm 이 더한 관절 변화). "
                        "--tip-offset-cm 기본 오프셋 · 처짐 보정 몫은 encoder 와 같이 그대로 → 가상 캔과 정책의 캔 관측이 보정을 넣기 전과 같아진다")
    p.add_argument("--policy-server-args", default="", help="정책 서버에 덧붙일 인자 (시험용: '--delay-ms 80', '--die-after-s 5')")
    return ap.parse_args()

def watchdog_args(a, ep_path: Path, log_csv: str) -> argparse.Namespace:
    """워치독 parse_args 로 옵션을 만든다 (워치독 기본값 그대로 + 실기동 재생 명령과 같은 인자)."""
    o_deg, c_deg = GLINK.grip_deg()                                       # 14-n: config/gripper.json (--grip-close-deg 로 덮음)
    if a.grip_close_deg is not None:
        c_deg = tuple(int(round(v)) for v in a.grip_close_deg)
    argv = ["vr_control_watchdog_biarm.py"] + SAFETY_ARGS + ["--gripper-open-deg", str(o_deg[0]), str(o_deg[1]),
                                                             "--gripper-close-deg", str(c_deg[0]), str(c_deg[1])]
    argv += ["--table", a.table] + (["--replay", str(ep_path)] if ep_path is not None else []) + ["--replay-speed", f"{a.speed:g}",
                                                            "--replay-auto", "--replay-wait-s", f"{a.wait_s:g}", "--replay-repeat", str(a.repeat),
                                                            "--log-csv", log_csv, "--udp-port", str(a.udp_port)]
    port = a.gripper_port if a.gripper_port is not None else (None if a.dry_run else DEFAULTS["gripper_port"])
    if a.dry_run:
        argv += ["--dry-run"]
    if port is None and a.dry_run:
        argv += ["--grip-virtual"]
    elif port is not None and port.lower() != "none":
        argv += ["--gripper-port", port]
    if a.cue_sound.lower() != "none":
        argv += ["--cue-sound", a.cue_sound]
    if a.robot_viz.lower() != "none":
        argv += ["--robot-viz", a.robot_viz]
    if not a.no_stall_stop:
        argv += ["--stall-stop"]
    if a.vr_guard:
        argv += ["--vr-guard"]
    if a.no_beep_local:
        argv += ["--no-beep-local"]
    old = sys.argv
    try:
        sys.argv = argv
        wa = W.parse_args()
    finally:
        sys.argv = old
    wa.argv_equiv = argv[1:]
    return wa

def main() -> int:
    a = parse()
    if a.list:
        list_episodes()
        return 0
    if a.policy:                                                              # 15b: 정책이면 에피소드 없음, 배율·회수 1
        a.speed, a.repeat = 1.0, 1
        if a.episode:
            print("[STOP] --policy 와 에피소드를 같이 줄 수 없다")
            return 2
    elif not a.episode:
        print("사용: replay_run.py <에피소드>   (목록: replay_run.py --list)")
        return 2
    if not (0.05 <= a.speed <= 2.0) or a.repeat < 1 or a.wait_s < 5.0:
        print("[STOP] --speed 0.05~2.0, --repeat 1 이상, --wait-s 5 이상")
        return 2
    ep_path = find_episode(a.episode) if not a.policy else None
    log_csv = a.log_csv or str(LOGS / f"{'policy' if a.policy else 'replay'}_{time.strftime('%Y%m%d_%H%M%S')}{'_dry' if a.dry_run else ''}.csv")
    wa = watchdog_args(a, ep_path, log_csv)

    # ---- 워치독 main() 과 같은 차례로 준비 (같은 함수·클래스)
    W.PR.set_hand_model(wa.hand_model)
    W.apply_yaw_in_limit(*wa.yaw_in_limit_deg)
    sides = W.selected_sides(wa.side)
    ref_path = W.newest_robot_ref(wa.robot_ref)
    if ref_path is None:
        print("[STOP] 로봇 기준값 파일이 없다 (logs/robot_ref_*.json). 먼저: run.sh P")
        return 2
    robot_ref = W.PR.RobotRef.load(ref_path)
    limits = W.limits_from_robot_ref(robot_ref, sides)
    if wa.mirror_limits == "on":
        limits, _ = W.mirror_limits(limits, robot_ref, sides)
    import table_limit as TLIM  # type: ignore
    import json
    try:
        table = TLIM.TableLimit(Path(wa.table).expanduser(), body=wa.table_body, margin_body_cm=wa.table_margin_body_cm,
                                margin_tip_cm=wa.table_margin_tip_cm)
        table_json = json.loads(Path(wa.table).expanduser().read_text(encoding="utf-8"))
    except (OSError, ValueError, KeyError) as exc:
        print(f"[STOP] 판 설정을 못 읽음: {exc}")
        return 2
    pol = None
    if a.policy:                                                              # 15b
        import policy_track as PT
        spec = a.policy if ":" in a.policy.split("/")[0] else (("mlp:" if a.policy.endswith(".pt") else "act:") + a.policy)
        print(f"정책 서버를 띄운다 (~/venvs/learn, 체크포인트 읽기에 수 초): {spec}", flush=True)
        try:
            pol = PT.PolicyTrack(spec, robot_ref, sides, table, can_start=a.can_start, can_goal=a.can_goal, ensemble=a.policy_ensemble,
                                 max_s=a.policy_max_s, limits=limits, open_deg=wa.gripper_open_deg, close_deg=wa.gripper_close_deg,
                                 dry_run=wa.dry_run, viz_target=wa.robot_viz, server_args=a.policy_server_args,
                                 **({"obs_source": a.policy_obs} if a.policy_obs != "encoder" else {}),
                                 **({"can_basis": a.policy_can_basis} if a.policy_can_basis != "logical" else {}),
                                 **({"assist": a.handover_assist} if a.handover_assist else {}),
                                 **({"grasp_assist": a.grasp_assist} if a.grasp_assist else {}))
            if a.grasp_assist_held_on_grasp:                           # 19-x30 (켤 때만)
                pol.ga_held_on_grasp = True
        except (OSError, ValueError, KeyError, RuntimeError) as exc:
            print(f"[STOP] 정책: {exc}")
            return 2
        reps = [pol]
    else:
        try:
            reps = [DREP.DemoReplay(ep_path, robot_ref, sides, speed=wa.replay_speed, move_s=wa.replay_move_s, can_target=wa.robot_viz)]
        except (OSError, ValueError, KeyError) as exc:
            print(f"[STOP] 에피소드: {exc}")
            return 2
    meta = reps[0].meta

    motor = W.DualArmMotorController(channels={"left": wa.left_channel, "right": wa.right_channel}, sides=sides,
                                     motor_ids={"left": wa.left_ids, "right": wa.right_ids}, urdf_limits=limits, kp=wa.kp, kd=wa.kd,
                                     torque_limit=wa.torque_limit, max_speed=wa.max_speed, dry_run=wa.dry_run)
    motor.yaw_dyn = {s: (float(robot_ref.rest[s]["shoulder_pitch"]), float(robot_ref.e90[s]["shoulder_yaw"])) for s in sides}
    if not wa.no_arm_collision and len(sides) == 2:
        motor.arm_gap_fn = W.make_arm_gap_fn(robot_ref, sides)
        motor.arm_gap_min = wa.arm_gap_cm / 100.0
    motor.table_fn = W.make_table_clear_fn(robot_ref, sides, table)
    motor.dry_run_base = W.model_park_pose(robot_ref, sides, motor.table_fn)     # dry-run 만 (실기동은 엔코더 값 = 판 위 주차)
    motor.vel_ff = bool(wa.vel_ff)
    meas_ok = not motor.dry_run or bool(getattr(motor, "sim_meas", False))

    # gripper_link 은 위에서 (GLINK)
    gripper = None
    gkw = dict(open_deg=[int(round(v)) for v in wa.gripper_open_deg], close_deg=[int(round(v)) for v in wa.gripper_close_deg],
               close_cm=wa.grip_close_cm, open_cm=wa.grip_open_cm, open_hold_s=wa.grip_open_hold_s)
    # 14-f D: 보낸 각도·시각 기록, 보드 상태 읽기 (0.1 초), READY (보드 재시작) 이면 다시 보냄, 목표 뒤 1.5 초 동안 0.2 초마다 (도착하면 그만)
    link_kw = dict(record=True, poll_s=0.1, repeat_s=0.2, repeat_for_s=1.5)
    if wa.gripper_port:
        try:
            gripper = GLINK.GripperController(None, **gkw, link=GLINK.GripperLink(wa.gripper_port, **link_kw))
        except Exception as exc:
            print(f"[그리퍼] 🔴 시작 못 함: {exc} — 그리퍼 없이 계속한다")
    elif wa.grip_virtual:
        gripper = GLINK.GripperController(None, **gkw, link=GLINK.GripperLink(None, record=True))
    pos_runner = W.PosRetargetRunner(sides, robot_ref, body_frame=wa.body_frame, limits=limits)    # VR 표시가 읽기만 한다
    robot_viz = None
    if wa.robot_viz:
        try:
            import robot_viz as RVIZ  # type: ignore
            robot_viz = RVIZ.RobotViz(wa.robot_viz, robot_ref, sides, W.motor_to_urdf_fn(robot_ref, sides), hz=wa.robot_viz_hz,
                                      real_size=wa.robot_viz_real_size, jaw_sign=wa.robot_viz_jaw_sign,
                                      jaw_offset_deg=wa.robot_viz_jaw_offset_deg, dry_run=wa.dry_run, table=table)
        except Exception as exc:
            print(f"[robot-viz] 🔴 시작 못 함: {exc} — VR 표시 없이 계속한다")
    cue = W.CueSound(wa.cue_sound, wa.cue_verbose, voice=False) if wa.cue_sound else None
    beeper = AT.Beeper(local=wa.beep_local, cue=cue)
    vr_link = AT.VrLink(None, AT.VR_HEAD_DROP_GUARD_CM, back=AT.VR_HEAD_BACK_GUARD) if wa.vr_guard else None
    receiver = W.BodyTrackingUdpReceiver(wa.udp_ip, wa.udp_port) if wa.vr_guard else None     # --vr-guard 일 때만 UDP 를 연다
    stall = AT.StallGuard(sides) if wa.stall_stop else None
    _models = {s: W.PR.ArmModel(s) for s in sides}
    _q_of = W.motor_to_urdf_fn(robot_ref, sides, _models)
    table_plain = motor.table_fn
    # 작업 지시 14-l --sag-comp (기본 끔)
    sag = None
    sag_off = {s: {j: 0.0 for j in W.JOINT_ORDER} for s in sides}
    sag_gain = 0.0
    sag_mult = {s: 1.0 for s in sides}          # 14-t' 배율 (팔마다)
    psag = None                                 # 19-x20 --place-sag-elbow-scale (기본 끔): {"k", "ho": 0 / 1 두 손 쥠 / 2 R 만 쥠, "gain": 0 ~ 1}
    view = motor
    if a.place_sag_elbow_scale is not None and not a.sag_comp:
        print("[주의] --place-sag-elbow-scale 은 --sag-comp 와 같이 쓴다 — 끔")
    if a.sag_comp:
        import sag_comp as SAG
        try:
            sag = SAG.SagModel.load(_q_of, _models, Path(a.sag_model).expanduser() if a.sag_model else SAG.MODEL_PATH,
                                    cap_deg=SAG.CAP_DEG if a.sag_cap_deg is None else a.sag_cap_deg)
        except (OSError, ValueError, KeyError) as exc:
            print(f"[STOP] --sag-comp: 처짐 모델을 못 읽음: {exc}")
            if pol is not None:
                pol.close()
            return 2
        sag_mult = {s: (a.sag_gain_arm[0 if s == "left" else 1] if a.sag_gain_arm else a.sag_gain) for s in sides}
        if a.place_sag_elbow_scale is not None and "right" in sides:          # 19-x20 (켤 때만)
            psag = {"k": min(max(float(a.place_sag_elbow_scale), 0.0), 1.0), "ho": 0, "gain": 0.0}

        def table_both(side, c, f=table_plain, m=sag):
            """판 면: 보낸 명령과 실제로 갈 자세 (보낸 명령 + 예상 처짐) 둘 다 재서 엄한 쪽."""
            p, q = f(side, c), f(side, m.actual(side, c))
            out = dict(p if p["g"] <= q["g"] else q)
            out["min"] = min(p["min"], q["min"])
            return out
        motor.table_fn = table_both
        view = SagView(motor, sag_off)
    # 작업 지시 14-w --tip-offset-cm (기본 끔)
    tip = None
    tip_dq = {s: {j: 0.0 for j in W.JOINT_ORDER} for s in sides}     # 보낼 목표 − 논리 목표 (오프셋 몫, rad)
    tip_cut = {s: {j: 0.0 for j in W.JOINT_ORDER} for s in sides}
    tip_gain = 0.0
    tip_info = {}
    tip_cap_shown = ()
    tip_cut_shown = ()
    ptip = None                          # 18-b --place-tip-offset-cm (기본 끔): {"off": 팔 → h j n cm, "gain"·"st"·"t0": 팔마다 세기·상태 (0 전 / 1 같이 쥠 / 2 걸림)·조건 시작 시각}
    gtip = None                          # 18-c --grasp-tip-offset-cm (기본 끔): {"off": 팔 → h j n cm, "gain": 세기 (처음 1), "st": 0 닫히기 전 / 1 다 닫힘 (빼는 중·뺌), "t0": 닫힘 시작 시각}
    carry = None                         # 18-e --carry-lift-cm (기본 끔): {"d": 팔 → cm, "gain": 세기, "st": 0 전 / 1 같이 쥠 / 2 들어 둠 / 3 내리는 중·내림, "t0", "gate": 열기 붙잡기 요청, "dq": 든 몫 (rad)}
    grip_hold = {"on": False}            # 18-d·18-e: 그리퍼 명령을 붙잡는 중 (apply_grip 을 미룬다)
    oset = None                          # 19-x18 --open-settle-cm (기본 끔): {"cm", "max_s", "st": 0 전 / 1 늦추는 중 / 2 보냄, "ho": 0 / 1 두 손 쥠 / 2 인계 뒤 R 만 쥠, "t0", "dz", "wait_s", "why", "gate"}
    hdep = None                          # 19-i --handover-depth-cm (기본 끔): {"hd": HandoverDepth, "st": 0 전 / 1 채우는 중 (R 닫기 붙잡음) / 2 채움 유지 / 3 빼는 중·뺌, "fill", "gain", "gate", …}
    if a.grasp_tip_release != "closed" and not a.grasp_tip_offset_cm:
        print("[주의] --grasp-tip-release 는 --grasp-tip-offset-cm 과 같이 쓴다 — 무시")
    if a.tip_roll_stop and not (a.tip_offset_cm or a.place_tip_offset_cm or a.grasp_tip_offset_cm or a.carry_lift_cm or a.handover_depth_cm is not None):
        print("[주의] --tip-roll-stop 은 집게 끝 오프셋 옵션 (--tip-offset-cm 등) 과 같이 쓴다 — 끔")
    if a.open_settle_cm is not None and not (a.tip_offset_cm or a.place_tip_offset_cm or a.grasp_tip_offset_cm or a.carry_lift_cm or a.handover_depth_cm is not None):
        print("[주의] --open-settle-cm 은 집게 끝 오프셋 옵션 (--tip-offset-cm 등) 과 같이 쓴다 — 끔")
    if a.tip_offset_cm or a.place_tip_offset_cm or a.grasp_tip_offset_cm or a.carry_lift_cm or a.handover_depth_cm is not None:
        import tip_offset as TOF
        import virtual_can as VCAN  # type: ignore
        try:
            tip_spec = TOF.parse_spec(a.tip_offset_cm) if a.tip_offset_cm else {}
        except ValueError as exc:
            print(f"[STOP] --tip-offset-cm: {exc}")
            if pol is not None:
                pol.close()
            return 2
        if a.place_tip_offset_cm:
            try:
                _pspec = {s: v for s, v in TOF.parse_spec(a.place_tip_offset_cm).items() if s in sides and len(sides) == 2 and bool(np.any(v != 0.0))}
            except ValueError as exc:
                print(f"[STOP] --place-tip-offset-cm: {exc}")
                if pol is not None:
                    pol.close()
                return 2
            if _pspec:
                ptip = {"off": _pspec, "gain": {s: 0.0 for s in _pspec}, "st": {s: 0 for s in _pspec}, "t0": {s: None for s in _pspec}}
            else:
                print("[주의] --place-tip-offset-cm: 값이 모두 0 이거나 그 팔을 쓰지 않는다 (두 팔일 때만) — 끔")
        if a.carry_lift_cm:
            _ct = [str(x) for x in a.carry_lift_cm]
            try:
                if len(_ct) % 2 or any(_ct[i].upper() not in ("L", "R") for i in range(0, len(_ct), 2)):
                    raise ValueError("형식: R <D> (cm, base 위쪽)")
                _cspec = {("left" if _ct[i].upper() == "L" else "right"): float(_ct[i + 1]) for i in range(0, len(_ct), 2)}
            except ValueError as exc:
                print(f"[STOP] --carry-lift-cm: {exc}")
                if pol is not None:
                    pol.close()
                return 2
            _cspec = {s: d for s, d in _cspec.items() if s in sides and len(sides) == 2 and d != 0.0}
            if _cspec:
                carry = {"d": _cspec, "gain": {s: 0.0 for s in _cspec}, "st": {s: 0 for s in _cspec}, "t0": {s: None for s in _cspec}, "gate": False,
                         "dq": {s: {j: 0.0 for j in W.JOINT_ORDER} for s in _cspec}}
            else:
                print("[주의] --carry-lift-cm: 값이 0 이거나 그 팔을 쓰지 않는다 (두 팔일 때만) — 끔")
        if a.grasp_tip_offset_cm:
            try:
                _gspec = {s: v for s, v in TOF.parse_spec(a.grasp_tip_offset_cm).items() if s in sides and bool(np.any(v != 0.0))}
            except ValueError as exc:
                print(f"[STOP] --grasp-tip-offset-cm: {exc}")
                if pol is not None:
                    pol.close()
                return 2
            if _gspec:
                gtip = {"off": _gspec, "gain": {s: 1.0 for s in _gspec}, "st": {s: 0 for s in _gspec}, "t0": {s: None for s in _gspec}}
                if a.grasp_tip_skip_start_grip:          # 19-x25 (켤 때만): 팔마다 0 시작 관문 닫힘 전 / 1 닫힘 봄 / 2 다 열림 봄 (이제부터 센다)
                    gtip["arm"] = {s: 0 for s in _gspec}
                if a.grasp_tip_release == "lift":        # 18-d: 상태 0 닫히기 전 / 1 다 닫힘 (유지) / 2 빼는 중·뺌. z0 닫힐 때 높이, tc 다 닫힌 시각, why 빼기 시작한 까닭, dz 지금 오른 높이
                    gtip.update(lift=max(0.0, float(a.grasp_tip_release_lift_cm)), z0={s: None for s in _gspec}, tc={s: None for s in _gspec},
                                why={s: "" for s in _gspec}, dz={s: 0.0 for s in _gspec}, gate=False)
            else:
                print("[주의] --grasp-tip-offset-cm: 값이 모두 0 이거나 그 팔을 쓰지 않는다 — 끔")
        _vc_tip = VCAN.VirtualCan(table, "L1", robot_ref, sides, _q_of)       # FK·집게 틀만 쓴다 (robot_viz 와 같은 체인)
        if a.open_settle_cm is not None:                                         # 19-x18 (켤 때만)
            if "right" in sides:
                oset = {"cm": max(0.0, float(a.open_settle_cm)), "max_s": max(0.0, float(a.open_settle_max_s)), "st": 0, "ho": 0, "t0": None, "dz": float("nan"),
                        "wait_s": 0.0, "why": "", "gate": False}
            else:
                print("[주의] --open-settle-cm: R 팔을 쓰지 않는다 — 끔")
        tip = TOF.TipOffset(_vc_tip.hand_of, {s: v for s, v in tip_spec.items() if s in sides},
                            cap_deg=TOF.CAP_DEG if a.tip_cap_deg is None else float(a.tip_cap_deg),       # 14-y --tip-cap-deg
                            **({"roll_stop": {s: ((math.radians(W.ROLL_STOP_DEG[s]), math.inf) if s == "left" else (-math.inf, math.radians(W.ROLL_STOP_DEG[s])))
                                              for s in sides if s in W.ROLL_STOP_DEG}} if a.tip_roll_stop else {}))         # 19-x19 (켤 때만)
        if a.tip_roll_stop:
            print("[집게 끝 오프셋] 어깨 roll 실물 멈춤 (19-x19): " + ", ".join(f"{'L' if s == 'left' else 'R'} {'≥' if s == 'left' else '≤'} {W.ROLL_STOP_DEG[s]:+.1f}°"
                                                                        for s in sides if s in W.ROLL_STOP_DEG) + " 안에서 나머지 관절로 오프셋을 낸다")
        tip_info = {s: {"dq": {j: 0.0 for j in TOF.POS_JOINTS}, "capped": [], "moved_hjn": [0.0, 0.0, 0.0], "err_cm": 0.0} for s in sides}

        def _gtip_grasp_ok(me):
            """19-x29 (--grasp-tip-release-on-grasp): 이 닫힘이 캔을 잡는 닫힘인가. L 잡기 보정이 켜진 적이 있는 (on_t) L = 보정이 닫기를 허락함,
            그 밖 (보정 없음 · 보정이 한 번도 안 켜짐 · R) = 가상 캔이 쥔 상태 (held_*)."""
            tr = getattr(seq, "track", None)
            ga = getattr(tr, "_ga", None)
            if me == "L" and ga is not None and ga.info.get("on_t") is not None:
                return ga.info.get("close_t") is not None
            vc_ = getattr(tr, "vcan", None)
            return vc_ is not None and str(tr.VCAN.STATE_NAMES[vc_.state]).startswith("held")

        def _gtip_z(side):
            """18-d: 그 손 집게 끝 높이 (cm, base). 엔코더 값, 없으면 (dry-run·NaN) 보낸 명령."""
            if meas_ok:
                try:
                    z = float(_vc_tip.hand_of(side, {j: float(motor.last_measured[side][j]) for j in W.JOINT_ORDER})[0][2])
                    if math.isfinite(z):
                        return z
                except (KeyError, TypeError, ValueError):
                    pass
            return float(_vc_tip.hand_of(side, {j: float(motor.command_target[side][j]) for j in W.JOINT_ORDER})[0][2])

        def table_tip(side, c, f=motor.table_fn, fp=table_plain, m=sag):
            """14-w 판 면: 지금까지의 검사 (보낸 명령·처짐 예상) 와 '실물 예상 자리' (실제로 갈 자세 − 오프셋 몫) 중 엄한 쪽."""
            p = f(side, c)
            base = m.actual(side, c) if m is not None else c
            q = fp(side, {j: float(base[j]) - tip_dq[side].get(j, 0.0) for j in W.JOINT_ORDER})
            out = dict(p if p["g"] <= q["g"] else q)
            out["min"] = min(p["min"], q["min"])
            return out
        motor.table_fn = table_tip
        if view is motor:
            view = SagView(motor, sag_off)
    # 14-m B: 주차 복귀 마지막 구간에서만 판 면 제한을 주차 자세 (모터를 켤 때의 실제 자세 — 판에 얹힘) 의 판 거리까지 허용
    table_core = motor.table_fn
    relax = {"allow": None}

    def table_relaxed(side, c, f=table_core):
        out = f(side, c)
        al = relax["allow"]
        if al is None or al.get(side) is None:
            return out
        out = dict(out)
        out["g"] = out["g"] - al[side]           # al < 0: g 가 주차 자세의 판 거리까지는 0 이상으로 보인다
        return out
    motor.table_fn = table_relaxed
    park_allow = {}                              # 팔마다 허용 (m, 0 이하). seq.park 가 생기면 한 번 계산
    sag_cut = {s: {j: 0.0 for j in W.JOINT_ORDER} for s in sides}
    sag_cut_shown = ()
    blk0 = {"tb": 0, "gb": 0, "yh": 0}

    import virtual_can as VCAN  # type: ignore
    if meta.get("start_slot") == VCAN.PAIR_START and meta.get("can_start_xy_cm") and meta.get("can_goal_xy_cm"):     # 19-b 좌표 쌍 기록: 그 판의 좌표로 움직이는 자리 S0 · G0 를 만든다
        can_check = VCAN.VirtualCan(table, VCAN.PAIR_START, robot_ref, sides, _q_of, pairs=[(*meta["can_start_xy_cm"], *meta["can_goal_xy_cm"])])
    else:
        can_check = VCAN.VirtualCan(table, meta.get("start_slot") or "L1", robot_ref, sides, _q_of)    # 겹침 판정만 (VR 로 보내지 않는다)
    if a.handover_depth_cm is not None:                                    # 19-i (켤 때만)
        if len(sides) != 2 or gripper is None or tip is None:
            print("[주의] --handover-depth-cm 은 두 팔 · 그리퍼가 있을 때만 — 끔")
        else:
            import handover_depth as HDM
            hdep = {"hd": HDM.HandoverDepth(VCAN.VirtualCan(table, "L1", robot_ref, sides, _q_of), can_check._slot_base(can_check.start_name),
                                            float(a.handover_depth_cm), max(0.0, float(a.handover_depth_max_cm)),
                                            HDM.GAP_MIN_CM if a.handover_depth_gap_cm is None else float(a.handover_depth_gap_cm)),
                    "M": HDM, "st": 0, "fill": 0.0, "gain": 0.0, "gate": False, "plan": {}, "t0": None, "t_full": None, "hold_s": 0.0, "closed": False}
    hooks = GripHooks(gripper, DE.JointMap(robot_ref), sides, wa.gripper_open_deg, limits=limits, can_check=can_check, table_fn=table_plain,
                      close_deg=wa.gripper_close_deg)
    hooks.sync = gripper is not None and not a.no_grip_sync       # 14-j: 그리퍼가 바뀌는 동안 재생 시간을 멈춘다
    hooks.hold_s = max(0.0, float(a.grip_hold_s))
    hooks.skip_warm = bool(a.no_grip_warm)                                 # 14-z: AutoRun 이 보고 grip_warm 을 건너뛴다
    hooks.open_first = bool(a.grip_open_first) and bool(a.no_grip_warm)     # 17-e: 건너뛸 때만 열기 확인 단계를 대신 한다
    warm_check = {"done": not a.no_grip_warm}                              # 14-z: 건너뛸 때 캔 놓을 시간에 보드 상태를 한 번 본다 (워밍업이 대신 보던 것)
    pause_on = a.pause_after_close > 0.0 and hooks.sync                    # 14-v (그리퍼 동기가 있어야 닫힘 순간을 안다)
    if a.pause_after_close > 0.0 and not hooks.sync:
        print("[주의] --pause-after-close 는 그리퍼 동기가 켜져 있어야 한다 (--no-grip-sync·그리퍼 없음) — 멈추지 않음")
    if pause_on:
        hooks.pause_s = float(a.pause_after_close)
        _l1 = next(c for c in table_json["can_slots"] if c["name"] == "L1")
        _l1x, _l1y = float(_l1["base_xy_cm"][0]), float(_l1["base_xy_cm"][1])
        pause_ref = np.array([_l1x, _l1y, table.top(_l1x / 100.0) * 100.0 + float(_l1.get("can_bottom_cm", 0.0))])   # L1 캔 축 밑 (판 윗면)

        def pause_center(side):
            """엔코더 (last_measured) FK 집게 가운데 [cm, base] = 집게 끝 (손목 + 13.3 cm·손 축) − 1 cm·손 축 (virtual_can.hand_of 와 같은 체인)."""
            if not hasattr(_models[side], "_O"):
                _models[side].points({j: 0.0 for j in W.JOINT_ORDER})
            q = _q_of(side, motor.last_measured[side])
            M, el = np.eye(4), None
            for n, O in zip(W.JOINT_ORDER, _models[side]._O):
                c_, s_ = math.cos(q[n]), math.sin(q[n])
                M = M @ O @ np.array([[c_, -s_, 0, 0], [s_, c_, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]])
                if n == "elbow_pitch":
                    el = M[:3, 3].copy()
            wr, ax = M[:3, 3], M[:3, 2]
            h = ax if float(np.dot(ax, wr - el)) > 0 else -ax
            return (wr + W.PR.ROBOT_GRIP_M * h) * 100.0 - PAUSE_CENTER_CM * h

        def pause_text(hand):
            side = "left" if hand == "L" else "right"
            c = pause_center(side)
            d = c - pause_ref
            return c, d, (f"{'왼팔' if hand == 'L' else '오른팔'} 엔코더 집게 가운데 base ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}) cm — "
                          f"L1 축 기준 앞뒤 {d[0]:+.1f} 좌우 {d[1]:+.1f} 높이 {d[2]:.1f} cm" + ("" if meas_ok else " (dry-run: 엔코더 없음 — 명령 값)"))

        def on_pause(hands, now):
            print(f"\n[멈춤 {a.pause_after_close:g} 초] {'·'.join('왼팔' if h == 'L' else '오른팔' for h in hands)} 닫힘 — 팔·그리퍼 그대로. 실물 집게 끝을 잰다 (아무 키 = 멈춤)")
            for h in hands:
                print("   " + pause_text(h)[2])
        hooks.on_pause = on_pause
    can_corr = None                      # 18-e 추가 --policy-can-basis encoder-nocorr: 가상 캔이 보는 엔코더 자세에서 뺄 보정 몫 (없으면 None)
    pend = None                          # 18-e --policy-end-after-release (기본 끔): 쥐었던 손이 다 열린 뒤 S 초에 정책을 끝낸다
    if pol is not None:
        pol.sync = hooks.sync                                                 # 14-j 그리퍼 동기를 정책에도
        pol.bind(view if tip is None else MeasShift(view, tip_dq))        # 14-w: 정책 관측은 오프셋 몫을 뺀 값                                                        # 관측: 논리 명령 (처짐 보정 켜면 보정 전 목표) / 실기동은 엔코더
        if carry is not None:
            pol.can_shift = carry["dq"]                                       # 18-e: 가상 캔이 보는 엔코더 자세에서 든 몫을 뺀다 (값은 고리에서 갱신)
        if a.policy_can_basis == "encoder-nocorr" and tip is not None and (ptip is not None or gtip is not None or carry is not None or hdep is not None):
            can_corr = {s: {j: 0.0 for j in W.JOINT_ORDER} for s in sides}    # 18-e 추가: 세 보정이 더한 관절 변화 (보낼 목표 − 기본 오프셋만 넣은 목표, rad)
            pol.can_shift = can_corr
        if a.policy_end_after_release is not None:
            pend = {"S": max(0.0, float(a.policy_end_after_release)), "held": False, "t0": None, "why": "",
                    "armed": 2 if not a.policy_end_skip_start_grip else 0,   # 19-x23 (켤 때만 0): 처음 닫힘 → 다 열림 (top 시작 관문) 한 번을 건너뛴 뒤 센다
                    "can_held": not a.policy_end_after_held}                # 19-x26 (켤 때만 False): 가상 캔이 쥔 상태 (held_*) 가 된 뒤의 놓음만 센다
    elif a.policy_end_after_release is not None:
        print("[주의] --policy-end-after-release 는 정책 실행 (--policy) 에서만 쓴다 — 무시")
    seq = AT.AutoRun(reps * wa.replay_repeat, beeper, move_s=wa.replay_move_s, wait_s=wa.replay_wait_s, vr_guard=vr_link,
                     stop_keys="아무 키나·Ctrl+C = 주차 뒤 멈춤", grip_hooks=hooks)
    # 작업 지시 14-ab --place-lower-cm (기본 0 = 꺼짐): 판에 놓는 열기 직전 그 손을 base 아래로
    plow = None
    plow_info = {"side": "", "cmd": 0.0, "act": 0.0, "blocked": 0, "said": set()}
    if a.place_lower_cm > 0.0:
        if pol is not None:
            print("[주의] --place-lower-cm 은 녹화 재생에만 건다 (정책 실행은 그리퍼 기록이 미리 없다) — 끔")
        else:
            import tip_offset as TOF
            import virtual_can as VCAN  # type: ignore
            _vc_pl = VCAN.VirtualCan(table, "L1", robot_ref, sides, _q_of)
            plow = {"d": float(a.place_lower_cm), "T": max(0.05, float(a.place_lower_s)), "events": {},
                    "ik": TOF.TipOffset(_vc_pl.hand_of, {}, cap_deg=TOF.CAP_DEG if a.tip_cap_deg is None else float(a.tip_cap_deg)),
                    "hand": _vc_pl.hand_of}

    def place_events(track):
        """14-ab: 판에 놓는 열기 = 한 손이 1 → 0 이 되는 프레임에서 다른 손은 그 앞뒤 모두 0 (아무도 같이 쥐지 않음).
        인계로 놓는 열기 (다른 손이 이미 닫혀 캔을 받은 뒤) 는 다른 손이 1 이라 빠진다. 0049_ep13: R 215 만 (L 115 는 R 이 1 이라 뺌)."""
        key = id(track)
        if key not in plow["events"]:
            ev = []
            g = np.asarray(getattr(track, "grip", np.zeros((0, 2))), float)
            tt = np.asarray(getattr(track, "t", np.zeros(0)), float)
            for i in range(1, len(g)):
                for k, side in ((0, "left"), (1, "right")):
                    if side in sides and g[i - 1, k] >= 0.5 and g[i, k] < 0.5 and g[i - 1, 1 - k] < 0.5 and g[i, 1 - k] < 0.5:
                        ev.append((float(tt[i]), side, i))
            plow["events"][key] = ev
        return plow["events"][key]

    def place_lower_now(track):
        """지금 재생 시각 tau 에서 팔마다 내릴 양 (cm). 열기 직전 T 초 (재생 배율 반영) 0 → D, 그 프레임 (grip_wait 로 열기 끝까지) D, 뒤 T 초 D → 0."""
        out = {s: 0.0 for s in sides}
        w = plow["T"] * float(getattr(track, "speed", 1.0))
        tau = float(track.tau)
        for tc, side, _ in place_events(track):
            if tc - w <= tau <= tc:
                x = (tau - (tc - w)) / w
            elif tc < tau <= tc + w:
                x = 1.0 - (tau - tc) / w
            else:
                continue
            x = min(max(x, 0.0), 1.0)
            out[side] = max(out[side], plow["d"] * x * x * (3.0 - 2.0 * x))
        return out

    def block_text(m, goal):
        """14-m: 첫 자세에 못 갔을 때 실제로 막은 것 (보낸 명령이 관절 한계에 붙음 / 판 면 / 양팔 충돌 / 비틀기 범위)."""
        out = []
        for s in sides:
            for j in W.JOINT_ORDER:
                if abs(m.command_target[s][j] - goal[s][j]) <= AT.READY_TOL_RAD:
                    continue
                c = motor.command_target[s][j]
                lo, hi = motor.urdf_limits[s][j]
                if min(abs(c - lo), abs(c - hi)) < 1e-6:
                    out.append(f"관절 한계 {s}.{j} (차 {math.degrees(m.command_target[s][j] - goal[s][j]):+.1f}°)")
                elif j == "shoulder_yaw" and motor.yaw_dyn is not None and s in motor.yaw_dyn:
                    wlo, whi = motor._yaw_window(s, motor.command_target[s]["shoulder_pitch"])
                    if min(abs(c - wlo), abs(c - whi)) < 1e-6:
                        out.append(f"비틀기 범위 {s}.shoulder_yaw")
        tb, gb, yh = motor.table_blocks - blk0["tb"], motor.gap_blocks - blk0["gb"], getattr(motor, "yaw_holds", 0) - blk0["yh"]
        if tb:
            out.append(f"판 면 (막음 {tb} 번)")
        if gb:
            out.append(f"양팔 충돌 방지 (막음 {gb} 번)")
        if yh:
            out.append(f"비틀기 범위로 어깨 앞뒤 멈춤 {yh} 번")
        return " · ".join(out) if out else "관절 한계·판 면·양팔 충돌이 아님 (속도 제한?)"
    seq.block_text = block_text

    # ---- 시작 화면
    cmd_line = "vr_control_watchdog_biarm.py " + " ".join(wa.argv_equiv)
    banner([f"{'정책 실행' if pol is not None else '녹화 재생'} — {pol.label if pol is not None else short_name(ep_path)}  ({meta.get('result')}, {reps[0].duration:.1f} 초 × 1/{wa.replay_speed:g}, {wa.replay_repeat} 회)"
            + ("   🧪 DRY-RUN (모터 안 움직임)" if wa.dry_run else ""),
            (f"캔 놓을 곳: 좌표 (x {meta['can_start_xy_cm'][0]:g}, y {meta['can_start_xy_cm'][1]:g}) — 판의 로봇 쪽 가장자리에서 {meta['can_start_xy_cm'][0] - 7:g} cm, "
             f"판 왼쪽 끝에서 {30.25 - meta['can_start_xy_cm'][1]:g} cm (19-b 좌표 쌍 기록, 도착 x {meta['can_goal_xy_cm'][0]:g}, y {meta['can_goal_xy_cm'][1]:g})"
             if (meta.get("start_slot") == "S0" and meta.get("can_start_xy_cm") and meta.get("can_goal_xy_cm")) else
             f"캔 놓을 곳: {slot_text(table_json, meta.get('start_slot', ''))}"),
            f"목표 자리: {meta.get('goal_slot') or '-'}   첫 캔 위치 (기록): x {reps[0].ep['can_pos'][0][0]:.1f}, y {reps[0].ep['can_pos'][0][1]:.1f} cm",
            f"기준값: {ref_path.name}   판: {Path(wa.table).name} (몸체 {wa.table_body})   그리퍼: "
            + (wa.gripper_port if wa.gripper_port else ("가상" if wa.grip_virtual else "없음"))
            + f"   닫힘 L {wa.gripper_close_deg[0]:g}° R {wa.gripper_close_deg[1]:g}° / 열림 {wa.gripper_open_deg[0]:g}° {wa.gripper_open_deg[1]:g}°",
            *([f"처짐 보정 (14-l): 🔵 켬 — 모델 {sag.info.get('made', '?')} (로그 {len(sag.info.get('files', []))} 개), 관절당 상한 {math.degrees(sag.cap):.0f}°, "
               "어깨 3 관절·팔꿈치만. 판 면은 보낸 명령과 실제로 갈 자세 둘 다"
               + ("" if all(v == 1.0 for v in sag_mult.values()) else
                  "   배율 " + " ".join(f"{'L' if s == 'left' else 'R'} ×{sag_mult[s]:g}" for s in sides) + " (14-t')")] if sag is not None else []),
            *([f"집게 끝 오프셋 (14-w): 🔵 " + "  ".join(f"{'L' if s == 'left' else 'R'} h {tip.off[s][0]:+g} j {tip.off[s][1]:+g} n {tip.off[s][2]:+g}"
                                                       for s in sides if s in tip.off) + f" cm (논리 목표의 집게 틀) — 위치 4 관절 IK, 관절당 상한 {tip.cap_deg:g}°. "
               "VR 표시·정책 관측은 오프셋 전, 판 면은 실물 예상 자리도"] if (tip is not None and ((ptip is None and gtip is None and carry is None) or tip.off)) else []),
            *([f"옮기는 동안 들어 두기 (18-e): 🔵 " + "  ".join(f"{'L' if s == 'left' else 'R'} +{d:g} cm" for s, d in carry["d"].items())
               + f" (base 위쪽) — 그 손만 쥔 뒤 {CARRY_RAMP_S:g} 초에 걸쳐 들고, 열기 전에 열기 명령을 붙잡고 {CARRY_RAMP_S:g} 초에 걸쳐 내린 뒤 연다. 관절당 상한 {tip.cap_deg:g}°"] if carry is not None else []),
            *([f"인계 목표 깊이 (19-i): 🔵 R 이 닫으려 할 때 L 손의 캔에 대한 깊이 < {hdep['hd'].s_star:g} cm 면 R 손 축으로 채움 (상한 {hdep['hd'].max_cm:g} cm, "
               f"두 그리퍼 간격 ≥ {hdep['hd'].gap_min:g} cm), 다 채울 때까지 닫기 붙잡음, R 이 연 뒤 {hdep['M'].RAMP_S:g} 초에 뺌"] if hdep is not None else []),
            *([f"놓은 뒤 끝내기 (18-e): 🔵 쥐었던 손이 다 열린 뒤 {a.policy_end_after_release:g} 초 → 정책 끝 (끝 자세 유지 → 수직 들기 → 주차)"]
              if (pol is not None and a.policy_end_after_release is not None) else []),
            *([f"잡는 동안 집게 끝 보정 (18-c): 🔵 " + "  ".join(f"{'L' if s == 'left' else 'R'} h {v[0]:+g} j {v[1]:+g} n {v[2]:+g}" for s, v in gtip["off"].items())
               + (f" cm — 시작부터 그 손이 다 닫힐 때까지 집게 끝 오프셋에 더하고, 그 뒤 {GRASP_TIP_RAMP_S:g} 초에 걸쳐 뺌. 관절당 상한 {tip.cap_deg:g}°" if "lift" not in gtip else
                  f" cm — 시작부터 넣고, 다 닫힌 뒤 집게 끝이 {gtip['lift']:g} cm 올라가면 (못 오르면 {GRASP_TIP_LIFT_MAX_S:g} 초 뒤, 다른 손이 닫기 전에는 반드시) {GRASP_TIP_RAMP_S:g} 초에 걸쳐 뺌 (18-d lift). 관절당 상한 {tip.cap_deg:g}°")]
              if gtip is not None else []),
            *([f"인계 뒤 집게 끝 보정 (18-b): 🔵 " + "  ".join(f"{'L' if s == 'left' else 'R'} h {v[0]:+g} j {v[1]:+g} n {v[2]:+g}" for s, v in ptip["off"].items())
               + f" cm — 그 손만 쥔 뒤 {PLACE_TIP_RAMP_S:g} 초에 걸쳐 집게 끝 오프셋에 더함 (연 뒤에도 유지, 주차 복귀에서 풀림). 관절당 상한 {tip.cap_deg:g}°"] if ptip is not None else []),
            *([f"내려놓기 (14-ab): 판에 놓는 열기 직전 {plow['T']:g} 초 동안 그 손을 base 아래로 {plow['d']:g} cm (열기 끝까지 유지, 연 뒤 {plow['T']:g} 초에 되돌림, 판 면 검사 그대로). 대상: "
               + (", ".join(f"{'L' if sd == 'left' else 'R'} 프레임 {fi}" for _, sd, fi in place_events(reps[0])) or "없음")] if plow is not None else []),
            *(["그리퍼 워밍업 생략 (14-z --no-grip-warm): 첫 자세 도착 → 바로 캔 놓을 시간. 보드 상태 (?) 는 그대로 읽는다"] if a.no_grip_warm else []),
            *([f"⏸ 닫힌 뒤 멈춤 (14-v): 그리퍼가 닫힐 때마다 {a.pause_after_close:g} 초 그대로 선다 — 엔코더 FK 집게 가운데와 L1 축 차이를 띄움 (실측용)"] if pause_on else []),
            "",
            "■ 멈추기: 이 창에서 아무 키나 (스페이스·엔터 포함) 또는 Ctrl+C → 그 자리 → 주차 → 힘 빼기",
            "  걸림·발산" + (" ·VR 끊김" if wa.vr_guard else "") + " 은 자동으로 멈춘다. Ctrl+C 두 번 = 바로 힘 빼기",
            "",
            f"순서: 삐 + 5 초 → 모터 켬 → 첫 자세 ({wa.replay_move_s:g} 초) → 캔 놓을 시간 {wa.replay_wait_s:g} 초 → 재생 → 끝 자세 2 초 → 주차 → 감쇠",
            f"같은 워치독 옵션: {cmd_line}"])
    print(f"로그: {log_csv}")

    flag = {"quit": False, "stop": None}
    motor.initialize()
    if receiver is not None:
        receiver.start()
    threading.Thread(target=key_worker, args=(flag,), daemon=True).start()
    log_file = open(log_csv, "w", newline="")
    log = csv.writer(log_file)
    header = ["t", "mode", "auto_round", "auto_stage", "replay_phase", "replay_t", "replay_frame"]
    for s in sides:
        for j in W.JOINT_ORDER:
            header += [f"{s}_{j}_cmd", f"{s}_{j}_meas"]
    header += ["grip_L", "grip_R", "table_clear_cm", "table_blocks", "gap_blocks", "stall_level", "vr_state"]
    # 14-f D: 보낸 그리퍼 각도와 시각 (마지막으로 부른 것), 부른 수·직렬에 쓴 수, 보드가 알려 준 지금·목표 각도·붙음, 상태 나이, 보드 재시작 수
    header += ["grip_cmd_A", "grip_cmd_B", "grip_cmd_t", "grip_calls", "grip_writes",
               "ard_A_cur", "ard_A_tgt", "ard_A_att", "ard_B_cur", "ard_B_tgt", "ard_B_att", "ard_status_age_s", "ard_resets",
               "ard_Q", "ard_M", "ard_S", "grip_wait", "grip_wait_total_s"]                      # 14-j
    header += ["grip_close_A", "grip_close_B"]                                                  # 14-n 닫힘 각도 (설정값)
    if sag is not None:                                                                         # 14-l (켤 때만)
        header += ["sag_gain", "sag_table_cm"] + [f"{s}_{j}_sag" for s in sides for j in W.JOINT_ORDER]
        header += ["sag_cut_max_deg", "sag_cut_joint"]                                          # 14-m: 관절 한계로 덜 한 보정
    header += ["park_relax_cm"]                                                                 # 14-m: 주차 마지막 구간 판 면 허용 (팔 중 작은 값)
    if pol is not None:
        header += PT.CSV_COLUMNS + ["pol_clip_deg"]                           # 15b (정책일 때만)
    if sag is not None and a.sag_gain_arm:
        header += [f"sag_gain_{s}" for s in sides]
    if pause_on:                                                                # 14-v (켤 때만): 멈춤 중인 손의 엔코더 FK 집게 가운데·L1 축 차
        header += ["pause_hand", "pause_left_s", "pause_cx", "pause_cy", "pause_cz", "pause_dL1_fwd", "pause_dL1_side", "pause_dL1_h"]
    if tip is not None:                                                         # 14-w (켤 때만): 세기, 팔마다 끝 이동 (논리 목표 틀 h·j·n cm)·관절 몫 (도)
        header += ["tip_gain"] + [f"{s}_tip_{k}" for s in sides for k in ("h", "j", "n")] + [f"{s}_{j}_tipdq_deg" for s in sides for j in TOF.POS_JOINTS]
        header += ["tip_capped", "tip_cut_max_deg", "tip_err_cm"]
    if ptip is not None:                                                        # 18-b (켤 때만): 팔마다 상태 (0 전 / 1 같이 쥠 / 2 걸림)·세기 0 ~ 1 (tip_gain 을 곱하기 전)
        header += [f"{s}_place_tip_{k}" for s in ptip["off"] for k in ("state", "gain")]
    if gtip is not None:                                                        # 18-c (켤 때만): 팔마다 상태 (0 닫히기 전 / 1 다 닫힘)·세기 1 ~ 0 (tip_gain 을 곱하기 전)
        header += [f"{s}_grasp_tip_{k}" for s in gtip["off"] for k in ("state", "gain")]
        if "arm" in gtip:                                                       # 19-x25 (켤 때만): 시작 관문 건너뛰기 상태 0 / 1 / 2
            header += [f"{s}_grasp_tip_arm" for s in gtip["off"]]
        if "lift" in gtip:                                                      # 18-d (lift 일 때만): 닫힌 뒤 오른 높이 cm, 빼기 시작한 까닭 (height / time / handover / open), 다른 손 닫기 붙잡는 중 1
            header += [f"{s}_grasp_tip_{k}" for s in gtip["off"] for k in ("lift_cm", "why")] + ["grasp_tip_grip_hold"]
    if carry is not None:                                                       # 18-e (켤 때만): 팔마다 상태 (0 전 / 1 같이 쥠 / 2 들어 둠 / 3 내리는 중·내림)·세기·든 양 cm, 열기 붙잡는 중 1
        header += [f"{s}_carry_{k}" for s in carry["d"] for k in ("state", "gain", "cm")] + ["carry_grip_hold"]
    if psag is not None:                                                        # 19-x20 (켤 때만): 상태 (0 / 1 두 손 쥠 / 2 R 만 쥠), R 팔꿈치 처짐 보정 비율
        header += ["psag_state", "psag_elbow_scale"]
    if oset is not None:                                                        # 19-x18 (켤 때만): 상태 (0 전 / 1 늦추는 중 / 2 보냄), 실측 − 의도 높이 cm, 늦춘 시간, 보낸 까닭, 붙잡는 중
        header += ["oset_state", "oset_dz_cm", "oset_wait_s", "oset_why", "oset_grip_hold"]
    if hdep is not None:                                                        # 19-i (켤 때만): 상태, 잰 깊이, 원한 · 채운 양, 세기, 채운 뒤 깊이 · 그리퍼 간격, 줄인 까닭, 붙잡은 시간, 붙잡는 중
        header += ["hd_state", "hd_s_cm", "hd_want_cm", "hd_fill_cm", "hd_gain", "hd_s_after_cm", "hd_gap_cm", "hd_cut", "hd_hold_s", "hd_grip_hold"]
    if can_corr is not None:                                                    # 18-e 추가 (encoder-nocorr 일 때만): 가상 캔이 보는 자세에서 뺀 보정 몫의 가장 큰 관절 값 (도)
        header += ["can_corr_max_deg"]
    if pend is not None:                                                        # 18-e (켤 때만): 다 열린 뒤 지난 시간 (초), 끝낸 까닭
        header += ["pol_end_wait_s", "pol_end"]
    if plow is not None:                                                        # 14-ab (켤 때만): 내리는 손, 목표 양, 실제 내린 양 (논리 명령 끝 높이 차), 내리는 중 판 면 막힘 수
        header += ["place_lower_side", "place_lower_cmd_cm", "place_lower_cm", "place_lower_blocked"]
    log.writerow(header)
    pol_stop = None                       # 15b: 정책 멈춤 이유 (한 번)
    clip_hist = []                        # (시각, 10° 넘음?) 최근 PT_CLIP_S 초
    clip_deg = 0.0

    rate = W.RateLimiter(frequency=W.LOOP_HZ)
    prev = time.monotonic()
    last_log = 0.0
    last_print = 0.0
    interrupts = 0
    state = {"hold": True}
    stage_prev = None
    try:
        while not seq.finished:
            try:
                now = time.monotonic()
                dt = now - prev
                prev = now
                if flag["stop"]:
                    if seq.stage not in ("user_hold", "vr_park", "stopping", "end"):
                        seq.request_stop(now, view, f"멈춤 ({flag['stop']})")
                    elif flag["stop"] == "Ctrl+C":
                        print("\n[멈춤] Ctrl+C 두 번 — 바로 힘을 뺀다")
                        break
                    flag["stop"] = None
                vr_kind = vr_link.update(now, receiver.get_latest()) if vr_link is not None else None
                if pol is not None and pol_stop is None and seq.stage in ("play", "grip_wait"):      # 15b 정책 전용 멈춤
                    over = (len(clip_hist) > 1 and clip_hist[-1][0] - clip_hist[0][0] >= PT_CLIP_S * 0.95
                            and sum(b for _, b in clip_hist) >= PT_CLIP_FRAC * len(clip_hist))
                    why = pol.stop_reason or (f"안전장치가 정책 목표를 {PT_CLIP_DEG:g}° 넘게 깎음 (최근 {PT_CLIP_S:g} 초의 {PT_CLIP_FRAC:.0%} 넘게, 지금 {clip_deg:.0f}°)"
                                              if over else "")
                    if why:
                        pol_stop = why
                        pol.stop_reason = pol.stop_reason or why                     # CSV pol_stop 에도
                        print(f"\n[정책] 🔴 멈춤 — {why}")
                        seq.request_stop(now, view, f"정책 멈춤 ({why})")
                if pend is not None and not pend["why"] and seq.stage in ("play", "grip_wait") and getattr(pol, "phase", "") == "play":     # 18-e B
                    _g = seq.track.grip_at(seq.track.tau)
                    _gs4 = gripper.state() if gripper is not None else {"L": 0, "R": 0}
                    _any = bool(_g["L"] or _g["R"] or pol.desired["L"] or pol.desired["R"] or int(_gs4["L"] or 0) or int(_gs4["R"] or 0))
                    if not pend["can_held"] and str(pol.VCAN.STATE_NAMES[pol.vcan.state]).startswith("held"):
                        pend["can_held"] = True                        # 19-x26: 캔을 쥔 적이 있다
                    if pend["armed"] < 2:                              # 19-x23: 시작 자세의 닫힘 (top 시작 관문) 은 쥔 것으로 치지 않는다
                        if pend["armed"] == 0 and _any:
                            pend["armed"] = 1                          # 시작 관문 닫힘을 봄
                        elif pend["armed"] == 1 and not _any:
                            pend["armed"] = 2                          # 다 열림 — 이제부터 센다
                    elif _any and pend["can_held"]:
                        pend["held"], pend["t0"] = True, None          # 쥐고 있다 (다시 쥐면 처음부터 센다)
                    elif pend["held"] and seq.stage == "play":         # 모두 다 열렸다 (grip_wait 끝)
                        if pend["t0"] is None:
                            pend["t0"] = now
                        if now - pend["t0"] >= pend["S"]:
                            pend["why"] = f"놓은 뒤 {pend['S']:g} 초"
                            pol.phase = "done"
                            pol.meta["result"] = "end_after_release"
                            print(f"\n[정책] ■ 쥐었던 손이 다 열린 뒤 {pend['S']:g} 초 — 정책을 끝낸다 (끝 자세 유지 → 수직 들기 → 주차). 정책 시간 {seq.track.tau:.1f} 초")
                target = seq.step(now, dt, view, state, meas_ok, vr_kind)
                if plow is not None and target is not None and seq.stage in ("play", "grip_wait") and getattr(seq, "track", None) is not None:   # 14-ab
                    pl = place_lower_now(seq.track)
                    pls = [s for s in sides if pl[s] > 0.0]
                    if pls:
                        s = pls[0]
                        z_goal = plow["hand"](s, target[s])[0][2]
                        z_now = plow["hand"](s, view.command_target[s])[0][2]              # 지난 걸음까지 실제로 보낸 논리 명령의 끝 높이
                        target = {x: dict(target[x]) for x in sides}
                        target[s], _pli = plow["ik"].apply_vec(s, target[s], (0.0, 0.0, -pl[s]))
                        plow_info.update(side=s, cmd=pl[s], act=max(0.0, z_goal - z_now))
                        if motor.table_blocks > plow_info.get("tb", motor.table_blocks):
                            plow_info["blocked"] += 1
                            if ("tb", s) not in plow_info["said"]:
                                plow_info["said"].add(("tb", s))
                                print(f"\n[내려놓기] 판 면에 막힘 — 그 걸음에서 멈춤 ({'L' if s == 'left' else 'R'} 내린 양 {plow_info['act']:.1f} / {pl[s]:.1f} cm)")
                        if _pli["capped"] and ("cap", s) not in plow_info["said"]:
                            plow_info["said"].add(("cap", s))
                            print(f"\n[내려놓기] 관절당 상한 {plow['ik'].cap_deg:g}° 에 걸림: " + ", ".join(_pli["capped"]))
                    else:
                        plow_info.update(side="", cmd=0.0, act=0.0)
                    plow_info["tb"] = motor.table_blocks
                pol_target = target if (pol is not None and seq.stage == "play") else None
                if seq.stage == "approach" and stage_prev != "approach":     # 14-m: 막음 수는 첫 자세로 갈 때부터 센다
                    blk0.update(tb=motor.table_blocks, gb=motor.gap_blocks, yh=getattr(motor, "yaw_holds", 0))
                stage_prev = seq.stage
                if not warm_check["done"] and seq.stage == "place" and seq.t_stage is not None and now - seq.t_stage >= 1.0:
                    warm_check["done"] = True                    # 14-z: 워밍업이 보던 것 — 보드 상태가 오는지, 14-g 뒤 펌웨어 (W 칸) 인지, 열려 있는지
                    if hooks.board:
                        lk = gripper.link
                        st, qwd = lk.status, lk.status_qwd
                        if st is None or now - st[0] > 1.0:
                            print("\n[그리퍼] ⚠ 워밍업 생략: 보드 상태 (?) 를 아직 못 받았다 — 그리퍼 동기가 4 초 뒤 멈출 수 있다. 직렬·전원 확인")
                        elif qwd is None:
                            print("\n[그리퍼] ⚠ 워밍업 생략: 보드 상태에 W 칸이 없다 (14-g 전 펌웨어) — 그리퍼 동기가 안 될 수 있다")
                        elif not (st[1] == st[2] == hooks.open_deg[0] and st[4] == st[5] == hooks.open_deg[1]):
                            print(f"\n[그리퍼] ⚠ 워밍업 생략: 그리퍼가 열림 ({hooks.open_deg[0]}°, {hooks.open_deg[1]}°) 이 아니다 — 지금 A {st[1]}° B {st[4]}°")
                        else:
                            print("\n[그리퍼] 워밍업 생략 — 보드 상태 정상 (열림, 대기열 비었음)")
                if getattr(seq, "park", None) is not None and not park_allow:
                    for s in sides:                              # 주차 자세 (보정했으면 그 보낸 명령도) 의 판 거리, 0 이하만
                        cands = [table_core(s, seq.park[s])["g"]]
                        if sag is not None:
                            cands.append(table_core(s, sag.command_for(s, seq.park[s], sag_mult[s]))["g"])
                        park_allow[s] = min(0.0, min(cands)) - PARK_RELAX_MARGIN_M
                el_st = now - seq.t_stage if seq.t_stage is not None else 0.0
                last_leg = ((seq.stage == "return" and el_st >= PARK_RELAX_FROM * seq.move_s)
                            or (seq.stage == "vr_park" and el_st >= PARK_RELAX_FROM * AT.VR_PARK_S))
                relax["allow"] = park_allow if (last_leg and park_allow) else None
                if target is not None and (sag is not None or tip is not None):   # 14-l: 실제가 목표에 오게 보낼 명령 (논리 명령 = 보낸 명령 + 보정량)
                    tt = target
                    if tip is not None:                              # 14-w: 목표 → 집게 끝 오프셋 IK (그 뒤 처짐 보정)
                        want_t = 1.0 if (motor.position_mode and seq.stage in TIP_STAGES) else 0.0
                        tip_gain += max(-dt / TIP_RAMP_S, min(dt / TIP_RAMP_S, want_t - tip_gain))
                        if ptip is not None:                         # 18-b: 인계가 끝났는지 (그 손만 쥠) 보고 세기를 0.5 초에 걸쳐 올린다
                            if tip_gain <= 0.0 or seq.stage in ("lead", "settle", "approach"):
                                for s in ptip["off"]:                # 주차 복귀로 다 풀렸거나 다음 회차 → 처음부터
                                    ptip["st"][s], ptip["t0"][s] = 0, None
                            elif seq.stage in ("play", "grip_wait") and getattr(seq, "track", None) is not None:
                                _g = seq.track.grip_at(seq.track.tau)
                                for s in ptip["off"]:
                                    me, ot = ("L", "R") if s == "left" else ("R", "L")
                                    if ptip["st"][s] == 0 and _g[me] and _g[ot]:
                                        ptip["st"][s] = 1            # 같이 쥠 (인계 중)
                                    elif ptip["st"][s] == 1 and not _g[me]:
                                        ptip["st"][s], ptip["t0"][s] = 0, None       # 받는 손이 먼저 엶 → 인계가 아님
                                    elif ptip["st"][s] == 1 and not _g[ot]:
                                        if ptip["t0"][s] is None:
                                            ptip["t0"][s] = now
                                        # 다른 손 열기가 끝남: 그리퍼 동기면 grip_wait 가 끝나 play 로 돌아온 때, 아니면 여는 시간 (서보 모델) 뒤
                                        _k = 0 if ot == "L" else 1
                                        _need = 0.0 if hooks.sync else open_sim_s(wa.gripper_open_deg, wa.gripper_close_deg, (_k == 0, _k == 1))
                                        if seq.stage == "play" and now - ptip["t0"][s] >= _need:
                                            ptip["st"][s] = 2
                                            print(f"\n[인계 뒤 집게 끝 보정] {'L' if s == 'left' else 'R'} 만 쥠 — {PLACE_TIP_RAMP_S:g} 초에 걸쳐 넣는다")
                            for s in ptip["off"]:
                                _w = 1.0 if ptip["st"][s] == 2 else 0.0
                                ptip["gain"][s] += max(-dt / PLACE_TIP_RAMP_S, min(dt / PLACE_TIP_RAMP_S, _w - ptip["gain"][s]))
                        if gtip is not None:                         # 18-c: 그 손이 처음 다 닫혔는지 보고 세기를 0.5 초에 걸쳐 내린다
                            if tip_gain <= 0.0 or seq.stage in ("lead", "settle", "approach"):
                                for s in gtip["off"]:                # 주차 복귀로 다 풀렸거나 다음 회차 → 처음부터 (다시 넣는다)
                                    gtip["st"][s], gtip["t0"][s] = 0, None
                                    if "arm" in gtip:
                                        gtip["arm"][s] = 0
                                    if "lift" in gtip:
                                        gtip["z0"][s], gtip["tc"][s], gtip["why"][s], gtip["dz"][s] = None, None, "", 0.0
                            elif seq.stage in ("play", "grip_wait") and getattr(seq, "track", None) is not None:
                                _g = seq.track.grip_at(seq.track.tau)
                                for s in gtip["off"]:
                                    me = "L" if s == "left" else "R"
                                    if "arm" in gtip and gtip["arm"][s] < 2:   # 19-x25: 시작 관문 닫힘 → 다 열림 한 번은 치지 않는다
                                        if gtip["arm"][s] == 0 and _g[me]:
                                            gtip["arm"][s] = 1
                                        elif gtip["arm"][s] == 1 and not _g[me]:
                                            gtip["arm"][s] = 2
                                            print(f"\n[잡는 동안 집게 끝 보정] {me} 시작 관문 닫힘을 건너뜀 — 다음 닫힘 (캔 잡기) 에서 뺀다")
                                    elif gtip["st"][s] == 0 and _g[me] and (not a.grasp_tip_release_on_grasp or _gtip_grasp_ok(me)):
                                        if gtip["t0"][s] is None:
                                            gtip["t0"][s] = now
                                        # 다 닫힘: 그리퍼 동기면 grip_wait 가 끝나 play 로 돌아온 때, 아니면 닫는 시간 (서보 모델) 뒤
                                        _k = 0 if me == "L" else 1
                                        _need = 0.0 if hooks.sync else open_sim_s(wa.gripper_open_deg, wa.gripper_close_deg, (_k == 0, _k == 1))
                                        if seq.stage == "play" and now - gtip["t0"][s] >= _need:
                                            gtip["st"][s] = 1
                                            if "lift" not in gtip:
                                                print(f"\n[잡는 동안 집게 끝 보정] {me} 다 닫힘 — {GRASP_TIP_RAMP_S:g} 초에 걸쳐 뺀다")
                                            else:                    # 18-d: 닫힐 때 높이를 적어 두고 유지
                                                gtip["tc"][s], gtip["z0"][s], gtip["dz"][s] = now, _gtip_z(s), 0.0
                                    elif gtip["st"][s] == 0:
                                        gtip["t0"][s] = None         # 닫기 전에 다시 열림 → 처음부터 센다
                                    if "lift" in gtip and gtip["st"][s] == 1 and gtip["tc"][s] is not None:      # 18-d: 유지 중 — 뺄 때를 본다
                                        ot = "R" if me == "L" else "L"
                                        _des = getattr(seq.track, "desired", None)
                                        gtip["dz"][s] = _gtip_z(s) - gtip["z0"][s]
                                        _why = ("handover" if (_g[ot] or (_des is not None and _des.get(ot))) else "open" if not _g[me]
                                                else "height" if gtip["dz"][s] >= gtip["lift"] else "time" if now - gtip["tc"][s] >= GRASP_TIP_LIFT_MAX_S else "")
                                        if _why:
                                            gtip["st"][s], gtip["why"][s] = 2, _why
                                            print(f"\n[잡는 동안 집게 끝 보정] {me} " + {"height": f"집게 끝이 {gtip['dz'][s]:.1f} cm 올라감", "time": f"다 닫힌 뒤 {GRASP_TIP_LIFT_MAX_S:g} 초 (오른 높이 {gtip['dz'][s]:.1f} cm)",
                                                                                   "handover": "다른 손이 닫으려 함 — 그 닫기를 붙잡고 먼저", "open": "그 손이 다시 엶"}[_why] + f" — {GRASP_TIP_RAMP_S:g} 초에 걸쳐 뺀다")
                                    elif "lift" in gtip and gtip["st"][s] == 2 and gtip["z0"][s] is not None:
                                        gtip["dz"][s] = _gtip_z(s) - gtip["z0"][s]
                            _rel = 2 if "lift" in gtip else 1
                            for s in gtip["off"]:
                                _w = 0.0 if gtip["st"][s] == _rel else 1.0
                                gtip["gain"][s] += max(-dt / GRASP_TIP_RAMP_S, min(dt / GRASP_TIP_RAMP_S, _w - gtip["gain"][s]))
                            if "lift" in gtip:                       # 18-d: 다 빠지기 전에 다른 손이 닫으려 하면 그 그리퍼 명령을 붙잡는다 (다 빠지면 놓는다)
                                _hold = False
                                if gripper is not None and seq.stage in ("play", "grip_wait") and getattr(seq, "track", None) is not None:
                                    _g2 = seq.track.grip_at(seq.track.tau)
                                    _d2 = getattr(seq.track, "desired", None) or _g2
                                    _gs = gripper.state()
                                    for s in gtip["off"]:
                                        ot = "L" if s == "right" else "R"       # 다른 손: 닫으려 하는데 아직 명령을 안 보냄
                                        if gtip["st"][s] == 2 and gtip["gain"][s] > 0.0 and (_g2[ot] or _d2.get(ot)) and not int(_gs[ot]):
                                            _hold = True
                                gtip["gate"] = _hold
                        if carry is not None:                        # 18-e: 인계가 끝났는지 (그 손만 쥠) 보고 들고, 그 손이 열려 하면 열기를 붙잡고 내린다
                            if tip_gain <= 0.0 or seq.stage in ("lead", "settle", "approach"):
                                for s in carry["d"]:
                                    carry["st"][s], carry["t0"][s] = 0, None
                                carry["gate"] = False
                            elif seq.stage in ("play", "grip_wait") and getattr(seq, "track", None) is not None:
                                _g = seq.track.grip_at(seq.track.tau)
                                _d3 = getattr(seq.track, "desired", None) or _g
                                _gs3 = gripper.state() if gripper is not None else None
                                _hold = False
                                for s in carry["d"]:
                                    me, ot = ("L", "R") if s == "left" else ("R", "L")
                                    if carry["st"][s] == 0 and _g[me] and _g[ot]:
                                        carry["st"][s] = 1
                                    elif carry["st"][s] == 1 and not _g[me]:
                                        carry["st"][s], carry["t0"][s] = 0, None
                                    elif carry["st"][s] == 1 and not _g[ot]:
                                        if carry["t0"][s] is None:
                                            carry["t0"][s] = now
                                        _k = 0 if ot == "L" else 1
                                        _need = 0.0 if hooks.sync else open_sim_s(wa.gripper_open_deg, wa.gripper_close_deg, (_k == 0, _k == 1))
                                        if seq.stage == "play" and now - carry["t0"][s] >= _need:
                                            carry["st"][s] = 2
                                            print(f"\n[옮기는 동안 들어 두기] {me} 만 쥠 — {CARRY_RAMP_S:g} 초에 걸쳐 {carry['d'][s]:g} cm 든다")
                                    elif carry["st"][s] == 2 and (not _g[me] or not _d3.get(me)):
                                        carry["st"][s] = 3
                                        print(f"\n[옮기는 동안 들어 두기] {me} 열려 함 — 열기를 붙잡고 {CARRY_RAMP_S:g} 초에 걸쳐 내린 뒤 연다")
                                    if carry["st"][s] == 3 and carry["gain"][s] > 0.0 and _gs3 is not None and int(_gs3[me]):
                                        _hold = True                 # 아직 열기 명령을 안 보냈고 덜 내렸다
                                carry["gate"] = _hold
                            for s in carry["d"]:
                                _w = 1.0 if carry["st"][s] == 2 else 0.0
                                carry["gain"][s] += max(-dt / CARRY_RAMP_S, min(dt / CARRY_RAMP_S, _w - carry["gain"][s]))
                        if hdep is not None:                         # 19-i: R 이 닫으려 할 때 깊이를 재서 모자라면 채운다 (R 닫기 붙잡음), R 이 연 뒤 뺀다
                            _hd = hdep["hd"]
                            if tip_gain <= 0.0 or seq.stage in ("lead", "settle", "approach"):
                                hdep.update(st=0, gate=False, t0=None, t_full=None, closed=False)
                                _hd.rel = None
                            elif seq.stage in ("play", "grip_wait") and getattr(seq, "track", None) is not None:
                                _g = seq.track.grip_at(seq.track.tau)
                                _d4 = getattr(seq.track, "desired", None) or _g
                                _gs4 = gripper.state()

                                def _hd_q(side):
                                    src = motor.last_measured if meas_ok else motor.command_target
                                    return {j: float(src[side][j]) for j in W.JOINT_ORDER}
                                if _hd.rel is None and seq.stage == "play" and _g["L"] and int(_gs4["L"]):
                                    _li = _hd.on_L_closed(_hd_q("left"))
                                    print(f"\n[인계 깊이] L 다 닫힘 — 캔을 L 손에 붙임 (L 깊이 {_li['s']:.1f}, 옆 {_li['j']:+.1f} → 가운데로, 높이 {_li['t']:.1f} cm)")
                                if hdep["st"] == 0 and _hd.rel is not None and _g["L"] and int(_gs4["L"]) and (_g["R"] or _d4.get("R")) and not int(_gs4["R"]):
                                    _exR = np.zeros(3)
                                    if ptip is not None and "right" in ptip["off"]:
                                        _exR = _exR + ptip["gain"]["right"] * ptip["off"]["right"]
                                    if gtip is not None and "right" in gtip["off"]:
                                        _exR = _exR + gtip["gain"]["right"] * gtip["off"]["right"]
                                    _pl = _hd.plan(_hd_q("left"), _hd_q("right"),
                                                   lambda f_, _e=_exR: tip.apply("right", target["right"], tip_gain, extra=_e + np.array([f_, 0.0, 0.0])),
                                                   motor.urdf_limits["right"], table_g=lambda q_: table_plain("right", q_)["g"] * 100.0)
                                    hdep.update(plan=_pl, fill=float(_pl["fill"]), t0=now, t_full=None, st=1 if _pl["fill"] > 0.0 else 2, gate=_pl["fill"] > 0.0)
                                    print(f"\n[인계 깊이] R 닫으려 함 — 깊이 {_pl['s']:.1f} cm (목표 {_hd.s_star:g}) → 채움 {_pl['fill']:.2f} cm"
                                          + (f" (원함 {_pl['want']:.2f}, 줄인 까닭: {_pl['why']})" if _pl["why"] else "") + (" — 닫기를 붙잡고 채운다" if _pl["fill"] > 0.0 else " — 그대로 닫는다"))
                                elif hdep["st"] == 1 and hdep["gain"] >= 1.0 - 1e-9:
                                    if hdep["t_full"] is None:
                                        hdep["t_full"] = now
                                    if now - hdep["t_full"] >= hdep["M"].SETTLE_S:
                                        hdep.update(st=2, gate=False, hold_s=now - hdep["t0"])
                                        print(f"\n[인계 깊이] 다 채움 ({hdep['fill']:.2f} cm) — R 닫기 보냄 (붙잡은 {hdep['hold_s']:.1f} 초)")
                                if hdep["st"] == 2 and int(_gs4["R"]):
                                    hdep["closed"] = True
                                elif hdep["st"] == 2 and hdep["closed"] and not int(_gs4["R"]) and not _g["R"] and hdep["fill"] > 0.0:
                                    hdep["st"] = 3
                                    print(f"\n[인계 깊이] R 엶 — {hdep['M'].RAMP_S:g} 초에 걸쳐 뺀다")
                            _w4 = 1.0 if hdep["st"] in (1, 2) else 0.0
                            hdep["gain"] += max(-dt / hdep["M"].RAMP_S, min(dt / hdep["M"].RAMP_S, _w4 - hdep["gain"]))
                        if oset is not None:                         # 19-x18: R 이 열려 할 때 실측 높이가 의도 높이에 올 때까지 열기를 늦춘다
                            if tip_gain <= 0.0 or seq.stage in ("lead", "settle", "approach") or gripper is None:
                                oset.update(st=0, ho=0, t0=None, gate=False)
                            elif seq.stage in ("play", "grip_wait") and getattr(seq, "track", None) is not None:
                                _g5 = seq.track.grip_at(seq.track.tau)
                                _d5 = getattr(seq.track, "desired", None) or _g5
                                if _g5["L"] and _g5["R"]:                # 놓기만: 두 손이 같이 쥔 뒤 L 이 먼저 열고 R 만 쥔 상태에서 R 이 열 때 (시작 때 여는 것 · 인계 중 다시 잡기는 빼고)
                                    oset["ho"] = 1
                                elif oset["ho"] == 1 and not _g5["L"] and _g5["R"]:
                                    oset["ho"] = 2
                                _open_want = (oset["ho"] == 2 and (not _g5["R"] or not _d5.get("R")) and bool(int(gripper.state()["R"])))     # 열려 하는데 아직 열기를 안 보냄
                                _other = bool((carry is not None and carry["gate"]) or (gtip is not None and gtip.get("gate")) or (hdep is not None and hdep["gate"]))
                                if _open_want and not _other:
                                    if oset["st"] != 1 and oset["st"] != 2:
                                        oset.update(st=1, t0=now, why="")
                                    if oset["st"] == 1:
                                        _qi = {j: float(target["right"][j]) + tip_dq["right"].get(j, 0.0) for j in W.JOINT_ORDER}
                                        _zi = float(_vc_tip.hand_of("right", _qi)[0][2])
                                        _ze = float("nan")
                                        if meas_ok:
                                            try:
                                                _ze = float(_vc_tip.hand_of("right", {j: float(motor.last_measured["right"][j]) for j in W.JOINT_ORDER})[0][2])
                                            except (KeyError, TypeError, ValueError):
                                                _ze = float("nan")
                                        oset["dz"] = _ze - _zi
                                        _el = now - oset["t0"]
                                        if not math.isfinite(_ze):
                                            oset["why"] = "엔코더 없음"
                                        elif abs(oset["dz"]) <= oset["cm"]:
                                            oset["why"] = "닿음"
                                        elif _el >= oset["max_s"]:
                                            oset["why"] = "시간"
                                        if oset["why"]:
                                            oset.update(st=2, wait_s=_el)
                                            print(f"\n[놓기 늦춤] R 열기 보냄 — {_el:.2f} 초 늦춤, 실측 − 의도 높이 {oset['dz']:+.2f} cm ({oset['why']})")
                                elif not _open_want and oset["st"] == 2:
                                    oset.update(st=0, ho=0, t0=None)
                                elif not _open_want and oset["ho"] == 1 and not _g5["R"]:
                                    oset["ho"] = 0
                                oset["gate"] = oset["st"] == 1
                        if (gtip is not None and "lift" in gtip) or carry is not None or hdep is not None or oset is not None:      # 18-d·18-e·19-i·19-x18: 붙잡기 요청이 있으면 그리퍼 명령을 미루고, 다 풀리면 그 고리에서 보낸다
                            _want = bool((gtip is not None and gtip.get("gate")) or (carry is not None and carry["gate"]) or (hdep is not None and hdep["gate"])
                                         or (oset is not None and oset["gate"]))
                            if _want and not grip_hold["on"]:
                                grip_hold["on"] = True
                                if hooks.sync and seq.stage == "grip_wait":
                                    hooks.hold_begin(now)
                            elif grip_hold["on"] and not _want:
                                grip_hold["on"] = False
                                if hooks.held:
                                    hooks.hold_end(now)
                        tt = {}
                        for s in sides:
                            _ex = None
                            if ptip is not None and s in ptip["off"]:
                                _ex = ptip["gain"][s] * ptip["off"][s]
                            if gtip is not None and s in gtip["off"]:
                                _ex = gtip["gain"][s] * gtip["off"][s] if _ex is None else _ex + gtip["gain"][s] * gtip["off"][s]
                            if hdep is not None and s == "right" and hdep["gain"] > 0.0 and hdep["fill"] > 0.0:      # 19-i: R 손 축 h 에 채운 양
                                _hf = np.array([hdep["gain"] * hdep["fill"], 0.0, 0.0])
                                _ex = _hf if _ex is None else _ex + _hf
                            if carry is not None and s in carry["d"] and carry["gain"][s] > 0.0:
                                tt[s], tip_info[s] = tip.apply(s, target[s], tip_gain, extra=_ex, extra_base=(0.0, 0.0, carry["gain"][s] * carry["d"][s]))
                                _wo, _ = tip.apply(s, target[s], tip_gain, extra=_ex) if _ex is not None else tip.apply(s, target[s], tip_gain)
                                for j in W.JOINT_ORDER:              # 든 몫만 (가상 캔이 보는 엔코더 자세에서 뺀다)
                                    carry["dq"][s][j] = tt[s][j] - _wo[j]
                            elif _ex is not None:
                                tt[s], tip_info[s] = tip.apply(s, target[s], tip_gain, extra=_ex)
                            else:
                                tt[s], tip_info[s] = tip.apply(s, target[s], tip_gain)
                            if pol is not None and s == "left":         # 19-x26 C: L 잡기 보정이 읽는 L 끝 오차 (rule tip_err_max_cm 일 때만 쓴다)
                                pol.tip_err_L = float(tip_info[s].get("err_cm", 0.0))
                            if carry is not None and s in carry["d"] and carry["gain"][s] <= 0.0:
                                for j in W.JOINT_ORDER:
                                    carry["dq"][s][j] = 0.0
                            if can_corr is not None:                 # 18-e 추가: 보정 몫 = 보낼 목표 − 기본 오프셋만 넣은 목표
                                if (_ex is not None and bool(np.any(_ex != 0.0))) or (carry is not None and s in carry["d"] and carry["gain"][s] > 0.0):
                                    _b0, _ = tip.apply(s, target[s], tip_gain)
                                    for j in W.JOINT_ORDER:
                                        can_corr[s][j] = tt[s][j] - _b0[j]
                                else:
                                    for j in W.JOINT_ORDER:
                                        can_corr[s][j] = 0.0
                            for j in W.JOINT_ORDER:
                                tip_dq[s][j] = tt[s][j] - target[s][j]
                        capk = tuple(sorted(f"{s}.{j}" for s in sides for j in tip_info[s]["capped"]))
                        if capk != tip_cap_shown:
                            tip_cap_shown = capk
                            if capk:
                                print(f"\n[집게 끝 오프셋] 관절당 상한 {tip.cap_deg:g}° 에 걸림 (오프셋을 다 못 함): " + ", ".join(capk)
                                      + "   남은 차 " + " ".join(f"{'L' if s == 'left' else 'R'} {tip_info[s]['err_cm']:.1f} cm" for s in sides if (tip.active(s) or (ptip is not None and s in ptip["off"]) or (gtip is not None and s in gtip["off"]) or (carry is not None and s in carry["d"]))))
                            elif seq.stage not in ("lead", "end"):
                                print("\n[집게 끝 오프셋] 상한 걸림 풀림")
                    if sag is not None:
                        want = 1.0 if (motor.position_mode and seq.stage in SAG_STAGES) else 0.0
                        sag_gain += max(-dt / SAG_RAMP_S, min(dt / SAG_RAMP_S, want - sag_gain))
                        raw = {s: sag.command_for(s, tt[s], sag_gain * sag_mult[s]) for s in sides}
                        if psag is not None and "right" in raw:          # 19-x20: R 만 쥔 동안 R 팔꿈치 처짐 보정 양을 K 배로
                            _w6 = 0.0
                            if gripper is not None and seq.stage in ("play", "grip_wait") and getattr(seq, "track", None) is not None:
                                _g6 = seq.track.grip_at(seq.track.tau)
                                if _g6["L"] and _g6["R"]:
                                    psag["ho"] = 1
                                elif psag["ho"] == 1 and not _g6["L"] and _g6["R"]:
                                    psag["ho"] = 2
                                elif not _g6["R"] and not int(gripper.state()["R"]):
                                    psag["ho"] = 0
                                _w6 = 1.0 if psag["ho"] == 2 else 0.0
                            else:
                                psag["ho"] = 0
                            psag["gain"] += max(-dt / PSAG_RAMP_S, min(dt / PSAG_RAMP_S, _w6 - psag["gain"]))
                            _sc = 1.0 - psag["gain"] * (1.0 - psag["k"])
                            _t6 = float(tt["right"]["elbow_pitch"])
                            raw["right"]["elbow_pitch"] = _t6 + _sc * (float(raw["right"]["elbow_pitch"]) - _t6)
                    else:
                        raw = {s: dict(tt[s]) for s in sides}
                    sent = {s: dict(raw[s]) for s in sides}
                    for s in sides:
                        # 14-m A: 보정한 명령을 모터와 같은 관절 한계·비틀기 범위로 먼저 자른다 → 보정은 한계까지만 (남는 처짐은 그대로),
                        # 논리 명령 (도착 판정·VR 표시) = 잘린 뒤 보낸 명령 + 남은 보정량 = 목표 (목표 자체가 한계 밖이면 보정 없을 때와 같이 한계)
                        tg_c = {}
                        for j in W.JOINT_ORDER:
                            lo, hi = motor.urdf_limits[s][j]
                            tg_c[j] = min(max(target[s][j], lo), hi)
                            sent[s][j] = min(max(sent[s][j], lo), hi)
                        if motor.yaw_dyn is not None and s in motor.yaw_dyn:
                            wlo, whi = motor._yaw_window(s, sent[s]["shoulder_pitch"])
                            sent[s]["shoulder_yaw"] = min(max(sent[s]["shoulder_yaw"], wlo), whi)
                            wlo, whi = motor._yaw_window(s, tg_c["shoulder_pitch"])
                            tg_c["shoulder_yaw"] = min(max(tg_c["shoulder_yaw"], wlo), whi)
                        if tip is None:
                            for j in W.JOINT_ORDER:
                                sag_off[s][j] = tg_c[j] - sent[s][j]
                                sag_cut[s][j] = (target[s][j] - raw[s][j]) - (target[s][j] - tg_c[j]) - sag_off[s][j]   # 하려던 보정 − 한 보정
                        else:                                        # 14-w: 오프셋 목표도 같은 한계로 자른 값 (tt_c) 기준으로 나눈다
                            tt_c = {j: min(max(tt[s][j], motor.urdf_limits[s][j][0]), motor.urdf_limits[s][j][1]) for j in W.JOINT_ORDER}
                            if motor.yaw_dyn is not None and s in motor.yaw_dyn:
                                wlo, whi = motor._yaw_window(s, tt_c["shoulder_pitch"])
                                tt_c["shoulder_yaw"] = min(max(tt_c["shoulder_yaw"], wlo), whi)
                            for j in W.JOINT_ORDER:
                                sag_off[s][j] = tg_c[j] - sent[s][j]                                 # VR 표시·도착 판정 = 오프셋 전 논리 목표
                                sag_cut[s][j] = (tt[s][j] - raw[s][j]) - (tt[s][j] - tt_c[j]) - (tt_c[j] - sent[s][j])
                                tip_cut[s][j] = (tt[s][j] - tt_c[j]) - (target[s][j] - tg_c[j])      # 관절 한계로 덜 한 오프셋 몫
                    if tip is not None:
                        tcut = {f"{s}.{j}": math.degrees(abs(v)) for s in sides for j in W.JOINT_ORDER for v in [tip_cut[s][j]] if math.degrees(abs(v)) >= SAG_CUT_PRINT_DEG}
                        tkey = tuple(sorted(tcut))
                        if tkey != tip_cut_shown:
                            tip_cut_shown = tkey
                            if tcut:
                                print("\n[집게 끝 오프셋] 관절 한계로 덜 함: " + ", ".join(f"{k} {v:.1f}°" for k, v in sorted(tcut.items())))
                    cuts = {f"{s}.{j}": math.degrees(abs(v)) for s in sides for j in W.JOINT_ORDER for v in [sag_cut[s][j]] if math.degrees(abs(v)) >= SAG_CUT_PRINT_DEG}
                    key = tuple(sorted(cuts)) if sag is not None else ()
                    if key != sag_cut_shown:
                        sag_cut_shown = key
                        if cuts:
                            print("\n[처짐 보정] 관절 한계로 보정을 덜 함 (남는 처짐은 그대로): "
                                  + ", ".join(f"{k} {v:.1f}°" for k, v in sorted(cuts.items())))
                        elif seq.stage not in ("lead", "end"):
                            print("\n[처짐 보정] 관절 한계 걸림 풀림 — 보정 다 함")
                    target = sent
                if target is not None:
                    motor.update_targets({s: {j: target[s][j] - motor.base_target[s][j] for j in W.JOINT_ORDER} for s in sides}, dt)
                    if not grip_hold["on"]:                           # 18-d·18-e: 붙잡는 동안은 그리퍼 명령을 미룬다
                        seq.apply_grip(gripper)
                if pol is not None:                                               # 15b: 정책 목표 − 실제 명령 (안전장치가 깎은 양, 논리 명령 기준)
                    if pol_target is not None:
                        clip_deg = math.degrees(max(abs(pol_target[s][j] - view.command_target[s][j]) for s in sides for j in W.JOINT_ORDER))
                        clip_hist.append((now, clip_deg > PT_CLIP_DEG))
                        while clip_hist and now - clip_hist[0][0] > PT_CLIP_S:
                            clip_hist.pop(0)
                    else:
                        clip_hist.clear()
                div = motor.send_targets()
                if div is not None:
                    print(f"\n[SAFETY] 발산: {div[0]}.{div[1]} (모터 {div[2]}) 명령 {div[3]:+.3f} 실제 {div[4]:+.3f} → 감쇠")
                if stall is not None:
                    if not motor.position_mode or not meas_ok:
                        stall.reset()
                    elif seq.stage not in ("stopping", "end"):
                        hit = stall.update(now, motor.command_target, motor.last_measured)
                        if hit is not None:
                            seq.abort(now, "걸림 — " + AT.stall_text(hit), motor, meas_ok)
                if robot_viz is not None:
                    robot_viz.tick(now, view, pos_runner, None, W.frame_bones, hold=False, tracking_ok=False, gripper=gripper)
                seq.tick_can(now, view)
                if now - last_log >= 0.05:
                    last_log = now
                    rp = seq.track
                    row = [f"{now:.3f}", "POS" if motor.position_mode else "DAMP/IDLE", str(seq.round + 1), seq.stage,
                           rp.phase, f"{rp.tau:.3f}", str(rp.frame_at(rp.tau))]
                    for s in sides:
                        for j in W.JOINT_ORDER:
                            row += [f"{motor.command_target[s][j]:.4f}", f"{motor.last_measured[s][j]:.4f}"]
                    gs = gripper.state() if gripper is not None else {"L": "", "R": ""}
                    row += [str(gs["L"]), str(gs["R"]),
                            f"{min(motor.table_fn(s, motor.command_target[s])['min'] for s in sides) * 100:.2f}", str(motor.table_blocks),
                            str(motor.gap_blocks), f"{stall.level:.3f}" if stall is not None else "", vr_kind or ("ok" if vr_link else "")]
                    lk = gripper.link if gripper is not None else None
                    if lk is not None and lk.sent_log:
                        tc, cmd = lk.sent_log[-1]
                        row += cmd.split()[1:3] + [f"{tc:.3f}", str(len(lk.sent_log)), str(lk.writes)]
                    else:
                        row += ["", "", "", "0", str(lk.writes) if lk is not None else ""]
                    st = lk.status if lk is not None else None
                    row += ([str(v) for v in st[1:]] + [f"{now - st[0]:.2f}"]) if st else [""] * 7
                    row += [str(lk.resets) if lk is not None else ""]
                    qwd = lk.status_qwd if lk is not None else None
                    msx = lk.status_ms if lk is not None else None
                    row += [str(qwd[0]) if qwd else "", str(msx[0]) if msx else "", str(msx[1]) if msx else "",
                            "1" if seq.stage == "grip_wait" else "0", f"{seq.grip_wait_total + ((now - seq.t_stage) if seq.stage == 'grip_wait' else 0.0):.2f}"]
                    row += [f"{wa.gripper_close_deg[0]:g}", f"{wa.gripper_close_deg[1]:g}"]
                    if sag is not None:
                        row += [f"{sag_gain * max(sag_mult.values()):.3f}", f"{min(table_plain(s, sag.actual(s, motor.command_target[s]))['min'] for s in sides) * 100:.2f}"]
                        row += [f"{sag_off[s][j]:.4f}" for s in sides for j in W.JOINT_ORDER]
                        mx = max(((abs(sag_cut[s][j]), f"{s}.{j}") for s in sides for j in W.JOINT_ORDER))
                        row += [f"{math.degrees(mx[0]):.2f}", mx[1] if math.degrees(mx[0]) >= SAG_CUT_SHOW_DEG else ""]
                    row += [f"{min(park_allow.values()) * 100:.2f}" if relax["allow"] is not None else ""]
                    if pol is not None:
                        row += pol.csv_row() + [f"{clip_deg:.2f}"]
                    if sag is not None and a.sag_gain_arm:
                        row += [f"{sag_gain * sag_mult[s]:.3f}" for s in sides]
                    if pause_on:
                        if hooks.pausing and seq.stage == "grip_wait":
                            pc, pd, _ = pause_text(hooks.pause_hand[0])
                            row += ["".join(hooks.pause_hand), f"{max(0.0, hooks.pause_until - now):.2f}"] + [f"{v:.2f}" for v in pc] + [f"{v:.2f}" for v in pd]
                        else:
                            row += [""] * 8
                    if tip is not None:
                        row += [f"{tip_gain:.3f}"] + [f"{v:.2f}" for s in sides for v in tip_info[s]["moved_hjn"]]
                        row += [f"{math.degrees(tip_info[s]['dq'][j]):.3f}" for s in sides for j in TOF.POS_JOINTS]
                        tmx = max((abs(tip_cut[s][j]), f"{s}.{j}") for s in sides for j in W.JOINT_ORDER)
                        row += [" ".join(f"{s}.{j}" for s in sides for j in tip_info[s]["capped"]), f"{math.degrees(tmx[0]):.2f}",
                                f"{max(tip_info[s]['err_cm'] for s in sides):.2f}"]
                    if ptip is not None:
                        row += [v for s in ptip["off"] for v in (str(ptip["st"][s]), f"{ptip['gain'][s]:.3f}")]
                    if gtip is not None:
                        row += [v for s in gtip["off"] for v in (str(gtip["st"][s]), f"{gtip['gain'][s]:.3f}")]
                        if "arm" in gtip:
                            row += [str(gtip["arm"][s]) for s in gtip["off"]]
                        if "lift" in gtip:
                            row += [v for s in gtip["off"] for v in (f"{gtip['dz'][s]:.2f}", gtip["why"][s])] + ["1" if gtip["gate"] else "0"]
                    if carry is not None:
                        row += [v for s in carry["d"] for v in (str(carry["st"][s]), f"{carry['gain'][s]:.3f}", f"{carry['gain'][s] * carry['d'][s] * tip_gain:.2f}")] + ["1" if carry["gate"] else "0"]
                    if psag is not None:
                        row += [str(psag["ho"]), f"{1.0 - psag['gain'] * (1.0 - psag['k']):.3f}"]
                    if oset is not None:
                        row += [str(oset["st"]), f"{oset['dz']:.2f}" if math.isfinite(oset["dz"]) else "", f"{oset['wait_s']:.2f}", oset["why"], "1" if oset["gate"] else "0"]
                    if hdep is not None:
                        _p4 = hdep["plan"]
                        row += [str(hdep["st"]), f"{_p4['s']:.2f}" if _p4 else "", f"{_p4['want']:.2f}" if _p4 else "", f"{hdep['fill']:.2f}", f"{hdep['gain']:.3f}",
                                f"{_p4['s_after']:.2f}" if _p4 else "", f"{_p4['gap']:.2f}" if _p4 else "", _p4.get("why", "") if _p4 else "", f"{hdep['hold_s']:.2f}",
                                "1" if hdep["gate"] else "0"]
                    if can_corr is not None:
                        row += [f"{math.degrees(max(abs(v) for s in sides for v in can_corr[s].values())):.2f}"]
                    if pend is not None:
                        row += [f"{(now - pend['t0']):.2f}" if pend["t0"] is not None else "", pend["why"]]
                    if plow is not None:
                        row += [plow_info["side"], f"{plow_info['cmd']:.2f}", f"{plow_info['act']:.2f}", str(plow_info["blocked"])]
                    log.writerow(row)
                    log_file.flush()
                if now - last_print >= 0.5:
                    last_print = now
                    left = ""
                    if seq.stage == "place":
                        left = f" 남은 {max(0.0, seq.wait_s - (now - seq.t_stage)):.0f}초"
                    elif seq.stage == "play":
                        left = f" {seq.track.tau:.1f}/{seq.track.duration:.1f}초"
                    elif pause_on and hooks.pausing and seq.stage == "grip_wait":       # 14-v
                        left = (f" ⏸ {'·'.join('왼팔' if h == 'L' else '오른팔' for h in hooks.pause_hand)} 닫힌 뒤 멈춤 남은 {max(0.0, hooks.pause_until - now):.0f}초 | "
                                + pause_text(hooks.pause_hand[0])[2].split(" — ")[1])
                    print(f"[{'POS' if motor.position_mode else 'DAMP'}] {seq.round + 1}/{len(seq.tracks)}회 {seq.stage}{left}   (멈춤: 아무 키나)      ",
                          end="\r", flush=True)
                rate.sleep()
            except KeyboardInterrupt:          # 키 읽기가 없을 때 (터미널 아님) 의 Ctrl+C
                interrupts += 1
                if interrupts >= 2 or not motor.position_mode:
                    print("\n[멈춤] Ctrl+C 두 번 — 바로 힘을 뺀다")
                    break
                flag["stop"] = "Ctrl+C"
    finally:
        flag["quit"] = True
        motor.enter_damping()
        motor.stop()
        if receiver is not None:
            receiver.stop()
        if gripper is not None:
            gripper.close()
            lk = gripper.link
            print(f"\n[그리퍼] 보낸 목표 {len(lk.sent_log)} 번, 직렬에 쓴 것 {lk.writes} 번, 보드 재시작 (READY) {lk.resets} 번"
                  + ("" if lk.status is None else f", 마지막 보드 상태 A {lk.status[1]}/{lk.status[2]}° B {lk.status[4]}/{lk.status[5]}°"))
        log_file.close()
        print(f"\n로그: {log_csv}")
        if pol is not None:
            pol.close()
            print(f"[정책] {pol.summary()}")
    code = seq.exit_code if seq.finished else 0
    if pol is not None and pol_stop is not None and code == 0:
        code = 4
        seq.reason = f"정책 멈춤 ({pol_stop})"
    print(f"[끝] {seq.reason or '다 끝남'} — 종료 코드 {code}")
    return code

if __name__ == "__main__":
    sys.exit(main())
