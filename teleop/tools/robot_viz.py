#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""로봇 그리퍼 위치·한계 상태를 VR(언리얼)로 보내는 송신부 — 워치독 --robot-viz IP:PORT 가 쓴다 (기본 끔).

설계: ~/cc_analysis/REPORT_vr_reach_design_20261021_1707.md §2·§3·§8.2 (1·2단계: 위치 + 한계 색, 3단계 집게 방향은 필드만).
- 읽기만 한다. 명령·로그·그리퍼 경로의 값을 바꾸지 않는다.
- 비차단 UDP. 보내기 실패는 한 번만 알리고 버린다 (CueSound 와 같은 성질) → 제어 루프를 막지 않는다.
- 점은 A 방식 역변환을 마친 '언리얼 원시 추적 좌표 cm' (워치독이 받는 패킷 joints[].pos 와 같은 틀).
  역변환: p_r [m, 로봇 base] → h = (p_r − Ps)/k [cm, 몸 틀] → 어깨(UpperArm 뼈) + h_x·fwd − h_y·right + h_z·up
  (HandTargeter.dirs P:919-921, HumanCalib.to_robot P:467-469 의 역. fwd/right/up = 그 프레임 _frame_cal 결과)
  쌍 매핑 구간 (2026-10-27, REPORT_grip_offset §5 안 2): 위 역변환은 팔마다 자기 어깨 기준 ÷k 라 1:1 로 맞춘 두 끝 간격과
  로봇 어깨 폭(35.2 → 사람 공간 61 cm)을 키워 맞댐에서 두 끝이 약 14 cm 엇갈려 보였다. 그래서 w = pair_w > 0 이면
  두 끝의 가운데 = 두 팔 역변환의 평균, 두 끝 사이 = 1:1 (로봇 cm 그대로) 로 놓는 이동량을 구해 팔 전체(팔꿈치·손목·끝·실제 끝)를
  w 만큼 옮긴다. w = 0 이면 예전과 같다. w 는 연속(쌍 매핑 섞는 비율)이라 표시도 튀지 않는다. 사람 손목 echo·range_* 는 옮기지 않는다.

패킷 v1, 364 바이트, 리틀 엔디언 (언리얼 RobotVizProbe.cpp FPacket 과 같은 순서):
  머리 20: magic "RVZ1", u32 seq, f64 t_mono, u8 state, u8 calib_id, u16 option 비트
  팔 172 × 2 (왼, 오른):
    3f tip_cmd, 3f tip_meas, 3f wrist, 3f elbow           (원시 추적 좌표 cm, 역변환 끝; 없으면 NaN)
    3f axis, 3f jaw                                       (같은 틀 단위 방향)
    3f human_wrist_echo                                   (그 프레임 사람 손목 원시 좌표, 검증용)
    3f tip_robot_cmd, 3f tip_robot_meas                   (로봇 base cm, 검증용)
    f pron_deg, u32 flags, u8 grip, u8 pair_w (쌍 매핑 비율 × 255, 반올림), 2 pad
      flags 비트: 0 관절 한계 잘림, 1 속도 뒤처짐, 2 끝까지 뻗음(손 목표 ≥ rmax − 4 cm), 3 충돌 방지, 4 비틀기 멈춤,
                  5 손 다시 맞춤, 6 등 뒤 넘김, 7 손목 유지, 8 한계 근접, 9 쌍 매핑, 10 너무 가까이 접음(가까운 쪽 거리 자르기),
                  11 인계 예외 켜짐 (워치독 --handover-exempt, 2026-10-30),
                  12 쌍 매핑 중 너무 가까이 (비트 10 대신, 2026-10-30 작업 지시 10-c 안 D),
                  13 가대 면 제한에 걸림 (워치독 --table, 2026-11-02 작업 지시 11-b)
    3f range_origin, 3f range_fwd, 3f range_right, 3f range_up, f range_scale   (사람 어깨 원시 좌표·몸 틀·사람 cm/로봇 cm)

그림용 쌍 매핑 잠금 (작업 지시 16-d 안 B, 워치독 --viz-pair-after-grasp, 기본 끔 = pair_gate None):
  pair_gate (인자 없는 함수, 참 = 가상 캔을 쥔 손이 있음) 가 거짓인 동안 그림에 쓰는 w 를 0 으로 (위 쌍 이동·패킷 pair_w 둘 다).
  바뀌는 순간 튀지 않게 PAIR_GATE_RAMP_S 동안 곧게 바꾼다. 로봇 명령 쪽 쌍 매핑·flags 비트 9·12 (실제 pair_w 기준) 는 그대로다.
  까닭 (REPORT_yellow_20261113_2036.md): 언리얼 실제 크기 집게는 끝을 w 만큼 패킷 끝 (÷k) 쪽으로 옮겨 w × 10.5 cm 앞으로 그리는데
  한 팔 자리 캔 그림은 w = 0 식이라, 첫 잡기 전에 두 손이 가까우면 그림에서 맞춰도 판정 깊이가 3 ~ 5 cm 모자랐다.

--table 을 주면 같은 포트에 1 Hz 짧은 패킷 RVT1 을 따로 보낸다 (364 패킷은 그대로). 형식은 table_limit.py RVT_HDR·RVT_SLOT.
  값은 로봇 base cm 그대로다. 언리얼이 매 틱 팔 패킷의 range_* 로 같은 변환을 해서 판·캔을 그린다 (RobotVizProbe.cpp 머리 주석)."""
from __future__ import annotations

import math
import socket
import struct
from typing import Callable, Dict, Optional, Sequence

import numpy as np

import pos_retarget as PR

MAGIC = b"RVZ1"
HDR = struct.Struct("<4sIdBBH")
ARM = struct.Struct("<" + "3f" * 9 + "fIBB2x" + "3f" * 4 + "f")   # B B = grip, pair_w (0~255, 2026-10-27 Pad[0] 자리)
PACKET_SIZE = HDR.size + 2 * ARM.size          # 364
assert PACKET_SIZE == 364

# 머리 state 비트
ST_POS, ST_DRY, ST_HOLD, ST_TRACK, ST_CALIB = 1, 2, 4, 8, 16
# 머리 option 비트 (u16)
OPT_REAL_SIZE = 1                               # --robot-viz-real-size: 그리는 쪽이 손 길이를 실제 크기(13.3 cm)로

# 팔별 flags (설계 §2.2)
F_JOINT_CLAMP = 1 << 0    # 목표가 관절 한계(또는 비틀기 범위)에 잘림: last_desired 가 한계 경계
F_SPEED_LAG = 1 << 1      # |cmd − desired| > 3° (속도 제한으로 뒤처짐)
F_REACH_FAR = 1 << 2      # 끝까지 뻗음: 손 목표 거리 ≥ rmax − REACH_FAR_TOL_M (임계거리 끝에 닿음, 넘어감 포함 — 2026-10-27:
                          #   팔을 곧게 펴고 보정하면 사람 편 팔 = 로봇 rmax 라 '넘어감' 기준으로는 파랑이 안 떴다)
F_COLLISION = 1 << 3      # 충돌 방지가 막음 (gap_blocks 증가 — 양팔 합 카운터라 양팔 모두 켬)
F_YAW_HOLD = 1 << 4       # 비틀기 범위로 어깨 앞뒤 멈춤 (yaw_holds 증가 — 양팔 합)
F_REFIT = 1 << 5          # 손 다시 맞춤 (refits 증가 + 이 팔의 다시 맞춤 결과가 있음)
F_FLIP_HOLD = 1 << 6      # 등 뒤 넘김 막기 (이 팔 flip_holds 증가)
F_WRIST_HOLD = 1 << 7     # 손목 A 유지 비율 < 1 (팔을 편 동안 손목 고정)
F_NEAR_LIMIT = 1 << 8     # 어느 관절이든 명령이 관절 한계에서 2° 안
F_PAIR = 1 << 9           # 쌍 매핑 w > 0
F_REACH_NEAR = 1 << 10    # 너무 가까이 접음: 손 목표 거리 < rmin − REACH_TOL_M (팔꿈치 굽힘 한계, 2026-10-21 색 규칙 수정)
F_HANDOVER = 1 << 11      # 인계 예외 켜짐 (motor.handover_ext > 0, 워치독 --handover-exempt — 양팔 같이)
F_REACH_NEAR_PAIR = 1 << 12  # 쌍 매핑 중(w > 0) 너무 가까이: 1:1 간격을 지키려 목표를 앞으로 미는 계획된 동작이라 빨강(비트 10)에서 뺐다
                             # (REPORT_handover_color 안 D). 언리얼은 이 비트를 몰라도 같은 패킷의 비트 9(쌍 매핑, 노랑)로 노랑이 된다
F_TABLE = 1 << 13         # 가대 면 제한에 걸림: 모터 쪽이 막았거나 손 목표를 TABLE_LIFT_CM 넘게 올림 (작업 지시 11-b, 12-c 부터 주황)
FLAG_NAMES = ("joint_clamp", "speed_lag", "reach_far", "collision", "yaw_hold", "refit", "flip_hold",
              "wrist_hold", "near_limit", "pair", "reach_near", "handover", "reach_near_pair", "table")
TABLE_LIFT_CM = 0.2
PAIR_GATE_RAMP_S = 0.3    # 16-d: 그림용 쌍 매핑 잠금이 바뀔 때 0 ↔ 1 로 가는 시간 (초)
TABLE_PERIOD = 1.0        # RVT1 주기 (초)
LAG_RAD = math.radians(3.0)
NEAR_RAD = math.radians(2.0)
# 거리 여유 1 cm: 너무 가까이 접음(비트 10)은 rmin − 1 cm 미만 (2026-10-21, 구현 보고서 §4.2).
REACH_TOL_M = 0.01
# 끝까지 뻗음(비트 2)만 4 cm (2026-10-30, 작업 지시 10 §4.1): 사람은 앞으로 뻗을 때 차렷만큼 곧게 펴지 않아 (Meta 추적 굽힘)
# 손 목표가 rmax 의 0.88~0.92 (rmax − 3.5 cm 안팎) 에 머문다. 0110·0038 재생: 앞으로 뻗기 파랑 4.7·7.1% → 60·73%, 평소 0 → 1.1~1.4%, 차렷 ≈ 100%.
REACH_FAR_TOL_M = 0.04
CLAMP_EPS = 1e-9
JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "elbow_roll")
NAN3 = np.full(3, np.nan)


def parse_target(target: str):
    host, _, port = target.partition(":")
    return host, int(port or 5022)


class RobotViz:
    """워치독 메인 루프에서 send_targets() 뒤에 tick() 을 부른다. q_of = 워치독 motor_to_urdf_fn(ref, sides)."""

    def __init__(self, target: str, ref, sides: Sequence[str], q_of: Callable[[str, Dict[str, float]], Dict[str, float]],
                 hz: float = 30.0, real_size: bool = False, jaw_sign: float = 1.0, jaw_offset_deg: float = 0.0,
                 dry_run: bool = False, table=None):
        self.addr = parse_target(target)
        self.table = table                                     # table_limit.TableLimit (없으면 RVT1·비트 13 없음)
        self.table_seq = 0
        self.table_last_t = -math.inf
        self._table_blocks = {s: 0 for s in sides}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.ref, self.sides, self.q_of = ref, tuple(sides), q_of
        self.period = 1.0 / max(float(hz), 1e-3)
        self.real_size, self.dry_run = bool(real_size), bool(dry_run)
        self.jaw_sign, self.jaw_offset = float(jaw_sign), math.radians(float(jaw_offset_deg))
        self.models = {s: PR.ArmModel(s) for s in self.sides}
        for m in self.models.values():
            m.points({j: 0.0 for j in JOINTS})                 # 관절 원점 _O 채우기
        self.seq = 0
        self.sent = 0
        self.error: Optional[str] = None
        self.last_t = -math.inf
        self.calib_id = 0
        self._cal_obj = None
        self._gap_blocks = self._yaw_holds = self._refits = 0
        self._flip = {s: 0 for s in self.sides}
        self.last: Dict[str, dict] = {}                        # csv 용: side → {tip_cmd_cm, tip_meas_cm, flags}
        self.pair_gate: Optional[Callable[[], bool]] = None    # 16-d (None = 끔: 예전과 같다, 머리 주석 "그림용 쌍 매핑 잠금")
        self._gate = 0.0                                       # 지금 잠금 비율 (0 = 그림 w 0, 1 = 실제 w)
        self._gate_t: Optional[float] = None
        self.last_pair_w = 0.0                                 # 마지막으로 보낸 그림 w (확인용)

    # ------------------------------------------------------------------ FK
    def fk(self, side: str, motor_vals: Dict[str, float]):
        """모터값 → (팔꿈치, 손목, 손 축, 집게 끝, 집게 벌어지는 축) — 로봇 base, m.
        ArmModel.points (P:236-248) 와 같은 체인. elbow_roll 은 q_of 가 0 으로 두므로 마지막 프레임 y 축 =
        '손목 URDF 0 일 때의 손목 관절 y 축' = CAD 영점에서 팔꿈치 축 방향 = 집게가 벌어지는 축 (REPORT_54:398)."""
        q = self.q_of(side, motor_vals)
        M = np.eye(4)
        el = None
        for n, O in zip(JOINTS, self.models[side]._O):
            c_, s_ = math.cos(q[n]), math.sin(q[n])
            M = M @ O @ np.array([[c_, -s_, 0, 0], [s_, c_, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]])
            if n == "elbow_pitch":
                el = M[:3, 3].copy()
        wr, ax, jaw0 = M[:3, 3].copy(), M[:3, 2].copy(), M[:3, 1].copy()
        hx = ax if float(np.dot(ax, wr - el)) > 0 else -ax
        return el, wr, hx, wr + PR.ROBOT_GRIP_M * hx, jaw0

    # ------------------------------------------------------------------ 한 번 보내기
    def due(self, now: float) -> bool:
        return now - self.last_t >= self.period

    def tick(self, now: float, motor, pos_runner, frame, frame_bones, *, hold: bool, tracking_ok: bool, gripper=None) -> None:
        """시간이 되었으면 패킷을 만들어 보낸다. 예외는 여기서 삼킨다 (제어 루프 보호)."""
        if self.table is not None and now - self.table_last_t >= TABLE_PERIOD:
            self.table_last_t = now
            self.send_table()
        if not self.due(now):
            return
        # 예정 시각을 누적해 평균 주기를 hz 에 맞춘다 (루프 5 ms 단위로 늦어진 몫을 다음 번에 갚음). 오래 멈췄으면 지금부터
        self.last_t = self.last_t + self.period if now - self.last_t < 2 * self.period else now
        try:
            payload = self.build(now, motor, pos_runner, frame, frame_bones, hold=hold, tracking_ok=tracking_ok, gripper=gripper)
        except Exception as exc:                                # 계산 실패도 루프를 멈추지 않는다
            if self.error != f"build: {exc}":
                self.error = f"build: {exc}"
                print(f"\n🔴 [robot-viz] 패킷 계산 실패: {exc}")
            return
        try:
            self.sock.sendto(payload, self.addr)
            self.sent += 1
        except OSError as exc:
            if self.error != str(exc):
                self.error = str(exc)
                print(f"\n🔴 [robot-viz] 못 보냄 ({self.addr[0]}:{self.addr[1]}): {exc}")

    def send_table(self) -> None:
        """RVT1 한 번 보내기 (머리 주석). 실패는 tick 과 같이 한 번만 알린다."""
        try:
            self.table_seq += 1
            m = self.models
            ps = {s: (m[s].shoulder_center() if s in m else np.full(3, float("nan"))) for s in ("left", "right")}
            self.sock.sendto(self.table.rvt_packet(self.table_seq, ps["left"], ps["right"]), self.addr)
        except Exception as exc:
            if self.error != str(exc):
                self.error = str(exc)
                print(f"\n🔴 [robot-viz] RVT1 못 보냄: {exc}")

    def build(self, now, motor, pos_runner, frame, frame_bones, *, hold, tracking_ok, gripper=None) -> bytes:
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        ht = getattr(pos_runner, "ht", None)
        cal_obj = getattr(pos_runner, "cal", None)
        if cal_obj is not None and cal_obj is not self._cal_obj:  # 차렷 보정마다 새 HumanCalib
            self._cal_obj = cal_obj
            self.calib_id = (self.calib_id + 1) & 0xFF
        state = ((ST_POS if motor.position_mode else 0) | (ST_DRY if self.dry_run else 0) | (ST_HOLD if hold else 0)
                 | (ST_TRACK if tracking_ok else 0) | (ST_CALIB if ht is not None else 0))
        opts = OPT_REAL_SIZE if self.real_size else 0
        parts = [HDR.pack(MAGIC, self.seq, float(now), state, self.calib_id, opts)]

        common = 0
        if motor.gap_blocks > self._gap_blocks:
            common |= F_COLLISION
        if motor.yaw_holds > self._yaw_holds:
            common |= F_YAW_HOLD
        self._gap_blocks, self._yaw_holds = motor.gap_blocks, motor.yaw_holds
        refit_new = pos_runner.refits > self._refits
        self._refits = pos_runner.refits
        hl = ht.last if ht is not None else {}
        pair_w = hl.get("pair_w", 0.0) or 0.0
        bones = frame_bones(frame) if frame is not None else {}
        gst = gripper.state() if gripper is not None else {}
        meas_ok = (not self.dry_run) and motor.position_mode

        arms: Dict[str, dict] = {}
        for side in ("left", "right"):
            if side not in self.sides:
                continue
            flags = common | (F_PAIR if pair_w > 0 else 0) | (F_HANDOVER if getattr(motor, "handover_ext", 0.0) > 0.0 else 0)
            cmd = motor.command_target[side]
            # 관절 한계 잘림 / 속도 뒤처짐 / 한계 근접
            ywin = motor._yaw_window(side, cmd["shoulder_pitch"]) if getattr(motor, "yaw_dyn", None) and side in motor.yaw_dyn else None
            for j in JOINTS:
                lo, hi = motor.urdf_limits[side][j]
                d = motor.last_desired[side][j]
                if abs(d - lo) < CLAMP_EPS or abs(d - hi) < CLAMP_EPS:
                    flags |= F_JOINT_CLAMP
                if j == "shoulder_yaw" and ywin is not None and (abs(d - ywin[0]) < CLAMP_EPS or abs(d - ywin[1]) < CLAMP_EPS):
                    flags |= F_JOINT_CLAMP
                if abs(cmd[j] - d) > LAG_RAD:
                    flags |= F_SPEED_LAG
                if cmd[j] - lo < NEAR_RAD or hi - cmd[j] < NEAR_RAD:
                    flags |= F_NEAR_LIMIT
            # 손 목표 거리 자르기 (ht.last["{side}_target"] = 자르기 전 목표, P:994-996)
            if ht is not None and f"{side}_target" in hl:
                dist = float(np.linalg.norm(np.asarray(hl[f"{side}_target"]) - ht.Ps[side]))
                ext = ht._ext.get(side, 0.0)
                rmin, rmax = (ht._eff_geom(side, ext)[1:] if ext else (ht.rmin[side], ht.rmax[side]))
                if dist >= rmax - REACH_FAR_TOL_M:
                    flags |= F_REACH_FAR
                elif dist < rmin - REACH_TOL_M:
                    flags |= F_REACH_NEAR_PAIR if pair_w > 0 else F_REACH_NEAR
            if self.table is not None:
                blocks = motor.table_blocks_by.get(side, 0)
                if blocks > self._table_blocks[side] or hl.get(f"{side}_table_lift_cm", 0.0) > TABLE_LIFT_CM:
                    flags |= F_TABLE
                self._table_blocks[side] = blocks
            rt = pos_runner.rts.get(side) if getattr(pos_runner, "rts", None) else None
            if refit_new and getattr(pos_runner, "_refit", {}).get(side) is not None:
                flags |= F_REFIT
            if rt is not None:
                if rt.flip_holds > self._flip[side]:
                    flags |= F_FLIP_HOLD
                self._flip[side] = rt.flip_holds
                if rt.wrist_hold and rt.el_prev is not None:
                    lo_w = max(PR.WRIST_HOLD_LO, rt.elbow_zero() + PR.WRIST_HOLD_BAND)
                    if (rt.el_prev - lo_w) / PR.WRIST_HOLD_BAND < 1.0:
                        flags |= F_WRIST_HOLD
            # 위치 (명령·실제)
            el, wr, hx, tip, jaw0 = self.fk(side, cmd)
            tip_m = self.fk(side, motor.last_measured[side])[3] if meas_ok else None
            dpron = cmd["elbow_roll"] - self.ref.e90[side]["elbow_roll"]
            pron_deg = math.degrees(PR.WRIST_MOTOR_SIGN[side] * dpron)
            a = self.jaw_sign * dpron + self.jaw_offset             # 집게 축을 손 축 둘레로 (3단계 보정값은 옵션)
            jaw = jaw0 * math.cos(a) + np.cross(hx, jaw0) * math.sin(a)
            U, _, Wb = PR.BONES[side]
            cal = rt.cal if rt is not None else None
            if ht is not None and cal is not None and U in bones and Wb in bones:
                sh = np.asarray(bones[U][0], float)
                k, Ps = ht.k[side], ht.Ps[side]

                def P(p, sh=sh, k=k, Ps=Ps, cal=cal):          # 기본 인자로 이 팔 값을 묶는다 (아래 쌍 이동에서 팔마다 다시 부름)
                    h = (p - Ps) / k
                    return sh + h[0] * cal.fwd - h[1] * cal.right + h[2] * cal.up

                def Dv(d, cal=cal):
                    return d[0] * cal.fwd - d[1] * cal.right + d[2] * cal.up

                v_tip, v_wr, v_el = P(tip), P(wr), P(el)
                v_tm = P(tip_m) if tip_m is not None else NAN3
                v_ax, v_jaw = Dv(hx), Dv(jaw)
                echo = np.asarray(bones[Wb][0], float)
                rng = (sh, np.asarray(cal.fwd, float), np.asarray(cal.right, float), np.asarray(cal.up, float), 1.0 / (100.0 * k))
            else:
                P = Dv = None
                v_tip = v_wr = v_el = v_tm = v_ax = v_jaw = echo = NAN3
                rng = (NAN3, NAN3, NAN3, NAN3, float("nan"))
            tr = tip * 100.0
            trm = tip_m * 100.0 if tip_m is not None else NAN3
            grip = int(gst.get("L" if side == "left" else "R", 0) or 0)
            arms[side] = {"pts": [v_tip, v_tm, v_wr, v_el], "dirs": [v_ax, v_jaw], "echo": echo, "tr": tr, "trm": trm,
                          "pron": pron_deg, "flags": flags, "grip": grip, "rng": rng, "tip": tip, "P": P, "Dv": Dv}
            self.last[side] = {"tip_cmd_cm": tr, "tip_meas_cm": trm, "flags": flags}
        # 쌍 매핑 구간: 두 끝 가운데 = 두 팔 역변환 평균, 사이 = 1:1. 팔 전체를 w 만큼 옮긴다 (모듈 머리 주석)
        w = min(max(float(pair_w), 0.0), 1.0) if math.isfinite(float(pair_w)) else 0.0
        if self.pair_gate is not None:                         # 16-d (켤 때만)
            want = 1.0 if self.pair_gate() else 0.0
            dt = 0.0 if self._gate_t is None else max(float(now) - self._gate_t, 0.0)
            self._gate_t = float(now)
            step = dt / PAIR_GATE_RAMP_S
            self._gate = min(self._gate + step, want) if want > self._gate else max(self._gate - step, want)
            w *= self._gate
        self.last_pair_w = w
        if w > 0 and all(s in arms and arms[s]["P"] is not None for s in ("left", "right")):
            c = 0.5 * (arms["left"]["tip"] + arms["right"]["tip"])
            mid = 0.5 * (arms["left"]["P"](c) + arms["right"]["P"](c))
            for a in arms.values():
                shift = w * (mid + a["Dv"]((a["tip"] - c) * 100.0) - a["pts"][0])
                a["pts"] = [p + shift for p in a["pts"]]
        w8 = int(round(w * 255))
        for side in ("left", "right"):
            a = arms.get(side)
            if a is None:
                parts.append(ARM.pack(*([float("nan")] * 27), float("nan"), 0, 0, 0, *([float("nan")] * 13)))
                continue
            rng = a["rng"]
            parts.append(ARM.pack(*a["pts"][0], *a["pts"][1], *a["pts"][2], *a["pts"][3], *a["dirs"][0], *a["dirs"][1],
                                  *a["echo"], *a["tr"], *a["trm"], a["pron"], a["flags"], a["grip"], w8,
                                  *rng[0], *rng[1], *rng[2], *rng[3], rng[4]))
        return b"".join(parts)


def unpack(payload: bytes) -> dict:
    """시험·기록용: 패킷 → dict."""
    if len(payload) != PACKET_SIZE or payload[:4] != MAGIC:
        raise ValueError(f"패킷 아님 ({len(payload)} 바이트)")
    m, seq, t, st, cid, opt = HDR.unpack_from(payload, 0)
    out = {"seq": seq, "t_mono": t, "state": st, "calib_id": cid, "options": opt}
    names = ["tip_cmd", "tip_meas", "wrist", "elbow", "axis", "jaw", "human_wrist_echo", "tip_robot_cmd", "tip_robot_meas"]
    for i, side in enumerate(("left", "right")):
        v = ARM.unpack_from(payload, HDR.size + i * ARM.size)
        d = {n: v[3 * k:3 * k + 3] for k, n in enumerate(names)}
        d["pron_deg"], d["flags"], d["grip"], d["pair_w"] = v[27], v[28], v[29], v[30] / 255.0
        r = v[31:]
        d["range_origin"], d["range_fwd"], d["range_right"], d["range_up"], d["range_scale"] = r[0:3], r[3:6], r[6:9], r[9:12], r[12]
        out[side] = d
    return out
