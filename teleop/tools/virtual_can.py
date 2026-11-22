#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VR 가상 캔 상태와 에피소드 기록 (작업 지시 13, 워치독 --virtual-can·--episodes, 기본 끔).

읽기만 한다: 모터 명령(command_target)과 그리퍼 판단(GripperController.state)을 읽어 캔 상태를 계산한다. 명령·그리퍼·364 패킷 값을 바꾸지 않는다.
틀: 로봇 base, cm (x 앞, y 왼, z 위). 캔 위치 = 밑면 가운데, 자세 = 캔 틀 → base 쿼터니언 (캔 틀 z = 캔 축, 밑면 → 윗면). 서 있으면 (0, 0, 0, 1).
그리퍼 틀: 원점 = 집게 끝, h = 손 축 (손목 → 끝), j = 집게가 벌어지는 축, n = h × j. 끝에서 손목 쪽 거리 s = −(p − 끝)·h.
  손 축·벌어지는 축은 robot_viz.py fk·build 와 같은 식이다 (VR 에 그려지는 집게와 같은 방향).

상태: on_table (자리 위에 섬), held_L, held_R, held_both, fallen.
잡힘: 그 팔 그리퍼가 열림 → 닫힘이 되는 순간에만 잰다. 캔 축 위 점들을 그리퍼 틀로 옮겨, 아래를 모두 만족하는 점을 찾는다 (여유 m = --virtual-can-margin-cm):
  |j| ≤ min(캔 반지름 + m, 열림 반간격)   집게 축 방향: 캔 축이 두 집게 사이
  −m ≤ s ≤ 집게 길이 + m                  손 축 방향: 집게 길이 안
  |n| ≤ 집게 반폭 + m                     셋째 축 방향: 캔 축이 집게 면을 지남
  여러 점이면 |n| 이 가장 작은 점. 그 점의 캔 밑면에서 높이 t 가 곧은 구간 (table.json straight_section_above_plate) 안이어야 한다.
  집게 축과 캔 수평면 사이 각 ≤ GRIP_TILT_MAX_DEG 이어야 한다.
  붙는 순간의 그리퍼↔캔 상대 자세를 저장해 그대로 따라간다. 넘어진 캔은 n 으로 다시 세울 때까지 잡지 않는다.
인계: 한 팔에 잡힌 채 다른 팔이 닫히면 같은 조건에 잡는 높이만 양쪽으로 --virtual-can-handover-extra-cm (기본 집게 반폭 0.85: 집게 폭 일부라도 곧은 구간에 닿으면) 넓혀 재고, 되면 held_both.
  (작업 지시 13-b, dry_vcan_0753: 먼저 잡은 손이 캔 가운데를 차지해 둘째 손은 위·아래 끝으로 가게 된다. 42.03 초 오른손 축 어긋남 0.3 cm 인데 높이 14.3 > 14.1 로 막혀 떨어뜨렸다.) 캔은 먼저 잡은 팔을 계속 따라간다. 먼저 잡은 팔이 열리면 다른 팔에 지금 자세 그대로 다시 붙인다
  (튀지 않게. 두 팔이 벌어진 양은 slip_cm 으로 알린다). 이것만 인계로 센다. 나중에 잡은 팔이 열리면 먼저 잡은 팔에 그대로 남고 인계가 아니다.
놓기 (작업 지시 13-c): 잡은 팔이 모두 열리는 순간 밑면이 판 안이고 (a) 밑면 가운데 판 위 높이 ≤ place_cm (--virtual-can-place-cm, 기본 PLACE_HEIGHT_CM),
  (b) 기울기 ≤ PLACE_TILT_DEG, (c) 놓기 직전 PLACE_SPEED_WINDOW_S 동안 캔 밑면 평균 속도 ≤ PLACE_SPEED_MS 이면 판 위로 내려 바로 선다. 아니면 fallen (캔 가운데를 판 안으로 넣어 눕힌다).
부딪힘: 아무도 잡지 않은 캔의 축 (반지름만큼) 이 집게·링크·몸체 상자와 겹치면 bump. 표시만 하고 캔은 움직이지 않는다.
  놓은 직후·새 에피소드 직후에 이미 겹쳐 있던 것은 세지 않는다 (한 번 떨어진 뒤부터 센다).
판 접촉 (작업 지시 13-d): 잡힌 캔 밑면 테두리의 가장 낮은 점이 판 윗면에 닿거나 파고들면 캔을 똑바로 세워 (축 수직, 밑면 가운데를 판 위에)
  잡은 팔마다 상대 자세를 새로 잡는다. 그 상태에서 놓으면 선다. 손을 들면 새 상대 자세로 따라간다. 닿기 시작할 때 1번 센다 (CONTACT_LEAVE_CM 넘게 떨어지면 다시 셈).
놓기 가능 (작업 지시 13-d): 잡힌 동안 매 걸음 "지금 모든 손을 놓으면 선다" 를 놓기와 같은 판정으로 잰다 (RVC1 place 비트 1).
목표 허용 (작업 지시 13-d): 목표 자리와 밑면 수평 거리 ≤ goal_tol_cm (--virtual-can-goal-tol-cm, 기본 GOAL_TOL_CM) 이면 자동 성공 (RVC1 place 비트 2 = 지금 범위 안).
판 뚫기 (작업 지시 13-c): 잡힌 캔의 가장 낮은 점 (밑면·윗면 테두리) 이 판 윗면 아래로 내려가면 RVC1 에는 그 깊이만큼 올려 판 위에 멈춘 것으로 보낸다.
  잡은 그리퍼와의 상대 자세·csv 의 can_x/y/z·그리퍼 명령은 그대로다. 에피소드에 들어간 횟수 (PEN_ENTER_CM 넘게 들어갈 때 1, 판 위로 나오면 다시 셈) 와 최대 깊이를 남긴다.

RVC1 패킷 (같은 포트, 30 Hz, 60 바이트 — 13-d 에 56 에서 늘림, 리틀 엔디언, RobotVizProbe.cpp FCanPacket 과 같은 순서):
  4s "RVC1", u32 seq, 3f 밑면 (base cm), 4f 쿼터니언 (x, y, z, w), 3f 기준 팔 집게 끝 (base cm, 잡혔을 때만, 없으면 NaN),
  u8 상태 (위 차례 0~4), u8 flags (1 에피소드 중, 2 bump, 4 왼팔 지금 닫으면 잡힘, 8 오른팔 지금 닫으면 잡힘, 위 4비트 = 목표 자리 번호 1~15, 0 없음 — 13-c), u8 기준 팔 (0 왼, 1 오른, 255 없음), u8 자리 (시작 자리에 그대로 서 있으면 번호 1~, 아니면 0),
  u16 에피소드 번호 (마지막 번호), u16 이 에피소드 bump 수,
  (13-d) u8 place (1 지금 다 놓으면 섬, 2 목표 허용 범위 안), u8 마지막 놓기 (0 아직 없음, 16 섬, 넘어짐이면 이유 비트 1 높이 2 기울기 4 속도 8 판 밖),
  u16 목표와 밑면 수평 거리 mm (목표 없음 0xFFFF).
미리 보기 (작업 지시 13-b): 매 걸음 팔마다 "지금 닫으면 잡힌다" 를 grip_check 로 잰다. 열린 팔, 잡고 있지 않은 팔, 서 있거나 다른 팔이 잡은 캔일 때만 참. 다른 팔이 잡았으면 인계 조건으로.
에피소드 파일 <log 이름>_episodes.csv: EPISODE_HEADER. 결과 auto_success (목표 자리) / fallen / timeout / manual_success (s) / manual_fail (f) / manual_abort (진행 중 n) / abort (종료). 다시 세운 시각까지 알아야 해서 다시 세울 때 (또는 종료 때) 쓴다.
키보드 없는 흐름 (작업 지시 13-b, reset_s 를 주면. 워치독 --episodes): 위치 모드가 되면 첫 에피소드 시작 → 성공·넘어짐·시간 초과 (timeout_s) 로 끝 → reset_s 뒤 시작 자리에 다시 세우고 새 에피소드.
  다시 세울 자리에 그리퍼가 겹치면 풀릴 때까지 기다린다. 다시 세우기를 기다리는 동안은 잡히지 않는다. 시간은 위치 모드·hold 아닐 때만 흐른다.
조합 순환 (작업 지시 16-a, combos 를 주면 — 워치독 --can-combos, 기본 끔): 에피소드가 끝날 때마다 (결과와 상관없이) 다음 (시작, 목표) 조합으로 바꾼다 (차례로 돈다).
  다시 세울 때 새 시작 자리에 세우고, RVC1 목표 자리 비트·자리 번호가 바로 바뀐다 (VR 은 목표 원·캔 위치로 안다). 바꿀 때 "ready" 소리 (switch 는 가상 그리퍼 "다 열림" 소리라 피함).
  에피소드 파일 끝에 COMBO_HEADER (그 에피소드의 시작·목표 좌표, 조합 번호, 그 조합 성공·전체 수) 를 더한다.
엄격 판정 (작업 지시 16-a, strict — 워치독 --grasp-strict, 기본 끔): 잡기·인계·놓기 품질을 재서 화면·소리로 알리고 에피소드 파일 끝 STRICT_HEADER 에 남긴다.
  결과 (result) 는 바꾸지 않는다. strict_fail = 1 이면 학습 데이터에서 뺄 후보 (demo_export 가 meta quality.strict 로 옮긴다).
  1 닫기 직전 열린 날·링크·몸체 ↔ 캔 몸통 (overlap 과 같은 상자) 최소 거리 < −STRICT_OPEN_CM (들어감) → 실패
  2 닫는 순간 집게 가운데 − 캔 축 (수직 성분) 의 손 축 성분이 −STRICT_DEPTH_CM 보다 작음 (덜 들어감), 닫히는 축 성분 |j| > STRICT_SIDE_CM → 실패
  3 두 손이 함께 쥔 동안 두 그리퍼 (날·링크·서보 몸체, 날은 캔을 쥔 간격) 최소 간격 < STRICT_GAP_CM → 경고 (상자 모델이 커서 실기 성공판도 0~0.4 cm 로 나온다, 16-a)
  4 판에 놓아 선 캔 밑면 − 목표 자리 수평 > STRICT_GOAL_CM → 경고 (성공 판정은 goal_tol 그대로)
노란 원 엄격 (작업 지시 16-d 안 C, ready_strict — 워치독 --grasp-strict 가 켠다, 기본 끔): 미리 보기 (RVC1 flags 4·8) 도
  grip_check 에 더해 엄격 2 (집게 가운데 덜 들어감 ≤ STRICT_DEPTH_CM, 닫히는 축 비킴 ≤ STRICT_SIDE_CM) 를 만족할 때만 켠다.
  잡힘 판정·엄격 판정·기록은 그대로다 (노란 원이 떠도 strict 실패가 나던 것을 없앤다, REPORT_yellow_20261113_2036.md §2)."""
from __future__ import annotations

import csv
import math
import socket
import struct
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

import pos_retarget as PR

RVC_MAGIC = b"RVC1"
RVC = struct.Struct("<4sI" + "3f" + "4f" + "3f" + "BBBB" + "HH" + "BBH")
PACKET_SIZE = RVC.size
assert PACKET_SIZE == 60
RVC_PERIOD = 1.0 / 30.0
FLAG_ACTIVE, FLAG_BUMP, FLAG_READY_L, FLAG_READY_R = 1, 2, 4, 8
GOAL_SHIFT = 4             # flags 위 4비트 = 목표 자리 번호 (작업 지시 13-c)
PLACE_OK, IN_GOAL = 1, 2   # place 바이트 (13-d)
WHY_HEIGHT, WHY_TILT, WHY_SPEED, WHY_OFF, WHY_STOOD = 1, 2, 4, 8, 16   # 마지막 놓기 바이트 (13-d)
NO_DIST = 0xFFFF
NO_ARM = 255

ON_TABLE, HELD_L, HELD_R, HELD_BOTH, FALLEN = range(5)
STATE_NAMES = ("on_table", "held_L", "held_R", "held_both", "fallen")

# 근거는 RobotVizProbe.cpp 머리 주석 "형태" 와 같다 (REPORT_54:387·398·445)
JAW_LEN_CM = 5.5           # 집게 길이
JAW_HALF_OPEN_CM = 4.2     # 열린 안쪽 반간격
JAW_HALF_CLOSED_CM = 0.5   # 닫힌 반간격
JAW_THICK_CM = 0.8         # 집게 하나 두께 (벌어지는 축)
JAW_HALF_WIDTH_CM = 0.85   # 집게 반폭 (셋째 축, 닫힌 두 끝 전체 폭 1.7)
GRIP_M = PR.ROBOT_GRIP_M   # 손목 관절 → 집게 끝
GRIP_MARGIN_CM = 1.0       # 판정 여유 (옵션). 반지름 + 1.0 = 3.9 < 열림 4.2, 사람 손 VR 위치 맞춤 오차를 받는다
# 지름 5.75 인 캔을 비스듬히 물 때 집게 사이 필요 폭 = 5.75 / cosθ. 열림 8.4 를 넘지 않는 θ = 46.8° → 45°
GRIP_TILT_MAX_DEG = 45.0
# 놓기 (작업 지시 13-c §1, 13 의 2 cm·30° 를 바꿈): 빈 캔은 3~4 cm 에서 똑바로 놓으면 대부분 선다 (사용자 실물 관찰)
PLACE_HEIGHT_CM = 4.0      # 밑면 가운데의 판 위 높이 (판 아래는 몇 cm 든 선다)
PLACE_TILT_DEG = 20.0      # 캔 축 기울기. 이 캔이 혼자 넘어지는 각 atan(반지름 2.875 / 무게중심 높이 7.8) = 20.2° 와 맞다 (13-d §5).
                           #   판에 닿으면 바로 세우므로 (머리 주석 "판 접촉") 판에 내려놓는 경우에는 걸리지 않는다
PLACE_SPEED_MS = 0.3       # 놓기 직전 캔 밑면 평균 속도 (m/s): 휘두르며 놓으면 넘어진다
PLACE_SPEED_WINDOW_S = 0.2
PEN_ENTER_CM = 0.1         # 판 뚫기를 셀 때 이만큼 넘게 들어가야 1번 (경계에서 깜빡여 수가 부풀지 않게)
GOAL_TOL_CM = 5.0          # 목표 허용 범위 (작업 지시 13-d, 13 의 2 cm 는 4~5 번에 1 번 성공이라 빡빡했다)
CONTACT_LEAVE_CM = 0.2     # 판 접촉에서 이만큼 넘게 떨어져야 다음 접촉을 센다
BUMP_REARM_S = 0.3         # 떨어진 뒤 이만큼 지나야 다음 bump 로 센다 (경계에서 깜빡여 수가 부풀지 않게)
BUMP_NEAR_CM = 35.0        # 집게 끝이 캔 가운데에서 이보다 멀면 겹침 계산을 건너뛴다 (몸체 끝 13.3 + 캔 대각선 절반 8.3 + 여유)
AXIS_EXTRA_CM = 3.0        # 캔 끝 너머 이만큼까지 재서 "잡는 높이가 곧은 구간 밖" 을 "손이 캔에 안 닿음" 과 나눈다
MISS_PRINT_CM = 15.0       # 닫혔는데 못 잡았을 때 이 거리 안이면 이유를 화면에 알린다
MISS_PRINT_S = 1.0         # 그 알림을 팔마다 이 간격보다 자주 찍지 않는다
# 작업 지시 14-l 잡기 정렬 (--grasp-align, 기본 끔): 집게 셋째 축 (닫히는 면의 수직) ↔ 캔 축 각, 둘째 손은 집게 가운데 ↔ 캔 축 거리도.
#   14-k: 11-03 성공 11 개의 둘째 손 각 3~12° (중앙 7), 거리 0.3~1.9 cm (중앙 1.0) → 10°·1.5 cm 면 9 개 통과. 먼저 잡는 손 각 10~20° (중앙 15) → 15° 면 6 개.
# 작업 지시 14-s (실측 calib_T/T_20261112_225314): 먼저 잡는 손은 집게 축 (j) 이 캔 축에 직각에서 어긋난 각 ≤ ALIGN_DEG_FIRST 로 바꾼다.
#   손을 숙여 낮게 잡는 것 (셋째 축 ↔ 캔 28°, 아래팔이 판에 붙을 수 없어 구조상 필요) 은 실제로 잘 잡히므로 막지 않고 상한 ALIGN_N_MAX_DEG 만 둔다.
#   실측 T1 j ⟂ 3°·n 28°, T2 j ⟂ 3°·n 5°. 14-q 실기 성공 L 닫기 j ⟂ 3~12°·n 19~27°.
#   둘째 손은 그대로 셋째 축 ↔ "L 이 쥔 가상 캔의 축" (L 의 숙임을 따라 기운 축) ≤ ALIGN_DEG_SECOND, 거리만 1.5 → 2.0 cm (실측 T4 1.2°·1.7 cm 가 통과하게).
ALIGN_DEG_FIRST = 15.0
ALIGN_DEG_SECOND = 10.0
ALIGN_AXIS_CM = 2.0
ALIGN_N_MAX_DEG = 45.0     # 먼저 잡는 손 셋째 축 ↔ 캔 축 상한 (숙임)
PUSH_REARM_S = BUMP_REARM_S       # 14-s 둘째 손 밀기 알림을 다시 세기까지
GRIP_CENTER_CM = 1.0       # 집게 가운데 = 끝에서 손 축 뒤로 이만큼 (table.json 자리 tip_past_axis_cm 1.0 과 같다, 14-k 분석과 같은 점)
HANDOVER_HEIGHT_EXTRA_CM = JAW_HALF_WIDTH_CM   # 둘째 손 잡는 높이 여유: 집게 가운데가 곧은 구간 밖이어도 폭 일부가 닿는 만큼 (머리 주석 "인계")
EPISODE_RESET_S = 3.0      # 작업 지시 13-b §2
EPISODE_TIMEOUT_S = 90.0   # 작업 지시 13-b §2
RELEASE_CLEAR_CM = 3.0     # 작업 지시 14-h --episode-end release: 두 집게 끝이 캔 겉면에서 이만큼 넘게
RELEASE_CLEAR_S = 0.5      #   떨어진 채 이만큼 이어져야 성공 (그리퍼는 둘 다 열림, 캔은 목표에 서 있음)
EPISODE_ENDS = ("place", "release")
# 겹침 상자 (s 범위, j 반폭, n 범위). 링크·몸체 치수는 table_limit.py BODY_REAL·_LINK 와 같은 값 (서보 쪽 6.1, 반대쪽 3.0, 좌우 반폭 3.0)
LINK_BOX = (5.5, 6.8, 2.25, -1.5, 1.5)
BODY_BOX = (6.8, 13.3, 3.0, -3.0, 6.1)
JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "elbow_roll")
EPISODE_HEADER = ["episode", "t_mono_start", "t_mono_end", "start_slot", "goal_slot", "result", "grip_order", "handover", "bump_count", "fallen", "t_mono_reset",
                  "place_h_cm", "place_tilt_deg", "place_speed_ms", "place_result", "penetration", "penetration_max_cm",
                  "place_why", "goal_dist_cm", "contact"]
CSV_COLUMNS = ["can_x", "can_y", "can_z", "can_qx", "can_qy", "can_qz", "can_qw", "can_state", "episode"]
CSV_COLUMNS_RELEASE = ["episode_phase", "tip_can_cm"]     # 작업 지시 14-h --episode-end release 일 때만 뒤에
HAND_KEY = {"left": "L", "right": "R"}
# 작업 지시 16-a (기본 끔)
COMBO_HEADER = ["start_x_cm", "start_y_cm", "goal_x_cm", "goal_y_cm", "combo_index", "combo_ok", "combo_n"]
# 작업 지시 19-b 좌표 쌍 (pairs, 기본 None = 끔): 자리 목록에 움직이는 자리 두 개를 실행 때 더하고 (파일은 그대로), 에피소드마다 그 xy 를 갈아 끼운다.
#   기록은 COMBO_HEADER 와 같은 칸 (start_slot = S0, goal_slot = G0, combo_index = 쌍 번호). VR 그림 패킷 (RVT1) 은 자리 목록을 1 초마다 다시 보내므로 따라온다.
PAIR_START, PAIR_GOAL = "S0", "G0"
# 손 축 (VR 실제 크기 집게 그림만 쓴다): 돌림 · 숙임을 좌표의 일차식으로. (기준 x, 기준 y, 그 자리 돌림°, 숙임°, 돌림 좌우 °/cm, 돌림 앞뒤 °/cm, 숙임 좌우 °/cm, 숙임 앞뒤 °/cm)
#   기준값 · 좌우 기울기 = 실기 로그의 L1 · L2 잡는 순간 / R1 · R2 놓는 순간 손 축 (논리 명령) 을 지나게. 앞뒤 기울기 = 범위 지도 해 (19-a, 같은 잡는 높이끼리)
HAND_AXIS_MODEL = {"left": (29.0, 13.0, -17.6, -15.1, 2.7125, 0.90, 0.125, 0.06), "right": (29.0, -14.0, 18.8, 3.9, 1.975, -2.65, 0.30, -0.20)}


def auto_hand_axis(arm: str, x: float, y: float) -> List[float]:
    """19-b: 자리 좌표 (base cm) → 그 자리에서의 손 축 (단위 벡터). 돌림 ±60°, 숙임 ±30° 로 자른다."""
    x0, y0, yaw0, pit0, ay, ax, py, px = HAND_AXIS_MODEL[arm]
    yaw = math.radians(max(-60.0, min(60.0, yaw0 + ay * (y - y0) + ax * (x - x0))))
    pit = math.radians(max(-30.0, min(30.0, pit0 + py * (y - y0) + px * (x - x0))))
    return [round(math.cos(pit) * math.cos(yaw), 4), round(math.cos(pit) * math.sin(yaw), 4), round(math.sin(pit), 4)]


def load_pairs(path) -> List[Tuple[float, float, float, float]]:
    """19-b: 좌표 쌍 파일. 한 줄에 시작 x, y, 도착 x, y (cm, base). 쉼표나 빈칸으로 나눈다. # 뒤는 설명, 빈 줄은 건너뛴다."""
    out = []
    for n, line in enumerate(Path(path).expanduser().read_text(encoding="utf-8").splitlines(), 1):
        t = line.split("#", 1)[0].replace(",", " ").split()
        if not t:
            continue
        if len(t) != 4:
            raise ValueError(f"{path} {n} 째 줄: 값이 4 개여야 한다 (시작 x y, 도착 x y) — {line.strip()!r}")
        try:
            out.append(tuple(float(v) for v in t))
        except ValueError:
            raise ValueError(f"{path} {n} 째 줄: 숫자가 아니다 — {line.strip()!r}")
    if not out:
        raise ValueError(f"{path}: 좌표 쌍이 없다")
    return out
STRICT_OPEN_CM = 0.3       # 1: 열린 날·링크·몸체가 캔 몸통에 이만큼 넘게 들어가면 실패 (실기: 안쪽 날이 캔을 밀었다, 14-v)
STRICT_DEPTH_CM = 1.0      # 2: 집게 가운데가 캔 축보다 손 축으로 이만큼 넘게 덜 들어가면 실패 (실기: R 이 날 끝으로 물어 미끄러짐, 14-aa)
STRICT_SIDE_CM = 1.0       # 2: 닫히는 축으로 이만큼 넘게 비키면 실패 (실기: L 바깥 1.4~2.8 cm, 한쪽 날이 먼저 닿음, 14-v)
STRICT_GAP_CM = 1.5        # 3: 두 그리퍼 최소 간격 (실기: R 이 L 서보를 긁음, 14-ab). 경고만 — 몸체 상자 (BODY_BOX) 가 실물보다 커서 실기 성공 3 판도 0.0~0.14 cm (16-a)
STRICT_GOAL_CM = 1.5       # 4: 경고만
STRICT_HEADER = ["strict_open_cm", "strict_L_depth_cm", "strict_L_side_cm", "strict_R_depth_cm", "strict_R_side_cm", "strict_gap_cm",
                 "strict_goal_cm", "strict_flags", "strict_fail"]
STRICT_GAP_PERIOD_S = 0.05
# 작업 지시 19-x 위쪽 잡기 (grasp_mode, 기본 "bottom" = 지금 그대로). top: L 이 서 있는 캔의 위쪽을 먼저 잡고 R 이 아래쪽으로 받는다 (19-w 계산).
#   잡는 높이 (캔 밑면에서 집게 가운데, 곧은 구간 0.6 ~ 14.1 안): L (먼저 손) 10 ~ 14, R (둘째 손) 3 ~ 5. 둘째 손 높이 여유 (handover_extra) 는 쓰지 않는다.
#   위쪽 잡기에서 R 이 서 있는 캔을 먼저 잡거나 L 이 둘째 손이 되면 "순서" 로 못 잡는다. VR 표시: 판 머리 권장 높이 (RVT1 GripH, 서 있는 캔의 띠) 를 12 로,
#   시작 · 목표 자리의 잡는 높이 두 값 (RVT1 GripH1 · GripH2) 을 L 띠 10 · 14, R 띠 3 · 5 로 (메모리의 자리 목록만, table.json 은 그대로).
#   19-x3: 이 VR 띠 바꾸기는 top_band 를 켤 때만 (기본 끔). 언리얼 (실제 크기 그림, 기본) 은 판을 머리 GripH × (K − Unit) 만큼 들고, 자리 원은
#   그 자리 GripH 평균으로 그려 둘이 같아야 원이 판 면에 붙는다. 머리 12 · 목표 3 · 5 (평균 4) 면 목표 원이 판보다 약 (4 − 12) × 0.74 ≈ 6 cm 아래 (dry_vcan_2017).
GRASP_MODES = ("bottom", "top")
TOP_FIRST_T_CM = (10.0, 14.0)
TOP_SECOND_T_CM = (3.0, 5.0)
TOP_BAND_HDR_CM = 12.0
TOP_LIVE_AXIS_NEAR_CM = 15.0   # 19-x6: top 에서 L 끝이 캔 겉면에서 이 안이면 시작 자리 손 축 (VR 실제 크기 그림 보정, RVT1) 을 지금 L 손 축으로
#   까닭: 언리얼은 판 위 캔을 그 자리 손 축 (auto_hand_axis = 밑 잡기 모델, 숙임 −15°) 으로 실제 크기 보정해 그린다 (RobotVizProbe.cpp 889 ~ 892).
#   top 의 L 은 거의 수평이고 덜 돌아가 있어, 그림이 집게 축으로 0.8 ~ 1.6 cm 비켜 보였다 (dry_vcan_2046 · 2206, 예상 ↔ 실제 비킴 기울기 0.99, 잔차 0.25 cm).
#   판정 · 로봇 명령은 그대로다 (그림 패킷의 손 축만). RVT1 은 1 초마다라 그림도 1 초마다 따라온다.
TOP_ALIGN_SECOND_DEG = 32.0   # 잡기 정렬 (--grasp-align) 을 켤 때 둘째 손 셋째 축 ↔ 쥔 캔 축 한도 (bottom 은 10°). 19-x2: 25 → 32 (아래 R 기울기 범위 위 끝)
# 19-x2 인계 각도 (top · real 일 때만 판정): 위를 잡는 손 (L) 은 곧게, 아래를 잡는 손 (R) 은 비스듬히. 근거 = 지금 방식 (L 이 아래) 실물 성공 각도:
#   아래 손 숙임 19 ~ 27° (11-12 원격 조종 성공 3 회), 그 손이 쥔 캔 기울기 28 ~ 32° (19-e 옛 시연 평균 · 19-j 실기 L1R1), 두 손 축 사이 97 ~ 109° (19-e).
#   R 기울기 = R 셋째 축 (닫힌 면의 수직) ↔ 쥔 캔 축 각 = 캔을 곧게 세웠을 때 R 손이 숙는 각. 18 ~ 32° (위 두 범위의 합), 15° 미만은 "인계 수평" (실물: 두 손 수평이면 놓을 때 불안정).
# 19-x4: 위쪽 잡기 실물 텔레옵 (hand_table_1920 · 1926 · 1939 · 1941, 엔코더, 인계 6 번) 으로 다시 정함:
#   두 손 축 사이 87.9 ~ 99.3° (지금 방식 97 ~ 109 와 다름) → 85 ~ 105. R 셋째 축 ↔ L 셋째 축 (캔 축 ≈ L 셋째 축) 15.4 ~ 21.1° → R 기울기 하한 18 → 15 (6 번 중 5 번이 18 아래).
#   누름: 19-v 실기 — 넘어진 판의 논리 누름 최소 1.11 cm, ×5 성공 판 전부 ≤ 0.86 cm → top 은 1.0 cm. 판에 닿는 순간 삑 (twist) 으로 알림 (VR 은 닿은 캔을 판 위에 멈춰 그려 손이 더 내려가는 것이 안 보인다).
TOP_REAL_TILT_DEG = (15.0, 32.0)
TOP_REAL_FLAT_DEG = 15.0
TOP_REAL_AXES_DEG = (85.0, 105.0)
TOP_REAL_PRESS_CM = 1.0
# 작업 지시 19-x 실물 기준 판정 (real, 기본 끔 — 워치독 --grasp-real, 수집 때 켬). 결과 (result) 는 바꾸지 않고 real_fail 로 남긴다 (엄격 판정과 같은 꼴).
#   깊이: R 이 L 이 쥔 캔을 받는 순간, 캔을 L 집게 축 가운데로 옮긴 자세 (19-h 가정 B — 닫히면 캔이 가운데로 밀림) 에서 R 둘째 손 판정 s (캔 축이 R 끝에서 손 축 뒤로 몇 cm)
#     ≥ REAL_DEPTH_LOGIC_CM. 엔코더 상당 3.0 cm = 논리 1.70 + Δ 1.30 (19-j delta.py: 실기 L1R1 8 판 Δ 1.30 ± 0.08, 19-j 깊이 검사 하한과 같은 값).
#   비킴: 닫는 순간 집게 가운데 ↔ 캔 축의 닫히는 축 성분 |j| ≤ REAL_SIDE_CM (L · R, R 은 가정 B 자세).
#   누름: 잡힌 캔이 판에 닿은 뒤 손이 더 내려간 양 (판 접촉으로 바로 세울 때 매 걸음 판 아래로 들어간 깊이의 합, 판에서 떨어지면 0) > REAL_PRESS_CM → 실패 (19-v: 눌러 놓으면 넘어짐).
#   부딪힘: 이 에피소드에서 아직 아무도 잡지 않았을 때 bump (집게 · 링크 · 몸체 상자가 캔에 겹침) → 실패 (19-v: 잡기 전 접촉).
REAL_DEPTH_LOGIC_CM = 1.70
REAL_SIDE_CM = 1.0
REAL_PRESS_CM = 0.5
REAL_SAY = {"depth": "인계 얕음", "side": "비킴", "press": "누름", "bump": "부딪힘", "flat": "인계 수평", "angle": "인계 각도"}       # cue_player 가 "say:문구" 를 말한다 (서버 PC 변경 없음)
REAL_HEADER = ["real_depth_cm", "real_L_side_cm", "real_R_side_cm", "real_press_cm", "real_bump", "real_tilt_deg", "real_axes_deg", "real_flags", "real_fail"]
# 작업 지시 19-x 시작 자세 (start_gate, 기본 None = 끔): 에피소드를 시작하려면 두 집게가 닫히고 두 집게 끝이 판 위 START_GATE_H_CM 이상,
#   시작 캔 축에서 수평으로 (반지름 + START_GATE_CLEAR_CM) 넘게 떨어진 채 START_GATE_HOLD_S 이어져야 한다 (19-v B3: 시연 시작 자세 통일).
START_GATE_H_CM = 12.0
START_GATE_CLEAR_CM = 3.0
START_GATE_HOLD_S = 0.5
START_GATE_SAY_S = 5.0          # 19-x2: 기다리는 동안 이 간격으로 안내 음성, 통과하면 "시작"
START_GATE_SAY = "두 손 집게 닫고 위로"
GATE_HEADER = ["start_gate_wait_s"]

Hand = Tuple[np.ndarray, np.ndarray]    # (집게 끝 cm, 3×3 열 = h, j, n)


def quat_of(R: np.ndarray) -> np.ndarray:
    """회전 행렬 → 쿼터니언 (x, y, z, w)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = 2.0 * math.sqrt(1.0 + t)
        q = ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    elif R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
        s = 2.0 * math.sqrt(max(1.0 + R[0, 0] - R[1, 1] - R[2, 2], 1e-12))
        q = (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    elif R[1, 1] >= R[2, 2]:
        s = 2.0 * math.sqrt(max(1.0 + R[1, 1] - R[0, 0] - R[2, 2], 1e-12))
        q = ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    else:
        s = 2.0 * math.sqrt(max(1.0 + R[2, 2] - R[0, 0] - R[1, 1], 1e-12))
        q = ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)
    return np.array(q, float)


def frame_with_z(z: np.ndarray) -> np.ndarray:
    """캔 축 z 를 셋째 열로 하는 회전 행렬 (눕힌 캔용. 축 둘레 각은 뜻이 없다)."""
    z = z / np.linalg.norm(z)
    a = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = np.cross(a, z)
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


class VirtualCan:
    """table = table_limit.TableLimit, start·goal = 자리 이름 (L1 …). tick() 을 제어 루프에서 부른다 (예외를 밖으로 내지 않는다)."""

    def __init__(self, table, start: str, ref, sides: Sequence[str], q_of: Callable[[str, Dict[str, float]], Dict[str, float]], *,
                 goal: Optional[str] = None, margin_cm: float = GRIP_MARGIN_CM, jaw_sign: float = 1.0, jaw_offset_deg: float = 0.0,
                 servo_sign: Optional[Dict[str, float]] = None, target: Optional[str] = None, episodes_path=None,
                 sound: Optional[Callable[[str], None]] = None, log: Callable[[str], None] = print,
                 handover_extra_cm: float = HANDOVER_HEIGHT_EXTRA_CM, reset_s: Optional[float] = None, timeout_s: float = EPISODE_TIMEOUT_S,
                 place_cm: float = PLACE_HEIGHT_CM, place_tilt_deg: float = PLACE_TILT_DEG, place_speed_ms: float = PLACE_SPEED_MS,
                 goal_tol_cm: float = GOAL_TOL_CM, upright_on_contact: bool = True, episode_end: str = "place",
                 align: Optional[Tuple[float, float, float]] = None, combos: Optional[Sequence[Tuple[str, str]]] = None, strict: bool = False,
                 ready_strict: bool = False, pairs: Optional[Sequence[Tuple[float, float, float, float]]] = None,
                 grasp_mode: str = "bottom", real: bool = False, real_depth_cm: float = REAL_DEPTH_LOGIC_CM, start_gate: Optional[float] = None,
                 top_band: bool = False, top_live_axis: bool = True):
        self.table, self.ref, self.sides, self.q_of = table, ref, tuple(sides), q_of
        if grasp_mode not in GRASP_MODES:                       # 19-x (기본 bottom = 지금 그대로)
            raise ValueError(f"grasp_mode 는 {GRASP_MODES} 중 하나")
        self.grasp_mode = grasp_mode
        self.top_live_axis = grasp_mode == "top" and bool(top_live_axis)   # 19-x6 (top 만, bottom 은 그대로)
        self.top_band = bool(top_band)                          # 19-x3: top 의 VR 띠 높이 바꾸기 (기본 끔 — 목표 원이 판 속에 묻혔다)
        self.real = bool(real)                                  # 19-x 실물 기준 판정 (False = 끔)
        self.real_depth = float(real_depth_cm)
        self.start_gate = None if start_gate is None else float(start_gate)   # 19-x 시작 자세 높이 cm (None = 끔)
        self._press = 0.0                                       # 19-x 누름 (잡힌 캔이 판에 닿은 뒤 손이 더 내려간 양)
        self._grip_j: Dict[str, float] = {}
        self._hands: Dict[str, "Hand"] = {}
        self._gate_since: Optional[float] = None
        self._gate_wait_t0: Optional[float] = None
        self._gate_said = False
        self.pairs = None                                       # 작업 지시 19-b 좌표 쌍 (None = 끔)
        if pairs:
            if combos:
                raise ValueError("좌표 쌍 (pairs) 과 조합 (combos) 은 같이 쓰지 않는다")
            self.pairs = [tuple(float(v) for v in p) for p in pairs]
            self._pair_check()
            have = {s["name"] for s in table.cfg["can_slots"]}
            for name, arm in ((PAIR_START, "left"), (PAIR_GOAL, "right")):      # 메모리의 자리 목록에만 더한다 (table.json 은 그대로)
                if name not in have:
                    table.cfg["can_slots"].append({"name": name, "kind": "arm", "arm": arm, "base_xy_cm": [0.0, 0.0], "can_bottom_cm": 0.0, "grip_h_cm": [8],
                                                   "tip_past_axis_cm": GRIP_CENTER_CM, "hand_axis": [1.0, 0.0, 0.0], "note": "19-b 움직이는 자리 (에피소드마다 xy 가 바뀐다)"})
            start, goal = PAIR_START, PAIR_GOAL
        cfg = table.cfg["objects"]["can"]
        self.radius, self.height = 0.5 * float(cfg["diameter"]), float(cfg["height"])
        self.straight = tuple(float(v) for v in cfg["straight_section_above_plate"])
        self.slots = {s["name"]: i for i, s in enumerate(table.cfg["can_slots"])}
        self.start_name = self._slot_name(start)
        self.goal_name = self._slot_name(goal) if goal else None
        if self.goal_name and float(table.cfg["can_slots"][self.slots[self.goal_name]].get("can_bottom_cm", 0.0)) > 0.0:
            raise ValueError(f"목표 자리 {self.goal_name} 는 공중 자리라 캔이 설 수 없다")
        # 작업 지시 16-a 조합 순환 (None = 끔: 고정 자리)
        self.combos = None
        if combos:
            self.combos = [(self._slot_name(a), self._slot_name(b)) for a, b in combos]
            for _, g in self.combos:
                if float(table.cfg["can_slots"][self.slots[g]].get("can_bottom_cm", 0.0)) > 0.0:
                    raise ValueError(f"목표 자리 {g} 는 공중 자리라 캔이 설 수 없다")
            self.combo_i = 0
            self.start_name, self.goal_name = self.combos[0]
            self.combo_stat = {f"{a}:{b}": [0, 0] for a, b in self.combos}          # [성공, 전체]
        if self.pairs is not None:                              # 19-b: 첫 쌍의 좌표를 자리에 넣는다 (reset 이 그 자리에 캔을 세운다)
            self.pair_i = 0
            self.pair_stat = [[0, 0] for _ in self.pairs]       # [성공, 전체]
            self._pair_set(0)
        self.strict = bool(strict)                              # 작업 지시 16-a (False = 끔)
        self.ready_strict = bool(ready_strict)                  # 작업 지시 16-d 안 C (False = 끔: 노란 원은 grip_check 만)
        self._strict_gap_t = -math.inf
        self.margin = float(margin_cm)
        self.handover_extra = float(handover_extra_cm)
        self.place_cm, self.place_tilt, self.place_speed = float(place_cm), float(place_tilt_deg), float(place_speed_ms)
        self._hist: List[Tuple[float, np.ndarray]] = []         # 잡힌 동안 (시각, 밑면) — 놓기 속도
        self.pen_depth = 0.0                                   # 지금 판 아래로 들어간 깊이 (잡힌 동안만, 표시용)
        self._pen_in = False
        self.last_place: Optional[dict] = None
        self.goal_tol = float(goal_tol_cm)
        self.upright = bool(upright_on_contact)                 # False 는 재생 대조용 (13-c 까지의 동작)
        self._contact_in = True                                # 서 있는 캔을 잡는 순간은 접촉으로 세지 않는다
        self.place_ok = False
        self.in_goal = False
        # 작업 지시 14-h: 에피소드 끝 정의. place = 놓아서 목표에 선 순간 (13-d 그대로), release = 그 뒤 손을 빼야 (RELEASE_CLEAR_*)
        if episode_end not in EPISODE_ENDS:
            raise ValueError(f"episode_end 는 {EPISODE_ENDS} 중 하나")
        self.episode_end = episode_end
        # 작업 지시 14-l: (먼저 잡는 손 각°, 둘째 손 각°, 둘째 손 축 거리 cm). None 이면 끔 (판정·출력이 13~14-k 와 같다)
        self.align = None if align is None else tuple(float(v) for v in align)
        self.align_miss = 0                                    # 정렬 조건만으로 못 잡은 닫기 수 (에피소드와 무관하게 합)
        self.phase = ""                     # release 만: "" (나르는 중) / "놓기 대기" (쥔 채 목표 안) / "손 빼기" (놓음, 손이 아직 가까움)
        self._clear_since = None
        self.tip_clear_cm = math.nan
        self.reset_s = None if reset_s is None else float(reset_s)   # None 이면 키로만 (머리 주석 "키보드 없는 흐름")
        self.timeout_s = float(timeout_s)
        self.jaw_sign, self.jaw_offset = float(jaw_sign), math.radians(float(jaw_offset_deg))
        self.servo_sign = {s: float((servo_sign or {}).get(s, 1.0)) for s in self.sides}
        self.models = {s: PR.ArmModel(s) for s in self.sides}
        for m in self.models.values():
            m.points({j: 0.0 for j in JOINTS})                 # 관절 원점 _O 채우기
        self.sound = sound or (lambda name: None)
        self.log = log
        self._axis_t = np.arange(-AXIS_EXTRA_CM, self.height + AXIS_EXTRA_CM + 1e-6, 0.25).tolist()
        self._bump_t = np.linspace(0.0, self.height, 17)
        # 상태
        self.state = ON_TABLE
        self.holders: List[str] = []                           # 잡은 차례 (첫째가 기준 팔)
        self.rel: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        self.pos = np.zeros(3)
        self.rot = np.eye(3)
        self.q = np.array([0.0, 0.0, 0.0, 1.0])
        self.at_slot = 0
        self.closed = {s: False for s in self.sides}
        self.ready = {s: False for s in self.sides}            # 지금 닫으면 잡힌다 (머리 주석 "미리 보기")
        self._miss_print_t = {s: -math.inf for s in self.sides}
        self.anchor_tip: Optional[np.ndarray] = None
        self.bumping = {s: False for s in self.sides}
        self._bump_clear_t = {s: -math.inf for s in self.sides}
        self._bump_skip = {s: False for s in self.sides}       # 놓은 직후처럼 이미 겹쳐 있던 것: 떨어질 때까지 세지 않는다
        self.bump_total = 0
        self.events: List[tuple] = []
        self.error: Optional[str] = None
        # 에피소드
        self.episode = 0                                       # 마지막 번호
        self.active = False
        self.ep: dict = {}
        self.results: List[dict] = []
        self._row: Optional[dict] = None                       # 끝났지만 다시 세운 시각을 기다리는 행
        self._clock = 0.0                                      # 흐름 시간 (위치 모드·hold 아닐 때만 흐른다)
        self._clock_last: Optional[float] = None
        self._ep_clock0 = 0.0
        self._reset_due: Optional[float] = 0.0 if self.reset_s is not None else None   # 첫 에피소드는 바로
        self._reset_waiting = False
        self._ep_file = None
        self._ep_writer = None
        self.episodes_path = Path(episodes_path) if episodes_path else None
        if self.episodes_path is not None:
            self._ep_file = open(self.episodes_path, "w", newline="")
            self._ep_writer = csv.writer(self._ep_file)
            self._ep_writer.writerow(self._ep_header())
            self._ep_file.flush()
        # 패킷
        self.addr = None
        self.sock = None
        if target:
            host, _, port = target.partition(":")
            self.addr = (host, int(port or 5022))
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setblocking(False)
        self.seq = 0
        self.sent = 0
        self.last_send = -math.inf
        self.reset()

    # ------------------------------------------------------------------ 자리
    def _pair_check(self) -> None:
        """19-b: 좌표 쌍이 판 안에 있는지 (캔이 다 들어오게 반지름만큼 안쪽)."""
        t, r = self.table, 0.5 * float(self.table.cfg["objects"]["can"]["diameter"])
        for n, (sx, sy, gx, gy) in enumerate(self.pairs, 1):
            for what, x, y in (("시작", sx, sy), ("도착", gx, gy)):
                if not (t.x0 * 100.0 + r <= x <= t.x1 * 100.0 - r and t.y0 * 100.0 + r <= y <= t.y1 * 100.0 - r):
                    raise ValueError(f"좌표 쌍 {n} 번 {what} ({x:g}, {y:g}) 가 판 밖이다 (캔 반지름 {r:g} cm 안쪽이어야 한다)")

    def _pair_set(self, i: int) -> None:
        """19-b: 움직이는 자리 두 개의 xy · 손 축을 i 번 쌍으로 (메모리의 자리 목록). VR 그림 패킷은 다음 1 초 주기에 따라온다."""
        sx, sy, gx, gy = self.pairs[i]
        for name, arm, x, y in ((PAIR_START, "left", sx, sy), (PAIR_GOAL, "right", gx, gy)):
            s = self.table.cfg["can_slots"][self.slots[name]]
            s["base_xy_cm"] = [x, y]
            s["hand_axis"] = auto_hand_axis(arm, x, y)

    def _slot_name(self, name: str) -> str:
        n = str(name).upper()
        if n not in self.slots:
            raise ValueError(f"자리 {name} 없음 (있는 자리: {', '.join(self.slots)})")
        return n

    def _slot_base(self, name: str) -> np.ndarray:
        s = self.table.cfg["can_slots"][self.slots[name]]
        x, y = float(s["base_xy_cm"][0]), float(s["base_xy_cm"][1])
        return np.array([x, y, self.table.top(x / 100.0) * 100.0 + float(s.get("can_bottom_cm", 0.0))])

    def _top_display(self) -> None:
        """19-x 위쪽 잡기: VR 잡는 높이 띠 (RVT1). 메모리의 판 설정 · 자리 목록만 바꾼다 (table.json 은 그대로)."""
        self.table.cfg["recommended_grip_h_cm"] = TOP_BAND_HDR_CM
        self.table.cfg["can_slots"][self.slots[self.start_name]]["grip_h_cm"] = list(TOP_FIRST_T_CM)
        if self.goal_name:
            self.table.cfg["can_slots"][self.slots[self.goal_name]]["grip_h_cm"] = list(TOP_SECOND_T_CM)

    def reset(self) -> None:
        """시작 자리에 다시 세운다. 그리퍼가 닫혀 있어도 다음 닫힘까지 붙지 않는다."""
        if self.grasp_mode == "top" and self.top_band:          # 19-x3: 띠 바꾸기는 따로 켤 때만 (RVT1 머리 GripH 가 언리얼 판 높이 기준이라 목표 원이 묻힘)
            self._top_display()
        self.state = ON_TABLE
        self.holders, self.rel = [], {}
        self.pos = self._slot_base(self.start_name)
        self.rot = np.eye(3)
        self.q = np.array([0.0, 0.0, 0.0, 1.0])
        self.at_slot = self.slots[self.start_name] + 1
        self.anchor_tip = None
        self._hist = []
        self.pen_depth, self._pen_in = 0.0, False
        self._contact_in = True
        self._press = 0.0                                      # 19-x 누름 (새로 세운 캔)
        self._skip_bumps()

    def _skip_bumps(self) -> None:
        self._bump_skip = {s: True for s in self.sides}
        self.bumping = {s: False for s in self.sides}

    # ------------------------------------------------------------------ 그리퍼 자세
    def hand_of(self, side: str, cmd: Dict[str, float]) -> Hand:
        """모터 명령 → 그리퍼 틀. robot_viz.py fk·build 와 같은 체인·같은 집게 축."""
        q = self.q_of(side, cmd)
        M = np.eye(4)
        el = None
        for n, O in zip(JOINTS, self.models[side]._O):
            c_, s_ = math.cos(q[n]), math.sin(q[n])
            M = M @ O @ np.array([[c_, -s_, 0, 0], [s_, c_, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]])
            if n == "elbow_pitch":
                el = M[:3, 3].copy()
        wr, ax, jaw0 = M[:3, 3].copy(), M[:3, 2].copy(), M[:3, 1].copy()
        hx = ax if float(np.dot(ax, wr - el)) > 0 else -ax
        a = self.jaw_sign * (cmd["elbow_roll"] - self.ref.e90[side]["elbow_roll"]) + self.jaw_offset
        jaw = jaw0 * math.cos(a) + np.cross(hx, jaw0) * math.sin(a)
        return (wr + GRIP_M * hx) * 100.0, np.column_stack((hx, jaw, np.cross(hx, jaw)))

    # ------------------------------------------------------------------ 판정
    def grip_check(self, hand: Hand, height_extra: float = 0.0, second: bool = False, side: Optional[str] = None) -> dict:
        """이 그리퍼가 지금 닫히면 캔을 잡는가. height_extra = 잡는 높이 범위를 양쪽으로 넓히는 양 (인계). 반환 ok, reason (ok 면 ""), t (밑면에서 잡는 높이), j, s, n, tilt_deg.
        정렬 (self.align, 14-l) 이 켜져 있으면 나머지가 다 맞을 때 더 본다: align_deg (셋째 축 ↔ 캔 축), axis_cm (집게 가운데 ↔ 캔 축), align_lim (각 한도).
        second = 다른 팔이 이미 잡고 있음 (둘째 손 한도·거리 조건)."""
        tip, R = hand
        z = self.rot[:, 2]
        tilt = math.degrees(math.asin(min(abs(float(np.dot(R[:, 1], z))), 1.0)))
        m = self.margin
        d = self.pos - tip
        # 캔 축 위 높이 t 에서 s = s0 + t·ds, j = j0 + t·dj, n = n0 + t·dn. 세 범위를 모두 만족하는 t 구간을 푼다
        s0, ds = -float(d @ R[:, 0]), -float(z @ R[:, 0])
        j0, dj = float(d @ R[:, 1]), float(z @ R[:, 1])
        n0, dn = float(d @ R[:, 2]), float(z @ R[:, 2])
        j_max, n_max = min(self.radius + m, JAW_HALF_OPEN_CM), JAW_HALF_WIDTH_CM + m
        limits = (("손 축", s0, ds, -m, JAW_LEN_CM + m), ("집게 축", j0, dj, -j_max, j_max), ("셋째 축", n0, dn, -n_max, n_max))
        lo, hi = -AXIS_EXTRA_CM, self.height + AXIS_EXTRA_CM
        for _, a, b, v0, v1 in limits:
            if abs(b) < 1e-9:
                if not v0 <= a <= v1:
                    lo, hi = 1.0, 0.0
            else:
                t0, t1 = sorted(((v0 - a) / b, (v1 - a) / b))
                lo, hi = max(lo, t0), min(hi, t1)
        if lo <= hi:                                           # |n| 이 가장 작은 점, 손 축이 캔 축과 나란하면 (n 이 그대로) 집게 가운데
            target = -n0 / dn if abs(dn) > 1e-3 else (0.5 * JAW_LEN_CM - s0) / ds if abs(ds) > 1e-9 else 0.5 * self.height
            t = min(max(target, lo), hi)
            reason = ""
        else:                                                  # 못 잡는 이유: 가장 많이 벗어난 축 (캔 축 위에서 벗어남 합이 가장 작은 점 기준)
            over = lambda a, b, v0, v1, x: max(v0 - (a + b * x), (a + b * x) - v1, 0.0)
            t = min(self._axis_t, key=lambda x: sum(over(a, b, v0, v1, x) for _, a, b, v0, v1 in limits))
            reason = max(limits, key=lambda c: over(c[1], c[2], c[3], c[4], t))[0]
        out = {"t": float(t), "j": j0 + t * dj, "s": s0 + t * ds, "n": n0 + t * dn, "tilt_deg": tilt, "ok": False, "reason": reason}
        if tilt > GRIP_TILT_MAX_DEG:
            out["reason"] = "집게 기울기"
        elif not reason and self.grasp_mode == "top":          # 19-x 위쪽 잡기 (켤 때만): 손마다 높이 범위, L 먼저
            w0, w1 = TOP_SECOND_T_CM if second else TOP_FIRST_T_CM
            if side is not None and (side == "left") == bool(second):
                out["reason"] = "순서"
            elif w0 - 1e-6 <= t <= w1 + 1e-6:
                out["ok"] = True
            else:
                out["reason"] = "잡는 높이"
        elif not reason:
            if self.straight[0] - height_extra - 1e-6 <= t <= self.straight[1] + height_extra + 1e-6:
                out["ok"] = True
            else:
                out["reason"] = "잡는 높이"
        if self.align is not None and out["ok"]:
            a_deg = math.degrees(math.acos(min(abs(float(np.dot(R[:, 2], z))), 1.0)))
            v = (tip - GRIP_CENTER_CM * R[:, 0]) - self.pos
            axis_cm = float(np.linalg.norm(v - float(np.dot(v, z)) * z))
            lim = self.align[1] if second else self.align[0]
            if second and self.grasp_mode == "top":           # 19-x: R 이 캔을 비스듬히 받아야 놓을 때 선다 (19-w: 손 숙임 15 ~ 17°)
                lim = max(lim, TOP_ALIGN_SECOND_DEG)
            out.update(align_deg=a_deg, axis_cm=axis_cm, align_lim=lim, second=bool(second))
            if second:                                         # 둘째 손: 셋째 축 ↔ 쥔 캔 축 (self.rot 은 먼저 잡은 손을 따라 기운다)
                if a_deg > lim:
                    out["ok"], out["reason"] = False, "정렬 각"
                elif axis_cm > self.align[2]:
                    out["ok"], out["reason"] = False, "정렬 거리"
            elif tilt > lim:                                   # 14-s 먼저 잡는 손: 집게 축 ⟂ 캔 축 어긋남
                out["ok"], out["reason"] = False, "집게 축 정렬"
            elif a_deg > ALIGN_N_MAX_DEG:                      # 숙임은 상한만
                out["ok"], out["reason"] = False, "숙임"
        return out

    def overlap(self, side: str, hand: Hand, closed: bool) -> bool:
        """아무도 잡지 않은 캔과 집게·링크·몸체 상자가 겹치는가."""
        tip, R = hand
        z = self.rot[:, 2]
        centre = self.pos + 0.5 * self.height * z
        if float(np.linalg.norm(centre - tip)) > BUMP_NEAR_CM:
            return False
        rel = (self.pos - tip)[None, :] + self._bump_t[:, None] * z[None, :]
        s, j, n = -(rel @ R[:, 0]), rel @ R[:, 1], rel @ R[:, 2]
        half = JAW_HALF_CLOSED_CM if closed else JAW_HALF_OPEN_CM
        sign = self.servo_sign[side]
        boxes = [(0.0, JAW_LEN_CM, half, half + JAW_THICK_CM, -JAW_HALF_WIDTH_CM, JAW_HALF_WIDTH_CM),
                 (0.0, JAW_LEN_CM, -half - JAW_THICK_CM, -half, -JAW_HALF_WIDTH_CM, JAW_HALF_WIDTH_CM)]
        for s0, s1, jw, n0, n1 in (LINK_BOX, BODY_BOX):
            lo, hi = (n0, n1) if sign > 0 else (-n1, -n0)
            boxes.append((s0, s1, -jw, jw, lo, hi))
        r2 = self.radius * self.radius
        for s0, s1, j0, j1, n0, n1 in boxes:
            d2 = (np.maximum(np.maximum(s0 - s, s - s1), 0.0) ** 2 + np.maximum(np.maximum(j0 - j, j - j1), 0.0) ** 2
                  + np.maximum(np.maximum(n0 - n, n - n1), 0.0) ** 2)
            if float(d2.min()) <= r2:
                return True
        return False

    # ------------------------------------------------------------------ 한 걸음
    # ------------------------------------------------------------------ 엄격 판정 (16-a, strict 일 때만 부른다)
    def _hand_boxes(self, side: str, half: float):
        """overlap 과 같은 상자 (그리퍼 틀: s 끝에서 손목 쪽, j 닫히는 축, n 셋째 축). half = 날 안쪽 반간격."""
        boxes = [(0.0, JAW_LEN_CM, half, half + JAW_THICK_CM, -JAW_HALF_WIDTH_CM, JAW_HALF_WIDTH_CM),
                 (0.0, JAW_LEN_CM, -half - JAW_THICK_CM, -half, -JAW_HALF_WIDTH_CM, JAW_HALF_WIDTH_CM)]
        sign = self.servo_sign[side]
        for s0, s1, jw, n0, n1 in (LINK_BOX, BODY_BOX):
            lo, hi = (n0, n1) if sign > 0 else (-n1, -n0)
            boxes.append((s0, s1, -jw, jw, lo, hi))
        return boxes

    @staticmethod
    def _box_dist(s, j, n, box):
        s0, s1, j0, j1, n0, n1 = box
        return np.sqrt(np.maximum(np.maximum(s0 - s, s - s1), 0.0) ** 2 + np.maximum(np.maximum(j0 - j, j - j1), 0.0) ** 2
                       + np.maximum(np.maximum(n0 - n, n - n1), 0.0) ** 2)

    def strict_open_cm(self, side: str, hand: Hand) -> float:
        """1: 열린 날 둘·링크·몸체와 캔 몸통 (축 둘레 반지름) 최소 거리 cm. 음수 = 들어간 깊이."""
        tip, R = hand
        z = self.rot[:, 2]
        rel = (self.pos - tip)[None, :] + self._bump_t[:, None] * z[None, :]
        s, j, n = -(rel @ R[:, 0]), rel @ R[:, 1], rel @ R[:, 2]
        return float(min(self._box_dist(s, j, n, b).min() for b in self._hand_boxes(side, JAW_HALF_OPEN_CM)) - self.radius)

    def strict_center(self, hand: Hand) -> Tuple[float, float]:
        """2: (덜 들어간 양 cm (+ = 짧음), 닫히는 축 비킴 cm) — 집게 가운데 − 캔 축 수직 성분을 그리퍼 틀로."""
        tip, R = hand
        z = self.rot[:, 2]
        v = (tip - GRIP_CENTER_CM * R[:, 0]) - self.pos
        perp = v - float(v @ z) * z
        return -float(perp @ R[:, 0]), float(perp @ R[:, 1])

    def _box_points(self, side: str, hand: Hand, half: float) -> np.ndarray:
        tip, R = hand
        pts = []
        for s0, s1, j0, j1, n0, n1 in self._hand_boxes(side, half):
            g = np.stack(np.meshgrid(np.linspace(s0, s1, max(2, int(math.ceil(s1 - s0)) + 1)), np.linspace(j0, j1, max(2, int(math.ceil(j1 - j0)) + 1)),
                                     np.linspace(n0, n1, max(2, int(math.ceil(n1 - n0)) + 1)), indexing="ij"), -1).reshape(-1, 3)
            pts.append(tip[None, :] - g[:, :1] * R[:, 0][None, :] + g[:, 1:2] * R[:, 1][None, :] + g[:, 2:3] * R[:, 2][None, :])
        return np.concatenate(pts, 0)

    def strict_gap_cm(self, hands: Dict[str, Hand]) -> float:
        """3: 캔을 쥔 두 그리퍼 (날 안쪽 = 캔 반지름) 상자 사이 최소 거리 cm (1 cm 격자 점 → 상대 상자)."""
        a, b = self.sides
        P = self._box_points(a, hands[a], self.radius)
        tip, R = hands[b]
        rel = P - tip[None, :]
        s, j, n = -(rel @ R[:, 0]), rel @ R[:, 1], rel @ R[:, 2]
        return float(min(self._box_dist(s, j, n, bx).min() for bx in self._hand_boxes(b, self.radius)))

    def _strict_flag(self, key: str, text: str, fail: bool) -> None:
        st = self.ep.get("strict")
        if st is None or key in st["flags"]:
            return
        st["flags"].append(key)
        st["fail"] = st["fail"] or fail
        self.log(f"\n[캔] {'🔴 엄격 실패' if fail else '⚠ 엄격 경고'} ({key}): {text}")
        self.sound("alarm" if fail else "tick")

    def _strict_on_close(self, side: str, hand: Hand) -> None:
        st = self.ep.get("strict")
        if st is None:
            return
        who = "왼팔" if side == "left" else "오른팔"
        oc = self.strict_open_cm(side, hand)
        st["open"] = oc if st["open"] is None else min(st["open"], oc)
        if oc < -STRICT_OPEN_CM:
            self._strict_flag(f"open_{HAND_KEY[side]}", f"{who} 닫기 직전 열린 날·몸체가 캔에 {-oc:.1f} cm 들어감 (캔을 민다)", True)
        short, side_cm = self.strict_center(hand)
        st[HAND_KEY[side]] = (short, side_cm)
        if short > STRICT_DEPTH_CM:
            self._strict_flag(f"depth_{HAND_KEY[side]}", f"{who} 집게 가운데가 캔 축보다 {short:.1f} cm 덜 들어감 (날 끝으로 묾)", True)
        if abs(side_cm) > STRICT_SIDE_CM:
            self._strict_flag(f"side_{HAND_KEY[side]}", f"{who} 집게 가운데가 닫히는 축으로 {side_cm:+.1f} cm 비킴 (한쪽 날이 먼저 닿음)", True)

    def _real_flag(self, key: str, text: str) -> None:
        """19-x 실물 기준 실패 (한 에피소드에 까닭마다 한 번). 화면 + 까닭별 음성 (cue_player "say:")."""
        rl = self.ep.get("real")
        if rl is None or key in rl["flags"]:
            return
        rl["flags"].append(key)
        rl["fail"] = True
        self.log(f"\n[캔] 🔴 실물 기준 실패 ({REAL_SAY[key.split('_')[0]]}): {text}")
        self.sound("say:" + REAL_SAY[key.split("_")[0]])

    def _depth_B(self, side: str, hand: Hand):
        """19-h 가정 B (캔을 먼저 손 집게 축 가운데로 옮긴 자세) 에서 둘째 손 판정과 비킴."""
        first = self.holders[0]
        jf = self._grip_j.get(first, 0.0)
        keep = self.pos
        try:
            self.pos = keep - jf * self._hands[first][1][:, 1]
            return self.grip_check(hand, self._extra(), True, side=side), self.strict_center(hand)[1]
        finally:
            self.pos = keep

    def _real_on_close(self, side: str, hand: Hand, chk: dict) -> None:
        """19-x: 잡히는 닫기마다 (붙이기 전). 먼저 손 = 비킴, 둘째 손 = 가정 B 자세에서 깊이 · 비킴."""
        rl = self.ep.get("real")
        if rl is None:
            return
        who = "왼팔" if side == "left" else "오른팔"
        if not self.holders:
            self._grip_j[side] = float(chk["j"])
            side_cm = self.strict_center(hand)[1]
            rl[HAND_KEY[side]] = side_cm
            if abs(side_cm) > REAL_SIDE_CM:
                self._real_flag(f"side_{HAND_KEY[side]}", f"{who} 집게 가운데가 닫히는 축으로 {side_cm:+.1f} cm 비킴 (> {REAL_SIDE_CM:g})")
            return
        first = self.holders[0]
        if first not in self._hands:
            return
        cb, side_cm = self._depth_B(side, hand)
        rl["depth"] = float(cb["s"])
        rl[HAND_KEY[side]] = side_cm
        if self.grasp_mode == "top":                           # 19-x2 인계 각도
            z = self.rot[:, 2]
            tilt = math.degrees(math.acos(min(abs(float(hand[1][:, 2] @ z)), 1.0)))
            axes = math.degrees(math.acos(max(-1.0, min(1.0, float(self._hands[first][1][:, 0] @ hand[1][:, 0])))))
            rl["tilt"], rl["axes"] = tilt, axes
            if tilt < TOP_REAL_FLAT_DEG:
                self._real_flag("flat", f"{who} 기울기 {tilt:.0f}° < {TOP_REAL_FLAT_DEG:g}° (두 손이 수평 — 놓을 때 불안정, 아래 손은 비스듬히)")
            elif not TOP_REAL_TILT_DEG[0] <= tilt <= TOP_REAL_TILT_DEG[1]:
                self._real_flag("angle", f"{who} 기울기 {tilt:.0f}° (허용 {TOP_REAL_TILT_DEG[0]:g} ~ {TOP_REAL_TILT_DEG[1]:g}°)")
            if not TOP_REAL_AXES_DEG[0] <= axes <= TOP_REAL_AXES_DEG[1]:
                self._real_flag("angle_axes", f"두 손 축 사이 {axes:.0f}° (허용 {TOP_REAL_AXES_DEG[0]:g} ~ {TOP_REAL_AXES_DEG[1]:g}°)")
        if cb["s"] < self.real_depth:
            self._real_flag("depth", f"{who} 인계 깊이 {cb['s']:.2f} cm < {self.real_depth:.2f} (논리, 엔코더 상당 {cb['s'] + 1.30:.1f} < {self.real_depth + 1.30:.1f})")
        if abs(side_cm) > REAL_SIDE_CM:
            self._real_flag(f"side_{HAND_KEY[side]}", f"{who} 집게 가운데가 닫히는 축으로 {side_cm:+.1f} cm 비킴 (> {REAL_SIDE_CM:g})")

    def _set_pose(self, pos: np.ndarray, rot: np.ndarray) -> None:
        self.pos, self.rot = pos, rot
        q = quat_of(rot)
        self.q = -q if float(np.dot(q, self.q)) < 0.0 else q     # csv 에서 부호가 튀지 않게

    def _event(self, now: float, kind: str, **info) -> None:
        self.events.append((now, kind, info))

    def _attach(self, now: float, side: str, hand: Hand, chk: dict) -> None:
        tip, R = hand
        self.rel[side] = (R.T @ self.rot, R.T @ (self.pos - tip))
        self.holders.append(side)
        self.at_slot = 0
        self._update_state()
        if self.active:
            self.ep["order"].append(HAND_KEY[side])
        self._event(now, "grip", side=side, state=STATE_NAMES[self.state], t=chk["t"], j=chk["j"], s=chk["s"], n=chk["n"])
        self.log(f"\n[캔] {'왼팔' if side == 'left' else '오른팔'} 잡음 → {STATE_NAMES[self.state]} "
                 f"(밑면에서 {chk['t']:.1f} cm, 집게 축 {chk['j']:+.1f}, 끝에서 {chk['s']:.1f})")

    def _update_state(self) -> None:
        if len(self.holders) >= 2:
            self.state = HELD_BOTH
        elif self.holders:
            self.state = HELD_L if self.holders[0] == "left" else HELD_R

    def _release_one(self, now: float, side: str, hands: Dict[str, Hand]) -> None:
        was_anchor = self.holders[0] == side
        self.holders.remove(side)
        del self.rel[side]
        if self.holders:
            other = self.holders[0]
            tip, R = hands[other]
            slip = 0.0
            if was_anchor:                                     # 다른 팔로 넘어감: 지금 자세 그대로 다시 붙인다 (머리 주석)
                slip = float(np.linalg.norm(tip + R @ self.rel[other][1] - self.pos))
                self.rel[other] = (R.T @ self.rot, R.T @ (self.pos - tip))
            self._update_state()
            if was_anchor:
                if self.active:
                    self.ep["handover"] = True
                self._event(now, "handover", to=other, slip_cm=slip)
                self.log(f"\n[캔] 인계 → {STATE_NAMES[self.state]} (두 손 벌어진 양 {slip:.1f} cm)")
            else:                                              # 나중에 잡은 팔이 놓음: 먼저 잡은 팔이 그대로 들고 있다 (인계 아님)
                self._event(now, "let_go", side=side, state=STATE_NAMES[self.state])
                self.log(f"\n[캔] {'왼팔' if side == 'left' else '오른팔'} 놓음 → {STATE_NAMES[self.state]}")
            return
        self._release_all(now)

    def _release_speed(self) -> float:
        """놓기 직전 PLACE_SPEED_WINDOW_S 동안 밑면 평균 속도 (m/s). 잡힌 시간이 더 짧으면 있는 만큼."""
        if len(self._hist) < 2:
            return 0.0
        t1, p1 = self._hist[-1]
        t0, p0 = self._hist[0]
        for t_, p_ in self._hist:
            if t_ <= t1 - PLACE_SPEED_WINDOW_S:
                t0, p0 = t_, p_
        return float(np.linalg.norm(p1 - p0)) / 100.0 / (t1 - t0) if t1 - t0 > 1e-3 else 0.0

    def _track(self, now: float) -> None:
        """잡힌 동안: 놓기 속도 기록, 판 뚫기 깊이 (머리 주석)."""
        self._hist.append((now, self.pos.copy()))
        while len(self._hist) > 2 and self._hist[1][0] <= now - PLACE_SPEED_WINDOW_S:
            self._hist.pop(0)
        z = self.rot[:, 2]
        drop = self.radius * math.sqrt(max(1.0 - float(z[2]) ** 2, 0.0))
        low = min(float(self.pos[2]), float(self.pos[2] + self.height * z[2])) - drop
        x, y, t = float(self.pos[0]), float(self.pos[1]), self.table
        on_plate = t.x0 * 100.0 <= x <= t.x1 * 100.0 and t.y0 * 100.0 <= y <= t.y1 * 100.0
        self.pen_depth = max(t.top(x / 100.0) * 100.0 - low, 0.0) if on_plate else 0.0
        if self.pen_depth > PEN_ENTER_CM and not self._pen_in:
            self._pen_in = True
            if self.active:
                self.ep["pen"] += 1
            self._event(now, "penetrate")
        elif self.pen_depth <= 0.0:
            self._pen_in = False
        if self.active:
            self.ep["pen_max"] = max(self.ep["pen_max"], self.pen_depth)

    def _goal_dist(self) -> Optional[float]:
        if not self.goal_name:
            return None
        g = self._slot_base(self.goal_name)
        return math.hypot(float(self.pos[0] - g[0]), float(self.pos[1] - g[1]))

    def _place_check(self) -> dict:
        """지금 모든 손을 놓으면 서는가 (놓기 판정, 머리 주석 "놓기"). why = 넘는 조건 글, bits = WHY_*."""
        z = self.rot[:, 2]
        tilt = math.degrees(math.acos(min(max(float(z[2]), -1.0), 1.0)))
        x, y = float(self.pos[0]), float(self.pos[1])
        t = self.table
        on_plate = t.x0 * 100.0 <= x <= t.x1 * 100.0 and t.y0 * 100.0 <= y <= t.y1 * 100.0
        top = t.top(x / 100.0) * 100.0
        h = float(self.pos[2]) - top
        v = self._release_speed()
        why, bits = [], 0
        for w, bad, b in (("판 밖", not on_plate, WHY_OFF), (f"높이 {h:.1f} > {self.place_cm:g} cm", h > self.place_cm, WHY_HEIGHT),
                          (f"기울기 {tilt:.0f} > {self.place_tilt:g}°", tilt > self.place_tilt, WHY_TILT),
                          (f"속도 {v:.2f} > {self.place_speed:g} m/s", v > self.place_speed, WHY_SPEED)):
            if bad:
                why.append(w)
                bits |= b
        return {"h": h, "tilt": tilt, "speed": v, "on_plate": on_plate, "top": top, "why": why, "bits": bits}

    def _contact(self, now: float, hands: Dict[str, Hand]) -> None:
        """잡힌 캔 밑면이 판에 닿으면 똑바로 세우고 상대 자세를 새로 잡는다 (머리 주석 "판 접촉")."""
        z = self.rot[:, 2]
        x, y, t = float(self.pos[0]), float(self.pos[1]), self.table
        if not self.upright or float(z[2]) <= 0.0 or not (t.x0 * 100.0 <= x <= t.x1 * 100.0 and t.y0 * 100.0 <= y <= t.y1 * 100.0):
            return
        top = t.top(x / 100.0) * 100.0
        low = float(self.pos[2]) - self.radius * math.sqrt(max(1.0 - float(z[2]) ** 2, 0.0))
        if low > top + 1e-9:
            if low > top + CONTACT_LEAVE_CM:
                self._contact_in = False
                self._press = 0.0                              # 19-x 누름: 판에서 떨어지면 처음부터
            return
        self._press += top - low                               # 19-x 누름: 바로 세우기 전에 판 아래로 들어간 깊이 (이 걸음 몫)
        if self.real and self.active and self.ep.get("real") is not None and self.ep.get("handover"):   # 인계 뒤 (놓는 손) 만
            rl = self.ep["real"]
            rl["press"] = max(rl["press"], self._press)
            lim = TOP_REAL_PRESS_CM if self.grasp_mode == "top" else REAL_PRESS_CM      # 19-x4 (top 만 1.0)
            if self._press > lim:
                self._real_flag("press", f"잡힌 캔이 판에 닿은 뒤 손이 {self._press:.1f} cm 더 내려감 (> {lim:g}, 눌러 놓기)")
        # 축을 수직으로. 축 둘레 각만 이어받아 매번 새로 만든다 (앞 회전에 곱해 쌓으면 200 Hz 로 누르는 동안 반올림이 커져 행렬이 망가졌다, 13-d 루프백)
        xa = self.rot[:, 0] if abs(float(self.rot[2, 0])) < 0.9 else self.rot[:, 1]
        xa = np.array([float(xa[0]), float(xa[1]), 0.0])
        xa = xa / np.linalg.norm(xa)
        rot = np.column_stack((xa, np.cross([0.0, 0.0, 1.0], xa), [0.0, 0.0, 1.0]))
        tilt = math.degrees(math.acos(min(max(float(z[2]), -1.0), 1.0)))
        old_pos, Q = self.pos.copy(), rot @ self.rot.T                  # 이번 보정 (밑면 가운데 기준 회전 + 옮김)
        self._set_pose(np.array([x, y, top]), rot)
        for i, side in enumerate(self.holders):
            tip, R = hands[side]
            if i == 0:                                         # 기준 팔: 지금 자세 그대로
                self.rel[side] = (R.T @ self.rot, R.T @ (self.pos - tip))
            else:                                              # 다른 팔: 같은 보정만 (두 손 벌어진 양은 인계 때 그대로 알린다)
                Rr, pr = self.rel[side]
                self.rel[side] = (R.T @ (Q @ (R @ Rr)), R.T @ (self.pos + Q @ (tip + R @ pr - old_pos) - tip))
        if not self._contact_in:
            self._contact_in = True
            if self.real and self.grasp_mode == "top" and self.active and self.ep.get("handover"):   # 19-x4: 놓을 캔이 판에 닿음 → 삑 (바로 열기)
                self.sound("twist")
            if self.active:
                self.ep["contact"] += 1
            self._event(now, "contact", tilt_deg=tilt, depth=top - low)

    def _release_all(self, now: float) -> None:
        self._skip_bumps()
        z = self.rot[:, 2]
        pc = self._place_check()
        h, tilt, v, on_plate, top, why = pc["h"], pc["tilt"], pc["speed"], pc["on_plate"], pc["top"], pc["why"]
        x, y, t = float(self.pos[0]), float(self.pos[1]), self.table
        gd = self._goal_dist()
        self._hist = []
        self.pen_depth, self._pen_in = 0.0, False
        self.place_ok = False
        self.last_place = {"h": h, "tilt": tilt, "speed": v, "result": "fallen" if why else "stood", "why": ", ".join(why),
                           "bits": pc["bits"] if why else WHY_STOOD, "goal_dist": gd}
        if self.active:
            self.ep["place"] = self.last_place
        if not why:
            self._set_pose(np.array([x, y, top]), np.eye(3))
            self._contact_in = True
            self.state = ON_TABLE
            self._event(now, "place", x=x, y=y, h=h, tilt_deg=tilt, speed_ms=v)
            if self.strict and self.active and gd is not None and self.ep.get("strict") is not None:   # 16-a 4 (경고만)
                self.ep["strict"]["goal"] = gd
                if gd > STRICT_GOAL_CM:
                    self._strict_flag("goal", f"놓은 캔 밑면이 목표 {self.goal_name} 에서 {gd:.1f} cm (> {STRICT_GOAL_CM:g})", False)
            self.log(f"\n[캔] 놓음 → 섬 ({x:.1f}, {y:.1f}) (판 위 {h:.1f} cm, 기울기 {tilt:.0f}°, 속도 {v:.2f} m/s)")
        else:
            d = np.array([z[0], z[1], 0.0])
            d = d / np.linalg.norm(d) if float(np.linalg.norm(d)) > 1e-3 else np.array([1.0, 0.0, 0.0])
            c = self.pos + 0.5 * self.height * z
            cx = min(max(float(c[0]), t.x0 * 100.0), t.x1 * 100.0)
            cy = min(max(float(c[1]), t.y0 * 100.0), t.y1 * 100.0)
            centre = np.array([cx, cy, t.top(cx / 100.0) * 100.0 + self.radius])
            self._set_pose(centre - 0.5 * self.height * d, frame_with_z(d))
            self.state = FALLEN
            if self.active:
                self.ep["fallen"] = True
            self._event(now, "fall", h=h, tilt_deg=tilt, on_plate=on_plate, speed_ms=v, why=", ".join(why))
            self.log(f"\n[캔] 놓음 → 넘어짐: {', '.join(why)} (판 위 {h:.1f} cm, 기울기 {tilt:.0f}°, 속도 {v:.2f} m/s)")

    def _extra(self) -> float:
        return self.handover_extra if self.holders else 0.0

    def _update_ready(self, hands: Dict[str, Hand]) -> None:
        can = self.state != FALLEN and self._reset_due is None
        for side in self.sides:
            self.ready[side] = (can and not self.closed[side] and side not in self.holders
                                and self.grip_check(hands[side], self._extra(), bool(self.holders), side=side)["ok"])
            if self.ready[side] and self.ready_strict:         # 16-d (켤 때만): 엄격 2 와 같은 식·같은 문턱
                short, side_cm = self.strict_center(hands[side])
                self.ready[side] = short <= STRICT_DEPTH_CM and abs(side_cm) <= STRICT_SIDE_CM
            if self.ready[side] and self.real and self.grasp_mode == "top" and self.holders and self.holders[0] in self._hands:   # 19-x4: 받는 손 노란 원 = 실물 기준 깊이
                cb, side_cm = self._depth_B(side, hands[side])
                self.ready[side] = cb["s"] >= self.real_depth and abs(side_cm) <= REAL_SIDE_CM

    def step(self, now: float, hands: Dict[str, Hand], grips: Dict[str, int], running: bool = True) -> None:
        """한 걸음. hands = 팔별 그리퍼 틀, grips = 팔별 0 열림 / 1 닫힘, running = 흐름 시간이 흐르는가 (위치 모드·hold 아님)."""
        self._hands = hands                                    # 19-x 실물 기준 판정 (둘째 손 깊이) 이 먼저 손 자세를 본다
        if self._clock_last is not None and running:           # 끝낼 때 다시 세울 시각을 잡으므로 이 걸음 판정보다 먼저 흐른다
            self._clock += now - self._clock_last
        self._clock_last = now
        changed = [(s, bool(grips.get(s, 0))) for s in self.sides if bool(grips.get(s, 0)) != self.closed[s]]
        for side, now_closed in changed:                       # 여는 것을 먼저 (같은 순간에 다른 손이 닫으면 놓은 뒤의 자세로 잰다)
            if not now_closed:
                self.closed[side] = False
                if side in self.holders:
                    self._release_one(now, side, hands)
        for side, now_closed in changed:
            if now_closed:
                self.closed[side] = True
                self._on_close(now, side, hands[side])
        if self.holders:
            first = self.holders[0]
            tip, R = hands[first]
            Rr, pr = self.rel[first]
            self._set_pose(tip + R @ pr, R @ Rr)
            self.anchor_tip = tip
            self._contact(now, hands)
            self._track(now)
        else:
            self.anchor_tip = None
        if self.strict and self.active and len(self.holders) == 2 and now - self._strict_gap_t >= STRICT_GAP_PERIOD_S:   # 16-a 3
            self._strict_gap_t = now
            g = self.strict_gap_cm(hands)
            st = self.ep.get("strict")
            if st is not None:
                st["gap"] = g if st["gap"] is None else min(st["gap"], g)
                if g < STRICT_GAP_CM:
                    self._strict_flag("gap", f"두 그리퍼 (서보 몸체 포함) 간격 {g:.1f} cm < {STRICT_GAP_CM:g} (서로 긁을 수 있다)", False)
        self._update_bump(now, hands)
        if self.align is not None:                             # 14-s (정렬 판정을 켤 때만)
            self._update_push(now, hands)
        self._check_goal(now, hands)
        self._cycle(now, hands, running)
        self._update_ready(hands)
        if (self.top_live_axis and self.pairs is not None and not self.holders and "left" in hands
                and self.at_slot == self.slots[PAIR_START] + 1):   # 19-x6: 시작 자리 그림 손 축 = 지금 L 손 축 (TOP_LIVE_AXIS_NEAR_CM 주석)
            tip, R = hands["left"]
            if self.tip_dist_cm(tip) <= TOP_LIVE_AXIS_NEAR_CM:
                self.table.cfg["can_slots"][self.slots[PAIR_START]]["hand_axis"] = [round(float(v), 4) for v in R[:, 0]]
        self.place_ok = bool(self.holders) and not self._place_check()["why"]
        gd = self._goal_dist()
        self.in_goal = gd is not None and self.state != FALLEN and gd <= self.goal_tol

    def _on_close(self, now: float, side: str, hand: Hand) -> None:
        if self.state == FALLEN or self._reset_due is not None:
            reason = "넘어짐" if self.state == FALLEN else "다시 세우기 전"
            self._event(now, "miss", side=side, reason=reason)
            if now - self._miss_print_t[side] >= MISS_PRINT_S and self.state == FALLEN:
                self._miss_print_t[side] = now
                self.log(f"\n[캔] {'왼팔' if side == 'left' else '오른팔'} 못 잡음: {reason}")
            return
        chk = self.grip_check(hand, self._extra(), bool(self.holders), side=side)
        if chk["ok"]:
            if self.strict and self.active:                    # 16-a (켤 때만)
                self._strict_on_close(side, hand)
            if self.real and self.active:                      # 19-x (켤 때만)
                self._real_on_close(side, hand, chk)
            self._attach(now, side, hand, chk)
            return
        al = {k: chk[k] for k in ("align_deg", "axis_cm", "align_lim") if k in chk}      # 14-l 정렬이 켜져 있고 나머지가 맞을 때만 있다
        self._event(now, "miss", side=side, reason=chk["reason"], t=chk["t"], j=chk["j"], s=chk["s"], n=chk["n"], tilt_deg=chk["tilt_deg"], **al)
        if al:
            self.align_miss += 1
            if self.active:
                self.ep["align_miss"] = self.ep.get("align_miss", 0) + 1
            who = "왼팔" if side == "left" else "오른팔"
            if self.holders:
                self.log(f"\n[캔] {who} 못 잡음: {chk['reason']} — 셋째 축 ↔ 쥔 캔 축 {al['align_deg']:.1f}° (한도 {al['align_lim']:.0f}°), "
                         f"집게 가운데 ↔ 캔 축 {al['axis_cm']:.1f} cm (한도 {self.align[2]:.1f} cm)")
            else:                                              # 14-s 먼저 잡는 손
                self.log(f"\n[캔] {who} 못 잡음: {chk['reason']} — 집게 축 ⟂ 캔 축 어긋남 {chk['tilt_deg']:.1f}° (한도 {al['align_lim']:.0f}°), "
                         f"숙임 (셋째 축 ↔ 캔 축) {al['align_deg']:.1f}° (한도 {ALIGN_N_MAX_DEG:.0f}°)")
            return
        dist = float(np.linalg.norm(self.pos + 0.5 * self.height * self.rot[:, 2] - hand[0]))
        if dist < MISS_PRINT_CM and now - self._miss_print_t[side] >= MISS_PRINT_S:
            self._miss_print_t[side] = now
            self.log(f"\n[캔] {'왼팔' if side == 'left' else '오른팔'} 못 잡음: {chk['reason']} "
                     f"(밑면에서 {chk['t']:.1f}, 집게 축 {chk['j']:+.1f}, 끝에서 {chk['s']:.1f}, 셋째 축 {chk['n']:+.1f}, 기울기 {chk['tilt_deg']:.0f}°)")

    def _update_bump(self, now: float, hands: Dict[str, Hand]) -> None:
        for side in self.sides:
            hit = (not self.holders) and self.overlap(side, hands[side], self.closed[side])
            if self._bump_skip[side]:
                self._bump_skip[side] = hit
                continue
            if hit and not self.bumping[side] and now - self._bump_clear_t[side] >= BUMP_REARM_S:
                self.bump_total += 1
                if self.active:
                    self.ep["bumps"] += 1
                self._event(now, "bump", side=side)
                if self.real and self.active and not self.ep["order"] and self.ep.get("real") is not None:   # 19-x: 잡기 전 부딪힘
                    self.ep["real"]["bump"] += 1
                    self._real_flag("bump", f"{'왼팔' if side == 'left' else '오른팔'} 집게 · 몸체가 잡기 전에 캔에 닿음 (실물이면 캔이 밀림)")
            if self.bumping[side] and not hit:
                self._bump_clear_t[side] = now
            self.bumping[side] = hit

    def _update_push(self, now: float, hands: Dict[str, Hand]) -> None:
        """14-s: 한 손이 쥔 캔을 다가가는 둘째 손 (이 에피소드에서 아직 안 잡은 손) 의 열린 집게·몸체가 민다 (닫기 전에 닿음). 놓고 빠지는 손은 세지 않는다. 실제로는 캔이 먼저 잡은 집게 안에서 돈다 → 화면·소리로 알린다."""
        if not hasattr(self, "pushing"):
            self.pushing = {s: False for s in self.sides}
            self._push_clear_t = {s: -math.inf for s in self.sides}
            self.push_total = 0
        for side in self.sides:
            hit = (self.active and len(self.holders) == 1 and side not in self.holders and not self.closed[side]
                   and HAND_KEY[side] not in self.ep.get("order", [])          # 이 에피소드에서 아직 잡은 적 없는 손 (다가가는 둘째 손) 만
                   and self.overlap(side, hands[side], False))
            if hit and not self.pushing[side] and now - self._push_clear_t[side] >= PUSH_REARM_S:
                self.push_total += 1
                if self.active:
                    self.ep["push"] = self.ep.get("push", 0) + 1
                self._event(now, "push", side=side)
                who, other = ("왼팔", "오른팔") if side == "left" else ("오른팔", "왼팔")
                self.log(f"\n[캔] ⚠ {who} 집게가 {other}이 쥔 캔을 밈 — 실제로는 캔이 {other} 집게 안에서 돈다. 조금 비켜서 캔 축에 맞춰 다시 대기")
                self.sound("tick")
            if self.pushing[side] and not hit:
                self._push_clear_t[side] = now
            self.pushing[side] = hit

    def _check_goal(self, now: float, hands: Optional[Dict[str, Hand]] = None) -> None:
        if self.episode_end == "release":
            self._check_release(now, hands)
            return
        if not (self.active and self.goal_name and self.state == ON_TABLE and self.ep["order"]):
            return
        if any(self.closed.values()):
            return
        g = self._slot_base(self.goal_name)
        if math.hypot(float(self.pos[0] - g[0]), float(self.pos[1] - g[1])) <= self.goal_tol:
            self.end_episode(now, "auto_success")

    def tip_dist_cm(self, tip: np.ndarray) -> float:
        """집게 끝 점에서 캔 (원통) 겉면까지 cm. 안이면 음수."""
        loc = self.rot.T @ (np.asarray(tip, float) - self.pos)
        rad = math.hypot(float(loc[0]), float(loc[1])) - self.radius
        vz = max(-float(loc[2]), float(loc[2]) - self.height)
        if rad <= 0.0 and vz <= 0.0:
            return max(rad, vz)
        return math.hypot(max(rad, 0.0), max(vz, 0.0))

    def _set_phase(self, phase: str) -> None:
        if phase != self.phase:
            self.phase = phase
            if phase:
                self.log(f"\n[에피소드 {self.episode}] {phase}" + (" — 그리퍼를 열고 손을 빼면 성공" if phase == "놓기 대기" else
                                                                  f" — 두 집게 끝이 캔에서 {RELEASE_CLEAR_CM:g} cm 넘게 {RELEASE_CLEAR_S:g} 초면 성공"))

    def _check_release(self, now: float, hands: Optional[Dict[str, Hand]]) -> None:
        """작업 지시 14-h --episode-end release: 놓고 손을 뺀 뒤 성공."""
        self.tip_clear_cm = (min(self.tip_dist_cm(hands[s][0]) for s in self.sides) if hands else math.nan)
        if not (self.active and self.goal_name and self.state != FALLEN and self.ep["order"]):
            self._clear_since = None
            self._set_phase("")
            return
        g = self._slot_base(self.goal_name)
        in_goal = math.hypot(float(self.pos[0] - g[0]), float(self.pos[1] - g[1])) <= self.goal_tol
        if self.holders:
            self._clear_since = None
            self._set_phase("놓기 대기" if in_goal else "")
            return
        if not (in_goal and self.state == ON_TABLE):
            self._clear_since = None
            self._set_phase("")
            return
        self._set_phase("손 빼기")
        clear = not any(self.closed.values()) and self.tip_clear_cm >= RELEASE_CLEAR_CM
        if not clear:
            self._clear_since = None
            return
        self._clear_since = now if self._clear_since is None else self._clear_since
        if now - self._clear_since >= RELEASE_CLEAR_S:
            self._clear_since = None
            self._set_phase("")
            self.end_episode(now, "auto_success")

    def _slot_clear(self, hands: Dict[str, Hand]) -> bool:
        """시작 자리에 다시 세워도 그리퍼와 겹치지 않는가."""
        pos, rot = self.pos, self.rot
        self.pos, self.rot = self._slot_base(self.start_name), np.eye(3)
        try:
            return not any(self.overlap(s, hands[s], self.closed[s]) for s in self.sides)
        finally:
            self.pos, self.rot = pos, rot

    def _cycle(self, now: float, hands: Dict[str, Hand], running: bool) -> None:
        """키보드 없는 흐름 (머리 주석). reset_s 가 없으면 아무것도 안 한다."""
        if self.reset_s is None:
            return
        if not running:
            return
        if self.active:
            if self.state == FALLEN:
                self.end_episode(now, "fallen")
            elif self.timeout_s > 0.0 and self._clock - self._ep_clock0 >= self.timeout_s:
                self.end_episode(now, "timeout")
        if self.active or self._reset_due is None or self._clock < self._reset_due:
            return
        if self.start_gate is not None and not self._gate_ok(now, hands):      # 19-x 시작 자세 (켤 때만)
            return
        if self._slot_clear(hands):
            self.new_episode(now)
        elif not self._reset_waiting:
            self._reset_waiting = True
            self._event(now, "reset_wait")
            self.log(f"\n[캔] {self.start_name} 자리에 그리퍼가 겹쳐 있다 — 비키면 다시 세운다")

    def _gate_ok(self, now: float, hands: Dict[str, Hand]) -> bool:
        """19-x 시작 자세: 두 집게 닫힘 + 두 집게 끝이 판 위 start_gate cm 이상 + 시작 캔 축에서 수평 (반지름 + START_GATE_CLEAR_CM) 넘게, START_GATE_HOLD_S 이어짐."""
        if self._gate_wait_t0 is None:
            self._gate_wait_t0 = now
        base = self._slot_base(self.start_name)
        why = []
        for s in self.sides:
            k = HAND_KEY[s]
            tip = hands[s][0]
            top = self.table.top(float(tip[0]) / 100.0) * 100.0
            if not self.closed[s]:
                why.append(f"{k} 집게 열림")
            if float(tip[2]) - top < self.start_gate:
                why.append(f"{k} 낮음 (판 위 {float(tip[2]) - top:.0f} cm)")
            if math.hypot(float(tip[0] - base[0]), float(tip[1] - base[1])) <= self.radius + START_GATE_CLEAR_CM:
                why.append(f"{k} 캔에 가까움")
        if why:
            self._gate_since = None
            last = getattr(self, "_gate_say_t", None)
            if last is None or now - last >= START_GATE_SAY_S:      # 19-x2: 기다리는 동안 5 초마다 안내 (처음 한 번은 바로)
                self._gate_say_t = now
                self.log(f"\n[에피소드] 시작 자세를 기다림 — 두 그리퍼를 닫고 두 손을 판 위 {self.start_gate:g} cm 넘게, 캔에서 떨어져 {START_GATE_HOLD_S:g} 초 "
                         f"(지금: {', '.join(why)})")
                self.sound("say:" + START_GATE_SAY)
            return False
        self._gate_since = now if self._gate_since is None else self._gate_since
        return now - self._gate_since >= START_GATE_HOLD_S

    # ------------------------------------------------------------------ 에피소드
    def new_episode(self, now: float) -> None:
        if self.active:
            self.end_episode(now, "manual_abort")
        self._flush_row(now)
        self.reset()
        self._reset_due, self._reset_waiting = None, False
        self._ep_clock0 = self._clock
        self.episode += 1
        self.active = True
        self.ep = {"t0": now, "order": [], "handover": False, "bumps": 0, "fallen": False, "place": None, "pen": 0, "pen_max": 0.0, "contact": 0}
        if self.strict:                                         # 16-a
            self.ep["strict"] = {"open": None, "L": None, "R": None, "gap": None, "goal": None, "flags": [], "fail": False}
        if self.real:                                           # 19-x
            self.ep["real"] = {"depth": None, "L": None, "R": None, "press": 0.0, "bump": 0, "tilt": None, "axes": None, "flags": [], "fail": False}
            self._press, self._grip_j = 0.0, {}
        if self.start_gate is not None:                         # 19-x: 시작 자세를 기다린 시간
            self.ep["gate_wait"] = (now - self._gate_wait_t0) if self._gate_wait_t0 is not None else 0.0
            self._gate_wait_t0, self._gate_since, self._gate_said = None, None, False
            self._gate_say_t = None
            self.sound("say:시작")                              # 19-x2: 관문 통과
        self._event(now, "episode", number=self.episode)
        self.log(f"\n[에피소드 {self.episode}] 시작 — 캔을 {self.start_name} 에 다시 세움" + (f", 목표 {self.goal_name}" if self.goal_name else ""))
        self.sound("go")

    def end_episode(self, now: float, result: str) -> bool:
        if not self.active:
            return False
        self.active = False
        e = self.ep
        row = {"episode": self.episode, "t_mono_start": f"{e['t0']:.3f}", "t_mono_end": f"{now:.3f}", "start_slot": self.start_name,
               "goal_slot": self.goal_name or "", "result": result, "grip_order": ">".join(e["order"]), "handover": int(e["handover"]),
               "bump_count": e["bumps"], "fallen": int(e["fallen"]), "t_mono_reset": ""}
        pl = e.get("place")                                     # 이 에피소드 마지막 놓기 (작업 지시 13-c §4)
        row.update({"place_h_cm": f"{pl['h']:.2f}" if pl else "", "place_tilt_deg": f"{pl['tilt']:.1f}" if pl else "",
                    "place_speed_ms": f"{pl['speed']:.3f}" if pl else "", "place_result": pl["result"] if pl else "",
                    "penetration": e.get("pen", 0), "penetration_max_cm": f"{e.get('pen_max', 0.0):.2f}",
                    "place_why": pl["why"] if pl else "", "goal_dist_cm": f"{pl['goal_dist']:.2f}" if pl and pl.get("goal_dist") is not None else "",
                    "contact": e.get("contact", 0)})
        if self.align is not None:                             # 14-l: 켤 때만 끝에 한 칸 (14-s: 둘째 손 밀기 수 한 칸 더)
            row["align_miss"] = str(e.get("align_miss", 0))
            row["second_push"] = str(e.get("push", 0))
        if self.combos is not None:                            # 16-a: 그 에피소드의 자리 좌표, 조합 성공·전체 수 (끝에 더함)
            key = f"{self.start_name}:{self.goal_name}"
            st = self.combo_stat[key]
            st[1] += 1
            st[0] += int(result in ("auto_success", "manual_success"))
            sb, gb = self._slot_base(self.start_name), self._slot_base(self.goal_name)
            row.update({"start_x_cm": f"{sb[0]:.2f}", "start_y_cm": f"{sb[1]:.2f}", "goal_x_cm": f"{gb[0]:.2f}", "goal_y_cm": f"{gb[1]:.2f}",
                        "combo_index": str(self.combo_i), "combo_ok": str(st[0]), "combo_n": str(st[1])})
        if self.pairs is not None:                             # 19-b: 조합과 같은 칸 (combo_index = 쌍 번호, 그 쌍의 성공·전체 수)
            st = self.pair_stat[self.pair_i]
            st[1] += 1
            st[0] += int(result in ("auto_success", "manual_success"))
            sb, gb = self._slot_base(self.start_name), self._slot_base(self.goal_name)
            row.update({"start_x_cm": f"{sb[0]:.2f}", "start_y_cm": f"{sb[1]:.2f}", "goal_x_cm": f"{gb[0]:.2f}", "goal_y_cm": f"{gb[1]:.2f}",
                        "combo_index": str(self.pair_i), "combo_ok": str(st[0]), "combo_n": str(st[1])})
        if self.strict:                                        # 16-a
            sx = e.get("strict") or {}
            f2 = lambda v: "" if v is None else f"{v:.2f}"
            row.update({"strict_open_cm": f2(sx.get("open")), "strict_L_depth_cm": f2((sx.get("L") or (None, None))[0]),
                        "strict_L_side_cm": f2((sx.get("L") or (None, None))[1]), "strict_R_depth_cm": f2((sx.get("R") or (None, None))[0]),
                        "strict_R_side_cm": f2((sx.get("R") or (None, None))[1]), "strict_gap_cm": f2(sx.get("gap")), "strict_goal_cm": f2(sx.get("goal")),
                        "strict_flags": " ".join(sx.get("flags", [])), "strict_fail": str(int(bool(sx.get("fail"))))})
        if self.real:                                          # 19-x
            rx = e.get("real") or {}
            f2r = lambda v: "" if v is None else f"{v:.2f}"
            row.update({"real_depth_cm": f2r(rx.get("depth")), "real_L_side_cm": f2r(rx.get("L")), "real_R_side_cm": f2r(rx.get("R")),
                        "real_press_cm": f2r(rx.get("press")), "real_bump": str(rx.get("bump", 0)), "real_tilt_deg": f2r(rx.get("tilt")),
                        "real_axes_deg": f2r(rx.get("axes")), "real_flags": " ".join(rx.get("flags", [])),
                        "real_fail": str(int(bool(rx.get("fail"))))})
        if self.start_gate is not None:                        # 19-x
            row["start_gate_wait_s"] = f"{e.get('gate_wait', 0.0):.2f}"
        self.results.append(row)
        self._row = row
        self._event(now, "episode_end", number=self.episode, result=result)
        self.log(f"\n[에피소드 {self.episode}] 끝: {result} ({now - e['t0']:.1f}초, 잡은 차례 {row['grip_order'] or '-'}, "
                 f"인계 {row['handover']}, bump {row['bump_count']}, 넘어짐 {row['fallen']})")
        if self.combos is not None and result != "abort":     # 16-a: 다음 조합 (차례로)
            self.combo_i = (self.combo_i + 1) % len(self.combos)
            self.start_name, self.goal_name = self.combos[self.combo_i]
            self.log(f"\n[조합] 다음 에피소드: {self.start_name} → {self.goal_name}   (성공/전체 "
                     + ", ".join(f"{k} {v[0]}/{v[1]}" for k, v in self.combo_stat.items()) + ")")
        if self.pairs is not None and result != "abort":      # 19-b: 다음 좌표 쌍 (차례로) — 자리 xy 를 갈아 끼운다
            self.pair_i = (self.pair_i + 1) % len(self.pairs)
            self._pair_set(self.pair_i)
            p = self.pairs[self.pair_i]
            self.log(f"\n[좌표 쌍] 다음 에피소드: {self.pair_i + 1} / {len(self.pairs)} 번  시작 ({p[0]:g}, {p[1]:g}) → 도착 ({p[2]:g}, {p[3]:g})   "
                     f"(성공/전체 합 {sum(v[0] for v in self.pair_stat)}/{sum(v[1] for v in self.pair_stat)})")
        if self.reset_s is not None and result != "abort":
            self._reset_due = self._clock + self.reset_s
            self.log(f"\n[에피소드] {self.reset_s:.0f}초 뒤 캔을 {self.start_name} 에 다시 세운다")
        self.sound({"manual_success": "done", "auto_success": "done", "manual_fail": "stop", "fallen": "stop", "timeout": "stop"}.get(result, "tick"))
        if self.combos is not None and result != "abort" and len(self.combos) > 1:
            self.sound("ready")                                # 16-a: 조합이 바뀜 (VR 목표 원·캔 자리도 바뀐다). switch 는 가상 그리퍼 "다 열림" 소리라 쓰지 않는다
        if self.pairs is not None and result != "abort" and len(self.pairs) > 1:
            self.sound("ready")                                # 19-b: 좌표 쌍이 바뀜 (조합과 같은 소리)
        return True

    def _ep_header(self) -> List[str]:
        return (EPISODE_HEADER + (["align_miss", "second_push"] if self.align is not None else [])
                + (COMBO_HEADER if (self.combos is not None or self.pairs is not None) else []) + (STRICT_HEADER if self.strict else [])
                + (REAL_HEADER if self.real else []) + (GATE_HEADER if self.start_gate is not None else []))

    def _flush_row(self, now: Optional[float]) -> None:
        """끝난 행을 다시 세운 시각과 함께 쓴다 (now = None 이면 다시 세우지 않고 종료)."""
        row, self._row = self._row, None
        if row is None:
            return
        if now is not None:
            row["t_mono_reset"] = f"{now:.3f}"
            self._event(now, "reset", number=row["episode"])
        if self._ep_writer is not None:
            self._ep_writer.writerow([row[k] for k in self._ep_header()])
            self._ep_file.flush()

    def handle_keys(self, now: float, state: dict) -> None:
        """워치독 keyboard_worker 가 세운 표시를 처리한다."""
        if state.pop("ep_new", False):
            self.new_episode(now)
        for key, result in (("ep_success", "manual_success"), ("ep_fail", "manual_fail")):
            if state.pop(key, False) and not self.end_episode(now, result):
                self.log("\n[에피소드] 진행 중이 아니다 — n 으로 시작")

    # ------------------------------------------------------------------ 제어 루프에서
    def tick(self, now: float, command_target: Dict[str, Dict[str, float]], gripper=None, state: Optional[dict] = None, running: bool = True) -> None:
        """예외를 삼킨다 (제어 루프 보호, robot_viz.py tick 과 같은 성질)."""
        try:
            if state is not None:
                self.handle_keys(now, state)
            hands = {s: self.hand_of(s, command_target[s]) for s in self.sides}
            gst = gripper.state() if gripper is not None else {}
            self.step(now, hands, {s: int(gst.get(HAND_KEY[s], 0) or 0) for s in self.sides}, running)
        except Exception as exc:
            if self.error != f"step: {exc}":
                self.error = f"step: {exc}"
                print(f"\n🔴 [가상 캔] 계산 실패: {exc}")
            return
        if self.sock is not None and now - self.last_send >= RVC_PERIOD:
            self.last_send = self.last_send + RVC_PERIOD if now - self.last_send < 2 * RVC_PERIOD else now
            try:
                self.sock.sendto(self.packet(), self.addr)
                self.sent += 1
            except OSError as exc:
                if self.error != str(exc):
                    self.error = str(exc)
                    print(f"\n🔴 [가상 캔] RVC1 못 보냄 ({self.addr[0]}:{self.addr[1]}): {exc}")

    def packet(self) -> bytes:
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        tip = self.anchor_tip if self.anchor_tip is not None else (float("nan"),) * 3
        flags = ((FLAG_ACTIVE if self.active else 0) | (FLAG_BUMP if any(self.bumping.values()) else 0)
                 | (FLAG_READY_L if self.ready.get("left") else 0) | (FLAG_READY_R if self.ready.get("right") else 0))
        anchor = NO_ARM if not self.holders else (0 if self.holders[0] == "left" else 1)
        bumps = self.ep.get("bumps", 0) if self.active else 0
        if self.goal_name:
            flags |= (self.slots[self.goal_name] + 1) << GOAL_SHIFT
        shown = self.pos + np.array([0.0, 0.0, self.pen_depth]) if self.holders else self.pos   # 판 뚫기: 표시만 판 위로
        place = (PLACE_OK if self.place_ok else 0) | (IN_GOAL if self.in_goal else 0)
        why = self.last_place["bits"] if self.last_place else 0
        gd = self._goal_dist()
        dist = NO_DIST if gd is None else min(int(round(gd * 10.0)), NO_DIST - 1)
        return RVC.pack(RVC_MAGIC, self.seq, *[float(v) for v in shown], *[float(v) for v in self.q], *[float(v) for v in tip],
                        self.state, flags, anchor, self.at_slot, self.episode & 0xFFFF, min(bumps, 0xFFFF), place, why, dist)

    # ------------------------------------------------------------------ 기록
    def csv_row(self) -> List[str]:
        row = ([f"{v:.2f}" for v in self.pos] + [f"{v:.4f}" for v in self.q]
               + [STATE_NAMES[self.state], str(self.episode if self.active else 0)])
        if self.episode_end == "release":                   # 작업 지시 14-h (켤 때만, 열 이름 CSV_COLUMNS_RELEASE)
            row += [{"": "-", "놓기 대기": "place_wait", "손 빼기": "withdraw"}[self.phase], f"{self.tip_clear_cm:.2f}"]
        return row

    def run_info(self) -> dict:
        return {"start": self.start_name, "goal": self.goal_name, "margin_cm": self.margin, "tilt_max_deg": GRIP_TILT_MAX_DEG,
                "release_height_cm": self.place_cm, "release_tilt_deg": self.place_tilt, "release_speed_ms": self.place_speed,
                "release_speed_window_s": PLACE_SPEED_WINDOW_S, "goal_radius_cm": self.goal_tol, "upright_on_contact": self.upright,
                "straight_cm": list(self.straight), "handover_extra_cm": self.handover_extra, "episode_reset_s": self.reset_s,
                "episode_timeout_s": self.timeout_s if self.reset_s is not None else None, "episodes_csv": str(self.episodes_path) if self.episodes_path else None,
                **({"episode_end": {"mode": "release", "clear_cm": RELEASE_CLEAR_CM, "clear_s": RELEASE_CLEAR_S}} if self.episode_end == "release" else {}),
                **({"grasp_align": {"first_deg": self.align[0], "second_deg": self.align[1], "second_axis_cm": self.align[2],
                                    "grip_center_cm": GRIP_CENTER_CM, "first_rule": "jaw_perp (14-s)", "first_n_max_deg": ALIGN_N_MAX_DEG,
                                    "second_ref": "held_can_axis"}} if self.align is not None else {}),
                **({"combos": [f"{a}:{b}" for a, b in self.combos]} if self.combos is not None else {}),
                **({"pairs": [list(p) for p in self.pairs], "pair_slots": [PAIR_START, PAIR_GOAL], "hand_axis_model": HAND_AXIS_MODEL} if self.pairs is not None else {}),
                **({"grasp_strict": {"open_cm": STRICT_OPEN_CM, "depth_cm": STRICT_DEPTH_CM, "side_cm": STRICT_SIDE_CM, "gap_cm": STRICT_GAP_CM,
                                     "goal_cm": STRICT_GOAL_CM, "fail_items": ["open", "depth", "side"], "warn_items": ["gap", "goal"]}} if self.strict else {}),
                **({"ready_strict": {"depth_cm": STRICT_DEPTH_CM, "side_cm": STRICT_SIDE_CM}} if self.ready_strict else {}),
                **({"grasp_mode": {"mode": "top", "first_t_cm": list(TOP_FIRST_T_CM), "second_t_cm": list(TOP_SECOND_T_CM), "first_arm": "left",
                                   "band_header_cm": TOP_BAND_HDR_CM if self.top_band else None, "align_second_deg_min": TOP_ALIGN_SECOND_DEG,
                                   "real_R_tilt_deg": list(TOP_REAL_TILT_DEG), "real_flat_deg": TOP_REAL_FLAT_DEG, "real_axes_deg": list(TOP_REAL_AXES_DEG),
                                   "real_press_cm": TOP_REAL_PRESS_CM, "contact_beep": "twist", "ready_second_depth": "real"}}
                   if self.grasp_mode == "top" else {}),
                **({"grasp_real": {"depth_logic_cm": self.real_depth, "depth_basis": "19-h B, Δ 1.30 (19-j)", "side_cm": REAL_SIDE_CM, "press_cm": REAL_PRESS_CM,
                                   "bump_before_grasp": "fail"}} if self.real else {}),
                **({"start_gate": {"h_cm": self.start_gate, "clear_cm": START_GATE_CLEAR_CM, "hold_s": START_GATE_HOLD_S, "grippers": "closed",
                                   "say_every_s": START_GATE_SAY_S, "say": START_GATE_SAY}}
                   if self.start_gate is not None else {})}

    def close(self, now: float) -> None:
        if self.active:
            self.end_episode(now, "abort")
        self._flush_row(None)
        if self._ep_file is not None:
            self._ep_file.close()
            self._ep_file = None


def unpack(payload: bytes) -> dict:
    """시험·기록용: RVC1 패킷 → dict."""
    if len(payload) != PACKET_SIZE or payload[:4] != RVC_MAGIC:
        raise ValueError(f"RVC1 아님 ({len(payload)} 바이트)")
    v = RVC.unpack(payload)
    return {"seq": v[1], "pos": v[2:5], "q": v[5:9], "anchor_tip": v[9:12], "state": v[12], "flags": v[13], "anchor": v[14], "slot": v[15],
            "episode": v[16], "bumps": v[17], "place": v[18], "why": v[19], "goal_dist_mm": v[20]}
