# 로봇 PC 측 VR 팔 트래킹 분석 리포트

작성일: 2026-09-11
대상 코드: `vr_control_watchdog_biarm.py` (이 저장소)
대상 로봇: Berkeley Humanoid Lite, 5-DoF/팔, CAN (can0=left, can1=right)

---

## 1. 요약

사용자가 관찰한 증상 — **"어깨 모터(1,2번=양팔 shoulder_pitch, 3,4번=양팔 shoulder_roll)가
자연스럽게 반영되지 않는다"** — 는 코드 분석과 언리얼 측(별도 팀/별도 Claude Code) 리포트를
교차 검증한 결과, **두 개의 독립적인 원인이 겹친 것**으로 판단된다.

| # | 원인 | 위치 | 심각도 | 조치 주체 |
| --- | --- | --- | --- | --- |
| 1 | **몸통(가슴) 회전이 어깨 명령에 그대로 섞여 들어감** — Unreal이 보내는 `L_UpperArm`/`R_UpperArm` 쿼터니언이 "트래킹 공간 절대 회전"이고 가슴 기준으로 상대화되어 있지 않음 | Unreal 송신 측 (`BodyTrackingSenderComponent.cpp`) | **치명 — 주 원인으로 추정** | 언리얼 측 (진행 중, 아래 §4) |
| 2 | **어깨 관절 소프트 리밋이 사실상 무제한(±10 rad)** — 코드 주석은 `arm_config.yaml` 실측값으로 교체했다고 하지만 실제 `DEFAULT_URDF_LIMITS` 딕셔너리 값은 교체되지 않았음 | `vr_control_watchdog_biarm.py:135-150` | **높음 — 로봇 PC 측 버그, 즉시 수정 가능** | 로봇 PC 측 (이 저장소) |

**정확히 이 두 관절 그룹(shoulder_pitch/shoulder_roll)만 증상이 있고 shoulder_yaw·elbow는
괜찮다는 사실 자체가 원인 #1을 강하게 뒷받침한다** — 아래 §3에서 설명.

---

## 2. `vr_control_watchdog_biarm.py` 구조 요약

- UDP(포트 5000)로 Unreal이 보내는 JSON을 수신 (`BodyTrackingUdpReceiver`).
  기대하는 스키마:
  ```json
  {"active": true, "joints": [
    {"name": "L_UpperArm", "pos": [x,y,z], "rot": [x,y,z,w]},
    {"name": "L_LowerArm", ...},
    {"name": "L_WristTwist", ...},
    {"name": "R_UpperArm", ...}, {"name": "R_LowerArm", ...}, {"name": "R_WristTwist", ...}
  ]}
  ```
  `pos`가 없거나 shape/finite 검증에 실패하면 **그 관절 전체(rot 포함)가 조용히 버려진다**
  (`core.py` 아님, `vr_control_watchdog_biarm.py:539-548`). `pos` 값 자체는 현재 리타게팅에
  쓰이지 않지만 **없으면 안 된다.**

- `ArmRetargeter`가 각 팔에 대해:
  1. `upper`/`lower`/`wrist` 쿼터니언을 EMA로 스무딩 (새 패킷이 올 때만 — `_smoothed_joints`).
  2. `'c'` 캘리브레이션 시점의 쿼터니언을 중립(neutral)으로 저장.
  3. 매 프레임 `upper_delta = neutral_upper⁻¹ · upper_current` 등 델타 쿼터니언 계산.
  4. **어깨만 별도로** twist-swing 분해(`shoulder_axis_components`): Y축(비틀림) 성분을
     먼저 정확히 분리하고, 나머지 swing 평면(roll↔shoulder_pitch, yaw↔shoulder_roll)에
     `suppress_minor_swing_axis`로 교차 누설 억제를 적용 — **단, CLI 기본값이
     `--swing-suppression-exponent 0.0`이라 기본 실행에서는 이 억제가 꺼져 있다.**
  5. `RETARGET_CONFIG`의 axis/sign/scale/max_motion으로 최종 라디안 값을 만들어
     motor position 명령으로 변환.

- 안전장치: UDP 0.5초 타임아웃 시 DAMPING, 명령-측정 오차 1.0 rad 초과 시 즉시 DAMPING
  (`send_targets`), `enter_position_from_current()`에서 URDF 리밋 밖 자세면 POSITION 진입 거부.

---

## 3. 원인 #1 — 어깨 관절이 몸통 회전에 오염됨 (언리얼 측 교차검증)

언리얼 측 저장소(`ciy-0518/unreal_hand-tracking-project`, `_teleop/` 디렉터리)를 확인한 결과,
**이미 같은 문제를 실측으로 확증**해 두었다:

- `_teleop/REPORT_02_tx_verification.md` §10.2 (T7 시험, **실제 VR 데이터**):
  팔을 몸에 고정한 채 몸통만 돌렸을 때, 현재 송신 방식에서 **어깨 명령 분산의 95.6%가
  몸통 요(yaw) 회전만으로 설명됨** (r = 0.978, 표준편차 54.67°). 가슴 기준으로 상대화하면
  r = −0.183, 표준편차가 6.6배 감소.
- 원인: Unreal이 보내는 `L_UpperArm`/`R_UpperArm`은 **트래킹 공간의 절대(world) 회전**이며
  가슴/몸통 기준으로 상대화되어 있지 않다 (`BodyTrackingSenderComponent.cpp` L487-488).
- 이 코드(`vr_control_watchdog_biarm.py`)의 `compute_deltas()`는 그 절대 회전을 그대로 받아
  `neutral_upper⁻¹ · upper_current`만 계산한다 — **몸통이 캘리브레이션 시점과 다른 방향을
  보고 있으면 그 차이가 통째로 어깨 명령에 섞여 들어간다.**
- `shoulder_axis_components()`가 3-DOF 어깨를 twist(=shoulder_yaw)/swing(=shoulder_pitch,
  shoulder_roll) 두 그룹으로 나누는데, **swing 평면(=motor 1,2,3,4)이 몸통 오염에 훨씬 더
  민감**하다 — twist 축(motor 5,6)은 이미 코드 자체 실측(`euler_probe_log`)으로 "깨끗하게
  분리됨"이 확인되어 있고, elbow/wrist(motor 7,8,9,10)는애초에 두 팔뚝 본 사이의 상대
  회전이라 몸통 회전에 원천적으로 불변이다.
  ⇒ **"1,2,3,4번만 이상하고 5번 이후는 괜찮다"는 사용자의 관찰과 정확히 일치한다.**

**결론: 이 문제의 근본 수정은 로봇 PC 코드가 아니라 Unreal 송신 측(가슴 기준 상대화)에서
이루어져야 한다.** 언리얼 팀은 이를 위해 이미 새 프로토콜(ICD v2, `_teleop/ICD_teleop_protocol_v2.md`)을
설계했고 구현 중이다 (자세한 내용과 로봇 PC 측 요청사항은 `docs/UNREAL_HANDOFF_INSTRUCTIONS.md` 참조).

---

## 4. 원인 #2 — 어깨 리밋 플레이스홀더 버그 (로봇 PC 측, 이 저장소)

`vr_control_watchdog_biarm.py:131-134`의 주석:

> "arm_config.yaml의 실측값(안전 여유 5% 적용됨)으로 교체. 기존엔 일반 URDF 모델의
> 이론값이 들어있었는데..."

그러나 바로 아래 `DEFAULT_URDF_LIMITS` 딕셔너리(L135-150)의 실제 값은 **모든 관절이
`(-10.0, 10.0)`** 이다 — 즉 **교체되지 않았다.** 반면 같은 저장소의 `arm_config.yaml`에는
실측 리밋이 있다:

| 관절 | 실측 리밋(rad) | `DEFAULT_URDF_LIMITS` 현재값 |
| --- | --- | --- |
| L shoulder_pitch | −0.565 ~ +0.484 | −10.0 ~ +10.0 |
| L shoulder_roll | 0.0 ~ +0.602 (**한쪽 방향만!**) | −10.0 ~ +10.0 |
| R shoulder_pitch | −0.560 ~ +0.651 | −10.0 ~ +10.0 |
| R shoulder_roll | −0.552 ~ 0.0 (**한쪽 방향만!**) | −10.0 ~ +10.0 |
| shoulder_yaw / elbow | ±1.5~1.7 (넓음) | −10.0 ~ +10.0 |

**어깨(pitch/roll)는 5관절 중 가동범위가 가장 좁고, 특히 roll은 편측(0~0.6rad)인데도**
코드는 사실상 무제한으로 취급한다. `RETARGET_CONFIG`의 `max_motion=2.5rad`(≈143°)와
결합하면, 원인 #1로 오염된 명령이 실제 기계적 가동범위를 4~5배 초과해서 명령될 수 있다.
결과:
- 어깨가 기계적 하드스톱에 부딪혀 갈리듯 움직이거나 멈춤 (부자연스러운 움직임의 직접 원인).
- `send_targets()`의 발산 워치독(1.0 rad 초과 시 즉시 DAMPING)이 튀어 팔 전체가 갑자기
  풀리는 현상으로 나타날 수 있음 — 이때 CAN/모터 하드웨어 이상으로 오인하기 쉬움.

**수정 방법 (이 저장소, 로봇 PC 측에서 독립적으로 가능):**
`load_urdf_limits()`가 `--urdf` 인자가 없을 때 폴백하는 `DEFAULT_URDF_LIMITS`를
`arm_config.yaml`의 실측값(5% 안전 여유 포함)으로 교체한다. 좌우 비대칭(shoulder_roll이
한쪽 방향만 열림)도 그대로 반영해야 한다. 이 수정은 언리얼 측 작업과 무관하게 지금 바로
적용 가능하며, 원인 #1이 고쳐지기 전에도 "하드스톱에 부딪혀 튀는" 증상을 줄여줄 수 있다
(단, 몸통 오염 자체는 해결하지 못한다).

---

## 5. 스코프가 아닌 것

언리얼 측 리포지토리 분석 중, `_teleop/HANDOFF.md` Phase 1 "발견 A"에서 **다른 로봇
(Dynamixel 6-DoF + 그리퍼, `COM4` 시리얼)을 전제로 한 별도의 구식 스크립트
(`vr_dual_arm_control2_ue_live.py`)**가 같은 저장소에 존재하며, 언리얼 측은 실제 로봇이
그 Dynamixel 암인지 Berkeley Humanoid Lite인지 아직 확답을 받지 못한 상태(미해결 결정 #1)임을
확인했다. **이 로봇 PC(Ubuntu 22.04, CAN, Berkeley Humanoid Lite, 5-DoF, 그리퍼 없음)가
실제 대상이며, 그 Dynamixel 스크립트의 버그(좌표계 거울상, 짐벌락 등)는 이 코드와 무관하다.**
이 답변은 `docs/UNREAL_HANDOFF_INSTRUCTIONS.md`에 포함해 전달한다.

---

## 6. 권장 다음 단계 (로봇 PC 측)

1. **즉시**: §4의 `DEFAULT_URDF_LIMITS`를 `arm_config.yaml` 실측값으로 교체 (독립적으로 가능).
2. **즉시 시도 가능**: Unreal이 이미 `Chest`/`L_Shoulder`/`R_Shoulder` 본을 기존 JSON(포트 5000)에
   추가 송신 중임이 확인됨(언리얼 측 REPORT_02 §D, 2026-09-11 재빌드 완료, 1,053개 프레임
   전수 수신 확인). ICD v2(바이너리, 포트 5010) 완성을 기다리지 않고, 이 코드에서
   `Chest` 쿼터니언을 받아 `upper_delta` 계산 전에 `quat_relative(chest, upper)`로
   가슴 기준 상대화를 선반영하는 **임시 완화책**을 테스트해볼 수 있다. 단 정확한 JSON
   필드명·좌표계 확인이 먼저 필요 — `docs/UNREAL_HANDOFF_INSTRUCTIONS.md` §7 참조.
3. **중기**: 언리얼 측 ICD v2(바이너리 UDP, 포트 5010/5011, 가슴 상대화 + One-Euro 필터 +
   ROS 좌표계 변환 내장)가 완성되면, 이 코드도 새 파서로 교체해야 한다. **이때
   `RETARGET_CONFIG`의 axis/sign 값은 재검증이 필요할 가능성이 높다** — 현재 값은 "가공되지
   않은 UE 원시 좌표"를 대상으로 경험적으로 튜닝된 값이기 때문이다 (§8 참조).
