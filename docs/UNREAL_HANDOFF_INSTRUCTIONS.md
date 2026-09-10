# 로봇 PC → 언리얼 팀(Claude Code) 전달 문서

**보내는 쪽:** 로봇 PC (Ubuntu 22.04, CAN, Berkeley Humanoid Lite 실물 로봇 구동)
저장소: `github.com/ciy-0518/ciy-berkeleyhumanoidlite`
대상 파일: `Berkeley-Humanoid-Lite-Lowlevel/vr_control_watchdog_biarm.py`

**받는 쪽:** 언리얼 프로젝트(`ciy-0518/unreal_hand-tracking-project`, `_teleop/`)를 개발 중인
VSCode Claude Code 세션.

**목적:** 그쪽 `_teleop/HANDOFF.md`의 "⏳ 미해결 결정 #1"(실제 로봇 사양)에 대한 확답,
그쪽이 이미 실측으로 찾아낸 "어깨 명령 몸통 오염" 문제가 실제 로봇에서도 정확히 같은 패턴으로
나타나고 있다는 독립적 확인, 그리고 ICD v2 전환을 위해 로봇 PC 측이 필요로 하는 것들의 목록.

이 문서는 그쪽의 `REPORT_0x` 문서들과 동일한 근거·서식(파일:줄번호, 실측 수치)을 따르려 했다.
`_teleop/HANDOFF.md`에 그쪽 세션이 이 내용을 반영해 기록해도 좋다.

---

## 1. 결론 요약 (바쁘면 이것만 읽어도 됨)

1. **미해결 결정 #1 답변: 실제 로봇은 Berkeley Humanoid Lite다.** Dynamixel 6-DoF+그리퍼
   (`vr_dual_arm_control2_ue_live.py`가 대상으로 하는 그 로봇)가 아니다. §2에 상세 스펙.
2. **`BodyTrackingSenderComponent.cpp` → UDP JSON → `192.168.0.65:5000`** 경로가 현재
   이 로봇을 실제로 구동하는 라이브 경로가 맞다 — 로봇 PC의 수신 스크립트
   (`vr_control_watchdog_biarm.py`)가 정확히 이 IP:포트에서 받고 있다.
3. **REPORT_02 §10.2의 T7 진단(어깨 명령 분산의 95.6%가 몸통 회전으로 설명, r=0.978)을
   실제 로봇 사용 중 관찰된 증상으로 독립 확인했다.** 로봇 PC 사용자가 "1,2,3,4번 모터
   (양팔 shoulder_pitch·shoulder_roll)만 부자연스럽고 5번 이후(shoulder_yaw, elbow)는
   괜찮다"고 보고했는데, 이는 로봇 PC 리타게팅 코드가 어깨만 twist/swing으로 분해하고
   swing 평면(=1,2,3,4번)이 몸통 오염에 가장 취약한 구조와 정확히 들어맞는다. §3 상세.
4. **Chest/L_Shoulder/R_Shoulder 본이 이미 기존 JSON 경로(포트 5000)에 라이브로 실려
   오고 있음을 확인했다** (REPORT_02 §D, 재빌드 완료·1,053프레임 전수 수신). 로봇 PC는
   ICD v2 완성 전에 이걸로 임시 완화책을 시도해보고 싶다 — 정확한 JSON 키/좌표계
   확인이 필요하다. §7.
5. **ICD v2(포트 5010/5011, 바이너리) 수신기는 로봇 PC 측이 구현할 준비가 되어 있다.**
   필요한 것: golden vector 확정본, One-Euro 파라미터 최종값, 그리고 좌표계 변경에 따른
   로봇 측 재보정이 필요하다는 점에 대한 상호 인지. §8.
6. **그리퍼는 이 로봇과 무관하다** (Berkeley Humanoid Lite 5-DoF 팔에는 그리퍼가 없음).
   §5.
7. **로봇 PC 측에서 별도로 발견한, 언리얼 측 조치가 필요 없는 로봇 PC 자체 버그**
   (어깨 소프트리밋 플레이스홀더)도 있음 — 혼동 방지를 위해 §6에 명시해 둔다.

---

## 2. 로봇 사양 확답 — `HANDOFF.md` 미해결 결정 #1

| 항목 | 값 |
| --- | --- |
| 로봇 | **Berkeley Humanoid Lite** (공식 저장소: `HybridRobotics/Berkeley-Humanoid-Lite`) |
| 팔 자유도 | **5-DoF/팔**, 그리퍼 **없음**, 손목 pitch/roll **없음** |
| 관절 순서(로봇 PC 코드 기준) | `shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch, elbow_roll` |
| 구동계 | **CAN (SocketCAN)**, 1 Mbit, `can0`=왼팔, `can1`=오른팔 (물리적으로 분리된 두 버스) |
| 모터 ID | 왼팔 `1,3,5,7,9` / 오른팔 `2,4,6,8,10` (관절 순서와 1:1 대응) |
| `wrist_yaw`(arm_config.yaml 명칭, ID 9/10) | 로봇 PC 코드의 `elbow_roll`과 **동일 모터** — 이름만 다름 |
| 로봇 PC OS | Ubuntu 22.04 LTS |
| 수신 스크립트 | `Berkeley-Humanoid-Lite-Lowlevel/vr_control_watchdog_biarm.py` |
| 현재 수신 포트 | UDP `0.0.0.0:5000` (Unreal이 `192.168.0.65:5000`으로 보낸 것을 수신) |
| 두 PC가 같은 LAN인가 | 예 (`192.168.0.x` 사설 대역, Tailscale 아님 — 그쪽 로그 분석과 일치) |

⇒ `_teleop/HANDOFF.md`의 "미해결 결정 #1"을 **해소**로 표시해도 된다.
`vr_dual_arm_control2_ue_live.py`(Dynamixel, `COM4`, 6-DoF+그리퍼)는 **이 로봇과 무관한
별개의(아마 이전 단계의) 스크립트**로 보인다 — 최소한 현재 이 로봇 PC를 구동하는 코드는
아니다.

---

## 3. 증상 교차검증 — 어깨(1,2,3,4번)만 나쁘고 나머지는 괜찮은 이유

로봇 PC의 `ArmRetargeter.compute_deltas()`(`vr_control_watchdog_biarm.py:639-698`)는
어깨와 그 외 관절을 **다른 방식**으로 계산한다:

- **어깨** (`shoulder_pitch`/`roll`/`yaw`, 모터 1,2,3,4,5,6): `upper_delta`
  (= `neutral_upper⁻¹ · upper_current`, `L_UpperArm`/`R_UpperArm`의 절대 회전을
  캘리브레이션 시점 기준으로만 상대화한 것)를 `shoulder_axis_components()`로
  twist(Y축, shoulder_yaw)와 swing(나머지, shoulder_pitch+shoulder_roll) 두 그룹으로
  분해한다.
- **팔꿈치/손목** (모터 7,8,9,10): `L_LowerArm`↔`L_UpperArm`, `L_WristTwist`↔`L_LowerArm`
  같은 **인접 본 사이의 상대 회전**만 쓴다.

**팔꿈치/손목 쪽은 두 본이 둘 다 팔에 붙어 있어서 몸통이 돌아도 그 상대 회전 자체는
변하지 않는다 — 구조적으로 몸통 오염에 불변이다.** 반면 어깨는 애초에 "월드 기준
절대 회전"에서 출발하므로 몸통이 돌면 그대로 오염된다. 코드 내부 실측(`euler_probe_log_*`,
`motor_log6.csv` 기반 docstring)도 twist(shoulder_yaw)는 "깨끗하게 분리됨"을 확인해뒀고,
swing 평면(shoulder_pitch↔shoulder_roll, 즉 1,2,3,4번)에서만 지속적인 교차 누설을
관측해 별도의 억제 로직(`suppress_minor_swing_axis`)까지 만들어 둔 상태다 — 다만 이 억제는
"두 swing 축끼리의 상호 누설"만 다루고, **몸통 회전이라는 제3의 오염원은 다루지 못한다.**

이건 정확히 그쪽 REPORT_02 §10.2(T7, r=0.978, 표준편차 54.67°→9.3° 상당의 6.6배 개선)와
같은 현상이 이 로봇에서도 재현되고 있다는 뜻이다. **로봇 PC 측 리타게팅 알고리즘을 아무리
고쳐도(twist-swing 분해, EMA, 억제 계수 등) 이 오염은 없앨 수 없다** — 오염이 델타 계산
"이전" 단계(Unreal이 절대 좌표를 보내는 단계)에서 이미 발생하기 때문이다.

---

## 4. 로봇 PC가 현재 기대하는 JSON 계약 (참고용)

포트 5000, 기존 JSON 경로 기준. `BodyTrackingUdpReceiver._parse_message`
(`vr_control_watchdog_biarm.py:523-555`)가 실제로 검증하는 내용:

```jsonc
{
  "active": true,
  "joints": [
    { "name": "L_UpperArm",   "pos": [x, y, z], "rot": [x, y, z, w] },
    { "name": "L_LowerArm",   "pos": [...],      "rot": [...] },
    { "name": "L_WristTwist", "pos": [...],      "rot": [...] },
    { "name": "R_UpperArm",   "pos": [...],      "rot": [...] },
    { "name": "R_LowerArm",   "pos": [...],      "rot": [...] },
    { "name": "R_WristTwist", "pos": [...],      "rot": [...] }
  ]
}
```

- `rot`은 `(x, y, z, w)` 순서(스칼라 마지막)로 파싱된다 — ICD v2 §3.1과 동일 규약.
- `pos`가 없거나 3성분이 아니거나 NaN/Inf면 **그 관절 전체(rot 포함)가 조용히 버려진다.**
  로봇 PC 코드는 현재 `pos` 값 자체는 리타게팅에 쓰지 않지만, **필드가 유효한 형태로는
  있어야 한다.**
- 필요한 6개 본 이름 외에 다른 이름(`Head`, `Chest`, `L_Shoulder`, `R_Shoulder` 등)이
  같이 와도 무시되며 문제되지 않는다 — 순수 추가는 항상 안전하다.
- `active: false`가 오거나 0.5초 이상 패킷이 없으면 로봇은 자동으로 DAMPING(팔 힘 빠짐)으로
  전환한다.

---

## 5. 스코프 아닌 것 — 그리퍼

`_teleop/HANDOFF.md`의 "그리퍼 배선 단절" 발견(발견 D)은 **이 로봇과 무관하다.**
Berkeley Humanoid Lite 5-DoF 팔에는 그리퍼가 없다(`arm_config.yaml`에 그리퍼 관절 없음).
그쪽에서 그리퍼를 후순위로 미룬 결정은 이 로봇 기준으로는 "영구히 스코프 아님"으로
바꿔도 무방하다 — 다만 그게 다른 로봇(Dynamixel 암)에도 여전히 유효한 프로젝트라면
그쪽 판단에 맡긴다.

---

## 6. 참고: 로봇 PC 측 자체 버그 (언리얼 측 조치 불필요)

`vr_control_watchdog_biarm.py`의 어깨 소프트리밋(`DEFAULT_URDF_LIMITS`)이 실측값과
동기화되지 않아 사실상 무제한(±10 rad)으로 동작하는 버그를 로봇 PC 측에서 발견해 별도로
수정 중이다 (`docs/VR_ARM_TRACKING_ANALYSIS_REPORT.md` §4 참조). **이건 순수 로봇 PC 측
문제이고 언리얼 송신 데이터와는 무관하다.** 다만 ICD v2 적용 후 테스트 중 어깨가 갑자기
DAMPING으로 풀리는 현상을 보게 되면, 그게 CAN/모터 하드웨어 문제가 아니라 이 리밋 버그
때문일 수 있다는 걸 참고하시라고 남겨둔다 (로봇 PC 측에서 먼저 고칠 예정).

---

## 7. 요청 #1 — Chest 본 임시 활용 (ICD v2 완성 전 임시 완화책)

REPORT_02 §D에서 `Chest`, `L_Shoulder`, `R_Shoulder` 본이 이미 재빌드된
`BodyTrackingSenderComponent.cpp`를 통해 **기존 JSON 경로(포트 5000)로 실제 라이브 송신
중**이고 1,053개 활성 프레임 전수 수신을 확인했다고 되어 있다. 로봇 PC 측은 ICD v2
완성/전환을 기다리지 않고, 이 `Chest` 쿼터니언으로 `L_UpperArm`/`R_UpperArm`을 미리
상대화(`chest⁻¹ · upper`)하는 **임시 완화책을 지금 바로 시험**해보고 싶다.
이를 위해 다음을 확인해서 알려주면 좋겠다:

1. 현재 JSON에서 `Chest` 본의 `rot` 값이 **어떤 좌표계**인가 — ICD v2에서 말하는 "UE
   왼손계 원시값 그대로"(`ros_converted` 미적용)가 맞는가? (REPORT_02 §D 기준으로는
   그렇게 보이지만, 재확인 요청)
2. `L_UpperArm`도 같은 좌표계(둘 다 UE 원시값)로 오는 게 맞는가 — 즉 로봇 PC가
   `quat_relative(chest_rot, upper_rot)`를 그대로 계산해도 되는가, 아니면 그 전에
   Unreal 쪽에서 이미 어떤 변환이 걸려 있는가?
3. 캘리브레이션(로봇 PC의 `'c'` 키) 시점에 팔뿐 아니라 **Chest도 같이 중립으로 저장**해야
   하는데, 이 방식이 그쪽이 설계 중인 ICD v2의 가슴 상대화 방식과 개념적으로 다른 점이
   있다면 미리 알려달라 (예: One-Euro 필터를 거친 가슴값이어야 하는지 등 — §8.2 참고).

**이 시험은 로봇 PC 측이 직접 수행하며, 언리얼 측 코드 변경을 요구하지 않는다.**
위 확인만 회신해주면 된다.

---

## 8. 요청 #2 — ICD v2 전환 대비 (`_teleop/ICD_teleop_protocol_v2.md` 검토 완료)

로봇 PC 측은 ICD v2 문서(포트 5010/5011, 1020B 고정 길이, 가슴 상대화 + One-Euro 필터 +
ROS 좌표 변환, seq/History 기반 손실 복구, CRC32, 워치독 규격)를 검토했고, **Python
수신기 구현을 맡을 준비가 되어 있다.** 진행을 위해 필요한 것:

### 8.1 산출물 공유 요청
- `_teleop/tools/golden_v2.bin` / `golden_v2.txt` / `verify_golden.py` — C++ 송신기가
  완성되는 대로 공유 요청. 로봇 PC의 Python 수신기를 이 골든 벡터로 바이트 단위 검증하겠다.
- 최종 확정된 One-Euro 파라미터(`min_cutoff`, `beta`) — 현재 §13에 `[미확인]`으로
  표시된 항목. 실측 튜닝 완료되면 공유 요청.
- ICD §13 항목 4(`history_count=2`의 충분성), 항목 5(복귀 램프 1.0초 적절성)의 최종
  결정값도 함께 공유해주면 로봇 PC 측 수신 램프 파라미터를 맞출 수 있다.

### 8.2 좌표계 전환에 따른 재보정 예고 (중요, 상호 인지 필요)
로봇 PC의 `RETARGET_CONFIG`(축 선택·부호·스케일)는 **현재 Unreal이 보내는 가공되지 않은
절대 좌표(UE 원시값)를 대상으로 경험적으로(실측 시행착오로) 튜닝된 값**이다. ICD v2가
가슴 상대화 + ROS 좌표 변환을 적용하면, 각 축이 나타내는 물리적 의미와 부호가 바뀔 수
있으므로 **로봇 PC 측은 ICD v2 전환 시 이 축/부호 테이블을 처음부터 재검증할 계획이다.**
이는 예상된 절차이며, 전환 직후 동작이 이전과 달라 보여도 "회귀"가 아니라 "재보정이
필요한 정상 과정"임을 미리 공유해둔다. ICD §2의 권고대로 **기존 JSON(5000) 경로와 새
ICD v2(5010) 경로를 당분간 병행 운용**하며 A/B로 비교할 예정이다.

### 8.3 로봇 PC 측이 만들 것
- ICD v2 바이너리 파서 (`struct` 포맷 `"<4sHHIQIffIfHHI"` 헤더 + `"<HH4f3f"` 본 + History
  + CRC32 trailer), `bone_id` 0~9 테이블 그대로 사용.
- 워치독: `now - last_rx > 100ms` → 홀드, `> 500ms` → 안전 자세 복귀, `dup_frame=1`은
  정상 수신으로 타이머 리셋, `chest_valid=0`이 200ms 이상 지속되면 `tracking_active=0`과
  동일 취급 — ICD §7(g), §8.1 그대로 구현 예정.
- ACK 송신(포트 5011, `"BHA2"` + `seq_ack` + `t_echo_ns`, 16B) — RTT 계측용.

**요청:** C++ 송신기 빌드/골든 벡터 검증이 끝나는 시점을 알려주면, 로봇 PC 측 수신기
작업을 그에 맞춰 시작하겠다.

---

## 9. 체크리스트 (언리얼 측 확인/회신 요청 항목만 모음)

- [ ] `_teleop/HANDOFF.md` 미해결 결정 #1을 "Berkeley Humanoid Lite, 5-DoF, CAN"으로 해소 표시
- [ ] §7의 `Chest`/`L_UpperArm` 좌표계 질문 3개 회신
- [ ] §8.1의 golden vector / One-Euro 최종 파라미터 / history_count·램프 시간 결정값 공유
      (완성되는 대로, 지금 당장이 아니어도 됨)
- [ ] ICD v2 C++ 송신기 빌드 완료 시점 공유 (로봇 PC 수신기 작업 착수 시점 조율용)
- [ ] (선택) 그리퍼 스코프를 "이 로봇 기준 영구 제외"로 처리할지 확인
