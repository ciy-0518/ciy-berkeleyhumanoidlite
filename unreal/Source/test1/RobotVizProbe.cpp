// RobotVizProbe.cpp
//
// 로봇 PC 워치독 --robot-viz 가 보내는 로봇 집게 위치·한계 상태를 VR 안에 그린다.
// 1·2단계 2026-10-21, 개선 2026-10-27 (솔리드 도형, 아래팔·손목 표시, 파랑 기준은 robot_viz.py 쪽),
// v3 2026-10-27 (쌍 매핑 반영 위치는 robot_viz.py 쪽 + 실제 크기 기준점 섞기, 곧은 아래팔, 패킷 사이 부드러운 움직임).
// 2026-10-30 (작업 지시 10): 인계 예외 비트 11 = 노랑, 팔꿈치 어긋남 토막 굵게 (솔리드 반지름 0.6, 선 두께 1.0).
// 2026-11-02 (작업 지시 11-b): 가대 면 제한 비트 13 = 빨강, 판·캔 자리·가상 캔 (아래 "판·캔 표시").
// 2026-11-03 (작업 지시 12-c): 가대 면 제한 비트 13 = 주황 (관절 한계·충돌은 빨강 그대로).
// 2026-11-03 (작업 지시 12-b): RobotVizCan 1 이상이면 판 표시도 켠다, 1초 상태 줄에 판 켬/끔·RVT1 받은 수·캔 번호·못 그리는 이유.
// 2026-11-03 (작업 지시 13): 워치독 --virtual-can 이 보내는 RVC1 을 받으면 고정 가상 캔 대신 실시간 자세로 그린다 (아래 "가상 캔 표시").
// 설계: ~/cc_analysis/REPORT_vr_reach_design_20261021_1707.md, 송신: _teleop/tools/robot_viz.py (패킷 v1, 364 바이트).
//
//   teleop.RobotViz 1 / 0            켜기 (UDP 5022 받기 시작) / 끄기
//   teleop.RobotVizStyle 1 / 0        1 = 솔리드 도형 (기본) / 0 = 디버그 선 (예비 — 솔리드 색이 안 바뀌거나 안 보일 때)
//   teleop.RobotVizRealSize -1/0/1    손 길이: -1 = 패킷 옵션을 따름 / 0 = 배율 크기 ÷k / 1 = 실제 크기 13.3 cm (기본)
//   teleop.RobotVizSmooth 1 / 0       패킷 사이 부드러운 움직임 (기본 켬) / 끄면 받은 위치 그대로
//   teleop.RobotVizSmoothMs <ms>      부드러운 움직임 시간 상수 (기본 40 ms)
//   teleop.RobotVizMeas 0 / 1         실제(엔코더) 위치를 옅은 작은 구로 (기본 끔, dry-run 에서는 값 없음)
//   teleop.RobotVizEcho 0 / 1         워치독이 받은 사람 손목 위치를 흰 작은 구로 (좌표 검증용, 기본 끔)
//   teleop.RobotVizTable 1 / 0        판과 캔 자리 원 테두리 (기본 끔, 워치독 --table 이 보내는 RVT1 이 있어야 그린다)
//   teleop.RobotVizCan <번호>         그 자리에 가상 캔 (0 = 끔, 1~4 왼팔 L1~L4, 5~8 오른팔 R1~R4, 9 판 위 인계 H1, 10 공중 인계 H2). 1 이상이면 판 표시도 켠다
//   teleop.RobotVizHideProps 1 / 0    판을 가리는 레벨 소품 숨기기 (기본 끔, 실행 중에만). 판 상자 ± 수평 10·위아래 100 cm 와 겹치는 StaticMesh. 바닥·벽·하늘 등은 뺀다
//   teleop.RobotVizHideBody 1 / 0     폰의 아바타 몸 메시 숨기기 (teleop.RobotViz 1 로 켤 때 기본 켬, 0 으로 끈다, 실행 중에만). 손 모델·컨트롤러는 그대로, 송신도 그대로
//   teleop.RobotVizListNear           판 근처 레벨 메시와 폰의 메시 목록을 로그에 (아무것도 바꾸지 않음)
//
// GripInputProbe.cpp 와 같은 방식: 콘솔 명령만 있고 블루프린트(VRPawn2)·레벨·BodyTrackingSender 를 건드리지 않는다.
// 새 UCLASS 가 없어 헤더가 없다. 모듈은 이미 test1.Build.cs 에 있는 것만 쓴다 (Sockets, Networking, Engine).
//
// 그리기 (수신·그리기는 월드 액터 틱이 끝난 뒤, FWorldDelegates::OnWorldPostActorTick):
//   솔리드 (기본): 표시를 켠 PIE 월드에 임시 액터 하나를 만들고 엔진 기본 도형(/Engine/BasicShapes/Cylinder·Cube·Sphere)의
//     StaticMeshComponent 를 붙여 매 틱 변환만 바꾼다. 충돌·그림자 끔.
//     색 = /Engine/BasicShapes/BasicShapeMaterial 의 동적 재질 인스턴스, 벡터 파라미터 "Color".
//     🔴 [미확인] 이 파라미터 이름은 로봇 PC 에서 확인할 수 없다. 색이 안 바뀌면 teleop.RobotVizStyle 0 으로 디버그 선을 쓴다.
//   디버그 선 (예비): DrawDebug* (BodyTrackingSenderComponent.cpp:541 과 같은 DrawDebugHelpers). 2026-10-26 VR 확인에서
//     헤드셋에는 "검은 심 + 색 테두리" 처럼 보였다 (PC 창은 정상).
//
// 좌표: 패킷 점 = 언리얼 원시 추적 좌표 cm (BodyTrackingSenderComponent.cpp:528 RawPosition 과 같은 틀, A 방식 역변환 끝).
//   월드 = 폰 액터 변환 · (원시 × PositionScale + VisualOffset) — BodyTrackingSenderComponent.cpp:529-531 과 같은 식.
//   PositionScale·VisualOffset 은 그 컴포넌트의 private UPROPERTY 라 리플렉션으로 읽는다 (없으면 1 / 0).
//
// 크기와 기준점 (2026-10-27 2227 녹화 재생 비교, ~/cc_analysis/REPORT_robot_viz_v3_*.md):
//   배율 크기 ÷k: 패킷 집게 끝·손목 그대로 (손 길이 13.3/k ≈ 23 cm).
//   실제 크기 (기본): 끝 = lerp(손목 + 13.3 cm × 손 축, 패킷 끝, w), 손목 = 끝 − 13.3 cm × 손 축 (w = 패킷 PairW / 255).
//     w = 0 (쌍 매핑 꺼짐) 은 손목 기준 → 표시 손목이 사람 손목에, 끝이 컨트롤러 앞끝 근처 (중앙값 2.5 / 3.0 cm).
//     w = 1 (맞댐) 은 끝 기준 → 두 끝이 로봇 실제 간격 1:1 (robot_viz.py 가 쌍 매핑을 반영해 옮긴 끝). 손목 기준만 쓰면 16.8 cm 벌어진다.
//     팔꿈치 점도 손목과 같은 양 옮긴다.
//
// 형태 (치수는 로봇 cm × Unit. Unit = 배율 크기면 사람 cm/로봇 cm (÷k), 실제 크기면 1 — 둘 다 폰 배율 곱함):
//   | 부분        | 모양 | 위치                                   | 크기                                                    | 색        |
//   | 아래팔      | 원통 | 손목 점 → 손 축 반대로 곧게, 길이 = 팔꿈치 점을 손 축에 투영한 거리 | 반지름 3 (충돌 모델 FOREARM_RADIUS 와 같음) | 옅은 중립색 |
//   | 팔꿈치 관절 | 구   | 팔꿈치 점                              | 반지름 1.5                                              | 옅은 중립색 |
//   | 팔꿈치 어긋남 | 원통 | 팔꿈치 점 → 아래팔 선 위 가장 가까운 점 (실물 약 2.5 cm 옆 어긋남) | 반지름 0.6         | 더 옅은 색 |
//   | 손목        | 구   | 손목 점                                | 반지름 2                                                | 상태색    |
//   | 그리퍼 몸체 | 상자 | 손목 → 집게 뿌리 (손 축 7.8 = 13.3 − 5.5) | 벌어지는 축 방향 5.7 (평판 폭) × 두께 5.0 (평판 두께)     | 상태색    |
//   | 집게 2개    | 상자 | 집게 뿌리 → 끝, 벌어지는 축 ± 반 간격  | 길이 5.5 × 두께 0.8 (벌어지는 축) × 폭 1.7; 반 간격 열림 4.2 / 닫힘 0.5 | 상태색 |
//   | 끝          | 구   | 집게 끝                                | 반지름 0.8                                              | 상태색    |
//   몸체 치수 근거 (REPORT_54 CAD 표): 그리퍼 좌우 폭 평판 5.7·링크 4.5, 두께 7.0 (= 평판 5.0 + 집게). 평판·서보가 축에서 한쪽으로
//   2.4~2.8 치우치지만 그 방향이 집게 축 기준 어느 쪽인지는 [미확인] 이라 가운데에 둔다 (두께 5.0 = 평판만).
//   집게 치수: 길이 5.5 (REPORT_54:387), 끝 하나 폭 0.8 · 닫힌 두 끝 전체 폭 1.7 (REPORT_54:398), 열린 안쪽 최대 8.4 (REPORT_54:445).
//
// 색 규칙 (팔마다, 패킷 flags 비트 — robot_viz.py F_*). 위에서부터 차례로, 하나라도 맞으면 그 색:
//   | 색   | 조건                                                                   | 비트              |
//   | 회색 | 0.5 초 넘게 패킷 없음, 또는 추적이 끊겨 점이 없어 마지막 위치를 그대로 둔 것 | —                 |
//   | 빨강 | 관절 한계 잘림 · 충돌 방지 · 비틀기 멈춤 · 등 뒤 넘김 · 너무 가까이 접음            | 0, 3, 4, 6, 10    |
//   | 주황 | 가대 면 제한 (손 목표를 판 위로 올림 · 모터가 판 쪽을 막음). 빨강 다음 우선 | 13                |
//   | 파랑 | 끝까지 뻗음 (손 목표 ≥ rmax − 4 cm — 임계거리 끝, 정상적인 끝이라 빨강과 나눈다) | 2           |
//   | 노랑 | 속도 뒤처짐 · 손 다시 맞춤 · 손목 유지 · 한계 근접 · 쌍 매핑 · 인계 예외      | 1, 5, 7, 8, 9, 11 |
//   | 초록 | 그 밖                                                                  | —                 |
//   (충돌 방지·비틀기 멈춤 카운터는 워치독에서 양팔 합이라 두 팔이 같이 켜진다.)
//   추적이 끊겨 집게 점이 NaN 으로 오면 마지막으로 그린 위치를 회색으로 계속 그린다. 새 보정(calib_id 변경)이 오면 지운다.
//
// 부드러운 움직임 (2026-10-27): 몸 추적 약 14 Hz, 패킷 30 Hz, 헤드셋 72~90 Hz 라 받은 위치를 그대로 그리면 뚝뚝 끊긴다.
//   렌더 프레임마다 그릴 위치를 받은 위치 쪽으로 지수로 따라간다: a = 1 − exp(−dt / τ), τ 기본 40 ms (프레임 속도와 무관).
//   두 패킷 사이 선형 보간 대신 이것을 고른 까닭: 패킷 내용이 바뀌는 간격이 일정하지 않고 (14 Hz 몸 프레임이 30 Hz 패킷에 2~3번
//   반복), 무선·Tailscale 경로라 패킷 간격도 흔들린다. 선형 보간은 두 패킷을 기다려야 하고 (한 패킷 늦음) 같은 내용 반복에서
//   멈췄다 가기를 되풀이한다. 지수 따라가기는 버퍼가 없고, 일정한 속도에서 늦음 = τ (40 ms) 이다.
//   바로 옮기는 경우 (따라가지 않음): 처음, 새 보정 (calib_id 변경), 집게 끝이나 손목이 20 cm 넘게 뛸 때, 손 축이 90° 넘게 돌 때,
//   회색 ↔ 색 전환, 끈 상태.
//
// 판·캔 표시 (2026-11-02, 작업 지시 11-b). 값은 RVT1 (1 Hz, 로봇 base cm, table_limit.py RVT_HDR·RVT_SLOT), 변환은 매 틱 팔 패킷의 Range* 로 한다 — 그리퍼 끝과 같은 변환:
//   Raw(팔, p) = RangeOrigin + h.X·RangeFwd − h.Y·RangeRight + h.Z·RangeUp, h = (p − Ps) × RangeScale (robot_viz.py P 와 같은 식)
//   판 네 모서리: 두 팔 Raw 의 평균 (쌍 매핑 가운데 식) → 크기도 배율대로 (실물 60 × 40 이 사람 공간에서 약 105 × 70).
//   한 팔 자리: 그 팔의 Raw. 실제 크기 손은 끝 = Raw(손목) + 13.3·축 이라 후보 자세의 손 축으로 맞춘다: 잡는 점 = Raw(c − (13.3 − d)·축) + (13.3 − d)·축·Unit.
//   인계 자리: 두 팔 Raw 의 평균 (w = 1 일 때 두 끝 가운데와 같은 식).
//   캔·띠·원은 실물 크기 × Unit. 실제 크기에서는 판을 위로 (K − Unit)·잡는 높이 올려 캔 밑면이 판에 닿게 그린다.
//   후보 자세와 다른 손 방향·쌍 매핑 중 한 팔 자리·두 팔 배율 차이로 생기는 어긋남은 ~/cc_analysis/REPORT_table_b_*.md.
//
// 가상 캔 표시 (2026-11-03, 작업 지시 13). 값은 RVC1 (30 Hz, 56 바이트, 로봇 base cm, tools/virtual_can.py). 계산은 파이썬, 여기서는 그리기만:
//   시작 자리에 서 있음: 그 자리 원과 같은 위치 (DrawTable 의 Base). 잡힘: 그린 그리퍼 끝 + (캔 밑면 − 집게 끝) × Unit — 그리퍼에 딱 붙어 보인다.
//   옮겨 서 있음·넘어짐: 판과 같은 변환 (두 팔 Raw 평균 + 판 올림) + 놓는 순간의 수평 어긋남 (판 틀로 저장). 판 그림 위에 놓이게 높이 어긋남은 뺀다.
//     판 변환만 쓰지 않는 까닭: 한 팔 자리 원이 판 변환에서 4~12 cm 떨어져 있어 (REPORT_table_b) 그리퍼와 캔이 어긋나 보인다.
//   상태가 바뀔 때 튀는 양은 0.12 초에 걸쳐 줄인다 (그리퍼를 늦게 따라가지 않게 위치 자체는 부드럽게 하지 않는다).
//   놓기 가능·목표 허용 (작업 지시 13-d): RVC1 Place 비트 1 (지금 다 놓으면 섬) 이면 잡힌 캔 밑면에 초록 고리, 비트 2 (목표 허용 범위 안) 면 목표 원을 밝은 색으로.
//     1초 상태 줄에 마지막 놓기 결과 (섬 / 넘어짐: 높이·기울기·속도·판 밖) 와 목표까지 거리. RVC1 은 13-d 에 56 → 60 바이트 (다른 칸의 뜻을 바꾸지 않으려고).
//   목표 자리 (작업 지시 13-c): RVC1 flags 위 4비트 = 목표 자리 번호 (1~, 0 없음). 그 자리 원 바깥에 초록 굵은 원. 시작 자리 원은 그대로.
//   판 뚫기 (작업 지시 13-c): 파이썬이 판 아래로 들어간 깊이만큼 올린 위치를 보내므로 여기서는 그대로 그린다 (그리퍼와 떨어져 판 위에 멈춰 보인다).
//   잡힘 미리 보기 (작업 지시 13-b): flags 4·8 (왼팔·오른팔이 지금 닫으면 잡힘) 이면 캔 위·아래 끝에 노란 테두리. 인계 중이면 둘째 손 기준 (판정은 파이썬).
//   색: 서 있음 은회색, 잡힘 청록, 넘어짐 어두운 빨강, 부딪힘(bump) 분홍. 1초 안에 RVC1 이 없으면 teleop.RobotVizCan 의 고정 캔으로 돌아간다.

#include "CoreMinimal.h"
#include "BodyTrackingSenderComponent.h"
#include "Common/UdpSocketBuilder.h"
#include "Components/MeshComponent.h"
#include "Components/SceneComponent.h"
#include "Components/SkinnedMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "Engine/SkinnedAsset.h"
#include "Engine/StaticMesh.h"
#include "EngineUtils.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "IPAddress.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "UObject/UnrealType.h"   // FindFProperty, FFloatProperty, FStructProperty

namespace RobotVizProbe
{
	constexpr int32 Port = 5022;
	constexpr int32 PacketSize = 364;
	constexpr double StaleSec = 0.5;

	enum : uint32
	{
		FJointClamp = 1u << 0, FSpeedLag = 1u << 1, FReachFar = 1u << 2, FCollision = 1u << 3, FYawHold = 1u << 4,
		FRefit = 1u << 5, FFlipHold = 1u << 6, FWristHold = 1u << 7, FNearLimit = 1u << 8, FPair = 1u << 9,
		FReachNear = 1u << 10, FHandover = 1u << 11, FTable = 1u << 13
	};
	constexpr uint32 RedMask = FJointClamp | FCollision | FYawHold | FFlipHold | FReachNear;
	constexpr uint32 OrangeMask = FTable;
	constexpr uint32 BlueMask = FReachFar;
	constexpr uint32 YellowMask = FSpeedLag | FRefit | FWristHold | FNearLimit | FPair | FHandover;
	constexpr uint8 StCalib = 16;
	constexpr uint16 OptRealSize = 1;

	// 형태 치수 (로봇 cm) — 머리 주석 표
	constexpr float HandLen = 13.3f;        // 손목 관절 → 집게 끝 (ROBOT_GRIP_M)
	constexpr float JawLen = 5.5f;          // 집게 길이
	constexpr float ForearmRadius = 3.0f;   // 아래팔 원통 (충돌 모델과 같음)
	constexpr float WristRadius = 2.0f;
	constexpr float BodyWidth = 5.7f;       // 그리퍼 몸체: 벌어지는 축 방향 (평판 폭)
	constexpr float BodyThick = 5.0f;       // 그리퍼 몸체: 셋째 축 방향 (평판 두께)
	constexpr float JawThick = 0.8f;        // 집게: 벌어지는 축 방향
	constexpr float JawWidth = 1.7f;        // 집게: 셋째 축 방향
	constexpr float JawHalfOpen = 4.2f;     // 반 간격 (열림)
	constexpr float JawHalfClosed = 0.5f;   // 반 간격 (닫힘)
	constexpr float TipRadius = 0.8f;
	constexpr float ElbowRadius = 1.5f;     // 팔꿈치 관절 구
	constexpr float OffsetRadius = 0.6f;    // 팔꿈치 → 아래팔 선 토막 (2026-10-30: 0.5 → 0.6, 잘 안 보여서)
	constexpr float OffsetLineThick = 1.0f; // 디버그 선 방식의 같은 토막 두께 (2026-10-30: 0.2 → 1.0)
	constexpr float SnapCm = 20.0f;         // 부드러운 움직임: 이보다 많이 뛰면 바로 옮김 (폰 배율 곱함)

#pragma pack(push, 1)
	struct FArm   // 172 바이트 — robot_viz.py ARM = "<" + "3f"*9 + "fIBB2x" + "3f"*4 + "f"
	{
		float TipCmd[3]; float TipMeas[3]; float Wrist[3]; float Elbow[3];
		float Axis[3]; float Jaw[3]; float HumanWristEcho[3]; float TipRobotCmd[3]; float TipRobotMeas[3];
		float PronDeg; uint32 Flags; uint8 Grip; uint8 PairW; uint8 Pad[2];   // PairW = 쌍 매핑 비율 × 255 (2026-10-27, 예전 Pad[0])
		float RangeOrigin[3]; float RangeFwd[3]; float RangeRight[3]; float RangeUp[3]; float RangeScale;
	};
	struct FPacket  // 364 바이트 — robot_viz.py HDR = "<4sIdBBH" + ARM × 2 (왼, 오른)
	{
		char Magic[4]; uint32 Seq; double TMono; uint8 State; uint8 CalibId; uint16 Options;
		FArm Arm[2];
	};
#pragma pack(pop)
	static_assert(sizeof(FArm) == 172, "FArm must be 172 bytes (robot_viz.py ARM)");
	static_assert(sizeof(FPacket) == PacketSize, "FPacket must be 364 bytes (robot_viz.py PACKET_SIZE)");

#pragma pack(push, 1)
	struct FTableHdr   // 76 바이트 — table_limit.py RVT_HDR = "<4sIHH" + "7f" + "3f" + "3f3f". 값 cm, 로봇 base
	{
		char Magic[4]; uint32 Seq; uint16 NumSlots; uint16 Reserved;
		float X0; float X1; float Y0; float Y1; float Z0; float Z1; float Thick;
		float CanDia; float CanHeight; float GripH;
		float PsL[3]; float PsR[3];
	};
	struct FTableSlot  // 48 바이트 — table_limit.py RVT_SLOT = "<4s" + "3f" + "3f" + "f" + "BB2x" + "3f"
	{
		char Name[4]; float Pos[3]; float HandAxis[3]; float TipPast;
		uint8 Kind; uint8 ArmId; uint8 Pad[2];      // Kind 0 한 팔, 1 판 위 인계, 2 공중 인계. ArmId 0 왼, 1 오른, 2 둘 다
		float GripH1; float GripH2; float Bottom;   // 잡는 높이 (캔 밑면에서, 0 = 없음), 캔 밑면이 판 위 몇 cm
	};
#pragma pack(pop)
	static_assert(sizeof(FTableHdr) == 76, "FTableHdr must be 76 bytes (table_limit.py RVT_HDR)");
	static_assert(sizeof(FTableSlot) == 48, "FTableSlot must be 48 bytes (table_limit.py RVT_SLOT)");
	constexpr int32 MaxSlots = 12;
	constexpr int32 RingSegs = 12;

#pragma pack(push, 1)
	struct FCanPacket   // 60 바이트 — virtual_can.py RVC = "<4sI" + "3f" + "4f" + "3f" + "BBBB" + "HH" + "BBH". 값 cm, 로봇 base
	{
		char Magic[4]; uint32 Seq;
		float Pos[3]; float Quat[4]; float AnchorTip[3];    // 캔 밑면 가운데, 캔 틀 → base (x, y, z, w; 캔 틀 z = 캔 축), 기준 팔 집게 끝
		uint8 State; uint8 Flags; uint8 Anchor; uint8 Slot; // State 아래 ECan, Flags 아래, Anchor 0 왼 / 1 오른 / 255 없음, Slot 시작 자리에 그대로면 번호 1~
		uint16 Episode; uint16 Bumps;
		uint8 Place; uint8 LastWhy; uint16 GoalDistMm;      // 13-d: Place 아래 CanPlace*, LastWhy 아래 CanWhy* (0 아직 없음), 목표까지 mm (0xFFFF 없음)
	};
#pragma pack(pop)
	static_assert(sizeof(FCanPacket) == 60, "FCanPacket must be 60 bytes (virtual_can.py RVC)");
	enum ECan : uint8 { CanOnTable, CanHeldL, CanHeldR, CanHeldBoth, CanFallen, CanStateNum };
	enum : uint8 { CanFlagActive = 1, CanFlagBump = 2, CanFlagReadyL = 4, CanFlagReadyR = 8 };
	constexpr int32 CanGoalShift = 4;     // Flags 위 4비트 = 목표 자리 번호 (virtual_can.py GOAL_SHIFT)
	enum : uint8 { CanPlaceOk = 1, CanInGoal = 2 };                                                      // virtual_can.py PLACE_OK, IN_GOAL
	enum : uint8 { CanWhyHeight = 1, CanWhyTilt = 2, CanWhySpeed = 4, CanWhyOff = 8, CanWhyStood = 16 };   // virtual_can.py WHY_*
	constexpr double CanStaleSec = 1.0;
	constexpr float CanFixTau = 0.12f;      // 상태가 바뀔 때 튀는 양을 줄이는 시간 상수 (초)

	FSocket* Socket = nullptr;
	FDelegateHandle PostTickHandle;
	FDelegateHandle CleanupHandle;
	FPacket Last;
	bool bHave = false;
	double LastRxSec = -1.0;
	double LastLogSec = -1.0;
	uint64 RxCount = 0;
	uint64 RxCountAtLog = 0;
	uint64 BadCount = 0;
	int32 RealSizeMode = 1;    // -1 패킷을 따름, 0 배율, 1 실제 (기본 — 머리 주석 "크기와 기준점")
	bool bSmooth = true;       // 패킷 사이 부드러운 움직임 (머리 주석)
	float SmoothMs = 40.0f;    // 그 시간 상수
	int32 StyleMode = 1;       // 1 솔리드 (기본), 0 디버그 선
	bool bShowMeas = false;
	bool bShowEcho = false;
	FArm LastGood[2];          // 팔마다 마지막으로 점이 있던 패킷 (추적 끊김 때 회색으로 계속 그림)
	bool bHaveGood[2] = { false, false };
	uint8 GoodCalibId = 0;
	FTableHdr TableHdr;        // 마지막 RVT1
	FTableSlot TableSlots[MaxSlots];
	bool bHaveTable = false;
	uint64 TableRx = 0;
	bool bShowTable = false;   // teleop.RobotVizTable
	int32 CanSlot = 0;         // teleop.RobotVizCan (0 끔, 1~)
	FCanPacket CanPkt;         // 마지막 RVC1
	bool bHaveCan = false;
	double CanRxSec = -1.0;
	uint64 CanRx = 0;
	FVector SlotBase[MaxSlots];   // DrawTable 이 매 틱 구한 자리별 캔 밑면 (월드)
	bool bSlotBaseKnown = false;
	bool bLiveShown = false;      // 지난 틱에 그렸다
	FVector LiveBase = FVector::ZeroVector;   // 그 위치 (튀는 양 포함)
	FVector LiveFix = FVector::ZeroVector;    // 줄여 가는 튀는 양
	uint8 LiveLastState = 0;
	uint16 LiveLastEpisode = 0;
	bool bLastHeld = false;       // 지난 틱에 잡혀 있었다 → 놓는 순간 어긋남을 저장
	FVector LastHeldP = FVector::ZeroVector, LastHeldBase = FVector::ZeroVector;
	FVector PlaceOff = FVector::ZeroVector;   // 놓는 순간의 수평 어긋남 (판 틀 앞·오른쪽 성분)
	bool bPlateKnown = false;  // 판 네 모서리의 월드 위치를 안다 (DrawTable 이 매 틱 갱신, 판 표시를 꺼도)
	FBox PlateBox(ForceInit);  // 그 모서리를 감싸는 월드 상자 (teleop.RobotVizHideProps·ListNear 가 쓴다)

	FVector V(const float* P) { return FVector(P[0], P[1], P[2]); }
	bool IsNaN3(const float* P) { return FMath::IsNaN(P[0]) || FMath::IsNaN(P[1]) || FMath::IsNaN(P[2]); }

	void Poll()
	{
		if (!Socket) return;
		uint8 Buffer[1024];
		uint32 Pending = 0;
		int32 Guard = 0;
		TSharedRef<FInternetAddr> From = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
		// TeleopSenderComponent.cpp:297 PollAck 와 같은 폴링 — 쌓인 것 중 마지막 하나만 쓴다
		while (Socket->HasPendingData(Pending) && Guard++ < 64)
		{
			int32 Read = 0;
			if (!Socket->RecvFrom(Buffer, sizeof(Buffer), Read, *From)) break;
			if (Read == PacketSize && FMemory::Memcmp(Buffer, "RVZ1", 4) == 0)
			{
				FMemory::Memcpy(&Last, Buffer, PacketSize);
				bHave = true;
				LastRxSec = FPlatformTime::Seconds();
				++RxCount;
			}
			else if (Read == (int32)sizeof(FCanPacket) && FMemory::Memcmp(Buffer, "RVC1", 4) == 0)
			{
				FMemory::Memcpy(&CanPkt, Buffer, sizeof(CanPkt));
				if (!bHaveCan)
				{
					bShowTable = true;      // 판 없이는 캔을 그리지 않는다. 처음 한 번만 켠다 (그 뒤 teleop.RobotVizTable 0 은 따른다)
					UE_LOG(LogTemp, Display, TEXT("[RobotViz] RVC1 받음 — 가상 캔을 실시간 자세로 그린다 (판 표시도 켰다)"));
				}
				bHaveCan = true;
				CanRxSec = FPlatformTime::Seconds();
				++CanRx;
			}
			else if (Read >= (int32)sizeof(FTableHdr) && FMemory::Memcmp(Buffer, "RVT1", 4) == 0)
			{
				FTableHdr H;
				FMemory::Memcpy(&H, Buffer, sizeof(H));
				if (H.NumSlots <= MaxSlots && Read == (int32)(sizeof(FTableHdr) + H.NumSlots * sizeof(FTableSlot)))
				{
					TableHdr = H;
					FMemory::Memcpy(TableSlots, Buffer + sizeof(FTableHdr), H.NumSlots * sizeof(FTableSlot));
					bHaveTable = true;
					++TableRx;
				}
				else
				{
					++BadCount;
				}
			}
			else
			{
				++BadCount;
			}
		}
	}

	// 원시 → 월드 (BodyTrackingSender 와 같은 식). 폰이 없으면 false.
	bool RawToWorld(UWorld* World, FTransform& OutPawn, float& OutScale, FVector& OutOffset)
	{
		OutScale = 1.0f;
		OutOffset = FVector::ZeroVector;
		APlayerController* PC = World->GetFirstPlayerController();
		APawn* Pawn = PC ? PC->GetPawn() : nullptr;
		if (!Pawn) return false;
		if (UBodyTrackingSenderComponent* C = Pawn->FindComponentByClass<UBodyTrackingSenderComponent>())
		{
			if (FFloatProperty* P = FindFProperty<FFloatProperty>(C->GetClass(), TEXT("PositionScale")))
			{
				OutScale = P->GetPropertyValue_InContainer(C);
			}
			if (FStructProperty* P = FindFProperty<FStructProperty>(C->GetClass(), TEXT("VisualOffset")))
			{
				OutOffset = *P->ContainerPtrToValuePtr<FVector>(C);
			}
		}
		OutPawn = Pawn->GetActorTransform();
		return true;
	}

	enum class EColor : uint8 { Grey, Red, Orange, Blue, Yellow, Green };

	EColor ColorKind(uint32 Flags, bool bStale)
	{
		if (bStale) return EColor::Grey;
		if (Flags & RedMask) return EColor::Red;
		if (Flags & OrangeMask) return EColor::Orange;
		if (Flags & BlueMask) return EColor::Blue;
		if (Flags & YellowMask) return EColor::Yellow;
		return EColor::Green;
	}

	const TCHAR* ColorName(EColor C)
	{
		switch (C)
		{
		case EColor::Grey: return TEXT("회색");
		case EColor::Red: return TEXT("빨강");
		case EColor::Orange: return TEXT("주황");
		case EColor::Blue: return TEXT("파랑");
		case EColor::Yellow: return TEXT("노랑");
		default: return TEXT("초록");
		}
	}

	FColor ColorOf(uint32 Flags, bool bStale)
	{
		switch (ColorKind(Flags, bStale))
		{
		case EColor::Grey: return FColor(128, 128, 128);
		case EColor::Red: return FColor(255, 30, 30);
		case EColor::Orange: return FColor(255, 130, 0);
		case EColor::Blue: return FColor(40, 120, 255);
		case EColor::Yellow: return FColor(255, 200, 0);
		default: return FColor(30, 220, 60);
		}
	}

	const FColor NeutralColor(190, 190, 200);   // 아래팔 (상태색 아님)
	const FColor MeasColor(150, 170, 255);
	const FColor EchoColor(255, 255, 255);
	const FColor OffsetColor(225, 225, 235);    // 팔꿈치 어긋남 토막 (중립색보다 옅게)

	// 두 그리기 방식이 같이 쓰는 형상 (월드 좌표)
	struct FArmGeom
	{
		FVector Elbow, Wrist, Root, Tip, Axis, Jaw, Side;   // Side = Axis × Jaw (셋째 축)
		FVector ForeEnd;                                    // 곧은 아래팔의 팔꿈치 쪽 끝 = 팔꿈치 점을 손 축 선에 투영한 점
		FQuat Rot;                                          // 로컬 Z = 손 축, X = 벌어지는 축, Y = 셋째 축
		float Unit = 1.0f;                                  // 로봇 1 cm 를 그릴 길이
		float PawnScale = 1.0f;                             // 사람(원시) 1 cm 를 그릴 길이
		float JawHalf = JawHalfOpen;
		bool bHasElbow = false;
		bool bHasMeas = false, bHasEcho = false;
		FVector Meas, Echo;
		float EchoUnit = 1.0f;
	};

	bool MakeGeom(const FArm& A, const FTransform& T, float Scale, const FVector& Offset, bool bRealSize, FArmGeom& G)
	{
		if (IsNaN3(A.TipCmd) || IsNaN3(A.Wrist) || IsNaN3(A.Axis)) return false;
		auto ToWorld = [&](const float* P) { return T.TransformPosition(V(P) * Scale + Offset); };
		G.Wrist = ToWorld(A.Wrist);
		G.Axis = T.TransformVectorNoScale(V(A.Axis)).GetSafeNormal();
		G.Jaw = T.TransformVectorNoScale(V(A.Jaw)).GetSafeNormal();
		// 배율 크기: 패킷의 집게 끝(÷k) 그대로, 길이 단위 = 사람 cm / 로봇 cm (RangeScale) × 폰 배율
		// 실제 크기: 끝 = lerp(손목 + 13.3 축, 패킷 끝, w), 손목 = 끝 − 13.3 축, 치수도 로봇 그대로 (× 폰 배율만) — 머리 주석
		const float PawnScale = Scale * T.GetScale3D().X;
		const float K = (A.RangeScale > 0.0f && !FMath::IsNaN(A.RangeScale) ? A.RangeScale : 1.0f) * PawnScale;
		G.PawnScale = PawnScale;
		G.Unit = bRealSize ? PawnScale : K;
		G.bHasElbow = !IsNaN3(A.Elbow);
		if (G.bHasElbow) G.Elbow = ToWorld(A.Elbow);
		const FVector TipPacket = ToWorld(A.TipCmd);
		if (bRealSize)
		{
			const float W = A.PairW / 255.0f;
			const FVector TipFromWrist = G.Wrist + G.Axis * (HandLen * G.Unit);
			G.Tip = TipFromWrist + (TipPacket - TipFromWrist) * W;
			const FVector Move = (G.Tip - G.Axis * (HandLen * G.Unit)) - G.Wrist;
			G.Wrist += Move;
			if (G.bHasElbow) G.Elbow += Move;              // 팔꿈치도 손목과 같은 양
		}
		else
		{
			G.Tip = TipPacket;
		}
		G.JawHalf = (A.Grip ? JawHalfClosed : JawHalfOpen) * G.Unit;
		G.bHasMeas = !IsNaN3(A.TipMeas);
		if (G.bHasMeas) G.Meas = ToWorld(A.TipMeas);
		G.bHasEcho = !IsNaN3(A.HumanWristEcho);
		if (G.bHasEcho) G.Echo = ToWorld(A.HumanWristEcho);
		G.EchoUnit = PawnScale;
		return true;
	}

	// 위치·방향(Tip, Wrist, Elbow, Axis, Jaw) 에서 나머지(Root, Side, Rot, ForeEnd) 를 만든다. 부드럽게 한 뒤에도 부른다.
	void FinishGeom(FArmGeom& G)
	{
		G.Axis = G.Axis.GetSafeNormal();
		if (G.Jaw.IsNearlyZero()) G.Jaw = FVector::CrossProduct(G.Axis, FVector::UpVector).GetSafeNormal();
		G.Jaw = (G.Jaw - G.Axis * FVector::DotProduct(G.Jaw, G.Axis)).GetSafeNormal();   // 손 축에 수직으로
		G.Side = FVector::CrossProduct(G.Axis, G.Jaw);
		G.Rot = FRotationMatrix::MakeFromZX(G.Axis, G.Jaw).ToQuat();
		G.Root = G.Tip - G.Axis * (JawLen * G.Unit);
		// 곧은 아래팔: 손목에서 손 축 반대로, 길이 = 팔꿈치 점을 손 축에 투영한 거리 (음수면 0)
		const float ForeLen = G.bHasElbow ? FMath::Max((float)FVector::DotProduct(G.Elbow - G.Wrist, -G.Axis), 0.0f) : 0.0f;
		G.ForeEnd = G.Wrist - G.Axis * ForeLen;
	}

	// ---------------------------------------------------------------- 부드러운 움직임 (머리 주석)
	FArmGeom Shown[2];
	bool bShownValid[2] = { false, false };
	bool bShownGrey[2] = { false, false };
	uint8 ShownCalibId[2] = { 0, 0 };
	double LastDrawSec = -1.0;

	// G = 이번 패킷으로 만든 형상 (FinishGeom 전). 반환 = 그릴 형상 (FinishGeom 끝).
	const FArmGeom& SmoothGeom(int32 S, const FArmGeom& G, bool bGrey, uint8 CalibId, float Dt)
	{
		FArmGeom& M = Shown[S];
		const float Snap = SnapCm * G.PawnScale;
		const bool bJump = !bShownValid[S] || !bSmooth || bShownGrey[S] != bGrey || ShownCalibId[S] != CalibId
			|| FVector::Dist(M.Tip, G.Tip) > Snap || FVector::Dist(M.Wrist, G.Wrist) > Snap
			|| FVector::DotProduct(M.Axis, G.Axis) < 0.0;    // 손 축이 90° 넘게 돌면 섞는 중간이 0 벡터가 될 수 있어 바로 옮김
		if (bJump)
		{
			M = G;
		}
		else
		{
			const float A = 1.0f - FMath::Exp(-Dt / FMath::Max(SmoothMs * 0.001f, 1e-3f));
			auto Follow = [A](FVector& X, const FVector& Y) { X += (Y - X) * A; };
			Follow(M.Tip, G.Tip);
			Follow(M.Wrist, G.Wrist);
			Follow(M.Axis, G.Axis);
			Follow(M.Jaw, G.Jaw);
			M.Unit += (G.Unit - M.Unit) * A;
			M.JawHalf += (G.JawHalf - M.JawHalf) * A;
			if (M.bHasElbow && G.bHasElbow) Follow(M.Elbow, G.Elbow); else M.Elbow = G.Elbow;
			if (M.bHasMeas && G.bHasMeas) Follow(M.Meas, G.Meas); else M.Meas = G.Meas;
			if (M.bHasEcho && G.bHasEcho) Follow(M.Echo, G.Echo); else M.Echo = G.Echo;
			M.bHasElbow = G.bHasElbow;
			M.bHasMeas = G.bHasMeas;
			M.bHasEcho = G.bHasEcho;
			M.PawnScale = G.PawnScale;
			M.EchoUnit = G.EchoUnit;
		}
		bShownValid[S] = true;
		bShownGrey[S] = bGrey;
		ShownCalibId[S] = CalibId;
		FinishGeom(M);
		return M;
	}

	// ---------------------------------------------------------------- 예비: 디버그 선 (teleop.RobotVizStyle 0)
	void DrawArmDebug(UWorld* World, const FArmGeom& G, uint32 Flags, bool bStale)
	{
		const FColor C = ColorOf(Flags, bStale);
		const float U = G.Unit;
		if (G.bHasElbow)
		{
			DrawDebugCylinder(World, G.ForeEnd, G.Wrist, ForearmRadius * U, 12, NeutralColor, false, -1.0f, 0, 0.3f);
			DrawDebugSphere(World, G.Elbow, ElbowRadius * U, 8, NeutralColor, false, -1.0f, 0, 0.3f);
			DrawDebugLine(World, G.Elbow, G.ForeEnd, OffsetColor, false, -1.0f, 0, OffsetLineThick);
		}
		DrawDebugSphere(World, G.Wrist, WristRadius * U, 10, C, false, -1.0f, 0, 0.5f);
		const FVector BodyCenter = (G.Wrist + G.Root) * 0.5f;
		const float BodyLen = FVector::Dist(G.Wrist, G.Root);
		DrawDebugBox(World, BodyCenter, FVector(BodyWidth * U * 0.5f, BodyThick * U * 0.5f, BodyLen * 0.5f), G.Rot, C, false, -1.0f, 0, 0.5f);
		const FVector JawCenter = G.Root + G.Axis * (JawLen * U * 0.5f);
		const FVector Ext(JawThick * U * 0.5f, JawWidth * U * 0.5f, JawLen * U * 0.5f);
		DrawDebugBox(World, JawCenter + G.Jaw * G.JawHalf, Ext, G.Rot, C, false, -1.0f, 0, 0.6f);
		DrawDebugBox(World, JawCenter - G.Jaw * G.JawHalf, Ext, G.Rot, C, false, -1.0f, 0, 0.6f);
		DrawDebugSphere(World, G.Tip, TipRadius * U, 8, C, false, -1.0f, 0, 0.6f);
		if (bShowMeas && G.bHasMeas) DrawDebugSphere(World, G.Meas, TipRadius * U, 8, MeasColor, false, -1.0f, 0, 0.3f);
		if (bShowEcho && G.bHasEcho) DrawDebugSphere(World, G.Echo, 1.0f * G.EchoUnit, 8, EchoColor, false, -1.0f, 0, 0.3f);
	}

	// ---------------------------------------------------------------- 기본: 솔리드 도형
	// 엔진 기본 도형은 모두 가운데 원점, 100 cm: Cylinder = 높이 100·지름 100 (Z 축), Cube = 한 변 100, Sphere = 지름 100.
	enum EPart : int32 { PForearm, PElbow, POffset, PWrist, PBody, PJawA, PJawB, PTip, PMeas, PEcho, PNum };

	TWeakObjectPtr<AActor> VizActor;
	TWeakObjectPtr<UWorld> VizWorld;
	TWeakObjectPtr<UStaticMeshComponent> Parts[2][PNum];
	TWeakObjectPtr<UMaterialInstanceDynamic> StatusMat[2];
	TWeakObjectPtr<UStaticMeshComponent> TPlate, TCan, TBand[2], TRing[MaxSlots][RingSegs], TLiveCan, TLiveBand, TLiveReady[2], TGoalRing[RingSegs], TLivePlace;
	TWeakObjectPtr<UMaterialInstanceDynamic> TLiveMat, TGoalMat;
	FColor GoalColorShown;
	bool bGoalColorSet = false;
	FColor LiveColorShown(0, 0, 0);
	bool bLiveColorSet = false;
	bool bTablePartsMade = false;
	EColor StatusShown[2] = { EColor::Green, EColor::Green };
	bool bStatusSet[2] = { false, false };

	FLinearColor Lin(const FColor& C) { return FLinearColor(C); }

	void DestroyVizActor()
	{
		if (AActor* A = VizActor.Get())
		{
			A->Destroy();
		}
		VizActor.Reset();
		VizWorld.Reset();
		for (int32 S = 0; S < 2; ++S)
		{
			for (int32 P = 0; P < PNum; ++P) Parts[S][P].Reset();
			StatusMat[S].Reset();
			bStatusSet[S] = false;
		}
		TPlate.Reset();
		TCan.Reset();
		TBand[0].Reset();
		TBand[1].Reset();
		TLiveCan.Reset();
		TLiveBand.Reset();
		TLiveMat.Reset();
		TLiveReady[0].Reset();
		TLiveReady[1].Reset();
		TLivePlace.Reset();
		TGoalMat.Reset();
		for (int32 Seg = 0; Seg < RingSegs; ++Seg) TGoalRing[Seg].Reset();
		bLiveColorSet = false;
		bGoalColorSet = false;
		for (int32 N = 0; N < MaxSlots; ++N)
		{
			for (int32 K = 0; K < RingSegs; ++K) TRing[N][K].Reset();
		}
		bTablePartsMade = false;
	}

	UStaticMeshComponent* AddPart(AActor* Owner, USceneComponent* Root, UStaticMesh* Mesh, UMaterialInterface* Mat)
	{
		UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(Owner);
		C->SetMobility(EComponentMobility::Movable);
		C->SetStaticMesh(Mesh);
		C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		C->SetCastShadow(false);
		C->SetupAttachment(Root);
		if (Mat) C->SetMaterial(0, Mat);
		C->RegisterComponent();
		Owner->AddInstanceComponent(C);
		C->SetVisibility(false);
		return C;
	}

	bool EnsureVizActor(UWorld* World)
	{
		if (VizActor.IsValid() && VizWorld.Get() == World) return true;
		DestroyVizActor();
		UStaticMesh* Cyl = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
		UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
		UStaticMesh* Sph = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
		UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
		if (!Cyl || !Cube || !Sph)
		{
			UE_LOG(LogTemp, Warning, TEXT("[RobotViz] 기본 도형을 못 불러옴 — teleop.RobotVizStyle 0 (디버그 선) 으로 쓴다"));
			StyleMode = 0;
			return false;
		}
		FActorSpawnParameters SP;
		SP.ObjectFlags |= RF_Transient;
		SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		AActor* A = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, SP);
		if (!A) return false;
		USceneComponent* Root = NewObject<USceneComponent>(A, TEXT("RobotVizRoot"));
		Root->SetMobility(EComponentMobility::Movable);
		A->SetRootComponent(Root);
		Root->RegisterComponent();
		auto MakeMat = [&](const FColor& C) -> UMaterialInstanceDynamic*
		{
			if (!Base) return nullptr;
			UMaterialInstanceDynamic* M = UMaterialInstanceDynamic::Create(Base, A);
			// 🔴 [미확인] BasicShapeMaterial 의 벡터 파라미터 이름 "Color". 이름이 다르면 색이 안 바뀐다 (기본 흰색).
			if (M) M->SetVectorParameterValue(TEXT("Color"), Lin(C));
			return M;
		};
		UMaterialInterface* Neutral = MakeMat(NeutralColor);
		UMaterialInterface* MeasM = MakeMat(MeasColor);
		UMaterialInterface* EchoM = MakeMat(EchoColor);
		UMaterialInterface* OffsetM = MakeMat(OffsetColor);
		for (int32 S = 0; S < 2; ++S)
		{
			UMaterialInstanceDynamic* St = MakeMat(ColorOf(0, false));
			StatusMat[S] = St;
			Parts[S][PForearm] = AddPart(A, Root, Cyl, Neutral);
			Parts[S][PElbow] = AddPart(A, Root, Sph, Neutral);
			Parts[S][POffset] = AddPart(A, Root, Cyl, OffsetM);
			Parts[S][PWrist] = AddPart(A, Root, Sph, St);
			Parts[S][PBody] = AddPart(A, Root, Cube, St);
			Parts[S][PJawA] = AddPart(A, Root, Cube, St);
			Parts[S][PJawB] = AddPart(A, Root, Cube, St);
			Parts[S][PTip] = AddPart(A, Root, Sph, St);
			Parts[S][PMeas] = AddPart(A, Root, Sph, MeasM);
			Parts[S][PEcho] = AddPart(A, Root, Sph, EchoM);
		}
		VizActor = A;
		VizWorld = World;
		UE_LOG(LogTemp, Display, TEXT("[RobotViz] 솔리드 표시 액터를 만들었다 (재질 %s)"), Base ? TEXT("BasicShapeMaterial") : TEXT("없음 — 기본 흰색"));
		return true;
	}

	void SetPart(int32 S, int32 P, bool bVisible, const FTransform& X = FTransform::Identity)
	{
		UStaticMeshComponent* C = Parts[S][P].Get();
		if (!C) return;
		if (bVisible) C->SetWorldTransform(X);
		if (C->IsVisible() != bVisible) C->SetVisibility(bVisible);
	}

	void HideArmSolid(int32 S)
	{
		for (int32 P = 0; P < PNum; ++P) SetPart(S, P, false);
	}

	void HideAllSolid()
	{
		HideArmSolid(0);
		HideArmSolid(1);
	}

	FTransform CylinderBetween(const FVector& A, const FVector& B, float Radius)
	{
		const FVector D = B - A;
		const float Len = D.Size();
		const FQuat Q = Len > KINDA_SMALL_NUMBER ? FRotationMatrix::MakeFromZ(D / Len).ToQuat() : FQuat::Identity;
		return FTransform(Q, (A + B) * 0.5f, FVector(2.0f * Radius / 100.0f, 2.0f * Radius / 100.0f, Len / 100.0f));
	}

	FTransform Sphere(const FVector& C, float Radius)
	{
		return FTransform(FQuat::Identity, C, FVector(2.0f * Radius / 100.0f));
	}

	// 상자: 로컬 X = 벌어지는 축, Y = 셋째 축, Z = 손 축 (G.Rot). Size = 전체 길이 (cm)
	FTransform Box(const FQuat& Q, const FVector& Center, const FVector& Size)
	{
		return FTransform(Q, Center, Size / 100.0f);
	}

	void DrawArmSolid(int32 S, const FArmGeom& G, uint32 Flags, bool bStale)
	{
		const EColor K = ColorKind(Flags, bStale);
		if (UMaterialInstanceDynamic* M = StatusMat[S].Get())
		{
			if (!bStatusSet[S] || StatusShown[S] != K)          // 색이 바뀔 때만 파라미터를 바꾼다
			{
				M->SetVectorParameterValue(TEXT("Color"), Lin(ColorOf(Flags, bStale)));
				StatusShown[S] = K;
				bStatusSet[S] = true;
			}
		}
		const float U = G.Unit;
		SetPart(S, PForearm, G.bHasElbow, G.bHasElbow ? CylinderBetween(G.ForeEnd, G.Wrist, ForearmRadius * U) : FTransform::Identity);
		SetPart(S, PElbow, G.bHasElbow, G.bHasElbow ? Sphere(G.Elbow, ElbowRadius * U) : FTransform::Identity);
		SetPart(S, POffset, G.bHasElbow, G.bHasElbow ? CylinderBetween(G.Elbow, G.ForeEnd, OffsetRadius * U) : FTransform::Identity);
		SetPart(S, PWrist, true, Sphere(G.Wrist, WristRadius * U));
		const float BodyLen = FVector::Dist(G.Wrist, G.Root);
		SetPart(S, PBody, true, Box(G.Rot, (G.Wrist + G.Root) * 0.5f, FVector(BodyWidth * U, BodyThick * U, BodyLen)));
		const FVector JawCenter = G.Root + G.Axis * (JawLen * U * 0.5f);
		const FVector JawSize(JawThick * U, JawWidth * U, JawLen * U);
		SetPart(S, PJawA, true, Box(G.Rot, JawCenter + G.Jaw * G.JawHalf, JawSize));
		SetPart(S, PJawB, true, Box(G.Rot, JawCenter - G.Jaw * G.JawHalf, JawSize));
		SetPart(S, PTip, true, Sphere(G.Tip, TipRadius * U));
		SetPart(S, PMeas, bShowMeas && G.bHasMeas, G.bHasMeas ? Sphere(G.Meas, TipRadius * U) : FTransform::Identity);
		SetPart(S, PEcho, bShowEcho && G.bHasEcho, G.bHasEcho ? Sphere(G.Echo, 1.0f * G.EchoUnit) : FTransform::Identity);
	}

	// ---------------------------------------------------------------- 판·캔 (머리 주석 "판·캔 표시")
	const FColor PlateColor(120, 120, 130);
	const FColor SlotColor(40, 220, 230);       // 한 팔 자리
	const FColor HandoverColor(230, 60, 220);   // 인계 자리
	const FColor GoalColor(60, 230, 80);        // 목표 자리 (굵은 원)
	const FColor GoalInColor(200, 255, 200);    // 캔이 목표 허용 범위 안 (밝게)
	const FColor PlaceOkColor(80, 200, 255);    // 지금 놓으면 섬 (캔 밑면 고리)
	const FColor CanColor(170, 175, 185);
	const FColor BandColor(255, 240, 170);      // 잡는 높이 띠 (옅은 색, 기본 재질이 불투명이라 색으로 나눈다)
	const FColor CanHeldColor(40, 210, 170);    // 가상 캔: 잡힘 (서 있음은 CanColor)
	const FColor CanFallenColor(150, 50, 45);   // 넘어짐
	const FColor CanBumpColor(255, 120, 200);   // 부딪힘
	const FColor CanReadyColor(255, 225, 40);   // 지금 닫으면 잡힘 (테두리)

	void EnsureTableParts()
	{
		AActor* A = VizActor.Get();
		if (bTablePartsMade || !A) return;
		USceneComponent* Root = A->GetRootComponent();
		UStaticMesh* Cyl = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
		UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
		UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
		if (!Cyl || !Cube) return;
		auto MakeMat = [&](const FColor& C) -> UMaterialInterface*
		{
			if (!Base) return nullptr;
			UMaterialInstanceDynamic* M = UMaterialInstanceDynamic::Create(Base, A);
			if (M) M->SetVectorParameterValue(TEXT("Color"), Lin(C));
			return M;
		};
		UMaterialInterface* SlotM = MakeMat(SlotColor);
		UMaterialInterface* HandM = MakeMat(HandoverColor);
		UMaterialInterface* BandM = MakeMat(BandColor);
		TPlate = AddPart(A, Root, Cube, MakeMat(PlateColor));
		TCan = AddPart(A, Root, Cyl, MakeMat(CanColor));
		TBand[0] = AddPart(A, Root, Cyl, BandM);
		TBand[1] = AddPart(A, Root, Cyl, BandM);
		UMaterialInstanceDynamic* LiveM = Base ? UMaterialInstanceDynamic::Create(Base, A) : nullptr;
		TLiveMat = LiveM;
		TLiveCan = AddPart(A, Root, Cyl, LiveM);
		TLiveBand = AddPart(A, Root, Cyl, BandM);
		UMaterialInterface* ReadyM = MakeMat(CanReadyColor);
		TLiveReady[0] = AddPart(A, Root, Cyl, ReadyM);
		TLiveReady[1] = AddPart(A, Root, Cyl, ReadyM);
		UMaterialInstanceDynamic* GoalM = Base ? UMaterialInstanceDynamic::Create(Base, A) : nullptr;
		if (GoalM) GoalM->SetVectorParameterValue(TEXT("Color"), Lin(GoalColor));
		TGoalMat = GoalM;
		for (int32 K = 0; K < RingSegs; ++K) TGoalRing[K] = AddPart(A, Root, Cube, GoalM);
		TLivePlace = AddPart(A, Root, Cyl, MakeMat(PlaceOkColor));
		// DrawTable 은 RVT1 을 받은 뒤에만 부르므로 자리 종류를 안다
		for (int32 N = 0; N < MaxSlots; ++N)
		{
			UMaterialInterface* M = (N < TableHdr.NumSlots && TableSlots[N].Kind != 0) ? HandM : SlotM;
			for (int32 K = 0; K < RingSegs; ++K) TRing[N][K] = AddPart(A, Root, Cube, M);
		}
		bTablePartsMade = true;
	}

	void SetTablePart(TWeakObjectPtr<UStaticMeshComponent>& W, bool bVisible, const FTransform& X = FTransform::Identity)
	{
		UStaticMeshComponent* C = W.Get();
		if (!C) return;
		if (bVisible) C->SetWorldTransform(X);
		if (C->IsVisible() != bVisible) C->SetVisibility(bVisible);
	}

	void HideTable()
	{
		SetTablePart(TPlate, false);
		SetTablePart(TCan, false);
		SetTablePart(TBand[0], false);
		SetTablePart(TBand[1], false);
		SetTablePart(TLiveCan, false);
		SetTablePart(TLiveBand, false);
		SetTablePart(TLiveReady[0], false);
		SetTablePart(TLiveReady[1], false);
		SetTablePart(TLivePlace, false);
		for (int32 K = 0; K < RingSegs; ++K) SetTablePart(TGoalRing[K], false);
		for (int32 N = 0; N < MaxSlots; ++N)
		{
			for (int32 K = 0; K < RingSegs; ++K) SetTablePart(TRing[N][K], false);
		}
	}

	bool LiveCanFresh() { return bHaveCan && FPlatformTime::Seconds() - CanRxSec < CanStaleSec; }

	// 판을 켰는데 그리지 못하는 이유 (1초 상태 줄용). 그릴 수 있으면 빈 문자열.
	const TCHAR* TableProblem(UWorld* World, const FArm* A[2])
	{
		if (!bShowTable) return TEXT("");
		if (!bHaveTable) return TEXT(" [판 못 그림: RVT1 없음 — 워치독 --table 확인]");
		if (!A[0] || !A[1]) return TEXT(" [판 못 그림: 팔 패킷 없음]");
		if (IsNaN3(A[0]->RangeOrigin) || IsNaN3(A[1]->RangeOrigin) || FMath::IsNaN(A[0]->RangeScale) || FMath::IsNaN(A[1]->RangeScale))
			return TEXT(" [판 못 그림: range 값 NaN — 보정 전이거나 손 위치 방식이 아님]");
		FTransform T; float Scale; FVector Offset;
		if (!RawToWorld(World, T, Scale, Offset)) return TEXT(" [판 못 그림: 폰 없음]");
		return TEXT("");
	}

	// A[0·1] = 지금 그리는 두 팔 패킷 (Range* 를 쓴다). 둘 다 있어야 그린다.
	void DrawTable(UWorld* World, const FArm* A[2], const FTransform& T, float Scale, const FVector& Offset, bool bRealSize, bool bSolid)
	{
		const bool bRangeOk = A[0] && A[1] && !IsNaN3(A[0]->RangeOrigin) && !IsNaN3(A[1]->RangeOrigin)
			&& !FMath::IsNaN(A[0]->RangeScale) && !FMath::IsNaN(A[1]->RangeScale);
		if (!bHaveTable || !bRangeOk)
		{
			bPlateKnown = false;
			bSlotBaseKnown = false;
			if (bSolid) HideTable();
			return;
		}
		auto Raw = [&](int32 S, const FVector& P)
		{
			const FVector H = (P - V(S == 0 ? TableHdr.PsL : TableHdr.PsR)) * A[S]->RangeScale;
			return V(A[S]->RangeOrigin) + V(A[S]->RangeFwd) * H.X - V(A[S]->RangeRight) * H.Y + V(A[S]->RangeUp) * H.Z;
		};
		auto RawDir = [&](int32 S, const FVector& D)
		{
			return V(A[S]->RangeFwd) * D.X - V(A[S]->RangeRight) * D.Y + V(A[S]->RangeUp) * D.Z;
		};
		auto ToWorld = [&](const FVector& P) { return T.TransformPosition(P * Scale + Offset); };
		auto Avg = [&](const FVector& P) { return ToWorld((Raw(0, P) + Raw(1, P)) * 0.5f); };
		const float PawnScale = Scale * T.GetScale3D().X;
		const float MeanRange = 0.5f * (A[0]->RangeScale + A[1]->RangeScale);
		const float K = MeanRange * PawnScale;
		const float Unit = bRealSize ? PawnScale : K;
		const FVector Up = T.TransformVectorNoScale((V(A[0]->RangeUp) + V(A[1]->RangeUp)) * 0.5f).GetSafeNormal();
		const FVector Fw = T.TransformVectorNoScale((V(A[0]->RangeFwd) + V(A[1]->RangeFwd)) * 0.5f).GetSafeNormal();
		const FVector Rt = FVector::CrossProduct(Up, Fw).GetSafeNormal();
		const FVector PlateLift = Up * (TableHdr.GripH * (K - Unit));
		const FVector C00 = Avg(FVector(TableHdr.X0, TableHdr.Y0, TableHdr.Z0)) + PlateLift;
		const FVector C10 = Avg(FVector(TableHdr.X1, TableHdr.Y0, TableHdr.Z1)) + PlateLift;
		const FVector C01 = Avg(FVector(TableHdr.X0, TableHdr.Y1, TableHdr.Z0)) + PlateLift;
		const FVector C11 = Avg(FVector(TableHdr.X1, TableHdr.Y1, TableHdr.Z1)) + PlateLift;
		// 판 표시를 꺼도 위치는 남긴다 — 소품 숨기기·목록이 쓴다
		PlateBox = FBox(ForceInit);
		PlateBox += C00;
		PlateBox += C10;
		PlateBox += C01;
		PlateBox += C11;
		bPlateKnown = true;
		if (!bShowTable)
		{
			bSlotBaseKnown = false;
			if (bSolid) HideTable();
			return;
		}
		if (bSolid) EnsureTableParts();
		const FVector PlateCenter = (C00 + C10 + C01 + C11) * 0.25f;
		const FVector PlateX = (C10 + C11 - C00 - C01) * 0.5f;
		const FVector PlateY = (C01 + C11 - C00 - C10) * 0.5f;
		const float Th = TableHdr.Thick * Unit;
		const FQuat PlateQ = FRotationMatrix::MakeFromXY(PlateX, PlateY).ToQuat();
		const FVector PlateBoxCenter = PlateCenter - PlateQ.GetAxisZ() * (0.5f * Th);
		if (bSolid) SetTablePart(TPlate, true, Box(PlateQ, PlateBoxCenter, FVector(PlateX.Size(), PlateY.Size(), Th)));
		else DrawDebugBox(World, PlateBoxCenter, FVector(PlateX.Size() * 0.5f, PlateY.Size() * 0.5f, Th * 0.5f), PlateQ, PlateColor, false, -1.0f, 0, 0.3f);
		const float R = TableHdr.CanDia * 0.5f * Unit;
		bool bCanDrawn = false;
		const int32 GoalSlot = LiveCanFresh() ? (int32)(CanPkt.Flags >> CanGoalShift) : 0;   // 0 = 없음
		bool bGoalDrawn = false;
		for (int32 N = 0; N < MaxSlots; ++N)
		{
			if (N >= TableHdr.NumSlots)
			{
				if (bSolid)
				{
					for (int32 Q = 0; Q < RingSegs; ++Q) SetTablePart(TRing[N][Q], false);
				}
				continue;
			}
			const FTableSlot& S = TableSlots[N];
			const FVector Surface = V(S.Pos);
			const float GripRef = S.GripH2 > 0.0f ? 0.5f * (S.GripH1 + S.GripH2) : S.GripH1;  // 캔 밑면에서 잡는 점
			FVector Base;
			if (S.Kind == 0 && S.ArmId < 2)
			{
				const int32 Arm = S.ArmId;
				const FVector C = Surface + FVector(0.0f, 0.0f, GripRef);
				const FVector Ax = V(S.HandAxis);
				const FVector AxW = T.TransformVectorNoScale(RawDir(Arm, Ax)).GetSafeNormal();
				const FVector Grip = bRealSize
					? ToWorld(Raw(Arm, C - Ax * (HandLen - S.TipPast))) + AxW * ((HandLen - S.TipPast) * Unit)
					: ToWorld(Raw(Arm, C));
				Base = Grip - Up * (GripRef * Unit);
			}
			else
			{
				const FVector C = Surface + FVector(0.0f, 0.0f, S.Bottom + GripRef);
				Base = Avg(C) - Up * (GripRef * Unit);
			}
			SlotBase[N] = Base;
			// 원: 한 팔 자리는 캔 밑면 (그리퍼가 올 곳), 인계 자리는 판 그림 위 (공중 인계도 원은 판에, 캔만 뜬다)
			const FVector RingC = (S.Kind == 0 ? Base : Avg(Surface) + PlateLift) + Up * (0.5f * Unit);
			const FColor RC = S.Kind == 0 ? SlotColor : HandoverColor;
			if (GoalSlot == N + 1)
			{
				const float RG = R + 0.8f * Unit;
				const FVector GC = RingC + Up * (0.1f * Unit);
				const bool bGoalIn = (CanPkt.Place & CanInGoal) != 0;
				const FColor GoalNow = bGoalIn ? GoalInColor : GoalColor;
				bGoalDrawn = true;
				if (bSolid)
				{
					if (UMaterialInstanceDynamic* GM = TGoalMat.Get())
					{
						if (!bGoalColorSet || GoalColorShown != GoalNow)
						{
							GM->SetVectorParameterValue(TEXT("Color"), Lin(GoalNow));
							GoalColorShown = GoalNow;
							bGoalColorSet = true;
						}
					}
					const float ChordG = 2.0f * RG * FMath::Sin(PI / RingSegs) * 1.1f;
					for (int32 Q = 0; Q < RingSegs; ++Q)
					{
						const float Ang = 2.0f * PI * (Q + 0.5f) / RingSegs;
						const FVector Dir = Fw * FMath::Cos(Ang) + Rt * FMath::Sin(Ang);
						const FQuat RQ = FRotationMatrix::MakeFromXZ(FVector::CrossProduct(Up, Dir), Up).ToQuat();
						SetTablePart(TGoalRing[Q], true, Box(RQ, GC + Dir * RG, FVector(ChordG, 1.2f * Unit, (bGoalIn ? 1.0f : 0.6f) * Unit)));
					}
				}
				else
				{
					DrawDebugCircle(World, GC, RG, 32, GoalNow, false, -1.0f, 0, bGoalIn ? 2.0f : 1.0f, Fw, Rt, false);
				}
			}
			if (bSolid)
			{
				const float Chord = 2.0f * R * FMath::Sin(PI / RingSegs) * 1.1f;
				for (int32 Q = 0; Q < RingSegs; ++Q)
				{
					const float Ang = 2.0f * PI * (Q + 0.5f) / RingSegs;
					const FVector Dir = Fw * FMath::Cos(Ang) + Rt * FMath::Sin(Ang);
					const FVector Tan = FVector::CrossProduct(Up, Dir);
					const FQuat RQ = FRotationMatrix::MakeFromXZ(Tan, Up).ToQuat();
					SetTablePart(TRing[N][Q], true, Box(RQ, RingC + Dir * R, FVector(Chord, 0.5f * Unit, 0.3f * Unit)));
				}
			}
			else
			{
				DrawDebugCircle(World, RingC, R, 32, RC, false, -1.0f, 0, 0.3f, Fw, Rt, false);
			}
			if (CanSlot == N + 1 && !LiveCanFresh())
			{
				const FVector TopP = Base + Up * (TableHdr.CanHeight * Unit);
				bCanDrawn = true;
				if (bSolid)
				{
					SetTablePart(TCan, true, CylinderBetween(Base, TopP, R));
					const float Gh[2] = { S.GripH1, S.GripH2 };
					for (int32 Q = 0; Q < 2; ++Q)
					{
						const FVector Bc = Base + Up * (Gh[Q] * Unit);
						SetTablePart(TBand[Q], Gh[Q] > 0.0f, CylinderBetween(Bc - Up * Unit, Bc + Up * Unit, R + 0.1f * Unit));
					}
				}
				else
				{
					DrawDebugCylinder(World, Base, TopP, R, 16, CanColor, false, -1.0f, 0, 0.3f);
					if (S.GripH1 > 0.0f) DrawDebugCircle(World, Base + Up * (S.GripH1 * Unit), R * 1.02f, 24, BandColor, false, -1.0f, 0, 0.3f, Fw, Rt, false);
					if (S.GripH2 > 0.0f) DrawDebugCircle(World, Base + Up * (S.GripH2 * Unit), R * 1.02f, 24, BandColor, false, -1.0f, 0, 0.3f, Fw, Rt, false);
				}
			}
		}
		if (bSolid && !bGoalDrawn)
		{
			for (int32 Seg = 0; Seg < RingSegs; ++Seg) SetTablePart(TGoalRing[Seg], false);   // K 는 위의 배율 변수라 다른 이름 (C4456)
		}
		if (bSolid && !bCanDrawn)
		{
			SetTablePart(TCan, false);
			SetTablePart(TBand[0], false);
			SetTablePart(TBand[1], false);
		}
		bSlotBaseKnown = true;
	}

	// 가상 캔 (머리 주석 "가상 캔 표시"). 팔을 그린 뒤에 부른다 (Shown 을 쓴다). A 는 DrawTable 과 같다.
	void DrawLiveCan(UWorld* World, const FArm* A[2], const FTransform& T, float Scale, const FVector& Offset, bool bRealSize, bool bSolid, float Dt)
	{
		const bool bRangeOk = A[0] && A[1] && !IsNaN3(A[0]->RangeOrigin) && !IsNaN3(A[1]->RangeOrigin)
			&& !FMath::IsNaN(A[0]->RangeScale) && !FMath::IsNaN(A[1]->RangeScale);
		if (!LiveCanFresh() || !bShowTable || !bHaveTable || !bRangeOk || !bSlotBaseKnown)
		{
			if (bSolid)
			{
				SetTablePart(TLiveCan, false);
				SetTablePart(TLiveBand, false);
				SetTablePart(TLiveReady[0], false);
				SetTablePart(TLiveReady[1], false);
				SetTablePart(TLivePlace, false);
			}
			bLiveShown = false;
			return;
		}
		auto Raw = [&](int32 S, const FVector& P)
		{
			const FVector H = (P - V(S == 0 ? TableHdr.PsL : TableHdr.PsR)) * A[S]->RangeScale;
			return V(A[S]->RangeOrigin) + V(A[S]->RangeFwd) * H.X - V(A[S]->RangeRight) * H.Y + V(A[S]->RangeUp) * H.Z;
		};
		auto DirW = [&](int32 S, const FVector& D)
		{
			return T.TransformVectorNoScale(V(A[S]->RangeFwd) * D.X - V(A[S]->RangeRight) * D.Y + V(A[S]->RangeUp) * D.Z);
		};
		auto ToWorld = [&](const FVector& P) { return T.TransformPosition(P * Scale + Offset); };
		const float PawnScale = Scale * T.GetScale3D().X;
		const float K = 0.5f * (A[0]->RangeScale + A[1]->RangeScale) * PawnScale;
		const float Unit = bRealSize ? PawnScale : K;
		const FVector Up = T.TransformVectorNoScale((V(A[0]->RangeUp) + V(A[1]->RangeUp)) * 0.5f).GetSafeNormal();
		const FVector Fw = T.TransformVectorNoScale((V(A[0]->RangeFwd) + V(A[1]->RangeFwd)) * 0.5f).GetSafeNormal();
		const FVector Rt = FVector::CrossProduct(Up, Fw).GetSafeNormal();
		auto PlateMap = [&](const FVector& P) { return ToWorld((Raw(0, P) + Raw(1, P)) * 0.5f) + Up * (TableHdr.GripH * (K - Unit)); };
		const FVector P = V(CanPkt.Pos);
		const float* Q = CanPkt.Quat;
		const FVector AxisBase(2.0f * (Q[0] * Q[2] + Q[1] * Q[3]), 2.0f * (Q[1] * Q[2] - Q[0] * Q[3]), 1.0f - 2.0f * (Q[0] * Q[0] + Q[1] * Q[1]));
		const int32 Arm = CanPkt.Anchor;
		const bool bHeld = CanPkt.State >= CanHeldL && CanPkt.State <= CanHeldBoth && Arm < 2 && bShownValid[Arm] && !IsNaN3(CanPkt.AnchorTip);
		FVector Base, AxisW;
		if (bHeld)
		{
			const FArmGeom& G = Shown[Arm];
			Base = G.Tip + DirW(Arm, P - V(CanPkt.AnchorTip)) * G.Unit;
			AxisW = DirW(Arm, AxisBase).GetSafeNormal();
			LastHeldP = P;
			LastHeldBase = Base;
			bLastHeld = true;
		}
		else
		{
			AxisW = (DirW(0, AxisBase) + DirW(1, AxisBase)).GetSafeNormal();
			if (CanPkt.Slot >= 1 && CanPkt.Slot <= TableHdr.NumSlots && CanPkt.Slot <= MaxSlots)
			{
				Base = SlotBase[CanPkt.Slot - 1];
				bLastHeld = false;
			}
			else
			{
				if (bLastHeld)
				{
					const FVector D = LastHeldBase - PlateMap(LastHeldP);
					PlaceOff = FVector(FVector::DotProduct(D, Fw), FVector::DotProduct(D, Rt), 0.0f);
					bLastHeld = false;
				}
				Base = PlateMap(P) + Fw * PlaceOff.X + Rt * PlaceOff.Y;
			}
		}
		// 상태가 바뀌면 튀는 양을 저장해 줄여 간다. 새 에피소드·처음은 바로 옮긴다
		if (!bLiveShown || CanPkt.Episode != LiveLastEpisode)
		{
			LiveFix = FVector::ZeroVector;
		}
		else if (CanPkt.State != LiveLastState)
		{
			LiveFix = LiveBase - Base;
		}
		LiveFix *= FMath::Exp(-Dt / CanFixTau);
		LiveBase = Base + LiveFix;
		LiveLastState = CanPkt.State;
		LiveLastEpisode = CanPkt.Episode;
		bLiveShown = true;
		const FColor C = (CanPkt.State == CanFallen) ? CanFallenColor
			: (CanPkt.State != CanOnTable) ? CanHeldColor
			: (CanPkt.Flags & CanFlagBump) ? CanBumpColor : CanColor;
		const FVector TopP = LiveBase + AxisW * (TableHdr.CanHeight * Unit);
		const float R = TableHdr.CanDia * 0.5f * Unit;
		const FVector BandC = LiveBase + AxisW * (TableHdr.GripH * Unit);
		const bool bBand = CanPkt.State == CanOnTable;
		const bool bReady = (CanPkt.Flags & (CanFlagReadyL | CanFlagReadyR)) != 0;
		const bool bPlaceOk = bHeld && (CanPkt.Place & CanPlaceOk) != 0;
		if (bSolid)
		{
			if (UMaterialInstanceDynamic* M = TLiveMat.Get())
			{
				if (!bLiveColorSet || LiveColorShown != C)
				{
					M->SetVectorParameterValue(TEXT("Color"), Lin(C));
					LiveColorShown = C;
					bLiveColorSet = true;
				}
			}
			SetTablePart(TLiveCan, true, CylinderBetween(LiveBase, TopP, R));
			SetTablePart(TLiveBand, bBand, CylinderBetween(BandC - AxisW * Unit, BandC + AxisW * Unit, R + 0.1f * Unit));
			const float ReadyR = R + 0.4f * Unit;
			SetTablePart(TLiveReady[0], bReady, CylinderBetween(LiveBase, LiveBase + AxisW * Unit, ReadyR));
			SetTablePart(TLiveReady[1], bReady, CylinderBetween(TopP - AxisW * Unit, TopP, ReadyR));
			SetTablePart(TLivePlace, bPlaceOk, CylinderBetween(LiveBase - AxisW * (0.2f * Unit), LiveBase + AxisW * (0.6f * Unit), R + 0.7f * Unit));
		}
		else
		{
			DrawDebugCylinder(World, LiveBase, TopP, R, 16, C, false, -1.0f, 0, 0.3f);
			if (bReady) DrawDebugCylinder(World, LiveBase, TopP, R + 0.4f * Unit, 16, CanReadyColor, false, -1.0f, 0, 0.3f);
			if (bPlaceOk) DrawDebugCircle(World, LiveBase, R + 0.7f * Unit, 24, PlaceOkColor, false, -1.0f, 0, 1.0f, Fw, Rt, false);
		}
	}

	// 1초 상태 줄 끝. RVC1 을 받은 적이 없으면 비어 있어 예전 줄과 같다.
	FString CanStatusText()
	{
		if (!bHaveCan) return FString();
		static const TCHAR* Names[CanStateNum] = { TEXT("서 있음"), TEXT("왼팔 잡음"), TEXT("오른팔 잡음"), TEXT("양팔 잡음"), TEXT("넘어짐") };
		const TCHAR* Name = CanPkt.State < CanStateNum ? Names[CanPkt.State] : TEXT("?");
		const TCHAR* Ready = (CanPkt.Flags & CanFlagReadyL) ? ((CanPkt.Flags & CanFlagReadyR) ? TEXT(" 잡힘가능 양손") : TEXT(" 잡힘가능 왼손")) : (CanPkt.Flags & CanFlagReadyR) ? TEXT(" 잡힘가능 오른손") : TEXT("");
		const uint8 Why = CanPkt.LastWhy;
		// Last 는 위의 전역 패킷 변수라 다른 이름 (C4459)
		const FString LastDrop = Why == 0 ? FString() : (Why & CanWhyStood) ? FString(TEXT(" 놓기 섬"))
			: FString::Printf(TEXT(" 놓기 넘어짐(%s%s%s%s)"), (Why & CanWhyHeight) ? TEXT(" 높이") : TEXT(""), (Why & CanWhyTilt) ? TEXT(" 기울기") : TEXT(""),
				(Why & CanWhySpeed) ? TEXT(" 속도") : TEXT(""), (Why & CanWhyOff) ? TEXT(" 판 밖") : TEXT(""));
		const FString GoalDist = CanPkt.GoalDistMm == 0xFFFF ? FString() : FString::Printf(TEXT(" %.1fcm%s"), CanPkt.GoalDistMm / 10.0f, (CanPkt.Place & CanInGoal) ? TEXT(" 안") : TEXT(""));
		return FString::Printf(TEXT(" 가상캔 %s%s%s%s%s 목표 %d%s 에피소드 %u%s bump %u (RVC1 %llu%s)"), Name, (CanPkt.Flags & CanFlagBump) ? TEXT(" 부딪힘") : TEXT(""), Ready,
			(CanPkt.Place & CanPlaceOk) ? TEXT(" 놓으면섬") : TEXT(""), *LastDrop,
			(int32)(CanPkt.Flags >> CanGoalShift), *GoalDist,
			(uint32)CanPkt.Episode, (CanPkt.Flags & CanFlagActive) ? TEXT(" 진행") : TEXT(" 대기"), (uint32)CanPkt.Bumps, CanRx, LiveCanFresh() ? TEXT("") : TEXT(", 끊김"));
	}

	// ---------------------------------------------------------------- 판을 가리는 소품·아바타 숨기기
	// 실행 중에만 SetHiddenInGame 으로 숨긴다. PIE 월드는 에디터 월드의 복사본이라 레벨·폰 파일은 바뀌지 않는다.
	// 표시 값(SetVisibility)은 다른 코드도 쓰므로 건드리지 않고, 원래 숨어 있던 것은 목록에 넣지 않아 되돌릴 때 드러나지 않는다.
	constexpr float PropMarginCm = 10.0f;     // 판 상자 수평 여유
	constexpr float PropVertCm = 100.0f;      // 판 상자 위아래 여유
	constexpr float FloorMinCm = 300.0f;      // 바닥: 가로·세로 모두 이 이상이고 윗면이 판보다 아래
	constexpr float WallMinHeightCm = 200.0f; // 벽: 이 높이 이상이면서 길이가 WallMinLenCm 이상
	constexpr float WallMinLenCm = 300.0f;
	constexpr float HugeCm = 2000.0f;         // 하늘 구 등: 어느 한 변이 이 이상
	constexpr double ListNoPlateCm = 500.0;   // 판 위치를 모를 때 목록에 넣을 폰 주변 거리

	enum class EPropKind : uint8 { Hide, Floor, Huge, Wall, Far };

	TArray<TWeakObjectPtr<USceneComponent>> HiddenProps;
	TArray<TWeakObjectPtr<USceneComponent>> HiddenBody;
	bool bHideProps = false;   // teleop.RobotVizHideProps
	bool bHideBody = false;    // teleop.RobotVizHideBody
	bool bPropsWaitLogged = false;

	EPropKind ClassifyProp(const FBox& B)
	{
		const FBox Near(PlateBox.Min - FVector(PropMarginCm, PropMarginCm, PropVertCm),
			PlateBox.Max + FVector(PropMarginCm, PropMarginCm, PropVertCm));
		if (!B.Intersect(Near)) return EPropKind::Far;
		const FVector Size = B.GetSize();
		if (Size.X >= FloorMinCm && Size.Y >= FloorMinCm && B.Max.Z <= PlateBox.Min.Z + 5.0) return EPropKind::Floor;
		if (Size.GetMax() >= HugeCm) return EPropKind::Huge;
		if (Size.Z >= WallMinHeightCm && FMath::Max(Size.X, Size.Y) >= WallMinLenCm) return EPropKind::Wall;
		return EPropKind::Hide;
	}

	const TCHAR* PropKindName(EPropKind K)
	{
		switch (K)
		{
		case EPropKind::Hide: return TEXT("숨김 대상");
		case EPropKind::Floor: return TEXT("제외: 바닥");
		case EPropKind::Huge: return TEXT("제외: 하늘 등 아주 큰 것");
		case EPropKind::Wall: return TEXT("제외: 벽");
		default: return TEXT("판에서 멂");
		}
	}

	APawn* PlayerPawn(UWorld* World)
	{
		APlayerController* PC = World ? World->GetFirstPlayerController() : nullptr;
		return PC ? PC->GetPawn() : nullptr;
	}

	// 폰에 속한 액터인가 (폰 자신, 폰에 붙은 액터, 폰이 주인인 액터). 이것들은 소품으로 치지 않는다.
	bool IsPawnSide(const AActor* A, const APawn* Pawn)
	{
		if (!Pawn) return false;
		for (const AActor* P = A; P; P = P->GetAttachParentActor())
		{
			if (P == Pawn || P->GetOwner() == Pawn) return true;
		}
		return false;
	}

	FString MeshAssetName(const UMeshComponent* C)
	{
		if (const UStaticMeshComponent* S = Cast<UStaticMeshComponent>(C))
		{
			return GetNameSafe(S->GetStaticMesh());
		}
		if (const USkinnedMeshComponent* K = Cast<USkinnedMeshComponent>(C))
		{
			return GetNameSafe(K->GetSkinnedAsset());
		}
		return TEXT("-");
	}

	FString DescribeProp(const AActor* A, const UStaticMeshComponent* C)
	{
		const FVector Size = C->Bounds.GetBox().GetSize();
		return FString::Printf(TEXT("%s / %s (%s) 크기 %.0f x %.0f x %.0f cm"), *A->GetName(), *C->GetName(), *MeshAssetName(C), Size.X, Size.Y, Size.Z);
	}

	// 레벨의 StaticMesh 컴포넌트를 돌며 F(액터, 컴포넌트). 표시 액터와 폰 쪽은 뺀다.
	template <typename FuncType>
	void ForEachLevelMesh(UWorld* World, FuncType F)
	{
		const APawn* Pawn = PlayerPawn(World);
		const AActor* Viz = VizActor.Get();
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			AActor* A = *It;
			if (!A || A == Viz || IsPawnSide(A, Pawn)) continue;
			TArray<UStaticMeshComponent*> Comps;
			A->GetComponents<UStaticMeshComponent>(Comps);
			for (UStaticMeshComponent* C : Comps)
			{
				if (C && C->IsRegistered() && C->GetStaticMesh()) F(A, C);
			}
		}
	}

	// 켜져 있는 동안 1초마다 부른다: 나중에 판이 옮겨 새로 겹친 소품도 숨긴다 (숨긴 것은 끌 때까지 그대로).
	void ApplyHideProps(UWorld* World)
	{
		if (!bHideProps) return;
		if (!bPlateKnown)
		{
			if (!bPropsWaitLogged)
			{
				UE_LOG(LogTemp, Display, TEXT("[RobotViz] HideProps 대기 — 판 위치를 아직 모른다 (RVT1 과 보정이 있어야 한다. 상태 줄의 '판 못 그림' 참고)"));
				bPropsWaitLogged = true;
			}
			return;
		}
		bPropsWaitLogged = false;
		int32 Added = 0;
		ForEachLevelMesh(World, [&](AActor* A, UStaticMeshComponent* C)
		{
			if (C->bHiddenInGame || !C->IsVisible()) return;      // 이미 안 보인다 (우리가 숨겼거나 원래 그렇다)
			if (ClassifyProp(C->Bounds.GetBox()) != EPropKind::Hide) return;
			C->SetHiddenInGame(true, false);
			HiddenProps.AddUnique(C);
			++Added;
			UE_LOG(LogTemp, Display, TEXT("[RobotViz] 소품 숨김: %s"), *DescribeProp(A, C));
		});
		if (Added > 0)
		{
			UE_LOG(LogTemp, Display, TEXT("[RobotViz] 소품 %d개를 더 숨겼다 (모두 %d개)"), Added, HiddenProps.Num());
		}
	}

	bool IsHandLike(const UMeshComponent* C)
	{
		return C->GetName().Contains(TEXT("Hand")) || C->GetClass()->GetName().Contains(TEXT("Hand")) || MeshAssetName(C).Contains(TEXT("Hand"));
	}

	// 켜져 있는 동안 1초마다 부른다: BodyTrackingSender 가 아바타를 초기화하며 다시 보이게 해도 또 숨긴다.
	// 보이기만 끈다. 송신은 OculusXRMovement::GetBodyState 를 읽을 뿐 메시를 쓰지 않는다.
	void ApplyHideBody(UWorld* World)
	{
		if (!bHideBody) return;
		APawn* Pawn = PlayerPawn(World);
		if (!Pawn) return;
		TArray<USkinnedMeshComponent*> Skinned;
		Pawn->GetComponents<USkinnedMeshComponent>(Skinned);
		for (USkinnedMeshComponent* C : Skinned)
		{
			if (!C || IsHandLike(C) || C->bHiddenInGame) continue;
			C->SetHiddenInGame(true, false);
			HiddenBody.AddUnique(C);
			UE_LOG(LogTemp, Display, TEXT("[RobotViz] 몸 숨김: %s (%s)"), *C->GetName(), *MeshAssetName(C));
		}
	}

	void RestoreList(TArray<TWeakObjectPtr<USceneComponent>>& List)
	{
		for (const TWeakObjectPtr<USceneComponent>& W : List)
		{
			if (USceneComponent* C = W.Get()) C->SetHiddenInGame(false, false);
		}
		List.Reset();
	}

	// 숨긴 기록과 켬 상태만 지운다 (VR Preview 가 끝날 때 — 월드째 사라지므로 되돌릴 것이 없다)
	void ForgetHidden()
	{
		HiddenProps.Reset();
		HiddenBody.Reset();
		bHideProps = false;
		bHideBody = false;
		bPropsWaitLogged = false;
	}

	UWorld* FindPlayWorld()
	{
		if (!GEngine) return nullptr;
		for (const FWorldContext& Context : GEngine->GetWorldContexts())
		{
			UWorld* W = Context.World();
			if (W && (Context.WorldType == EWorldType::PIE || Context.WorldType == EWorldType::Game)) return W;
		}
		return nullptr;
	}

	// 폰의 메시 컴포넌트 목록 (붙은 액터 포함). 무엇을 숨기고 무엇을 남기는지 함께 찍는다.
	void LogPawnMeshes(UWorld* World)
	{
		APawn* Pawn = PlayerPawn(World);
		if (!Pawn)
		{
			UE_LOG(LogTemp, Display, TEXT("[RobotViz] 폰이 없다"));
			return;
		}
		TArray<AActor*> Actors;
		Pawn->GetAttachedActors(Actors, true, true);
		Actors.Insert(Pawn, 0);
		int32 NBody = 0, NHand = 0, NOther = 0;
		for (AActor* A : Actors)
		{
			TArray<UMeshComponent*> Comps;
			A->GetComponents<UMeshComponent>(Comps);
			for (UMeshComponent* C : Comps)
			{
				if (!C) continue;
				const bool bSkinned = C->IsA<USkinnedMeshComponent>();
				const bool bHand = bSkinned && IsHandLike(C);
				const bool bBody = bSkinned && !bHand && A == Pawn;
				bBody ? ++NBody : (bHand ? ++NHand : ++NOther);
				UE_LOG(LogTemp, Display, TEXT("[RobotViz]   폰 %s / %s [%s] (%s) 보임 %d 게임숨김 %d — %s"),
					*A->GetName(), *C->GetName(), *C->GetClass()->GetName(), *MeshAssetName(C), C->IsVisible() ? 1 : 0, C->bHiddenInGame ? 1 : 0,
					bBody ? TEXT("HideBody 가 숨긴다") : (bHand ? TEXT("손 모델: 숨기지 않는다") : TEXT("컨트롤러·헤드셋·디버그 구체 등: 숨기지 않는다")));
			}
		}
		UE_LOG(LogTemp, Display, TEXT("[RobotViz] 폰 메시: 몸 %d개는 HideBody 대상, 손 모델 %d개와 그 밖 %d개(컨트롤러 모델 포함)는 숨기지 않는다"), NBody, NHand, NOther);
	}

	void LogNearProps(UWorld* World)
	{
		const APawn* Pawn = PlayerPawn(World);
		if (bPlateKnown)
		{
			UE_LOG(LogTemp, Display, TEXT("[RobotViz] 판 상자 %s ~ %s (여유 수평 %.0f, 위아래 %.0f cm)"), *PlateBox.Min.ToCompactString(), *PlateBox.Max.ToCompactString(), PropMarginCm, PropVertCm);
		}
		else
		{
			UE_LOG(LogTemp, Display, TEXT("[RobotViz] 판 위치를 아직 모른다 — 폰 주변 %.0f cm 안의 메시를 판정 없이 찍는다"), ListNoPlateCm);
		}
		int32 N = 0;
		ForEachLevelMesh(World, [&](AActor* A, UStaticMeshComponent* C)
		{
			const FBox B = C->Bounds.GetBox();
			if (bPlateKnown)
			{
				const EPropKind K = ClassifyProp(B);
				if (K == EPropKind::Far) return;
				UE_LOG(LogTemp, Display, TEXT("[RobotViz]   근처 %s — %s, 보임 %d 게임숨김 %d"), *DescribeProp(A, C), PropKindName(K), C->IsVisible() ? 1 : 0, C->bHiddenInGame ? 1 : 0);
			}
			else
			{
				if (!Pawn || B.ComputeSquaredDistanceToPoint(Pawn->GetActorLocation()) > ListNoPlateCm * ListNoPlateCm) return;
				UE_LOG(LogTemp, Display, TEXT("[RobotViz]   주변 %s, 보임 %d 게임숨김 %d"), *DescribeProp(A, C), C->IsVisible() ? 1 : 0, C->bHiddenInGame ? 1 : 0);
			}
			++N;
		});
		UE_LOG(LogTemp, Display, TEXT("[RobotViz] 근처 레벨 메시 %d개"), N);
	}

	// ---------------------------------------------------------------- 매 틱
	void PostActorTick(UWorld* World, ELevelTick, float)
	{
		if (!World || (World->WorldType != EWorldType::PIE && World->WorldType != EWorldType::Game)) return;
		Poll();
		const double Now = FPlatformTime::Seconds();
		const bool bStale = !bHave || Now - LastRxSec > StaleSec;
		// 마지막으로 점이 있던 위치를 팔마다 보관. 새 보정이 오면 지운다
		if (bHave)
		{
			if (Last.CalibId != GoodCalibId)
			{
				bHaveGood[0] = bHaveGood[1] = false;
				GoodCalibId = Last.CalibId;
			}
			for (int32 S = 0; S < 2; ++S)
			{
				if (!IsNaN3(Last.Arm[S].TipCmd) && !IsNaN3(Last.Arm[S].Wrist) && !IsNaN3(Last.Arm[S].Axis))
				{
					LastGood[S] = Last.Arm[S];
					bHaveGood[S] = true;
				}
			}
		}
		// 팔마다 이번에 그릴 것: 점이 있으면 지금 패킷, 없으면 마지막 위치를 회색으로
		const FArm* Draw[2] = { nullptr, nullptr };
		bool bGrey[2] = { bStale, bStale };
		for (int32 S = 0; S < 2; ++S)
		{
			if (bHave && (Last.State & StCalib) && !IsNaN3(Last.Arm[S].TipCmd))
			{
				Draw[S] = &Last.Arm[S];
			}
			else if (bHaveGood[S])
			{
				Draw[S] = &LastGood[S];
				bGrey[S] = true;
			}
		}
		const bool bRealSize = RealSizeMode < 0 ? (Last.Options & OptRealSize) != 0 : RealSizeMode == 1;
		if (Now - LastLogSec >= 1.0)   // 1초에 한 줄 (색 이름은 VR 밖 확인용)
		{
			UE_LOG(LogTemp, Display, TEXT("[RobotViz] rx %llu/s (누적 %llu, 버림 %llu)%s seq %u calib %u flags L 0x%03X R 0x%03X 쌍 %.2f 색 L %s R %s (%s, %s, %s) 판 %s RVT1 %llu 캔 %d%s%s"),
				RxCount - RxCountAtLog, RxCount, BadCount, bStale ? TEXT(" [수신 없음]") : TEXT(""),
				bHave ? Last.Seq : 0u, bHave ? Last.CalibId : 0u, bHave ? Last.Arm[0].Flags : 0u, bHave ? Last.Arm[1].Flags : 0u,
				bHave ? Last.Arm[0].PairW / 255.0f : 0.0f,
				Draw[0] ? ColorName(ColorKind(Draw[0]->Flags, bGrey[0])) : TEXT("-"),
				Draw[1] ? ColorName(ColorKind(Draw[1]->Flags, bGrey[1])) : TEXT("-"),
				StyleMode == 1 ? TEXT("솔리드") : TEXT("디버그 선"), bRealSize ? TEXT("실제 크기") : TEXT("배율 크기"),
				bSmooth ? TEXT("부드럽게") : TEXT("그대로"),
				bShowTable ? TEXT("켬") : TEXT("끔"), TableRx, CanSlot, TableProblem(World, Draw), *CanStatusText());
			LastLogSec = Now;
			RxCountAtLog = RxCount;
			ApplyHideProps(World);
			ApplyHideBody(World);
		}
		const float Dt = LastDrawSec < 0.0 ? 0.0f : (float)FMath::Clamp(Now - LastDrawSec, 0.0, 0.1);
		LastDrawSec = Now;
		const bool bSolid = StyleMode == 1 && EnsureVizActor(World);
		if (!bSolid && VizActor.IsValid()) HideAllSolid();     // 디버그 선으로 바꾸면 솔리드는 숨긴다
		FTransform T; float Scale; FVector Offset;
		if ((!Draw[0] && !Draw[1]) || !RawToWorld(World, T, Scale, Offset))
		{
			if (bSolid)
			{
				HideAllSolid();
				HideTable();
			}
			bShownValid[0] = bShownValid[1] = false;
			return;
		}
		DrawTable(World, Draw, T, Scale, Offset, bRealSize, bSolid);
		for (int32 S = 0; S < 2; ++S)
		{
			FArmGeom G;
			if (!Draw[S] || !MakeGeom(*Draw[S], T, Scale, Offset, bRealSize, G))
			{
				if (bSolid) HideArmSolid(S);
				bShownValid[S] = false;                        // 다시 나타나면 바로 그 위치에
				continue;
			}
			const FArmGeom& Sm = SmoothGeom(S, G, bGrey[S], Last.CalibId, Dt);
			if (bSolid) DrawArmSolid(S, Sm, Draw[S]->Flags, bGrey[S]);
			else DrawArmDebug(World, Sm, Draw[S]->Flags, bGrey[S]);
		}
		DrawLiveCan(World, Draw, T, Scale, Offset, bRealSize, bSolid, Dt);
	}

	void Stop()
	{
		if (PostTickHandle.IsValid())
		{
			FWorldDelegates::OnWorldPostActorTick.Remove(PostTickHandle);
			PostTickHandle.Reset();
		}
		if (CleanupHandle.IsValid())
		{
			FWorldDelegates::OnWorldCleanup.Remove(CleanupHandle);
			CleanupHandle.Reset();
		}
		DestroyVizActor();
		if (Socket)
		{
			Socket->Close();
			if (ISocketSubsystem* SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)) SS->DestroySocket(Socket);
			Socket = nullptr;
			UE_LOG(LogTemp, Display, TEXT("[RobotViz] OFF (받은 패킷 %llu)"), RxCount);
		}
		bHave = false;
		bHaveTable = false;
		bHaveCan = false;
		bLiveShown = false;
		bLastHeld = false;
		bSlotBaseKnown = false;
		bPlateKnown = false;
		LastRxSec = -1.0;
		bHaveGood[0] = bHaveGood[1] = false;
		bShownValid[0] = bShownValid[1] = false;
		LastDrawSec = -1.0;
	}

	void OnWorldCleanup(UWorld* World, bool, bool)
	{
		// PIE 를 끝내면 임시 액터·소켓·델리게이트를 정리한다 (test1.cpp:16 HandleWorldCleanup 과 같은 시점)
		if (World && (World->WorldType == EWorldType::PIE || World->WorldType == EWorldType::Game))
		{
			ForgetHidden();   // PIE 월드째 사라진다 — 숨김은 저장되지 않고, 다음 VR Preview 는 꺼진 상태로 시작한다
			Stop();
		}
	}

	void Start()
	{
		if (Socket) return;
		Socket = FUdpSocketBuilder(TEXT("RobotVizRx"))
			.AsNonBlocking()
			.AsReusable()
			.BoundToPort(Port)
			.WithReceiveBufferSize(1 << 16)
			.Build();
		if (!Socket)
		{
			UE_LOG(LogTemp, Warning, TEXT("[RobotViz] UDP %d 바인딩 실패 — 다른 프로그램이 쓰고 있거나 방화벽"), Port);
			return;
		}
		RxCount = RxCountAtLog = BadCount = 0;
		PostTickHandle = FWorldDelegates::OnWorldPostActorTick.AddStatic(&PostActorTick);
		CleanupHandle = FWorldDelegates::OnWorldCleanup.AddStatic(&OnWorldCleanup);
		UE_LOG(LogTemp, Display, TEXT("[RobotViz] ON — UDP %d 대기 (패킷 %d 바이트, %s)"), Port, PacketSize,
			StyleMode == 1 ? TEXT("솔리드") : TEXT("디버그 선"));
		// 몸이 판을 가리므로 표시를 켤 때 몸 숨기기도 켠다 (teleop.RobotVizHideBody 0 으로 끈다). 소품 숨기기는 기본 끔.
		// 폰이 아직 없으면 1초 갱신(ApplyHideBody)이 나중에 숨기고 "몸 숨김" 줄을 찍는다.
		bHideBody = true;
		ApplyHideBody(FindPlayWorld());
		UE_LOG(LogTemp, Display, TEXT("[RobotViz] HideBody = 1 (기본 켬, 몸 메시 %d개 숨김, 송신은 그대로. 끄려면 teleop.RobotVizHideBody 0)"), HiddenBody.Num());
	}

	FAutoConsoleCommand CmdOn(
		TEXT("teleop.RobotViz"),
		TEXT("로봇 집게 표시: teleop.RobotViz 1 = 켜기 (아바타 몸 숨기기도 켠다), 0 = 끄기 (워치독 --robot-viz <이 PC>:5022)"),
		FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Args)
		{
			if (Args.Num() > 0 && FCString::Atoi(*Args[0]) != 0)
			{
				Start();
			}
			else
			{
				// 끄면 판도 안 그리고 1초 갱신도 멈추므로 숨긴 것을 되돌린다
				if (bHideBody || bHideProps)
				{
					UE_LOG(LogTemp, Display, TEXT("[RobotViz] HideBody = 0, HideProps = 0 (몸 메시 %d개, 소품 %d개를 되돌렸다)"), HiddenBody.Num(), HiddenProps.Num());
				}
				RestoreList(HiddenProps);
				RestoreList(HiddenBody);
				ForgetHidden();
				Stop();
			}
		}));

	FAutoConsoleCommand CmdHideProps(
		TEXT("teleop.RobotVizHideProps"),
		TEXT("판을 가리는 레벨 소품 숨기기: 1 = 켜기, 0 = 되돌리기 (기본). 실행 중에만, 저장되지 않는다. 바닥·벽·하늘은 빼고, 숨긴 이름을 로그에 찍는다. teleop.RobotViz 1 이 필요하다"),
		FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Args)
		{
			const bool bOn = Args.Num() > 0 && FCString::Atoi(*Args[0]) != 0;
			UWorld* World = FindPlayWorld();
			if (bOn && (!Socket || !World))
			{
				UE_LOG(LogTemp, Warning, TEXT("[RobotViz] HideProps: VR Preview 중에 teleop.RobotViz 1 을 먼저 켜라"));
				return;
			}
			bHideProps = bOn;
			if (bOn)
			{
				bPropsWaitLogged = false;
				ApplyHideProps(World);
				UE_LOG(LogTemp, Display, TEXT("[RobotViz] HideProps = 1 (숨긴 소품 %d개, 판 위치 %s)"), HiddenProps.Num(), bPlateKnown ? TEXT("앎") : TEXT("모름 — 알게 되면 숨긴다"));
			}
			else
			{
				const int32 N = HiddenProps.Num();
				RestoreList(HiddenProps);
				UE_LOG(LogTemp, Display, TEXT("[RobotViz] HideProps = 0 (소품 %d개를 되돌렸다)"), N);
			}
		}));

	FAutoConsoleCommand CmdHideBody(
		TEXT("teleop.RobotVizHideBody"),
		TEXT("폰의 아바타 몸 메시 숨기기: 1 = 켜기 (teleop.RobotViz 1 로 켤 때 기본), 0 = 되돌리기. 보이기만 끄고 몸 추적 송신은 그대로다. 손 모델·컨트롤러는 숨기지 않는다. teleop.RobotViz 1 이 필요하다"),
		FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Args)
		{
			const bool bOn = Args.Num() > 0 && FCString::Atoi(*Args[0]) != 0;
			UWorld* World = FindPlayWorld();
			if (bOn && (!Socket || !World))
			{
				UE_LOG(LogTemp, Warning, TEXT("[RobotViz] HideBody: VR Preview 중에 teleop.RobotViz 1 을 먼저 켜라"));
				return;
			}
			bHideBody = bOn;
			if (bOn)
			{
				ApplyHideBody(World);
				LogPawnMeshes(World);
				UE_LOG(LogTemp, Display, TEXT("[RobotViz] HideBody = 1 (몸 메시 %d개 숨김, 송신은 그대로)"), HiddenBody.Num());
			}
			else
			{
				const int32 N = HiddenBody.Num();
				RestoreList(HiddenBody);
				UE_LOG(LogTemp, Display, TEXT("[RobotViz] HideBody = 0 (몸 메시 %d개를 되돌렸다)"), N);
			}
		}));

	FAutoConsoleCommand CmdListNear(
		TEXT("teleop.RobotVizListNear"),
		TEXT("판 근처 레벨 메시와 폰의 메시 컴포넌트 목록을 로그에 찍는다 (아무것도 바꾸지 않는다)"),
		FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Args)
		{
			UWorld* World = FindPlayWorld();
			if (!World)
			{
				UE_LOG(LogTemp, Warning, TEXT("[RobotViz] ListNear: VR Preview 중에만 쓸 수 있다"));
				return;
			}
			LogNearProps(World);
			LogPawnMeshes(World);
		}));

	FAutoConsoleCommand CmdStyle(
		TEXT("teleop.RobotVizStyle"),
		TEXT("그리기 방식: 1 = 솔리드 도형 (기본), 0 = 디버그 선 (예비)"),
		FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Args)
		{
			if (Args.Num() > 0) StyleMode = FCString::Atoi(*Args[0]) != 0 ? 1 : 0;
			UE_LOG(LogTemp, Display, TEXT("[RobotViz] Style = %d (%s)"), StyleMode, StyleMode == 1 ? TEXT("솔리드") : TEXT("디버그 선"));
		}));

	FAutoConsoleCommand CmdRealSize(
		TEXT("teleop.RobotVizRealSize"),
		TEXT("손 길이: -1 = 워치독 옵션을 따름, 0 = 배율 크기, 1 = 실제 크기 13.3 cm (기본, 쌍 매핑 때 끝 기준으로 섞음)"),
		FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Args)
		{
			if (Args.Num() > 0) RealSizeMode = FMath::Clamp(FCString::Atoi(*Args[0]), -1, 1);
			UE_LOG(LogTemp, Display, TEXT("[RobotViz] RealSize = %d"), RealSizeMode);
		}));

	FAutoConsoleCommand CmdSmooth(
		TEXT("teleop.RobotVizSmooth"),
		TEXT("패킷 사이 부드러운 움직임: 1 = 켜기 (기본), 0 = 받은 위치 그대로"),
		FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Args)
		{
			if (Args.Num() > 0) bSmooth = FCString::Atoi(*Args[0]) != 0;
			UE_LOG(LogTemp, Display, TEXT("[RobotViz] Smooth = %d (시간 상수 %.0f ms)"), bSmooth ? 1 : 0, SmoothMs);
		}));

	FAutoConsoleCommand CmdSmoothMs(
		TEXT("teleop.RobotVizSmoothMs"),
		TEXT("부드러운 움직임 시간 상수 ms (기본 40, 5~500). 일정한 속도에서 표시가 이만큼 늦는다"),
		FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Args)
		{
			if (Args.Num() > 0) SmoothMs = FMath::Clamp(FCString::Atof(*Args[0]), 5.0f, 500.0f);
			UE_LOG(LogTemp, Display, TEXT("[RobotViz] SmoothMs = %.0f"), SmoothMs);
		}));

	FAutoConsoleCommand CmdMeas(
		TEXT("teleop.RobotVizMeas"),
		TEXT("실제(엔코더) 위치 작은 구: 1 = 켜기, 0 = 끄기 (기본)"),
		FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Args)
		{
			bShowMeas = Args.Num() > 0 && FCString::Atoi(*Args[0]) != 0;
		}));

	FAutoConsoleCommand CmdTable(
		TEXT("teleop.RobotVizTable"),
		TEXT("판과 캔 자리: 1 = 켜기, 0 = 끄기 (기본). 워치독 --table 이 보내는 RVT1 이 있어야 그린다"),
		FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Args)
		{
			bShowTable = Args.Num() > 0 && FCString::Atoi(*Args[0]) != 0;
			UE_LOG(LogTemp, Display, TEXT("[RobotViz] Table = %d (RVT1 %s, 받은 수 %llu, 자리 %d)"), bShowTable ? 1 : 0, bHaveTable ? TEXT("있음") : TEXT("없음"), TableRx, bHaveTable ? (int32)TableHdr.NumSlots : 0);
		}));

	FAutoConsoleCommand CmdCan(
		TEXT("teleop.RobotVizCan"),
		TEXT("가상 캔: <번호> 1~4 왼팔 L1~L4, 5~8 오른팔 R1~R4, 9 판 위 인계 H1, 10 공중 인계 H2, 0 = 끄기 (1 이상이면 판 표시도 켠다)"),
		FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Args)
		{
			CanSlot = Args.Num() > 0 ? FMath::Clamp(FCString::Atoi(*Args[0]), 0, MaxSlots) : 0;
			UE_LOG(LogTemp, Display, TEXT("[RobotViz] Can = %d"), CanSlot);
			if (CanSlot >= 1 && !bShowTable)
			{
				bShowTable = true;
				UE_LOG(LogTemp, Display, TEXT("[RobotViz] 캔을 켜서 판 표시도 자동으로 켰다 (Table = 1)"));
			}
		}));

	FAutoConsoleCommand CmdEcho(
		TEXT("teleop.RobotVizEcho"),
		TEXT("워치독이 받은 사람 손목 (흰 작은 구, 좌표 검증용): 1 = 켜기, 0 = 끄기 (기본)"),
		FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Args)
		{
			bShowEcho = Args.Num() > 0 && FCString::Atoi(*Args[0]) != 0;
		}));
}
