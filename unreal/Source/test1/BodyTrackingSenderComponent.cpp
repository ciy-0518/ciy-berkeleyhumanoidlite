// BodyTrackingSenderComponent.cpp
#include "BodyTrackingSenderComponent.h"

#include "Camera/CameraComponent.h"
#include "Components/SkinnedMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Common/UdpSocketBuilder.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/WorldSettings.h"
#include "IPAddress.h"
#include "IXRTrackingSystem.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

#include "OculusXRMovement.h"
#include "OculusXRMovementTypes.h"

UBodyTrackingSenderComponent::UBodyTrackingSenderComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UBodyTrackingSenderComponent::BeginPlay()
{
	Super::BeginPlay();

	// NOTE: This list must stay in the same order as TargetJoints[] in
	// GetBodyJsonAndUpdateVisual(), because the debug spheres are indexed by
	// the position in that loop.
	// The last three entries were appended for the teleoperation reference
	// frame (see _teleop/PLAN2.md 5.5). Existing entries and their order are
	// unchanged, so receivers that look joints up by name keep working.
	JointNames = {
		TEXT("Head"),
		TEXT("L_UpperArm"),
		TEXT("L_LowerArm"),
		TEXT("L_WristTwist"),
		TEXT("R_UpperArm"),
		TEXT("R_LowerArm"),
		TEXT("R_WristTwist"),
		TEXT("Chest"),
		TEXT("L_Shoulder"),
		TEXT("R_Shoulder")
	};

	// Limit UDP transmission to 15 Hz or slower, even if an older Blueprint
	// instance still contains the previous 0.033-second value.
	SendInterval = FMath::Max(SendInterval, 0.0667f);

	if (bShowInEngineDebug)
	{
		InitializeJointSpheres();
	}

	InitializeAvatarViewAlignment();

	SenderSocket = FUdpSocketBuilder(TEXT("BodyTrackingUdpSender"))
		.AsReusable();

	if (!SenderSocket)
	{
		UE_LOG(LogTemp, Error, TEXT("BodyTrackingSender: Failed to create UDP sender socket"));
		return;
	}

	ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);

	if (SocketSubsystem)
	{
		RemoteAddr = SocketSubsystem->CreateInternetAddr();

		bool bIsValidIp = false;
		RemoteAddr->SetIp(*RemoteIp, bIsValidIp);
		RemoteAddr->SetPort(RemotePort);

		if (bIsValidIp)
		{
			UE_LOG(LogTemp, Warning, TEXT("BodyTrackingSender UDP Target: %s:%d"), *RemoteIp, RemotePort);
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("BodyTrackingSender Invalid UDP IP: %s"), *RemoteIp);
		}
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("BodyTrackingSender: SocketSubsystem is null"));
	}

	TryStartBodyTracking();
}

void UBodyTrackingSenderComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (bBodyTrackingStarted && GEngine && GEngine->XRSystem.IsValid())
	{
		if (!OculusXRMovement::StopBodyTracking())
		{
			UE_LOG(LogTemp, Warning, TEXT("BodyTrackingSender: StopBodyTracking failed"));
		}
	}

	bBodyTrackingStarted = false;
	ClearJointSpheres();

	if (SenderSocket)
	{
		SenderSocket->Close();

		ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
		if (SocketSubsystem)
		{
			SocketSubsystem->DestroySocket(SenderSocket);
		}

		SenderSocket = nullptr;
	}

	Super::EndPlay(EndPlayReason);
}

void UBodyTrackingSenderComponent::TickComponent(
	float DeltaTime,
	ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction
)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	const float CurrentTime = World->GetTimeSeconds();

	UpdateAvatarViewAlignment();

	if (bBodyTrackingStarted &&
		GEngine &&
		GEngine->XRSystem.IsValid() &&
		!OculusXRMovement::IsBodyTrackingEnabled())
	{
		bBodyTrackingStarted = false;
	}

	if (!bBodyTrackingStarted)
	{
		TryStartBodyTracking();
	}

	if (CurrentTime - LastSendTime < SendInterval)
	{
		return;
	}

	FString JsonData;

	if (!bBodyTrackingStarted)
	{
		JsonData = TEXT("{\"active\":false,\"joints\":[]}");
	}
	else if (!GetBodyJsonAndUpdateVisual(JsonData, DeltaTime))
	{
		JsonData = TEXT("{\"active\":false,\"joints\":[]}");
	}

	AppendGripField(JsonData);
	SendUdpData(JsonData);
	LastSendTime = CurrentTime;
}

bool UBodyTrackingSenderComponent::TryStartBodyTracking()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return false;
	}

	const float CurrentTime = World->GetTimeSeconds();
	if (CurrentTime - LastTrackingStartAttemptTime < TrackingStartRetryInterval)
	{
		return false;
	}

	LastTrackingStartAttemptTime = CurrentTime;

	if (!GEngine || !GEngine->XRSystem.IsValid())
	{
		LogTrackingWarning(TEXT("XR system is not ready"));
		return false;
	}

	if (!OculusXRMovement::IsBodyTrackingSupported())
	{
		LogTrackingWarning(TEXT("Body tracking is not supported by the current XR runtime"));
		return false;
	}

	if (!OculusXRMovement::IsBodyTrackingEnabled() &&
		!OculusXRMovement::StartBodyTrackingByJointSet(EOculusXRBodyJointSet::UpperBody))
	{
		LogTrackingWarning(TEXT("StartBodyTrackingByJointSet failed"));
		return false;
	}

	if (!OculusXRMovement::RequestBodyTrackingFidelity(EOculusXRBodyTrackingFidelity::High))
	{
		UE_LOG(LogTemp, Warning, TEXT("BodyTrackingSender: High fidelity request was not accepted"));
	}

	bBodyTrackingStarted = true;
	UE_LOG(LogTemp, Warning, TEXT("BodyTrackingSender: Upper-body tracking started"));
	return true;
}

void UBodyTrackingSenderComponent::InitializeAvatarViewAlignment()
{
	if (!bAlignAvatarToVRCamera)
	{
		return;
	}

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	BodyAvatarMesh = nullptr;
	VRCamera = nullptr;
	bAvatarAlignmentInitialized = false;

	TArray<USkinnedMeshComponent*> SkinnedMeshes;
	Owner->GetComponents<USkinnedMeshComponent>(SkinnedMeshes);

	for (USkinnedMeshComponent* SkinnedMesh : SkinnedMeshes)
	{
		if (SkinnedMesh && SkinnedMesh->GetFName() == BodyAvatarComponentName)
		{
			BodyAvatarMesh = SkinnedMesh;
			break;
		}
	}

	if (!BodyAvatarMesh)
	{
		for (USkinnedMeshComponent* SkinnedMesh : SkinnedMeshes)
		{
			if (SkinnedMesh && SkinnedMesh->GetFName() == BodyAvatarFallbackComponentName)
			{
				BodyAvatarMesh = SkinnedMesh;
				break;
			}
		}
	}

	TArray<UCameraComponent*> CameraComponents;
	Owner->GetComponents<UCameraComponent>(CameraComponents);

	for (UCameraComponent* CameraComponent : CameraComponents)
	{
		if (CameraComponent && CameraComponent->GetFName() == CameraComponentName)
		{
			VRCamera = CameraComponent;
			break;
		}
	}

	if (!VRCamera && CameraComponents.Num() > 0)
	{
		VRCamera = CameraComponents[0];
	}

	if (!BodyAvatarMesh)
	{
		FString AvailableMeshNames;
		for (USkinnedMeshComponent* SkinnedMesh : SkinnedMeshes)
		{
			if (SkinnedMesh)
			{
				AvailableMeshNames += SkinnedMesh->GetName() + TEXT(" ");
			}
		}

		UE_LOG(
			LogTemp,
			Warning,
			TEXT("BodyTrackingSender: Skinned Mesh Component named %s or %s was not found. Available: %s"),
			*BodyAvatarComponentName.ToString(),
			*BodyAvatarFallbackComponentName.ToString(),
			*AvailableMeshNames
		);
		return;
	}

	if (!VRCamera)
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("BodyTrackingSender: Camera Component named %s was not found."),
			*CameraComponentName.ToString()
		);
		return;
	}

	BodyAvatarMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	BodyAvatarMesh->SetVisibility(true, true);
	BodyAvatarMesh->SetHiddenInGame(false, true);

	if (bHideAvatarHead && BodyAvatarMesh->GetBoneIndex(AvatarEyeBoneName) != INDEX_NONE)
	{
		BodyAvatarMesh->HideBoneByName(AvatarEyeBoneName, PBO_None);
	}

	bAvatarAlignmentInitialized = true;

	UE_LOG(
		LogTemp,
		Warning,
		TEXT("BodyTrackingSender: Aligning %s to camera %s with camera offset %s"),
		*BodyAvatarMesh->GetName(),
		*VRCamera->GetName(),
		*AvatarCameraOffset.ToString()
	);
}

void UBodyTrackingSenderComponent::UpdateAvatarViewAlignment()
{
	if (!bAlignAvatarToVRCamera)
	{
		return;
	}

	if (!BodyAvatarMesh || !VRCamera || !bAvatarAlignmentInitialized)
	{
		InitializeAvatarViewAlignment();
	}

	if (!BodyAvatarMesh || !VRCamera || !bAvatarAlignmentInitialized)
	{
		return;
	}

	const FVector CameraLocation = VRCamera->GetComponentLocation();
	const FRotator CameraRotation = VRCamera->GetComponentRotation();

	FRotator AvatarRotation = CameraRotation;
	if (bUseCameraYawOnly)
	{
		AvatarRotation.Pitch = 0.0f;
		AvatarRotation.Roll = 0.0f;
	}

	AvatarRotation.Yaw += AvatarYawOffsetDegrees;

	const FVector AvatarLocation =
		CameraLocation +
		AvatarRotation.RotateVector(AvatarCameraOffset) +
		AvatarWorldOffset;

	BodyAvatarMesh->SetWorldLocationAndRotation(AvatarLocation, AvatarRotation);
}

void UBodyTrackingSenderComponent::InitializeJointSpheres()
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(
		nullptr,
		TEXT("/Engine/BasicShapes/Sphere.Sphere")
	);

	if (!SphereMesh)
	{
		UE_LOG(LogTemp, Error, TEXT("BodyTrackingSender: Could not load sphere mesh"));
		return;
	}

	JointSpheres.Empty();

	for (int32 i = 0; i < JointNames.Num(); ++i)
	{
		UStaticMeshComponent* SphereComp = NewObject<UStaticMeshComponent>(Owner);
		if (!SphereComp)
		{
			continue;
		}

		SphereComp->SetStaticMesh(SphereMesh);
		SphereComp->SetWorldScale3D(FVector(SphereScale));
		SphereComp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		SphereComp->SetVisibility(true);
		SphereComp->RegisterComponent();

		if (Owner->GetRootComponent())
		{
			SphereComp->AttachToComponent(
				Owner->GetRootComponent(),
				FAttachmentTransformRules::KeepWorldTransform
			);
		}

		JointSpheres.Add(SphereComp);
	}
}

void UBodyTrackingSenderComponent::ClearJointSpheres()
{
	for (UStaticMeshComponent* SphereComp : JointSpheres)
	{
		if (SphereComp)
		{
			SphereComp->DestroyComponent();
		}
	}

	JointSpheres.Empty();
}

bool UBodyTrackingSenderComponent::GetBodyJsonAndUpdateVisual(FString& OutJson, float DeltaTime)
{
	AActor* Owner = GetOwner();
	UWorld* World = GetWorld();

	if (!Owner || !World)
	{
		return false;
	}

	FOculusXRBodyState BodyState;
	const float WorldToMeters = World->GetWorldSettings()->WorldToMeters;

	if (!OculusXRMovement::GetBodyState(BodyState, WorldToMeters))
	{
		LogTrackingWarning(TEXT("GetBodyState failed"));
		return false;
	}

	if (!BodyState.IsActive)
	{
		LogTrackingWarning(TEXT("BodyState is not active"));
		return false;
	}

	struct FTargetJoint
	{
		const TCHAR* Name;
		EOculusXRBoneID BoneId;
	};

	// The first seven entries are the original payload and are left untouched:
	// existing receivers look joints up by name, so appending is backwards
	// compatible.
	//
	// Chest and the two shoulders were appended so the receiver can express arm
	// rotations relative to the torso instead of in absolute tracking space.
	// Without a torso reference, turning the body changes the shoulder angles
	// even when the arm has not moved.
	//
	// Enumerator names verified against
	// Engine/Plugins/Marketplace/MetaXR/Source/OculusXRMovement/Public/OculusXRMovementTypes.h
	// (BodyChest = 5, BodyLeftShoulder = 8, BodyRightShoulder = 13).
	//
	// Hips and SpineMiddle were appended on 2026-09-22 (REPORT_16 sec.6, REPORT_19
	// sec.3): Chest and the shoulders rotate with the arm, so they cannot serve as
	// a per-frame torso frame. Bones further from the arm are needed.
	// Only these two are added, not all four spine bones: at ~94 bytes per joint
	// the JSON packet grows from a measured 969 B to about 1157 B, which still fits
	// the Tailscale (WireGuard) tunnel MTU of ~1252 B payload. Adding SpineLower and
	// SpineUpper as well would reach ~1345 B and fragment on that path
	// (_teleop/REVIEW_20260919_unreal.md sec.4.2).
	// Enumerator names verified in the same header: BodyHips = 1, BodySpineMiddle = 3.
	static const FTargetJoint TargetJoints[] = {
		{ TEXT("Head"),         EOculusXRBoneID::BodyHead },
		{ TEXT("L_UpperArm"),   EOculusXRBoneID::BodyLeftArmUpper },
		{ TEXT("L_LowerArm"),   EOculusXRBoneID::BodyLeftArmLower },
		{ TEXT("L_WristTwist"), EOculusXRBoneID::BodyLeftHandWristTwist },
		{ TEXT("R_UpperArm"),   EOculusXRBoneID::BodyRightArmUpper },
		{ TEXT("R_LowerArm"),   EOculusXRBoneID::BodyRightArmLower },
		{ TEXT("R_WristTwist"), EOculusXRBoneID::BodyRightHandWristTwist },
		{ TEXT("Chest"),        EOculusXRBoneID::BodyChest },
		{ TEXT("L_Shoulder"),   EOculusXRBoneID::BodyLeftShoulder },
		{ TEXT("R_Shoulder"),   EOculusXRBoneID::BodyRightShoulder },
		{ TEXT("Hips"),         EOculusXRBoneID::BodyHips },
		{ TEXT("SpineMiddle"),  EOculusXRBoneID::BodySpineMiddle }
	};

	const auto GetValidJoint = [&BodyState](EOculusXRBoneID BoneId) -> const FOculusXRBodyJoint*
	{
		const int32 BoneIndex = static_cast<int32>(BoneId);
		if (!BodyState.Joints.IsValidIndex(BoneIndex))
		{
			return nullptr;
		}

		const FOculusXRBodyJoint& Joint = BodyState.Joints[BoneIndex];
		return Joint.bIsValid ? &Joint : nullptr;
	};

	FString ScreenText;
	FString JsonString = TEXT("{\"active\":true,\"joints\":[");

	bool bFirstJoint = true;

	for (int32 i = 0; i < UE_ARRAY_COUNT(TargetJoints); ++i)
	{
		const FTargetJoint& Target = TargetJoints[i];
		const FOculusXRBodyJoint* Joint = GetValidJoint(Target.BoneId);

		if (!Joint)
		{
			continue;
		}

		const FVector RawPosition = Joint->Position;
		FQuat RawRotation = Joint->Orientation.Quaternion();
		RawRotation.Normalize();

		const FVector WorldPosition = Owner->GetActorTransform().TransformPosition(
			(RawPosition * PositionScale) + VisualOffset
		);

		if (bShowInEngineDebug && JointSpheres.IsValidIndex(i) && JointSpheres[i])
		{
			JointSpheres[i]->SetWorldLocation(WorldPosition);
			JointSpheres[i]->SetWorldScale3D(FVector(SphereScale));
		}

		if (bShowInEngineDebug)
		{
			DrawDebugString(
				World,
				WorldPosition + FVector(0.0f, 0.0f, 10.0f),
				Target.Name,
				nullptr,
				FColor::Green,
				0.0f,
				true
			);
		}

		if (!bFirstJoint)
		{
			JsonString += TEXT(",");
		}

		bFirstJoint = false;

		JsonString += FString::Printf(
			TEXT("{\"name\":\"%s\",")
			TEXT("\"pos\":[%.3f,%.3f,%.3f],")
			TEXT("\"rot\":[%.5f,%.5f,%.5f,%.5f]}"),
			Target.Name,
			RawPosition.X,
			RawPosition.Y,
			RawPosition.Z,
			RawRotation.X,
			RawRotation.Y,
			RawRotation.Z,
			RawRotation.W
		);

		ScreenText += FString::Printf(
			TEXT("%s: %.1f %.1f %.1f\n"),
			Target.Name,
			RawPosition.X,
			RawPosition.Y,
			RawPosition.Z
		);

	}

	JsonString += TEXT("]}");
	OutJson = JsonString;

	if (bShowInEngineDebug && GEngine && !ScreenText.IsEmpty())
	{
		GEngine->AddOnScreenDebugMessage(
			1,
			0.05f,
			FColor::Green,
			ScreenText
		);
	}

	return true;
}

bool UBodyTrackingSenderComponent::GetFingerTipLocations(
	const FOculusXRBodyState& BodyState,
	bool bLeftHand,
	FVector& OutThumbTip,
	FVector& OutIndexTip
) const
{
	const EOculusXRBoneID ThumbTipBone = bLeftHand
		? EOculusXRBoneID::BodyLeftHandThumbTip
		: EOculusXRBoneID::BodyRightHandThumbTip;
	const EOculusXRBoneID IndexTipBone = bLeftHand
		? EOculusXRBoneID::BodyLeftHandIndexTip
		: EOculusXRBoneID::BodyRightHandIndexTip;

	const int32 ThumbIndex = static_cast<int32>(ThumbTipBone);
	const int32 IndexIndex = static_cast<int32>(IndexTipBone);

	if (!BodyState.Joints.IsValidIndex(ThumbIndex) || !BodyState.Joints.IsValidIndex(IndexIndex))
	{
		return false;
	}

	const FOculusXRBodyJoint& ThumbJoint = BodyState.Joints[ThumbIndex];
	const FOculusXRBodyJoint& IndexJoint = BodyState.Joints[IndexIndex];

	if (!ThumbJoint.bIsValid || !IndexJoint.bIsValid)
	{
		return false;
	}

	OutThumbTip = ThumbJoint.Position;
	OutIndexTip = IndexJoint.Position;
	return true;
}

UBodyTrackingSenderComponent::FGripCommand UBodyTrackingSenderComponent::UpdateGripCommand(
	const FOculusXRBodyState& BodyState,
	bool bLeftHand,
	float DeltaTime
)
{
	FGripCommand Command;

	float& FilteredHandGap01 = bLeftHand ? LeftFilteredHandGap01 : RightFilteredHandGap01;
	float& PrevJGTargetDeg = bLeftHand ? LeftPrevJGTargetDeg : RightPrevJGTargetDeg;

	FVector ThumbTip;
	FVector IndexTip;
	const bool bTrackingValid = GetFingerTipLocations(BodyState, bLeftHand, ThumbTip, IndexTip);

	Command.bTrackingValid = bTrackingValid;

	if (!bTrackingValid)
	{
		Command.Grip01 = 1.0f - FilteredHandGap01;
		Command.GripState = UpdateGripState(bLeftHand, false, Command.Grip01, DeltaTime);
		Command.JGTargetDeg = PrevJGTargetDeg;
		Command.FilteredHandGap01 = FilteredHandGap01;
		return Command;
	}

	const float RawPinchDistanceCm = FVector::Distance(ThumbTip, IndexTip);
	Command.RawPinchDistanceMm = RawPinchDistanceCm * 10.0f;

	const float Denominator = FMath::Max(HandOpenGapMm - HandClosedGapMm, 1.0f);
	const float CurrentHandGap01 = FMath::Clamp(
		(Command.RawPinchDistanceMm - HandClosedGapMm) / Denominator,
		0.0f,
		1.0f
	);

	const float Alpha = FMath::Clamp(GripFilterAlpha, 0.0f, 1.0f);
	FilteredHandGap01 = FMath::Lerp(FilteredHandGap01, CurrentHandGap01, Alpha);

	Command.FilteredHandGap01 = FilteredHandGap01;
	Command.Grip01 = 1.0f - FilteredHandGap01;
	Command.GripState = UpdateGripState(bLeftHand, true, Command.Grip01, DeltaTime);

	const float DesiredJGTargetDeg = MapHandGapToJGAngle(bLeftHand, FilteredHandGap01);
	const float MaxDelta = FMath::Max(MaxJGSpeedDegPerSec, 0.0f) * FMath::Max(DeltaTime, 0.0f);

	Command.JGTargetDeg = FMath::Clamp(
		DesiredJGTargetDeg,
		PrevJGTargetDeg - MaxDelta,
		PrevJGTargetDeg + MaxDelta
	);

	PrevJGTargetDeg = Command.JGTargetDeg;
	return Command;
}

float UBodyTrackingSenderComponent::MapHandGapToJGAngle(bool bLeftHand, float HandGap01) const
{
	const bool bInvert = bLeftHand ? bInvertLeftGripper : bInvertRightGripper;
	const float ClosedDeg = bLeftHand ? LeftJGClosedDeg : RightJGClosedDeg;
	const float OpenDeg = bLeftHand ? LeftJGOpenDeg : RightJGOpenDeg;

	const float Alpha = FMath::Clamp(HandGap01, 0.0f, 1.0f);
	const float FromDeg = bInvert ? OpenDeg : ClosedDeg;
	const float ToDeg = bInvert ? ClosedDeg : OpenDeg;
	const float MinDeg = FMath::Min(ClosedDeg, OpenDeg);
	const float MaxDeg = FMath::Max(ClosedDeg, OpenDeg);

	return FMath::Clamp(FMath::Lerp(FromDeg, ToDeg, Alpha), MinDeg, MaxDeg);
}

int32 UBodyTrackingSenderComponent::UpdateGripState(
	bool bLeftHand,
	bool bTrackingValid,
	float Grip01,
	float DeltaTime
)
{
	float& HoldTimer = bLeftHand ? LeftHoldTimer : RightHoldTimer;
	int32& State = bLeftHand ? LeftGripState : RightGripState;

	if (!bTrackingValid)
	{
		State = 4;
		return State;
	}

	const int32 PreviousState = State;

	if ((PreviousState == 1 || PreviousState == 2) && Grip01 < 0.35f)
	{
		HoldTimer = 0.0f;
		State = 3;
		return State;
	}

	if (Grip01 < 0.25f)
	{
		HoldTimer = 0.0f;
		State = 0;
		return State;
	}

	if (Grip01 >= 0.75f)
	{
		HoldTimer += FMath::Max(DeltaTime, 0.0f);
		State = HoldTimer >= GripHoldThresholdSeconds ? 2 : 1;
		return State;
	}

	HoldTimer = 0.0f;
	State = 1;
	return State;
}

FString UBodyTrackingSenderComponent::GripCommandToJsonFields(
	const FGripCommand& LeftGrip,
	const FGripCommand& RightGrip,
	float Timestamp
) const
{
	return FString::Printf(
		TEXT(",\"end_effectors\":{")
		TEXT("\"left\":{\"grip_01\":%.3f,\"grip_state\":%d,\"jg_target_deg\":%.3f,")
		TEXT("\"raw_pinch_distance_mm\":%.3f,\"filtered_hand_gap_01\":%.3f,\"tracking_valid\":%s},")
		TEXT("\"right\":{\"grip_01\":%.3f,\"grip_state\":%d,\"jg_target_deg\":%.3f,")
		TEXT("\"raw_pinch_distance_mm\":%.3f,\"filtered_hand_gap_01\":%.3f,\"tracking_valid\":%s}},")
		TEXT("\"left_grip_01\":%.3f,\"left_grip_state\":%d,\"left_jg_target_deg\":%.3f,")
		TEXT("\"right_grip_01\":%.3f,\"right_grip_state\":%d,\"right_jg_target_deg\":%.3f,")
		TEXT("\"timestamp\":%.3f"),
		LeftGrip.Grip01,
		LeftGrip.GripState,
		LeftGrip.JGTargetDeg,
		LeftGrip.RawPinchDistanceMm,
		LeftGrip.FilteredHandGap01,
		LeftGrip.bTrackingValid ? TEXT("true") : TEXT("false"),
		RightGrip.Grip01,
		RightGrip.GripState,
		RightGrip.JGTargetDeg,
		RightGrip.RawPinchDistanceMm,
		RightGrip.FilteredHandGap01,
		RightGrip.bTrackingValid ? TEXT("true") : TEXT("false"),
		LeftGrip.Grip01,
		LeftGrip.GripState,
		LeftGrip.JGTargetDeg,
		RightGrip.Grip01,
		RightGrip.GripState,
		RightGrip.JGTargetDeg,
		Timestamp
	);
}

// 그리퍼용 입력을 최상위 "grip" 키로 덧붙인다. 손마다 [src, pinch_cm] 배열:
//   "grip":{"L":["hand",12.3],"R":["ctrl",2.3]}
// "active"/"joints" 는 한 글자도 바꾸지 않고, 마지막 '}' 앞에만 끼워 넣는다
// (로봇 PC 의 기존 수신기는 json 으로 읽고 .get() 으로 키를 찾으므로 그대로 동작한다).
//   src      : OpenXR 현재 인터랙션 프로파일. "controller" 포함 -> "ctrl",
//              "hand_interaction" 포함 -> "hand", 그 밖 -> "none".
//              (IsHandPositionValid 는 컨트롤러를 쥐어도 1 이라 쓰지 않는다. GripInputProbe 2026-10-06)
//   pinch_cm : 바디트래킹 ThumbTip-IndexTip 거리 원시값, 소수 1자리.
//              필터·평균·직전값 유지·보간 없음. 손끝 본이 무효면 null.
// 트리거는 넣지 않는다. 짧은 배열 형식은 Tailscale MTU(~1252 B) 여유 때문이다 —
// 12본 실측 최대 1162 B 에 객체 형식을 붙이면 넘는다 (2026-10-07).
// 비활성 패킷({"active":false,...})에도 붙인다 — src 는 바디트래킹과 무관하게 알 수 있다.
void UBodyTrackingSenderComponent::AppendGripField(FString& JsonData)
{
	if (!JsonData.EndsWith(TEXT("}")))
	{
		return;
	}

	FOculusXRBodyState BodyState;
	bool bBodyValid = false;
	float CmPerUnit = 1.0f;

	UWorld* World = GetWorld();
	if (bBodyTrackingStarted && World && World->GetWorldSettings())
	{
		const float WorldToMeters = World->GetWorldSettings()->WorldToMeters;
		bBodyValid = WorldToMeters > 0.0f
			&& OculusXRMovement::GetBodyState(BodyState, WorldToMeters)
			&& BodyState.IsActive;
		CmPerUnit = bBodyValid ? 100.0f / WorldToMeters : 1.0f;
	}

	const auto HandField = [&](EControllerHand Hand, bool bLeftHand) -> FString
	{
		FString Profile;
		if (GEngine && GEngine->XRSystem.IsValid())
		{
			GEngine->XRSystem->GetCurrentInteractionProfile(Hand, Profile);
		}

		const TCHAR* Source = TEXT("none");
		if (Profile.Contains(TEXT("controller")))
		{
			Source = TEXT("ctrl");
		}
		else if (Profile.Contains(TEXT("hand_interaction")))
		{
			Source = TEXT("hand");
		}

		FString Pinch = TEXT("null");
		FVector ThumbTip;
		FVector IndexTip;
		if (bBodyValid && GetFingerTipLocations(BodyState, bLeftHand, ThumbTip, IndexTip))
		{
			const float PinchCm = FVector::Distance(ThumbTip, IndexTip) * CmPerUnit;
			if (FMath::IsFinite(PinchCm))
			{
				Pinch = FString::Printf(TEXT("%.1f"), PinchCm);
			}
		}

		return FString::Printf(TEXT("[\"%s\",%s]"), Source, *Pinch);
	};

	JsonData.LeftChopInline(1);   // 마지막 '}'
	JsonData += TEXT(",\"grip\":{\"L\":");
	JsonData += HandField(EControllerHand::Left, true);
	JsonData += TEXT(",\"R\":");
	JsonData += HandField(EControllerHand::Right, false);
	JsonData += TEXT("}}");
}

void UBodyTrackingSenderComponent::SendUdpData(const FString& JsonData)
{
	if (!SenderSocket || !RemoteAddr.IsValid())
	{
		return;
	}

	FTCHARToUTF8 Convert(*JsonData);
	int32 BytesSent = 0;

	const bool bSent = SenderSocket->SendTo(
		reinterpret_cast<const uint8*>(Convert.Get()),
		Convert.Length(),
		BytesSent,
		*RemoteAddr
	);

	if (!bSent)
	{
		// REPORT_01 은 이 지점에서 880회 실패를 세었지만 원인 코드가 없어
		// 이유를 알 수 없었다. 이제 에러코드를 남긴다.
		// 로그는 1초 1회로 스로틀한다. REPORT_01 의 집계는 "스로틀 없음 =
		// 로그 줄 수가 곧 실패 횟수"에 기대고 있었으므로, 그 성질을 잃지
		// 않도록 누적 합계(total)를 같이 찍는다.
		++SendFailCount;

		UWorld* World = GetWorld();
		const float CurrentTime = World ? World->GetTimeSeconds() : 0.0f;

		if (CurrentTime - LastSendFailLogTime >= 1.0f)
		{
			ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);

			const ESocketErrors ErrorCode = SocketSubsystem
				? SocketSubsystem->GetLastErrorCode()
				: SE_NO_ERROR;
			const TCHAR* ErrorText = SocketSubsystem
				? SocketSubsystem->GetSocketError(ErrorCode)
				: TEXT("SocketSubsystem is null");

			UE_LOG(
				LogTemp,
				Warning,
				TEXT("BodyTrackingSender: UDP SendTo failed (error %d %s, sent %d/%d bytes, total %lld)"),
				static_cast<int32>(ErrorCode),
				ErrorText,
				BytesSent,
				Convert.Length(),
				SendFailCount
			);

			LastSendFailLogTime = CurrentTime;
		}
	}
}

void UBodyTrackingSenderComponent::LogTrackingWarning(const TCHAR* Message)
{
	UWorld* World = GetWorld();
	const float CurrentTime = World ? World->GetTimeSeconds() : 0.0f;

	if (CurrentTime - LastTrackingWarningLogTime >= 1.0f)
	{
		UE_LOG(LogTemp, Warning, TEXT("BodyTrackingSender: %s"), Message);
		LastTrackingWarningLogTime = CurrentTime;
	}
}
