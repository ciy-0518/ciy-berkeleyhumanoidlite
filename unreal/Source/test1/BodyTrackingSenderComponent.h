// BodyTrackingSenderComponent.h
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"

#include "BodyTrackingSenderComponent.generated.h"

class UStaticMeshComponent;
class USkinnedMeshComponent;
class UCameraComponent;
class FSocket;
class FInternetAddr;

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class TEST1_API UBodyTrackingSenderComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UBodyTrackingSenderComponent();

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
	virtual void TickComponent(
		float DeltaTime,
		ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction
	) override;

private:
	void InitializeJointSpheres();
	void ClearJointSpheres();

	bool TryStartBodyTracking();
	bool GetBodyJsonAndUpdateVisual(FString& OutJson, float DeltaTime);
	void InitializeAvatarViewAlignment();
	void UpdateAvatarViewAlignment();
	void SendUdpData(const FString& JsonData);
	void AppendGripField(FString& JsonData);
	void LogTrackingWarning(const TCHAR* Message);

private:
	struct FGripCommand
	{
		float Grip01 = 0.0f;
		int32 GripState = 4;
		float JGTargetDeg = 0.0f;
		float RawPinchDistanceMm = -1.0f;
		float FilteredHandGap01 = 0.0f;
		bool bTrackingValid = false;
	};

	bool GetFingerTipLocations(
		const struct FOculusXRBodyState& BodyState,
		bool bLeftHand,
		FVector& OutThumbTip,
		FVector& OutIndexTip
	) const;

	FGripCommand UpdateGripCommand(
		const struct FOculusXRBodyState& BodyState,
		bool bLeftHand,
		float DeltaTime
	);

	float MapHandGapToJGAngle(bool bLeftHand, float HandGap01) const;
	int32 UpdateGripState(bool bLeftHand, bool bTrackingValid, float Grip01, float DeltaTime);
	FString GripCommandToJsonFields(const FGripCommand& LeftGrip, const FGripCommand& RightGrip, float Timestamp) const;

	UPROPERTY()
	TArray<UStaticMeshComponent*> JointSpheres;

	UPROPERTY(Transient)
	TObjectPtr<USkinnedMeshComponent> BodyAvatarMesh = nullptr;

	UPROPERTY(Transient)
	TObjectPtr<UCameraComponent> VRCamera = nullptr;

	TArray<FString> JointNames;

	FSocket* SenderSocket = nullptr;
	TSharedPtr<FInternetAddr> RemoteAddr;

	bool bBodyTrackingStarted = false;
	float LastTrackingStartAttemptTime = -1000.0f;
	float LastTrackingWarningLogTime = -1000.0f;
	float LastSendTime = -1000.0f;
	float LastLogTime = -1000.0f;

	// SendTo 실패 진단 (REPORT_01 §7.5 / 권고 6).
	// 로그는 1초에 1회로 스로틀하되, 스로틀 때문에 횟수를 잃지 않도록
	// 누적 합계를 로그에 같이 싣는다.
	float LastSendFailLogTime = -1000.0f;
	int64 SendFailCount = 0;

	UPROPERTY(EditAnywhere, Category = "Body Tracking")
	float TrackingStartRetryInterval = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Body Tracking Debug")
	bool bShowInEngineDebug = false;

	UPROPERTY(EditAnywhere, Category = "Body Tracking Debug")
	float PositionScale = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Body Tracking Debug")
	FVector VisualOffset = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "Body Tracking Debug")
	float SphereScale = 0.06f;

	UPROPERTY(EditAnywhere, Category = "End Effector Grip")
	float HandClosedGapMm = 15.0f;

	UPROPERTY(EditAnywhere, Category = "End Effector Grip")
	float HandOpenGapMm = 90.0f;

	UPROPERTY(EditAnywhere, Category = "End Effector Grip")
	float LeftJGClosedDeg = 4.42f;

	UPROPERTY(EditAnywhere, Category = "End Effector Grip")
	float LeftJGOpenDeg = 94.65f;

	UPROPERTY(EditAnywhere, Category = "End Effector Grip")
	float RightJGClosedDeg = -60.0f;

	UPROPERTY(EditAnywhere, Category = "End Effector Grip")
	float RightJGOpenDeg = 52.0f;

	UPROPERTY(EditAnywhere, Category = "End Effector Grip")
	bool bInvertLeftGripper = false;

	UPROPERTY(EditAnywhere, Category = "End Effector Grip")
	bool bInvertRightGripper = false;

	UPROPERTY(EditAnywhere, Category = "End Effector Grip")
	float GripFilterAlpha = 0.2f;

	UPROPERTY(EditAnywhere, Category = "End Effector Grip")
	float MaxJGSpeedDegPerSec = 120.0f;

	UPROPERTY(EditAnywhere, Category = "End Effector Grip")
	float GripHoldThresholdSeconds = 0.2f;

	float LeftFilteredHandGap01 = 0.0f;
	float RightFilteredHandGap01 = 0.0f;
	float LeftPrevJGTargetDeg = 34.0f;
	float RightPrevJGTargetDeg = -20.0f;
	float LeftHoldTimer = 0.0f;
	float RightHoldTimer = 0.0f;
	int32 LeftGripState = 4;
	int32 RightGripState = 4;

	UPROPERTY(EditAnywhere, Category = "Body Avatar View Alignment")
	bool bAlignAvatarToVRCamera = true;

	UPROPERTY(EditAnywhere, Category = "Body Avatar View Alignment")
	FName BodyAvatarComponentName = TEXT("BodyAvatarMesh");

	UPROPERTY(EditAnywhere, Category = "Body Avatar View Alignment")
	FName BodyAvatarFallbackComponentName = TEXT("PoseableMesh");

	UPROPERTY(EditAnywhere, Category = "Body Avatar View Alignment")
	FName CameraComponentName = TEXT("Camera");

	UPROPERTY(EditAnywhere, Category = "Body Avatar View Alignment")
	FName AvatarEyeBoneName = TEXT("Head");

	UPROPERTY(EditAnywhere, Category = "Body Avatar View Alignment")
	bool bUseCameraYawOnly = true;

	UPROPERTY(EditAnywhere, Category = "Body Avatar View Alignment")
	bool bHideAvatarHead = true;

	UPROPERTY(EditAnywhere, Category = "Body Avatar View Alignment")
	float AvatarYawOffsetDegrees = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Body Avatar View Alignment")
	FVector AvatarCameraOffset = FVector(0.0f, 0.0f, -160.0f);

	UPROPERTY(EditAnywhere, Category = "Body Avatar View Alignment")
	FVector AvatarWorldOffset = FVector::ZeroVector;

	bool bAvatarAlignmentInitialized = false;

	UPROPERTY(EditAnywhere, Category = "UDP")
	FString RemoteIp = TEXT("ROBOT_PC_IP");

	UPROPERTY(EditAnywhere, Category = "UDP")
	int32 RemotePort = 5000;

	UPROPERTY(EditAnywhere, Category = "UDP", meta = (ClampMin = "0.0667", UIMin = "0.0667"))
	float SendInterval = 0.0667f;
};
