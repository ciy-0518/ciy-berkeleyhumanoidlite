#include "CsvMotionRecorder.h"

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "OculusXRMovement.h"
#include "OculusXRMovementTypes.h"

FCsvMotionRecorder::FCsvMotionRecorder()
	: bIsRecording(false)
	, SampleIntervalSeconds(0.1f)
{
}

void FCsvMotionRecorder::Start(UWorld* World)
{
	if (bIsRecording)
	{
		return;
	}

	RecordingWorld = World;
	OutputPath = FPaths::ProjectSavedDir() / TEXT("MotionLog.csv");
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(OutputPath), true);

	Rows.Reset();
	Rows.Add(
		TEXT("TimeSeconds,Frame,TrackingActive,")
		TEXT("HeadX,HeadY,HeadZ,")
		TEXT("LUpperArmX,LUpperArmY,LUpperArmZ,LLowerArmX,LLowerArmY,LLowerArmZ,LWristX,LWristY,LWristZ,")
		TEXT("RUpperArmX,RUpperArmY,RUpperArmZ,RLowerArmX,RLowerArmY,RLowerArmZ,RWristX,RWristY,RWristZ,")
		TEXT("LeftUpperArmLength,LeftForearmLength,LeftArmLength,")
		TEXT("RightUpperArmLength,RightForearmLength,RightArmLength")
	);

	IFileManager::Get().Delete(*OutputPath, false, true, true);

	if (!FFileHelper::SaveStringArrayToFile(Rows, *OutputPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to create CSV motion log: %s"), *OutputPath);
	}

	TickHandle = FTSTicker::GetCoreTicker().AddTicker(
		FTickerDelegate::CreateRaw(this, &FCsvMotionRecorder::Tick),
		SampleIntervalSeconds);

	bIsRecording = true;
	Tick(0.0f);
}

void FCsvMotionRecorder::Stop()
{
	if (!bIsRecording)
	{
		return;
	}

	if (TickHandle.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(TickHandle);
		TickHandle.Reset();
	}

	bIsRecording = false;
	RecordingWorld.Reset();

	if (!OutputPath.IsEmpty())
	{
		if (!FFileHelper::SaveStringArrayToFile(Rows, *OutputPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
		{
			UE_LOG(LogTemp, Warning, TEXT("Failed to save CSV motion log: %s"), *OutputPath);
		}
	}
}

bool FCsvMotionRecorder::Tick(float DeltaTime)
{
	UWorld* World = RecordingWorld.IsValid() ? RecordingWorld.Get() : FindGameWorld();
	if (!World)
	{
		return true;
	}

	const auto AddInactiveTrackingRow = [this, World]()
	{
		TArray<FString> Columns;
		Columns.Reserve(30);
		Columns.Add(FString::Printf(TEXT("%.3f"), World->GetTimeSeconds()));
		Columns.Add(LexToString(GFrameCounter));
		Columns.Add(TEXT("false"));
		for (int32 i = 0; i < 27; ++i)
		{
			Columns.Add(TEXT(""));
		}

		const FString Row = FString::Join(Columns, TEXT(","));
		Rows.Add(Row);
		FFileHelper::SaveStringToFile(Row + LINE_TERMINATOR, *OutputPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append);
	};

	if (!GEngine || !GEngine->XRSystem.IsValid() || !OculusXRMovement::IsBodyTrackingEnabled())
	{
		AddInactiveTrackingRow();
		return true;
	}

	FOculusXRBodyState BodyState;
	const float WorldToMeters = World->GetWorldSettings()->WorldToMeters;
	if (!OculusXRMovement::GetBodyState(BodyState, WorldToMeters) || !BodyState.IsActive)
	{
		AddInactiveTrackingRow();
		return true;
	}

	const auto GetJointPosition = [&BodyState](EOculusXRBoneID BoneId, FVector& OutPosition) -> bool
	{
		const int32 BoneIndex = static_cast<int32>(BoneId);
		if (!BodyState.Joints.IsValidIndex(BoneIndex))
		{
			return false;
		}

		const FOculusXRBodyJoint& Joint = BodyState.Joints[BoneIndex];
		if (!Joint.bIsValid)
		{
			return false;
		}

		OutPosition = Joint.Position;
		return true;
	};

	FVector Head = FVector::ZeroVector;
	FVector LeftUpperArm = FVector::ZeroVector;
	FVector LeftLowerArm = FVector::ZeroVector;
	FVector LeftWrist = FVector::ZeroVector;
	FVector RightUpperArm = FVector::ZeroVector;
	FVector RightLowerArm = FVector::ZeroVector;
	FVector RightWrist = FVector::ZeroVector;

	const bool bHasHead = GetJointPosition(EOculusXRBoneID::BodyHead, Head);
	const bool bHasLeftUpperArm = GetJointPosition(EOculusXRBoneID::BodyLeftArmUpper, LeftUpperArm);
	const bool bHasLeftLowerArm = GetJointPosition(EOculusXRBoneID::BodyLeftArmLower, LeftLowerArm);
	const bool bHasLeftWrist = GetJointPosition(EOculusXRBoneID::BodyLeftHandWristTwist, LeftWrist);
	const bool bHasRightUpperArm = GetJointPosition(EOculusXRBoneID::BodyRightArmUpper, RightUpperArm);
	const bool bHasRightLowerArm = GetJointPosition(EOculusXRBoneID::BodyRightArmLower, RightLowerArm);
	const bool bHasRightWrist = GetJointPosition(EOculusXRBoneID::BodyRightHandWristTwist, RightWrist);

	const auto AddVectorColumns = [](TArray<FString>& Columns, bool bHasValue, const FVector& Value)
	{
		if (!bHasValue)
		{
			Columns.Add(TEXT(""));
			Columns.Add(TEXT(""));
			Columns.Add(TEXT(""));
			return;
		}

		Columns.Add(FString::Printf(TEXT("%.3f"), Value.X));
		Columns.Add(FString::Printf(TEXT("%.3f"), Value.Y));
		Columns.Add(FString::Printf(TEXT("%.3f"), Value.Z));
	};

	const auto AddLengthColumn = [](TArray<FString>& Columns, bool bHasStart, const FVector& Start, bool bHasEnd, const FVector& End)
	{
		if (!bHasStart || !bHasEnd)
		{
			Columns.Add(TEXT(""));
			return;
		}

		Columns.Add(FString::Printf(TEXT("%.3f"), FVector::Distance(Start, End)));
	};

	TArray<FString> Columns;
	Columns.Reserve(30);
	Columns.Add(FString::Printf(TEXT("%.3f"), World->GetTimeSeconds()));
	Columns.Add(LexToString(GFrameCounter));
	Columns.Add(TEXT("true"));
	AddVectorColumns(Columns, bHasHead, Head);
	AddVectorColumns(Columns, bHasLeftUpperArm, LeftUpperArm);
	AddVectorColumns(Columns, bHasLeftLowerArm, LeftLowerArm);
	AddVectorColumns(Columns, bHasLeftWrist, LeftWrist);
	AddVectorColumns(Columns, bHasRightUpperArm, RightUpperArm);
	AddVectorColumns(Columns, bHasRightLowerArm, RightLowerArm);
	AddVectorColumns(Columns, bHasRightWrist, RightWrist);
	AddLengthColumn(Columns, bHasLeftUpperArm, LeftUpperArm, bHasLeftLowerArm, LeftLowerArm);
	AddLengthColumn(Columns, bHasLeftLowerArm, LeftLowerArm, bHasLeftWrist, LeftWrist);
	AddLengthColumn(Columns, bHasLeftUpperArm, LeftUpperArm, bHasLeftWrist, LeftWrist);
	AddLengthColumn(Columns, bHasRightUpperArm, RightUpperArm, bHasRightLowerArm, RightLowerArm);
	AddLengthColumn(Columns, bHasRightLowerArm, RightLowerArm, bHasRightWrist, RightWrist);
	AddLengthColumn(Columns, bHasRightUpperArm, RightUpperArm, bHasRightWrist, RightWrist);

	const FString Row = FString::Join(Columns, TEXT(","));
	Rows.Add(Row);
	FFileHelper::SaveStringToFile(Row + LINE_TERMINATOR, *OutputPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append);
	return true;
}

UWorld* FCsvMotionRecorder::FindGameWorld() const
{
	if (!GEngine)
	{
		return nullptr;
	}

	for (const FWorldContext& WorldContext : GEngine->GetWorldContexts())
	{
		UWorld* World = WorldContext.World();
		if (World && (World->WorldType == EWorldType::Game || World->WorldType == EWorldType::PIE))
		{
			return World;
		}
	}

	return nullptr;
}

FString FCsvMotionRecorder::EscapeCsv(const FString& Value) const
{
	FString Escaped = Value;
	Escaped.ReplaceInline(TEXT("\""), TEXT("\"\""));

	if (Escaped.Contains(TEXT(",")) || Escaped.Contains(TEXT("\"")) || Escaped.Contains(TEXT("\n")) || Escaped.Contains(TEXT("\r")))
	{
		return FString::Printf(TEXT("\"%s\""), *Escaped);
	}

	return Escaped;
}
