#pragma once

#include "CoreMinimal.h"
#include "Containers/Ticker.h"

class FCsvMotionRecorder
{
public:
	FCsvMotionRecorder();

	void Start(UWorld* World = nullptr);
	void Stop();

private:
	bool Tick(float DeltaTime);
	UWorld* FindGameWorld() const;
	FString EscapeCsv(const FString& Value) const;

private:
	bool bIsRecording;
	float SampleIntervalSeconds;
	FTSTicker::FDelegateHandle TickHandle;
	TWeakObjectPtr<UWorld> RecordingWorld;
	FString OutputPath;
	TArray<FString> Rows;
};
