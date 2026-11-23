// Fill out your copyright notice in the Description page of Project Settings.

#include "test1.h"
#include "Engine/World.h"
#include "Modules/ModuleManager.h"
#include "CsvMotionRecorder.h"

class FTest1Module : public FDefaultGameModuleImpl
{
public:
	virtual void StartupModule() override
	{
		FDefaultGameModuleImpl::StartupModule();

		PostWorldInitializationHandle = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &FTest1Module::HandlePostWorldInitialization);
		WorldCleanupHandle = FWorldDelegates::OnWorldCleanup.AddRaw(this, &FTest1Module::HandleWorldCleanup);
	}

	virtual void ShutdownModule() override
	{
		FWorldDelegates::OnPostWorldInitialization.Remove(PostWorldInitializationHandle);
		FWorldDelegates::OnWorldCleanup.Remove(WorldCleanupHandle);

		CsvMotionRecorder.Stop();

		FDefaultGameModuleImpl::ShutdownModule();
	}

private:
	void HandlePostWorldInitialization(UWorld* World, const UWorld::InitializationValues IVS)
	{
		if (World && (World->WorldType == EWorldType::Game || World->WorldType == EWorldType::PIE))
		{
			CsvMotionRecorder.Start(World);
		}
	}

	void HandleWorldCleanup(UWorld* World, bool bSessionEnded, bool bCleanupResources)
	{
		if (World && (World->WorldType == EWorldType::Game || World->WorldType == EWorldType::PIE))
		{
			CsvMotionRecorder.Stop();
		}
	}

	FCsvMotionRecorder CsvMotionRecorder;
	FDelegateHandle PostWorldInitializationHandle;
	FDelegateHandle WorldCleanupHandle;
};

IMPLEMENT_PRIMARY_GAME_MODULE(FTest1Module, test1, "test1");

