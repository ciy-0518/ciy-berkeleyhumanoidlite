// test1.Build.cs
using UnrealBuildTool;

public class test1 : ModuleRules
{
	public test1(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
			"Sockets",
			"Networking",
			"OculusXRMovement",
			"OculusXRInput",
			"HeadMountedDisplay",
			"OculusXRPassthrough",
			"OculusXRHMD"
		});
	}
}
