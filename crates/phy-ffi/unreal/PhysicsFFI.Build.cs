// PhysicsFFI.Build.cs — 示例 Unreal 模块构建脚本,接入 phy_ffi 第三方库。
//
// 把本文件作为你自己 UE 模块(如 PhysicsFFI)的 Build.cs,并把 phy_ffi 的
// 头/库/dll 放到本模块的 ThirdParty 目录:
//   Source/PhysicsFFI/PhysicsFFI.Build.cs   (本文件)
//   ThirdParty/phy_ffi/Include/PhysicsFFI.h (crates/phy-ffi/unreal/PhysicsFFI.h)
//   ThirdParty/phy_ffi/Lib/Win64/phy_ffi.lib
//   (运行时) Binaries/Win64/phy_ffi.dll
//
// 复制到你的 UE 模块目录后按需改名(通常改为 <YourModule>.Build.cs)。

using System.IO;
using UnrealBuildTool;

public class PhysicsFFI : ModuleRules
{
    public PhysicsFFI(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine" });

        string ThirdParty = Path.Combine(ModuleDirectory, "..", "ThirdParty", "phy_ffi");

        // 头文件
        PublicIncludePaths.Add(Path.Combine(ThirdParty, "Include"));

        // 库(按平台挑选)
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            string Lib = Path.Combine(ThirdParty, "Lib", "Win64", "phy_ffi.lib");
            if (File.Exists(Lib))
            {
                PublicAdditionalLibraries.Add(Lib);
                PublicDelayLoadDLLs.Add("phy_ffi.dll");
            }
        }
        // 其他平台(linux/mac)在此分支补充对应的 .so/.dylib 与导入库。
    }
}
