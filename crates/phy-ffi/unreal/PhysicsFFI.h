// PhysicsFFI.h — Unreal Engine (C++) 集成示例 for phy-ffi (ABI v2).
//
// 用法:
//   1. 把 phy_ffi.dll / phy_ffi.lib 放到模块的 ThirdParty 目录,在 Build.cs 里
//      PublicAdditionalLibraries.Add(phy_ffi.lib); PublicDelayLoadDLLs.Add("phy_ffi.dll");
//      运行时把 dll 拷到游戏可执行目录(或 Pak 的 Binaries)。
//   2. 包含本头文件,用 phy_ffi_abi_version() 核对契约(fail-fast)。
//   3. 主循环 Tick 里 phy_world_step(dt),用 phy_world_get_rigid_transforms 把
//      f64 位姿写入 FTransform / FVector / FQuat。
//
// 注意:
//   - 句柄是 void*,模块卸载/关卡切换时务必 phy_world_destroy 释放。
//   - 所有读回通道为 double(f64);若项目用 float,改用 _f32 系列函数省一半带宽。

#pragma once

#include "CoreMinimal.h"

// 若使用 dllexport 的静态导入,Unreal 推荐用第三方库头声明;此处用 extern "C" 直接声明
// phy_ffi 导出的 C 符号(与 phy_ffi.h 完全一致)。确保链接 phy_ffi.lib。
extern "C"
{
    // 契约
    uint32_t phy_ffi_abi_version();

    // 场景工厂
    void* phy_world_create_fluid();
    void* phy_world_create_rigid();
    void* phy_world_create_rigid_empty();
    void* phy_world_create_granular();
    void* phy_world_create_coupled();

    // 步进
    int32_t phy_world_step(void* w, double dt);
    int32_t phy_world_step_checked(void* w, double dt);
    int32_t phy_world_step_f32(void* w, float dt);

    // 查询
    double  phy_world_time(void* w);
    size_t  phy_world_sub_count(void* w);
    size_t  phy_world_fluid_count(void* w);
    size_t  phy_world_rigid_count(void* w);

    // 读回(f64 通道;buf 由调用方分配,长度 = count * stride)
    size_t phy_world_get_fluid_positions(void* w, double* buf, size_t len);
    size_t phy_world_get_fluid_velocities(void* w, double* buf, size_t len);
    size_t phy_world_get_rigid_transforms(void* w, double* buf, size_t len);
    size_t phy_world_get_fluid_positions_f32(void* w, float* buf, size_t len);
    size_t phy_world_get_rigid_transforms_f32(void* w, float* buf, size_t len);

    // 刚体增删 / 受力
    // shape_kind: 0=Sphere(半径取 inertia3[0]),1=Box(半长取 inertia3[0..2])
    // mass: kg;0 表示静态。pos7 = [px,py,pz, qw,qi,qj,qk];inertia3 = [Ixx,Iyy,Izz]
    int64_t phy_world_rigid_add_body(void* w, int32_t shape_kind, double mass,
                                     const double* pos7, const double* inertia3);
    int32_t phy_world_rigid_apply_force(void* w, int64_t id, const double* f3, double dt, int32_t mode);
    int32_t phy_world_rigid_apply_torque(void* w, int64_t id, const double* t3, double dt, int32_t mode);
    int32_t phy_world_rigid_get_velocity(void* w, int64_t id, double* out3);
    int32_t phy_world_rigid_get_angular_velocity(void* w, int64_t id, double* out3);

    // 序列化
    int32_t phy_world_save(void* w, const char* path);
    void*   phy_world_load(const char* path);

    // 释放
    void phy_world_destroy(void* w);
}

// 把 phy_ffi 的 7×double 位姿写进 Unreal 的 FTransform。
inline FTransform PhyToUnrealTransform(const double* buf, int32 BodyIndex)
{
    const double* o = buf + BodyIndex * 7;
    FVector Pos(o[0], o[1], o[2]);
    FQuat Rot(o[3], o[4], o[5], o[6]); // (w, x, y, z) —— phy 与 Unreal 同序
    return FTransform(Rot, Pos);
}
