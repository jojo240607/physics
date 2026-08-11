#ifndef PHY_FFI_H
#define PHY_FFI_H

#pragma once

/* 本文件由 cbindgen 自动生成,勿手改。运行: PHY_FFI_GEN_HEADER=1 cargo build -p phy-ffi */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

// ABI 版本(单调递增整数)。破坏任一 `phy_*` 符号签名 / 移除符号 / 改变
// `PhyWorldHandle` 布局时 **必须** +1,并在 CHANGELOG 记录。
#define PHY_FFI_ABI_VERSION 2

// 语义主版本(破坏性变更 +1,`0.x` 阶段允许 minor 间破坏性改动)。
#define PHY_FFI_VERSION_MAJOR 0

// 语义次版本(向后兼容新增)。
#define PHY_FFI_VERSION_MINOR 1

// 语义修订号(patch)。
#define PHY_FFI_VERSION_PATCH 0

// 不透明句柄。C/C++ 侧只见 `PhyWorldHandle *`,不可解引用;
// 真实 `World<f64>` 由 Rust 侧 `Box` 拥有,经指针转换在边界传递。
typedef struct {
    uint8_t _private[0];
} PhyWorldHandle;

// 返回当前库的 ABI 版本(`PHY_FFI_ABI_VERSION`)。
//
// C/C++ 业务应在加载后调用并断言 `>=` 自身编译期期望版本,版本不匹配时
// 拒绝加载(fail-fast),避免调用到签名已变更的 `phy_*` 符号。
uint32_t phy_ffi_abi_version(void);

// 创建流体(溃坝)世界。返回不透明句柄(失败返回 NULL)。
//
// # 示例(Rust 侧等价调用;实践中由 C/C++/Unity/C# 经 `phy_ffi.h` 调用)
//
// ```
// use phy_ffi::{phy_world_create_fluid, phy_world_step_checked,
//               phy_world_fluid_count, phy_world_destroy};
//
// let w = phy_world_create_fluid();
// assert!(!w.is_null());
// // 推进并用看门狗保证数值有限;返回 0 表示健康。
// for _ in 0..50 {
//     assert_eq!(phy_world_step_checked(w, 0.005), 0);
// }
// assert!(phy_world_fluid_count(w) > 0);
// phy_world_destroy(w); // 释放,空指针安全 no-op
// ```
PhyWorldHandle *phy_world_create_fluid(void);

// 创建刚体(下落球)世界。
PhyWorldHandle *phy_world_create_rigid(void);

// 创建颗粒(堆积)世界。
PhyWorldHandle *phy_world_create_granular(void);

// 创建流体+刚体耦合世界。
PhyWorldHandle *phy_world_create_coupled(void);

// 推进仿真 `dt` 秒。返回 0 成功,-1 失败(空指针或 panic)。
int32_t phy_world_step(PhyWorldHandle *w, double dt);

// 带看门狗的步进(数值健康度保护)。
//
// 返回值:
// - `0` : 步进成功且全部子系统数值有限(NaN/Inf 检查通过);
// - `-1`: 空指针或内部 panic;
// - `2` : 检测到非有限值(NaN/Inf),世界停在该帧(见 [`phy_core::WorldError::NonFinite`]);
// - `3` : 某子系统一帧内未推进时间(卡死/被跳过)。
//
// 业务(游戏/防战建模)在“数据必须有限才能喂给渲染或下游模型”时优先用本接口,
// 失败后可调用 `phy_world_destroy` 释放并用最后已知良好状态回滚。
int32_t phy_world_step_checked(PhyWorldHandle *w,
                               double dt);

// 当前仿真时间。空指针返回 NaN。
double phy_world_time(PhyWorldHandle *w);

// 子系统数量。
uintptr_t phy_world_sub_count(PhyWorldHandle *w);

// 统计所有流体子系统的粒子总数。
uintptr_t phy_world_fluid_count(PhyWorldHandle *w);

// 把所有流体粒子的位置(x,y,z 交错)写入 `buf`(长度 `len` 个 f64)。
// 返回实际写入的粒子数(不是 f64 个数)。若 buf 太小则只写前 `len/3` 个粒子。
uintptr_t phy_world_get_fluid_positions(PhyWorldHandle *w,
                                        double *buf,
                                        uintptr_t len);

// 把所有流体粒子的速度(x,y,z 交错)写入 `buf`。返回写入粒子数。
uintptr_t phy_world_get_fluid_velocities(PhyWorldHandle *w, double *buf, uintptr_t len);

// 所有刚体子系统的刚体数量。
uintptr_t phy_world_rigid_count(PhyWorldHandle *w);

// 把所有刚体的位姿(pos.x,y,z + quat.w,i,j,k,共 7 个 f64 交错)写入 `buf`。
// 返回写入的刚体数。
uintptr_t phy_world_get_rigid_transforms(PhyWorldHandle *w, double *buf, uintptr_t len);

// 序列化整个世界到 JSON 文件(路径为 UTF-8 C 字符串)。返回 0 成功,-1 失败。
int32_t phy_world_save(PhyWorldHandle *w, const char *path);

// 从 JSON 文件载入世界,返回新句柄(失败返回 NULL)。
PhyWorldHandle *phy_world_load(const char *path);

// 释放世界句柄。重复释放或空指针安全(no-op)。
void phy_world_destroy(PhyWorldHandle *w);

// # 双精度条目(P3)
//
// 内部世界始终以 `f64` 运行(精度/确定性见 S8 回放契约),但 FFI 边界额外提供
// **f32 通道**:步进的 `dt` 可为 f32(`phy_world_step_f32`),所有读回缓冲可为
// f32(`*_f32` 系列),写回时 cast `f64 → f32`。游戏/实时业务用 f32 通道可节省
// 一半内存带宽并直接对接 GPU/Unity 的 f32 顶点缓冲,无需自己转换。
//
// 注意:同一世界可混用 f32/f64 入口(精度在边界转换,内部状态恒 f64),ABI 稳定。
// 步进仿真 `dt` 秒(f32 入口,内部 cast 为 f64)。返回 0 成功,-1 失败。
int32_t phy_world_step_f32(PhyWorldHandle *w,
                           float dt);

// 把所有流体粒子的位置(x,y,z 交错)写入 `buf`(长度 `len` 个 f32)。
// 返回实际写入的粒子数。f32 通道,省一半带宽,直接对接 GPU/Unity f32 缓冲。
uintptr_t phy_world_get_fluid_positions_f32(PhyWorldHandle *w, float *buf, uintptr_t len);

// 把所有流体粒子的速度(x,y,z 交错)写入 f32 `buf`。返回写入粒子数。
uintptr_t phy_world_get_fluid_velocities_f32(PhyWorldHandle *w, float *buf, uintptr_t len);

// 把所有刚体的位姿(pos.x,y,z + quat.w,i,j,k,共 7 个 f32 交错)写入 `buf`。
// 返回写入的刚体数。
uintptr_t phy_world_get_rigid_transforms_f32(PhyWorldHandle *w, float *buf, uintptr_t len);

// 向刚体子系统追加一个刚体,返回其索引(>=0);失败(空指针/无刚体子系统/panic)返回 -1。
//
// - `shape_kind`: 0=Sphere(半径取 `inertia3[0]`), 1=Box(半长取 `inertia3[0..2]`)。
// - `mass`: 质量(kg);0 表示静态/无限质量(inv_mass=0)。
// - `pos7`: 7×f64 = pos.xyz + quat.wijk(机体->世界)。
// - `inertia3`: 体坐标系三个主转动惯量(Ixx,Iyy,Izz),写入对角 `inv_inertia_local`。
int64_t phy_world_rigid_add_body(PhyWorldHandle *w,
                                 int32_t shape_kind,
                                 double mass,
                                 const double *pos7,
                                 const double *inertia3);

// 施加世界系力(牛顿)到指定刚体,直接积分进线速度:`vel += f * inv_mass * dt`。
// 返回 0 成功,-1 失败(空指针/id 越界/无刚体子系统/panic)。`mode` 当前按累加(0)处理。
int32_t phy_world_rigid_apply_force(PhyWorldHandle *w,
                                    int64_t id,
                                    const double *f3,
                                    double dt,
                                    int32_t _mode);

// 施加世界系力矩(N·m)到指定刚体,直接积分进角速度:`ang_vel += I_world⁻¹ * t * dt`,
// 其中 `I_world⁻¹ = rot * inv_inertia_local * rotᵀ`。返回 0 成功,-1 失败。
int32_t phy_world_rigid_apply_torque(PhyWorldHandle *w,
                                     int64_t id,
                                     const double *t3,
                                     double dt,
                                     int32_t _mode);

// 读回指定刚体的世界系线速度(3×f64)到 `out3`。返回 0 成功,-1 失败。
int32_t phy_world_rigid_get_velocity(PhyWorldHandle *w, int64_t id, double *out3);

// 读回指定刚体的世界系角速度(rad/s,3×f64)到 `out3`。返回 0 成功,-1 失败。
int32_t phy_world_rigid_get_angular_velocity(PhyWorldHandle *w, int64_t id, double *out3);

#endif  /* PHY_FFI_H */
