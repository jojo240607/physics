#ifndef PHY_FFI_H
#define PHY_FFI_H

#pragma once

/* 本文件由 cbindgen 自动生成,勿手改。运行: PHY_FFI_GEN_HEADER=1 cargo build -p phy-ffi */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

// 不透明句柄。C/C++ 侧只见 `PhyWorldHandle *`,不可解引用;
// 真实 `World<f64>` 由 Rust 侧 `Box` 拥有,经指针转换在边界传递。
typedef struct {
    uint8_t _private[0];
} PhyWorldHandle;

// 创建流体(溃坝)世界。返回不透明句柄(失败返回 NULL)。
PhyWorldHandle *phy_world_create_fluid(void);

// 创建刚体(下落球)世界。
PhyWorldHandle *phy_world_create_rigid(void);

// 创建颗粒(堆积)世界。
PhyWorldHandle *phy_world_create_granular(void);

// 创建流体+刚体耦合世界。
PhyWorldHandle *phy_world_create_coupled(void);

// 推进仿真 `dt` 秒。返回 0 成功,-1 失败(空指针或 panic)。
int32_t phy_world_step(PhyWorldHandle *w, double dt);

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

#endif  /* PHY_FFI_H */
