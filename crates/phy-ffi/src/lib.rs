//! 物理引擎 C ABI 层(L3 库化 hardening)。
//!
//! 把统一 `World<f64>` 调度封装为稳定的 `extern "C"` 接口,供 C/C++/Unity/Unreal 业务
//! 通过 `cdylib`(`.dll`/`.so`/`.dylib`)链接调用。所有入口都经 `catch_unwind` 包裹,
//! 保证 Rust panic 不会跨 FFI 边界 unwind(那是 UB)。
//!
//! 句柄约定:业务侧只见到 `PhyWorldHandle *`(不透明,不可解引用),Rust 侧用 `Box` 拥有
//! 真实的 `World<f64>` 并在边界做指针转换。空指针返回 `NULL`(或 -1)。
//!
//! 编译产物:`cargo build -p phy-ffi --release` → `target/release/phy_ffi.{dll,so,dylib}`。
//! C 头:`PHY_FFI_GEN_HEADER=1 cargo build -p phy-ffi` → `crates/phy-ffi/phy_ffi.h`。

// Windows-GNU 目标下,rustc 为 cdylib 自动生成的 `.def` 会被 MinGW `ld` 报
// "corrupt .drectve at end of def file"(rust-lang/rust#50633,工具链已知缺陷,
// 与引擎代码无关,且不影响 MSVC 打包路径 —— 见 build_package.py 的 llvm-dlltool)。
// 该 linker lint 在此 crate 静音,保持构建信号干净。
#![allow(linker_messages)]

// ===== ABI 稳定性契约(P7) =====================================================
//
// 本 crate 的所有 `#[no_mangle] pub extern "C" fn phy_*` 是**稳定 ABI 契约**:
// - 禁止改动其签名、移除符号或改变 `PhyWorldHandle` 布局,除非同步把
//   `PHY_FFI_ABI_VERSION` 自增,并在 CHANGELOG 的 "Changed" 节记录。
// - C/C++/Unity/Unreal 业务可在运行时核对 `PHY_FFI_ABI_VERSION` 与自身编译期
//   期望版本,不匹配时拒绝加载(fail-fast)。
// - 内部辅助(`as_world`/`box_world`/`cstr_path`/`nonnull_slice*`)为 crate 私有,
//   不属于 ABI 契约,可自由重构。
//
// 语义版本(`PHY_FFI_VERSION_*`)与 `[workspace.package].version` 对齐;
// `0.x` 阶段 Rust crate API 仍可在 minor 间变动,Rust 消费方需锁定精确版本。

/// ABI 版本(单调递增整数)。破坏任一 `phy_*` 符号签名 / 移除符号 / 改变
/// `PhyWorldHandle` 布局时 **必须** +1,并在 CHANGELOG 记录。
pub const PHY_FFI_ABI_VERSION: u32 = 2;

/// 语义主版本(破坏性变更 +1,`0.x` 阶段允许 minor 间破坏性改动)。
pub const PHY_FFI_VERSION_MAJOR: u32 = 0;
/// 语义次版本(向后兼容新增)。
pub const PHY_FFI_VERSION_MINOR: u32 = 1;
/// 语义修订号(patch)。
pub const PHY_FFI_VERSION_PATCH: u32 = 0;

use std::ffi::CStr;
use std::panic::{self, AssertUnwindSafe};

use phy_core::World;
use phy_fluid::{FluidSubsystem, FluidWorld, SphParams};
use phy_granular::subsystem::GranularSubsystem;
use phy_granular::world::GranularWorld;
use phy_math::Vec3;
use phy_rigid::shape::Shape;
use phy_rigid::subsystem::RigidSubsystem;
use phy_rigid::world::RigidWorld;
use phy_rigid::Body;

/// 不透明句柄。C/C++ 侧只见 `PhyWorldHandle *`,不可解引用;
/// 真实 `World<f64>` 由 Rust 侧 `Box` 拥有,经指针转换在边界传递。
#[repr(C)]
pub struct PhyWorldHandle {
    _private: [u8; 0],
}

// ---- 句柄辅助 ----------------------------------------------------------------

/// 在所有 FFI 入口包一层 catch_unwind,避免 panic 跨 FFI(UB)。
/// 用 `AssertUnwindSafe` 包裹,业务闭包无需满足 `UnwindSafe`(原始指针/可变借用默认不满足)。
fn guard<R>(f: impl FnOnce() -> R) -> Option<R> {
    panic::catch_unwind(AssertUnwindSafe(f)).ok()
}

/// 把业务侧不透明句柄转回内部 `&mut World`(空指针返回 None)。
fn as_world(w: *mut PhyWorldHandle) -> Option<&'static mut World<f64>> {
    if w.is_null() {
        None
    } else {
        // 安全:句柄由 `box_world` 产生,指向有效 `Box<World>`。
        Some(unsafe { &mut *(w as *mut World<f64>) })
    }
}

/// 把内部 `World` 装箱为不透明句柄(供返回)。
fn box_world(w: World<f64>) -> *mut PhyWorldHandle {
    Box::into_raw(Box::new(w)) as *mut PhyWorldHandle
}

fn cstr_path(ptr: *const std::os::raw::c_char) -> Option<String> {
    if ptr.is_null() {
        return None;
    }
    let c = unsafe { CStr::from_ptr(ptr) };
    c.to_str().ok().map(|s| s.to_string())
}

// ---- ABI 版本查询(供 C 侧运行时核对契约) ------------------------------------

/// 返回当前库的 ABI 版本(`PHY_FFI_ABI_VERSION`)。
///
/// C/C++ 业务应在加载后调用并断言 `>=` 自身编译期期望版本,版本不匹配时
/// 拒绝加载(fail-fast),避免调用到签名已变更的 `phy_*` 符号。
#[no_mangle]
pub extern "C" fn phy_ffi_abi_version() -> u32 {
    PHY_FFI_ABI_VERSION
}

// ---- 场景工厂(构建典型多物理 World,不依赖 GUI 的 phy-demo) ------------------

fn build_fluid_world() -> World<f64> {
    let params = SphParams::<f64>::defaults();
    let mut fworld = FluidWorld::<f64>::new(params);
    // 溃坝:左半盒填充粒子。
    fworld.fill_box(
        Vec3::new(-4.0, -4.0, -1.0),
        Vec3::new(0.0, 4.0, 1.0),
        0.18,
        0.05,
    );
    let mut world = World::<f64>::new();
    world.add_subsystem(Box::new(FluidSubsystem::new(fworld)));
    world
}

fn build_rigid_world() -> World<f64> {
    let mut rworld = RigidWorld::<f64>::new();
    // 静态地面。
    rworld.add_body(Body::new(
        Shape::Box {
            half: Vec3::new(10.0, 0.5, 10.0),
        },
        Vec3::new(0.0, -5.0, 0.0),
        0.0,
    ));
    // 5 个下落球。
    for i in 0..5usize {
        rworld.add_body(Body::new(
            Shape::Sphere { r: 0.5 },
            Vec3::new((i as f64 - 2.0) * 1.2, 3.0, 0.0),
            1.0,
        ));
    }
    let mut world = World::<f64>::new();
    world.add_subsystem(Box::new(RigidSubsystem::new(rworld)));
    world
}

fn build_granular_world() -> World<f64> {
    let mut gworld = GranularWorld::<f64>::new();
    gworld.set_bounds(Vec3::new(-5.0, -5.0, -5.0), Vec3::new(5.0, 5.0, 5.0));
    gworld.fill_grid(500usize, 0.2, 1.0, 1.1);
    let mut world = World::<f64>::new();
    world.add_subsystem(Box::new(GranularSubsystem::new(gworld)));
    world
}

fn build_coupled_world() -> World<f64> {
    // 流体 + 刚体耦合(浮力/碰撞)。
    let params = SphParams::<f64>::defaults();
    let mut fworld = FluidWorld::<f64>::new(params);
    fworld.fill_box(
        Vec3::new(-3.0, -4.0, -1.0),
        Vec3::new(3.0, 0.0, 1.0),
        0.18,
        0.05,
    );
    let mut rworld = RigidWorld::<f64>::new();
    rworld.add_body(Body::new(
        Shape::Box {
            half: Vec3::new(10.0, 0.5, 10.0),
        },
        Vec3::new(0.0, -5.0, 0.0),
        0.0,
    ));
    rworld.add_body(Body::new(
        Shape::Sphere { r: 0.6 },
        Vec3::new(0.0, 2.0, 0.0),
        1.0,
    ));

    let mut world = World::<f64>::new();
    world.add_subsystem(Box::new(RigidSubsystem::new(rworld)));
    world.add_subsystem(Box::new(FluidSubsystem::new(fworld)));
    world
}

// ---- 导出接口 ----------------------------------------------------------------

/// 创建流体(溃坝)世界。返回不透明句柄(失败返回 NULL)。
///
/// # 示例(Rust 侧等价调用;实践中由 C/C++/Unity/C# 经 `phy_ffi.h` 调用)
///
/// ```
/// use phy_ffi::{phy_world_create_fluid, phy_world_step_checked,
///               phy_world_fluid_count, phy_world_destroy};
///
/// let w = phy_world_create_fluid();
/// assert!(!w.is_null());
/// // 推进并用看门狗保证数值有限;返回 0 表示健康。
/// for _ in 0..50 {
///     assert_eq!(phy_world_step_checked(w, 0.005), 0);
/// }
/// assert!(phy_world_fluid_count(w) > 0);
/// phy_world_destroy(w); // 释放,空指针安全 no-op
/// ```
#[no_mangle]
pub extern "C" fn phy_world_create_fluid() -> *mut PhyWorldHandle {
    guard(build_fluid_world)
        .map(box_world)
        .unwrap_or(std::ptr::null_mut())
}

/// 创建刚体(下落球)世界。
#[no_mangle]
pub extern "C" fn phy_world_create_rigid() -> *mut PhyWorldHandle {
    guard(build_rigid_world)
        .map(box_world)
        .unwrap_or(std::ptr::null_mut())
}

/// 创建**空**刚体世界（无地面、无演示球）。适用于需要自定义被控对象/地面的场景
/// （如四旋翼仿真：调用方自行 `phy_world_rigid_add_body` 添加机体与可选地面）。
#[no_mangle]
pub extern "C" fn phy_world_create_rigid_empty() -> *mut PhyWorldHandle {
    let mut rworld = RigidWorld::<f64>::new();
    let mut world = World::<f64>::new();
    world.add_subsystem(Box::new(RigidSubsystem::new(rworld)));
    box_world(world)
}

/// 创建颗粒(堆积)世界。
#[no_mangle]
pub extern "C" fn phy_world_create_granular() -> *mut PhyWorldHandle {
    guard(build_granular_world)
        .map(box_world)
        .unwrap_or(std::ptr::null_mut())
}

/// 创建流体+刚体耦合世界。
#[no_mangle]
pub extern "C" fn phy_world_create_coupled() -> *mut PhyWorldHandle {
    guard(build_coupled_world)
        .map(box_world)
        .unwrap_or(std::ptr::null_mut())
}

/// 推进仿真 `dt` 秒。返回 0 成功,-1 失败(空指针或 panic)。
#[no_mangle]
pub extern "C" fn phy_world_step(w: *mut PhyWorldHandle, dt: f64) -> i32 {
    let world = match as_world(w) {
        Some(w) => w,
        None => return -1,
    };
    guard(|| world.step(dt)).map(|_| 0).unwrap_or(-1)
}

/// 带看门狗的步进(数值健康度保护)。
///
/// 返回值:
/// - `0` : 步进成功且全部子系统数值有限(NaN/Inf 检查通过);
/// - `-1`: 空指针或内部 panic;
/// - `2` : 检测到非有限值(NaN/Inf),世界停在该帧(见 [`phy_core::WorldError::NonFinite`]);
/// - `3` : 某子系统一帧内未推进时间(卡死/被跳过)。
///
/// 业务(游戏/防战建模)在“数据必须有限才能喂给渲染或下游模型”时优先用本接口,
/// 失败后可调用 `phy_world_destroy` 释放并用最后已知良好状态回滚。
#[no_mangle]
pub extern "C" fn phy_world_step_checked(w: *mut PhyWorldHandle, dt: f64) -> i32 {
    let world = match as_world(w) {
        Some(w) => w,
        None => return -1,
    };
    match guard(|| world.step_checked(dt)) {
        Some(Ok(())) => 0,
        Some(Err(phy_core::WorldError::NonFinite { .. })) => 2,
        Some(Err(phy_core::WorldError::Stalled { .. })) => 3,
        None => -1,
    }
}

/// 当前仿真时间。空指针返回 NaN。
#[no_mangle]
pub extern "C" fn phy_world_time(w: *mut PhyWorldHandle) -> f64 {
    match as_world(w) {
        Some(world) => world.time(),
        None => f64::NAN,
    }
}

/// 子系统数量。
#[no_mangle]
pub extern "C" fn phy_world_sub_count(w: *mut PhyWorldHandle) -> usize {
    as_world(w).map(|world| world.subsystem_count()).unwrap_or(0)
}

/// 统计所有流体子系统的粒子总数。
#[no_mangle]
pub extern "C" fn phy_world_fluid_count(w: *mut PhyWorldHandle) -> usize {
    as_world(w)
        .map(|world| {
            let n = world.subsystem_count();
            let mut total = 0usize;
            for i in 0..n {
                if let Some(sub) = world.get(i) {
                    if let Some(fluid) = sub.as_any().downcast_ref::<FluidSubsystem<f64>>() {
                        total += fluid.world.particles.len();
                    }
                }
            }
            total
        })
        .unwrap_or(0)
}

/// 把所有流体粒子的位置(x,y,z 交错)写入 `buf`(长度 `len` 个 f64)。
/// 返回实际写入的粒子数(不是 f64 个数)。若 buf 太小则只写前 `len/3` 个粒子。
#[no_mangle]
pub extern "C" fn phy_world_get_fluid_positions(
    w: *mut PhyWorldHandle,
    buf: *mut f64,
    len: usize,
) -> usize {
    let (world, out) = match (as_world(w), nonnull_slice(buf, len)) {
        (Some(world), Some(out)) => (world, out),
        _ => return 0,
    };
    let mut written = 0usize;
    let n = world.subsystem_count();
    for i in 0..n {
        if let Some(sub) = world.get(i) {
            if let Some(fluid) = sub.as_any().downcast_ref::<FluidSubsystem<f64>>() {
                for p in &fluid.world.particles {
                    if written + 3 > len {
                        return written / 3;
                    }
                    out[written] = p.pos.x;
                    out[written + 1] = p.pos.y;
                    out[written + 2] = p.pos.z;
                    written += 3;
                }
            }
        }
    }
    written / 3
}

/// 把所有流体粒子的速度(x,y,z 交错)写入 `buf`。返回写入粒子数。
#[no_mangle]
pub extern "C" fn phy_world_get_fluid_velocities(
    w: *mut PhyWorldHandle,
    buf: *mut f64,
    len: usize,
) -> usize {
    let (world, out) = match (as_world(w), nonnull_slice(buf, len)) {
        (Some(world), Some(out)) => (world, out),
        _ => return 0,
    };
    let mut written = 0usize;
    let n = world.subsystem_count();
    for i in 0..n {
        if let Some(sub) = world.get(i) {
            if let Some(fluid) = sub.as_any().downcast_ref::<FluidSubsystem<f64>>() {
                for p in &fluid.world.particles {
                    if written + 3 > len {
                        return written / 3;
                    }
                    out[written] = p.vel.x;
                    out[written + 1] = p.vel.y;
                    out[written + 2] = p.vel.z;
                    written += 3;
                }
            }
        }
    }
    written / 3
}

/// 所有刚体子系统的刚体数量。
#[no_mangle]
pub extern "C" fn phy_world_rigid_count(w: *mut PhyWorldHandle) -> usize {
    as_world(w)
        .map(|world| {
            let n = world.subsystem_count();
            let mut total = 0usize;
            for i in 0..n {
                if let Some(sub) = world.get(i) {
                    if let Some(rigid) = sub.as_any().downcast_ref::<RigidSubsystem<f64>>() {
                        total += rigid.world.bodies.len();
                    }
                }
            }
            total
        })
        .unwrap_or(0)
}

/// 把所有刚体的位姿(pos.x,y,z + quat.w,i,j,k,共 7 个 f64 交错)写入 `buf`。
/// 返回写入的刚体数。
#[no_mangle]
pub extern "C" fn phy_world_get_rigid_transforms(
    w: *mut PhyWorldHandle,
    buf: *mut f64,
    len: usize,
) -> usize {
    let (world, out) = match (as_world(w), nonnull_slice(buf, len)) {
        (Some(world), Some(out)) => (world, out),
        _ => return 0,
    };
    let mut written = 0usize;
    let n = world.subsystem_count();
    for i in 0..n {
        if let Some(sub) = world.get(i) {
            if let Some(rigid) = sub.as_any().downcast_ref::<RigidSubsystem<f64>>() {
                for b in &rigid.world.bodies {
                    if written + 7 > len {
                        return written / 7;
                    }
                    out[written] = b.pos.x;
                    out[written + 1] = b.pos.y;
                    out[written + 2] = b.pos.z;
                    out[written + 3] = b.rot.w;
                    out[written + 4] = b.rot.i;
                    out[written + 5] = b.rot.j;
                    out[written + 6] = b.rot.k;
                    written += 7;
                }
            }
        }
    }
    written / 7
}

/// 序列化整个世界到 JSON 文件(路径为 UTF-8 C 字符串)。返回 0 成功,-1 失败。
#[no_mangle]
pub extern "C" fn phy_world_save(w: *mut PhyWorldHandle, path: *const std::os::raw::c_char) -> i32 {
    let p = match cstr_path(path) {
        Some(p) => p,
        None => return -1,
    };
    match as_world(w) {
        Some(world) => phy_io::save_world(world, std::path::Path::new(&p))
            .map(|_| 0)
            .unwrap_or(-1),
        None => -1,
    }
}

/// 从 JSON 文件载入世界,返回新句柄(失败返回 NULL)。
#[no_mangle]
pub extern "C" fn phy_world_load(path: *const std::os::raw::c_char) -> *mut PhyWorldHandle {
    let p = match cstr_path(path) {
        Some(p) => p,
        None => return std::ptr::null_mut(),
    };
    guard(|| phy_io::load_world::<f64>(std::path::Path::new(&p)))
        .map(box_world)
        .unwrap_or(std::ptr::null_mut())
}

/// 释放世界句柄。重复释放或空指针安全(no-op)。
#[no_mangle]
pub extern "C" fn phy_world_destroy(w: *mut PhyWorldHandle) {
    if w.is_null() {
        return;
    }
    guard(|| unsafe {
        drop(Box::from_raw(w as *mut World<f64>));
    });
}

/// 把非空裸指针 + 长度转为可变切片(空指针返回 None,避免越界)。
fn nonnull_slice<'a>(ptr: *mut f64, len: usize) -> Option<&'a mut [f64]> {
    if ptr.is_null() {
        None
    } else {
        Some(unsafe { std::slice::from_raw_parts_mut(ptr, len) })
    }
}

/// f32 变体:`*mut f32` 缓冲切片(对齐游戏/Unity/GPU 的 f32 世界,省 50% 带宽)。
fn nonnull_slice_f32<'a>(ptr: *mut f32, len: usize) -> Option<&'a mut [f32]> {
    if ptr.is_null() {
        None
    } else {
        Some(unsafe { std::slice::from_raw_parts_mut(ptr, len) })
    }
}

/// # 双精度条目(P3)
///
/// 内部世界始终以 `f64` 运行(精度/确定性见 S8 回放契约),但 FFI 边界额外提供
/// **f32 通道**:步进的 `dt` 可为 f32(`phy_world_step_f32`),所有读回缓冲可为
/// f32(`*_f32` 系列),写回时 cast `f64 → f32`。游戏/实时业务用 f32 通道可节省
/// 一半内存带宽并直接对接 GPU/Unity 的 f32 顶点缓冲,无需自己转换。
///
/// 注意:同一世界可混用 f32/f64 入口(精度在边界转换,内部状态恒 f64),ABI 稳定。

/// 步进仿真 `dt` 秒(f32 入口,内部 cast 为 f64)。返回 0 成功,-1 失败。
#[no_mangle]
pub extern "C" fn phy_world_step_f32(w: *mut PhyWorldHandle, dt: f32) -> i32 {
    let world = match as_world(w) {
        Some(w) => w,
        None => return -1,
    };
    guard(|| world.step(dt as f64)).map(|_| 0).unwrap_or(-1)
}

/// 把所有流体粒子的位置(x,y,z 交错)写入 `buf`(长度 `len` 个 f32)。
/// 返回实际写入的粒子数。f32 通道,省一半带宽,直接对接 GPU/Unity f32 缓冲。
#[no_mangle]
pub extern "C" fn phy_world_get_fluid_positions_f32(
    w: *mut PhyWorldHandle,
    buf: *mut f32,
    len: usize,
) -> usize {
    let (world, out) = match (as_world(w), nonnull_slice_f32(buf, len)) {
        (Some(world), Some(out)) => (world, out),
        _ => return 0,
    };
    let mut written = 0usize;
    let n = world.subsystem_count();
    for i in 0..n {
        if let Some(sub) = world.get(i) {
            if let Some(fluid) = sub.as_any().downcast_ref::<FluidSubsystem<f64>>() {
                for p in &fluid.world.particles {
                    if written + 3 > len {
                        return written / 3;
                    }
                    out[written] = p.pos.x as f32;
                    out[written + 1] = p.pos.y as f32;
                    out[written + 2] = p.pos.z as f32;
                    written += 3;
                }
            }
        }
    }
    written / 3
}

/// 把所有流体粒子的速度(x,y,z 交错)写入 f32 `buf`。返回写入粒子数。
#[no_mangle]
pub extern "C" fn phy_world_get_fluid_velocities_f32(
    w: *mut PhyWorldHandle,
    buf: *mut f32,
    len: usize,
) -> usize {
    let (world, out) = match (as_world(w), nonnull_slice_f32(buf, len)) {
        (Some(world), Some(out)) => (world, out),
        _ => return 0,
    };
    let mut written = 0usize;
    let n = world.subsystem_count();
    for i in 0..n {
        if let Some(sub) = world.get(i) {
            if let Some(fluid) = sub.as_any().downcast_ref::<FluidSubsystem<f64>>() {
                for p in &fluid.world.particles {
                    if written + 3 > len {
                        return written / 3;
                    }
                    out[written] = p.vel.x as f32;
                    out[written + 1] = p.vel.y as f32;
                    out[written + 2] = p.vel.z as f32;
                    written += 3;
                }
            }
        }
    }
    written / 3
}

/// 把所有刚体的位姿(pos.x,y,z + quat.w,i,j,k,共 7 个 f32 交错)写入 `buf`。
/// 返回写入的刚体数。
#[no_mangle]
pub extern "C" fn phy_world_get_rigid_transforms_f32(
    w: *mut PhyWorldHandle,
    buf: *mut f32,
    len: usize,
) -> usize {
    let (world, out) = match (as_world(w), nonnull_slice_f32(buf, len)) {
        (Some(world), Some(out)) => (world, out),
        _ => return 0,
    };
    let mut written = 0usize;
    let n = world.subsystem_count();
    for i in 0..n {
        if let Some(sub) = world.get(i) {
            if let Some(rigid) = sub.as_any().downcast_ref::<RigidSubsystem<f64>>() {
                for b in &rigid.world.bodies {
                    if written + 7 > len {
                        return written / 7;
                    }
                    out[written] = b.pos.x as f32;
                    out[written + 1] = b.pos.y as f32;
                    out[written + 2] = b.pos.z as f32;
                    out[written + 3] = b.rot.w as f32;
                    out[written + 4] = b.rot.i as f32;
                    out[written + 5] = b.rot.j as f32;
                    out[written + 6] = b.rot.k as f32;
                    written += 7;
                }
            }
        }
    }
    written / 7
}

// ---- 刚体单实例操控(四旋翼等"被控对象"接口, P-quad) --------------------------
//
// 现有 FFI 只暴露整世界级读回(`get_rigid_transforms`)。四旋翼仿真需要按 id 施加
// 旋翼推力(力/力矩)并读回线/角速度以生成 IMU 真值,故补以下符号。
//
// 积分语义:引擎 `RigidWorld::step` 仅对 `vel` 加重力、**不动 `ang_vel`**,也不消费
// 任何"外力"字段。因此 `apply_force`/`apply_torque` 在 `step` 前由调用方每帧调用一次,
// 直接把世界系力/力矩按半隐式欧拉积分进 `vel`/`ang_vel`(`× inv_mass × dt` /
// `× I_world⁻¹ × dt`)。`mode`:0=累加(多次调用叠加),1=覆盖(先清该 id 上一帧积分量)。
// 覆盖模式需在 handle 外维护"上次施加量",本层不存状态 —— 调用方每帧只调一次累加即可。

/// 形状类型枚举(供 `phy_world_rigid_add_body` 的 `shape_kind`)。
/// 0=Sphere, 1=Box。
const SHAPE_BOX: i32 = 1;

/// 向刚体子系统追加一个刚体,返回其索引(>=0);失败(空指针/无刚体子系统/panic)返回 -1。
///
/// - `shape_kind`: 0=Sphere(半径取 `inertia3[0]`), 1=Box(半长取 `inertia3[0..2]`)。
/// - `mass`: 质量(kg);0 表示静态/无限质量(inv_mass=0)。
/// - `pos7`: 7×f64 = pos.xyz + quat.wijk(机体->世界)。
/// - `inertia3`: 体坐标系三个主转动惯量(Ixx,Iyy,Izz),写入对角 `inv_inertia_local`。
#[no_mangle]
pub extern "C" fn phy_world_rigid_add_body(
    w: *mut PhyWorldHandle,
    shape_kind: i32,
    mass: f64,
    pos7: *const f64,
    inertia3: *const f64,
) -> i64 {
    let (world, p7, i3) = match (as_world(w), nonnull_slice(pos7 as *mut f64, 7), nonnull_slice(inertia3 as *mut f64, 3)) {
        (Some(world), Some(p7), Some(i3)) => (world, p7, i3),
        _ => return -1,
    };
    guard(|| {
        for i in 0..world.subsystem_count() {
            if let Some(sub) = world.get_mut(i) {
                if let Some(rigid) = sub.as_any_mut().downcast_mut::<RigidSubsystem<f64>>() {
                    let shape = match shape_kind {
                        SHAPE_BOX => phy_rigid::shape::Shape::Box {
                            half: phy_math::Vec3::new(i3[0], i3[1], i3[2]),
                        },
                        _ => phy_rigid::shape::Shape::Sphere { r: i3[0] },
                    };
                    let rot = phy_math::na::UnitQuaternion::from_quaternion(
                        phy_math::na::Quaternion::new(p7[3], p7[4], p7[5], p7[6]),
                    );
                    // `mass` 语义为质量(kg);0 表示静态。`Body::new` 接收的是反质量(inv_mass)。
                    let inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
                    let mut body = phy_rigid::Body::new(
                        shape,
                        phy_math::Vec3::new(p7[0], p7[1], p7[2]),
                        inv_mass,
                    );
                    body.rot = rot;
                    // 用调用方提供的体坐标主转动惯量覆盖几何估计值(Ixx,Iyy,Izz)。
                    body.inv_inertia_local = phy_math::Mat3::from_diagonal(
                        &phy_math::na::Vector3::new(1.0 / i3[0], 1.0 / i3[1], 1.0 / i3[2]),
                    );
                    return rigid.world.add_body(body) as i64;
                }
            }
        }
        -1
    })
    .unwrap_or(-1)
}

/// 施加世界系力(牛顿)到指定刚体,直接积分进线速度:`vel += f * inv_mass * dt`。
/// 返回 0 成功,-1 失败(空指针/id 越界/无刚体子系统/panic)。`mode` 当前按累加(0)处理。
#[no_mangle]
pub extern "C" fn phy_world_rigid_apply_force(
    w: *mut PhyWorldHandle,
    id: i64,
    f3: *const f64,
    dt: f64,
    _mode: i32,
) -> i32 {
    let (world, f) = match (as_world(w), nonnull_slice(f3 as *mut f64, 3)) {
        (Some(world), Some(f)) => (world, f),
        _ => return -1,
    };
    guard(|| {
        let id = id as usize;
        for i in 0..world.subsystem_count() {
            if let Some(sub) = world.get_mut(i) {
                if let Some(rigid) = sub.as_any_mut().downcast_mut::<RigidSubsystem<f64>>() {
                    if id < rigid.world.bodies.len() {
                        let b = &mut rigid.world.bodies[id];
                        if b.inv_mass > 0.0 {
                            b.vel.x += f[0] * b.inv_mass * dt;
                            b.vel.y += f[1] * b.inv_mass * dt;
                            b.vel.z += f[2] * b.inv_mass * dt;
                        }
                        return 0;
                    }
                }
            }
        }
        -1
    })
    .unwrap_or(-1)
}

/// 施加世界系力矩(N·m)到指定刚体,直接积分进角速度:`ang_vel += I_world⁻¹ * t * dt`,
/// 其中 `I_world⁻¹ = rot * inv_inertia_local * rotᵀ`。返回 0 成功,-1 失败。
#[no_mangle]
pub extern "C" fn phy_world_rigid_apply_torque(
    w: *mut PhyWorldHandle,
    id: i64,
    t3: *const f64,
    dt: f64,
    _mode: i32,
) -> i32 {
    let (world, t) = match (as_world(w), nonnull_slice(t3 as *mut f64, 3)) {
        (Some(world), Some(t)) => (world, t),
        _ => return -1,
    };
    guard(|| {
        let id = id as usize;
        for i in 0..world.subsystem_count() {
            if let Some(sub) = world.get_mut(i) {
                if let Some(rigid) = sub.as_any_mut().downcast_mut::<RigidSubsystem<f64>>() {
                    if id < rigid.world.bodies.len() {
                        let b = &mut rigid.world.bodies[id];
                        if b.inv_mass > 0.0 {
                            // I_world⁻¹ = rot * inv_inertia_local * rotᵀ
                            let iw = b.rot.to_rotation_matrix()
                                * b.inv_inertia_local
                                * b.rot.to_rotation_matrix().transpose();
                            let av = phy_math::Vec3::new(
                                iw[(0, 0)] * t[0] + iw[(0, 1)] * t[1] + iw[(0, 2)] * t[2],
                                iw[(1, 0)] * t[0] + iw[(1, 1)] * t[1] + iw[(1, 2)] * t[2],
                                iw[(2, 0)] * t[0] + iw[(2, 1)] * t[1] + iw[(2, 2)] * t[2],
                            );
                            b.ang_vel.x += av.x * dt;
                            b.ang_vel.y += av.y * dt;
                            b.ang_vel.z += av.z * dt;
                        }
                        return 0;
                    }
                }
            }
        }
        -1
    })
    .unwrap_or(-1)
}

/// 读回指定刚体的世界系线速度(3×f64)到 `out3`。返回 0 成功,-1 失败。
#[no_mangle]
pub extern "C" fn phy_world_rigid_get_velocity(
    w: *mut PhyWorldHandle,
    id: i64,
    out3: *mut f64,
) -> i32 {
    let (world, out) = match (as_world(w), nonnull_slice(out3, 3)) {
        (Some(world), Some(out)) => (world, out),
        _ => return -1,
    };
    guard(|| {
        let id = id as usize;
        for i in 0..world.subsystem_count() {
            if let Some(sub) = world.get(i) {
                if let Some(rigid) = sub.as_any().downcast_ref::<RigidSubsystem<f64>>() {
                    if id < rigid.world.bodies.len() {
                        let v = rigid.world.bodies[id].vel;
                        out[0] = v.x;
                        out[1] = v.y;
                        out[2] = v.z;
                        return 0;
                    }
                }
            }
        }
        -1
    })
    .unwrap_or(-1)
}

/// 读回指定刚体的世界系角速度(rad/s,3×f64)到 `out3`。返回 0 成功,-1 失败。
#[no_mangle]
pub extern "C" fn phy_world_rigid_get_angular_velocity(
    w: *mut PhyWorldHandle,
    id: i64,
    out3: *mut f64,
) -> i32 {
    let (world, out) = match (as_world(w), nonnull_slice(out3, 3)) {
        (Some(world), Some(out)) => (world, out),
        _ => return -1,
    };
    guard(|| {
        let id = id as usize;
        for i in 0..world.subsystem_count() {
            if let Some(sub) = world.get(i) {
                if let Some(rigid) = sub.as_any().downcast_ref::<RigidSubsystem<f64>>() {
                    if id < rigid.world.bodies.len() {
                        let av = rigid.world.bodies[id].ang_vel;
                        out[0] = av.x;
                        out[1] = av.y;
                        out[2] = av.z;
                        return 0;
                    }
                }
            }
        }
        -1
    })
    .unwrap_or(-1)
}

#[cfg(test)]
mod tests {
    use std::ffi::CString;
    use super::*;

    #[test]
    fn ffi_fluid_lifecycle_and_readback() {
        let w = phy_world_create_fluid();
        assert!(!w.is_null(), "create_fluid 应返回非空句柄");
        assert!(phy_world_sub_count(w) >= 1);
        let n = phy_world_fluid_count(w);
        assert!(n > 0, "流体世界应有粒子");

        // 步进 200 步,位置应发生变化且不崩溃。
        let t0 = phy_world_time(w);
        for _ in 0..200 {
            assert_eq!(phy_world_step(w, 0.005), 0);
        }
        let t1 = phy_world_time(w);
        assert!((t1 - t0 - 1.0).abs() < 1e-6, "步进后时间应推进约 1.0");

        // 读回位置。
        let mut buf = vec![0.0f64; n * 3];
        let got = phy_world_get_fluid_positions(w, buf.as_mut_ptr(), buf.len());
        assert_eq!(got, n);
        // 不应有 NaN。
        assert!(
            buf.iter().all(|v| v.is_finite()),
            "流体位置不应含 NaN/Inf"
        );

        phy_world_destroy(w);
    }

    #[test]
    fn ffi_save_load_roundtrip() {
        let w = phy_world_create_coupled();
        assert!(!w.is_null());
        for _ in 0..50 {
            assert_eq!(phy_world_step(w, 0.005), 0);
        }
        let path = std::env::temp_dir().join("phy_ffi_test_world.json");
        let cpath = CString::new(path.to_str().unwrap()).unwrap();
        assert_eq!(phy_world_save(w, cpath.as_ptr()), 0);

        let w2 = phy_world_load(cpath.as_ptr());
        assert!(!w2.is_null(), "load 应成功");
        assert_eq!(phy_world_sub_count(w2), phy_world_sub_count(w));
        phy_world_destroy(w);
        phy_world_destroy(w2);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn ffi_null_pointer_is_safe() {
        // 空指针不应 panic,应返回安全哨兵值。
        assert_eq!(phy_world_step(std::ptr::null_mut(), 0.01), -1);
        assert!(phy_world_time(std::ptr::null_mut()).is_nan());
        assert_eq!(phy_world_fluid_count(std::ptr::null_mut()), 0);
        phy_world_destroy(std::ptr::null_mut()); // 安全 no-op
    }

    #[test]
    fn ffi_checked_step_reports_health() {
        let w = phy_world_create_fluid();
        assert!(!w.is_null());
        // 健康世界:200 步后看门狗应始终返回 0(有限值)。
        for _ in 0..200 {
            let rc = phy_world_step_checked(w, 0.005);
            assert_eq!(rc, 0, "流体世界不应触发看门狗");
        }
        // 空指针返回 -1。
        assert_eq!(phy_world_step_checked(std::ptr::null_mut(), 0.01), -1);
        phy_world_destroy(w);
    }

    #[test]
    fn ffi_f32_channel_matches_f64() {
        // P3:f32 读回通道应与 f64 读回在 f32 精度内一致(边界只是 cast)。
        let w = phy_world_create_fluid();
        assert!(!w.is_null());
        for _ in 0..120 {
            assert_eq!(phy_world_step_f32(w, 0.005), 0);
        }
        let n = phy_world_fluid_count(w);
        assert!(n > 0);

        let mut buf64 = vec![0.0f64; n * 3];
        let mut buf32 = vec![0.0f32; n * 3];
        let g64 = phy_world_get_fluid_positions(w, buf64.as_mut_ptr(), buf64.len());
        let g32 = phy_world_get_fluid_positions_f32(w, buf32.as_mut_ptr(), buf32.len());
        assert_eq!(g64, n);
        assert_eq!(g32, n);

        // f32 读回应是 f64 的精确 cast(逐元素相等)。
        for i in 0..n * 3 {
            assert_eq!(buf32[i], buf64[i] as f32, "f32 通道应为 f64 的精确 cast");
            assert!(buf32[i].is_finite());
        }
        phy_world_destroy(w);
    }

    #[test]
    fn ffi_f32_null_pointer_is_safe() {
        // f32 通道空指针同样安全。
        assert_eq!(phy_world_step_f32(std::ptr::null_mut(), 0.01), -1);
        assert_eq!(
            phy_world_get_fluid_positions_f32(std::ptr::null_mut(), std::ptr::null_mut(), 0),
            0
        );
        assert_eq!(
            phy_world_get_rigid_transforms_f32(std::ptr::null_mut(), std::ptr::null_mut(), 0),
            0
        );
    }
}
