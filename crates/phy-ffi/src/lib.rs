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
