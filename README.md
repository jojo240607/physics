# physics — 多物理场仿真引擎(Rust / C ABI)

一个**确定性、可库化**的 Rust 多物理场仿真引擎,覆盖流体(SPH)、刚体(接触/CCD/关节/车辆/破碎)、颗粒(PBD)、软体(质点-弹簧)、光学(折射/焦散)、标量场(热/波/烟/声)等多子系统,统一挂在 `World<T>` 上按帧驱动并支持双向耦合。

面向**游戏 / 防战建模 / 科学可视化**业务:既可作为 Rust 库直接依赖,也可经 **C ABI(`phy-ffi`)** 供 C/C++/Unity(C# P/Invoke)/Unreal 调用。

---

## 特性

- **统一世界 `World<T>`**:子系统(`Subsystem` trait)按 `step → couple` 顺序驱动,耦合阶段可跨子系统双向交互(流体↔刚体、软体↔流体、四向热耦合等)。
- **确定性**:固定步长 + Jacobi 并行(rayon)+ 固定约化顺序,重复运行逐位一致,存档重放一致(见 `crates/phy-demo/tests/determinism.rs`)。
- **数值安全看门狗**:`World::step_checked` / `step_skipping_checked` 在每帧后逐子系统扫描 NaN/Inf(`Subsystem::validate`),异常即返回 `Err(WorldError)`,不再静默污染下游。
- **C ABI 稳定接口**:`phy-ffi` 暴露不透明句柄(`PhyWorldHandle *`)与 `extern "C"` 函数,panic 被 `catch_unwind` 兜住,绝不跨 FFI 边界 unwind。
- **Web GPU 后端(可选)**:`wasm32 + feature=gpu` 下经 WebGPU 把流体/颗粒/光学/焦散搬到 GPU,运行时可切换(W7)。**仅限浏览器演示**,库化交付默认走 CPU。

---

## 工作区结构

| crate | 内容 |
|---|---|
| `phy-core` | `World` / `Subsystem` 抽象、事件总线、统计、看门狗(`WorldError` / `validate` / `step_checked`) |
| `phy-math` | `Vec3` / `Quat` / `Mat3` 数学原语(nalgebra 封装) |
| `phy-fluid` | SPH 流体(密度/压力/粘性 + 双向耦合) |
| `phy-rigid` | 刚体(顺序冲量 + CCD + 关节 + 车辆 + Voronoi 破碎) |
| `phy-granular` | 颗粒介质(PBD 接触约束 + 边界) |
| `phy-soft` | 软体(质点-弹簧 + velocity-Verlet) |
| `phy-optics` | 几何光学(折射/反射/焦散) |
| `phy-field` | 标量场(热扩散 / 波动 / 烟 / 声) |
| `phy-solid` | 固体 FEM |
| `phy-io` | JSON / CSV / 轨迹存档载入(serde) |
| `phy-sdk` | 集成 SDK:`PhysicsBuilder` 声明式建世界 + 强类型句柄重导出 + 教程 doctest(M4) |
| `phy-demo` | 桌面演示 + 集成/E2E 测试 + 性能基准(`benches/perf.rs`) |
| `phy-demo-web` | `wasm32 + gpu` 浏览器演示(WebGPU) |
| `phy-ffi` | C ABI 层(`cdylib` + `rlib`)+ 生成 `phy_ffi.h` |

---

## 快速上手(Rust)

```toml
# Cargo.toml
[dependencies]
phy-core   = { path = "crates/phy-core" }
phy-fluid  = { path = "crates/phy-fluid" }
phy-rigid  = { path = "crates/phy-rigid" }
phy-math   = { path = "crates/phy-math" }
nalgebra   = "0.33"
```

```rust
use phy_core::{World, Subsystem};
use phy_fluid::{FluidWorld, FluidSubsystem, SphParams};
use phy_math::Vec3 as V3;

// 1) 造一个流体世界
let mut fluid = FluidWorld::<f64>::new(SphParams::<f64>::defaults());
fluid.fill_box(V3::new(-1.0, 1.0, -1.0), V3::new(1.0, 3.0, 1.0), 0.3, 0.1);

// 2) 挂进统一 World
let mut world: World<f64> = World::new();
world.add_subsystem(Box::new(FluidSubsystem::new(fluid)));

// 3) 普通步进(零开销,不跑看门狗)
for _ in 0..200 {
    world.step(0.01);
}

// 4) 业务需要"数值必须有限"时用看门狗步进
let mut world2: World<f64> = World::new();
world2.add_subsystem(Box::new(FluidSubsystem::new(
    FluidWorld::<f64>::new(SphParams::<f64>::defaults()),
)));
for _ in 0..200 {
    // 返回 Ok(()) 表示本帧全部有限;Err(WorldError) 表示出现 NaN/Inf
    world2.step_checked(0.01).expect("仿真发散");
}
```

更多示例见各 crate 公共 API 的文档注释(含可运行的 doctest:`cargo test --doc`)。

### 集成 SDK(推荐业务接入)

若你不想逐个拼装 `phy-*` crate,可直接依赖 **`phy-sdk`** —— 它把子系统拼装、强类型句柄取回、存档封装成统一入口,并提供可运行的集成教程 doctest:

```toml
# Cargo.toml
[dependencies]
phy-sdk = { path = "crates/phy-sdk" }
```

```rust
use phy_sdk::{PhysicsBuilder, World, get_as_mut, granular::{GranularSubsystem, Grain}};

// 1) 声明式启用子系统,自动建世界并挂载(索引=声明顺序)。
let mut world: World<f64> = PhysicsBuilder::new().granular().build();

// 2) 取回强类型句柄做细粒度控制(Grain::new(pos, radius, mass))。
if let Some(g) = get_as_mut::<GranularSubsystem<f64>>(&mut world, 0) {
    for i in 0..100 {
        let x = (i % 10) as f64 * 0.2 - 1.0;
        let y = (i / 10) as f64 * 0.2;
        g.world.add(Grain::new(nalgebra::Vector3::new(x, y, 0.0), 0.1, 1.0));
    }
}

// 3) 步进(也支持 world.step_checked(dt) 取回数值看门狗 Err)。
for _ in 0..30 { world.step(1.0 / 60.0); }

// 4) 存档:save_world_json(&world) 序列化为字符串,load_world_json(&s) 读回。
```

`PhysicsBuilder` 支持 `.fluid() / .rigid() / .granular() / .soft() / .field() / .optics() / .solid()` 任意组合;句柄经 `get_as::<T>(&world, idx)` / `get_as_mut::<T>(&mut world, idx)` 取回(类型不符或越界返回 `None`,不 panic)。详见 `cargo doc -p phy-sdk`。

### 性能基线

```bash
cargo bench -p phy-demo   # SPH 溃坝 N 体 / 刚体 M 体接触 单帧耗时基线
```

---

## 快速上手(C / C++ / Unity / Unreal)

构建动态库(产出 `target/release/phy_ffi.dll` / `.so` / `.dylib` + `phy_ffi.h`):

```bash
cargo build -p phy-ffi --release
```

把 `phy_ffi.h`(位于 `crates/phy-ffi/phy_ffi.h`)与编译出的库链接进你的工程。`phy_ffi.h` 只暴露**不透明句柄** `PhyWorldHandle *`,不泄露任何 Rust 模板类型,对 C/C++/C# 完全安全。

```c
#include "phy_ffi.h"

PhyWorldHandle *w = phy_world_create_fluid();   // 流体(溃坝)世界
for (int i = 0; i < 200; i++) {
    int rc = phy_world_step_checked(w, 0.005);  // 0=健康 / -1=空指针/panic / 2=NaN / 3=卡死
    if (rc != 0) { /* 报警 / 回滚 */ break; }
}
double t = phy_world_time(w);
size_t n = phy_world_fluid_count(w);
// 读回粒子:double buf[3*n]; phy_world_get_fluid_positions(w, buf, n);
phy_world_destroy(w);                            // 释放(空指针为安全 no-op)
```

可用工厂:`phy_world_create_fluid` / `phy_world_create_rigid` / `phy_world_create_granular` / `phy_world_create_coupled`。
读回:`phy_world_get_fluid_positions` / `..._velocities`、`phy_world_get_rigid_transforms`、`phy_world_sub_count` / `phy_world_fluid_count` / `phy_world_rigid_count`。
存档:`phy_world_save(w, path)` / `phy_world_load(path) -> PhyWorldHandle *`(可空)。

> 重新生成头文件(仅开发期需要):`PHY_FFI_GEN_HEADER=1 cargo build -p phy-ffi`。

---

## 构建与测试

```bash
cargo build --workspace                 # 全量构建
cargo test --workspace                  # 单元测试 + E2E(默认跳过重负载回归)
cargo test --release -p phy-demo -- --ignored   # 完整数值回归(确定性 + 稳定性,~1 分钟)
cargo test --doc                        # 文档 doctest
cargo bench -p phy-demo                 # 性能基线
```

---

## 库化业务的 GPU 约束

- **GPU 后端仅限 Web 演示**(`wasm32 + feature=gpu`,浏览器 WebGPU):A 档(SPH 受力、颗粒 PBD 接触、光学逐像素、焦散)上 GPU,W7 可运行时接管流体/颗粒 step。
- **库化交付默认 CPU(rayon 并行)确定性实现,不含 GPU**:桌面 wgpu 在 MinGW 工具链下链接崩溃,且刚体/关节/破碎/软体本就不适合 GPU。
- 若业务确需 GPU:换非 MinGW 工具链(Linux / macOS / **MSVC VS2019+**)可重新启用 wgpu 桌面后端;或以已验证的 CPU 数值内核为参考实现自写 compute shader。

---

## 状态

| 里程碑 | 状态 |
|---|---|
| L1 数学/全局稳定性回归 | ✅ |
| L2 数值确定性(重复 / 存档重放) | ✅ |
| L3 C ABI(`phy-ffi`,不透明句柄 + panic 守卫) | ✅ |
| L4 性能基线 + NaN 看门狗 + W7 路由 CPU 验证 | ✅ |

剩余:① 文档/doctest 持续补充中;② GPU 数值一致性需浏览器端人工核对(见 `PLAN.md` §5.8 L4-3)。

详见 [`PLAN.md`](./PLAN.md)(完整路线图与里程碑记录)。
