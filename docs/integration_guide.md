# 业务集成指南（物理引擎接入）

> 目标：让业务方在 **10 行代码内** 搭出一个可 step 的物理世界，并理解版本兼容边界。
> 两种入口：① 直接用底层 `phy-*` crate（见 `integration_minimal.rs`）；
> ② 用统一便捷层 [`phy-sdk`](../../crates/phy-sdk/src/lib.rs)（推荐，doctest 为单一事实源）。
> 文档更新：2026-08-12（新增 `phy-sdk` / 角色控制器 D4 / 场景 DSL 章节）。

## 1. 核心抽象（稳定公共 API）

| 层 | 类型 | 职责 |
|---|---|---|
| 便捷层 | `phy_sdk::{PhysicsBuilder, World, get_as, get_as_mut}` | 统一入口：一行声明子系统、取回强类型句柄、存档 |
| 内核 | `phy_core::World<T>` | 持有所有子系统 + 共享状态，驱动 `step` |
| 内核 | `phy_core::Subsystem` trait | 任意物理规则的统一接口（`step` / `validate` / `kinetic_energy` …）|
| 内核 | `World::step_checked(dt)` | 带看门狗的步进：任一子系统 NaN/Inf 即返回 `Err<WorldError>` |
| 流体 | `phy_fluid::FluidWorld<T>` + `FluidSubsystem` | SPH 自由液面 |
| 颗粒 | `phy_granular::GranularWorld<T>` + `GranularSubsystem` | PBD 接触堆积 |
| 刚体 | `phy_rigid::RigidWorld<T>` + `RigidSubsystem` + `CharacterController` | 碰撞/关节/车辆 + 胶囊角色（D4）|
| 软体 | `phy_soft::SoftBody<T>` + 软体子系统 | 绳索/布料 |
| 场 | `phy_field::*` | 热/波/烟/声学标量场 |
| 光学 | `phy_optics::OpticScene<T>` + 光学子系统 | 光线追踪 |

## 2. 最小工作流（伪代码）

```rust
let mut world: World<f64> = World::new();
world.add_subsystem(Box::new(FluidSubsystem::new(fluid_world)));
world.add_subsystem(Box::new(GranularSubsystem::new(granular_world)));

const DT: f64 = 1.0 / 60.0;
loop {
    match world.step_checked(DT) {
        Ok(()) => { /* 渲染 world 状态 */ }
        Err(e)  => { /* 安全降级: 回滚 / 暂停 / 上报 */ }
    }
}
```

完整可编译版本见 `integration_minimal.rs`。

> **推荐入口（`phy-sdk`）**：上面是直接拼装底层 crate 的"最薄"做法。日常业务接入更推荐
> 用 [`phy-sdk`](../../crates/phy-sdk/src/lib.rs) 的统一入口：`PhysicsBuilder` 一行声明子系统、
> `get_as`/`get_as_mut` 取回强类型句柄、`save_world`/`load_world` 存档。其所有 API 片段都以内嵌
> **doctest** 形式随源码维护（`cargo test -p phy-sdk` 全过），本文档不再手抄，以 doctest 为单一事实源。

## 2.1 推荐：用 `phy-sdk` 搭世界

```rust
use phy_sdk::{PhysicsBuilder, World};

// 一行声明要启用的子系统,SDK 自动建好内部 World 并挂载。
let mut world: World<f64> = PhysicsBuilder::new().rigid().fluid().build();
// 驱动:生产环境用 step_checked(看门狗),此处用 step。
for _ in 0..60 {
    world.step(1.0 / 60.0);
}
```

取回子系统做细粒度控制（索引 = `build()` 声明顺序）：

```rust
use phy_sdk::{get_as_mut, rigid::RigidSubsystem};

if let Some(rigid) = get_as_mut::<RigidSubsystem<f64>>(&mut world, 0) {
    println!("刚体数量 = {}", rigid.world.bodies.len());
}
```

## 2.2 角色控制器（D4，已接入 SDK / demo / DSL）

胶囊角色用 kinematic 体 + 每帧输入驱动，撞墙自动 slide，不会被碰撞推开：

```rust
use phy_sdk::{PhysicsBuilder, World, get_as_mut};
use phy_sdk::rigid::{RigidSubsystem, CharacterController};
use phy_math::Vec3;

let mut world: World<f64> = PhysicsBuilder::new().rigid().build();
let mut rigid = get_as_mut::<RigidSubsystem<f64>>(&mut world, 0).unwrap();
// 静态地面
rigid.world.add_body(phy_rigid::shape::Body {
    shape: phy_rigid::shape::Shape::Box { half: Vec3::new(20.0, 0.5, 20.0) },
    pos: Vec3::new(0.0, -0.5, 0.0),
    inv_mass: 0.0,
    ..Default::default()
});
let mut cc = CharacterController::new(&mut rigid.world, Vec3::new(0.0, 3.0, 0.0));
cc.speed = 5.0; cc.jump_speed = 6.0;
// 每帧:玩家输入 (dir, jump) 驱动 + step
for _ in 0..240 {
    cc.update(&mut rigid.world, 1.0 / 120.0, Vec3::zeros(), false);
    rigid.world.step(1.0 / 120.0);
}
```

## 2.3 关卡 DSL（JSON `SceneDesc`）

`phy-rigid` 的 `SceneDesc` 支持角色出生点，可经 JSON 反序列化一键落地 `CharacterController`：

```rust
use phy_rigid::scene::SceneDesc;

let json = r#"{
  "bodies": [],
  "character_spawn": { "pos": [0.0, 3.0, 0.0], "speed": 5.0, "jump_speed": 7.0 }
}"#;
let desc: SceneDesc<f64> = serde_json::from_str(json).unwrap();
assert!(desc.character_spawn.is_some());
// load_scene_json 会据 character_spawn 自动建好 RoleController 并挂到 RigidWorld.character。
```


## 3. 版本与兼容承诺

- **语义化版本**：当前 `0.1.0`（预发布）。`0.x` 期间 **API 可能破坏性变更**，升级前请读 `CHANGELOG.md`。
- **ABI 稳定性契约**（P7）：`phy-ffi` 的 C/WASM 接口有版本化 ABI 契约，跨语言嵌入（C/JS）以 `phy-ffi` 为唯一稳定边界；Rust 业务侧以 `phy-core` + 各 `phy-*` crate 的公共类型为准。
- **看门狗契约**：`step_checked` 保证"非有限值不污染下一帧"，业务侧**必须**用 `step_checked` 而非裸 `step` 接入生产循环。

## 4. 已知集成限制（务必先读）

1. **Granular 规模悬崖**（G2 ⚠️）：`GranularWorld` 在 >1000 实体时单帧性能崩溃（见 `perf_baseline.md`）。业务侧颗粒场景 >1k 需等待 M2-fix 或自行分块。
2. **SPH 能量不守恒**（G3 ⚠️）：无外力闭合系统下 SPH 数值能量自发注入（见 `stability.rs` 注释）。长时积分场景需业务侧监控动能或等 M3-fix。
3. **原生后端未验证**（G5）：桌面原生（非 wasm）wgpu 路径在本机未打通，当前仅 wasm/WebGPU 路径可用。

## 5. 接入检查清单

- [ ] 用 `World::step_checked` 接入主循环（不用裸 `step`）
- [ ] 处理 `Err(WorldError)` 的安全降级分支
- [ ] 确认目标场景粒子数在已验证规模内（SPH ≤10k / Granular ≤1k，见 `perf_baseline.md`）
- [ ] 锁版本（Cargo.lock）+ 订阅 `CHANGELOG.md`
- [ ] 长时场景加动能监控（防 G3 能量注入）
