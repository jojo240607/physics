# M4 业务集成指南（最小可用 API）

> 目标：让业务方在 **10 行代码内** 搭出一个可 step 的物理世界，并理解版本兼容边界。
> 配套示例：`crates/phy-demo/examples/integration_minimal.rs`

## 1. 核心抽象（稳定公共 API）

| 层 | 类型 | 职责 |
|---|---|---|
| 内核 | `phy_core::World<T>` | 持有所有子系统 + 共享状态，驱动 `step` |
| 内核 | `phy_core::Subsystem` trait | 任意物理规则的统一接口（`step` / `validate` / `kinetic_energy` …）|
| 内核 | `World::step_checked(dt)` | 带看门狗的步进：任一子系统 NaN/Inf 即返回 `Err<WorldError>` |
| 流体 | `phy_fluid::FluidWorld<T>` + `FluidSubsystem` | SPH 自由液面 |
| 颗粒 | `phy_granular::GranularWorld<T>` + `GranularSubsystem` | PBD 接触堆积 |
| 刚体 | `phy_rigid::RigidWorld<T>` + `RigidSubsystem` | 碰撞/关节/车辆 |
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
