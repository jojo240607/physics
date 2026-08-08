# 物理引擎规划 (PLAN)

> 一个用 Rust 开发的、模块化可组合的**世界物理仿真内核**,目标是既能做科研仿真,
> 也能作为游戏引擎的物理底层,逼真模拟世界物理规则(刚体 / 流体 / 光学 / 其他场)。

---

## 1. 愿景与目标

- **核心定位**:不是单一物理引擎,而是一个**插件式的多物理场 World 内核**。
  刚体、流体、光学、热/电磁等,都是挂在同一个 `World` 上的"物理子系统 (Subsystem)"。
- **双用途**:
  - 科研向:高精度、可复现、轨迹/场导出、不变量回归测试。
  - 游戏向:实时、逼真、与 wgpu 渲染打通。
- **统一 API,可切换精度**:同一套接口,科研走高精度/离线路径,游戏走实时近似。

---

## 2. 关键设计决策(已与用户确认)

| 维度 | 决策 | 理由 |
|---|---|---|
| 数学库 | **nalgebra**,泛型 `RealField` (f32/f64 可切换) | 科研需精度与泛型,nalgebra 带 `RealField` 抽象与线性代数生态 |
| 架构 | 插件式 `Subsystem` trait + `World` 统一驱动 | 多物理场可组合、可双向耦合 |
| 流体 | **SPH 粒子法** | 与刚体双向耦合自然(浮沉、阻力),实时性与通用性最佳 |
| 光学 | **双后端可切换**(离线光路追踪 / 实时近似) | 科研要逼真、游戏要帧率,同一 API 两套后端 |
| 渲染 | **纯 Rust 软件光栅化**(winit + softbuffer) | 本机 MinGW 8.1 链接器对 wgpu 巨型依赖树崩溃(`corrupt .drectve`),故放弃 GPU 后端,改用 CPU 光栅化,保证任意工具链可编译运行 |
| 确定性 / WASM | 暂不做,但架构保持 `no_std` 友好以备将来 | 当前优先功能与逼真度 |

---

## 3. 目录结构 (Cargo workspace)

```
physics/
├── Cargo.toml              # workspace 根
├── PLAN.md                 # 本规划文档
├── crates/
│   ├── phy-core/           # World、Subsystem trait、time-step、事件总线、空间索引
│   ├── phy-math/           # nalgebra 泛型重导出 + 包围体/场/网格工具
│   ├── phy-rigid/          # 刚体动力学(含 collision broad/narrow + solver)
│   │   ├── collision/      # broad-phase(SAP) + narrow-phase(GJK/EPA/SAT)
│   │   └── solver/         # 顺序冲量约束求解(接触+摩擦)
│   ├── phy-fluid/          # 流体 SPH 粒子法
│   ├── phy-optics/         # 光学双后端(光路追踪 / 实时近似)
│   ├── phy-field/          # 其他连续场:热传导、电磁、引力场
│   ├── phy-io/             # 场景描述解析、轨迹/场导出(CSV/二进制)
│   └── phy-demo/           # wgpu 3D 可玩 Demo(刚体+流体+光学统一渲染)
└── tests/                  # 跨 crate 不变量回归测试
```

### 核心抽象(伪代码)

```rust
/// 任意物理子系统都实现此 trait,World 在每个 time-step 依次驱动。
trait Subsystem<T: RealField> {
    /// 推进自身一个时间步。
    fn step(&mut self, world: &mut World<T>, dt: T);
    /// 与其他子系统的交互(如流体推动刚体、光被玻璃折射)。
    fn couple(&self, world: &mut World<T>);
}

/// 世界:持有所有已注册子系统与共享状态(刚体、粒子、场、空间索引)。
struct World<T: RealField> {
    subsystems: Vec<Box<dyn Subsystem<T>>>,
    // ... 共享实体/场/事件总线
}
```

子系统间通过 `World` 共享状态做**双向耦合**(例如水面浮起刚体、光被玻璃折射)。

---

## 4. 里程碑

| 阶段 | 内容 | 交付 |
|---|---|---|
| **M0 地基** | workspace + crate 骨架; `phy-core` 的 `World`/`Subsystem`/`RealField` 接入; `phy-math` 基础类型; 单刚体自由落体解析校验 | ✅ `cargo test` 跑通(2 测试) |
| **M1 碰撞检测** | (phy-rigid) SAP broad-phase + 球/盒 SAT + 凸体 GJK-EPA; 接触点/法线/穿透深度 | ✅ 11 测试全过(球/盒/凸体相交与分离、旋转盒、SAP→Narrow 管线) |
| **M2 刚体动力学** | 半隐式欧拉积分 + 顺序冲量(SI)速度求解(接触+库仑摩擦) + Split-Impulse 位置修正; 接触分离容差(pen≈0 视为相交) | ✅ 13 测试全过(单盒落地不穿透、双盒堆叠稳定) |
| **M3 3D Demo** | N 盒/球落地堆叠,3D 实时渲染 + 轨道相机 + 交互(暂停/重置/加盒/加球) + 朗伯光照 | ✅ `phy-demo` 纯 Rust 软件光栅化(winit 0.30 + softbuffer 0.4),透视插值+Z 缓冲+朗伯光照;本机 MinGW 链接器对 wgpu 巨型依赖树崩溃,故从 wgpu 转向软件管线 |
| **M5 流体 SPH** | SPH 粒子法 + 与刚体耦合(浮沉/阻力) | 水面浮动刚体 | ✅ `phy-fluid` 新 crate: Müller 2003 弱可压缩 SPH(Poly6/Spiky/ViscLaplacian 核) + 均匀空间哈希邻居搜索; 6 测试全过(密度收敛、溃坝不越界、静/动态刚体耦合浮力、粒子数守恒) |
| **M6 光学双后端** | 离线光路追踪 / 实时近似,精度开关切换 | 折射/焦散/阴影 | ✅ `phy-optics` 新 crate: 复用 `phy_rigid::Shape` 几何 + `Surface`(albedo/IOR/roughness/transparent); Snell 折射 + Schlick-Fresnel + 反射数学; `Whitted`(递归反射/折射,离线)与 `Approx`(单次折射+阴影,实时)双后端实现同一 `Renderer` trait; `OpticSubsystem` 适配 `phy_core::Subsystem` 含 `render_camera` 针孔成像; 5 测试全过(法向折射方向、Fresnel 区间、球求交、离/实时渲染非空/透射增亮)。Demo 按 `O` 进入光学模式,实时渲染玻璃球+地面 |
| **M7 其他场** | 热/电磁等连续场(网格有限差分) | 可扩展场规则 | ✅ `phy-field` 新 crate: `ScalarField<T>` 三维规则网格标量场 + Dirichlet/Neumann 边界; `step_diffusion`(热扩散 FTCS ∂u/∂t=α∇²u)与 `step_wave`(波动/电磁标量 leapfrog ∂²u/∂t²=c²∇²u); `HeatField`/`WaveField` 适配 `phy_core::Subsystem`。5 测试全过(Neumann 热守恒、热核形状比值匹配解析解、1D 行波传播≈c·t 且稳定有界、无源波动幅度不爆炸、子系统适配)。关键陷阱:拉普拉斯模板不可再除 dx²(步进位已乘 α·dt/dx²),否则数值扩散被放大 dx² 倍导致解失真 |

| **M8 多物理场 Demo 集成** | `phy-demo` 用 `phy_core::World` 统一调度 M3/M5/M7,键盘循环切换渲染模式 | 一个可玩多物理界面 | ✅ `phy-demo` 重构为单 `World` 驱动:注册 `RigidSubsystem`(M3)+`FluidSubsystem`(M5)+`HeatField`(M7),`World::step(dt)` 顺序推进各子系统;`Subsystem` trait 加 `Any` supertrait + `as_any()` 方法、`World` 加 `get(i)`,使渲染层可 `downcast_ref` 取回具体子系统数据。Demo 模式 `Rigid/Fluid/Heat/Optics`(`O` 循环,`F`/`H` 直达,`R` 重置):刚体用点/盒光栅化,流体 SPH 粒子云(蓝点),热场 32×32 切片投影到地面(蓝→红色阶)。光学模式复用 M6 `OpticSubsystem::render_camera`。`cargo test --workspace` 全过

> 注:M4 预留给软体/约束优化与 WASM(用户暂未要求,架构已留接口)。

---

## 5. 验证策略

- 每个 milestone 都有**单元测试断言物理不变量**:
  - 自由落体位置符合解析解
  - 能量守恒 / 衰减合理
  - 无穿透(接触约束)
  - 堆叠稳定(关键不变量)
- `phy-io` 导出轨迹/场,供科研后处理与回归比对。
- Demo 同时作为"肉眼验证"与"帧率/性能基线"。

---

## 6. 决策记录 (Decision Log)

- 2026-08-07: 用户确认面向**通用科研/仿真**; 首期目标**完整可玩 Demo**; 数学库**由我推荐 → nalgebra 泛型**。
- 2026-08-07: 精度选**泛型可切换 (RealField)**; Demo 形态选 **wgpu 3D**。
- 2026-08-07: 用户扩展愿景至**流体 + 光学 + 其他物理规则**,目标**科研 + 游戏双用途、逼真模拟世界**。
- 2026-08-07: 流体选 **SPH 粒子法**; 光学选 **双后端可切换**; 确定性/WASM **暂不做**(架构预留)。
- 2026-08-07: M2 完成。求解器采用**顺序冲量 + 累积冲量钳制 + Split-Impulse 伪速度位置修正**(避免抖动/能量注入); 接触 SAT 加 `sep_eps=-1e-6` 容差,使恰好接触(pen≈0)被判为相交,修复堆叠测试中下盒穿地。
- 2026-08-08: M3 完成。Demo 因本机 MinGW 8.1 链接器对 wgpu 巨型依赖树崩溃(`corrupt .drectve`, `ld returned 5`),**从 wgpu 转向纯 Rust 软件光栅化方案**:`winit 0.30` + `softbuffer 0.4` 做窗口/帧缓冲呈现,自研 CPU 光栅化器(`raster.rs`:透视正确插值 + Z 缓冲 + 朗伯光照)渲染 `RigidWorld<f64>` 的盒/球实例。无需 GPU 后端,保证在任意 MinGW 工具链下可编译运行。交互:拖拽旋转、滚轮缩放、P 暂停、R 重置、G 加盒、B 加球、I 统计。
- 2026-08-08: M5 完成。`phy-rigid` 新增 `Shape::contains_local` + `Body::to_local/to_world`; 新建 `phy-fluid` crate: Müller 2003 弱可压缩 SPH(Poly6 密度/Spiky 压力梯度/ViscLaplacian 粘性核) + 均匀空间哈希邻居搜索。默认 h=0.2,单粒子质量由晶格核求和反算(`lattice_mass`)保证静止密度收敛。双向刚体耦合 `couple_bodies`: 静态体作不可穿透边界(位置推回 + 法向速度阻尼),动态体受阿基米德浮力 `-ρf·V_sub·g` + 无滑阻力反作用冲量。`FluidSubsystem` 适配 `phy_core::Subsystem`。6 测试全过(密度收敛、溃坝不越界、粒子数守恒、静/动态刚体耦合)。注意: 浮力测试须关闭重力隔离纯上举力,否则自由下落流体的下拽耦合会掩盖浮力。
- 2026-08-08: M6 完成。新建 `phy-optics` crate: 复用 `phy_rigid::Shape` 作几何 + 自研 `Surface`(albedo/IOR/roughness/transparent),光学体 `OpticBody` 由刚体位姿+形状+表面组成,`OpticScene` 统一求交(球/盒 Slab/凸体 Möller–Trumbore)。光学数学 `math.rs`: Snell `refract`(含 TIR 返回 None)、`reflect`、Schlick `fresnel`、`f0_of`。双后端实现同一 `Renderer` trait: `Whitted`(递归反射+折射,离线高保真)与 `Approx`(单次折射近似+阴影射线,实时)。`OpticSubsystem` 适配 `phy_core::Subsystem` 并提供 `render_camera` 针孔成像 + `Precision` 切换开关。`to_rgba8` 输出 ABGR 供软件帧缓冲。5 测试全过。`phy-demo` 接入: 按 `O` 切换光学模式,用 `Approx` 后端实时渲染玻璃球 + 地面(复用相机位姿)。
- 2026-08-08: M7 完成。新建 `phy-field` crate: `ScalarField<T>` 三维规则网格标量场 + Dirichlet/Neumann 边界(`neighbor_val` 越界按 bc 返回),`step_diffusion`(热扩散 FTCS 显式 ∂u/∂t=α∇²u)与 `step_wave`(波动/电磁标量 leapfrog 显式 ∂²u/∂t²=c²∇²u,零初速度自动由 u 复制 prior); `HeatField`/`WaveField` 适配 `phy_core::Subsystem`。5 测试全过。关键陷阱: `laplacian` 返回原始模板和(未除 dx²),步进位已乘 `α·dt/dx²`(扩散)或 `c²·dt²/dx²`(波动); 若 laplacian 再除 dx² 会把数值扩散放大 dx² 倍导致解失真(实测热核比值 0.21 vs 解析 0.0019)。验证: Neumann 下热守恒、热核形状比值匹配解析解、1D 行波传播距离≈c·t 且幅度有界、无源波动不爆炸。
- 2026-08-08: M8 完成。`phy-demo` 重构为**单 `World` 统一调度**:构造时向 `World<f64>` 注册 `RigidSubsystem`(M3 刚体,新增 `phy-rigid::subsystem` 适配)、`FluidSubsystem`(M5 SPH)、`HeatField`(M7 热场),每帧 `World::step(1/60)` 顺序推进;"统一调度"要求渲染层能取回各子系统内部数据,故 `phy-core` 给 `Subsystem<T>: Any` supertrait + 默认无(各 impl 显式提供 `fn as_any(&self)->&dyn Any{self}`),`World` 加 `get(i)->&Box<dyn Subsystem>`; Demo 渲染用 `world.get(IDX).unwrap().as_any().downcast_ref::<T>()` 取数据。模式 `Rigid/Fluid/Heat/Optics`(`O` 循环切换,`F`/`H` 直达,`R` 重置):刚体点/盒光栅化、SPH 粒子云(蓝点)、热场 32×32 切片投影地面色阶(蓝→红)、光学复用 M6 `OpticSubsystem::render_camera`。`Subsystem::as_any` 不能给默认实现 `self`(trait object 非 Sized),必须各 impl 提供 `{self}`。验证: `cargo test --workspace` 全过(M5 SPH 套件仍 ~106s,物理正确)。
