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

| **M4 软体** | 质点-弹簧软体(Mass-Spring)+ velocity-Verlet 集成 + 接入 World/Demo | 可形变体(布料/果冻) | ✅ 新建 `phy-soft` crate:`Particle`(pos/vel/inv_mass) + `Spring`(Hooke+阻尼,结构/剪切/弯曲) + `SoftBody`(规则 3D 晶格生成,顶部层钉扎) + `SoftSubsystem` 适配 `phy_core::Subsystem`。积分用 **velocity-Verlet**(对常加速度精确,首帧无偏),弹簧力 + 重力 + 地面碰撞(单向推出+法向反弹)。8 测试全过(自由落体匹配解析解、拉伸弹簧回弹至 rest、晶格无 NaN、不穿透地面、子系统适配 World、静态球推出、可动球双向冲量)。Demo 新增 `Soft` 模式(`O` 循环/`S` 直达):质点(按速度青→洋红着色)+ 弹簧线段(灰白)渲染 |

| **M4b 软-刚耦合** | 软体质点↔刚体(M3)碰撞双向耦合 | 软体把球推开/球压扁软体 | ✅ `SoftBody::collide_body(&Body)->Vec3` 用刚体形状 `contains_local` 检测穿透,沿外法线(6 轴支撑估算法线 + 二分求穿透深度)推出质点并反射法向速度;可动刚体按 `body_coupling` 收到反向冲量。`Subsystem` trait 加 `as_any_mut`(6 个 impl 补默认) |

| **M4c 耦合架构统一** | 把跨子系统耦合并入 `World::couple` 阶段 | 统一耦合入口,移除 Demo 特例 | ✅ `Subsystem::couple` 签名改为 `(&mut self, world: &mut World<T>, dt: &T)`;`World::step` 在调每个子系统 `couple` 前用 `remove`/`insert` 临时把它移出 `subsystems`,使 `couple` 内能经 `world.get_mut(j)` 安全可变访问其他子系统而无别名。`SoftSubsystem::couple` 据此直接对刚体子系统做 `collide_body`(两阶段:克隆刚体快照→可变软体算碰撞+冲量→可变刚体施加),`Scene` 不再特判 `couple_soft_rigid`。`raster.rs` 的 `draw_mesh`/`raster_triangle`/`edge` 加 `#[allow(dead_code)]`(预留网格光栅化原语)。`cargo test --workspace` 全过(软体 8、刚体 13、流体 6、场 5、光学 5) |

| **M4d 软-流耦合** | 软体质点↔流体(M5)双向耦合(浮力 + 阻力 + 动量交换) | 软体漂浮/沉入流体、推开流体 | ✅ `phy-fluid` 新增 `CouplePoint<T>`(pos/vel/mass/force 输出)+ `FluidWorld::couple_points(&mut self, &mut [CouplePoint], dt, drag, soft_density)`:对盒内点用均匀网格采样邻域平均流速 `v_f`,施加浮力 `f=−ρf·(mass/ρsoft)·g` + 阻力 `drag·mass·(v_f−v)`,并把等大反向冲量按质量比分配到邻域流体粒子(soft→fluid 动量交换)。`phy-soft` 加 `phy-fluid` 依赖;`SoftSubsystem` 新增 `fluid_drag`/`soft_density` 字段,`couple` 动态查找流体子系统下标(不依赖注册顺序)→打包质点成 `CouplePoint`→写回速度+累加力(供下一帧 velocity-Verlet)。新增单测 `couple_points_buoyancy_upward`(浮力向上+流体反向动量)与端到端 `world_couples_soft_fluid_rigid`(World 挂刚/流/软,step 40 帧无 NaN 且浸入点受浮力)。`cargo test --workspace` 全过(软体 9、刚体 13、流体 7、场 5、光学 5) |

| **M4e 流体-热场耦合** | 流体(M5)↔热场(M7)双向耦合(热浮力 + 平流加热) | 热流体自然对流、流体加热场 | ✅ `phy-field` 新增 `HeatFieldLike<T>` trait(`dims`/`origin`/`cell_size`/`sample_trilinear`/`add_source`,`: Any` 供 downcast),`ScalarField` 加 `origin: Vec3<T>` + `with_origin`(`step_diffusion` 已在 M7 把 `src` 项按 `src[i]*dt` 注入);`phy-core` 的 `World` 暴露 `remove`/`insert`(供 `couple` 阶段暂存自身);`phy-fluid` 加 `phy-field` 依赖,`FluidSubsystem` 新增 `thermal_expansion`/`heat_gain` 字段,`couple` 动态 downcast 查找 `HeatField` 下标(不依赖注册顺序)→`FluidWorld::couple_heat(&mut self, heat, dt, t_ref, beta, heat_gain)`:逐粒子三线性采样温度 `T`,密度 `ρ(T)=ρf/(1+β·(T−T_ref))`,热浮力修正 `a += −g·(ρf−ρT)/ρf`(向上),并以 `heat_gain·‖v‖·dt` 注入热源到所在网格单元(`add_source` 进 `src`,需 `step_diffusion` 才生效,避免凭空升温)。新增单测 `couple_heat_warmer_fluid_rises`(热粒子 acc.y > −9.81)与 `couple_heat_injects_source_into_moving_region`(运动粒子+`heat.field.step_diffusion` 后中心温度 > 0);端到端 `world_couples_fluid_heat_thermal_buoyancy`/`world_couples_fluid_heat_source_injection`(World 挂流+热,step 20 帧无 NaN 且中心升温)。`cargo test --workspace` 全过(软体 9、刚体 13、流体 9、场 5、光学 5) |

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
- 2026-08-08: M4 完成。新建 `phy-soft` crate: **质点-弹簧软体**。核心类型 `Particle`(pos/vel/inv_mass/force) + `Spring`(a,b,rest,k,damp,结构/剪切/面对角/体对角/跨2格长程) + `SoftBody`(规则 3D 晶格生成,顶部层钉扎 inv_mass=0 模拟悬挂) + `SoftSubsystem` 适配 `phy_core::Subsystem`。**积分器选 velocity-Verlet**(非位置 Verlet):标准位置 Verlet 对常加速有 O(dt) 全局误差(首帧 `x1 = x0 + a·dt²` 而非 `x0 + 0.5·a·dt²`,偏差随帧数累积 ~0.5·a·t·dt),velocity-Verlet 两遍算力、位置更新用旧速度+半步旧加速度、速度更新用新旧加速度均值,对常加速精确到机器精度(实测自由落体 y 误差 <1e-12 vs 解析解)。5 测试全过。Demo 接入 `Soft` 模式(`O` 循环 / `S` 直达):质点按速率青→洋红着色 + 弹簧灰白线段;`raster.rs` 加 `draw_line`(Bresenham 带深度测试)渲染弹簧。软体-刚体双向耦合暂未做(单向静态地面碰撞),留待 World 的 `couple` 阶段。验证: `cargo test --workspace` 全过。
- 2026-08-08: M4b 完成。软体↔刚体(M3)双向碰撞耦合接入 Demo。`SoftBody::collide_body(&Body<T>) -> Vec3<T>`:对每个可动点,经 `Body::to_local` + `Shape::contains_local` 检测穿透;穿透时以 6 主轴支撑(`Shape::support_local`)估表面外法线,沿局部法线二分求穿透深度(`Shape::bounding_sphere_r` 定上界,20 次迭代)把质点推出,并反射法向速度(`body_restitution`);可动刚体(`inv_mass>0`)按 `body_coupling` 累计反向冲量并由调用方施加(`Body::apply_impulse`)。`Subsystem` trait 补 `as_any_mut(&mut self)`(6 个 crate 的 impl 各补 `{self}`),`World::get_mut` 已存在,渲染/耦合层可变 downcast 取回具体子系统。`phy-demo::Scene::couple_soft_rigid` 每帧 `world.step` 后两阶段执行:阶段一克隆刚体快照(释放对 World 的不可变借用)→可变借软体算碰撞+冲量;阶段二可变借刚体施加冲量——分离两阶段以规避 Rust 同一 `Vec<Box<dyn Subsystem>>` 内同时双可变借用的冲突。8 测试全过(含静态球推出、可动球双向冲量)。验证: `cargo test --workspace` 全过。
- 2026-08-08: M4c 完成。把跨子系统耦合并入 `World::couple` 统一阶段(落实 M8 设计预留的 "彼时 `step`/`couple` 可改为接收 `&mut World`")。`Subsystem::couple` 签名改为 `fn couple(&mut self, world: &mut World<T>, dt: &T)`(默认空实现);`World::step` 在调第 `i` 个子系统 `couple` 前用 `remove(i)` 把它移出 `subsystems`、以 `&mut World`(不含自身)为参调用、再 `insert(i)` 回原位——这样 `couple` 内可经 `world.get_mut(j)` 安全可变访问其他子系统而无别名(因 `me` 是独立局部 `Box`、与 `self.subsystems` 不重叠)。`SoftSubsystem::couple` 据此直接取出刚体子系统做 `collide_body`(两阶段:克隆刚体快照释放不可变借用→可变软体算碰撞+冲量→可变刚体施加),`Scene::step` 移除特例 `couple_soft_rigid`,耦合随 `World::step` 自动发生。`raster.rs` 的 `draw_mesh`/`raster_triangle`/`edge` 加 `#[allow(dead_code)]`(预留网格光栅化原语,当前 rigid 用线框/盒绘制)。验证: `cargo test --workspace` 全过(软体 8、刚体 13、流体 6、场 5、光学 5);耦合仍有效(同 M4b 测试覆盖)。
- 2026-08-08: M4d 完成。软体↔流体(M5)双向耦合。`phy-fluid` 新增 `CouplePoint<T>{pos,vel,mass,force}` 与 `FluidWorld::couple_points(&mut self, &mut [CouplePoint<T>], dt, drag, soft_density)`:对流体盒内点用均匀网格 `for_each_neighbor` 采样邻域平均流速 `v_f`,施加浮力 `f_b=−ρf·(mass/ρsoft)·g`(阿基米德,`ρsoft=soft_density`)+ 线性阻力 `f_d=drag·mass·(v_f−v)`,合力写入 `pt.force`;同时把等大反向冲量 `−(f_b+f_d)·dt` 按质量比分配到邻域流体粒子(soft→fluid 动量交换,流体被软体推开)。为避免 `phy-fluid`↔`phy-soft` 循环依赖,`phy-soft` 单向依赖 `phy-fluid`,`SoftSubsystem` 新增 `fluid_drag`/`soft_density` 字段;`couple` 中**动态遍历 `world` 查找刚体/流体子系统下标**(不依赖注册顺序——此前硬编码 `RIGID_IDX=1/FLUID_IDX=2` 与 Demo 实际顺序 rigid=0/fluid=1 不符,会导致耦合静默跳过,已修),把软体质点打包成 `CouplePoint` 交给 `couple_points`,再把合力写回质点速度+累加力(供下一帧 velocity-Verlet 使用)。新增单测 `couple_points_buoyancy_upward`(静止流体中淹没点受向上浮力且流体获向下反向动量)与端到端 `world_couples_soft_fluid_rigid`(World 挂刚/流/软,step 40 帧无 NaN 且浸入点 `force.y > −9.81`)。验证: `cargo test --workspace` 全过(软体 9、刚体 13、流体 7、场 5、光学 5)。
- 2026-08-08: M4e 完成。流体↔热场(M7)双向耦合。为避免 `phy-fluid`↔`phy-field` 循环依赖,采用 trait 解耦:`phy-field` 新增 `pub trait HeatFieldLike<T: RealField+Copy>: Any`(`dims`/`origin`/`cell_size`/`sample_trilinear`/`add_source`),`HeatField` 实现之并 delegate;`phy-fluid` **单向依赖 `phy-field`** 而非具体类型,`couple_heat` 只接受 `&mut dyn HeatFieldLike<T>`。世界空间↔网格映射需场有原点,故 `ScalarField` 加 `pub origin: Vec3<T>` + `with_origin`(`new` 默认零原点)。两个效应:(1) 热浮力——密度修正 `ρ(T)=ρf/(1+β·(T−T_ref))`,加速度修正 `a += −g·(ρf−ρT)/ρf`(β=`thermal_expansion`, 注意用 `−g` 因重力向量 y 分量本就为负,向上为 +);(2) 平流加热——以 `heat_gain·‖v‖·dt` 经 `add_source` 注入网格 `src`(热源需 `HeatField::step_diffusion` 才并入 `u`,避免凭空升温)。`World` 暴露 `remove`/`insert`(M4c `couple` 阶段已把自身移出,故 `couple` 内可 `world.remove(heat_idx)` 取回可变 `HeatField` 再 `couple_heat` 后 `insert` 回,无别名)。`FluidSubsystem::couple` **动态 downcast 查找 `HeatField` 下标**(不依赖注册顺序),`t_ref` 取场初值。`phy-demo::scene` 新增 `build_fluid_heat_world`(暖顶 50℃ 温度场 + 箱 −1..1 填流体,`thermal_expansion=0.5`/`heat_gain=0.1`)。新增单测 `couple_heat_warmer_fluid_rises`/`couple_heat_injects_source_into_moving_region` 与端到端 `world_couples_fluid_heat_thermal_buoyancy`/`world_couples_fluid_heat_source_injection`(step 20 帧无 NaN 且中心升温)。验证: `cargo test --workspace` 全过(软体 9、刚体 13、流体 9、场 5、光学 5);注意三线性采样偏移量须经 `T::to_f64()` 转浮点,不可用 `as` 转泛型。
