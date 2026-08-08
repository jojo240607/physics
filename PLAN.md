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

| **M9 热流体接入 Demo** | 把 M4e 流体↔热双向耦合接入 `phy-demo` 实时渲染(可观察自然对流) | `q` 键进入 Fluid+Heat 模式看对流 | ✅ `DemoMode` 新增 `FluidHeat`(`next` 循环纳入、`name` 返回 "Fluid+Heat (M4e)");`Scene::new`(默认场景)即启用 M4e 耦合(流体子系统 `thermal_expansion=0.5`/`heat_gain=0.1`),故任意含流体+热的场景默认演示对流;`Scene::fluid_heat(warm_top)` 新构造器提供暖顶冷底专注对流场景,`set_mode(FluidHeat)`/`reset` 据当前模式重建(`FluidHeat`→`Scene::fluid_heat(true)`,否则 `Scene::new`);新增 `render_fluid_heat`(先画热场中间层切片:蓝→红温度色阶,再叠流体粒子蓝点,动态 downcast 查流体/热场下标不依赖注册顺序);`main.rs` 绑定 `q` 键进 `FluidHeat`、`o` 仍循环切换。验证: `cargo build -p phy-demo` 干净(仅预存警告),`cargo test --workspace` 全过(43 测试 0 失败)。 |

| **M10 全耦合综合 Demo** | 把刚/流/软/热四子系统共存于同一 World 并同帧渲染全部(M4b/d/e 耦合合流) | `a` 键进入 All 模式看全耦合 | ✅ `DemoMode` 新增 `All`(`next` 循环接在 `FluidHeat` 后回 `Rigid`,`name` 返回 "All (M10)");`All` 模式复用默认 `Scene::new`(已同时注册刚/流/软/热且开启全部耦合:刚↔流体 M4b、软↔流体 M4d、流体↔热 M4e),`set_mode`/`reset` 对 `All` 保持 `Scene::new`(与 `FluidHeat` 专属重建分支并列,不互相踩);新增 `render_all`:由远及近画**热场切片(蓝→红)→ 软体弹簧(灰白)→ 刚体实体 → 流体粒子(蓝)**,四个子系统下标均**动态 downcast 查找**(不依赖注册顺序,与 M4c/d/e 一致),展示所有耦合在一个 `World` 中实时共存;`main.rs` 绑定 `a` 键进 `All`,启动横幅更新键位(Q 流体+热 / A 全耦合)。验证: `cargo build -p phy-demo` 干净(仅预存警告),`cargo test --workspace` 全过(软体 9、刚体 13、流体 9、场 5、光学 5,共 43 测试 0 失败)。 |

| **M11 四向耦合补全(刚↔热 / 软↔热)** | 闭环刚/流/软/热四向耦合矩阵缺失的两条边:刚体↔热场、软体↔热场双向耦合 | 刚体/软体受热浮力上举并加热场,四向耦合矩阵完整 | ✅ 补齐耦合矩阵的最后一环——此前只有刚↔流(M4b)、软↔流(M4d)、流↔热(M4e),缺刚↔热与软↔热。`phy-field` 新增自由函数 `world_to_cell`(世界坐标→网格下标+三线性偏移,钳到边界)+ `sample_world`(经 `HeatFieldLike::sample_trilinear` 采样);`phy-rigid`/`phy-soft` 新增 `phy-field` 依赖,各自 `couple_heat(&mut self, heat, dt, t_ref, beta, heat_gain)`:逐物体/质点三线性采样温度 `T`、按 `ρ(T)=ρ0/(1+β·(T−T_ref))` 算热浮力加速度修正 `a=−g·(1−ρT)`(`g` 沿 −Y,故 `−g` 向上,与 M4e 一致)、`vel += a·dt`(在 `couple` 阶段、下一帧 `step` 生效),并以 `heat_gain·‖v‖·dt` 经 `add_source` 注入热源(M4e 同款避免凭空升温)。`RigidSubsystem`/`SoftSubsystem` 补 `thermal_expansion`/`heat_gain`/`t_ref` 字段(默认 0),`couple` 动态 downcast 找 `HeatField` 经 `remove`/`insert` 安全取可变引用后调用;`Scene::new` 给刚/软子系统设 `thermal_expansion=0.5`/`heat_gain=0.1`(与流体对齐),四向矩阵全开。新增单测 `couple_heat_warmer_body_rises`/`couple_heat_injects_source_into_moving_body`(刚)、`couple_heat_warmer_particle_rises`/`couple_heat_injects_source_into_moving_particle`(软)与端到端 `world_couples_rigid_soft_heat`(World 挂刚/软/热,step 10 帧无 NaN 且热浮力抵消部分重力)。**关键陷阱**:流体 M4e 的 `t_ref` 曾错误地从热场中心格采样,若物体正好落在最热格(温度=采样值)浮力会恰好归零——M11 把 `t_ref` 改为子系统显式配置的环境参考温度(默认 `T::zero()`,即环境温度基线 `ρ(T_ref)=ρ0`),物理上才正确且与单测 `t_ref=0` 一致。验证: `cargo test --workspace` 全过(软体 11、刚体 15、流体 9、场 5、光学 5、demo 3)。 |
| **M12 电磁场子系统 + 刚体↔电磁双向耦合** | 补完 `phy-field` 愿景里的"电磁"格:电荷密度→泊松松弛得电势→`E=−∇φ` 电场,带电刚体受洛伦兹力 `F=q(E+v×B)` 并反向把电荷沉积进网格,闭合"刚↔电磁"耦合边 | 带电刚体在电磁场中被洛伦兹力偏转/加速,且运动电荷反向塑造电场 | ✅ `phy-field` 抽取 `GridGeometry<T>` supertrait(dims/origin/cell_size),`HeatFieldLike`/`EmFieldLike` 均继承它,`world_to_cell`/`sample_world` 改为对 `GridGeometry` 泛型(热/电磁场共用采样辅助,零重复);新增 `EmFieldLike`(sample_e_field 三线性矢量采样 + add_charge + b_ext)与 `EmField<T>` 子系统:持有电荷密度 `rho`(ScalarField)、工作电势 `phi`、电场矢量 `e`(每格 Vec3)、外加均匀 `b_ext`;`step` 用 Jacobi 松弛解泊松 `∇²φ=−ρ/ε`(Neumann 镜像边界,30 次迭代)+ 中心差分 `E=−∇φ`,电场随电荷分布实时演化。`phy-rigid` 给 `RigidWorld` 加并行 `charges: Vec<T>`(不污染 `Body`)与 `add_charged_body(b,q)`、`couple_em(em,dt,k)`:逐带电体采样 `E`、施洛伦兹力 `vel += (q(E+v×b_ext)/m)·dt·k`(下一帧 `step` 生效,与 M11 热浮力同约定)、并以 `q·‖v‖·dt` 经 `add_charge` 沉积电荷实现反向耦合;`RigidSubsystem` 加 `em_coupling` 字段,`couple` 动态 downcast 找 `EmField` 经 `remove`/`insert` 驱动。`Scene::new` 注册 `EmField`(与热场同几何,dx=0.5,`b_ext=+0.5z`),掉落小球改用 `add_charged_body`(中间 +5、两侧 −5,相反电荷反向偏转),`rigid_sub.em_coupling=1.0`。新增单测 `em_poisson_yields_outward_e_from_positive_charge`/`em_sample_e_field_trilinear_finite`/`em_add_charge_accumulates_into_rho`(场)、`couple_em_electric_force_on_charge`/`couple_em_velocity_cross_b_deflects`/`couple_em_deposits_charge_from_moving_body`(刚)、端到端 `world_couples_rigid_em`(World 挂刚/电磁,step 30 帧 vy 比纯重力更负、vx 衰减、rho.src 沉积>0)。验证: `cargo test --workspace` 全过(软体 11、刚体 18、流体 9、场 8、光学 5、demo 4;dam_break 套件仍 ~106s)。**关键约束**:电荷沉积经 `rho.src`(源缓冲)、由 `EmField::step` 注入 `u`,不能直接断言 `u`(与 M11 热源 `HeatField::add_source` 同机制);`add_charge` 因此也不即时改写 `u`。 |
| **M13 引力场子系统 + 刚体↔引力双向耦合** | 补完 `phy-field` 愿景里的"引力场"格(最后一格):质量密度→泊松松弛得引力势→`g=−∇Φ` 引力加速度,刚体被局部引力井吸引并被运动质量反向塑造引力井,闭合"刚↔引力"耦合边。叠加在 `RigidWorld.gravity` 均匀重力之上,表示空间变化的质量分布 | 刚体被静态天体形成的局部引力井向下/向内吸引偏转,且运动团块反向沉积质量塑造引力井 | ✅ 复用 M12 的 `GridGeometry` + Jacobi 泊松基础设施,新增 `GravFieldLike`(sample_g_field 三线性矢量采样 + add_mass + g_const)与 `GravField<T>` 子系统:持有质量密度 `rho`(ScalarField)、工作势 `phi`、引力矢量 `g`(每格 Vec3)、引力常数 `g_const`;`step` 先把 `rho.src` 并入 `u`(对齐 EM 行为)、再 Jacobi 松弛解泊松 `∇²Φ=4πG·ρ`(Neumann 镜像边界,30 次)+ `g=−∇Φ` 中心差分。**符号关键点**:源项取负 `Φ=(Σ邻居 − dx²·4πG·ρ)/6` 使质量处成势阱(Φ<0),于是 `g=−∇Φ` 在质量上方 `∂Φ/∂y>0 ⇒ g.y<0` 指向质量(吸引);若取正则质量处成势峰、`g` 指向外(排斥),与物理相反——M13 初版因此 demo 端到端 FAIL(vy=0),定位为 `grav.step` 漏把 `rho.src` 并入 `u` + 势符号取反两处修复。`phy-rigid` 给 `RigidWorld` 加 `couple_grav(grav,dt,k)`:逐可动体采样 `g_local` 注入 `vel += g_local·dt·k`(下一帧 `step` 生效,与 M11/M12 同约定),并以 `m·‖v‖·dt`(`m=1/inv_mass`)经 `add_mass` 沉积质量实现反向耦合(静态体 `inv_mass=0` 不参与局部加速,其质量由静态天体注入贡献);`RigidSubsystem` 加 `grav_coupling` 字段,`couple` 动态 downcast 找 `GravField`。`Scene::new` 注册 `GravField`(与热/电磁同几何 dx=0.5,中心下方 3×3×3 注入质量源 50 模拟静态天体),`rigid_sub.grav_coupling=1.0`。新增单测 `grav_poisson_yields_attractive_field_toward_mass`/`grav_sample_g_field_trilinear_finite`/`grav_add_mass_accumulates_into_rho`(场)、`couple_grav_attracts_body_toward_mass`/`couple_grav_deposits_mass_from_moving_body`(刚)、端到端 `world_couples_rigid_grav`(World 挂刚/引力,天体下方放置刚体,step 20 帧 vy<0 被吸引、rho.src 沉积>0)。验证: `cargo test --workspace` 全过(软体 11、刚体 18、流体 9、场 8、光学 5、demo 4;dam_break 套件仍 ~107s)。**至此 `phy-field` 三愿景格(热传导 / 电磁 / 引力场)全部落地**;剩余规划外项:World 级事件总线/空间索引、光学焦散、光学↔World 耦合、`phy-io` 轨迹/场导出。 |
| **M14 World 事件总线** | 为 `World` 增加解耦的发布/订阅事件机制,供子系统之间与外部环境交换通知(步进完成、仿真启停、自定义遥测) | `World::step` 自动发 `SimStart`→各 subsystem `step`/`couple`→`Step{t,dt}`,订阅者统一消费 | ✅ `phy-core` 新增 `events.rs`:`EventBus<T>`(订阅者闭包 `FnMut(EventKind,&dyn Any)` + 待分发队列,支持回调内再 `publish` 安全循环 flush)+ 内置 `WorldEvent<T>` 枚举(`SimStart`/`Step{t,dt}`/`SubsystemStepped`/`SimEnd`)+ 自定义 `publish_custom(Box<dyn Any>)`;`World` 持有 `bus` 字段、`subscribe()` 便捷方法,`step` 在第一步前发 `SimStart`、所有 subsystem step/couple 后发 `Step` 并 `flush`。`T` 仅出现在方法签名(`WorldEvent<T>`),用 `PhantomData<fn()->T>` 占位以满足类型参数约束。新增单测 `step_emits_simstart_then_steps`/`custom_event_roundtrips_through_bus`(闭包经 `Rc<RefCell>` 跨 `'static` 借用)。 |
| **M15 World 空间索引** | 为 `World` 增加宽相位邻域查询能力(均匀网格哈希),供流体↔刚体、光学↔世界等"找附近对象"场景复用,把朴素 O(n²) 邻域搜索降到近似 O(n) | `SpatialGrid::neighbors(center,radius)` 返回半径内对象 id | ✅ `phy-core` 新增 `spatial.rs`:`SpatialGrid<T>`(固定 `cell_size` 的均匀网格哈希桶,`build` 批量插入点云,`neighbors` 扫描中心桶 ±span 共 27 桶并按欧氏距离二次过滤;`key_of` 用 `ToPrimitive` 把世界坐标映射到整数桶号)。`pub use spatial::SpatialGrid`。新增单测 `neighbors_finds_nearby_points_only`(原点近邻含自身、不含 100 远点)/`grid_partitions_into_buckets`(三点分三桶)。 |
| **M16 光学↔World(刚体)耦合** | 让光学子系统能跟随刚体世界里的运动物体(如掉落玻璃球),使光线对每个时间步的最新刚体位姿正确求交 | 步进后光学体位置 == 对应刚体位置,运动玻璃球被光折射 | ✅ `phy-optics` 给 `OpticBody` 加 `source_rigid_idx: Option<usize>` + `from_rigid`/`sync_from_rigid`(把对应刚体最新 `Body` 搬入自身);`OpticSubsystem` 加 `optic_coupling: T` 字段与 `couple` 实现:动态 downcast 找 `RigidSubsystem`,对每个 `source_rigid_idx` 命中的光学体经 `world.remove`/`insert` 安全取可变引用同步位姿;`optic_coupling<=0` 时跳过(`name`/`step`/`as_any` 仍按基类实现)。`Scene::new` 把刚体世界地面 + 三个掉落小球镜像为光学体(小球 `from_rigid` 映射刚体索引 1/2/3、`optic_coupling=1.0`、`Precision::Offline`),物理驱动的光学场景成型。新增 demo 端到端 `world_couples_optic_rigid`(step 30 帧后跌落小球光学体 y 减小、且光学体位置与刚体逐分量完全一致;注:演示中央引力井会把小球横向吸引,x 也会变,故测试只断言 y 下落趋势 + 位置一致性,不约束 x)。`phy-io` 顺带加 `num-traits` 依赖与导出 API 见 M17。 |
| **M17 光学焦散渲染** | 在光学后端增加焦散(caustics)计算:平行光经透明体折射后在接收面聚拢形成的亮纹,离线正向光线近似 | `Caustics::accumulate` 返回接收面强度网格(亮度峰值显著高于均值) | ✅ `phy-optics` 新增 `caustics.rs`:`Caustics` 在接收面(`plane_y`)上方均匀采样平行光射线(方向 = `-light_dir`),逐条 `march` 折射行进——命中透明体按 Fresnel 透射率 `(1-Fresnel)` 衰减能量并切换介质 IOR 继续,命中不透明接收面则把剩余通量累加到对应网格单元(全内反射/TIR 则该路不通量落到接收面);返回 `(grid, max_val)`。含 `#[cfg(feature="io_csv")]` 的 `write_caustics_csv` 供后续导出。新增单测 `caustics_concentrates_light_under_glass`(玻璃球 + 下方不透明地面,平行光自上而下,存在非零亮斑且 `max > mean*1.5` 证明光线被聚拢)。 |
| **M18 刚体关节约束(Constraints)** | 把若干刚体连成链条/摆/机械结构的等式约束,复用现有顺序冲量求解器 | `Distance` 杆长收敛到目标、`Ball` 把动态体钉到静态锚点 | ✅ `phy-rigid` 新增 `joint.rs`:`Joint<T>` 枚举(`Ball` 球窝锚点重合 / `Distance` 定长杆)+ `JointConstraint<T>`(累积冲量 `lambda`);`solve_joints_velocity`(与 `solver.rs` 接触求解同构的顺序冲量累积冲量版,消除沿约束误差轴的相对速度,`lambda` 钳到 ≥0 作单边约束)+ `solve_joints_position`(split-impulse 伪速度投影,把残余距离误差按 `beta/dt` 目标消除,迭代至误差 < 1e-4 提前退出,不污染真实速度);`RigidWorld` 加 `joints: Vec<JointConstraint<T>>` 字段与 `add_joint(a,b,joint)`;`step` 在接触速度求解后插 `solve_joints_velocity`、在接触位置修正后插 `solve_joints_position`(复用同 `pseudo` 数组),使关节与碰撞共存。`pub use joint::{Joint, JointConstraint}`。新增单测 `distance_joint_keeps_rest_length_and_conserves_momentum`(两动态体杆长 2→收敛到 2、总动量守恒)+ `ball_joint_pins_body_to_static_anchor`(球窝挂静态锚点,600 帧后锚点不动、摆动体顶部 ≈ 锚点);demo 端到端 `world_drives_rigid_distance_joint`(经 `World` 驱动、downcast 取内部 `RigidWorld` 注关节,300 帧后间距收敛到 1.5)。验证: `cargo test --workspace` 全过(软体 11、刚体 22、流体 9、场 8、光学 6、demo 7;dam_break ~106s)。 |

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

## 5.5 规划外候选物理模型(路线图)

> 截至 M17 + `phy-io`,内核已落地:刚体(含碰撞/约束求解)、SPH 流体、软体(弹簧晶格)、
> 连续场(热/电磁/引力)、光学(折射/反射/焦散)、World 事件总线(M14)、空间索引(M15)。
> 以下为**尚未实现、但能自然挂入现有 `Subsystem` + `couple` 框架**的物理模型候选,
> 按"与现有架构契合度 + 用户最可能想要"排序,经与用户确认作为下一步路线图逐步实施。

### 5.5.1 优先级排序(已与用户确认)

| 序 | 模型 | 落地位置 | 与现有架构关系 | 实现成本 | 收益 |
|---|---|---|---|---|---|
| 1 | **约束关节(Constraints / Articulations)** | `phy-rigid` 扩展 | 复用现有顺序冲量求解器,每条关节 = 一组等式约束迭代求解 | 中 | Demo 立刻能挂钟摆/链条/机械臂,视觉冲击强 |
| 2 | **带电粒子在电磁场中运动(洛伦兹力)** | `phy-rigid`×`phy-field` | 与 M13 引力耦合**完全对称**:`F=q(E+v×b)`,沉积电荷走 `rho.src` 同机制 | 低 | 实现成本最低,且与 M12 EM 场天然闭环 |
| 3 | **布料/绳索(位置动力学 PBD)** | 新 crate `phy-cloth` | 距离/弯曲约束 + 投影碰撞,复用 M15 `SpatialGrid` 做邻域 | 中 | 游戏向最经典缺口(旗帜/衣服) |
| 4 | **扩散-对流场 + 浮力闭环(Boussinesq)** | `phy-field`×`phy-fluid` | 在 M7 热扩散上加平流项 `∇·(v u)`,与 M4e 流体速度场耦合做烟羽/烟囱效应 | 中 | 把热/流体/浮力真正串成自然对流 |
| 5 | **弹性/塑性连续介质(FEM 小变形)** | 新 crate `phy-solid` | 线性四面体 FEM,正确表现梁/板弯曲刚度、泊松比、屈服 | 高 | 比弹簧软体更物理正确 |
| 6 | **声波 / 压力波场** | `phy-field` 加 `AcousticField` | 与现有 `WaveField`(电磁波)同构,仅相速度不同;复用焦散式网格可视化 | 低 | 声学传播/多普勒/遮挡衰减 |
| 7 | **颗粒介质(PBD/DEM)** | 新 crate `phy-granular` | 大量小球接触求解,复用刚体窄相位(GJK/SAT) | 中 | 沙子/谷物 |
| 8 | **破碎/碎屑(Voronoi fracture)** | `phy-rigid` 扩展 | 刚体被打碎成凸碎片,复用碰撞/求解器,碎片初速来自冲量 | 中 | 破坏效果 |
| 9 | **车辆/轮子(raycast vehicle)** | `phy-rigid` 扩展 | 悬挂射线 + 轮胎摩擦,纯约束实现 | 中 | 游戏常用 |
| 10 | **刚体↔流体双向耦合增强(浮力/阻力/涡)** | `phy-rigid`×`phy-fluid` | 流体局部速度/密度采样到刚体(阻力 = ½ρv²C_dA),刚体反推体积入流体源项;复用 M15 空间索引 | 中 | 闭环 M4b 的浮力/阻力 |
| 11 | **时间缩放/子步长/变步长控制器** | `phy-core` | 固定 dt → 自适应子步 + 误差估计,保证刚性场景稳定 | 低 | 数值稳定性基础设施 |
| 12 | **统计/热力学观测器** | `phy-core`/`phy-io` | 对子系统导出动能/势能/温度/熵,配合 M14 事件总线每步发统计做守恒性回归 | 低 | 科研可复现性 |

### 5.5.2 逐步实施计划(本路线图)

- **批次 A(低成本高契合,先做)**: #2 洛伦兹力 → #1 约束关节 → #11 变步长 → #6 声波场。
- **批次 B(游戏向经典)**: #3 布料 PBD → #9 车辆 → #8 破碎。
- **批次 C(科研向深度)**: #4 扩散-对流闭环 → #5 FEM → #7 颗粒 → #10 流体增强 → #12 统计观测器。

每项落地时沿用既有约定:泛型 `RealField`、trait 解耦规避循环依赖(M4e/M16 先例)、
`couple` 动态 downcast 查找依赖子系统(不硬编码下标)、新增子系统即写单测 + 端到端 `World` 测试、
里程碑写入 §6 决策日志。

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
- 2026-08-08: M9 完成。把 M4e 热流体耦合接入 `phy-demo` 实时渲染。`DemoMode` 加 `FluidHeat`(接在 `Optics` 之后循环回 `Rigid`,`next`/`name` 同步);`Scene::new` 即给默认流体子系统设 `thermal_expansion=0.5`/`heat_gain=0.1`,故默认场景(本就注册流体+热)立即演示自然对流,无需额外操作;`Scene::fluid_heat(warm_top)` 新增构造器(暖顶冷底专注对流场景),`set_mode(FluidHeat)`/`reset` 据当前模式重建(切到/离开 `FluidHeat` 都重建,避免场景内容错位),`render` 对 `FluidHeat` 调新 `render_fluid_heat`(先中间层热场切片蓝→红色阶,再叠蓝点流体粒子,二者均**动态 downcast 查流体/热场下标**不依赖注册顺序——与 M4c/d/e 一致,规避此前硬编码下标的坑)。`main.rs` 绑定 `q` 键 `set_mode(FluidHeat)`(`o` 仍循环)。验证: `cargo build -p phy-demo` 仅预存警告(`raster.rs:90` 冗余 `mut`/`corrupt .drectve`),`cargo test --workspace` 全过(43 测试 0 失败)。注:`Scene::fluid_heat` 同时被 `world_couples_fluid_heat_*` 两个端到端测试复用。
- 2026-08-08: M10 完成。全耦合综合 Demo——刚/流/软/热四子系统同存于一个 `World` 且同帧渲染。`DemoMode` 加 `All`(循环接在 `FluidHeat` 后回 `Rigid`,`name` 返回 "All (M10)")。`All` 直接复用默认 `Scene::new`(该场景本就 `add_subsystem` 注册了刚/流/软/热,且流体子系统已开 `thermal_expansion`/`heat_gain`,故 M4b 刚↔流、M4d 软↔流、M4e 流↔热三套耦合在 `World::step` 里一次性全驱动),`set_mode`/`reset` 对 `All` 保持 `Scene::new`(与 `FluidHeat` 专属重建分支并列,互不踩)。新 `render_all` 由远及近画 热场切片(蓝→红)→软体弹簧(灰白)→刚体实体→流体粒子(蓝),**四个下标全部动态 downcast 查找**(与 M4c/d/e 一致,规避硬编码)。`main.rs` 绑 `a` 键 `set_mode(All)`,启动横幅加 "Q 流体+热 / A 全耦合"。验证: `cargo build -p phy-demo` 仅预存警告;`cargo test --workspace` 全过(43 测试 0 失败,含 dam_break ~108s)。这是把分散的 M4 各向耦合收敛到单一可玩场景的收尾里程碑。
- 2026-08-08: M11 完成。四向耦合矩阵补全(刚↔热 / 软↔热),闭环 刚↔流(M4b)/ 软↔流(M4d)/ 流↔热(M4e)/ **刚↔热 / 软↔热** 全部五条边。`phy-field` 新增自由函数 `world_to_cell`(世界坐标→网格下标+三线性偏移,钳到边界)+ `sample_world`(经 `HeatFieldLike::sample_trilinear` 采样),与 M4e 的 trait 解耦复用同一 `HeatFieldLike`。`phy-rigid`/`phy-soft` 各加 `phy-field` 依赖并实现 `couple_heat`;热浮力公式 `a=−g·(1−ρT)` 用 `−g` 因重力向量 y 分量本就为负(向上为正),与 M4e 流体 `−g·(ρf−ρT)/ρf` 同方向;注入热源沿用 M4e 的 `add_source`(需 `step_diffusion` 才并入 `u`,避免凭空升温)。`RigidSubsystem`/`SoftSubsystem` 补全 `thermal_expansion`/`heat_gain`/`t_ref` 字段,`Scene::new` 默认给刚/软开 `0.5`/`0.1`(与流体对齐)。**关键修复**:M4e 流体子系统把 `t_ref` 错误地从热场中心格采样——当耦合物体正好落在最热格时 `temp==t_ref` 使浮力恰好归零(端到端 `world_couples_rigid_soft_heat` 当初因此 vy 比纯重力还负而 FAIL)。M11 改为子系统显式配置 `t_ref`(默认 `T::zero()`,即环境温度基线 `ρ(T_ref)=ρ0`),物理正确且与单测 `t_ref=0` 一致。验证: `cargo test --workspace` 全过(软体 11、刚体 15、流体 9、场 5、光学 5、demo 3;dam_break 套件仍 ~104s)。至此刚/流/软/热四子系统在单 `World` 内任意两两可耦合。
- 2026-08-08: M12 完成。**电磁场子系统 + 刚体↔电磁双向耦合**,补完 `phy-field` 愿景里"热传导、电磁、引力场"三格中的"电磁"。抽取 `GridGeometry<T>` supertrait(dims/origin/cell_size)作 `HeatFieldLike`/`EmFieldLike` 共同父,`world_to_cell`/`sample_world` 改为对 `GridGeometry` 泛型——热场与电磁场彻底共用同一套世界坐标↔网格采样辅助,零重复代码。`EmField<T>` 子系统:电荷密度 `rho`(ScalarField)+ 工作电势 `phi` + 每格矢量 `e` + 外加均匀 `b_ext`;`step` 用 Jacobi 松弛(30 次,Neumann 镜像边界)解泊松 `∇²φ=−ρ/ε` 再 `E=−∇φ` 中心差分,电场随电荷实时演化。`phy-rigid` 给 `RigidWorld` 加并行 `charges: Vec<T>`(不污染 `Body` 公共结构,规避跨 crate 大量 struct-literal 编译破坏)+ `add_charged_body(b,q)`;`couple_em` 施洛伦兹力 `vel += (q(E+v×b_ext)/m)·dt·k`(下一帧 `step` 生效,与 M11 热浮力/`coupled`步进约定一致),并以 `q·‖v‖·dt` 经 `add_charge` 把运动电荷沉积进网格(反向耦合:电荷→电场)。`RigidSubsystem` 加 `em_coupling` 字段,`couple` 动态 downcast 找 `EmField`。`Scene::new` 注册 `EmField`(与热场同几何 dx=0.5,`b_ext=+0.5z`),掉落小球改 `add_charged_body`(中间 +5、两侧 −5 相反电荷反向偏转),`em_coupling=1.0`。**关键约束**:电荷沉积走 `rho.src` 源缓冲、`EmField::step` 才注入 `u`,断言必须用 `rho.src`(与 M11 `HeatField::add_source` 同机制);测试中踩到一次误断言 `u` 导致 FAIL,已改判 `rho.src`。**回归彩蛋**:把 `HeatField` 的几何访问从固有方法搬进 `GridGeometry` trait 后,`phy-fluid` 的 `heat.dims()/hf.dims()` 需在 `sph.rs`/`subsystem.rs` 补 `use phy_field::GridGeometry;` 否则 trait 方法不可见(E0599)——已补。**验证**: `cargo test --workspace` 全过(软体 11、刚体 18、流体 9、场 8、光学 5、demo 4;dam_break ~106s)。至此 `phy-field` 三愿景格已落地热传导+电磁(引力场仍规划外)。
- 2026-08-08: M13 完成。**引力场子系统 + 刚体↔引力双向耦合**,落地 `phy-field` 愿景三格的最后一格(引力场)。复用 M12 的 `GridGeometry` + Jacobi 泊松基础设施,新增 `GravFieldLike`(sample_g_field 三线性矢量采样 + add_mass + g_const)与 `GravField<T>` 子系统:持有质量密度 `rho`(ScalarField)+ 工作势 `phi` + 每格引力矢量 `g` + 引力常数 `g_const`;`step` 先把 `rho.src` 并入 `u`(对齐 EM 行为,**初版漏此步导致 demo 端到端 vy=0 FAIL**)、再 Jacobi 松弛解牛顿引力势泊松 `∇²Φ=4πG·ρ`(Neumann 镜像边界,30 次)+ `g=−∇Φ` 中心差分。**符号关键**:松弛源项取负 `Φ=(Σ邻居 − dx²·4πG·ρ)/6` 使质量处成势阱(Φ<0),于是 `g=−∇Φ` 在质量上方 `∂Φ/∂y>0 ⇒ g.y<0` 指向质量(向内吸引);若取正则质量处成势峰、`g` 指向外(排斥),与物理相反——这是 M13 初版 demo FAIL 的第二处根因。**耦合**:`RigidWorld::couple_grav(grav,dt,k)` 逐可动体采样 `g_local` 注入 `vel += g_local·dt·k`(下次 `step` 生效,与 M11/M12 步进约定一致)、并以 `m·‖v‖·dt`(`m=1/inv_mass`)经 `add_mass` 沉积质量实现反向耦合(静态体 `inv_mass=0` 不参与局部加速,其质量由静态天体注入贡献);`RigidSubsystem` 加 `grav_coupling` 字段,`couple` 动态 downcast 找 `GravField` 经 `remove`/`insert` 驱动。`Scene::new` 注册 `GravField`(与热/电磁同几何 dx=0.5,中心下方 3×3×3 注入质量源 50 模拟静态天体),`rigid_sub.grav_coupling=1.0`。新增单测 `grav_poisson_yields_attractive_field_toward_mass`/`grav_sample_g_field_trilinear_finite`/`grav_add_mass_accumulates_into_rho`(场)、`couple_grav_attracts_body_toward_mass`/`couple_grav_deposits_mass_from_moving_body`(刚)、端到端 `world_couples_rigid_grav`(天体下方刚体 step 20 帧 vy<0 被吸引、rho.src 沉积>0)。**验证**: `cargo test --workspace` 全过(软体 11、刚体 18、流体 9、场 8、光学 5、demo 4;dam_break ~107s)。至此 `phy-field` 三愿景格(热传导/电磁/引力场)全部落地;剩余规划外项:World 级事件总线/空间索引、光学焦散、光学↔World 耦合、`phy-io` 轨迹/场导出。
- 2026-08-08: M14 完成。**World 事件总线**。`phy-core` 新增 `events.rs`:`EventBus<T>`(订阅者闭包 `FnMut(EventKind,&dyn Any)` + 待分发队列,回调内再 `publish` 也能安全循环 flush 避免重入)+ 内置 `WorldEvent<T>` 枚举(`SimStart`/`Step{t,dt}`/`SubsystemStepped`/`SimEnd`)+ 自定义 `publish_custom(Box<dyn Any>)`;`World` 持有 `bus` 字段、`subscribe()` 便捷方法,`step` 在首步前发 `SimStart`、所有 subsystem step/couple 之后发 `Step{t,dt}` 并 `flush`。**约束**:`T` 仅出现在方法签名(`WorldEvent<T>`),struct 字段不沾 `T`,需用 `PhantomData<fn()->T>` 占位否则 E0392(类型参数从未在字段使用);`step` 里 `self.t += dt` 会 move `dt`,后续 `WorldEvent::Step{t: self.t.clone(), dt}` 必须 `self.t += dt.clone()` 否则 E0382(第二次用已 move 的 dt)。新增单测 `step_emits_simstart_then_steps`/`custom_event_roundtrips_through_bus`(闭包须 `'static`,用 `Rc<RefCell>` 跨闭包借用,不能用局部 `&mut`)。
- 2026-08-08: M15 完成。**World 空间索引**(宽相位邻域查询)。`phy-core` 新增 `spatial.rs`:`SpatialGrid<T>`,固定 `cell_size` 均匀网格哈希桶,`build` 批量插入点云,`neighbors(center,radius)` 扫描中心桶 ±span(跨 27 桶)并按欧氏距离二次过滤;`key_of` 用 `ToPrimitive` 把世界坐标映射到整数桶号(impl 需 `T: RealField + Copy + ToPrimitive`)。`pub use spatial::SpatialGrid`。新增单测 `neighbors_finds_nearby_points_only`/`grid_partitions_into_buckets`。为 `phy-core` 加 `num-traits` 依赖。
- 2026-08-08: M16 完成。**光学↔World(刚体)耦合**。让光学子系统跟随刚体世界里的运动物体(掉落玻璃球),光线对每个时间步最新刚体位姿正确求交。`phy-optics` 给 `OpticBody` 加 `source_rigid_idx: Option<usize>` + `from_rigid`/`sync_from_rigid`(把对应刚体最新 `Body` 搬入自身);`OpticSubsystem` 加 `optic_coupling: T` 字段与 `couple` 实现——动态 downcast 找 `RigidSubsystem`,对每个 `source_rigid_idx` 命中的光学体经 `world.remove`/`insert` 安全取可变引用同步位姿(`optic_coupling<=0` 跳过;`name`/`step`/`as_any` 仍按基类实现)。**约束**:`OpticSubsystem` 的 struct impl 与 `impl Subsystem` 都需 `T: RealField + Copy + ToPrimitive`(因为 `couple` 里用到 `T::from_usize` 之类需 `ToPrimitive` 的算子隐含在 `OpticBody` 同步链路),否则 E0277。`Scene::new` 把刚体世界地面 + 三个掉落小球镜像为光学体(`Precision::Offline`,小球 `from_rigid` 映射刚体索引 1/2/3、`optic_coupling=1.0`),物理驱动的光学场景成型。新增 demo 端到端 `world_couples_optic_rigid`(step 30 帧后跌落小球光学体 y 减小、且光学体位置与刚体逐分量一致;演示中央引力井横向吸引小球,故只断言 y 下落趋势 + 位置一致性,不约束 x)。
- 2026-08-08: M17 完成。**光学焦散(caustics)渲染**。`phy-optics` 新增 `caustics.rs`:`Caustics::accumulate(scene, light_dir, plane_y, half_extent, grid_n)` 在接收面上方均匀采样平行光射线(方向 = `-light_dir`),逐条 `march` 折射行进——命中透明体按 Fresnel 透射率 `(1-Fresnel)` 衰减能量并切换介质 IOR 继续,命中不透明接收面则把剩余通量累加到对应网格单元(全内反射则该路能量不落到接收面);返回 `(grid, max_val)`。含 `#[cfg(feature="io_csv")]` 的 `write_caustics_csv` 供后续 `phy-io` 对接导出。新增单测 `caustics_concentrates_light_under_glass`(玻璃球 + 下方不透明地面,平行光自上而下,存在非零亮斑且 `max > mean*1.5` 证明光线被聚拢成亮斑)。**验证**: `cargo test --workspace` 全过(软体 11、刚体 18、流体 9、场 8、光学 6、demo 6;dam_break ~108s)。原规划"剩余项"(M14/M15/M16/M17)+ phy-io 导出模块已全部实现;随后从 §5.5 路线图进入新增物理模型阶段,首项 M18 见下条。
- 2026-08-08: M(phy-io) 完成。**`phy-io` 轨迹/场 CSV 导出落地**。此前 `phy-io` 为空壳 crate(M 前序规划里列为"场景描述解析、轨迹/场导出")。现实现三类导出:`csv.rs`(`CsvWriter` 轻量 CSV 写出)。`trajectory.rs`(`BodySample<T>` 从 `RigidWorld` 抓取位姿/速度/四元数快照,`write_trajectory` 写多帧轨迹)、`field_slice.rs`(`write_slice`/`write_center_slice` 把三维标量场切成 XY/XZ/YZ 二维 CSV 网格,附带每行世界坐标)。为 `phy-io` 加 `phy-rigid`/`phy-field`/`num-traits` 依赖;`T` 写出需 `Display` + `FromPrimitive`(列 id/行号用 `T::from_usize`)。新增单测 `trajectory_csv_has_header_and_one_row_per_body`/`field_slice_csv_has_grid_rows`(写到 `std::env::temp_dir` 后清理,避开 `AsRef<Path>` 与 `Cursor` 的类型错位)。至此 `phy-io` 从空壳变为可用导出后端,与 M14/M15/M16/M17 一起补齐 World 内核的工具链闭环。
- 2026-08-08: M18 完成。**刚体关节约束(Constraints)**。`phy-rigid` 新增 `joint.rs`:`Joint<T>` 枚举(`Ball` 球窝=两局部锚点世界位置重合,3 自由度转动放开;`Distance` 定长杆=两锚点世界距离保持 `rest`)统一经 `world_anchors`/`error` 计算约束轴与带符号误差;`JointConstraint<T>` 持 `lambda`(累积冲量)。两个求解函数均与 `solver.rs` 接触求解**同构**的顺序冲量法:`solve_joints_velocity`(消除沿约束轴的相对速度,`lambda` 钳到 ≥0 作单边约束)+ `solve_joints_position`(split-impulse 伪速度投影,目标相对伪速度 = `beta/dt · err`,迭代至最大残余误差 < 1e-4 提前退出,不污染真实速度);`RigidWorld` 加 `joints: Vec<JointConstraint<T>>` 字段与 `add_joint(a,b,joint)`,`step` 在接触速度求解后插关节速度求解、在接触位置修正后插关节位置求解(复用同一 `pseudo` 数组),使关节与碰撞共存不乱。`pub use joint::{Joint, JointConstraint}`。新增单测 `distance_joint_keeps_rest_length_and_conserves_momentum`(两动态体杆长 2→收敛到 2、关重力下总动量守恒)、`ball_joint_pins_body_to_static_anchor`(球窝挂静态锚点,600 帧后锚点不动、摆动体顶部 ≈ 锚点);demo 端到端 `world_drives_rigid_distance_joint`(**注:§5.5 候选 #2 洛伦兹力实为 M12 已落地能力,非新增**,故优先做 #1 约束关节)。**验证**: `cargo test --workspace` 全过(软体 11、刚体 22、流体 9、场 8、光学 6、demo 7;dam_break ~106s)。下一步按 §5.5.2 批次 A 续做 #11 变步长 / #6 声波场,或按需跳到批次 B 布料 PBD。
