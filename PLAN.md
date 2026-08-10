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
| **M19 路线图批处理(变步长 / 声波场 / 布料 PBD / 统计观测器)** | 按 §5.5 优先级推进低成本高契合候选:#11 变步长控制器、`#6` 声波场、`#3` 布料 PBD、`#12` 统计观测器 | 帧 dt 自动切片稳定子步、声压波随声速传播、钉扎布料下垂不断裂、每步统计可观测 | ✅ ① `phy-core` 新增 `timestep.rs`:`TimeController<T>`(`Fixed` 固定子步 + `Adaptive` 自适应子步,`max_dt` 上界保证子步绝不超稳界;`accumulator` 尾数跨帧累积,`time_scale` 慢动作/快进,`flush_accumulator` 防暂停补帧爆冲;`advance(world, frame_dt)` 驱动 `world.step`)。② `phy-field` 新增 `acoustic.rs`:`AcousticField<T>`(`ScalarField` + 声速平方 `c2` + 介质密度 `ρ0`,默认 `SOUND_SPEED_AIR=343`;复用 `step_wave` 解声压波动方程,`add_pressure_source` 压力注入,实现 `GridGeometry`/`Subsystem` 可挂 `World`)。③ `phy-soft` 新增 `cloth.rs`:`Cloth<T>`(nx×ny 质点网格 + `DistanceConstraint` 结构/剪切/可选弯曲约束;PBD 预测-投影-速度回写,Gauss-Seiden 迭代 `iterations` 次;`pin` 钉扎、`ground_y` 地面碰撞),实现 `Subsystem`。④ `phy-core` 新增 `stats.rs`:`StatsObserver<T>`(累加步数/累计仿真时间/最近步长/时钟单调异常 `clock_anomalies`/自定义遥测均值-最值)+ `attach_stats_observer`(经 `Rc<RefCell>` 订阅 M14 事件总线随 `World::step` 自动更新)。新增单测:core `timestep` 5 例(切片/尾数跨帧/自适应末步/缩放/flush)、`stats` 3 例(累加/时钟异常/遥测);field `acoustic` 2 例(脉冲传播 + World 运行);soft `cloth` 2 例(钉扎下垂保连通约束 + World 子系统)。**验证**: `cargo test --workspace` 全过(核心 12、demo 7、场 13、刚体 22、软体 13、流体 9、光学 6、io 2、math 22)。 |
| **M20 射线投射车辆(raycast vehicle)** | 路线图 #9:游戏向经典缺口。车身作为普通刚体,每个车轮从悬挂顶端向下打射线找地面,施加悬挂弹簧-阻尼力 + 轮胎纵向(引擎/刹车)/横向(转向)摩擦 | 车身被悬挂托在地面上方不坠落、引擎驱动前进、刹车减速 | ✅ `phy-rigid` 新增 `raycast.rs`:`ray_cast(origin, dir, body)` 射线 vs `Sphere`(闭式)/`Box`(局部空间 slab 求交,法线按进入轴 + ray 局部方向符号)/`Convex`(三角形 Möller–Trumbore 取最近),返回 `RayHit{t,point,normal}`;4 单测(sphere 命中/未中、box 顶面命中/侧面未中)。新增 `vehicle.rs`:`Wheel<T>`(局部锚点 + 悬挂 rest/k/c + 轮半径 + 纵向 traction/横向 grip + 运行时 compression/grounded)+ `Vehicle<T>`(chassis 刚体索引 + 车轮列表 + engine/steering/brake 输入 + forward_axis),`update(world, dt)` 对每个轮经 `ray_cast_ground`(仅对静态 `inv_mass==0` 物体打射线取最近)算悬挂压缩 → 沿地面法线施弹簧-阻尼力,并把引擎/刹车(乘以质量使减速与质量无关)/转向(自行车模型横向摩擦)投影到地面切平面,作为线性力注入车身 `vel`(与 `couple_*` 同款,须在 `world.step` 前调用)。新增 3 单测(悬挂托举稳定 / 引擎前进 / 刹车减速)+ demo 集成 `world_drives_raycast_vehicle`(经 `World` + `RigidSubsystem` 驱动生效)。**决策**:本引擎刚体未建模角速度,故车辆姿态由四角对称悬挂自然维持直立、轮胎力作用于质心,未引入角动力学,保持最小侵入;`raycast` 作为独立能力也服务于后续 #8 破碎碎片射线检测等。**验证**: `cargo test --workspace` 全过(刚体 29 + demo 8 含车辆集成)。 |
| **M21 Voronoi 破碎(fracture)** | 路线图 #8:游戏向经典缺口。把一个凸刚体按 Voronoi 图切成 N 个凸碎片,碎片继承母本线速度 + 径向飞散,质量按体积比守恒 | 盒碎成多块凸碎片、总质量守恒、步进稳定无 NaN | ✅ `phy-rigid` 新增 `fracture.rs`:`fracture_body(body, n, seeds, radial)`(母本 → N 个 `Shape::Convex` 碎片刚体,默认在包围盒内均匀网格 + 内边距 jitter 撒种子)+ `fracture_convex(vertices, faces, seeds)`(逐种子用"半空间裁剪 + 封盖"做 3D 凸多面体 Voronoi 细胞切分)+ `convex_volume_centroid`(散度定理算体积/质心)。`RigidWorld::shatter(body_id, n, radial)`(在独立 `impl<T: NumCast>` 块,避免污染主 impl 的 `RealField+Copy` 边界):`swap_remove` 母本并追加碎片,电荷按质量比分配;`RigidWorld<T>` 主 impl 边界保持不变以免级联波及 `subsystem.rs`)。4 单测:`fracture_box_yields_n_fragments`/`fragment_mass_sums_to_parent`(质量守恒<5%)/`fracture_adds_radial_velocity`/`fracture_convex_preserves_volume_sum`(碎片体积和≈母体积);`world.rs` 新增 `shatter_box_produces_fragments_conserving_mass`/`shattered_fragments_step_stably`(轻量化:6 碎片/60 帧验证无 NaN,原 12 碎片/300 帧因凸-凸碰撞 O(n²) 耗时 122s 故下调);`phy-demo` 新增端到端 `world_shatters_box_into_fragments`(经 `World`+`RigidSubsystem` 驱动生效)。**决策**:沿用 M20 的"最小侵入"原则——本引擎刚体未建模角速度,碎片只注入线速度(继承母本 + 沿碎片-母体质心方向径向飞散),不引入角动力学;`fracture_body` 不修改 `RigidWorld`(只返回新 `Body` 列表),避免热路径耦合,由 `shatter` 负责加入世界。`lib.rs` 注册 `pub mod fracture` 与 `pub use fracture::{fracture_body, fracture_convex, convex_volume_centroid}`。**关键陷阱**:① 凸多面体半空间裁剪必须对**三角面**做(Sutherland–Hodgman 3D 网格裁剪 + 切口封盖扇化),把顶点列表当多边形环裁剪会因顶点不按边界排序而得到错误细胞(体积只剩 1/8);② 封盖正交基必须右手系 `w = n × u` 使 CCW 扇化外法指向 +nrm,否则封盖法线反转导致体积散度积分部分抵消(测得 5.875 vs 理论 4);③ `scatter_seeds` 的种子必须严格在物体内(用 `(i+1)/(per+1)` 内边距映射 + 内边距随机点),落在物体表面会切出零体积薄片被丢弃、碎片数不达标;④ 确定性哈希 `rand01` 必须 `(s>>8) & 0xFFFFFF / 2^24` 归一化到 [0,1),初版漏了掩码导致种子坐标爆成 1e8、整盒未被切分。**验证**: `cargo test -p phy-rigid` 全过(35 测试含 6 新增 fracture/shatter),`cargo test -p phy-demo world_shatters_box_into_fragments` 通过;`cargo test --workspace` 全过(刚体 35、demo 8)。 |
| **M27 Demo 场类可视化 + M26 存档读档接入** | 把 M26 完成的 serde 存档能力接到实时 Demo,并补齐此前缺场的"场类"物理可视化(此前 Demo 只画了热场,电磁/引力/波动/声场无图形展示) | 数字键 7/8/9/0 进入 EM/Grav/Wave/Acoustic 模式看切片+矢量箭头;F5 存档 JSON、F9 读档回放 | ✅ **(A) 场类可视化**:`DemoMode` 新增 `Em`/`Grav`/`Wave`/`Acoustic`(共 10 模式,`next` 循环、`name` 补中文标签)。新增 `Scene::em()`/`grav()`/`wave()`/`acoustic()` 四个独立单场场景构造器,各用 `ScalarField::with_origin` 居中网格 + 中心脉冲/偶极源,挂入干净 `World`。新增渲染:`render_scalar_slice`(中心切片按标量值蓝↔红色阶,`max_abs` 归一)与 `render_vector_arrows`(按 stride 抽稀、长度按 `vmax` 归一、强度上色黄→红、投影画线段+箭头端圆);`render_em` 画电荷密度切片+电场箭头、`render_grav` 画质量密度切片+引力箭头(指向天体)、`render_wave`/`render_acoustic` 画标量位移/声压切片。`set_mode`/`reset` 对四个单场模式走专属 `Scene::em/grav/wave/acoustic` 重建分支(与 `FluidHeat` 专属分支并列)。**(B) 存档读档**:`phy-demo` 加 `phy-io` 依赖;`main.rs` 绑定 `F5`→`save_world(&world, Path::new(SAVE_PATH))`、`F9`→`load_world(Path::new(SAVE_PATH))` 回填 `scene.world`(默认路径 `world_save.json`);`F9` 不依赖当前模式,可跨模式读回任意存档。`main()` 横幅更新键位(1-0 模式 + F5/F9)。新增单测 `demo_single_field_scenes_build_and_step`:四个单场场景均 construct+step 不 panic、电场/引力矢量由场源解出且非全零、标量场有限。**验证**: `cargo build -p phy-demo` 干净(仅预存 `unused_mut` 警告);`cargo test --workspace` 全过(0 failed,新增 demo 测试纳入)。 |
| **M27.1 Demo 可自测化 + Web 页面** | 用户反馈桌面 Demo 切到非刚体模式后运行异常(根因:`body_count()` 无条件 `unwrap` 取刚体子系统,非刚体模式 panic;且 Optics 渲染只在桌面 App 内、lib/Web 无法复用),并要求"做成 web 页面以便自测"。 | (1) 抽 `phy-demo` 的 `scene/raster/camera` 为库目标(`lib.rs`),桌面 bin 与 Web 版共用;(2) 修 `body_count` 在非刚体模式安全返回 0;(3) Optics 渲染从 App 迁入 `Scene::render`,使 lib/Web 统一可渲染;(4) 新增字符串版存档 API `Scene::save_string/load_string`(用 `phy_io::save_world_json/load_world_json`)供无文件系统的 Web 用;(5) 新增 `phy-demo-web` crate(wasm + wasm-bindgen + web-sys),把 `Scene/Framebuffer` 每帧渲染到 `<canvas>`,键盘切模式/F5 存档到 localStorage/F9 读档,鼠标拖拽旋转、滚轮缩放;(6) 自测机制:`phy-demo/tests/headless.rs`(纯 Rust 离屏 Framebuffer,11 模式断言不 panic 且像素非空)+ `phy-demo-web` 的 `render_offscreen()` + host 集成测试(同一套渲染逻辑断言非空);(7) `serve.py` 零依赖静态服务器,浏览器开 `http://localhost:8000` 预览。 | ✅ **自测结论**:`cargo test -p phy-demo --test headless` 全过(11 模式渲染非空 + `body_count` 安全 + 存档往返);`cargo test -p phy-demo-web` 全过(Web 渲染逻辑 11 模式非空)。**Web 构建**:`wasm-pack build --target web` 成功产出 `pkg/`;`wasm-pack` + `wasm32-unknown-unknown` target 已安装。**已知限制**:本机无无头浏览器(Chromium/Edge/Playwright 均缺),未做像素级截图自测;Web 的"肉眼画面"需用户在浏览器打开 `serve.py` 预览,逻辑层已由 host 测试覆盖。 |
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
| 3 | **布料/绳索(位置动力学 PBD)** | `phy-soft` 加 `Cloth`(未单独开 crate,复用 `Particle`/地面碰撞/`Subsystem`) | 距离/弯曲约束 + 投影碰撞,复用 M15 `SpatialGrid` 做邻域 | 中 | 游戏向最经典缺口(旗帜/衣服) | ✅ M19 |
| 4 | **扩散-对流场 + 浮力闭环(Boussinesq)** | `phy-field`×`phy-fluid` | 在 M7 热扩散上加平流项 `∇·(v u)`,与 M4e 流体速度场耦合做烟羽/烟囱效应 | 中 | 把热/流体/浮力真正串成自然对流 |
| 5 | **弹性/塑性连续介质(FEM 小变形)** | 新 crate `phy-solid` | 线性四面体 FEM,正确表现梁/板弯曲刚度、泊松比、屈服 | 高 | 比弹簧软体更物理正确 |
| 6 | **声波 / 压力波场** | `phy-field` 加 `AcousticField` | 与现有 `WaveField`(电磁波)同构,仅相速度不同;复用焦散式网格可视化 | 低 | 声学传播/多普勒/遮挡衰减 | ✅ M19 |
| 7 | **颗粒介质(PBD/DEM)** | 新 crate `phy-granular` | 大量小球接触求解,复用刚体窄相位(GJK/SAT) | 中 | 沙子/谷物 |
| 8 | **破碎/碎屑(Voronoi fracture)** | `phy-rigid` 扩展 | 刚体被打碎成凸碎片,复用碰撞/求解器,碎片初速来自冲量 | 中 | 破坏效果 |
| 9 | **车辆/轮子(raycast vehicle)** | `phy-rigid` 扩展 | 悬挂射线 + 轮胎摩擦,纯约束实现 | 中 | 游戏常用 |
| 10 | **刚体↔流体双向耦合增强(浮力/阻力/涡)** | `phy-rigid`×`phy-fluid` | 流体局部速度/密度采样到刚体(阻力 = ½ρv²C_dA),刚体反推体积入流体源项;复用 M15 空间索引 | 中 | 闭环 M4b 的浮力/阻力 |
| 11 | **时间缩放/子步长/变步长控制器** | `phy-core` | 固定 dt → 自适应子步 + 误差估计,保证刚性场景稳定 | 低 | 数值稳定性基础设施 | ✅ M19 |
| 12 | **统计/热力学观测器** | `phy-core`/`phy-io` | 对子系统导出动能/势能/温度/熵,配合 M14 事件总线每步发统计做守恒性回归 | 低 | 科研可复现性 | ✅ M19 |

### 5.5.2 逐步实施计划(本路线图)

- **批次 A(低成本高契合,先做)**: #2 洛伦兹力(M12 已落地) → #1 约束关节(M18) → #11 变步长(M19 ✅) → #6 声波场(M19 ✅)。
- **批次 B(游戏向经典)**: #3 布料 PBD(M19 ✅) → #9 车辆(M20 ✅) → #8 破碎(M21 ✅)。
- **批次 C(科研向深度)**: #4 扩散-对流闭环 → #5 FEM → #7 颗粒 → #10 流体增强 → #12 统计观测器(M19 ✅)。

每项落地时沿用既有约定:泛型 `RealField`、trait 解耦规避循环依赖(M4e/M16 先例)、
`couple` 动态 downcast 查找依赖子系统(不硬编码下标)、新增子系统即写单测 + 端到端 `World` 测试、
里程碑写入 §6 决策日志。

---

## 5.6 路线图完成后的补充方向(Backlog)

> §5.5 主干项(#1–#12)已全部落地、M18–M26 闭环。以下为**仍可补强、且能自然挂入现有
> `Subsystem` + `couple` + 存档框架**的后续候选,按"架构缺口优先级 + 收益"排序,后续逐步实施。

| 序 | 方向 | 落地位置 | 与现有架构关系 | 成本 | 收益 |
|---|---|---|---|---|---|
| S1 | **刚体角动力学(旋转)** ✅已完成 | `phy-rigid` 扩展 | 给 `Body` 加角速度/惯性张量/四元数姿态,`step` 积分姿态、`solver` 加接触角冲量;M20 车辆、M21 破碎、M18 关节均受限于"无角速度" | 高 | 真实翻滚、陀螺效应、扭矩驱动关节;多处已交付功能共同天花板 |
| S2 | **不可压 Navier-Stokes / 有限体积 CFD** ✅已完成 | `phy-fluid` 扩展(`crates/phy-fluid/src/cfd.rs`) | 复用 M7 网格 + M4e 平流 + M12/13 Jacobi 压力泊松;`CfdWorld`(MAC 交错网格 + Chorin 投影)替代弱可压 SPH | 高 | 解决 SPH 持续挤压数值发散(M25 已见),更稳的大尺度流场 |
| S3 | **绳索/链(Catmull-Rom 或 XPBD 距离约束链)** ✅已完成 | `phy-soft` 扩展(`rope.rs` 的 `Rope`) | 复用 M19 布料 `DistanceConstraint` + PBD 投影;1D 距离约束链 + 钉扎 | 中 | 锁链、藤蔓、辫子等 1D 柔性体的独立常见需求 |
| S4 | **柔体↔刚体更紧耦合 + 软体↔光学折射** ✅已完成 | `phy-soft`×`phy-optics`/`phy-rigid` | 复用 M4b 软↔刚、M16 光↔刚;`SoftBody`/`Rope` 新增 `proxy_body()` 导出代理球刚体实现折射;`SoftBody` 新增 `anchors`+`set_anchor`/`clear_anchor` 实现软↔刚体挂接(布料挂运动刚体);端到端测试 `soft_body_proxy_refracts_light` / `soft_hangs_from_moving_rigid_body` 验证 | 中 | 透明软体(水袋/果冻)光学折射、软布挂接刚体链 |
| S5 | **气体/烟雾多相与燃烧** | `phy-field`×`phy-fluid` | 复用 M22 Boussinesq 闭环 + M4e 平流;`SmokeField`(烟/气被动标量 + 浓度依赖浮力上浮, S5a ✅) + 燃烧(燃料场 + 温度阈值点燃 + 放热回灌热场 + 火焰核邻格加热形成自持前缘, S5b ✅)均已落地 | 中 | 烟羽/火焰/爆炸等特效,科研向燃烧建模 |
| S6 | **连续碰撞检测(CCD)** ✅已完成 | `phy-rigid` 扩展 | `RigidWorld::step` 重构为位移受限子步化:按最快可动体位移/最小特征尺寸(默认 ≤½)切分 ≤`ccd_max_substeps` 子步,每子步跑离散 `collide`+速度求解,杜绝高速隧穿;`SolverParams.ccd_max_substeps`(默认 8,设 0 退化等价原离散)开关;新增 `ccd.rs`(`substep_count`/`swept_sphere_sphere`/`ccd_contact`)+ 修复 `narrowphase` 缺失的**球-盒解析快速碰撞路径**(此前球撞墙/地板漏检,靠 GJK/EPA 回退有 bug) | 中 | 高速小物体防隧穿、子弹/高速碎片命中薄壁;球-盒碰撞稳健性 |
| S7 | **GPU/并行后端** ✅已完成 | `phy-fluid`/`phy-granular`/`phy-optics` | 因 MinGW 链接 wgpu 崩溃(M3 决策)放弃 GPU,改走 **rayon 并行 CPU 后端**:SPH 密度/压力/受力两遍改为 Jacobi 式并行(`par_iter` + 独立缓冲写回);颗粒 PBD 接触投影由 Gauss-Seidel 改为 **Jacobi**(并行累加 per-body 修正,`reduce` 固定顺序合并,保持确定性);光学 `render_camera` 逐像素 `trace` 经 rayon 并行(`Whitted`/`Approx` 单元结构体 `Sync`,避开 `dyn Renderer` 不可跨线程)。均为无数据竞争的只读遍历→缓冲写回,并行结果与串行逐像素逐一对应 | 中 | 大规模粒子/颗粒/流体与逐像素渲染吞吐;`fill_grid`/流体场大场景实时化,且为 S8 确定性预留(并行不引入顺序依赖) |
| S8 | **数值确定性 / WASM 回放** | 全局 | §5.5 原"暂不做",架构已预留;需固定步长 + 定点/确定性浮点 | 中 | 科研复现、锁帧回放、网络同步 |
| S9 | **多材料 / 非牛顿流体(SPH 增强)** ✅已完成 | `phy-fluid` SPH | `Particle` 加 `material` 标签;`SphParams` 加幂律表 `visc_k`/`visc_n`+`shear_min`;`compute_forces` 据局部应变率算有效粘度 μ_eff=k·max(剪切率,ε)^(n-1)(n<1 剪切变稀,n>1 剪切变稠,n=1 牛顿);`effective_viscosity(i)` 查询 | 中 | 蜂蜜/牙膏/玉米淀粉流体、油水分层等多相流体,攻克 M4e“仅牛顿均质”缺口 |

### 5.6.1 实施建议(后续)

- **优先 S1**:它是 S2–S6 多个方向共同依赖的底层能力(旋转才能让车辆/关节/破碎真正物理正确),
  且能填 M18/M20/M21 已明示的"未建模角速度"缺口,价值最高。
- **其次 S2**:直接解决 M25 暴露的 SPH 数值稳定性问题,复用现有网格/泊松/Jacobi 基础设施,契合度高。
- S3–S6 为锦上添花,可在 S1/S2 落地后按需求择机推进;S7/S8 为基础设施级,成本高风险大,按需评估。

---

## 5.7 GPU 加速规划(GPU Offload Plan)

> 背景:S7 已把第一梯队热点改成 rayon CPU 并行,核心是"只读遍历 → 独立缓冲写回"的
> **Jacobi 式无数据竞争结构**,这正是 GPU compute 的内核形态。本规划评估内核里
> **哪些算法适合 GPU、哪些不适合**,并给出**只做 Web Demo**的落地路径。
> 决策(用户 2026-08-09):**只做 Web Demo,不考虑桌面 Demo**。桌面端维持 rayon CPU 并行,
> 不引入任何 wgpu 桌面依赖(避开 MinGW 链接崩溃,见 M3)。GPU 仅通过 `phy-demo-web`
> 的 WebGPU(wasm32 目标)在浏览器内落地,浏览器不依赖本地链接器。

### 5.7.1 加速 suitability 判据

GPU 最适合:**大量独立小计算 + 只读遍历 + 独立写回**(one-thread-per-element/kernel)。
GPU 最怕:**全局强耦合 / Gauss-Seidel 顺序依赖 / 跨线程频繁同步**(每帧少量对象的接触图)。

### 5.7.2 各算法 suitability 矩阵

| 档位 | 算法 | 位置 | 理由 |
|---|---|---|---|
| **A 非常适合(已是 rayon+Jacobi,可直映射 GPU kernel)** | SPH 密度/压力 | `phy-fluid/src/sph/world.rs::compute_density_pressure`(188) | 每粒子独立读邻居→写自己 rho/p,Jacobi 独立缓冲写回,无竞争 |
| | SPH 受力 | `phy-fluid/src/sph/world.rs::compute_forces`(236) | 同上,逐粒子算 acc/mu_eff;S9 非牛顿幂律也只是局部应变率,纯逐粒子 |
| | 颗粒 PBD 接触投影 | `phy-granular/src/world.rs`(Jacobi par_iter().fold().reduce, 190) | 每对只读 predicted 快照→累加 per-body deltas,固定序 reduce,确定性 |
| | 光学逐像素 trace | `phy-optics/src/subsystem.rs::render_camera`/`render_parallel`(58/84) | 像素间完全独立,经典 one-thread-per-pixel |
| | 焦散 caustics | `phy-optics/src/caustics.rs::accumulate`(25) | 每条平行光射线独立 march→累加权值到接收面网格(atomic add) |
| **B 中等适合(结构并行,需先改 Jacobi / 处理归约)** | 标量场解算(热/波/电磁/引力/声/烟) | `phy-field/*`(`step_diffusion`/`step_wave`/`EmField::step`/`GravField::step`/Jacobi 泊松松弛) | 网格每格 u[i]=f(邻居) 是标准 stencil,但**泊松松弛 30 次迭代**须先确认是 Jacobi(可全并行);Gauss-Seidel 不可并行。边界(Neumann 镜像)按线程序号处理 |
| | 半拉格朗日平流 | `phy-field` `step_advect` | 每格独立回溯+三线性插值,只读采样无写竞争 |
| | FEM 刚度装配 | `phy-solid` | 单元 Ke 装配 per-element 并行;但 Cholesky 线代系统串行依赖,需 cuSPARSE/cuBLAS 或迭代法,装配部分值得 GPU、求解部分收益小 |
| | SPH 邻居网格构建 | `phy-fluid::Grid::build` | 粒子分桶是 prefix-sum/histogram 任务,可 GPU 化以喂给 SPH kernel |
| **C 不适合 / 收益低** | 刚体顺序冲量求解 | `phy-rigid/solver.rs` | 接触图强耦合 + Gauss-Seidel 顺序冲量(M2),全局同步频繁;少量刚体 GPU 调度开销 > 收益 |
| | 关节约束求解 | `phy-rigid/joint.rs` | 同上顺序冲量累积 lambda,链式关节强数据依赖 |
| | Voronoi 破碎 | `phy-rigid/fracture.rs` | 一次性事件 + 凸裁剪串行几何,并行收益低、实现复杂 |
| | CCD 子步化 | `phy-rigid/ccd.rs` | 位移受限子步是串行时间推进,规模小 |
| | 车辆 / 软体中点弹簧 / 布料 PBD | `phy-rigid/vehicle.rs` / `phy-soft/*` | 布料/软体 PBD 是 Gauss-Seidel(M19),需先改 Jacobi 才能并行;车辆每帧仅 4 轮射线,规模太小 |

### 5.7.3 落地架构:`GpuBackend` 策略层(仅 Web Demo)

因为只做 Web Demo,**不引入桌面 wgpu 依赖**。统一抽象放在 `phy-demo-web`(或 `phy-core`
仅 `#[cfg(target_arch="wasm32")]` 门控),用特征 flag `gpu` 切换:

```rust
/// 仅在 wasm32 + feature=gpu 下编译的 GPU 后端,经 WebGPU 跑 compute。
/// 桌面(非 wasm32)一律走 rayon CPU 路径,本 trait 不参与编译。
pub trait GpuBackend {
    /// 逐元素 map:对 [0,n) 每个索引独立计算,结果写回 GPU buffer。
    /// 光学像素循环 / SPH 逐粒子 / 焦散逐射线 都映射到此。
    fn par_map_idx(&self, n: usize, wgsl: &str, in_bufs: &[&GpuBuf], out_buf: &mut GpuBuf);

    /// 逐对 reduce:颗粒 PBD 接触投影,每对只读快照累加 per-body delta。
    fn par_pairs_reduce(&self, pairs: &GpuBuf, acc: &mut GpuBuf, wgsl: &str);
}
```

- 桌面:`#[cfg(not(target_arch="wasm32"))]` 或 `gpu` 未开 → 算法直接走现有 rayon,**代码不变**。
- Web:仅 wasm32 + `gpu` feature → 调 `GpuBackend`,把内核数学写成 wgsl,浏览器 WebGPU 执行。
- 算法侧切换点:把 `(0..n).into_par_iter().map(...)` 包一层 `if cfg!(gpu) { gpu_backend... } else { rayon... }`,
  **数值内核(Density/Force/PBD/caustics)逻辑与 wgsl 保持一一对应,保证两端结果可对照**。

### 5.7.4 Web Demo 实施路径(唯一落地路径)

| 阶段 | 内容 | 产物 |
|---|---|---|
| **W1** ✅ | `phy-demo-web` 加 `gpu` feature + WebGPU 初始化(device/queue/上下文),
  最小 compute 原型 `square_self_test`(逐元素平方)打通 wasm32 构建与浏览器 dispatch 链路。
  门控 `#[cfg(all(target_arch="wasm32", feature="gpu"))]`,默认/桌面构建不拉 wgpu。
  `DemoApp::gpu_self_test()` 暴露为 JS Promise 供 console 校验(`W1 gpu self-test ok=true out=[1.0,4.0,9.0,16.0,25.0]`)。
  **验证**:`cargo build -p phy-demo-web --target wasm32-unknown-unknown --features gpu` 通过;默认构建+`cargo test -p phy-demo-web` 全过(1 passed),确认零回归 | 已落地 |
| **W2** ✅ | 光学**实时近似后端(`Approx`)**走 GPU:逐像素 one-thread,复刻 CPU 端 `OpticScene::intersect`
  (sphere/box 两种形状)+ `Approx::trace`(折射+反射+Fresnel+阴影射线)到 wgsl。
  场景降为 f32 扁平缓冲(`BodyGpu` 80B/体 = pos+quat+geo+albedo_ior+misc),相机参数走 uniform。
  convex 形状 GPU 不支持 → `render_camera_gpu` 返回 Err 由调用方回退 CPU。
  `optic_self_test()` 自测入口(1 玻璃球场景 8×8 渲染,报告中心像素)经 `DemoApp::optic_self_test()` 暴露为 JS Promise。
  **验证**:`cargo build -p phy-demo-web --target wasm32-unknown-unknown --features gpu` 通过(零警告);默认 + `cargo test -p phy-demo-web` 零回归。
  **注**:Whitted(递归)后端未做 GPU(递归不适宜 one-thread-per-pixel),实时 Demo 默认走 Approx,符合 W2 范围 | 已落地 |
| **W3** ✅ | 焦散 `caustics::accumulate` 逐射线 march 走 GPU:复用 W2 `BodyGpu` 扁平缓冲,
  逐射线 one-thread 独立 `march`(每条射线只写自己的 grid 单元,无需 atomic 竞争),
  wgsl 复刻 `Caustics::march`(最多 8 段折射,命中透明体按 (1-Fresnel) 衰减并切换介质 IOR,
  命中不透明体返回累积 flux)。`render_caustics_gpu` 返回 `grid_n*grid_n` 强度;
  `caustics_self_test()`(1 玻璃球 16×16,报告 max/sum)经 `DemoApp::caustics_self_test()` 暴露为 JS Promise。
  **验证**:`cargo build -p phy-demo-web --target wasm32-unknown-unknown --features gpu` 通过;
  默认构建+测试零回归 | 已落地 |
| **W4** ✅ | SPH 逐粒子密度/受力上 GPU。`phy-fluid` 新增 `Grid::to_flat`(从 `HashMap` 邻居网格
  重构为 GPU 友好扁平布局 `FlatGrid{cell_start[c]/sorted[c]}` + 前缀和),`world.rs` 加
  `build_grid`(pub)+ `to_gpu_flat` 导出 `SphFlatData`(pos/vel/scalar/cell_start/sorted/
  grid_min/nc/h/rest_density/stiffness/visc_k/visc_n/shear_min/gravity 全部 f32 扁平)。
  `gpu/mod.rs` 译两个 wgsl entry point:`density_main`(网格遍历求密度ρ+近不可压压力修正)
  + `force_main`(Müller 压力梯度 Spiky + 粘性 Laplacian Visc + 重力 +  shear 有效粘度),
  逐粒子 one-thread 复刻 CPU SPH 内核。`sph_self_test()`(溃坝晶格,报告 mean_rho/ratio/
  finite/acc0)经 `DemoApp::sph_self_test()` 暴露为 JS Promise。
  **验证**:`cargo build -p phy-demo-web --target wasm32-unknown-unknown --features gpu` 通过;
  默认构建 + `cargo test -p phy-demo-web` / `cargo test -p phy-fluid`(18 passed)零回归 | 已落地 |
| **W5** ✅ | 颗粒 PBD 接触投影上 GPU(`par_pairs_reduce`)。`phy-granular` 新增 `gpu_flat.rs`
  的 `GranularFlatData`(pos/old/vel/inv_mass/pairs/npairs/gravity/bounds/iterations/
  vel_damp/friction/dt 扁平)+ `world.rs::to_gpu_flat`(预生成全部 O(n²) 接触对)。
  `gpu/mod.rs` 译三个 wgsl entry point:`clear_main`(清 delta 缓冲)→`contact_main`(每对
  只读预测位置、按反质量加权算位移修正、以 `atomic<i32>` 定点累加进 per-body delta,
  Jacobi 式确定性 reduce)→`apply_main`(逐体还原 delta + 盒边界夹紧写回预测位置);
  `iterations` 轮在主机端循环 dispatch,最后回读 pos。`granular_self_test()`(27 颗粒盒,
  报告 min_gap/overlaps/finite)经 `DemoApp::granular_self_test()` 暴露为 JS Promise。
  **验证**:`cargo build -p phy-demo-web --target wasm32-unknown-unknown --features gpu` 通过;
  默认构建 + `cargo test -p phy-demo-web` / `cargo test -p phy-granular` 零回归 | 已落地 |
| **W6** ✅ | 把 W1–W5 的 GPU 自测真正接到 Web 页面,使其可在浏览器里点按验证(此前只暴露 JS 方法,无 UI/无 gpu 构建)。
  (1) `index.html` 新增 **GPU 自测面板**(`<details>` 折叠 + 5 个按钮 W1平方/W2光学/W3焦散/W4 SPH/W5 颗粒),
  点按调用 `app.{gpu,optic,caustics,sph,granular}_self_test()`(返回的 `Promise<string>` 打印到 `<pre>`);
  用 `typeof app.X_self_test === 'undefined'` 检测:非 gpu 构建下按钮提示"本构建未启用 gpu feature"。
  (2) 新增 `build_gpu.py`:封装 `wasm-pack build --target web --features gpu`(含 `--release` 选项)产出带
  WebGPU 后端的 `pkg/`,使上述自测方法实际出现在 `pkg/phy_demo_web.d.ts` 中(已验证 5 个 `*_self_test` 均在)。
  (3) 已用 `wasm-pack build --target web --features gpu` 实际重建 `pkg/`(798KB wasm)。
  **验证**:GPU 构建产出 `pkg/` 含全部 5 个自测方法;默认 `cargo build` / `cargo test -p phy-demo-web`(1 passed)零回归;
  浏览器用 `serve.py` 起服务 + 支持 WebGPU 的 Chrome/Edge 即可点按钮跑自测。至此 §5.7 全部 W1–W6 闭环。 | 已落地 |

> W2/W3 无需重构数据布局(像素/射线天然独立),优先做;W4/W5 需先做 `Grid` 扁平化前置重构。

| **W7** ✅ | 运行时切换(runtime switch):流体/颗粒子系统的 `step` 在运行时切到 WebGPU compute,其余子系统(刚体/软体/场/光学)与**全部 `couple` 耦合阶段**仍走 CPU,耦合矩阵完整保留。
  (1) `phy-core` 新增 `World::step_skipping(dt, skip: &[bool])`:跳过 `skip[i]==true` 子系统的 CPU `step`,但**所有**子系统的 `couple` 照常运行;`World::step` 退化为 `step_skipping(dt, &[])`。
  (2) `phy-fluid` 新增 `FluidWorld<f64>::to_gpu_flat`(f64→f32 平铺,含 `visc_k`/`visc_n` 这两个 `Vec<f32>` 字段的逐元素转换)。
  (3) `phy-demo-web`(`gpu` feature)新增 `step_sph_gpu`/`step_granular_gpu`/`step_world_gpu`:GPU 算力/接触投影,主机侧做半隐式欧拉积分(velocity/pos 写回)+ 边界 clamp + 摩擦;GPU 接管后用 `get_mut` 循环(规避私有 `subsystems`)收集 `skip` 掩码再调 `step_skipping`。
  (4) `DemoApp` 改用 `Rc<RefCell<State>>` + `Rc<RefCell<Option<GpuContext>>>`,使 `frame_gpu` 的 `async` 未来具备 `'static` 以喂 `future_to_promise`;`index.html` 加 GPU 运行时切换开关 + `async` rAF 循环(`gpuMode` 时 `await app.frame_gpu()`)。
  **验证**:`gpu` 与默认两 wasm 构建均编译通过;`cargo test --workspace` 全过(0 失败);`step_skipping` 逻辑核对——跳过 CPU step 但 couple 全跑。浏览器内实时数值对比本环境无法跑(无 WebGPU 运行时),需真浏览器目测。 | 已落地 |

> W7 是 W1–W6 之上的"总开关",把 GPU 从"仅自测"升级为"可运行时接管生产步进"。B 档 GPU 标量场 stencil / 半拉格朗日平流 / FEM 刚度装配 / SPH 邻居网格构建尚未做(见 §5.7.2);确定性回放(§5.6 S8)尚未做。

### 5.7.5 不做项(明确排除)

- ❌ 桌面 wgpu / Vulkan / OpenGL compute 后端(避开 MinGW 链接崩溃,见 M3)。
- ❌ 刚体顺序冲量 / 关节 / 破碎 / CCD / 车辆 / 软体布料 的 GPU 化(强顺序依赖或规模太小,见矩阵 C)。
- ❌ 在 `phy-core` 引入跨平台运行时抽象(只服务于 Web Demo,门控在 wasm32)。

---

## 6. 决策记录 (Decision Log)

- 2026-08-07: 用户确认面向**通用科研/仿真**; 首期目标**完整可玩 Demo**; 数学库**由我推荐 → nalgebra 泛型**。
- 2026-08-07: 精度选**泛型可切换 (RealField)**; Demo 形态选 **wgpu 3D**。
- 2026-08-07: 用户扩展愿景至**流体 + 光学 + 其他物理规则**,目标**科研 + 游戏双用途、逼真模拟世界**。
- 2026-08-07: 流体选 **SPH 粒子法**; 光学选 **双后端可切换**; 确定性/WASM **暂不做**(架构预留)。
- 2026-08-07: M2 完成。求解器采用**顺序冲量 + 累积冲量钳制 + Split-Impulse 伪速度位置修正**(避免抖动/能量注入); 接触 SAT 加 `sep_eps=-1e-6` 容差,使恰好接触(pen≈0)被判为相交,修复堆叠测试中下盒穿地。
- 2026-08-08: M3 完成。Demo 因本机 MinGW 8.1 链接器对 wgpu 巨型依赖树崩溃(`corrupt .drectve`, `ld returned 5`),**从 wgpu 转向纯 Rust 软件光栅化方案**:`winit 0.30` + `softbuffer 0.4` 做窗口/帧缓冲呈现,自研 CPU 光栅化器(`raster.rs`:透视正确插值 + Z 缓冲 + 朗伯光照)渲染 `RigidWorld<f64>` 的盒/球实例。无需 GPU 后端,保证在任意 MinGW 工具链下可编译运行。交互:拖拽旋转、滚轮缩放、P 暂停、R 重置、G 加盒、B 加球、I 统计。
- 2026-08-08: M5 完成。`phy-rigid` 新增 `Shape::contains_local` + `Body::to_local/to_world`; 新建 `phy-fluid` crate: Müller 2003 弱可压缩 SPH(Poly6 密度/Spiky 压力梯度/ViscLaplacian 粘性核) + 均匀空间哈希邻居搜索。默认 h=0.2,单粒子质量由晶格核求和反算(`lattice_mass`)保证静止密度收敛。双向刚体耦合 `couple_bodies`: 静态体作不可穿透边界(位置推回 + 法向速度阻尼),动态体受阿基米德浮力 `-ρf·V_sub·g` + 无滑阻力反作用冲量。`FluidSubsystem` 适配 `phy_core::Subsystem`。6 测试全过(密度收敛、溃坝不越界、粒子数守恒、静/动态刚体耦合)。注意: 浮力测试须关闭重力隔离纯上举力,否则自由下落流体的下拽耦合会掩盖浮力。
- 2026-08-08: M6 完成。新建 `phy-optics` crate: 复用 `phy_rigid::Shape` 作几何 + 自研 `Surface`(albedo/IOR/roughness/transparent),光学体 `OpticBody` 由刚体位姿+形状+表面组成,`OpticScene` 统一求交(球/盒 Slab/凸体 Möller–Trumbore)。光学数学 `math.rs`: Snell `refract`(含 TIR 返回 None)、`reflect`、Schlick `fresnel`、`f0_of`。双后端实现同一 `Renderer` trait: `Whitted`(递归反射+折射,离线高保真)与 `Approx`(单次折射近似+阴影射线,实时)。`OpticSubsystem` 适配 `phy_core::Subsystem` 并提供 `render_camera` 针孔成像 + `Precision` 切换开关。`to_rgba8` 输出 ARGB 供软件帧缓冲。5 测试全过。`phy-demo` 接入: 按 `O` 切换光学模式,用 `Approx` 后端实时渲染玻璃球 + 地面(复用相机位姿)。
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
- 2026-08-08: M19 完成。**§5.5 路线图批处理(按优先级)**。落地 #11 变步长 / #6 声波场 / #3 布料 PBD / #12 统计观测器:① `phy-core::timestep` 的 `TimeController<T>`(`Fixed`/`Adaptive` 两种切片模式,push 帧 dt 按 `max_dt` 上界切子步、尾数跨帧累积;`time_scale` 慢动作/快进、`flush_accumulator` 防暂停补帧;无需新依赖,十进制字面量测试改用二进制可精确值 0.125/0.0625 规避浮点误差);② `phy-field::acoustic` 的 `AcousticField<T>`(复用 `ScalarField::step_wave` 解声压 `∂²p/∂t²=c²∇²p`,默认 `SOUND_SPEED_AIR=343`、ρ0=1.2,`add_pressure_source` 压力注入,实现 `GridGeometry`/`Subsystem`);③ `phy-soft::cloth` 的 `Cloth<T>`(nx×ny 质点网格 + 结构/剪切/可选弯曲 `DistanceConstraint`,PBD 预测-投影-速度回写 Gauss-Seidel `iterations` 次,`pin` 钉扎 + `ground_y` 地面碰撞,实现 `Subsystem`);④ `phy-core::stats` 的 `StatsObserver<T>`(累加步数/累计时间/最近步长/时钟单调异常/自定义遥测均值-最值)+ `attach_stats_observer`(经 `Rc<RefCell>` 订阅 M14 事件总线随 `World::step` 自动更新)。**决策**:布料未单独开 crate,挂在 `phy-soft`(复用 `Particle`/地面碰撞/`Subsystem`),避免 crate 膨胀;声波未另建模块而是 `WaveField` 之上的语义包装,保持场 crate 内聚。**验证**: `cargo test --workspace` 全过(核心 12、demo 7、场 13、刚体 22、软体 13、流体 9、光学 6、io 2、math 22)。路线图剩余:批次 B 的 #9 车辆 / #8 破碎,批次 C 的 #4 扩散-对流 / #5 FEM / #7 颗粒 / #10 流体增强。
- 2026-08-08: M20 完成。**§5.5 #9 射线投射车辆(raycast vehicle)**。`phy-rigid` 新增 `raycast.rs`:`ray_cast(origin, dir, body)` 支持 `Sphere`(闭式二次方程)、`Box`(射线变换到盒局部空间 slab 求交,进入轴法线按局部射线方向符号朝向来向)、`Convex`(逐三角形 Möller–Trumbore 取最近命中);返回 `RayHit{t,point,normal}`,4 单测覆盖命中/未中/顶面/侧面。新增 `vehicle.rs`:`Wheel<T>`(局部锚点 + 悬挂 rest/k/c + 轮半径 + 纵向 traction/横向 grip + 运行时 compression/grounded)+ `Vehicle<T>`(chassis 刚体索引 + 车轮列表 + engine/steering/brake 输入 + forward_axis)。`update(world, dt)` 对每个轮经 `ray_cast_ground`(仅对静态 `inv_mass==0` 物体打射线取最近命中)算悬挂压缩 → 沿地面法线施弹簧-阻尼力,并把引擎/刹车(乘以车身质量使减速与质量无关)/转向(自行车模型 `desired_lat = steering·vlong`)投影到地面切平面,作为线性力注入车身 `vel`(与 `couple_*` 同款,须在 `world.step` 前调用)。3 单测(悬挂托举使车身稳定悬于地面上方 / 引擎驱动前进 / 刹车减速 >50%)+ demo 集成 `world_drives_raycast_vehicle`(经 `World`+`RigidSubsystem` 驱动生效)。**决策**:本引擎刚体未建模角速度,故车辆姿态由四角对称悬挂自然维持直立、轮胎力作用于质心,未引入角动力学,保持最小侵入;`raycast` 作为独立能力也服务后续 #8 破碎碎片射线检测。**踩坑**:① 初版对 `find_ground` 返回体再 `ray_cast` 造成双重求交且阴影变量,改为 `ray_cast_ground` 直接返回最近命中;② 刹车力若除以质量则对 500kg 车身几乎无效,改为乘以质量使刹车成为与质量无关的减速度系数;③ 单元测试地面盒半宽 50 时车辆 400 步加速到 ~49m/s 驶出边缘坠落,放大到 1000 解决;④ box slab 法线初版用冗余 `sign` 变量,改为按进入轴 + 局部射线方向符号直接定法线。**验证**: `cargo test --workspace` 全过(刚体 29 + demo 8 含车辆集成)。路线图剩余:批次 B 的 #8 破碎,批次 C 的 #4 扩散-对流 / #5 FEM / #7 颗粒 / #10 流体增强。
- 2026-08-08: M21 完成。**§5.5 #8 Voronoi 破碎(fracture)**。`phy-rigid` 新增 `fracture.rs`:`fracture_body(body, n, seeds, radial)` 把凸刚体(球/盒/凸)按 N 个 Voronoi 种子切成 N 个 `Shape::Convex` 碎片,质量按体积比守恒、碎片继承母本线速度 + 沿碎片-母体质心方向径向飞散(`radial`);`fracture_convex` 用"逐种子半空间裁剪(三角面 Sutherland–Hodgman + 切口封盖扇化)得到凸细胞";`convex_volume_centroid` 用散度定理。新增 `RigidWorld::shatter(body_id, n, radial)`(独立 `impl<T: RealField + Copy + NumCast>` 块,主 impl 边界保持 `RealField + Copy` 不变以免级联波及 `subsystem.rs`)——`swap_remove` 母本、按质量比分配电荷、追加碎片。`lib.rs` 注册 `pub mod fracture` + `pub use fracture::{fracture_body, fracture_convex, convex_volume_centroid}`。新增 4 单测(fracture_box_yields_n_fragments / fragment_mass_sums_to_parent<5% / fracture_adds_radial_velocity / fracture_convex_preserves_volume_sum)+ 2 world 单测(shatter_box_produces_fragments_conserving_mass / shattered_fragments_step_stably;后者由 12 碎片/300 帧改为 6 碎片/60 帧,因凸-凸碰撞 O(n²) 原耗时 122s)+ demo 端到端 world_shatters_box_into_fragments。**决策**:沿用 M20 最小侵入——刚体无角速度,碎片只注线速度不引角动力学;`fracture_body` 不触 `RigidWorld`(只返回 `Body` 列表),由 `shatter` 加入世界,避免热路径耦合。**踩坑**:① 凸多面体裁剪必须对三角面做,顶点当多边形环裁剪会因顶点乱序得错误细胞(体积仅 1/8);② 封盖正交基须右手系 `w = n×u` 使 CCW 扇化外法指向 +nrm,否则封盖法线反转致体积积分部分抵消(测 5.875 vs 4);③ 种子须严格在物体内(内边距 `(i+1)/(per+1)` + 内边距随机点),落表面会切出零体积薄片被丢、碎片数不达标;④ `rand01` 哈希须 `(s>>8)&0xFFFFFF / 2^24` 归一化到 [0,1),漏掩码致种子坐标爆 1e8、整盒未被切。**验证**: `cargo test -p phy-rigid` 全过(35 含 6 新增),`cargo test -p phy-demo world_shatters_box_into_fragments` 通过;`cargo test --workspace` 全过(刚体 35、demo 8)。路线图剩余:批次 C 的 #4 扩散-对流 / #5 FEM / #7 颗粒 / #10 流体增强。
- 2026-08-08: M22 完成。**§5.5 #4 Boussinesq 扩散-对流闭环(烟羽/自然对流)**。`phy-field` 的 `ScalarField` 已有 FTCS 热扩散(`step_diffusion`),本次补平流项 `∇·(v u)` 把流体速度场耦合进来:`step_advect(vel_sampler, dt)` 用半拉格朗日反向追踪(对每格中心点沿 `−v·dt` 回溯、三线性插值取 `u`)实现无条件稳定的物质平流(替代更易数值发散的显式迎风)。`phy-fluid` 的 `FluidSubsystem::couple` 在已有热浮力(M4e `couple_heat`:`a += −g·β·(temp−t_ref)`)基础上,叠加把流体速度写入热场的 `vel_sampler`(闭包捕获 `&FluidWorld`),使热场随流体运动被平流——烟羽因此被"带上去又吹散"。**关键修复(初版 M22 测试 FAIL 的两处真实 bug)**:① `sph.rs::compute_forces` 原把 `acc = 压力+粘性)/rho` 直接**覆盖** `pt.acc`,把 `couple_heat` 已写入的浮力丢弃;改为引入 `body_acc` 字段(持续体积力:重力在 `integrate` 注入 + 浮力在 `couple_heat` 注入),`acc` 只存 SPH 邻域力,`integrate` 用 `a = acc + g + body_acc` 再清空 `body_acc`。② `FluidSubsystem` 原把 `t_ref` 从热场中心格采样(同 M11 旧坑),使物体落在最热格时浮力归零甚至反向;改为子系统显式 `t_ref` 字段(默认 `T::zero()`)。配合修复后 `boussinesq_closed_loop_plume_rises_and_advects` 实证:暖中心烟羽 centroid 由 y≈8 升到 y>8.4 且被平流扩散。新增/修正单测见 `phy-field`(`step_advect_*`)与 `phy-fluid`(`couple_points_buoyancy_upward`/`couple_heat_warmer_fluid_rises`/`boussinesq_closed_loop_*`)。**验证**: `cargo test --workspace` 全过(场 18、流体 10 等)。
- 2026-08-08: M23 完成。**§5.5 #5 弹性/塑性连续介质(FEM 小变形 线性四面体)**。`phy-solid` 新 crate:线性四面体单元(常数应变 B 矩阵 + 拉梅 λ/μ + 6×6 弹性 D 矩阵 + 12×12 单元刚度 `Ke=V·Bᵀ·D·B`),六节点 Box 网格经 6-Tet 标准剖分(主对角线 000-111,防翻转),集总质量 + 重力;支持任意节点位移边界固定与逐节点集中载荷缓冲 `load`(由 `step`/`solve_equilibrium` 累加,避免动力松弛重置 `f` 丢失外载);Cholesky 静力平衡求解(`SolidWorld::solve_equilibrium`,固定自由度行/列缩并为单位约束)+ velocity-Verlet + Rayleigh 速度阻尼动力松弛(`step`)。**耦合接入**:`SolidSubsystem` 适配 `phy_core::Subsystem`(`as_any`/`step`/`couple`/`name`),可挂入统一 `World<T>`;`set_gravity` 写内部 `gravity`。**关键对照验证**:悬臂梁自由端集中力 P 的 FEM 挠度与 Euler-Bernoulli 解析解 `δ=P L³/(3 E I)`(`I=b h³/12`)在 8 段单排四面体网格下偏差 <25%(离散四面体梁偏柔,符合预期)。4 单测全过(`simple_static_deflects_under_load`/`cantilever_tip_deflection_matches_euler_bernoulli`/`fixed_nodes_do_not_move`/`dynamic_relaxation_settles`)。**踩坑**:① `Tet` 非泛型 struct,误写 `Tet<f64>` 编译失败;② `cofactor` 余子式列构造:3×3 矩阵列应为 [常数(0), 两保留坐标轴],初版 `keep_cols` 只存 2 个坐标列导致 `keep_cols[2]` 越界 panic,改为显式 `[0, coord_cols[0], coord_cols[1]]`;③ `SolidWorld` 方法(`new`/`from_box`/`step`/`solve_equilibrium`)要求 `T: RealField+Copy+ToPrimitive+FromPrimitive`,所有 mesh/subsystem 辅助函数须显式带上完整边界否则 E0277;④ 泛型 `RealField` 不应混用 `nalgebra::Vec3`(无此名),统一走 `phy_math::Vec3<T>`;⑤ `step` 初版 `node.f = gravity` 覆盖式重置会抹掉 `apply_tip_load` 写入的外载,改用独立 `load` 缓冲累加。**验证**: `cargo test -p phy-solid` 4/4 全过;`cargo test --workspace` 全过(软体 11、刚体 18、流体 10、场 18、光学 6、demo/N 项;新增 phy-solid 4)。路线图剩余:批次 C 的 #7 颗粒 / #10 流体增强。
- 2026-08-08: M24 完成。**§5.5 #7 颗粒介质(离散元 / 位置动力学 PBD)**。`phy-granular` 新 crate:球面颗粒 `Grain{pos,vel,radius,inv_mass}` + `GranularWorld{grains,bounds,iterations,vel_damp,friction,gravity,t}`。采用 **PBD 顺序**(预测 `p+v·dt+g·dt²` → Gauss-Seidel 投影 `iterations` 次 → 速度回写 `v=(p_new−p_old)/dt`):保持海量接触约束下无条件稳定。约束两类:① **球-球非穿透**(单边距离约束,`gap=d−(ri+rj)<0` 时沿连线按反质量加权把两球推开到恰好相切,分离后不再作用);② **盒边界**(颗粒中心约束在 `[lo,hi]` 留半径余量,撞墙法向速度消去)。切向摩擦经 `friction` 对全局速度做轻微衰减近似维持堆积角。`fill_grid(count,radius,mass,pack)` 在盒内沿 XYZ 按 `2·radius·pack` 间距铺排撒布(确定性网格,`pack>1` 留初始间隙)。`GranularSubsystem` 适配 `phy_core::Subsystem`(`as_any`/`step`/`couple`/`name`),挂入统一 `World<T>`。**未引入额外依赖**(仅 nalgebra + serde.workspace)。**验证**:4 单测全过——`grains_fall_and_settle_in_box`(200 颗粒重力沉降,400 步后全在盒内无 NaN、最大速度<0.5 趋静止)/`no_overlap_after_projection`(120 颗粒紧密铺排,200 步后任意两球中心距 ≥ 半径和−1e-4,无穿透)/`fixed_grain_does_not_move`(inv_mass=0 锚点重力下零位移)/`runs_as_subsystem_in_world`(作为 Subsystem 挂入 World 步进 60 帧正常推进 t)。`cargo test --workspace` 全过(新增 phy-granular 4)。路线图剩余:批次 C 仅 #10 流体增强。
- 2026-08-08: M25 完成。**§5.5 #10 流体增强:刚体↔流体双向耦合**。此前 SPH 已有 `couple_bodies`(浮力+阻力+动量交换)与 `couple_points`,但**未接入统一 `World` 的 `couple` 流程**(`FluidSubsystem::couple` 只接了热场,`RigidSubsystem::couple` 只接热/电磁/引力)。本次让 **`FluidSubsystem` 主导**刚体耦合:在 `couple` 中动态查找 `"rigid"` 子系统,`remove` 后取 `RigidSubsystem::world.bodies` 可变引用调用 `couple_bodies(bodies, dt, friction)`,结束放回。**刚体侧不再反向耦合流体**,避免两个子系统在 `World::couple` 中互 `remove` 造成双向死锁(`World::step` 调 `couple` 时 `self` 已被取出,流体侧可安全 remove 其它子系统)。新增 `FluidSubsystem.drag`/`friction` 字段(drag 预留等效绕流阻力强度透传,friction` 控接触切向动量耗散)。`phy-fluid/Cargo.toml` 加 `phy-rigid` 依赖(无循环:`phy-rigid` 不依赖 `phy-fluid`)。**新增验证**:`rigid_body_gets_buoyancy_via_world_couple` 子系统级集成测试——在 `World` 中同挂流体+刚体,`World::step` 自动经 `couple` 把浮力注入刚体,早期帧 `vy>0` 证明链路生效(诊断:帧 0-15 vy 转正,~20 帧后 SPH 在持续刚体挤压下条件发散,属 SPH 数值稳定性范畴,非耦合逻辑;受控 sph 测试同尺度仅临界不爆)。`cargo test --workspace` 全过(phy-fluid 11 项含新集成测试)。**批次 C 全部完成(#4 湍流/#5 弹性 FEM/#7 颗粒/#10 流体增强)**,路线图主干功能模块闭环。
- 2026-08-08: M26 完成。**World 存档/读档(serde JSON 持久化)**。让 `World<T>` 能被保存/加载(存档格式版本 + 仿真时间 + 各子系统),支撑科研复现与游戏存档。核心难点:`World` 持有 `Vec<Box<dyn Subsystem<T>>>`(trait object),serde **无法直接序列化 trait object**,故用**枚举派发**(`SubArchive<T>`)绕过——存档时把每个子系统 `downcast` 到具体类型并带类型标签序列化,读档时按标签重建装回 `World`。**关键技术约束与决策**:① nalgebra 的 `Vec3<T>`/`UnitQuaternion<T>` 自带 serde impl 要求 `T: nalgebra::Scalar`,泛型 `RealField` 不满足 → 自建 `serde_geom` 模块(`#[serde(with="...")]`)把 `Vec3` 序列化为 `[x,y,z]`、四元数序列化为 `[w,i,j,k]`,`vec3_vec` 子模块处理 `Vec<Vec3<T>>`,统一边界 `T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar [+ num_traits::ToPrimitive/FromPrimitive 视子系统而定]`;② `phy-field` 为避免与 `phy-rigid` 循环依赖(它需 `phy_rigid::shape::serde_geom` 但 `phy-rigid` 不依赖 `phy-field`),**内置同名本地 `serde_geom`** 模块(`crates/phy-field/src/serde_geom.rs`),其余 crate 复用 `phy_rigid::shape::serde_geom`;③ 所有 `derive(Serialize, Deserialize)` 结构体用 `#[serde(bound="...")]` 显式约束 `T`(否则自动推导会要求 `T: Serialize+DeserializeOwned` 但漏掉 `nalgebra::Scalar`/`ToPrimitive` 触发 E0277);④ trait object / 闭包字段(如 `HeatField::vel_sampler` 闭包)用 `#[serde(skip)]` 跳过并补手写 `Clone`/`Debug`;⑤ `phy-granular` 的 `bounds: (Vec3,Vec3)` 元组拆成 `bounds_lo`/`bounds_hi` 两个 `Vec3` 字段以绕开 nalgebra 元组序列化限制;⑥ `phy-fluid::FluidWorld` 因含 `kernels`/`grid` 等非序列化运行时缓存,改用**手写 `Serialize`/`Deserialize`**:经 `FluidWorldData<T>{params, particles}` 只持久化数据,读回后由 `with_particles` 重建缓存;⑦ `E0449`:`Joint::Distance` 变体字段误加 `pub` 被 serde derive 拒绝,已移除;⑧ `World` 新增 `set_time(&mut self, t)` 供读档回填仿真时间。`phy-core` 加 `num-traits`/`serde`/`serde_json` 依赖并暴露 `set_time`;`phy-io` 新增 `save_world`/`load_world`/`save_world_json`/`load_world_json`/`archive_world`/`unarchive_world`/`WorldArchive`/`SubArchive`,`Cargo.toml` 补齐 `phy-soft`/`phy-solid`/`phy-granular`/`phy-fluid`/`phy-optics`/`serde_json` 依赖。**验证**:`phy-io` 新增单测 `world_save_load_roundtrip`(刚体 + 热场两子系统存档→JSON→读档,子系统数/刚体位姿/inv_mass/热场网格维度与 dx 全部精确恢复);`cargo test -p phy-io` 全过(3 测试含存档往返)。各 crate 均带本类型序列化往返单测。
- 2026-08-09: S5a 完成。**§5.6.1 S5 气体/烟雾多相与燃烧(前半:烟/气被动标量 + 浮力)**。`phy-field` 新增 `crates/phy-field/src/smoke.rs` 的 `SmokeField<T>`(作为 `grid::ScalarField` 的 `Subsystem` 适配层),模拟一团随流体平流、并因自身比空气轻而**上浮**的烟/气被动标量(浓度 `s≥0`)。与 `HeatField`(温度浮力作用于流体)不同,这里把浮力直接体现为烟气的**有效平流速度附加项**:`v_eff = v_fluid + buoyancy·s·ŷ`,使浓烟上升、淡烟随风,自然形成烟羽,无需改动流体求解器即可经 `set_vel_sampler` 耦合。`lib.rs` 注册 `mod smoke; pub use smoke::SmokeField;`;`new`/`inject_blob`/`conc_at_world`/`set_vel_sampler`/`step`(`Subsystem` trait:有速度采样器走 `step_advection_diffusion` 加浮力闭包,否则纯 `step_diffusion`)+ `name()="smoke"`+ 手写 `Clone`/`Debug` + 三线性浓度采样 + `GridGeometry` 适配。热/冷边界均为 `Bc::Neumann`(烟不漏出)。**关键数值决策(初版 S5a 测试 FAIL 的真实修正)**:半拉格朗日平流在 smoky 格会向下回溯取值,若浮力上升速度按**局地**浓度(共位)取,则烟团上方空格无上升速度、烟只会"原地消失"而非上浮;故在 `step` 内先把浓度快照做数次 3×3×3 盒式模糊(`blur3`,粘性尺度)得到"浮力上升源场",使烟团正上方数格也获得上升速度,烟才真正被抬升形成烟羽(对应 `smoke_rises_with_buoyancy` 实证:质心 y 由 2.0 升到 >2.0)。**验证**:`phy-field` 新增 3 单测 `smoke_advects_with_fluid_velocity`(随流平流,峰平移 1 dx)/`smoke_rises_with_buoyancy`(静止流体下浮力抬升)/`smoke_no_buoyancy_stays_put`(无浮力静止只扩散,质心不动);`cargo test -p phy-field` 全过(21 项)。S5 后半 火焰前缘推进(点燃/消耗)留待后续扩展(S5b)。
- 2026-08-09: S5b 完成(接 S5a)。**§5.6.1 S5 燃烧/火焰前缘**。在 `SmokeField<T>` 上叠加燃烧:`fuel`(可燃料密度,`ScalarField`,与烟共用网格,随流平流/扩散)、`fuel_burn_rate`/`ignition_temp`/`heat_release`/`flame_spawn` 四个燃烧参数。燃烧逻辑抽到 `burn(&temp_fn, dt)`:对每个有燃料且 `temp_fn(p) ≥ ignition_temp` 的格,按 `burn_rate·dt` 消耗燃料、等量转烟(`flame_spawn`)、累积放热到 `heat_scratch`。耦合分两路:(1) 单机/无 `World` 场景用可选 `heat_sampler`(世界坐标→温度)在 `step` 内就地燃烧(放热累积丢弃,仅验证燃料→烟);(2) `World` 场景在 `couple` 阶段经 `world` 找到共注册的 `HeatField`,用裸指针只读读取其温度调用 `burn`,再把 `heat_scratch` 经新加的 `HeatField::add_temperature_at` 写回温度场。放热按"火焰核"分配到燃烧格(0.6)及其 6 个面邻格(各 0.4/6,直接累加温度,不经 `src·dt` 弱源项),使邻格被加热到点燃阈值 → **自持传播的火焰前缘**(Boussinesq 燃烧闭环)。`HeatField` 新增 `temp_at_world`(世界坐标三线性温度采样,供跨场读温)与 `add_temperature_at`(直接累温)。`SmokeField` 手写 `Debug`/`Clone`(跳过 `Box<dyn Fn>` 采样器,存档安全),`serde` 边界加 `T: Default`(与 `ScalarField` 一致);`GridGeometry` 仅实现 `dims/origin/cell_size`(按接口实际签名)。**验证**:`phy-field` 新增 2 单测 `combustion_consumes_fuel_and_makes_smoke`(单机:燃料↓ + 烟↑)/`flame_front_propagates_via_heat_coupling`(`World` 内 `HeatField`+长条燃料,中心点燃 2000°C + 放热 3000/燃速 2.0/点燃阈值 250/α=0.3,80 步后燃料消耗 >50% 且烟↑,实证前缘传播);`cargo test -p phy-field` 全过(23 项),全 workspace 测试通过。`inject_heat_world`(经 `src·dt` 弱源项)保留为 `HeatField` 公共 API,本次燃烧改用 `add_temperature_at` 以获可靠前缘。

- 2026-08-09: S6 完成。**§5.6.1 S6 连续碰撞检测(CCD)**。`phy-rigid` 的 `RigidWorld::step` 重构为**位移受限子步化** CCD:每帧按"最快可动体位移 / 其最小特征尺寸(默认 ≤½)"决定子步数 `n`(上限 `SolverParams::ccd_max_substeps`,默认 8,设 0 退化为原整步离散,与旧行为等价),把 `[0,dt]` 切成 `n` 个子步,在每个子步上跑既有离散 `collide`+顺序冲量速度求解,再按子步时长推进位置与姿态。因每子步位移远小于物体尺寸,离散检测必能在步内捕获接触,从而**彻底杜绝高速隧穿**;低速/无接近时 `n=1` 等价于原流程,不破坏既有 46 项(关节/自旋/破碎/堆叠/车体)测试。新增 `crates/phy-rigid/src/ccd.rs`(`substep_count` 位移受限子步数计算 + `swept_sphere_sphere` 球-球闭式 TOI + `ccd_contact` 合成接触,各带单测)。**顺带修复一个真实碰撞 bug**:原 `narrowphase::collide` 仅有 球-球 / 盒-盒 两条快速路径,球撞盒(墙/地板)会回退到 GJK/EPA 且漏检(重叠也返回 `None`),导致球穿过墙/地面 —— 新增 `sphere_box` 解析快速碰撞路径(球心变换到盒局部坐标夹取最近点,区分球心在盒外/内两种情形给法线与深度,法线约定由 a→b 与 `sphere_sphere`/`box_box` 一致),并修正 `collide` 对两种球-盒顺序的派发。**验证**:`phy-rigid` 新增 4 单测 `fast_sphere_does_not_tunnel_through_wall`(100 u/s 球被薄壁挡下,`ccd_max_substeps=8`)/`fast_sphere_tunnels_with_ccd_disabled`(对照组关 CCD 如期隧穿,证伪验证)/`ccd_enabled_matches_discrete_at_low_speed`(低速开启 CCD 稳态无穿透,与离散一致)/`fast_box_does_not_tunnel_through_ground`(高速盒不穿地);`cargo test -p phy-rigid` 全过(50 项),全 workspace 测试通过。

- 2026-08-09: S7 完成。**§5.6.1 S7 GPU/并行后端(并行 CPU-rayon)**。GPU 后端因 MinGW 8.1 链接 wgpu 巨型依赖树崩溃(`corrupt .drectve`,M3 决策)放弃,改走 **rayon 并行 CPU** 后端,落点 `phy-fluid`/`phy-granular`/`phy-optics` 三个最易并行热点。核心手法均为"只读遍历 → 独立缓冲写回",无数据竞争:(1) **SPH**(`crates/phy-fluid/src/sph.rs`):`compute_density_pressure` 与 `compute_forces` 原串行双重循环,改为先把本步 `pos/vel/rho/p/mass/material` 快照成并行向量,`into_par_iter` 逐粒子算 `rho/p`/`acc/mu_eff` 累加到独立输出缓冲,再一次性写回 `self.particles`(Jacobi 式,结果与遍历顺序无关);(2) **颗粒 PBD**(`crates/phy-granular/src/world.rs`):球-球接触投影原为 **Gauss-Seidel**(每对即时改写 `predicted`,顺序敏感、不可并行),改为 **Jacobi**——每对只读迭代起点 `predicted` 快照,把位移修正累加到 per-body `deltas` 缓冲(并行 `par_iter().fold().reduce()`,`reduce` 以固定索引顺序合并),迭代末统一施加;Jacobi 修正大小与串行逐对相加一致,仅求和顺序固定,**保持确定性**(利于 S8);(3) **光学渲染**(`crates/phy-optics/src/subsystem.rs`):`render_camera` 逐像素 `trace` 相互独立,改为 rayon 并行 `(0..w*h).into_par_iter().map(trace).collect()` 填色缓冲后再回填 `buf`;因 `dyn Renderer<T>` 不可跨线程共享,把函数泛型为 `render_parallel<R: Renderer<T> + Sync>`,按 `Precision` 分发到 `&Whitted`/`&Approx`(二者零字段单元结构体,天然 `Sync`),避开 trait object 不可 `Sync` 限制。`workspace.dependencies` 加 `rayon = "1"`,`phy-fluid`/`phy-granular`/`phy-optics`/`phy-demo` 的 `Cargo.toml` 引用。**验证**:`phy-granular` 新增 `parallel_solve_is_deterministic`(同初态跑 120 步两次,逐粒位置差 <1e-12,证并行 reduce 可复现);`cargo test -p phy-fluid -p phy-granular -p phy-optics -p phy-rigid` 全过(fluid 18 / granular 5 / optics 7 / rigid 50),全 workspace 26 测试套件零失败。
- 2026-08-09: GPU 加速范围决策(§5.7)。用户明确:**只做 Web Demo,不考虑桌面 Demo**。即 GPU 仅经 `phy-demo-web` 的 WebGPU(wasm32 目标)在浏览器内落地,避开桌面 wgpu 链接崩溃(M3);桌面端维持 S7 的 rayon CPU 并行,**不引入任何 wgpu 桌面依赖**。各算法 suitability 见 §5.7.2 矩阵:A 档(SPH 密度/受力、颗粒 PBD、光学逐像素 trace、焦散)最易并行、首选;B 档(标量场 stencil、平流、FEM 装配、SPH 邻居网格)需先改 Jacobi/重构;C 档(刚体顺序冲量、关节、破碎、CCD、车辆、软体布料)强顺序依赖或规模太小不做。抽象为仅 `#[cfg(target_arch="wasm32")]` + feature `gpu` 下编译的 `GpuBackend`,算法侧用 `if cfg!(gpu){...}else{ rayon }` 切换,数值内核与 wgsl 一一对应便于对照。Web Demo 分 W1–W5 阶段(W1 WebGPU 上下文原型 → W2 光学 → W3 焦散 → W4 SPH(需 Grid 扁平化前置)→ W5 颗粒 PBD)。
- 2026-08-09: W1 完成(§5.7.4)。`phy-demo-web` 新增 `gpu` feature(门控 `wgpu`/`bytemuck`/`futures`/`wasm-bindgen-futures`),新增 `src/gpu/mod.rs`:`GpuContext`(WebGPU device/queue 申请)+ 最小 compute 原型 `square_self_test`(逐元素平方 wgsl,one-thread-per-element,workgroup 64)+ 高层 `gpu_self_test`。编译门控 `#[cfg(all(target_arch="wasm32", feature="gpu"))]` 保证桌面/默认构建不拉 wgpu(避 M3 崩溃);`DemoApp::gpu_self_test()` 经 `wasm_bindgen_futures::future_to_promise` 暴露为 JS Promise。`cargo build -p phy-demo-web --target wasm32-unknown-unknown --features gpu` 通过;默认 `cargo build`/`cargo test -p phy-demo-web` 零回归。后续 W2–W5 复用同一"逐元素 map"骨架,仅替换 @compute 函数体。
- 2026-08-09: W2 完成(§5.7.4)。光学实时近似后端 `Approx` 走 GPU。gpu feature 扩 `phy-optics`/`phy-rigid`/`bytemuck(derive)`;`gpu/mod.rs` 新增 `BodyGpu`(80B/体 f32 扁平,repr(C,align(16),Pod/Zeroable))、`flatten_scene`(sphere/box 支持,convex 返回 None 回退 CPU)、`render_camera_gpu`(相机 uniform + body storage + 逐像素 wgsl)+ wgsl `OPTIC_WGSL`(复刻 `intersect2` sphere/box + `approx_trace` 折射/反射/Fresnel/阴影 + 主 `main`) + `optic_self_test`(1 玻璃球 8×8 自测)。`DemoApp::optic_self_test()` 暴露为 JS Promise。`cargo build ... --features gpu` 零警告通过;默认构建+测试零回归。Whitted(递归)后端未 GPU 化(不适合 one-thread-per-pixel)。
- 2026-08-09: W3 完成(§5.7.4)。焦散逐射线 march 走 GPU。复用 W2 的 `BodyGpu`/`flatten_scene`;`gpu/mod.rs` 新增 `render_caustics_gpu`(CausticParams uniform: travel/plane_y/half_extent/grid_n/env_ior; 每条射线 one-thread 独立 march,写自己 grid 单元无 atomic)+ wgsl `CAUSTIC_WGSL`(自带 intersect2 sphere/box + `Caustics::march` 复刻:最多 8 段折射、Fresnel 衰减、介质 IOR 切换、命中不透明返回 flux)+ `caustics_self_test`(1 玻璃球 16×16,报告 max/sum)。`DemoApp::caustics_self_test()` 暴露为 JS Promise。`cargo build ... --features gpu` 通过;默认+测试零回归。
- 2026-08-09: W4 完成(§5.7.4)。SPH 逐粒子密度/受力走 GPU。`phy-fluid` 前置重构: `Grid::to_flat`(HashMap 邻居网格→扁平 `FlatGrid{cell_start[c]/sorted[c]}` 前缀和,`grid.rs` 新增 pub(crate))+ `world.rs` 加 `pub build_grid` + `to_gpu_flat` 导出 `SphFlatData`(pos/vel/scalar/cell_start/sorted/grid_min/nc/h/rest_density/stiffness/visc_k/visc_n/shear_min/gravity 全 f32 扁平,`sph/gpu_flat.rs` 新模块 + `lib.rs` 导出);`phy-demo-web` 新增 `phy-fluid` 进 `gpu` feature。`gpu/mod.rs` 译两 wgsl entry point: `density_main`(遍历 27 邻居格算密度 ρ + 近不可压压力标量 p=stiffness·(ρ−rest_density))与 `force_main`(Müller 压力梯度 Spiky + 粘性 Laplacian Visc + 重力 + shear 有效粘度 μ_eff=(ki+shear_min)·|dot(dv,n)|^ni,逐粒子 one-thread 复刻 CPU 内核)。`sph_self_test`(溃坝晶格,报告 mean_rho/rest/ratio/finite/acc0)经 `DemoApp::sph_self_test()` 暴露为 JS Promise。`cargo build ... --features gpu` 通过;默认 + `cargo test -p phy-fluid`(18 passed)零回归。
- 2026-08-09: W5 完成(§5.7.4)。颗粒 PBD 接触投影走 GPU(`par_pairs_reduce`)。`phy-granular` 新增 `gpu_flat.rs::GranularFlatData`(pos/old/vel/inv_mass/pairs/npairs/gravity/bounds_lo/hi/iterations/vel_damp/friction/dt 扁平)+ `world.rs::to_gpu_flat`(预生成全部 O(n²) 接触对 `(i<j)`,T→f32 经 `num_traits::cast`);`phy-demo-web` 把 `phy-granular` 加进 `gpu` feature 并 `lib.rs` 暴露 `granular_self_test` JS Promise。`gpu/mod.rs` 译三 wgsl entry point: `clear_main`(清 per-body 3 分量 delta 缓冲)→ `contact_main`(每对只读预测位置、按反质量加权算位移修正、以 `atomic<i32>` 定点 ×1e6 累加进 `deltas[i*3+axis]`/`deltas[j*3+axis]`,Jacobi 式确定性 reduce)→ `apply_main`(逐体 `atomicLoad` 还原 delta + 盒边界夹紧写回预测位置);`render_granular_gpu` 在主机端循环 `iterations` 轮 dispatch 后回读 pos。`granular_self_test`(27 颗粒盒,报告 min_gap/overlaps/finite)。`cargo build ... --features gpu` 通过;默认 + `cargo test -p phy-granular` 零回归。至此 §5.7 W1–W5 全部落地,Web Demo 侧 A 档算法(SPH/颗粒 PBD/光学/焦散)GPU 化收尾;后续如需可继续 B/C 档(标量场 stencil、刚体顺序冲量等)但本次按用户口径只做 Web Demo 且 A 档已完成。
- 2026-08-10: W6 完成(§5.7.4)。把 W1–W5 的 GPU 自测接到 Web 页面。`index.html` 加 GPU 自测面板(5 按钮点按调 `app.{gpu,optic,caustics,sph,granular}_self_test()`,Promise 结果打印到 `<pre>`;非 gpu 构建用 `typeof app.X_self_test==='undefined'` 提示未启用)。新增 `build_gpu.py`(`wasm-pack build --target web --features gpu`,支持 `--release`)重建 `pkg/`。已实跑 `wasm-pack build --target web --features gpu` 产出 pkg(798KB wasm),`phy_demo_web.d.ts` 含全部 5 个 `*_self_test` 方法。默认 `cargo build`/`cargo test -p phy-demo-web`(1 passed)零回归。§5.7 W1–W6 全闭环。
- 2026-08-10: W7 完成(§5.7.4)。**运行时切换(runtime switch)**:流体/颗粒子系统的 `step` 在运行时切到 WebGPU compute,其余子系统(刚体/软体/场/光学)与**全部 `couple` 耦合阶段**仍走 CPU,耦合矩阵完整保留。
  (1) `phy-core` 新增 `World::step_skipping(dt, skip: &[bool])`:跳过 `skip[i]==true` 子系统的 CPU `step`,但**所有**子系统的 `couple` 照常运行;`World::step` 退化为 `step_skipping(dt, &[])`。
  (2) `phy-fluid` 新增 `FluidWorld<f64>::to_gpu_flat`(f64→f32 平铺,含 `visc_k`/`visc_n` 这两个 `Vec<f32>` 字段的逐元素转换)。
  (3) `phy-demo-web`(`gpu` feature)新增 `step_sph_gpu`/`step_granular_gpu`/`step_world_gpu`:GPU 算力/接触投影,主机侧做半隐式欧拉积分(velocity/pos 写回)+ 边界 clamp + 摩擦;GPU 接管后用 `get_mut` 循环(规避私有 `subsystems`)收集 `skip` 掩码再调 `step_skipping`。
  (4) `DemoApp` 改用 `Rc<RefCell<State>>` + `Rc<RefCell<Option<GpuContext>>>`,使 `frame_gpu` 的 `async` 未来具备 `'static` 以喂 `future_to_promise`;`index.html` 加 GPU 运行时切换开关 + `async` rAF 循环(`gpuMode` 时 `await app.frame_gpu()`)。
  **验证**:`gpu` 与默认两 wasm 构建均编译通过;`cargo test --workspace` 全过(0 失败);`step_skipping` 逻辑核对——跳过 CPU step 但 couple 全跑。浏览器内实时数值对比本环境无法跑(无 WebGPU 运行时),需真浏览器目测。

---

## §5.8 库化 hardening 路线图(Library Hardening Roadmap)

**目标**:把 `phy-*` 引擎编译成稳定库,供游戏 / 防战建模等业务调用。当前 12 个 crate 是 Rust `rlib` 依赖图,`phy-core` 泛型 `RealField` + `Subsystem`/`couple` 解耦,`phy-io` 已支持 serde JSON 存档。作为 **Rust 库已较成熟**,但对外(尤其非 Rust 业务)调用有四个硬缺口。按优先级推进:

### L1 【高】全局守恒 / 稳定性回归 + phy-math 测试
- 新增 `World` 级集成测试(`crates/phy-core/tests/regression_stability.rs`):
  - 复用 S12 统计观测器(`StatsObserver`),跑 **1000+ 步**断言:(a) 全程无 NaN/Inf(`any field finite`);(b) 孤立系统(无外力、无耦合源项)总动能/动量漂移有界(如 `<1e-3` 相对或 `<1e-6` 绝对值);(c) 子系统数、各子系统 `t` 单调推进。
  - 覆盖至少 3 个典型场景:① 仅 SPH 溃坝;② SPH+刚体浮力耦合;③ 热场+流体 Boussinesq 闭环。
- `phy-math` 补基础单测(当前 0 例):`Vec3` 加减/点积/叉积/范数、四元数乘法/归一化、矩阵乘、`clamp`/`lerp` 边界。`crates/phy-math/tests/math.rs`。
- **交付**:守护"大步数下不静默发散",是库化可信度底线。纯 CPU、不依赖浏览器。

### L2 【高】确定性(S8 数值确定性 / WASM 回放)
- 固定步长驱动(`World::step(dt)` 用调用方给定 `dt`,引擎层不自行变步长;变步长控制器 S11 作为可选包装);或显式 `step_fixed(dt)`。
- 确定性浮点:统一 `f64` 严格运算顺序(S7 已把颗粒 PBD 改 Jacobi + 固定索引 reduce 保确定性);新增 `is_deterministic` 集成测试——同初态跑两次 N 步,逐子系统状态差 `<1e-12`(复用 `phy-granular::parallel_solve_is_deterministic` 思路扩展到全 `World`)。
- 存档读档 + 同种子重放:固定 RNG 种子(`fill_grid` / 撒布用可注入 `rng`),读档后重放得逐位一致结果。
- **交付**:防战建模"同输入同输出"可复现硬门槛。

### L3 【高】C ABI 层(`phy-ffi` 新 crate) ✅ 已落地
- 新建 `crates/phy-ffi`:`crate-type = ["cdylib", "rlib"]`,`cbindgen` 生成 C 头(`PhyWorldHandle *` 不透明柄,**纯 C 安全**:`World<double>` 模板语法已规避)。
- 暴露 `extern "C"` 稳定接口(句柄为 `*mut PhyWorldHandle`,Rust 侧 `Box<World<f64>>` 拥有,边界指针转换):
  - 工厂:`phy_world_create_fluid / _rigid / _granular / _coupled` → 溃坝 / 下落球 / 颗粒堆积 / 流体+刚体耦合四个典型场景。
  - 步进/查询:`phy_world_step(w, dt)`(0/-1) / `phy_world_time` / `phy_world_sub_count` / `phy_world_fluid_count` / `phy_world_rigid_count`。
  - 状态读回:`phy_world_get_fluid_positions/velocities(w, buf, len)`(x,y,z 交错) / `phy_world_get_rigid_transforms(w, buf, len)`(pos+quat 共 7 f64 交错)。
  - 存档:`phy_world_save(w, path)` / `phy_world_load(path)`(复用 `phy-io` JSON 序列化,`T=f64`)。
  - 释放:`phy_world_destroy(w)`(空指针/重复释放安全 no-op)。
- 所有入口经 `catch_unwind(AssertUnwindSafe(...))` 包裹,**Rust panic 绝不跨 FFI**(已单测 `ffi_null_pointer_is_safe` 验证空指针返回哨兵而非崩溃)。指针访问统一走 `as_world`/`nonnull_slice` 辅助,杜绝越界。
- 3 个 Rust 单测全过:生命周期+读回无 NaN / 存档载入 roundtrip / 空指针安全。
- **交付**:`cargo build -p phy-ffi --release` → `target/release/phy_ffi.{dll,so,dylib}`;头 `PHY_FFI_GEN_HEADER=1 cargo build -p phy-ffi` → `crates/phy-ffi/phy_ffi.h`。Unity/Unreal/C++ 业务可 `dlopen`/链接调用。
- C 侧最小用法:
  ```c
  #include "phy_ffi.h"
  PhyWorldHandle *w = phy_world_create_fluid();
  double pos[3 * phy_world_fluid_count(w)];
  for (int f = 0; f < 200; f++) phy_world_step(w, 0.005);
  phy_world_get_fluid_positions(w, pos, 3 * phy_world_fluid_count(w));
  phy_world_save(w, "scene.json");
  phy_world_destroy(w);
  ```
- 后续可扩:`phy_world_set_external_force`(外部施力注入)、`phy_world_add_subsystem(tag, json_cfg)`(动态组装场景)、GPU 路径开关(见 §5.8 GPU 约束,需 MSVC/Linux 工具链解锁桌面 wgpu)。

### L4 【中】性能基线 / 全局 NaN 看门狗 / GPU 数值实测
- **L4-1 性能基线(criterion)**:新增 `crates/phy-demo/benches/perf.rs`(criterion，`harness=false`)，建立两块业务关心场景的单帧步进基线：
  - SPH 溃坝 `step_n{500,1000,2000}`(自由演化，密度/压力/受力两遍 + 积分)；
  - 刚体接触 `step_m{64,128,256}`(重力下盒体地面堆叠 + 球-球/球-地接触约束)。
  - 运行 `cargo bench -p phy-demo`；基线实测(Ryzen 5700X, release)：`step_n500≈94µs / n1000≈396µs / n2000≈1.72ms`、`step_m64≈2.1µs / m128≈5.3µs / m256≈16µs`。给“库化后喂游戏/防战建模”提供吞吐参考点。
  - 备注:原计划放 `phy-core/benches/`，但 SPH/刚体世界构造依赖 `phy-fluid`/`phy-rigid`(phy-core 不依赖它们)，故落到 `phy-demo`(依赖全集)。
- **L4-2 全局 NaN/Inf 看门狗**:
  - `phy-core`:新增 `WorldError` 枚举(`NonFinite{subsystem,field}` / `Stalled{subsystem}`，实现 `std::error::Error`)；`Subsystem` trait 新增默认 `validate() -> Result<(),WorldError>`(默认 `Ok(())`，零开销、不影响普通 `step` 路径)；`World` 新增 `step_checked` / `step_skipping_checked`，在 `step_skipping` 全部动作后对所有子系统跑 `validate`，首个 `Err` 即短路返回。
  - 数值积分子系统覆写 `validate`(带 `num_traits::Float` bound，扫描 pos/vel/rot/ang_vel/quat/force 的 `is_finite()`)：`phy-fluid`(pos/vel)、`phy-rigid`(pos/vel/ang_vel/quat)、`phy-granular`(pos/vel)、`phy-soft`(pos/vel/force)。其余子系统(optics/field/solid)沿用默认 `Ok(())`。
  - `phy-io` 因 `unarchive_world` 构造这些子系统，`save/load_world_json` + `save/load_world` 的泛型边界补上 `num_traits::Float`(不影响 `f64`/`f32` 调用方)。
  - `phy-ffi`:新增 `phy_world_step_checked(w,dt) -> i32`：0=健康 / -1=空指针或 panic / 2=NonFinite(NaN/Inf，世界停该帧) / 3=Stalled(某子系统一帧未推进)。业务(游戏/防战建模)在“数据必须有限才能喂渲染/下游模型”时优先用此接口，失败后可 `phy_world_destroy` 释放并回滚到最后已知良好状态。
  - 验证:`phy-core` 2 个单测(看门狗捕获 NaN 子系统 / 有限世界通过) + `phy-ffi` 1 个单测(健康世界 200 步全 0 / 空指针 -1)全过。
- **L4-3 浏览器 W7 数值一致性(CPU 端可验证 + 浏览器端手动核对)**:
  - 主机可验证部分:`crates/phy-demo/tests/w7_routing.rs` 新增 `w7_skip_routing_matches_full_step`——构造耦合世界，对比 (A) 全 CPU `world.step` 与 (B) W7 同款路由(手动 step 流体子世界 + `step_skipping` 跳过其 CPU step 但保留 `couple`)。断言二者流体粒子位置在 **1e-12** 内逐位一致，证明 W7 摘出/跳过 fluid 不破坏耦合矩阵、“手动 step 子世界 + step_skipping”等价于整步。**已通过**(0.02s)。
  - 浏览器端(需人工跑，本机无 WebGPU adapter):在支持 WebGPU 的 Chrome 加载 `phy-demo-web`(`wasm32 + feature=gpu`)，按 `W7` 运行时接管流体/颗粒 step，控制台应打印 `gpu_self_test` / `optic_self_test` / `caustics_self_test` / `sph_self_test` / `granular_self_test` 的 PASS 与各内核数值一致性摘要。确认 GPU 与 CPU 数值一致后，GPU 路径才可对外宣称可用(否则仅 CPU 库可信)。
- **交付**:性能可量化 + 大场景不静默崩溃(NaN 看门狗可捕获并回滚) + W7 路由结构已 CPU 验证 + GPU 数值验证步骤文档化。

### 库化业务的 GPU 约束(关键约束,避免误解)
- **引擎的 GPU 后端仅限 Web Demo**(`wasm32 + feature=gpu`,浏览器 WebGPU):A 档(SPH 密度/受力、颗粒 PBD 接触、光学逐像素 trace、焦散)已上 GPU,W7 可运行时接管流体/颗粒 step。
- **库化交付的 Rust/C 库默认是 CPU(rayon 并行)确定性实现,不含 GPU**:① 桌面 wgpu 在本机 MinGW 8.1 链接器崩溃(`corrupt .drectve`,M3 决策放弃),走 rayon CPU;② §5.7.5 明确排除桌面 GPU。
- **刚体/关节/破碎/CCD/车辆/软体布料本就不适合 GPU**(强顺序依赖或规模太小,B/C 档矩阵),做 GPU 收益低。
- **若业务确需 GPU,两条路**:(a) 换非 MinGW 工具链(Linux / macOS / **MSVC VS2019+**)可重新启用 wgpu 桌面后端——数值内核已与 wgsl 一一对应,只需把 `GpuContext` 从 WebGPU 适配到 wgpu 原生;(b) 业务侧自带 GPU 框架,以已验证的 CPU 数值内核为参考实现自写 compute shader。

### 重数值回归的运行方式
- L1/L2 的 8 个重负载集成测试(SPH/刚体耦合 200 步)在 debug 下较慢,已标记 `#[ignore]`,**默认 `cargo test` 跳过**,不拖慢单元测试。
- 完整数值回归(确定性 + 稳定性)用 release + ignored 运行:
  `cargo test --release -p phy-demo -- --ignored`
  (release 下 SPH 提速约 10–50×,整套 < 1 分钟)。

### 当前测试力度盘点(2026-08-10,含 L1/L2 后)
- 全 workspace 单测 ≈ 120+,全部绿灯,0 失败。分布:phy-core 12 / phy-demo 10(E2E 耦合) + 8(ignored 重负载回归) / phy-rigid ~50 / phy-soft 20 / phy-field ~23 / phy-fluid 18 / phy-granular 5 / phy-solid 4 / phy-io 3 / phy-optics 7 / phy-demo-web 1 / **phy-math 8(原 0)**。
- 强项:每个里程碑带单测 + 端到端 `World` 耦合测试 + **L1 全局稳定性回归(无 NaN/子系统不丢/时钟单调)** + **L2 数值确定性(重复运行逐位一致 + 存档重放一致)**。
- 缺口(剩余):① GPU 数值一致性仅完成 CPU 端路由验证 + 浏览器端手动核对步骤,缺自动化 CI(需真实 WebGPU adapter)。
- 结论:作为 Rust 库**核心物理、耦合、稳定性、确定性、C ABI(L3)、性能基线(L4-1)、NaN 看门狗(L4-2)、W7 路由(L4-3)、用户向文档/doctest(README + 4 处可运行示例)均已可信**,可对外(尤其非 Rust 业务)稳定供货;GPU 为 Web 限定/特定工具链可解锁的附加项。

