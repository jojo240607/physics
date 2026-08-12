# 后续计划：游戏级通用引擎补齐 + GPU 默认策略

> 2026-08-12 建立。目标：把当前"确定性 + 多物理耦合 + 跨平台"引擎补到**通用游戏级**，
> 并在不破坏确定性的前提下让密集计算**默认走 GPU 省 CPU**。

---

## 一、四项补齐计划（对标 Box2D/PhysX 游戏级入场券）

### D1 — 关节扩充（价值最高，入场券）✅ 已完成(2026-08-12)
| 子项 | 说明 |
|---|---|
| `Hinge`（铰链） | ✅ 局部轴对齐 + 沿轴自由旋转（约束 3 平动 + 2 转动）。车辆车轮 / 门 / 齿轮前提 |
| `Prismatic`（滑块） | ✅ 沿轴 1 平动自由（约束 2 垂直平动 + 3 转动锁死）。液压杆 / 抽屉 / 活塞 |
| `Weld`（焊点） | ✅ 锁死全部 6 DOF（3 平动 + 3 转动角对齐）。把碎片焊回 / 强化结构 |
| `Motor`（马达） | ✅ Hinge 沿轴角速度驱动 + Prismatic 沿轴线速度驱动（目标速度 + 最大力/扭矩） |
| 求解扩展 | ✅ 关节求解升级为**含角冲量**：转动约束用 `inv_inertia_world` 有效质量，点约束含角速度项（Box2D 同款逐轴求解） |
| 测试 | ✅ `weld_joint_keeps_relative_pose_and_syncs_angular_velocity` / `hinge_joint_with_motor_rotates_around_aligned_axis` / `prismatic_joint_with_motor_slides_along_axis`（约束收敛 + 锚点/轴对齐 + Motor 驱动 + 动量守恒），72 测试全绿 |

### D2 — 碰撞器扩充（Capsule / Heightfield / Compound 全部完成 ✅ 2026-08-12）
| 子项 | 说明 |
|---|---|
| `Capsule` | ✅ 线段 + 半径，沿局部 y 轴。`support_local`/`contains_local`/`bounding_sphere_r`/`set_inertia_from_shape` 全实现 |
| narrow-phase（Capsule） | ✅ 解析快速路径：`capsule_sphere` / `capsule_capsule` / `capsule_box`（法线由第一个参数指向第二个，含盒内推出分支）；raycast/ccd/fracture 补全 |
| **发现** | ⚠️ 既有 **GJK 对"平滑体 + 尖角(Box)"组合不稳健**(迭代发散,判相交失败;Sphere/Box 因走快速路径不受影响,Convex-Convex 若走 GJK 需注意)。Capsule 已走解析路径规避。 |
| `Heightfield`（高度场） | ✅ 静态地形 `nx×nz` 格点高度 + 双线性插值查询;`heightfield_vs_body` 窄相支持 Sphere/Box/Capsule(法线由地形指向动态体,垂直向上);raycast 待补。测试 `sphere_rests_on_heightfield_terrain`(球落地形停表面) |
| `Compound`（复合） | ✅ `Shape::Compound{SubShape[]}`,每子形状 = shape + 局部 offset/quat;support 取各子沿 dir 最远、contains 任一子含 p、inertia 平行轴合并;窄相 **递归逐子形状**(sub_body 继承父速度/角速度含 offset 贡献),raycast 递归取最近;破碎 mesh 合并。测试 `compound_body_rests_on_ground`(复合体落地停地面不穿透) |
| 测试 | ✅ 3 个：`capsule_support_on_axis` / `capsule_contains_and_bounds` / `capsule_rests_on_ground_via_gjk_epa`(落地不穿透、静止) |

### D3 — 求解器增强（warm-start 已完成，块求解待做）
| 子项 | 说明 |
|---|---|
| Warm-starting（热启动） | ✅ 2026-08-12:`ContactConstraint` 加 `warm_started`,`RigidWorld` 加跨帧 `contact_impulses` 缓存(仅保留当帧活跃 pair, 防膨胀);窄相重建从缓存恢复法向/切向冲量初值,`solve_velocity` 迭代前施加。5 层高堆叠在 warm-start 下收敛稳定(0.5/1.5/2.5/3.5/4.5),是 B1 已知限制"滑动接触能量注入"的主修法之一 |
| 块求解器（block solve） | ✅ 2026-08-12：`solve_velocity` 法向 + 两个切向摩擦方向耦合进 **3×3 有效质量矩阵**(`build_k3` 由 `K=Σ (1/m)I + I⁻¹(r rᵀ−|r|²I)` 投影到 (n,t1,t2) 基),在**单一迭代循环**内联合求解(替代原"先解完法向再解摩擦"两循环);摩擦锥每步基于最新法向冲量夹紧,静止堆叠/斜坡更稳。切向基 `tangent_basis(n)` 现算(Contact 只存法向)。清理了原文件里 `build_k3` 的重复定义 |
| 关节角度求解 | ✅ D1 已实现（多维 λ 用逆有效质量矩阵） |
| 测试 | ✅ `warm_start_stabilizes_tall_stack`(5 层堆叠稳定 + 缓存非空) / `stacked_boxes_converge_without_jitter` / `box_lands_on_ground_no_penetration` / `resting_box_falls_asleep_and_stays_put` 等, 81 phy-rigid + 全 workspace 绿 |

### D4 — 角色控制器（已完成）/ ragdoll（待做）
| 子项 | 说明 |
|---|---|
| `CharacterController` | ✅ 2026-08-12:`crates/phy-rigid/src/character_controller.rs`,kinematic 胶囊体,`update(world,dt,move_dir,want_jump)`:水平移动(归一化×speed) + 垂直重力累积(`vel_y` 跨帧)/跳跃 + `slide` 与场景所有体做胶囊碰撞检测并沿法线推出(落地 grounded/撞墙/爬坡);位置由 CC 直接控制,body vel=0 避免 step 双重积分 |
| Ragdoll 装配 | ✅ 2026-08-12：`crates/phy-rigid/src/ragdoll.rs`,`RagdollBuilder` 给定根位置/朝向/质量把 10 个胶囊肢体(头/躯干/上臂×2/前臂×2/大腿×2/小腿×2)用 Ball(颈/肩/髋) + Hinge(肘/膝,沿 y 轴屈伸) 串成被动布偶,一次性注入 `RigidWorld`;导出 `Ragdoll`/`RagdollLimb`/`RagdollParams` |
| 测试 | ✅ `character_falls_to_ground_and_grounded` / `character_can_jump` / `character_moves_horizontally` / `ragdoll_assembles_with_correct_counts_and_pose` / `ragdoll_falls_under_gravity_and_stays_connected` / `ragdoll_joints_keep_limbs_attached`(颈/膝锚点分离 <0.25), 84 phy-rigid + 全 workspace 绿 |

**执行顺序**：D1 → D2 → D3 → D4（依赖链：关节 → 碰撞器 → 求解器稳 → 角色/布偶）。

---

## 二、GPU 默认策略（省 CPU，不破坏确定性）

### 2.1 可直接用 GPU 的（已逐粒子验证）
| 子系统 | GPU 入口 | 真机实测 | 省 CPU 收益 |
|---|---|---|---|
| SPH 流体 | `step_sph_gpu` | 46.6k@105fps | ~60% CPU |
| 颗粒 PBD | `step_granular_gpu` | 10k@433fps | ~90% CPU（24.8ms→2.3ms） |
| 光学渲染 | `render_camera_gpu` / `render_caustics_gpu` | 逐像素/射线 | 仅 sphere 场景（遇 convex 回退 CPU） |

### 2.2 不能直接 GPU 的（硬限制）
- **刚体/关节/破碎**：顺序依赖 λ 迭代（island 内逐约束），当前 CPU 确定性实现。GPU 化是 PhysX 级大工程，且破坏"逐位确定性"（GPU 浮点不保证）。
- **软体/布料/绳索 PBD**：与颗粒同构可 GPU，但**目前未写 GPU 内核**。

### 2.3 默认 GPU 策略（已实现 ✅ 2026-08-12）
运行时 `GpuStrategy` 开关（定义在 `phy-demo-web/src/gpu/mod.rs`，`Default = Auto`）：
- `Auto`（默认）：检测到 adapter → **SPH/颗粒走 GPU，刚体/关节走 CPU**；无 adapter 全回退 CPU。
- `ForceCpu`：强制全 CPU（确定性 / 离线回放）。`GpuContext::init_with` 直接返回专门错误，调用方回退 `frame()`。
- `ForceGpu`：强制 GPU（无 adapter 时 `init_with` 返回明确错误，便于 CI/调测暴露问题）。

**前置硬性改造（已完成）**：
- `GpuContext::init()` 现在**一次性预编译全部 5 个 compute pipeline**（square/optic/caustic/sph 双 entry/granular 三 entry）+ 常驻 bind group layout/pipeline layout，存于 `GpuPipelines`，常驻于 `GpuContext.pipelines`。
- 各 `render_*_gpu` / `step_*_gpu` 不再重建 pipeline/shader，每帧只重建按输入尺寸变化的 data buffer + bind group，**复用常驻 pipeline**（`pass.set_pipeline` 取 `ctx.pipelines.*`），只在 dispatch + 回读。
- Web `DemoApp` 增加 `gpu_strategy` 字段 + `set_gpu_strategy` / `get_gpu_strategy`，`frame_gpu` 按策略惰性 `init_with`；`ForceCpu` 或缺 adapter 时自动走纯 CPU 帧，确定性不破坏。
- 顺带修复 `flatten_scene` 对 `Shape` 新变种（Capsule/Heightfield/Compound）的非穷尽匹配（原仅覆盖 Sphere/Box/Convex → 编译失败；现对 GPU 不支持的 4 种均回退 CPU）。

**落地文件**：`phy-demo-web/src/gpu/mod.rs`（`GpuStrategy` + `GpuPipelines` + `GpuContext::init/init_with`）+ `phy-demo-web/src/lib.rs`（`set_gpu_strategy` + `frame_gpu` 策略路由）。

**待办**：`gpu_perf` 复测（pipeline 持久化后应显著更快，待有 adapter 的真机/CI 环境验证）。

---

## 三、里程碑落点与验收
- D1–D4 各自带 `#[test]` 回归 + `World` 端到端，纳入 `cargo test --workspace`。
- GPU 策略默认 `Auto`，全测试无 adapter 时全回退 CPU，**确定性不破坏**（`determinism.rs` 全绿）。
- GPU 性能再测：`gpu_perf` 复测（pipeline 持久化后应显著更快）。

## 四、优先级建议
1. **D1 关节扩充**（游戏向价值最高，求解器可扩展）—— ✅ 已完成。
2. **GPU pipeline 持久化 + `GpuStrategy` 默认 Auto**（省 CPU 可交付能力）—— ✅ 已完成（2026-08-12 真机复测：见下表）。

   **§2.3 真机复测结果（真实 WebGPU adapter，wgpu 30 / Vulkan / NVIDIA Quadro P2200）**
   工具链 `stable-x86_64-pc-windows-msvc --release` 跑 `examples/gpu_perf.rs`（3 预热 + 20 计时帧，单帧 ms）：

   | 场景 | n | GPU ms/frame | fps | vsCPU(f64 单线程) |
   |---|---|---|---|---|
   | SPH   sph_1k  | 1000  | 0.749 | 1334.6 | 2.1x |
   | SPH   sph_10k | 9261  | 1.501 | 666.2  | 4.5x |
   | SPH   sph_50k | 46656 | 7.133 | 140.2  | —    |
   | GRAN  gran_1k | 1000  | 0.439 | 2279.0 | 6.9x |
   | GRAN  gran_5k | 5000  | 1.158 | 863.5  | 10.8x |
   | GRAN  gran_10k| 10000 | 1.150 | 869.5  | 21.6x |

   - **GRAN 10k 由管线未持久化时的 ~1.88 ms 降到 1.150 ms/帧（≈1.6×）**，证明 §2.3 的 `GpuPipelines` 常驻缓存生效（每帧不再 `create_compute_pipeline`）。
   - 全部场景满足 30fps 与 60fps 预算；SPH 50k 140fps 远超实时。
   - 复测同时修复了两个真实 bug：`GpuPipelines.sph` / `GpuPipelines.granular` 是 `mk_pipeline("...", "main")` 冗余管线，但其 WGSL 没有 `main` entry（SPH 用 `density_main`/`force_main`、GRAN 用 `clear_main`/`contact_main`/`apply_main`），导致 `GpuContext::init` 在真实 adapter 上 panic；已删除这两个死字段，渲染路径本就用 `build_sph_pipelines` / `build_granular_pipelines` 的常驻版本。
   - 注：GLSL→WGSL 内核全量审批（M3）仍待做；M1 adapter 验收 + G2 性能基准已在真机跑通。
3. D2 → D3 → D4（D2 ✅、D3 块求解 ✅、D4 ragdoll 装配 ✅ 均已完成；四项补齐计划全部收口）。
