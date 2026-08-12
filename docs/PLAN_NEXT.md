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

### D2 — 碰撞器扩充（Capsule 已完成，Heightfield/Compound 待做）
| 子项 | 说明 |
|---|---|
| `Capsule` | ✅ 线段 + 半径，沿局部 y 轴。`support_local`/`contains_local`/`bounding_sphere_r`/`set_inertia_from_shape` 全实现 |
| narrow-phase（Capsule） | ✅ 解析快速路径：`capsule_sphere` / `capsule_capsule` / `capsule_box`（法线由第一个参数指向第二个，含盒内推出分支）；raycast/ccd/fracture 补全 |
| **发现** | ⚠️ 既有 **GJK 对"平滑体 + 尖角(Box)"组合不稳健**(迭代发散,判相交失败;Sphere/Box 因走快速路径不受影响,Convex-Convex 若走 GJK 需注意)。Capsule 已走解析路径规避。 |
| `Heightfield`（高度场） | ⏳ 地形。赛车 / 物理平台刚需 |
| `Compound`（复合） | ⏳ 一个刚体挂多个子碰撞器。机械臂 / 车辆底盘 |
| 测试 | ✅ 3 个：`capsule_support_on_axis` / `capsule_contains_and_bounds` / `capsule_rests_on_ground_via_gjk_epa`(落地不穿透、静止) |

### D3 — 求解器增强（堆叠稳定性）
| 子项 | 说明 |
|---|---|
| Warm-starting（热启动） | 每帧保留上次 λ 作初值，消除静止堆叠抖动（B1 已知限制"滑动接触能量注入"主修法） |
| 块求解器（block solve） | 摩擦 + 法向作一个 2×2 块解，提高稳定性（Box2D 做法） |
| 关节角度求解 | 多维 λ 用逆有效质量矩阵（D1 的求解扩展） |
| 测试 | 高堆叠（10+ 盒）静止无抖动、滑动接触无能量注入 |

### D4 — 角色控制器 / ragdoll
| 子项 | 说明 |
|---|---|
| `CharacterController` | 胶囊体 + 移动 / 爬坡 / 下落，非动力学驱动 |
| Ragdoll 装配 | Ball/Hinge 串起骨骼，Rust 端装配工具 |
| 依赖 | D1（Hinge）+ D2（Capsule）完成后天然支持 |
| 测试 | 角色爬坡不穿透、ragdoll 落地不 NaN |

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

### 2.3 默认 GPU 策略（推荐实现）
运行时 `GpuStrategy` 开关：
- `Auto`（默认）：检测到 adapter → **SPH/颗粒走 GPU，刚体/关节走 CPU**；无 adapter 全回退 CPU。
- `ForceCpu`：强制全 CPU（确定性 / 离线回放）。
- `ForceGpu`：强制 GPU（失败回退 CPU）。

**前置硬性改造（必须）**：
- `render_sph_gpu` / `render_granular_gpu` 目前**每帧从零重建 pipeline/shader/buffer**，开销大。
  → 改为 **持久化复用**（`GpuContext` 持有 pipeline/bind group，帧循环只 dispatch + 回读）。
  否则省下的 CPU 被重建开销抵消。

**落地文件**：`phy-demo-web/src/gpu/strategy.rs`（GpuStrategy + Auto 检测）+ `GpuContext` pipeline 缓存。

---

## 三、里程碑落点与验收
- D1–D4 各自带 `#[test]` 回归 + `World` 端到端，纳入 `cargo test --workspace`。
- GPU 策略默认 `Auto`，全测试无 adapter 时全回退 CPU，**确定性不破坏**（`determinism.rs` 全绿）。
- GPU 性能再测：`gpu_perf` 复测（pipeline 持久化后应显著更快）。

## 四、优先级建议
1. **D1 关节扩充**（游戏向价值最高，求解器可扩展）。
2. **GPU pipeline 持久化 + `GpuStrategy` 默认 Auto**（省 CPU 可交付能力）。
3. D2 → D3 → D4。
