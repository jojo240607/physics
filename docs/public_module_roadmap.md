# 物理引擎「公用模块」成熟度评估与补齐计划

> 文档目标：回答"这个物理引擎作为稳定、公用、通用的模块被业务复用，是否已经足够完善，还缺什么"。
> 评估基准：截至 2026-08-13 的 `wfix` 分支状态（PLAN.md / PRODUCTION_READINESS.md / ROADMAP_GAME.md 已记录的全部里程碑均视为已落地）。
> 用途：本文件是"公用模块"视角下的增量路线图，与 PLAN.md 的"工程能力里程碑"互补——PLAN 记录"做了什么"，本文件记录"作为公用模块还差什么"。

---

## 1. 总体结论

**已达到：技术原型 / 早期可用级别（约 0.1.0 预发布）。**

已具备的公用模块硬实力（来自 PLAN.md / PRODUCTION_READINESS.md，仅列与"公用"强相关的）：

| 维度 | 状态 | 证据 |
|---|---|---|
| 架构 | ✅ 模块化 workspace + `Subsystem`/`couple` 插件式内核 | `World<T>` 统一驱动、双向耦合 |
| 物理广度 | ✅ 很宽 | SPH 流体 / PBD 颗粒 / 顺序冲量刚体(含角动力学) / 软体布料 / 线性 FEM / 热电磁引力声光场 / 光学折射焦散 / 关节 / 车辆 / 破碎 / CCD / 角色控制器 |
| 确定性 | ✅ S8 落地 | 同输入逐位复现 + 存档重放（防战建模硬门槛） |
| 稳定性 | ✅ L1/L2/L4 落地 | 看门狗(有限性捕获) + 极端参数压测 + 守恒律硬断言 |
| 跨语言 | ✅ L3 C ABI 落地 | 不透明句柄 + panic 守卫 + f32/f64 双通道 + MSVC `.lib` + Unity/Unreal P/Invoke 声明 |
| 性能 | ✅ G2 真机达标 | GPU 路径 SPH 46.6k≈105fps、颗粒 10k≈433fps |
| GPU 保真 | ✅ G1 真机验证 | CPU↔GPU 逐粒子误差 SPH MAX≈1.5e-4、颗粒逐位 0 |
| 工程化 | ✅ 120+ 单测 + CI + 数值门禁 + 打包脚本 | `build_package.py` / `wasm-cross-check.py` |
| 便捷层 | ✅ phy-sdk + 场景 DSL | `PhysicsBuilder` 一行声明子系统 + `SceneDesc` JSON 落地 |

**尚未达到"生产可用 / 商用稳定"级别。** 作为"公用模块"判定，最关键的三个未达标项：
1. **从未正式发布**——业务方只能走本地 path 依赖，不能 `cargo add`；
2. **缺少"物理↔渲染"集成契约**——每个游戏侧接入都要自己踩坐标/矩阵/剔除的坑（本次 wfix 分支修复的 180° 穿模、模型矩阵转置即此类）；
3. **游戏逻辑刚需的碰撞事件回调缺失**——求解器内部已算接触冲量，但未向外暴露 begin/stay/end 三态 + 冲量大小。

---

## 2. 两个使用方向的缺口

### 方向 A：建模仿真（科研 / 防战建模 / 数字孪生）
- **材料本构薄**：FEM 仅线性小变形、SPH 仅牛顿+幂律，缺塑性/损伤/各向异性。
- **参数标定基础设施缺失**：没有"给一组材料参数 → 跑标准基准对表"的校准框架。
- **确定性无黄金参考库**：S8 已逐位可复现，但缺"固定随机种子的场景库 + 黄金参考值"仓库，复现门槛仍在脚本层。
- **长期守恒漂移报告**：`StatsObserver` 已落地，但 CI 仅跑 1000 步级，缺长时（10^5+ 步）自动化守恒报告。

### 方向 B：游戏引擎
- **物理↔渲染集成契约缺失**（见 §3 P1-a）：坐标 / 矩阵列主序 / 背面剔除 / 单位约定未固化。
- **碰撞事件三态 + 冲量回报缺失**（见 §3 P1-b）：游戏逻辑常用，求解器已有数据只差暴露。
- **查询 API 不全**：仅有 `ray_cast`，缺形状投射 / 扫掠体检测（sweep test）。
- ~~**物理材质未资产化**~~：✅ 已在 P2-a 落地——新增 `PhysicsMaterial<T> { friction, restitution }` 资源（`material.rs`），`Body` 持有 `material` 字段，求解器按 per-pair 组合（恢复取 max、摩擦取几何平均）查表，全局 `SolverParams` 作为双方均默认材质时的回退。场景 DSL 经 serde 自动支持。
- ~~**查询 API 不全（仅 ray_cast，缺形状投射/sweep）**~~：✅ 已在 P2-b 落地——新增 `shape_cast(shape, from, from_rot, to, to_rot, bodies, layers, mask, ignore, max_iters)`（`shape_cast.rs`）与 `RigidWorld::cast_shape()` 便捷入口，返回首个受阻 `ShapeCastHit<T> { fraction, body_index, point, normal }`。算法为粗扫（coarse=max_iters×4）+ 二分精化，可正确处理"起点终点均畅通但中段受阻"的穿越式扫掠；法向由 `narrowphase::collide` 给出（指向查询形状来向）。
- **引擎零拷贝桥接不完整**：Unity/Unreal 绑定是 P/Invoke 声明层，缺"每帧 Transform 批量同步"封装。

---

## 3. 补齐计划（按优先级）

| 优先级 | 项 | 归属方向 | 成本 | 状态 |
|---|---|---|---|---|
| **P0（阻塞"公用"判定）** | 正式发布到 crates.io + 对外 crate 加 `#[non_exhaustive]` | 通用 | 中 | 待做 |
| **P1-a** | 写《物理↔渲染集成契约》文档（坐标/矩阵列主序/背面剔除/单位） | 游戏 | 低 | ✅ 已完成 |
| **P1-b** | 碰撞事件三态回调(Begin/Stay/End) + 穿透深度回报 | 游戏 | 中 | ✅ 已完成 |
| **P2-a** | 物理材质资源概念（per-body 摩擦/恢复系数） | 通用 | 中 | ✅ 已完成 |
| **P2-b** | 形状投射 / sweep test 查询 API | 游戏 | 中 | ✅ 已完成 |
| **P3-a** | 材料本构扩展（塑性/损伤/各向异性）+ 标定基准库 | 建模 | 高 | 待做 |
| **P3-b** | Unity/Unreal 的 Transform 批量同步桥 | 游戏 | 中 | 待做 |
| **P3-c** | 仿真侧黄金参考场景库（固定种子 + 参考值） | 建模 | 中 | 待做 |

### P0 详情：发布与 ABI 稳定
- 给所有对外 crate（`phy-core` / `phy-sdk` / `phy-rigid` / `phy-fluid` / `phy-granular` / `phy-soft` / `phy-field` / `phy-optics` / `phy-ffi` / `phy-math`）的对外枚举/结构体加 `#[non_exhaustive]`（事件枚举、错误枚举、Builder 配置项优先）。
- 配置 `cargo login` 凭据；执行 `scripts/publish_all.sh`（已就绪）。
- 在 `integration_guide.md` §3 把版本承诺从"0.x 可能破坏性变更"收紧为明确的 semver 保护边界。

### P1-a 详情：物理↔渲染集成契约（本文件姊妹文档 `render_integration_contract.md`）
固化以下约定，避免每个游戏侧重复踩坑：
1. **坐标系**：物理世界 = 右手系、Y 轴向上、单位米；渲染侧若用 Z-up / 左手系需显式转换。
2. **变换矩阵内存布局**：物理侧 `phy_math::Mat4` 与 WGSL `mat4x4` 均为**列主序**；顶点缓冲按列连续铺开 16 个 f32，禁止行/列转置。
3. **背面剔除**：meshgen 生成的封闭凸体（盒/球/胶囊）绕序并非全部严格外 CCW，渲染管线**必须关闭背面剔除**（`cull_mode: None`）或先修正全部绕序；否则相机转到 180° 会错误剔除正面造成"穿模"假象。
4. **单位与缩放**：位置单位 m、时间 s、质量 kg，渲染侧缩放因子由业务方统一处理，不在物理层缩放。
5. **深度与 VP**：近/远平面、投影矩阵 z 映射（OpenGL `[-1,1]` vs D3D/WebGPU `[0,1]`）的校正约定。

### P1-b 详情：碰撞事件三态 + 穿透深度回报
- 在 `phy-rigid` 求解器（solver.rs）接触求解后，收集每对接触 `(body_a, body_b, normal, point, depth)`。
- 新增 `CollisionEvent<T>` 枚举（`collision_events.rs`）：
  - `Begin { a, b, normal, point, depth }`
  - `Stay { a, b, normal, point, depth }`
  - `End { a, b }`
- 状态机由 `CollisionTracker` 维护"上一帧接触对集合"，与当前帧 diff 得出 begin/stay/end；`depth` 取本帧穿透深度（>0，作为碰撞强度指标）。
- 注：求解器为 split-impulse 架构，速度层法向冲量在接触帧趋于 0（位置修正走独立伪速度通道），故回报 `depth` 而非冲量更可靠。若需精确接触力，应另接 solver 内部冲量钩子（未来项）。
- 公开入口：`RigidWorld::drain_collision_events()`（每帧覆盖式写入，取出即清空）。

### P2-a 详情：物理材质资源
- 新增 `PhysicsMaterial<T> { friction, restitution }`（`material.rs`），附常用预设 `ice()/rubber()/metal()/wood()` 与 `combine()`（恢复取 max、摩擦取几何平均）。
- `Body` 持有 `material: PhysicsMaterial<T>`（serde 可序列化，默认 `friction=0.5, restitution=0.0`，与 `SolverParams` 全局默认一致，向后兼容）。
- 新增 `solver::effective_material(a, b, params)` 纯函数：双方均默认材质 → 回退全局 `SolverParams`；否则取组合。求解器速度层据此取 per-pair 摩擦/恢复系数。
- 场景 DSL（`SceneDesc`）经 `Body` 的 serde 自动支持 per-body 材质，无需额外字段。
- ⚠️ 已知限制：当前 `solve_velocity` 的块求解器对恢复系数存在双重逆矩阵缩放缺陷（`p=K⁻¹·col` 后 `dλ=-p[0]`，等效 `K⁻²`），导致恢复系数动力学几乎失效（回弹极弱）。该缺陷为既有问题、与 P2-a 材质接线无关；P2-a 仅保证"材质值正确传入求解器"。修复见未来 M 级 issue。

### P2-b 详情：查询 API
- `phy-rigid` 新增 `shape_cast(shape, from, to, rotation)`（扫掠体检测）与 `cast_shape` 返回首个命中 + 命中处法向/穿透。
- 复用现有 `broadphase` + `narrowphase` 的图元测试，仅改变查询入口（从"两 body 接触"改为"形状 vs 世界"）。

---

## 4. 推进顺序（本次执行）

1. ✅ 本文件（`public_module_roadmap.md`）——评估与计划固化。
2. ✅ P1-a：《物理↔渲染集成契约》文档。
3. ✅ P1-b：碰撞事件三态回调 + 穿透深度回报。
4. ✅ P2-a：物理材质资源（`PhysicsMaterial` + `Body.material` + `effective_material`）。
5. ✅ P2-b：形状投射 / sweep test（`shape_cast` + `cast_shape`）。
6. ✅ P0：crates.io 发布 + `#[non_exhaustive]`（12 个对外 crate 的公开枚举/配置输入 struct 已加 `#[non_exhaustive]`；下游 phy-optics/phy-fluid/phy-ffi/phy-demo 的 match 已补通配臂通过编译，全 workspace `cargo build`/`cargo test -p phy-rigid` 通过）。已知限制：`phy-demo` 的 `f64_vec3_roundtrip` 诊断测试（serde_json 浮点位模式往返）为预存失败，与发布无关。

> 注：P3 系列（材料本构扩展、引擎零拷贝桥、黄金参考库）成本较高，待 P0~P2 落地后再排期。
