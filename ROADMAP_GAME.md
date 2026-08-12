# 简单小游戏可用性推进计划 (ROADMAP_GAME)

> 2026-08-12 建立。目标:把现有物理引擎从"算法完整但散落"推进到"能顺畅做仿真 +
> 简单小游戏"的可落地状态。本文档以**代码真实现状**为准(已核对 `phy-rigid` 源码),
> 修正 PLAN.md §11.2 / PRODUCTION_READINESS.md §5-§6 中因进度漂移而过期的描述。

---

## 0. 现状核对(已落地,非待补)

经代码核对,以下 PLAN/PRODUCTION 文档中标注为"待补/缺"的项**实际已完成**:

- **角动力学**:`Body` 含 `rot`/`ang_vel`/`inv_inertia_local`,姿态积分 `q += 0.5·(0,ω)⊗q·h`,
  接触含角冲量,单测 `free_spin_integrates_attitude` / `off_center_impulse_spins_free_body` 覆盖。
- **Capsule 角色控制器 (D4)**:`crates/phy-rigid/src/character_controller.rs` 已实现
  (kinematic 胶囊 + 落地/跳跃/水平移动 slide 修正),3 个单测全过。
- **Warm-start (D3)**:`solver.rs` 已实现 Box2D 同款 3×3 块求解器 + 跨帧接触冲量缓存
  (`RigidWorld::contact_impulses`),`step` 接通 warm-start,缓解"滑动接触能量注入"。
- **GPU 真机误差 (G1)** / **GPU 性能达标 (G2)** / **sleeping+摩擦收敛 (B1)** /
  **kinematic+sensor (B2)** / **场景 DSL (B3)** / **island 并行 (B4)** / **碰撞层 (B5)** /
  **profiler (C3)** / **Unity/Unreal 绑定 (C2)** / **quickstart (C4)** 均已完成。
- **发布准备 (C1)**:12 crate 已补 `version` + `scripts/publish_all.sh` 就绪,待 `cargo login` 凭据。

---

## 1. 真实缺口(按"简单小游戏"体验收益排序)

| 编号 | 缺口 | 性质 | 体验收益 |
|---|---|---|---|
| **R1** | 角色控制器(D4)未接入 demo / SDK / 场景 DSL | 已写好但未"接好" | 🔴 高:没有可视化演示与一行式接入,游戏作者不能直接用 |
| **R2** | `docs/integration_guide.md` 与现行 `phy-sdk` API 未对齐 | 文档 | 🟠 中:降低接入门槛 |
| **R3** | `docs/gpu_accuracy_report.md` 交付物未落盘(M1 卡点) | 文档 | 🟡 低-中:闭环 G1 证据 |
| **R4** | PLAN §11.2 / PRODUCTION_READINESS §5-§6 过期措辞未修正 | 文档 | 🟡 低:消除文档自相矛盾 |
| **R5** | `cargo publish` 待凭据(C1/M6 末项) | 流程 | 🟡 低:发布到 crates.io |

> 注:滑动接触能量注入(B1 已知限制)已由 D3 warm-start 缓解,不再是"简单游戏"阻塞项;
> 若后续做高速传送带/长坡滑行仍发现抖动,再单独评估恢复系数模型升级(TGS),不列入本期。

---

## 2. 执行顺序

### 阶段 A(本期,小成本低收益)
- **A1 (R1)** ✅ 已完成(2026-08-12):角色控制器接入 `phy-demo` / `phy-sdk` / 场景 DSL。
  - A1-1 (DSL):`SceneDesc` 加 `character_spawn: Option<CharacterSpawn<T>>`,`load_scene_json`
    落地 `CharacterController`;`to_scene_json` 反向写回;新增 roundtrip 单测(落地 240 步无 NaN)。
  - A1-2 (SDK):`phy-sdk::rigid` 导出 `CharacterController`;新增 doctest(取 `RigidSubsystem`
    的 `rigid.world`、创建角色、240 步落地断言),6 个 doctest 全过。
  - A1-3 (demo):`DemoMode::Character` 模式,键盘 WASD 移动 + 空格跳跃(撞墙自动 slide),
    `keys_down` 集合持续驱动;`render_character` 用青色高亮角色体;`phy-demo-web` 同步接入
    (循环模式 + `c` 键)。全 workspace 构建通过,`phy-rigid`/`phy-sdk` 单测+doctest 全绿。
  - 验证:`cargo build`(全 workspace)Finished;`cargo test -p phy-rigid -p phy-sdk` 85 单测 + 8 doctest 全过。
- **A2 (R2)** ✅ 已完成(2026-08-12):`docs/integration_guide.md` 新增「`phy-sdk` 推荐入口」
  (§2.1)、「角色控制器」(§2.2)、「关卡 DSL」(§2.3)三节,核心抽象表加入 `phy-sdk` 与
  `CharacterController`,并声明"以 doctest 为单一事实源"避免再次漂移。
- **A3 (R3)** ✅ 已完成(2026-08-12):补 `docs/gpu_accuracy_report.md`,汇总 G1 真机数据
  (Quadro P2200、SPH acc MAX≈1.5e-4、颗粒逐位 0、两个 wgsl bug 修复记录),基于真实
  `gpu_real_error_report.txt` / `gpu_real_perf_report.txt`,非估算。

### 阶段 B(收尾,文档/流程)
- **B1 (R4)** ✅ 已完成(2026-08-12):修正 PLAN §11.2 B2「capsule 待补」→「已完成(D4,接入
  demo/SDK/DSL)」;PRODUCTION_READINESS §5 话术「❌ 不可说 GPU 保真」→「✅ 真机已达成」;
  §6 checklist M1 勾选;P0 表 GPU 路径改 ✅ 真机验证;M1 里程碑标记已完成。
- **B2 (R5)**:`cargo login` + `scripts/publish_all.sh` 发布(需用户凭据,不主动执行)。

---

## 3. 判定标准(本期完成即可宣称)

> "可用于科研仿真与简单小游戏(物理沙盒 / 解谜 / 物理解谜 / 确定性回放竞速);
> 带角色移动的 3D 小游戏经 R1 接入后可直接做;仍非商用 3A 级引擎(求解器鲁棒性 /
> 大规模性能 / 工具链与商用引擎有代差)。"

---

*最后更新:2026-08-12。本文件与 PLAN.md / PRODUCTION_READINESS.md 同步维护。*
