# 生产可用性路线图 (PRODUCTION READINESS)

> 本文档评估本物理引擎从"工程原型 / 早期阶段"迈向"可投入实际业务"所需的差距与里程碑。
> 它是 `PLAN.md` 的姊妹文档:PLAN 描述**愿景与架构决策**,本文档描述**距离可用还差什么、按什么顺序补**。

---

## 0. 一句话结论

当前状态可以对外表述为:

> **"一个具有真实物理算法、工程化验证、架构良好的物理引擎项目(技术原型 / 早期可用阶段)。"**

但**不能**表述为"已可投入实际业务"。判定门槛见 §6。

---

## 1. 已达成的现状(基线)

| 维度 | 状态 | 证据 |
|---|---|---|
| 架构 | ✅ 模块化 Rust workspace | `crates/` 下 phy-core / phy-math / phy-rigid / phy-fluid / phy-granular / phy-optics / phy-soft / phy-solid / phy-demo / phy-demo-web |
| 流体算法 | ✅ 真实 SPH | poly6 / spiky_grad / visc_lap 核,密度-压力-力计算(`phy-fluid`) |
| 颗粒算法 | ✅ 真实 PBD | Jacobi 接触投影(`phy-granular`,W5) |
| 多层级验证 | ✅ host 测试 + wasm 守卫 | `cargo test` 全通过;`wasm-cross-check.py --gpu` 编译守卫通过 |
| GPU 路径 | ⚠️ 已落地但仅代理验证 | `gpu_ref.rs` 纯 Rust 复刻 wgsl;真实 adapter 误差对比**未跑** |
| ABI 稳定性 | ✅ P7 契约落地 | 底层 ABI 版本化契约已建立 |

---

## 2. 差距清单(按业务影响排序)

### G1 — GPU 数值一致性仅代理验证(最高优先级)
- **现状**:`gpu_ref.rs` 是 wgsl 的 host 端串行复刻,验证"wgsl 自身契约"(finite / 密度守恒 / 不穿透)。
- **问题**:wgsl 是 CPU 生产实现的**简化移植**(压力系数 `(p_i+p_j)/(2ρ_j)`、粘性 `(k+shear_min)` 线性,与 CPU 幂律不同)。
- **缺口**:真实 WebGPU adapter 上的 **CPU↔GPU 误差对比从未跑过**(本机桌面 wgpu 链接崩溃,环境限制,待浏览器人工目测)。
- **业务影响**:业务级可用性要求真实硬件上的数值保真度证据,目前缺失。

### G2 — 无性能 / 规模基准
- **现状(部分交付)**:已建立 host 端 perf harness(`crates/phy-demo/src/bin/perf_sweep.rs`)并产出 `docs/perf_baseline.md`,扫描 1k/10k(SPH)+ 1k/5k(Granular)。
- **缺口**:真实 WebGPU adapter 上的 GPU 路径 perf 对比缺失(本机无 adapter);Granular 在 >1k 量级存在**实现级性能悬崖**(见下)。
- **业务影响**:游戏/仿真首先要回答"能跑多少实体、什么帧率";Granular 现状下**不可用于 >1000 实体的业务场景**。

> ⚠️ **M2 实测发现 — Granular 性能悬崖**:`GranularWorld::step` 的 Jacobi+rayon 接触投影在 5000 颗粒时单帧 **1167ms(0.9fps)**,相较 1000 颗粒 50ms 劣化 23×。根因是 per-task 全量 `vec![zero; n]` delta 缓冲分配/合并开销,非算法本质。这是**真实生产阻塞项**,必须先修(见 M2-fix)才能谈颗粒业务可用性。SPH 路径缩放健康(10k=23fps,近线性)。

### G3 — 无精度 / 稳定性边界测试
- **现状(部分交付)**:已建立稳定性回归套件 `crates/phy-demo/tests/stability.rs`(5 测试:SPH 闭合不变量 / Granular 落体 / 极高刚度 / 零质量 / 极大 dt),均通过 `step_checked` 看门狗兜底验证不 panic、无 NaN/Inf 污染。
- **缺口**:看门狗只检测非有限值,不检测"有限但爆炸"(能量自发注入);真实数值保真度(能量/动量守恒率)未达业务级。
- **业务影响**:业务场景常触碰边界,未验证即引入隐性崩溃/数值爆炸风险。

> ⚠️ **M3 实测发现 — SPH 闭合系统能量不守恒**:无重力静止初始条件下,SPH 1000 步后动能从 e0≈0 自发增长到 **e1≈1.0e4**(压力不对称 + 初始重叠导致数值能量注入)。看门狗不报错(值有限),但闭合系统不应自发产生能量。这是**真实求解器缺陷**,需在 M3-fix 处理(symplectic 积分 / 压力对称化 / 无初始重叠初始化)。Granular 落体 300 步动能有界(不爆炸),极高刚度/零质量/极大 dt 均由看门狗安全捕获或有限兜底。

### G4 — 无真实业务集成示例
- **缺口**:`phy-demo` / `phy-demo-web` 是演示,不是可嵌入 SDK;无 API 教程、无版本兼容承诺(虽 P7 有底层 ABI 契约)。
- **业务影响**:接入成本高,业务方无法快速评估集成工作量。

### G5 — 跨平台 / 可移植性
- **现状(已决策规避 + 部分验证)**:
  - ✅ **原生 CPU 后端 + C ABI**:`phy-ffi`(cdylib)在桌面 MinGW 工具链编译通过(已验证),业务方可直接把 `.dll`/`.so` 链入 C/C++/Unity/Unreal,走 `phy-core` 的 rayon 并行 CPU 实现,**不依赖 wgpu**。
  - ✅ **Web/WASM 路径**:`phy-demo-web`(wasm32 + gpu)编译守卫通过(`wasm-cross-check.py --gpu`)。
  - ⚠️ **原生 wgpu GPU 后端**:本机 MinGW 链接 wgpu 巨型依赖树崩溃(`corrupt .drectve`),**项目已主动决策放弃**(见 PLAN §M3 / README),业务默认 CPU 后端;GPU 仅经 `phy-demo-web` 的 WebGPU 在浏览器演示。
- **业务影响**:桌面/服务器原生部署可用(CPU + C ABI);仅浏览器场景有 GPU 加速演示。GPU 原生后端非业务阻塞项(刚体/软体本就不适合 GPU)。

### G6 — 无 CI/CD、发布流程、基准回归门禁
- **现状(部分交付)**:已补齐 CI 工作流(`.github/workflows/ci.yml`):
  - `host`:workspace 全编译 + 全测试 + **M3 稳定性套件** + **M2 perf_sweep 冒烟** + release 重测试。
  - `native`:**三桌面平台(ubuntu/windows/macos)编译 `phy-ffi`**(验证 M5 原生 C-ABI 后端)。
  - `wasm`:wasm32 交叉编译守卫。
  - 新增 `release.yml`(tag 触发,发布前全量验证骨架,不自动 publish,保守)。
- **缺口**:尚未接基准回归"数值阈值门禁"(仅冒烟确认能跑,未比对基线 ±20%);发布流程为骨架(未启用 `cargo publish`)。
- **业务影响**:已能防"改了不编译/不测试",但还需补基准数值门禁才能防"悄然变慢/变不准"。

---

## 3. 里程碑路线图

### 里程碑 M1 — GPU 保真度验证(解锁 G1)
- [ ] 在真实 WebGPU adapter(浏览器或原生)上跑 CPU vs GPU 端到端误差对比。
- [ ] 产出基准报告:不同粒子数 / 场景下的逐粒子误差分布、最大/均方误差。
- [ ] 若误差超阈值,回修 wgsl 或统一 CPU/GPU 数学路径。
- **交付物**:`docs/gpu_accuracy_report.md` + 可复现测试脚本。

### 里程碑 M2 — 性能与规模基准(解锁 G2)
- [x] 建立 perf harness:固定场景 + 扫描粒子数(`crates/phy-demo/src/bin/perf_sweep.rs`,1k/10k SPH + 1k/5k Granular)。
- [x] 产出 `docs/perf_baseline.md`(debug 构建基线 + 业务 SLO 对照 + 性能悬崖发现)。
- [ ] **M2-fix(阻塞)**:重构 `GranularWorld::step` 接触投影缓冲,消除 per-task 全量 vec 分配,目标 5000 颗粒重回 <100ms/帧。
- [ ] 修复后补 100k Granular 扫描 + release 构建复测。
- [ ] 采集真实 WebGPU adapter 上 GPU 路径 perf 对比(本机无 adapter,待浏览器/原生 GPU)。
- **交付物**:`crates/phy-demo/src/bin/perf_sweep.rs` + `docs/perf_baseline.md`(已落地,待 M2-fix)。

### 里程碑 M3 — 稳定性压测(解锁 G3)
- [x] 稳定性回归套件 `crates/phy-demo/tests/stability.rs`(5 测试,全部通过 `step_checked` 看门狗兜底验证)。
- [x] 实测发现 SPH 闭合系统能量不守恒(已知缺口,G3 ⚠️),Granular 落体有界、极端参数由看门狗安全捕获。
- [ ] **M3-fix(建议)**:SPH 求解器稳定性改进(symplectic 积分 / 压力对称化 / 无初始重叠初始化),消除闭合系统能量自发注入。
- [ ] 防爆炸/防穿透的安全兜底(如子步、钳制)接入 `step_checked` 有限性之外的有界性检测。
- **交付物**:`crates/phy-demo/tests/stability.rs`(已落地,待 M3-fix 收敛能量守恒)。

### 里程碑 M4 — 业务集成 SDK(解锁 G4)
- [ ] 抽取稳定公共 API 层(与 P7 ABI 契约对齐)。
- [ ] 编写集成示例(非 demo):独立仓库可 `cargo add` 使用。
- [ ] 写 API 教程 + 版本兼容策略(SemVer + 破坏性变更公告)。
- **交付物**:`phy-sdk` crate + `docs/integration_guide.md`。

### 里程碑 M5 — 原生后端与可移植性(解锁 G5)
- [x] 原生 CPU 后端 + C ABI(`phy-ffi` cdylib)在桌面 MinGW 编译验证通过,原生部署可用。
- [x] 确认"原生 wgpu GPU 后端"为项目主动决策规避(M3),非业务阻塞项。
- [ ] (可选)在 CI 增加桌面原生 target 编译门禁(见 M6)。
- [ ] Linux / macOS / Windows 原生 + wasm 四端 CI 编译验证。
- **交付物**:多平台 CI 矩阵 + 原生后端说明。

### 里程碑 M6 — 工程化交付(解锁 G6)
- [x] CI 工作流 `ci.yml`:host 全测试 + M3 稳定性套件 + M2 perf_sweep 冒烟 + 三平台原生 `phy-ffi` 编译 + wasm 守卫。
- [x] 发布流程骨架 `release.yml`(tag 触发,发布前全量验证,保守不自动 publish)。
- [ ] 基准回归数值门禁(perf_sweep 输出与 `docs/perf_baseline.md` 比对 ±20% 阈值)。
- [ ] 启用 `cargo publish`(需配置 registry token + 审阅节奏)。
- **交付物**:`.github/workflows/ci.yml` + `.github/workflows/release.yml`(已落地,待数值门禁)。
- [ ] 基准结果纳入 PR 卡点(性能回归报警)。
- **交付物**:`.github/workflows/` + 发布 runbook。

---

## 4. 优先级建议

```
P0(阻塞业务判定):  M1 (GPU 保真)  →  M2 (性能基准)
P1(稳定性必备):    M3 (稳定性压测)
P2(落地必需):      M4 (集成 SDK)  →  M5 (原生后端)
P3(规模化协作):    M6 (CI/CD)
```

最短路径:完成 **M1 + M2 + M3** 即可对外宣称"技术可用、有保真度与性能证据";
完成 **M4** 后才算"业务可集成";**M5 + M6** 是规模化与多端交付的收尾。

---

## 5. 当前可对外表述的话术

- ✅ 可说:"模块化 Rust 物理引擎,覆盖 SPH 流体 / PBD 颗粒 / 刚体 / 光学,有真实算法与多层级验证。"
- ✅ 可说:"GPU 路径已落地,并提供 host 端 wgsl 等价参考与可编译守卫。"
- ❌ 不可说:"已通过真实硬件 GPU 保真验证。"
- ❌ 不可说:"已具备生产级性能 / 稳定性 / 集成能力。"

---

## 6. "可用于实际业务"判定门槛(Checklist)

全部勾选后,方可对外宣称生产可用:

- [ ] M1:真实 adapter CPU↔GPU 误差报告通过阈值
- [ ] M2:目标规模下达到业务 SLO 帧率
- [ ] M3:长时积分 + 极端参数无崩溃 / 无爆炸
- [ ] M4:有可集成 SDK + 教程 + 版本策略
- [ ] M5:目标部署平台原生后端可用
- [ ] M6:CI 门禁 + 发布流程就位

---

*最后更新:2026-08-11。本文件与 `PLAN.md` 同步维护。*
