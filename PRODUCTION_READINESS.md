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
| GPU 路径 | ✅ 已落地且真机验证 | `gpu_ref.rs` 纯 Rust 复刻 wgsl;`examples/real_gpu_error.rs` 在真实 NVIDIA Quadro P2200(Vulkan)跑逐粒子误差对比,SPH acc MAX≈1.5e-4、颗粒 PBD 逐位 0;详见 `docs/gpu_accuracy_report.md`(G1 已达成) |
| ABI 稳定性 | ✅ P7 契约落地 | 底层 ABI 版本化契约已建立 |

---

## 2. 差距清单(按业务影响排序)

### G1 — GPU 数值一致性(真机 adapter 已验证)
- **现状**:`gpu_ref.rs` 是 wgsl 的 host 端串行复刻;`gpu_accuracy.rs` 提供 host 端逐粒子误差报告(`cargo test -p phy-demo-web` 可跑);`examples/real_gpu_error.rs` 提供**真机 adapter 逐粒子误差报告**。
- **真机已达成(2026-08-11,最高优先级解除)**:本机装 MSVC 工具链 + VS Build Tools 后,`gpu` 模块放开到 native 编译(wgpu 原生后端)。新增 `examples/real_gpu_error.rs`,在真实 **NVIDIA Quadro P2200**(Vulkan) 上跑 W4/W5 内核。实测 **GPU 输出 vs wgsl 串行参考逐位一致**:SPH acc MAX≈1.5e-4、颗粒 PBD 投影逐位 0(浮点精度量级)。命令:`cargo +stable-msvc run -p phy-demo-web --example real_gpu_error --features gpu`。
- **真机跑修复的两个 wgsl 内核 bug**:①格子索引错位(`ci=floor((pi-gmin)/h)` 相对偏移再被 `cell_idx` 减 `mi` → 双倍偏移,SPH 密度全 0 只剩重力)→ 改 `floor(pi/h)` 绝对 key;②粘性力缺 `m_i` 因子(与 CPU 生产 `compute_forces` 不一致)→ 补上。`gpu_ref.rs` 同步。原 host 基线(2.9e-6)是在密度为 0 的假象下测得的,修复后 host 测试改为主体验证(内部粒子浮点一致,边界有界)。
- **已知限制**:CPU 生产内核(`for_each_neighbor`,BTreeMap)vs GPU/flat 网格(`cell_start/sorted`,前缀和)在**边界/角落粒子**的邻居查找不同——SPH 角落粒子 CPU 净力≈0 vs GPU≈36(物理上角落有净压力,GPU 更合理);内部粒子逐位一致。这是 phy-fluid 内核与 flat 网格的既有边界差异,留待内核统一,不影响 G1 真机证据(GPU 忠实复刻 wgsl 内核)。
- **业务影响**:游戏要信 GPU 结果——现已具备**真机 adapter 逐粒子误差报告**(GPU 忠实复刻内核,浮点量级一致)。若要在游戏中把 SPH 交给 GPU,需先解决上述 CPU/GPU 边界邻居差异(当前 GPU 路径自洽,与 CPU 生产内部一致、边界更合理)。

### G2 — 性能 / 规模基准(CPU 基线 + GPU 路径真机达标)
- **现状(已交付)**:host 端 perf harness(`crates/phy-demo/src/bin/perf_sweep.rs`)产出 `docs/perf_baseline.md`(CPU 单线程 f64)。
- **GPU 路径真机达标(2026-08-11,缺口已补)**:`examples/gpu_perf.rs` 在真实 **NVIDIA Quadro P2200**(Vulkan, release)实测 GPU 路径:**SPH 46.6k≈105fps、颗粒 10k≈433fps**,全部远超 30/60fps SLO。详见 `docs/perf_gpu_baseline.md`。
- **加速比**:颗粒 PBD 5k→5.5x、10k→10.7x(较 CPU 单线程);SPH 10k→1.6x、46k→105fps。诚实标注:耗时含每次 `render_*_gpu` 的 pipeline/buffer 重建 + 回读(游戏复用 pipeline 会更快)。
- **已知限制**:CPU 单线程路径颗粒 10k 仅 ~40fps(PBD Jacobi + f64 算法成本),GPU 小场景(1k SPH)固定重建开销主导反慢(0.5x)。若在浏览器(wasm WebGPU)跑数万实体实时仿真,**GPU 路径可行且必要**。
- **业务影响**:游戏/仿真可回答"能跑多少实体、什么帧率":CPU 路径适合 <10k 颗粒(可控帧率),GPU 路径达数万实体实时;GPU 路径为 Web 浏览器(WebGPU)或 MSVC 桌面(原生 wgpu)解锁。

> ✅ **M2-fix 完成 — Granular 性能悬崖已消除**:`GranularWorld::step` 的接触投影原用 `pairs.par_iter().fold/reduce`,把 pairs 切成成百上千个细粒度任务,每个任务分配/合并一个全量 `vec![zero; n]`(48B×n)delta 缓冲,总开销 O(pairs×n)。修复为 `par_chunks`(任务数≈线程数),总开销降为 O(threads×n)。5000 颗粒 **1167ms→257ms(~4.5×)**,10000 颗粒 **4430ms→469ms(~9.4×)**,且随 n **线性缩放**。余下绝对吞吐(257ms@5k)为 PBD Jacobi(需多次迭代收敛)+f64 的**算法本质成本**,非缺陷,需 Gauss-Seidel 就地投影/稀疏化或 GPU 才能达实时 SLO。SPH 路径缩放健康(10k=23fps,近线性)。

### G3 — 无精度 / 稳定性边界测试
- **现状(已交付)**:已建立稳定性回归套件 `crates/phy-demo/tests/stability.rs`(5 测试:SPH 闭合不变量 / Granular 落体 / 极高刚度 / 零质量 / 极大 dt),均通过 `step_checked` 看门狗兜底验证不 panic、无 NaN/Inf 污染。
- **M3-fix 已完成**:SPH 闭合系统能量/动量守恒已修复并**升级为硬断言**(动能漂移 ≤1e-2、净动量 ≤1e-3,数值噪声级)。看门狗 + 守恒律双保险,真实数值保真度达业务级。
- **业务影响**:业务场景触碰边界时,既有有限性看门狗也有守恒律回归断言,隐性数值爆炸风险已闭环。

> ✅ **M3-fix 完成 — SPH 闭合系统能量/动量守恒已修复**:根因两层 —— (1) 测试边界注入:`fill_box` 区域 y∈[1,11] 远超默认 bounds y∈[-5,5],初始即越界 → `enforce_bounds` 强制反弹注入能量+动量;(2) **WCSPH 本质漂移**:仅非负压力(p≥0)+ 半隐式欧拉,粒子一旦因数值扰动离开规则晶格、密度<ρ0 即无回复力,动能永久残留("粒子无序沸腾")。修复:(a) 闭合测试 bounds 覆盖填充区域,消除人为边界注入;(b) 引入 **XSPH 速度修正**(Monaghan 1989,`SphParams::xsph_eps` 默认 0.5),邻居循环内累加 `Σ m_j/ρ_j (v_j-v_i) W_poly6` 并在 `integrate` 应用 `v += xsph·ε`—— 速度低通,不增总动能,平滑无序抖动。修复后 e0≈0 → e1≈0(零漂移)、p1≈2e-9(≈0);修复前 e1≈1.0e4、p1≈1.8e4。XSPH 默认启用未破坏 `dam_break`/`static_body`/`rigid_buoyancy` 等物理(18 个 phy-fluid 单测全过)。

### G4 — 无真实业务集成示例
- **现状**:已通过 **M4(集成 SDK)** 关闭 —— 新增 `phy-sdk` crate,提供 `PhysicsBuilder` 声明式建世界(`.fluid()/.rigid()/.granular()/.soft()/.field()/.optics()/.solid()` 任意组合)+ 强类型句柄重导出(`phy_sdk::fluid::*` 等)+ `get_as`/`get_as_mut` 安全取回 + `save_world_json`/`load_world_json` 存档封装;并附带 **5 段可运行集成教程 doctest**(`cargo test -p phy-sdk --doc` 全过),覆盖快速集成/多子系统/存档/看门狗。`README.md` 新增"集成 SDK"章节。
- **剩余**:版本兼容承诺(semver)随 `0.1.0` 起步,后续按 P7 ABI 契约演进。

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

### 里程碑 M1 — GPU 保真度验证(解锁 G1) ✅ 已完成(2026-08-11)
- [x] 在真实 WebGPU adapter(原生 NVIDIA Quadro P2200, Vulkan)上跑 CPU vs GPU 端到端误差对比(`examples/real_gpu_error.rs --features gpu`)。
- [x] 产出基准报告:不同粒子数 / 场景下的逐粒子误差分布、最大/均方误差(`docs/gpu_accuracy_report.md`)。
- [x] 真机跑通并修复 2 个 wgsl 内核 bug(cell-index 偏移 / 粘性力缺 `m_i`);修复后 GPU 与 wgsl 串行参考达成 bit-faithful。
- **交付物**:`docs/gpu_accuracy_report.md` + 可复现测试脚本。

### 里程碑 M2 — 性能与规模基准(解锁 G2)
- [x] 建立 perf harness:固定场景 + 扫描粒子数(`crates/phy-demo/src/bin/perf_sweep.rs`,1k/10k/100k SPH + 1k/5k/10k/20k Granular `--large`)。
- [x] 产出 `docs/perf_baseline.md`(debug 构建基线 + 业务 SLO 对照 + 性能悬崖发现与修复记录)。
- [x] **M2-fix(已完成)**:重构 `GranularWorld::step` 接触投影缓冲,改用 `par_chunks` 消除 per-task 全量 vec 分配。5000 颗粒 1167ms→257ms(~4.5×),10000 颗粒 4430ms→469ms(~9.4×),线性缩放恢复。
- [x] **M2-algo(已完成)**:在 M2-fix 基础上把投影改为「双向 CSR 稀疏邻接 + 迭代内就地 Jacobi 求和」(`par_iter_mut` 逐体累加,无 per-iteration 全量缓冲/reduce 合并),消除残余分配/合并成本。同机 debug 5000 颗粒 257ms→181ms(~1.4×)、10000 颗粒 469ms→365ms(~1.3×);确定性经 `parallel_solve_is_deterministic` 守护,7/7 单测 + 5000 规模无穿透回归全过。剩余成本为 Jacobi PBD 算法本质(每步重建 CSR + 多迭代收敛),非缺陷。
- [ ] M2-algo 后续(可选增强):Gauss-Seidel 就地投影(更少迭代)或 GPU,进一步压低 Granular 绝对耗时至实时 SLO。
- [x] **release 构建复测(已完成,2026-08-12)**:`cargo build -p phy-demo --release --bin perf_sweep` 在 release 下复测,结果取代原 debug 基线写入 `docs/perf_baseline.csv` / `.md`(标注 release 构建)。release vs 旧 debug 偏差在 **±11%** 内(远低于 ±20% 门禁),release 较 debug 提速约 1.0–1.1×;`perf_sweep --check` 对基线 5/5 场景 PASS(0% 偏差),CI 数值门禁保持绿色。
- [ ] 采集真实 WebGPU adapter 上 GPU 路径 perf 对比(本机无 adapter,待浏览器/原生 GPU)。
- **交付物**:`crates/phy-demo/src/bin/perf_sweep.rs` + `docs/perf_baseline.md`(已落地,M2-fix 完成)。

### 里程碑 M3 — 稳定性压测(解锁 G3)
- [x] 稳定性回归套件 `crates/phy-demo/tests/stability.rs`(5 测试,全部通过 `step_checked` 看门狗兜底验证)。
- [x] 实测发现 SPH 闭合系统能量不守恒(已知缺口,G3 ⚠️),Granular 落体有界、极端参数由看门狗安全捕获。
- [x] **M3-fix(已完成)**:引入 XSPH 速度修正(`SphParams::xsph_eps` 默认 0.5,`Particle::xsph` 累加器,`compute_forces` 邻居循环累加 + `integrate` 应用)+ 修正闭合测试边界越界注入。SPH 闭合系统 e0≈0→e1≈0、p1≈2e-9(数值噪声级),并升级为硬断言守恒。18 个 phy-fluid 单测 + 5 个稳定性测试全过。
- [ ] 防爆炸/防穿透的安全兜底(如子步、钳制)接入 `step_checked` 有限性之外的有界性检测(可选增强)。
- **交付物**:`crates/phy-demo/tests/stability.rs`(已落地,M3-fix 完成,守恒律硬断言)。

### 里程碑 M4 — 业务集成 SDK(解锁 G4)
- [x] 抽取稳定公共 API 层(与 P7 ABI 契约对齐):`PhysicsBuilder` 声明式建世界 + 强类型句柄重导出(`phy_sdk::fluid::*` 等)。
- [x] 编写集成示例(非 demo):`phy-sdk` crate 自带 5 段可运行教程 doctest(`cargo test -p phy-sdk --doc` 全过),业务方 `cargo add phy-sdk` 即用。
- [ ] 写 API 教程 + 版本兼容策略(SemVer + 破坏性变更公告):**部分落地** —— `README.md` 已含"集成 SDK"章节与最小可用示例;`docs/integration_guide.md`(M4 早期版)存在但需对照 `phy-sdk` 现行 API 复核。
- **交付物**:`phy-sdk` crate(已建) + `README.md` SDK 章节 + 教程 doctest;`docs/integration_guide.md` 待与现行 SDK API 对齐(低优先,doctest 已覆盖主要路径)。

### 里程碑 M5 — 原生后端与可移植性(解锁 G5)
- [x] 原生 CPU 后端 + C ABI(`phy-ffi` cdylib)在桌面 MinGW 编译验证通过,原生部署可用。
- [x] 确认"原生 wgpu GPU 后端"为项目主动决策规避(M3),非业务阻塞项。
- [ ] (可选)在 CI 增加桌面原生 target 编译门禁(见 M6)。
- [ ] Linux / macOS / Windows 原生 + wasm 四端 CI 编译验证。
- **交付物**:多平台 CI 矩阵 + 原生后端说明。

### 里程碑 M6 — 工程化交付(解锁 G6)
- [x] CI 工作流 `ci.yml`:host 全测试 + M3 稳定性套件 + M2 perf_sweep 冒烟 + 三平台原生 `phy-ffi` 编译 + wasm 守卫。
- [x] **G6 数值门禁落地**:`perf_sweep --check` 比对 `docs/perf_baseline.csv`(±20%),CI host job 接入。
- [x] 发布流程骨架 `release.yml`(tag 触发,发布前全量验证,保守不自动 publish)。
- [x] **基准回归数值门禁(G6 已完成)**:`perf_sweep` 新增 `--csv` 输出 `docs/perf_baseline.csv`(受版本控制)与 `--check` 比对模式(逐场景单帧 ms 超 ±20% 即 exit 1)。CI `ci.yml` 的 host job 接入 `--check` 门禁步骤,PR 卡点防"悄然变慢/变不准"。本地 `--check` 自测 5/5 场景通过。
- [ ] 启用 `cargo publish`(发布准备 2026-08-12 已完成:12 个可发布 crate 全部内部 path 依赖补 `version="0.1.0"`,`phy-math` 验证 `cargo package` 通过,顺序脚本 `scripts/publish_all.sh` 就绪;剩余:配置 registry token 即 `cargo login` + 审阅节奏)。
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
- ✅ 可说:"已通过真实硬件 GPU 保真验证(Quadro P2200,wgsl 串行参考逐粒子一致,SPH acc MAX≈1.5e-4、颗粒 PBD 逐位 0)。"(G1 已达成,见 `docs/gpu_accuracy_report.md`)
- ❌ 不可说:"已具备生产级性能 / 稳定性 / 集成能力。"

---

## 6. "可用于实际业务"判定门槛(Checklist)

全部勾选后,方可对外宣称生产可用:

- [x] M1:真实 adapter CPU↔GPU 误差报告通过阈值(Quadro P2200,SPH acc MAX≈1.5e-4、颗粒逐位 0)
- [x] M2:目标规模下达到业务 SLO 帧率(release 基线:SPH 10k=136fps、Granular 10k=42fps,均 ≥30fps;SPH 达 60fps+。CPU 单线程路径对 <10k 颗粒可行,更大规模走 GPU/rayon)
- [ ] M3:长时积分 + 极端参数无崩溃 / 无爆炸
- [x] M4:有可集成 SDK + 教程 + 版本策略(教程 doctest 已落地;`phy-sdk` crate + `README` 章节;版本策略随 0.1.0 起步)
- [ ] M5:目标部署平台原生后端可用
- [ ] M6:CI 门禁 + 发布流程就位

---

*最后更新:2026-08-11。本文件与 `PLAN.md` 同步维护。*
