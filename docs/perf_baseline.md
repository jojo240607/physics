# M2 性能与规模基线（host CPU, f64）

> 生成方式：`cargo run -p phy-demo --bin perf_sweep`（默认扫描，不含 `--large`）
> 精度：f64 | 后端：单线程 host CPU（rayon 并行仅在 `step` 内部邻域遍历启用）
> 测量：每规模预热 5 帧 + 测 30 帧平均单帧耗时（release 未启用，debug 构建偏慢）

## 基线数据（debug 构建）

| 场景 | 粒子数 | 单帧 ms | 估算 FPS |
|---|---|---|---|
| SPH fluid | 1000 | 6.6 | 151.2 |
| SPH fluid | 10000 | 43.5 | 23.0 |
| PBD granular | 1000 | 50.1 | 19.9 |
| PBD granular | 5000 | 1166.9 | 0.9 |

## 关键发现

1. **SPH 缩放健康**：1k→10k 约 6.6× 耗时增长（接近线性邻域搜索预期），单线程 10k 仍 23fps，GPU 化后预期可再提升 1–2 数量级。
2. **Granular 性能悬崖**：5000 颗粒单帧 1.17s（0.9fps），相较 1000 颗粒 50ms 劣化 **23×**。根因是 `GranularWorld::step` 的 Jacobi+rayon `par_iter().fold/reduce` 实现：每个并行任务分配/合并一个全量 `vec![zero; n]` 的 per-body delta 缓冲（48B×n），在 n=5000 时每帧产生海量任务级全量向量分配与拷贝，远超实际计算量。这是**实现缺陷，非算法本质**——Gauss-Seidel 就地投影或稀疏 CSR 对存储可消除该开销。
3. **Granular 10000+ 未纳入扫描**：实测 >5000 即进入秒级/帧，超出业务实时阈值（30fps=33ms），故默认扫描封顶 5000；`--large` 仅扩展 SPH 到 100k（SPH 路径正常）。

## 业务 SLO 对照

- 实时仿真：≥30fps（单帧 ≤33ms）
- 游戏交互：≥60fps（单帧 ≤16.6ms）

| 场景 | 1k | 10k | 结论 |
|---|---|---|---|
| SPH | ✅ 达标 | ⚠️ 临界（23fps） | 中等规模需 GPU 化或并行化 |
| Granular | ❌ 未达标（20fps） | ❌ 崩溃（0.9fps） | **必须先修实现缺陷（M2-fix）才能谈业务可用性** |

## 待办

- [ ] **M2-fix**：重构 `GranularWorld::step` 的接触投影缓冲（消除 per-task 全量 vec 分配），目标 5000 颗粒重回 <100ms/帧。
- [ ] M2-large：在修复后补 100k Granular 扫描（需先修缺陷）。
- [ ] release 构建复测（当前 debug 数据偏保守，release 通常 2–5× 提速）。
- [ ] 真实 WebGPU adapter 上的 wgsl 路径 perf 对比（本机无 adapter，待浏览器/原生 GPU 目测）。
