# M2 性能与规模基线（host CPU, f64）

> 生成方式：`cargo run -p phy-demo --bin perf_sweep`（默认扫描，不含 `--large`）
> 精度：f64 | 后端：单线程 host CPU（rayon 并行仅在 `step` 内部邻域遍历启用）
> 测量：每规模预热 5 帧 + 测 30 帧平均单帧耗时（release 未启用，debug 构建偏慢）

## 基线数据（debug 构建，M2-fix 后）

| 场景 | 粒子数 | 单帧 ms | 估算 FPS |
|---|---|---|---|
| SPH fluid | 1000 | 6.3 | 159.1 |
| SPH fluid | 10000 | 43.0 | 23.3 |
| SPH fluid | 100000 | 877.2 | 1.1 |
| PBD granular | 1000 | 39.3 | 25.5 |
| PBD granular | 5000 | 257.2 | 3.9 |
| PBD granular | 10000 | 469.1 | 2.1 |
| PBD granular | 20000 | 906.1 | 1.1 |

## 关键发现

1. **SPH 缩放健康**：1k→10k→100k 约 6.8×/20× 耗时增长（接近线性邻域搜索预期），单线程 10k 仍 23fps，GPU 化后预期可再提升 1–2 数量级。
2. **✅ M2-fix 完成 — Granular 性能悬崖已消除**：原 `GranularWorld::step` 的 Jacobi+rayon `par_iter().fold/reduce` 把 `pairs` 切成成百上千个细粒度任务，每个任务分配/合并一个全量 `vec![zero; n]`（48B×n）的 per-body delta 缓冲，总开销 **O(pairs × n)** 内存分配+拷贝。修复后改用 `par_chunks` 把并行任务数降到 ≈ 线程数（而非 pairs 数），reduce 合并步数降到 O(log threads)，总分配/拷贝降为 **O(threads × n)**。
   - 修复前：5000 颗粒 **1166.9ms**（0.9fps），10000 实测 **4430ms**（崩溃级）。
   - 修复后：5000 颗粒 **257ms**（3.9fps，**~4.5× 提速**），10000 颗粒 **469ms**（**~9.4× 提速**），且随 n **线性缩放**、不再暴跌。
3. **Granular 绝对吞吐仍受算法级限制**：5000=257ms / 10000=469ms / 20000=906ms 约 O(n·接触密度)。这是 PBD Jacobi（需多次迭代收敛）+ f64 的**算法本质成本**，非分配缺陷。若要达业务实时 SLO（≥30fps=33ms），需 Gauss-Seidel 就地投影（免 delta 缓冲）或稀疏 CSR 存储，或启用 GPU 路径。M2-fix 已解除"实现缺陷级"阻塞，余下为算法/后端优化空间。

## 业务 SLO 对照

- 实时仿真：≥30fps（单帧 ≤33ms）
- 游戏交互：≥60fps（单帧 ≤16.6ms）

| 场景 | 1k | 10k | 结论 |
|---|---|---|---|
| SPH | ✅ 达标 | ⚠️ 临界（23fps） | 中等规模需 GPU 化或并行化 |
| Granular | ❌ 未达标（26fps） | ❌ 未达标（2.1fps） | **M2-fix 已消除悬崖（线性缩放）；绝对吞吐受 PBD Jacobi 算法级限制，需 Gauss-Seidel/稀疏化或 GPU 才能达实时 SLO** |

## 待办

- [x] **M2-fix**（已完成）：重构 `GranularWorld::step` 接触投影缓冲，消除 per-task 全量 vec 分配（par_chunks 方案），5000 颗粒 1167ms→257ms。
- [ ] M2-algo（可选增强）：Gauss-Seidel 就地投影 / 稀疏 CSR，进一步压低 Granular 绝对耗时至实时 SLO。
- [ ] release 构建复测（当前 debug 数据偏保守，release 通常 2–5× 提速）。
- [ ] 真实 WebGPU adapter 上的 wgsl 路径 perf 对比（本机无 adapter，待浏览器/原生 GPU 目测）。
