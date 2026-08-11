# M2 性能与规模基线(host CPU, f64)

- 生成: runtime | 大规模(100k): 未含(加 --large) | 精度: f64 | 后端: 单线程 host CPU

| 场景 | 粒子数 | 单帧 ms | 估算 FPS |
|---|---|---|---|
| SPH fluid | 1000 | 6.558 | 152.5 |
| SPH fluid | 10000 | 47.019 | 21.3 |
| PBD granular | 1000 | 27.363 | 36.5 |
| PBD granular | 5000 | 181.009 | 5.5 |
| PBD granular | 10000 | 365.554 | 2.7 |

## 解读

- 以上为 **单线程 CPU 参考实现** 的吞吐下限;真实 WebGPU(wgsl)路径应显著更快,
但本机无 adapter,故 host 基线即为当前可测参考。
- 业务 SLO 参考:实时仿真通常要求 ≥ 30fps(单帧 ≤ 33ms),游戏要求 ≥ 60fps(≤ 16.6ms)。
- 若某规模单帧耗时超 SLO,需启用 GPU 路径或并行化(rayon)才能落地业务。

## M2-algo 增量(2026-08-11)

在 M2-fix 已消除 per-task 全量 delta 缓冲分配悬崖的基础上,M2-algo 进一步把
`GranularWorld::step` 的约束投影从 `par_chunks().fold().reduce()`(每迭代分配/合并
全量 `vec![zero; n]` delta 缓冲)改为 **双向 CSR 稀疏邻接 + 迭代内就地 Jacobi 求和**
(`par_iter_mut().enumerate()` 逐体累加,无中间缓冲、无 reduce 合并)。确定性(字节级
一致,S8)经 `parallel_solve_is_deterministic` 回归守护。

同机 debug 构建对比(数值受 debug 优化惩罚,仅看相对):

| 规模 | M2-fix | M2-algo | 改善 |
|---|---|---|---|
| 1000 | — | 27.4ms | — |
| 5000 | 257ms | **181ms** | ~1.4× |
| 10000 | 469ms | **365ms** | ~1.3× |

绝对吞吐再降 ~30%,但剩余成本已属 **Jacobi PBD 本质**(每步重建 CSR + 每迭代 O(pairs)
算术 + 需多次迭代收敛),非缺陷。进一步压到实时 SLO 需 Gauss-Seidel 就地投影(更少迭代)
或 GPU,属后续可选增强,不在本次范围。SPH 路径未改动,数值与先前一致。
