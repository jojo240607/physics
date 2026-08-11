# M2 性能与规模基线(host CPU, f64)

- 生成: runtime | 大规模(100k): 未含(加 --large) | 精度: f64 | 后端: 单线程 host CPU
- **数值门禁源**:`docs/perf_baseline.csv`(本表由同次 release 运行生成,仅供人读)。`perf_sweep --check` 在 **release** 下比对 CSV,任一场景单帧 ms 超 ±20% 判回归(G6)。更新基线:`cargo run --release -p phy-demo --bin perf_sweep -- --csv docs/perf_baseline.csv --out docs/perf_baseline.md`(须确认性能改进确为有意后再提交 CSV)。

| 场景 | 粒子数 | 单帧 ms | 估算 FPS |
|---|---|---|---|
| SPH fluid | 1000 | 1.594 | 627.2 |
| SPH fluid | 10000 | 6.795 | 147.2 |
| PBD granular | 1000 | 3.022 | 330.9 |
| PBD granular | 5000 | 12.470 | 80.2 |
| PBD granular | 10000 | 24.792 | 40.3 |

## 解读

- 以上为 **单线程 CPU 参考实现** 的吞吐下限;真实 WebGPU(wgsl)路径应显著更快,
但本机无 adapter,故 host 基线即为当前可测参考。
- 业务 SLO 参考:实时仿真通常要求 ≥ 30fps(单帧 ≤ 33ms),游戏要求 ≥ 60fps(≤ 16.6ms)。
- 若某规模单帧耗时超 SLO,需启用 GPU 路径或并行化(rayon)才能落地业务。
