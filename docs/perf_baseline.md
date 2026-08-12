# M2 性能与规模基线(host CPU, f64)

- 生成: runtime(release 构建) | 大规模(100k): 未含(加 --large) | 精度: f64 | 后端: 单线程 host CPU

| 场景 | 粒子数 | 单帧 ms | 估算 FPS |
|---|---|---|---|
| SPH fluid | 1000 | 1.827 | 547.4 |
| SPH fluid | 10000 | 7.321 | 136.6 |
| PBD granular | 1000 | 3.299 | 303.1 |
| PBD granular | 5000 | 13.482 | 74.2 |
| PBD granular | 10000 | 23.749 | 42.1 |

## 解读

- 以上为 **单线程 CPU 参考实现** 的吞吐下限;真实 WebGPU(wgsl)路径应显著更快,
但本机无 adapter,故 host 基线即为当前可测参考。
- 业务 SLO 参考:实时仿真通常要求 ≥ 30fps(单帧 ≤ 33ms),游戏要求 ≥ 60fps(≤ 16.6ms)。
- 若某规模单帧耗时超 SLO,需启用 GPU 路径或并行化(rayon)才能落地业务。
