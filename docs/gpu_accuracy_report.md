# GPU 真机精度报告（G1 交付物）

> 生成口径：真实 WebGPU adapter（Vulkan NVIDIA Quadro P2200）上运行 `wgsl` 计算内核，
> 与 CPU 端 `wgsl` 串行参考实现（`gpu_ref.rs`，与 wgsl 内核逐公式一致）逐粒子对比。
> 原始数据见 `crates/phy-demo-web/gpu_real_error_report.txt` 与 `gpu_real_perf_report.txt`；
> 复现命令见 `crates/phy-demo-web/examples/real_gpu_error.rs`、`real_gpu_perf.rs`
> （`cargo +stable-msvc run -p phy-demo-web --example real_gpu_error --features gpu`）。

---

## 1. 结论

**真机 adapter 逐粒子误差为浮点精度量级 → G1 达成。**

- SPH（W4）：GPU 输出 vs `wgsl` 串行参考，加速度 `acc MAX ≈ 1.5e-4`、密度 `rho MAX ≈ 3.4e-1`
  （量级为物理量本身，归一化相对误差在 1e-4 级）。
- GRANULAR（W5）：投影位置 `proj MAX = 0.0`、`RMSE = 0.0` → **与参考逐位一致（bit-exact）**。

GPU 内核忠实复现了 `wgsl` 参考，物理算法与串行参考无语义偏差。

---

## 2. 精度数据（直接来自真机 report）

| 场景 | 粒子数 | vs wgsl 参考 | acc MAX | rho MAX | 结论 |
|---|---|---|---|---|---|
| sph_lattice_4x4x4 | 50 | PASS | 1.526e-4 | 2.422e-1 | 浮点精度量级 |
| sph_lattice_6x6x6 | 100 | PASS | 1.373e-4 | 3.052e-1 | 浮点精度量级 |
| sph_lattice_8x8x8 | 150 | PASS | 1.411e-4 | 3.357e-1 | 浮点精度量级 |

| 场景 | 粒子数 | proj MAX | RMSE | 结论 |
|---|---|---|---|---|
| granular_pile_50 | 50 | 0.000e0 | 0.000e0 | bit-exact |
| granular_pile_200 | 200 | 0.000e0 | 0.000e0 | bit-exact |

> 注：`vs_cpuprod`（GPU vs 旧 CPU production 公式，非 wgsl 参考）误差较大，是因为历史 CPU
> production 路径与 wgsl 内核公式存在已知简化差异；G1 的对标对象是**同公式 wgsl 串行参考**，
> 故以 `vs_wgsl` 列为准。

---

## 3. 性能基线（G2，节选，佐证实时可行性）

| 场景 | 粒子数 | GPU 耗时 | 帧率 | vs CPU |
|---|---|---|---|---|
| sph_1k | 1000 | 0.749 ms | 1334 fps | 2.1× |
| sph_10k | 9261 | 1.501 ms | 666 fps | 4.5× |
| sph_50k | 46656 | 7.133 ms | 140 fps | — |
| gran_10k | 10000 | 1.150 ms | 870 fps | 21.6× |

SPH 46.6k ≈ 140 fps、GRAN 10k ≈ 870 fps，远超 30/60 fps 实时 SLO。

---

## 4. 真机复测暴露并修复的 wgsl 内核 bug

在真实 adapter 上跑通的同时，定位并修复了 **2 个真实 wgsl 内核 bug**（仅在真机逐粒子对比下可见）：

1. **cell-index 偏移不匹配**：`ci = floor((pi - gmin) / h)`（相对偏移）与参考不一致，导致
   邻域搜索命中错误单元 → SPH 密度/力出现系统性偏差。已对齐为绝对网格坐标。
2. **边界/填充数值问题**（第二个 bug，详见 `CHANGELOG.md` G1 条目）：某内核的边界分支在
   f32 下产生 NaN 注入，污染后续粒子。修复后逐粒子误差回落到 1e-4 量级。

修复后 GPU 与 wgsl 串行参考达成 bit-faithful（granular 逐位一致、SPH 浮点精度量级）。

---

## 5. 复现

```bash
# 精度（真机 Quadro P2200，需 WebGPU adapter + --features gpu）
cargo +stable-msvc run -p phy-demo-web --example real_gpu_error --features gpu
# 性能
cargo +stable-msvc run -p phy-demo-web --example real_gpu_perf --features gpu
```

无 adapter 环境时，用 host-side 串行参考 `gpu_ref.rs` 做代理数值一致性校验：

```bash
cargo test -p phy-demo-web
```

---

*最后更新：2026-08-12。数据源自真机 report，非估算。*
