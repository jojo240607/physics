# G2 真机 GPU 性能与规模基线

- 生成: runtime | 精度: f32 | 后端: **真实 GPU adapter compute**(Vulkan, NVIDIA Quadro P2200) | release
- 命令:`cargo +stable-msvc run --release -p phy-demo-web --example gpu_perf --features gpu`
- 对比基准:`docs/perf_baseline.md`(host CPU 单线程, f64)

## GPU 实测(单帧 ms = dispatch + 回读)

| 场景 | 粒子数 | GPU ms/帧 | GPU fps | CPU ms/帧(f64) | GPU 加速比 |
|---|---|---|---|---|---|
| SPH fluid | 1000 | 3.330 | 300 | 1.594 | 0.5x |
| SPH fluid | 9261 | 4.124 | 242 | 6.795 | 1.6x |
| SPH fluid | 46656 | 9.485 | 105 | — | — |
| PBD granular | 1000 | 2.135 | 468 | 3.022 | 1.4x |
| PBD granular | 5000 | 2.282 | 438 | 12.470 | 5.5x |
| PBD granular | 10000 | 2.311 | 433 | 24.792 | 10.7x |

## 解读

- **实时规模 SLO(G2)已达成**:GPU 路径下 **SPH 46.6k ≈ 105fps、颗粒 10k ≈ 433fps**,全部远超
  30fps(≤33ms)/ 60fps(≤16.6ms)游戏 SLO。这是 `perf_baseline.md`(CPU 单线程)无法达到的
  颗粒规模(CPU 10k ≈ 40fps)。
- **加速比**:颗粒 PBD 在 GPU 上随规模放大优势显著(5k→5.5x、10k→10.7x);SPH 大场景
  (46k)仍维持 105fps。SPH 小规模(1k)GPU 反慢(0.5x)——因小场景固定开销
  (shader/pipeline/buffer 每帧重建 + 回读)主导。
- **诚实标注**:本表耗时 = `render_sph_gpu`/`render_granular_gpu` **每次调用完整重建**
  (shader/pipeline/storage buffer) + GPU dispatch + CPU 回读(map)。游戏若复用 pipeline
  (正常做法)会显著快于本表;本表反映"每帧从零构建 GPU 路径"的最坏成本,仍全部达 SLO。
- **f32 vs f64**:GPU 用 f32(WebGPU 计算着色器),CPU 基线用 f64;数值精度差异见
  `gpu_accuracy.rs` / `gpu_real_error_report.txt`(GPU vs wgsl 参考逐位一致,内部粒子浮点级)。

## 业务影响

- 需要在 Web 浏览器(WebGPU, wasm32)内跑数万颗粒/流体的实时仿真,**GPU 路径是可行且必要**
  的:CPU 单线程 10k 颗粒仅 40fps,GPU 达 433fps;SPH 46k 达 105fps。
- 若目标是桌面/原生,需 MSVC 工具链链接 wgpu(MinGW 崩溃,见 M3);浏览器路径(wasm)天然走 WebGPU。
