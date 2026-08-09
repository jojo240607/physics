//! W4 GPU 数据扁平化:把 `FluidWorld<f32>` 当前状态导出为 GPU 友好的纯 f32/primitive 结构。
//!
//! 不依赖 wgpu(保持 `phy-fluid` 为纯算法 crate);`phy-demo-web` 的 `gpu` 模块消费
//! `SphFlatData`,把字段上传到 WebGPU buffer 并跑 wgsl 内核(复刻 `compute_density_pressure`
//! + `compute_forces`)。本文件只定义数据结构(`FluidWorld::to_gpu_flat` 实现在 `world.rs`,
//! 因需访问私有字段)。
//!
//! - `pos`/`vel`/`scalar` 三个 storage buffer:每个粒子一个 `vec4`
//!   (`scalar = (rho, p, mass, material)`),输出 `out_rho_p`/`out_acc_mu` 同布局。
//! - `cell_start`/`sorted` 为网格前缀和(`Grid::to_flat` 产出)。
//! - 核系数在 wgsl 端用 `h` 重算(与 `Kernels` 一致),故只传 `h` + 物性参数。

/// GPU 友好的 SPH 扁平数据(纯 f32 / primitive,无泛型)。
pub struct SphFlatData {
    pub n: usize,
    pub pos: Vec<[f32; 4]>,
    pub vel: Vec<[f32; 4]>,
    pub scalar: Vec<[f32; 4]>,
    pub cell_start: Vec<i32>,
    pub sorted: Vec<i32>,
    pub grid_min: [i64; 3],
    pub nc: [usize; 3],
    pub h: f32,
    pub rest_density: f32,
    pub stiffness: f32,
    pub visc_k: Vec<f32>,
    pub visc_n: Vec<f32>,
    pub shear_min: f32,
    pub gravity: [f32; 3],
}
