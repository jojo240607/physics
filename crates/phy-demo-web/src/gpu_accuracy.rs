//! M1 — GPU 数值一致性对比 harness(host 端可执行,非门控)。
//!
//! 本模块提供 **CPU 生产实现 vs wgsl 参考实现** 的量化对比框架,作为
//! `PRODUCTION_READINESS.md` 里程碑 M1 的 host 端交付物。
//!
//! # 重要定位(诚实声明)
//!
//! W4/W5 的 wgsl 内核是 CPU 生产实现(`phy-fluid` / `phy-granular`)的**简化移植**:
//! - SPH 压力力系数:wgsl 用 `(p_i+p_j)/(2ρ_j)`,CPU 用对称式 `(p_i/ρ_i² + p_j/ρ_j²)`;
//! - 粘度:wgsl 用线性 `(k+shear_min)`,CPU 用幂律 `k·shear^(n-1)`;
//! - 压力状态方程:wgsl 允许负压 `p = k(rho-ρ0)`,CPU 用 `max(0, k(rho-ρ0))`。
//!
//! 因此本 harness 对比出的差异,**主要来源是"生产 vs 简化移植"的数学路径不同**,
//! 而非"真实 WebGPU adapter 的浮点误差"。真实 adapter 的 CPU↔GPU 误差对比需
//! 在浏览器/原生 wgpu 环境跑通(见 `export_flat_for_adapter` + 对应 JS/CLI 消费者),
//! 本 harness 为其建立**误差基线**与**数据导出接口**。
//!
//! 验收判据(M1 host 部分):
//! 1. 同一 `FlatData` 下,gpu_ref 输出全部 finite(已由 `gpu_ref.rs` 测试覆盖);
//! 2. CPU 生产 step 全程无 NaN/Inf(数值稳定性代理);
//! 3. 量化的"生产 vs wgsl 参考"偏差落于合理区间(文档化,非逐位相等)。

use phy_fluid::{FluidWorld, SphParams};
use phy_granular::world::GranularWorld;
use phy_math::Vec3;

use crate::gpu_ref::{cpu_granular_wgsl_reference, cpu_sph_wgsl_reference};

// ----------------------------------------------------------------------------
// 报告结构(可序列化为 Markdown / JSON)。
// ----------------------------------------------------------------------------

/// 单个场景的对比统计。
#[derive(Debug, Clone)]
pub struct CompareStat {
    pub scene: String,
    pub n_particles: usize,
    /// CPU 生产 step 后单步加速度场与 wgsl 参考加速度场的均方误差。
    pub acc_mse: f32,
    /// 同上,最大绝对误差。
    pub acc_max: f32,
    /// CPU 生产 step 全程是否 finite(无 NaN/Inf)。
    pub cpu_finite: bool,
    /// gpu_ref 参考输出是否全部 finite。
    pub wgsl_finite: bool,
}

impl CompareStat {
    pub fn to_markdown(&self) -> String {
        format!(
            "| {} | {} | {:.4e} | {:.4e} | {} | {} |",
            self.scene,
            self.n_particles,
            self.acc_mse,
            self.acc_max,
            self.cpu_finite,
            self.wgsl_finite
        )
    }
}

/// 完整 M1 报告。
#[derive(Debug, Clone)]
pub struct AccuracyReport {
    pub generated: String,
    pub env: String,
    pub note: String,
    pub stats: Vec<CompareStat>,
}

impl AccuracyReport {
    pub fn to_markdown(&self) -> String {
        let mut s = String::new();
        s.push_str("# M1 GPU 数值一致性报告(host 端代理)\n\n");
        s.push_str(&format!("- 生成时间: {}\n", self.generated));
        s.push_str(&format!("- 环境: {}\n", self.env));
        s.push_str(&format!("- 说明: {}\n\n", self.note));
        s.push_str("| 场景 | 粒子数 | 加速度 MSE | 加速度 MAX | CPU finite | wgsl finite |\n");
        s.push_str("|---|---|---|---|---|---|\n");
        for st in &self.stats {
            s.push_str(&st.to_markdown());
            s.push('\n');
        }
        s.push_str("\n## 解读\n\n");
        s.push_str(
            "本表量化的是 **CPU 生产实现 vs wgsl 简化移植参考** 的单步加速度偏差。\n\
             由于 wgsl 是简化移植(见模块头注释),该偏差主要反映两路数学公式的固有差异,\n\
             而非真实 GPU 浮点误差。真实 adapter(浏览器/原生 wgpu)的 CPU↔GPU 误差对比\n\
             需结合 `export_flat_for_adapter` 导出的 `FlatData` 在 GPU 端重算后回填。\n",
        );
        s
    }
}

// ----------------------------------------------------------------------------
// SPH:CPU 生产 vs wgsl 参考。
// ----------------------------------------------------------------------------

/// 用 `FluidWorld::<f32>` 构建静止晶格场景,对比 CPU step 反推加速度 vs wgsl 参考加速度。
///
/// 注意:CPU 生产在 `step` 内做完密度/压力/力后直接积分,这里用"位置二阶差分"反推
/// 单步加速度近似:`a ≈ (pos_after - pos_before)/dt² - gravity`,再与 gpu_ref 对
/// 同一 `FlatData` 算出的 `acc_mu[0..3]` 对比。两者公式不同,故只做量化、不要求一致。
pub fn probe_sph_cpu_vs_wgsl(scene: &str, n_per: usize, spacing: f32, h: f32) -> CompareStat {
    let mut params = SphParams::<f32>::defaults();
    params.h = h;
    let mut w = FluidWorld::<f32>::new(params);
    // 规则晶格填充(h 决定了晶格质量反算有效)。
    let margin = h * 0.5;
    w.fill_box(
        Vec3::new(-1.0, 0.0, -1.0),
        Vec3::new(1.0, (n_per as f32 - 1.0) * spacing + 0.1, 1.0),
        spacing,
        margin,
    );
    // 先 step 一次以建立网格(否则 to_gpu_flat 邻居为空)。
    let dt = 0.01f32;
    w.step(dt);
    // 导出当前状态给 wgsl 参考。
    let flat = w.to_gpu_flat();
    let n = flat.n;
    let (_, acc_mu) = cpu_sph_wgsl_reference(&flat);

    // CPU 生产反推加速度:再 step 一次,用位置二阶差分。
    let pos_before: Vec<[f32; 3]> = w.particles.iter().map(|p| [p.pos.x, p.pos.y, p.pos.z]).collect();
    w.step(dt);
    let pos_after: Vec<[f32; 3]> = w.particles.iter().map(|p| [p.pos.x, p.pos.y, p.pos.z]).collect();

    let g = [flat.gravity[0], flat.gravity[1], flat.gravity[2]];
    let mut mse = 0.0f32;
    let mut max_err = 0.0f32;
    let mut cpu_finite = true;
    for i in 0..n {
        let mut err2 = 0.0f32;
        for a in 0..3 {
            // 反推 CPU 加速度(减重力,因 gpu_ref 的 acc 已含重力,这里对齐到"合力加速度")。
            let a_cpu = (pos_after[i][a] - pos_before[i][a]) / (dt * dt) - g[a];
            let a_ref = acc_mu[i][a];
            if !a_cpu.is_finite() || !a_ref.is_finite() {
                cpu_finite = false;
            }
            let e = a_cpu - a_ref;
            err2 += e * e;
            max_err = max_err.max(e.abs());
        }
        mse += err2 / 3.0;
    }
    mse /= n as f32;

    let wgsl_finite = acc_mu.iter().all(|m| {
        m[0].is_finite() && m[1].is_finite() && m[2].is_finite() && m[3].is_finite()
    });

    CompareStat {
        scene: scene.to_string(),
        n_particles: n,
        acc_mse: mse,
        acc_max: max_err,
        cpu_finite,
        wgsl_finite,
    }
}

// ----------------------------------------------------------------------------
// Granular:CPU 生产 vs wgsl 参考。
// ----------------------------------------------------------------------------

/// 用 `GranularWorld` 构建重叠球场景,对比 CPU 投影后位置 vs wgsl 参考投影位置。
pub fn probe_granular_cpu_vs_wgsl(scene: &str, n: usize, radius: f32) -> CompareStat {
    let mut gw = GranularWorld::<f32>::new();
    gw.iterations = 4;
    // 网格撒布,留初始间隙。
    gw.fill_grid(n, radius, 1.0, 1.05);
    // 导出 GPU flat(需先 step 一次建邻居?GranularWorld::to_gpu_flat 内部处理)。
    let flat = gw.to_gpu_flat();
    let n_particles = flat.n;
    // wgsl 参考投影。
    let ref_pos = cpu_granular_wgsl_reference(&flat);

    // CPU 生产投影:step 一次。
    let dt = 0.016f32;
    gw.step(dt);
    let mut cpu_pos = Vec::with_capacity(n_particles);
    for g in &gw.grains {
        cpu_pos.push([g.pos.x, g.pos.y, g.pos.z]);
    }

    let mut mse = 0.0f32;
    let mut max_err = 0.0f32;
    let mut cpu_finite = true;
    let limit = n_particles.min(ref_pos.len());
    for i in 0..limit {
        for a in 0..3 {
            let a_cpu = cpu_pos[i][a];
            let a_ref = ref_pos[i][a];
            if !a_cpu.is_finite() || !a_ref.is_finite() {
                cpu_finite = false;
            }
            let e = a_cpu - a_ref;
            mse += e * e;
            max_err = max_err.max(e.abs());
        }
    }
    mse /= (limit * 3) as f32;

    let wgsl_finite = ref_pos.iter().all(|p| p[0].is_finite() && p[1].is_finite() && p[2].is_finite());

    CompareStat {
        scene: scene.to_string(),
        n_particles,
        acc_mse: mse,
        acc_max: max_err,
        cpu_finite,
        wgsl_finite,
    }
}

// ----------------------------------------------------------------------------
// 入口:生成完整报告。
// ----------------------------------------------------------------------------

/// 跑一组预定义场景,产出 M1 host 端报告。
pub fn run_accuracy_report() -> AccuracyReport {
    let mut stats = Vec::new();
    // SPH 静止晶格(不同规模)。
    stats.push(probe_sph_cpu_vs_wgsl("sph_lattice_4x4x4", 4, 0.12, 0.2));
    stats.push(probe_sph_cpu_vs_wgsl("sph_lattice_6x6x6", 6, 0.12, 0.2));
    // Granular 堆积。
    stats.push(probe_granular_cpu_vs_wgsl("granular_pile_50", 50, 0.3));
    stats.push(probe_granular_cpu_vs_wgsl("granular_pile_200", 200, 0.3));

    let env = format!("host:{}", std::env::consts::OS);
    AccuracyReport {
        generated: "runtime".to_string(),
        env,
        note: "CPU 生产 vs wgsl 简化移植参考(非真实 adapter 误差;为 M1 adapter 阶段建基线)".to_string(),
        stats,
    }
}

// ----------------------------------------------------------------------------
// 真实 adapter 数据导出接口(预留)。
// ----------------------------------------------------------------------------

/// 导出一套 SPH `SphFlatData` 供真实 WebGPU adapter 消费(浏览器 JS / 原生 CLI)。
///
/// 返回人类可读的调试字符串(字段名 + 长度),消费者据此在 GPU 端用等价 wgsl 内核
/// 重算 `out_rho_p` / `out_acc_mu`,回填后与本 harness 的 CPU 结果做逐粒子误差对比,
/// 完成 M1 真实 adapter 部分。后续可改用 `bytemuck`(gpu feature 下可用)做紧凑二进制布局。
///
/// 不依赖 `serde`,确保 host 端非 gpu 构建也能编译导出接口。
pub fn export_flat_for_adapter(flat: &phy_fluid::SphFlatData) -> String {
    format!(
        "SphFlatData {{ n={}, h={:.4}, rest_density={:.4}, stiffness={:.4}, shear_min={:.4}, \
         pos.len={}, vel.len={}, scalar.len={}, cell_start.len={}, sorted.len={}, \
         grid_min={:?}, nc={:?}, gravity={:?} }}",
        flat.n,
        flat.h,
        flat.rest_density,
        flat.stiffness,
        flat.shear_min,
        flat.pos.len(),
        flat.vel.len(),
        flat.scalar.len(),
        flat.cell_start.len(),
        flat.sorted.len(),
        flat.grid_min,
        flat.nc,
        flat.gravity,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn m1_sph_cpu_vs_wgsl_finite_and_reasonable() {
        let st = probe_sph_cpu_vs_wgsl("test_sph", 4, 0.12, 0.2);
        assert!(st.cpu_finite, "CPU 生产 step 出现非有限值");
        assert!(st.wgsl_finite, "wgsl 参考输出非有限");
        // 不要求 MSE≈0(wgsl 是简化移植);只要求有限且偏差有界(< 1e6 加速度量级合理)。
        assert!(st.acc_mse.is_finite() && st.acc_max.is_finite());
        assert!(st.acc_max < 1.0e6, "偏差异常大: {}", st.acc_max);
    }

    #[test]
    fn m1_granular_cpu_vs_wgsl_finite() {
        let st = probe_granular_cpu_vs_wgsl("test_gran", 50, 0.3);
        assert!(st.cpu_finite, "CPU 生产颗粒 step 非有限");
        assert!(st.wgsl_finite, "wgsl 参考颗粒非有限");
        assert!(st.acc_mse.is_finite() && st.acc_max.is_finite());
    }

    #[test]
    fn m1_report_generates() {
        let rep = run_accuracy_report();
        let md = rep.to_markdown();
        assert!(md.contains("M1 GPU 数值一致性报告"));
        assert_eq!(rep.stats.len(), 4);
    }
}
