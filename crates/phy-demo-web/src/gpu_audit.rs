//! M3 内核全量审批(audit):把 GPU 计算着色器的输出与 **CPU 生产实现**逐像素 / 逐单元
//! 数值对比,以确认所有 WGSL 内核(OPTIC / CAUSTIC;SPH/GRANULAR 已在 `gpu_accuracy`
//! 覆盖)正确复刻了它们对应的 GLSL/CPU 算法。
//!
//! 这是 "GLSL→WGSL 内核全量审批" 的关键一环:wgpu 的编译期类型检查只能保证语法,
//! 不能保证数值语义与 CPU 一致。此模块在真实 adapter 上跑同样的输入,断言:
//! - OPTIC:每像素 RGB 与 CPU `Approx::trace` 的 RMSE 在 f32 容差内(折射/朗伯/阴影一致)。
//! - CAUSTIC:每网格单元强度与 CPU `Caustics::accumulate` 的 RMSE 在 f32 容差内。
//!
//! 射线生成 / 网格采样严格复刻 WGSL 的 `main` 与 `render_caustics_gpu`,保证喂入
//! 完全一致的坐标,只比较"算法"差异。

use crate::gpu::{GpuContext, render_camera_gpu, render_caustics_gpu};
use phy_math::Vec3 as V3;
use phy_optics::{Approx, Caustics, OpticBody, OpticScene, Renderer, Surface};
use phy_rigid::{Body, Shape};

/// 复刻 WGSL `main` 的逐像素射线生成(eye/target/up/fov → 单位方向)。
fn optic_ray(
    x: u32,
    y: u32,
    width: u32,
    height: u32,
    eye: V3<f32>,
    target: V3<f32>,
    up: V3<f32>,
    tan_w: f32,
    tan_h: f32,
) -> V3<f32> {
    let fwd = (target - eye).normalize();
    let mut right = fwd.cross(&up);
    if right.norm() > 1e-12 {
        right = right.normalize();
    } else {
        right = V3::new(1.0, 0.0, 0.0);
    }
    let true_up = right.cross(&fwd);
    let u = (2.0 * x as f32 / ((width - 1) as f32) - 1.0) * tan_w;
    let v = (1.0 - 2.0 * y as f32 / ((height - 1) as f32)) * tan_h;
    let dir = (fwd + right * u + true_up * v).normalize();
    dir
}

/// 误差指标。
struct ErrStat {
    max_abs: f32,
    mean_abs: f32,
    rms: f32,
    count: usize,
}

fn stat(diffs: &[f32]) -> ErrStat {
    let n = diffs.len();
    let mut sum_abs = 0.0f32;
    let mut sum_sq = 0.0f32;
    let mut max_abs = 0.0f32;
    for &d in diffs {
        let a = d.abs();
        sum_abs += a;
        sum_sq += d * d;
        if a > max_abs {
            max_abs = a;
        }
    }
    ErrStat {
        max_abs,
        mean_abs: if n > 0 { sum_abs / n as f32 } else { 0.0 },
        rms: if n > 0 { (sum_sq / n as f32).sqrt() } else { 0.0 },
        count: n,
    }
}

/// 在真实 adapter 上审批 OPTIC 内核:GPU `render_camera_gpu` vs CPU `Approx::trace`。
pub async fn audit_optic(ctx: &GpuContext) -> Result<String, String> {
    let mut scene = OpticScene::<f32>::new();
    scene.background = V3::new(0.05f32, 0.07, 0.1);
    scene.env_ior = 1.0;
    // 玻璃球(透射)+ 不透明红盒(朗伯 + 阴影),覆盖折射与朗伯两条分支。
    let glass = Body::new(Shape::Sphere { r: 1.0 }, V3::new(0.0f32, 0.0, -3.0), 0.0);
    scene.add(OpticBody::new(glass, Surface::glass(1.5, V3::new(1.0, 1.0, 1.0))));
    let boxb = Body::new(
        Shape::Box {
            half: V3::new(0.6f32, 0.6, 0.6),
        },
        V3::new(1.2f32, -0.5, -2.0),
        0.0,
    );
    scene.add(OpticBody::new(
        boxb,
        Surface::diffuse(V3::new(0.9f32, 0.1, 0.1)),
    ));

    let w = 32u32;
    let h = 32u32;
    let fov = 1.0f32;
    let tan_h = (fov * 0.5).tan();
    let aspect = (w as f32) / (h as f32);
    let tan_w = tan_h * aspect;
    let eye = V3::new(0.0f32, 0.0, 0.0);
    let target = V3::new(0.0f32, 0.0, -1.0);
    let up = V3::new(0.0f32, 1.0, 0.0);

    // GPU 输出。
    let mut gpu = vec![V3::new(0.0f32, 0.0, 0.0); (w * h) as usize];
    render_camera_gpu(ctx, &scene, &mut gpu, w as usize, h as usize, &eye, &target, &up, fov)
        .await?;

    // CPU 参考:逐像素用相同射线调 Approx::trace。
    let approx = Approx;
    let mut diffs = Vec::with_capacity((w * h) as usize * 3);
    for y in 0..h {
        for x in 0..w {
            let dir = optic_ray(x, y, w, h, eye, target, up, tan_w, tan_h);
            let c = approx.trace(&scene, &eye, &dir);
            let g = gpu[(y * w + x) as usize];
            diffs.push(c.x - g.x);
            diffs.push(c.y - g.y);
            diffs.push(c.z - g.z);
        }
    }
    let s = stat(&diffs);
    // f32 容差:折射/朗伯/阴影链路允许 1e-3 级误差(并行求和、sqrt 顺序差异)。
    let tol = 2e-3f32;
    if s.max_abs > tol {
        return Err(format!(
            "M3 optic: max_abs={:.3e} > tol={:.1e} (rms={:.3e}, n={})",
            s.max_abs, tol, s.rms, s.count
        ));
    }
    Ok(format!(
        "M3 optic PASS: max_abs={:.3e} mean_abs={:.3e} rms={:.3e} n={}",
        s.max_abs, s.mean_abs, s.rms, s.count
    ))
}

/// 在真实 adapter 上审批 CAUSTIC 内核:GPU `render_caustics_gpu` vs CPU `Caustics::accumulate`。
pub async fn audit_caustics(ctx: &GpuContext) -> Result<String, String> {
    let mut scene = OpticScene::<f32>::new();
    scene.env_ior = 1.0;
    let glass = Body::new(Shape::Sphere { r: 1.0 }, V3::new(0.0f32, 0.5, -3.0), 0.0);
    scene.add(OpticBody::new(glass, Surface::glass(1.5, V3::new(1.0, 1.0, 1.0))));

    let light_dir = V3::new(0.3f32, 1.0, 0.2);
    let plane_y = 0.0f32;
    let half_extent = 6.0f32;
    let grid_n = 24usize;

    // GPU 输出。
    let gpu = render_caustics_gpu(ctx, &scene, &light_dir, plane_y, half_extent, grid_n).await?;

    // CPU 参考。
    let caustics = Caustics;
    let (cpu, _max) = caustics.accumulate(&scene, &light_dir, plane_y, half_extent, grid_n);

    let mut diffs = Vec::with_capacity(grid_n * grid_n);
    for j in 0..grid_n {
        for i in 0..grid_n {
            let g = gpu[j * grid_n + i];
            let c = cpu[j][i];
            diffs.push(c - g);
        }
    }
    let s = stat(&diffs);
    // 焦散是能量累积,允许 1e-3 级误差(逐格射线起点/法向/EPS 数值差异)。
    let tol = 2e-3f32;
    if s.max_abs > tol {
        return Err(format!(
            "M3 caustics: max_abs={:.3e} > tol={:.1e} (rms={:.3e}, n={})",
            s.max_abs, tol, s.rms, s.count
        ));
    }
    Ok(format!(
        "M3 caustics PASS: max_abs={:.3e} mean_abs={:.3e} rms={:.3e} n={}",
        s.max_abs, s.mean_abs, s.rms, s.count
    ))
}

/// 汇总审批:依次跑 OPTIC + CAUSTIC,返回拼接报告;任一不通过则整体 Err。
pub async fn audit_all(ctx: &GpuContext) -> Result<String, String> {
    let o = audit_optic(ctx).await?;
    let c = audit_caustics(ctx).await?;
    Ok(format!("{}\n{}", o, c))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gpu::GpuContext;

    /// 在真实 adapter 上审批 OPTIC/CAUSTIC 内核。
    /// 无 GPU adapter 的环境(如 CI 无显卡)优雅跳过,不报失败。
    #[test]
    fn m3_audit_kernels_on_real_adapter() {
        pollster::block_on(async {
            let ctx = match GpuContext::init().await {
                Ok(c) => c,
                Err(_) => {
                    eprintln!("M3 audit skipped: no GPU adapter in this environment");
                    return;
                }
            };
            let report = audit_all(&ctx)
                .await
                .expect("M3 audit must pass on real adapter");
            println!("{}", report);
        });
    }
}
