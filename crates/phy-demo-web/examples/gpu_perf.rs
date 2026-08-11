//! G2 真机性能基准:native 桌面 wgpu 驱动**真实 GPU adapter**,测量 W4 SPH / W5 颗粒
//! wgsl 内核在 GPU 上的**单帧耗时**(含 dispatch + 回读),对比 CPU 单线程基线(perf_sweep),
//! 输出加速比与 SLO(30fps / 60fps)判断。
//!
//! 前提:MSVC 工具链 + `cargo +stable-msvc run --example gpu_perf --features gpu --release`。
use std::time::Instant;

use phy_demo_web::gpu::{GpuContext, render_sph_gpu, render_granular_gpu};

/// 构造目标粒子数 n 的 SPH 场景:固定 box,反推间距使填充 ≈ n。
fn sph_flat(target: usize) -> phy_fluid::SphFlatData {
    use phy_fluid::{FluidWorld, SphParams};
    let mut params = SphParams::<f32>::defaults();
    let half = 3.0f32;
    let spacing = (8.0 * half.powi(3) / target as f32).powf(1.0 / 3.0);
    params.h = spacing * 2.0; // 光滑长度 = 2 倍间距,保证有邻居
    let mut w = FluidWorld::<f32>::new(params);
    w.fill_box(
        phy_math::Vec3::new(-half, 1.0, -half),
        phy_math::Vec3::new(half, 1.0 + 2.0 * half, half),
        spacing,
        0.1,
    );
    w.step(0.01f32);
    w.to_gpu_flat()
}

/// 构造目标粒子数 n 的颗粒场景:box 足够大容纳 n 个半径 r 的颗粒。
fn gran_flat(n: usize, radius: f32) -> phy_granular::GranularFlatData {
    use phy_granular::world::GranularWorld;
    let mut world = GranularWorld::<f32>::new();
    let per_side = ((n as f32).cbrt() as usize).max(1) + 1;
    let extent = per_side as f32 * radius * 2.0 * 1.05;
    world.set_bounds(
        phy_math::Vec3::new(-extent / 2.0, 0.0, -extent / 2.0),
        phy_math::Vec3::new(extent / 2.0, extent, extent / 2.0),
    );
    world.fill_grid(n, radius, 1.0f32, 1.05f32);
    world.to_gpu_flat()
}

/// 预热 warm 帧 + 计时 frames 帧,返回平均单帧 ms。
fn measure_ms<F: FnMut()>(mut f: F, warm: usize, frames: usize) -> f64 {
    for _ in 0..warm {
        f();
    }
    let t0 = Instant::now();
    for _ in 0..frames {
        f();
    }
    t0.elapsed().as_secs_f64() / frames as f64 * 1000.0
}

fn main() {
    pollster::block_on(run());
}

async fn run() {
    let ctx = match GpuContext::init().await {
        Ok(c) => c,
        Err(e) => {
            println!("NO_ADAPTER_OR_INIT_FAIL: {}", e);
            return;
        }
    };

    // 枚举 adapter 身份。
    let mut adapter_desc = String::new();
    {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let backends: Vec<wgpu::Adapter> =
            instance.enumerate_adapters(wgpu::Backends::all()).await;
        if let Some(a) = backends.first() {
            let info = a.get_info();
            adapter_desc = format!(
                "{:?} {} (vendor={:#x} device={:04x})",
                info.backend, info.name, info.vendor, info.device
            );
        }
    }

    println!("===== G2 真机 GPU 性能基准 =====");
    println!("adapter: {}", adapter_desc);
    println!("精度: f32 | 后端: 真实 GPU adapter compute | 对比: host CPU 单线程基线(f64)");
    println!("");

    let mut lines = Vec::new();
    lines.push(format!("G2 real-adapter perf report generated at {}", std::env::consts::OS));
    lines.push(format!("adapter: {}", adapter_desc));

    // CPU 基线(来自 docs/perf_baseline.md,host f64 单线程,供加速比参考)。
    // SPH: 1k=1.594ms, 10k=6.795ms; Granular: 1k=3.022ms, 5k=12.470ms, 10k=24.792ms。
    let cpu_sph: &[(usize, f64)] = &[(1000, 1.594), (10000, 6.795)];
    let cpu_gran: &[(usize, f64)] = &[(1000, 3.022), (5000, 12.470), (10000, 24.792)];

    // SPH 场景:目标 1k / 10k / 50k。
    let sph_scenes = [("sph_1k", 1000usize), ("sph_10k", 10000usize), ("sph_50k", 50000usize)];
    for (name, target) in sph_scenes {
        let flat = sph_flat(target);
        let n = flat.n;
        // 预热 3 次(shader 编译/pipeline 创建),再计时 20 帧。
        let ms = measure_ms(
            || {
                let _ = pollster::block_on(render_sph_gpu(&ctx, &flat));
            },
            3,
            20,
        );
        let fps = 1000.0 / ms;
        // 找 CPU 基线加速比。
        let accel = cpu_sph
            .iter()
            .find(|(cn, _)| *cn >= n)
            .map(|(_, cms)| format!("{:.1}x", *cms / ms))
            .unwrap_or_else(|| "—".to_string());
        let slo30 = if ms <= 33.3 { "✓" } else { "✗" };
        let slo60 = if ms <= 16.6 { "✓" } else { "✗" };
        println!(
            "SPH  {:<8} n={:>6} | GPU {:.3} ms/frame ({:.1} fps) | vsCPU {accel} | 30fps:{slo30} 60fps:{slo60}",
            name, n, ms, fps
        );
        lines.push(format!(
            "SPH  {:<8} n={:>6} | GPU {:.3} ms ({:.1} fps) | vsCPU {} | 30fps:{} 60fps:{}",
            name, n, ms, fps, accel, slo30, slo60
        ));
    }

    // 颗粒场景。
    let gran_scenes = [
        ("gran_1k", 1000usize, 0.1f32),
        ("gran_5k", 5000usize, 0.1f32),
        ("gran_10k", 10000usize, 0.1f32),
    ];
    for (name, n, r) in gran_scenes {
        let flat = gran_flat(n, r);
        let np = flat.n;
        let ms = measure_ms(
            || {
                let _ = pollster::block_on(render_granular_gpu(&ctx, &flat));
            },
            3,
            20,
        );
        let fps = 1000.0 / ms;
        let accel = cpu_gran
            .iter()
            .find(|(cn, _)| *cn >= np)
            .map(|(_, cms)| format!("{:.1}x", *cms / ms))
            .unwrap_or_else(|| "—".to_string());
        let slo30 = if ms <= 33.3 { "✓" } else { "✗" };
        let slo60 = if ms <= 16.6 { "✓" } else { "✗" };
        println!(
            "GRAN {:<8} n={:>6} | GPU {:.3} ms/frame ({:.1} fps) | vsCPU {accel} | 30fps:{slo30} 60fps:{slo60}",
            name, np, ms, fps
        );
        lines.push(format!(
            "GRAN {:<8} n={:>6} | GPU {:.3} ms ({:.1} fps) | vsCPU {} | 30fps:{} 60fps:{}",
            name, np, ms, fps, accel, slo30, slo60
        ));
    }

    // 落盘报告。
    if let Ok(dir) = std::env::var("CARGO_MANIFEST_DIR") {
        let path = std::path::Path::new(&dir).join("gpu_real_perf_report.txt");
        if let Ok(mut f) = std::fs::File::create(&path) {
            use std::io::Write;
            for l in &lines {
                let _ = writeln!(f, "{}", l);
            }
            println!("report written: {}", path.display());
        }
    }
}
