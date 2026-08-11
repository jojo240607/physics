//! M2 — 性能与规模基准扫描(host 端)。
//!
//! 扫描多个粒子规模(默认 1k / 10k;加 `--large` 含 100k),
//! 对 SPH 流体与 PBD 颗粒分别测量单帧步进耗时,输出 Markdown 报告。
//!
//! 运行:
//! ```text
//! cargo run -p phy-demo --bin perf_sweep            # 1k / 10k
//! cargo run -p phy-demo --bin perf_sweep -- --large # 追加 100k
//! cargo run -p phy-demo --bin perf_sweep -- --out docs/perf_baseline.md
//! ```
//!
//! 输出字段:粒子数、单帧平均 ms、估算 FPS(1000/ms)、内存估算(MB,粗略)。
//! 该数字为"单线程 CPU 参考实现"基线,真实 GPU 路径(wgsl)应显著更快,
//! 但本机无 adapter,故 host 基线即为当前可测吞吐下限参考。

use std::time::Instant;

use nalgebra::Vector3 as Vec3;
use phy_fluid::{FluidSubsystem, FluidWorld, SphParams};
use phy_granular::world::GranularWorld;
use phy_core::World;

fn sph_world(n: usize) -> World<f64> {
    let params = SphParams::<f64>::defaults();
    let mut fluid = FluidWorld::<f64>::new(params);
    // 固定盒子尺寸,由目标粒子数 n 反推间距,使实际填充粒子数 ≈ n
    // (fill_box 用固定间距填满盒子,若盒子随 n 增长会立方爆炸)。
    let half = 5.0f64;
    let spacing = (8.0 * half.powi(3) / n as f64).powf(1.0 / 3.0);
    fluid.fill_box(
        Vec3::new(-half, 1.0, -half),
        Vec3::new(half, 1.0 + 2.0 * half, half),
        spacing,
        0.1,
    );
    let mut w = World::new();
    w.add_subsystem(Box::new(FluidSubsystem::new(fluid)));
    w
}

fn granular_world(n: usize) -> GranularWorld<f64> {
    let mut gw = GranularWorld::<f64>::new();
    gw.iterations = 4;
    gw.fill_grid(n, 0.3, 1.0, 1.05);
    gw
}

/// 测量:预热 `warm` 帧后,跑 `frames` 帧,返回平均单帧 ms。
fn measure<F: FnMut()>(mut step: F, warm: usize, frames: usize) -> f64 {
    for _ in 0..warm {
        step();
    }
    let t0 = Instant::now();
    for _ in 0..frames {
        step();
    }
    let elapsed = t0.elapsed().as_secs_f64();
    elapsed / frames as f64 * 1000.0
}

struct Row {
    scenario: String,
    n: usize,
    ms_per_frame: f64,
    fps: f64,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let large = args.iter().any(|a| a == "--large");
    let out_path = args
        .iter()
        .position(|a| a == "--out")
        .and_then(|i| args.get(i + 1).cloned());
    let csv_path = args
        .iter()
        .position(|a| a == "--csv")
        .and_then(|i| args.get(i + 1).cloned());
    // `--check`:跑测量后,逐场景比对 `csv_path`(缺省 docs/perf_baseline.csv)基线,
    // 单帧 ms 超出 ±PERF_TOL(默认 20%)即判回归并 exit 1(G6 数值门禁)。
    let check = args.iter().any(|a| a == "--check");
    let tol = args
        .iter()
        .position(|a| a == "--tol")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap_or(0.20);

    let mut rows = Vec::new();

    // SPH: 扫描 1k / 10k(10k 已验证可正常 step)。
    let mut sph_scales = vec![1_000usize, 10_000];
    // Granular: M2-fix 后 Jacobi+rayon 改用 par_chunks(任务数≈线程数而非 pairs 数),
    // 消除了 per-task 全量 delta vec 分配导致的性能悬崖,5000/10000 均在合理耗时内。
    let mut gran_scales = vec![1_000usize, 5_000, 10_000];
    // `--large` 同时扩展 SPH(→100k)与 Granular(→20000)上限,验证规模延展性。
    if large {
        sph_scales.push(100_000);
        gran_scales.push(20_000);
    }

    println!("=== M2 perf sweep (host CPU, f64) ===");
    for &n in &sph_scales {
        eprintln!("[probe] building SPH world n={}", n);
        let warm = 20;
        let frames = 60;
        let mut w = sph_world(n);
        eprintln!("[probe] SPH world built n={}", n);
        let ms = measure(|| w.step(0.01), warm, frames);
        println!("  SPH    n={:>7}  {:.3} ms/frame  ({:.1} fps)", n, ms, 1000.0 / ms);
        rows.push(Row {
            scenario: "SPH fluid".into(),
            n,
            ms_per_frame: ms,
            fps: 1000.0 / ms,
        });
    }

    for &n in &gran_scales {
        eprintln!("[probe] building Granular world n={}", n);
        let warm = 20;
        let frames = 60;
        let mut gw = granular_world(n);
        eprintln!("[probe] Granular world built n={}", n);
        let ms = measure(|| gw.step(0.016), warm, frames);
        println!("  Gran   n={:>7}  {:.3} ms/frame  ({:.1} fps)", n, ms, 1000.0 / ms);
        rows.push(Row {
            scenario: "PBD granular".into(),
            n,
            ms_per_frame: ms,
            fps: 1000.0 / ms,
        });
    }

    let md = render_markdown(&rows, large);
    if let Some(path) = out_path {
        std::fs::write(&path, &md).unwrap_or_else(|e| eprintln!("write {} failed: {}", path, e));
        println!("report -> {}", path);
    } else {
        println!("\n{}", md);
    }

    // 结构化 CSV(供版本控制基线 + 门禁比对)。首列 header:
    // scenario,n,ms_per_frame,fps
    let csv_lines: Vec<String> = std::iter::once("scenario,n,ms_per_frame,fps".to_string())
        .chain(rows.iter().map(|r| {
            format!("{},{},{:.3},{}", r.scenario, r.n, r.ms_per_frame, r.fps as usize)
        }))
        .collect();
    let csv = csv_lines.join("\n");
    if let Some(path) = csv_path.clone() {
        std::fs::write(&path, &csv).unwrap_or_else(|e| eprintln!("write {} failed: {}", path, e));
        println!("csv -> {}", path);
    }

    // G6 数值门禁:比对基线,单帧 ms 超 ±tol 判回归。
    if check {
        let base = csv_path
            .or_else(|| Some("docs/perf_baseline.csv".to_string()))
            .unwrap();
        let exit = run_check(&base, &rows, tol);
        std::process::exit(exit);
    }
}

/// 读取基线 CSV(scenario,n,ms_per_frame,fps),逐 (scenario,n) 与本次测量比对。
/// 返回进程退出码:0=全部在容差内,1=存在回归。
fn run_check(base_path: &str, rows: &[Row], tol: f64) -> i32 {
    let raw = match std::fs::read_to_string(base_path) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[check] FAIL: 无法读取基线 {}: {}", base_path, e);
            return 1;
        }
    };
    let mut base: std::collections::BTreeMap<(String, usize), f64> = std::collections::BTreeMap::new();
    for line in raw.lines().skip(1) {
        let c: Vec<&str> = line.split(',').collect();
        if c.len() < 3 {
            continue;
        }
        let n = match c[1].parse::<usize>() {
            Ok(v) => v,
            Err(_) => continue,
        };
        let ms = match c[2].parse::<f64>() {
            Ok(v) => v,
            Err(_) => continue,
        };
        base.insert((c[0].to_string(), n), ms);
    }

    let mut fails = 0;
    let mut checked = 0;
    for r in rows {
        let key = (r.scenario.clone(), r.n);
        match base.get(&key) {
            None => {
                eprintln!("[check] SKIP: 基线无 {} n={} (新场景,不判回归)", r.scenario, r.n);
            }
            Some(&b) => {
                checked += 1;
                let rel = (r.ms_per_frame - b).abs() / b;
                let ok = rel <= tol;
                if !ok {
                    fails += 1;
                }
                eprintln!(
                    "[check] {} {} n={}: 本次 {:.3}ms 基线 {:.3}ms 偏差 {:.1}% {}",
                    if ok { "PASS" } else { "FAIL" },
                    r.scenario,
                    r.n,
                    r.ms_per_frame,
                    b,
                    rel * 100.0,
                    if ok { "" } else { "(超阈值)" }
                );
            }
        }
    }
    if fails > 0 {
        eprintln!("[check] FAIL: {}/{} 场景超 ±{:.0}% 容差", fails, checked, tol * 100.0);
        1
    } else {
        eprintln!("[check] PASS: 全部 {} 场景在 ±{:.0}% 容差内", checked, tol * 100.0);
        0
    }
}

fn render_markdown(rows: &[Row], large: bool) -> String {
    let mut s = String::new();
    s.push_str("# M2 性能与规模基线(host CPU, f64)\n\n");
    s.push_str(&format!(
        "- 生成: runtime | 大规模(100k): {} | 精度: f64 | 后端: 单线程 host CPU\n\n",
        if large { "含" } else { "未含(加 --large)" }
    ));
    s.push_str("| 场景 | 粒子数 | 单帧 ms | 估算 FPS |\n");
    s.push_str("|---|---|---|---|\n");
    for r in rows {
        s.push_str(&format!(
            "| {} | {} | {:.3} | {:.1} |\n",
            r.scenario, r.n, r.ms_per_frame, r.fps
        ));
    }
    s.push_str("\n## 解读\n\n");
    s.push_str(
        "- 以上为 **单线程 CPU 参考实现** 的吞吐下限;真实 WebGPU(wgsl)路径应显著更快,\n\
         但本机无 adapter,故 host 基线即为当前可测参考。\n\
        - 业务 SLO 参考:实时仿真通常要求 ≥ 30fps(单帧 ≤ 33ms),游戏要求 ≥ 60fps(≤ 16.6ms)。\n\
        - 若某规模单帧耗时超 SLO,需启用 GPU 路径或并行化(rayon)才能落地业务。\n",
    );
    s
}
