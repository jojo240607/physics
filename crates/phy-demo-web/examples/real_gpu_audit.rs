//! M3 真机内核审批:native 桌面 wgpu 驱动**真实 GPU adapter**,跑 W2 OPTIC / W3 CAUSTIC
//! 的 wgsl compute 内核,把 GPU 逐像素/逐单元输出与 CPU 生产实现(`Approx::trace` /
//! `Caustics::accumulate`)对比,输出逐像素 max / RMSE —— 即 M3 要求的"GLSL→WGSL 内核
//! 全量数值审批"证据(OPTIC/CAUSTIC;SPH/GRANULAR 由 `real_gpu_error.rs` 覆盖)。
//!
//! 前提:MSVC 工具链(MinGW 无法链接 wgpu)+ `cargo +stable-msvc run --features gpu`.
//! 本机需有可用 GPU adapter;无 adapter 时打印 NO_ADAPTER 并优雅退出。
use phy_demo_web::gpu::GpuContext;
use phy_demo_web::gpu_audit::audit_all;

fn main() {
    pollster::block_on(run());
}

async fn run() {
    let ctx = match GpuContext::init().await {
        Ok(c) => c,
        Err(e) => {
            println!("NO_ADAPTER: {}", e);
            return;
        }
    };
    match audit_all(&ctx).await {
        Ok(report) => println!("M3 AUDIT PASS\n{}", report),
        Err(e) => {
            println!("M3 AUDIT FAIL: {}", e);
            std::process::exit(1);
        }
    }
}
