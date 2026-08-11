//! 临时:验证 native(桌面)wgpu 能否枚举到真实 GPU adapter。
//! 仅用于确认本机是否具备跑 G1 真机误差报告的条件。
use wgpu::Instance;

fn main() {
    pollster::block_on(run());
}

async fn run() {
    let instance = Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
    // 枚举所有后端(原生下会枚举 Vulkan / DX12)。
    let backends: Vec<wgpu::Adapter> = instance
        .enumerate_adapters(wgpu::Backends::all())
        .await;
    if backends.is_empty() {
        println!("NO_ADAPTER: 本机无可用 GPU adapter(或后端驱动未就绪)");
        return;
    }
    for a in &backends {
        let info = a.get_info();
        println!(
            "ADAPTER: backend={:?} vendor={:#x} device={:04x} name={} type={:?}",
            info.backend,
            info.vendor,
            info.device,
            info.name,
            info.device_type
        );
    }
}
