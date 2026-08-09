//! 无窗口自测:对每个 DemoMode 构造场景 -> step 若干步 -> 渲染到离屏 Framebuffer,
//! 断言 (a) 不 panic,(b) 像素不全为背景色(即确实有内容绘制)。
//! 用于在没有桌面/浏览器环境时验证渲染管线。

use phy_demo::{Camera, DemoMode, Framebuffer, Scene};

fn count_non_bg(fb: &Framebuffer, bg: [u8; 3]) -> usize {
    let mut n = 0;
    for &px in &fb.pixels {
        let r = ((px >> 16) & 0xff) as u8;
        let g = ((px >> 8) & 0xff) as u8;
        let b = (px & 0xff) as u8;
        if r != bg[0] || g != bg[1] || b != bg[2] {
            n += 1;
        }
    }
    n
}

fn main() {
    let w = 320u32;
    let h = 240u32;
    // Framebuffer::clear 使用 pack(0.05,0.07,0.10) = (13,18,26)。
    let bg = [13u8, 18u8, 26u8];
    let modes = [
        DemoMode::Rigid,
        DemoMode::Fluid,
        DemoMode::Heat,
        DemoMode::Soft,
        DemoMode::Optics,
        DemoMode::FluidHeat,
        DemoMode::Em,
        DemoMode::Grav,
        DemoMode::Wave,
        DemoMode::Acoustic,
        DemoMode::All,
    ];
    let mut failures = Vec::new();
    for m in modes {
        let mut scene = Scene::new();
        scene.set_mode(m);
        let mut non_bg_after = 0usize;
        for step in 0..5 {
            scene.step();
            let mut fb = Framebuffer::new(w, h);
            fb.clear();
            scene.render(&mut fb, &Camera::default());
            let n = count_non_bg(&fb, bg);
            if step == 4 {
                non_bg_after = n;
            }
        }
        let ok = non_bg_after > 0;
        println!(
            "[{}] mode={:?} non_bg_pixels={} -> {}",
            if ok { "PASS" } else { "FAIL" },
            m,
            non_bg_after,
            if ok { "OK" } else { "EMPTY" }
        );
        if !ok {
            failures.push(m);
        }
    }

    // 额外:模拟 F5/F9 存档读档(不依赖桌面路径)
    let mut scene = Scene::new();
    scene.set_mode(DemoMode::All);
    for _ in 0..3 {
        scene.step();
    }
    let json = phy_io::save_world_json(&scene.world);
    let reloaded = phy_io::load_world_json::<f64>(&json);
    println!(
        "[CHECK] save/load roundtrip subsystems={} bytes={}",
        reloaded.subsystem_count(),
        json.len()
    );

    // 额外:body_count 在非 Rigid 模式下不应 panic
    let mut scene = Scene::new();
    scene.set_mode(DemoMode::Heat);
    let bc = scene.body_count();
    println!("[CHECK] body_count() in Heat mode = {} (no panic)", bc);

    if !failures.is_empty() {
        eprintln!("FAILED modes: {:?}", failures);
        std::process::exit(1);
    }
    println!("ALL MODES OK");
}
