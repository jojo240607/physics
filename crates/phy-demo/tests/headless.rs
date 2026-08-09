//! 无窗口自测:对每种 DemoMode 构造 -> step -> 渲染到离屏 Framebuffer,
//! 断言 (a) 不 panic,(b) 像素不全为背景色(确有内容绘制)。
//! 这是 Web 化之前/之后统一的逻辑回归,纯 Rust 运行,无需桌面或浏览器。

use phy_demo::{Camera, DemoMode, Framebuffer, Scene};

fn count_non_bg(fb: &Framebuffer) -> usize {
    let bg = fb.pixels.first().copied().unwrap_or(0);
    fb.pixels.iter().filter(|&&p| p != bg).count()
}

#[test]
fn all_modes_render_without_panic() {
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
    let cam = Camera::default();
    let w = 320u32;
    let h = 240u32;
    let mut failures = Vec::new();

    for m in modes {
        let mut scene = Scene::new();
        scene.set_mode(m);
        let mut last_non_bg = 0usize;
        for step in 0..5 {
            scene.step();
            let mut fb = Framebuffer::new(w, h);
            fb.clear(); // 与 App 一致:先清背景
            scene.render(&mut fb, &cam);
            let n = count_non_bg(&fb);
            if step == 4 {
                last_non_bg = n;
            }
        }
        let ok = last_non_bg > 0;
        println!(
            "[{}] mode={:?} non_bg_pixels={}",
            if ok { "PASS" } else { "FAIL" },
            m,
            last_non_bg
        );
        if !ok {
            failures.push(m);
        }
    }
    assert!(failures.is_empty(), "空渲染模式: {:?}", failures);
}

#[test]
fn body_count_safe_on_non_rigid() {
    // 单场场景(Em/Grav/Wave/Acoustic)构造后 world 中无刚体子系统,body_count 应安全返回 0。
    assert_eq!(Scene::em().body_count(), 0);
    assert_eq!(Scene::grav().body_count(), 0);
    assert_eq!(Scene::wave().body_count(), 0);
    assert_eq!(Scene::acoustic().body_count(), 0);
    // 非刚体模式下若仍含刚体(如 new() 默认世界),不应 panic。
    let mut scene = Scene::new();
    scene.set_mode(DemoMode::Heat);
    let _ = scene.body_count(); // 不应 panic
}

#[test]
fn save_load_roundtrip_string() {
    let mut scene = Scene::new();
    scene.set_mode(DemoMode::All);
    for _ in 0..3 {
        scene.step();
    }
    let json = scene.save_string().expect("save to string");
    assert!(!json.is_empty());
    scene.load_string(&json).expect("load from string");
    assert!(scene.world.subsystem_count() > 0);
}
