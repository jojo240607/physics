//! Headless 冒烟测试:验证游戏世界能用物理引擎 SDK 正常推进,无 NaN/panic。
//! 不依赖窗口/渲染,可在 CI 无显示环境运行。

use phy_game_demo::Game;
use phy_math::na::Vector3;

#[test]
fn world_builds_with_expected_body_count() {
    let g = Game::new();
    // 1 地面 + 4 墙 + 1 台阶 + 5 箱子 + 4 球 + 1 玩家 capsule = 16
    let rigid = phy_sdk::get_as::<phy_rigid::RigidSubsystem<f64>>(&g.world, 0).unwrap();
    assert_eq!(rigid.world.bodies.len(), 16, "关卡体数量不符合预期");
}

#[test]
fn player_falls_and_lands_on_ground() {
    let mut g = Game::new();
    // 无输入自由落体 2 秒(240 步),角色应从出生点 y=2 落到地面附近(y≈1.3)。
    for _ in 0..240 {
        g.step_with_input(Vector3::zeros(), false);
    }
    assert!(g.all_finite(), "落体后世界含非有限数");
    let p = g.player_pos();
    assert!(p.y < 2.0 && p.y > 0.0, "角色未落到地面附近:y={}", p.y);
    assert!(g.cc.grounded, "角色应已落地(grounded)");
}

#[test]
fn player_moves_forward_with_input() {
    let mut g = Game::new();
    // 先落地
    for _ in 0..240 {
        g.step_with_input(Vector3::zeros(), false);
    }
    let p0 = g.player_pos();
    // 向 -z 方向持续移动 1 秒(相机 yaw=0 时 fwd=(0,0,1),按 W 沿 +z;这里直接用 -z)
    let dir = Vector3::new(0.0, 0.0, -1.0);
    for _ in 0..120 {
        g.step_with_input(dir, false);
    }
    assert!(g.all_finite(), "移动后世界含非有限数");
    let p1 = g.player_pos();
    assert!((p1.z - p0.z).abs() > 0.5, "角色未水平移动:dz={}", p1.z - p0.z);
}

#[test]
fn player_jump_leaves_ground_then_returns() {
    let mut g = Game::new();
    for _ in 0..240 {
        g.step_with_input(Vector3::zeros(), false);
    }
    assert!(g.cc.grounded, "起跳前应已落地");
    // 跳一下
    g.step_with_input(Vector3::zeros(), true);
    assert!(!g.cc.grounded, "起跳后应离地");
    // 继续无输入,应再次落地且无 NaN
    for _ in 0..240 {
        g.step_with_input(Vector3::zeros(), false);
    }
    assert!(g.all_finite(), "跳跃落地后世界含非有限数");
    assert!(g.cc.grounded, "跳跃后应再次落地");
}

#[test]
fn long_run_stays_finite() {
    let mut g = Game::new();
    for step in 0..600 {
        // 混合输入:移动 + 周期跳
        let dir = Vector3::new(1.0, 0.0, 0.0);
        let jump = (step % 120) == 0;
        g.step_with_input(dir, jump);
    }
    assert!(g.all_finite(), "长时间混合输入后世界含非有限数");
}
