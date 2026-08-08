//! 固体 FEM 自测。
//!
//! 关键对照:悬臂梁自由端受集中力 P 时,小变形挠度解析解(Euler-Bernoulli)
//! `δ = P L³ / (3 E I)`,其中矩形截面惯性矩 `I = b h³ / 12`。
//! 我们用 FEM 静态求解得到自由端位移,与解析解比较(允许网格离散误差)。

use phy_math::{RealField, Vec3};

use crate::fem::SolidWorld;
use crate::mesh::{apply_tip_load, cantilever_box, tip_displacement, MeshParams};

fn f64abs(x: f64) -> f64 {
    if x < 0.0 {
        -x
    } else {
        x
    }
}

#[test]
fn simple_static_deflects_under_load() {
    // 单个四面体 + 末端载荷,位移应向下(负 y)且量级合理。
    let young = 1.0e6;
    let nu = 0.3;
    let mut w: SolidWorld<f64> = SolidWorld::new(young, nu, 1.0);
    w.from_box(
        Vec3::zeros(),
        Vec3::new(1.0, 1.0, 1.0),
        1,
        1,
        1,
        false,
    );
    w.nodes[0].fixed = true;
    w.nodes[1].fixed = true;
    w.nodes[2].fixed = true;
    // 对节点 3 施加 +y 力,应产生 +y 位移。
    w.nodes[3].f += Vec3::new(0.0, 100.0, 0.0);
    let ok = w.solve_equilibrium();
    assert!(ok, "静力求解应成功");
    assert!(w.nodes[3].u.y > 0.0, "受 +y 力应产生 +y 位移");
    assert!(w.nodes[0].u.norm() < 1e-12, "固定节点位移应为 0");
}

#[test]
fn cantilever_tip_deflection_matches_euler_bernoulli() {
    // 悬臂梁:长 L=1.0,截面 b=h=0.1,8 段;E=1e9,nu=0.3。
    let l_f = 1.0f64;
    let b = 0.1f64;
    let h = 0.1f64;
    let e = 1.0e9f64;
    let p_load = 10.0f64; // 自由端集中力(N),沿 -y
    let i_moment = b * h.powi(3) / 12.0;
    let delta_analytic = p_load * l_f.powi(3) / (3.0 * e * i_moment);

    let params = MeshParams::<f64> {
        lo: Vec3::zeros(),
        hi: Vec3::new(l_f, h, b),
        segs: (8, 1, 1),
        young: e,
        poisson: 0.3,
        rho: 1.0,
        fix_x_min: true,
    };
    let mut w = cantilever_box(&params);
    let n_tip = apply_tip_load(&mut w, Vec3::new(0.0, -p_load, 0.0));
    assert!(n_tip > 0, "自由端应有节点受力");

    let ok = w.solve_equilibrium();
    assert!(ok, "悬臂梁静力求解应成功");

    let delta_fem = tip_displacement(&w, 1); // y 轴位移(应为负)
    let delta_fem_abs = f64abs(delta_fem);
    // 离散四面体梁偏柔(尤其单排单元),容许 Tolerance。
    let tol = delta_analytic * 0.25;
    assert!(
        f64abs(delta_fem_abs - delta_analytic) <= tol,
        "FEM 挠度 {:.6e} 应与解析解 {:.6e} 接近(容差 {:.6e})",
        delta_fem_abs,
        delta_analytic,
        tol
    );
}

#[test]
fn fixed_nodes_do_not_move() {
    let params = MeshParams::<f64>::default();
    let mut w = cantilever_box(&params);
    apply_tip_load(&mut w, Vec3::new(0.0, -1.0, 0.0));
    assert!(w.solve_equilibrium());
    for n in w.nodes.iter() {
        if n.fixed {
            assert!(n.u.norm() < 1e-12, "固定节点位移必须为 0");
        }
    }
}

#[test]
fn dynamic_relaxation_settles() {
    // 动力松弛若干步后,自由端位移应趋于稳定(与静力解接近)。
    let params = MeshParams::<f64> {
        lo: Vec3::zeros(),
        hi: Vec3::new(1.0, 0.1, 0.1),
        segs: (6, 1, 1),
        young: 1.0e8,
        poisson: 0.3,
        rho: 1.0,
        fix_x_min: true,
    };
    let mut w = cantilever_box(&params);
    // 跑动力松弛,记录尾段位移变化,判定是否收敛(趋稳)。
    let dt = 0.001;
    let mut prev = tip_displacement(&w, 1);
    let mut settled = false;
    for step in 0..1000 {
        apply_tip_load(&mut w, Vec3::new(0.0, -5.0, 0.0)); // 复加载荷(step 会重置 f)
        w.step(dt);
        if step >= 950 {
            let cur = tip_displacement(&w, 1);
            let delta = f64abs(cur - prev);
            let scale = f64abs(cur) + 1e-9;
            if delta / scale < 1e-3 {
                settled = true;
                break;
            }
            prev = cur;
        }
    }
    assert!(settled, "动力松弛应在 1000 步内收敛趋稳(尾段位移变化 < 0.1%)");
}

// 确保 RealField 在测试中可用(静默导入守卫)。
#[allow(dead_code)]
fn _assert_realfield<T: RealField>() {}
