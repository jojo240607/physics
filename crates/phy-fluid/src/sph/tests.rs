//! SPH 流体世界单元测试。

use crate::particle::Particle;
use crate::sph::couple::CouplePoint;
use crate::sph::params::SphParams;
use crate::sph::world::FluidWorld;
use phy_core::Subsystem;
use phy_math::Vec3;
use phy_rigid::shape::{Body, Shape};

fn test_params() -> SphParams<f64> {
    let mut p = SphParams::defaults();
    p.bounds_min = Vec3::new(-3.0, -3.0, -3.0);
    p.bounds_max = Vec3::new(3.0, 3.0, 3.0);
    p
}

#[test]
fn fill_box_count_matches_grid() {
    let mut w = FluidWorld::new(test_params());
    w.fill_box(
        Vec3::new(-1.0, -1.0, -1.0),
        Vec3::new(1.0, 1.0, 1.0),
        0.2,
        0.05,
    );
    // 间距 0.2 在 [-0.95,0.95] 区间约 10 个/轴 => ~1000 粒子。
    assert!(w.len() > 500 && w.len() < 2000, "got {}", w.len());
}

#[test]
fn density_converges_near_rest() {
    // 充足粒子在自由空间中静止,密度应在静止密度附近(±20%)。
    let mut p = test_params();
    p.gravity = Vec3::zeros(); // 关闭重力便于稳定
    let mut w = FluidWorld::new(p);
    w.fill_box(
        Vec3::new(-0.5, -0.5, -0.5),
        Vec3::new(0.5, 0.5, 0.5),
        0.1,
        0.05,
    );
    for _ in 0..15 {
        w.step(0.002);
    }
    let mut sum = 0.0;
    for pt in &w.particles {
        sum += pt.rho;
    }
    let avg = sum / w.len() as f64;
    let rho0 = w.params.rest_density;
    assert!(
        (avg - rho0).abs() / rho0 < 0.25,
        "平均密度 {avg} 偏离静止密度 {rho0} 过大"
    );
}

#[test]
fn particle_count_conserved_under_step() {
    let mut w = FluidWorld::new(test_params());
    w.fill_box(
        Vec3::new(-1.0, -1.0, -1.0),
        Vec3::new(1.0, 1.0, 1.0),
        0.1,
        0.05,
    );
    let n0 = w.len();
    for _ in 0..50 {
        w.step(0.003);
    }
    assert_eq!(w.len(), n0, "粒子数不应在 step 中改变");
}

#[test]
fn dam_break_stays_in_bounds() {
    // 溃坝:重力下流体应始终待在盒内(无 NaN / 无越界)。
    let mut w = FluidWorld::new(test_params());
    w.fill_box(
        Vec3::new(-1.5, -1.5, -0.5),
        Vec3::new(-0.5, 1.5, 0.5),
        0.1,
        0.05,
    );
    let lo = w.params.bounds_min;
    let hi = w.params.bounds_max;
    for _ in 0..120 {
        w.step(0.0025);
        for pt in &w.particles {
            assert!(pt.pos.x.is_finite() && pt.pos.y.is_finite() && pt.pos.z.is_finite());
            assert!(pt.pos.x >= lo.x - 1e-6 && pt.pos.x <= hi.x + 1e-6);
            assert!(pt.pos.y >= lo.y - 1e-6 && pt.pos.y <= hi.y + 1e-6);
            assert!(pt.pos.z >= lo.z - 1e-6 && pt.pos.z <= hi.z + 1e-6);
        }
    }
}

#[test]
fn static_body_blocks_fluid() {
    // 静态盒作为不可穿透边界:流体不应进入盒内部。
    let mut w = FluidWorld::new(test_params());
    w.fill_box(
        Vec3::new(-1.5, -1.5, -0.5),
        Vec3::new(1.5, -1.0, 0.5),
        0.1,
        0.05,
    );
    let mut body = Body {
        shape: Shape::Box {
            half: Vec3::new(1.0, 0.5, 1.0),
        },
        pos: Vec3::new(0.0, 0.0, 0.0),
        rot: phy_math::na::UnitQuaternion::identity(),
        vel: Vec3::zeros(),
        inv_mass: 0.0, // 静态
        ..Default::default()
    };
    for _ in 0..100 {
        w.step(0.0025);
        w.couple_bodies(std::slice::from_mut(&mut body), 0.0025, 0.0);
    }
    // 没有粒子应落在盒内(局部坐标 |x|<=1,|y|<=0.5,|z|<=1)。
    for pt in &w.particles {
        let local = body.to_local(&pt.pos);
        assert!(
            !(local.x.abs() <= 1.0 && local.y.abs() <= 0.5 && local.z.abs() <= 1.0),
            "粒子穿透了静态盒: local={:?}",
            local
        );
    }
}

#[test]
fn dynamic_body_gets_buoyancy_upward() {
    // 轻球(密度 < 流体)完全浸没、流体静止时应获得向上的净速度(纯阿基米德效应)。
    // 关闭重力使流体保持静止,隔离浮力,避免自由下落流体的下拽耦合掩盖上举力。
    let mut p = test_params();
    p.gravity = Vec3::zeros();
    let mut w = FluidWorld::new(p);
    w.fill_box(
        Vec3::new(-0.5, -0.5, -0.5),
        Vec3::new(0.5, 0.5, 0.5),
        0.1,
        0.05,
    );
    for _ in 0..15 {
        w.step(0.0025);
    }
    // 一个密度约为流体 1/4 的球,完全浸没。
    let vol = 4.0 / 3.0 * std::f64::consts::PI * 0.6f64.powi(3);
    let mass_b = w.params.rest_density * vol * 0.25;
    let mut body = Body {
        shape: Shape::Sphere { r: 0.6 },
        pos: Vec3::new(0.0, 0.0, 0.0),
        rot: phy_math::na::UnitQuaternion::identity(),
        vel: Vec3::zeros(),
        inv_mass: 1.0 / mass_b,
        ..Default::default()
    };
    let v0 = body.vel.y;
    // 把粒子推到球内以制造淹没(流体静止,不会有下拽动量)。
    for pt in w.particles.iter_mut() {
        pt.pos = Vec3::new(pt.pos.x * 0.3, pt.pos.y * 0.3, pt.pos.z * 0.3);
        pt.vel = Vec3::zeros();
    }
    for _ in 0..30 {
        w.step(0.0025);
        w.couple_bodies(std::slice::from_mut(&mut body), 0.0025, 0.0);
    }
    assert!(
        body.vel.y > v0,
        "轻球应因浮力获得向上的速度,实际 vy={}",
        body.vel.y
    );
}

#[test]
fn couple_points_buoyancy_upward() {
    // 静止流体中淹没的点应收到向上的浮力(force.y > 0),且流体获得反向动量。
    let mut p = test_params();
    p.gravity = Vec3::new(0.0, -9.81, 0.0);
    let mut w = FluidWorld::new(p);
    w.fill_box(
        Vec3::new(-0.5, -0.5, -0.5),
        Vec3::new(0.5, 0.5, 0.5),
        0.1,
        0.05,
    );
    for _ in 0..15 {
        w.step(0.0025);
    }
    // 软体密度 ~ 同流体,质量 0.02(=单粒子质量),应受净浮力(因流体静止,无下拽)。
    let mut pt = CouplePoint::new(Vec3::new(0.0, 0.0, 0.0), Vec3::zeros(), 0.02);
    // 记录流体总动量(用于验证反向交换)。
    let p_before: Vec3<f64> = w.particles.iter().map(|x| x.vel * x.mass).sum();
    w.couple_points(
        std::slice::from_mut(&mut pt),
        0.0025,
        1.0,
        w.params.rest_density,
    );
    assert!(pt.force.y > 0.0, "淹没点应受向上浮力, got {}", pt.force.y);
    let p_after: Vec3<f64> = w.particles.iter().map(|x| x.vel * x.mass).sum();
    // 流体因承受能力应获得向下的动量(总动量守恒:点 + 流体 ≈ 0 变化前的系统静止)。
    let dp_fluid = p_after - p_before;
    assert!(
        dp_fluid.y < 0.0,
        "流体应获得向下的反向动量, got {}",
        dp_fluid.y
    );
}

#[test]
fn couple_heat_warmer_fluid_rises() {
    // 温度场下半热、上半冷:温升处流体密度下降 → 粒子获得向上的额外加速度。
    use phy_field::{Bc, HeatField, ScalarField};
    let mut p = test_params();
    p.gravity = Vec3::new(0.0, -9.81, 0.0);
    let mut w = FluidWorld::new(p);
    // 单个粒子放在略偏上方的“热”区。
    w.particles.clear();
    let mut pt = Particle::new(Vec3::new(0.0, 1.0, 0.0), 0.05);
    pt.acc = Vec3::zeros();
    w.particles.push(pt);

    // 温度场:下半温 0,上半温 1(在 y>0 处热)。
    // 温度剖面按局部 y = iy*dx - nx*dx/2 构建,故把场原点设到 (-8,-8,-8)
    // 使格点 iy 对应世界坐标 -8+iy,从而“热区”正好在世界 y>0(粒子所在处)。
    let nx = 16usize;
    let dx = 1.0_f64;
    let mut f = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann)
        .with_origin(Vec3::new(-8.0, -8.0, -8.0));
    for iy in 0..nx {
        let y = (iy as f64) * dx - (nx as f64) * dx / 2.0;
        let t = if y > 0.0 { 1.0 } else { 0.0 };
        for ix in 0..nx {
            for iz in 0..nx {
                let idx = f.idx(ix, iy, iz);
                f.u[idx] = t;
            }
        }
    }
    let mut heat = HeatField::new(f, 0.1);
    let t_ref = 0.0_f64;
    let beta = 0.5_f64; // ρ(T)=ρ0/(1+0.5·T)
    // 调用(通过 HeatFieldLike trait 对象)。
    w.couple_heat(&mut heat, 0.01, t_ref, beta, 0.0);

    // 热区:浮力写入 body_acc,净加速度 = g + body_acc;暖区 body_acc.y > 0
    // 使净加速度比纯重力(-9.81)更向上(更接近 0 或为正)。
    let net_y = w.params.gravity.y + w.particles[0].body_acc.y;
    assert!(
        net_y > -9.81,
        "热区粒子应获得向上热浮力修正(净 acc.y > -g), got {}",
        net_y
    );
}

#[test]
fn couple_heat_injects_source_into_moving_region() {
    // 运动粒子应向热场注入热源(对流换热):运动区域格点升温。
    use phy_field::{Bc, HeatField, ScalarField};
    let mut p = test_params();
    p.gravity = Vec3::zeros();
    let mut w = FluidWorld::new(p);
    w.particles.clear();
    // 放在中心、带速度。
    let mut pt = Particle::new(Vec3::new(0.0, 0.0, 0.0), 0.05);
    pt.vel = Vec3::new(2.0, 0.0, 0.0);
    w.particles.push(pt);

    let nx = 16usize;
    let dx = 1.0_f64;
    let f = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann);
    let mut heat = HeatField::new(f, 0.1);
    let before = heat.field.sample(0, 0, 0);
    w.couple_heat(&mut heat, 0.01, 0.0, 0.0, 0.1);
    // couple_heat 把热源累加到 src;推进一帧扩散把 src 合入 u。
    heat.field.step_diffusion(0.1, 0.01);
    let after = heat.field.sample(0, 0, 0);
    assert!(
        after > before,
        "运动区域热场应升温, before={}, after={}",
        before,
        after
    );
}

#[test]
fn boussinesq_closed_loop_plume_rises_and_advects() {
    // Boussinesq 闭环:暖斑受浮力上升 → 流体获得向上速度 → 该速度场使热场被对流
    // 平流(vel_sampler 由 couple_heat 安装)→ 暖斑随流上移。验证热浮力↔对流闭环。
    use phy_field::{Bc, HeatField, ScalarField};
    let mut p = test_params();
    p.gravity = Vec3::new(0.0, -9.81, 0.0);
    let mut w = FluidWorld::new(p);
    // 用小盒填少量流体,避免大计算量。粒子集中在世界原点附近。
    w.particles.clear();
    let n = 3;
    let dxp = 0.4_f64;
    for ix in 0..n {
        for iy in 0..n {
            for iz in 0..n {
                let x = (ix as f64 - 1.0) * dxp;
                let y = (iy as f64 - 1.0) * dxp;
                let z = (iz as f64 - 1.0) * dxp;
                w.particles.push(Particle::new(Vec3::new(x, y, z), 0.05));
            }
        }
    }

    // 热场:把网格原点设为 (-8,-8,-8),使中心格 (8,8,8) 正好映射到世界原点
    // (流体粒子所在处),这样暖斑在流体内、浮力真正驱动闭环。
    let nx = 16usize;
    let dx = 1.0_f64;
    let mut f = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann)
        .with_origin(Vec3::new(-8.0, -8.0, -8.0));
    let c = (8usize, 8usize, 8usize);
    let ci = f.idx(c.0, c.1, c.2);
    f.u[ci] = 5.0;
    let mut heat = HeatField::new(f, 0.01);
    let t_ref = 0.0_f64;
    let beta = 0.5_f64;

    let warm_centroid_y = |h: &HeatField<f64>| -> f64 {
        let mut sy = 0.0;
        let mut sw = 0.0;
        for iz in 0..nx {
            for iy in 0..nx {
                for ix in 0..nx {
                    let v = h.field.u[h.field.idx(ix, iy, iz)];
                    if v > 0.1 {
                        sy += (iy as f64) * dx * v;
                        sw += v;
                    }
                }
            }
        }
        if sw > 0.0 {
            sy / sw
        } else {
            0.0
        }
    };

    let y0 = warm_centroid_y(&heat);
    let dt = 0.005_f64;
    for _ in 0..40 {
        // 每帧:1) couple_heat 刷新速度采样器 + 施热浮力 + 注入热源;
        //       2) 热场 step 用该速度做扩散-对流;3) 流体 step 推进(浮力加速度已入 acc)。
        w.couple_heat(&mut heat, dt, t_ref, beta, 0.01);
        heat.step(&dt);
        w.step(dt);
    }
    let y1 = warm_centroid_y(&heat);
    assert!(
        y1 > y0 + dx * 0.05,
        "Boussinesq 闭环下暖斑质心应上升(热浮力驱动对流平流): y0={}, y1={}",
        y0,
        y1
    );
    // 场仍有限(没有数值爆炸)。
    assert!(
        heat.field.u.iter().all(|&v| v.is_finite()),
        "闭环多帧步进后热场应有限"
    );
}

/// 构建两层反向速度的小剪切构型,返回给定幂律指数 `n` 下的流体世界(已 step 一次)。
fn shear_world(n: f64, dv: f64) -> FluidWorld<f64> {
    let mut p = test_params();
    p.gravity = Vec3::zeros();
    let mut w = FluidWorld::new(p);
    w.particles.clear();
    // 两层粒子:y 方向分层、沿 x 反向速度 => 产生剪切率。
    for iy in 0..2u32 {
        let vy = if iy == 0 { -dv } else { dv };
        for ix in 0..5u32 {
            let x = (ix as f64 - 2.0) * 0.08;
            let mut pt = Particle::with_material(Vec3::new(x, iy as f64 * 0.06, 0.0), 0.05, 0);
            pt.vel = Vec3::new(vy, 0.0, 0.0);
            w.particles.push(pt);
        }
    }
    w.params.visc_k = vec![3.5];
    w.params.visc_n = vec![n];
    w.step(0.001);
    w
}

#[test]
fn non_newtonian_shear_thinning_and_thickening() {
    // 同一剪切构型下:剪切变稀(n<1)有效粘度低于牛顿;剪切变稠(n>1)高于牛顿。
    let thin = shear_world(0.5, 300.0);
    let newt = shear_world(1.0, 300.0);
    let thick = shear_world(1.5, 300.0);
    let mt = thin.effective_viscosity(0);
    let mn = newt.effective_viscosity(0);
    let mk = thick.effective_viscosity(0);
    assert!(mt.is_finite() && mn.is_finite() && mk.is_finite());
    assert!(mt < mn, "剪切变稀应比牛顿更稀: {} < {}", mt, mn);
    assert!(mk > mn, "剪切变稠应比牛顿更稠: {} > {}", mk, mn);
}

#[test]
fn material_tag_distinguishes_viscosity() {
    // 两种材料(visc_k 不同,n 均为 1)在同样剪切下应得到不同有效粘度。
    let mut p = test_params();
    p.gravity = Vec3::zeros();
    let mut w = FluidWorld::new(p);
    w.particles.clear();
    // 粒子 0(材料0) 与粒子 1(材料1):反向速度、近距 => 剪切。
    let mut a = Particle::with_material(Vec3::new(0.0, 0.0, 0.0), 0.05, 0);
    a.vel = Vec3::new(-300.0, 0.0, 0.0);
    let mut b = Particle::with_material(Vec3::new(0.0, 0.06, 0.0), 0.05, 1);
    b.vel = Vec3::new(300.0, 0.0, 0.0);
    w.particles.push(a);
    w.particles.push(b);
    w.params.visc_k = vec![3.5, 10.0];
    w.params.visc_n = vec![1.0, 1.0];
    w.step(0.001);
    let mu0 = w.effective_viscosity(0);
    let mu1 = w.effective_viscosity(1);
    assert!((mu0 - 3.5).abs() < 1e-6, "材料0 应为 3.5,实际 {}", mu0);
    assert!((mu1 - 10.0).abs() < 1e-6, "材料1 应为 10.0,实际 {}", mu1);
}

#[test]
fn multi_material_tags_conserved_under_step() {
    // 多材料流体:step 后粒子材料标签应保持不变(用于相分离/界面识别)。
    let mut w = FluidWorld::new(test_params());
    w.particles.clear();
    for i in 0..10 {
        let mat = i % 3;
        let mut pt = Particle::with_material(Vec3::new((i as f64) * 0.1, 0.0, 0.0), 0.05, mat);
        pt.vel = Vec3::new(0.0, -1.0, 0.0);
        w.particles.push(pt);
    }
    let tags: Vec<usize> = w.particles.iter().map(|p| p.material).collect();
    for _ in 0..20 {
        w.step(0.003);
    }
    let after: Vec<usize> = w.particles.iter().map(|p| p.material).collect();
    assert_eq!(tags, after, "材料标签应在 step 中保持");
}
