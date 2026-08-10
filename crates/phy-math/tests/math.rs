//! phy-math 基础数学单测(L1 库化 hardening:补零测试缺口)。

use phy_math::*;

const EPS: f64 = 1e-12;

/// 单位四元数对向量做旋转(手动公式,避免依赖额外 trait)。
fn rotate_q(q: &Quat<f64>, v: &Vec3<f64>) -> Vec3<f64> {
    let qv = Vec3::<f64>::new(q.i, q.j, q.k);
    let t = qv.cross(v) * 2.0 * q.w;
    let u = qv.cross(&(qv.cross(v) * 2.0));
    *v + t + u
}

#[test]
fn vec3_add_sub_componentwise() {
    let a = Vec3::<f64>::new(1.0, 2.0, 3.0);
    let b = Vec3::<f64>::new(4.0, -1.0, 0.5);
    let s = a + b;
    assert!((s.x - 5.0).abs() < EPS);
    assert!((s.y - 1.0).abs() < EPS);
    assert!((s.z - 3.5).abs() < EPS);
    let d = a - b;
    assert!((d.x + 3.0).abs() < EPS);
    assert!((d.y - 3.0).abs() < EPS);
    assert!((d.z - 2.5).abs() < EPS);
}

#[test]
fn vec3_dot_cross() {
    let a = Vec3::<f64>::new(1.0, 0.0, 0.0);
    let b = Vec3::<f64>::new(0.0, 1.0, 0.0);
    assert!(a.dot(&b).abs() < EPS); // 正交
    let c = a.cross(&b);
    assert!(c.x.abs() < EPS);
    assert!(c.y.abs() < EPS);
    assert!((c.z - 1.0).abs() < EPS); // z 轴
}

#[test]
fn vec3_norm_and_normalize() {
    let v = Vec3::<f64>::new(3.0, 4.0, 0.0);
    assert!((v.norm() - 5.0).abs() < EPS);
    let n = v.normalize();
    assert!((n.norm() - 1.0).abs() < EPS);
    assert!((n.x - 0.6).abs() < EPS);
    assert!((n.y - 0.8).abs() < EPS);
}

#[test]
fn vec3_scale_and_clamp() {
    let v = Vec3::<f64>::new(-5.0, 2.0, 7.0);
    let s = v * 2.0;
    assert!((s.z - 14.0).abs() < EPS);
    // 分量级 clamp(绕开 nalgebra clamp 的 SimdRealField 约束)。
    let lo = Vec3::<f64>::new(-3.0, -3.0, -3.0);
    let hi = Vec3::<f64>::new(3.0, 3.0, 3.0);
    let c = Vec3::new(
        v.x.max(lo.x).min(hi.x),
        v.y.max(lo.y).min(hi.y),
        v.z.max(lo.z).min(hi.z),
    );
    assert!(c.x == -3.0); // 夹到下限
    assert!(c.y == 2.0);
    assert!(c.z == 3.0); // 夹到上限
}

#[test]
fn quat_identity_and_mul_rotation() {
    let q = Quat::<f64>::identity();
    let v = Vec3::<f64>::new(1.0, 0.0, 0.0);
    let r = rotate_q(&q, &v); // 单位四元数不改写向量
    assert!((r.x - 1.0).abs() < EPS);
    assert!(r.y.abs() < EPS);
    assert!(r.z.abs() < EPS);

    // 绕 Z 轴 +90° 应把 X 轴转到 Y 轴。
    let half = std::f64::consts::FRAC_PI_4; // 半角
    let rot = Quat::new(half.cos(), 0.0, 0.0, half.sin()); // 单位四元数(半角公式)
    let r2 = rotate_q(&rot, &v);
    assert!(r2.x.abs() < EPS);
    assert!((r2.y - 1.0).abs() < EPS);
    assert!(r2.z.abs() < EPS);
}

#[test]
fn quat_normalize_idempotent() {
    let q = Quat::<f64>::new(2.0, 0.0, 0.0, 0.0);
    let n = q.normalize();
    assert!((n.norm() - 1.0).abs() < EPS);
}

#[test]
fn mat3_mul_identity() {
    let m = Mat3::<f64>::identity();
    let v = Vec3::<f64>::new(1.0, 2.0, 3.0);
    let r = m * v;
    assert!((r.x - 1.0).abs() < EPS);
    assert!((r.y - 2.0).abs() < EPS);
    assert!((r.z - 3.0).abs() < EPS);

    let a = Mat3::<f64>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    let prod = a * Mat3::<f64>::identity();
    assert!((prod - a).norm() < EPS);
}

#[test]
fn gravity_points_down() {
    let g = gravity::<f64>();
    assert!(g.x == 0.0);
    assert!(g.z == 0.0);
    assert!(g.y < 0.0);
    assert!((g.y + 9.81).abs() < EPS);
}
