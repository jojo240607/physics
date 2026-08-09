//! # phy-optics
//!
//! 光学双后端(离线光路追踪 / 实时近似),以精度开关切换。
//!
//! - `scene`:光学场景数据模型(复用 `phy_rigid::Shape` 几何 + 表面材质/介质折射率),
//!   并提供射线-球/盒/凸体求交。
//! - `math`:折射(Snell)/反射/Fresnel 等光学几何。
//! - `renderer`:`Whitted`(递归反射+折射,离线高保真)与 `Approx`(单次折射+阴影,实时)。
//! - `subsystem`:把 `OpticScene` 适配为 `phy_core::Subsystem`,并含 `render_camera` 离线成像辅助。
//!
//! 用法示例:
//! ```ignore
//! let mut scene = OpticScene::<f64>::new();
//! scene.add(OpticBody::new(sphere_body, Surface::glass(1.5, Vec3::new(1.0,1.0,1.0))));
//! let sub = OpticSubsystem::new(scene, Precision::Offline);
//! ```

mod math;
mod renderer;
mod scene;
mod subsystem;
mod caustics;

pub use caustics::Caustics;

pub use math::{f0_of, fresnel, normalize, reflect, refract};
pub use renderer::{Approx, Renderer, Whitted};
pub use scene::{OpticBody, OpticScene, Surface};
pub use subsystem::{OpticSubsystem, Precision};

/// 把颜色(Vec3<f64>,0..1)转成 u32 ARGB(供软件帧缓冲,0xAARRGGBB)。
pub fn to_rgba8(c: &phy_math::Vec3<f64>) -> u32 {
    let clamp = |x: f64| (x.max(0.0).min(1.0) * 255.0).round() as u32;
    let r = clamp(c.x);
    let g = clamp(c.y);
    let b = clamp(c.z);
    0xFF00_0000 | (r << 16) | (g << 8) | b
}

#[cfg(test)]
mod tests {
    use super::*;
    use phy_math::{na, Vec3};
    use phy_rigid::{Body, Shape};

    fn body_sphere(r: f64) -> Body<f64> {
        Body {
            shape: Shape::Sphere { r },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            inv_mass: 0.0,
        }
    }

    #[test]
    fn refract_bends_toward_normal_air_to_glass() {
        // 法向入射:折射方向应与入射反向(同向穿过),且 eta<1 时角度变。
        let i: Vec3<f64> = Vec3::new(0.0, 0.0, -1.0);
        let n: Vec3<f64> = Vec3::new(0.0, 0.0, 1.0); // 指向入射侧(+Z)
        let t = refract(&i, &n, 1.0_f64 / 1.5_f64).unwrap();
        // 法向入射时折射方向 = i 方向(继续传播)= (0,0,-1)
        assert!((t.z + 1.0).abs() < 1e-9_f64, "法向入射折射方向应为 (0,0,-1),得到 {}", t);
    }

    #[test]
    fn fresnel_in_range() {
        let f = fresnel(0.0_f64, 0.04);
        assert!(f >= 0.0 && f <= 1.0, "Fresnel 应在 [0,1],得到 {}", f);
        // 垂直入射(f0)应接近 f0。
        let f0 = fresnel(1.0_f64, 0.04);
        assert!((f0 - 0.04).abs() < 1e-9, "垂直入射 Fresnel 应=f0,得到 {}", f0);
    }

    #[test]
    fn sphere_intersect_hits_front() {
        let b = OpticBody::new(body_sphere(1.0), Surface::glass(1.5, Vec3::new(1.0, 1.0, 1.0)));
        // 从 +Z 朝原点打射线,应击中 z=+1 处,法线 (0,0,1)。
        let ro = Vec3::new(0.0, 0.0, 5.0);
        let rd = Vec3::new(0.0, 0.0, -1.0);
        let (t, n) = b.intersect_local(&ro, &rd).unwrap();
        assert!((t - 4.0).abs() < 1e-9, "t 应为 4,得到 {}", t);
        assert!((n.z - 1.0).abs() < 1e-9, "法线应为 (0,0,1),得到 {}", n);
    }

    #[test]
    fn offline_render_glass_sphere_non_background() {
        // 离线 Whitted:玻璃球应在画面中产生偏离背景的像素(折射/反射)。
        let mut scene = OpticScene::<f64>::new();
        scene.add(OpticBody::new(
            body_sphere(1.0),
            Surface::glass(1.5, Vec3::new(0.8, 0.9, 1.0)),
        ));
        let sub = OpticSubsystem::new(scene, Precision::Offline);
        let (w, h) = (32usize, 32usize);
        let mut buf = vec![Vec3::new(0.0, 0.0, 0.0); w * h];
        let eye = Vec3::new(0.0, 0.0, 5.0);
        let target = Vec3::new(0.0, 0.0, 0.0);
        let up = Vec3::new(0.0, 1.0, 0.0);
        sub.render_camera(&mut buf, w, h, &eye, &target, &up, std::f64::consts::FRAC_PI_4);
        let bg = buf[0]; // 角点应仍是背景
        let mut diff = 0usize;
        for c in &buf {
            if (c.x - bg.x).abs() > 1e-6 || (c.y - bg.y).abs() > 1e-6 || (c.z - bg.z).abs() > 1e-6 {
                diff += 1;
            }
        }
        assert!(diff > 0, "玻璃球应使部分像素偏离背景");
    }

    #[test]
    fn approx_render_lighter_than_opaque() {
        // 实时 Approx:透明球中心区域亮度应高于纯背景(透射背景光)。
        let mut scene = OpticScene::<f64>::new();
        scene.add(OpticBody::new(
            body_sphere(1.0),
            Surface::glass(1.33, Vec3::new(1.0, 1.0, 1.0)),
        ));
        let sub = OpticSubsystem::new(scene, Precision::Realtime);
        let (w, h) = (64usize, 64usize);
        let mut buf = vec![Vec3::new(0.0, 0.0, 0.0); w * h];
        let eye = Vec3::new(0.0, 0.0, 5.0);
        let target = Vec3::new(0.0, 0.0, 0.0);
        let up = Vec3::new(0.0, 1.0, 0.0);
        sub.render_camera(&mut buf, w, h, &eye, &target, &up, std::f64::consts::FRAC_PI_4);
        // 中心像素(穿过球体中心)亮度应高于角落背景。
        let center = buf[(h / 2) * w + w / 2];
        let corner = buf[0];
        let lum = |c: &Vec3<f64>| c.x + c.y + c.z;
        assert!(
            lum(&center) > lum(&corner),
            "透射中心亮度应高于背景: center={}, corner={}",
            lum(&center),
            lum(&corner)
        );
    }

    #[test]
    fn caustics_concentrates_light_under_glass() {
        // 焦散(M17):一束平行光自上而下穿过玻璃球,应在球体正下方地面形成亮斑,
        // 即网格中既有非零单元(被照到),峰值又显著高于平均值。
        let mut scene = OpticScene::<f64>::new();
        scene.add(OpticBody::new(
            body_sphere(1.0),
            Surface::glass(1.5, Vec3::new(1.0, 1.0, 1.0)),
        ));
        // 接收面:球下方的水平不透明地面(焦散落点)。
        scene.add(OpticBody::new(
            Body {
                shape: Shape::Box {
                    half: Vec3::new(20.0, 0.5, 20.0),
                },
                pos: Vec3::new(0.0, -3.0, 0.0),
                rot: na::UnitQuaternion::identity(),
                vel: Vec3::zeros(),
                inv_mass: 0.0,
            },
            Surface::diffuse(Vec3::new(0.9, 0.9, 0.9)),
        ));
        let caustics = Caustics;
        let light_dir = Vec3::new(0.0, 1.0, 0.0); // 光源在正上方
        let (grid, max_val) = caustics.accumulate(&scene, &light_dir, -3.0, 4.0, 64);
        let n = grid.len() * grid[0].len();
        let sum: f64 = grid.iter().flatten().map(|v| v).sum();
        let mean = sum / n as f64;
        // 存在被照亮的单元。
        assert!(max_val > 0.0, "应存在焦散亮度峰值");
        // 峰值明显高于均值(光线被聚拢成亮斑,而非均匀铺满)。
        assert!(
            max_val > mean * 1.5,
            "焦散应聚拢: max={}, mean={}",
            max_val,
            mean
        );
    }
}

