//! 光学双后端:离线 Whitted 光线追踪 / 实时近似。
//!
//! 二者实现同一 `Renderer` trait,以精度开关切换:
//! - `Whitted`:递归反射+折射(Snell + Schlick-Fresnel),适合离线科研图。
//! - `Approx`:单次折射近似 + 阴影射线,低开销,适合实时 Demo。

use num_traits::FromPrimitive;
use phy_math::{na, RealField, Vec3};

use crate::math::{f0_of, fresnel, normalize, reflect, refract};
use crate::scene::{OpticScene, Surface};

/// 渲染后端接口。
pub trait Renderer<T: RealField + Copy> {
    /// 追踪一条射线,返回其贡献的颜色(RGB,0..1)。
    fn trace(&self, scene: &OpticScene<T>, ro: &Vec3<T>, rd: &Vec3<T>) -> Vec3<T>;

    /// 后端名(用于调试)。
    fn name(&self) -> &'static str;
}

/// 由颜色通道做简单 gamma/钳制,保证输出在 0..1。
fn clamp_color<T: RealField + Copy>(c: &Vec3<T>) -> Vec3<T> {
    let one = T::one();
    let zero = T::zero();
    Vec3::new(
        c.x.max(zero).min(one),
        c.y.max(zero).min(one),
        c.z.max(zero).min(one),
    )
}

/// 离线 Whitted 追踪器(递归反射/折射)。
pub struct Whitted;

impl<T: RealField + Copy> Renderer<T> for Whitted {
    fn name(&self) -> &'static str {
        "whitted"
    }

    fn trace(&self, scene: &OpticScene<T>, ro: &Vec3<T>, rd: &Vec3<T>) -> Vec3<T> {
        self.trace_rec(scene, ro, rd, scene.env_ior, 0)
    }
}

impl Whitted {
    fn trace_rec<T: RealField + Copy>(
        &self,
        scene: &OpticScene<T>,
        ro: &Vec3<T>,
        rd: &Vec3<T>,
        cur_ior: T,
        depth: usize,
    ) -> Vec3<T> {
        let one = T::one();
        if depth >= scene.max_depth {
            return scene.background;
        }
        let hit = match scene.intersect(ro, rd) {
            Some(h) => h,
            None => return scene.background,
        };
        let (t, n, idx) = hit;
        let body = &scene.bodies[idx];
        let surf = &body.surface;
        let p = *ro + *rd * t;

        // 判断进入还是离开(法线需指向入射侧)。
        let cosi = -rd.dot(&n);
        let (n_face, entering, eta, n_from, n_to) = if cosi > T::zero() {
            (n, true, cur_ior / surf.ior, cur_ior, surf.ior)
        } else {
            (-n, false, surf.ior / cur_ior, surf.ior, cur_ior)
        };
        let abs_cosi = cosi.abs();
        let f0 = f0_of(n_from, n_to);

        if surf.transparent {
            // 折射 + 反射(带 Fresnel 权重)。
            let fr = fresnel(abs_cosi, f0);
            let reflected = reflect(rd, &n_face);
            let ro2 = p + reflected * epsilon::<T>();
            let refl_col = self.trace_rec(scene, &ro2, &reflected, cur_ior, depth + 1);

            let mut col = refl_col * fr;
            if let Some(t_dir) = refract(rd, &n_face, eta) {
                let ft = one - fr;
                let ro3 = p + t_dir * epsilon::<T>();
                // 透射介质切换:进入时换成物体 IOR,离开时换回环境。
                let next_ior = if entering { surf.ior } else { scene.env_ior };
                let trans_col = self.trace_rec(scene, &ro3, &t_dir, next_ior, depth + 1);
                col += (trans_col * ft).component_mul(&surf.albedo);
            }
            clamp_color(&col)
        } else {
            // 不透明:漫反射 + 阴影 + 一点镜面高光。
            let light_dir = normalize(&Vec3::new(
                T::from_f64(0.5).unwrap(),
                T::one(),
                T::from_f64(0.3).unwrap(),
            ));
            let diff = n_face.dot(&light_dir).max(T::zero());
            let shade = if self.in_shadow(scene, &p, &light_dir) {
                T::from_f64(0.15).unwrap() // 环境光
            } else {
                T::from_f64(0.15).unwrap() + diff * T::from_f64(0.85).unwrap()
            };
            clamp_color(&(surf.albedo * shade))
        }
    }

    /// 阴影射线:从 `p` 朝 `light_dir` 是否有不透明遮挡。
    fn in_shadow<T: RealField + Copy>(
        &self,
        scene: &OpticScene<T>,
        p: &Vec3<T>,
        light_dir: &Vec3<T>,
    ) -> bool {
        let ro = *p + *light_dir * epsilon::<T>();
        if let Some((_, _, idx)) = scene.intersect(&ro, light_dir) {
            return scene.bodies[idx].surface.transparent == false;
        }
        false
    }
}

/// 实时近似追踪器:单次折射近似 + 阴影射线(无递归)。
pub struct Approx;

impl<T: RealField + Copy> Renderer<T> for Approx {
    fn name(&self) -> &'static str {
        "approx"
    }

    fn trace(&self, scene: &OpticScene<T>, ro: &Vec3<T>, rd: &Vec3<T>) -> Vec3<T> {
        let hit = match scene.intersect(ro, rd) {
            Some(h) => h,
            None => return scene.background,
        };
        let (t, n, idx) = hit;
        let body = &scene.bodies[idx];
        let surf = &body.surface;
        let p = *ro + *rd * t;

        let cosi = -rd.dot(&n);
        let abs_cosi = cosi.abs();

        if surf.transparent {
            // 单次折射近似:折射后再打一条环境射线取背景/透射色。
            let eta = scene.env_ior / surf.ior;
            let fr = fresnel(abs_cosi, f0_of(scene.env_ior, surf.ior));
            let refl = reflect(rd, &n);
            let ro_r = p + refl * epsilon::<T>();
            let refl_col = match scene.intersect(&ro_r, &refl) {
                None => scene.background,
                Some((_, n2, i2)) => {
                    let s2 = &scene.bodies[i2].surface;
                    let lambert = n2
                        .dot(&Vec3::new(T::zero(), T::one(), T::zero()))
                        .max(T::zero())
                        * T::from_f64(0.5).unwrap();
                    s2.albedo * (T::from_f64(0.5).unwrap() + lambert)
                }
            };
            let trans_col = if let Some(t_dir) = refract(rd, &n, eta) {
                let ro_t = p + t_dir * epsilon::<T>();
                match scene.intersect(&ro_t, &t_dir) {
                    None => scene.background.component_mul(&surf.albedo), // 透射到背景(带玻璃染色)
                    Some((_, _, i2)) => {
                        scene.bodies[i2].surface.albedo.component_mul(&surf.albedo)
                    }
                }
            } else {
                refl_col // TIR
            };
            let ft = T::one() - fr;
            clamp_color(&(refl_col * fr + trans_col * ft))
        } else {
            // 不透明:朗伯 + 阴影。
            let light_dir = normalize(&Vec3::new(
                T::from_f64(0.5).unwrap(),
                T::one(),
                T::from_f64(0.3).unwrap(),
            ));
            let diff = n.dot(&light_dir).max(T::zero());
            let shade = if self.in_shadow(scene, &p, &light_dir) {
                T::from_f64(0.2).unwrap()
            } else {
                T::from_f64(0.2).unwrap() + diff * T::from_f64(0.8).unwrap()
            };
            clamp_color(&(surf.albedo * shade))
        }
    }
}

impl Approx {
    fn in_shadow<T: RealField + Copy>(
        &self,
        scene: &OpticScene<T>,
        p: &Vec3<T>,
        light_dir: &Vec3<T>,
    ) -> bool {
        let ro = *p + *light_dir * epsilon::<T>();
        if let Some((_, _, idx)) = scene.intersect(&ro, light_dir) {
            return scene.bodies[idx].surface.transparent == false;
        }
        false
    }
}

/// 偏移 epsilon,避免自交。
fn epsilon<T: RealField + Copy>() -> T {
    T::from_f64(1e-4).unwrap()
}
