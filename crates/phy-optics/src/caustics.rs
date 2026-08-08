//! 焦散(caustics)渲染(M17):平行光经透明体折射后在接收面上聚集形成的亮纹。
//!
//! 做法:在接收面(默认水平地面)上方均匀采样一组平行光射线(方向 = `-light_dir`,
//! 自上方射入),逐条做折射行进(refraction march):穿过透明体时按 Fresnel 透光率
//! 衰减能量,命中不透明接收面时把剩余通量累加到对应网格单元。最后返回二维强度网格,
//! 供可视化或导出。
//!
//! 这是离线焦散的经典"正向光线"近似:不递归、不做颜色,只关心能量分布。

use phy_math::{na, RealField, Vec3};

use crate::math::{f0_of, fresnel, normalize, refract};
use crate::scene::OpticScene;

/// 焦散计算:平行光正向行进,累加透射能量到接收面网格。
pub struct Caustics;

impl Caustics {
    /// 计算接收面(`y = plane_y`)上的焦散强度网格。
    ///
    /// - `light_dir`:指向光源的单位向量(光线实际传播方向为 `-light_dir`)。
    /// - `half_extent`:接收面采样半宽(对称 [-h,h]×[-h,h])。
    /// - `grid_n`:每轴网格数(`(2h)/grid_n` 为单元尺寸)。
    /// - 返回 `(grid, max_val)`:`grid[j][i]` 为 (x_i, z_j) 处累计通量,`max_val` 为峰值。
    pub fn accumulate<T: RealField + Copy>(
        &self,
        scene: &OpticScene<T>,
        light_dir: &Vec3<T>,
        plane_y: T,
        half_extent: T,
        grid_n: usize,
    ) -> (Vec<Vec<T>>, T) {
        let ldir = normalize(light_dir);
        // 光线传播方向 = 指向光源的反向(从上方射向接收面)。
        let travel = -ldir;
        let zero = T::zero();
        let one = T::one();
        let eps = T::from_f64(1e-4).unwrap();

        let mut grid: Vec<Vec<T>> = vec![vec![zero; grid_n]; grid_n];
        let mut max_val = zero;

        let cell = (half_extent * T::from_f64(2.0).unwrap()) / T::from_usize(grid_n).unwrap();
        // 射线起点高度:接收面上方一个足够覆盖所有光学体的距离。
        let top = plane_y + T::from_f64(20.0).unwrap();

        for j in 0..grid_n {
            for i in 0..grid_n {
                let x = -half_extent + (T::from_usize(i).unwrap() + one / T::from_f64(2.0).unwrap()) * cell;
                let z = -half_extent + (T::from_usize(j).unwrap() + one / T::from_f64(2.0).unwrap()) * cell;
                let ro0 = Vec3::new(x, top, z);
                let throughput = self.march(scene, &ro0, &travel);
                grid[j][i] = throughput;
                if throughput > max_val {
                    max_val = throughput;
                }
            }
        }
        (grid, max_val)
    }

    /// 单条光线的折射行进:返回命中接收面时的累计透射通量。
    fn march<T: RealField + Copy>(
        &self,
        scene: &OpticScene<T>,
        ro: &Vec3<T>,
        rd: &Vec3<T>,
    ) -> T {
        let eps = T::from_f64(1e-4).unwrap();
        let one = T::one();
        let mut ro = *ro;
        let mut rd = *rd;
        let mut cur_ior = scene.env_ior;
        let mut flux = one;
        // 限制折射段数,避免极端几何下死循环。
        for _ in 0..8 {
            let hit = match scene.intersect(&ro, &rd) {
                Some(h) => h,
                None => return T::zero(),
            };
            let (t, n, idx) = hit;
            let surf = &scene.bodies[idx].surface;
            let p = ro + rd * t;
            let cosi = -rd.dot(&n);
            let (n_face, entering, eta, n_from, n_to) = if cosi > T::zero() {
                (n, true, cur_ior / surf.ior, cur_ior, surf.ior)
            } else {
                (-n, false, surf.ior / cur_ior, surf.ior, cur_ior)
            };
            let abs_cosi = cosi.abs();
            let f0 = f0_of(n_from, n_to);

            if surf.transparent {
                // 透射分支:能量按 (1-Fresnel) 衰减,进入/离开切换介质 IOR。
                let ft = one - fresnel(abs_cosi, f0);
                flux = flux * ft;
                let t_dir = match refract(&rd, &n_face, eta) {
                    Some(d) => d,
                    None => return T::zero(), // 全内反射 → 能量不落到接收面
                };
                cur_ior = if entering { surf.ior } else { scene.env_ior };
                ro = p + t_dir * eps;
                rd = t_dir;
                // 继续穿过透明体,寻找后续命中。
            } else {
                // 命中不透明接收面:把剩余通量作为该处焦散亮度返回。
                return flux;
            }
        }
        T::zero()
    }
}

/// 把焦散网格写成 CSV(强度值,网格坐标隐式为单元索引)。
#[cfg(feature = "io_csv")]
pub fn write_caustics_csv<T: RealField + Copy + std::fmt::Display, P: AsRef<std::path::Path>>(
    path: P,
    grid: &[Vec<T>],
) -> std::io::Result<()> {
    use std::fs::File;
    use std::io::Write;
    let mut f = File::create(path)?;
    for row in grid {
        let line: Vec<String> = row.iter().map(|v| v.to_string()).collect();
        f.write_all(line.join(",").as_bytes())?;
        f.write_all(b"\n")?;
    }
    Ok(())
}
