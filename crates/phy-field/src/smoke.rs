//! 烟气/多相被动标量场子系统(S5 前半:气体/烟雾多相与浮力)。
//!
//! `SmokeField` 是 `grid::ScalarField` 的 `Subsystem` 适配层,模拟一团随流体
//! 平流、并因自身比空气轻而**上浮**的烟/气被动标量(浓度 `s≥0`)。
//!
//! 与 `HeatField`(温度浮力作用于流体)不同,这里把浮力直接体现为烟气的
//! **有效平流速度附加项**:`v_eff = v_fluid + buoyancy·s·ŷ`,使浓烟上升、
//! 淡烟随风,自然形成烟羽。无需改动流体求解器即可耦合。
//!
//! 燃烧火焰前缘推进(点燃/消耗)为 S5 后半,留待后续扩展。

use phy_core::Subsystem;
use phy_math::{RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::grid_geometry::GridGeometry;
use crate::grid::ScalarField;
use crate::world_to_cell;

/// 对一维行主序 3D 标量网格做 `passes` 次 3×3×3 盒式模糊(边界钳制)。
/// 用于把浮力上升源场平滑到烟团上方数格,使半拉格朗日平流能真正把烟抬升。
fn blur3<T: RealField + Copy>(src: &[T], nx: usize, ny: usize, nz: usize, passes: usize) -> Vec<T> {
    let mut cur = src.to_vec();
    for _ in 0..passes {
        let mut next = vec![T::zero(); cur.len()];
        for z in 0..nz {
            for y in 0..ny {
                for x in 0..nx {
                    let mut sum = T::zero();
                    let mut cnt = 0usize;
                    for dz in -1i64..=1 {
                        for dy in -1i64..=1 {
                            for dx in -1i64..=1 {
                                let xx = x as i64 + dx;
                                let yy = y as i64 + dy;
                                let zz = z as i64 + dz;
                                if xx < 0
                                    || yy < 0
                                    || zz < 0
                                    || xx >= nx as i64
                                    || yy >= ny as i64
                                    || zz >= nz as i64
                                {
                                    continue;
                                }
                                sum += cur
                                    [(xx as usize) + nx * ((yy as usize) + ny * (zz as usize))];
                                cnt += 1;
                            }
                        }
                    }
                    let inv = T::from_usize(cnt).unwrap_or(T::one());
                    next[x + nx * (y + ny * z)] = sum / inv;
                }
            }
        }
        cur = next;
    }
    cur
}

/// 烟/气被动标量场子系统。
#[derive(Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar")]
pub struct SmokeField<T: RealField + Copy> {
    /// 底层标量场(烟浓度 s)。
    pub field: ScalarField<T>,
    /// 烟扩散率(湍流扩散,通常很小)。
    pub diffusion: T,
    /// 浮力系数:每单位浓度向上的附加速度(烟比空气轻)。
    pub buoyancy: T,
    /// 上次步进的稳定性数 r=α·dt/dx²。
    pub last_r: T,
    /// 流体速度场采样器(世界坐标 → 速度)。由流体耦合写入,使烟随流平流。
    /// 闭包不可序列化,存档时跳过,加载后由耦合方重新安装。
    #[serde(skip)]
    pub vel_sampler: Option<Box<dyn Fn(Vec3<T>) -> Vec3<T>>>,
}

impl<T: RealField + Copy + std::fmt::Debug> std::fmt::Debug for SmokeField<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SmokeField")
            .field("field", &self.field)
            .field("diffusion", &self.diffusion)
            .field("buoyancy", &self.buoyancy)
            .field("last_r", &self.last_r)
            .field("vel_sampler", &self.vel_sampler.is_some())
            .finish()
    }
}

impl<T: RealField + Copy> Clone for SmokeField<T> {
    fn clone(&self) -> Self {
        SmokeField {
            field: self.field.clone(),
            diffusion: self.diffusion,
            buoyancy: self.buoyancy,
            last_r: self.last_r,
            vel_sampler: None,
        }
    }
}

impl<T: RealField + Copy + num_traits::ToPrimitive> SmokeField<T> {
    /// 构造烟场适配层。
    pub fn new(field: ScalarField<T>, diffusion: T, buoyancy: T) -> Self {
        Self {
            field,
            diffusion,
            buoyancy,
            last_r: T::zero(),
            vel_sampler: None,
        }
    }

    /// 安装流体速度场采样器(典型来自 SPH 速度场)。传入 `None` 退化为纯扩散。
    pub fn set_vel_sampler(&mut self, f: Option<Box<dyn Fn(Vec3<T>) -> Vec3<T>>>) {
        self.vel_sampler = f;
    }

    /// 注入一团烟(向格点累加浓度)。
    pub fn emit(&mut self, cx: usize, cy: usize, cz: usize, amount: T) {
        self.field.add_source(cx, cy, cz, amount);
    }

    /// 世界坐标处三线性采样烟浓度(越界夹紧到边界格)。
    pub fn conc_at_world(&self, p: Vec3<T>) -> T
    where
        T: num_traits::ToPrimitive,
    {
        let (cx, cy, cz, tx, ty, tz) = world_to_cell(self, p);
        let fx = T::from_f64(tx).unwrap_or(T::zero());
        let fy = T::from_f64(ty).unwrap_or(T::zero());
        let fz = T::from_f64(tz).unwrap_or(T::zero());
        let u = &self.field.u;
        let nx = self.field.nx;
        let ny = self.field.ny;
        let nz = self.field.nz;
        let smp = |x: usize, y: usize, z: usize| -> T {
            u[x + nx * (y + ny * z)]
        };
        let c000 = smp(cx, cy, cz);
        let c100 = smp((cx + 1).min(nx - 1), cy, cz);
        let c010 = smp(cx, (cy + 1).min(ny - 1), cz);
        let c110 = smp((cx + 1).min(nx - 1), (cy + 1).min(ny - 1), cz);
        let c001 = smp(cx, cy, (cz + 1).min(nz - 1));
        let c101 = smp((cx + 1).min(nx - 1), cy, (cz + 1).min(nz - 1));
        let c011 = smp(cx, (cy + 1).min(ny - 1), (cz + 1).min(nz - 1));
        let c111 = smp((cx + 1).min(nx - 1), (cy + 1).min(ny - 1), (cz + 1).min(nz - 1));
        let x00 = c000 + (c100 - c000) * fx;
        let x10 = c010 + (c110 - c010) * fx;
        let x01 = c001 + (c101 - c001) * fx;
        let x11 = c011 + (c111 - c011) * fx;
        let y0 = x00 + (x10 - x00) * fy;
        let y1 = x01 + (x11 - x01) * fy;
        y0 + (y1 - y0) * fz
    }

    /// 有效平流速度 = 流体速度 + 浮力附加项(`buoyancy·s·ŷ`)。
    /// 仅作诊断/外部查询用;实际步进由 `step` 内部的回溯点采样实现。
    pub fn eff_vel(&self, p: Vec3<T>) -> Vec3<T> {
        let mut v = match &self.vel_sampler {
            Some(f) => f(p),
            None => Vec3::zeros(),
        };
        if self.buoyancy > T::zero() {
            let s = self.conc_at_world(p);
            v.y += self.buoyancy * s;
        }
        v
    }
}

impl<T: RealField + Copy> GridGeometry<T> for SmokeField<T> {
    fn dims(&self) -> (usize, usize, usize) {
        (self.field.nx, self.field.ny, self.field.nz)
    }
    fn origin(&self) -> Vec3<T> {
        self.field.origin
    }
    fn cell_size(&self) -> T {
        self.field.dx
    }
}

impl<T: RealField + Copy + num_traits::ToPrimitive> Subsystem<T> for SmokeField<T> {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
    fn step(&mut self, dt: &T) {
        if self.vel_sampler.is_some() {
            // 扩散-对流混合步:流速 + 浓度依赖浮力上浮。
            // 半拉格朗日平流在内部会回溯源场;若闭包同时读取正在被写入的 self.field
            // 会触发借用冲突,故先把当前浓度网格拷成只读快照供浮力采样。
            let buoy = self.buoyancy;
            // 取出速度采样器本地持有(闭包只借用它 + 浓度只读快照,不借用 self.field),
            // 步进结束后再放回 self.vel_sampler。
            let sampler = self.vel_sampler.take();
            let sampler_ref = &sampler;
            let snap_u = self.field.u.clone();
            let (nx, ny, nz) = (self.field.nx, self.field.ny, self.field.nz);
            let dx = self.field.dx;
            let o = self.field.origin;
            // 把浓度快照做几次盒式模糊,得到"浮力上升源场"。浮力驱动的上升气流
            // 具有有限粘性尺度,会延伸到烟团上方数格;若仅按局地浓度取速度,半拉格朗日
            // 平流无法把烟从 smoky 格抬到上方空格(回溯点落在空格内取不到烟)。模糊后
            // 烟团正上方也获得上升速度,烟便真正上浮形成烟羽。
            let up_field = blur3(&snap_u, nx, ny, nz, 3);
            let eff = move |p: Vec3<T>| -> Vec3<T> {
                // 速度 = 流体速度(经原始闭包)+ 浓度依赖浮力上浮。
                let fluid = match sampler_ref {
                    Some(f) => f(p),
                    None => Vec3::zeros(),
                };
                let mut v = fluid;
                if buoy > T::zero() {
                    // 仅由流体速度估计回溯点(浮力项量级小,忽略其自反馈)。
                    let back = p - fluid * *dt;
                    let fx = ((back.x - o.x) / dx).to_f64().unwrap_or(0.0);
                    let fy = ((back.y - o.y) / dx).to_f64().unwrap_or(0.0);
                    let fz = ((back.z - o.z) / dx).to_f64().unwrap_or(0.0);
                    let clamp_idx = |f: f64, n: usize| -> (usize, f64) {
                        if n <= 1 {
                            return (0, 0.0);
                        }
                        if f <= 0.0 {
                            (0, 0.0)
                        } else if f >= (n - 1) as f64 {
                            (n - 2, 1.0)
                        } else {
                            let fl = f.floor();
                            (fl as usize, f - fl)
                        }
                    };
                    let (cx, tx) = clamp_idx(fx, nx);
                    let (cy, ty) = clamp_idx(fy, ny);
                    let (cz, tz) = clamp_idx(fz, nz);
                    let smp = |x: usize, y: usize, z: usize| -> T {
                        up_field[x + nx * (y + ny * z)]
                    };
                    let fx = T::from_f64(tx).unwrap_or(T::zero());
                    let fy = T::from_f64(ty).unwrap_or(T::zero());
                    let fz = T::from_f64(tz).unwrap_or(T::zero());
                    let c000 = smp(cx, cy, cz);
                    let c100 = smp((cx + 1).min(nx - 1), cy, cz);
                    let c010 = smp(cx, (cy + 1).min(ny - 1), cz);
                    let c110 = smp((cx + 1).min(nx - 1), (cy + 1).min(ny - 1), cz);
                    let c001 = smp(cx, cy, (cz + 1).min(nz - 1));
                    let c101 = smp((cx + 1).min(nx - 1), cy, (cz + 1).min(nz - 1));
                    let c011 = smp(cx, (cy + 1).min(ny - 1), (cz + 1).min(nz - 1));
                    let c111 = smp((cx + 1).min(nx - 1), (cy + 1).min(ny - 1), (cz + 1).min(nz - 1));
                    let x00 = c000 + (c100 - c000) * fx;
                    let x10 = c010 + (c110 - c010) * fx;
                    let x01 = c001 + (c101 - c001) * fx;
                    let x11 = c011 + (c111 - c011) * fx;
                    let y0 = x00 + (x10 - x00) * fy;
                    let y1 = x01 + (x11 - x01) * fy;
                    let s = y0 + (y1 - y0) * fz;
                    v.y += buoy * s;
                }
                v
            };
            self.last_r = self.field.step_advection_diffusion(&eff, self.diffusion, *dt);
            // 把速度采样器放回(若闭包期间未被移动消耗)。
            self.vel_sampler = sampler;
        } else {
            self.last_r = self.field.step_diffusion(self.diffusion, *dt);
        }
    }
    fn name(&self) -> &'static str {
        "smoke"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Bc;

    #[test]
    fn smoke_advects_with_fluid_velocity() {
        // 安装水平速度场,烟浓度应随流平流(无浮力时纯随流)。
        let mut field = ScalarField::<f64>::new(16, 16, 16, 0.5, 0.0, Bc::Neumann);
        let ci = field.idx(4, 8, 8);
        field.u[ci] = 1.0;
        let mut smoke = SmokeField::new(field, 0.0, 0.0); // 无浮力
        smoke.set_vel_sampler(Some(Box::new(|_p: Vec3<f64>| Vec3::new(0.5, 0.0, 0.0))));
        smoke.step(&1.0); // dt=1 => 平移 1 dx
        let new_ci = smoke.field.idx(5.min(15), 8, 8);
        assert!(
            smoke.field.u[new_ci] > 0.8,
            "烟随流平流后峰应到 i=5, 实测={}",
            smoke.field.u[new_ci]
        );
        assert!(smoke.field.u[ci] < 0.2, "原位置烟应被带走, 实测={}", smoke.field.u[ci]);
    }

    #[test]
    fn smoke_rises_with_buoyancy() {
        // 浮力>0 时,即便流体静止,浓烟也应向上漂移(y 增大)。
        let mut field = ScalarField::<f64>::new(16, 16, 16, 0.5, 0.0, Bc::Neumann);
        // 在底部中央放一团烟。
        let ci = field.idx(8, 2, 8);
        field.u[ci] = 1.0;
        let mut smoke = SmokeField::new(field, 0.0, 1.0); // buoyancy=1.0
        smoke.set_vel_sampler(Some(Box::new(|_p: Vec3<f64>| Vec3::zeros()))); // 流体静止
        let cy0 = centroid_y(&smoke);
        for _ in 0..10 {
            smoke.step(&0.5);
        }
        let cy1 = centroid_y(&smoke);
        assert!(cy1 > cy0, "有浮力时烟团质心应上升: {} -> {}", cy0, cy1);
    }

    /// 无浮力、流体静止时烟团质心应基本不动(只扩散)。
    #[test]
    fn smoke_no_buoyancy_stays_put() {
        let mut field = ScalarField::<f64>::new(16, 16, 16, 0.5, 0.0, Bc::Neumann);
        let ci = field.idx(8, 8, 8);
        field.u[ci] = 1.0;
        let mut smoke = SmokeField::new(field, 0.0, 0.0); // 无浮力
        smoke.set_vel_sampler(Some(Box::new(|_p: Vec3<f64>| Vec3::zeros()))); // 流体静止
        let cy0 = centroid_y(&smoke);
        for _ in 0..10 {
            smoke.step(&0.5);
        }
        let cy1 = centroid_y(&smoke);
        assert!(
            (cy1 - cy0).abs() < 1.0,
            "无浮力静止时烟团质心应基本不动: {} -> {}",
            cy0,
            cy1
        );
    }

    fn centroid_y(s: &SmokeField<f64>) -> f64 {
        let nx = s.field.nx;
        let ny = s.field.ny;
        let nz = s.field.nz;
        let mut wsum = 0.0;
        let mut ysum = 0.0;
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let v = s.field.sample(i, j, k);
                    wsum += v;
                    ysum += v * j as f64;
                }
            }
        }
        if wsum > 1e-12 {
            ysum / wsum
        } else {
            0.0
        }
    }
}
