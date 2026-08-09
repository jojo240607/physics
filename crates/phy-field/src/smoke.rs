//! 烟/气被动标量场 + 燃烧(燃料/火焰前缘)子系统(S5a + S5b)。
//!
//! `SmokeField` 是 `grid::ScalarField` 的 `Subsystem` 适配层,对一团随流体平流、并因
//! 自身比空气轻而**上浮**的烟/气被动标量(浓度 `s≥0`)建模:
//!
//! ```text
//! ∂s/∂t + (v_fluid + buoyancy·s·ŷ)·∇s = α∇²s        (烟平流 + 浓度依赖浮力)
//! ```
//!
//! S5b 在其上叠加**燃烧**:一处额外的 `fuel`(可燃料密度)在场内随流平流扩散;当某格
//! 温度 ≥ `ignition_temp`(经共注册的 `HeatField` 在 `couple` 阶段读取,或经
//! `heat_sampler` 在单机场景读取)且仍有燃料时,燃料按 `fuel_burn_rate` 被消耗,等量转成
//! 烟(`flame_spawn`)并向热场释放热量(`heat_release`,在 `couple` 阶段经 `world` 注入
//! `HeatField::src`)。放热抬升邻格温度 → 点燃其燃料 → 形成**自持传播的火焰前缘**
//! (典型 Boussinesq 燃烧闭环)。

use num_traits::ToPrimitive;
use phy_math::{RealField, Vec3};
use phy_core::Subsystem;
use std::any::Any;

use crate::grid::ScalarField;
use crate::heat::HeatField;
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

/// 烟/气被动标量场 + 燃烧子系统。
#[derive(Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Default + Serialize + DeserializeOwned + nalgebra::Scalar")]
pub struct SmokeField<T: RealField + Copy> {
    /// 烟/气浓度场(s≥0),被平流 + 浓度依赖浮力抬升。
    pub field: ScalarField<T>,
    /// 扩散系数(烟的热/动量扩散代理)。
    pub diffusion: T,
    /// 浮力强度(浓度依赖上浮:`v_y += buoyancy·s`)。
    pub buoyancy: T,
    /// 燃料密度场(可燃气/可燃质,0 表示无可燃)。与烟共用网格。
    pub fuel: ScalarField<T>,
    /// 燃料消耗率(每秒消耗 = burn_rate·fuel)。
    pub fuel_burn_rate: T,
    /// 点燃温度阈值(温度 ≥ 此值的燃料格才会燃烧)。
    pub ignition_temp: T,
    /// 单位燃料燃烧释放的热量(注入热场,温度单位)。
    pub heat_release: T,
    /// 单位燃料燃烧生成的烟量(燃料 → 烟转换系数)。
    pub flame_spawn: T,
    /// 流体速度采样器(世界坐标 → 速度),None 表示静止流体。
    #[serde(skip)]
    pub vel_sampler: Option<Box<dyn Fn(Vec3<T>) -> Vec3<T>>>,
    /// 温度采样器(世界坐标 → 温度),供无 `World`/`HeatField` 的单机场景读取温度;
    /// 在 `World` 中由 `couple` 直接读取共注册 `HeatField`,此字段可留空。
    #[serde(skip)]
    pub heat_sampler: Option<Box<dyn Fn(Vec3<T>) -> T>>,
    /// 本帧燃烧放热累积(逐格),在 `couple` 阶段注入热场后清零。
    #[serde(skip)]
    pub heat_scratch: Vec<T>,
    /// 上一次平流/扩散步的稳定性数(诊断)。
    #[serde(skip)]
    pub last_r: T,
}

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

impl<T: RealField + Copy + ToPrimitive> SmokeField<T> {
    /// 构造 `nx×ny×nz` 烟场,初始烟浓度与燃料均为 0,原点默认 (0,0,0)。
    pub fn new(nx: usize, ny: usize, nz: usize, dx: T, diffusion: T, buoyancy: T) -> Self {
        let n = nx * ny * nz;
        Self {
            field: ScalarField::new(nx, ny, nz, dx, T::zero(), crate::Bc::Neumann),
            diffusion,
            buoyancy,
            fuel: ScalarField::new(nx, ny, nz, dx, T::zero(), crate::Bc::Neumann),
            fuel_burn_rate: T::from_f64(0.8).unwrap_or(T::zero()),
            ignition_temp: T::from_f64(300.0).unwrap_or(T::zero()),
            heat_release: T::from_f64(200.0).unwrap_or(T::zero()),
            flame_spawn: T::from_f64(1.0).unwrap_or(T::zero()),
            vel_sampler: None,
            heat_sampler: None,
            heat_scratch: vec![T::zero(); n],
            last_r: T::zero(),
        }
    }

    /// 设置网格原点(世界坐标)并返回 `self`,链式构造。
    pub fn with_origin(mut self, origin: Vec3<T>) -> Self {
        self.field = self.field.clone().with_origin(origin);
        self.fuel = self.fuel.clone().with_origin(origin);
        self
    }

    /// 安装流体速度采样器(世界坐标 → 速度)。None 表示静止流体。
    pub fn set_vel_sampler(&mut self, f: Box<dyn Fn(Vec3<T>) -> Vec3<T>>) {
        self.vel_sampler = Some(f);
    }

    /// 安装温度采样器(世界坐标 → 温度)。None 表示无热耦合(不燃烧)。
    pub fn set_heat_sampler(&mut self, f: Box<dyn Fn(Vec3<T>) -> T>) {
        self.heat_sampler = Some(f);
    }

    /// 在网格 `(cx,cy,cz)` 附近半径 `r`(格)的球内注入烟浓度 `amount`(叠加)。
    pub fn inject_blob(&mut self, cx: usize, cy: usize, cz: usize, r: usize, amount: T) {
        let r2 = (r * r) as i64;
        let (nx, ny, nz) = (self.field.nx, self.field.ny, self.field.nz);
        for z in 0..nz {
            for y in 0..ny {
                for x in 0..nx {
                    let dx = x as i64 - cx as i64;
                    let dy = y as i64 - cy as i64;
                    let dz = z as i64 - cz as i64;
                    if dx * dx + dy * dy + dz * dz <= r2 {
                        let i = self.field.idx(x, y, z);
                        self.field.u[i] += amount;
                    }
                }
            }
        }
    }

    /// 在网格 `(cx,cy,cz)` 附近半径 `r`(格)的球内注入燃料 `amount`(叠加)。
    pub fn inject_fuel(&mut self, cx: usize, cy: usize, cz: usize, r: usize, amount: T) {
        let r2 = (r * r) as i64;
        let (nx, ny, nz) = (self.fuel.nx, self.fuel.ny, self.fuel.nz);
        for z in 0..nz {
            for y in 0..ny {
                for x in 0..nx {
                    let dx = x as i64 - cx as i64;
                    let dy = y as i64 - cy as i64;
                    let dz = z as i64 - cz as i64;
                    if dx * dx + dy * dy + dz * dz <= r2 {
                        let i = self.fuel.idx(x, y, z);
                        self.fuel.u[i] += amount;
                    }
                }
            }
        }
    }

    /// 在网格 `(cx,cy,cz)` 处直接设置燃料(替换)。
    pub fn set_fuel_at(&mut self, cx: usize, cy: usize, cz: usize, amount: T) {
        let i = self.fuel.idx(cx, cy, cz);
        self.fuel.u[i] = amount;
    }

    /// 总烟量(守恒/诊断用)。
    pub fn total_smoke(&self) -> T {
        self.field.sum()
    }

    /// 总燃料量(应随燃烧单调下降)。
    pub fn total_fuel(&self) -> T {
        self.fuel.sum()
    }

    /// 三线性采样某世界坐标处的烟浓度。
    pub fn conc_at_world(&self, p: Vec3<T>) -> T {
        let (x, y, z, tx, ty, tz) = world_to_cell(self, p);
        let nx = self.field.nx;
        let ny = self.field.ny;
        let nz = self.field.nz;
        let c000 = self.field.sample(x, y, z);
        let c100 = self.field.sample((x + 1).min(nx - 1), y, z);
        let c010 = self.field.sample(x, (y + 1).min(ny - 1), z);
        let c110 = self.field.sample((x + 1).min(nx - 1), (y + 1).min(ny - 1), z);
        let c001 = self.field.sample(x, y, (z + 1).min(nz - 1));
        let c101 = self.field.sample((x + 1).min(nx - 1), y, (z + 1).min(nz - 1));
        let c011 = self.field.sample(x, (y + 1).min(ny - 1), (z + 1).min(nz - 1));
        let c111 = self
            .field
            .sample((x + 1).min(nx - 1), (y + 1).min(ny - 1), (z + 1).min(nz - 1));
        let fx = T::from_f64(tx).unwrap_or(T::zero());
        let fy = T::from_f64(ty).unwrap_or(T::zero());
        let fz = T::from_f64(tz).unwrap_or(T::zero());
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

    /// 燃烧一步:对每个有燃料且 `temp_fn(p) ≥ ignition_temp` 的格,按 `burn_rate·dt`
    /// 消耗燃料、生成烟、累积放热到 `heat_scratch`。`temp_fn` 由调用方提供(读热场或
    /// 单机温度采样器)。`dt` 为时间步长。
    fn burn(&mut self, temp_fn: &dyn Fn(Vec3<T>) -> T, dt: T) {
        let n = self.field.u.len();
        if self.heat_scratch.len() != n {
            self.heat_scratch = vec![T::zero(); n];
        }
        let (nx, ny, nz) = (self.field.nx, self.field.ny, self.field.nz);
        let dx = self.field.dx;
        let o = self.field.origin;
        let burn = self.fuel_burn_rate;
        let ig = self.ignition_temp;
        let hr = self.heat_release;
        let fs = self.flame_spawn;
        let zero = T::zero();
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let idx = self.field.idx(i, j, k);
                    if self.fuel.u[idx] <= zero {
                        continue;
                    }
                    let wp = Vec3::new(
                        o.x + dx * T::from_usize(i).unwrap_or(zero),
                        o.y + dx * T::from_usize(j).unwrap_or(zero),
                        o.z + dx * T::from_usize(k).unwrap_or(zero),
                    );
                    let temp = temp_fn(wp);
                    if temp >= ig {
                        let consumed = (burn * dt * self.fuel.u[idx]).min(self.fuel.u[idx]);
                        self.fuel.u[idx] -= consumed;
                        self.field.u[idx] += fs * consumed;
                        self.heat_scratch[idx] += hr * consumed;
                    }
                }
            }
        }
    }

    /// 把 `heat_scratch` 累积的放热注入 `heat`。放热按"火焰核"分配到燃烧格及其 6 个
    /// 面邻格(直接累加到温度场,不经 `src·dt` 弱源项),保证邻格被加热到点燃阈值,
    /// 形成自持传播的火焰前缘。
    fn inject_heat_into(&mut self, heat: &mut HeatField<T>) {
        let (nx, ny, nz) = (self.field.nx, self.field.ny, self.field.nz);
        let center = T::from_f64(0.6).unwrap_or(T::zero());
        let neigh = T::from_f64(0.4).unwrap_or(T::zero()) / T::from_f64(6.0).unwrap_or(T::one());
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let idx = i + nx * (j + ny * k);
                    let amt = self.heat_scratch[idx];
                    if amt > T::zero() {
                        // 燃烧格自身。
                        heat.add_temperature_at(i, j, k, center * amt);
                        // 6 个面邻格(越界钳止)。
                        for (di, dj, dk) in [
                            (1, 0, 0),
                            (-1, 0, 0),
                            (0, 1, 0),
                            (0, -1, 0),
                            (0, 0, 1),
                            (0, 0, -1),
                        ] {
                            let ni = i as i64 + di;
                            let nj = j as i64 + dj;
                            let nk = k as i64 + dk;
                            if ni >= 0
                                && nj >= 0
                                && nk >= 0
                                && ni < nx as i64
                                && nj < ny as i64
                                && nk < nz as i64
                            {
                                heat.add_temperature_at(
                                    ni as usize,
                                    nj as usize,
                                    nk as usize,
                                    neigh * amt,
                                );
                            }
                        }
                    }
                }
            }
        }
        for v in self.heat_scratch.iter_mut() {
            *v = T::zero();
        }
    }
}

impl<T: RealField + Copy + std::fmt::Debug> std::fmt::Debug for SmokeField<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SmokeField")
            .field("field", &self.field)
            .field("diffusion", &self.diffusion)
            .field("buoyancy", &self.buoyancy)
            .field("fuel", &self.fuel)
            .field("fuel_burn_rate", &self.fuel_burn_rate)
            .field("ignition_temp", &self.ignition_temp)
            .field("heat_release", &self.heat_release)
            .field("flame_spawn", &self.flame_spawn)
            .field("vel_sampler", &self.vel_sampler.is_some())
            .field("heat_sampler", &self.heat_sampler.is_some())
            .field("last_r", &self.last_r)
            .finish()
    }
}

impl<T: RealField + Copy> Clone for SmokeField<T> {
    fn clone(&self) -> Self {
        SmokeField {
            field: self.field.clone(),
            diffusion: self.diffusion,
            buoyancy: self.buoyancy,
            fuel: self.fuel.clone(),
            fuel_burn_rate: self.fuel_burn_rate,
            ignition_temp: self.ignition_temp,
            heat_release: self.heat_release,
            flame_spawn: self.flame_spawn,
            vel_sampler: None,
            heat_sampler: None,
            heat_scratch: self.heat_scratch.clone(),
            last_r: self.last_r,
        }
    }
}

use crate::GridGeometry;

impl<T: RealField + Copy + ToPrimitive> GridGeometry<T> for SmokeField<T> {
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

impl<T: RealField + Copy + ToPrimitive + DeserializeOwned + Serialize + nalgebra::Scalar> Subsystem<T>
    for SmokeField<T>
{
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn step(&mut self, dt: &T) {
        // 取出速度采样器本地持有(闭包只借用它 + 浓度只读快照,不借用 self.field),
        // 步进结束后再放回 self.vel_sampler。
        let sampler = self.vel_sampler.take();
        let sampler_ref = &sampler;

        // ---- 烟平流 + 扩散(含浓度依赖浮力) ----
        if sampler.is_some() {
            // 把浓度快照做几次盒式模糊,得到"浮力上升源场"。
            let snap_u = self.field.u.clone();
            let (nx, ny, nz) = (self.field.nx, self.field.ny, self.field.nz);
            let dx = self.field.dx;
            let o = self.field.origin;
            let buoy = self.buoyancy;
            let up_field = blur3(&snap_u, nx, ny, nz, 3);
            let eff = move |p: Vec3<T>| -> Vec3<T> {
                let fluid = match sampler_ref {
                    Some(f) => f(p),
                    None => Vec3::zeros(),
                };
                let mut v = fluid;
                if buoy > T::zero() {
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
            // 燃料随同一流场平流 + 扩散(无浮力)。
            let fuel_eff = move |p: Vec3<T>| -> Vec3<T> {
                match sampler_ref {
                    Some(f) => f(p),
                    None => Vec3::zeros(),
                }
            };
            self.fuel.step_advection_diffusion(&fuel_eff, self.diffusion, *dt);
            self.vel_sampler = sampler;
        } else {
            self.last_r = self.field.step_diffusion(self.diffusion, *dt);
            self.fuel.step_diffusion(self.diffusion, *dt);
        }

        // ---- 单机燃烧:若有 temperature 采样器,直接就地燃烧(无 World/HeatField) ----
        if self.heat_sampler.is_some() {
            let temp_fn = self.heat_sampler.take().unwrap();
            self.burn(&*temp_fn, *dt);
            self.heat_sampler = Some(temp_fn);
            // 无 HeatField 可注入,放热累积丢弃(单机烟雾测试不验证热闭环)。
            for v in self.heat_scratch.iter_mut() {
                *v = T::zero();
            }
        }
    }

    fn couple(&mut self, world: &mut phy_core::world::World<T>, dt: &T) {
        // 在 World 中:找到共注册的 HeatField,读取其温度就地燃烧,
        // 再把放热注入热场。放热抬升邻格温度 → 点燃更多燃料 → 自持传播的火焰前缘。
        // 先定位热场下标。
        let mut target: Option<usize> = None;
        for j in 0..world.subsystem_count() {
            let is_heat = match world.get_mut(j) {
                Some(hs) => hs.as_any_mut().downcast_mut::<HeatField<T>>().is_some(),
                None => false,
            };
            if is_heat {
                target = Some(j);
                break;
            }
        }
        let j = match target {
            Some(j) => j,
            None => return,
        };
        // 取 HeatField 只读裸指针,供燃烧阶段读取温度(期间热场不被写入,self 独占修改)。
        let heat_ptr: *const HeatField<T> = {
            let hs = world.get_mut(j).unwrap();
            hs.as_any_mut().downcast_mut::<HeatField<T>>().unwrap() as *const HeatField<T>
        };
        let temp_fn = move |p: Vec3<T>| -> T { unsafe { (*heat_ptr).temp_at_world(p) } };
        self.burn(&temp_fn, *dt);
        // 重新获取可变热场引用,把累积放热注入。
        let hs = world.get_mut(j).unwrap();
        let heat = hs.as_any_mut().downcast_mut::<HeatField<T>>().unwrap();
        self.inject_heat_into(heat);
    }

    fn name(&self) -> &'static str {
        "smoke"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phy_core::world::World;

    #[test]
    fn smoke_advects_with_fluid_velocity() {
        let mut s = SmokeField::<f64>::new(16, 16, 16, 0.5, 0.0, 1.0);
        s.inject_blob(2, 8, 8, 1, 1.0);
        let before = s.conc_at_world(Vec3::new(0.5 * 2.0, 0.5 * 8.0, 0.5 * 8.0));
        // 沿 +x 匀速流,平流应把烟峰整体平移。
        s.set_vel_sampler(Box::new(|_p: Vec3<f64>| Vec3::new(0.5, 0.0, 0.0)));
        s.step(&1.0);
        let after = s.conc_at_world(Vec3::new(0.5 * 2.0 + 0.5, 0.5 * 8.0, 0.5 * 8.0));
        assert!(after > 0.8 * before, "烟应随流平流平移, before={} after={}", before, after);
    }

    #[test]
    fn smoke_rises_with_buoyancy() {
        let mut s = SmokeField::<f64>::new(16, 16, 16, 0.5, 0.0, 1.0);
        s.inject_blob(8, 2, 8, 1, 1.0); // 烟团在较低处
        let cy0 = {
            let mut w = 0.0;
            let mut n = 0.0;
            for k in 0..16 {
                for j in 0..16 {
                    for i in 0..16 {
                        let c = s.field.u[s.field.idx(i, j, k)];
                        w += c * (j as f64);
                        n += c;
                    }
                }
            }
            w / n
        };
        s.set_vel_sampler(Box::new(|_p: Vec3<f64>| Vec3::zeros()));
        for _ in 0..5 {
            s.step(&0.5);
        }
        let cy1 = {
            let mut w = 0.0;
            let mut n = 0.0;
            for k in 0..16 {
                for j in 0..16 {
                    for i in 0..16 {
                        let c = s.field.u[s.field.idx(i, j, k)];
                        w += c * (j as f64);
                        n += c;
                    }
                }
            }
            w / n
        };
        assert!(cy1 > cy0, "浮力应使烟团质心上升, cy0={} cy1={}", cy0, cy1);
    }

    #[test]
    fn smoke_no_buoyancy_stays_put() {
        let mut s = SmokeField::<f64>::new(16, 16, 16, 0.5, 0.0, 0.0);
        s.inject_blob(8, 8, 8, 1, 1.0);
        let cy0 = {
            let mut w = 0.0;
            let mut n = 0.0;
            for k in 0..16 {
                for j in 0..16 {
                    for i in 0..16 {
                        let c = s.field.u[s.field.idx(i, j, k)];
                        w += c * (j as f64);
                        n += c;
                    }
                }
            }
            w / n
        };
        s.set_vel_sampler(Box::new(|_p: Vec3<f64>| Vec3::zeros()));
        for _ in 0..5 {
            s.step(&0.5);
        }
        let cy1 = {
            let mut w = 0.0;
            let mut n = 0.0;
            for k in 0..16 {
                for j in 0..16 {
                    for i in 0..16 {
                        let c = s.field.u[s.field.idx(i, j, k)];
                        w += c * (j as f64);
                        n += c;
                    }
                }
            }
            w / n
        };
        assert!((cy1 - cy0).abs() < 1e-9, "无浮力时质心应不动, cy0={} cy1={}", cy0, cy1);
    }

    // ---- S5b:燃烧 / 火焰前缘 ----
    #[test]
    fn combustion_consumes_fuel_and_makes_smoke() {
        // 单机场景:用温度采样器返回"中心高温",验证燃料被消耗、烟被生成。
        let mut s = SmokeField::<f64>::new(16, 16, 16, 0.5, 0.0, 0.0);
        s.inject_fuel(8, 8, 8, 2, 1.0);
        s.set_heat_sampler(Box::new(|p: Vec3<f64>| {
            let c = Vec3::new(0.5 * 8.0, 0.5 * 8.0, 0.5 * 8.0);
            if (p - c).norm() < 3.0 * 0.5 {
                500.0
            } else {
                0.0
            }
        }));
        let fuel0 = s.total_fuel();
        let smoke0 = s.total_smoke();
        for _ in 0..10 {
            s.step(&0.1);
        }
        let fuel1 = s.total_fuel();
        let smoke1 = s.total_smoke();
        assert!(fuel1 < fuel0, "燃料应被消耗, fuel0={} fuel1={}", fuel0, fuel1);
        assert!(
            smoke1 > smoke0,
            "燃烧应生成烟, smoke0={} smoke1={}",
            smoke0,
            smoke1
        );
    }

    #[test]
    fn flame_front_propagates_via_heat_coupling() {
        // 端到端:World 内 HeatField + SmokeField(一长条燃料),中心点燃。
        // 燃烧放热经 couple 注入热场,热场扩散抬升邻格温度 → 点燃更多燃料 → 前缘传播。
        let mut world: World<f64> = World::new();
        let heat_sf = ScalarField::<f64>::new(32, 8, 8, 0.5, 0.0, crate::Bc::Neumann);
        let heat = HeatField::<f64>::new(heat_sf, 0.3);
        let mut smoke = SmokeField::<f64>::new(32, 8, 8, 0.5, 0.0, 0.0);
        // 在 x∈[4,28] 注入燃料长条。
        for x in 4..28 {
            smoke.inject_fuel(x, 4, 4, 1, 1.0);
        }
        // 数值演示参数:放热较大、点燃阈值适中、燃速较快,以形成清晰可观测的火焰前缘。
        smoke.heat_release = 3000.0;
        smoke.ignition_temp = 250.0;
        smoke.fuel_burn_rate = 2.0;
        // 中心点燃:直接把热场中心温度设为高温(首步 step 仅做扩散,不衰减该初值)。
        let mut heat = heat;
        {
            let i = heat.field.idx(16, 4, 4);
            heat.field.u[i] = 2000.0;
        }
        world.add_subsystem(Box::new(heat));
        world.add_subsystem(Box::new(smoke));

        let fuel0 = {
            let sm = world
                .get(1)
                .unwrap()
                .as_any()
                .downcast_ref::<SmokeField<f64>>()
                .unwrap();
            sm.total_fuel()
        };
        for _ in 0..80 {
            world.step(0.05);
        }
        let (fuel1, smoke1) = {
            let sm = world
                .get(1)
                .unwrap()
                .as_any()
                .downcast_ref::<SmokeField<f64>>()
                .unwrap();
            (sm.total_fuel(), sm.total_smoke())
        };
        assert!(
            fuel1 < fuel0 * 0.5,
            "火焰前缘应显著推进消耗燃料, fuel0={} fuel1={}",
            fuel0,
            fuel1
        );
        assert!(smoke1 > 0.0, "燃烧应生成烟, smoke1={}", smoke1);
    }
}
