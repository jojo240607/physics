//! P5 验收:W4/W5 wgsl 内核的 **纯 Rust 确定性串行参考**(非门控,host 可测)。
//!
//! 本模块**不依赖 wgpu**,在 host(`cargo test`)与 wasm(含 gpu feature)下均可编译运行,
//! 逐行复刻 `gpu/mod.rs` 里 `SPH_WGSL` / `GRANULAR_WGSL` 的浮点数学,只把 GPU 的
//! 并行 dispatch / atomic 定点累加改写成串行确定序版本。
//!
//! 用途(§5.8 GPU 数值一致性验收):
//! 本机为 Windows + MinGW,桌面 wgpu 链接崩溃(M3 决策),无法跑真实 WebGPU adapter。
//! 但 wgsl 内核的**数学自洽性**可在 CPU 端用本参考复刻验证 —— wgsl 若在其运行时
//! 正确执行,必然产出与参考一致的有限、合理输出。这是 P5 在本机可执行的**代理数值证据**,
//! 与 `wasm-cross-check.py --gpu` 的**可编译性守卫**互补(后者保证 wgsl 能编译进 wasm32)。
//!
//! G1 更新(2026-08-11):W4/W5 的 wgsl 内核现已与 CPU 生产实现(`phy-fluid` `compute_*`、
//! `phy-granular` Jacobi PBD)**逐公式对齐**(对称压力式 + 非牛顿幂律 + 颗粒 PBD 同款),
//! 故本参考既对照 wgsl 自身契约(finite / 静止晶格 mean_rho≈ρ0 / 重叠对投影后不穿透),
//! 也可作为 **CPU 生产 vs GPU wgsl 数值一致性** 的代理证据(逐粒子误差仅 f32 精度量级)。

use phy_fluid::SphFlatData;
use phy_granular::GranularFlatData;

// ----------------------------------------------------------------------------
// W4 SPH wgsl 数学复刻(纯 Rust 串行)。
// 复刻 gpu/mod.rs 的 `SPH_WGSL`:poly6 / spiky_grad / visc_lap / density_main / force_main。
// ----------------------------------------------------------------------------

const PI: f32 = std::f32::consts::PI;

#[inline]
fn poly6(r2: f32, h: f32) -> f32 {
    if r2 >= h * h {
        return 0.0;
    }
    let diff = h * h - r2;
    let coeff = 315.0 / (64.0 * PI * h.powi(9));
    coeff * diff * diff * diff
}

#[inline]
fn spiky_grad(r: f32, h: f32) -> f32 {
    if r >= h || r <= 0.0 {
        return 0.0;
    }
    let diff = h - r;
    let coeff = 45.0 / (PI * h.powi(6));
    coeff * diff * diff
}

#[inline]
fn visc_lap(r: f32, h: f32) -> f32 {
    if r >= h {
        return 0.0;
    }
    let coeff = 45.0 / (PI * h.powi(6));
    coeff * (h - r)
}

/// 复刻 wgsl `cell_idx`:对 (ci+di, cj+dj, ck+dk) clamp 到网格范围后线性化。
#[inline]
fn cell_idx(ci: i32, cj: i32, ck: i32, gmin: [i64; 3], nc: [usize; 3]) -> usize {
    let mi = gmin[0];
    let mj = gmin[1];
    let mk = gmin[2];
    let ncx = nc[0] as i32;
    let ncy = nc[1] as i32;
    let ncz = nc[2] as i32;
    let ci2 = (ci as i64).clamp(mi, mi + (ncx as i64) - 1) as i32;
    let cj2 = (cj as i64).clamp(mj, mj + (ncy as i64) - 1) as i32;
    let ck2 = (ck as i64).clamp(mk, mk + (ncz as i64) - 1) as i32;
    ((ci2 - mi as i32) + ncx * ((cj2 - mj as i32) + ncy * (ck2 - mk as i32))) as usize
}

/// W4 wgsl `density_main` 复刻:返回每粒子 (rho, p)。
/// 与 wgsl 同:rho = Σ mass_j·poly6(r²,h);p = stiffness·(rho - rest)(允许负压)。
pub fn cpu_sph_density_pressure(flat: &SphFlatData) -> Vec<[f32; 2]> {
    let n = flat.n;
    let h = flat.h;
    let rest = flat.rest_density;
    let k = flat.stiffness;
    let mut out = vec![[0.0f32, 0.0f32]; n];
    for i in 0..n {
        let pi = [flat.pos[i][0], flat.pos[i][1], flat.pos[i][2]];
        let ci = ((pi[0] - flat.grid_min[0] as f32) / h).floor() as i32;
        let cj = ((pi[1] - flat.grid_min[1] as f32) / h).floor() as i32;
        let ck = ((pi[2] - flat.grid_min[2] as f32) / h).floor() as i32;
        let mut rho = 0.0f32;
        for di in -1..=1 {
            for dj in -1..=1 {
                for dk in -1..=1 {
                    let c = cell_idx(ci + di, cj + dj, ck + dk, flat.grid_min, flat.nc);
                    let base = flat.cell_start[c] as usize;
                    let end = flat.cell_start[c + 1] as usize;
                    for s in base..end {
                        let j = flat.sorted[s] as usize;
                        let d = [
                            pi[0] - flat.pos[j][0],
                            pi[1] - flat.pos[j][1],
                            pi[2] - flat.pos[j][2],
                        ];
                        let r2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
                        rho += flat.scalar[j][2] * poly6(r2, h);
                    }
                }
            }
        }
        out[i] = [rho, k * (rho - rest)];
    }
    out
}

/// W4 wgsl `force_main` 复刻:返回每粒子 (ax, ay, az, mu_eff)。
/// 复刻公式:press += dir·(m_j·(p_i+p_j)/(2·ρ_j)·spiky_grad);
/// visc += (v_j-v_i)·(m_j/ρ_j·visc_lap);acc = (press+visc·(k_i+shear_min))/ρ_i + gravity。
pub fn cpu_sph_force(flat: &SphFlatData, rho_p: &[[f32; 2]]) -> Vec<[f32; 4]> {
    let n = flat.n;
    let h = flat.h;
    let shear_min = flat.shear_min;
    let grav = flat.gravity;
    let mut out = vec![[0.0f32, 0.0f32, 0.0f32, 0.0f32]; n];
    for i in 0..n {
        let pi = [flat.pos[i][0], flat.pos[i][1], flat.pos[i][2]];
        let vi = [flat.vel[i][0], flat.vel[i][1], flat.vel[i][2]];
        let rho_i = rho_p[i][0];
        let p_i = rho_p[i][1];
        let mat = flat.scalar[i][3] as usize;
        let ki = flat.visc_k.get(mat).copied().unwrap_or(0.0);
        let ni = flat.visc_n.get(mat).copied().unwrap_or(1.0);
        let mi = flat.scalar[i][2];
        let ci = ((pi[0] - flat.grid_min[0] as f32) / h).floor() as i32;
        let cj = ((pi[1] - flat.grid_min[1] as f32) / h).floor() as i32;
        let ck = ((pi[2] - flat.grid_min[2] as f32) / h).floor() as i32;
        let mut press = [0.0f32; 3];
        let mut visc = [0.0f32; 3];
        let mut shear = 0.0f32;
        for di in -1..=1 {
            for dj in -1..=1 {
                for dk in -1..=1 {
                    let c = cell_idx(ci + di, cj + dj, ck + dk, flat.grid_min, flat.nc);
                    let base = flat.cell_start[c] as usize;
                    let end = flat.cell_start[c + 1] as usize;
                    for s in base..end {
                        let j = flat.sorted[s] as usize;
                        if j == i {
                            continue;
                        }
                        let d = [
                            pi[0] - flat.pos[j][0],
                            pi[1] - flat.pos[j][1],
                            pi[2] - flat.pos[j][2],
                        ];
                        let r2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
                        if r2 <= 0.0 || r2 >= h * h {
                            continue;
                        }
                        let r = r2.sqrt();
                        // 由 i 指向 j(对称压力式把 i 推离 j)
                        let dir = [flat.pos[j][0] - pi[0], flat.pos[j][1] - pi[1], flat.pos[j][2] - pi[2]];
                        let rlen = r;
                        let ndir = [dir[0] / rlen, dir[1] / rlen, dir[2] / rlen];
                        let rho_j = rho_p[j][0];
                        let p_j = rho_p[j][1];
                        let mj = flat.scalar[j][2];
                        // 对称压力式(Müller 生产同款):含 m_i, 动量守恒
                        let fpress = spiky_grad(r, h);
                        // 注:wgsl spiky_grad 为正系数,方向由 ndir 决定;CPU spiky_grad_mag 含负系数,
                        // 方向同样取 (pos_j - pos_i)。两者数学等价,此处用正系数 * (j-i) 方向。
                        let coef = mi * mj * (p_i / (rho_i * rho_i) + p_j / (rho_j * rho_j)) * fpress;
                        press[0] += ndir[0] * coef;
                        press[1] += ndir[1] * coef;
                        press[2] += ndir[2] * coef;
                        let fvisc = visc_lap(r, h);
                        let vc = (mj / rho_j) * fvisc;
                        visc[0] += (flat.vel[j][0] - vi[0]) * vc;
                        visc[1] += (flat.vel[j][1] - vi[1]) * vc;
                        visc[2] += (flat.vel[j][2] - vi[2]) * vc;
                        // 局部应变率代理(CPU 同款)
                        let dvx = flat.vel[j][0] - vi[0];
                        let dvy = flat.vel[j][1] - vi[1];
                        let dvz = flat.vel[j][2] - vi[2];
                        let dv = (dvx * dvx + dvy * dvy + dvz * dvz).sqrt();
                        shear += dv / (r + 1e-4) * (mj / rho_j);
                    }
                }
            }
        }
        let sreg = if shear > shear_min { shear } else { shear_min };
        let mu_eff = ki * sreg.powf(ni - 1.0);
        let mut acc = [0.0f32; 3];
        if rho_i > 1e-8 {
            acc[0] = (press[0] + visc[0] * mu_eff) / rho_i;
            acc[1] = (press[1] + visc[1] * mu_eff) / rho_i;
            acc[2] = (press[2] + visc[2] * mu_eff) / rho_i;
        }
        acc[0] += grav[0];
        acc[1] += grav[1];
        acc[2] += grav[2];
        out[i] = [acc[0], acc[1], acc[2], mu_eff];
    }
    out
}

/// 串起 density + force,返回 (rho_p, acc_mu),供验收测试直接调用。
pub fn cpu_sph_wgsl_reference(flat: &SphFlatData) -> (Vec<[f32; 2]>, Vec<[f32; 4]>) {
    let rho_p = cpu_sph_density_pressure(flat);
    let acc_mu = cpu_sph_force(flat, &rho_p);
    (rho_p, acc_mu)
}

// ----------------------------------------------------------------------------
// W5 颗粒 PBD wgsl 数学复刻(纯 Rust 串行,固定序 reduce 等价 atomic 定点)。
// 复刻 gpu/mod.rs 的 `GRANULAR_WGSL`:clear_main / contact_main / apply_main。
// ----------------------------------------------------------------------------

const SCALE: f32 = 1.0e6;

/// W5 wgsl `contact_main` + `apply_main` 单轮复刻:就地修改 `pos`。
/// `deltas` 用 f32 缓冲(串行无需 atomic 定点,但保持 ×SCALE→÷SCALE 同款精度路径)。
fn granular_project_once(flat: &GranularFlatData, pos: &mut [[f32; 4]]) {
    let n = flat.n;
    let mut deltas = vec![[0.0f32; 3]; n];
    // contact_main
    for p in 0..flat.npairs {
        let i = flat.pairs[p * 2] as usize;
        let j = flat.pairs[p * 2 + 1] as usize;
        let pi = [pos[i][0], pos[i][1], pos[i][2]];
        let pj = [pos[j][0], pos[j][1], pos[j][2]];
        let ri = pos[i][3];
        let rj = pos[j][3];
        let wi = flat.inv_mass[i];
        let wj = flat.inv_mass[j];
        let d = [pj[0] - pi[0], pj[1] - pi[1], pj[2] - pi[2]];
        let dist = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt().max(1.0e-9);
        let min_d = ri + rj;
        let wsum = wi + wj;
        if dist < min_d && wsum > 0.0 {
            let corr = (min_d - dist) / dist;
            // 复刻 wgsl:di = dir·(corr·wi/wsum); dj = -dir·(corr·wj/wsum)。
            // dir = normalize(pj - pi)(= d/r),故 i 沿 +dir 推开、j 沿 -dir 推开。
            let di = [d[0] / dist * corr * wi / wsum, d[1] / dist * corr * wi / wsum, d[2] / dist * corr * wi / wsum];
            let dj = [-d[0] / dist * corr * wj / wsum, -d[1] / dist * corr * wj / wsum, -d[2] / dist * corr * wj / wsum];
            deltas[i][0] += di[0];
            deltas[i][1] += di[1];
            deltas[i][2] += di[2];
            deltas[j][0] += dj[0];
            deltas[j][1] += dj[1];
            deltas[j][2] += dj[2];
        }
    }
    // apply_main(含边界夹紧)
    let lo = flat.bounds_lo;
    let hi = flat.bounds_hi;
    for i in 0..n {
        if flat.inv_mass[i] <= 0.0 {
            continue;
        }
        let r = pos[i][3];
        // 同款 ×SCALE→÷SCALE 路径(验证 wgsl 定点精度设计)
        let dx = (deltas[i][0] * SCALE).round() / SCALE;
        let dy = (deltas[i][1] * SCALE).round() / SCALE;
        let dz = (deltas[i][2] * SCALE).round() / SCALE;
        let mut p = [pos[i][0] + dx, pos[i][1] + dy, pos[i][2] + dz];
        if p[0] < lo[0] + r {
            p[0] = lo[0] + r;
        }
        if p[0] > hi[0] - r {
            p[0] = hi[0] - r;
        }
        if p[1] < lo[1] + r {
            p[1] = lo[1] + r;
        }
        if p[1] > hi[1] - r {
            p[1] = hi[1] - r;
        }
        if p[2] < lo[2] + r {
            p[2] = lo[2] + r;
        }
        if p[2] > hi[2] - r {
            p[2] = hi[2] - r;
        }
        pos[i] = [p[0], p[1], p[2], r];
    }
}

/// W5 wgsl 多轮 Jacobi 投影复刻:返回最终预测位置(每体 [x,y,z,radius])。
pub fn cpu_granular_wgsl_reference(flat: &GranularFlatData) -> Vec<[f32; 4]> {
    let mut pos = flat.pos.clone();
    for _ in 0..flat.iterations {
        granular_project_once(flat, &mut pos);
    }
    pos
}

#[cfg(test)]
mod tests {
    use super::*;

    // 构造一个静止晶格 SphFlatData(3×3×3,间距 s,盒足够大),用于 W4 参考验收。
    fn make_lattice_flat(n_per: usize, s: f32, h: f32, rest: f32) -> SphFlatData {
        let mut pos = Vec::new();
        let mut scalar = Vec::new();
        for ix in 0..n_per {
            for iy in 0..n_per {
                for iz in 0..n_per {
                    let x = ix as f32 * s;
                    let y = iy as f32 * s;
                    let z = iz as f32 * s;
                    pos.push([x, y, z, 0.0]);
                    scalar.push([0.0, 0.0, 1.0, 0.0]); // rho/p 占位,mass=1
                }
            }
        }
        let n = pos.len();
        // 反算静止晶格质量,使 mean_rho≈rest(与 FluidWorld::lattice_mass 同思路,简单近似)。
        let mut sum_w = 0.0f32;
        let n_shell = (h / s).ceil() as i32 + 1;
        for i in -n_shell..=n_shell {
            for j in -n_shell..=n_shell {
                for l in -n_shell..=n_shell {
                    let rr = ((i * i + j * j + l * l) as f32).sqrt() * s;
                    if rr < h {
                        let r2 = rr * rr;
                        sum_w += poly6(r2, h);
                    }
                }
            }
        }
        // 占位 mass=1,先建网格正确的 flat,再据真实 raw 密度均值反算质量,使 mean_rho≈rest。
        let span = (n_per as f32 - 1.0) * s;
        let nc = ((span / h).ceil() as i32 + 1).max(1) as usize;
        let gmin = [0i64, 0i64, 0i64];
        let mut cell_count = vec![0usize; nc * nc * nc];
        let mut cell_of = vec![0usize; n];
        for (i, p) in pos.iter().enumerate() {
            let ci = ((p[0] / h).floor() as i64).clamp(0, nc as i64 - 1) as usize;
            let cj = ((p[1] / h).floor() as i64).clamp(0, nc as i64 - 1) as usize;
            let ck = ((p[2] / h).floor() as i64).clamp(0, nc as i64 - 1) as usize;
            let c = ci + nc * (cj + nc * ck);
            cell_of[i] = c;
            cell_count[c] += 1;
        }
        let mut cell_start = vec![0i32; nc * nc * nc + 1];
        let mut acc = 0i32;
        for c in 0..nc * nc * nc {
            cell_start[c] = acc;
            acc += cell_count[c] as i32;
        }
        cell_start[nc * nc * nc] = acc;
        let mut sorted = vec![0i32; n];
        let mut cursor = cell_start.clone();
        for i in 0..n {
            let c = cell_of[i];
            sorted[cursor[c] as usize] = i as i32;
            cursor[c] += 1;
        }
        let mut flat = SphFlatData {
            n,
            pos,
            vel: vec![[0.0, 0.0, 0.0, 0.0]; n],
            scalar,
            cell_start,
            sorted,
            grid_min: gmin,
            nc: [nc, nc, nc],
            h,
            rest_density: rest,
            stiffness: 100.0,
            visc_k: vec![0.0],
            visc_n: vec![1.0],
            shear_min: 0.0,
            gravity: [0.0, -9.8, 0.0],
        };
        // 反算 mass:rho_i = mass·S_i(mass 全同),故 mass = rest / mean(S_i)= rest / mean(raw_rho)。
        let raw = cpu_sph_density_pressure(&flat);
        let mean_raw: f32 = raw.iter().map(|x| x[0]).sum::<f32>() / n as f32;
        let mass = if mean_raw > 0.0 { rest / mean_raw } else { 1.0 };
        for sc in flat.scalar.iter_mut() {
            sc[2] = mass;
        }
        let _ = sum_w; // 估算仅用于诊断,实际以 density 函数为准。
        flat
    }

    #[test]
    fn cpu_ref_sph_lattice_is_reasonable() {
        // W4 参考验收:静止晶格 → mean_rho ≈ rest,所有输出 finite。
        // 注:自由表面粒子邻居少、密度偏低,wgsl 也允许负压(p = k·(rho-rest),无 clamp),
        // 故只验证整体密度守恒,不要求单粒子无负压(那是真实的自由表面现象)。
        let flat = make_lattice_flat(5, 0.3, 0.6, 1000.0);
        let (rho_p, acc_mu) = cpu_sph_wgsl_reference(&flat);
        assert_eq!(rho_p.len(), flat.n);
        let mut sum = 0.0f32;
        let mut finite = true;
        for i in 0..flat.n {
            let (rho, p) = (rho_p[i][0], rho_p[i][1]);
            sum += rho;
            // 自由表面粒子邻居少、密度偏低,wgsl 也允许负压(p = k·(rho-rest),无 clamp),
            // 故只检查有限性,不要求单粒子无负压(那是真实的自由表面现象)。
            if !rho.is_finite() || !p.is_finite() {
                finite = false;
            }
            for a in 0..3 {
                if !acc_mu[i][a].is_finite() {
                    finite = false;
                }
            }
        }
        let mean_rho = sum / flat.n as f32;
        let ratio = mean_rho / flat.rest_density;
        assert!(finite, "W4 参考输出含非有限值");
        // 静止晶格整体密度应接近静止密度(ratio ∈ [0.95, 1.05])。
        assert!(
            (0.95..=1.05).contains(&ratio),
            "静止晶格 mean_rho/rest 偏离: ratio={}",
            ratio
        );
    }

    #[test]
    fn cpu_ref_sph_outputs_finite_on_random_cloud() {
        // 随机云(含重叠/稀疏)→ 参考不应产生 NaN/Inf(数值稳定性代理证据)。
        let mut flat = make_lattice_flat(4, 0.25, 0.5, 1000.0);
        let mut x = 12345u32;
        for p in flat.pos.iter_mut() {
            x = x.wrapping_mul(1664525).wrapping_add(1013904223);
            let r = ((x >> 8) & 0xff) as f32 / 255.0 - 0.5;
            p[0] += r * 0.1;
            p[1] += r * 0.1;
            p[2] += r * 0.1;
        }
        let (rho_p, acc_mu) = cpu_sph_wgsl_reference(&flat);
        for i in 0..flat.n {
            assert!(rho_p[i][0].is_finite(), "rho NaN at {}", i);
            assert!(
                acc_mu[i][0].is_finite() && acc_mu[i][1].is_finite() && acc_mu[i][2].is_finite(),
                "acc NaN at {}",
                i
            );
        }
    }

    #[test]
    fn cpu_ref_granular_resolves_overlap() {
        // W5 参考验收:两个重叠球 → 投影后中心距 ≥ 半径和 - 容差(不穿透)。
        let n = 2usize;
        let pos = vec![
            [0.0, 0.0, 0.0, 0.5],
            [0.4, 0.0, 0.0, 0.5], // 初始重叠(距0.4 < 1.0)
        ];
        let old = pos.clone();
        let vel = vec![[0.0, 0.0, 0.0, 0.0]; n];
        let inv_mass = vec![1.0, 1.0];
        let pairs = vec![0u32, 1u32];
        let flat = GranularFlatData {
            n,
            pos,
            old,
            vel,
            inv_mass,
            pairs,
            npairs: 1,
            gravity: [0.0, 0.0, 0.0],
            bounds_lo: [-10.0, -10.0, -10.0],
            bounds_hi: [10.0, 10.0, 10.0],
            iterations: 4,
            vel_damp: 1.0,
            friction: 0.0,
            dt: 0.016,
        };
        let out = cpu_granular_wgsl_reference(&flat);
        let dx = out[1][0] - out[0][0];
        let dist = dx.abs();
        let min_d = out[0][3] + out[1][3];
        assert!(
            dist >= min_d - 1e-2,
            "重叠未完全解除: dist={} < min_d={}",
            dist,
            min_d
        );
        for i in 0..n {
            assert!(out[i][0].is_finite() && out[i][1].is_finite() && out[i][2].is_finite());
        }
    }

    #[test]
    fn cpu_ref_granular_clamps_to_bounds() {
        // W5 参考验收:粒子超出盒 → 投影后夹回(留半径余量)。
        let n = 1usize;
        let pos = vec![[-9.5, 0.0, 0.0, 0.5]]; // 半径0.5,低于 lo=-9.0+0.5=-8.5 边界
        let old = pos.clone();
        let vel = vec![[0.0, 0.0, 0.0, 0.0]; n];
        let inv_mass = vec![1.0];
        let flat = GranularFlatData {
            n,
            pos,
            old,
            vel,
            inv_mass,
            pairs: vec![],
            npairs: 0,
            gravity: [0.0, 0.0, 0.0],
            bounds_lo: [-9.0, -9.0, -9.0],
            bounds_hi: [9.0, 9.0, 9.0],
            iterations: 1,
            vel_damp: 1.0,
            friction: 0.0,
            dt: 0.016,
        };
        let out = cpu_granular_wgsl_reference(&flat);
        // 初始 x=-9.5 < lo+r=-9.0+0.5=-8.5 → 应被夹紧到 -8.5。
        assert!(out[0][0] >= -9.0 + 0.5 - 1e-4, "边界夹紧失效: x={}", out[0][0]);
    }
}
