//! 线性四面体 FEM 连续介质力学核心。
//!
//! 采用**线性四面体单元(常数应变)** 的小变形弹性静/动力学:
//! - 单元刚度矩阵 `Ke = V·Bᵀ·D·B`(B 为分片常数应变矩阵,D 为各向同性弹性矩阵);
//! - 全局刚度矩阵装配后,经固定位移边界(Dirichlet)消元求**静力平衡** `K·u = f`;
//! - 运行时 `step` 用**动力松弛(显式 velocity-Verlet + Rayleigh 阻尼)** 朝平衡演化,
//!   使悬臂梁自由端偏移到与材料力学解析解(梁弯曲柔度 EI)一致。
//!
//! 本实现刻意保持小网格密集求解(测试用),以便结果可直接与解析理论对照。

use nalgebra::DMatrix;
use phy_math::{RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

/// 网格节点:含当前位置、位移、速度、内力、是否固定、集总质量。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct Node<T: RealField + Copy> {
    /// 初始(未变形)参考位置。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub x0: Vec3<T>,
    /// 当前位移(相对 x0)。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub u: Vec3<T>,
    /// 速度。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub v: Vec3<T>,
    /// 当前受力(内力 + 外力)。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub f: Vec3<T>,
    /// 是否固定(位移边界)。
    pub fixed: bool,
    /// 集总质量。
    pub mass: T,
}

impl<T: RealField + Copy> Node<T> {
    /// 当前世界位置 = 参考位置 + 位移。
    pub fn pos(&self) -> Vec3<T> {
        self.x0 + self.u
    }
}

/// 四面体单元:四个节点索引(局部编号 0..3)。
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Tet {
    /// 节点索引(指向 `SolidWorld::nodes`)。
    pub n: [usize; 4],
}

/// 固体世界:节点 + 四面体网格 + 材料参数 + 求解状态。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct SolidWorld<T: RealField + Copy> {
    /// 所有节点。
    pub nodes: Vec<Node<T>>,
    /// 所有四面体单元。
    pub tets: Vec<Tet>,
    /// 杨氏模量 E(抗拉/压刚度)。
    pub young: T,
    /// 泊松比 ν(横向变形比,须 ∈ (-1, 0.5))。
    pub poisson: T,
    /// 密度 ρ(用于动力松弛质量集总与重力)。
    pub density: T,
    /// 重力加速度向量(向下;如 (0,-9.81,0))。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub gravity: Vec3<T>,
    /// 外部集中载荷缓冲(逐节点,由加载函数写入;`step`/`solve_equilibrium` 累加)。
    #[serde(with = "phy_rigid::shape::serde_geom::vec3_vec")]
    pub load: Vec<Vec3<T>>,
    /// Rayleigh 阻尼系数(α,速度比例),保证动力松弛收敛。
    pub damping: T,
    /// 仿真时间。
    pub t: T,
}

impl<T: RealField + Copy + num_traits::ToPrimitive + num_traits::FromPrimitive> SolidWorld<T> {
    /// 构造空固体世界。
    pub fn new(young: T, poisson: T, density: T) -> Self {
        Self {
            nodes: Vec::new(),
            tets: Vec::new(),
            young,
            poisson,
            density,
            gravity: Vec3::new(T::zero(), T::from_f64(-9.81).unwrap(), T::zero()),
            load: Vec::new(),
            damping: T::from_f64(2.0).unwrap(),
            t: T::zero(),
        }
    }

    /// 由规则六面体网格(沿 x/y/z 各 m/n/k 单元)生成四面体化固体块;
    /// 可选把某个面(如 x==0)的节点固定为支座。
    pub fn from_box(
        &mut self,
        lo: Vec3<T>,
        hi: Vec3<T>,
        nx: usize,
        ny: usize,
        nz: usize,
        fix_x_min: bool,
    ) {
        self.nodes.clear();
        self.tets.clear();
        let nx1 = nx + 1;
        let ny1 = ny + 1;
        let nz1 = nz + 1;
        // 节点
        for iz in 0..nz1 {
            for iy in 0..ny1 {
                for ix in 0..nx1 {
                    let fx = T::from_usize(ix).unwrap() / T::from_usize(nx).unwrap();
                    let fy = T::from_usize(iy).unwrap() / T::from_usize(ny).unwrap();
                    let fz = T::from_usize(iz).unwrap() / T::from_usize(nz).unwrap();
                    let x = lo.x + (hi.x - lo.x) * fx;
                    let y = lo.y + (hi.y - lo.y) * fy;
                    let z = lo.z + (hi.z - lo.z) * fz;
                    let fixed = fix_x_min && ix == 0;
                    self.nodes.push(Node {
                        x0: Vec3::new(x, y, z),
                        u: Vec3::zeros(),
                        v: Vec3::zeros(),
                        f: Vec3::zeros(),
                        fixed,
                        mass: T::zero(),
                    });
                }
            }
        }
        let id = |ix: usize, iy: usize, iz: usize| ix + iy * nx1 + iz * nx1 * ny1;
        // 每立方体格剖成 5/6 个四面体(采用 6 Tet 标准剖分,保证不翻转)。
        for iz in 0..nz {
            for iy in 0..ny {
                for ix in 0..nx {
                    let c000 = id(ix, iy, iz);
                    let c100 = id(ix + 1, iy, iz);
                    let c010 = id(ix, iy + 1, iz);
                    let c110 = id(ix + 1, iy + 1, iz);
                    let c001 = id(ix, iy, iz + 1);
                    let c101 = id(ix + 1, iy, iz + 1);
                    let c011 = id(ix, iy + 1, iz + 1);
                    let c111 = id(ix + 1, iy + 1, iz + 1);
                    // 6 四面体剖分(主对角线 000-111)。
                    self.push_tet([c000, c100, c110, c111]);
                    self.push_tet([c000, c100, c111, c101]);
                    self.push_tet([c000, c110, c111, c011]);
                    self.push_tet([c000, c111, c101, c001]);
                    self.push_tet([c000, c011, c111, c001]);
                    self.push_tet([c000, c010, c011, c110]);
                }
            }
        }
        self.compute_lumped_mass();
        self.load = vec![Vec3::zeros(); self.nodes.len()];
    }

    fn push_tet(&mut self, n: [usize; 4]) {
        self.tets.push(Tet { n });
    }

    /// 按单元体积把密度分配到节点,得到集总质量。
    fn compute_lumped_mass(&mut self) {
        for nd in self.nodes.iter_mut() {
            nd.mass = T::zero();
        }
        for tet in &self.tets {
            let v = self.tet_volume(tet);
            let m = self.density * v / T::from_f64(4.0).unwrap(); // 每节点 1/4 单元质量
            for &i in &tet.n {
                self.nodes[i].mass += m;
            }
        }
    }

    /// 拉梅常数 λ、μ(由 E、ν 反算)。
    fn lame(&self) -> (T, T) {
        let e = self.young;
        let nu = self.poisson;
        let one = T::one();
        let two = T::from_f64(2.0).unwrap();
        let mu = e / (two * (one + nu));
        let lam = e * nu / ((one + nu) * (one - two * nu));
        (lam, mu)
    }

    /// 计算四面体带符号体积 *6 = det(M)。
    fn six_volume(&self, tet: &Tet) -> T {
        let p: [Vec3<T>; 4] = [
            self.nodes[tet.n[0]].x0,
            self.nodes[tet.n[1]].x0,
            self.nodes[tet.n[2]].x0,
            self.nodes[tet.n[3]].x0,
        ];
        // M = [[1,x0,y0,z0],...]; det 用标量三重积公式:
        // 6V = (x1-x0)·((x2-x0)×(x3-x0))
        let a = p[1] - p[0];
        let b = p[2] - p[0];
        let c = p[3] - p[0];
        a.dot(&b.cross(&c))
    }

    fn tet_volume(&self, tet: &Tet) -> T {
        let six = self.six_volume(tet);
        let six_abs = if six < T::zero() { -six } else { six };
        six_abs / T::from_f64(6.0).unwrap()
    }

    /// 线性四面体形状函数梯度(∂N_i/∂x, ∂N_i/∂y, ∂N_i/∂z),i=0..3。
    /// 通过 4x4 坐标矩阵的余子式求得,结果除以 6V。
    fn shape_gradients(&self, tet: &Tet) -> [Vec3<T>; 4] {
        let p: [Vec3<T>; 4] = [
            self.nodes[tet.n[0]].x0,
            self.nodes[tet.n[1]].x0,
            self.nodes[tet.n[2]].x0,
            self.nodes[tet.n[3]].x0,
        ];
        // 构造 M 的余子式:对列 2/3/4(对应 x/y/z)在各行 r 的余子式。
        let six = self.six_volume(tet);
        let mut g = [Vec3::zeros(); 4];
        for (i, gi) in g.iter_mut().enumerate() {
            // 余子式 C_{col,row}: 去掉第 row 行、第 col 列后的 3x3 行列式,带符号 (-1)^{row+col}
            let gx = cofactor(&p, i, 0);
            let gy = cofactor(&p, i, 1);
            let gz = cofactor(&p, i, 2);
            *gi = Vec3::new(gx, gy, gz) / six;
        }
        g
    }

    /// 装配并求解**静力平衡**:K·u = f。固定节点对应的位移自由度为 0。
    /// 返回是否求解成功(矩阵正定)。外力由 `self.nodes[i].f` 给出(已含重力/载荷)。
    pub fn solve_equilibrium(&mut self) -> bool {
        let n = self.nodes.len();
        let ndof = 3 * n;
        if ndof == 0 {
            return true;
        }
        let (lam, mu) = self.lame();
        // 全局刚度(密集)
        let mut k = DMatrix::<T>::zeros(ndof, ndof);
        // D 矩阵(6x6) 以标量形式直接用 λ,μ 展开。
        let zero = T::zero();
        let two = T::from_f64(2.0).unwrap();
        let l2m = lam + two * mu;
        for tet in &self.tets {
            let g = self.shape_gradients(tet);
            let v = self.tet_volume(tet);
            // 逐对节点贡献 Ke(12x12)
            for a in 0..4 {
                for b in 0..4 {
                    // 3x3 子块 B_a^T · D · B_b
                    let ga = g[a];
                    let gb = g[b];
                    // 应变-位移耦合系数
                    let cxx = ga.x * gb.x;
                    let cyy = ga.y * gb.y;
                    let czz = ga.z * gb.z;
                    let cxy = ga.x * gb.y + ga.y * gb.x;
                    let cyz = ga.y * gb.z + ga.z * gb.y;
                    let czx = ga.z * gb.x + ga.x * gb.z;
                    // Ke 子块 (3x3):
                    // u_a分量对 u_b分量
                    let kxx = l2m * cxx + mu * (cyy + czz);
                    let kxy = lam * ga.x * gb.y + mu * ga.y * gb.x;
                    let kxz = lam * ga.x * gb.z + mu * ga.z * gb.x;
                    let kyx = lam * ga.y * gb.x + mu * ga.x * gb.y;
                    let kyy = l2m * cyy + mu * (cxx + czz);
                    let kyz = lam * ga.y * gb.z + mu * ga.z * gb.y;
                    let kzx = lam * ga.z * gb.x + mu * ga.x * gb.z;
                    let kzy = lam * ga.z * gb.y + mu * ga.y * gb.z;
                    let kzz = l2m * czz + mu * (cxx + cyy);
                    let ke = [
                        [kxx, kxy, kxz],
                        [kyx, kyy, kyz],
                        [kzx, kzy, kzz],
                    ];
                    for ca in 0..3 {
                        for cb in 0..3 {
                            let row = 3 * tet.n[a] + ca;
                            let col = 3 * tet.n[b] + cb;
                            k[(row, col)] += ke[ca][cb] * v;
                        }
                    }
                }
            }
        }
        // 右端:外力(node.f 当前值 + 集中载荷缓冲)。
        let mut f = DMatrix::<T>::zeros(ndof, 1);
        for (i, nd) in self.nodes.iter().enumerate() {
            let mut fe = nd.f;
            if i < self.load.len() && !nd.fixed {
                fe += self.load[i];
            }
            f[(3 * i, 0)] = fe.x;
            f[(3 * i + 1, 0)] = fe.y;
            f[(3 * i + 2, 0)] = fe.z;
        }
        // 固定边界:把对应行/列缩并为单位(强约束 u=0)。
        for i in 0..n {
            if self.nodes[i].fixed {
                for d in 0..3 {
                    let r = 3 * i + d;
                    for c in 0..ndof {
                        k[(r, c)] = zero;
                        k[(c, r)] = zero;
                    }
                    k[(r, r)] = T::one();
                    f[(r, 0)] = zero;
                }
            }
        }
        // 求解(K 对称正定 → Cholesky;退化时返回失败)。
        let ch = match k.cholesky() {
            Some(c) => c,
            None => return false,
        };
        let sol = ch.solve(&f);
        if sol.nrows() != ndof {
            return false;
        }
        for (i, nd) in self.nodes.iter_mut().enumerate() {
            nd.u.x = sol[(3 * i, 0)];
            nd.u.y = sol[(3 * i + 1, 0)];
            nd.u.z = sol[(3 * i + 2, 0)];
            nd.v = Vec3::zeros();
        }
        true
    }

    /// 显式动力松弛一步(velocity-Verlet + Rayleigh 速度阻尼),朝平衡演化。
    /// 内力 = -K·u(线性弹性),外力 = 重力 + 用户载荷(已存于 node.f 的“外力”部分)。
    /// 这里直接在每步重算内力(经单元应力),避免全局矩阵求逆。
    pub fn step(&mut self, dt: T) {
        // 1) 计算内力:对每单元按当前位移求节点力 = -(K_e · u_e)。
        for (i, nd) in self.nodes.iter_mut().enumerate() {
            nd.f = self.gravity * nd.mass; // 外力 = 重力(集总)
            if i < self.load.len() && !nd.fixed {
                nd.f += self.load[i]; // 叠加用户集中载荷
            }
        }
        let (lam, mu) = self.lame();
        let two = T::from_f64(2.0).unwrap();
        let l2m = lam + two * mu;
        for tet in &self.tets {
            let g = self.shape_gradients(tet);
            let v = self.tet_volume(tet);
            // 单元位移向量(12)
            let mut ue = [Vec3::<T>::zeros(); 4];
            for a in 0..4 {
                ue[a] = self.nodes[tet.n[a]].u;
            }
            // 每节点内力 f_int_a = -∑_b V · (B_a^T D B_b) u_b = -∑_b Ke_ab u_b
            for a in 0..4 {
                let mut fa = Vec3::zeros();
                for b in 0..4 {
                    let ga = g[a];
                    let gb = g[b];
                    let cxx = ga.x * gb.x;
                    let cyy = ga.y * gb.y;
                    let czz = ga.z * gb.z;
                    let cxy = ga.x * gb.y + ga.y * gb.x;
                    let cyz = ga.y * gb.z + ga.z * gb.y;
                    let czx = ga.z * gb.x + ga.x * gb.z;
                    let kxx = l2m * cxx + mu * (cyy + czz);
                    let kxy = lam * ga.x * gb.y + mu * ga.y * gb.x;
                    let kxz = lam * ga.x * gb.z + mu * ga.z * gb.x;
                    let kyx = lam * ga.y * gb.x + mu * ga.x * gb.y;
                    let kyy = l2m * cyy + mu * (cxx + czz);
                    let kyz = lam * ga.y * gb.z + mu * ga.z * gb.y;
                    let kzx = lam * ga.z * gb.x + mu * ga.x * gb.z;
                    let kzy = lam * ga.z * gb.y + mu * ga.y * gb.z;
                    let kzz = l2m * czz + mu * (cxx + cyy);
                    let fb = ue[b];
                    fa.x += (kxx * fb.x + kxy * fb.y + kxz * fb.z) * v;
                    fa.y += (kyx * fb.x + kyy * fb.y + kyz * fb.z) * v;
                    fa.z += (kzx * fb.x + kzy * fb.y + kzz * fb.z) * v;
                }
                // 内力反向(弹性恢复力),加到节点内力累加器(此处 node.f 暂存内力,
                // 稍后与外力合并)。为简单,直接累加到独立变量。
                self.nodes[tet.n[a]].f -= fa;
            }
        }
        // 2) 积分(velocity-Verlet):a = f/m; v += a·dt; 阻尼; u += v·dt。
        let damp = self.damping;
        for nd in self.nodes.iter_mut() {
            if nd.fixed {
                nd.v = Vec3::zeros();
                continue;
            }
            let m = if nd.mass > T::zero() {
                nd.mass
            } else {
                T::one()
            };
            let a = nd.f / m;
            nd.v += a * dt;
            nd.v *= T::one() / (T::one() + damp * dt); // Rayleigh 速度阻尼
            nd.u += nd.v * dt;
        }
        self.t += dt;
    }

    /// 把所有节点位移归零(重置变形)。
    pub fn reset_displacement(&mut self) {
        for nd in self.nodes.iter_mut() {
            nd.u = Vec3::zeros();
            nd.v = Vec3::zeros();
            nd.f = Vec3::zeros();
        }
        self.t = T::zero();
    }

    /// 在节点集合上施加集中力(用于载荷测试 / 端部加载)。
    pub fn add_load(&mut self, node_indices: &[usize], force: Vec3<T>) {
        for &i in node_indices {
            if i < self.nodes.len() {
                self.nodes[i].f += force;
            }
        }
    }

    /// 取某节点当前竖直位移(沿轴 ax: 0=x,1=y,2=z)。
    pub fn node_disp(&self, i: usize, ax: usize) -> T {
        match ax {
            0 => self.nodes[i].u.x,
            1 => self.nodes[i].u.y,
            _ => self.nodes[i].u.z,
        }
    }
}

/// 坐标矩阵 M 在 (去掉第 row 行、第 col 列) 处的余子式(带符号),
/// col 取 0/1/2 分别对应 x/y/z(列索引 2/3/4)。
fn cofactor<T: RealField + Copy>(p: &[Vec3<T>; 4], row: usize, col: usize) -> T {
    // 构造 3x3 余子式矩阵:取除 row 外的三行,列 = [常数(0), X, Y, Z] 去掉坐标轴 (col+1)。
    // 故列集合恒为 [0, 其余两坐标轴],长度 3。
    let coord_cols: Vec<usize> = [1usize, 2, 3]
        .iter()
        .copied()
        .filter(|&c| c != col + 1)
        .collect();
    // 完整 3 列表:第 0 列恒为常数 1,后两列为保留坐标轴。
    let cols: [usize; 3] = [0, coord_cols[0], coord_cols[1]];
    let rows: [usize; 3] = [
        (0..4).filter(|&r| r != row).nth(0).unwrap(),
        (0..4).filter(|&r| r != row).nth(1).unwrap(),
        (0..4).filter(|&r| r != row).nth(2).unwrap(),
    ];
    // 矩阵元素:列 0 为常数 1,列 1/2 取对应坐标轴分量。
    let m = |r: usize, c: usize| -> T {
        match cols[c] {
            0 => T::one(),                 // 常数 1
            1 => p[r].x,
            2 => p[r].y,
            _ => p[r].z,
        }
    };
    let (r0, r1, r2) = (rows[0], rows[1], rows[2]);
    let (c0, c1, c2) = (0usize, 1, 2);
    let det = m(r0, c0) * (m(r1, c1) * m(r2, c2) - m(r2, c1) * m(r1, c2))
        - m(r0, c1) * (m(r1, c0) * m(r2, c2) - m(r2, c0) * m(r1, c2))
        + m(r0, c2) * (m(r1, c0) * m(r2, c1) - m(r2, c0) * m(r1, c1));
    // 符号 (-1)^{row + (col+1)}
    let sign = if (row + col + 1) % 2 == 0 { T::one() } else { -T::one() };
    det * sign
}
