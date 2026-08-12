//! 几何形状定义与基础查询。
//!
//! M1 支持:球、轴对齐盒(AABB)、凸多面体(以顶点+面定义)。
//! 所有形状都提供"支撑点 (support)"查询,供 GJK/EPA 使用。

use phy_math::{na, Mat3, RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

/// 自定义 serde 模块:把 nalgebra 泛型类型序列化为纯元组,绕过 nalgebra
/// 自带的 `Matrix<T>: Serialize`(要求 `T: nalgebra::Scalar`)带来的 impl 传播问题。
///
/// 元素以原始位模式(u64)序列化,保证 `save → load` 逐位(bit-exact)还原,
/// 避免文本 JSON 对落在 1-ULP 边界的浮点(如 `-1.5999999999999999` 与 `-1.6`)
/// 在反序列化时被 `f64::from_str` 舍入到相邻可表示值,破坏 replay 的确定性。
pub mod serde_geom {
    use num_traits::FromPrimitive;
    use phy_math::{na, RealField, Vec3};
    use serde::de::DeserializeOwned;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    /// 把 `T` 标量逐位(bit-exact)编码为 `u64` 位模式,用于 JSON 存档/读档。
    ///
    /// 文本 JSON 序列化对恰好落在 1-ULP 边界的 f64(如 `-1.5999999999999999`
    /// 与 `-1.6`)在反序列化时会被 Rust 的 `f64::from_str` 舍入到相邻可表示值,
    /// 破坏 replay 的确定性。改为传输原始位模式即可逐位还原。
    ///
    /// `RealField: ComplexField: SupersetOf<f64>`,故 `na::try_convert(v)` 将
    /// 任意标量 `T` 无损转为 `f64`;反方向用 `FromPrimitive::from_f64`(同样是
    /// `ComplexField` 的 supertrait),二者都只需 `T: RealField` 即可,不会把
    /// 额外的 trait bound 传播到所有使用 `serde_geom` 的容器类型。
    #[inline]
    fn to_bits<T: RealField + Copy>(v: T) -> u64 {
        let f: f64 = na::try_convert(v).expect("scalar -> f64");
        f.to_bits()
    }
    #[inline]
    fn from_bits<T: RealField + Copy + FromPrimitive>(bits: u64) -> T {
        let f = f64::from_bits(bits);
        T::from_f64(f).expect("f64 -> scalar")
    }

    pub fn serialize<S: Serializer, T: RealField + Serialize + Copy>(v: &Vec3<T>, s: S) -> Result<S::Ok, S::Error> {
        [to_bits(v[0]), to_bits(v[1]), to_bits(v[2])].serialize(s)
    }
    pub fn deserialize<'de, D: Deserializer<'de>, T: RealField + DeserializeOwned + Copy + num_traits::FromPrimitive>(
        d: D,
    ) -> Result<Vec3<T>, D::Error> {
        let a = <[u64; 3]>::deserialize(d)?;
        Ok(Vec3::new(from_bits(a[0]), from_bits(a[1]), from_bits(a[2])))
    }

    pub mod vec3_vec {
        use super::{from_bits, to_bits};
        use phy_math::{RealField, Vec3};
        use serde::de::DeserializeOwned;
        use serde::{Deserialize, Deserializer, Serialize, Serializer};

        pub fn serialize<S: Serializer, T: RealField + Serialize + Copy>(
            v: &Vec<Vec3<T>>,
            s: S,
        ) -> Result<S::Ok, S::Error> {
            v.iter()
                .map(|x| [to_bits(x[0]), to_bits(x[1]), to_bits(x[2])])
                .collect::<Vec<_>>()
                .serialize(s)
        }
        pub fn deserialize<'de, D: Deserializer<'de>, T: RealField + DeserializeOwned + Copy + num_traits::FromPrimitive>(
            d: D,
        ) -> Result<Vec<Vec3<T>>, D::Error> {
            let arr = <Vec<[u64; 3]>>::deserialize(d)?;
            Ok(arr
                .into_iter()
                .map(|a| Vec3::new(from_bits(a[0]), from_bits(a[1]), from_bits(a[2])))
                .collect())
        }
    }

    pub mod quat {
        use super::{from_bits, to_bits};
        use phy_math::{na, RealField};
        use serde::de::DeserializeOwned;
        use serde::{Deserialize, Deserializer, Serialize, Serializer};

        pub fn serialize<S: Serializer, T: RealField + Serialize + Copy>(
            q: &na::UnitQuaternion<T>,
            s: S,
        ) -> Result<S::Ok, S::Error> {
            let c = q.quaternion().clone();
            [to_bits(c.w), to_bits(c.i), to_bits(c.j), to_bits(c.k)].serialize(s)
        }
        pub fn deserialize<'de, D: Deserializer<'de>, T: RealField + DeserializeOwned + Copy + num_traits::FromPrimitive>(
            d: D,
        ) -> Result<na::UnitQuaternion<T>, D::Error> {
            let a = <[u64; 4]>::deserialize(d)?;
            let q = na::Quaternion::new(from_bits(a[0]), from_bits(a[1]), from_bits(a[2]), from_bits(a[3]));
            Ok(na::UnitQuaternion::new_normalize(q))
        }
    }

    /// 体坐标系逆惯性张量(Matrix3<T>)的序列化:展平为 9 元素行主序数组。
    pub mod mat3 {
        use super::{from_bits, to_bits};
        use phy_math::{Mat3, RealField};
        use serde::de::DeserializeOwned;
        use serde::{Deserialize, Deserializer, Serialize, Serializer};

        pub fn serialize<S: Serializer, T: RealField + Serialize + Copy>(
            m: &Mat3<T>,
            s: S,
        ) -> Result<S::Ok, S::Error> {
            [
                to_bits(m[(0, 0)]), to_bits(m[(0, 1)]), to_bits(m[(0, 2)]),
                to_bits(m[(1, 0)]), to_bits(m[(1, 1)]), to_bits(m[(1, 2)]),
                to_bits(m[(2, 0)]), to_bits(m[(2, 1)]), to_bits(m[(2, 2)]),
            ]
            .serialize(s)
        }
        pub fn deserialize<'de, D: Deserializer<'de>, T: RealField + DeserializeOwned + Copy + num_traits::FromPrimitive>(
            d: D,
        ) -> Result<Mat3<T>, D::Error> {
            let a = <[u64; 9]>::deserialize(d)?;
            Ok(Mat3::from_row_slice(&[
                from_bits(a[0]), from_bits(a[1]), from_bits(a[2]),
                from_bits(a[3]), from_bits(a[4]), from_bits(a[5]),
                from_bits(a[6]), from_bits(a[7]), from_bits(a[8]),
            ]))
        }
    }
}

/// 刚体的碰撞形状。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub enum Shape<T: RealField + Copy> {
    /// 球:中心在局部原点,半径 r。
    Sphere { r: T },
    /// 胶囊体:沿局部 **y 轴**的线段(半长 `half_height`)+ 半径 `r`(球扫掠线段)。
    /// 角色控制器 / 人形肢体 / 车辆轮胎的常用碰撞体。凸体,走通用 GJK+EPA 窄相。
    Capsule {
        /// 线段半长(沿局部 y 轴)。
        half_height: T,
        /// 半径。
        r: T,
    },
    /// 轴对齐盒(局部空间半长 extents)。
    Box {
        #[serde(with = "serde_geom")]
        half: Vec3<T>,
    },
    /// 凸多面体:顶点(局部坐标)与索引面。
    Convex {
        #[serde(with = "serde_geom::vec3_vec")]
        vertices: Vec<Vec3<T>>,
        faces: Vec<[usize; 3]>,
    },
    /// 高度场(静态地形):二维格点高度,世界 XZ 平面内 `nx × nz` 个高度值,格距 `cell`。
    /// 高度值按 **z-major** 存储(`heights[x + nx*z]`),原点在局部 `(-nx/2*cell, 0, -nz/2*cell)`。
    /// 非凸,走专用窄相(heightfield_sphere/capsule/box),不可用于 GJK。
    Heightfield {
        /// X 方向格点数。
        nx: usize,
        /// Z 方向格点数。
        nz: usize,
        /// 相邻格点间距。
        cell: T,
        /// 高度值(z-major,长度 `nx*nz`;T 为 f32/f64,serde 默认支持)。
        heights: Vec<T>,
    },
    /// 复合体(Compound):一个刚体挂多个子碰撞器(机械臂/车辆底盘/人形肢体)。
    /// 每个子碰撞器有自己的形状 + 相对质心的局部偏移/旋转。
    /// 窄相逐子形状递归;support 取各子形状沿 dir 最远点;contains 任一子形状含 p。
    Compound {
        subshapes: Vec<SubShape<T>>,
    },
}

/// 复合体的子碰撞器:形状 + 相对 Body 质心的局部位姿。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct SubShape<T: RealField + Copy> {
    /// 子形状(局部坐标,原点在自身中心)。
    pub shape: Shape<T>,
    /// 子形状原点相对质心的局部平移。
    #[serde(with = "serde_geom")]
    pub offset: Vec3<T>,
    /// 子形状局部旋转(相对 Body 朝向)。
    #[serde(with = "serde_geom::quat")]
    pub quat: na::UnitQuaternion<T>,
}

impl<T: RealField + Copy> Shape<T> {
    /// 支撑点:在给定世界方向 `dir`(已含物体朝向)上,形状表面最远的点(局部坐标)。
    /// 球/盒有闭式解;凸多面体用顶点枚举。
    pub fn support_local(&self, dir: &Vec3<T>) -> Vec3<T> {
        match self {
            Shape::Sphere { r } => {
                // 球面最远点 = 沿 dir 的单位向量 * r(退化方向给 +X)。
                let n = dir.normalize();
                n * *r
            }
            Shape::Capsule { half_height, r } => {
                // 线段上沿 dir 投影最大的端点(投影 = ±h*dir.y),再加 r*单位方向。
                // dir.y==0 时线段上任意点投影相同,取中点使纯 x/z 方向无 y 偏移。
                let n = dir.normalize();
                let end = if dir.y > T::zero() {
                    Vec3::new(T::zero(), *half_height, T::zero())
                } else if dir.y < T::zero() {
                    Vec3::new(T::zero(), -*half_height, T::zero())
                } else {
                    Vec3::zeros()
                };
                end + n * *r
            }
            Shape::Box { half } => Vec3::new(
                if dir.x >= T::zero() { half.x } else { -half.x },
                if dir.y >= T::zero() { half.y } else { -half.y },
                if dir.z >= T::zero() { half.z } else { -half.z },
            ),
            Shape::Convex { vertices, .. } => {
                let mut best = vertices[0];
                let mut best_dot = vertices[0].dot(dir);
                for v in vertices.iter().skip(1) {
                    let d = v.dot(dir);
                    if d > best_dot {
                        best_dot = d;
                        best = *v;
                    }
                }
                best
            }
            // 高度场非凸,不用于 GJK;支撑返回局部原点(仅防误用崩溃)。
            Shape::Heightfield { .. } => Vec3::zeros(),
            // 复合体:取各子形状沿 dir 最远的支撑点。
            Shape::Compound { subshapes } => {
                let mut best = subshapes[0].offset;
                let mut best_dot = T::zero();
                for sub in subshapes.iter() {
                    // 子形状局部 dir。
                    let d_local = sub.quat.inverse() * *dir;
                    let s = sub.shape.support_local(&d_local);
                    // 变换回 Body 局部:offset + quat * s。
                    let world_s = sub.offset + sub.quat * s;
                    let d = world_s.dot(dir);
                    if d > best_dot {
                        best_dot = d;
                        best = world_s;
                    }
                }
                best
            }
        }
    }

    /// 最小包围球半径(用于 broad-phase 的球包围)。
    pub fn bounding_sphere_r(&self) -> T {
        match self {
            Shape::Sphere { r } => *r,
            Shape::Capsule { half_height, r } => *half_height + *r,
            Shape::Box { half } => half.norm(),
            Shape::Convex { vertices, .. } => vertices
                .iter()
                .map(|v| v.norm())
                .fold(T::zero(), |a, b| if a > b { a } else { b }),
            // 高度场包围球取对角线半长(覆盖整个地形,供 broad-phase)。
            Shape::Heightfield { nx, nz, cell, heights } => {
                let hx = T::from_f64(*nx as f64).unwrap() * *cell * T::from_f64(0.5).unwrap();
                let hz = T::from_f64(*nz as f64).unwrap() * *cell * T::from_f64(0.5).unwrap();
                let hy = heights
                    .iter()
                    .fold(T::zero(), |a, &h| if h.abs() > a { h.abs() } else { a });
                (hx * hx + hy * hy + hz * hz).sqrt()
            }
            // 复合体:取 max(子形状包围球半径 + offset 长度)。
            Shape::Compound { subshapes } => subshapes
                .iter()
                .map(|s| s.shape.bounding_sphere_r() + s.offset.norm())
                .fold(T::zero(), |a, b| if a > b { a } else { b }),
        }
    }

    /// 局部空间点是否位于形状内部(含表面)。
    /// 球/盒为闭式解;凸多面体用"所有面同侧(内法线朝向内)"判定。
    pub fn contains_local(&self, p: &Vec3<T>) -> bool {
        match self {
            Shape::Sphere { r } => p.norm() <= *r,
            Shape::Capsule { half_height, r } => {
                // 到 y 轴线段 [−h,h] 的最近点距离 ≤ r。
                let y = p.y.clamp(-*half_height, *half_height);
                let cx = p.x * p.x + (p.y - y) * (p.y - y) + p.z * p.z;
                cx <= (*r) * (*r)
            }
            Shape::Box { half } => {
                p.x.abs() <= half.x && p.y.abs() <= half.y && p.z.abs() <= half.z
            }
            Shape::Convex {
                vertices, faces, ..
            } => {
                for f in faces {
                    let a = vertices[f[0]];
                    let b = vertices[f[1]];
                    let c = vertices[f[2]];
                    // 由外向内的法线近似:取 (b-a)×(c-a) 指向质心一侧,
                    // 若 p 在法线反向侧(外侧)则不在内部。
                    let n = (b - a).cross(&(c - a));
                    let to_p = *p - a;
                    if to_p.dot(&n) > T::zero() {
                        return false;
                    }
                }
                true
            }
            // 高度场非凸,不支持点包含(走专用窄相)。
            Shape::Heightfield { .. } => false,
            // 复合体:任一子形状包含 p(把 p 变换到子形状局部)。
            Shape::Compound { subshapes } => subshapes.iter().any(|s| {
                let p_local = s.quat.inverse() * (*p - s.offset);
                s.shape.contains_local(&p_local)
            }),
        }
    }
}

/// 物体的世界位姿 + 形状,构成一个可参与碰撞的实体。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct Body<T: RealField + Copy> {
    pub shape: Shape<T>,
    /// 世界平移。
    #[serde(with = "serde_geom")]
    pub pos: Vec3<T>,
    /// 世界旋转(四元数)。
    #[serde(with = "serde_geom::quat")]
    pub rot: na::UnitQuaternion<T>,
    /// 线速度(世界)。
    #[serde(with = "serde_geom")]
    pub vel: Vec3<T>,
    /// 角速度(世界系,rad/s)。
    #[serde(with = "serde_geom")]
    pub ang_vel: Vec3<T>,
    /// 体坐标系下的逆惯性张量(世界系逆惯性 = rot * inv_inertia_local * rotᵀ)。
    /// 静态/无限转动惯量物体置 0 矩阵。
    #[serde(with = "serde_geom::mat3")]
    pub inv_inertia_local: Mat3<T>,
    /// 反质量(0 = 静态/无限质量)。
    pub inv_mass: T,
    /// 休眠标志(B1):动能长期低于阈值后由 `RigidWorld::step` 置位,
    /// 置位后跳过速度积分/推进(零 CPU),直到被邻近运动体唤醒。
    pub sleeping: bool,
    /// 已持续"低动能"的累计时间(B1),超过 `SolverParams::sleep_time` 即休眠。
    pub sleep_time: T,
    /// B5 碰撞层(bitmask):本 body 所属的层。仅当双方 `layers & 对方 mask` 均非零时才碰撞。
    pub layers: u32,
    /// B5 碰撞掩码(bitmask):本 body 允许与哪些层碰撞。默认 `u32::MAX`(与所有层互通)。
    pub collision_mask: u32,
    /// B2 运动学体标志:不受重力/接触冲量驱动(求解器视其有效反质量为 0),
    /// 但由用户每帧设定 `vel` 主动移动,并推开动态体(角色控制器/传送带/移动平台基础)。
    pub kinematic: bool,
    /// B2 传感器/触发器标志:参与窄相碰撞检测,但【不】产生接触约束(不施加冲量、
    /// 不阻止穿透),仅在 `RigidWorld::sensor_contacts` 中报告重叠事件(拾取道具/触发区域/
    /// 角色进入判定等)。传感器仍受碰撞层(B5)过滤与休眠唤醒(B1)影响。
    pub is_sensor: bool,
}

impl<T: RealField + Copy> Default for Body<T> {
    /// 默认体:退化球体(半径 0)、位于原点、单位朝向、零速度、零逆质量与零逆惯性。
    ///
    /// 逆惯性默认 0(= 无旋转响应),需要真实转动的物体请用 `Body::new` 或
    /// 构造后调用 `set_inertia_from_shape`。该默认仅用于让既有字面量用
    /// `..Default::default()` 补齐新增字段而不破坏编译。
    fn default() -> Self {
        Body {
            shape: Shape::Sphere { r: T::zero() },
            pos: Vec3::zeros(),
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            ang_vel: Vec3::zeros(),
            inv_inertia_local: Mat3::zeros(),
            inv_mass: T::zero(),
            sleeping: false,
            sleep_time: T::zero(),
            layers: u32::MAX,
            collision_mask: u32::MAX,
            kinematic: false,
            is_sensor: false,
        }
    }
}

impl<T: RealField + Copy> Body<T> {
    /// 便捷构造:形状 + 世界位置 + 质量(0 表示静态)。
    ///
    /// 反质量由 `inv_mass` 给出(`mass==0` => 静态)。构造后自动按几何估计并写入
    /// 体坐标逆惯性张量(`set_inertia_from_shape`)。朝向初值单位四元数,速度 0。
    pub fn new(shape: Shape<T>, pos: Vec3<T>, inv_mass: T) -> Self {
        let mut b = Body {
            shape,
            pos,
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            ang_vel: Vec3::zeros(),
            inv_inertia_local: Mat3::zeros(),
            inv_mass,
            sleeping: false,
            sleep_time: T::zero(),
            layers: u32::MAX,
            collision_mask: u32::MAX,
            kinematic: false,
            is_sensor: false,
        };
        b.set_inertia_from_shape();
        b
    }

    /// 支撑点(世界坐标):局部支撑点经旋转+平移变换。
    pub fn support(&self, dir_world: &Vec3<T>) -> Vec3<T> {
        // 把世界方向转回局部空间求支撑,再变换回世界。
        let dir_local = self.rot.inverse() * dir_world;
        let p_local = self.shape.support_local(&dir_local);
        self.rot * p_local + self.pos
    }

    /// 世界坐标 -> 局部坐标。
    pub fn to_local(&self, p_world: &Vec3<T>) -> Vec3<T> {
        self.rot.inverse() * (*p_world - self.pos)
    }

    /// 局部坐标 -> 世界坐标。
    pub fn to_world(&self, p_local: &Vec3<T>) -> Vec3<T> {
        self.rot * *p_local + self.pos
    }

    /// 线速度(供求解器读取)。
    pub fn lin_vel(&self) -> Vec3<T> {
        self.vel
    }

    /// 世界坐标系下的逆惯性张量(由体坐标逆惯性经旋转抬升)。
    pub fn inv_inertia_world(&self) -> Mat3<T> {
        let r = self.rot.quaternion().clone();
        // 用四元数构造旋转矩阵(nalgebra UnitQuaternion -> Matrix3)。
        let m = na::Matrix3::from_columns(&[
            self.rot * Vec3::x(),
            self.rot * Vec3::y(),
            self.rot * Vec3::z(),
        ]);
        let _ = r;
        m * self.inv_inertia_local * m.transpose()
    }

    /// 施加线冲量(静态物体 inv_mass=0 无效果)。
    pub fn apply_impulse(&mut self, j: Vec3<T>) {
        self.vel += j * self.inv_mass;
    }

    /// 在接触点 `r`(相对质心的世界向量)施加冲量 `j`,更新线速度与角速度。
    pub fn apply_impulse_at(&mut self, j: Vec3<T>, r: Vec3<T>) {
        if self.kinematic {
            return; // B2:运动学体不接受任何冲量(由用户直接设定 vel)。
        }
        self.vel += j * self.inv_mass;
        let torque_imp = r.cross(&j);
        self.ang_vel += self.inv_inertia_world() * torque_imp;
    }

    /// B5 碰撞过滤:仅当双方层位与掩码均匹配时才发生碰撞。
    /// 规则:`a.layers & b.collision_mask != 0 && b.layers & a.collision_mask != 0`。
    /// 默认 `layers==collision_mask==u32::MAX` 时恒为 `true`(与现有行为一致)。
    pub fn can_collide_with(&self, other: &Body<T>) -> bool {
        (self.layers & other.collision_mask) != 0 && (other.layers & self.collision_mask) != 0
    }

    /// B2 有效反质量:运动学体返回 0(求解器不对其施加接触冲量,但按自身 `vel` 主动移动)。
    pub fn eff_inv_mass(&self) -> T {
        if self.kinematic {
            T::zero()
        } else {
            self.inv_mass
        }
    }

    /// 由质量与几何计算并写入体坐标逆惯性张量(球体/盒/凸多面体的主惯量近似)。
    ///
    /// 静态物体(`inv_mass==0`)置 0 矩阵(无旋转响应)。
    pub fn set_inertia_from_shape(&mut self) {
        if self.inv_mass <= T::zero() {
            self.inv_inertia_local = Mat3::zeros();
            return;
        }
        let mass = T::one() / self.inv_mass;
        // 用包围盒半长作为等效惯量估计(对角张量,局部主轴 = 世界轴)。
        let half = match &self.shape {
            Shape::Sphere { r } => Vec3::new(*r, *r, *r),
            Shape::Capsule { half_height, r } => {
                let e = *half_height + *r;
                Vec3::new(*r, e, *r)
            }
            Shape::Box { half } => *half,
            Shape::Convex { vertices, .. } => {
                // 取各轴最大投影作为半长。
                let mut h = Vec3::new(T::zero(), T::zero(), T::zero());
                for v in vertices {
                    h.x = if v.x.abs() > h.x { v.x.abs() } else { h.x };
                    h.y = if v.y.abs() > h.y { v.y.abs() } else { h.y };
                    h.z = if v.z.abs() > h.z { v.z.abs() } else { h.z };
                }
                h
            }
            Shape::Heightfield { nx, nz, cell, heights } => {
                // 地形惯量估计:半长取网格范围,高度取最大 |h|。
                let hx = T::from_f64(*nx as f64).unwrap() * *cell * T::from_f64(0.5).unwrap();
                let hz = T::from_f64(*nz as f64).unwrap() * *cell * T::from_f64(0.5).unwrap();
                let hy = heights
                    .iter()
                    .fold(T::zero(), |a, &h| if h.abs() > a { h.abs() } else { a });
                Vec3::new(hx, hy, hz)
            }
            // 复合体:半长取各子形状半长(含 offset)的最大值。
            Shape::Compound { subshapes } => {
                let mut h = Vec3::new(T::zero(), T::zero(), T::zero());
                for s in subshapes.iter() {
                    // 用子形状包围盒半径 + offset 作为范围。
                    let br = s.shape.bounding_sphere_r() + s.offset.norm();
                    h.x = if br > h.x { br } else { h.x };
                    h.y = if br > h.y { br } else { h.y };
                    h.z = if br > h.z { br } else { h.z };
                }
                h
            }
        };
        // 实心长方体主惯量: I_x = m/12 (y²+z²) 等(球用 r 等价)。
        let c = mass / T::from_f64(12.0).unwrap();
        let ix = c * (half.y * half.y + half.z * half.z);
        let iy = c * (half.x * half.x + half.z * half.z);
        let iz = c * (half.x * half.x + half.y * half.y);
        let inv = Mat3::from_diagonal(&Vec3::new(
            if ix > T::zero() { T::one() / ix } else { T::zero() },
            if iy > T::zero() { T::one() / iy } else { T::zero() },
            if iz > T::zero() { T::one() / iz } else { T::zero() },
        ));
        self.inv_inertia_local = inv;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sphere_support_on_axis() {
        let s = Shape::<f64>::Sphere { r: 2.0 };
        let p = s.support_local(&Vec3::new(1.0, 0.0, 0.0));
        assert!((p - Vec3::new(2.0, 0.0, 0.0)).norm() < 1e-9);
    }

    #[test]
    fn box_support_corner() {
        let s = Shape::<f64>::Box {
            half: Vec3::new(1.0, 2.0, 3.0),
        };
        let p = s.support_local(&Vec3::new(1.0, 1.0, 1.0));
        assert!((p - Vec3::new(1.0, 2.0, 3.0)).norm() < 1e-9);
    }

    #[test]
    fn capsule_support_on_axis() {
        // 半高 2 + 半径 1 的胶囊,沿 +y 支撑点应在 (0,3,0)。
        let s = Shape::<f64>::Capsule {
            half_height: 2.0,
            r: 1.0,
        };
        let p = s.support_local(&Vec3::new(0.0, 1.0, 0.0));
        assert!((p - Vec3::new(0.0, 3.0, 0.0)).norm() < 1e-9, "沿+y支撑 {}", p);
        // 沿 +x:线段中心 (0,0,0) + r*+x = (1,0,0)。
        let p2 = s.support_local(&Vec3::new(1.0, 0.0, 0.0));
        assert!((p2 - Vec3::new(1.0, 0.0, 0.0)).norm() < 1e-9, "沿+x支撑 {}", p2);
    }

    #[test]
    fn capsule_contains_and_bounds() {
        let s = Shape::<f64>::Capsule {
            half_height: 2.0,
            r: 1.0,
        };
        // 内部点:胶囊轴上的点和侧面内点。
        assert!(s.contains_local(&Vec3::new(0.0, 0.0, 0.0)));
        assert!(s.contains_local(&Vec3::new(0.9, 1.5, 0.0)), "侧面内点应包含");
        assert!(!s.contains_local(&Vec3::new(1.1, 1.5, 0.0)), "超出半径应不包含");
        // 包围球半径 = h + r = 3。
        assert!((s.bounding_sphere_r() - 3.0).abs() < 1e-9);
    }

    #[test]
    fn convex_support_extreme() {
        let s = Shape::<f64>::Convex {
            vertices: vec![
                Vec3::new(-1.0, -1.0, -1.0),
                Vec3::new(1.0, -1.0, -1.0),
                Vec3::new(0.0, 1.0, 0.0),
            ],
            faces: vec![[0, 1, 2]],
        };
        let p = s.support_local(&Vec3::new(0.0, 1.0, 0.0));
        assert!((p - Vec3::new(0.0, 1.0, 0.0)).norm() < 1e-9);
    }

    #[test]
    fn shape_body_serde_roundtrip() {
        use serde_json;
        let b = Body::<f64> {
            shape: Shape::Convex {
                vertices: vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)],
                faces: vec![[0, 1, 2]],
            },
            pos: Vec3::new(1.0, 2.0, 3.0),
            rot: na::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3),
            vel: Vec3::new(0.5, 0.0, -0.5),
            inv_mass: 0.25,
            ..Default::default()
        };
        let json = serde_json::to_string(&b).unwrap();
        let b2: Body<f64> = serde_json::from_str(&json).unwrap();
        assert!((b.pos - b2.pos).norm() < 1e-12);
        assert!((b.vel - b2.vel).norm() < 1e-12);
        assert!((b.rot.quaternion().w - b2.rot.quaternion().w).abs() < 1e-12);
        match (&b.shape, &b2.shape) {
            (Shape::Convex { vertices: v1, .. }, Shape::Convex { vertices: v2, .. }) => {
                assert_eq!(v1.len(), v2.len());
            }
            _ => panic!("shape kind mismatch"),
        }
    }

    #[test]
    fn body_new_sets_inertia_for_dynamic() {
        // 动态球应写入非零逆惯性;静态体(inv_mass=0)逆惯性应为 0。
        let dyn_b = Body::<f64>::new(Shape::Sphere { r: 1.0 }, Vec3::zeros(), 1.0);
        assert!(dyn_b.inv_inertia_local.trace() > 0.0, "动态体应有逆惯性");
        let stat_b = Body::<f64>::new(Shape::Sphere { r: 1.0 }, Vec3::zeros(), 0.0);
        assert!(stat_b.inv_inertia_local.norm() < 1e-12, "静态体逆惯性应为 0");
    }

    #[test]
    fn box_inertia_is_diagonal_in_local_frame() {
        // 盒体在体坐标下逆惯性应为对角(主轴=世界轴),且长宽越大对应轴惯量越小。
        let b = Body::<f64>::new(
            Shape::Box {
                half: Vec3::new(2.0, 1.0, 0.5),
            },
            Vec3::zeros(),
            1.0,
        );
        // 沿 x 的惯量正比于 (y²+z²),y、z 较小 => Ix 最小 => 逆惯性最大。
        let m = b.inv_inertia_local;
        assert!(m[(0, 1)].abs() < 1e-12 && m[(0, 2)].abs() < 1e-12 && m[(1, 2)].abs() < 1e-12,
            "体坐标逆惯性应是对角");
        assert!(m[(0, 0)] > m[(1, 1)] && m[(1, 1)] > m[(2, 2)],
            "x 轴(短轴)逆惯性应最大");
    }

    #[test]
    fn inv_inertia_world_rotates_with_body() {
        // 把体绕 z 转 90°,逆惯性矩阵应随之旋转,且对角线元素交换。
        let mut b = Body::<f64>::new(
            Shape::Box {
                half: Vec3::new(2.0, 1.0, 0.5),
            },
            Vec3::zeros(),
            1.0,
        );
        let iw0 = b.inv_inertia_world();
        b.rot = na::UnitQuaternion::from_axis_angle(&Vec3::z_axis(), std::f64::consts::FRAC_PI_2);
        let iw90 = b.inv_inertia_world();
        // 旋转后 (0,0) 与原 (1,1) 应近似相等,(1,1) 与原 (0,0) 相等。
        assert!((iw90[(0, 0)] - iw0[(1, 1)]).abs() < 1e-9, "旋转 90° 后逆惯性 (0,0) 应等于原 (1,1)");
        assert!((iw90[(1, 1)] - iw0[(0, 0)]).abs() < 1e-9, "旋转 90° 后逆惯性 (1,1) 应等于原 (0,0)");
    }

    #[test]
    fn apply_impulse_at_off_center_yields_angular_velocity() {
        // 在偏离质心处施加横向冲量应同时产生线速度和角速度。
        let mut b = Body::<f64>::new(Shape::Sphere { r: 1.0 }, Vec3::zeros(), 1.0);
        let r = Vec3::new(0.0, 1.0, 0.0); // 在质心正上方施力
        let j = Vec3::new(1.0, 0.0, 0.0); // 沿 +x 冲量 => 绕 -z 旋转
        b.apply_impulse_at(j, r);
        assert!((b.vel - Vec3::new(1.0, 0.0, 0.0)).norm() < 1e-9, "线速度应为 j/m");
        assert!(b.ang_vel.norm() > 1e-9, "离轴冲量应产生角速度");
        assert!(b.ang_vel.z < 0.0, "绕 +y 位矢 × +x 冲量 => 角速度沿 -z");
    }
}
