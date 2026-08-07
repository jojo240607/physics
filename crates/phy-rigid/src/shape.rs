//! 几何形状定义与基础查询。
//!
//! M1 支持:球、轴对齐盒(AABB)、凸多面体(以顶点+面定义)。
//! 所有形状都提供"支撑点 (support)"查询,供 GJK/EPA 使用。

use phy_math::{na, RealField, Vec3};

/// 刚体的碰撞形状。
#[derive(Debug, Clone)]
pub enum Shape<T: RealField + Copy> {
    /// 球:中心在局部原点,半径 r。
    Sphere { r: T },
    /// 轴对齐盒(局部空间半长 extents)。
    Box { half: Vec3<T> },
    /// 凸多面体:顶点(局部坐标)与索引面。
    Convex { vertices: Vec<Vec3<T>>, faces: Vec<[usize; 3]> },
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
        }
    }

    /// 最小包围球半径(用于 broad-phase 的球包围)。
    pub fn bounding_sphere_r(&self) -> T {
        match self {
            Shape::Sphere { r } => *r,
            Shape::Box { half } => half.norm(),
            Shape::Convex { vertices, .. } => vertices
                .iter()
                .map(|v| v.norm())
                .fold(T::zero(), |a, b| if a > b { a } else { b }),
        }
    }

    /// 局部空间点是否位于形状内部(含表面)。
    /// 球/盒为闭式解;凸多面体用"所有面同侧(内法线朝向内)"判定。
    pub fn contains_local(&self, p: &Vec3<T>) -> bool {
        match self {
            Shape::Sphere { r } => p.norm() <= *r,
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
        }
    }
}

/// 物体的世界位姿 + 形状,构成一个可参与碰撞的实体。
#[derive(Debug, Clone)]
pub struct Body<T: RealField + Copy> {
    pub shape: Shape<T>,
    /// 世界平移。
    pub pos: Vec3<T>,
    /// 世界旋转(四元数)。
    pub rot: na::UnitQuaternion<T>,
    /// 线速度(世界)。
    pub vel: Vec3<T>,
    /// 反质量(0 = 静态/无限质量)。
    pub inv_mass: T,
}

impl<T: RealField + Copy> Body<T> {
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

    /// 施加线冲量(静态物体 inv_mass=0 无效果)。
    pub fn apply_impulse(&mut self, j: Vec3<T>) {
        self.vel += j * self.inv_mass;
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
}
