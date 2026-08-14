//! 物理材质(per-body 摩擦 / 恢复系数资源)。
//!
//! 把原本只能全局设置的 `SolverParams::friction` / `SolverParams::restitution`
//! 下沉到每个 `Body`,使"橡胶球 vs 冰面""金属块 vs 木地板"等组合拥有各自的
//! 真实手感。作为**资源**而非算法参数,它可经场景 DSL / 资源表加载,与
//! 渲染侧材质(如 `surface_friction`)对齐,便于编辑器内调参。
//!
//! 接触求解时,一对 body 的材质按组合规则合并:
//! - 恢复系数:`max(a, b)`(任一方弹性大则接触弹性大,Bullet 同款)。
//! - 摩擦系数:几何平均 `sqrt(a·b)`(Couomb 接触,中等摩擦,Box2D 同款)。
//! `SolverParams` 的同名字段保留为"未显式赋材质 body"的全局回退默认值。

use phy_math::RealField;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

/// 物理材质:描述接触层面的表面属性。
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct PhysicsMaterial<T: RealField + Copy> {
    /// 摩擦系数(≥0)。0 = 无摩擦(冰面),1 = 强摩擦(橡胶)。
    pub friction: T,
    /// 恢复系数 [0,1]。`0` = 完全非弹性(泥/黏土),`1` = 完全弹性(钢球)。
    pub restitution: T,
}

impl<T: RealField + Copy> Default for PhysicsMaterial<T> {
    fn default() -> Self {
        Self {
            friction: T::from_f64(0.5).unwrap(),
            restitution: T::from_f64(0.0).unwrap(),
        }
    }
}

impl<T: RealField + Copy> PhysicsMaterial<T> {
    /// 便捷构造。
    pub fn new(friction: T, restitution: T) -> Self {
        Self {
            friction,
            restitution,
        }
    }

    /// 常见预设:冰面(低摩擦、低弹性)。
    pub fn ice() -> Self {
        Self::new(
            T::from_f64(0.05).unwrap(),
            T::from_f64(0.1).unwrap(),
        )
    }

    /// 常见预设:橡胶(高摩擦、中弹性)。
    pub fn rubber() -> Self {
        Self::new(
            T::from_f64(0.9).unwrap(),
            T::from_f64(0.7).unwrap(),
        )
    }

    /// 常见预设:金属(中摩擦、高弹性)。
    pub fn metal() -> Self {
        Self::new(
            T::from_f64(0.4).unwrap(),
            T::from_f64(0.6).unwrap(),
        )
    }

    /// 常见预设:木材(中摩擦、低弹性)。
    pub fn wood() -> Self {
        Self::new(
            T::from_f64(0.5).unwrap(),
            T::from_f64(0.2).unwrap(),
        )
    }

    /// 合并两个材质为接触层等效属性。
    ///
    /// - 恢复系数取 `max`(任一方弹性大则整体弹性大)。
    /// - 摩擦系数取几何平均 `sqrt(a·b)`(中等摩擦,符合 Coulomb 接触经验)。
    pub fn combine(&self, other: &PhysicsMaterial<T>) -> PhysicsMaterial<T> {
        let e = if self.restitution > other.restitution {
            self.restitution
        } else {
            other.restitution
        };
        let mu = (self.friction * other.friction).sqrt();
        PhysicsMaterial {
            friction: mu,
            restitution: e,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_matches_solver_params() {
        // 与 SolverParams::default 的 friction/restitution 一致,保证向后兼容。
        let m = PhysicsMaterial::<f64>::default();
        assert!((m.friction - 0.5).abs() < 1e-12);
        assert!((m.restitution - 0.0).abs() < 1e-12);
    }

    #[test]
    fn combine_restitution_is_max() {
        let a = PhysicsMaterial::<f64>::rubber(); // e=0.7
        let b = PhysicsMaterial::<f64>::wood(); // e=0.2
        let c = a.combine(&b);
        assert!((c.restitution - 0.7).abs() < 1e-12, "restitution 应取 max");
    }

    #[test]
    fn combine_friction_is_geometric_mean() {
        let a = PhysicsMaterial::new(0.9, 0.0);
        let b = PhysicsMaterial::new(0.4, 0.0);
        let c = a.combine(&b);
        let expected = (0.9 * 0.4f64).sqrt();
        assert!((c.friction - expected).abs() < 1e-12, "friction 应取几何平均");
    }
}
