//! 空间索引(M15):均匀网格哈希,用于宽相位邻域查询。
//!
//! 把点云按固定边长网格分桶,支撑"给定点/半径,返回邻近对象 id"的查询,
//! 将朴素 O(n²) 邻域搜索降为近似 O(n)(桶内 + 邻近 27 桶)。供流体↔刚体、
//! 光学↔世界耦合等需要"附近对象"的场合复用。

use std::collections::HashMap;

use phy_math::RealField;
use phy_math::Vec3;

/// 由网格索引 `(ix,iy,iz)` 构成的无符号键。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct CellKey {
    ix: i64,
    iy: i64,
    iz: i64,
}

/// 均匀网格空间索引。
///
/// 对象以 `(id, 位置)` 批量插入;查询时返回半径内对象 id(含自身,如需剔除由调用方判断)。
pub struct SpatialGrid<T: RealField> {
    cell_size: T,
    inv_cell: T,
    buckets: HashMap<CellKey, Vec<usize>>,
    points: Vec<Vec3<T>>,
}

impl<T: RealField + Copy + num_traits::ToPrimitive> SpatialGrid<T> {
    /// 用给定边长构造空索引。
    pub fn with_cell_size(cell_size: T) -> Self {
        assert!(cell_size > T::zero(), "cell_size 必须为正");
        Self {
            cell_size,
            inv_cell: T::one() / cell_size,
            buckets: HashMap::new(),
            points: Vec::new(),
        }
    }

    /// 清空并重用(保留容量)。
    pub fn clear(&mut self) {
        self.buckets.clear();
        self.points.clear();
    }

    fn key_of(&self, p: Vec3<T>) -> CellKey {
        let to_i = |x: T| -> i64 {
            let v = x * self.inv_cell;
            num_traits::ToPrimitive::to_i64(&v).unwrap_or(0)
        };
        CellKey {
            ix: to_i(p.x),
            iy: to_i(p.y),
            iz: to_i(p.z),
        }
    }

    /// 批量插入点云;`id` 即插入顺序索引。
    pub fn build(&mut self, points: &[Vec3<T>]) {
        self.clear();
        self.points = points.to_vec();
        for (id, p) in points.iter().enumerate() {
            let k = self.key_of(*p);
            self.buckets.entry(k).or_default().push(id);
        }
    }

    /// 查询以 `center` 为中心、`radius` 为半径球内的对象 id(可能含自身)。
    pub fn neighbors(&self, center: Vec3<T>, radius: T) -> Vec<usize> {
        let r = if radius <= T::zero() {
            self.cell_size
        } else {
            radius
        };
        let span = num_traits::ToPrimitive::to_i64(&(r * self.inv_cell)).unwrap_or(0).max(0);
        let ck = self.key_of(center);
        let mut out = Vec::new();
        let r2 = r * r;
        for dx in -span..=span {
            for dy in -span..=span {
                for dz in -span..=span {
                    let key = CellKey {
                        ix: ck.ix + dx,
                        iy: ck.iy + dy,
                        iz: ck.iz + dz,
                    };
                    if let Some(ids) = self.buckets.get(&key) {
                        for &id in ids {
                            let d = self.points[id] - center;
                            if d.dot(&d) <= r2 {
                                out.push(id);
                            }
                        }
                    }
                }
            }
        }
        out
    }

    /// 桶数量(调试/统计用)。
    pub fn bucket_count(&self) -> usize {
        self.buckets.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phy_math::Vec3;

    #[test]
    fn neighbors_finds_nearby_points_only() {
        // 三个点:原点、近邻 (1,0,0)、远点 (100,0,0)。
        let pts = vec![
            Vec3::new(0.0_f64, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(100.0, 0.0, 0.0),
        ];
        let mut g = SpatialGrid::with_cell_size(2.0);
        g.build(&pts);
        let near = g.neighbors(Vec3::new(0.0, 0.0, 0.0), 1.5);
        assert!(near.contains(&0), "包含原点自身");
        assert!(near.contains(&1), "包含近邻");
        assert!(!near.contains(&2), "不含远点");
    }

    #[test]
    fn grid_partitions_into_buckets() {
        let pts = vec![
            Vec3::new(0.0_f64, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(0.0, 10.0, 0.0),
        ];
        let mut g = SpatialGrid::with_cell_size(5.0);
        g.build(&pts);
        // 三个点分属不同桶。
        assert_eq!(g.bucket_count(), 3);
    }
}
