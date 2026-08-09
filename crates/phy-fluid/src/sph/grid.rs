//! 均匀空间哈希网格:邻居查询(单元 = 光滑长度 h),保证线性复杂度。

use std::collections::HashMap;
use std::marker::PhantomData;

use num_traits::ToPrimitive;
use phy_math::{RealField, Vec3};

/// 均匀空间哈希网格:键为 (i,j,k) 整数单元坐标。
pub(crate) struct Grid<T: RealField + Copy + ToPrimitive> {
    cell: T,
    map: HashMap<(i64, i64, i64), Vec<usize>>,
    _t: PhantomData<T>,
}

impl<T: RealField + Copy + ToPrimitive> Grid<T> {
    pub(crate) fn new(cell: T) -> Self {
        Self {
            cell,
            map: HashMap::new(),
            _t: PhantomData,
        }
    }

    #[inline]
    fn key_of(&self, p: &Vec3<T>) -> (i64, i64, i64) {
        let inv = T::one() / self.cell;
        (
            (p.x * inv).floor().to_i64().unwrap_or(0),
            (p.y * inv).floor().to_i64().unwrap_or(0),
            (p.z * inv).floor().to_i64().unwrap_or(0),
        )
    }

    pub(crate) fn build(&mut self, particles: &[crate::particle::Particle<T>]) {
        self.map.clear();
        for (i, pt) in particles.iter().enumerate() {
            self.map.entry(self.key_of(&pt.pos)).or_default().push(i);
        }
    }

    /// 访问位置 `p` 的 3x3x3 邻域单元内所有粒子索引(含自身单元)。
    pub(crate) fn for_each_neighbor<F: FnMut(usize)>(&self, p: &Vec3<T>, mut f: F) {
        let (ci, cj, ck) = self.key_of(p);
        for di in -1..=1i64 {
            for dj in -1..=1i64 {
                for dk in -1..=1i64 {
                    if let Some(ids) = self.map.get(&(ci + di, cj + dj, ck + dk)) {
                        for &id in ids {
                            f(id);
                        }
                    }
                }
            }
        }
    }
}
