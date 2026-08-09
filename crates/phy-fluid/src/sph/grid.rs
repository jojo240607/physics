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

    /// GPU 友好扁平布局(W4 前置):把 HashMap 单元转成固定范围网格 + prefix-sum。
    ///
    /// 不改 `build`/`for_each_neighbor` 现有行为;仅从已构建的 `map` 派生 GPU 数据。
    /// 越界单元坐标 clamp 到 `[min, max]` 范围(误差仅影响极少量边界粒子,可接受)。
    pub(crate) fn to_flat(&self) -> FlatGrid<T> {
        use std::collections::BTreeMap;
        // 扫描所有出现的单元,求包围盒。
        let mut min_i = i64::MAX;
        let mut min_j = i64::MAX;
        let mut min_k = i64::MAX;
        let mut max_i = i64::MIN;
        let mut max_j = i64::MIN;
        let mut max_k = i64::MIN;
        for (k, v) in self.map.iter() {
            if v.is_empty() {
                continue;
            }
            if k.0 < min_i {
                min_i = k.0;
            }
            if k.0 > max_i {
                max_i = k.0;
            }
            if k.1 < min_j {
                min_j = k.1;
            }
            if k.1 > max_j {
                max_j = k.1;
            }
            if k.2 < min_k {
                min_k = k.2;
            }
            if k.2 > max_k {
                max_k = k.2;
            }
        }
        if min_i > max_i {
            // 空网格。
            return FlatGrid {
                cell_start: vec![0],
                sorted: vec![],
                min_i: 0,
                min_j: 0,
                min_k: 0,
                ncx: 1,
                ncy: 1,
                ncz: 1,
                cell: self.cell,
            };
        }
        // 范围 +1 padding,避免邻居查询越界 clamp 丢粒子。
        let min_i = min_i - 1;
        let min_j = min_j - 1;
        let min_k = min_k - 1;
        let ncx = (max_i - min_i + 3) as usize;
        let ncy = (max_j - min_j + 3) as usize;
        let ncz = (max_k - min_k + 3) as usize;
        let nc = ncx * ncy * ncz;

        // 把 (i,j,k) 映射到 1D: c = (i-min_i) + ncx*((j-min_j) + ncy*(k-min_k))
        let flat_idx = |i: i64, j: i64, k: i64| -> usize {
            let ci = (i - min_i) as usize;
            let cj = (j - min_j) as usize;
            let ck = (k - min_k) as usize;
            ci + ncx * (cj + ncy * ck)
        };

        // 计数每个被占用单元(从 map 直接取,越界 clamp)。
        let mut counts = vec![0u32; nc];
        let mut cell_of: BTreeMap<(i64, i64, i64), usize> = BTreeMap::new();
        for (key, ids) in self.map.iter() {
            if ids.is_empty() {
                continue;
            }
            let ci = key.0.clamp(min_i, min_i + ncx as i64 - 1);
            let cj = key.1.clamp(min_j, min_j + ncy as i64 - 1);
            let ck = key.2.clamp(min_k, min_k + ncz as i64 - 1);
            let c = flat_idx(ci, cj, ck);
            cell_of.insert((key.0, key.1, key.2), c);
            counts[c] += ids.len() as u32;
        }

        // prefix-sum → cell_start (size nc+1)
        let mut cell_start = vec![0i32; nc + 1];
        let mut acc = 0i32;
        for c in 0..nc {
            cell_start[c] = acc;
            acc += counts[c] as i32;
        }
        cell_start[nc] = acc;

        // 填 sorted
        let total: usize = acc as usize;
        let mut sorted = vec![0i32; total];
        let mut cursor = cell_start.clone();
        for (key, ids) in self.map.iter() {
            if ids.is_empty() {
                continue;
            }
            let c = *cell_of.get(key).unwrap();
            for &id in ids {
                sorted[cursor[c] as usize] = id as i32;
                cursor[c] += 1;
            }
        }

        FlatGrid {
            cell_start,
            sorted,
            min_i,
            min_j,
            min_k,
            ncx,
            ncy,
            ncz,
            cell: self.cell,
        }
    }
}

/// GPU 友好网格扁平数据(W4):`cell_start[c..c+1]` 为单元 c 在 `sorted` 中的粒子索引区间。
pub(crate) struct FlatGrid<T: RealField + Copy + ToPrimitive> {
    pub cell_start: Vec<i32>,
    pub sorted: Vec<i32>,
    pub min_i: i64,
    pub min_j: i64,
    pub min_k: i64,
    pub ncx: usize,
    pub ncy: usize,
    pub ncz: usize,
    pub cell: T,
}
