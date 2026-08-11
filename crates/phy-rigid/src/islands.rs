//! B4 刚体岛屿(island)分组 + 并行求解基础设施(商用游戏引擎补齐计划 §11)。
//!
//! 岛屿 = 通过接触约束或关节**连通**的刚体集合。不同岛屿之间互不依赖,可在
//! 求解时并行处理(每个岛屿只写自己涉及的刚体,索引互不相交 → 数据无竞争)。
//! 岛屿**内部**仍保持顺序冲量求解(与单线程一致,保证确定性)。
//!
//! 连通关系来源:
//! - 接触约束 `(a, b)`
//! - 关节约束 `(a, b)`
//! 静态体(`inv_mass == 0`)也参与连通——它作为"世界锚"把接触它的动态体拉进同一岛屿。

/// 并查集(路径压缩 + 按秩合并)。
struct Dsu {
    parent: Vec<usize>,
    rank: Vec<usize>,
}

impl Dsu {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
            rank: vec![0; n],
        }
    }

    fn find(&mut self, x: usize) -> usize {
        let mut root = x;
        while self.parent[root] != root {
            root = self.parent[root];
        }
        // 路径压缩。
        let mut cur = x;
        while self.parent[cur] != root {
            let next = self.parent[cur];
            self.parent[cur] = root;
            cur = next;
        }
        root
    }

    fn union(&mut self, a: usize, b: usize) {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra == rb {
            return;
        }
        // 按秩合并。
        if self.rank[ra] < self.rank[rb] {
            self.parent[ra] = rb;
        } else if self.rank[ra] > self.rank[rb] {
            self.parent[rb] = ra;
        } else {
            self.parent[rb] = ra;
            self.rank[ra] += 1;
        }
    }
}

/// 构建岛屿分组。
///
/// 返回 `Vec<Vec<usize>>`:每个元素是同一岛屿内的**全局**刚体索引集合(已排序、去重)。
/// 单连通分量即使只有一个体(如孤立静态体)也自成一组。
///
/// # 参数
/// - `n`:刚体总数。
/// - `contact_pairs`:接触约束的 `(a, b)` 索引对(已过滤传感器)。
/// - `joint_pairs`:关节约束的 `(a, b)` 索引对。
pub fn build_islands(
    n: usize,
    contact_pairs: &[(usize, usize)],
    joint_pairs: &[(usize, usize)],
) -> Vec<Vec<usize>> {
    let mut dsu = Dsu::new(n);
    for &(a, b) in contact_pairs {
        if a < n && b < n {
            dsu.union(a, b);
        }
    }
    for &(a, b) in joint_pairs {
        if a < n && b < n {
            dsu.union(a, b);
        }
    }
    // 按根分组。
    let mut groups: std::collections::BTreeMap<usize, Vec<usize>> =
        std::collections::BTreeMap::new();
    for i in 0..n {
        let root = dsu.find(i);
        groups.entry(root).or_default().push(i);
    }
    // BTreeMap 保证按 root 升序 → 岛屿处理顺序固定(确定性)。
    groups.into_values().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn islands_separates_disconnected_components() {
        // 4 体:0-1 接触,2 孤立,3 与 0 无关但... 这里 0-1 连通,2 独立,3 独立。
        let pairs = vec![(0usize, 1usize)];
        let islands = build_islands(4, &pairs, &[]);
        // 体 0,1 一组;体 2 一组;体 3 一组 → 共 3 个岛屿。
        assert_eq!(islands.len(), 3);
        // 找到含 0 的岛屿,应包含 0 和 1。
        let with_01 = islands.iter().find(|g| g.contains(&0)).unwrap();
        assert_eq!(with_01.len(), 2);
        assert!(with_01.contains(&1));
        // 含 2 的岛屿仅 2。
        let with_2 = islands.iter().find(|g| g.contains(&2)).unwrap();
        assert_eq!(with_2.len(), 1);
    }

    #[test]
    fn islands_unions_via_joints() {
        // 体 0-1 经接触,1-2 经关节 → 0,1,2 同岛;体 3 独立。
        let contacts = vec![(0usize, 1usize)];
        let joints = vec![(1usize, 2usize)];
        let islands = build_islands(4, &contacts, &joints);
        assert_eq!(islands.len(), 2);
        let big = islands.iter().find(|g| g.len() == 3).unwrap();
        assert!(big.contains(&0) && big.contains(&1) && big.contains(&2));
    }

    #[test]
    fn islands_every_body_present_once() {
        // 所有体必须恰好出现在一个岛屿中(无丢失、无重复)。
        let contacts = vec![(0usize, 2usize), (1usize, 3usize)];
        let joints = vec![(2usize, 4usize)];
        let islands = build_islands(5, &contacts, &joints);
        let mut all: Vec<usize> = islands.into_iter().flatten().collect();
        all.sort_unstable();
        assert_eq!(all, vec![0, 1, 2, 3, 4]);
    }
}
