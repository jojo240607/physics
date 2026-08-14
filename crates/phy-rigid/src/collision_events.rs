//! 碰撞事件三态回调(B2 / 游戏逻辑刚需)。
//!
//! 游戏逻辑常用"碰撞开始/持续/结束"三态事件 + 接触冲量大小(用于播放音效、触发
//! 伤害、铰链解锁等)。求解器内部已算得每次接触的法向冲量(`ContactConstraint::
//! normal_impulse`),此处仅做"跨帧活跃 pair 集合 diff"生成三态,不引入额外物理。
//!
//! 用法:`RigidWorld::step` 之后调用 `world.drain_collision_events()` 取出本帧事件
//! (取出即清空)。事件基于"上一帧末态活跃接触 vs 本帧末态活跃接触"判定,符合游戏
//! 每帧接触状态语义。

use std::collections::{HashMap, HashSet};

use phy_math::{RealField, Vec3};

/// 碰撞事件三态。
///
/// - `Begin`：一对 body 在本帧首次进入接触(上一帧末态不在接触、本帧末态在接触)。
/// - `Stay`：接触持续(上一帧与本帧末态都在接触)。
/// - `End`：一对 body 在本帧结束接触(上一帧末态在接触、本帧末态不在接触)。
///
/// `a`/`b` 为 body 全局索引(已保证 `a < b`)。`normal` 由 a 指向 b(世界系,单位向量)。
/// `point` 为本帧代表接触点(世界系)。`depth` 为本帧穿透深度(>0,米),**即碰撞强度指标**:
///   - 本引擎用 split-impulse 求解器,穿透的位置修正走独立的 pseudo-velocity 通道,
///     速度层法向冲量在接触帧趋于 0(不可用于强度估算)。故游戏侧应以 `depth` 判断碰撞
///     强度:深度大=撞击猛(可触发破碎/重音效),稳态堆叠 depth≈slop(很小)≠剧烈。近似接触
///     力可用 `depth / dt * effective_mass`(业务方按自身单位换算)。
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum CollisionEvent<T: RealField> {
    Begin {
        a: usize,
        b: usize,
        normal: Vec3<T>,
        point: Vec3<T>,
        depth: T,
    },
    Stay {
        a: usize,
        b: usize,
        normal: Vec3<T>,
        point: Vec3<T>,
        depth: T,
    },
    End {
        a: usize,
        b: usize,
    },
}

/// 碰撞事件追踪器:维护"上一帧活跃接触 pair 集合",与当前帧 diff 生成三态事件。
#[derive(Debug, Clone, Default)]
pub struct CollisionTracker {
    /// 上一帧末态的活跃接触 pair 集合(已归一化 `a < b`)。
    prev: HashSet<(usize, usize)>,
}

impl CollisionTracker {
    /// 创建空追踪器。
    pub fn new() -> Self {
        Self::default()
    }

    /// 给定本帧末态活跃接触,返回本帧事件序列,并更新内部 `prev` 为当前帧。
    ///
    /// `contacts` 每个元素为 `(a, b, normal, point, depth)`(全局索引,无需预先归一化
    /// a<b,本函数内部归一化;depth 为穿透深度>0,作为碰撞强度指标)。一对 body 多个接触
    /// 点时只取首个作代表几何(后续如需全部接触点可扩展为 `Vec<Contact>`)。
    pub fn update<T: RealField + Copy>(
        &mut self,
        contacts: &[(usize, usize, Vec3<T>, Vec3<T>, T)],
    ) -> Vec<CollisionEvent<T>> {
        let mut cur = HashSet::new();
        let mut map: HashMap<(usize, usize), (usize, usize, Vec3<T>, Vec3<T>, T)> =
            HashMap::new();
        for &(a, b, n, p, d) in contacts {
            let key = if a < b { (a, b) } else { (b, a) };
            cur.insert(key);
            // 取第一个出现的接触几何作为代表(多接触点时可扩展)。
            map.entry(key).or_insert((a.min(b), a.max(b), n, p, d));
        }

        let mut events = Vec::new();
        // 当前帧的 pair:上一帧也在 → Stay,否则 Begin。
        for &key in &cur {
            let (a, b, n, p, d) = map[&key];
            if self.prev.contains(&key) {
                events.push(CollisionEvent::Stay {
                    a,
                    b,
                    normal: n,
                    point: p,
                    depth: d,
                });
            } else {
                events.push(CollisionEvent::Begin {
                    a,
                    b,
                    normal: n,
                    point: p,
                    depth: d,
                });
            }
        }
        // 上一帧在、当前帧不在 → End。
        for &key in &self.prev {
            if !cur.contains(&key) {
                events.push(CollisionEvent::End { a: key.0, b: key.1 });
            }
        }

        self.prev = cur;
        events
    }

    /// 重置追踪状态(如世界重建 / 加载场景后,避免把旧世界接触当成本帧 End)。
    pub fn reset(&mut self) {
        self.prev.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phy_math::na::Vector3;

    fn v(x: f64, y: f64, z: f64) -> Vec3<f64> {
        Vector3::new(x, y, z)
    }

    #[test]
    fn tracker_begin_stay_end() {
        let mut tr = CollisionTracker::new();
        // 帧1:无接触 → 无事件。
        let e0: Vec<CollisionEvent<f64>> = tr.update(&[]);
        assert!(e0.is_empty(), "无接触时应无事件");

        // 帧2:(2,5)进入接触 → Begin。
        let e1 = tr.update(&[(2usize, 5usize, v(0.0, 1.0, 0.0), v(0.0, 0.0, 0.0), 0.1)]);
        assert_eq!(e1.len(), 1);
        match &e1[0] {
            CollisionEvent::Begin { a, b, depth, .. } => {
                assert_eq!((*a, *b), (2, 5));
                assert!((*depth - 0.1).abs() < 1e-12);
            }
            _ => panic!("应为 Begin"),
        }

        // 帧3:相同 pair 仍在接触 → Stay。
        let e2 = tr.update(&[(5usize, 2usize, v(0.0, 1.0, 0.0), v(0.0, 0.0, 0.0), 0.05)]);
        assert_eq!(e2.len(), 1);
        match &e2[0] {
            CollisionEvent::Stay { a, b, depth, .. } => {
                // a<b 归一化应得到 (2,5)。
                assert_eq!((*a, *b), (2, 5));
                assert!((*depth - 0.05).abs() < 1e-12);
            }
            _ => panic!("应为 Stay"),
        }

        // 帧4:接触消失 → End。
        let e3: Vec<CollisionEvent<f64>> = tr.update(&[]);
        assert_eq!(e3.len(), 1);
        match &e3[0] {
            CollisionEvent::End { a, b } => assert_eq!((*a, *b), (2, 5)),
            _ => panic!("应为 End"),
        }
    }

    #[test]
    fn tracker_normalizes_a_lt_b() {
        let mut tr = CollisionTracker::new();
        // 传入 (b,a) 反序,事件应归一化为 (a,b) 且 a<b。
        let e: Vec<CollisionEvent<f64>> =
            tr.update(&[(9usize, 3usize, v(1.0, 0.0, 0.0), v(0.0, 0.0, 0.0), 0.0)]);
        match &e[0] {
            CollisionEvent::Begin { a, b, .. } => assert!(*a < *b && (*a, *b) == (3, 9)),
            _ => panic!("应为 Begin 且 a<b"),
        }
    }
}
