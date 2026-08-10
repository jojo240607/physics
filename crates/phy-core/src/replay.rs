//! 确定性回放(§5.6 S8)。
//!
//! 防战建模 / 可复现演示要求:**同一初始世界 + 同一外部输入序列 → 逐位一致的轨迹**。
//! 本模块提供录制 / 回放基础设施:
//!
//! - [`Rng`]:确定性伪随机源(SplitMix64)。子系统在 `step`/`couple` 中经
//!   [`World::seed`](crate::World::seed) 取用,使随机构造(起始抖动、采样)可复现。
//! - [`Replay`]:录制初始世界快照(调用方以 JSON 字符串形式提供,避免 `phy-core`
//!   反向依赖 `phy-io`)+ 逐帧外部输入 `(dt, seed)`。
//! - [`ReplayPlayer`]:从同一快照重建世界,按录制序列重放,得到逐位一致的终态。
//!
//! 设计要点:`Replay` 本身不持有 `World`(避免泛型 / 依赖纠缠)。录制时调用方负责
//! 在首帧前 dump 一次世界快照字符串;回放时调用方提供 `load(json) -> World<f64>`。
//! 这样 `phy-core` 保持对 `phy-io` 无依赖,而真实存档/读档仍由 `phy-io` 完成。

use serde::{Deserialize, Serialize};

use crate::world::World;

/// 确定性伪随机源(SplitMix64)。
///
/// 仅依赖一个 64 位状态,`next_u64` 输出经 final mix,序列完全由初值决定,
/// 跨平台逐位一致(不依赖 `std` 的 `HashMap` 等不确定顺序结构)。
///
/// 子系统应在每帧经 [`World::seed`](crate::World::seed) 取得世界当前帧种子,
/// 再 `Rng::seed(seed).next_*` 派生本帧所需的随机数。由于种子即帧输入的一部分,
/// 录制序列 `(dt, seed)` 后回放即可复现。
#[derive(Debug, Clone, Copy, Default)]
pub struct Rng {
    state: u64,
}

impl Rng {
    /// 以给定种子构造。
    pub fn seed(seed: u64) -> Self {
        Self { state: seed }
    }

    /// 推进状态并返回 64 位随机数。
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// 返回 `[0, 1)` 区间的 `f64`。
    pub fn next_f64(&mut self) -> f64 {
        // 取高 53 位映射到 [0,1)。
        ((self.next_u64() >> 11) as f64) / (1u64 << 53) as f64
    }

    /// 派生一个独立的子流(用于把"帧种子"进一步按用途分流,互不串扰)。
    pub fn fork(&mut self) -> Rng {
        Rng {
            state: self.next_u64(),
        }
    }
}

/// 单帧外部输入:时间步长 + 该帧确定性随机种子。
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct FrameInput {
    /// 该帧时间步长。
    pub dt: f64,
    /// 该帧确定性随机种子(0 表示无随机性需求)。
    pub seed: u64,
}

/// 回放录制:初始世界快照 + 逐帧输入序列。
///
/// 快照以 JSON 字符串保存(由调用方用 `phy_io::save_world_json` 生成),本结构
/// 只负责持久化字符串,不解析其内容,从而 `phy-core` 不依赖 `phy-io`。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Replay {
    /// 格式版本(兼容性)。
    pub version: u32,
    /// 初始世界快照(JSON,由 `phy_io::save_world_json` 生成)。
    pub snapshot: String,
    /// 逐帧外部输入序列。
    pub frames: Vec<FrameInput>,
}

impl Replay {
    /// 以初始世界快照 JSON 新建录制(尚未记录任何帧)。
    pub fn new(snapshot: String) -> Self {
        Self {
            version: 1,
            snapshot,
            frames: Vec::new(),
        }
    }

    /// 记录一帧输入(在 `world.step_seeded(dt, seed)` 之前调用)。
    pub fn record(&mut self, dt: f64, seed: u64) {
        self.frames.push(FrameInput { dt, seed });
    }

    /// 序列化为 JSON 字符串(用于磁盘 / 网络传输)。
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).expect("serialize replay")
    }

    /// 从 JSON 字符串恢复。
    pub fn from_json(json: &str) -> Self {
        serde_json::from_str(json).expect("deserialize replay")
    }

    /// 总帧数。
    pub fn len(&self) -> usize {
        self.frames.len()
    }

    /// 是否为空(无录制帧)。
    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }

    /// 构造回放器(消费 self)。
    pub fn into_player(self) -> ReplayPlayer {
        ReplayPlayer { replay: self }
    }
}

/// 回放器:从 `Replay` 重建世界并重放输入序列。
///
/// 泛型参数 `L` 为 `Fn(&str) -> World<f64>` 的加载闭包(通常由 `phy_io::load_world_json`
/// 提供),用于把快照 JSON 还原成世界。
pub struct ReplayPlayer {
    replay: Replay,
}

impl ReplayPlayer {
    /// 初始世界快照 JSON(供调用方校验 / 调试)。
    pub fn snapshot(&self) -> &str {
        &self.replay.snapshot
    }

    /// 重放全部录制帧。
    ///
    /// `load` 把快照 JSON 还原为 `World<f64>`;`step` 是「逐帧推进」闭包,内部应调用
    /// `world.step_seeded(frame.dt, frame.seed)`。返回终态世界供调用方与录制时比对
    /// (逐位一致 ⇒ `phy_io::save_world_json(&final) == replay.snapshot()` 经同序列重放
    /// 后应与原始终态相等)。
    pub fn replay<L, S>(self, load: L, mut step: S) -> World<f64>
    where
        L: Fn(&str) -> World<f64>,
        S: FnMut(&mut World<f64>, FrameInput),
    {
        let mut world = load(&self.replay.snapshot);
        for f in self.replay.frames {
            step(&mut world, f);
        }
        world
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::{Subsystem, World};
    use std::any::Any;

    #[test]
    fn rng_is_deterministic_and_portable() {
        let mut a = Rng::seed(12345);
        let mut b = Rng::seed(12345);
        let va: Vec<u64> = (0..8).map(|_| a.next_u64()).collect();
        let vb: Vec<u64> = (0..8).map(|_| b.next_u64()).collect();
        assert_eq!(va, vb, "同种子必须产生同序列");
        assert_ne!(va[0], va[1], "序列不应恒为常数");

        // fork 子流也确定。
        let mut a = Rng::seed(999);
        let mut b = Rng::seed(999);
        assert_eq!(a.fork().next_u64(), b.fork().next_u64());
    }

    #[test]
    fn rng_next_f64_in_unit_interval() {
        let mut r = Rng::seed(7);
        for _ in 0..1000 {
            let x = r.next_f64();
            assert!((0.0..1.0).contains(&x), "应在 [0,1): got {}", x);
        }
    }

    // 用 couple 取 world.seed() 的变体子系统。
    struct SeedAcc {
        acc: u64,
        steps: usize,
    }
    impl Subsystem<f64> for SeedAcc {
        fn as_any(&self) -> &dyn Any {
            self
        }
        fn as_any_mut(&mut self) -> &mut dyn Any {
            self
        }
        fn step(&mut self, _dt: &f64) {
            self.steps += 1;
        }
        fn couple(&mut self, world: &mut World<f64>, _dt: &f64) {
            let mut rng = Rng::seed(world.seed());
            self.acc = self.acc.wrapping_add(rng.next_u64());
        }
    }

    #[test]
    fn replay_reproduces_seed_driven_trajectory() {
        // 构造初始世界(快照用 "INIT" 占位字符串;回放时 load 闭包据此重建)。
        let build = || {
            let mut w: World<f64> = World::new();
            w.add_subsystem(Box::new(SeedAcc { acc: 0, steps: 0 }));
            w
        };
        let load = |_snap: &str| build();

        // 录制阶段:10 帧,每帧一个确定性种子。
        let mut world = build();
        let mut replay = Replay::new("INIT".to_string());
        let mut recorder_acc = 0u64;
        for i in 0..10u64 {
            let dt = 0.016;
            let seed = 0x1000 + i * 7;
            replay.record(dt, seed);
            // 仿真侧也用同一 seed 步进,并独立累计 acc 以与回放比对。
            world.step_seeded(dt, seed);
            let sub = world
                .get_mut(0)
                .unwrap()
                .as_any_mut()
                .downcast_mut::<SeedAcc>()
                .unwrap();
            recorder_acc = sub.acc; // 最后一帧的累计值
        }
        let recorded_final_acc = recorder_acc;

        // 回放阶段:从同一快照 + 录制序列重放。
        let replayed = replay.into_player().replay(load, |w, f| {
            w.step_seeded(f.dt, f.seed);
        });
        let replayed_acc = replayed
            .get(0)
            .unwrap()
            .as_any()
            .downcast_ref::<SeedAcc>()
            .unwrap()
            .acc;

        assert_eq!(
            replayed_acc, recorded_final_acc,
            "回放必须逐位复现种子驱动的轨迹"
        );
    }

    #[test]
    fn replay_json_roundtrip() {
        let mut r = Replay::new("{\"snapshot\":1}".to_string());
        r.record(0.1, 42);
        r.record(0.2, 7);
        let json = r.to_json();
        let back = Replay::from_json(&json);
        assert_eq!(back.snapshot, "{\"snapshot\":1}");
        assert_eq!(back.frames.len(), 2);
        assert_eq!(back.frames[0], FrameInput { dt: 0.1, seed: 42 });
    }
}
