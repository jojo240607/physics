//! W5: GPU 友好扁平数据。把 `GranularWorld` 的颗粒 + 接触对导出为 GPU 可消费的
//! 紧凑缓冲(位置/速度/半径/反质量分开存,接触对为 `u32` 索引对),供 WebGPU
//! compute 的 `par_pairs_reduce` 内核逐对累加 per-body 位移修正。
//!
//! 数值内核与 `world.rs` 的 Jacobi PBD 完全对应:每对只读预测位置快照,计算
//! 位移修正并累加(确定性固定序 reduce),与 CPU 端逐对相加结果一致。

/// GPU 扁平颗粒 + 接触对数据(W5 前置)。
pub struct GranularFlatData {
    /// 颗粒数。
    pub n: usize,
    /// 预测位置缓冲(每体 `[x,y,z,radius]`)。
    pub pos: Vec<[f32; 4]>,
    /// 旧位置缓冲(速度回写用,[x,y,z,0])。
    pub old: Vec<[f32; 4]>,
    /// 速度缓冲([x,y,z,0])。
    pub vel: Vec<[f32; 4]>,
    /// 反质量(0 = 固定体)。
    pub inv_mass: Vec<f32>,
    /// 接触对(`(i,j)`,`i<j`),flat 为 `[i0,j0,i1,j1,...]`。
    pub pairs: Vec<u32>,
    /// 接触对数量。
    pub npairs: usize,
    /// 重力向量 `[gx,gy,gz]`。
    pub gravity: [f32; 3],
    /// 容器盒下界 `[lo_x,lo_y,lo_z]`。
    pub bounds_lo: [f32; 3],
    /// 容器盒上界 `[hi_x,hi_y,hi_z]`。
    pub bounds_hi: [f32; 3],
    /// PBD 投影迭代次数。
    pub iterations: u32,
    /// 速度阻尼(<1 衰减)。
    pub vel_damp: f32,
    /// 切向摩擦系数。
    pub friction: f32,
    /// 时间步长。
    pub dt: f32,
}
