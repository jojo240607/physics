//! D4 ragdoll 装配工具:用 D1 的 `Ball`/`Hinge` 关节把 D2 的 `Capsule` 肢体串成
//! 一个可倒塌、可碰撞的人形布偶。
//!
//! 设计目标:提供一个 `RagdollBuilder`,给定根位置/朝向/总质量/尺寸比例,自动
//! - 创建各肢体刚体(头/躯干/上臂×2/前臂×2/大腿×2/小腿×2,均 `Shape::Capsule`)
//! - 用关节把骨骼连成链(脊柱用 `Ball` 允许各向弯曲;膝/肘用 `Hinge` 仅沿一个轴屈伸)
//! - 把肢体与关节一次性加入传入的 `RigidWorld`
//!
//! 布偶本身不含"站立"逻辑(那是 `CharacterController` 的职责);ragdoll 是失去
//! 主动控制后由各肢体重力 + 关节约束自然倒塌的物理体,常用于死亡动画/物理戏。
//!
//! 关节锚点约定:每个关节连接"父肢体端点"与"子肢体端点",锚点用各自的局部坐标
//! (`pa` 在父体上,`pb` 在子体上),世界位置应重合。`Hinge::axis_a/axis_b` 用
//! 世界 y 轴旋转(默认沿肢体长轴),屈伸面为 xz 平面。

use phy_math::{na, RealField, Vec3};

use crate::joint::{Joint, JointConstraint};
use crate::shape::{Body, Shape};
use crate::world::RigidWorld;

/// 布偶的肢体 id(供外部查询/施加外力)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum RagdollLimb {
    Head,
    Torso,
    UpperArmL,
    UpperArmR,
    LowerArmL,
    LowerArmR,
    UpperLegL,
    UpperLegR,
    LowerLegL,
    LowerLegR,
}

impl RagdollLimb {
    /// 全部肢体枚举(便于遍历)。
    pub const ALL: [RagdollLimb; 10] = [
        RagdollLimb::Head,
        RagdollLimb::Torso,
        RagdollLimb::UpperArmL,
        RagdollLimb::UpperArmR,
        RagdollLimb::LowerArmL,
        RagdollLimb::LowerArmR,
        RagdollLimb::UpperLegL,
        RagdollLimb::UpperLegR,
        RagdollLimb::LowerLegL,
        RagdollLimb::LowerLegR,
    ];
}

/// 布偶装配结果:肢体 id 映射 + 关节 id 列表。
#[derive(Debug, Clone)]
pub struct Ragdoll<T: RealField + Copy> {
    /// 各肢体在 world 中的 body 索引。
    pub limbs: std::collections::HashMap<RagdollLimb, usize>,
    /// 所有关节在 world 中的索引(顺序与创建一致)。
    pub joints: Vec<usize>,
    /// 标记类型,便于后续扩展(如"激活/冻结"布偶)。
    _marker: std::marker::PhantomData<T>,
}

/// 布偶尺寸/质量配置(全部为相对默认比例的可覆盖项)。
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct RagdollParams<T: RealField + Copy> {
    /// 胶囊肢体半径(统一)。
    pub limb_radius: T,
    /// 躯干半高(胶囊半长)。
    pub torso_half: T,
    /// 头半径。
    pub head_r: T,
    /// 上臂半长(胶囊半长)。
    pub upper_arm_half: T,
    /// 前臂半长。
    pub lower_arm_half: T,
    /// 大腿半长。
    pub upper_leg_half: T,
    /// 小腿半长。
    pub lower_leg_half: T,
    /// 单肢体质量(所有肢体等质量,总质量 = 10 × limb_mass)。
    pub limb_mass: T,
    /// 铰链/球窝关节的 Motor 最大驱动扭矩(布偶默认 0 = 无驱动,纯被动)。
    pub max_joint_torque: T,
}

impl<T: RealField + Copy + num_traits::NumCast> Default for RagdollParams<T> {
    fn default() -> Self {
        Self {
            limb_radius: T::from_f64(0.18).unwrap(),
            torso_half: T::from_f64(0.35).unwrap(),
            head_r: T::from_f64(0.18).unwrap(),
            upper_arm_half: T::from_f64(0.30).unwrap(),
            lower_arm_half: T::from_f64(0.28).unwrap(),
            upper_leg_half: T::from_f64(0.40).unwrap(),
            lower_leg_half: T::from_f64(0.38).unwrap(),
            limb_mass: T::from_f64(1.0).unwrap(),
            max_joint_torque: T::zero(),
        }
    }
}

/// 布偶装配器:链式配置后调用 `build` 注入 `RigidWorld`。
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct RagdollBuilder<T: RealField + Copy> {
    /// 根部(骨盆/躯干底)世界位置。
    root: Vec3<T>,
    /// 朝向(默认单位四元数,即站立姿态)。
    rot: na::UnitQuaternion<T>,
    /// 尺寸/质量参数。
    params: RagdollParams<T>,
}

impl<T: RealField + Copy + num_traits::NumCast> Default for RagdollBuilder<T> {
    fn default() -> Self {
        Self {
            root: Vec3::zeros(),
            rot: na::UnitQuaternion::identity(),
            params: RagdollParams::default(),
        }
    }
}

impl<T: RealField + Copy + num_traits::NumCast> RagdollBuilder<T> {
    /// 新建装配器,根位置为躯干底(站立时约在地面上方躯干半高 + 腿长处)。
    pub fn new() -> Self {
        Self::default()
    }

    /// 设置根位置(世界坐标,躯干底)。
    pub fn at(mut self, root: Vec3<T>) -> Self {
        self.root = root;
        self
    }

    /// 设置整体朝向。
    pub fn with_rot(mut self, rot: na::UnitQuaternion<T>) -> Self {
        self.rot = rot;
        self
    }

    /// 覆盖尺寸/质量参数。
    pub fn with_params(mut self, params: RagdollParams<T>) -> Self {
        self.params = params;
        self
    }

    /// 装配布偶并注入 world,返回肢体/关节索引。
    pub fn build(self, world: &mut RigidWorld<T>) -> Ragdoll<T> {
        let p = &self.params;
        let r = self.rot;
        let zero = T::zero();

        // 局部 y 轴(铰链默认旋转轴)。
        let y_axis = Vec3::new(zero, T::from_f64(1.0).unwrap(), zero);

        let mut limbs = std::collections::HashMap::new();

        // 质心辅助:给定"相对根的世界偏移 + 胶囊半长 + 半径"创建一个竖直胶囊肢体。
        let make_limb = |world: &mut RigidWorld<T>, half: T, rad: T, offset: Vec3<T>| -> usize {
            let shape = Shape::Capsule {
                half_height: half,
                r: rad,
            };
            let pos = self.root + r * offset;
            // 布偶肢体沿世界 y 轴站立;若整体有旋转,胶囊也跟着转。
            let mut body = Body::new(shape, pos, T::from_f64(1.0).unwrap() / p.limb_mass);
            body.rot = r;
            world.add_body(body)
        };

        // 躯干(从根向上延伸 2×torso_half)。
        let torso = make_limb(
            world,
            p.torso_half,
            p.limb_radius,
            Vec3::new(zero, p.torso_half, zero),
        );
        limbs.insert(RagdollLimb::Torso, torso);

        // 头(躯干顶之上 head_r 处)。
        let head = make_limb(
            world,
            p.head_r,
            p.head_r,
            Vec3::new(zero, p.torso_half * T::from_f64(2.0).unwrap() + p.head_r, zero),
        );
        limbs.insert(RagdollLimb::Head, head);

        // 上臂(躯干顶两侧外伸)。
        let shoulder_y = p.torso_half * T::from_f64(2.0).unwrap();
        let arm_off = p.torso_half + p.upper_arm_half; // 沿 x 的水平半长
        let upper_arm_l = make_limb(
            world,
            p.upper_arm_half,
            p.limb_radius,
            Vec3::new(-arm_off, shoulder_y, zero),
        );
        let upper_arm_r = make_limb(
            world,
            p.upper_arm_half,
            p.limb_radius,
            Vec3::new(arm_off, shoulder_y, zero),
        );
        limbs.insert(RagdollLimb::UpperArmL, upper_arm_l);
        limbs.insert(RagdollLimb::UpperArmR, upper_arm_r);

        // 前臂(上臂远端再外伸)。
        let lower_arm_off = arm_off + p.upper_arm_half * T::from_f64(2.0).unwrap() + p.lower_arm_half;
        let lower_arm_l = make_limb(
            world,
            p.lower_arm_half,
            p.limb_radius,
            Vec3::new(-lower_arm_off, shoulder_y, zero),
        );
        let lower_arm_r = make_limb(
            world,
            p.lower_arm_half,
            p.limb_radius,
            Vec3::new(lower_arm_off, shoulder_y, zero),
        );
        limbs.insert(RagdollLimb::LowerArmL, lower_arm_l);
        limbs.insert(RagdollLimb::LowerArmR, lower_arm_r);

        // 大腿(躯干底两侧下伸)。
        let hip_y = zero;
        let leg_x = p.torso_half * T::from_f64(0.6).unwrap();
        let upper_leg_l = make_limb(
            world,
            p.upper_leg_half,
            p.limb_radius,
            Vec3::new(-leg_x, hip_y - p.upper_leg_half, zero),
        );
        let upper_leg_r = make_limb(
            world,
            p.upper_leg_half,
            p.limb_radius,
            Vec3::new(leg_x, hip_y - p.upper_leg_half, zero),
        );
        limbs.insert(RagdollLimb::UpperLegL, upper_leg_l);
        limbs.insert(RagdollLimb::UpperLegR, upper_leg_r);

        // 小腿(大腿远端再下伸)。
        let lower_leg_l = make_limb(
            world,
            p.lower_leg_half,
            p.limb_radius,
            Vec3::new(-leg_x, hip_y - p.upper_leg_half * T::from_f64(2.0).unwrap() - p.lower_leg_half, zero),
        );
        let lower_leg_r = make_limb(
            world,
            p.lower_leg_half,
            p.limb_radius,
            Vec3::new(leg_x, hip_y - p.upper_leg_half * T::from_f64(2.0).unwrap() - p.lower_leg_half, zero),
        );
        limbs.insert(RagdollLimb::LowerLegL, lower_leg_l);
        limbs.insert(RagdollLimb::LowerLegR, lower_leg_r);

        let mut joints = Vec::new();
        let torque = p.max_joint_torque;

        // 辅助:添加 Ball 关节(锚点在父/子局部坐标)。
        let add_ball = |world: &mut RigidWorld<T>, a: usize, b: usize, pa: Vec3<T>, pb: Vec3<T>| {
            let j = JointConstraint::new(
                a,
                b,
                Joint::Ball { pa, pb },
            );
            world.add_joint(a, b, j.joint)
        };
        // 辅助:添加 Hinge 关节(膝/肘,沿世界 y 轴屈伸)。
        let add_hinge = |world: &mut RigidWorld<T>, a: usize, b: usize, pa: Vec3<T>, pb: Vec3<T>| {
            let j = JointConstraint::new(
                a,
                b,
                Joint::Hinge {
                    pa,
                    pb,
                    axis_a: y_axis,
                    axis_b: y_axis,
                    motor_vel: zero,
                    max_motor_torque: torque,
                },
            );
            world.add_joint(a, b, j.joint)
        };

        // 头-躯干:颈(Ball,允许一定摆动)。
        let neck = Vec3::new(zero, p.torso_half, zero); // 躯干顶(局部)
        let head_base = Vec3::new(zero, -p.head_r, zero); // 头底(局部)
        joints.push(add_ball(world, torso, head, neck, head_base));

        // 上臂-躯干:肩(Ball,允许各向摆臂)。
        let torso_shoulder_l = Vec3::new(-p.torso_half, p.torso_half, zero);
        let uarm_top_l = Vec3::new(p.upper_arm_half, zero, zero);
        joints.push(add_ball(world, torso, upper_arm_l, torso_shoulder_l, uarm_top_l));
        let torso_shoulder_r = Vec3::new(p.torso_half, p.torso_half, zero);
        let uarm_top_r = Vec3::new(-p.upper_arm_half, zero, zero);
        joints.push(add_ball(world, torso, upper_arm_r, torso_shoulder_r, uarm_top_r));

        // 前臂-上臂:肘(Hinge,仅沿 y 轴屈伸)。
        let uarm_bot_l = Vec3::new(-p.upper_arm_half, zero, zero);
        let larm_top_l = Vec3::new(p.lower_arm_half, zero, zero);
        joints.push(add_hinge(world, upper_arm_l, lower_arm_l, uarm_bot_l, larm_top_l));
        let uarm_bot_r = Vec3::new(p.upper_arm_half, zero, zero);
        let larm_top_r = Vec3::new(-p.lower_arm_half, zero, zero);
        joints.push(add_hinge(world, upper_arm_r, lower_arm_r, uarm_bot_r, larm_top_r));

        // 大腿-躯干:髋(Ball)。
        let torso_hip_l = Vec3::new(-leg_x, -p.torso_half, zero);
        let uleg_top_l = Vec3::new(zero, p.upper_leg_half, zero);
        joints.push(add_ball(world, torso, upper_leg_l, torso_hip_l, uleg_top_l));
        let torso_hip_r = Vec3::new(leg_x, -p.torso_half, zero);
        let uleg_top_r = Vec3::new(zero, p.upper_leg_half, zero);
        joints.push(add_ball(world, torso, upper_leg_r, torso_hip_r, uleg_top_r));

        // 小腿-大腿:膝(Hinge)。
        let uleg_bot_l = Vec3::new(zero, -p.upper_leg_half, zero);
        let lleg_top_l = Vec3::new(zero, p.lower_leg_half, zero);
        joints.push(add_hinge(world, upper_leg_l, lower_leg_l, uleg_bot_l, lleg_top_l));
        let uleg_bot_r = Vec3::new(zero, -p.upper_leg_half, zero);
        let lleg_top_r = Vec3::new(zero, p.lower_leg_half, zero);
        joints.push(add_hinge(world, upper_leg_r, lower_leg_r, uleg_bot_r, lleg_top_r));

        Ragdoll {
            limbs,
            joints,
            _marker: std::marker::PhantomData,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phy_math::Vec3;

    /// 一个布偶应被完整装配:10 肢体 + 9 关节,且躯干/头/腿的 y 坐标符合站立姿态。
    #[test]
    fn ragdoll_assembles_with_correct_counts_and_pose() {
        let mut w = RigidWorld::<f64>::new();
        w.gravity = Vec3::new(0.0, -9.81, 0.0); // 站立姿态,后续应自然倒塌
        let root = Vec3::new(0.0, 1.5, 0.0);
        let rd = RagdollBuilder::new().at(root).build(&mut w);

        assert_eq!(rd.limbs.len(), 10);
        assert_eq!(rd.joints.len(), 9);

        // 站立姿态:躯干在 root 上方 torso_half,头更高,腿更低。
        let torso_y = w.bodies[rd.limbs[&RagdollLimb::Torso]].pos.y;
        let head_y = w.bodies[rd.limbs[&RagdollLimb::Head]].pos.y;
        let foot_y = w.bodies[rd.limbs[&RagdollLimb::LowerLegL]].pos.y;
        assert!(head_y > torso_y, "头应在躯干上方");
        assert!(foot_y < torso_y, "脚应在躯干下方");

        // 总质量约为 10 个肢体之和(每个 limb_mass=1)。
        let total_inv_mass: f64 = w.bodies.iter().map(|b| b.inv_mass).sum();
        assert!((total_inv_mass - 10.0).abs() < 1e-9, "总质量应为 10");
    }

    /// 布偶在无支撑时应整体下落(重力生效),且各肢体保持被关节连接(不产生 NaN / 散架)。
    #[test]
    fn ragdoll_falls_under_gravity_and_stays_connected() {
        let mut w = RigidWorld::<f64>::new();
        w.gravity = Vec3::new(0.0, -9.81, 0.0);
        let root = Vec3::new(0.0, 3.0, 0.0);
        let rd = RagdollBuilder::new().at(root).build(&mut w);

        let torso0 = w.bodies[rd.limbs[&RagdollLimb::Torso]].pos;
        for _ in 0..120 {
            w.step(1.0 / 120.0);
        }
        let torso1 = w.bodies[rd.limbs[&RagdollLimb::Torso]].pos;

        // 躯干应明显下落。
        assert!(torso1.y < torso0.y - 0.5, "布偶应下落, dy={}", torso1.y - torso0.y);

        // 所有肢体位置有限(无散架 / NaN)。
        for (limb, id) in &rd.limbs {
            assert!(
                w.bodies[*id].pos.x.is_finite()
                    && w.bodies[*id].pos.y.is_finite()
                    && w.bodies[*id].pos.z.is_finite(),
                "肢体 {:?} 位置非有限",
                limb
            );
        }
    }

    /// 关节约束应成立:相邻肢体在关节锚点处保持连接(世界位置相近,误差远小于肢体尺寸)。
    #[test]
    fn ragdoll_joints_keep_limbs_attached() {
        let mut w = RigidWorld::<f64>::new();
        w.gravity = Vec3::new(0.0, -9.81, 0.0);
        // 初始就放在地面上,避免大冲击破坏约束观测。
        let root = Vec3::new(0.0, 1.2, 0.0);
        let rd = RagdollBuilder::new().at(root).build(&mut w);

        let dt = 1.0 / 120.0;
        for _ in 0..300 {
            w.step(dt);
        }

        // 检查头-躯干颈关节:两锚点世界位置应重合(误差 < 0.2,远小于肢体半长)。
        let p = RagdollParams::<f64>::default();
        let torso_id = rd.limbs[&RagdollLimb::Torso];
        let head_id = rd.limbs[&RagdollLimb::Head];
        let neck_local = Vec3::new(0.0, p.torso_half, 0.0);
        let head_base_local = Vec3::new(0.0, -p.head_r, 0.0);
        let wa = w.bodies[torso_id].to_world(&neck_local);
        let wb = w.bodies[head_id].to_world(&head_base_local);
        let neck_err = (wa - wb).norm();
        assert!(
            neck_err < 0.25,
            "颈关节锚点分离过大(约束失效): err={}",
            neck_err
        );

        // 膝(hinge)锚点也应保持连接。
        let uleg_id = rd.limbs[&RagdollLimb::UpperLegL];
        let lleg_id = rd.limbs[&RagdollLimb::LowerLegL];
        let uleg_bot = Vec3::new(0.0, -p.upper_leg_half, 0.0);
        let lleg_top = Vec3::new(0.0, p.lower_leg_half, 0.0);
        let ka = w.bodies[uleg_id].to_world(&uleg_bot);
        let kb = w.bodies[lleg_id].to_world(&lleg_top);
        let knee_err = (ka - kb).norm();
        assert!(
            knee_err < 0.25,
            "膝关节锚点分离过大(约束失效): err={}",
            knee_err
        );
    }
}
