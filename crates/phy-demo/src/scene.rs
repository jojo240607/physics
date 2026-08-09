//! Demo 场景:多物理场世界。
//!
//! 用 `phy_core::World` 统一调度刚体(M3)、流体 SPH(M5)、连续标量场(M7)。
//! 各子系统独立演化(本里程碑不做跨场耦合),由 `mode` 决定 Demo 当前可视化哪一个。

use std::any::Any;

use phy_core::World;
use phy_field::{AcousticField, EmField, GravField, HeatField, ScalarField, WaveField, Bc};
use phy_fluid::{FluidSubsystem, FluidWorld, SphParams};
use phy_math::{na, RealField, Vec3};
use phy_optics::{OpticBody, OpticScene, OpticSubsystem, Precision, Surface};
use phy_rigid::{Body, Joint, RigidSubsystem, RigidWorld, Shape};
use phy_soft::{SoftBody, SoftSubsystem};

use crate::camera::Camera;
use crate::raster::Framebuffer;

/// 演示模式(决定当前可视化哪个子系统)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DemoMode {
    /// 刚体(默认)。
    Rigid,
    /// 流体 SPH 粒子云。
    Fluid,
    /// 连续标量场(热扩散)切片。
    Heat,
    /// 软体(质点-弹簧,M4)。
    Soft,
    /// 光学(玻璃球 + 地面,Whitted/Approx 离线/实时)。
    Optics,
    /// 流体 + 热场联合耦合(M4e 流体↔热双向耦合,M9 接入 Demo)。
    FluidHeat,
    /// 全耦合综合场景(M10):刚/流/软/热四子系统共存于同一 World,渲染全部。
    All,
    /// 电磁场(M12):电荷密度切片 + 电场矢量箭头。
    Em,
    /// 引力场(M13):质量密度切片 + 引力矢量箭头。
    Grav,
    /// 波动场(M22):标量压强/位移切片。
    Wave,
    /// 声场(M22):标量声压切片。
    Acoustic,
}

impl DemoMode {
    /// 循环到下一个模式。
    pub fn next(self) -> Self {
        match self {
            DemoMode::Rigid => DemoMode::Fluid,
            DemoMode::Fluid => DemoMode::Heat,
            DemoMode::Heat => DemoMode::Soft,
            DemoMode::Soft => DemoMode::Optics,
            DemoMode::Optics => DemoMode::FluidHeat,
            DemoMode::FluidHeat => DemoMode::All,
            DemoMode::All => DemoMode::Em,
            DemoMode::Em => DemoMode::Grav,
            DemoMode::Grav => DemoMode::Wave,
            DemoMode::Wave => DemoMode::Acoustic,
            DemoMode::Acoustic => DemoMode::Rigid,
        }
    }

    /// 模式名(用于 HUD)。
    pub fn name(self) -> &'static str {
        match self {
            DemoMode::Rigid => "Rigid",
            DemoMode::Fluid => "Fluid(SPH)",
            DemoMode::Heat => "Heat Field",
            DemoMode::Soft => "Soft Body",
            DemoMode::Optics => "Optics",
            DemoMode::FluidHeat => "Fluid+Heat (M4e)",
            DemoMode::All => "All (M10)",
            DemoMode::Em => "EM Field (M12)",
            DemoMode::Grav => "Grav Field (M13)",
            DemoMode::Wave => "Wave Field (M22)",
            DemoMode::Acoustic => "Acoustic Field (M22)",
        }
    }
}

/// 子系统在 World 中的固定索引(用于 downcast 渲染)。
const IDX_RIGID: usize = 0;
const IDX_FLUID: usize = 1;
const IDX_HEAT: usize = 2;
const IDX_SOFT: usize = 3;

/// Demo 场景:持有统一世界。
pub struct Scene {
    /// 统一调度世界(刚体 + 流体 + 热场)。
    pub world: World<f64>,
    /// 当前可视化模式。
    pub mode: DemoMode,
    /// 累计步数。
    pub steps: u64,
}

impl Scene {
    /// 构造默认场景:地面 + 掉落刚体 + 流体块 + 热斑。
    pub fn new() -> Self {
        let mut world: World<f64> = World::default();

        // --- 刚体(M3) ---
        let mut rigid = RigidWorld::<f64>::new();
        // 地面(静止大质量盒)。
        rigid.add_body(Body {
            shape: Shape::Box {
                half: Vec3::new(20.0, 0.5, 20.0),
            },
            pos: Vec3::new(0.0, -0.5, 0.0),
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            inv_mass: 0.0,
        
            ..Default::default()
        });
        // 掉落小球(带电,演示电磁洛伦兹力偏转)。
        for i in 0..3 {
            let r = 1.0 + 0.3 * (i as f64);
            rigid.add_charged_body(
                Body::new(
                    Shape::Sphere { r },
                    Vec3::new(-6.0 + i as f64 * 6.0, 8.0 + i as f64 * 2.0, 0.0),
                    1.0 / 2.0,
                ),
                if i == 1 { 5.0 } else { -5.0 }, // 中间球 +5,两侧 -5(相反电荷反向偏转)
            );
        }

        // --- 流体(M5) ---
        let params = phy_fluid::SphParams::<f64>::defaults();
        let mut fluid = FluidWorld::<f64>::new(params);
        fluid.fill_box(
            Vec3::new(-2.0, 1.0, -2.0), // 盒最小角
            Vec3::new(2.0, 5.0, 2.0),   // 盒最大角
            0.3,                        // 粒子间距
            0.1,                        // 内壁内缩
        );

        // --- 热场(M7) ---
        // 32x1x32 薄切片(2D 热扩散),dx=0.5,中心热斑。
        let mut field = ScalarField::<f64>::new(32, 1, 32, 0.5, 0.0, Bc::Neumann);
        let cx = 16;
        let cz = 16;
        for dz in -2..=2 {
            for dx in -2..=2 {
                let x = (cx as isize + dx) as usize;
                let z = (cz as isize + dz) as usize;
                if x < field.nx && z < field.nz {
                    let i = field.idx(x, 0, z);
                    field.u[i] = 1.0;
                }
            }
        }
        let heat = HeatField::<f64>::new(field, 0.1);

        // --- 电磁场(M12) ---
        // 与热场同几何的电荷密度网格(覆盖 [-8,8]³,dx=0.5),默认零电荷;洛伦兹力由
        // 带电刚体在 couple 阶段采样电场施加。外加均匀 B 默认零(纯电场力为主)。
        let erho = ScalarField::<f64>::new(32, 32, 32, 0.5, 0.0, Bc::Neumann);
        let mut em = EmField::<f64>::build(erho, 1.0);
        em.b_ext = Vec3::new(0.0, 0.0, 0.5); // 轻微外加 B,演示 v×B 偏转

        // --- 引力场(M13) ---
        // 与热场/电磁同几何的质量密度网格(覆盖 [-8,8]³,dx=0.5)。在中心下方放一个
        // 静态大质量天体(质量源注入一个网格),形成局部引力井,演示刚体被吸引偏转。
        let grho = ScalarField::<f64>::new(32, 32, 32, 0.5, 0.0, Bc::Neumann);
        let mut grav = GravField::<f64>::build(grho, 1.0);
        // 天体质量源:放在中心 (16,8,16) 区域附近(网格坐标原点在 -8,故世界坐标 0 对应格 16)。
        let gcx = 16usize;
        let gcy = 8usize; // 偏下方(y=-4),演示上方小球被向下吸引
        let gcz = 16usize;
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    grav.rho.add_source(
                        (gcx as isize + dx) as usize,
                        (gcy as isize + dy) as usize,
                        (gcz as isize + dz) as usize,
                        50.0,
                    );
                }
            }
        }

        // --- 软体(M4) ---
        // 悬挂的 6x6x6 晶格软块(顶部层钉扎),落在地面上方自由晃动。
        let soft = SoftBody::<f64>::from_lattice(6, 6, 6, 0.6, Vec3::new(0.0, 2.0, 0.0));
        // 地面高度与刚体地面一致(y=-0.5 是地面盒顶)。
        let mut soft = soft;
        soft.ground_y = -0.5;

        // --- 默认场景启用 M4e 流体↔热双向耦合 ---
        // 让流体子系统感知热场:热浮力 + 对流换热(默认场景即演示自然对流)。
        let mut fluid_sub = FluidSubsystem::new(fluid);
        fluid_sub.thermal_expansion = 0.5; // β:热浮力温度膨胀系数
        fluid_sub.heat_gain = 0.1; // 运动区域对流注入热源

        // --- 默认场景启用 M11 刚体↔热、软体↔热双向耦合 ---
        // 刚体/软体也感知热场:热浮力 + 对流换热,闭合四子系统耦合矩阵。
        let mut rigid_sub = RigidSubsystem::new(rigid);
        rigid_sub.thermal_expansion = 0.5;
        rigid_sub.heat_gain = 0.1;
        rigid_sub.em_coupling = 1.0; // M12: 启用刚体↔电磁洛伦兹耦合
        rigid_sub.grav_coupling = 1.0; // M13: 启用刚体↔引力井双向耦合
        let mut soft_sub = SoftSubsystem::new(soft);
        soft_sub.thermal_expansion = 0.5;
        soft_sub.heat_gain = 0.1;

        // 注册到统一世界(顺序即 step 顺序)。
        world.add_subsystem(Box::new(rigid_sub));
        world.add_subsystem(Box::new(fluid_sub));
        world.add_subsystem(Box::new(heat));
        world.add_subsystem(Box::new(soft_sub));
        world.add_subsystem(Box::new(em)); // 电磁场(M12)
        world.add_subsystem(Box::new(grav)); // 引力场(M13)

        // --- 光学(M16):光学↔世界(刚体)耦合演示 ---
        // 把刚体世界中的地面 + 三个掉落小球镜像为光学体,并标记 source_rigid_idx,
        // 让 OpticSubsystem::couple 每步把刚体最新位姿搬入光学场景(运动玻璃球被光正确折射)。
        let mut optic_scene = OpticScene::<f64>::new();
        // 地面:不透明灰盒(静态,source=None)。
        optic_scene.add(OpticBody::new(
            Body {
                shape: Shape::Box {
                    half: Vec3::new(20.0, 0.5, 20.0),
                },
                pos: Vec3::new(0.0, -0.5, 0.0),
                rot: na::UnitQuaternion::identity(),
                vel: Vec3::zeros(),
                inv_mass: 0.0,
            
            ..Default::default()
        },
            Surface::diffuse(Vec3::new(0.4, 0.4, 0.45)),
        ));
        // 三个掉落小球:玻璃质感,分别映射到刚体索引 1/2/3(随刚体运动)。
        for i in 0..3usize {
            let r = 1.0 + 0.3 * (i as f64);
            optic_scene.add(OpticBody::from_rigid(
                Body::new(
                    Shape::Sphere { r },
                    Vec3::new(-6.0 + i as f64 * 6.0, 8.0 + i as f64 * 2.0, 0.0),
                    1.0 / 2.0,
                ),
                Surface::glass(1.5, Vec3::new(0.8, 0.9, 1.0)),
                i + 1, // 刚体索引:地面=0,故小球为 1/2/3
            ));
        }
        let mut optic_sub = OpticSubsystem::new(optic_scene, Precision::Offline);
        optic_sub.optic_coupling = 1.0; // 启用光学↔刚体同步
        world.add_subsystem(Box::new(optic_sub));

        Self {
            world,
            mode: DemoMode::Rigid,
            steps: 0,
        }
    }

    /// 构造纯流体↔热场耦合演示场景(M4e 验证)。
    ///
    /// 流体盒填充 [-1,1]³、热场覆盖 [-3,3]³ 网格,暖顶冷底配置触发自然对流。
    /// 用于 `DemoMode::FluidHeat` 模式,可单独观察热浮力与对流换热。
    pub fn fluid_heat(warm_top: bool) -> Self {
        let mut world: World<f64> = World::default();

        // --- 流体(M5) ---
        let mut fp = SphParams::defaults();
        fp.bounds_min = Vec3::new(-3.0, -3.0, -3.0);
        fp.bounds_max = Vec3::new(3.0, 3.0, 3.0);
        fp.gravity = Vec3::new(0.0, -9.81, 0.0);
        let mut fluid = FluidWorld::new(fp);
        fluid.fill_box(
            Vec3::new(-1.0, -1.0, -1.0),
            Vec3::new(1.0, 1.0, 1.0),
            0.3,
            0.05,
        );
        let mut fluid_sub = FluidSubsystem::new(fluid);
        fluid_sub.thermal_expansion = 0.5;
        fluid_sub.heat_gain = 0.1;
        world.add_subsystem(Box::new(fluid_sub));

        // --- 热场(M7):下半冷、上半热(暖顶冷底),与流体盒对齐 ---
        let nx = 16usize;
        let dx = 6.0 / (nx as f64 - 1.0); // 覆盖 [-3,3]
        let mut f = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann)
            .with_origin(Vec3::new(-3.0, -3.0, -3.0));
        for iy in 0..nx {
            let y = -3.0 + (iy as f64) * dx;
            let t = if warm_top && y > 0.0 { 50.0 } else { 0.0 };
            for ix in 0..nx {
                for iz in 0..nx {
                    let idx = f.idx(ix, iy, iz);
                    f.u[idx] = t;
                }
            }
        }
        let heat = HeatField::new(f, 0.1);
        world.add_subsystem(Box::new(heat));

        Self {
            world,
            mode: DemoMode::FluidHeat,
            steps: 0,
        }
    }

    // ---- 单场演示场景(场类可视化 M26) ----
    // 这些模式各只挂一个场子系统,聚焦展示该场的几何与矢量(激光/引力井/波前)。

    /// 电磁场演示(M12):均匀电荷密度网格 + 中心一对等量异号点电荷,
    /// 渲染电荷密度切片 + 电场矢量箭头。
    pub fn em() -> Self {
        let mut world: World<f64> = World::default();
        let n = 32usize;
        let span = 16.0;
        let dx = span / (n as f64 - 1.0);
        let mut rho = ScalarField::<f64>::new(n, n, n, dx, 0.0, Bc::Neumann)
            .with_origin(Vec3::new(-span / 2.0, -span / 2.0, -span / 2.0));
        // 中心偏上放一个正电荷,偏下放一个负电荷(对称,演示偶极子场)。
        // 直接写入 u(EmField::step 解泊松时读 rho.u;子系统 step 不刷 src→u)。
        let cx = n / 2;
        let cy_pos = n * 3 / 4;
        let cy_neg = n / 4;
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dxk in -1..=1 {
                    let ip = rho.idx((cx as isize + dxk) as usize, (cy_pos as isize + dy) as usize, (cx as isize + dz) as usize);
                    rho.u[ip] = 10.0;
                    let ineg = rho.idx((cx as isize + dxk) as usize, (cy_neg as isize + dy) as usize, (cx as isize + dz) as usize);
                    rho.u[ineg] = -10.0;
                }
            }
        }
        let mut em = EmField::<f64>::build(rho, 1.0);
        em.b_ext = Vec3::new(0.0, 0.0, 0.2);
        world.add_subsystem(Box::new(em));
        Self { world, mode: DemoMode::Em, steps: 0 }
    }

    /// 引力场演示(M13):均匀质量密度网格 + 中心一个大质量天体,
    /// 渲染质量密度切片 + 引力矢量箭头(指向天体)。
    pub fn grav() -> Self {
        let mut world: World<f64> = World::default();
        let n = 32usize;
        let span = 16.0;
        let dx = span / (n as f64 - 1.0);
        let mut rho = ScalarField::<f64>::new(n, n, n, dx, 0.0, Bc::Neumann)
            .with_origin(Vec3::new(-span / 2.0, -span / 2.0, -span / 2.0));
        let c = n / 2;
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dxk in -1..=1 {
                    let i = rho.idx((c as isize + dxk) as usize, (c as isize + dy) as usize, (c as isize + dz) as usize);
                    rho.u[i] = 50.0;
                }
            }
        }
        let grav = GravField::<f64>::build(rho, 1.0);
        world.add_subsystem(Box::new(grav));
        Self { world, mode: DemoMode::Grav, steps: 0 }
    }

    /// 波动场演示(M22):中心脉冲初始位移,渲染标量位移切片。
    pub fn wave() -> Self {
        let mut world: World<f64> = World::default();
        let n = 48usize;
        let span = 16.0;
        let dx = span / (n as f64 - 1.0);
        let mut f = ScalarField::<f64>::new(n, n, n, dx, 0.0, Bc::Neumann)
            .with_origin(Vec3::new(-span / 2.0, -span / 2.0, -span / 2.0));
        let c = n / 2;
        for dz in -2..=2 {
            for dxk in -2..=2 {
                let i = f.idx((c as isize + dxk) as usize, c, (c as isize + dz) as usize);
                f.u[i] = 1.0;
            }
        }
        let wave = WaveField::<f64>::new(f, 30.0 * 30.0);
        world.add_subsystem(Box::new(wave));
        Self { world, mode: DemoMode::Wave, steps: 0 }
    }

    /// 声场演示(M22):中心声源脉冲,渲染标量声压切片。
    pub fn acoustic() -> Self {
        let mut world: World<f64> = World::default();
        let n = 48usize;
        let span = 16.0;
        let dx = span / (n as f64 - 1.0);
        let mut f = ScalarField::<f64>::new(n, n, n, dx, 0.0, Bc::Neumann)
            .with_origin(Vec3::new(-span / 2.0, -span / 2.0, -span / 2.0));
        let c = n / 2;
        for dz in -2..=2 {
            for dxk in -2..=2 {
                let i = f.idx((c as isize + dxk) as usize, c, (c as isize + dz) as usize);
                f.u[i] = 1.0;
            }
        }
        let ac = AcousticField::<f64>::new(f, 343.0 * 343.0, 0.01);
        world.add_subsystem(Box::new(ac));
        Self { world, mode: DemoMode::Acoustic, steps: 0 }
    }

    /// 推进一帧(固定子步)。
    ///
    /// `World::step` 内部已按 `step` → `couple` 顺序驱动所有子系统,
    /// 软体↔刚体的双向耦合在 `SoftSubsystem::couple` 中经 `World` 完成。
    pub fn step(&mut self) {
        let dt = 1.0 / 60.0;
        self.world.step(dt);
        self.steps += 1;
    }

    /// 切换模式。
    pub fn toggle_mode(&mut self) {
        self.mode = self.mode.next();
    }

    /// 直接设置模式。
    pub fn set_mode(&mut self, mode: DemoMode) {
        // 切到/离开流体热场模式时重建场景(专注对流演示)。
        if mode == DemoMode::FluidHeat || self.mode == DemoMode::FluidHeat {
            if mode == DemoMode::FluidHeat {
                *self = Scene::fluid_heat(true);
                return;
            }
            *self = Scene::new();
            return;
        }
        // 切到/离开单场模式(Em/Grav/Wave/Acoustic)时重建为对应的独立场场景。
        let single_field = matches!(
            mode,
            DemoMode::Em | DemoMode::Grav | DemoMode::Wave | DemoMode::Acoustic
        );
        let leaving_single_field = matches!(
            self.mode,
            DemoMode::Em | DemoMode::Grav | DemoMode::Wave | DemoMode::Acoustic
        );
        if single_field || leaving_single_field {
            *self = match mode {
                DemoMode::Em => Scene::em(),
                DemoMode::Grav => Scene::grav(),
                DemoMode::Wave => Scene::wave(),
                DemoMode::Acoustic => Scene::acoustic(),
                _ => Scene::new(),
            };
            return;
        }
        self.mode = mode;
    }

    /// 重置场景为新构造状态。
    pub fn reset(&mut self) {
        *self = match self.mode {
            DemoMode::FluidHeat => Scene::fluid_heat(true),
            DemoMode::Em => Scene::em(),
            DemoMode::Grav => Scene::grav(),
            DemoMode::Wave => Scene::wave(),
            DemoMode::Acoustic => Scene::acoustic(),
            _ => Scene::new(),
        };
    }

    /// 当前刚体数量(用于 HUD)。非刚体模式下安全返回 0(无刚体子系统)。
    pub fn body_count(&self) -> usize {
        match self.world.get(IDX_RIGID) {
            Some(sub) => match sub.as_any().downcast_ref::<RigidSubsystem<f64>>() {
                Some(r) => r.world.bodies.len(),
                None => 0,
            },
            None => 0,
        }
    }

    /// 把当前 World 序列化为 JSON 字符串(用于 Web / 无文件系统的存档)。
    pub fn save_string(&self) -> Result<String, String> {
        Ok(phy_io::save_world_json(&self.world))
    }

    /// 从 JSON 字符串载入 World(替换当前 world,保留 mode/相机)。
    pub fn load_string(&mut self, s: &str) -> Result<(), String> {
        let w = phy_io::load_world_json(s);
        self.world = w;
        Ok(())
    }

    /// 渲染当前模式。
    pub fn render(&self, fb: &mut Framebuffer, cam: &Camera) {
        match self.mode {
            DemoMode::Rigid => self.render_rigid(fb, cam),
            DemoMode::Fluid => self.render_fluid(fb, cam),
            DemoMode::Heat => self.render_heat(fb, cam),
            DemoMode::Soft => self.render_soft(fb, cam),
            DemoMode::Optics => self.render_optics(fb, cam),
            DemoMode::FluidHeat => self.render_fluid_heat(fb, cam),
            DemoMode::All => self.render_all(fb, cam),
            DemoMode::Em => self.render_em(fb, cam),
            DemoMode::Grav => self.render_grav(fb, cam),
            DemoMode::Wave => self.render_wave(fb, cam),
            DemoMode::Acoustic => self.render_acoustic(fb, cam),
        }
    }

    fn render_rigid(&self, fb: &mut Framebuffer, cam: &Camera) {
        let sub = self
            .world
            .get(IDX_RIGID)
            .unwrap()
            .as_any()
            .downcast_ref::<RigidSubsystem<f64>>()
            .unwrap();
        for body in &sub.world.bodies {
            render_body(fb, cam, body);
        }
    }

    fn render_fluid(&self, fb: &mut Framebuffer, cam: &Camera) {
        let sub = self
            .world
            .get(IDX_FLUID)
            .unwrap()
            .as_any()
            .downcast_ref::<FluidSubsystem<f64>>()
            .unwrap();
        for p in &sub.world.particles {
            if let Some((sx, sy, depth)) = project_point(cam, fb, p.pos) {
                let r = (2.0 * depth).clamp(1.0, 8.0) as i32;
                fb.fill_circle(sx, sy, r, depth as f32, [40u8, 120u8, 255u8]);
            }
        }
    }

    fn render_heat(&self, fb: &mut Framebuffer, cam: &Camera) {
        let sub = self
            .world
            .get(IDX_HEAT)
            .unwrap()
            .as_any()
            .downcast_ref::<HeatField<f64>>()
            .unwrap();
        let f = &sub.field;
        let nx = f.nx;
        let nz = f.nz;
        for ix in 0..nx {
            for iz in 0..nz {
                let v = f.u[f.idx(ix, 0, iz)];
                let wx = (ix as f64 - nx as f64 / 2.0) * f.dx;
                let wz = (iz as f64 - nz as f64 / 2.0) * f.dx;
                let wp = Vec3::new(wx, 0.6, wz);
                if let Some((sx, sy, depth)) = project_point(cam, fb, wp) {
                    let t = v.clamp(0.0, 1.0);
                    let col = [(t * 255.0) as u8, 40u8, ((1.0 - t) * 255.0) as u8];
                    fb.fill_circle(sx, sy, 3, depth as f32, col);
                }
            }
        }
    }

    // --- 场类可视化渲染(M26):标量切片 + 矢量箭头 ---

    /// 渲染一个标量场的中心切片(背景色块)。
    fn render_scalar_slice(
        &self,
        fb: &mut Framebuffer,
        cam: &Camera,
        field: &ScalarField<f64>,
        lo: [u8; 3],
        hi: [u8; 3],
    ) {
        let nx = field.nx;
        let nz = field.nz;
        let tmax = field.max_abs().max(1e-6);
        for ix in 0..nx {
            for iz in 0..nz {
                let v = field.u[field.idx(ix, field.ny / 2, iz)];
                let wx = field.origin.x + (ix as f64) * field.dx;
                let wy = field.origin.y + (field.ny as f64 / 2.0) * field.dx;
                let wz = field.origin.z + (iz as f64) * field.dx;
                if let Some((sx, sy, depth)) = project_point(cam, fb, Vec3::new(wx, wy + 0.05, wz)) {
                    let t = (v / tmax).clamp(-1.0, 1.0);
                    let (a, b) = if t >= 0.0 { (lo, hi) } else { (hi, lo) };
                    let u = t.abs();
                    let col = [
                        ((a[0] as f64 * (1.0 - u) + b[0] as f64 * u)) as u8,
                        ((a[1] as f64 * (1.0 - u) + b[1] as f64 * u)) as u8,
                        ((a[2] as f64 * (1.0 - u) + b[2] as f64 * u)) as u8,
                    ];
                    fb.fill_circle(sx, sy, 3, depth as f32, col);
                }
            }
        }
    }

    /// 渲染一个矢量场(以箭头表示),在中心切片上按 stride 采样。
    fn render_vector_arrows(
        &self,
        fb: &mut Framebuffer,
        cam: &Camera,
        field: &ScalarField<f64>,
        vec: &[Vec3<f64>],
        scale: f64,
    ) {
        let nx = field.nx;
        let ny = field.ny;
        let nz = field.nz;
        let stride = 4usize; // 抽稀,避免箭头过密
        let maxn = vec.len().max(1) as f64;
        let vmax = (0..vec.len())
            .map(|i| vec[i].norm())
            .fold(0.0_f64, f64::max)
            .max(1e-6);
        for ix in (0..nx).step_by(stride) {
            for iz in (0..nz).step_by(stride) {
                let iy = ny / 2;
                let i = field.idx(ix, iy, iz);
                let v = vec[i % vec.len()];
                let wx = field.origin.x + (ix as f64) * field.dx;
                let wy = field.origin.y + (iy as f64) * field.dx;
                let wz = field.origin.z + (iz as f64) * field.dx;
                let base = Vec3::new(wx, wy, wz);
                let tip = base + v * (scale / vmax);
                if let (Some((ax, ay, da)), Some((bx, by, db))) =
                    (project_point(cam, fb, base), project_point(cam, fb, tip))
                {
                    let depth = ((da + db) * 0.5) as f32;
                    // 按强度上色:弱=黄,强=红。
                    let t = (v.norm() / vmax).clamp(0.0, 1.0);
                    let col = [(200.0 + t * 55.0) as u8, (220.0 - t * 200.0) as u8, 40u8];
                    fb.draw_line(ax, ay, bx, by, depth, col);
                    fb.fill_circle(bx, by, 2, depth, col);
                }
                // 标记索引上限防越界(实际网格与 vec 同形)。
                let _ = maxn;
            }
        }
    }

    fn render_em(&self, fb: &mut Framebuffer, cam: &Camera) {
        let sub = self
            .world
            .get(0)
            .unwrap()
            .as_any()
            .downcast_ref::<EmField<f64>>()
            .unwrap();
        // 电荷密度切片(蓝=负,红=正)。
        self.render_scalar_slice(fb, cam, &sub.rho, [40u8, 80u8, 255u8], [255u8, 60u8, 60u8]);
        // 电场矢量箭头。
        self.render_vector_arrows(fb, cam, &sub.rho, &sub.e, 4.0);
    }

    fn render_grav(&self, fb: &mut Framebuffer, cam: &Camera) {
        let sub = self
            .world
            .get(0)
            .unwrap()
            .as_any()
            .downcast_ref::<GravField<f64>>()
            .unwrap();
        // 质量密度切片(灰白)。
        self.render_scalar_slice(fb, cam, &sub.rho, [20u8, 20u8, 20u8], [220u8, 220u8, 220u8]);
        // 引力矢量箭头(指向天体)。
        self.render_vector_arrows(fb, cam, &sub.rho, &sub.g, 4.0);
    }

    fn render_wave(&self, fb: &mut Framebuffer, cam: &Camera) {
        let sub = self
            .world
            .get(0)
            .unwrap()
            .as_any()
            .downcast_ref::<WaveField<f64>>()
            .unwrap();
        self.render_scalar_slice(fb, cam, &sub.field, [20u8, 40u8, 200u8], [200u8, 200u8, 60u8]);
    }

    fn render_acoustic(&self, fb: &mut Framebuffer, cam: &Camera) {
        let sub = self
            .world
            .get(0)
            .unwrap()
            .as_any()
            .downcast_ref::<AcousticField<f64>>()
            .unwrap();
        self.render_scalar_slice(fb, cam, &sub.field, [20u8, 120u8, 60u8], [200u8, 60u8, 200u8]);
    }

    /// 光学演示:用 phy-optics 的实时近似后端渲染一个玻璃球 + 地面,
    /// 复用传入相机位姿直接写入帧缓冲像素。从 App 迁入,使 lib/Web 版也能渲染光学。
    fn render_optics(&self, fb: &mut Framebuffer, cam: &Camera) {
        use phy_optics::to_rgba8;
        let (w, h) = (fb.width as usize, fb.height as usize);
        let yaw = cam.yaw as f64;
        let pitch = cam.pitch as f64;
        let dist = cam.distance as f64;
        let target = Vec3::new(0.0, 0.0, 0.0);
        let eye = Vec3::new(
            dist * (pitch.cos()) * (yaw.sin()),
            dist * pitch.sin(),
            dist * (pitch.cos()) * (yaw.cos()),
        ) + target;
        let up = Vec3::new(0.0, 1.0, 0.0);
        let fov = std::f64::consts::FRAC_PI_4;

        let mut scene = OpticScene::<f64>::new();
        scene.add(OpticBody::new(
            Body {
                shape: Shape::Box {
                    half: Vec3::new(4.0, 0.1, 4.0),
                },
                pos: Vec3::new(0.0, -1.5, 0.0),
                rot: na::UnitQuaternion::identity(),
                vel: Vec3::zeros(),
                inv_mass: 0.0,
            
            ..Default::default()
        },
            Surface::diffuse(Vec3::new(0.5, 0.5, 0.5)),
        ));
        scene.add(OpticBody::new(
            Body {
                shape: Shape::Sphere { r: 1.0 },
                pos: Vec3::new(0.0, 0.0, 0.0),
                rot: na::UnitQuaternion::identity(),
                vel: Vec3::zeros(),
                inv_mass: 0.0,
            
            ..Default::default()
        },
            Surface::glass(1.5, Vec3::new(0.9, 0.95, 1.0)),
        ));
        scene.add(OpticBody::new(
            Body {
                shape: Shape::Sphere { r: 0.5 },
                pos: Vec3::new(1.8, -0.5, 0.5),
                rot: na::UnitQuaternion::identity(),
                vel: Vec3::zeros(),
                inv_mass: 0.0,
            
            ..Default::default()
        },
            Surface::glass(1.33, Vec3::new(0.4, 0.6, 1.0)),
        ));

        let sub = OpticSubsystem::new(scene, Precision::Realtime);
        let mut buf = vec![Vec3::new(0.0, 0.0, 0.0); w * h];
        sub.render_camera(&mut buf, w, h, &eye, &target, &up, fov);
        for i in 0..buf.len() {
            fb.pixels[i] = to_rgba8(&buf[i]);
        }
    }

    fn render_soft(&self, fb: &mut Framebuffer, cam: &Camera) {
        let sub = self
            .world
            .get(IDX_SOFT)
            .unwrap()
            .as_any()
            .downcast_ref::<SoftSubsystem<f64>>()
            .unwrap();
        let body = &sub.body;
        // 弹簧(线段,灰白)。
        for s in &body.springs {
            let pa = body.particles[s.a].pos;
            let pb = body.particles[s.b].pos;
            if let (Some((ax, ay, da)), Some((bx, by, db))) =
                (project_point(cam, fb, pa), project_point(cam, fb, pb))
            {
                let depth = ((da + db) * 0.5) as f32;
                fb.draw_line(ax, ay, bx, by, depth, [180u8, 180u8, 200u8]);
            }
        }
        // 质点(蓝绿,按速度上色)。
        for p in &body.particles {
            if let Some((sx, sy, depth)) = project_point(cam, fb, p.pos) {
                let speed = p.vel.norm();
                // 静止=青,快=洋红。
                let t = (speed / 6.0).clamp(0.0, 1.0);
                let col = [
                    (t * 255.0) as u8,
                    (220.0 - t * 160.0) as u8,
                    (200.0 - t * 40.0) as u8,
                ];
                fb.fill_circle(sx, sy, 3, depth as f32, col);
            }
        }
    }

    /// 流体 + 热场联合渲染(M4e 耦合可视化)。
    ///
    /// 先画热场切片(蓝→红温度色阶),再叠加热流体粒子(蓝点),
    /// 让用户直观看到热浮力导致的对流(暖区粒子上浮、运动区加热场)。
    fn render_fluid_heat(&self, fb: &mut Framebuffer, cam: &Camera) {
        // 动态查找流体/热场子系统索引(不依赖注册顺序)。
        let mut fidx = None;
        let mut hidx = None;
        for i in 0..self.world.subsystem_count() {
            if let Some(s) = self.world.get(i) {
                if s.as_any().downcast_ref::<FluidSubsystem<f64>>().is_some() {
                    fidx = Some(i);
                }
                if s.as_any().downcast_ref::<HeatField<f64>>().is_some() {
                    hidx = Some(i);
                }
            }
        }

        // 1) 热场切片(背景)。
        if let Some(hi) = hidx {
            if let Some(h) = self.world.get(hi).unwrap().as_any().downcast_ref::<HeatField<f64>>() {
                let f = &h.field;
                let nx = f.nx;
                let ny = f.ny;
                let nz = f.nz;
                let iy = ny / 2; // 中间层切片。
                let tmax = f.max_abs().max(1e-6);
                for ix in 0..nx {
                    for iz in 0..nz {
                        let v = f.u[f.idx(ix, iy, iz)];
                        let wx = f.origin.x + (ix as f64) * f.dx;
                        let wy = f.origin.y + (iy as f64) * f.dx;
                        let wz = f.origin.z + (iz as f64) * f.dx;
                        if let Some((sx, sy, depth)) = project_point(cam, fb, Vec3::new(wx, wy + 0.05, wz)) {
                            let t = (v / tmax).clamp(0.0, 1.0);
                            let col = [(t * 255.0) as u8, 40u8, ((1.0 - t) * 255.0) as u8];
                            fb.fill_circle(sx, sy, 3, depth as f32, col);
                        }
                    }
                }
            }
        }

        // 2) 流体粒子(前景,蓝点)。
        if let Some(fi) = fidx {
            if let Some(fs) = self.world.get(fi).unwrap().as_any().downcast_ref::<FluidSubsystem<f64>>() {
                for p in &fs.world.particles {
                    if let Some((sx, sy, depth)) = project_point(cam, fb, p.pos) {
                        let r = (2.0 * depth).clamp(1.0, 8.0) as i32;
                        fb.fill_circle(sx, sy, r, depth as f32, [40u8, 120u8, 255u8]);
                    }
                }
            }
        }
    }

    /// 全耦合综合渲染(M10):四子系统同框。
    ///
    /// 顺序(由远及近):热场切片(蓝→红)→ 软体弹簧(灰白)→ 刚体(实体)→ 流体粒子(蓝)。
    /// 动态 downcast 查各子系统下标,不依赖注册顺序;展示 M4b/d/e 全部耦合共存于一个 World。
    fn render_all(&self, fb: &mut Framebuffer, cam: &Camera) {
        // 动态查找四个子系统索引。
        let mut ridx = None;
        let mut fidx = None;
        let mut sidx = None;
        let mut hidx = None;
        for i in 0..self.world.subsystem_count() {
            if let Some(s) = self.world.get(i) {
                if s.as_any().downcast_ref::<RigidSubsystem<f64>>().is_some() {
                    ridx = Some(i);
                }
                if s.as_any().downcast_ref::<FluidSubsystem<f64>>().is_some() {
                    fidx = Some(i);
                }
                if s.as_any().downcast_ref::<SoftSubsystem<f64>>().is_some() {
                    sidx = Some(i);
                }
                if s.as_any().downcast_ref::<HeatField<f64>>().is_some() {
                    hidx = Some(i);
                }
            }
        }

        // 1) 热场切片(背景)。
        if let Some(hi) = hidx {
            if let Some(h) = self.world.get(hi).unwrap().as_any().downcast_ref::<HeatField<f64>>() {
                let f = &h.field;
                let nx = f.nx;
                let ny = f.ny;
                let nz = f.nz;
                let iy = ny / 2;
                let tmax = f.max_abs().max(1e-6);
                for ix in 0..nx {
                    for iz in 0..nz {
                        let v = f.u[f.idx(ix, iy, iz)];
                        let wx = f.origin.x + (ix as f64) * f.dx;
                        let wy = f.origin.y + (iy as f64) * f.dx;
                        let wz = f.origin.z + (iz as f64) * f.dx;
                        if let Some((sx, sy, depth)) = project_point(cam, fb, Vec3::new(wx, wy + 0.05, wz)) {
                            let t = (v / tmax).clamp(0.0, 1.0);
                            let col = [(t * 255.0) as u8, 40u8, ((1.0 - t) * 255.0) as u8];
                            fb.fill_circle(sx, sy, 3, depth as f32, col);
                        }
                    }
                }
            }
        }

        // 2) 软体弹簧(灰白)。
        if let Some(si) = sidx {
            if let Some(ss) = self.world.get(si).unwrap().as_any().downcast_ref::<SoftSubsystem<f64>>() {
                let body = &ss.body;
                for s in &body.springs {
                    let pa = body.particles[s.a].pos;
                    let pb = body.particles[s.b].pos;
                    if let (Some((ax, ay, da)), Some((bx, by, db))) =
                        (project_point(cam, fb, pa), project_point(cam, fb, pb))
                    {
                        let depth = ((da + db) * 0.5) as f32;
                        fb.draw_line(ax, ay, bx, by, depth, [180u8, 180u8, 200u8]);
                    }
                }
            }
        }

        // 3) 刚体(实体)。
        if let Some(ri) = ridx {
            if let Some(rs) = self.world.get(ri).unwrap().as_any().downcast_ref::<RigidSubsystem<f64>>() {
                for b in &rs.world.bodies {
                    render_body(fb, cam, b);
                }
            }
        }

        // 4) 流体粒子(前景,蓝点)。
        if let Some(fi) = fidx {
            if let Some(fs) = self.world.get(fi).unwrap().as_any().downcast_ref::<FluidSubsystem<f64>>() {
                for p in &fs.world.particles {
                    if let Some((sx, sy, depth)) = project_point(cam, fb, p.pos) {
                        let r = (2.0 * depth).clamp(1.0, 8.0) as i32;
                        fb.fill_circle(sx, sy, r, depth as f32, [40u8, 120u8, 255u8]);
                    }
                }
            }
        }
    }
}
/// 复用 `Camera::view_proj`(f32 矩阵)。
fn project_point(cam: &Camera, fb: &Framebuffer, world: Vec3<f64>) -> Option<(i32, i32, f64)> {
    let aspect = fb.width as f32 / fb.height as f32;
    let vp = cam.view_proj(aspect);
    let clip = vp
        * na::Vector4::new(world.x as f32, world.y as f32, world.z as f32, 1.0);
    if clip.w <= 1e-5 {
        return None;
    }
    let inv = 1.0 / clip.w;
    let ndc_x = clip.x * inv;
    let ndc_y = clip.y * inv;
    let sx = ((ndc_x * 0.5 + 0.5) * fb.width as f32) as i32;
    let sy = ((1.0 - (ndc_y * 0.5 + 0.5)) * fb.height as f32) as i32;
    let depth = inv as f64; // 1/w
    Some((sx, sy, depth))
}

/// 渲染单个刚体(地面 + 球体/盒)。
fn render_body(fb: &mut Framebuffer, cam: &Camera, body: &Body<f64>) {
    match &body.shape {
        Shape::Sphere { r } => {
            if let Some((sx, sy, depth)) = project_point(cam, fb, body.pos) {
                let r_screen = (*r * depth) as i32;
                let col = if body.inv_mass < 1e-9 {
                    [90u8, 90u8, 90u8]
                } else {
                    [220u8, 60u8, 60u8]
                };
                fb.fill_circle(sx, sy, r_screen.max(1), depth as f32, col);
            }
        }
        Shape::Box { half } => {
            let corners = [
                Vec3::new(body.pos.x - half.x, body.pos.y - half.y, body.pos.z - half.z),
                Vec3::new(body.pos.x + half.x, body.pos.y - half.y, body.pos.z - half.z),
                Vec3::new(body.pos.x + half.x, body.pos.y + half.y, body.pos.z - half.z),
                Vec3::new(body.pos.x - half.x, body.pos.y + half.y, body.pos.z - half.z),
                Vec3::new(body.pos.x - half.x, body.pos.y - half.y, body.pos.z + half.z),
                Vec3::new(body.pos.x + half.x, body.pos.y - half.y, body.pos.z + half.z),
                Vec3::new(body.pos.x + half.x, body.pos.y + half.y, body.pos.z + half.z),
                Vec3::new(body.pos.x - half.x, body.pos.y + half.y, body.pos.z + half.z),
            ];
            let mut minx = i32::MAX;
            let mut maxx = i32::MIN;
            let mut miny = i32::MAX;
            let mut maxy = i32::MIN;
            let mut depth = f64::INFINITY;
            for v in &corners {
                if let Some((sx, sy, d)) = project_point(cam, fb, *v) {
                    depth = depth.min(d);
                    minx = minx.min(sx);
                    maxx = maxx.max(sx);
                    miny = miny.min(sy);
                    maxy = maxy.max(sy);
                }
            }
            if minx > maxx || miny > maxy {
                return;
            }
            let col = if body.inv_mass < 1e-9 {
                [90u8, 90u8, 90u8]
            } else {
                [220u8, 60u8, 60u8]
            };
            for y in miny..=maxy {
                for x in minx..=maxx {
                    fb.set_depth(x, y, depth as f32, col);
                }
            }
        }
        Shape::Convex { .. } => { /* 凸体简略不渲染 */ }
    }
}

// 确保 Subsystem trait 仍是 Any(供 downcast)。
fn _assert_any<T: RealField>(_: &dyn Any) {}

#[cfg(test)]
mod tests {
    use super::*;

    fn build_fluid_heat_world(warm_top: bool) -> World<f64> {
        let mut w = World::<f64>::new();
        // 流体:小盒、重力开启。
        let mut fp = SphParams::defaults();
        fp.bounds_min = Vec3::new(-3.0, -3.0, -3.0);
        fp.bounds_max = Vec3::new(3.0, 3.0, 3.0);
        fp.gravity = Vec3::new(0.0, -9.81, 0.0);
        let mut fluid = FluidWorld::new(fp);
        fluid.fill_box(
            Vec3::new(-1.0, -1.0, -1.0),
            Vec3::new(1.0, 1.0, 1.0),
            0.3,
            0.05,
        );
        let mut fsub = FluidSubsystem::new(fluid);
        fsub.thermal_expansion = 0.5; // 开启热浮力
        fsub.heat_gain = 0.1; // 开启对流热源
        w.add_subsystem(Box::new(fsub));

        // 热场:下半冷、上半热(与流体盒对齐)。
        let nx = 16usize;
        let dx = 6.0 / (nx as f64 - 1.0); // 覆盖 [-3,3]
        let mut f = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann)
            .with_origin(Vec3::new(-3.0, -3.0, -3.0));
        for iy in 0..nx {
            let y = -3.0 + (iy as f64) * dx;
            let t = if warm_top && y > 0.0 { 50.0 } else { 0.0 };
            for ix in 0..nx {
                for iz in 0..nx {
                    let idx = f.idx(ix, iy, iz);
                    f.u[idx] = t;
                }
            }
        }
        let heat = HeatField::new(f, 0.1);
        w.add_subsystem(Box::new(heat));
        w
    }

    #[test]
    fn world_couples_fluid_heat_thermal_buoyancy() {
        // 上热下冷 → 上方流体粒子获得向上浮力修正(竖直速度向上)。
        // 注意 World::step 顺序为 step→couple,浮力经 body_acc 在下一帧 integrate 才生效,
        // 故需多步(step 几帧)后浮力才会反映到粒子速度上。
        let mut w = build_fluid_heat_world(true);
        let dt = 1.0 / 60.0;
        for _ in 0..4 {
            w.step(dt);
        }
        // 动态查找流体子系统索引(不依赖注册顺序)。
        let mut fidx = None;
        for i in 0..w.subsystem_count() {
            if let Some(s) = w.get(i) {
                if s.as_any().downcast_ref::<FluidSubsystem<f64>>().is_some() {
                    fidx = Some(i);
                    break;
                }
            }
        }
        let fidx = fidx.expect("流体子系统应已注册");
        let fsub = w
            .get(fidx)
            .unwrap()
            .as_any()
            .downcast_ref::<FluidSubsystem<f64>>()
            .unwrap();
        // 取一个位于 y>0(热区)的粒子,验证数帧后其竖直速度向上
        // (热浮力经 body_acc 在 integrate 中叠加,使暖粒子上举)。
        let mut found = false;
        for p in &fsub.world.particles {
            if p.pos.y > 0.0 {
                // 暖粒子应获向上浮力 → 竖直速度为正(向上)。
                assert!(
                    p.vel.y > 0.0,
                    "热区粒子应获向上热浮力修正(vel.y > 0), vel.y={}",
                    p.vel.y
                );
                found = true;
                break;
            }
        }
        assert!(found, "应存在位于热区的流体粒子");
    }

    #[test]
    fn world_couples_optic_rigid() {
        // 光学↔刚体耦合(M16):步进后,标记为 source_rigid_idx 的光学体应与对应刚体位置一致,
        // 且刚体因重力下落时光学体随之移动。
        let mut scene = Scene::new();
        let dt = 1.0 / 60.0;
        // 找到光学子系统索引。
        let mut oidx = None;
        for i in 0..scene.world.subsystem_count() {
            if let Some(s) = scene.world.get(i) {
                if s.as_any().downcast_ref::<OpticSubsystem<f64>>().is_some() {
                    oidx = Some(i);
                    break;
                }
            }
        }
        let oidx = oidx.expect("光学子系统应已注册");
        // 取第 2 个光学体(对应刚体索引 1 的第一个掉落小球)初始位置。
        let before = {
            let s = scene
                .world
                .get(oidx)
                .unwrap()
                .as_any()
                .downcast_ref::<OpticSubsystem<f64>>()
                .unwrap();
            s.scene.bodies[1].body.pos
        };
        for _ in 0..30 {
            scene.world.step(dt);
        }
        let after = {
            let s = scene
                .world
                .get(oidx)
                .unwrap()
                .as_any()
                .downcast_ref::<OpticSubsystem<f64>>()
                .unwrap();
            s.scene.bodies[1].body.pos
        };
        // 光学体随刚体下落(y 减小)。注意:演示场景中央有一个引力井,会把小球向 x=0 横向
        // 吸引,故 x 也可能变化 —— 这里只验证竖直下落趋势,以及"光学体位置 == 刚体位置"。
        assert!(after.y < before.y, "掉落小球光学体应随刚体下移: {} -> {}", before.y, after.y);
        // 光学体位置必须与对应刚体位置完全一致(同一步 couple 同步)。
        let rigid_pos = {
            let mut ridx = None;
            for i in 0..scene.world.subsystem_count() {
                if let Some(s) = scene.world.get(i) {
                    if s.as_any().downcast_ref::<RigidSubsystem<f64>>().is_some() {
                        ridx = Some(i);
                        break;
                    }
                }
            }
            let rs = scene
                .world
                .get(ridx.unwrap())
                .unwrap()
                .as_any()
                .downcast_ref::<RigidSubsystem<f64>>()
                .unwrap();
            rs.world.bodies[1].pos
        };
        assert!(
            (after.x - rigid_pos.x).abs() < 1e-12
                && (after.y - rigid_pos.y).abs() < 1e-12
                && (after.z - rigid_pos.z).abs() < 1e-12,
            "光学体位置必须与刚体完全一致"
        );
    }

    #[test]
    fn world_couples_fluid_heat_source_injection() {
        // 流体运动 → 热场运动区域升温(对流换热)。均匀场初始 0,注入后中心升温。
        let mut w = build_fluid_heat_world(false);
        // 动态查找热场索引。
        let mut hidx = None;
        for i in 0..w.subsystem_count() {
            if let Some(s) = w.get(i) {
                if s.as_any().downcast_ref::<HeatField<f64>>().is_some() {
                    hidx = Some(i);
                    break;
                }
            }
        }
        let hidx = hidx.expect("热场子系统应已注册");
        // 给流体粒子一个初始速度,确保有运动强度可注入热源。
        let mut fidx2 = None;
        for i in 0..w.subsystem_count() {
            if let Some(s) = w.get(i) {
                if s.as_any().downcast_ref::<FluidSubsystem<f64>>().is_some() {
                    fidx2 = Some(i);
                    break;
                }
            }
        }
        if let Some(fi) = fidx2 {
            if let Some(fs) = w.get_mut(fi) {
                if let Some(fs) = fs.as_any_mut().downcast_mut::<FluidSubsystem<f64>>() {
                    for p in fs.world.particles.iter_mut() {
                        p.vel = Vec3::new(1.5, 0.0, 0.0);
                    }
                }
            }
        }
        let center = {
            let heat = w
                .get(hidx)
                .unwrap()
                .as_any()
                .downcast_ref::<HeatField<f64>>()
                .unwrap();
            heat.field.sample(nx_center(), nx_center(), nx_center())
        };
        for _ in 0..20 {
            w.step(1.0 / 60.0);
        }
        let heat = w
            .get(hidx)
            .unwrap()
            .as_any()
            .downcast_ref::<HeatField<f64>>()
            .unwrap();
        let after = heat.field.sample(nx_center(), nx_center(), nx_center());
        assert!(
            after > center,
            "运动区热场中心应因注入升温, before={}, after={}",
            center,
            after
        );
        // 全部有限。
        assert!(heat.field.max_abs().is_finite());
    }

    fn nx_center() -> usize {
        8
    }

    /// M11 集成:刚体↔热场 + 软体↔热场双向耦合在统一 World 中生效(无 NaN 且热浮力驱动)。
    #[test]
    fn world_couples_rigid_soft_heat() {
        let mut w = World::<f64>::new();

        // 热场:中心高温 T=100,其余 0(3x3x3 网格覆盖 [0,3]^3)。
        let nx = 3usize;
        let dx = 1.0;
        let mut f = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann);
        let hot = f.idx(1, 1, 1);
        f.u[hot] = 100.0;
        let mut heat_sub = HeatField::new(f, 0.1);
        // 让热场在 step 里做扩散,thermally meaningful。
        w.add_subsystem(Box::new(heat_sub));

        // 刚体:球放在热场中心 (1,1,1)。
        let mut rworld = RigidWorld::new();
        rworld.gravity = Vec3::new(0.0, -9.81, 0.0);
        rworld.add_body(Body::new(
            Shape::Sphere { r: 0.2 },
            Vec3::new(1.0, 1.0, 1.0),
            1.0,
        ));
        let mut rsub = RigidSubsystem::new(rworld);
        rsub.thermal_expansion = 0.5;
        rsub.heat_gain = 0.1;
        w.add_subsystem(Box::new(rsub));

        // 软体:单个质点在热场中心。
        let mut sbody = SoftBody::new(0.0);
        sbody.gravity = Vec3::new(0.0, -9.81, 0.0);
        sbody.particles.push(phy_soft::Particle {
            pos: Vec3::new(1.0, 1.0, 1.0),
            vel: Vec3::zeros(),
            force: Vec3::zeros(),
            inv_mass: 1.0,
        });
        let mut ssub = SoftSubsystem::new(sbody);
        ssub.thermal_expansion = 0.5;
        ssub.heat_gain = 0.1;
        w.add_subsystem(Box::new(ssub));

        // step 几帧,热浮力应在 couple 阶段把竖直速度上举(抵消重力)。
        for _ in 0..10 {
            w.step(1.0 / 60.0);
        }

        // 1) 刚体竖直速度应被热浮力显著上举(> 纯重力下落值)。
        let mut ridx = None;
        for i in 0..w.subsystem_count() {
            if let Some(s) = w.get(i) {
                if s.as_any().downcast_ref::<RigidSubsystem<f64>>().is_some() {
                    ridx = Some(i);
                    break;
                }
            }
        }
        let ridx = ridx.unwrap();
        let rsub = w
            .get(ridx)
            .unwrap()
            .as_any()
            .downcast_ref::<RigidSubsystem<f64>>()
            .unwrap();
        assert!(
            rsub.world.bodies[0].vel.y > -9.81 * (10.0 / 60.0),
            "热浮力应上举刚体, vy={}",
            rsub.world.bodies[0].vel.y
        );
        assert!(rsub.world.bodies[0].vel.y.is_finite());

        // 2) 软体质点同样被上举且有限。
        let mut sidx = None;
        for i in 0..w.subsystem_count() {
            if let Some(s) = w.get(i) {
                if s.as_any().downcast_ref::<SoftSubsystem<f64>>().is_some() {
                    sidx = Some(i);
                    break;
                }
            }
        }
        let sidx = sidx.unwrap();
        let ssub = w
            .get(sidx)
            .unwrap()
            .as_any()
            .downcast_ref::<SoftSubsystem<f64>>()
            .unwrap();
        assert!(ssub.body.particles[0].vel.y.is_finite());
        assert!(
            ssub.body.particles[0].vel.y > -9.81 * (10.0 / 60.0),
            "热浮力应上举软体质点, vy={}",
            ssub.body.particles[0].vel.y
        );
    }

    /// M12 集成:刚体↔电磁场双向耦合在统一 World 中生效。
    ///
    /// 构造带电刚体 + 电磁场(含外加 B),step 多帧后:电场/磁场洛伦兹力使带电体速度
    /// 被改变(且全场有限);运动带电体把电荷沉积进网格(rho.src 非零)。
    #[test]
    fn world_couples_rigid_em() {
        let mut w = World::<f64>::new();

        // 电磁场:覆盖 [-3,3]³、dx=0.5,外加 Z 向 B。
        let nx = 13usize;
        let dx = 6.0 / (nx as f64 - 1.0);
        let erho = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann)
            .with_origin(Vec3::new(-3.0, -3.0, -3.0));
        let mut em = EmField::<f64>::build(erho, 1.0);
        em.b_ext = Vec3::new(0.0, 0.0, 2.0);
        w.add_subsystem(Box::new(em));

        // 刚体:带电球,初速度 +X,在 X-Z 平面运动,受 v×B=(X×Z)=-Y 偏转 + 电场(零,纯磁)。
        let mut rworld = RigidWorld::new();
        rworld.gravity = Vec3::new(0.0, -9.81, 0.0);
        let mut em_ball = Body::new(Shape::Sphere { r: 0.2 }, Vec3::new(0.0, 0.0, 0.0), 1.0);
        em_ball.vel = Vec3::new(2.0, 0.0, 0.0); // 初速 +X
        rworld.add_charged_body(em_ball, 1.0);
        let mut rsub = RigidSubsystem::new(rworld);
        rsub.em_coupling = 1.0;
        w.add_subsystem(Box::new(rsub));

        // 记录初速,step 后应有偏转(纯重力只改 vy,vx 不受重力影响)。
        let vx0 = 2.0;
        for _ in 0..30 {
            w.step(1.0 / 60.0);
        }

        let ridx = (0..w.subsystem_count())
            .find(|&i| w.get(i).unwrap().as_any().downcast_ref::<RigidSubsystem<f64>>().is_some())
            .unwrap();
        let rsub = w
            .get(ridx)
            .unwrap()
            .as_any()
            .downcast_ref::<RigidSubsystem<f64>>()
            .unwrap();
        let b = &rsub.world.bodies[0];
        assert!(b.vel.x.is_finite() && b.vel.y.is_finite() && b.vel.z.is_finite());
        // v×B 产生 -Y 偏转:vy 应明显偏离纯重力值(纯重力 30 帧 ≈ -9.81*0.5=-4.9;
        // 叠加磁偏转后 vy 应更负,且 vz 也因耦合产生非零分量)。
        assert!(b.vel.y < -9.81 * (30.0 / 60.0), "v×B 应额外下压 vy, vy={}", b.vel.y);
        assert!(b.vel.x < vx0, "磁场应使 vx 衰减(能量转入 z), vx={}", b.vel.x);

        // 运动带电体把电荷沉积进 rho.src(双向耦合:电荷→电场)。
        let eidx = (0..w.subsystem_count())
            .find(|&i| w.get(i).unwrap().as_any().downcast_ref::<EmField<f64>>().is_some())
            .unwrap();
        let em = w
            .get(eidx)
            .unwrap()
            .as_any()
            .downcast_ref::<EmField<f64>>()
            .unwrap();
        let deposited: f64 = em.rho.src.iter().sum();
        assert!(deposited > 0.0, "运动带电体应把电荷沉积进电场网格");
    }

    /// M13 集成:刚体↔引力场双向耦合在统一 World 中生效。
    ///
    /// 构造一个静态大质量天体(质量源注入引力网格)+ 一个上方刚体,step 多帧后:
    /// 天体形成的局部引力井应把刚体向下(朝向天体)加速;运动刚体同时把质量沉积进网格。
    #[test]
    fn world_couples_rigid_grav() {
        let mut w = World::<f64>::new();

        // 引力场:覆盖 [-8,8]³、dx=0.5,中心下方放静态天体质量源。
        let nx = 32usize;
        let dx = 16.0 / (nx as f64 - 1.0);
        let grho = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann)
            .with_origin(Vec3::new(-8.0, -8.0, -8.0));
        let mut grav = GravField::<f64>::build(grho, 1.0);
        // 天体放在低处 (world y=-4 → 格 8),形成向下吸引力。
        let gcx = 16usize;
        let gcy = 8usize;
        let gcz = 16usize;
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dxk in -1..=1 {
                    grav.rho.add_source(
                        (gcx as isize + dxk) as usize,
                        (gcy as isize + dy) as usize,
                        (gcz as isize + dz) as usize,
                        50.0,
                    );
                }
            }
        }
        w.add_subsystem(Box::new(grav));

        // 刚体:放在天体正上方 (world y=+4),初速为零,应被引力井向下加速。
        let mut rworld = RigidWorld::new();
        rworld.gravity = Vec3::new(0.0, 0.0, 0.0); // 关掉均匀重力,专测局部引力井
        rworld.add_body(Body::new(
            Shape::Sphere { r: 0.2 },
            Vec3::new(0.0, 4.0, 0.0), // 天体上方
            1.0,
        ));
        let mut rsub = RigidSubsystem::new(rworld);
        rsub.grav_coupling = 1.0;
        w.add_subsystem(Box::new(rsub));

        // step 数帧后,刚体应被向下吸引(朝向天体,v.y 应变为负)。
        for _ in 0..20 {
            w.step(1.0 / 60.0);
        }

        let ridx = (0..w.subsystem_count())
            .find(|&i| w.get(i).unwrap().as_any().downcast_ref::<RigidSubsystem<f64>>().is_some())
            .unwrap();
        let rsub = w
            .get(ridx)
            .unwrap()
            .as_any()
            .downcast_ref::<RigidSubsystem<f64>>()
            .unwrap();
        let b = &rsub.world.bodies[0];
        assert!(b.vel.y.is_finite(), "引力耦合后速度应有限");
        assert!(b.vel.y < 0.0, "刚体应被下方天体引力井向下吸引, vy={}", b.vel.y);

        // 运动刚体把质量沉积进 rho.src(双向耦合:质量→引力井)。
        let gidx = (0..w.subsystem_count())
            .find(|&i| w.get(i).unwrap().as_any().downcast_ref::<GravField<f64>>().is_some())
            .unwrap();
        let grav = w
            .get(gidx)
            .unwrap()
            .as_any()
            .downcast_ref::<GravField<f64>>()
            .unwrap();
        let deposited: f64 = grav.rho.src.iter().sum();
        assert!(deposited > 0.0, "运动刚体应把质量沉积进引力场网格");
    }

    /// M26 演示扩展:四种单场模式场景能正常构造并步进(不 panic、数值有限)。
    #[test]
    fn demo_single_field_scenes_build_and_step() {
        let dt = 1.0 / 60.0;
        // 电磁场模式:电荷密度 + 电场矢量在步进后有限且非全零。
        {
            let mut s = Scene::em();
            s.world.step(dt);
            let sub = s
                .world
                .get(0)
                .unwrap()
                .as_any()
                .downcast_ref::<EmField<f64>>()
                .unwrap();
            let e_norm: f64 = sub.e.iter().map(|v| v.norm()).sum();
            assert!(sub.rho.max_abs().is_finite());
            assert!(e_norm.is_finite() && e_norm > 0.0, "电场矢量应已由电荷分布解出");
        }
        // 引力场模式:质量密度 + 引力矢量有限。
        {
            let mut s = Scene::grav();
            s.world.step(dt);
            let sub = s
                .world
                .get(0)
                .unwrap()
                .as_any()
                .downcast_ref::<GravField<f64>>()
                .unwrap();
            let g_norm: f64 = sub.g.iter().map(|v| v.norm()).sum();
            assert!(sub.rho.max_abs().is_finite());
            assert!(g_norm.is_finite() && g_norm > 0.0, "引力矢量应已由质量分布解出");
        }
        // 波动场模式:标量场有限。
        {
            let mut s = Scene::wave();
            for _ in 0..5 {
                s.world.step(dt);
            }
            let sub = s
                .world
                .get(0)
                .unwrap()
                .as_any()
                .downcast_ref::<WaveField<f64>>()
                .unwrap();
            assert!(sub.field.max_abs().is_finite());
        }
        // 声场模式:标量场有限。
        {
            let mut s = Scene::acoustic();
            for _ in 0..5 {
                s.world.step(dt);
            }
            let sub = s
                .world
                .get(0)
                .unwrap()
                .as_any()
                .downcast_ref::<AcousticField<f64>>()
                .unwrap();
            assert!(sub.field.max_abs().is_finite());
        }
    }


    /// M18 集成:刚体关节(Distance)经 `World` 驱动生效 —— 两体间距收敛到杆长。
    ///
    /// 构造一个含刚体子系统的 `World`,经 downcast 取出内部 `RigidWorld` 注入关节,
    /// step 多帧后两体间距应等于 `rest`,且总动量守恒(无外力)。
    #[test]
    fn world_drives_rigid_distance_joint() {
        let mut w = World::<f64>::new();
        let mut rworld = RigidWorld::new();
        rworld.gravity = Vec3::zeros();
        rworld.add_body(Body::new(
            Shape::Sphere { r: 0.2 },
            Vec3::new(-1.0, 0.0, 0.0), // 初始间距 2
            1.0,
        ));
        rworld.add_body(Body::new(
            Shape::Sphere { r: 0.2 },
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
        ));
        // 杆长 1.5:步进后应把间距从 2 拉回 1.5。
        rworld.add_joint(0, 1, Joint::Distance {
            pa: Vec3::zeros(),
            pb: Vec3::zeros(),
            rest: 1.5,
        });
        let mut rsub = RigidSubsystem::new(rworld);
        w.add_subsystem(Box::new(rsub));

        for _ in 0..300 {
            w.step(1.0 / 120.0);
        }

        let ridx = (0..w.subsystem_count())
            .find(|&i| w.get(i).unwrap().as_any().downcast_ref::<RigidSubsystem<f64>>().is_some())
            .unwrap();
        let rsub = w
            .get(ridx)
            .unwrap()
            .as_any()
            .downcast_ref::<RigidSubsystem<f64>>()
            .unwrap();
        let dist = (rsub.world.bodies[1].pos - rsub.world.bodies[0].pos).norm();
        assert!((dist - 1.5).abs() < 0.05, "关节杆长应收敛到 1.5, 实际 {}", dist);
    }

    /// M20 集成:射线投射车辆(raycast vehicle)经 `World` 驱动生效 —— 车身被悬挂托在
    /// 地面上方,引擎驱动使其前进。复用刚体子系统(`RigidSubsystem`),每帧先 `vehicle.update`
    /// 注入悬挂/轮胎力,再 `world.step`。
    #[test]
    fn world_drives_raycast_vehicle() {
        let mut w = World::<f64>::new();
        let mut rworld = RigidWorld::new();
        rworld.gravity = Vec3::new(0.0, -9.81, 0.0);
        // 静态大地面(1500 半宽,足够长不会驶出边缘)。
        rworld.add_body(Body {
            shape: Shape::Box {
                half: Vec3::new(1500.0, 0.5, 1500.0),
            },
            pos: Vec3::new(0.0, -0.5, 0.0),
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 0.0,
        
            ..Default::default()
        });
        // 车身(500 kg 盒),悬空在地面上方。
        let cid = rworld.add_body(Body::new(
            Shape::Box {
                half: Vec3::new(1.0, 0.25, 0.5),
            },
            Vec3::new(0.0, 1.0, 0.0),
            1.0 / 500.0,
        ));
        // 四轮:四角,悬挂自然长度 0.6、刚度 8000、阻尼 800、轮半径 0.3。
        let rest = 0.6;
        let k = 8000.0;
        let c = 800.0;
        let r = 0.3;
        let wheels = vec![
            phy_rigid::Wheel::new(Vec3::new(-0.9, -0.25, 0.4), rest, k, c, r, 1.0, 1.0),
            phy_rigid::Wheel::new(Vec3::new(0.9, -0.25, 0.4), rest, k, c, r, 1.0, 1.0),
            phy_rigid::Wheel::new(Vec3::new(-0.9, -0.25, -0.4), rest, k, c, r, 1.0, 1.0),
            phy_rigid::Wheel::new(Vec3::new(0.9, -0.25, -0.4), rest, k, c, r, 1.0, 1.0),
        ];
        let mut veh = phy_rigid::Vehicle::new(cid, wheels);
        veh.set_engine(3000.0);

        let mut rsub = RigidSubsystem::new(rworld);
        w.add_subsystem(Box::new(rsub));

        let dt = 1.0 / 120.0;
        let ridx = (0..w.subsystem_count())
            .find(|&i| w.get(i).unwrap().as_any().downcast_ref::<RigidSubsystem<f64>>().is_some())
            .unwrap();

        // 先让车辆着地稳定。
        for _ in 0..200 {
            {
                let rs = w.get_mut(ridx).unwrap().as_any_mut().downcast_mut::<RigidSubsystem<f64>>().unwrap();
                veh.update(&mut rs.world, dt);
            }
            w.step(dt);
        }
        let y_grounded = {
            let rs = w.get(ridx).unwrap().as_any().downcast_ref::<RigidSubsystem<f64>>().unwrap();
            rs.world.bodies[cid].pos.y
        };
        assert!(y_grounded > 0.3 && y_grounded < 1.5, "车身应被悬挂托在地面上方, y={}", y_grounded);

        let x0 = {
            let rs = w.get(ridx).unwrap().as_any().downcast_ref::<RigidSubsystem<f64>>().unwrap();
            rs.world.bodies[cid].pos.x
        };
        // 引擎驱动前进。
        for _ in 0..200 {
            {
                let rs = w.get_mut(ridx).unwrap().as_any_mut().downcast_mut::<RigidSubsystem<f64>>().unwrap();
                veh.update(&mut rs.world, dt);
            }
            w.step(dt);
        }
        let x1 = {
            let rs = w.get(ridx).unwrap().as_any().downcast_ref::<RigidSubsystem<f64>>().unwrap();
            rs.world.bodies[cid].pos.x
        };
        assert!(x1 > x0, "引擎应驱动车身前进, dx={}", x1 - x0);
    }

    /// M21 / #8 Voronoi 破碎端到端:把一个盒刚体碎成多个凸碎片,碎片继承线速度并
    /// 在重力下自由下落(经 `World` + `RigidSubsystem` 驱动生效)。验证 `shatter` 与
    /// 子系统调度闭环。
    #[test]
    fn world_shatters_box_into_fragments() {
        let mut w = World::<f64>::new();
        let mut rworld = RigidWorld::new();
        rworld.gravity = Vec3::new(0.0, -9.81, 0.0);
        // 母本盒(8 kg),静止悬在空中。
        let pid = rworld.add_body(Body::new(
            Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            Vec3::new(0.0, 5.0, 0.0),
            1.0 / 8.0,
        ));
        let rsub = RigidSubsystem::new(rworld);
        w.add_subsystem(Box::new(rsub));

        let ridx = (0..w.subsystem_count())
            .find(|&i| w.get(i).unwrap().as_any().downcast_ref::<RigidSubsystem<f64>>().is_some())
            .unwrap();

        // 碎裂:6 块,径向飞散 1.5。
        let frag_ids = {
            let rs = w.get_mut(ridx).unwrap().as_any_mut().downcast_mut::<RigidSubsystem<f64>>().unwrap();
            rs.world.shatter(pid, 6, 1.5)
        };
        assert!(frag_ids.len() >= 4, "应碎出至少 4 块,得 {}", frag_ids.len());
        // 母本已从世界移除(原 pid 处不再是同一个盒)。
        let n_before = {
            let rs = w.get(ridx).unwrap().as_any().downcast_ref::<RigidSubsystem<f64>>().unwrap();
            rs.world.bodies.len()
        };
        assert!(n_before >= frag_ids.len(), "世界应包含碎片");

        // 步进 60 帧,碎片应下落且无 NaN。
        for _ in 0..60 {
            w.step(1.0 / 120.0);
        }
        let all_finite = {
            let rs = w.get(ridx).unwrap().as_any().downcast_ref::<RigidSubsystem<f64>>().unwrap();
            frag_ids.iter().all(|&id| {
                rs.world.bodies[id].pos.x.is_finite()
                    && rs.world.bodies[id].pos.y.is_finite()
                    && rs.world.bodies[id].pos.z.is_finite()
            })
        };
        assert!(all_finite, "碎片位置不应出现 NaN");
    }
}
