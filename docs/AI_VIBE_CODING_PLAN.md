# AI Vibe Coding 适配计划

> 策略变更：**本物理引擎不再定位为「给人类软件开发者的中间件」，而是「给 AI 做 vibe coding 时调用的中间库」**。
>
> 这意味着 API 的**第一消费者是 LLM**，而不是人类工程师。设计优先级随之改变：
> - 人类能容忍"索引 0 = 第一个声明的子系统"、"向下转型取句柄"、"不透明指针"；
> - AI 会乱猜函数名、搞错坐标系/单位、传错数组布局、忽略错误返回值、复制 `expect` 导致 panic。
>
> 目标：**让 AI 在一次或两次对话内就写出可编译、物理正确、不崩溃的仿真代码**，并把"AI 最容易踩的坑"前置到代码与示例里。

---

## 设计原则（贯穿所有改动）

1. **意图式 API 优先于机制式 API**：AI 说"造 100 个下落球"，应有 `spawn_spheres(count, ...)` 而非先 `add_body` + 手写 `Body::new(Shape::Sphere{..}, ..)`。
2. **具名取回优先于索引取回**：`world.rigid()` 返回 `Option<&RigidSubsystem>`，而非 `get_as::<RigidSubsystem>(&world, 0)`。
3. **fail-soft 优先于 panic**：所有入口返回 `Result`/`Option`/哨兵值；任何 FFI 函数绝不 unwind 到调用方。
4. **坐标系/单位/返回值语义写在代码 doc 里**（AI 检索得到），而非只在独立 md。
5. **任务式示例库（recipes）**：每个文件 = 一个 AI 可整段复制改参的完整 demo，比 API 文档更可靠。
6. **多语言可达**：AI 常生成 Python/C++/C# 而非 Rust，需提供 Python 包装层 + 补全 FFI 工厂。

---

## 改动清单（按优先级）

### P0 — 降低 AI 90% 编译失败率

- [x] **P0-1 `phy-sdk`：具名子系统取回**
  新增 `WorldExt::fluid()/rigid()/granular()/soft()/field()/optics()/solid() -> Option<&Subsystem>` 与 `mut` 变体，内部按类型 downcast，AI 不用记索引顺序。保留 `get_as`/`get_as_mut` 作为底层逃生口。
- [x] **P0-2 `phy-sdk`：意图式 spawn API**
  在 `WorldExt` 上新增高层方法（不要求先取回子系统句柄）：
  - `spawn_floor(half: Vec3, y: f64)` —— 静态地面（最常见需求）。
  - `spawn_sphere(pos, r, mass) -> BodyId`、`spawn_box(pos, half, mass) -> BodyId`、`spawn_static_sphere/box`。
  - `spawn_sphere_grid(pos, count, spacing, r, mass) -> Vec<BodyId>`。
  返回 `BodyId`（新类型包装 `usize`），AI 不用自己存索引。
  （流体/颗粒的批量灌装走既有 `fluid().fill_box` / `granular().fill_grid`，意图式薄封装暂未做，因底层 `FluidWorld`/`GranularWorld` 已足够直白。）
- [x] **P0-3 `phy-sdk`：坐标/单位 doc 契约**
  在 `phy-sdk` crate 顶层 `#![doc]` 与 `WorldExt` trait doc 写死坐标/单位契约（右手系, +Y 向上, 米; 四元数 w,x,y,z 机体->世界; 重力 -Y 9.81）。`RigidWorld` 等底层契约可由 AI 经 `use phy_sdk::WorldExt` 顶层说明检索到。

### P1 — 任务式示例库（recipes）

- [x] **P1-1 `crates/phy-sdk/examples/recipes_*` 可编译可运行 demo**（每个文件可整段复制），均为 P0 的 `WorldExt` API 驱动：
  - `recipes_drop_bouncing_balls.rs`（刚体 + 地面 + 125 下落球网格 + 读回位姿）✅ 运行通过
  - `recipes_water_dam_break.rs`（SPH 溃坝）✅ 运行通过
  - `recipes_sand_pile.rs`（颗粒堆积）✅ 运行通过
  - `recipes_cloth_flag.rs`（软体 PBD 旗帜）✅ 运行通过
  - `recipes_heat_plate.rs`（标量热场扩散）✅ 运行通过
  每个文件头部有 `//! 用途: ...` + `//! 运行: cargo run -p phy-sdk --example recipes_xxx` + 坐标契约。
  待补：`optics_mirror.rs`（光学/波动）、`rigid_drone.rs`（四旋翼）留作 P1 收尾，需先确认 OpticScene/四旋翼高层 API 形态。
- [x] **P1-2 `phy-sdk` doctest 升级为"复制即可用"片段**，已含：world_basic（含 AI 推荐用法：具名取回 + 意图式 spawn + 坐标契约）、char_pickup（CharacterController 用法）、save_load（序列化往返）。均通过 `cargo test --doc`。

### P2 — Python 包装层（AI 最常生成的语言）

- [ ] **P2-1/P2-2/P2-3 Python 封装层 —— 已决议跳过**：本仓库是纯 Rust SDK，AI vibe-coding 目标语言即 Rust，`phy-sdk` 的 `WorldExt` + recipes 已是"给 LLM 的友好 API 直通车"，无跨语言 FFI 需求。Python ctypes 封装仅在"生成 Python/C++/C#"场景才有意义，故整段 P2 不执行（计划第 6 条设计原则的前提不成立）。

### P3 — FFI 补全（让非 Rust 语言用全套物理）

- [x] **P3-1 `phy-ffi` 补齐空世界工厂**：新增 `phy_world_create_soft_empty` / `phy_world_create_field_empty` / `phy_world_create_optics_empty` / `phy_world_create_solid_empty`（与 `phy-sdk::PhysicsBuilder` 能力对齐，空世界供调用方自行填充）。新增符号属向后兼容，ABI 版本保持 `2` 不变。已加 doctest 覆盖 4 工厂 + destroy，全过。
- [ ] **P3-2 `phy-ffi` 补齐读回**：软体顶点、标量场网格、光学场、实体网格的 `get_*` 函数（数组布局写进头注释）。依赖 P3-1 工厂 + 调用方填充路径，留作下一步。
- [ ] **P3-3 重新生成 `phy_ffi.h`**（cbindgen）并核对 ABI 版本：P3-1 仅新增符号，ABI 版本不变；待 P3-2 落地后一并重生成头文件。

### P4 — 稳定性承诺对 AI 友好

- [x] **P4-1 `phy-sdk` 顶层 `#![doc]` 加显式稳定性警告**：`0.x 阶段 API 可能变动，业务锁定精确版本 = "0.1.0"`（已写入 crate 根 doc）。
- [ ] **P4-2 所有 `#[non_exhaustive]` 已在 wfix 分支标注（完成）**；补充：AI 生成 `match` 漏通配由编译器抓，不构成问题。

---

## 执行顺序

1. P0-1 + P0-2 + P0-3（核心 API 改造，投入小、收益最大）
2. P1-1 + P1-2（示例库）
3. P2-1 + P2-2 + P2-3（Python 层）
4. P3-1 + P3-2 + P3-3（FFI 补全）
5. P4-1（稳定性 doc）

每步完成后跑 `cargo build -p phy-sdk` / `cargo test -p phy-sdk` / `cargo build --examples` 验证，确保不破坏现有 API（向后兼容：旧 `get_as`/`PhysicsBuilder` 保留）。

---

## 验收标准

- AI 用"造 100 个下落球并读回位姿"prompt，基于 recipes + 新 API 能在 1 次生成内编译通过。
- `cargo test -p phy-sdk` 全绿（含新 doctest）。
- `python/phy.py` 示例可跑通（需先编译 cdylib）。
- FFI 头文件含全部子系统的工厂与读回，ABI 版本记录一致。
