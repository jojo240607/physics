# phy-game-demo — Physics Sandbox Parkour

基于本仓库物理引擎 SDK（`phy-sdk` / `phy-rigid` / `phy-core`）的 **3D 小游戏 demo**。
纯软件渲染（复用 `phy-demo` 的极简 Z-buffer 光栅化器），无 GPU 依赖，可在一个独立 crate 里完整运行。

## 玩法

第三人称角色（胶囊）在带围墙的平台关卡上跑跳，平台上有可推的箱子与可滚动的球。

| 按键 | 行为 |
|------|------|
| `W` `A` `S` `D` | 相对相机方向的水平移动 |
| `Space` | 跳跃 |
| 鼠标左键拖拽 | 旋转相机视角 |
| `Q` / `E` | 拉远 / 拉近相机 |
| `R` | 重置角色到出生点 |
| `Esc` | 退出 |

窗口标题实时显示角色坐标与 `grounded` 状态。

## 架构

- `src/lib.rs` — `Game` 游戏状态与物理推进逻辑（`step_with_input` / `step` / `render` / `all_finite`），**可 headless 单元测试**。
- `src/main.rs` — winit 窗口事件循环 + softbuffer 帧缓冲展示，薄渲染层。
- `src/meshgen.rs` — 物理 shape（Box / Sphere / Capsule）→ 光栅化三角网格生成。
- `tests/headless.rs` — 5 个无窗口冒烟测试（落体落地 / 水平移动 / 跳跃往返 / 长时间有限性）。

## 运行

```bash
cargo run -p phy-game-demo
```

Headless 测试（CI 无显示环境可跑）：

```bash
cargo test -p phy-game-demo --test headless
```

## 接入方式

本 crate 作为物理引擎 workspace 的一个 member，通过 path 依赖直接引用：

```toml
phy-sdk   = { path = "../phy-sdk" }
phy-rigid = { path = "../phy-rigid" }
phy-core  = { path = "../phy-core" }
phy-demo  = { path = "../phy-demo" }   # 复用 Camera + 软件光栅化器
```

World 构建：

```rust
let mut world: World<f64> = PhysicsBuilder::new().rigid().build();
let rigid = get_as_mut::<RigidSubsystem<f64>>(&mut world, 0).unwrap();
// ... add_body(静态地面 / 动态箱子 / 球) ...
let mut cc = CharacterController::new(&mut rigid.world, spawn);
// 每帧:cc.update(&mut rigid.world, dt, dir, jump); rigid.world.step(dt);
```

## 附带修复

本 demo 暴露并修复了 `phy-rigid::CharacterController` 的一个真实 bug：角色**静止在地面上时 `grounded` 标志会丢失**（因 vel_y 归零后本帧不再下穿，`slide` 无法确认接触）。修复：着地态额外施加微小向下探测位移（`probe = 0.02`）让 `slide` 持续确认接触，走下台阶时探测不再穿透即自然开始下落。修复后 `character_falls_to_ground_and_grounded` 等单测仍全绿。
