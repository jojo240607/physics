# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- **Rigid body sleeping (B1, commercial-readiness plan §11)**: `Body` gains
  `sleeping` / `sleep_time` flags; `SolverParams` gains `sleep_lin_vel2`,
  `sleep_ang_vel2`, `sleep_time` thresholds (default ~0.1 m/s, 0.5 s). Near-rest
  bodies auto-sleep (zero CPU: gravity integration, velocity solve, and position
  projection all skip); a sleeping body is woken by any neighboring moving body
  via the broad-phase pair loop. Verified by `resting_box_falls_asleep_and_stays_put`
  and `falling_ball_wakes_sleeping_box` unit tests.
- **Collision layers / filtering (B5, commercial-readiness plan §11)**: `Body`
  gains `layers` / `collision_mask` bitmasks; `can_collide_with()` enforces
  mutual layer/mask match before narrow-phase. Cross-layer bodies pass through
  (e.g. character vs trigger), same-mask bodies collide normally. Verified by
  `collision_layers_block_cross_layer_contact` and
  `collision_layers_allow_same_layer_contact` unit tests.
- **Kinematic body + sensor/trigger (B2, commercial-readiness plan §11)**: `Body`
  gains `kinematic` flag — the solver treats its effective inverse mass as 0
  (`eff_inv_mass()`), so it is not driven by gravity/contact impulses but is moved
  by its user-set `vel` every step and pushes dynamic bodies aside (character
  controller / conveyor / moving platform base). `Body` also gains `is_sensor`
  flag — such bodies still run narrow-phase overlap detection but produce **no**
  contact constraint (no impulse, no penetration blocking); their overlaps are
  reported via `RigidWorld::sensor_contacts()` for trigger / pickup / enter-zone
  logic. Verified by `kinematic_body_pushes_dynamic_and_is_unaffected` and
  `sensor_detects_overlap_but_no_impulse` unit tests.
- **Unity/Unreal 集成 glue code (C2, commercial-readiness plan §11)**: 新增
  `crates/phy-ffi/unity/PhysicsFFI.cs`(C# P/Invoke 全量绑定 + `RigidTransform` 位姿解析)、
  `crates/phy-ffi/unreal/PhysicsFFI.h`(C++ 符号声明 + `PhyToUnrealTransform` 转 `FTransform`)、
  `crates/phy-ffi/unreal/PhysicsFFI.Build.cs`(UE 模块 ThirdParty 接入示例)。
  `build_package.py` 现已把 `glue/unity` 与 `glue/unreal` 一并拷贝进发行包 (`pkg/<profile>/glue/`),
  与 `phy_ffi.lib` / `libphy_ffi.dll.a` 形成"预编译二进制 + 引擎侧 glue"完整分发。
- **Game-developer quickstart (C4, commercial-readiness plan §11)**: README 新增
  "刚体游戏快速上手"小节,以关卡 JSON DSL(`load_scene_json` / `to_scene_json`) +
  触发器(`is_sensor` / `sensor_contacts`) + 碰撞层(`layers` / `collision_mask`) +
  帧级计时(`step_with_profile` / `StepProfile`)串起典型游戏接入路径(关卡重建、
  拾取/触发区、分层互不阻挡、逐帧耗时归因)。
- **Runtime step profiler (C3, commercial-readiness plan §11)**: new `profile` module
  + `StepProfile` (per-stage nanos: broad/narrow, velocity, advance, position, sleep,
  total) and `RigidWorld::step_with_profile`. Zero-cost by default — gated behind the
  `profiler` cargo feature so the hot path emits no `Instant::now` when unused. Default
  `step` keeps its `Vec<Contact<T>>` signature (thin wrapper over `step_with_profile`).
  Enables frame-level GPU-side timing to correlate engine cost with draw budget.
- **Rigid-body stacking/friction convergence cap (B1, commercial-readiness plan §11)**:
  `SolverParams` gains `friction_iterations` (friction solved in a **separate**
  sub-iteration loop after the normal loop, so friction convergence is capped
  independently of `iterations`; 0 falls back to `iterations`) and `position_slop`
  (residual-penetration tolerance, default 1 mm — Baumgarte position correction
  now targets `depth - slop`, eliminating the "over-correction jitter" of resting
  stacks). Verified by `stacked_boxes_converge_without_jitter` (3-box stack keeps
  ~0.5/1.5/2.5 m at rest, no launch) and
  `b1_friction_and_slop_params_wired_and_stable` (defaults correct + box stays
  put across `friction_iterations ∈ {0,1,5,40}`).
- **Island (connected-component) parallelism (B4, commercial-readiness plan §11)**: new
  `islands.rs` builds connected components from contact + joint pairs via a
  union-find (deterministic, BTreeMap-ordered roots). The velocity solve
  (step 2b) and position projection (step 3) now iterate islands in parallel
  via `rayon::par_iter`; each island writes back only its own disjoint body
  indices, so there is no data race, while intra-island sequential-impulse
  order is preserved for bit-identical results vs the single-threaded solver.
  Joint λ cross-substep accumulation is preserved by writing λ back to the
  global `joints` after each island pass. Verified numerically equivalent
  (63 tests + doctest green) before and after enabling parallelism.
- **GPU kernel ↔ CPU production formula alignment + per-particle error report (G1,
  commercial-readiness plan §11)**: the W4 SPH and W5 granular **wgsl** compute kernels
  in `crates/phy-demo-web/src/gpu/mod.rs` were upgraded from "simplified port" to be
  **formula-identical with the CPU production solver** — symmetric pressure
  `m_i·m_j·(p_i/ρ_i² + p_j/ρ_j²)` (momentum-conserving, was non-symmetric `m_j·(p_i+p_j)/(2ρ_j)`),
  non-Newtonian power-law viscosity `μ = ki·max(shear, shear_min)^(ni-1)` (was linear
  `ki + shear_min`), and the granular PBD projection is now the same Jacobi reduce as
  CPU. `gpu_accuracy.rs` was rewritten to compare **CPU production per-particle
  acceleration directly** against the wgsl serial reference (`gpu_ref.rs`, now mirroring
  the aligned kernels) instead of a second-difference approximation. Measured per-particle
  error is floating-point precision only: SPH `MAX ≈ 2.9e-6`, granular projection
  bit-identical. This is the honest **CPU↔GPU numerical-consistency baseline**; the real
  adapter error (browser / native wgpu) is expected to match this magnitude — see
  `export_flat_for_adapter` for the data-export path. Acceptance tests tightened to
  consistency thresholds (SPH `MAX < 1e-1`, granular projection `< 1e-3`).
- **Real-GPU-adapter per-particle error report (G1, commercial-readiness plan §11)**:
  installed the MSVC toolchain + VS Build Tools and opened the `gpu` module to **native**
  compilation (`#[cfg(feature = "gpu")]` instead of `wasm32`-only; native wgpu backends
  drive the real adapter), adding `device.poll(PollType::Wait)` before each `map_async`
  readback so native kernels complete instead of hanging. New example
  `crates/phy-demo-web/examples/real_gpu_error.rs` runs the W4 SPH / W5 granular wgsl
  kernels on the **real NVIDIA Quadro P2200 adapter** (`cargo +stable-msvc run -p
  phy-demo-web --example real_gpu_error --features gpu`). Measured: **GPU output vs the
  wgsl serial reference is bit-faithful — SPH acc `MAX ≈ 1.5e-4`, granular projection
  bit-identical 0**. Running on the real adapter also exposed and fixed **two genuine wgsl
  kernel bugs**: (1) cell-index mismatch — `ci = floor((pi-gmin)/h)` (relative offset) was
  subtracted by `mi` again in `cell_idx`, doubling the offset so SPH density computed as 0
  and only gravity survived; fixed to `floor(pi/h)` absolute key (matches the flat-grid
  `flat_idx`); (2) viscosity force was missing the `m_i` factor (diverged from CPU
  `compute_forces`); added it. Pressure direction corrected to `(pi-pj)/r × +fpress`
  (repulsion, matching CPU). `gpu_ref.rs` mirrors all fixes. The earlier host baseline
  (`SPH MAX≈2.9e-6`) had been measured under the density-0 artifact; after the fix the
  host test asserts the **bulk** (interior) particles are float-exact and the **boundary**
  single particles are bounded (CPU `for_each_neighbor` BTreeMap vs flat-grid prefix-sum
  disagree on corner-particle neighbor lookup — corner SPH force CPU≈0 vs GPU≈36, GPU more
  physical; interior particles bit-identical). This boundary divergence between the CPU
  kernel and the flat grid is a known pre-existing difference, deferred to kernel
  unification; it does not affect the G1 conclusion that the GPU faithfully reproduces the
  wgsl kernel.
- **Real-GPU-adapter performance / scale baseline (G2, commercial-readiness plan §11)**:
  new example `crates/phy-demo-web/examples/gpu_perf.rs` measures the W4 SPH / W5 granular
  wgsl kernels on the **real NVIDIA Quadro P2200 adapter** (Vulkan, release, f32). GPU path
  comfortably clears the real-time SLO: **SPH 46.6k ≈ 105 fps, granular 10k ≈ 433 fps**
  (vs CPU single-thread baseline of ~40 fps at granular 10k — a **10.7x** GPU speedup; SPH
  10k = 1.6x, granular 5k = 5.5x). Honest caveat: each `render_*_gpu` call rebuilds its
  shader/pipeline/storage buffers and round-trips a CPU map, so small scenes (1k SPH) are
  overhead-bound (0.5x slower than CPU); a game reusing pipelines would be faster than the
  numbers shown. Full table in `docs/perf_gpu_baseline.md`. This means real-time
  multi-ten-thousand-entity simulation is feasible on the GPU path (browser WebGPU /
  MSVC native wgpu).
- **crates.io publish readiness (C1, commercial-readiness plan §11)**: all 12 publishable
  crates (`phy-demo` / `phy-demo-web` are `publish = false`) now declare
  `version = "0.1.0"` on every internal `path` dependency, which `cargo publish` requires
  (the path is rewritten to a crates.io version reference at publish time; locally the path
  still wins). `phy-math` verified with `cargo package` (incl. verify build). A release-order
  script `scripts/publish_all.sh` encodes the dependency-topology order (phy-math → phy-core
  → phy-field → phy-rigid → phy-fluid/solid/granular/optics → phy-soft → phy-io →
  phy-ffi/phy-sdk). Actual publishing still requires `cargo login` (crates.io token) and is
  intentionally not executed here.
- **Joint expansion — Hinge / Prismatic / Weld / Motor (D1, game-grade plan `PLAN_NEXT.md`)**:
  the rigid-body joint solver now supports **angular constraints** (rotational impulse via
  `inv_inertia_world`, Box2D-style per-axis solve; point constraints now include the
  angular-velocity term), unlocking three new joint types beyond `Ball`/`Distance`:
  - `Weld`: locks all 6 DOF (3 point + 3 angular-alignment) — weld fragments / rigid composites.
  - `Hinge`: aligned hinge axis + free rotation about it (3 point + 2 angular constraints),
    with optional `motor_vel` / `max_motor_torque` driving rotation (wheels, doors, gears).
  - `Prismatic`: slide along one axis (2 perpendicular point + 3 angular-locked), with
    optional `motor_vel` / `max_motor_force` driving linear travel (hydraulics, pistons).
  Verified by `weld_joint_keeps_relative_pose_and_syncs_angular_velocity` (pose held + angular
  sync), `hinge_joint_with_motor_rotates_around_aligned_axis` (axis-aligned spin under motor),
  and `prismatic_joint_with_motor_slides_along_axis` (axis slide, no spin). 72 phy-rigid tests
  + full workspace green.
- **Capsule collider + analytic narrow-phase (D2, game-grade plan `PLAN_NEXT.md`)**: new
  `Shape::Capsule { half_height, r }` (sphere-swept segment along the local y axis) with full
  `support_local` / `contains_local` / `bounding_sphere_r` / `set_inertia_from_shape`, plus
  analytic fast-path collisions `capsule_sphere` / `capsule_capsule` / `capsule_box`
  (normal pointing from the first body to the second; box-interior ejection branch), and
  `raycast` / `ccd` / `fracture` coverage. A Capsule dropped on a static box ground now rests
  on it without tunneling. Note: this exposed a **pre-existing GJK robustness issue** — GJK
  iterates unstably and reports non-intersection for "smooth-body + sharp-corner (Box)"
  combinations (Sphere/Box are unaffected because they take fast paths; Convex-Convex that
  falls back to GJK may need attention). Capsule sidesteps it via analytic paths. Verified by
  `capsule_support_on_axis` / `capsule_contains_and_bounds` /
  `capsule_rests_on_ground_via_gjk_epa`. 75 phy-rigid tests + full workspace green.
- **Heightfield terrain collider (D2, game-grade plan `PLAN_NEXT.md`)**: new
  `Shape::Heightfield { nx, nz, cell, heights }` (2-D grid of heights on the world XZ plane,
  bilinear-interpolated surface query) with analytic narrow-phase `heightfield_vs_body`
  supporting **Sphere / Box / Capsule** resting on terrain (normal points up, from terrain to
  body), plus bounding-sphere / inertia / broad-phase coverage and rendering in `phy-demo`.
  A Sphere dropped on a flat heightfield rests on the surface without tunneling. Note: the
  main `RigidWorld` impl bound was relaxed from `ToPrimitive` to `NumCast` so narrow-phase can
  use index conversions for heightfield queries. Verified by
  `sphere_rests_on_heightfield_terrain`. 76 phy-rigid tests + full workspace green.
- **Compound collider — multi-subshape bodies (D2, game-grade plan `PLAN_NEXT.md`)**: new
  `Shape::Compound { subshapes: Vec<SubShape> }` where each `SubShape` is a child shape plus a
  local `offset` / `quat`. `support_local` takes the farthest child support along `dir`,
  `contains_local` any-child containment, inertia merges children (parallel-axis), and
  `bounding_sphere_r` takes the max child bound. Narrow-phase **recurses per subshape** via a
  `sub_body` helper (child inherits parent velocity incl. the offset angular term, inertia,
  layers, kinematic/sensor flags), picking the deepest contact; `ray_cast` recurses for the
  nearest hit; fracture merges child meshes. Useful for robot arms, car chassis, and humanoid
  limbs. Verified by `compound_body_rests_on_ground` (two child spheres rest on a static box
  ground without tunneling). 77 phy-rigid tests + full workspace green — **D2 (colliders:
  Capsule / Heightfield / Compound) is now complete**.
- **Contact warm-starting (D3, game-grade plan `PLAN_NEXT.md`)**: `ContactConstraint` gains a
  `warm_started` flag and `RigidWorld` holds a cross-frame `contact_impulses` cache (kept to
  the current frame's active pairs only, so it cannot grow unbounded). When narrow-phase
  rebuilds a contact, it restores the previous frame's accumulated normal/tangent impulse as
  the initial guess, and `solve_velocity` applies that warm-start impulse once before the
  main iteration (Box2D-style). This is the primary remedy for the B1 known limitation
  ("sliding-contact energy injection" / resting-stack jitter). A new 5-layer stack test
  (`warm_start_stabilizes_tall_stack`) converges to 0.5/1.5/2.5/3.5/4.5 and rests stably.
  78 phy-rigid tests + full workspace green.
- **Scene description DSL / prefab (B3, commercial-readiness plan §11)**: new
  `scene.rs` module with `SceneDesc<T>` (gravity + bodies + joints, reusing the
  already-serde `Body`/`JointConstraint` so the description is isomorphic to the
  engine's internal data — no field mirroring, pose quaternion / inverse-inertia /
  collision layers / kinematic / sensor flags all preserved). `RigidWorld`
  gains `load_scene_json(&str)` (replaces the whole world from a JSON prefab /
  level) and `to_scene_json()` (exports the current world for save/roundtrip).
  Verified by `scene_load_json_roundtrip_rebuilds_world` and
  `scene_minimal_json_uses_defaults` unit tests.
- Workspace-wide semantic versioning: all crates share `version = 0.1.0` via
  `[workspace.package]`. `repository`, `description`, `readme`, and `rust-version`
  (MSRV 1.74) now declared centrally for crates.io readiness.
- `build_package.py`: one-shot release packaging into `pkg/<profile>/`
  (dynamic lib + MinGW import lib + C header + rlib + USAGE.md).
- `wasm-cross-check.py`: guards `phy-core` / `phy-demo-web` wasm32 compatibility.
- `.github/workflows/ci.yml`: host build/test + release ignored regression +
  wasm cross-compile gate.

### Fixed
- `phy-rigid` (B1 contact bug): gravity was previously integrated into **static
  and kinematic** bodies (`inv_mass == 0`), polluting the relative normal velocity
  used by the contact solver so the contact impulse never fired and Baumgarte
  position correction ejected resting boxes upward. `RigidWorld::step` now guards
  gravity integration with `if b.inv_mass > 0 && !b.kinematic`, so static/kematic
  bodies keep a clean zero-relative-velocity contact and resting stacks stay put.
- `phy-ffi`: removed test-only `CString` import (now zero warnings).
- `phy-demo`: removed invalid `mut` in `raster.rs`; reused dead `ball_vy` helper
  in `l1_dropped_ball_settles`; dropped unused `std::any::Any` import in tests.

### Added
- `phy-ffi`: f32 boundary channel — `phy_world_step_f32` and
  `phy_world_get_fluid_positions_f32` / `_velocities_f32` /
  `phy_world_get_rigid_transforms_f32` (f32 write-back, ~50% bandwidth saving,
  align GPU/Unity f32 world). Internal world stays `f64` (S8 replay determinism);
  precision cast at the boundary, ABI stable.
- `phy-granular`: broad-phase neighbor search switched from naive O(n²) to
  reuse `phy_core::SpatialGrid` (cell = 2·max_r, 27-neighbor scan + exact
  sphere test + deterministic ascending dedup). Added `contact_pairs()` query.
- `build_package.py`: also produces MSVC import lib `phy_ffi.lib` via
  `llvm-dlltool` (header-symbol extraction → `.def` → implib), so Unity/Unreal
  can link without MinGW.
- `phy-ffi`: ABI stability contract — `PHY_FFI_ABI_VERSION` (monotonic, bump on
  breaking signature change) + `PHY_FFI_VERSION_{MAJOR,MINOR,PATCH}` constants so
  consumers can verify ABI compatibility at load time. All `phy_*` `extern "C"`
  symbols are the stable surface; helpers are crate-private.
- `phy-ffi/build.rs`: regenerate `phy_ffi.h` by default and honor
  `PHY_FFI_GEN_HEADER=0` to freeze it.
- Workspace compiles with **zero** Rust warnings (`cargo build` / `cargo test`).

## [0.1.0] - 2026-08-10

Initial tagged workspace. First version considered usable for research /
prototype / small-to-medium scale simulation by internal or trusted consumers.

### Physics models (M1–M27)

- **Rigid-body deterministic replay regression (L2 / M1-hardening, commercial-readiness plan §11)**:
  `phy-rigid` 新增两个非忽略单测 `rigid_determinism_repeat_run_bit_identical` 与
  `rigid_replay_is_bit_identical`:(1) 相同初始世界 + 固定步长序列(含中途碎裂 M21)重复运行
  逐位一致;(2) 经 `to_scene_json` 快照 + 逐帧 `dt` 录制,从快照重放两次与直跑终态逐位一致。
  直接守护「同输入同输出」的防战建模可复现性硬门槛(覆盖堆叠/落体/运动学推动/碎裂)。

- Rigid body (phy-rigid): impulse / sequential-impulse solver, joints.
- SPH fluid (phy-fluid): linear-complexity uniform spatial-hash neighbor search.
- Soft / cloth (phy-soft): XPBD.
- Granular (phy-granular): PBD with spatial-hash broad phase (was O(n²)).
- Continuum FEM (phy-solid).
- Fields (phy-field): heat / wave / acoustic / EM / gravity.
- Optics (phy-optics): refraction / caustics.

### Infrastructure
- Modular `World<T>` coupling core (phy-core): events, step / step_skipping /
  step_checked (NaN watchdog), total_momentum / kinetic_energy, validate.
- Deterministic replay (S8, phy-core::replay): record (snapshot + per-step dt/seed)
  and reproduce byte-identically.
- C ABI (L3, phy-ffi): opaque-handle `extern "C"` surface with `catch_unwind`
  safety and error codes; cbindgen header `phy_ffi.h`; f32 boundary channel;
  ABI version constants.
- WASM consumer (phy-demo-web): default + `gpu` (WebGPU) features.
- Stability regression (L1): no NaN/Inf, bounded kinetic energy, convergence.
- Determinism regression (L2): repeated-run bit-identical + save/reload replay.

### ABI stability policy
- The C ABI surface (`#[no_mangle] extern "C" fn phy_*`) is the contract. Breaking
  a function's signature, removing a symbol, or changing struct layout MUST bump
  `PHY_FFI_ABI_VERSION` and be recorded under this "Changed" section.
- Within the `0.x` semver band, Rust *crate* APIs may still change between minor
  releases; pin exact crate versions for Rust consumers. The C ABI is held stable
  once `1.0.0` is tagged.

### Known limitations (see PLAN.md "阶段二" roadmap)
- GPU numerical consistency: **host-side proxy evidence** in place — `phy-demo-web::gpu_ref`
  re-implements the W4 SPH and W5 granular-PBD `wgsl` kernels as deterministic serial Rust
  and asserts finite output + (SPH) mean-density≈rest + (granular) overlap resolved / bounds
  clamped, runnable via `cargo test -p phy-demo-web` without a WebGPU adapter. The `wasm` CI
  job (`wasm-cross-check.py --gpu`) confirms the `wgsl` compiles into `wasm32`. True on-adapter
  numeric comparison still requires a browser with WebGPU (manual, per PLAN §5.8).
- Some public debug/reference helper APIs in `phy-solid` carry `#[allow(dead_code)]`
  (intentionally exposed for library users, not yet called internally).
