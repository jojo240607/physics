# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Workspace-wide semantic versioning: all crates share `version = 0.1.0` via
  `[workspace.package]`. `repository`, `description`, `readme`, and `rust-version`
  (MSRV 1.74) now declared centrally for crates.io readiness.
- `build_package.py`: one-shot release packaging into `pkg/<profile>/`
  (dynamic lib + MinGW import lib + C header + rlib + USAGE.md).
- `wasm-cross-check.py`: guards `phy-core` / `phy-demo-web` wasm32 compatibility.
- `.github/workflows/ci.yml`: host build/test + release ignored regression +
  wasm cross-compile gate.

### Fixed
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
- GPU numerical consistency not yet verified on a real WebGPU adapter (P5).
- Some public debug/reference helper APIs in `phy-solid` carry `#[allow(dead_code)]`
  (intentionally exposed for library users, not yet called internally).
