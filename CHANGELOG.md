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

## [0.1.0] - 2026-08-10

Initial tagged workspace. First version considered usable for research /
prototype / small-to-medium scale simulation by internal or trusted consumers.

### Physics models (M1–M27)
- Rigid body (phy-rigid): impulse / sequential-impulse solver, joints.
- SPH fluid (phy-fluid): linear-complexity uniform spatial-hash neighbor search.
- Soft / cloth (phy-soft): XPBD.
- Granular (phy-granular): PBD (naive O(n²) neighbor search — see roadmap).
- Continuum FEM (phy-solid).
- Fields (phy-field): heat / wave / acoustic / EM / gravity.
- Optics (phy-optics): refraction / caustics.

### Infrastructure
- Modular `World<T>` coupling core (phy-core): events, step / step_skipping /
  step_checked (NaN watchdog), total_momentum / kinetic_energy, validate.
- Deterministic replay (S8, phy-core::replay): record (snapshot + per-step dt/seed)
  and reproduce byte-identically.
- C ABI (L3, phy-ffi): opaque-handle `extern "C"` surface with `catch_unwind`
  safety and error codes; cbindgen header `phy_ffi.h`.
- WASM consumer (phy-demo-web): default + `gpu` (WebGPU) features.
- Stability regression (L1): no NaN/Inf, bounded kinetic energy, convergence.
- Determinism regression (L2): repeated-run bit-identical + save/reload replay.

### Known limitations (see PLAN.md "阶段二" roadmap)
- FFI surface is `f64`-only (math layer already supports `f32`).
- Granular / field neighbor search is O(n²); fluid already uses a spatial hash.
- No MSVC `.lib` import library yet (MinGW `.dll.a` only).
- GPU numerical consistency not yet verified on a real WebGPU adapter.
