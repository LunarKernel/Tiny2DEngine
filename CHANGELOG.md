# Changelog

All notable project changes are recorded here. Product releases use Semantic
Versioning; identifiers such as V9 and V14 describe experiment generations,
not product versions.

## [Unreleased]

### Added

- V17 ImpactLab for deterministic circle-circle time-of-impact comparisons
  against the previous discrete collision path.
- V16 ContactLab for native circle impacts, per-body collision materials, and
  controlled sliding-to-rolling experiments.
- V15 ForceLab for coupled translation and rotation of a rectangular rigid
  body driven through a centered or eccentric spring attachment.
- V11 RollLab for sliding-to-rolling motion of disks and hoops in uniform
  electric and magnetic fields.
- V12 OrbitLab for exact charged-particle motion in uniform electric and
  magnetic fields.
- V13 Gravito-Orbit, extending OrbitLab with uniform gravity.
- V14 Driven PivotLab, extending the V10 physical pendulum with periodic drive,
  damping, resonance monitoring, and drive power.

### Changed

- The engine implementation was split into `Engine/internal/` units
  (body_math, validation, contacts, solver) behind an unchanged public
  header, and the two `Update` overloads were unified onto one step path
  protected by a bit-exact trajectory characterization test.
- Every lab now runs on a shared application shell
  (`Sandbox/app/lab_shell.h`); the model-selection menu renders from a lab
  registry, scrolls when the list outgrows the window, and keeps Quit
  reachable.
- All test suites share one assertion harness (`tests/test_support.h`).
- The root `README.md` is now the concise overview required by
  `CONTRIBUTING.md`; the implementation guide lives in
  `docs/DEVELOPER_GUIDE.md`.
- The project is English-only: the README, contribution guide, PR template,
  and developer guide were rewritten in English, and the Chinese guide was
  replaced by `docs/DEVELOPER_GUIDE.md` describing the current
  architecture.
- Shared fixed-step timing, bounded simulation history, and common UI helpers
  now support all experiment families.
- Runtime recovery, compiler diagnostics, automated verification, and CI checks
  were strengthened without changing validated V9 or V10 behavior.

## [2.0.0] - 2026-07-17

### Added

- V10 PivotLab with a physical pendulum, movable charged mass, gravity and
  electric-field torque, damping, telemetry, and history inspection.
- A model-selection screen that keeps the V9 incline laboratory independently
  available.

## [1.0.0] - 2026-07-17

### Added

- V9 Incline Laboratory with two blocks, collisions, friction, a spring,
  electric fields, SI calibration, telemetry, history, and defensive input
  validation.

[Unreleased]: https://github.com/LunarKernel/Tiny2DEngine/compare/v2.0.0...HEAD
[2.0.0]: https://github.com/LunarKernel/Tiny2DEngine/releases/tag/v2.0.0
[1.0.0]: https://github.com/LunarKernel/Tiny2DEngine/commit/2adf9d56d5c62d86c6af9bfa498db6ff96e37ae4
