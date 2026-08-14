# Changelog

All notable project changes are recorded here. Product releases use Semantic
Versioning; identifiers such as V9 and V14 describe experiment generations,
not product versions.

## [Unreleased]

### Added

- V21 versioned CSV export: a shared UI-free writer
  (`Sandbox/csv_export`, tiny2d-csv format 1) with RFC-4180 escaping,
  round-trip numeric formatting (9 significant digits for floats, 17
  for doubles), and metadata comment lines carrying the product
  version, model identifier, complete input parameters, and the run
  state at export time. StackLab and ChaosLab monitors gained an
  Export CSV button writing `<slug>_<timestamp>.csv` into the working
  directory with collision-safe `_<n>` suffixes; rows are keyed on
  each sample's stored time, never a uniform index interval.
- Engine hardening from the V20 review's deferred notes: worlds using
  a contact cache are validated to at most 2^20 bodies per shape kind
  (the cache key packs 20-bit body indices; a larger index would have
  silently corrupted keys), with the limit documented on the
  `ContactCache` lifecycle contract and regression-tested; the
  StackLab interface-load fixed-step division and energy-scale choice
  are now documented in place.
- V20 StackLab: a ten-box stack that truly rests through the engine's
  new warm-starting contact solver, with live ROADMAP §11 criteria,
  per-interface load telemetry validated against Newtonian statics
  ((n-k) m g to 0.000% measured), and offset and collapse
  demonstrations.
- Engine solver stabilization: tunable `SolverSettings` (iterations,
  position slop, correction factor; defaults bit-identical to the
  historical constants) and a caller-owned `ContactCache` enabling an
  accumulated-impulse warm-starting contact formulation with stable
  manifold feature ids, deterministic pair-fallback matching, and a
  wall-anchored iteration structure, behind a new full-control `Update`
  overload. The historical cold path is unchanged and golden-guarded.
- A documented, measured limitation: offset stacks stand without
  collapse but wobble at 0.06-0.10 m/s from manifold point-count
  flicker at marginally-clipped tilted interfaces; a
  persistent-contact-manifolds trigger row was added to the roadmap's
  deferred-work table.
- V19 ChaosLab: a point-mass double pendulum on rod constraints with
  small-angle normal-mode anchors, conservative and damped energy budgets
  in the 25-degree regime, rod-drift and determinism guarantees, and a
  built-in shadow run whose separation telemetry makes chaotic divergence
  visible.
- Engine rod constraints: `AnchorRod` (a dynamic body held at a fixed
  distance from a world anchor, free to swing) and `LinkRod` (two dynamic
  bodies at a fixed distance), bilateral and COM-attached, with signed
  axial-force telemetry, composable with pins and ropes through the new
  `ConstraintSet` Update overload.
- A documented, quantified engine limitation: velocity-projection
  constraint stepping loses mechanical energy at a rate scaling as
  v^4 dt / L^2 under fast rotation; the ChaosLab substeps 32x and binds
  its energy criteria at 25 degrees, and an energy-consistent-integration
  trigger row was added to the roadmap's deferred-work table.
- V18 AtwoodLab: two hanging blocks on a massless, inextensible,
  non-slipping rope over a pinned pulley with real rotational inertia,
  validated against the analytical Atwood acceleration, tension pair,
  no-slip coupling, rope-length drift, and conservative and damped energy
  budgets.
- Engine bilateral constraints: `RevolutePin` (a dynamic circle pinned to a
  world anchor with free rotation) and `PulleyRope` (a center-of-mass rope
  over a pinned pulley with no-slip rotational coupling), solved by
  per-rope 2x2 velocity impulses plus full position projection, with rope
  tension and pin-force reaction telemetry through a new constrained
  `Update` overload.
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

- The `ConstraintSet` overload now forwards to the new full-control
  overload with default solver settings and no cache, so all five public
  `Update` entry points share one step path; the wall contact resolve
  was split into velocity and snap halves (recombined bit-identically on
  the cold path) so the warm path can anchor every iteration.
- The constrained `Update` overload taking pin and rope vectors now
  forwards to the new `ConstraintSet` overload with empty rod vectors, so
  all four public `Update` entry points share one step path.
- All three public `Update` overloads now share one step path: the mixed
  overload forwards to the constrained overload with empty constraint
  sets. The integration loop was split into velocity and position phases
  so the constraint solve can run between them; golden trajectory
  checkpoints recorded before the restructuring keep the no-constraint
  path bit-identical.
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
