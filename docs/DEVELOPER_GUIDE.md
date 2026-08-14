# Tiny2D Engine Developer Guide

> Current experiment generation: V21 (unreleased) / Latest release: 2.0.0
> (V10)<br>
> C++17 · SDL2 · Dear ImGui · CMake + vcpkg

This guide documents the system as it exists after the 2026-08 architecture
reconstruction. It replaces the earlier Chinese-language guide, which
described the pre-reconstruction (V14-era) layout.

## 1. Purpose

Tiny2D Engine is a compact, verifiable two-dimensional physics laboratory.
It is not a general-purpose game engine. Its value is transparent physical
models, measurable results, and a reusable core that grows only when a real
experiment requires it. The intended workflow: pick an experiment, enter
SI-unit parameters, predict the behavior, run and inspect the simulation,
and compare the numbers against physical laws or analytical solutions.

## 2. Architecture

```
Sandbox/main.cc            application bootstrap and lab selection menu
Sandbox/app/lab_registry.h the table of installed labs
Sandbox/*_simulation.cc    one UI file per lab (setup, scene, monitor)
Sandbox/app/lab_shell.h    shared frame loop and lab lifecycle
Sandbox/*_model.{h,cc}     one UI-free physics model per lab
Engine/tiny2d_engine.h     the engine's public API (the only public header)
Engine/internal/*          engine implementation units
```

The dependency direction is fixed and acyclic:
`main -> registry -> lab UI -> shell -> model -> Engine`. The engine must
not depend on SDL, ImGui, or any specific experiment; the headless CI job
builds the engine and every model with GCC and Clang under
`-Wall -Wextra -Wpedantic -Werror` to enforce this.

### Engine internals

`Engine/tiny2d_engine.cc` holds the public entry points, parameter
validation, semi-implicit Euler integration, and step orchestration. The
supporting units under `Engine/internal/` are implementation details in
namespace `tiny2d::internal` with no stability guarantee:

- `body_math.h` — vector algebra, rotations, vertices, axes, inverse mass,
  moments of inertia (header-only).
- `validation.{h,cc}` — the failure-atomic input validation contract.
- `contacts.{h,cc}` — SAT rectangle contacts with face clipping, circle and
  rectangle-circle contacts, circle-circle time of impact, and the
  missed-sweep test that gates CCD.
- `solver.{h,cc}` — material resolution and mixing, the impulse solver
  (including the coupled two-point normal solve), friction, and the four
  window-wall contacts. The solver templates accept any body type that
  exposes mass, pose, and velocity state, so rectangle-circle pairs resolve
  through the same code path as same-shape pairs.
- `constraints.{h,cc}` — bilateral constraint resolution: revolute pins,
  pulley-rope constraints, and world-anchored and body-to-body rods,
  composable through a `ConstraintSet`. Validation, the per-round velocity
  solve (a direct 2x2 block solve per rope coupled to the pulley's angular
  velocity; scalar axial solves per rod), and the full position projection
  that keeps constraint-length drift from accumulating.
- `warm_contacts.{h,cc}` — the warm-starting accumulated-impulse contact
  formulation used when a caller passes a `ContactCache`: body-body
  manifolds detected once per step with stable feature ids (face indices
  and clip-point ids threaded through the SAT clipping) and a 2%
  axis-preference hysteresis, deterministic exact-then-pair-fallback
  impulse matching, warm application completed by a wall velocity anchor,
  accumulated normal/tangent clamping on a fixed per-manifold tangent
  basis with the two-point block solve retained, wall-anchored velocity
  iterations, geometric position passes plus the wall snap, and sorted
  write-back with stale eviction. The historical cold path (no cache) is
  untouched and golden-guarded; SolverSettings defaults reproduce its
  constants bit for bit (ROADMAP §3.2 additive coexistence — migrating
  the cold path needs its own parity evidence).

## 3. Engine semantics

- **Units:** caller-selected but internally consistent; the engine performs
  no SI or pixel conversion. Labs use SI units and convert at their own
  boundary.
- **Axes and angles:** +X points right, +Y points down; angles are radians
  and a positive angle or angular velocity appears clockwise on screen.
- **Bodies:** `Rectangle` and `Circle` are plain aggregates. Zero mass makes
  a body static; a dynamic body needs mass >= 1e-6. `fixed_rotation` locks
  the angle. `Circle` chooses `kSolidDisk` (I = mr^2/2) or `kHoop`
  (I = mr^2) inertia.
- **Loads:** `AddForceAtPoint` and `AddTorque` accumulate force and torque
  for the next successful `Update`, which consumes and clears them — even
  when `delta_time` is zero.
- **Materials:** `CollisionMaterial` either inherits all world values with
  the `{-1, -1, -1}` sentinel or specifies all three fields. Contact
  restitution takes the larger body value; both friction values mix by
  geometric mean.
- **Validation:** every public operation validates completely before
  mutating anything and throws `std::invalid_argument` on invalid input, so
  failure paths leave inputs unchanged.
- **Stepping:** there is exactly one integration and solver path behind
  three public `Update` overloads. The rectangle-only overload delegates to
  the mixed rectangle/circle overload (no circles, CCD disabled, legacy
  restitution threshold 20), which forwards to the constrained overload
  with empty constraint sets. `TestLegacyUpdateMatchesMixedUpdateTrajectories`
  and `TestMixedUpdateMatchesConstrainedUpdateWithoutConstraints` keep the
  chain bit-identical, and `TestGoldenMixedTrajectoryCheckpoints` pins the
  whole no-constraint trajectory to checkpoints recorded before the
  constrained step existed; none of the three may be weakened.
- **Constraints:** `RevolutePin` pins a dynamic circle's center to a world
  anchor with rotation free; `PulleyRope` is a massless, inextensible,
  non-slipping rope from one body's center of mass over a pinned pulley to
  another body's center of mass, with fixed world anchor points where the
  segments leave the pulley; `AnchorRod` holds a dynamic body's center at
  a fixed distance from a world anchor (free to swing); `LinkRod` holds
  two dynamic bodies at a fixed distance. All constraint kinds compose
  through a `ConstraintSet`. Constraint velocity errors are removed by
  impulses between force integration and position advancement (each rope
  is an exact 2x2 block solve coupled to the pulley's angular velocity;
  rods are scalar axial solves, sequential over kSolverIterations rounds
  for chains); contact resolution runs unchanged; constraint position
  errors are then fully projected out, so length drift stays at
  float-rounding level. Reported reactions: rope tensions (positive =
  taut), signed rod axial forces (positive = tension), and the pin force,
  which excludes the rope wrap load because rope anchors are fixed world
  points — labs assemble the physical axle load as pulley weight plus both
  tensions. The rope is bilateral (no slack modeling), rods attach at
  centers of mass only, and CCD cannot be combined with constraints; these
  limits are validated, not silent.
- **Contact solver formulations:** the cold path re-detects contacts and
  solves per-iteration-clamped impulses every round (historical,
  bit-identical, golden-guarded); the warm path (opt-in via
  `ContactCache`) accumulates impulses across steps so resting stacks
  carry their loads instead of rebuilding them from zero — the measured
  difference between a 0.227 m/s jitter and exact-zero rest on the
  ten-box reference. Warm-path limitation (measured): offset stacks
  stand but wobble at 0.06-0.10 m/s because marginally-clipped tilted
  interfaces flicker their second manifold point, cold-starting it on
  every reappearance; the roadmap's persistent-contact-manifolds trigger
  records the fix path.
- **Constraint energy limitation (measured):** velocity-projection
  constraint stepping deletes the centripetally rotated axial velocity
  component every step, losing kinetic energy at a rate scaling as
  v^4 dt / L^2 under fast rotation (first order in dt; present even when
  the solve is exact, so solver iterations cannot remove it). ChaosLab
  therefore substeps 32x and binds its energy criteria in the 25-degree
  regime; whip-heavy chaotic trajectories lose several percent of their
  released energy per minute, bounded by a regression ceiling in the V19
  suite. The recorded upgrade path is the roadmap's
  energy-consistent-constraint-integration trigger.
- **Continuous collision detection:** when `enable_circle_circle_ccd` is
  true, the step first integrates speculatively; if a circle pair that is
  separated at both endpoints would cross inside the step, the world is
  advanced impact-to-impact at the earliest times of impact (stable root
  form `c / (-b + sqrt(disc))`), each contact resolved with restitution
  allowed, before the discrete solver iterations run. Otherwise the
  discrete result is preserved exactly.
- **Damping:** exponential per-body damping applies after force
  integration: `v = (v0 + F/m * dt) * exp(-rate * dt)`.
  `TestPerBodyExponentialDamping` locks this ordering; ForceLab's energy
  accounting depends on it.

## 4. The lab pattern

Each lab consists of a **model** and a **UI file**.

The model (`*_model.h/.cc`) is deterministic, UI-free, and independently
testable. It follows one shape everywhere:

- `Config` — validated SI parameters with documented ranges; a
  `Get*ConfigError` function returns `nullptr` or a stable message.
- `MakeInitial*State` — builds the starting `State` (throws on invalid
  config).
- `Step*(config, dt, state*)` — advances one fixed step; returns false and
  leaves the state unchanged on invalid input or an unstable result.
- `Derived` — quantities computed from a state for telemetry and testing:
  energies, torques, analytical references, error terms.
- `Find*State(history, t)` — nearest-sample history lookup.

The UI file provides the lab's setup screen, scene drawing, and monitor
panel. Standard continuous labs declare a traits struct and call
`shell::RunLab`; the shell owns the SDL event pump, the ImGui frame,
rendering, the fixed-step clock, history recording, and the start / pause /
error / inspect protocol. The traits contract is documented at the top of
`Sandbox/app/lab_shell.h`. Labs with nonstandard lifecycles (the V9 incline
lab renders with raw SDL under the ImGui layer; V17 ImpactLab runs a
single-step evidence loop with replay) build directly on
`shell::RunFrameLoop`.

Shared utilities: `fixed_step_clock.h` (frame-time accumulation into fixed
physics steps), `simulation_history.h` (bounded, decimating history append),
and `sim_ui.h` (slider-plus-input widgets, arrow drawing, and the
CSV-export-to-working-directory helper).

### CSV export (tiny2d-csv format 1)

`Sandbox/csv_export.{h,cc}` is the shared, UI-free export writer
(ROADMAP §12): `BuildCsv` emits `#`-prefixed metadata lines — the format
version, the product version (single-sourced from `vcpkg.json` through
CMake's `TINY2D_PRODUCT_VERSION`), the model identifier, one
`# param <field>: <value>` line per config field, and a status line
recording the run state at export time (the §12 "terminal status" for a
live lab) — then a header row and one row per history sample. Fields are
RFC-4180 escaped; floats print with 9 significant digits and doubles
with 17, so every value round-trips exactly. The time column is each
sample's stored time, never a uniform index interval. Rejection is
atomic (`std::invalid_argument`, no partial output) and output is
deterministic (equal inputs give equal bytes).

Model-layer entry points `BuildStackCsv` and `BuildChaosCsv` are the two
consumers (§3.4 threshold); each lab monitor's Export CSV button writes
`<slug>_<yyyymmdd_hhmmss>.csv` into the process working directory,
appending an `_<n>` attempt suffix instead of overwriting on a
collision. Labs touched in the future should adopt the same writer.

## 5. The experiments

| Lab | Physics | Analytical anchors |
| --- | --- | --- |
| V9 Incline Laboratory | Two blocks on a ramp and floor with friction, a spring, and a uniform electric field | Energy accounting, SI calibration |
| V11 RollLab | Disk or hoop sliding into pure rolling under friction plus E and B fields | Slip time, rolling condition, energy loss |
| V13 Gravito-Orbit | Charged particle in uniform E, B, and gravity | Exact cyclotron/drift trajectory (V12 preset preserved) |
| V14 Driven PivotLab | Physical pendulum with damping and periodic torque | Small-angle period, resonance ratio, drive power (V10 behavior preserved) |
| V15 ForceLab | Rectangle driven through a centered or eccentric spring attachment | Centered-case period, energy budget with damping loss |
| V16 ContactLab | Circle impacts and rolling with per-body materials | Momentum/energy checks, rolling slip tolerance |
| V17 ImpactLab | Discrete vs CCD lanes over one fixed step | Swept-circle entry/exit times, expected post-impact state, per-lane error terms |
| V18 AtwoodLab | Two hanging blocks on a rope over a pinned massive pulley | a = (m_b - m_a) g / (m_a + m_b + I/R^2), tension pair, no-slip coupling, rope-length drift, energy budget |
| V19 ChaosLab | Point-mass double pendulum on rod constraints with a shadow run | Small-angle normal modes (2%), 25-degree energy budget, rod drift, factor-1000 divergence from a 1e-4 rad offset |
| V20 StackLab | Warm-started resting box stacks with live criteria telemetry | Per-interface loads = (n-k) m g (1% time-averaged; 0.000% measured), exact-zero resting speed on the aligned reference, offset stand-without-collapse |

V21 is a cross-cutting measurement generation, not a lab: versioned CSV
export (see section 4) for the StackLab and ChaosLab monitors, plus the
engine's 2^20 cached-body bound from the V20 review's deferred notes.

## 6. Adding a lab

1. Write the model pair (`Sandbox/<name>_model.h/.cc`) with documented SI
   units, axes, signs, and ranges.
2. Write the UI file (`Sandbox/<name>_simulation.cc`): a traits struct on
   `shell::RunLab` for a standard lab, or a custom frame on
   `shell::RunFrameLoop`.
3. Declare the run function in `Sandbox/simulations.h` and add one entry to
   `Sandbox/app/lab_registry.h`.
4. Add the model source, the simulation source, and a test suite to
   `CMakeLists.txt`. Tests link production targets and use the shared
   `CHECK` harness from `tests/test_support.h`.
5. Every model needs at least one analytical anchor, a conservation or
   dissipation law, explicit tolerances, a deterministic test, and a
   long-run finite-state check.

## 7. Verification

`tools/verify.ps1 -Profile Fast` runs clang-format (`--Werror`, tracked and
untracked sources), the Debug build, all test suites, and whitespace checks.
`-Profile Full` adds Release, ASan, clang-tidy, and the engine-only
warnings-as-errors build. CI additionally builds headless on Ubuntu with GCC
and Clang. GCC and Clang reject unused file-local functions under `-Werror`;
MSVC does not warn about them, so run formatting and keep dead code out
before pushing.

## 8. Versioning

`version-semver` in `vcpkg.json` is the single source of the product
version; CMake and the window title derive from it, and CSV exports
record it in their metadata. V-numbers (V9 through V21) name experiment
generations and never become semantic versions. Releases tag
`v<version-semver>` and update `CHANGELOG.md`.
