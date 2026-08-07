# Tiny2D Engine Long-Term Development Roadmap

- **Document status:** Proposal
- **Project baseline:** V14 development line, based on the 2.0 release
- **Target milestone:** Tiny2D Physics Lab 3.0
- **Last updated:** 2026-08-07

## 1. Purpose

Tiny2D Engine should evolve from a collection of physics demonstrations into
a compact, verifiable, and reproducible two-dimensional physics laboratory.
The project is not intended to compete with general-purpose commercial game
engines. Its value should come from transparent physical models, measurable
results, and a reusable core that grows only when a real experiment requires
it.

The intended user workflow is:

1. Select a physical experiment.
2. Enter parameters in SI units.
3. Predict the expected behavior.
4. Run, pause, and inspect the simulation.
5. Compare numerical results with physical laws or analytical solutions.
6. Export enough information to reproduce the experiment.

The terms **MUST**, **SHOULD**, and **MAY** in this document express required,
recommended, and optional work respectively.

## 2. Current Baseline

The current development line contains four independent experiment families:

- **V9 Incline Laboratory:** blocks, an incline, a floor, collisions,
  friction, a spring, a uniform electric field, SI calibration, telemetry, and
  history inspection.
- **V11 RollLab:** sliding-to-rolling transitions for a solid disk or hoop,
  including friction and uniform electric and magnetic fields.
- **V13 Gravito-Orbit:** charged-particle motion in uniform electric,
  magnetic, and gravitational fields, while preserving the V12 no-gravity
  preset.
- **V14 Driven PivotLab:** the V10 physical pendulum extended with damping,
  periodic driving torque, resonance monitoring, and drive power.

The reusable engine currently provides rotating rectangular bodies, SAT
collision detection, one- and two-point contact manifolds, impulse response,
friction, restitution, static bodies, fixed rotation, field acceleration, and
boundary collisions. The Sandbox layer provides model-specific SI units,
fixed-step clocks, bounded history, parameter interfaces, monitoring, and
visualization.

The main limitations relevant to this roadmap are:

- no general external-force or applied-torque interface;
- no native circle collision body;
- no per-body material properties;
- no general joint, rope, or bilateral constraint solver;
- no continuous collision detection for high-speed motion;
- no time-series plotting, data export, or experiment file format;
- no general-purpose contact cache, warm start, or sleeping system.

## 3. Strategic Principles

### 3.1 Physics first

Every feature release MUST answer a clear physical question. A new reusable
engine abstraction SHOULD be introduced only when a concrete experiment needs
it.

### 3.2 Preserve validated experiments

V9 and V10 behavior MUST remain available. New core paths SHOULD be additive
until parity tests demonstrate that migrating an existing model is safe.

### 3.3 Keep the dependency boundary

Reusable mathematics, body dynamics, collision, and constraints belong in
`Engine/`. Model equations, SI calibration, menus, telemetry, and rendering
belong in `Sandbox/`. The dependency direction MUST remain
`Sandbox -> Engine`; Engine MUST NOT depend on SDL2, ImGui, or a named
experiment.

### 3.4 Prefer evidence over infrastructure

Broad-phase acceleration, sleeping, parallel physics, and additional
abstraction layers MUST NOT be added without a measured need. A second real
consumer is the normal threshold for extracting shared model code.

### 3.5 Make numerical behavior observable

Each model MUST expose at least one analytical anchor, conservation or
dissipation law, explicit error tolerance, deterministic test, and long-run
finite-state check.

## 4. Roadmap Overview

| Stage | Working title | Primary physics objective | Reusable capability |
| --- | --- | --- | --- |
| Foundation | V14 consolidation | Stabilize the current development line | Version, API, and CI consistency |
| V15 | ForceLab | Eccentric spring coupled translation and rotation | Applied forces and torques |
| V16 | ContactLab | Circle impacts and rolling contact | Circle bodies and per-body materials |
| V17 | ImpactLab, conditional | High-speed collision | Continuous collision detection |
| V18 | AtwoodLab | Massive pulley and inextensible rope | Bilateral and rotational constraints |
| V19 | ChaosLab | Double-pendulum nonlinear dynamics | Multi-body articulated motion |
| V20 | StackLab | Stable persistent contact | Iterative solver stabilization |
| Release | Physics Lab 3.0 | Reproducible physics experiments | Data, packaging, and release contract |

## 5. Foundation Gate: Consolidate V14

This gate changes project consistency, not validated physics behavior.

- Product versions and experiment identifiers MUST be separated. Semantic
  versions such as `2.1.0` describe releases; V9 or V14 identifies an
  experiment generation.
- CMake, vcpkg metadata, application title, Git tag, and release notes SHOULD
  derive from one product-version source.
- Public Engine APIs MUST document units, axes, angle direction, valid ranges,
  exception behavior, and whether a failed operation preserves state.
- Headless Engine and model builds SHOULD be verified with both GCC and Clang,
  in addition to the existing Visual Studio checks.
- The default branch SHOULD require the relevant CI checks and reject force
  pushes.
- The concise repository README and the detailed Chinese implementation guide
  SHOULD remain separate documents.

**Exit condition:** the current models pass the complete verification matrix,
and the repository presents one unambiguous product version.

## 6. V15 ForceLab: Eccentric Spring Rigid Body

### Goal

Simulate a rectangular rigid body connected to a fixed anchor by a spring at
an adjustable attachment point. A centered attachment produces ordinary
harmonic translation. An off-center attachment couples translation and
rotation through the torque

$$
\tau = \mathbf{r} \times \mathbf{F}.
$$

### Required capability

- Apply an arbitrary force to a dynamic body for one fixed step.
- Apply a force at a world-space point and derive its torque about the center
  of mass.
- Integrate externally supplied torque without changing the legacy field and
  collision behavior.
- Support optional linear and angular damping with explicit units.
- Monitor displacement, velocity, angle, angular velocity, spring energy,
  kinetic energy, and total mechanical energy.

### Acceptance criteria

- In the centered, undamped configuration, the measured period differs from
  $T = 2\pi\sqrt{m/k}$ by no more than 1%.
- Total mechanical-energy drift remains below 0.5% over 100 periods in the
  conservative reference configuration.
- In the damped reference case, mechanical energy does not exceed its initial
  value by more than 0.1%, and mechanical plus recorded dissipated energy
  remains within 0.5% of its initial value.
- An off-center force produces the expected torque sign and angular
  acceleration.
- V9-V14 behavior remains unchanged and all regression suites pass.

## 7. V16 ContactLab: Circle Bodies and Materials

### Goal

Promote circles from a Sandbox-only analytical idea to a native Engine
collision shape, then use controlled impact experiments to validate momentum,
energy, and rotation.

### Required capability

- Dynamic and static circle bodies.
- Circle-circle and circle-rectangle contact detection and response.
- Physically documented rotational inertia for solid disks and hoops.
- Per-body friction and restitution values with a documented mixing rule.
- Momentum, angular momentum, translational energy, and rotational energy
  telemetry.

The first implementation SHOULD use the smallest representation that supports
rectangles and circles. It SHOULD NOT introduce a shape inheritance hierarchy,
arbitrary polygons, or compound bodies.

### Acceptance criteria

- Collision detection and response are symmetric when body order is swapped.
- Linear momentum error is below 0.1% in the elastic reference case.
- Kinetic-energy error is below 1% in the elastic reference case.
- The inertia model matches $I = \frac{1}{2}mR^2$ for a solid disk and
  $I = mR^2$ for a hoop.
- Static friction stays within its limit, and kinetic friction opposes slip.

## 8. V17 ImpactLab: Conditional Continuous Collision Detection

V17 SHOULD be implemented only after a deterministic V16 regression test
demonstrates tunneling at a physically supported speed.

If triggered, the initial scope is:

- compute the earliest time of impact for the minimum required shape pair;
- advance to that instant, resolve the contact, and advance the remaining
  substep;
- preserve deterministic ordering when more than one collision is possible;
- preserve the existing low-speed discrete-contact behavior.

**Acceptance criteria:** an object that travels farther than its diameter in
one fixed step does not pass through the tested target, and repeated runs
produce identical results.

If no supported experiment demonstrates tunneling, the project SHOULD skip
this stage and proceed to V18.

## 9. V18 AtwoodLab: Massive Pulley and Rope Constraints

### Goal

Simulate two masses connected by a massless, inextensible rope passing over a
pulley with nonzero rotational inertia.

### Required capability

- constant-distance or rope-length constraints;
- a revolute constraint for the pulley axis;
- no-slip coupling between rope speed and pulley angular speed;
- constraint reaction and rope-tension telemetry;
- conservative and dissipative configurations.

The main analytical anchor is

$$
a = \frac{(m_2-m_1)g}{m_1+m_2+I/R^2}.
$$

### Acceptance criteria

- Simulated acceleration differs from the analytical result by no more than
  1% in the reference case.
- The two masses have equal speed magnitude and opposite rope direction.
- Rope-length drift remains below $10^{-4}$ m over 60 simulated seconds.
- Conservative mechanical-energy drift remains below 0.5% over the reference
  run.

## 10. V19 ChaosLab: Double Pendulum

### Goal

Use the V18 constraint foundation to simulate a double pendulum and expose
nonlinear coupling, modal behavior, energy exchange, and sensitivity to
initial conditions.

### Required capability

- two linked rotating bodies with independent mass and length;
- configurable initial angles and angular velocities;
- optional damping;
- paired runs with a small initial-condition difference;
- angle, angular velocity, energy, phase, and trajectory-separation telemetry.

A uniform electric field and charged bobs MAY be added only after the pure
gravity model satisfies its acceptance tests.

### Acceptance criteria

- Small-angle normal-mode frequencies differ from analytical values by no
  more than 2%.
- Conservative energy drift remains below 1% over 60 simulated seconds.
- Identical configurations produce identical checkpoints.
- Nearby initial conditions may diverge physically, but constraints remain
  finite and stable without NaN or unbounded length error.

## 11. V20 StackLab: Persistent Contact Stability

### Goal

Make resting and multi-contact systems stable enough to represent the core
behavior expected from a small game-physics engine.

### Candidate capability

- configurable solver iterations;
- persistent contact identifiers and cached impulses;
- warm starting;
- stable resting-contact thresholds;
- sleeping only if it materially improves the target stack scenario.

### Acceptance criteria

For a ten-body reference stack simulated for 60 seconds:

- maximum contact penetration stays below 0.5% of the smallest body dimension;
- during the final 10 seconds, each resting body stays below $10^{-3}$ m/s
  linear speed and $10^{-3}$ rad/s angular speed;
- during the final 10 seconds, each resting center of mass drifts by less than
  0.1% of the smallest body dimension;
- total mechanical energy never exceeds its initial value by more than 0.1%;
- all states remain finite and repeated runs produce identical checkpoints;
- V9 ramp and junction behavior remains within its established tolerances.

## 12. Cross-Cutting Experiment Capabilities

Measurement features should grow alongside the physics roadmap instead of
occupying several releases without new physical value.

The project SHOULD progressively add:

- one or two selectable time-series curves per experiment;
- two history cursors and differences such as $\Delta t$, $\Delta x$,
  $\Delta v$, and $\Delta E$;
- CSV export with real time, SI units, model identifier, product version,
  complete input parameters, and terminal status;
- a versioned experiment file for save, load, and deterministic replay;
- analytical value, simulated value, absolute error, and relative error;
- event markers for collisions, rolling transition, loss of contact,
  extrema, and terminal states.

The first plot SHOULD use existing ImGui drawing facilities. A new plotting
dependency is justified only when the native implementation demonstrably
cannot meet the required interaction or scale.

History samples have explicit timestamps and may be compacted. Plotting and
export MUST use each sample's stored time rather than assuming a uniform index
interval.

## 13. Verification Contract for Every Physics Version

Every new model MUST include:

1. documented SI units, axes, signs, angle direction, and valid ranges;
2. at least one analytical or independently calculated reference result;
3. an energy, momentum, constraint, or dissipation-law test;
4. non-finite, illegal, boundary, and extreme-finite input tests;
5. deterministic fixed-step tests with explicit floating-point tolerances;
6. at least 60 simulated seconds of finite-state validation;
7. failure atomicity where an operation can reject input;
8. a visual UI smoke test;
9. passing Fast and Full repository verification profiles.

New tests MUST link production targets and MUST NOT include implementation
`.cc` files directly.

## 14. Trigger-Based Work, Not Scheduled Work

The following capabilities remain deferred until a measurable trigger exists:

| Capability | Trigger |
| --- | --- |
| Broad phase | Pair checks measurably dominate frame time or supported scenes exceed roughly 50-100 dynamic bodies |
| Sleeping | Resting-body processing is a demonstrated performance or stability problem |
| Parallel physics | Single-threaded physics misses the target frame budget in a supported benchmark |
| General convex polygons | A committed experiment cannot be represented by rectangles and circles |
| Full-project `double` conversion | Precision tests show `float` is the limiting error source |
| Test-framework migration | The current lightweight tests prevent practical filtering, diagnosis, or reporting |

## 15. Explicit Non-Goals

The 3.0 roadmap does not include:

- an ECS, scripting language, plugin ABI, or general scene editor;
- 3D physics, soft bodies, fluids, or finite-element analysis;
- GPU physics or speculative multithreading;
- network accounts, cloud synchronization, or course management;
- automatic migration of all validated legacy models to every new core API;
- a large plotting, reporting, or serialization dependency without measured
  need.

## 16. Physics Lab 3.0 Release Gate

The project is ready for a formal 3.0 release when:

- V9 and V10 compatibility tests still pass;
- V15, V16, V18, V19, and V20 satisfy their numerical acceptance criteria;
- V17 also satisfies its criteria when the documented tunneling trigger makes
  continuous collision detection part of the release scope;
- Debug, Release, AddressSanitizer, clang-tidy, and Engine-only
  warnings-as-errors checks pass;
- every released model provides measurable telemetry and at least one theory
  comparison;
- experiment configuration and CSV export are versioned and reproducible;
- a Windows distribution runs on a clean machine without Visual Studio;
- CMake metadata, vcpkg metadata, application title, Git tag, and changelog
  agree on the release version;
- the repository contains an explicit license and release checksums.

## 17. Recommended Next Action

The next feature proposal SHOULD be **V15 ForceLab: Eccentric Spring Rigid
Body**. It is the smallest experiment that adds a genuinely reusable physics
capability while connecting the project's existing spring, rigid-body, and
rotational work.

The V15 proposal should define the force and torque API, coordinate and unit
conventions, the centered analytical reference case, the eccentric coupled
case, and exact regression tolerances before implementation begins.
