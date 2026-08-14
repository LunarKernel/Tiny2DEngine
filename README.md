# Tiny2D Engine

> Product version: 2.0.0 / Latest generation: V21 (unreleased)<br>
> Language: C++17 · Graphics: SDL2 · UI: Dear ImGui · Build: CMake + vcpkg

Tiny2D Engine is a small two-dimensional physics laboratory aimed at rigid
bodies and introductory physics problems. The reusable physics core lives in
`Engine/`; ten independent experiments live in `Sandbox/`. Every experiment
offers SI-unit parameter entry, deterministic fixed-step integration, live
telemetry, history inspection, and quantities that can be checked against
analytical solutions.

Two numbering schemes coexist: `2.0.0` is the semantic product version
(single-sourced from `vcpkg.json`); `V9` through `V21` are experiment
generation codes, not product versions.

## Experiments

| Experiment | Topic |
| --- | --- |
| V9 Incline Laboratory | Incline, floor, spring, friction, uniform electric field |
| V11 RollLab | Disk/hoop transition from sliding to pure rolling |
| V13 Gravito-Orbit | Charged particle in uniform E, B, and gravity fields (includes the V12 preset) |
| V14 Driven PivotLab | Physical pendulum with damping and periodic drive (includes V10 behavior) |
| V15 ForceLab | Coupled translation and rotation driven through a centered or eccentric spring |
| V16 ContactLab | Native circle bodies, per-body materials, impact and rolling contact |
| V17 ImpactLab | Circle-circle continuous collision detection compared against the discrete path |
| V18 AtwoodLab | Massive pulley and rope constraints validated against the analytical Atwood acceleration |
| V19 ChaosLab | Double pendulum on rod constraints with normal-mode anchors and shadow-run divergence |
| V20 StackLab | Warm-started box stacks that truly rest, with per-interface loads validated against statics |

V21 adds versioned CSV export (tiny2d-csv format 1) to the StackLab and
ChaosLab monitors: metadata records the product version, model, complete
input parameters, and run state, and rows reproduce each history
sample's stored time. The remaining labs adopt the shared writer as they
are next touched.

## Build and test

From a VS2022 developer terminal:

```
cmake --preset msvc-x64
cmake --build --preset debug
ctest --preset test-debug
```

Or run the full verification gate directly:

```
powershell -File tools/verify.ps1 -Profile Fast   # format + Debug + tests
powershell -File tools/verify.ps1 -Profile Full   # adds Release/ASan/tidy
```

## Code layout

- `Engine/` — the reusable physics core (rectangular and circular rigid
  bodies, SAT and circle contacts, impulse solver, optional circle-circle
  CCD, bilateral constraints — revolute pins, pulley ropes, anchor and
  link rods — with reaction telemetry, tunable solver settings with an
  opt-in warm-starting contact cache, semi-implicit Euler integration).
  The implementation is split by responsibility under `Engine/internal/`
  (body_math, validation, contacts, solver, constraints); the only public
  API is `tiny2d_engine.h`. The engine does not depend on SDL or ImGui.
- `Sandbox/` — the experiment layer. Each lab is a UI-free physics model
  (`*_model.h/.cc`, the Config/State/Derived/Step quadruple, independently
  testable) plus one UI file (`*_simulation.cc`). The shared application
  shell lives in `Sandbox/app/lab_shell.h` (event loop, fixed-step
  advancement, pause/replay); the experiment list lives in
  `Sandbox/app/lab_registry.h`.
- `tests/` — the assertion support header shared by every test suite.

The dependency direction is fixed:
`main → registry → lab UI → shell → model → Engine`.

## More documentation

- Implementation guide: [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
- Long-term roadmap: [ROADMAP.md](ROADMAP.md)
- Change history: [CHANGELOG.md](CHANGELOG.md)
- Contribution and verification workflow: [CONTRIBUTING.md](CONTRIBUTING.md)
