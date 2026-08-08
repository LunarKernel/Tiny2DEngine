# Tiny2D Engine — V17 Project Review

**Reviewed:** working tree on `codex/v14-driven-pivotlab`, HEAD `724b466`, plus the
uncommitted V15–V17 changeset.
**Reviewer:** independent read-only review (no source files were modified).
**Date:** 2026-08-08

---

## 1. Scope of this review

### Instruction documents read

| Document | Purpose | State |
| --- | --- | --- |
| `AGENTS.md` | Agent scope, `持续更新` trigger, automated-iteration workflow, verification contract | Unchanged since `724b466` |
| `CONTRIBUTING.md` | Build/test commands, iteration process, **new** version-and-docs policy | Modified (+10) |
| `ROADMAP.md` | Long-term plan, V15–V20 goals and acceptance criteria | Committed at `724b466`, **not updated for V15–V17** |
| `CHANGELOG.md` | Keep-a-Changelog history, SemVer vs experiment-generation split | New, untracked |
| `README.md` / `docs/DEVELOPER_GUIDE.zh-CN.md` | Chinese implementation guide | Modified / new, **byte-identical duplicates** |

### Code read

- `Engine/tiny2d_engine.h` (188 lines) and `Engine/tiny2d_engine.cc` (1743 lines) — full read of
  the diff plus close reading of material mixing, circle contact, rectangle–circle contact,
  circle-circle time of impact, and both `Update` overloads.
- All three new labs: `force_lab_*`, `contact_lab_*`, `impact_lab_*` (models, headers, step
  functions; simulation UI skimmed).
- `CMakeLists.txt`, `CMakePresets.json`, `.github/workflows/ci.yml`, `Sandbox/main.cc`,
  `Sandbox/simulations.h`.
- Test suites: `Engine/tiny2d_engine_test.cc` and the seven Sandbox suites (test-function
  inventory plus targeted reads).

### Current size

| Area | Lines |
| --- | --- |
| Engine (impl + header) | 1,931 |
| Engine tests | 1,402 |
| Sandbox models (impl + headers) | 4,620 |
| Sandbox simulations + `main.cc` | 4,296 |
| Sandbox tests | 4,334 |
| **Test-to-production ratio** | **≈ 0.55 : 1** |

---

## 2. Verification actually performed

Both gates were run locally against the working tree.

```
powershell -File tools/verify.ps1 -Profile Fast    → PASSED
powershell -File tools/verify.ps1 -Profile Full    → PASSED
```

| Check | Result |
| --- | --- |
| clang-format `--dry-run --Werror` (tracked + untracked `.cc`/`.h`) | Clean |
| Debug build + `ctest --preset test-debug` | **8/8 passed** (8.97 s) |
| Release build + `ctest --preset test-release` | **8/8 passed** (1.78 s) |
| AddressSanitizer build + `ctest --preset test-asan` | Passed (inside the VS2022 dev shell) |
| clang-tidy preset build (`WarningsAsErrors: '*'`) | Clean |
| Engine-only build, `TINY2D_WARNINGS_AS_ERRORS=ON` | Clean |
| `git diff --check` / `git diff --cached --check` | Clean |

Suites: `Tiny2DEngineTests`, `Tiny2DSimulationTests`, `Tiny2DRotationPendulumTests`,
`Tiny2DRollingDiskTests`, `Tiny2DLorentzParticleTests`, `Tiny2DForceLabTests`,
`Tiny2DContactLabTests`, `Tiny2DImpactLabTests`.

> Note: running `ctest --preset test-asan` from a plain shell fails with `0xc0000135`
> (DLL not found). This is the documented dev-shell requirement in `AGENTS.md`, not a
> project defect — `verify.ps1` enters the shell itself and the ASan suite passes there.

**Not verified:** the new `headless` CI job (Ubuntu / GCC / Clang) has never executed, because
the changeset is uncommitted and the workflow only triggers on push or PR. The
GUI was not launched, so no visual smoke test was performed.

---

## 3. What is strong

**The Engine API additions are unusually well specified.** Every new symbol in
`tiny2d_engine.h` documents its unit system, axis convention, sign convention, valid range,
lifetime, and failure mode. `CollisionMaterial` uses an all-or-nothing `{-1,-1,-1}` sentinel
whose partial-specification case is explicitly rejected with a readable message
(`tiny2d_engine.cc:78`) rather than silently half-inheriting. `Rectangle::material` is
deliberately placed last so existing aggregate initializers keep their field mapping — a
thoughtful compatibility detail.

**Failure atomicity is maintained throughout.** The mixed `Update` runs a complete
validation pass over both vectors before mutating anything (`tiny2d_engine.cc:1541`), and all
three new lab `Step*` functions integrate on a copy and commit only after re-validating. The
documented "invalid input leaves both vectors unchanged" contract is genuinely upheld.

**The CCD design is disciplined.** Conditional activation via `HasMissedCircleImpact` means the
existing discrete result is preserved unless a sweep actually would be missed; the substep loop
is bounded by `circles.size() * kSolverIterations` so it cannot spin; and the early-outs in
`FindCircleTimeOfImpact` (`c < 0` already overlapping, `b >= 0` separating) correctly hand those
cases back to the discrete solver. One subtle detail is right that is easy to get wrong:
`HasMissedCircleImpact` sweeps from the *start* position using the *post-integration* velocity,
which is exactly the segment semi-implicit Euler traverses.

**V17's dual-lane design is the right way to demonstrate the feature.** Running
`discrete_circles` and `ccd_circles` through the same production `Update` with only the CCD flag
differing turns "does CCD work?" into a measurable, testable comparison with an analytical
reference (`ImpactLabDerived` carries entry/exit time, expected post-impact state, and per-lane
error terms). This directly satisfies ROADMAP §8.

**Test coverage maps onto the stated acceptance criteria.** 18 new Engine tests plus three new
lab suites cover inertia formulas, torque sign, damping, zero-`delta_time` load clearing,
order symmetry, material mixing, low-speed CCD/discrete equivalence, multi-impact determinism,
and 60-second determinism runs. `TestLowSpeedCircleCcdMatchesDiscreteUpdate` is the key
regression that protects the "preserve existing behavior" promise.

**Infrastructure improved in the right places.** The product version is now single-sourced from
`vcpkg.json` through CMake into the window title, removing the drift I flagged previously. The
new `headless` CI job builds the Engine and all Sandbox models under GCC *and* Clang with
`-Wall -Wextra -Wpedantic -Werror` — that is the single highest-value addition in this
changeset, because it turns the "Engine must not depend on SDL/ImGui" rule into a
cross-compiler, cross-platform gate.

---

## 4. Findings

Ordered by impact. None of these block the gates; all eight suites pass.

### 4.1 `ROADMAP.md` now contradicts the code it describes — *process risk*

`ROADMAP.md` was committed at `724b466`, before V15–V17 were written, and has not been updated.
It currently states as **current limitations** exactly what this changeset delivered:

| ROADMAP §2 "limitation" | Delivered by |
| --- | --- |
| "no general external-force or applied-torque interface" | V15 (`AddForceAtPoint`, `AddTorque`) |
| "no native circle collision body" | V16 (`Circle`) |
| "no per-body material properties" | V16 (`CollisionMaterial`) |
| "no continuous collision detection for high-speed motion" | V17 (circle-circle CCD) |

§2 still says "four independent experiment families"; there are now seven. §17 "Recommended
Next Action" still recommends **V15 ForceLab** as the next proposal.

This matters more than ordinary doc staleness. `AGENTS.md` instructs the autonomous loop to
"Prefer an unfinished approved requirement, then a repository roadmap item" — so the next
`持续更新` trigger reading this file will conclude that V15 is the next thing to build.

**Suggested fix:** move V15/V16/V17 from "planned" to a "Delivered" section, refresh §2's
baseline and limitation list, and repoint §17 at V18 AtwoodLab. Consider adding a line to
`AGENTS.md` requiring the roadmap to be updated in the same iteration that lands a roadmap item,
so the two can never disagree again.

### 4.2 `README.md` and `docs/DEVELOPER_GUIDE.zh-CN.md` are byte-identical, and both are stale

`cmp` reports the two 46,801-byte files as identical. Meanwhile the *new* `CONTRIBUTING.md`
section states the intended policy:

> 根目录 `README.md` 保持简洁；详细中文实现说明位于 `docs/DEVELOPER_GUIDE.zh-CN.md`。

The policy was documented but the split was never performed — the guide was **copied, not
moved**. Two 46 KB files with no differences will diverge on the first edit to either.

Both copies are also still at V14: `README.md` line 3 reads `当前开发版本：V14（未发布）`, and the
overview says the project "提供四个独立入口" and names only V9, V11, V13, and V14. There is no
mention of ForceLab, ContactLab, or ImpactLab anywhere in either file.

**Suggested fix:** truncate `README.md` to the concise overview the policy calls for (what the
project is, how to build, what the seven labs are, links to `docs/`, `ROADMAP.md`, `CHANGELOG.md`),
keep the long-form content only in `docs/DEVELOPER_GUIDE.zh-CN.md`, and update that guide for
V15–V17 — most importantly the Engine chapters, which no longer describe circles, materials,
applied loads, or CCD.

### 4.3 The model-selection screen very likely overflows the 800 px window — *needs the mandated visual smoke test*

`ChooseModel` (`Sandbox/main.cc:57`) creates a full-display window with
`ImGuiWindowFlags_NoDecoration`, which I confirmed expands to include
`ImGuiWindowFlags_NoScrollbar` (`imgui.h:1093`). The list grew from four labs to seven.

Rough layout arithmetic at the default 1200×800 with 16 px Segoe UI and default style
(`ItemSpacing.y = 4`, button height 36, `Spacing()` between entries):

```
header (cursor 25 + title + subtitle + spacing + separator)  ≈  90 px
7 lab entries × (title 20 + wrapped description ~36 + button 40 + spacing 8) ≈ 672 px
Quit button (42 + 4)                                          ≈  46 px
                                                          total ≈ 808 px  vs. 800 px available
```

`NoScrollWithMouse` is *not* set, so the wheel still scrolls and nothing is permanently
unreachable — but there is no visible scrollbar, so a clipped **Quit** button would look like a
missing control. The four-lab version fit comfortably at roughly 520 px, so this is new.

I could not confirm this by running the GUI. `AGENTS.md` requires a visual smoke test for UI
changes, and this is precisely the change that needs one.

**Suggested fixes, cheapest first:** drop `ImGuiWindowFlags_NoScrollbar` from the flag set (keep
the rest of `NoDecoration` by spelling out the three other flags); or put the lab list in a
scrolling `BeginChild` with the Quit button pinned below it; or compact each entry to a single
description line and shrink `kModelButtonHeight`.

### 4.4 `FindCircleTimeOfImpact` uses the cancellation-prone quadratic root

`tiny2d_engine.cc:681`:

```cpp
const double time = (-b - std::sqrt(discriminant)) / a;
```

With `b < 0` guaranteed by the guard above it, `-b > 0` and `sqrt(b*b - a*c) <= |b|` whenever
`c >= 0`. The subtraction therefore cancels precisely when `a*c << b*b` — that is, when `c` is
near zero, which is the **near-touching / grazing** configuration that the
`kSupportedGrazing` preset exists to exercise.

The algebraically identical but numerically stable form avoids the cancellation entirely:

```cpp
const double time = c / (-b + std::sqrt(discriminant));
```

The `AddTrigonometricVelocityRoots` helper in `lorentz_particle_model.cc:263` already uses
exactly this stabilised-root technique, so the codebase knows the pattern — it just was not
applied here. Given that V17's headline claim is *deterministic* time of impact, this is worth
tightening even though the current tests pass.

### 4.5 ForceLab re-derives Engine-internal integration order to account for dissipated energy

`StepForceLab` (`force_lab_model.cc`) reconstructs, in the Sandbox, the exact sequence the
Engine will perform inside `Update`: apply `applied_force / mass * dt`, then multiply by
`exp(-rate * dt)`, then do the same for angular. It uses that reconstruction to compute
`damping_loss` before calling `Update`.

The arithmetic is correct today — I checked it line-by-line against `integrate` at
`tiny2d_engine.cc:1550`, including the fact that ForceLab passes `electric_field = {}` and
`gravity = 0`, which is what makes the omission of field acceleration valid.

The risk is coupling, not correctness: the Sandbox now depends on private ordering inside
`Update`. If the Engine ever reorders damping relative to force integration, or switches to a
different damping discretisation, this accounting silently drifts, and the 0.5 % test tolerance
may not catch a small systematic error.

**Suggested fix:** either have `Update` optionally report per-body dissipated energy (an output
parameter or a small result struct), or compute the loss from the *observed* velocity change
across the call rather than a predicted one. The second option is a local change and removes
the coupling entirely.

### 4.6 The CCD path allocates two vectors every step

`tiny2d_engine.cc:1598`:

```cpp
std::vector<Rectangle> integrated_rectangles = rectangles;
std::vector<Circle> integrated_circles = circles;
```

These copies happen on **every** `Update` call whenever `enable_circle_circle_ccd && delta_time > 0
&& circles.size() > 1` — including the overwhelmingly common case where no impact is missed and
the copies are discarded. At ImpactLab's 480 Hz that is two heap allocations per step in the
Engine hot path.

The existing `ponytail:` comment at `tiny2d_engine.cc:1632` acknowledges the O(n²) scan and the
event cap but not the allocation. For ImpactLab's two-circle scenes the cost is irrelevant;
it becomes relevant the moment a denser experiment enables the flag.

**Suggested fix:** hoist the scratch buffers to caller-provided storage, or add a cheap
conservative pre-test (max-speed × `delta_time` versus minimum pair separation) that skips the
speculative integration entirely in the common case.

### 4.7 `std::size_t` used without including `<cstddef>`

`Sandbox/contact_lab_model.cc` (2 uses) and `Sandbox/impact_lab_model.cc` (1 use) reference
`std::size_t` but include only `<algorithm>`, `<array>`, `<cmath>`, `<stdexcept>`, `<utility>`,
and `<limits>`. `Engine/tiny2d_engine.cc` includes `<cstddef>` correctly, so the convention
exists.

This will almost certainly still compile on libstdc++ and libc++ through transitive includes,
so it is hygiene rather than a live break — but it is exactly the class of latent portability
issue the new headless job was added to surface, and it costs one line each to fix.

### 4.8 The new headless CI job carries unverified assumptions

The job configures with `-G Ninja` and `-DCMAKE_CXX_COMPILER=clang++` on `ubuntu-24.04`. I have
no way to confirm from here that both `ninja` and `clang++` are present on that runner image by
default, and the job has never run because the change is uncommitted and the workflow triggers
only on push/PR.

**Suggested fix:** add an explicit `sudo apt-get install -y ninja-build clang` step (cheap
insurance, a few seconds), or fall back to `-G "Unix Makefiles"` which needs nothing extra.
Either way, push the branch so the job actually executes before this work is considered done —
`codex/**` is now in the trigger list, so a push is sufficient.

### 4.9 The V16 → V17 gate is satisfied, but not where the ROADMAP says

ROADMAP §8 states:

> V17 SHOULD be implemented only after a deterministic V16 regression test demonstrates
> tunneling at a physically supported speed.

`Sandbox/contact_lab_test.cc` contains no tunneling test. The demonstration lives in
`Engine/tiny2d_engine_test.cc` (`TestSupportedCircleSweepDoesNotTunnel`,
`TestFastCircleTravelsBeyondDiameterWithoutTunneling`) and in V17's own suite
(`TestSupportedGrazingTunnelingAndCcd`).

The *substance* of the gate is met — `kSupportedGrazing` runs at 9.8–10 m/s, inside ContactLab's
supported ±10 m/s range, and `ImpactLabDerived::discrete_tunneled` is asserted true. Only the
placement differs from what the roadmap prescribed. Worth a sentence in the roadmap update so
the traceability is on the record rather than implied.

### 4.10 Minor observations

- **Three `ponytail:` tags remain** (`tiny2d_engine.cc:1632`, `incline_spring_model.cc:273`,
  `main.cc:191`). Down from six. Nothing in the repo defines the tag; `NOTE:` would be
  self-explanatory to a future reader.
- **Conditional CCD is inherently path-discontinuous.** A parameter nudge that flips
  `HasMissedCircleImpact` switches the whole world between the discrete and swept solvers, which
  can produce a visible jump in results. This is intrinsic to the design (and arguably the
  thing V17 exists to show), but ImpactLab's monitor should state plainly which lane produced
  the numbers on screen so a user never mistakes a solver switch for physics.
- **`vcpkg.json` remains at `2.0.0`**, correctly, per the new `CONTRIBUTING.md` policy that V-numbers
  are experiment generations rather than product versions. The window title now reads
  `Tiny2D Engine 2.0.0 | V17 Development`, which is consistent and unambiguous.
- **`CHANGELOG.md` lists V11–V14 under `[Unreleased]`** alongside V15–V17. Since V11–V14 are
  committed and pushed while V15–V17 are not, a reader cannot tell from the changelog what is
  actually on a branch. Splitting released-to-branch from working-tree entries would help.

---

## 5. Recommended next steps

In priority order:

1. **Update `ROADMAP.md`** (§2 baseline, §2 limitations, §17 next action) and note the §8 gate
   evidence. This is the highest-leverage fix because the autonomous loop reads it.
2. **Perform the visual smoke test** on the seven-entry selection screen at 1200×800 and fix the
   overflow if confirmed (§4.3). `AGENTS.md` requires this for UI changes regardless.
3. **Resolve the README / DEVELOPER_GUIDE duplication** and bring the guide up to V17 (§4.2).
4. **Push the branch so the headless GCC/Clang job runs**, after hardening its tool assumptions
   (§4.8) and adding the two missing `<cstddef>` includes (§4.7).
5. **Stabilise the time-of-impact root** (§4.4) — a one-line change directly on V17's critical path.
6. **Decouple ForceLab's dissipation accounting from Engine internals** (§4.5).
7. **Remove the per-step CCD allocations** (§4.6) before any denser experiment enables the flag.

### Looking further ahead

The Engine is at 1,931 lines and now carries two shape types, materials, applied loads, damping,
and an optional CCD path in a single translation unit with a single `Update` that takes eleven
parameters. V18 (rope/pulley constraints) and V20 (persistent contact stability) will both push
on this. Before V18 lands I would suggest:

- **Split `tiny2d_engine.cc`** along its natural seams — geometry/contact generation, the
  impulse solver, and integration/stepping — while keeping the public header as-is. This is a
  pure file-organisation change with no behavioural risk, and it is much cheaper now than after
  a constraint solver is threaded through it.
- **Introduce a parameter struct for `Update`.** Eleven positional parameters with eight defaults
  is already hard to read at call sites, and V18's constraint iteration count plus V20's
  sleeping thresholds will add more. A `WorldStepSettings` aggregate would keep the existing
  overloads working as thin wrappers.
- **Consider the roadmap's own advice on scope.** ROADMAP §3.4 says "prefer evidence over
  infrastructure"; the V15–V17 sequence honoured that well by pairing each Engine capability
  with an experiment that measures it. V18 should keep that pairing — a rope constraint with no
  analytically checkable Atwood result would be the first capability in this project not
  pinned down by a measurable acceptance criterion.

---

## 6. Summary

This is a large, careful, well-tested changeset. Three new experiment families and a
substantially extended Engine landed with the full verification contract green — formatting,
Debug, Release, ASan, clang-tidy, engine-only, and eight test suites — and the new headless
GCC/Clang CI job is a genuine step up in rigour.

**No correctness defect was found in the new physics.** The findings above are, in order: one
stale planning document that will actively misdirect the automated workflow, one documentation
duplication that was specified but not executed, one probable UI overflow that the project's own
process already requires a smoke test to catch, one numerical-robustness improvement on V17's
critical path, and a handful of coupling, allocation, and hygiene items.

The code is in good shape. The documents that describe it are the part that has fallen behind.
