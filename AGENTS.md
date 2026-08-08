# Tiny2DEngine repository guidance

## Scope

- Use C++17, CMake, SDL2, ImGui, and the existing Google-style `.clang-format`.
- Keep reusable physics code in `Engine/` and simulation-specific code in
  `Sandbox/`. The engine's public API is `Engine/tiny2d_engine.h`; its
  implementation is split across `Engine/internal/` (body_math, validation,
  contacts, solver) plus the step orchestration in `tiny2d_engine.cc`.
  `tiny2d::internal` symbols are implementation details with no stability
  guarantee.
- In `Sandbox/`, each lab is a UI-free physics model (`*_model.h/.cc`, the
  Config/State/Derived/Step pattern) plus a UI file (`*_simulation.cc`) built
  on the shared shell `Sandbox/app/lab_shell.h`. The selection menu renders
  from `Sandbox/app/lab_registry.h`; adding a lab means a model, a traits
  declaration or custom frame on the shell, a registry entry, CMake targets,
  and a test suite.
- Keep the dependency direction
  `main -> registry -> lab UI -> shell -> model -> Engine`; `Engine` must not
  depend on SDL UI, ImGui, or a specific simulation (the headless CI job
  enforces this).
- Preserve the validated behavior of every delivered experiment generation
  (V9 through V17) unless the task explicitly changes it. The rectangle-only
  `Update` must stay trajectory-identical to the mixed `Update` with no
  circles; `TestLegacyUpdateMatchesMixedUpdateTrajectories` guards this and
  must not be weakened.
- Do not add large dependencies, speculative abstractions, or unrelated engine
  features.

## Workflow

- Read affected code and tests, then run `git status -sb` before editing.
- Preserve user changes, especially an already modified `README.md`.
- Diagnose root causes before fixes and keep one writer per file area.
- Treat units and sign conventions as part of the physics model; state any
  changed convention.
- Document units, axes, angle direction, and valid ranges for public physics
  APIs. Use fixed seeds for randomized tests and explicit tolerances for
  floating-point checks.
- New tests must link production targets; do not add new tests that include a
  `.cc` implementation file. Use the shared `CHECK` harness from
  `tests/test_support.h` instead of a per-file copy.
- Keep the root `README.md` a concise overview; long-form implementation
  documentation belongs in `docs/DEVELOPER_GUIDE.md`. The project
  is English-only: code, comments, documentation, and UI text.
- An iteration that delivers a `ROADMAP.md` item must update `ROADMAP.md`
  (baseline, limitations, and next action) and `CHANGELOG.md` in the same
  iteration, so the planning documents never contradict the code.
- Never silence warnings, disable tests, or reduce test coverage just to pass a
  check.
- Do not commit, tag, push, or publish unless the user explicitly asks.

## Continuous update trigger

A user message triggers this section only when, after trimming surrounding
whitespace, its entire content is `continuous update`. Mentions or
quotations do not trigger it. The standalone command explicitly authorizes
one autonomous version iteration, including the branch, commit, and push
actions below.

- Run the baseline first. Treat every file already dirty at baseline as
  whole-file protected: do not edit, stage, or partially stage it. Choose a
  different feature if necessary; stop as a hard blocker if no meaningful
  feature can avoid the protected files.
- Start from the current verified HEAD and create a new
  `codex/<next-version>-<slug>` branch for every trigger. Derive the next
  version from the highest actual simulation version in the project, not from
  arbitrary version-like text. Never reuse or overwrite an existing branch;
  add a unique suffix when needed.
- Prefer an unfinished approved requirement, then a repository roadmap item,
  otherwise choose the smallest meaningful physics-focused feature. Complete
  exactly one version per trigger using the automated iteration below.
- Resolve ordinary design choices from repository evidence without asking for
  confirmation. Progress updates are allowed, but must not pause the work.
- A failed gate returns to the relevant earlier stage for repair. Any change
  after cached-diff review requires all applicable Fast, Full, and UI checks,
  explicit staging, and the independent cached-diff review to run again.
- Before committing, require Git author `LunarKernel` with
  `293355731+LunarKernel@users.noreply.github.com`, authenticated GitHub account
  `LunarKernel`, and `origin` belonging to `LunarKernel/Tiny2DEngine`.
- The only authorized publication is a normal, non-force
  `git push -u origin <new-branch>` for that iteration. The trigger does not
  authorize pushing a default branch, rewriting history, deleting branches,
  opening or merging a PR, tagging, or creating a GitHub Release.
- Stop and report without requesting confirmation only when credentials or
  permissions are missing, a destructive action would be required, protected
  files cannot be preserved, or safe repair attempts cannot pass a gate.

The trigger finishes when the new branch is successfully pushed. Starting
another version requires another standalone trigger message.

## Automated iteration

When a task explicitly requests an autonomous iteration, use this sequence:

1. **Baseline:** inspect `git status -sb`, affected production code, and tests.
   Record every pre-existing dirty file as protected; never stage it.
2. **Feature contract:** state the goal, user-visible behavior, acceptance
   checks, non-goals, affected layer, compatibility requirements, and test
   plan. Physics work must also state units, axes, signs, and valid ranges.
3. **Specification review:** ask an independent read-only reviewer to return
   `APPROVED`, `BLOCKING`, and `NON-BLOCKING` findings. Do not implement while
   a blocking finding remains.
4. **Implementation:** assign one writer per file area, make only contracted
   changes, and run `powershell -File tools/verify.ps1 -Profile Fast`.
5. **Final validation:** run the `Full` profile for C++ or build-system work.
   UI changes additionally require a visual smoke test.
6. **Pre-commit review:** stage only an explicit file list, never `git add .`
   or `git add -A`. Have an independent read-only reviewer inspect the cached
   diff. Any change to the reviewed cached diff invalidates the review and
   validation.
7. **Commit:** only when the task authorized it. Recheck the cached file list,
   `git diff --cached --check`, author identity, and protected files before
   committing. Push/PR/release actions require their own explicit scope; wait
   for existing GitHub CI when a PR is created.

The workflow stops and returns to the relevant earlier stage on any failed
gate. It must never use stash, reset, clean, or destructive checkout to hide
unrelated user changes.

## Verification

- `powershell -File tools/verify.ps1 -Profile Fast` bundles the routine gate
  (clang-format `--Werror` over tracked and untracked sources, Debug build,
  `ctest`, whitespace checks); the `Full` profile adds Release, ASan,
  clang-tidy, and the engine-only warnings-as-errors build. Run formatting in
  write mode on touched files before the gate, since the check is strict.
- Configure from a VS2022 Developer terminal with `cmake --preset msvc-x64`.
- Build and test with `cmake --build --preset debug` and
  `ctest --preset test-debug`.
- Before a release, also run the `release` and `test-release` presets,
  `git diff --check`, and clang-format in check mode on changed C++ files.
- Use the `asan` and `test-asan` presets for memory-sensitive changes; run ASan
  tests from a VS2022 Developer terminal so its runtime DLL is on `PATH`.
- Run `cmake --build --preset tidy` for substantial C++ changes.
- Physics changes need deterministic tests; UI changes also need a visual smoke
  test.
