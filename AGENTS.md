# Tiny2DEngine repository guidance

## Scope

- Use C++17, CMake, SDL2, ImGui, and the existing Google-style `.clang-format`.
- Keep reusable physics code in `Engine/` and simulation-specific code in
  `Sandbox/`.
- Keep the dependency direction `Sandbox -> Engine`; `Engine` must not depend on
  SDL UI, ImGui, or a specific simulation.
- Preserve validated V9 and V10 behavior unless the task explicitly changes it.
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
  `.cc` implementation file.
- Never silence warnings, disable tests, or reduce test coverage just to pass a
  check.
- Do not commit, tag, push, or publish unless the user explicitly asks.

## Verification

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
