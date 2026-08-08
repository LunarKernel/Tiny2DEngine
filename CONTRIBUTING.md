# Contributing to Tiny2DEngine

## Development environment

- Visual Studio 2022: import `.vsconfig` from the repository root in the
  installer.
- CMake 3.24 or newer.
- Run the commands below from **Developer PowerShell for VS 2022**; it
  provides MSVC, Ninja, and `VCPKG_ROOT`.

## Build and test

```powershell
cmake --preset msvc-x64
cmake --build --preset debug
ctest --preset test-debug
```

Before opening a pull request, also run:

```powershell
cmake --build --preset release
ctest --preset test-release
cmake --preset msvc-x64-tidy
cmake --build --preset tidy
git diff --check
```

Changes that touch memory, lifetimes, or container boundaries should also run
the `asan` and `test-asan` presets.

Alternatively, use the unified verification entry point from a plain
PowerShell; the script locates and loads VS2022 itself:

```powershell
# During development: formatting, Debug build, Debug tests, diff check
powershell -File tools/verify.ps1 -Profile Fast

# Before committing: adds Release, ASan, clang-tidy, and the engine-only build
powershell -File tools/verify.ps1 -Profile Full
```

## Automated iteration process

1. Record the current Git status and any pre-existing changes that must be
   protected.
2. Write down the feature goal, non-goals, acceptance criteria, compatibility
   requirements, and test plan.
3. Have a read-only reviewer approve the plan; do not start implementing
   while a blocking finding remains.
4. After a minimal implementation, run the `Fast` verification; run the
   `Full` verification on the final candidate.
5. Stage with an explicit file list — never `git add .` — and have a
   read-only reviewer inspect the staged diff.
6. Commit only when the task explicitly authorizes it; push, PR, merge, and
   release each require their own authorization.

Any change to a reviewed staged diff invalidates the review and the
verification; the affected stages must run again.

## Versions and documentation

- `version-semver` in `vcpkg.json` is the single source of the product
  version; CMake and the application title derive from it.
- Identifiers such as V9 and V14 name experiment generations, not semantic
  product versions.
- On a release, `CHANGELOG.md` and the Git tag `v<version-semver>` must match
  the product version.
- Keep the root `README.md` concise; the detailed implementation guide lives
  in `docs/DEVELOPER_GUIDE.md`.
- The project is English-only: code, comments, documentation, commit
  messages, and UI text.

## Code standards

- Use C++17 with the repository's `.clang-format` and `.clang-tidy`.
- Reusable physics code belongs in `Engine/`; UI and concrete experiments
  belong in `Sandbox/`.
- Public physics APIs must document units, axes, positive directions, and
  valid ranges.
- Randomized tests use fixed seeds; floating-point comparisons state their
  tolerances explicitly.
- Do not commit `build/`, `CMakeUserPresets.json`, or editor caches.

## Pull requests

One pull request solves one clearly stated problem. Describe the behavior
change, the verification commands, and whether physical units, sign
conventions, or the UI changed. Never make CI pass by disabling tests or
silencing warnings.
