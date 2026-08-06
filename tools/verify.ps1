[CmdletBinding()]
param(
  [ValidateSet("Fast", "Full")]
  [string]$Profile = "Fast"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Invoke-Native {
  param(
    [Parameter(Mandatory)] [string]$FilePath,
    [Parameter(Mandatory)] [string[]]$ArgumentList
  )

  Write-Host "> $FilePath $($ArgumentList -join ' ')" -ForegroundColor Cyan
  & $FilePath @ArgumentList
  if ($LASTEXITCODE -ne 0) {
    throw "$FilePath failed with exit code $LASTEXITCODE."
  }
}

function Get-NativeLines {
  param(
    [Parameter(Mandatory)] [string]$FilePath,
    [Parameter(Mandatory)] [string[]]$ArgumentList
  )

  $output = @(& $FilePath @ArgumentList)
  if ($LASTEXITCODE -ne 0) {
    throw "$FilePath failed with exit code $LASTEXITCODE."
  }
  return $output
}

function Enter-Vs2022Environment {
  $programFilesX86 = [Environment]::GetEnvironmentVariable("ProgramFiles(x86)")
  $vswhere = Join-Path $programFilesX86 `
      "Microsoft Visual Studio\Installer\vswhere.exe"
  if (-not (Test-Path -LiteralPath $vswhere)) {
    throw "vswhere.exe was not found. Install Visual Studio 2022 first."
  }

  $vsInstall = & $vswhere -latest -version "[17.0,18.0)" -products * `
      -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
      -property installationPath | Select-Object -First 1
  if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($vsInstall)) {
    throw "Visual Studio 2022 with the C++ toolchain was not found."
  }

  $devShell = Join-Path $vsInstall "Common7\Tools\Launch-VsDevShell.ps1"
  & $devShell -Arch amd64 -HostArch amd64 -SkipAutomaticLocation
  if (-not $?) {
    throw "Failed to initialize the Visual Studio 2022 developer shell."
  }

  if ($env:CI -eq "true" -and $env:VCPKG_INSTALLATION_ROOT) {
    $env:VCPKG_ROOT = $env:VCPKG_INSTALLATION_ROOT
  } else {
    $env:VCPKG_ROOT = Join-Path $vsInstall "VC\vcpkg"
  }
  $env:VCPKG_VISUAL_STUDIO_PATH = $vsInstall
  $script:VsInstall = (Resolve-Path -LiteralPath $vsInstall).Path

  $script:ClangFormat =
      Join-Path $vsInstall "VC\Tools\Llvm\x64\bin\clang-format.exe"
  if (-not (Test-Path -LiteralPath $script:ClangFormat)) {
    throw "clang-format was not found. Install the Visual Studio LLVM tools."
  }
}

function Get-CachedCompiler {
  param([Parameter(Mandatory)] [string]$BuildDirectory)

  $cachePath = Join-Path $BuildDirectory "CMakeCache.txt"
  if (-not (Test-Path -LiteralPath $cachePath)) {
    return $null
  }

  $match = Select-String -LiteralPath $cachePath `
      -Pattern "^CMAKE_CXX_COMPILER:FILEPATH=(.+)$" | Select-Object -First 1
  if (-not $match) {
    return $null
  }

  return [IO.Path]::GetFullPath($match.Matches[0].Groups[1].Value)
}

function Test-CMakeCacheUsesVs2022 {
  param([Parameter(Mandatory)] [string]$BuildDirectory)

  $cachePath = Join-Path $BuildDirectory "CMakeCache.txt"
  if (-not (Test-Path -LiteralPath $cachePath)) {
    return $true
  }

  $compiler = Get-CachedCompiler $BuildDirectory
  $vsPrefix = $script:VsInstall.TrimEnd("\") + "\"
  return $null -ne $compiler -and
      (Test-Path -LiteralPath $compiler -PathType Leaf) -and
      $compiler.StartsWith($vsPrefix, [StringComparison]::OrdinalIgnoreCase)
}

function Assert-Vs2022Compiler {
  param([Parameter(Mandatory)] [string]$BuildDirectory)

  $compiler = Get-CachedCompiler $BuildDirectory
  if ($null -eq $compiler) {
    throw "CMAKE_CXX_COMPILER is missing from $BuildDirectory/CMakeCache.txt."
  }

  $vsPrefix = $script:VsInstall.TrimEnd("\") + "\"
  if (-not $compiler.StartsWith($vsPrefix,
      [StringComparison]::OrdinalIgnoreCase)) {
    throw "CMake selected a compiler outside Visual Studio 2022: $compiler"
  }
}

function Invoke-PresetConfigure {
  param(
    [Parameter(Mandatory)] [string]$Preset,
    [Parameter(Mandatory)] [string]$BuildDirectory
  )

  $arguments = @("--preset", $Preset)
  if (-not (Test-CMakeCacheUsesVs2022 $BuildDirectory)) {
    $arguments += "--fresh"
  }
  Invoke-Native "cmake" $arguments
  Assert-Vs2022Compiler $BuildDirectory
}

function Test-Formatting {
  $tracked = Get-NativeLines "git" @("ls-files", "--", "*.cc", "*.h")
  $untracked = Get-NativeLines "git" `
      @("ls-files", "--others", "--exclude-standard", "--", "*.cc", "*.h")
  $files = @($tracked + $untracked) |
      Where-Object { -not [string]::IsNullOrWhiteSpace($_) } |
      Sort-Object -Unique
  if ($files.Count -gt 0) {
    Invoke-Native $script:ClangFormat (@("--dry-run", "--Werror") + $files)
  }
}

function Test-Diffs {
  Invoke-Native "git" @("diff", "--check")
  Invoke-Native "git" @("diff", "--cached", "--check")
}

function Invoke-DebugChecks {
  Invoke-PresetConfigure "msvc-x64" "build"
  Invoke-Native "cmake" @("--build", "--preset", "debug", "--parallel")
  Invoke-Native "ctest" @("--preset", "test-debug", "--output-on-failure")
}

function Invoke-FullChecks {
  Invoke-Native "cmake" @("--build", "--preset", "release", "--parallel")
  Invoke-Native "ctest" @("--preset", "test-release", "--output-on-failure")

  Invoke-PresetConfigure "msvc-x64-asan" "build/asan"
  Invoke-Native "cmake" @("--build", "--preset", "asan", "--parallel")
  Invoke-Native "ctest" @("--preset", "test-asan", "--output-on-failure")

  Invoke-PresetConfigure "msvc-x64-tidy" "build/tidy"
  Invoke-Native "cmake" @("--build", "--preset", "tidy", "--parallel")

  $engineArguments = @(
    "-S", ".",
    "-B", "build/engine-only",
    "-G", "Ninja",
    "-DCMAKE_BUILD_TYPE=Release",
    "-DTINY2D_BUILD_SANDBOX=OFF",
    "-DBUILD_TESTING=OFF",
    "-DTINY2D_WARNINGS_AS_ERRORS=ON"
  )
  if (-not (Test-CMakeCacheUsesVs2022 "build/engine-only")) {
    $engineArguments += "--fresh"
  }
  Invoke-Native "cmake" $engineArguments
  Assert-Vs2022Compiler "build/engine-only"
  Invoke-Native "cmake" @("--build", "build/engine-only", "--parallel")
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$previousLocation = Get-Location
$statusBefore = $null
$statusCaptured = $false

try {
  Set-Location -LiteralPath $repoRoot
  $gitRoot = (Get-NativeLines "git" @("rev-parse", "--show-toplevel") |
      Select-Object -First 1)
  if ((Resolve-Path -LiteralPath $gitRoot).Path -ne
      (Resolve-Path -LiteralPath $repoRoot).Path) {
    throw "tools/verify.ps1 must run inside the Tiny2DEngine repository."
  }

  $statusBefore = @(
    @(Get-NativeLines "git" `
        @("status", "--porcelain=v1", "--untracked-files=all")) |
        Sort-Object
  )
  $statusCaptured = $true

  Enter-Vs2022Environment
  Test-Formatting
  Invoke-DebugChecks
  if ($Profile -eq "Full") {
    Invoke-FullChecks
  }
  Test-Diffs

  Write-Host "$Profile verification passed." -ForegroundColor Green
} finally {
  try {
    if ($statusCaptured) {
      $statusAfter = @(
        @(Get-NativeLines "git" `
            @("status", "--porcelain=v1", "--untracked-files=all")) |
            Sort-Object
      )
      if (Compare-Object $statusBefore $statusAfter) {
        throw "Verification changed the source worktree. Inspect git status."
      }
    }
  } finally {
    Set-Location -LiteralPath $previousLocation.Path
  }
}
