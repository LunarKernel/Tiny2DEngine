# Tiny2DEngine 贡献指南

## 开发环境

- Visual Studio 2022：在安装器中导入仓库根目录的 `.vsconfig`。
- CMake 3.21 或更高版本。
- 从 **Developer PowerShell for VS 2022** 运行下列命令；它会提供 MSVC、
  Ninja 和 `VCPKG_ROOT`。

## 构建与测试

```powershell
cmake --preset msvc-x64
cmake --build --preset debug
ctest --preset test-debug
```

提交合并请求前还应运行：

```powershell
cmake --build --preset release
ctest --preset test-release
cmake --preset msvc-x64-tidy
cmake --build --preset tidy
git diff --check
```

涉及内存、生命周期或容器边界的修改还应运行 `asan` 和 `test-asan` 预设。

## 代码规范

- 使用 C++17 和仓库根目录的 `.clang-format`、`.clang-tidy`。
- 可复用物理代码放在 `Engine/`；界面和具体实验放在 `Sandbox/`。
- 公共物理 API 必须说明单位、坐标轴、正方向和有效范围。
- 随机测试固定种子，浮点比较明确写出容差。
- 不提交 `build/`、`CMakeUserPresets.json` 或编辑器缓存。

## 合并请求

一次合并请求只解决一个清晰问题。说明行为变化、验证命令，以及物理量纲、
符号约定或 UI 是否发生变化。不要通过关闭测试或警告来让 CI 通过。
