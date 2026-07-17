# Tiny2D Engine

一个面向高中物理场景的二维刚体模拟实验项目。V9 是首个正式版本，重点是
可配置的物理量、清晰的实时监测，以及可重复验证的数值行为。

## 技术

- C++17
- CMake
- SDL2
- Dear ImGui

## 当前功能

- 矩形刚体、重力、碰撞、摩擦与恢复系数
- 斜面、底板、坡底双向过渡与左侧弹簧
- 均匀电场、带电物块及离开约束面的安全检测
- SI 单位与像素模拟自动换算
- 可暂停、按时间查看的位置、速度和加速度监测

## 构建与运行

```powershell
cmake --preset msvc-x64
cmake --build --preset debug
./build/Debug/Sandbox.exe
```

## 测试

测试不依赖 Debug `assert`，因此 Debug 和 Release 都会执行完整判定。

```powershell
cmake --build --preset debug
ctest --preset test-debug

cmake --build --preset release
ctest --preset test-release
```

测试覆盖物理核心、非法数值、SI 换算、电场、斜面转场、弹簧、长时间运行及
固定随机种子的性质检查。物理核心遇到非有限值或不合法状态时会抛出
`std::invalid_argument`，并在修改物体状态前终止本次更新。

## 当前状态

V9 — first formal release
