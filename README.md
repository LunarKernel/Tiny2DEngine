# Tiny2D Engine

一个面向高中与大学普通物理场景的二维模拟实验项目。V10 是正式版本 2.0，
新增转动模型，同时完整保留 V9 斜面实验。

## 技术

- C++17
- CMake
- SDL2
- Dear ImGui

## V10 新增内容

- 新增模型选择页，可独立进入 V9 或 V10
- 新增 PivotLab：均匀杆与可调带电配重组成的物理摆
- 可设置杆长、杆与配重质量、配重位置、初始角度和初始角速度
- 支持重力力矩、均匀电场力矩和可选线性转动阻尼
- 实时显示转动惯量、角运动、各项力矩、能量和小角度周期
- 支持 Space 暂停、历史时刻查询以及电场/重力场方向箭头
- 独立验证公式、能量、阻尼、非法输入和长期运行稳定性

## V9 保留功能

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

测试覆盖物理核心、非法数值、SI 换算、电场、斜面转场、弹簧、物理摆公式、
能量与阻尼、长时间运行及固定随机种子的性质检查。物理核心遇到非有限值或
不合法状态时会在继续更新前停止。

## 当前状态

V10 / 2.0 — formal release。V9 可继续从启动页独立运行。
