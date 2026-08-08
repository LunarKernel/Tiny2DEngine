# Tiny2D Engine

> 产品版本：2.0.0 / 最新实验代：V17（未发布）<br>
> 开发语言：C++17 · 图形：SDL2 · 界面：Dear ImGui · 构建：CMake + vcpkg

Tiny2D Engine 是一个面向二维刚体和高中、大学普通物理题目的小型物理
实验室。可复用的物理内核位于 `Engine/`，七个独立实验位于 `Sandbox/`。
所有实验都提供 SI 单位的参数设置、确定性定步长积分、实时监测、
历史时刻查询，以及可与解析解对照的验证量。

两套编号并存：`2.0.0` 是语义化产品版本（单一来源是 `vcpkg.json`）；
`V9` 到 `V17` 是物理实验的迭代代号，不是产品版本号。

## 实验列表

| 实验 | 主题 |
| --- | --- |
| V9 Incline Laboratory | 斜面、地面、弹簧、摩擦、均匀电场 |
| V11 RollLab | 圆盘/圆环由滑动进入纯滚动 |
| V13 Gravito-Orbit | 均匀电场、磁场与重力中的带电粒子（含 V12 预设） |
| V14 Driven PivotLab | 带阻尼与周期驱动的物理摆（含 V10 行为） |
| V15 ForceLab | 中心或偏心弹簧驱动的矩形刚体平动与转动耦合 |
| V16 ContactLab | 原生圆形刚体、材质化碰撞与滚动接触 |
| V17 ImpactLab | 圆-圆连续碰撞检测与离散路径的对照实验 |

## 构建与测试

在 VS2022 开发者终端中：

```
cmake --preset msvc-x64
cmake --build --preset debug
ctest --preset test-debug
```

或直接运行完整验证门禁：

```
powershell -File tools/verify.ps1 -Profile Fast   # 格式 + Debug + 测试
powershell -File tools/verify.ps1 -Profile Full   # 追加 Release/ASan/tidy
```

## 代码结构

- `Engine/` — 可复用物理内核（矩形与圆形刚体、SAT 与圆接触、冲量求解、
  可选圆-圆 CCD、半隐式欧拉积分）。实现按职责拆分在
  `Engine/internal/`（body_math、validation、contacts、solver），
  公共 API 只有 `tiny2d_engine.h`。Engine 不依赖 SDL/ImGui。
- `Sandbox/` — 实验层。每个实验一个纯物理模型
  （`*_model.h/.cc`，Config/State/Derived/Step 四件套，可独立测试）
  和一个界面文件（`*_simulation.cc`）。共享的应用外壳位于
  `Sandbox/app/lab_shell.h`（事件循环、定步长推进、暂停/回看），
  实验清单位于 `Sandbox/app/lab_registry.h`。
- `tests/` — 各测试套件共用的断言支撑头文件。

依赖方向固定为 `main → registry → 实验界面 → shell → 模型 → Engine`。

## 更多文档

- 详细中文实现说明：[docs/DEVELOPER_GUIDE.zh-CN.md](docs/DEVELOPER_GUIDE.zh-CN.md)
- 长期路线图：[ROADMAP.md](ROADMAP.md)
- 变更记录：[CHANGELOG.md](CHANGELOG.md)
- 贡献与验证流程：[CONTRIBUTING.md](CONTRIBUTING.md)
