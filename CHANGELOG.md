# 变更记录

本文件记录本项目所有值得注意的变更。
格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

## [Unreleased]

### 新增

- 仓库根 `README.md`：项目定位、项目索引、三档快速开始、结构说明、文档索引与路线图
- `CONTRIBUTING.md`、`CHANGELOG.md`、`.github/` 下的 CI 与 issue / PR 模板
- `docs/adding-a-project.md`：monorepo 新项目接入规范
- `projects/tianyi25-sim/README.md`：项目级入口文档

### 变更

- **目录重构为 monorepo 布局**：
  - `tianyi_robot_sim_ws/` → `projects/tianyi25-sim/ros2_ws/`
  - `algorithm_template_model/` → `packages/reusable_model/`
- `packages/reusable_model` 改为标准 **src 布局**（`src/reusable_model/`），
  `pyproject.toml` 的 `packages.find` 与 `pytest` 的 `pythonpath` 同步修正
- `pyproject.toml` 描述与包 docstring 去除内部项目代号，改为通用描述

### 修复

- **修复 `tianyi25_sim` 无法构建的问题**：`CMakeLists.txt` 的
  `install(DIRECTORY worlds launch config models urdf …)` 引用了一个并不存在于仓库中的
  `models/` 目录（Git 不追踪空目录，该目录在提交时丢失），导致任何人在干净克隆后
  `colcon build` 都会在安装阶段失败。已补回 `models/.gitkeep`，
  本地实测：缺失时 `Failed <<< tianyi25_sim [exited with code 1]`，补回后 `2 packages finished`。
- **修复 `reusable_model` 无法导入、测试全部失败的回归**：目录曾被改名为
  `algorithm_template_model`，但包名、`import`、`pyproject.toml`、`conftest.py` 仍指向
  `reusable_model`，导致 12 个测试模块全部 collection error、`pip install -e .` 找不到包。
  现在 **128 个测试全部通过**，setuptools 可正确发现 9 个包。

## [0.1.0] - 2026-10-01

### 新增

- 天轶 2.5 人形机器人 URDF 模型与 24 个 STL 网格资源包 `tianyi25_urdf`
- Gazebo Harmonic 仿真包 `tianyi25_sim`：世界文件、四轮底盘、2D 雷达、关节轨迹控制器、
  ROS↔Gazebo 桥接、关节扫掠节点、里程计 TF 节点
- SLAM Toolbox 在线建图与 Nav2 导航链路
- 可复用 Python 构建块库（几何 / 栅格 / 运动 / 视觉 / IO / 跟踪 / 硬件 / 运行时）
- Apache-2.0 许可证

[Unreleased]: https://github.com/CGmoke/robot-forge/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/CGmoke/robot-forge/releases/tag/v0.1.0
