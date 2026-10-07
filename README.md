<div align="center">

# RobotForge · 机器人锻造厂

**机器人二次开发的开源工作台** —— 一个仓库，容纳多个机器人项目：仿真 · 导航 · 感知 · 抓取 · 算法复用。

[![License](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic%20(gz--sim%208)-F58113.svg)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB.svg)](https://www.python.org/)
[![tests](https://img.shields.io/badge/tests-128%20passed-brightgreen.svg)](packages/reusable_model/tests)
[![CI](https://github.com/CGmoke/robot-forge/actions/workflows/python-ci.yml/badge.svg)](https://github.com/CGmoke/robot-forge/actions/workflows/python-ci.yml)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)

[简体中文](README.md) · [English](#english)

</div>

<!--
📷 演示媒体占位：录一段 10~20 秒的 GIF（Gazebo 里机器人迎宾挥手 / RViz 建图导航），
   存为 docs/assets/demo.gif，然后删掉本注释并启用下面这行。带演示动图的仓库 star 转化率明显更高。
![demo](docs/assets/demo.gif)
-->

---

## 📌 这是什么

RobotForge 是一个**面向机器人二次开发的 monorepo**。它把同一个机器人的仿真、导航、感知、抓取等工程，
以及多个项目共用的算法库，收在一个仓库里统一维护。

四条设计原则：

| 原则 | 含义 |
| --- | --- |
| 🎯 **一个项目一个目录** | `projects/<project>/` 自包含：独立的 README、独立的环境、独立可运行，删掉不影响别人 |
| 🧩 **复用下沉到 `packages/`** | 多个项目共用的算法与工具抽成库，杜绝复制粘贴式复用 |
| ✅ **能跑起来才算数** | 每个项目都提供最短可复现步骤 + 明确的"怎么确认它是好的" |
| 🧪 **自动化守护质量** | Python 库有 128 个单测与 CI，ROS 2 工作空间有构建检查 |

> 如果你的项目只想做**一个**机器人的深度开发，这个结构依然适用 —— 它只是把"以后要加第二个项目"的成本提前降到了 0。

## 🗂 项目索引

| 项目 | 是什么 | 技术栈 | 状态 |
| --- | --- | --- | --- |
| [`projects/tianyi25-sim`](projects/tianyi25-sim/README.md) | 天轶 2.5 人形机器人（四轮底盘 + 双腿柱 + 双臂 + 3-DoF 头部）的 Gazebo 物理仿真、SLAM 在线建图与 Nav2 迎宾导航 | ROS 2 Jazzy · Gazebo Harmonic | ✅ 可运行 |
| [`packages/reusable_model`](packages/reusable_model/README.md) | 与具体项目解耦的 Python 构建块：3D 几何、占据栅格与距离场、平面逆运动学、针孔视觉与点云、YAML/NPZ 序列化、IoU 多目标跟踪、Modbus 夹爪、子进程桥接 | Python 3.10+ · numpy | ✅ 128 tests |

**想加新项目？** 见 [docs/adding-a-project.md](docs/adding-a-project.md) —— 接入一个机器人项目只需要 3 步。

## 🚀 快速开始

按你**现在想做什么**挑一条，不需要全部执行。

### A. 只想看机器人模型（约 1 分钟）

```bash
git clone https://github.com/CGmoke/robot-forge.git
cd robot-forge/projects/tianyi25-sim/ros2_ws
colcon build --packages-select tianyi25_urdf --symlink-install
source install/setup.bash
ros2 launch tianyi25_urdf display.launch.py      # RViz + 关节滑块
```

依赖：ROS 2 Jazzy、`rviz2`、`joint-state-publisher-gui`。不需要 Gazebo。

### B. 跑 Gazebo 物理仿真 + SLAM + Nav2 迎宾导航（约 10 分钟）

```bash
# 1) 依赖
sudo apt install -y ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-robot-state-publisher \
                    ros-jazzy-xacro ros-jazzy-rviz2 python3-yaml \
                    ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs \
                    ros-jazzy-nav2-rviz-plugins

# 2) 构建
cd projects/tianyi25-sim/ros2_ws
colcon build --packages-select tianyi25_urdf tianyi25_sim --symlink-install
source install/setup.bash

# 3) 起仿真（世界 + 机器人 + 桥接 + 关节扫掠）
ros2 launch tianyi25_sim gazebo.launch.py

# 4) 另开一个终端：SLAM 建图 + Nav2
source install/setup.bash
ros2 launch tianyi25_sim tianyi_nav.launch.py rviz:=true
```

**怎么确认它是好的**（第三个终端）：

```bash
ros2 topic hz /scan          # 期望 ~10 Hz
ros2 topic hz /odom          # 期望 ~30 Hz
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}"   # 车应该动起来，SLAM 开始出图
```

机器人长什么样、每个 launch 参数什么意思、遇到问题怎么查，见
[`projects/tianyi25-sim/README.md`](projects/tianyi25-sim/README.md)。

### C. 只想用 Python 复用库（约 30 秒）

```bash
cd packages/reusable_model
python -m pip install -e ".[test]"
python -m pytest          # 期望 128 passed
```

```python
from reusable_model.geometry.boxes import iou
from reusable_model.tracking.iou_tracker import IoUTracker

tracker = IoUTracker(min_hits=1)
frame = tracker.update([{"bbox": [0, 0, 10, 10], "label": "person"}])
print(frame[0]["track_id"], iou([0, 0, 10, 10], [2, 2, 12, 12]))
```

## 🧱 仓库结构

```
robot-forge/
├── README.md                  # 你在这里：仓库总入口
├── CONTRIBUTING.md            # 怎么提 issue / PR
├── CHANGELOG.md               # 版本变更记录
├── LICENSE                    # Apache-2.0
├── .github/                   # CI、issue / PR 模板
├── docs/                      # 跨项目的通用文档
│   └── adding-a-project.md    # 新项目接入规范（monorepo 的地基）
├── projects/                  # 每个机器人项目一个目录，自包含、可独立运行
│   └── tianyi25-sim/
│       ├── README.md          # 项目入口：能干什么、怎么跑、坑在哪
│       └── ros2_ws/           # 该项目自己的 ROS 2 工作空间
│           └── src/
│               ├── tianyi25_urdf/   # 纯资源包：URDF + 24 个 STL + RViz
│               └── tianyi25_sim/    # 仿真包：世界 + 控制器 + 桥接 + Nav2
└── packages/                  # 跨项目复用的库
    └── reusable_model/        # src 布局的 Python 包
        ├── README.md
        ├── pyproject.toml
        ├── src/reusable_model/   # geometry / gridmap / motion / vision / io / tracking / hardware / runtime
        └── tests/                # 128 个单测
```

**约定**：`projects/` 放"某个机器人的完整工程"，`packages/` 放"谁都能用的库"。
`projects/` 之间**不允许**互相依赖；公共部分一律下沉到 `packages/`。

## 🖥 环境要求

| 用途 | 要求 |
| --- | --- |
| ROS 2 项目 | Ubuntu 24.04 + ROS 2 Jazzy；仿真需要 Gazebo Harmonic（gz-sim 8，随 `ros-jazzy-ros-gz-*` 提供） |
| Python 库 | Python 3.10+，仅硬依赖 `numpy`；可选依赖按需安装（`[vision]` / `[hardware]` / `[yaml]`） |

仿真链路**不依赖** `ros2_control` / `gz_ros2_control` —— 关节与底盘都用 Gazebo 内置插件驱动。

## 📚 文档索引

| 文档 | 内容 |
| --- | --- |
| [`.github` 里的 CI 与模板](.github/) | 提交前会跑什么、issue/PR 怎么写 |
| [`docs/adding-a-project.md`](docs/adding-a-project.md) | 新增机器人项目的目录规范与自检清单 |
| [`projects/tianyi25-sim/README.md`](projects/tianyi25-sim/README.md) | 天轶 2.5 仿真项目总览：环境、构建、4 个运行入口、话题与 TF 链、常见问题 |
| [`projects/tianyi25-sim/ros2_ws/src/tianyi25_sim/README.md`](projects/tianyi25-sim/ros2_ws/src/tianyi25_sim/README.md) | 仿真包细节：物理整定、雷达/四轮几何、Nav2 参数、TF 树、踩坑记录 |
| [`projects/tianyi25-sim/ros2_ws/src/tianyi25_urdf/README.md`](projects/tianyi25-sim/ros2_ws/src/tianyi25_urdf/README.md) | URDF 模型结构、关节链、网格资源、显示 launch |
| [`packages/reusable_model/README.md`](packages/reusable_model/README.md) | 复用库的模块索引与 API 概览 |

## 🗺 路线图

见 [CHANGELOG.md](CHANGELOG.md) 了解已完成的内容。近期方向：

- [ ] **v0.2**：`reusable_model` 发布到 PyPI，补齐 API 文档与类型标注
- [ ] ROS 2 CI 增强：加入 `colcon test`、`ament_lint` 与 launch 冒烟测试
- [ ] `projects/tianyi25-sim`：补一段演示 GIF；接入实机部署路径（底盘 REST 接口）
- [ ] 新增项目模板：把"接入一个新机器人"变成复制一个目录
- [ ] 英文文档与英文 README 完整版

欢迎在 [Issues](https://github.com/CGmoke/robot-forge/issues) 里认领或提新需求。

## 🤝 贡献

欢迎任何形式的参与：报 bug、修文档、加测试、提新项目。

- 提交前请读 [CONTRIBUTING.md](CONTRIBUTING.md)
- **加新项目**请先看 [docs/adding-a-project.md](docs/adding-a-project.md)
- 不确定要不要做，先开个 [Issue](https://github.com/CGmoke/robot-forge/issues) 聊，避免白写

## 📄 协议

本项目采用 [Apache License 2.0](LICENSE)。

## English

**RobotForge** is an open-source monorepo for **robot secondary development**: sim, navigation, perception,
manipulation and reusable algorithm packages — one repo, many robot projects.

- `projects/` — self-contained, runnable robot projects (each with its own README and ROS 2 workspace).
- `packages/` — reusable, project-agnostic libraries shared across projects.

Current contents:

| Path | What it is | Stack | Status |
| --- | --- | --- | --- |
| [`projects/tianyi25-sim`](projects/tianyi25-sim/README.md) | Gazebo simulation, SLAM mapping and Nav2 navigation for the Tianyi 2.5 humanoid (4-wheel base + dual arms + 3-DoF head) | ROS 2 Jazzy, Gazebo Harmonic | ✅ runnable |
| [`packages/reusable_model`](packages/reusable_model/README.md) | Reusable Python building blocks: 3D geometry, occupancy grids, planar IK, pinhole vision, IO, IoU tracking, Modbus gripper, subprocess bridge | Python 3.10+, numpy | ✅ 128 tests |

Quick start: see the [中文快速开始](#-快速开始) above — the commands are identical. Contributions are welcome,
please read [CONTRIBUTING.md](CONTRIBUTING.md). Licensed under [Apache-2.0](LICENSE).
