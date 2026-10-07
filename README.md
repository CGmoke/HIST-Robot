<div align="center">

# RobotForge · 机器人锻造厂

**机器人软件的可复用构建块** —— 与具体机器人解耦、拿来即用的 Python 算法库。

[![License](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB.svg)](https://www.python.org/)
[![tests](https://img.shields.io/badge/tests-128%20passed-brightgreen.svg)](packages/reusable_model/tests)
[![CI](https://github.com/CGmoke/robot-forge/actions/workflows/python-ci.yml/badge.svg)](https://github.com/CGmoke/robot-forge/actions/workflows/python-ci.yml)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)

[简体中文](README.md) · [English](#english)

</div>

---

## 📌 这是什么

RobotForge 是一个面向机器人软件开发的 **monorepo 骨架 + 可复用算法库**。

| 目录 | 放什么 | 当前状态 |
| --- | --- | --- |
| [`packages/`](packages/) | 跨项目复用的库，与具体机器人无关 | ✅ **公开内容都在这里** —— 见下文 |
| [`projects/`](projects/) | 具体的机器人工程项目（仿真 / 导航 / 抓取） | ⚠️ **目录里当前只有约定说明**，没有工程内容；那些工程在内部仓库维护，不在本仓库 |

> 📖 **`projects/` 是公开可见的目录**，和仓库里其它文件一样。GitHub 没有「某个目录只有我能看」
> 这种设置 —— 仓库设为 public，里面的每一个文件对所有人都可见。所以**不要把任何
> 不想公开的内容放进本仓库**，哪怕只放一会儿：一旦推送，历史与 PR 引用里都可能留下痕迹。

三条设计原则：

| 原则 | 含义 |
| --- | --- |
| 🧩 **复用下沉** | 多个项目共用的算法与工具抽成库，杜绝复制粘贴式复用 |
| 🔌 **与机器人解耦** | `packages/` 下的代码不含绝对路径、不读环境变量、不依赖任何具体机器人的配置 |
| 🧪 **自动化守护** | 128 个单测 + 三档 Python 版本的 CI，每个模块附带可执行 doctest |

## 🧩 当前主要内容：`reusable_model`

从真实机器人项目中沉淀出来的 Python 构建块。**硬依赖只有 `numpy`**，
重型依赖（OpenCV / pyserial / PyYAML）一律惰性导入 —— 不装也能用其余部分。

| 子包 | 内容 |
| --- | --- |
| `geometry` | 由轴约束构造旋转矩阵、两向量最小旋转、全 24 种序列的欧拉角提取（对万向锁安全）、IoU / 重叠度评分、掩码质心 |
| `gridmap` | 占据栅格与 ROS `*.yaml` 地图互转、并查集连通分量、膨胀式区域合并、多源 Dijkstra 距离场、梯度下降航向 |
| `motion` | 双连杆平面逆运动学解析解（含肘部圆选择）、通用 LM 求解器、含加速度斜坡与超调抑制的制动速度曲线 |
| `vision` | 针孔相机内参 / 像素到射线 / 3D 投影、深度采样（中位数·均值·分位数）、点云反投影与体素降采样、最小二乘平面拟合与射线求交 |
| `io` | 带类型的 YAML 加载与导出、长度前缀二进制信封（可选用 pyarrow 承载）、面向检测批次的自描述 NPZ 打包 |
| `tracking` | 带确认机制与轨迹生命周期的贪心 IoU 多目标跟踪器、稳定连续帧门限 |
| `hardware` | 依据 udev 标识 / 序列号 / 主动探测查找串口设备、Modbus RTU 并联夹爪客户端（内置 Robotiq 2F 档位，支持空跑） |
| `runtime` | 父子解释器之间基于行的通信桥接、ctypes 库查找与 `RTLD_GLOBAL` 预加载、噪声输出流过滤 |

完整模块索引见 [`packages/reusable_model/README.md`](packages/reusable_model/README.md)。

## 🚀 快速开始

```bash
git clone https://github.com/CGmoke/robot-forge.git
cd robot-forge/packages/reusable_model
python -m pip install -e ".[test]"
python -m pytest          # 期望 128 passed
```

30 秒上手：

```python
from reusable_model.geometry.boxes import iou
from reusable_model.tracking.iou_tracker import IoUTracker

tracker = IoUTracker(min_hits=1)
frame = tracker.update([{"bbox": [0, 0, 10, 10], "label": "person"}])
print(frame[0]["track_id"], iou([0, 0, 10, 10], [2, 2, 12, 12]))
```

不安装也能用 —— 把 `src/` 加进 `PYTHONPATH` 即可：

```bash
PYTHONPATH=src python -c "from reusable_model.geometry.boxes import iou; print(iou([0,0,10,10],[2,2,12,12]))"
```

## 🧱 仓库结构

```
robot-forge/
├── README.md                  # 你在这里：仓库总入口
├── NOTICE                     # 版权与来源声明（说明为何 projects/ 为空）
├── CONTRIBUTING.md            # 怎么提 issue / PR
├── CHANGELOG.md               # 版本变更记录
├── LICENSE                    # Apache-2.0
├── .github/                   # CI、issue / PR 模板
├── docs/
│   └── adding-a-project.md    # 新项目接入规范 + 知识产权自检
├── projects/                  # 机器人工程项目（公开目录；当前只有约定说明）
│   └── README.md              # 目录约定、什么能放 / 什么不能放
└── packages/                  # 跨项目复用的库
    └── reusable_model/        # src 布局的 Python 包
        ├── README.md
        ├── pyproject.toml
        ├── src/reusable_model/   # geometry / gridmap / motion / vision / io / tracking / hardware / runtime
        └── tests/                # 128 个单测
```

**约定**：`projects/` 放"某个机器人的完整工程"，`packages/` 放"谁都能用的库"。
`projects/` 之间**不允许**互相依赖；公共部分一律下沉到 `packages/`。

## ❓ 为什么 `projects/` 里没有东西

**先澄清一点**：`projects/` 这个目录本身是**公开可见**的 —— 任何人都能浏览、下载里面的文件。
它现在只有一个 `README.md`（说明目录约定），并没有被隐藏。

没有工程内容的原因是：本仓库的机器人工程项目（仿真、导航、抓取）**依赖机器人厂商
随设备交付的模型资产**（URDF、STL 网格），其版权属于厂商、未取得再分发授权。
为避免侵权风险，这些工程已连**当前版本与 Git 历史**一起移出公开仓库，
改在**内部独立仓库**维护 —— 这才是"内部"的含义：工程在别处，不是目录被隐藏。

本项目选择**不把厂商的模型文件传上来再说**。如果你在别处见过同类项目公开分发厂商模型，
那是他们的选择，不代表这里也会这么做。

`projects/` 目录保留下来，是为将来**原创的、或已获明确再分发授权**的机器人项目预留位置。
接入前需要过 [`docs/adding-a-project.md`](docs/adding-a-project.md) 里的知识产权自检 ——
那里面写清了什么能提交、什么不能。详见 [`NOTICE`](NOTICE)。

## 🖥 环境要求

| 用途 | 要求 |
| --- | --- |
| Python 库 | Python 3.10+，仅硬依赖 `numpy`；可选依赖按需安装（`[vision]` / `[hardware]` / `[yaml]`） |
| ROS 2 项目 | 当前无公开的 ROS 2 项目；`projects/` 接纳新项目后会在该项目的 README 中说明 |

## 📚 文档索引

| 文档 | 内容 |
| --- | --- |
| [`packages/reusable_model/README.md`](packages/reusable_model/README.md) | 复用库的模块索引、安装方式与 API 概览 |
| [`projects/README.md`](projects/README.md) | `projects/` 的目录约定、可放 / 不可放内容的对照表 |
| [`docs/adding-a-project.md`](docs/adding-a-project.md) | 新增机器人项目的目录规范、README 模板与接入自检清单（含知识产权） |
| [`NOTICE`](NOTICE) | 版权与来源声明 |
| [`.github`](.github/) | 提交前会跑什么、issue / PR 怎么写 |

## 🗺 路线图

见 [CHANGELOG.md](CHANGELOG.md) 了解已完成的内容。近期方向：

- [ ] **v0.2**：`reusable_model` 发布到 PyPI，补齐类型标注与 API 文档
- [ ] 继续把内部项目中的通用部分下沉到 `packages/`
- [ ] 补充性能基准与更多 doctest 示例
- [ ] 英文文档与英文 README 完整版

欢迎在 [Issues](https://github.com/CGmoke/robot-forge/issues) 里认领或提新需求。

## 🤝 贡献

欢迎任何形式的参与：报 bug、修文档、加测试、提新模块。

- 提交前请读 [CONTRIBUTING.md](CONTRIBUTING.md)
- **加新项目**请先看 [docs/adding-a-project.md](docs/adding-a-project.md) —— 特别是知识产权自检
- 不确定要不要做，先开个 [Issue](https://github.com/CGmoke/robot-forge/issues) 聊，避免白写

## 📄 协议

本项目采用 [Apache License 2.0](LICENSE)。全部公开内容均为原创，不含第三方资产，详见 [NOTICE](NOTICE)。

## English

**RobotForge** is a monorepo skeleton and **reusable Python toolkit for robot software** —
project-agnostic building blocks you can drop into your own stack.

| Path | Contents | Status |
| --- | --- | --- |
| [`packages/reusable_model`](packages/reusable_model/README.md) | Reusable building blocks: 3D geometry, occupancy grids & distance fields, planar IK, pinhole vision & point clouds, YAML/NPZ IO, IoU multi-object tracking, Modbus gripper, subprocess bridge | ✅ 128 tests |
| [`projects/`](projects/README.md) | Robot projects (sim, navigation, manipulation) | ⚠️ **empty** — only the convention doc lives here; the projects are kept in a separate internal repository |

> **`projects/` is publicly visible**, like every other file in this repository. GitHub has no
> per-directory visibility: a public repo exposes all of its contents. Never put anything here that
> you do not want public — once pushed, it can linger in history and pull-request refs even after
> deletion.

Robot projects are kept out of this repository because they depend on robot-model assets delivered by
a hardware vendor, whose copyright we do not hold and have no redistribution license for. Rather than
upload them first and ask later, this repository simply does not contain them — they were removed from
both the current revision and the Git history. `projects/` is reserved for future projects that are
original or explicitly licensed for redistribution.

Only hard dependency is `numpy`; heavy dependencies (OpenCV / pyserial / PyYAML) are imported lazily.

```bash
git clone https://github.com/CGmoke/robot-forge.git
cd robot-forge/packages/reusable_model && python -m pip install -e ".[test]" && python -m pytest
```

Contributions are welcome — please read [CONTRIBUTING.md](CONTRIBUTING.md).
Licensed under [Apache-2.0](LICENSE).
