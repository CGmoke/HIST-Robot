<div align="center">

# RobotForge · 机器人锻造厂

**机器人软件的可复用构建块** —— 与具体机器人解耦、拿来即用的 Python 算法库。

**如果对你有帮助，请给颗小星星吧。**

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
| [`projects/`](projects/) | 具体的机器人工程项目           |**仅公开原创程序**


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

本项目采用 [Apache License 2.0](LICENSE)。
