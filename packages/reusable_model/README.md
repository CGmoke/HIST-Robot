# reusable_model

从机器人项目中抽取出的**自包含、可复用** Python 构建块。此处所有内容都与来源项目解耦：不含绝对路径、不读取环境变量、不依赖项目专属配置，也不使用 `print()`（各模块统一通过 `logging.getLogger(__name__)` 记录日志）。

## 环境要求

- Python 3.10+
- `numpy`（唯一的硬依赖）
- 可选依赖：`opencv-python-headless` / `pyarrow`（vision 子包）、`pyserial` /
  `minimalmodbus`（hardware 子包）、`PyYAML`（配置文件）。可选模块在使用处
  惰性导入，因此未安装它们时，包内其余部分仍可正常工作。

## 安装与使用

采用标准 **src 布局**：源码在 `src/reusable_model/`，测试在 `tests/`。

```bash
cd packages/reusable_model
python -m pip install -e .                          # 仅核心依赖
python -m pip install -e ".[vision,hardware,yaml,test]"   # 含可选依赖
python -m pytest                                    # 128 个单测
```

不安装也可以直接用 —— 把 `src/` 加进 `PYTHONPATH` 即可：

```bash
PYTHONPATH=src python -c "from reusable_model.geometry.boxes import iou; print(iou([0,0,10,10],[2,2,12,12]))"
```

## 模块索引

### geometry

| 模块 | 内容 |
| --- | --- |
| `reusable_model.geometry.rotations3d` | 由轴约束构造旋转矩阵、两向量间的最小旋转、考虑限位的角度折叠、欧拉角提取（支持全部 24 种序列，且对万向锁安全）。 |
| `reusable_model.geometry.boxes` | IoU / IoMin / 重叠度评分、边界框转掩码、边界框与掩码的质心。 |

### gridmap

| 模块 | 内容 |
| --- | --- |
| `reusable_model.gridmap.occupancy` | `OccupancyGrid`、`GridGeometry`、ROS `*.yaml` 地图转换。 |
| `reusable_model.gridmap.regions` | 并查集、连通分量标记、基于膨胀的区域合并。 |
| `reusable_model.gridmap.distance_field` | 支持速度掩码与局部重算的多源 Dijkstra 距离场。 |
| `reusable_model.gridmap.descent` | 由距离场计算梯度下降航向（加权平面拟合）。 |

### motion

| 模块 | 内容 |
| --- | --- |
| `reusable_model.motion.planar_ik` | 双连杆平面逆运动学解析解（含肘部圆选择）与通用 Levenberg-Marquardt 求解器。 |
| `reusable_model.motion.speed_profile` | 考虑制动的速度曲线（`v = sqrt(2ad)`），含加速度斜坡与超调抑制。 |

### vision

| 模块 | 内容 |
| --- | --- |
| `reusable_model.vision.pinhole` | 针孔相机内参、像素到射线、3D 投影。 |
| `reusable_model.vision.depth` | 深度采样（中位数 / 均值 / 分位数）、深度渲染、边界框重叠检查。 |
| `reusable_model.vision.pointcloud` | 点云的反投影、降采样、滤波与坐标变换。 |
| `reusable_model.vision.planes` | 最小二乘平面拟合、射线-平面求交、地面采样。 |

### io

| 模块 | 内容 |
| --- | --- |
| `reusable_model.io.yaml_config` | 带类型的 YAML 加载 / 导出、路径解析、`YamlStore`。 |
| `reusable_model.io.binary_envelope` | 长度前缀的二进制信封（可选由 pyarrow 承载内容）。 |
| `reusable_model.io.detection_npz` | 面向检测批次的自描述 NPZ 打包 / 解包。 |

### tracking

| 模块 | 内容 |
| --- | --- |
| `reusable_model.tracking.iou_tracker` | 带确认机制与轨迹生命周期的贪心 IoU 多目标跟踪器。 |
| `reusable_model.tracking.stable_streak` | “目标是否连续 N 帧保持不动”的确认门限。 |

### hardware

| 模块 | 内容 |
| --- | --- |
| `reusable_model.hardware.serial_discovery` | 依据 udev 标识、序列号或主动探测来查找串口设备。 |
| `reusable_model.hardware.modbus_gripper` | Modbus RTU 夹爪客户端（内置 Robotiq 2F 档位），支持空跑模式。 |

### runtime

| 模块 | 内容 |
| --- | --- |
| `reusable_model.runtime.subprocess_bridge` | 父解释器与子解释器之间基于行的通信桥接。 |
| `reusable_model.runtime.ctypes_loader` | ctypes 库查找、`RTLD_GLOBAL` 预加载、C 垫片编译。 |
| `reusable_model.runtime.stdout_filter` | 过滤嘈杂的 stdout/stderr 输出行，并静音指定日志记录器。 |

## 快速示例

```python
from reusable_model.geometry.boxes import iou, overlap_score
from reusable_model.tracking.iou_tracker import IoUTracker

tr = IoUTracker(min_hits=1)
frame = tr.update([{"bbox": [0, 0, 10, 10], "label": "person"}])
print(frame[0]["track_id"], iou([0, 0, 10, 10], [2, 2, 12, 12]))
```

## 运行测试

```bash
cd packages/reusable_model
python -m pytest
```

每个模块还附带可执行的 doctest：

```bash
python -m doctest src/reusable_model/geometry/boxes.py
```
