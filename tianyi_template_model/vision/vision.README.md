# vision 子包说明

`reusable_model.vision` 是从机器人视觉栈中抽取的纯 Python 子包，把「一张深度/彩色图像」
转换为「机器人可用的三维几何」所需的全部基础运算集中在一起。它只依赖 `numpy`
（导入时），OpenCV 与 SciPy 均为按需惰性导入，因此可以在无 ROS、无图形界面的
服务器环境中使用。

子包由四个模块组成：

- `pinhole.py`：针孔相机模型（内参、反投影、投影、射线）；
- `depth.py`：深度图的采样、渲染与区域辅助；
- `pointcloud.py`：点云的采样、精简、滤波、变换与导出；
- `planes.py`：平面拟合、射线-平面求交与相对地面的测量。

`__init__.py` 通过 `from .xxx import *` 再导出四个模块的公开接口，并设置
`__all__ = ["depth", "pinhole", "planes", "pointcloud"]`，因此可以用
`from reusable_model.vision import Intrinsics, sample_depth, ...` 直接使用。

---

## 1. 模块总览

| 文件 | 主要职责 | 对外公开接口 |
| --- | --- | --- |
| `pinhole.py` | 针孔相机模型：内参数据类，像素↔三维点的双向转换，视线射线，彩色/深度像素坐标换算 | `Intrinsics`、`pixel_to_3d`、`pixel_to_3d_batch`、`project`、`project_batch`、`ray_direction`、`pixel_to_depth_pixel` |
| `depth.py` | 深度图采样与单位换算，有效掩码，区域（包围框）辅助，伪彩色渲染，框内颜色比例 | `HSV_GRASS_GREEN`、`DepthLimits`、`infer_depth_scale`、`valid_mask`、`depth_to_meters`、`sample_depth`、`bbox_depth`、`render_depth`、`relative_box`、`color_ratio_in_bbox` |
| `pointcloud.py` | 深度图→点云的反投影与采样，体素降采样与几何滤波，坐标系变换，法线估计，PLY 导出，封装处理器 | `backproject_depth`、`grid_sample`、`random_sample`、`voxel_downsample`、`filter_by_height`、`filter_by_horizontal_distance`、`transform_points`、`transform_points_by`、`compute_normals`、`write_ply`、`PointCloudProcessor`（另含 `SAMPLING_MODES`） |
| `planes.py` | 总体最小二乘平面拟合与定向，射线-平面求交，检测框周围的地面环形采样，拟合质量度量，检测的物理面积与形状分析 | `fit_plane`、`fit_plane_oriented`、`ray_plane_intersection`、`pixel_to_plane`、`sample_ground_ring`、`plane_residuals`、`plane_fit_rmse`、`point_to_plane_distance`、`bbox_3d_area`、`fit_ellipse_to_points`、`elongation_from_points` |

---

## 2. 依赖关系与调用层级

### 2.1 第三方依赖

| 依赖 | 引入方式 | 使用位置 |
| --- | --- | --- |
| `numpy` | 模块导入时 | 全部四个模块 |
| `cv2`（OpenCV） | 惰性导入 | `depth.render_depth`、`depth.color_ratio_in_bbox`、`planes.fit_ellipse_to_points` |
| `scipy.spatial` | 惰性导入 | `pointcloud.compute_normals`（KD-tree 近邻搜索） |

不使用 ROS 或其他子包。`pinhole.Intrinsics.from_ros_camera_info` 接受的是一个
**普通映射**（含 `width`、`height`、`k` 键），并不导入 `sensor_msgs`。

### 2.2 对其他子包的依赖

**本子包不依赖 `reusable_model` 中的任何其他子包**，只依赖标准库与上述第三方库。

### 2.3 子包内部调用层级

- `pinhole.py`：叶子模块，**不依赖**其他三个模块。
- `depth.py`：叶子模块，**不依赖**其他三个模块。
- `pointcloud.py` **依赖**：
  - `depth.valid_mask`（采样时做深度范围过滤）；
  - `pinhole.Intrinsics`、`pinhole.pixel_to_3d_batch`（反投影）。
- `planes.py` **依赖**：
  - `depth.sample_depth`、`depth.valid_mask`（角点深度采样与范围过滤）；
  - `pinhole.Intrinsics`、`pinhole.pixel_to_3d`、`pinhole.pixel_to_3d_batch`、
    `pinhole.ray_direction`（反投影与射线）。

依赖方向为单向：`pinhole` / `depth` → `pointcloud` / `planes`，不存在环。

---

## 3. 各模块职责与接口定义

约定：所有三维点均位于 :mod:`reusable_model.vision.pinhole` 定义的**相机光学坐标系**
（`+x` 向右、`+y` 向下、`+z` 向前）。深度图默认为 `uint16` 毫米，`depth_scale`
表示「每米对应的原始单位数」（默认 1000.0）。

### 3.1 pinhole.py

**职责**：实现教科书式的针孔模型 `u = fx * X / Z + cx`、`v = fy * Y / Z + cy`
及其逆运算，并集中提供内参对象与射线计算。

**公开 API**

| 接口 | 签名（要点） | 返回 / 异常 |
| --- | --- | --- |
| `Intrinsics` | `@dataclass(frozen=True)`；字段 `fx, fy, cx, cy, width, height, depth_scale=1000.0` | 冻结数据类，构造时校验 |
| `Intrinsics.to_matrix()` | `() -> np.ndarray` | `(3, 3)` 相机矩阵 |
| `Intrinsics.from_matrix(k, width, height, depth_scale=1000.0)` | `@classmethod` | 由 `(3, 3)` 矩阵或 9 元素行优先序列构造 |
| `Intrinsics.from_ros_camera_info(msg_dict, depth_scale=1000.0)` | `@classmethod` | 由普通映射/对象（`width`、`height`、`k`）构造 |
| `Intrinsics.scaled(factor)` | `-> Intrinsics` | 图像均匀缩放后的内参 |
| `Intrinsics.resized(width, height)` | `-> Intrinsics` | 匹配新分辨率的内参（支持非均匀缩放） |
| `pixel_to_3d(u, v, depth_m, intrinsics)` | `-> np.ndarray` | `(3,)` 相机系点（米） |
| `pixel_to_3d_batch(uv, depths, intrinsics, *, return_valid=False)` | `-> np.ndarray` 或元组 | `(N, 3)` 点；无效项为 `NaN` |
| `project(x, y, z, intrinsics)` | `-> tuple[float, float, float]` | `(u, v, depth_raw)`；`z <= 0` 抛 `ValueError` |
| `project_batch(points, intrinsics, *, return_valid=False)` | `-> np.ndarray` 或元组 | `(N, 3)` 的 `[u, v, depth_raw]`；无效项为 `NaN` |
| `ray_direction(u, v, intrinsics)` | `-> np.ndarray` | `(3,)` 单位视线射线 |
| `pixel_to_depth_pixel(u, v, color_shape, depth_shape)` | `-> tuple[int, int]` | 彩色像素对应的深度像素（已钳制） |

**使用示例**

```python
from reusable_model.vision import Intrinsics, pixel_to_3d, project, ray_direction

intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)

point = pixel_to_3d(320.0, 240.0, 2.0, intr)   # 主点处 2 米深 -> [0, 0, 2]
print(point)                                    # [0. 0. 2.]

u, v, depth_raw = project(2.0, 0.0, 2.0, intr)  # -> (820.0, 240.0, 2000.0)
d = ray_direction(820.0, 240.0, intr)           # 单位向量，z 分量为正
```

### 3.2 depth.py

**职责**：把噪声大、空洞多、单位不明确的原始深度缓冲区转换为机器人可决策的数值：
鲁棒的单点/区域深度、有效掩码与单位换算、伪彩色渲染，以及「物体下方地面」这类
几何区域辅助。

**核心约定**：每个采样器都按度量深度范围过滤，并以 `None`（而非可疑数值）表示
「无测量值」。

**公开 API**

| 接口 | 签名（要点） | 返回 / 异常 |
| --- | --- | --- |
| `HSV_GRASS_GREEN` | `tuple[tuple[int, int, int], tuple[int, int, int]]` | 修剪草坪绿色的 HSV 上下界 |
| `DepthLimits` | `@dataclass(frozen=True)`；`min_depth=0.05, max_depth=20.0` | 深度范围数据类 |
| `infer_depth_scale(depth)` | `-> float` | 从深度量级推断的缩放系数 |
| `valid_mask(depth, *, depth_scale=1000.0, min_depth=0.05, max_depth=20.0)` | `-> np.ndarray` | 布尔有效掩码 |
| `depth_to_meters(depth, depth_scale=1000.0)` | `-> np.ndarray` | 以米为单位的深度 |
| `sample_depth(depth, u, v, *, depth_scale=1000.0, window=11, statistic="median", min_depth=0.05, max_depth=20.0, min_valid_pixels=1, percentile=50.0)` | `-> float \| None` | 米；无有效测量返回 `None` |
| `bbox_depth(depth, bbox, *, depth_scale=1000.0, statistic="median", percentile=50.0, min_valid_pixels=8, min_depth=0.05, max_depth=20.0)` | `-> float \| None` | 框内鲁棒深度（米） |
| `render_depth(depth, *, depth_scale=1000.0, max_depth=None, colormap="turbo", percentile_lo=2.0, percentile_hi=98.0, size=None, annotate=True)` | `-> np.ndarray \| None` | 伪彩色 BGR 图像；需 `cv2` |
| `relative_box(bbox, *, y0_frac=0.0, y1_frac=1.0, x0_frac=0.0, x1_frac=1.0, image_shape=None, pad_x=0, pad_y=0)` | `-> tuple[int, int, int, int]` | 父框内的子框 |
| `color_ratio_in_bbox(bgr, bbox, *, lower_hsv, upper_hsv)` | `-> float` | 框内命中比例；需 `cv2` |

`statistic` 取 `"median"`、`"mean"` 或 `"percentile"`（`STATISTICS` 常量）。
异常：参数类型/范围错误抛 `TypeError` / `ValueError`；`render_depth` 与
`color_ratio_in_bbox` 在缺少 OpenCV 时抛 `ImportError`。

**使用示例**

```python
import numpy as np
from reusable_model.vision import sample_depth, bbox_depth, valid_mask

depth = np.full((480, 640), 2000, dtype=np.uint16)  # 2 米处的平地

print(sample_depth(depth, 320, 240))                # 2.0
print(sample_depth(depth, 320, 240, statistic="mean"))
print(bbox_depth(depth, (0, 0, 640, 480)))          # 2.0

mask = valid_mask(depth, depth_scale=1000.0, min_depth=0.1, max_depth=10.0)
print(mask.dtype, mask.all())                       # bool True
```

### 3.3 pointcloud.py

**职责**：从 RGB-D 帧生成点云的短流程——反投影/采样、体素降采样、几何滤波、
坐标系变换、法线估计与导出。`PointCloudProcessor` 把内参与深度范围封装为一个对象。

模块常量：`SAMPLING_MODES = ("grid", "random", "full")`。

**公开 API**

| 接口 | 签名（要点） | 返回 / 异常 |
| --- | --- | --- |
| `backproject_depth(depth, intrinsics, *, depth_scale=1000.0, min_depth=0.1, max_depth=10.0, stride=1)` | `-> tuple[np.ndarray, np.ndarray]` | `(points, pixels)`：`(N, 3)` 点与 `(N, 2)` 像素 |
| `grid_sample(depth, intrinsics, *, grid_step=32, depth_scale=1000.0, min_depth=0.1, max_depth=10.0)` | `-> tuple[np.ndarray, np.ndarray]` | 均匀栅格采样 |
| `random_sample(depth, intrinsics, *, num_samples=3000, seed=None, depth_scale=1000.0, min_depth=0.1, max_depth=10.0)` | `-> tuple[np.ndarray, np.ndarray]` | 均匀随机采样 |
| `voxel_downsample(points, *, voxel_size)` | `-> np.ndarray` | 每个体素至多一点的 `(M, 3)` 数组 |
| `filter_by_height(points, *, z_min=None, z_max=None)` | `-> np.ndarray` | 保留 `z` 在区间内的点 |
| `filter_by_horizontal_distance(points, *, origin=(0.0, 0.0), max_distance=None)` | `-> np.ndarray` | 保留水平半径内的点 |
| `transform_points(points, matrix_4x4)` | `-> np.ndarray` | 施加齐次变换 |
| `transform_points_by(points, *, rotation=None, translation=None)` | `-> np.ndarray` | 分别施加旋转/平移 |
| `compute_normals(points, *, k=16, origin=None)` | `-> np.ndarray` | `(N, 3)` 单位法线；需 `scipy` |
| `write_ply(path, points, colors=None, *, binary=False)` | `-> None` | 写 PLY 文件 |
| `PointCloudProcessor(intrinsics, *, depth_scale=None, min_depth=0.1, max_depth=10.0, grid_step=32, num_samples=3000)` | 类 | 见下方方法 |
| `PointCloudProcessor.from_rgbd(rgb, depth, *, sampling="grid", **kwargs)` | `-> tuple[np.ndarray, np.ndarray, np.ndarray \| None]` | `(points, pixels, colors)` |
| `PointCloudProcessor.to_obstacles(points, *, z_min=0.05, z_max=1.2, origin=(0.0, 0.0), max_distance=None)` | `-> np.ndarray` | 世界系障碍物点 |
| `PointCloudProcessor.to_ply(path, points, colors=None)` | `-> None` | 写 ASCII PLY |

**使用示例**

```python
import numpy as np
from reusable_model.vision import Intrinsics, PointCloudProcessor, voxel_downsample

intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
proc = PointCloudProcessor(intr, min_depth=0.1, max_depth=10.0)

depth = np.full((64, 64), 1500, dtype=np.uint16)   # 1.5 米深的平面
points, pixels, colors = proc.from_rgbd(None, depth)   # 默认 grid 采样
print(points.shape[1], colors is None)                 # 3 True

cloud = np.array([[1.0, 0.0, 0.5], [1.0, 0.0, -1.0], [50.0, 0.0, 0.5]])
print(proc.to_obstacles(cloud, max_distance=5.0).tolist())  # [[1.0, 0.0, 0.5]]

sparse = voxel_downsample(points, voxel_size=0.1)
print(sparse.shape[1])                                 # 3
```

### 3.4 planes.py

**职责**：以平面代替原始深度来获得可操作的三维位置。提供总体最小二乘平面拟合
（含朝向约定）、射线-平面求交、检测框周围的地面环形采样、拟合质量度量，以及
检测的物理面积与形状/伸长分析。

**公开 API**

| 接口 | 签名（要点） | 返回 / 异常 |
| --- | --- | --- |
| `fit_plane(points)` | `-> tuple[np.ndarray, np.ndarray] \| None` | `(unit_normal, centroid)` 或 `None` |
| `fit_plane_oriented(points, *, reference_axis=2, sign=-1.0)` | `-> tuple[np.ndarray, np.ndarray] \| None` | 法线定向到已知一侧 |
| `ray_plane_intersection(ray_origin, ray_direction, plane_normal, plane_point, *, parallel_eps=1e-9)` | `-> np.ndarray \| None` | `(3,)` 交点或 `None` |
| `pixel_to_plane(u, v, plane_normal, plane_point, intrinsics)` | `-> np.ndarray \| None` | 像素射线与平面交点 |
| `sample_ground_ring(depth, bbox, intrinsics, *, margin_px=60, exclude_px=10, step_px=4, min_points=10, depth_scale=1000.0, min_depth=0.05, max_depth=20.0)` | `-> tuple[np.ndarray, tuple \| None, list]` | `(points_3d, plane, samples)` |
| `plane_residuals(points, normal, point)` | `-> np.ndarray` | `(N,)` 有符号垂直距离 |
| `plane_fit_rmse(points, normal, point)` | `-> float` | 垂直距离 RMS |
| `point_to_plane_distance(p, normal, point)` | `-> float` | 绝对垂直距离 |
| `bbox_3d_area(depth, bbox, intrinsics, **depth_kwargs)` | `-> float \| None` | 物理面积（m²）或 `None` |
| `fit_ellipse_to_points(points_2d, *, min_points=5)` | `-> tuple[float, float, float] \| None` | `(major, minor, major_angle_rad)`；需 `cv2` |
| `elongation_from_points(points_2d, *, min_points=5)` | `-> tuple[float, float \| None]` | `(aspect, major_angle)`；需 `cv2` |

**使用示例**

```python
import numpy as np
from reusable_model.vision import Intrinsics, fit_plane, ray_plane_intersection, pixel_to_plane

pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]])
normal, centroid = fit_plane(pts)
print(np.allclose(np.abs(normal), [0.0, 0.0, 1.0]))   # True

hit = ray_plane_intersection([0.0, 0.0, 0.0], [0.0, 1.0, 0.0],
                             [0.0, 1.0, 0.0], [0.0, 2.0, 0.0])
print(np.allclose(hit, [0.0, 2.0, 0.0]))              # True

intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
h = pixel_to_plane(320, 240, [0.0, 0.0, 1.0], [0.0, 0.0, 2.0], intr)
print(np.allclose(h, [0.0, 0.0, 2.0]))                # True
```

---

## 4. 模块间交互逻辑

数据流遵循「标定 → 深度 → 点云 → 平面」的单向管线：

```
pinhole.Intrinsics ──┐
                     ├──► pointcloud.backproject_depth / grid_sample / random_sample
depth.valid_mask ────┘                │
                                      ▼
                        pointcloud.voxel_downsample / filter_by_* / transform_*
                                      │
                                      ▼
                        pointcloud.PointCloudProcessor.from_rgbd → to_obstacles

depth.sample_depth ──┐
                     ├──► planes.pixel_to_plane ──► 物体接触点 / 抓取目标
planes.fit_plane ────┘
pinhole.ray_direction ──► planes.ray_plane_intersection
```

典型调用链：

1. **避障点云**：`Intrinsics` → `grid_sample` → `voxel_downsample` →
   `transform_points` → `filter_by_height` / `filter_by_horizontal_distance`。
   这里 `grid_sample` 内部经 `pinhole.pixel_to_3d_batch` 反投影，并用
   `depth.valid_mask` 过滤无效像素。
2. **带颜色的点云**：`PointCloudProcessor.from_rgbd` 按 `sampling` 分派到
   `grid_sample` / `random_sample` / `backproject_depth`，再用彩色图像取色；
   随后 `to_obstacles` 依次调用 `filter_by_height` 与
   `filter_by_horizontal_distance`。
3. **落点定位**：`sample_ground_ring` 在检测框周围采样地面（内部用
   `depth.valid_mask` 与 `pinhole.pixel_to_3d_batch`），得到局部地面平面；
   再用 `pixel_to_plane`（内部 `pinhole.ray_direction` +
   `ray_plane_intersection`）求得物体在地面上的落点。

---

## 5. 快速上手

```python
import numpy as np
from reusable_model.vision import (
    Intrinsics, PointCloudProcessor, sample_depth, transform_points,
    fit_plane_oriented,
)

# 1) 构造内参（也可用 Intrinsics.from_matrix / from_ros_camera_info）
intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)

# 2) 准备一帧深度（此处为 2 米处的平地）
depth = np.full((480, 640), 2000, dtype=np.uint16)

# 3) 单点/区域鲁棒深度（单位：米）
print(sample_depth(depth, 320, 240))                 # 2.0

# 4) 生成点云（相机光学坐标系），并降采样以控制规模
proc = PointCloudProcessor(intr, min_depth=0.1, max_depth=10.0)
points, pixels, colors = proc.from_rgbd(None, depth, sampling="grid")
print(points.shape[1], colors is None)               # 3 True

# 5) 变换到世界/机器人坐标系后，再滤波为障碍物集合
to_world = np.eye(4)                                 # 实际使用中来自 TF / 正运动学
world_points = transform_points(points, to_world)
obstacles = proc.to_obstacles(world_points)
print(obstacles.shape[1])                            # 3

# 6) 地面平面拟合（法线朝向相机，z 分量为负）
plane = fit_plane_oriented(points, reference_axis=2, sign=-1.0)
print(plane is not None)
```

---

## 6. 测试与验证

子包使用模块内的 doctest 与 `tests/test_vision.py` 两套测试。运行前建议加
`PYTHONDONTWRITEBYTECODE=1` 与 `-p no:cacheprovider`，避免在只读/沙箱环境中写入
`__pycache__` 与缓存目录。

```bash
# 语法编译检查（工作目录 reusable_model/）
python3 -m py_compile vision/pinhole.py vision/depth.py \
    vision/pointcloud.py vision/planes.py vision/__init__.py

# 单元测试
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest tests/test_vision.py -q -p no:cacheprovider

# 模块内 doctest（仓库根目录为 Garden/）
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/home/moke/Coding/Garden \
    python3 -m pytest --doctest-modules reusable_model/vision -q -p no:cacheprovider
```

无第三方测试框架或额外依赖需求；OpenCV/SciPy 相关的用例在其未安装时会自行跳过。
