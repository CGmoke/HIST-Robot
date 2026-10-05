# gridmap 子包说明

`reusable_model.gridmap` 是从二维导航栈中抽取的一套纯 Python 栅格工具库，围绕「占据栅格（occupancy grid）—连通区域（connected component）—米制距离场（distance field）—下降航向（descent）」这条主线组织。它只依赖 `numpy`（可选依赖 OpenCV，且均为惰性导入），不读取任何文件、不绑定任何项目配置，可以独立在测试、notebook 或服务中使用。

---

## 1. 模块总览

| 文件 | 主要职责 | 对外公开接口（`__all__`） |
| --- | --- | --- |
| `__init__.py` | 子包入口，星号重导出四个模块 | `__all__ = ["descent", "distance_field", "occupancy", "regions"]`（均为子模块名） |
| `occupancy.py` | 二维占据栅格，显式约定世界坐标与像素坐标的对应关系；提供 ROS `map_server` YAML / PGM 互操作 | `FREE`、`OCCUPIED`、`UNKNOWN`、`GridGeometry`、`OccupancyGrid` |
| `regions.py` | 面向二维导航栅格的连通分量区域分析：并查集、两种标记方式、膨胀合并、门洞连通图、质心/内部点 | `RegionInfo`、`UnionFind`、`connectivity_from_segments`、`draw_lines`、`label_connected`、`label_with_step_constraint`、`merge_regions_by_dilation`、`region_centroid_world`、`region_point_inside` |
| `distance_field.py` | 单目标距离场：多源 Dijkstra（堆版与 numpy 波前版）、带制动斜坡的速度掩码、动态障碍时间戳、局部重规划、序列化与可视化 | `DistanceField`、`create_distance_field_for_region` |
| `descent.py` | 由距离场推导连续航向：加权最小二乘平面拟合、路径预测、投影行进距离 | `descent_direction`、`predict_path`、`projected_motion_distance` |

> 说明：`gridmap.__all__` 只列出四个子模块名，因此 `from reusable_model.gridmap import *` 导入的是子模块本身（`occupancy`、`regions`、`distance_field`、`descent`）。若需要具体类与函数，请按 `from reusable_model.gridmap.occupancy import OccupancyGrid` 这一形式显式导入，或通过 `from reusable_model.gridmap import occupancy` 后以 `occupancy.OccupancyGrid` 访问。

---

## 2. 依赖关系与调用层级

### 2.1 第三方依赖

- **必需**：`numpy`。四个模块都直接 `import numpy as np`。
- **可选（惰性导入）**：`opencv-python`（`cv2`）。仅以下功能需要，缺失时会抛出带安装提示的 `RuntimeError`（`save_visualization` 则抛 `ImportError`）：
  - `regions.merge_regions_by_dilation`（形态学膨胀合并区域）
  - `regions.draw_lines`（画线段）
  - `distance_field.DistanceField.generate(..., region_ids=...)` 触发的区域合并
  - `distance_field.DistanceField.save_visualization`
- **不导入 `yaml`**：`OccupancyGrid.to_ros_yaml` 返回普通 `dict`，`OccupancyGrid.from_ros_yaml` 接收普通 `dict`，由调用方自行用 `yaml` 读写。

### 2.2 跨子包依赖

本子包**不依赖 `reusable_model` 下的任何其他子包**。`gridmap` 内部四个模块的相互依赖如下（仅两处）：

- `regions.py` → `from .occupancy import GridGeometry`
- `distance_field.py` → `from .occupancy import GridGeometry`、`from .regions import merge_regions_by_dilation`

`occupancy.py` 与 `descent.py` 不依赖任何同包模块（`descent.py` 仅用 `numpy` 与标准库）。

### 2.3 内部调用层级与数据流

```text
occupancy.py                 （无内部依赖）
   │  GridGeometry / OccupancyGrid.region_labels()
   ▼
regions.py                   （依赖 occupancy.GridGeometry）
   │  label_connected / label_with_step_constraint → labels, RegionInfo[]
   │  merge_regions_by_dilation（可选，需 cv2）
   ▼
distance_field.py            （依赖 occupancy.GridGeometry、regions.merge_regions_by_dilation）
   │  DistanceField.generate(...) → distances / speed_mask / boundary_mask
   ▼
descent.py                   （仅依赖 numpy）
      descent_direction / predict_path / projected_motion_distance
```

典型数据流：占据栅格（`OccupancyGrid`）→ 可通行掩码（`region_labels()`）→ 连通区域标签（`regions.label_connected`）→ 单目标距离场（`DistanceField.generate` 或 `create_distance_field_for_region`）→ 连续航向（`descent.descent_direction`）。

---

## 3. 各模块职责与接口定义

### 3.1 `occupancy.py`

**职责**：把「图像的哪个角是原点、y 轴朝哪个方向增长」这一最常见的导航 bug 来源显式化。

- *内部（库）约定*：`data[py, px]`，行 `0` 对应世界 y 的**最小值**，列 `0` 对应世界 x 的**最小值**，世界 y 随行索引增大。
- *图像 / PGM / ROS `map_server` 约定*：行 `0` 是图像**顶部**（世界 y 最大值）。用 `flip_vertically()` 在两种约定间转换。
- 灰度值遵循 ROS/PGM 约定：值越大越空闲。

#### 常量

| 名称 | 类型 | 值 | 含义 |
| --- | --- | --- | --- |
| `FREE` | `int` | `254` | 已知可通行栅格 |
| `OCCUPIED` | `int` | `0` | 已知被阻挡栅格 |
| `UNKNOWN` | `int` | `205` | 从未被观测的栅格 |

#### `GridGeometry`（frozen dataclass）

不可变的轴对齐度量坐标系，可在占据栅格与距离场之间安全共享。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `origin_x` | `float` | 像素 `(0, 0)` 左下角的世界 x，单位米 |
| `origin_y` | `float` | 像素 `(0, 0)` 左下角的世界 y，单位米 |
| `resolution` | `float` | 单元尺寸，米/像素，必须 `> 0` |
| `width` | `int` | 列数，至少为 1 |
| `height` | `int` | 行数，至少为 1 |

属性：

| 属性 | 返回类型 | 含义 |
| --- | --- | --- |
| `x_max` | `float` | `origin_x + width * resolution` |
| `y_max` | `float` | `origin_y + height * resolution` |
| `bounds` | `tuple[float, float, float, float]` | `(x_min, y_min, x_max, y_max)` |

异常：构造时对字段类型/取值做校验，抛 `TypeError`（非实数）或 `ValueError`（非有限、`resolution <= 0`、`width`/`height` 非 `>= 1` 的整数）。

#### `OccupancyGrid`

| 成员 | 签名 | 返回 | 说明 |
| --- | --- | --- | --- |
| `__init__` | `(data: ArrayLike, geometry: GridGeometry)` | `None` | 用像素数组及其坐标系包装栅格；`geometry.width/height` 必须与 `data.shape[1]/[0]` 一致 |
| `from_array`（classmethod） | `(data, *, origin=(0.0, 0.0), resolution: float)` | `OccupancyGrid` | 从裸数组加原点、分辨率构建 |
| `world_to_pixel` | `(x: float, y: float)` | `tuple[int, int]` | 世界坐标 → 像素索引，**夹取**到 `[0, width-1]` / `[0, height-1]` |
| `pixel_to_world` | `(px: int, py: int)` | `tuple[float, float]` | 像素索引 → 单元中心世界坐标（可用小数索引做插值） |
| `world_to_pixel_batch` | `(xs: ArrayLike, ys: ArrayLike)` | `tuple[np.ndarray, np.ndarray]` | 向量化 `world_to_pixel`，`int64`，含相同夹取 |
| `pixel_to_world_batch` | `(px: ArrayLike, py: ArrayLike)` | `tuple[np.ndarray, np.ndarray]` | 向量化 `pixel_to_world`，`float64` |
| `in_bounds_pixel` | `(px: int, py: int)` | `bool` | `0 <= px < width` 且 `0 <= py < height` |
| `in_bounds_world` | `(x: float, y: float)` | `bool` | 点是否落在 `geometry.bounds` 内（上界为开区间） |
| `contains_world` | `(x: float, y: float)` | `bool` | `in_bounds_world` 的口语化别名 |
| `value_at_pixel` | `(px: int, py: int, *, default: Any = <哨兵>)` | `int` | 读取灰度值；越界且未给 `default` 时抛 `ValueError` |
| `value_at_world` | `(x: float, y: float, *, default: Any = <哨兵>)` | `int` | 读取世界位置处灰度值；越界且未给 `default` 时抛 `ValueError` |
| `to_ros_yaml` | `(image_name: str, *, occupied_thresh=0.65, free_thresh=0.196, negate=0)` | `dict[str, Any]` | 生成 ROS `map_server` 的 YAML 内容（`origin` 恒为 `[x, y, 0.0]`） |
| `from_ros_yaml`（classmethod） | `(yaml_dict: Mapping[str, Any], image: ArrayLike)` | `OccupancyGrid` | 从 `map_server` YAML dict 与图像构建；图像会被翻转为内部约定 |
| `flip_vertically` | `()` | `OccupancyGrid` | 行序反转的副本；几何保持不变 |
| `region_labels` | `(free_min: int = UNKNOWN + 1)` | `np.ndarray` | 返回 `int32` 掩码，`1` 空闲 / `0` 其余，默认阈值 `206` |

属性：`data`（`(height, width)` 的 `uint8` 数组）、`geometry`（`GridGeometry`）。

关键异常：`__init__` 抛 `TypeError`（`geometry` 类型错误或 `data` 非 array-like）、`ValueError`（非二维、形状不匹配、值非有限或超出 `[0, 255]`）；`to_ros_yaml` 校验阈值范围与 `negate ∈ {0, 1}`；`from_ros_yaml` 忽略（并告警）`origin` 的 yaw 分量。

**示例**：

```python
import numpy as np
from reusable_model.gridmap.occupancy import FREE, OCCUPIED, OccupancyGrid

grid = OccupancyGrid.from_array(
    [[FREE, OCCUPIED], [OCCUPIED, FREE]], origin=(-1.0, -1.0), resolution=0.1
)
print(grid.geometry.bounds)              # (-1.0, -1.0, -0.8, -0.8)
print(grid.value_at_world(-0.95, -0.95)) # 254
print(grid.world_to_pixel(-0.95, -0.95)) # (0, 0)
print(grid.flip_vertically().data.tolist())  # [[0, 254], [254, 0]]
```

---

### 3.2 `regions.py`

**职责**：回答拓扑导航反复追问的楼层平面问题——哪些单元同属一个可行走房间、房间之间如何通过门洞相接、房间里哪一点保证落在内部。

- `label_connected`：对布尔空闲掩码做普通连通性标记。
- `label_with_step_constraint`：额外拒绝合并地面高度差超过 `max_step` 的相邻单元，把楼梯/坡道切分成独立房间。

#### `RegionInfo`（frozen dataclass）

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `id` | `int` | 区域 id，始终 `>= 1` |
| `area_cells` | `int` | 属于该区域的单元数 |
| `centroid_pixel` | `tuple[float, float]` | `(x, y) = (列, 行)` 平均位置，单位像素 |
| `bbox` | `tuple[int, int, int, int]` | 包含式 `(x_min, y_min, x_max, y_max)` 像素包围盒 |

#### `UnionFind`

带路径压缩与按大小合并的并查集。传入 `size` 使用稠密 `numpy` 数组，省略则使用基于 `dict` 的稀疏结构。

| 成员 | 签名 | 返回 | 说明 |
| --- | --- | --- | --- |
| `__init__` | `(size: int | None = None)` | `None` | 稠密模式元素为 `0..size-1`；稀疏模式首次使用时发现元素 |
| `find` | `(element: int)` | `int` | 返回所在集合的根；路径压缩 |
| `union` | `(a: int, b: int)` | `bool` | 合并两个集合，返回是否真正发生了合并 |
| `groups` | `()` | `dict[int, list[int]]` | `{根: [成员...]}` |

#### 模块级函数

| 函数 | 签名 | 返回 | 说明 |
| --- | --- | --- | --- |
| `label_connected` | `(mask: ArrayLike, *, connectivity=8, min_area_cells=0, order="desc")` | `tuple[np.ndarray, list[RegionInfo]]` | 普通连通分量标记；输出 `int32` 标签与按 id 排序的 `RegionInfo` 列表 |
| `label_with_step_constraint` | `(free_mask: ArrayLike, height_map: ArrayLike, *, max_step: float, connectivity=4, min_area_cells=0, order="desc")` | `tuple[np.ndarray, list[RegionInfo]]` | 同时满足「都空闲」且 `|Δheight| <= max_step` 才相连 |
| `merge_regions_by_dilation` | `(labels: ArrayLike, region_ids: Sequence[int], *, kernel_size=7, iterations=2)` | `np.ndarray` | 通过门洞（膨胀重叠）把区域焊接起来；**需要 OpenCV** |
| `region_point_inside` | `(labels: ArrayLike, region_id: int)` | `tuple[int, int]` | 返回保证落在该区域内的 `(px, py)` |
| `region_centroid_world` | `(labels: ArrayLike, region_id: int, geometry: GridGeometry)` | `tuple[float, float]` | 区域的世界坐标质心（单元中心均值） |
| `connectivity_from_segments` | `(labels: ArrayLike, segments, *, normal_probe_px=10, samples=None)` | `dict[str, dict[int, Any]]` | 从门洞线段推导区域连通图，返回 `{'matrix': ..., 'connections': ...}` |
| `draw_lines` | `(image: ArrayLike, segments, color: Sequence[int], thickness: int = 2)` | `np.ndarray` | 在图像副本上画线段；**需要 OpenCV**，不修改输入 |

关键异常：`label_connected` / `label_with_step_constraint` 对 `connectivity`（须 4 或 8）、`min_area_cells`（`>= 0`）、`order`（`'desc'`/`'asc'`）做校验；`label_with_step_constraint` 还要求 `free_mask` 与 `height_map` 形状一致、`max_step` 有限非负。`merge_regions_by_dilation` 要求 `kernel_size` 为 `>= 3` 的奇数、`iterations >= 1`，且 `region_ids` 均存在于 `labels` 中，否则抛 `ValueError`，缺 OpenCV 抛 `RuntimeError`。

**示例**（无需 OpenCV）：

```python
import numpy as np
from reusable_model.gridmap.regions import label_connected, region_point_inside

mask = np.array([[1, 1, 0, 0],
                 [1, 1, 0, 2],
                 [0, 0, 0, 2]])
labels, regions = label_connected(mask, connectivity=4)
print([(r.id, r.area_cells) for r in regions])   # [(1, 4), (2, 2)]
print(region_point_inside(labels, 1))            # 落在区域 1 内的 (px, py)
```

---

### 3.3 `distance_field.py`

**职责**：为区域内单个目标构建到所有单元的米制最短路径距离场。值以 `float32` 米存储，无法到达的单元保存哨兵值 `DistanceField.UNREACHABLE = -1.0`（而非 `inf`，以便于定点缓冲、避免污染算术、并使有效性检查退化为 `value < 0`）。

#### `DistanceField`

| 成员 | 签名 | 返回 | 说明 |
| --- | --- | --- | --- |
| `__init__` | `(*, resolution: float = 0.06)` | `None` | 固定单元尺寸；须为有限且 `> 0` |
| `generate` | `(mask, target_world: Sequence[float], *, geometry: GridGeometry, brake_distance=1.0, robot_radius=0.0, region_ids=None, dilation=None)` | `None` | 裁剪到掩码包围盒 → 重采样 → 构建速度掩码 → 从目标做单源 Dijkstra；结果存于实例 |
| `dijkstra_multi_seed` | `(seeds, cost_mask, *, block_value=0.0)` | `np.ndarray` | 优先队列版多源 Dijkstra，返回 `float32` 累积代价，不可达为 `+inf` |
| `dijkstra_multi_seed_numpy` | `(seeds, cost_mask, *, block_value=0.0)` | `np.ndarray` | 向量化波前传播（Bellman-Ford 风格），大种子集合时更快 |
| `world_to_grid` | `(x: float, y: float)` | `tuple[int, int]` | 世界 → 距离场单元，**不做夹取**（可能为负/越界） |
| `grid_to_world` | `(px: int, py: int)` | `tuple[float, float]` | 距离场单元 → 单元中心世界坐标 |
| `world_to_grid_batch` | `(xs: ArrayLike, ys: ArrayLike)` | `tuple[np.ndarray, np.ndarray]` | 向量化 `world_to_grid`，同样不夹取 |
| `distance_at_world` | `(x: float, y: float)` | `float` | 到目标距离；越界或不可达返回 `UNREACHABLE` |
| `is_inside` | `(x: float, y: float, threshold: float = 0.8)` | `bool` | 速度掩码取值是否超过 `threshold`（针对带制动斜坡的掩码） |
| `update_obstacle_timestamps` | `(points_world_xy: ArrayLike | None, now=None)` | `int` | 为障碍点覆盖的单元打观测时间戳，返回落在场内的点数 |
| `valid_obstacle_mask` | `(now: float | None = None, timeout: float = 1.0)` | `np.ndarray` | 返回未超时障碍的 `bool` 网格 |
| `recompute_local` | `(robot_world: Sequence[float], *, search_radius: float, obstacles_xy=None, obstacle_timestamps=None, obstacle_timeout=1.0, robot_radius=0.4, acceleration=0.5, stop_when_no_path=False, connectivity_from=None)` | `np.ndarray | None` | 机器人窗口内结合实时障碍重算局部场；窗口完全在场外返回 `None` |
| `generate_inverted` | `(*, boundary_threshold: float = 0.9)` | `np.ndarray` | 镜像场：区域边缘为 `0`、向内递增（到最近墙壁的步行距离）；结果不存实例 |
| `to_array` | `()` | `np.ndarray` | 距离数组的私有 `float32` 副本 |
| `from_array`（classmethod） | `(data, *, geometry: GridGeometry, target_grid=None)` | `DistanceField` | 从存储的距离数组与坐标系重建（仅支持只读查询） |
| `save_visualization` | `(path: str, *, overlay_arrow: Sequence[float] | None = None)` | `np.ndarray` | 写出伪彩色 PNG 并返回 BGR 图像；**需要 OpenCV** |

属性（生成前均为 `None`）：`resolution`（构造时固定）、`geometry`、`distances`、`speed_mask`、`boundary_mask`、`target_grid`、`shape`。类常量：`UNREACHABLE: ClassVar[float] = -1.0`。

#### `create_distance_field_for_region`

```python
create_distance_field_for_region(
    labels, geometry, region_ids, target_world,
    *, resolution=0.06, brake_distance=1.0, robot_radius=0.0
) -> DistanceField
```

便捷入口：读入 `regions` 产生的标签数组、一组区域 id 与目标点，先经门洞把区域焊接在一起，再构建距离场。不读写任何文件。**区域合并需要 OpenCV**。

关键异常：`generate` / `create_distance_field_for_region` 抛 `TypeError`（类型错误）、`ValueError`（形状不匹配、掩码无可通行单元、参数越界、给出了 `dilation` 却无 `region_ids`、目标不在区域内）、`RuntimeError`（要求合并但缺 OpenCV）。`from_array` 重建的场不支持 `recompute_local`（会抛明确错误）。

**示例**（无需 OpenCV）：

```python
import numpy as np
from reusable_model.gridmap.occupancy import GridGeometry
from reusable_model.gridmap.distance_field import DistanceField

geometry = GridGeometry(0.0, 0.0, 0.5, 4, 4)
mask = np.ones((4, 4), bool)
mask[:, 3] = False                      # 右侧一堵墙
field = DistanceField(resolution=0.5)
field.generate(mask, (0.25, 0.25), geometry=geometry, brake_distance=0.0)
print(field.shape, field.target_grid)   # (5, 4) (0, 0)
print(field.distances[0, 0], field.distances[0, 2])  # 0.0 1.0
print(field.distance_at_world(99.0, 99.0))           # -1.0（越界 → UNREACHABLE）
```

---

### 3.4 `descent.py`

**职责**：用「在米制半径内对所有单元做加权最小二乘平面拟合」取代「看八个邻居挑最便宜的一个」，从而得到连续、无栅格量化、抗噪声的下降航向。

约定：距离场以 `[行, 列]` 索引，行 `0` 对应 y 最小值；三个函数都在**距离场自身的米制坐标系**中工作，即原点为 `(0, 0)` 的 `GridGeometry` 坐标系。负值与非有限单元视为「无数据」，被排除在拟合之外。

| 函数 | 签名 | 返回 | 说明 |
| --- | --- | --- | --- |
| `descent_direction` | `(field_values: ArrayLike, robot_grid_xy: Sequence[int], *, resolution: float, radius_meters: float = 0.3)` | `float | None` | 最速下降航向，弧度，范围 `(-pi, pi]`；样本不足或拟合退化返回 `None` |
| `predict_path` | `(field_values: ArrayLike, start_world_xy: Sequence[float], start_yaw: float, *, resolution: float, steps: int = 2, step_size: float = 0.1, radius_meters: float = 0.3)` | `list[tuple[float, float, float]]` | 逐步重拟合，预览距离场会把机器人带向何处 |
| `projected_motion_distance` | `(field_values: ArrayLike, world_xy: Sequence[float], yaw: float, *, resolution: float, radius_meters: float = 0.3, samples: int = 16)` | `float` | 沿当前航向估计还能前进多远 |

关键异常：`TypeError`（非数字 array-like、坐标对类型错误）、`ValueError`（场非二维、`resolution`/`radius_meters` 非正有限、`robot_grid_xy` 非场内整数单元）。`descent_direction` 对退化情形返回 `None`，调用方必须把它当作「没有可信航向」。

内部辅助常量：`_MIN_SAMPLES = 3`（拟合所需最少样本）、`_MIN_DESCENT = 1e-6`（视为有进展的最小下降量）。

**示例**：

```python
import math
import numpy as np
from reusable_model.gridmap.descent import descent_direction

resolution = 0.1
rows, cols = np.mgrid[0:21, 0:21]
cone = resolution * np.hypot(cols - 10, rows - 10)   # 目标位于 (10, 10)
heading = descent_direction(cone, (10, 16), resolution=resolution, radius_meters=0.35)
print(round(math.degrees(heading), 3))              # -90.0
```

---

## 4. 模块间交互逻辑

模块之间通过**普通 numpy 数组 + `GridGeometry`** 传递数据，彼此不共享可变状态：

1. `occupancy.OccupancyGrid` 持有像素数组与 `GridGeometry`，`region_labels()` 产出 `int32` 的 `0/1` 可通行掩码。
2. `regions.label_connected`（或 `label_with_step_constraint`）消费该掩码，返回 `labels`（`int32`，`0` 表示无区域）与 `RegionInfo` 列表；`id == 1` 是最大区域。
3. `distance_field.DistanceField.generate(mask, target_world, geometry=..., region_ids=...)` 接收标签数组与区域 id，内部通过 `regions.merge_regions_by_dilation` 把选定区域经门洞焊接（需 OpenCV），随后裁剪、重采样、构建速度掩码并做单源 Dijkstra。
4. `descent.descent_direction(field.distances, robot_grid_xy, resolution=...)` 消费距离场数组（其分辨率即 `field.resolution`），返回连续航向；`predict_path` 在此之上向前推演，`projected_motion_distance` 回答「沿当前航向还能走多远」。

典型调用链：

```text
OccupancyGrid.region_labels()
    → regions.label_connected()
        → distance_field.create_distance_field_for_region()
            → descent.descent_direction()
```

坐标转换约定一致：`OccupancyGrid.pixel_to_world` 与 `occupancy._world_from_pixel` 均为 `origin + (pixel + 0.5) * resolution`；`DistanceField.world_to_grid` 与 `GridGeometry` 使用同一套 `origin_x/origin_y/resolution`，因此可以在「占据栅格 ↔ 距离场 ↔ 下降航向」之间无歧义地换算。

---

## 5. 快速上手

以下示例只依赖 `numpy`，不涉及 OpenCV：

```python
import numpy as np
from reusable_model.gridmap import occupancy, distance_field, descent

# 1) 构造占据栅格：254 空闲、0 占据，中间一堵墙
data = np.full((40, 40), occupancy.OCCUPIED, dtype=np.uint8)
data[5:35, 5:35] = occupancy.FREE
data[:, 20] = occupancy.OCCUPIED
grid = occupancy.OccupancyGrid.from_array(data, origin=(0.0, 0.0), resolution=0.1)

# 2) 用可通行掩码直接构建单目标距离场（不触发区域合并，无需 cv2）
mask = grid.region_labels() > 0
field = distance_field.DistanceField(resolution=0.1)
field.generate(mask, (1.0, 1.0), geometry=grid.geometry)

# 3) 查询距离并求下降航向（目标与查询点位于同一侧，避开中间那堵墙）
print("到目标距离:", round(field.distance_at_world(1.5, 1.5), 3))   # 约 1.054
px, py = field.world_to_grid(1.5, 1.5)
heading = descent.descent_direction(field.distances, (px, py), resolution=field.resolution)
print("下降航向(度):", None if heading is None else round(float(np.degrees(heading)), 1))
```

若需要跨房间导航（经门洞合并多个连通分量），改用 `regions.label_connected` + `distance_field.create_distance_field_for_region`，此时需先安装 OpenCV：

```python
from reusable_model.gridmap import regions, distance_field

labels, infos = regions.label_connected(mask, connectivity=8, min_area_cells=10)
field = distance_field.create_distance_field_for_region(
    labels, grid.geometry, [r.id for r in infos], (1.0, 1.0), resolution=0.1,
)
```

---

## 6. 测试与验证

### 6.1 语法编译检查

```bash
cd /home/moke/Coding/Garden/reusable_model && python3 -m py_compile \
    gridmap/occupancy.py gridmap/regions.py gridmap/distance_field.py \
    gridmap/descent.py gridmap/__init__.py
```

### 6.2 冒烟测试（pytest）

```bash
cd /home/moke/Coding/Garden/reusable_model && \
    PYTHONDONTWRITEBYTECODE=1 python3 -m pytest tests/test_gridmap.py -q -p no:cacheprovider
```

`tests/test_gridmap.py` 覆盖 `TestOccupancy`、`TestRegions`、`TestDistanceField`、`TestDescent` 四组用例。

### 6.3 doctest

```bash
cd /home/moke/Coding/Garden && \
    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/home/moke/Coding/Garden \
    python3 -m pytest --doctest-modules reusable_model/gridmap -q -p no:cacheprovider
```

三组命令应全部通过。`PYTHONDONTWRITEBYTECODE=1` 与 `-p no:cacheprovider` 用于避免在只读路径下写入 `__pycache__` 与缓存文件。
