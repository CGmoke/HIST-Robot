"""把距离场转换为航向：平面拟合下降与路径预测。

距离场（见 :mod:`reusable_model.gridmap.distance_field`）告诉你每个单元的代价。跟随它最朴素
的方法是查看八个邻居并朝最便宜的那个迈步。这样做能用，但也会让机器人明显抖动：
答案只能是八个方向之一，因此当机器人跨过单元边界时，被指令的航向会突变 45 度，
而这种突变恰好发生在距离场足够平坦、以至于两个邻居的排序由舍入噪声决定的地方。

本模块用一个在米制半径内对所有单元做的**加权最小二乘平面拟合**来取代上述方法。
把 ``a*x + b*y + c = z`` 拟合到距离场并返回 ``atan2(-b, -a)``，就得到拟合平面的
最速下降方向，它是：

*连续的*——位置的微小变化只引起航向的微小变化，因此控制器看到的是平滑信号而不是
阶梯；
*无栅格量化*——结果不限于 45 度的整数倍，斜向几何（比如 20 度的走廊）会被顺着走
而不会被锯齿状地走；
*容忍噪声*——半径内的每个样本都按接近程度加权参与，因此单个坏单元无法翻转航向。

本模块提供三个函数：:func:`descent_direction` 用于单个位姿，
:func:`predict_path` 把该估计向前推演若干步，:func:`projected_motion_distance`
用于回答「鉴于我正指向*这里*，在这个航向不再划算之前我实际上还能走多远？」。

约定
    距离场是普通的二维数组，以 ``[行, 列]`` 索引，行 ``0`` 对应 y 最小值，与
    :mod:`reusable_model.gridmap.occupancy` 一致。三个函数都在*距离场自身的米制坐标系*中
    工作：单元 ``(px, py)`` 在 x 方向覆盖
    ``[px * resolution, (px + 1) * resolution)``，y 方向同理，也就是原点为
    ``(0, 0)`` 的 :class:`~reusable_model.gridmap.occupancy.GridGeometry` 的坐标系。当你的
    距离场原点非零时，请先减去它——``x - geometry.origin_x``——因为方向具有平移
    不变性，而世界位置没有。负值和非有限的单元值被视为「无数据」（这正是
    :attr:`~reusable_model.gridmap.distance_field.DistanceField.UNREACHABLE` 的样子），并被
    排除在所有拟合之外。

依赖：仅 :mod:`numpy` 与标准库。
"""

from __future__ import annotations

import logging
import math
from typing import Any, Sequence

import numpy as np
from numpy.typing import ArrayLike

logger = logging.getLogger(__name__)

__all__ = [
    "descent_direction",
    "predict_path",
    "projected_motion_distance",
]

#: 尝试平面拟合前所需的最少样本数。模型有三个系数，因此样本少于该值就变成了插值
#: 而非拟合，得到的梯度只会是样本布局的假象。
_MIN_SAMPLES: int = 3

#: 在 :func:`projected_motion_distance` 中仍算作有进展的最小距离场下降量（单位
#: 米）。低于该值时，「每米下降对应的前进距离」这一比值就变成除以舍入噪声，而非
#: 一次测量。
_MIN_DESCENT: float = 1e-6

_TWO_PI: float = 2.0 * math.pi


def _finite_float(value: Any, name: str) -> float:
    """把 ``value`` 转换为有限的 Python ``float``。

    参数:
        value: 候选数字。``bool`` 会被拒绝，因为 ``True`` 在 Python 中是合法的
            ``int``，但它永远不是有意义的米或弧度值。
        name: 错误消息中使用的参数名。

    返回:
        以 ``float`` 表示的值。

    异常:
        TypeError: 若 ``value`` 不是实数。
        ValueError: 若 ``value`` 不是有限数。
    """
    if isinstance(value, bool) or not isinstance(value, (int, float, np.floating, np.integer)):
        raise TypeError(f"{name} must be a real number, got {type(value).__name__}: {value!r}")
    as_float = float(value)
    if not math.isfinite(as_float):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return as_float


def _positive_float(value: Any, name: str) -> float:
    """把 ``value`` 转换为严格为正的有限 ``float``。

    参数:
        value: 候选数字。
        name: 错误消息中使用的参数名。

    返回:
        以 ``float`` 表示的值。

    异常:
        TypeError: 若 ``value`` 不是实数。
        ValueError: 若 ``value`` 不是有限数或 ``<= 0``。
    """
    as_float = _finite_float(value, name)
    if as_float <= 0.0:
        raise ValueError(f"{name} must be > 0, got {value!r}")
    return as_float


def _positive_int(value: Any, name: str) -> int:
    """把 ``value`` 转换为至少为 1 的 ``int``。

    参数:
        value: 候选整数。
        name: 错误消息中使用的参数名。

    返回:
        以 ``int`` 表示的值。

    异常:
        TypeError: 若 ``value`` 不是整数。
        ValueError: 若 ``value`` 小于 1。
    """
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise TypeError(f"{name} must be an integer, got {type(value).__name__}: {value!r}")
    as_int = int(value)
    if as_int < 1:
        raise ValueError(f"{name} must be >= 1, got {value!r}")
    return as_int


def _as_field(field_values: ArrayLike, name: str = "field_values") -> np.ndarray:
    """把距离场转换为连续内存的二维 ``float64`` 数组。

    转换为 ``float64`` 是刻意的：最小二乘正规方程会对设计矩阵取平方，而对本身已
    带有累积传播误差的 ``float32`` 米制值取平方，足以让拟合出的梯度在一个按构造
    对称的距离场上明显地不对称。

    参数:
        field_values: 距离（单位米）的二维 array-like。负值和非有限项表示
            「无数据」。
        name: 错误消息中使用的参数名。

    返回:
        ``(H, W)`` 的 ``float64`` 数组。

    异常:
        TypeError: 若 ``field_values`` 不是数字的 array-like。
        ValueError: 若 ``field_values`` 不是二维或为空。
    """
    try:
        array = np.asarray(field_values, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be array-like of numbers, got {field_values!r}") from exc
    if array.ndim != 2:
        raise ValueError(f"{name} must be a 2D (height, width) array, got shape {array.shape}")
    if array.size == 0:
        raise ValueError(f"{name} must not be empty, got shape {array.shape}")
    return np.ascontiguousarray(array)


def _as_xy(value: Any, name: str) -> tuple[float, float]:
    """把 ``value`` 转换为两个有限浮点数组成的对。

    参数:
        value: 含两个元素的 ``(x, y)`` 序列或 array-like。
        name: 错误消息中使用的参数名。

    返回:
        以 Python float 表示的 ``(x, y)``。

    异常:
        TypeError: 若 ``value`` 不是数字的 array-like。
        ValueError: 若 ``value`` 不是恰好两个有限分量。
    """
    try:
        array = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be an (x, y) pair of numbers, got {value!r}") from exc
    if array.size != 2 or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be two finite numbers (x, y), got {value!r}")
    return float(array[0]), float(array[1])


def _neighbour_window(
    field: np.ndarray, px: int, py: int, radius_meters: float, resolution: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """收集某个单元周围米制圆盘内的有效样本。

    圆盘以*单元*为单位枚举（即半径的包围盒），然后按真实米制距离过滤，因此包围盒
    的四角单元——它们最远可达 ``sqrt(2)`` 个半径——不会渗入拟合。仅靠加权还不够：
    半径之外的样本会得到负权重。

    参数:
        field: ``(H, W)`` 的 ``float64`` 距离数组。
        px: 中心单元所在的列。
        py: 中心单元所在的行。
        radius_meters: 圆盘半径，单位米。
        resolution: 单元尺寸，单位米。

    返回:
        可用样本的 ``(offset_x, offset_y, values, distances)``，均为等长的 1D
        ``float64`` 数组：距中心单元的米制偏移、该处的距离场取值，以及样本到中心
        的距离。中心单元本身被排除，因为穿过自身锚点做平面拟合会让单个取值——最
        可能已经过时的那个——把梯度拉向自己。当圆盘内没有可用样本时返回四个长度为
        零的数组；如何界定「样本太少」由调用方决定。
    """
    height, width = field.shape
    reach = max(1, int(radius_meters / resolution))
    dy_min, dy_max = max(-reach, -py), min(reach, height - 1 - py)
    dx_min, dx_max = max(-reach, -px), min(reach, width - 1 - px)
    if dy_min > dy_max or dx_min > dx_max:
        empty = np.empty(0, dtype=np.float64)
        return empty, empty.copy(), empty.copy(), empty.copy()

    ddy, ddx = np.mgrid[dy_min : dy_max + 1, dx_min : dx_max + 1]
    values = field[py + ddy, px + ddx]
    distances = resolution * np.hypot(ddx, ddy)
    usable = (
        ~((ddy == 0) & (ddx == 0))          # 中心单元是锚点，不是样本
        & np.isfinite(values)               # NaN 会毒害整个拟合
        & (values >= 0.0)                   # 负值 == 「不可达」哨兵
        & (distances < radius_meters)       # 保留圆盘，剔除包围盒四角
    )
    return (
        ddx[usable].astype(np.float64) * resolution,
        ddy[usable].astype(np.float64) * resolution,
        values[usable],
        distances[usable],
    )


def descent_direction(
    field_values: ArrayLike,
    robot_grid_xy: Sequence[int],
    *,
    resolution: float,
    radius_meters: float = 0.3,
) -> float | None:
    """返回距离场在某个单元处的最速下降航向。

    对 ``robot_grid_xy`` 周围 ``radius_meters`` 内的每个有效单元，用加权最小二乘
    拟合平面 ``a*x + b*y + c = z``，其中 ``x``/``y`` 是以该单元为原点的米制坐标，
    ``z`` 是距离场取值。样本权重为 ``radius_meters - distance``，因此机器人即将
    进入的单元占主导，而圆盘边缘的单元——那里的距离场最不像平面——几乎不计入。
    返回的角度是 ``atan2(-b, -a)``：负梯度，即到目标的距离下降最快的方向。

    为什么用平面拟合而不是「八个邻居中最小的那个」？因为邻居最小值是*量化*估计：
    它的输出只能落在八条射线上，因此跟随它的机器人会以 45 度为单位转动，而在局部
    平坦处，两个邻居的排序由浮点舍入而非几何决定。拟合出的梯度是位置的连续函数
    ——把机器人移动一毫米，航向只变化零点几度——这正是速度控制器避免在走廊中线
    附近产生极限环所需的性质。拟合还对整个圆盘取平均，因此一个被破坏的单元只会
    略微掰弯航向，而不会主宰它。

    拟合在*局部米制偏移*中进行，而非绝对世界坐标。这除了让函数与距离场原点无关
    之外，还能让设计矩阵保持良态：大型场地的世界坐标约为 ``1e5`` 量级，而偏移为
    ``1e-1`` 量级，正规方程会对这些数取平方。

    参数:
        field_values: 距离（单位米）的二维数组。负值和非有限单元会被忽略，因此
            :attr:`~reusable_model.gridmap.distance_field.DistanceField.UNREACHABLE` 无需
            特殊处理。
        robot_grid_xy: 机器人单元的 ``(px, py)`` = ``(列, 行)``。
        resolution: 单元尺寸，单位米；必须 ``> 0``。
        radius_meters: 拟合圆盘的半径，单位米。它应至少跨两个单元
            （``> 2 * resolution``），否则存活的样本太少；2 到 5 个单元的半径是
            平滑度与切角之间的良好折中。

    返回:
        下降航向，单位弧度，范围 ``(-pi, pi]``，从距离场的 +x 轴朝 +y 轴度量。当
        圆盘内的有效单元少于 :data:`_MIN_SAMPLES`，或拟合退化时返回 ``None``——
        调用方必须把 ``None`` 当作「没有可信航向」并回退，绝不能当作零。

    异常:
        TypeError: 若 ``field_values`` 不是数字的 array-like，或 ``robot_grid_xy``
            不是数字对。
        ValueError: 若 ``field_values`` 不是二维、``resolution`` 或
            ``radius_meters`` 不是正有限数，或 ``robot_grid_xy`` 不是距离场内的
            整数单元。

    示例:
        >>> import numpy as np
        >>> resolution = 0.1
        >>> rows, cols = np.mgrid[0:21, 0:21]
        >>> cone = resolution * np.hypot(cols - 10, rows - 10)   # goal at (10, 10)
        >>> heading = descent_direction(cone, (10, 16), resolution=resolution,
        ...                             radius_meters=0.35)
        >>> round(math.degrees(heading), 3)
        -90.0
    """
    field = _as_field(field_values)
    step = _positive_float(resolution, "resolution")
    radius = _positive_float(radius_meters, "radius_meters")

    try:
        cell = np.asarray(robot_grid_xy, dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"robot_grid_xy must be a (px, py) pair of integers, got {robot_grid_xy!r}"
        ) from exc
    if cell.size != 2 or not np.all(np.isfinite(cell)):
        raise ValueError(
            f"robot_grid_xy must be two finite integers (px, py), got {robot_grid_xy!r}"
        )
    if cell[0] != math.floor(cell[0]) or cell[1] != math.floor(cell[1]):
        raise ValueError(
            f"robot_grid_xy must hold integer cell indices, got {robot_grid_xy!r}"
        )
    px, py = int(cell[0]), int(cell[1])
    if not (0 <= px < field.shape[1] and 0 <= py < field.shape[0]):
        raise ValueError(
            f"robot_grid_xy=({px}, {py}) is outside the {field.shape[1]}x{field.shape[0]} "
            "field (indices are (column, row))"
        )

    offset_x, offset_y, values, distances = _neighbour_window(field, px, py, radius, step)
    if values.size < _MIN_SAMPLES:
        logger.debug(
            "descent_direction at (%d, %d): only %d valid sample(s) within %.3f m, "
            "need %d",
            px,
            py,
            int(values.size),
            radius,
            _MIN_SAMPLES,
        )
        return None

    design = np.column_stack([offset_x, offset_y, np.ones_like(offset_x)])
    weights = radius - distances
    sqrt_weights = np.sqrt(weights)[:, np.newaxis]
    try:
        coefficients, _, rank, _ = np.linalg.lstsq(
            design * sqrt_weights, values * sqrt_weights[:, 0], rcond=None
        )
    except np.linalg.LinAlgError as exc:  # pragma: no cover - lstsq 极少抛出
        logger.debug("plane fit failed at (%d, %d): %s", px, py, exc)
        return None
    if rank < 2 or not np.all(np.isfinite(coefficients)):
        # 所有样本都落在一条直线上：垂直于该线的梯度分量并非由数据决定，
        # 因此任何答案都是编造出来的。
        logger.debug(
            "descent_direction at (%d, %d): degenerate fit (rank %d), no heading",
            px,
            py,
            int(rank),
        )
        return None

    slope_x, slope_y = float(coefficients[0]), float(coefficients[1])
    if slope_x == 0.0 and slope_y == 0.0:
        logger.debug("descent_direction at (%d, %d): field is locally flat", px, py)
        return None
    return float(math.atan2(-slope_y, -slope_x))


def predict_path(
    field_values: ArrayLike,
    start_world_xy: Sequence[float],
    start_yaw: float,
    *,
    resolution: float,
    steps: int = 2,
    step_size: float = 0.1,
    radius_meters: float = 0.3,
) -> list[tuple[float, float, float]]:
    """把下降估计向前推演，预览距离场会把机器人带向何处。

    该预览不是规划：每一步都会在上一步落点处重新拟合平面，因此结果是纯梯度跟随者
    在接下来 ``steps * step_size`` 米内会产生的轨迹。控制器用它来决定*是否*采纳
    该下降方向——例如检查预览是否保持在走廊内，或把「跟随距离场」与「保持当前
    航向」作比较。

    第 ``0`` 步很特殊。它*记录*的航向是局部拟合出的下降方向，但它的*运动*沿
    ``start_yaw``。这种不对称正是关键：机器人在物理上朝向 ``start_yaw``，无法瞬移
    到理想方向，因此第一次推进必须反映现实，而报告的航向仍然显示距离场想要的方向。
    从第 ``1`` 步起，运动与航向都是同一个下降方向。

    当后续某一步找不到有效邻域时——它漂进了不可达的口袋区域，或漂出了窗口边缘的
    数据圆盘——会沿用上一步的方向而不是中止。单次拟合缺失是采样假象，并非走廊结束
    的证据，在那里截断预览会让它随机器人移动在两点与三点之间闪烁。沿用不会无限
    循环，因为步数计数器始终递增；当*从未*算出过任何方向时就没有可沿用的东西，
    预览便会停止。

    参数:
        field_values: 距离（单位米）的二维数组，位于模块 docstring 所述的坐标系。
        start_world_xy: 该坐标系中的起始位置 ``(x, y)``，单位米。
        start_yaw: 机器人实际朝向的航向，单位弧度；仅用于第一次推进。
        resolution: 单元尺寸，单位米。
        steps: 要生成的预览点数；``0`` 返回空列表。
        step_size: 预览点之间的距离，单位米。让它与 ``resolution`` 相当——小很多
            则相邻拟合几乎相同，大很多则预览会穿墙。
        radius_meters: 传给 :func:`descent_direction` 的拟合半径。

    返回:
        一个 ``(x, y, heading)`` 元组列表，长度至多为 ``steps``；当至少能算出一个
        航向时，总是从 ``start_world_xy`` 开始。空列表意味着第一个位置就没有产出
        可用航向。

    异常:
        TypeError: 若 ``field_values`` 不是数字的 array-like，或 ``start_world_xy``
            不是数字对。
        ValueError: 若 ``field_values`` 不是二维、``resolution``、``step_size``
            或 ``radius_meters`` 不是正有限数、``steps`` 为负、``start_yaw`` 非
            有限，或 ``start_world_xy`` 落在距离场之外。

    示例:
        >>> import numpy as np
        >>> resolution = 0.1
        >>> rows, cols = np.mgrid[0:21, 0:21]
        >>> cone = resolution * np.hypot(cols - 10, rows - 10)
        >>> path = predict_path(cone, (1.05, 1.65), -math.pi / 2,
        ...                     resolution=resolution, steps=3, step_size=0.2,
        ...                     radius_meters=0.35)
        >>> len(path)
        3
        >>> [round(math.degrees(heading), 1) for _, _, heading in path]
        [-90.0, -90.0, -90.0]
        >>> [round(y, 3) for _, y, _ in path]
        [1.65, 1.45, 1.25]
    """
    field = _as_field(field_values)
    step = _positive_float(resolution, "resolution")
    advance = _positive_float(step_size, "step_size")
    radius = _positive_float(radius_meters, "radius_meters")
    yaw = _finite_float(start_yaw, "start_yaw")
    if isinstance(steps, bool) or not isinstance(steps, (int, np.integer)):
        raise TypeError(f"steps must be an integer, got {type(steps).__name__}: {steps!r}")
    if int(steps) < 0:
        raise ValueError(f"steps must be >= 0, got {steps!r}")
    count = int(steps)

    x, y = _as_xy(start_world_xy, "start_world_xy")
    height, width = field.shape
    start_px, start_py = math.floor(x / step), math.floor(y / step)
    if not (0 <= start_px < width and 0 <= start_py < height):
        raise ValueError(
            f"start_world_xy=({x}, {y}) maps to cell ({start_px}, {start_py}), outside the "
            f"{width}x{height} field; subtract the field origin first"
        )

    path: list[tuple[float, float, float]] = []
    previous_direction: float | None = None
    index = 0
    while index < count:
        px, py = math.floor(x / step), math.floor(y / step)
        if not (0 <= px < width and 0 <= py < height):
            logger.debug("predict_path step %d left the field at (%.3f, %.3f)", index, x, y)
            break

        direction = descent_direction(field, (px, py), resolution=step, radius_meters=radius)
        if direction is None:
            if previous_direction is None:
                logger.debug("predict_path step %d has no valid neighbourhood", index)
                break
            # 采样假象：沿用上一个可信的航向，而不是截断预览。
            logger.debug(
                "predict_path step %d has no valid neighbourhood; reusing %.3f rad",
                index,
                previous_direction,
            )
            direction = previous_direction

        path.append((x, y, direction))
        heading = yaw if index == 0 else direction
        x += advance * math.cos(heading)
        y += advance * math.sin(heading)
        previous_direction = direction
        index += 1

    logger.debug("predict_path produced %d point(s)", len(path))
    return path


def projected_motion_distance(
    field_values: ArrayLike,
    world_xy: Sequence[float],
    yaw: float,
    *,
    resolution: float,
    radius_meters: float = 0.3,
    samples: int = 16,
) -> float:
    """估计机器人沿当前航向还能走多远。

    它回答的是与 :func:`descent_direction` 不同的问题。下降方向说明机器人*应该*去
    哪里；而速度规划器还需要知道它当前走出的距离中有多少是真正的进展。具体而言：
    「如果我继续沿着 ``yaw`` 指向，在它不再是一种高效接近目标的方式之前我能走
    多少米？」

    该估计在圆周上均匀枚举 ``samples`` 个方向，并对每个方向在 ``radius_meters``
    处采样距离场。一个样本会得到两个量：

    ``motion``
        位移在当前航向上的投影，``radius * cos(theta - yaw)``——该位移代表多少
        前进距离；
    ``descent``
        ``field[centre] - field[sample]``——它让目标近了多少。

    两者之比就是该方向的效率：``1.0`` 表示径直朝向目标，更大表示斜向。取最佳比值
    并用它缩放运动量，就把观测到的效率外推一步，即投影行进距离。只有既前进
    （``motion > 0``）又有进展（``descent > 0``）的样本参与；若都不满足，答案为
    ``0.0``——该航向已耗尽，机器人应当转向。

    枚举方向而非单元（原实现的做法）消除了两个偏差：单元版本采样的是网格恰好落在
    圆盘内的东西，因此角分辨率随 ``resolution`` 变化且在坐标轴方向最密。固定的
    角度扫描在任何分辨率下都给出相同的 ``samples`` 个方向。

    .. note::
        当某个方向与局部下降方向正交时，该比值是无界的，因此返回值是一种*启发式*，
        而非保证的净空距离：在指令运动前，请把它夹取到机器人自身的运动学与规划限制
        之内。``0.0`` 是唯一带有硬含义的值（「没有任何前进方向能带来进展」）。

    参数:
        field_values: 距离（单位米）的二维数组，位于模块 docstring 所述的坐标系。
        world_xy: 该坐标系中的位置 ``(x, y)``，单位米。
        yaw: 当前航向，单位弧度；投影轴。
        resolution: 单元尺寸，单位米。
        radius_meters: 每个候选方向探测的距离，单位米。半径越大看得越远，但越依赖
            距离场在局部是线性的。
        samples: 要枚举的方向数；至少 1。16 给出 22.5 度的角分辨率，这比差速驱动
            通常能利用的更精细。

    返回:
        投影行进距离，单位米，``>= 0.0``。``0.0`` 表示没有任何候选方向既前进又使
        距离场下降。

    异常:
        TypeError: 若 ``field_values`` 不是数字的 array-like，或 ``world_xy``
            不是数字对。
        ValueError: 若 ``field_values`` 不是二维、``resolution`` 或
            ``radius_meters`` 不是正有限数、``samples`` 不是 ``>= 1`` 的整数、
            ``yaw`` 非有限，或 ``world_xy`` 落在距离场之外。

    示例:
        >>> import numpy as np
        >>> resolution = 0.1
        >>> rows, cols = np.mgrid[0:21, 0:21]
        >>> cone = resolution * np.hypot(cols - 10, rows - 10)
        >>> towards = projected_motion_distance(cone, (1.05, 1.65), -math.pi / 2,
        ...                                     resolution=resolution)
        >>> away = projected_motion_distance(cone, (1.05, 1.65), math.pi / 2,
        ...                                  resolution=resolution)
        >>> towards > 0.0, away
        (True, 0.0)
    """
    field = _as_field(field_values)
    step = _positive_float(resolution, "resolution")
    radius = _positive_float(radius_meters, "radius_meters")
    heading = _finite_float(yaw, "yaw")
    count = _positive_int(samples, "samples")

    x, y = _as_xy(world_xy, "world_xy")
    height, width = field.shape
    px, py = math.floor(x / step), math.floor(y / step)
    if not (0 <= px < width and 0 <= py < height):
        raise ValueError(
            f"world_xy=({x}, {y}) maps to cell ({px}, {py}), outside the "
            f"{width}x{height} field; subtract the field origin first"
        )

    centre = float(field[py, px])
    if not math.isfinite(centre) or centre < 0.0:
        logger.debug(
            "projected_motion_distance at (%d, %d): the cell itself has no valid "
            "distance (%.3f)",
            px,
            py,
            centre,
        )
        return 0.0

    angles = np.arange(count, dtype=np.float64) * (_TWO_PI / count)
    probe_x = px + np.rint(radius * np.cos(angles) / step).astype(np.int64)
    probe_y = py + np.rint(radius * np.sin(angles) / step).astype(np.int64)
    inside = (probe_x >= 0) & (probe_x < width) & (probe_y >= 0) & (probe_y < height)
    if not np.any(inside):
        logger.debug("projected_motion_distance at (%d, %d): no probe cell in bounds", px, py)
        return 0.0

    values = field[probe_y[inside], probe_x[inside]]
    live_angles = angles[inside]
    descent = centre - values
    motion = radius * np.cos(live_angles - heading)
    usable = (
        np.isfinite(values)
        & (values >= 0.0)
        & (descent > _MIN_DESCENT)
        & (motion > 0.0)
    )
    if not np.any(usable):
        logger.debug(
            "projected_motion_distance at (%d, %d): heading %.3f rad makes no forward "
            "progress within %.3f m",
            px,
            py,
            heading,
            radius,
        )
        return 0.0

    ratio = motion[usable] / descent[usable]
    projected = motion[usable] * ratio
    best = int(np.argmax(projected))
    logger.debug(
        "projected_motion_distance: best ratio %.3f over %.3f m of motion -> %.3f m",
        float(ratio[best]),
        float(motion[usable][best]),
        float(projected[best]),
    )
    return float(projected[best])
