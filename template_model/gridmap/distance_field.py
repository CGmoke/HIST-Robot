"""二维导航栅格上的度量距离场与带制动的速度掩码。

距离场为楼层平面的每个单元回答一个问题：*如果机器人必须绕开墙壁，从这里到目标
有多远？* 答案不是欧氏距离，而是最便宜路径的长度，仅凭这一个数组就足以驱动机器
人：在任何位姿下，朝值最小的相邻单元迈一步，就是在沿最短路径前进。
:mod:`reusable_model.gridmap.descent` 会把该数组转换为平滑的航向。

有三个理念让本模块不止是教科书式的 Dijkstra：

*代价是时间，而非长度。*
    传播代价是 ``step_length_metres / cell_speed``。因此一个单元不仅表示「能否到
    达」，还表示「在这里能开多快」，由此得到的距离场会偏好宽阔走廊，即使狭窄走廊
    在几何上更短。把某单元的速度设为零就把它从图中彻底移除。

*速度由到最近墙壁的距离推导。*
    :meth:`DistanceField.generate` 会构建一个 ``[0, 1]`` 的速度掩码，其中单元的
    取值是在那里安全使用的全速比例。距离区域边界小于 ``robot_radius`` 的单元被
    直接阻挡；处于 ``robot_radius`` 与 ``robot_radius + brake_distance`` 之间的
    单元获得线性斜坡，因此尊重该掩码的控制器总能先刹停再触及墙壁。

*全局场，局部细化。*
    全局场对每个目标只生成一次。随后 :meth:`DistanceField.recompute_local` 会在
    机器人周围的较小窗口内以实时传感器障碍重新求解，并保留全局值作为边界条件。
    这正是动态避障足够廉价的原因：每个控制周期只需重访几千个单元，而非整张地图。

距离场拥有**自己的**分辨率，并被刻意与它取自的地图分辨率解耦。两个方向上的
``scale_factor`` 讨论见 :meth:`DistanceField.generate`。

坐标约定原样继承自 :mod:`reusable_model.gridmap.occupancy`：``array[py, px]``，行 ``0`` 对应
世界 y 最小值、列 ``0`` 对应世界 x 最小值，因此世界 y 随行索引增大。除了
:meth:`DistanceField.save_visualization`（它必须遵守顶行优先的图像约定）之外，
任何地方都不做翻转。

依赖：:mod:`numpy` 及标准库。OpenCV 是可选的，由
:func:`reusable_model.gridmap.regions.merge_regions_by_dilation`（在把若干已标记区域焊接在
一起时使用）与 :meth:`DistanceField.save_visualization` 惰性导入。
"""

from __future__ import annotations

import heapq
import logging
import math
import time
from typing import Any, ClassVar, Iterable, Sequence

import numpy as np
from numpy.typing import ArrayLike

from .occupancy import GridGeometry
from .regions import merge_regions_by_dilation

logger = logging.getLogger(__name__)

__all__ = [
    "DistanceField",
    "create_distance_field_for_region",
]

#: 8 邻域偏移，形式为 ``(dy, dx, step_length_in_cells)``。对角线的代价为
#: ``sqrt(2)``，使得传播出的值近似米制距离而非棋盘距离；若用 4 邻域传播，每次
#: 对角移动都会被高估约 41%，并让下降方向吸附到坐标轴上。该近似仍是各向异性的：
#: 8 连通栅格度量对真实欧氏距离最多高估约 8%，在偏离坐标轴 22.5 度时最差。这是
#: 栅格的代价，也是为什么在需要厘米级精确距离的场合不应使用该距离场。
_NEIGHBOURS_8: tuple[tuple[int, int, float], ...] = (
    (-1, -1, math.sqrt(2.0)),
    (-1, 0, 1.0),
    (-1, 1, math.sqrt(2.0)),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (1, -1, math.sqrt(2.0)),
    (1, 0, 1.0),
    (1, 1, math.sqrt(2.0)),
)

#: 当调用方未覆盖时，用于把相邻区域焊接在一起的 ``(kernel_size, iterations)``。
#: 7x7 元素应用两次可向外触及 6 个像素，能桥接典型门框而不会吞掉家具。
_DEFAULT_DILATION: tuple[int, int] = (7, 2)

#: 切片掩码时，低于该速度值即视为「被阻挡」。掩码是浮点斜坡，因此精确的
#: ``== 0`` 判断会保留那些只是在挪动的单元；该阈值让掩码的二值视图与连续视图保持
#: 一致。
_SPEED_EPS: float = 0.01

#: 收集与全局场在窗口内最小值并列的单元时所允许的米数裕量。两条不同最短路径上
#: 累积的 ``float32`` 代价会因舍入噪声而不同，没有该容差时，同样好的种子中只有
#: 一个能存活下来。
_SEED_VALUE_TOLERANCE: float = 0.1


def _require_cv2() -> Any:
    """按需导入 OpenCV，并给出可操作的错误消息。

    返回:
        导入的 :mod:`cv2` 模块。

    异常:
        ImportError: 若未安装 OpenCV。只有
            :meth:`DistanceField.save_visualization` 需要它；生成与查询距离场
            无需它即可工作。
    """
    try:
        import cv2  # noqa: PLC0415 - 惰性、可选依赖
    except ImportError as exc:  # pragma: no cover - 取决于环境
        raise ImportError(
            "this helper needs OpenCV; install it with `pip install opencv-python` "
            "(generating and querying a distance field works without it)"
        ) from exc
    return cv2


def _finite_float(value: Any, name: str) -> float:
    """把 ``value`` 转换为有限的 Python ``float``。

    参数:
        value: 候选数字。``bool`` 会被拒绝，因为 ``True`` 在 Python 中是合法的
            ``int``，但它永远不是有意义的米制值。
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


def _non_negative_float(value: Any, name: str) -> float:
    """把 ``value`` 转换为非负的有限 ``float``。

    参数:
        value: 候选数字。
        name: 错误消息中使用的参数名。

    返回:
        以 ``float`` 表示的值。

    异常:
        TypeError: 若 ``value`` 不是实数。
        ValueError: 若 ``value`` 不是有限数或 ``< 0``。
    """
    as_float = _finite_float(value, name)
    if as_float < 0.0:
        raise ValueError(f"{name} must be >= 0, got {value!r}")
    return as_float


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


def _pixel_from_world(geometry: GridGeometry, x: float, y: float) -> tuple[int, int]:
    """把世界坐标点映射到 ``geometry`` 的单元索引，不夹取。

    这是 :meth:`reusable_model.gridmap.occupancy.OccupancyGrid.world_to_pixel` 的无数组孪生：
    约定相同（单元 ``(0, 0)`` 位于世界 x、y 的最小值处，行索引随世界 y 增大），但
    它直接在裸的 :class:`GridGeometry` 上操作，因此不必仅为转换一个点而构造像素
    数组。它还刻意*不*夹取：距离场查询必须能够区分「在场外」与「在场边界上」，而
    夹取会抹掉这一区别。

    参数:
        geometry: 要转换到的坐标系。
        x: 世界 x，单位米。
        y: 世界 y，单位米。

    返回:
        ``(px, py)`` 单元索引；可能为负或超出栅格。

    异常:
        TypeError: 若 ``x`` 或 ``y`` 不是实数。
        ValueError: 若 ``x`` 或 ``y`` 不是有限数。
    """
    fx, fy = _as_xy((x, y), "world point")
    return (
        math.floor((fx - geometry.origin_x) / geometry.resolution),
        math.floor((fy - geometry.origin_y) / geometry.resolution),
    )


def _world_from_pixel(geometry: GridGeometry, px: float, py: float) -> tuple[float, float]:
    """把（可能为小数的）单元索引映射到其世界坐标位置。

    与 :meth:`reusable_model.gridmap.occupancy.OccupancyGrid.pixel_to_world` 对应：返回的点是
    单元的*中心*，即在其左下角之上、之右各半个分辨率处。

    参数:
        geometry: 要转换的坐标系。
        px: 列索引，可以是小数。
        py: 行索引，可以是小数。

    返回:
        世界 ``(x, y)`` 坐标，单位米。

    异常:
        TypeError: 若 ``px`` 或 ``py`` 不是实数。
        ValueError: 若 ``px`` 或 ``py`` 不是有限数。
    """
    fx, fy = _as_xy((px, py), "cell index")
    return (
        geometry.origin_x + (fx + 0.5) * geometry.resolution,
        geometry.origin_y + (fy + 0.5) * geometry.resolution,
    )


def _as_2d_array(value: Any, name: str, dtype: Any = None) -> np.ndarray:
    """把 ``value`` 转换为连续的二维数组。

    参数:
        value: array-like 输入。
        name: 错误消息中使用的参数名。
        dtype: 可选的转换后 dtype；``None`` 保持输入 dtype。

    返回:
        连续的 ``(H, W)`` 数组。

    异常:
        TypeError: 若 ``value`` 不是 array-like，或无法转换为 ``dtype``。
        ValueError: 若 ``value`` 不是二维。
    """
    try:
        array = np.asarray(value) if dtype is None else np.asarray(value, dtype=dtype)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be array-like, got {value!r}") from exc
    if array.ndim != 2:
        raise ValueError(f"{name} must be a 2D (height, width) array, got shape {array.shape}")
    return np.ascontiguousarray(array)


def _as_cost_mask(value: Any, name: str = "cost_mask") -> np.ndarray:
    """把 ``value`` 转换为 ``float32`` 的速度/代价掩码。

    布尔掩码也可接受并变为 ``0.0``/``1.0``，这让「空闲或阻挡」与「有多快」成为
    同一参数的两个视图。

    参数:
        value: 逐单元速度的二维 array-like。
        name: 错误消息中使用的参数名。

    返回:
        连续的 ``(H, W)`` ``float32`` 数组。非有限值条目按原样保留；传播会把它们
        视为不可通行。

    异常:
        TypeError: 若 ``value`` 不是数字的 array-like。
        ValueError: 若 ``value`` 不是二维。
    """
    return _as_2d_array(value, name, dtype=np.float32)


def _normalise_seeds(
    seeds: Any,
    shape: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """把两种可接受的种子表示形式展平为坐标数组。

    之所以支持两种形式，是因为两个使用场景的种子数量相差若干个数量级：

    *一串 ``(x, y)`` 或 ``(x, y, cost)`` 对*
        适合局部重算或单个目标产生的那一小撮种子。
    *形状与代价掩码相同的二维布尔数组*
        当*每个*被阻挡单元都是种子时需要，这正是到边界距离传播的构建方式。在真实
        地图上那是数十万个单元，为每个单元构造一个 Python 元组的开销超过传播本身。

    参数:
        seeds: 上述两种形式之一。
        shape: 种子所索引的代价掩码的 ``(height, width)``。

    返回:
        ``(rows, cols, costs)``：``int64`` 索引数组，外加一个 ``float64`` 初始代价
        数组，三者长度相等。

    异常:
        TypeError: 若 ``seeds`` 既不是形状正确的布尔数组，也不是坐标序列的可迭代
            对象。
        ValueError: 若某个种子格式错误、含非整数或负索引、落在掩码之外，或携带负
            代价。
    """
    height, width = shape
    if (
        isinstance(seeds, np.ndarray)
        and seeds.ndim == 2
        and seeds.dtype == np.bool_
        and seeds.shape == shape
    ):
        rows, cols = np.nonzero(seeds)
        return (
            rows.astype(np.int64),
            cols.astype(np.int64),
            np.zeros(rows.size, dtype=np.float64),
        )

    if isinstance(seeds, (str, bytes)) or not isinstance(seeds, Iterable):
        raise TypeError(
            "seeds must be an iterable of (x, y) / (x, y, cost) pairs, or a 2D "
            f"boolean array shaped {shape}; got {type(seeds).__name__}: {seeds!r}"
        )

    cols_list: list[int] = []
    rows_list: list[int] = []
    costs_list: list[float] = []
    for index, seed in enumerate(seeds):
        if isinstance(seed, (str, bytes)) or not isinstance(seed, Iterable):
            raise TypeError(
                f"seeds[{index}] must be a (x, y) or (x, y, cost) sequence, got {seed!r}"
            )
        try:
            values = np.asarray(seed, dtype=float).reshape(-1)
        except (TypeError, ValueError) as exc:
            raise TypeError(f"seeds[{index}] must hold numbers, got {seed!r}") from exc
        if values.size not in (2, 3) or not np.all(np.isfinite(values)):
            raise ValueError(
                f"seeds[{index}] must be (x, y) or (x, y, cost) finite numbers, got {seed!r}"
            )
        if values[0] != math.floor(values[0]) or values[1] != math.floor(values[1]):
            raise ValueError(
                f"seeds[{index}] must hold integer cell indices, got {seed!r}"
            )
        sx, sy = int(values[0]), int(values[1])
        cost = float(values[2]) if values.size == 3 else 0.0
        if cost < 0.0:
            raise ValueError(f"seeds[{index}] initial cost must be >= 0, got {cost!r}")
        if not (0 <= sx < width and 0 <= sy < height):
            raise ValueError(
                f"seeds[{index}] = ({sx}, {sy}) is outside the {width}x{height} cost mask"
            )
        cols_list.append(sx)
        rows_list.append(sy)
        costs_list.append(cost)

    return (
        np.asarray(rows_list, dtype=np.int64),
        np.asarray(cols_list, dtype=np.int64),
        np.asarray(costs_list, dtype=np.float64),
    )


def _parse_dilation(dilation: Sequence[int]) -> tuple[int, int]:
    """校验一个 ``(kernel_size, iterations)`` 对。

    参数:
        dilation: 两个整数组成的序列。

    返回:
        以 Python int 表示的 ``(kernel_size, iterations)``。

    异常:
        TypeError: 若 ``dilation`` 不是两个整数组成的序列。
        ValueError: 若 ``kernel_size`` 不是奇数且至少为 3，或 ``iterations`` 不是
            至少为 1。
    """
    if isinstance(dilation, (str, bytes)) or not isinstance(dilation, Iterable):
        raise TypeError(
            f"dilation must be a (kernel_size, iterations) pair of integers, got {dilation!r}"
        )
    values = list(dilation)
    if len(values) != 2 or any(
        isinstance(value, bool) or not isinstance(value, (int, np.integer)) for value in values
    ):
        raise TypeError(
            f"dilation must be a (kernel_size, iterations) pair of integers, got {dilation!r}"
        )
    kernel_size, iterations = int(values[0]), int(values[1])
    if kernel_size < 3 or kernel_size % 2 == 0:
        raise ValueError(
            f"dilation kernel_size must be an odd integer >= 3 pixels, got {values[0]!r}"
        )
    if iterations < 1:
        raise ValueError(f"dilation iterations must be >= 1, got {values[1]!r}")
    return kernel_size, iterations


def _as_1d(values: ArrayLike, name: str) -> np.ndarray:
    """校验一个由有限数字组成的一维 array-like。

    参数:
        values: 待检查的 array-like。
        name: 错误消息中使用的参数名。

    返回:
        形状为 ``(n,)`` 的 ``float64`` 数组。

    异常:
        TypeError: 若 ``values`` 不是数字的 array-like。
        ValueError: 若结果不是一维或含非有限值。
    """
    try:
        array = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be array-like of numbers, got {values!r}") from exc
    if array.ndim != 1:
        raise ValueError(f"{name} must be 1D, got shape {array.shape}")
    if array.size and not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite numbers, got {values!r}")
    return array


class DistanceField:
    """到单一目标的最短路径距离，外加它构建时所用的速度掩码。

    该对象刻意是有状态且单目标的：:meth:`generate` 固定目标、区域与坐标系，之后
    每次查询都针对该快照作答。再次调用 :meth:`generate` 可重新设定目标；之前的
    距离场会被丢弃。

    值以 ``float32`` 米存储。无法到达目标的单元保存 :attr:`UNREACHABLE` 而非
    ``inf``——原因见该常量。

    属性:
        resolution: 距离场的单元尺寸，单位米，在构造时固定。

    示例:
        >>> import numpy as np
        >>> from reusable_model.gridmap.occupancy import GridGeometry
        >>> geometry = GridGeometry(0.0, 0.0, 0.1, 21, 21)
        >>> field = DistanceField(resolution=0.1)
        >>> field.generate(np.ones((21, 21), bool), (1.05, 1.05), geometry=geometry)
        >>> round(field.distance_at_world(1.05, 1.05), 6)
        0.0
        >>> d = field.distance_at_world(0.05, 1.05)   # 位于目标左侧 1 m 处
        >>> 0.9 < d < 1.15
        True
    """

    #: 保存在无法到达目标的单元中的值。
    #:
    #: 使用负的哨兵值而非 ``inf``，出于三个实际原因：
    #:
    #: * ``inf`` 无法在距离场通常最终落入的整数与定点缓冲区中表示（共享内存、
    #:   ``uint16`` costmap、图像文件），而 ``-1.0`` 经过 ``float32`` ``.npy``
    #:   往返后能逐位保持不变；
    #: * ``inf`` 会污染算术——一个不可达邻居就足以把最小二乘平面拟合变成 ``nan``，
    #:   而 ``nan`` 比较会静默返回 ``False``，而不是响亮地报错；
    #: * 有效性检查变成一次廉价的 ``value < 0`` 比较，而无需在每个使用处调用
    #:   ``np.isfinite``。
    #:
    #: 由于真实距离永远不会为负，该哨兵值绝不会与合法值混淆。
    UNREACHABLE: ClassVar[float] = -1.0

    def __init__(self, *, resolution: float = 0.06) -> None:
        """创建一个单元尺寸固定的空距离场。

        参数:
            resolution: 距离场的单元尺寸，单位米。它*独立*于距离场将要取自的地图
                分辨率；两者的协调方式见 :meth:`generate`。

        异常:
            TypeError: 若 ``resolution`` 不是实数。
            ValueError: 若 ``resolution`` 不是有限数或 ``<= 0``。

        示例:
            >>> DistanceField(resolution=0.05).resolution
            0.05
            >>> DistanceField().resolution
            0.06
        """
        self.resolution: float = _positive_float(resolution, "resolution")
        self._geometry: GridGeometry | None = None
        self._distances: np.ndarray | None = None
        self._speed_mask: np.ndarray | None = None
        self._boundary_mask: np.ndarray | None = None
        self._target_grid: tuple[int, int] | None = None
        self._obstacle_timestamps: np.ndarray | None = None

    # ------------------------------------------------------------------
    # 已生成状态的只读视图
    # ------------------------------------------------------------------
    @property
    def geometry(self) -> GridGeometry | None:
        """距离场的坐标系；在 :meth:`generate` 运行前为 ``None``。

        原点即距离场包围盒左下角的世界位置，因此仅凭距离场自身的 ``geometry`` 就
        足以在世界坐标与距离场单元之间转换。

        返回:
            距离场的 :class:`~reusable_model.gridmap.occupancy.GridGeometry`。

        示例:
            >>> DistanceField().geometry is None
            True
        """
        return self._geometry

    @property
    def distances(self) -> np.ndarray | None:
        """实时的 ``(H, W)`` ``float32`` 距离数组，或 ``None``。

        请把结果视为只读；需要私有副本时使用 :meth:`to_array`。原地修改它会悄悄
        让距离场与其计算时所用的速度掩码失去同步。

        返回:
            以米为单位的距离数组，无法到达目标的单元为 :attr:`UNREACHABLE`。

        示例:
            >>> DistanceField().distances is None
            True
        """
        return self._distances

    @property
    def speed_mask(self) -> np.ndarray | None:
        """``[0, 1]`` 的带制动斜坡速度掩码，或 ``None``。

        返回:
            ``(H, W)`` ``float32`` 数组：每个单元可安全使用的全速比例。

        示例:
            >>> DistanceField().speed_mask is None
            True
        """
        return self._speed_mask

    @property
    def boundary_mask(self) -> np.ndarray | None:
        """*未膨胀*、未制动的全局障碍集合，或 ``None``。

        这是距离场所针对区域的二值 ``1`` 内部 / ``0`` 外部视图，被重采样到距离场
        栅格上。它被与 :attr:`speed_mask` 分开保存，因为局部重规划需要这个硬几何：
        对已经制动的掩码再次膨胀会重复施加制动斜坡。

        返回:
            ``(H, W)`` 的 ``0.0`` 与 ``1.0`` ``float32`` 数组。

        示例:
            >>> DistanceField().boundary_mask is None
            True
        """
        return self._boundary_mask

    @property
    def target_grid(self) -> tuple[int, int] | None:
        """目标的 ``(px, py)`` 距离场单元，或 ``None``。

        返回:
            目标单元索引，保证其距离为 ``0.0``。

        示例:
            >>> DistanceField().target_grid is None
            True
        """
        return self._target_grid

    @property
    def shape(self) -> tuple[int, int] | None:
        """距离场数组的 ``(height, width)``，或 ``None``。

        返回:
            数组形状，与 ``geometry.height``/``geometry.width`` 一致。

        示例:
            >>> DistanceField().shape is None
            True
        """
        if self._distances is None:
            return None
        return int(self._distances.shape[0]), int(self._distances.shape[1])

    def _require(self) -> GridGeometry:
        """返回坐标系；若距离场尚未生成则抛出异常。

        返回:
            距离场的 :class:`GridGeometry`。

        异常:
            ValueError: 若 :meth:`generate`（或 :meth:`from_array`）尚未运行。
        """
        if self._geometry is None or self._distances is None:
            raise ValueError(
                "this DistanceField is empty; call generate() or from_array() first"
            )
        return self._geometry

    # ------------------------------------------------------------------
    # 生成
    # ------------------------------------------------------------------
    def generate(
        self,
        mask: ArrayLike,
        target_world: Sequence[float],
        *,
        geometry: GridGeometry,
        brake_distance: float = 1.0,
        robot_radius: float = 0.0,
        region_ids: Sequence[int] | None = None,
        dilation: Sequence[int] | None = None,
    ) -> None:
        """为某一个（或若干）区域内的单个目标构建距离场。

        流程为：

        1. 把 ``mask`` 变为布尔可通行掩码，可选地通过门洞把若干已标记区域焊接在
           一起；
        2. 裁剪到该掩码的包围盒，这样某个房间里的目标就不必为建筑的其他部分付出
           代价；
        3. 以 :attr:`resolution` 把裁剪结果重采样到距离场栅格上；
        4. 推导带制动斜坡的速度掩码 (:meth:`_build_speed_mask`)；
        5. 在该掩码上从目标做一次单源 Dijkstra。

        **两种分辨率，一个缩放因子。** ``scale_factor =
        geometry.resolution / self.resolution`` 是每个地图像素对应的距离场单元
        数，它在两个方向上都会被应用：距离场单元 ``(px, py)`` 采样地图像素
        ``(round(px / scale_factor) + offset_x, ...)``，而地图像素 ``(mx, my)``
        落入距离场单元 ``(round((mx - offset_x) * scale_factor), ...)``。

        *距离场比地图更细*（``self.resolution < geometry.resolution``，
        ``scale_factor > 1``）：距离场比地图拥有更多单元。最近邻重采样会重复地图
        像素，因此不会创造新信息——好处是对梯度更密集的采样，这会让
        :func:`reusable_model.gridmap.descent.descent_direction` 中的平面拟合更平滑。代价
        是内存与传播时间，二者都随 ``scale_factor ** 2`` 增长。

        *距离场比地图更粗*（``self.resolution > geometry.resolution``，
        ``scale_factor < 1``）：最近邻重采样*丢弃*地图像素，因此一像素厚的墙可能
        完全消失，距离场会径直流穿它。只有当地图最薄的障碍也有若干像素宽时才降采样，
        或先膨胀障碍图层。

        参数:
            mask: 形状为 ``(geometry.height, geometry.width)`` 的二维数组。当
                ``region_ids=None`` 时它是可通行掩码（非零 = 空闲）；否则它是区域
                id 的标签数组，其中 ``0`` 表示「无区域」。
            target_world: 目标的世界 ``(x, y)`` 位置，单位米。它必须落在（可选的
                膨胀后）区域掩码之内。
            geometry: ``mask`` 的坐标系。
            brake_distance: 每堵墙前的线性速度斜坡长度，单位米。必须 ``>= 0``；
                ``0`` 会把斜坡退化为 ``robot_radius`` 处的硬阶跃。
            robot_radius: 距区域边界小于该值的单元被直接阻挡，单位米。原始实现在此
                硬编码 ``0.0``，并附有关于「避免不可达边缘点」的注释，而这正是权衡
                所在：正的半径让距离场对机器人的足印诚实，但当狭窄通道*以及目标
                本身*靠近墙壁时，它可能把它们封死。``0.0`` 让每个空闲单元都可到达，
                并把足印留给局部重规划器处理，反正它会在实时障碍周围重新膨胀。
                默认 ``0.0``。
            region_ids: 要包含的区域 id。给出时，``mask`` 被当作标签数组，所选区域
                会用 :func:`reusable_model.gridmap.regions.merge_regions_by_dilation` 合并，
                后者会把两个膨胀区域重叠的单元——即门洞——加回来。
            dilation: 用于合并的 ``(kernel_size, iterations)`` 覆盖值；默认
                ``(7, 2)``。仅在配合 ``region_ids`` 时才有意义。

        返回:
            ``None``。结果存储在实例上，并通过 :attr:`distances`、
            :attr:`speed_mask`、:attr:`boundary_mask`、:attr:`target_grid` 与
            :attr:`geometry` 访问。

        异常:
            TypeError: 若 ``mask`` 不是 array-like、``geometry`` 不是
                :class:`GridGeometry`，或 ``target_world`` 不是数字对。
            ValueError: 若 ``mask`` 不是二维或其形状与坐标系不一致、掩码中没有任何
                可通行单元、``brake_distance``/``robot_radius`` 超出范围、给出了
                ``dilation`` 却没有 ``region_ids``、``target_world`` 未落在区域内，
                或目标单元被 ``robot_radius`` 的膨胀阻挡。
            RuntimeError: 若要求区域合并但缺少 OpenCV。

        示例:
            >>> import numpy as np
            >>> geometry = GridGeometry(0.0, 0.0, 0.5, 4, 4)
            >>> mask = np.ones((4, 4), bool)
            >>> mask[:, 3] = False            # 右侧的一堵墙
            >>> field = DistanceField(resolution=0.5)
            >>> field.generate(mask, (0.25, 0.25), geometry=geometry,
            ...                brake_distance=0.0)
            >>> field.shape, field.target_grid   # 裁剪之外多出一个安全单元
            ((5, 4), (0, 0))
            >>> field.distances[0, 0], field.distances[0, 2]
            (0.0, 1.0)
        """
        if not isinstance(geometry, GridGeometry):
            raise TypeError(
                f"geometry must be a GridGeometry, got {type(geometry).__name__}: {geometry!r}"
            )
        brake = _non_negative_float(brake_distance, "brake_distance")
        radius = _non_negative_float(robot_radius, "robot_radius")
        target_x, target_y = _as_xy(target_world, "target_world")

        array = _as_2d_array(mask, "mask")
        if array.shape != (geometry.height, geometry.width):
            raise ValueError(
                f"mask shape {array.shape} does not match geometry (height, width) "
                f"{(geometry.height, geometry.width)}"
            )

        if region_ids is None:
            if dilation is not None:
                raise ValueError(
                    f"dilation={dilation!r} only applies when region_ids is given; "
                    "pass region_ids or drop dilation"
                )
            region_mask = array != 0
        else:
            if isinstance(region_ids, (str, bytes)) or not isinstance(region_ids, Iterable):
                raise TypeError(f"region_ids must be a sequence of integers, got {region_ids!r}")
            ids = sorted({int(value) for value in region_ids})
            if not ids:
                raise ValueError("region_ids must contain at least one region id, got an empty sequence")
            kernel_size, iterations = (
                _DEFAULT_DILATION if dilation is None else _parse_dilation(dilation)
            )
            merged = merge_regions_by_dilation(
                array, ids, kernel_size=kernel_size, iterations=iterations
            )
            # ``merge_regions_by_dilation`` 返回的是把门洞单元重新标记后的
            # *完整* 标签数组，因此必须显式过滤掉未被选中的房间，否则距离场会
            # 毫无顾忌地穿过它们。
            region_mask = np.isin(merged, np.asarray(ids, dtype=merged.dtype))

        rows, cols = np.nonzero(region_mask)
        if rows.size == 0:
            raise ValueError(
                f"the mask has no traversable cell (region_ids={region_ids!r}); "
                "a distance field needs at least one"
            )
        min_x, max_x = int(cols.min()), int(cols.max())
        min_y, max_y = int(rows.min()), int(rows.max())

        # 每个地图像素对应的距离场单元数。关于大于或小于 1.0 时的行为，
        # 见 docstring。
        scale_factor = geometry.resolution / self.resolution
        field_width = int((max_x - min_x + 1) * scale_factor) + 1
        field_height = int((max_y - min_y + 1) * scale_factor) + 1
        if field_width < 1 or field_height < 1:
            raise ValueError(  # pragma: no cover - 不可达，作为不变量保留
                f"degenerate field size ({field_width}, {field_height}) from scale_factor "
                f"{scale_factor!r}"
            )

        # 距离场坐标系从裁剪框的左下角开始，因此距离场单元 (0, 0) 与地图像素
        # (min_x, min_y) 表示地面上同一平方米。
        self._geometry = GridGeometry(
            origin_x=geometry.origin_x + min_x * geometry.resolution,
            origin_y=geometry.origin_y + min_y * geometry.resolution,
            resolution=self.resolution,
            width=field_width,
            height=field_height,
        )

        mask_px, mask_py = _pixel_from_world(geometry, target_x, target_y)
        if (
            not (0 <= mask_px < geometry.width and 0 <= mask_py < geometry.height)
            or not bool(region_mask[mask_py, mask_px])
        ):
            raise ValueError(
                f"target_world=({target_x}, {target_y}) maps to mask cell "
                f"({mask_px}, {mask_py}), which is not a traversable cell of "
                f"region_ids={region_ids!r}; pick a goal inside the region"
            )

        target_px, target_py = _pixel_from_world(self._geometry, target_x, target_y)
        if not (0 <= target_px < field_width and 0 <= target_py < field_height):
            raise ValueError(  # pragma: no cover - 由包围盒构造保证不会发生
                f"target_world=({target_x}, {target_y}) maps to field cell "
                f"({target_px}, {target_py}), outside the {field_width}x{field_height} field"
            )

        speed_mask, boundary_mask = self._build_speed_mask(
            region_mask,
            offset_x=min_x,
            offset_y=min_y,
            scale_factor=scale_factor,
            field_width=field_width,
            field_height=field_height,
            brake_distance=brake,
            robot_radius=radius,
        )
        if speed_mask[target_py, target_px] <= 0.0:
            raise ValueError(
                f"the goal cell ({target_px}, {target_py}) was blocked by the "
                f"robot_radius={radius} inflation, so no cell could ever be "
                "reached; lower robot_radius or move the goal away from the wall"
            )

        self._speed_mask = speed_mask
        self._boundary_mask = boundary_mask
        self._target_grid = (target_px, target_py)
        self._obstacle_timestamps = np.zeros((field_height, field_width), dtype=np.float64)

        seed_mask = np.zeros((field_height, field_width), dtype=np.bool_)
        seed_mask[target_py, target_px] = True
        distances = self.dijkstra_multi_seed(seed_mask, speed_mask)
        reached = int(np.count_nonzero(np.isfinite(distances)))
        if reached == 0:  # pragma: no cover - 目标单元自身总是可达
            logger.warning("the goal cell produced no reachable cell at all")
        distances[~np.isfinite(distances)] = self.UNREACHABLE
        self._distances = distances

        logger.debug(
            "distance field %dx%d at %.3f m (scale_factor=%.3f) for target (%.3f, %.3f): "
            "%d/%d cells reached",
            field_width,
            field_height,
            self.resolution,
            scale_factor,
            target_x,
            target_y,
            reached,
            distances.size,
        )

    def _build_speed_mask(
        self,
        region_mask: np.ndarray,
        *,
        offset_x: int,
        offset_y: int,
        scale_factor: float,
        field_width: int,
        field_height: int,
        brake_distance: float,
        robot_radius: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """把区域掩码重采样到距离场网格，并在墙壁附近对速度做斜坡处理。

        会输出两个数组，它们之间的区别很重要：

        ``boundary_mask``
            区域在距离场分辨率下的二值 ``1``/``0`` 掩码，不含膨胀、不含斜坡。
            它是 :meth:`recompute_local` 使用的*全局障碍集*。
        ``speed_mask``
            用 ``[0, 1]`` 范围内的系数缩放后的 ``boundary_mask``，该系数随到最近
            被阻挡单元的距离从 ``robot_radius`` 增长到
            ``robot_radius + brake_distance``，并从 ``0`` 线性增长到 ``1``。

        正是这个斜坡让控制器能把距离场当作限速而非停车标志：距墙 ``d`` 处的单元
        被赋予一个速度，使得以控制器减速度刹车时仍能在 ``d - robot_radius`` 内
        停下。到墙的距离本身是通过从*所有被阻挡单元同时向外*运行
        :meth:`dijkstra_multi_seed_numpy` 得到的，这比形态学腐蚀既更廉价又更准确，
        因为它沿真实的 8 连通几何传播，而不是用方形结构元。

        这里使用最近邻重采样而非插值，因为区域掩码是类别型的：对「空闲」和「墙」
        取平均会得到一个既非此也非彼的半空闲单元，而双线性上采样会把墙角磨圆。

        参数:
            region_mask: 地图像素单位的 ``(H, W)`` 布尔可通行掩码。
            offset_x: ``region_mask`` 中距离场左下角所在的列。
            offset_y: ``region_mask`` 中距离场左下角所在的行。
            scale_factor: 每个地图像素对应的距离场单元数。
            field_width: 输出数组的宽度。
            field_height: 输出数组的高度。
            brake_distance: 线性斜坡的宽度，单位米。``0`` 会让斜坡变成硬台阶。
            robot_radius: 距墙比该值更近的单元变为不可通行，单位米。

        返回:
            ``(speed_mask, boundary_mask)``，两者均为
            ``(field_height, field_width)`` 的 ``float32``。

        异常:
            ValueError: 若 ``region_mask`` 不是二维。

        示例:
            >>> import numpy as np
            >>> field = DistanceField(resolution=0.1)
            >>> region = np.zeros((5, 5), bool); region[1:4, 1:4] = True
            >>> speed, boundary = field._build_speed_mask(
            ...     region, offset_x=0, offset_y=0, scale_factor=1.0,
            ...     field_width=5, field_height=5, brake_distance=0.15,
            ...     robot_radius=0.0)
            >>> float(boundary[2, 2]), float(boundary[0, 0])
            (1.0, 0.0)
            >>> 0.0 < float(speed[1, 1]) < 1.0 and float(speed[2, 2]) == 1.0
            True
        """
        region = _as_2d_array(region_mask, "region_mask")
        region_bool = region != 0

        # 最近邻重采样：距离场单元 -> 地图像素 -> 布尔值。
        src_x = np.rint(np.arange(field_width, dtype=np.float64) / scale_factor + offset_x)
        src_y = np.rint(np.arange(field_height, dtype=np.float64) / scale_factor + offset_y)
        src_xi = src_x.astype(np.int64)
        src_yi = src_y.astype(np.int64)
        ok_x = (src_xi >= 0) & (src_xi < region_bool.shape[1])
        ok_y = (src_yi >= 0) & (src_yi < region_bool.shape[0])
        sampled = region_bool[np.ix_(np.where(ok_y, src_yi, 0), np.where(ok_x, src_xi, 0))]
        inside = sampled & ok_y[:, None] & ok_x[None, :]

        boundary_mask = inside.astype(np.float32)
        speed_mask = boundary_mask.copy()

        # 每个单元到最近被阻挡单元的距离，仅通过空闲空间测量
        # （被阻挡单元无法承载波前）。
        seeds = ~inside
        if np.any(seeds):
            to_wall = self.dijkstra_multi_seed_numpy(seeds, boundary_mask)
        else:
            to_wall = np.full(boundary_mask.shape, np.inf, dtype=np.float32)

        if robot_radius > 0.0:
            unsafe = to_wall < robot_radius
            speed_mask[unsafe] = 0.0
            logger.debug("robot_radius=%.3f blocked %d cell(s)", robot_radius, int(np.count_nonzero(unsafe)))

        if brake_distance > 0.0:
            factor = (to_wall - robot_radius) / brake_distance
            factor[~np.isfinite(factor)] = 1.0
            np.clip(factor, 0.0, 1.0, out=factor)
        else:
            # 退化斜坡：硬膨胀一结束就获得全速。
            factor = (to_wall > robot_radius).astype(np.float32)
        speed_mask *= factor.astype(np.float32)

        return speed_mask, boundary_mask

    # ------------------------------------------------------------------
    # 传播内核
    # ------------------------------------------------------------------
    def dijkstra_multi_seed(
        self,
        seeds: Any,
        cost_mask: ArrayLike,
        *,
        block_value: float = 0.0,
    ) -> np.ndarray:
        """在速度加权网格上运行优先队列版多源 Dijkstra。

        边的代价为 ``step_length_metres / cost_mask[destination]``，因此该掩码被
        解读为*速度*：某单元的速度翻倍，进入它的代价就减半。``cost_mask <=
        block_value``（或取值非有限）的单元不可通行，永远不会被进入——注意被测试
        的是*目标*单元的值，因此阻挡单个单元会切断所有经过它的路径。

        经典堆实现会把每个单元恰好确定一次，因此其代价为被到达单元数 ``k`` 的
        ``O(k log k)``。当只有少量单元被到达时——例如房间中的单个目标、一条狭窄
        走廊——以及当可达集合远小于整个数组时，这是正确的选择，因为它从不触碰
        无法到达的单元。对于宽阔的多源前沿（大地图中每个被阻挡单元都作为一个
        到墙距离变换的种子），逐单元的 Python 开销会占主导；此时请改用
        :meth:`dijkstra_multi_seed_numpy`。

        参数:
            seeds: ``(x, y)`` 或 ``(x, y, cost)`` 对的迭代器，或与 ``cost_mask``
                形状相同、标记种子的二维布尔数组（见 :func:`_normalise_seeds`）。
                ``x`` 是列，``y`` 是行。
            cost_mask: 每单元速度的二维数组；布尔掩码会被解读为 ``0.0``/``1.0``。
            block_value: ``cost_mask <= block_value`` 的单元不可通行。

        返回:
            以米为单位、累积代价的 ``(H, W)`` ``float32`` 数组，从未被到达的单元
            为 ``+inf``。把结果存入距离场的调用方会用 :attr:`UNREACHABLE` 替换
            ``inf``。

        异常:
            TypeError: 若 ``cost_mask`` 不是 array-like，或 ``seeds`` 类型错误。
            ValueError: 若 ``cost_mask`` 不是二维、``block_value`` 非有限，或某个
                种子格式错误或落在掩码之外。

        示例:
            >>> import numpy as np
            >>> field = DistanceField(resolution=0.5)
            >>> mask = np.ones((3, 3), np.float32)
            >>> mask[1, 1] = 0.0                       # 需要绕行的一个洞
            >>> dist = field.dijkstra_multi_seed([(0, 0)], mask)
            >>> [round(float(v), 3) for v in (dist[0, 0], dist[0, 1], dist[2, 0])]
            [0.0, 0.5, 1.0]
            >>> bool(np.isinf(dist[1, 1]))             # 被阻挡的单元保持无穷大
            True
            >>> round(float(dist[2, 2]), 4)            # 绕行的最短路径
            1.7071
        """
        cost = _as_cost_mask(cost_mask)
        _finite_float(block_value, "block_value")
        rows, cols, costs = _normalise_seeds(seeds, cost.shape)
        height, width = cost.shape

        distances = np.full(cost.shape, np.inf, dtype=np.float32)
        visited = np.zeros(cost.shape, dtype=np.bool_)
        passable = np.isfinite(cost) & (cost > block_value)

        heap: list[tuple[float, int, int]] = []
        for sy, sx, initial in zip(rows.tolist(), cols.tolist(), costs.tolist()):
            if initial < distances[sy, sx]:
                distances[sy, sx] = np.float32(initial)
            heapq.heappush(heap, (float(distances[sy, sx]), sy, sx))

        step_metres = float(self.resolution)
        processed = 0
        while heap:
            current, y, x = heapq.heappop(heap)
            if visited[y, x]:
                continue
            visited[y, x] = True
            processed += 1
            for dy, dx, step in _NEIGHBOURS_8:
                ny, nx = y + dy, x + dx
                if ny < 0 or ny >= height or nx < 0 or nx >= width:
                    continue
                if visited[ny, nx] or not passable[ny, nx]:
                    continue
                candidate = current + (step * step_metres) / float(cost[ny, nx])
                if candidate < distances[ny, nx]:
                    distances[ny, nx] = candidate
                    heapq.heappush(heap, (candidate, ny, nx))

        logger.debug("heapq dijkstra finalised %d cell(s)", processed)
        return distances

    def dijkstra_multi_seed_numpy(
        self,
        seeds: Any,
        cost_mask: ArrayLike,
        *,
        block_value: float = 0.0,
    ) -> np.ndarray:
        """与堆版本采用相同代价模型的向量化波前传播。

        它不按代价为单元排序，而是执行反复的松弛扫描：上一轮扫描中被改进的每个
        单元都会松弛其八个邻居，扫描重复进行直到没有单元再被改进。这相当于以前
        进先出（FIFO）前沿实现的 Bellman-Ford，因此一旦收敛，结果与堆版本产生的
        *相同* 最优解——没有 visited 数组，也没有堆，只有整数组的 numpy 运算。

        当种子集合很大、前沿很宽时使用它，这正是 :meth:`_build_speed_mask` 背后的
        到最近墙距离变换以及 :meth:`recompute_local` 中障碍膨胀的情形：那里成千上
        万个单元同时作为种子，波前覆盖了窗口的大部分，堆的逐单元 Python 开销会
        高出一个数量级。对于大地图中的单个目标，:meth:`dijkstra_multi_seed` 更优，
        因为它从不访问无法到达的单元。

        这里用 ``height * width`` 作为迭代上限。Bellman-Ford 在最坏情形下至多需要
        ``|V| - 1`` 次扫描，因此该上界既能保证终止，又绝不会截断真实的传播；触发
        上界会记录一条警告，因为这意味着代价掩码中存在由数值噪声构成的递减环。

        参数:
            seeds: ``(x, y)`` 或 ``(x, y, cost)`` 对的迭代器，或与 ``cost_mask``
                形状相同的二维布尔数组。布尔形式尤其为该方法而存在：把「每个被
                阻挡单元」作为元组列表来播种会构造数十万个 Python 对象。
            cost_mask: 每单元速度的二维数组。
            block_value: ``cost_mask <= block_value`` 的单元不可通行。

        返回:
            以米为单位、累积代价的 ``(H, W)`` ``float32`` 数组，不可达处为
            ``+inf``。

        异常:
            TypeError: 若 ``cost_mask`` 不是 array-like，或 ``seeds`` 类型错误。
            ValueError: 若 ``cost_mask`` 不是二维、``block_value`` 非有限，或某个
                种子格式错误或落在掩码之外。

        示例:
            >>> import numpy as np
            >>> field = DistanceField(resolution=0.5)
            >>> mask = np.ones((1, 5), np.float32)
            >>> seeds = np.zeros((1, 5), bool); seeds[0, 0] = True
            >>> field.dijkstra_multi_seed_numpy(seeds, mask)[0].tolist()
            [0.0, 0.5, 1.0, 1.5, 2.0]
        """
        cost = _as_cost_mask(cost_mask)
        _finite_float(block_value, "block_value")
        rows, cols, costs = _normalise_seeds(seeds, cost.shape)
        height, width = cost.shape

        distances = np.full(cost.shape, np.inf, dtype=np.float32)
        if rows.size:
            np.minimum.at(distances, (rows, cols), costs.astype(np.float32))
        passable = np.isfinite(cost) & (cost > block_value)

        frontier = np.zeros(cost.shape, dtype=np.bool_)
        if rows.size:
            frontier[rows, cols] = True

        step_metres = float(self.resolution)
        max_iterations = max(1, int(height) * int(width))
        iterations = 0
        while iterations < max_iterations and frontier.any():
            cy, cx = np.nonzero(frontier)
            current = distances[cy, cx]
            nxt = np.zeros(cost.shape, dtype=np.bool_)
            for dy, dx, step in _NEIGHBOURS_8:
                ny, nx = cy + dy, cx + dx
                in_bounds = (ny >= 0) & (ny < height) & (nx >= 0) & (nx < width)
                if not in_bounds.any():
                    continue
                ny, nx, base = ny[in_bounds], nx[in_bounds], current[in_bounds]
                open_cells = passable[ny, nx]
                if not open_cells.any():
                    continue
                ny, nx, base = ny[open_cells], nx[open_cells], base[open_cells]
                candidate = base + (step * step_metres) / cost[ny, nx]
                improved = candidate < distances[ny, nx]
                if improved.any():
                    iy, ix = ny[improved], nx[improved]
                    distances[iy, ix] = candidate[improved]
                    nxt[iy, ix] = True
            frontier = nxt
            iterations += 1

        # 循环结束要么是因为前沿已空（已收敛），要么是因为达到迭代上限
        # 而仍有待处理的工作。
        if frontier.any():
            logger.warning(
                "wavefront propagation stopped after the %d-iteration ceiling without "
                "converging; the cost mask may contain numerically unstable values",
                max_iterations,
            )
        else:
            logger.debug("numpy wavefront converged after %d sweep(s)", iterations)
        return distances

    # ------------------------------------------------------------------
    # 坐标变换
    # ------------------------------------------------------------------
    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """把世界位置转换为距离场单元索引，**不做夹取**。

        与 :meth:`reusable_model.gridmap.occupancy.OccupancyGrid.world_to_pixel` 不同，结果
        可能为负或超出距离场范围。这是有意为之：夹取后的索引会把「机器人离开了
        地图」变成「机器人在边界上」，调用方就会从一个它并不在其中的单元开始规划。
        请用 :attr:`shape` 检查结果，或使用会替你完成检查的
        :meth:`distance_at_world`。

        参数:
            x: 世界坐标 x，单位米。
            y: 世界坐标 y，单位米。

        返回:
            ``(px, py)`` 距离场单元索引（列、行）。

        异常:
            ValueError: 若距离场尚未生成，或坐标不是有限数。

        示例:
            >>> import numpy as np
            >>> field = DistanceField(resolution=0.5)
            >>> field.generate(np.ones((4, 4), bool), (0.25, 0.25),
            ...                geometry=GridGeometry(0.0, 0.0, 0.5, 4, 4))
            >>> field.world_to_grid(1.25, 0.75)
            (2, 1)
            >>> field.world_to_grid(-9.0, 0.0)   # not clamped
            (-18, 0)
        """
        geometry = self._require()
        return _pixel_from_world(geometry, x, y)

    def grid_to_world(self, px: int, py: int) -> tuple[float, float]:
        """把距离场单元索引转换为该单元中心的世界位置。

        参数:
            px: 列索引；插值时可以为小数。
            py: 行索引；插值时可以为小数。

        返回:
            ``(x, y)`` 世界坐标，单位米。

        异常:
            ValueError: 若距离场尚未生成，或索引不是有限数。

        示例:
            >>> import numpy as np
            >>> field = DistanceField(resolution=0.5)
            >>> field.generate(np.ones((4, 4), bool), (0.25, 0.25),
            ...                geometry=GridGeometry(0.0, 0.0, 0.5, 4, 4))
            >>> field.grid_to_world(2, 1)
            (1.25, 0.75)
        """
        geometry = self._require()
        return _world_from_pixel(geometry, px, py)

    def world_to_grid_batch(
        self, xs: ArrayLike, ys: ArrayLike
    ) -> tuple[np.ndarray, np.ndarray]:
        """向量化版本的 :meth:`world_to_grid`，同样不做夹取。

        参数:
            xs: 世界 x 值的 array-like。
            ys: 世界 y 值的 array-like，长度与 ``xs`` 相同。

        返回:
            ``(px, py)`` 距离场单元索引的 ``int64`` 数组。

        异常:
            TypeError: 若某个输入不是数字的 array-like。
            ValueError: 若距离场尚未生成、某个输入不是一维、两个输入长度不同，
                或某个取值非有限。

        示例:
            >>> import numpy as np
            >>> field = DistanceField(resolution=0.5)
            >>> field.generate(np.ones((4, 4), bool), (0.25, 0.25),
            ...                geometry=GridGeometry(0.0, 0.0, 0.5, 4, 4))
            >>> px, py = field.world_to_grid_batch([0.1, 1.9], [0.1, 0.6])
            >>> px.tolist(), py.tolist()
            ([0, 3], [0, 1])
        """
        geometry = self._require()
        x = _as_1d(xs, "xs")
        y = _as_1d(ys, "ys")
        if x.shape != y.shape:
            raise ValueError(f"xs and ys must have the same length, got {x.shape} and {y.shape}")
        return (
            np.floor((x - geometry.origin_x) / geometry.resolution).astype(np.int64),
            np.floor((y - geometry.origin_y) / geometry.resolution).astype(np.int64),
        )

    def distance_at_world(self, x: float, y: float) -> float:
        """读取某个世界位置到目标的距离。

        参数:
            x: 世界坐标 x，单位米。
            y: 世界坐标 y，单位米。

        返回:
            距离，单位米；当该位置落在距离场之外或无法到达目标时返回
            :attr:`UNREACHABLE`。返回哨兵值而非抛异常，能让控制循环中的逐位姿查询
            无需分支；该哨兵为负数，因此绝不会被误认为是真实距离。

        异常:
            ValueError: 若距离场尚未生成，或坐标不是有限数。

        示例:
            >>> import numpy as np
            >>> field = DistanceField(resolution=0.5)
            >>> field.generate(np.ones((4, 4), bool), (0.25, 0.25),
            ...                geometry=GridGeometry(0.0, 0.0, 0.5, 4, 4))
            >>> field.distance_at_world(0.25, 0.25)
            0.0
            >>> field.distance_at_world(99.0, 99.0)
            -1.0
        """
        self._require()
        px, py = self.world_to_grid(x, y)
        height, width = self._distances.shape  # type: ignore[union-attr]
        if not (0 <= px < width and 0 <= py < height):
            return self.UNREACHABLE
        return float(self._distances[py, px])  # type: ignore[index]

    def is_inside(self, x: float, y: float, threshold: float = 0.8) -> bool:
        """报告某个世界位置是否足够安全地位于区域内部。

        测试针对的是*带制动斜坡*的速度掩码，而非原始区域：某个单元在几何上可能
        是空闲的，但仍可能因离墙太近而不算「内部」。因此默认阈值 ``0.8`` 排除了
        制动斜坡的外侧部分。

        参数:
            x: 世界坐标 x，单位米。
            y: 世界坐标 y，单位米。
            threshold: 必须被超过的速度掩码取值，范围 ``[0, 1]``。

        返回:
            当该位置位于距离场内且其速度掩码取值超过 ``threshold`` 时为
            ``True``。

        异常:
            ValueError: 若距离场尚未生成，或 ``threshold`` 非有限或超出
                ``[0, 1]``。

        示例:
            >>> import numpy as np
            >>> field = DistanceField(resolution=0.5)
            >>> field.generate(np.ones((4, 4), bool), (0.25, 0.25),
            ...                geometry=GridGeometry(0.0, 0.0, 0.5, 4, 4),
            ...                brake_distance=0.0)
            >>> field.is_inside(1.75, 1.75), field.is_inside(99.0, 0.0)
            (True, False)
        """
        geometry = self._require()
        level = _finite_float(threshold, "threshold")
        if not 0.0 <= level <= 1.0:
            raise ValueError(f"threshold must be within [0, 1], got {threshold!r}")
        px, py = self.world_to_grid(x, y)
        if not (0 <= px < geometry.width and 0 <= py < geometry.height):
            return False
        return bool(self._speed_mask[py, px] > level)  # type: ignore[index]

    # ------------------------------------------------------------------
    # 动态障碍
    # ------------------------------------------------------------------
    def update_obstacle_timestamps(
        self,
        points_world_xy: ArrayLike | None,
        now: float | Sequence[float] | None = None,
    ) -> int:
        """为新观测到的障碍点所覆盖的单元打上时间戳。

        动态障碍以逐单元*观测时间*网格的形式保存，而非布尔掩码。原因在于传感器
        可能只在几帧内看到某个行人，随后就让他消失在柱子后面；布尔掩码要么会立刻
        抹去该障碍（从而直接朝一个人规划路径），要么会永久保留它（从而把机器人
        冻住）。时间戳让 :meth:`valid_obstacle_mask` 能回答「该单元在最近
        ``timeout`` 秒内是否被观测为被阻挡」，从而优雅地衰减，无需显式移除。

        从未被打过时间戳的单元保持 ``0.0``，无论调用方传入的 ``now`` 是多少，都
        始终被视为「不是障碍」。

        参数:
            points_world_xy: 世界 ``(x, y)`` 位置（单位米）的 ``(N, 2)``
                array-like。落在距离场之外的点会被忽略。``None`` 或空输入是
                空操作，这让「本周期无检测」的表达很廉价。
            now: 观测时间，单位秒。``None`` 使用 :func:`time.time`；标量应用于
                每个点；长度为 ``N`` 的序列为每个点打上各自的时间（回放数据包时
                很有用）。

        返回:
            落在距离场内的点数。

        异常:
            TypeError: 若 ``points_world_xy`` 不是数字的 array-like。
            ValueError: 若距离场尚未生成、``points_world_xy`` 不是 ``(N, 2)``
                或含有非有限值，或 ``now`` 既不是 ``None``、有限标量，也不是
                长度为 ``N`` 的有限序列。

        示例:
            >>> import numpy as np
            >>> field = DistanceField(resolution=0.5)
            >>> field.generate(np.ones((4, 4), bool), (0.25, 0.25),
            ...                geometry=GridGeometry(0.0, 0.0, 0.5, 4, 4))
            >>> field.update_obstacle_timestamps([[1.25, 0.75]], now=100.0)
            1
            >>> int(field.valid_obstacle_mask(now=100.5, timeout=1.0).sum())
            1
            >>> int(field.valid_obstacle_mask(now=200.0, timeout=1.0).sum())
            0
        """
        geometry = self._require()
        if points_world_xy is None:
            return 0
        try:
            points = np.asarray(points_world_xy, dtype=float)
        except (TypeError, ValueError) as exc:
            raise TypeError(
                f"points_world_xy must be an (N, 2) array-like of world coordinates, "
                f"got {points_world_xy!r}"
            ) from exc
        if points.size == 0:
            return 0
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError(
                f"points_world_xy must be an (N, 2) array of world (x, y), got shape "
                f"{points.shape}"
            )
        if not np.all(np.isfinite(points)):
            raise ValueError(f"points_world_xy must be finite, got {points_world_xy!r}")

        count = points.shape[0]
        if now is None:
            stamps = np.full(count, time.time(), dtype=np.float64)
        else:
            stamps = np.asarray(now, dtype=np.float64).reshape(-1)
            if stamps.size == 1:
                stamps = np.full(count, float(stamps[0]), dtype=np.float64)
            if stamps.shape != (count,) or not np.all(np.isfinite(stamps)):
                raise ValueError(
                    f"now must be None, a finite scalar, or {count} finite timestamps; "
                    f"got {now!r}"
                )

        px, py = self.world_to_grid_batch(points[:, 0], points[:, 1])
        inside = (px >= 0) & (px < geometry.width) & (py >= 0) & (py < geometry.height)
        if not inside.any():
            logger.debug(
                "all %d obstacle point(s) fell outside the field; nothing stamped", count
            )
            return 0
        self._obstacle_timestamps[py[inside], px[inside]] = stamps[inside]  # type: ignore[index]
        return int(np.count_nonzero(inside))

    def valid_obstacle_mask(
        self, now: float | None = None, timeout: float = 1.0
    ) -> np.ndarray:
        """返回尚未超时的障碍的布尔网格。

        当某单元最近一次观测晚于 ``now - timeout`` 时，它就算作被阻挡。被标记为
        ``0.0``（从未被观测）的单元会被显式排除，因此结果绝不取决于 ``now`` 恰好
        距离纪元有多远。

        参数:
            now: 参考时间，单位秒；``None`` 使用 :func:`time.time`。
            timeout: 一次观测保持有效的时长，单位秒。``0`` 只保留恰好在 ``now``
                观测到的单元。

        返回:
            ``(H, W)`` 的 ``bool`` 数组；``True`` 表示「当前是障碍」。

        异常:
            ValueError: 若距离场尚未生成、``now`` 非有限，或 ``timeout`` 为负
                或非有限。

        示例:
            >>> import numpy as np
            >>> field = DistanceField(resolution=0.5)
            >>> field.generate(np.ones((4, 4), bool), (0.25, 0.25),
            ...                geometry=GridGeometry(0.0, 0.0, 0.5, 4, 4))
            >>> field.update_obstacle_timestamps([[0.75, 0.75]], now=50.0)
            1
            >>> field.valid_obstacle_mask(now=50.9, timeout=1.0)[1, 1]
            True
            >>> field.valid_obstacle_mask(now=51.1, timeout=1.0)[1, 1]
            False
        """
        self._require()
        window = _non_negative_float(timeout, "timeout")
        reference = time.time() if now is None else _finite_float(now, "now")
        stamps = self._obstacle_timestamps
        assert stamps is not None  # 由 _require() 保证
        return (stamps > reference - window) & (stamps > 0.0)

    # ------------------------------------------------------------------
    # 局部重规划
    # ------------------------------------------------------------------
    def recompute_local(
        self,
        robot_world: Sequence[float],
        *,
        search_radius: float,
        obstacles_xy: ArrayLike | None = None,
        obstacle_timestamps: ArrayLike | None = None,
        obstacle_timeout: float = 1.0,
        robot_radius: float = 0.4,
        acceleration: float = 0.5,
        stop_when_no_path: bool = False,
        connectivity_from: Sequence[float] | None = None,
    ) -> np.ndarray | None:
        """在机器人周围的窗口内，结合实时障碍重新求解距离场。

        全局场是*边界条件*，而不是初始猜测：窗口内的单元从全局上最接近目标的单元
        重新传播，因此只要没有障碍干扰，局部结果就与全局结果一致，只在必须绕行时
        才绕行。只触及 ``O((2R/resolution)^2)`` 个单元，这正是它能在控制频率下负担
        得起的原因。

        局部代价掩码由四层组合而成：

        1. 未膨胀的全局障碍集（:attr:`boundary_mask`），这样窗口内的墙壁永远不会
           被遗忘；
        2. 带时间戳的动态障碍，由 ``obstacles_xy`` 刷新；
        3. 按 ``robot_radius`` 做的硬膨胀——距任何障碍比足迹更近的单元速度置 ``0``；
        4. *速度锥* ``v = sqrt(2 * acceleration * max(d - robot_radius, 0))``，并截断
           到 ``[0, 1]``，其中 ``d`` 是到最近障碍的距离。这是运动学恒等式
           ``v^2 = 2 a s`` 的反解：以速度 ``v`` 行驶时机器人需要 ``v^2 / (2 a)`` 米
           才能停下，因此一个单元只能以它仍能在触及膨胀足迹之前停下的速度进入。

        **种子的选择是微妙之处。** 窗口从*全局*距离最小的那些单元重新传播，这通常
        是窗口远侧的一条窄带。当这条窄带位于机器人半径的一半以内时，两种截然不同的
        情形会产生相同的画面：机器人基本已到达，或者窗口内根本没有路径，而那个
        「最小值」只是离机器人最近的单元。在后一种情形下播种会告诉机器人：贴着它
        刚刚检测到的障碍走就是在前进。区分依据是全局值本身：若窗口内的最小值超过
        ``2 * resolution``，机器人显然不在目标处，因此丢弃机器人附近的最小值。若
        窗口最终没有任何种子，则返回一个全为 :attr:`UNREACHABLE` 的数组，调用方
        必须将其解读为「无局部路径」。

        参数:
            robot_world: 机器人的 ``(x, y)`` 世界位置，单位米。
            search_radius: 方形窗口的半宽，单位米。窗口会被裁剪到距离场内，因此
                靠近边界的机器人会得到较小的窗口。
            obstacles_xy: *当前*观测到的障碍的世界位置 ``(N, 2)``；它们会在构建
                窗口前被打入持久的时间戳网格，因此能在此次调用后继续存在，并在
                ``obstacle_timeout`` 之后衰减。``None`` 或空表示「没有新障碍」。
            obstacle_timestamps: ``obstacles_xy`` 可选的逐点观测时间——标量或长度为
                ``N`` 的序列。``None`` 会用当前时间为它们全部打戳。
            obstacle_timeout: 一次观测保持有效的秒数。
            robot_radius: 用于硬膨胀的足迹半径，单位米。必须 ``> 0``：不膨胀的话，
                距离场会穿过机器人无法实际占据的单元。
            acceleration: 速度锥使用的制动减速度，单位 m/s^2。
            stop_when_no_path: 为 ``False``（默认）时，窗口会额外限制到从
                ``connectivity_from`` 可达的单元，从而阻止机器人规划出穿过它并不
                连通的口袋区域的绕行。为 ``True`` 时跳过该限制，让调用方能看到未经
                限制的距离场并决定停下。
            connectivity_from: 用于衡量可达性的 ``(x, y)`` 世界位置；默认为
                ``robot_world``。

        返回:
            局部距离的 ``(h, w)`` ``float32`` 数组，单位米，无局部路径处为
            :attr:`UNREACHABLE`；当窗口完全落在距离场之外时为 ``None``。该数组以
            ``[y - y_min, x - x_min]`` 索引；需要这些偏移量的调用方应完全按照本方法
            的做法，从 ``world_to_grid(robot_world)`` 和 ``search_radius`` 推导它们。

        异常:
            TypeError: 若 ``robot_world`` 或 ``connectivity_from`` 不是数字对。
            ValueError: 若距离场不是由 :meth:`generate` 生成的（用
                :meth:`from_array` 还原的距离场没有障碍集）、
                ``search_radius``/``acceleration`` 不 ``> 0``、
                ``robot_radius``/``obstacle_timeout`` 为负，或
                ``connectivity_from`` 落在窗口之外。

        示例:
            >>> import numpy as np
            >>> geometry = GridGeometry(0.0, 0.0, 0.1, 41, 41)
            >>> field = DistanceField(resolution=0.1)
            >>> field.generate(np.ones((41, 41), bool), (2.05, 2.05),
            ...                geometry=geometry, brake_distance=0.0)
            >>> clear = field.recompute_local((0.55, 0.55), search_radius=1.0,
            ...                               robot_radius=0.15, acceleration=0.5)
            >>> blocked = field.recompute_local((0.55, 0.55), search_radius=1.0,
            ...                                 obstacles_xy=[[0.75, 0.75]],
            ...                                 robot_radius=0.15, acceleration=0.5)
            >>> float(blocked[5, 5]) > float(clear[5, 5]) > 0.0
            True
        """
        if self._distances is None or self._geometry is None:
            raise ValueError(
                "this DistanceField is empty; call generate() or from_array() first"
            )
        if self._boundary_mask is None:
            raise ValueError(
                "recompute_local needs the global obstacle set built by generate(); "
                "a field restored with from_array() does not carry one"
            )
        robot_x, robot_y = _as_xy(robot_world, "robot_world")
        radius = _positive_float(search_radius, "search_radius")
        footprint = _positive_float(robot_radius, "robot_radius")
        deceleration = _positive_float(acceleration, "acceleration")
        timeout = _non_negative_float(obstacle_timeout, "obstacle_timeout")
        assert self._obstacle_timestamps is not None  # 由上面的检查保证

        if obstacle_timestamps is not None and obstacles_xy is None:
            raise ValueError(
                "obstacle_timestamps was given without obstacles_xy; per-point "
                "observation times only apply to the points passed in obstacles_xy"
            )
        if obstacles_xy is not None:
            self.update_obstacle_timestamps(obstacles_xy, now=obstacle_timestamps)

        geometry = self._geometry
        centre_x, centre_y = self.world_to_grid(robot_x, robot_y)
        grid_radius = max(1, int(radius / self.resolution))
        x_min = max(0, centre_x - grid_radius)
        x_max = min(geometry.width - 1, centre_x + grid_radius)
        y_min = max(0, centre_y - grid_radius)
        y_max = min(geometry.height - 1, centre_y + grid_radius)
        if x_max < x_min or y_max < y_min:
            logger.debug(
                "the %.3f m window around (%.3f, %.3f) falls outside the field",
                radius,
                robot_x,
                robot_y,
            )
            return None

        rows = slice(y_min, y_max + 1)
        cols = slice(x_min, x_max + 1)
        window_shape = (y_max - y_min + 1, x_max - x_min + 1)

        # 第 1 + 2 层：全局墙壁与带时限的动态障碍。
        local = np.ones(window_shape, dtype=np.float32)
        local[self._boundary_mask[rows, cols] < _SPEED_EPS] = 0.0
        local[self.valid_obstacle_mask(timeout=timeout)[rows, cols]] = 0.0

        # 第 3 + 4 层：足迹膨胀、连通性、速度锥。
        if np.any(local <= 0.0):
            to_obstacle = self.dijkstra_multi_seed_numpy(local <= 0.0, local)
            local[to_obstacle < footprint] = 0.0

            if not stop_when_no_path:
                source = robot_world if connectivity_from is None else connectivity_from
                src_x, src_y = _as_xy(source, "connectivity_from")
                src_px, src_py = self.world_to_grid(src_x, src_y)
                local_px, local_py = src_px - x_min, src_py - y_min
                if not (0 <= local_px < window_shape[1] and 0 <= local_py < window_shape[0]):
                    raise ValueError(
                        f"connectivity_from=({src_x}, {src_y}) maps to field cell "
                        f"({src_px}, {src_py}), outside the window x=[{x_min}, {x_max}] "
                        f"y=[{y_min}, {y_max}]"
                    )
                robot_seed = np.zeros(window_shape, dtype=np.bool_)
                robot_seed[local_py, local_px] = True
                reachable = self.dijkstra_multi_seed_numpy(robot_seed, local)
                # 丢弃不连通的口袋区域，正是防止机器人「绕路」进入再也
                # 出不来的死胡同的原因。
                local[~np.isfinite(reachable)] = 0.0

            clearance = np.maximum(to_obstacle - footprint, 0.0)
            cone = np.sqrt(2.0 * deceleration * clearance)
            cone[~np.isfinite(to_obstacle)] = 1.0
            np.clip(cone, 0.0, 1.0, out=cone)
            local = np.minimum(local, cone.astype(np.float32))

        # 种子：全局最近的单元，剔除机器人附近那些有歧义的单元。
        global_slice = self._distances[rows, cols]
        robot_px, robot_py = centre_x - x_min, centre_y - y_min
        half_radius = max(1, int(radius / 2.0 / self.resolution))
        usable = (local > _SPEED_EPS) & (global_slice >= 0.0)
        seeds: list[tuple[int, int, float]] = []
        if np.any(usable):
            minimum = float(np.min(global_slice[usable]))
            tie_rows, tie_cols = np.nonzero(
                usable & (np.abs(global_slice - minimum) < _SEED_VALUE_TOLERANCE)
            )
            for ty, tx in zip(tie_rows.tolist(), tie_cols.tolist()):
                near_robot = abs(tx - robot_px) < half_radius and abs(ty - robot_py) < half_radius
                if near_robot and minimum > 2.0 * self.resolution:
                    # 目标仍很远时机器人旁边出现最小值，意味着「该窗口内无路」，
                    # 而不是「快到了」。
                    continue
                seeds.append((tx, ty, float(global_slice[ty, tx])))

        if seeds:
            internal = self.dijkstra_multi_seed(seeds, local)
        else:
            logger.debug("no usable seed in the local window; reporting no local path")
            internal = np.full(window_shape, np.inf, dtype=np.float32)
        internal[~np.isfinite(internal)] = self.UNREACHABLE
        return internal

    # ------------------------------------------------------------------
    # 派生场
    # ------------------------------------------------------------------
    def generate_inverted(self, *, boundary_threshold: float = 0.9) -> np.ndarray:
        """构建镜像场：区域边缘为 ``0``，向内递增。

        :attr:`distances` 衡量「离目标有多远」，而镜像场衡量「我在区域内部有多深」。
        它的种子是制动斜坡上的单元——空闲、但速度低于 ``boundary_threshold``，也
        就是贴着墙壁的那一圈——并且传播被限制在完全空闲的内部，因此某单元的值就是
        到最近墙壁的步行距离。

        其用途是把机器人*驶入*某个区域，而不是驶向一个点：沿该场下降会走向墙壁，
        因此要向内移动，可取 :func:`reusable_model.gridmap.descent.descent_direction` 返回的
        方向并加上 ``math.pi``（等价地，对 ``peak - inverted`` 下降）。与普通距离场
        结合，就构成了「先到那个房间，再远离它的墙壁」这两半。

        种子用 :func:`numpy.nonzero` 收集，而不是原实现的逐单元嵌套循环——后者每次
        调用都要花费 ``O(height * width)`` 次 Python 迭代。

        参数:
            boundary_threshold: 空闲单元低于该速度掩码取值时算作边缘，范围
                ``(0, 1]``。调低它只会把几乎被阻挡的单元作为种子，调高它会深入
                斜坡更深处播种。

        返回:
            到边缘距离（单位米）的 ``(H, W)`` ``float32`` 数组，内部传播无法到达
            的单元为 :attr:`UNREACHABLE`。结果*不会*保存在实例上。

        异常:
            ValueError: 若距离场尚未生成，或 ``boundary_threshold`` 非有限或超出
                ``(0, 1]``。

        示例:
            >>> import numpy as np
            >>> geometry = GridGeometry(0.0, 0.0, 0.1, 21, 21)
            >>> mask = np.zeros((21, 21), bool)
            >>> mask[1:20, 1:20] = True        # 四周都是墙的房间
            >>> field = DistanceField(resolution=0.1)
            >>> field.generate(mask, (1.05, 1.05), geometry=geometry,
            ...                brake_distance=0.3)
            >>> inverted = field.generate_inverted()
            >>> round(float(inverted[16, 16]), 3)      # 距边缘一个单元
            0.1
            >>> float(inverted[9, 9]) > float(inverted[16, 16]) > 0.0
            True
        """
        self._require()
        threshold = _finite_float(boundary_threshold, "boundary_threshold")
        if not 0.0 < threshold <= 1.0:
            raise ValueError(
                f"boundary_threshold must be within (0, 1], got {boundary_threshold!r}"
            )
        assert self._speed_mask is not None

        speed = self._speed_mask
        rim = (speed > 0.0) & (speed < threshold)
        inverted = np.full(speed.shape, np.inf, dtype=np.float32)
        if not np.any(rim):
            logger.warning(
                "no cell has a speed in (0, %.3f); the inverted field is empty -- "
                "lower boundary_threshold or check brake_distance",
                threshold,
            )
            inverted[:] = self.UNREACHABLE
            return inverted

        interior = (speed >= threshold).astype(np.float32)
        propagated = self.dijkstra_multi_seed(rim, interior)
        inverted[:] = propagated
        inverted[~np.isfinite(inverted)] = self.UNREACHABLE
        return inverted

    # ------------------------------------------------------------------
    # 序列化 / 调试
    # ------------------------------------------------------------------
    def to_array(self) -> np.ndarray:
        """返回距离数组的一份私有副本。

        该副本是 ``float32``，不可达单元保存为 :attr:`UNREACHABLE`，这正是选择该
        哨兵值而非 ``inf`` 的原因：结果可以用 :func:`numpy.save` 写出，再逐位读回。
        坐标系*不*包含在该数组中——请把
        ``geometry.origin_x/origin_y/resolution/width/height`` 与它一起持久化，
        并用 :meth:`from_array` 重建。

        返回:
            ``(H, W)`` 的 ``float32`` 数组。

        异常:
            ValueError: 若距离场尚未生成。

        示例:
            >>> import numpy as np
            >>> field = DistanceField(resolution=0.5)
            >>> field.generate(np.ones((3, 3), bool), (0.25, 0.25),
            ...                geometry=GridGeometry(0.0, 0.0, 0.5, 3, 3))
            >>> array = field.to_array()
            >>> array.dtype, array.shape
            (dtype('float32'), (4, 4))
        """
        self._require()
        assert self._distances is not None
        return np.array(self._distances, dtype=np.float32, copy=True)

    @classmethod
    def from_array(
        cls,
        data: ArrayLike,
        *,
        geometry: GridGeometry,
        target_grid: Sequence[int] | None = None,
    ) -> "DistanceField":
        """从存储的距离数组及其坐标系重建距离场。

        只恢复距离值。速度掩码、未膨胀的障碍集和时间戳网格都是*生成的产物*，无法仅
        从距离恢复，因此重建后的距离场支持所有只读查询
        （:meth:`distance_at_world`、:meth:`world_to_grid`、
        :func:`reusable_model.gridmap.descent.descent_direction`），但不支持
        :meth:`recompute_local`——它会抛出明确的错误，而不是假装没有障碍。

        参数:
            data: 距离（单位米）的二维数组；负值会被解读为 :attr:`UNREACHABLE`。
            geometry: 生成该数组时所用的坐标系。``geometry.resolution`` 会成为
                距离场的分辨率，因此数组与坐标系不会彼此漂移。
            target_grid: ``(px, py)`` 目标单元。``None`` 会选取取值最小的非负单元，
                对任何由 :meth:`generate` 构建的距离场而言那就是目标。

        返回:
            一个可直接查询的 :class:`DistanceField`。

        异常:
            TypeError: 若 ``geometry`` 不是 :class:`GridGeometry`，或 ``data``
                不是 array-like。
            ValueError: 若 ``data`` 不是二维、其形状与坐标系不一致，或
                ``target_grid`` 不是 ``data`` 内的单元。

        示例:
            >>> import numpy as np
            >>> geometry = GridGeometry(0.0, 0.0, 0.5, 3, 3)
            >>> stored = np.array([[0.0, 0.5, 1.0], [0.5, 1.0, 1.5],
            ...                    [1.0, 1.5, 2.0]], dtype=np.float32)
            >>> field = DistanceField.from_array(stored, geometry=geometry)
            >>> field.target_grid, field.distance_at_world(0.75, 0.25)
            ((0, 0), 0.5)
        """
        if not isinstance(geometry, GridGeometry):
            raise TypeError(
                f"geometry must be a GridGeometry, got {type(geometry).__name__}: {geometry!r}"
            )
        array = _as_2d_array(data, "data", dtype=np.float32)
        if array.shape != (geometry.height, geometry.width):
            raise ValueError(
                f"data shape {array.shape} does not match geometry (height, width) "
                f"{(geometry.height, geometry.width)}"
            )

        field = cls(resolution=geometry.resolution)
        field._geometry = geometry
        field._distances = array

        if target_grid is None:
            reachable = np.where(array >= 0.0, array, np.inf)
            if not np.any(np.isfinite(reachable)):
                raise ValueError(
                    "data holds no reachable cell (every value is negative), so no "
                    "goal cell can be inferred; pass target_grid explicitly"
                )
            row, col = np.unravel_index(int(np.argmin(reachable)), array.shape)
            field._target_grid = (int(col), int(row))
        else:
            px, py = _as_xy(target_grid, "target_grid")
            if px != math.floor(px) or py != math.floor(py):
                raise ValueError(f"target_grid must hold integer cell indices, got {target_grid!r}")
            if not (0 <= int(px) < geometry.width and 0 <= int(py) < geometry.height):
                raise ValueError(
                    f"target_grid=({int(px)}, {int(py)}) is outside the "
                    f"{geometry.width}x{geometry.height} field"
                )
            field._target_grid = (int(px), int(py))
        return field

    def save_visualization(
        self,
        path: str,
        *,
        overlay_arrow: Sequence[float] | None = None,
    ) -> np.ndarray:
        """写出距离场的伪彩色 PNG 并返回图像。

        距离会被归一化到 ``[0, 254]`` 并经过 JET 颜色映射；``255`` 被保留不用，以便
        叠加物仍可区分。目标用白色圆圈标出，且在写出前数组会垂直翻转，因为图像格式
        的行号向下计数，而距离场向上计数——直接写 ``self._distances`` 会让地图绕其
        水平中线镜像（见 :mod:`reusable_model.gridmap.occupancy` 的模块 docstring）。

        参数:
            path: 目标文件名。父目录必须已存在；OpenCV 不会创建它们。
            overlay_arrow: 可选的 ``(x, y, yaw)``，世界坐标与弧度，会绘成绿色箭头
                ——这是检查 :mod:`reusable_model.gridmap.descent` 产生的航向与其来源距离场是否
                一致的自然方式。

        返回:
            写出的 BGR ``uint8`` 图像，形状为 ``(H, W, 3)``。

        异常:
            ImportError: 若未安装 OpenCV。
            ValueError: 若距离场尚未生成、``path`` 不是非空字符串，或
                ``overlay_arrow`` 不是三个有限数。
            OSError: 若 OpenCV 报告文件无法写出。

        示例:
            >>> import numpy as np
            >>> field = DistanceField(resolution=0.5)
            >>> field.generate(np.ones((3, 3), bool), (0.25, 0.25),
            ...                geometry=GridGeometry(0.0, 0.0, 0.5, 3, 3))
            >>> image = field.save_visualization("/tmp/field.png")   # doctest: +SKIP
            >>> image.shape                                          # doctest: +SKIP
            (4, 4, 3)
        """
        geometry = self._require()
        cv2 = _require_cv2()
        if not isinstance(path, str):
            raise TypeError(f"path must be a string, got {type(path).__name__}: {path!r}")
        if not path.strip():
            raise ValueError("path must be a non-empty file name, got ''")
        arrow: tuple[float, float, float] | None = None
        if overlay_arrow is not None:
            values = np.asarray(overlay_arrow, dtype=float).reshape(-1)
            if values.size != 3 or not np.all(np.isfinite(values)):
                raise ValueError(
                    f"overlay_arrow must be three finite numbers (x, y, yaw), got "
                    f"{overlay_arrow!r}"
                )
            arrow = (float(values[0]), float(values[1]), float(values[2]))

        assert self._distances is not None and self._target_grid is not None
        scaled = np.array(self._distances, dtype=np.float32, copy=True)
        scaled[scaled < 0.0] = 0.0
        peak = float(scaled.max())
        grey = (scaled / peak * 254.0).astype(np.uint8) if peak > 0.0 else scaled.astype(np.uint8)
        coloured = cv2.applyColorMap(grey, cv2.COLORMAP_JET)

        tx, ty = self._target_grid
        cv2.circle(coloured, (tx, ty), max(1, min(coloured.shape[:2]) // 40), (255, 255, 255), -1)

        # 图像约定：第 0 行对应世界 y 的最大值。
        image = np.ascontiguousarray(np.flipud(coloured))
        height = image.shape[0]
        if arrow is not None:
            ax, ay, yaw = arrow
            px, py = _pixel_from_world(geometry, ax, ay)
            length = max(2, min(image.shape[0], image.shape[1]) // 4)
            end_x = px + length * math.cos(yaw)
            end_y = py + length * math.sin(yaw)
            cv2.arrowedLine(
                image,
                (px, height - 1 - py),
                (int(round(end_x)), height - 1 - int(round(end_y))),
                (0, 255, 0),
                2,
                tipLength=0.3,
            )

        if not cv2.imwrite(path, image):
            raise OSError(f"OpenCV could not write the visualisation to {path!r}")
        logger.debug("distance field visualisation written to %s", path)
        return image


def create_distance_field_for_region(
    labels: ArrayLike,
    geometry: GridGeometry,
    region_ids: Sequence[int],
    target_world: Sequence[float],
    *,
    resolution: float = 0.06,
    brake_distance: float = 1.0,
    robot_radius: float = 0.0,
) -> DistanceField:
    """一次调用为一组已标记区域构建 :class:`DistanceField`。

    这是常见拓扑导航场景的便捷入口：来自 :mod:`reusable_model.gridmap.regions` 的标签数组、
    构成一个可导航区域的房间 id，以及它们内部的一个目标。传播前会通过门洞把这些
    区域焊接在一起，因此即使标记把它们分成了两个连通分量，也能从走廊到达厨房里的
    目标。

    与原辅助函数不同，它不读取文件：没有 ``.npy`` 路径、没有 JSON 附属文件、没有
    输出目录。加载标签数组、选择在哪里写调试图像都是调用方的事，这让该函数在测试、
    notebook 或服务中都能使用，无需固定的文件系统布局。

    参数:
        labels: 形状为 ``(geometry.height, geometry.width)`` 的区域 id 二维数组；
            ``0`` 表示「无区域」。
        geometry: ``labels`` 的坐标系。
        region_ids: 要焊接在一起并在其中导航的区域；至少一个，且每个 id 都必须
            存在于 ``labels`` 中。
        target_world: 目标 ``(x, y)``，单位米，位于其中一个区域内。
        resolution: 距离场单元尺寸，单位米。
        brake_distance: 墙前速度斜坡的宽度，单位米。
        robot_radius: 区域边界的硬膨胀，单位米。默认值为何是 ``0.0`` 见
            :meth:`DistanceField.generate`。

    返回:
        一个已生成的 :class:`DistanceField`。

    异常:
        TypeError: 若 ``labels`` 不是 array-like，或 ``geometry`` 不是
            :class:`GridGeometry`。
        ValueError: 若标签数组与坐标系不匹配、``region_ids`` 为空或引用了缺失的
            区域、``resolution``/``brake_distance``/``robot_radius`` 超出范围，
            或目标不在合并后的区域内。
        RuntimeError: 若未安装 OpenCV（膨胀合并需要它）。

    示例:
        >>> import numpy as np
        >>> labels = np.zeros((9, 9), np.int32)
        >>> labels[:, 0:4] = 1
        >>> labels[:, 5:9] = 2
        >>> labels[4, 4] = 1                       # 一个像素宽的门洞
        >>> geometry = GridGeometry(0.0, 0.0, 0.1, 9, 9)
        >>> field = create_distance_field_for_region(labels, geometry, [1, 2],
        ...                                          (0.65, 0.45), resolution=0.1)
        >>> field.distance_at_world(0.15, 0.45) > 0.0
        True
    """
    field = DistanceField(resolution=resolution)
    field.generate(
        labels,
        target_world,
        geometry=geometry,
        brake_distance=brake_distance,
        robot_radius=robot_radius,
        region_ids=region_ids,
    )
    return field
