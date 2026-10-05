"""二维占据栅格，并显式约定世界坐标与像素坐标的对应关系。

导航 bug 最常见的单一来源，就是关于「图像的哪个角是地图原点、y 轴朝哪个方向
增长」的默许分歧。本模块把这一约定显式化：

*内部（库）约定*
    ``data[py, px]``，其中行 ``0`` 对应世界 y 的**最小值**，列 ``0`` 对应
    世界 x 的**最小值**。因此世界 y 随行索引增大，与世界坐标系本身完全一致。
    :class:`OccupancyGrid` 中的每个方法都遵循这一约定。

*图像 / PGM / ROS map_server 约定*
    行 ``0`` 是图像的**顶部**，即世界 y 的最大值，因为图像格式与 OpenCV 的行
    从上往下计数，而世界 y 从下往上计数。使用
    :meth:`OccupancyGrid.flip_vertically` 在两种约定之间转换，并在写地图文件
    前阅读 :meth:`OccupancyGrid.to_ros_yaml` 的说明。

灰度值遵循 ROS/PGM 约定：*值越大*表示*越空闲*：``254`` 空闲，``205`` 未知，
``0`` 占据。

依赖：:mod:`numpy` 与 :mod:`yaml`。OpenCV 是可选的，仅在需要它的辅助函数中
惰性导入。
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike

logger = logging.getLogger(__name__)

__all__ = [
    "FREE",
    "OCCUPIED",
    "UNKNOWN",
    "GridGeometry",
    "OccupancyGrid",
]

#: 已知可通行栅格的灰度值（ROS/PGM：值越大越空闲）。
FREE: int = 254
#: 已知被阻挡栅格的灰度值。
OCCUPIED: int = 0
#: 从未被观测过栅格的灰度值。
UNKNOWN: int = 205

# 哨兵值，告知 ``value_at_*`` 抛出异常而非返回默认值。
_MISSING: Any = object()


@dataclass(frozen=True)
class GridGeometry:
    """二维栅格的轴对齐度量坐标系。

    ``origin_x``/``origin_y`` 是像素 ``(0, 0)`` 左下角的世界坐标——等价于
    栅格所覆盖的 x 最小值与 y 最小值。一个栅格单元占据
    ``[origin_x + px * resolution, origin_x + (px + 1) * resolution)``。

    该 dataclass 是不可变的（frozen），因此一个 geometry 可以在占据栅格与
    距离场之间共享，而不会被其中一方悄悄修改。

    属性:
        origin_x: 像素 ``(0, 0)`` 左下角的世界 x，单位米。
        origin_y: 像素 ``(0, 0)`` 左下角的世界 y，单位米。
        resolution: 每个像素对应的单元尺寸，单位米/像素。
        width: 列数（x 方向范围），至少为 1。
        height: 行数（y 方向范围），至少为 1。

    异常:
        TypeError: 若某个字段无法解释为数字。
        ValueError: 若 ``resolution`` 不是有限且严格为正的数，若
            ``width``/``height`` 不是整数且至少为 1，或若
            ``origin_x``/``origin_y`` 不是有限数。

    示例:
        >>> g = GridGeometry(origin_x=-1.0, origin_y=-2.0, resolution=0.05,
        ...                  width=100, height=80)
        >>> g.x_max, g.y_max
        (4.0, 2.0)
        >>> g.bounds
        (-1.0, -2.0, 4.0, 2.0)
    """

    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int

    def __post_init__(self) -> None:
        """强制转换并校验每个字段。

        异常:
            TypeError: 若某个字段不是实数。
            ValueError: 若某个字段不是有限数，若 ``resolution <= 0``，或若
                ``width``/``height`` 不是 ``>= 1`` 的整数。
        """
        for name in ("origin_x", "origin_y", "resolution"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float, np.floating, np.integer)):
                raise TypeError(
                    f"GridGeometry.{name} must be a real number, got "
                    f"{type(value).__name__}: {value!r}"
                )
            if not math.isfinite(float(value)):
                raise ValueError(f"GridGeometry.{name} must be finite, got {value!r}")
            object.__setattr__(self, name, float(value))

        if self.resolution <= 0.0:
            raise ValueError(
                f"GridGeometry.resolution must be > 0 metres per pixel, got {self.resolution!r}"
            )

        for name in ("width", "height"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
                if isinstance(value, float) and float(value).is_integer():
                    value = int(value)
                else:
                    raise TypeError(
                        f"GridGeometry.{name} must be an integer number of cells, got "
                        f"{type(value).__name__}: {value!r}"
                    )
            if int(value) < 1:
                raise ValueError(f"GridGeometry.{name} must be >= 1 cell, got {value!r}")
            object.__setattr__(self, name, int(value))

    @property
    def x_max(self) -> float:
        """栅格右边界（开区间上界）的世界 x。

        返回:
            ``origin_x + width * resolution``，单位米。

        示例:
            >>> GridGeometry(0.0, 0.0, 0.1, 10, 5).x_max
            1.0
        """
        return self.origin_x + self.width * self.resolution

    @property
    def y_max(self) -> float:
        """栅格上边界（开区间上界）的世界 y。

        返回:
            ``origin_y + height * resolution``，单位米。

        示例:
            >>> GridGeometry(0.0, 0.0, 0.1, 10, 5).y_max
            0.5
        """
        return self.origin_y + self.height * self.resolution

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """栅格的世界包围盒。

        返回:
            ``(x_min, y_min, x_max, y_max)``，单位米。

        示例:
            >>> GridGeometry(1.0, 2.0, 0.5, 4, 6).bounds
            (1.0, 2.0, 3.0, 5.0)
        """
        return (self.origin_x, self.origin_y, self.x_max, self.y_max)


class OccupancyGrid:
    """二维占据栅格，外加它所处的度量坐标系。

    像素数组按模块 docstring 所述的内部约定存储：``data[0, 0]`` 是位于世界 x
    最小值与世界 y 最小值的栅格单元，世界 y 随行索引增大。

    属性:
        data: ``(height, width)`` 的 ``uint8`` 灰度值数组，值越大表示越空闲
            （:data:`FREE`、:data:`UNKNOWN`、:data:`OCCUPIED`）。
        geometry: 描述 ``data`` 的 :class:`GridGeometry`。
    """

    def __init__(self, data: ArrayLike, geometry: GridGeometry) -> None:
        """用现有像素数组及其坐标系包装成栅格。

        参数:
            data: ``[0, 255]`` 范围内灰度值的二维 array-like。只要每个值都是
                整数，浮点数也可接受。
            geometry: 栅格坐标系；其 ``width``/``height`` 必须与
                ``data.shape[1]``/``data.shape[0]`` 一致。

        异常:
            TypeError: 若 ``geometry`` 不是 :class:`GridGeometry`，或
                ``data`` 不是 array-like。
            ValueError: 若 ``data`` 不是二维、其形状与 geometry 不一致，或某个
                值不是有限数或超出 ``[0, 255]``。

        示例:
            >>> geom = GridGeometry(0.0, 0.0, 0.5, 2, 2)
            >>> grid = OccupancyGrid([[FREE, OCCUPIED], [UNKNOWN, FREE]], geom)
            >>> grid.data.dtype
            dtype('uint8')
            >>> grid.value_at_pixel(0, 0) == FREE
            True
        """
        if not isinstance(geometry, GridGeometry):
            raise TypeError(
                f"geometry must be a GridGeometry, got {type(geometry).__name__}: {geometry!r}"
            )
        try:
            array = np.asarray(data)
        except (TypeError, ValueError) as exc:
            raise TypeError(f"data must be array-like, got {data!r}") from exc
        if array.ndim != 2:
            raise ValueError(
                f"data must be a 2D (height, width) array, got ndim={array.ndim} "
                f"with shape {array.shape}"
            )
        expected = (geometry.height, geometry.width)
        if array.shape != expected:
            raise ValueError(
                f"data shape {array.shape} does not match geometry (height, width) "
                f"{expected}"
            )
        if not np.issubdtype(array.dtype, np.integer):
            as_float = np.asarray(array, dtype=float)
            if not np.all(np.isfinite(as_float)):
                raise ValueError("data contains non-finite values; expected grey levels in [0, 255]")
            if not np.all(np.equal(np.mod(as_float, 1.0), 0.0)):
                raise ValueError(
                    "data contains fractional values; occupancy grey levels must be "
                    "integers in [0, 255]"
                )
            array = as_float.astype(np.int64)
        if array.size and (int(array.min()) < 0 or int(array.max()) > 255):
            raise ValueError(
                f"data must hold grey levels in [0, 255], got min={int(array.min())} "
                f"max={int(array.max())}"
            )
        self.data: np.ndarray = np.ascontiguousarray(array, dtype=np.uint8)
        self.geometry: GridGeometry = geometry

    @classmethod
    def from_array(
        cls,
        data: ArrayLike,
        *,
        origin: Sequence[float] = (0.0, 0.0),
        resolution: float,
    ) -> "OccupancyGrid":
        """从裸数组加原点与分辨率构建一个栅格。

        参数:
            data: ``[0, 255]`` 范围内灰度值的二维 array-like；其形状决定
                ``height`` 与 ``width``。
            origin: 像素 ``(0, 0)`` 左下角的世界 ``(x, y)`` 坐标。
            resolution: 每个像素对应的单元尺寸，单位米/像素，必须 ``> 0``。

        返回:
            一个新的 :class:`OccupancyGrid`。

        异常:
            TypeError: 若 ``origin`` 不是两个数字组成的序列。
            ValueError: 若 ``origin`` 不是两个有限分量，或数组/分辨率无效
                （见 :class:`GridGeometry`）。

        示例:
            >>> grid = OccupancyGrid.from_array([[FREE, FREE], [OCCUPIED, FREE]],
            ...                                 origin=(-1.0, -1.0), resolution=0.1)
            >>> grid.geometry.width, grid.geometry.height
            (2, 2)
            >>> grid.geometry.bounds
            (-1.0, -1.0, -0.8, -0.8)
        """
        if isinstance(origin, (str, bytes)) or not isinstance(origin, Sequence):
            try:
                origin_array = np.asarray(origin, dtype=float).reshape(-1)
            except (TypeError, ValueError) as exc:
                raise TypeError(f"origin must be a (x, y) sequence, got {origin!r}") from exc
        else:
            origin_array = np.asarray(origin, dtype=float).reshape(-1)
        if origin_array.size != 2 or not np.all(np.isfinite(origin_array)):
            raise ValueError(f"origin must be two finite numbers (x, y), got {origin!r}")

        array = np.asarray(data)
        if array.ndim != 2:
            raise ValueError(
                f"data must be a 2D (height, width) array, got ndim={array.ndim} "
                f"with shape {array.shape}"
            )
        geometry = GridGeometry(
            origin_x=float(origin_array[0]),
            origin_y=float(origin_array[1]),
            resolution=resolution,
            width=int(array.shape[1]),
            height=int(array.shape[0]),
        )
        return cls(data, geometry)

    # ------------------------------------------------------------------
    # 坐标变换
    # ------------------------------------------------------------------
    def world_to_pixel(self, x: float, y: float) -> tuple[int, int]:
        """把世界坐标点转换为像素索引，并夹取到栅格范围内。

        结果始终是有效索引：地图外的查询会被夹取到 ``[0, width - 1]`` 与
        ``[0, height - 1]``，这样调用方无需做边界检查即可索引 ``data``。注意
        夹取上界是 ``width - 1``（最后一个有效列）；更早版本的代码夹取到
        ``width``，对位于或超出右/上边界的点会产生越界索引。若需要区分
        「在地图之外」与「在地图边界上」，请先用 :meth:`in_bounds_world`。

        参数:
            x: 世界 x，单位米。
            y: 世界 y，单位米。

        返回:
            ``(px, py)`` 像素索引，``px`` 在 ``[0, width - 1]``、``py`` 在
            ``[0, height - 1]`` 范围内。

        异常:
            TypeError: 若 ``x`` 或 ``y`` 不是实数。
            ValueError: 若 ``x`` 或 ``y`` 不是有限数。

        示例:
            >>> grid = OccupancyGrid.from_array(np.zeros((4, 6), np.uint8),
            ...                                 origin=(0.0, 0.0), resolution=1.0)
            >>> grid.world_to_pixel(2.5, 1.5)
            (2, 1)
            >>> grid.world_to_pixel(99.0, -99.0)  # clamped
            (5, 0)
        """
        fx, fy = _finite_floats(x, y)
        px = math.floor((fx - self.geometry.origin_x) / self.geometry.resolution)
        py = math.floor((fy - self.geometry.origin_y) / self.geometry.resolution)
        return (
            min(max(px, 0), self.geometry.width - 1),
            min(max(py, 0), self.geometry.height - 1),
        )

    def pixel_to_world(self, px: int, py: int) -> tuple[float, float]:
        """把像素索引转换为其中心的世界坐标。

        参数:
            px: 列索引；插值时可以是小数。
            py: 行索引；插值时可以是小数。

        返回:
            该单元中心的世界 ``(x, y)`` 坐标，单位米。

        异常:
            TypeError: 若 ``px`` 或 ``py`` 不是实数。
            ValueError: 若 ``px`` 或 ``py`` 不是有限数。

        示例:
            >>> grid = OccupancyGrid.from_array(np.zeros((4, 6), np.uint8),
            ...                                 origin=(0.0, 0.0), resolution=1.0)
            >>> grid.pixel_to_world(2, 1)
            (2.5, 1.5)
            >>> grid.world_to_pixel(*grid.pixel_to_world(2, 1))
            (2, 1)
        """
        fx, fy = _finite_floats(px, py)
        res = self.geometry.resolution
        return (
            self.geometry.origin_x + (fx + 0.5) * res,
            self.geometry.origin_y + (fy + 0.5) * res,
        )

    def world_to_pixel_batch(
        self, xs: ArrayLike, ys: ArrayLike
    ) -> tuple[np.ndarray, np.ndarray]:
        """向量化的 :meth:`world_to_pixel`，包含相同的夹取行为。

        参数:
            xs: 世界 x 值的 array-like。
            ys: 世界 y 值的 array-like，长度与 ``xs`` 相同。

        返回:
            ``(px, py)`` 已夹取的 ``int64`` 像素索引数组。

        异常:
            ValueError: 若两个输入长度不同、含非有限值，或不是一维。

        示例:
            >>> grid = OccupancyGrid.from_array(np.zeros((4, 6), np.uint8),
            ...                                 origin=(0.0, 0.0), resolution=1.0)
            >>> px, py = grid.world_to_pixel_batch([0.5, 3.5], [1.5, 2.5])
            >>> px.tolist(), py.tolist()
            ([0, 3], [1, 2])
        """
        x = _finite_1d(xs, "xs")
        y = _finite_1d(ys, "ys")
        if x.shape != y.shape:
            raise ValueError(f"xs and ys must have the same length, got {x.shape} and {y.shape}")
        res = self.geometry.resolution
        px = np.floor((x - self.geometry.origin_x) / res).astype(np.int64)
        py = np.floor((y - self.geometry.origin_y) / res).astype(np.int64)
        np.clip(px, 0, self.geometry.width - 1, out=px)
        np.clip(py, 0, self.geometry.height - 1, out=py)
        return px, py

    def pixel_to_world_batch(
        self, px: ArrayLike, py: ArrayLike
    ) -> tuple[np.ndarray, np.ndarray]:
        """向量化的 :meth:`pixel_to_world`。

        参数:
            px: 列索引的 array-like。
            py: 行索引的 array-like，长度与 ``px`` 相同。

        返回:
            ``(x, y)`` 单元中心世界坐标的 ``float64`` 数组。

        异常:
            ValueError: 若两个输入长度不同或含非有限值。

        示例:
            >>> grid = OccupancyGrid.from_array(np.zeros((4, 6), np.uint8),
            ...                                 origin=(0.0, 0.0), resolution=2.0)
            >>> x, y = grid.pixel_to_world_batch([0, 1], [2, 3])
            >>> x.tolist(), y.tolist()
            ([1.0, 3.0], [5.0, 7.0])
        """
        cx = _finite_1d(px, "px").astype(float)
        cy = _finite_1d(py, "py").astype(float)
        if cx.shape != cy.shape:
            raise ValueError(f"px and py must have the same length, got {cx.shape} and {cy.shape}")
        res = self.geometry.resolution
        return (
            self.geometry.origin_x + (cx + 0.5) * res,
            self.geometry.origin_y + (cy + 0.5) * res,
        )

    # ------------------------------------------------------------------
    # 边界 / 查询
    # ------------------------------------------------------------------
    def in_bounds_pixel(self, px: int, py: int) -> bool:
        """判断像素索引是否位于栅格内部。

        参数:
            px: 列索引。
            py: 行索引。

        返回:
            当 ``0 <= px < width`` 且 ``0 <= py < height`` 时为 ``True``。

        示例:
            >>> grid = OccupancyGrid.from_array(np.zeros((4, 6), np.uint8),
            ...                                 resolution=1.0)
            >>> grid.in_bounds_pixel(5, 3), grid.in_bounds_pixel(6, 3)
            (True, False)
        """
        return 0 <= int(px) < self.geometry.width and 0 <= int(py) < self.geometry.height

    def in_bounds_world(self, x: float, y: float) -> bool:
        """判断世界坐标点是否位于栅格的包围盒内。

        上界是开区间：恰好位于 ``x_max`` 的点属于最后一列之后的单元，而该单元
        并不存在。

        参数:
            x: 世界 x，单位米。
            y: 世界 y，单位米。

        返回:
            当该点落在 ``geometry.bounds`` 内时为 ``True``。

        异常:
            TypeError: 若 ``x`` 或 ``y`` 不是实数。
            ValueError: 若 ``x`` 或 ``y`` 不是有限数。

        示例:
            >>> grid = OccupancyGrid.from_array(np.zeros((4, 6), np.uint8),
            ...                                 origin=(0.0, 0.0), resolution=1.0)
            >>> grid.in_bounds_world(5.9, 3.9), grid.in_bounds_world(6.0, 3.9)
            (True, False)
        """
        fx, fy = _finite_floats(x, y)
        g = self.geometry
        return g.origin_x <= fx < g.x_max and g.origin_y <= fy < g.y_max

    def contains_world(self, x: float, y: float) -> bool:
        """:meth:`in_bounds_world` 的别名，名称更口语化。

        参数:
            x: 世界 x，单位米。
            y: 世界 y，单位米。

        返回:
            当该点位于栅格内部时为 ``True``。

        示例:
            >>> grid = OccupancyGrid.from_array(np.zeros((2, 2), np.uint8),
            ...                                 resolution=1.0)
            >>> grid.contains_world(0.5, 0.5)
            True
        """
        return self.in_bounds_world(x, y)

    def value_at_pixel(self, px: int, py: int, *, default: Any = _MISSING) -> int:
        """读取某个像素的灰度值。

        参数:
            px: 列索引。
            py: 行索引。
            default: 索引越界时返回的值。省略时，越界索引会抛出异常而非静默
                夹取——错误答案比异常更糟。

        返回:
            灰度值（Python ``int``）；若索引越界且提供了 ``default``，则返回
            ``default``。

        异常:
            ValueError: 若索引越界且未给出 ``default``。消息中包含越界索引与
                栅格尺寸。

        示例:
            >>> grid = OccupancyGrid.from_array([[FREE, OCCUPIED]], resolution=1.0)
            >>> grid.value_at_pixel(1, 0)
            0
            >>> grid.value_at_pixel(5, 0, default=UNKNOWN)
            205
        """
        if self.in_bounds_pixel(px, py):
            return int(self.data[int(py), int(px)])
        if default is not _MISSING:
            return default
        raise ValueError(
            f"pixel ({px}, {py}) is outside the {self.geometry.width}x{self.geometry.height} "
            f"grid; pass default= to get a fallback value instead of this error"
        )

    def value_at_world(self, x: float, y: float, *, default: Any = _MISSING) -> int:
        """读取世界坐标位置处的灰度值。

        参数:
            x: 世界 x，单位米。
            y: 世界 y，单位米。
            default: 位置位于栅格之外时返回的值。省略时，越界位置会抛出异常。

        返回:
            灰度值（Python ``int``）；若点位于栅格之外且提供了 ``default``，
            则返回 ``default``。

        异常:
            TypeError: 若 ``x`` 或 ``y`` 不是实数。
            ValueError: 若 ``x``/``y`` 不是有限数，或点位于栅格之外且未给出
                ``default``。

        示例:
            >>> grid = OccupancyGrid.from_array([[FREE, OCCUPIED]],
            ...                                 origin=(0.0, 0.0), resolution=1.0)
            >>> grid.value_at_world(1.5, 0.5)
            0
            >>> grid.value_at_world(9.0, 0.5, default=UNKNOWN)
            205
        """
        _finite_floats(x, y)
        if not self.in_bounds_world(x, y):
            if default is not _MISSING:
                return default
            raise ValueError(
                f"world point ({x}, {y}) is outside the grid bounds "
                f"{self.geometry.bounds}; pass default= to get a fallback value"
            )
        px, py = self.world_to_pixel(x, y)
        return int(self.data[py, px])

    # ------------------------------------------------------------------
    # 与 ROS map_server 的互操作
    # ------------------------------------------------------------------
    def to_ros_yaml(
        self,
        image_name: str,
        *,
        occupied_thresh: float = 0.65,
        free_thresh: float = 0.196,
        negate: int = 0,
    ) -> dict[str, Any]:
        """生成 ROS ``map_server`` 所需的 YAML 附属文件内容。

        ``origin`` 字段为 ``[x, y, yaw]``，始终指向*图像*的左下角；无论行的
        顺序如何，该点与 ``geometry.origin_x/origin_y`` 相同。

        .. note::
            ``map_server`` 读取图像时从顶行开始，因此伴随本 YAML 的图像文件
            必须由 ``flip_vertically().data`` 写出——直接写 ``self.data`` 会
            让地图沿水平中心线镜像。

        参数:
            image_name: ``image`` 键的值，通常是 YAML 文件旁的裸文件名。
            occupied_thresh: 高于该占据度的像素视为占据。
            free_thresh: 低于该占据度的像素视为空闲。
            negate: ``0``（默认）或 ``1``；为 ``1`` 时加载器在阈值判断前反转
                像素值。

        返回:
            可直接用 :func:`yaml.safe_dump` 导出的普通 ``dict``。

        异常:
            TypeError: 若 ``image_name`` 不是字符串。
            ValueError: 若 ``image_name`` 为空、若阈值不在 ``[0, 1]`` 内、若
                ``free_thresh >= occupied_thresh``，或若 ``negate`` 不是
                ``0``/``1``。

        示例:
            >>> grid = OccupancyGrid.from_array([[FREE]], origin=(1.0, 2.0), resolution=0.05)
            >>> sorted(grid.to_ros_yaml("map.pgm"))
            ['free_thresh', 'image', 'negate', 'occupied_thresh', 'origin', 'resolution']
            >>> grid.to_ros_yaml("map.pgm")["origin"]
            [1.0, 2.0, 0.0]
        """
        if not isinstance(image_name, str):
            raise TypeError(f"image_name must be a string, got {type(image_name).__name__}")
        if not image_name.strip():
            raise ValueError("image_name must be a non-empty path/name, got ''")
        for name, value in (("occupied_thresh", occupied_thresh), ("free_thresh", free_thresh)):
            if not 0.0 <= float(value) <= 1.0:
                raise ValueError(f"{name} must be within [0, 1], got {value!r}")
        if float(free_thresh) >= float(occupied_thresh):
            raise ValueError(
                f"free_thresh must be smaller than occupied_thresh, got "
                f"{free_thresh!r} >= {occupied_thresh!r}"
            )
        if int(negate) not in (0, 1):
            raise ValueError(f"negate must be 0 or 1, got {negate!r}")

        return {
            "image": image_name,
            "resolution": float(self.geometry.resolution),
            "origin": [float(self.geometry.origin_x), float(self.geometry.origin_y), 0.0],
            "negate": int(negate),
            "occupied_thresh": float(occupied_thresh),
            "free_thresh": float(free_thresh),
        }

    @classmethod
    def from_ros_yaml(
        cls,
        yaml_dict: Mapping[str, Any],
        image: ArrayLike,
    ) -> "OccupancyGrid":
        """从 ROS ``map_server`` 的 YAML dict 及其图像构建栅格。

        假定图像采用标准 PGM/ROS 布局（行 ``0`` 是图像顶部，即世界 y 的最大
        值），并将其翻转为内部约定，因此之后 ``data[0, 0]`` 是位于 ``origin``
        处的单元。

        参数:
            yaml_dict: 含 ``resolution`` 与 ``origin`` 键的映射。``origin`` 的第
                三个元素（yaw）可接受并会被忽略；``negate`` 通过反转灰度值生效。
            image: 从 PGM/PNG 文件加载的灰度值二维 array-like。

        返回:
            采用内部约定的新 :class:`OccupancyGrid`。

        异常:
            TypeError: 若 ``yaml_dict`` 不是映射。
            ValueError: 若缺少必需键、若 ``origin`` 不是两个或三个有限数、若
                ``resolution`` 不是正数，或若图像对
                :class:`OccupancyGrid` 无效。

        示例:
            >>> meta = {"resolution": 0.1, "origin": [0.0, 0.0, 0.0], "negate": 0}
            >>> grid = OccupancyGrid.from_ros_yaml(meta, [[OCCUPIED], [FREE]])
            >>> grid.value_at_pixel(0, 0)  # 图像的最底行
            254
        """
        if not isinstance(yaml_dict, Mapping):
            raise TypeError(
                f"yaml_dict must be a mapping, got {type(yaml_dict).__name__}: {yaml_dict!r}"
            )
        for key in ("resolution", "origin"):
            if key not in yaml_dict:
                raise ValueError(
                    f"map yaml is missing the required key {key!r}; got keys "
                    f"{sorted(yaml_dict)!r}"
                )

        try:
            resolution = float(yaml_dict["resolution"])
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"map yaml 'resolution' must be a positive number of metres per pixel, "
                f"got {yaml_dict['resolution']!r}"
            ) from exc
        if not math.isfinite(resolution):
            raise ValueError(
                f"map yaml 'resolution' must be finite, got {yaml_dict['resolution']!r}"
            )
        origin = np.asarray(yaml_dict["origin"], dtype=float).reshape(-1)
        if origin.size not in (2, 3) or not np.all(np.isfinite(origin)):
            raise ValueError(
                f"origin must be [x, y] or [x, y, yaw] finite numbers, got "
                f"{yaml_dict['origin']!r}"
            )
        if origin.size == 3 and abs(float(origin[2])) > 1e-9:
            logger.warning(
                "map origin yaw %.6f rad is ignored; this grid representation is "
                "axis-aligned",
                float(origin[2]),
            )

        negate = int(yaml_dict.get("negate", 0))
        if negate not in (0, 1):
            raise ValueError(f"negate must be 0 or 1, got {yaml_dict.get('negate')!r}")

        array = np.asarray(image)
        if array.ndim != 2:
            raise ValueError(
                f"image must be a 2D array of grey levels, got ndim={array.ndim} "
                f"with shape {array.shape}"
            )
        if negate:
            # 先在浮点空间中反转，这样非整数输入会由构造函数拒绝，而不是被
            # 静默截断。
            array = 255 - np.asarray(array, dtype=float)

        geometry = GridGeometry(
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            resolution=resolution,
            width=int(array.shape[1]),
            height=int(array.shape[0]),
        )
        # PGM 第 0 行 = 世界 y 最大值 -> 内部第 0 行 = 世界 y 最小值。
        return cls(np.flipud(array), geometry)

    # ------------------------------------------------------------------
    # 派生视图
    # ------------------------------------------------------------------
    def flip_vertically(self) -> "OccupancyGrid":
        """返回行顺序反转后的副本。

        转换为图像约定时会*把*内部的第 ``0`` 行（世界 y 最小值）变成图像的底部
        行，这正是 OpenCV、PGM 与 ROS ``map_server`` 所期望的；反向转换是同一
        个操作，因为垂直翻转是自身的逆运算。geometry 保持不变：``origin`` 仍
        表示地图的最小 x / 最小 y 角，而不是数组的某个角。

        返回:
            一个新的 :class:`OccupancyGrid`，与本对象不共享任何可变状态。

        示例:
            >>> grid = OccupancyGrid.from_array([[1, 2], [3, 4]], resolution=1.0)
            >>> grid.flip_vertically().data.tolist()
            [[3, 4], [1, 2]]
        """
        return OccupancyGrid(np.flipud(self.data), self.geometry)

    def region_labels(self, free_min: int = UNKNOWN + 1) -> np.ndarray:
        """返回可供区域分析使用的可通行单元 ``int32`` 掩码。

        输出形状专为 :mod:`reusable_model.gridmap.regions` 设计：``1`` 标记可以行走的
        单元，``0`` 标记其他所有单元，因此可以直接送入 ``label_connected``，
        或在单个区域足够时当作标签数组使用。

        默认阈值是 ``UNKNOWN + 1 = 206``，对应 ROS 映射
        ``occupancy = (255 - value) / 255`` 与 ``free_thresh = 0.196``：只保留
        明确空闲的单元，未知与占据单元都被排除。调低 ``free_min`` 可以把
        「未知」也视为可通行。

        参数:
            free_min: 包含式灰度阈值，高于该值的单元视为空闲。

        返回:
            ``(height, width)`` 的 ``int32`` 数组，元素为 0 与 1。

        异常:
            ValueError: 若 ``free_min`` 不是 ``[0, 256]`` 内的整数。

        示例:
            >>> grid = OccupancyGrid.from_array([[FREE, UNKNOWN], [OCCUPIED, FREE]],
            ...                                 resolution=1.0)
            >>> grid.region_labels().tolist()
            [[1, 0], [0, 1]]
        """
        if not isinstance(free_min, (int, np.integer)) or isinstance(free_min, bool):
            raise TypeError(f"free_min must be an integer, got {free_min!r}")
        if not 0 <= int(free_min) <= 256:
            raise ValueError(f"free_min must be within [0, 256], got {free_min!r}")
        return (self.data.astype(np.int32) >= int(free_min)).astype(np.int32)


def _finite_floats(x: Any, y: Any) -> tuple[float, float]:
    """校验一对标量坐标。

    参数:
        x: 第一个坐标。
        y: 第二个坐标。

    返回:
        以 Python float 表示的这一对值。

    异常:
        TypeError: 若任一值不是实数。
        ValueError: 若任一值不是有限数。
    """
    out: list[float] = []
    for name, value in (("x", x), ("y", y)):
        if isinstance(value, bool) or not isinstance(value, (int, float, np.floating, np.integer)):
            raise TypeError(f"{name} must be a real number, got {type(value).__name__}: {value!r}")
        as_float = float(value)
        if not math.isfinite(as_float):
            raise ValueError(f"{name} must be finite, got {value!r}")
        out.append(as_float)
    return out[0], out[1]


def _finite_1d(values: ArrayLike, name: str) -> np.ndarray:
    """校验一个由有限数字组成的一维 array-like。

    参数:
        values: 待检查的 array-like。
        name: 错误消息中使用的参数名。

    返回:
        形状为 ``(n,)`` 的 ``float64`` 数组。

    异常:
        TypeError: 若 ``values`` 不是 array-like。
        ValueError: 若结果不是一维或含非有限值。
    """
    try:
        array = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be array-like of numbers, got {values!r}") from exc
    if array.ndim != 1:
        raise ValueError(f"{name} must be 1D, got shape {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite numbers, got {values!r}")
    return array
