"""面向二维导航栅格的连通分量区域分析。

这里的原语回答拓扑导航器反复追问的、关于楼层平面的几个问题：

* 哪些单元属于同一个可行走房间？
* 哪些房间通过门洞彼此相接，它们相距多远？
* 哪里有一个*保证*落在给定房间内部的点（用于画标签，或作为「前往房间中心」
  指令的目标）？

提供两种标记（labeling）方式。 :func:`label_connected` 是对布尔空闲掩码的普通
连通性。 :func:`label_with_step_constraint` 还额外拒绝合并地面高度差超过
``max_step`` 的相邻单元；正是这一条额外的谓词，把点云高度图切分成每段楼梯或
每层坡道各自独立的房间，因为一个无法攀爬 20 cm 台阶的机器人不应把两层平台
视为同一区域。

依赖：:mod:`numpy`。OpenCV 是可选的，仅形态学辅助函数
:func:`merge_regions_by_dilation` 与绘图辅助函数 :func:`draw_lines` 需要它；
两者都惰性导入，并在缺失时抛出清晰的错误。
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any, Iterable, Sequence

import numpy as np
from numpy.typing import ArrayLike

from .occupancy import GridGeometry

logger = logging.getLogger(__name__)

__all__ = [
    "RegionInfo",
    "UnionFind",
    "connectivity_from_segments",
    "draw_lines",
    "label_connected",
    "label_with_step_constraint",
    "merge_regions_by_dilation",
    "region_centroid_world",
    "region_point_inside",
]

_NEIGHBOUR_OFFSETS_4: tuple[tuple[int, int], ...] = ((0, 1), (1, 0))
_NEIGHBOUR_OFFSETS_8: tuple[tuple[int, int], ...] = ((0, 1), (1, 0), (1, 1), (1, -1))


def _require_cv2() -> Any:
    """按需导入 OpenCV，并给出可操作的错误消息。

    返回:
        导入的 :mod:`cv2` 模块。

    异常:
        RuntimeError: 若未安装 OpenCV。本模块只有形态学与绘图辅助函数需要它；
            标记（labelling）功能无需它即可工作。
    """
    try:
        import cv2  # noqa: PLC0415 - 惰性、可选依赖
    except ImportError as exc:  # pragma: no cover - 取决于环境
        raise RuntimeError(
            "this helper needs OpenCV; install it with `pip install opencv-python` "
            "(the labelling functions in this module work without it)"
        ) from exc
    return cv2


def _as_2d_bool(mask: ArrayLike, name: str) -> np.ndarray:
    """把 ``mask`` 转换为二维布尔数组。

    参数:
        mask: array-like；非零元素变为 ``True``。
        name: 错误消息中使用的参数名。

    返回:
        连续的 ``(H, W)`` ``bool`` 数组。

    异常:
        TypeError: 若 ``mask`` 不是 array-like。
        ValueError: 若 ``mask`` 不是二维。
    """
    try:
        array = np.asarray(mask)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be array-like, got {mask!r}") from exc
    if array.ndim != 2:
        raise ValueError(f"{name} must be a 2D (height, width) array, got shape {array.shape}")
    return np.ascontiguousarray(array != 0)


def _as_2d_labels(labels: ArrayLike, name: str = "labels") -> np.ndarray:
    """把 ``labels`` 转换为二维有符号整数数组。

    参数:
        labels: 区域 id 的 array-like；``0`` 表示「无区域」。
        name: 错误消息中使用的参数名。

    返回:
        ``(H, W)`` 的 ``int32`` 数组。

    异常:
        TypeError: 若 ``labels`` 不是 array-like 或不是数值。
        ValueError: 若 ``labels`` 不是二维或含负 id。
    """
    try:
        array = np.asarray(labels)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be array-like, got {labels!r}") from exc
    if array.ndim != 2:
        raise ValueError(f"{name} must be a 2D (height, width) array, got shape {array.shape}")
    if not np.issubdtype(array.dtype, np.integer):
        try:
            as_float = np.asarray(array, dtype=float)
        except (TypeError, ValueError) as exc:
            raise TypeError(
                f"{name} must hold integer region ids, got dtype {array.dtype}"
            ) from exc
        if not np.all(np.isfinite(as_float)) or not np.all(np.equal(np.mod(as_float, 1.0), 0.0)):
            raise ValueError(
                f"{name} must hold integer region ids, got non-integral values "
                f"(dtype {array.dtype})"
            )
        array = as_float.astype(np.int64)
    if array.size and int(array.min()) < 0:
        raise ValueError(
            f"{name} must not hold negative region ids (0 means 'no region'), "
            f"got min={int(array.min())}"
        )
    return np.ascontiguousarray(array, dtype=np.int32)


def _neighbour_slices(
    rows: int, cols: int, dy: int, dx: int
) -> tuple[tuple[slice, slice], tuple[slice, slice]]:
    """返回把每个单元与其 ``(dy, dx)`` 邻居配对的索引切片。

    参数:
        rows: 数组高度。
        cols: 数组宽度。
        dy: 邻居的行偏移。
        dx: 邻居的列偏移。

    返回:
        ``(slice_a, slice_b)``，使得 ``A[slice_a]`` 与 ``A[slice_b]`` 是形状相同、
        且已配对的单元块。

    异常:
        ValueError: 若偏移量大于数组，则不存在任何配对。
    """
    if abs(dy) >= rows or abs(dx) >= cols:
        raise ValueError(
            f"neighbour offset ({dy}, {dx}) does not fit a {rows}x{cols} array"
        )
    row_a, row_b = (0, dy) if dy >= 0 else (-dy, 0)
    col_a, col_b = (0, dx) if dx >= 0 else (-dx, 0)
    n_rows = rows - abs(dy)
    n_cols = cols - abs(dx)
    return (
        (slice(row_a, row_a + n_rows), slice(col_a, col_a + n_cols)),
        (slice(row_b, row_b + n_rows), slice(col_b, col_b + n_cols)),
    )


def _pair_edges(
    free: np.ndarray,
    connectivity: int,
    height_map: np.ndarray | None = None,
    max_step: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """收集所有可接受邻接对的扁平单元 id。

    参数:
        free: ``(H, W)`` 布尔可通行掩码。
        connectivity: ``4`` 或 ``8``。
        height_map: 可选的 ``(H, W)`` 地面高度数组。给出时，只有当两个高度都有
            限且差值不超过 ``max_step`` 时该对才可接受。
        max_step: 允许的最大高度差，单位米。

    返回:
        ``(edge_a, edge_b)``，等长的扁平 ``int64`` id 数组。
    """
    rows, cols = free.shape
    flat = np.arange(rows * cols, dtype=np.int64).reshape(rows, cols)
    offsets = _NEIGHBOUR_OFFSETS_4 if connectivity == 4 else _NEIGHBOUR_OFFSETS_8

    ids_a: list[np.ndarray] = []
    ids_b: list[np.ndarray] = []
    for dy, dx in offsets:
        slice_a, slice_b = _neighbour_slices(rows, cols, dy, dx)
        ok = free[slice_a] & free[slice_b]
        if height_map is not None:
            ha = height_map[slice_a]
            hb = height_map[slice_b]
            with np.errstate(invalid="ignore"):
                ok &= np.isfinite(ha) & np.isfinite(hb) & (np.abs(ha - hb) <= max_step)
        if not np.any(ok):
            continue
        ids_a.append(flat[slice_a][ok])
        ids_b.append(flat[slice_b][ok])

    if not ids_a:
        empty = np.empty(0, dtype=np.int64)
        return empty, empty
    return np.concatenate(ids_a), np.concatenate(ids_b)


def _build_labelled_regions(
    free: np.ndarray,
    roots: np.ndarray,
    min_area_cells: int,
    order: str,
) -> tuple[np.ndarray, list["RegionInfo"]]:
    """把逐单元的并查集根转换为带编号、带统计的区域。

    区域按面积排名编号为 ``1..N``，因此 ``id == 1`` 是最大的房间；面积相同时
    按根 id 打破平局，以保证结果在不同运行与不同平台上确定一致。

    参数:
        free: ``(H, W)`` 布尔掩码，标记参与了标记过程的单元。
        roots: 扁平 ``int64`` 根 id 数组，每个 ``free`` 单元一个。
        min_area_cells: 小于该值的区域被丢弃并归回 ``0``。
        order: ``'desc'`` 表示最大区域编号在前，``'asc'`` 表示最小在前。

    返回:
        ``(labels, regions)``，其中 ``labels`` 是 ``(H, W)`` 的 ``int32`` 数组，
        ``regions`` 是按 id 排序的 :class:`RegionInfo` 列表。
    """
    rows, cols = free.shape
    num_cells = rows * cols
    labels_flat = np.zeros(num_cells, dtype=np.int32)

    free_ids = np.flatnonzero(free.ravel())
    if free_ids.size == 0:
        return labels_flat.reshape(rows, cols), []

    free_roots = roots[free_ids]
    # 先按根把所有单元分组一次，然后在每个连续段内做归约。
    sort_order = np.argsort(free_roots, kind="stable")
    sorted_roots = free_roots[sort_order]
    sorted_ids = free_ids[sort_order]
    segment_starts = np.flatnonzero(np.r_[True, sorted_roots[1:] != sorted_roots[:-1]])
    segment_roots = sorted_roots[segment_starts]

    xs = sorted_ids % cols
    ys = sorted_ids // cols
    counts = np.add.reduceat(np.ones(xs.size, dtype=np.int64), segment_starts)
    sums_x = np.add.reduceat(xs.astype(float), segment_starts)
    sums_y = np.add.reduceat(ys.astype(float), segment_starts)
    x_min = np.minimum.reduceat(xs, segment_starts)
    x_max = np.maximum.reduceat(xs, segment_starts)
    y_min = np.minimum.reduceat(ys, segment_starts)
    y_max = np.maximum.reduceat(ys, segment_starts)

    kept = [i for i in range(segment_roots.size) if counts[i] >= max(1, min_area_cells)]
    if order == "desc":
        kept.sort(key=lambda i: (-int(counts[i]), int(segment_roots[i])))
    else:
        kept.sort(key=lambda i: (int(counts[i]), int(segment_roots[i])))

    root_to_id = np.zeros(num_cells, dtype=np.int32)
    regions: list[RegionInfo] = []
    for new_id, slot in enumerate(kept, start=1):
        root = int(segment_roots[slot])
        root_to_id[root] = new_id
        count = int(counts[slot])
        regions.append(
            RegionInfo(
                id=new_id,
                area_cells=count,
                centroid_pixel=(float(sums_x[slot] / count), float(sums_y[slot] / count)),
                bbox=(int(x_min[slot]), int(y_min[slot]), int(x_max[slot]), int(y_max[slot])),
            )
        )

    labels_flat[free_ids] = root_to_id[free_roots]
    return labels_flat.reshape(rows, cols), regions


@dataclass(frozen=True)
class RegionInfo:
    """一个已标记区域的摘要统计信息。

    属性:
        id: 标签数组中的区域 id，始终 ``>= 1``。
        area_cells: 属于该区域的单元数量。
        centroid_pixel: ``(x, y)`` = ``(列, 行)`` 的平均位置，单位为像素。
            对于凹形区域（L 形、环形），该点可能落在区域之外；需要保证属于
            该区域的像素时请使用 :func:`region_point_inside`。
        bbox: 包含式 ``(x_min, y_min, x_max, y_max)`` 像素包围盒。
    """

    id: int
    area_cells: int
    centroid_pixel: tuple[float, float]
    bbox: tuple[int, int, int, int]


class UnionFind:
    """带路径压缩与按大小合并的并查集（disjoint-set forest）。

    它是栅格单元连通性背后的核心工具：单元是元素，任意的邻接谓词决定哪些对会
    被 :meth:`union` 合并。把它与标记代码分离，正是
    :func:`label_with_step_constraint` 得以实现的原因——只换谓词，分组机制不变。

    两种后端共享同一套 API。传入 ``size`` 会为元素 ``0 .. size - 1`` 分配稠密的
    numpy 数组（快速，内部用于扁平的单元 id）；省略则得到基于 dict 的稀疏结构，
    可接受任意非负整数元素。

    示例:
        >>> uf = UnionFind(5)
        >>> uf.union(0, 1); uf.union(3, 4); uf.union(1, 4)
        True
        True
        True
        >>> sorted(len(members) for members in uf.groups().values())
        [1, 4]
        >>> uf.find(0) == uf.find(4)
        True
    """

    def __init__(self, size: int | None = None) -> None:
        """创建一个空的森林。

        参数:
            size: 给出时，元素是稠密范围 ``0 .. size - 1``。为 ``None`` 时，
                元素在首次使用时被发现。

        异常:
            TypeError: 若 ``size`` 不是整数或 ``None``。
            ValueError: 若 ``size`` 为负。
        """
        if size is None:
            self._size: int | None = None
            self._parent: dict[int, int] = {}
            self._rank: dict[int, int] = {}
            return
        if isinstance(size, bool) or not isinstance(size, (int, np.integer)):
            raise TypeError(f"size must be an integer or None, got {size!r}")
        if int(size) < 0:
            raise ValueError(f"size must be >= 0, got {size!r}")
        self._size = int(size)
        self._array_parent = np.arange(self._size, dtype=np.int64)
        self._array_size = np.ones(self._size, dtype=np.int64)

    def _check(self, element: Any, name: str) -> int:
        """校验一个元素 id 并以 Python ``int`` 返回。

        参数:
            element: 候选元素。
            name: 用于错误消息的参数名。

        返回:
            以 ``int`` 表示的元素。

        异常:
            TypeError: 若 ``element`` 不是整数。
            ValueError: 若 ``element`` 为负或超出稠密范围。
        """
        if isinstance(element, bool) or not isinstance(element, (int, np.integer)):
            raise TypeError(f"{name} must be an integer element id, got {element!r}")
        value = int(element)
        if value < 0:
            raise ValueError(f"{name} must be >= 0, got {value}")
        if self._size is not None and value >= self._size:
            raise ValueError(
                f"{name}={value} is outside the declared range [0, {self._size}); "
                f"construct UnionFind(size={value + 1}) or omit size for sparse mode"
            )
        return value

    def find(self, element: int) -> int:
        """返回 ``element`` 所在集合的代表元（根）。

        路径压缩会把经过的链压平到根上，因此重复查询接近 O(1)。

        参数:
            element: 元素 id。在稀疏模式下，未知元素会以自己的单元素集合开始。

        返回:
            根 id。

        异常:
            TypeError: 若 ``element`` 不是整数。
            ValueError: 若 ``element`` 为负或超出稠密范围。

        示例:
            >>> uf = UnionFind()
            >>> uf.find(7) == 7
            True
        """
        value = self._check(element, "element")
        if self._size is None:
            parent = self._parent
            if value not in parent:
                parent[value] = value
                self._rank[value] = 1
                return value
            root = value
            while parent[root] != root:
                root = parent[root]
            while parent[value] != root:  # 路径压缩
                parent[value], value = root, parent[value]
            return root

        parent = self._array_parent
        root = value
        while parent[root] != root:
            root = int(parent[root])
        while parent[value] != root:
            parent[value], value = root, int(parent[value])
        return root

    def union(self, a: int, b: int) -> bool:
        """合并包含 ``a`` 与 ``b`` 的两个集合。

        按大小合并会把较小的树挂到较大的树上，从而保持森林较浅。

        参数:
            a: 第一个元素 id。
            b: 第二个元素 id。

        返回:
            若合并了两个不同的集合则返回 ``True``，若它们本就在同一集合则返回
            ``False``。

        异常:
            TypeError: 若某个元素不是整数。
            ValueError: 若某个元素为负或超出稠密范围。

        示例:
            >>> uf = UnionFind(3)
            >>> uf.union(0, 1), uf.union(0, 1)
            (True, False)
        """
        left = self._check(a, "a")
        right = self._check(b, "b")
        root_left = self.find(left)
        root_right = self.find(right)
        if root_left == root_right:
            return False

        if self._size is None:
            if self._rank[root_left] < self._rank[root_right]:
                root_left, root_right = root_right, root_left
            self._parent[root_right] = root_left
            self._rank[root_left] += self._rank[root_right]
            return True

        if self._array_size[root_left] < self._array_size[root_right]:
            root_left, root_right = root_right, root_left
        self._array_parent[root_right] = root_left
        self._array_size[root_left] += self._array_size[root_right]
        return True

    def groups(self) -> dict[int, list[int]]:
        """返回所有集合，以其根为键。

        返回:
            映射 ``{root_id: sorted(member_ids)}``，覆盖目前见过的所有元素
            （稠密模式：全部 ``size`` 个元素）。

        示例:
            >>> uf = UnionFind(4)
            >>> uf.union(0, 2)
            True
            >>> {root: members for root, members in sorted(uf.groups().items())}
            {0: [0, 2], 1: [1], 3: [3]}
        """
        buckets: dict[int, list[int]] = {}
        if self._size is None:
            elements = sorted(self._parent)
        else:
            elements = range(self._size)
        for element in elements:
            buckets.setdefault(self.find(element), []).append(int(element))
        return buckets


def label_connected(
    mask: ArrayLike,
    *,
    connectivity: int = 8,
    min_area_cells: int = 0,
    order: str = "desc",
) -> tuple[np.ndarray, list[RegionInfo]]:
    """对布尔空闲掩码的连通分量进行标记。

    非零值的单元是「空闲」的；其他所有单元在输出中保持为 ``0``。彼此接触的空闲
    单元（4 邻域或 8 邻域）构成一个区域，区域按面积编号为 ``1..N``，因此 id
    ``1`` 是最大的区域——当调用方只关心「主房间」时，这是一个方便的约定。

    参数:
        mask: 二维 array-like；非零表示空闲。
        connectivity: ``4``（上/下/左/右）或 ``8``（还包含对角）。``8`` 会让对角
            接触的单元保持在同一区域，这正是全向或可斜行机器人想要的；``4`` 更
            严格，可避免把仅在一个角接触的两条走廊合并。
        min_area_cells: 单元数少于此值的区域被丢弃（重置为 0）。``0`` 或 ``1``
            保留全部。
        order: ``'desc'`` 让最大区域编号在前，``'asc'`` 让最小在前。

    返回:
        ``(labels, regions)``：一个 ``(H, W)`` 的 ``int32`` 标签数组，以及与之
        对应、按区域 id 排序的 :class:`RegionInfo` 列表。

    异常:
        TypeError: 若 ``mask`` 不是 array-like。
        ValueError: 若 ``mask`` 不是二维、``connectivity`` 不是 4 或 8、
            ``min_area_cells`` 为负，或 ``order`` 不是 ``'desc'``/``'asc'``。

    示例:
        >>> mask = [[1, 1, 0, 0],
        ...         [1, 1, 0, 2],
        ...         [0, 0, 0, 2]]
        >>> labels, regions = label_connected(mask, connectivity=4)
        >>> [r.area_cells for r in regions]
        [4, 2]
        >>> labels[0, 0], labels[1, 3], labels[0, 2]
        (1, 2, 0)
    """
    free = _as_2d_bool(mask, "mask")
    if connectivity not in (4, 8):
        raise ValueError(f"connectivity must be 4 or 8, got {connectivity!r}")
    if isinstance(min_area_cells, bool) or not isinstance(min_area_cells, (int, np.integer)):
        raise TypeError(f"min_area_cells must be an integer, got {min_area_cells!r}")
    if int(min_area_cells) < 0:
        raise ValueError(f"min_area_cells must be >= 0, got {min_area_cells!r}")
    if order not in ("desc", "asc"):
        raise ValueError(f"order must be 'desc' or 'asc', got {order!r}")

    num_cells = free.size
    union_find = UnionFind(num_cells)
    edges_a, edges_b = _pair_edges(free, int(connectivity))
    for a, b in zip(edges_a.tolist(), edges_b.tolist()):
        union_find.union(a, b)

    roots = np.array([union_find.find(i) for i in range(num_cells)], dtype=np.int64) if num_cells else np.empty(0, np.int64)
    return _build_labelled_regions(free, roots, int(min_area_cells), order)


def label_with_step_constraint(
    free_mask: ArrayLike,
    height_map: ArrayLike,
    *,
    max_step: float,
    connectivity: int = 4,
    min_area_cells: int = 0,
    order: str = "desc",
) -> tuple[np.ndarray, list[RegionInfo]]:
    """标记那些同时通过可攀爬高度剖面相连的空闲单元。

    两个相邻单元只有在*同时*满足「都是空闲」**且**
    ``|height_a - height_b| <= max_step`` 时才会归入同一区域。这就是台阶/坡度
    约束：它把一段楼梯按每层平台切分成不同区域，让路缘与车行道保持分离，并阻止
    规划器规划出一条机器人物理上无法穿越的悬崖路线。高度为非有限值（无地面回波）
    的单元永远不会与任何单元相连。

    这里 ``connectivity`` 默认为 ``4`` 而非 ``8``：对角跨步会叠加两个高度差，因此
    一条可攀爬边的对角邻居很容易超过 ``max_step``，从而悄悄打开一条捷径。

    参数:
        free_mask: 二维 array-like；非零表示空闲。
        height_map: 地面高度的二维 array-like，单位米，形状与 ``free_mask`` 相同。
            非有限值条目被视为「无地面」。
        max_step: 机器人可穿越的最大高度差，单位米。
        connectivity: ``4`` 或 ``8``。
        min_area_cells: 单元数少于此值的区域被丢弃。
        order: ``'desc'`` 让最大区域编号在前，``'asc'`` 让最小在前。

    返回:
        ``(labels, regions)``，与 :func:`label_connected` 完全一致。

    异常:
        TypeError: 若某个输入不是 array-like。
        ValueError: 若两个数组形状不同、任一不是二维、``max_step`` 不是有限且
            非负，或 ``connectivity``/``min_area_cells``/``order`` 无效。

    示例:
        >>> free = [[1, 1, 1], [1, 1, 1]]
        >>> heights = [[0.0, 0.0, 0.9], [0.0, 0.0, 0.9]]  # a 0.9 m step at column 2
        >>> labels, regions = label_with_step_constraint(free, heights, max_step=0.2,
        ...                                              connectivity=4)
        >>> [r.area_cells for r in regions]
        [4, 2]
        >>> labels[0, 1], labels[0, 2]
        (1, 2)
    """
    free = _as_2d_bool(free_mask, "free_mask")
    try:
        heights = np.asarray(height_map, dtype=float)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"height_map must be array-like of numbers, got {height_map!r}") from exc
    if heights.shape != free.shape:
        raise ValueError(
            f"height_map shape {heights.shape} must match free_mask shape {free.shape}"
        )
    if isinstance(max_step, bool) or not isinstance(max_step, (int, float, np.floating, np.integer)):
        raise TypeError(f"max_step must be a real number of metres, got {max_step!r}")
    if not math.isfinite(float(max_step)):
        raise ValueError(f"max_step must be finite, got {max_step!r}")
    if float(max_step) < 0.0:
        raise ValueError(f"max_step must be >= 0 metres, got {max_step!r}")
    if connectivity not in (4, 8):
        raise ValueError(f"connectivity must be 4 or 8, got {connectivity!r}")
    if isinstance(min_area_cells, bool) or not isinstance(min_area_cells, (int, np.integer)):
        raise TypeError(f"min_area_cells must be an integer, got {min_area_cells!r}")
    if int(min_area_cells) < 0:
        raise ValueError(f"min_area_cells must be >= 0, got {min_area_cells!r}")
    if order not in ("desc", "asc"):
        raise ValueError(f"order must be 'desc' or 'asc', got {order!r}")

    num_cells = free.size
    union_find = UnionFind(num_cells)
    edges_a, edges_b = _pair_edges(
        free, int(connectivity), height_map=heights, max_step=float(max_step)
    )
    for a, b in zip(edges_a.tolist(), edges_b.tolist()):
        union_find.union(a, b)

    roots = (
        np.array([union_find.find(i) for i in range(num_cells)], dtype=np.int64)
        if num_cells
        else np.empty(0, np.int64)
    )
    return _build_labelled_regions(free, roots, int(min_area_cells), order)


def merge_regions_by_dilation(
    labels: ArrayLike,
    region_ids: Sequence[int],
    *,
    kernel_size: int = 7,
    iterations: int = 2,
) -> np.ndarray:
    """通过分隔区域的门洞把若干区域焊接在一起。

    连通分量标记产生的区域在构造上就是互不相交的：一个门框、一道门槛或一像素厚
    的墙就足以把走廊切成两段，此时单独在其中一个区域内计算的距离场永远无法到达
    另一个区域。这里采用的技巧是*分别膨胀每个选中的区域*，并保留至少两个膨胀
    掩码重叠的单元。这些单元恰好是两个区域彼此靠近到 ``kernel_size`` 以内的狭窄
    通道，因此把它们加回去就能重新连通这些区域，而不会把它们膨胀进墙里或膨胀到
    无关的邻居。

    返回的数组是一张*合并后的标签图*，而不是严格的逐区域标签：重叠单元被赋予参与
    其中的最小区域 id，以保证结果确定。只需要可通行掩码的调用方应使用
    ``merge_regions_by_dilation(...) > 0``。

    参数:
        labels: 区域 id 的二维 array-like；``0`` 表示「无区域」。
        region_ids: 要焊接在一起的区域。至少需要一个 id；要出现任何重叠则至少
            需要两个。
        kernel_size: 方形结构元素的边长，单位为像素。必须是奇数且至少为 3；膨胀
            向外可达 ``(kernel_size - 1) // 2 * iterations`` 个像素。
        iterations: 每个区域的膨胀遍数。

    返回:
        ``(H, W)`` 的 ``int32`` 标签数组：选中的区域，外加其膨胀重叠的单元。

    异常:
        TypeError: 若 ``labels`` 不是 array-like，或 ``region_ids`` 不是整数序列。
        ValueError: 若 ``labels`` 不是二维、``region_ids`` 为空、某个 id 在
            ``labels`` 中不存在、``kernel_size`` 不是 ``>= 3`` 的奇数，或
            ``iterations`` 不是 ``>= 1``。
        RuntimeError: 若未安装 OpenCV。

    示例:
        >>> import numpy as np
        >>> labels = np.zeros((5, 9), np.int32)
        >>> labels[:, 0:3] = 1      # left room
        >>> labels[:, 6:9] = 2      # 右侧房间，中间隔 3 像素
        >>> merged = merge_regions_by_dilation(labels, [1, 2], kernel_size=3,
        ...                                    iterations=2)
        >>> merged[:, 4].tolist()   # 膨胀相遇处的桥接单元
        [1, 1, 1, 1, 1]
        >>> merged[:, 3].tolist()   # 仍在区域 1 内，不是重叠
        [0, 0, 0, 0, 0]
    """
    label_array = _as_2d_labels(labels)
    if isinstance(region_ids, (str, bytes)) or not isinstance(region_ids, Iterable):
        raise TypeError(f"region_ids must be a sequence of integers, got {region_ids!r}")
    ids = [int(r) if isinstance(r, (int, np.integer)) and not isinstance(r, bool) else None
           for r in region_ids]
    if any(value is None for value in ids):
        raise TypeError(
            f"region_ids must contain only integers, got {list(region_ids)!r}"
        )
    region_ids_int: list[int] = [int(value) for value in ids]  # type: ignore[arg-type]
    if not region_ids_int:
        raise ValueError("region_ids must contain at least one region id, got an empty sequence")
    if any(rid <= 0 for rid in region_ids_int):
        raise ValueError(
            f"region_ids must be positive (0 means 'no region'), got {region_ids_int!r}"
        )
    if isinstance(kernel_size, bool) or not isinstance(kernel_size, (int, np.integer)):
        raise TypeError(f"kernel_size must be an integer, got {kernel_size!r}")
    if int(kernel_size) < 3 or int(kernel_size) % 2 == 0:
        raise ValueError(
            f"kernel_size must be an odd integer >= 3 pixels, got {kernel_size!r}"
        )
    if isinstance(iterations, bool) or not isinstance(iterations, (int, np.integer)):
        raise TypeError(f"iterations must be an integer, got {iterations!r}")
    if int(iterations) < 1:
        raise ValueError(f"iterations must be >= 1, got {iterations!r}")

    present = set(np.unique(label_array).tolist())
    missing = [rid for rid in region_ids_int if rid not in present]
    if missing:
        raise ValueError(
            f"region ids {missing} are not present in labels (available ids: "
            f"{sorted(present)})"
        )

    cv2 = _require_cv2()
    kernel = np.ones((int(kernel_size), int(kernel_size)), dtype=np.uint8)

    dilated: list[np.ndarray] = []
    for rid in sorted(set(region_ids_int)):
        single = (label_array == rid).astype(np.uint8)
        if not np.any(single):  # pragma: no cover - 已由存在性检查保证
            continue
        dilated.append(cv2.dilate(single, kernel, iterations=int(iterations)))

    merged = label_array.copy()
    if len(dilated) >= 2:
        # 被两个或更多膨胀覆盖的单元是选中区域之间的狭窄通道；其他单元保留
        # 其原始标签。
        overlap = np.sum(np.stack(dilated, axis=0), axis=0) >= 2
        first_id = min(set(region_ids_int))
        merged[overlap] = first_id
    return np.ascontiguousarray(merged, dtype=np.int32)


def region_point_inside(labels: ArrayLike, region_id: int) -> tuple[int, int]:
    """返回一个保证位于 ``region_id`` 内部的像素。

    质心是自然之选，但对于 L 形或环形区域，它可能完全落在区域之外，从而悄悄产生
    错误的导航目标。因此本函数逐级升级：

    1. 取四舍五入后的质心，如果它恰好带有正确的标签；
    2. 围绕质心做扩张环搜索（每个半径 16 个方向），即使对超大区域也能廉价地找到
       附近的一个内部像素；
    3. 取距离质心最近的成员像素，按构造它一定位于区域内部。

    参数:
        labels: 区域 id 的二维 array-like。
        region_id: 要采样的区域；必须为正且存在。

    返回:
        一个满足 ``labels[py, px] == region_id`` 的 ``(px, py)`` =
        ``(列, 行)`` 像素。

    异常:
        TypeError: 若 ``labels`` 不是 array-like 或 ``region_id`` 不是整数。
        ValueError: 若 ``labels`` 不是二维、``region_id`` 不为正，或没有任何单元
            带有该 id。

    示例:
        >>> import numpy as np
        >>> labels = np.zeros((3, 3), np.int32)
        >>> labels[0, :] = 7        # 仅顶行 -> 第 0 行的质心
        >>> region_point_inside(labels, 7)
        (1, 0)
    """
    label_array = _as_2d_labels(labels)
    if isinstance(region_id, bool) or not isinstance(region_id, (int, np.integer)):
        raise TypeError(f"region_id must be an integer, got {region_id!r}")
    if int(region_id) <= 0:
        raise ValueError(f"region_id must be positive (0 means 'no region'), got {region_id!r}")

    ys, xs = np.nonzero(label_array == int(region_id))
    if xs.size == 0:
        raise ValueError(
            f"region {int(region_id)} has no cells in labels (available ids: "
            f"{sorted(set(np.unique(label_array).tolist()))})"
        )

    rows, cols = label_array.shape
    centroid_x = float(xs.mean())
    centroid_y = float(ys.mean())
    px, py = int(round(centroid_x)), int(round(centroid_y))
    if 0 <= px < cols and 0 <= py < rows and int(label_array[py, px]) == int(region_id):
        return px, py

    # 围绕质心的扩张环搜索。
    max_radius = int(max(rows, cols))
    for radius in range(1, max_radius + 1):
        for step in range(16):
            angle = 2.0 * math.pi * step / 16.0
            tx = int(round(centroid_x + radius * math.cos(angle)))
            ty = int(round(centroid_y + radius * math.sin(angle)))
            if 0 <= tx < cols and 0 <= ty < rows and int(label_array[ty, tx]) == int(region_id):
                return tx, ty

    closest = int(np.argmin((xs.astype(float) - centroid_x) ** 2 + (ys.astype(float) - centroid_y) ** 2))
    logger.debug(
        "region %d: ring search failed, falling back to the pixel closest to the centroid",
        int(region_id),
    )
    return int(xs[closest]), int(ys[closest])


def region_centroid_world(
    labels: ArrayLike,
    region_id: int,
    geometry: GridGeometry,
) -> tuple[float, float]:
    """返回区域的世界坐标质心。

    质心先在像素空间计算（平均列、平均行），再通过坐标系映射，因此结果是区域各
    单元的质心，而不是其包围盒的中心。对于凹形区域，它可能落在区域之外；需要可
    用的目标点时，请与 :func:`region_point_inside` 配合使用。

    参数:
        labels: 区域 id 的二维 array-like。
        region_id: 要测量的区域；必须为正且存在。
        geometry: ``labels`` 的 :class:`~reusable_model.gridmap.occupancy.GridGeometry`。

    返回:
        包含质心的那个单元中心的世界 ``(x, y)`` 坐标，单位米。

    异常:
        TypeError: 若 ``labels`` 不是 array-like、``region_id`` 不是整数，或
            ``geometry`` 不是 :class:`GridGeometry`。
        ValueError: 若 ``labels`` 不是二维、区域为空，或标签数组与坐标系尺寸不
            匹配。

    示例:
        >>> import numpy as np
        >>> geom = GridGeometry(0.0, 0.0, 1.0, 3, 3)
        >>> labels = np.zeros((3, 3), np.int32); labels[0, 0] = 1; labels[0, 2] = 1
        >>> region_centroid_world(labels, 1, geom)
        (1.5, 0.5)
    """
    label_array = _as_2d_labels(labels)
    if not isinstance(geometry, GridGeometry):
        raise TypeError(
            f"geometry must be a GridGeometry, got {type(geometry).__name__}: {geometry!r}"
        )
    if label_array.shape != (geometry.height, geometry.width):
        raise ValueError(
            f"labels shape {label_array.shape} does not match geometry (height, width) "
            f"{(geometry.height, geometry.width)}"
        )
    if isinstance(region_id, bool) or not isinstance(region_id, (int, np.integer)):
        raise TypeError(f"region_id must be an integer, got {region_id!r}")
    if int(region_id) <= 0:
        raise ValueError(f"region_id must be positive (0 means 'no region'), got {region_id!r}")

    ys, xs = np.nonzero(label_array == int(region_id))
    if xs.size == 0:
        raise ValueError(f"region {int(region_id)} has no cells in labels")
    # 各单元中心均值对应的世界坐标（与 OccupancyGrid 的 pixel_to_world
    # 约定一致：origin + (pixel + 0.5) * resolution）。
    cx = geometry.origin_x + (float(xs.mean()) + 0.5) * geometry.resolution
    cy = geometry.origin_y + (float(ys.mean()) + 0.5) * geometry.resolution
    return (cx, cy)


def _region_centroids(labels: np.ndarray) -> dict[int, tuple[float, float]]:
    """计算每个正区域的像素质心。

    参数:
        labels: ``(H, W)`` 的 ``int32`` 标签数组。

    返回:
        以像素为单位的映射 ``{region_id: (centroid_x, centroid_y)}``。
    """
    ids, counts = np.unique(labels[labels > 0], return_counts=True)
    centroids: dict[int, tuple[float, float]] = {}
    for region_id, count in zip(ids.tolist(), counts.tolist()):
        ys, xs = np.nonzero(labels == region_id)
        centroids[int(region_id)] = (float(xs.mean()), float(ys.mean()))
        del count
    return centroids


def connectivity_from_segments(
    labels: ArrayLike,
    segments: Sequence[Sequence[Sequence[float]]],
    *,
    normal_probe_px: int = 10,
    samples: int | None = None,
) -> dict[str, dict[int, Any]]:
    """从门洞式线段推导区域连通图。

    门、闸机与电梯门槛通常被标注为横跨开口的一条线段。线段本身不携带区域信息，
    但它两侧的区域携带：沿线段法线在两侧各探测若干像素，就能揭示这个开口连接的是
    哪一对区域。对每条已标注的线段重复这一过程，便得到全局规划器所需的拓扑图，
    而无需任何路径搜索。

    边权近似通过该开口的通行代价：
    ``distance(centroid_left, segment_midpoint) + distance(segment_midpoint,
    centroid_right)``，单位为像素。当多条线段连接同一对区域时，矩阵保留最便宜的
    那条，而 ``connections`` 会列出全部。

    参数:
        labels: 区域 id 的二维 array-like；``0`` 表示「无区域」，永远不会被用作
            图节点。
        segments: ``((x1, y1), (x2, y2))`` 像素坐标端点的序列。
        normal_probe_px: 沿线段法线在每一侧采集区域 id 时行走的距离，单位为像素。
            更大的值可容忍较厚的墙；过大的值会开始看到下一堵墙后面的区域。
        samples: 沿每条线段采样的点数。``None`` 使用
            ``max(int(length), 10)``，即大致每个像素一个采样，并以十为下限，这样
            较短的线段仍能被妥善探测。

    返回:
        含两个条目的 dict：

        * ``'matrix'``：``{region_i: {region_j: weight}}``，对称且不含自环。
        * ``'connections'``：``{region_i: [{'region': j, 'weight': w,
          'segment_index': k}, ...]}``，按权重排序，每个连接该对的线段一个条目。

    异常:
        TypeError: 若 ``labels`` 不是 array-like，或 ``segments`` 不是由两点线段
            组成的序列。
        ValueError: 若 ``labels`` 不是二维、某条线段不是恰好两个有限的二维端点、
            ``normal_probe_px`` 不是 ``>= 1``，或 ``samples`` 给出但不是 ``>= 1``。

    示例:
        >>> import numpy as np
        >>> labels = np.zeros((5, 9), np.int32)
        >>> labels[:, 0:4] = 1
        >>> labels[:, 5:9] = 2
        >>> door = ((4.0, 0.0), (4.0, 4.0))      # 画在 1 像素厚的墙中
        >>> graph = connectivity_from_segments(labels, [door], normal_probe_px=3)
        >>> sorted(graph["matrix"][1]), sorted(graph["matrix"][2])
        ([2], [1])
        >>> graph["connections"][1][0]["segment_index"]
        0
    """
    label_array = _as_2d_labels(labels)
    if isinstance(segments, (str, bytes)) or not isinstance(segments, Iterable):
        raise TypeError(
            f"segments must be a sequence of ((x1, y1), (x2, y2)) pairs, got {segments!r}"
        )
    if isinstance(normal_probe_px, bool) or not isinstance(normal_probe_px, (int, np.integer)):
        raise TypeError(f"normal_probe_px must be an integer, got {normal_probe_px!r}")
    if int(normal_probe_px) < 1:
        raise ValueError(f"normal_probe_px must be >= 1 pixel, got {normal_probe_px!r}")
    if samples is not None:
        if isinstance(samples, bool) or not isinstance(samples, (int, np.integer)):
            raise TypeError(f"samples must be an integer or None, got {samples!r}")
        if int(samples) < 1:
            raise ValueError(f"samples must be >= 1 or None, got {samples!r}")

    parsed_segments: list[tuple[float, float, float, float]] = []
    for index, segment in enumerate(segments):
        try:
            endpoints = np.asarray(segment, dtype=float)
        except (TypeError, ValueError) as exc:
            raise TypeError(
                f"segments[{index}] must be ((x1, y1), (x2, y2)) numbers, got {segment!r}"
            ) from exc
        if endpoints.shape != (2, 2) or not np.all(np.isfinite(endpoints)):
            raise ValueError(
                f"segments[{index}] must be two finite 2D endpoints, got {segment!r}"
            )
        (x1, y1), (x2, y2) = endpoints
        parsed_segments.append((float(x1), float(y1), float(x2), float(y2)))

    rows, cols = label_array.shape
    centroids = _region_centroids(label_array)
    matrix: dict[int, dict[int, float]] = {}
    connections: dict[int, list[dict[str, Any]]] = {}

    for index, (x1, y1, x2, y2) in enumerate(parsed_segments):
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length < 1.0:
            logger.warning(
                "segment %d is shorter than 1 pixel (%.3f); its normal is undefined, skipping",
                index,
                length,
            )
            continue
        unit_x, unit_y = dx / length, dy / length
        normal_x, normal_y = -unit_y, unit_x
        num_points = int(samples) if samples is not None else max(int(length), 10)

        sides: tuple[set[int], set[int]] = (set(), set())
        for side_index, direction in enumerate((1.0, -1.0)):
            found = sides[side_index]
            for step in range(num_points + 1):
                t = step / num_points
                base_x = x1 + t * dx
                base_y = y1 + t * dy
                for offset in range(1, int(normal_probe_px) + 1):
                    px = int(round(base_x + direction * normal_x * offset))
                    py = int(round(base_y + direction * normal_y * offset))
                    if 0 <= px < cols and 0 <= py < rows:
                        region = int(label_array[py, px])
                        if region > 0:
                            found.add(region)

        left, right = sides
        mid_x, mid_y = (x1 + x2) * 0.5, (y1 + y2) * 0.5
        for left_id in sorted(left):
            for right_id in sorted(right):
                if left_id == right_id:
                    continue
                left_centroid = centroids[left_id]
                right_centroid = centroids[right_id]
                weight = math.hypot(left_centroid[0] - mid_x, left_centroid[1] - mid_y)
                weight += math.hypot(mid_x - right_centroid[0], mid_y - right_centroid[1])
                if weight <= 0.0:
                    weight = 1.0
                pair_matrix = matrix.setdefault(left_id, {})
                existing = pair_matrix.get(right_id)
                if existing is None or weight < existing:
                    pair_matrix[right_id] = weight
                reverse = matrix.setdefault(right_id, {})
                reverse_existing = reverse.get(left_id)
                if reverse_existing is None or weight < reverse_existing:
                    reverse[left_id] = weight

                for node, other in ((left_id, right_id), (right_id, left_id)):
                    connections.setdefault(node, []).append(
                        {"region": other, "weight": weight, "segment_index": index}
                    )

    for entries in connections.values():
        entries.sort(key=lambda item: (item["weight"], item["segment_index"]))
    return {"matrix": matrix, "connections": connections}


def draw_lines(
    image: ArrayLike,
    segments: Sequence[Sequence[Sequence[float]]],
    color: Sequence[int],
    thickness: int = 2,
) -> np.ndarray:
    """把线段绘制到图像副本上。

    用于可视化送入 :func:`connectivity_from_segments` 的门/电梯线段。输入图像
    *不会*被修改；返回的是一个副本，因为悄悄修改调用方的数组是调试痛苦的经典
    来源。

    参数:
        image: 要绘制的二维或三维 array-like（灰度或 BGR）。
        segments: ``((x1, y1), (x2, y2))`` 像素端点的序列。
        color: 颜色，用整数序列表示，例如 BGR 中的 ``(0, 0, 255)`` 表示红色。
            灰度图像请使用单元素序列。
        thickness: 线宽，单位为像素，至少为 1。

    返回:
        ``image`` 的副本（dtype 相同），其上绘制了线段。

    异常:
        TypeError: 若 ``image`` 不是 array-like 或 ``color`` 不是整数序列。
        ValueError: 若 ``segments``/``color``/``thickness`` 格式错误，或颜色通道
            数与图像不匹配。
        RuntimeError: 若未安装 OpenCV。

    示例:
        >>> import numpy as np
        >>> canvas = np.zeros((5, 5), np.uint8)
        >>> out = draw_lines(canvas, [((0, 0), (4, 4))], (255,), thickness=1)
        >>> int(out[2, 2]), int(canvas[2, 2])   # 仅画在副本上
        (255, 0)
    """
    try:
        base = np.asarray(image)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"image must be array-like, got {image!r}") from exc
    if base.ndim not in (2, 3):
        raise ValueError(f"image must be 2D or 3D (H, W[, channels]), got shape {base.shape}")
    if isinstance(segments, (str, bytes)) or not isinstance(segments, Iterable):
        raise TypeError(
            f"segments must be a sequence of ((x1, y1), (x2, y2)) pairs, got {segments!r}"
        )
    if isinstance(color, (str, bytes)) or not isinstance(color, Iterable):
        raise TypeError(f"color must be a sequence of integers, got {color!r}")
    color_values = [int(c) for c in color]
    if not color_values:
        raise ValueError("color must contain at least one channel value, got an empty sequence")
    expected_channels = 1 if base.ndim == 2 else base.shape[2]
    if len(color_values) != expected_channels:
        raise ValueError(
            f"color has {len(color_values)} channel(s) but the image has "
            f"{expected_channels}; got {color_values!r}"
        )
    if isinstance(thickness, bool) or not isinstance(thickness, (int, np.integer)):
        raise TypeError(f"thickness must be an integer, got {thickness!r}")
    if int(thickness) < 1:
        raise ValueError(f"thickness must be >= 1 pixel, got {thickness!r}")

    cv2 = _require_cv2()
    canvas = base.copy()
    for index, segment in enumerate(segments):
        try:
            endpoints = np.asarray(segment, dtype=float)
        except (TypeError, ValueError) as exc:
            raise TypeError(
                f"segments[{index}] must be ((x1, y1), (x2, y2)) numbers, got {segment!r}"
            ) from exc
        if endpoints.shape != (2, 2) or not np.all(np.isfinite(endpoints)):
            raise ValueError(
                f"segments[{index}] must be two finite 2D endpoints, got {segment!r}"
            )
        (x1, y1), (x2, y2) = endpoints
        cv2.line(
            canvas,
            (int(round(x1)), int(round(y1))),
            (int(round(x2)), int(round(y2))),
            tuple(color_values),
            int(thickness),
        )
    return canvas
