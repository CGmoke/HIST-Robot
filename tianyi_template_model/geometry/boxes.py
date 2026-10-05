"""轴对齐的 2D 边界框几何。

一组用于像素坐标矩形的小型纯函数工具箱。它们是目标检测后处理、视觉跟踪与掩码
工具的底层支撑，并且刻意不依赖任何特定的检测器或跟踪器：

* 交集 / 并集统计量（:func:`iou`、:func:`iou_min`、:func:`overlap_score`）；
* 边界框与布尔掩码之间的相互转换（:func:`bbox_to_mask`、:func:`clip_bbox`）；
* 计算边界框与掩码的质心（:func:`bbox_center`、:func:`mask_center`）。

边界框始终是 ``(x1, y1, x2, y2)`` 元组，左上角包含、右下角不包含，这正是
OpenCV、YOLO 系列检测器以及大多数跟踪基准所采用的约定。第四个元素之后的多余
元素（例如置信度值）在所有函数中都会被忽略。

依赖：仅 :mod:`numpy`。
"""

from __future__ import annotations

import logging
import math
from typing import Iterable, Sequence, Tuple

import numpy as np
from numpy.typing import ArrayLike

logger = logging.getLogger(__name__)

__all__ = [
    "bbox_area",
    "bbox_center",
    "bbox_to_mask",
    "clip_bbox",
    "iou",
    "iou_min",
    "mask_center",
    "overlap_score",
]

Box = Tuple[float, float, float, float]


def _as_box(value: Sequence[float], name: str = "bbox") -> tuple[float, float, float, float]:
    """将 ``value`` 转换为四个有限浮点数。

    只读取前四个元素，其后的每个元素都会被忽略，因此调用方可以传入带有额外
    字段的检测结果。

    参数:
        value: 至少包含四个数值元素的序列。
        name: 用于错误消息中的参数名。

    返回:
        由浮点数构成的 ``(x1, y1, x2, y2)`` 元组。

    异常:
        TypeError: 如果 ``value`` 不是数值序列。
        ValueError: 如果元素少于四个，或者这四个中的任意一个不是有限值。

    示例:
        >>> _as_box([0, 0, 10, 20, 0.9])
        (0.0, 0.0, 10.0, 20.0)
    """
    try:
        coords = tuple(float(v) for v in value[:4])
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be a sequence of numbers, got {value!r}") from exc
    if len(coords) < 4:
        raise ValueError(f"{name} must have at least 4 elements, got {len(value)}: {value!r}")
    if not all(math.isfinite(v) for v in coords):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return coords  # type: ignore[return-value]


def bbox_area(bbox: Sequence[float]) -> float:
    """``bbox`` 的面积，恒为非负。

    参数:
        bbox: ``(x1, y1, x2, y2)`` 坐标。

    返回:
        ``(x2 - x1) * (y2 - y1)`` 并裁剪到零，因此退化框会报告 ``0.0``
        而不是负数。

    示例:
        >>> bbox_area([0, 0, 4, 5])
        20.0
        >>> bbox_area([4, 5, 0, 0])
        0.0
    """
    x1, y1, x2, y2 = _as_box(bbox)
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def _intersection(a: Sequence[float], b: Sequence[float]) -> float:
    """两个边界框交集的面积。"""
    ax1, ay1, ax2, ay2 = _as_box(a, "a")
    bx1, by1, bx2, by2 = _as_box(b, "b")
    iw = min(ax2, bx2) - max(ax1, bx1)
    ih = min(ay2, by2) - max(ay1, by1)
    if iw <= 0.0 or ih <= 0.0:
        return 0.0
    return iw * ih


def iou(a: Sequence[float], b: Sequence[float]) -> float:
    """两个边界框的交并比（Intersection over Union）。

    参数:
        a: 第一个边界框 ``(x1, y1, x2, y2)``。
        b: 第二个边界框 ``(x1, y1, x2, y2)``。

    返回:
        ``[0.0, 1.0]`` 范围内的 ``交集 / 并集``；当两个边界框不重叠时
        为 ``0.0``。

    示例:
        >>> round(iou([0, 0, 10, 10], [5, 5, 15, 15]), 3)
        0.143
        >>> iou([0, 0, 2, 2], [10, 10, 12, 12])
        0.0
    """
    inter = _intersection(a, b)
    if inter <= 0.0:
        return 0.0
    union = bbox_area(a) + bbox_area(b) - inter
    return inter / union if union > 0.0 else 0.0


def iou_min(a: Sequence[float], b: Sequence[float]) -> float:
    """交集除以较小边界框的面积。

    与 IoU 不同，当一个边界框完全包含另一个时该值可达 ``1.0``，这使它成为
    "目标是否漂移"检查的自然度量：一个小框在一个大的静止框内部移动时仍会
    得到很高的分数。

    参数:
        a: 第一个边界框 ``(x1, y1, x2, y2)``。
        b: 第二个边界框 ``(x1, y1, x2, y2)``。

    返回:
        ``[0.0, 1.0]`` 范围内的 ``交集 / min(area_a, area_b)``。

    示例:
        >>> iou_min([0, 0, 20, 20], [5, 5, 15, 15])
        1.0
        >>> round(iou_min([0, 0, 10, 10], [5, 5, 15, 15]), 3)
        0.25
    """
    inter = _intersection(a, b)
    if inter <= 0.0:
        return 0.0
    smaller = min(bbox_area(a), bbox_area(b))
    return inter / smaller if smaller > 0.0 else 0.0


def overlap_score(a: Sequence[float], b: Sequence[float]) -> float:
    """IoU 与 IoMin 的最大值 —— 跟踪所使用的重叠度量。

    一个框可以完全位于另一个框内部，而 IoU 却出奇地低（例如一个小框位于大框
    内部时 IoU 为 0.2）。与 IoMin 取最大值可以保留这类匹配：嵌套的框显然是
    *同一个目标*，即便并集很大。

    参数:
        a: 第一个边界框 ``(x1, y1, x2, y2)``。
        b: 第二个边界框 ``(x1, y1, x2, y2)``。

    返回:
        ``[0.0, 1.0]`` 范围内的 ``max(iou(a, b), iou_min(a, b))``。

    示例:
        >>> round(overlap_score([0, 0, 10, 10], [0, 0, 100, 100]), 3)
        1.0
    """
    return max(iou(a, b), iou_min(a, b))


def clip_bbox(bbox: Sequence[float], width: int, height: int) -> tuple[int, int, int, int]:
    """将像素框裁剪到图像范围内并取整为整数坐标。

    参数:
        bbox: ``(x1, y1, x2, y2)`` 像素坐标。
        width: 图像宽度（像素）；必须非负。
        height: 图像高度（像素）；必须非负。

    返回:
        位于 ``[0, width]`` / ``[0, height]`` 内的整数 ``(x1, y1, x2, y2)``。
        完全位于图像之外的框会塌缩为零面积框。

    异常:
        ValueError: 如果 ``width`` 或 ``height`` 为负或不是整数。

    示例:
        >>> clip_bbox([-5, 0, 30, 10], width=20, height=20)
        (0, 0, 20, 10)
    """
    if not isinstance(width, int) or not isinstance(height, int):
        raise TypeError(f"width/height must be int, got {width!r}/{height!r}")
    if width < 0 or height < 0:
        raise ValueError(f"width/height must be non-negative, got {width}x{height}")
    x1, y1, x2, y2 = _as_box(bbox)
    cx1 = min(width, max(0, int(round(x1))))
    cy1 = min(height, max(0, int(round(y1))))
    cx2 = min(width, max(0, int(round(x2))))
    cy2 = min(height, max(0, int(round(y2))))
    return cx1, cy1, cx2, cy2


def bbox_to_mask(bbox: Sequence[float], width: int, height: int) -> np.ndarray:
    """在 ``bbox`` 内部将布尔掩码填充为 ``True``。

    参数:
        bbox: ``(x1, y1, x2, y2)`` 像素坐标。
        width: 图像宽度（像素）。
        height: 图像高度（像素）。

    返回:
        形状为 ``(height, width)`` 的 ``bool`` 数组，裁剪后的框内部为
        ``True``。退化框或位于图像之外的框会返回全 ``False`` 掩码而
        不抛出异常。

    示例:
        >>> m = bbox_to_mask([1, 1, 3, 3], width=4, height=4)
        >>> m.sum()
        4
    """
    x1, y1, x2, y2 = clip_bbox(bbox, width, height)
    mask = np.zeros((height, width), dtype=bool)
    if x2 > x1 and y2 > y1:
        mask[y1:y2, x1:x2] = True
    return mask


def bbox_center(bbox: Sequence[float]) -> tuple[int, int]:
    """边界框的质心，四舍五入为整数像素。

    参数:
        bbox: ``(x1, y1, x2, y2)`` 坐标。

    返回:
        ``(round((x1+x2)/2), round((y1+y2)/2))``。

    示例:
        >>> bbox_center([0, 0, 4, 2])
        (2, 1)
    """
    x1, y1, x2, y2 = _as_box(bbox)
    return (int(round((x1 + x2) / 2.0)), int(round((y1 + y2) / 2.0)))


def mask_center(mask: ArrayLike) -> tuple[int, int] | None:
    """2D 布尔掩码中 ``True`` 像素的质心。

    参数:
        mask: 2D 数组（或可转换为 2D 数组的对象）。多余的维度会先被压缩；
            空掩码没有质心。

    返回:
        非零像素的 ``(round(x_mean), round(y_mean))``，当没有 ``True`` 像素
        或掩码无法被解释为 2D 时返回 ``None``。

    示例:
        >>> mask_center([[0, 1], [1, 0]])
        (0, 0)
        >>> mask_center([[0, 0], [0, 0]]) is None
        True
    """
    try:
        arr = np.asarray(mask)
    except Exception:  # noqa: BLE001 -- 非数组输入降级为 None
        return None
    if arr.ndim > 2:
        arr = np.squeeze(arr)
    if arr.ndim != 2 or arr.size == 0:
        return None
    ys, xs = np.nonzero(arr)
    if xs.size == 0:
        return None
    return (int(round(float(xs.mean()))), int(round(float(ys.mean()))))
