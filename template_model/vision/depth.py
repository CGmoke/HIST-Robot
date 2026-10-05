"""深度图采样、渲染与区域辅助函数。

真实传感器输出的深度缓冲区噪声大、空洞多、单位也不明确（多数驱动是 16 位毫米，
另一些则是 ``float32`` 米）。本模块汇集了一小批操作，用于把这样的缓冲区转换为
机器人可用于决策的数值：

* 针对单个像素或包围框的*鲁棒*深度值
  （:func:`sample_depth`、:func:`bbox_depth`），
* 有效掩码与单位换算（:func:`valid_mask`、
  :func:`depth_to_meters`、:func:`infer_depth_scale`），
* 用于日志与调试的伪彩色渲染（:func:`render_depth`），
* 用于探测“物体正下方的地面”的几何区域辅助函数
  （:func:`relative_box`）以及在框内度量颜色证据的函数
  （:func:`color_ratio_in_bbox`）。

核心设计决策是：**每个采样器都按度量深度范围过滤，并报告“无测量值”而不是一个
可疑的数值**。返回 ``None`` 会迫使调用方去决定缺失测量意味着什么，这比一个稍后
会变成相机原点处三维点的静默 ``0.0`` 要安全得多。

模块级只导入 ``numpy``；``cv2`` 在需要它的两个函数
（:func:`render_depth`、:func:`color_ratio_in_bbox`）内部惰性导入，
这样在没有 OpenCV 的无头环境中采样数学仍然可用。
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any, Iterable, Sequence

import numpy as np

logger = logging.getLogger(__name__)

__all__ = [
    "HSV_GRASS_GREEN",
    "DepthLimits",
    "infer_depth_scale",
    "valid_mask",
    "depth_to_meters",
    "sample_depth",
    "bbox_depth",
    "render_depth",
    "relative_box",
    "color_ratio_in_bbox",
]

#: 修剪过的草坪绿色的 HSV 边界，``(lower, upper)`` 采用 OpenCV 的 8 位 HSV
#: 编码（``H`` 在 ``[0, 179]``，``S``/``V`` 在 ``[0, 255]``）。
#:
#: 较宽的色相跨度（35..95）覆盖从黄绿到青绿，使阴影中的草与阳光下的草都能匹配；
#: 饱和度/明度下限（40/40）用于排除灰色路面和高光，这正是让该比值可作为
#: “这个框是否立在草坪上？”证据的原因。由于绿色位于色相环中部，无论源数组是
#: BGR 还是 RGB，该范围都相同——只有红色/蓝色检测才依赖通道顺序。
HSV_GRASS_GREEN: tuple[tuple[int, int, int], tuple[int, int, int]] = (
    (35, 40, 40),
    (95, 255, 255),
)

#: :func:`sample_depth` 与 :func:`bbox_depth` 接受的统计量名称。
STATISTICS: tuple[str, ...] = ("median", "mean", "percentile")


def _require_cv2() -> Any:
    """惰性导入 OpenCV，并给出可操作的错误信息。

    返回：
        ``cv2`` 模块。

    异常：
        ImportError: 若未安装 OpenCV。
    """
    try:
        import cv2  # noqa: PLC0415 - 有意惰性导入，见模块 docstring
    except ImportError as exc:  # pragma: no cover - 取决于运行环境
        raise ImportError(
            "this function needs OpenCV; install it with `pip install opencv-python` "
            "(headless servers: `pip install opencv-python-headless`)"
        ) from exc
    return cv2


def _finite_float(value: Any, name: str) -> float:
    """将 ``value`` 转换为有限的 ``float``，否则携带该值抛出异常。

    参数：
        value: 候选数值。
        name: 用于错误信息的参数名。

    返回：
        该值转换后的有限 ``float``。

    异常：
        TypeError: 若 ``value`` 不是实数（``bool`` 被拒绝，因 ``True`` 会静默地
            表现为 ``1.0``）。
        ValueError: 若 ``value`` 为 NaN 或无穷大。
    """
    if isinstance(value, bool) or not isinstance(value, (int, float, np.floating, np.integer)):
        raise TypeError(f"{name} must be a real number, got {value!r} ({type(value).__name__})")
    out = float(value)
    if not math.isfinite(out):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return out


def _positive_float(value: Any, name: str) -> float:
    """将 ``value`` 转换为严格为正的有限 ``float``，否则抛出异常。

    参数：
        value: 候选数值。
        name: 用于错误信息的参数名。

    返回：
        该值转换后的正 ``float``。

    异常：
        TypeError: 若 ``value`` 不是实数。
        ValueError: 若其为 NaN、无穷大或 ``<= 0``。
    """
    out = _finite_float(value, name)
    if out <= 0.0:
        raise ValueError(f"{name} must be > 0, got {value!r}")
    return out


def _non_negative_float(value: Any, name: str) -> float:
    """将 ``value`` 转换为有限的 ``float >= 0``，否则抛出异常。

    参数：
        value: 候选数值。
        name: 用于错误信息的参数名。

    返回：
        该值转换后的非负 ``float``。

    异常：
        TypeError: 若 ``value`` 不是实数。
        ValueError: 若其为 NaN、无穷大或负数。
    """
    out = _finite_float(value, name)
    if out < 0.0:
        raise ValueError(f"{name} must be >= 0, got {value!r}")
    return out


def _count_int(value: Any, name: str, *, minimum: int = 1) -> int:
    """将计数类参数（窗口大小、像素数）转换为 ``int``。

    参数：
        value: 候选计数值。
        name: 用于错误信息的参数名。
        minimum: 可接受的最小值。

    返回：
        该值转换后的 Python ``int``。

    异常：
        TypeError: 若 ``value`` 不是整数（``bool`` 被拒绝）。
        ValueError: 若其小于 ``minimum``。
    """
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise TypeError(
            f"{name} must be an integer >= {minimum}, got {value!r} ({type(value).__name__})"
        )
    out = int(value)
    if out < minimum:
        raise ValueError(f"{name} must be >= {minimum}, got {value!r}")
    return out


def _as_depth(depth: Any, name: str = "depth") -> np.ndarray:
    """校验深度缓冲区，并以二维 ``float64`` 数组返回。

    在这里一次性转换为 ``float64``，使采样器除以深度缩放时不会让 ``uint16``
    累加器溢出，也使 ``NaN``/``inf`` 这类哨兵值可以被表示。

    参数：
        depth: 深度图；任意二维数组类对象，元素为原始深度值。
        name: 用于错误信息的参数名。

    返回：
        ``(H, W)`` 的 ``float64`` 数组。

    异常：
        TypeError: 若 ``depth`` 不是数组类对象。
        ValueError: 若 ``depth`` 不是二维，或为空。
    """
    try:
        arr = np.asarray(depth, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"{name} must be an array-like depth image, got {type(depth).__name__}: {exc}"
        ) from exc
    if arr.ndim != 2:
        raise ValueError(
            f"{name} must be a 2-D image of shape (H, W), got shape {arr.shape}"
        )
    if arr.size == 0:
        raise ValueError(f"{name} must not be empty, got shape {arr.shape}")
    return arr


def _check_limits(min_depth: Any, max_depth: Any) -> tuple[float, float]:
    """校验一个度量深度范围。

    参数：
        min_depth: 最小合理距离，单位为米（允许 ``0``，仅表示“不做近端裁剪”）。
        max_depth: 最大合理距离，单位为米。

    返回：
        以浮点数表示的 ``(min_depth, max_depth)``。

    异常：
        TypeError: 若某个边界不是实数。
        ValueError: 若某个边界为负数/非有限，或 ``max_depth`` 未超过
            ``min_depth``。
    """
    lo = _non_negative_float(min_depth, "min_depth")
    hi = _positive_float(max_depth, "max_depth")
    if hi <= lo:
        raise ValueError(
            f"max_depth must be > min_depth, got min_depth={min_depth!r} and "
            f"max_depth={max_depth!r}"
        )
    return lo, hi


def _bbox_rect(bbox: Any, height: int, width: int, *, name: str = "bbox") -> tuple[int, int, int, int]:
    """将包围框转换为经钳制的整数矩形。

    左上角使用 ``floor``、右下角使用 ``ceil``，因此带小数的框总能覆盖它触及的
    每个像素——若两个角都取整，会静默地缩小小框（一个 2.5 像素的框可能坍缩为
    空）。

    参数：
        bbox: 以像素为单位的 ``(x1, y1, x2, y2)``；末尾的额外元素（某些检测
            格式会追加一个分数）会被忽略。
        height: 用于钳制的图像高度。
        width: 用于钳制的图像宽度。
        name: 用于错误信息的参数名。

    返回：
        图像内的 ``(x0, y0, x1, y1)``，末端为开区间。当输入完全位于图像之外时，
        该框可能为空（``x1 <= x0`` 或 ``y1 <= y0``）；调用方必须处理这种情况。

    异常：
        TypeError: 若 ``bbox`` 不是数值序列。
        ValueError: 若其元素少于 4 个，或含非有限值。
    """
    if isinstance(bbox, (str, bytes)):
        raise TypeError(f"{name} must be a sequence (x1, y1, x2, y2), got {bbox!r}")
    try:
        values = [float(v) for v in bbox]
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"{name} must be a sequence of numbers (x1, y1, x2, y2), got {bbox!r}"
        ) from exc
    if len(values) < 4:
        raise ValueError(
            f"{name} must hold 4 numbers (x1, y1, x2, y2), got {len(values)}: {bbox!r}"
        )
    if not all(math.isfinite(v) for v in values[:4]):
        raise ValueError(f"{name} must hold finite numbers, got {bbox!r}")

    x1, y1, x2, y2 = values[:4]
    x0 = min(max(int(math.floor(x1)), 0), width)
    y0 = min(max(int(math.floor(y1)), 0), height)
    xe = min(max(int(math.ceil(x2)), 0), width)
    ye = min(max(int(math.ceil(y2)), 0), height)
    return x0, y0, max(xe, x0), max(ye, y0)


def _reduce(values: np.ndarray, statistic: str, percentile: float) -> float:
    """将有效深度值（单位为米）归约为单个代表值。

    参数：
        values: 非空的一维数组，元素为以米为单位的有效深度。
        statistic: :data:`STATISTICS` 之一（已转为小写并校验）。
        percentile: 当 ``statistic == 'percentile'`` 时使用的百分位数。

    返回：
        归约后的以米为单位的深度。
    """
    if statistic == "median":
        return float(np.median(values))
    if statistic == "mean":
        return float(np.mean(values))
    return float(np.percentile(values, percentile))


def _normalise_statistic(statistic: Any, percentile: Any) -> tuple[str, float]:
    """校验统计量名称及其百分位参数。

    参数：
        statistic: 请求的归约方式；大小写不敏感。
        percentile: ``[0, 100]`` 内的百分位数，仅在 ``'percentile'`` 时使用。

    返回：
        ``(statistic_lower_case, percentile)``。

    异常：
        TypeError: 若 ``statistic`` 不是字符串，或 ``percentile`` 不是实数。
        ValueError: 若统计量未知，或百分位数超出范围。
    """
    if not isinstance(statistic, str):
        raise TypeError(
            f"statistic must be a string, one of {STATISTICS}, got {statistic!r} "
            f"({type(statistic).__name__})"
        )
    name = statistic.strip().lower()
    if name not in STATISTICS:
        raise ValueError(
            f"statistic must be one of {STATISTICS}, got {statistic!r}"
        )
    pct = _non_negative_float(percentile, "percentile")
    if pct > 100.0:
        raise ValueError(f"percentile must be within [0, 100], got {percentile!r}")
    return name, pct


@dataclass(frozen=True)
class DepthLimits:
    """用于剔除坏测量的合理度量深度范围。

    真实深度传感器对“无回波”会报告 ``0``，并且偶尔会在反光或极暗表面上发出荒谬的
    值。裁剪到一个物理合理范围，可以在两类失效模式进入三维计算之前就将其消除。
    默认值（5 厘米 .. 20 米）适合室内/室外移动机器人：近于 5 厘米已在传感器自身
    最近量程之内，远于 20 米时结构光或双目深度相机产生的本就只是噪声。

    属性：
        min_depth: 可接受的最小距离，单位为米。
        max_depth: 可接受的最大距离，单位为米。

    异常：
        TypeError: 若某个边界不是实数。
        ValueError: 若某个边界为负数/非有限，或 ``max_depth <= min_depth``。

    示例：
        >>> DepthLimits()
        DepthLimits(min_depth=0.05, max_depth=20.0)
        >>> DepthLimits(0.2, 5.0).max_depth
        5.0
        >>> DepthLimits(5.0, 1.0)
        Traceback (most recent call last):
            ...
        ValueError: max_depth must be > min_depth, got min_depth=5.0 and max_depth=1.0
    """

    min_depth: float = 0.05
    max_depth: float = 20.0

    def __post_init__(self) -> None:
        """校验范围；各值以普通浮点数存储。"""
        lo, hi = _check_limits(self.min_depth, self.max_depth)
        object.__setattr__(self, "min_depth", lo)
        object.__setattr__(self, "max_depth", hi)


def infer_depth_scale(depth: Any) -> float:
    """推测将原始深度值换算为米的除数。

    启发式规则就是每个 RGB-D 调试工具最终都会写的那种：若缓冲区最大值超过 20，
    它不可能以米为单位（没有手持深度相机能测到 20 米），因此必然是毫米。其他情况
    一律假定已经是米。

    这是*针对无标注数据的启发式规则*，不能替代驱动文档给出的缩放：一幅近距离
    场景的毫米深度图，其最大值会低于 20，从而被误读为米。当传感器会告知单位时，
    请始终显式传入 ``depth_scale``（绝大多数 16 位 ``uint16`` 流都是毫米，即
    ``1000.0``）。

    参数：
        depth: 原始深度值构成的深度图。

    返回：
        当数据看起来是毫米时返回 ``1000.0``，否则返回 ``1.0``。

    异常：
        TypeError: 若 ``depth`` 不是数组类对象。
        ValueError: 若 ``depth`` 不是二维，或为空。

    示例：
        >>> infer_depth_scale(np.full((4, 4), 1500, dtype=np.uint16))
        1000.0
        >>> infer_depth_scale(np.full((4, 4), 1.5, dtype=np.float32))
        1.0
    """
    arr = _as_depth(depth)
    finite = arr[np.isfinite(arr)]
    if finite.size == 0:
        logger.warning("depth image holds no finite value; assuming metres (scale 1.0)")
        return 1.0
    peak = float(np.max(finite))
    return 1000.0 if peak > 20.0 else 1.0


def valid_mask(
    depth: Any,
    *,
    depth_scale: float = 1000.0,
    min_depth: float = 0.05,
    max_depth: float = 20.0,
) -> np.ndarray:
    """返回标出具有合理测量值的像素的布尔掩码。

    当像素的原始值有限、且其度量深度落在 ``[min_depth, max_depth]`` 内时，该像素
    有效。由于 ``min_depth`` 默认是正数，“无回波”哨兵值 ``0`` 会被同一测试拒绝
    ——无需特殊处理。

    参数：
        depth: 原始深度值构成的深度图。
        depth_scale: 每米对应的原始深度单位数（``metres = raw / depth_scale``）。
        min_depth: 可接受的最小距离，单位为米。
        max_depth: 可接受的最大距离，单位为米。

    返回：
        ``(H, W)`` 布尔数组，深度可用处为 ``True``。

    异常：
        TypeError: 若 ``depth`` 不是数组类对象，或某个缩放不是数值。
        ValueError: 若深度图不是二维/为空，或范围无效。

    示例：
        >>> d = np.zeros((2, 3), dtype=np.uint16)
        >>> d[0, 0] = 1500
        >>> valid_mask(d).tolist()
        [[True, False, False], [False, False, False]]
    """
    arr = _as_depth(depth)
    scale = _positive_float(depth_scale, "depth_scale")
    lo, hi = _check_limits(min_depth, max_depth)
    metres = arr / scale
    return np.isfinite(metres) & (metres >= lo) & (metres <= hi)


def depth_to_meters(depth: Any, depth_scale: float = 1000.0) -> np.ndarray:
    """将原始深度图换算为米。

    输出为 ``float32``：它把 ``float64`` 的内存流量减半，同时保留约 7 位有效数字，
    远超深度传感器所能提供的精度。非有限的原始值变为 ``NaN``，“无回波”哨兵 ``0``
    仍保持 ``0.0``，因此 :func:`valid_mask` 在该结果上依然可用。

    参数：
        depth: 原始深度值构成的深度图。
        depth_scale: 每米对应的原始深度单位数（``metres = raw / depth_scale``）。

    返回：
        形状相同、以米为单位的 ``float32`` 数组。

    异常：
        TypeError: 若 ``depth`` 不是数组类对象，或缩放不是数值。
        ValueError: 若深度图不是二维/为空，或缩放 ``<= 0``。

    示例：
        >>> float(depth_to_meters(np.array([[1500, 0]], dtype=np.uint16))[0, 0])
        1.5
        >>> depth_to_meters(np.array([[1500]], dtype=np.uint16)).dtype
        dtype('float32')
    """
    arr = _as_depth(depth)
    scale = _positive_float(depth_scale, "depth_scale")
    return (arr / scale).astype(np.float32)


def sample_depth(
    depth: Any,
    u: float,
    v: float,
    *,
    depth_scale: float = 1000.0,
    window: int = 11,
    statistic: str = "median",
    min_depth: float = 0.05,
    max_depth: float = 20.0,
    min_valid_pixels: int = 1,
    percentile: float = 50.0,
) -> float | None:
    """读取像素 ``(u, v)`` 附近的鲁棒度量深度。

    单个深度像素几乎从不可信：传感器会在无纹理和反光表面上散布零值空洞，物体边缘
    还会产生位于物体与背景之间的“飞点”。对一个小邻域求平均是标准做法，而归约方式
    的选择是在两种相互竞争的风险之间取权衡：

    * ``'median'``（默认）能忽略窗口中最多一半的样本，因此少量空洞或一个背景
      离群点都无法改变结果。当像素位于物体边缘时，它是合适的默认选择。
    * ``'mean'`` 使用每个有效样本，在干净、平坦的表面上更平滑、略更准确，但一个
      幸存下来的离群点就会拖动结果——仅建议用于大范围、同质的区域。
    * ``'percentile'`` 在两者之间插值：较低的百分位（例如 25）会偏向*最近*的表面，
      当你采样可能混入背景像素的物体顶部时，这正是你想要的。

    窗口以 ``(u, v)`` 为中心，半宽为 ``window // 2``，并会被裁剪到图像内，因此靠近
    边界的像素只会使用更小的图块，而不是失败。

    参数：
        depth: 原始深度值构成的深度图。
        u: 像素列坐标（小数值会四舍五入到最近的像素）。
        v: 像素行坐标（小数值会四舍五入到最近的像素）。
        depth_scale: 每米对应的原始深度单位数（``metres = raw / depth_scale``）。
        window: 方形邻域的整边长，单位为像素；``1`` 表示只采样单个像素。
        statistic: 要应用的归约方式，:data:`STATISTICS` 之一。
        min_depth: 可接受的最小距离，单位为米。
        max_depth: 可接受的最大距离，单位为米。
        min_valid_pixels: 在返回值之前所需的、落在范围内的最少像素数。
        percentile: ``[0, 100]`` 内的百分位数，供 ``statistic='percentile'`` 使用。

    返回：
        以米为单位的深度；当窗口中有效像素少于 ``min_valid_pixels`` 时返回
        ``None``——用 ``None``（而非 ``0.0``）可确保缺失测量绝不会被误认为真实
        距离。

    异常：
        TypeError: 若某参数为非数值类型。
        ValueError: 若 ``(u, v)`` 位于图像之外、``window`` 或 ``min_valid_pixels``
            小于 1、深度范围无效，或 ``statistic``/``percentile`` 未知/超范围。

    示例：
        >>> d = np.zeros((5, 5), dtype=np.uint16)
        >>> d[1:4, 1:4] = 2000
        >>> sample_depth(d, 2, 2, window=3)
        2.0
        >>> sample_depth(d, 0, 0, window=1) is None
        True
        >>> sample_depth(d, 2, 2, window=3, min_valid_pixels=100) is None
        True
    """
    arr = _as_depth(depth)
    scale = _positive_float(depth_scale, "depth_scale")
    lo, hi = _check_limits(min_depth, max_depth)
    stat, pct = _normalise_statistic(statistic, percentile)
    window_px = _count_int(window, "window")
    required = _count_int(min_valid_pixels, "min_valid_pixels")

    height, width = arr.shape
    uu = _finite_float(u, "u")
    vv = _finite_float(v, "v")
    if not 0.0 <= uu <= width - 1.0:
        raise ValueError(f"u must lie inside the depth image of width {width}, got {u!r}")
    if not 0.0 <= vv <= height - 1.0:
        raise ValueError(f"v must lie inside the depth image of height {height}, got {v!r}")
    ui = int(round(uu))
    vi = int(round(vv))

    half = window_px // 2
    y0, y1 = max(0, vi - half), min(height, vi + half + 1)
    x0, x1 = max(0, ui - half), min(width, ui + half + 1)
    patch = arr[y0:y1, x0:x1]

    metres = patch[np.isfinite(patch)] / scale
    valid = metres[(metres >= lo) & (metres <= hi)]
    if valid.size < required:
        return None
    return _reduce(valid, stat, pct)


def bbox_depth(
    depth: Any,
    bbox: Any,
    *,
    depth_scale: float = 1000.0,
    statistic: str = "median",
    percentile: float = 50.0,
    min_valid_pixels: int = 8,
    min_depth: float = 0.05,
    max_depth: float = 20.0,
) -> float | None:
    """返回包围框的鲁棒度量深度。

    ``min_valid_pixels`` 这道防线是关键。检测器会兴高采烈地在玻璃上、黑色物体上
    以及两个表面之间的空隙处给出框——这些区域深度传感器几乎测不到任何东西。没有
    防护时，幸存的这两三个像素（往往还是*透过*物体看到的背景）会被当作物体深度
    上报，于是所有下游距离都会偏差数米，却看起来完全合理。要求至少 8 个有效像素，
    意味着一个大部分是空洞的框会返回 ``None``，而不是一个捏造的距离。

    参数：
        depth: 原始深度值构成的深度图。
        bbox: 以**深度图**像素为单位的 ``(x1, y1, x2, y2)``；末尾的额外元素会被
            忽略。当框来自尺寸不同的彩色图像时，请先使用
            :func:`reusable_model.vision.pinhole.pixel_to_depth_pixel`。
        depth_scale: 每米对应的原始深度单位数。
        statistic: 要应用的归约方式，:data:`STATISTICS` 之一。
        percentile: ``[0, 100]`` 内的百分位数，供 ``statistic='percentile'`` 使用。
        min_valid_pixels: 框内落在范围内的最少像素数。
        min_depth: 可接受的最小距离，单位为米。
        max_depth: 可接受的最大距离，单位为米。

    返回：
        以米为单位的深度；当框为空/退化、位于图像之外，或有效像素少于
        ``min_valid_pixels`` 时返回 ``None``。

    异常：
        TypeError: 若 ``depth``/``bbox`` 格式不正确，或某参数不是数值。
        ValueError: 若 ``bbox`` 少于 4 个有限数值、深度范围无效，或
            ``statistic``/``percentile`` 未知。

    示例：
        >>> d = np.full((10, 10), 3000, dtype=np.uint16)
        >>> bbox_depth(d, (2, 2, 6, 6))
        3.0
        >>> d[:5, :] = 0            # 上半部分是空洞
        >>> bbox_depth(d, (2, 2, 6, 6), min_valid_pixels=20) is None
        True
    """
    arr = _as_depth(depth)
    scale = _positive_float(depth_scale, "depth_scale")
    lo, hi = _check_limits(min_depth, max_depth)
    stat, pct = _normalise_statistic(statistic, percentile)
    required = _count_int(min_valid_pixels, "min_valid_pixels")

    height, width = arr.shape
    x0, y0, x1, y1 = _bbox_rect(bbox, height, width)
    if x1 <= x0 or y1 <= y0:
        return None
    patch = arr[y0:y1, x0:x1]
    metres = patch[np.isfinite(patch)] / scale
    valid = metres[(metres >= lo) & (metres <= hi)]
    if valid.size < required:
        return None
    return _reduce(valid, stat, pct)


def render_depth(
    depth: Any,
    *,
    depth_scale: float = 1000.0,
    max_depth: float | None = None,
    colormap: str | int = "turbo",
    percentile_lo: float = 2.0,
    percentile_hi: float = 98.0,
    size: tuple[int, int] | None = None,
    annotate: bool = True,
) -> np.ndarray | None:
    """将深度图渲染为供人观看的伪彩色 BGR 图像。

    归一化有意使用*有效*像素的百分位数而不是真实最小/最大值：否则单个坏点
    （65535 mm）或一处杂散反射（0.1 mm）会把色带拉伸到荒谬的范围，使整个场景
    压成同一种色相。在第 2/98 百分位处裁剪能让场景主体在视觉上仍可分辨，只牺牲
    极端尾部，而那里本就不携带信息。

    无效像素被涂成黑色而不是蓝色，这样空洞就能与“非常近”区分开；同时会在左上角
    标注实际使用的范围，因为一幅没有刻度说明的伪彩色图是无法解读的。

    参数：
        depth: 原始深度值构成的深度图。
        depth_scale: 每米对应的原始深度单位数。
        max_depth: 可选，显式指定色带的远端，单位为米；默认取有效像素的
            ``percentile_hi`` 百分位数。
        colormap: 任意 OpenCV ``COLORMAP_*`` 名称（省略前缀，如 ``'turbo'``、
            ``'jet'``、``'inferno'``）或原始 colormap id。默认使用 Turbo，
            因为它在感知上接近均匀，且在灰度下仍可读。
        percentile_lo: 被映射到颜色 0 的有效像素百分位数。
        percentile_hi: 被映射到颜色 255 的有效像素百分位数。
        size: 可选，将结果缩放到的 ``(width, height)``，使用最近邻，以保持空洞
            边界锐利（插值会凭空造出从未测量过的深度）。
        annotate: 在左上角绘制渲染的深度范围。

    返回：
        ``(H, W, 3)`` 的 ``uint8`` BGR 图像；当深度图完全没有有效像素时返回
        ``None``（此时没有有意义的内容可绘制）。

    异常：
        ImportError: 若未安装 OpenCV。
        TypeError: 若某参数为非数值类型。
        ValueError: 若深度图不是二维/为空、百分位数超出 ``[0, 100]`` 或非递增、
            ``size`` 格式不正确，或 colormap 名称未知。

    示例：
        >>> d = np.full((6, 8), 1500, dtype=np.uint16)
        >>> d[0, 0] = 0
        >>> img = render_depth(d, annotate=False)
        >>> img.shape
        (6, 8, 3)
        >>> bool((img[0, 0] == 0).all())      # 空洞保持黑色
        True
        >>> render_depth(np.zeros((4, 4), dtype=np.uint16)) is None
        True
    """
    cv2 = _require_cv2()
    arr = _as_depth(depth)
    scale = _positive_float(depth_scale, "depth_scale")
    lo_pct = _non_negative_float(percentile_lo, "percentile_lo")
    hi_pct = _non_negative_float(percentile_hi, "percentile_hi")
    if lo_pct > 100.0 or hi_pct > 100.0:
        raise ValueError(
            f"percentile_lo/percentile_hi must lie within [0, 100], got "
            f"{percentile_lo!r} and {percentile_hi!r}"
        )
    if hi_pct <= lo_pct:
        raise ValueError(
            f"percentile_hi must be > percentile_lo, got percentile_lo={percentile_lo!r} "
            f"and percentile_hi={percentile_hi!r}"
        )
    if not isinstance(annotate, bool):
        raise TypeError(f"annotate must be a bool, got {annotate!r}")

    metres = arr / scale
    valid = np.isfinite(metres) & (metres > 0.0)
    if not np.any(valid):
        logger.warning("render_depth: depth image of shape %s has no valid pixel", arr.shape)
        return None
    values = metres[valid]

    lo = float(np.percentile(values, lo_pct))
    hi = float(max_depth) if max_depth is not None else float(np.percentile(values, hi_pct))
    if max_depth is not None:
        hi = _positive_float(max_depth, "max_depth")
    if hi <= lo:
        # 完全平坦的场景（或显式 max_depth 低于近端百分位）会导致除零；用毫米
        # 量级的 epsilon 拓宽色带，使输出保持良定义而不是崩溃。
        hi = lo + max(1e-3, abs(lo) * 1e-3)

    norm = np.zeros(arr.shape, dtype=np.uint8)
    ramp = np.clip((metres - lo) / (hi - lo), 0.0, 1.0) * 255.0
    norm[valid] = ramp[valid].astype(np.uint8)

    cmap = colormap
    if isinstance(cmap, str):
        attribute = f"COLORMAP_{cmap.strip().upper()}"
        resolved = getattr(cv2, attribute, None)
        if resolved is None:
            raise ValueError(
                f"unknown colormap {colormap!r}; pass an OpenCV COLORMAP_* name such as "
                f"'turbo', 'jet', 'inferno' or 'magma'"
            )
        cmap = int(resolved)
    elif isinstance(cmap, (int, np.integer)) and not isinstance(cmap, bool):
        cmap = int(cmap)
    else:
        raise TypeError(
            f"colormap must be a string name or an integer id, got {colormap!r} "
            f"({type(colormap).__name__})"
        )

    colored = cv2.applyColorMap(norm, cmap)
    colored[~valid] = 0

    if size is not None:
        if isinstance(size, (str, bytes)):
            raise TypeError(f"size must be a (width, height) pair, got {size!r}")
        dims = tuple(size)
        if len(dims) != 2:
            raise ValueError(f"size must be a (width, height) pair, got {size!r}")
        target_w = int(dims[0])
        target_h = int(dims[1])
        if target_w < 1 or target_h < 1:
            raise ValueError(f"size dimensions must be >= 1, got {size!r}")
        colored = cv2.resize(
            colored, (target_w, target_h), interpolation=cv2.INTER_NEAREST
        )

    if annotate:
        text = f"depth {lo:.2f}-{hi:.2f} m"
        origin = (8, 22)
        # 亮色文字下方的深色描边能让标签在任何颜色上都可读。
        cv2.putText(colored, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(colored, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return colored


def relative_box(
    bbox: Any,
    *,
    y0_frac: float = 0.0,
    y1_frac: float = 1.0,
    x0_frac: float = 0.0,
    x1_frac: float = 1.0,
    image_shape: Any = None,
    pad_x: int = 0,
    pad_y: int = 0,
) -> tuple[int, int, int, int]:
    """按父框自身尺寸的分数偏移，从 ``bbox`` 中截取一个子框。

    用父框的分数来表达区域，可使其与分辨率、尺度无关，这也是感知流程中每个
    “探测检测框某一部分”的启发式规则最终都会这样写的原因。两个典型用法：

    * **足部框**——人框底部 38% 的位置，即鞋所在处，从而可将其深度与地面深度
      作比较::

          relative_box(person_bbox, y0_frac=0.62)

    * **地面探测**——框*下方*的一条细长区域，它位于父框之外，因此无需填充技巧，
      只需用大于 1 的分数::

          relative_box(person_bbox, x0_frac=0.15, x1_frac=0.85,
                       y0_frac=1.0, y1_frac=1.10)

    ``pad_x``/``pad_y`` 会让结果对称地向外扩张，且*可能*把它推出父框之外
    （这正是要点：“脚下方那条区域，再留一点余量”）。由于填充后的区域可能越出
    图像，请传入 ``image_shape`` 对其钳制。

    参数：
        bbox: 父 ``(x1, y1, x2, y2)`` 框，单位为像素。
        y0_frac: 子框上边缘，以父框高度的分数表示。``[0, 1]`` 之外的分数指向父框
            之外的区域：负数表示其上方，``> 1`` 表示其下方。
        y1_frac: 子框下边缘，以父框高度的分数表示。
        x0_frac: 子框左边缘，以父框宽度的分数表示（负数表示父框左侧）。
        x1_frac: 子框右边缘，以父框宽度的分数表示（``> 1`` 表示父框右侧）。
        image_shape: 可选 ``(height, width)``（完整数组形状亦可），用于把结果
            钳制到图像内。
        pad_x: 在子框左*和*右两侧各增加的像素数。
        pad_y: 在子框上*和*下两侧各增加的像素数。

    返回：
        整数 ``(x1, y1, x2, y2)``。当钳制将其完全移除时，框可能退化
        （``x2 <= x1``）；:func:`bbox_depth` 与 :func:`color_ratio_in_bbox`
        会把这样的框视为“无测量值”。

    异常：
        TypeError: 若 ``bbox`` 不是数值序列，或某个分数/填充不是实数。
        ValueError: 若 ``bbox`` 少于 4 个有限数值、某个分数对是反的
            （``y1_frac < y0_frac``），或某个填充为负。

    示例：
        >>> relative_box((100, 100, 200, 300), y0_frac=0.62)
        (100, 224, 200, 300)
        >>> relative_box((100, 100, 200, 300), x0_frac=0.15, x1_frac=0.85,
        ...              y0_frac=1.0, y1_frac=1.10)
        (115, 300, 185, 320)
        >>> relative_box((100, 100, 200, 300), y1_frac=1.5, image_shape=(320, 240),
        ...              pad_y=4)
        (100, 96, 200, 320)
    """
    if isinstance(bbox, (str, bytes)):
        raise TypeError(f"bbox must be a sequence (x1, y1, x2, y2), got {bbox!r}")
    try:
        values = [float(value) for value in bbox]
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"bbox must be a sequence of numbers (x1, y1, x2, y2), got {bbox!r}"
        ) from exc
    if len(values) < 4:
        raise ValueError(
            f"bbox must hold 4 numbers (x1, y1, x2, y2), got {len(values)}: {bbox!r}"
        )
    if not all(math.isfinite(value) for value in values[:4]):
        raise ValueError(f"bbox must hold finite numbers, got {bbox!r}")
    x1, y1, x2, y2 = values[:4]

    fractions = {
        "y0_frac": _finite_float(y0_frac, "y0_frac"),
        "y1_frac": _finite_float(y1_frac, "y1_frac"),
        "x0_frac": _finite_float(x0_frac, "x0_frac"),
        "x1_frac": _finite_float(x1_frac, "x1_frac"),
    }
    if fractions["y1_frac"] < fractions["y0_frac"]:
        raise ValueError(
            f"y1_frac must be >= y0_frac, got y0_frac={y0_frac!r} and y1_frac={y1_frac!r}"
        )
    if fractions["x1_frac"] < fractions["x0_frac"]:
        raise ValueError(
            f"x1_frac must be >= x0_frac, got x0_frac={x0_frac!r} and x1_frac={x1_frac!r}"
        )
    pad_x_px = _non_negative_float(pad_x, "pad_x")
    pad_y_px = _non_negative_float(pad_y, "pad_y")

    # 尺寸为零的父框会让每个分数都坍缩成一个点；将跨度保持为 >= 1 像素，
    # 既与原始启发式的行为一致，也避免把一个已经退化的框再切分成空。
    box_w = max(x2 - x1, 1.0)
    box_h = max(y2 - y1, 1.0)

    left = x1 + fractions["x0_frac"] * box_w - pad_x_px
    right = x1 + fractions["x1_frac"] * box_w + pad_x_px
    top = y1 + fractions["y0_frac"] * box_h - pad_y_px
    bottom = y1 + fractions["y1_frac"] * box_h + pad_y_px

    if image_shape is not None:
        if isinstance(image_shape, (str, bytes)):
            raise TypeError(f"image_shape must be a (height, width) sequence, got {image_shape!r}")
        dims = tuple(image_shape)
        if len(dims) < 2:
            raise ValueError(
                f"image_shape must hold at least (height, width), got {image_shape!r}"
            )
        img_h = int(dims[0])
        img_w = int(dims[1])
        left = min(max(left, 0.0), float(img_w))
        right = min(max(right, 0.0), float(img_w))
        top = min(max(top, 0.0), float(img_h))
        bottom = min(max(bottom, 0.0), float(img_h))

    out_x1, out_y1 = int(round(left)), int(round(top))
    out_x2, out_y2 = int(round(right)), int(round(bottom))
    return out_x1, out_y1, max(out_x2, out_x1), max(out_y2, out_y1)


def color_ratio_in_bbox(
    bgr: Any,
    bbox: Any,
    *,
    lower_hsv: Sequence[int] | Iterable[int],
    upper_hsv: Sequence[int] | Iterable[int],
) -> float:
    """返回包围框中被某个 HSV 颜色范围覆盖的比例。

    颜色比例是廉价、类别无关的证据：“这些脚下区域有 70% 是草坪绿”比仅凭检测器的
    标签是强得多的信号，而且不需要任何模型。在 HSV 而非 BGR 空间工作能让该测试
    在光照变化下依然可用，因为色相基本不受亮度影响，而原始通道则不然。

    该范围是参数（而非硬编码的绿色），因此同一辅助函数可服务于任意颜色测试；
    :data:`HSV_GRASS_GREEN` 提供了草坪绿的边界。

    参数：
        bgr: BGR 通道顺序的 ``uint8`` 图像，形状 ``(H, W, 3)`` 或 ``(H, W, 4)``
            （alpha 通道会被丢弃）。
        bbox: 以图像像素为单位的 ``(x1, y1, x2, y2)``；末尾的额外元素会被忽略。
        lower_hsv: 闭区间下界 ``(H, S, V)``，其中 ``H`` 在 ``[0, 179]``，
            ``S``/``V`` 在 ``[0, 255]``。
        upper_hsv: 闭区间上界，各分量 ``>= lower_hsv``。

    返回：
        ``[0.0, 1.0]`` 内的比例。对于空框或完全在图像外的框返回 ``0.0``——这不算
        错误，因为“未找到证据”对一个不存在的区域而言是诚实的答案；格式错误的
        *参数*仍会抛异常。

    异常：
        ImportError: 若未安装 OpenCV。
        TypeError: 若 ``bgr`` 不是 ``uint8`` 数组，或某个边界不是数值。
        ValueError: 若 ``bgr`` 不是三维且通道数为 3/4、某个边界超出 OpenCV 的
            HSV 范围、``lower_hsv`` 逐分量大于 ``upper_hsv``，或 ``bbox`` 少于
            4 个有限数值。

    示例：
        >>> import numpy as np
        >>> green = np.zeros((10, 10, 3), dtype=np.uint8)
        >>> green[:, :] = (0, 200, 0)                 # BGR green
        >>> lower, upper = HSV_GRASS_GREEN
        >>> color_ratio_in_bbox(green, (0, 0, 10, 10), lower_hsv=lower, upper_hsv=upper)
        1.0
        >>> color_ratio_in_bbox(green, (50, 50, 60, 60), lower_hsv=lower, upper_hsv=upper)
        0.0
    """
    cv2 = _require_cv2()

    if not isinstance(bgr, np.ndarray):
        try:
            bgr = np.asarray(bgr)
        except (TypeError, ValueError) as exc:
            raise TypeError(
                f"bgr must be an array-like uint8 image, got {type(bgr).__name__}"
            ) from exc
    if bgr.dtype != np.uint8:
        raise TypeError(
            f"bgr must be a uint8 image (OpenCV's 8-bit HSV encoding is assumed), "
            f"got dtype {bgr.dtype}"
        )
    if bgr.ndim != 3 or bgr.shape[2] not in (3, 4):
        raise ValueError(
            f"bgr must have shape (H, W, 3) or (H, W, 4) in BGR order, got {bgr.shape}"
        )
    image = bgr[:, :, :3] if bgr.shape[2] == 4 else bgr

    bounds: list[tuple[int, int, int]] = []
    for bound, bound_name in ((lower_hsv, "lower_hsv"), (upper_hsv, "upper_hsv")):
        try:
            components = [float(value) for value in bound]
        except (TypeError, ValueError) as exc:
            raise TypeError(
                f"{bound_name} must be a sequence of 3 numbers (H, S, V), got {bound!r}"
            ) from exc
        if len(components) != 3 or not all(math.isfinite(value) for value in components):
            raise ValueError(
                f"{bound_name} must be 3 finite numbers (H, S, V), got {bound!r}"
            )
        h, s, v = (int(round(value)) for value in components)
        if not (0 <= h <= 179):
            raise ValueError(
                f"{bound_name} hue must lie within OpenCV's [0, 179], got {bound!r}"
            )
        for channel_value, channel_name in ((s, "saturation"), (v, "value")):
            if not (0 <= channel_value <= 255):
                raise ValueError(
                    f"{bound_name} {channel_name} must lie within [0, 255], got {bound!r}"
                )
        bounds.append((h, s, v))
    lower, upper = bounds
    if any(low > high for low, high in zip(lower, upper)):
        raise ValueError(
            f"lower_hsv must be component-wise <= upper_hsv, got lower_hsv={lower_hsv!r} "
            f"and upper_hsv={upper_hsv!r}"
        )

    height, width = image.shape[:2]
    x0, y0, x1, y1 = _bbox_rect(bbox, height, width)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    patch = image[y0:y1, x0:x1]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    return float(np.count_nonzero(mask)) / float(mask.size)
