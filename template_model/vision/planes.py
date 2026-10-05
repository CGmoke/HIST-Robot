"""平面拟合、射线-平面求交与相对地面的测量。

深度相机告诉你*表面*在哪里，但几乎每个操作决策都需要一个放手或放轮子的位置：
**物体下方的地面**。那个点很少能直接测量到，这正是本模块的用途。它提供

* 带明确朝向约定的总体最小二乘（total-least-squares）平面拟合
  （:func:`fit_plane`、:func:`fit_plane_oriented`），
* 将一个像素转换为地面点的射线-平面求交
  （:func:`ray_plane_intersection`、:func:`pixel_to_plane`），
* 一个环形采样器，在检测框*周围*采集地面，使平面可以就地拟合而非凭空假设
  （:func:`sample_ground_ring`），
* 拟合质量度量（:func:`plane_residuals`、:func:`plane_fit_rmse`、
  :func:`point_to_plane_distance`），
* 针对检测的、与距离无关的物理尺寸度量（:func:`bbox_3d_area`），
* 点集的形状/伸长分析（:func:`fit_ellipse_to_points`、
  :func:`elongation_from_points`）。

为什么用平面而非原始深度？检测框中心像素的深度值*并不*是物体与地面接触处的距离：
它落在顶面、光亮盖子上，或落在传感器完全没有回波的空洞里。将中心视线射线与紧挨
物体拟合出的地面平面求交，可以消除这一整类误差，因为地面面积大、呈哑光、测量可靠
——正好与物体相反。完整论证见 :func:`pixel_to_plane`。

坐标系约定：点的坐标为 :mod:`reusable_model.vision.pinhole` 的**相机光学坐标系**
（``x`` 向右、``y`` 向下、``z`` 向前）。此处不依赖 ROS 或机器人模型；若需要，
请在之后把拟合出的平面变换到机体坐标系。

依赖：导入时仅依赖 :mod:`numpy`。``cv2`` 由 :func:`fit_ellipse_to_points` 惰性
导入，因此平面拟合与射线投射在无 OpenCV 的无头环境中也能工作。
"""

from __future__ import annotations

import logging
import math
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

from .depth import sample_depth, valid_mask
from .pinhole import Intrinsics, pixel_to_3d, pixel_to_3d_batch, ray_direction

logger = logging.getLogger(__name__)

__all__ = [
    "fit_plane",
    "fit_plane_oriented",
    "ray_plane_intersection",
    "pixel_to_plane",
    "sample_ground_ring",
    "plane_residuals",
    "plane_fit_rmse",
    "point_to_plane_distance",
    "bbox_3d_area",
    "fit_ellipse_to_points",
    "elongation_from_points",
]

#: 第二奇异值与第一奇异值之比 ``s2 / s1``，低于该值时点集被视为退化
#: （点共线或重合）。
#:
#: 一个平面需要两个独立的延展方向。当点共线时，两个较小的奇异值都只是数值噪声，
#: 而「法线」是垂直于该线的平面内的任意方向——这是一个毫无意义的答案，但对调用方
#: 而言看起来却完全有效。拒绝它比调试一次横向偏了 30 cm 的抓取要划算得多。
_DEGENERATE_RATIO: float = 1e-9

#: 椭圆拟合在被视为退化之前可以报告的最小轴长（以点集的单位计）。
#: ``cv2.fitEllipse`` 对重复点会欣然返回一个零宽度矩形，而用它做除数会得到
#: ``inf`` 长宽比，从而通过任何 ``> threshold`` 判定。
_MIN_ELLIPSE_AXIS: float = 1e-9

#: OpenCV 至少需要 5 个点才能拟合二次曲线（椭圆有五个自由度），因此当请求的
#: ``min_points`` 更小时会被静默提升到该值。
_CV2_FIT_ELLIPSE_MIN_POINTS: int = 5


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
    """将计数类参数（像素步长、点数）转换为 ``int``。

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


def _as_points(points: Any, dim: int, name: str) -> np.ndarray:
    """校验点集，并以二维 ``float64`` 数组返回。

    *空*可迭代对象会被接受并重塑为 ``(0, dim)``：感知流水线会让这些辅助函数在
    滤波结果上串联作用，若拒绝合法的空情形，就会迫使每个调用方都特殊处理它。

    参数：
        points: ``(N, dim)`` 的数组类对象，元素为有限坐标。
        dim: 每个点期望的坐标数（2 或 3）。
        name: 用于错误信息的参数名。

    返回：
        ``(N, dim)`` 的 ``float64`` 数组。

    异常：
        TypeError: 若 ``points`` 不是数组类对象。
        ValueError: 若列数不等于 ``dim``，或某个坐标非有限。
    """
    try:
        arr = np.asarray(points, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"{name} must be an array-like (N, {dim}) point set, got "
            f"{type(points).__name__}: {exc}"
        ) from exc
    if arr.size == 0:
        return arr.reshape(0, dim)
    if arr.ndim != 2 or arr.shape[1] != dim:
        raise ValueError(
            f"{name} must have shape (N, {dim}), got {arr.shape} from "
            f"{np.asarray(points).tolist()!r}"
        )
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must contain only finite coordinates, got {arr.tolist()!r}")
    return arr


def _as_vector3(value: Any, name: str) -> np.ndarray:
    """将 ``value`` 转换为由 3 个元素组成的有限 ``float64`` 向量。

    参数：
        value: 任何恰好含 3 个元素的数组类对象。
        name: 用于错误信息的参数名。

    返回：
        ``(3,)`` 的 ``float64`` 数组。

    异常：
        TypeError: 若 ``value`` 不是数组类对象。
        ValueError: 若其不含恰好 3 个有限数值。
    """
    try:
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be array-like with 3 elements, got {value!r}") from exc
    if arr.size != 3:
        raise ValueError(f"{name} must have exactly 3 elements, got {arr.size}: {value!r}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return arr


def _as_unit_vector3(value: Any, name: str) -> np.ndarray:
    """将 ``value`` 转换为有限的单位 3-向量。

    参数：
        value: 任何含 3 个元素且范数非零的数组类对象。
        name: 用于错误信息的参数名。

    返回：
        ``(3,)`` 的 ``float64`` 单位向量。

    异常：
        TypeError: 若 ``value`` 不是数组类对象。
        ValueError: 若其不含 3 个有限数值，或其范数（接近）为零、无法从中恢复
            任何方向。
    """
    vec = _as_vector3(value, name)
    norm = float(np.linalg.norm(vec))
    if norm < 1e-12:
        raise ValueError(
            f"{name} must not be the zero vector (no direction can be recovered "
            f"from it), got {value!r}"
        )
    return vec / norm


def _as_depth(depth: Any, name: str = "depth") -> np.ndarray:
    """校验深度缓冲区，并以二维 ``float64`` 数组返回。

    这里一次性完成转换，使后续算术不会让 ``uint16`` 累加器溢出，也使
    ``NaN``/``inf`` 这类哨兵值可以被表示。

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
        min_depth: 最小合理距离，单位为米（允许 ``0``，仅表示「不做近端裁剪」）。
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


def _bbox_rect(
    bbox: Any,
    height: int,
    width: int,
    *,
    name: str = "bbox",
) -> tuple[int, int, int, int]:
    """将包围框转换为经钳制的整数矩形。

    左上角使用 ``floor``、右下角使用 ``ceil``，因此带小数的框总能覆盖它触及的
    每个像素——若两个角都取整，会静默地缩小小框。

    参数：
        bbox: 以像素为单位的 ``(x1, y1, x2, y2)``；末尾的额外元素（某些检测
            格式会追加一个分数）会被忽略。
        height: 用于钳制的图像高度。
        width: 用于钳制的图像宽度。
        name: 用于错误信息的参数名。

    返回：
        图像内的 ``(x0, y0, x1, y1)``，末端为开区间。当输入完全位于图像之外时，
        该框可能为空（``x1 <= x0``）；调用方必须处理这种情况。

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


def _triangle_area(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    """返回三维三角形 ``a``、``b``、``c`` 的面积（单位 m^2）。

    使用叉积形式 ``0.5 * |(b - a) x (c - a)|`` 而非海伦公式，因为它不需要边长，
    且对非常细长的三角形仍保持精确——在这类情形下海伦公式会发生灾难性抵消。

    参数：
        a: 第一个角点，``(3,)``。
        b: 第二个角点，``(3,)``。
        c: 第三个角点，``(3,)``。

    返回：
        三角形面积，非负 ``float``（当三个角点共线时为 ``0.0``）。
    """
    return 0.5 * float(np.linalg.norm(np.cross(b - a, c - a)))


# -- 平面拟合 --------------------------------------------------------


def fit_plane(points: ArrayLike) -> tuple[np.ndarray, np.ndarray] | None:
    """用总体最小二乘（SVD）对三维点集拟合平面。

    该拟合最小化*垂直距离的平方和*——而不是沿某一轴的误差平方和。这一区别在此处很
    重要：普通回归 ``z = a x + b y + c`` 假定因变量轴是含噪的那一轴，而对深度相机
    看到的地面来说这是错误的（噪声沿视线方向，也就是主要沿 ``z``）。总体最小二乘
    对称地对待三个坐标，因此是正确的估计方法。

    解是中心化后点矩阵**最小**奇异值所对应的右奇异向量：记 ``X`` 为
    ``points - centroid`` 的 ``(N, 3)`` 矩阵，其 SVD ``X = U S Vh`` 按解释方差
    从大到小排列 ``Vh`` 的各行，因此 ``Vh[-1]`` 是点云最薄的方向——即法线。

    参数：
        points: ``(N, 3)`` 的数组类对象，元素为有限的三维点，且均在同一坐标系中。

    返回：
        ``(unit_normal, centroid)``，其中 ``unit_normal`` 是长度为 1 的 ``(3,)``
        ``float64`` 向量，``centroid`` 是输入点的 ``(3,)`` 均值（该点位于拟合出的
        平面上）。当给出的点少于 3 个，或点集退化（所有点重合，或共线以致平面不被
        唯一确定）时返回 ``None``——绝不返回一个凭空编造的法线。

        法线的符号是**任意的**：SVD 确定的是直线而非朝向。当调用方需要已知的一侧
        时，请使用 :func:`fit_plane_oriented`。

    异常：
        TypeError: 若 ``points`` 不是数组类对象。
        ValueError: 若 ``points`` 的形状不是 ``(N, 3)``，或含非有限坐标。

    示例：
        >>> import numpy as np
        >>> pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
        ...                 [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]])
        >>> normal, centroid = fit_plane(pts)
        >>> np.allclose(np.abs(normal), [0.0, 0.0, 1.0])
        True
        >>> np.allclose(centroid, [0.5, 0.5, 0.0])
        True
        >>> fit_plane(np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])) is None
        True
        >>> fit_plane(np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
        ...                     [2.0, 0.0, 0.0]])) is None       # collinear
        True
    """
    arr = _as_points(points, 3, "points")
    if arr.shape[0] < 3:
        logger.debug("fit_plane: need >= 3 points, got %d", arr.shape[0])
        return None

    centroid = arr.mean(axis=0)
    centered = arr - centroid
    _, singular, vh = np.linalg.svd(centered, full_matrices=False)
    if singular[0] <= 0.0 or singular[1] <= _DEGENERATE_RATIO * singular[0]:
        logger.debug(
            "fit_plane: degenerate point set (singular values %s); the plane is "
            "not uniquely determined",
            np.array2string(singular, precision=3),
        )
        return None

    normal = vh[-1]
    norm = float(np.linalg.norm(normal))
    return normal / norm, centroid


def fit_plane_oriented(
    points: ArrayLike,
    *,
    reference_axis: int = 2,
    sign: float = -1.0,
) -> tuple[np.ndarray, np.ndarray] | None:
    """拟合平面并将法线翻转到参考轴的已知一侧。

    :func:`fit_plane` 返回的法线符号是任意的，而任意的符号是一个潜在 bug：每个
    下游判断——比较 ``dot(normal, view_direction)``，或决定「这是地板还是天花板？」
    ——都会在一半的帧上静默地反号。

    典型的用例是**在相机光学坐标系中观测到的地面平面**（``z`` 向前、射出镜头）。
    地面位于相机下方且在前方，因此*指向相机*的法线的 ``z`` 分量为负。这正好就是
    ``reference_axis=2, sign=-1.0``（默认值）：返回的法线满足 ``normal[2] < 0``。
    如此定向会让 :func:`ray_plane_intersection` 中射线/平面的分母对前向射线为负，
    这是一个廉价的健全性检查，也使 :func:`plane_residuals` 的有符号残差可被解释
    （「正号表示位于地面上方」）。

    参数：
        points: ``(N, 3)`` 的数组类对象，元素为有限的三维点。
        reference_axis: 其符号决定朝向的轴索引（``0`` = x，``1`` = y，``2`` = z）。
        sign: ``normal[reference_axis]`` 所需符号，必须非零。``-1.0`` 要求分量为负，
            ``+1.0`` 要求为正。

    返回：
        ``(unit_normal, centroid)``，与 :func:`fit_plane` 完全相同，必要时对法线
        取反，使 ``normal[reference_axis] * sign > 0``。当拟合本身失败时返回
        ``None``。

    异常：
        TypeError: 若 ``points`` 不是数组类对象、``reference_axis`` 不是整数，或
            ``sign`` 不是实数。
        ValueError: 若 ``points`` 格式不正确、``reference_axis`` 不是 0/1/2，或
            ``sign`` 为零或非有限。

    示例：
        >>> import numpy as np
        >>> pts = np.array([[0.0, 0.0, 2.0], [1.0, 0.0, 2.0],
        ...                 [0.0, 1.0, 2.0], [1.0, 1.0, 2.0]])
        >>> normal, _ = fit_plane_oriented(pts)          # default: z < 0
        >>> np.allclose(normal, [0.0, 0.0, -1.0])
        True
        >>> normal, _ = fit_plane_oriented(pts, sign=1.0)
        >>> np.allclose(normal, [0.0, 0.0, 1.0])
        True
    """
    axis = _count_int(reference_axis, "reference_axis", minimum=0)
    if axis > 2:
        raise ValueError(
            f"reference_axis must be 0 (x), 1 (y) or 2 (z), got {reference_axis!r}"
        )
    wanted = _finite_float(sign, "sign")
    if wanted == 0.0:
        raise ValueError(
            f"sign must be non-zero (it selects which side of the plane the "
            f"normal points to), got {sign!r}"
        )

    fitted = fit_plane(points)
    if fitted is None:
        return None
    normal, centroid = fitted
    if normal[axis] * wanted < 0.0:
        normal = -normal
    if abs(float(normal[axis])) < 1e-6:
        # 平面（几乎）平行于参考轴，因此其法线（几乎）垂直于它：所请求的朝向在
        # 数值上没有意义。返回该拟合仍然比直接失败更有用，但调用方应当知道这个
        # 符号并不具有指示性。
        logger.warning(
            "fit_plane_oriented: normal %s is nearly perpendicular to axis %d "
            "(component %.3e); the requested sign=%.1f orientation is not "
            "meaningful for this plane",
            np.array2string(normal, precision=4), axis, float(normal[axis]), wanted,
        )
    return normal, centroid


# -- 射线 / 平面求交 --------------------------------------------


def ray_plane_intersection(
    ray_origin: ArrayLike,
    ray_direction: ArrayLike,
    plane_normal: ArrayLike,
    plane_point: ArrayLike,
    *,
    parallel_eps: float = 1e-9,
) -> np.ndarray | None:
    """求射线与无限平面的交点。

    对射线参数求解 ``dot(n, origin + t * direction - point) = 0``，即
    ``t = dot(n, point - origin) / dot(n, direction)``，并返回该 ``t`` 处的三维点。
    下面两种拒绝情形都返回 ``None``，而不是一个被钳制或外推的点，因为在这两种情形下
    平面上*没有*任何点是正确答案，而任何编造出来的点都会看起来合情合理：

    * **平行**（``|dot(n, direction)| < parallel_eps``）——射线与平面永不相交，
      或完全位于平面内。对地面平面而言，这发生在相机正对地平线时；除法会爆炸，
      而真实答案退向无穷远。
    * **位于原点之后**（``t <= 0``）——几何交点存在，但位于相机之后。拟合到机器人
      身后墙面的平面，或翻转过的法线，都会产生这种情形；该点在物理上不可观测，
      绝不能用作抓取目标。

    参数：
        ray_origin: ``(3,)`` 射线起点；对像素射线而言即相机中心。
        ray_direction: ``(3,)`` 射线方向，非零。要使*交点*正确，它不必是单位长度；
          但只有单位方向才能使 ``t`` 成为从原点出发的真实度量距离。
          :func:`reusable_model.vision.pinhole.ray_direction` 返回单位向量。
        plane_normal: ``(3,)`` 平面法线，非零；不必是单位长度。
        plane_point: ``(3,)`` 平面上的任意一点（拟合的质心即可）。
        parallel_eps: 对 ``|dot(normal, direction)|`` 的阈值，低于该值时射线被视为
          平行。它与*原始*点积比较，因此只对（接近）单位长度的输入有意义。

    返回：
        与输入同一坐标系下的 ``(3,)`` ``float64`` 交点，或当射线与平面平行、或与
        平面相交于原点之后时返回 ``None``。

    异常：
        TypeError: 若某个参数不是数组类对象，或 ``parallel_eps`` 不是实数。
        ValueError: 若某个向量不含 3 个有限数值、``ray_direction`` 或
            ``plane_normal`` 是零向量，或 ``parallel_eps`` 为负数。

    示例：
        >>> import numpy as np
        >>> hit = ray_plane_intersection([0.0, 0.0, 0.0], [0.0, 1.0, 0.0],
        ...                              [0.0, 1.0, 0.0], [0.0, 2.0, 0.0])
        >>> np.allclose(hit, [0.0, 2.0, 0.0])
        True
        >>> ray_plane_intersection([0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
        ...                        [0.0, 1.0, 0.0], [0.0, 2.0, 0.0]) is None
        True
        >>> ray_plane_intersection([0.0, 0.0, 0.0], [0.0, -1.0, 0.0],
        ...                        [0.0, 1.0, 0.0], [0.0, 2.0, 0.0]) is None
        True
    """
    eps = _non_negative_float(parallel_eps, "parallel_eps")
    origin = _as_vector3(ray_origin, "ray_origin")
    direction = _as_vector3(ray_direction, "ray_direction")
    normal = _as_vector3(plane_normal, "plane_normal")
    point = _as_vector3(plane_point, "plane_point")
    if float(np.linalg.norm(direction)) < 1e-12:
        raise ValueError(
            f"ray_direction must not be the zero vector (a ray needs a "
            f"direction), got {ray_direction!r}"
        )
    if float(np.linalg.norm(normal)) < 1e-12:
        raise ValueError(
            f"plane_normal must not be the zero vector (it defines the plane), "
            f"got {plane_normal!r}"
        )

    denominator = float(np.dot(normal, direction))
    if abs(denominator) < eps:
        logger.debug(
            "ray_plane_intersection: ray parallel to the plane "
            "(dot(normal, direction) = %.3e < %.3e)", denominator, eps,
        )
        return None
    t = float(np.dot(normal, point - origin)) / denominator
    if t <= 0.0:
        logger.debug(
            "ray_plane_intersection: intersection at t=%.6f lies behind the ray "
            "origin; rejecting", t,
        )
        return None
    return origin + t * direction


def pixel_to_plane(
    u: float,
    v: float,
    plane_normal: ArrayLike,
    plane_point: ArrayLike,
    intrinsics: Any,
) -> np.ndarray | None:
    """返回像素 ``(u, v)`` 的视线射线与平面的交点。

    这是 :func:`reusable_model.vision.pinhole.ray_direction`（穿过该像素的单位射线）与
    :func:`ray_plane_intersection` 的组合，也是把一个检测转换为可操作三维位置最有
    用的方式。其推理值得详细说明，因为显而易见的替代方案——「直接读取中心像素的
    深度」——在实践中会不断失败：

    * **中心像素不在地面上。** 对于立在地板上的物体，其包围框或掩码的中心投影到
      物体的*顶面*或*中部*。那里的深度是到物体表面的距离，比夹爪或轮子实际需要的
      接触点近了数十厘米。误差随物体高度增大，因此高物体看起来会系统性地比实际更
      靠近机器人。
    * **中心像素常常是空洞。** 深度传感器在无纹理、深色、光亮或透明的表面上会失效
      ——而这正是塑料袋或瓶子的中部。那里缺失或为零的深度要么导致异常，要么更糟，
      产生相机中心处一个静默的 ``(0, 0, 0)`` 点。
    * **中心像素可能是背景。** 检测器会为部分遮挡或凹陷的物体画出包围框；此时框
      中心属于物体后面的墙面，报告的距离会大出数米。
    * **边缘是最糟糕的采样位置。** 即便中心附近有鲁棒的中值窗口，也只能对那里的
      像素求平均；在物体/地面边界附近，传感器会产生介于两个表面之间、二者皆非的
      「飞点」。

    将中心*射线*与紧挨物体拟合出的地面平面求交可绕开全部四个问题。地面面积大、呈
    哑光、测量可靠，因此平面拟合稳定；射线承载着物体中心像素真正给出的唯一信息
    （一个方向）；而结果就是沿该方向直接落在地面上的点——物体的落点足迹。它还是
    自校验的：若射线与地面平行，或与地面相交于相机之后，
    :func:`ray_plane_intersection` 会返回 ``None``，而不是一个看似合理的错误数值。

    参数：
        u: 内参所属图像中的像素列（可为亚像素）。
        v: 像素行（可为亚像素）。
        plane_normal: 相机光学坐标系下的 ``(3,)`` 平面法线，例如来自
            :func:`fit_plane_oriented`。
        plane_point: ``(3,)`` 平面上的任意一点，例如拟合的质心。
        intrinsics: :class:`reusable_model.vision.pinhole.Intrinsics` 类对象（任何暴露
            ``fx``、``fy``、``cx``、``cy`` 的对象）。

    返回：
        相机光学坐标系下的 ``(3,)`` ``float64`` 点（米），或当像素射线未在相机前方
        击中平面时返回 ``None``。

    异常：
        TypeError: 若某个坐标不是数值、某个向量不是数组类对象，或内参格式不正确。
        ValueError: 若某个像素坐标或向量分量非有限，或某个向量是零向量。

    示例：
        >>> import numpy as np
        >>> intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
        >>> plane_z = np.array([0.0, 0.0, 1.0])            # the plane z = 2 m
        >>> hit = pixel_to_plane(820, 240, plane_z, [0.0, 0.0, 2.0], intr)
        >>> np.allclose(hit, [2.0, 0.0, 2.0])
        True
        >>> pixel_to_plane(320, 240, [0.0, 1.0, 0.0], [0.0, 1.0, 0.0],
        ...                intr) is None                    # ray || plane
        True
    """
    direction = ray_direction(u, v, intrinsics)
    origin = np.zeros(3, dtype=np.float64)
    return ray_plane_intersection(origin, direction, plane_normal, plane_point)


# -- 地面环形采样 -------------------------------------------------


def sample_ground_ring(
    depth: Any,
    bbox: Any,
    intrinsics: Any,
    *,
    margin_px: int = 60,
    exclude_px: int = 10,
    step_px: int = 4,
    min_points: int = 10,
    depth_scale: float = 1000.0,
    min_depth: float = 0.05,
    max_depth: float = 20.0,
) -> tuple[np.ndarray, tuple[np.ndarray, np.ndarray] | None, list[tuple[int, int, float]]]:
    """在检测框周围采样地面，并拟合局部地面平面。

    采样区域是一个**环**：包围框向外扩展 ``margin_px`` 所得的区域，减去同一包围框
    向外扩展 ``exclude_px`` 所得的区域。这两个宽度都是有意为之。

    *外侧余量。* 紧挨物体的地面经常被物体本身遮挡（传感器看到的是物体而非地面，
    物体的阴影处正是其自身像素所在），而仅用边界处的一小把点拟合出的平面会被该
    边界主导。向外扩展到 ``margin_px`` 像素可以采集到真正可见的地面，并在图像中占据
    足够的范围以约束平面的*倾斜*而不仅是高度。在典型的 1280x720 分辨率下，60 像素
    大约是手臂可及处 10-20 cm 的地面——近得仍属于*同一*局部地面（中间没有路缘或
    台阶），又远得足以避开物体。

    *被排除的带。* 这部分容易被省略，却难以调试。物体/地面边界处的深度是系统性
    错误的：传感器会在一个横跨两个距离差异很大的表面的采样足迹上做积分，产生位于
    物体与地板之间任意位置的「混合」或「飞」像素。单个位于物体半高处的此类像素就能
    让平面拟合倾斜几度，而这些像素构成*环绕物体的一圈*，它们的偏差是相干的而非
    随机的——求平均无法消除它。因此保留 ``exclude_px`` 像素的余量可以防止物体自身
    的轮廓污染用于定位该物体的那个表面。当检测框很紧时，排除框也正是阻止物体像素
    进入拟合的原因。

    每个采样点还会用 :func:`reusable_model.vision.depth.valid_mask` 做范围检查，因此传感器空洞
    （``0``）、反射以及超出量程的回波永远不会进入拟合；三维点则来自
    :func:`reusable_model.vision.pinhole.pixel_to_3d_batch`，使投影数学只存在于一处。

    参数：
        depth: 原始深度值构成的深度图（默认为 ``uint16`` 毫米）。
        bbox: 检测框的 ``(x1, y1, x2, y2)``，以**深度图**像素为单位；末尾的额外
            元素会被忽略。当框来自尺寸不同的彩色图像时，请先使用
            :func:`reusable_model.vision.pinhole.pixel_to_depth_pixel`。
        intrinsics: 与深度图匹配的 :class:`reusable_model.vision.pinhole.Intrinsics` 类对象。
        margin_px: 在框外多远范围内采集地面，单位为像素。
        exclude_px: *被排除*的带在框外延伸多远，单位为像素。必须小于 ``margin_px``。
        step_px: 采样栅格的像素步长。步长为 4 时点数保持在数百量级——足以稳定拟合，
            又足够廉价以对每帧每个检测运行。
        min_points: 尝试拟合所需的最少有效采样数。至少为 3（平面需要三个点）；
            默认值 10 为 SVD 留出足够的冗余以平均掉传感器噪声。
        depth_scale: 每米对应的原始深度单位数（``米 = 原始值 / depth_scale``）。
        min_depth: 可接受的最小距离，单位为米。
        max_depth: 可接受的最大距离，单位为米。

    返回：
        ``(points_3d, plane, samples)``，其中

        * ``points_3d`` 是相机光学坐标系下有效采样点的 ``(N, 3)`` ``float64`` 数组
          （可能为 ``(0, 3)``）；
        * ``plane`` 是 ``(unit_normal, centroid)``，法线朝向相机
          （``normal[2] < 0``）；当有效采样少于 ``min_points`` 或点集退化时为
          ``None``；
        * ``samples`` 是每个有效采样的 ``(u, v, depth_m)`` 三元组列表，可直接绘制
          到图像上进行调试。即便拟合失败也会返回它，因为「为什么失败」正是可视化
          工具能够回答的问题。

    异常：
        TypeError: 若 ``depth``/``bbox`` 格式不正确、某个像素计数不是整数，或内参
            格式不正确。
        ValueError: 若深度图不是二维/为空、``bbox`` 含少于 4 个有限数值、
            ``margin_px <= exclude_px``（此时环在构造上即为空）、某个步长/计数太小，
            或深度范围无效。

    示例：
        >>> import numpy as np
        >>> intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
        >>> flat = np.full((120, 160), 2000, dtype=np.uint16)   # floor at 2 m
        >>> pts, plane, samples = sample_ground_ring(flat, (60, 40, 100, 80), intr)
        >>> pts.shape[0] > 100 and len(samples) == pts.shape[0]
        True
        >>> np.allclose(plane[0], [0.0, 0.0, -1.0])     # 法线朝向相机
        True
        >>> np.allclose(plane[1][2], 2.0)               # centroid on z = 2 m
        True
    """
    arr = _as_depth(depth)
    height, width = arr.shape
    scale = _positive_float(depth_scale, "depth_scale")
    lo, hi = _check_limits(min_depth, max_depth)
    margin = _count_int(margin_px, "margin_px", minimum=0)
    exclude = _count_int(exclude_px, "exclude_px", minimum=0)
    step = _count_int(step_px, "step_px")
    required = _count_int(min_points, "min_points", minimum=3)
    if margin <= exclude:
        raise ValueError(
            f"margin_px must be > exclude_px, otherwise the sampled ring is "
            f"empty by construction; got margin_px={margin_px!r} and "
            f"exclude_px={exclude_px!r}"
        )

    x0, y0, x1, y1 = _bbox_rect(bbox, height, width)
    outer_x0 = max(0, x0 - margin)
    outer_y0 = max(0, y0 - margin)
    outer_x1 = min(width, x1 + margin)
    outer_y1 = min(height, y1 + margin)
    # 排除框有意*不*钳制到图像内：触及边界的检测框仍必须排除会落到图像外的
    # 那条带。
    inner_x0, inner_y0 = x0 - exclude, y0 - exclude
    inner_x1, inner_y1 = x1 + exclude, y1 + exclude

    empty: np.ndarray = np.empty((0, 3), dtype=np.float64)
    rows = np.arange(outer_y0, outer_y1, step)
    cols = np.arange(outer_x0, outer_x1, step)
    if rows.size == 0 or cols.size == 0:
        logger.debug(
            "sample_ground_ring: bbox %s leaves no ring pixel inside the %dx%d "
            "image (margin=%d)", (x0, y0, x1, y1), height, width, margin,
        )
        return empty, None, []

    uu, vv = np.meshgrid(cols, rows)
    outside_object = ~(
        (uu >= inner_x0) & (uu <= inner_x1) & (vv >= inner_y0) & (vv <= inner_y1)
    )
    raw = arr[vv, uu]
    in_range = valid_mask(raw, depth_scale=scale, min_depth=lo, max_depth=hi)
    # 额外的 `raw > 0` 守卫使 pixel_to_3d_batch（它拒绝非正深度）在调用方设置
    # min_depth=0 时仍可用。
    selected = outside_object & in_range & (raw > 0.0)

    sel_u = uu[selected]
    sel_v = vv[selected]
    metres = raw[selected] / scale
    samples: list[tuple[int, int, float]] = [
        (int(u), int(v), float(d)) for u, v, d in zip(sel_u.tolist(), sel_v.tolist(), metres.tolist())
    ]
    if sel_u.size == 0:
        logger.debug(
            "sample_ground_ring: no valid ground pixel around bbox %s "
            "(depth range %.2f-%.2f m)", (x0, y0, x1, y1), lo, hi,
        )
        return empty, None, samples

    uv = np.column_stack((sel_u.astype(np.float64), sel_v.astype(np.float64)))
    points = pixel_to_3d_batch(uv, metres, intrinsics)
    if points.shape[0] < required:
        logger.debug(
            "sample_ground_ring: only %d valid ground point(s) (< min_points=%d); "
            "not fitting a plane", points.shape[0], required,
        )
        return points, None, samples

    plane = fit_plane_oriented(points, reference_axis=2, sign=-1.0)
    if plane is None:
        logger.debug(
            "sample_ground_ring: %d ground points are degenerate; no plane",
            points.shape[0],
        )
    return points, plane, samples


# -- 拟合质量 ----------------------------------------------------------


def plane_residuals(
    points: ArrayLike,
    normal: ArrayLike,
    point: ArrayLike,
) -> np.ndarray:
    """返回每个点到平面的有符号垂直距离。

    与 :func:`point_to_plane_distance` 不同，这里保留符号，因为它承载着物理上有意义
    的信息：当法线朝向相机时，正残差表示「位于平面上远离相机的一侧」，这正是属于
    物体而非地板的离群点暴露自身的方式。

    参数：
        points: ``(N, 3)`` 的数组类对象，元素为有限的三维点。
        normal: ``(3,)`` 平面法线，非零；它会在内部被归一化，因此非单位向量除了
            改变结果的尺度外不产生任何影响。
        point: ``(3,)`` 平面上的任意一点。

    返回：
        ``(N,)`` 的 ``float64`` 数组，元素为以 ``points`` 的单位表示的有符号距离
        （``(0, 3)`` 的输入会得到空数组）。

    异常：
        TypeError: 若某个参数不是数组类对象。
        ValueError: 若 ``points`` 的形状不是 ``(N, 3)``、某个向量不含 3 个有限
            数值，或 ``normal`` 是零向量。

    示例：
        >>> import numpy as np
        >>> pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.5], [0.0, 1.0, -0.25]])
        >>> plane_residuals(pts, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]).tolist()
        [0.0, 0.5, -0.25]
    """
    arr = _as_points(points, 3, "points")
    unit = _as_unit_vector3(normal, "normal")
    on_plane = _as_vector3(point, "point")
    if arr.shape[0] == 0:
        return np.empty(0, dtype=np.float64)
    return (arr - on_plane) @ unit


def plane_fit_rmse(
    points: ArrayLike,
    normal: ArrayLike,
    point: ArrayLike,
) -> float:
    """返回 ``points`` 到平面的垂直距离均方根（RMS）。

    这个数值说明了平面拟合是否*可信*。SVD 总会给出一个答案，包括对那些完全不共面
    的点集（一丛灌木、一堆揉皱的袋子）；而 RMSE 正是区分「地面」与「采样环里恰好
    存在的东西」的依据。请把它与传感器自身的噪声下限相比较——结构光相机在 1 m 处
    为几毫米，在 5 m 处为几厘米——并拒绝高于该下限的拟合。

    参数：
        points: ``(N, 3)`` 的数组类对象，含 ``N >= 1`` 个有限点。
        normal: ``(3,)`` 平面法线，非零。
        point: ``(3,)`` 平面上的任意一点。

    返回：
        均方根残差，非负 ``float``，单位为 ``points`` 的单位。

    异常：
        TypeError: 若某个参数不是数组类对象。
        ValueError: 若 ``points`` 为空或格式不正确，或某个向量无效。

    示例：
        >>> import numpy as np
        >>> pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.02], [0.0, 1.0, -0.02]])
        >>> round(plane_fit_rmse(pts, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]), 6)
        0.01633
    """
    residuals = plane_residuals(points, normal, point)
    if residuals.size == 0:
        raise ValueError(
            "points must hold at least one point to compute an RMSE, got an "
            "empty (0, 3) array"
        )
    return float(np.sqrt(np.mean(np.square(residuals))))


def point_to_plane_distance(
    p: ArrayLike,
    normal: ArrayLike,
    point: ArrayLike,
) -> float:
    """返回单个点到平面的绝对垂直距离。

    参数：
        p: ``(3,)`` 查询点。
        normal: ``(3,)`` 平面法线，非零；会在内部被归一化。
        point: ``(3,)`` 平面上的任意一点。

    返回：
        距离，非负 ``float``，单位为 ``p`` 的单位。

    异常：
        TypeError: 若某个参数不是数组类对象。
        ValueError: 若某个向量不含 3 个有限数值，或 ``normal`` 是零向量。

    示例：
        >>> point_to_plane_distance([1.0, 2.0, 3.0], [0.0, 0.0, 2.0],
        ...                         [0.0, 0.0, 1.0])
        2.0
    """
    query = _as_vector3(p, "p")
    return float(abs(plane_residuals(query.reshape(1, 3), normal, point)[0]))


# -- 检测的物理尺寸 ----------------------------------------


def bbox_3d_area(
    depth: Any,
    bbox: Any,
    intrinsics: Any,
    **depth_kwargs: Any,
) -> float | None:
    """返回包围框所张成的物理面积（单位 m^2）。

    包围框的四个角被转换为度量深度（使用与其他地方相同的鲁棒加窗采样器
    :func:`reusable_model.vision.depth.sample_depth`），反投影到相机坐标系，再由所得的空间
    四边形沿一条对角线拆成两个三角形，把它们的面积（``0.5 * |AB x AC|``）相加。
    拆分是必要的，因为四个角一般*不*共面：它们位于一个具有有限厚度、且在透视下
    被观测的物体上。

    检测器已经报告了像素面积，何必多此一举？因为像素面积度量的是*表观*尺寸，它把
    物体有多大和物体有多远混为一谈。0.5 m 处的瓶盖和 5 m 处的板条箱可能产生相同的
    框。物理面积使得近处的小物体和远处的大物体可以直接比较，而这正是任何基于尺寸的
    决策所需要的——「这片垃圾是否大到值得捡」「它能否放进手指之间」「它是叶子还是
    袋子」。它还能使阈值成为物体本身的属性而非相机安装方式的属性，因此同一常量在
    更换镜头或改变相机高度后依然成立。

    参数：
        depth: 原始深度值构成的深度图。
        bbox: 以**深度图**像素为单位的 ``(x1, y1, x2, y2)``；末尾的额外元素会被
            忽略。
        intrinsics: 与深度图匹配的 :class:`reusable_model.vision.pinhole.Intrinsics` 类对象。
            除非在 ``depth_kwargs`` 中被覆盖，否则使用其 ``depth_scale``。
        **depth_kwargs: 原样转发给 :func:`reusable_model.vision.depth.sample_depth`
            （``window``、``statistic``、``min_depth``、``max_depth``、
            ``min_valid_pixels`` 等）。

    返回：
        面积（单位 m^2），以 ``float`` 表示；当框退化/位于图像之外，或**任一**角点
        没有有效深度时返回 ``None``。返回 ``None`` 而非部分面积，是因为四边形的三个
        角并不能确定其大小：静默地只报告那个三角形会使结果低估约一半，并以一个错误
        的数值通过 ``> min_area`` 判定。

    异常：
        TypeError: 若 ``depth``/``bbox`` 格式不正确、内参格式不正确，或某个
            ``depth_kwargs`` 值的类型错误。
        ValueError: 若深度图不是二维/为空、``bbox`` 含少于 4 个有限数值，或某个
            采样器参数超出范围。

    示例：
        >>> import numpy as np
        >>> intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
        >>> flat = np.full((480, 640), 2000, dtype=np.uint16)     # wall at 2 m
        >>> area = bbox_3d_area(flat, (270, 190, 370, 290), intr, window=1)
        >>> round(area, 6)                                        # 0.4 m x 0.4 m
        0.16
        >>> hole = np.zeros((480, 640), dtype=np.uint16)
        >>> bbox_3d_area(hole, (270, 190, 370, 290), intr, window=1) is None
        True
    """
    arr = _as_depth(depth)
    height, width = arr.shape
    x0, y0, x1, y1 = _bbox_rect(bbox, height, width)
    if x1 <= x0 or y1 <= y0:
        logger.debug("bbox_3d_area: bbox %s is empty inside the image", bbox)
        return None

    kwargs = dict(depth_kwargs)
    kwargs.setdefault("depth_scale", float(getattr(intrinsics, "depth_scale", 1000.0)))

    # 角点是像素*索引*：较小的一角已被 _bbox_rect 钳制到图像内，较大的一角是
    # 开区间端点，会被拉回到最后一列/行，使 sample_depth（对图像外坐标会抛异常）
    # 总能见到一个合法像素。
    u_hi = min(x1, width - 1)
    v_hi = min(y1, height - 1)
    corners = (
        (x0, y0),
        (u_hi, y0),
        (u_hi, v_hi),
        (x0, v_hi),
    )
    points: list[np.ndarray] = []
    for u, v in corners:
        depth_m = sample_depth(arr, float(u), float(v), **kwargs)
        if depth_m is None:
            logger.debug(
                "bbox_3d_area: no valid depth at corner (%d, %d); cannot size "
                "the box", u, v,
            )
            return None
        points.append(pixel_to_3d(float(u), float(v), depth_m, intrinsics))

    return _triangle_area(points[0], points[1], points[2]) + _triangle_area(
        points[0], points[2], points[3]
    )


# -- 形状分析 -------------------------------------------------------


def fit_ellipse_to_points(
    points_2d: ArrayLike,
    *,
    min_points: int = 5,
) -> tuple[float, float, float] | None:
    """对二维点集拟合椭圆，并返回其轴长与朝向。

    在拟合之前，点会被归约为其**凸包**。原因有二：``cv2.fitEllipse`` 是对*每个*
    输入点做代数最小二乘拟合，因此稠密的内部（一个填充的分割掩码会贡献成千上万个
    根本不在边界上的点）会使二次曲线偏向团块的质心分布而非其轮廓；而凸包恰恰是约束
    边界的那个点集。对凸包拟合还使开销与掩码分辨率无关。

    参数：
        points_2d: ``(N, 2)`` 的数组类对象，元素为有限的平面坐标，坐标系由调用方
            决定（地面上的米、图像中的像素——返回的轴长沿用相同单位）。
        min_points: 所需的最少点数。小于 5 的值会被提升到 5，因为二次曲线有五个
            自由度，而 ``cv2.fitEllipse`` 拒绝更少的点。

    返回：
        ``(major, minor, major_angle_rad)``，其中 ``major >= minor > 0`` 是
        OpenCV 所报告的**完整**轴长（半轴的两倍；通常只有它们的比值重要），
        ``major_angle_rad`` 是*长*轴朝向的弧度值，从点坐标系的 ``+x`` 轴逆时针
        度量，并归一化到 ``[0, 2*pi)``。当给出的点少于 ``min_points``、凸包退化、
        OpenCV 拟合失败，或某个轴坍缩为零时返回 ``None``。

    异常：
        ImportError: 若未安装 OpenCV。
        TypeError: 若 ``points_2d`` 不是数组类对象，或 ``min_points`` 不是整数。
        ValueError: 若 ``points_2d`` 的形状不是 ``(N, 2)``，或含非有限坐标。

    示例：
        >>> import numpy as np
        >>> t = np.linspace(0.0, 2.0 * np.pi, 60, endpoint=False)
        >>> pts = np.column_stack((0.30 * np.cos(t), 0.10 * np.sin(t)))
        >>> major, minor, angle = fit_ellipse_to_points(pts)
        >>> round(major / minor, 6)                 # 轴比精确成立
        3.0
        >>> fit_ellipse_to_points(pts[:4]) is None
        True
    """
    cv2 = _require_cv2()
    arr = _as_points(points_2d, 2, "points_2d")
    required = max(_count_int(min_points, "min_points"), _CV2_FIT_ELLIPSE_MIN_POINTS)
    if arr.shape[0] < required:
        logger.debug(
            "fit_ellipse_to_points: %d point(s) < min_points=%d; no ellipse",
            arr.shape[0], required,
        )
        return None

    hull = cv2.convexHull(arr.astype(np.float32).reshape(-1, 1, 2))
    if hull is None or len(hull) < _CV2_FIT_ELLIPSE_MIN_POINTS:
        logger.debug(
            "fit_ellipse_to_points: convex hull has %d point(s); cannot fit a "
            "conic", 0 if hull is None else len(hull),
        )
        return None

    try:
        (_, _), (width, height_axes), angle_deg = cv2.fitEllipse(hull)
    except cv2.error as exc:
        logger.debug("fit_ellipse_to_points: cv2.fitEllipse failed: %s", exc)
        return None
    if width < _MIN_ELLIPSE_AXIS or height_axes < _MIN_ELLIPSE_AXIS:
        logger.debug(
            "fit_ellipse_to_points: degenerate ellipse axes (%.3e, %.3e)",
            width, height_axes,
        )
        return None

    # OpenCV 的角度属于旋转矩形的*宽度*轴，因此当高度反而是较长的那一条时，
    # 长轴与其相差 90 度。
    if width >= height_axes:
        return float(width), float(height_axes), math.radians(float(angle_deg)) % (2.0 * math.pi)
    return (
        float(height_axes),
        float(width),
        math.radians(float(angle_deg) + 90.0) % (2.0 * math.pi),
    )


def elongation_from_points(
    points_2d: ArrayLike,
    *,
    min_points: int = 5,
) -> tuple[float, float | None]:
    """返回形状的伸长率（长宽比）与长轴方向。

    关键之处在于你喂入的是*哪些*点。在**图像空间**中对掩码轮廓拟合椭圆，度量的是
    物体沿视线方向投影出的轮廓，而透视会拉伸该轮廓：一个平放在地面上的圆盘，从上方
    1 m、前方 1 m 的相机看去，会呈现为一个远边被压缩、近边被扩张的椭圆。此时恢复出的
    「伸长率」是相机位姿的属性，而非物体的属性，并且会随物体远离或靠近机器人而增大
    ——这恰恰是你不想在形状描述子上看到的行为，因为这类描述子用于判断「这是不是我
    该沿其轴向抓取的长棍，还是我该横向抓取的紧凑瓶子？」。

    解决办法是在物体真正所在的场所测量其形状：反投影深度点，把它们落到拟合出的地面
    平面上（即只保留它们在某个度量坐标系中的两个水平坐标），再对*这些*点拟合椭圆。
    结果便以米为单位，不随距离和相机倾角变化，而长轴角度是世界中一个真实的方向，
    可以转换为抓取偏航角。

    参数：
        points_2d: ``(N, 2)`` 的数组类对象，元素为平面坐标。请传入度量地面平面坐标
            （见上文）；像素坐标也可用，但会带有刚才描述的透视畸变。
        min_points: 所需的最少点数，与 :func:`fit_ellipse_to_points` 一样提升到 5。

    返回：
        ``(aspect, major_angle)``，其中 ``aspect = major / minor >= 1.0``，
        ``major_angle`` 是长轴朝向的弧度值（或当 ``aspect < 1.0`` 时为 ``None``；
        由于 :func:`fit_ellipse_to_points` 会对轴排序，成功的拟合不会出现这种情况
        ——该守卫是为了让传入手工轴长的调用方不会得到一个属于*短*轴的方向）。
        完全无法拟合的形状返回 ``(1.0, None)``，即「没有伸长的证据」：缺失的测量
        以中性值报告而非抛出异常，因为调用方会对长宽比做阈值判断，而缺失的测量绝
        不能被误判为细长物体。

    异常：
        ImportError: 若未安装 OpenCV。
        TypeError: 若 ``points_2d`` 不是数组类对象，或 ``min_points`` 不是整数。
        ValueError: 若 ``points_2d`` 的形状不是 ``(N, 2)``，或含非有限坐标。

    示例：
        >>> import numpy as np
        >>> t = np.linspace(0.0, 2.0 * np.pi, 60, endpoint=False)
        >>> stick = np.column_stack((0.30 * np.cos(t), 0.10 * np.sin(t)))
        >>> aspect, angle = elongation_from_points(stick)
        >>> round(aspect, 6)
        3.0
        >>> elongation_from_points(stick[:3])
        (1.0, None)
    """
    fitted = fit_ellipse_to_points(points_2d, min_points=min_points)
    if fitted is None:
        return 1.0, None
    major, minor, major_angle = fitted
    if minor <= 0.0:
        return 1.0, None
    aspect = major / minor
    return aspect, (major_angle if aspect >= 1.0 else None)
