"""从 RGB-D 帧生成点云：采样、滤波、变换与导出。

深度图是一张*图片*；机器人需要的是一*组障碍物*。本模块包含从前者到后者的短流程：

1. 将深度缓冲区**反投影**为度量三维点（:func:`backproject_depth`），
   采样密度按任务选择（:func:`grid_sample` 用于实时避障，
   :func:`random_sample` 用于配准或形状处理）；
2. **精简**点云（:func:`voxel_downsample`），使其规模取决于场景体积而非相机分辨率；
3. 将其**滤波**为真正重要的部分（:func:`filter_by_height`、
   :func:`filter_by_horizontal_distance`）；
4. 将其**变换**到规划器所用的坐标系（:func:`transform_points`、
   :func:`transform_points_by`）；
5. 可选地**估计法线**（:func:`compute_normals`）并**导出**以供检查
   （:func:`write_ply`）。

:class:`PointCloudProcessor` 把第 1-5 步封装在一个对象之后，该对象携带内参与深度
范围限制，而这正是节点或脚本实际想要持有的东西。

有两处约定值得先说明：

* 采样器输出时点位于**相机光学坐标系**（:mod:`reusable_model.vision.pinhole`），之后位于
  调用方将其变换到的任意坐标系。高度与距离滤波器假定的是*机器人/世界*坐标系
  （``z`` 向上），因此请在滤波前先做变换。
* 无效深度像素会被**丢弃**，绝不用零填充或插值。深度为零会反投影到 ``(0, 0, 0)``，
  即恰好位于相机中心的一个幽灵点，它会通过所有距离滤波，最终作为机器人自身位置处
  一个永久“被占据”的栅格进入障碍物集合。丢弃该像素是唯一安全的选择。

依赖：导入时依赖 :mod:`numpy`。``scipy`` 由 :func:`compute_normals` 惰性导入
（用于 k 近邻搜索），而 ``cv2``/ROS 完全不被使用，因此本模块在只安装了 numpy 的
机器上也能工作。
"""

from __future__ import annotations

import logging
import math
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

from .depth import valid_mask
from .pinhole import Intrinsics, pixel_to_3d_batch

logger = logging.getLogger(__name__)

__all__ = [
    "backproject_depth",
    "grid_sample",
    "random_sample",
    "voxel_downsample",
    "filter_by_height",
    "filter_by_horizontal_distance",
    "transform_points",
    "transform_points_by",
    "compute_normals",
    "write_ply",
    "PointCloudProcessor",
]

#: :meth:`PointCloudProcessor.from_rgbd` 接受的采样策略。
SAMPLING_MODES: tuple[str, ...] = ("grid", "random", "full")

#: 在单次 ``write`` 调用前缓存的 ASCII PLY 顶点行数。逐行写入在大点云上会被系统
#: 调用开销主导；而一次性写入整个文件又会构造出比点云本身还大的字符串。
_PLY_ASCII_CHUNK: int = 8192

#: 在 :func:`voxel_downsample` 中可打包进 ``int64`` 键而不冒溢出风险的最大栅格索引。
_MAX_PACKED_KEY: int = 1 << 62


def _require_scipy_spatial() -> Any:
    """惰性导入 ``scipy.spatial``，并给出可操作的错误信息。

    返回：
        ``scipy.spatial`` 模块。

    异常：
        ImportError: 若未安装 SciPy。
    """
    try:
        import scipy.spatial as spatial  # noqa: PLC0415 - 有意惰性导入
    except ImportError as exc:  # pragma: no cover - 取决于运行环境
        raise ImportError(
            "this function needs SciPy for the k-nearest-neighbour search; "
            "install it with `pip install scipy`"
        ) from exc
    return spatial


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
    """将计数类参数（步长、采样数）转换为 ``int``。

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


def _optional_float(value: Any, name: str) -> float | None:
    """将可选边界转换为有限 ``float``，``None`` 原样通过。

    参数：
        value: ``None`` 或候选数值。
        name: 用于错误信息的参数名。

    返回：
        当 ``value`` 为 ``None`` 时返回 ``None``，否则返回该值转换后的有限 ``float``。

    异常：
        TypeError: 若 ``value`` 既不是 ``None`` 也不是实数。
        ValueError: 若 ``value`` 为 NaN 或无穷大。
    """
    if value is None:
        return None
    return _finite_float(value, name)


def _as_points(points: Any, dim: int, name: str) -> np.ndarray:
    """校验点集，并以二维 ``float64`` 数组返回。

    *空*可迭代对象会被接受并重塑为 ``(0, dim)``，这样各滤波器可以串联作用在一个
    被更早阶段清空的点云上，而无需每个调用方都特殊处理。

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


def _as_vector(value: Any, dim: int, name: str) -> np.ndarray:
    """将 ``value`` 转换为由 ``dim`` 个元素组成的有限 ``float64`` 向量。

    参数：
        value: 任何恰好含 ``dim`` 个元素的数组类对象。
        dim: 所需长度。
        name: 用于错误信息的参数名。

    返回：
        ``(dim,)`` 的 ``float64`` 数组。

    异常：
        TypeError: 若 ``value`` 不是数组类对象。
        ValueError: 若其不含恰好 ``dim`` 个有限数值。
    """
    try:
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"{name} must be array-like with {dim} elements, got {value!r}"
        ) from exc
    if arr.size != dim:
        raise ValueError(
            f"{name} must have exactly {dim} elements, got {arr.size}: {value!r}"
        )
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return arr


def _as_depth(depth: Any, name: str = "depth") -> np.ndarray:
    """校验深度缓冲区，并以二维 ``float64`` 数组返回。

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
        min_depth: 最小合理距离，单位为米。
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


def _as_rng(seed: Any) -> np.random.Generator:
    """将种子参数转换为 :class:`numpy.random.Generator`。

    既接受预构建的生成器又接受整数，正是让随机采样器可用于可复现流程的原因：调用方
    既可以交出自己拥有的生成器（使整次运行共享一条随机流），也可以给一个整数
    （使单次调用完全可复现）。

    参数：
        seed: ``None`` 表示新建一个不可预测的生成器；``int`` 或
            :class:`numpy.random.SeedSequence` 表示可复现的生成器；或一个已有的
            :class:`numpy.random.Generator`。

    返回：
        一个 :class:`numpy.random.Generator`。

    异常：
        TypeError: 若 ``seed`` 不属于任何可接受的类型（``bool`` 被拒绝，因
            ``True`` 会静默地变成种子 ``1``）。
    """
    if isinstance(seed, np.random.Generator):
        return seed
    if seed is None:
        return np.random.default_rng()
    if isinstance(seed, np.random.SeedSequence):
        return np.random.default_rng(seed)
    if isinstance(seed, (int, np.integer)) and not isinstance(seed, bool):
        return np.random.default_rng(int(seed))
    raise TypeError(
        f"seed must be None, an int, a numpy.random.SeedSequence or a "
        f"numpy.random.Generator, got {seed!r} ({type(seed).__name__})"
    )


def _require_intrinsics(intrinsics: Any) -> Any:
    """校验 ``intrinsics`` 是否表现得像 :class:`Intrinsics`。

    采用鸭子类型（而非 ``isinstance``）可使这些辅助函数适用于任何暴露
    ``fx``/``fy``/``cx``/``cy`` 的对象，包括直接从文件路径加载的 pinhole 模块的
    第二份副本。

    参数：
        intrinsics: 候选的内参对象。

    返回：
        原封不动的同一对象。

    异常：
        TypeError: 若缺少必需属性。
    """
    missing = [name for name in ("fx", "fy", "cx", "cy") if not hasattr(intrinsics, name)]
    if missing:
        raise TypeError(
            f"intrinsics must expose {missing} (got {type(intrinsics).__name__}: "
            f"{intrinsics!r}); expected a pinhole.Intrinsics-like object"
        )
    return intrinsics


def _as_colors(colors: Any, count: int, name: str = "colors") -> np.ndarray:
    """校验逐点颜色数组，并以 ``(N, 3) uint8`` 返回。

    浮点输入被解释为归一化的 ``[0, 1]`` 通道并缩放到 ``[0, 255]``；整数输入被钳制
    到 ``[0, 255]``。这与每个图像库既有的约定一致，因此同时接受两者可避免一类
    “我的点云为什么是黑的？”的 bug。

    参数：
        colors: ``(N, 3)`` 或 ``(N, 4)`` 的数组类对象（alpha 通道会被丢弃）。
        count: 所需行数，即点数。
        name: 用于错误信息的参数名。

    返回：
        ``(N, 3)`` 的 ``uint8`` 数组。

    异常：
        TypeError: 若 ``colors`` 不是数组类对象。
        ValueError: 若其不是二维且列数为 3 或 4，或其行数与 ``count`` 不匹配。
    """
    try:
        arr = np.asarray(colors)
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"{name} must be an array-like (N, 3) or (N, 4) image, got "
            f"{type(colors).__name__}: {exc}"
        ) from exc
    if arr.ndim != 2 or arr.shape[1] not in (3, 4):
        raise ValueError(
            f"{name} must have shape (N, 3) or (N, 4), got {arr.shape} from "
            f"{np.asarray(colors).tolist()!r}"
        )
    if arr.shape[0] != count:
        raise ValueError(
            f"{name} must hold one colour per point: got {arr.shape[0]} colours "
            f"for {count} points"
        )
    rgb = arr[:, :3]
    if np.issubdtype(rgb.dtype, np.floating):
        peak = float(np.max(rgb)) if rgb.size else 0.0
        rgb = rgb * 255.0 if peak <= 1.0 else rgb
    return np.clip(rgb, 0.0, 255.0).astype(np.uint8)


def _as_path(path: Any, name: str) -> str:
    """校验文件系统路径参数，并以 ``str`` 返回。

    参数：
        path: 字符串或任意 :class:`os.PathLike`（例如 ``pathlib.Path``）。
        name: 用于错误信息的参数名。

    返回：
        以非空字符串表示的路径。

    异常：
        TypeError: 若 ``path`` 既不是字符串也不是路径类对象。
        ValueError: 若其为空或仅含空白字符。
    """
    candidate = path
    if hasattr(candidate, "__fspath__"):
        candidate = candidate.__fspath__()
    if not isinstance(candidate, str):
        raise TypeError(
            f"{name} must be a str or os.PathLike path, got {path!r} "
            f"({type(path).__name__})"
        )
    if not candidate.strip():
        raise ValueError(f"{name} must be a non-empty path, got {path!r}")
    return candidate


# -- 反投影 --------------------------------------------------------------


def backproject_depth(
    depth: Any,
    intrinsics: Any,
    *,
    depth_scale: float = 1000.0,
    min_depth: float = 0.1,
    max_depth: float = 10.0,
    stride: int = 1,
) -> tuple[np.ndarray, np.ndarray]:
    """将深度图转换为相机光学坐标系下的度量点云。

    每个被采样的像素都经由针孔模型映射
    （:func:`reusable_model.vision.pinhole.pixel_to_3d_batch`），因此投影数学只存在于一处。
    原始深度落在 ``[min_depth, max_depth]`` 之外的像素——包括无处不在的 ``0``
    “无回波”哨兵——会被**丢弃**。

    在这里，丢弃而不是零填充是重要的决策。深度为零会反投影到 ``(0, 0, 0)``，即
    恰好位于相机中心的一个点。这样的幽灵点会通过所有下游距离滤波（它就在原点，
    因而“很近”），落入障碍物集合，并表现为机器人正上方一个永久被占据的栅格——
    这会完全阻止机器人移动，而且是静默发生的。用邻居插值补洞也好不到哪去：跨越
    物体边缘插值会生成一个从未被测量过的表面，而那里恰恰最需要精度。

    参数：
        depth: 原始深度值构成的深度图（默认 ``uint16`` 毫米）。
        intrinsics: 与深度图相匹配的 :class:`reusable_model.vision.pinhole.Intrinsics` 类对象。
        depth_scale: 每米对应的原始深度单位数（``metres = raw / depth_scale``）。
        min_depth: 可接受的最小距离，单位为米。
        max_depth: 可接受的最大距离，单位为米。
        stride: 两个方向上的像素步长；``1`` 表示保留每个像素。

    返回：
        ``(points, pixel_coords)``，其中 ``points`` 是光学坐标系下以米为单位的
        ``(N, 3)`` ``float32`` 数组，``pixel_coords`` 是它们来源的 ``(u, v)`` 像素
        构成的 ``(N, 2)`` ``int32`` 数组。之所以返回像素坐标，是因为给点云着色、
        计算图像空间法线或查询掩码都需要这次往返。当图像中没有任何有效测量值时，
        两个数组的 ``N == 0``。

    异常：
        TypeError: 若 ``depth`` 不是数组类对象、内参格式不正确，或 ``stride`` 不是
            整数。
        ValueError: 若深度图不是二维/为空、深度范围无效，或 ``stride`` 小于 1。

    示例：
        >>> import numpy as np
        >>> intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
        >>> depth = np.zeros((4, 4), dtype=np.uint16)
        >>> depth[1, 1] = 2000                       # one pixel at 2 m
        >>> points, pixels = backproject_depth(depth, intr)
        >>> points.shape, pixels.shape, points.dtype
        ((1, 3), (1, 2), dtype('float32'))
        >>> pixels[0].tolist()
        [1, 1]
        >>> backproject_depth(np.zeros((4, 4), dtype=np.uint16), intr)[0].shape
        (0, 3)
    """
    arr = _as_depth(depth)
    _require_intrinsics(intrinsics)
    scale = _positive_float(depth_scale, "depth_scale")
    lo, hi = _check_limits(min_depth, max_depth)
    step = _count_int(stride, "stride")

    rows = np.arange(0, arr.shape[0], step)
    cols = np.arange(0, arr.shape[1], step)
    if rows.size == 0 or cols.size == 0:
        return (
            np.empty((0, 3), dtype=np.float32),
            np.empty((0, 2), dtype=np.int32),
        )

    raw = arr[np.ix_(rows, cols)]
    mask = valid_mask(raw, depth_scale=scale, min_depth=lo, max_depth=hi) & (raw > 0.0)
    uu, vv = np.meshgrid(cols, rows)
    sel_u = uu[mask]
    sel_v = vv[mask]
    if sel_u.size == 0:
        logger.debug(
            "backproject_depth: no pixel of the %dx%d image lies within "
            "[%.2f, %.2f] m", arr.shape[0], arr.shape[1], lo, hi,
        )
        return (
            np.empty((0, 3), dtype=np.float32),
            np.empty((0, 2), dtype=np.int32),
        )

    metres = raw[mask] / scale
    uv = np.column_stack((sel_u.astype(np.float64), sel_v.astype(np.float64)))
    # float32 将内存流量减半，而该点云只会被滤波器和一个 PLY 写入器消费；深度
    # 传感器提供的精度远低于 float32 保留的七位有效数字。
    points = pixel_to_3d_batch(uv, metres, intrinsics).astype(np.float32)
    pixels = np.column_stack((sel_u, sel_v)).astype(np.int32)
    return points, pixels


def grid_sample(
    depth: Any,
    intrinsics: Any,
    *,
    grid_step: int = 32,
    depth_scale: float = 1000.0,
    min_depth: float = 0.1,
    max_depth: float = 10.0,
) -> tuple[np.ndarray, np.ndarray]:
    """对均匀像素网格做反投影，即标准的避障点云。

    当点云要喂给代价地图或障碍物列表时，规则网格是合适的采样器：它均匀覆盖图像
    （不会有某个区域像随机采样那样偶然被过度代表），它是确定性的，且代价只是一次
    带步长的读取——无需扫描整个缓冲区检查有效性，也无需生成随机索引。

    在 1280x720 的帧上使用 ``grid_step=32`` 会访问 ``40 x 23 = 920`` 个像素，
    丢弃空洞后大约得到 **880 个点**。这是避障的甜点区：密到 32 像素的格子
    （在 3 米处、典型焦距下约 5 厘米）藏不住一条椅子腿，又小到整条流程——采样、
    滤波、插入栅格——都远低于一毫秒，因此能以相机帧率运行。把步长减半会使点数变为
    四倍，对导航几乎毫无收益；把它加倍则会开始漏掉细长障碍物。

    参数：
        depth: 原始深度值构成的深度图。
        intrinsics: :class:`reusable_model.vision.pinhole.Intrinsics` 类对象。
        grid_step: 网格在两个方向上的像素间距。
        depth_scale: 每米对应的原始深度单位数。
        min_depth: 可接受的最小距离，单位为米。
        max_depth: 可接受的最大距离，单位为米。

    返回：
        与 :func:`backproject_depth` 完全相同的 ``(points, pixel_coords)``
        （``float32`` ``(N, 3)`` 与 ``int32`` ``(N, 2)``）。

    异常：
        TypeError: 若 ``depth`` 不是数组类对象、内参格式不正确，或 ``grid_step``
            不是整数。
        ValueError: 若深度图不是二维/为空、深度范围无效，或 ``grid_step`` 小于 1。

    示例：
        >>> import numpy as np
        >>> intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
        >>> depth = np.full((64, 64), 1500, dtype=np.uint16)
        >>> points, pixels = grid_sample(depth, intr, grid_step=32)
        >>> points.shape, pixels.tolist()
        ((4, 3), [[0, 0], [32, 0], [0, 32], [32, 32]])
    """
    step = _count_int(grid_step, "grid_step")
    return backproject_depth(
        depth,
        intrinsics,
        depth_scale=depth_scale,
        min_depth=min_depth,
        max_depth=max_depth,
        stride=step,
    )


def random_sample(
    depth: Any,
    intrinsics: Any,
    *,
    num_samples: int = 3000,
    seed: Any = None,
    depth_scale: float = 1000.0,
    min_depth: float = 0.1,
    max_depth: float = 10.0,
) -> tuple[np.ndarray, np.ndarray]:
    """对*有效*像素的均匀随机子集做反投影。

    随机采样是面向形状的消费者所需要的：点到平面 ICP、RANSAC 平面提取和法线估计
    都对网格的规则结构敏感（网格会与场景中任何周期性结构发生混叠，并使 RANSAC
    抽样偏向整行），而均匀随机子集没有这种结构。*从有效像素中无放回抽样*——而不是
    抽取像素再丢弃无效者——才使 ``num_samples`` 有意义：一幅 40% 是空洞的帧仍会
    返回所请求的点数，而不是一个静默缩水的点云。

    参数：
        depth: 原始深度值构成的深度图。
        intrinsics: :class:`reusable_model.vision.pinhole.Intrinsics` 类对象。
        num_samples: 请求的点数；只有当图像的有效像素更少时才会返回更少。
        seed: ``None`` 表示不可预测的抽样；``int`` 或
            :class:`numpy.random.SeedSequence` 表示可复现的抽样；或一个已有的
            :class:`numpy.random.Generator`，用于与流程其余部分共享随机流。
        depth_scale: 每米对应的原始深度单位数。
        min_depth: 可接受的最小距离，单位为米。
        max_depth: 可接受的最大距离，单位为米。

    返回：
        与 :func:`backproject_depth` 完全相同的 ``(points, pixel_coords)``。点按图像
        扫描顺序而不是抽取顺序返回，因此输出不依赖随机抽取的内部布局，在不同种子间
        也保持可比较。

    异常：
        TypeError: 若 ``depth`` 不是数组类对象、内参格式不正确、``num_samples``
            不是整数，或 ``seed`` 的类型不受支持。
        ValueError: 若深度图不是二维/为空、深度范围无效，或 ``num_samples`` 小于 1。

    示例：
        >>> import numpy as np
        >>> intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
        >>> depth = np.full((16, 16), 1500, dtype=np.uint16)
        >>> depth[0, 0] = 0                                   # one hole
        >>> a = random_sample(depth, intr, num_samples=8, seed=42)[0]
        >>> b = random_sample(depth, intr, num_samples=8, seed=42)[0]
        >>> a.shape, bool(np.array_equal(a, b))
        ((8, 3), True)
    """
    arr = _as_depth(depth)
    _require_intrinsics(intrinsics)
    scale = _positive_float(depth_scale, "depth_scale")
    lo, hi = _check_limits(min_depth, max_depth)
    count = _count_int(num_samples, "num_samples")
    rng = _as_rng(seed)

    mask = valid_mask(arr, depth_scale=scale, min_depth=lo, max_depth=hi) & (arr > 0.0)
    candidates = np.flatnonzero(mask.ravel())
    if candidates.size == 0:
        logger.debug(
            "random_sample: the %dx%d image holds no pixel within [%.2f, %.2f] m",
            arr.shape[0], arr.shape[1], lo, hi,
        )
        return (
            np.empty((0, 3), dtype=np.float32),
            np.empty((0, 2), dtype=np.int32),
        )
    take = min(count, int(candidates.size))
    if take < candidates.size:
        chosen = rng.choice(candidates, size=take, replace=False)
    else:
        chosen = candidates
    chosen = np.sort(chosen)

    sel_v, sel_u = np.unravel_index(chosen, arr.shape)
    metres = arr[sel_v, sel_u] / scale
    uv = np.column_stack((sel_u.astype(np.float64), sel_v.astype(np.float64)))
    points = pixel_to_3d_batch(uv, metres, intrinsics).astype(np.float32)
    pixels = np.column_stack((sel_u, sel_v)).astype(np.int32)
    return points, pixels


# -- 精简与滤波 ----------------------------------------------------------


def voxel_downsample(points: ArrayLike, *, voxel_size: float) -> np.ndarray:
    """将点云稀释为至多每个立方体素一个点。

    体素网格是让点云规模取决于场景*体积*、而不是相机分辨率或最近墙面恰好有多近的
    标准做法：否则，机器人在接近一面墙时，其点数会恰好在它最无力承担额外计算时
    暴涨。

    实现上用向下取整除法把每个点哈希为整数栅格索引，将三个索引打包进一个 ``int64``
    键，再用 :func:`numpy.unique` 找出每个键首次出现的位置。这是向量化的——没有对
    点的 Python 循环——而打包键使其成为一次排序而非逐轴去重。若打包范围放不进
    ``int64``（对大场景而言体素荒谬地小），函数会回退为直接对索引三元组去重，
    而不是溢出成错误的键。

    保留的是每个体素中按输入顺序出现的**第一个**点，而不是质心。当点云要喂给障碍
    检查时，保留真实测量值很重要：一堵薄墙两侧点的质心是墙内部的点，即机器人随后
    可能驶入的自由空间。这也使该操作具有幂等性且开销低。

    参数：
        points: ``(N, 3)`` 的数组类对象，元素为有限点。
        voxel_size: 立方体素的边长，单位与 ``points`` 相同（度量点云则为米）。

    返回：
        ``(M, 3)`` 的 ``float64`` 数组，``M <= N``，包含每个被占据体素中按其在
        ``points`` 中出现顺序排列的第一个点。

    异常：
        TypeError: 若 ``points`` 不是数组类对象，或 ``voxel_size`` 不是实数。
        ValueError: 若 ``points`` 形状不是 ``(N, 3)`` 或含非有限坐标、``voxel_size <= 0``，
            或体素网格过大以致无法用 64 位整数索引。

    示例：
        >>> import numpy as np
        >>> pts = np.array([[0.10, 0.10, 0.10], [0.12, 0.11, 0.10],
        ...                 [5.00, 5.00, 5.00]])
        >>> voxel_downsample(pts, voxel_size=0.5).tolist()
        [[0.1, 0.1, 0.1], [5.0, 5.0, 5.0]]
        >>> voxel_downsample(np.empty((0, 3)), voxel_size=0.1).shape
        (0, 3)
    """
    pts = _as_points(points, 3, "points")
    size = _positive_float(voxel_size, "voxel_size")
    if pts.shape[0] == 0:
        return pts.copy()

    scaled = np.floor(pts / size)
    shifted = scaled - scaled.min(axis=0)
    if np.any(shifted > float(_MAX_PACKED_KEY)):
        raise ValueError(
            f"voxel_size={voxel_size!r} is too small for the extent of the "
            f"cloud: cell indices exceed 2**62, so they cannot be packed into "
            f"an int64 key"
        )
    cells = shifted.astype(np.int64)
    nx = int(cells[:, 0].max()) + 1
    ny = int(cells[:, 1].max()) + 1
    nz = int(cells[:, 2].max()) + 1

    if nx * ny * nz >= _MAX_PACKED_KEY:
        _, first = np.unique(cells, axis=0, return_index=True)
    else:
        key = (cells[:, 0] * ny + cells[:, 1]) * nz + cells[:, 2]
        _, first = np.unique(key, return_index=True)
    # np.unique 按键值顺序报告每个键首次出现的位置；对索引排序可恢复调用方的点序。
    return pts[np.sort(first)].copy()


def filter_by_height(
    points: ArrayLike,
    *,
    z_min: float | None = None,
    z_max: float | None = None,
) -> np.ndarray:
    """保留高度 ``z`` 落在某个区间内的点。

    它和 :func:`filter_by_horizontal_distance` 一起，把原始点云变为可用的障碍物
    集合。深度相机会返回它能看到的*一切*，而其中几乎所有内容与驾驶无关：地板不是
    障碍物，天花板或树冠对地面机器人不是障碍物，30 米外的墙*暂时*也不是障碍物。
    只保留机器人机体实际占据的那一层——例如小型地面平台取 ``z`` 在
    ``[0.05, 1.2]`` 米——就能去掉地板（否则它会主导点云，使每个栅格都被标记为
    被占据）、去掉机器人能从下方通过的高处几何，并留下一个“有点即真的挡路”的
    集合。

    边界为闭区间且可选；任意一侧为 ``None`` 表示“该侧不裁剪”，因此两个参数都不传
    时相当于一次复制操作。

    参数：
        points: ``(N, 3)`` 的数组类对象，元素为有限点，其第三个坐标为高度。这是
            *机器人/世界*坐标系约定：请先用 :func:`transform_points` 变换相机坐标系
            的点云，因为在光学坐标系中 ``z`` 是深度而非高度。
        z_min: 保留的最低高度；``None`` 表示无下界。
        z_max: 保留的最高高度；``None`` 表示无上界。

    返回：
        ``(M, 3)`` 的 ``float64`` 数组，``M <= N``（可能为 ``(0, 3)``）。

    异常：
        TypeError: 若 ``points`` 不是数组类对象，或某个边界不是实数。
        ValueError: 若 ``points`` 形状不是 ``(N, 3)``、某个边界非有限，或
            ``z_min > z_max``。

    示例：
        >>> import numpy as np
        >>> pts = np.array([[0.0, 0.0, -0.5], [0.0, 0.0, 0.5], [0.0, 0.0, 3.0]])
        >>> filter_by_height(pts, z_min=0.0, z_max=1.0).tolist()
        [[0.0, 0.0, 0.5]]
        >>> filter_by_height(pts, z_min=1.0).shape
        (1, 3)
    """
    pts = _as_points(points, 3, "points")
    lo = _optional_float(z_min, "z_min")
    hi = _optional_float(z_max, "z_max")
    if lo is not None and hi is not None and lo > hi:
        raise ValueError(
            f"z_min must be <= z_max, got z_min={z_min!r} and z_max={z_max!r}"
        )
    if pts.shape[0] == 0:
        return pts.copy()

    keep = np.ones(pts.shape[0], dtype=bool)
    if lo is not None:
        keep &= pts[:, 2] >= lo
    if hi is not None:
        keep &= pts[:, 2] <= hi
    return pts[keep]


def filter_by_horizontal_distance(
    points: ArrayLike,
    *,
    origin: ArrayLike = (0.0, 0.0),
    max_distance: float | None = None,
) -> np.ndarray:
    """保留距 ``origin`` 在水平半径内的点。

    距离只在 ``xy`` 平面上度量、忽略高度，因为那正是地面机器人运动的平面：3 米外的
    障碍物无论在脚踝还是胸口高度都重要，而 50 米外的点无论多高都完全无关。丢弃远场
    不仅是优化——深度噪声随距离增长（对双目和结构光大致是二次的），因此远处点是点云
    中最不可信的部分，也是最可能凭空捏造出并不存在的障碍物的部分。

    参数：
        points: ``(N, 3)`` 的数组类对象，元素为在 ``xy`` 平面即地面的坐标系中的
            有限点。
        origin: 圆盘的 ``(2,)`` 中心，坐标系与单位同 ``points``。默认为坐标系原点，
            通常即机器人基座。
        max_distance: 要保留的水平半径（闭区间）；``None`` 表示不裁剪。

    返回：
        ``(M, 3)`` 的 ``float64`` 数组，``M <= N``（可能为 ``(0, 3)``）。

    异常：
        TypeError: 若 ``points``/``origin`` 不是数组类对象，或 ``max_distance``
            不是实数。
        ValueError: 若 ``points`` 形状不是 ``(N, 3)``、``origin`` 不含两个有限
            数值，或 ``max_distance`` 为负。

    示例：
        >>> import numpy as np
        >>> pts = np.array([[1.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 9.0, 0.0]])
        >>> filter_by_horizontal_distance(pts, max_distance=2.0).tolist()
        [[1.0, 0.0, 0.0]]
        >>> filter_by_horizontal_distance(pts, origin=(10.0, 0.0),
        ...                               max_distance=0.5).shape
        (1, 3)
    """
    pts = _as_points(points, 3, "points")
    centre = _as_vector(origin, 2, "origin")
    radius = None if max_distance is None else _non_negative_float(max_distance, "max_distance")
    if pts.shape[0] == 0 or radius is None:
        return pts.copy()

    offsets = pts[:, :2] - centre
    distances = np.sqrt(np.einsum("ni,ni->n", offsets, offsets))
    return pts[distances <= radius]


# -- 坐标系变换 ----------------------------------------------------------


def transform_points(points: ArrayLike, matrix_4x4: ArrayLike) -> np.ndarray:
    """对点云施加齐次 4x4 变换。

    使用齐次形式（而非分开的旋转与平移），是因为这正是每个机器人软件栈已经产出的
    东西：一次 TF 查询、一次正运动学调用或一次 ICP 配准都返回单个 ``4x4`` 矩阵，
    而用一次乘法施加它既避免了额外分配，也避免了忘记平移的可能。

    参数：
        points: ``(N, 3)`` 的数组类对象，元素为有限点。
        matrix_4x4: ``(4, 4)`` 的数组类对象，其左上 ``3x3`` 块为旋转/缩放，最后一列
            （前三个元素）为平移。底行会被忽略，因此底行不完全等于 ``[0, 0, 0, 1]``
            的矩阵也能工作——但它同样不会被检查，请传入真正的刚体变换。

    返回：
        ``(N, 3)`` 的 ``float64`` 数组，元素为变换后的点。

    异常：
        TypeError: 若某参数不是数组类对象。
        ValueError: 若 ``points`` 形状不是 ``(N, 3)``、``matrix_4x4`` 形状不是
            ``(4, 4)``，或任一含非有限值。

    示例：
        >>> import numpy as np
        >>> shift = np.eye(4)
        >>> shift[0, 3] = 1.0                      # translate 1 m along x
        >>> transform_points(np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]]),
        ...                  shift).tolist()
        [[1.0, 0.0, 0.0], [2.0, 2.0, 3.0]]
    """
    pts = _as_points(points, 3, "points")
    try:
        matrix = np.asarray(matrix_4x4, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"matrix_4x4 must be an array-like (4, 4) matrix, got {matrix_4x4!r}"
        ) from exc
    if matrix.shape != (4, 4):
        raise ValueError(
            f"matrix_4x4 must have shape (4, 4), got {matrix.shape} from "
            f"{np.asarray(matrix_4x4).tolist()!r}"
        )
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"matrix_4x4 must contain only finite values, got {matrix.tolist()!r}")
    if pts.shape[0] == 0:
        return pts.copy()

    homogeneous = np.empty((pts.shape[0], 4), dtype=np.float64)
    homogeneous[:, :3] = pts
    homogeneous[:, 3] = 1.0
    return (homogeneous @ matrix.T)[:, :3]


def transform_points_by(
    points: ArrayLike,
    *,
    rotation: ArrayLike | None = None,
    translation: ArrayLike | None = None,
) -> np.ndarray:
    """施加分别给出的旋转和/或平移。

    当二者来自不同地方时（例如来自定位话题的航向角，以及来自 URDF 的杆臂），或只
    需要其中之一时（只传 ``translation`` 就是纯平移，这是刚性安装传感器常见的
    “相机到基座”情形），这很方便。

    先施加旋转再施加平移，即结果为 ``R @ p + t``，与齐次矩阵 ``[[R, t], [0, 1]]``
    的布局一致。

    参数：
        points: ``(N, 3)`` 的数组类对象，元素为有限点。
        rotation: ``(3, 3)`` 的数组类对象；``None`` 表示跳过旋转。
        translation: ``(3,)`` 的数组类对象；``None`` 表示跳过平移。

    返回：
        ``(N, 3)`` 的 ``float64`` 数组，元素为变换后的点（当两个参数均为 ``None``
        时返回副本）。

    异常：
        TypeError: 若某参数不是数组类对象。
        ValueError: 若 ``points`` 形状不是 ``(N, 3)``、``rotation`` 形状不是
            ``(3, 3)``、``translation`` 不含三个有限数值，或任一输入含非有限值。

    示例：
        >>> import numpy as np
        >>> rot_z90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        >>> transform_points_by(np.array([[1.0, 0.0, 0.0]]), rotation=rot_z90,
        ...                     translation=[0.0, 0.0, 2.0]).tolist()
        [[0.0, 1.0, 2.0]]
        >>> transform_points_by(np.array([[1.0, 2.0, 3.0]])).tolist()
        [[1.0, 2.0, 3.0]]
    """
    pts = _as_points(points, 3, "points")
    if rotation is None and translation is None:
        return pts.copy()
    if pts.shape[0] == 0:
        return pts.copy()

    out = pts
    if rotation is not None:
        try:
            matrix = np.asarray(rotation, dtype=np.float64)
        except (TypeError, ValueError) as exc:
            raise TypeError(
                f"rotation must be an array-like (3, 3) matrix, got {rotation!r}"
            ) from exc
        if matrix.shape != (3, 3):
            raise ValueError(
                f"rotation must have shape (3, 3), got {matrix.shape} from "
                f"{np.asarray(rotation).tolist()!r}"
            )
        if not np.all(np.isfinite(matrix)):
            raise ValueError(f"rotation must contain only finite values, got {matrix.tolist()!r}")
        out = out @ matrix.T
    if translation is not None:
        out = out + _as_vector(translation, 3, "translation")
    return out


# -- 法线 ----------------------------------------------------------------


def compute_normals(
    points: ArrayLike,
    *,
    k: int = 16,
    origin: ArrayLike | None = None,
) -> np.ndarray:
    """由每个点的 ``k`` 个最近邻估计其单位法线。

    对每个点用 KD-tree 找到邻域，去中心化后对其 ``3x3`` 协方差做特征分解；**最小**
    特征值对应的特征向量是方差最小的方向，对于局部平面的表面即为法线。这与
    :func:`reusable_model.vision.planes.fit_plane` 是同一种总体最小二乘平面拟合，只是逐邻域
    进行、并对所有邻域一次向量化完成（一次批量 :func:`numpy.linalg.eigh` 调用而非
    Python 循环），这正是它能在数千点上负担得起的原因。

    ``k=16`` 是一种折中：邻居更少，法线会跟随传感器噪声而非表面；邻居更多，则会
    在边缘处做平均——那里“真实”法线并不存在，而宽窗口会凭空造出一个既不属于哪一侧
    表面的法线。

    特征分解确定的是一条*直线*而非朝向，因此原始法线的符号是任意的，并会在相邻点
    之间翻转。这对任何与朝向相关的用途都毫无用处（渲染、平面合并、判断表面哪一侧是
    自由空间）。传入 ``origin``——传感器位置——可让每条法线都指向观察者，依据是深度
    相机看到的表面总是朝向相机：当 ``dot(n, origin - p) < 0`` 时把法线翻转。

    参数：
        points: ``(N, 3)`` 的数组类对象，含 ``N >= 3`` 个有限点。
        k: 邻域大小，至少为 3（一个平面需要三个点）。大于 ``N`` 的值会被钳制为
            ``N``。
        origin: ``(3,)`` 传感器位置，用于一致地定向法线；``None`` 表示保留任意的
            特征向量符号。

    返回：
        ``(N, 3)`` 的 ``float64`` 数组，每个输入点一个单位法线，顺序相同。协方差在
        数值上为零的邻域（所有点重合）会产生一行零，因为它不定义任何方向。

    异常：
        ImportError: 若未安装 SciPy。
        TypeError: 若 ``points``/``origin`` 不是数组类对象，或 ``k`` 不是整数。
        ValueError: 若 ``points`` 形状不是 ``(N, 3)`` 或少于 3 个点，或 ``k`` 小于 3。

    示例：
        >>> import numpy as np
        >>> grid = np.array([[x, y, 0.0] for x in range(4) for y in range(4)])
        >>> normals = compute_normals(grid, k=4, origin=[0.0, 0.0, -5.0])
        >>> normals.shape
        (16, 3)
        >>> np.allclose(normals[:, 2], -1.0)      # 全部朝向传感器
        True
    """
    spatial = _require_scipy_spatial()
    pts = _as_points(points, 3, "points")
    neighbours = _count_int(k, "k", minimum=3)
    if pts.shape[0] < 3:
        raise ValueError(
            f"points must hold at least 3 points to estimate a normal, got "
            f"{pts.shape[0]}: {np.asarray(points).tolist()!r}"
        )
    if neighbours > pts.shape[0]:
        logger.debug(
            "compute_normals: k=%d clamped to the %d available point(s)",
            neighbours, pts.shape[0],
        )
        neighbours = int(pts.shape[0])

    tree = spatial.cKDTree(pts)
    _, indices = tree.query(pts, k=neighbours)
    indices = np.atleast_2d(np.asarray(indices, dtype=np.int64))
    neighbourhood = pts[indices]                       # (N, k, 3)
    centered = neighbourhood - neighbourhood.mean(axis=1)[:, None, :]
    covariance = np.einsum("nki,nkj->nij", centered, centered) / float(max(neighbours - 1, 1))

    # eigh 按升序返回特征值，因此特征向量矩阵的第 0 列属于最小特征值。
    _, eigenvectors = np.linalg.eigh(covariance)
    normals = eigenvectors[:, :, 0]
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    # 长度为零的行意味着邻域完全退化；把它保持为零是诚实地报告“无法线”，
    # 而不是凭空造一个方向。
    normals = normals / np.where(lengths > 1e-12, lengths, 1.0)

    if origin is not None:
        sensor = _as_vector(origin, 3, "origin")
        towards_sensor = np.einsum("ni,ni->n", normals, sensor - pts)
        normals[towards_sensor < 0.0] *= -1.0
    return normals


# -- 导出 ----------------------------------------------------------------


def write_ply(
    path: Any,
    points: ArrayLike,
    colors: ArrayLike | None = None,
    *,
    binary: bool = False,
) -> None:
    """将点云写入 PLY 文件。

    PLY 是每个网格工具（MeshLab、CloudCompare、Open3D、Blender）都能读取的格式，
    因此这里写出的点云可以在不依赖本库任何东西的情况下被查看——这正是目的所在：
    通过*看*点云来调试感知故障，胜过在日志里读数字。没有使用任何第三方 PLY 写入器；
    ASCII 路径是纯字符串格式化，二进制路径是一次结构化数组导出。

    默认为 ``binary=False``（ASCII），因为文件用文本编辑器仍可读，也不会因字节序
    不匹配而损坏；对于大点云请使用 ``binary=True``，它既小好几倍、写入又快一个数量级。
    二进制输出为小端序，这基本上是所有消费者都期望的。

    参数：
        path: 目标文件路径（``str`` 或 ``os.PathLike``）。父目录**不会**被创建。
        points: ``(N, 3)`` 的数组类对象，元素为有限坐标。
        colors: 可选的 ``(N, 3)`` 或 ``(N, 4)`` 逐点颜色；浮点数按 ``[0, 1]`` 读取，
            整数按 ``[0, 255]`` 读取。
        binary: 写出 ``binary_little_endian`` 而不是 ``ascii``。

    返回：
        ``None``。

    异常：
        TypeError: 若 ``path`` 不是路径、``points``/``colors`` 不是数组类对象，或
            ``binary`` 不是布尔值。
        ValueError: 若 ``path`` 为空、``points`` 形状不是 ``(N, 3)``、``colors``
            形状不是 ``(N, 3)``/``(N, 4)`` 且 ``N`` 不匹配，或某个坐标非有限。
        OSError: 若文件无法打开或写入。

    示例：
        >>> import numpy as np, os, tempfile
        >>> target = os.path.join(tempfile.mkdtemp(), "cloud.ply")
        >>> write_ply(target, np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 2.0]]))
        >>> lines = open(target, encoding="ascii").read().splitlines()
        >>> lines[0], lines[1], lines[-2]
        ('ply', 'format ascii 1.0', '0.000000 0.000000 1.000000')
    """
    if not isinstance(binary, bool):
        raise TypeError(f"binary must be a bool, got {binary!r} ({type(binary).__name__})")
    pts = _as_points(points, 3, "points")
    target = _as_path(path, "path")
    cols = None if colors is None else _as_colors(colors, pts.shape[0])

    header = [
        "ply",
        f"format {'binary_little_endian' if binary else 'ascii'} 1.0",
        "comment written by reusable_model.vision.pointcloud",
        f"element vertex {pts.shape[0]}",
        "property float x",
        "property float y",
        "property float z",
    ]
    if cols is not None:
        header.extend(("property uchar red", "property uchar green", "property uchar blue"))
    header.append("end_header")

    with open(target, "wb") as handle:
        handle.write(("\n".join(header) + "\n").encode("ascii"))
        if pts.shape[0] == 0:
            return
        if binary:
            fields = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
            if cols is not None:
                fields.extend((("red", "<u1"), ("green", "<u1"), ("blue", "<u1")))
            block = np.empty(pts.shape[0], dtype=np.dtype(fields))
            block["x"], block["y"], block["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
            if cols is not None:
                block["red"], block["green"], block["blue"] = cols[:, 0], cols[:, 1], cols[:, 2]
            block.tofile(handle)
            return

        def _format(row: np.ndarray) -> str:
            if cols is None:
                return f"{row[0]:.6f} {row[1]:.6f} {row[2]:.6f}"
            return (
                f"{row[0]:.6f} {row[1]:.6f} {row[2]:.6f} "
                f"{row[3]} {row[4]} {row[5]}"
            )

        merged = pts if cols is None else np.hstack([pts, cols.astype(np.float64)])
        buffer: list[str] = []
        for row in merged:
            buffer.append(_format(row))
            if len(buffer) >= _PLY_ASCII_CHUNK:
                handle.write(("\n".join(buffer) + "\n").encode("ascii"))
                buffer.clear()
        if buffer:
            handle.write(("\n".join(buffer) + "\n").encode("ascii"))


# -- 门面 ----------------------------------------------------------------


class PointCloudProcessor:
    """把内参与深度范围限制同采样/滤波辅助函数捆绑在一起。

    本模块的自由函数都需要同样三项上下文——内参、深度缩放和一个合理深度范围——而
    在每个调用点都传它们，正是一条流程最终对同一相机出现两个不同 ``max_depth`` 的
    原因。把它们一次性保存在一个对象里可消除这类 bug，并提供一个记录默认值的地方：
    0.1 米 .. 10.0 米是典型手持/机器人深度相机的量程，更近的落在传感器盲区内，
    更远的则是噪声。

    处理器在调用之间不保留任何状态：没有帧缓存，因为以“同一幅图像”为键的缓存会在
    调用方就地修改其缓冲区的瞬间静默返回过期的几何——而相机驱动确实会这么做。

    属性：
        intrinsics: 每次调用都会使用的 :class:`reusable_model.vision.pinhole.Intrinsics` 类对象。
        depth_scale: 每米对应的原始深度单位数。
        min_depth: 可接受的最小距离，单位为米。
        max_depth: 可接受的最大距离，单位为米。
        grid_step: :meth:`from_rgbd` 在 ``'grid'`` 模式下的默认像素间距。
        num_samples: :meth:`from_rgbd` 在 ``'random'`` 模式下的默认点数。

    异常：
        TypeError: 若 ``intrinsics`` 未暴露 ``fx``/``fy``/``cx``/``cy``，或某个数值
            字段为非数值类型。
        ValueError: 若深度范围无效、``depth_scale <= 0``，或某个像素计数小于 1。

    示例：
        >>> import numpy as np
        >>> proc = PointCloudProcessor(Intrinsics(500.0, 500.0, 320.0, 240.0,
        ...                                       640, 480))
        >>> proc.grid_step, proc.min_depth, proc.max_depth
        (32, 0.1, 10.0)
        >>> depth = np.full((64, 64), 1500, dtype=np.uint16)
        >>> points, pixels, colors = proc.from_rgbd(None, depth)
        >>> points.shape, colors is None
        ((4, 3), True)
    """

    def __init__(
        self,
        intrinsics: Any,
        *,
        depth_scale: float | None = None,
        min_depth: float = 0.1,
        max_depth: float = 10.0,
        grid_step: int = 32,
        num_samples: int = 3000,
    ) -> None:
        """存储并校验共享上下文。

        参数：
            intrinsics: :class:`reusable_model.vision.pinhole.Intrinsics` 类对象。
            depth_scale: 每米对应的原始深度单位数；``None`` 表示当对象带有
                ``intrinsics.depth_scale`` 时采用之，否则用 1000.0（毫米，常见的
                ``uint16`` 约定）。
            min_depth: 可接受的最小距离，单位为米。
            max_depth: 可接受的最大距离，单位为米。
            grid_step: ``sampling='grid'`` 的默认像素间距。
            num_samples: ``sampling='random'`` 的默认点数。

        异常：
            TypeError: 若 ``intrinsics`` 格式不正确，或某个数值参数为非数值类型。
            ValueError: 若深度范围无效，或某个计数小于 1。
        """
        self.intrinsics = _require_intrinsics(intrinsics)
        if depth_scale is None:
            depth_scale = float(getattr(intrinsics, "depth_scale", 1000.0))
        self.depth_scale = _positive_float(depth_scale, "depth_scale")
        self.min_depth, self.max_depth = _check_limits(min_depth, max_depth)
        self.grid_step = _count_int(grid_step, "grid_step")
        self.num_samples = _count_int(num_samples, "num_samples")

    def _limits(self, overrides: dict[str, Any]) -> dict[str, float]:
        """将每次调用的深度范围覆盖值与实例默认值合并。

        三个深度键会从 ``overrides`` 中弹出，使剩下的内容可以被报告为未知参数；
        这些值本身的校验留给采样器，它们会抛出与往常相同的错误信息。

        参数：
            overrides: 调用时传入的关键字参数，就地修改。

        返回：
            含 ``depth_scale``、``min_depth`` 和 ``max_depth`` 的字典。
        """
        return {
            "depth_scale": overrides.pop("depth_scale", self.depth_scale),
            "min_depth": overrides.pop("min_depth", self.min_depth),
            "max_depth": overrides.pop("max_depth", self.max_depth),
        }

    def from_rgbd(
        self,
        rgb: Any,
        depth: Any,
        *,
        sampling: str = "grid",
        **kwargs: Any,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
        """从一帧 RGB-D 数据构建彩色点云。

        参数：
            rgb: 与深度分辨率匹配的彩色图像，形状为 ``(H, W, 3)`` 或
                ``(H, W, 4)``（BGR 或 RGB——通道会被原样复制，不做解释）。
                传入 ``None`` 表示跳过着色。
            depth: 原始深度值构成的深度图，形状为 ``(H, W)``。
            sampling: :data:`SAMPLING_MODES` 之一：

                * ``'grid'``——均匀像素栅格（:func:`grid_sample`）；这是默认值，
                  也是避障场景的正确选择，因为它确定性、均匀覆盖图像，
                  且只需一次跨步读取。
                * ``'random'``——均匀随机有效像素（:func:`random_sample`）；
                  更适合 ICP/RANSAC/法线相关工作，因为规则栅格可能与场景结构
                  产生混叠。
                * ``'full'``——每个像素（``stride=1`` 的
                  :func:`backproject_depth`）；仅用于离线工作，因为它会为每个
                  像素生成一个点。
            **kwargs: 每次调用的覆盖值。``depth_scale``、``min_depth`` 与
                ``max_depth`` 适用于所有模式；``grid_step`` 适用于 ``'grid'``，
                ``num_samples``/``seed`` 适用于 ``'random'``，``stride`` 适用于
                ``'full'``。

        返回：
            ``(points, pixel_coords, colors)``，其中 ``points`` 是相机光学坐标系下
            的 ``(N, 3)`` ``float32`` 点云，``pixel_coords`` 是其来源的 ``(N, 2)``
            ``int32`` 像素，``colors`` 是从 ``rgb`` 采样得到的 ``(N, 3)``
            ``uint8`` 颜色（当 ``rgb`` 为 ``None`` 时为 ``None``）。

        异常：
            TypeError: 若 ``depth``/``rgb`` 不是数组类对象，或某个数值覆盖值的
                类型错误。
            ValueError: 若 ``sampling`` 未知、传入了意外的关键字参数、深度图不是
                二维/为空、深度范围无效，或 ``rgb`` 与深度分辨率不匹配。

        示例：
            >>> import numpy as np
            >>> proc = PointCloudProcessor(Intrinsics(500.0, 500.0, 4.0, 4.0,
            ...                                       8, 8))
            >>> depth = np.full((8, 8), 2000, dtype=np.uint16)
            >>> rgb = np.zeros((8, 8, 3), dtype=np.uint8)
            >>> points, pixels, colors = proc.from_rgbd(rgb, depth,
            ...                                         sampling="full")
            >>> points.shape, colors.shape
            ((64, 3), (64, 3))
        """
        if not isinstance(sampling, str):
            raise TypeError(
                f"sampling must be a string, one of {SAMPLING_MODES}, got "
                f"{sampling!r} ({type(sampling).__name__})"
            )
        mode = sampling.strip().lower()
        if mode not in SAMPLING_MODES:
            raise ValueError(
                f"sampling must be one of {SAMPLING_MODES}, got {sampling!r}"
            )
        overrides = dict(kwargs)
        limits = self._limits(overrides)
        mode_kwargs: dict[str, Any] = {}
        if mode == "grid":
            mode_kwargs["grid_step"] = overrides.pop("grid_step", self.grid_step)
        elif mode == "random":
            mode_kwargs["num_samples"] = overrides.pop("num_samples", self.num_samples)
            mode_kwargs["seed"] = overrides.pop("seed", None)
        else:
            mode_kwargs["stride"] = overrides.pop("stride", 1)
        if overrides:
            raise ValueError(
                f"unexpected keyword argument(s) {sorted(overrides)!r} for "
                f"sampling={sampling!r}; 'grid' accepts grid_step, 'random' "
                f"accepts num_samples/seed, 'full' accepts stride, and all "
                f"three accept depth_scale/min_depth/max_depth"
            )

        if mode == "grid":
            points, pixels = grid_sample(depth, self.intrinsics, **limits, **mode_kwargs)
        elif mode == "random":
            points, pixels = random_sample(depth, self.intrinsics, **limits, **mode_kwargs)
        else:
            points, pixels = backproject_depth(
                depth, self.intrinsics, **limits, **mode_kwargs
            )

        colors = self._colors_at(rgb, depth, pixels) if rgb is not None else None
        return points, pixels, colors

    def _colors_at(self, rgb: Any, depth: Any, pixels: np.ndarray) -> np.ndarray:
        """查询每个被采样像素的颜色。

        此处彩色流与深度流必须共享同一分辨率：``pixels`` 是深度图坐标，用它们去
        索引尺寸不同的彩色图像会静默地采到错误的位置（越靠近图像角落错得越厉害）。
        分辨率不匹配会直接报错而非做重缩放，因为仅凭形状无法推断出正确的重缩放
        系数。

        参数：
            rgb: 彩色图像，``(H, W, 3)`` 或 ``(H, W, 4)``。
            depth: 深度图，其形状定义采样栅格。
            pixels: ``(N, 2)`` ``int32`` 深度图像素坐标。

        返回：
            ``(N, 3)`` ``uint8`` 颜色，通道顺序与 ``rgb`` 一致。

        异常：
            TypeError: 若 ``rgb`` 不是数组类对象。
            ValueError: 若 ``rgb`` 不是带 3/4 通道的三维数组，或其前两维与深度图
                不一致。
        """
        try:
            image = np.asarray(rgb)
        except (TypeError, ValueError) as exc:
            raise TypeError(
                f"rgb must be an array-like (H, W, 3) image, got {type(rgb).__name__}: {exc}"
            ) from exc
        if image.ndim != 3 or image.shape[2] not in (3, 4):
            raise ValueError(
                f"rgb must have shape (H, W, 3) or (H, W, 4), got {image.shape}"
            )
        depth_arr = _as_depth(depth)
        if image.shape[:2] != depth_arr.shape:
            raise ValueError(
                f"rgb and depth must share a resolution to be sampled with the "
                f"same pixel coordinates, got rgb {image.shape[:2]} and depth "
                f"{depth_arr.shape}; resize or align the streams first"
            )
        if pixels.shape[0] == 0:
            return np.empty((0, 3), dtype=np.uint8)
        return _as_colors(image[pixels[:, 1], pixels[:, 0], :], pixels.shape[0], "rgb")

    def to_obstacles(
        self,
        points: ArrayLike,
        *,
        z_min: float | None = 0.05,
        z_max: float | None = 1.2,
        origin: ArrayLike = (0.0, 0.0),
        max_distance: float | None = None,
    ) -> np.ndarray:
        """将世界坐标系下的点云精简为真正属于障碍物的点。

        导航栈需要的两级滤波正是如此：先剔除机器人下方的一切（地面）和上方的一切
        （天花板、枝叶、门框），再剔除远到无关紧要的一切。默认值描述的是一台小型
        地面平台——5 cm 的地面余量可吸收地面上的深度噪声而又不放行低矮障碍物，
        1.2 m 高于机身但低于天花板——而其中每一个都是参数，因为合适的数值完全
        取决于机器人本身。

        参数：
            points: ``(N, 3)`` 数组类对象，位于 ``z`` 向上的**世界/机器人**坐标系
                中。相机坐标系下的点云必须先经过 :func:`transform_points`。
            z_min: 保留的最低高度，``None`` 表示不设下界。
            z_max: 保留的最高高度，``None`` 表示不设上界。
            origin: 要保留的水平圆盘的 ``(2,)`` 中心。
            max_distance: 要保留的水平半径，``None`` 表示不做裁剪。

        返回：
            ``(M, 3)`` ``float64`` 障碍物点，可能为 ``(0, 3)``。

        异常：
            TypeError: 若 ``points``/``origin`` 不是数组类对象，或某个边界不是
                实数。
            ValueError: 若 ``points`` 的形状不是 ``(N, 3)``、某个边界非有限、
                ``z_min > z_max``，或 ``max_distance`` 为负数。

        示例：
            >>> import numpy as np
            >>> proc = PointCloudProcessor(Intrinsics(500.0, 500.0, 32.0,
            ...                                       32.0, 64, 64))
            >>> cloud = np.array([[1.0, 0.0, 0.5], [1.0, 0.0, -1.0],
            ...                   [50.0, 0.0, 0.5]])
            >>> proc.to_obstacles(cloud, max_distance=5.0).tolist()
            [[1.0, 0.0, 0.5]]
        """
        height_filtered = filter_by_height(points, z_min=z_min, z_max=z_max)
        return filter_by_horizontal_distance(
            height_filtered, origin=origin, max_distance=max_distance
        )

    def to_ply(
        self,
        path: Any,
        points: ArrayLike,
        colors: ArrayLike | None = None,
    ) -> None:
        """将点云写入 ASCII PLY 文件以供检查。

        这里默认使用 ASCII（而非 :func:`write_ply` 提供的二进制选项），因为本方法
        的目的是调试：ASCII 文件可以在机器人本机、通过 SSH 直接用文本编辑器打开，
        无需把它拷贝到任何地方。

        参数：
            path: 目标文件路径（``str`` 或 ``os.PathLike``）。
            points: ``(N, 3)`` 数组类对象，元素为有限坐标。
            colors: 可选的 ``(N, 3)``/``(N, 4)`` 逐点颜色。

        返回：
            ``None``。

        异常：
            TypeError: 若 ``path`` 不是路径，或某个数组格式不正确。
            ValueError: 若 ``path`` 为空、``points`` 的形状不是 ``(N, 3)``，
                或 ``colors`` 与点数不匹配。
            OSError: 若文件无法写入。

        示例：
            >>> import numpy as np, os, tempfile
            >>> proc = PointCloudProcessor(Intrinsics(500.0, 500.0, 4.0, 4.0,
            ...                                       8, 8))
            >>> target = os.path.join(tempfile.mkdtemp(), "obstacles.ply")
            >>> proc.to_ply(target, np.array([[0.0, 0.0, 1.0]]))
            >>> open(target, encoding="ascii").read().splitlines()[3]
            'element vertex 1'
        """
        write_ply(path, points, colors, binary=False)
