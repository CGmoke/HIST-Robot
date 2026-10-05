"""针孔相机模型：内参、反投影、投影与射线投射。

本模块实现了几乎每个 RGB-D 传感器驱动都会用到的教科书式针孔相机模型：

    u = fx * X / Z + cx
    v = fy * Y / Z + cy

以及它的逆运算（将一个像素加上一个度量深度反投影为三维点）。所有量都以
**相机光学坐标系**表示：

    * ``+x`` 指向图像右侧，
    * ``+y`` 指向图像下方，
    * ``+z`` 指向前方、射出镜头，即深度轴。

采用这一约定（而非机器人领域常见的“x 向前、y 向左、z 向上”的机体坐标系）
可以让上面的投影方程无需任何额外旋转即成立，因此所有标定工具和深度驱动输出
的都是这一约定。请在反投影*之后*再转换到机体/世界坐标系。

设计说明：

* 仅导入 ``numpy``；不依赖 OpenCV、SciPy 或 ROS。
* ``Intrinsics.from_ros_camera_info`` 有意接受一个*普通映射*，而不是
  ``sensor_msgs.msg.CameraInfo``。在此导入 ROS 消息类型会使本库在 ROS 工作
  空间之外无法使用，因此期望调用方自行提取所需的少数几个字段
  （``msg.width``、``msg.height``、``msg.k``）——参见下面的示例。
* 在自由函数的调用点处，内参采用鸭子类型（duck-typing）而非 ``isinstance``
  检查，因此由等价 dataclass（或测试中手工编写的替代品）生成的对象可以原样使用。
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

logger = logging.getLogger(__name__)

__all__ = [
    "Intrinsics",
    "pixel_to_3d",
    "pixel_to_3d_batch",
    "project",
    "project_batch",
    "ray_direction",
    "pixel_to_depth_pixel",
]


def _finite_float(value: Any, name: str) -> float:
    """将 ``value`` 转换为有限的 ``float``，否则携带该值抛出异常。

    参数：
        value: 调用方在需要实数处传入的任意值。
        name: 用于错误信息的参数名。

    返回：
        该值转换后的有限 Python ``float``。

    异常：
        TypeError: 若 ``value`` 无法转换为 ``float``（包括 ``bool``，因其
            ``True`` 会静默地表现为 ``1.0``、从而掩盖调用方的 bug，故一并拒绝）。
        ValueError: 若 ``value`` 为 NaN 或无穷大。
    """
    if isinstance(value, bool):
        raise TypeError(f"{name} must be a real number, got bool {value!r}")
    try:
        out = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"{name} must be a real number, got {value!r} ({type(value).__name__})"
        ) from exc
    if not math.isfinite(out):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return out


def _positive_int(value: Any, name: str) -> int:
    """将 ``value`` 转换为 ``int >= 1``，否则携带该值抛出异常。

    形如 ``720.0`` 的整值浮点数会被接受，因为分辨率常来自 YAML/JSON 配置，
    而在那里每个标量都是浮点数。

    参数：
        value: 候选的图像尺寸。
        name: 用于错误信息的参数名。

    返回：
        该值转换后的正 Python ``int``。

    异常：
        TypeError: 若 ``value`` 不是整数或整值浮点数。
        ValueError: 若整数值小于 1。
    """
    if isinstance(value, bool) or not isinstance(value, (int, float, np.integer)):
        raise TypeError(
            f"{name} must be an integer >= 1, got {value!r} ({type(value).__name__})"
        )
    number = float(value)
    if not math.isfinite(number) or not float(number).is_integer():
        raise TypeError(f"{name} must be an integer >= 1, got {value!r}")
    out = int(number)
    if out < 1:
        raise ValueError(f"{name} must be >= 1, got {value!r}")
    return out


def _require_intrinsics(intrinsics: Any) -> Any:
    """校验 ``intrinsics`` 是否表现得像 :class:`Intrinsics`。

    采用鸭子类型（而非 ``isinstance``）可使这些辅助函数适用于任何暴露
    ``fx``/``fy``/``cx``/``cy`` 的对象——包括直接从文件路径加载的、本模块的
    第二份副本。

    参数：
        intrinsics: 候选的内参对象。

    返回：
        原封不动的同一对象。

    异常：
        TypeError: 若缺少必需属性，或其不是数值。
        ValueError: 若焦距为零/负数，或某值非有限。
    """
    missing = [name for name in ("fx", "fy", "cx", "cy") if not hasattr(intrinsics, name)]
    if missing:
        raise TypeError(
            f"intrinsics must expose {missing} (got {type(intrinsics).__name__}: "
            f"{intrinsics!r}); expected a pinhole.Intrinsics-like object"
        )
    fx = _finite_float(intrinsics.fx, "intrinsics.fx")
    fy = _finite_float(intrinsics.fy, "intrinsics.fy")
    _finite_float(intrinsics.cx, "intrinsics.cx")
    _finite_float(intrinsics.cy, "intrinsics.cy")
    if fx <= 0.0:
        raise ValueError(f"intrinsics.fx must be > 0, got {intrinsics.fx!r}")
    if fy <= 0.0:
        raise ValueError(f"intrinsics.fy must be > 0, got {intrinsics.fy!r}")
    return intrinsics


def _shape_hw(shape: Any, name: str) -> tuple[int, int]:
    """从数组形状或 ``(h, w)`` 数对中提取 ``(height, width)``。

    参数：
        shape: 一个 numpy 形状元组，或任意可迭代对象，其前两项为高度和宽度
            （诸如通道数之类的额外项会被忽略）。
        name: 用于错误信息的参数名。

    返回：
        以正整数表示的 ``(height, width)``。

    异常：
        TypeError: 若 ``shape`` 不可迭代，或其前两项不是整数。
        ValueError: 若 ``shape`` 的元素少于两项，或某个维度小于 1。
    """
    if isinstance(shape, (str, bytes)):
        raise TypeError(
            f"{name} must be a sequence like (height, width), got {shape!r}"
        )
    try:
        dims = tuple(shape)
    except TypeError as exc:
        raise TypeError(
            f"{name} must be a sequence like (height, width), got {shape!r} "
            f"({type(shape).__name__})"
        ) from exc
    if len(dims) < 2:
        raise ValueError(f"{name} must hold at least (height, width), got {shape!r}")
    height = _positive_int(dims[0], f"{name}[0] (height)")
    width = _positive_int(dims[1], f"{name}[1] (width)")
    return height, width


@dataclass(frozen=True)
class Intrinsics:
    """某一路已标定图像流的不可变针孔内参。

    实例被设为 frozen（不可变），这样它就能在线程间共享（相机驱动通常只发布
    一次内参，之后每个消费者只读取它）而无需防御性拷贝，同时下游的辅助函数也
    无法悄悄改动他人的标定参数。

    属性：
        fx: 水平焦距，单位为像素。
        fy: 垂直焦距，单位为像素。
        cx: 主点列坐标，单位为像素（从图像左边缘量起）。
        cy: 主点行坐标，单位为像素（从图像上边缘量起）。
        width: 这些内参所标定的图像宽度，单位为像素。
        height: 这些内参所标定的图像高度，单位为像素。
        depth_scale: 将原始深度值换算为米的**除数**，即
            ``metres = raw / depth_scale``。默认值 ``1000.0`` 对应无处不在的
            16 位毫米深度图；若驱动已直接给出 ``float32`` 米，则用 ``1.0``。

    异常：
        TypeError: 若某字段为非数值类型，或 ``width``/``height`` 不是整数。
        ValueError: 若 ``fx``/``fy``/``depth_scale`` 不是严格为正，或任意值为
            NaN/无穷大，或某个分辨率小于 1。

    示例：
        >>> intr = Intrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        ...                   width=640, height=480)
        >>> intr.to_matrix().shape
        (3, 3)
        >>> intr.scaled(0.5).fx
        250.0
        >>> intr.resized(320, 240).cx
        160.0
        >>> Intrinsics(0.0, 500.0, 320.0, 240.0, 640, 480)
        Traceback (most recent call last):
            ...
        ValueError: fx must be > 0 (pixels per unit angle), got 0.0
    """

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    depth_scale: float = 1000.0

    def __post_init__(self) -> None:
        """规范化并校验每个字段。

        规范化（``float``/``int`` 强制转换）需要使用 ``object.__setattr__``，
        因为该 dataclass 是 frozen 的；这样做可以保证后续运算使用精确的 Python
        标量，而不会把 ``numpy`` 标量类型泄漏到结果中。
        """
        fx = _finite_float(self.fx, "fx")
        fy = _finite_float(self.fy, "fy")
        cx = _finite_float(self.cx, "cx")
        cy = _finite_float(self.cy, "cy")
        width = _positive_int(self.width, "width")
        height = _positive_int(self.height, "height")
        depth_scale = _finite_float(self.depth_scale, "depth_scale")

        if fx <= 0.0:
            raise ValueError(f"fx must be > 0 (pixels per unit angle), got {self.fx!r}")
        if fy <= 0.0:
            raise ValueError(f"fy must be > 0 (pixels per unit angle), got {self.fy!r}")
        if depth_scale <= 0.0:
            raise ValueError(
                f"depth_scale must be > 0 (raw depth units per metre), got {self.depth_scale!r}"
            )

        object.__setattr__(self, "fx", fx)
        object.__setattr__(self, "fy", fy)
        object.__setattr__(self, "cx", cx)
        object.__setattr__(self, "cy", cy)
        object.__setattr__(self, "width", width)
        object.__setattr__(self, "height", height)
        object.__setattr__(self, "depth_scale", depth_scale)

        if not (0.0 <= cx <= float(width)) or not (0.0 <= cy <= float(height)):
            # 从纸面上看，主点落在传感器之外是合法的（在重度裁剪或已校正的
            # 图像流中确实会出现），但更常见的原因是 cx/cy 写反或分辨率搞错，
            # 因此让它显式可见。
            logger.warning(
                "principal point (cx=%.3f, cy=%.3f) lies outside the %dx%d image; "
                "check for swapped cx/cy or a resolution mismatch",
                cx, cy, width, height,
            )

    # -- 转换 --------------------------------------------------------------

    def to_matrix(self) -> np.ndarray:
        """返回 3x3 相机矩阵 ``K``。

        斜切项 ``K[0, 1]`` 恒为零：本模型不携带斜切，现代传感器也不存在可测量
        量级的斜切。

        返回：
            一个 ``(3, 3)`` 的 ``float64`` 数组
            ``[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]``。

        示例：
            >>> Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480).to_matrix()[0, 2]
            320.0
        """
        return np.array(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    @classmethod
    def from_matrix(
        cls,
        k: Any,
        width: int,
        height: int,
        depth_scale: float = 1000.0,
    ) -> "Intrinsics":
        """从相机矩阵构建内参。

        参数：
            k: 一个 ``(3, 3)`` 的数组类对象，或按行优先顺序排列的 9 个元素的
                扁平序列（即 ROS 用于 ``CameraInfo.k`` 的布局）。
            width: 图像宽度，单位为像素。
            height: 图像高度，单位为像素。
            depth_scale: 每米对应的原始深度单位数（参见 :class:`Intrinsics`）。

        返回：
            一个新的 :class:`Intrinsics`。

        异常：
            TypeError: 若 ``k`` 不是数值，或某个维度不是整数。
            ValueError: 若 ``k`` 没有 9 个元素，其最后一行（经 ``k[2, 2]``
                归一化后）不是 ``[0, 0, 1]`` 形式，或提取出的焦距不为正。

        示例：
            >>> Intrinsics.from_matrix([500, 0, 320, 0, 500, 240, 0, 0, 1],
            ...                        640, 480).fy
            500.0
        """
        arr = np.asarray(k, dtype=np.float64)
        if arr.shape == (9,):
            arr = arr.reshape(3, 3)
        if arr.shape != (3, 3):
            raise ValueError(
                f"k must be a (3, 3) matrix or a flat sequence of 9 floats, "
                f"got shape {arr.shape} from {np.asarray(k).tolist()!r}"
            )
        if not np.all(np.isfinite(arr)):
            raise ValueError(f"k must contain only finite values, got {arr.tolist()!r}")

        if abs(arr[2, 2]) < 1e-12:
            raise ValueError(
                f"k[2, 2] must be non-zero to normalise the matrix, got {arr[2, 2]!r}"
            )
        if abs(arr[2, 2] - 1.0) > 1e-9:
            # 有些标定流程存储的 K 带缩放；在读取 fx/fy/cx/cy 之前先归一化，
            # 使最后一行恰好变为 [0, 0, 1]。
            arr = arr / arr[2, 2]
        if abs(arr[2, 0]) > 1e-9 or abs(arr[2, 1]) > 1e-9:
            raise ValueError(
                f"k last row must be [0, 0, 1] for a pinhole camera, got "
                f"{arr[2].tolist()!r}"
            )
        if abs(arr[0, 1]) > 1e-9:
            logger.warning(
                "ignoring non-zero skew term k[0, 1]=%.6f; this model assumes "
                "axis-aligned pixels", arr[0, 1],
            )
        return cls(
            fx=float(arr[0, 0]),
            fy=float(arr[1, 1]),
            cx=float(arr[0, 2]),
            cy=float(arr[1, 2]),
            width=width,
            height=height,
            depth_scale=depth_scale,
        )

    @classmethod
    def from_ros_camera_info(
        cls,
        msg_dict: Any,
        depth_scale: float = 1000.0,
    ) -> "Intrinsics":
        """从 ROS ``sensor_msgs/CameraInfo`` 的*数据*构建内参。

        该 classmethod 从不导入 ROS。可传入以下任意一种：

        * 带有 ``width``、``height`` 和 ``k`` 键的普通映射（``k`` 是 9 元素的
          行优先相机矩阵），例如订阅者由 ``msg.width`` / ``msg.height`` /
          ``msg.k`` 构建的字典，或 ``msg.__dict__``；
        * 或任何以属性形式暴露这三者的对象，例如 ROS 消息本身——只使用
          ``getattr``，因此不需要任何 ROS 类型。

        当存在 ``depth_scale`` 键/属性时，它会覆盖函数参数；这一点很便利，因为
        在许多驱动中深度缩放随相机描述一起传递，但不在 ``CameraInfo`` 中。

        参数：
            msg_dict: 带有 ``width``、``height`` 和 ``k`` 的映射或带属性的对象。
            depth_scale: 备用值，每米对应的原始深度单位数。

        返回：
            一个新的 :class:`Intrinsics`。

        异常：
            TypeError: 若 ``msg_dict`` 既不是映射，也不是带有所需属性的对象，
                或某字段为非数值类型。
            ValueError: 若缺少 ``width``/``height``/``k``，或 ``k`` 不是有效的
                相机矩阵。

        示例：
            >>> info = {"width": 640, "height": 480,
            ...         "k": [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]}
            >>> Intrinsics.from_ros_camera_info(info).cx
            320.0
        """
        def _get(key: str, default: Any = None) -> Any:
            if isinstance(msg_dict, Mapping):
                if key in msg_dict:
                    return msg_dict[key]
                # ROS 使用小写字段名；同时容忍手写配置里出现的首字母大写写法。
                upper = key.upper()
                if upper in msg_dict:
                    return msg_dict[upper]
                return default
            return getattr(msg_dict, key, default)

        if not isinstance(msg_dict, Mapping) and not any(
            hasattr(msg_dict, name) for name in ("width", "height", "k")
        ):
            raise TypeError(
                f"msg_dict must be a mapping or an object with width/height/k "
                f"attributes, got {msg_dict!r} ({type(msg_dict).__name__})"
            )

        width = _get("width")
        height = _get("height")
        if width is None or height is None:
            available = (
                sorted(msg_dict) if isinstance(msg_dict, Mapping)
                else [n for n in ("width", "height", "k", "p", "d") if hasattr(msg_dict, n)]
            )
            raise ValueError(
                f"camera info must provide 'width' and 'height', found only {available!r}"
            )
        k = _get("k")
        if k is None:
            raise ValueError(
                "camera info must provide the 3x3 camera matrix as 'k' "
                "(9 floats, row-major); rectified-only streams that fill just "
                "'p' should pass p[:, :3] instead"
            )
        scale = _get("depth_scale")
        return cls.from_matrix(
            k,
            width=width,
            height=height,
            depth_scale=depth_scale if scale is None else scale,
        )

    # -- 重缩放 ------------------------------------------------------------

    def scaled(self, factor: float) -> "Intrinsics":
        """返回同一相机在图像均匀缩放后的内参。

        每个以像素表示的量——焦距、主点和分辨率——都按 ``factor`` 缩放，这正是
        单个乘数就足够的原因。深度缩放是*深度值*的属性，而非图像几何的属性，
        因此原样保留。

        参数：
            factor: 缩放因子，例如图像缩小一半时为 ``0.5``。

        返回：
            一个新的 :class:`Intrinsics`，其 ``fx``、``fy``、``cx``、``cy``、
            ``width`` 和 ``height`` 均已缩放（分辨率四舍五入到最近的整数，
            且不会低于 1）。

        异常：
            TypeError: 若 ``factor`` 不是实数。
            ValueError: 若 ``factor`` 不是严格为正。

        示例：
            >>> intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
            >>> intr.scaled(2.0).width
            1280
            >>> intr.scaled(2.0).cy
            480.0
        """
        value = _finite_float(factor, "factor")
        if value <= 0.0:
            raise ValueError(f"factor must be > 0, got {factor!r}")
        return Intrinsics(
            fx=self.fx * value,
            fy=self.fy * value,
            cx=self.cx * value,
            cy=self.cy * value,
            width=max(1, int(round(self.width * value))),
            height=max(1, int(round(self.height * value))),
            depth_scale=self.depth_scale,
        )

    def resized(self, width: int, height: int) -> "Intrinsics":
        """返回与新的图像分辨率相匹配的内参。

        水平与垂直方向独立处理，因此非均匀重缩放（会改变像素宽高比）也能保持
        正确——当 16:9 的视频流被压缩进 4:3 的网络输入时，这是常见情形。

        参数：
            width: 目标图像宽度，单位为像素。
            height: 目标图像高度，单位为像素。

        返回：
            一个适用于目标分辨率的新 :class:`Intrinsics`。

        异常：
            TypeError: 若某个维度不是整数。
            ValueError: 若某个维度小于 1。

        示例：
            >>> intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
            >>> intr.resized(1280, 960).fx
            1000.0
            >>> intr.resized(320, 480).fy
            500.0
        """
        new_width = _positive_int(width, "width")
        new_height = _positive_int(height, "height")
        sx = new_width / self.width
        sy = new_height / self.height
        return Intrinsics(
            fx=self.fx * sx,
            fy=self.fy * sy,
            cx=self.cx * sx,
            cy=self.cy * sy,
            width=new_width,
            height=new_height,
            depth_scale=self.depth_scale,
        )


# -- 反投影 --------------------------------------------------------------


def pixel_to_3d(
    u: float,
    v: float,
    depth_m: float,
    intrinsics: Any,
) -> np.ndarray:
    """将一个像素加上一个度量深度反投影到相机光学坐标系。

    映射关系为 ``x = (u - cx) * d / fx``、``y = (v - cy) * d / fy``、
    ``z = d``，即把像素视为一条射线，其沿 ``+z`` 的距离由深度传感器给出。
    坐标遵循本模块所记录的光学坐标系约定：``x`` 向右、``y`` 向下、``z`` 向前。

    非正或非有限的深度会被拒绝，而不是被修补：深度*是*唯一确定射线距离的量，
    因此凭空捏造一个深度（钳制到最小值、回退到默认量程）会静默地产生一个看似
    合理、实则错误的三维点——对于抓取或避障流程而言，这严格劣于让调用方能够
    处理的显式失败。

    参数：
        u: 像素列坐标（可为亚像素）。
        v: 像素行坐标（可为亚像素）。
        depth_m: 以**米**为单位的深度，严格为正且有限。
        intrinsics: :class:`Intrinsics` 类对象。

    返回：
        一个 ``(3,)`` 的 ``float64`` 数组 ``[x, y, z]``，单位为米。

    异常：
        TypeError: 若某参数不是数值，或内参格式不正确。
        ValueError: 若 ``depth_m`` 为 NaN、无穷大或 ``<= 0``，或像素坐标非有限。

    示例：
        >>> intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
        >>> pixel_to_3d(320, 240, 2.0, intr)
        array([0., 0., 2.])
        >>> pixel_to_3d(820, 240, 2.0, intr)
        array([2., 0., 2.])
        >>> pixel_to_3d(320, 240, 0.0, intr)
        Traceback (most recent call last):
            ...
        ValueError: depth_m must be finite and > 0 metres, got 0.0 -- a depth of 0 (or NaN) means the sensor has no measurement, and guessing a ray distance would fabricate a wrong 3D point
    """
    intr = _require_intrinsics(intrinsics)
    uu = _finite_float(u, "u")
    vv = _finite_float(v, "v")
    depth = _finite_float(depth_m, "depth_m")
    if depth <= 0.0:
        raise ValueError(
            f"depth_m must be finite and > 0 metres, got {depth_m!r} -- a depth of "
            f"0 (or NaN) means the sensor has no measurement, and guessing a ray "
            f"distance would fabricate a wrong 3D point"
        )
    return np.array(
        [
            (uu - intr.cx) * depth / intr.fx,
            (vv - intr.cy) * depth / intr.fy,
            depth,
        ],
        dtype=np.float64,
    )


def pixel_to_3d_batch(
    uv: Any,
    depths: Any,
    intrinsics: Any,
    *,
    return_valid: bool = False,
) -> Any:
    """对 :func:`pixel_to_3d` 在大量像素上的向量化实现。

    与标量版本不同，此函数在深度无效时**不会**抛异常。批处理调用方通常处理
    整个掩码或包围框，其中出现少量空洞是正常的，若在第一个空洞处就中止，会
    迫使用户编写缓慢的 Python 过滤循环。无效项会变成 ``NaN`` 行，并通过可选的
    有效性掩码报告；只要输入可能包含空洞，就应使用 ``return_valid=True``。

    参数：
        uv: ``(N, 2)`` 的数组类对象，元素为 ``(u, v)`` 像素坐标。
        depths: ``(N,)`` 的数组类对象，元素为以米为单位的深度（任意正深度均
            有效；``0``、负值和非有限项无效）。
        intrinsics: :class:`Intrinsics` 类对象。
        return_valid: 为 ``True`` 时同时返回布尔有效性掩码。

    返回：
        ``(N, 3)`` 的 ``float64`` 点集；当设置了 ``return_valid`` 时返回
        ``(points, valid)`` 元组。深度无效的行其值为 ``NaN``，且
        ``valid == False``。

    异常：
        TypeError: 若内参格式不正确。
        ValueError: 若 ``uv`` 不是二维且列数为 2，或 ``depths`` 的元素个数
            不等于 ``N``。

    示例：
        >>> intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
        >>> pts, ok = pixel_to_3d_batch([[320, 240], [820, 240]], [2.0, 0.0],
        ...                             intr, return_valid=True)
        >>> ok.tolist()
        [True, False]
        >>> pts.shape
        (2, 3)
        >>> bool(np.isnan(pts[1]).all())
        True
    """
    intr = _require_intrinsics(intrinsics)
    uv_arr = np.asarray(uv, dtype=np.float64)
    if uv_arr.ndim != 2 or uv_arr.shape[1] != 2:
        raise ValueError(
            f"uv must have shape (N, 2) of pixel coordinates, got {uv_arr.shape} "
            f"from {np.asarray(uv).tolist()!r}"
        )
    depth_arr = np.asarray(depths, dtype=np.float64).reshape(-1)
    if depth_arr.shape[0] != uv_arr.shape[0]:
        raise ValueError(
            f"depths must hold one value per pixel: got {depth_arr.shape[0]} depths "
            f"for {uv_arr.shape[0]} pixels"
        )

    valid = (
        np.isfinite(depth_arr)
        & (depth_arr > 0.0)
        & np.all(np.isfinite(uv_arr), axis=1)
    )
    # 将无效深度替换为 1.0 以便参与运算，避免除零告警或 inf 泄漏到不相关的行；
    # 这些行会在下面被覆写为 NaN。
    safe = np.where(valid, depth_arr, 1.0)
    points = np.empty((uv_arr.shape[0], 3), dtype=np.float64)
    points[:, 0] = (uv_arr[:, 0] - intr.cx) * safe / intr.fx
    points[:, 1] = (uv_arr[:, 1] - intr.cy) * safe / intr.fy
    points[:, 2] = safe
    points[~valid] = np.nan
    if return_valid:
        return points, valid
    return points


# -- 投影 ----------------------------------------------------------------


def project(
    x: float,
    y: float,
    z: float,
    intrinsics: Any,
) -> tuple[float, float, float]:
    """将光学坐标系中的单个点投影到图像平面上。

    参数：
        x: 光学坐标系下的 ``x``，单位为米（向右）。
        y: 光学坐标系下的 ``y``，单位为米（向下）。
        z: 光学坐标系下的 ``z``，单位为米（向前），即深度。
        intrinsics: :class:`Intrinsics` 类对象。

    返回：
        ``(u, v, depth_raw)``，其中 ``u``/``v`` 为亚像素图像坐标，
        ``depth_raw`` 是以原始深度图像单位表示的 ``z``，即
        ``z * intrinsics.depth_scale``（默认缩放下为毫米）。之所以返回原始值，
        是因为调用方在做 z-buffering 或遮挡测试时正是拿它与深度缓冲区比较。

    异常：
        TypeError: 若某参数不是数值，或内参格式不正确。
        ValueError: 若 ``z <= 0``——位于图像平面上或其后的点没有透视投影
            （除法会爆炸或翻转符号），把它钳制进图像中永远不是正确做法。

    示例：
        >>> intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
        >>> project(2.0, 0.0, 2.0, intr)
        (820.0, 240.0, 2000.0)
        >>> project(0.0, 0.0, -1.0, intr)
        Traceback (most recent call last):
            ...
        ValueError: z must be > 0 (points behind the camera cannot be projected), got -1.0
    """
    intr = _require_intrinsics(intrinsics)
    xx = _finite_float(x, "x")
    yy = _finite_float(y, "y")
    zz = _finite_float(z, "z")
    if zz <= 0.0:
        raise ValueError(
            f"z must be > 0 (points behind the camera cannot be projected), got {z!r}"
        )
    u = xx * intr.fx / zz + intr.cx
    v = yy * intr.fy / zz + intr.cy
    scale = getattr(intr, "depth_scale", 1000.0)
    return float(u), float(v), float(zz * _finite_float(scale, "intrinsics.depth_scale"))


def project_batch(
    points: Any,
    intrinsics: Any,
    *,
    return_valid: bool = False,
) -> Any:
    """对 :func:`project` 在大量点上的向量化实现。

    坐标为 ``z <= 0`` 或非有限的点会被标记为无效并填充 ``NaN``，而不是抛异常，
    原因与 :func:`pixel_to_3d_batch` 相同：投影后的点云几乎总会包含少数位于
    相机后方的点。

    参数：
        points: ``(N, 3)`` 的数组类对象，元素为光学坐标系下以米为单位的点。
        intrinsics: :class:`Intrinsics` 类对象。
        return_valid: 为 ``True`` 时同时返回布尔有效性掩码。

    返回：
        ``(N, 3)`` 的 ``float64`` 数组，各列依次为 ``[u, v, depth_raw]``；
        当设置了 ``return_valid`` 时返回 ``(projected, valid)`` 元组。

    异常：
        TypeError: 若内参格式不正确。
        ValueError: 若 ``points`` 不是二维且列数为 3。

    示例：
        >>> intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
        >>> out, ok = project_batch([[2.0, 0.0, 2.0], [0.0, 0.0, -1.0]], intr,
        ...                         return_valid=True)
        >>> ok.tolist()
        [True, False]
        >>> out[0].tolist()
        [820.0, 240.0, 2000.0]
    """
    intr = _require_intrinsics(intrinsics)
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(
            f"points must have shape (N, 3), got {pts.shape} from "
            f"{np.asarray(points).tolist()!r}"
        )
    z = pts[:, 2]
    valid = np.all(np.isfinite(pts), axis=1) & (z > 0.0)
    safe_z = np.where(valid, z, 1.0)
    out = np.empty((pts.shape[0], 3), dtype=np.float64)
    out[:, 0] = pts[:, 0] * intr.fx / safe_z + intr.cx
    out[:, 1] = pts[:, 1] * intr.fy / safe_z + intr.cy
    scale = _finite_float(getattr(intr, "depth_scale", 1000.0), "intrinsics.depth_scale")
    out[:, 2] = safe_z * scale
    out[~valid] = np.nan
    if return_valid:
        return out, valid
    return out


# -- 射线 ----------------------------------------------------------------


def ray_direction(u: float, v: float, intrinsics: Any) -> np.ndarray:
    """返回穿过像素 ``(u, v)`` 的单位视线射线。

    这就是在单位深度处的反投影 ``[(u - cx)/fx, (v - cy)/fy, 1]``，再归一化到
    长度 1。归一化很重要：:func:`planes.ray_plane_intersection` 求解
    ``origin + t * direction`` 中的 ``t``，只有单位方向才能让 ``t`` 成为距相机
    中心的真实度量距离——而这正是调用方用于阈值判断的量（例如“地面交点是否在
    机械臂可及范围内？”）。

    参数：
        u: 像素列坐标（可为亚像素）。
        v: 像素行坐标（可为亚像素）。
        intrinsics: :class:`Intrinsics` 类对象。

    返回：
        光学坐标系下指向远离相机中心方向的 ``(3,)`` ``float64`` 单位向量；
        其 ``z`` 分量恒为正。

    异常：
        TypeError: 若某参数不是数值，或内参格式不正确。
        ValueError: 若像素坐标非有限。

    示例：
        >>> intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
        >>> ray_direction(320, 240, intr)
        array([0., 0., 1.])
        >>> float(np.round(np.linalg.norm(ray_direction(640, 480, intr)), 12))
        1.0
    """
    intr = _require_intrinsics(intrinsics)
    uu = _finite_float(u, "u")
    vv = _finite_float(v, "v")
    direction = np.array(
        [(uu - intr.cx) / intr.fx, (vv - intr.cy) / intr.fy, 1.0],
        dtype=np.float64,
    )
    # z 分量恰好为 1，因此范数不可能为零。
    return direction / np.linalg.norm(direction)


def pixel_to_depth_pixel(
    u: float,
    v: float,
    color_shape: Any,
    depth_shape: Any,
) -> tuple[int, int]:
    """将彩色图像像素映射到对应的深度图像像素。

    即便驱动宣称图像流已“对齐”，彩色与深度也常常来自不同传感器、具有不同分辨率
    （经典例子是 1280x720 的彩色对 848x480 或 640x480 的深度）。对齐消除的是
    *视差*，而不是分辨率差异，因此一个以彩色像素为单位报告包围框的检测器必须先
    对其坐标重新缩放，再去索引深度缓冲区——否则每次深度查找都会偏离分辨率比例，
    且越靠近图像边缘偏差越大。

    结果会被钳制到深度图像内，因此位于边界上的彩色像素绝不会产生越界的索引。

    参数：
        u: 彩色图像中的列坐标。
        v: 彩色图像中的行坐标。
        color_shape: 彩色图像形状；``(height, width)`` 或 ``(h, w, 3)``。
        depth_shape: 深度图像形状；``(height, width)`` 或 ``(h, w, c)``。

    返回：
        ``(depth_u, depth_v)``，深度图像内的整数像素坐标。

    异常：
        TypeError: 若某坐标不是数值，或某个形状格式不正确。
        ValueError: 若某个形状中存在小于 1 的维度。

    示例：
        >>> pixel_to_depth_pixel(320, 240, (480, 640), (240, 320))
        (160, 120)
        >>> pixel_to_depth_pixel(639, 479, (480, 640), (240, 320))
        (319, 239)
    """
    uu = _finite_float(u, "u")
    vv = _finite_float(v, "v")
    color_h, color_w = _shape_hw(color_shape, "color_shape")
    depth_h, depth_w = _shape_hw(depth_shape, "depth_shape")

    du = int(round(uu * depth_w / color_w))
    dv = int(round(vv * depth_h / color_h))
    return (
        min(max(du, 0), depth_w - 1),
        min(max(dv, 0), depth_h - 1),
    )
