"""平面双连杆逆运动学（planar two-link IK）、肘圆冗余与 LM 精修。

这里汇集了三块看似无关的内容，因为它们正是一台带肢体、必须在嵌入式板上运行的
机器人对一个求解器所需的三样东西：

* :class:`PlanarTwoLinkIK`——在竖直平面内运动的双连杆链的解析解/闭式解逆运动学
  （closed-form IK）。它既是一条腿（髋俯仰 + 膝俯仰，决定机体高度与前后位置），
  也是简化到平面内的手臂肩-肘三角形。之所以用闭式解，是因为在 100 Hz 的平衡
  控制回路中，数值求解器既更慢又更不可预测。
* :func:`elbow_circle` 及其相关函数——7 自由度（DOF）手臂**冗余**自由度的几何
  参数化。给定肩与腕，肘并不固定：它可以在一个圆上滑动。在该圆上选取一点*就是*
  在选择冗余自由度，因此手臂全部的“自然姿态”逻辑都集中于这一处选择
  （:func:`select_elbow_on_circle`）。
* :func:`levenberg_marquardt` 与 :func:`numeric_jacobian`——一个小巧、依赖极少的
  非线性最小二乘求解器，用于对给定的初始姿态进行精修，直至末端执行器真正落到
  目标点上。

约定：角度单位为弧度，距离单位为米，平面链位于 ``x`` 向前 / ``z`` 向上的平面内
（``q = 0`` 表示“沿 ``+z`` 竖直向上”，即腿完全伸直站立的姿态）。此处不涉及
ROS、特定机器人或电机耦合表；由调用方负责把返回的关节角映射为电机指令。

依赖：导入时依赖 :mod:`numpy`。``scipy`` 仅由
:meth:`PlanarTwoLinkIK.solve_numeric` 惰性导入，因此解析求解器——即实时控制器
所使用的那个——除 numpy 外不需要任何东西。
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any, Callable, Sequence

import numpy as np
from numpy.typing import ArrayLike

logger = logging.getLogger(__name__)

__all__ = [
    "PlanarTwoLink",
    "PlanarIKResult",
    "PlanarTwoLinkIK",
    "elbow_circle",
    "lowest_point_on_circle",
    "circle_plane_intersections",
    "select_elbow_on_circle",
    "levenberg_marquardt",
    "numeric_jacobian",
]

#: 检测关节角是否超出其行程限位时允许的裕量。
#:
#: 若求解器因为某个角度超出限位 1e-12 rad 就报告失败，那还不如没有：该值在物理上
#: 完全没问题，调用方却不得不扰动目标后重试。1e-3 rad（0.06 度）远低于任何真实
#: 减速器的分辨率，同时又能吸收累积的浮点误差。
_LIMIT_TOL: float = 1e-3

#: :func:`elbow_circle` 中三角不等式以及 :meth:`PlanarTwoLinkIK.solve`
#: 中可达性测试的相对裕量。理由同 :data:`_LIMIT_TOL`。
_GEOMETRY_EPS: float = 1e-9

#: :func:`levenberg_marquardt` 中线性搜索接受判据的 Armijo 系数：当某一步使误差
#: 下降至少该比例乘以步长时，该步被接受。
_ARMIJO_C: float = 1e-4

#: 当关节限位仅一侧为无穷、因而没有中点时，用作初始猜测的角度（弧度）。
_OPEN_LIMIT_SEED: float = 1.0


def _finite_float(value: Any, name: str) -> float:
    """把 ``value`` 转换为有限 ``float``，否则带着违规值抛出异常。

    参数：
        value: 待转换的数值。
        name: 用于错误消息的参数名。

    返回：
        该值对应的有限 ``float``。

    异常：
        TypeError: 当 ``value`` 不是实数时抛出（``bool`` 会被拒绝，因为
            ``True`` 会悄然表现成 ``1.0``）。
        ValueError: 当 ``value`` 为 NaN 或无穷时抛出。
    """
    if isinstance(value, bool) or not isinstance(value, (int, float, np.floating, np.integer)):
        raise TypeError(f"{name} must be a real number, got {value!r} ({type(value).__name__})")
    out = float(value)
    if not math.isfinite(out):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return out


def _positive_float(value: Any, name: str) -> float:
    """把 ``value`` 转换为严格为正的有限 ``float``，否则抛出异常。

    参数：
        value: 待转换的数值。
        name: 用于错误消息的参数名。

    返回：
        该值对应的正 ``float``。

    异常：
        TypeError: 当 ``value`` 不是实数时抛出。
        ValueError: 当其为 NaN、无穷或 ``<= 0`` 时抛出。
    """
    out = _finite_float(value, name)
    if out <= 0.0:
        raise ValueError(f"{name} must be > 0, got {value!r}")
    return out


def _non_negative_float(value: Any, name: str) -> float:
    """把 ``value`` 转换为 ``>= 0`` 的有限 ``float``，否则抛出异常。

    参数：
        value: 待转换的数值。
        name: 用于错误消息的参数名。

    返回：
        该值对应的非负 ``float``。

    异常：
        TypeError: 当 ``value`` 不是实数时抛出。
        ValueError: 当其为 NaN、无穷或负数时抛出。
    """
    out = _finite_float(value, name)
    if out < 0.0:
        raise ValueError(f"{name} must be >= 0, got {value!r}")
    return out


def _count_int(value: Any, name: str, *, minimum: int = 1) -> int:
    """把计数类参数（迭代预算、轴索引）转换为 ``int``。

    参数：
        value: 待转换的计数值。
        name: 用于错误消息的参数名。
        minimum: 可接受的最小值。

    返回：
        该值对应的 Python ``int``。

    异常：
        TypeError: 当 ``value`` 不是整数时抛出（``bool`` 会被拒绝）。
        ValueError: 当其小于 ``minimum`` 时抛出。
    """
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise TypeError(
            f"{name} must be an integer >= {minimum}, got {value!r} ({type(value).__name__})"
        )
    out = int(value)
    if out < minimum:
        raise ValueError(f"{name} must be >= {minimum}, got {value!r}")
    return out


def _axis_index(axis: Any, name: str) -> int:
    """把世界坐标轴索引转换为 ``0``、``1`` 或 ``2``。

    参数：
        axis: 待转换的轴索引。
        name: 用于错误消息的参数名。

    返回：
        该轴索引对应的 ``int``。

    异常：
        TypeError: 当 ``axis`` 不是整数时抛出。
        ValueError: 当其不是 0（x）、1（y）或 2（z）时抛出。
    """
    out = _count_int(axis, name, minimum=0)
    if out > 2:
        raise ValueError(f"{name} must be 0 (x), 1 (y) or 2 (z), got {axis!r}")
    return out


def _as_vector(value: Any, dim: int, name: str) -> np.ndarray:
    """把 ``value`` 转换为长度为 ``dim`` 的有限 ``float64`` 向量。

    参数：
        value: 任意恰好包含 ``dim`` 个元素的类数组对象。
        dim: 要求的长度。
        name: 用于错误消息的参数名。

    返回：
        ``(dim,)`` 的 ``float64`` 数组。

    异常：
        TypeError: 当 ``value`` 不是类数组对象时抛出。
        ValueError: 当其并非恰好包含 ``dim`` 个有限数值时抛出。
    """
    try:
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"{name} must be array-like with {dim} element(s), got {value!r}"
        ) from exc
    if arr.size != dim:
        raise ValueError(
            f"{name} must have exactly {dim} element(s), got {arr.size}: {value!r}"
        )
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return arr


def _as_vector_flat(value: Any, name: str) -> np.ndarray:
    """把 ``value`` 转换为任意长度的有限一维 ``float64`` 数组。

    参数：
        value: 任意可展平为有限向量的类数组对象。
        name: 用于错误消息的参数名。

    返回：
        ``(n,)`` 的 ``float64`` 数组，其中 ``n >= 1``。

    异常：
        TypeError: 当 ``value`` 不是类数组对象时抛出。
        ValueError: 当其为空或包含非有限数值时抛出。
    """
    try:
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be an array-like vector, got {value!r}") from exc
    if arr.size == 0:
        raise ValueError(f"{name} must hold at least one element, got {value!r}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return arr


def _limits_pair(limits: Any, name: str) -> tuple[float, float]:
    """把关节限位参数规范化为 ``(lower, upper)`` 浮点对。

    ``None`` 表示“该关节无限制”，会被转换为 ``(-inf, inf)``，模块其余部分无需
    任何特殊处理即可对其进行比较——而且 :func:`scipy.optimize.least_squares`
    也接受它作为边界。

    参数：
        limits: ``None``，或以弧度表示的两元素序列 ``(lower, upper)``。
        name: 用于错误消息的参数名。

    返回：
        ``(lower, upper)``，为有限或无穷的浮点数。

    异常：
        TypeError: 当 ``limits`` 既不是 ``None`` 也不是数值序列时抛出。
        ValueError: 当其并非恰好包含两个有限数值，或 ``lower > upper`` 时抛出。
    """
    if limits is None:
        return -math.inf, math.inf
    if isinstance(limits, (str, bytes)):
        raise TypeError(
            f"{name} must be None or a (lower, upper) pair in radians, got {limits!r}"
        )
    try:
        pair = [float(value) for value in limits]
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"{name} must be None or a (lower, upper) pair of numbers, got {limits!r}"
        ) from exc
    if len(pair) != 2:
        raise ValueError(
            f"{name} must hold exactly 2 numbers (lower, upper), got {len(pair)}: {limits!r}"
        )
    lo, hi = pair
    if not (math.isfinite(lo) and math.isfinite(hi)):
        raise ValueError(f"{name} must hold finite radians, got {limits!r}")
    if lo > hi:
        raise ValueError(f"{name} lower bound must be <= upper bound, got {limits!r}")
    return lo, hi


def _inside_limits(angle: float, limits: tuple[float, float]) -> bool:
    """判断在 :data:`_LIMIT_TOL` 容差内 ``angle`` 是否位于 ``limits`` 之内。

    参数：
        angle: 关节角，单位为弧度。
        limits: ``(lower, upper)`` 对，可能为无穷。

    返回：
        当角度位于行程范围内时返回 ``True``。
    """
    lo, hi = limits
    return lo - _LIMIT_TOL <= angle <= hi + _LIMIT_TOL


def _limit_seed(limits: tuple[float, float]) -> float:
    """为具有给定限位的关节返回一个合理的初始角度。

    行程范围的中点是理想的中性初值：它距两端限位都尽可能远，因此带边界的最小二乘
    求解无论目标需要朝哪个方向移动都有余量。当一侧为无穷时不存在中点，此时改用
    距有限一侧限位一个任意但固定的偏移量。

    参数：
        limits: ``(lower, upper)`` 对，可能为无穷。

    返回：
        一个有限的弧度角度。
    """
    lo, hi = limits
    if math.isinf(lo) and math.isinf(hi):
        return 0.0
    if math.isinf(lo):
        return hi - _OPEN_LIMIT_SEED
    if math.isinf(hi):
        return lo + _OPEN_LIMIT_SEED
    return 0.5 * (lo + hi)


@dataclass(frozen=True)
class PlanarTwoLink:
    """在竖直平面内运动的双连杆链几何。

    该链为：固定基座位于 ``(base_x, base_z)``，长度为 ``l1`` 的第一个连杆绕基座
    转动 ``q1``，长度为 ``l2`` 的第二个连杆绕肘部**相对于第一个连杆**转动
    ``q2``。角度从 ``+z`` 向 ``+x`` 方向度量，因此 ``q1 = q2 = 0`` 表示完全
    伸展、竖直向上的链。

    该类被设为冻结（frozen），因为几何是机器的固有属性：一个能悄然改变自身连杆
    长度的求解器会产生无法复现的结果，而在控制器线程与规划器线程之间共享同一个
    实例也不应需要加锁。

    属性：
        l1: 第一个连杆的长度，单位米，严格为正。
        l2: 第二个连杆的长度，单位米，严格为正。
        base_x: 基座关节的水平位置，单位米。真实腿部的髋很少恰好位于足印原点
          正上方，把该偏移内嵌于此，可使 :meth:`PlanarTwoLinkIK.solve` 相对
          解析式只需一次纯平移。
        base_z: 基座关节的高度，单位米。

    异常：
        TypeError: 当某字段不是实数时抛出。
        ValueError: 当 ``l1``/``l2`` 不为严格正，或某字段非有限时抛出。

    示例：
        >>> PlanarTwoLink(0.30, 0.35, base_z=0.06)
        PlanarTwoLink(l1=0.3, l2=0.35, base_x=0.0, base_z=0.06)
        >>> PlanarTwoLink(0.0, 0.35)
        Traceback (most recent call last):
            ...
        ValueError: l1 must be > 0, got 0.0
    """

    l1: float
    l2: float
    base_x: float = 0.0
    base_z: float = 0.0

    def __post_init__(self) -> None:
        """规范化并校验每个字段。

        由于该 dataclass 被冻结，规范化为普通 ``float`` 需要
        ``object.__setattr__``；这同时也能避免 ``numpy`` 标量类型渗入调用方
        用于打印或序列化的结果。

        异常：
            TypeError: 当某字段不是实数时抛出。
            ValueError: 当某连杆长度不为严格正，或某字段为 NaN/无穷时抛出。
        """
        l1 = _positive_float(self.l1, "l1")
        l2 = _positive_float(self.l2, "l2")
        base_x = _finite_float(self.base_x, "base_x")
        base_z = _finite_float(self.base_z, "base_z")
        object.__setattr__(self, "l1", l1)
        object.__setattr__(self, "l2", l2)
        object.__setattr__(self, "base_x", base_x)
        object.__setattr__(self, "base_z", base_z)


@dataclass
class PlanarIKResult:
    """一次逆运动学求解的结果。

    一次求解可能以三种本质不同的方式失败——目标不可达、可达性没问题但所需肘角
    超出其行程范围、以及数值求解未收敛——而希望对三者分别作出不同响应（靠近些
    重试、重新规划落足点，或上报故障）的调用方，需要的不只是一个布尔值。
    ``message`` 以可直接原样记录的形式承载这种区别；即使求解失败，只要关节字段
    有意义也会被填充，从而调用方可以查看解是*如何*违反限位的，而无需自行重新推导。

    属性：
        success: 仅当返回的角度在容差范围内到达目标**且**满足给定限位时为
            ``True``。
        q1: 第一个关节角，单位弧度（当求解早期即失败时为 ``0.0``）。
        q2: 第二个关节角，单位弧度，相对于第一个连杆。
        message: 人类可读的结果说明，成功时为 ``'ok'``。
        reachable_distance: 从基座关节到目标的距离，单位米，即施加可达性测试
            所用的 ``r``。失败时也会报告，因为“超出工作空间多远”正是调用方
            钳制目标时首先需要的信息。

    示例：
        >>> PlanarIKResult(True, 0.1, -0.2, "ok", 0.5).as_dict()["q2"]
        -0.2
        >>> PlanarIKResult(False, message="unreachable").success
        False
    """

    success: bool
    q1: float = 0.0
    q2: float = 0.0
    message: str = ""
    reachable_distance: float = 0.0

    def __post_init__(self) -> None:
        """转换并校验每个字段。

        异常：
            TypeError: 当 ``success`` 不是布尔值、``message`` 不是字符串，
                或某数值字段不是实数时抛出。
            ValueError: 当某数值字段为 NaN/无穷时抛出。
        """
        if not isinstance(self.success, bool):
            raise TypeError(
                f"success must be a bool, got {self.success!r} "
                f"({type(self.success).__name__})"
            )
        if not isinstance(self.message, str):
            raise TypeError(
                f"message must be a str, got {self.message!r} "
                f"({type(self.message).__name__})"
            )
        self.q1 = _finite_float(self.q1, "q1")
        self.q2 = _finite_float(self.q2, "q2")
        self.reachable_distance = _non_negative_float(
            self.reachable_distance, "reachable_distance"
        )

    def as_dict(self) -> dict[str, Any]:
        """以普通字典形式返回结果。

        适用于日志记录、把结果放入传输格式，以及希望在无需导入该类的情况下比较
        整个结果的测试。

        返回：
            ``{'success', 'q1', 'q2', 'message', 'reachable_distance'}``。

        示例：
            >>> sorted(PlanarIKResult(True, 0.1, -0.2, "ok", 0.5).as_dict())
            ['message', 'q1', 'q2', 'reachable_distance', 'success']
        """
        return {
            "success": self.success,
            "q1": self.q1,
            "q2": self.q2,
            "message": self.message,
            "reachable_distance": self.reachable_distance,
        }


class PlanarTwoLinkIK:
    """:class:`PlanarTwoLink` 链的闭式解逆运动学。

    解析求解不过两行三角运算，但两个决策决定着一条腿是表现良好，还是偶尔会踢到
    地面：

    *选择哪个肘分支。* ``cos(q2)`` 只决定 ``|q2|``；``+|q2|`` 与 ``-|q2|`` 都
    能到达目标。该选择**由关节限位驱动**，而非硬编码符号，因为正确的分支是硬件
    的固有属性：只能向后弯曲的膝（``upper <= 0``）必须取负分支，而行程对称的
    关节两个分支都可以。猜错并不会产生略微不同的姿态——它会产生镜像构型，对腿
    而言就意味着把膝驱向地面。

    *如实报告失败。* 目标不可达或肘角不可允许时，返回一个失败的
    :class:`PlanarIKResult` 并给出相关数值，绝不返回一个悄然偏离目标的钳制角度。
    收到被钳制关节角的控制器无法区分“我们已到工作空间边缘”与“我们没问题”，
    只会继续推进。

    示例：
        >>> ik = PlanarTwoLinkIK(PlanarTwoLink(0.25, 0.25, base_z=0.5))
        >>> ik.stand_height
        1.0
        >>> result = ik.solve(0.0, 0.75, q2_limits=(-2.5, 0.0))
        >>> result.success, round(result.q2, 6)
        (True, -2.094395)
        >>> ik.verify(result, 0.0, 0.75) < 1e-12
        True
    """

    def __init__(self, geometry: PlanarTwoLink) -> None:
        """把求解器绑定到某一条链的几何上。

        参数：
            geometry: 待求解的 :class:`PlanarTwoLink`。

        异常：
            TypeError: 当 ``geometry`` 不是 :class:`PlanarTwoLink` 时抛出。
        """
        if not isinstance(geometry, PlanarTwoLink):
            raise TypeError(
                f"geometry must be a PlanarTwoLink, got {geometry!r} "
                f"({type(geometry).__name__})"
            )
        self.geometry = geometry

    @property
    def stand_height(self) -> float:
        """返回链完全伸展时末端点的高度。

        此时 ``q1 = q2 = 0``，即腿的零位姿态：链能达到的最高点，因此既是机器人
        的标称站立高度，也是任何高度指令的上界。控制器会不断用它做钳制，所以它
        是一个属性，而不是让调用方反复重算（并最终因忘记 ``base_z`` 而出错）的
        东西。

        返回：
            ``base_z + l1 + l2``，单位米。

        示例：
            >>> PlanarTwoLinkIK(PlanarTwoLink(0.125, 0.125, base_z=0.25)).stand_height
            0.5
        """
        return self.geometry.base_z + self.geometry.l1 + self.geometry.l2

    def forward(self, q1: float, q2: float) -> tuple[float, float]:
        """返回一对关节角对应的末端点位置。

        正运动学（forward kinematics）使逆解可被检验；一个无法往返验证的求解器，
        其 bug 只能靠机器人摔倒来发现。第二个角度相对于第一个连杆，因此连杆 2
        的绝对角度为 ``q1 + q2``。

        参数：
            q1: 第一个关节角，单位弧度，从 ``+z`` 向 ``+x`` 度量。
            q2: 第二个关节角，单位弧度，相对于第一个连杆。

        返回：
            ``(x, z)``，单位米，位于与基座相同的坐标系中。

        异常：
            TypeError: 当某角度不是实数时抛出。
            ValueError: 当某角度为 NaN 或无穷时抛出。

        示例：
            >>> import math
            >>> ik = PlanarTwoLinkIK(PlanarTwoLink(0.25, 0.25, base_z=0.5))
            >>> ik.forward(0.0, 0.0)
            (0.0, 1.0)
            >>> round(ik.forward(math.pi / 2, 0.0)[0], 12)
            0.5
        """
        a1 = _finite_float(q1, "q1")
        a2 = _finite_float(q2, "q2")
        l1 = self.geometry.l1
        l2 = self.geometry.l2
        x = self.geometry.base_x + l1 * math.sin(a1) + l2 * math.sin(a1 + a2)
        z = self.geometry.base_z + l1 * math.cos(a1) + l2 * math.cos(a1 + a2)
        return x, z

    def solve(
        self,
        x: float,
        z: float,
        *,
        q1_limits: Sequence[float] | None = None,
        q2_limits: Sequence[float] | None = None,
        tolerance: float = 1e-3,
    ) -> PlanarIKResult:
        """对平面内某目标点解析求解该链。

        首先把目标平移到基座坐标系（``xr = x - base_x``，``zr = z - base_z``），
        使整个解就是基座位于原点的链的教科书解，然后：

        1. 把 ``r = hypot(xr, zr)`` 与链实际可达的圆环区间
           ``|l1 - l2| <= r <= l1 + l2`` 比较；
        2. 对基座-肘-腕三角形使用余弦定理得到
           ``cos(q2) = (r^2 - l1^2 - l2^2) / (2 l1 l2)``，进而得到肘角幅值
           ``|q2| = acos(clip(cos(q2), -1, 1))``。该 clip 并非装饰：在工作空间
           最边缘处，分子会以若干 ULP 超出分母，而 ``acos`` 作用于
           ``1.0000000000000002`` 会得到 NaN，并污染每一条关节指令；
        3. 肘的**符号**由 ``q2_limits`` 选出——当关节无法正向弯曲
           （``upper <= 0``）时优先取负分支，否则优先取正分支——若第一个分支
           落在行程范围之外，则尝试另一个分支；
        4. ``q1 = atan2(xr, zr) - atan2(l2 sin(q2), l1 + l2 cos(q2))``：第一项
           是从基座看目标的方向，第二项是折叠的第二个连杆使腕偏离该方向的
           旋转量，减去它即可正确指向连杆 1。

        参数：
            x: 目标的水平坐标，单位米。
            z: 目标的竖直坐标，单位米。
            q1_limits: 第一个关节的 ``(lower, upper)`` 行程范围，单位弧度；
                为 ``None`` 表示无限制。
            q2_limits: 第二个关节的 ``(lower, upper)`` 行程范围，单位弧度；
                为 ``None`` 表示无限制。**它正是选择肘分支的依据**；不传该
                参数表示“两个分支均可”，此时返回正分支。
            tolerance: 可达性测试的裕量，单位米。因浮点噪声而超出工作空间几
                微米的目标仍应能求解。

        返回：
            一个 :class:`PlanarIKResult`。失败时 ``message`` 会说明是哪项测试
            失败，并引用相关数值（``r``、``l1 + l2``、两个候选 ``q2`` 值以及
            被违反的范围），从而无需重新计算即可记录并处理该失败。

        异常：
            TypeError: 当某坐标、限位对或 ``tolerance`` 的类型非数值时抛出。
            ValueError: 当某坐标或 ``tolerance`` 非有限，或限位对格式错误/
                上下界颠倒时抛出。

        示例：
            >>> ik = PlanarTwoLinkIK(PlanarTwoLink(0.25, 0.25, base_z=0.5))
            >>> ik.solve(0.0, 0.75, q2_limits=(-2.5, 0.0)).q1 > 0.0
            True
            >>> ik.solve(0.0, 5.0).message.count("unreachable")
            1
            >>> ik.solve(0.0, 0.75, q2_limits=(-1.0, -0.5)).success
            False
        """
        target_x = _finite_float(x, "x")
        target_z = _finite_float(z, "z")
        slack = _non_negative_float(tolerance, "tolerance")
        limits1 = _limits_pair(q1_limits, "q1_limits")
        limits2 = _limits_pair(q2_limits, "q2_limits")

        l1 = self.geometry.l1
        l2 = self.geometry.l2
        xr = target_x - self.geometry.base_x
        zr = target_z - self.geometry.base_z
        r = math.hypot(xr, zr)
        reach = l1 + l2
        inner = abs(l1 - l2)
        if r > reach + slack or r < inner - slack:
            message = (
                f"target unreachable: r={r:.6f} m from the base must lie within "
                f"[|l1-l2|={inner:.6f}, l1+l2={reach:.6f}] m"
            )
            logger.debug("PlanarTwoLinkIK.solve: %s", message)
            return PlanarIKResult(False, message=message, reachable_distance=r)

        cos_q2 = (r * r - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        q2_mag = math.acos(float(np.clip(cos_q2, -1.0, 1.0)))

        # 限位决定肘分支：无法正向弯曲的关节必须向另一侧折叠，而对称关节在
        # 不超出其行程范围时取正分支。
        branches = (-q2_mag, q2_mag) if limits2[1] <= 0.0 else (q2_mag, -q2_mag)
        q2 = next(
            (branch for branch in branches if _inside_limits(branch, limits2)),
            None,
        )
        if q2 is None:
            message = (
                f"q2 out of limits: both elbow branches ({branches[0]:.6f}, "
                f"{branches[1]:.6f}) violate q2_limits [{limits2[0]:.6f}, "
                f"{limits2[1]:.6f}] for r={r:.6f}"
            )
            logger.debug("PlanarTwoLinkIK.solve: %s", message)
            return PlanarIKResult(False, message=message, reachable_distance=r)

        q1 = math.atan2(xr, zr) - math.atan2(
            l2 * math.sin(q2), l1 + l2 * math.cos(q2)
        )
        if not _inside_limits(q1, limits1):
            message = (
                f"q1={q1:.6f} outside q1_limits [{limits1[0]:.6f}, "
                f"{limits1[1]:.6f}]"
            )
            logger.debug("PlanarTwoLinkIK.solve: %s", message)
            return PlanarIKResult(
                False, q1=q1, q2=q2, message=message, reachable_distance=r
            )
        return PlanarIKResult(True, q1=q1, q2=q2, message="ok", reachable_distance=r)

    def verify(self, result: PlanarIKResult, x: float, z: float) -> float:
        """返回一次求解的位置残差，用于往返检验。

        每次逆运动学求解都应把其答案回代正运动学进行验证：它能立刻发现符号错误、
        分支选错以及角度约定搞错的问题，而代价仅是两对 ``sin``/``cos``。返回
        残差而非布尔值，是因为真正有意义的问题不是“它是否恰好为零”，而是“它是
        否低于本次运动所需的容差”。

        参数：
            result: 一个 :class:`PlanarIKResult`；无论它是否被标记为成功，都会
                使用其角度，因此失败的求解也能被检查。
            x: 目标的水平坐标，单位米。
            z: 目标的竖直坐标，单位米。

        返回：
            结果角度经 :meth:`forward` 得到的点与目标之间的欧氏距离，单位米。

        异常：
            TypeError: 当 ``result`` 不是 :class:`PlanarIKResult`，或某坐标
                不是实数时抛出。
            ValueError: 当某坐标非有限时抛出。

        示例：
            >>> ik = PlanarTwoLinkIK(PlanarTwoLink(0.25, 0.25, base_z=0.5))
            >>> result = ik.solve(0.15, 0.70, q2_limits=(-2.5, 0.0))
            >>> ik.verify(result, 0.15, 0.70) < 1e-12
            True
            >>> ik.verify(PlanarIKResult(True, 0.0, 0.0, "ok"), 0.15, 0.70) > 0.1
            True
        """
        if not isinstance(result, PlanarIKResult):
            raise TypeError(
                f"result must be a PlanarIKResult, got {result!r} "
                f"({type(result).__name__})"
            )
        target_x = _finite_float(x, "x")
        target_z = _finite_float(z, "z")
        fx, fz = self.forward(result.q1, result.q2)
        return math.hypot(fx - target_x, fz - target_z)

    def solve_numeric(
        self,
        x: float,
        z: float,
        *,
        q1_limits: Sequence[float] | None = None,
        q2_limits: Sequence[float] | None = None,
        q0: ArrayLike | None = None,
        tol: float = 1e-3,
        forward_fn: Callable[[float, float], Any] | None = None,
    ) -> PlanarIKResult:
        """用带边界的非线性最小二乘求解同一目标。

        只要链确实就是平面内的两个连杆，就应使用解析的 :meth:`solve`——它精确、
        只需几微秒，且没有收敛性需要担心。当解析式无法描述该机器时才使用*本*
        方法，实践中这意味着一种情形：连杆偏置并非纯粹沿连杆方向的链。若髋的
        关节原点偏离俯仰轴几厘米，末端点作为 ``q`` 的函数就不再是
        ``l1 sin(q1) + l2 sin(q1 + q2)``，也就没有闭式解可写。把该链自己的
        正运动学作为 ``forward_fn`` 传入，同样这一套感知限位、可验证的接口依然
        可用。

        当需要的是*代价*而非精确目标时（朝一个不可达点尽量伸展），它也是合适的
        工具，因为带边界的最小二乘会自然地停在最接近的可允许姿态上，而不是失败。

        参数：
            x: 目标的水平坐标，单位米。
            z: 目标的竖直坐标，单位米。
            q1_limits: 第一个关节的 ``(lower, upper)`` 行程范围，单位弧度；
                为 ``None`` 表示无限制。用作求解器边界，因此返回的角度保证满足
                它们。
            q2_limits: 第二个关节的 ``(lower, upper)`` 行程范围，单位弧度；
                为 ``None`` 表示无限制。
            q0: 可选的 ``(2,)`` 初始猜测。``None`` 时取每个行程范围的中点作为
                种子，这是最稳健的中性选择；传入的猜测会被裁剪进边界内，因为
                求解器不接受不可行的起点。
            tol: 可接受的残差，单位米。
            forward_fn: 可选的 ``f(q1, q2) -> (x, z)``，用以替代
                :meth:`forward`，适用于解析模型无法描述的链。

        返回：
            一个 :class:`PlanarIKResult`，未收敛时其 ``message`` 报告残差。

        异常：
            ImportError: 当未安装 SciPy 时抛出。
            TypeError: 当某参数类型非数值，或 ``forward_fn`` 不可调用时抛出。
            ValueError: 当某坐标/``tol`` 非有限、限位对格式错误、``q0`` 不含
                两个数值，或 ``forward_fn`` 未返回两个值时抛出。

        示例：
            >>> ik = PlanarTwoLinkIK(PlanarTwoLink(0.25, 0.25, base_z=0.5))
            >>> result = ik.solve_numeric(0.0, 0.75, q1_limits=(-1.5, 1.5),
            ...                           q2_limits=(-2.5, 0.0))
            >>> result.success
            True
            >>> ik.verify(result, 0.0, 0.75) < 1e-6
            True
        """
        try:
            from scipy.optimize import least_squares  # noqa: PLC0415 - 有意惰性导入
        except ImportError as exc:  # pragma: no cover - 取决于运行环境
            raise ImportError(
                "solve_numeric needs SciPy for the bounded least-squares solve; "
                "install it with `pip install scipy`"
            ) from exc

        target_x = _finite_float(x, "x")
        target_z = _finite_float(z, "z")
        accepted = _positive_float(tol, "tol")
        limits1 = _limits_pair(q1_limits, "q1_limits")
        limits2 = _limits_pair(q2_limits, "q2_limits")
        kinematics = self.forward
        if forward_fn is not None:
            if not callable(forward_fn):
                raise TypeError(
                    f"forward_fn must be callable with signature f(q1, q2) -> "
                    f"(x, z), got {forward_fn!r} ({type(forward_fn).__name__})"
                )
            kinematics = forward_fn

        target = np.array([target_x, target_z], dtype=np.float64)
        r = math.hypot(target_x - self.geometry.base_x, target_z - self.geometry.base_z)

        def residual(q: np.ndarray) -> np.ndarray:
            out = np.asarray(kinematics(float(q[0]), float(q[1])), dtype=np.float64).reshape(-1)
            if out.size != 2:
                raise ValueError(
                    f"forward_fn must return exactly 2 values (x, z), got {out.size}"
                )
            return out - target

        lower = np.array([limits1[0], limits2[0]], dtype=np.float64)
        upper = np.array([limits1[1], limits2[1]], dtype=np.float64)
        if q0 is None:
            start = np.array([_limit_seed(limits1), _limit_seed(limits2)], dtype=np.float64)
        else:
            start = np.clip(_as_vector(q0, 2, "q0"), lower, upper)

        solution = least_squares(residual, start, bounds=(lower, upper))
        error = float(np.linalg.norm(solution.fun))
        q1 = float(solution.x[0])
        q2 = float(solution.x[1])
        if error <= accepted:
            return PlanarIKResult(True, q1=q1, q2=q2, message="ok", reachable_distance=r)
        message = (
            f"numeric IK did not converge: residual {error:.6e} m > tol "
            f"{accepted:.6e} m (scipy success={solution.success})"
        )
        logger.debug("PlanarTwoLinkIK.solve_numeric: %s", message)
        return PlanarIKResult(
            False, q1=q1, q2=q2, message=message, reachable_distance=r
        )


# -- 肘圆：7 自由度手臂的冗余自由度 ---------------------------------------


def elbow_circle(
    l1: float,
    l2: float,
    shoulder: ArrayLike,
    wrist: ArrayLike,
) -> tuple[np.ndarray, float, np.ndarray, np.ndarray] | None:
    """返回双骨骼链的肘部可能所在的圆。

    这就是 7 自由度冗余的参数化方式。一条 7 自由度手臂被要求在某个 6 自由度位姿
    下放置手时，会多出一个自由度，而这并非抽象概念：当肩与腕被固定，上臂与前臂
    的长度就是固定的，于是肘被约束在两个球面的交线上——一个平面垂直于肩-腕
    连线的**圆**。该圆上的每一点都是实现同一手部位姿的合法手臂构型。因此，在圆
    上选一点*就是*选择冗余自由度，“自然姿态”“远离躯干”“像人一样肘上抬”等
    种种概念都可归结为在圆上选取某个角度的规则。随后针对选定的肘点求解其余关节，
    就化为一组普通的 3 自由度问题。

    构造方式是标准的：令 ``d = |wrist - shoulder|``，由两个直角三角形可得肩到
    圆心沿肩-腕轴的距离 ``a = (l1^2 - l2^2 + d^2) / (2 d)``，半径就是余下的
    那条直角边 ``sqrt(l1^2 - a^2)``。平面内基向量 ``u``、``v`` 通过把该轴与
    与它最不共线的世界坐标轴做叉积得到，因此该基向量组良态且确定（相同输入恒
    给出相同的 ``u``）。

    参数：
        l1: 上臂（肩到肘）长度，单位米，严格为正。
        l2: 前臂（肘到腕）长度，单位米，严格为正。
        shoulder: ``(3,)`` 肩位置。
        wrist: ``(3,)`` 腕位置。

    返回：
        ``(centre, radius, u, v)``，其中 ``centre`` 为 ``(3,)`` 圆心，
        ``radius`` 为其半径（单位米），``u``/``v`` 为圆所在平面的一组正交
        归一基。参数角 ``phi`` 处的点为
        ``centre + radius * (cos(phi) * u + sin(phi) * v)``。当三角不等式不
        成立（``d >= l1 + l2`` 或 ``d <= |l1 - l2|``）时返回 ``None``，即腕
        不可达，或恰好处于圆坍缩为一点、已无可选冗余的边界上。

    异常：
        TypeError: 当某长度不是实数，或某位置不是类数组对象时抛出。
        ValueError: 当某长度不为严格正，或某位置并非包含三个有限数值时抛出。

    示例：
        >>> import numpy as np
        >>> centre, radius, u, v = elbow_circle(0.3, 0.3, [0.0, 0.0, 1.0],
        ...                                     [0.3, 0.0, 1.0])
        >>> np.allclose(centre, [0.15, 0.0, 1.0]), round(radius, 6)
        (True, 0.259808)
        >>> elbow_circle(0.3, 0.3, [0.0, 0.0, 1.0], [1.0, 0.0, 1.0]) is None
        True
    """
    upper = _positive_float(l1, "l1")
    forearm = _positive_float(l2, "l2")
    start = _as_vector(shoulder, 3, "shoulder")
    end = _as_vector(wrist, 3, "wrist")

    span = end - start
    d = float(np.linalg.norm(span))
    if d < _GEOMETRY_EPS:
        logger.debug(
            "elbow_circle: shoulder %s and wrist %s coincide; no chain axis",
            np.array2string(start, precision=4), np.array2string(end, precision=4),
        )
        return None
    if d > upper + forearm - _GEOMETRY_EPS or d < abs(upper - forearm) + _GEOMETRY_EPS:
        logger.debug(
            "elbow_circle: |wrist - shoulder| = %.6f m lies outside the open "
            "interval (|l1-l2|=%.6f, l1+l2=%.6f); the elbow locus degenerates",
            d, abs(upper - forearm), upper + forearm,
        )
        return None

    along = (upper * upper - forearm * forearm + d * d) / (2.0 * d)
    squared = upper * upper - along * along
    if squared < 0.0:
        # 仅可能因舍入而出现：上面的可达性测试已蕴含 squared >= 0。这里把微小
        # 的负值钳制为 0 而不是返回 None，好让位于工作空间内一纳米处的腕仍能
        # 得到肘部。
        if squared > -1e-8:
            squared = 0.0
        else:  # pragma: no cover - 由上面的可达性测试保证
            logger.debug("elbow_circle: negative squared radius %.3e", squared)
            return None

    axis = span / d
    hint = np.array([0.0, 0.0, 1.0]) if abs(float(axis[2])) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(axis, hint)
    u = u / float(np.linalg.norm(u))
    v = np.cross(axis, u)
    return start + along * axis, math.sqrt(squared), u, v


def _point_on_circle(
    centre: np.ndarray,
    radius: float,
    u: np.ndarray,
    v: np.ndarray,
    phi: float,
) -> np.ndarray:
    """求参数角 ``phi`` 处的圆上点。

    参数：
        centre: ``(3,)`` 圆心。
        radius: 圆半径。
        u: ``(3,)`` 平面内第一个基向量。
        v: ``(3,)`` 平面内第二个基向量。
        phi: 参数角，单位弧度。

    返回：
        ``(3,)`` 的 ``float64`` 点 ``centre + radius * (cos(phi) u + sin(phi) v)``。
    """
    return centre + radius * (math.cos(phi) * u + math.sin(phi) * v)


def lowest_point_on_circle(
    centre: ArrayLike,
    radius: float,
    u: ArrayLike,
    v: ArrayLike,
) -> np.ndarray:
    """返回圆上使世界坐标 ``z`` 最小的点。

    把圆写作 ``p(phi) = c + r (cos(phi) u + sin(phi) v)``，其高度为
    ``z(phi) = c_z + r (u_z cos(phi) + v_z sin(phi))``。任何形如
    ``a cos(phi) + b sin(phi)`` 的表达式都是单个正弦量
    ``hypot(a, b) cos(phi - alpha)``，其中 ``alpha = atan2(b, a)``，因此它在
    ``phi = alpha`` 处*取最大*，在 ``phi = alpha + pi`` 处*取最小*——这正是那
    个 ``+ pi`` 的由来，也是最容易出错的地方：直接用 ``alpha`` 会返回**最高**的
    肘，而且在一系列只检查“点是否在圆上”的测试中看起来都合理。

    “最低肘”是人类手臂自然的休息姿态，也是让前臂远离躯干的姿态，因此它是
    :func:`select_elbow_on_circle` 的默认选择。

    参数：
        centre: ``(3,)`` 圆心。
        radius: 圆半径，非负。
        u: ``(3,)`` 平面内第一个基向量。
        v: ``(3,)`` 平面内第二个基向量。

    返回：
        圆上 ``z`` 最小的 ``(3,)`` ``float64`` 点。

    异常：
        TypeError: 当某参数不是类数组对象，或 ``radius`` 不是实数时抛出。
        ValueError: 当某向量并非包含三个有限数值，或 ``radius`` 为负时抛出。

    示例：
        >>> import numpy as np
        >>> centre, radius, u, v = elbow_circle(0.3, 0.3, [0.0, 0.0, 1.0],
        ...                                     [0.3, 0.0, 1.0])
        >>> np.allclose(lowest_point_on_circle(centre, radius, u, v),
        ...             [0.15, 0.0, 1.0 - radius], atol=1e-12)
        True
    """
    c = _as_vector(centre, 3, "centre")
    r = _non_negative_float(radius, "radius")
    basis_u = _as_vector(u, 3, "u")
    basis_v = _as_vector(v, 3, "v")
    phi = math.atan2(float(basis_v[2]), float(basis_u[2])) + math.pi
    return _point_on_circle(c, r, basis_u, basis_v, phi)


def circle_plane_intersections(
    centre: ArrayLike,
    radius: float,
    u: ArrayLike,
    v: ArrayLike,
    *,
    plane_axis: int,
    plane_value: float,
) -> list[float]:
    """返回圆与某轴对齐平面相交处的参数角。

    该平面为 ``{ p : p[plane_axis] == plane_value }``，即垂直于某世界坐标轴——
    这正是肢体自碰撞约束通常的表述方式（“肘必须保持在肩平面 ``y = y_s`` 的
    外侧”，“膝必须保持在髋平面 ``x = x_h`` 的前方”）。

    把圆的表达式代入平面方程会得到单个正弦量
    ``r (u_a cos(phi) + v_a sin(phi)) = plane_value - centre[a]``，即
    ``R cos(phi - alpha) = c``，其中 ``R = r * hypot(u_a, v_a)``，
    ``alpha = atan2(r v_a, r u_a)``。它有零个、一个或两个解：

    * ``|c| > R``——平面与圆不相交：**空列表**。
    * ``|c| == R``——平面与圆相切：**一个角度**，当圆从平面下方触及平面时为
      ``alpha``，从上方触及时为 ``alpha + pi``。此处若在两者中选错，会把肘放到
      约束的反侧，而约束存在的意义正是防止这一点。
    * ``|c| < R``——两个角度，``alpha +- acos(c / R)``。

    参数：
        centre: ``(3,)`` 圆心。
        radius: 圆半径，非负。
        u: ``(3,)`` 平面内第一个基向量。
        v: ``(3,)`` 平面内第二个基向量。
        plane_axis: 平面所垂直的世界坐标轴（0/1/2）。
        plane_value: 平面沿 ``plane_axis`` 的坐标。

    返回：
        参数角列表，单位弧度（不做规范化），长度为 0、1 或 2。可用
        ``centre + radius * (cos(phi) * u + sin(phi) * v)`` 求值。当圆完全
        位于平面*之内*时，解集是整个圆，此时返回空列表，因为任何有限列表都无法
        表示它。

    异常：
        TypeError: 当某参数不是类数组对象，或某标量不是实数时抛出。
        ValueError: 当某向量并非包含三个有限数值、``radius`` 为负，或
            ``plane_axis`` 不是 0/1/2 时抛出。

    示例：
        >>> import numpy as np
        >>> centre, radius, u, v = elbow_circle(0.3, 0.3, [0.0, 0.0, 1.0],
        ...                                     [0.3, 0.0, 1.0])
        >>> phis = circle_plane_intersections(centre, radius, u, v,
        ...                                   plane_axis=1, plane_value=-0.2)
        >>> len(phis)
        2
        >>> on_plane = [centre + radius * (np.cos(t) * u + np.sin(t) * v)
        ...             for t in phis]
        >>> all(abs(float(p[1]) + 0.2) < 1e-9 for p in on_plane)
        True
        >>> circle_plane_intersections(centre, radius, u, v, plane_axis=1,
        ...                            plane_value=-5.0)
        []
    """
    axis = _axis_index(plane_axis, "plane_axis")
    value = _finite_float(plane_value, "plane_value")
    c = _as_vector(centre, 3, "centre")
    r = _non_negative_float(radius, "radius")
    basis_u = _as_vector(u, 3, "u")
    basis_v = _as_vector(v, 3, "v")

    amplitude_u = r * float(basis_u[axis])
    amplitude_v = r * float(basis_v[axis])
    offset = value - float(c[axis])
    amplitude = math.hypot(amplitude_u, amplitude_v)
    if amplitude < 1e-12:
        if abs(offset) < 1e-9:
            logger.warning(
                "circle_plane_intersections: the whole circle lies in the plane "
                "axis=%d value=%.6f; infinitely many intersections, returning "
                "none", axis, value,
            )
        else:
            logger.debug(
                "circle_plane_intersections: circle is parallel to plane "
                "axis=%d value=%.6f and does not meet it", axis, value,
            )
        return []
    if abs(offset) > amplitude + 1e-9:
        return []

    alpha = math.atan2(amplitude_v, amplitude_u)
    if abs(offset) >= amplitude - 1e-9:
        # 相切：唯一的接触点位于平面正侧时为 alpha，位于负侧时为 alpha + pi。
        return [alpha if offset > 0.0 else alpha + math.pi]
    delta = math.acos(float(np.clip(offset / amplitude, -1.0, 1.0)))
    return [alpha + delta, alpha - delta]


def select_elbow_on_circle(
    l1: float,
    l2: float,
    shoulder: ArrayLike,
    wrist: ArrayLike,
    *,
    plane_axis: int,
    plane_value: float,
    outside_test: Callable[[np.ndarray], Any],
    target_forward_axis: int = 0,
) -> np.ndarray | None:
    """为 7 自由度手臂选择冗余的肘部位置。

    决策规则按应用顺序如下：

    1. **取最低的肘。** ``lowest_point_on_circle`` 给出的是放松状态下人类手臂
       所采取的姿态，从力学上讲，也是让前臂自然垂落、远离躯干而非折向躯干的
       姿态。
    2. **若它已满足 ``outside_test``，则保留它。** 该约束是单侧安全规则，满足
       它的姿态无需修正——而即便如此仍去修正，只会让肘偏离自然姿态且毫无益处。
    3. **否则把肘滑动到约束平面上**，并在（至多两个）交点之间选择：当腕沿
       ``target_forward_axis`` 位于肩的前方时取*较低*的那个，位于后方时取
       *较高*的那个。

    第 3 步的物理意义正是让手臂看起来自然的关键。被强制置于躯干平面外侧的肘无法
    再竖直下垂；它必须向前或向后摆动才能留在该平面上。当手向**前**伸时，把肘向
    后下方摆动会使它位于肩的后下方——这正是人推东西或在身前递物时所采用的构型，
    也是让上臂不压向胸口的构型。当手向**后**伸时，则是镜像情形：肘必须向前上方
    抬起，这正是人伸手进后袋或将某物从肩上方抛过时的动作。在这两者中选错，不仅
    看起来别扭——它会让上臂横跨躯干，随后的动作就会把它驱向身体。

    参数：
        l1: 上臂长度，单位米，严格为正。
        l2: 前臂长度，单位米，严格为正。
        shoulder: ``(3,)`` 肩位置。
        wrist: ``(3,)`` 腕位置。
        plane_axis: 自碰撞平面所垂直的世界坐标轴（0/1/2）；对于 ``y = const``
            的躯干平面通常为 ``1``。
        plane_value: 该平面沿 ``plane_axis`` 的坐标。请在身体原始坐标上加上
            所需的间隙：平面应位于躯干之*外*，而非贴在躯干上。
        outside_test: 接收 ``(3,)`` 肘位置的可调用对象，当肘位于平面可接受一侧
            时返回真值，例如对间隙朝向 ``+y`` 的左臂可用
            ``(lambda e: e[1] >= plane_value)``。它之所以是参数而非直接比较，
            是因为对对称机器人的一侧而言“外侧”是 ``>=``，对另一侧则是 ``<=``。
        target_forward_axis: 用于度量“腕位于肩前方”的世界坐标轴（默认 ``0``，
            即 ``x``）。

    返回：
        ``(3,)`` 的 ``float64`` 肘位置；当以这些连杆长度无法达到该腕时返回
        ``None``（见 :func:`elbow_circle`）。当圆始终不与约束平面相交时，返回
        最低的肘并记录一条警告：无法满足的间隙约束不应让手臂瘫软，但调用方理应
        知情。

    异常：
        TypeError: 当某长度不是实数、某位置不是类数组对象，或 ``outside_test``
            不可调用时抛出。
        ValueError: 当某长度不为严格正、某位置并非包含三个有限数值，或某轴
            索引不是 0/1/2 时抛出。

    示例：
        >>> import numpy as np
        >>> arm = dict(l1=0.3, l2=0.3, shoulder=[0.0, 0.0, 1.0],
        ...            wrist=[0.3, 0.0, 1.0])
        >>> elbow = select_elbow_on_circle(
        ...     plane_axis=1, plane_value=-0.2,
        ...     outside_test=lambda e: e[1] <= -0.2, **arm)
        >>> round(float(elbow[1]), 6), float(elbow[2]) < 1.0
        (-0.2, True)
        >>> relaxed = select_elbow_on_circle(
        ...     plane_axis=1, plane_value=-0.2,
        ...     outside_test=lambda e: e[1] <= 0.5, **arm)
        >>> abs(float(relaxed[1])) < 1e-12              # 已在外部：保留
        True
        >>> select_elbow_on_circle(0.3, 0.3, [0.0, 0.0, 1.0], [9.0, 0.0, 1.0],
        ...                        plane_axis=1, plane_value=0.0,
        ...                        outside_test=lambda e: True) is None
        True
    """
    if not callable(outside_test):
        raise TypeError(
            f"outside_test must be a callable taking a (3,) elbow position, got "
            f"{outside_test!r} ({type(outside_test).__name__})"
        )
    axis = _axis_index(plane_axis, "plane_axis")
    value = _finite_float(plane_value, "plane_value")
    forward_axis = _axis_index(target_forward_axis, "target_forward_axis")
    start = _as_vector(shoulder, 3, "shoulder")
    end = _as_vector(wrist, 3, "wrist")

    geometry = elbow_circle(l1, l2, start, end)
    if geometry is None:
        return None
    centre, radius, u, v = geometry

    lowest = lowest_point_on_circle(centre, radius, u, v)
    if outside_test(lowest):
        return lowest

    angles = circle_plane_intersections(
        centre, radius, u, v, plane_axis=axis, plane_value=value
    )
    if not angles:
        logger.warning(
            "select_elbow_on_circle: the elbow circle (centre %s, radius %.4f) "
            "never crosses the plane axis=%d value=%.6f; keeping the lowest "
            "elbow %s, which violates the constraint",
            np.array2string(centre, precision=4), radius, axis, value,
            np.array2string(lowest, precision=4),
        )
        return lowest

    candidates = [_point_on_circle(centre, radius, u, v, phi) for phi in angles]
    if len(candidates) == 1:
        return candidates[0]

    # 向前伸取用低肘，向后伸取用高肘；其物理含义见函数级说明。
    reaching_forward = float(end[forward_axis] - start[forward_axis]) > 0.0
    if reaching_forward:
        return min(candidates, key=lambda point: float(point[2]))
    return max(candidates, key=lambda point: float(point[2]))


# -- 非线性最小二乘 (nonlinear least squares) -----------------------------


def numeric_jacobian(
    fn: Callable[[np.ndarray], Any],
    q: ArrayLike,
    *,
    eps: float = 1e-6,
    output_dim: int | None = None,
) -> np.ndarray:
    """用中心差分近似 ``fn`` 在 ``q`` 处的雅可比（Jacobian）。

    这里使用中心差分（``(f(q + eps e_i) - f(q - eps e_i)) / (2 eps)``）而非
    前向差分，因为其截断误差为 ``O(eps^2)`` 而非 ``O(eps)``：在 ``eps = 1e-6``
    时截断带来的相对误差约为 ``1e-12``，只剩下舍入误差是真正的限制。通常的
    权衡同样适用——``eps`` 不能小到逼近机器精度，否则两个几乎相等的值相减会
    毁掉所有有效位；对弧度/米而言 ``1e-6`` 是标准的折中值。

    它的存在使求解器可以构建在*任意*正运动学函数之上，而无需推导其解析雅可比；
    对于关节耦合或带偏置的链而言，那正是大部分工作（以及大部分 bug）所在。

    参数：
        fn: 把 ``(n,)`` 参数向量映射为 ``(m,)`` 输出的可调用对象。
        q: ``(n,)`` 求值点。
        eps: 有限差分步长，严格为正。
        output_dim: 期望的 ``m``；给定时会用它校验 ``fn(q)`` 的长度，使得
            悄然返回错误数量值的前向模型在此处就失败，而不是产出一个形状错误
            的雅可比。

    返回：
        ``(m, n)`` 的 ``float64`` 雅可比，其第 ``i`` 列为 ``fn`` 关于
        ``q[i]`` 的偏导数。

    异常：
        TypeError: 当 ``fn`` 不可调用、``q`` 不是类数组对象，或
            ``eps``/``output_dim`` 不是实数/整数时抛出。
        ValueError: 当 ``q`` 为空或非有限、``eps <= 0``、``fn`` 返回空向量或
            尺寸不同的向量，或 ``output_dim`` 不匹配时抛出。

    示例：
        >>> import numpy as np
        >>> J = numeric_jacobian(lambda v: np.array([v[0] ** 2, v[0] * v[1]]),
        ...                      np.array([2.0, 3.0]))
        >>> np.allclose(J, [[4.0, 0.0], [3.0, 2.0]], atol=1e-6)
        True
    """
    if not callable(fn):
        raise TypeError(f"fn must be callable, got {fn!r} ({type(fn).__name__})")
    point = _as_vector_flat(q, "q")
    step = _positive_float(eps, "eps")

    base = np.asarray(fn(point), dtype=np.float64).reshape(-1)
    if base.size == 0:
        raise ValueError(f"fn must return a non-empty vector, got {base.tolist()!r}")
    if output_dim is not None:
        expected = _count_int(output_dim, "output_dim")
        if base.size != expected:
            raise ValueError(
                f"fn returned {base.size} value(s) but output_dim={output_dim!r} "
                f"was requested"
            )
    rows = base.size

    jacobian = np.empty((rows, point.size), dtype=np.float64)
    for index in range(point.size):
        plus = point.copy()
        minus = point.copy()
        plus[index] += step
        minus[index] -= step
        f_plus = np.asarray(fn(plus), dtype=np.float64).reshape(-1)
        f_minus = np.asarray(fn(minus), dtype=np.float64).reshape(-1)
        if f_plus.size != rows or f_minus.size != rows:
            raise ValueError(
                f"fn must return a constant-size vector: got {base.size} at q, "
                f"{f_plus.size} at q + {step} e[{index}] and {f_minus.size} at "
                f"q - {step} e[{index}]"
            )
        jacobian[:, index] = (f_plus - f_minus) / (2.0 * step)
    return jacobian


def levenberg_marquardt(
    residual_fn: Callable[[np.ndarray], Any],
    jacobian_fn: Callable[[np.ndarray], Any],
    q0: ArrayLike,
    *,
    max_iters: int = 20,
    tol: float = 1e-4,
    damping: float = 1e-3,
    damping_up: float = 4.0,
    damping_down: float = 0.3,
    damping_min: float = 1e-7,
    damping_max: float = 10.0,
    line_search_steps: int = 8,
    clip_fn: Callable[[np.ndarray], Any] | None = None,
) -> tuple[np.ndarray, float, bool]:
    """用带线性搜索的 Levenberg-Marquardt（LM）方法最小化 ``||residual_fn(q)||``。

    LM 通过单个标量 ``lambda`` 在两个极端之间插值：当 ``lambda -> 0`` 时，步长
    由 ``(J^T J + lambda I) dq = -J^T r`` 给出，即 Gauss-Newton 步，它在解附近
    二次收敛，但在远离解时可能极不稳定；当 ``lambda -> inf`` 时它退化为步长极小
    的梯度下降，总能取得进展但极其缓慢。该方法的全部价值就在于自动让 ``lambda``
    在这些状态之间移动：

    * **接受步**——模型有效，于是更信任它：``lambda`` 乘以 ``damping_down``
      （``0.3``），并以 ``damping_min`` 钳制，下一次迭代将采取更长、更接近
      Newton 的步。
    * **拒绝步**——线性化骗了人，于是更不信任它：``lambda`` 乘以
      ``damping_up``（``4.0``），并以 ``damping_max`` 钳制，下一次尝试将从
      *同一个* ``q`` 出发采取更短、更接近梯度的步。

    步的接受采用针对误差范数的 Armijo 式判据
    ``err_try < err * (1 - 1e-4 * alpha)``：试探点不仅要更好，而且要按与步长
    ``alpha`` 成比例的量更好。若只要求 ``err_try < err``，求解器可能接受一个
    仅把误差改善 1e-16 的步，然后永远停滞，因为它一直在“成功”却没有移动。
    比例项会拒绝这类步，而回溯（把 ``alpha`` 折半至多 ``line_search_steps``
    次）随后会找到一个真正下降的步。

    最后，返回的是**迄今见过的最优迭代点**，而非最后一个迭代点，``converged``
    则报告是否达到容差。控制回路中的精修求解器绝不能让情况变糟：当预算在振荡
    中耗尽时，最后一个迭代点可能明显劣于早先的某个点，返回它就会把“未完全收敛”
    变成“把手往后挪了”。在关键场合，调用方应检查 ``converged`` 并回退到初始
    姿态。

    参数：
        residual_fn: 把 ``(n,)`` 参数映射为 ``(m,)`` 残差向量的可调用对象；
            目标函数是其欧氏范数。
        jacobian_fn: 把 ``(n,)`` 参数映射为 ``residual_fn`` 的 ``(m, n)``
            雅可比的可调用对象（:func:`numeric_jacobian` 可生成一个）。
        q0: ``(n,)`` 初始猜测。
        max_iters: 迭代预算，至少为 1。
        tol: 残差范数的收敛阈值。
        damping: 初始 ``lambda``，严格为正。
        damping_up: 拒绝步之后增大 ``lambda`` 的因子，``> 1``。
        damping_down: 接受步之后缩小 ``lambda`` 的因子，位于 ``(0, 1)``。
        damping_min: ``lambda`` 的下限钳制值。
        damping_max: ``lambda`` 的上限钳制值。
        line_search_steps: 每次迭代对 ``alpha`` 折半的最大次数。
        clip_fn: 可选的可调用对象，在每次对试探 ``q`` 求值前施加，例如用于
            强制关节限位。采用裁剪而非拒绝，能让求解器*沿着*限位滑动，而不是
            从限位上弹开。

    返回：
        ``(best_q, best_err, converged)``：迄今最优迭代点的 ``(n,)``
        ``float64`` 参数、其残差范数（``float``，当没有任何迭代点产生有限残差
        时为 ``inf``），以及 ``best_err <= tol`` 是否成立。

    异常：
        TypeError: 当某可调用参数不可调用、``q0`` 不是类数组对象，或某标量
            参数不是实数时抛出。
        ValueError: 当 ``q0`` 为空或非有限、``max_iters``/``line_search_steps``
            小于 1、``tol`` 为负、``damping_up <= 1``、``damping_down`` 不在
            ``(0, 1)`` 内、``damping_min > damping_max``、某阻尼边界不为正、
            雅可比形状与残差不匹配，或 ``residual_fn`` 返回空向量时抛出。

    示例：
        >>> import numpy as np
        >>> def residual(q):
        ...     return np.array([q[0] ** 2 + q[1] ** 2 - 4.0, q[0] - q[1]])
        >>> def jacobian(q):
        ...     return np.array([[2.0 * q[0], 2.0 * q[1]], [1.0, -1.0]])
        >>> q, err, ok = levenberg_marquardt(residual, jacobian,
        ...                                  np.array([1.0, 1.0]))
        >>> ok, round(float(q[0]), 5), err < 1e-3
        (True, 1.41422, True)
    """
    if not callable(residual_fn):
        raise TypeError(
            f"residual_fn must be callable, got {residual_fn!r} "
            f"({type(residual_fn).__name__})"
        )
    if not callable(jacobian_fn):
        raise TypeError(
            f"jacobian_fn must be callable, got {jacobian_fn!r} "
            f"({type(jacobian_fn).__name__})"
        )
    if clip_fn is not None and not callable(clip_fn):
        raise TypeError(
            f"clip_fn must be None or callable, got {clip_fn!r} "
            f"({type(clip_fn).__name__})"
        )
    q = _as_vector_flat(q0, "q0")
    iterations = _count_int(max_iters, "max_iters")
    threshold = _non_negative_float(tol, "tol")
    lam = _positive_float(damping, "damping")
    grow = _finite_float(damping_up, "damping_up")
    shrink = _finite_float(damping_down, "damping_down")
    lam_min = _positive_float(damping_min, "damping_min")
    lam_max = _positive_float(damping_max, "damping_max")
    if grow <= 1.0:
        raise ValueError(
            f"damping_up must be > 1 so that a rejected step shrinks the next "
            f"one, got {damping_up!r}"
        )
    if not 0.0 < shrink < 1.0:
        raise ValueError(
            f"damping_down must lie within (0, 1) so that an accepted step grows "
            f"the next one, got {damping_down!r}"
        )
    if lam_min > lam_max:
        raise ValueError(
            f"damping_min must be <= damping_max, got damping_min={damping_min!r} "
            f"and damping_max={damping_max!r}"
        )
    halvings = _count_int(line_search_steps, "line_search_steps")

    def evaluate(vector: np.ndarray) -> tuple[float, np.ndarray]:
        """返回 ``residual_fn`` 在 ``vector`` 处的 ``(范数, 残差)``。

        非有限的残差会映射为 ``inf`` 而不是抛出异常：让前向模型穿过奇异位形的
        姿态，在线性搜索中是很正常的求值对象，它只需在比较中落败即可。

        参数：
            vector: ``(n,)`` 参数。

        返回：
            ``(error, residual)``，其中 ``error`` 为残差范数或 ``inf``。

        异常：
            ValueError: 当 ``residual_fn`` 返回空向量时抛出。
        """
        values = np.asarray(residual_fn(vector), dtype=np.float64).reshape(-1)
        if values.size == 0:
            raise ValueError(
                f"residual_fn must return a non-empty vector, got {values.tolist()!r}"
            )
        if not np.all(np.isfinite(values)):
            return math.inf, values
        return float(np.linalg.norm(values)), values

    best_q = q.copy()
    best_err = math.inf
    converged = False

    for _ in range(iterations):
        err, residual = evaluate(q)
        if err < best_err:
            best_err, best_q = err, q.copy()
        if err <= threshold:
            converged = True
            break
        if not math.isfinite(err):
            logger.debug(
                "levenberg_marquardt: residual is not finite at q=%s; stopping",
                np.array2string(q, precision=6),
            )
            break

        jacobian = np.asarray(jacobian_fn(q), dtype=np.float64)
        if jacobian.ndim != 2 or jacobian.shape != (residual.size, q.size):
            raise ValueError(
                f"jacobian_fn must return a ({residual.size}, {q.size}) matrix "
                f"matching the residual and parameter sizes, got shape "
                f"{jacobian.shape}"
            )

        normal = jacobian.T @ jacobian + lam * np.eye(q.size)
        gradient = -(jacobian.T @ residual)
        try:
            step = np.linalg.solve(normal, gradient)
        except np.linalg.LinAlgError:
            step = np.linalg.lstsq(normal, gradient, rcond=None)[0]
        if not np.all(np.isfinite(step)):
            lam = min(lam * grow, lam_max)
            continue

        alpha = 1.0
        accepted = False
        for _ in range(halvings):
            trial = q + alpha * step
            if clip_fn is not None:
                trial = np.asarray(clip_fn(trial), dtype=np.float64).reshape(-1)
                if trial.size != q.size:
                    raise ValueError(
                        f"clip_fn must return {q.size} value(s), got {trial.size}"
                    )
            err_try, _ = evaluate(trial)
            # Armijo 式：改善量必须与步长成比例。
            if err_try < err * (1.0 - _ARMIJO_C * alpha):
                q = trial
                lam = max(lam * shrink, lam_min)
                accepted = True
                break
            alpha *= 0.5
        if not accepted:
            lam = min(lam * grow, lam_max)

    # 循环可能在接受某步后立即退出，使该迭代点未被评分；此处补上，让“最优”
    # 名副其实。
    final_err, _ = evaluate(q)
    if final_err < best_err:
        best_err, best_q = final_err, q.copy()
    if final_err <= threshold:
        converged = True
    if not converged:
        logger.debug(
            "levenberg_marquardt: stopped after %d iteration(s) with best error "
            "%.6e > tol %.6e", iterations, best_err, threshold,
        )
    return best_q, best_err, converged
