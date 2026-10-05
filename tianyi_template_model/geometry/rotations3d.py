"""通用的 3D 旋转工具。

本模块汇总了机器人控制器日常所需的旋转代数：

* 由一到两个*轴约束*构造完整的 3x3 旋转矩阵
  （例如"工具 +z 轴必须指向 -x 方向"）；
* 将一个向量旋转到另一个向量的最小旋转；
* 感知关节限位的角度折叠，使得当某个等价的 2*pi 别名位于其行程范围内时，
  关节不会被命令额外旋转半圈；
* 欧拉角分支选择（Euler angle branch selection），这正是让腕部求解器表现
  平滑、而不是在两个数学上都合法的解之间跳变的原因。

一切均采用右手、列向量约定：对于旋转矩阵 ``R``，第 ``i`` 列是第 ``i`` 个基向量
的像，即 ``R[:, 0]`` 表示局部 ``+x`` 在父坐标系中的最终指向。

依赖：仅 :mod:`numpy`。Rodrigues 公式与欧拉角提取均在此直接实现，因此本模块
不依赖 scipy；不过其约定与 :mod:`scipy.spatial.transform` 一致，如果你已经依赖
scipy，二者可以互换使用。
"""

from __future__ import annotations

import logging
import math
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike

logger = logging.getLogger(__name__)

__all__ = [
    "AXIS_KEYS",
    "assert_rotation_matrix",
    "euler_to_matrix",
    "is_partial_rotation",
    "is_rotation_matrix",
    "matrix_to_euler",
    "normalize_constraints",
    "project_onto_plane",
    "rotation_between",
    "rotation_from_axis",
    "rotation_from_two_axes",
    "select_euler_branches",
    "wrap_to_limits",
]

#: 单个轴约束键可接受的拼写。前导的 ``+``/``-`` 用于选择约束的是机体轴的
#: 哪个*方向*，因此 ``{'z': [-1, 0, 0]}`` 与 ``{'-z': [1, 0, 0]}`` 含义相同。
AXIS_KEYS: frozenset[str] = frozenset(
    {"x", "y", "z", "+x", "+y", "+z", "-x", "-y", "-z"}
)

_TWO_PI: float = 2.0 * math.pi
_EPS: float = 1e-12
_LIMIT_TOL: float = 1e-6
_ALIASES: tuple[int, ...] = (-2, -1, 0, 1, 2)

#: 欧拉角提取的奇异阈值，作用于中间角的正弦（真欧拉 / proper Euler）或
#: 余弦（Tait-Bryan）——即闭式解所除的那个量。
#:
#: 它被有意设置得比机器精度高出许多个数量级，因为 ``acos``/``asin`` 在 ``+-1``
#: 处的导数无穷大：仅被 ``1e-16`` 扰动的输入就会产生约 ``1.4e-8`` 的角度误差。
#: 更严格的阈值会让闭式解以一个约 1e-8 的除数运行，并返回各自毫无意义的
#: 角度（重构误差可达 2.0，即符号翻转）。在 ``1e-6`` 时，双精度噪声的最坏
#: 放大约为 ``1e-10`` 弧度，可以忽略不计，而真正非奇异的位姿不受影响。
_GIMBAL_TOL: float = 1e-6


def _as_vector3(value: ArrayLike, name: str) -> np.ndarray:
    """将 ``value`` 转换为有限的浮点 3 维向量。

    参数:
        value: 任意恰好包含三个元素的类数组对象。
        name: 参数的人类可读名称，用于错误消息。

    返回:
        形状为 ``(3,)`` 的连续 ``float64`` 数组。

    异常:
        TypeError: 如果 ``value`` 不是类数组对象。
        ValueError: 如果 ``value`` 不含三个元素或包含非有限数值。
    """
    try:
        arr = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be array-like, got {value!r}") from exc
    if arr.size != 3:
        raise ValueError(f"{name} must have exactly 3 elements, got {arr.size}: {value!r}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return arr


def _unit(value: ArrayLike, name: str) -> np.ndarray:
    """返回归一化为单位长度的 ``value``。

    参数:
        value: 包含三个元素的类数组对象。
        name: 用于错误消息中的参数名。

    返回:
        形状为 ``(3,)`` 的单位 ``float64`` 向量。

    异常:
        ValueError: 如果该向量为（接近）零向量，因为无法从中恢复出方向。
    """
    vec = _as_vector3(value, name)
    norm = float(np.linalg.norm(vec))
    if norm < 1e-12:
        raise ValueError(f"{name} must not be the zero vector, got {value!r}")
    return vec / norm


def _axis_index(axis: str) -> int:
    """将 ``'-y'`` 这类轴键映射到其列索引 ``0``/``1``/``2``。

    参数:
        axis: 轴键，可选地带 ``+`` 或 ``-`` 前缀。

    返回:
        列索引（``x`` -> 0，``y`` -> 1，``z`` -> 2）。

    异常:
        TypeError: 如果 ``axis`` 不是字符串。
        ValueError: 如果 ``axis`` 不属于 :data:`AXIS_KEYS`。
    """
    if not isinstance(axis, str):
        raise TypeError(f"axis must be a string, got {type(axis).__name__}: {axis!r}")
    key = axis.strip().lower()
    if key not in AXIS_KEYS:
        raise ValueError(
            f"axis must be one of {sorted(AXIS_KEYS)}, got {axis!r}"
        )
    return "xyz".index(key.lstrip("+-"))


def _parse_axis_direction(axis: str, direction: ArrayLike) -> tuple[int, np.ndarray]:
    """将一个轴约束解析为 ``(列索引, 目标单位向量)``。

    ``axis`` 的符号前缀会被折叠进返回的方向中：要求 ``'-z'`` 沿 ``d`` 与要求
    ``'z'`` 沿 ``-d`` 是等价的。

    参数:
        axis: 轴键，例如 ``'z'`` 或 ``'-z'``。
        direction: *正*机体轴的目标方向。其长度无关紧要，只有方向有意义。

    返回:
        元组 ``(index, unit_direction)``，其中 ``index`` 是必须与
        ``unit_direction`` 对齐的矩阵列。

    异常:
        TypeError: 如果 ``axis`` 不是字符串或 ``direction`` 不是类数组对象。
        ValueError: 如果 ``axis`` 未知或 ``direction`` 是零向量。

    示例:
        >>> idx, d = _parse_axis_direction("-z", [1.0, 0.0, 0.0])
        >>> idx, np.allclose(d, [-1.0, 0.0, 0.0])
        (2, True)
    """
    index = _axis_index(axis)
    sign = -1.0 if axis.strip().lower().startswith("-") else 1.0
    return index, sign * _unit(direction, "direction")


def is_partial_rotation(spec: Any) -> bool:
    """判断 ``spec`` 是否为部分（1 个或 2 个轴）旋转约束。

    部分旋转规范（partial rotation spec）是从轴键（:data:`AXIS_KEYS`）到
    3 维向量的映射。它描述一个*不完整*的姿态：一个轴固定一个自由度（绕该轴的
    滚转仍然自由），两个轴则完全固定姿态。三个或更多轴会对 3D 旋转造成过约束，
    因此不是合法的部分规范。

    参数:
        spec: 候选规范。任何不是由轴键构成的非空映射的对象都会返回 ``False``
            而不是抛出异常，这使它可安全地用作对用户输入的类型判别器。

    返回:
        当 ``spec`` 是包含一个或两个轴键的映射时返回 ``True``。

    示例:
        >>> is_partial_rotation({"z": [-1.0, 0.0, 0.0]})
        True
        >>> is_partial_rotation(np.eye(3))
        False
        >>> is_partial_rotation({"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]})
        False
    """
    if not isinstance(spec, Mapping) or isinstance(spec, np.ndarray):
        return False
    if not 1 <= len(spec) <= 2:
        return False
    return all(isinstance(k, str) and k.strip().lower() in AXIS_KEYS for k in spec)


def normalize_constraints(spec: Mapping[str, ArrayLike]) -> list[tuple[str, np.ndarray]]:
    """校验部分旋转规范并返回规范化的轴约束。

    每一项都会被检查，其符号前缀被折叠进方向，方向被归一化。返回的轴名始终是
    裸 ``'x'``/``'y'``/``'z'``，以便下游代码直接索引矩阵列。

    参数:
        spec: 形如 ``{'z': [-1, 0, 0], '-y': [0, 0, 1]}`` 的映射。

    返回:
        一个或两个 ``(axis, unit_direction)`` 元组构成的列表，顺序遵循
        ``spec`` 的迭代顺序。

    异常:
        TypeError: 如果 ``spec`` 不是映射。
        ValueError: 如果 ``spec`` 为空、包含多于两个约束、重复同一轴
            （``'y'`` 与 ``'-y'`` 是同一轴）、使用未知键，或包含零方向。

    示例:
        >>> cons = normalize_constraints({"-z": [2.0, 0.0, 0.0]})
        >>> cons[0][0], np.allclose(cons[0][1], [-1.0, 0.0, 0.0])
        ('z', True)
    """
    if not isinstance(spec, Mapping):
        raise TypeError(
            f"partial rotation spec must be a mapping of axis keys to directions, "
            f"got {type(spec).__name__}: {spec!r}"
        )
    if len(spec) == 0:
        raise ValueError(
            "partial rotation spec is empty; pass at least one axis constraint, "
            "or None to leave the orientation unconstrained"
        )
    if len(spec) > 2:
        raise ValueError(
            f"a 3D orientation is fully determined by 2 axis constraints, got {len(spec)}: "
            f"{sorted(spec)!r}"
        )

    out: list[tuple[str, np.ndarray]] = []
    seen: set[int] = set()
    for axis, direction in spec.items():
        index, unit_dir = _parse_axis_direction(axis, direction)
        if index in seen:
            raise ValueError(
                f"duplicate axis constraint: {axis!r} targets the same body axis as an "
                f"earlier entry; drop one of them"
            )
        seen.add(index)
        out.append(("xyz"[index], unit_dir))
    return out


def _complete_frame(cols: list[np.ndarray | None], free_index: int) -> np.ndarray:
    """用另外两列的循环叉积填充 ``cols[free_index]``。

    循环规则 ``c_i = c_{i+1} x c_{i+2}``（下标对 3 取模）正是 ``[c0, c1, c2]``
    构成右手正交归一基的条件，因此使用它能从构造上保持 ``det(R) = +1``。

    参数:
        cols: 三元素列表；其中两项是单位向量，位于 ``free_index`` 的项为 ``None``。
        free_index: 需要计算的列。

    返回:
        由各列拼装而成的 ``(3, 3)`` 旋转矩阵。

    异常:
        ValueError: 如果两个已知列未全部设置。
    """
    first = cols[(free_index + 1) % 3]
    second = cols[(free_index + 2) % 3]
    if first is None or second is None:
        raise ValueError(
            f"cannot complete column {free_index}: the other two columns must be set, "
            f"got {[None if c is None else 'set' for c in cols]!r}"
        )
    cols[free_index] = _unit(np.cross(first, second), "cross product")
    return np.column_stack(cols)


def rotation_from_two_axes(
    axis0: str,
    dir0: ArrayLike,
    axis1: str,
    dir1: ArrayLike,
) -> np.ndarray:
    """由两个轴约束构造右手旋转矩阵。

    该构造是纯代数的，因此既快速又沿轨迹连续（无需迭代搜索，也不会发生分支
    翻转）：

    1. 主列就是给定的 ``dir0``；
    2. 次列是将 ``dir1`` 投影到垂直于 ``dir0`` 的平面上并重新归一化得到的
       ——保留投影的符号，使次轴仍偏向所要求的 ``dir1``；
    3. 剩余的列由另外两列的循环叉积得到；
    4. 如果拼装后的行列式为负，则翻转次列并重复第 3 步，因此结果始终是
       合法旋转。

    由于次方向只被*近似*满足（它必须与主方向保持正交），``dir0`` 被精确满足，
    而 ``dir1`` 则在正交性允许的范围内被尽可能满足。

    参数:
        axis0: 主机体轴键，例如 ``'z'`` 或 ``'-z'``。
        dir0: ``axis0`` 必须对齐的世界方向。
        axis1: 次机体轴键；必须与 ``axis0`` 不同。
        dir1: ``axis1`` 应当对齐的世界方向。

    返回:
        ``(3, 3)`` 旋转矩阵，其第 ``axis0`` 列等于 ``unit(dir0)``。

    异常:
        TypeError: 如果某个轴不是字符串或某个方向不是类数组对象。
        ValueError: 如果两个约束指向同一轴、两个方向近乎平行
            （``|dot| > 0.98``）以致投影退化，或某个方向是零向量。

    示例:
        >>> R = rotation_from_two_axes("z", [0.0, 0.0, -1.0], "x", [1.0, 0.0, 0.0])
        >>> np.allclose(R[:, 2], [0.0, 0.0, -1.0]) and np.allclose(R[:, 0], [1.0, 0.0, 0.0])
        True
        >>> round(float(np.linalg.det(R)), 6)
        1.0
    """
    i0, d0 = _parse_axis_direction(axis0, dir0)
    i1, d1 = _parse_axis_direction(axis1, dir1)
    if i0 == i1:
        raise ValueError(
            f"the two constraints must target different body axes, both are "
            f"{axis0!r}/{axis1!r} -> '{'xyz'[i0]}'"
        )
    parallelism = abs(float(np.dot(d0, d1)))
    if parallelism > 0.98:
        raise ValueError(
            f"axis constraints are nearly parallel (|dot(dir0, dir1)| = {parallelism:.4f} "
            f"> 0.98): projecting dir1={np.array2string(d1, precision=3)} onto the plane "
            f"perpendicular to dir0={np.array2string(d0, precision=3)} degenerates. "
            f"Pick a secondary direction that is roughly orthogonal to the primary one."
        )

    cols: list[np.ndarray | None] = [None, None, None]
    cols[i0] = d0
    projected = d1 - float(np.dot(d1, d0)) * d0
    secondary = _unit(projected, "projected secondary direction")
    if float(np.dot(secondary, d1)) < 0.0:
        secondary = -secondary
    cols[i1] = secondary

    free = 3 - i0 - i1
    matrix = _complete_frame(cols, free)
    if float(np.linalg.det(matrix)) < 0.0:
        # 防御性处理：循环补全通常保证 det = +1。若数值漂移产生了左手系，
        # 则镜像次轴并重建，而不是返回一个非法旋转。
        logger.debug("two-axis frame came out left-handed; flipping secondary axis")
        cols[i1] = -secondary
        matrix = _complete_frame(cols, free)
    return matrix


def rotation_from_axis(axis: str, direction: ArrayLike) -> np.ndarray:
    """构造将单个机体轴锁定到某方向的旋转矩阵。

    单个约束会让绕该轴的滚转保持自由，因此需要一条补全规则。此处使用的规则在
    ``direction`` 上是确定且连续的：选取一个世界基向量作为提示，将其投影到
    垂直于被锁定轴的平面上，放入第 ``(i + 1) % 3`` 列，最后一列由循环叉积得到。

    除非该提示与锁定的轴近乎平行，否则提示取 ``e[(i + 2) % 3]``；若近乎平行则
    改用 ``e[(i + 1) % 3]``。因此除提示切换附近的狭窄锥形区域外，补全处处平滑，
    这对任何固定参考的补全都是不可避免的（球面上不存在连续的全局标架场
    ——毛球定理 / hairy ball theorem）。

    参数:
        axis: 机体轴键，例如 ``'z'`` 或 ``'-y'``。
        direction: 该轴必须对齐的世界方向。

    返回:
        ``(3, 3)`` 旋转矩阵，满足 ``R[:, index(axis)] == unit(direction)``。

    异常:
        TypeError: 如果 ``axis`` 不是字符串或 ``direction`` 不是类数组对象。
        ValueError: 如果 ``axis`` 未知或 ``direction`` 是零向量。

    示例:
        >>> R = rotation_from_axis("z", [0.0, 0.0, -1.0])
        >>> np.allclose(R[:, 2], [0.0, 0.0, -1.0])
        True
        >>> round(float(np.linalg.det(R)), 6)
        1.0
    """
    index, locked = _parse_axis_direction(axis, direction)

    hint = np.zeros(3)
    hint[(index + 2) % 3] = 1.0
    if abs(float(np.dot(locked, hint))) > 0.9:
        hint = np.zeros(3)
        hint[(index + 1) % 3] = 1.0

    free_dir = hint - float(np.dot(hint, locked)) * locked
    cols: list[np.ndarray | None] = [None, None, None]
    cols[index] = locked
    cols[(index + 1) % 3] = _unit(free_dir, "free axis hint")
    return _complete_frame(cols, (index + 2) % 3)


def _perpendicular_to(vec: np.ndarray) -> np.ndarray:
    """返回一个与 ``vec`` 正交的确定性单位向量。

    将与 ``vec`` 最不对齐的世界轴与 ``vec`` 做叉积；相同情形按索引顺序打破，
    从而使结果可复现。

    参数:
        vec: 单位 3 维向量。

    返回:
        与 ``vec`` 正交的单位 3 维向量。

    异常:
        ValueError: 如果无法构造出垂直向量（仅在输入为零向量时发生）。
    """
    reference = np.zeros(3)
    reference[int(np.argmin(np.abs(vec)))] = 1.0
    return _unit(np.cross(vec, reference), "perpendicular fallback")


def _rotvec_to_matrix(rotvec: ArrayLike) -> np.ndarray:
    """通过 Rodrigues 公式将旋转向量转换为矩阵（仅用 numpy）。

    保持私有且无依赖，使本模块除 numpy 外无需任何东西。
    ``R = I + sin(t) K + (1 - cos t) K^2``，其中 ``K`` 是单位轴的反对称矩阵，
    ``t`` 是旋转角 ``|rotvec|``。

    参数:
        rotvec: 按角度缩放的轴，形状 ``(3,)``。

    返回:
        形状为 ``(3, 3)`` 的 ``float64`` 数组。

    异常:
        ValueError: 如果 ``rotvec`` 不是有限的长度 3 向量。
    """
    v = np.asarray(rotvec, dtype=float).reshape(3)
    if not np.all(np.isfinite(v)):
        raise ValueError(f"rotvec must be finite, got {rotvec!r}")
    angle = float(np.linalg.norm(v))
    if angle < _EPS:
        return np.eye(3)
    axis = v / angle
    k = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def _axis_matrix(axis_index: int, angle: float) -> np.ndarray:
    """绕单个世界轴的基元旋转。

    参数:
        axis_index: ``0`` 表示 X，``1`` 表示 Y，``2`` 表示 Z。
        angle: 旋转角，单位为弧度，右手方向为正。

    返回:
        形状为 ``(3, 3)`` 的 ``float64`` 数组。

    异常:
        ValueError: 如果 ``axis_index`` 不是 0、1 或 2，或 ``angle`` 不是有限值。
    """
    if axis_index not in (0, 1, 2):
        raise ValueError(f"axis_index must be 0 (X), 1 (Y) or 2 (Z), got {axis_index!r}")
    if not math.isfinite(angle):
        raise ValueError(f"angle must be finite, got {angle!r}")
    c, s = math.cos(angle), math.sin(angle)
    m = np.eye(3)
    a, b = (axis_index + 1) % 3, (axis_index + 2) % 3
    m[a, a] = c
    m[a, b] = -s
    m[b, a] = s
    m[b, b] = c
    return m


def _validate_seq(seq: str) -> tuple[tuple[int, int, int], bool]:
    """校验欧拉轴序列并返回轴索引。

    当序列在 ``XYZxyz`` 中命名三个轴且**没有两个相邻轴相等**时，它是合法的。
    连续重复同一个轴（``XXY``、``ZZZ``）是退化的：这两个旋转可交换并塌缩为
    单个自由度，因此该三元组不再参数化 ``SO(3)``，下面的闭式提取将会除以一个
    结构性为零的量。非连续重复（``XYX``、``ZXZ``）属于真欧拉（proper Euler）
    族，完全合法。

    参数:
        seq: 取自 ``XYZxyz`` 的三字符序列。

    返回:
        ``((i, j, k), intrinsic)``，其中每个索引为 X/Y/Z 对应的 0/1/2，
        ``intrinsic`` 对大写序列为 ``True``。

    异常:
        TypeError: 如果 ``seq`` 不是字符串。
        ValueError: 如果 ``seq`` 不是恰好三个取自 ``XYZxyz`` 的字符，
            或者有两个相邻轴相等。

    示例:
        >>> _validate_seq("ZYX")
        ((2, 1, 0), True)
        >>> _validate_seq("zyx")
        ((2, 1, 0), False)
        >>> _validate_seq("XYX")[0]
        (0, 1, 0)
    """
    if not isinstance(seq, str):
        raise TypeError(f"seq must be a string, got {type(seq).__name__}")
    if len(seq) != 3 or any(c not in "XYZxyz" for c in seq):
        raise ValueError(
            f"seq must be a 3 character Euler axis sequence over XYZxyz "
            f"(e.g. 'ZYX' or 'zxz'), got {seq!r}"
        )
    upper = seq.upper()
    if upper[0] == upper[1] or upper[1] == upper[2]:
        raise ValueError(
            f"consecutive Euler axes must differ (e.g. 'ZYX' or 'XYX'), "
            f"got {seq!r}: repeating an axis consecutively is degenerate and "
            f"cannot parameterise a 3D rotation"
        )
    intrinsic = seq.isupper()
    return tuple(int("XYZ".index(c)) for c in upper), intrinsic  # type: ignore[return-value]


def euler_to_matrix(angles: ArrayLike, seq: str = "ZYX") -> np.ndarray:
    """由欧拉角合成旋转矩阵。

    这是 :func:`_matrix_to_euler` 的精确逆运算，并遵循
    :mod:`scipy.spatial.transform` 的约定：**大写** ``seq`` 表示内旋
    （intrinsic rotation，绕运动的机体轴，从左到右施加），**小写** ``seq``
    表示外旋（extrinsic rotation，绕固定的世界轴，每一个都左乘到结果上）。

    提供此函数是为了让调用方无需引入 scipy 即可构造并往返测试旋转。

    参数:
        angles: 三个以弧度表示的角度，顺序与 ``seq`` 匹配。
        seq: 取自 ``XYZxyz`` 的欧拉轴序列。默认为 ``"ZYX"``，即大多数
            机器人技术栈使用的偏航/俯仰/滚转（yaw/pitch/roll）顺序。

    返回:
        形状为 ``(3, 3)`` 的 ``float64`` 数组。

    异常:
        TypeError: 如果 ``angles`` 不是类数组对象或 ``seq`` 不是字符串。
        ValueError: 如果 ``angles`` 不含三个有限值，或 ``seq`` 不是合法的
            三字符轴序列。

    示例:
        对于 ``"ZYX"``，角度顺序为 ``(z, y, x)``，因此纯偏航是*第一个*
        元素::

            >>> import numpy as np
            >>> r = euler_to_matrix([np.pi / 2, 0.0, 0.0], "ZYX")
            >>> np.allclose(r @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
            True
            >>> np.allclose(matrix_to_euler(r, "ZYX"), [np.pi / 2, 0.0, 0.0])
            True
    """
    a = np.asarray(angles, dtype=float).reshape(-1)
    if a.size != 3 or not np.all(np.isfinite(a)):
        raise ValueError(f"angles must be three finite values, got {angles!r}")
    (i, j, k), intrinsic = _validate_seq(seq)
    m_i, m_j, m_k = _axis_matrix(i, a[0]), _axis_matrix(j, a[1]), _axis_matrix(k, a[2])
    if intrinsic:
        return m_i @ m_j @ m_k
    return m_k @ m_j @ m_i


def _solve_gimbal_lock(
    matrix: np.ndarray, seq: str, middle: float, free_index: int
) -> float:
    """在欧拉万向锁处恢复可观测的旋转。

    在万向锁（gimbal lock）处，第一个和第三个角度不再可独立观测——只有它们的
    某个组合可观测——因此分解本质上是多义的。与其为 24 种序列逐一推导符号约定
    （易出错，且是隐蔽 bug 的经典来源），这里将第三个角度固定为零，并选取最能
    复现 ``matrix`` 的第一个角度。

    "最佳"意味着最大化 ``trace(R^T M)``，它关于旋转误差是单调的。任何这样的迹
    都具有 ``A cos(a1) + B sin(a1) + C`` 的形式，因此在 ``a1 = 0, pi/2, pi``
    处的三次求值可精确确定 ``A``、``B``、``C``，其最大化点为 ``atan2(B, A)``。
    无需迭代，无需分类讨论。

    参数:
        matrix: 目标 ``(3, 3)`` 旋转矩阵。
        seq: 取自 ``XYZxyz`` 的欧拉轴序列。
        middle: 被锁定的中间角，单位为弧度。
        free_index: ``angles`` 中保持自由的位置（``0`` 表示保持第一个角度自由
            并将第三个角度置零，这也是此处使用的约定）。

    返回:
        位置 ``free_index`` 处恢复出的角度；另一个自由槽位为 0。

    异常:
        ValueError: 如果 ``free_index`` 不是 0 或 2。
    """
    if free_index not in (0, 2):
        raise ValueError(f"free_index must be 0 or 2, got {free_index!r}")

    def trace_at(a1: float) -> float:
        angles = [0.0, 0.0, 0.0]
        angles[free_index] = a1
        angles[1] = middle
        return float(np.trace(euler_to_matrix(angles, seq).T @ matrix))

    f0, f90, f180 = trace_at(0.0), trace_at(math.pi / 2), trace_at(math.pi)
    c = 0.5 * (f0 + f180)
    a_coef = 0.5 * (f0 - f180)
    b_coef = f90 - c
    if abs(a_coef) < 1e-12 and abs(b_coef) < 1e-12:
        # 两个系数都为零：该矩阵对这个角度同样不携带任何信息。
        # 零与其他任何规范答案一样好。
        return 0.0
    return math.atan2(b_coef, a_coef)


def _matrix_to_euler(matrix: ArrayLike, seq: str) -> np.ndarray:
    """从旋转矩阵提取欧拉角（仅用 numpy）。

    支持 ``XYZxyz`` 上所有三字符轴序列，与
    :mod:`scipy.spatial.transform.Rotation.as_euler` 的约定一致：大写表示
    *内旋*（绕运动的机体轴，从左到右施加），小写表示*外旋*（固定的世界轴）。
    外旋序列通过将其反转、按内旋提取、再反转结果来处理，因为
    ``R_x(a) R_y(b) R_z(c) == R_z(c) R_y(b) R_x(a)``。

    两族需要不同的闭式形式：

    * **Tait-Bryan**（三个轴互不相同，例如 ``ZYX``）：中间角来自单个矩阵元素的
      ``asin``，因此只能确定到一个分支歧义。这正是
      :func:`select_euler_branches` 存在的原因。
    * **真欧拉 / Proper Euler**（首尾轴相同，例如 ``ZXZ``）：中间角来自 ``acos``。

    在万向锁处，第一个和第三个角度塌缩为一个可观测自由度。该情形由
    :func:`_solve_gimbal_lock` 处理，它返回一个仍能重构输入矩阵的规范答案
    （第三个角为零）。需要在万向锁处保持连续性的调用方应跟踪上一个解并优先
    选择最接近的等价解，这正是 :func:`wrap_to_limits` 的用途。

    参数:
        matrix: 旋转矩阵，形状 ``(3, 3)``。
        seq: 取自 ``XYZxyz`` 的三字符轴序列。

    返回:
        形状为 ``(3,)`` 的 ``float64`` 弧度角数组，顺序与 ``seq`` 匹配。

    异常:
        TypeError: 如果 ``matrix`` 不是类数组对象或 ``seq`` 不是字符串。
        ValueError: 如果 ``matrix`` 无法重塑为 ``(3, 3)``，或 ``seq`` 不是
            合法的三字符轴序列。

    示例:
        >>> import numpy as np
        >>> r = euler_to_matrix([0.2, 0.3, 0.4], "ZYX")
        >>> np.allclose(_matrix_to_euler(r, "ZYX"), [0.2, 0.3, 0.4])
        True
    """
    (i, j, k), intrinsic = _validate_seq(seq)
    r = np.asarray(matrix, dtype=float)
    if r.size != 9:
        raise ValueError(f"matrix must be shape (3, 3), got {np.shape(matrix)!r}")
    r = r.reshape(3, 3)

    # 外旋等价于将序列与角度都反转后的内旋。
    work_seq = seq.upper() if intrinsic else seq.upper()[::-1]
    wi, wj, wk = (int("XYZ".index(c)) for c in work_seq)
    # (0, 1, 2) 的偶置换 -> +1，奇置换 -> -1。
    parity = 1 if (wj - wi) % 3 == 1 else -1

    if wi == wk:
        # 真欧拉 / Proper Euler（例如 ZXZ）；l 是剩余的那个轴。
        l = 3 - wi - wj
        c2 = float(np.clip(r[wi, wi], -1.0, 1.0))
        a2 = math.acos(c2)
        # 条件数说明：acos 在 +-1 处导数无穷大，因此 c2 中 1e-16 的舍入误差
        # 已经会产生约 1.4e-8 的角度。奇异判定因此必须比机器精度宽松得多，
        # 并且它作用于 sin(a2) —— 也就是闭式解所除的那个量。
        s2 = math.sin(a2)
        if abs(s2) < _GIMBAL_TOL:
            a1 = _solve_gimbal_lock(r, work_seq, a2, 0)
            a3 = 0.0
        else:
            a1 = math.atan2(r[wj, wi], -parity * r[l, wi])
            a3 = math.atan2(r[wi, wj], parity * r[wi, l])
    else:
        # Tait-Bryan（例如 ZYX）。
        s2 = float(np.clip(parity * r[wi, wk], -1.0, 1.0))
        a2 = math.asin(s2)
        # 与上面相同的条件数论证，现在针对 +-1 处的 asin：检验 cos(a2)，
        # 也就是闭式解所除的那个量。
        c2 = math.cos(a2)
        if abs(c2) < _GIMBAL_TOL:
            a1 = _solve_gimbal_lock(r, work_seq, a2, 0)
            a3 = 0.0
        else:
            a1 = math.atan2(-parity * r[wj, wk], r[wk, wk])
            a3 = math.atan2(-parity * r[wi, wj], r[wi, wi])

    out = np.array([a1, a2, a3], dtype=float)
    return out if intrinsic else out[::-1].copy()


def matrix_to_euler(matrix: ArrayLike, seq: str = "ZYX") -> np.ndarray:
    """:func:`euler_to_matrix` 的公开逆函数。

    从旋转矩阵提取欧拉角，同时处理两个欧拉族以及两种万向锁情形。完整的推导与
    条件数说明见 :func:`_matrix_to_euler`；这是调用方应当使用的名称。

    参数:
        matrix: 旋转矩阵，形状 ``(3, 3)``。
        seq: 取自 ``XYZxyz`` 且没有两个相邻轴相等的欧拉轴序列。默认为
            ``"ZYX"``（偏航/俯仰/滚转）。

    返回:
        形状为 ``(3,)`` 的 ``float64`` 弧度角数组，顺序与 ``seq`` 匹配。

    异常:
        TypeError: 如果 ``matrix`` 不是类数组对象或 ``seq`` 不是字符串。
        ValueError: 如果 ``matrix`` 无法重塑为 ``(3, 3)``，或 ``seq`` 不是
            合法的轴序列。

    示例:
        >>> import numpy as np
        >>> m = euler_to_matrix([0.2, 0.3, 0.4], "ZYX")
        >>> np.allclose(matrix_to_euler(m, "ZYX"), [0.2, 0.3, 0.4])
        True
    """
    return _matrix_to_euler(matrix, seq)


def rotation_between(v_from: ArrayLike, v_to: ArrayLike) -> np.ndarray:
    """将 ``v_from`` 旋转到 ``v_to`` 的最小旋转。

    旋转轴为 ``v_from x v_to``，角度为两向量之间的夹角，即单位球面上的最短弧。
    反平行情形（角度为 ``pi``）有无穷多解；此处通过 :func:`_perpendicular_to`
    选取一个确定性的垂直轴，使相同输入的重复调用始终给出相同的矩阵。

    参数:
        v_from: 源方向（无需单位长度）。
        v_to: 目标方向（无需单位长度）。

    返回:
        ``(3, 3)`` 旋转矩阵 ``R``，满足 ``R @ unit(v_from) == unit(v_to)``。

    异常:
        TypeError: 如果任一参数不是类数组对象。
        ValueError: 如果任一向量长度为零或包含非有限值。

    示例:
        >>> R = rotation_between([1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
        >>> np.allclose(R @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
        True
        >>> np.allclose(rotation_between([1.0, 0.0, 0.0], [1.0, 0.0, 0.0]), np.eye(3))
        True
    """
    a = _unit(v_from, "v_from")
    b = _unit(v_to, "v_to")
    cosine = float(np.clip(np.dot(a, b), -1.0, 1.0))

    if cosine > 1.0 - 1e-10:
        return np.eye(3)
    if cosine < -1.0 + 1e-10:
        axis = _perpendicular_to(a)
        return _rotvec_to_matrix(math.pi * axis)

    axis = _unit(np.cross(a, b), "rotation axis")
    return _rotvec_to_matrix(math.acos(cosine) * axis)


def project_onto_plane(
    v: ArrayLike,
    normal: ArrayLike,
    *,
    eps: float = 1e-9,
) -> np.ndarray:
    """去除 ``v`` 中沿 ``normal`` 方向的分量。

    参数:
        v: 待投影的向量。
        normal: 平面法线；无需单位长度。
        eps: 投影长度的退化阈值。

    返回:
        形状为 ``(3,)`` 的 ``float64`` 数组：``v - (v . n_hat) n_hat``。

    异常:
        TypeError: 如果任一参数不是类数组对象。
        ValueError: 如果 ``normal`` 是零向量、``eps`` 不为正，或投影退化
            （``norm < eps``），即 ``v`` 平行于平面法线、不携带平面内的方向。
            将此视为"无意见"的调用方应捕获 ``ValueError``。

    示例:
        >>> np.allclose(project_onto_plane([1.0, 2.0, 3.0], [0.0, 0.0, 1.0]), [1.0, 2.0, 0.0])
        True
    """
    if not eps > 0.0:
        raise ValueError(f"eps must be positive, got {eps!r}")
    vec = _as_vector3(v, "v")
    n_hat = _unit(normal, "normal")
    projected = vec - float(np.dot(vec, n_hat)) * n_hat
    norm = float(np.linalg.norm(projected))
    if norm < eps:
        raise ValueError(
            f"projection of v={np.array2string(vec, precision=6)} onto the plane with "
            f"normal={np.array2string(n_hat, precision=6)} is degenerate "
            f"(norm {norm:.3e} < eps {eps:.3e}); v is parallel to the normal"
        )
    return projected


def wrap_to_limits(
    angle: float,
    reference: float,
    lower: float | None = None,
    upper: float | None = None,
) -> float:
    """按 2*pi 的整数倍折叠 ``angle``，使其满足限位与连续性要求。

    关节角只在模 2*pi 的意义下有定义，然而控制器非常在意选择*哪一个*代表值：
    当 ``-3.1 rad`` 描述的是同一姿态时却命令 ``+3.2 rad``，会让机械臂毫无理由地
    多转半圈。此辅助函数以 ``reference`` 为中心生成 ``angle`` 的 2*pi 别名，
    并选取：

    1. 在所有位于 ``[lower, upper]`` 内的别名中，最接近 ``reference`` 的那个；或
    2. 如果没有别名落在行程范围内，则直接取最接近 ``reference`` 的别名
       （之后调用方可自行检测越界）。

    参数:
        angle: 待折叠的角度，单位为弧度。
        reference: 首选值，通常是当前已命令的角度。
        lower: 关节下限，单位为弧度；``None`` 表示无限制。
        upper: 关节上限，单位为弧度；``None`` 表示无限制。

    返回:
        选中的别名，单位为弧度。

    异常:
        TypeError: 如果某个数值参数不是实数。
        ValueError: 如果任一参数不是有限值，或 ``lower > upper``。

    示例:
        >>> import math
        >>> round(wrap_to_limits(math.pi + 0.1, 0.0), 6) == round(0.1 - math.pi, 6)
        True
        >>> wrap_to_limits(3.0, 3.0, lower=-1.0, upper=1.0)  # no alias fits
        3.0
        >>> round(wrap_to_limits(3.0, 3.0 - 2 * math.pi, lower=-1.0, upper=1.0), 6)
        -3.283185
    """
    for name, value in (("angle", angle), ("reference", reference)):
        if isinstance(value, bool) or not isinstance(value, (int, float, np.floating, np.integer)):
            raise TypeError(f"{name} must be a real number, got {type(value).__name__}: {value!r}")
        if not math.isfinite(float(value)):
            raise ValueError(f"{name} must be finite, got {value!r}")
    angle = float(angle)
    reference = float(reference)

    lo = None if lower is None else float(lower)
    hi = None if upper is None else float(upper)
    if lo is not None and not math.isfinite(lo):
        raise ValueError(f"lower limit must be finite or None, got {lower!r}")
    if hi is not None and not math.isfinite(hi):
        raise ValueError(f"upper limit must be finite or None, got {upper!r}")
    if lo is not None and hi is not None and lo > hi:
        raise ValueError(f"lower limit must be <= upper limit, got [{lo}, {hi}]")

    base = angle + _TWO_PI * round((reference - angle) / _TWO_PI)
    candidates = [base + _TWO_PI * k for k in _ALIASES]

    if lo is not None or hi is not None:
        floor = -math.inf if lo is None else lo - _LIMIT_TOL
        ceiling = math.inf if hi is None else hi + _LIMIT_TOL
        inside = [v for v in candidates if floor <= v <= ceiling]
        if inside:
            return min(inside, key=lambda v: abs(v - reference))
    return min(candidates, key=lambda v: abs(v - reference))


def _normalize_limits(
    limits: ArrayLike | None,
) -> list[tuple[float | None, float | None]]:
    """将 ``(3, 2)`` 的限位参数转换为逐关节的 ``(lo, hi)`` 对。

    ``None`` 项（行或标量）表示"该关节无限制"。

    参数:
        limits: ``None``、``(3, 2)`` 的类数组对象，或长度为 3 且各项为 ``None``
            或 ``(lower, upper)`` 对的序列。

    返回:
        由三个 ``(lower_or_None, upper_or_None)`` 元组构成的列表。

    异常:
        ValueError: 如果形状不正确，或某一对的 ``lower > upper``。
    """
    if limits is None:
        return [(None, None)] * 3

    arr = np.asarray(limits, dtype=object)
    if arr.shape not in ((3, 2), (3,)):
        raise ValueError(
            f"limits must have shape (3, 2) or be a length-3 sequence of "
            f"(lower, upper) pairs / None, got shape {arr.shape}: {limits!r}"
        )

    out: list[tuple[float | None, float | None]] = []
    for index, row in enumerate(arr.tolist()):
        if row is None:
            out.append((None, None))
            continue
        pair = np.asarray(row, dtype=float).reshape(-1)
        if pair.size != 2:
            raise ValueError(
                f"limits[{index}] must be a (lower, upper) pair or None, got {row!r}"
            )
        lo, hi = float(pair[0]), float(pair[1])
        if not (math.isfinite(lo) and math.isfinite(hi)):
            raise ValueError(f"limits[{index}] must be finite, got {row!r}")
        if lo > hi:
            raise ValueError(f"limits[{index}] lower must be <= upper, got [{lo}, {hi}]")
        out.append((lo, hi))
    return out


def select_euler_branches(
    matrix: ArrayLike,
    seq: str = "ZYX",
    limits: ArrayLike | None = None,
    reference: ArrayLike | None = None,
) -> np.ndarray:
    """为旋转选择对关节限位友好的欧拉分解。

    对于任意轴序列，3x3 旋转都有两个合法的欧拉分解。对于 Tait-Bryan 序列
    （三个互不相同的轴，例如 ``'ZYX'``），它们是 ``(a, b, c)`` 与
    ``(a + pi, pi - b, c + pi)``；对于真欧拉 / proper Euler 序列（存在重复轴，
    例如 ``'ZYZ'``），它们是 ``(a, b, c)`` 与 ``(a + pi, -b, c + pi)``。
    除此之外，每个单独的角度都可以平移任意 2*pi 的整数倍。

    本函数枚举两个分支，用 :func:`wrap_to_limits` 折叠三个角度中的每一个，然后
    按字典序键 ``(0 表示每个角度都在其限位内，否则为 1, sum |angle - reference|)``
    为分支打分并返回胜者。结果是既尊重硬件行程范围、又使关节运动尽可能小的解。

    参数:
        matrix: 待分解的 ``(3, 3)`` 旋转矩阵。
        seq: 取自 ``XYZxyz`` 的欧拉轴序列。大写表示内旋，小写表示外旋。
        limits: ``(3, 2)`` 的 ``[lower, upper]`` 弧度限位数组，或长度为 3 的
            序列，其中某项可为 ``None`` 表示对应关节无限制。``None`` 会完全
            禁用限位检查。
        reference: 长度为 3 的首选角度数组（通常是当前关节角）。``None`` 视为
            全零。

    返回:
        形状为 ``(3,)`` 的 ``float64`` 数组，按 ``seq`` 隐含的顺序保存选中的
        角度。

    异常:
        TypeError: 如果 ``matrix`` 不是类数组对象或 ``seq`` 不是字符串。
        ValueError: 如果 ``matrix`` 不是合法旋转、``seq`` 不是三字符轴序列，
            或 ``limits``/``reference`` 形状不正确。

    示例:
        用 numpy 构造内旋 ``ZYX`` 矩阵 ``Rz(0.2) @ Ry(0.3) @ Rx(0.4)``，
        然后恢复角度::

            >>> c1, s1 = np.cos(0.2), np.sin(0.2)
            >>> rz = np.array([[c1, -s1, 0.0], [s1, c1, 0.0], [0.0, 0.0, 1.0]])
            >>> c2, s2 = np.cos(0.3), np.sin(0.3)
            >>> ry = np.array([[c2, 0.0, s2], [0.0, 1.0, 0.0], [-s2, 0.0, c2]])
            >>> c3, s3 = np.cos(0.4), np.sin(0.4)
            >>> rx = np.array([[1.0, 0.0, 0.0], [0.0, c3, -s3], [0.0, s3, c3]])
            >>> r = rz @ ry @ rx
            >>> np.allclose(select_euler_branches(r, "ZYX"), [0.2, 0.3, 0.4])
            True

        在存在关节限位时，最接近参考位姿的分支胜出::

            >>> lim = np.array([[-3.0, 3.0], [-1.5, 1.5], [-3.0, 3.0]])
            >>> np.allclose(select_euler_branches(r, "ZYX", limits=lim,
            ...                                   reference=[0.0, 0.0, 0.0]),
            ...             [0.2, 0.3, 0.4])
            True
    """
    assert_rotation_matrix(matrix)
    if not isinstance(seq, str) or len(seq) != 3 or any(ch not in "XYZxyz" for ch in seq):
        raise ValueError(
            f"seq must be a 3 character Euler axis sequence over XYZxyz (e.g. 'ZYX'), "
            f"got {seq!r}"
        )

    pairs = _normalize_limits(limits)

    if reference is None:
        ref = np.zeros(3)
    else:
        ref = np.asarray(reference, dtype=float).reshape(-1)
        if ref.size != 3 or not np.all(np.isfinite(ref)):
            raise ValueError(
                f"reference must be a finite length-3 array of angles, got {reference!r}"
            )

    primary = _matrix_to_euler(np.asarray(matrix, dtype=float), seq)
    is_tait_bryan = len(set(seq.upper())) == 3
    if is_tait_bryan:
        secondary = np.array(
            [primary[0] + math.pi, math.pi - primary[1], primary[2] + math.pi]
        )
    else:
        secondary = np.array([primary[0] + math.pi, -primary[1], primary[2] + math.pi])

    best: np.ndarray | None = None
    best_key: tuple[int, float] | None = None
    for branch in (primary, secondary):
        folded = [
            wrap_to_limits(float(branch[i]), float(ref[i]), pairs[i][0], pairs[i][1])
            for i in range(3)
        ]
        within = all(
            lo is None or hi is None or (lo - _LIMIT_TOL <= v <= hi + _LIMIT_TOL)
            for v, (lo, hi) in zip(folded, pairs)
        )
        cost = float(sum(abs(v - r) for v, r in zip(folded, ref)))
        key = (0 if within else 1, cost)
        if best_key is None or key < best_key:
            best_key, best = key, np.array(folded, dtype=float)

    assert best is not None  # 保证：该循环总会执行两次
    logger.debug(
        "euler branch selection seq=%s tait_bryan=%s within_limits=%s cost=%.4f",
        seq,
        is_tait_bryan,
        best_key[0] == 0,
        best_key[1],
    )
    return best


def is_rotation_matrix(m: Any, tol: float = 1e-6) -> bool:
    """检查 ``m`` 是否为合法的 3D 旋转矩阵。

    判据是 ``R @ R.T == I`` 且 ``det(R) == +1``（在 ``tol`` 容差内）。
    行列式为 ``-1``（反射）会被拒绝。

    参数:
        m: 候选矩阵。
        tol: 两项检查所用的绝对容差。

    返回:
        如果 ``m`` 是有限值、``(3, 3)``、正交归一且行列式为 ``+1``，则返回
        ``True``。

    异常:
        ValueError: 如果 ``tol`` 为负。

    示例:
        >>> is_rotation_matrix(np.eye(3))
        True
        >>> is_rotation_matrix(np.diag([1.0, 1.0, -1.0]))
        False
    """
    if tol < 0.0:
        raise ValueError(f"tol must be non-negative, got {tol!r}")
    if not isinstance(m, (np.ndarray, Sequence)):
        return False
    try:
        arr = np.asarray(m, dtype=float)
    except (TypeError, ValueError):
        return False
    if arr.shape != (3, 3) or not np.all(np.isfinite(arr)):
        return False
    if float(np.abs(arr @ arr.T - np.eye(3)).max()) > tol:
        return False
    return abs(float(np.linalg.det(arr)) - 1.0) <= tol


def assert_rotation_matrix(m: Any, tol: float = 1e-6) -> np.ndarray:
    """校验 ``m`` 为旋转矩阵并以 ``float64`` 返回。

    参数:
        m: 候选 ``(3, 3)`` 矩阵。
        tol: 正交归一性与行列式检查的绝对容差。

    返回:
        与 ``m`` 相等的 ``(3, 3)`` ``float64`` 数组。

    异常:
        TypeError: 如果 ``m`` 不是类数组对象或无法转换为浮点数。
        ValueError: 如果 ``m`` 的形状不是 ``(3, 3)``、包含非有限值、
            不是正交归一，或行列式不为 ``+1``。

    示例:
        >>> assert_rotation_matrix(np.eye(3)).shape
        (3, 3)
    """
    if not isinstance(m, (np.ndarray, Sequence)):
        raise TypeError(
            f"rotation matrix must be array-like, got {type(m).__name__}: {m!r}"
        )
    try:
        arr = np.asarray(m, dtype=float)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"rotation matrix must hold real numbers, got {m!r}") from exc
    if arr.shape != (3, 3):
        raise ValueError(f"rotation matrix must have shape (3, 3), got {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"rotation matrix must be finite, got {arr!r}")
    ortho_error = float(np.abs(arr @ arr.T - np.eye(3)).max())
    if ortho_error > tol:
        raise ValueError(
            f"matrix is not orthonormal: max |R @ R.T - I| = {ortho_error:.3e} > tol {tol:.3e}"
        )
    determinant = float(np.linalg.det(arr))
    if abs(determinant - 1.0) > tol:
        raise ValueError(
            f"matrix is not a proper rotation: det = {determinant:.6f}, expected +1 "
            f"(a determinant of -1 indicates a reflection)"
        )
    return arr
