# motion 子包说明

`reusable_model.motion` 是从 Hulk / Zev 机器人栈中抽取的运动学规划基础原语子包，与具体项目解耦：不含绝对路径、环境变量与项目专用配置，模块内统一通过 `logging.getLogger(__name__)` 记录日志。子包当前包含两个相互独立的模块：平面逆运动学（planar IK）与制动感知速度曲线（speed profile）。

## 1. 模块总览

| 文件 | 主要职责 | 对外公开接口（主要 API 名） |
| --- | --- | --- |
| `reusable_model/motion/__init__.py` | 子包入口，聚合导出两个模块的公开符号 | `__all__ = ["planar_ik", "speed_profile"]`（并星号导入两个模块的全部公开名） |
| `reusable_model/motion/planar_ik.py` | 平面双连杆逆运动学闭式解、7 自由度手臂的肘圆冗余参数化、通用非线性最小二乘（Levenberg-Marquardt）精修 | `PlanarTwoLink`、`PlanarIKResult`、`PlanarTwoLinkIK`、`elbow_circle`、`lowest_point_on_circle`、`circle_plane_intersections`、`select_elbow_on_circle`、`levenberg_marquardt`、`numeric_jacobian` |
| `reusable_model/motion/speed_profile.py` | 面向轨迹控制器的制动感知速度曲线：制动距离反解、加速度斜坡钳制、临近目标点的阻尼减速 | `BrakingProfile` |

## 2. 依赖关系与调用层级

### 2.1 第三方依赖

- `planar_ik.py`：导入时依赖 `numpy`（硬依赖）；`scipy` 仅由 `PlanarTwoLinkIK.solve_numeric` 在调用时惰性导入，属于可选依赖。因此实时控制器使用的解析求解路径只依赖 numpy。
- `speed_profile.py`：仅使用标准库（`logging`、`math`）。
- 未引入任何新的第三方依赖。

### 2.2 对其他子包的依赖

经检查，`motion` 子包**无任何跨子包依赖**：`planar_ik.py` 与 `speed_profile.py` 均只从标准库和 numpy 导入，不引用 `reusable_model` 下的 geometry、gridmap、vision、io、tracking、hardware、runtime 等任何兄弟子包，也未被这些子包反向依赖所要求。`motion` 可作为独立的运动学工具使用。

### 2.3 子包内部文件之间的调用层级

`planar_ik.py` 与 `speed_profile.py` **彼此完全独立**，不存在相互导入或调用；二者各自可单独使用。`motion/__init__.py` 仅通过 `from .planar_ik import *` 与 `from .speed_profile import *` 将两者的公开符号聚合到子包命名空间。

各文件内部的层次结构如下：

`planar_ik.py`：

1. 参数校验工具层（私有）：`_finite_float` → `_positive_float`、`_non_negative_float`；`_count_int` → `_axis_index`；`_as_vector`、`_as_vector_flat`、`_limits_pair`、`_inside_limits`、`_limit_seed`。
2. 数据模型层：`PlanarTwoLink`（冻结 dataclass，几何）、`PlanarIKResult`（dataclass，求解结果）。
3. 平面 IK 层：`PlanarTwoLinkIK`，内部使用上面的校验工具；`solve_numeric` 额外惰性使用 `scipy.optimize.least_squares`。
4. 肘圆冗余层：`elbow_circle`（基础几何）→ `_point_on_circle`、`lowest_point_on_circle`、`circle_plane_intersections` → `select_elbow_on_circle`（在上者之上做姿态选择）。
5. 通用非线性最小二乘层：`numeric_jacobian`（中心差分雅可比）与 `levenberg_marquardt`（带线性搜索的 LM 求解器，内部使用 Armijo 判据与自适应阻尼 `lambda`）。

`speed_profile.py`：仅 `BrakingProfile` 一个类；其私有辅助 `_damping_linear`、`_damping_angular`、`_damp`、`_validate_factor` 仅被本类的方法调用。

## 3. 各模块职责与接口定义

### 3.1 planar_ik.py

**职责**：提供竖直平面内双连杆链的闭式解逆运动学、7 自由度手臂肘部冗余自由度的圆参数化与姿态选择，以及一个依赖极少的非线性最小二乘求解器（用于在解析模型不适用时精修姿态）。

角度约定：角度单位为弧度，距离单位为米；平面链位于 `x` 向前 / `z` 向上的平面内，`q = 0` 表示沿 `+z` 竖直向上。返回的关节角到电机指令的映射由调用方完成。

**公开 API 清单**

#### `PlanarTwoLink`（冻结 dataclass）

二维平面内双连杆链的几何。

- 字段：`l1: float`（第一连杆长度，米，严格为正）、`l2: float`（第二连杆长度，米，严格为正）、`base_x: float = 0.0`（基座水平位置，米）、`base_z: float = 0.0`（基座高度，米）。
- 异常：字段非实数时 `TypeError`；`l1`/`l2` 不为严格正或字段非有限时 `ValueError`。

#### `PlanarIKResult`（dataclass）

一次逆运动学求解的结果。

- 字段：`success: bool`、`q1: float = 0.0`、`q2: float = 0.0`、`message: str = ""`、`reachable_distance: float = 0.0`。
- 方法：`as_dict() -> dict[str, Any]`，返回 `{'success', 'q1', 'q2', 'message', 'reachable_distance'}`。
- 异常：`success` 非 bool、`message` 非 str 或数值字段非实数时 `TypeError`；数值字段为 NaN/无穷时 `ValueError`。

#### `PlanarTwoLinkIK(geometry: PlanarTwoLink)`

闭式解逆运动学求解器。

- `__init__(geometry: PlanarTwoLink) -> None`：绑定几何。`geometry` 非 `PlanarTwoLink` 时抛 `TypeError`。
- 属性 `stand_height -> float`：链完全伸展（`q1 = q2 = 0`）时末端点高度，即 `base_z + l1 + l2`（米）。
- `forward(q1: float, q2: float) -> tuple[float, float]`：正运动学，返回 `(x, z)`（米）。角度非实数时 `TypeError`，NaN/无穷时 `ValueError`。
- `solve(x: float, z: float, *, q1_limits: Sequence[float] | None = None, q2_limits: Sequence[float] | None = None, tolerance: float = 1e-3) -> PlanarIKResult`：解析求解。`q2_limits` 决定肘分支；目标不可达或角度超限时返回 `success=False` 的结果（不抛异常）。类型错误时 `TypeError`，坐标/`tolerance` 非有限或限位对格式错误时 `ValueError`。
- `verify(result: PlanarIKResult, x: float, z: float) -> float`：把结果角度回代 `forward`，返回其与目标点的欧氏距离（米）。`result` 类型错误时 `TypeError`，坐标非有限时 `ValueError`。
- `solve_numeric(x: float, z: float, *, q1_limits=None, q2_limits=None, q0: ArrayLike | None = None, tol: float = 1e-3, forward_fn: Callable[[float, float], Any] | None = None) -> PlanarIKResult`：带边界的非线性最小二乘求解，可传入自定义正运动学 `forward_fn`。未安装 SciPy 时 `ImportError`；类型错误时 `TypeError`；坐标/`tol` 非有限、限位对格式错误、`q0` 元素数不为 2 或 `forward_fn` 返回值数不为 2 时 `ValueError`。

#### `elbow_circle(l1: float, l2: float, shoulder: ArrayLike, wrist: ArrayLike) -> tuple[np.ndarray, float, np.ndarray, np.ndarray] | None`

返回肘部可达的圆：`(centre, radius, u, v)`；腕不可达或处于圆坍缩的边界时返回 `None`。

#### `lowest_point_on_circle(centre: ArrayLike, radius: float, u: ArrayLike, v: ArrayLike) -> np.ndarray`

返回圆上世界坐标 `z` 最小的点（最低肘）。`radius` 为负或向量元素数不为 3 时 `ValueError`。

#### `circle_plane_intersections(centre: ArrayLike, radius: float, u: ArrayLike, v: ArrayLike, *, plane_axis: int, plane_value: float) -> list[float]`

返回圆与轴对齐平面 `p[plane_axis] == plane_value` 相交处的参数角列表（长度 0、1 或 2）。`plane_axis` 非 0/1/2、`radius` 为负或向量元素数不为 3 时 `ValueError`。

#### `select_elbow_on_circle(l1: float, l2: float, shoulder: ArrayLike, wrist: ArrayLike, *, plane_axis: int, plane_value: float, outside_test: Callable[[np.ndarray], Any], target_forward_axis: int = 0) -> np.ndarray | None`

为 7 自由度手臂选择冗余肘位：优先最低肘，已满足 `outside_test` 则保留，否则滑到约束平面上并在交点中按前伸/后伸选择较低或较高者。腕不可达时返回 `None`。`outside_test` 不可调用、长度非实数、位置非类数组时 `TypeError`；长度非严格正、位置元素数不为 3、轴索引非 0/1/2 时 `ValueError`。

#### `numeric_jacobian(fn: Callable[[np.ndarray], Any], q: ArrayLike, *, eps: float = 1e-6, output_dim: int | None = None) -> np.ndarray`

用中心差分近似 `fn` 在 `q` 处的 `(m, n)` 雅可比。`fn` 不可调用、`q` 非类数组或 `eps`/`output_dim` 类型错误时 `TypeError`；`q` 为空或非有限、`eps <= 0`、`fn` 返回空/尺寸不一致向量或 `output_dim` 不匹配时 `ValueError`。

#### `levenberg_marquardt(residual_fn, jacobian_fn, q0, *, max_iters: int = 20, tol: float = 1e-4, damping: float = 1e-3, damping_up: float = 4.0, damping_down: float = 0.3, damping_min: float = 1e-7, damping_max: float = 10.0, line_search_steps: int = 8, clip_fn: Callable[[np.ndarray], Any] | None = None) -> tuple[np.ndarray, float, bool]`

带线性搜索的自适应阻尼 LM 最小二乘，返回 `(best_q, best_err, converged)`；`best_q` 为迄今最优迭代点（非最后一点）。参数类型错误时 `TypeError`；取值非法（如 `damping_up <= 1`、`damping_down` 不在 `(0, 1)`、`damping_min > damping_max` 等）、`q0` 非有限、雅可比形状不匹配或残差为空时 `ValueError`。

**可运行示例**

```python
import numpy as np

from reusable_model.motion.planar_ik import (
    PlanarTwoLink,
    PlanarTwoLinkIK,
    elbow_circle,
    numeric_jacobian,
    levenberg_marquardt,
)

# 1) 解析逆运动学
geom = PlanarTwoLink(0.25, 0.25, base_z=0.5)
ik = PlanarTwoLinkIK(geom)
print("站立高度:", ik.stand_height)          # 1.0
result = ik.solve(0.0, 0.75, q2_limits=(-2.5, 0.0))
print("求解成功:", result.success)
print("关节角 q1/q2:", result.q1, result.q2)
print("往返残差:", ik.verify(result, 0.0, 0.75))

# 2) 肘圆与冗余选择（7 自由度手臂）
centre, radius, u, v = elbow_circle(0.3, 0.3, [0.0, 0.0, 1.0], [0.3, 0.0, 1.0])
print("圆心/半径:", centre, radius)

# 3) 通用 LM 求解（雅可比可由 numeric_jacobian 生成）
def residual(q):
    return np.array([q[0] ** 2 + q[1] ** 2 - 4.0, q[0] - q[1]])

q, err, ok = levenberg_marquardt(
    residual, lambda q: numeric_jacobian(residual, q), np.array([1.0, 1.0])
)
print("LM 收敛:", ok, "解:", q, "残差:", err)
```

### 3.2 speed_profile.py

**职责**：根据距目标的剩余距离与当前速度，计算每个控制周期应当下发的速度。以制动距离 `v = sqrt(2 * a * d)` 为基础，配合加速度斜坡钳制（防止单步速度突变）与临近目标点的指数阻尼减速（防止超调），对线性轴与角度轴均适用。模块为纯运动学，不涉及传感器、障碍物或硬件。

**公开 API 清单**

#### `BrakingProfile`

带制动距离控制的速度曲线，封装全部可调常数。构造参数均为关键字参数（`*` 之后）：

| 参数 | 默认值 | 含义 | 约束 |
| --- | --- | --- | --- |
| `forward_acceleration` | `0.8` | 加速时线性加速度（m/s²） | 必须为正 |
| `backward_acceleration` | `-0.8` | 减速时线性加速度（m/s²） | 必须为负 |
| `angular_acceleration` | `2.0` | 角加速度幅值（rad/s²） | 必须为正 |
| `max_forward_acceleration_step` | `0.1` | 线性速度单步最大增量（m/s²） | 必须为正 |
| `max_backward_acceleration_step` | `-0.2` | 线性速度单步最大减量（m/s²） | 必须为负 |
| `braking_linear_distance_factor` | `0.65` | 线性制动阻尼起始距离（m） | — |
| `braking_linear_exponential_factor` | `1.2` | 线性阻尼混合指数 | — |
| `braking_linear_velocity_factor` | `0.2` | 线性轴距离约为 0 时保留速度比例 | 须在 `[0, 1]` |
| `braking_angular_distance_factor` | `math.pi / 2` | 角度制动阻尼起始距离（rad） | — |
| `braking_angular_exponential_factor` | `1.5` | 角度阻尼混合指数 | — |
| `braking_angular_velocity_factor` | `0.1` | 角度轴距离约为 0 时保留速度比例 | 须在 `[0, 1]` |

- 异常：带符号参数符号错误或阻尼比例超出 `[0, 1]` 时 `ValueError`。

方法：

- `acceleration(current_velocity: float, target_velocity: float, time_interval: float, *, angular: bool = False) -> float`：返回本步可用的带符号加速度（m/s² 或 rad/s²），单步幅值按 `max_*_acceleration_step` 钳制。`time_interval` 不为正时 `ValueError`。
- `target_velocity(current_velocity: float, target_velocity: float, time_interval: float, *, angular: bool = False) -> float`：在单步内把速度变化钳制到可实现斜坡，不越过 `target_velocity`。
- `braking_velocity(distance: float, acceleration: float) -> float`（静态方法）：返回恰好停在目标点所需速度 `sign(distance) * sqrt(2 * |acceleration| * |distance|)`。`distance` 为零时 `ValueError`。
- `motion_strategy(current_velocity: float, max_velocity: float, time_interval: float, *, distance: float | None = None, target_linear_velocity: float | None = None, angular: bool = False, navigating: bool = False) -> float`：综合策略，计算本步应下发的速度。`distance`（制动到点）与 `target_linear_velocity`（巡航目标）至少提供一个，同时给出时 `distance` 优先。`max_velocity` 不为正、或两者均为 `None` 时 `ValueError`。

**可运行示例**

```python
from reusable_model.motion.speed_profile import BrakingProfile

p = BrakingProfile()

# 距停止点 0.3 m、当前 0.5 m/s，上限 0.6 m/s：返回本步应下发速度
v = p.motion_strategy(0.5, 0.6, 0.02, distance=0.3)
print("下发速度:", v)

# 已知目标速度时的单步斜坡
v_next = p.target_velocity(0.0, 0.8, 0.1)
print("斜坡后速度:", round(v_next, 2))        # 0.08

# 制动距离反解
print("制动速度:", BrakingProfile.braking_velocity(0.3, 0.8))

# 角度轴（例如转向）
w = p.motion_strategy(0.0, 2.0, 0.02, distance=0.5, angular=True)
print("角速度:", w)
```

## 4. 模块间交互逻辑

`motion` 子包内部两个模块相互独立，典型调用链各自闭合，由上层（路径规划 / 步态控制 / 底盘控制）组合使用：

- **逆运动学链路（planar_ik）**：上层路径规划或步态规划给出足端/手端在平面内的目标点 `(x, z)`，调用 `PlanarTwoLinkIK.solve`（或需要自定义正运动学模型时用 `solve_numeric`）得到关节角 `q1`、`q2`；随后可用 `verify` 把结果回代 `forward` 做往返校验。若模型为带肘部冗余的手臂，则先用 `elbow_circle` 求出肘部所在圆，用 `select_elbow_on_circle` 依据 `outside_test` 约束选定冗余肘位，再对选定肘点求解其余关节。调试或对精度要求更高的场合，可用 `numeric_jacobian` 配合 `levenberg_marquardt` 对已播种的姿态进行精修，直至末端落到目标。
- **速度规划链路（speed_profile）**：上层底盘/转向控制每个控制周期把剩余距离（或目标速度）、当前速度、时间步长传入 `BrakingProfile.motion_strategy`，得到本步应下发的线速度或角速度；再由控制器转换为电机指令。该过程不依赖 IK，也不产生关节角。
- **数据流**：两条链路的数据不共享——`planar_ik` 处理“位置 ↔ 关节角”的空间映射，`speed_profile` 处理“距离/速度 ↔ 时间步速度指令”的时序映射。上层若需要同时控制位置与速度，需自行把两者的输出组合（例如以 IK 得到关节目标，以速度曲线得到时间标度）。

## 5. 快速上手

`reusable_model` 包位于仓库 `reusable_model/` 目录下，直接在该目录或其父目录将 `reusable_model` 纳入 `PYTHONPATH` 即可导入（也可 `pip install -e .`）。

```python
from reusable_model.motion.planar_ik import PlanarTwoLink, PlanarTwoLinkIK
from reusable_model.motion.speed_profile import BrakingProfile

# 平面双连杆逆运动学（例如一条腿的髋-膝链）
ik = PlanarTwoLinkIK(PlanarTwoLink(0.25, 0.25, base_z=0.5))
res = ik.solve(0.10, 0.80, q2_limits=(-2.5, 0.0))
print(res.success, res.q1, res.q2)

# 制动感知速度曲线（例如底盘驶向目标点）
profile = BrakingProfile()
speed = profile.motion_strategy(
    current_velocity=0.4, max_velocity=0.6, time_interval=0.02, distance=0.5
)
print(speed)
```

也可直接从子包命名空间导入（`reusable_model.motion` 已聚合导出两个模块的公开符号）：

```python
from reusable_model.motion import PlanarTwoLink, PlanarTwoLinkIK, BrakingProfile
```

## 6. 测试与验证

在 `reusable_model` 目录下运行单元测试：

```bash
cd /home/moke/Coding/Garden/reusable_model
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest tests/test_motion.py tests/test_speed_profile.py -q -p no:cacheprovider
```

在仓库根目录运行模块内 doctest：

```bash
cd /home/moke/Coding/Garden
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/home/moke/Coding/Garden python3 -m pytest --doctest-modules reusable_model/motion -q -p no:cacheprovider
```

说明：`tests/test_motion.py` 覆盖 `PlanarTwoLinkIK` 的可达/不可达、限位与结果字段，以及 `elbow_circle` 的几何与不可达分支；`tests/test_speed_profile.py` 覆盖 `BrakingProfile` 的加速度钳制、斜坡不超调、制动速度符号与 `motion_strategy` 的各类分支及参数校验。两个模块的 docstring 均含可执行 doctest，由 `--doctest-modules` 一并验证。
