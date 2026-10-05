# geometry 子包说明

`reusable_model.geometry` 是从 Hulk/Zev 机器人栈中抽取的纯几何工具子包，提供两组彼此独立、
互不耦合的能力：

- **2D 轴对齐边界框**（`boxes.py`）：目标检测后处理、视觉跟踪与掩码工具所需的
  面积、交并比、裁剪、掩码与质心计算；
- **3D 旋转代数**（`rotations3d.py`）：由轴约束构造旋转矩阵、向量间最小旋转、
  欧拉角与旋转矩阵互转、以及感知关节限位的角度折叠。

整个子包只依赖标准库与 `numpy`，不依赖 scipy、OpenCV 等任何重型库。

## 1. 模块总览

| 文件 | 主要职责 | 对外公开接口（主要 API 名） |
| --- | --- | --- |
| `__init__.py` | 聚合子包内两个模块的公开 API，使 `reusable_model.geometry.<name>` 可直接访问 | `__all__`（`["boxes", "rotations3d"]`）、经由 `from .boxes import *` 与 `from .rotations3d import *` 导出的名称 |
| `boxes.py` | 轴对齐 2D 边界框几何：IoU/IoMin/重叠度、裁剪、掩码、质心 | `bbox_area`、`bbox_center`、`bbox_to_mask`、`clip_bbox`、`iou`、`iou_min`、`mask_center`、`overlap_score`；类型别名 `Box` |
| `rotations3d.py` | 通用 3D 旋转代数：轴约束构阵、最小旋转、欧拉角互转、限位角度折叠、分支选择 | `AXIS_KEYS`、`assert_rotation_matrix`、`euler_to_matrix`、`is_partial_rotation`、`is_rotation_matrix`、`matrix_to_euler`、`normalize_constraints`、`project_onto_plane`、`rotation_between`、`rotation_from_axis`、`rotation_from_two_axes`、`select_euler_branches`、`wrap_to_limits` |

## 2. 依赖关系与调用层级

### 第三方依赖

- `numpy`（唯一硬依赖）：`boxes.py` 使用 `numpy` 与 `numpy.typing.ArrayLike`
  返回掩码数组；`rotations3d.py` 使用 `numpy` 完成向量/矩阵运算、Rodrigues 公式
  与欧拉角提取。
- 其余均为标准库：`logging`、`math`、`typing`。

### 对其他子包的依赖

`geometry` **不依赖任何其他子包**（无跨子包 import），但它是被依赖方：

- `reusable_model.io.detection_npz`：`from ..geometry.boxes import bbox_to_mask`；
- `reusable_model.tracking.iou_tracker`：`from ..geometry.boxes import bbox_area, iou as _box_iou`；
- `reusable_model.tracking.stable_streak`：`from ..geometry.boxes import iou as _box_iou`。

因此修改 `boxes.py` 中这些函数的签名或行为会直接影响 `io`、`tracking` 子包。

### 子包内部调用层级

```
reusable_model/geometry/
├── __init__.py
│   ├── from .boxes import *          # 聚合 boxes 的公开 API
│   └── from .rotations3d import *    # 聚合 rotations3d 的公开 API
│   __all__ = ["boxes", "rotations3d"]  # 仅列举子模块名（影响 from reusable_model.geometry import *）
├── boxes.py            # 自包含，仅依赖 numpy / 标准库
└── rotations3d.py      # 自包含，仅依赖 numpy / 标准库
```

- `boxes.py` 与 `rotations3d.py` 之间**没有任何相互 import**，可独立使用。
- `__init__.py` 只做聚合：`from .boxes import *` / `from .rotations3d import *`
  会把两个模块的公开名称提升到子包命名空间，所以 `reusable_model.geometry.iou(...)` 与
  `reusable_model.geometry.rotations3d.matrix_to_euler(...)` 都可用。注意 `__init__.py` 的
  `__all__` 仅列出 `"boxes"`、`"rotations3d"`，它只约束
  `from reusable_model.geometry import *` 的星号导入行为，不影响上面被聚合进来的函数名。

## 3. 各模块职责与接口定义

### 3.1 boxes.py

**职责**：面向像素坐标矩形（边界框）的纯函数工具箱。约定边界框恒为
`(x1, y1, x2, y2)` 元组，左上角包含、右下角不包含（OpenCV / YOLO 约定）；
第四个元素之后的多余元素（如置信度）在所有函数中都会被忽略。

所有接收 `bbox` 的函数都通过内部 `_as_box` 做校验，因此共享同一组异常契约：

- `TypeError`：`bbox` 不是数值序列；
- `ValueError`：元素少于 4 个，或前四个元素中存在非有限值。

**公开 API**

| 签名 | 说明 | 返回 | 异常 |
| --- | --- | --- | --- |
| `bbox_area(bbox: Sequence[float]) -> float` | 边界框面积 | `(x2-x1)*(y2-y1)`，裁剪到零，退化框返回 `0.0` | `TypeError` / `ValueError`（同 `_as_box`） |
| `iou(a, b) -> float` | 交并比 | `[0.0, 1.0]` 的 `交集/并集`，不重叠为 `0.0` | `TypeError` / `ValueError` |
| `iou_min(a, b) -> float` | 交集除以较小框面积 | `[0.0, 1.0]`，一个框完全包含另一个时为 `1.0` | `TypeError` / `ValueError` |
| `overlap_score(a, b) -> float` | `max(iou, iou_min)`，跟踪用重叠度 | `[0.0, 1.0]` | `TypeError` / `ValueError` |
| `clip_bbox(bbox, width: int, height: int) -> tuple[int, int, int, int]` | 裁剪到图像并取整 | 位于 `[0,width]`/`[0,height]` 内的整数框；完全出界则塌缩为零面积框 | `TypeError`（`width`/`height` 非 `int`）；`ValueError`（`width`/`height` 为负；或 `bbox` 不合法） |
| `bbox_to_mask(bbox, width: int, height: int) -> np.ndarray` | 在框内填充布尔掩码 | 形状 `(height, width)` 的 `bool` 数组；退化/出界框返回全 `False` | 同 `clip_bbox` |
| `bbox_center(bbox) -> tuple[int, int]` | 边界框质心（四舍五入） | `(round((x1+x2)/2), round((y1+y2)/2))` | `TypeError` / `ValueError` |
| `mask_center(mask: ArrayLike) -> tuple[int, int] | None` | 掩码中 `True` 像素的质心 | 非零像素的 `(round(x_mean), round(y_mean))`；无 `True` 像素或无法解释为 2D 时返回 `None` | 不抛出（异常被捕获并降级为 `None`） |

模块级符号：`logger = logging.getLogger(__name__)`，以及类型别名
`Box = Tuple[float, float, float, float]`（`Box` 仅在模块内定义，不在 `__all__` 中）。

**使用示例**

```python
from reusable_model.geometry import boxes

# 1) 重叠度量
print(boxes.iou([0, 0, 10, 10], [5, 5, 15, 15]))        # 0.14285714285714285
print(boxes.iou_min([0, 0, 20, 20], [5, 5, 15, 15]))    # 1.0（小框完全嵌入）
print(boxes.overlap_score([0, 0, 10, 10], [0, 0, 100, 100]))  # 1.0

# 2) 裁剪与掩码（width=4, height=4）
print(boxes.clip_bbox([-5, 0, 30, 10], width=20, height=20))  # (0, 0, 20, 10)
mask = boxes.bbox_to_mask([1, 1, 3, 3], width=4, height=4)
print(mask.shape, mask.dtype, int(mask.sum()))               # (4, 4) bool 4

# 3) 质心
print(boxes.bbox_center([0, 0, 4, 2]))                  # (2, 1)
print(boxes.mask_center([[0, 1], [1, 0]]))              # (0, 0)
print(boxes.mask_center([[0, 0], [0, 0]]))              # None
```

### 3.2 rotations3d.py

**职责**：通用 3D 旋转代数。全部采用右手、列向量约定：对旋转矩阵 `R`，第 `i` 列
是第 `i` 个基向量的像（`R[:, 0]` 表示局部 `+x` 在父坐标系中的指向）。模块内自带
Rodrigues 公式与欧拉角提取，无需 scipy；但约定与 `scipy.spatial.transform` 一致。

模块级符号：

- `AXIS_KEYS: frozenset[str]`：单个轴约束键的合法拼写集合，
  即 `{"x","y","z","+x","+y","+z","-x","-y","-z"}`；前导 `+`/`-` 选择约束的机体轴
  方向，`{'z': [-1, 0, 0]}` 与 `{'-z': [1, 0, 0]}` 等价。
- `logger = logging.getLogger(__name__)`。

**公开 API**

| 签名 | 说明 | 返回 | 异常 |
| --- | --- | --- | --- |
| `is_partial_rotation(spec: Any) -> bool` | 判断 `spec` 是否为 1~2 个轴的旋转约束映射 | `bool`；非映射/空/超过 2 轴均返回 `False` | 不抛出 |
| `normalize_constraints(spec: Mapping[str, ArrayLike]) -> list[tuple[str, np.ndarray]]` | 校验并规范化约束，把符号前缀折叠进方向、归一化，轴名统一为 `'x'/'y'/'z'` | 1~2 个 `(axis, unit_direction)` 元组 | `TypeError`（非映射）；`ValueError`（空、超过 2 个、重复轴、未知键、零方向） |
| `rotation_from_two_axes(axis0: str, dir0: ArrayLike, axis1: str, dir1: ArrayLike) -> np.ndarray` | 由两个轴约束构造右手旋转矩阵（主列精确满足 `dir0`，次列在正交性允许范围内逼近 `dir1`） | `(3, 3)` 旋转矩阵 | `TypeError`（轴非字符串/方向非类数组）；`ValueError`（同轴、两者近乎平行 `|dot|>0.98`、或零方向） |
| `rotation_from_axis(axis: str, direction: ArrayLike) -> np.ndarray` | 锁定单个机体轴到某方向（绕该轴滚转自由，用确定性提示向量补全） | `(3, 3)` 旋转矩阵，`R[:, index(axis)] == unit(direction)` | `TypeError` / `ValueError`（未知轴、零方向） |
| `euler_to_matrix(angles: ArrayLike, seq: str = "ZYX") -> np.ndarray` | 由欧拉角合成旋转矩阵；大写序列=内旋，小写=外旋 | `(3, 3)` `float64` 数组 | `TypeError`（`angles` 非类数组或 `seq` 非字符串）；`ValueError`（角度非 3 个有限值、或序列非法） |
| `matrix_to_euler(matrix: ArrayLike, seq: str = "ZYX") -> np.ndarray` | 从旋转矩阵提取欧拉角（支持全部 24 种序列、万向锁安全） | `(3,)` 弧度角数组 | `TypeError` / `ValueError` |
| `select_euler_branches(matrix, seq: str = "ZYX", limits: ArrayLike | None = None, reference: ArrayLike | None = None) -> np.ndarray` | 在两个欧拉分支中选出既满足限位、又最接近 `reference` 的解 | `(3,)` `float64` 角数组 | `TypeError`；`ValueError`（非法旋转、非法序列、`limits`/`reference` 形状错误） |
| `rotation_between(v_from: ArrayLike, v_to: ArrayLike) -> np.ndarray` | 把 `v_from` 旋转到 `v_to` 的最小旋转（反平行时用确定性垂直轴） | `(3, 3)` 旋转矩阵 | `TypeError`；`ValueError`（零向量或非有限值） |
| `project_onto_plane(v, normal, *, eps: float = 1e-9) -> np.ndarray` | 去除 `v` 中沿 `normal` 的分量 | `(3,)` `float64` 数组：`v - (v·n̂)n̂` | `TypeError`；`ValueError`（`normal` 为零、`eps<=0`、或投影退化 `norm<eps`） |
| `wrap_to_limits(angle: float, reference: float, lower: float | None = None, upper: float | None = None) -> float` | 按 `2π` 的整数倍折叠角度：优先选在 `[lower, upper]` 内且最接近 `reference` 的别名 | 选中的别名（弧度） | `TypeError`（非实数）；`ValueError`（非有限、或 `lower > upper`） |
| `is_rotation_matrix(m: Any, tol: float = 1e-6) -> bool` | 检查 `R@R.T==I` 且 `det(R)==+1`（`-1` 的反射被拒） | `bool` | `ValueError`（`tol` 为负）；其余情况返回 `False` |
| `assert_rotation_matrix(m: Any, tol: float = 1e-6) -> np.ndarray` | 校验并返回 `float64` 旋转矩阵 | `(3, 3)` `float64` 数组 | `TypeError`（非类数组/无法转 float）；`ValueError`（形状非 `(3,3)`、非有限、非正交归一、行列式非 `+1`） |

内部辅助函数（不在 `__all__` 中，供模块内复用，也出现在 doctest 中）：
`_as_vector3`、`_unit`、`_axis_index`、`_parse_axis_direction`、`_complete_frame`、
`_perpendicular_to`、`_rotvec_to_matrix`、`_axis_matrix`、`_validate_seq`、
`_solve_gimbal_lock`、`_matrix_to_euler`、`_normalize_limits`。

**使用示例**

```python
import numpy as np
from reusable_model.geometry import rotations3d as r3

# 1) 欧拉角 <-> 旋转矩阵 往返（默认 ZYX = yaw/pitch/roll）
m = r3.euler_to_matrix([0.2, 0.3, 0.4], "ZYX")
print(np.allclose(r3.matrix_to_euler(m, "ZYX"), [0.2, 0.3, 0.4]))  # True

# 2) 由两个轴约束构造旋转矩阵：工具 +z 指向 -z，机体 +x 指向 +x
R = r3.rotation_from_two_axes("z", [0.0, 0.0, -1.0], "x", [1.0, 0.0, 0.0])
print(np.allclose(R[:, 2], [0.0, 0.0, -1.0]), r3.is_rotation_matrix(R))  # True True

# 3) 单轴锁定
Ra = r3.rotation_from_axis("z", [1.0, 0.0, 0.0])
print(np.allclose(Ra[:, 2], [1.0, 0.0, 0.0]))                        # True

# 4) 向量间最小旋转
Rr = r3.rotation_between([1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
print(np.allclose(Rr @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]))            # True

# 5) 投影到平面
print(r3.project_onto_plane([1.0, 2.0, 3.0], [0.0, 0.0, 1.0]))       # [1. 2. 0.]

# 6) 限位角度折叠：3.0 rad 在 [-1, 1] 内无别名可用时保持原值
print(r3.wrap_to_limits(3.0, 3.0, lower=-1.0, upper=1.0))            # 3.0
print(round(r3.wrap_to_limits(3.0, 3.0 - 2 * np.pi, lower=-1.0, upper=1.0), 6))  # -3.283185

# 7) 分支选择：给定限位与参考角，选出最合适的欧拉分解
lim = np.array([[-3.0, 3.0], [-1.5, 1.5], [-3.0, 3.0]])
angles = r3.select_euler_branches(m, "ZYX", limits=lim, reference=[0.0, 0.0, 0.0])
print(np.allclose(angles, [0.2, 0.3, 0.4]))                          # True
```

## 4. 模块间交互逻辑

### 4.1 子包内数据流

- `boxes.py` 与 `rotations3d.py` 完全独立，二者之间没有调用关系；`__init__.py`
  仅做命名空间聚合。
- `boxes.py` 内部：所有公开函数统一调用 `_as_box` 完成「取前四个元素 + 有限性校验」，
  `iou` / `iou_min` / `overlap_score` 共用 `_intersection` 计算交面积；
  `bbox_to_mask` 先调用 `clip_bbox` 再填充数组；`mask_center` 用
  `np.nonzero` 求均值。

```
bbox / a,b ──> _as_box ──> {bbox_area, bbox_center}
                      └──> _intersection ──> iou / iou_min ──> overlap_score
clip_bbox ──> bbox_to_mask
mask (ndarray) ──> np.nonzero ──> mask_center
```

### 4.2 `rotations3d.py` 内部调用链

典型调用链之一是 `select_euler_branches`（图中省略了间接层）：

```
select_euler_branches(matrix, seq, limits, reference)
  ├─ assert_rotation_matrix(matrix)        # 先校验输入是合法旋转
  ├─ _normalize_limits(limits)             # 把 (3,2)/长度3 的限位归一为逐关节 (lo, hi)
  ├─ _matrix_to_euler(matrix, seq)         # 取主分支角
  │    ├─ _validate_seq(seq)               # 校验序列、判定内旋/外旋
  │    ├─ 万向锁分支 ──> _solve_gimbal_lock(...)  ──> euler_to_matrix(...)
  │    └─ _axis_matrix(...)                # （由 euler_to_matrix 间接使用）
  ├─ 构造次分支 (a+π, π-b, c+π) 或 (a+π, -b, c+π)
  └─ 对每个分支逐角调用 wrap_to_limits(angle, reference, lo, hi)   # 折叠到限位内
     最终按 (是否全部在限位内, Σ|角-参考|) 字典序打分选优
```

其他关键链路：

- `rotation_from_two_axes` → `_parse_axis_direction`（内部 `_axis_index`、`_unit`）
  → `_complete_frame`（循环叉积补齐第三列），行列式为负时翻转次列后重建；
- `rotation_from_axis` → `_parse_axis_direction` → `_complete_frame`；
- `rotation_between` → `_unit` →（反平行时）`_perpendicular_to` → `_rotvec_to_matrix`；
- `euler_to_matrix` ↔ `matrix_to_euler` 互为逆运算，均以 `_validate_seq` 与
  `_axis_matrix` 为基础；
- `project_onto_plane` → `_as_vector3` / `_unit`。

### 4.3 与外部子包的交互

`io` 与 `tracking` 子包从 `reusable_model.geometry.boxes` 导入函数，形成数据流：

```
reusable_model.tracking.iou_tracker:   bbox_area, iou  ← reusable_model.geometry.boxes
reusable_model.tracking.stable_streak: iou             ← reusable_model.geometry.boxes
reusable_model.io.detection_npz:       bbox_to_mask    ← reusable_model.geometry.boxes
```

即：检测/跟踪结果中的 `(x1, y1, x2, y2)` 边界框先由 `geometry.boxes` 提供重叠度量与
掩码转换，再由 `tracking` / `io` 消费。`geometry` 自身不反向依赖这些子包。

## 5. 快速上手

最小可运行示例（无需安装，直接以 `reusable_model` 所在目录为工作目录或加入 `PYTHONPATH`）：

```python
import numpy as np

# 方式一：从子包命名空间直接访问被聚合的名称
from reusable_model.geometry import iou, overlap_score, matrix_to_euler, euler_to_matrix

# 方式二：显式导入具体模块
from reusable_model.geometry.boxes import bbox_area, bbox_to_mask, clip_bbox
from reusable_model.geometry import rotations3d as r3

# 边界框重叠
score = overlap_score([0, 0, 10, 10], [0, 0, 100, 100])
print(score)                                   # 1.0
print(iou([0, 0, 2, 2], [10, 10, 12, 12]))     # 0.0

# 裁剪与掩码
print(clip_bbox([-5, 0, 30, 10], width=20, height=20))   # (0, 0, 20, 10)
mask = bbox_to_mask([1, 1, 3, 3], width=4, height=4)
print(int(mask.sum()))                          # 4

# 旋转
R = euler_to_matrix([0.1, 0.2, 0.3], "ZYX")
print(np.allclose(matrix_to_euler(R, "ZYX"), [0.1, 0.2, 0.3]))   # True
print(np.allclose(r3.rotation_between([1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
                  @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]))          # True
```

## 6. 测试与验证

在仓库根目录 `reusable_model/` 下运行单元测试（`pyproject.toml` 已配置 `testpaths=["tests"]`
与 `pythonpath=[".."]`）：

```bash
cd /home/moke/Coding/Garden/reusable_model
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest tests/test_boxes.py tests/test_rotations3d.py -q -p no:cacheprovider
```

运行本子包内的 doctest（需把仓库根目录加入 `PYTHONPATH` 以便 `import reusable_model...` 解析）：

```bash
cd /home/moke/Coding/Garden
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/home/moke/Coding/Garden \
  python3 -m pytest --doctest-modules reusable_model/geometry -q -p no:cacheprovider
```

仅做语法检查：

```bash
cd /home/moke/Coding/Garden/reusable_model
python3 -m py_compile geometry/boxes.py geometry/rotations3d.py geometry/__init__.py
```
