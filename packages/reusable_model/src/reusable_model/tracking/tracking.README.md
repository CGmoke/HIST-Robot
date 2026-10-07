# tracking 子包说明

`reusable_model.tracking` 是轻量级目标跟踪子包，只用标准库加
`numpy`（经由 `reusable_model.geometry.boxes`）实现两件彼此独立的事：

- **IoU 多目标跟踪**（`iou_tracker.py`）：贪心逐帧数据关联，为检测框跨帧分配
  稳定的 `track_id`，并在多相机 / 多会话场景下按 key 隔离关联状态；
- **稳定连续帧判定**（`stable_streak.py`）：判断同一个 `track_id` 是否在连续
  多帧里持续保持原地，即"稳定"确认。

整个子包没有运动模型、外观描述子或线性分配，属于"够用即可"的跟踪方案。

## 1. 模块总览

| 文件 | 主要职责 | 对外公开接口（主要 API 名） |
| --- | --- | --- |
| `__init__.py` | 聚合两个模块的公开名称到子包命名空间 | `__all__`（`["iou_tracker", "stable_streak"]`），以及经由 `from .iou_tracker import *` 与 `from .stable_streak import *` 提升的名称 |
| `iou_tracker.py` | 面向检测框的贪心 IoU 多目标跟踪：跨帧分配 `track_id`、按 key 组织线程安全跟踪器、挑选主目标 | `IoUTracker`、`KeyedTrackerStore`、`pick_primary_track_id` |
| `stable_streak.py` | 统计同一目标保持原地的连续帧数，达到阈值即判定为稳定 | `StableStreak` |

## 2. 依赖关系与调用层级

### 第三方依赖

- **标准库**：`__init__.py` 无额外导入；`iou_tracker.py` 使用 `logging`、
  `dataclasses`、`threading`、`typing`；`stable_streak.py` 使用 `logging`、
  `typing`。两个模块都带有 `from __future__ import annotations`。
- 子包自身**不直接导入 numpy**；`stable_streak.py` 模块 docstring 中提到的
  "仅 numpy"指的是其依赖的 `reusable_model.geometry.boxes` 使用 numpy。

### 对其他子包的依赖

两个模块都依赖 `reusable_model.geometry.boxes`：

```python
# iou_tracker.py
from ..geometry.boxes import bbox_area, iou as _box_iou
# stable_streak.py
from ..geometry.boxes import iou as _box_iou
```

即 `iou_tracker.py` 使用 `iou`（别名 `_box_iou`）做匹配打分，使用 `bbox_area`
挑选主目标；`stable_streak.py` 只使用 `iou`（别名 `_box_iou`）判断边界框是否
仍在原位。`tracking` 不反向被 `geometry` 依赖。

### 子包内部调用层级

`iou_tracker.py` 与 `stable_streak.py` **彼此没有任何相互 import**，可独立使用；
`__init__.py` 仅做命名空间聚合。

```
reusable_model/tracking/
├── __init__.py
│   ├── from .iou_tracker import *     # 聚合 iou_tracker 的公开 API
│   ├── from .stable_streak import *   # 聚合 stable_streak 的公开 API
│   └── __all__ = ["iou_tracker", "stable_streak"]   # 仅列举子模块名
├── iou_tracker.py      # 自包含；依赖 reusable_model.geometry.boxes 与标准库
│   ├── _Track（内部 dataclass）
│   ├── _box_4（内部辅助函数）
│   └── IoUTracker / pick_primary_track_id / KeyedTrackerStore
└── stable_streak.py    # 自包含；依赖 reusable_model.geometry.boxes 与标准库
    ├── _box_4（内部辅助函数，与 iou_tracker 中各有一份独立实现）
    └── StableStreak
```

- 两个文件**共享同一组概念**：边界框统一为 `(x1, y1, x2, y2)`，均以 IoU 作为
  匹配 / 稳定度量，且都含一个仅在本模块内使用的 `_box_4`（互不调用，是两份
  各自独立的实现）。
- `__init__.py` 的 `__all__` 只约束 `from reusable_model.tracking import *` 的行为，不
  影响被 `import *` 提升进来的类名：因此 `from reusable_model.tracking import IoUTracker`
  可直接使用。

## 3. 各模块职责与接口定义

### 3.1 iou_tracker.py

**职责**：一个贪心（greedy）的逐帧 IoU 数据关联跟踪器。每一帧把当前轨迹与
检测框两两比较 IoU，按分数降序排序后贪心配对；未匹配的检测框新建轨迹；连续
`max_age` 帧未匹配的轨迹被删除。遵循 SORT 的**确认（confirmation）**约定：
只有命中次数达到 `min_hits` 的轨迹才会对外发布 `track_id`。对象除内部轨迹
列表外无状态。

模块级符号：`logger = logging.getLogger(__name__)`；`__all__ = ["IoUTracker",
"KeyedTrackerStore", "pick_primary_track_id"]`。内部 `_Track`（dataclass，
字段 `track_id`、`bbox`、`label`、`confidence`、`hits=1`、`time_since_update=0`）
与 `_box_4` 不在 `__all__` 中。

**公开 API**

| 签名 | 说明 | 返回 | 异常 |
| --- | --- | --- | --- |
| `IoUTracker(iou_threshold: float = 0.3, max_age: int = 30, min_hits: int = 1)` | dataclass；`iou_threshold` 为轨迹与检测框被视为匹配的最小 IoU（`[0.0, 1.0]`）；`max_age` 为轨迹被删除前允许连续未匹配的帧数（`>= 0`）；`min_hits` 为上报 `track_id` 前所需的连续命中次数（`>= 1`） | `IoUTracker` 实例 | `ValueError`（任一阈值越界，在 `__post_init__` 中抛出） |
| `IoUTracker.reset() -> None` | 丢弃所有轨迹并把 id 计数器重置为 1 | `None` | 无 |
| `IoUTracker.active_track_ids -> list[int]`（property） | 当前存活的 `track_id`，按创建顺序 | `list[int]` | 无 |
| `IoUTracker.update(detections: Sequence[dict[str, Any]]) -> list[dict[str, Any]]` | 将一帧检测框与轨迹关联；检测框为字典，至少含 `bbox`（`(x1, y1, x2, y2)` 或更长序列），可选含 `label`、`confidence` | 与输入等长的新字典列表，保持输入顺序与内容；已确认项新增整型键 `track_id`，未确认项不含该键 | `KeyError`（某个检测框缺少 `"bbox"` 键）；`TypeError` / `ValueError`（`bbox` 非数值或元素不足 4 个，来自 `reusable_model.geometry.boxes` / `_box_4`） |
| `pick_primary_track_id(detections: Iterable[dict[str, Any]]) -> int \| None` | 在已跟踪的检测框中选出主目标：取边界框面积最大者，面积相同时置信度更高者优先 | 选中的 `track_id`；若没有任何检测框携带 `track_id` 则返回 `None` | `TypeError` / `ValueError`（当某检测框带有非法 `bbox` 时，由 `reusable_model.geometry.boxes.bbox_area` 抛出；`bbox` 缺失或为 `None` 时按 `(0, 0, 0, 0)` 处理，不抛异常） |

`KeyedTrackerStore`（线程安全容器）：

| 签名 | 说明 | 返回 | 异常 |
| --- | --- | --- | --- |
| `KeyedTrackerStore(*, iou_threshold: float = 0.3, max_age: int = 30, min_hits: int = 1)` | 按 key 管理多个 `IoUTracker` 实例，参数会转发给每个新建的跟踪器；key 可为任意可哈希值，`None` 映射到 `"default"` | `KeyedTrackerStore` 实例 | 构造时不校验；参数非法会在首次创建 `IoUTracker` 时抛出 `ValueError` |
| `KeyedTrackerStore.reset(key: Hashable \| None = None) -> None` | 丢弃 `key` 的所有轨迹；该 key 不存在时创建一个全新的跟踪器 | `None` | `ValueError`（参数非法且需新建跟踪器时，来自 `IoUTracker.__post_init__`） |
| `KeyedTrackerStore.update(key: Hashable \| None, detections: Sequence[dict[str, Any]]) -> list[dict[str, Any]]` | 更新 `key` 对应的跟踪器，首次使用时创建 | 同 `IoUTracker.update` | `ValueError`（首次创建时参数非法）；其余同 `IoUTracker.update` |

> 说明：`KeyedTrackerStore` 内部用 `threading.Lock` 串行化 `reset` / `update`，
> 因此对多个 key 的访问是线程安全的；但**同一个** `IoUTracker` 实例本身不
> 带锁，若绕过 `KeyedTrackerStore` 直接共享单个 `IoUTracker`，需要调用方自行
> 加锁。

**使用示例**

```python
from reusable_model.tracking import IoUTracker, KeyedTrackerStore, pick_primary_track_id

# 单相机跟踪
tracker = IoUTracker(iou_threshold=0.3, max_age=30, min_hits=1)
first = tracker.update([
    {"bbox": [0, 0, 10, 10], "label": "person", "confidence": 0.9},
    {"bbox": [50, 50, 60, 60], "label": "person", "confidence": 0.8},
])
print([d["track_id"] for d in first])      # [1, 2]

second = tracker.update([{"bbox": [2, 2, 12, 12], "label": "person", "confidence": 0.95}])
print(second[0]["track_id"])               # 1（与上一帧的大框匹配）
print(tracker.active_track_ids)            # [1, 2]

# 挑选主目标：面积最大者胜出
print(pick_primary_track_id(first))        # 1

# 多相机 / 多会话：各 key 状态隔离，id 各自从 1 开始
store = KeyedTrackerStore(min_hits=1)
a = store.update("cam-a", [{"bbox": [0, 0, 10, 10]}])
b = store.update("cam-b", [{"bbox": [0, 0, 10, 10]}])
print(a[0]["track_id"], b[0]["track_id"])  # 1 1

# 确认门限：min_hits=3 时前两帧不发布 track_id
slow = IoUTracker(min_hits=3)
print("track_id" in slow.update([{"bbox": [0, 0, 10, 10]}])[0])  # False
print("track_id" in slow.update([{"bbox": [1, 1, 11, 11]}])[0])  # False
print(slow.update([{"bbox": [2, 2, 12, 12]}])[0]["track_id"])    # 1
```

### 3.2 stable_streak.py

**职责**：判断同一目标是否在连续多帧中保持原地。内部维护 `streak` 计数：若本帧
的 `track_id` 与上一帧相同，且当前边界框与上一帧边界框的 IoU 不低于
`stable_iou`，则计数加一；否则重置（有目标帧从 1 重新开始，无目标帧归零）。
当计数达到 `min_stable_frames` 时判定为稳定。严格的重置规则：`track_id` 改变
会重置，位移过大（IoU 过低）会重置，没有检测框会归零。

模块级符号：`logger = logging.getLogger(__name__)`；`__all__ = ["StableStreak"]`；
内部辅助函数 `_box_4` 不在 `__all__` 中。

**公开 API**

| 签名 | 说明 | 返回 | 异常 |
| --- | --- | --- | --- |
| `StableStreak(min_stable_frames: int = 3, stable_iou: float = 0.5)` | `min_stable_frames` 为判定稳定所需的连续帧数（`>= 1`）；`stable_iou` 为当前框与上一帧框被视为同一位置的最小 IoU（`[0.0, 1.0]`） | `StableStreak` 实例 | `ValueError`（任一参数越界） |
| `StableStreak.reset() -> None` | 清空 `streak` 并遗忘上一帧（`_prev_track_id`、`_prev_bbox` 置 `None`） | `None` | 无 |
| `StableStreak.update(track_id: int \| None, bbox: Sequence[float] \| None) -> bool` | 用新的一帧推进计数并报告稳定性；两个参数须同时给出；任一为 `None` 视为该帧无目标并重置计数 | `True` 当且仅当本帧有目标且 `streak >= min_stable_frames` | `ValueError`（给了 `track_id` 但 `bbox` 为 `None`，或 `bbox` 元素少于 4 个）；`TypeError`（`bbox` 非数值） |

实例属性（可读）：`min_stable_frames: int`、`stable_iou: float`、
`streak: int`（当前连续帧数，初值 0）；以及私有的 `_prev_track_id`、
`_prev_bbox`。

**使用示例**

```python
from reusable_model.tracking import StableStreak

s = StableStreak(min_stable_frames=3, stable_iou=0.5)
print(s.update(1, [0, 0, 10, 10]))   # False（第 1 帧，streak=1）
print(s.update(1, [1, 1, 11, 11]))   # False（第 2 帧，streak=2）
print(s.update(1, [2, 2, 12, 12]))   # True （第 3 帧，streak=3，达到阈值）
print(s.streak)                      # 3

# 位移过大 -> 重置为 1
print(s.update(1, [50, 50, 60, 60])) # False
print(s.streak)                      # 1

# 无检测框 -> 归零
print(s.update(None, None))          # False
print(s.streak)                      # 0
```

## 4. 模块间交互逻辑

两个模块在**数据层面**串联：`iou_tracker` 为每一帧的检测框赋予稳定
`track_id`，`stable_streak` 再据此判断目标是否稳定。典型调用链：

```
检测框列表 [{"bbox": (x1,y1,x2,y2), "label": ..., "confidence": ...}, ...]
      │
      ▼  IoUTracker.update(detections)             # 贪心 IoU 关联
带 track_id 的结果 [{"bbox": ..., "track_id": 1}, ...]
      │
      ├─ pick_primary_track_id(result)  ──> 主目标 track_id（面积最大、置信度次之）
      │
      ▼  StableStreak.update(track_id, bbox)       # 逐帧喂入主目标
   True / False（本帧是否已稳定）
```

- `IoUTracker` 与 `StableStreak` 之间**没有直接依赖**，由调用方在二者之间传递
  `(track_id, bbox)`；因此 `StableStreak` 也可以接收来自其他跟踪器的 `track_id`。
- 二者都以 IoU 为度量、都接受 `(x1, y1, x2, y2)` 边界框，因而"先跟踪、后判稳"
  的组合最自然：先用 `iou_tracker` 得到跨帧一致的 `track_id`，再用
  `stable_streak` 过滤忽隐忽现或来回抖动的目标，仅在明确稳定后才触发下游动作
  （如抓取、凝视锁定）。
- `KeyedTrackerStore` 位于 `IoUTracker` 之上，为多路流分别持有一个跟踪器；
  每路的输出可各自接入独立的 `StableStreak`。

## 5. 快速上手

无需安装，以 `reusable_model` 所在目录为工作目录或把仓库根目录加入 `PYTHONPATH` 即可：

```python
# 方式一：从子包命名空间直接访问被聚合的类
from reusable_model.tracking import IoUTracker, StableStreak, pick_primary_track_id

# 方式二：显式导入具体模块
from reusable_model.tracking.iou_tracker import IoUTracker, KeyedTrackerStore, pick_primary_track_id
from reusable_model.tracking.stable_streak import StableStreak

tracker = IoUTracker(min_hits=1, iou_threshold=0.3)
streak = StableStreak(min_stable_frames=2, stable_iou=0.5)

for bbox in ([0, 0, 10, 10], [1, 1, 11, 11], [2, 2, 12, 12]):
    det = tracker.update([{"bbox": bbox, "label": "target", "confidence": 0.9}])[0]
    tid = det.get("track_id")
    stable = streak.update(tid, det["bbox"]) if tid is not None else streak.update(None, None)
    print(tid, stable)
# 1 False
# 1 True
# 1 True
```

## 6. 测试与验证

单元测试（`reusable_model/pyproject.toml` 已配置 `testpaths` 与 `pythonpath`）：

```bash
cd /home/moke/Coding/Garden/reusable_model
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest tests/test_tracking.py -q -p no:cacheprovider
```

运行本子包的 doctest（需把仓库根目录加入 `PYTHONPATH` 以便 `import reusable_model...` 解析）：

```bash
cd /home/moke/Coding/Garden
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/home/moke/Coding/Garden \
  python3 -m pytest --doctest-modules reusable_model/tracking -q -p no:cacheprovider
```

仅做语法检查：

```bash
cd /home/moke/Coding/Garden/reusable_model
python3 -m py_compile tracking/iou_tracker.py tracking/stable_streak.py tracking/__init__.py
```
