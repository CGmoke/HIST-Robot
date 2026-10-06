"""面向检测框的轻量级 IoU 多目标跟踪。

这是一个贪心（greedy）的逐帧数据关联跟踪器：当检测框需要在多帧之间获得稳定的
``track_id``，而完整的 Kalman / SORT 流水线又显得过重时，它非常适用。它以牺牲
精度换取零外部依赖：没有运动模型、没有外观描述子，也没有线性分配——而是按 IoU
分数降序排序并贪心分配配对。

跟踪器遵循 SORT 的*确认（confirmation）*约定：只有当某条轨迹被匹配 ``min_hits``
次之后，检测框才会发布其 ``track_id``；而一条轨迹在 ``max_age`` 帧未被匹配后
会被删除。除内部轨迹列表外，该对象在多次调用之间是无状态的，因此可由任意检测源
驱动。

对于每路会话都需要独立关联状态的多相机 / 多会话场景，参见
:class:`KeyedTrackerStore`。

依赖：仅标准库。
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from threading import Lock
from typing import Any, Hashable, Iterable, Sequence

from ..geometry.boxes import bbox_area, iou as _box_iou

logger = logging.getLogger(__name__)

__all__ = [
    "IoUTracker",
    "KeyedTrackerStore",
    "pick_primary_track_id",
]


@dataclass
class _Track:
    track_id: int
    bbox: tuple[float, float, float, float]
    label: str
    confidence: float
    hits: int = 1
    time_since_update: int = 0


@dataclass
class IoUTracker:
    """贪心 IoU 跟踪器，为检测框分配持久的 ``track_id``。

    参数：
        iou_threshold: 轨迹与检测框被视为匹配所需的最小 IoU。低于该阈值的
            检测框会新建一条轨迹。取值必须位于 ``[0.0, 1.0]``。
        max_age: 轨迹在被删除前允许连续未被匹配的帧数。必须为非负整数。
        min_hits: 轨迹上报其 ``track_id`` 前所需的连续匹配次数。``1`` 表示
            立即上报；更大的值可过滤单帧闪烁。必须为正整数。

    异常：
        ValueError: 任一阈值参数超出其取值范围时抛出。
    """

    iou_threshold: float = 0.3
    max_age: int = 30
    min_hits: int = 1
    _next_id: int = 1
    _tracks: list[_Track] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not 0.0 <= self.iou_threshold <= 1.0:
            raise ValueError(
                f"iou_threshold must be in [0, 1], got {self.iou_threshold!r}"
            )
        if self.max_age < 0:
            raise ValueError(f"max_age must be non-negative, got {self.max_age!r}")
        if self.min_hits < 1:
            raise ValueError(f"min_hits must be positive, got {self.min_hits!r}")

    def reset(self) -> None:
        """丢弃所有轨迹，并将 id 计数器重置为 1。"""
        self._next_id = 1
        self._tracks.clear()

    @property
    def active_track_ids(self) -> list[int]:
        """当前存活的 ``track_id`` 列表，按创建顺序排列。"""
        return [t.track_id for t in self._tracks]

    def update(self, detections: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
        """将一帧检测框与轨迹进行关联。

        检测框为字典，至少携带 ``bbox``（``(x1, y1, x2, y2)`` 或更长的序列），
        并可选携带 ``label`` 与 ``confidence``。

        参数：
            detections: 该帧的检测框列表，顺序不限。

        返回：
            一个新的字典列表，每个输入检测框对应一项，保持输入顺序与内容不变。
            已确认的检测框会新增一个 ``track_id`` 整数键；未确认的检测框
            （命中次数低于 ``min_hits``）其余键保持不变。

        示例：
            >>> tr = IoUTracker(min_hits=1)
            >>> a = tr.update([{"bbox": [0, 0, 10, 10], "label": "x"}])
            >>> a[0]["track_id"]
            1
            >>> b = tr.update([{"bbox": [2, 2, 12, 12], "label": "x"}])
            >>> b[0]["track_id"]
            1
        """
        for tr in self._tracks:
            tr.time_since_update += 1

        # 为每个（轨迹, 检测框）配对打分，并按分数降序排序，
        # 使贪心分配优先选择最强匹配。
        pairs: list[tuple[float, int, int]] = []
        for ti, tr in enumerate(self._tracks):
            for di, det in enumerate(detections):
                score = _box_iou(tr.bbox, det["bbox"])
                if score >= self.iou_threshold:
                    pairs.append((score, ti, di))
        pairs.sort(reverse=True)

        assigned_tid: dict[int, int] = {}
        used_t: set[int] = set()
        used_d: set[int] = set()
        for _, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            tr = self._tracks[ti]
            det = detections[di]
            tr.bbox = _box_4(det["bbox"])
            tr.label = str(det.get("label", ""))
            tr.confidence = float(det.get("confidence", 0.0))
            tr.hits += 1
            tr.time_since_update = 0
            if tr.hits >= self.min_hits:
                assigned_tid[di] = tr.track_id

        # 未被任何轨迹匹配的检测框会新建轨迹。
        for di, det in enumerate(detections):
            if di in used_d:
                continue
            tid = self._next_id
            self._next_id += 1
            self._tracks.append(
                _Track(
                    track_id=tid,
                    bbox=_box_4(det["bbox"]),
                    label=str(det.get("label", "")),
                    confidence=float(det.get("confidence", 0.0)),
                )
            )
            if self.min_hits <= 1:
                assigned_tid[di] = tid

        self._tracks = [t for t in self._tracks if t.time_since_update <= self.max_age]

        out: list[dict[str, Any]] = []
        for di, det in enumerate(detections):
            item = dict(det)
            tid = assigned_tid.get(di)
            if tid is not None:
                item["track_id"] = int(tid)
            out.append(item)
        return out


def _box_4(value: Sequence[float]) -> tuple[float, float, float, float]:
    """提取 bbox 的前四个元素，作为浮点数元组返回。"""
    try:
        coords = tuple(float(v) for v in value[:4])
    except (TypeError, ValueError) as exc:
        raise TypeError(f"detection bbox must be numeric, got {value!r}") from exc
    if len(coords) < 4:
        raise ValueError(f"detection bbox needs 4 elements, got {value!r}")
    return coords  # type: ignore[return-value]


def pick_primary_track_id(detections: Iterable[dict[str, Any]]) -> int | None:
    """在一帧已跟踪的检测框中选出主目标。

    主目标是边界框面积最大的已确认检测框，面积相同时优先选择置信度更高的。
    这是跟踪完成后，凝视或接近控制器所需的"我们正在跟踪谁"的答案。

    参数：
        detections: 检测框列表，其中一部分可能携带 ``track_id``。

    返回：
        选中的 ``track_id``；若没有任何检测框携带则返回 ``None``。

    示例：
        >>> dets = [{"bbox": [0, 0, 5, 5], "track_id": 1},
        ...         {"bbox": [0, 0, 8, 8], "track_id": 2}]
        >>> pick_primary_track_id(dets)
        2
    """
    best_id: int | None = None
    best_key = (-1.0, -1.0)
    for det in detections:
        tid = det.get("track_id")
        if tid is None:
            continue
        key = (
            bbox_area(det.get("bbox") or (0, 0, 0, 0)),
            float(det.get("confidence") or 0.0),
        )
        if key > best_key:
            best_key = key
            best_id = int(tid)
    return best_id


class KeyedTrackerStore:
    """按 key 组织的线程安全（thread-safe）:class:`IoUTracker` 集合。

    当一个进程需要服务于多条相互独立、且不能共享关联状态的流（相机、会话、
    机器人 id）时非常有用。key 可以是任意可哈希值；``None`` 映射到
    ``"default"`` 跟踪器。

    参数：
        iou_threshold: 转发给每个 :class:`IoUTracker`。
        max_age: 转发给每个 :class:`IoUTracker`。
        min_hits: 转发给每个 :class:`IoUTracker`。
    """

    def __init__(
        self,
        *,
        iou_threshold: float = 0.3,
        max_age: int = 30,
        min_hits: int = 1,
    ) -> None:
        self.iou_threshold = iou_threshold
        self.max_age = max_age
        self.min_hits = min_hits
        self._lock = Lock()
        self._trackers: dict[Hashable, IoUTracker] = {}

    def reset(self, key: Hashable | None = None) -> None:
        """丢弃 ``key`` 对应的所有轨迹；若该 key 不存在则创建一个全新的跟踪器。"""
        with self._lock:
            k: Hashable = key if key is not None else "default"
            if k in self._trackers:
                self._trackers[k].reset()
            else:
                self._trackers[k] = self._new()

    def update(
        self, key: Hashable | None, detections: Sequence[dict[str, Any]]
    ) -> list[dict[str, Any]]:
        """更新 ``key`` 对应的跟踪器；首次使用时创建它。"""
        with self._lock:
            k: Hashable = key if key is not None else "default"
            tracker = self._trackers.get(k)
            if tracker is None:
                tracker = self._new()
                self._trackers[k] = tracker
            return tracker.update(detections)

    def _new(self) -> IoUTracker:
        return IoUTracker(
            iou_threshold=self.iou_threshold,
            max_age=self.max_age,
            min_hits=self.min_hits,
        )
