"""用于判断目标在连续帧之间是否*稳定（stable）*。

跟踪库回答的是*目标现在在哪里*；本模块回答的是*目标是否一直待在原地*。
它维护一个连续帧的计数（streak）：在这些帧中，同一个 ``track_id`` 与其上一帧
边界框的 IoU 始终不低于给定阈值，并报告该连续帧首次越过配置阈值的帧。

这正是抓取触发、凝视保持与行为标注流水线中"仅在物体静止 N 帧后才锁定"行为
背后的模式：连续帧过滤器会剔除那些忽隐忽现或来回抖动的目标，只有在它们明确
稳定下来后才予以确认。

该逻辑对何种情况会重置连续帧刻意保持严格：

* ``track_id`` 改变会重置；
* 边界框位移过大（IoU 低于 ``stable_iou``）会重置；
* 某一帧没有检测框会将计数重置为零。

依赖：标准库与 :mod:`reusable_model.geometry.boxes`（仅 numpy）。
"""

from __future__ import annotations

import logging
from typing import Sequence

from ..geometry.boxes import iou as _box_iou

logger = logging.getLogger(__name__)

__all__ = ["StableStreak"]


class StableStreak:
    """统计单个目标保持原地的连续帧数。

    参数：
        min_stable_frames: 在 :meth:`is_stable` 返回 ``True`` 之前所需的连续
            稳定帧数。必须为正整数。
        stable_iou: 当前边界框与上一帧边界框（且 ``track_id`` 相同）被视为
            稳定所需的最小 IoU。取值必须位于 ``[0.0, 1.0]``。

    异常：
        ValueError: 参数超出其取值范围时抛出。

    示例：
        >>> s = StableStreak(min_stable_frames=2, stable_iou=0.5)
        >>> s.update(1, [0, 0, 10, 10])   # first sighting
        False
        >>> s.update(1, [1, 1, 11, 11])   # 第二帧，有重叠
        True
        >>> s.update(1, [2, 2, 12, 12])
        True
        >>> s.update(1, [50, 50, 60, 60]) # 跳走了 -> 重置
        False
    """

    def __init__(self, min_stable_frames: int = 3, stable_iou: float = 0.5) -> None:
        if min_stable_frames < 1:
            raise ValueError(
                f"min_stable_frames must be positive, got {min_stable_frames!r}"
            )
        if not 0.0 <= stable_iou <= 1.0:
            raise ValueError(f"stable_iou must be in [0, 1], got {stable_iou!r}")
        self.min_stable_frames = min_stable_frames
        self.stable_iou = stable_iou
        self.streak: int = 0
        self._prev_track_id: int | None = None
        self._prev_bbox: tuple[float, float, float, float] | None = None

    def reset(self) -> None:
        """清空连续帧计数，并遗忘上一帧。"""
        self.streak = 0
        self._prev_track_id = None
        self._prev_bbox = None

    def update(
        self, track_id: int | None, bbox: Sequence[float] | None
    ) -> bool:
        """用新的一帧推进连续帧计数，并报告稳定性。

        参数：
            track_id: 本帧被跟踪目标的 id；若本帧没有目标则传 ``None``。
            bbox: 本帧目标的 ``(x1, y1, x2, y2)`` 边界框；若不存在则传
                ``None``。只有 ``track_id`` 与 ``bbox`` 同时存在才算作一帧；
                其中任意一个传 ``None`` 都会重置连续帧计数。

        返回：
            当且仅当目标存在*且*本帧连续帧计数已达到 ``min_stable_frames``
            时返回 ``True``。

        异常：
            ValueError: 给定了 ``track_id`` 但 ``bbox`` 为 ``None``，或元素
                少于四个时抛出。
        """
        bbox4: tuple[float, float, float, float] | None = None
        if track_id is not None:
            if bbox is None:
                raise ValueError("bbox must be given whenever track_id is given")
            bbox4 = _box_4(bbox)

        if (
            track_id is not None
            and bbox4 is not None
            and self._prev_track_id == track_id
            and self._prev_bbox is not None
            and _box_iou(bbox4, self._prev_bbox) >= self.stable_iou
        ):
            self.streak += 1
        else:
            self.streak = 1 if (track_id is not None and bbox4 is not None) else 0

        self._prev_track_id = track_id
        self._prev_bbox = bbox4
        return bool(
            track_id is not None and bbox4 is not None and self.streak >= self.min_stable_frames
        )


def _box_4(value: Sequence[float]) -> tuple[float, float, float, float]:
    try:
        coords = tuple(float(v) for v in value[:4])
    except (TypeError, ValueError) as exc:
        raise TypeError(f"bbox must be numeric, got {value!r}") from exc
    if len(coords) < 4:
        raise ValueError(f"bbox needs 4 elements, got {value!r}")
    return coords  # type: ignore[return-value]
