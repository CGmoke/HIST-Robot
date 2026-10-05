"""检测批次的 NPZ 序列化。

检测结果在进程之间（例如 GPU 推理 worker 与决策循环）以 NumPy 数组的形式
传输，因为它们可能非常庞大：逐像素的掩码写成压缩的原始数组远比写成 JSON
便宜。本模块把一组检测字典打包成一个自描述、压缩的 ``.npz`` 字节负载，并能
再解包回来——这正是本项目检测流水线所用的格式，并泛化到任意字符串字段。

负载布局：

* ``count`` —— 检测数量；
* ``masks`` —— ``(N, H, W)`` 的布尔分割掩码；
* ``bboxes`` —— ``(N, 4)`` float32 的 ``(x1, y1, x2, y2)`` 边界框；
* ``confidences`` —— ``(N,)`` float32 分数；
* 每个请求的字符串字段对应一个 ``(N,)`` 对象数组；
* ``str_field_names`` —— 字符串字段的名字，这样 :func:`unpack_detections`
  无需旁路信息即可重建字典。

约定与 :mod:`reusable_model.geometry.boxes` 一致（边界框左上闭、右下开）。空检测列表
往返序列化后仍为 ``[]``。

依赖：仅 :mod:`numpy` 和 :mod:`reusable_model.geometry.boxes`。
"""

from __future__ import annotations

import logging
from io import BytesIO
from typing import Any, Mapping, Sequence

import numpy as np

from ..geometry.boxes import bbox_to_mask

logger = logging.getLogger(__name__)

__all__ = [
    "DEFAULT_STRING_FIELDS",
    "pack_detections",
    "unpack_detections",
]

#: 每个检测所记录的字符串元数据，对应多数检测流水线会附带的内容。
#: 任意子集都可传给 :func:`pack_detections`。
DEFAULT_STRING_FIELDS: tuple[str, ...] = ("label", "name_zh", "waste_type", "category_id")


def _as_bool_mask(value: Any, index: int) -> np.ndarray:
    arr = np.asarray(value)
    if arr.ndim != 2:
        raise ValueError(
            f"detection {index} mask must be 2D, got {arr.ndim}D shape {arr.shape}"
        )
    return arr.astype(bool)


def _as_bbox(value: Any, index: int) -> np.ndarray:
    try:
        coords = np.asarray([float(v) for v in value[:4]], dtype=np.float32)
    except (TypeError, ValueError, IndexError) as exc:
        raise ValueError(
            f"detection {index} bbox needs >= 4 numeric elements, got {value!r}"
        ) from exc
    return coords


def pack_detections(
    results: Sequence[Mapping[str, Any]],
    str_fields: Sequence[str] = DEFAULT_STRING_FIELDS,
) -> bytes:
    """把检测字典打包成压缩的 NPZ 字节负载。

    参数:
        results: 检测结果；每一项都必须带 ``mask``（二维 array-like）、``bbox``
            （4 个及以上元素）和 ``confidence``（数值）。缺少 ``mask`` 时会被
            容忍，并通过 :func:`reusable_model.geometry.boxes.bbox_to_mask` 从 ``bbox`` 补齐。
        str_fields: 每个检测要存储的字符串元数据字段名（缺失时存为 ``""``）。

    返回:
        以 ``bytes`` 表示的 NPZ 负载。

    异常:
        ValueError: 掩码形状不一致，或某个检测缺少必需的数值字段。

    示例:
        >>> data = pack_detections([{"mask": [[0, 1], [1, 0]],
        ...                           "bbox": [0, 0, 2, 2],
        ...                           "confidence": 0.9, "label": "x"}])
        >>> len(data) > 0
        True
    """
    if str_fields is None:
        raise TypeError("str_fields must be an iterable of names, got None")
    field_names: tuple[str, ...] = tuple(str_fields)
    buf = BytesIO()

    if not results:
        np.savez_compressed(buf, count=0)
        return buf.getvalue()

    masks: list[np.ndarray] = []
    bboxes: list[np.ndarray] = []
    confidences: list[float] = []
    for index, det in enumerate(results):
        mask = det.get("mask")
        if mask is None:
            w = int(round(float(det["bbox"][2])))
            h = int(round(float(det["bbox"][3])))
            mask = bbox_to_mask(det["bbox"], w, h)
        masks.append(_as_bool_mask(mask, index))
        bboxes.append(_as_bbox(det.get("bbox"), index))
        try:
            confidences.append(float(det["confidence"]))
        except (TypeError, ValueError, KeyError) as exc:
            raise ValueError(
                f"detection {index} needs a numeric 'confidence', got {det.get('confidence')!r}"
            ) from exc

    stacked_masks = np.stack(masks, axis=0)
    packed: dict[str, Any] = {
        "masks": stacked_masks,
        "bboxes": np.stack(bboxes, axis=0),
        "confidences": np.asarray(confidences, dtype=np.float32),
        "count": len(results),
    }
    for name in field_names:
        packed[name] = np.asarray(
            [str(det.get(name, "")) for det in results], dtype=object
        )
    packed["str_field_names"] = np.asarray(field_names, dtype=object)

    np.savez_compressed(buf, **packed)
    return buf.getvalue()


def unpack_detections(data: bytes) -> list[dict[str, Any]]:
    """从 NPZ 负载重建检测字典。

    参数:
        data: 由 :func:`pack_detections` 产生的负载。

    返回:
        一个字典列表，每项含 ``mask``（布尔数组）、``bbox``（四个浮点数的列表）、
        ``confidence``（浮点数）以及所存储的字符串字段。空负载返回 ``[]``。

    异常:
        ValueError: 如果 ``data`` 不是有效的检测负载（包括 ``None``）。

    示例:
        >>> data = pack_detections([{"mask": [[1]], "bbox": [0, 0, 1, 1],
        ...                           "confidence": 0.5}])
        >>> dets = unpack_detections(data)
        >>> dets[0]["confidence"] == 0.5
        True
    """
    if not data:
        return []
    try:
        stored = np.load(BytesIO(data), allow_pickle=True)
        count = int(stored["count"]) if "count" in stored else 0
    except Exception as exc:  # noqa: BLE001 -- 任何格式错误的负载都属于用户错误
        raise ValueError("data is not a valid detection NPZ payload") from exc
    if count == 0:
        return []

    field_names = [str(n) for n in stored["str_field_names"]] if "str_field_names" in stored else []

    results: list[dict[str, Any]] = []
    for i in range(count):
        item: dict[str, Any] = {
            "mask": stored["masks"][i],
            "bbox": stored["bboxes"][i].tolist(),
            "confidence": float(stored["confidences"][i]),
        }
        for name in field_names:
            item[name] = str(stored[name][i])
        results.append(item)
    return results
