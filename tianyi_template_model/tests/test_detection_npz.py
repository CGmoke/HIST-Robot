"""reusable_model.io.detection_npz 的测试。"""

from __future__ import annotations

import numpy as np
import pytest

from reusable_model.io.detection_npz import pack_detections, unpack_detections


def _sample_results():
    return [
        {
            "mask": np.array([[0, 1], [1, 0]], dtype=bool),
            "bbox": [0, 0, 2, 2],
            "confidence": 0.9,
            "label": "bottle",
            "name_zh": "瓶子",
        },
        {
            "mask": np.array([[1, 1], [1, 1]], dtype=bool),
            "bbox": [5, 5, 9, 9],
            "confidence": 0.4,
            "label": "can",
        },
    ]


class TestRoundTrip:
    def test_full_round_trip(self):
        results = _sample_results()
        data = pack_detections(results)
        out = unpack_detections(data)
        assert len(out) == 2
        assert np.array_equal(out[0]["mask"], results[0]["mask"])
        assert out[0]["bbox"] == [0.0, 0.0, 2.0, 2.0]
        assert out[0]["confidence"] == pytest.approx(0.9)
        assert out[0]["label"] == "bottle"
        assert out[0]["name_zh"] == "瓶子"
        assert out[1]["label"] == "can"

    def test_empty_list_round_trip(self):
        data = pack_detections([])
        assert data
        assert unpack_detections(data) == []

    def test_empty_bytes(self):
        assert unpack_detections(b"") == []

    def test_missing_mask_filled_from_bbox(self):
        data = pack_detections([{"bbox": [0, 0, 3, 3], "confidence": 0.5}])
        out = unpack_detections(data)
        assert out[0]["mask"].shape == (3, 3)
        assert out[0]["mask"].sum() == 9

    def test_custom_string_fields(self):
        data = pack_detections(
            [{"mask": [[1]], "bbox": [0, 0, 1, 1], "confidence": 0.5, "kind": "A"}],
            str_fields=("kind",),
        )
        out = unpack_detections(data)
        assert out[0]["kind"] == "A"


class TestValidation:
    def test_mask_must_be_2d(self):
        with pytest.raises(ValueError):
            pack_detections([{"mask": np.zeros((2, 2, 2)), "bbox": [0, 0, 2, 2], "confidence": 0.5}])

    def test_confidence_required(self):
        with pytest.raises(ValueError):
            pack_detections([{"mask": [[1]], "bbox": [0, 0, 1, 1]}])

    def test_corrupt_payload_rejected(self):
        with pytest.raises(ValueError):
            unpack_detections(b"not an npz file at all")
