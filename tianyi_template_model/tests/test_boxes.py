"""reusable_model.geometry.boxes 的测试。"""

from __future__ import annotations

import numpy as np
import pytest

from reusable_model.geometry.boxes import (
    bbox_area,
    bbox_center,
    bbox_to_mask,
    clip_bbox,
    iou,
    iou_min,
    mask_center,
    overlap_score,
)


class TestMetrics:
    def test_bbox_area(self):
        assert bbox_area([0, 0, 4, 5]) == 20.0
        assert bbox_area([4, 5, 0, 0]) == 0.0  # 反向框 -> 被裁剪为零

    def test_iou_exact_and_disjoint(self):
        assert iou([0, 0, 10, 10], [0, 0, 10, 10]) == pytest.approx(1.0)
        assert iou([0, 0, 2, 2], [10, 10, 12, 12]) == 0.0

    def test_iou_known_ratio(self):
        # 交集 = 5x5 = 25，并集 = 100 + 100 - 25 = 175
        assert iou([0, 0, 10, 10], [5, 5, 15, 15]) == pytest.approx(25 / 175)

    def test_iou_min_nested_box_is_one(self):
        assert iou_min([0, 0, 20, 20], [5, 5, 15, 15]) == 1.0

    def test_overlap_score_is_max(self):
        a, b = [0, 0, 10, 10], [0, 0, 100, 100]
        assert overlap_score(a, b) == 1.0
        assert overlap_score(a, b) == max(iou(a, b), iou_min(a, b))

    def test_extra_fields_ignored(self):
        assert iou([0, 0, 10, 10, 0.9], [0, 0, 10, 10]) == 1.0


class TestConversion:
    def test_clip_bbox(self):
        assert clip_bbox([-5, 0, 30, 10], width=20, height=20) == (0, 0, 20, 10)
        assert clip_bbox([50, 50, 60, 60], width=20, height=20) == (20, 20, 20, 20)

    def test_bbox_to_mask(self):
        m = bbox_to_mask([1, 1, 3, 3], width=4, height=4)
        assert m.shape == (4, 4)
        assert m.dtype == bool
        assert int(m.sum()) == 4
        assert not m[0, 0] and bool(m[1, 1])

    def test_bbox_to_mask_out_of_image_is_empty(self):
        m = bbox_to_mask([10, 10, 12, 12], width=4, height=4)
        assert not m.any()

    def test_bbox_center(self):
        assert bbox_center([0, 0, 4, 2]) == (2, 1)

    def test_mask_center(self):
        assert mask_center([[0, 1], [1, 0]]) == (0, 0)
        assert mask_center(np.zeros((3, 3))) is None
        assert mask_center([[0, 0], [0, 0]]) is None


class TestValidation:
    def test_bbox_requires_four_elements(self):
        with pytest.raises(ValueError):
            bbox_area([0, 0, 10])

    def test_bbox_rejects_non_numeric(self):
        with pytest.raises(TypeError):
            bbox_area([0, 0, "ten", 10])

    def test_clip_rejects_bad_resolution(self):
        with pytest.raises(ValueError):
            clip_bbox([0, 0, 1, 1], width=-1, height=10)
        with pytest.raises(TypeError):
            clip_bbox([0, 0, 1, 1], width=2.5, height=10)
