"""reusable_model.tracking.iou_tracker 与 reusable_model.tracking.stable_streak 的测试。"""

from __future__ import annotations

import pytest

from reusable_model.tracking.iou_tracker import IoUTracker, KeyedTrackerStore, pick_primary_track_id
from reusable_model.tracking.stable_streak import StableStreak


def _det(bbox, label="p", conf=0.9):
    return {"bbox": list(bbox), "label": label, "confidence": conf}


class TestIoUTracker:
    def test_immediate_confirmation_at_min_hits_1(self):
        tr = IoUTracker(min_hits=1)
        out = tr.update([_det([0, 0, 10, 10])])
        assert out[0]["track_id"] == 1

    def test_same_track_reassigned(self):
        tr = IoUTracker(min_hits=1)
        a = tr.update([_det([0, 0, 10, 10])])
        b = tr.update([_det([2, 2, 12, 12])])
        assert a[0]["track_id"] == b[0]["track_id"]

    def test_new_track_when_disjoint(self):
        tr = IoUTracker(min_hits=1)
        a = tr.update([_det([0, 0, 10, 10])])
        b = tr.update([_det([50, 50, 60, 60])])
        assert a[0]["track_id"] != b[0]["track_id"]

    def test_min_hits_gates_confirmation(self):
        tr = IoUTracker(min_hits=3)
        first = tr.update([_det([0, 0, 10, 10])])
        assert "track_id" not in first[0]
        second = tr.update([_det([1, 1, 11, 11])])
        assert "track_id" not in second[0]
        third = tr.update([_det([2, 2, 12, 12])])
        assert third[0]["track_id"] == 1

    def test_track_expires_after_max_age(self):
        tr = IoUTracker(min_hits=1, max_age=2)
        first = tr.update([_det([0, 0, 10, 10])])
        tid = first[0]["track_id"]
        for _ in range(3):  # 3 个空帧 > max_age
            tr.update([])
        reappear = tr.update([_det([3, 3, 13, 13])])
        assert reappear[0]["track_id"] != tid

    def test_reset(self):
        tr = IoUTracker(min_hits=1)
        tr.update([_det([0, 0, 10, 10])])
        tr.reset()
        out = tr.update([_det([0, 0, 10, 10])])
        assert out[0]["track_id"] == 1

    def test_active_track_ids(self):
        tr = IoUTracker(min_hits=1)
        tr.update([_det([0, 0, 10, 10]), _det([50, 50, 60, 60])])
        assert set(tr.active_track_ids) == {1, 2}

    def test_validation(self):
        with pytest.raises(ValueError):
            IoUTracker(iou_threshold=1.5)
        with pytest.raises(ValueError):
            IoUTracker(max_age=-1)
        with pytest.raises(ValueError):
            IoUTracker(min_hits=0)


class TestKeyedTrackerStore:
    def test_keys_are_isolated(self):
        store = KeyedTrackerStore(min_hits=1)
        a = store.update("cam-a", [_det([0, 0, 10, 10])])
        b = store.update("cam-b", [_det([0, 0, 10, 10])])
        assert a[0]["track_id"] == b[0]["track_id"] == 1  # 各自从 1 开始

    def test_none_key_uses_default(self):
        store = KeyedTrackerStore(min_hits=1)
        out = store.update(None, [_det([0, 0, 10, 10])])
        assert out[0]["track_id"] == 1


class TestPickPrimary:
    def test_largest_box_wins(self):
        dets = [
            {"bbox": [0, 0, 5, 5], "track_id": 1},
            {"bbox": [0, 0, 8, 8], "track_id": 2},
        ]
        assert pick_primary_track_id(dets) == 2

    def test_none_when_no_ids(self):
        assert pick_primary_track_id([{"bbox": [0, 0, 5, 5]}]) is None


class TestStableStreak:
    def test_becomes_stable_after_threshold(self):
        s = StableStreak(min_stable_frames=2, stable_iou=0.5)
        assert s.update(1, [0, 0, 10, 10]) is False
        assert s.update(1, [1, 1, 11, 11]) is True
        assert s.update(1, [2, 2, 12, 12]) is True

    def test_jump_resets_streak(self):
        s = StableStreak(min_stable_frames=2, stable_iou=0.5)
        s.update(1, [0, 0, 10, 10])
        s.update(1, [1, 1, 11, 11])
        assert s.update(1, [50, 50, 60, 60]) is False
        assert s.streak == 1

    def test_change_of_track_id_resets(self):
        s = StableStreak(min_stable_frames=2, stable_iou=0.5)
        s.update(1, [0, 0, 10, 10])
        assert s.update(2, [1, 1, 11, 11]) is False
        assert s.streak == 1

    def test_missing_detection_zeroes_streak(self):
        s = StableStreak(min_stable_frames=2, stable_iou=0.5)
        s.update(1, [0, 0, 10, 10])
        s.update(1, [1, 1, 11, 11])
        s.update(None, None)
        assert s.streak == 0

    def test_bbox_required_with_track_id(self):
        s = StableStreak()
        with pytest.raises(ValueError):
            s.update(1, None)

    def test_validation(self):
        with pytest.raises(ValueError):
            StableStreak(min_stable_frames=0)
        with pytest.raises(ValueError):
            StableStreak(stable_iou=1.5)
