"""reusable_model.gridmap（occupancy、regions、distance_field、descent）的冒烟测试。"""

from __future__ import annotations

import numpy as np
import pytest

from reusable_model.gridmap.descent import descent_direction
from reusable_model.gridmap.distance_field import DistanceField, create_distance_field_for_region
from reusable_model.gridmap.occupancy import FREE, OCCUPIED, GridGeometry, OccupancyGrid
from reusable_model.gridmap.regions import RegionInfo, label_connected


class TestOccupancy:
    def test_from_array_geometry(self):
        grid = OccupancyGrid.from_array(
            [[FREE, FREE], [OCCUPIED, FREE]], origin=(-1.0, -1.0), resolution=0.1
        )
        assert grid.geometry.width == 2
        assert grid.geometry.height == 2
        assert grid.data.dtype == np.uint8
        assert grid.geometry.bounds == (-1.0, -1.0, -0.8, -0.8)

    def test_shape_mismatch_rejected(self):
        geom = GridGeometry(0.0, 0.0, 0.5, 3, 2)
        with pytest.raises(ValueError):
            OccupancyGrid([[FREE, FREE], [FREE, FREE]], geom)

    def test_grid_geometry_validation(self):
        with pytest.raises(ValueError):
            GridGeometry(0.0, 0.0, 0.0, 2, 2)


class TestRegions:
    def test_label_connected(self):
        mask = np.array(
            [
                [1, 1, 0],
                [1, 0, 0],
                [0, 0, 1],
            ],
            dtype=bool,
        )
        labels, infos = label_connected(mask, connectivity=8, min_area_cells=1)
        assert set(np.unique(labels)) == {0, 1, 2}
        assert isinstance(infos, list)
        assert len(infos) == 2
        assert all(isinstance(info, RegionInfo) for info in infos)

    def test_min_area_filters(self):
        mask = np.array([[1, 0], [0, 0]], dtype=bool)
        labels, infos = label_connected(mask, connectivity=4, min_area_cells=2)
        assert len(infos) == 0


class TestDistanceField:
    def test_create_for_region(self):
        geom = GridGeometry(0.0, 0.0, 0.1, 10, 10)
        labels = np.zeros((10, 10), dtype=np.int64)
        labels[2:8, 2:8] = 1
        field = create_distance_field_for_region(
            labels, geom, region_ids=[1], target_world=[0.5, 0.5],
            resolution=0.1, brake_distance=0.2,
        )
        assert isinstance(field, DistanceField)

    def test_own_resolution_decoupled_from_map(self):
        geom = GridGeometry(0.0, 0.0, 0.2, 10, 10)
        labels = np.zeros((10, 10), dtype=np.int64)
        labels[3:7, 3:7] = 1
        field = create_distance_field_for_region(
            labels, geom, region_ids=[1], target_world=[1.0, 1.0],
            resolution=0.1, brake_distance=0.2,
        )
        # 该距离场保持自身的分辨率，并暴露距离网格。
        assert field.resolution == 0.1
        assert field.distances is not None


class TestDescent:
    def test_descent_heading_toward_lower_field(self):
        # 距离场沿 +x 方向递减：朝向应指向 +x（yaw 为 0）。
        field = np.zeros((5, 5), dtype=float)
        for px in range(5):
            field[:, px] = 4 - px  # x=0 处值为 4 …… x=4 处为 0
        yaw = descent_direction(field, robot_grid_xy=[2, 2], resolution=0.1, radius_meters=0.3)
        assert yaw is not None
        assert abs(yaw) < np.deg2rad(5.0)  # 约等于 +x

    def test_descent_none_in_invalid_region(self):
        field = np.full((3, 3), np.inf)
        assert descent_direction(field, robot_grid_xy=[1, 1], resolution=0.1) is None
