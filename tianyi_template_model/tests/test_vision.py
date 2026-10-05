"""reusable_model.vision（pinhole、depth、planes、pointcloud）的冒烟测试。"""

from __future__ import annotations

import numpy as np
import pytest

from reusable_model.vision.depth import sample_depth
from reusable_model.vision.pinhole import Intrinsics, pixel_to_3d, project, ray_direction
from reusable_model.vision.planes import fit_plane, fit_plane_oriented, ray_plane_intersection
from reusable_model.vision.pointcloud import backproject_depth, filter_by_height, voxel_downsample


@pytest.fixture
def intr() -> Intrinsics:
    return Intrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480)


class TestPinhole:
    def test_project_pixel_round_trip(self, intr):
        pt = pixel_to_3d(400.0, 300.0, 2.0, intr)
        u, v, z = project(pt[0], pt[1], pt[2], intr)
        assert u == pytest.approx(400.0, abs=1e-6)
        assert v == pytest.approx(300.0, abs=1e-6)
        # project() 返回的深度以原始深度图像单位（毫米）表示。
        assert z == pytest.approx(2.0 * intr.depth_scale)

    def test_principal_axis_projects_to_center(self, intr):
        pt = pixel_to_3d(intr.cx, intr.cy, 1.5, intr)
        u, v, _ = project(pt[0], pt[1], pt[2], intr)
        assert u == pytest.approx(intr.cx)
        assert v == pytest.approx(intr.cy)

    def test_ray_direction_unit_length(self, intr):
        ray = ray_direction(100.0, 200.0, intr)
        assert np.linalg.norm(ray) == pytest.approx(1.0)
        assert ray[2] > 0

    def test_intrinsics_validation(self):
        with pytest.raises(ValueError):
            Intrinsics(fx=0.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480)


class TestDepth:
    def test_sample_depth_median(self):
        depth = np.full((100, 100), 2000, dtype=np.uint16)  # 毫米
        d = sample_depth(depth, 50, 50, depth_scale=1000.0, window=5)
        assert d == pytest.approx(2.0)

    def test_sample_depth_filters_zeros(self):
        depth = np.zeros((30, 30), dtype=np.uint16)
        depth[14:16, 14:16] = 1000
        d = sample_depth(depth, 15, 15, depth_scale=1000.0, window=5)
        assert d == pytest.approx(1.0)

    def test_sample_depth_none_when_invalid(self):
        depth = np.zeros((30, 30), dtype=np.uint16)
        assert sample_depth(depth, 15, 15, depth_scale=1000.0, window=5) is None


class TestPlanes:
    def test_fit_plane_horizontal(self):
        xs = np.linspace(-1, 1, 11)
        ys = np.linspace(-1, 1, 11)
        gx, gy = np.meshgrid(xs, ys)
        points = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, 0.5)], axis=1)
        normal, centroid = fit_plane(points)
        assert centroid is not None
        assert normal is not None
        # 法向量应平行于 z 轴。
        assert abs(abs(normal[2]) - 1.0) < 1e-6
        assert np.linalg.norm(normal) == pytest.approx(1.0)

    def test_fit_plane_oriented_points_up(self):
        points = np.array(
            [[-1, -1, 1.0], [1, -1, 1.0], [-1, 1, 1.0], [1, 1, 1.0]], dtype=float
        )
        normal, _ = fit_plane_oriented(points, reference_axis=2, sign=1.0)
        assert normal is not None
        assert normal[2] > 0  # 向上

    def test_ray_plane_intersection(self):
        hit = ray_plane_intersection(
            [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0]
        )
        assert hit is not None
        np.testing.assert_allclose(hit, [0.0, 0.0, 2.0], atol=1e-9)

    def test_ray_plane_parallel_returns_none(self):
        hit = ray_plane_intersection(
            [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0]
        )
        assert hit is None


class TestPointCloud:
    def test_backproject_depth(self, intr):
        depth = np.full((4, 4), 1000, dtype=np.uint16)  # 1 米
        points, valid = backproject_depth(depth, intr, depth_scale=1000.0, stride=1)
        assert points.shape[1] == 3
        assert valid.any()
        assert np.all(points[:, 2] == pytest.approx(1.0))

    def test_voxel_downsample(self):
        rng = np.random.default_rng(0)
        points = rng.uniform(0.0, 1.0, size=(1000, 3))
        down = voxel_downsample(points, voxel_size=0.5)
        assert down.shape[0] < points.shape[0]
        assert down.shape[1] == 3

    def test_filter_by_height(self):
        points = np.array([[0, 0, 0.1], [0, 0, 1.5], [0, 0, 2.0]], dtype=float)
        kept = filter_by_height(points, z_min=0.5, z_max=2.5)
        assert kept.shape[0] == 2
