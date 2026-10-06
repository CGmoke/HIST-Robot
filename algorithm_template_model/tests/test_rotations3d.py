"""reusable_model.geometry.rotations3d 的冒烟测试。"""

from __future__ import annotations

import numpy as np
import pytest

from reusable_model.geometry.rotations3d import (
    assert_rotation_matrix,
    euler_to_matrix,
    is_rotation_matrix,
    matrix_to_euler,
    rotation_between,
    rotation_from_axis,
    wrap_to_limits,
)


class TestRotationMatrices:
    def test_euler_round_trip(self):
        angles = [0.3, -0.4, 1.1]
        m = euler_to_matrix(angles, "ZYX")
        back = matrix_to_euler(m, "ZYX")
        np.testing.assert_allclose(back, angles, atol=1e-6)

    def test_euler_round_trip_proper_euler(self):
        angles = [0.3, 0.7, -0.5]
        m = euler_to_matrix(angles, "XYZ")  # 真正的欧拉角：x-y-x 族？不——XYZ 属于 Tait-Bryan
        # 改用真正的欧拉序列。
        m2 = euler_to_matrix(angles, "XYX")
        back = matrix_to_euler(m2, "XYX")
        np.testing.assert_allclose(back, angles, atol=1e-6)

    def test_is_rotation_matrix(self):
        m = euler_to_matrix([0.1, 0.2, 0.3], "ZYX")
        assert is_rotation_matrix(m)
        assert is_rotation_matrix(np.eye(3))
        assert not is_rotation_matrix(np.zeros((3, 3)))

    def test_assert_rotation_matrix_passes_through(self):
        m = euler_to_matrix([0.1, 0.2, 0.3], "ZYX")
        assert assert_rotation_matrix(m) is not None

    def test_rotation_between_known(self):
        r = rotation_between([1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
        # +x 必须映射到 +y。
        np.testing.assert_allclose(r @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)

    def test_rotation_between_same_vector_is_identity(self):
        r = rotation_between([0.0, 0.0, 1.0], [0.0, 0.0, 1.0])
        np.testing.assert_allclose(r, np.eye(3), atol=1e-9)

    def test_rotation_between_opposite_vectors(self):
        # 对径向量对应绕任意垂直轴的 180 度旋转。
        r = rotation_between([1.0, 0.0, 0.0], [-1.0, 0.0, 0.0])
        np.testing.assert_allclose(r @ np.array([1.0, 0.0, 0.0]), [-1.0, 0.0, 0.0], atol=1e-6)

    def test_rotation_from_axis(self):
        r = rotation_from_axis("z", [1.0, 0.0, 0.0])
        # 机体 +z 轴被映射到 +x。
        np.testing.assert_allclose(r[:, 2], [1.0, 0.0, 0.0], atol=1e-9)
        assert is_rotation_matrix(r)


class TestWrapToLimits:
    def test_wraps_outside_limits(self):
        wrapped = wrap_to_limits(3.5, 0.0, -3.0, 3.0)
        assert -3.0 <= wrapped <= 3.0

    def test_no_change_when_inside(self):
        assert wrap_to_limits(0.5, 0.0, -1.0, 1.0) == pytest.approx(0.5)
