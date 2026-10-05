"""reusable_model.motion.planar_ik 的冒烟测试。"""

from __future__ import annotations

import numpy as np
import pytest

from reusable_model.motion.planar_ik import PlanarTwoLink, PlanarTwoLinkIK, elbow_circle


class TestPlanarTwoLinkIK:
    def test_reachable_target(self):
        geom = PlanarTwoLink(0.3, 0.35, base_z=0.06)
        ik = PlanarTwoLinkIK(geom)
        # 连杆完全伸直并竖直向上：q1=q2=0 时可达 z = l1+l2+base_z。
        result = ik.solve(0.0, geom.l1 + geom.l2 + geom.base_z)
        assert result.success
        np.testing.assert_allclose([result.q1, result.q2], [0.0, 0.0], atol=1e-6)

    def test_out_of_reach_fails(self):
        geom = PlanarTwoLink(0.3, 0.35)
        ik = PlanarTwoLinkIK(geom)
        result = ik.solve(10.0, 0.0)
        assert not result.success

    def test_joint_limits_respected(self):
        geom = PlanarTwoLink(0.3, 0.35)
        ik = PlanarTwoLinkIK(geom)
        result = ik.solve(
            0.0, geom.l1 + geom.l2,
            q1_limits=[-0.1, 0.1], q2_limits=[-0.1, 0.1],
        )
        # 要么在限制范围内成功，要么干净地失败——绝不超出范围。
        if result.success:
            assert -0.1 - 1e-6 <= result.q1 <= 0.1 + 1e-6
            assert -0.1 - 1e-6 <= result.q2 <= 0.1 + 1e-6

    def test_result_has_expected_fields(self):
        geom = PlanarTwoLink(0.3, 0.35)
        ik = PlanarTwoLinkIK(geom)
        result = ik.solve(0.2, 0.5)
        assert hasattr(result, "success")
        assert hasattr(result, "q1")
        assert hasattr(result, "q2")


class TestElbowCircle:
    def test_circle_geometry(self):
        centre, radius, u, v = elbow_circle(0.3, 0.3, [0.0, 0.0, 1.0], [0.3, 0.0, 1.0])
        np.testing.assert_allclose(centre, [0.15, 0.0, 1.0], atol=1e-9)
        assert radius == pytest.approx(0.2598076211353316)
        assert np.linalg.norm(u) == pytest.approx(1.0)
        assert np.linalg.norm(v) == pytest.approx(1.0)

    def test_out_of_reach_returns_none(self):
        assert elbow_circle(0.3, 0.3, [0.0, 0.0, 1.0], [1.0, 0.0, 1.0]) is None
