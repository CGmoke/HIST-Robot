"""reusable_model.motion.speed_profile 的测试。"""

from __future__ import annotations

import math

import pytest

from reusable_model.motion.speed_profile import BrakingProfile


@pytest.fixture
def p() -> BrakingProfile:
    return BrakingProfile()


class TestAcceleration:
    def test_forward_acceleration(self, p):
        assert p.acceleration(0.0, 0.4, 0.02) == pytest.approx(0.8)

    def test_backward_acceleration(self, p):
        assert p.acceleration(0.5, 0.0, 0.02) == pytest.approx(-0.8)

    def test_angular_uses_angular_acceleration(self, p):
        assert p.acceleration(0.0, 0.1, 0.02, angular=True) == pytest.approx(2.0)

    def test_step_clamp_on_long_interval(self, p):
        # 0.8 * 1.0 > 0.1，因此单步限制生效。
        assert p.acceleration(0.0, 0.4, 1.0) == pytest.approx(0.1)
        assert p.acceleration(0.4, 0.0, 1.0) == pytest.approx(-0.2)

    def test_rejects_bad_interval(self, p):
        with pytest.raises(ValueError):
            p.acceleration(0.0, 0.1, 0.0)


class TestTargetVelocity:
    def test_ramp_never_overshoots(self, p):
        v = p.target_velocity(0.0, 0.8, 0.1)
        assert v == pytest.approx(0.08)
        v2 = p.target_velocity(0.08, 0.8, 0.1)
        assert v2 <= 0.8

    def test_decel_floor(self, p):
        v = p.target_velocity(0.5, 0.0, 0.02)
        assert v == pytest.approx(0.484)


class TestBrakingVelocity:
    def test_known_value(self):
        v = BrakingProfile.braking_velocity(0.3, 0.8)
        assert v == pytest.approx(math.sqrt(2 * 0.8 * 0.3))

    def test_sign_matches_distance(self):
        assert BrakingProfile.braking_velocity(-0.3, 0.8) < 0

    def test_rejects_zero_distance(self):
        with pytest.raises(ValueError):
            BrakingProfile.braking_velocity(0.0, 0.8)


class TestMotionStrategy:
    def test_result_within_bounds(self, p):
        v = p.motion_strategy(0.5, 0.6, 0.02, distance=0.3)
        assert 0.0 <= v <= 0.6

    def test_target_velocity_mode(self, p):
        v = p.motion_strategy(0.0, 0.6, 0.02, target_linear_velocity=0.5)
        assert v == pytest.approx(0.016)  # 0.8 * 0.02，向 0.5 加速

    def test_stop_when_no_motion(self, p):
        v = p.motion_strategy(0.4, 0.6, 0.02, distance=0.005)
        assert 0.0 <= v < 0.4

    def test_max_velocity_clamp(self, p):
        # 距离远超最大速度所能覆盖的范围：结果被限制在最大值。
        v = p.motion_strategy(0.0, 0.3, 0.02, distance=10.0)
        assert v <= 0.3

    def test_requires_a_target(self, p):
        with pytest.raises(ValueError):
            p.motion_strategy(0.0, 0.6, 0.02)
        with pytest.raises(ValueError):
            p.motion_strategy(0.0, 0.0, 0.02, distance=0.3)

    def test_angular_mode_runs(self, p):
        v = p.motion_strategy(0.0, 2.0, 0.02, distance=0.5, angular=True)
        assert 0.0 <= v <= 2.0


class TestValidation:
    def test_signed_accelerations(self):
        with pytest.raises(ValueError):
            BrakingProfile(forward_acceleration=0.0)
        with pytest.raises(ValueError):
            BrakingProfile(backward_acceleration=0.1)
        with pytest.raises(ValueError):
            BrakingProfile(angular_acceleration=0.0)
        with pytest.raises(ValueError):
            BrakingProfile(max_forward_acceleration_step=0.0)
        with pytest.raises(ValueError):
            BrakingProfile(max_backward_acceleration_step=0.1)

    def test_damping_factors(self):
        with pytest.raises(ValueError):
            BrakingProfile(braking_linear_velocity_factor=1.5)
        with pytest.raises(ValueError):
            BrakingProfile(braking_angular_velocity_factor=-0.1)
