"""面向轨迹控制器的制动感知速度曲线（brake-aware velocity profile）。

给定距目标的剩余距离与机器人当前速度，这些辅助函数回答驱动控制器在每个控制周期
都要问的那一个问题：*我现在应当下发多大的速度？* 答案由两个运动学思想构成：

* *制动距离（braking distance）。* 以 ``a`` 减速的物体需要 ``v**2 / (2a)``
  米才能停下。将其反解即可得到此刻仍能恰好停在目标点所能下发的速度：
  ``v = sqrt(2 * a * d)``。
* *加速度斜坡（acceleration ramp）。* 即便目标速度已知，下发速度也不能突变；
  它每步以受限的变化率逼近目标速度。当控制周期变慢时，该变化率会被进一步钳制，
  从而避免一个迟到的控制步一次性注入巨大的加速度。

通过在目标点附近对制动速度进行阻尼即可实现平稳停靠：当 ``abs(distance)`` 小于
某个 *距离系数* 时，制动速度会按指数混合被按比例调小，从而抑制线性轴与角度轴的
超调（overshoot）。

本模块是纯运动学计算——不涉及传感器、障碍物数据或硬件。:class:`BrakingProfile`
汇集了所有可调常数；其方法都是各自参数的纯函数，因此非常易于单元测试。

依赖：仅标准库。
"""

from __future__ import annotations

import logging
import math

logger = logging.getLogger(__name__)

__all__ = ["BrakingProfile"]


class BrakingProfile:
    """带制动距离控制的运动学速度曲线。

    所有加速度都是带符号的：正值沿正速度方向加速，负值减速。
    ``max_forward_acceleration_step`` 与 ``max_backward_acceleration_step``
    限制*单步*加速度变化量，以防控制间隔过长。

    参数：
        forward_acceleration: 加速时的线性加速度（m/s^2）。必须为正。
        backward_acceleration: 减速时的线性加速度（m/s^2）。必须为负。
        angular_acceleration: 角加速度，两个方向均取该幅值（rad/s^2）。
            必须为正。
        max_forward_acceleration_step: 线性速度单步允许的最大增量
            （m/s^2）。必须为正。
        max_backward_acceleration_step: 线性速度单步允许的最大减量
            （m/s^2）。必须为负。
        braking_linear_distance_factor: 低于该距离时线性制动速度开始被
            阻尼（m）。
        braking_linear_exponential_factor: 线性阻尼混合的指数；取值越大，
            越接近目标点时才降低速度。
        braking_linear_velocity_factor: 距离约为 0 时保留的制动速度最小
            比例（线性轴）。
        braking_angular_distance_factor: 低于该距离时角度制动速度开始被
            阻尼（rad）。
        braking_angular_exponential_factor: 角度阻尼混合的指数。
        braking_angular_velocity_factor: 距离约为 0 时保留的制动速度最小
            比例（角度轴）。

    异常：
        ValueError: 带符号参数符号错误，或阻尼系数超出 ``[0, 1]``。
    """

    def __init__(
        self,
        *,
        forward_acceleration: float = 0.8,
        backward_acceleration: float = -0.8,
        angular_acceleration: float = 2.0,
        max_forward_acceleration_step: float = 0.1,
        max_backward_acceleration_step: float = -0.2,
        braking_linear_distance_factor: float = 0.65,
        braking_linear_exponential_factor: float = 1.2,
        braking_linear_velocity_factor: float = 0.2,
        braking_angular_distance_factor: float = math.pi / 2,
        braking_angular_exponential_factor: float = 1.5,
        braking_angular_velocity_factor: float = 0.1,
    ) -> None:
        if forward_acceleration <= 0:
            raise ValueError(
                f"forward_acceleration must be positive, got {forward_acceleration!r}"
            )
        if backward_acceleration >= 0:
            raise ValueError(
                f"backward_acceleration must be negative, got {backward_acceleration!r}"
            )
        if angular_acceleration <= 0:
            raise ValueError(
                f"angular_acceleration must be positive, got {angular_acceleration!r}"
            )
        if max_forward_acceleration_step <= 0:
            raise ValueError(
                f"max_forward_acceleration_step must be positive, "
                f"got {max_forward_acceleration_step!r}"
            )
        if max_backward_acceleration_step >= 0:
            raise ValueError(
                f"max_backward_acceleration_step must be negative, "
                f"got {max_backward_acceleration_step!r}"
            )
        self.forward_acceleration = forward_acceleration
        self.backward_acceleration = backward_acceleration
        self.angular_acceleration = angular_acceleration
        self.max_forward_acceleration_step = max_forward_acceleration_step
        self.max_backward_acceleration_step = max_backward_acceleration_step
        self.braking_linear_distance_factor = braking_linear_distance_factor
        self.braking_linear_exponential_factor = braking_linear_exponential_factor
        self.braking_linear_velocity_factor = braking_linear_velocity_factor
        self.braking_angular_distance_factor = braking_angular_distance_factor
        self.braking_angular_exponential_factor = braking_angular_exponential_factor
        self.braking_angular_velocity_factor = braking_angular_velocity_factor
        self._validate_factor("braking_linear_velocity_factor", braking_linear_velocity_factor)
        self._validate_factor("braking_angular_velocity_factor", braking_angular_velocity_factor)

    @staticmethod
    def _validate_factor(name: str, value: float) -> None:
        if not 0.0 <= value <= 1.0:
            raise ValueError(f"{name} must be in [0, 1], got {value!r}")

    # ------------------------------------------------------------------
    # 基础构件
    # ------------------------------------------------------------------
    def acceleration(
        self,
        current_velocity: float,
        target_velocity: float,
        time_interval: float,
        *,
        angular: bool = False,
    ) -> float:
        """本次控制步可用的带符号加速度。

        单步幅值会被钳制，使得较长的控制间隔（例如错过一次定时器 tick）不会
        一次性注入巨大的速度变化：在这种情况下，返回的加速度*就是*该步长限制。

        参数：
            current_velocity: 控制步开始时的速度（m/s 或 rad/s）。
            target_velocity: 控制步结束时期望达到的速度。
            time_interval: 控制步时长，单位秒；必须为正。
            angular: 为 ``True`` 时选择角度轴。

        返回：
            带符号的加速度，单位 m/s^2（或 rad/s^2）。

        异常：
            ValueError: 当 ``time_interval`` 不为正时抛出。

        示例：
            >>> p = BrakingProfile()
            >>> p.acceleration(0.0, 0.4, 0.02)
            0.8
            >>> p.acceleration(0.0, 0.4, 1.0)   # 一大步 -> 被限幅
            0.1
        """
        if time_interval <= 0:
            raise ValueError(
                f"time_interval must be positive, got {time_interval!r}"
            )
        if angular:
            return self.angular_acceleration if target_velocity > current_velocity else -self.angular_acceleration
        if target_velocity > current_velocity:
            accel = self.forward_acceleration
            step = self.max_forward_acceleration_step
        else:
            accel = self.backward_acceleration
            step = self.max_backward_acceleration_step
        if abs(accel * time_interval) > abs(step):
            return step
        return accel

    def target_velocity(
        self,
        current_velocity: float,
        target_velocity: float,
        time_interval: float,
        *,
        angular: bool = False,
    ) -> float:
        """将速度变化钳制到一个控制步内可实现的斜坡。

        参数：
            current_velocity: 控制步开始时的速度。
            target_velocity: 控制步结束时期望达到的速度。
            time_interval: 控制步时长，单位秒。
            angular: 为 ``True`` 时选择角度轴。

        返回：
            应用加速度斜坡后的新速度，绝不越过 ``target_velocity`` 而产生超调。

        示例：
            >>> p = BrakingProfile()
            >>> round(p.target_velocity(0.0, 0.8, 0.1), 2)
            0.08
        """
        accel = self.acceleration(current_velocity, target_velocity, time_interval, angular=angular)
        delta = accel * time_interval
        if target_velocity > current_velocity:
            return min(current_velocity + delta, target_velocity)
        return max(current_velocity + delta, target_velocity)

    @staticmethod
    def braking_velocity(distance: float, acceleration: float) -> float:
        """恰好停在目标点所需的速度：``sqrt(2 * |a| * |d|)``。

        返回值的符号与 ``distance`` 的符号一致。

        参数：
            distance: 剩余距离，前方为正（m 或 rad）。必须非零。
            acceleration: 可用的带符号减速度幅值（m/s^2 或 rad/s^2）。
                其符号被忽略。

        返回：
            带符号速度 ``sign(distance) * sqrt(2 * |acceleration| * |distance|)``。

        异常：
            ValueError: 当 ``distance`` 为零时抛出。
        """
        if distance == 0:
            raise ValueError(f"distance must be non-zero, got 0.0")
        return math.sqrt(2.0 * abs(acceleration) * abs(distance)) * (1.0 if distance > 0 else -1.0)

    # ------------------------------------------------------------------
    # 综合策略
    # ------------------------------------------------------------------
    def motion_strategy(
        self,
        current_velocity: float,
        max_velocity: float,
        time_interval: float,
        *,
        distance: float | None = None,
        target_linear_velocity: float | None = None,
        angular: bool = False,
        navigating: bool = False,
    ) -> float:
        """计算本次控制步应当下发的速度。

        ``distance``（制动到目标点）与 ``target_linear_velocity``（巡航目标
        速度）二选一；至少提供其中一个，当两者都给出时 ``distance`` 优先。

        参数：
            current_velocity: 当前速度（m/s 或 rad/s）。
            max_velocity: 绝对速度上限（m/s 或 rad/s）。必须为正。
            time_interval: 控制步时长，单位秒。
            distance: 到停止点的带符号剩余距离（m 或 rad）。
                接近零时会对制动速度进行阻尼以避免超调。
            target_linear_velocity: 未给出距离时要达到的固定速度
                （m/s 或 rad/s）。
            angular: 为 ``True`` 时选择角度轴及其对应整定参数。
            navigating: 为 ``True`` 时选择导航整定参数组（不同的阻尼距离）。

        返回：
            本次控制步应当下发的速度。若 ``distance`` 与
            ``target_linear_velocity`` 都不具实际意义（均为零或 ``None``），
            结果为按斜坡停车，即目标速度变为 0。

        异常：
            ValueError: 当 ``max_velocity`` 不为正，或 ``distance`` 与
                ``target_linear_velocity`` 均为 ``None`` 时抛出。

        示例：
            >>> p = BrakingProfile()
            >>> v = p.motion_strategy(0.5, 0.6, 0.02, distance=0.3)
            >>> 0.0 <= v <= 0.6
            True
        """
        if max_velocity <= 0:
            raise ValueError(f"max_velocity must be positive, got {max_velocity!r}")
        if distance is None and target_linear_velocity is None:
            raise ValueError(
                "either distance or target_linear_velocity must be given"
            )

        has_motion = (distance is not None and abs(distance) > 0.01) or (
            target_linear_velocity is not None and abs(target_linear_velocity) > 0.01
        )

        if has_motion:
            if distance is not None:
                accel = self.acceleration(
                    current_velocity, 0.0, time_interval, angular=angular
                )
                braking_velocity = self.braking_velocity(distance, accel)
            else:
                braking_velocity = float(target_linear_velocity)  # type: ignore[arg-type]

            if angular:
                if distance is not None:
                    d_factor, e_factor, v_factor = self._damping_angular(navigating)
                    braking_velocity *= self._damp(distance, d_factor, e_factor, v_factor)
            else:
                if distance is not None:
                    d_factor, e_factor, v_factor = self._damping_linear(navigating)
                    braking_velocity *= self._damp(distance, d_factor, e_factor, v_factor)

            if abs(braking_velocity) > max_velocity:
                braking_velocity = max_velocity * math.copysign(1.0, braking_velocity)
        else:
            braking_velocity = 0.0

        return self.target_velocity(
            current_velocity, braking_velocity, time_interval, angular=angular
        )

    # ------------------------------------------------------------------
    # 阻尼辅助函数
    # ------------------------------------------------------------------
    def _damping_linear(self, navigating: bool) -> tuple[float, float, float]:
        if navigating:
            return (1.0, 1.5, 0.2)
        return (
            self.braking_linear_distance_factor,
            self.braking_linear_exponential_factor,
            self.braking_linear_velocity_factor,
        )

    def _damping_angular(self, navigating: bool) -> tuple[float, float, float]:
        if navigating:
            return (math.pi / 2, 1.6, 0.10)
        return (
            self.braking_angular_distance_factor,
            self.braking_angular_exponential_factor,
            self.braking_angular_velocity_factor,
        )

    @staticmethod
    def _damp(
        distance: float,
        distance_factor: float,
        exponential_factor: float,
        velocity_factor: float,
    ) -> float:
        """根据剩余距离计算 ``[0, 1]`` 区间的阻尼系数。

        当 ``|distance| >= distance_factor`` 时该系数为 1（全速）；当
        ``|distance| = 0`` 时该系数为 ``velocity_factor``。指数控制随着逼近
        目标点阻尼介入的快慢。
        """
        if distance_factor <= 0:
            return 1.0
        scaling = min(abs(distance) / distance_factor, 1.0)
        scaling = scaling**exponential_factor
        return 1.0 * scaling + (1.0 - scaling) * velocity_factor
