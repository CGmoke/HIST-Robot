#!/usr/bin/env python3
"""向 Gazebo 的 JointTrajectoryController 周期下发正弦扫掠轨迹。

配合 Gazebo 内置控制器使用（无需 ros2_control）：
    发布 trajectory_msgs/JointTrajectory 到 /model/<model_name>/joint_trajectory，
    gazebo.launch.py 里的 ros_gz_bridge 会把它桥接成 gz.msgs.JointTrajectory。

关节表（中心角/振幅）、频率、相位差全部来自 config/gz_joint_sweep.yaml，
本节点不做任何硬编码，改参数只需改 YAML。

启停服务:
    std_srvs/SetBool 到 /tianyi25_sim/set_sweep_enabled（名字由 enable_service 参数决定）。
    false = 暂停扫掠（不再下发轨迹），true = 恢复；恢复时相位连续、不跳变。
    play_motion 桩在播放礼仪动作前会先关掉扫掠，避免两个节点同时往
    JointTrajectoryController 下发轨迹互相覆盖（gz 会告警 "received while
    executing a previous trajectory" 并丢弃后到的那条）。

用法:
    ros2 run tianyi25_sim gz_joint_sweep --ros-args \
        -p config_file:=<install>/share/tianyi25_sim/config/gz_joint_sweep.yaml
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import List

import rclpy
import yaml
from rclpy.node import Node
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GzJointSweep(Node):
    """把正弦扫掠采样成一条 JointTrajectory 反复下发，由 Gazebo 侧插值执行。"""

    def __init__(self) -> None:
        super().__init__("gz_joint_sweep")

        self.declare_parameter("config_file", "")
        self.declare_parameter("trajectory_topic", "")
        self.declare_parameter("enable_service", "/tianyi25_sim/set_sweep_enabled")

        cfg_path = Path(str(self.get_parameter("config_file").value))
        if not cfg_path.is_file():
            raise RuntimeError(
                f"配置文件不存在: {cfg_path}；请用 -p config_file:=<路径> 指定"
            )
        cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))

        joints = cfg["joints"]
        self._names: List[str] = list(joints.keys())
        self._center = [float(v[0]) for v in joints.values()]
        self._amp = [float(v[1]) for v in joints.values()]

        self._omega = 2.0 * math.pi * float(cfg["frequency"])
        self._phase_step = float(cfg["phase_step"])
        self._horizon = float(cfg["horizon"])
        self._dt = float(cfg["sample_dt"])
        self._period = float(cfg["publish_period"])

        topic = str(self.get_parameter("trajectory_topic").value)
        self._pub = self.create_publisher(JointTrajectory, topic, 10)
        self._t0 = self.get_clock().now()
        self._enabled = True
        self._paused_at = None

        srv_name = str(self.get_parameter("enable_service").value)
        self.create_service(SetBool, srv_name, self._on_enable)
        self.create_timer(self._period, self._publish)

        self.get_logger().info(
            f"Gazebo 关节扫掠已启动: {len(self._names)} 个关节 -> {topic}, "
            f"{self._omega / (2.0 * math.pi):.3f} Hz 正弦, "
            f"每次下发 {self._horizon}s 轨迹、每 {self._period}s 重发; "
            f"启停服务 {srv_name}"
        )

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response):
        if request.data and not self._enabled:
            # 恢复时把时间基准整体后移，让正弦相位从暂停处接上，避免关节跳变
            if self._paused_at is not None:
                self._t0 += self.get_clock().now() - self._paused_at
                self._paused_at = None
            self._enabled = True
            self._publish()
            response.message = "扫掠已恢复"
        elif not request.data and self._enabled:
            self._enabled = False
            self._paused_at = self.get_clock().now()
            response.message = "扫掠已暂停"
        else:
            response.message = "扫掠状态未变化"
        response.success = True
        self.get_logger().info(f"{response.message}（play_motion 桩调用）")
        return response

    def _position(self, index: int, t: float) -> float:
        return self._center[index] + self._amp[index] * math.sin(
            self._omega * t + index * self._phase_step
        )

    def _publish(self) -> None:
        if not self._enabled:
            return
        t_now = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        steps = max(1, int(round(self._horizon / self._dt)))

        traj = JointTrajectory()
        traj.joint_names = self._names
        for k in range(steps + 1):
            offset = k * self._dt
            point = JointTrajectoryPoint()
            point.positions = [
                self._position(i, t_now + offset) for i in range(len(self._names))
            ]
            point.time_from_start.sec = int(offset)
            point.time_from_start.nanosec = int(round((offset % 1.0) * 1e9))
            traj.points.append(point)

        self._pub.publish(traj)


def main() -> None:
    rclpy.init()
    node = GzJointSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()