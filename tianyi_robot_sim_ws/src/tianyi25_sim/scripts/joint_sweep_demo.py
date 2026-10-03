#!/usr/bin/env python3
"""关节扫掠演示节点 —— 让 RViz 中的天轶 2.5 模型持续运动。

纯 rclpy + sensor_msgs 实现，不依赖 joint_state_publisher_gui，
用于在缺少 GUI 依赖时验证「运动控制功能可响应」。

所有扫掠关节的中心值与振幅均落在 URDF <limit> 范围内（见下表），
腿部关节保持 0 不动，仅驱动腰/头/双臂，符合迎宾场景的安全约定。

用法:
    ros2 run tianyi25_sim joint_sweep_demo
    ros2 run tianyi25_sim joint_sweep_demo --ros-args -p frequency:=0.3 -p amplitude_scale:=0.5
"""
from __future__ import annotations

import math
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# 关节名 -> (中心角 rad, 振幅 rad)；振幅比例由 amplitude_scale 参数统一缩放
#
# ⚠️ 必须发布 URDF 中【全部可动关节】，一个都不能少：
#    robot_state_publisher 只会为 /joint_states 里出现过的关节发布 TF。
#    漏掉任一可动关节 → 该处 TF 缺失 → TF 树被割裂成多棵
#    （tf2 报 "Tf has two or more unconnected trees."）。
#    因此非扫掠关节也以振幅 0 发布，把树补齐。
SWEEP_JOINTS: Dict[str, Tuple[float, float]] = {
    # —— 保持不动（振幅 0），仅用于补齐 TF 树 ——
    "first_leg_pitch_joint": (0.0, 0.0),     # limit 0.0 ~ 1.3090
    "second_leg_pitch_joint": (0.0, 0.0),    # limit -2.6180 ~ 0.0
    "shoulder_yaw_l_joint": (0.0, 0.0),      # limit ±2.9671
    "shoulder_yaw_r_joint": (0.0, 0.0),      # limit ±2.9671
    "elbow_yaw_l_joint": (0.0, 0.0),         # limit ±2.7925
    "elbow_yaw_r_joint": (0.0, 0.0),         # limit ±2.7925
    "wrist_roll_l_joint": (0.0, 0.0),        # limit ±1.6057
    "wrist_roll_r_joint": (0.0, 0.0),        # limit ±1.6057
    # —— 参与扫掠 ——
    "waist_pitch_joint": (0.15, 0.15),       # limit 0.0 ~ 2.1817
    "waist_yaw_joint": (0.0, 0.35),          # limit ±3.1416
    "head_yaw_joint": (0.0, 0.60),           # limit ±1.5708
    "head_pitch_joint": (0.0, 0.30),         # limit ±0.4363
    "head_roll_joint": (0.0, 0.25),          # limit ±0.4538
    "shoulder_pitch_l_joint": (0.0, 0.50),   # limit ±2.9671
    "shoulder_pitch_r_joint": (0.0, 0.50),   # limit ±2.9671
    "shoulder_roll_l_joint": (0.45, 0.45),   # limit -0.2094 ~ 2.6180
    "shoulder_roll_r_joint": (-0.45, 0.45),  # limit -2.6180 ~ 0.2094
    "elbow_pitch_l_joint": (-0.75, 0.75),    # limit -2.6180 ~ 0.2618
    "elbow_pitch_r_joint": (-0.75, 0.75),    # limit -2.6180 ~ 0.2618
    "wrist_pitch_l_joint": (0.10, 0.35),     # limit -0.7854 ~ 1.0472
    "wrist_pitch_r_joint": (0.10, 0.35),     # limit -0.7854 ~ 1.0472
}

# 各关节相位错开，运动看起来更自然
PHASE_STEP = 0.5


class JointSweepDemo(Node):
    """周期性发布 /joint_states，位置按正弦规律扫掠。"""

    def __init__(self) -> None:
        super().__init__("joint_sweep_demo")
        self._rate_hz = float(self.declare_parameter("rate", 50.0).value)
        self._freq_hz = float(self.declare_parameter("frequency", 0.15).value)
        self._scale = float(self.declare_parameter("amplitude_scale", 1.0).value)

        self._names = list(SWEEP_JOINTS)
        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self._t0 = self.get_clock().now()

        self.create_timer(1.0 / self._rate_hz, self._on_timer)
        self.get_logger().info(
            f"关节扫掠演示已启动: {len(self._names)} 个关节, "
            f"{self._freq_hz} Hz 正弦, 发布频率 {self._rate_hz} Hz"
        )

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        t = (now - self._t0).nanoseconds * 1e-9
        omega = 2.0 * math.pi * self._freq_hz

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = self._names
        msg.position = [
            center + amp * self._scale * math.sin(omega * t + i * PHASE_STEP)
            for i, (center, amp) in enumerate(SWEEP_JOINTS.values())
        ]
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = JointSweepDemo()
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