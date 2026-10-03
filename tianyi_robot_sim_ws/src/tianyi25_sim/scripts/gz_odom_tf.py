#!/usr/bin/env python3
"""把 Gazebo 里程计 /odom 转成 odom -> base_footprint 的 TF。

为什么需要这个节点（而不是直接桥接 gz 的 TF）：
  gz 的 DiffDrive 插件会把 odom TF 发到它自己的 tf_topic 上，frame 名带模型作用域
  前缀（如 tianyi25/odom、tianyi25/chassis_link），与 robot_state_publisher 发布的
  base_footprint 对不上。这里用 /odom 消息自己发 TF，frame 名完全可控：
      odom -> base_footprint
  另一半 map -> odom 由 SLAM Toolbox（或 AMCL）发布。

订阅: /odom (nav_msgs/msg/Odometry)
发布: /tf  (tf2_msgs/msg/TFMessage)
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class OdomTf(Node):
    def __init__(self) -> None:
        super().__init__("gz_odom_tf")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value

        self._br = TransformBroadcaster(self)
        # gz 侧里程计是持续高频流，用 best_effort 与 sensor 风格更匹配；
        # 这里用 reliable 也行——ros_gz_bridge 默认建 reliable 订阅，保持一致。
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_odom, qos
        )
        self.get_logger().info(
            f"publishing TF {self._odom_frame} -> {self._base_frame} from "
            f"{self.get_parameter('odom_topic').value}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        # 平面移动：只取 x/y/yaw，z 与 roll/pitch 交给本体 URDF 的固定关节决定，
        # 避免把地面接触抖动带进 TF 树（Nav2 只需要平面位姿）。
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        self._br.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()