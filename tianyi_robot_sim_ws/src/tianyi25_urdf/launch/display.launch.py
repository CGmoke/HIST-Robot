#!/usr/bin/env python3
"""天轶 2.5 模型本地显示（x86_64 原生 ROS2 Jazzy）

直接读取纯 URDF（不含 xacro 宏），因此不依赖 xacro。
用法:
    ros2 launch tianyi25_urdf display.launch.py                # 全功能：GUI 滑块 + RViz
    ros2 launch tianyi25_urdf display.launch.py use_gui:=false # 仅 RViz（关节静止）
依赖:
    ros-jazzy-joint-state-publisher-gui  (use_gui:=true 时需要)
    ros-jazzy-rviz2, ros-jazzy-robot-state-publisher
"""
from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory("tianyi25_urdf"))
    urdf_path = pkg_share / "urdf" / "tianyi25_urdf.urdf"
    rviz_config_path = pkg_share / "rviz" / "urdf.rviz"

    # 纯 URDF，无需 xacro 展开
    robot_description = urdf_path.read_text(encoding="utf-8")

    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="启动 joint_state_publisher_gui（关节滑块，需 joint-state-publisher-gui）",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="启动 RViz2",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                condition=IfCondition(use_gui),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config_path)],
                condition=IfCondition(use_rviz),
            ),
        ]
    )