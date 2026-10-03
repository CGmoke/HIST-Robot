#!/usr/bin/env python3
"""基础仿真场景：天轶 2.5 模型显示 + 关节持续运动。

组合 = tianyi25_urdf 的 display（RSP + RViz2，关闭关节滑块 GUI）
     + tianyi25_sim 的 joint_sweep_demo（纯 rclpy 发布动态 /joint_states）

用法:
    ros2 launch tianyi25_sim display_motion.launch.py
    ros2 launch tianyi25_sim display_motion.launch.py frequency:=0.4 use_rviz:=false
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    frequency = LaunchConfiguration("frequency")
    use_rviz = LaunchConfiguration("use_rviz")

    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tianyi25_urdf"), "launch", "display.launch.py"]
            )
        ),
        launch_arguments={
            "use_gui": "false",  # 本场景用扫掠节点代替关节滑块
            "use_rviz": use_rviz,
        }.items(),
    )

    joint_sweep_demo = Node(
        package="tianyi25_sim",
        executable="joint_sweep_demo",
        name="joint_sweep_demo",
        output="screen",
        parameters=[{"frequency": frequency}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "frequency",
                default_value="0.15",
                description="关节正弦扫掠频率 (Hz)",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="是否启动 RViz2",
            ),
            display,
            joint_sweep_demo,
        ]
    )