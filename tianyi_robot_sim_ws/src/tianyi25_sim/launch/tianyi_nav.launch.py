#!/usr/bin/env python3
"""SLAM Toolbox（在线建图）+ Nav2 导航栈 —— 仿真链路第 4 段。

启动内容：
    slam_toolbox online_async_launch.py   建图；发布 /map 与 TF 的 map->odom 段
    nav2_bringup navigation_launch.py     controller/planner/behavior/BT/waypoint/smoother
    rviz2（可选，rviz:=true）              加载 Nav2 默认视图，方便看代价地图与路径

不启动 AMCL / map_server：定位由 SLAM Toolbox 在线建图承担（本项目选型）。

TF 链（本文件补上缺少的 map->odom 段，其余由 gazebo.launch.py 提供）：
    map ─slam_toolbox→ odom ─gz_odom_tf→ base_footprint ─robot_state_publisher→ 其余

Nav2 输出的 /cmd_vel 经 ros_gz_bridge 交给 gz DiffDrive 插件驱动底盘。

用法：
    # 已单独跑着 gazebo.launch.py 时，只补这一段：
    ros2 launch tianyi25_sim tianyi_nav.launch.py
    # 看建图与路径：
    ros2 launch tianyi25_sim tianyi_nav.launch.py rviz:=true

前提（一次性）：
    sudo apt install -y ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs \\
                        ros-jazzy-slam-toolbox ros-jazzy-nav2-rviz-plugins
    安装后无需重新 colcon build，直接再次运行本 launch 即可。
"""
from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _require(pkg: str, install_hint: str) -> Path:
    """取包 share 目录；缺失时抛出【可读】的错误，而不是 launch traceback。"""
    try:
        return Path(get_package_share_directory(pkg))
    except Exception as exc:  # PackageNotFoundError
        raise RuntimeError(
            f"\n[tianyi_nav] 找不到 ROS 包 '{pkg}'：{exc}\n"
            f"  请先安装（无需 sudo 以外的准备）：\n      {install_hint}\n"
            f"  装好后直接重跑本 launch，不需要重新 colcon build。\n"
        ) from exc


def generate_launch_description() -> LaunchDescription:
    sim_share = Path(get_package_share_directory("tianyi25_sim"))
    slam_share = _require("slam_toolbox", "sudo apt install -y ros-jazzy-slam-toolbox")
    nav2_share = _require(
        "nav2_bringup",
        "sudo apt install -y ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs",
    )

    slam_params = str(sim_share / "config" / "slam_toolbox_params.yaml")
    nav2_params = str(sim_share / "config" / "nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(slam_share / "launch" / "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": slam_params,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_share / "launch" / "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params,
            "use_sim_time": use_sim_time,
            "autostart": "true",  # 免去手动 ros2 lifecycle set configure/activate
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(nav2_share / "rviz" / "nav2_default_view.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="统一用 Gazebo 仿真时间"
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="是否顺带启动 RViz2（Nav2 默认视图，需 nav2_rviz_plugins）",
            ),
            LogInfo(
                msg=(
                    "tianyi_nav: 启动 SLAM Toolbox（在线建图）+ Nav2 导航栈。\n"
                    "  提示：机器人需要先「动一动」，SLAM 才能把 /map 建起来，"
                    "NavFn 也才能在已知栅格上规划路径。\n"
                    "  手动试走：ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "
                    "\"{linear: {x: 0.15}}\""
                )
            ),
            slam,
            navigation,
            rviz_node,
        ]
    )