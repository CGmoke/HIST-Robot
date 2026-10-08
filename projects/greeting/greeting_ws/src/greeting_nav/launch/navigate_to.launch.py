#!/usr/bin/env python3
"""navigate_to.launch.py —— 启动【实机】导航 NavigateTo 服务端（greeting_nav）。

用法：
    ros2 launch greeting_nav navigate_to.launch.py
    ros2 launch greeting_nav navigate_to.launch.py dry_run:=true          # 无真机自测
    ros2 launch greeting_nav navigate_to.launch.py apply_speed_limit:=true

前置条件（实机）：
    1) 底盘固件在运行，REST 可达： curl http://192.168.11.10:9090/api/core/system/v1/robot/health
    2) 本机已在底盘 IP 白名单内
    3) 已完成现场建图与定位（get_pose() 有效）
    4) 🔴 急停完全弹出、工作人员持急停就位（底盘 SDK 无软件急停）

调试用：
    ros2 action send_goal /greeting/navigate_to greeting_interfaces/action/NavigateTo \
        "{waypoint: standby, timeout_s: 30.0}" --feedback
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            "waypoints_file", default_value="",
            description="点位表 YAML；留空则用 share/greeting_nav/config/waypoints.yaml",
        ),
        DeclareLaunchArgument(
            "host", default_value="192.168.11.10",
            description="底盘 REST 主机（现场按实际网段修改）",
        ),
        DeclareLaunchArgument(
            "rest_port", default_value="9090",
            description="底盘 REST 端口",
        ),
        DeclareLaunchArgument(
            "request_timeout_s", default_value="10.0",
            description="单次 HTTP 请求超时(s)",
        ),
        DeclareLaunchArgument(
            "dry_run", default_value="false",
            description="🔴 true=不驱动底盘，仅验证 Action 链路；现场实机必须为 false",
        ),
        DeclareLaunchArgument(
            "default_timeout_s", default_value="60.0",
            description="goal 未给 timeout_s 时的兜底超时(s)",
        ),
        DeclareLaunchArgument(
            "poll_period_s", default_value="0.2",
            description="轮询位姿/动作状态的周期(s)",
        ),
        DeclareLaunchArgument(
            "feedback_period_s", default_value="0.1",
            description="feedback 发布周期(s)",
        ),
        DeclareLaunchArgument(
            "max_retries", default_value="1",
            description="超时重试次数（规划方案 §7.6：重试 1 次）",
        ),
        DeclareLaunchArgument(
            "apply_speed_limit", default_value="false",
            description="启动时是否设定底盘【全局限速】；moveto 本身不接受速度参数，"
                        "这是唯一限速杠杆，默认关闭以免意外改动全局设置",
        ),
        DeclareLaunchArgument(
            "max_moving_speed_mps", default_value="0.4",
            description="【仅 apply_speed_limit:=true 生效】线速度上限(m/s)，"
                        "规划方案 §7.6 建议 ≤0.3~0.5；固件硬上限 1.5",
        ),
        DeclareLaunchArgument(
            "max_angular_speed_radps", default_value="0.5",
            description="【仅 apply_speed_limit:=true 生效】角速度上限(rad/s)，固件硬上限 1.5708",
        ),
    ]

    navigate_to_server = Node(
        package="greeting_nav",
        executable="navigate_to_server",
        name="greeting_navigate_to",
        output="screen",
        parameters=[{
            "waypoints_file": LaunchConfiguration("waypoints_file"),
            "host": LaunchConfiguration("host"),
            "rest_port": LaunchConfiguration("rest_port"),
            "request_timeout_s": LaunchConfiguration("request_timeout_s"),
            "dry_run": LaunchConfiguration("dry_run"),
            "default_timeout_s": LaunchConfiguration("default_timeout_s"),
            "poll_period_s": LaunchConfiguration("poll_period_s"),
            "feedback_period_s": LaunchConfiguration("feedback_period_s"),
            "max_retries": LaunchConfiguration("max_retries"),
            "apply_speed_limit": LaunchConfiguration("apply_speed_limit"),
            "max_moving_speed_mps": LaunchConfiguration("max_moving_speed_mps"),
            "max_angular_speed_radps": LaunchConfiguration("max_angular_speed_radps"),
        }],
    )

    return LaunchDescription(args + [navigate_to_server])
