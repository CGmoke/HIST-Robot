#!/usr/bin/env python3
"""teleop.launch.py —— 启动遥控器按键网关（greeting_teleop/joy_mapper）。

输入：/sbus_data/event（bodyctrl_msgs/msg/SbusData，平台官方按键事件话题）
输出：/greeting/panel_command（std_msgs/String）
      下游：panel_command_bridge（动作快捷键）/ flow_command_bridge（全流程命令 -> orchestrator）

用法：
    # 正常启动（需已标定 config/teleop_map.yaml）
    ros2 launch greeting_teleop teleop.launch.py
    # 标定模式：只打印原始按键事件与摇杆值，不发布任何命令
    ros2 launch greeting_teleop teleop.launch.py monitor:=true
    # 同时启用 /sbus_data(Joy) 的 12 轴映射（按需）
    ros2 launch greeting_teleop teleop.launch.py joy_topic:=/sbus_data

前置条件：
    · 平台遥控器进程在跑，/sbus_data/event 有数据：
          ros2 topic hz /sbus_data/event
    · 命令消费方按需启动：动作快捷键需 greeting_body 的动作服务端；
      全流程命令（如 G组+A/B/C/D=goto:N）需 flow_command_bridge + greeting_orchestrator
      （可用 greeting_bringup.launch.py 一键启动）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            "event_topic", default_value="/sbus_data/event",
            description="SBUS 按键事件话题（bodyctrl_msgs/msg/SbusData）",
        ),
        DeclareLaunchArgument(
            "joy_topic", default_value="",
            description="可选 Joy 轴话题；留空不订阅，填 /sbus_data 可启用 12 轴映射",
        ),
        DeclareLaunchArgument(
            "command_topic", default_value="/greeting/panel_command",
            description="迎宾命令话题（std_msgs/String，契约 §2 T2）",
        ),
        DeclareLaunchArgument(
            "map_file", default_value="",
            description="按键映射表 YAML；留空则用 share/greeting_teleop/config/teleop_map.yaml",
        ),
        DeclareLaunchArgument(
            "deadband", default_value="0.5",
            description="轴阈值（|value|>deadband 判为拨动）",
        ),
        DeclareLaunchArgument(
            "operator_id", default_value="rc",
            description="操作人标识（追溯用）",
        ),
        DeclareLaunchArgument(
            "monitor", default_value="false",
            description="true=标定模式：只打印按键事件/摇杆值，不发布命令",
        ),
        DeclareLaunchArgument(
            "monitor_period_s", default_value="2.0",
            description="标定模式打印周期(s)",
        ),
    ]

    joy_mapper = Node(
        package="greeting_teleop",
        executable="joy_mapper",
        name="greeting_joy_mapper",
        output="screen",
        parameters=[{
            "event_topic": LaunchConfiguration("event_topic"),
            "joy_topic": LaunchConfiguration("joy_topic"),
            "command_topic": LaunchConfiguration("command_topic"),
            "map_file": LaunchConfiguration("map_file"),
            "deadband": LaunchConfiguration("deadband"),
            "operator_id": LaunchConfiguration("operator_id"),
            "monitor": LaunchConfiguration("monitor"),
            "monitor_period_s": LaunchConfiguration("monitor_period_s"),
        }],
    )

    return LaunchDescription(args + [joy_mapper])
