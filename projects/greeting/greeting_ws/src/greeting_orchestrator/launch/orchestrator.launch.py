#!/usr/bin/env python3
"""启动迎宾编排层（唯一指挥者 / 唯一状态机）。

用法:
    ros2 launch greeting_orchestrator orchestrator.launch.py
    ros2 launch greeting_orchestrator orchestrator.launch.py approved_script_version:=1.1

启动后：
    ros2 topic echo /greeting/state                       # 看状态机当前状态
    ros2 topic pub --once /greeting/control_cmd greeting_interfaces/msg/ConsoleCommand \
        "{cmd: start}"                                    # 开始接待
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")
    approved = LaunchConfiguration("approved_script_version")
    nav_timeout = LaunchConfiguration("default_nav_timeout")
    motion_lead = LaunchConfiguration("default_motion_lead_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="使用 Gazebo 的 /clock 仿真时间",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="",
                description="讲稿 YAML；留空则用包内 config/greeting_script.yaml",
            ),
            DeclareLaunchArgument(
                "approved_script_version",
                default_value="1.1",
                description="已审核的讲稿版本号，reload_script 时校验（契约 §3）",
            ),
            DeclareLaunchArgument(
                "default_nav_timeout",
                default_value="60.0",
                description="讲稿未写 timeout_s 时的导航超时",
            ),
            DeclareLaunchArgument(
                "default_motion_lead_s",
                default_value="3.0",
                description="cue 步骤未写 lead_s 时，动作相对语音的默认提前量(s)",
            ),
            Node(
                package="greeting_orchestrator",
                executable="orchestrator_node",
                name="greeting_orchestrator",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {"config_file": config_file},
                    # 必须显式声明为 str：否则 launch 会把 "1.0" 推断成 DOUBLE，
                    # 与节点里 declare_parameter 的字符串默认值冲突而启动失败
                    {
                        "approved_script_version": ParameterValue(
                            approved, value_type=str
                        )
                    },
                    {
                        "default_nav_timeout": ParameterValue(
                            nav_timeout, value_type=float
                        )
                    },
                    {
                        "default_motion_lead_s": ParameterValue(
                            motion_lead, value_type=float
                        )
                    },
                ],
            ),
        ]
    )