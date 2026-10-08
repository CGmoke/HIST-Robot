"""greeting_voice 启动入口：语音迎宾播报节点。

三条语音链路：
  1) voice_greet_tts_node  —— 遥控器 G 左/右拨 + A/B/C/D 键【人工】触发播报；
  2) simple_action_voice_node —— 平台「简单动作模式」H 右拨 + 连按 B×N + 短按 A
                                【人工】触发播报（与平台动作回放并行，序号一一对应）；
  3) speak_action_server   —— /greeting/speak 动作服务端，供【编排层自动】驱动播报
                              （支撑"动作比语音早 1s/3s 触发"的时序编排）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('greeting_voice')
    default_params = os.path.join(pkg_share, 'config', 'voice.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='语音节点参数文件（含 voice_greet_tts_node / simple_action_voice_node 两段）'),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='true 时只打日志、不调用 TTS 服务（无硬件联调用）'),
        DeclareLaunchArgument(
            'speak_server',
            default_value='true',
            description='是否随之启动 /greeting/speak 动作服务端（编排层驱动播报用）'),
        DeclareLaunchArgument(
            'simple_action_voice',
            default_value='true',
            description='是否随之启动 H右+B×N+A 简单动作语音节点'),
        Node(
            package='greeting_voice',
            executable='voice_greet_tts_node',
            name='voice_greet_tts_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'dry_run': LaunchConfiguration('dry_run')},
            ],
        ),
        Node(
            package='greeting_voice',
            executable='simple_action_voice_node',
            name='simple_action_voice_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'dry_run': LaunchConfiguration('dry_run')},
            ],
            condition=IfCondition(LaunchConfiguration('simple_action_voice')),
        ),
        Node(
            package='greeting_voice',
            executable='speak_action_server',
            name='greeting_speak_server',
            output='screen',
            condition=IfCondition(LaunchConfiguration('speak_server')),
        ),
    ])
