#!/usr/bin/env python3
"""greeting_bringup.launch.py —— 迎宾一键启动（手柄遥控直达动作 + 触发大会流程）。

组成：
  1) greeting_body/motion.launch.py      实机礼仪动作 PlayMotion 服务端（/greeting/play_motion）
  2) greeting_teleop/teleop.launch.py    遥控器 SBUS 按键网关（/sbus_data/event -> /greeting/panel_command）
  3) greeting_teleop panel_command_bridge 动作直连桥（/greeting/panel_command -> /greeting/play_motion）
  4) greeting_teleop flow_command_bridge  流程命令桥（/greeting/panel_command -> /greeting/control_cmd）
  5) greeting_voice speak_action_server   /greeting/speak 动作服务端（编排层语音播报所需）
  6) greeting_voice simple_action_voice_node 平台「简单动作模式」(H右 + 连按B×N + 短按A)
                                          的语音播报（与平台动作回放并行，序号一一对应）
  7) greeting_orchestrator/orchestrator.launch.py 编排层状态机（讲稿 cue：动作与语音同步）

注：只起 greeting_voice 的 speak_action_server 与 simple_action_voice_node，
    【不】起 voice_greet_tts_node —— 后者同样监听 /sbus_data/event 的 G+A/B/C/D
    （默认播报"预留N"），会与 joy_mapper 的 goto:<段号> 撞车、并盖过讲稿语音。
    simple_action_voice_node 监听的是 H 档位，与 joy_mapper（E 档位）不冲突。

数据流：
  /sbus_data/event ─joy_mapper─> /greeting/panel_command ─panel_command_bridge─> /greeting/play_motion
                                                                  │                      │
                                                                  │         greeting_body motion_server
                                                                  │                      ↓ /arm|/head|/waist/cmd
                                                                  └─flow_command_bridge─> /greeting/control_cmd
                                                                                                ↓
                                                                       greeting_orchestrator 讲稿（cue：动作 + 语音）

手柄按键（组合键：先拨前提开关，再按 A/B/C/D）：
  H 右   = stop  （停止并回到 IDLE，安全键）
  E上+A/B/C = 单动作 salute_bow / wave_official / photo_pose
  G左+A/B/C/D = 第 1/2/3/4 段     G右+A/B/C = 第 5/6/7 段
              （分段点播 goto:<段号>：只播该段，播完回 IDLE，不续播）
  注：整段连播 start（一键走完 7 段，动作比语音提前 1s/3s，由讲稿 lead_s 决定）
      当前未绑定按键 —— 原 D 已改作分段键（G左 + D = 第 4 段），
      如需保留可在 teleop_map.yaml 的 keys: 另择一键置为 start（如 f_up: start）。

用法：
    ros2 launch greeting_teleop greeting_bringup.launch.py
    ros2 launch greeting_teleop greeting_bringup.launch.py speed_limit:=0.5
    ros2 launch greeting_teleop greeting_bringup.launch.py monitor:=true              # 标定模式
    ros2 launch greeting_teleop greeting_bringup.launch.py bridge_speed_scale:=0.4
    ros2 launch greeting_teleop greeting_bringup.launch.py with_orchestrator:=false    # 只用动作快捷键
    ros2 launch greeting_teleop greeting_bringup.launch.py with_voice:=false          # 不起语音服务端
    ros2 launch greeting_teleop greeting_bringup.launch.py with_simple_action_voice:=false  # 不起 H右+B×N+A 语音
    ros2 launch greeting_teleop greeting_bringup.launch.py with_voice:=false with_orchestrator:=false

前置条件（实机）：
    · 已 source /home/nvidia/xos/setup.bash（提供 bodyctrl_msgs / ros2_bridge_msgs）
    · 平台已上电，/sbus_data/event 有数据；按手柄 A 键完成自检后 robot_control 才使能，
      此时 /arm/cmd 才有订阅者（否则动作会被硬预检拒绝）。

⚠️ 实机安全：首次调试务必低速（speed_limit≤0.5）、空载、有人守护、急停可达。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    body_launch = os.path.join(
        get_package_share_directory("greeting_body"), "launch", "motion.launch.py"
    )
    teleop_launch = os.path.join(
        get_package_share_directory("greeting_teleop"), "launch", "teleop.launch.py"
    )
    orchestrator_launch = os.path.join(
        get_package_share_directory("greeting_orchestrator"), "launch", "orchestrator.launch.py"
    )

    args = [
        # ---- 动作服务端参数（透传给 greeting_body/motion.launch.py）----
        DeclareLaunchArgument(
            "speed_limit", default_value="0.7",
            description="动作速度上限（硬约束 ≤0.7）；实机调试建议 0.3~0.5",
        ),
        DeclareLaunchArgument(
            "arm_current", default_value="5.0",
            description="手臂电流上限(A)",
        ),
        DeclareLaunchArgument(
            "required_groups", default_value="['arm']",
            description="硬预检通道分组：无订阅者则拒绝执行（自检完成前 /arm/cmd 无订阅者）",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz", default_value="50.0",
            description="关节帧下发频率(Hz)",
        ),
        DeclareLaunchArgument(
            "motions_file", default_value="",
            description="动作表 YAML；留空用 share/greeting_body/config/motions.yaml",
        ),
        # ---- 手柄网关参数（透传给 greeting_teleop/teleop.launch.py）----
        DeclareLaunchArgument(
            "joy_topic", default_value="",
            description="可选 Joy 轴话题；留空不订阅，填 /sbus_data 启用 12 轴映射",
        ),
        DeclareLaunchArgument(
            "map_file", default_value="",
            description="按键映射表 YAML；留空用 share/greeting_teleop/config/teleop_map.yaml",
        ),
        DeclareLaunchArgument(
            "monitor", default_value="false",
            description="true=标定模式：只打印按键事件，不发布命令",
        ),
        # ---- 直连桥参数 ----
        DeclareLaunchArgument(
            "bridge_speed_scale", default_value="0.5",
            description="手柄触发动作时的速度缩放（桥内钳到 ≤ bridge_speed_scale_max）",
        ),
        DeclareLaunchArgument(
            "bridge_speed_scale_max", default_value="0.7",
            description="桥内速度硬上限，强制 ≤0.7",
        ),
        DeclareLaunchArgument(
            "bridge_preempt", default_value="false",
            description="true=新按键动作抢占正在执行的动作（默认忽略，避免轨迹叠加）",
        ),
        DeclareLaunchArgument(
            "bridge_allowed_motions", default_value="",
            description="允许手柄触发的动作名白名单（逗号分隔）；留空=不校验",
        ),
        # ---- 流程命令桥参数 ----
        DeclareLaunchArgument(
            "flow_operator_id", default_value="rc",
            description="流程命令桥转发时写入 ConsoleCommand.operator_id（追溯用）",
        ),
        # ---- 编排层参数（触发大会流程所必需）----
        DeclareLaunchArgument(
            "with_orchestrator", default_value="true",
            description="true=同时启动编排层，G组+A/B/C/D 可分段点播（goto:<段号>）",
        ),
        DeclareLaunchArgument(
            "approved_script_version", default_value="1.1",
            description="已审核讲稿版本号（须与 greeting_script.yaml 的 version 一致）",
        ),
        # ---- 语音：/greeting/speak 动作服务端（编排层每段语音步骤的消费方）----
        DeclareLaunchArgument(
            "with_voice", default_value="true",
            description="true=启动 /greeting/speak 动作服务端（编排层语音播报所需）"),
        # ---- 语音：平台简单动作模式（H右 + 连按B×N + 短按A）的语音播报 ----
        DeclareLaunchArgument(
            "with_simple_action_voice", default_value="true",
            description="true=启动 simple_action_voice_node（H右+B×N+A 动作的语音）"),
        DeclareLaunchArgument(
            "voice_params_file",
            default_value=os.path.join(
                get_package_share_directory("greeting_voice"), "config", "voice.yaml"),
            description="simple_action_voice_node 参数文件（文本/连按窗口）"),
    ]

    motion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(body_launch),
        launch_arguments={
            "speed_limit": LaunchConfiguration("speed_limit"),
            "arm_current": LaunchConfiguration("arm_current"),
            "required_groups": LaunchConfiguration("required_groups"),
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            "motions_file": LaunchConfiguration("motions_file"),
        }.items(),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch),
        launch_arguments={
            "joy_topic": LaunchConfiguration("joy_topic"),
            "map_file": LaunchConfiguration("map_file"),
            "monitor": LaunchConfiguration("monitor"),
        }.items(),
    )

    bridge = Node(
        package="greeting_teleop",
        executable="panel_command_bridge",
        name="greeting_panel_bridge",
        output="screen",
        parameters=[{
            "command_topic": "/greeting/panel_command",
            "action_name": "/greeting/play_motion",
            "speed_scale": LaunchConfiguration("bridge_speed_scale"),
            "speed_scale_max": LaunchConfiguration("bridge_speed_scale_max"),
            "preempt": LaunchConfiguration("bridge_preempt"),
            "allowed_motions": LaunchConfiguration("bridge_allowed_motions"),
        }],
    )

    # 流程命令桥：把全流程命令（start/goto/stop…）转成编排层的 ConsoleCommand
    flow_bridge = Node(
        package="greeting_teleop",
        executable="flow_command_bridge",
        name="greeting_flow_bridge",
        output="screen",
        parameters=[{
            "command_topic": "/greeting/panel_command",
            "control_topic": "/greeting/control_cmd",
            "operator_id": LaunchConfiguration("flow_operator_id"),
        }],
    )

    # 语音：/greeting/speak 动作服务端（编排层每段的语音步骤消费方，
    # 内部转调平台 TTS 服务 /intelligent_interaction/tts/play）。
    voice = Node(
        package="greeting_voice",
        executable="speak_action_server",
        name="greeting_speak_server",
        output="screen",
        condition=IfCondition(LaunchConfiguration("with_voice")),
    )

    # 语音：平台简单动作模式（H 右拨 + 连按 B×N + 短按 A）的播报。
    # 序号与平台 joystick_bridge_node.SIMPLE_ACTION_FILES 一一对应：
    #   1=挥手(0706_hello) 2=击掌(Give_me_fire) 3=碰拳(Fist) 4=敬礼(Salute)
    simple_action_voice = Node(
        package="greeting_voice",
        executable="simple_action_voice_node",
        name="simple_action_voice_node",
        output="screen",
        parameters=[LaunchConfiguration("voice_params_file")],
        condition=IfCondition(LaunchConfiguration("with_simple_action_voice")),
    )

    # 编排层：讲稿 cue（动作 + 语音同步）。实机用系统时间，故 use_sim_time=false。
    orchestrator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orchestrator_launch),
        launch_arguments={
            "use_sim_time": "false",
            "approved_script_version": LaunchConfiguration("approved_script_version"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_orchestrator")),
    )

    return LaunchDescription(
        args + [motion, teleop, bridge, flow_bridge, voice, simple_action_voice, orchestrator])