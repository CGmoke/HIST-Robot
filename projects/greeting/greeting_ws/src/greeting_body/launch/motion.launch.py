#!/usr/bin/env python3
"""motion.launch.py —— 启动【实机】礼仪动作 PlayMotion 服务端（greeting_body）。

用法：
    ros2 launch greeting_body motion.launch.py
    ros2 launch greeting_body motion.launch.py speed_limit:=0.5 publish_rate_hz:=20.0

前置条件（实机）：
    1) 已 source 实机环境： source /home/nvidia/xos/setup.bash
    2) 整机控制已使能、/robot_state 有反馈、/arm/cmd 等通道有订阅者；
       可用 ros2 topic info /arm/cmd 确认订阅者数量 ≥1，否则命令会被静默丢弃。

⚠️ 实机安全：首次调试务必先在低速（speed_limit≤0.5）、空载、有人守护、
   急停可达的条件下测试；确认无异常后再恢复正常速度。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        # 空字符串 -> 节点自动使用 share/greeting_body/config/ 下的默认文件
        DeclareLaunchArgument(
            "motions_file", default_value="",
            description="动作表 YAML；留空则用 share/greeting_body/config/motions.yaml",
        ),
        DeclareLaunchArgument(
            "joint_map_file", default_value="",
            description="关节映射 YAML；留空则用 share/greeting_body/config/joint_map.yaml",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz", default_value="50.0",
            description="关节帧下发频率(Hz)，实机命令需 ≥10Hz；50Hz 可降低滞后型抖动",
        ),
        DeclareLaunchArgument(
            "speed_limit", default_value="0.7",
            description="速度上限（硬约束 ≤0.7）；实机调试建议 0.3~0.5",
        ),
        DeclareLaunchArgument(
            "warmup_s", default_value="1.5",
            description="起步 warmup 时长(s)：保持当前位姿、低电流低 Kp，软化 SDK 模式移交",
        ),
        DeclareLaunchArgument(
            "warmup_speed", default_value="0.3",
            description="warmup 期间位置模式(头/腰)的柔和速度上限(rad/s)",
        ),
        DeclareLaunchArgument(
            "warmup_current_start", default_value="1.0",
            description="warmup 首帧电流上限比例(0~1)，越小移交越柔和",
        ),
        DeclareLaunchArgument(
            "warmup_kp_start", default_value="0.6",
            description="warmup 首帧臂力位混合 Kp 比例(0~1)，线性 ramp 到 1.0",
        ),
        DeclareLaunchArgument(
            "hold_s", default_value="0.5",
            description="动作收尾回到中立位后的保持时长(s)",
        ),
        DeclareLaunchArgument(
            "arm_control_mode", default_value="0",
            description="手臂控制模式：0=位置模式(天轶2.5 官方做法，默认，SDK 不传增益) "
                        "1=力位混合(仅回退实验用，本机型实测会持续高频啸叫)",
        ),
        DeclareLaunchArgument(
            "max_follow_err_rad", default_value="0.10",
            description="臂关节期望相对实测的最大超前量(rad)，防止 Kp 顶死限位粘滑颤振",
        ),
        DeclareLaunchArgument(
            "arm_kp_scale", default_value="1.0",
            description="【仅 arm_control_mode:=1 生效】臂力位混合 Kp 缩放",
        ),
        DeclareLaunchArgument(
            "arm_kd_scale", default_value="1.0",
            description="【仅 arm_control_mode:=1 生效】臂力位混合 Kd 缩放",
        ),
        DeclareLaunchArgument(
            "arm_feedforward_scale", default_value="1.0",
            description="前馈速度缩放：设 0 可定位是否为前馈驱动；也可用 0.5 软化",
        ),
        DeclareLaunchArgument(
            "required_groups", default_value="['arm']",
            description="硬预检通道分组：无订阅者则直接拒绝执行",
        ),
        DeclareLaunchArgument(
            "arm_current", default_value="5.0",
            description="手臂电流上限(A)；对齐 body_control 的 DEFAULT_CURRENT=5，调试可再降",
        ),
        DeclareLaunchArgument(
            "head_speed", default_value="0.5",
            description="【位置模式/静止(保持)段】头部速度上限(rad/s)；运动段 spd 由轨迹瞬时速度决定",
        ),
        DeclareLaunchArgument(
            "waist_speed", default_value="0.5",
            description="【位置模式/静止(保持)段】腰部速度上限(rad/s)；运动段 spd 由轨迹瞬时速度决定",
        ),
        DeclareLaunchArgument(
            "head_current", default_value="5.0",
            description="头部电流上限(A)；对齐 body_control 的 DEFAULT_CURRENT=5，调试可再降",
        ),
        DeclareLaunchArgument(
            "waist_current", default_value="5.0",
            description="腰部电流上限(A)；对齐 body_control 的 DEFAULT_CURRENT=5，调试可再降",
        ),
        DeclareLaunchArgument(
            "feedforward_speed_margin", default_value="1.2",
            description="【位置模式/运动段】spd = |轨迹瞬时速度| × margin 的余量，只需略 >1；"
                        "调大更跟手但易 rush-and-wait，调小更柔和",
        ),
        DeclareLaunchArgument(
            "min_feedforward_speed", default_value="0.05",
            description="【位置模式】spd 绝对下限(rad/s)；必须远小于运动段真实速度",
        ),
        DeclareLaunchArgument(
            "idle_scan_enabled", default_value="true",
            description="无动作时是否自动循环待机扫视 idle_scan_loop（false=整体关闭）",
        ),
        DeclareLaunchArgument(
            "idle_scan_motion", default_value="idle_scan_loop",
            description="待机扫视动作名（须为 motions.yaml 中的无缝循环动作）",
        ),
        DeclareLaunchArgument(
            "idle_scan_delay_s", default_value="3.0",
            description="最后一个动作结束后静止多久开始扫视(s)",
        ),
        DeclareLaunchArgument(
            "idle_scan_speed_scale", default_value="0.5",
            description="待机扫视速度（会被钳到 ≤ speed_limit）",
        ),
        DeclareLaunchArgument(
            "idle_scan_pause_on_h_right", default_value="true",
            description="H 停在右档（平台简单动作模式直发关节）时暂停扫视",
        ),
        DeclareLaunchArgument(
            "idle_scan_pause_on_voice", default_value="true",
            description="F 上拨 + 长按 A 开启语音功能时暂停扫视并回 home 中立位",
        ),
        DeclareLaunchArgument(
            "voice_long_press_s", default_value="1.0",
            description="A 长按判定阈值(s)，须与平台 bridge_config long_press_thresholds.a 对齐",
        ),
    ]

    motion_server = Node(
        package="greeting_body",
        executable="motion_server",
        name="greeting_body_motion_server",
        output="screen",
        parameters=[{
            "motions_file": LaunchConfiguration("motions_file"),
            "joint_map_file": LaunchConfiguration("joint_map_file"),
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            "speed_limit": LaunchConfiguration("speed_limit"),
            "warmup_s": LaunchConfiguration("warmup_s"),
            "warmup_speed": LaunchConfiguration("warmup_speed"),
            "warmup_current_start": LaunchConfiguration("warmup_current_start"),
            "warmup_kp_start": LaunchConfiguration("warmup_kp_start"),
            "hold_s": LaunchConfiguration("hold_s"),
            "arm_control_mode": LaunchConfiguration("arm_control_mode"),
            "max_follow_err_rad": LaunchConfiguration("max_follow_err_rad"),
            "arm_kp_scale": LaunchConfiguration("arm_kp_scale"),
            "arm_kd_scale": LaunchConfiguration("arm_kd_scale"),
            "arm_feedforward_scale": LaunchConfiguration("arm_feedforward_scale"),
            "required_groups": LaunchConfiguration("required_groups"),
            "arm_current": LaunchConfiguration("arm_current"),
            "head_speed": LaunchConfiguration("head_speed"),
            "waist_speed": LaunchConfiguration("waist_speed"),
            "head_current": LaunchConfiguration("head_current"),
            "waist_current": LaunchConfiguration("waist_current"),
            "feedforward_speed_margin": LaunchConfiguration("feedforward_speed_margin"),
            "min_feedforward_speed": LaunchConfiguration("min_feedforward_speed"),
            "idle_scan_enabled": LaunchConfiguration("idle_scan_enabled"),
            "idle_scan_motion": LaunchConfiguration("idle_scan_motion"),
            "idle_scan_delay_s": LaunchConfiguration("idle_scan_delay_s"),
            "idle_scan_speed_scale": LaunchConfiguration("idle_scan_speed_scale"),
            "idle_scan_pause_on_h_right": LaunchConfiguration("idle_scan_pause_on_h_right"),
            "idle_scan_pause_on_voice": LaunchConfiguration("idle_scan_pause_on_voice"),
            "voice_long_press_s": LaunchConfiguration("voice_long_press_s"),
        }],
    )

    return LaunchDescription(args + [motion_server])