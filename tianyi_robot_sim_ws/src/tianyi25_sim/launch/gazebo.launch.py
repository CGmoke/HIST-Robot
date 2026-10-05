#!/usr/bin/env python3
"""天轶 2.5 —— Gazebo Harmonic (gz-sim 8) 物理仿真（本体底盘四轮 4WD + 2D 雷达）。

链路：
  gz sim (greeting_world.sdf)  物理引擎 + Sensors 系统（雷达渲染）
  robot_state_publisher        发布 /robot_description 与 TF（base_footprint 以下）
  ros_gz_sim create            把模型生成进 Gazebo（从 /robot_description 取描述）
  ros_gz_bridge                /clock、/joint_states、/odom、/scan、cmd_vel、关节轨迹
  gz_joint_sweep               周期下发上半身正弦扫掠轨迹
  gz_odom_tf                   把 /odom 转成 odom→base_footprint 的 TF

上半身关节驱动用 Gazebo 内置 JointTrajectoryController，不需要 ros2_control。
底盘驱动用 Gazebo 内置 DiffDrive，不需要 ros2_control。

导航（SLAM + Nav2）另行启动，见 tianyi_nav.launch.py。

用法:
    ros2 launch tianyi25_sim gazebo.launch.py
    ros2 launch tianyi25_sim gazebo.launch.py world:=empty.sdf spawn_z:=0.012
    ros2 launch tianyi25_sim gazebo.launch.py sweep_config:=wave_greet.yaml

sweep_config 指定 config/ 下的扫掠档（只写文件名），默认 gz_joint_sweep.yaml。
该档是【整套仿真整定的入口】：joints / frequency、arm_control（PID 与逐关节力矩表）、
posture_lock（姿态锁止弹簧）、model_name / world_name 全部从它读取 ——
xacro（经同名 xacro arg）与 launch 都按它取参数，换档即整体切换，无需改代码。
例如 wave_greet.yaml = 保持固定姿态、只摆右肩挥手。
"""
from __future__ import annotations

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _create_nodes(context):
    """按 sweep_config 指定的档建节点。

    必须放在 OpaqueFunction 里：sweep_config 是 LaunchConfiguration，只有 launch 时
    才拿得到实际值，而 model_name 要拼进 traj_topic / cmd_vel 这些字符串。
    """
    sim_share = Path(get_package_share_directory("tianyi25_sim"))
    sweep_config = LaunchConfiguration("sweep_config").perform(context)
    cfg = yaml.safe_load(
        (sim_share / "config" / sweep_config).read_text(encoding="utf-8")
    )
    model_name = cfg["model_name"]
    world_name = cfg["world_name"]

    xacro_file = sim_share / "urdf" / "tianyi25_gazebo.urdf.xacro"

    # 仿真侧唯一的话题名，bridge 与扫掠节点共用
    traj_topic = f"/model/{model_name}/joint_trajectory"
    # DiffDrive 插件无 topic 参数，cmd_vel 硬编码为 /model/<模型名>/cmd_vel；
    # 用本节点的 ROS 侧 remapping 把它映射成标准 /cmd_vel（gz 侧名字不受影响）。
    gz_cmd_vel_topic = f"/model/{model_name}/cmd_vel"

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(
                        [
                            "xacro ",
                            str(xacro_file),
                            # 把当前档一并传给 xacro，让它读同一份 yaml 的
                            # arm_control / posture_lock / joints
                            " sweep_config:=",
                            LaunchConfiguration("sweep_config"),
                        ]
                    ),
                    value_type=str,
                ),
                "use_sim_time": True,
            }
        ],
    )

    spawn_model = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_tianyi25",
        output="screen",
        arguments=[
            "-world", world_name,
            "-topic", "robot_description",
            "-name", model_name,
            "-z", LaunchConfiguration("spawn_z"),
        ],
    )

    # 方向符号: '[' = gz -> ROS, ']' = ROS -> gz
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            f"{traj_topic}@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            f"{gz_cmd_vel_topic}@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
        remappings=[(gz_cmd_vel_topic, "/cmd_vel")],
    )

    joint_sweep = Node(
        package="tianyi25_sim",
        executable="gz_joint_sweep",
        name="gz_joint_sweep",
        output="screen",
        parameters=[
            {
                # sweep_config 只写文件名，实际路径 = share/<pkg>/config/<名>；
                # 与 xacro 读的是同一份文件
                "config_file": PathJoinSubstitution(
                    [
                        FindPackageShare("tianyi25_sim"),
                        "config",
                        LaunchConfiguration("sweep_config"),
                    ]
                ),
                "trajectory_topic": traj_topic,
                # 与 Gazebo /clock 对齐：相位与轨迹时间都用仿真时间，
                # 否则实时率不为 1 时扫掠频率会与设定值不符
                "use_sim_time": True,
            }
        ],
    )

    odom_tf = Node(
        package="tianyi25_sim",
        executable="gz_odom_tf",
        name="gz_odom_tf",
        output="screen",
        parameters=[{"odom_topic": "/odom", "use_sim_time": True}],
    )

    return [robot_state_publisher, spawn_model, bridge, joint_sweep, odom_tf]


def generate_launch_description() -> LaunchDescription:
    sim_share = Path(get_package_share_directory("tianyi25_sim"))
    default_world = str(sim_share / "worlds" / "greeting_world.sdf")

    world = LaunchConfiguration("world")

    # URDF 里的 package://tianyi25_urdf/meshes/*.STL 会被 sdformat 改写成
    # model://tianyi25_urdf/...，而 gz 只认 GZ_SIM_RESOURCE_PATH。把 install/share
    # 加进去（其下正好有 tianyi25_urdf/ 目录），否则网格与 collision 全部加载失败
    # —— 表现为模型可见但直接穿透地面。
    urdf_share_parent = str(Path(get_package_share_directory("tianyi25_urdf")).parent)
    gz_resource_path = os.pathsep.join(
        p for p in [urdf_share_parent, os.environ.get("GZ_SIM_RESOURCE_PATH", "")] if p
    )
    set_gz_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", gz_resource_path
    )

    # -r: 启动即运行（不暂停）；-v 3: 常规日志
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": ["-r -v 3 ", world]}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Gazebo 世界文件（默认本包的房间：地面+墙+家具+传感器系统）",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.012",
                # 整机直接坐在本体 base 自带的四个真实轮子上：真实轮底相对
                # base_footprint = axle_z(0.06) - wheel_radius(0.072) = -0.012。
                # 故 0.012 让四轮落地；网格最低点 -0.0105 比轮底高 1.5 mm，
                # base 本体（含网格内焊死的轮形）不会触地。
                description="模型生成高度 (m)。默认 0.012 = 本体四轮轮底深度，正好落地",
            ),
            DeclareLaunchArgument(
                "sweep_config",
                default_value="gz_joint_sweep.yaml",
                description=(
                    "config/ 下的整定档文件名（只写文件名）。xacro 与扫掠节点都读它，"
                    "档内自包含 joints/频率、arm_control、posture_lock、model/world。"
                    "默认 gz_joint_sweep.yaml（全身自然摆动）；"
                    "wave_greet.yaml = 保持固定姿态、只摆右肩挥手"
                ),
            ),
            set_gz_resource_path,
            gz_sim,
            # sweep_config 是 LaunchConfiguration，必须等 launch 时才有值，
            # 故把依赖它的节点放进 OpaqueFunction 里构造
            OpaqueFunction(function=_create_nodes),
        ]
    )
