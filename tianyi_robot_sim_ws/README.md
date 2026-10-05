# 天轶 2.5 机器人仿真工作空间（tianyi_robot_sim_ws）

天轶 2.5 人形机器人（四轮底盘 + 双腿柱 + 双臂 + 三自由度头部）的 ROS 2 仿真工作空间，
基于 **ROS 2 Jazzy** + **Gazebo Harmonic（gz-sim 8）**。

工作空间只包含两个包，职责分层清晰：

| 包 | 类型 | 职责 |
| --- | --- | --- |
| `tianyi25_urdf` | 纯资源包 | URDF 模型、24 个 STL 网格、RViz 配置、本地显示 launch |
| `tianyi25_sim` | 仿真包 | Gazebo 世界、模型物理扩展、控制器配置、ROS↔Gazebo 桥接、SLAM + Nav2、扫掠/里程计节点 |

依赖方向是单向的：`tianyi25_sim` 通过 `xacro:include` 复用 `tianyi25_urdf` 的纯 URDF 与网格，
不修改上游任何内容。因此可以单独使用 `tianyi25_urdf` 做纯模型显示，不需要 Gazebo。

---

## 1. 目录结构

```
tianyi_robot_sim_ws/
├── src/
│   ├── tianyi25_urdf/                 # 上游资源包（模型 + 网格）
│   │   ├── urdf/                      # 5 个 URDF（主模型 / wholebody / dual / 单臂 ×2）
│   │   ├── meshes/                    # 24 个 STL 网格
│   │   ├── rviz/urdf.rviz             # 显示用 RViz 配置
│   │   ├── launch/display.launch.py   # RSP + 关节滑块 + RViz
│   │   └── config/joint_names_tianyi25_urdf.yaml
│   └── tianyi25_sim/                  # 仿真包
│       ├── urdf/tianyi25_gazebo.urdf.xacro   # include 纯 URDF，追加四轮/雷达/gz 插件
│       ├── worlds/greeting_world.sdf         # 12×12 m 迎宾房间 + 家具 + Sensors 系统
│       ├── config/                            # 扫掠/动作/点位/SLAM/Nav2 参数
│       ├── launch/                            # 3 个仿真入口
│       ├── scripts/                           # 3 个 rclpy 节点
│       └── models/
├── build/  install/  log/             # colcon 产物（不入版本库）
└── .gitkeep
```

---

## 2. 环境要求

- Ubuntu 24.04 + ROS 2 Jazzy（`/opt/ros/jazzy/setup.bash` 已 source）
- Gazebo Harmonic（gz-sim 8），随下列 vendor 包提供，**无需** ros2_control / gz_ros2_control
- Python 3 与 `python3-yaml`（launch 与扫掠节点读 YAML 用）

```bash
# 必需
sudo apt install -y \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-robot-state-publisher ros-jazzy-xacro \
  ros-jazzy-rviz2 python3-yaml

# 可选：纯模型显示的关节滑块
sudo apt install -y ros-jazzy-joint-state-publisher-gui

# 可选：SLAM + Nav2（tianyi_nav.launch.py 需要）
sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup \
                    ros-jazzy-nav2-msgs ros-jazzy-nav2-rviz-plugins
```

> 仿真里的关节驱动与底盘驱动都用 Gazebo 内置插件
> （`JointTrajectoryController` / `DiffDrive`），**不依赖 ros2_control**。

---

## 3. 构建

```bash
cd tianyi_robot_sim_ws
colcon build --packages-select tianyi25_urdf tianyi25_sim --symlink-install
source install/setup.bash          # 每个新终端都要 source
```

`tianyi25_urdf` 必须一起构建：`tianyi25_sim` 的 xacro 通过
`package://tianyi25_urdf/meshes/*.STL` 引用网格，缺了会导致模型加载失败。

---

## 4. 运行

三个仿真入口都在 `tianyi25_sim` 包里。

### 4.1 纯模型显示（RViz，无 Gazebo）

```bash
# 全功能：关节滑块 + RViz
ros2 launch tianyi25_urdf display.launch.py

# 仅 RViz（关节静止）
ros2 launch tianyi25_urdf display.launch.py use_gui:=false
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `use_gui` | `true` | 启动 `joint_state_publisher_gui`（需装 joint-state-publisher-gui） |
| `use_rviz` | `true` | 启动 RViz2 |

### 4.2 RViz 关节扫掠演示（无 Gazebo）

`tianyi25_urdf` 的显示 + `joint_sweep_demo` 节点（纯 rclpy 发布正弦 `/joint_states`）。

```bash
ros2 launch tianyi25_sim display_motion.launch.py
ros2 launch tianyi25_sim display_motion.launch.py frequency:=0.4 use_rviz:=false
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `frequency` | `0.15` | 关节正弦扫掠频率 (Hz) |
| `use_rviz` | `true` | 是否启动 RViz2 |

### 4.3 Gazebo 物理仿真（核心）

```bash
ros2 launch tianyi25_sim gazebo.launch.py
ros2 launch tianyi25_sim gazebo.launch.py world:=empty.sdf spawn_z:=0.012
ros2 launch tianyi25_sim gazebo.launch.py sweep_config:=wave_greet.yaml
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `world` | `worlds/greeting_world.sdf` | 世界文件 |
| `spawn_z` | `0.012` | 生成高度 (m)，默认值等于四轮轮底深度，正好落地 |
| `sweep_config` | `gz_joint_sweep.yaml` | `config/` 下的整定档文件名（只写文件名）。xacro 与扫掠节点都按它读取（档内自包含 joints/频率、arm_control、posture_lock、model/world）。默认全身自然摆动；`wave_greet.yaml` = 保持一个固定姿态、只摆右肩挥手 |

拉起的节点：

| 节点 | 作用 |
| --- | --- |
| `gz sim` | 物理引擎 + Sensors 系统（雷达渲染） |
| `robot_state_publisher` | 发布 `/robot_description` 与 `base_footprint` 以下的 TF |
| `ros_gz_sim create` | 把模型生成到 Gazebo（从 `/robot_description` 取描述） |
| `ros_gz_bridge` | 话题桥接（见 §5） |
| `gz_joint_sweep` | 周期下发上半身正弦扫掠轨迹 |
| `gz_odom_tf` | 把 `/odom` 转成 `odom → base_footprint` 的 TF |

> 注意：不要用 `empty.sdf` 跑 SLAM —— 它没有 Sensors 系统，雷达不出数据，且空世界无特征会漂移。

### 4.4 SLAM Toolbox 在线建图 + Nav2 导航

需要已装 Nav2 / slam_toolbox。**建议两个终端分别运行**：

```bash
# 终端 1：仿真
ros2 launch tianyi25_sim gazebo.launch.py

# 终端 2：建图 + 导航
ros2 launch tianyi25_sim tianyi_nav.launch.py
ros2 launch tianyi25_sim tianyi_nav.launch.py rviz:=true    # 顺带开 RViz 看代价地图
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `use_sim_time` | `true` | 统一使用 Gazebo 仿真时间 |
| `rviz` | `false` | 是否启动 RViz2（Nav2 默认视图） |

说明：

- 本链路**不使用 AMCL / map_server**，定位由 SLAM Toolbox 在线建图承担；
- `autostart=true`，不需要手动 `ros2 lifecycle set` configure/activate；
- 机器人需要先动起来，SLAM 才能建出 `/map`，NavFn 才能在已知栅格上规划路径：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}"
```

> 命名说明：导航入口的真实文件名是 **`tianyi_nav.launch.py`**，工作空间内引用已统一修正。
> `greeting_demo.launch.py` 以及 `greeting_sim_stubs` / `greeting_orchestrator` /
> `greeting_interfaces` 属于**外部的 greeting 工作空间**（`projects/greeting/greeting_ws`），本仓库不含。

---

## 5. 话题与 TF 链

桥接话题（`[` = gz → ROS，`]` = ROS → gz），模型名取自当前整定档（`config/<sweep_config>`）的
`model_name`（默认 `tianyi25`）：

| 话题 | 方向 | 说明 |
| --- | --- | --- |
| `/clock` | gz → ROS | 仿真时间 |
| `/joint_states` | gz → ROS | 关节状态 |
| `/odom` | gz → ROS | 里程计（DiffDrive） |
| `/scan` | gz → ROS | 2D 雷达（前向 270°，10 Hz） |
| `/model/tianyi25/joint_trajectory` | ROS → gz | 上半身关节轨迹 |
| `/model/tianyi25/cmd_vel` | ROS → gz | 底盘速度，launch 重映射为 **`/cmd_vel`** |

TF 链：

```
map ──slam_toolbox──► odom ──gz_odom_tf──► base_footprint ──robot_state_publisher──► base ─► 四轮/雷达/上半身
                        ▲
                   gz DiffDrive /odom
```

数据流：

```
Nav2 ──/cmd_vel──► ros_gz_bridge ──► gz DiffDrive ──► 四轮
gz_joint_sweep ──JointTrajectory──► gz JointTrajectoryController ──► 上半身
gz gpu_lidar ──/scan──► ros_gz_bridge ──► SLAM Toolbox / Nav2 代价地图
```

> 注意：仿真里底盘与本体被焊成**一个模型**，这样才能给 Nav2 提供一棵连通的 TF 树。
> 这是仿真专用处理，实机形态仍按接口契约走（底盘 REST :9090）。

---

## 6. 验证与调试

```bash
# 仿真是否健康
ros2 topic hz /scan                 # 期望 ~10 Hz
ros2 topic hz /odom                 # 期望 ~30 Hz
ros2 topic echo /joint_states --once

# TF 树是否连通
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_footprint

# 手动驱动底盘（速度不得超过 DiffDrive 上限 0.4 m/s、0.8 rad/s）
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.4}}"

# 扫掠启停（false=暂停，true=恢复，恢复时相位连续不跳变）
ros2 service call /tianyi25_sim/set_sweep_enabled std_srvs/srv/SetBool "{data: false}"
```

常见问题速查（详细排查见两个包各自的 README）：

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| 模型穿透地面 / 网格不显示 | `GZ_SIM_RESOURCE_PATH` 缺 `tianyi25_urdf` 的 share 父目录 | 用 `gazebo.launch.py` 启动（已设该环境变量） |
| 雷达无数据 | 世界缺 Sensors 系统 | 用 `greeting_world.sdf`，不要用 `empty.sdf` |
| 轮子空转但车不走 | 真实轮与网格内焊死轮同点接触退化 | 确认 `wheel_radius=0.072`、`base` 的 `mu=0` |
| 关节收到轨迹不出力 | 控制器关节配置顺序错位 | 确认 `joint_cfg_list` 逐关节交错展开 |
| 躯干塌倒 / 打招呼时弯腰 | 缺姿态锁止弹簧（尤其腰俯仰） | 确认 `posture_lock` 中 first_leg=1200、waist=400 |
| Nav2 起不来 | 缺 Nav2 / slam_toolbox 或参数段缺失 | 按 §2 安装；使用本包 `nav2_params.yaml` |
| TF 树割裂 | 某可动关节缺 `/joint_states` | 用 `joint_sweep_demo` 补齐全关节 |
| 停止仿真后仍有进程 | gz / bridge 节点未随 launch 退出 | 手动结束残留的 `gz sim server`、`parameter_bridge`、`gz_joint_sweep`、`gz_odom_tf` |

---

## 7. 关键约定

- `tianyi25_sim/config/` 下由 `sweep_config` 选中的整定档是仿真整定的**唯一真源**
  （`model_name`、`world_name`、扫掠频率、关节表、位置环整定、姿态锁止弹簧刚度）。
  xacro / `gazebo.launch.py` / 扫掠节点都读它，改参数只改这一处；档与档之间的
  `arm_control` / `posture_lock` 是复制的，改机器人级整定要同步两份。
- `model_name` 必须与 `world_name` 和世界文件里的 `<world name="...">` 保持一致
  （默认 `tianyi25` / `greeting`）。
- Nav2 的速度参数**不得超过** DiffDrive 上限：linear ±0.4 m/s、angular ±0.8 rad/s。
- URDF 的 `<surface><friction>`、`<joint><damping>` 会被 Jazzy 的 URDF→SDF 转换器丢弃，
  必须改用 gz 的 `<mu1>/<mu2>`、`<implicitSpringDamper>` 等扩展。
- 腿部关节在迎宾场景恒为 0，所有角度均已核对 URDF `<limit>`。

---

## 8. 相关文档

- [tianyi25_urdf/README.md](src/tianyi25_urdf/README.md) —— 模型、网格、URDF 结构与显示
- [tianyi25_sim/README.md](src/tianyi25_sim/README.md) —— 仿真物理扩展、世界、配置、节点与桥接详解
