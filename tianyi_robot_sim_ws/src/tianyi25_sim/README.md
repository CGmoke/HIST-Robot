# tianyi25\_sim

天轶 2.5 仿真包 —— Gazebo Harmonic（gz-sim 8）世界、模型扩展、控制器配置、
ROS↔Gazebo 桥接、SLAM Toolbox + Nav2 导航栈，以及全部仿真启动入口。

***

## 1. 包的定位与依赖关系

### 1.1 在整机链路中的位置

```
tianyi25_urdf（纯 URDF + STL）           ┌──────────────────┐
        │ xacro:include                  │  greeting_sim_stubs │
        ▼                                │  greeting_orchestrator │  ← 运行时被本包 launch include
┌───────────────────────┐  include       └──────────────────┘
│     tianyi25_sim      │◄────────────── greeting_demo.launch.py
│  世界/模型/桥接/导航    │                （config 又被两个桩反过来读取）
│  扫掠节点/odom TF 节点  │
└───────────────────────┘
```

### 1.2 依赖一览

| 方向              | 依赖                                                                                                      | 用途                                                 |
| --------------- | ------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| 构建工具            | `ament_cmake`                                                                                           | <br />                                             |
| 运行依赖            | `tianyi25_urdf`                                                                                         | xacro include 纯 URDF + 网格                          |
| 运行依赖            | `ros_gz_sim`、`ros_gz_bridge`                                                                            | Gazebo 启动与消息桥接                                     |
| 运行依赖            | `robot_state_publisher`                                                                                 | 发布 `/robot_description` 与 TF                       |
| 运行依赖            | `xacro`                                                                                                 | 展开 `tianyi25_gazebo.urdf.xacro`                    |
| 运行依赖            | `rclpy`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`tf2_ros`、`trajectory_msgs`、`std_srvs`、`rosgraph_msgs` | 各节点消息类型                                            |
| 🔴 **运行时（未声明）** | `greeting_sim_stubs`、`greeting_orchestrator`                                                            | `greeting_demo.launch.py` include，但**故意不声明**以避包依赖环 |

🔴 关于包依赖环：`greeting_sim_stubs` / `greeting_orchestrator` 运行时读本包 `config`，
若本包再声明对它们的 `exec_depend` 会成环。缺失时报 "package not found"，按提示构建即可。

### 1.3 构建

```bash
colcon build --packages-select tianyi25_urdf tianyi25_sim --symlink-install
```

***

## 2. 文件结构与职责

```
tianyi25_sim/
├── urdf/
│   └── tianyi25_gazebo.urdf.xacro   # include 纯 URDF + 追加四轮/雷达/gz 插件
├── worlds/
│   └── greeting_world.sdf          # 12×12m 房间 + 家具 + Sensors 系统
├── config/
│   ├── gz_joint_sweep.yaml         # 关节扫掠 + 上半身位置环整定 + 腿柱弹簧（唯一真源）
│   ├── motions.yaml                # 礼仪动作关键帧表
│   ├── waypoints.yaml              # 迎宾点位表（仿真专用）
│   ├── slam_toolbox_params.yaml    # SLAM Toolbox 在线建图参数
│   └── nav2_params.yaml            # Nav2 参数
├── launch/
│   ├── gazebo.launch.py            # 世界+模型+桥接+扫掠+odom TF
│   ├── greeting_nav.launch.py      # SLAM Toolbox + Nav2
│   ├── greeting_demo.launch.py     # 一键拉起整条链路
│   └── display_motion.launch.py    # 纯 RViz 显示 + 关节扫掠
├── scripts/
│   ├── gz_joint_sweep.py           # 向 gz 控制器周期下发正弦扫掠轨迹
│   ├── gz_odom_tf.py               # /odom → odom→base_footprint TF
│   └── joint_sweep_demo.py         # RViz 演示用 /joint_states 正弦发布
├── CMakeLists.txt / package.xml
└── LICENSE
```

`CMakeLists.txt` 安装规则：

- `worlds launch config models urdf` → `share/tianyi25_sim/`（供 `get_package_share_directory` 引用）；
- `scripts/*.py` → `lib/tianyi25_sim/`，改名去 `.py`（供 `ros2 run` 与 `Node(executable=…)` 调用）。

| 可执行名               | 脚本                            |
| ------------------ | ----------------------------- |
| `joint_sweep_demo` | `scripts/joint_sweep_demo.py` |
| `gz_joint_sweep`   | `scripts/gz_joint_sweep.py`   |
| `gz_odom_tf`       | `scripts/gz_odom_tf.py`       |

***

## 3. 机器人模型（`urdf/tianyi25_gazebo.urdf.xacro`）

做法：**include** **`tianyi25_urdf`** **的纯 URDF（不改其内容）**，只在本包追加仿真专用内容，
使 RViz 动显用的纯 URDF 保持干净。

### 3.1 追加内容

1. **四轮底盘**：`base.STL` 网格四角本身内置四个轮形（静态、无关节）。本文件在**重合**位置补
   4 个 `continuous` 轮关节（只有 collision/inertial，无 visual），构成 **4WD skid-steer**，由 gz `DiffDrive` 驱动。
2. **2D 雷达**（`gpu_lidar`）：前向 270°，直接贴在 `base` 前甲板（无立柱）。
3. **gz 插件**：`JointTrajectoryController`（上半身）、`JointStatePublisher`、`DiffDrive`（四轮+里程计）。

### 3.2 四轮几何参数

| 参数                             | 值             | 说明                   |
| ------------------------------ | ------------- | -------------------- |
| `wheel_radius`                 | 0.072         | 真实碰撞轮半径（比网格轮形大 2 mm） |
| `wheel_width`                  | 0.08          | 轮宽                   |
| `wheel_separation`             | 0.44          | 左右轮心平面距 = 2×0.22     |
| `axle_z`                       | 0.06          | 轮轴在 base 中的高度        |
| `rear_axle_x` / `front_axle_x` | −0.20 / +0.30 | 前后轴                  |
| `wheel_y`                      | ±0.22         | 左右轮心                 |

真实轮底 = `axle_z − wheel_radius` = −0.012 → `spawn_z = 0.012` 让四轮落地。

### 3.3 两个关键物理处理

- **(a) 真实轮大 2 mm**：真实轮先触地承担全部支撑，焊死在 base 的网格轮形离地 1.5\~2 mm，
  避免"转轮 + 焊死轮同点接触"的退化受力（ODE 法向力分配不稳 → 轮子空转车不走）。
- **(b)** **`base`** **摩擦覆写为 0**：`<gazebo reference="base"><mu1>0</mu1><mu2>0</mu2>`，
  网格轮形即使擦地也只无摩擦滑过；抓地只由 4 个真实轮（`mu=1.0`）提供。

> 🔴 URDF 的 `<collision><surface><friction>` 会被 Jazzy 的 URDF→SDF 转换器**丢弃**，
> 必须用 gz 的 `<mu1>/<mu2>` 扩展。

### 3.4 雷达规格

| 项  | 值                                                    |
| -- | ---------------------------------------------------- |
| 安装 | `base` 前甲板，`lidar_x=0.20, z=0.245`（与 base 上表面齐平，无立柱） |
| 帧  | `gz_frame_id = lidar_link`（与 RSP 的 TF 对齐，避免模型作用域前缀）  |
| 扫描 | 540 samples，±135°（270° 前向），`update_rate=10`          |
| 量程 | 0.30\~12.0 m，分辨率 0.01，高斯噪声 σ=0.01                    |
| 话题 | `/scan`                                              |

### 3.5 gz 插件

| 插件                          | 作用         | 关键话题                                                                  |
| --------------------------- | ---------- | --------------------------------------------------------------------- |
| `JointTrajectoryController` | 上半身关节轨迹执行  | `/model/tianyi25/joint_trajectory`                                    |
| `JointStatePublisher`       | 关节状态       | `joint_states`                                                        |
| `DiffDrive`                 | 四轮驱动 + 里程计 | cmd\_vel 固定 `/model/tianyi25/cmd_vel`（launch 重映射为 `/cmd_vel`）；`/odom` |
| `Sensors`（写在 world 里）       | 雷达渲染       | —                                                                     |

**DiffDrive 速度上限**：linear ±0.4 m/s、angular ±0.8 rad/s、linear accel ±0.5、angular accel ±0.8。

> 🔴 Nav2 的速度参数**不得超过**这些上限（`velocity_smoother` 再兜一层）。

**腿柱锁止弹簧**：`first/second_leg_pitch_joint` 加 K=400 Nm/rad 扭转弹簧
（`<implicitSpringDamper>1` + `<springStiffness>`）。原因：0 位是倒立摆**不稳定平衡点**，
重力力矩 ≈ 215 Nm/rad > 位置环饱和力矩 142 Nm，不加弹簧躯干必塌。K=400 > 215 → 0 位转为稳定平衡。

> 🔴 不要在 URDF 里用 `<parent link="world">` 固定关节"站住"整机 —— gz-sim 8.15 下模型会被判为不可动，
> 内部关节既不驱动也不受重力。整机站立由四轮承担。

***

## 4. 世界文件（`worlds/greeting_world.sdf`）

- **12×12 m 房间**：四面墙（厚 0.2、高 2.0）+ 地面，给 SLAM 提供边界与特征；
- **家具**：接待台（3.0, 0）、立柱 A/B（−3.0, ±2.5）、箱子 A/B（2.5, 3.5）/（−4.0, −3.5）；
- **系统插件**：Physics / UserCommands / SceneBroadcaster / Contact + 🔴 **Sensors（`render_engine=ogre2`）**。

> 🔴 不用 `empty.sdf`：它没有 Sensors 系统，`gpu_lidar` 不出数据；纯空世界 SLAM 无特征会漂移。
> 世界名 `greeting` 与 `gz_joint_sweep.yaml` 的 `world_name` 必须一致（`ros_gz_sim create -world`）。

***

## 5. 配置文件详解

### 5.1 `config/gz_joint_sweep.yaml`（仿真整定**唯一真源**）

被 xacro / launch / 扫掠节点 / play\_motion 桩**共用**，避免关节名重复维护。

| 段             | 内容                                                    |
| ------------- | ----------------------------------------------------- |
| 顶层            | `model_name`、`world_name`、扫掠频率/相位/时长/采样/重发周期          |
| `joints`      | 关节名 → `[中心角, 振幅]`；振幅 0 = 仅保持（补齐 TF 树、防垂臂）；腿部恒 0       |
| `arm_control` | 上半身位置环整定：`torque_ratio`、逐关节 `torque_limits_nm`、PID 增益 |
| `leg_lock`    | 腿柱扭转弹簧刚度（`first/second_leg_pitch_joint = 400`）        |

🔴 `position_cmd_min/max` 语义是关节**力矩饱和值 (Nm)**，不是角速度；
按 `<limit effort> × torque_ratio` 逐关节取值（轻载头部给 20 会冲限位）。

🔴 `JointTrajectoryController` **按元素出现顺序**解析配置：`<joint_name>` 后紧跟该关节的
`<initial_position>` 与 PID，必须**逐关节交错展开**（`joint_cfg_list` 宏），否则解析错位、
增益取不到值 → "收到轨迹但关节不出力"。

### 5.2 `config/motions.yaml`（动作关键帧表）

| 键         | 内容                                             |
| --------- | ---------------------------------------------- |
| `home`    | 中立待机姿态（与扫掠中心角一致，衔接不跳变）                         |
| `motions` | 动作名 → `{duration, keyframes:[{t, positions}]}` |

动作：`salute_bow` / `wave_official` / `point_right` / `point_left` / `farewell_bow` / `photo_pose` / `idle_scan`。

🔴 腿部恒 0，迎宾场景禁腿部动作；所有角度已核对 URDF `<limit>`。

### 5.3 `config/waypoints.yaml`（仿真专用点位表）

`frame_id: map`；点位：`standby` / `desk` / `pillar_a` / `pillar_b` / `exit`（`{x, y, yaw}`）。
🔴 与 `greeting_world.sdf` 家具对应，**不得当实机点位用**。

### 5.4 `config/slam_toolbox_params.yaml`（在线建图）

`mode=mapping`、`base_frame=base_footprint`、`odom_frame=odom`、`scan_topic=/scan`、
Ceres 求解器；发布 TF 的 `map→odom` 段与 `/map`。
🔴 270° 前向雷达正后方 \~90° 无数据，`minimum_travel_heading` 取略大（0.2 rad）避免小角度抖动。

### 5.5 `config/nav2_params.yaml`（Nav2）

| 节点                  | 要点                                                                                        |
| ------------------- | ----------------------------------------------------------------------------------------- |
| `bt_navigator`      | `global_frame=map`，`robot_base_frame=base_footprint`；**故意不写** `plugin_lib_names`（用内置默认全量） |
| `controller_server` | 15 Hz；`FollowPath` = **RPP**（`desired_linear_vel=0.30`，`allow_reversing=false`——倒车无雷达数据）  |
| `smoother_server`   | `SimpleSmoother`                                                                          |
| `planner_server`    | `NavFn`（Dijkstra），`allow_unknown=true`（建图初期）                                              |
| `behavior_server`   | spin/backup/drive\_on\_heading/wait；`local_frame=odom`、`global_frame=map`                 |
| `velocity_smoother` | `max_velocity=[0.40, 0.0, 1.00]`（与 DiffDrive 一致）                                          |
| `collision_monitor` | `polygons: ["FootprintApproach"]`，🔴 `enabled: false`（直通模式，见下）                            |
| `docking_server`    | 无充电桩但**必须能 configure**，给"未使用"配置                                                           |
| `local_costmap`     | 6×6 rolling，`robot_radius=0.45`，obstacle+inflation                                        |

🔴 **collision\_monitor 为何退化直通**：270° 雷达在 ±123°\~±135° 打到本体自回波（49 个 0.30 m 常驻点），
且落在 `robot_radius` 内，任何多边形都会被永久命中 → 被持续限速/刹停。故关掉判据多边形，只保留
速度链直通（`cmd_vel_smoothed → cmd_vel`），避障交给代价地图（`obstacle_min_range=0.40` 滤自回波）。
不能写 `polygons: []`（ROS 2 YAML 无法表达空数组，会抛异常）。

🔴 **速度链**：`controller/behavior → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed → collision_monitor → cmd_vel → ros_gz_bridge → gz DiffDrive`。

***

## 6. Launch 与调用方法

### 6.1 `gazebo.launch.py` —— 核心仿真

```bash
ros2 launch tianyi25_sim gazebo.launch.py
ros2 launch tianyi25_sim gazebo.launch.py world:=empty.sdf spawn_z:=0.012
```

| launch 参数 | 默认                                | 说明                  |
| --------- | --------------------------------- | ------------------- |
| `world`   | `<pkg>/worlds/greeting_world.sdf` | 世界文件                |
| `spawn_z` | `0.012`                           | 生成高度（= 四轮轮底深度，正好落地） |

拉起节点：`gz sim`、`robot_state_publisher`、`ros_gz_sim create`、`ros_gz_bridge`、`gz_joint_sweep`、`gz_odom_tf`。

**桥接话题**（`[` = gz→ROS，`]` = ROS→gz）：

| 桥                                  | 方向                      |
| ---------------------------------- | ----------------------- |
| `/clock`                           | gz→ROS                  |
| `/joint_states`                    | gz→ROS                  |
| `/model/tianyi25/joint_trajectory` | ROS→gz                  |
| `/odom`                            | gz→ROS                  |
| `/scan`                            | gz→ROS                  |
| `/model/tianyi25/cmd_vel`          | ROS→gz（重映射为 `/cmd_vel`） |

🔴 `GZ_SIM_RESOURCE_PATH` 需含 `tianyi25_urdf` 的 share 父目录，否则网格/collision 加载失败（模型穿透地面）。

### 6.2 `greeting_nav.launch.py` —— SLAM + Nav2

```bash
ros2 launch tianyi25_sim greeting_nav.launch.py
ros2 launch tianyi25_sim greeting_nav.launch.py rviz:=true
```

| launch 参数      | 默认      | 说明                                |
| -------------- | ------- | --------------------------------- |
| `use_sim_time` | `true`  | <br />                            |
| `rviz`         | `false` | 顺带启动 RViz2（需 `nav2_rviz_plugins`） |

前提（一次性）：

```bash
sudo apt install -y ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs \
                    ros-jazzy-slam-toolbox ros-jazzy-nav2-rviz-plugins
```

`_require()` 在缺包时抛**可读错误**而非 traceback。

### 6.3 `greeting_demo.launch.py` —— 一键全链路

```bash
ros2 launch tianyi25_sim greeting_demo.launch.py
ros2 launch tianyi25_sim greeting_demo.launch.py nav:=true     # 需先装 Nav2
```

依次 include：`gazebo.launch.py` → `stubs.launch.py` → `orchestrator.launch.py` →（`nav:=true` 时）`greeting_nav.launch.py`。

拉起后触发接待：

```bash
ros2 topic pub --once /greeting/control_cmd greeting_interfaces/msg/ConsoleCommand "{cmd: 'start'}"
ros2 topic echo /greeting/state
```

### 6.4 `display_motion.launch.py` —— 纯 RViz 演示

```bash
ros2 launch tianyi25_sim display_motion.launch.py
ros2 launch tianyi25_sim display_motion.launch.py frequency:=0.4 use_rviz:=false
```

\= `tianyi25_urdf` 的 display（`use_gui=false`）+ `joint_sweep_demo`。

***

## 7. 三个仿真节点

### 7.1 `gz_joint_sweep`

周期向 gz `JointTrajectoryController` 下发正弦扫掠轨迹。

| 参数                 | 类型       | 默认                                | 说明                |
| ------------------ | -------- | --------------------------------- | ----------------- |
| `config_file`      | `string` | `""`（**必填**）                      | 读取 `joints`/频率/相位 |
| `trajectory_topic` | `string` | `""`                              | 输出话题              |
| `enable_service`   | `string` | `/tianyi25_sim/set_sweep_enabled` | `SetBool` 启停服务    |

```
pos[i](t) = center[i] + amp[i] · sin(ω·t + i·phase_step)
```

- 每次下发 `horizon` 秒轨迹、每 `publish_period` 秒重发；
- 启停服务 `false`=暂停、`true`=恢复（**恢复时相位连续、不跳变**，通过平移 `_t0` 实现）。

> 🔴 与 `play_motion` 桩共用同一轨迹话题；桩播放前先暂停扫掠，避免两条轨迹互相覆盖
> （gz 会告警 `received while executing a previous trajectory`）。

### 7.2 `gz_odom_tf`

把 gz `/odom` 转成 `odom → base_footprint` 的 TF（不用 gz 自带 TF，因 frame 名带模型前缀）。

| 参数           | 类型       | 默认               | 说明 |
| ------------ | -------- | ---------------- | -- |
| `odom_topic` | `string` | `/odom`          | 输入 |
| `odom_frame` | `string` | `odom`           | 父帧 |
| `base_frame` | `string` | `base_footprint` | 子帧 |

- 平面移动：只取 x/y/yaw，z 与 roll/pitch 置 0（避免地面接触抖动进 TF 树）；
- 另一半 `map → odom` 由 SLAM Toolbox（或 AMCL）发布。

### 7.3 `joint_sweep_demo`

纯 `rclpy` 发布 `/joint_states` 正弦扫掠，供 RViz 演示（不依赖 `joint_state_publisher_gui`）。

| 参数                | 类型       | 默认     | 说明        |
| ----------------- | -------- | ------ | --------- |
| `rate`            | `double` | `50.0` | 发布频率 (Hz) |
| `frequency`       | `double` | `0.15` | 正弦频率 (Hz) |
| `amplitude_scale` | `double` | `1.0`  | 振幅统一缩放    |

🔴 **必须发布 URDF 中全部可动关节**：`robot_state_publisher` 只为 `/joint_states` 出现过的关节发 TF，
漏一个 → TF 树被割裂（`Tf has two or more unconnected trees`）。故非扫掠关节也以振幅 0 发布。

***

## 8. TF 链与数据流

```
map ──slam_toolbox──► odom ──gz_odom_tf──► base_footprint ──robot_state_publisher──► base ─► 四轮/雷达/上半身
                        ▲
                   gz DiffDrive /odom
```

```
Nav2 ──/cmd_vel──► ros_gz_bridge ──► gz DiffDrive ──► 四轮
play_motion ──JointTrajectory──► /model/tianyi25/joint_trajectory ──► gz JointTrajectoryController ──► 上半身
gz gpu_lidar ──/scan──► ros_gz_bridge ──► SLAM Toolbox / Nav2 代价地图
```

🔴 **仿真偏离契约之处**：契约 §9 冻结"底盘管 map/odom/base\_link、本体管关节姿态、两套 TF 不连通"（实机形态）。
仿真里 Nav2 必须有一棵连通 TF，故把底盘与本体焊成一个模型。这是**仿真专用**处理，不改契约；实机仍走 `NavigateTo → REST :9090`。

***

## 9. 验证与调试

```bash
# 仿真是否健康
ros2 topic hz /scan          # ~10 Hz
ros2 topic hz /odom          # ~30 Hz
ros2 topic echo /joint_states --once

# TF 树是否连通
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_footprint

# 手动驱动底盘
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.4}}"

# 扫掠启停
ros2 service call /tianyi25_sim/set_sweep_enabled std_srvs/srv/SetBool "{data: false}"
```

**常见问题**

| 现象                       | 原因                                         | 处理                                       |
| ------------------------ | ------------------------------------------ | ---------------------------------------- |
| 模型穿透地面 / 网格不显示           | `GZ_SIM_RESOURCE_PATH` 缺 urdf share 父目录    | 用 `gazebo.launch.py`（已设环境变量）             |
| 雷达无数据                    | 世界缺 Sensors 系统                             | 用 `greeting_world.sdf`，非 `empty.sdf`     |
| 轮子空转车不走                  | 真轮与焊死轮同点接触退化                               | 确认 `wheel_radius=0.072`、`base` mu=0      |
| 关节收到轨迹不出力                | 控制器关节配置顺序错位                                | 检查 `joint_cfg_list` 交错展开                 |
| 躯干塌倒                     | 缺腿柱弹簧                                      | 确认 `leg_stiffness`/`springStiffness=400` |
| `graceful controller` 崩溃 | costmap `width/height` 写成浮点                | 必须整数（`6` 非 `6.0`）                        |
| Nav2 起不来                 | 缺 `collision_monitor`/`docking_server` 参数段 | 用本包 `nav2_params.yaml`                   |
| 底盘被持续刹停                  | collision\_monitor 多边形命中自回波                | 确认 `FootprintApproach.enabled: false`    |
| TF 树割裂                   | 某可动关节缺 `/joint_states`                     | `joint_sweep_demo` 补齐全部关节                |

***

## 10. 注意事项

- 🔴 `gz_joint_sweep.yaml` 是仿真整定的**唯一真源**，xacro/launch/节点/桩均读它，改参数只改这一处。
- 🔴 `JointTrajectoryController` 关节配置**必须逐关节交错**。
- 🔴 URDF 的 `<surface><friction>`、`<joint><damping>` 都会被 Jazzy 转换器丢弃，需用 gz 扩展。
- 🔴 停止仿真后可能残留 `gz sim server` / `parameter_bridge` / `gz_joint_sweep` / `gz_odom_tf` 进程，需手动终止。
- 🔴 本包是**仿真侧**产物，实机形态仍遵循接口契约（底盘 REST :9090）。

***

## 11. 相关文档

- [tianyi25\_urdf/README.md](../tianyi25_urdf/README.md)

