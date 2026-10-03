# tianyi25\_urdf

天轶 2.5 人形机器人 URDF 模型与 STL 网格资源包。

沿用厂商包名，保持 URDF 中 `package://tianyi25_urdf/meshes/…` 引用**零改写**。
本包是**纯资源包**：只提供模型文件、RViz 配置与一个显示 launch，不含运行节点。

***

## 1. 包的定位与依赖关系

### 1.1 在整机链路中的位置

```
        ┌────────────────────────────┐
        │       tianyi25_urdf        │  ← 本包（模型 + 网格 + 显示）
        │ URDF / STL / RViz / display│
        └───────────┬────────────────┘
                    │ xacro:include（tianyi25_gazebo.urdf.xacro）
                    │ package:// 引用网格
                    ▼
              tianyi25_sim（Gazebo 仿真扩展）
```

🔴 本包处于依赖图**最底层**（上游资源），`tianyi25_sim` 通过 `xacro:include` 复用纯 URDF，
**不修改**本包任何内容。

### 1.2 依赖一览

| 方向   | 依赖                          | 类型                 | 用途                      |
| ---- | --------------------------- | ------------------ | ----------------------- |
| 构建工具 | `ament_cmake`               | `buildtool_depend` | <br />                  |
| 运行依赖 | `urdf`                      | `exec_depend`      | URDF 解析                 |
| 运行依赖 | `xacro`                     | `exec_depend`      | 宏展开（供下游使用）              |
| 运行依赖 | `robot_state_publisher`     | `exec_depend`      | 发布 TF                   |
| 运行依赖 | `joint_state_publisher_gui` | `exec_depend`      | 关节滑块（`use_gui:=true` 时） |
| 运行依赖 | `rviz2`                     | `exec_depend`      | 可视化                     |
| 被依赖  | `tianyi25_sim`              | `exec_depend`      | 复用模型与网格                 |

> 均使用 `exec_depend` 而非 `depend`，避免未安装时 CMake `find_package` 失败。

### 1.3 构建

```bash
colcon build --packages-select tianyi25_urdf --symlink-install
```

***

## 2. 文件结构与职责

```
tianyi25_urdf/
├── urdf/
│   ├── tianyi25_urdf.urdf            # 主模型（wholebody，RSP/SLAM/仿真均引用）
│   ├── tianyi25_urdf_wholebody.urdf  # 重构版：关节顺序重排，满足 endpose_wholebody_controller
│   ├── tianyi25_urdf_dual.urdf       # 双臂版（robot name: tianyi25_dual）
│   ├── qpArm_description_L.urdf      # 左单臂描述
│   └── qpArm_description_R.urdf      # 右单臂描述
├── meshes/                           # 24 个 STL 网格（base、双臂、头、腿等）
├── rviz/
│   └── urdf.rviz                     # 显示用 RViz 配置
├── launch/
│   └── display.launch.py             # 本地显示：RSP + 关节滑块 + RViz
├── config/
│   └── joint_names_tianyi25_urdf.yaml  # controller 关节名列表
├── CMakeLists.txt / package.xml
└── LICENSE
```

`CMakeLists.txt` 安装规则：`urdf meshes rviz launch config` → `share/tianyi25_urdf/`
（供 URDF 中的 `package://` 与 launch 的 `get_package_share_directory` 引用）。

***

## 3. URDF 文件详解

### 3.1 `tianyi25_urdf.urdf`（主模型）

- **robot name**：`tianyi25_urdf`
- **根坐标系**：`base_footprint`（`endpose_wholebody_controller` 需要此 frame）
- **结构**：`base_footprint →(fixed)→ base → 链式关节 → …`

| 层级  | 关节（revolute）                                                                                 | 说明     |
| --- | -------------------------------------------------------------------------------------------- | ------ |
| 身体链 | `first_leg_pitch_joint` → `second_leg_pitch_joint` → `waist_pitch_joint` → `waist_yaw_joint` | 腿柱 + 腰 |
| 右臂链 | `shoulder_pitch/roll/yaw_r` → `elbow_pitch/yaw_r` → `wrist_pitch/roll_r`                     | 7 DoF  |
| 左臂链 | `shoulder_pitch/roll/yaw_l` → `elbow_pitch/yaw_l` → `wrist_pitch/roll_l`                     | 7 DoF  |
| 头链  | `head_yaw` → `head_pitch` → `head_roll`                                                      | 3 DoF  |

其他 link：`left_tcp_link` / `right_tcp_link`（末端固定）、`camera_head_link`（相机固定）、
`base_collision_3/4`（碰撞球，供 `model_` 使用）。

**关键质量参数**：`base` 质量 **100.13 kg**（含躯干），`waist_yaw_link` 14.88 kg。

> 🔴 `base` 的 62 kg 级躯干重量是**倒立摆不稳定平衡**的根源（见 `tianyi25_sim` 腿柱弹簧）。

### 3.2 `tianyi25_urdf_wholebody.urdf`（重构版）

- **robot name**：`tianyi25_wholebody`
- **关节顺序重排**：`body → L arm → R arm → head(fixed) → 碰撞球`，以满足 `endpose_wholebody_controller` 的 18 revolute 索引要求。
- 从 `tianyi25_urdf.urdf` 提取数据重新排列；头链放在**末尾**，不影响 controller 的 revolute 索引。

### 3.3 `tianyi25_urdf_dual.urdf`（双臂版）

- **robot name**：`tianyi25_dual`
- 结构：`base → waist_yaw_link`，腰部与头部各关节为 **fixed**，仅双臂为 revolute，附带大量碰撞球（`left/right_collision_*`、`*_free_collision_*`）。

### 3.4 `qpArm_description_L.urdf` / `_R.urdf`（单臂描述）

- 左/右**单臂**描述（无对侧手臂，`:L` 含 `left_tcp_link`，`:R` 含 `right_tcp_link`）。

### 3.5 `config/joint_names_tianyi25_urdf.yaml`

供控制器使用的关节名列表（顺序敏感）：

```yaml
controller_joint_names: ['', 'first_leg_pitch_joint', 'second_leg_pitch_joint',
  'waist_pitch_joint', 'waist_yaw_joint', 'head_yaw_joint', 'head_pitch_joint', 'head_roll_joint',
  'shoulder_pitch_r_joint', ..., 'wrist_roll_r_joint',
  'shoulder_pitch_l_joint', ..., 'wrist_roll_l_joint']
```

> 首元素为空字符串 `''` 是厂商格式占位；共 21 个可动关节（1 空 + 20 关节 + 尾逗号）。

***

## 4. 网格资源（`meshes/`）

| 类别  | 文件                                                                                                 |
| --- | -------------------------------------------------------------------------------------------------- |
| 底盘  | `base.STL`（四角内置四个静态轮形）                                                                             |
| 腿/腰 | `first_leg_pitch_link.STL`、`second_leg_pitch_link.STL`、`waist_pitch_link.STL`、`waist_yaw_link.STL` |
| 双肩  | `shoulder_pitch/roll/yaw_{l,r}_link.STL`                                                           |
| 肘/腕 | `elbow_{pitch,yaw}_{l,r}_link.STL`、`wrist_{pitch,roll}_{l,r}_link.STL`                             |
| 末端  | `left_tcp_link.STL`、`right_tcp_link.STL`                                                           |
| 头部  | `head_{yaw,pitch,roll}_link.STL`、`camera_head_link.STL`                                            |

URDF 中通过 `package://tianyi25_urdf/meshes/<file>.STL` 引用。

> 🔴 网格只含**几何**；`base.STL` 内的轮形是**静态绘制**，无旋转关节。
> 仿真侧真实轮关节由 `tianyi25_sim` 在重合位置补出（见其 README §3）。

***

## 5. Launch 与调用方法

### 5.1 `launch/display.launch.py`

直接读取**纯 URDF**（非 xacro），因此不依赖 xacro。

```bash
# 全功能：关节滑块 GUI + RViz
ros2 launch tianyi25_urdf display.launch.py

# 仅 RViz（关节静止）
ros2 launch tianyi25_urdf display.launch.py use_gui:=false
```

| launch 参数  | 默认     | 说明                                                            |
| ---------- | ------ | ------------------------------------------------------------- |
| `use_gui`  | `true` | 启动 `joint_state_publisher_gui`（需 `joint-state-publisher-gui`） |
| `use_rviz` | `true` | 启动 RViz2                                                      |

拉起节点：

| 节点                          | 条件         | 作用                           |
| --------------------------- | ---------- | ---------------------------- |
| `joint_state_publisher_gui` | `use_gui`  | 发布 `/joint_states`（滑块）       |
| `robot_state_publisher`     | 始终         | 发布 `/robot_description` 与 TF |
| `rviz2`                     | `use_rviz` | 加载 `rviz/urdf.rviz`          |

依赖：`ros-jazzy-joint-state-publisher-gui`（`use_gui:=true` 时）、`ros-jazzy-rviz2`、`ros-jazzy-robot-state-publisher`。

### 5.2 被下游复用的方式（`tianyi25_sim`）

```xml
<!-- tianyi25_sim/urdf/tianyi25_gazebo.urdf.xacro -->
<xacro:include filename="$(find tianyi25_urdf)/urdf/tianyi25_urdf.urdf"/>
```

xacro 展开后交给 `robot_state_publisher` 与 `ros_gz_sim create`。

### 5.3 直接命令行查看（无 GUI 依赖）

```bash
# 校验 URDF 合法性
check_urdf install/tianyi25_urdf/share/tianyi25_urdf/urdf/tianyi25_urdf.urdf

# 打印关节
ros2 run urdf_parser_py ...  # 或
xacro install/tianyi25_urdf/share/tianyi25_urdf/urdf/tianyi25_urdf.urdf | head
```

***

## 6. 验证与调试

```bash
# 启动显示
ros2 launch tianyi25_urdf display.launch.py

# 另一终端查看 TF
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_footprint base

# 关节状态
ros2 topic echo /joint_states --once
```

**常见问题**

| 现象                            | 原因                                       | 处理                                                     |
| ----------------------------- | ---------------------------------------- | ------------------------------------------------------ |
| RViz 中模型全白/不显示网格              | `package://` 未解析（未 source 工作空间）          | `source install/setup.bash`                            |
| `xacro` 报宏错误                  | 使用 `display.launch.py` 读的纯 URDF 不含 xacro | 纯 URDF 无需 xacro；勿手动 xacro 展开                           |
| TF 树割裂                        | 关节滑块只发部分关节                               | 用 `joint_sweep_demo` 补齐全关节，或确认滑块的关节集合                  |
| 缺 `joint_state_publisher_gui` | 未安装                                      | `sudo apt install ros-jazzy-joint-state-publisher-gui` |

***

## 7. 注意事项

- 🔴 本包是**上游资源包**，下游 `tianyi25_sim` 通过 include 复用，**不修改**本包内容。
- 🔴 修改任何 URDF/网格须同步核查 `config/joint_names_tianyi25_urdf.yaml` 的关节顺序。
- 🔴 纯 URDF 是静态几何 + 惯性 + 限位；**动力学整定**（位置环、弹簧、摩擦）在 `tianyi25_sim`。
- 🔴 厂商包名 `tianyi25_urdf` 不可改，否则 `package://` 引用全部失效。

***

## 8. 相关文档

- [tianyi25\_sim/README.md](../tianyi25_sim/README.md)（仿真的物理扩展）

