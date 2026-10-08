# greeting_body

天轶 2.5 **实机**上半身（头 / 双臂 / 腰）礼仪动作库 —— `PlayMotion` Action 服务端。

把 `config/motions.yaml` 里的关键帧动作，翻译成实机 `ros2_bridge_msgs` 控制帧
（`/head/cmd` + `/arm/cmd` + `/waist/cmd`），以 **≥10 Hz 连续流式下发**（实机命令非一次性生效，必须持续保持）。

本包作为 `/greeting/play_motion` Action 服务端，直连实机电机，编排层通过该 Action 驱动本体动作。

---

## 1. 包的定位与依赖关系

### 1.1 在整机链路中的位置

```
        greeting_orchestrator（唯一指挥者）
                    │ Action 客户端
                    ▼
        /greeting/play_motion  (greeting_interfaces/action/PlayMotion)
                    │
                    ▼
              greeting_body        ← 本包
              （实机：流式关节帧）
                    │ ≥10 Hz 连续下发
              ┌─────┼──────────────┐
              ▼     ▼              ▼
        /head/cmd  /arm/cmd   /waist/cmd
        (HeadCtrl) (ArmCtrl)  (WaistCtrl)
              │        │           │
              └────────► 实机控制桥 / 电机 ◄────────┘
                    （订阅 /robot_state 取实测角）
```

🔴 **单一指挥者规则**：本包只做 `/greeting/play_motion` 的**服务端**，不主动调用任何其他 Action；
动作由 `greeting_orchestrator` 编排下发。

### 1.2 依赖一览

| 方向 | 依赖 | 用途 |
| --- | --- | --- |
| 运行依赖 | `rclpy`、`rclpy.action` | Action 服务端、节点 |
| 运行依赖 | `greeting_interfaces` | `PlayMotion` 动作类型 |
| 运行依赖 | `ros2_bridge_msgs` | `HeadCtrl` / `ArmCtrl` / `WaistCtrl` / `MotorCtrl` / `RobotState` |
| 运行依赖 | `std_msgs` | 控制帧 `Header` |
| 运行依赖 | `python3-yaml` | 读取 `motions.yaml` / `joint_map.yaml` |
| 运行依赖 | `launch`、`launch_ros` | launch 文件 |

🔴 `ros2_bridge_msgs` 是**实机平台**（`/home/nvidia/xos`）提供的消息包 ——
因此本包**只在实机上构建运行**。

### 1.3 构建（实机）

```bash
# 前置：已 source 实机平台环境（提供 ros2_bridge_msgs）
source /home/nvidia/xos/setup.bash

cd /home/nvidia/greeting-body-motion/greeting_ws
colcon build --packages-select greeting_interfaces greeting_body --symlink-install
source install/setup.bash
```

---

## 2. 文件结构与职责

```
greeting_body/
├── greeting_body/
│   ├── __init__.py
│   ├── body_adapter.py       # 实机本体通道适配层：URDF 关节角 -> 电机控制帧 + 全部安全约束
│   └── motion_server.py      # /greeting/play_motion Action 服务端 + 关键帧插值
├── config/
│   ├── joint_map.yaml        # URDF 关节名 -> 电机 ID 映射 + 各关节 URDF 行程(rad)
│   └── motions.yaml          # 礼仪动作关键帧表（home + motions）
├── launch/
│   └── motion.launch.py      # 启动 motion_server + 参数
├── test/                     # ament 模板测试（copyright / flake8 / pep257）
├── setup.py / setup.cfg / package.xml
└── resource/greeting_body
```

可执行入口（`setup.py` `console_scripts`）：

| executable | 模块 | 说明 |
| --- | --- | --- |
| `motion_server` | `greeting_body.motion_server:main` | 唯一的可执行节点 |

| 文件 | 职责 |
| --- | --- |
| `body_adapter.py` | 姿态 → 控制帧翻译、软限位/单帧增量/跟随误差钳制、通道预检、电机健康门禁 |
| `motion_server.py` | Action 服务端、`motions.yaml` 展开、smoothstep 插值、warmup、feedback |
| `joint_map.yaml` | 关节分组 / 顺序 / 电机 ID / 行程；🔴 不含腿部 |
| `motions.yaml` | `home` 中立姿态 + 各动作 `{duration, keyframes}` |

---

## 3. 对外接口

### 3.1 Action 服务端

| 项 | 值 |
| --- | --- |
| 动作名 | `/greeting/play_motion` |
| 类型 | `greeting_interfaces/action/PlayMotion` |
| Goal | `string motion_name`、`float32 speed_scale` |
| Result | `bool success`、`string message` |
| Feedback | `float32 progress`（0.0~1.0） |

**Goal 校验**：`motion_name` 不在动作表 → `REJECT`。
**speed 钳制**：服务端是**最后一道闸**，`speed = clamp(speed_scale, 0.05, speed_limit)`，`speed_limit` 硬性 ≤0.7。

### 3.2 订阅（输入）

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/robot_state` | `ros2_bridge_msgs/RobotState` | 关节实测角 + 电机错误码；用于**动作起点**与**健康门禁** |

### 3.3 发布（输出）

| 话题 | 类型 | 分组 | 说明 |
| --- | --- | --- | --- |
| `/head/cmd` | `HeadCtrl` | head | 头 3 电机（id 1/2/3） |
| `/arm/cmd` | `ArmCtrl` | arm | 左右臂 **14 电机合并为一条**（id 11~17 / 21~27） |
| `/waist/cmd` | `WaistCtrl` | waist | 腰 2 电机（id 31/32） |

🔴 **腿部电机 51/52 永不下发**（有倾倒风险，契约 §7.1）；`joint_map.yaml` 中根本不含腿。

---

## 4. 实机控制约定（`body_adapter.py`）

| 约定 | 值 | 说明 |
| --- | --- | --- |
| 控制模式 | `mode=0`（位置模式） | 对齐天轶 2.5 官方 `body_control`；SDK 不传增益，刚度由驱动器内部决定 |
| `label` | `0` | 实机约定 |
| `reserved` | `165` | **SDK 模式移交魔数**：首次以 165 下发，驱动器才接受 SDK 位置命令 |
| `header.frame_id` | `head` / `arm` / `waist` | 见 `joint_map.yaml` 的 `frame_ids` |
| 整组下发 | 必需 | 左臂 7 + 右臂 7 合并为**一条** `/arm/cmd`，缺一不可 |
| 下发频率 | 默认 50 Hz（≥10 Hz） | 实机命令需连续保持；50 Hz 可降低滞后型抖动 |
| 通道预检 | 必需 | 话题无订阅者时命令被静默丢弃 |
| 速度 `spd` | 位置模式 = 「期望速度」 | 必须 ≥ 轨迹实际速度，否则电机持续滞后追赶 → 抖动 |

**速度下发策略**：逐关节取「配置速度 × `speed_scale`」与「前馈速度 × 1.5 余量」的**较大者**，
再统一钳到 `MAX_FEEDFORWARD_SPEED = 3.0 rad/s`（对齐官方 `body_control` 的 `MAX_SPEED`）。
前馈速度优先用**解析速度**（`motion_server._vel_at` 的 smoothstep 导数），避免差分被下发时刻抖动放大。

> ⚠️ 回退路径 `arm_control_mode:=1`（力位混合，显式下发 Kp/Kd）在本机型实测会**持续高频啸叫**，
> 非官方做法，**不要默认启用**；仅在排查时用 `arm_kp_scale`/`arm_kd_scale` 逐步下调。

---

## 5. 安全约束（本层是不可绕过的最后一道闸）

| 约束 | 默认值 | 参数 |
| --- | --- | --- |
| 腿部电机禁用 | 硬编码 | `FORBIDDEN_GROUPS`（`joint_map.yaml` 不含腿） |
| 软限位收缩 | `0.05 rad` | `soft_limit_margin_rad`（URDF `<limit>` 内缩） |
| 单帧位置增量 | `≤0.2 rad` | `max_step_rad`（避免阶跃跳变） |
| 臂跟随误差钳制 | `≤0.10 rad` | `max_follow_err_rad`（期望位置限制在「实测 ± 该值」，防 Kp 顶死限位粘滑颤振） |
| 速度缩放上限 | `≤0.7` | `speed_limit`（礼仪硬要求） |
| 前馈速度上限 | `≤3.0 rad/s` | `MAX_FEEDFORWARD_SPEED`（常量） |
| 通道预检 | `required_groups=['arm']` | 无订阅者 → 直接 aborted |
| 电机健康门禁 | 开 | 有反馈、`error==0`、角度落在硬限位内，否则 aborted |

**电机健康门禁为什么必要**：控制桥在跑、话题也有订阅者，电机仍可能通讯掉线（`error=0x8130` "motor lost
connection"），此时 `pos/cur` 恒为 0.0 的**假值**。若把 0.0 当成实测起点，插值基线错误 → 动作首帧大幅跳变。

🔴 **本方案不能替代实机测试**：首次调试务必低速（`speed_limit≤0.5`）、空载、有人守护、急停可达。

---

## 6. 配置详解

### 6.1 `config/joint_map.yaml`（关节 → 电机 ID）

| 分组 | 关节 → 电机 ID |
| --- | --- |
| head | 1=`head_roll`(歪头) 2=`head_pitch`(抬头) 3=`head_yaw`(转头) |
| arm(左) | 11=肩pitch 12=肩roll 13=肩yaw 14=肘pitch 15=肘yaw 16=腕pitch 17=腕roll |
| arm(右) | 21=肩pitch 22=肩roll 23=肩yaw 24=肘pitch 25=肘yaw 26=腕pitch 27=腕roll |
| waist | 31=`waist_yaw` 32=`waist_pitch` |

- `groups` 的**顺序即 ctrl 数组顺序，不可调换**；`limits` 与真机关节行程逐项一致。
- 启动时 `_load_joint_map` 校验：分组合法（不含 leg）、关节不重复、`limits` 齐全，否则抛异常拒绝启动。

### 6.2 `config/motions.yaml`（动作关键帧表）

| 键 | 内容 |
| --- | --- |
| `home` | 中立待机姿态（**全 19 个关节 = 0.00**），动作起止位姿，衔接不跳变 |
| `motions` | 动作名 → `{duration: 名义时长(s), keyframes: [{t, positions}]}` |

**关键帧语义**：

- 写了 `positions` → 与 `home` **合并**（未写的关节取 `home`）；
- 不写 `positions` → **保持上一帧**（用于「停留一拍」）；
- 起始帧 `t=0` 的 `positions` 仅占位，**实机起点取 `/robot_state` 实测角**（平滑、不跳变）。

**动作一览（本表实际存在的 8 个）**：

| 动作名 | 实际时长 | 语义 |
| --- | --- | --- |
| `greet_open_arms` | 3.2 s | 原地抬手迎宾（双臂平缓外张，讲稿第 1 段） |
| `guide_pose` | 5.6 s | 轻抬引导姿态（右臂引导手，讲稿第 2/6 段） |
| `point_side_right` | 4.9 s | 水平侧指指引（右臂水平侧指 + 腰/头转向，第 3 段） |
| `point_left` | 5.6 s | 左臂水平前指（第 4 段签到指引） |
| `welcome_present` | 5.6 s | 单臂侧前摊掌介绍（第 5 段回转面向来宾） |
| `gentle_bow` | 6.4 s | 轻缓致意礼（微躬身 + 双臂下压，第 7 段） |
| `point_right` | 5.6 s | 右臂前伸指引（头/腰同向，备用动作） |
| `idle_scan_loop` | 6.0 s | 待机扫视无缝循环（只动头/腰，供后台扫视直连，见 §7.2） |

> ⚠️ **时长以关键帧为准**：服务端取 `duration` 字段为名义时长，但若**末帧 `t` 超过 `duration`**，
> 则以末帧为准并打印告警（`motion_server._run`）。表中"实际时长"即末帧 `t`：
> 例如 `gentle_bow` 声明 `duration: 4.4`，但末帧 `t=6.4`，实际按 **6.4 s** 播放。

🔴 全部角度均已核对真机关节行程，且**全部不动腿**。
🔴 动作与讲稿段落一一对应；`salute_bow` / `wave_official` / `farewell_bow` / `photo_pose` / `idle_scan`
等旧动作已在动作表精简时移除，`motion:<动作名>` 与讲稿 `motion_name` 均须取自上表。

---

## 7. 参数说明

### 7.1 `launch/motion.launch.py` 暴露的参数（可直接 `key:=value` 覆盖）

| 参数 | 类型 | 默认 | 说明 |
| --- | --- | --- | --- |
| `motions_file` | string | `""` | 动作表；留空用 `share/greeting_body/config/motions.yaml` |
| `joint_map_file` | string | `""` | 关节映射；留空用 `share/greeting_body/config/joint_map.yaml` |
| `publish_rate_hz` | double | `50.0` | 关节帧下发频率 (Hz)，需 ≥10 |
| `speed_limit` | double | `0.7` | 速度上限（硬约束 ≤0.7） |
| `warmup_s` | double | `1.5` | 起步 warmup 时长 (s) |
| `warmup_speed` | double | `0.3` | warmup 期间位置模式（头/腰）柔和速度上限 (rad/s) |
| `warmup_current_start` | double | `1.0` | warmup 首帧电流比例 (0~1) |
| `warmup_kp_start` | double | `0.6` | warmup 首帧臂 Kp 比例 (0~1)，ramp 到 1.0（仅 mode=1） |
| `hold_s` | double | `0.5` | 收尾回中立后的保持时长 (s) |
| `arm_control_mode` | int | `0` | `0`=位置（默认）/ `1`=力位混合（回退实验） |
| `max_follow_err_rad` | double | `0.10` | 臂期望相对实测的最大超前量 (rad) |
| `arm_kp_scale` | double | `1.0` | 【仅 `arm_control_mode:=1`】臂 Kp 缩放 |
| `arm_kd_scale` | double | `1.0` | 【仅 `arm_control_mode:=1`】臂 Kd 缩放 |
| `arm_feedforward_scale` | double | `1.0` | 前馈速度缩放（设 0 可定位是否前馈驱动） |
| `required_groups` | string[] | `['arm']` | 硬预检通道分组 |
| `arm_current` | double | `5.0` | 手臂电流上限 (A) |
| `head_speed` / `waist_speed` | double | `0.5` | 头 / 腰位置模式「静止(保持)段」速度上限 (rad/s) |
| `head_current` / `waist_current` | double | `5.0` | 头 / 腰电流上限 (A) |
| `feedforward_speed_margin` | double | `1.2` | 运动段 `spd = |轨迹瞬时速度| × margin` 余量（略 >1） |
| `min_feedforward_speed` | double | `0.05` | 位置模式 `spd` 绝对下限 (rad/s) |
| `idle_scan_enabled` | bool | `true` | 无动作时是否自动循环待机扫视（`false`=整体关闭） |
| `idle_scan_motion` | string | `idle_scan_loop` | 待机扫视动作名（须为无缝循环动作） |
| `idle_scan_delay_s` | double | `3.0` | 最后一个动作结束后静止多久开始扫视 (s) |
| `idle_scan_speed_scale` | double | `0.5` | 待机扫视速度（会被钳到 ≤ `speed_limit`） |
| `idle_scan_pause_on_h_right` | bool | `true` | H 停在右档时暂停扫视（见 §7.2） |
| `idle_scan_pause_on_voice` | bool | `true` | F 上拨 + 长按 A 开启语音功能时暂停扫视并回 home（见 §7.3） |
| `voice_long_press_s` | double | `1.0` | A 长按判定阈值 (s)，须与平台 `bridge_config.long_press_thresholds.a` 对齐 |

节点内另有**未被 launch 暴露**的参数（可用 `--ros-args -p` 覆盖）：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `soft_limit_margin_rad` | `0.05` | 软限位收缩量 (rad) |
| `max_step_rad` | `0.2` | 单帧位置增量上限 (rad) |
| `arm_speed` | `1.0` | 臂位置模式「静止(保持)段」速度上限 (rad/s) |
| `robot_state_topic` | `/robot_state` | 关节反馈话题 |
| `idle_scan_entry_s` | `1.5` | 进入/退出扫视、语音回中立位的平滑过渡时长 (s) |
| `idle_scan_sbus_topic` | `/sbus_data/event` | 扫视门控监听的按键事件话题 |

### 7.2 待机扫视（idle scan）

无动作 goal 且静止 `idle_scan_delay_s`（默认 3 s）后，后台线程自动**无缝循环**播放
`idle_scan_loop`（只动头/腰两类关节，其余关节全程保持实测起点不动）：

- 进入扫视时用 smoothstep 在 `idle_scan_entry_s`（默认 1.5 s）内从实测姿态**软过渡**到循环起点；
- 循环体以 `local = (elapsed × speed) % nominal` 取模，起止姿态相同，**首尾无缝衔接**；
- **新 goal 一到立即让位**（`_idle_stop` + 动作互斥 `_idle_lock`），动作结束后重新计时；
- **H 停在右档**（平台「简单动作模式」直发关节、绕过本服务端）时暂停扫视，避免两条链路互抢关节；
- 参数 `idle_scan_enabled:=false` 可整体关闭。

> 🔴 待机扫视依赖平台的按键事件（`/sbus_data/event`）；缺 `bodyctrl_msgs` 时自动禁用 H 档/语音门控。

### 7.3 语音门控（F 上拨 + 长按 A）

平台侧无「语音功能是否开启」的状态话题，故本服务端**自行识别该按键组合**当作语音开关
（须 `idle_scan_pause_on_voice:=true`）：

- 前提：F 拨杆停在**上档**（`KEY_F_UP`）；松开/回中/下拨即撤销前提；
- 触发：F 上档时**长按 A**（按下到松开 `≥ voice_long_press_s`，默认 1.0 s）→ **切换**语音开关；
- **语音开启期间**：待机扫视让位，头/腰在 `idle_scan_entry_s` 内**平滑回 `home` 中立位并持续保持**
  （位置模式命令需连续下发才生效），直到语音关闭或正式动作/H 档抢占；
- **语音关闭**：立即放行扫视（无需再等 `idle_scan_delay_s`），从循环起点无缝续扫。

> ⚠️ `voice_long_press_s` 必须与平台 `bridge_config.yaml` 的 `long_press_thresholds.a` 一致，
> 否则平台与本服务端对「长按」判定不一致。
> 🔴 长按 A 同时也会被平台判定为 long（开关 lyre 语音交互），二者语义一致，属预期行为。

---

## 8. 启动与调用

### 8.1 前置条件（实机）

1. 已 `source /home/nvidia/xos/setup.bash`（提供 `ros2_bridge_msgs`）；
2. 整机控制已使能，**急停完全弹出、工作人员持急停就位**；
3. 确认控制通道有订阅者、`/robot_state` 有反馈：

```bash
ros2 topic info /arm/cmd      # Subscribers 数需 ≥1
ros2 topic info /head/cmd     # 若 enable_head 未开可能为 0
ros2 topic echo /robot_state --once
```

### 8.2 启动

```bash
# 默认启动（速度上限 0.7）
ros2 launch greeting_body motion.launch.py

# 实机调试：低速 + 低电流
ros2 launch greeting_body motion.launch.py speed_limit:=0.5 arm_current:=3.0

# 只需手臂（默认）：required_groups=['arm']；若头/腰通道未开，动作会跳过该部位并告警
ros2 launch greeting_body motion.launch.py speed_limit:=0.7 required_groups:="['arm','head','waist']"
```

> ⚠️ `:=` 与值之间**不能有空格**（`required_groups:="[...]"`），否则报 `malformed launch argument`。

启动日志应包含 `实机 PlayMotion 服务端就绪`、动作表路径与动作数量、下发频率、速度上限、臂增益缩放。

### 8.3 发送动作（调试）

```bash
ros2 action send_goal /greeting/play_motion greeting_interfaces/action/PlayMotion \
  "{motion_name: point_side_right, speed_scale: 0.5}" --feedback
```

预期：先 warmup（保持当前位姿约 1.5 s），再播放动作；实机时长 = 名义时长 / `speed_scale`
（如 `point_side_right` 名义 4.9 s，`speed_scale=0.7` → 实机 7.0 s），收尾回到 `home` 并保持 0.5 s。

### 8.4 编程调用（Action 客户端）

```python
from rclpy.action import ActionClient
from greeting_interfaces.action import PlayMotion

cli = ActionClient(node, PlayMotion, "/greeting/play_motion")
cli.wait_for_server(timeout_sec=3.0)

goal = PlayMotion.Goal()
goal.motion_name = "point_left"
goal.speed_scale = 0.5          # 🔴 ≤ 0.7
handle = cli.send_goal_async(goal).result()
result = handle.get_result_async().result()
print(result.result.success, result.result.message)
```

---

## 9. 验证与调试

```bash
# 服务端与动作列表
ros2 action list | grep play_motion
ros2 node info /greeting_body_motion_server

# 看实际下发的控制帧
ros2 topic echo /arm/cmd --once
ros2 topic hz /arm/cmd            # 播放期间应接近 publish_rate_hz

# 看关节反馈
ros2 topic echo /robot_state --once

# 重新构建（若新动作未生效）
colcon build --packages-select greeting_body --symlink-install && source install/setup.bash
```

**常见问题**

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 拒绝目标 `未知动作` | 动作名拼写错 / 不在本表 | 对照 `motions.yaml` 的 `motions` 键 |
| `控制通道未就绪` | `/arm/cmd` 等无订阅者 | 确认整机控制使能；检查 `enable_head` 是否开启 |
| `电机未就绪`（故障码 0x8130） | 电机通讯掉线，`pos` 恒为 0.0 假值 | 检查电机供电/通讯，重启控制桥 |
| 关节收到命令但不动 | 通道无订阅者或电机掉线 | 见上两行；`ros2 topic info` 查订阅者 |
| 动作首帧大幅跳变 | 起点取到假值 0.0 | 确认健康门禁生效、`/robot_state` 真实有效 |
| 运动中持续高频啸叫 | 用了 `arm_control_mode:=1` | 改回 `0`；或逐步下调 `arm_kp_scale` |
| 关节滞后追赶型抖动 | 位置模式速度上限过低 | 提高 `publish_rate_hz`（如 50）或确认前馈速度生效 |
| 动作在关键帧处顿挫 | （本包已用 smoothstep 规避） | 确认运行的是当前版本代码 |
| 实机时长远长于名义 | `speed_scale` 偏小 | 期望时长 = 名义 / `speed_scale` |
| 编译找不到 `ros2_bridge_msgs` | 未 source 实机平台环境 | `source /home/nvidia/xos/setup.bash` 后重建 |

---

## 10. 注意事项

- 🔴 **腿部电机永不下发**（契约 §7.1）；新增关节/动作时严禁把腿加入 `joint_map.yaml`。
- 🔴 `speed_scale` 在**编排层与本服务端各钳一次**（双层兜底，礼仪硬要求）。
- 🔴 动作表角度必须核对真机关节行程，并留 ≥0.05 rad 软限位余量。
- 🔴 每次动作前必须通过**通道预检 + 电机健康门禁**；否则拒绝执行。
- 🔴 `arm_control_mode` 默认 `0`（位置模式，对齐官方）；`1` 仅作回退实验，实测本机型啸叫。
- 🔴 避免多个控制发布者同时控制同一话题（transport / robot_control / motion_server 冲突会导致电机打架）。
- 🔴 本方案不能替代实机测试，最终效果需现场验证。

---

## 11. 相关文档

- [《接口契约冻结表》v1.0](../../接口契约冻结表.md) §5 / §7.1
- [《酒店迎宾项目规划方案》](../../酒店迎宾项目规划方案.md)
- [greeting_interfaces/README.md](../greeting_interfaces/README.md)（`PlayMotion` 接口定义）
- [greeting_orchestrator/README.md](../greeting_orchestrator/README.md)（动作的唯一调用方）