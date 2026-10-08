# greeting_interfaces

迎宾机器人自建接口包（消息 / 服务 / 动作）。

本包是《接口契约冻结表》v1.0 §5 的**代码载体**，是接口定义的**唯一事实来源**。
不包含任何可执行节点，只生成 ROS 2 的 `.msg` / `.srv` / `.action` 接口类型。

---

## 1. 包的定位与依赖关系

### 1.1 在整机链路中的位置

```
        ┌─────────────────────────────────────────────┐
        │            greeting_interfaces               │  ← 本包（纯接口定义）
        │  msg / srv / action，无节点、无逻辑           │
        └───────────────┬─────────────────────────────┘
                        │ exec_depend（被引用）
                        ▼
                greeting_orchestrator
                （唯一指挥者 / 状态机）
```

### 1.2 依赖一览

| 方向 | 包 / 依赖 | 说明 |
| --- | --- | --- |
| 构建工具 | `ament_cmake`、`rosidl_default_generators` | 生成接口代码 |
| 消息依赖 | `std_msgs` | 所有消息的 `Header` 字段 |
| 消息依赖 | `action_msgs` | Action 的 `GoalStatus` 等基础类型 |
| 运行时 | `rosidl_default_runtime` | 接口的运行时支持 |
| 被依赖方 | `greeting_orchestrator`、`greeting_body`、`greeting_nav`、`greeting_teleop` | 均 `exec_depend` 本包 |

🔴 本包**不依赖**任何其他自建包，处于依赖图的最底层，必须最先构建。

### 1.3 构建顺序

```bash
cd <工作空间>
colcon build --packages-select greeting_interfaces --symlink-install
```

> 后续包的构建依赖本包，因此若全量重建，`greeting_interfaces` 会自动排在最前。

---

## 2. 文件结构与职责

```
greeting_interfaces/
├── CMakeLists.txt        # rosidl_generate_interfaces 声明全部接口
├── package.xml           # 依赖声明；member_of_group = rosidl_interface_packages
├── msg/
│   ├── ConsoleCommand.msg    # T1 控制台/手柄指令
│   ├── GreetingEvent.msg     # T3 迎宾事件
│   ├── PersonDetection.msg   # T4 人员感知
│   ├── GreetingIntent.msg    # T5 语音意图
│   └── SubsystemHealth.msg   # T7 子系统健康
├── srv/
│   └── GreetingControl.srv   # S1/S2 控制与讲稿重载
└── action/
    ├── Speak.action          # A1 语音播报
    ├── NavigateTo.action     # A2 导航到点
    └── PlayMotion.action     # A3 动作播放
```

| 文件 | 类型 | 承载话题 / 服务 / 动作 | 责任 Owner |
| --- | --- | --- | --- |
| `msg/ConsoleCommand.msg` | 消息 | `/greeting/control_cmd` | P |
| `msg/GreetingEvent.msg` | 消息 | `/greeting/events` | C（perception 源）/ P |
| `msg/PersonDetection.msg` | 消息 | `/greeting/perception/person` | C |
| `msg/GreetingIntent.msg` | 消息 | `/greeting/intent` | B |
| `msg/SubsystemHealth.msg` | 消息 | `/greeting/health` | P |
| `srv/GreetingControl.srv` | 服务 | `/greeting/control`、`/greeting/reload_script` | P |
| `action/Speak.action` | 动作 | `/greeting/speak` | B |
| `action/NavigateTo.action` | 动作 | `/greeting/navigate_to` | A |
| `action/PlayMotion.action` | 动作 | `/greeting/play_motion` | A |

### CMakeLists.txt 关键片段

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConsoleCommand.msg"
  "msg/GreetingEvent.msg"
  "msg/PersonDetection.msg"
  "msg/GreetingIntent.msg"
  "msg/SubsystemHealth.msg"
  "srv/GreetingControl.srv"
  "action/Speak.action"
  "action/NavigateTo.action"
  "action/PlayMotion.action"
  DEPENDENCIES std_msgs
)
```

🔴 新增接口时必须同时：① 在 `rosidl_generate_interfaces` 登记；② 走《多人协作开发规范》§6.3 的契约变更四步流程。

---

## 3. 接口定义（冻结全文）

### 3.1 `msg/ConsoleCommand.msg` —— T1 控制台指令

```text
std_msgs/Header header
string cmd               # start | pause | resume | next | prev | goto | stop | manual
string arg               # goto 的段号等参数
string operator_id       # 操作人标识（追溯用）
```

| 字段 | 单位 | 取值 | 说明 |
| --- | --- | --- | --- |
| `cmd` | — | 枚举 | `start` 开始 / `pause` 暂停 / `resume` 恢复 / `next` 下一段 / `prev` 上一段 / `goto` 跳转 / `stop` 停止 / `manual` 人工接管 |
| `arg` | — | 字符串 | 仅 `goto` 使用（段号，如 `"2"`）；其余为空串 |
| `operator_id` | — | 字符串 | 操作人标识，用于追溯 |

**话题**：`/greeting/control_cmd`　**QoS**：`reliable` / depth 10 / `volatile`

### 3.2 `msg/GreetingEvent.msg` —— T3 迎宾事件

```text
std_msgs/Header header
uint8 EVENT_LEADER_ARRIVED = 1   # 辅助：领导就位提示（不自动接待）
uint8 EVENT_LEADER_LEAVE   = 2
uint8 EVENT_MANUAL_START   = 3
uint8 EVENT_END            = 4
uint8 event_type
float32 distance_m
string source            # perception | console | panel
```

| 字段 | 单位 | 取值 | 说明 |
| --- | --- | --- | --- |
| `event_type` | — | 枚举 | 见上方常量 |
| `distance_m` | m | ≥ 0 | 事件相关距离 |
| `source` | — | 枚举 | `perception` / `console` / `panel` |

🔴 `EVENT_LEADER_ARRIVED` **仅作提示**，不自动触发接待（契约 §5.1）。

**话题**：`/greeting/events`　**QoS**：`reliable` / depth 10 / `volatile`

### 3.3 `msg/PersonDetection.msg` —— T4 人员感知

```text
std_msgs/Header header
bool detected
float32 distance_m
float32 bearing_rad
int32 count
float32 confidence
```

| 字段 | 单位 | 范围 | 说明 |
| --- | --- | --- | --- |
| `detected` | — | bool | `false` 时后续距离/方位字段无效（置 0） |
| `distance_m` | m | ≥ 0 | 人员距离 |
| `bearing_rad` | rad | ±π | 相对机体正前方，**逆时针为正** |
| `count` | — | ≥ 0 | 人数 |
| `confidence` | — | 0.0~1.0 | 🔴 **仅本字段用于阈值判断**，下游不得反推距离 |

**话题**：`/greeting/perception/person`　**QoS**：`best_effort` / depth 5 / `volatile`　**频率**：~10 Hz

### 3.4 `msg/GreetingIntent.msg` —— T5 语音意图

```text
std_msgs/Header header
string raw_text
string intent            # ask_topic | greeting | farewell | unknown
string slot_json         # 例: {"topic":"innovation_chain"}
float32 confidence
```

| 字段 | 取值 | 说明 |
| --- | --- | --- |
| `raw_text` | — | 原始识别文本 |
| `intent` | 枚举 | `ask_topic` / `greeting` / `farewell` / `unknown` |
| `slot_json` | JSON 字符串 | 🔴 **只填槽位，不允许生成文案**（契约 §6） |
| `confidence` | 0.0~1.0 | 置信度 |

**话题**：`/greeting/intent`　**QoS**：`reliable` / depth 10 / `volatile`

### 3.5 `msg/SubsystemHealth.msg` —— T7 子系统健康

```text
std_msgs/Header header
bool robot_control_alive
bool arm_cmd_has_subscriber
bool chassis_rest_ok
bool audio_ok
bool script_version_ok
uint8 slam_status        # 0 正常 / 1 丢失 / 2 精度下降 / 3 异常 / 5 等待重定位 / 6 重定位中
uint8 dtc_level          # 0 OK / 1 INFO / 2 WARNING / 3 ERROR / 4 FATAL
string detail
```

| 字段 | 枚举 / 取值 | 说明 |
| --- | --- | --- |
| `slam_status` | 0/1/2/3/5/6 | 数值语义取自底盘 `nav_status` |
| `dtc_level` | 0 OK / 1 INFO / 2 WARNING / 3 ERROR / 4 FATAL | 🔴 ≥3 触发降级 |
| `detail` | 字符串 | 人类可读描述 |

**话题**：`/greeting/health`　**QoS**：`reliable` / depth 10 / `volatile`　**频率**：1 Hz

### 3.6 `srv/GreetingControl.srv` —— S1 / S2

```text
string cmd               # start | pause | resume | stop | next | prev | goto | manual
string arg
string operator_id
---
bool success
string message
string current_state     # 返回当前状态与讲稿段号
```

| 服务名 | 用途 |
| --- | --- |
| `/greeting/control` | 与话题同语义的控制入口 |
| `/greeting/reload_script` | 重载讲稿；🔴 必须校验审核版本号，不符则拒绝加载（契约 §3） |

### 3.7 `action/Speak.action` —— A1（服务端：B）

```text
# goal
string text
bool interrupt           # true = 打断当前播报
float32 resume_point     # 续播起始位置（0.0~1.0）
---
# result
bool success
string message
---
# feedback
float32 progress         # 0.0 ~ 1.0
string speaking_text
```

| 字段 | 范围 | 说明 |
| --- | --- | --- |
| `resume_point` | 0.0~1.0 | `0.0` 从头播；`>0` 从比例位置续播 |

### 3.8 `action/NavigateTo.action` —— A2（服务端：A）

```text
# goal
string waypoint          # 点位名，查 config/waypoints.yaml
float32 timeout_s
---
# result
bool success
string message
---
# feedback
string phase             # navigating | arrived | retrying | relocalizing
float32 distance_remaining_m
```

| 字段 | 单位 | 范围 | 说明 |
| --- | --- | --- | --- |
| `timeout_s` | s | > 0 | 超时返回失败，由编排层决定重试或原地接待 |
| `phase` | — | 枚举 | 反馈阶段 |

### 3.9 `action/PlayMotion.action` —— A3（服务端：A）

```text
# goal
string motion_name       # greet_open_arms | guide_pose | point_side_right | point_left
                         # | welcome_present | gentle_bow | point_right | idle_scan_loop
float32 speed_scale      # ★礼仪动作统一 ≤0.7，避免机械感与幅度过大
---
# result
bool success
string message
---
# feedback
float32 progress
```

| 字段 | 范围 | 说明 |
| --- | --- | --- |
| `motion_name` | 枚举 | 动作名，查 `greeting_body/config/motions.yaml` |
| `speed_scale` | ≤ 0.7 | 🔴 礼仪硬要求，**服务端需二次钳制** |

---

## 4. QoS 规划（冻结）

| 话题 | 可靠性 | 深度 | 持久性 | 理由 |
| --- | --- | --- | --- | --- |
| `/greeting/control_cmd` | `reliable` | 10 | `volatile` | 主持人指令绝不可丢 |
| `/greeting/panel_command` | `reliable` | 10 | `volatile` | 按键兜底 |
| `/greeting/events` | `reliable` | 10 | `volatile` | 事件不能丢 |
| `/greeting/health` | `reliable` | 10 | `volatile` | 状态快照需可靠 |
| `/greeting/state` | `reliable` | 1 | 🔴 `transient_local` | 控制台接入即拿到当前状态 |
| `/greeting/perception/person` | `best_effort` | 5 | `volatile` | 辅助感知，丢帧无妨 |
| `/greeting/intent` | `reliable` | 10 | `volatile` | 意图不能丢 |
| 全部 `/greeting/*` Action | 沿用 rclpy/rclcpp 默认 | — | — | 🔴 服务端与客户端不得自定义覆盖 |

🔴 **所有自建话题的订阅端必须显式声明 QoS**，禁止依赖默认值（跨主机 DDS QoS 不匹配的表现是"话题存在但收不到数据"）。

---

## 5. 模块调用方法

### 5.1 验证接口已被正确生成

```bash
source install/setup.bash
ros2 interface show greeting_interfaces/msg/ConsoleCommand
ros2 interface show greeting_interfaces/msg/PersonDetection
ros2 interface show greeting_interfaces/msg/GreetingIntent
ros2 interface show greeting_interfaces/msg/SubsystemHealth
ros2 interface show greeting_interfaces/msg/GreetingEvent
ros2 interface show greeting_interfaces/srv/GreetingControl
ros2 interface show greeting_interfaces/action/Speak
ros2 interface show greeting_interfaces/action/NavigateTo
ros2 interface show greeting_interfaces/action/PlayMotion
```

### 5.2 Python 使用示例

**发布消息（`ConsoleCommand`）**

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from greeting_interfaces.msg import ConsoleCommand

rclpy.init()
node = Node("cmd_pub")
pub = node.create_publisher(
    ConsoleCommand, "/greeting/control_cmd",
    QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
)
msg = ConsoleCommand()
msg.cmd = "start"
msg.arg = ""
msg.operator_id = "operator-A"
pub.publish(msg)
rclpy.spin_once(node, timeout_sec=0.5)
rclpy.shutdown()
```

**订阅消息（`SubsystemHealth`）**

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy
from greeting_interfaces.msg import SubsystemHealth

def cb(msg):
    print(f"dtc_level={msg.dtc_level} chassis_rest_ok={msg.chassis_rest_ok}")

node.create_subscription(
    SubsystemHealth, "/greeting/health", cb,
    QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
)
```

**调用服务（`GreetingControl`）**

```python
from greeting_interfaces.srv import GreetingControl

cli = node.create_client(GreetingControl, "/greeting/control")
cli.wait_for_service(timeout_sec=3.0)
req = GreetingControl.Request()
req.cmd = "goto"
req.arg = "2"
req.operator_id = "operator-A"
resp = cli.call(request=req)     # 或 cli.call_async(req)
print(resp.success, resp.message, resp.current_state)
```

**调用动作（`NavigateTo`）**

```python
from rclpy.action import ActionClient
from greeting_interfaces.action import NavigateTo

cli = ActionClient(node, NavigateTo, "/greeting/navigate_to")
cli.wait_for_server(timeout_sec=3.0)

goal = NavigateTo.Goal()
goal.waypoint = "desk"
goal.timeout_s = 60.0
handle = cli.send_goal_async(goal).result()      # 简化写法；生产环境用 future 轮询
result = handle.get_result_async().result()
print(result.result.success, result.result.message)
```

### 5.3 CLI 使用示例

```bash
# 触发接待
ros2 topic pub --once /greeting/control_cmd greeting_interfaces/msg/ConsoleCommand \
  "{cmd: 'start', arg: '', operator_id: 'cli'}"

# 查看系统健康
ros2 topic echo /greeting/health

# 调用控制服务
ros2 service call /greeting/control greeting_interfaces/srv/GreetingControl \
  "{cmd: 'goto', arg: '2', operator_id: 'cli'}"

# 发送导航动作目标
ros2 action send_goal /greeting/navigate_to greeting_interfaces/action/NavigateTo \
  "{waypoint: 'desk', timeout_s: 60.0}"
```

---

## 6. 注意事项

- 🔴 本包是**接口名 + 字段（含顺序）+ QoS + 语义**的冻结对象；任何变更必须走契约变更四步流程，并在《接口契约冻结表》§9 追加记录。
- 🔴 `Header.frame_id` 必须填 `""` 或有效坐标系名，**不填垃圾值**。
- 🔴 `PersonDetection.confidence` 是唯一的阈值判断字段；`PlayMotion.speed_scale` 服务端必须二次钳制。
- 各包内部实现（函数、类、参数默认值）**不在冻结范围内**。

---

## 7. 相关文档

- [《接口契约冻结表》v1.0](../../接口契约冻结表.md)
- [《酒店迎宾项目规划方案》](../../酒店迎宾项目规划方案.md)