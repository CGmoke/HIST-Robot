# greeting_ws —— 天轶 2.5 迎宾机器人 ROS 2 工作空间

酒店迎宾场景的 ROS 2 **实机**工作空间：以天轶 2.5 人形机器人为本体，
在真机上运行「迎宾 / 引导 / 送别」全流程，并按接口契约与平台本体、底盘对接。

- **ROS 版本**：ROS 2 Jazzy
- **本体**：天轶 2.5 实机（上半身头 / 双臂 / 腰 + 移动底盘）
- **导航**：底盘 REST `:9090`（knewbots 固件），固定范围内短距移位
- **构建**：colcon
- **语言**：Python 3（`rclpy`）

---

## 1. 快速开始

```bash
# 0) 进入工作空间
cd ~/greeting_ws

# 1) 前置：source 实机平台环境（提供 ros2_bridge_msgs / bodyctrl_msgs 等）
source /opt/ros/jazzy/setup.bash
source /home/nvidia/xos/setup.bash

# 2) 构建（首次或改动后）
colcon build --symlink-install

# 3) source 工作空间（每个新终端都要执行）
source install/setup.bash

# 4) 一键拉起整条迎宾链路（动作服务端 + 手柄网关 + 两个桥 + 语音 + 编排层）
ros2 launch greeting_teleop greeting_bringup.launch.py

# 5) 另开终端触发接待
ros2 topic pub --once /greeting/control_cmd greeting_interfaces/msg/ConsoleCommand "{cmd: 'start'}"
ros2 topic echo /greeting/state
```

> 🔴 实机安全：首次调试务必低速（`speed_limit≤0.5`）、空载、有人守护、急停可达。

---

## 2. 目录结构

```
greeting_ws/
├── src/
│   ├── greeting_interfaces/     # ① 接口（msg/srv/action），依赖图最底层
│   ├── greeting_body/           # ② 实机上半身动作服务端（/greeting/play_motion）
│   ├── greeting_nav/            # ③ 实机底盘 REST 导航服务端（/greeting/navigate_to）
│   ├── greeting_voice/          # ④ 语音（/greeting/speak + 遥控 TTS 播报）
│   ├── greeting_teleop/         # ⑤ 手柄（SBUS）按键网关 + 动作/流程命令桥
│   └── greeting_orchestrator/   # ⑥ 迎宾状态机（唯一指挥者）
├── 多人协作开发规范.md
├── 接口契约冻结表.md
└── 酒店迎宾项目规划方案.md
```

---

## 3. 包一览与依赖分层

| # | 包 | 类型 | 职责 | 依赖 |
| --- | --- | --- | --- | --- |
| ① | `greeting_interfaces` | `ament_cmake` | 9 个自建接口（5 msg / 1 srv / 3 action） | `std_msgs`、`action_msgs` |
| ② | `greeting_body` | `ament_python` | 实机上半身动作服务端，≥10 Hz 流式下发关节帧 | `greeting_interfaces`、`ros2_bridge_msgs` |
| ③ | `greeting_nav` | `ament_python` | 实机底盘 REST 导航服务端 | `greeting_interfaces`、`python3-requests` |
| ④ | `greeting_voice` | `ament_python` | `/greeting/speak` 动作服务端 + 遥控 TTS 播报 | `greeting_interfaces`、`bodyctrl_msgs`、`interaction_msgs` |
| ⑤ | `greeting_teleop` | `ament_python` | 按键网关 + 动作直连桥 + 流程命令桥 | `greeting_interfaces`、`bodyctrl_msgs` |
| ⑥ | `greeting_orchestrator` | `ament_python` | 状态机，按讲稿驱动三个 Action | `greeting_interfaces` |

### 依赖与调用层级

```
greeting_interfaces ──（所有包的接口依赖，最底层）
        │
        ├──► greeting_body / greeting_nav / greeting_voice
        │
        ├──► greeting_teleop ──运行时 include──► greeting_body、greeting_orchestrator
        │
        └──► greeting_orchestrator（唯一指挥者）
                    │ 调用三个 Action
                    ├─► /greeting/speak         → greeting_voice   （本体 TTS）
                    ├─► /greeting/navigate_to   → greeting_nav     （底盘 REST :9090）
                    └─► /greeting/play_motion   → greeting_body    （实机电机控制帧）
```

- 构建顺序由依赖自动决定；`--symlink-install` 便于改 Python/配置后免重建。
- 🔴 `greeting_teleop/greeting_bringup.launch.py` 在**运行时** include `greeting_body/motion.launch.py`
  与 `greeting_orchestrator/orchestrator.launch.py`，该两处依赖**不在 `package.xml` 声明**；
  缺失时报 "package not found"，按提示构建即可。

---

## 4. 功能架构与数据流

```
                      ┌───────────────────────────────┐
                      │     greeting_orchestrator     │  唯一指挥者 / 唯一状态机
                      └───┬──────────┬──────────┬─────┘
              Action 客户端 │          │          │
                          ▼          ▼          ▼
                  /greeting/speak  /greeting/navigate_to  /greeting/play_motion
                          │          │          │
              ┌───────────┘          │          └────────────┐
              ▼                      ▼                       ▼
       greeting_voice           greeting_nav            greeting_body
       （本体 TTS 播报）      （底盘 REST :9090）      （流式关节帧 → 电机）
                                     │                       │
                                     ▼                       ▼
                          knewbots 底盘固件      /head|/arm|/waist/cmd
```

**底盘**：`greeting_nav` → 底盘 REST `http://192.168.11.10:9090`（fire-and-forget 下发 + 自行轮询位姿到点）
**上半身**：`greeting_body` → `/head/cmd` + `/arm/cmd` + `/waist/cmd`（≥10 Hz 连续流式下发，缺一不可）

> 🔴 契约 §9 冻结「底盘与本体两套 TF 不连通」（实机形态）。
> 🔴 腿部电机永不下发（契约 §7.1）；动作 `speed_scale ≤ 0.7`（编排层与服务端各钳一次）。

---

## 5. 常用启动入口

| 命令 | 作用 | 归属包 |
| --- | --- | --- |
| `ros2 launch greeting_teleop greeting_bringup.launch.py` | **一键全链路**：动作服务端 + 手柄网关 + 两个桥 + 语音 + 编排层 | greeting_teleop |
| `ros2 launch greeting_body motion.launch.py [speed_limit:=0.5]` | 实机上半身动作服务端（`/greeting/play_motion`） | greeting_body |
| `ros2 launch greeting_nav navigate_to.launch.py [dry_run:=true]` | 实机底盘 REST 导航服务端（`/greeting/navigate_to`） | greeting_nav |
| `ros2 launch greeting_voice voice_greet.launch.py [dry_run:=true]` | 遥控按键语音播报（TTS） | greeting_voice |
| `ros2 launch greeting_teleop teleop.launch.py [monitor:=true]` | 仅手柄按键网关（按键标定） | greeting_teleop |
| `ros2 launch greeting_orchestrator orchestrator.launch.py` | 迎宾状态机（单独启动） | greeting_orchestrator |

### 5.1 运行期操作

```bash
# 状态机状态
ros2 topic echo /greeting/state

# 控制指令：start / pause / resume / next / prev / goto / stop / manual
ros2 topic pub --once /greeting/control_cmd greeting_interfaces/msg/ConsoleCommand "{cmd: 'goto', arg: '2'}"

# 系统健康（三个布尔反映三个 Action 服务端在线情况）
ros2 topic echo /greeting/health
```

---

## 6. 环境要求与一次性准备

| 项 | 要求 |
| --- | --- |
| 系统 | Ubuntu 24.04 |
| ROS | ROS 2 Jazzy（`/opt/ros/jazzy`） |
| 实机平台 | `/home/nvidia/xos`（提供 `ros2_bridge_msgs` / `bodyctrl_msgs` / `interaction_msgs`），编译前需 `source` |
| 底盘 | knewbots 固件 REST API，默认 `http://192.168.11.10:9090` |
| Python 依赖 | `python3-requests`（底盘 REST 客户端）、`python3-yaml`（配置解析） |

**环境变量提示**

```bash
# 若 RSP 因日志目录权限拒绝启动，先改日志目录
export ROS_LOG_DIR=$HOME/.ros/log

# 编译/运行前必须 source 实机平台环境（提供 ros2_bridge_msgs / bodyctrl_msgs）
source /home/nvidia/xos/setup.bash
```

---

## 7. 常见问题

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `Package 'xxx' not found` | 未构建或未 source | `colcon build` + `source install/setup.bash` |
| 编译找不到 `ros2_bridge_msgs` / `bodyctrl_msgs` | 未 source 实机平台环境 | `source /home/nvidia/xos/setup.bash` 后重建 |
| 状态机停在 `IDLE` | 未下发 `start` | 发 `/greeting/control_cmd` |
| 提示"动作服务端不在线" | 动作 / 语音服务端未起 | 启动对应服务端（`greeting_bringup.launch.py` 已含） |
| 动作被拒绝：`控制通道未就绪` | `/arm/cmd` 无订阅者（平台自检未完成） | 按手柄 `A` 完成自检，确认整机控制使能 |
| `底盘 REST 不可达` | 网络 / IP 白名单 / 固件未运行 | 用 `curl` 验证；检查 `host` |
| 导航一直超时 | 距离过远 / 障碍 / 速度过慢 | 提高 `timeout_s`；检查路径与限速 |

---

## 8. 文档索引

### 项目级文档

| 文档 | 内容 |
| --- | --- |
| [接口契约冻结表.md](./接口契约冻结表.md) | **唯一事实来源**：接口名/字段/QoS/语义/状态机（改名或加字段须走契约变更流程） |
| [酒店迎宾项目规划方案.md](./酒店迎宾项目规划方案.md) | 场景、系统架构、阶段规划 |
| [多人协作开发规范.md](./多人协作开发规范.md) | 分工、协作流程、契约变更四步 |

### 各包 README

| 包 | 文档 |
| --- | --- |
| `greeting_interfaces` | [README](./src/greeting_interfaces/README.md) |
| `greeting_body` | [README](./src/greeting_body/README.md) |
| `greeting_nav` | [README](./src/greeting_nav/README.md) |
| `greeting_voice` | [README](./src/greeting_voice/README.md) |
| `greeting_teleop` | [README](./src/greeting_teleop/README.md) |
| `greeting_orchestrator` | [README](./src/greeting_orchestrator/README.md) |

---

## 9. 开发约定

- 🔴 **接口是冻结对象**：字段顺序、QoS、语义变更必须同步更新 `greeting_interfaces` 与《接口契约冻结表》。
- 🔴 **单一指挥者**：只有 `greeting_orchestrator` 调用三个 Action，模块之间禁止互调。
- 🔴 **腿部电机永不下发**（契约 §7.1）；迎宾场景禁腿部动作。
- 参数优先落在 YAML：`greeting_body/config/*.yaml`（动作/关节映射）、`greeting_nav/config/waypoints.yaml`（点位）、
  `greeting_orchestrator/config/greeting_script.yaml`（讲稿）是各模块的唯一真源。
- 礼仪硬要求：动作 `speed_scale ≤ 0.7`（编排层与服务端各钳一次）。
