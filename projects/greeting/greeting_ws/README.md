# greeting_ws —— 酒店迎宾机器人操作指南（实机）

天轶 2.5 迎宾机器人工作空间。本指南面向《大会流程》的**实机**运行，
从编译到触发完整讲解流程、观察动作与语音的提前量，覆盖启动 / 运行 / 调试 / 清理全流程。

> 讲稿剧本：[greeting_orchestrator/config/greeting_script.yaml](src/greeting_orchestrator/config/greeting_script.yaml)

> 台词原文：[大会流程.md](大会流程.md)　

>动作库：[greeting_body/config/motions.yaml](src/greeting_body/config/motions.yaml)

---

## 0. 前置条件

| 项 | 要求 |
| --- | --- |
| 系统 | Ubuntu 24.04（x86_64 原生，**不要** source 厂商 aarch64 打包栈的 `setup.bash`） |
| ROS | ROS 2 Jazzy（`/opt/ros/jazzy/setup.bash`） |
| 实机平台 | `/home/nvidia/xos`（提供 `bodyctrl_msgs` / `ros2_bridge_msgs` / `interaction_msgs`），编译前需 `source` |
| 底盘 | knewbots 固件 REST API，默认 `http://192.168.11.10:9090` |

一次性依赖（需 sudo 密码，终端执行）：

```bash
# 底盘 REST 客户端 + 配置解析（多数系统已自带）
sudo apt install -y python3-requests python3-yaml
```

---

## 1. 首次准备：编译工作空间

```bash
# ROS 日志目录先改到 /tmp，避免 RSP 因权限被拒启动
export ROS_LOG_DIR=/tmp/ros_log ROS_HOME=/tmp/ros_home

source /opt/ros/jazzy/setup.bash
source /home/nvidia/xos/setup.bash          # 平台消息包（缺则 greeting_voice/teleop/body 编译失败）

cd /home/nvidia/greeting-body-motion/greeting_ws
colcon build --symlink-install              # 首次全量；之后改单包用 --packages-select
source install/setup.bash
```

编译通过后自检：

```bash
ros2 pkg list | grep greeting
# → greeting_interfaces greeting_orchestrator greeting_body greeting_voice greeting_teleop greeting_nav
```

> ⚠️ `--symlink-install` 下改 `config/*.yaml`（讲稿、动作表）**无需重新编译**；
> 但节点只在启动时读一次配置：讲稿可调 `/greeting/reload_script` 热重载，
> 动作表 / 按键映射则需重启服务，见 [§2.1 开机自启与重启](#21-开机自启与重启systemd-服务)。

---

## 2. 一键启动实机全链路

```bash
source /opt/ros/jazzy/setup.bash && source /home/nvidia/xos/setup.bash && source install/setup.bash
ros2 launch greeting_teleop greeting_bringup.launch.py
```

该命令依次拉起：

| 顺序 | 组件 | 内容 |
| --- | --- | --- |
| 1 | `greeting_body/motion.launch.py` | 实机上半身动作 `PlayMotion` 服务端（`/greeting/play_motion`） |
| 2 | `greeting_teleop/teleop.launch.py` | 遥控器 SBUS 按键网关（`/sbus_data/event` → `/greeting/panel_command`） |
| 3 | `greeting_teleop/panel_command_bridge` | 动作直连桥（`/greeting/panel_command` → `/greeting/play_motion`） |
| 4 | `greeting_teleop/flow_command_bridge` | 流程命令桥（`/greeting/panel_command` → `/greeting/control_cmd`） |
| 5 | `greeting_voice` `speak_action_server` | `/greeting/speak` 动作服务端（编排层语音播报所需） |
| 6 | `greeting_voice` `simple_action_voice_node` | 平台「简单动作模式」语音播报 |
| 7 | `greeting_orchestrator/orchestrator.launch.py` | 迎宾状态机（唯一指挥者） |

可选参数：

```bash
ros2 launch greeting_teleop greeting_bringup.launch.py speed_limit:=0.5          # 实机调试低速
ros2 launch greeting_teleop greeting_bringup.launch.py monitor:=true            # 仅标定按键
ros2 launch greeting_teleop greeting_bringup.launch.py with_orchestrator:=false  # 只用动作快捷键
```

启动成功标志（日志中应能看到）：
- `实机 PlayMotion 服务端就绪`，动作表 **8 个动作**（`greet_open_arms` / `guide_pose` / `point_side_right` /
  `point_left` / `welcome_present` / `gentle_bow` / `point_right` / `idle_scan_loop`）；
- 编排层就绪：讲稿 7 段，等待 `/greeting/control_cmd` 下发 `start`。

### 2.1 开机自启与重启（systemd 服务）

实机已把上面的 `greeting_bringup.launch.py` 注册为 systemd 服务 **`greeting-autostart.service`**
（开机自启；主进程退出后由 systemd 自动重新拉起）。因此**日常不手动跑 launch，改用 `systemctl` 管理**：

```bash
sudo systemctl restart greeting-autostart.service   # 重启全链路（改配置后最常用）
sudo systemctl status  greeting-autostart.service   # 查看运行状态
sudo systemctl stop    greeting-autostart.service   # 停止（下次调试再手动 start）
sudo systemctl start   greeting-autostart.service   # 启动

journalctl -u greeting-autostart -f                 # 实时看日志（动作/编排层打印都在这里）
```

**🔴 什么时候必须 `restart`？**

`--symlink-install` 只保证 `src/` 里的 YAML 立刻反映到 `install/`（**无需 `colcon build`**），
但**多数节点只在 `__init__` 里读一次配置、没有热重载**，所以：

| 改动内容 | 是否需要 `restart` |
| --- | --- |
| `greeting_body/config/motions.yaml`（动作关键帧） | ✅ 需要（`MotionLibrary` 启动时载入一次） |
| `greeting_teleop/config/teleop_map.yaml`（按键映射） | ✅ 需要（`joy_mapper` 启动时载入一次） |
| `greeting_body/config/joint_map.yaml`、launch 参数（如 `speed_limit`） | ✅ 需要（参数在 `__init__` 中读取，`ros2 param set` 无效） |
| `greeting_orchestrator/config/greeting_script.yaml`（讲稿） | ❌ 不用，调 `/greeting/reload_script` 热重载即可（见 §5） |
| `.py` / launch 代码 | 需先 `colcon build`，再 `restart` |

改完配置后的标准动作：

```bash
sudo systemctl restart greeting-autostart.service
journalctl -u greeting-autostart -f | grep "播放动作"    # 例如确认：名义 7.2s -> 实际 12.0s
```

> 🔴 **不要用 `sudo` 去手动启动节点**：`sudo` 默认不保留 `LD_LIBRARY_PATH`，
> 平台库（`libxfault.so` 等）会加载失败。手动调试请在**普通用户终端**里
> `source /opt/ros/jazzy/setup.bash && source /home/nvidia/xos/setup.bash && source install/setup.bash`
> 后再跑 `ros2 launch`。

---

## 3. 运行《大会流程》

另开一个终端（同样 source 两处 setup），触发接待：

```bash
source /opt/ros/jazzy/setup.bash && source /home/nvidia/xos/setup.bash && source install/setup.bash

# 开始接待（从第 1 段起，按讲稿顺序走完 7 段）
ros2 topic pub --once /greeting/control_cmd greeting_interfaces/msg/ConsoleCommand \
  "{cmd: 'start', arg: '', operator_id: 'cli'}"

# 观察状态机推进
ros2 topic echo /greeting/state
```

期望效果：
- 机器人逐段执行「**动作先行 + 语音跟随**」，同一步内动作与语音**并行**；
- 同一步内**动作比语音提前的时长由讲稿 `cue.lead_s` 逐段决定**（第 1、7 段 `1.0 s`；
  第 4~6 段 `3.0 s`；第 2、3 段 `0.0 s`，即动作与语音同时触发）；
- 一段内动作与语音都结束后才推进下一段，7 段走完回到 `IDLE`。

也可用服务触发：

```bash
ros2 service call /greeting/control greeting_interfaces/srv/GreetingControl \
  "{cmd: 'start', arg: '', operator_id: 'cli'}"
```

---

## 4. 讲稿 ↔ 动作 / 语音对照表

| 段 | 状态名 | 动作 | speed_scale | 语音提前量 lead_s | 内容 |
| --- | --- | --- | --- | --- | --- |
| 1 | `GREETING` | `greet_open_arms` | 0.5 | **1.0 s** | 开场欢迎（抬手迎宾手势） |
| 2 | `INTRO` | `guide_pose` | 0.6 | 0.0 s | 自我介绍（轻抬引导姿态，动作与语音同时起） |
| 3 | `EXPLAIN` | 先 `speak`（大会板块长文本）→ 再 `point_side_right` | 0.7 | 0.0 s | 大会板块讲解 + 指向金穗厅 |
| 4 | `CHECKIN` | `point_left` | 0.6 | 3.0 s | 指向签到处 |
| 5 | `WELCOME` | `welcome_present` | 0.6 | 3.0 s | 回转面向来宾 + 单臂摊掌介绍 |
| 6 | `GUIDE` | `guide_pose` | 0.6 | 3.0 s | 引路指引 |
| 7 | `FAREWELL` | `gentle_bow` | 0.6 | **1.0 s** | 离场致意礼（轻缓躬身） |

说明：
- 讲稿 `version: "1.1"`，必须与编排层参数 `approved_script_version`（默认 `1.1`）一致；
- **第 3 段有两个 step**：先纯语音播报大会板块介绍，再以 `cue` 起「侧指动作 + 一句语音」（`lead_s=0.0`）；
- `speed_scale` 编排层与服务端**各钳一次**（≤0.7，礼仪硬约束）；
- 动作关键帧不走腿部关节（腿部禁用，防倾倒）；
- 语音由 `greeting_voice` 的 `/greeting/speak` 包装本体 TTS 播报。

---

## 5. 控制指令

| `cmd` | 作用 | 参数 `arg` |
| --- | --- | --- |
| `start` | 从第 1 段开始接待 | — |
| `pause` | 暂停当前段 | — |
| `resume` | 恢复 | — |
| `next` / `prev` | 跳到下一段 / 上一段（**连播**，继续往下走） | — |
| `goto` | **单段点播**指定段（动作 + 语音同步，播完回 `IDLE`，**不续播**） | 段号，如 `2` |
| `stop` | 中止当前动作，回 `IDLE` | — |
| `manual` | 中止并进入人工接管 | — |

> 🔴 `goto`（单段点播）与 `start`/`next`/`prev`（连播）的区别：`goto:3` 只播第 3 段，
> 语音播完即回到 `IDLE`，**不会自动播第 4 段**；`start` 则从第 1 段一路走完 7 段。
> 遥控器"一段一个按键"用的就是 `goto`（见 §5.1）。

```bash
# 单段点播示例（只播第 3 段，播完回 IDLE）
ros2 topic pub --once /greeting/control_cmd greeting_interfaces/msg/ConsoleCommand \
  "{cmd: 'goto', arg: '3', operator_id: 'cli'}"

# 重载讲稿（校验审核版本号，不符则拒绝）
ros2 service call /greeting/reload_script greeting_interfaces/srv/GreetingControl \
  "{cmd: '', arg: '', operator_id: 'cli'}"
```

### 5.1 手柄 / 遥控器按键触发动作

> 完整说明见 [greeting_teleop/README.md](src/greeting_teleop/README.md) §5 / §8；
> 映射表本体在 [teleop_map.yaml](src/greeting_teleop/config/teleop_map.yaml)。

链路（两条下游）：

```
遥控器按键 → /sbus_data/event → (joy_mapper) → /greeting/panel_command
   ├→ (panel_command_bridge)  → /greeting/play_motion → 动作服务端        # 单个动作
   └→ (flow_command_bridge)   → /greeting/control_cmd → orchestrator 讲稿  # 整段大会流程
```

**★ 急停**—— 常驻键，无需前提：

| 按键 | 命令 | 效果 |
| --- | --- | --- |
| `H 右` | `stop` | 停止并回到 `IDLE`（安全键） |

> 整段连播 `start`（第 1 段起自动走完 7 段）**当前未绑定常驻按键**（`D` 已改作分段键）；
> 如需保留，可在 [teleop_map.yaml](src/greeting_teleop/config/teleop_map.yaml) 的 `keys:` 另择一键置为 `start`（如 `f_up: start`）。

**★★ 分段点播（一段一键，动作 + 语音同步）**—— 组合键：**先把 E 三档开关拨到「上」或「下」选组，再按 A/B/C**；
每按一次只播该段，播完自动回 `IDLE`，**不续播下一段**（命令为 `goto:<段号>`）：

| 前提（开关档位） | 按 `A` | 按 `B` | 按 `C` |
| --- | --- | --- | --- |
| `E` 拨「上」 | 第 1 段 `goto:1` 欢迎致辞 + 迎宾手势 | 第 2 段 `goto:2` 自我介绍 + 引导姿态 | 第 3 段 `goto:3` 大会板块 + 侧指金穗厅 |
| `E` 拨「下」 | 第 4 段 `goto:4` 签到指引 + 左臂指向 | 第 5 段 `goto:5` 资源聚合 + 单臂摊掌 | 第 6 段 `goto:6` 引路指引 + 引导姿态 |

- 组合键前提**默认不生效**：上电后必须先拨动 `E` 开关才激活（安全设计）；
- ⚠️ `E` 拨「中」无对应分段组，按 A/B/C 不触发段号（回退到常驻表）；
- ⚠️ 第 7 段（离场致意 `gentle_bow`）**未绑定分段键**，可用下方命令注入或 `goto:7`。

> 段号与 [greeting_script.yaml](src/greeting_orchestrator/config/greeting_script.yaml) 的 `id` 一一对应。
> 键位可在 [teleop_map.yaml](src/greeting_teleop/config/teleop_map.yaml) 的 `guards:` 段自行调整（改 YAML 后 `--symlink-install` 无需重编译，重启节点生效）。

- ⚠️ **`F` 上拨 + 长按 `A`** = 切换「语音功能」：暂停待机扫视并平滑回中立位（在 `motion_server` 内识别，详见
  [greeting_body/README.md](src/greeting_body/README.md) §7.3）；A 已用于本表，请避免在非 `F` 档下长按；
- 摇杆 `sticks`、Joy 轴 `axes` 默认**未启用**，按需在 YAML 中取消注释；
- 命令词表：`start | pause | resume | next | prev | goto:<段号> | stop | manual | motion:<动作名>`，
  一个键可发多条，用 `;` 连接（如 `"stop;motion:greet_open_arms"`）。

```bash
# 一键启动（实机）：动作服务端 + 手柄网关 + 两个桥 + 语音 + 编排层
ros2 launch greeting_teleop greeting_bringup.launch.py
ros2 launch greeting_teleop greeting_bringup.launch.py with_orchestrator:=false  # 只用动作快捷键

# 标定模式：只打印按键/摇杆事件，不发布命令（先确认映射无误）
ros2 launch greeting_teleop teleop.launch.py monitor:=true
```

> 🔴 全流程命令（`start` / `goto:<段号>`）需要 `greeting_orchestrator` 在运行才有消费方（`greeting_bringup.launch.py` 默认已包含）。

**按键触发语音（TTS，`greeting_voice`）**：`greeting_bringup.launch.py` 只起
`simple_action_voice_node`（`H` 右拨 + 连按 `B`×N + 短按 `A` → 播第 N 句，配合平台「简单动作模式」）；
`voice_greet_tts_node`（`G` 左/右拨 + `A/B/C/D` → 播对应文本）需单独启动。详见
[greeting_voice/README.md](src/greeting_voice/README.md)。

> ⚠️ 以上按键链路依赖平台 SBUS 事件（`/sbus_data/event`），属于实机功能；
> 也可不经手柄，在终端直接注入命令验证桥接（需先启动对应节点）：
> ```bash
> ros2 topic pub --once /greeting/panel_command std_msgs/msg/String "{data: 'start'}"          # 整段连播（走完 7 段）
> ros2 topic pub --once /greeting/panel_command std_msgs/msg/String "{data: 'goto:3'}"         # 单段点播第 3 段（等同于 E 拨上 + C）
> ros2 topic pub --once /greeting/panel_command std_msgs/msg/String "{data: 'motion:greet_open_arms'}"  # 单动作
> ```
> ⚠️ 实机按键触发动作的前提：平台 `robot_status == Running` 自检完成、`/arm/cmd` 已有订阅者，动作才可能被执行；上电前请确认急停可用。

---

## 6. 分步手动启动（调试用）

需要单独观察每个组件时，开 3 个终端分别启动（`greeting_bringup.launch.py` 的反向拆解）：

```bash
# 终端 A：实机上半身动作服务端（/greeting/play_motion）
ros2 launch greeting_body motion.launch.py

# 终端 B：实机底盘 REST 导航服务端（/greeting/navigate_to）
ros2 launch greeting_nav navigate_to.launch.py

# 终端 C：编排层（状态机）
ros2 launch greeting_orchestrator orchestrator.launch.py
```

> 需要遥控按键链路时，另起 `ros2 launch greeting_teleop teleop.launch.py`；
> 需要语音播报时，另起 `ros2 launch greeting_voice voice_greet.launch.py`。

> `greeting_body` 默认读取 `share/greeting_body/config/motions.yaml`（**已含讲稿用到的全部 8 个动作**），
> 因此**无需**再传 `-p motions_file:=...` 覆盖。

纯动作调试（不起编排层，直接单发动作）：

```bash
ros2 action send_goal /greeting/play_motion greeting_interfaces/action/PlayMotion \
  "{motion_name: 'gentle_bow', speed_scale: 0.6}"
```

---

## 7. 验证与调试

```bash
# 拓扑与在线服务端
ros2 node list
ros2 action list | grep greeting          # 应含 speak / navigate_to / play_motion
ros2 topic echo /greeting/health          # 三个布尔反映服务端在线情况

# 状态机当前状态（reliable + transient_local，后订阅也能收到最新值）
ros2 topic echo /greeting/state

# 本体关节反馈与下发的控制帧
ros2 topic echo /robot_state --once       # 关节实测角 + 电机错误码
ros2 topic hz /arm/cmd                    # 动作播放期间应接近 publish_rate_hz

# 底盘连通性（不经过 ROS）
curl -s http://192.168.11.10:9090/api/core/system/v1/robot/health
```

**常见问题**

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 状态停在 `IDLE` | 未下发 `start` / 讲稿为空 | 发 `/greeting/control_cmd`；看编排层加载日志 |
| 提示"动作服务端不在线" | 动作 / 语音服务端未起 | 确认对应服务端已启动（`greeting_bringup.launch.py` 已含） |
| 启动报 `approved_script_version` 类型错误 | 命令行 `-p` 把 `1.1` 推断成 DOUBLE | 用 launch 启动（已声明 `value_type=str`），别用命令行直接传 `1.1` |
| `reload_script` 失败 | 讲稿 `version` ≠ `approved_script_version` | 对齐版本号 |
| 拒绝目标 `未知动作` | 动作名拼写错 / 不在动作表 | 对照 `greeting_body/config/motions.yaml` |
| `控制通道未就绪` | `/arm/cmd` 等无订阅者 | 按手柄 `A` 完成平台自检，确认整机控制使能 |
| `电机未就绪`（故障码 0x8130） | 电机通讯掉线，`pos` 恒为 0.0 假值 | 检查电机供电/通讯，重启控制桥 |
| `底盘 REST 不可达` | 网络 / IP 白名单 / 固件未运行 | 用 `curl` 验证；检查 `host` |
| RSP 因权限无法启动 | ROS 日志目录不可写 | `export ROS_LOG_DIR=/tmp/ros_log ROS_HOME=/tmp/ros_home` |

---

## 8. 停止与清理

正常停止：在各终端按 `Ctrl+C`。

> ⚠️ 若节点是由 systemd 服务 `greeting-autostart.service` 拉起的（实机常态，见 §2.1），
> `Ctrl+C` 或 `kill` 子进程只会触发 systemd **自动重新拉起**，并不是真的停住。
> 要真正停下请用 `sudo systemctl stop greeting-autostart.service`。

停止后可能残留节点进程，需手动清理：

```bash
pgrep -af "orchestrator_node|motion_server|navigate_to_server|speak_action_server|simple_action_voice_node|voice_greet_tts_node|joy_mapper|panel_command_bridge|flow_command_bridge"
# 按 PID 逐个终止（注意：勿用 killall/pkill，避免误杀其他 ROS 进程）
kill <PID> [<PID> ...]
```

---

## 9. 实机注意事项（务必注意）

- 实机 `navigate_to` 走底盘 REST `:9090`，且**底盘与本体 TF 不连通**；
- 实机 `play_motion` 由 `greeting_body` 的 `motion_server` 以 **≥10 Hz 连续流式关节帧**下发；
- 实机 `speak` 由 `greeting_voice` 的 `/greeting/speak` 包装本体 TTS；
- 实机上电涉及**急停、限位、电压电流、操作权限**，必须先做安全确认与隔离测试。

> 🔴 首次调试务必低速（`speed_limit≤0.5`）、空载、有人守护、急停可达。
> 🔴 底盘 SDK **无软件急停**，安全依赖**物理急停 + 工作人员就近值守**；上电前确认物理急停可用。

---

## 10. 相关文档

| 文档 | 内容 |
| --- | --- |
| [大会流程.md](大会流程.md) | 讲稿原文（动作提示 + 台词） |
| [接口契约冻结表.md](接口契约冻结表.md) | 话题 / 服务 / Action 接口与安全约定 |
| [多人协作开发规范.md](多人协作开发规范.md) | 分工与协作约定 |
| [greeting_orchestrator/README.md](src/greeting_orchestrator/README.md) | 编排层（讲稿格式、`cue` 步骤、参数） |
| [greeting_body/README.md](src/greeting_body/README.md) | 实机上半身动作服务端 |
| [greeting_nav/README.md](src/greeting_nav/README.md) | 实机底盘 REST 导航服务端 |
| [greeting_voice/README.md](src/greeting_voice/README.md) | 语音（遥控触发 + `/greeting/speak`） |
| [greeting_teleop/README.md](src/greeting_teleop/README.md) | 手柄（SBUS）按键网关 |
