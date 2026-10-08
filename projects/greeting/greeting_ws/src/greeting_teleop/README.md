# greeting_teleop

遥控器（SBUS）按键网关 —— 订阅平台遥控器的官方事件话题 `/sbus_data/event`
（`bodyctrl_msgs/msg/SbusData`），把**命名按键事件**与**摇杆方向**翻译成迎宾命令，
发布到 `/greeting/panel_command`（`std_msgs/String`）。

本包另含两个命令桥：

- `panel_command_bridge`：把 `motion:<动作名>` 命令直接转成 `/greeting/play_motion`
  （`PlayMotion`）的 Action Goal，实现**手柄按键直达单个礼仪动作**；
- `flow_command_bridge`：把**全流程命令**（`start`/`pause`/`next`/`goto:<段号>`/`stop`…）
  转成 `/greeting/control_cmd`（`greeting_interfaces/ConsoleCommand`），
  供 `greeting_orchestrator` 消费，实现**手柄按键一键开始整段大会流程**——
  动作与语音的同步由编排层讲稿的 `cue` 步骤负责（动作比语音提前 1s/3s）。

对应接口契约 §2 的 **T2 面板/遥控命令通道**，是「无上位机时用遥控器兜底操作迎宾流程」的入口。

一键启动（手柄 + 动作栈 + 两个桥 + 编排层）：
`ros2 launch greeting_teleop greeting_bringup.launch.py`

---

## 1. 包的定位与依赖关系

### 1.1 在整机链路中的位置

```
        遥控器(SBUS) / 平台遥控进程
                    │
                    ▼
        /sbus_data/event  (bodyctrl_msgs/msg/SbusData)   ← 按键/拨杆边沿事件（主输入）
        /sbus_data        (sensor_msgs/msg/Joy)          ← 12 轴（可选，默认不订阅）
                    │
                    ▼
        greeting_teleop / joy_mapper      ← 本包
                    │ /greeting/panel_command (std_msgs/String, reliable/10)
                    ├────────► greeting_teleop / flow_command_bridge    ← 本包
                    │              │ /greeting/control_cmd (ConsoleCommand)
                    │              ▼
                    │          greeting_orchestrator（讲稿 cue：动作 + 语音同步）
                    ▼
        greeting_teleop / panel_command_bridge   ← 本包
                    │ /greeting/play_motion (PlayMotion, Action)
                    ▼
        greeting_body / motion_server（实机 臂/头/腰 动作执行）
```

🔴 **单一指挥者**：`joy_mapper` **只发布命令**；`panel_command_bridge` 只调用动作 Action
（`/greeting/play_motion`）；`flow_command_bridge` 只发布编排层指令
（`/greeting/control_cmd`）。三者都不直接碰 `/head|arm|waist/cmd`。

### 1.2 依赖一览

| 方向 | 依赖 | 用途 |
| --- | --- | --- |
| 运行依赖 | `rclpy` | 节点实现 |
| 运行依赖 | `std_msgs` | `/greeting/panel_command`（`String`） |
| 运行依赖 | `bodyctrl_msgs` | `SbusData`（按键事件 + 摇杆，平台官方消息包） |
| 运行依赖 | `sensor_msgs` | `Joy`（可选 12 轴，仅 `joy_topic` 非空时使用） |
| 运行依赖 | `python3-yaml` | 读取 `teleop_map.yaml` |
| 运行依赖 | `launch`、`launch_ros` | launch 文件 |
| 运行依赖 | `greeting_interfaces` | `panel_command_bridge` 用 `PlayMotion` 动作；`flow_command_bridge` 用 `ConsoleCommand` 消息 |
| 运行依赖 | `greeting_body` | `greeting_bringup.launch.py` include 其 `motion.launch.py` |
| 运行依赖 | `greeting_orchestrator` | `greeting_bringup.launch.py` include 其 `orchestrator.launch.py`（全流程命令的消费方） |

🔴 `bodyctrl_msgs` 由**实机平台**（`/home/nvidia/xos`）提供 —— 本包只在实机侧使用。

### 1.3 构建

```bash
# 前置（实机）：source /home/nvidia/xos/setup.bash（提供 bodyctrl_msgs）
cd /home/nvidia/greeting-body-motion/greeting_ws
colcon build --packages-select greeting_teleop --symlink-install
source install/setup.bash
```

---

## 2. 文件结构与职责

```
greeting_teleop/
├── greeting_teleop/
│   ├── __init__.py
│   ├── joy_mapper.py             # 节点 1：按键/摇杆 -> /greeting/panel_command
│   ├── panel_command_bridge.py   # 节点 2：motion:<名> -> /greeting/play_motion (Action)
│   └── flow_command_bridge.py    # 节点 3：全流程命令 -> /greeting/control_cmd (ConsoleCommand)
├── config/
│   └── teleop_map.yaml       # 按键/摇杆/组合键 -> 命令 映射表
├── launch/
│   ├── teleop.launch.py      # 仅启动 joy_mapper + 参数
│   └── greeting_bringup.launch.py  # 一键启动：动作服务端 + joy_mapper + 两桥 + 编排层
├── test/
│   └── test_flake8.py
├── setup.py / setup.cfg / package.xml
└── resource/greeting_teleop
```

可执行入口（`setup.py` `console_scripts`）：

| executable | 模块 |
| --- | --- |
| `joy_mapper` | `greeting_teleop.joy_mapper:main` |
| `panel_command_bridge` | `greeting_teleop.panel_command_bridge:main` |
| `flow_command_bridge` | `greeting_teleop.flow_command_bridge:main` |

---

## 3. 对外接口

### 3.1 订阅（输入）

| 话题 | 类型 | QoS | 说明 |
| --- | --- | --- | --- |
| `/sbus_data/event` | `bodyctrl_msgs/SbusData` | `reliable` / 10 | **主输入**；`key_event_new` 已由平台做边沿解码，无需自做上升沿检测 |
| `/sbus_data` | `sensor_msgs/Joy` | `best_effort` / 10 | **可选**，仅 `joy_topic` 非空时订阅（`buttons` 为空，仅 12 轴有用） |

### 3.2 发布（输出）

| 话题 | 类型 | QoS | 说明 |
| --- | --- | --- | --- |
| `/greeting/panel_command` | `std_msgs/String` | `reliable` / 10 | 命令词写入 `data`；契约 §8 T2 |

### 3.3 命令词表（写入 `panel_command.data`）

| 类别 | 取值 |
| --- | --- |
| 全流程 | `start` / `pause` / `resume` / `next` / `prev` / `stop` / `manual` |
| 单段点播 | `goto:<段号>`（如 `goto:2`）：只播该段，播完回 `IDLE`，**不续播**（遥控器"一段一键"） |
| 动作快捷键 | `motion:<动作名>`（动作名取自 `greeting_body/config/motions.yaml`） |

- 一个按键可同时发**多条**命令，用 `;` 连接（逐条独立发布），如 `stop;motion:greet_open_arms`。
- 发布前用 `_is_valid()` 校验：非法命令**丢弃并告警**。

### 3.4 直连桥 `panel_command_bridge` 接口

| 方向 | 话题 / Action | 类型 | QoS | 说明 |
| --- | --- | --- | --- | --- |
| 订阅 | `/greeting/panel_command` | `std_msgs/String` | `reliable` / 10 | 只处理 `motion:<名>` 与 `stop`/`manual` |
| 客户端 | `/greeting/play_motion` | `greeting_interfaces/PlayMotion` | Action 默认 | 发送 Goal，`speed_scale` 钳到 ≤0.7 |

命令处理与安全策略：

| 命令 | 行为 |
| --- | --- |
| `motion:<动作名>` | 发 Goal（`speed_scale` = `speed_scale`，上限 `speed_scale_max`≤0.7） |
| `stop` / `manual` | **取消**正在执行的动作（与 `guard.release` 配合，用于中止） |
| 其他（`start`/`next`/…） | 忽略（orchestrator 职责） |

- 🔴 **忙碌策略**：动作执行中再收到 `motion:` 默认**忽略并告警**，避免两条轨迹叠加导致关节冲突；
  置 `preempt:=true` 则先取消当前动作、待其结束后再补发新动作。
- 🔴 **双层速度兜底**：桥内钳一次（≤0.7），`motion_server` 再钳一次。
- 🔴 **白名单**：`allowed_motions` 非空时，只允许列出的动作名（逗号/空格分隔）。

### 3.5 流程命令桥 `flow_command_bridge` 接口

| 方向 | 话题 | 类型 | QoS | 说明 |
| --- | --- | --- | --- | --- |
| 订阅 | `/greeting/panel_command` | `std_msgs/String` | `reliable` / 10 | 只处理全流程命令 |
| 发布 | `/greeting/control_cmd` | `greeting_interfaces/ConsoleCommand` | `reliable` / 10 | `cmd`/`arg`/`operator_id` |

命令处理：

| 命令 | 行为 |
| --- | --- |
| `start` / `pause` / `resume` / `next` / `prev` / `stop` / `manual` | 原样转发（`arg=''`） |
| `goto:<段号>` | 拆成 `cmd='goto'`、`arg='<段号>'`（orchestrator 侧为**单段点播**：播完该段即回 `IDLE`） |
| `motion:<动作名>` | **忽略**（由 `panel_command_bridge` 直连动作 Action） |
| 其他 | 忽略并告警 |

- 🔴 本桥**不做任何编排/时序处理**：讲稿推进、动作与语音的提前量（`cue` 的 `lead_s`）
  全部由 `greeting_orchestrator` 决定。
- 🔴 需 `greeting_orchestrator` 在运行，全流程命令才有消费方；否则命令发出无人响应。
- 参数：`command_topic`（默认 `/greeting/panel_command`）、`control_topic`（默认 `/greeting/control_cmd`）、
  `operator_id`（默认 `rc`，写入 `ConsoleCommand.operator_id` 供追溯）。

---

## 4. 命名键（与 `SbusData.KEY_*` 一一对应，无需记数字下标）

| 物理控件 | 命名键 |
| --- | --- |
| 点动按键 A/B/C/D | `a` `a_up` \| `b` `b_up` \| `c` `c_up` \| `d` `d_up`（`a`=按下瞬间；`a_up`=松开瞬间） |
| 三档开关 E/F | `e_up` `e_mid` `e_down` \| `f_up` `f_mid` `f_down` |
| 左右拨杆 G/H | `g_left` `g_mid` `g_right` \| `h_left` `h_mid` `h_right` |

**摇杆命名键**（来自 `SbusData.x1/y1/x2/y2`，方向上升沿触发，无需订阅 `Joy`）：

| 命名键 | 含义 |
| --- | --- |
| `x1_high` / `x1_low` | 左摇杆 X：右 / 左 |
| `y1_high` / `y1_low` | 左摇杆 Y：上 / 下（文档定义 y1 下=-1 上=+1） |
| `x2_high` / `x2_low` | 右摇杆 X：右 / 左 |
| `y2_high` / `y2_low` | 右摇杆 Y：上 / 下 |

> `button_*` 状态字段（`-1` 松开 / `0` 中间 / `1` `2` 两端）**仅作 monitor 观察，不用于触发**；
> 触发一律以 `key_event_new` 边沿事件为准。

---

## 5. 组合键（多组前提 guards）

机制：`guards` 是**列表**，每组有独立的前提键（`key`）与按键表（`keys`）；前提键
**处于该档位**时该组生效，离开即失效，并可选下发 `release` 命令中止正在播放的动作。
运行时**按列表顺序取第一个"当前生效"的组**；都不命中时回退到常驻按键表 `keys`。

当前默认配置（2 组，按优先级排列；仅用于**分段点播**）：

```yaml
guards:
  - key: e_up          # 组1：E 拨「上」= 分段组（第 1~4 段）
    keys: {a: 'goto:1', b: 'goto:2', c: 'goto:3', d: 'goto:4'}
  - key: e_down        # 组2：E 拨「下」= 分段组（第 5~7 段）
    keys: {a: 'goto:5', b: 'goto:6', c: 'goto:7'}
```

按键总览（**先拨 E 前提，再按 A/B/C/D**）：

| 前提（开关档位） | `A` | `B` | `C` | `D` |
| --- | --- | --- | --- | --- |
| `E` 拨「上」 | 第 1 段 `goto:1` | 第 2 段 `goto:2` | 第 3 段 `goto:3` | 第 4 段 `goto:4` |
| `E` 拨「下」 | 第 5 段 `goto:5` | 第 6 段 `goto:6` | 第 7 段 `goto:7` | — |

- 判定依据为**事件值**（`KEY_E_UP` / `KEY_E_DOWN` …），不依赖 `button_*` 状态位。
- **同一开关族内各组互斥**：E 只有一个档位生效。
- 前提**默认未生效**：上电后需先拨动 E 才激活（安全设计）。
- ⚠️ **E 只有「上」「下」两个分段组**：拨到「中」无对应组，此时按 A/B/C/D 不会触发段号
  （回退到常驻按键表，而常驻表只有 `h_right: stop`）；拨到「上」或「下」后按 A/B/C/D
  会直接触发该组段号，请确认符合预期（防误触）。
- ⚠️ **F 上拨 + 长按 `A`** = 切换平台「语音功能」开关（由 `greeting_body/motion_server` 侧识别，
  用于暂停待机扫视并让头/腰回中立位；见 [greeting_body/README.md](../greeting_body/README.md) §7.3）；
  长按 A 平台同时会切换 lyre 语音交互，二者语义一致。**短按 A 不受影响**（按下即触发段号）。
- 旧写法单个 `guard:` 块仍受支持（自动并入 `guards` 列表，优先级最低）。

### 5.1 常驻按键（急停）

不受前提限制、始终生效：

```yaml
keys:
  h_right: stop     # H 右  ：急停回 IDLE（安全键，随时可中止流程）
```

- **H 右 = `stop`**：随时中止当前段与动作，回到 `IDLE`（实机安全兜底）。
- **整段连播（`start`）当前未绑定常驻按键**：如需"一键走完 7 段"，另择一键写入 `keys:` 并置为 `start`
  （如 `f_up: start`）；该命令经 `flow_command_bridge` → `/greeting/control_cmd` → `greeting_orchestrator`
  按讲稿顺序**走完 7 段**，同一步内**动作比语音提前触发**（由讲稿 `cue.lead_s` 决定，按键侧无需任何配置）。
- **分段点播**见 §5（E 上/下 选组 + A/B/C/D 选段）：每按一次只播该段（动作 + 语音同步），
  语音播完即回 `IDLE`，**不自动续播下一段**；可反复单点任意段。
- 段号与 `greeting_orchestrator/config/greeting_script.yaml` 的 `id` 一一对应
  （`goto:<段号>` 段号不能越界，1~7）。
- 其余常驻键（`h_left`=next / `h_mid`=manual / `f_mid`=resume）默认注释，按需启用。
- 🔴 全流程命令（含 `start` 与 `goto`）都需要 `greeting_orchestrator` 在运行
  （`greeting_bringup.launch.py` 默认会启动它；置 `with_orchestrator:=false` 时只保留动作快捷键）。

---

## 6. 配置详解（`config/teleop_map.yaml`）

| 段 | 作用 |
| --- | --- |
| `guards` | **组合键前提（多组，列表）**：每项 `key`（前提键）/ `release`（解除时下发，可选）/ `keys`（该组按键表）；**按顺序即优先级** |
| `guard` | 兼容旧写法（单个前提块）；存在时自动并入 `guards` 列表末尾 |
| `keys` | **常驻按键**（不受前提限制，始终生效）；已启用 `h_right:stop`（分段点播为 `E上/下 + A/B/C/D`），其余默认注释 |
| `sticks` | 摇杆方向 → 命令（默认不启用；推到对应方向触发一次） |
| `axes` | Joy 轴号 → `{high, low}` 命令（默认不启用；需 `joy_topic:=/sbus_data`） |
| `buttons` | 兼容旧写法：`{<SbusData 常量int>: 命令}`（新配置建议用 `keys`） |

⚠️ 平台层：**F 上拨 + 长按 A** 会切换语音交互（lyre），并被 `greeting_body/motion_server`
用作「语音功能」门控（暂停待机扫视、头/腰回中立位，见
[greeting_body/README.md](../greeting_body/README.md) §7.3）。本表 A 触发用的是**短按**
（按下即触发），不受长按逻辑影响。

---

## 7. 参数说明

`launch/teleop.launch.py` 暴露的参数：

| 参数 | 类型 | 默认 | 说明 |
| --- | --- | --- | --- |
| `event_topic` | string | `/sbus_data/event` | SBUS 按键事件话题 |
| `joy_topic` | string | `""` | 可选 Joy 轴话题；留空不订阅，填 `/sbus_data` 启用 12 轴映射 |
| `command_topic` | string | `/greeting/panel_command` | 命令话题 |
| `map_file` | string | `""` | 映射表；留空用包内 `config/teleop_map.yaml` |
| `deadband` | double | `0.5` | 轴阈值（`|value|>deadband` 才算拨动） |
| `operator_id` | string | `rc` | 操作人标识（追溯用，目前仅日志） |
| `monitor` | bool | `false` | `true`=标定模式：只打印、不发布 |
| `monitor_period_s` | double | `2.0` | 标定模式打印周期 (s) |

`panel_command_bridge` 的参数（`greeting_bringup.launch.py` 用 `bridge_*` 前缀暴露）：

| 参数 | 类型 | 默认 | 说明 |
| --- | --- | --- | --- |
| `command_topic` | string | `/greeting/panel_command` | 命令话题 |
| `action_name` | string | `/greeting/play_motion` | 动作 Action 名 |
| `speed_scale` | double | `0.5` | 手柄触发动作的速度缩放 |
| `speed_scale_max` | double | `0.7` | 速度硬上限，**强制 ≤0.7** |
| `preempt` | bool | `false` | 新动作是否抢占正在执行的动作 |
| `allowed_motions` | string | `""` | 动作名白名单（逗号/空格分隔），留空=不校验 |
| `action_timeout_s` | double | `2.0` | 等待动作服务端就绪的超时 (s) |

`flow_command_bridge` 的参数（`greeting_bringup.launch.py` 用 `flow_*` 前缀暴露）：

| 参数 | 类型 | 默认 | 说明 |
| --- | --- | --- | --- |
| `command_topic` | string | `/greeting/panel_command` | 输入命令话题 |
| `control_topic` | string | `/greeting/control_cmd` | 输出控制话题（编排层订阅） |
| `operator_id` | string | `rc` | 写入 `ConsoleCommand.operator_id`（追溯用） |

---

## 8. 启动与调用

### 8.1 前置条件

```bash
# 平台遥控进程在跑，/sbus_data/event 有数据
ros2 topic hz /sbus_data/event
```

### 8.2 启动

```bash
# 正常启动（需已标定 teleop_map.yaml）
ros2 launch greeting_teleop teleop.launch.py

# 标定模式：只打印原始按键事件与摇杆值，不发布任何命令
ros2 launch greeting_teleop teleop.launch.py monitor:=true

# 同时启用 /sbus_data(Joy) 的 12 轴映射
ros2 launch greeting_teleop teleop.launch.py joy_topic:=/sbus_data
```

启动日志应包含：模式（工作/标定）、输入→输出话题、按键/摇杆/轴映射项数、组合键前提、`operator_id`。

### 8.3 一键启动（手柄遥控直达动作 + 触发大会流程）

`greeting_bringup.launch.py` 一次拉起 **动作服务端 + 手柄网关 + 两个桥 + 编排层**：

```bash
ros2 launch greeting_teleop greeting_bringup.launch.py
ros2 launch greeting_teleop greeting_bringup.launch.py speed_limit:=0.5 bridge_speed_scale:=0.4
ros2 launch greeting_teleop greeting_bringup.launch.py monitor:=true              # 标定按键
ros2 launch greeting_teleop greeting_bringup.launch.py with_orchestrator:=false   # 只用动作快捷键
```

链路：

```
/sbus_data/event → (joy_mapper) → /greeting/panel_command
      ├─ (panel_command_bridge) → /greeting/play_motion → (motion_server) → /arm|head|waist/cmd
      └─ (flow_command_bridge)  → /greeting/control_cmd → (greeting_orchestrator 讲稿 cue)
```

按键：**H 右 = stop**（急停；整段连播 `start` 当前未绑定常驻键）；
**分段点播 = 先拨 E 到「上」或「下」选组，再按 A/B/C/D**（`goto:1..7`，每段一键，播完回 IDLE）；
另有 **F 上 + 长按 A** 切换平台语音功能（开启时暂停待机扫视，见 greeting_body §7.3）。

### 8.4 开机自启动（systemd）

工作空间提供了入口脚本与 systemd 单元（`greeting_ws/scripts/`）：

```bash
# 1) 安装单元文件（需 sudo，请自行执行）
sudo cp /home/nvidia/greeting-body-motion/greeting_ws/scripts/greeting-autostart.service \
        /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now greeting-autostart.service

# 2) 查看状态与日志
systemctl status greeting-autostart.service
journalctl -u greeting-autostart.service -f
```

- 单元以 `User=root` 运行，与平台进程（`proc_manager`/`robot_control` 均为 root）一致，
  规避 FastDDS 共享内存跨用户权限导致的 DDS 发现问题；`Slice=xos.slice`，`After=proc_manager.service`。
- 入口脚本 `scripts/greeting_autostart.sh` 依次 source
  `/opt/ros/jazzy/setup.bash` → `/home/nvidia/xos/setup.bash` → `install/setup.bash`，再 `exec ros2 launch …`。
- 可用环境变量覆盖：`GREETING_SPEED_LIMIT`、`GREETING_ARM_CURRENT`、`GREETING_BRIDGE_SPEED_SCALE`、`GREETING_EXTRA_ARGS`。
- 脚本内建 `flock` **单实例保护**：双实例会让 `/arm|/head|/waist/cmd` 出现重复发布者、电机互斗。
- 回退：`sudo systemctl disable --now greeting-autostart.service`。
- ⚠️ 开机流程：服务随开机起（此时动作服务端只是待命）→ 按手柄 **A** 触发平台自检
  （A 键由平台 `proc_manager` 处理，`robot_control` 使能后 `/arm/cmd` 才有订阅者）
  → 拨 E 到「上」或「下」→ 按 A/B/C/D 分段点播。

### 8.5 标定流程（推荐）

1. `monitor:=true` 启动，逐个按 A/B/C/D、拨 E/F/G/H、推四个摇杆；
2. 观察日志 `key_event 旧 -> 新` 与 `前提[...] : 生效/未生效`，确认命名键与实际控件对应；
3. 据实填写 `config/teleop_map.yaml`；
4. 正常模式重启，`ros2 topic echo /greeting/panel_command` 确认命令发布正确。

### 8.6 手动验证命令通路

```bash
ros2 topic echo /greeting/panel_command
# 触发遥控器按键后，应看到对应字符串（如 goto:3）

# 也可不经手柄，直接注入命令验证桥接
ros2 topic pub --once /greeting/panel_command std_msgs/String "{data: 'motion:greet_open_arms'}"
ros2 action list | grep play_motion          # 确认服务端就绪

# 验证全流程命令桥：应看到 /greeting/control_cmd 出现 cmd=start（需 orchestrator 在跑才会真正执行）
ros2 topic echo /greeting/control_cmd greeting_interfaces/msg/ConsoleCommand
ros2 topic pub --once /greeting/panel_command std_msgs/String "{data: 'start'}"
ros2 topic echo /greeting/state              # 应切换到 GREETING 并逐段推进

# 验证分段点播：应看到 cmd=goto arg=3，状态进入 EXPLAIN，播完回到 IDLE（不续播 CHECKIN）
ros2 topic pub --once /greeting/panel_command std_msgs/String "{data: 'goto:3'}"
```

---

## 9. 验证与调试

```bash
# 节点与话题
ros2 node info /greeting_joy_mapper
ros2 topic list | grep sbus
ros2 topic echo /greeting/panel_command
```

**常见问题**

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 完全没有命令发布 | `/sbus_data/event` 无数据 / 映射表为空 | `ros2 topic hz`；用 `monitor:=true` 标定 |
| 按键无反应 | 命名键写错 / 前提未生效 | 见标定流程；分段组合键需先拨 `E` 到「上」或「下」（`E` 拨「中」无分段组，不触发段号） |
| 同一按键命令重复触发 | 多网卡 DDS 多路径重复投递 | 已内建去重（同一 `key_event_new` 只处理一次）；若仍重复检查网卡配置 |
| E 上/下 + A/B/C/D 无反应 | E 未拨到上/下档，或 `greeting_orchestrator` / `flow_command_bridge` 未运行 | 确认 E 档位；`ros2 node list` 检查消费方是否在线 |
| 长按 A 后待机扫视停了 | F 上拨 + 长按 A 切换了「语音功能」门控（暂停扫视、头/腰回中立位） | 再长按一次 A 关闭语音，扫视即恢复（见 greeting_body §7.3） |
| 摇杆映射不触发 | `sticks` 段被注释 / 未达 `deadband` | 取消注释；调小 `deadband` |
| Joy 轴映射不触发 | `joy_topic` 为空 | `joy_topic:=/sbus_data` |
| 编译找不到 `bodyctrl_msgs` | 未 source 实机平台环境 | `source /home/nvidia/xos/setup.bash` 后重建 |
| 命令发出但迎宾流程无反应 | 未启动 `greeting_orchestrator`（`/greeting/control_cmd` 无订阅者），或未启动 `flow_command_bridge` | 检查 `ros2 node list` 是否有 `greeting_orchestrator` / `greeting_flow_bridge`；用 `greeting_bringup.launch.py` 一键启动 |
| 动作快捷键有效但 `start` 无效 | `with_orchestrator:=false` 或编排层未起 | 去掉该参数重启，或在另一终端 `ros2 launch greeting_orchestrator orchestrator.launch.py` |

---

## 10. 注意事项

- 🔴 **单一指挥者**：本包节点只发布命令 / 只调用动作 Action，不直接控制本体
  （不碰 `/arm|/head|/waist/cmd`）。
- 🔴 **命令下游有两条**：
  - `motion:<名>` / `stop` / `manual` → `panel_command_bridge` → `/greeting/play_motion`（单动作，直达）；
  - 全流程命令（`start`/`pause`/`resume`/`next`/`prev`/`goto:<段号>`/`stop`/`manual`）
    → `flow_command_bridge` → `/greeting/control_cmd`（`ConsoleCommand`）
    → `greeting_orchestrator` 讲稿（**动作 + 语音同步，动作比语音提前，提前量由讲稿 `cue.lead_s` 逐段决定**）。
  - ⚠️ 因此**全流程命令需要 `greeting_orchestrator` 在运行**才有消费方；
    `greeting_bringup.launch.py` 默认会一并启动它（`with_orchestrator:=false` 可关闭）。
- 🔴 组合键前提**默认未生效**（安全设计），上电后需显式拨到前提档位才激活。
- 🔴 分段点播键（`goto:<段号>`）位于**组合键**中：需先拨 `E` 到「上」或「下」，再按 `A/B/C/D`。
  详见 §5 按键总览表。
- 🔴 触发一律以平台边沿事件 `key_event_new` 为准，`button_*` 状态位不作触发依据。

---

## 11. 相关文档

- [《接口契约冻结表》v1.0](../../接口契约冻结表.md) §2 T2 / §8
- [greeting_orchestrator/README.md](../greeting_orchestrator/README.md)（命令的消费方）
- [greeting_body/README.md](../greeting_body/README.md)（`motion:<动作名>` 的动作表来源）
- [greeting_nav/README.md](../greeting_nav/README.md)（导航动作服务端）