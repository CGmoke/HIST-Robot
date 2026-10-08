# greeting_orchestrator

迎宾编排层 —— **唯一指挥者 / 唯一状态机**（接口契约 §1）。

按 `config/greeting_script.yaml` 的讲解顺序，依次调用 `speak` / `navigate_to` / `play_motion`
三个 Action，驱动整条迎宾流程；对外提供状态发布、健康上报与控制服务。

> 讲稿支持**复合步骤 `cue`**：一步内**并行**编排「动作 + 语音」，**动作比语音提前 `lead_s` 触发**，
> 两者都结束后再推进下一步。第 1、7 段 `lead_s=1.0`；第 4~6 段 `lead_s=3.0`；第 2、3 段 `lead_s=0.0`（动作与语音同时触发）。

---

## 1. 包的定位与依赖关系

### 1.1 在整机链路中的位置

```
                  ┌───────────────────────────────┐
                  │     greeting_orchestrator     │  ← 本包
                  │  唯一指挥者 / 唯一状态机（P）  │
                  └──┬──────────┬──────────┬──────┘
        Action 客户端 │          │          │
                     ▼          ▼          ▼
          /greeting/speak  /greeting/navigate_to  /greeting/play_motion
             (A1)              (A2)                   (A3)
                     ▲          ▲          ▲
                     │          │          │
        实机服务端（语音 / 底盘 REST :9090 / 机械臂）
```

🔴 **单一指挥者规则**：只有本节点调用上述三个 Action，模块之间**禁止互相调用**。

### 1.2 依赖一览

| 方向 | 依赖 | 用途 |
| --- | --- | --- |
| 运行依赖 | `rclpy` | 节点实现 |
| 运行依赖 | `std_msgs` | `/greeting/state`（`String`） |
| 运行依赖 | `greeting_interfaces` | 全部 msg / srv / action |
| 运行依赖 | `python3-yaml` | 读取讲稿 YAML |
| 运行依赖 | `launch`、`launch_ros` | launch 文件 |
| 被依赖 | — | 本包不被其他自建包 `exec_depend`（由 `greeting_teleop/greeting_bringup.launch.py` 运行时 include） |

### 1.3 构建

```bash
colcon build --packages-select greeting_interfaces greeting_orchestrator --symlink-install
```

---

## 2. 文件结构与职责

```
greeting_orchestrator/
├── greeting_orchestrator/
│   ├── __init__.py
│   └── orchestrator_node.py     # 唯一可执行节点：GreetingOrchestrator
├── config/
│   └── greeting_script.yaml     # 讲稿（唯一"剧本"）
├── launch/
│   └── orchestrator.launch.py   # 启动节点 + 参数
├── setup.py / setup.cfg / package.xml
└── resource/greeting_orchestrator
```

| 文件 | 职责 |
| --- | --- |
| `orchestrator_node.py` | 状态机、讲稿推进、三个 Action 客户端、服务端、状态/健康发布 |
| `greeting_script.yaml` | 讲稿段与 step（`speak` / `motion` / `nav` / `cue`），含审核版本号 |
| `orchestrator.launch.py` | 声明并注入五个参数 |

---

## 3. 对外接口定义

### 3.1 订阅（输入）

| 话题 | 类型 | QoS | 回调 | 作用 |
| --- | --- | --- | --- | --- |
| `/greeting/control_cmd` | `ConsoleCommand` | `reliable` / 10 | `_on_cmd` | 控制台指令入口 |
| `/greeting/events` | `GreetingEvent` | `reliable` / 10 | `_on_event` | 迎宾事件（人工开始 / 结束 / 领导提示） |
| `/greeting/intent` | `GreetingIntent` | `reliable` / 10 | `_on_intent` | 语音意图（仅记录，不生成文案） |
| `/greeting/perception/person` | `PersonDetection` | `best_effort` / 5 | `_on_person` | 人员感知（仅 debug 日志） |

### 3.2 发布（输出）

| 话题 | 类型 | QoS | 频率 | 说明 |
| --- | --- | --- | --- | --- |
| `/greeting/state` | `std_msgs/String` | `reliable` / depth 1 / 🔴 `transient_local` | 状态变化时 | 状态机当前状态 |
| `/greeting/health` | `SubsystemHealth` | `reliable` / 10 | 1 Hz | 子系统健康 |

### 3.3 服务

| 服务名 | 类型 | 说明 |
| --- | --- | --- |
| `/greeting/control` | `GreetingControl` | 与 `/greeting/control_cmd` 同语义 |
| `/greeting/reload_script` | `GreetingControl` | 重载讲稿；校验审核版本号不符则拒绝 |

### 3.4 Action 客户端

| 客户端 | 类型 | 目标服务端 |
| --- | --- | --- |
| `/greeting/speak` | `Speak` | B（实机语音服务端） |
| `/greeting/navigate_to` | `NavigateTo` | A（实机底盘 REST :9090） |
| `/greeting/play_motion` | `PlayMotion` | A（实机机械臂动作服务端） |

---

## 4. 状态机与交互逻辑

### 4.1 状态集合

| 状态 | 含义 |
| --- | --- |
| `IDLE` | 空闲待机 |
| `GREETING` / `INTRO` / `EXPLAIN` / … | 正在执行对应讲稿段（取该段 `name` 大写） |
| `PAUSED` | 已暂停 |
| `MANUAL` | 人工接管 |

- 可推进状态集合 `self._running_states = {"RUNNING"} ∪ {各段 name 大写}`：
  加载讲稿时**动态并入**每段的 `name`，避免某段状态被 `_tick` 视为非运行态而卡住。
- 新增 / 修改讲稿段名后无需改代码，重新加载讲稿即生效。

### 4.2 主循环（`_tick`，0.5 s 定时）

```
每 0.5s:
  若 状态 ∉ self._running_states 或 已暂停 → 直接返回
  若 有 pending Action（正在执行）→ _poll_pending() 检查 future
  否则 → _start_next_step() 下发下一步
```

### 4.3 讲稿推进逻辑

```
segments[i].steps[j]
  ├─ kind: speak  → Speak.Goal(text)
  ├─ kind: motion → PlayMotion.Goal(motion_name, speed_scale ← 钳到 ≤0.7)
  ├─ kind: nav    → NavigateTo.Goal(waypoint, timeout_s ← 默认参数兜底)
  └─ kind: cue    → 并行下发「动作 + 语音」，动作比语音早 lead_s 触发
        │
        ▼
   单步：_send() → send_goal_async → future(stage=goal)
        │ 目标被接受
        ▼
   handle.get_result_async() → future(stage=result)
        │ 结果返回（成功或失败均继续）
        ▼
   _advance_step() → 步满 _advance_segment() → 段满则回到 IDLE
```

**`cue` 复合步骤（动作先行）**

- 立即下发 `PlayMotion`；用一次性定时器（`lead_s` 秒后）再下发 `Speak`，两者**并行**执行。
- `lead_s` 缺省取参数 `default_motion_lead_s`（默认 3.0）；讲稿按段显式覆盖：
  第 1、7 段 `lead_s=1.0`；第 4~6 段 `lead_s=3.0`；第 2、3 段 `lead_s=0.0`（动作与语音同时触发）。
- 两个子任务都到达 `result` 阶段后，日志汇总并 `_advance_step()`；`_abort_pending` 会同时取消
  定时器与两个 `goal_handle`。
- 说明：实机 `motion_server` 有 `warmup_s`（默认 1.5 s）软化移交段；若希望"可见动作"也保持同样
  提前量，可把 `lead_s` 各加 `warmup_s`。

- 目标被拒绝 / 超时未成功时**记录告警并继续下一步**，不阻塞整条讲稿。
- 服务端不在线时，仅告警一次（`_warn_missing`），等待其启动。

### 4.4 指令语义（`_handle_command`）

| `cmd` | 前置条件 | 行为 | 返回消息 |
| --- | --- | --- | --- |
| `start` | — | 从第 0 段开始，**连播**走完全部段（`_single_segment=False`） | 开始接待 |
| `pause` | 状态 ∈ `self._running_states` | 置 `PAUSED` | 已暂停 / 当前状态不可暂停 |
| `resume` | 状态 == `PAUSED` | 恢复原段状态 | 已恢复 / 当前状态不可恢复 |
| `next` | — | 段号 +1（不越界），**连播**（`_single_segment=False`） | 跳到下一段 |
| `prev` | — | 段号 −1（不越界），**连播**（`_single_segment=False`） | 回到上一段 |
| `goto` | `arg` 为合法段号（1~段数） | **单段点播**：置 `_single_segment=True`，只播第 `arg` 段，播完回 `IDLE`，不续播 | 点播第 N 段（播完回到 IDLE）/ 参数非法 / 越界 |
| `stop` | — | 中止 pending，置 `IDLE` | 已停止 |
| `manual` | — | 中止 pending，置 `MANUAL` | 进入人工接管 |

> 🔴 **连播 vs 单段点播**：`_single_segment` 标志由 `start`/`next`/`prev`/`EVENT_MANUAL_START`
> 置 `False`、由 `goto` 置 `True`。`_advance_segment()` 在标志为 `True` 时本段结束即回 `IDLE`，
> 不再 `_start_segment(idx+1)`。遥控器"一段一个按键"即依赖此语义（见 greeting_teleop §5.1）。

### 4.5 事件语义（`_on_event`）

| `event_type` | 行为 |
| --- | --- |
| `EVENT_MANUAL_START` | 置 `_single_segment=False`，`_start_segment(0)` 开始接待（连播） |
| `EVENT_END` | 中止 pending，置 `IDLE` |
| `EVENT_LEADER_ARRIVED` | 🔴 仅日志提示，**不自动接待**（契约 §5.1） |
| `EVENT_LEADER_LEAVE` | 仅日志提示 |

---

## 5. 参数说明

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `config_file` | `string` | `""` | 讲稿 YAML 路径；留空则用包内 `config/greeting_script.yaml` |
| `state_topic` | `string` | `/greeting/state` | 状态发布话题 |
| `default_nav_timeout` | `double` | `60.0` | 讲稿 step 未写 `timeout_s` 时的导航超时 (s) |
| `approved_script_version` | `string` | `"1.1"` | 已审核讲稿版本号，`reload_script` 校验用 |
| `default_motion_lead_s` | `double` | `3.0` | `cue` 步骤未显式写 `lead_s` 时，动作相对语音的默认提前量 (s) |

### 讲稿 YAML 结构（`config/greeting_script.yaml`）

```yaml
version: "1.1"          # 🔴 审核版本号，必须与 approved_script_version 一致
segments:
  - id: 1
    name: GREETING      # 状态名（大写后作为 /greeting/state）
    steps:
      # 复合步骤：动作先行、语音跟随（并行）；本段用于「开场打招呼」→ lead_s: 1.0
      - kind: cue
        motion_name: greet_open_arms
        speed_scale: 0.5    # 🔴 ≤ 0.7
        text: "…"
        lead_s: 1.0         # 动作比语音早 1.0s（第 4~6 段写 3.0，第 2、3 段写 0.0 同时触发）
      # 也可单独使用三类单步：
      - kind: speak
        text: "…"
      - kind: motion
        motion_name: point_left
        speed_scale: 0.5
      - kind: nav
        waypoint: desk
        timeout_s: 60.0
```

---

## 6. 调用方法

### 6.1 启动

```bash
cd <工作空间>
colcon build --packages-select greeting_interfaces greeting_orchestrator --symlink-install
source install/setup.bash

# 默认参数启动
ros2 launch greeting_orchestrator orchestrator.launch.py

# 自定义审核版本
ros2 launch greeting_orchestrator orchestrator.launch.py approved_script_version:=1.1
```

launch 参数：`config_file`、`approved_script_version`(str)、`default_nav_timeout`(float)、`default_motion_lead_s`(float)。

> ⚠️ launch 中 `approved_script_version` 必须显式声明为 `str`（`ParameterValue(..., value_type=str)`），
> 否则 `1.0` 会被推断为 `DOUBLE`，与节点内字符串默认值冲突导致启动失败。

### 6.2 运行期操作

```bash
# 看状态
ros2 topic echo /greeting/state

# 触发接待
ros2 topic pub --once /greeting/control_cmd greeting_interfaces/msg/ConsoleCommand \
  "{cmd: 'start', arg: '', operator_id: 'cli'}"

# 单段点播：只播第 2 段（动作 + 语音），播完回 IDLE，不续播第 3 段
ros2 topic pub --once /greeting/control_cmd greeting_interfaces/msg/ConsoleCommand \
  "{cmd: 'goto', arg: '2', operator_id: 'cli'}"

# 健康上报
ros2 topic echo /greeting/health

# 服务调用
ros2 service call /greeting/control greeting_interfaces/srv/GreetingControl \
  "{cmd: 'pause', arg: '', operator_id: 'cli'}"
```

### 6.3 Python 内部集成（示例）

```python
# 直接构造节点（测试用）
from greeting_orchestrator.orchestrator_node import GreetingOrchestrator
import rclpy

rclpy.init()
node = GreetingOrchestrator()
rclpy.spin(node)
```

---

## 7. 验证与调试

```bash
# 节点与话题拓扑
ros2 node info /greeting_orchestrator
ros2 topic list | grep greeting
rqt_graph

# 服务端是否在线（对应 /greeting/health 的三个布尔）
ros2 action list | grep greeting
ros2 node list

# 手动单测某个 Action 服务端
ros2 action send_goal /greeting/play_motion greeting_interfaces/action/PlayMotion \
  "{motion_name: 'point_left', speed_scale: 0.5}"
```

**常见问题**

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 状态永远停在 `IDLE` | 未下发 `start`，或讲稿为空 | 下发 `/greeting/control_cmd`；检查讲稿加载日志 |
| 提示"动作服务端不在线" | 实机动作服务端未起 | 先启动 `greeting_body` 的动作服务 |
| `reload_script` 失败 | 讲稿 `version` ≠ `approved_script_version` | 对齐版本号或改参数 |
| `/greeting/state` 收不到 | 订阅端 QoS 未用 `transient_local` | 订阅端声明 `TRANSIENT_LOCAL`/depth 1 |
| 讲稿版本被推断成 double | launch 未指定 `value_type=str` | 见 §6.1 注意 |

---

## 8. 注意事项

- 🔴 本节点是**唯一指挥者**；新增业务请在此编排，勿让模块互调。
- 🔴 `speed_scale` 在编排层与动作服务端**各钳一次**（双层兜底，礼仪硬要求）。
- 🔴 讲稿是**审核对象**，修改后须同步 `approved_script_version`。
- 编排层只依赖 `/greeting/*` 接口契约，不关心各 Action 服务端的具体实现。

---

## 9. 相关文档

- [《接口契约冻结表》v1.0](../../接口契约冻结表.md) §1 / §3 / §5 / §6 / §8
- [greeting_interfaces/README.md](../greeting_interfaces/README.md)