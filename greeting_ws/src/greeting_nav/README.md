# greeting_nav

天轶 2.5 **实机**导航包 —— `NavigateTo` Action 服务端，走底盘 REST `:9090`（knewbots 固件）完成
固定范围内短距移位，并按契约发布 `navigating` / `arrived` / `retrying` / `relocalizing` 反馈。

编排层通过 `/greeting/navigate_to` Action 调用本包完成移位。

---

## 1. 包的定位与依赖关系

### 1.1 在整机链路中的位置

```
        greeting_orchestrator（唯一指挥者）
                    │ Action 客户端
                    ▼
        /greeting/navigate_to  (greeting_interfaces/action/NavigateTo)
                    │
                    ▼
              greeting_nav        ← 本包
              （实机：底盘 REST :9090）
                    │
                    ▼
        knewbots 底盘固件 REST API
        http://192.168.11.10:9090
        （fire-and-forget + 自行轮询位姿）
```

🔴 **单一指挥者规则**：本包只做 `/greeting/navigate_to` 的**服务端**，导航由 `greeting_orchestrator` 编排下发。

### 1.2 依赖一览

| 方向 | 依赖 | 用途 |
| --- | --- | --- |
| 运行依赖 | `rclpy`、`rclpy.action` | Action 服务端、节点 |
| 运行依赖 | `greeting_interfaces` | `NavigateTo` 动作类型 |
| 运行依赖 | `python3-requests` | 底盘 REST HTTP 客户端 |
| 运行依赖 | `python3-yaml` | 读取 `waypoints.yaml` |
| 运行依赖 | `ament_index_python` | 定位包内默认 config |
| 运行依赖 | `launch`、`launch_ros` | launch 文件 |

本包**不依赖 ros2_bridge_msgs**，可在纯 ROS 2 环境构建；但运行需底盘 REST 可达。

### 1.3 构建

```bash
cd /home/nvidia/greeting-body-motion/greeting_ws
colcon build --packages-select greeting_interfaces greeting_nav --symlink-install
source install/setup.bash
```

---

## 2. 文件结构与职责

```
greeting_nav/
├── greeting_nav/
│   ├── __init__.py
│   ├── chassis_client.py      # 底盘 REST 客户端（传输层，不含任何 ROS 概念，可单独联调）
│   └── navigate_to_server.py  # /greeting/navigate_to Action 服务端 + 任务状态机
├── config/
│   └── waypoints.yaml         # 点位表（🔴 现场实测标定）+ 坐标围栏
├── launch/
│   └── navigate_to.launch.py  # 启动服务端 + 参数
├── test/                      # ament 模板测试（copyright / flake8 / pep257）
├── setup.py / setup.cfg / package.xml
└── resource/greeting_nav
```

可执行入口（`setup.py` `console_scripts`）：

| executable | 模块 |
| --- | --- |
| `navigate_to_server` | `greeting_nav.navigate_to_server:main` |

| 文件 | 职责 |
| --- | --- |
| `chassis_client.py` | HTTP 封装、错误分类（`NavError`）、速度硬上限把关、健康/位姿/定位/运动/重定位接口 |
| `navigate_to_server.py` | Pre-flight、围栏拒绝、下发、轮询到点判定、重试、重定位、dry-run |

---

## 3. 对外接口

### 3.1 Action 服务端

| 项 | 值 |
| --- | --- |
| 动作名 | `/greeting/navigate_to` |
| 类型 | `greeting_interfaces/action/NavigateTo` |
| Goal | `string waypoint`、`float32 timeout_s`（≤0 时用 `default_timeout_s`） |
| Result | `bool success`、`string message` |
| Feedback | `string phase`、`float32 distance_remaining_m` |

**`phase` 取值**：`navigating`（导航中）/ `arrived`（到达）/ `retrying`（重试）/ `relocalizing`（重定位中）。

**Goal 校验**（`_on_goal`，任一失败即 `REJECT`）：

1. `waypoint` 必须存在于点位表；
2. 点位坐标必须落在**应用层坐标围栏**内。

---

## 4. 主流程（`_execute`）

```
dry_run? ──是──► 不发任何 REST，走 navigating -> arrived 假反馈
   │否
   ▼
for attempt in 1..(max_retries+1):
    ├─ Pre-flight ①  get_robot_health()   健康异常 -> 失败「原地接待」
    ├─ Pre-flight ②  _ensure_localization()  定位不可用 -> 重定位一次 -> 复检
    ├─ 下发           move_to(x, y, yaw, MOVE_WITH_THETA) -> action_id
    ├─ 轮询 _poll()   位姿到点判定 / 超时 / 取消 / 显式失败
    │     ├─ arrived  -> succeed()
    │     ├─ cancel   -> canceled()
    │     └─ timeout/failed -> 取消该 action_id，进入下一轮重试
    └─（重试前 feedback: retrying，并 backoff）
重试耗尽 -> 可选 recover_localization() -> 失败「原地接待 + 告警」
```

### 4.1 到点判定（主判据是**位姿**，契约要求）

| 判据 | 参数 | 默认 |
| --- | --- | --- |
| 位置误差 | `arrive_pos_tol_m` | `0.10 m` |
| 角度误差 | `arrive_yaw_tol_deg` | `2.0°` |
| 静止确认时长 | `still_confirm_s` | `0.5 s` |
| 静止位置变化 | `still_pos_tol_m` | `0.02 m` |
| 静止角度变化 | `still_yaw_tol_deg` | `0.5°` |

> ⚠️ `get_action_status()` 返回字段名厂家文档**未固化**，**不可作为到点主判据**；
> 服务端只用它**保守**探测「显式失败」（扫描 `fail/error/abort/...` 关键字），未知一律视为运行中。

### 4.2 定位可用性（`_ensure_localization`）

同时满足以下三条才视为可用，否则触发 `recover_localization()` 并等待 `relocalize_wait_s` 后复检：

- `pose.status == 0`（否则定位异常）；
- `pose.confidence ≥ min_confidence`（默认 0.6）；
- `localization_status.slam_status == 0`（0 正常 / 1 丢失 / 2 精度下降 / 3 异常 / 5 等待重定位 / 6 重定位中 …）。

### 4.3 失败语义

| 情况 | `success` | `message` 摘要 |
| --- | --- | --- |
| 到达 | true | `已到达 <name>（x, y）` |
| dry-run | true | `[DRY-RUN] 模拟到达 <name> —— 未真实移动` |
| 底盘健康异常 | false | `底盘健康异常（…），请原地接待` |
| 定位不可用 | false | `定位不可用（重定位后仍无效），请原地接待` |
| 底盘 REST 不可达 | false | `底盘 REST 不可达，请全程原地接待` |
| 超时 | false | `导航超时：<name> …请原地接待` |
| 被取消 | false | `导航被取消：<name>` |

---

## 5. 底盘 REST 接口（`chassis_client.py`）

| 方法 | HTTP | 路径 | 说明 |
| --- | --- | --- | --- |
| `get_robot_health()` | GET | `/api/core/system/v1/robot/health` | 每次下发前的 pre-flight；返回结构化 `ChassisHealth` |
| `get_pose()` | GET | `/api/core/navigation/v1/pose` | `dict(x, y, yaw, confidence, status)` |
| `get_localization_status()` | GET | `/api/core/slam/v1/localization/status` | `slam_type`, `slam_status` |
| `move_to(x, y, yaw, flags)` | PUT | `/api/core/navigation/v1/actions/moveto` | 返回 `action_id`；**fire-and-forget** |
| `move_by(...)` | PUT | `/api/core/navigation/v1/actions/moveby` | 相对移动（不依赖 SLAM，本版不主动调用，留作脱困） |
| `get_action_status(id)` | GET | `/api/core/motion/v1/actions/{id}` | 仅作显式失败探测 |
| `cancel_action([ids])` | PUT | `/api/core/navigation/v1/cancel_action` | 空列表 = 取消全部 |
| `recover_localization(x,y,yaw)` | PUT | `/api/core/slam/v1/localization/recover` | 重定位 |
| `set_speed_params(lin, ang)` | PUT | `/api/core/system/v1/parameter` | **全局**限速（`moveto` 本身不接受速度参数） |

### 5.1 错误分类（`NavError`）

| 码 | 名称 | 含义 |
| --- | --- | --- |
| 1002 | `REQUEST_TIMEOUT` | 请求超时 |
| 1003 | `INVALID_REQUEST` | 参数校验失败 |
| 3001 | `CONNECTION_ERROR` | 无法连接底盘（网线/IP 白名单/固件） |
| 3002 | `HTTP_ERROR` | REST 返回非 2xx |
| 5001 | `WAYPOINT_NOT_FOUND` | 点位不存在 |
| 5002 | `LOCALIZATION_NOT_READY` | 定位未就绪 |
| 5003 | `CHASSIS_HEALTH_ERROR` | 底盘健康异常 |
| 9001 | `INTERNAL_ERROR` | 兜底（如响应缺 `action_id`） |

### 5.2 速度硬上限（本模块提前拦截）

| 量 | 固件硬上限 | 常量 |
| --- | --- | --- |
| 线速度 | `1.5 m/s` | `MAX_MOVING_SPEED_MPS` |
| 角速度 | `1.5708 rad/s` | `MAX_ANGULAR_SPEED_RADPS` |

🔴 `moveto` 不接受速度参数，**唯一限速杠杆是全局 `set_speed_params`**，由 `apply_speed_limit` 控制是否启用
（默认关闭，避免意外改动全局设置）。

### 5.3 脱离 ROS 联调

`chassis_client.py` 不含任何 ROS 概念，可独立验证底盘连通性：

```bash
python3 -c "
from greeting_nav.chassis_client import ChassisClient
c = ChassisClient()
print(c.get_robot_health().describe())
print(c.get_pose())
c.close()"
```

---

## 6. 配置详解（`config/waypoints.yaml`）

| 键 | 内容 |
| --- | --- |
| `frame_id` | 坐标系（默认 `map`，现场建图原点） |
| `waypoints` | 点位名 → `{x, y, yaw}` |
| `range_limit` | 应用层坐标围栏 `x: [min,max]`、`y: [min,max]`，**点位超出即 REJECT** |

默认点位：`standby`（迎宾位）/ `explain_point`（讲解位）/ `photo_point`（合影位），当前均为占位值 0.0。

🔴 **坐标必须现场实测标定，禁止推算。**

**标定方法**：

```bash
# 1) 用底盘遥控把机器人移到目标位置
# 2) 读取当前位姿
curl -s http://192.168.11.10:9090/api/core/navigation/v1/pose
# 3) 回填 x / y / yaw 到本文件；重启 navigate_to_server 生效
```

---

## 7. 参数说明

`launch/navigate_to.launch.py` 暴露的参数：

| 参数 | 类型 | 默认 | 说明 |
| --- | --- | --- | --- |
| `waypoints_file` | string | `""` | 点位表；留空用包内 `config/waypoints.yaml` |
| `host` | string | `192.168.11.10` | 底盘 REST 主机（现场按网段修改） |
| `rest_port` | int | `9090` | 底盘 REST 端口 |
| `request_timeout_s` | double | `10.0` | 单次 HTTP 请求超时 (s) |
| `dry_run` | bool | `false` | 🔴 `true`=不驱动底盘，仅验证 Action 链路 |
| `default_timeout_s` | double | `60.0` | goal 未给 `timeout_s` 时的兜底超时 (s) |
| `poll_period_s` | double | `0.2` | 轮询周期 (s) |
| `feedback_period_s` | double | `0.1` | feedback 周期 (s) |
| `max_retries` | int | `1` | 超时重试次数（规划方案 §7.6） |
| `apply_speed_limit` | bool | `false` | 启动时是否设定底盘**全局**限速 |
| `max_moving_speed_mps` | double | `0.4` | 【仅 `apply_speed_limit:=true`】线速度上限 |
| `max_angular_speed_radps` | double | `0.5` | 【仅 `apply_speed_limit:=true`】角速度上限 |

节点内另有未被 launch 暴露的参数（可用 `--ros-args -p` 覆盖）：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `arrive_pos_tol_m` | `0.10` | 到点位置容差 (m) |
| `arrive_yaw_tol_deg` | `2.0` | 到点角度容差 (°) |
| `still_confirm_s` | `0.5` | 静止确认时长 (s) |
| `still_pos_tol_m` / `still_yaw_tol_deg` | `0.02` / `0.5` | 静止判定阈值 |
| `min_confidence` | `0.6` | 定位最低置信度 |
| `retry_backoff_s` | `2.0` | 重试前等待 (s) |
| `relocalize_on_failure` | `true` | 重试耗尽后是否触发重定位 |
| `relocalize_wait_s` | `5.0` | 重定位等待 (s) |
| `dry_run_duration_s` | `6.0` | dry-run 模拟耗时 (s) |
| `range_limit_x` / `range_limit_y` | `[-3.0, 3.0]` | 围栏兜底（YAML 缺失时用） |

---

## 8. 启动与调用

### 8.1 前置条件（实机）

1. 底盘固件在运行，REST 可达：

```bash
curl http://192.168.11.10:9090/api/core/system/v1/robot/health
```

2. 本机已在底盘 IP 白名单内；
3. 已完成现场建图与定位（`get_pose()` 有效）；
4. 🔴 **急停完全弹出、工作人员持急停就位**（底盘 SDK **无软件急停**）。

### 8.2 启动

```bash
# 正常启动
ros2 launch greeting_nav navigate_to.launch.py

# 无真机自测（不驱动底盘）
ros2 launch greeting_nav navigate_to.launch.py dry_run:=true

# 启动时设定全局限速
ros2 launch greeting_nav navigate_to.launch.py apply_speed_limit:=true \
  max_moving_speed_mps:=0.4 max_angular_speed_radps:=0.5

# 现场网段不同
ros2 launch greeting_nav navigate_to.launch.py host:=192.168.11.20
```

### 8.3 发送导航目标

```bash
ros2 action send_goal /greeting/navigate_to greeting_interfaces/action/NavigateTo \
  "{waypoint: standby, timeout_s: 30.0}" --feedback
```

### 8.4 编程调用

```python
from rclpy.action import ActionClient
from greeting_interfaces.action import NavigateTo

cli = ActionClient(node, NavigateTo, "/greeting/navigate_to")
cli.wait_for_server(timeout_sec=3.0)

goal = NavigateTo.Goal()
goal.waypoint = "explain_point"
goal.timeout_s = 60.0
handle = cli.send_goal_async(goal).result()
result = handle.get_result_async().result()
print(result.result.success, result.result.message)
```

---

## 9. 验证与调试

```bash
# 服务端在线？
ros2 action list | grep navigate_to
ros2 node info /greeting_navigate_to

# 点位与围栏是否加载（启动日志）
#   "点位表已加载: ... （frame_id=map，N 个点位: [...]）"
#   "坐标围栏（应用层软限位）: x∈(...) y∈(...)"

# 直接验证底盘连通性（不经过 ROS）
python3 -c "from greeting_nav.chassis_client import ChassisClient; c=ChassisClient(); print(c.get_pose()); c.close()"
```

**常见问题**

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 目标被 REJECT | 点位名不在表内 / 坐标超出围栏 | 对照 `waypoints.yaml`；重标坐标 |
| `底盘 REST 不可达` | 网络/IP 白名单/固件未运行 | 用 `curl` 验证；检查 `host` |
| `底盘健康异常` | 底盘有 warning/error/fatal | 排查底盘 DTC，转「原地接待」 |
| `定位不可用（重定位后仍无效）` | SLAM 丢失 / 精度下降 | 现场重新标定定位后再试 |
| 导航一直超时 | 距离过远 / 障碍 / 速度过慢 | 提高 `timeout_s`；检查路径与限速 |
| 到点但报超时 | 容差过严 / 底盘停不稳 | 放宽 `arrive_pos_tol_m` / `still_*` |
| 报到达但位置偏差大 | 容差过大 | 收紧 `arrive_pos_tol_m` / `arrive_yaw_tol_deg` |
| dry-run 时机器人不动 | **正常**，dry-run 不驱动底盘 | 现场置 `dry_run:=false` |

---

## 10. 注意事项

- 🔴 底盘 SDK **无软件急停**、无导航暂停/恢复：安全依赖**物理急停 + 工作人员就近值守**。
- 🔴 `dry_run` 默认 `false`；**首次联调务必先 `dry_run:=true` 验证 Action 链路**，再上实机。
- 🔴 点位坐标与围栏必须现场标定；`range_limit` 是「应用层 + 物理护栏」双重保障中的**应用层**一重。
- 🔴 `set_speed_params` 是**全局**限速，默认不启用，避免意外改动底盘全局设置。
- 🔴 `move_to` 是 fire-and-forget，必须自行轮询位姿判定到达（本服务端已内建）。
- 🔴 本方案不能替代实机测试，最终效果必须现场验证。

---

## 11. 相关文档

- [《接口契约冻结表》v1.0](../../接口契约冻结表.md) §5 / §7.6
- [《酒店迎宾项目规划方案》](../../酒店迎宾项目规划方案.md) §3.3 / §7.6
- [greeting_interfaces/README.md](../greeting_interfaces/README.md)（`NavigateTo` 接口定义）
- [greeting_orchestrator/README.md](../greeting_orchestrator/README.md)（导航的唯一调用方）