#!/usr/bin/env python3
"""joy_mapper —— 遥控器(SBUS)按键 -> 迎宾命令 网关（greeting_teleop）。

输入源（按官方平台文档）：
  · /sbus_data/event  类型 bodyctrl_msgs/msg/SbusData  ← 按键/拨杆事件（主输入）
      平台已做边沿解码，字段 key_event_new 直接给出 KEY_A_DOWN / KEY_E_UP ……
      常量，无需自做上升沿检测；并附带 button_a..h 状态与 x1/y1/x2/y2 摇杆。
  · /sbus_data        类型 sensor_msgs/msg/Joy          ← 12 个轴（buttons 为空），
      仅作可选的模拟量扩展输入（默认不订阅）；SbusData 自带的 x1/y1/x2/y2
      已足够覆盖左右摇杆，通常无需再订阅 Joy。

输出：
  · /greeting/panel_command  std_msgs/String（reliable/depth10）
    由 greeting_orchestrator 统一消费（接口契约 §2 T2）。

命令词表（写入 panel_command.data）：
  · 全流程   ： start | pause | resume | next | prev | goto:<段号> | stop | manual
  · 动作快捷键： motion:<动作名>
        动作名取自 greeting_body/config/motions.yaml：
        salute_bow / wave_official / point_right / point_left /
        farewell_bow / photo_pose / idle_scan
  · 一个按键也可同时发多条命令，用 ';' 连接（逐条独立发布），
        如 "stop;motion:salute_bow"。

🔴 单一指挥者：本节点只发布命令，不直接调用任何 Action，也不碰 /head|arm|waist/cmd。
🔴 按键映射在 config/teleop_map.yaml 中按【命名键】配置；置 monitor:=true 只打印
   原始 SbusData（key_event / button_* / 摇杆）用于标定，不发布任何命令。

命名键（与 bodyctrl_msgs/SbusData.KEY_* 一一对应）：
  点动按键： a  a_up  |  b  b_up  |  c  c_up  |  d  d_up
            （a=按下瞬间触发；a_up=松开瞬间触发，按需使用）
  三档 E/F ： e_up  e_mid  e_down | f_up  f_mid  f_down
  左右 G/H ： g_left g_mid g_right| h_left h_mid h_right

摇杆命名键（来自 SbusData.x1/y1/x2/y2，方向上升沿触发，无需订阅 Joy）：
  x1_high  x1_low   （左摇杆 X：右/左）
  y1_high  y1_low   （左摇杆 Y：上/下，文档定义 y1 下=-1 上=+1）
  x2_high  x2_low   （右摇杆 X：右/左）
  y2_high  y2_low   （右摇杆 Y：上/下）

button_* 状态字段语义（仅作 monitor 观察，不用于触发）：
  -1=松开/复位, 0=中间位置, 1=按下/拨动到一端, 2=拨动到另一端。
  触发以 key_event_new 边沿事件为准，状态字段仅辅助标定。

🔴 组合键（前提 Guard）支持【多组】：`guards` 为一个列表，每组有自己的前提键与按键表。
   运行时按【列表顺序】取第一个"当前生效"的组；都不生效时回退到常驻按键表 `keys`。
   下面是 `guards` 的一个典型写法示例（实际生效的映射以 config/teleop_map.yaml 为准）：
       guards:
         - key: e_up          # 前提：E 三档开关在"上"
           release: manual    # 前提解除时下发（可选）：中止正在播放的动作
           keys:
             a: motion:salute_bow
             b: motion:wave_official
             c: motion:photo_pose
         - key: g_left        # 前提：G 拨到"左"
           keys: {a: 'goto:1', b: 'goto:4', c: 'goto:7'}
         - key: g_mid         # 前提：G 拨到"中"
           keys: {a: 'goto:2', b: 'goto:5'}
         - key: g_right       # 前提：G 拨到"右"
           keys: {a: 'goto:3', b: 'goto:6'}
   判定依据为事件值（KEY_E_UP / KEY_G_LEFT …），不依赖 button_* 状态位
   （官方文档未说明其 1/2 哪端为"上"）。同一开关族内各组互斥；不同族可并存，
   靠列表顺序定优先级（故把"单动作组"排在前，E 不在"上"时才落到 G 分段组）。
   ⚠️ 三档开关【总有一档在生效】，因此 G 组在某档位会持续生效（上电首次拨动 G 后）。
   前提默认【未生效】，上电后需先拨动对应开关才会激活该组。

参数：
  event_topic       SBUS 事件话题，默认 /sbus_data/event
  joy_topic         Joy 轴话题，默认空字符串（不订阅）；填 "/sbus_data" 可启用 12 轴映射
  command_topic     命令话题，默认 /greeting/panel_command
  map_file          映射表 YAML；留空用 share/greeting_teleop/config/teleop_map.yaml
  deadband          轴阈值（|value|>deadband 才算拨动），默认 0.5
  operator_id       操作人标识（追溯用），默认 rc
  monitor           true=标定模式：只打印、不发布，默认 false
  monitor_period_s  标定模式打印周期(s)，默认 2.0
"""
from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from bodyctrl_msgs.msg import SbusData
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# 仅当 joy_topic 非空时才导入 sensor_msgs，避免在无轴映射场景下的多余依赖
from sensor_msgs.msg import Joy  # noqa: E402  (延迟使用，保留导入)

# 全流程命令（与 greeting_orchestrator._handle_command 对齐）
FLOW_COMMANDS = {"start", "pause", "resume", "next", "prev", "stop", "manual"}
MOTION_PREFIX = "motion:"
GOTO_PREFIX = "goto:"

# 命名键 -> SbusData.KEY_* 常量
KEY_MAP = {
    # 点动按键 A/B/C/D：按下触发；a_up 等=松开触发（按需使用）
    "a": SbusData.KEY_A_DOWN,
    "a_up": SbusData.KEY_A_UP,
    "b": SbusData.KEY_B_DOWN,
    "b_up": SbusData.KEY_B_UP,
    "c": SbusData.KEY_C_DOWN,
    "c_up": SbusData.KEY_C_UP,
    "d": SbusData.KEY_D_DOWN,
    "d_up": SbusData.KEY_D_UP,
    # 三档开关 E/F：上/中/下
    "e_up": SbusData.KEY_E_UP,
    "e_mid": SbusData.KEY_E_MID,
    "e_down": SbusData.KEY_E_DOWN,
    "f_up": SbusData.KEY_F_UP,
    "f_mid": SbusData.KEY_F_MID,
    "f_down": SbusData.KEY_F_DOWN,
    # 左右拨杆 G/H：左/中/右
    "g_left": SbusData.KEY_G_LEFT,
    "g_mid": SbusData.KEY_G_MID,
    "g_right": SbusData.KEY_G_RIGHT,
    "h_left": SbusData.KEY_H_LEFT,
    "h_mid": SbusData.KEY_H_MID,
    "h_right": SbusData.KEY_H_RIGHT,
}

# 摇杆命名键 -> (SbusData 字段名, 方向 high/low)
# 文档：x1 左(-1)↔右(+1)，y1 下(-1)↔上(+1)，x2/y2 同理
STICK_MAP = {
    "x1_high": ("x1", "high"),
    "x1_low": ("x1", "low"),
    "y1_high": ("y1", "high"),
    "y1_low": ("y1", "low"),
    "x2_high": ("x2", "high"),
    "x2_low": ("x2", "low"),
    "y2_high": ("y2", "high"),
    "y2_low": ("y2", "low"),
}

# 事件值 -> 命名键（反向查表，用于日志与前提判定）
EVENT_NAMES = {v: k for k, v in KEY_MAP.items()}


def _event_family(name: str) -> str:
    """取命名键所属的物理开关族：'e_up'/'e_mid'/'e_down' -> 'e'；'a'/'a_up' -> 'a'。"""
    return name.split("_", 1)[0]


class JoyMapper(Node):
    def __init__(self) -> None:
        super().__init__("greeting_joy_mapper")
        self.declare_parameter("event_topic", "/sbus_data/event")
        self.declare_parameter("joy_topic", "")
        self.declare_parameter("command_topic", "/greeting/panel_command")
        self.declare_parameter("map_file", "")
        self.declare_parameter("deadband", 0.5)
        self.declare_parameter("operator_id", "rc")
        self.declare_parameter("monitor", False)
        self.declare_parameter("monitor_period_s", 2.0)

        self._event_topic = str(self.get_parameter("event_topic").value)
        self._joy_topic = str(self.get_parameter("joy_topic").value)
        self._cmd_topic = str(self.get_parameter("command_topic").value)
        self._deadband = float(self.get_parameter("deadband").value)
        self._operator_id = str(self.get_parameter("operator_id").value)
        self._monitor = bool(self.get_parameter("monitor").value)
        self._monitor_period = float(self.get_parameter("monitor_period_s").value)

        self._key_cmds, self._stick_cmds, self._axes, self._guard_groups = self._load_map(
            str(self.get_parameter("map_file").value)
        )

        self._axis_prev: dict[int, str] = {}
        self._stick_prev: dict[str, str] = {}   # 字段名 -> 上次方向 high/low/none
        self._last_key_event: int = SbusData.KEY_NONE
        # 各前提组的生效状态由 _guard_groups[i]["active"] 维护（默认全 False，安全）
        self._last_monitor_log = 0.0

        # 契约 §8 T2：reliable / depth 10
        pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(String, self._cmd_topic, pub_qos)

        # 按键事件：平台侧通常为 reliable（事件不可丢），订阅端对齐 reliable。
        event_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._event_sub = self.create_subscription(
            SbusData, self._event_topic, self._on_event, event_qos
        )

        # 可选：Joy 12 轴（/sbus_data 的 buttons 为空，仅轴有用）
        self._joy_sub = None
        if self._joy_topic:
            joy_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self._joy_sub = self.create_subscription(
                Joy, self._joy_topic, self._on_joy, joy_qos
            )

        mode = "【标定模式·只打印不发布】" if self._monitor else "工作模式"
        guard_info = (
            "；组合键前提组 "
            + str(len(self._guard_groups))
            + " 个（按优先级："
            + "、".join(g["label"] for g in self._guard_groups)
            + "）"
            if self._guard_groups else ""
        )
        self.get_logger().info(
            f"joy_mapper 就绪（{mode}）："
            f"{self._event_topic}(SbusData){' + ' + self._joy_topic + '(Joy)' if self._joy_topic else ''}"
            f" -> {self._cmd_topic}；按键映射 {len(self._key_cmds)} 项"
            f" / 摇杆映射 {len(self._stick_cmds)} 项"
            f"{' / Joy 轴映射 ' + str(len(self._axes)) + ' 项' if self._axes else ''}"
            f"{guard_info}；operator_id={self._operator_id}"
        )
        if not self._key_cmds and not self._stick_cmds and not self._axes \
                and not any(g["cmds"] for g in self._guard_groups):
            self.get_logger().warn("映射表为空：将不会产生任何命令（可置 monitor:=true 标定）")

    # ------------------------------------------------------------------ 映射表
    def _load_map(self, path: str) -> tuple[dict, dict, dict, list]:
        if not path:
            path = str(
                Path(get_package_share_directory("greeting_teleop"))
                / "config"
                / "teleop_map.yaml"
            )
        # 组合键前提组列表（空列表 = 未启用任何组合键）
        guards: list[dict] = []
        try:
            data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"读取映射表失败 {path}: {exc}")
            return {}, {}, {}, guards

        # 命名键 -> 命令
        key_cmds: dict[int, str] = {}
        for name, cmd in (data.get("keys") or {}).items():
            name = str(name).strip().lower()
            if name not in KEY_MAP:
                self.get_logger().warn(
                    f"未知命名键 {name!r}（合法值：a/a_up..h_right），已忽略"
                )
                continue
            key_cmds[KEY_MAP[name]] = str(cmd)

        # 兼容旧写法：buttons: {<SbusData 常量int>: 命令}
        for k, v in (data.get("buttons") or {}).items():
            try:
                key_cmds[int(k)] = str(v)
            except ValueError:
                self.get_logger().warn(f"buttons 键非整数 {k!r}，已忽略")

        # 摇杆映射（来自 SbusData.x1/y1/x2/y2，方向上升沿触发）
        stick_cmds: dict[str, str] = {}
        for name, cmd in (data.get("sticks") or {}).items():
            name = str(name).strip().lower()
            if name not in STICK_MAP:
                self.get_logger().warn(
                    f"未知摇杆键 {name!r}（合法值：x1_high/x1_low..y2_high/y2_low），已忽略"
                )
                continue
            stick_cmds[name] = str(cmd)

        # Joy 轴映射（仅当启用 joy_topic 时生效）：轴号 -> {high: 命令, low: 命令}
        axes: dict[int, dict] = {}
        for k, v in (data.get("axes") or {}).items():
            if isinstance(v, dict):
                entry = {d: str(v[d]) for d in ("high", "low") if v.get(d)}
            else:
                entry = {"high": str(v)}
            if entry:
                axes[int(k)] = entry

        # 组合键前提组（多组）：guards 为列表，按顺序即优先级；
        # 兼容旧写法单个 guard 块（自动并入列表，优先级最低）。
        guards_raw = data.get("guards")
        if guards_raw is None:
            legacy = data.get("guard")
            guards_raw = [legacy] if legacy else []
        if not isinstance(guards_raw, list):
            self.get_logger().warn("guards 应为列表（每项含 key/release/keys），已忽略")
            guards_raw = []
        for i, graw in enumerate(guards_raw):
            if not isinstance(graw, dict):
                self.get_logger().warn(f"guards[{i}] 非映射结构，已忽略")
                continue
            gkey = str(graw.get("key", "")).strip().lower()
            if not gkey:
                self.get_logger().warn(f"guards[{i}] 缺少 key，已忽略该组")
                continue
            if gkey not in KEY_MAP:
                self.get_logger().warn(
                    f"guards[{i}].key 非法 {gkey!r}（合法值：a/e_up/g_left 等），已忽略该组"
                )
                continue
            cmds: dict[int, str] = {}
            for name, cmd in (graw.get("keys") or {}).items():
                name = str(name).strip().lower()
                if name in KEY_MAP:
                    cmds[KEY_MAP[name]] = str(cmd)
                else:
                    self.get_logger().warn(f"guards[{i}].keys 键名非法 {name!r}，已忽略")
            release = str(graw.get("release", "") or "")
            guards.append(
                {
                    "event": KEY_MAP[gkey],
                    "family": _event_family(gkey),
                    "label": gkey,
                    "release": release,
                    "cmds": cmds,
                    "active": False,
                }
            )
            self.get_logger().info(
                f"组合键前提组[{i}] 已加载：{gkey} -> {len(cmds)} 项"
                f"{'，解除时下发 ' + release if release else ''}"
            )

        self.get_logger().info(f"映射表已加载: {path}")
        return key_cmds, stick_cmds, axes, guards

    # ------------------------------------------------------------------ 回调
    def _on_event(self, msg: SbusData) -> None:
        """SbusData 事件回调：key_event_new 已为边沿事件，直接查表触发；
        先更新组合键前提状态，再按前提选择映射表；同时处理摇杆方向。"""
        if self._monitor:
            # 标定模式：不发布，但仍追踪前提状态以便观察
            ev = int(msg.key_event_new)
            if ev == SbusData.KEY_NONE:
                self._last_key_event = SbusData.KEY_NONE
            elif ev != self._last_key_event:
                self._last_key_event = ev
                self._update_guards(ev)
            self._log_monitor(msg)
            return
        ev = int(msg.key_event_new)
        if ev == SbusData.KEY_NONE:
            self._last_key_event = SbusData.KEY_NONE
            self._handle_sticks(msg)
            return
        # 去重：同一事件可能因多网卡 DDS 多路径投递而重复到达，只处理一次
        if ev != self._last_key_event:
            self._last_key_event = ev
            self._update_guards(ev)                # 先更新各组前提状态
            cmd = self._resolve_cmd(ev)
            if cmd:
                self._emit(cmd, f"key_event={self._event_name(ev)}")
            else:
                self.get_logger().debug(
                    f"未映射按键事件 {self._event_name(ev)}（标定可用 monitor）"
                )
        # 摇杆：每帧都处理方向变化（SbusData 持续发布，摇杆值每帧更新）
        self._handle_sticks(msg)

    def _update_guards(self, ev: int) -> None:
        """依据事件更新各前提组的生效状态：只有同一开关族的组会被刷新（族内互斥），
        不同族可同时生效；优先级由 guards 列表顺序决定（见 _resolve_cmd）。"""
        name = EVENT_NAMES.get(ev)
        if name is None:
            return
        fam = _event_family(name)
        for g in self._guard_groups:
            if g["family"] != fam:
                continue
            was = g["active"]
            g["active"] = (ev == g["event"])
            if g["active"] != was:
                self.get_logger().info(
                    f"[guard] 前提 {g['label']} {'生效' if g['active'] else '解除'}"
                )
            # 前提解除时可选下发（如 manual：中止正在播放的动作）；标定模式不发布
            if was and not g["active"] and g["release"] and not self._monitor:
                self._emit(g["release"], f"guard[{g['label']}]解除")

    def _resolve_cmd(self, ev: int) -> str | None:
        """按 guards 顺序取第一个"当前生效"组的命令；都不命中时回退常驻按键表。"""
        for g in self._guard_groups:
            if g["active"]:
                cmd = g["cmds"].get(ev)
                if cmd:
                    return cmd
        return self._key_cmds.get(ev)

    def _handle_sticks(self, msg: SbusData) -> None:
        """对 x1/y1/x2/y2 做方向上升沿检测并触发映射命令。"""
        for name, (field, direction) in STICK_MAP.items():
            if name not in self._stick_cmds:
                continue
            val = float(getattr(msg, field))
            if val > self._deadband:
                state = "high"
            elif val < -self._deadband:
                state = "low"
            else:
                state = "none"
            prev = self._stick_prev.get(field, "none")
            if state != prev and state == direction:
                self._emit(self._stick_cmds[name], f"stick[{field}]={val:+.2f}->{direction}")
            self._stick_prev[field] = state

    def _on_joy(self, msg: Joy) -> None:
        """可选 Joy 轴回调：按方向上升沿触发。"""
        if self._monitor:
            return
        for i, val in enumerate(msg.axes):
            mapping = self._axes.get(i)
            if not mapping:
                continue
            if val > self._deadband:
                state = "high"
            elif val < -self._deadband:
                state = "low"
            else:
                state = "none"
            prev = self._axis_prev.get(i, "none")
            if state != prev and state != "none":
                cmd = mapping.get(state)
                if cmd:
                    self._emit(cmd, f"axis[{i}]={val:+.2f}->{state}")
            self._axis_prev[i] = state

    # ------------------------------------------------------------------ 发布
    def _emit(self, cmd_blob: str, source: str) -> None:
        """发布命令；支持用 ';' 连接多条（逐条独立发布）。"""
        for raw in cmd_blob.split(";"):
            cmd = raw.strip()
            if not cmd:
                continue
            if not self._is_valid(cmd):
                self.get_logger().warn(f"{source} 映射到非法命令 {cmd!r}，已丢弃")
                continue
            self._pub.publish(String(data=cmd))
            self.get_logger().info(f"[{source}] 发布命令 -> {self._cmd_topic}: {cmd!r}")

    def _is_valid(self, cmd: str) -> bool:
        if cmd in FLOW_COMMANDS:
            return True
        if cmd.startswith(MOTION_PREFIX):
            return len(cmd) > len(MOTION_PREFIX)
        if cmd.startswith(GOTO_PREFIX):
            return cmd[len(GOTO_PREFIX):].strip().isdigit()
        return False

    # ------------------------------------------------------------------ 标定
    def _event_name(self, ev: int) -> str:
        return EVENT_NAMES.get(ev, f"UNKNOWN({ev})")

    def _log_monitor(self, msg: SbusData) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_monitor_log < self._monitor_period:
            return
        self._last_monitor_log = now
        ev_new = self._event_name(int(msg.key_event_new))
        ev_old = self._event_name(int(msg.key_event_old))
        btns = (
            f"a={msg.button_a} b={msg.button_b} c={msg.button_c} d={msg.button_d} "
            f"e={msg.button_e} f={msg.button_f} g={msg.button_g} h={msg.button_h}"
        )
        sticks = f"x1={msg.x1:+.2f} y1={msg.y1:+.2f} x2={msg.x2:+.2f} y2={msg.y2:+.2f}"
        guard = ""
        if self._guard["family"] is not None:
            guard = f" | 前提[{self._guard['label']}]: {'生效' if self._guard_on else '未生效'}"
        self.get_logger().info(
            f"[monitor] key_event {ev_old} -> {ev_new} | buttons[{btns}] | sticks[{sticks}]{guard}"
        )


def main() -> None:
    rclpy.init()
    node = JoyMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
