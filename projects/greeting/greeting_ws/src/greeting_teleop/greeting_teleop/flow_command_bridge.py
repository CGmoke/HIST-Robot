#!/usr/bin/env python3
"""flow_command_bridge —— 遥控按键命令 -> 编排层控制指令桥（greeting_teleop）。

作用：把 joy_mapper 发布的 /greeting/panel_command（std_msgs/String）里的
     【全流程命令】转成 /greeting/control_cmd（greeting_interfaces/ConsoleCommand），
     使【手柄按键即可启动整段大会流程】——动作与语音的同步由 orchestrator 的
     cue 步骤编排（动作比语音早 lead_s 触发），本桥不做任何时序处理。

为什么需要本节点：
     panel_command_bridge 只把 `motion:<名>` 直连到动作 Action；
     而 orchestrator 订阅的是 /greeting/control_cmd（ConsoleCommand）。
     两者话题类型不同，此前没有转换节点，导致全流程命令无法驱动讲稿。

输入：/greeting/panel_command   std_msgs/String        （reliable / depth 10，契约 §8 T2）
输出：/greeting/control_cmd     greeting_interfaces/ConsoleCommand（reliable / depth 10）

命令处理：
  · start / pause / resume / next / prev / stop / manual -> 原样转发（arg=''）
  · goto:<段号>                                          -> cmd='goto'，arg='<段号>'
  · motion:<动作名>                                      -> 忽略（由 panel_command_bridge 直连动作）
  · 其余非法命令                                         -> 忽略并告警

🔴 单一指挥者：本节点只转发【命令】，不调用任何 Action、不直接发布 /arm|/head|/waist/cmd。
🔴 讲稿推进、动作与语音的时序（cue 的 lead_s）与安全预检全部由 orchestrator / 动作服务端决定。

参数：
  command_topic  输入命令话题，默认 /greeting/panel_command
  control_topic  输出控制话题，默认 /greeting/control_cmd
  operator_id    操作人标识（追溯用），默认 rc
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from greeting_interfaces.msg import ConsoleCommand

#: 全流程命令（与 ConsoleCommand.cmd 词表、joy_mapper.FLOW_COMMANDS 一致）
FLOW_COMMANDS = {"start", "pause", "resume", "next", "prev", "stop", "manual"}
MOTION_PREFIX = "motion:"
GOTO_PREFIX = "goto:"


class FlowCommandBridge(Node):
    """把 /greeting/panel_command 的全流程命令转成 /greeting/control_cmd 的 ConsoleCommand。"""

    def __init__(self) -> None:
        super().__init__("greeting_flow_bridge")
        self.declare_parameter("command_topic", "/greeting/panel_command")
        self.declare_parameter("control_topic", "/greeting/control_cmd")
        self.declare_parameter("operator_id", "rc")

        command_topic = str(self.get_parameter("command_topic").value)
        control_topic = str(self.get_parameter("control_topic").value)
        self._operator_id = str(self.get_parameter("operator_id").value)

        # 契约 §8：指令类话题 reliable / depth 10（按键命令与主持人指令都不可丢）
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(ConsoleCommand, control_topic, qos)
        self.create_subscription(String, command_topic, self._on_command, qos)

        self.get_logger().info(
            f"flow_command_bridge 就绪：{command_topic}(String) -> "
            f"{control_topic}(ConsoleCommand)；operator_id={self._operator_id}"
        )

    # ------------------------------------------------------------------ 命令入口
    def _on_command(self, msg: String) -> None:
        raw = msg.data.strip()
        if not raw:
            return
        low = raw.lower()

        if low.startswith(MOTION_PREFIX):
            # 动作快捷键由 panel_command_bridge 直连动作 Action，本桥不处理
            return

        if low.startswith(GOTO_PREFIX):
            arg = raw[len(GOTO_PREFIX):].strip()
            if not arg:
                self.get_logger().warn(f"命令 {raw!r} 缺少段号，已忽略")
                return
            self._forward("goto", arg)
            return

        if low in FLOW_COMMANDS:
            self._forward(low, "")
            return

        self.get_logger().warn(f"非全流程命令，已忽略：{raw!r}")

    def _forward(self, cmd: str, arg: str) -> None:
        out = ConsoleCommand()
        out.cmd = cmd
        out.arg = arg
        out.operator_id = self._operator_id
        self._pub.publish(out)
        self.get_logger().info(f"转发全流程命令 -> cmd={cmd!r} arg={arg!r}")


def main() -> None:
    rclpy.init()
    node = FlowCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()