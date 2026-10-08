#!/usr/bin/env python3
"""panel_command_bridge —— 遥控按键命令 -> 动作 Action 直连桥（greeting_teleop）。

作用：把 joy_mapper 发布的 /greeting/panel_command（std_msgs/String）里的
     `motion:<动作名>` 直接转成 /greeting/play_motion（PlayMotion）的 Action Goal，
     使【手柄按键即可直接触发礼仪动作】，无需经过 orchestrator。

输入：/greeting/panel_command   std_msgs/String   （reliable / depth 10，契约 §8 T2）
输出：/greeting/play_motion     greeting_interfaces/action/PlayMotion（Action Client）

命令处理：
  · motion:<动作名>  -> 发送 Action Goal（speed_scale 钳到 ≤ speed_scale_max）
  · stop / manual    -> 取消正在执行的动作（配合 teleop 的 guard.release 中止动作）
  · 其余命令         -> 忽略（start/pause/resume/next/prev/goto 属 orchestrator 职责）

🔴 单一指挥者安全：本节点只调用动作 Action，绝不直接发布 /arm|/head|/waist/cmd。
🔴 speed_scale 硬上限 0.7（礼仪硬约束；动作服务端还会再钳一次，属双层兜底）。
🔴 忙碌策略：默认 preempt=false —— 动作执行中收到新动作请求直接忽略并告警，
   避免两条轨迹叠加造成关节冲突。置 true 时先取消当前动作、待其结束后再发新动作。

参数：
  command_topic     命令话题，默认 /greeting/panel_command
  action_name       动作 Action 名，默认 /greeting/play_motion
  speed_scale       动作默认速度缩放，默认 0.5
  speed_scale_max   速度硬上限，默认 0.7（超过 0.7 会被强制降到 0.7）
  preempt           true=新动作抢占正在执行的动作，默认 false
  allowed_motions   动作名白名单（逗号/空格分隔）；留空=不校验（由服务端兜底）
  action_timeout_s  等待 Action 服务端就绪的超时(s)，默认 2.0
"""
from __future__ import annotations

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from greeting_interfaces.action import PlayMotion

MOTION_PREFIX = "motion:"
# 礼仪硬约束：速度缩放上限
SPEED_SCALE_HARD_MAX = 0.7
# 交由 orchestrator 处理、但在本桥用作文"中止动作"的全流程命令
_CANCEL_CMDS = {"stop", "manual"}


class PanelCommandBridge(Node):
    """把 /greeting/panel_command 的 `motion:<名>` 命令转成 PlayMotion Action Goal。"""

    def __init__(self) -> None:
        super().__init__("greeting_panel_bridge")
        self.declare_parameter("command_topic", "/greeting/panel_command")
        self.declare_parameter("action_name", "/greeting/play_motion")
        self.declare_parameter("speed_scale", 0.5)
        self.declare_parameter("speed_scale_max", SPEED_SCALE_HARD_MAX)
        self.declare_parameter("preempt", False)
        self.declare_parameter("allowed_motions", "")
        self.declare_parameter("action_timeout_s", 2.0)

        self._command_topic = str(self.get_parameter("command_topic").value)
        self._action_name = str(self.get_parameter("action_name").value)
        self._timeout = float(self.get_parameter("action_timeout_s").value)
        self._preempt = bool(self.get_parameter("preempt").value)

        # 速度：上限强制 ≤0.7；默认值再取不高于上限
        req_max = abs(float(self.get_parameter("speed_scale_max").value))
        if req_max > SPEED_SCALE_HARD_MAX:
            self.get_logger().warn(
                f"speed_scale_max={req_max} 超礼仪硬上限，强制降为 {SPEED_SCALE_HARD_MAX}"
            )
        self._speed_max = min(req_max, SPEED_SCALE_HARD_MAX) or SPEED_SCALE_HARD_MAX
        asked = abs(float(self.get_parameter("speed_scale").value))
        self._speed_scale = min(asked, self._speed_max)

        raw_allowed = str(self.get_parameter("allowed_motions").value).strip()
        self._allowed = {t for t in raw_allowed.replace(",", " ").split() if t}

        self._goal_handle = None       # 正在执行/待取消的目标句柄；None=空闲
        self._cancelling = False       # 已发起取消、等待结果中
        self._pending_motion = None    # preempt 模式下等待补发的动作名

        self._client = ActionClient(self, PlayMotion, self._action_name)

        # 契约 §8 T2：reliable / depth 10（按键命令不可丢）
        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, self._command_topic, self._on_command, sub_qos)

        self.get_logger().info(
            f"panel_command_bridge 就绪：{self._command_topic}(String) -> "
            f"{self._action_name}(PlayMotion)；speed_scale={self._speed_scale:.2f}"
            f"（上限 {self._speed_max:.2f}）；preempt={self._preempt}；"
            f"白名单={'未启用' if not self._allowed else sorted(self._allowed)}"
        )
        if self._client.wait_for_server(timeout_sec=self._timeout):
            self.get_logger().info(f"{self._action_name} 动作服务端已就绪")
        else:
            # 不致命：动作服务端可能稍后才启动，发送时会再等一次
            self.get_logger().warn(
                f"{self._action_name} 尚未就绪（等待 {self._timeout}s）——"
                "请确认 greeting_body motion_server 已运行"
            )

    # ------------------------------------------------------------------ 命令入口
    def _on_command(self, msg: String) -> None:
        cmd = msg.data.strip()
        if not cmd:
            return

        low = cmd.lower()
        if low in _CANCEL_CMDS:
            if self._goal_handle is None:
                self.get_logger().info(f"收到 {low!r}：当前无动作在执行")
            elif self._cancelling:
                self.get_logger().debug(f"收到 {low!r}：动作已在取消中")
            else:
                self.get_logger().warn(f"收到 {low!r}：取消正在执行的动作")
                self._cancel_active(f"命令 {low}")
            return

        if not cmd.startswith(MOTION_PREFIX):
            # start/pause/resume/next/prev/goto 等由 orchestrator 负责，本桥不处理
            self.get_logger().debug(f"非本桥职责的命令，已忽略：{cmd!r}")
            return

        name = cmd[len(MOTION_PREFIX):].strip()
        if not name:
            self.get_logger().warn(f"命令 {cmd!r} 缺少动作名，已忽略")
            return
        if self._allowed and name not in self._allowed:
            self.get_logger().warn(
                f"动作 {name!r} 不在白名单 {sorted(self._allowed)} 内，已拒绝"
            )
            return

        if self._goal_handle is not None:
            if not self._preempt:
                self.get_logger().warn(
                    f"动作执行中，忽略 {name!r}（如需抢占请设 preempt:=true）"
                )
                return
            # 抢占：先取消当前动作，结果回调里补发
            self._pending_motion = name
            self.get_logger().warn(f"抢占：先取消当前动作，随后补发 {name!r}")
            self._cancel_active(f"被 {name} 抢占")
            return

        self._send_motion(name)

    # ------------------------------------------------------------------ 动作收发
    def _send_motion(self, name: str) -> None:
        if not self._client.server_is_ready() and not self._client.wait_for_server(
            timeout_sec=self._timeout
        ):
            self.get_logger().error(
                f"动作服务端 {self._action_name} 未就绪，{name!r} 无法执行；"
                "请确认 greeting_body motion_server 已启动"
            )
            return

        speed = min(abs(self._speed_scale), self._speed_max)
        goal = PlayMotion.Goal()
        goal.motion_name = name
        goal.speed_scale = float(speed)

        self.get_logger().info(f"触发动作 {name!r}（speed_scale={speed:.2f}）")
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001 — 兜底，避免回调异常中断
            self.get_logger().error(f"发送动作目标失败：{exc}")
            return
        if not goal_handle.accepted:
            self.get_logger().error("动作目标被服务端拒绝（未知动作名或前置检查未过）")
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        progress = float(feedback_msg.feedback.progress)
        self.get_logger().debug(f"动作执行进度 {progress * 100:.0f}%")

    def _on_result(self, future) -> None:
        self._goal_handle = None
        self._cancelling = False
        try:
            wrapped = future.result()
            result = wrapped.result
            ok = bool(getattr(result, "success", False))
            msg = str(getattr(result, "message", ""))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"获取动作结果失败：{exc}")
        else:
            if ok:
                self.get_logger().info(f"动作执行完成：{msg or '成功'}")
            else:
                self.get_logger().error(f"动作执行结束（未成功）：{msg or '未知原因'}")

        # preempt：被抢占后补发等待中的动作
        if self._pending_motion:
            name, self._pending_motion = self._pending_motion, None
            self.get_logger().info(f"补发被抢占的动作 {name!r}")
            self._send_motion(name)

    def _cancel_active(self, reason: str) -> None:
        """发起取消；句柄保留到结果回调，期间的 busy 判定仍成立，避免轨迹叠加。"""
        if self._goal_handle is None or self._cancelling:
            return
        self._cancelling = True
        self.get_logger().info(f"取消动作（{reason}）")
        self._goal_handle.cancel_goal_async()


def main() -> None:
    rclpy.init()
    node = PanelCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()