#!/usr/bin/env python3
"""speak_action_server —— 实机 /greeting/speak 动作服务端（greeting_voice / 语音 Owner B）。

契约依据：接口契约冻结表 §4 / §5.3 —— 迎宾编排层只通过 /greeting/speak
（greeting_interfaces/action/Speak）驱动播报；A1 服务端由 B 负责。

本节点把 Speak Goal 落到本体 TTS 服务 /intelligent_interaction/tts/play
（interaction_msgs/srv/TtsService，字段 {text, type, cmd, audio_id, extra}）：

    1. cmd='query' 先查播报状态（playing 时仅记录，append 即排队）；
    2. cmd='append', type='text', audio_id=<audio_id> 排队播报目标文本；
    3. 轮询 cmd='query' 直到播报结束（或到按文本长度估算的兜底时长），
       期间按 Speak.Feedback 发布 progress / speaking_text；
    4. 目标被取消（如遥控急停）时，用 cmd='stop', audio_id=<audio_id> 真正打断音频。

🔴 audio_id 必须是 audio_config 中开启 "stop": true 的文本型实体（默认 audio_guide）；
   纯 stop 不带 audio_id 会被平台忽略（"stop requires audio_id"），音频会播完。

与 voice_greet_tts_node 的区别：
    · voice_greet_tts_node = 遥控器按键【人工】触发播报；
    · 本节点 = 编排层动作驱动【自动】播报，供"动作比语音早 1s/3s 触发"的时序编排使用。
    两者共用同一 TTS 服务，可同时运行（append 会排队）。

🔴 执行回调里会阻塞轮询（必须 ReentrantCallbackGroup + MultiThreadedExecutor），
   否则 cancel / 其它回调无法被处理。
"""
from __future__ import annotations

import time

import rclpy
from greeting_interfaces.action import Speak
from interaction_msgs.srv import TtsService
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class SpeakActionServer(Node):
    """把 /greeting/speak（Speak）转成本体 TTS 播报。"""

    def __init__(self) -> None:
        super().__init__("greeting_speak_server")
        self.declare_parameter("tts_service", "/intelligent_interaction/tts/play")
        #: 播报所用的 TTS 实体（audio_id）。必须选用 audio_config 中带 "stop": true 的
        #: 文本型实体（当前为 audio_guide），否则急停时本体 TTS 无法被打断
        #: （平台规则：stop 必须带 audio_id，且该 audio_id 需开启 stop 权限）。
        self.declare_parameter("audio_id", "audio_guide")
        self.declare_parameter("service_timeout_s", 5.0)
        self.declare_parameter("poll_period_s", 0.2)
        #: 起播宽限：一直没看到 'playing' 时，至少等这么久才允许按估算结束
        self.declare_parameter("start_grace_s", 1.0)
        #: 连续多久查询为非 'playing' 才判定播报结束（防两帧之间的空窗误判）
        self.declare_parameter("settle_s", 0.4)
        #: 文本 -> 时长的兜底估算（中文 TTS 约每字秒数）
        self.declare_parameter("estimate_s_per_char", 0.22)
        #: 兜底最短播报时长(s)
        self.declare_parameter("min_duration_s", 1.5)
        #: 兜底硬上限：估时长 × (1+hard_factor) + hard_margin_s，超过即强制结束
        self.declare_parameter("hard_factor", 1.0)
        self.declare_parameter("hard_margin_s", 5.0)

        self._tts = str(self.get_parameter("tts_service").value)
        self._audio_id = str(self.get_parameter("audio_id").value)
        self._timeout = float(self.get_parameter("service_timeout_s").value)
        self._poll = float(self.get_parameter("poll_period_s").value)
        self._grace = float(self.get_parameter("start_grace_s").value)
        self._settle = float(self.get_parameter("settle_s").value)
        self._s_per_char = float(self.get_parameter("estimate_s_per_char").value)
        self._min_dur = float(self.get_parameter("min_duration_s").value)
        self._hard_factor = float(self.get_parameter("hard_factor").value)
        self._hard_margin = float(self.get_parameter("hard_margin_s").value)

        self._cli = self.create_client(TtsService, self._tts)
        self._group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self,
            Speak,
            "/greeting/speak",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._group,
        )
        self.get_logger().info(
            f"实机 Speak 服务端就绪: /greeting/speak -> {self._tts}"
            f"（query 查态 + append 排队播报；audio_id={self._audio_id}，"
            f"取消时以该 audio_id 下发 stop 打断音频）"
        )

    # ---------------------------------------------------------------- 回调
    def _on_goal(self, goal_request: Speak.Goal) -> GoalResponse:
        if not goal_request.text:
            self.get_logger().warn("拒绝空文本的 Speak 目标")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> Speak.Result:
        req = goal_handle.request
        text = req.text
        if req.interrupt:
            # 契约 §5.3 的 interrupt=true：带 audio_id 下发 stop 打断当前播报，再排队新文本
            self.get_logger().info("interrupt=true：先打断当前播报再排队新文本")
            self._stop_tts()

        if not self._cli.wait_for_service(timeout_sec=self._timeout):
            self.get_logger().error(f"TTS 服务 {self._tts} 未就绪，播报目标中止")
            goal_handle.abort()
            return self._result(False, f"TTS 服务 {self._tts} 未就绪")

        # 1) 先查态（playing 时仅记录，append 排队）
        state = self._call("query", "")
        if state is not None and state.success and state.status == "playing":
            self.get_logger().info("当前正在播报，本次目标将排队（append）")

        # 2) 排队播报
        if req.resume_point > 0.0:
            # 契约 §5.3：resume_point>0 表示续播；TTS 无定位接口，无法实现，按从头播报
            self.get_logger().warn(
                f"resume_point={req.resume_point:.2f}>0：TTS 无定位能力，按从头播报"
            )
        appended = self._call("append", text, self._audio_id)
        if appended is None or not appended.success:
            self.get_logger().error("TTS append 未成功")
            goal_handle.abort()
            return self._result(False, "TTS append 未成功")

        # 3) 轮询直到播报结束
        est = max(self._min_dur, len(text) * self._s_per_char)
        hard = est * (1.0 + self._hard_factor) + self._hard_margin
        t0 = time.monotonic()
        seen_playing = False
        idle_since = None
        while True:
            elapsed = time.monotonic() - t0
            if goal_handle.is_cancel_requested:
                # 真正打断本体 TTS：必须带 audio_id 下发 stop，纯 stop 会被平台忽略
                self._stop_tts()
                goal_handle.canceled()
                msg = f"播报目标被取消，已打断音频（已等 {elapsed:.1f}s）"
                self.get_logger().warn(msg)
                return self._result(False, msg)

            q = self._call("query", "")
            playing = bool(q is not None and q.success and q.status == "playing")
            if playing:
                seen_playing = True
                idle_since = None
            elif idle_since is None:
                idle_since = elapsed

            done = (
                seen_playing
                and idle_since is not None
                and elapsed - idle_since >= self._settle
            )
            if not done and not seen_playing and elapsed >= est + self._grace:
                done = True
            if not done and elapsed >= hard:
                self.get_logger().warn(
                    f"播报时长超兜底上限 {hard:.1f}s，强制判定结束（检查 TTS 状态回读）"
                )
                done = True
            if done:
                break

            fb = Speak.Feedback()
            fb.progress = float(min(elapsed / est, 1.0))
            fb.speaking_text = text
            goal_handle.publish_feedback(fb)
            time.sleep(self._poll)

        total = time.monotonic() - t0
        goal_handle.succeed()
        msg = f"播报结束（{total:.1f}s，seen_playing={seen_playing}）"
        self.get_logger().info(msg)
        return self._result(True, msg)

    # ---------------------------------------------------------------- 工具
    def _call(self, cmd: str, text: str, audio_id: str = ""):
        """同步调用 TTS 服务（阻塞等待 future；执行器多线程保证响应能被处理）。"""
        req = TtsService.Request()
        req.text = text
        req.type = "text" if text else ""
        req.cmd = cmd
        req.audio_id = audio_id
        req.extra = ""
        future = self._cli.call_async(req)
        deadline = time.monotonic() + self._timeout
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().warn(f"TTS {cmd} 调用超时（{self._timeout:.1f}s）")
                return None
            time.sleep(0.02)
        try:
            return future.result()
        except Exception as exc:  # 服务调用异常边界
            self.get_logger().error(f"TTS {cmd} 调用异常: {exc}")
            return None

    def _stop_tts(self) -> None:
        """带 audio_id 下发 stop，真正打断本体 TTS。

        平台规则（libaudio.so 实测）：纯 stop 无 audio_id 会被忽略
        （'stop requires audio_id'），且该 audio_id 需在 audio_config.json 中
        开启 "stop": true（当前 audio_guide 满足）。
        """
        resp = self._call("stop", "", self._audio_id)
        if resp is None:
            self.get_logger().warn(f"TTS stop(audio_id={self._audio_id}) 无应答")
        elif not resp.success:
            self.get_logger().warn(
                f"TTS stop(audio_id={self._audio_id}) 未成功: status={resp.status}"
            )

    @staticmethod
    def _result(success: bool, message: str) -> Speak.Result:
        result = Speak.Result()
        result.success = success
        result.message = message
        return result


def main() -> None:
    rclpy.init()
    node = SpeakActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()