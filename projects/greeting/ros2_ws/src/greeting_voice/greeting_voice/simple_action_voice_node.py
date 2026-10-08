#!/usr/bin/env python3
"""simple_action_voice_node —— 平台「简单动作模式」(H右 + 连按B×N + 短按A) 的语音播报。

背景
----
平台 joystick_bridge_node 对「H 右拨 + 连按 B N 次 + 短按 A」的响应，是直接回放
share/joystick/param/simpleaction/*.json 的关节轨迹（1=挥手 2=击掌 3=碰拳 4=敬礼），
该链路**不经过本项目的 joy_mapper / orchestrator**，因此这条路径上默认没有任何语音。
本节点在项目侧补齐：并行监听同一份按键事件流，用与平台完全一致的规则识别出序号 N，
再调用本体 TTS 服务排队播报第 N 句文本。

识别规则（🔴 必须与平台 bridge_config.yaml 对齐，否则"按 2 次 B"会出现动作对、语音错位）
----
    · 待命：H 拨到「右」（KEY_H_RIGHT=20）；H 离开右档（LEFT/MID）撤销待命；
    · 计数：B 按下（KEY_B_DOWN=4）累加。语义完全对齐平台的 _update_press_counters：
        - count>0 且距上次有效按下 > press_max_interval_s(2.0) → 计数清零后重新从 1 计；
        - 距上次有效按下 < press_min_interval_s(0.15) → 抖动，忽略本次（不计数、不刷新基准）；
        - 否则 count += 1，并刷新基准时间。
    · 触发：A 短按 —— A 按下（KEY_A_DOWN=2）到 A 松开（KEY_A_UP=1）时长
      < short_press_s(1.0)，与 bridge_config 的 long_press_thresholds.a 一致；
      长按 A 平台判定为 long（会切 lyre 语音交互），本节点同样不触发。
    · 序号 N ∈ [1,4] 且文本非空才播报，与平台"仅回放已配置序号"一致。

⚠️ 本节点只发语音，绝不发布 /arm|/head|/waist/cmd；动作仍由平台节点负责，二者互不干扰。
🛑 急停：订阅 /greeting/panel_command，收到 stop/manual 时以 cmd=stop/audio_id=<audio_id>
   打断本节点播报（平台规则：stop 必须带 audio_id，且该实体需在 audio_config 开启 stop 权限）。
⚠️ 已知不完全对齐点：平台 FSM 规则（H 左拨 + A + 连按 B）命中时也会清零 b 计数，
   本节点不做该分支（仅 H 左拨时撤销待命）。混用导航类组合键时序号可能不同步。
"""
import time

import rclpy
from bodyctrl_msgs.msg import SbusData
from interaction_msgs.srv import TtsService
from rclpy.node import Node
from std_msgs.msg import String

# 急停命令词（与 joy_mapper / flow_command_bridge 的命令词表一致）：收到即打断本节点播报
STOP_COMMANDS = {'stop', 'manual'}

# 按键事件常量：优先取消息定义中的常量，缺失时回退到 SDK 文档给的数值
KEY_A_DOWN = getattr(SbusData, 'KEY_A_DOWN', 2)
KEY_A_UP = getattr(SbusData, 'KEY_A_UP', 1)
KEY_B_DOWN = getattr(SbusData, 'KEY_B_DOWN', 4)
KEY_H_LEFT = getattr(SbusData, 'KEY_H_LEFT', 18)
KEY_H_MID = getattr(SbusData, 'KEY_H_MID', 19)
KEY_H_RIGHT = getattr(SbusData, 'KEY_H_RIGHT', 20)

# 与平台 SIMPLE_ACTION_MAX 一致：1=挥手 2=击掌 3=碰拳 4=敬礼
MAX_ACTION_INDEX = 4


class SimpleActionVoiceNode(Node):
    """H 右拨待命 → 连按 B 选序号 → 短按 A 触发 → TTS 播报对应文本。"""

    def __init__(self):
        super().__init__('simple_action_voice_node')

        self.declare_parameter('sbus_topic', '/sbus_data/event')
        self.declare_parameter('tts_service', '/intelligent_interaction/tts/play')
        #: 播报所用 TTS 实体（audio_id）：必须是 audio_config 中带 "stop": true 的文本型实体，
        #: 否则急停时无法打断本节点的音频（纯 stop 无 audio_id 会被平台忽略）
        self.declare_parameter('audio_id', 'audio_guide')
        #: 急停命令来源：joy_mapper 发布的 /greeting/panel_command（std_msgs/String）
        self.declare_parameter('command_topic', '/greeting/panel_command')
        # 连按/短按判定窗口，默认值即平台 bridge_config.yaml 的取值，改动前请同步确认两侧
        self.declare_parameter('press_min_interval_s', 0.15)
        self.declare_parameter('press_max_interval_s', 2.0)
        self.declare_parameter('short_press_s', 1.0)
        self.declare_parameter('service_timeout_s', 5.0)
        self.declare_parameter('dry_run', False)
        for idx in range(1, MAX_ACTION_INDEX + 1):
            self.declare_parameter(f'text_{idx}', '')

        self._sbus_topic = self.get_parameter('sbus_topic').value
        self._tts_service = self.get_parameter('tts_service').value
        self._audio_id = self.get_parameter('audio_id').value
        self._command_topic = self.get_parameter('command_topic').value
        self._press_min = float(self.get_parameter('press_min_interval_s').value)
        self._press_max = float(self.get_parameter('press_max_interval_s').value)
        self._short_press = float(self.get_parameter('short_press_s').value)
        self._timeout_s = float(self.get_parameter('service_timeout_s').value)
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._texts = {
            idx: self.get_parameter(f'text_{idx}').value
            for idx in range(1, MAX_ACTION_INDEX + 1)
            if self.get_parameter(f'text_{idx}').value
        }

        self._armed = False          # H 是否停在「右」档
        self._b_count = 0            # B 有效按下次数
        self._b_last_accept = None   # 上次"被计数"的按下时刻（秒）
        self._a_press_time = None    # 本次 A 按下时刻（秒）
        self._busy = False           # 是否有 TTS 请求在途
        self._busy_start = None
        self._pending_text = None

        self._sub = self.create_subscription(
            SbusData, self._sbus_topic, self._on_sbus, 10)
        # 急停：监听 /greeting/panel_command，收到 stop/manual 立即打断本节点播报
        self.create_subscription(
            String, self._command_topic, self._on_command, 10)
        self._cli = self.create_client(TtsService, self._tts_service)
        self._timer = self.create_timer(1.0, self._check_timeout)

        enabled = '/'.join(
            f'{idx}:{text}' for idx, text in sorted(self._texts.items())) or '无（全部留空）'
        self.get_logger().info(
            f'simple_action_voice_node 启动: sbus={self._sbus_topic}, tts={self._tts_service}, '
            f'audio_id={self._audio_id}, 急停命令来自 {self._command_topic}, '
            f'连按窗口=[{self._press_min},{self._press_max}]s, A短按<{self._short_press}s, '
            f'dry_run={self._dry_run}, 已启用文本={{{enabled}}}')

    # ---------- 按键事件 ----------

    def _on_sbus(self, msg):
        ev = msg.key_event_new
        now = time.monotonic()

        if ev == KEY_H_RIGHT:
            if not self._armed:
                self._armed = True
                self.get_logger().info(
                    f'H 已右拨，待命中：连按 B 选序号(1~{MAX_ACTION_INDEX})，短按 A 触发播报')
            return
        if ev in (KEY_H_MID, KEY_H_LEFT):
            if self._armed:
                self._armed = False
                self.get_logger().info('H 离开右档，撤销待命')
            return

        if not self._armed:
            return

        if ev == KEY_B_DOWN:
            self._on_b_press(now)
        elif ev == KEY_A_DOWN:
            self._a_press_time = now
        elif ev == KEY_A_UP:
            self._on_a_release(now)

    def _on_b_press(self, now):
        """与平台 _update_press_counters 完全一致的计数语义。"""
        dt = now - self._b_last_accept if self._b_last_accept is not None else float('inf')
        if self._b_count > 0 and self._press_max > 0.0 and dt > self._press_max:
            self._b_count = 0
        if dt >= self._press_min:
            self._b_count += 1
            self._b_last_accept = now
            self.get_logger().info(f'B 有效连按计数 = {self._b_count}')
        else:
            self.get_logger().debug(f'B 连按间隔 {dt:.3f}s < {self._press_min}s，抖动忽略')

    def _on_a_release(self, now):
        t_down = self._a_press_time
        self._a_press_time = None
        if t_down is None:
            return
        if now - t_down >= self._short_press:
            self.get_logger().info(
                f'A 长按 {now - t_down:.2f}s，平台判定为 long，不触发简单动作，本节点同样跳过')
            return

        count = self._b_count
        if not (1 <= count <= MAX_ACTION_INDEX):
            self.get_logger().info(
                f'A 短按但 B 计数={count} 不在 1~{MAX_ACTION_INDEX}，平台不会回放动作，不播报')
            return
        self._b_count = 0  # 与平台一致：仅在序号有效并触发时清零

        text = self._texts.get(count, '')
        if not text:
            self.get_logger().info(f'序号 {count} 未配置文本，只做动作不播报')
            return
        self._trigger_play(count, text)

    # ---------- 急停 ----------

    def _on_command(self, msg):
        """收到急停命令（stop/manual）：打断本节点正在播报的音频。"""
        cmd = msg.data.strip().lower()
        if cmd not in STOP_COMMANDS:
            return
        if self._dry_run:
            self.get_logger().info(f'[dry_run] 收到急停命令 {cmd!r}，将打断播报')
            return
        if not self._cli.service_is_ready():
            self.get_logger().warn(f'收到急停命令 {cmd!r}，但 TTS 服务未就绪，无法打断')
            return
        req = TtsService.Request()
        req.text = ''
        req.type = ''
        req.cmd = 'stop'
        req.audio_id = self._audio_id  # 平台规则：stop 必须带 audio_id 才生效
        self._cli.call_async(req)
        self._reset_busy()
        self.get_logger().warn(f'收到急停命令 {cmd!r}，已下发 TTS stop(audio_id={self._audio_id})')

    # ---------- TTS 调用 ----------

    def _trigger_play(self, index, text):
        label = f'H右+B×{index}+A'
        if self._busy:
            self.get_logger().info(f'上一次播报请求仍在处理，忽略本次 {label} 触发')
            return

        if self._dry_run:
            self.get_logger().info(f'[dry_run] {label} 将播报: {text}')
            return

        if not self._cli.service_is_ready():
            self.get_logger().warn(
                f'TTS 服务 {self._tts_service} 尚未就绪，忽略本次 {label} 触发')
            return

        self._busy = True
        self._busy_start = self.get_clock().now()
        self._pending_text = (label, text)
        req = TtsService.Request()
        req.text = ''
        req.type = ''
        req.cmd = 'query'
        self._cli.call_async(req).add_done_callback(self._on_query_done)

    def _on_query_done(self, future):
        try:
            resp = future.result()
        except Exception as exc:  # 服务调用异常边界
            self._reset_busy()
            self.get_logger().error(f'TTS query 失败: {exc}')
            return

        label = self._pending_text[0] if self._pending_text else '?'
        if resp.success and resp.status == 'playing':
            self._reset_busy()
            self.get_logger().info(f'当前正在播报，忽略本次 {label} 触发（append 即排队，避免叠加）')
            return

        _, text = self._pending_text
        req = TtsService.Request()
        req.text = text
        req.type = 'text'
        req.cmd = 'append'
        req.audio_id = self._audio_id  # 与急停 stop 用同一实体，保证停得掉
        self._cli.call_async(req).add_done_callback(self._on_append_done)

    def _on_append_done(self, future):
        label, text = self._pending_text or ('?', '')
        self._reset_busy()
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f'TTS append 失败: {exc}')
            return
        if resp.success:
            self.get_logger().info(f'{label} 语音已排队播报: {text}')
        else:
            self.get_logger().warn(f'TTS append 未成功: status={resp.status}')

    # ---------- 超时保护 ----------

    def _check_timeout(self):
        if not self._busy or self._busy_start is None:
            return
        elapsed = (self.get_clock().now() - self._busy_start).nanoseconds / 1e9
        if elapsed > self._timeout_s:
            self._reset_busy()
            self.get_logger().warn(
                f'TTS 服务 {elapsed:.1f}s 无应答，已复位（检查 adaptive_tts 是否在运行）')

    def _reset_busy(self):
        self._busy = False
        self._busy_start = None
        self._pending_text = None


def main(args=None):
    rclpy.init(args=args)
    node = SimpleActionVoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
