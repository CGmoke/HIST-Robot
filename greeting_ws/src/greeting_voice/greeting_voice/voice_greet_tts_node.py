"""voice_greet_tts_node — 遥控器 G 左/右拨 + A/B/C/D 键触发 TTS 播报。

流程（接口依据：天轶 2.5 ROS2 SDK 二次开发文档 §3.4 / §7.2.5）：
  1. 订阅 /sbus_data/event（bodyctrl_msgs/msg/SbusData）
     - key_event_new == KEY_G_LEFT(15)  → G 左拨待命
     - key_event_new == KEY_G_RIGHT(17) → G 右拨待命
     - key_event_new == KEY_G_MID(16)   → 撤销待命
     - 待命中按 A(2)/B(4)/C(6)/D(8)_DOWN → 播报对应文本
       G 左拨 → 预留1~4；G 右拨 → 预留5~8
  2. 触发后调用 /intelligent_interaction/tts/play（interaction_msgs/srv/TtsService）
     - 先 cmd=query 查询播报状态，playing 时忽略本次按键（防重复排队）
     - 否则 cmd=append / type=text / audio_id=<audio_id> 排队播报
  3. 订阅 /greeting/panel_command：收到 stop/manual 急停命令时，
     以 cmd=stop / audio_id=<audio_id> 打断本节点播报（平台要求 stop 必须带 audio_id）
  4. 服务无应答超时自动复位，避免节点卡死在 busy 状态
"""

import rclpy
from rclpy.node import Node

from bodyctrl_msgs.msg import SbusData
from interaction_msgs.srv import TtsService
from std_msgs.msg import String

# 急停命令词（与 joy_mapper / flow_command_bridge 的命令词表一致）：收到即打断本节点播报
STOP_COMMANDS = {'stop', 'manual'}

# G 三档开关事件常量：优先取消息定义中的常量，缺失时回退到 SDK 文档给出的数值
KEY_G_LEFT = getattr(SbusData, 'KEY_G_LEFT', 15)
KEY_G_MID = getattr(SbusData, 'KEY_G_MID', 16)
KEY_G_RIGHT = getattr(SbusData, 'KEY_G_RIGHT', 17)

# 触发键表：(G 档位, 按键名, DOWN 事件常量名, 回退事件值, 默认播报文本)
# 八个组合地位完全相同，新增/删除组合只改这张表。
TRIGGER_KEYS = [
    ('left', 'A', 'KEY_A_DOWN', 2, '预留1'),
    ('left', 'B', 'KEY_B_DOWN', 4, '预留2'),
    ('left', 'C', 'KEY_C_DOWN', 6, '预留3'),
    ('left', 'D', 'KEY_D_DOWN', 8, '预留4'),
    ('right', 'A', 'KEY_A_DOWN', 2, '预留5'),
    ('right', 'B', 'KEY_B_DOWN', 4, '预留6'),
    ('right', 'C', 'KEY_C_DOWN', 6, '预留7'),
    ('right', 'D', 'KEY_D_DOWN', 8, '预留8'),
]


class VoiceGreetTtsNode(Node):
    """G 左/右拨待命 → A/B/C/D 按下 → 查询 TTS 状态 → 排队播报对应文本。"""

    def __init__(self):
        super().__init__('voice_greet_tts_node')

        self.declare_parameter('sbus_topic', '/sbus_data/event')
        self.declare_parameter('tts_service', '/intelligent_interaction/tts/play')
        #: 播报所用 TTS 实体（audio_id）：必须是 audio_config 中带 "stop": true 的文本型实体，
        #: 否则急停时无法打断本节点的音频（纯 stop 无 audio_id 会被平台忽略）
        self.declare_parameter('audio_id', 'audio_guide')
        #: 急停命令来源：joy_mapper 发布的 /greeting/panel_command（std_msgs/String）
        self.declare_parameter('command_topic', '/greeting/panel_command')
        for side, name, _, _, default_text in TRIGGER_KEYS:
            self.declare_parameter(f'text_{side}_{name.lower()}', default_text)
        self.declare_parameter('service_timeout_s', 5.0)
        self.declare_parameter('dry_run', False)

        self._sbus_topic = self.get_parameter('sbus_topic').value
        self._tts_service = self.get_parameter('tts_service').value
        self._audio_id = self.get_parameter('audio_id').value
        self._command_topic = self.get_parameter('command_topic').value
        self._timeout_s = float(self.get_parameter('service_timeout_s').value)
        self._dry_run = bool(self.get_parameter('dry_run').value)

        # 按键事件值 → {G 档位: (按键名, 播报文本)}；文本为空的组合不启用
        self._key_texts = {}
        for side, name, const_name, fallback, _ in TRIGGER_KEYS:
            ev = getattr(SbusData, const_name, fallback)
            text = self.get_parameter(f'text_{side}_{name.lower()}').value
            if text:
                self._key_texts.setdefault(ev, {})[side] = (name, text)

        self._g_side = None   # None=未待命；'left'/'right'=G 已拨到该档
        self._busy = False    # 是否有 TTS 请求在途
        self._busy_start = None
        self._pending_text = None  # 本次触发待播报的 (标签, 文本)

        self._sub = self.create_subscription(
            SbusData, self._sbus_topic, self._on_sbus, 10)
        # 急停：监听 /greeting/panel_command，收到 stop/manual 立即打断本节点播报
        self.create_subscription(
            String, self._command_topic, self._on_command, 10)
        self._cli = self.create_client(TtsService, self._tts_service)
        self._timer = self.create_timer(1.0, self._check_timeout)

        combos = '/'.join(
            f'G{side}+{name}'
            for by_side in self._key_texts.values()
            for side, (name, _) in sorted(by_side.items()))
        self.get_logger().info(
            f'voice_greet_tts_node 启动: sbus={self._sbus_topic}, '
            f'tts={self._tts_service}, audio_id={self._audio_id}, '
            f'急停命令来自 {self._command_topic}, 已启用组合={{{combos}}}, dry_run={self._dry_run}')

    # ---------- 按键事件处理 ----------

    def _on_sbus(self, msg):
        ev = msg.key_event_new
        if ev == KEY_G_LEFT:
            if self._g_side != 'left':
                self._g_side = 'left'
                self.get_logger().info('G 已左拨，待命中：按 A/B/C/D 播放预留1~4')
        elif ev == KEY_G_RIGHT:
            if self._g_side != 'right':
                self._g_side = 'right'
                self.get_logger().info('G 已右拨，待命中：按 A/B/C/D 播放预留5~8')
        elif ev == KEY_G_MID:
            if self._g_side is not None:
                self._g_side = None
                self.get_logger().info('G 已回中，撤销待命')
        else:
            by_side = self._key_texts.get(ev)
            if by_side is None:
                return
            if self._g_side and self._g_side in by_side:
                name, text = by_side[self._g_side]
                self._trigger_play(f'G{self._g_side}+{name}', text)
            elif self._g_side:
                self.get_logger().debug(
                    f'该组合未启用（G{self._g_side}+按键），忽略')
            else:
                self.get_logger().debug('按键触发但 G 未拨动，忽略')

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

    def _trigger_play(self, key_label, text):
        if self._busy:
            self.get_logger().info(
                f'上一次播报请求仍在处理，忽略本次 {key_label} 触发')
            return

        if self._dry_run:
            self.get_logger().info(f'[dry_run] {key_label} 将播报: {text}')
            return

        if not self._cli.service_is_ready():
            self.get_logger().warn(
                f'TTS 服务 {self._tts_service} 尚未就绪，忽略本次 {key_label} 触发')
            return

        self._busy = True
        self._busy_start = self.get_clock().now()
        self._pending_text = (key_label, text)
        req = TtsService.Request()
        req.text = ''
        req.type = ''
        req.cmd = 'query'
        future = self._cli.call_async(req)
        future.add_done_callback(self._on_query_done)

    def _on_query_done(self, future):
        try:
            resp = future.result()
        except Exception as exc:  # 服务调用异常边界
            self._reset_busy()
            self.get_logger().error(f'TTS query 失败: {exc}')
            return

        key_label = self._pending_text[0] if self._pending_text else '?'
        if resp.success and resp.status == 'playing':
            self._reset_busy()
            self.get_logger().info(
                f'当前正在播报，忽略本次 {key_label} 触发（防重复排队）')
            return

        self._append_text()

    def _append_text(self):
        _, text = self._pending_text
        req = TtsService.Request()
        req.text = text
        req.type = 'text'
        req.cmd = 'append'
        req.audio_id = self._audio_id  # 与急停 stop 用同一实体，保证停得掉
        future = self._cli.call_async(req)
        future.add_done_callback(self._on_append_done)

    def _on_append_done(self, future):
        key_label, text = self._pending_text or ('?', '')
        self._reset_busy()
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f'TTS append 失败: {exc}')
            return

        if resp.success:
            self.get_logger().info(f'{key_label} 语音已排队播报: {text}')
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
    node = VoiceGreetTtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
