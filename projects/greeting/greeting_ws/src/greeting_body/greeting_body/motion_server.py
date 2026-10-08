#!/usr/bin/env python3
"""motion_server.py —— 实机 PlayMotion 服务端（greeting_body / 成员A）

对外提供 /greeting/play_motion (greeting_interfaces/action/PlayMotion)，
把 config/motions.yaml 里的礼仪动作（与 tianyi25_sim 同一套「打招呼」动作）
在【天轶 2.5 实机】上播放：

    动作名 -> (config/joint_map.yaml) -> 电机 ID -> /head/cmd + /arm/cmd + /waist/cmd

与仿真桩 greeting_sim_stubs/play_motion_server.py 的差异：
    · 执行侧从「一条 JointTrajectory 交给 Gazebo 控制器」换成
      「≥10 Hz 连续流式下发实机关节帧」（实机命令非一次性生效，必须持续保持）；
    · 起点取 /robot_state 实测角，先 warmup 保持当前位姿软化 SDK 模式移交，再开始动作；
    · 每个关节按 URDF <limit> 收缩软限位、单帧增量限幅，腿部永不下发。

参数（见 config/motion_params.yaml 或 launch 内直接给定）：
    motions_file / joint_map_file / publish_rate_hz / speed_limit /
    soft_limit_margin_rad / max_step_rad / warmup_s / warmup_speed /
    warmup_current_start / warmup_kp_start / hold_s /
    arm_speed / arm_current / head_speed / head_current / waist_speed / waist_current /
    arm_control_mode / max_follow_err_rad /
    arm_kp_scale / arm_kd_scale / arm_feedforward_scale /
    required_groups / robot_state_topic /
    idle_scan_enabled / idle_scan_motion / idle_scan_delay_s / idle_scan_speed_scale /
    idle_scan_entry_s / idle_scan_pause_on_h_right / idle_scan_sbus_topic /
    idle_scan_pause_on_voice / voice_long_press_s

待机扫视：无动作 goal 且静止 idle_scan_delay_s 后，后台线程自动循环播放
idle_scan_loop（只动头/腰）；新 goal 一到立即让位。H 停在右档（平台「简单动作
模式」直发关节、绕过本服务端）时暂停扫视，避免两条链路互抢关节。
语音门控：F 上拨 + 长按 A 每次触发即切换「语音功能」开关（平台侧无状态话题，
故本项目侧自行识别该按键组合，长按阈值与 platform bridge_config 的
long_press_thresholds.a 对齐）；语音开启期间扫视停止，头/腰平滑回 home 中立位
并持续保持，关闭后立即恢复扫视。设 idle_scan_enabled:=false 可整体关闭。
"""
from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Dict, List, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from greeting_interfaces.action import PlayMotion
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .body_adapter import BodyAdapter, FEEDFORWARD_SPEED_MARGIN, MIN_FEEDFORWARD_SPEED

try:  # 仅用于「H 右档暂停待机扫视」门控；缺失时自动降级（不影响动作播放）
    from bodyctrl_msgs.msg import SbusData
except ImportError:  # pragma: no cover
    SbusData = None

#: 礼仪动作的硬性速度上限（接口契约：统一 ≤0.7，避免机械感与幅度过大）
HARD_SPEED_LIMIT = 0.7
#: feedback 最小间隔(s)
FEEDBACK_PERIOD_S = 0.1
#: 受控分组（腿部永不在内）
ALL_GROUPS = ("head", "arm", "waist")
#: warmup 首帧手臂力位混合 Kp 比例（相对满增益），线性 ramp 到 1.0
WARMUP_KP_START = 0.6
#: 待机扫视轮询周期(s)：决定"新动作到来后多久让出"与"静止多久后开扫"的粒度
IDLE_POLL_S = 0.2
#: 待机扫视期间只扫动的关节前缀（其余关节全程保持实测起点不动）
IDLE_SCAN_PREFIXES = ("head_", "waist_")
#: 按键事件常量：优先取消息定义中的常量，缺失时回退到 SDK 文档给的数值
if SbusData is not None:
    KEY_A_UP = getattr(SbusData, "KEY_A_UP", 1)
    KEY_A_DOWN = getattr(SbusData, "KEY_A_DOWN", 2)
    KEY_H_LEFT = getattr(SbusData, "KEY_H_LEFT", 18)
    KEY_H_MID = getattr(SbusData, "KEY_H_MID", 19)
    KEY_H_RIGHT = getattr(SbusData, "KEY_H_RIGHT", 20)
    KEY_F_UP = getattr(SbusData, "KEY_F_UP", 12)
    KEY_F_MID = getattr(SbusData, "KEY_F_MID", 13)
    KEY_F_DOWN = getattr(SbusData, "KEY_F_DOWN", 14)
else:  # pragma: no cover
    KEY_A_UP, KEY_A_DOWN = 1, 2
    KEY_H_LEFT, KEY_H_MID, KEY_H_RIGHT = 18, 19, 20
    KEY_F_UP, KEY_F_MID, KEY_F_DOWN = 12, 13, 14


class MotionLibrary:
    """读取并校验 motions.yaml，负责关键帧展开。"""

    def __init__(self, motions_file: str, log) -> None:
        data = yaml.safe_load(Path(motions_file).read_text(encoding="utf-8"))
        self._home = {k: float(v) for k, v in (data.get("home") or {}).items()}
        self._motions = data.get("motions") or {}
        if not self._home:
            raise ValueError(f"{motions_file}: 缺少 home 中立姿态定义")
        if not self._motions:
            raise ValueError(f"{motions_file}: 未定义任何动作")
        self._validate(motions_file, log)

    def _validate(self, path: str, log) -> None:
        known = set(self._home)
        for name, motion in self._motions.items():
            for kf in motion.get("keyframes", []):
                for joint in (kf.get("positions") or {}):
                    if joint not in known:
                        log.error(
                            f"{path}: 动作 {name!r} 引用了 home 中不存在的关节 {joint!r}，"
                            "该角度将被忽略"
                        )

    @property
    def home(self) -> Dict[str, float]:
        return dict(self._home)

    @property
    def names(self) -> List[str]:
        return list(self._motions)

    def __contains__(self, name: str) -> bool:
        return name in self._motions

    def duration(self, name: str) -> float:
        return float(self._motions[name].get("duration", 2.0))

    def frames(self, name: str) -> List[Tuple[float, Dict[str, float]]]:
        """展开 keyframes -> [(t, pose)]（按 t 升序）。

        语义与仿真桩一致：写了 positions 的帧与 home 合并（未写的关节取 home）；
        不写 positions 的帧保持上一帧姿态（用于「停留」）。
        """
        frames: List[Tuple[float, Dict[str, float]]] = []
        prev = dict(self._home)
        for kf in self._motions[name].get("keyframes", []):
            if "positions" in kf:
                pose = dict(self._home)
                pose.update(
                    {k: float(v) for k, v in (kf["positions"] or {}).items() if k in self._home}
                )
                prev = pose
            else:
                pose = prev
            frames.append((float(kf["t"]), pose))
        frames.sort(key=lambda item: item[0])
        return frames


class MotionServer(Node):
    """实机 PlayMotion Action 服务端。"""

    def __init__(self) -> None:
        super().__init__("greeting_body_motion_server")
        share = Path(get_package_share_directory("greeting_body"))

        self.declare_parameter("motions_file", "")
        self.declare_parameter("joint_map_file", "")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("speed_limit", HARD_SPEED_LIMIT)
        self.declare_parameter("soft_limit_margin_rad", 0.05)
        self.declare_parameter("max_step_rad", 0.2)
        self.declare_parameter("warmup_s", 1.5)
        self.declare_parameter("warmup_speed", 0.3)
        self.declare_parameter("warmup_current_start", 1.0)
        self.declare_parameter("warmup_kp_start", WARMUP_KP_START)
        self.declare_parameter("hold_s", 0.5)
        self.declare_parameter("arm_speed", 1.0)
        self.declare_parameter("arm_current", 5.0)
        self.declare_parameter("head_speed", 0.5)
        self.declare_parameter("head_current", 5.0)
        self.declare_parameter("waist_speed", 0.5)
        self.declare_parameter("waist_current", 5.0)
        #: 手臂控制模式：0=位置模式(天轶2.5 官方做法，默认) 1=力位混合(仅回退实验用，
        #: 本机型实测会持续高频啸叫)
        self.declare_parameter("arm_control_mode", 0)
        #: 臂关节期望相对实测的最大超前量(rad)，防止 Kp 顶死限位粘滑颤振
        self.declare_parameter("max_follow_err_rad", 0.10)
        #: 力位混合增益/前馈缩放：增益基准取自天工3.0(dex)，本机天轶2.5 电机不同，
        #: 运动中若持续高频啸叫（电流环极限环），优先下调 arm_kp_scale。
        self.declare_parameter("arm_kp_scale", 1.0)
        self.declare_parameter("arm_kd_scale", 1.0)
        self.declare_parameter("arm_feedforward_scale", 1.0)
        #: 位置模式运动段 spd = |轨迹瞬时速度| × margin（只略 >1，现场调参主旋钮）
        self.declare_parameter("feedforward_speed_margin", FEEDFORWARD_SPEED_MARGIN)
        #: 位置模式 spd 绝对下限(rad/s)，必须远小于任何运动段真实速度
        self.declare_parameter("min_feedforward_speed", MIN_FEEDFORWARD_SPEED)
        self.declare_parameter("required_groups", ["arm"])
        self.declare_parameter("robot_state_topic", "/robot_state")
        # ---- 待机扫视（无动作时自动循环 idle_scan_loop；新动作一到立即抢占）----
        self.declare_parameter("idle_scan_enabled", True)
        self.declare_parameter("idle_scan_motion", "idle_scan_loop")
        #: 最后一个动作结束后静止多久才开扫（给编排层连续段之间留缓冲）
        self.declare_parameter("idle_scan_delay_s", 3.0)
        #: 扫视速度（同样受 HARD_SPEED_LIMIT 约束）
        self.declare_parameter("idle_scan_speed_scale", 0.5)
        #: 从实测姿态软过渡到扫视起点的时间(s)，避免开机/外来姿态被瞬间拽回
        self.declare_parameter("idle_scan_entry_s", 1.5)
        #: H 停在右档（平台简单动作待命/回放，直发关节绕过本服务端）时暂停扫视
        self.declare_parameter("idle_scan_pause_on_h_right", True)
        self.declare_parameter("idle_scan_sbus_topic", "/sbus_data/event")
        #: F 上拨 + 长按 A = 平台语音功能开/关；开启期间扫视让位并回 home 中立位
        self.declare_parameter("idle_scan_pause_on_voice", True)
        #: A 长按判定阈值(s)，须与平台 bridge_config 的 long_press_thresholds.a 对齐
        self.declare_parameter("voice_long_press_s", 1.0)

        p = self.get_parameter
        motions_file = str(p("motions_file").value) or str(share / "config" / "motions.yaml")
        joint_map_file = str(p("joint_map_file").value) or str(share / "config" / "joint_map.yaml")

        self._library = MotionLibrary(motions_file, self.get_logger())
        self._speed_limit = min(float(p("speed_limit").value), HARD_SPEED_LIMIT)
        self._warmup_s = float(p("warmup_s").value)
        self._warmup_speed = float(p("warmup_speed").value)
        self._warmup_current_start = float(p("warmup_current_start").value)
        self._warmup_kp_start = float(p("warmup_kp_start").value)
        self._hold_s = float(p("hold_s").value)
        self._required_groups = [str(g) for g in p("required_groups").value]
        self._idle_enabled = bool(p("idle_scan_enabled").value)
        self._idle_motion = str(p("idle_scan_motion").value)
        self._idle_delay_s = max(float(p("idle_scan_delay_s").value), 0.0)
        self._idle_speed = min(
            max(float(p("idle_scan_speed_scale").value), 0.05), self._speed_limit
        )
        self._idle_entry_s = max(float(p("idle_scan_entry_s").value), 0.0)
        self._idle_pause_on_h = bool(p("idle_scan_pause_on_h_right").value)
        self._idle_sbus_topic = str(p("idle_scan_sbus_topic").value)
        self._idle_pause_on_voice = bool(p("idle_scan_pause_on_voice").value)
        self._voice_long_press_s = max(float(p("voice_long_press_s").value), 0.1)

        self._adapter = BodyAdapter(
            self,
            joint_map_file,
            soft_limit_margin_rad=float(p("soft_limit_margin_rad").value),
            max_step_rad=float(p("max_step_rad").value),
            publish_rate_hz=float(p("publish_rate_hz").value),
            arm_speed=float(p("arm_speed").value),
            arm_current=float(p("arm_current").value),
            head_speed=float(p("head_speed").value),
            head_current=float(p("head_current").value),
            waist_speed=float(p("waist_speed").value),
            waist_current=float(p("waist_current").value),
            arm_control_mode=int(p("arm_control_mode").value),
            max_follow_err_rad=float(p("max_follow_err_rad").value),
            arm_kp_scale=float(p("arm_kp_scale").value),
            arm_kd_scale=float(p("arm_kd_scale").value),
            arm_feedforward_scale=float(p("arm_feedforward_scale").value),
            robot_state_topic=str(p("robot_state_topic").value),
            feedforward_speed_margin=float(p("feedforward_speed_margin").value),
            min_feedforward_speed=float(p("min_feedforward_speed").value),
        )

        self._cb_group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self,
            PlayMotion,
            "/greeting/play_motion",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._cb_group,
        )

        # ---- 待机扫视状态 ----
        self._active_lock = threading.Lock()
        self._active_goals = 0             # 在执行的动作数（>0 表示正在动，扫视须让位）
        self._idle_lock = threading.Lock()  # 串行化对 BodyAdapter 的下发（动作 vs 扫视）
        self._idle_stop = threading.Event()
        self._idle_shutdown = threading.Event()
        self._h_armed = False              # H 是否停在右档（平台简单动作待命）
        self._voice_active = False         # 语音功能是否开启（F上+长按A 切换）
        self._f_up = False                 # F 拨杆是否停在上档（语音组合键前提）
        self._a_down_t = None              # A 按下时刻（用于长按判定）
        self._last_motion_end = time.monotonic()
        self._idle_thread = None

        self.get_logger().info(
            "实机 PlayMotion 服务端就绪: /greeting/play_motion\n"
            f"    动作表 {motions_file}（{len(self._library.names)} 个动作: {self._library.names}）\n"
            f"    关节表 {joint_map_file}；下发频率 {self._adapter.publish_rate_hz:.0f} Hz；"
            f"速度上限 {self._speed_limit}；手臂控制模式 {self._adapter.arm_control_mode}\n"
            f"    臂增益缩放 Kp×{self._adapter.arm_gain_scales[0]:.2f} "
            f"Kd×{self._adapter.arm_gain_scales[1]:.2f} "
            f"前馈×{self._adapter.arm_gain_scales[2]:.2f}\n"
            f"    硬预检通道 {self._required_groups}；腿部电机永不下发"
        )

        # ---- 按键门控：H 右档 / F上+长按A 语音开关 均订阅 /sbus_data/event ----
        if self._idle_enabled and (self._idle_pause_on_h or self._idle_pause_on_voice):
            if SbusData is None:
                self.get_logger().warn(
                    "未找到 bodyctrl_msgs.msg.SbusData，H 右档/语音暂停扫视门控已禁用"
                )
            else:
                self.create_subscription(
                    SbusData, self._idle_sbus_topic, self._on_sbus, 10
                )

        # ---- 启动待机扫视线程 ----
        if self._idle_enabled and self._idle_motion not in self._library:
            self.get_logger().warn(
                f"待机扫视动作 {self._idle_motion!r} 不在动作表 {self._library.names}，已禁用"
            )
            self._idle_enabled = False
        if self._idle_enabled:
            self._idle_thread = threading.Thread(
                target=self._idle_worker, name="idle_scan", daemon=True
            )
            self._idle_thread.start()
            self.get_logger().info(
                f"待机扫视已启用：动作 {self._idle_motion!r}，静止 {self._idle_delay_s:.1f}s "
                f"后自动循环，speed_scale={self._idle_speed:.2f}，"
                f"H 右档暂停={self._idle_pause_on_h}，"
                f"语音( F上+长按A≥{self._voice_long_press_s:.1f}s )暂停={self._idle_pause_on_voice}"
            )

    # ---------------------------------------------------------------- Action 回调
    def _on_goal(self, goal_request) -> GoalResponse:
        if goal_request.motion_name not in self._library:
            self.get_logger().warn(
                f"拒绝目标：未知动作 {goal_request.motion_name!r}，可用 {self._library.names}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> PlayMotion.Result:
        name = goal_handle.request.motion_name
        # 抢占待机扫视：先置位让扫视循环尽快让出，再取锁串行化对 BodyAdapter 的写入，
        # 保证「动作」与「扫视」任一时刻只有一个在下发关节帧
        self._begin_active()
        try:
            with self._idle_lock:
                return self._run(goal_handle, name)
        except Exception as exc:  # noqa: BLE001 — 兜底，避免异常让 action 悬空
            self.get_logger().error(f"动作 {name!r} 执行异常：{exc}")
            goal_handle.abort()
            result = PlayMotion.Result()
            result.success = False
            result.message = f"动作 {name} 执行异常：{exc}"
            return result
        finally:
            self._end_active()

    # ---------------------------------------------------------------- 占用/扫视互斥
    def _begin_active(self) -> None:
        with self._active_lock:
            self._active_goals += 1
        self._idle_stop.set()

    def _end_active(self) -> None:
        with self._active_lock:
            self._active_goals = max(self._active_goals - 1, 0)
            all_done = self._active_goals == 0
        self._last_motion_end = time.monotonic()
        if all_done:
            self._idle_stop.clear()

    def _is_active(self) -> bool:
        with self._active_lock:
            return self._active_goals > 0

    def _run(self, goal_handle, name: str) -> PlayMotion.Result:
        req = goal_handle.request
        # 服务端是最后一道闸：礼仪动作速度统一钳到 ≤0.7
        speed = min(max(float(req.speed_scale), 0.05), self._speed_limit)

        frames = self._library.frames(name)
        nominal = self._library.duration(name)
        if frames and frames[-1][0] > nominal:
            self.get_logger().warn(
                f"动作 {name!r} 末帧 {frames[-1][0]:.1f}s 超过名义时长 {nominal:.1f}s，以末帧为准"
            )
            nominal = frames[-1][0]

        # 硬预检 1：必需通道无订阅者时直接失败，避免动作做到一半
        ok, detail = self._adapter.check_channels(self._required_groups)
        if not ok:
            self.get_logger().error(f"动作 {name!r} 前置检查失败：{detail}")
            goal_handle.abort()
            result = PlayMotion.Result()
            result.success = False
            result.message = f"控制通道未就绪：{detail}"
            return result

        # 硬预检 2：电机健康（有反馈、无故障码、角度合理）。
        # 通道有订阅者 ≠ 电机在线：通讯掉线时 pos/cur 恒为 0.0 假值，
        # 既执行不了动作，又会污染起点基线，必须在动作开始前拦下。
        ok_h, detail_h = self._adapter.motor_health(self._required_groups)
        if not ok_h:
            self.get_logger().error(f"动作 {name!r} 前置检查失败：{detail_h}")
            goal_handle.abort()
            result = PlayMotion.Result()
            result.success = False
            result.message = f"电机未就绪：{detail_h}"
            return result

        # 非必需分组仅告警（例如 /head/cmd 未在线的降级运行）
        for group in ALL_GROUPS:
            if group in self._required_groups:
                continue
            ok_g, detail_g = self._adapter.check_channels([group])
            if not ok_g:
                self.get_logger().warn(f"动作 {name!r}：{detail_g}（该部位本次不动）")

        home = self._library.home
        start = self._adapter.start_pose(home)
        healthy = self._adapter.healthy_measured()
        self.get_logger().info(
            f"动作 {name!r} 起点：{len(healthy)}/{len(start)} 个关节取 /robot_state 健康实测角，其余回退 home"
        )

        # 起点 warmup：保持当前位姿、低电流起步，软化整机->SDK 的模式移交瞬态
        self._warmup(start)

        # 时间轴：实测起点(0) + 跳过 t≈0 的占位帧（语义同仿真桩）
        timeline: List[Tuple[float, Dict[str, float]]] = [(0.0, start)]
        timeline += [(t, pose) for t, pose in frames if t > 1e-6]
        total = nominal / speed
        self.get_logger().info(
            f"播放动作 {name!r}：名义 {nominal:.1f}s，speed_scale={speed:.2f} -> 实际 {total:.1f}s"
        )

        period = 1.0 / max(self._adapter.publish_rate_hz, 1.0)
        t0 = time.monotonic()
        last_fb = 0.0
        while True:
            elapsed = time.monotonic() - t0
            if goal_handle.is_cancel_requested:
                self.get_logger().warn(f"动作 {name!r} 收到取消，平滑回到中立位")
                self._adapter.hold(home, self._hold_s, speed_scale=speed)
                goal_handle.canceled()
                result = PlayMotion.Result()
                result.success = False
                result.message = f"动作 {name} 被取消（已播 {elapsed:.1f}s）"
                return result
            if elapsed >= total:
                break

            pose = self._pose_at(timeline, elapsed * speed)
            vel = {
                j: v * speed
                for j, v in self._vel_at(timeline, elapsed * speed).items()
            }
            self._adapter.command(pose, speed_scale=speed, velocity=vel)

            if elapsed - last_fb >= FEEDBACK_PERIOD_S:
                fb = PlayMotion.Feedback()
                fb.progress = float(min(elapsed / total, 1.0))
                goal_handle.publish_feedback(fb)
                last_fb = elapsed
            time.sleep(period)

        # 收尾：回到 home 并保持一小段，避免动作结束后姿态松垮
        self._adapter.hold(home, self._hold_s, speed_scale=speed)
        goal_handle.succeed()
        result = PlayMotion.Result()
        result.success = True
        result.message = f"动作 {name} 播放完成（{total:.1f}s）"
        self.get_logger().info(result.message)
        return result

    # ---------------------------------------------------------------- 待机扫视
    def _on_sbus(self, msg) -> None:
        """按键门控：H 三档（右档=平台简单动作） + F上+长按A（语音功能开/关）。"""
        ev = msg.key_event_new

        # -- H 三档 --
        if ev == KEY_H_RIGHT:
            if not self._h_armed:
                self._h_armed = True
                self.get_logger().info("H 已右拨（平台简单动作待命）：待机扫视暂停")
        elif ev in (KEY_H_MID, KEY_H_LEFT):
            if self._h_armed:
                self._h_armed = False
                self.get_logger().info("H 离开右档：待机扫视恢复")

        # -- F 拨杆（语音组合键前提：F 停在上档） --
        if ev == KEY_F_UP:
            self._f_up = True
        elif ev in (KEY_F_MID, KEY_F_DOWN):
            self._f_up = False

        # -- F上 + 长按 A：切换语音功能开关 --
        if ev == KEY_A_DOWN:
            self._a_down_t = time.monotonic()
        elif ev == KEY_A_UP and self._a_down_t is not None:
            hold = time.monotonic() - self._a_down_t
            self._a_down_t = None
            if self._idle_pause_on_voice and self._f_up and hold >= self._voice_long_press_s:
                self._voice_active = not self._voice_active
                if self._voice_active:
                    self.get_logger().info(
                        f"语音功能开启（F上+长按A {hold:.1f}s）：待机扫视暂停，头/腰回 home 中立位"
                    )
                else:
                    # 语音关闭：立即放行扫视（无需再等 idle_scan_delay_s）
                    self._last_motion_end = time.monotonic() - self._idle_delay_s
                    self.get_logger().info(
                        "语音功能关闭：待机扫视恢复"
                    )
            else:
                self.get_logger().debug(
                    f"A {hold:.1f}s 未构成 F上+长按A（F_up={self._f_up}），语音开关不变"
                )

    def _voice_gate(self) -> bool:
        """扫视是否因语音功能开启而暂停（并回 home 中立位）。"""
        return self._idle_pause_on_voice and self._voice_active

    def _idle_worker(self) -> None:
        """后台轮询：无动作且在位静止 idle_scan_delay_s 后，进入无缝待机扫视。"""
        while not self._idle_shutdown.is_set():
            time.sleep(IDLE_POLL_S)
            if self._idle_shutdown.is_set():
                return
            if self._is_active() or self._idle_stop.is_set():
                continue
            if self._h_armed:
                self._last_motion_end = time.monotonic()
                continue
            # 语音功能开启时绕过静止延时，直接进入「回中立位并保持」；
            # 关闭后由 _on_sbus 直接放行，同样无需再等延时
            if (not self._voice_gate()
                    and time.monotonic() - self._last_motion_end < self._idle_delay_s):
                continue
            if not self._idle_lock.acquire(blocking=False):
                continue  # 有动作在跑，避开
            try:
                if self._is_active() or self._idle_stop.is_set() or self._h_armed:
                    continue
                self._stream_idle_loop()
            except Exception as exc:  # noqa: BLE001 — 线程内兜底，避免线程退出
                self.get_logger().error(f"待机扫视异常：{exc}")
                self._last_motion_end = time.monotonic()
            finally:
                self._idle_lock.release()

    def _stream_idle_loop(self) -> None:
        """循环播放 idle_scan_loop；只动头/腰，其余关节保持实测起点不动。

        起点取 /robot_state 实测并用 smoothstep 软过渡到扫视首帧，避免开机/平台
        直发姿态被瞬间拽回；循环用 nominal 时间轴取模，首尾同姿故接缝连续。
        """
        frames = self._library.frames(self._idle_motion)
        if len(frames) < 2 or frames[-1][0] <= 1e-6:
            self.get_logger().warn(f"待机扫视动作 {self._idle_motion!r} 关键帧不足，跳过")
            return

        ok, detail = self._adapter.check_channels(self._required_groups)
        if not ok:
            self._last_motion_end = time.monotonic()
            self.get_logger().debug(f"待机扫视前置检查未过（{detail}），稍后重试")
            return
        ok_h, detail_h = self._adapter.motor_health(self._required_groups)
        if not ok_h:
            self._last_motion_end = time.monotonic()
            self.get_logger().debug(f"待机扫视电机未就绪（{detail_h}），稍后重试")
            return

        speed = self._idle_speed
        nominal = frames[-1][0]
        rate = max(self._adapter.publish_rate_hz, 1.0)
        period = 1.0 / rate

        start = self._adapter.start_pose(self._library.home)
        # 非头/腰关节（含双臂）全程保持实测起点，避免把外来姿态拽回 home
        held = {k: v for k, v in start.items() if not k.startswith(IDLE_SCAN_PREFIXES)}
        target = dict(held)
        target.update(
            {k: v for k, v in frames[0][1].items() if k.startswith(IDLE_SCAN_PREFIXES)}
        )

        if self._voice_gate():
            # 语音功能开启：不扫描，直接平滑回 home 中立位并持续保持
            self._stream_voice_home(held, speed, rate, period)
            return

        entry_t = max(self._idle_entry_s, 1e-3)
        entry = [(0.0, start), (entry_t, target)]
        n_entry = max(int(entry_t * rate), 1)
        self.get_logger().info(
            f"进入待机扫视 {self._idle_motion!r}：循环 speed_scale={speed:.2f}，"
            f"仅头/腰运动，{entry_t:.1f}s 软过渡到起点"
        )
        try:
            for i in range(n_entry):
                if (self._idle_stop.is_set() or self._is_active()
                        or self._h_armed or self._voice_gate()):
                    return
                t = (i + 1) * entry_t / n_entry
                self._adapter.command(
                    self._pose_at(entry, t),
                    speed_scale=speed,
                    velocity={j: v * speed for j, v in self._vel_at(entry, t).items()},
                )
                time.sleep(period)

            t0 = time.monotonic()
            while not self._idle_stop.is_set() and not self._is_active():
                if self._h_armed or self._idle_shutdown.is_set():
                    break
                if self._voice_gate():
                    # 语音功能开启：让位并平滑回 home 中立位（关闭后从循环起点无缝续扫）
                    self._stream_voice_home(held, speed, rate, period)
                    t0 = time.monotonic()
                    continue
                local = ((time.monotonic() - t0) * speed) % nominal
                src = self._pose_at(frames, local)
                pose = dict(held)
                pose.update(
                    {k: v for k, v in src.items() if k.startswith(IDLE_SCAN_PREFIXES)}
                )
                vel = {
                    k: v * speed
                    for k, v in self._vel_at(frames, local).items()
                    if k.startswith(IDLE_SCAN_PREFIXES)
                }
                self._adapter.command(pose, speed_scale=speed, velocity=vel)
                time.sleep(period)
        finally:
            self.get_logger().info("待机扫视结束，等待新动作或指令")

    def _stream_voice_home(self, held: Dict[str, float], speed: float,
                           rate: float, period: float) -> None:
        """语音功能开启：头/腰从当前扫视姿态平滑回到 home 中立位并持续保持。

        · 非头/腰关节全程保持 held（实测起点）不动；
        · 实机位置模式命令需连续下发才生效，故回位后进入「保持」循环，
          直到语音关闭、正式动作/H档/退出抢占为止。
        """
        home = self._library.home
        measured = self._adapter.start_pose(home)
        start = dict(held)
        start.update(
            {k: v for k, v in measured.items() if k.startswith(IDLE_SCAN_PREFIXES)}
        )
        target = dict(held)
        target.update(
            {k: v for k, v in home.items() if k.startswith(IDLE_SCAN_PREFIXES)}
        )

        dur = max(self._idle_entry_s, 1e-3)
        timeline = [(0.0, start), (dur, target)]
        n = max(int(dur * rate), 1)
        self.get_logger().info(
            f"语音功能开启：待机扫视让位，头/腰 {dur:.1f}s 平滑回 home 中立位并保持"
        )
        for i in range(n):
            if (self._idle_stop.is_set() or self._is_active()
                    or self._idle_shutdown.is_set() or not self._voice_gate()):
                return
            t = (i + 1) * dur / n
            self._adapter.command(
                self._pose_at(timeline, t),
                speed_scale=speed,
                velocity={j: v * speed for j, v in self._vel_at(timeline, t).items()},
            )
            time.sleep(period)

        # 持续保持中立位（实机命令需连续下发），直到语音关闭/动作或H档抢占
        while (self._voice_gate() and not self._idle_stop.is_set()
               and not self._is_active() and not self._h_armed
               and not self._idle_shutdown.is_set()):
            self._adapter.command(target, speed_scale=speed)
            time.sleep(period)

    def destroy_node(self):
        self._idle_shutdown.set()
        self._idle_stop.set()
        thread = self._idle_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        return super().destroy_node()

    # ---------------------------------------------------------------- 时间轴插值
    def _warmup(self, pose: Dict[str, float]) -> None:
        """软化 SDK 模式移交的一次性瞬态。

        保持 pose 不动（=当前实测位，误差≈0），同时：
          · 电流上限从 warmup_current_start 线性升到 1.0，限制移交瞬间的冲击力；
          · 手臂力位混合 Kp 从 WARMUP_KP_START 线性升到 1.0，进一步吸收刚度阶跃。
        位置模式分组（头/腰）沿用 speed_override 的柔和速度上限。
        """
        if self._warmup_s <= 0.0:
            return
        rate = max(self._adapter.publish_rate_hz, 1.0)
        n = max(int(self._warmup_s * rate), 1)
        c0 = min(max(self._warmup_current_start, 0.0), 1.0)
        k0 = min(max(self._warmup_kp_start, 0.0), 1.0)
        self.get_logger().info(
            f"warmup {self._warmup_s:.1f}s：保持当前位姿，电流 {c0:.1f}->1.0，"
            f"Kp×{k0:.1f}->1.0，位置模式速度上限 {self._warmup_speed}"
        )
        for i in range(n):
            r = (i + 1) / n
            self._adapter.command(
                pose,
                current_scale=c0 + (1.0 - c0) * r,
                speed_override=self._warmup_speed,
                kp_scale=k0 + (1.0 - k0) * r,
            )
            time.sleep(1.0 / rate)

    @staticmethod
    def _pose_at(
        timeline: List[Tuple[float, Dict[str, float]]], t: float
    ) -> Dict[str, float]:
        """在展开后的关键帧时间轴上按名义时间 t 做【平滑(smoothstep)】插值。

        线性插值在关键帧反折处速度不连续（速度阶跃），实机会表现为顿挫/抖动；
        用 smoothstep a²(3-2a) 让每段起止速度都趋近 0，速度连续、无阶跃。
        """
        if t <= timeline[0][0]:
            return dict(timeline[0][1])
        if t >= timeline[-1][0]:
            return dict(timeline[-1][1])
        for i in range(1, len(timeline)):
            t1, p1 = timeline[i]
            if t <= t1:
                t0, p0 = timeline[i - 1]
                span = t1 - t0
                a = 0.0 if span <= 1e-9 else (t - t0) / span
                a = a * a * (3.0 - 2.0 * a)  # smoothstep：消除关键帧速度阶跃
                out: Dict[str, float] = {}
                for key in set(p0) | set(p1):
                    v0 = p0.get(key, p1[key])
                    v1 = p1.get(key, p0[key])
                    out[key] = v0 + a * (v1 - v0)
                return out
        return dict(timeline[-1][1])

    @staticmethod
    def _vel_at(
        timeline: List[Tuple[float, Dict[str, float]]], t: float
    ) -> Dict[str, float]:
        """smoothstep 轨迹的【解析角速度】(rad/s，名义时间轴)。

        比「相邻命令差分」干净：差分会被下发时刻抖动放大（50 Hz 循环里 sleep 的实际
        周期有 ±10~20% 波动），再经力位混合的 Kd 放大成功力矩纹波 → 持续高频啸叫。
        解析导数无此噪声。调用方需再乘以 speed（名义时间 -> 实际时间的缩放）。
        """
        if t <= timeline[0][0] or t >= timeline[-1][0]:
            return {}
        for i in range(1, len(timeline)):
            t1, p1 = timeline[i]
            if t <= t1:
                t0, p0 = timeline[i - 1]
                span = t1 - t0
                if span <= 1e-9:
                    return {}
                a = (t - t0) / span
                # d/dt [p0 + a²(3-2a)(p1-p0)] = 6a(1-a)/span * (p1-p0)
                k = 6.0 * a * (1.0 - a) / span
                out: Dict[str, float] = {}
                for key in set(p0) | set(p1):
                    out[key] = k * (p1.get(key, p0[key]) - p0.get(key, p1[key]))
                return out
        return {}


def main() -> None:
    rclpy.init()
    node = MotionServer()
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