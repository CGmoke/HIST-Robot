#!/usr/bin/env python3
"""body_adapter.py —— 实机本体通道适配层（greeting_body / 成员A）

把「URDF 关节名 -> 角度(rad)」的抽象姿态，翻译成天轶 2.5 实机的
ros2_bridge_msgs 控制帧，并集中承担全部实机安全约束。

实机控制约定（《接口契约冻结表》§7.1，与 xos/joystick joint_control_service.py 实测一致）：
    · 话题： /head/cmd (HeadCtrl) / /arm/cmd (ArmCtrl) / /waist/cmd (WaistCtrl)；
    · 构帧： label=0，reserved=165（首次下发触发整机->SDK 模式移交），
             header.frame_id = head / arm / waist；
    · 左右臂 14 个电机合并为【一条】/arm/cmd（必须整组下发）；
    · 命令非一次性生效：需以 ≥10 Hz 连续下发保持；
    · 话题无订阅者时命令被静默丢弃 —— 下发前必须预检。

控制模式（与天轶 2.5 官方实现 body_control 对齐）：
    · 官方 cpp_joint_controller/tianyi_25_transport.cpp 的构帧是 mode=0 位置模式，
      kp=0.0、kd=0.0、tor=0.0，只下发 pos/spd/cur —— SDK 不传增益，刚度由驱动器
      内部决定。故本适配层【默认也用 mode=0 且不写 kp/kd】；
    · 若强行用 mode=1 力位混合并显式下发 Kp/Kd（本文件保留了这条回退路径），
      等于给驱动器叠加一套外部增益；实测在本机型上会引起持续高频啸叫，
      非官方做法，不要默认启用；
    · spd 在位置模式是「期望速度」：必须 ≥ 轨迹实际速度，否则电机持续滞后追赶
      → 抖动。官方 body_control 的做法是按【各关节行程】分配速度
      （joint_controller._calculate_speeds_for_duration：spd_j = |Δq_j| / 运动时间，
      并受 MAX_SPEED 限制），以避免行程小的关节先到再干等（rush-and-wait 抖动）。
      本层取其【逐帧瞬时】等价式：运动段 spd = |轨迹瞬时速度| ×
      FEEDFORWARD_SPEED_MARGIN（只略 >1）；仅轨迹静止的保持段才回退到「配置速度」。
      注意不可在运动段用「配置速度」兜底：段首尾瞬时速度趋零时它会把 spd 顶到远高于
      真实轨迹速度的值，电机冲到下一个设定点后干等 → 卡顿 + 满流保持嗡鸣。

安全约束（本类是不可绕过的最后一道闸）：
    · 腿部电机（51/52）永不下发（有倾倒风险）—— joint_map.yaml 中根本不含腿；
    · 每个关节按 URDF <limit> 收缩 soft_limit_margin_rad 后钳制；
    · 单帧位置增量 ≤ max_step_rad，避免阶跃跳变；
    · 手臂期望位置再按「实测反馈 ±max_follow_err_rad」钳制，防止 Kp 顶死限位粘滑颤振；
    · speed_scale 由上层钳到 ≤0.7，本层只用它衰减下发速度上限。
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import yaml
from ros2_bridge_msgs.msg import ArmCtrl, HeadCtrl, MotorCtrl, RobotState, WaistCtrl
from std_msgs.msg import Header

#: 控制模式（见 ArmCtrl/HeadCtrl/WaistCtrl.msg 注释）：0=位置 1=力位混合 2=速度 ...
#: 天轶 2.5 官方实现（Garden_task/zev/body_control/cpp_joint_controller/
#: tianyi_25_transport.cpp）一律使用 mode=0 位置模式，且 kp=kd=0.0、tor=0.0，
#: 只下发 pos/spd/cur —— 即增益由驱动器内部决定，SDK 不传增益。
MODE_POSITION = 0
MODE_HYBRID = 1
#: SDK 模式移交魔数：首次以 reserved=165 下发，驱动器才接受 SDK 位置命令
RESERVED_SDK = 165

#: 手臂力位混合增益（沿用 xos/joystick joint_control_service.py 的 xmigcs 持有态，
#: 保证动作前后刚度连续、无冲击）。Kd 提供阻尼，抑制粘滑颤振。
#: 顺序 = 肩pitch/肩roll/肩yaw/肘pitch/肘yaw/腕pitch/腕roll（左右臂同序）。
ARM_KP_HYBRID = (300.0, 300.0, 200.0, 300.0, 100.0, 100.0, 100.0)
ARM_KD_HYBRID = (15.0, 15.0, 15.0, 15.0, 5.0, 5.0, 5.0)
#: 臂关节名 -> 增益下标（用于左右臂同序取增益）
_ARM_JOINT_ORDER = (
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
    "elbow_pitch", "elbow_yaw", "wrist_pitch", "wrist_roll",
)

#: 臂关节期望位置相对实测反馈的最大超前量(rad)：防止 Kp 顶死机械限位造成粘滑颤振
MAX_FOLLOW_ERR = 0.10
#: 前馈速度的安全上限(rad/s)：对齐 body_control 的 MAX_SPEED = 3.0（天轶 2.5 官方值）
MAX_FEEDFORWARD_SPEED = 3.0
#: 位置模式【运动段】spd 相对轨迹瞬时速度的余量：spd = |ff| × margin。
#: 只需略大于 1（覆盖下发时刻抖动）。严禁再叠加「配置速度兜底」—— 那会在段首尾
#: ff≈0 时把 spd 顶到远高于真实轨迹速度的值 → rush-and-wait 卡顿 + 满流保持嗡鸣。
FEEDFORWARD_SPEED_MARGIN = 1.2
#: 位置模式 spd 绝对下限(rad/s)，对齐官方 max(..., 0.01) 地板。
#: 必须远小于任何运动段真实速度，否则退回「低限速→滞后追赶」抖动。
MIN_FEEDFORWARD_SPEED = 0.05
#: |ff| 低于此阈值判定为轨迹【完全静止】（保持段 / 本段不参与的关节，解析速度精确为 0）：
#: 目标不动、不存在 rush-and-wait，沿用配置速度作为保持与纠偏速度上限。
#: 必须取极小值：取大值会把「缓慢爬行」误判成静止并回退到配置速度，
#: 重新引入「spd 远高于真实轨迹速度」的 rush-and-wait 卡顿。
STATIONARY_FF_EPS = 1e-9

_GROUP_TOPICS = {"head": "/head/cmd", "arm": "/arm/cmd", "waist": "/waist/cmd"}
_MSG_TYPES = {"head": HeadCtrl, "arm": ArmCtrl, "waist": WaistCtrl}
#: 腿部相关分组永不在此出现（安全红线）
FORBIDDEN_GROUPS = ("leg", "left_leg", "right_leg")


def _arm_joint_index(joint: str) -> Optional[int]:
    """由关节名取左右臂统一的 0..6 下标（肩pitch..腕roll），失败返回 None。"""
    for i, base in enumerate(_ARM_JOINT_ORDER):
        if joint.startswith(base + "_"):
            return i
    return None


@dataclass(frozen=True)
class JointSpec:
    """单个关节的实机映射信息。"""

    joint: str      # URDF 关节名，如 shoulder_roll_l_joint
    group: str      # 控制分组：head / arm / waist
    motor_id: int   # 实机电机 ID
    lower: float    # URDF 行程下限 (rad)
    upper: float    # URDF 行程上限 (rad)


class BodyAdapter:
    """实机上半身（头/双臂/腰）控制通道封装。"""

    def __init__(
        self,
        node,
        joint_map_file: str,
        *,
        soft_limit_margin_rad: float = 0.05,
        max_step_rad: float = 0.2,
        publish_rate_hz: float = 10.0,
        arm_speed: float = 0.5,
        arm_current: float = 5.0,
        head_speed: float = 0.5,
        head_current: float = 5.0,
        waist_speed: float = 0.5,
        waist_current: float = 5.0,
        arm_control_mode: int = MODE_POSITION,
        max_follow_err_rad: float = MAX_FOLLOW_ERR,
        arm_kp_scale: float = 1.0,
        arm_kd_scale: float = 1.0,
        arm_feedforward_scale: float = 1.0,
        robot_state_topic: str = "/robot_state",
        feedforward_speed_margin: float = FEEDFORWARD_SPEED_MARGIN,
        min_feedforward_speed: float = MIN_FEEDFORWARD_SPEED,
    ) -> None:
        self._node = node
        self._log = node.get_logger()

        self._margin = float(soft_limit_margin_rad)
        self._max_step = float(max_step_rad)
        self._rate = float(publish_rate_hz)
        self._reserved = RESERVED_SDK
        #: 手臂控制模式：MODE_POSITION(天轶2.5 官方做法，默认) / MODE_HYBRID(仅回退实验用)
        self._arm_mode = int(arm_control_mode)
        self._max_follow_err = float(max_follow_err_rad)
        #: 位置模式运动段 spd 的余量与绝对下限（见模块常量说明，可现场调参）
        self._ff_margin = float(feedforward_speed_margin)
        self._min_ff_speed = float(min_feedforward_speed)
        #: 力位混合增益/前馈缩放（实测调参旋钮）。
        #: 增益值取自天工3.0(dex)，本机为天轶2.5，电机与减速比不同 —— 若运动中
        #: 出现持续高频啸叫（电流环极限环/速度反馈噪声被 Kd 放大），优先下调 kp_scale。
        self._arm_kp_scale = float(arm_kp_scale)
        self._arm_kd_scale = float(arm_kd_scale)
        self._arm_ff_scale = float(arm_feedforward_scale)

        self._speeds = {"head": head_speed, "arm": arm_speed, "waist": waist_speed}
        self._currents = {"head": head_current, "arm": arm_current, "waist": waist_current}

        self._load_joint_map(joint_map_file)

        # 控制通道
        self._pubs = {}
        self._topics = {}
        for group, msg_type in _MSG_TYPES.items():
            if group in FORBIDDEN_GROUPS:  # 冗余保险，正常不会命中
                raise ValueError(f"禁止为分组 {group!r} 建立控制通道（腿部安全红线）")
            topic = _GROUP_TOPICS[group]
            self._topics[group] = topic
            self._pubs[group] = node.create_publisher(msg_type, topic, 10)

        # 关节反馈
        self._lock = threading.Lock()
        self._measured: Dict[int, float] = {}   # motor_id -> pos(rad)
        self._motor_error: Dict[int, int] = {}  # motor_id -> 电机错误码(0 为正常)
        self._last_cmd: Dict[str, float] = {}   # joint -> 上一次下发的 pos(rad)
        self._last_pub_t: Optional[float] = None  # 上一次下发时刻(monotonic)，用于兜底差分
        self._state_count = 0
        node.create_subscription(RobotState, robot_state_topic, self._on_robot_state, 10)

    # ---------------------------------------------------------------- 载入与校验
    def _load_joint_map(self, path: str) -> None:
        cfg = yaml.safe_load(open(path, "r", encoding="utf-8"))
        limits = cfg.get("limits", {})
        groups_raw = cfg.get("groups", {})
        self._frame_ids = dict(cfg.get("frame_ids", {}))

        groups: Dict[str, List[JointSpec]] = {}
        by_joint: Dict[str, JointSpec] = {}
        for group, items in groups_raw.items():
            if group in FORBIDDEN_GROUPS:
                raise ValueError(f"joint_map.yaml 含禁用分组 {group!r}（腿部安全红线）")
            if group not in _MSG_TYPES:
                raise ValueError(f"joint_map.yaml 含未知分组 {group!r}，支持 {list(_MSG_TYPES)}")
            specs: List[JointSpec] = []
            for it in items:
                joint = str(it["joint"])
                motor_id = int(it["id"])
                lim = limits.get(joint)
                if not lim or len(lim) != 2:
                    raise ValueError(f"joint_map.yaml 缺少关节 {joint!r} 的 limits")
                spec = JointSpec(joint, group, motor_id, float(lim[0]), float(lim[1]))
                specs.append(spec)
                if joint in by_joint:
                    raise ValueError(f"关节 {joint!r} 在 joint_map.yaml 中重复定义")
                by_joint[joint] = spec
            if not specs:
                raise ValueError(f"joint_map.yaml 分组 {group!r} 为空")
            groups[group] = specs

        if not groups:
            raise ValueError("joint_map.yaml 未定义任何分组")
        self._groups = groups
        self._by_joint = by_joint

        # 手臂力位混合增益表：按关节名索引（左右臂同序）
        self._arm_gains: Dict[str, Tuple[float, float]] = {}
        for spec in groups.get("arm", []):
            idx = _arm_joint_index(spec.joint)
            if idx is None:
                raise ValueError(
                    f"无法为臂关节 {spec.joint!r} 匹配力位混合增益，"
                    f"用户关节名须以 {_ARM_JOINT_ORDER} 之一开头"
                )
            self._arm_gains[spec.joint] = (ARM_KP_HYBRID[idx], ARM_KD_HYBRID[idx])

    # ---------------------------------------------------------------- 反馈解析
    def _on_robot_state(self, msg: RobotState) -> None:
        with self._lock:
            for part in (msg.head, msg.waist, msg.arm, msg.leg):
                for st in part.status:
                    mid = int(st.name)
                    self._measured[mid] = float(st.pos)
                    self._motor_error[mid] = int(st.error)
            self._state_count += 1

    def has_feedback(self) -> bool:
        with self._lock:
            return self._state_count > 0

    def measured(self) -> Dict[str, float]:
        """返回 /robot_state 中已收到的关节实测角（仅本包受控关节，不做健康判断）。"""
        with self._lock:
            snap = dict(self._measured)
        return {j: snap[s.motor_id] for j, s in self._by_joint.items() if s.motor_id in snap}

    def motor_health(self, groups: Sequence[str]) -> Tuple[bool, str]:
        """电机健康门禁：确认待用分组的电机「有反馈、无故障、角度合理」。

        实机教训：控制桥在跑、话题也有订阅者，电机仍可能处于通讯掉线状态
        （error=0x8130 "motor lost connection"），此时 pos/cur 恒为 0.0 的假值。
        仅凭 check_channels() 发现不了，必须再校验错误码与角度是否落在硬限位内，
        否则会把 0.0 当成实测起点，导致插值基线错误、动作首帧大幅跳变。
        """
        with self._lock:
            snap_pos = dict(self._measured)
            snap_err = dict(self._motor_error)
        bad: List[str] = []
        for group in groups:
            for spec in self._groups.get(group, []):
                mid = spec.motor_id
                if mid not in snap_pos:
                    bad.append(f"{spec.joint}(id{mid}) 无反馈")
                    continue
                err = snap_err.get(mid, 0)
                if err != 0:
                    bad.append(f"{spec.joint}(id{mid}) 故障码 0x{err:04X}")
                    continue
                pos = snap_pos[mid]
                if not (spec.lower - 0.2 <= pos <= spec.upper + 0.2):
                    bad.append(f"{spec.joint}(id{mid}) 角度异常 {pos:.3f} rad")
        if bad:
            return False, "电机健康检查未通过：" + "；".join(bad)
        return True, "电机健康"

    def healthy_measured(self) -> Dict[str, float]:
        """仅返回「无故障且角度合理」的实测角，供起点计算使用。"""
        with self._lock:
            snap_pos = dict(self._measured)
            snap_err = dict(self._motor_error)
        out: Dict[str, float] = {}
        for joint, spec in self._by_joint.items():
            pos = snap_pos.get(spec.motor_id)
            if pos is None or snap_err.get(spec.motor_id, 0) != 0:
                continue
            if spec.lower - 0.2 <= pos <= spec.upper + 0.2:
                out[joint] = pos
        return out

    def start_pose(self, fallback: Dict[str, float]) -> Dict[str, float]:
        """动作起点：优先用「健康」实测角；缺失或异常的项回退 fallback（通常为 home）。"""
        pose = dict(fallback)
        pose.update(self.healthy_measured())
        return pose

    # ---------------------------------------------------------------- 通道预检
    def check_channels(self, groups: Sequence[str]) -> Tuple[bool, str]:
        """确认待用通道有订阅者；否则实机命令会被静默丢弃。"""
        missing = []
        for g in groups:
            topic = self._topics.get(g)
            if topic is None:
                continue
            if self._node.count_subscribers(topic) < 1:
                missing.append(topic)
        if missing:
            return False, "以下控制通道当前无订阅者，命令不会生效：{}".format(", ".join(missing))
        return True, "控制通道就绪"

    # ---------------------------------------------------------------- 限幅
    def _clamp(self, spec: JointSpec, pos: float, reference: Optional[float]) -> float:
        lo, hi = spec.lower + self._margin, spec.upper - self._margin
        if lo > hi:  # 行程过窄时退回硬限位
            lo, hi = spec.lower, spec.upper
        pos = min(max(pos, lo), hi)
        if reference is not None:
            pos = min(max(pos, reference - self._max_step), reference + self._max_step)
        return pos

    def _clamp_follow_err(
        self, spec: JointSpec, pos: float, measured: Dict[int, float], errors: Dict[int, int]
    ) -> float:
        """把臂关节期望位置限制在「实测反馈 ± max_follow_err_rad」内。

        根因：动作角度超出实际可动范围时，Kp 持续以满电流顶死机械限位 → 粘滑颤振/异响。
        钳制后跟随误差有界、输出力有界，不再顶死。正常运动时实际误差远小于阈值，
        钳制不会触发；反馈缺失或电机故障时不钳制（退化为原行为，并由健康门禁拦下）。
        """
        act = measured.get(spec.motor_id)
        if act is None or errors.get(spec.motor_id, 0) != 0:
            return pos
        return min(max(pos, act - self._max_follow_err), act + self._max_follow_err)

    # ---------------------------------------------------------------- 下发
    def command(
        self,
        pose: Dict[str, float],
        *,
        groups: Sequence[str] = ("head", "arm", "waist"),
        speed_scale: float = 1.0,
        current_scale: float = 1.0,
        speed_override: Optional[float] = None,
        kp_scale: float = 1.0,
        velocity: Optional[Dict[str, float]] = None,
    ) -> bool:
        """下发一帧姿态。

        pose 为「关节名 -> 目标角(rad)」的部分字典；未提供的关节沿用上一次下发值。
        每个分组必须凑齐该组全部关节才会下发（臂必须整组），否则跳过并告警。

        手臂走力位混合(mode=1)：显式 Kp/Kd + 前馈速度 + 跟随误差钳制；
        头/腰走位置模式(mode=0)：spd 为期望速度上限 —— 运动段取「轨迹瞬时速度
        ×FEEDFORWARD_SPEED_MARGIN」，静止段取「配置速度 × speed_scale」。
        speed_override 仅作用于位置模式分组，且优先级最高（warmup 用）。

        velocity 为「关节名 -> 期望角速度(rad/s)」的解析前馈速度，给定时优先使用；
        否则回退为「相对上一次下发、按实际时间差」的差分速度。解析速度没有逐帧
        抖动（下发时刻抖动会被差分放大，再经 Kd 变成电流纹波 → 高频啸叫）。
        """
        now = time.monotonic()
        with self._lock:
            last = dict(self._last_cmd)
            measured = dict(self._measured)
            errors = dict(self._motor_error)
            dt = None if self._last_pub_t is None else max(now - self._last_pub_t, 1e-3)
        self._last_pub_t = now

        planned: Dict[str, float] = {}
        for group in groups:
            specs = self._groups.get(group)
            if not specs:
                continue
            targets: Dict[str, float] = {}
            incomplete = False
            for spec in specs:
                ref = last.get(spec.joint)
                raw = pose.get(spec.joint, ref)
                if raw is None:
                    incomplete = True
                    break
                pos = self._clamp(spec, float(raw), ref)
                if group == "arm":
                    pos = self._clamp_follow_err(spec, pos, measured, errors)
                targets[spec.joint] = pos
            if incomplete:
                self._log.warn(
                    f"分组 {group} 关节不完整（缺少目标与历史值），本帧跳过该组下发"
                )
                continue
            planned.update(targets)

        if not planned:
            return False

        for group in groups:
            specs = self._groups.get(group)
            if not specs or not all(s.joint in planned for s in specs):
                continue
            hybrid = group == "arm" and self._arm_mode == MODE_HYBRID
            ctrls = []
            for spec in specs:
                target = planned[spec.joint]
                ref = last.get(spec.joint)
                # 前馈速度(rad/s)：优先解析速度；否则回退实际时间差的差分（首帧 -> 0）
                if velocity is not None and spec.joint in velocity:
                    ff = float(velocity[spec.joint])
                elif ref is not None and dt is not None:
                    ff = (target - ref) / dt
                else:
                    ff = 0.0
                ff = min(max(ff, -MAX_FEEDFORWARD_SPEED), MAX_FEEDFORWARD_SPEED)
                mc = MotorCtrl()
                mc.name = spec.motor_id
                mc.pos = target
                if hybrid:
                    kp, kd = self._arm_gains.get(spec.joint, (0.0, 0.0))
                    mc.kp = kp * max(kp_scale, 0.0) * max(self._arm_kp_scale, 0.0)
                    mc.kd = kd * max(self._arm_kd_scale, 0.0)
                    mc.tor = 0.0
                    mc.spd = min(abs(ff * self._arm_ff_scale), MAX_FEEDFORWARD_SPEED)
                    if ff < 0.0:
                        mc.spd = -mc.spd
                else:
                    if speed_override is not None:
                        # warmup：上层显式指定的柔和速度上限（目标≈实测，基本不动）
                        spd = float(speed_override)
                    elif abs(ff) < STATIONARY_FF_EPS:
                        # 轨迹静止（保持段/起止帧）：目标不动，不存在 rush-and-wait，
                        # 用配置速度保证保持与纠偏能力（可用 *_speed 现场调参）。
                        spd = float(self._speeds[group]) * max(speed_scale, 0.0)
                    else:
                        # 运动段：spd 跟随轨迹【瞬时】速度 —— 官方 spd_j=|Δq_j|/T 的逐帧等价式。
                        # 不再用配置速度兜底：它会在段首尾 ff≈0 时把 spd 顶到 0.35(腰)/0.7(臂)，
                        # 远高于真实轨迹速度 → 电机冲到 20ms 设定点后干等 = 卡顿 + 满流嗡鸣。
                        spd = abs(ff) * self._ff_margin
                    mc.spd = min(max(spd, self._min_ff_speed), MAX_FEEDFORWARD_SPEED)
                mc.cur = float(self._currents[group]) * max(current_scale, 0.0)
                ctrls.append(mc)
            self._pubs[group].publish(self._make_msg(group, ctrls, hybrid))

        with self._lock:
            self._last_cmd.update(planned)
        return True

    def _make_msg(self, group: str, ctrls: List[MotorCtrl], hybrid: bool = False):
        msg = _MSG_TYPES[group]()
        msg.header = Header()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_ids.get(group, group)
        msg.mode = MODE_HYBRID if hybrid else MODE_POSITION
        msg.label = 0
        msg.reserved = self._reserved
        msg.ctrl = ctrls
        return msg

    # ---------------------------------------------------------------- 便捷方法
    def group_joints(self, group: str) -> List[str]:
        return [s.joint for s in self._groups.get(group, [])]

    @property
    def publish_rate_hz(self) -> float:
        return self._rate

    @property
    def arm_control_mode(self) -> int:
        return self._arm_mode

    @property
    def arm_gain_scales(self) -> Tuple[float, float, float]:
        """(Kp缩放, Kd缩放, 前馈缩放)，供启动日志展示。"""
        return (self._arm_kp_scale, self._arm_kd_scale, self._arm_ff_scale)

    def hold(self, pose: Dict[str, float], seconds: float, *, speed_scale: float = 1.0) -> None:
        """在给定姿态上持续下发 seconds 秒（实机命令需连续保持才生效）。"""
        deadline = time.monotonic() + max(seconds, 0.0)
        period = 1.0 / max(self._rate, 1.0)
        while True:
            self.command(pose, speed_scale=speed_scale)
            if time.monotonic() >= deadline:
                break
            time.sleep(period)