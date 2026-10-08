#!/usr/bin/env python3
"""实机侧 NavigateTo 服务端 —— 包 /greeting/navigate_to (greeting_interfaces/action/NavigateTo)。

职责：接收 ``waypoint`` 点位名，走底盘 REST ``:9090`` 把机器人移到该点位，
并按契约发布 ``navigating`` / ``arrived`` / ``retrying`` / ``relocalizing`` 反馈。

接口契约（规划方案 §7.6 / §3.3）：
    下发 action → 轮询 GET action_id → 判定到达/超时 → 记录位姿 → 推进
    （REST 无服务端队列，必须自建任务状态机）

    Pre-flight          get_robot_health() + get_pose()（+ localization/status）
    到点判定            位置 ≤10cm、角度 ≤2° + 静止确认
    失败处理            超时 → 重试 1 次 → alg_relocalize → 原地接待 + 告警
    固定范围边界        应用层坐标围栏（点位超出即 REJECT）
    L3 底盘守护         REST 不可达则全程原地接待

与仿真桩的关系：``greeting_sim_stubs/navigate_to_server.py``（Nav2）与
本节点（REST）**不同时运行**，二者提供同一个 ``/greeting/navigate_to`` 契约，
编排层无需感知差异。

🔴 安全边界（务必阅读）：
    - 底盘 SDK **无软件急停**、无导航暂停/恢复：安全依赖物理急停 + 工作人员就近值守
    - ``dry_run=true`` 时机器人**不会移动**，仅验证 Action 链路
    - 本方案不能替代实机测试，最终效果必须现场验证

参数（详见 launch/navigate_to.launch.py）：
    waypoints_file / host / rest_port / request_timeout_s / default_timeout_s
    poll_period_s / feedback_period_s / arrive_pos_tol_m / arrive_yaw_tol_deg
    still_confirm_s / still_pos_tol_m / still_yaw_tol_deg / min_confidence
    max_retries / retry_backoff_s / relocalize_on_failure / relocalize_wait_s
    dry_run / dry_run_duration_s / apply_speed_limit / max_moving_speed_mps
    max_angular_speed_radps / range_limit_x / range_limit_y
"""
from __future__ import annotations

import math
import time
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from greeting_interfaces.action import NavigateTo
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from greeting_nav.chassis_client import (ChassisClient, ChassisNavError,
                                         MOVE_WITH_THETA, NavError)

#: 动作状态里出现这些词（小写）即判为「显式失败」（保守策略，见模块 docstring）
_FAIL_TOKENS = {"fail", "failed", "failure", "error", "cancelled", "canceled",
                "abort", "aborted"}


def normalize_angle(rad: float) -> float:
    """把角度规整到 [-PI, PI]。"""
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def _flatten_values(obj: Any) -> List[Any]:
    """递归展平 dict/list，取出所有标量（用于动作状态的关键字扫描）。"""
    out: List[Any] = []
    if isinstance(obj, dict):
        for value in obj.values():
            out.extend(_flatten_values(value))
    elif isinstance(obj, (list, tuple)):
        for value in obj:
            out.extend(_flatten_values(value))
    else:
        out.append(obj)
    return out


class NavigateToServer(Node):
    """把点位名翻译成底盘 REST 导航任务的 NavigateTo 动作服务端。"""

    def __init__(self) -> None:
        super().__init__("greeting_navigate_to")

        # ---------------------------------------------------- 参数声明
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("host", "192.168.11.10")
        self.declare_parameter("rest_port", 9090)
        self.declare_parameter("request_timeout_s", 10.0)
        self.declare_parameter("default_timeout_s", 60.0)
        self.declare_parameter("poll_period_s", 0.2)
        self.declare_parameter("feedback_period_s", 0.1)
        self.declare_parameter("arrive_pos_tol_m", 0.10)
        self.declare_parameter("arrive_yaw_tol_deg", 2.0)
        self.declare_parameter("still_confirm_s", 0.5)
        self.declare_parameter("still_pos_tol_m", 0.02)
        self.declare_parameter("still_yaw_tol_deg", 0.5)
        self.declare_parameter("min_confidence", 0.6)
        self.declare_parameter("max_retries", 1)
        self.declare_parameter("retry_backoff_s", 2.0)
        self.declare_parameter("relocalize_on_failure", True)
        self.declare_parameter("relocalize_wait_s", 5.0)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("dry_run_duration_s", 6.0)
        self.declare_parameter("apply_speed_limit", False)
        self.declare_parameter("max_moving_speed_mps", 0.4)
        self.declare_parameter("max_angular_speed_radps", 0.5)
        self.declare_parameter("range_limit_x", [-3.0, 3.0])
        self.declare_parameter("range_limit_y", [-3.0, 3.0])

        self._default_timeout = float(self.get_parameter("default_timeout_s").value)
        self._poll_period = float(self.get_parameter("poll_period_s").value)
        self._feedback_period = float(self.get_parameter("feedback_period_s").value)
        self._pos_tol = float(self.get_parameter("arrive_pos_tol_m").value)
        self._yaw_tol = math.radians(
            float(self.get_parameter("arrive_yaw_tol_deg").value))
        self._still_confirm = float(self.get_parameter("still_confirm_s").value)
        self._still_pos_tol = float(self.get_parameter("still_pos_tol_m").value)
        self._still_yaw_tol = math.radians(
            float(self.get_parameter("still_yaw_tol_deg").value))
        self._min_confidence = float(self.get_parameter("min_confidence").value)
        self._max_retries = max(int(self.get_parameter("max_retries").value), 0)
        self._retry_backoff = float(self.get_parameter("retry_backoff_s").value)
        self._relocalize_on_failure = bool(
            self.get_parameter("relocalize_on_failure").value)
        self._relocalize_wait = float(
            self.get_parameter("relocalize_wait_s").value)
        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._dry_run_duration = float(
            self.get_parameter("dry_run_duration_s").value)

        # ---------------------------------------------------- 点位表
        wp_file = str(self.get_parameter("waypoints_file").value)
        if not wp_file:
            wp_file = str(
                Path(get_package_share_directory("greeting_nav"))
                / "config" / "waypoints.yaml"
            )
        data = yaml.safe_load(Path(wp_file).read_text(encoding="utf-8")) or {}
        self._frame_id = str(data.get("frame_id", "map"))
        self._waypoints: Dict[str, Dict[str, Any]] = data.get("waypoints", {}) or {}
        if not self._waypoints:
            raise RuntimeError(f"点位表为空，请检查 {wp_file}")
        self.get_logger().info(
            f"点位表已加载: {wp_file}（frame_id={self._frame_id}，"
            f"{len(self._waypoints)} 个点位: {list(self._waypoints)}）"
        )

        # 围栏：优先点位表内的 range_limit，缺失则用参数
        fence = data.get("range_limit") or {}
        self._fence_x = self._read_fence_axis(fence.get("x"), "range_limit_x")
        self._fence_y = self._read_fence_axis(fence.get("y"), "range_limit_y")
        self.get_logger().info(
            f"坐标围栏（应用层软限位）: x∈{self._fence_x} y∈{self._fence_y}"
        )

        # ---------------------------------------------------- 底盘客户端
        self._client = ChassisClient(
            host=str(self.get_parameter("host").value),
            rest_port=int(self.get_parameter("rest_port").value),
            timeout_s=float(self.get_parameter("request_timeout_s").value),
        )
        self.get_logger().info(f"底盘 REST 目标: {self._client.api_url}")

        if bool(self.get_parameter("apply_speed_limit").value):
            self._apply_speed_limit()

        if self._dry_run:
            self.get_logger().warn(
                "=" * 68
                + "\n⚠  DRY-RUN 模式：机器人【不会真的移动】，仅验证 Action 链路。"
                + "\n⚠  现场实机接待前必须把 dry_run 设为 false。"
                + "\n" + "=" * 68
            )

        # ---------------------------------------------------- Action 服务端
        self._group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self,
            NavigateTo,
            "/greeting/navigate_to",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._group,
        )
        self.get_logger().info("NavigateTo 服务端就绪: /greeting/navigate_to")

    # ------------------------------------------------------------ 初始化工具
    def _read_fence_axis(self, from_yaml: Any, param_name: str) -> Tuple[float, float]:
        """取某一轴的 [min, max]：YAML 优先，否则回退参数。"""
        if isinstance(from_yaml, (list, tuple)) and len(from_yaml) == 2:
            lo, hi = float(from_yaml[0]), float(from_yaml[1])
        else:
            values = list(self.get_parameter(param_name).value)
            lo, hi = float(values[0]), float(values[1])
        return (min(lo, hi), max(lo, hi))

    def _apply_speed_limit(self) -> None:
        """启动时设定全局限速（失败仅告警，不阻断启动）。"""
        lin = float(self.get_parameter("max_moving_speed_mps").value)
        ang = float(self.get_parameter("max_angular_speed_radps").value)
        try:
            self._client.set_speed_params(max_moving_speed=lin,
                                          max_angular_speed=ang)
            self.get_logger().info(
                f"已设定全局限速: 线 {lin} m/s / 角 {ang} rad/s"
            )
        except ChassisNavError as exc:
            self.get_logger().warn(f"设定限速失败（忽略）: {exc}")

    def _in_fence(self, x: float, y: float) -> bool:
        return (self._fence_x[0] <= x <= self._fence_x[1]
                and self._fence_y[0] <= y <= self._fence_y[1])

    # ------------------------------------------------------------ 回调
    def _on_goal(self, goal_request: NavigateTo.Goal) -> GoalResponse:
        name = goal_request.waypoint
        if name not in self._waypoints:
            self.get_logger().warn(
                f"拒绝目标：点位 {name!r} 不在点位表 {list(self._waypoints)} 中"
            )
            return GoalResponse.REJECT
        wp = self._waypoints[name]
        x, y = float(wp["x"]), float(wp["y"])
        if not self._in_fence(x, y):
            self.get_logger().warn(
                f"拒绝目标：点位 {name!r} 坐标 ({x:.2f}, {y:.2f}) 超出固定范围围栏 "
                f"x∈{self._fence_x} y∈{self._fence_y}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------ 主流程
    def _execute(self, goal_handle) -> NavigateTo.Result:
        req = goal_handle.request
        name = req.waypoint
        wp = self._waypoints.get(name)
        if wp is None:  # 理论上已被 _on_goal 拦掉
            return self._fail(goal_handle, f"未知点位 {name!r}")
        x, y = float(wp["x"]), float(wp["y"])
        yaw = float(wp.get("yaw", 0.0))
        timeout = float(req.timeout_s) if req.timeout_s > 0.0 else self._default_timeout

        self.get_logger().info(
            f"导航目标: {name} -> ({x:.2f}, {y:.2f}, yaw {yaw:.2f}) "
            f"@{self._frame_id}，超时 {timeout:.0f}s"
        )

        if self._dry_run:
            return self._execute_dry_run(goal_handle, name, x, y, timeout)

        attempts = self._max_retries + 1
        for attempt in range(1, attempts + 1):
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle, name)

            if attempt > 1:
                self.get_logger().warn(f"重试 {attempt - 1}/{self._max_retries}: {name}")
                self._publish_feedback(goal_handle, "retrying", float("nan"))
                time.sleep(self._retry_backoff)

            # --- Pre-flight ①：底盘健康 ---
            try:
                health = self._client.get_robot_health()
            except ChassisNavError as exc:
                return self._fail(goal_handle, self._error_text(exc))
            if not health.is_healthy:
                return self._fail(
                    goal_handle,
                    f"底盘健康异常（{health.describe()}），请原地接待")
            self.get_logger().info(f"Pre-flight 健康检查通过（{health.describe()}）")

            # --- Pre-flight ②：定位可用性（必要时重定位一次） ---
            ok, msg = self._ensure_localization(goal_handle)
            if not ok:
                return self._fail(goal_handle, msg)

            # --- 下发 ---
            try:
                action_id = self._client.move_to(x, y, yaw, MOVE_WITH_THETA)
            except ChassisNavError as exc:
                return self._fail(goal_handle, self._error_text(exc))
            self.get_logger().info(f"已下发导航任务 action_id={action_id}")

            outcome = self._poll(goal_handle, name, action_id, x, y, yaw, timeout)

            if outcome == "arrived":
                self._publish_feedback(goal_handle, "arrived", 0.0)
                goal_handle.succeed()
                result = NavigateTo.Result()
                result.success = True
                result.message = f"已到达 {name}（{x:.2f}, {y:.2f}）"
                self.get_logger().info(result.message)
                return result
            if outcome == "cancel":
                return self._cancel(goal_handle, name)

            self._safe_cancel(action_id)
            self.get_logger().warn(
                f"本轮导航{'超时' if outcome == 'timeout' else '失败'}: {name}")

        # --- 重试耗尽：最后尝试重定位，然后交由编排层原地接待 ---
        if self._relocalize_on_failure:
            self._publish_feedback(goal_handle, "relocalizing", float("nan"))
            self.get_logger().warn("导航失败，触发 alg_relocalize 重定位")
            try:
                self._client.recover_localization()
            except ChassisNavError as exc:
                self.get_logger().warn(f"重定位下发失败: {exc}")
        return self._timeout(goal_handle, name)

    # ------------------------------------------------------------ 定位预检
    def _localization_ok(self) -> bool:
        """位姿 status / confidence + 定位状态 slam_status 全部正常才算可用。"""
        pose = self._client.get_pose()
        status = pose.get("status")
        if status is not None and int(status) != 0:
            return False
        confidence = pose.get("confidence")
        if confidence is not None and float(confidence) < self._min_confidence:
            return False
        loc = self._client.get_localization_status()
        slam_status = loc.get("slam_status")
        if slam_status is not None and int(slam_status) != 0:
            return False
        return True

    def _ensure_localization(self, goal_handle) -> Tuple[bool, str]:
        """定位不可用时尝试重定位一次；返回 (是否可用, 失败说明)。"""
        try:
            if self._localization_ok():
                return True, ""
            self._publish_feedback(goal_handle, "relocalizing", float("nan"))
            self.get_logger().warn("定位不可用，触发 alg_relocalize 后复检")
            self._client.recover_localization()
        except ChassisNavError as exc:
            return False, self._error_text(exc)
        time.sleep(self._relocalize_wait)
        try:
            if self._localization_ok():
                return True, ""
        except ChassisNavError as exc:
            return False, self._error_text(exc)
        return False, "定位不可用（重定位后仍无效），请原地接待"

    # ------------------------------------------------------------ 轮询
    def _poll(self, goal_handle, name: str, action_id: int,
              x: float, y: float, yaw: float, timeout: float) -> str:
        """轮询直到到达/超时/取消/显式失败，返回 'arrived'|'timeout'|'cancel'|'failed'。

        到点判定的主判据是**位姿**（契约要求）：位置 ≤ pos_tol、角度 ≤ yaw_tol，
        且静止确认通过；``get_action_status`` 只用于提前发现显式失败。
        """
        deadline = time.monotonic() + timeout
        still: Deque[Tuple[float, float, float, float]] = deque()
        last_fb = 0.0

        while True:
            now = time.monotonic()
            if goal_handle.is_cancel_requested:
                self._safe_cancel(action_id)
                return "cancel"
            if now > deadline:
                return "timeout"

            # 显式失败探测（字段名未固化 -> 保守解析）
            try:
                status = self._client.get_action_status(action_id)
                if self._is_action_failed(status):
                    self.get_logger().warn(f"底盘报告动作失败: {status}")
                    return "failed"
            except ChassisNavError as exc:
                if exc.code == NavError.CONNECTION_ERROR:
                    self.get_logger().error(f"轮询中 REST 失联: {exc}")
                    return "failed"
                self.get_logger().warn(f"动作状态查询异常（忽略）: {exc}")

            # 位姿
            try:
                pose = self._client.get_pose()
                cx = float(pose["x"])
                cy = float(pose["y"])
                cyaw = float(pose.get("yaw", 0.0))
            except (ChassisNavError, KeyError, TypeError, ValueError) as exc:
                self.get_logger().warn(f"位姿查询异常（本轮忽略）: {exc}")
                time.sleep(self._poll_period)
                continue
            if not (math.isfinite(cx) and math.isfinite(cy)
                    and math.isfinite(cyaw)):
                self.get_logger().warn("位姿含非有限值（本轮忽略）")
                time.sleep(self._poll_period)
                continue

            dist = math.hypot(cx - x, cy - y)
            dyaw = normalize_angle(cyaw - yaw)

            if now - last_fb >= self._feedback_period:
                self._publish_feedback(goal_handle, "navigating", dist)
                last_fb = now

            still.append((now, cx, cy, cyaw))
            while len(still) > 1 and (now - still[0][0]) > self._still_confirm:
                still.popleft()

            if (dist <= self._pos_tol and abs(dyaw) <= self._yaw_tol
                    and self._still_ok(still, now)):
                self.get_logger().info(
                    f"到达 {name}: 距目标 {dist:.3f} m，角度偏差 "
                    f"{math.degrees(dyaw):.2f}°"
                )
                return "arrived"

            time.sleep(self._poll_period)

    def _still_ok(self, still: Deque[Tuple[float, float, float, float]],
                  now: float) -> bool:
        """静止确认：窗口内位姿变化量均小于阈值，判定机器人已停稳。"""
        if len(still) < 2:
            return False
        span = now - still[0][0]
        if span < self._still_confirm * 0.9:
            return False
        t0, x0, y0, yaw0 = still[0]
        for _t, sx, sy, syaw in still:
            if math.hypot(sx - x0, sy - y0) > self._still_pos_tol:
                return False
            if abs(normalize_angle(syaw - yaw0)) > self._still_yaw_tol:
                return False
        return True

    @staticmethod
    def _is_action_failed(status: Any) -> bool:
        """保守判定「显式失败」：只认明确的失败关键字，未知一律视为运行中。"""
        if not isinstance(status, dict):
            return False
        for value in _flatten_values(status):
            if isinstance(value, str) and value.strip().lower() in _FAIL_TOKENS:
                return True
        return False

    # ------------------------------------------------------------ dry-run
    def _execute_dry_run(self, goal_handle, name: str, x: float, y: float,
                         timeout: float) -> NavigateTo.Result:
        """不发任何 REST，仅走 navigating -> arrived 的反馈流程。"""
        duration = min(self._dry_run_duration, timeout)
        fake_distance = max(math.hypot(x, y), 0.1)
        self.get_logger().warn(
            f"[DRY-RUN] 不驱动底盘：{name} 将在 {duration:.1f}s 后报到达"
        )
        t0 = time.monotonic()
        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= duration:
                break
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle, name)
            self._publish_feedback(
                goal_handle, "navigating",
                float(max(fake_distance * (1.0 - elapsed / duration), 0.0)))
            time.sleep(self._feedback_period)

        self._publish_feedback(goal_handle, "arrived", 0.0)
        goal_handle.succeed()
        result = NavigateTo.Result()
        result.success = True
        result.message = f"[DRY-RUN] 模拟到达 {name}（{x:.2f}, {y:.2f}）—— 未真实移动"
        self.get_logger().warn(result.message)
        return result

    # ------------------------------------------------------------ 工具
    def _publish_feedback(self, goal_handle, phase: str, distance: float) -> None:
        feedback = NavigateTo.Feedback()
        feedback.phase = phase
        feedback.distance_remaining_m = float(distance)
        goal_handle.publish_feedback(feedback)

    def _safe_cancel(self, action_id: int) -> None:
        try:
            self._client.cancel_action([action_id])
        except ChassisNavError as exc:
            self.get_logger().warn(f"取消动作 {action_id} 失败（忽略）: {exc}")

    @staticmethod
    def _error_text(exc: ChassisNavError) -> str:
        if exc.code == NavError.CONNECTION_ERROR:
            return "底盘 REST 不可达，请全程原地接待"
        return f"底盘接口异常 {exc.code.name}: {exc.message}"

    def _fail(self, goal_handle, text: str) -> NavigateTo.Result:
        goal_handle.abort()
        result = NavigateTo.Result()
        result.success = False
        result.message = text
        self.get_logger().error(text)
        return result

    def _timeout(self, goal_handle, name: str) -> NavigateTo.Result:
        goal_handle.abort()
        result = NavigateTo.Result()
        result.success = False
        result.message = f"导航超时：{name} 未在限定时间内到达，请原地接待"
        self.get_logger().warn(result.message)
        return result

    def _cancel(self, goal_handle, name: str) -> NavigateTo.Result:
        goal_handle.canceled()
        result = NavigateTo.Result()
        result.success = False
        result.message = f"导航被取消：{name}"
        self.get_logger().warn(result.message)
        return result

    def destroy_node(self) -> bool:
        try:
            self._client.close()
        except Exception:  # noqa: BLE001 - 关闭失败不应影响退出
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = NavigateToServer()
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
