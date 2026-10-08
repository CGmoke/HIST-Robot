#!/usr/bin/env python3
"""底盘 REST 客户端 —— 天轶 2.5 底盘（knewbots 固件）的 HTTP 封装。

本模块是 ``/greeting/navigate_to`` 服务端的传输层，**不含任何 ROS 概念**，
可脱离 ROS 单独联调::

    python3 -c "
    from greeting_nav.chassis_client import ChassisClient
    c = ChassisClient()
    print(c.get_robot_health())
    print(c.get_pose())
    c.close()"

蓝本：``Garden_task/zev/knewbots_sdk/knewbot_chassis_nav.py``（按本场景裁剪：
去掉 DTC 查表、POI、虚拟轨道、shuttle、go_home 等无关能力）。

接口契约（规划方案 §3.3 / §7.6）：
    - 底盘 REST 端口 9090，默认 host 192.168.11.10
    - 运动接口是 fire-and-forget：返回 ``action_id`` 后必须自行轮询
    - 无软件急停、无导航暂停/恢复、无 cmd_vel 流式速度（安全只能靠物理急停）
    - 速度硬上限 1.5 m/s / 1.5708 rad/s（超过直接拒绝，由本模块把关）
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Dict, List, Optional

import requests

# ---------------------------------------------------------------- 默认常量
DEFAULT_HOST = "192.168.11.10"
DEFAULT_REST_PORT = 9090
DEFAULT_TIMEOUT_S = 10.0

#: moveto 的 flags：边转边走（SDK MOVE_WITH_THETA）
MOVE_WITH_THETA = 0x00000001
#: moveto 的 flags：到点后再转（SDK 中 0 = rotate at start or after arrival）
MOVE_THETA_AT_END = 0

#: 🔴 固件速度硬上限（超过固件会拒绝，这里提前拦截给出清晰报错）
MAX_MOVING_SPEED_MPS = 1.5
MAX_ANGULAR_SPEED_RADPS = 1.5708


class NavError(IntEnum):
    """错误分类（数值沿用 knewbots_sdk，便于与厂家文档对照）。"""

    OK = 0
    # 1xxx: 请求层（超时 / 参数校验）
    REQUEST_TIMEOUT = 1002
    INVALID_REQUEST = 1003
    # 3xxx: 网络 / REST
    CONNECTION_ERROR = 3001
    HTTP_ERROR = 3002
    # 5xxx: 业务逻辑
    WAYPOINT_NOT_FOUND = 5001
    LOCALIZATION_NOT_READY = 5002
    CHASSIS_HEALTH_ERROR = 5003
    # 9xxx: 兜底
    INTERNAL_ERROR = 9001


class ChassisNavError(Exception):
    """底盘接口统一异常，携带 :class:`NavError` 错误码。"""

    def __init__(self, code: NavError, message: str = "",
                 cause: Optional[Exception] = None):
        self.code = code
        self.message = message or code.name
        self.cause = cause
        super().__init__(f"[{code.value}] {self.message}")


@dataclass
class ChassisHealth:
    """``get_robot_health()`` 的结构化结果。

    ``base_error_ids`` 是原始 uint32 DTC 码，**不查表**（中文解读归
    ``greeting_health`` 负责，本模块只透传）。
    """

    has_warning: bool = False
    has_error: bool = False
    has_fatal: bool = False
    base_error_ids: List[int] = field(default_factory=list)

    @property
    def is_healthy(self) -> bool:
        """无 warning / error / fatal 时才算健康。"""
        return not (self.has_warning or self.has_error or self.has_fatal)

    def describe(self) -> str:
        """给日志用的一行摘要。"""
        if self.is_healthy:
            return "健康（无告警）"
        dtc = ", ".join(f"0x{eid:08X}" for eid in self.base_error_ids) or "无明细"
        return (f"异常 warning={self.has_warning} error={self.has_error} "
                f"fatal={self.has_fatal} DTC=[{dtc}]")


def _as_int(raw: Any) -> int:
    """把 ``12`` / ``'12'`` / ``'0x0c'`` 统一成 int；失败抛 INTERNAL_ERROR。"""
    if isinstance(raw, bool):
        raise ChassisNavError(NavError.INTERNAL_ERROR,
                              f"无法解析 action_id: {raw!r}")
    if isinstance(raw, int):
        return raw
    if isinstance(raw, float) and raw.is_integer():
        return int(raw)
    if isinstance(raw, str):
        try:
            return int(raw, 0)
        except ValueError as exc:
            raise ChassisNavError(NavError.INTERNAL_ERROR,
                                  f"无法解析 action_id: {raw!r}") from exc
    raise ChassisNavError(NavError.INTERNAL_ERROR,
                          f"无法解析 action_id: {raw!r}")


class ChassisClient:
    """底盘 REST 客户端（仅依赖 ``requests``）。"""

    def __init__(self, host: str = DEFAULT_HOST,
                 rest_port: int = DEFAULT_REST_PORT,
                 timeout_s: float = DEFAULT_TIMEOUT_S) -> None:
        self.host = str(host)
        self.rest_port = int(rest_port)
        self.timeout_s = float(timeout_s)
        self.api_url = f"http://{self.host}:{self.rest_port}"

        self._session = requests.Session()
        self._session.headers.update({
            "Accept": "application/json",
            "Content-Type": "application/json",
        })

    # ------------------------------------------------------ 资源管理
    def close(self) -> None:
        """释放 HTTP 连接池。"""
        self._session.close()

    def __enter__(self) -> "ChassisClient":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    # ------------------------------------------------------ 内部请求
    def _request(self, method: str, path: str, **kwargs) -> Any:
        """发一次 HTTP 请求并把各类网络异常分类为 :class:`ChassisNavError`。"""
        url = f"{self.api_url}{path}"
        try:
            resp = self._session.request(method, url,
                                         timeout=self.timeout_s, **kwargs)
            resp.raise_for_status()
        except requests.ConnectionError as exc:
            raise ChassisNavError(
                NavError.CONNECTION_ERROR,
                f"无法连接底盘 {self.api_url}（检查网线/IP 白名单/固件是否运行）",
                cause=exc) from exc
        except requests.Timeout as exc:
            raise ChassisNavError(
                NavError.REQUEST_TIMEOUT,
                f"请求超时: {method} {path}", cause=exc) from exc
        except requests.HTTPError as exc:
            code = NavError.HTTP_ERROR
            if resp.status_code == 400:
                code = NavError.INVALID_REQUEST
            elif resp.status_code >= 500:
                code = NavError.INTERNAL_ERROR
            raise ChassisNavError(
                code, f"REST {resp.status_code}: {resp.text[:200]}",
                cause=exc) from exc

        if "json" in resp.headers.get("Content-Type", ""):
            return resp.json()
        return resp.text

    def _get(self, path: str, **kwargs) -> Any:
        return self._request("GET", path, **kwargs)

    def _put(self, path: str, data: Optional[dict] = None, **kwargs) -> Any:
        return self._request("PUT", path, json=data, **kwargs)

    @staticmethod
    def _extract_action_id(result: Any) -> int:
        """从 ``{'action_id': ...}`` 中取出 int 形式的 action_id。"""
        if isinstance(result, dict) and "action_id" in result:
            return _as_int(result["action_id"])
        raise ChassisNavError(NavError.INTERNAL_ERROR,
                              f"响应中缺少 action_id: {result!r}")

    # ------------------------------------------------------ 健康 / 位姿
    def get_robot_health(self) -> ChassisHealth:
        """``GET /api/core/system/v1/robot/health``（每次下发前的 pre-flight）。"""
        data = self._get("/api/core/system/v1/robot/health")
        if not isinstance(data, dict):
            raise ChassisNavError(NavError.INTERNAL_ERROR,
                                  f"health 返回非 JSON 对象: {data!r}")
        raw_errors = data.get("baseError", []) or []
        ids: List[int] = []
        for eid in raw_errors:
            try:
                ids.append(_as_int(eid))
            except ChassisNavError:
                continue  # 个别码解析不了不影响整体健康判定
        return ChassisHealth(
            has_warning=bool(data.get("hasWarning", False)),
            has_error=bool(data.get("hasError", False)),
            has_fatal=bool(data.get("hasFatal", False)),
            base_error_ids=ids,
        )

    def get_pose(self) -> Dict[str, Any]:
        """``GET /api/core/navigation/v1/pose`` -> dict(x, y, yaw, confidence, status)。

        ``status`` 非 0 表示定位异常（需 ``recover_localization()``）。
        """
        data = self._get("/api/core/navigation/v1/pose")
        if not isinstance(data, dict):
            raise ChassisNavError(NavError.INTERNAL_ERROR,
                                  f"pose 返回非 JSON 对象: {data!r}")
        return data

    def get_localization_status(self) -> Dict[str, Any]:
        """``GET /api/core/slam/v1/localization/status`` -> dict(slam_type, slam_status)。

        slam_status: 0 正常 / 1 丢失 / 2 精度下降 / 3 异常 /
                     4 空闲 / 5 等待重定位 / 6 重定位中 / 7 加载地图 / 8 等待传感器
        """
        data = self._get("/api/core/slam/v1/localization/status")
        if not isinstance(data, dict):
            raise ChassisNavError(NavError.INTERNAL_ERROR,
                                  f"localization/status 返回非 JSON 对象: {data!r}")
        return data

    # ------------------------------------------------------ 运动
    def move_to(self, x: float, y: float, yaw: float = 0.0,
                flags: int = MOVE_WITH_THETA) -> int:
        """``PUT /api/core/navigation/v1/actions/moveto``，返回 ``action_id``。

        Args:
            x, y: 目标位置（m，map 帧）。
            yaw: 目标朝向（rad，[-PI, PI]）。
            flags: 默认边转边走。

        注意：fire-and-forget，必须自行轮询位姿或动作状态判定到达。
        """
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ChassisNavError(NavError.INVALID_REQUEST,
                                  f"move_to 参数非有限值: x={x} y={y} yaw={yaw}")
        data = {"pose": {"x": float(x), "y": float(y), "yaw": float(yaw)},
                "flags": int(flags)}
        result = self._put("/api/core/navigation/v1/actions/moveto", data=data)
        return self._extract_action_id(result)

    def move_by(self, direction: float = 0.0, distance: int = 0,
                move_mode: int = 0, v: float = 0.0, w: float = 0.0) -> int:
        """``PUT /api/core/navigation/v1/actions/moveby``（相对移动，不依赖 SLAM）。

        本版服务端不主动调用，仅保留接口以便定位异常时脱困。

        Args:
            direction: 相对朝向（rad，[-PI, PI]）。
            distance: 距离（mm，[50, 5000]；0 = 不移动）。
            move_mode: 0 = 平移，1 = 原地转。
            v: 线速度上限（m/s，0 = 不限）；w: 角速度上限（rad/s，0 = 不限）。
        """
        if not (-math.pi <= direction <= math.pi):
            raise ChassisNavError(NavError.INVALID_REQUEST,
                                  f"direction={direction} 超出 [-PI, PI]")
        if move_mode == 0 and distance != 0 and not (50 <= distance <= 5000):
            raise ChassisNavError(
                NavError.INVALID_REQUEST,
                f"distance={distance} 超出 [50, 5000]（0=不移动）")
        data = {"move_mode": int(move_mode), "direction": float(direction),
                "distance": int(distance), "v": float(v), "w": float(w)}
        result = self._put("/api/core/navigation/v1/actions/moveby", data=data)
        return self._extract_action_id(result)

    def get_action_status(self, action_id: int) -> Dict[str, Any]:
        """``GET /api/core/motion/v1/actions/{action_id}``。

        ⚠️ 该返回的字段名厂家文档未固化，**不可作为到点主判据**；
        服务端只用它提前发现「显式失败」，到点判定以 :meth:`get_pose` 为准。
        """
        data = self._get(f"/api/core/motion/v1/actions/{int(action_id)}")
        if not isinstance(data, dict):
            return {"raw": data}
        return data

    def cancel_action(self, action_list: Optional[List[int]] = None) -> Dict[str, Any]:
        """``PUT /api/core/navigation/v1/cancel_action``（空列表 = 取消全部）。

        无「导航暂停/恢复」能力：主持人暂停 = 取消 + 记录现场，恢复时重新下发。
        """
        data = {"action_list": [int(a) for a in (action_list or [])]}
        result = self._put("/api/core/navigation/v1/cancel_action", data=data)
        return result if isinstance(result, dict) else {}

    # ------------------------------------------------------ 恢复 / 限速
    def recover_localization(self, x: float = 0.0, y: float = 0.0,
                             yaw: float = 0.0) -> int:
        """``PUT /api/core/slam/v1/localization/recover``（重定位），返回 ``action_id``。"""
        data = {"pose": {"x": float(x), "y": float(y), "yaw": float(yaw)}}
        result = self._put("/api/core/slam/v1/localization/recover", data=data)
        return self._extract_action_id(result)

    def set_speed_params(self, max_moving_speed: Optional[float] = None,
                         max_angular_speed: Optional[float] = None) -> None:
        """``PUT /api/core/system/v1/parameter`` 设定**全局**速度上限。

        🔴 moveto 接口本身不接受速度参数，这是唯一的限速杠杆，且为全局生效。
        固件硬上限：线速度 ≤1.5 m/s、角速度 ≤1.5708 rad/s。

        Args:
            max_moving_speed: 线速度上限（m/s）。
            max_angular_speed: 角速度上限（rad/s）。

        Raises:
            ChassisNavError: 参数超限或两个参数都未提供。
        """
        data: Dict[str, float] = {}
        if max_moving_speed is not None:
            if max_moving_speed > MAX_MOVING_SPEED_MPS:
                raise ChassisNavError(
                    NavError.INVALID_REQUEST,
                    f"max_moving_speed={max_moving_speed} 超过硬上限 "
                    f"{MAX_MOVING_SPEED_MPS} m/s")
            data["max_moving_speed"] = float(max_moving_speed)
        if max_angular_speed is not None:
            if max_angular_speed > MAX_ANGULAR_SPEED_RADPS:
                raise ChassisNavError(
                    NavError.INVALID_REQUEST,
                    f"max_angular_speed={max_angular_speed} 超过硬上限 "
                    f"{MAX_ANGULAR_SPEED_RADPS} rad/s")
            data["max_angular_speed"] = float(max_angular_speed)
        if not data:
            raise ChassisNavError(NavError.INVALID_REQUEST,
                                  "max_moving_speed / max_angular_speed 至少给一个")
        self._put("/api/core/system/v1/parameter", data=data)
