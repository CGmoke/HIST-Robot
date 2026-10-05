"""Modbus-RTU 并联夹爪驱动（Robotiq 风格的寄存器映射）。

本驱动所控制的夹爪，是那种随处可见的"写三个寄存器 / 读三个寄存器"的 Modbus-RTU
并联夹爪：命令块位于低寄存器基址，状态块位于高寄存器基址，二者都打包进 16 位
寄存器，且每个字只有**高字节**被使用。所有与设备相关的内容都放在
:class:`GripperProfile` 中，因此换用另一个厂商的映射只需新建一个 profile，而
无需新写一个驱动。

关于这个硬件，有两点值得先说明，因为整个 API 都是围绕它们塑造的：

**状态字是一个位域，其中一个字段回答"我正抓着东西吗？"。** ``gOBJ`` 区分
*手指仍在移动*（0）、*手指在张开时停下*（1——物体太大抓不住）、*手指在闭合时
停下*（2——**物体已被抓住**）、以及*手指到达了指令位置*（3——里面什么也没有）。
把 2 与 3 搞混，就是成功抓取与手臂空手举起之间的区别：二者看起来都像"运动
结束"。

**抓取只能在更紧的方向上重试。** 一旦报告 ``gOBJ == 2``，手指正以恰好被指令
的力抓着负载。把它们张开——哪怕一点点，哪怕"只是为了重新放置"——都会松开物体。
因此 :meth:`ModbusGripper.squeeze_more` 绝不会指令一个比当前更宽的位置；它只能
收紧。

依赖项（``minimalmodbus`` 与 ``pyserial``）在 ``__init__`` 内部导入，因此本模块
在没有夹爪、也没装驱动栈的机器上仍能干净地导入——这正是编排代码及其测试可以
无条件导入它的原因。传入 ``dry_run=True`` 可针对内部模拟而非硬件来演练整个命令
序列。

示例
-------
::

    from reusable_model.hardware.modbus_gripper import ModbusGripper

    with ModbusGripper("/dev/serial/by-id/usb-FTDI_...-if00-port0") as gripper:
        gripper.reset()
        gripper.activate()
        gripper.close(0.9)                       # 指令一次稳固的闭合
        status = gripper.read_status()
        if not gripper.is_object_held(status, target=0.9):
            status = gripper.squeeze_more(1.0)   # 重试，只能更紧
        print(gripper.status_text(status))
"""

from __future__ import annotations

import dataclasses
import logging
import time
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

logger = logging.getLogger(__name__)

__all__ = [
    "GripperProfile",
    "ROBOTIQ_2F",
    "OBJECT_STATES",
    "FAULT_CODES",
    "MAJOR_FAULTS",
    "ModbusGripper",
]

#: Modbus 功能码，表示"读取保持寄存器"。它属于 Modbus 标准而非设备特性，因此是
#: 常量而不是 profile 字段。
_READ_FUNCTION_CODE = 3

#: 动作请求寄存器的 ``gACT`` 位："激活夹爪"。
_ACTIVATE_BIT = 0x0100

#: 动作请求寄存器的 ``gGTO`` 位："执行所请求的移动"。
_GOTO_BIT = 0x0800


#: 每个 ``gOBJ`` 值的含义。这四个状态正是本驱动的全部意义所在——它们是唯一能区分
#: "手指停下是因为正抓着东西"（2）与"手指停下是因为到达目标、二者之间空无一物"
#: （3）的信号。
OBJECT_STATES: dict[int, str] = {
    0: "moving (fingers in motion)",
    1: "open-blocked (object too large to grasp)",
    2: "close-blocked (object held)",
    3: "in-position (fingers reached the commanded target, nothing held)",
}

#: 每个 ``gFLT`` 故障码的含义。此处未列出的码会以数字报告，而不会凭空编造含义。
FAULT_CODES: dict[int, str] = {
    0x0: "no fault",
    0x5: "action delayed (activation required; re-activate the gripper)",
    0x7: "not activated",
    0x8: "overheating",
    0x9: "communication timeout",
    0xA: "undervoltage",
    0xB: "automatic release in progress",
    0xC: "internal fault",
    0xD: "activation fault",
    0xE: "overcurrent",
    0xF: "automatic release completed",
}

#: 出现这些故障后，绝不能向夹爪下达任何指令。
#:
#: 其中每一个要么意味着驱动已经松开了负载（自动释放进行中 / 已完成、欠压），要么
#: 意味着进一步的指令可能使它松开（过流、激活故障、内部故障）。安全的反应是停止、
#: 上报，并交由人决定——尤其*绝不要*通过复位并重新张开去"恢复"，那恰恰是会丢下
#: 夹爪原本所抓物体的操作序列。
MAJOR_FAULTS: frozenset[int] = frozenset({0xA, 0xB, 0xC, 0xD, 0xE, 0xF})


@dataclass(frozen=True)
class GripperProfile:
    """某一夹爪系列不可变的 Modbus 寄存器映射与运动默认值。

    采用 frozen，是为了让一个 profile 可以在线程之间、以及多个夹爪实例之间共享
    （双臂机器人使用两个对象、一个 profile），而不会有某个调用点替所有人重调
    力度的风险。

    状态字位布局（所有字段都位于其寄存器的**高字节**，这是厂商约定——低字节未
    使用）::

        register 0 (read_base + 0), high byte = gripper status
            bit 0     gACT   夹爪已激活
            bit 3     gGTO   一次移动已被锁存 / 请求
            bits 4-5  gSTA   激活状态（参见 activated_status）
            bits 6-7  gOBJ   物体检测状态（参见 OBJECT_STATES）
        register 1 (read_base + 1), high byte = fault status
            low nibble gFLT  故障码（参见 FAULT_CODES）
        register 2 (read_base + 2), high byte = gPO
            finger position, 0 = fully open .. position_scale = fully closed

    属性:
        slave_id: 夹爪在 RS-485 总线上的 Modbus 从站地址。每条总线必须唯一；
            ``9`` 是厂商默认值。
        baudrate: 串口波特率。必须与设备自身的设置一致，该设置由厂商工具配置，
            并在断电后保持。
        write_base: 三寄存器动作请求块的起始寄存器。
        read_base: 三寄存器状态块的起始寄存器。
        speed: 默认手指速度，``0``（最慢）到 ``255``（最快）。发送在第三个请求
            寄存器的**高字节**。
        force: 默认夹持力度，``0``（最小）到 ``255``（最大）。发送在第三个请求
            寄存器的**低字节**。``180`` 对刚性物体足够稳固，又不会在其上留下
            痕迹；对沉重或易滑的负载可调高。
        activate_request: 激活夹爪的请求字（置位 ``gACT``）。
        goto_request: 指令一次移动的请求字（置位 ``gACT | gGTO``）。
        timeout: 串口读取超时，单位为秒。刻意保持很短：状态寄存器是在循环中轮询
            的，长超时会把一次丢包变成整个控制周期的卡顿。
        position_scale: 完全闭合所对应的原始 ``gPO``/目标单位。``255`` 表示位置
            字节被直接使用，即 ``target == gPO / 255``。
        activated_status: 表示"已激活并就绪"的 ``gSTA`` 值。轮询这个确切值（而
            非非零的 ``gACT``）正是让 :meth:`ModbusGripper.activate` 只在运动
            指令确实会被接受之后才返回的原因。

    异常:
        TypeError: 若某个字段不是预期类型的数字。
        ValueError: 若某个字段超出其协议范围，或某个请求字未置位定义它的位。

    示例:
        >>> ROBOTIQ_2F.slave_id, ROBOTIQ_2F.write_base, ROBOTIQ_2F.read_base
        (9, 1000, 2000)
        >>> GripperProfile(slave_id=250)
        Traceback (most recent call last):
            ...
        ValueError: slave_id must be within 0-247 (Modbus RTU), got 250
    """

    slave_id: int = 9
    baudrate: int = 115200
    write_base: int = 1000
    read_base: int = 2000
    speed: int = 255
    force: int = 180
    activate_request: int = 0x0100
    goto_request: int = 0x0900
    timeout: float = 0.2
    position_scale: int = 255
    activated_status: int = 3

    def __post_init__(self) -> None:
        """根据 Modbus 与厂商范围校验每一个字段。

        校验在构造时进行，因此一个写错的 profile 会在定义它的那一行就失败，而不是
        三分钟后以一个来自根本不存在设备的 Modbus 异常形式出现。
        """
        for name in ("slave_id", "baudrate", "write_base", "read_base", "speed",
                     "force", "activate_request", "goto_request", "position_scale",
                     "activated_status"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an int, got {type(value).__name__}: {value!r}")
        if not isinstance(self.timeout, (int, float)) or isinstance(self.timeout, bool):
            raise TypeError(f"timeout must be a number, got {type(self.timeout).__name__}: {self.timeout!r}")
        object.__setattr__(self, "timeout", float(self.timeout))

        if not 0 <= self.slave_id <= 247:
            raise ValueError(f"slave_id must be within 0-247 (Modbus RTU), got {self.slave_id}")
        if self.baudrate <= 0:
            raise ValueError(f"baudrate must be > 0, got {self.baudrate}")
        for name in ("write_base", "read_base"):
            value = getattr(self, name)
            if not 0 <= value <= 0xFFFF:
                raise ValueError(f"{name} must be a register address within 0-65535, got {value}")
        for name in ("speed", "force", "activated_status"):
            value = getattr(self, name)
            limit = 3 if name == "activated_status" else 255
            if not 0 <= value <= limit:
                raise ValueError(f"{name} must be within 0-{limit}, got {value}")
        for name in ("activate_request", "goto_request"):
            value = getattr(self, name)
            if not 0 <= value <= 0xFFFF:
                raise ValueError(f"{name} must fit in a 16-bit register, got {value}")
        if self.timeout <= 0:
            raise ValueError(f"timeout must be > 0 seconds, got {self.timeout}")
        if self.position_scale <= 0:
            raise ValueError(f"position_scale must be > 0, got {self.position_scale}")
        if not self.activate_request & _ACTIVATE_BIT:
            raise ValueError(
                f"activate_request must set the gACT bit ({_ACTIVATE_BIT:#06x}), "
                f"got {self.activate_request:#06x}"
            )
        if not self.goto_request & _GOTO_BIT:
            raise ValueError(
                f"goto_request must set the gGTO bit ({_GOTO_BIT:#06x}), "
                f"got {self.goto_request:#06x}"
            )


#: 常见的 Robotiq 2F 系列并联夹爪的 profile。
ROBOTIQ_2F = GripperProfile()

#: Profile 字段，用于对调用方提供的 profile 对象进行鸭子类型判断。
_PROFILE_FIELDS: tuple[str, ...] = tuple(f.name for f in dataclasses.fields(GripperProfile))


def _require_profile(profile: Any) -> Any:
    """校验 ``profile`` 是否暴露了每一个 :class:`GripperProfile` 字段。

    采用鸭子类型而非 ``isinstance`` 检查，这样调用方定义的等价 dataclass（或从文件
    路径加载的本模块第二份副本，插件加载器就会这么做）也能原样工作。

    参数:
        profile: 候选 profile 对象。

    返回:
        同一个对象。

    异常:
        TypeError: 若缺少某个必需属性。

    示例:
        >>> _require_profile(ROBOTIQ_2F) is ROBOTIQ_2F
        True
    """
    missing = [name for name in _PROFILE_FIELDS if not hasattr(profile, name)]
    if missing:
        raise TypeError(
            f"profile must expose {missing} (got {type(profile).__name__}: {profile!r}); "
            f"expected a modbus_gripper.GripperProfile-like object"
        )
    return profile


def _byte(value: Any, *, what: str) -> int:
    """校验一个单字节的寄存器字段。

    参数:
        value: 候选值。
        what: 出现在错误消息中的名称。

    返回:
        转换为 ``int`` 后的该值。

    异常:
        TypeError: 若 ``value`` 不是整数。
        ValueError: 若其超出 ``0-255``。

    示例:
        >>> _byte(180, what="force")
        180
    """
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{what} must be an int 0-255, got {type(value).__name__}: {value!r}")
    if not 0 <= value <= 255:
        raise ValueError(f"{what} must be within 0-255 (one register byte), got {value!r}")
    return value


def _unit_interval(value: Any, *, what: str) -> float:
    """校验一个归一化的 ``0.0 <= value <= 1.0`` 量。

    参数:
        value: 候选值。
        what: 出现在错误消息中的名称。

    返回:
        转换为 ``float`` 后的该值。

    异常:
        TypeError: 若 ``value`` 不是实数（``bool`` 会被拒绝：``True`` 目标会默默
            表示"完全闭合"）。
        ValueError: 若其超出 ``[0.0, 1.0]``。

    示例:
        >>> _unit_interval(0.9, what="target")
        0.9
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{what} must be a real number, got {type(value).__name__}: {value!r}")
    out = float(value)
    if not 0.0 <= out <= 1.0:
        raise ValueError(f"{what} must be within 0.0-1.0 (0.0 = fully open), got {value!r}")
    return out


def _positive_seconds(value: Any, *, what: str, allow_zero: bool = False) -> float:
    """校验一个时长。

    参数:
        value: 候选的秒数。
        what: 出现在错误消息中的名称。
        allow_zero: 接受 ``0``（含义为"检查一次，不等待"）。

    返回:
        转换为 ``float`` 后的该值。

    异常:
        TypeError: 若 ``value`` 不是实数。
        ValueError: 若其为负，或在 ``allow_zero`` 为 ``False`` 时为零。

    示例:
        >>> _positive_seconds(0.5, what="timeout")
        0.5
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{what} must be a number of seconds, got {type(value).__name__}: {value!r}")
    out = float(value)
    if out < 0 or (out == 0 and not allow_zero):
        raise ValueError(
            f"{what} must be {'>= 0' if allow_zero else '> 0'} seconds, got {value!r}"
        )
    return out


def _require_status(status: Any) -> dict[str, Any]:
    """校验一个已解码的状态映射。

    各方法接受一个已读取的状态，以便 20 Hz 的循环每周期只读一次设备，就能据此回答
    多个问题。这只有在映射确实是已解码状态时才成立，因此这里检查各键，而不是指望
    一个 ``KeyError`` 在某个无从下手的地方冒出来。

    参数:
        status: 候选状态映射。

    返回:
        作为普通 ``dict`` 的该映射。

    异常:
        TypeError: 若 ``status`` 不是映射。
        ValueError: 若缺少某个必需的键；消息中会列出它们。

    示例:
        >>> sorted(_require_status({k: 0 for k in ("gACT", "gGTO", "gSTA", "gOBJ", "gFLT", "gPO")}))
        ['gACT', 'gFLT', 'gGTO', 'gOBJ', 'gPO', 'gSTA']
    """
    if not isinstance(status, Mapping):
        raise TypeError(f"status must be a mapping, got {type(status).__name__}: {status!r}")
    required = ("gACT", "gGTO", "gSTA", "gOBJ", "gFLT", "gPO")
    missing = [key for key in required if key not in status]
    if missing:
        raise ValueError(
            f"status is missing the field(s) {missing}; expected a dict as returned by "
            f"ModbusGripper.read_status (required keys: {list(required)})"
        )
    return dict(status)


def _motion_in_progress(status: Mapping[str, Any]) -> bool:
    """手指是否仍在移动。

    ``gOBJ == 0`` 是寄存器映射自身对"手指正在运动"的定义。那个诱人的替代方案——观察
    ``gGTO`` 位清零——并不可靠：``gGTO`` 是*已锁存命令的回显*，而且若干固件版本在
    整个激活期间都让它保持置位，因此等待它的轮询循环要么立即返回，要么永不返回。
    ``gOBJ`` 也正是抓取测试所依赖的字段，这让"它是否在动？"与"它是否抓着？"都源自
    同一个一致的来源。

    参数:
        status: 一个已解码的状态映射。

    返回:
        当手指正在移动时为 ``True``。

    示例:
        >>> _motion_in_progress({"gOBJ": 0}), _motion_in_progress({"gOBJ": 2})
        (True, False)
    """
    return int(status["gOBJ"]) == 0


class ModbusGripper:
    """单个 Modbus-RTU 并联夹爪的驱动。

    串口在 ``__init__`` 中打开，并在对象的整个生命周期内**保持打开**。这是刻意
    的选择，理由是算术上的：20 Hz 的状态轮询每周期只有 50 ms，而
    ``close_port_after_each_call`` 会给每一次事务额外加上一对 ``open``/``close``
    系统调用*以及*一次 USB 控制传输往返。在挂接在集线器上的适配器上，这通常就是
    10-20 ms，足以把轮询循环推过其截止时间，并让抓取检测结果滞后一帧。保持句柄
    打开还能避免 ``ExclusiveOpen`` 失败——当另一个进程在重新打开的窗口期内仍持有
    该串口时就会出现这种失败。

    每次事务前都会清空缓冲区（``clear_buffers_before_each_transaction``），因为
    一个迟到的响应——在上一次读取超时之后才到达——否则会滞留在驱动缓冲区中，并被
    当作*下一个*请求的答案来解析。这种失步是静默且永久性的，而且看起来与夹爪时好
    时坏一模一样。

    参数:
        port: 串口设备路径。请使用 ``/dev/serial/by-id/...`` 符号链接而非
            ``/dev/ttyUSB*``；枚举顺序名称为何在重启后不稳定，参见
            :mod:`reusable_model.hardware.serial_discovery`。
        profile: 寄存器映射与默认值。任何暴露 :class:`GripperProfile` 字段的对象
            都可接受。
        dry_run: 为 ``True`` 时不导入任何依赖、不打开任何串口、也没有一个字节到达
            线缆。每个方法都会记录它*本会*做什么，并改动对状态寄存器的内部模拟，
            因此无需接上夹爪，就能在笔记本电脑上演练激活序列、抓取重试逻辑与状态
            解码。日志可通过 :attr:`dry_run_journal` 获取。

    异常:
        TypeError: 若 ``port`` 不是字符串，``profile`` 缺少字段，或 ``dry_run``
            不是布尔值。
        ValueError: 若 ``port`` 为空白。
        ImportError: 若未安装 ``minimalmodbus`` 或 ``pyserial``（消息中会给出安装
            命令）。在 dry-run 模式下完全跳过。
        serial.SerialException: 当串口无法打开时传播（它不存在、被另一个进程持有，
            或适配器被拔出）。

    示例:
        >>> gripper = ModbusGripper("fake-port", dry_run=True)
        >>> gripper.connected
        True
        >>> gripper.reset(); gripper.activate(timeout=0.1)
        >>> gripper.go_to(0.5); gripper.read_status()["gPO"]
        128
        >>> gripper.close_port(); gripper.connected
        False
    """

    def __init__(self, port: str, *, profile: Any = ROBOTIQ_2F, dry_run: bool = False) -> None:
        if not isinstance(port, str):
            raise TypeError(f"port must be a str device path, got {type(port).__name__}: {port!r}")
        if not port.strip():
            raise ValueError(f"port must be a non-blank device path, got {port!r}")
        if not isinstance(dry_run, bool):
            raise TypeError(f"dry_run must be a bool, got {type(dry_run).__name__}: {dry_run!r}")

        self.port: str = port.strip()
        self.profile: Any = _require_profile(profile)
        self.dry_run: bool = dry_run

        self._serial: Any = None
        self._instrument: Any = None
        self._closed = False

        # Dry-run 设备状态。它按硬件本会返回的三个寄存器来表示，并由同一个函数解码，
        # 因此模拟演练的是真实的位打包逻辑，而不是它的另一份并行实现。
        self._sim_activated = False
        self._sim_goto = False
        self._sim_position = 0.0
        self._sim_gobj = 0
        self._sim_gflt = 0
        self._journal: list[dict[str, Any]] = []

        if dry_run:
            logger.info(
                "gripper dry run on %r: no port is opened and every command is simulated",
                self.port,
            )
            return

        try:
            import minimalmodbus  # noqa: PLC0415 - 可选依赖，仅硬件模式
            import serial  # noqa: PLC0415 - 可选依赖，仅硬件模式
        except ImportError as exc:
            missing = getattr(exc, "name", "") or str(exc)
            raise ImportError(
                f"driving a gripper needs the missing module {missing}: "
                f"pip install minimalmodbus pyserial "
                f"(or construct ModbusGripper(port, dry_run=True) to run without hardware)"
            ) from exc

        handle = serial.Serial(
            self.port,
            self.profile.baudrate,
            8,          # 数据位
            "N",        # 校验位：无
            1,          # 停止位
            self.profile.timeout,
        )
        instrument = minimalmodbus.Instrument(
            handle,
            self.profile.slave_id,
            minimalmodbus.MODE_RTU,
            close_port_after_each_call=False,
        )
        instrument.clear_buffers_before_each_transaction = True
        self._serial = handle
        self._instrument = instrument
        logger.info(
            "gripper ready on %s (slave=%d, baud=%d)",
            self.port,
            self.profile.slave_id,
            self.profile.baudrate,
        )

    # ------------------------------------------------------------------- 状态

    @property
    def connected(self) -> bool:
        """当前是否可以发送命令。

        在 dry-run 模式下，它报告的是模拟连接状态，该状态会一直保持，直到调用
        :meth:`close_port`——因此以此为守卫的编排代码在有硬件与无硬件时的行为完全
        一致。

        返回:
            当串口已打开（或模拟正在运行）且尚未调用 :meth:`close_port` 时为
            ``True``。

        示例:
            >>> ModbusGripper("fake", dry_run=True).connected
            True
        """
        if self._closed:
            return False
        if self.dry_run:
            return True
        return self._serial is not None and bool(self._serial.is_open)

    @property
    def dry_run_journal(self) -> tuple[dict[str, Any], ...]:
        """被模拟而非实际发送的命令，按顺序排列。

        在硬件模式下为空。之所以要有这份日志，是因为 dry run 没有其它可观察的效果：
        正是它让测试能够断言指令了*哪些*位置——例如一次抓取重试从未把手指张开。

        返回:
            由 ``{"op": ..., ...}`` dict 组成的元组（是一份副本，因此调用方无法破坏
            内部列表）。

        示例:
            >>> g = ModbusGripper("fake", dry_run=True); g.go_to(0.5)
            >>> g.dry_run_journal[-1]["op"], g.dry_run_journal[-1]["target"]
            ('go_to', 0.5)
        """
        return tuple(dict(entry) for entry in self._journal)

    def _record(self, **entry: Any) -> None:
        """向 :attr:`dry_run_journal` 追加一条模拟命令。

        参数:
            **entry: 描述该命令的字段；``op`` 由调用方添加。

        返回:
            ``None``。

        示例:
            不直接调用；参见各命令方法。
        """
        self._journal.append(entry)
        logger.debug("gripper dry run: %s", entry)

    # ----------------------------------------------------------- 寄存器胶水

    def _require_open(self) -> None:
        """在 :meth:`close_port` 之后拒绝命令。

        返回:
            ``None``。

        异常:
            RuntimeError: 若夹爪已关闭。默默什么都不做要糟糕得多：调用方会以为它
                已经下达了释放指令。

        示例:
            不直接调用。
        """
        if self._closed:
            raise RuntimeError(
                f"gripper on {self.port!r} is closed; construct a new ModbusGripper to reuse it"
            )

    def _read_registers(self, address: int, count: int) -> list[int]:
        """读取 ``count`` 个保持寄存器。

        参数:
            address: 起始寄存器地址。
            count: 要读取的 16 位寄存器数量。

        返回:
            寄存器值。在 dry-run 模式下，这些值来自内部模拟，并通过与真实值完全
            相同的路径解码。

        异常:
            RuntimeError: 若夹爪已关闭。
            minimalmodbus.ModbusException: 总线出错时传播（无响应、CRC 校验失败）。
                控制循环中的调用方应将之视为"本周期的读数不可用"。

        示例:
            >>> len(ModbusGripper("fake", dry_run=True)._read_registers(2000, 3))
            3
        """
        self._require_open()
        if self.dry_run:
            return self._simulated_registers()[:count]
        if self._instrument is None:  # pragma: no cover - 由 _require_open 守卫
            raise RuntimeError(f"gripper on {self.port!r} has no Modbus instrument")
        return [int(v) for v in self._instrument.read_registers(address, count, _READ_FUNCTION_CODE)]

    def _write_registers(self, address: int, values: Sequence[int]) -> None:
        """写入一个保持寄存器块。

        参数:
            address: 起始寄存器地址。
            values: 要写入的寄存器值，按顺序。

        返回:
            ``None``。

        异常:
            RuntimeError: 若夹爪已关闭。
            minimalmodbus.ModbusException: 总线出错时传播。

        示例:
            >>> ModbusGripper("fake", dry_run=True)._write_registers(1000, [0, 0, 0])
        """
        self._require_open()
        payload = [int(v) & 0xFFFF for v in values]
        if self.dry_run:
            self._apply_simulated_write(address, payload)
            return
        if self._instrument is None:  # pragma: no cover - 由 _require_open 守卫
            raise RuntimeError(f"gripper on {self.port!r} has no Modbus instrument")
        self._instrument.write_registers(address, payload)

    # --------------------------------------------------------------- 空跑

    def _simulated_registers(self) -> list[int]:
        """把模拟的设备状态编码进三个状态寄存器。

        它是 :meth:`decode_status` 的逆操作，使用相同的字节与位位置，因此 dry run
        不会偏离真实的寄存器布局。

        返回:
            由 16 位寄存器值组成的三元素列表。

        示例:
            >>> g = ModbusGripper("fake", dry_run=True); g.activate(timeout=0.05)
            >>> (g._simulated_registers()[0] >> 12) & 0b11      # gSTA 字段
            3
        """
        status = 0
        if self._sim_activated:
            status |= 0b1                       # gACT
            status |= self.profile.activated_status << 4   # gSTA
        if self._sim_goto:
            status |= 0b1 << 3                  # gGTO
        status |= (self._sim_gobj & 0b11) << 6  # gOBJ
        units = int(round(self._sim_position * self.profile.position_scale)) & 0xFF
        return [status << 8, (self._sim_gflt & 0x0F) << 8, units << 8]

    def _apply_simulated_write(self, address: int, values: Sequence[int]) -> None:
        """按设备固件的方式解释一次寄存器写入。

        解码*编码后*的请求字（而不是相信 Python 层的意图），正是让模拟有意义的关键：
        如果 :meth:`go_to` 把速度与力度字节打包反了，dry run 显示出的位置也会跟着错。

        参数:
            address: 被写入的寄存器地址。
            values: 被写入的寄存器值。

        返回:
            ``None``。

        示例:
            不直接调用；参见 :meth:`_write_registers`。
        """
        if address != self.profile.write_base:
            logger.debug(
                "gripper dry run: ignoring a write to %#x (the profile's request block "
                "starts at %d)",
                address,
                self.profile.write_base,
            )
            return
        padded = list(values) + [0] * (3 - len(values))
        request = padded[0] & 0xFFFF
        if request == 0 and not any(padded[1:]):
            self._sim_activated = False
            self._sim_goto = False
            self._sim_gobj = 0
            self._sim_position = 0.0
            return
        if request & _GOTO_BIT:
            units = padded[1] & 0xFF
            self._sim_position = min(1.0, units / float(self.profile.position_scale))
            self._sim_gobj = 3                  # 移动立即完成
            self._sim_goto = True
        elif request & _ACTIVATE_BIT:
            self._sim_activated = True
            self._sim_goto = False
            self._sim_gobj = 3
        else:
            logger.warning(
                "gripper dry run: request word %#06x sets neither gACT nor gGTO; "
                "the device would ignore it",
                request,
            )

    # ------------------------------------------------------------- 状态字

    @staticmethod
    def decode_status(registers: Sequence[int], *, profile: Any = ROBOTIQ_2F) -> dict[str, Any]:
        """把三个状态寄存器解码为具名位域。

        每个寄存器只有**高字节**承载数据，这是厂商约定，也是 ``>> 8`` 移位的原因；
        转而读取低字节会得到一串零，看起来就像一个完全合法的"未激活、无故障、完全
        张开"状态。

        参数:
            registers: 至少三个 16 位寄存器值，读取自 ``profile.read_base``。
            profile: 为派生字段 ``position`` 提供 ``position_scale`` 的 profile。

        返回:
            一个 dict，含 ``gACT``、``gGTO``、``gSTA``、``gOBJ``、``gFLT``、``gPO``
            （原始 0-255 位置）以及 ``position``（``gPO / position_scale``）。

        异常:
            TypeError: 若 ``registers`` 不是整数序列。
            ValueError: 若给出的寄存器少于三个。

        示例:
            >>> ModbusGripper.decode_status([0xD0 << 8, 0x00 << 8, 0x80 << 8])["gOBJ"]
            3
            >>> # gACT（第 0 位）+ gOBJ=2（第 6-7 位）-> “已夹持物体”，故障 0xE
            >>> ModbusGripper.decode_status([0b1000_0001 << 8, 0x0E << 8, 0x40 << 8])["gOBJ"]
            2
        """
        if isinstance(registers, (str, bytes)) or not isinstance(registers, Sequence):
            raise TypeError(
                f"registers must be a sequence of ints, got {type(registers).__name__}: "
                f"{registers!r}"
            )
        if len(registers) < 3:
            raise ValueError(
                f"the status block needs 3 registers, got {len(registers)}: {list(registers)!r}"
            )
        scale = getattr(profile, "position_scale", ROBOTIQ_2F.position_scale)
        if isinstance(scale, bool) or not isinstance(scale, int) or scale <= 0:
            raise ValueError(f"profile.position_scale must be a positive int, got {scale!r}")
        words = [int(v) & 0xFFFF for v in registers[:3]]
        gripper_status = (words[0] >> 8) & 0xFF
        fault_status = (words[1] >> 8) & 0xFF
        position_units = (words[2] >> 8) & 0xFF
        return {
            "gACT": gripper_status & 0b1,
            "gGTO": (gripper_status >> 3) & 0b1,
            "gSTA": (gripper_status >> 4) & 0b11,
            "gOBJ": (gripper_status >> 6) & 0b11,
            "gFLT": fault_status & 0x0F,
            "gPO": position_units,
            "position": position_units / float(scale),
        }

    def read_status(self) -> dict[str, Any]:
        """读取并解码夹爪的状态块。

        返回:
            由 :meth:`decode_status` 生成的 dict。

        异常:
            RuntimeError: 若夹爪已关闭。
            minimalmodbus.ModbusException: 总线出错时传播。

        示例:
            >>> g = ModbusGripper("fake", dry_run=True); g.activate(timeout=0.05)
            >>> g.read_status()["gSTA"]
            3
        """
        registers = self._read_registers(self.profile.read_base, 3)
        return self.decode_status(registers, profile=self.profile)

    def _position_of(self, status: Mapping[str, Any]) -> float:
        """返回已解码状态所对应的归一化手指位置。

        存在派生字段 ``position`` 时直接使用，否则从原始 ``gPO`` 字节重新计算，因此
        手工构造的状态 dict（测试夹具，或从日志中回放的一行）也能工作。

        参数:
            status: 一个已解码的状态映射。

        返回:
            以 ``0.0``-``1.0`` 比例表示的位置。

        示例:
            >>> g = ModbusGripper("fake", dry_run=True)
            >>> g._position_of({"gPO": 128})
            0.5019607843137255
        """
        raw = status.get("position")
        if raw is None:
            return int(status["gPO"]) / float(self.profile.position_scale)
        return float(raw)

    def status_text(self, status: Mapping[str, Any] | None = None) -> str:
        """把状态渲染成一行便于阅读的日志。

        数字代码会用 :data:`OBJECT_STATES` / :data:`FAULT_CODES` 加以注释，因为
        ``gOBJ=2`` 与 ``gOBJ=3`` 只差一个字符，对负载而言含义却相反。凌晨三点在
        日志里从一个原始数字去辨认这一区别，正是物体被掉落的方式。

        参数:
            status: 一个已解码的状态。``None`` 表示读取设备，这会消耗一次总线事务。

        返回:
            单行字符串，例如
            ``pos=0.502 gOBJ=2 (close-blocked (object held)) gFLT=0x0 (no fault)
            gSTA=3 gACT=1 gGTO=1``。

        异常:
            TypeError: 若给出了 ``status`` 但不是映射。
            ValueError: 若 ``status`` 缺少必需的键。

        示例:
            >>> g = ModbusGripper("fake", dry_run=True); g.activate(timeout=0.05)
            >>> "gSTA=3" in g.status_text()
            True
        """
        current = self.read_status() if status is None else _require_status(status)
        fault_text = FAULT_CODES.get(int(current["gFLT"]), "unknown")
        object_text = OBJECT_STATES.get(int(current["gOBJ"]), "unknown")
        return (
            f"pos={self._position_of(current):.3f} "
            f"gOBJ={int(current['gOBJ'])} ({object_text}) "
            f"gFLT=0x{int(current['gFLT']):x} ({fault_text}) "
            f"gSTA={int(current['gSTA'])} gACT={int(current['gACT'])} "
            f"gGTO={int(current['gGTO'])}"
        )

    def has_major_fault(self, status: Mapping[str, Any] | None = None) -> bool:
        """夹爪是否报告了禁止继续下指令的故障。

        参数:
            status: 一个已解码的状态，或 ``None`` 表示读取设备。

        返回:
            当 ``gFLT`` 位于 :data:`MAJOR_FAULTS` 中时为 ``True``。

        异常:
            TypeError: 若给出了 ``status`` 但不是映射。
            ValueError: 若 ``status`` 缺少必需的键。

        示例:
            >>> g = ModbusGripper("fake", dry_run=True)
            >>> g.has_major_fault({"gACT": 1, "gGTO": 0, "gSTA": 3, "gOBJ": 2,
            ...                    "gFLT": 0xE, "gPO": 100})
            True
        """
        current = self.read_status() if status is None else _require_status(status)
        return int(current["gFLT"]) in MAJOR_FAULTS

    # ---------------------------------------------------------------- 命令

    def reset(self) -> None:
        """清空动作请求块，使夹爪失活。

        写入全零是厂商的复位序列。它会松开手指，因此绝不能在被抓着负载时调用——
        这正是 :meth:`squeeze_more` 在重大故障下干脆什么都不做、而不是通过复位去
        "恢复"的原因。

        返回:
            ``None``。

        异常:
            RuntimeError: 若夹爪已关闭。
            minimalmodbus.ModbusException: 总线出错时传播。

        示例:
            >>> ModbusGripper("fake", dry_run=True).reset()
        """
        self._write_registers(self.profile.write_base, [0, 0, 0])
        if self.dry_run:
            self._record(op="reset")
        logger.debug("gripper reset on %s", self.port)

    def activate(self, timeout: float = 10.0, poll_interval: float = 0.1) -> None:
        """激活夹爪，并阻塞直到它报告"就绪"。

        激活是异步的：请求字会立即返回，而驱动会运行自己的归位序列，在该序列完成
        之前下达的移动指令会被忽略，或从一个未知的参考位置执行。因此会轮询状态，
        直到 ``gSTA`` 等于 ``profile.activated_status``。

        循环至少轮询一次，因此 ``timeout=0`` 的含义是"检查当前状态，不等待"，而不是
        "永不检查"。

        参数:
            timeout: 最长的总等待时间，单位为秒。
            poll_interval: 两次状态读取之间的休眠时间。保持远大于串口超时，以免一个
                慢响应被放大成对 RS-485 总线的忙轮询（该总线是半双工的：把它淹掉会
                让每一次事务都更慢）。

        返回:
            夹爪激活完成后返回 ``None``。

        异常:
            TypeError: 若某个时长不是数字。
            ValueError: 若 ``timeout`` 为负，或 ``poll_interval`` 不为正。
            TimeoutError: 若先到达截止时间。消息中会包含最后一次解码的状态，用以
                区分"从未激活"（``gSTA=0``，通常是接线或从站 id 问题）与"卡在激活
                中"（``gSTA=1``，通常是欠压或手指被挡住）。
            RuntimeError: 若夹爪已关闭。

        示例:
            >>> g = ModbusGripper("fake", dry_run=True)
            >>> g.activate(timeout=0.05); g.read_status()["gSTA"]
            3
        """
        wait = _positive_seconds(timeout, what="timeout", allow_zero=True)
        step = _positive_seconds(poll_interval, what="poll_interval")

        self._write_registers(self.profile.write_base, [self.profile.activate_request, 0, 0])
        if self.dry_run:
            self._record(op="activate", request=self.profile.activate_request)

        deadline = time.monotonic() + wait
        last: dict[str, Any] | None = None
        while True:
            last = self.read_status()
            if int(last["gSTA"]) == int(self.profile.activated_status):
                logger.info("gripper activated on %s (%s)", self.port, self.status_text(last))
                return
            if time.monotonic() >= deadline:
                break
            time.sleep(min(step, max(0.0, deadline - time.monotonic())))
        raise TimeoutError(
            f"gripper activation on {self.port!r} did not reach gSTA="
            f"{self.profile.activated_status} within {wait:.2f}s; "
            f"last status: {self.status_text(last)}"
        )

    def go_to(self, target: float, *, force: int | None = None, speed: int | None = None) -> None:
        """指令一个手指位置。

        移动会立即开始，且*不*等待；请使用 :meth:`wait_motion_complete` 或
        :meth:`is_object_held` 来观察结果。在命令内部等待会让该方法无法用于那种
        在手指运动期间还要做别的事情的控制循环。

        参数:
            target: 归一化位置，``0.0`` 为完全张开，``1.0`` 为完全闭合。
            force: 覆盖本次移动的 ``profile.force``（``0-255``）。
            speed: 覆盖本次移动的 ``profile.speed``（``0-255``）。

        返回:
            ``None``。

        异常:
            TypeError: 若 ``target``/``force``/``speed`` 类型错误。
            ValueError: 若 ``target`` 超出 ``[0.0, 1.0]``，或某个字节字段超出
                ``0-255``。超范围的目标会被拒绝而非截断：截断会掩盖单位错误
                （用了角度而非比例，或用了毫米），否则它会以"夹爪总是完全闭合"
                的形式显现出来。
            RuntimeError: 若夹爪已关闭。

        示例:
            >>> g = ModbusGripper("fake", dry_run=True); g.activate(timeout=0.05)
            >>> g.go_to(0.75); g.read_status()["gPO"]          # round(0.75 * 255)
            191
            >>> g.go_to(1.5)
            Traceback (most recent call last):
                ...
            ValueError: target must be within 0.0-1.0 (0.0 = fully open), got 1.5
        """
        ratio = _unit_interval(target, what="target")
        force_byte = _byte(self.profile.force if force is None else force, what="force")
        speed_byte = _byte(self.profile.speed if speed is None else speed, what="speed")
        units = max(0, min(int(self.profile.position_scale), int(round(ratio * self.profile.position_scale))))
        # 第三个寄存器：速度在高字节，力度在低字节。
        self._write_registers(
            self.profile.write_base,
            [self.profile.goto_request, units, speed_byte * 256 + force_byte],
        )
        if self.dry_run:
            self._record(
                op="go_to",
                target=ratio,
                position_units=units,
                force=force_byte,
                speed=speed_byte,
            )
        logger.debug(
            "gripper go_to target=%.3f (units=%d, force=%d, speed=%d) on %s",
            ratio,
            units,
            force_byte,
            speed_byte,
            self.port,
        )

    def open(self) -> None:
        """指令一次完全张开。

        返回:
            ``None``。

        异常:
            RuntimeError: 若夹爪已关闭。
            minimalmodbus.ModbusException: 总线出错时传播。

        示例:
            >>> g = ModbusGripper("fake", dry_run=True); g.activate(timeout=0.05)
            >>> g.open(); g.read_status()["gPO"]
            0
        """
        self.go_to(0.0)

    def close(self, target: float = 1.0) -> None:
        """指令一次闭合移动。

        参数:
            target: 要闭合到的归一化位置。略低于 ``1.0``（例如 ``0.9``）往往更好：
                无论哪种方式驱动都会持续对物体施力，而留出余量意味着
                :meth:`squeeze_more` 仍能进一步收紧。

        返回:
            ``None``。

        异常:
            TypeError: 若 ``target`` 不是实数。
            ValueError: 若 ``target`` 超出 ``[0.0, 1.0]``。
            RuntimeError: 若夹爪已关闭。

        示例:
            >>> g = ModbusGripper("fake", dry_run=True); g.activate(timeout=0.05)
            >>> g.close(0.9); g.read_status()["position"]
            0.9019607843137255
        """
        self.go_to(target)

    def wait_motion_complete(self, timeout: float = 1.0, poll_interval: float = 0.05) -> bool:
        """阻塞直到手指停止移动。

        "已停止"就是 ``gOBJ != 0``——为何使用 ``gOBJ`` 字段而非 ``gGTO`` 位，参见
        :func:`_motion_in_progress`。注意停止并不等于*成功*：之后 ``gOBJ`` 为 1、2
        或 3，调用方必须看它究竟是哪一个。

        参数:
            timeout: 最长等待时间，单位为秒。``0`` 表示只读取一次。
            poll_interval: 两次状态读取之间的休眠时间。

        返回:
            若运动在截止时间前结束则为 ``True``，否则为 ``False``（会以警告记录，
            因为一次永不完成的移动通常意味着夹爪未激活，或总线正在丢包）。

        异常:
            TypeError: 若某个时长不是数字。
            ValueError: 若 ``timeout`` 为负，或 ``poll_interval`` 不为正。
            RuntimeError: 若夹爪已关闭。

        示例:
            >>> g = ModbusGripper("fake", dry_run=True); g.activate(timeout=0.05)
            >>> g.go_to(0.5); g.wait_motion_complete(timeout=0.05)
            True
        """
        wait = _positive_seconds(timeout, what="timeout", allow_zero=True)
        step = _positive_seconds(poll_interval, what="poll_interval")

        deadline = time.monotonic() + wait
        while True:
            status = self.read_status()
            if not _motion_in_progress(status):
                return True
            if time.monotonic() >= deadline:
                break
            time.sleep(min(step, max(0.0, deadline - time.monotonic())))
        logger.warning(
            "gripper on %s was still moving after %.2fs (%s)",
            self.port,
            wait,
            self.status_text(status),
        )
        return False

    def squeeze_more(
        self,
        target: float = 1.0,
        *,
        force: int | None = None,
        wait_s: float = 1.0,
        step: float = 0.02,
        poll_interval: float = 0.05,
    ) -> dict[str, Any]:
        """收紧已有的抓取——绝不放松。

        这是 :meth:`is_object_held` 判定夹持力度勉强之后的重新尝试路径，它所遵守的
        约束才是关键所在::

            tighter = min(1.0, max(target, current_position + step))

        因此指令位置绝不会*低于*测得位置。抓取成功后手指正以恰好被赋予的力抓着负载，
        所以任何更宽的指令——哪怕只是略微张开的"重新放置"——都会松开物体。只能收紧的
        重试可以无条件安全调用；能放松的重试则必须由容易写错的逻辑加以守卫，而失败
        模式就是掉落负载。

        重大故障会短路该方法，并原样返回状态。在过流、欠压或自动释放进行中时，驱动
        要么已经松手、要么即将松手，而那种直觉式的"恢复"（复位、重新激活、重新
        张开）恰恰是保证丢失负载的操作序列。停止并上报是唯一安全的做法；这个决定
        属于人，或属于更高层的恢复状态机，而不是一个抓取重试辅助函数。

        参数:
            target: 期望的归一化位置。低于当前位置的值会被提升到 ``current + step``，
                绝不会向下跟随。
            force: 覆盖本次移动的 ``profile.force``。
            wait_s: 等待收紧移动完成的时间。
            step: 施加的最小额外闭合量，以归一化单位计。在 2F-85 上 ``0.02`` 约为
                半毫米——足以消除间隙，又小到不会捏烂一颗熟番茄。
            poll_interval: 等待期间两次状态读取之间的休眠时间。

        返回:
            移动*之后*读取的状态（或在重大故障使其停止时原样返回的状态）。抓取现在
            是否良好由调用方决定；本方法刻意不抛出异常，因为"仍未抓住"是调用方必须
            据以分支的正常结果。

        异常:
            TypeError: 若某个参数类型错误。
            ValueError: 若 ``target`` 超出 ``[0.0, 1.0]``，``step`` 超出
                ``(0.0, 1.0]``，或某个时长无效。
            RuntimeError: 若夹爪已关闭。

        示例:
            >>> g = ModbusGripper("fake", dry_run=True); g.activate(timeout=0.05)
            >>> g.go_to(0.50)
            >>> after = g.squeeze_more(0.40, wait_s=0.05)   # 请求的宽度比 0.50 *更大*
            >>> after["position"] >= 0.50                    # ... 而且仍然会收紧
            True
        """
        ratio = _unit_interval(target, what="target")
        increment = _unit_interval(step, what="step")
        if increment <= 0.0:
            raise ValueError(f"step must be > 0 (it is the minimum extra closing), got {step!r}")
        wait = _positive_seconds(wait_s, what="wait_s", allow_zero=True)
        step_seconds = _positive_seconds(poll_interval, what="poll_interval")

        status = self.read_status()
        if self.has_major_fault(status):
            logger.warning(
                "gripper on %s reports a major fault (%s); not commanding any motion -- "
                "resetting or opening under this fault would release the payload",
                self.port,
                self.status_text(status),
            )
            return status

        current = self._position_of(status)
        tighter = min(1.0, max(ratio, current + increment))
        if tighter < current:  # pragma: no cover - 不可达，保留作为断言
            raise AssertionError(
                f"squeeze_more would widen the grip from {current:.3f} to {tighter:.3f}"
            )
        logger.info(
            "gripper squeeze_more on %s: current=%.3f requested=%.3f commanded=%.3f",
            self.port,
            current,
            ratio,
            tighter,
        )
        self.go_to(tighter, force=force)
        self.wait_motion_complete(timeout=wait, poll_interval=step_seconds)
        return self.read_status()

    def is_object_held(
        self,
        status: Mapping[str, Any] | None = None,
        *,
        target: float = 0.85,
        position_slack: float = 0.03,
    ) -> bool:
        """判定手指是否真的抓着东西。

        组合了两个彼此独立的信号，任一单独都不充分：

        * ``gOBJ == 2``（"闭合受阻"）是设备自身的答案，但在某些固件版本上它会比机械
          停止晚一个状态周期，或对非常薄的物体根本不置位，因此仅信任它会漏掉那些
          物理上其实良好的抓取。
        * 位置测试——手指明显停在指令目标之前，因为中间有东西——对任何固件都有效，
          但当指令目标本就与测得位置接近时会误判（一个几乎闭合的夹爪无论有没有抓着
          东西，看起来都像"已抓住"）。

        要求*任一*信号成立可让漏报率保持较低（即便固件从不置位 ``gOBJ``，真实的抓取
        也能被检测到），而松弛阈值则让误报率保持较低。比较是**有方向的**
        （``measured < commanded - slack``）：手指在物理上不可能越过闭合指令，而使用
        绝对差值会在运动途中*下压*目标时把结果报告为已抓住物体。

        参数:
            status: 一个已解码的状态，或 ``None`` 表示读取设备。
            target: 本次抓取所指令的位置，以归一化单位计。它必须是传给 :meth:`go_to`
                / :meth:`close` 的值，否则位置测试会与一个从未下达过的指令相比较。
                ``0.85`` 的默认值与典型的"稳固闭合"指令相符。
            position_slack: 距 ``target`` 还差多少（以归一化单位计）仍算作"达到了
                它"。必须 ``>= 0``。

        返回:
            当认为正抓着物体时为 ``True``。

        异常:
            TypeError: 若给出了 ``status`` 但不是映射。
            ValueError: 若 ``status`` 缺少必需的键，``target`` 超出 ``[0.0, 1.0]``，
                或 ``position_slack`` 为负。

        示例:
            >>> g = ModbusGripper("fake", dry_run=True); g.activate(timeout=0.05)
            >>> g.go_to(0.9)
            >>> g.is_object_held(target=0.9)      # 已到达目标：未夹持任何物体
            False
            >>> g.is_object_held(target=0.9, status={**g.read_status(), "gOBJ": 2})
            True
        """
        commanded = _unit_interval(target, what="target")
        if isinstance(position_slack, bool) or not isinstance(position_slack, (int, float)):
            raise TypeError(
                f"position_slack must be a number, got {type(position_slack).__name__}: "
                f"{position_slack!r}"
            )
        if position_slack < 0:
            raise ValueError(f"position_slack must be >= 0, got {position_slack!r}")

        current = self.read_status() if status is None else _require_status(status)
        object_state = int(current["gOBJ"])
        measured = self._position_of(current)
        if object_state == 2:
            logger.debug(
                "gripper on %s holds an object: gOBJ=2 (close-blocked) at pos=%.3f",
                self.port,
                measured,
            )
            return True
        if object_state == 1:
            # 在张开时停下：物体太大抓不住。此处报告"已抓住"会让手臂空着手指去
            # 抬升。
            logger.debug(
                "gripper on %s is open-blocked (gOBJ=1) at pos=%.3f; nothing is held",
                self.port,
                measured,
            )
            return False
        held = measured < commanded - float(position_slack)
        logger.debug(
            "gripper on %s position test: measured=%.3f commanded=%.3f slack=%.3f -> %s",
            self.port,
            measured,
            commanded,
            position_slack,
            held,
        )
        return held

    # ---------------------------------------------------------------- 生命周期

    def close_port(self) -> None:
        """释放串口。

        具备幂等性与异常安全性，因此可以从 ``finally`` 块、信号处理器或
        :meth:`__exit__` 中调用，而不会掩盖正在处理的错误。调用之后，每个命令方法
        都会抛出 :class:`RuntimeError`，而不是默默什么都不做：以为自己释放了抓取的
        调用方，必须被告知其实没有。

        返回:
            ``None``。

        示例:
            >>> g = ModbusGripper("fake", dry_run=True); g.close_port(); g.close_port()
        """
        if self._closed:
            return
        self._closed = True
        handle, self._serial, self._instrument = self._serial, None, None
        if handle is not None:
            try:
                handle.close()
            except Exception as exc:  # noqa: BLE001 - 关机流程绝不能抛出异常
                logger.debug("closing the gripper port %s failed: %s", self.port, exc)
        logger.debug("gripper port %s closed", self.port)

    def __enter__(self) -> ModbusGripper:
        """返回 ``self``，以供 ``with`` 块使用。

        返回:
            已打开的夹爪。

        示例:
            >>> with ModbusGripper("fake", dry_run=True) as g:
            ...     _ = g.connected
        """
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        """关闭串口，绝不抑制异常。

        参数:
            exc_type: 异常类型，或 ``None``。
            exc: 异常实例，或 ``None``。
            tb: 回溯对象，或 ``None``。

        返回:
            ``None``（恒为假值，因此异常会继续传播）。

        示例:
            参见 :meth:`__enter__`。
        """
        self.close_port()

    def __repr__(self) -> str:
        """返回用于调试的表示。

        返回:
            一个包含串口、从站 id 与连接状态的字符串。之所以包含串口，是因为双臂
            机器人会记录两个否则完全相同的实例。

        示例:
            >>> repr(ModbusGripper("fake", dry_run=True))
            "ModbusGripper(port='fake', slave_id=9, dry_run=True, connected=True)"
        """
        return (
            f"ModbusGripper(port={self.port!r}, slave_id={self.profile.slave_id}, "
            f"dry_run={self.dry_run}, connected={self.connected})"
        )
