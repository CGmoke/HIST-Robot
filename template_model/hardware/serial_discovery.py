"""将 USB 串口设备探测并解析为稳定的逻辑名称。

一台机器人若带有两个完全相同的 USB 转 RS-485 适配器（每个夹爪一个），就会
遇到 ``/dev/ttyUSB*`` 无法解决的命名问题。内核按照*枚举顺序*分配这些编号，
而枚举顺序取决于上电时序、集线器拓扑以及 USB 控制器恰好先轮询哪个适配器。
重启机器，或拔插其中一个适配器，"left" 与 "right" 就会互换。此后所有下游
症状看起来都像控制 bug 而非命名 bug：机器人用错手臂去抓取，而日志却毫无
异常。

udev 通过在 ``/dev/serial/by-id/`` 下发布符号链接来解决这个问题，链接名中
嵌入了适配器的 USB 厂商、产品以及**序列号**。有两点使它们成为理想的构建
基础：

* 名称源自设备本身，而非枚举顺序，因此能经受重启与拔插；
* **对这些名称排序即可得到稳定的左右分配**，即便完全不知道哪个适配器是
  哪个。序列号对每个适配器固定，因此 by-id 路径的字典序在每次启动时都相同
  ——排序后的第一项始终在物理上对应同一个适配器。而直接对
  ``/dev/ttyUSB*`` 通配符排序没有这种保证，因为它排序的是内核本次启动时
  任意分配的数字。

因此本模块同时提供两半能力：底层的列举与探测（:func:`list_by_id_ports`、
:func:`list_ports`、:func:`find_by_serial`、:func:`probe_port`），以及一个
四层解析器（:func:`resolve_ports`），它把 ``{"left": ..., "right": ...}``
转换为具体的设备路径。

解析器不会因解析失败而抛出异常——只有 ``spec`` 格式错误时才会。找不到某个
夹爪的机器人应当能启动、大声记录日志，并使用它*确实*拥有的硬件继续运行；
因为一根 USB 线松动就中止整个启动流程，会把一台降级运行的机器人变成一台
彻底死掉的机器人。每一个决策（以及每一次拒绝及其所属层级）都会被记录，因为
在现场调试时那份日志是唯一的证据。

PySerial 采用惰性导入：列举 ``/dev/serial/by-id`` 只需 :mod:`glob`，因此本
模块的大部分功能在没有安装 PySerial 的机器上也能工作。

示例
-------
::

    from reusable_model.hardware.serial_discovery import resolve_ports

    def is_gripper(port: str) -> bool:
        from reusable_model.hardware.modbus_gripper import ModbusGripper
        with ModbusGripper(port) as gripper:
            gripper.read_status()
        return True

    ports = resolve_ports(
        {"left": "", "right": ""},          # 未显式指定路径：自动发现它们
        serial_map={"left": "DAANTRS9", "right": "DAANTMYY"},
        probe_fn=is_gripper,                # 只占用能响应 Modbus 的端口
        prefer=("left", "right"),           # left = 排序后第一个适配器
    )
"""

from __future__ import annotations

import glob
import json
import logging
import os
import threading
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence

import yaml

logger = logging.getLogger(__name__)

__all__ = [
    "DEFAULT_BY_ID_DIR",
    "list_by_id_ports",
    "list_ports",
    "find_by_serial",
    "probe_port",
    "load_mapping_file",
    "resolve_ports",
]

#: udev 存放设备标识符号链接的目录。这是 Linux 的约定而非项目设置；将其暴露为
#: 参数，是为了能传入非标准的 udev 规则目录（或测试夹具）。
DEFAULT_BY_ID_DIR = "/dev/serial/by-id"

#: 使模式被解释为 glob 而非子串的 glob 元字符。
_GLOB_CHARS = "*?["

#: :func:`load_mapping_file` 可接受的文件扩展名。
_MAPPING_SUFFIXES = (".yaml", ".yml", ".json")

_PYSERIAL_HINT = "pip install pyserial"


def _as_path(value: Any, *, what: str) -> Path:
    """将 ``str`` / :class:`os.PathLike` 强制转换为 :class:`pathlib.Path`。

    参数:
        value: 候选路径。
        what: 出现在错误消息中的参数名。

    返回:
        转换为 ``Path`` 后的该值。

    异常:
        TypeError: 若 ``value`` 不是字符串或类路径对象。

    示例:
        >>> _as_path("/dev/ttyUSB0", what="port").name
        'ttyUSB0'
    """
    if isinstance(value, Path):
        return value
    if isinstance(value, (str, os.PathLike)):
        return Path(os.fspath(value))
    raise TypeError(f"{what} must be a str or os.PathLike, got {type(value).__name__}: {value!r}")


def _require_port(port: Any, *, what: str = "port") -> str:
    """校验一个串口设备路径。

    参数:
        port: 候选设备路径。
        what: 出现在错误消息中的参数名。

    返回:
        去除首尾空白后的路径（``str``）。

    异常:
        TypeError: 若 ``port`` 不是字符串或类路径对象。
        ValueError: 若其为空白。

    示例:
        >>> _require_port("/dev/ttyUSB0")
        '/dev/ttyUSB0'
    """
    text = os.fspath(port) if isinstance(port, os.PathLike) else port
    if not isinstance(text, str):
        raise TypeError(f"{what} must be a str device path, got {type(port).__name__}: {port!r}")
    text = text.strip()
    if not text:
        raise ValueError(f"{what} must be a non-blank device path, got {port!r}")
    return text


# --------------------------------------------------------------------- 探测


def list_by_id_ports(pattern: str = "*", *, by_id_dir: str | os.PathLike[str] = DEFAULT_BY_ID_DIR) -> list[str]:
    """列出 udev ``by-id`` 串口符号链接，并排序。

    ``pattern`` 在包含 ``*``、``?`` 或 ``[`` 时按 glob 解释，否则按不区分大小写的
    **子串**解释。两种读法在实践中都有用：``"usb-FTDI_USB_TO_RS-485_*"`` 表达的是
    适配器系列，而 ``"FTDI"`` 或 ``"DAANTRS9"`` 则是人问"它插上了吗？"时会输入的
    内容。

    结果是经过排序的，这一点至关重要：by-id 名称源自适配器的 USB 序列号，因此其
    字典序是*硬件*的属性，在每次启动时都相同。于是把 ``candidates[0]`` 分配给
    "left"、``candidates[1]`` 分配给 "right"，在重启与拔插后都可复现。而对
    ``/dev/ttyUSB*`` 通配符排序则没有这种保证——它排序的是内核枚举编号，这些
    编号是 USB 协议栈恰好以某种顺序轮询适配器时任意给出的。

    参数:
        pattern: 施加于文件名上的 glob 或子串过滤器。
        by_id_dir: 要扫描的目录。

    返回:
        排序后的绝对设备路径。当目录不存在（没有 USB 串口适配器的机器是正常
        情况，而非错误）或没有任何匹配时，返回空列表。

    异常:
        TypeError: 若 ``pattern`` 不是字符串，或 ``by_id_dir`` 不是类路径对象。
        ValueError: 若 ``pattern`` 包含路径分隔符（那样会把一个列举辅助函数变成
            目录遍历）。

    示例:
        >>> isinstance(list_by_id_ports(), list)
        True
        >>> list_by_id_ports("FTDI", by_id_dir="/nonexistent-dir-xyz")
        []
    """
    if not isinstance(pattern, str):
        raise TypeError(f"pattern must be a str, got {type(pattern).__name__}: {pattern!r}")
    if os.sep in pattern or "/" in pattern:
        raise ValueError(f"pattern must not contain a path separator, got {pattern!r}")
    base = _as_path(by_id_dir, what="by_id_dir")
    if not base.is_dir():
        logger.debug("list_by_id_ports: %s does not exist; no by-id serial devices", base)
        return []

    if any(char in pattern for char in _GLOB_CHARS):
        matches = glob.glob(str(base / pattern))
    else:
        needle = pattern.strip().lower()
        matches = [str(p) for p in glob.glob(str(base / "*")) if needle in Path(p).name.lower()]
    ports = sorted(m for m in matches if Path(m).exists())
    logger.debug("list_by_id_ports(%r) -> %s", pattern, ports)
    return ports


def _import_list_ports() -> Any:
    """按需导入 ``serial.tools.list_ports``。

    参数:
        无。

    返回:
        ``serial.tools.list_ports`` 模块。

    异常:
        ImportError: 若未安装 PySerial。消息中会给出安装命令，因为机器人上出现
            "No module named serial.tools" 时，绝大多数情况是缺少依赖而非拼写
            错误。

    示例:
        不直接调用；参见 :func:`list_ports`。
    """
    try:
        from serial.tools import list_ports  # noqa: PLC0415 - 可选依赖
    except ImportError as exc:
        raise ImportError(f"listing serial ports needs PySerial ({_PYSERIAL_HINT})") from exc
    return list_ports


def _usb_id(value: Any, *, what: str) -> int | None:
    """校验一个可选的 USB 厂商/产品标识符。

    参数:
        value: ``None``（不做过滤）或 ``0..0xFFFF`` 范围内的整数。
        what: 出现在错误消息中的参数名。

    返回:
        该整数，或 ``None``。

    异常:
        TypeError: 若 ``value`` 既不是 ``None`` 也不是整数。
        ValueError: 若其超出 16 位范围。

    示例:
        >>> _usb_id(0x0403, what="vid")
        1027
    """
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{what} must be an int or None, got {type(value).__name__}: {value!r}")
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"{what} must be within 0x0000-0xFFFF, got {value!r} ({hex(value)})")
    return value


def list_ports(vid: int | None = None, pid: int | None = None) -> list[dict[str, Any]]:
    """以普通 dict 的形式枚举串口，可选地按 USB ID 过滤。

    将 PySerial 的 ``ListPortInfo`` 对象包装成普通 dict 是刻意为之：这些对象只在
    枚举结果存活期间可用，无法被 pickle 到工作进程，而且会迫使调用方为了读取
    ``port.device`` 而导入 PySerial。只保留真正重要的六个字段的 dict，能让应用
    的其余部分免受该依赖的束缚。

    结果按设备名排序。``comports()`` 返回的串口顺序是枚举顺序，它与
    ``/dev/ttyUSB*`` 编号一样在重启后不稳定，因此未排序的结果会让任何"第一个
    串口胜出"的逻辑在多次运行之间反复横跳。

    参数:
        vid: 要求匹配的 USB 厂商 id，``None`` 表示接受任意值。
        pid: 要求匹配的 USB 产品 id，``None`` 表示接受任意值。

    返回:
        一个 dict 列表，键为 ``device``、``serial_number``、``vendor_id``、
        ``product_id``、``description`` 和 ``manufacturer``。缺失的属性报告为
        ``None``。

    异常:
        TypeError: 若 ``vid``/``pid`` 不是整数或 ``None``。
        ValueError: 若某个 id 超出 ``0x0000-0xFFFF``。
        ImportError: 若未安装 PySerial（:func:`pip install pyserial`）。

    示例:
        >>> for port in list_ports():       # doctest: +SKIP
        ...     print(port["device"], port["serial_number"])
    """
    want_vid = _usb_id(vid, what="vid")
    want_pid = _usb_id(pid, what="pid")
    list_ports_module = _import_list_ports()

    found: list[dict[str, Any]] = []
    for info in list_ports_module.comports():
        if want_vid is not None and info.vid != want_vid:
            continue
        if want_pid is not None and info.pid != want_pid:
            continue
        found.append(
            {
                "device": info.device,
                "serial_number": info.serial_number,
                "vendor_id": info.vid,
                "product_id": info.pid,
                "description": info.description,
                "manufacturer": info.manufacturer,
            }
        )
    found.sort(key=lambda item: str(item.get("device") or ""))
    logger.debug("list_ports(vid=%s, pid=%s) -> %d port(s)", vid, pid, len(found))
    return found


def find_by_serial(
    serial_fragment: str,
    *,
    candidates: Iterable[str] | None = None,
    by_id_dir: str | os.PathLike[str] = DEFAULT_BY_ID_DIR,
) -> str | None:
    """把设备序列号（或其中一段）映射到设备路径。

    匹配是针对 *by-id 文件名* 进行的、不区分大小写的子串测试，udev 正是在文件名
    中嵌入序列号。一个片段就足够了——FTDI 序列号的末尾几个字符正是技术人员无需
    拔下任何东西就能从标签上读到的内容——而且大小写无关紧要，因为厂商报告序列号
    时对大小写的处理并不一致。

    若某个值本身已是一个存在的绝对路径，则原样返回。这使得本函数可以安全地作用于
    一个既可能是序列号、也可能是路径的字段，而逐机校准文件往往正是手写成这种
    形式。

    参数:
        serial_fragment: 序列号或其中一段可辨识的片段。
        candidates: 要搜索的设备路径。``None`` 表示以无过滤器方式搜索
            :func:`list_by_id_ports`。
        by_id_dir: 当 ``candidates`` 为 ``None`` 时使用的目录。

    返回:
        第一个匹配的设备路径（按排序顺序，故可复现），无匹配时为 ``None``。

    异常:
        TypeError: 若 ``serial_fragment`` 不是字符串，或 ``candidates`` 中含有
            非字符串项。

    示例:
        >>> find_by_serial("DAANTRS9", candidates=["/dev/serial/by-id/usb-FTDI_X_DAANTRS9-if00-port0"])
        '/dev/serial/by-id/usb-FTDI_X_DAANTRS9-if00-port0'
        >>> find_by_serial("NOPE", candidates=["/dev/serial/by-id/usb-FTDI_X_DAANTRS9-if00-port0"]) is None
        True
    """
    if not isinstance(serial_fragment, str):
        raise TypeError(
            f"serial_fragment must be a str, got {type(serial_fragment).__name__}: {serial_fragment!r}"
        )
    needle = serial_fragment.strip()
    if not needle:
        raise ValueError(f"serial_fragment must be a non-blank string, got {serial_fragment!r}")

    if needle.startswith("/") and Path(needle).exists():
        logger.debug("find_by_serial: %r is already an existing path", needle)
        return needle

    if candidates is None:
        pool = list_by_id_ports("*", by_id_dir=by_id_dir)
    else:
        if isinstance(candidates, (str, bytes)) or not isinstance(candidates, Iterable):
            raise TypeError(
                f"candidates must be an iterable of str or None, got {type(candidates).__name__}: "
                f"{candidates!r}"
            )
        pool = []
        for item in candidates:
            if not isinstance(item, str):
                raise TypeError(f"candidates entries must be str, got {type(item).__name__}: {item!r}")
            pool.append(item)

    upper = needle.upper()
    for port in sorted(pool):
        if upper in Path(port).name.upper():
            logger.debug("find_by_serial: %r matched %s", needle, port)
            return port
    logger.warning(
        "no device matches serial %r; available by-id ports: %s", needle, sorted(pool) or "<none>"
    )
    return None


# ----------------------------------------------------------------------- 探测


def probe_port(port: str, probe_fn: Callable[[str], Any], *, timeout: float = 0.5) -> bool:
    """询问调用方提供的探测函数：某设备是否在 ``port`` 上应答。

    探测函数由调用方提供，因为"这个串口上是不是我的设备"完全取决于协议：Modbus
    夹爪会应答保持寄存器读取，激光雷达会应答启动命令，电机控制器会应答版本查询。
    本辅助函数贡献了每个协议都需要、而每个实现往往都会做错的两件事：

    * **任何异常都意味着"否"**，以 DEBUG 级别记录。在探测过程中，打开属于其它
      设备的串口并收到超时、权限错误或乱码是正常的——甚至是预料之中的。让这样
      一个串口抛出异常会中止整个扫描。
    * **探测在截止时间内运行。** 对无应答者的串口进行读取会阻塞整个串口超时时长，
      而卡死的 USB 设备可能在驱动内部无限期阻塞。因此探测运行在一个守护线程上，
      并以 ``timeout`` 进行 join；若它尚未返回，则拒绝该串口并放弃该线程（Python
      无法杀死线程，而守护线程绝不能拖住进程退出）。代价是每个无望的串口可能泄漏
      一个线程，与永远无法完成的启动相比这很廉价。

    参数:
        port: 要探测的设备路径。
        probe_fn: ``probe_fn(port)``，当预期设备应答时返回真值。串口的打开与关闭
            由它负责。
        timeout: 等待探测返回的秒数。

    返回:
        仅当探测在截止时间内返回真值时才为 ``True``。

    异常:
        TypeError: 若 ``port`` 不是字符串，``probe_fn`` 不可调用，或 ``timeout``
            不是数字。
        ValueError: 若 ``port`` 为空白，或 ``timeout`` 为负。

    示例:
        >>> probe_port("/dev/null", lambda p: True, timeout=0.2)
        True
        >>> probe_port("/dev/null", lambda p: 1 / 0, timeout=0.2)   # exception -> False
        False
        >>> probe_port("/dev/null", lambda p: __import__("time").sleep(5), timeout=0.05)
        False
    """
    device = _require_port(port)
    if not callable(probe_fn):
        raise TypeError(f"probe_fn must be callable, got {type(probe_fn).__name__}: {probe_fn!r}")
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)):
        raise TypeError(f"timeout must be a number, got {type(timeout).__name__}: {timeout!r}")
    if timeout < 0:
        raise ValueError(f"timeout must be >= 0, got {timeout!r}")

    outcome: list[bool] = []

    def run() -> None:
        """执行探测，把每一种失败都转换为 ``False``。"""
        try:
            outcome.append(bool(probe_fn(device)))
        except Exception as exc:  # noqa: BLE001 - 探测必须能挺过死掉的串口
            logger.debug("probe of %s failed (%s: %s)", device, type(exc).__name__, exc)
            outcome.append(False)

    worker = threading.Thread(target=run, name=f"serial-probe-{Path(device).name}", daemon=True)
    worker.start()
    worker.join(float(timeout))
    if worker.is_alive():
        logger.warning(
            "probe of %s did not return within %.2fs; treating it as unavailable "
            "(the probe thread is abandoned)",
            device,
            timeout,
        )
        return False
    result = bool(outcome[0]) if outcome else False
    logger.debug("probe of %s -> %s", device, result)
    return result


# ------------------------------------------------------------------- 映射文件


def load_mapping_file(path: str | os.PathLike[str]) -> dict[str, Any]:
    """加载 ``{逻辑名: {...}}`` 映射文件（YAML 或 JSON）。

    格式由文件扩展名决定，因为映射文件是在描述该机器的人在机器上手工编写的，而两
    种写法在实践中都会出现。无法识别的扩展名会直接报错而不是猜测：YAML 是 JSON 的
    超集，因此"干脆按 YAML 试试"会接受一个误命名为 ``.txt`` 的文件，并默默产生空
    映射。

    参数:
        path: 要读取的文件。

    返回:
        解析后的映射（可能为空）。

    异常:
        TypeError: 若 ``path`` 不是类路径对象。
        ValueError: 若扩展名不在 :data:`_MAPPING_SUFFIXES` 中，若文件无法解析，或
            若其根节点不是映射。
        FileNotFoundError: 若文件不存在。路径是显式给出的，因此其缺失属于失误，
            而非某个可选功能被关闭（后者请向 :func:`resolve_ports` 传入
            ``mapping_file=None``）。

    示例:
        >>> import tempfile, pathlib
        >>> tmp = pathlib.Path(tempfile.mkdtemp()) / "map.yaml"
        >>> _ = tmp.write_text("left:\\n  port: /dev/ttyUSB0\\n  ok: true\\n")
        >>> load_mapping_file(tmp)["left"]["port"]
        '/dev/ttyUSB0'
    """
    target = _as_path(path, what="path").expanduser()
    suffix = target.suffix.lower()
    if suffix not in _MAPPING_SUFFIXES:
        raise ValueError(
            f"unsupported mapping file extension {suffix!r} for {target}; "
            f"accepted: {list(_MAPPING_SUFFIXES)}"
        )
    if not target.is_file():
        raise FileNotFoundError(f"port mapping file not found: {target}")

    try:
        text = target.read_text(encoding="utf-8")
    except OSError as exc:
        raise ValueError(f"cannot read port mapping file {target}: {exc}") from exc

    try:
        if suffix == ".json":
            parsed = json.loads(text) if text.strip() else {}
        else:
            parsed = yaml.safe_load(text)
    except (json.JSONDecodeError, yaml.YAMLError) as exc:
        mark = getattr(exc, "problem_mark", None)
        location = f" at line {mark.line + 1}, column {mark.column + 1}" if mark is not None else ""
        raise ValueError(f"cannot parse port mapping file {target}{location}: {exc}") from exc

    if parsed is None:
        return {}
    if not isinstance(parsed, dict):
        raise ValueError(
            f"port mapping file {target} must hold a mapping of name -> entry, "
            f"got {type(parsed).__name__}: {parsed!r}"
        )
    logger.info("loaded port mapping from %s (%d entr(ies))", target, len(parsed))
    return dict(parsed)


# --------------------------------------------------------------------- 解析器


def _normalize_spec(spec: Any) -> dict[str, str]:
    """把 ``spec`` 参数规范化为 ``{逻辑名: 显式路径}``。

    参数:
        spec: 名称到设备路径的映射（当调用方没有显式值、希望自动探测时，路径可为
            ``None`` 或空白），或一个逻辑名序列。

    返回:
        一个保持插入顺序的 dict；其迭代顺序即解析器的优先级顺序，因此关心哪个夹爪
        赢得第一个适配器的调用方应传入有序映射。

    异常:
        TypeError: 若 ``spec`` 不是映射或字符串序列，或某个映射值不是
            字符串/路径/``None``。
        ValueError: 若某个逻辑名为空白。

    示例:
        >>> _normalize_spec({"left": None, "right": "/dev/ttyUSB1"})
        {'left': '', 'right': '/dev/ttyUSB1'}
        >>> _normalize_spec(["left", "right"])
        {'left': '', 'right': ''}
    """
    out: dict[str, str] = {}
    if isinstance(spec, Mapping):
        for key, value in spec.items():
            if not isinstance(key, str) or not key.strip():
                raise ValueError(f"spec keys must be non-blank strings, got {key!r}")
            if value is None:
                text = ""
            elif isinstance(value, os.PathLike):
                text = os.fspath(value).strip()
            elif isinstance(value, str):
                text = value.strip()
            else:
                raise TypeError(
                    f"spec[{key!r}] must be a str device path or None, got "
                    f"{type(value).__name__}: {value!r}"
                )
            out[key.strip()] = text
        return out
    if isinstance(spec, (str, bytes)) or not isinstance(spec, Iterable):
        raise TypeError(
            f"spec must be a mapping of name -> device path, or a sequence of names; "
            f"got {type(spec).__name__}: {spec!r}"
        )
    for item in spec:
        if not isinstance(item, str) or not item.strip():
            raise ValueError(f"spec entries must be non-blank strings, got {item!r}")
        out[item.strip()] = ""
    return out


def _try_claim(
    name: str,
    port: str,
    *,
    claimed: set[str],
    tier: str,
    probe_fn: Callable[[str], Any] | None,
    probe_timeout: float,
) -> str | None:
    """尝试把 ``port`` 分配给 ``name``，并拒绝重复分配。

    每一种拒绝原因都会以警告形式记录，并同时带上逻辑名与产生该结果的层级。正是这种
    配对让解析器在现场可调试：'left: /dev/serial/by-id/... does not exist (tier
    mapping-file)' 告诉操作员应重新运行校准，而光秃秃的一句 'not found' 什么也说
    不了。

    参数:
        name: 逻辑设备名（例如 ``"left"``）。
        port: 候选设备路径。
        claimed: 已分配给其它逻辑名的路径；成功时会更新。一个物理串口最多只能支撑
            一个名称，因为共享同一串口的两个逻辑设备会同时向同一从站写入，破坏彼此
            的事务。
        tier: 解析层级的可读名称，用于记录日志。
        probe_fn: 可选的设备探测函数；给定时，未通过探测的串口会被拒绝。
        probe_timeout: 传给 :func:`probe_port` 的截止时间。

    返回:
        认领到的设备路径，串口被拒绝时为 ``None``。

    示例:
        >>> seen: set[str] = set()
        >>> _try_claim("left", "/dev/null", claimed=seen, tier="demo",
        ...            probe_fn=None, probe_timeout=0.1)
        '/dev/null'
    """
    if port in claimed:
        logger.warning(
            "port %s is already assigned to another device; refusing to give it to %r (tier %s)",
            port,
            name,
            tier,
        )
        return None
    if not Path(port).exists():
        logger.warning("device path %s for %r does not exist (tier %s)", port, name, tier)
        return None
    if probe_fn is not None and not probe_port(port, probe_fn, timeout=probe_timeout):
        logger.warning("device %s for %r did not answer the probe (tier %s)", port, name, tier)
        return None
    claimed.add(port)
    logger.info("resolved %r -> %s (tier %s)", name, port, tier)
    return port


def _entry_disabled(mapping: Mapping[str, Any], name: str) -> bool:
    """映射文件中的某个条目是否显式地把 ``name`` 标记为不可用。

    与 :func:`_entry_from_mapping` 分开实现，是为了让自动探测层能问同一个问题：
    ``ok: false`` 条目意味着有人查看过这台机器，并记录了该设备缺失或损坏，因此
    在第 4 层给它分配一个碰巧探测到的串口会架空这个标记。

    参数:
        mapping: 已加载的映射文件。
        name: 要查找的逻辑设备名。

    返回:
        当该条目存在且带有 ``ok: false`` 时为 ``True``。

    示例:
        >>> _entry_disabled({"left": {"ok": False}}, "left")
        True
        >>> _entry_disabled({"left": {"port": "/dev/ttyUSB0"}}, "left")
        False
    """
    entry = mapping.get(name)
    return isinstance(entry, Mapping) and entry.get("ok") is False


def _entry_from_mapping(
    mapping: Mapping[str, Any],
    name: str,
    *,
    by_id_dir: str | os.PathLike[str] = DEFAULT_BY_ID_DIR,
) -> tuple[str | None, str]:
    """从一个映射文件条目中提取候选串口与来源标签。

    参数:
        mapping: 已加载的映射文件。
        name: 要查找的逻辑设备名。
        by_id_dir: 当条目给出的是序列号而非路径时，要搜索的目录。

    返回:
        一个 ``(候选串口或 None, 来源标签)`` 元组。不存在的串口也会被返回，以便
        :func:`_try_claim` 能记录确切原因；只有真正缺失/无效的条目才会产生
        ``None``。

    示例:
        >>> _entry_from_mapping({"left": {"port": "/dev/ttyUSB0"}}, "left")
        ('/dev/ttyUSB0', 'mapping-file')
    """
    entry = mapping.get(name)
    if entry is None:
        return None, "mapping-file"
    if not isinstance(entry, Mapping):
        logger.warning(
            "mapping entry for %r must be a mapping of port/serial/ok, got %s: %r",
            name,
            type(entry).__name__,
            entry,
        )
        return None, "mapping-file"
    if _entry_disabled(mapping, name):
        # 显式的 "ok: false" 是一句深思熟虑、由人写下的话，表示该设备在这台机器上
        # 已知损坏或缺失。遵从它（而不是照样去探测，也不是让第 4 层给它一个碰巧
        # 发现的串口）可以让一个有记录的问题适配器不进入结果。
        logger.warning("mapping marks %r as unavailable (ok: false); skipping", name)
        return None, "mapping-file"

    port = str(entry.get("port") or "").strip()
    serial = str(entry.get("serial") or entry.get("ftdi_serial") or "").strip()
    label = "mapping-file"
    if port and Path(port).exists():
        return port, label
    if serial:
        found = find_by_serial(serial, by_id_dir=by_id_dir)
        if found:
            return found, f"{label}/serial={serial}"
        logger.warning("mapping serial %r for %r matched no device", serial, name)
    if port:
        return port, label
    return None, label


def resolve_ports(
    spec: Mapping[str, str | None] | Sequence[str],
    *,
    mapping_file: str | os.PathLike[str] | None = None,
    serial_map: Mapping[str, str] | None = None,
    probe_fn: Callable[[str], Any] | None = None,
    prefer: Sequence[str] | None = None,
    by_id_dir: str | os.PathLike[str] = DEFAULT_BY_ID_DIR,
    candidate_pattern: str = "*",
    probe_timeout: float = 0.5,
) -> dict[str, str]:
    """以四层策略把逻辑设备名解析为物理串口。

    各层按顺序尝试，先成功者胜出。这个顺序是一种具体性排序——最权威的事实来源在
    前，最依赖猜测的在后：

    1. **显式路径** —— ``spec`` 中已有的值。有人特意把它放在那里（命令行覆盖、逐机
       unit 文件），因此它优先于任何文件或启发式。
    2. **映射文件** —— ``{name: {port, serial, ok}}``，通常由一次性校准运行生成并
       随机器参数一起保存。``ok: false`` 的条目会被跳过而不探测，*并且*也被排除在
       第 4 层之外，因为该标记是一句人写的声明，表示该设备在这台机器上已知缺失。
       条目可以给出序列号而非路径，这样即使适配器被移到另一个 USB 接口也能存活。
    3. **序列号** —— ``serial_map``，形如 ``{name: fragment}``，通过
       :func:`find_by_serial` 解析。这与第 2 层是同样的信息，但以代码方式提供；当
       序列号是构建产物的属性而非机器的属性时，代码正是它该待的地方。
    4. **自动探测** —— 按排序顺序遍历每个未被认领的 by-id 串口，按 ``prefer`` 给定
       的顺序分配（回退到 ``spec`` 顺序）。排序后的 by-id 名称在重启后是稳定的，
       因此即便这一层完全不知道哪个适配器是哪个，每次也能得到相同的左右分配。给定
       ``probe_fn`` 时，只考虑设备应答的串口；不给定时，分配纯粹是位置性的，由调用
       方承担认领到无关适配器的风险。

    一个 ``claimed`` 集合横跨全部四层，因此一个物理串口永远不会支撑两个逻辑名——
    两个驱动向同一从站写入会破坏彼此的 Modbus 事务。

    **解析失败是警告，绝不是异常。** 缺少一个夹爪应当让机器人降级运行，而不是阻止
    它启动：调用方只会拿回已解析的名称，并自行决定怎么办。唯一例外是 ``spec`` 格式
    错误，那属于编程错误，必须大声报错。

    参数:
        spec: 逻辑名到设备路径的映射（未知时为空白/``None``），或一个纯逻辑名序列。
            迭代顺序即优先级顺序。
        mapping_file: 可选的逐机校准文件；参见 :func:`load_mapping_file`。
        serial_map: 可选的 ``{name: serial fragment}`` 表。
        probe_fn: 可选的 ``probe_fn(port) -> bool``，用于在认领前确认设备。
        prefer: 各名称接收自动探测候选串口的顺序。不在 ``spec`` 中的名称会被忽略
            （并警告）；不在 ``prefer`` 中的名称保持其 ``spec`` 顺序并最后服务。
        by_id_dir: 第 3、4 层要扫描的目录。
        candidate_pattern: 施加于第 4 层扫描的 glob/子串过滤器，例如
            ``"usb-FTDI_*"`` 只考虑某个适配器系列。
        probe_timeout: 传给 :func:`probe_port` 的每串口截止时间。

    返回:
        ``{逻辑名: 设备路径}``，只包含已解析的名称。

    异常:
        TypeError: 若 ``spec`` 不是映射/字符串序列，若 ``serial_map``/``prefer``
            类型错误，或若给出了 ``probe_fn`` 但不可调用。
        ValueError: 若某个逻辑名为空白，或 ``probe_timeout`` 为负。
        FileNotFoundError: 当显式指定的映射文件不存在时，从
            :func:`load_mapping_file` 传播而来。

    示例:
        >>> resolve_ports({"left": "/nonexistent-port"})   # 仅告警，绝不抛异常
        {}
        >>> resolve_ports(["left"], by_id_dir="/nonexistent-dir-xyz")
        {}
    """
    wanted = _normalize_spec(spec)
    if probe_fn is not None and not callable(probe_fn):
        raise TypeError(f"probe_fn must be callable or None, got {type(probe_fn).__name__}: {probe_fn!r}")
    if serial_map is not None and not isinstance(serial_map, Mapping):
        raise TypeError(
            f"serial_map must be a mapping of name -> serial fragment or None, got "
            f"{type(serial_map).__name__}: {serial_map!r}"
        )
    if isinstance(probe_timeout, bool) or not isinstance(probe_timeout, (int, float)):
        raise TypeError(
            f"probe_timeout must be a number, got {type(probe_timeout).__name__}: {probe_timeout!r}"
        )
    if probe_timeout < 0:
        raise ValueError(f"probe_timeout must be >= 0, got {probe_timeout!r}")

    order: list[str] = list(wanted)
    if prefer is not None:
        if isinstance(prefer, (str, bytes)) or not isinstance(prefer, Iterable):
            raise TypeError(f"prefer must be a sequence of str or None, got {prefer!r}")
        prioritised: list[str] = []
        for item in prefer:
            if not isinstance(item, str) or not item.strip():
                raise ValueError(f"prefer entries must be non-blank strings, got {item!r}")
            key = item.strip()
            if key not in wanted:
                logger.warning("prefer names %r, which is not in spec %s; ignoring it", key, order)
                continue
            if key not in prioritised:
                prioritised.append(key)
        order = prioritised + [name for name in wanted if name not in prioritised]

    out: dict[str, str] = {}
    claimed: set[str] = set()
    mapping: dict[str, Any] = {} if mapping_file is None else load_mapping_file(mapping_file)

    def claim(name: str, port: str | None, tier: str) -> bool:
        """尝试把 ``port`` 分配给 ``name`` 并记录结果。

        参数:
            name: 逻辑设备名。
            port: 候选路径，该层无所获时为 ``None``。
            tier: 用于记录日志的层级标签。

        返回:
            当该名称现已解析时为 ``True``。

        示例:
            不直接调用；参见 :func:`resolve_ports`。
        """
        if port is None:
            return False
        got = _try_claim(
            name, port, claimed=claimed, tier=tier, probe_fn=probe_fn, probe_timeout=probe_timeout
        )
        if got is None:
            return False
        out[name] = got
        return True

    # 第 1 层：显式路径优先于其它一切。
    for name in order:
        explicit = wanted.get(name, "")
        if explicit:
            claim(name, explicit, "explicit-path")

    # 第 2 层：逐机映射文件。
    if mapping:
        for name in order:
            if name in out:
                continue
            port, label = _entry_from_mapping(mapping, name, by_id_dir=by_id_dir)
            claim(name, port, label)

    # 第 3 层：以代码方式提供的序列号。
    if serial_map:
        for name in order:
            if name in out:
                continue
            fragment = serial_map.get(name)
            if fragment is None:
                continue
            if not isinstance(fragment, str) or not fragment.strip():
                logger.warning(
                    "serial_map entry for %r must be a non-blank str, got %r; skipping",
                    name,
                    fragment,
                )
                continue
            claim(
                name,
                find_by_serial(fragment, by_id_dir=by_id_dir),
                f"serial={fragment}",
            )

    # 第 4 层：对剩余项按排序自动探测。被映射文件显式标记为不可用的名称会排除
    # 在外："ok: false" 是一句人写的声明，表示该设备已知缺失，在此处碰巧重新发现
    # 它会默默推翻那个决定。
    disabled = sorted(
        name for name in order if name not in out and mapping and _entry_disabled(mapping, name)
    )
    if disabled:
        logger.info(
            "skipping auto-probe for %s: the mapping file marks them unavailable (ok: false)",
            disabled,
        )
    pending = [name for name in order if name not in out and name not in disabled]
    if pending:
        pool = [
            port
            for port in list_by_id_ports(candidate_pattern, by_id_dir=by_id_dir)
            if port not in claimed
        ]
        if pool and probe_fn is not None:
            responding = [p for p in pool if probe_port(p, probe_fn, timeout=probe_timeout)]
            logger.debug(
                "auto-probe: %d of %d candidate port(s) answered: %s",
                len(responding),
                len(pool),
                responding,
            )
            pool = responding
        elif pool:
            logger.warning(
                "no probe_fn given; assigning %d candidate port(s) positionally by sorted "
                "by-id name (%s), which cannot verify the device type",
                len(pool),
                pool,
            )
        if not pool:
            logger.warning(
                "no serial device could be resolved for %s (by-id dir %s, pattern %r)",
                pending,
                by_id_dir,
                candidate_pattern,
            )
        for name, port in zip(pending, pool):
            # 候选池已经过探测过滤，因此告知 _try_claim 不要再探测一次：第二次探测
            # 会使该层的最坏延迟翻倍（每个串口一次串口超时），却换不来任何额外信息。
            got = _try_claim(
                name, port, claimed=claimed, tier="auto-probe", probe_fn=None, probe_timeout=0.0
            )
            if got is not None:
                out[name] = got
        still = [name for name in pending if name not in out]
        if still:
            logger.warning(
                "unresolved device(s) %s: %d candidate port(s) available (%s)",
                still,
                len(pool),
                pool,
            )

    logger.info("resolve_ports: %d of %d name(s) resolved: %s", len(out), len(wanted), out)
    return out
