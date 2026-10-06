"""YAML 配置基础设施：加载、实例化数据类、原子写入。

每个由 YAML 配置的服务都会反复遇到三个独立问题，本模块一次性解决它们：

1. **映射不是配置对象。** 到处传递 ``cfg["model"]["trt"]["confidence"]`` 意味着
   每个使用方都要重新实现默认值，任何拼写错误都会变成凌晨三点在机器人上才发现的
   ``KeyError``，而且没有任何东西记录文件的结构。:func:`from_dict` 改为遍历一棵
   （可能嵌套的）:mod:`dataclasses` 树：数据类*就是*模式，默认值与字段放在一起，
   缺失的必填键会连同文件路径和缺失的点分键一起报告。
2. **配置模式会演进，而已部署的机器不会。** 把 ``precision`` 重命名为
   ``model.pytorch.precision`` 会破坏所有在重命名之前写下的 YAML。手写的 "如果旧键
   存在，就把它推进新的嵌套字典" 兼容垫片虽然正确但繁琐且容易出错（它们绝不能覆盖
   显式设置的新键）。:func:`from_dict` 接受一张 ``扁平的旧键 -> 点分的新路径`` 的
   ``aliases`` 表，并在字段转换*之前*以 ``setdefault`` 语义应用它，因此新写法总是
   胜出。
3. **默认情况下写配置文件不是原子的。** ``open(path, "w")`` 会立即截断目标文件；
   在截断与最终 ``write`` 之间发生崩溃、磁盘写满或 ``SIGKILL``，都会留下一个已无法
   解析的半截文件——机器随后无法启动到*已知的*状态，而运维人员也丢失了此前的良好
   配置。:func:`dump_yaml` 会在**同一目录**下写一个临时文件，并以
   :func:`os.replace` 收尾，后者在 POSIX 的单个文件系统内是原子的。

模块级只导入标准库和 ``PyYAML``，因此本文件在任何环境中都可安全导入。

环境变量覆盖
---------------------
:func:`load_config` 还可以从名为 ``{env_prefix}_{FIELD_NAME_UPPER}`` 的环境变量
覆盖顶层标量字段。正因如此，一个镜像才能部署到多台机器上：YAML 承载调好的默认值，
而 systemd unit / 启动脚本无需编辑机器人上的文件即可翻转 ``MYAPP_PORT`` 或
``MYAPP_LOG_LEVEL``。

示例
-------
::

    from dataclasses import dataclass, field
    from reusable_model.io.yaml_config import load_config

    @dataclass(frozen=True)
    class TlsConfig:
        enabled: bool = False
        ca_path: str = ""

    @dataclass(frozen=True)
    class AppConfig:
        host: str = "127.0.0.1"
        port: int = 8080
        tls: TlsConfig = field(default_factory=TlsConfig)

    cfg = load_config(
        AppConfig, "service.yaml",
        env_prefix="MYAPP",                      # MYAPP_PORT=9001 覆盖 port
        aliases={"tls_enabled": "tls.enabled"},  # 旧的扁平键仍然可用
    )
"""

from __future__ import annotations

import dataclasses
import logging
import os
import sys
import tempfile
import threading
import types
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence, Union, get_args, get_origin

import yaml

logger = logging.getLogger(__name__)

__all__ = [
    "ConfigError",
    "load_yaml",
    "dump_yaml",
    "from_dict",
    "load_config",
    "resolve_path",
    "YamlStore",
    "BOOL_TRUE_STRINGS",
    "BOOL_FALSE_STRINGS",
]

#: 当布尔值以文本形式到达时被接受为 ``True`` 的拼写（YAML 1.1 对
#: ``on``/``yes`` 的语义含糊，而环境变量始终是字符串）。
BOOL_TRUE_STRINGS: tuple[str, ...] = ("1", "true", "yes", "on")

#: 被接受为 ``False`` 的拼写；两个元组之外的任何值都是错误。
BOOL_FALSE_STRINGS: tuple[str, ...] = ("0", "false", "no", "off")


class ConfigError(ValueError):
    """配置文件无法被读取、解析或实例化。

    继承自 :class:`ValueError`（而非 :class:`RuntimeError`）既能让把 "坏输入数据"
    统一处理的调用方继续使用 ``except ValueError`` 捕获点，又能让启动路径借助这个
    专用类型区分 "运维写错了文件" 与 "磁盘不见了"。

    出错的 ``path`` 与 ``key`` 会随实例一起传递，使 CLI 无需反向解析消息即可打印
    ``service.yaml: model.trt.confidence``。

    参数:
        message: 人类可读的说明，不含位置信息。
        path: 该值来自的文件（或点分逻辑来源），若已知。
        key: 导致失败的配置键，若已知。

    异常:
        TypeError: 如果 ``message`` 不是字符串。（构造器是唯一可能发生此情况的地方；
            异常本身由各加载函数抛出。）

    示例:
        >>> err = ConfigError("missing key", path="service.yaml", key="port")
        >>> err.key, err.path
        ('port', 'service.yaml')
        >>> "service.yaml" in str(err) and "port" in str(err)
        True
    """

    def __init__(self, message: str, *, path: str | os.PathLike[str] | None = None, key: str | None = None) -> None:
        if not isinstance(message, str):
            raise TypeError(f"message must be a str, got {type(message).__name__}: {message!r}")
        self.message = message
        self.path: str | None = None if path is None else str(path)
        self.key: str | None = None if key is None else str(key)
        where: list[str] = []
        if self.path is not None:
            where.append(f"source={self.path}")
        if self.key is not None:
            where.append(f"key={self.key}")
        super().__init__(message if not where else f"{message} ({', '.join(where)})")


# --------------------------------------------------------------------------- IO


def _as_path(value: Any, *, what: str) -> Path:
    """把 ``str`` / :class:`os.PathLike` 强制转换为 :class:`pathlib.Path`。

    参数:
        value: 候选路径。
        what: 用于错误消息的参数名。

    返回:
        以 ``Path`` 表示的值（不做展开或解析）。

    异常:
        TypeError: 如果 ``value`` 不是字符串或 path-like。
        ValueError: 如果得到的路径为空白。

    示例:
        >>> _as_path("a/b.yaml", what="path").name
        'b.yaml'
    """
    if isinstance(value, Path):
        candidate = value
    elif isinstance(value, (str, os.PathLike)):
        candidate = Path(os.fspath(value))
    else:
        raise TypeError(f"{what} must be a str or os.PathLike, got {type(value).__name__}: {value!r}")
    if not str(candidate).strip():
        raise ValueError(f"{what} must be a non-blank path, got {value!r}")
    return candidate


def load_yaml(
    path: str | os.PathLike[str],
    *,
    default: Mapping[str, Any] | None = None,
    allow_missing: bool = False,
) -> dict[str, Any]:
    """把一个 YAML 文件读取为 ``dict``。

    "读取配置文件" 的每一种失败模式都会被转成一个点明文件名的异常，因为这条消息
    通常就是全部诊断信息：

    * 文件缺失抛出 :class:`FileNotFoundError`（当设置了 ``allow_missing`` 时则
      返回 ``default``——"此文件可选" 与 "此文件必须存在" 的区别属于调用方，而非
      此处）；
    * 语法错误抛出携带 PyYAML 问题标记（行/列）的 :class:`ConfigError`，这正是
      运维修复该文件所需的信息；
    * 根为列表或标量的文档抛出 :class:`ConfigError`，因为本模块其余部分都假定
      根是一个映射。

    *空* 文档无论 ``default`` 如何都产出 ``{}``：``default`` 描述的是文件缺失时
    的回退值，而把存在但为空的文件同等对待会掩盖一次被截断的写入。

    参数:
        path: 要读取的文件。``~`` 会被展开。
        default: 当文件缺失且 ``allow_missing`` 为 ``True`` 时返回（副本）的映射。
            ``None`` 表示 ``{}``。
        allow_missing: 为 ``True`` 时，文件缺失不算错误。

    返回:
        解析后的文档，作为普通 ``dict``。

    异常:
        TypeError: 如果 ``path`` 不是 path-like，或 ``default`` 不是映射。
        ValueError: 如果 ``path`` 为空白，或 ``allow_missing`` 不是 bool。
        FileNotFoundError: 如果文件不存在且 ``allow_missing`` 为 ``False``。
        ConfigError: 如果文件不可读、不是合法 YAML，或其根不是映射。

    示例:
        >>> import tempfile, pathlib
        >>> tmp = pathlib.Path(tempfile.mkdtemp())
        >>> target = tmp / "cfg.yaml"
        >>> _ = target.write_text("host: localhost\\nport: 8080\\n")
        >>> load_yaml(target)
        {'host': 'localhost', 'port': 8080}
        >>> load_yaml(tmp / "absent.yaml", allow_missing=True, default={"port": 1})
        {'port': 1}
    """
    target = _as_path(path, what="path").expanduser()
    if default is not None and not isinstance(default, Mapping):
        raise TypeError(f"default must be a mapping or None, got {type(default).__name__}: {default!r}")
    if not isinstance(allow_missing, bool):
        raise ValueError(f"allow_missing must be a bool, got {allow_missing!r}")

    if not target.is_file():
        if allow_missing:
            logger.debug("load_yaml: %s does not exist; using the supplied default", target)
            return dict(default) if default is not None else {}
        raise FileNotFoundError(
            f"configuration file not found: {target} "
            f"(pass allow_missing=True to tolerate an absent file)"
        )

    try:
        text = target.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"cannot read configuration file: {exc}", path=target) from exc

    try:
        parsed = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        mark = getattr(exc, "problem_mark", None)
        location = f" at line {mark.line + 1}, column {mark.column + 1}" if mark is not None else ""
        problem = getattr(exc, "problem", None) or str(exc)
        raise ConfigError(f"invalid YAML{location}: {problem}", path=target) from exc

    if parsed is None:
        return {}
    if not isinstance(parsed, dict):
        raise ConfigError(
            f"configuration root must be a mapping, got {type(parsed).__name__}: {parsed!r}",
            path=target,
        )
    return dict(parsed)


def dump_yaml(
    data: Mapping[str, Any] | Sequence[Any],
    path: str | os.PathLike[str],
    *,
    sort_keys: bool = False,
) -> None:
    """把 ``data`` 原子地序列化到 ``path``。

    写入会落到**目标目录内**的一个临时文件，并以 :func:`os.replace` 收尾。这两个
    细节都很重要：

    * 写入过程中发生崩溃、磁盘写满或 ``SIGKILL`` 绝不能留下一个被截断的配置文件
      ——否则下次启动会因一个*看起来*存在的文件而失败，而此前良好的内容已经不在；
    * 临时文件必须位于同一目录，因为 :func:`os.replace` 只在单个文件系统内是原子的
      （跨文件系统会以 ``EXDEV`` 失败），而 ``/tmp`` 常常是 tmpfs。

    如果序列化失败，临时文件会被再次删除，因此一个坏 ``data`` 值绝不会在目标旁边
    留下垃圾。

    ``sort_keys`` 默认为 ``False``：配置文件是给人读的，保留作者的顺序（把相关的键
    分组）比规范的键顺序更有价值。当输出要由机器做差异比较时再传 ``True``。

    参数:
        data: 一个映射或一个由 YAML 安全值组成的序列。裸标量会被拒绝，因为一个只
            装着单个数字的 "配置文件" 几乎总是调用方的 bug。
        path: 目标文件。缺失的父目录会被创建。
        sort_keys: 按键名字母顺序排序映射。

    返回:
        ``None``。

    异常:
        TypeError: 如果 ``data`` 不是映射/序列、``path`` 不是 path-like，或
            ``sort_keys`` 不是 bool。
        ValueError: 如果 ``path`` 为空白或 ``data`` 是字符串/bytes。
        ConfigError: 如果 ``data`` 无法被 ``yaml.safe_dump`` 序列化，或文件无法
            写入。

    示例:
        >>> import tempfile, pathlib
        >>> tmp = pathlib.Path(tempfile.mkdtemp()) / "out.yaml"
        >>> dump_yaml({"port": 8080, "host": "localhost"}, tmp)
        >>> tmp.read_text().strip().splitlines()[0]
        'port: 8080'
    """
    if isinstance(data, (str, bytes, bytearray)):
        raise ValueError(f"data must be a mapping or a sequence, got {type(data).__name__}: {data!r}")
    if not isinstance(data, (Mapping, Sequence)):
        raise TypeError(
            f"data must be a mapping or a sequence, got {type(data).__name__}: {data!r}"
        )
    if not isinstance(sort_keys, bool):
        raise TypeError(f"sort_keys must be a bool, got {type(sort_keys).__name__}: {sort_keys!r}")
    target = _as_path(path, what="path").expanduser()

    try:
        text = yaml.safe_dump(
            data if isinstance(data, (dict, list)) else dict(data),
            sort_keys=sort_keys,
            allow_unicode=True,
            default_flow_style=False,
        )
    except yaml.YAMLError as exc:
        raise ConfigError(f"cannot serialise configuration data: {exc}", path=target) from exc

    directory = target.parent
    try:
        directory.mkdir(parents=True, exist_ok=True)
        handle = tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=str(directory),
            prefix=f".{target.name}.",
            suffix=".tmp",
            delete=False,
        )
    except OSError as exc:
        raise ConfigError(f"cannot prepare a temporary file next to {target}: {exc}", path=target) from exc

    tmp_name = handle.name
    try:
        with handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_name, target)
    except OSError as exc:
        try:
            os.unlink(tmp_name)
        except OSError:
            logger.debug("dump_yaml: could not remove the temporary file %s", tmp_name)
        raise ConfigError(f"cannot write configuration file {target}: {exc}", path=target) from exc
    logger.debug("dump_yaml: wrote %d bytes to %s", len(text), target)


# --------------------------------------------------------------- 数据类粘合


def _parse_bool(value: Any, *, key: str, source: str | None) -> bool:
    """转换一个可能以文本形式到达的布尔值。

    ``bool("false")`` 的结果是 ``True``，这是配置处理中最具破坏性的静默转换：
    运维在环境变量里写了 ``enabled: false``，开关却*打开*了，而且没有任何提示。
    这里只接受显式的允许列表；其余一切都会抛出异常。

    参数:
        value: 一个 ``bool``、一个取值在 ``{0, 1}`` 的 ``int``，或一个字符串。
        key: 用于错误消息的字段名。
        source: 用于错误消息的文件/点分路径。

    返回:
        布尔值。

    异常:
        ConfigError: 如果 ``value`` 不是被接受的拼写之一。

    示例:
        >>> _parse_bool("YES", key="flag", source=None)
        True
        >>> _parse_bool("maybe", key="flag", source=None)
        Traceback (most recent call last):
            ...
        reusable_model.io.yaml_config.ConfigError: flag must be a boolean; accepted spellings are ('1', 'true', 'yes', 'on') / ('0', 'false', 'no', 'off'), got 'maybe' (key=flag)
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        if value in (0, 1):
            return bool(value)
        raise ConfigError(f"{key} must be a boolean, got int {value!r}", path=source, key=key)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in BOOL_TRUE_STRINGS:
            return True
        if text in BOOL_FALSE_STRINGS:
            return False
        raise ConfigError(
            f"{key} must be a boolean; accepted spellings are "
            f"{BOOL_TRUE_STRINGS} / {BOOL_FALSE_STRINGS}, got {value!r}",
            path=source,
            key=key,
        )
    raise ConfigError(f"{key} must be a boolean, got {type(value).__name__}: {value!r}", path=source, key=key)


def _coerce_scalar(value: Any, annotation: Any, *, key: str, source: str | None) -> Any:
    """把 ``value`` 转换为注解所声明的标量类型，或原样传递。

    只转换配置文件能够有意义地表达的注解（``str``、``int``、``float``、``bool``、
    :class:`~pathlib.Path`）。其他一切——``Any``、``dict[str, Any]``、``list[int]``、
    第三方类——都会原样返回，因为为其猜测一种转换恰恰是本模块存在所要防止的静默
    错误行为。

    ``int``/``float`` 字段拒绝布尔值：在 Python 中 ``isinstance(True, int)`` 为
    ``True``，否则 ``retries: yes`` 会变成 ``1``。``int`` 字段接受整值浮点数，
    因为 YAML 解析器和 JSON 往返序列化会顺理成章地把 ``8080`` 变成 ``8080.0``。

    参数:
        value: 来自映射的原始值（从不为 ``None``）。
        annotation: 目标字段解析后的类型注解。
        key: 用于错误消息的字段名。
        source: 用于错误消息的文件/点分路径。

    返回:
        转换后的值；当注解不是受支持的标量时，返回未改动的 ``value``。

    异常:
        ConfigError: 如果该值无法转换为注解所声明的类型。

    示例:
        >>> _coerce_scalar("8080", int, key="port", source=None)
        8080
        >>> _coerce_scalar("models/x", Path, key="path", source=None).name
        'x'
    """
    if annotation is Any or annotation is None or annotation is object:
        return value
    if annotation is bool:
        return _parse_bool(value, key=key, source=source)
    if annotation is int:
        if isinstance(value, bool):
            raise ConfigError(f"{key} must be an int, got bool {value!r}", path=source, key=key)
        if isinstance(value, int):
            return value
        if isinstance(value, float):
            if value.is_integer():
                return int(value)
            raise ConfigError(
                f"{key} must be an int, got the non-integral float {value!r}", path=source, key=key
            )
        if isinstance(value, str):
            try:
                return int(value.strip(), 0) if value.strip()[:2].lower() in ("0x", "0o", "0b") else int(value.strip())
            except ValueError as exc:
                raise ConfigError(f"{key} must be an int, got {value!r}", path=source, key=key) from exc
        raise ConfigError(f"{key} must be an int, got {type(value).__name__}: {value!r}", path=source, key=key)
    if annotation is float:
        if isinstance(value, bool):
            raise ConfigError(f"{key} must be a float, got bool {value!r}", path=source, key=key)
        if isinstance(value, (int, float)):
            return float(value)
        if isinstance(value, str):
            try:
                return float(value.strip())
            except ValueError as exc:
                raise ConfigError(f"{key} must be a float, got {value!r}", path=source, key=key) from exc
        raise ConfigError(f"{key} must be a float, got {type(value).__name__}: {value!r}", path=source, key=key)
    if annotation is str:
        if isinstance(value, str):
            return value
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            return str(value)
        if isinstance(value, bool):
            return "true" if value else "false"
        raise ConfigError(
            f"{key} must be a str, got {type(value).__name__}: {value!r}", path=source, key=key
        )
    if annotation is Path or (isinstance(annotation, type) and issubclass(annotation, Path)):
        if isinstance(value, Path):
            return value
        if isinstance(value, (str, os.PathLike)):
            return Path(os.fspath(value))
        raise ConfigError(f"{key} must be a path, got {type(value).__name__}: {value!r}", path=source, key=key)
    return value


def _field_types(cls: type) -> dict[str, Any]:
    """把数据类的字段注解解析为真正的类型对象。

    ``from __future__ import annotations`` 会把注解存为*字符串*，因此 ``field.type``
    无法用于判断 "这个字段是不是嵌套数据类？"。:func:`typing.get_type_hints` 会在
    定义该类的模块命名空间中求值这些注解；当它失败时（某个前向引用的类型在运行期
    确实不可用），注解会改为**逐字段**求值，这样一个坏的前向引用就不会悄悄让同一
    类中所有其他字段都无法进行嵌套数据类转换。

    无法解析的字段会保留其原始字符串注解——这会让 :func:`_convert_value` 原样传递
    它们的值——并记录一条警告，因为一个本该是数据类的地方却保持为普通 dict 的嵌套
    块，正是那种要花数小时才能找到的静默退化。

    参数:
        cls: 一个数据类类型。

    返回:
        字段名到解析后注解的映射（无法解析时为字符串）。

    示例:
        >>> @dataclasses.dataclass
        ... class Demo:
        ...     port: int = 1
        >>> _field_types(Demo)["port"]
        <class 'int'>
    """
    import typing

    try:
        return dict(typing.get_type_hints(cls))
    except Exception as exc:  # noqa: BLE001 - 无法解析的提示不应中断加载
        logger.debug("cannot resolve the type hints of %s in one pass: %s", cls.__name__, exc)

    module_vars = dict(getattr(sys.modules.get(cls.__module__, None), "__dict__", {}))
    try:
        # 一个类可以生活在其 ``__module__`` 之外的命名空间中（doctest 示例、
        # ``exec`` 构建的模块、notebook 单元）。模块字典通常是解析注解的地方，
        # 但真正定义该类的那个最浅栈帧未必是模块——因此合并所有可见栈帧的全局
        # 变量；持有该类名的那个胜出。
        frame = sys._getframe(1)
        while frame is not None:
            module_vars.update(frame.f_globals)
            frame = frame.f_back
    except (ValueError, AttributeError):
        pass
    class_vars = dict(vars(cls))
    resolved: dict[str, Any] = {}
    unresolved: list[str] = []
    for item in dataclasses.fields(cls):
        annotation = item.type
        if isinstance(annotation, str):
            try:
                # 与 typing.get_type_hints 执行的求值相同；该字符串来自这个类自身
                # 的源码，而不是用户数据。
                annotation = eval(annotation, module_vars, class_vars)  # noqa: S307
            except Exception:  # noqa: BLE001 - 改为保留原始注解
                unresolved.append(item.name)
                annotation = item.type
        resolved[item.name] = annotation
    if unresolved:
        logger.warning(
            "%s: could not resolve the annotation(s) of %s; those fields keep their raw "
            "value instead of being converted (a nested dataclass would stay a dict)",
            cls.__name__,
            unresolved,
        )
    return resolved


def _unwrap_optional(annotation: Any) -> tuple[Any, bool]:
    """把 ``Optional[X]`` / ``X | None`` 拆分为 ``(X, is_nullable)``。

    参数:
        annotation: 一个已解析的类型注解。

    返回:
        一个元组，包含非 ``None`` 的注解，以及是否允许 ``None``。由多个真实类型
        组成的联合类型会原样返回（只剥离 ``None``），因为它没有单一适用的转换。

    示例:
        >>> _unwrap_optional(int | None)
        (<class 'int'>, True)
        >>> _unwrap_optional(int)
        (<class 'int'>, False)
    """
    origin = get_origin(annotation)
    if origin is Union or origin is types.UnionType:
        args = get_args(annotation)
        real = [a for a in args if a is not types.NoneType]
        nullable = len(real) != len(args)
        if len(real) == 1:
            return real[0], nullable
        if not real:
            return types.NoneType, nullable
        return Union[tuple(real)], nullable  # noqa: UP007 - 运行时构建的联合类型
    return annotation, False


def _join_source(source: str | None, key: str) -> str:
    """构建用于嵌套错误消息的点分位置字符串。

    参数:
        source: 父位置，根位置处为 ``None``。
        key: 正在进入的字段名。

    返回:
        ``"parent.key"``，在根位置处则为 ``key``。

    示例:
        >>> _join_source("service.yaml", "model")
        'service.yaml: model'
    """
    return key if source is None else f"{source}: {key}"


def _apply_aliases(
    data: dict[str, Any],
    aliases: Mapping[str, str] | None,
    *,
    known: Iterable[str] = (),
    source: str | None = None,
) -> dict[str, Any]:
    """把旧的扁平键改写为其嵌套位置。

    对目标叶子使用 ``setdefault`` 语义：如果新拼写已经存在则它胜出，因为显式写下的
    嵌套键比同一文件中残留的旧扁平键表达了更新的意图。改写会复制它深入到的每一个
    映射，因此调用方的字典绝不会被改动（否则一个复用原始文档的调用方——用于日志、
    用于某个 ``raw`` 字段——就会看到兼容垫片所做的修改）。

    一个被消费掉的旧键会从结果中移除，除非它同时也是一个已声明的字段名；这样就能
    避免 ``strict=True`` 报告一个垫片已经搬走的键。

    参数:
        data: 正在被转换的映射的浅拷贝。
        aliases: ``{旧键: 点分新路径}``。不含点的目标就是在同一映射内的纯重命名。
        known: 目标数据类已声明的字段名。
        source: 用于错误消息的位置字符串。

    返回:
        改写后的映射（一个新的 dict）。

    异常:
        TypeError: 如果 ``aliases`` 不是映射，或某个键/目标不是字符串。
        ConfigError: 如果某个别名目标为空白或包含空片段。

    示例:
        >>> _apply_aliases({"precision": "fp16"}, {"precision": "pytorch.precision"})
        {'pytorch': {'precision': 'fp16'}}
    """
    if not aliases:
        return data
    if not isinstance(aliases, Mapping):
        raise TypeError(f"aliases must be a mapping, got {type(aliases).__name__}: {aliases!r}")
    declared = set(known)
    out = data
    for legacy, dotted in aliases.items():
        if not isinstance(legacy, str) or not isinstance(dotted, str):
            raise TypeError(
                f"aliases entries must be str -> str, got {legacy!r} -> {dotted!r}"
            )
        if legacy not in out:
            continue
        parts = [p for p in dotted.split(".")]
        if not dotted.strip() or any(not p.strip() for p in parts):
            raise ConfigError(
                f"alias target for {legacy!r} must be a non-empty dotted path, got {dotted!r}",
                path=source,
                key=legacy,
            )
        cursor = out
        for part in parts[:-1]:
            child = cursor.get(part)
            if isinstance(child, Mapping):
                child = dict(child)
            elif child is None:
                child = {}
            else:
                raise ConfigError(
                    f"alias {legacy!r} -> {dotted!r} cannot be applied: {part!r} already "
                    f"holds a {type(child).__name__}",
                    path=source,
                    key=legacy,
                )
            cursor[part] = child
            cursor = child
        leaf = parts[-1]
        if leaf not in cursor:
            cursor[leaf] = out[legacy]
            logger.debug("applied configuration alias %r -> %r", legacy, dotted)
        else:
            logger.debug(
                "ignoring alias %r -> %r because %r is set explicitly", legacy, dotted, leaf
            )
        if legacy not in declared:
            out.pop(legacy, None)
    return out


def _convert_value(
    raw: Any,
    annotation: Any,
    *,
    name: str,
    source: str | None,
    strict: bool,
) -> Any:
    """把一个原始值转换为对应字段注解所声明的类型。

    参数:
        raw: 取自映射的值。
        annotation: 该字段解析后的注解。
        name: 字段名，用于错误消息。
        source: 位置字符串，用于错误消息。
        strict: 转发给嵌套的 :func:`from_dict` 调用。

    返回:
        转换后的值。

    异常:
        ConfigError: 如果非可选字段收到了 ``None``、某个嵌套数据类字段持有的不是
            映射，或某个标量无法转换。

    示例:
        >>> _convert_value("8080", int, name="port", source=None, strict=False)
        8080
    """
    inner, nullable = _unwrap_optional(annotation)
    if raw is None:
        if nullable or inner is Any or inner is types.NoneType:
            return None
        raise ConfigError(
            f"{name} must not be null (expected {getattr(inner, '__name__', inner)})",
            path=source,
            key=name,
        )
    if isinstance(inner, type) and dataclasses.is_dataclass(inner):
        if isinstance(raw, inner):
            return raw
        if not isinstance(raw, Mapping):
            raise ConfigError(
                f"{name} must be a mapping to build {inner.__name__}, "
                f"got {type(raw).__name__}: {raw!r}",
                path=source,
                key=name,
            )
        return from_dict(inner, raw, source=_join_source(source, name), strict=strict)
    return _coerce_scalar(raw, inner, key=name, source=source)


def from_dict(
    cls: type,
    data: Mapping[str, Any],
    *,
    source: str | None = None,
    strict: bool = False,
    aliases: Mapping[str, str] | None = None,
) -> Any:
    """从一个映射构建（可能嵌套的）数据类实例。

    数据类就是模式：带默认值的字段在映射中可选，不带默认值的字段为必填，其缺失会
    抛出 :class:`ConfigError`，并同时指出键与来源。嵌套的数据类字段会被递归转换，
    因此 YAML 中的 ``model.pytorch.precision`` 会变成
    ``cfg.model.pytorch.precision``，无需任何手写粘合代码。

    除非设置 ``strict``，未知键会被忽略。默认忽略它们是有意为之：YAML 文件常常
    携带充当注释的键、工具元数据，或由同一次部署中*另一个*组件消费的键，因为这些
    而拒绝启动比容忍它们更糟。``strict=True`` 适合由一个组件完全拥有的配置文件，
    否则一个拼错的键会悄悄回退到默认值。

    参数:
        cls: 要实例化的数据类类型。
        data: 字段名到值的映射。
        source: 对 ``data`` 来源的描述（文件路径、点分键）。它只出现在错误消息和
            日志中。
        strict: 对不是 ``cls`` 字段的键抛出 :class:`ConfigError`。
        aliases: ``{旧扁平键: 点分新路径}``，在转换之前以 ``setdefault`` 语义应用，
            用于向后兼容。只在本层应用；嵌套数据类接收到的是已改写好的子树。

    返回:
        ``cls`` 的一个实例。

    异常:
        TypeError: 如果 ``cls`` 不是数据类类型、``data`` 不是映射、``strict`` 不是
            bool，或 ``aliases`` 形状不对。
        ConfigError: 如果缺少必填键、某个值的类型与其注解不符、某个嵌套块不是映射，
            或（在严格模式下）存在未知键。

    示例:
        >>> @dataclasses.dataclass(frozen=True)
        ... class Sub:
        ...     precision: str = "fp16"
        >>> @dataclasses.dataclass(frozen=True)
        ... class Root:
        ...     name: str
        ...     sub: Sub = dataclasses.field(default_factory=Sub)
        >>> from_dict(Root, {"name": "x", "precision": "fp32"},
        ...           aliases={"precision": "sub.precision"}).sub.precision
        'fp32'
        >>> from_dict(Root, {"sub": {}})
        Traceback (most recent call last):
            ...
        reusable_model.io.yaml_config.ConfigError: missing required key 'name' for Root; declared keys are ['name', 'sub'] (key=name)
    """
    if not isinstance(cls, type) or not dataclasses.is_dataclass(cls):
        raise TypeError(
            f"cls must be a dataclass type, got {cls!r} ({type(cls).__name__})"
        )
    if not isinstance(data, Mapping):
        raise TypeError(
            f"data must be a mapping for {cls.__name__}, got {type(data).__name__}: {data!r}"
        )
    if not isinstance(strict, bool):
        raise TypeError(f"strict must be a bool, got {type(strict).__name__}: {strict!r}")

    fields = dataclasses.fields(cls)
    declared = {f.name for f in fields}
    payload = _apply_aliases(dict(data), aliases, known=declared, source=source)
    hints = _field_types(cls)

    kwargs: dict[str, Any] = {}
    for item in fields:
        if item.name not in payload:
            has_default = (
                item.default is not dataclasses.MISSING
                or item.default_factory is not dataclasses.MISSING  # type: ignore[misc]
            )
            if not has_default:
                raise ConfigError(
                    f"missing required key {item.name!r} for {cls.__name__}"
                    + (f"; declared keys are {sorted(declared)}" if declared else ""),
                    path=source,
                    key=item.name,
                )
            continue
        kwargs[item.name] = _convert_value(
            payload[item.name],
            hints.get(item.name, item.type),
            name=item.name,
            source=source,
            strict=strict,
        )

    extra = sorted(set(payload) - declared)
    if extra:
        if strict:
            raise ConfigError(
                f"unknown key(s) {extra} for {cls.__name__}; declared keys are {sorted(declared)}",
                path=source,
                key=extra[0],
            )
        logger.debug("%s: ignoring unknown key(s) %s", source or cls.__name__, extra)

    try:
        return cls(**kwargs)
    except (TypeError, ValueError) as exc:
        raise ConfigError(
            f"cannot build {cls.__name__} from {source or '<mapping>'}: {exc}", path=source
        ) from exc


def _env_var_name(prefix: str, field_name: str) -> str:
    """为一个字段构建其环境变量名。

    参数:
        prefix: 调用方提供的前缀，已校验为非空白。
        field_name: 数据类字段名。

    返回:
        ``"{PREFIX}_{FIELD}"``，其中字段名转为大写，这是配置类环境变量近乎通用的
        约定。

    示例:
        >>> _env_var_name("myapp", "log_level")
        'myapp_LOG_LEVEL'
    """
    return f"{prefix}_{field_name.upper()}"


def _coerce_env(raw: str, annotation: Any, *, var: str, key: str) -> Any:
    """把环境变量字符串转换为字段注解所声明的类型。

    参数:
        raw: 变量的值（始终是字符串）。
        annotation: 目标字段解析后的注解。
        var: 变量名，用于错误消息。
        key: 字段名，用于错误消息。

    返回:
        转换后的值；当注解不是受支持的标量时，返回未改动的 ``raw``。

    异常:
        ConfigError: 如果该值无法按注解类型解析。消息会指出变量名，使运维知道该
            取消设置哪个变量。

    示例:
        >>> _coerce_env("0", bool, var="MYAPP_DEBUG", key="debug")
        False
    """
    inner, _nullable = _unwrap_optional(annotation)
    try:
        return _coerce_scalar(raw, inner, key=f"{var} (field {key})", source=os.environ.get(var, ""))
    except ConfigError as exc:
        raise ConfigError(str(exc.message), path=f"env:{var}", key=key) from exc


def load_config(
    cls: type,
    path: str | os.PathLike[str],
    *,
    env_prefix: str | None = None,
    aliases: Mapping[str, str] | None = None,
    strict: bool = False,
) -> Any:
    """把一个 YAML 文件加载为数据类，可选地应用环境变量覆盖。

    这相当于 :func:`load_yaml` 后再调用 :func:`from_dict`，外加环境变量覆盖层。
    覆盖值从 ``{env_prefix}_{FIELD_NAME_UPPER}`` 读取，作用于每个**顶层标量**字段
    （``str``、``int``、``float``、``bool``、:class:`~pathlib.Path`），并按字段的
    注解进行转换：

    * ``bool`` 不区分大小写地接受 ``1``/``true``/``yes``/``on`` 与
      ``0``/``false``/``no``/``off``。其他任何拼写都会抛出 :class:`ConfigError`，
      而不是回退到 :func:`bool`——后者会把 ``"false"`` 变成 ``True``。
    * 空字符串*不会*被当作 "未设置"：它会赋给 ``str`` 字段，而对数值/布尔字段则被
      拒绝。静默的 "空即忽略" 规则正是某次部署最终跑在没人选择过的默认值上的原因。
    * 嵌套数据类字段无法从环境变量触达。请使用 ``aliases`` 加一个顶层字段，或直接
      编辑文件——发明一种点分变量语法会使可覆盖的范围无法枚举。

    ``env_prefix=None`` 会完全禁用该特性，不读取任何环境变量，从而使函数保持纯粹、
    结果可复现。

    由于数据类可能是冻结的，当普通赋值抛出
    :class:`dataclasses.FrozenInstanceError` 时，覆盖值会改用
    :func:`object.__setattr__` 写入。

    参数:
        cls: 要实例化的数据类类型。
        path: 要读取的 YAML 文件。
        env_prefix: 覆盖变量的前缀，例如 ``"MYAPP"``。``None`` 禁用覆盖。
        aliases: 转发给 :func:`from_dict` 的旧键表。
        strict: 转发给 :func:`from_dict`。

    返回:
        ``cls`` 的一个实例。

    异常:
        TypeError: 如果 ``cls`` 不是数据类、``path`` 不是 path-like，或
            ``env_prefix`` 不是字符串/``None``。
        ValueError: 如果 ``env_prefix`` 为空白。
        FileNotFoundError: 如果文件不存在。
        ConfigError: 如果文件无效、缺少必填键，或某个环境变量无法转换。

    示例:
        >>> import os, tempfile, pathlib
        >>> @dataclasses.dataclass
        ... class Svc:
        ...     host: str = "127.0.0.1"
        ...     port: int = 8080
        ...     debug: bool = False
        >>> tmp = pathlib.Path(tempfile.mkdtemp()) / "svc.yaml"
        >>> _ = tmp.write_text("port: 1\\n")
        >>> os.environ["DEMO_PORT"] = "9001"; os.environ["DEMO_DEBUG"] = "yes"
        >>> cfg = load_config(Svc, tmp, env_prefix="DEMO")
        >>> (cfg.port, cfg.debug)
        (9001, True)
    """
    if env_prefix is not None:
        if not isinstance(env_prefix, str):
            raise TypeError(
                f"env_prefix must be a str or None, got {type(env_prefix).__name__}: {env_prefix!r}"
            )
        if not env_prefix.strip():
            raise ValueError(f"env_prefix must be a non-blank string, got {env_prefix!r}")
        env_prefix = env_prefix.strip()

    target = _as_path(path, what="path")
    data = load_yaml(target)
    cfg = from_dict(cls, data, source=str(target), strict=strict, aliases=aliases)

    if env_prefix is None:
        return cfg

    hints = _field_types(cls) if isinstance(cls, type) else {}
    for item in dataclasses.fields(cfg):  # type: ignore[arg-type]
        var = _env_var_name(env_prefix, item.name)
        if var not in os.environ:
            continue
        annotation = hints.get(item.name, item.type)
        inner, _nullable = _unwrap_optional(annotation)
        if inner not in (str, int, float, bool, Path) and not (
            isinstance(inner, type) and issubclass(inner, Path)
        ):
            logger.debug(
                "environment override %s ignored: field %r is annotated %r, not a scalar",
                var,
                item.name,
                inner,
            )
            continue
        value = _coerce_env(os.environ[var], annotation, var=var, key=item.name)
        try:
            setattr(cfg, item.name, value)
        except dataclasses.FrozenInstanceError:
            object.__setattr__(cfg, item.name, value)
        logger.info("environment override applied: %s -> %s=%r", var, item.name, value)
    return cfg


def resolve_path(
    value: str | os.PathLike[str],
    *,
    root: str | os.PathLike[str] | None = None,
    roots_marker: str = "models",
) -> Path:
    """把配置文件中写下的路径针对运行期根目录进行解析。

    它泛化了每个推理服务都需要的 "模型路径" 解析。按顺序分三种情况：

    1. **绝对路径**——原样返回（已解析）。写下绝对路径的运维就是要那个路径；重新
       加根会是个意外。
    2. **首个组成部分等于 ``roots_marker``**——针对 ``root/roots_marker`` 解析。
       配置文件存储的是*逻辑*资产路径（``models/sam3/weights.plan``），它们必须
       在部署级资产根下找到：这个目录可能是指向共享存储的符号链接、一个只读挂载，
       或是按机器提供的某个位置。针对该标记解析可使逻辑路径保持可移植，而物理位置
       仍是部署细节。
    3. **其他一切**——相对于 ``root`` 解析。

    文件不必存在：本函数关心的是*它会在哪里*，调用方通常想在 "未找到" 错误里报告
    解析后的路径。

    参数:
        value: 配置中写下的路径。``~`` 会被展开。
        root: 运行期根目录。``None`` 表示当前工作目录。
        roots_marker: 用于选择资产根的首个路径组成部分。必须是单个、非空白的
            组成部分。

    返回:
        解析后的 :class:`~pathlib.Path`。

    异常:
        TypeError: 如果 ``value``/``root`` 不是 path-like，或 ``roots_marker`` 不是
            字符串。
        ValueError: 如果 ``value`` 或 ``roots_marker`` 为空白，或标记中包含路径
            分隔符。

    示例:
        >>> resolve_path("/abs/x.plan", root="/opt/app").as_posix()
        '/abs/x.plan'
        >>> resolve_path("models/sam3/w.plan", root="/opt/app").as_posix()
        '/opt/app/models/sam3/w.plan'
        >>> resolve_path("config/svc.yaml", root="/opt/app").as_posix()
        '/opt/app/config/svc.yaml'
    """
    candidate = _as_path(value, what="value").expanduser()
    if not isinstance(roots_marker, str):
        raise TypeError(
            f"roots_marker must be a str, got {type(roots_marker).__name__}: {roots_marker!r}"
        )
    if not roots_marker.strip():
        raise ValueError(f"roots_marker must be a non-blank path component, got {roots_marker!r}")
    if os.sep in roots_marker or (os.altsep and os.altsep in roots_marker) or "/" in roots_marker:
        raise ValueError(
            f"roots_marker must be a single path component without separators, got {roots_marker!r}"
        )
    base = Path.cwd() if root is None else _as_path(root, what="root").expanduser()

    if candidate.is_absolute():
        return candidate.resolve()
    parts = candidate.parts
    if parts and parts[0] == roots_marker:
        return base.joinpath(roots_marker, *parts[1:]).resolve()
    return (base / candidate).resolve()


class YamlStore:
    """一个以文件名为主键、由目录支撑且带缓存的 YAML 文档集合。

    约定是 **"文件名主干即主键"**：一个装有 ``garbage.yaml`` 和 ``behavior.yaml``
    的目录，会以 ``"garbage"`` 和 ``"behavior"`` 暴露这些文档。这让新增一个条目
    变成一次文件系统操作——把 YAML 文件丢进目录即可——无需更新注册表，也无需评审
    代码改动，这正是提示词库、模型档案和按机器的参数集采用这种方式组织的原因。

    重新加载是**显式**的（:meth:`reload`）。这里刻意不做文件监视：监视器会引入
    第二种、不可见地改变进程状态的途径，使一个编辑文件的测试因时序而时过时不过，
    也让生产行为依赖于容器内 inotify 是否工作。当确实需要热更新时，请从信号处理器
    或管理端点调用 :meth:`reload`。

    文件通过两条相互独立的规则被排除，因为它们覆盖不同的意图：``skip_prefix``
    隐藏*不是*条目的文件（前导下划线读作 "草稿/私有"，也让运维可以通过重命名来禁用
    一个条目），而 ``skip_names`` 隐藏*确实*有意义但会被单独消费的文件（一个共享的
    ``extra.yaml`` 片段，或一张不应作为可选项集合提供的查找表）。

    所有读取以及 :meth:`reload` 执行的换入都发生在锁保护之下，且 :meth:`reload`
    会先构建一个全新的字典再换入，因此读者要么看到旧集合、要么看到新集合——绝不会
    看到一个半填充的混合体。

    参数:
        directory: 要扫描的目录。构造时必须存在；在这里失败会立刻指向拼错的路径，
            而不是在第一个请求的深处才暴露出来。
        skip_names: 要排除的文件名主干（不含扩展名）。
        skip_prefix: 排除*名称*以此前缀开头的文件。``""`` 或 ``None`` 禁用该规则。
        pattern: 在 ``directory`` 内使用的 glob（不递归）。不得包含路径分隔符。

    异常:
        TypeError: 如果 ``directory`` 不是 path-like、``skip_names`` 中含有非字符串，
            或 ``pattern``/``skip_prefix`` 不是字符串。
        ValueError: 如果 ``pattern`` 为空白或包含分隔符。
        FileNotFoundError: 如果 ``directory`` 不存在。
        ConfigError: 如果其中的任何文件不可读、不是合法 YAML，或未持有映射。

    示例:
        >>> import tempfile, pathlib
        >>> tmp = pathlib.Path(tempfile.mkdtemp())
        >>> _ = (tmp / "alpha.yaml").write_text("a: 1\\n")
        >>> _ = (tmp / "_draft.yaml").write_text("b: 2\\n")
        >>> store = YamlStore(tmp)
        >>> store.keys(), store["alpha"], len(store), "alpha" in store
        (['alpha'], {'a': 1}, 1, True)
    """

    def __init__(
        self,
        directory: str | os.PathLike[str],
        *,
        skip_names: Iterable[str] = (),
        skip_prefix: str | None = "_",
        pattern: str = "*.yaml",
    ) -> None:
        self.directory: Path = _as_path(directory, what="directory").expanduser()
        if isinstance(skip_names, (str, bytes)) or not isinstance(skip_names, Iterable):
            raise TypeError(
                f"skip_names must be an iterable of str, got {type(skip_names).__name__}: "
                f"{skip_names!r}"
            )
        self.skip_names: frozenset[str] = frozenset(skip_names)
        for name in self.skip_names:
            if not isinstance(name, str):
                raise TypeError(f"skip_names entries must be str, got {type(name).__name__}: {name!r}")
        if skip_prefix is not None and not isinstance(skip_prefix, str):
            raise TypeError(
                f"skip_prefix must be a str or None, got {type(skip_prefix).__name__}: {skip_prefix!r}"
            )
        self.skip_prefix: str = "" if skip_prefix is None else skip_prefix
        if not isinstance(pattern, str) or not pattern.strip():
            raise ValueError(f"pattern must be a non-blank glob string, got {pattern!r}")
        if os.sep in pattern or "/" in pattern:
            raise ValueError(f"pattern must not contain a path separator, got {pattern!r}")
        self.pattern: str = pattern

        self._lock = threading.RLock()
        self._items: dict[str, Any] = {}
        self.reload()

    # ----------------------------------------------------------------- 加载

    def _include(self, path: Path) -> bool:
        """决定一个被扫描到的文件是否成为一个条目。

        参数:
            path: :attr:`directory` 内的候选文件。

        返回:
            当该文件应以其主干名加载时返回 ``True``。

        示例:
            >>> import tempfile, pathlib
            >>> tmp = pathlib.Path(tempfile.mkdtemp()); _ = (tmp / "a.yaml").write_text("{}")
            >>> YamlStore(tmp)._include(tmp / "_a.yaml")
            False
        """
        if self.skip_prefix and path.name.startswith(self.skip_prefix):
            logger.debug("YamlStore: skipping %s (prefix %r)", path.name, self.skip_prefix)
            return False
        if path.stem in self.skip_names:
            logger.debug("YamlStore: skipping %s (listed in skip_names)", path.name)
            return False
        return True

    def reload(self) -> dict[str, Any]:
        """重新扫描目录并替换已缓存的文档。

        加载会先进行到一个本地字典中，只有在每个文件都成功解析之后才在锁保护下
        换入，因此单个损坏的文件会让此前良好的集合保持原样*并且*抛出异常：一次
        部分应用的重新加载远比以上两种结果中的任何一种都更难诊断。

        返回:
            新的 ``{主干: 文档}`` 映射的浅拷贝。

        异常:
            FileNotFoundError: 如果该目录自构造以来已消失。
            ConfigError: 如果任何文件不可读、不是合法 YAML，或未持有映射。消息会
                指出该文件名。

        示例:
            >>> import tempfile, pathlib
            >>> tmp = pathlib.Path(tempfile.mkdtemp()); _ = (tmp / "a.yaml").write_text("v: 1")
            >>> store = YamlStore(tmp)
            >>> _ = (tmp / "b.yaml").write_text("v: 2")
            >>> sorted(store.reload())
            ['a', 'b']
        """
        if not self.directory.is_dir():
            raise FileNotFoundError(f"configuration directory not found: {self.directory}")
        loaded: dict[str, Any] = {}
        for path in sorted(self.directory.glob(self.pattern)):
            if not path.is_file() or not self._include(path):
                continue
            document = load_yaml(path)
            if path.stem in loaded:
                # 只有在两个模式/扩展名塌缩到同一个主干名时才会走到这里；否则
                # 这套主键约定会悄悄丢失一个文件。
                raise ConfigError(
                    f"duplicate store key {path.stem!r}: {path} collides with an already "
                    f"loaded file of the same stem",
                    path=path,
                )
            loaded[path.stem] = document
        with self._lock:
            self._items = loaded
        logger.info(
            "YamlStore: loaded %d document(s) from %s (%s)",
            len(loaded),
            self.directory,
            ", ".join(sorted(loaded)) or "<empty>",
        )
        return dict(loaded)

    # ------------------------------------------------------------------ 访问

    def keys(self) -> list[str]:
        """返回排序后的条目名。

        排序而非按插入顺序，是因为该结果经常在错误消息中展示给人看
        （"unknown set 'x', available: ..."），而稳定的顺序使这些消息在多次运行
        之间可以相互比较。

        返回:
            一个新的键列表。

        示例:
            >>> import tempfile, pathlib
            >>> tmp = pathlib.Path(tempfile.mkdtemp()); _ = (tmp / "a.yaml").write_text("{}")
            >>> YamlStore(tmp).keys()
            ['a']
        """
        with self._lock:
            return sorted(self._items)

    def get(self, name: str, default: Any = None) -> Any:
        """返回一个文档，或在名称未知时返回 ``default``。

        参数:
            name: 条目名（即文件名主干，按字面匹配——不做大小写折叠，因为主键在
                区分大小写的文件系统上是真实文件名，折叠会让两个不同的文件相撞）。
            default: 名称未知时返回的值。

        返回:
            该文档，或 ``default``。

        异常:
            TypeError: 如果 ``name`` 不是字符串。

        示例:
            >>> import tempfile, pathlib
            >>> tmp = pathlib.Path(tempfile.mkdtemp()); _ = (tmp / "a.yaml").write_text("v: 1")
            >>> YamlStore(tmp).get("nope", default={}) == {}
            True
        """
        if not isinstance(name, str):
            raise TypeError(f"name must be a str, got {type(name).__name__}: {name!r}")
        with self._lock:
            return self._items.get(name, default)

    def __getitem__(self, name: str) -> Any:
        """返回一个文档，在名称未知时抛出异常。

        错误中会列出可用的名称：未知的键几乎总是一个笔误或一个未部署的文件，而这份
        可选项列表能把两种情况都变成五秒钟就能修好的问题。

        参数:
            name: 条目名。

        返回:
            该文档。

        异常:
            KeyError: 如果 ``name`` 不在存储中。

        示例:
            >>> import tempfile, pathlib
            >>> tmp = pathlib.Path(tempfile.mkdtemp()); _ = (tmp / "a.yaml").write_text("v: 1")
            >>> YamlStore(tmp)["a"]
            {'v': 1}
        """
        if not isinstance(name, str):
            raise TypeError(f"name must be a str, got {type(name).__name__}: {name!r}")
        with self._lock:
            if name not in self._items:
                available = sorted(self._items)
                raise KeyError(
                    f"unknown entry {name!r} in {self.directory}; available: {available}"
                )
            return self._items[name]

    def __contains__(self, name: object) -> bool:
        """``name`` 是否是一个已加载的条目。

        参数:
            name: 候选条目名。非字符串一律视为不存在。

        返回:
            当名称存在时返回 ``True``。

        示例:
            >>> import tempfile, pathlib
            >>> tmp = pathlib.Path(tempfile.mkdtemp()); _ = (tmp / "a.yaml").write_text("{}")
            >>> store = YamlStore(tmp); ("a" in store, 3 in store)
            (True, False)
        """
        if not isinstance(name, str):
            return False
        with self._lock:
            return name in self._items

    def __iter__(self) -> Any:
        """遍历排序后的条目名。

        遍历一份快照可使循环免受并发的 :meth:`reload` 影响（在遍历字典的同时改动
        它会抛出异常）。

        返回:
            一个遍历键的迭代器。

        示例:
            >>> import tempfile, pathlib
            >>> tmp = pathlib.Path(tempfile.mkdtemp()); _ = (tmp / "a.yaml").write_text("{}")
            >>> list(YamlStore(tmp))
            ['a']
        """
        with self._lock:
            return iter(sorted(self._items))

    def __len__(self) -> int:
        """返回已加载条目的数量。

        返回:
            条目数量。

        示例:
            >>> import tempfile, pathlib
            >>> tmp = pathlib.Path(tempfile.mkdtemp()); _ = (tmp / "a.yaml").write_text("{}")
            >>> len(YamlStore(tmp))
            1
        """
        with self._lock:
            return len(self._items)

    def __repr__(self) -> str:
        """返回一个调试用的表示。

        返回:
            一个包含目录、glob 与已加载键的字符串。

        示例:
            >>> import tempfile, pathlib
            >>> tmp = pathlib.Path(tempfile.mkdtemp()); _ = (tmp / "a.yaml").write_text("{}")
            >>> "keys=['a']" in repr(YamlStore(tmp))
            True
        """
        with self._lock:
            return (
                f"YamlStore(directory={str(self.directory)!r}, pattern={self.pattern!r}, "
                f"keys={sorted(self._items)!r})"
            )
