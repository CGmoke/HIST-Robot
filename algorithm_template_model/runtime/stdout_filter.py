"""过滤 stdout/stderr 流并静音噪声日志记录器。

长时间运行的机器人会打印大量内容。其中大部分是有用的，但某些子系统会在每个
控制周期向终端刷屏（性能计时、参数转储、"initialized" 消息）。本模块提供两种
相互独立的机制，在不改动输出代码的前提下抑制这些噪声：

* :class:`FilteredStream` 是一个类文件包装器，会丢弃任何包含指定子串之一的行，
  并可选地吞掉被丢弃行之后的那一空行。把它安装到 ``sys.stdout`` 或
  ``sys.stderr`` 上，噪声就在接收端消失了。
* :func:`quiet_loggers` 将一组指定的 :mod:`logging` 日志记录器降低到某个阈值
  级别，用于处理流过滤无法看到的、基于 ``logging`` 的噪声。

:func:`install_filter` 会把 :class:`FilteredStream` 安装为 ``sys.stdout``
和/或 ``sys.stderr`` 并返回它，这样调用方之后可以通过赋值恢复原始流。

依赖：仅标准库。
"""

from __future__ import annotations

import logging
import sys
from typing import Iterable, TextIO

logger = logging.getLogger(__name__)

__all__ = ["FilteredStream", "install_filter", "quiet_loggers"]


class FilteredStream:
    """类文件流包装器，丢弃匹配禁用子串的行。

    该包装器会缓冲不完整的写入，并在每次遇到换行时按整行判断，正如终端的缓冲
    行为。没有换行的文本会由 :meth:`flush` 转发。非字符串写入会用 ``str()`` 转换，
    因此 ``print(1)`` 仍然可用。

    参数:
        drop_substrings: 包含其中任一子串的行都会被丢弃。可以是空集合（此时不丢弃
            任何行），但不能为 ``None``。
        stream: 要包装的流；默认为 ``sys.stdout``。
        drop_blank: 为 ``True`` 时，还会丢弃紧随被丢弃行之后的第一个*空*行。这样可以
            清理许多刷屏调用方在其内容之后打印的空行。
        marker_attr: 打在包装器上的属性名，使得在同一流上第二次调用
            :meth:`install_filter` 成为空操作。

    异常:
        TypeError: 当 ``drop_substrings`` 为 ``None`` 时。
    """

    def __init__(
        self,
        drop_substrings: Iterable[str],
        *,
        stream: TextIO | None = None,
        drop_blank: bool = True,
        marker_attr: str = "_use_filtered_stream",
    ) -> None:
        if drop_substrings is None:
            raise TypeError("drop_substrings must be an iterable of strings, got None")
        self.drop_substrings: tuple[str, ...] = tuple(drop_substrings)
        self.stream: TextIO = stream if stream is not None else sys.stdout
        self.drop_blank = drop_blank
        self.marker_attr = marker_attr
        self._buf = ""
        self._pending_blank = False
        setattr(self.stream, marker_attr, True)

    @property
    def installed(self) -> bool:
        """当此包装器当前是某个标记背后的流时返回 ``True``。"""
        return bool(getattr(self.stream, self.marker_attr, False))

    def _should_drop(self, text: str) -> bool:
        return any(k in text for k in self.drop_substrings)

    def write(self, s: str) -> int:
        if not isinstance(s, str):
            s = str(s)
        self._buf += s
        out = 0
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            text = line.rstrip("\r")
            if self._should_drop(text):
                self._pending_blank = True
                out += len(line) + 1
                continue
            if self.drop_blank and self._pending_blank and not text.strip():
                out += len(line) + 1
                continue
            self._pending_blank = False
            out += self.stream.write(line + "\n")
        return out

    def flush(self) -> None:
        if self._buf:
            text = self._buf
            self._buf = ""
            if not self._should_drop(text):
                if not (self.drop_blank and self._pending_blank and not text.strip()):
                    self.stream.write(text)
        self.stream.flush()

    def fileno(self) -> int:
        """转发被包装流的文件描述符（用于 selector）。"""
        return self.stream.fileno()

    def isatty(self) -> bool:
        try:
            return self.stream.isatty()
        except Exception:  # noqa: BLE001 -- 已分离的流返回 False
            return False


def install_filter(
    drop_substrings: Iterable[str],
    *,
    streams: str = "stdout",
    drop_blank: bool = True,
) -> dict[str, FilteredStream | None]:
    """把 :class:`FilteredStream` 包装器安装到 ``sys.stdout``/``sys.stderr`` 上。

    幂等：已带有标记属性的流不会被改动，因此从多个初始化路径调用本函数是安全的。

    参数:
        drop_substrings: 子串；任何包含其中之一的整行都会被丢弃。
        streams: 用逗号分隔的 ``"stdout"``/``"stderr"`` 子集，表示要包装哪些流。
            未知的名称会被忽略。
        drop_blank: 转发给 :class:`FilteredStream`。

    返回:
        从流名称到已安装包装器的映射；若该流已被包装或未被请求，则对应的值为
        ``None``。

    示例:
        >>> from io import StringIO
        >>> sink = StringIO()
        >>> w = FilteredStream(["spam"], stream=sink)
        >>> w.write("spam line\\nclean line\\n")
        21
        >>> sink.getvalue()
        'clean line\\n'
    """
    result: dict[str, FilteredStream | None] = {}
    for name in ("stdout", "stderr"):
        result[name] = None
        if name not in (streams or "").split(","):
            continue
        stream = getattr(sys, name)
        marker = "_use_filtered_stream"
        if isinstance(stream, FilteredStream) or getattr(stream, marker, False):
            continue
        wrapper = FilteredStream(
            drop_substrings, stream=stream, drop_blank=drop_blank, marker_attr=marker
        )
        setattr(sys, name, wrapper)
        result[name] = wrapper
    return result


def quiet_loggers(names: Iterable[str], *, level: int = logging.WARNING) -> int:
    """将每个匹配的日志记录器降低到 ``level`` 级别。

    ``names`` 可以包含精确的日志记录器名称或前缀；前缀匹配会静默覆盖之后导入的
    模块，这在框架于导入时按名称配置嵌套日志记录器时很有用。

    参数:
        names: 要静音的日志记录器名称或前缀。
        level: 目标级别，例如 ``logging.WARNING``。

    返回:
        级别实际被更改的日志记录器数量。

    示例:
        >>> quiet_loggers(["myapp"], level=logging.ERROR)
        0
    """
    if names is None:
        raise TypeError("names must be an iterable of strings, got None")
    prefixes: tuple[str, ...] = tuple(names)
    changed = 0
    for full_name in list(logging.Logger.manager.loggerDict):
        full_name = str(full_name)
        if any(full_name == p or full_name.startswith(p + ".") for p in prefixes):
            logger_obj = logging.getLogger(full_name)
            if logger_obj.level != level:
                logger_obj.setLevel(level)
                changed += 1
    return changed
