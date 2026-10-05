"""与传输方式无关的父/子桥接，用于调用到另一个不兼容的运行时。

何时需要它？
------------
有些依赖组合根本无法共存于同一个进程中：

* 一个为 Python 3.10 构建的 conda 环境和一个为 Python 3.12 构建的系统 ROS 2 安装
  ——两者的 C-ABI 不兼容，因此在同一个解释器中导入二者会段错误或在扩展加载阶段
  失败；
* 一个锁定了特定解释器构建（或只提供某个 Python 版本 wheel）的厂商 SDK，而应用
  的其余部分运行在另一个版本上；
* 硬隔离需求：脆弱的运行时应当能够自行崩溃，而不把主控制回路一起拖垮。

跨过这道边界去导入是不可能的，但*以流的方式跨边界发送命令*成本很低。本模块实现了
这一模式：父进程把外部运行时作为子进程启动，并向其 stdin 写入以换行分隔的文本；
运行在另一个解释器下的子进程读取这些行，并把它们重新发布到自己所拥有的任意原生
API 上（ROS 发布者、厂商 SDK 调用等）。父进程从不导入外部依赖，因此两个运行时保持
在各自的地址空间中。

这些设计规则源自对原始机器人代码库的调试：

1. 子进程*必须*在关停时发出最后一个 "stop" 帧。诸如底盘看门狗之类的消费者会锁存
   它最后看到的命令，所以如果桥接悄无声息地死掉，机器人会永远执行最后那个速度。
   因此 ``run_bridge_child`` 保证 ``on_exit()`` 在 ``finally`` 块中执行。
2. 父进程*必须*把 ``BrokenPipeError`` 视为 "链路断开"，而不是崩溃：它会把
   ``connected`` 置为 ``False`` 并返回 ``False``，使 20 Hz 的控制回路优雅降级，
   而不是在热路径中抛异常。
3. 最有用的诊断信息是启动时捕获的子进程 stderr（``ModuleNotFoundError``、缺失的
   ``setup.bash``、错误的 topic 名）。子进程 stderr 管道由后台线程抽干到一个有界
   缓冲区，并原样包含在 :class:`BridgeStartError` 中。

线协议
------
纯 UTF-8 文本行。一行 == 一帧。空行被忽略。等于哨兵值（默认 ``CLOSE``）的一帧会
终止子进程循环。这样可以只靠 ``tail -f`` / ``strace`` 就完成调试，并且不需要任何
组帧库。

示例
----
子进程脚本 ``ros_bridge_child.py``（在*外部*解释器下运行）::

    #!/usr/bin/python3.12
    import sys
    from reusable_model.runtime.subprocess_bridge import run_bridge_child

    def main() -> int:
        topic = sys.argv[1] if len(sys.argv) > 1 else "/cmd"
        import rclpy                                  # 外部运行时，延迟导入
        from rclpy.node import Node
        from geometry_msgs.msg import Twist

        rclpy.init(args=None)
        node = Node("bridge_child")
        pub = node.create_publisher(Twist, topic, 10)

        def publish(values: list[str]) -> None:
            if len(values) != 3:
                raise ValueError(f"expected 3 fields, got {values!r}")
            msg = Twist()
            msg.linear.x = float(values[0])
            msg.linear.y = float(values[1])
            msg.angular.z = float(values[2])
            pub.publish(msg)

        def stop() -> None:                          # 保证发出最后的零帧
            pub.publish(Twist())
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

        return run_bridge_child(publish, on_exit=stop)

    if __name__ == "__main__":
        raise SystemExit(main())

父进程侧（在*另一个*解释器中运行）::

    from reusable_model.runtime.subprocess_bridge import SubprocessBridge

    with SubprocessBridge(
        ["/usr/bin/python3.12", "ros_bridge_child.py", "/cmd"],
        line_format="{:.6f} {:.6f} {:.6f}\\n",
    ) as bridge:
        for _ in range(100):
            bridge.send(0.10, 0.0, 0.25)     # 链路断开时返回 False
        bridge.close(final_values=(0.0, 0.0, 0.0))   # 显式停止帧
"""

from __future__ import annotations

import json
import logging
import os
import string
import subprocess
import sys
import threading
import time
from collections import deque
from typing import Any, Callable, Iterable, Sequence, TextIO

logger = logging.getLogger(__name__)

__all__ = [
    "BridgeError",
    "BridgeStartError",
    "SubprocessBridge",
    "run_bridge_child",
    "DEFAULT_SENTINEL",
    "DEFAULT_LINE_FORMAT",
]

#: 通知子进程循环关停的帧。写入时会带上结尾换行。
DEFAULT_SENTINEL = "CLOSE\n"

#: 默认的三浮点帧布局（例如 ``vx vy wz`` 归一化速度）。
DEFAULT_LINE_FORMAT = "{:.6f} {:.6f} {:.6f}\n"

#: 为诊断而保留的 stderr 行数。
_STDERR_BUFFER_LINES = 200


class BridgeError(RuntimeError):
    """桥接生命周期失败的基类。

    专门设置一个类型，使调用方能够区分 "外部运行时从未启动起来" 与同一代码路径中
    抛出的其他无关 OS 错误。

    示例:
        >>> try:
        ...     raise BridgeError("link down")
        ... except BridgeError as exc:
        ...     isinstance(exc, RuntimeError)
        True
    """


class BridgeStartError(BridgeError):
    """当子进程在启动期间退出时抛出。

    它携带退出码和捕获的 stderr 文本，因为实践中只有子进程自身的 traceback（缺失
    的模块、未 source 的环境、错误的参数）才能解释失败原因——父进程侧除了 "管道
    关闭" 之外什么也看不到。

    参数:
        message: 人类可读的摘要。
        argv: 启动时使用的命令行。
        returncode: 子进程退出码；若它仍在运行则为 ``None``。
        stderr: 捕获的子进程 stderr（可能为空）。

    示例:
        >>> err = BridgeStartError("died", ["python3", "x.py"], 1, "ModuleNotFoundError")
        >>> err.returncode, "ModuleNotFound" in str(err)
        (1, True)
    """

    def __init__(
        self,
        message: str,
        argv: Sequence[str],
        returncode: int | None = None,
        stderr: str = "",
    ) -> None:
        detail = stderr.strip()
        full = f"{message} (argv={list(argv)!r}, returncode={returncode})"
        if detail:
            full = f"{full}\n--- child stderr ---\n{detail}"
        super().__init__(full)
        self.argv = list(argv)
        self.returncode = returncode
        self.stderr = stderr


def _formatter_arity(line_format: str) -> int:
    """统计 ``str.format`` 模板中位置替换字段的数量。

    自动字段编号（``"{} {}"``）按出现次数计数，这与 ``str.format`` 消费位置参数的
    方式一致。

    参数:
        line_format: 一个 ``str.format`` 风格的模板。

    返回:
        该模板消费的位置字段数量。

    异常:
        TypeError: 当 ``line_format`` 不是 ``str`` 时。
        ValueError: 当模板无法被 :mod:`string` 解析时。

    示例:
        >>> _formatter_arity("{:.6f} {:.6f}\\n")
        2
    """
    if not isinstance(line_format, str):
        raise TypeError(f"line_format must be a str or callable, got {type(line_format).__name__}")
    count = 0
    try:
        for _literal, field_name, _spec, _conv in string.Formatter().parse(line_format):
            if field_name is not None:
                count += 1
    except (ValueError, KeyError) as exc:
        raise ValueError(f"invalid line_format {line_format!r}: {exc}") from exc
    return count


def _ensure_newline(text: str) -> str:
    """返回以恰好一个换行结尾的 ``text``。

    参数:
        text: 不涉及组帧问题的帧载荷。

    返回:
        带有一个结尾 ``"\\n"`` 的 ``text``。

    示例:
        >>> _ensure_newline("CLOSE")
        'CLOSE\\n'
    """
    return text if text.endswith("\n") else text + "\n"


class SubprocessBridge:
    """到外部运行时的、以换行分隔 stdin 桥接的父进程侧。

    该桥接拥有一个 :class:`subprocess.Popen`，其 stdin 是行缓冲的文本管道。写入在
    :class:`threading.Lock` 保护下进行，这样遥测线程和控制线程都能发布数据而不会
    交错出半帧——子进程会把撕裂的行静默地错误解析，这远比丢帧更糟。

    ``stdout`` 被有意重定向到 ``DEVNULL``：该通道是单向的，父进程绝不能阻塞在一个
    它不去服务的读取端上。``stderr`` 保留为管道，由守护线程抽干到一个有界 deque；
    若没有这个抽干线程，爱说话的子进程会填满 64 KiB 的管道缓冲区并造成死锁。

    格式化器在构造时即被校验。``str`` 模板具有固定的位置参数元数，因此 ``send`` 能在
    把垃圾写上线缆*之前*拒绝数量不匹配的值。当元数动态变化时可接受可调用格式化器
    （``(*values) -> str``）；它必须返回 ``str``，并自行负责结尾换行（缺失时会追加
    一个）。

    参数:
        argv: 子进程的命令行，例如 ``["/usr/bin/python3", "child.py"]``。必须是非空的
            ``str`` / :class:`os.PathLike` 序列。
        sentinel: 通知子进程关停的行。缺失结尾换行时会追加一个。
        line_format: 供 :meth:`send` 使用的 ``str.format`` 模板，或一个可调用对象
            ``(*values) -> str``。
        env: 子进程的环境。``None`` 表示继承父进程的环境。当子进程*不应*看到父进程的
            ``PYTHONPATH``/``LD_LIBRARY_PATH`` 时（这往往正是两个运行时不兼容的根源），
            传入一个显式映射。
        cwd: 子进程的工作目录；``None`` 表示继承。
        startup_timeout: :meth:`start` 用来证明子进程没有立即死亡的等待秒数。这个等待
            是有意为之：导入沉重外部栈的子进程会*较晚*失败（在数百毫秒之后），只有
            等待才能捕获它的 traceback。需要快速路径时可传入一个小值。
        encode: 管道的文本编码（双向）。

    异常:
        TypeError: 当 ``argv`` 不是字符串/类路径序列，或 ``env``/``cwd``/``encode``
            类型错误时。
        ValueError: 当 ``argv`` 为空、``sentinel`` 为空白，或 ``line_format`` 无法
            解析时。

    示例:
        >>> import sys
        >>> child = [sys.executable, "-c", "import sys; list(sys.stdin)"]
        >>> bridge = SubprocessBridge(child, startup_timeout=0.2)
        >>> bridge.start()
        >>> bridge.send(0.1, 0.0, 0.25)
        True
        >>> bridge.close(final_values=(0.0, 0.0, 0.0)); bridge.connected
        False
    """

    def __init__(
        self,
        argv: Sequence[str | os.PathLike[str]],
        *,
        sentinel: str = DEFAULT_SENTINEL,
        line_format: str | Callable[..., str] = DEFAULT_LINE_FORMAT,
        env: dict[str, str] | None = None,
        cwd: str | os.PathLike[str] | None = None,
        startup_timeout: float = 3.0,
        encode: str = "utf-8",
    ) -> None:
        self.argv: list[str] = self._validate_argv(argv)
        if not isinstance(sentinel, str):
            raise TypeError(f"sentinel must be a str, got {type(sentinel).__name__}")
        if not sentinel.strip():
            raise ValueError(f"sentinel must be a non-blank line, got {sentinel!r}")
        self.sentinel: str = _ensure_newline(sentinel)
        if not isinstance(encode, str) or not encode:
            raise ValueError(f"encode must be a non-empty codec name, got {encode!r}")
        self.encode = encode
        if env is not None and not isinstance(env, dict):
            raise TypeError(f"env must be a dict or None, got {type(env).__name__}")
        self.env = env
        self.cwd = str(cwd) if cwd is not None else None
        if not isinstance(startup_timeout, (int, float)) or isinstance(startup_timeout, bool):
            raise TypeError(
                f"startup_timeout must be a number, got {type(startup_timeout).__name__}"
            )
        if startup_timeout < 0:
            raise ValueError(f"startup_timeout must be >= 0, got {startup_timeout!r}")
        self.startup_timeout = float(startup_timeout)

        self._formatter: str | Callable[..., str]
        self._arity: int | None
        if callable(line_format):
            self._formatter = line_format
            self._arity = None
        else:
            self._arity = _formatter_arity(line_format)
            if self._arity == 0:
                raise ValueError(
                    f"line_format {line_format!r} has no replacement fields; "
                    "pass a callable formatter for dynamic frames"
                )
            self._formatter = line_format

        self._proc: subprocess.Popen[str] | None = None
        self._lock = threading.Lock()
        self._connected = False
        self._closed = False
        self._stderr_lines: deque[str] = deque(maxlen=_STDERR_BUFFER_LINES)
        self._stderr_thread: threading.Thread | None = None

    # ------------------------------------------------------------------ 辅助方法

    @staticmethod
    def _validate_argv(argv: Sequence[str | os.PathLike[str]]) -> list[str]:
        """规范化并校验子进程命令行。

        参数:
            argv: 候选命令行。

        返回:
            以 ``str`` 列表形式给出的命令行。

        异常:
            TypeError: 当 ``argv`` 是裸字符串、不可迭代，或含有非字符串/非路径元素时。
            ValueError: 当 ``argv`` 为空时。

        示例:
            >>> SubprocessBridge._validate_argv(["/bin/echo", "hi"])
            ['/bin/echo', 'hi']
        """
        if isinstance(argv, (str, bytes)) or isinstance(argv, os.PathLike):
            raise TypeError(
                f"argv must be a sequence of arguments, got a single "
                f"{type(argv).__name__}: {argv!r}"
            )
        if not isinstance(argv, Iterable):
            raise TypeError(f"argv must be a sequence, got {type(argv).__name__}")
        items = [os.fspath(a) if isinstance(a, os.PathLike) else a for a in argv]
        if not items:
            raise ValueError("argv must not be empty")
        for item in items:
            if not isinstance(item, str):
                raise TypeError(
                    f"argv entries must be str or os.PathLike, got "
                    f"{type(item).__name__}: {item!r}"
                )
        return items

    def _drain_stderr(self, stream: TextIO) -> None:
        """把子进程 stderr 复制到有界的诊断缓冲区。

        运行在守护线程上，因此爱输出内容的子进程绝不会填满 OS 管道并把自己阻塞住。
        各行会被保留（最新的在最后）供 :attr:`stderr_tail` 使用。

        参数:
            stream: 子进程的 stderr 文本流。

        示例:
            不直接调用；参见 :meth:`start`。
        """
        try:
            for line in stream:
                self._stderr_lines.append(line.rstrip("\n"))
        except (OSError, ValueError):
            # 管道已被 close()/kill() 关闭；没有可上报的内容了。
            pass

    # ------------------------------------------------------------------- 状态

    @property
    def connected(self) -> bool:
        """子进程是否被认定存活且可写。

        返回:
            仅当进程已启动、尚未退出，且没有任何写入以 ``BrokenPipeError`` 失败时，
            才为 ``True``。

        示例:
            >>> SubprocessBridge(["/bin/true"]).connected
            False
        """
        return self._connected

    @property
    def pid(self) -> int | None:
        """子进程的 PID；若从未启动过则为 ``None``。

        示例:
            >>> SubprocessBridge(["/bin/true"]).pid is None
            True
        """
        return self._proc.pid if self._proc is not None else None

    @property
    def returncode(self) -> int | None:
        """子进程的退出码；若它正在运行/尚未启动则为 ``None``。

        示例:
            >>> SubprocessBridge(["/bin/true"]).returncode is None
            True
        """
        return self._proc.returncode if self._proc is not None else None

    def stderr_tail(self, limit: int = 40) -> str:
        """返回最近捕获的子进程 stderr 行。

        参数:
            limit: 要返回的末尾行数上限。

        返回:
            拼接后的 stderr 尾部（子进程没有输出时为空）。

        异常:
            ValueError: 当 ``limit`` 不是正数时。

        示例:
            >>> SubprocessBridge(["/bin/true"]).stderr_tail()
            ''
        """
        if limit <= 0:
            raise ValueError(f"limit must be positive, got {limit!r}")
        return "\n".join(list(self._stderr_lines)[-limit:])

    def poll(self) -> int | None:
        """对子进程存活状态进行非阻塞检查。

        同时会更新 :attr:`connected`：一旦子进程已退出，桥接即处于断开状态，后续
        :meth:`send` 调用会短路返回 ``False``。

        返回:
            子进程的退出码；若它仍在运行（或从未启动）则为 ``None``。

        示例:
            >>> SubprocessBridge(["/bin/true"]).poll() is None
            True
        """
        if self._proc is None:
            return None
        code = self._proc.poll()
        if code is not None and self._connected:
            self._connected = False
            logger.warning("bridge child exited with code %s", code)
        return code

    # ------------------------------------------------------------------- 启动

    def start(self) -> None:
        """启动子进程并阻塞直到证明它存活。

        该方法最多等待 ``startup_timeout`` 秒。若子进程在该窗口内退出，则抛出
        :class:`BridgeStartError` 并带上捕获的 stderr——那份 traceback 几乎总是真正的
        答案（外部解释器中缺失的依赖、未 source 的 ROS 环境、错误的参数）。

        返回:
            ``None``。

        异常:
            BridgeStartError: 子进程在启动窗口内退出，或 OS 拒绝启动它时。捕获的
                stderr 包含在消息中。
            ValueError: 桥接已被关闭时。

        示例:
            >>> import sys
            >>> child = [sys.executable, "-c", "import sys; list(sys.stdin)"]
            >>> b = SubprocessBridge(child, startup_timeout=0.2)
            >>> b.start(); b.connected
            True
        """
        if self._closed:
            raise ValueError("bridge is closed and cannot be restarted")
        if self._proc is not None and self._proc.poll() is None:
            self._connected = True
            return

        try:
            proc = subprocess.Popen(  # noqa: S603 - argv 由调用方提供
                self.argv,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                encoding=self.encode,
                env=self.env,
                cwd=self.cwd,
            )
        except OSError as exc:
            raise BridgeStartError(
                f"failed to spawn bridge child: {exc}", self.argv, None, ""
            ) from exc

        self._proc = proc
        if proc.stderr is not None:
            self._stderr_thread = threading.Thread(
                target=self._drain_stderr,
                args=(proc.stderr,),
                name="subprocess-bridge-stderr",
                daemon=True,
            )
            self._stderr_thread.start()

        try:
            code = proc.wait(timeout=self.startup_timeout)
        except subprocess.TimeoutExpired:
            code = None
        if code is not None:
            if self._stderr_thread is not None:
                self._stderr_thread.join(timeout=1.0)
            stderr = self.stderr_tail()
            self._proc = None
            raise BridgeStartError(
                f"bridge child exited immediately with code {code}",
                self.argv,
                code,
                stderr,
            )

        self._connected = True
        logger.info("bridge ready (pid=%s, argv=%s)", proc.pid, self.argv)

    def connect(self, timeout: float | None = None) -> bool:
        """:meth:`start` 的不抛异常变体，供控制回路使用。

        参数:
            timeout: 本次尝试覆盖 ``startup_timeout``。``None`` 表示沿用构造函数中的
                值。

        返回:
            子进程启动成功时为 ``True``，否则为 ``False``。原因（包括子进程的 stderr）
            会以 ``ERROR`` 级别记录日志，并可通过 :meth:`stderr_tail` 获取。

        异常:
            TypeError: 当 ``timeout`` 既不是 ``None`` 也不是数字时。
            ValueError: 当 ``timeout`` 为负数时。

        示例:
            >>> import sys
            >>> child = [sys.executable, "-c", "import sys; list(sys.stdin)"]
            >>> b = SubprocessBridge(child, startup_timeout=0.2)
            >>> b.connect(timeout=0.2)
            True
        """
        if timeout is not None:
            if not isinstance(timeout, (int, float)) or isinstance(timeout, bool):
                raise TypeError(f"timeout must be a number or None, got {timeout!r}")
            if timeout < 0:
                raise ValueError(f"timeout must be >= 0, got {timeout!r}")
            saved, self.startup_timeout = self.startup_timeout, float(timeout)
        else:
            saved = None
        try:
            self.start()
            return True
        except BridgeStartError as exc:
            logger.error("bridge connect failed: %s", exc)
            return False
        finally:
            if saved is not None:
                self.startup_timeout = saved

    # -------------------------------------------------------------------- 发送

    def _format(self, values: tuple[Any, ...]) -> str:
        """根据位置参数渲染出一帧。

        参数:
            values: 传给 :meth:`send` 的参数。

        返回:
            以换行结尾的帧文本。

        异常:
            ValueError: 当配置的是 ``str`` 格式化器而值的数量与其元数不匹配时。
            TypeError: 当可调用格式化器返回的不是 ``str`` 时。

        示例:
            >>> b = SubprocessBridge(["/bin/true"])
            >>> b._format((1.0, 2.0, 3.0)).strip()
            '1.000000 2.000000 3.000000'
        """
        formatter = self._formatter
        if callable(formatter):
            text = formatter(*values)
            if not isinstance(text, str):
                raise TypeError(
                    f"line_format callable must return str, got {type(text).__name__}"
                )
        else:
            if self._arity is not None and len(values) != self._arity:
                raise ValueError(
                    f"line_format {formatter!r} expects {self._arity} value(s), "
                    f"got {len(values)}: {values!r}"
                )
            try:
                text = formatter.format(*values)
            except (IndexError, KeyError, ValueError) as exc:
                raise ValueError(
                    f"cannot format {values!r} with {formatter!r}: {exc}"
                ) from exc
        return _ensure_newline(text)

    def _write(self, text: str) -> bool:
        """在发送锁保护下写入并刷新一帧。

        参数:
            text: 以换行结尾的帧。

        返回:
            成功时为 ``True``，链路断开时为 ``False``。

        示例:
            不直接调用；请使用 :meth:`send` / :meth:`send_line`。
        """
        proc = self._proc
        if not self._connected or proc is None or proc.stdin is None:
            return False
        if proc.poll() is not None:
            self._connected = False
            logger.warning("bridge child has exited; dropping frame")
            return False
        with self._lock:
            try:
                proc.stdin.write(text)
                proc.stdin.flush()
            except BrokenPipeError:
                self._connected = False
                logger.warning("bridge stdin is broken; marking link down")
                return False
            except (OSError, ValueError) as exc:
                # ValueError：写入一个已被并发 close() 关闭的流。
                self._connected = False
                logger.warning("bridge write failed (%s); marking link down", exc)
                return False
        return True

    def send(self, *values: Any) -> bool:
        """把 ``values`` 格式化为一个帧并发布出去。

        对已断开的链路绝不抛异常：控制回路应当轮询返回值，并退回到自己的安全行为。

        参数:
            *values: 由 ``line_format`` 消费的位置参数。

        返回:
            帧被写入并刷新时为 ``True``；若桥接未连接或管道已断开则为 ``False``。

        异常:
            ValueError: 当值的数量与 ``str`` 格式化器的元数不匹配，或这些值无法被
                格式化时。

        示例:
            >>> b = SubprocessBridge(["/bin/true"])
            >>> b.send(0.1, 0.2, 0.3)   # never started
            False
        """
        return self._write(self._format(values))

    def send_line(self, text: str) -> bool:
        """发布一行已格式化好的文本（绕过 ``line_format``）。

        参数:
            text: 帧载荷。缺失时会追加一个结尾换行。

        返回:
            写入成功时为 ``True``；链路断开时为 ``False``。

        异常:
            TypeError: 当 ``text`` 不是 ``str`` 时。
            ValueError: 当 ``text`` 含有内嵌换行时（那会注入第二个帧并使协议失去
                同步）。

        示例:
            >>> b = SubprocessBridge(["/bin/true"])
            >>> b.send_line("0 0 0")
            False
        """
        if not isinstance(text, str):
            raise TypeError(f"text must be a str, got {type(text).__name__}")
        if "\n" in text.rstrip("\n"):
            raise ValueError(f"send_line takes exactly one frame, got {text!r}")
        return self._write(_ensure_newline(text))

    def send_json(self, obj: Any) -> bool:
        """把一个 JSON 对象作为单独一行帧发布。

        使用紧凑分隔符，使载荷中绝不可能含有换行，从而对结构化数据也保持 "一行 ==
        一帧" 的约定。

        参数:
            obj: 任意可 JSON 序列化的对象。

        返回:
            写入成功时为 ``True``；链路断开时为 ``False``。

        异常:
            TypeError: 当 ``obj`` 不可 JSON 序列化时。

        示例:
            >>> b = SubprocessBridge(["/bin/true"])
            >>> b.send_json({"vx": 0.1})
            False
        """
        try:
            text = json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
        except TypeError as exc:
            raise TypeError(f"send_json payload is not JSON serialisable: {exc}") from exc
        return self._write(text + "\n")

    # ------------------------------------------------------------------- 关闭

    def close(self, *, final_values: Sequence[Any] | None = None, wait_timeout: float = 2.0) -> None:
        """关闭桥接：可选的停止帧、哨兵、关闭 stdin、等待。

        顺序很重要。最后那个帧给消费者一个显式的 "stop"（这样看门狗就不会锁存最后
        一个运动命令），哨兵让子进程运行自己的清理逻辑，而不是在发布中途被杀掉，之后
        才关闭 stdin 并回收进程。当子进程无视这一切时，``kill()`` 是最后手段。

        该方法幂等且异常安全：它可以从信号处理器、``finally`` 块中被调用，或连续调用
        两次，并且对已断开的链路绝不抛异常。

        参数:
            final_values: 在哨兵之前进行最后一次 :meth:`send` 所用的值，例如速度桥接的
                ``(0.0, 0.0, 0.0)``。``None`` 表示跳过。
            wait_timeout: 在杀掉子进程之前等待其干净退出的秒数。

        返回:
            ``None``。

        异常:
            TypeError: 当给出了 ``final_values`` 但它不是序列时。
            ValueError: 当 ``wait_timeout`` 为负数时。

        示例:
            >>> import sys
            >>> child = [sys.executable, "-c", "import sys; list(sys.stdin)"]
            >>> b = SubprocessBridge(child, startup_timeout=0.2)
            >>> b.start(); b.close(final_values=(0.0, 0.0, 0.0)); b.connected
            False
        """
        if final_values is not None:
            if isinstance(final_values, (str, bytes)) or not isinstance(final_values, Iterable):
                raise TypeError(
                    f"final_values must be a sequence or None, got "
                    f"{type(final_values).__name__}"
                )
            final_values = tuple(final_values)
        if not isinstance(wait_timeout, (int, float)) or isinstance(wait_timeout, bool):
            raise TypeError(f"wait_timeout must be a number, got {wait_timeout!r}")
        if wait_timeout < 0:
            raise ValueError(f"wait_timeout must be >= 0, got {wait_timeout!r}")

        if self._closed:
            return
        self._closed = True

        proc = self._proc
        if proc is None:
            self._connected = False
            return

        # 最后那个帧必须在链路仍被视为连通时写入，因此 ``_connected`` 之后才清除。
        # 顺序至关重要：消费者的看门狗依赖这是它见到的最后一个运动帧。
        if final_values is not None:
            try:
                self.send(*final_values)
            except Exception as exc:  # noqa: BLE001 - 关停流程绝不能抛异常
                logger.debug("final frame failed: %s", exc)
        self._connected = False

        with self._lock:
            stdin = proc.stdin
            if stdin is not None:
                try:
                    if proc.poll() is None:
                        stdin.write(self.sentinel)
                        stdin.flush()
                except (BrokenPipeError, OSError, ValueError) as exc:
                    logger.debug("sentinel write failed: %s", exc)
                try:
                    stdin.close()
                except (OSError, ValueError) as exc:
                    logger.debug("stdin close failed: %s", exc)

        try:
            proc.wait(timeout=wait_timeout)
        except subprocess.TimeoutExpired:
            logger.warning(
                "bridge child did not exit within %.1fs; killing pid=%s",
                wait_timeout,
                proc.pid,
            )
            try:
                proc.kill()
                proc.wait(timeout=1.0)
            except (OSError, subprocess.TimeoutExpired) as exc:
                logger.warning("failed to reap bridge child pid=%s: %s", proc.pid, exc)

        if self._stderr_thread is not None:
            self._stderr_thread.join(timeout=0.5)
            self._stderr_thread = None
        logger.debug("bridge closed (returncode=%s)", proc.returncode)

    # ------------------------------------------------------------ 上下文管理器

    def __enter__(self) -> SubprocessBridge:
        """启动桥接，子进程起不来时快速失败。

        返回:
            ``self``，且已连接。

        异常:
            BridgeStartError: 从 :meth:`start` 透传，因此 ``with`` 块绝不会在链路已死
                的情况下静默运行。

        示例:
            >>> import sys
            >>> child = [sys.executable, "-c", "import sys; list(sys.stdin)"]
            >>> with SubprocessBridge(child, startup_timeout=0.2) as b:
            ...     _ = b.connected
        """
        self.start()
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        """关闭桥接，仅在干净退出时发送零帧。

        发生异常时跳过最后那个帧：调用方已经在展开栈，哨兵加上 ``kill()`` 兜底就足够
        了。子进程的 ``on_exit`` 仍会发出它自己的停止帧，这正是看门狗所依赖的保证。

        参数:
            exc_type: 异常类型，或 ``None``。
            exc: 异常实例，或 ``None``。
            tb: traceback，或 ``None``。

        返回:
            ``None``（异常绝不会被吞掉）。

        示例:
            参见 :meth:`__enter__`。
        """
        self.close(wait_timeout=self.startup_timeout if self.startup_timeout else 2.0)

    def __repr__(self) -> str:
        """返回调试用的表示。

        返回:
            包含 argv、pid 和连接状态的字符串。

        示例:
            >>> repr(SubprocessBridge(["/bin/true"]))
            "SubprocessBridge(argv=['/bin/true'], pid=None, connected=False)"
        """
        return (
            f"SubprocessBridge(argv={self.argv!r}, pid={self.pid!r}, "
            f"connected={self._connected})"
        )


def run_bridge_child(
    handler: Callable[[list[str]], Any],
    *,
    sentinel: str = "CLOSE",
    on_exit: Callable[[], Any] | None = None,
    stream: TextIO | None = None,
) -> int:
    """子进程侧循环：从 stdin 读取帧并把它们分派给 ``handler``。

    这是 :class:`SubprocessBridge` 的镜像。它运行在*外部*解释器下，因此绝不能从父进程
    的环境中导入任何东西。

    为什么 ``on_exit`` 是无条件的：重新发布的流的消费者通常由看门狗保护，而看门狗会
    锁存它收到的最后一帧。如果子进程只是返回——在 EOF、异常或 ``Ctrl-C`` 时——最后的
    运动命令会一直保持锁存，硬件会持续执行它。在 ``finally`` 块中发出一个最终的零帧/
    刷新帧，正是让桥接死亡变得安全的原因。

    handler 抛出的异常会按行捕获并记录日志：一个畸形的帧（撕裂的写入、手输的测试行）
    绝不能拖垮一个长期存在的桥接。只有哨兵、EOF 或 ``KeyboardInterrupt`` 会结束循环。

    参数:
        handler: 每帧调用一次，参数是按空白切分后的字段，例如
            ``["0.100000", "0.000000"]``。字段解析和范围检查属于这里。
        sentinel: 结束循环的行（不含换行）。与 strip 后的行比较，因此容忍来自 CRLF
            生产者的结尾 ``\\r``。
        on_exit: 清理回调，无论循环如何终止，都会在 ``finally`` 块中恰好调用一次。
            用它来发布最终的停止帧。
        stream: 要读取的输入流；默认为 :data:`sys.stdin`。为测试而暴露。

    返回:
        干净关停时为 ``0``，循环被中断时为 ``1``。

    异常:
        TypeError: 当 ``handler`` 不可调用，或 ``sentinel``/``stream`` 类型错误时。
        ValueError: 当 ``sentinel`` 为空白时。

    示例:
        >>> import io
        >>> seen: list[list[str]] = []
        >>> flushed: list[bool] = []
        >>> run_bridge_child(seen.append, sentinel="CLOSE",
        ...                  on_exit=lambda: flushed.append(True),
        ...                  stream=io.StringIO("0.1 0.2\\nbad line\\nCLOSE\\n"))
        0
        >>> seen, flushed
        ([['0.1', '0.2'], ['bad', 'line']], [True])
    """
    if not callable(handler):
        raise TypeError(f"handler must be callable, got {type(handler).__name__}")
    if not isinstance(sentinel, str):
        raise TypeError(f"sentinel must be a str, got {type(sentinel).__name__}")
    if not sentinel.strip():
        raise ValueError(f"sentinel must be a non-blank line, got {sentinel!r}")
    source: TextIO = sys.stdin if stream is None else stream
    if not hasattr(source, "__iter__"):
        raise TypeError(f"stream must be an iterable text stream, got {type(source).__name__}")

    want = sentinel.strip()
    interrupted = False
    try:
        for raw in source:
            line = raw.strip()
            if not line:
                continue
            if line == want:
                logger.debug("bridge child received sentinel %r", want)
                break
            try:
                handler(line.split())
            except Exception:  # noqa: BLE001 - 一个坏帧绝不能杀死桥接
                logger.exception("bridge child handler failed for line %r", line)
    except KeyboardInterrupt:
        interrupted = True
        logger.warning("bridge child interrupted")
    except (OSError, ValueError) as exc:
        # ValueError：stdin 被父进程的 close() 从下方关闭了。
        interrupted = True
        logger.warning("bridge child input ended abnormally: %s", exc)
    finally:
        if on_exit is not None:
            if not callable(on_exit):
                raise TypeError(f"on_exit must be callable or None, got {on_exit!r}")
            try:
                on_exit()
            except Exception:  # noqa: BLE001 - 清理不得掩盖关停过程
                logger.exception("bridge child on_exit failed")
    return 1 if interrupted else 0


def _selftest(argv_hint: str = "python3 -m reusable_model.runtime.subprocess_bridge") -> int:
    """用作模块 ``__main__`` 入口的往返冒烟测试。

    参数:
        argv_hint: 未使用；保留以维持稳定的签名。

    返回:
        进程退出码（成功时为 ``0``）。

    示例:
        不直接调用；把模块当脚本运行即可。
    """
    del argv_hint
    logging.basicConfig(level=logging.INFO)
    child_src = (
        "import sys\n"
        "for line in sys.stdin:\n"
        "    line = line.strip()\n"
        "    if not line:\n"
        "        continue\n"
        "    if line == 'CLOSE':\n"
        "        break\n"
        "sys.stderr.write('child done\\n')\n"
    )
    bridge = SubprocessBridge(
        [sys.executable, "-c", child_src], startup_timeout=2.0, line_format="{:.3f} {:.3f}\n"
    )
    bridge.start()
    for i in range(5):
        ok = bridge.send(i * 0.1, -i * 0.1)
        logger.info("frame %d accepted=%s", i, ok)
        time.sleep(0.01)
    bridge.close(final_values=(0.0, 0.0))
    logger.info("child stderr: %s", bridge.stderr_tail())
    return 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
