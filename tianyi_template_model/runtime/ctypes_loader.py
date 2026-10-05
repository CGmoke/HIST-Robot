"""通过 :mod:`ctypes` 从 Python 安全加载原生共享库。

当一个 Python 进程必须驱动 C++ 厂商 SDK 时，总是一再出现三个问题，本模块的存在就是
为了把它们一次性解决：

1. **同名的陈旧库赢得了查找。** ``dlopen`` 会依据*加载器搜索路径*
   （``LD_LIBRARY_PATH``、``RUNPATH``、默认目录）按 *soname* 解析目标的 ``DT_NEEDED``
   条目。如果依赖的旧副本位于该路径中更靠前的位置——被 source 的厂商 ``setup.bash``
   是经典元凶——垫片就会绑定到旧库，并在运行时以令人困惑的 "undefined symbol" 或
   "function returned -1" 失败。修复办法是在加载垫片**之前**，用绝对路径对*已知良好*
   的依赖执行 ``CDLL(dep, mode=RTLD_GLOBAL)``：glibc 随后会复用已加载的 soname，而
   搜索路径根本不会被查询。:func:`load_library` 通过 ``preload`` 完成这件事。
2. **ctypes 无法调用 C++。** 名称会被 mangle，类布局在不同编译器之间不具备 ABI 稳定
   性，而 ``std::function`` 回调根本无法跨越边界。答案是一层薄薄的 ``extern "C"``
   垫片；参见 :data:`EXTERN_C_SHIM_TEMPLATE` 和 :func:`build_shared_library`。
3. **``init`` 返回成功并不意味着 "就绪"。** 许多原生客户端会同步打开 socket，但在后台
   线程中完成认证/状态握手。真正意义上的第一条命令随后会以 "not ready" 或
   "no permission" 错误码失败。用一个空操作命令轮询直到成功，可以消除这个竞态；参见
   :func:`poll_until_ready`。

这里的一切都仅依赖标准库（``ctypes``、``subprocess``），因此在没有编译器或没有厂商
SDK 的机器上导入本模块绝不会失败。

示例
----
加载一个垫片及其依赖、声明签名并等待就绪::

    from reusable_model.runtime.ctypes_loader import (
        CFunction, declare, load_library, poll_until_ready,
    )
    import ctypes

    sdk = load_library("/opt/vendor/lib/libvendor_sdk.so", preload=None,
                       mode=ctypes.RTLD_GLOBAL)
    lib = load_library("/opt/vendor/lib/libvendor_shim.so",
                       preload=["/opt/vendor/lib/libvendor_sdk.so"],
                       mode=ctypes.RTLD_GLOBAL)

    init = CFunction(declare(lib.vn_init, [ctypes.c_char_p], ctypes.c_int),
                     ok_codes=(0,), error_map={-1: "socket connect failed"})
    send = CFunction(declare(lib.vn_send, [ctypes.c_double] * 3, ctypes.c_int),
                     ok_codes=(0,), error_map={-2008: "enable switch not pressed"})

    init(b"192.168.1.10")
    ready = poll_until_ready(lambda: send(0.0, 0.0, 0.0) == 0, timeout=10.0)
"""

from __future__ import annotations

import ctypes
import logging
import os
import re
import string
import subprocess
import time
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence

logger = logging.getLogger(__name__)

__all__ = [
    "NativeCallError",
    "NativeBuildError",
    "EXTERN_C_SHIM_TEMPLATE",
    "DEFAULT_LIBRARY_PATTERNS",
    "load_library",
    "declare",
    "CFunction",
    "find_library_file",
    "poll_until_ready",
    "render_c_shim",
    "build_shared_library",
]

#: 由 :func:`find_library_file` 按顺序尝试的 glob 模式。
DEFAULT_LIBRARY_PATTERNS: tuple[str, ...] = (
    "lib{name}.so",
    "{name}.so",
    "{name}*.so*",
)

_IDENTIFIER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_STD_RE = re.compile(r"^[A-Za-z+0-9]+$")

#: 在本进程中已用 ``RTLD_GLOBAL`` 加载过的路径，用于避免重复工作。
_PRELOADED: set[str] = set()


class NativeCallError(RuntimeError):
    """原生函数返回的码不在可接受集合内。

    厂商 SDK 绝大多数都采用 ``0 = 成功，负数 = 错误码`` 的约定，而忽略返回值会让
    "夹爪未使能" 变成一个静默的空操作。用 :class:`CFunction` 包装函数可以杜绝这一点：
    每个非成功的码都会变成异常，同时携带数字码（供程序化处理）和人类可读的映射说明。

    这里不内置任何厂商的错误码表——由调用方提供 ``error_map``。

    参数:
        code: 原生函数返回的数字值。
        message: 人类可读的解释。为空时会构造一条通用消息。
        function: 原生函数的名称，用于格式化后的消息。

    示例:
        >>> err = NativeCallError(-2008, "enable switch not pressed", function="vn_send")
        >>> err.code
        -2008
        >>> "vn_send" in str(err)
        True
    """

    def __init__(self, code: int, message: str = "", *, function: str = "") -> None:
        if not isinstance(code, int):
            raise TypeError(f"code must be an int, got {type(code).__name__}: {code!r}")
        self.code = code
        self.function = function
        self.message = message or f"native call failed with code {code}"
        prefix = f"{function}: " if function else ""
        super().__init__(f"{prefix}{self.message} (code={code})")


class NativeBuildError(RuntimeError):
    """共享库构建（编译器调用）失败。

    携带完整的命令行和编译器的 stderr，因为缺失的头文件或无法解析的 ``-l`` 标志只能
    从它们那里诊断出来。

    参数:
        message: 摘要文本。
        argv: 执行的编译器命令行。
        returncode: 编译器的退出状态。
        stderr: 捕获的编译器诊断信息。

    示例:
        >>> err = NativeBuildError("build failed", ["g++", "x.cpp"], 1, "no such file")
        >>> err.returncode
        1
    """

    def __init__(
        self,
        message: str,
        argv: Sequence[str],
        returncode: int,
        stderr: str = "",
    ) -> None:
        detail = (stderr or "").strip()
        full = f"{message} (returncode={returncode}, argv={list(argv)!r})"
        if detail:
            full = f"{full}\n--- compiler stderr ---\n{detail}"
        super().__init__(full)
        self.argv = list(argv)
        self.returncode = returncode
        self.stderr = stderr


def _as_path(value: Any, *, what: str) -> Path:
    """把 ``str`` / :class:`os.PathLike` 转换为 :class:`pathlib.Path`。

    参数:
        value: 候选路径。
        what: 用于错误消息中的名称。

    返回:
        以 ``Path`` 形式给出的值。

    异常:
        TypeError: 当 ``value`` 不是字符串或类路径对象时。

    示例:
        >>> _as_path("lib.so", what="path").name
        'lib.so'
    """
    if isinstance(value, Path):
        return value
    if isinstance(value, (str, os.PathLike)):
        return Path(os.fspath(value))
    raise TypeError(f"{what} must be a str or os.PathLike, got {type(value).__name__}: {value!r}")


def _as_str_sequence(value: Any, *, what: str) -> list[str]:
    """把 ``str``/类路径的序列转换为 ``str`` 列表。

    参数:
        value: ``None``（视为空）或一个非字符串的可迭代对象。
        what: 用于错误消息中的名称。

    返回:
        保持顺序的字符串列表。

    异常:
        TypeError: 当 ``value`` 是裸字符串、不可迭代，或含有错误类型的元素时。

    示例:
        >>> _as_str_sequence(["a", Path("b")], what="libs")
        ['a', 'b']
    """
    if value is None:
        return []
    if isinstance(value, (str, bytes, os.PathLike)):
        raise TypeError(f"{what} must be a sequence, got a single {type(value).__name__}: {value!r}")
    if not isinstance(value, Iterable):
        raise TypeError(f"{what} must be a sequence, got {type(value).__name__}")
    out: list[str] = []
    for item in value:
        if isinstance(item, os.PathLike):
            out.append(os.fspath(item))
        elif isinstance(item, str):
            out.append(item)
        else:
            raise TypeError(
                f"{what} entries must be str or os.PathLike, got "
                f"{type(item).__name__}: {item!r}"
            )
    return out


def load_library(
    path: str | os.PathLike[str],
    *,
    preload: Sequence[str | os.PathLike[str]] | None = None,
    mode: int | None = None,
    search_deps_in: Sequence[str | os.PathLike[str]] | str | os.PathLike[str] | None = None,
) -> ctypes.CDLL:
    """加载共享库，可选地先固定其依赖。

    ``preload`` 的存在源于一种微妙且调试代价高昂的失败模式：``dlopen`` 依据*加载器
    搜索路径*解析库的 ``DT_NEEDED`` soname，而**不是**依据正在被加载的库所在目录。当
    依赖的旧副本位于该路径中更靠前的位置（通常由被 source 的 ``setup.bash``/``env.sh``
    导出）时，新构建的垫片会静默地绑定到它，随后以 "undefined symbol" 失败，或返回
    一个对你所编译的版本毫无意义的错误码。用绝对路径配合 ``RTLD_GLOBAL`` 预加载正确
    的文件，会先把它的 soname 注册到全局命名空间，这样后续加载就会复用它而无需搜索。

    参数:
        path: 目标 ``.so``/``.dylib``/``.dll`` 的绝对或相对路径。不含目录分隔符的裸名称
            （例如 ``"m"``）会通过 :func:`ctypes.util.find_library` 解析，因为 ``dlopen``
            自身并不应用 ``lib``/``.so`` 命名约定。
        preload: 要在目标之前按顺序用 ``RTLD_GLOBAL`` 加载的依赖库。已被本辅助函数加载过
            的重复项会被跳过。
        mode: ``dlopen`` 模式标志。``None`` 使用 :data:`ctypes.DEFAULT_MODE`。当其他库
            必须看到本库的符号时（垫片的常见情形），传入 :data:`ctypes.RTLD_GLOBAL`。
        search_deps_in: 用于搜索依赖的额外目录。其实现方式是 "用 ``RTLD_GLOBAL`` 预加载
            在其中找到的每个 ``lib*.so*``" 外加一次 ``LD_LIBRARY_PATH`` 更新，因为运行
            中的进程无法追溯修改 glibc 的搜索路径。只要依赖集合已知，就优先使用显式的
            ``preload`` 列表——这个选项是一把钝器。

    返回:
        加载得到的 :class:`ctypes.CDLL` 句柄。

    异常:
        FileNotFoundError: 当目标（或某个 preload 条目）不存在时。消息包含解析后的路径
            和一条构建提示。
        OSError: 当 ``dlopen`` 拒绝该库时（ELF class 错误、符号未解析、传递依赖缺失）。
            会附加解析后的路径和一条诊断提示。
        ValueError: 当 ``mode`` 不是整数时。

    示例:
        >>> import ctypes
        >>> libm = load_library("m", mode=ctypes.RTLD_GLOBAL)
        >>> declare(libm.sqrt, [ctypes.c_double], ctypes.c_double)(4.0)
        2.0
    """
    target = _as_path(path, what="path")
    if mode is not None and not isinstance(mode, int):
        raise ValueError(f"mode must be an int or None, got {mode!r}")
    load_mode = ctypes.DEFAULT_MODE if mode is None else mode

    deps: list[Path] = []
    for entry in _as_str_sequence(preload, what="preload"):
        dep = _as_path(entry, what="preload entry").expanduser()
        if not dep.is_file():
            raise FileNotFoundError(
                f"preload dependency not found: {dep.resolve()} "
                "(build it first, or pass the correct absolute path)"
            )
        deps.append(dep.resolve())

    for entry in _as_str_sequence(search_deps_in, what="search_deps_in"):
        directory = _as_path(entry, what="search_deps_in entry").expanduser()
        if not directory.is_dir():
            raise FileNotFoundError(
                f"search_deps_in directory not found: {directory.resolve()}"
            )
        directory = directory.resolve()
        found = sorted(p for p in directory.glob("lib*.so*") if p.is_file())
        if not found:
            logger.warning("search_deps_in %s contains no lib*.so* files", directory)
        deps.extend(p.resolve() for p in found)
        existing = os.environ.get("LD_LIBRARY_PATH", "")
        os.environ["LD_LIBRARY_PATH"] = (
            f"{directory}{os.pathsep}{existing}" if existing else str(directory)
        )
        logger.debug("added %s to LD_LIBRARY_PATH (affects child processes only)", directory)

    for dep in deps:
        key = str(dep)
        if key in _PRELOADED:
            continue
        try:
            ctypes.CDLL(key, mode=ctypes.RTLD_GLOBAL)
        except OSError as exc:
            raise OSError(
                f"failed to preload dependency {key}: {exc}. "
                f"Inspect its own dependencies with: ldd {key}"
            ) from exc
        _PRELOADED.add(key)
        logger.debug("preloaded dependency with RTLD_GLOBAL: %s", key)

    separators = [os.sep] + ([os.altsep] if os.altsep else [])
    has_separator = any(sep in str(target) for sep in separators)
    if has_separator:
        resolved = target.expanduser().resolve()
        if not resolved.is_file():
            hint = (
                "build it (see build_shared_library) or pass the correct absolute path"
                if str(target).endswith(".so")
                else "check the path"
            )
            raise FileNotFoundError(f"native library not found: {resolved} ({hint})")
        load_name = str(resolved)
    else:
        # 裸名称：dlopen 不应用 lib/.so 命名约定（那是链接编辑器的工作），因此先通过
        # ctypes.util.find_library 解析，失败时再原样把名称交给加载器。
        from ctypes.util import find_library

        resolved_name = find_library(str(target))
        if resolved_name:
            logger.debug("resolved library name %r to %r", str(target), resolved_name)
            load_name = resolved_name
        else:
            load_name = str(target)

    try:
        handle = ctypes.CDLL(load_name, mode=load_mode)
    except OSError as exc:
        raise OSError(
            f"failed to load native library {load_name}: {exc}. "
            f"Inspect unresolved symbols with: ldd {load_name}"
        ) from exc
    logger.debug("loaded native library %s (mode=%s)", load_name, load_mode)
    return handle


def declare(
    func: Any,
    argtypes: Sequence[Any] | None,
    restype: Any,
) -> Any:
    """给外部函数附加显式的 ctypes 签名。

    没有 ``argtypes`` 时 ctypes 只能靠猜：整数按 C ``int`` 传递（会截断 64 位值），
    浮点按 C ``double``，指针能否工作全凭运气。没有显式的 ``restype`` 时，每次调用都被
    假定返回 C ``int``。把两者都声明出来，是可靠的绑定与内存损坏之间的分界线，而在一次
    调用中完成声明则能让调用点保持声明式且易于审查。

    参数:
        func: 从已加载库获得的一个 ctypes 函数指针（``lib.symbol``）。
        argtypes: 位置参数类型，例如 ``[ctypes.c_char_p, ctypes.c_double]``。对于 void
            函数为 ``None`` 或 ``[]``。
        restype: 返回类型，例如 ``ctypes.c_int``。``None`` 表示 C ``int``（ctypes 自身
            的约定）。

    返回:
        同一个函数对象，因此调用可以链式书写。

    异常:
        TypeError: 当 ``func`` 不是 ctypes 函数指针，或 ``argtypes`` 不是序列时。

    示例:
        >>> import ctypes
        >>> libm = ctypes.CDLL("libm.so.6")
        >>> sqrt = declare(libm.sqrt, [ctypes.c_double], ctypes.c_double)
        >>> sqrt(9.0)
        3.0
    """
    if not isinstance(func, ctypes._CFuncPtr):  # noqa: SLF001 - ctypes 的公开检查方式
        raise TypeError(
            f"func must be a ctypes function pointer (lib.symbol), got "
            f"{type(func).__name__}: {func!r}"
        )
    if argtypes is not None and (
        isinstance(argtypes, (str, bytes)) or not isinstance(argtypes, Iterable)
    ):
        raise TypeError(f"argtypes must be a sequence or None, got {argtypes!r}")
    func.argtypes = [] if argtypes is None else list(argtypes)
    func.restype = restype
    return func


class CFunction:
    """带已声明签名和返回码校验的 ctypes 函数。

    它把几乎无处不在的原生约定 "``0`` 表示成功，其他都是错误码" 通用化，而不硬编码任何
    厂商的错误码表。包装器的价值在于失败绝不会被忽略：数字码及其映射描述一同随
    :class:`NativeCallError` 传递，因此调用方可以对暂时性错误码重试、对致命错误码中止。

    这里有意*不*使用 ``errcheck``。它运行在 ctypes 的调用机制内部，在那里抛异常会产生
    令人困惑的 traceback，且无法区分 "函数失败了" 和 "ctypes 转换结果失败了"。在 Python
    中检查则能让栈回溯指向调用点。

    参数:
        func: 一个 ctypes 函数指针（通常已通过 :func:`declare`）。
        argtypes: 便捷快捷方式；给出时会应用到 ``func``。
        restype: 便捷快捷方式；给出时会应用到 ``func``。
        name: 用于错误消息中的名称。默认为 ctypes 的符号名。
        ok_codes: 被视为成功的返回值。必须非空。
        error_map: 已知错误码到人类可读文本的映射。不在映射中的码仍会被抛出，只是带一条
            通用消息。

    异常:
        TypeError: 当 ``func`` 不是 ctypes 函数指针、``error_map`` 不是整数键的映射，或
            ``name`` 不是字符串时。
        ValueError: 当 ``ok_codes`` 为空或含有非整数时。

    示例:
        >>> import ctypes
        >>> libm = ctypes.CDLL("libm.so.6")
        >>> fpclassify = CFunction(declare(libm.isinf, [ctypes.c_double], ctypes.c_int),
        ...                        ok_codes=(0, 1), error_map={-1: "domain error"})
        >>> fpclassify(float("inf"))
        1
    """

    def __init__(
        self,
        func: Any,
        *,
        argtypes: Sequence[Any] | None = None,
        restype: Any = None,
        name: str | None = None,
        ok_codes: Iterable[int] = (0,),
        error_map: Mapping[int, str] | None = None,
    ) -> None:
        if argtypes is not None or restype is not None:
            declare(func, argtypes, ctypes.c_int if restype is None else restype)
        if not isinstance(func, ctypes._CFuncPtr):  # noqa: SLF001
            raise TypeError(
                f"func must be a ctypes function pointer, got {type(func).__name__}: {func!r}"
            )
        self._func = func
        self.name: str = name or getattr(func, "__name__", "native_function")
        if not isinstance(self.name, str):
            raise TypeError(f"name must be a str, got {type(name).__name__}: {name!r}")

        if isinstance(ok_codes, (str, bytes)) or not isinstance(ok_codes, Iterable):
            raise TypeError(f"ok_codes must be an iterable of ints, got {ok_codes!r}")
        codes: set[int] = set()
        for code in ok_codes:
            if isinstance(code, bool) or not isinstance(code, int):
                raise ValueError(f"ok_codes entries must be ints, got {code!r}")
            codes.add(code)
        if not codes:
            raise ValueError("ok_codes must contain at least one accepted return code")
        self.ok_codes: frozenset[int] = frozenset(codes)

        self.error_map: dict[int, str] = {}
        if error_map is not None:
            if not isinstance(error_map, Mapping):
                raise TypeError(f"error_map must be a mapping, got {type(error_map).__name__}")
            for code, text in error_map.items():
                if isinstance(code, bool) or not isinstance(code, int):
                    raise TypeError(f"error_map keys must be ints, got {code!r}")
                self.error_map[code] = str(text)

    @property
    def argtypes(self) -> list[Any] | None:
        """所包装函数已声明的参数类型。

        示例:
            >>> import ctypes
            >>> libm = ctypes.CDLL("libm.so.6")
            >>> CFunction(declare(libm.sqrt, [ctypes.c_double], ctypes.c_double)).argtypes
            [<class 'ctypes.c_double'>]
        """
        return list(self._func.argtypes or [])

    @property
    def restype(self) -> Any:
        """所包装函数已声明的返回类型。

        示例:
            >>> import ctypes
            >>> libm = ctypes.CDLL("libm.so.6")
            >>> CFunction(declare(libm.sqrt, [ctypes.c_double], ctypes.c_double)).restype
            <class 'ctypes.c_double'>
        """
        return self._func.restype

    @property
    def raw(self) -> Any:
        """底层的 ctypes 函数指针，供高级用法使用。

        用它来绕过返回码校验（例如在轮询就绪时，此时失败是*预期*的且不应抛异常）。

        示例:
            >>> import ctypes
            >>> libm = ctypes.CDLL("libm.so.6")
            >>> CFunction(declare(libm.sqrt, [ctypes.c_double], ctypes.c_double)).raw(4.0)
            2.0
        """
        return self._func

    def __call__(self, *args: Any) -> int:
        """调用原生函数并校验其返回码。

        参数:
            *args: 原样转发给原生函数的参数。它们必须与已声明的 ``argtypes`` 匹配；
                ctypes 负责转换。

        返回:
            原始返回值（始终是 ``ok_codes`` 的成员）。

        异常:
            NativeCallError: 当返回的码不在 ``ok_codes`` 中时。消息在有映射文本时包含
                该文本，以及数字码。
            ArgumentError: 当某个参数无法转换为已声明类型时，由 ctypes 透传。

        示例:
            >>> import ctypes
            >>> libm = ctypes.CDLL("libm.so.6")
            >>> isinf = CFunction(declare(libm.isinf, [ctypes.c_double], ctypes.c_int),
            ...                   ok_codes=(0,))
            >>> isinf(float("inf"))
            Traceback (most recent call last):
            ...
            reusable_model.runtime.ctypes_loader.NativeCallError: isinf: native call failed ...
        """
        result = self._func(*args)
        code = result if isinstance(result, int) else int(result)
        if code in self.ok_codes:
            return code
        message = self.error_map.get(code, "")
        raise NativeCallError(code, message, function=self.name)

    def try_call(self, *args: Any) -> int | None:
        """调用原生函数，把失败码转换为 ``None``。

        适用于轮询循环，其中 "尚未就绪" 是正常、预期的结果，而不是值得抛异常的错误。

        参数:
            *args: 转发给原生函数的参数。

        返回:
            成功时返回返回码，失败时返回 ``None``。

        示例:
            >>> import ctypes
            >>> libm = ctypes.CDLL("libm.so.6")
            >>> isinf = CFunction(declare(libm.isinf, [ctypes.c_double], ctypes.c_int))
            >>> isinf.try_call(float("inf")) is None
            True
        """
        try:
            return self(*args)
        except NativeCallError:
            return None

    def __repr__(self) -> str:
        """返回调试用的表示。

        返回:
            包含符号名和可接受返回码的字符串。

        示例:
            >>> import ctypes
            >>> libm = ctypes.CDLL("libm.so.6")
            >>> repr(CFunction(declare(libm.sqrt, [ctypes.c_double], ctypes.c_double)))
            "CFunction(name='sqrt', ok_codes=(0,))"
        """
        return f"CFunction(name={self.name!r}, ok_codes={tuple(sorted(self.ok_codes))!r})"


def find_library_file(
    directory: str | os.PathLike[str],
    name: str,
    *,
    patterns: Sequence[str] | None = None,
) -> Path | None:
    """通过在目录中按常规名称做 glob 来定位共享库。

    厂商构建在命名上并不统一：有的发布 ``libfoo.so``，有的发布 ``foo.so``，而带版本的
    发行版还会产生 ``libfoo.so.1.1.0``。按顺序尝试这些模式并对每个模式的匹配结果排序，
    可以让结果在不同机器上保持确定性——当带版本和不带版本的副本同时存在时，任意的
    ``glob`` 顺序正是 "这里能用、那里坏了" 这类 bug 的来源。

    参数:
        directory: 要搜索的目录（不递归）。
        name: 不含 ``lib`` 前缀或扩展名的库基本名，例如 ``"vendor_sdk"``。
        patterns: glob 模板；会替换其中的 ``{name}``。默认为
            :data:`DEFAULT_LIBRARY_PATTERNS`。

    返回:
        第一个匹配的常规文件，以绝对 :class:`~pathlib.Path` 形式给出；当目录不存在或
        没有匹配项（会记日志）时为 ``None``。

    异常:
        TypeError: 当 ``directory`` 不是路径，或 ``patterns`` 不是字符串序列时。
        ValueError: 当 ``name`` 为空白，或某个模式会产生路径分隔符时。

    示例:
        >>> find_library_file("/usr/lib/x86_64-linux-gnu", "m") is not None
        True
        >>> find_library_file("/nonexistent-dir-xyz", "m") is None
        True
    """
    base = _as_path(directory, what="directory")
    if not isinstance(name, str):
        raise TypeError(f"name must be a str, got {type(name).__name__}: {name!r}")
    if not name.strip():
        raise ValueError(f"name must be a non-blank library base name, got {name!r}")
    templates = list(DEFAULT_LIBRARY_PATTERNS) if patterns is None else _as_str_sequence(
        patterns, what="patterns"
    )
    if not templates:
        raise ValueError("patterns must not be empty")

    if not base.is_dir():
        logger.debug("find_library_file: directory does not exist: %s", base)
        return None

    for template in templates:
        glob_text = template.format(name=name)
        if os.sep in glob_text or glob_text.startswith("."):
            raise ValueError(
                f"pattern {template!r} must not produce a path traversal: {glob_text!r}"
            )
        matches = sorted(p for p in base.glob(glob_text) if p.is_file())
        if matches:
            return matches[0].resolve()
    logger.debug("find_library_file: no %s library found in %s", name, base)
    return None


def poll_until_ready(
    probe: Callable[[], Any],
    *,
    timeout: float = 10.0,
    interval: float = 0.05,
) -> bool:
    """轮询一个零参数的就绪探针，直到它成功或时间耗尽。

    动机：原生客户端的 ``init`` 常常在 socket 一打开就返回，而认证/状态握手在后台线程
    中异步完成。在这个窗口内发出第一条真正的命令会以 "not ready" 或 "no permission"
    错误码失败，而由于它是竞态，看起来像是间歇性的——每当某些无关的启动工作恰好耗时更久
    时它就会消失。用一个*空操作*命令（零速度、状态读取）轮询直到被接受，以最多
    ``interval`` 的额外延迟为代价消除了这个竞态，而且这个空操作在构造上是无害的。

    ``probe`` 抛出的异常被视为 "尚未就绪"，并以 DEBUG 级别记录，因此一个与硬件通信且偶尔
    超时的探针不会中止握手。最后一次异常会在失败警告中被报告。

    参数:
        probe: 零参数可调用对象，在就绪后返回真值。
        timeout: 最长总等待时间，单位为秒。``0`` 表示只探测一次。
        interval: 两次探测之间的休眠时间，单位为秒。

    返回:
        若探针在截止时间前返回真值则为 ``True``，否则为 ``False``。

    异常:
        TypeError: 当 ``probe`` 不可调用时。
        ValueError: 当 ``timeout`` 为负数，或 ``interval`` 不为正数时。

    示例:
        >>> calls = iter([False, False, True])
        >>> poll_until_ready(lambda: next(calls), timeout=1.0, interval=0.001)
        True
        >>> poll_until_ready(lambda: False, timeout=0.01, interval=0.001)
        False
    """
    if not callable(probe):
        raise TypeError(f"probe must be callable, got {type(probe).__name__}: {probe!r}")
    if not isinstance(timeout, (int, float)) or isinstance(timeout, bool):
        raise TypeError(f"timeout must be a number, got {type(timeout).__name__}: {timeout!r}")
    if not isinstance(interval, (int, float)) or isinstance(interval, bool):
        raise TypeError(f"interval must be a number, got {type(interval).__name__}: {interval!r}")
    if timeout < 0:
        raise ValueError(f"timeout must be >= 0, got {timeout!r}")
    if interval <= 0:
        raise ValueError(f"interval must be > 0, got {interval!r}")

    deadline = time.monotonic() + float(timeout)
    attempts = 0
    last_error: BaseException | None = None
    while True:
        attempts += 1
        try:
            if probe():
                logger.debug("readiness probe succeeded after %d attempt(s)", attempts)
                return True
            last_error = None
        except Exception as exc:  # noqa: BLE001 - 不稳定的探针不得中止等待
            last_error = exc
            logger.debug("readiness probe raised (treated as not ready): %s", exc)
        if time.monotonic() >= deadline:
            break
        time.sleep(min(interval, max(0.0, deadline - time.monotonic())))
    logger.warning(
        "readiness probe still failing after %.2fs (%d attempt(s)%s)",
        timeout,
        attempts,
        f", last error: {last_error}" if last_error else "",
    )
    return False


#: 一个向 ctypes 暴露 C++ 单例的 ``extern "C"`` 垫片骨架。
#:
#: 用 :func:`render_c_shim` 渲染它并改写各函数体；*结构*是可复用的部分：
#:
#: * 每个导出函数都使用 C 链接，且只使用 C 兼容类型（``int``、``double``、``char *``、
#:   指针）。没有类、没有引用、没有异常——异常逃逸进 ctypes 会穿过没有 unwind 表的
#:   栈帧并中止进程。
#: * C++ 对象通过单例访问器取得，而不是不透明句柄，因此 Python 侧永远不必管理一个它
#:   看不见的生命周期。
#: * 从库*自身*回调线程到达的数据会被复制进一个受 mutex 保护的全局缓存。Python 侧随后
#:   用一个非阻塞 getter 读取该缓存，而不是尝试接收 ``std::function`` 回调——后者根本
#:   无法跨越 C 边界。必须使用 mutex，因为写入方是一个不涉及 Python GIL 的外部线程。
#: * getter 返回 ``0``/负数码，并通过调用方提供的出参指针写出数据，这正是
#:   :class:`CFunction` 所校验的形式。
EXTERN_C_SHIM_TEMPLATE = """\
// extern "C" shim for $library_name.
// Generated from reusable_model.runtime.ctypes_loader.EXTERN_C_SHIM_TEMPLATE -- adapt the bodies.
#include "$header"

#include <mutex>

namespace {

// Adapt to the payload layout; bounds the copy in ${prefix}_get below.
constexpr int kPayloadSize = 8;

std::mutex g_mutex;
$payload_type g_cache{};
bool g_cache_valid = false;

// Runs on the vendor library's background thread. It must only copy into the cache:
// calling back into the library here can deadlock, and touching Python objects would
// require the GIL, which this thread does not hold.
void OnUpdate(const $payload_type &value) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_cache = value;
    g_cache_valid = true;
}

}  // namespace

extern "C" {

// Returns 0 when ready, negative on failure. Never throws: an exception escaping
// into ctypes aborts the interpreter.
int ${prefix}_init(void) {
    auto &client = $instance_expr;
    if (!client.IsInitialized()) {
        return -1;
    }
    return client.RegisterCallback(OnUpdate) == 0 ? 0 : -2;
}

// 1 once at least one frame has been received, 0 while the asynchronous handshake
// is still in flight. Polled from Python via poll_until_ready().
int ${prefix}_ready(void) {
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_cache_valid ? 1 : 0;
}

// Copies the cached payload into caller-owned storage. 0 = ok, -1 = no data yet.
int ${prefix}_get(double *out, int count) {
    if (out == nullptr || count <= 0) {
        return -1;
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_cache_valid) {
        return -1;
    }
    for (int i = 0; i < count && i < kPayloadSize; ++i) {
        out[i] = g_cache.values[i];
    }
    return 0;
}

}  // extern "C"
"""


def render_c_shim(
    *,
    library_name: str,
    header: str,
    prefix: str,
    payload_type: str,
    instance_expr: str,
) -> str:
    """把 :data:`EXTERN_C_SHIM_TEMPLATE` 渲染为具体的 C++ 源代码文本。

    使用 :class:`string.Template`（``$name`` / ``${name}`` 占位符）而非 ``str.format``，
    因为 C++ 源码中满是花括号，用后者就得处处写成双花括号，使模板无法阅读。

    参数:
        library_name: 生成的头注释中使用的人类可读名称。
        header: 要 ``#include`` 的头文件，例如 ``"vendor_sdk.h"``。
        prefix: 导出函数的 C 符号前缀（``vn`` 会产生 ``vn_init``/``vn_ready``/
            ``vn_get``）。
        payload_type: 存储在受 mutex 保护缓存中的 C++ 类型。
        instance_expr: 产生客户端引用的 C++ 表达式，例如
            ``"VendorClient::GetInstance(cfg)"``。

    返回:
        渲染后的 C++ 源代码字符串（不写入磁盘）。

    异常:
        ValueError: 当任一值为空白，或某个值不是 C 标识符/``prefix`` 安全记号时。
        TypeError: 当任一值不是字符串时。

    示例:
        >>> src = render_c_shim(library_name="demo", header="demo.h", prefix="dm",
        ...                     payload_type="DemoState",
        ...                     instance_expr="DemoClient::GetInstance()")
        >>> 'int dm_init(void)' in src and '#include "demo.h"' in src
        True
    """
    values = {
        "library_name": library_name,
        "header": header,
        "prefix": prefix,
        "payload_type": payload_type,
        "instance_expr": instance_expr,
    }
    for key, value in values.items():
        if not isinstance(value, str):
            raise TypeError(f"{key} must be a str, got {type(value).__name__}: {value!r}")
        if not value.strip():
            raise ValueError(f"{key} must be a non-blank string, got {value!r}")
    for key in ("prefix",):
        if not _IDENTIFIER_RE.match(values[key]):
            raise ValueError(
                f"{key} must be a C identifier [A-Za-z_][A-Za-z0-9_]* , got {values[key]!r}"
            )
    for key in ("header", "payload_type", "instance_expr"):
        if "\n" in values[key]:
            raise ValueError(f"{key} must be a single line, got {values[key]!r}")

    template = string.Template(EXTERN_C_SHIM_TEMPLATE)
    try:
        return template.substitute(**values)
    except (KeyError, ValueError) as exc:
        raise ValueError(f"cannot render shim template: {exc}") from exc


def build_shared_library(
    source: str | os.PathLike[str],
    output: str | os.PathLike[str],
    *,
    include_dirs: Sequence[str | os.PathLike[str]] | None = None,
    library_dirs: Sequence[str | os.PathLike[str]] | None = None,
    libraries: Sequence[str] | None = None,
    std: str = "c++17",
    extra_flags: Sequence[str] | None = None,
    compiler: str = "g++",
) -> subprocess.CompletedProcess[str]:
    """把 C/C++ 源文件编译为位置无关的共享库。

    显式拼装编译器命令行并在不使用 shell 的情况下运行，因此任何参数都不会受到 shell
    展开的影响。``-Wl,-rpath,$ORIGIN`` 并*不*会被隐式加入；当运行时依赖就位于产物旁边时
    通过 ``extra_flags`` 传入它（这能防止厂商的 ``.so`` 被 ``LD_LIBRARY_PATH`` 上陈旧的
    副本遮蔽——参见 :func:`load_library`）。

    参数:
        source: 要编译的源文件。必须存在。
        output: 共享对象的目标路径。父目录会被创建。
        include_dirs: 头文件搜索目录（``-I``）。每个都必须存在。
        library_dirs: 库搜索目录（``-L``）。每个都必须存在。
        libraries: 库基本名（``-l``），例如 ``["vendor_sdk", "pthread"]``。
        std: 语言标准，例如 ``"c++17"`` 或 ``"c11"``。
        extra_flags: 追加在 ``-l`` 条目之后的其他原始标志。
        compiler: 编译器可执行文件（``g++``、``clang++``、``cc``）。

    返回:
        成功构建（returncode 为 0）的 :class:`subprocess.CompletedProcess`，其
        ``stdout``/``stderr`` 以文本形式捕获。

    异常:
        FileNotFoundError: 当 ``source``、某个 include 目录或某个 library 目录不存在时。
        TypeError: 当任一参数类型错误时。
        ValueError: 当 ``source``/``output``/``compiler``/``std`` 为空白、``std`` 不是
            标志安全记号，或某个库名不是标识符时。
        NativeBuildError: 当编译器以非零状态退出时。其消息包含完整命令行和编译器的
            stderr。

    示例:
        >>> import tempfile, pathlib
        >>> tmp = pathlib.Path(tempfile.mkdtemp())
        >>> src = tmp / "add.cpp"
        >>> _ = src.write_text('extern "C" int add(int a, int b) { return a + b; }')
        >>> out = tmp / "libadd.so"
        >>> result = build_shared_library(src, out, std="c++17")
        >>> result.returncode, out.is_file()
        (0, True)
    """
    src = _as_path(source, what="source").expanduser()
    dst = _as_path(output, what="output").expanduser()
    if str(src).strip() == "":
        raise ValueError(f"source must be a non-blank path, got {source!r}")
    if not src.is_file():
        raise FileNotFoundError(f"source file not found: {src.resolve()}")
    if str(dst).strip() == "":
        raise ValueError(f"output must be a non-blank path, got {output!r}")
    if not isinstance(compiler, str) or not compiler.strip():
        raise ValueError(f"compiler must be a non-blank executable name, got {compiler!r}")
    if " " in compiler.strip():
        raise ValueError(f"compiler must be a single executable name, got {compiler!r}")
    if not isinstance(std, str) or not _STD_RE.match(std):
        raise ValueError(f"std must look like 'c++17' or 'c11', got {std!r}")

    includes: list[str] = []
    for entry in _as_str_sequence(include_dirs, what="include_dirs"):
        directory = _as_path(entry, what="include_dirs entry").expanduser()
        if not directory.is_dir():
            raise FileNotFoundError(f"include directory not found: {directory.resolve()}")
        includes.append(str(directory.resolve()))

    libdirs: list[str] = []
    for entry in _as_str_sequence(library_dirs, what="library_dirs"):
        directory = _as_path(entry, what="library_dirs entry").expanduser()
        if not directory.is_dir():
            raise FileNotFoundError(f"library directory not found: {directory.resolve()}")
        libdirs.append(str(directory.resolve()))

    libs: list[str] = []
    for entry in _as_str_sequence(libraries, what="libraries"):
        if not entry.strip():
            raise ValueError(f"libraries entries must be non-blank, got {entry!r}")
        if not _IDENTIFIER_RE.match(entry):
            raise ValueError(
                f"libraries entry must be a bare library name (no 'lib', no '.so'), "
                f"got {entry!r}"
            )
        libs.append(entry)

    flags = _as_str_sequence(extra_flags, what="extra_flags")

    dst = dst.resolve()
    dst.parent.mkdir(parents=True, exist_ok=True)

    argv: list[str] = [
        compiler,
        "-shared",
        "-fPIC",
        f"-std={std}",
        "-O2",
    ]
    argv.extend(f"-I{path}" for path in includes)
    argv.append(str(src))
    argv.extend(["-o", str(dst)])
    argv.extend(f"-L{path}" for path in libdirs)
    argv.extend(f"-l{name}" for name in libs)
    argv.extend(flags)

    logger.info("building shared library: %s", " ".join(argv))
    completed = subprocess.run(  # noqa: S603 - argv 已完全校验，且不使用 shell
        argv,
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        raise NativeBuildError(
            f"failed to build shared library {dst}",
            argv,
            completed.returncode,
            completed.stderr,
        )
    if not dst.is_file():
        raise NativeBuildError(
            f"compiler reported success but {dst} does not exist",
            argv,
            completed.returncode,
            completed.stderr,
        )
    logger.info("built %s", dst)
    return completed
