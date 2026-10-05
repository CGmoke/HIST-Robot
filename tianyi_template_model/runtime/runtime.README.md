# runtime 子包说明

本子包提供与机器人运行时打交道的底层支撑能力：跨解释器的子进程桥接、原生共享库的安全
加载，以及 stdout/stderr 流过滤与日志静音。所有注释与文档均为简体中文。

## 1. 模块总览

| 文件 | 主要职责 | 对外公开接口（主要 API 名） |
| --- | --- | --- |
| `__init__.py` | 聚合子包公开接口；以 `from .X import *` 导出入三个模块的公开名称 | `__all__`（`ctypes_loader`、`stdout_filter`、`subprocess_bridge`） |
| `subprocess_bridge.py` | 与传输方式无关的父/子桥接：父进程把外部运行时作为子进程启动，通过以换行分隔的 stdin 行协议发送命令；子进程侧循环负责读取并分派 | `SubprocessBridge`、`run_bridge_child`、`BridgeError`、`BridgeStartError`、`DEFAULT_SENTINEL`、`DEFAULT_LINE_FORMAT` |
| `ctypes_loader.py` | 通过 `ctypes` 安全加载原生共享库：预加载依赖、声明签名、校验返回码、轮询就绪、生成并编译 `extern "C"` 垫片 | `load_library`、`declare`、`CFunction`、`find_library_file`、`poll_until_ready`、`render_c_shim`、`build_shared_library`、`NativeCallError`、`NativeBuildError`、`EXTERN_C_SHIM_TEMPLATE`、`DEFAULT_LIBRARY_PATTERNS` |
| `stdout_filter.py` | 过滤 stdout/stderr 流中的噪声行，并降低 `logging` 日志记录器级别 | `FilteredStream`、`install_filter`、`quiet_loggers` |

## 2. 依赖关系与调用层级

- **第三方依赖**：无。本子包仅使用 Python 标准库（`ctypes`、`subprocess`、`logging`、
  `threading`、`json`、`collections`、`pathlib`、`string`、`os`、`re`、`sys`、`time`）。
- **对其他子包的依赖**：无。经检视，`runtime` 内的任何文件都不导入 `reusable_model` 的其他子包，
  也不被它们反向依赖（就本子包范围而言）。
- **子包内部文件之间的调用层级**：三个功能模块彼此**相互独立**，不存在相互导入：
  - `subprocess_bridge.py` 不导入 `ctypes_loader.py` 或 `stdout_filter.py`；
  - `ctypes_loader.py` 不导入另外两个模块；
  - `stdout_filter.py` 不导入另外两个模块。
  - 唯一的内部引用来自 `__init__.py`，它用 `from .ctypes_loader import *`、
    `from .stdout_filter import *`、`from .subprocess_bridge import *` 把三个模块的
    公开接口聚合到子包命名空间。

## 3. 各模块职责与接口定义

### 3.1 subprocess_bridge.py

**职责**：实现父/子解释器之间的行协议桥接。父进程用一个 `subprocess.Popen` 启动外部
运行时，并为其 stdin 写入以换行分隔的命令帧；子进程在自己的解释器下运行
`run_bridge_child`，把每一行分派给业务回调。父进程从不导入外部依赖，因此两个运行时
保持隔离。模块还内置了 stderr 抽干线程（防止管道填满造成死锁）、启动期子进程存活校验，
以及"链路断开即降级返回 `False`"的控制回路友好语义。

模块级常量：

- `DEFAULT_SENTINEL = "CLOSE\n"`：通知子进程循环关停的帧（带结尾换行）。
- `DEFAULT_LINE_FORMAT = "{:.6f} {:.6f} {:.6f}\n"`：默认的三浮点帧布局。

#### `SubprocessBridge`

桥接的父进程侧。

```python
SubprocessBridge(
    argv: Sequence[str | os.PathLike[str]],
    *,
    sentinel: str = DEFAULT_SENTINEL,
    line_format: str | Callable[..., str] = DEFAULT_LINE_FORMAT,
    env: dict[str, str] | None = None,
    cwd: str | os.PathLike[str] | None = None,
    startup_timeout: float = 3.0,
    encode: str = "utf-8",
) -> None
```

- `argv`：子进程命令行，非空序列。
- `sentinel`：关停哨兵行（缺失结尾换行时自动追加）。
- `line_format`：`send` 使用的 `str.format` 模板，或可调用对象 `(*values) -> str`。
- `env`：子进程环境，`None` 继承父进程。
- `cwd`：子进程工作目录，`None` 继承。
- `startup_timeout`：`start` 等待子进程证明存活的秒数。
- `encode`：管道文本编码。
- 构造期异常：`TypeError`（类型错误）、`ValueError`（`argv` 为空、`sentinel` 为空白、
  `line_format` 无法解析）。

属性：

- `connected -> bool`：子进程是否被认定存活且可写。
- `pid -> int | None`：子进程 PID；从未启动为 `None`。
- `returncode -> int | None`：子进程退出码；运行中/未启动为 `None`。

方法：

- `stderr_tail(limit: int = 40) -> str`：返回最近捕获的子进程 stderr 行；`limit <= 0`
  抛 `ValueError`。
- `poll() -> int | None`：非阻塞检查子进程存活并同步 `connected`。
- `start() -> None`：启动子进程并阻塞至多 `startup_timeout` 秒；子进程在启动窗口内退出
  或 OS 拒绝启动时抛 `BridgeStartError`；桥接已关闭时抛 `ValueError`。
- `connect(timeout: float | None = None) -> bool`：`start` 的不抛异常变体；`timeout`
  类型错误抛 `TypeError`、为负抛 `ValueError`。
- `send(*values: Any) -> bool`：按 `line_format` 渲染一帧并发布；值数量与模板元数不匹配
  或无法格式化时抛 `ValueError`。
- `send_line(text: str) -> bool`：发布一行已格式化文本（绕过 `line_format`）；非 str 抛
  `TypeError`，含内嵌换行抛 `ValueError`。
- `send_json(obj: Any) -> bool`：把一个 JSON 对象作为单行帧发布；不可序列化抛
  `TypeError`。
- `close(*, final_values: Sequence[Any] | None = None, wait_timeout: float = 2.0) -> None`：
  幂等地关停桥接（可选停止帧 → 哨兵 → 关闭 stdin → 等待/回收）；`final_values` 非序列抛
  `TypeError`，`wait_timeout` 为负抛 `ValueError`。
- `__enter__() -> SubprocessBridge` / `__exit__(exc_type, exc, tb) -> None`：上下文管理器
  协议；进入时调用 `start` 并快速失败。
- `__repr__() -> str`：调试表示。

#### `run_bridge_child`

子进程侧循环。

```python
run_bridge_child(
    handler: Callable[[list[str]], Any],
    *,
    sentinel: str = "CLOSE",
    on_exit: Callable[[], Any] | None = None,
    stream: TextIO | None = None,
) -> int
```

- `handler`：每帧调用一次，参数是按空白切分后的字段列表。
- `sentinel`：结束循环的行（与 strip 后的行比较）。
- `on_exit`：清理回调，无论循环如何终止都在 `finally` 中恰好调用一次。
- `stream`：输入流，默认 `sys.stdin`（为测试而暴露）。
- 返回：干净关停返回 `0`，被中断返回 `1`。
- 异常：`handler` 不可调用或 `sentinel`/`stream` 类型错误抛 `TypeError`；`sentinel` 为
  空白抛 `ValueError`。

#### `BridgeError` / `BridgeStartError`

- `BridgeError(RuntimeError)`：桥接生命周期失败的基类。
- `BridgeStartError(message: str, argv: Sequence[str], returncode: int | None = None, stderr: str = "")`
  ：子进程在启动期间退出时抛出，携带命令行、退出码与捕获的 stderr。

**使用示例**：

```python
import sys
from reusable_model.runtime.subprocess_bridge import SubprocessBridge, run_bridge_child

# 父进程侧：向子解释器发送命令帧
child = [sys.executable, "-c", "import sys; list(sys.stdin)"]
with SubprocessBridge(child, startup_timeout=0.5) as bridge:
    print(bridge.connected)          # True
    print(bridge.send(0.1, 0.0, 0.25))  # True
    bridge.close(final_values=(0.0, 0.0, 0.0))

# 子进程侧：读取并分派帧
import io
seen: list[list[str]] = []
code = run_bridge_child(
    seen.append,
    sentinel="CLOSE",
    on_exit=lambda: None,
    stream=io.StringIO("0.1 0.2\nCLOSE\n"),
)
print(code, seen)                    # 0 [['0.1', '0.2']]
```

### 3.2 ctypes_loader.py

**职责**：解决通过 `ctypes` 驱动 C++ 厂商 SDK 时的三类反复出现的问题——同名陈旧库遮蔽
查找、ctypes 无法直接调用 C++、以及 `init` 返回成功但尚未就绪的竞态。提供按绝对路径
预加载依赖、显式签名声明、返回码校验、就绪轮询，以及 `extern "C"` 垫片的生成与编译。

模块级常量：

- `DEFAULT_LIBRARY_PATTERNS: tuple[str, ...] = ("lib{name}.so", "{name}.so", "{name}*.so*")`
  ：`find_library_file` 按顺序尝试的 glob 模式。
- `EXTERN_C_SHIM_TEMPLATE: str`：`extern "C"` 垫片骨架（`string.Template` 模板）。

#### `load_library`

```python
load_library(
    path: str | os.PathLike[str],
    *,
    preload: Sequence[str | os.PathLike[str]] | None = None,
    mode: int | None = None,
    search_deps_in: Sequence[str | os.PathLike[str]] | str | os.PathLike[str] | None = None,
) -> ctypes.CDLL
```

- `path`：目标库路径；不带目录分隔符的裸名称经 `ctypes.util.find_library` 解析。
- `preload`：在目标之前用 `RTLD_GLOBAL` 预加载的依赖（已加载过的会被跳过）。
- `mode`：`dlopen` 模式标志，`None` 用 `ctypes.DEFAULT_MODE`。
- `search_deps_in`：额外依赖搜索目录（会预加载其中的 `lib*.so*` 并更新
  `LD_LIBRARY_PATH`）。
- 返回：加载得到的 `ctypes.CDLL`。
- 异常：`FileNotFoundError`（目标或 preload 条目不存在）、`OSError`（`dlopen` 拒绝）、
  `ValueError`（`mode` 非整数）。

#### `declare`

```python
declare(func: Any, argtypes: Sequence[Any] | None, restype: Any) -> Any
```

给 ctypes 函数指针附加显式签名并返回同一对象。`func` 非函数指针或 `argtypes` 非序列时抛
`TypeError`。

#### `CFunction`

```python
CFunction(
    func: Any,
    *,
    argtypes: Sequence[Any] | None = None,
    restype: Any = None,
    name: str | None = None,
    ok_codes: Iterable[int] = (0,),
    error_map: Mapping[int, str] | None = None,
) -> None
```

- `func`：ctypes 函数指针（通常已过 `declare`）。
- `argtypes`/`restype`：便捷快捷方式，给出时应用到 `func`。
- `name`：错误消息中使用的名称，默认取 ctypes 符号名。
- `ok_codes`：视为成功的返回码集合，必须非空。
- `error_map`：错误码到可读文本的映射。
- 构造期异常：`TypeError`（`func` 非函数指针、`error_map` 非整数键映射、`name` 非
  str）、`ValueError`（`ok_codes` 为空或含非整数）。

属性：`argtypes -> list[Any] | None`、`restype -> Any`、`raw -> Any`（底层 ctypes
函数指针，绕过返回码校验）。

方法：

- `__call__(*args: Any) -> int`：调用并校验返回码；非成功码抛 `NativeCallError`，参数
  无法转换为已声明类型时由 ctypes 抛 `ArgumentError`。
- `try_call(*args: Any) -> int | None`：把失败码转换为 `None`。
- `__repr__() -> str`：调试表示。

#### `NativeCallError` / `NativeBuildError`

- `NativeCallError(code: int, message: str = "", *, function: str = "")`：原生函数返回
  码不在可接受集合内；`code` 非整数抛 `TypeError`。
- `NativeBuildError(message: str, argv: Sequence[str], returncode: int, stderr: str = "")`
  ：共享库构建（编译器调用）失败。

#### `find_library_file`

```python
find_library_file(
    directory: str | os.PathLike[str],
    name: str,
    *,
    patterns: Sequence[str] | None = None,
) -> Path | None
```

在目录中按常规命名 glob 定位共享库；返回第一个匹配的绝对 `Path`，目录不存在或无匹配时
返回 `None`。异常：`TypeError`（`directory`/`patterns` 类型错误）、`ValueError`（`name`
为空白或模式产生路径分隔符）。

#### `poll_until_ready`

```python
poll_until_ready(
    probe: Callable[[], Any],
    *,
    timeout: float = 10.0,
    interval: float = 0.05,
) -> bool
```

轮询零参数就绪探针，直到成功或超时。返回 `True`/`False`。异常：`TypeError`（`probe`
不可调用）、`ValueError`（`timeout` 为负或 `interval` 不为正）。

#### `render_c_shim`

```python
render_c_shim(
    *,
    library_name: str,
    header: str,
    prefix: str,
    payload_type: str,
    instance_expr: str,
) -> str
```

把 `EXTERN_C_SHIM_TEMPLATE` 渲染为具体 C++ 源代码字符串（不写盘）。异常：`ValueError`
（值为空白或不是 C 标识符/安全记号）、`TypeError`（值非 str）。

#### `build_shared_library`

```python
build_shared_library(
    source: str | os.PathLike[str],
    output: str | os.PathLike[str],
    *,
    include_dirs: Sequence[str | os.PathLike[str]] | None = None,
    library_dirs: Sequence[str | os.PathLike[str]] | None = None,
    libraries: Sequence[str] | None = None,
    std: str = "c++17",
    extra_flags: Sequence[str] | None = None,
    compiler: str = "g++",
) -> subprocess.CompletedProcess[str]
```

把 C/C++ 源文件编译为位置无关共享库，不使用 shell。返回成功构建的
`subprocess.CompletedProcess`。异常：`FileNotFoundError`（源文件/目录不存在）、
`TypeError`（参数类型错误）、`ValueError`（名称为空白或非法记号）、`NativeBuildError`
（编译器非零退出）。

**使用示例**：

```python
import ctypes
from reusable_model.runtime.ctypes_loader import CFunction, declare, load_library, poll_until_ready

# 加载 libm 并声明签名
libm = load_library("m", mode=ctypes.RTLD_GLOBAL)
sqrt = declare(libm.sqrt, [ctypes.c_double], ctypes.c_double)
print(sqrt(4.0))                                   # 2.0

# 包装返回码校验：接受 0/1 两个成功码
fpclassify = CFunction(declare(libm.isinf, [ctypes.c_double], ctypes.c_int),
                       ok_codes=(0, 1))
print(fpclassify(float("inf")))                    # 1

# try_call 把失败码转换为 None（默认 ok_codes=(0,)，1 视为失败）
isinf = CFunction(declare(libm.isinf, [ctypes.c_double], ctypes.c_int), ok_codes=(0,))
print(isinf.try_call(float("inf")) is None)        # True

# 轮询就绪
calls = iter([False, False, True])
print(poll_until_ready(lambda: next(calls), timeout=1.0, interval=0.001))  # True
```

### 3.3 stdout_filter.py

**职责**：在不改动输出代码的前提下抑制噪声。`FilteredStream` 是类文件包装器，按整行
丢弃包含禁用子串的行，并可吞掉紧随其后的空行；`quiet_loggers` 处理流过滤看不到的、
基于 `logging` 的噪声。

#### `FilteredStream`

```python
FilteredStream(
    drop_substrings: Iterable[str],
    *,
    stream: TextIO | None = None,
    drop_blank: bool = True,
    marker_attr: str = "_use_filtered_stream",
) -> None
```

- `drop_substrings`：包含其中任一子串的行被丢弃；可为空集合但不能为 `None`。
- `stream`：要包装的流，默认 `sys.stdout`。
- `drop_blank`：为 `True` 时也丢弃紧随被丢弃行之后的第一个空行。
- `marker_attr`：打在包装器上的标记属性名。
- 异常：`drop_substrings` 为 `None` 抛 `TypeError`。

属性：`installed -> bool`（当前是否为标记背后的流）。

方法：`write(s: str) -> int`、`flush() -> None`、`fileno() -> int`、`isatty() -> bool`。

#### `install_filter`

```python
install_filter(
    drop_substrings: Iterable[str],
    *,
    streams: str = "stdout",
    drop_blank: bool = True,
) -> dict[str, FilteredStream | None]
```

把 `FilteredStream` 安装到 `sys.stdout`/`sys.stderr` 上，幂等。返回从流名称到已安装包装
器的映射（已包装或未请求的为 `None`）。

#### `quiet_loggers`

```python
quiet_loggers(names: Iterable[str], *, level: int = logging.WARNING) -> int
```

把每个匹配的日志记录器降低到 `level` 级别，返回实际被更改的数量；`names` 为 `None` 抛
`TypeError`。

**使用示例**：

```python
from io import StringIO
from reusable_model.runtime.stdout_filter import FilteredStream, install_filter, quiet_loggers

sink = StringIO()
w = FilteredStream(["spam"], stream=sink)
w.write("spam line\nclean line\n")
print(repr(sink.getvalue()))          # 'clean line\n'

installed = install_filter(["perf timing"], streams="stdout,stderr")
print(sorted(installed))              # ['stderr', 'stdout']

print(quiet_loggers(["myapp"], level=30))  # 0（无匹配记录器时为 0）
```

## 4. 模块间交互逻辑

三个模块在运行时互不依赖，但在一个真实机器人集成里常被编排在一起，典型数据流如下：

1. **启动外部运行时**：父进程用 `SubprocessBridge` 启动一个运行在另一个解释器下的
   子进程（外部运行时），子进程侧运行 `run_bridge_child`，把行协议中的每条命令重新发布
   到它自己的原生 API（ROS 发布者、厂商 SDK 调用等）。父进程从不导入外部依赖。
2. **抑制控制回路噪声**：在主控制进程里用 `stdout_filter.install_filter` 把
   `sys.stdout`/`sys.stderr` 包装成 `FilteredStream`，丢弃每个控制周期刷屏的计时与参数
   转储；用 `quiet_loggers` 压低基于 `logging` 的噪声。二者与桥接的命令发送相互独立。
3. **加载原生库并预加载到全局命名空间**：用 `ctypes_loader.load_library` 以绝对路径
   `RTLD_GLOBAL` 预加载依赖，再加载垫片；用 `declare` 声明签名、`CFunction` 包装并校验
   返回码、`poll_until_ready` 等待异步握手完成、`render_c_shim` + `build_shared_library`
   生成并编译垫片。

典型调用链示例：

```
父进程
  ├─ install_filter(...)            # 压制噪声，与控制流解耦
  ├─ quiet_loggers(...)
  ├─ SubprocessBridge(argv).start() # 启动外部运行时子进程
  │     └─ 子进程 run_bridge_child(handler, on_exit=stop)   # 外部解释器内
  │           └─ handler(...) → 原生 API（ROS 发布者 / SDK 调用）
  ├─ bridge.send(...) / send_json(...)   # 链路断开时返回 False，控制回路降级
  └─ bridge.close(final_values=(0,0,0))  # 显式停止帧 + 哨兵，保证看门狗不锁存

原生桥接进程（可选，独立）
  ├─ load_library(dep, mode=RTLD_GLOBAL)     # 先预加载依赖
  ├─ load_library(shim, preload=[dep], ...)  # 再加载垫片
  ├─ declare(...) / CFunction(...)           # 声明签名并校验返回码
  └─ poll_until_ready(probe, timeout=...)    # 等待异步握手完成
```

注意：`subprocess_bridge` 与 `ctypes_loader` 通常位于不同的进程/解释器中，二者之间没有
直接的函数调用关系；它们在概念上配合（一个负责"发送控制命令"，一个负责"直接驱动原生
库"），但代码层面各自独立。

## 5. 快速上手

```python
# 子包聚合导入
from reusable_model.runtime import (
    # subprocess_bridge
    SubprocessBridge, run_bridge_child,
    # ctypes_loader
    load_library, declare, CFunction, poll_until_ready,
    # stdout_filter
    FilteredStream, install_filter, quiet_loggers,
)
```

最小可运行示例（父进程启动子解释器并发送一帧）：

```python
import sys
from reusable_model.runtime import SubprocessBridge

# 子进程只是把 stdin 读空然后退出
child = [sys.executable, "-c", "import sys; list(sys.stdin)"]

bridge = SubprocessBridge(child, startup_timeout=0.5)
bridge.start()
print(bridge.send(0.1, 0.0, 0.25))   # True
bridge.close(final_values=(0.0, 0.0, 0.0))
print(bridge.connected)              # False
```

最小可运行示例（过滤噪声并静音日志）：

```python
from reusable_model.runtime import install_filter, quiet_loggers

wrappers = install_filter(["perf timing", "params:"], streams="stdout,stderr")
print(wrappers["stdout"] is not None)
print(quiet_loggers(["reusable_model.runtime"], level=40))
```

## 6. 测试与验证

在仓库中运行 doctest（模块内嵌示例）：

```bash
cd /home/moke/Coding/Garden && \
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/home/moke/Coding/Garden \
python3 -m pytest --doctest-modules reusable_model/runtime -q -p no:cacheprovider
```

运行子包单元测试：

```bash
cd /home/moke/Coding/Garden/reusable_model && \
PYTHONDONTWRITEBYTECODE=1 \
python3 -m pytest tests/test_runtime.py tests/test_stdout_filter.py -q -p no:cacheprovider
```

说明：`PYTHONDONTWRITEBYTECODE=1` 与 `-p no:cacheprovider` 用于避免在受限环境下写入
`__pycache__`/`.pytest_cache`。此外还可做语法检查：

```bash
cd /home/moke/Coding/Garden/reusable_model && \
python3 -m py_compile runtime/subprocess_bridge.py runtime/ctypes_loader.py \
    runtime/stdout_filter.py runtime/__init__.py
```
