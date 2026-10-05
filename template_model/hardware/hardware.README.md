# hardware 子包说明

本子包提供机器人硬件层的两个可复用模块：**串口设备探测与解析**（`serial_discovery`）
和 **Modbus RTU 并联夹爪驱动**（`modbus_gripper`）。所有模块均不包含绝对路径、环境
变量或项目专属配置，且不调用 `print()`（通过 `logging.getLogger(__name__)` 记录
日志）。

## 1. 模块总览

| 文件 | 主要职责 | 对外公开接口（主要 API 名） |
| --- | --- | --- |
| `__init__.py` | 子包入口，经 `from .modbus_gripper import *` 与 `from .serial_discovery import *` 重新导出两个模块的全部公开符号 | `__all__ = ["modbus_gripper", "serial_discovery"]`（星号导入随之导出下表两个模块的所有公开名） |
| `serial_discovery.py` | 把 USB 串口设备（尤其 `/dev/serial/by-id/` 下的 udev 符号链接）探测并解析为稳定的逻辑名称；提供底层列举、按序列号查找、带截止时间的探测，以及四层解析器 | `DEFAULT_BY_ID_DIR`、`list_by_id_ports`、`list_ports`、`find_by_serial`、`probe_port`、`load_mapping_file`、`resolve_ports` |
| `modbus_gripper.py` | 驱动单个 Modbus-RTU 并联夹爪（Robotiq 2F 风格寄存器映射）；含寄存器映射 profile、状态字解码、激活/移动/抓取重试逻辑、状态文本渲染，以及不接触硬件的 dry-run（空跑）模拟 | `GripperProfile`、`ROBOTIQ_2F`、`OBJECT_STATES`、`FAULT_CODES`、`MAJOR_FAULTS`、`ModbusGripper` |

## 2. 依赖关系与调用层级

- **第三方依赖**
  - `pyserial`：**惰性导入**。仅在 `serial_discovery._import_list_ports()`（被
    `list_ports()` 调用）与 `modbus_gripper.ModbusGripper.__init__`（非 dry-run
    分支）内部导入。未安装时，`list_by_id_ports` 等纯 `glob` 功能仍可正常使用。
  - `minimalmodbus`：**惰性导入**。仅在 `ModbusGripper.__init__`（非 dry-run
    分支）内部导入；`dry_run=True` 时完全跳过。
  - `PyYAML`：在 `serial_discovery.py` 模块顶层直接 `import yaml`（**非惰性**）。
    因此导入 `serial_discovery`（进而导入 `reusable_model.hardware`）需要已安装 PyYAML；
    它仅用于 `load_mapping_file` 解析 `.yaml` / `.yml` 映射文件。
- **对其他子包的依赖**：无。经检查，`hardware` 内两个文件均不导入 `reusable_model` 下的其它
  子包（`geometry`、`gridmap`、`io`、`motion`、`runtime`、`tracking`、`vision`），
  也不依赖它们的数据结构。
- **子包内部文件之间的调用层级**：`serial_discovery.py` 与 `modbus_gripper.py`
  **相互独立**，二者之间没有任何模块级依赖或导入关系。`serial_discovery.py` 的
  模块 docstring 示例中虽演示了用 `ModbusGripper` 充当 `probe_fn`，但那只是文档
  示例，代码层面二者解耦。`__init__.py` 只是把两者聚合导出。

## 3. 各模块职责与接口定义

### 3.1 serial_discovery.py

**职责**：解决"两个完全相同的 USB 转 RS-485 适配器如何稳定区分左右"的问题。
模块既提供底层列举与探测，也提供四层解析器 `resolve_ports`，把
`{"left": ..., "right": ...}` 转换为具体设备路径。解析失败只记录警告、从不抛出；
只有 `spec` 格式错误才会抛出异常。

**模块常量**

- `DEFAULT_BY_ID_DIR: str = "/dev/serial/by-id"` —— udev 设备标识符号链接目录。

**公开 API 清单**

- `list_by_id_ports(pattern: str = "*", *, by_id_dir: str | os.PathLike[str] = DEFAULT_BY_ID_DIR) -> list[str]`
  - 参数：
    - `pattern`：对文件名施加的 glob（含 `*?[` 时）或不区分大小写子串过滤器。
    - `by_id_dir`：要扫描的目录。
  - 返回：排序后的绝对设备路径列表；目录不存在或无匹配时返回 `[]`。
  - 异常：`TypeError`（`pattern` 非字符串或 `by_id_dir` 非类路径对象）、
    `ValueError`（`pattern` 含路径分隔符）。
  - 说明：排序是稳定左右分配的基础，因为 by-id 名称源自硬件序列号。

- `list_ports(vid: int | None = None, pid: int | None = None) -> list[dict[str, Any]]`
  - 参数：`vid` / `pid` 为要匹配的 USB 厂商/产品 id，`None` 表示不过滤。
  - 返回：dict 列表，键为 `device`、`serial_number`、`vendor_id`、`product_id`、
    `description`、`manufacturer`（按 `device` 排序，缺失属性为 `None`）。
  - 异常：`TypeError`、`ValueError`（id 超出 `0x0000-0xFFFF`）、`ImportError`
    （未安装 PySerial，消息提示 `pip install pyserial`）。

- `find_by_serial(serial_fragment: str, *, candidates: Iterable[str] | None = None, by_id_dir: str | os.PathLike[str] = DEFAULT_BY_ID_DIR) -> str | None`
  - 参数：`serial_fragment` 为序列号或其片段；`candidates` 为要搜索的设备路径
    （`None` 表示以无过滤器方式搜索 `list_by_id_ports`）；`by_id_dir` 在
    `candidates is None` 时使用。
  - 返回：第一个匹配的设备路径（按排序，可复现）；无匹配返回 `None`。若
    `serial_fragment` 本身是已存在的绝对路径，则原样返回。
  - 异常：`TypeError`（`serial_fragment` 非字符串，或 `candidates` 含非字符串）。

- `probe_port(port: str, probe_fn: Callable[[str], Any], *, timeout: float = 0.5) -> bool`
  - 参数：`port` 为设备路径；`probe_fn(port)` 为调用方提供的探测函数（返回真值即
    视为应答，由它负责打开/关闭串口）；`timeout` 为等待探测返回的秒数。
  - 返回：仅当探测在截止时间内返回真值时 `True`。
  - 异常：`TypeError`（`port` 非字符串、`probe_fn` 不可调用或 `timeout` 非数字）、
    `ValueError`（`port` 为空白或 `timeout` 为负）。
  - 说明：探测在守护线程上运行并以 `timeout` join，任何异常都视为"否"。

- `load_mapping_file(path: str | os.PathLike[str]) -> dict[str, Any]`
  - 参数：`path` 为映射文件路径，扩展名须为 `.yaml` / `.yml` / `.json`。
  - 返回：解析后的映射（可能为空 dict）。
  - 异常：`TypeError`（非类路径对象）、`ValueError`（扩展名不支持、无法解析或根
    节点非映射）、`FileNotFoundError`（文件不存在）。

- `resolve_ports(spec, *, mapping_file=None, serial_map=None, probe_fn=None, prefer=None, by_id_dir=DEFAULT_BY_ID_DIR, candidate_pattern="*", probe_timeout=0.5) -> dict[str, str]`
  - 完整签名：
    `resolve_ports(spec: Mapping[str, str | None] | Sequence[str], *, mapping_file: str | os.PathLike[str] | None = None, serial_map: Mapping[str, str] | None = None, probe_fn: Callable[[str], Any] | None = None, prefer: Sequence[str] | None = None, by_id_dir: str | os.PathLike[str] = DEFAULT_BY_ID_DIR, candidate_pattern: str = "*", probe_timeout: float = 0.5) -> dict[str, str]`
  - 参数：
    - `spec`：逻辑名到设备路径的映射（未知时为空白/`None`），或纯逻辑名序列；
      迭代顺序即优先级顺序。
    - `mapping_file`：可选的逐机校准文件（经 `load_mapping_file` 加载）。
    - `serial_map`：可选的 `{name: serial fragment}` 表。
    - `probe_fn`：可选的 `probe_fn(port) -> bool`，用于认领前确认设备。
    - `prefer`：各名称接收自动探测候选串口的顺序；不在 `spec` 中的名称会被忽略
      （并警告）。
    - `by_id_dir`：第 3、4 层要扫描的目录。
    - `candidate_pattern`：施加于第 4 层扫描的 glob/子串过滤器。
    - `probe_timeout`：传给 `probe_port` 的每串口截止时间。
  - 返回：`{逻辑名: 设备路径}`，只包含已解析的名称；解析失败仅记录警告。
  - 异常：`TypeError`（`spec` 类型错误、`serial_map`/`prefer` 类型错误、
    `probe_fn` 不可调用）、`ValueError`（逻辑名为空白或 `probe_timeout` 为负）、
    `FileNotFoundError`（显式指定的映射文件不存在时由 `load_mapping_file` 抛出）。
  - 四层优先级：显式路径 → 映射文件 → 代码内序列号 → 自动探测。

**使用示例**

```python
from reusable_model.hardware.serial_discovery import list_by_id_ports, resolve_ports

# 1) 仅列举（不需要 PySerial，仅依赖 glob）
for path in list_by_id_ports("usb-FTDI_*"):
    print(path)

# 2) 四层解析；没有显式路径时按序列号映射解析，失败只告警不抛出
ports = resolve_ports(
    {"left": "", "right": ""},           # 空串表示"未显式指定，交由解析器处理"
    serial_map={"left": "DAANTRS9", "right": "DAANTMYY"},
    prefer=("left", "right"),            # left 取得排序第一的适配器
)
print(ports.get("left"), ports.get("right"))

# 3) 用夹爪探测确认串口（此示例涉及硬件；请先用 dry_run 验证逻辑）
def is_gripper(port: str) -> bool:
    from reusable_model.hardware.modbus_gripper import ModbusGripper
    with ModbusGripper(port, dry_run=True) as gripper:   # 真实使用请去掉 dry_run
        gripper.read_status()
    return True

ports = resolve_ports(["left", "right"], probe_fn=is_gripper, probe_timeout=0.5)
```

### 3.2 modbus_gripper.py

**职责**：驱动单个 Modbus-RTU 并联夹爪。寄存器映射与运动默认值集中在
`GripperProfile` 中，换厂商只需换 profile。串口在构造时打开并在对象生命周期内保持
打开；每次事务前清空缓冲区，避免迟到响应被误当作下一次事务的应答。
`dry_run=True` 时不导入依赖、不打开串口、不发一个字节，而是对内部状态寄存器模拟
执行整套命令序列，供无硬件环境演练与测试。

**模块常量**

- `OBJECT_STATES: dict[int, str]` —— `gOBJ` 各值含义（0 移动中 / 1 张开受阻 /
  2 闭合受阻（抓住物体）/ 3 到达指令位置（未抓物））。
- `FAULT_CODES: dict[int, str]` —— `gFLT` 故障码含义。
- `MAJOR_FAULTS: frozenset[int]` —— `{0xA, 0xB, 0xC, 0xD, 0xE, 0xF}`，出现这些
  故障后禁止下达任何指令。

**`GripperProfile`（frozen dataclass，不可变）**

字段及默认值：`slave_id=9`、`baudrate=115200`、`write_base=1000`、`read_base=2000`、
`speed=255`、`force=180`、`activate_request=0x0100`、`goto_request=0x0900`、
`timeout=0.2`、`position_scale=255`、`activated_status=3`。

- `__post_init__(self) -> None`：根据 Modbus 与厂商范围校验每个字段（构造时即
  失败）。异常：`TypeError`（字段类型错误，如 `timeout` 非数字）、`ValueError`
  （超出协议范围，或请求字未置位定义它的位）。
- 模块级实例 `ROBOTIQ_2F = GripperProfile()`：常见 Robotiq 2F 系列并联夹爪的
  profile。

**`ModbusGripper` 类**

- `__init__(self, port: str, *, profile: Any = ROBOTIQ_2F, dry_run: bool = False) -> None`
  - 参数：`port` 为串口设备路径（建议用 `/dev/serial/by-id/...`）；`profile` 为
    寄存器映射与默认值（任何暴露 `GripperProfile` 字段的对象均可）；`dry_run=True`
    表示不接触硬件的空跑模式。
  - 异常：`TypeError`（`port` 非字符串、`profile` 缺字段、`dry_run` 非布尔值）、
    `ValueError`（`port` 为空白）、`ImportError`（未安装 `minimalmodbus`/`pyserial`，
    dry-run 模式跳过）、`serial.SerialException`（串口打不开）。
- `connected -> bool`（属性）：当前是否可发送命令。
- `dry_run_journal -> tuple[dict[str, Any], ...]`（属性）：被模拟而非实际发送的
  命令日志（副本）；硬件模式下为空。
- `decode_status(registers: Sequence[int], *, profile: Any = ROBOTIQ_2F) -> dict[str, Any]`（静态方法）：
  把三个状态寄存器解码为具名位域，返回含 `gACT`、`gGTO`、`gSTA`、`gOBJ`、`gFLT`、
  `gPO`（原始 0-255 位置）与 `position`（`gPO / position_scale`）的 dict。
  异常：`TypeError`（非整数序列）、`ValueError`（少于三个寄存器或
  `profile.position_scale` 非正整数）。
- `read_status(self) -> dict[str, Any]`：读取并解码状态块。异常：`RuntimeError`
  （已关闭）、`minimalmodbus.ModbusException`（总线错误）。
- `status_text(self, status: Mapping[str, Any] | None = None) -> str`：把状态渲染
  成一行日志（`None` 表示先读取设备）。异常：`TypeError`、`ValueError`（`status`
  非映射或缺键）。
- `has_major_fault(self, status: Mapping[str, Any] | None = None) -> bool`：`gFLT`
  是否属于 `MAJOR_FAULTS`。异常：`TypeError`、`ValueError`。
- `reset(self) -> None`：清零动作请求块、使夹爪失活（会松开手指）。异常：
  `RuntimeError`、`minimalmodbus.ModbusException`。
- `activate(self, timeout: float = 10.0, poll_interval: float = 0.1) -> None`：
  激活并阻塞轮询到 `gSTA == profile.activated_status`。`timeout=0` 表示只检查一次。
  异常：`TypeError`、`ValueError`、`TimeoutError`（超时，消息含最后一次状态）、
  `RuntimeError`。
- `go_to(self, target: float, *, force: int | None = None, speed: int | None = None) -> None`：
  指令归一化位置 `0.0`（全张）～`1.0`（全闭），不等待移动完成。异常：
  `TypeError`、`ValueError`（`target` 超范围或字节字段超 `0-255`）、`RuntimeError`。
- `open(self) -> None`：指令完全张开，等价于 `go_to(0.0)`。
- `close(self, target: float = 1.0) -> None`：指令闭合到 `target`（略小于 `1.0`
  如 `0.9` 通常更好）。异常：`TypeError`、`ValueError`、`RuntimeError`。
- `wait_motion_complete(self, timeout: float = 1.0, poll_interval: float = 0.05) -> bool`：
  阻塞直到手指停止移动（`gOBJ != 0`）；`timeout=0` 表示只读一次。异常：
  `TypeError`、`ValueError`、`RuntimeError`。
- `squeeze_more(self, target: float = 1.0, *, force: int | None = None, wait_s: float = 1.0, step: float = 0.02, poll_interval: float = 0.05) -> dict[str, Any]`：
  **只收紧、绝不放松**的抓取重试（实际指令位置为
  `min(1.0, max(target, current_position + step))`）；遇重大故障时短路并原样返回
  状态。异常：`TypeError`、`ValueError`（`target` 或 `step` 超范围、时长无效）、
  `RuntimeError`。
- `is_object_held(self, status: Mapping[str, Any] | None = None, *, target: float = 0.85, position_slack: float = 0.03) -> bool`：
  结合 `gOBJ == 2` 与有方向的位置测试判断是否抓住物体。异常：`TypeError`、
  `ValueError`（缺键、`target` 超范围、`position_slack` 为负）。
- `close_port(self) -> None`：释放串口；幂等、异常安全，可从 `finally` 调用。调用后
  各命令方法会抛 `RuntimeError`。
- `__enter__` / `__exit__`：上下文管理器（`with` 语句），退出时关闭串口且不抑制异常。
- `__repr__(self) -> str`：包含 `port`、`slave_id`、`dry_run`、`connected` 的调试
  表示。

**使用示例**

```python
from reusable_model.hardware.modbus_gripper import ModbusGripper, ROBOTIQ_2F

# 先用 dry_run 在无硬件环境下验证整套逻辑（不导入依赖、不打开串口）
with ModbusGripper("/dev/null", dry_run=True) as gripper:
    gripper.reset()
    gripper.activate(timeout=0.05)
    gripper.close(0.9)                     # 指令一次稳固闭合
    status = gripper.read_status()
    if not gripper.is_object_held(status, target=0.9):
        status = gripper.squeeze_more(1.0)  # 重试，只能更紧
    print(gripper.status_text(status))
    print(gripper.dry_run_journal[-1])      # 查看模拟命令日志

# 接上真实硬件时去掉 dry_run，使用稳定的 by-id 路径
# with ModbusGripper("/dev/serial/by-id/usb-FTDI_...-if00-port0") as gripper:
#     gripper.reset(); gripper.activate(); gripper.close(0.9)
```

## 4. 模块间交互逻辑

两个模块在设计上解耦，典型配合方式是"先探测、后驱动"：

1. 用 `serial_discovery` 找到稳定的物理串口：`list_by_id_ports` 列出 udev by-id
   符号链接，`resolve_ports` 按"显式路径 → 映射文件 → 序列号 → 自动探测"四层
   解析出 `{"left": ..., "right": ...}`，必要时用 `probe_port` 配一个探测函数确认
   该串口上确实是自己要的设备。
2. 把解析得到的设备路径交给 `ModbusGripper`：构造实例（真实硬件时
   `dry_run=False`），依次 `reset()` → `activate()` → `go_to()/close()/open()`，
   再用 `read_status()`、`is_object_held()`、`wait_motion_complete()`、
   `squeeze_more()` 观察与修正抓取，最后 `close_port()`（或用 `with` 语句自动
   关闭）。

探测函数 `probe_fn` 是二者的衔接点：`probe_port` / `resolve_ports` 只要求它是
`Callable[[str], Any]`，由调用方自由实现；常见做法是在其中构造一个
`ModbusGripper(port)` 并调用 `read_status()`，以确认该串口上的 Modbus 从站确实
应答。

## 5. 快速上手

```python
# 经子包入口导入（需已安装 PyYAML，因为 serial_discovery 在模块顶层 import yaml）
from reusable_model.hardware import (
    list_by_id_ports,
    resolve_ports,
    ModbusGripper,
    ROBOTIQ_2F,
)

# 最小示例：dry-run 驱动一个"虚拟"夹爪，不需要任何硬件
with ModbusGripper("fake-port", dry_run=True) as gripper:
    gripper.reset()
    gripper.activate(timeout=0.05)
    gripper.close(1.0)
    print(gripper.connected, gripper.read_status()["gOBJ"])
```

## 6. 测试与验证

`modbus_gripper` 与 `serial_discovery` 的可执行 doctest 与单元测试命令如下。

运行 doctest（在本仓库根目录 `Garden/` 下执行，以便 `reusable_model` 可被导入）：

```bash
cd /home/moke/Coding/Garden && PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/home/moke/Coding/Garden python3 -m pytest --doctest-modules reusable_model/hardware -q -p no:cacheprovider
```

运行 hardware 单元测试：

```bash
cd /home/moke/Coding/Garden/reusable_model && PYTHONDONTWRITEBYTECODE=1 python3 -m pytest tests/test_hardware.py -q -p no:cacheprovider
```

语法检查：

```bash
cd /home/moke/Coding/Garden/reusable_model && python3 -m py_compile hardware/serial_discovery.py hardware/modbus_gripper.py hardware/__init__.py
```

## 7. 安全提示

- **先空跑后上硬件**：任何新逻辑都应先在 `dry_run=True` 下验证命令序列、位置打包与
  抓取重试方向，确认无误后再去掉 `dry_run` 接触真实硬件。
- **急停与限位**：驱动本身不提供急停。务必在系统层面配备硬件急停与机械限位；夹爪
  闭合时若无物体阻挡，可能持续施力，须由上层逻辑限定目标位置与力。
- **不要在抓住负载时复位或张开**：`reset()`、`open()` 以及任何"更宽"的位置指令都会
  松开手指。出现 `MAJOR_FAULTS` 中的故障（过流、欠压、自动释放等）时，`squeeze_more`
  会拒绝动作；此时应停止、上报并交由人处理，**切勿**通过"复位再张开"去恢复。
- **电压与电流**：确认夹爪供电电压/电流在其铭牌范围内，欠压与过流都会触发故障码，
  并可能导致掉件。
- **串口权限**：Linux 下访问 `/dev/ttyUSB*` 或 `/dev/serial/by-id/*` 通常需要加入
  `dialout` 组或配置 udev 规则；同一串口被两个进程打开会相互破坏 Modbus 事务，请
  确保一个物理串口只由一个逻辑设备使用。
- **优先使用稳定的设备路径**：使用 `/dev/serial/by-id/...` 而非 `/dev/ttyUSB*`，
  因为后者的编号在重启/拔插后会变化，可能导致左右夹爪互换。
