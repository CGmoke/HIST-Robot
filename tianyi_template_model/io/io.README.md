# io 子包说明

本子包提供与业务无关的序列化与配置辅助能力：YAML 到数据类的加载 / 原子写回、
NPZ 检测批次序列化，以及"一个 JSON 头部 + N 个二进制附件"的二进制信封。

## 1. 模块总览

| 文件 | 主要职责 | 对外公开接口（主要 API 名） |
| --- | --- | --- |
| `__init__.py` | 以 `from .xxx import *` 重导出三个子模块，并声明子包级 `__all__`。 | `binary_envelope`、`detection_npz`、`yaml_config`（子包命名空间） |
| `yaml_config.py` | YAML 加载 / 原子写入、把（嵌套）映射实例化为 `dataclasses`、旧键别名迁移、环境变量覆盖、配置路径解析、按目录缓存的配置集合。 | `ConfigError`、`load_yaml`、`dump_yaml`、`from_dict`、`load_config`、`resolve_path`、`YamlStore`、`BOOL_TRUE_STRINGS`、`BOOL_FALSE_STRINGS` |
| `binary_envelope.py` | 把 JSON 头部与二进制附件打包成单个 blob（`raw` 长度前缀帧或 Arrow IPC 流），并提供媒体类型校验、枚举规范化与图像负载编解码辅助。 | `pack`、`unpack`、`require_content_type`、`normalize_choice`、`pack_image`、`expected_rgb888_bytes`、`validate_rgb888`、`rgb888_to_array`、`array_to_rgb888`、`jpeg_to_array`、`array_to_jpeg`、`UnsupportedMediaTypeError`、`MAGIC`、`ENVELOPE_VERSION`、`MIME_BINARY`、`MIME_ARROW`、`RGB888_CHANNELS`、`BACKENDS` |
| `detection_npz.py` | 把检测字典列表打包为自描述、压缩的 `.npz` 字节负载，并支持还原。 | `pack_detections`、`unpack_detections`、`DEFAULT_STRING_FIELDS` |

## 2. 依赖关系与调用层级

### 2.1 第三方依赖

- `numpy`（硬依赖）：`binary_envelope.py` 与 `detection_npz.py` 在模块级 `import numpy as np`。
- `PyYAML`：`yaml_config.py` 在模块级 `import yaml`。即使用该模块需要安装 `PyYAML`（非惰性导入）。
- `pyarrow`（可选，惰性导入）：仅 `binary_envelope.py` 的 `backend='arrow'` 分支需要，导入发生在 `_import_pyarrow()` 被调用时；未安装时抛出带安装提示的 `ImportError`。
- `opencv-python`（可选，惰性导入）：仅 JPEG 辅助函数（`jpeg_to_array` / `array_to_jpeg`）需要，导入发生在 `_import_cv2()` 被调用时。

### 2.2 对其他子包的依赖

- `detection_npz.py` 依赖 `reusable_model.geometry.boxes.bbox_to_mask`（`from ..geometry.boxes import bbox_to_mask`），用于在检测缺少 `mask` 时按 `bbox` 补齐掩码。这是本子包唯一对其他子包的依赖。
- `yaml_config.py` 与 `binary_envelope.py` 不依赖本包其他子包。

### 2.3 子包内部调用层级

三个实现文件相互独立，彼此不 import：

- `yaml_config.py`：独立，仅依赖标准库 + PyYAML。
- `binary_envelope.py`：独立，仅依赖标准库 + numpy（可选 pyarrow / opencv）。
- `detection_npz.py`：独立，仅依赖标准库 + numpy + `reusable_model.geometry.boxes`。

唯一的内部关联在 `__init__.py`：它以 `from .binary_envelope import *`、`from .detection_npz import *`、`from .yaml_config import *` 把三者的公开符号汇总到 `reusable_model.io` 命名空间。

## 3. 各模块职责与接口定义

### 3.1 `yaml_config.py`

**职责**：解决"YAML 配置"反复出现的三个问题：映射不是配置对象、配置模式演进而机器不演进、默认写文件不原子。核心思路是"数据类即模式"：`from_dict` / `load_config` 沿数据类字段递归转换，默认值写在字段旁；`dump_yaml` 通过"同目录临时文件 + `os.replace`"实现原子写入；`aliases` 表以 `setdefault` 语义把旧的扁平键迁移到新的嵌套路径。

**模块常量**

- `BOOL_TRUE_STRINGS: tuple[str, ...] = ("1", "true", "yes", "on")` —— 文本布尔被接受为 `True` 的拼写。
- `BOOL_FALSE_STRINGS: tuple[str, ...] = ("0", "false", "no", "off")` —— 被接受为 `False` 的拼写；两个元组之外的值都会报错。

**公开 API**

#### `class ConfigError(ValueError)`

配置文件无法被读取、解析或实例化时抛出。继承自 `ValueError`。

- 构造：`ConfigError(message: str, *, path: str | os.PathLike[str] | None = None, key: str | None = None) -> None`
  - `message`：人类可读说明，不含位置信息。
  - `path`：该值来自的文件或点分逻辑来源，可为 `None`。
  - `key`：导致失败的配置键，可为 `None`。
- 属性：`message: str`、`path: str | None`、`key: str | None`。
- 异常：`message` 非 `str` 时抛 `TypeError`。

#### `load_yaml(path, *, default=None, allow_missing=False) -> dict[str, Any]`

- `path: str | os.PathLike[str]`：要读取的文件，`~` 会被展开。
- `default: Mapping[str, Any] | None = None`：文件缺失且 `allow_missing=True` 时返回（副本）的映射；`None` 表示 `{}`。
- `allow_missing: bool = False`：为 `True` 时文件缺失不算错误。
- 返回：解析后的普通 `dict`。
- 异常：`TypeError`（`path` 非 path-like，或 `default` 非映射）、`ValueError`（`path` 空白，或 `allow_missing` 非 bool）、`FileNotFoundError`（文件不存在且 `allow_missing=False`）、`ConfigError`（文件不可读 / 非合法 YAML / 根不是映射）。

#### `dump_yaml(data, path, *, sort_keys=False) -> None`

- `data: Mapping[str, Any] | Sequence[Any]`：映射或序列；裸标量被拒绝。
- `path: str | os.PathLike[str]`：目标文件，缺失的父目录会被创建。
- `sort_keys: bool = False`：是否按键名排序。
- 返回：`None`。
- 异常：`TypeError`（`data` 非映射/序列、`path` 非 path-like、`sort_keys` 非 bool）、`ValueError`（`path` 空白或 `data` 是字符串/bytes）、`ConfigError`（序列化或写入失败）。

#### `from_dict(cls, data, *, source=None, strict=False, aliases=None) -> Any`

- `cls: type`：要实例化的数据类类型。
- `data: Mapping[str, Any]`：字段名到值的映射。
- `source: str | None = None`：`data` 来源描述，仅出现在错误消息与日志中。
- `strict: bool = False`：为 `True` 时对不是 `cls` 字段的键抛出 `ConfigError`。
- `aliases: Mapping[str, str] | None = None`：`{旧扁平键: 点分新路径}`，在转换前以 `setdefault` 语义应用。
- 返回：`cls` 实例。
- 异常：`TypeError`（`cls` 非数据类、`data` 非映射、`strict` 非 bool、`aliases` 形状错误）、`ConfigError`（缺少必填键、值类型不符、嵌套块非映射、严格模式下存在未知键）。

#### `load_config(cls, path, *, env_prefix=None, aliases=None, strict=False) -> Any`

- `cls: type`：数据类类型。
- `path: str | os.PathLike[str]`：要读取的 YAML 文件。
- `env_prefix: str | None = None`：覆盖变量前缀，如 `"MYAPP"`；`None` 禁用覆盖。
- `aliases: Mapping[str, str] | None = None`：转发给 `from_dict`。
- `strict: bool = False`：转发给 `from_dict`。
- 返回：`cls` 实例。
- 异常：`TypeError`（`cls` 非数据类、`path` 非 path-like、`env_prefix` 非字符串/`None`）、`ValueError`（`env_prefix` 空白）、`FileNotFoundError`（文件不存在）、`ConfigError`（文件无效、缺少必填键或环境变量无法转换）。
- 说明：覆盖值来自 `{env_prefix}_{FIELD_NAME_UPPER}`，仅作用于顶层标量字段（`str`/`int`/`float`/`bool`/`pathlib.Path`）；数据类为冻结时通过 `object.__setattr__` 写入。

#### `resolve_path(value, *, root=None, roots_marker="models") -> Path`

- `value: str | os.PathLike[str]`：配置中写下的路径，`~` 会被展开。
- `root: str | os.PathLike[str] | None = None`：运行期根目录；`None` 表示当前工作目录。
- `roots_marker: str = "models"`：选择资产根的首个路径组成部分，必须是单个非空白片段。
- 返回：解析后的 `pathlib.Path`。
- 异常：`TypeError`（`value`/`root` 非 path-like，或 `roots_marker` 非字符串）、`ValueError`（`value`/`roots_marker` 空白，或标记含路径分隔符）。

#### `class YamlStore`

以文件名主干为主键、由目录支撑且带缓存的 YAML 文档集合。读取与换入在 `threading.RLock` 保护下进行。

- 构造：`YamlStore(directory, *, skip_names=(), skip_prefix="_", pattern="*.yaml")`
  - `directory: str | os.PathLike[str]`：要扫描的目录，构造时必须存在。
  - `skip_names: Iterable[str] = ()`：要排除的文件名主干。
  - `skip_prefix: str | None = "_"`：排除名称以此前缀开头的文件；`""` 或 `None` 禁用该规则。
  - `pattern: str = "*.yaml"`：目录内使用的 glob（不递归），不得含路径分隔符。
  - 异常：`TypeError`、`ValueError`、`FileNotFoundError`、`ConfigError`（见类 docstring）。
- 属性：`directory: Path`、`skip_names: frozenset[str]`、`skip_prefix: str`、`pattern: str`。
- `reload() -> dict[str, Any]`：重新扫描并替换缓存文档，返回新的 `{主干: 文档}` 映射浅拷贝；异常 `FileNotFoundError`、`ConfigError`。
- `keys() -> list[str]`：返回排序后的条目名。
- `get(name: str, default: Any = None) -> Any`：返回一个文档或 `default`；名称非字符串抛 `TypeError`。
- `__getitem__(name: str) -> Any`：返回一个文档；名称未知抛 `KeyError`（消息含可用名称）。
- `__contains__(name: object) -> bool`：是否为已加载条目，非字符串返回 `False`。
- `__iter__()`：遍历排序后的键快照。
- `__len__() -> int`：已加载条目数量。
- `__repr__() -> str`：调试表示，含目录、glob 与已加载键。

**使用示例**

```python
from dataclasses import dataclass, field
from reusable_model.io.yaml_config import load_config, dump_yaml, resolve_path

@dataclass(frozen=True)
class TlsConfig:
    enabled: bool = False
    ca_path: str = ""

@dataclass(frozen=True)
class AppConfig:
    host: str = "127.0.0.1"
    port: int = 8080
    tls: TlsConfig = field(default_factory=TlsConfig)

# 读取 service.yaml，允许旧扁平键 tls_enabled 迁移到 tls.enabled，
# 并允许用 MYAPP_PORT / MYAPP_LOG_LEVEL 之类的环境变量覆盖顶层标量字段。
cfg = load_config(
    AppConfig, "service.yaml",
    env_prefix="MYAPP",
    aliases={"tls_enabled": "tls.enabled"},
)

# 原子写回
dump_yaml({"host": cfg.host, "port": cfg.port}, "out.yaml", sort_keys=True)

# 逻辑资产路径解析
plan = resolve_path("models/sam3/weights.plan", root="/opt/app")
```

### 3.2 `binary_envelope.py`

**职责**：把结构化 JSON 头部与其后的 N 个二进制附件编码为单个不透明 blob。默认 `raw` 后端使用标准库 `struct` 的长度前缀帧（含幻数与版本号，便于检测截断）；`arrow` 后端使用 Arrow IPC 流，便于与列式消费方零拷贝互操作。另含 HTTP `Content-Type` 校验、枚举值规范化与图像负载编解码工具。

**模块常量**

- `ENVELOPE_VERSION = 1`：原始帧版本。
- `MIME_BINARY = "application/octet-stream"`：`struct` 帧式信封的媒体类型。
- `MIME_ARROW = "application/vnd.apache.arrow.stream"`：Arrow IPC 流后端的媒体类型。
- `MAGIC = b"ENV1"`：标识原始信封的四字节前缀。
- `RGB888_CHANNELS = 3`：原始 RGB888 的每像素字节数。
- `BACKENDS: tuple[str, ...] = ("raw", "arrow")`：`backend` 可接受的值。

**公开 API**

#### `class UnsupportedMediaTypeError(ValueError)`

负载携带了本端点无法解码的媒体类型时抛出；HTTP 层可据此映射为 415。

- 构造：`UnsupportedMediaTypeError(message: str, received: str = "", accepted: Sequence[str] = ())`。
- 属性：`message: str`、`received: str`、`accepted: tuple[str, ...]`。

#### `pack(header, *attachments, backend="raw") -> bytes`

- `header: Mapping[str, Any]`：可 JSON 序列化的映射；元素 0 始终是它。
- `*attachments: bytes`：按顺序排列的二进制负载，允许为空。
- `backend: str = "raw"`：`'raw'` 或 `'arrow'`。
- 返回：编码后的信封 `bytes`。
- 异常：`TypeError`（`header` 非映射 / 不可 JSON 序列化、附件非 bytes-like、`backend` 非字符串）、`ValueError`（`backend` 未知）、`ImportError`（`backend='arrow'` 且缺少 PyArrow）。

#### `unpack(data, *, backend="raw") -> tuple[dict[str, Any], list[bytes]]`

- `data: bytes`：由相同后端 `pack` 产生的字节。
- `backend: str = "raw"`：`'raw'` 或 `'arrow'`。
- 返回：`(头部映射, 附件列表)`。
- 异常：`TypeError`（`data` 非 bytes-like 或 `backend` 非字符串）、`ValueError`（幻数/版本/长度不匹配、主体非合法 UTF-8 JSON、元素 0 非 JSON 对象）、`ImportError`（缺少 PyArrow）。

#### `require_content_type(value, *, accepted=(MIME_BINARY, MIME_ARROW)) -> str`

- `value: str`：收到的 `Content-Type` 头值（参数会被剥离，比较不区分大小写）。
- `accepted: Sequence[str] = (MIME_BINARY, MIME_ARROW)`：可接受的媒体类型。
- 返回：规范化（小写、去参数）后的媒体类型。
- 异常：`TypeError`（`value` 非字符串，或 `accepted` 非字符串序列）、`ValueError`（`accepted` 为空或含非字符串）、`UnsupportedMediaTypeError`（媒体类型不被接受）。

#### `normalize_choice(value, allowed, *, aliases=None, default=None, field="value") -> str`

- `value: Any`：原始值；`None` 或空白字符串表示未提供。
- `allowed: Iterable[str]`：规范拼写。
- `aliases: Mapping[str, str] | None = None`：`{别名: 规范值}`。
- `default: str | None = None`：`value` 为 `None`/空白时返回；`None` 则把缺失视为错误。
- `field: str = "value"`：用于错误消息的字段名。
- 返回：规范值（必为 `allowed` 成员）。
- 异常：`TypeError`（`allowed`/`aliases`/`field` 类型错误）、`ValueError`（`allowed` 为空、别名目标或 `default` 不在 `allowed` 中、`value` 无匹配）。

#### `expected_rgb888_bytes(width, height) -> int`

- 参数：`width: int`、`height: int`（像素）。
- 返回：`width * height * 3`。
- 异常：`TypeError`（尺寸非整数）、`ValueError`（尺寸小于 1）。

#### `validate_rgb888(data, width, height) -> None`

- `data: bytes | memoryview`：像素缓冲区；`width` / `height` 为像素尺寸。
- 返回：`None`。
- 异常：`TypeError`（`data` 非 bytes-like 或尺寸非整数）、`ValueError`（尺寸小于 1，或缓冲区大小不等于 `width * height * 3`）。

#### `rgb888_to_array(data, width, height) -> np.ndarray`

- 参数同 `validate_rgb888`。
- 返回：`(height, width, 3)` 的 `uint8` 数组，是 `data` 的**零拷贝视图**（只读）。
- 异常：`TypeError`、`ValueError`。

#### `array_to_rgb888(arr) -> bytes`

- `arr: np.ndarray`：必须是 `(height, width, 3)` 的 `uint8` RGB 数组。
- 返回：行主序 RGB888 缓冲区 `bytes`。
- 异常：`TypeError`（非 `ndarray` 或 dtype 非 `uint8`）、`ValueError`（非三维或无 3 通道）。

#### `jpeg_to_array(data) -> np.ndarray`

- `data: bytes`：JPEG 文件字节。
- 返回：解码后的 `(height, width, 3)` `uint8` **RGB** 数组。
- 异常：`TypeError`（非 bytes-like）、`ValueError`（空或无法解码）、`ImportError`（缺少 OpenCV）。

#### `array_to_jpeg(arr, *, quality=90) -> bytes`

- `arr: np.ndarray`：`(height, width, 3)` 的 `uint8` RGB 数组。
- `quality: int = 90`：JPEG 质量 0-100。
- 返回：编码后的 JPEG `bytes`。
- 异常：`TypeError`（`arr` 非 uint8 `ndarray` 或 `quality` 非整数）、`ValueError`（形状错误、`quality` 越界或编码失败）、`ImportError`（缺少 OpenCV）。

#### `pack_image(*, rgb888=None, jpeg=None, width=0, height=0) -> dict[str, Any]`

- `rgb888: bytes | None = None`：原始 RGB888 像素缓冲区。
- `jpeg: bytes | None = None`：JPEG 文件字节。与 `rgb888` 互斥。
- `width: int = 0` / `height: int = 0`：帧尺寸（`0` = 未知）；对 `rgb888` 必填正数。
- 返回：含 `encoding`（`'rgb888'`/`'jpeg'`）、`width`、`height`、`attachments`（单元素名字列表）、`payload`（要传给 `pack` 的字节）的字典。
- 异常：`TypeError`（负载非 bytes-like 或尺寸非整数）、`ValueError`（两者都给或都未给、`rgb888` 缺少正尺寸、长度不匹配）。

**使用示例**

```python
from reusable_model.io.binary_envelope import pack, unpack, pack_image

# 一张 JPEG 图像：pack_image 给出要并入信封头部的名字与要作为附件的字节
image = pack_image(jpeg=open("frame.jpg", "rb").read(), width=1280, height=720)

blob = pack(
    {
        "kind": "frame",
        "frame_id": 7,
        "attachments": image["attachments"],
        "encoding": image["encoding"],
        "width": image["width"],
        "height": image["height"],
    },
    image["payload"],
)

header, attachments = unpack(blob)
assert header["frame_id"] == 7 and attachments[0] == image["payload"]
```

### 3.3 `detection_npz.py`

**职责**：把检测字典列表打包为一个自描述、压缩的 `.npz` 字节负载并支持还原。负载布局为：`count`、`masks` `(N, H, W)` 布尔掩码、`bboxes` `(N, 4)` float32、`confidences` `(N,)` float32、每个字符串字段一个 `(N,)` 对象数组，以及 `str_field_names`。约定与 `reusable_model.geometry.boxes` 一致（左上闭、右下开的边界框）；空列表往返后为 `[]`。

**模块常量**

- `DEFAULT_STRING_FIELDS: tuple[str, ...] = ("label", "name_zh", "waste_type", "category_id")`：默认捕获的字符串元数据字段名。

**公开 API**

#### `pack_detections(results, str_fields=DEFAULT_STRING_FIELDS) -> bytes`

- `results: Sequence[Mapping[str, Any]]`：检测结果；每项须带 `mask`（二维 array-like）、`bbox`（4+ 元素）与 `confidence`（数值）。缺少 `mask` 时用 `reusable_model.geometry.boxes.bbox_to_mask` 从 `bbox` 补齐。
- `str_fields: Sequence[str] = DEFAULT_STRING_FIELDS`：每个检测要存储的字符串字段名（缺失存为 `""`）。
- 返回：NPZ 负载 `bytes`。
- 异常：`ValueError`（掩码形状不一致或缺少必需数值字段）；`str_fields` 为 `None` 时抛 `TypeError`。

#### `unpack_detections(data) -> list[dict[str, Any]]`

- `data: bytes`：由 `pack_detections` 产生的负载；空 bytes 返回 `[]`。
- 返回：字典列表，每项含 `mask`（bool 数组）、`bbox`（四个浮点数列表）、`confidence`（float）及所存储的字符串字段。
- 异常：`ValueError`（`data` 不是有效的检测负载，包括 `None`）。

**使用示例**

```python
from reusable_model.io.detection_npz import pack_detections, unpack_detections

data = pack_detections([
    {
        "mask": [[0, 1], [1, 0]],
        "bbox": [0, 0, 2, 2],
        "confidence": 0.9,
        "label": "bottle",
        "name_zh": "瓶子",
    },
])

dets = unpack_detections(data)
print(dets[0]["label"], dets[0]["name_zh"], dets[0]["confidence"])
```

## 4. 模块间交互逻辑

- **检测结果的序列化链路**：推理进程产出检测字典列表（含 `mask`/`bbox`/`confidence` 及字符串字段）→ `pack_detections` 用 numpy 堆叠为数组并以 `np.savez_compressed` 写入内存缓冲 → 得到 `.npz` 字节负载 → 通过任意传输送到决策进程 → `unpack_detections` 用 `np.load` 还原为字典列表。缺少 `mask` 的检测在此链路开头借助 `reusable_model.geometry.boxes.bbox_to_mask` 补齐。
- **二进制信封承载多附件**：`pack_image` 先把一张图像描述为一个 `payload`（JPEG 或 RGB888 字节）与一个名字列表；调用方把该名字列表放进信封头部，并把 `payload` 作为 `*attachments` 之一传给 `pack`。`pack` 令元素 0 为 JSON 头部、其余按参数顺序为附件，编码为 `raw` 帧或 Arrow 流；`unpack` 反向拆出 `(header, attachments)`，消费方按头部中的名字对应到附件。
- **配置链路**：`load_yaml` 读文件 → `from_dict`（可带 `aliases`）实例化数据类 → `load_config` 额外叠加环境变量覆盖；`dump_yaml` 负责原子写回；`resolve_path` 把配置里的逻辑路径映射到运行期根；`YamlStore` 把一整个目录的 YAML 以文件名主干为键缓存起来。
- 三个实现文件之间没有相互 import，可独立使用；仅 `reusable_model.io` 这一命名空间通过 `__init__.py` 汇总它们。

## 5. 快速上手

```python
# 三种能力各自独立导入即可
from reusable_model.io.yaml_config import load_yaml, dump_yaml, load_config, resolve_path, YamlStore
from reusable_model.io.binary_envelope import pack, unpack, pack_image
from reusable_model.io.detection_npz import pack_detections, unpack_detections

# 1) YAML 往返
dump_yaml({"port": 8080, "host": "localhost"}, "cfg.yaml")
cfg = load_yaml("cfg.yaml")
assert cfg == {"port": 8080, "host": "localhost"}

# 2) 二进制信封
blob = pack({"kind": "frame"}, b"pixels", b"mask")
header, parts = unpack(blob)
assert header["kind"] == "frame" and parts == [b"pixels", b"mask"]

# 3) 检测批次
data = pack_detections([{"mask": [[1]], "bbox": [0, 0, 1, 1], "confidence": 0.5}])
assert unpack_detections(data)[0]["confidence"] == 0.5
```

## 6. 测试与验证

在子包目录内运行单元测试（`PYTHONDONTWRITEBYTECODE=1` 与 `-p no:cacheprovider` 用于避免在受限环境中写入缓存目录）：

```bash
cd /home/moke/Coding/Garden/reusable_model
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest tests/test_io.py tests/test_detection_npz.py -q -p no:cacheprovider
```

运行本子包全部模块的 doctest：

```bash
cd /home/moke/Coding/Garden
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/home/moke/Coding/Garden python3 -m pytest --doctest-modules reusable_model/io -q -p no:cacheprovider
```

语法检查：

```bash
cd /home/moke/Coding/Garden/reusable_model
python3 -m py_compile io/__init__.py io/yaml_config.py io/binary_envelope.py io/detection_npz.py
```
