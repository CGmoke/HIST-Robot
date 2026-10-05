"""二进制信封：一个 JSON 头部加上 N 个二进制附件，合并为单个 blob。

该模式来自消息流水线：它们必须把一个*帧*——结构化元数据与一张数兆字节的图像
并排——通过只承载单个不透明主体的传输（一次 HTTP POST、一个消息队列负载、一列
Arrow 数据）发送。把两者拆成独立请求会多花一次往返，更糟的是，会失去"元数据
与像素属于同一体"这一保证。

这里采用的布局是 Apache Arrow 系流水线推广开来的那一种：负载是一个**二进制
数组**，其元素 ``0`` 为 UTF-8 的 JSON 头部，其余元素为附件，顺序由头部指定。
两种后端都能产生该数组：

``backend='raw'``（默认）
    标准库 :mod:`struct` 帧。无第三方依赖、无模式，且精确的字节布局记录在下方，
    使非 Python 对端也能用一页代码实现它。

``backend='arrow'``
    基于 ``pa.binary()`` 数组的 Arrow IPC 流。仅当对端已经使用 Arrow 时才选它：
    它能换来与列式消费方的零拷贝互操作，代价是一个重量级依赖。PyArrow 是惰性
    导入的，因此*本模块在没有它的机器上也能干净导入*。

原始帧布局（所有整数均为小端序）
---------------------------------------------
::

    offset  size  field
    0       4     MAGIC            b'ENV1'
    4       1     version          ENVELOPE_VERSION
    5       4     element_count    number of elements, header included
    9       ...   element_count x (uint64 length | length bytes)

元素 0 始终是 JSON 头部；元素 1..N 是附件。逐元素的 ``uint64`` 长度前缀正是
截断可被检测的原因：一次不完整的读取会报告为 "element 2 declares 1234 bytes,
900 available"，而不是稍后表现为一张损坏的图像。

图像负载
--------------
:func:`pack_image` 泛化了每个面向摄像头的服务都必须做的 "原始 vs JPEG" 选择。
这个权衡很直白，值得说明：原始 RGB888 在两端都消耗**零**编解码 CPU，但单帧
1280x720 就有 2.76 MB，因此只有当生产者与消费者共享一台机器（或一条快速链路）
且 CPU 正忙于推理时才有意义。JPEG 体积大约小 20 倍，但在*两端*都要付出几毫秒
CPU，是跨网络时的正确选择。两者在构造上互斥——同时发送意味着为一张图像付出
两种代价。

示例
-------
::

    from reusable_model.io.binary_envelope import pack, unpack, pack_image

    image = pack_image(jpeg=open("frame.jpg", "rb").read(), width=1280, height=720)
    blob = pack(
        {"kind": "frame", "frame_id": 7, "attachments": image["attachments"],
         "encoding": image["encoding"], "width": image["width"], "height": image["height"]},
        image["payload"],
    )
    header, attachments = unpack(blob)
    assert header["frame_id"] == 7 and attachments[0] == image["payload"]
"""

from __future__ import annotations

import io
import json
import logging
import struct
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

logger = logging.getLogger(__name__)

__all__ = [
    "ENVELOPE_VERSION",
    "MIME_BINARY",
    "MIME_ARROW",
    "MAGIC",
    "RGB888_CHANNELS",
    "BACKENDS",
    "UnsupportedMediaTypeError",
    "pack",
    "unpack",
    "require_content_type",
    "normalize_choice",
    "expected_rgb888_bytes",
    "validate_rgb888",
    "rgb888_to_array",
    "array_to_rgb888",
    "jpeg_to_array",
    "array_to_jpeg",
    "pack_image",
]

#: 写入每个信封的原始帧格式版本。仅当布局改变时才递增它，并同时在 :func:`unpack`
#: 中加入迁移分支。
ENVELOPE_VERSION = 1

#: :mod:`struct` 帧式信封（不透明字节流）的媒体类型。
MIME_BINARY = "application/octet-stream"

#: Arrow IPC 流后端的媒体类型。
MIME_ARROW = "application/vnd.apache.arrow.stream"

#: 标识原始信封的四字节前缀，使错误的主体（HTML 错误页、JSON 错误对象、
#: 来自其他协议的流）在第一个字节就响亮地失败，而不是被误解析成一个头部长度。
MAGIC = b"ENV1"

#: 原始 RGB888 布局的每像素字节数（每个通道一个无符号字节）。
RGB888_CHANNELS = 3

#: ``backend`` 参数可接受的值。
BACKENDS: tuple[str, ...] = ("raw", "arrow")

#: ``<4sBI`` == magic | uint8 版本号 | uint32 元素个数。
_FRAME_HEADER = struct.Struct("<4sBI")

#: ``<Q`` == uint64 元素长度前缀。
_LENGTH_PREFIX = struct.Struct("<Q")

#: PyArrow 是可选的；这段文本让失败信息更具可操作性。
_PYARROW_HINT = "pip install pyarrow"

#: OpenCV 是可选的；只有 JPEG 辅助函数需要它。
_CV2_HINT = "pip install opencv-python"


class UnsupportedMediaTypeError(ValueError):
    """负载到达时携带了本端点无法解码的媒体类型。

    使用一个专用类型（而非裸的 ``ValueError``），才能让 HTTP 层在一个 ``except``
    子句里把该失败映射为 **415 Unsupported Media Type**——这是"你的
    ``Content-Type`` 不对"的正确状态码，而 400（"你的主体格式错误"）才是解码器
    错误应当产生的状态。在网络框架之外它仍然是 ``ValueError``，因此通用的输入
    校验处理器可以继续工作。

    参数:
        message: 人类可读的说明。
        received: 被拒绝的媒体类型。
        accepted: 本可接受的媒体类型。

    异常:
        自身不抛出；抛出该异常的是 :func:`require_content_type`。

    示例:
        >>> err = UnsupportedMediaTypeError("bad type", "text/plain", ("application/octet-stream",))
        >>> err.received, err.accepted
        ('text/plain', ('application/octet-stream',))
        >>> isinstance(err, ValueError)
        True
    """

    def __init__(self, message: str, received: str = "", accepted: Sequence[str] = ()) -> None:
        super().__init__(message)
        self.message = message
        self.received = received
        self.accepted: tuple[str, ...] = tuple(accepted)


def _require_backend(backend: Any) -> str:
    """校验并规范化 ``backend`` 参数。

    参数:
        backend: 调用方提供的后端名。

    返回:
        转为小写后的后端名。

    异常:
        TypeError: 如果 ``backend`` 不是字符串。
        ValueError: 如果它不是 :data:`BACKENDS` 中的一员；错误消息会列出它们。

    示例:
        >>> _require_backend("RAW")
        'raw'
    """
    if not isinstance(backend, str):
        raise TypeError(f"backend 必须是 str，实际为 {type(backend).__name__}：{backend!r}")
    name = backend.strip().lower()
    if name not in BACKENDS:
        raise ValueError(f"未知的 backend {backend!r}；允许值：{list(BACKENDS)}")
    return name


def _as_bytes(value: Any, *, what: str) -> bytes:
    """把 bytes-like 对象强制转换为不可变的 ``bytes``。

    接受 ``bytearray`` 和 ``memoryview``，是因为大型附件的生产者（摄像头驱动、
    解码器）经常交出其自身缓冲区上的视图；在此处转换为 ``bytes`` 只复制一次，
    而不是让一个可变缓冲区以别名方式留在帧里。

    参数:
        value: 候选缓冲区。
        what: 用于错误消息的描述。

    返回:
        以 ``bytes`` 表示的值。

    异常:
        TypeError: 如果 ``value`` 不是 bytes-like。``str`` 被特别指出，因为静默
            地编码它会掩盖真正的 bug。

    示例:
        >>> _as_bytes(bytearray(b"ab"), what="attachment")
        b'ab'
    """
    if isinstance(value, bytes):
        return value
    if isinstance(value, (bytearray, memoryview)):
        return bytes(value)
    if isinstance(value, str):
        raise TypeError(
            f"{what} 必须是 bytes-like，实际为 str {value!r}；请显式编码它 "
            f"（例如 value.encode('utf-8')）"
        )
    raise TypeError(f"{what} 必须是 bytes-like，实际为 {type(value).__name__}：{value!r}")


def _encode_header(header: Any) -> bytes:
    """把信封头部序列化为 UTF-8 的 JSON。

    使用紧凑分隔符：头部随每一帧传输，因此每个键省下的几个字节就是白得的带宽，
    并且没有换行也让头部可以安全地记录在一行日志里。

    参数:
        header: 可 JSON 序列化的映射。

    返回:
        UTF-8 编码的 JSON 对象。

    异常:
        TypeError: 如果 ``header`` 不是映射或无法 JSON 序列化。

    示例:
        >>> _encode_header({"kind": "frame"})
        b'{"kind":"frame"}'
    """
    if not isinstance(header, Mapping):
        raise TypeError(
            f"header 必须是映射，实际为 {type(header).__name__}：{header!r}"
        )
    try:
        return json.dumps(dict(header), ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise TypeError(f"header 无法 JSON 序列化：{exc}") from exc


def _import_pyarrow() -> Any:
    """按需导入 PyArrow，把它的缺失转化为可操作的错误。

    参数:
        无。

    返回:
        ``pyarrow`` 模块。

    异常:
        ImportError: 如果未安装 PyArrow，消息中会带上安装命令。若改为在模块作用域
            导入，会使整个模块——包括无依赖的 ``raw`` 后端——在没有 PyArrow 的
            机器上无法使用。

    示例:
        不直接调用；参见 :func:`pack`。
    """
    try:
        import pyarrow as pa  # noqa: PLC0415 - 可选依赖，延迟导入
    except ImportError as exc:
        raise ImportError(
            f"'arrow' 信封后端需要 PyArrow（{_PYARROW_HINT}）；"
            f"请使用 backend='raw' 以获得无依赖的帧"
        ) from exc
    return pa


def _import_cv2() -> Any:
    """按需导入 OpenCV，把它的缺失转化为可操作的错误。

    参数:
        无。

    返回:
        ``cv2`` 模块。

    异常:
        ImportError: 如果未安装 OpenCV，消息中会带上安装命令。

    示例:
        不直接调用；参见 :func:`jpeg_to_array`。
    """
    try:
        import cv2  # noqa: PLC0415 - 可选依赖，延迟导入
    except ImportError as exc:
        raise ImportError(f"JPEG 辅助函数需要 OpenCV（{_CV2_HINT}）") from exc
    return cv2


# ------------------------------------------------------------------- raw 后端


def _pack_raw(elements: Sequence[bytes]) -> bytes:
    """用原始的长度前缀布局把一组元素封装成帧。

    参数:
        elements: 元素 0 是头部，其余是附件。

    返回:
        完整的帧。

    示例:
        >>> _pack_raw([b"{}", b"ab"])[:9] == b"ENV1" + bytes([1]) + (2).to_bytes(4, "little")
        True
    """
    out = io.BytesIO()
    out.write(_FRAME_HEADER.pack(MAGIC, ENVELOPE_VERSION, len(elements)))
    for element in elements:
        out.write(_LENGTH_PREFIX.pack(len(element)))
        out.write(element)
    return out.getvalue()


def _unpack_raw(data: bytes) -> list[bytes]:
    """把原始帧拆回其元素，并校验每一个偏移量。

    参数:
        data: 完整的帧。

    返回:
        各元素，头部在最前。

    异常:
        ValueError: 如果缓冲区短于帧头部、幻数字节不匹配、版本不受支持，或声明的
            元素长度超过实际可用的字节数。每条消息都会同时给出期望值与观测值，
            因为 "truncated at 4096 bytes" 正是区分代理主体大小限制与读取撕裂的
            依据。

    示例:
        >>> _unpack_raw(_pack_raw([b"{}", b"ab"]))
        [b'{}', b'ab']
    """
    if len(data) < _FRAME_HEADER.size:
        raise ValueError(
            f"被截断的信封：帧头部需要 {_FRAME_HEADER.size} 字节，"
            f"但只有 {len(data)} 字节可用"
        )
    magic, version, count = _FRAME_HEADER.unpack_from(data, 0)
    if magic != MAGIC:
        raise ValueError(
            f"不是信封：期望幻数 {MAGIC!r}，实际为 {magic!r} "
            f"（主体是否来自其他协议，或是一个 HTML/JSON 错误页？）"
        )
    if version != ENVELOPE_VERSION:
        raise ValueError(
            f"不支持的信封版本：期望 {ENVELOPE_VERSION}，实际为 {version}"
        )
    if count < 1:
        raise ValueError(
            f"信封声明了 {count} 个元素；至少需要头部元素"
        )

    offset = _FRAME_HEADER.size
    elements: list[bytes] = []
    for index in range(count):
        if offset + _LENGTH_PREFIX.size > len(data):
            raise ValueError(
                f"被截断的信封：元素 {index} 在偏移 {offset} 处需要 {_LENGTH_PREFIX.size} 字节"
                f"的长度前缀，但 {len(data)} 字节中仅剩 {len(data) - offset} 字节"
            )
        (length,) = _LENGTH_PREFIX.unpack_from(data, offset)
        offset += _LENGTH_PREFIX.size
        end = offset + length
        if end > len(data):
            raise ValueError(
                f"被截断的信封：元素 {index} 声明了 {length} 字节，但仅有 "
                f"{len(data) - offset} 字节可用（缓冲区为 {len(data)} 字节）"
            )
        elements.append(data[offset:end])
        offset = end
    return elements


# ----------------------------------------------------------------- arrow 后端


def _pack_arrow(elements: Sequence[bytes]) -> bytes:
    """把各元素封装为基于 ``pa.binary()`` 数组的 Arrow IPC 流。

    Arrow 流自带帧格式，因此*不会*写入 :data:`MAGIC` 和
    :data:`ENVELOPE_VERSION`；应用层版本应放在头部字典里，这样 Arrow 消费方无需
    理解本模块即可读取它。

    参数:
        elements: 元素 0 是头部，其余是附件。

    返回:
        序列化后的 IPC 流。

    异常:
        ImportError: 如果未安装 PyArrow。

    示例:
        需要 PyArrow；参见 :func:`pack`。
    """
    pa = _import_pyarrow()
    import pyarrow.ipc as ipc  # noqa: PLC0415 - 仅在 PyArrow 存在时才可达

    array = pa.array(list(elements), type=pa.binary())
    sink = pa.BufferOutputStream()
    with ipc.new_stream(sink, pa.schema([pa.field("parts", array.type)])) as writer:
        writer.write_batch(pa.RecordBatch.from_arrays([array], names=["parts"]))
    return sink.getvalue().to_pybytes()


def _unpack_arrow(data: bytes) -> list[bytes]:
    """从 Arrow IPC 流中读回各元素。

    参数:
        data: 由 :func:`_pack_arrow` 产生的流。

    返回:
        各元素，头部在最前。

    异常:
        ImportError: 如果未安装 PyArrow。
        ValueError: 如果流中不含任何行，或某个单元格既不是 bytes 也不是文本
            （即模式不是本模块写入的 ``pa.binary()`` 那一种）。

    示例:
        需要 PyArrow；参见 :func:`unpack`。
    """
    pa = _import_pyarrow()
    import pyarrow.ipc as ipc  # noqa: PLC0415 - 仅在 PyArrow 存在时才可达

    try:
        reader = ipc.open_stream(data)
    except (pa.ArrowInvalid, OSError) as exc:
        raise ValueError(f"无法读取 Arrow IPC 流：{exc}") from exc

    elements: list[bytes] = []
    while True:
        batch = reader.read_next_batch()
        if batch is None:
            break
        if batch.num_rows == 0:
            continue
        for cell in batch.column(0).to_pylist():
            if isinstance(cell, memoryview):
                cell = bytes(cell)
            elif isinstance(cell, str):
                cell = cell.encode("utf-8")
            elif cell is not None and not isinstance(cell, bytes):
                raise ValueError(
                    f"信封元素必须是二进制，实际为 {type(cell).__name__}：{cell!r}"
                )
            elements.append(b"" if cell is None else cell)
    if not elements:
        raise ValueError("空信封：Arrow 流中不含任何元素")
    return elements


# ------------------------------------------------------------------- 公开 API


def pack(header: Mapping[str, Any], *attachments: bytes, backend: str = "raw") -> bytes:
    """把一个 JSON 头部和若干二进制附件打包成单个 blob。

    元素 0 始终是头部；附件按参数顺序紧随其后。把这一不变量放在编码器内部
    （而不是让每个调用方自行拼装元素列表），才使 ``unpack`` 无需模式即可承诺
    返回 ``(dict, list[bytes])``。

    参数:
        header: 描述负载的、可 JSON 序列化的映射。应用通常自行添加
            ``"kind"``/``"version"`` 键，以及一个为每个 blob 命名的
            ``"attachments"`` 列表，这样消费方就能按含义而非下标找到附件。
        *attachments: 二进制负载，按顺序排列。空附件是合法的（一个恰好缺失的
            可选 blob 仍然占据它的位置）。
        backend: ``'raw'``（标准库帧，默认）或 ``'arrow'``。

    返回:
        编码后的信封。

    异常:
        TypeError: 如果 ``header`` 不是映射或无法 JSON 序列化、某个附件不是
            bytes-like，或 ``backend`` 不是字符串。
        ValueError: 如果 ``backend`` 未知。
        ImportError: 如果 ``backend='arrow'`` 且缺少 PyArrow
            （:func:`pip install pyarrow`）。

    示例:
        >>> blob = pack({"kind": "frame"}, b"pixels", b"mask")
        >>> header, parts = unpack(blob)
        >>> header["kind"], parts
        ('frame', [b'pixels', b'mask'])
    """
    name = _require_backend(backend)
    payload = _encode_header(header)
    elements = [payload] + [_as_bytes(a, what=f"attachments[{i}]") for i, a in enumerate(attachments)]
    if name == "arrow":
        return _pack_arrow(elements)
    return _pack_raw(elements)


def unpack(data: bytes, *, backend: str = "raw") -> tuple[dict[str, Any], list[bytes]]:
    """把信封拆分为头部和附件。

    参数:
        data: 由 :func:`pack` 以相同后端产生的字节。
        backend: ``'raw'``（默认）或 ``'arrow'``。

    返回:
        一个元组，包含解码后的头部映射，以及附件列表（按打包顺序，可能为空）。

    异常:
        TypeError: 如果 ``data`` 不是 bytes-like 或 ``backend`` 不是字符串。
        ValueError: 如果幻数字节、版本或任一元素长度与帧声明的不符，如果主体
            不是有效的 UTF-8 JSON，或如果元素 0 不是 JSON 对象。消息总是同时
            给出期望值与观测值。
        ImportError: 如果 ``backend='arrow'`` 且缺少 PyArrow。

    示例:
        >>> header, parts = unpack(pack({"n": 1}, b"x"))
        >>> header, parts
        ({'n': 1}, [b'x'])
        >>> unpack(b"NOPE" + b"\\x00" * 32)
        Traceback (most recent call last):
            ...
        ValueError: 不是信封：期望幻数 b'ENV1'，实际为 b'NOPE' ...
    """
    name = _require_backend(backend)
    if isinstance(data, str):
        raise TypeError(
            f"data 必须是 bytes-like，实际为长度为 {len(data)} 的 str；"
            f"请先解码传输主体"
        )
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise TypeError(f"data 必须是 bytes-like，实际为 {type(data).__name__}：{data!r}")
    blob = bytes(data)

    elements = _unpack_arrow(blob) if name == "arrow" else _unpack_raw(blob)
    try:
        header = json.loads(elements[0].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(
            f"信封头部不是有效的 UTF-8 JSON（{len(elements[0])} 字节）：{exc}"
        ) from exc
    if not isinstance(header, dict):
        raise ValueError(
            f"信封头部必须是 JSON 对象，实际为 {type(header).__name__}：{header!r}"
        )
    return header, list(elements[1:])


def require_content_type(value: str, *, accepted: Sequence[str] = (MIME_BINARY, MIME_ARROW)) -> str:
    """对照可接受的媒体类型校验一个 HTTP ``Content-Type``。

    参数（``; charset=utf-8``、``; boundary=...``）会被剥离，比较时不区分大小写，
    因为 RFC 9110 规定媒体类型本身不区分大小写，而各客户端对参数的写法并不一致。
    若直接比较原始头字符串，就会拒绝掉一个完全合法的
    ``application/octet-stream; charset=binary``。

    参数:
        value: 收到的 ``Content-Type`` 头值。
        accepted: 本端点能够解码的媒体类型。

    返回:
        规范化（转小写、去参数）后的媒体类型，便于调用方据此分派。

    异常:
        TypeError: 如果 ``value`` 不是字符串（在多数框架中缺失的头会以 ``None``
            到达；请在边界处传入 ``""`` 或处理 ``None``，以保持消息有意义）。
        ValueError: 如果 ``accepted`` 为空或含非字符串。
        UnsupportedMediaTypeError: 如果媒体类型不被接受。消息会指出收到的值并
            列出可接受的值——HTTP 层把该异常映射为
            **415 Unsupported Media Type**。

    示例:
        >>> require_content_type("Application/Octet-Stream; charset=binary")
        'application/octet-stream'
        >>> require_content_type("text/plain")
        Traceback (most recent call last):
            ...
        reusable_model.io.binary_envelope.UnsupportedMediaTypeError: ... 'text/plain'...
    """
    if not isinstance(value, str):
        raise TypeError(f"value 必须是 str 类型的 Content-Type，实际为 {type(value).__name__}：{value!r}")
    if isinstance(accepted, (str, bytes)) or not isinstance(accepted, Iterable):
        raise TypeError(f"accepted 必须是 str 序列，实际为 {accepted!r}")
    allowed = [str(item).strip().lower() for item in accepted]
    if not allowed:
        raise ValueError("accepted 必须至少列出一个媒体类型")
    if any(not item for item in allowed):
        raise ValueError(f"accepted 条目必须是非空白媒体类型，实际为 {list(accepted)!r}")

    media_type = value.split(";", 1)[0].strip().lower()
    if media_type not in allowed:
        raise UnsupportedMediaTypeError(
            f"不支持的媒体类型 {value!r}（规范化为 {media_type!r}）；"
            f"允许值：{allowed}",
            received=media_type,
            accepted=tuple(allowed),
        )
    return media_type


def normalize_choice(
    value: Any,
    allowed: Iterable[str],
    *,
    aliases: Mapping[str, str] | None = None,
    default: str | None = None,
    field: str = "value",
) -> str:
    """把一个自由形式的字符串字段规范化为固定取值集合中的一员。

    来自配置文件、查询字符串或别的团队客户端的类枚举字段，从来不会以规范拼写出现：
    ``"Edge"``、``" edge "`` 和 ``"local"`` 都表示同一件事。拒绝它们会产生支持
    工单；静默接受任何输入又会产生悄悄为假的 ``if mode == "edge"`` 分支。本函数
    走中间路线：去除空白、转小写、应用一张显式的别名表、回退到显式默认值，并在
    无任何匹配时抛出带完整可接受值列表的异常。

    返回的字符串始终是规范化后的 ``allowed`` 集合成员，因此调用方可以直接用
    ``==`` 比较而无需再次规范化。

    参数:
        value: 原始值。``None`` 或空白字符串表示 "未提供"。
        allowed: 规范拼写。在去除空白/转小写后进行比较。
        aliases: ``{别名: 规范值}``。每个别名都按与 ``value`` 相同的方式规范化；
            每个目标必须是 ``allowed`` 的成员（会预先校验，因此表中的笔误不会
            悄悄产生一个任何 ``if`` 分支都处理不了的值）。
        default: 当 ``value`` 为 ``None``/空白时返回。取 ``None`` 时则改为把
            缺失的值视为错误。
        field: 用于错误消息的字段名。

    返回:
        规范值。

    异常:
        TypeError: 如果 ``allowed``/``aliases``/``field`` 类型不对。
        ValueError: 如果 ``allowed`` 为空、某个别名目标不在 ``allowed`` 中、
            ``default`` 不在 ``allowed`` 中，或 ``value`` 无任何匹配（消息会
            同时列出允许值与别名，因此无需阅读源码即可看清修复方式）。

    示例:
        >>> normalize_choice(" Local ", ("edge", "cloud"), aliases={"local": "edge"})
        'edge'
        >>> normalize_choice(None, ("edge", "cloud"), default="edge")
        'edge'
        >>> normalize_choice("gpu", ("edge", "cloud"))
        Traceback (most recent call last):
            ...
        ValueError: 无效的 value 'gpu'（规范化为 'gpu'）；允许值：['cloud', 'edge']
    """
    if isinstance(allowed, (str, bytes)) or not isinstance(allowed, Iterable):
        raise TypeError(f"allowed 必须是 str 的可迭代对象，实际为 {allowed!r}")
    canonical = [str(item).strip().lower() for item in allowed]
    if not canonical:
        raise ValueError("allowed 必须至少包含一个值")
    if any(not item for item in canonical):
        raise ValueError(f"allowed 条目必须是非空白字符串，实际为 {list(allowed)!r}")
    if not isinstance(field, str) or not field.strip():
        raise ValueError(f"field 必须是非空白 str，实际为 {field!r}")

    lookup: dict[str, str] = {}
    if aliases is not None:
        if not isinstance(aliases, Mapping):
            raise TypeError(f"aliases 必须是映射，实际为 {type(aliases).__name__}：{aliases!r}")
        for alias, target in aliases.items():
            if not isinstance(alias, str) or not isinstance(target, str):
                raise TypeError(f"aliases 条目必须是 str -> str，实际为 {alias!r} -> {target!r}")
            key = alias.strip().lower()
            value_canonical = target.strip().lower()
            if value_canonical not in canonical:
                raise ValueError(
                    f"别名 {alias!r} 映射到 {target!r}，但它不在允许值 "
                    f"{sorted(set(canonical))} 之中"
                )
            lookup[key] = value_canonical

    fallback: str | None = None
    if default is not None:
        if not isinstance(default, str):
            raise TypeError(f"default 必须是 str 或 None，实际为 {type(default).__name__}：{default!r}")
        fallback = default.strip().lower()
        if fallback not in canonical and fallback not in lookup:
            raise ValueError(
                f"default {default!r} 不在允许值 {sorted(set(canonical))} 之中"
            )
        fallback = lookup.get(fallback, fallback)

    if value is None or (isinstance(value, str) and not value.strip()):
        if fallback is not None:
            return fallback
        raise ValueError(
            f"{field} 是必需的；允许值：{sorted(set(canonical))}"
            + (f"（别名：{sorted(lookup)}）" if lookup else "")
        )

    key = str(value).strip().lower()
    if key in canonical:
        return key
    if key in lookup:
        return lookup[key]
    raise ValueError(
        f"无效的 {field} {value!r}（规范化为 {key!r}）；允许值：{sorted(set(canonical))}"
        + (f"，别名：{sorted(lookup)}" if lookup else "")
        + (f"，默认值：{fallback!r}" if fallback else "")
    )


# --------------------------------------------------------------- 图像负载


def _positive_int(value: Any, *, what: str) -> int:
    """把 ``value`` 强制转换为一个 ``int >= 1``。

    参数:
        value: 候选尺寸。
        what: 用于错误消息的名称。

    返回:
        一个正整数 ``int``。

    异常:
        TypeError: 如果 ``value`` 不是整数（拒绝 bool：``True`` 会悄悄变成 1 像素
            的尺寸）。
        ValueError: 如果该值小于 1。

    示例:
        >>> _positive_int(720, what="height")
        720
    """
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise TypeError(f"{what} 必须是 >= 1 的 int，实际为 {type(value).__name__}：{value!r}")
    out = int(value)
    if out < 1:
        raise ValueError(f"{what} 必须 >= 1，实际为 {value!r}")
    return out


def expected_rgb888_bytes(width: int, height: int) -> int:
    """返回 RGB888 缓冲区的精确字节大小。

    参数:
        width: 图像宽度（像素）。
        height: 图像高度（像素）。

    返回:
        ``width * height * 3``。

    异常:
        TypeError: 如果某个尺寸不是整数。
        ValueError: 如果某个尺寸小于 1。

    示例:
        >>> expected_rgb888_bytes(1280, 720)
        2764800
    """
    return _positive_int(width, what="width") * _positive_int(height, what="height") * RGB888_CHANNELS


def validate_rgb888(data: bytes | memoryview, width: int, height: int) -> None:
    """校验原始 RGB888 缓冲区的尺寸恰好符合预期。

    这项检查正是原始格式的全部意义所在：RGB888 缓冲区不带头部，因此错误的
    宽/高组合不会响亮地失败——它会重排成一张歪斜、沿对角线撕裂、但看上去仍
    "像"一帧的图像。比较字节数是唯一一种在边界处廉价捕获该问题的方法。

    参数:
        data: 像素缓冲区。
        width: 图像宽度（像素）。
        height: 图像高度（像素）。

    返回:
        ``None``。

    异常:
        TypeError: 如果 ``data`` 不是 bytes-like 或某个尺寸不是整数。
        ValueError: 如果某个尺寸小于 1，或缓冲区大小不等于
            ``width * height * 3``（消息中会给出两个数字）。

    示例:
        >>> validate_rgb888(b"\\x00" * 12, 2, 2)
        >>> validate_rgb888(b"\\x00" * 11, 2, 2)
        Traceback (most recent call last):
            ...
        ValueError: rgb888 缓冲区有 11 字节，但需要 2x2x3 = 12 字节...
    """
    if isinstance(data, str):
        raise TypeError(
            f"data 必须是 bytes-like，实际为 str；请编码它或传入原始缓冲区"
        )
    if not isinstance(data, (bytes, bytearray, memoryview, np.ndarray)):
        raise TypeError(f"data 必须是 bytes-like，实际为 {type(data).__name__}：{data!r}")
    expected = expected_rgb888_bytes(width, height)
    actual = data.nbytes if isinstance(data, np.ndarray) else len(data)
    if actual != expected:
        raise ValueError(
            f"rgb888 缓冲区有 {actual} 字节，但需要 {width}x{height}x{RGB888_CHANNELS} "
            f"= {expected} 字节（请检查随它一起传递的 width/height）"
        )


def rgb888_to_array(data: bytes | memoryview, width: int, height: int) -> np.ndarray:
    """把原始 RGB888 缓冲区包装成 ``(height, width, 3)`` 的 uint8 数组。

    结果是 ``data`` 的**零拷贝视图**，因此是只读的。这是有意为之：避免拷贝正是
    传输原始 RGB888 而非 JPEG 的全部意义，而每帧悄悄复制 2.76 MB 只会付出线路
    格式的代价却得不到它的好处。在结果上绘图前请先调用
    :meth:`numpy.ndarray.copy`。

    通道顺序是 **RGB**，不是 OpenCV 的 BGR——参见 :func:`jpeg_to_array`，它会
    显式转换。

    参数:
        data: 像素缓冲区，恰好 ``width * height * 3`` 字节。
        width: 图像宽度（像素）。
        height: 图像高度（像素）。

    返回:
        一个查看 ``data`` 的 ``(height, width, 3)`` ``uint8`` 数组。

    异常:
        TypeError: 如果 ``data`` 不是 bytes-like 或某个尺寸不是整数。
        ValueError: 如果缓冲区大小与尺寸不匹配。

    示例:
        >>> rgb888_to_array(bytes(range(12)), 2, 2).shape
        (2, 2, 3)
    """
    validate_rgb888(data, width, height)
    buffer = data.tobytes() if isinstance(data, np.ndarray) else bytes(data)
    return np.frombuffer(buffer, dtype=np.uint8).reshape(
        _positive_int(height, what="height"), _positive_int(width, what="width"), RGB888_CHANNELS
    )


def array_to_rgb888(arr: np.ndarray) -> bytes:
    """把 ``(height, width, 3)`` 的 uint8 数组序列化为原始 RGB888 字节。

    参数:
        arr: 一个 RGB 数组。必须是 ``uint8``：接受浮点数组会悄悄截断数值，而接受
            BGR 会产生红蓝互换的图像，且在每个只检查形状的测试里仍然"通过"。

    返回:
        行主序的 RGB888 缓冲区（``len == width * height * 3``）。

    异常:
        TypeError: 如果 ``arr`` 不是 ``ndarray`` 或其数据类型不是 ``uint8``。
        ValueError: 如果它不是三维或没有 3 个通道。

    示例:
        >>> array_to_rgb888(np.zeros((2, 2, 3), dtype=np.uint8)) == bytes(12)
        True
    """
    if not isinstance(arr, np.ndarray):
        raise TypeError(f"arr 必须是 numpy ndarray，实际为 {type(arr).__name__}：{arr!r}")
    if arr.dtype != np.uint8:
        raise TypeError(
            f"arr 的 dtype 必须是 uint8，实际为 {arr.dtype}；请显式转换 "
            f"（例如 np.clip(arr, 0, 255).astype(np.uint8)）"
        )
    if arr.ndim != 3 or arr.shape[2] != RGB888_CHANNELS:
        raise ValueError(
            f"arr 必须具有形状 (height, width, {RGB888_CHANNELS})，实际为 {arr.shape}"
        )
    return np.ascontiguousarray(arr).tobytes()


def jpeg_to_array(data: bytes) -> np.ndarray:
    """把 JPEG 缓冲区解码成 ``(height, width, 3)`` 的 uint8 **RGB** 数组。

    OpenCV 会解码为 BGR，因为那是它自身 I/O 函数期望的顺序；转成 RGB 的过程放在
    这里，是为了让本模块在处处都只讲一种颜色顺序（与 :func:`rgb888_to_array`
    相同）。跳过它正是那个经典的"模型照常运行，但每个检测结果都错了"的 bug。

    参数:
        data: JPEG 文件字节。

    返回:
        解码后的 RGB 数组。

    异常:
        TypeError: 如果 ``data`` 不是 bytes-like。
        ValueError: 如果缓冲区为空或无法解码（截断或损坏的 JPEG 会让
            ``cv2.imdecode`` 返回 ``None`` 而非抛异常，否则会在流水线下游变成
            令人困惑的 ``NoneType`` 属性错误）。
        ImportError: 如果未安装 OpenCV（:func:`pip install opencv-python`）。

    示例:
        >>> buf = array_to_jpeg(np.full((4, 6, 3), 200, dtype=np.uint8))
        >>> jpeg_to_array(buf).shape
        (4, 6, 3)
    """
    blob = _as_bytes(data, what="data")
    if not blob:
        raise ValueError("无法解码空的 JPEG 缓冲区")
    cv2 = _import_cv2()
    decoded = cv2.imdecode(np.frombuffer(blob, dtype=np.uint8), cv2.IMREAD_COLOR)
    if decoded is None:
        raise ValueError(
            f"无法解码 JPEG 负载（{len(blob)} 字节）：缓冲区被截断"
            f"或不是 JPEG 图像"
        )
    return cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB)


def array_to_jpeg(arr: np.ndarray, *, quality: int = 90) -> bytes:
    """把 RGB 的 uint8 数组编码为 JPEG。

    参数:
        arr: 一个 RGB（不是 BGR）的 ``(height, width, 3)`` ``uint8`` 数组；内部会
            交换通道顺序以匹配 OpenCV 编码器的期望。
        quality: JPEG 质量 0-100。``90`` 是机器视觉中常见的折中：对检测器而言
            视觉上无损，同时比原始 RGB888 大约小 20 倍。低于约 70 时，块状伪影
            开始影响检测分数。

    返回:
        编码后的 JPEG 字节。

    异常:
        TypeError: 如果 ``arr`` 不是 uint8 ``ndarray`` 或 ``quality`` 不是整数。
        ValueError: 如果数组形状不对、``quality`` 超出 0-100，或编码器失败。
        ImportError: 如果未安装 OpenCV。

    示例:
        >>> len(array_to_jpeg(np.zeros((8, 8, 3), dtype=np.uint8), quality=50)) > 0
        True
    """
    if not isinstance(arr, np.ndarray):
        raise TypeError(f"arr 必须是 numpy ndarray，实际为 {type(arr).__name__}：{arr!r}")
    if arr.dtype != np.uint8:
        raise TypeError(f"arr 的 dtype 必须是 uint8，实际为 {arr.dtype}")
    if arr.ndim != 3 or arr.shape[2] != RGB888_CHANNELS:
        raise ValueError(
            f"arr 必须具有形状 (height, width, {RGB888_CHANNELS})，实际为 {arr.shape}"
        )
    if isinstance(quality, bool) or not isinstance(quality, (int, np.integer)):
        raise TypeError(f"quality 必须是 0-100 的 int，实际为 {type(quality).__name__}：{quality!r}")
    if not 0 <= int(quality) <= 100:
        raise ValueError(f"quality 必须在 0-100 之间，实际为 {quality!r}")

    cv2 = _import_cv2()
    bgr = cv2.cvtColor(np.ascontiguousarray(arr), cv2.COLOR_RGB2BGR)
    ok, buffer = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise ValueError(f"对形状为 {arr.shape} 的数组进行 JPEG 编码失败")
    return buffer.tobytes()


def pack_image(
    *,
    rgb888: bytes | None = None,
    jpeg: bytes | None = None,
    width: int = 0,
    height: int = 0,
) -> dict[str, Any]:
    """描述一个图像负载，使其能够被附加到一个信封上。

    ``rgb888`` 与 ``jpeg`` **互斥**：它们是同一帧的两种编码，同时接受两者意味着
    为一张图像既付出原始 RGB888 的带宽*又*付出 JPEG 的 CPU，并且对 "哪一种才是
    权威的" 没有明确答案。这项检查发生在打包时（此时违规的调用方还在调用栈上），
    而不是在网络另一端的解包时。

    ``width``/``height`` 对 ``rgb888`` 是必需的（该缓冲区自身不带头部，因此尺寸
    是解释它的唯一依据），对 ``jpeg`` 则是可选的（它们可以从负载中恢复；``0``
    表示 "未提供"，对于想在解码前先分配内存的消费方而言，这是一个合法的提示值）。

    参数:
        rgb888: 原始 RGB888 像素缓冲区，或 ``None``。
        jpeg: JPEG 文件字节，或 ``None``。
        width: 帧宽度（像素，``0`` = 未知）。
        height: 帧高度（像素，``0`` = 未知）。

    返回:
        一个字典，包含：

        * ``encoding`` —— ``'rgb888'`` 或 ``'jpeg'``；
        * ``width`` / ``height`` —— 校验后的尺寸；
        * ``attachments`` —— 要合并进信封头部的单元素名字列表，便于消费方按名
          找到该 blob；
        * ``payload`` —— 作为附件传给 :func:`pack` 的字节。

    异常:
        TypeError: 如果某个负载不是 bytes-like 或某个尺寸不是整数。
        ValueError: 如果两个负载都给出、都没给出、``rgb888`` 在缺少正尺寸的情况下
            被使用，或其长度与 ``width * height * 3`` 不匹配。

    示例:
        >>> info = pack_image(rgb888=bytes(12), width=2, height=2)
        >>> info["encoding"], info["attachments"], len(info["payload"])
        ('rgb888', ['rgb888'], 12)
        >>> pack_image(rgb888=bytes(12), jpeg=b"\\xff\\xd8", width=2, height=2)
        Traceback (most recent call last):
            ...
        ValueError: rgb888 与 jpeg 互斥...
    """
    if rgb888 is not None and jpeg is not None:
        raise ValueError(
            f"rgb888 与 jpeg 互斥（得到 {len(_as_bytes(rgb888, what='rgb888'))} "
            f"和 {len(_as_bytes(jpeg, what='jpeg'))} 字节）；请只发送一种图像编码"
        )
    if rgb888 is None and jpeg is None:
        raise ValueError("需要一个图像负载：请传入 rgb888 或 jpeg 之一")

    if isinstance(width, bool) or not isinstance(width, (int, np.integer)):
        raise TypeError(f"width 必须是 >= 0 的 int，实际为 {type(width).__name__}：{width!r}")
    if isinstance(height, bool) or not isinstance(height, (int, np.integer)):
        raise TypeError(f"height 必须是 >= 0 的 int，实际为 {type(height).__name__}：{height!r}")
    if width < 0 or height < 0:
        raise ValueError(f"width/height 必须 >= 0，实际为 {width!r}x{height!r}")

    if rgb888 is not None:
        buffer = _as_bytes(rgb888, what="rgb888")
        if width <= 0 or height <= 0:
            raise ValueError(
                f"rgb888 要求正的 width/height，因为该缓冲区自身不携带 "
                f"头部；实际为 {width!r}x{height!r}"
            )
        validate_rgb888(buffer, int(width), int(height))
        encoding = "rgb888"
        payload = buffer
    else:
        payload = _as_bytes(jpeg, what="jpeg")
        if not payload:
            raise ValueError("jpeg 负载为空")
        encoding = "jpeg"

    logger.debug("pack_image：编码=%s，%dx%d（%d 字节）", encoding, width, height, len(payload))
    return {
        "encoding": encoding,
        "width": int(width),
        "height": int(height),
        "attachments": [encoding],
        "payload": payload,
    }
