"""reusable_model —— 从 Hulk/Zev 机器人代码栈中抽取的可复用构建块。

本包汇集了一批通用、自包含的 Python 模块，它们抽取自两个大型机器人项目
（行为编排、自定义导航、开放词汇感知与机械臂控制）。每个模块都满足：

* 仅依赖标准库与 :mod:`numpy`（可选的重型依赖在用到时才惰性导入）；
* 与来源项目解耦 —— 不包含绝对路径、不读取环境变量、不依赖项目专属配置；
* 在边界处校验输入，并以 参数/返回/异常 格式的 docstring 描述其 API。

按功能划分的目录结构：

* :mod:`reusable_model.geometry` —— 3D 旋转、边界框；
* :mod:`reusable_model.gridmap` —— 占据栅格、连通区域、距离场；
* :mod:`reusable_model.motion` —— 平面逆运动学、运动学速度曲线；
* :mod:`reusable_model.vision` —— 针孔相机、深度采样、点云、平面；
* :mod:`reusable_model.io` —— YAML 配置、NPZ 检测负载、二进制信封；
* :mod:`reusable_model.tracking` —— IoU 多目标跟踪、稳定连续帧门限；
* :mod:`reusable_model.hardware` —— 串口发现、Modbus 夹爪；
* :mod:`reusable_model.runtime` —— 子进程桥接、ctypes 加载、输出流过滤。

使用 ``import reusable_model.geometry.rotations3d``（或更短的 ``from reusable_model.geometry
import rotations3d``）即可开始。
"""

from __future__ import annotations

__version__ = "0.1.0"

__all__ = ["__version__"]
