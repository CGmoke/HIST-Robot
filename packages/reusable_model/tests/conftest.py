"""共享 fixture 与导入路径设置。"""

from __future__ import annotations

import sys
from pathlib import Path

# `reusable_model` 源码位于 tests/ 上一级的 src/ 目录（src 布局），因此必须把
# src/ 加入 sys.path，`import reusable_model...` 才能正确解析。
_USE_ROOT = Path(__file__).resolve().parent.parent / "src"
if str(_USE_ROOT) not in sys.path:
    sys.path.insert(0, str(_USE_ROOT))
