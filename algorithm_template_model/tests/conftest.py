"""共享 fixture 与导入路径设置。"""

from __future__ import annotations

import sys
from pathlib import Path

# `reusable_model` 包位于 tests/ 的上一级目录，因此必须把它的父目录加入
# sys.path，`import reusable_model...` 才能正确解析。
_USE_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_USE_ROOT) not in sys.path:
    sys.path.insert(0, str(_USE_ROOT))
