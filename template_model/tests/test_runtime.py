"""reusable_model.runtime（subprocess_bridge、ctypes_loader）的测试。"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from reusable_model.runtime.ctypes_loader import (
    CFunction,
    NativeCallError,
    find_library_file,
    poll_until_ready,
)
from reusable_model.runtime.subprocess_bridge import SubprocessBridge


class TestSubprocessBridge:
    def test_send_lines_and_clean_close(self):
        # 子进程读取完 stdin 后，在哨兵信号下干净退出。
        code = "import sys\nfor line in sys.stdin:\n    sys.stdin  # consume\n"
        bridge = SubprocessBridge([sys.executable, "-u", "-c", code], startup_timeout=5.0)
        with bridge:
            assert bridge.connected
            assert bridge.send_line("hello")
            assert bridge.send(0.1, 0.0, 0.25)
        assert bridge.returncode == 0
        assert not bridge.connected

    def test_sentinel_closes_child(self):
        code = "import sys\nfor line in sys.stdin:\n    pass\n"
        bridge = SubprocessBridge([sys.executable, "-u", "-c", code], startup_timeout=5.0)
        bridge.start()
        assert bridge.send_line("go")
        bridge.close()
        assert bridge.returncode == 0

    def test_bad_argv_rejected(self):
        with pytest.raises(ValueError):
            SubprocessBridge([])
        with pytest.raises(TypeError):
            SubprocessBridge("/bin/echo")  # 裸字符串，而非序列


class TestCtypesLoader:
    def test_find_library_file(self, tmp_path):
        d = Path(tmp_path)
        (d / "libmylib.so").write_text("", encoding="utf-8")
        found = find_library_file(d, "mylib")
        assert found is not None
        assert found.name == "libmylib.so"
        assert find_library_file(d, "does-not-exist") is None

    def test_poll_until_ready(self):
        counter = {"n": 0}

        def probe():
            counter["n"] += 1
            return counter["n"] >= 3

        assert poll_until_ready(probe, timeout=1.0, interval=0.001) is True

    def test_poll_timeout(self):
        assert poll_until_ready(lambda: False, timeout=0.02, interval=0.001) is False

    def test_native_call_error(self):
        err = NativeCallError(7, "boom")
        assert err.code == 7
        assert "boom" in str(err)

    def test_cfunction_requires_func(self):
        with pytest.raises(TypeError):
            CFunction(None)  # type: ignore[arg-type]
