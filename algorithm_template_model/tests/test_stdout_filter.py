"""reusable_model.runtime.stdout_filter 的测试。"""

from __future__ import annotations

from io import StringIO

import pytest

from reusable_model.runtime.stdout_filter import FilteredStream, install_filter, quiet_loggers


class TestFilteredStream:
    def test_drops_matching_lines(self):
        sink = StringIO()
        w = FilteredStream(["spam"], stream=sink)
        w.write("spam line\nkeep me\n")
        assert sink.getvalue() == "keep me\n"

    def test_drops_blank_line_after_dropped(self):
        sink = StringIO()
        w = FilteredStream(["spam"], stream=sink, drop_blank=True)
        w.write("spam line\n\nkeep\n")
        assert sink.getvalue() == "keep\n"

    def test_drop_blank_disabled(self):
        sink = StringIO()
        w = FilteredStream(["spam"], stream=sink, drop_blank=False)
        w.write("spam line\n\nkeep\n")
        assert sink.getvalue() == "\nkeep\n"

    def test_partial_line_flushed(self):
        sink = StringIO()
        w = FilteredStream(["spam"], stream=sink)
        w.write("no newline yet")
        assert sink.getvalue() == ""
        w.flush()
        assert sink.getvalue() == "no newline yet"

    def test_non_string_coerced(self):
        sink = StringIO()
        w = FilteredStream([], stream=sink)
        w.write(42)
        w.flush()
        assert sink.getvalue() == "42"

    def test_fileno_forwards(self):
        class FakeStream:
            def fileno(self):
                return 7

        w = FilteredStream(["x"], stream=FakeStream())  # type: ignore[arg-type]
        assert w.fileno() == 7

    def test_rejects_none_drop_list(self):
        with pytest.raises(TypeError):
            FilteredStream(None)  # type: ignore[arg-type]


class TestInstallFilter:
    def test_installs_on_requested_streams(self, monkeypatch):
        original_stdout = __import__("sys").stdout
        try:
            result = install_filter(["spam"], streams="stdout,stderr")
            assert result["stdout"] is not None
            assert result["stderr"] is not None
            assert result["stdout"].drop_substrings == ("spam",)
            # 第二次调用为空操作
            again = install_filter(["spam"], streams="stdout")
            assert again["stdout"] is None
        finally:
            __import__("sys").stdout = original_stdout


class TestQuietLoggers:
    def test_prefix_matching(self):
        import logging

        logging.getLogger("myapp.core").setLevel(logging.INFO)
        changed = quiet_loggers(["myapp"], level=logging.ERROR)
        assert changed >= 1
        assert logging.getLogger("myapp.core").level == logging.ERROR

    def test_rejects_none(self):
        with pytest.raises(TypeError):
            quiet_loggers(None)  # type: ignore[arg-type]
