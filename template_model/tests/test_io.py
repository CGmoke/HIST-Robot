"""reusable_model.io.yaml_config 与 reusable_model.io.binary_envelope 的测试。"""

from __future__ import annotations

import pytest

from reusable_model.io.binary_envelope import pack, unpack
from reusable_model.io.yaml_config import dump_yaml, load_yaml


class TestYamlConfig:
    def test_dump_load_round_trip(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        data = {"name": "robot", "limits": [0.1, 0.2], "nested": {"a": 1}}
        dump_yaml(data, path)
        loaded = load_yaml(path)
        assert loaded == data

    def test_load_missing_with_default(self, tmp_path):
        loaded = load_yaml(tmp_path / "nope.yaml", default={"x": 1}, allow_missing=True)
        assert loaded == {"x": 1}

    def test_load_missing_rejected(self, tmp_path):
        with pytest.raises(FileNotFoundError):
            load_yaml(tmp_path / "nope.yaml")

    def test_load_invalid_yaml_raises(self, tmp_path):
        path = tmp_path / "bad.yaml"
        path.write_text("a: [1, 2\n", encoding="utf-8")
        with pytest.raises(Exception):
            load_yaml(path)


class TestBinaryEnvelope:
    def test_round_trip_raw(self):
        blob = pack({"kind": "img", "version": 1}, b"\x00\x01\x02", b"payload")
        header, attachments = unpack(blob, backend="raw")
        assert header == {"kind": "img", "version": 1}
        assert attachments == [b"\x00\x01\x02", b"payload"]

    def test_no_attachments(self):
        blob = pack({"kind": "meta"})
        header, attachments = unpack(blob)
        assert attachments == []

    def test_round_trip_empty_header(self):
        blob = pack({})
        header, attachments = unpack(blob)
        assert header == {}
        assert attachments == []

    def test_corrupt_payload_rejected(self):
        with pytest.raises(Exception):
            unpack(b"\xde\xad\xbe\xef")
