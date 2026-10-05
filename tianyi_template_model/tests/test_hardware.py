"""reusable_model.hardware（serial_discovery、modbus_gripper 空运行）的测试。"""

from __future__ import annotations

from pathlib import Path

import pytest

from reusable_model.hardware.modbus_gripper import ModbusGripper
from reusable_model.hardware.serial_discovery import list_by_id_ports, probe_port


class TestSerialDiscovery:
    def test_list_by_id_ports(self, tmp_path):
        d = Path(tmp_path)
        (d / "usb-FTDI_ABCD1234-if00-port0").write_text("", encoding="utf-8")
        (d / "usb-CH340_XYZ-if00-port1").write_text("", encoding="utf-8")
        ports = list_by_id_ports("*", by_id_dir=d)
        assert len(ports) == 2
        assert all(p.startswith(str(d)) for p in ports)

    def test_list_by_id_ports_pattern(self, tmp_path):
        d = Path(tmp_path)
        (d / "usb-FTDI_A-if00-port0").write_text("", encoding="utf-8")
        (d / "usb-CH340_B-if00-port1").write_text("", encoding="utf-8")
        ports = list_by_id_ports("*FTDI*", by_id_dir=d)
        assert len(ports) == 1

    def test_probe_port(self):
        assert probe_port("/dev/null", lambda port: True) is True
        assert probe_port("/dev/null", lambda port: False) is False
        assert probe_port("/dev/null", lambda port: (_ for _ in ()).throw(OSError("no"))) is False


class TestModbusGripper:
    def test_dry_run_never_touches_serial(self):
        g = ModbusGripper(port="/nonexistent/port", dry_run=True)
        # 空运行模式下不会打开端口。
        g.activate(timeout=0.01, poll_interval=0.001)
        g.close(target=0.0)
        g.squeeze_more()
        held = g.is_object_held()
        assert held in (True, False)
        assert g.dry_run_journal  # 每个动作都被记录

    def test_port_required(self):
        with pytest.raises(TypeError):
            ModbusGripper(port=123)  # type: ignore[arg-type]
        with pytest.raises(ValueError):
            ModbusGripper(port="   ")
