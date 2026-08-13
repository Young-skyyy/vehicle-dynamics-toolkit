# -*- coding: utf-8 -*-
"""Pytest unit tests for can_demo.py"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import pytest
import os
from vehicle_dynamics_toolkit.can_demo import (
    CAN_MESSAGES,
    DTC_DATABASE,
    encode_signal,
    decode_signal,
    build_can_frame,
    parse_can_frame,
    VehicleECU,
    generate_dbc,
    simulate_dtc_check,
    generate_frame,
    _signal_bit_positions,
    _SIGNAL_NAME_EN,
)


# encode_signal / decode_signal

class TestSignalEncodeDecode:
    def test_encode_simple(self):
        sig = {"name": "test", "start": 0, "len": 8, "scale": 1.0, "offset": 0}
        assert encode_signal(50, sig) == 50

    def test_encode_with_scale_offset(self):
        sig = {"name": "coolant", "start": 0, "len": 8, "scale": 1.0, "offset": -40}
        # raw = (value - offset) / scale = (85 - (-40)) / 1 = 125
        assert encode_signal(85, sig) == 125

    def test_encode_throttle(self):
        sig = {"name": "throttle", "start": 0, "len": 8, "scale": 0.4, "offset": 0}
        # raw = (50 - 0) / 0.4 = 125
        assert encode_signal(50, sig) == 125

    def test_encode_engine_speed(self):
        sig = {"name": "rpm", "start": 8, "len": 16, "scale": 0.25, "offset": 0}
        # raw = (3000 - 0) / 0.25 = 12000
        assert encode_signal(3000, sig) == 12000

    def test_encode_vehicle_speed(self):
        sig = {"name": "speed", "start": 32, "len": 16, "scale": 0.01, "offset": 0}
        # raw = (80 - 0) / 0.01 = 8000
        assert encode_signal(80, sig) == 8000

    def test_decode_simple(self):
        sig = {"name": "test", "start": 0, "len": 8, "scale": 1.0, "offset": 0}
        assert decode_signal(50, sig) == 50

    def test_decode_with_offset(self):
        sig = {"name": "coolant", "start": 0, "len": 8, "scale": 1.0, "offset": -40}
        assert decode_signal(125, sig) == 85.0

    def test_roundtrip(self):
        """encode(decode) should return original raw value for simple signals."""
        sig = {"name": "test", "start": 0, "len": 8, "scale": 0.4, "offset": 0}
        raw = encode_signal(50, sig)
        decoded = decode_signal(raw, sig)
        assert decoded == 50.0

    def test_roundtrip_with_offset(self):
        sig = {"name": "temp", "start": 0, "len": 8, "scale": 1, "offset": -40}
        raw = encode_signal(85, sig)
        decoded = decode_signal(raw, sig)
        assert decoded == pytest.approx(85, rel=1e-6)

    def test_encode_clamps_to_max(self):
        """Value that exceeds max range should be clamped."""
        sig = {"name": "gear", "start": 0, "len": 4, "scale": 1, "offset": 0}
        # 4 bits → max raw = 15
        raw = encode_signal(100, sig)
        assert raw == 15

    def test_encode_clamps_to_min(self):
        sig = {"name": "gear", "start": 0, "len": 4, "scale": 1, "offset": 0}
        raw = encode_signal(-10, sig)
        assert raw == 0


# build_can_frame / parse_can_frame

class TestCANFrameBuildParse:
    @pytest.fixture
    def engine_msg_def(self):
        return CAN_MESSAGES["EngineData"]

    @pytest.fixture
    def abs_msg_def(self):
        return CAN_MESSAGES["ABS_WheelSpeed"]

    @pytest.fixture
    def body_msg_def(self):
        return CAN_MESSAGES["BodyControl"]

    def test_build_returns_8_bytes(self, engine_msg_def):
        signal_values = [30, 2000, 85, 60, 40]
        data = build_can_frame(engine_msg_def, signal_values)
        assert len(data) == 8
        assert all(0 <= b <= 255 for b in data)

    def test_roundtrip_engine_data(self, engine_msg_def):
        signal_values = [30, 2000, 85, 60, 40]
        data = build_can_frame(engine_msg_def, signal_values)
        parsed = parse_can_frame(data, engine_msg_def)
        assert parsed["节气门位置"] == pytest.approx(30, rel=0.05)
        assert parsed["发动机转速"] == pytest.approx(2000, rel=0.05)
        assert parsed["冷却液温度"] == pytest.approx(85, rel=0.05)
        assert parsed["车速"] == pytest.approx(60, rel=0.05)

    def test_roundtrip_abs_wheel_speed(self, abs_msg_def):
        signal_values = [80, 80.2, 79.8, 80.1]
        data = build_can_frame(abs_msg_def, signal_values)
        parsed = parse_can_frame(data, abs_msg_def)
        assert parsed["左前轮速"] == pytest.approx(80, rel=0.05)
        assert parsed["右前轮速"] == pytest.approx(80.2, rel=0.05)

    def test_roundtrip_body_control(self, body_msg_def):
        signal_values = [0, 0, 0, 0, 1, 0, 0, 0]
        data = build_can_frame(body_msg_def, signal_values)
        parsed = parse_can_frame(data, body_msg_def)
        assert parsed["左前门"] == 0
        assert parsed["近光灯"] == 1
        assert parsed["后备箱"] == 0

    def test_build_all_messages(self):
        """Every message in CAN_MESSAGES should be buildable."""
        for name, msg_def in CAN_MESSAGES.items():
            n_signals = len(msg_def["signals"])
            dummy_values = [0] * n_signals
            data = build_can_frame(msg_def, dummy_values)
            assert len(data) == 8, f"Failed building {name}"

    def test_parse_all_messages(self):
        """Every message should be parseable from zero data."""
        for name, msg_def in CAN_MESSAGES.items():
            parsed = parse_can_frame([0] * 8, msg_def)
            assert len(parsed) == len(msg_def["signals"]), f"Failed parsing {name}"


# VehicleECU

class TestVehicleECU:
    def test_initial_state(self):
        ecu = VehicleECU()
        assert ecu.rpm == 800
        assert ecu.speed == 0
        assert ecu.coolant_temp == 25
        assert ecu.gear == 0
        assert ecu.soc == 80.0
        assert ecu.brake_pressure == 0
        assert not ecu.accelerating

    def test_update_does_not_exceed_bounds(self):
        ecu = VehicleECU()
        for _ in range(500):  # simulate 5 seconds
            ecu.update(0.01)
            assert 800 <= ecu.rpm <= 6000
            assert 0 <= ecu.speed <= 120
            assert 25 <= ecu.coolant_temp <= 95
            assert ecu.soc > 0
            assert 0 <= ecu.gear <= 5
            assert 0 <= ecu.brake_pressure <= 5

    def test_speed_increases_when_accelerating(self):
        ecu = VehicleECU()
        ecu.update(0.01)
        assert ecu.accelerating  # should start accelerating
        assert ecu.rpm >= 800

    def test_gear_changes_with_speed(self):
        ecu = VehicleECU()
        ecu.speed = 50
        ecu.update(0.01)
        # Speed > 40 should give gear >= 4
        assert ecu.gear >= 4


# generate_frame

class TestGenerateFrame:
    def test_engine_data_returns_8_bytes(self):
        ecu = VehicleECU()
        data = generate_frame("EngineData", CAN_MESSAGES["EngineData"], ecu, 0)
        assert len(data) == 8

    def test_all_message_types_return_8_bytes(self):
        ecu = VehicleECU()
        for name in CAN_MESSAGES:
            data = generate_frame(name, CAN_MESSAGES[name], ecu, 5.0)
            assert len(data) == 8, f"{name} returned {len(data)} bytes"


# DTC 故障码

class TestDTC:
    def test_dtc_database_not_empty(self):
        assert len(DTC_DATABASE) > 0

    def test_all_dtcs_have_required_keys(self):
        for code, dtc in DTC_DATABASE.items():
            assert "desc" in dtc
            assert "ecu" in dtc
            assert len(dtc["desc"]) > 0
            assert len(dtc["ecu"]) > 0

    def test_simulate_dtc_check_runs(self):
        """simulate_dtc_check should return a dict with active_codes and details."""
        result = simulate_dtc_check()
        assert "active_codes" in result
        assert "details" in result


# generate_dbc

class TestDBCGeneration:
    def test_generates_file(self, tmp_path):
        filepath = str(tmp_path / "test.dbc")
        generate_dbc(filepath=filepath, baudrate=500000)
        assert os.path.exists(filepath)

    def test_dbc_contains_message_ids(self, tmp_path):
        filepath = str(tmp_path / "test.dbc")
        generate_dbc(filepath=filepath)
        with open(filepath, "r", encoding="utf-8") as f:
            content = f.read()
        for name, msg_def in CAN_MESSAGES.items():
            assert str(msg_def["id"]) in content, f"Missing ID {msg_def['id']} for {name}"

    def test_dbc_contains_baudrate(self, tmp_path):
        """波特率保留在函数参数/返回值里，但不写入 BS_（cantools 不解析 BS_ 后的数值）"""
        filepath = str(tmp_path / "test.dbc")
        info = generate_dbc(filepath=filepath, baudrate=500000)
        assert info["baudrate"] == 500000
        with open(filepath, "r", encoding="utf-8") as f:
            content = f.read()
        assert "BS_:" in content
        assert "500000" not in content

    def test_dbc_contains_signal_definitions(self, tmp_path):
        filepath = str(tmp_path / "test.dbc")
        generate_dbc(filepath=filepath)
        with open(filepath, "r", encoding="utf-8") as f:
            content = f.read()
        # Check that engine speed signal exists
        assert "SG_" in content
        assert "GenMsgCycleTime" in content

    def test_dbc_intel_byte_order(self, tmp_path):
        """车速信号设为 Intel 字节序，DBC 应输出 @1，信号名为 ASCII"""
        filepath = str(tmp_path / "test.dbc")
        generate_dbc(filepath=filepath)
        with open(filepath, "r", encoding="utf-8") as f:
            content = f.read()
        # VehicleSpeed(车速) 是 Intel -> DBC 输出 @1
        assert "VehicleSpeed" in content
        assert "车速" not in content  # 信号名必须为 ASCII
        assert "@1" in content  # 至少有一个 Intel 信号

    def test_dbc_spec_compliant(self, tmp_path):
        """回归测试：信号名 ASCII、接收方逗号分隔、BS_ 不带数值"""
        filepath = str(tmp_path / "test.dbc")
        generate_dbc(filepath=filepath)
        with open(filepath, "r", encoding="utf-8") as f:
            content = f.read()
        assert "BS_:" in content
        assert "BS_: " not in content
        for line in content.splitlines():
            if line.startswith(" SG_"):
                name = line.split(":", 1)[0].replace("SG_", "").strip()
                assert name.isascii(), f"信号名必须为 ASCII: {name!r}"
                receivers = line.rsplit('"', 1)[1].strip()
                assert "," in receivers, f"接收方必须用逗号分隔: {line!r}"

    def test_teardown_cleanup(self, tmp_path):
        """Generated file should be valid and not empty."""
        filepath = str(tmp_path / "test.dbc")
        generate_dbc(filepath=filepath)
        assert os.path.getsize(filepath) > 100  # should be a substantial file


# CAN_MESSAGES definition integrity

class TestCANMessages:
    def test_all_messages_have_required_fields(self):
        required = {"id", "cycle_ms", "desc", "signals"}
        for name, msg in CAN_MESSAGES.items():
            for field in required:
                assert field in msg, f"{name} missing {field}"

    def test_all_signals_have_required_fields(self):
        required = {"name", "start", "len", "scale", "offset", "unit", "byte_order"}
        for name, msg in CAN_MESSAGES.items():
            for sig in msg["signals"]:
                for field in required:
                    assert field in sig, f"{name}.{sig.get('name', '?')} missing {field}"

    def test_signal_bit_positions_no_overlap(self):
        """Signals within a message must not overlap on the actual wire bits."""
        for name, msg in CAN_MESSAGES.items():
            bits_used = set()
            for sig in msg["signals"]:
                positions = _signal_bit_positions(
                    sig["start"], sig["len"], sig.get("byte_order", "motorola")
                )
                wire_bits = {(b, bit) for b, bit, _ in positions}
                overlap = bits_used & wire_bits
                assert not overlap, (
                    f"Overlap in {name} signal {sig['name']}: {sorted(overlap)}"
                )
                bits_used |= wire_bits

    def test_all_signals_fit_in_64_bits(self):
        """Every signal bit must land in one of the 8 payload bytes."""
        for name, msg in CAN_MESSAGES.items():
            for sig in msg["signals"]:
                positions = _signal_bit_positions(
                    sig["start"], sig["len"], sig.get("byte_order", "motorola")
                )
                for byte_idx, bit_in_byte, _ in positions:
                    assert 0 <= byte_idx < 8, (
                        f"{name}.{sig['name']} bit lands in byte {byte_idx}"
                    )
                    assert 0 <= bit_in_byte <= 7

    def test_byte_order_values_are_valid(self):
        """byte_order 只能是 motorola 或 intel"""
        for name, msg in CAN_MESSAGES.items():
            for sig in msg["signals"]:
                bo = sig["byte_order"]
                assert bo in ("motorola", "intel"), (
                    f"{name}.{sig['name']} invalid byte_order: {bo}"
                )


class TestIntelByteOrder:
    """Intel (小端) 字节序编解码 roundtrip 测试"""

    @pytest.fixture
    def intel_msg(self):
        return {
            "id": 0x100,
            "cycle_ms": 10,
            "desc": "Intel 测试报文",
            "signals": [
                {"name": "sig16", "start": 8,  "len": 16, "scale": 0.1, "offset": 0, "unit": "", "byte_order": "intel"},
                {"name": "sig8",  "start": 7,  "len": 8,  "scale": 1,   "offset": 0, "unit": "", "byte_order": "motorola"},
                {"name": "sig32", "start": 24, "len": 32, "scale": 0.01, "offset": 0, "unit": "", "byte_order": "intel"},
            ],
        }

    def test_intel_16bit_roundtrip(self, intel_msg):
        """sig16 scale=0.1, 物理值 50.0 → raw=500 → 应解码回 50.0"""
        data = build_can_frame(intel_msg, [50.0, 0, 0])
        parsed = parse_can_frame(data, intel_msg)
        assert parsed["sig16"] == pytest.approx(50.0, rel=0.05)

    def test_intel_32bit_roundtrip(self, intel_msg):
        """sig32 scale=0.01, 物理值 987.65 → raw=98765 → 应解码回 987.65"""
        data = build_can_frame(intel_msg, [0, 0, 987.65])
        parsed = parse_can_frame(data, intel_msg)
        assert parsed["sig32"] == pytest.approx(987.65, rel=0.01)

    def test_mixed_byte_order_roundtrip(self, intel_msg):
        """同一报文内 Motorola + Intel 信号混合编解码"""
        data = build_can_frame(intel_msg, [50.0, 200, 100.0])
        parsed = parse_can_frame(data, intel_msg)
        assert parsed["sig8"]  == pytest.approx(200, rel=0.05)
        assert parsed["sig16"] == pytest.approx(50.0, rel=0.05)
        assert parsed["sig32"] == pytest.approx(100.0, rel=0.01)

    def test_intel_signal_bit_positions(self):
        """验证 Intel 16-bit 信号从 start_bit=8 的位布局"""
        positions = _signal_bit_positions(start_bit=8, length=16, byte_order="intel")
        # Intel LSB first: i=0→bit 8, i=15→bit 23
        assert positions[0] == (1, 0, 0)     # byte 1, bit 0, shift 0 (LSB)
        assert positions[7] == (1, 7, 7)     # byte 1, bit 7, shift 7
        assert positions[8] == (2, 0, 8)     # byte 2, bit 0, shift 8
        assert positions[15] == (2, 7, 15)   # byte 2, bit 7, shift 15 (MSB)

    def test_motorola_signal_bit_positions(self):
        """Motorola 16-bit signal at start_bit=24 follows the DBC standard."""
        positions = _signal_bit_positions(start_bit=24, length=16, byte_order="motorola")
        # network start = 8*3 + (7-0) = 31: MSB at byte3 bit0, then toward byte5
        assert positions[0] == (3, 0, 15)    # byte 3, bit 0, shift 15 (MSB)
        assert positions[7] == (4, 1, 8)     # byte 4, bit 1, shift 8
        assert positions[8] == (4, 0, 7)     # byte 4, bit 0, shift 7
        assert positions[15] == (5, 1, 0)    # byte 5, bit 1, shift 0 (LSB)

    def test_motorola_non_byte_aligned(self):
        """Non-byte-aligned Motorola: start_bit=8 -> byte1 bit0, then byte2/3."""
        positions = _signal_bit_positions(start_bit=8, length=16, byte_order="motorola")
        assert positions[0] == (1, 0, 15)    # MSB at byte 1, bit 0
        assert positions[1] == (2, 7, 14)    # next bit wraps to byte 2, bit 7
        assert positions[-1] == (3, 1, 0)    # LSB at byte 3, bit 1

    def test_motorola_byte_aligned_16bit(self):
        """start_bit=15, len=16 occupies bytes 1..2 exactly, MSB first."""
        positions = _signal_bit_positions(start_bit=15, length=16, byte_order="motorola")
        assert positions[0] == (1, 7, 15)
        assert positions[7] == (1, 0, 8)
        assert positions[8] == (2, 7, 7)
        assert positions[15] == (2, 0, 0)


# cantools compatibility: the generated DBC must load in cantools without
# overlapping signals, and our bit layout must match cantools byte for byte.
# These are the regression tests for the Motorola big-endian layout bug.

class TestCantoolsCompatibility:
    """Regression tests: our layout matches the DBC standard / cantools."""

    @pytest.fixture
    def cantools(self):
        return pytest.importorskip("cantools")

    @pytest.fixture
    def db(self, cantools, tmp_path):
        filepath = str(tmp_path / "simulated_ecu.dbc")
        generate_dbc(filepath=filepath)
        return cantools.database.load_file(filepath)

    @staticmethod
    def _ascii_names(msg_def):
        return [_SIGNAL_NAME_EN.get(s["name"], s["name"]) for s in msg_def["signals"]]

    def test_dbc_loads_without_overlap(self, db):
        """cantools can load the generated DBC (no overlapping signals)."""
        assert len(db.messages) == len(CAN_MESSAGES)

    def test_encode_layout_matches_cantools(self, db):
        """Same raw values: our build_can_frame bytes == cantools bytes."""
        cases = {
            "EngineData": [30, 2000, 80, 60, 40],
            "BatteryStatus": [80, 380, 30, 30, 26],
            "ABS_WheelSpeed": [80, 80, 79, 80],
            "Transmission": [3, 80, 2500],
            "BodyControl": [0, 0, 0, 0, 1, 0, 0, 0],
        }
        for msg_name, values in cases.items():
            msg_def = CAN_MESSAGES[msg_name]
            raws = [encode_signal(v, s) for v, s in zip(values, msg_def["signals"])]
            ours = build_can_frame(msg_def, values)
            ct_msg = db.get_message_by_name(msg_name)
            theirs = ct_msg.encode(
                dict(zip(self._ascii_names(msg_def), raws)), scaling=False
            )
            assert bytes(ours) == theirs, f"{msg_name}: layout mismatch vs cantools"

    def test_decode_layout_matches_cantools(self, db):
        """cantools decodes our frame bytes back to the same raw values."""
        cases = {
            "EngineData": [30, 2000, 80, 60, 40],
            "BatteryStatus": [80, 380, 30, 30, 26],
            "ABS_WheelSpeed": [80, 80, 79, 80],
            "Transmission": [3, 80, 2500],
            "BodyControl": [0, 0, 0, 0, 1, 0, 0, 0],
        }
        for msg_name, values in cases.items():
            msg_def = CAN_MESSAGES[msg_name]
            raws = [encode_signal(v, s) for v, s in zip(values, msg_def["signals"])]
            data = build_can_frame(msg_def, values)
            ct_msg = db.get_message_by_name(msg_name)
            decoded = ct_msg.decode(bytes(data), scaling=False)
            got = [decoded[name] for name in self._ascii_names(msg_def)]
            assert got == raws, f"{msg_name}: cantools decode mismatch"

    def test_physical_encode_matches_cantools(self, db):
        """Clean physical values (exact raw) also encode identically.

        Values are chosen so (value - offset) / scale is an exact integer.
        (80.1 / 0.01 type float truncation is a separate encode_signal issue.)
        """
        cases = {
            "EngineData": [30, 2000, 80, 60, 40],
            "BatteryStatus": [80, 380, 30, 30, 26],
            "Transmission": [3, 80, 2500],
            "BodyControl": [0, 0, 0, 0, 1, 0, 0, 0],
        }
        for msg_name, values in cases.items():
            msg_def = CAN_MESSAGES[msg_name]
            ours = build_can_frame(msg_def, values)
            ct_msg = db.get_message_by_name(msg_name)
            theirs = ct_msg.encode(
                dict(zip(self._ascii_names(msg_def), values)), scaling=True
            )
            assert bytes(ours) == theirs, f"{msg_name}: physical encode mismatch"
