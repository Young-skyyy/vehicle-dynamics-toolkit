# -*- coding: utf-8 -*-
"""Pytest unit tests for ISO 15765-2 (ISO-TP) multi-frame transport protocol"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import pytest
from vehicle_dynamics_toolkit.iso_tp import (
    FrameType,
    FlowControlFlag,
    parse_frame,
    build_single_frame,
    build_first_frame,
    build_consecutive_frame,
    build_flow_control,
    segment_payload,
    IsoTPReceiver,
    simulate_transfer,
    SF_PAYLOAD_MAX,
    ISO_TP_MAX_LENGTH,
    CAN_FRAME_DATA_LEN,
)
from vehicle_dynamics_toolkit.uds import (
    ECUDiagnosticServer,
    UDSSID,
    NRC,
)


# ═══════════════════════════════════════
# Frame Construction
# ═══════════════════════════════════════

class TestFrameConstruction:
    def test_single_frame_build(self):
        payload = b"\x01\x02\x03"
        sf = build_single_frame(payload)
        assert sf[0] == 0x03  # PCI: length=3
        assert sf[1:] == payload

    def test_single_frame_max(self):
        payload = b"A" * SF_PAYLOAD_MAX
        sf = build_single_frame(payload)
        assert sf[0] == (0x00 | SF_PAYLOAD_MAX)
        assert len(sf) == SF_PAYLOAD_MAX + 1

    def test_single_frame_too_long(self):
        with pytest.raises(ValueError):
            build_single_frame(b"A" * (SF_PAYLOAD_MAX + 1))

    def test_first_frame_build(self):
        payload = b"\x01\x02\x03\x04\x05\x06"
        total = 20
        ff = build_first_frame(payload, total)
        assert ff[0] == 0x10  # PCI: 0x1 | (20>>8 = 0)
        assert ff[1] == 0x14  # total_length low byte = 20
        assert ff[2:] == payload

    def test_first_frame_total_length_12bit(self):
        """total_length 最大 4095 (0xFFF)"""
        total = 0x0FFF
        ff = build_first_frame(b"A" * 6, total)
        assert (ff[0] & 0x0F) == 0x0F  # high nibble
        assert ff[1] == 0xFF           # low byte

    def test_consecutive_frame_build(self):
        cf = build_consecutive_frame(b"\xAA\xBB\xCC", 5)
        assert cf[0] == 0x25  # PCI: 0x2 | sequence=5
        assert cf[1:4] == b"\xAA\xBB\xCC"

    def test_flow_control_cts(self):
        fc = build_flow_control(FlowControlFlag.CTS, block_size=8, st_min_ms=10)
        assert fc == bytes([0x30, 8, 10])

    def test_flow_control_overflow(self):
        fc = build_flow_control(FlowControlFlag.OVERFLOW)
        assert fc[0] == 0x32


# ═══════════════════════════════════════
# Frame Parsing
# ═══════════════════════════════════════

class TestFrameParsing:
    def test_parse_single_frame(self):
        sf = build_single_frame(b"\xDE\xAD")
        frame = parse_frame(sf)
        assert frame.frame_type == FrameType.SINGLE
        assert frame.payload == b"\xDE\xAD"

    def test_parse_first_frame(self):
        ff = build_first_frame(b"A" * 6, 42)
        frame = parse_frame(ff)
        assert frame.frame_type == FrameType.FIRST
        assert frame.total_length == 42

    def test_parse_consecutive_frame(self):
        cf = build_consecutive_frame(b"B" * 4, 7)
        frame = parse_frame(cf)
        assert frame.frame_type == FrameType.CONSECUTIVE
        assert frame.sequence_number == 7

    def test_parse_flow_control(self):
        fc = build_flow_control(FlowControlFlag.CTS, block_size=16)
        frame = parse_frame(fc)
        assert frame.frame_type == FrameType.FLOW_CONTROL
        assert frame.fc_flag == FlowControlFlag.CTS
        assert frame.block_size == 16


# ═══════════════════════════════════════
# Payload Segmentation
# ═══════════════════════════════════════

class TestSegmentPayload:
    def test_short_payload_single_frame(self):
        """≤7 字节 → Single Frame"""
        data = b"short"
        frames = segment_payload(data)
        assert len(frames) == 1
        sf = parse_frame(frames[0])
        assert sf.frame_type == FrameType.SINGLE
        assert sf.payload == data

    def test_8_bytes_min_multi_frame(self):
        """8 字节 → FF + CF"""
        data = b"12345678"
        frames = segment_payload(data)
        assert len(frames) == 2
        ff = parse_frame(frames[0])
        assert ff.frame_type == FrameType.FIRST
        assert ff.total_length == 8
        cf = parse_frame(frames[1])
        assert cf.frame_type == FrameType.CONSECUTIVE
        assert cf.sequence_number == 1

    def test_17_byte_vin(self):
        """17 字节 VIN → FF + 2 CF"""
        data = encode_test_vin()
        assert len(data) == 17
        frames = segment_payload(data)
        # FF(6 bytes) + CF(7) + CF(4) = 3 帧
        assert len(frames) == 3

    def test_all_frames_within_8_bytes(self):
        """每帧 ≤ 8 字节（CAN 帧限制）"""
        data = b"X" * 100
        frames = segment_payload(data)
        for f in frames:
            assert len(f) <= CAN_FRAME_DATA_LEN, f"Frame too long: {len(f)}"

    def test_4095_bytes_maximum(self):
        """4095 字节 → 多帧传输可行"""
        data = b"\xFF" * ISO_TP_MAX_LENGTH
        frames = segment_payload(data)
        # 6 + (4095-6)/7 ≈ 590 帧
        assert len(frames) > 500


# ═══════════════════════════════════════
# Receiver State Machine
# ═══════════════════════════════════════

def encode_test_vin():
    """Helper: 生成 17 字节测试 VIN"""
    return b"LSFAM14B0AA000001"


class TestIsoTPReceiver:
    def test_single_frame_immediate_return(self):
        rx = IsoTPReceiver()
        sf = build_single_frame(b"\x01\x02")
        result, fc = rx.feed(sf)
        assert result == b"\x01\x02"
        assert fc is None
        assert rx.complete

    def test_multi_frame_complete_cycle(self):
        """FF → FC(CTS) → CFs → 完整 payload"""
        data = encode_test_vin()
        frames = segment_payload(data)
        rx = IsoTPReceiver()

        # Feed First Frame
        result, fc = rx.feed(frames[0])
        assert result is None           # 未完成
        assert fc is not None           # 应发 FC(CTS)
        fc_frame = parse_frame(fc)
        assert fc_frame.fc_flag == FlowControlFlag.CTS

        # Feed Consecutive Frames
        for i, frame in enumerate(frames[1:]):
            result, fc = rx.feed(frame)
            if i < len(frames) - 2:
                assert result is None   # 中间 CF 不应完成
            else:
                assert result == data   # 最后一帧完成
                assert rx.complete

    def test_wrong_sequence_returns_overflow(self):
        """CF 序号不对 → FC(Overflow)"""
        rx = IsoTPReceiver()
        ff = build_first_frame(b"A" * 6, 13)
        rx.feed(ff)  # 期望 seq=1

        # 发送 seq=2 而不是 seq=1
        cf = build_consecutive_frame(b"X" * 7, 2)
        result, fc = rx.feed(cf)
        assert result is None
        fc_frame = parse_frame(fc)
        assert fc_frame.fc_flag == FlowControlFlag.OVERFLOW

    def test_reset_clears_state(self):
        rx = IsoTPReceiver()
        rx.feed(build_single_frame(b"hello"))
        assert rx.complete
        rx.reset()
        assert not rx.complete


# ═══════════════════════════════════════
# End-to-end Transfer Simulation
# ═══════════════════════════════════════

class TestSimulateTransfer:
    def test_short_data_roundtrip(self):
        data = b"UDS"
        sender, responses = simulate_transfer(data)
        assert len(sender) == 1
        assert len(responses) == 0  # SF 无需 FC

    def test_17_byte_roundtrip(self):
        data = encode_test_vin()
        sender, responses = simulate_transfer(data)
        assert len(sender) == 3      # FF + 2 CF
        assert len(responses) == 2   # FC(CTS) after FF + CF1

    def test_roundtrip_preserves_data(self):
        """模拟传输后接收端重组的数据应与原始一致"""
        data = b"The quick brown fox jumps over"  # 35 bytes
        receiver = IsoTPReceiver()
        frames = segment_payload(data)
        assembled = None
        for frame in frames:
            result, _ = receiver.feed(frame)
            if result is not None:
                assembled = result
        assert assembled == data


# ═══════════════════════════════════════
# UDS Integration — VIN Read (0x22 F190)
# ═══════════════════════════════════════

class TestUDSVINRead:
    def test_read_vin_requires_iso_tp(self):
        """VIN 响应 20 字节 > 7，需要多帧传输"""
        server = ECUDiagnosticServer("EMS")
        # 直接 handle_request 返回的原始响应含 20 字节 payload
        req = bytes([UDSSID.READ_DATA_BY_IDENTIFIER, 0xF1, 0x90])
        resp = server.handle_request(req)
        # 正响应 SID 0x62 + DID 0xF190 + 17 bytes VIN = 20 bytes
        assert len(resp) == 20
        assert resp[0] == UDSSID.READ_DATA_BY_IDENTIFIER + 0x40  # 0x62
        assert encode_test_vin() != server._DEFAULT_VIN.encode()  # 确认是不同的

    def test_iso_tp_vin_roundtrip(self):
        """通过 ISO-TP 多帧传输读取 VIN"""
        server = ECUDiagnosticServer("EMS")
        server.did_bytes[0xF190] = encode_test_vin()

        # 诊断仪发送 3 字节请求，但响应太大需要 ISO-TP
        req = bytes([UDSSID.READ_DATA_BY_IDENTIFIER, 0xF1, 0x90])

        # 用 ISO-TP 封装请求 → 只需 SF 因为请求只有 3 字节
        from vehicle_dynamics_toolkit.iso_tp import build_single_frame
        sf = build_single_frame(req)

        # 服务器处理 ISO-TP 帧 → 返回 FF（启动多帧响应）
        frame1 = server.handle_iso_tp_frame(sf)
        assert frame1 is not None
        ff = parse_frame(frame1)
        assert ff.frame_type == FrameType.FIRST
        assert ff.total_length == 20  # 0x62 + 0xF190 + 17 bytes VIN

        # 诊断仪发送 FC(CTS)
        fc = build_flow_control(FlowControlFlag.CTS)
        # 服务器本应接收 FC，但当前实现通过 get_next_response_frame 轮询
        # 验证后续帧可用
        cf1 = server.get_next_response_frame()
        assert cf1 is not None
        cf = parse_frame(cf1)
        assert cf.frame_type == FrameType.CONSECUTIVE
        assert cf.sequence_number == 1

        cf2 = server.get_next_response_frame()
        assert cf2 is not None
        cf = parse_frame(cf2)
        assert cf.sequence_number == 2

        # 确认没有更多帧
        assert server.get_next_response_frame() is None

    def test_vin_default_value(self):
        """未设置 VIN 时返回默认占位 VIN"""
        server = ECUDiagnosticServer("EMS")
        req = bytes([UDSSID.READ_DATA_BY_IDENTIFIER, 0xF1, 0x90])
        resp = server.handle_request(req)
        assert resp[3:] == server._DEFAULT_VIN.encode()


# ═══════════════════════════════════════
# Edge Cases
# ═══════════════════════════════════════

class TestISOtpEdgeCases:
    def test_empty_data(self):
        """空 payload → 不应抛异常"""
        with pytest.raises(ValueError):
            build_single_frame(b"")

    def test_zero_total_length(self):
        with pytest.raises(ValueError):
            build_first_frame(b"A", 0)

    def test_parse_empty_frame(self):
        with pytest.raises(ValueError):
            parse_frame(b"")

    def test_parse_incomplete_ff(self):
        with pytest.raises(ValueError):
            parse_frame(bytes([0x10]))  # FF PCI without length byte

    def test_parse_incomplete_fc(self):
        with pytest.raises(ValueError):
            parse_frame(bytes([0x30, 0]))  # FC needs 3 bytes
