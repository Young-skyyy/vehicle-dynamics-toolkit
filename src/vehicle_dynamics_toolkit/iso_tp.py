# -*- coding: utf-8 -*-
"""
ISO 15765-2 (ISO-TP) 多帧传输协议实现
======================================

将超长诊断报文（>7 字节）拆分为 CAN 帧序列，在接收端重组。
UDS 诊断中读取 VIN（17 字节）、刷写固件等操作均依赖此协议。

帧类型 (N_PCI byte):
    Single Frame (SF):      0x0 + payload_length (4 bits)
    First Frame (FF):       0x1 + total_length (12 bits: 4+8)
    Consecutive Frame (CF): 0x2 + sequence_number (4 bits)
    Flow Control (FC):      0x3 + FC_flag (CTS=0/WAIT=1/Overflow=2)
"""

from __future__ import annotations
from dataclasses import dataclass
from enum import IntEnum

# ═══════════════════════════════════════════════════════════════════════════
# 协议常量
# ═══════════════════════════════════════════════════════════════════════════

# CAN 帧最大数据长度（标准 CAN 2.0A）
CAN_FRAME_DATA_LEN = 8

# 各帧类型的 payload 容量（不含 PCI 开销）
SF_PAYLOAD_MAX = 7    # Single Frame: 1 byte PCI + 7 data
FF_PAYLOAD_MAX = 6    # First Frame: 2 bytes PCI + 6 data
CF_PAYLOAD_MAX = 7    # Consecutive Frame: 1 byte PCI + 7 data

# 最大传输长度
ISO_TP_MAX_LENGTH = 4095  # 12-bit length field


class FrameType(IntEnum):
    """N_PCI 帧类型 (PCI byte 高 4 bits)"""
    SINGLE = 0x0
    FIRST  = 0x1
    CONSECUTIVE = 0x2
    FLOW_CONTROL = 0x3


class FlowControlFlag(IntEnum):
    """Flow Control 标志 (FC 帧 data[1] 低 4 bits)"""
    CTS = 0  # Continue To Send
    WAIT = 1  # Wait
    OVERFLOW = 2  # Overflow — 接收端缓冲区不足


# ═══════════════════════════════════════════════════════════════════════════
# 帧结构解析
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class IsoTPFrame:
    """解析后的 ISO-TP 帧"""
    frame_type: FrameType
    payload: bytes
    # Single Frame: 无
    # First Frame: total_length (int)
    # Consecutive Frame: sequence_number (int, 0-15)
    # Flow Control: fc_flag (FlowControlFlag), block_size (int), st_min_ms (int)
    total_length: int | None = None
    sequence_number: int | None = None
    fc_flag: FlowControlFlag | None = None
    block_size: int = 0
    st_min_ms: int = 0


def parse_frame(data: bytes) -> IsoTPFrame:
    """解析 CAN 帧 payload 为 ISO-TP 帧结构。

    不校验帧合法性（如 CF 序号连续性），仅做结构提取。
    """
    if len(data) < 1:
        raise ValueError("ISO-TP frame requires at least 1 byte")

    pci_byte = data[0]
    frame_type = FrameType((pci_byte >> 4) & 0x0F)

    if frame_type == FrameType.SINGLE:
        length = pci_byte & 0x0F
        if length == 0 or length > SF_PAYLOAD_MAX:
            raise ValueError(f"Invalid SF length: {length}")
        payload = data[1:1 + length]
        return IsoTPFrame(FrameType.SINGLE, payload)

    elif frame_type == FrameType.FIRST:
        if len(data) < 2:
            raise ValueError("FF requires at least 2 bytes")
        total_length = ((pci_byte & 0x0F) << 8) | data[1]
        if total_length == 0 or total_length > ISO_TP_MAX_LENGTH:
            raise ValueError(f"Invalid FF total length: {total_length}")
        payload = data[2:2 + FF_PAYLOAD_MAX]
        return IsoTPFrame(FrameType.FIRST, payload, total_length=total_length)

    elif frame_type == FrameType.CONSECUTIVE:
        seq = pci_byte & 0x0F
        payload = data[1:1 + CF_PAYLOAD_MAX]
        return IsoTPFrame(FrameType.CONSECUTIVE, payload, sequence_number=seq)

    else:  # Flow Control
        if len(data) < 3:
            raise ValueError("FC requires at least 3 bytes")
        fc_flag = FlowControlFlag(pci_byte & 0x0F)
        block_size = data[1]
        st_min_ms = data[2]
        return IsoTPFrame(FrameType.FLOW_CONTROL, data,
                          fc_flag=fc_flag, block_size=block_size,
                          st_min_ms=st_min_ms)


# ═══════════════════════════════════════════════════════════════════════════
# 帧构造
# ═══════════════════════════════════════════════════════════════════════════

def build_single_frame(payload: bytes) -> bytes:
    """构造 Single Frame: [0x0N, data...]，N = len(payload)"""
    n = len(payload)
    if n == 0:
        raise ValueError("SF payload must not be empty")
    if n > SF_PAYLOAD_MAX:
        raise ValueError(f"SF payload too long: {n} > {SF_PAYLOAD_MAX}")
    return bytes([0x00 | n]) + payload


def build_first_frame(payload: bytes, total_length: int) -> bytes:
    """构造 First Frame: [0x1Y, ZZ, data...], Y = total_length>>8, ZZ = total_length&0xFF"""
    if total_length <= 0:
        raise ValueError(f"Total length must be positive: {total_length}")
    if total_length > ISO_TP_MAX_LENGTH:
        raise ValueError(f"Total length too large: {total_length}")
    high = (total_length >> 8) & 0x0F
    low = total_length & 0xFF
    return bytes([0x10 | high, low]) + payload[:FF_PAYLOAD_MAX]


def build_consecutive_frame(payload: bytes, sequence: int) -> bytes:
    """构造 Consecutive Frame: [0x2X, data...], X = sequence_number (0-15)"""
    seq = sequence & 0x0F
    return bytes([0x20 | seq]) + payload[:CF_PAYLOAD_MAX]


def build_flow_control(fc_flag: FlowControlFlag,
                       block_size: int = 0,
                       st_min_ms: int = 0) -> bytes:
    """构造 Flow Control Frame: [0x3X, block_size, st_min_ms]"""
    return bytes([0x30 | int(fc_flag), block_size, st_min_ms])


# ═══════════════════════════════════════════════════════════════════════════
# 发送端 — 将完整 payload 拆分为 CAN 帧序列
# ═══════════════════════════════════════════════════════════════════════════

def segment_payload(data: bytes) -> list[bytes]:
    """将任意长度的 payload 拆分为 ISO-TP 帧序列（CAN 帧 payload 列表）。

    不含 Flow Control 交互逻辑——这是纯分段函数。
    交互式传输参见 IsoTPTransmitter / IsoTPReceiver。

    Args:
        data: 完整 payload

    Returns:
        list[bytes]: ISO-TP 帧序列。每帧 ≤ 8 字节，可直接填入 CAN data 字段。
    """
    total = len(data)
    frames: list[bytes] = []

    if total <= SF_PAYLOAD_MAX:
        # Single Frame 即可
        frames.append(build_single_frame(data))
    else:
        # First Frame: 前 6 字节
        frames.append(build_first_frame(data[:FF_PAYLOAD_MAX], total))

        # Consecutive Frames: 每次 7 字节
        remaining = data[FF_PAYLOAD_MAX:]
        seq = 1
        while remaining:
            chunk = remaining[:CF_PAYLOAD_MAX]
            remaining = remaining[CF_PAYLOAD_MAX:]
            frames.append(build_consecutive_frame(chunk, seq))
            seq = (seq + 1) & 0x0F

    return frames


# ═══════════════════════════════════════════════════════════════════════════
# 接收端 — 重组 ISO-TP 帧为完整 payload
# ═══════════════════════════════════════════════════════════════════════════

class IsoTPReceiver:
    """ISO-TP 接收端状态机。

    接收 FF → 分配缓冲区 → 接收 CF → 全部收齐后返回完整 payload。
    同时生成 FC 帧供调用者发送回发送端。
    """

    def __init__(self):
        self._buffer = bytearray()
        self._expected_length = 0
        self._received_length = 0
        self._expected_seq = 0
        self._complete = False

    @property
    def complete(self) -> bool:
        return self._complete

    def feed(self, raw_frame: bytes) -> tuple[bytes | None, bytes | None]:
        """喂入一个 ISO-TP CAN 帧 payload。

        Args:
            raw_frame: CAN 帧 data（含 PCI 字节）

        Returns:
            (assembled_payload, flow_control_frame)
            - assembled_payload: 完成时返回完整数据，否则为 None
            - flow_control_frame: 需发回的 FC 帧（CTS/WAIT/Overflow），不需要时为 None
        """
        frame = parse_frame(raw_frame)

        if frame.frame_type == FrameType.SINGLE:
            self._complete = True
            return frame.payload, None

        elif frame.frame_type == FrameType.FIRST:
            # 开始新的多帧接收
            self._buffer = bytearray(frame.payload)
            self._expected_length = frame.total_length
            self._received_length = len(frame.payload)
            self._expected_seq = 1  # 下一个期望的 CF 序号
            self._complete = False

            # 发送 Flow Control: Continue To Send
            fc = build_flow_control(FlowControlFlag.CTS, block_size=0, st_min_ms=0)
            return None, fc

        elif frame.frame_type == FrameType.CONSECUTIVE:
            if frame.sequence_number != self._expected_seq:
                # 序号不连续 — 发送 Overflow 并丢弃
                fc = build_flow_control(FlowControlFlag.OVERFLOW)
                self._complete = True
                return None, fc

            self._buffer.extend(frame.payload)
            self._received_length += len(frame.payload)
            self._expected_seq = (self._expected_seq + 1) & 0x0F

            if self._received_length >= self._expected_length:
                self._complete = True
                return bytes(self._buffer[:self._expected_length]), None
            else:
                # 每收到 1 帧 CF 就回复一次 CTS
                fc = build_flow_control(FlowControlFlag.CTS, st_min_ms=0)
                return None, fc

        else:
            # Flow Control — 不应该由接收端收到
            return None, None

    def reset(self):
        """重置状态机"""
        self._buffer = bytearray()
        self._expected_length = 0
        self._received_length = 0
        self._expected_seq = 0
        self._complete = False


# ═══════════════════════════════════════════════════════════════════════════
# 双向交互式传输模拟
# ═══════════════════════════════════════════════════════════════════════════

def simulate_transfer(payload: bytes) -> tuple[list[bytes], list[bytes]]:
    """完整的 ISO-TP 交互模拟：发送端分段 → 接收端重组。

    Args:
        payload: 待传输的完整数据

    Returns:
        (sender_frames, receiver_responses)
        - sender_frames:  发送端发出的所有帧（含 SF 或 FF+CFs）
        - receiver_responses: 接收端回复的 FC 帧序列
    """
    sender_frames = segment_payload(payload)
    receiver_responses: list[bytes] = []
    receiver = IsoTPReceiver()

    for frame in sender_frames:
        assembled, fc = receiver.feed(frame)
        if fc is not None:
            receiver_responses.append(fc)

    return sender_frames, receiver_responses


# ═══════════════════════════════════════════════════════════════════════════
# 实用函数 — VIN 读取
# ═══════════════════════════════════════════════════════════════════════════

# 典型 VIN: 17 位 ASCII 字符串 (WMI + VDS + VIS)
# 存储为 DID 0xF190, 17 字节
def encode_vin(vin: str) -> bytes:
    """编码 VIN 为字节串（ASCII），长度必须为 17。"""
    if len(vin) != 17:
        raise ValueError(f"VIN must be 17 characters, got {len(vin)}")
    return vin.encode("ascii")


def decode_vin(data: bytes) -> str:
    """解码字节串为 VIN 字符串。"""
    return data.decode("ascii")
