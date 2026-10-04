"""CAN signal definitions, pure encoding/decoding and frame-size estimation.

This module does not instantiate an ECU, run a scenario or write a log.
"""
from __future__ import annotations

# 1. CAN 帧定义

CAN_MESSAGES = {
    # 发动机 ECU —— 周期 10ms
    "EngineData": {
        "id": 0x0C9,
        "cycle_ms": 10,
        "desc": "发动机数据",
        "signals": [
            {"name": "节气门位置",   "start": 7,  "len": 8,  "scale": 0.4,   "offset": 0,    "unit": "%",  "byte_order": "motorola"},
            {"name": "发动机转速",   "start": 15,  "len": 16, "scale": 0.25,  "offset": 0,    "unit": "rpm", "byte_order": "motorola"},
            {"name": "冷却液温度",   "start": 31, "len": 8,  "scale": 1,     "offset": -40,  "unit": "degC", "byte_order": "motorola"},
            {"name": "车速",         "start": 32, "len": 16, "scale": 0.01,  "offset": 0,    "unit": "km/h", "byte_order": "intel"},
            {"name": "进气歧管压力", "start": 55, "len": 8,  "scale": 1,     "offset": 0,    "unit": "kPa", "byte_order": "motorola"},
        ],
    },

    # 电池管理系统 BMS —— 周期 100ms
    "BatteryStatus": {
        "id": 0x180,
        "cycle_ms": 100,
        "desc": "电池状态",
        "signals": [
            {"name": "SOC",          "start": 7,  "len": 8,  "scale": 0.5,   "offset": 0,   "unit": "%",     "byte_order": "motorola"},
            {"name": "总电压",       "start": 15,  "len": 16, "scale": 0.1,   "offset": 0,   "unit": "V",     "byte_order": "motorola"},
            {"name": "电流",         "start": 31, "len": 16, "scale": 0.1,   "offset": -500, "unit": "A",    "byte_order": "motorola"},
            {"name": "最高单体温度",  "start": 47, "len": 8,  "scale": 1,     "offset": -40, "unit": "degC",  "byte_order": "motorola"},
            {"name": "最低单体温度",  "start": 55, "len": 8,  "scale": 1,     "offset": -40, "unit": "degC",  "byte_order": "motorola"},
        ],
    },

    # ABS/ESP 制动控制器 —— 周期 20ms
    "ABS_WheelSpeed": {
        "id": 0x210,
        "cycle_ms": 20,
        "desc": "轮速与制动",
        "signals": [
            {"name": "左前轮速",    "start": 7,  "len": 16, "scale": 0.01,  "offset": 0,   "unit": "km/h", "byte_order": "motorola"},
            {"name": "右前轮速",    "start": 23, "len": 16, "scale": 0.01,  "offset": 0,   "unit": "km/h", "byte_order": "motorola"},
            {"name": "左后轮速",    "start": 39, "len": 16, "scale": 0.01,  "offset": 0,   "unit": "km/h", "byte_order": "motorola"},
            {"name": "右后轮速",    "start": 55, "len": 16, "scale": 0.01,  "offset": 0,   "unit": "km/h", "byte_order": "motorola"},
        ],
    },

    # 变速箱 TCU —— 周期 50ms
    "Transmission": {
        "id": 0x288,
        "cycle_ms": 50,
        "desc": "变速箱状态",
        "signals": [
            {"name": "当前档位",   "start": 7,  "len": 4,  "scale": 1,   "offset": 0,   "unit": "",     "byte_order": "motorola"},
            {"name": "变速箱油温", "start": 15,  "len": 8,  "scale": 1,   "offset": -40, "unit": "degC",  "byte_order": "motorola"},
            {"name": "输出轴转速", "start": 23, "len": 16, "scale": 1,   "offset": 0,   "unit": "rpm",  "byte_order": "motorola"},
        ],
    },

    # 车身控制器 BCM —— 周期 200ms
    "BodyControl": {
        "id": 0x320,
        "cycle_ms": 200,
        "desc": "车身状态",
        "signals": [
            {"name": "左前门",     "start": 7,  "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
            {"name": "右前门",     "start": 5,  "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
            {"name": "左后门",     "start": 3,  "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
            {"name": "右后门",     "start": 1,  "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
            {"name": "近光灯",     "start": 15,  "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
            {"name": "远光灯",     "start": 13, "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
            {"name": "转向灯",     "start": 11, "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
            {"name": "后备箱",     "start": 9, "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
        ],
    },
}


# 2. CAN 帧编码/解码

def encode_signal(value: float, sig: dict) -> int:
    """将物理值编码为原始整数值"""
    raw = int((value - sig["offset"]) / sig["scale"])
    max_val = (1 << sig["len"]) - 1
    return max(0, min(raw, max_val))


def decode_signal(raw: int, sig: dict) -> float:
    """将原始整数值解码为物理值"""
    return round(raw * sig["scale"] + sig["offset"], 2)


def _signal_bit_positions(start_bit: int, length: int,
                          byte_order: str) -> list[tuple[int, int, int]]:
    """Return (byte_idx, bit_in_byte, signal_bit_shift) for each signal bit.

    DBC bit numbering: within each byte, bit 0 = LSB, bit 7 = MSB.
    - Motorola: start_bit is the position of the signal MSB. Fill order:
      MSB first, then step down within the byte (7 -> 0); when a byte is
      exhausted, continue at bit 7 of the NEXT byte (byte index increases).
    - Intel:    start_bit is the position of the signal LSB. Fill order:
      LSB first, byte index increases.
    """
    positions = []
    # Motorola "network bit number": reverse bit order inside each byte,
    # so bit 7 of a byte maps to the lowest network bit of that byte.
    network_start = 8 * (start_bit // 8) + (7 - start_bit % 8)

    for i in range(length):
        if byte_order == "intel":
            bitnum = start_bit + i
            byte_idx = bitnum // 8
            bit_in_byte = bitnum % 8
            shift = i  # LSB first
        else:  # motorola
            bitnum = network_start + i
            byte_idx = bitnum // 8
            bit_in_byte = 7 - bitnum % 8
            shift = length - 1 - i  # MSB first

        positions.append((byte_idx, bit_in_byte, shift))

    return positions


def build_can_frame(msg_def: dict, signal_values: list[float]) -> list[int]:
    """根据信号值列表构建 8 字节 CAN 数据帧，支持 Motorola/Intel 字节序。"""
    data = [0] * 8
    for i, sig in enumerate(msg_def["signals"]):
        raw = encode_signal(signal_values[i], sig)
        byte_order = sig.get("byte_order", "motorola")
        positions = _signal_bit_positions(sig["start"], sig["len"], byte_order)

        for byte_idx, bit_in_byte, shift in positions:
            if byte_idx < 8 and (raw >> shift) & 1:
                data[byte_idx] |= (1 << bit_in_byte)
    return data


def parse_can_frame(data: list[int], msg_def: dict) -> dict[str, float]:
    """根据信号定义解析 8 字节 CAN 数据帧，支持 Motorola/Intel 字节序。"""
    result = {}
    for sig in msg_def["signals"]:
        raw = 0
        byte_order = sig.get("byte_order", "motorola")
        positions = _signal_bit_positions(sig["start"], sig["len"], byte_order)

        for byte_idx, bit_in_byte, shift in positions:
            if byte_idx < 8 and (data[byte_idx] >> bit_in_byte) & 1:
                raw |= (1 << shift)

        result[sig["name"]] = decode_signal(raw, sig)
    return result


def frame_bits(data_bytes):
    """计算一帧 CAN 2.0A 标准帧的总位数（含位填充估计）。

    CAN 2.0A 帧结构：
      SOF(1) + ID(11) + RTR(1) + IDE(1) + r0(1) + DLC(4)
      + Data(N×8) + CRC(15) + CRC_Delim(1) + ACK(1) + ACK_Delim(1)
      + EOF(7) + IFS(3) = 47 + 8×N

    位填充规则：SOF 到 CRC（不含 CRC_Delim）之间，每连续 5 个相同 bit
    插入 1 个反 bit。这里用 (overhead + data_bits) // 10 做简化估算（≈10%）。
    """
    overhead = 47          # SOF+ID+RTR+IDE+r0+DLC+CRC+CRC_Delim+ACK+ACK_Delim+EOF+IFS
    data_bits = data_bytes * 8
    stuffing = (overhead + data_bits) // 10  # 位填充估算 ~10%
    return overhead + data_bits + stuffing
