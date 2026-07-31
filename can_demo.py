# -*- coding: utf-8 -*-
"""
CAN 总线多 ECU 仿真器：ECU 报文生成、DBC 导出、ASC 日志、DTC 故障码
"""

from __future__ import annotations

import time
import random
import struct

from uds import DTC_DATABASE, ECUDiagnosticServer, run_diagnostic_session, print_diagnostic_session


# 1. CAN 帧定义

CAN_MESSAGES = {
    # 发动机 ECU —— 周期 10ms
    "EngineData": {
        "id": 0x0C9,
        "cycle_ms": 10,
        "desc": "发动机数据",
        "signals": [
            {"name": "节气门位置",   "start": 0,  "len": 8,  "scale": 0.4,   "offset": 0,    "unit": "%",  "byte_order": "motorola"},
            {"name": "发动机转速",   "start": 8,  "len": 16, "scale": 0.25,  "offset": 0,    "unit": "rpm", "byte_order": "motorola"},
            {"name": "冷却液温度",   "start": 24, "len": 8,  "scale": 1,     "offset": -40,  "unit": "degC", "byte_order": "motorola"},
            {"name": "车速",         "start": 32, "len": 16, "scale": 0.01,  "offset": 0,    "unit": "km/h", "byte_order": "intel"},
            {"name": "进气歧管压力", "start": 48, "len": 8,  "scale": 1,     "offset": 0,    "unit": "kPa", "byte_order": "motorola"},
        ],
    },

    # 电池管理系统 BMS —— 周期 100ms
    "BatteryStatus": {
        "id": 0x180,
        "cycle_ms": 100,
        "desc": "电池状态",
        "signals": [
            {"name": "SOC",          "start": 0,  "len": 8,  "scale": 0.5,   "offset": 0,   "unit": "%",     "byte_order": "motorola"},
            {"name": "总电压",       "start": 8,  "len": 16, "scale": 0.1,   "offset": 0,   "unit": "V",     "byte_order": "motorola"},
            {"name": "电流",         "start": 24, "len": 16, "scale": 0.1,   "offset": -500, "unit": "A",    "byte_order": "motorola"},
            {"name": "最高单体温度",  "start": 40, "len": 8,  "scale": 1,     "offset": -40, "unit": "degC",  "byte_order": "motorola"},
            {"name": "最低单体温度",  "start": 48, "len": 8,  "scale": 1,     "offset": -40, "unit": "degC",  "byte_order": "motorola"},
        ],
    },

    # ABS/ESP 制动控制器 —— 周期 20ms
    "ABS_WheelSpeed": {
        "id": 0x210,
        "cycle_ms": 20,
        "desc": "轮速与制动",
        "signals": [
            {"name": "左前轮速",    "start": 0,  "len": 16, "scale": 0.01,  "offset": 0,   "unit": "km/h", "byte_order": "motorola"},
            {"name": "右前轮速",    "start": 16, "len": 16, "scale": 0.01,  "offset": 0,   "unit": "km/h", "byte_order": "motorola"},
            {"name": "左后轮速",    "start": 32, "len": 16, "scale": 0.01,  "offset": 0,   "unit": "km/h", "byte_order": "motorola"},
            {"name": "右后轮速",    "start": 48, "len": 16, "scale": 0.01,  "offset": 0,   "unit": "km/h", "byte_order": "motorola"},
        ],
    },

    # 变速箱 TCU —— 周期 50ms
    "Transmission": {
        "id": 0x288,
        "cycle_ms": 50,
        "desc": "变速箱状态",
        "signals": [
            {"name": "当前档位",   "start": 0,  "len": 4,  "scale": 1,   "offset": 0,   "unit": "",     "byte_order": "motorola"},
            {"name": "变速箱油温", "start": 8,  "len": 8,  "scale": 1,   "offset": -40, "unit": "degC",  "byte_order": "motorola"},
            {"name": "输出轴转速", "start": 16, "len": 16, "scale": 1,   "offset": 0,   "unit": "rpm",  "byte_order": "motorola"},
        ],
    },

    # 车身控制器 BCM —— 周期 200ms
    "BodyControl": {
        "id": 0x320,
        "cycle_ms": 200,
        "desc": "车身状态",
        "signals": [
            {"name": "左前门",     "start": 0,  "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
            {"name": "右前门",     "start": 2,  "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
            {"name": "左后门",     "start": 4,  "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
            {"name": "右后门",     "start": 6,  "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
            {"name": "近光灯",     "start": 8,  "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
            {"name": "远光灯",     "start": 10, "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
            {"name": "转向灯",     "start": 12, "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
            {"name": "后备箱",     "start": 14, "len": 2,  "scale": 1, "offset": 0, "unit": "", "byte_order": "motorola"},
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
    """计算信号每个 bit 的 (byte_idx, bit_in_byte, signal_bit_shift)。

    - Motorola: MSB first, 字节地址递减跨字节
    - Intel:    LSB first, 字节地址递增跨字节

    Returns:
        list of (byte_idx, bit_in_byte, shift) tuples, length 个
    """
    positions = []
    start_byte = start_bit // 8
    start_bit_in_byte = start_bit % 8

    for i in range(length):
        byte_ofs = i // 8
        bit_ofs = i % 8

        if byte_order == "intel":
            byte_idx = start_byte + byte_ofs
            bit_in_byte = start_bit_in_byte + bit_ofs
            if bit_in_byte >= 8:
                byte_idx += 1
                bit_in_byte -= 8
            shift = i  # LSB first
        else:  # motorola
            byte_idx = start_byte - byte_ofs
            bit_in_byte = start_bit_in_byte + bit_ofs
            if bit_in_byte >= 8:
                byte_idx += 1
                bit_in_byte -= 8
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


# 3. ECU 仿真器

class VehicleECU:
    """整车 ECU 状态机，维持车辆运行参数随时间连续变化。

    Args:
        seed: 随机种子，None = 不可复现，传入 int 可固定仿真结果
    """

    def __init__(self, seed: int | None = 42):
        self._rng = random.Random(seed)
        self.rpm = 800                              # 怠速
        self.throttle = 0
        self.speed = 0                              # km/h
        self.coolant_temp = 25                      # 冷启动
        self.gear = 0                               # P 档
        self.soc = 80.0                             # 电池 SOC
        self.brake_pressure = 0
        self.accelerating = False

    def update(self, dt_s):
        """每 dt 秒更新一次车辆状态"""
        # 模拟一个简单的驾驶循环
        if not self.accelerating and self.speed <= 0:
            self.accelerating = True
            self.gear = 1
        if self.speed >= 80:
            self.accelerating = False

        if self.accelerating:
            self.throttle = min(80, self.throttle + self._rng.uniform(0, 10) * dt_s)
            self.rpm += int(500 * dt_s)
            self.speed += 3 * dt_s
        else:
            self.throttle = max(0, self.throttle - self._rng.uniform(5, 15) * dt_s)
            self.rpm -= int(300 * dt_s)
            self.speed = max(0, self.speed - 2 * dt_s)

        self.rpm = max(800, min(6000, self.rpm))
        self.speed = max(0, min(120, self.speed))
        self.coolant_temp = min(95, self.coolant_temp + 0.5 * dt_s)
        self.soc -= 0.001 * dt_s  # 缓慢放电

        # 档位随车速变化
        if self.speed > 60:
            self.gear = 5
        elif self.speed > 40:
            self.gear = 4
        elif self.speed > 25:
            self.gear = 3
        elif self.speed > 10:
            self.gear = 2
        elif self.speed > 0:
            self.gear = 1
        else:
            self.gear = 0

        self.brake_pressure = self._rng.uniform(0, 5) if not self.accelerating else 0


# 4. CAN 帧信号生成器 —— 每个 ECU 类型一个独立函数，通过字典 dispatch

def _gen_engine_data(veh, sim_time):
    """EMS 发动机数据信号值"""
    return [veh.throttle, veh.rpm, veh.coolant_temp, veh.speed, veh._rng.randint(30, 50)]


def _gen_battery_status(veh, sim_time):
    """BMS 电池状态信号值"""
    return [veh.soc, veh._rng.uniform(350, 400), veh._rng.uniform(-10, 50),
            veh._rng.uniform(25, 35), veh._rng.uniform(22, 30)]


def _gen_abs_wheel_speed(veh, sim_time):
    """ABS 四轮轮速信号值"""
    base = veh.speed
    return [base + veh._rng.uniform(-0.5, 0.5),
            base + veh._rng.uniform(-0.5, 0.5),
            base + veh._rng.uniform(-0.3, 0.3),
            base + veh._rng.uniform(-0.3, 0.3)]


def _gen_transmission(veh, sim_time):
    """TCU 变速箱状态信号值"""
    return [veh.gear, veh.coolant_temp + 10, veh.rpm]


def _gen_body_control(veh, sim_time):
    """BCM 车身控制信号值（0=关闭 1=打开 2=故障 3=无效）"""
    return [
        0, 0, 0, 0,                           # 四门关闭
        1 if sim_time > 2 else 0,              # 近光灯（2秒后开启）
        0, 0,                                  # 远光/转向灯关
        0,                                      # 后备箱关
    ]


# ECU 名称 → 信号生成函数 映射表
_FRAME_GENERATORS = {
    "EngineData":      _gen_engine_data,
    "BatteryStatus":   _gen_battery_status,
    "ABS_WheelSpeed":  _gen_abs_wheel_speed,
    "Transmission":    _gen_transmission,
    "BodyControl":     _gen_body_control,
}


def generate_frame(name, msg_def, veh, sim_time):
    """根据 ECU 类型和当前车辆状态，生成 8 字节 CAN 数据帧。

    通过 _FRAME_GENERATORS 字典 dispatch 到对应信号生成函数，
    新增 ECU 只需添加函数 + 字典条目。
    """
    generator = _FRAME_GENERATORS.get(name)
    if generator is None:
        raise ValueError(f"未知的 ECU 消息类型: {name}")
    signal_values = generator(veh, sim_time)
    return build_can_frame(msg_def, signal_values)


# 5. CAN 总线仿真主循环

def simulate_can_bus(duration_s: float = 5) -> dict:
    """模拟 CAN 总线运行 duration_s 秒，返回结构化数据。

    Returns:
        dict: {
            "duration_s": float,
            "total_messages": int,
            "frames": list[dict],
        }
    """
    veh = VehicleECU()
    dt = 0.01  # 10ms 主循环步长
    total_steps = int(duration_s / dt)

    timers = {name: 0 for name in CAN_MESSAGES}
    msg_count = 0
    frames = []

    for step in range(total_steps):
        sim_time = step * dt
        veh.update(dt)

        for name, msg_def in CAN_MESSAGES.items():
            timers[name] += dt * 1000  # 累计毫秒
            if timers[name] >= msg_def["cycle_ms"]:
                timers[name] -= msg_def["cycle_ms"]
                frame_data = generate_frame(name, msg_def, veh, sim_time)
                parsed = parse_can_frame(frame_data, msg_def)
                frames.append({
                    "time_s": round(sim_time, 4),
                    "id": msg_def["id"],
                    "name": name,
                    "signals": parsed,
                    "data": frame_data,
                })
                msg_count += 1

    return {
        "duration_s": duration_s,
        "total_messages": msg_count,
        "frames": frames,
    }


# 6. DTC 故障码仿真
# DTC_DATABASE 从 uds.py 统一导入（单一数据源）


def simulate_dtc_check(seed: int | None = 42) -> dict:
    """模拟诊断仪读取故障码，返回结构化数据。

    Args:
        seed: 随机种子，默认 42 保证可复现

    Returns:
        dict: {
            "active_codes": list[str],
            "details": list[dict],
        }
    """
    rng = random.Random(seed)
    active = rng.sample(list(DTC_DATABASE.keys()), k=rng.randint(0, 2))
    details = [{"code": code, "ecu": DTC_DATABASE[code]["ecu"],
                "desc": DTC_DATABASE[code]["desc"]} for code in active]

    return {
        "active_codes": active,
        "details": details,
    }


# 7. DBC 文件生成器
# DBC 文件格式：Vector CAN 数据库标准，CANoe / CANalyzer 直接读取。

def generate_dbc(filepath: str = "simulated_ecu.dbc", baudrate: int = 500000) -> dict:
    """从 CAN_MESSAGES 生成标准 DBC 文件"""
    # ECU → 报文映射
    _MSG_TO_ECU = {
        "EngineData":     "EMS",
        "BatteryStatus":  "BMS",
        "ABS_WheelSpeed": "ABS",
        "Transmission":   "TCU",
        "BodyControl":    "BCM",
    }
    nodes = sorted(set(_MSG_TO_ECU.values()))

    lines = []
    lines.append('VERSION ""\n')
    lines.append("\nNS_ : \n\tNS_DESC_\n\tCM_\n\tBA_DEF_\n\tBA_\n\tVAL_\n")
    lines.append(f"\nBS_: {baudrate}\n")

    # BU_: 节点列表
    lines.append(f"\nBU_: {' '.join(nodes)}\n")

    for msg_name, msg_def in CAN_MESSAGES.items():
        can_id = msg_def["id"]
        dlc = 8
        transmitter = _MSG_TO_ECU.get(msg_name, "ECU")
        lines.append(f"\nBO_ {can_id} {msg_name}: {dlc} {transmitter}")

        for sig in msg_def["signals"]:
            sig_name = sig["name"].replace(" ", "_")
            start = sig["start"]
            length = sig["len"]
            # DBC: @0 = Motorola (大端), @1 = Intel (小端)
            byte_order = "0" if sig.get("byte_order", "motorola") == "motorola" else "1"
            signed = "-" if sig["offset"] < 0 else "+"
            scale = sig["scale"]
            offset = sig["offset"]
            min_val = 0
            max_val = round((1 << length) - 1)
            unit = sig["unit"] if sig["unit"] else ""
            receivers = " ".join(n for n in nodes if n != transmitter) if len(nodes) > 1 else "Vector__XXX"

            lines.append(
                f' SG_ {sig_name} : {start}|{length}@{byte_order}{signed} '
                f'({scale},{offset}) [{min_val}|{max_val}] "{unit}"  {receivers}'
            )

    # 周期属性
    lines.append('\nBA_DEF_ SG_ "GenMsgCycleTime" INT 0 65535;')
    lines.append('BA_DEF_DEF_ "GenMsgCycleTime" 0;')
    for msg_name, msg_def in CAN_MESSAGES.items():
        lines.append(f'BA_ "GenMsgCycleTime" BO_ {msg_def["id"]} {msg_def["cycle_ms"]};')

    with open(filepath, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    return {
        "filepath": filepath,
        "message_count": len(CAN_MESSAGES),
        "baudrate": baudrate,
    }


# 8. CAN 总线高级仿真（负载率 + ASC + 错误注入）

def simulate_can_bus_advanced(duration_s: float = 10, baudrate: int = 500000,
                               error_rate: float = 0.001,
                               asc_log: str | None = "can_log.asc") -> dict:
    """增强版 CAN 仿真: 总线负载统计 + ASC 日志 + 错误帧注入

    Returns:
        dict: {"total_frames", "error_frames", "avg_load_pct", "bus_load_samples", "dbc_info"}
    """
    veh = VehicleECU()
    dt = 0.01
    total_steps = int(duration_s / dt)
    timers = {name: 0 for name in CAN_MESSAGES}

    total_bits = 0
    total_frames = 0
    error_frames = 0
    bus_load_samples = []

    asc_lines = []
    if asc_log:
        from datetime import datetime
        asc_lines.append("date " + datetime.now().strftime("%a %b %d %H:%M:%S %Y"))
        asc_lines.append("base hex  timestamps absolute")
        asc_lines.append("internal events logged")
        asc_lines.append("// 模拟 ECU: EMS, BMS, ABS, TCU, BCM")
        asc_lines.append("Begin Triggerblock")

    last_report = -2.0
    frames_this_window = 0

    def calc_frame_bits(dlc=8):
        """标准 CAN 2.0A 帧总位数: SOF+ID+RTR+IDE+r0+DLC+Data+CRC+ACK+EOF+IFS"""
        return 1 + 11 + 1 + 1 + 1 + 4 + dlc * 8 + 15 + 1 + 7 + 3

    for step in range(total_steps):
        sim_time = step * dt
        veh.update(dt)

        for name, msg_def in CAN_MESSAGES.items():
            timers[name] += dt * 1000
            if timers[name] >= msg_def["cycle_ms"]:
                timers[name] -= msg_def["cycle_ms"]

                # 错误帧注入（使用 ECU 的 RNG，保证可复现）
                is_error = veh._rng.random() < error_rate
                if is_error:
                    error_frames += 1
                    frame_bits = 6  # 主动错误标志 = 6 dominant bits
                    if asc_log:
                        asc_lines.append(f"{sim_time:11.6f} 1  ErrorFrame      E")
                else:
                    frame_data = generate_frame(name, msg_def, veh, sim_time)
                    frame_bits = calc_frame_bits(len(frame_data))
                    if asc_log:
                        data_hex = " ".join(f"{b:02X}" for b in frame_data)
                        asc_lines.append(
                            f"{sim_time:11.6f} 1  {msg_def['id']:>7d}             Rx   d 8 {data_hex}"
                        )

                total_bits += frame_bits
                total_frames += 1
                frames_this_window += 1

        # 每秒报告负载率
        if sim_time - last_report >= 1.0:
            load_pct = (total_bits / baudrate) / (sim_time + 0.001) * 100
            bus_load_samples.append((sim_time, load_pct))
            frames_this_window = 0
            last_report = sim_time

    avg_load = (total_bits / baudrate) / duration_s * 100

    # 保存 ASC 日志
    if asc_log:
        asc_lines.append("End Triggerblock")
        with open(asc_log, "w", encoding="utf-8") as f:
            f.write("\n".join(asc_lines))

    # 生成 DBC
    dbc_info = generate_dbc()

    return {
        "total_frames": total_frames,
        "error_frames": error_frames,
        "avg_load_pct": avg_load,
        "bus_load_samples": bus_load_samples,
        "dbc_info": dbc_info,
    }


if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════╗
║     CAN 总线多 ECU 仿真器                      ║
║     发动机 | BMS | ABS | 变速箱 | 车身         ║
╚══════════════════════════════════════════════╝
    """)

    # 场景 1：CAN 总线增强仿真（负载率 + ASC 日志 + 错误帧 + DBC 生成）
    result = simulate_can_bus_advanced(duration_s=10, error_rate=0.002)
    print(f"\n  CAN 增强仿真结果: {result['total_frames']} 帧 | "
          f"错误帧 {result['error_frames']} | 平均负载 {result['avg_load_pct']:.1f}%")
    dbc = result["dbc_info"]
    print(f"  DBC 文件: {dbc['filepath']} ({dbc['message_count']} 条报文, {dbc['baudrate']//1000}kbps)")

    # 场景 2：DTC 故障码扫描
    dtc_result = simulate_dtc_check()
    print(f"\n  DTC 故障码扫描")
    print(f"  {'-'*40}")
    if not dtc_result["active_codes"]:
        print("    无故障码（系统正常）")
    else:
        for d in dtc_result["details"]:
            print(f"    {d['code']} | {d['ecu']} | {d['desc']}")

    # 场景 3：UDS 诊断会话演示
    ems_server = ECUDiagnosticServer("EMS", {
        0x000C: 2500.0,   # 发动机转速
        0x000D: 60.0,     # 车速
        0x0005: 90.0,     # 冷却液温度
        0x0011: 35.0,     # 节气门位置
    })
    steps = run_diagnostic_session(ems_server)
    print_diagnostic_session(steps, ems_server)
