# -*- coding: utf-8 -*-
"""
UDS (ISO 14229) 诊断协议栈：Session / Security Access / DID / DTC / Tester Present

模拟 ECU 诊断服务器 + 诊断仪交互，集成到 CAN 总线仿真中。
"""

from __future__ import annotations
import time
import random
from dataclasses import dataclass
from typing import cast
from enum import IntEnum
from .iso_tp import LegacyIsoTPAdapter, encode_vin, decode_vin
from .timing import SimulationClock
import math


# ---- UDS Service IDs ----

class UDSSID(IntEnum):
    DIAGNOSTIC_SESSION_CONTROL = 0x10
    ECU_RESET                = 0x11
    CLEAR_DIAGNOSTIC_INFORMATION = 0x14
    READ_DATA_BY_IDENTIFIER  = 0x22
    SECURITY_ACCESS          = 0x27
    READ_DTC_INFORMATION     = 0x19
    TESTER_PRESENT           = 0x3E

# 正响应 = SID + 0x40
POSITIVE_RESPONSE_OFFSET = 0x40
# 负响应 = 0x7F
NEGATIVE_RESPONSE_SID = 0x7F


# ---- Negative Response Codes ----

class NRC(IntEnum):
    GENERAL_REJECT            = 0x10
    SERVICE_NOT_SUPPORTED     = 0x11
    SUB_FUNCTION_NOT_SUPPORTED = 0x12
    INCORRECT_MESSAGE_LENGTH  = 0x13
    CONDITIONS_NOT_CORRECT    = 0x22
    REQUEST_OUT_OF_RANGE      = 0x31
    SECURITY_ACCESS_DENIED    = 0x33


# ---- DTC Status Byte (ISO 14229-1, Table D.1) ----

class DTCStatusMask(IntEnum):
    """DTC Status Byte 各位含义"""
    TEST_FAILED                          = 0x01  # bit 0
    TEST_FAILED_THIS_OPERATION_CYCLE     = 0x02  # bit 1
    PENDING_DTC                          = 0x04  # bit 2
    CONFIRMED_DTC                        = 0x08  # bit 3
    TEST_NOT_COMPLETED_SINCE_LAST_CLEAR  = 0x10  # bit 4
    TEST_FAILED_SINCE_LAST_CLEAR         = 0x20  # bit 5
    TEST_NOT_COMPLETED_THIS_OPERATION_CYCLE = 0x40  # bit 6
    WARNING_INDICATOR_REQUESTED          = 0x80  # bit 7


def dtc_status_byte(test_failed: bool = False, confirmed: bool = False,
                    pending: bool = False, mil_on: bool = False) -> int:
    """构造 DTC Status Byte。"""
    status = 0
    if test_failed:
        status |= DTCStatusMask.TEST_FAILED
        status |= DTCStatusMask.TEST_FAILED_THIS_OPERATION_CYCLE
        status |= DTCStatusMask.TEST_FAILED_SINCE_LAST_CLEAR
    if confirmed:
        status |= DTCStatusMask.CONFIRMED_DTC
    if pending:
        status |= DTCStatusMask.PENDING_DTC
    if mil_on:
        status |= DTCStatusMask.WARNING_INDICATOR_REQUESTED
    return status


def decode_dtc_status(status_byte: int) -> dict[str, bool]:
    """解码 DTC Status Byte 各位。"""
    return {
        "testFailed":          bool(status_byte & DTCStatusMask.TEST_FAILED),
        "testFailedThisCycle": bool(status_byte & DTCStatusMask.TEST_FAILED_THIS_OPERATION_CYCLE),
        "pendingDTC":          bool(status_byte & DTCStatusMask.PENDING_DTC),
        "confirmedDTC":        bool(status_byte & DTCStatusMask.CONFIRMED_DTC),
        "notCompletedSinceClear": bool(status_byte & DTCStatusMask.TEST_NOT_COMPLETED_SINCE_LAST_CLEAR),
        "failedSinceClear":    bool(status_byte & DTCStatusMask.TEST_FAILED_SINCE_LAST_CLEAR),
        "notCompletedThisCycle": bool(status_byte & DTCStatusMask.TEST_NOT_COMPLETED_THIS_OPERATION_CYCLE),
        "warningIndicator":    bool(status_byte & DTCStatusMask.WARNING_INDICATOR_REQUESTED),
    }


# ---- DTC Database（升级版，含 Status Byte）----

DTC_DATABASE = {
    "P0301": {"desc": "1缸失火检测",           "ecu": "EMS", "status": dtc_status_byte(test_failed=True, confirmed=True, mil_on=True)},
    "P0420": {"desc": "催化转化器效率低于阈值",  "ecu": "EMS", "status": dtc_status_byte(pending=True)},
    "U0100": {"desc": "与 ECM/PCM 失去通讯",   "ecu": "CAN", "status": dtc_status_byte(test_failed=True, confirmed=True)},
    "C0035": {"desc": "左前轮速传感器电路故障",  "ecu": "ABS", "status": dtc_status_byte(test_failed=True, confirmed=True, mil_on=True)},
    "B1A00": {"desc": "环境光传感器故障",       "ecu": "BCM", "status": dtc_status_byte(pending=True)},
    "P0A7F": {"desc": "电池组劣化",            "ecu": "BMS", "status": dtc_status_byte(test_failed=True, confirmed=True, mil_on=True)},
}


# ---- Standard DIDs（常用数据标识符）----

@dataclass(frozen=True)
class DIDDefinition:
    identifier: int
    name: str
    data_type: str = "uint"
    byte_length: int = 2
    unit: str = ""
    scale: float = 1.0
    offset: float = 0.0
    read_sessions: tuple[str, ...] = ("default", "extended", "programming")
    security_level: int = 0

    def __post_init__(self) -> None:
        if (not 0 <= self.identifier <= 0xFFFF or self.data_type not in ("uint", "int", "ascii")
                or self.byte_length <= 0 or not math.isfinite(self.scale) or self.scale <= 0
                or not math.isfinite(self.offset)):
            raise ValueError("invalid DID definition")


_STANDARD_DIDS: dict[int, DIDDefinition] = {
    0x000C: DIDDefinition(0x000C, "发动机转速", byte_length=2, unit="rpm"),
    0x000D: DIDDefinition(0x000D, "车速", byte_length=2, unit="km/h", scale=0.01),
    0x0005: DIDDefinition(0x0005, "冷却液温度", byte_length=1, unit="degC", offset=-40),
    0x0011: DIDDefinition(0x0011, "节气门位置", byte_length=1, unit="%", scale=0.4),
    0x004C: DIDDefinition(0x004C, "油门踏板位置", byte_length=1, unit="%", scale=0.4),
    0x000F: DIDDefinition(0x000F, "进气温度", byte_length=1, unit="degC", offset=-40),
    0xF190: DIDDefinition(0xF190, "VIN 车辆识别码", data_type="ascii", byte_length=17),
}


# ---- UDS 诊断会话 ----

class DiagnosticSession:
    """单个 ECU 的诊断会话状态机。

    Attributes:
        session_type: "default" | "extended" | "programming"
        security_level: 0 = unlocked, 1 = locked → need SecurityAccess
        last_tester_present: 上次收到 0x3E 的时间戳
        s3_timeout: S3 Server 超时（秒），超时回退到 default session
    """

    def __init__(self, s3_timeout: float = 5.0, clock: SimulationClock | None = None):
        self.session_type = "default"
        self.security_level = 0
        self.last_tester_present = 0.0
        self.s3_timeout = s3_timeout
        self.clock = clock

    def tick(self, dt_s: float) -> None:
        """Explicit simulation stepping; injected clocks are preferred."""
        if self.clock is None:
            self.clock = SimulationClock(time.monotonic())
        self.clock.advance(dt_s)

    def check_timeout(self) -> bool:
        if self.session_type == "default":
            return False
        now = time.monotonic() if self.clock is None else self.clock.now
        return now - self.last_tester_present > self.s3_timeout

    def on_tester_present(self):
        self.last_tester_present = time.monotonic() if self.clock is None else self.clock.now

    def goto_default(self):
        self.session_type = "default"
        self.security_level = 0


# ---- ECU 诊断服务器 ----

class ECUDiagnosticServer:
    """处理单个 ECU（如 EMS、ABS）的 UDS 诊断请求。

    Args:
        ecu_name:   "EMS" / "BMS" / "ABS" / "TCU" / "BCM"
        did_values: {DID: current_value} 动态更新的 ECU 数据
    """

    # 不同 session 下可访问的服务
    _SESSION_SERVICES = {
        "default":      {UDSSID.TESTER_PRESENT, UDSSID.READ_DATA_BY_IDENTIFIER, UDSSID.READ_DTC_INFORMATION, UDSSID.DIAGNOSTIC_SESSION_CONTROL},
        "extended":     {UDSSID.TESTER_PRESENT, UDSSID.READ_DATA_BY_IDENTIFIER, UDSSID.READ_DTC_INFORMATION, UDSSID.CLEAR_DIAGNOSTIC_INFORMATION, UDSSID.DIAGNOSTIC_SESSION_CONTROL, UDSSID.ECU_RESET, UDSSID.SECURITY_ACCESS},
        "programming":  {UDSSID.TESTER_PRESENT, UDSSID.DIAGNOSTIC_SESSION_CONTROL, UDSSID.ECU_RESET, UDSSID.SECURITY_ACCESS},
    }

    def __init__(self, ecu_name: str, did_values: dict[int, float] | None = None,
                 did_definitions: dict[int, DIDDefinition] | None = None,
                 clock: SimulationClock | None = None, rng: random.Random | None = None):
        self.ecu_name = ecu_name
        self.session = DiagnosticSession(clock=clock)
        self._rng = rng if rng is not None else random.Random()
        self.did_values: dict[int, float] = dict(did_values or {})
        self.did_definitions = dict(_STANDARD_DIDS)
        if did_definitions:
            self.did_definitions.update(did_definitions)
        # 非数值型 DID（如 VIN 17 字节字符串）
        self.did_bytes: dict[int, bytes] = {}
        self.dtc_status: dict[str, int] = {
            code: int(cast(int, dtc["status"]))
            for code, dtc in DTC_DATABASE.items()
            if dtc["ecu"] == ecu_name
        }
        self._legacy_transport: LegacyIsoTPAdapter | None = None
        for did in self.did_values:
            self.did_definitions.setdefault(did, DIDDefinition(did, f"DID_{did:04X}"))

    def update_did(self, did: int, value: float):
        """更新 DID 实时值（由 CAN 仿真主循环调用）。"""
        self.did_values[did] = value
        self.did_definitions.setdefault(
            did, DIDDefinition(did, f"DID_{did:04X}")
        )

    def expire_session(self) -> None:
        if self.session.check_timeout():
            self.session.goto_default()
            if hasattr(self, "_pending_seed"):
                del self._pending_seed

    def handle_request(self, request: bytes) -> bytes:
        """处理一条 UDS 请求，返回响应字节串。

        Args:
            request: 原始请求字节（至少 1 字节 SID）

        Returns:
            响应字节串（含 SID）；空 bytes 表示不响应
        """
        self.expire_session()
        if len(request) < 1:
            return b""

        sid = request[0]

        # 0x3E（Tester Present）可在任何 session 接收
        if sid == UDSSID.TESTER_PRESENT:
            return self._handle_tester_present()

        # 服务权限检查
        if sid not in self._SESSION_SERVICES.get(self.session.session_type, set()):
            return self._negative_response(sid, NRC.CONDITIONS_NOT_CORRECT)

        # 分发
        if sid == UDSSID.DIAGNOSTIC_SESSION_CONTROL:
            return self._handle_session_control(request)
        elif sid == UDSSID.READ_DATA_BY_IDENTIFIER:
            return self._handle_read_did(request)
        elif sid == UDSSID.READ_DTC_INFORMATION:
            return self._handle_read_dtc(request)
        elif sid == UDSSID.CLEAR_DIAGNOSTIC_INFORMATION:
            return self._handle_clear_dtc(request)
        elif sid == UDSSID.ECU_RESET:
            return self._handle_ecu_reset(request)
        elif sid == UDSSID.SECURITY_ACCESS:
            return self._handle_security_access(request)

        return self._negative_response(sid, NRC.SERVICE_NOT_SUPPORTED)

    def _handle_tester_present(self) -> bytes:
        self.session.on_tester_present()
        # 正响应：不需要回复 0x7E，只应答 zero-subfunction 时返回 0x7E
        return bytes([UDSSID.TESTER_PRESENT + POSITIVE_RESPONSE_OFFSET])

    def _handle_session_control(self, request: bytes) -> bytes:
        if len(request) < 2:
            return self._negative_response(UDSSID.DIAGNOSTIC_SESSION_CONTROL, NRC.INCORRECT_MESSAGE_LENGTH)
        sub = request[1]
        session_map = {
            0x01: "default",
            0x02: "programming",
            0x03: "extended",
        }
        if sub not in session_map:
            return self._negative_response(UDSSID.DIAGNOSTIC_SESSION_CONTROL, NRC.SUB_FUNCTION_NOT_SUPPORTED)
        self.session.session_type = session_map[sub]
        self.session.on_tester_present()
        return bytes([UDSSID.DIAGNOSTIC_SESSION_CONTROL + POSITIVE_RESPONSE_OFFSET, sub])

    def _handle_read_did(self, request: bytes) -> bytes:
        if len(request) < 3:
            return self._negative_response(UDSSID.READ_DATA_BY_IDENTIFIER, NRC.INCORRECT_MESSAGE_LENGTH)
        did = (request[1] << 8) | request[2]
        definition = self.did_definitions.get(did)
        if definition is None or self.session.session_type not in definition.read_sessions:
            return self._negative_response(UDSSID.READ_DATA_BY_IDENTIFIER, NRC.REQUEST_OUT_OF_RANGE)
        if self.session.security_level < definition.security_level:
            return self._negative_response(UDSSID.READ_DATA_BY_IDENTIFIER, NRC.SECURITY_ACCESS_DENIED)

        if definition.data_type == "ascii":
            value = self.did_bytes.get(did, encode_vin(self._DEFAULT_VIN))
            if len(value) != definition.byte_length:
                return self._negative_response(UDSSID.READ_DATA_BY_IDENTIFIER, NRC.REQUEST_OUT_OF_RANGE)
            return bytes([UDSSID.READ_DATA_BY_IDENTIFIER + POSITIVE_RESPONSE_OFFSET,
                          request[1], request[2]]) + value

        if did not in self.did_values:
            return self._negative_response(UDSSID.READ_DATA_BY_IDENTIFIER, NRC.REQUEST_OUT_OF_RANGE)
        raw_val = self.did_values[did]
        if not math.isfinite(raw_val):
            return self._negative_response(UDSSID.READ_DATA_BY_IDENTIFIER, NRC.REQUEST_OUT_OF_RANGE)
        raw = int(round((raw_val - definition.offset) / definition.scale))
        min_raw = -(1 << (definition.byte_length * 8 - 1)) if definition.data_type == "int" else 0
        max_raw = (1 << (definition.byte_length * 8 - (1 if definition.data_type == "int" else 0))) - 1
        if not min_raw <= raw <= max_raw:
            return self._negative_response(UDSSID.READ_DATA_BY_IDENTIFIER, NRC.REQUEST_OUT_OF_RANGE)
        return bytes([UDSSID.READ_DATA_BY_IDENTIFIER + POSITIVE_RESPONSE_OFFSET,
                      request[1], request[2]]) + raw.to_bytes(
                          definition.byte_length, "big", signed=definition.data_type == "int")

    def _handle_read_dtc(self, request: bytes) -> bytes:
        if len(request) < 2:
            return self._negative_response(UDSSID.READ_DTC_INFORMATION, NRC.INCORRECT_MESSAGE_LENGTH)
        sub = request[1]
        # 0x02: Report DTC by Status Mask
        # 0x0A: Report Supported DTCs
        if sub == 0x02:
            # Status Mask 在 request[2]
            mask = request[2] if len(request) >= 3 else 0xFF
            matched: list[tuple[str, int]] = []
            for code, status in self.dtc_status.items():
                if status & mask:
                    matched.append((code, status))
            # DTC Availability Mask + DTCs
            response = bytes([UDSSID.READ_DTC_INFORMATION + POSITIVE_RESPONSE_OFFSET, 0x02])
            response += (1).to_bytes(1, "big")  # DTC Availability Mask (1 byte)
            response += (0xFF).to_bytes(1, "big")  # All DTC status bits available
            for code, status in matched:
                dtc_bytes = int(code[1:], 16).to_bytes(3, "big")  # P0301 → 0x0301
                response += dtc_bytes + status.to_bytes(1, "big")
            return response
        elif sub == 0x0A:
            # 返回支持的 DTC 数量
            return bytes([UDSSID.READ_DTC_INFORMATION + POSITIVE_RESPONSE_OFFSET, 0x0A,
                          len(self.dtc_status)])
        else:
            return self._negative_response(UDSSID.READ_DTC_INFORMATION, NRC.SUB_FUNCTION_NOT_SUPPORTED)

    def set_dtc(self, code: str, status: int | None = None) -> None:
        """Set an ECU-local DTC status for fault injection or test adapters."""
        if code not in self.dtc_status:
            raise ValueError(f"DTC {code} is not configured for ECU {self.ecu_name}")
        if status is not None and not 0 <= status <= 255:
            raise ValueError("DTC status must fit one byte")
        self.dtc_status[code] = (
            dtc_status_byte(test_failed=True, pending=True, confirmed=True)
            if status is None else status
        )

    def clear_dtcs(self) -> None:
        """Clear all ECU-local DTC status bits."""
        for code in self.dtc_status:
            self.dtc_status[code] = 0

    def _handle_clear_dtc(self, request: bytes) -> bytes:
        if len(request) != 4:
            return self._negative_response(UDSSID.CLEAR_DIAGNOSTIC_INFORMATION, NRC.INCORRECT_MESSAGE_LENGTH)
        group = int.from_bytes(request[1:4], "big")
        if group not in (0xFFFFFF, 0x000000):
            return self._negative_response(UDSSID.CLEAR_DIAGNOSTIC_INFORMATION, NRC.REQUEST_OUT_OF_RANGE)
        self.clear_dtcs()
        return bytes([UDSSID.CLEAR_DIAGNOSTIC_INFORMATION + POSITIVE_RESPONSE_OFFSET])

    def _handle_security_access(self, request: bytes) -> bytes:
        """0x27 Security Access：requestSeed (0x01) / sendKey (0x02)。

        subfunction 最高位 (0x80) 为 Suppress Positive Response (SPR)：
        - 置位时正响应被抑制（返回 b""），负响应仍正常发送
        - requestSeed = 0x01 / 0x81，sendKey = 0x02 / 0x82

        requestSeed: 返回 2 字节随机数
        sendKey:     key = seed ^ 0x5555 (16-bit XOR)，校验通过提升 security_level
        """
        if len(request) < 2:
            return self._negative_response(UDSSID.SECURITY_ACCESS, NRC.INCORRECT_MESSAGE_LENGTH)

        sub = request[1]
        suppress = (sub & 0x80) != 0  # bit7 = Suppress Positive Response
        actual_sub = sub & 0x7F

        if actual_sub == 0x01:  # requestSeed
            self._pending_seed = self._rng.randint(0, 0xFFFF)
            if suppress:
                return b""  # 抑制正响应
            return bytes([UDSSID.SECURITY_ACCESS + POSITIVE_RESPONSE_OFFSET, 0x01]) + \
                self._pending_seed.to_bytes(2, "big")

        elif actual_sub == 0x02:  # sendKey
            if len(request) < 4:
                return self._negative_response(UDSSID.SECURITY_ACCESS, NRC.INCORRECT_MESSAGE_LENGTH)
            if not hasattr(self, "_pending_seed"):
                return self._negative_response(UDSSID.SECURITY_ACCESS, NRC.CONDITIONS_NOT_CORRECT)
            received_key = int.from_bytes(request[2:4], "big")
            expected_key = self._pending_seed ^ 0x5555
            if received_key == expected_key:
                self.session.security_level = 1
                del self._pending_seed
                if suppress:
                    return b""  # 抑制正响应
                return bytes([UDSSID.SECURITY_ACCESS + POSITIVE_RESPONSE_OFFSET, 0x02])
            else:
                return self._negative_response(UDSSID.SECURITY_ACCESS, NRC.REQUEST_OUT_OF_RANGE)

        return self._negative_response(UDSSID.SECURITY_ACCESS, NRC.SUB_FUNCTION_NOT_SUPPORTED)

    def _handle_ecu_reset(self, request: bytes) -> bytes:
        if len(request) < 2:
            return self._negative_response(UDSSID.ECU_RESET, NRC.INCORRECT_MESSAGE_LENGTH)
        sub = request[1]
        if sub == 0x01:  # Hard Reset
            return bytes([UDSSID.ECU_RESET + POSITIVE_RESPONSE_OFFSET, 0x01])
        return self._negative_response(UDSSID.ECU_RESET, NRC.SUB_FUNCTION_NOT_SUPPORTED)

    def _negative_response(self, sid: int, nrc: int) -> bytes:
        return bytes([NEGATIVE_RESPONSE_SID, sid, nrc])

    # ═════════════════════════════════════════════════════════════
    # ISO-TP 多帧传输支持
    # ═════════════════════════════════════════════════════════════

    _DEFAULT_VIN = "WVWZZZ3CZ9E000001"  # 17位占位 VIN

    def handle_iso_tp_frame(self, can_data: bytes, timestamp_s: float = 0.0) -> bytes | None:
        """Legacy polling adapter; CAN integrations use IsoTPChannel instead."""
        if self._legacy_transport is None:
            self._legacy_transport = LegacyIsoTPAdapter(self.handle_request)
        return self._legacy_transport.feed(can_data, timestamp_s)

    def get_next_response_frame(self) -> bytes | None:
        if self._legacy_transport is None:
            return None
        return self._legacy_transport.next_frame()


# ---- 诊断仪（模拟）----

def run_diagnostic_session(server: ECUDiagnosticServer) -> list[dict[str, object]]:
    """执行一组典型 UDS 诊断步骤，返回结构化结果。

    Args:
        server: ECUDiagnosticServer 实例

    Returns:
        list of dict，每个步骤含:
            {"label": str, "request_desc": str, "request": bytes,
             "response": bytes, "parsed": dict | None}
    """
    steps = []

    # 1. Tester Present（default session，无需切换）
    req = bytes([UDSSID.TESTER_PRESENT])
    resp = server.handle_request(req)
    steps.append({
        "label": "TesterPresent",
        "request_desc": "[0x3E] Tester Present",
        "request": req,
        "response": resp,
        "parsed": None,
    })

    # 2. 切换到 Extended Session
    req = bytes([UDSSID.DIAGNOSTIC_SESSION_CONTROL, 0x03])
    resp = server.handle_request(req)
    steps.append({
        "label": "ExtSession",
        "request_desc": "[0x10 03] Extended Session",
        "request": req,
        "response": resp,
        "parsed": None,
    })

    # 3. 读取 DID 0x000C（发动机转速）
    req = bytes([UDSSID.READ_DATA_BY_IDENTIFIER, 0x00, 0x0C])
    resp = server.handle_request(req)
    parsed = None
    if resp[0] != NEGATIVE_RESPONSE_SID:
        val = int.from_bytes(resp[3:], "big")
        parsed = {"did": 0x000C, "raw_value": val}
    steps.append({
        "label": "ReadRPM",
        "request_desc": "[0x22 000C] Read RPM",
        "request": req,
        "response": resp,
        "parsed": parsed,  # type: ignore[dict-item]
    })

    # 4. 读取 DID 0x000D（车速）
    req = bytes([UDSSID.READ_DATA_BY_IDENTIFIER, 0x00, 0x0D])
    resp = server.handle_request(req)
    parsed = None
    if resp[0] != NEGATIVE_RESPONSE_SID:
        val = int.from_bytes(resp[3:], "big")
        parsed = {"did": 0x000D, "raw_value": val, "value_kmh": round(val * 0.01)}
    steps.append({
        "label": "ReadSpeed",
        "request_desc": "[0x22 000D] Read Speed",
        "request": req,
        "response": resp,
        "parsed": parsed,  # type: ignore[dict-item]
    })

    # 5. 读取 DTC（Status Mask = 0xFF = all）
    req = bytes([UDSSID.READ_DTC_INFORMATION, 0x02, 0xFF])
    resp = server.handle_request(req)
    parsed = None
    if resp[0] != NEGATIVE_RESPONSE_SID:
        dtcs = []
        offset = 4  # skip SID+0x40, sub-function, availability mask byte, mask
        while offset + 3 < len(resp):
            dtc_num = int.from_bytes(resp[offset:offset + 3], "big")
            status = resp[offset + 3]
            code_prefix = {0: "P0", 1: "P1", 2: "C0", 3: "C1", 4: "B0", 5: "B1", 6: "U0", 7: "U1"}
            code = f"{code_prefix.get(dtc_num >> 12, 'P0')}{dtc_num & 0xFFF:03X}"
            desc = DTC_DATABASE.get(code, {}).get("desc", "Unknown")
            status_bits = decode_dtc_status(status)
            dtcs.append({
                "code": code, "desc": desc,
                "status_byte": status, "status_decoded": status_bits,
            })
            offset += 4
        parsed = {"dtc_count": len(dtcs), "dtcs": dtcs}  # type: ignore[dict-item]
    steps.append({
        "label": "ReadDTC",
        "request_desc": "[0x19 02] Read DTC",
        "request": req,
        "response": resp,
        "parsed": parsed,  # type: ignore[dict-item]
    })

    # 6. 回到 Default Session
    req = bytes([UDSSID.DIAGNOSTIC_SESSION_CONTROL, 0x01])
    resp = server.handle_request(req)
    steps.append({
        "label": "DefaultSession",
        "request_desc": "[0x10 01] Default Session",
        "request": req,
        "response": resp,
        "parsed": None,
    })

    return steps  # type: ignore[return-value]


def print_diagnostic_session(steps: list[dict], server: ECUDiagnosticServer):
    """格式化打印诊断会话步骤结果。

    Args:
        steps: run_diagnostic_session() 返回的步骤列表
        server: 对应的 ECUDiagnosticServer 实例
    """
    print(f"\n{'='*60}")
    print(f"  UDS 诊断会话 — {server.ecu_name}")
    print(f"{'='*60}")

    for step in steps:
        req_desc = step["request_desc"]
        resp = step["response"]

        if resp[0] == NEGATIVE_RESPONSE_SID:
            print(f"  {req_desc:<28} → NR: {resp.hex()}")
            continue

        label = step["label"]
        if label == "TesterPresent":
            print(f"  {req_desc:<28} → {resp.hex()}")

        elif label == "ExtSession":
            print(f"  {req_desc:<28} → {resp.hex()}")

        elif label == "ReadRPM" and step["parsed"]:
            p = step["parsed"]
            print(f"  {req_desc:<28} → DID={p['did']:04X}, raw={p['raw_value']}")

        elif label == "ReadSpeed" and step["parsed"]:
            p = step["parsed"]
            print(f"  {req_desc:<28} → raw={p['raw_value']} (={p['value_kmh']:.0f} km/h)")

        elif label == "ReadDTC" and step["parsed"]:
            p = step["parsed"]
            print(f"  {req_desc:<28} → {p['dtc_count']} DTCs")
            for dtc in p["dtcs"]:
                s = dtc["status_decoded"]
                status_str = "confirmed" if s["confirmedDTC"] else "pending"
                print(f"    {dtc['code']} | {dtc['desc']} | {status_str} (0x{dtc['status_byte']:02X})")

        elif label == "DefaultSession":
            print(f"  {req_desc:<28} → {resp.hex()}")

        else:
            print(f"  {req_desc:<28} → {resp.hex()}")


def diagnostic_session_demo(server: ECUDiagnosticServer):
    """运行并打印诊断会话（向后兼容包装）。"""
    steps = run_diagnostic_session(server)
    print_diagnostic_session(steps, server)
