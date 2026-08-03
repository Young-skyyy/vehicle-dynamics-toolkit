# -*- coding: utf-8 -*-
"""Pytest unit tests for UDS diagnostic protocol module"""

import pytest
from uds import (
    UDSSID,
    NRC,
    DTCStatusMask,
    dtc_status_byte,
    decode_dtc_status,
    DTC_DATABASE,
    DiagnosticSession,
    ECUDiagnosticServer,
    POSITIVE_RESPONSE_OFFSET,
    NEGATIVE_RESPONSE_SID,
)


# ---- DTC Status Byte ----

class TestDTCStatusByte:
    def test_confirmed_fault(self):
        status = dtc_status_byte(test_failed=True, confirmed=True, mil_on=True)
        decoded = decode_dtc_status(status)
        assert decoded["testFailed"]
        assert decoded["confirmedDTC"]
        assert decoded["warningIndicator"]

    def test_pending_only(self):
        status = dtc_status_byte(pending=True)
        decoded = decode_dtc_status(status)
        assert decoded["pendingDTC"]
        assert not decoded["confirmedDTC"]
        assert not decoded["testFailed"]

    def test_no_fault(self):
        status = dtc_status_byte()
        assert status == 0
        decoded = decode_dtc_status(status)
        assert not any(decoded.values())

    def test_all_dtcs_have_status_byte(self):
        for code, dtc in DTC_DATABASE.items():
            assert "status" in dtc
            assert 0 <= dtc["status"] <= 255


# ---- DiagnosticSession ----

class TestDiagnosticSession:
    def test_starts_in_default(self):
        s = DiagnosticSession()
        assert s.session_type == "default"

    def test_tester_present_updates_timer(self):
        s = DiagnosticSession(s3_timeout=0.01)
        s.session_type = "extended"
        s.on_tester_present()
        assert not s.check_timeout()

    def test_timeout_reverts_to_default(self):
        import time
        s = DiagnosticSession(s3_timeout=0.01)
        s.session_type = "extended"
        time.sleep(0.02)
        assert s.check_timeout()
        s.goto_default()
        assert s.session_type == "default"

    def test_default_session_never_times_out(self):
        s = DiagnosticSession(s3_timeout=0.01)
        import time
        time.sleep(0.02)
        assert not s.check_timeout()  # default 会话不超时


# ---- ECUDiagnosticServer ----

@pytest.fixture
def ems_server():
    return ECUDiagnosticServer("EMS", {
        0x000C: 2500.0,   # RPM
        0x000D: 60.0,     # km/h
        0x0005: 90.0,     # degC
    })


class TestECUDiagnosticServerBasic:
    def test_tester_present(self, ems_server):
        resp = ems_server.handle_request(bytes([UDSSID.TESTER_PRESENT]))
        assert resp == bytes([UDSSID.TESTER_PRESENT + POSITIVE_RESPONSE_OFFSET])

    def test_session_control_extended(self, ems_server):
        resp = ems_server.handle_request(bytes([UDSSID.DIAGNOSTIC_SESSION_CONTROL, 0x03]))
        assert resp[0] == UDSSID.DIAGNOSTIC_SESSION_CONTROL + POSITIVE_RESPONSE_OFFSET
        assert resp[1] == 0x03

    def test_session_control_default(self, ems_server):
        resp = ems_server.handle_request(bytes([UDSSID.DIAGNOSTIC_SESSION_CONTROL, 0x01]))
        assert resp[0] == UDSSID.DIAGNOSTIC_SESSION_CONTROL + POSITIVE_RESPONSE_OFFSET

    def test_unknown_sid_returns_negative(self, ems_server):
        resp = ems_server.handle_request(bytes([0xAA, 0x00]))
        assert resp[0] == NEGATIVE_RESPONSE_SID
        assert resp[1] == 0xAA

    def test_empty_request(self, ems_server):
        assert ems_server.handle_request(b"") == b""


class TestECUDiagnosticServerReadDID:
    def test_read_rpm(self, ems_server):
        resp = ems_server.handle_request(bytes([UDSSID.READ_DATA_BY_IDENTIFIER, 0x00, 0x0C]))
        assert resp[0] == UDSSID.READ_DATA_BY_IDENTIFIER + POSITIVE_RESPONSE_OFFSET
        val = int.from_bytes(resp[3:], "big")
        assert val == 2500  # RPM

    def test_read_speed(self, ems_server):
        resp = ems_server.handle_request(bytes([UDSSID.READ_DATA_BY_IDENTIFIER, 0x00, 0x0D]))
        assert resp[0] == UDSSID.READ_DATA_BY_IDENTIFIER + POSITIVE_RESPONSE_OFFSET
        val = int.from_bytes(resp[3:], "big")
        assert val == 6000  # raw value (60 / 0.01)

    def test_unknown_did_returns_nr(self, ems_server):
        resp = ems_server.handle_request(bytes([UDSSID.READ_DATA_BY_IDENTIFIER, 0xFF, 0xFF]))
        assert resp[0] == NEGATIVE_RESPONSE_SID
        assert resp[2] == NRC.REQUEST_OUT_OF_RANGE

    def test_update_did(self, ems_server):
        ems_server.update_did(0x000C, 3000)
        resp = ems_server.handle_request(bytes([UDSSID.READ_DATA_BY_IDENTIFIER, 0x00, 0x0C]))
        val = int.from_bytes(resp[3:], "big")
        assert val == 3000


class TestECUDiagnosticServerReadDTC:
    def test_read_dtc_by_status_mask(self, ems_server):
        resp = ems_server.handle_request(bytes([UDSSID.READ_DTC_INFORMATION, 0x02, 0xFF]))
        assert resp[0] == UDSSID.READ_DTC_INFORMATION + POSITIVE_RESPONSE_OFFSET
        assert resp[1] == 0x02

    def test_read_supported_dtcs(self, ems_server):
        resp = ems_server.handle_request(bytes([UDSSID.READ_DTC_INFORMATION, 0x0A]))
        assert resp[0] == UDSSID.READ_DTC_INFORMATION + POSITIVE_RESPONSE_OFFSET
        assert resp[1] == 0x0A
        # EMS has 3 DTCs: P0301, P0420, U0100
        count = sum(1 for d in DTC_DATABASE.values() if d["ecu"] == "EMS")
        assert resp[2] == count


class TestECUDiagnosticServerECUReset:
    def test_hard_reset_in_extended(self, ems_server):
        # 先切换到 extended session
        ems_server.handle_request(bytes([UDSSID.DIAGNOSTIC_SESSION_CONTROL, 0x03]))
        resp = ems_server.handle_request(bytes([UDSSID.ECU_RESET, 0x01]))
        assert resp == bytes([UDSSID.ECU_RESET + POSITIVE_RESPONSE_OFFSET, 0x01])

    def test_reset_denied_in_default(self, ems_server):
        """ECU Reset 在 default session 下应被拒绝"""
        resp = ems_server.handle_request(bytes([UDSSID.ECU_RESET, 0x01]))
        assert resp[0] == NEGATIVE_RESPONSE_SID
        assert resp[1] == UDSSID.ECU_RESET
        assert resp[2] == NRC.CONDITIONS_NOT_CORRECT


class TestECUDiagnosticNegativeResponses:
    def test_too_short_session_control(self, ems_server):
        resp = ems_server.handle_request(bytes([UDSSID.DIAGNOSTIC_SESSION_CONTROL]))
        assert resp[0] == NEGATIVE_RESPONSE_SID
        assert resp[2] == NRC.INCORRECT_MESSAGE_LENGTH

    def test_too_short_read_did(self, ems_server):
        resp = ems_server.handle_request(bytes([UDSSID.READ_DATA_BY_IDENTIFIER, 0x00]))
        assert resp[0] == NEGATIVE_RESPONSE_SID
        assert resp[2] == NRC.INCORRECT_MESSAGE_LENGTH

    def test_unsupported_sub_function(self, ems_server):
        resp = ems_server.handle_request(bytes([UDSSID.DIAGNOSTIC_SESSION_CONTROL, 0xFF]))
        assert resp[0] == NEGATIVE_RESPONSE_SID
        assert resp[2] == NRC.SUB_FUNCTION_NOT_SUPPORTED


# ---- Security Access (0x27) ----

class TestSecurityAccess:
    """0x27 Security Access — requestSeed + sendKey"""

    @pytest.fixture
    def server(self):
        srv = ECUDiagnosticServer("EMS")
        # 切换到 extended session（0x27 需要在该 session 下）
        srv.handle_request(bytes([UDSSID.DIAGNOSTIC_SESSION_CONTROL, 0x03]))
        return srv

    def test_request_seed_returns_2_bytes(self, server):
        resp = server.handle_request(bytes([UDSSID.SECURITY_ACCESS, 0x01]))
        assert resp[0] == UDSSID.SECURITY_ACCESS + POSITIVE_RESPONSE_OFFSET
        assert resp[1] == 0x01
        assert len(resp) == 4  # SID+0x40 + sub + seed(2 bytes)

    def test_full_unlock_sequence(self, server):
        # 1. requestSeed
        resp = server.handle_request(bytes([UDSSID.SECURITY_ACCESS, 0x01]))
        seed = int.from_bytes(resp[2:4], "big")

        # 2. sendKey = seed ^ 0x5555
        key = seed ^ 0x5555
        resp = server.handle_request(bytes([UDSSID.SECURITY_ACCESS, 0x02]) + key.to_bytes(2, "big"))
        assert resp[0] == UDSSID.SECURITY_ACCESS + POSITIVE_RESPONSE_OFFSET
        assert resp[1] == 0x02
        assert server.session.security_level == 1, "解锁后 security_level 应为 1"

    def test_wrong_key_returns_negative(self, server):
        # requestSeed
        resp = server.handle_request(bytes([UDSSID.SECURITY_ACCESS, 0x01]))
        seed = int.from_bytes(resp[2:4], "big")

        # 发送错误 key
        wrong_key = (seed ^ 0x5555) + 1
        resp = server.handle_request(bytes([UDSSID.SECURITY_ACCESS, 0x02]) + wrong_key.to_bytes(2, "big"))
        assert resp[0] == NEGATIVE_RESPONSE_SID
        assert resp[1] == UDSSID.SECURITY_ACCESS
        assert resp[2] == NRC.REQUEST_OUT_OF_RANGE

    def test_send_key_without_seed_returns_nr(self, server):
        """未先 requestSeed 直接 sendKey 应返回 NRC 0x22"""
        resp = server.handle_request(bytes([UDSSID.SECURITY_ACCESS, 0x02, 0x00, 0x00]))
        assert resp[0] == NEGATIVE_RESPONSE_SID
        assert resp[2] == NRC.CONDITIONS_NOT_CORRECT

    def test_request_seed_suppress_positive_response(self, server):
        """0x81 requestSeed + SPR：正响应被抑制（b""），但 seed 仍被记录"""
        resp = server.handle_request(bytes([UDSSID.SECURITY_ACCESS, 0x81]))
        assert resp == b""
        assert hasattr(server, "_pending_seed"), "SPR 下 seed 仍应生成"

    def test_send_key_suppress_positive_response(self, server):
        """0x82 sendKey + SPR：解锁成功但正响应被抑制"""
        resp = server.handle_request(bytes([UDSSID.SECURITY_ACCESS, 0x01]))
        seed = int.from_bytes(resp[2:4], "big")
        key = seed ^ 0x5555
        resp = server.handle_request(bytes([UDSSID.SECURITY_ACCESS, 0x82]) + key.to_bytes(2, "big"))
        assert resp == b""
        assert server.session.security_level == 1, "SPR 下解锁逻辑仍应生效"

    def test_spr_error_still_returns_negative(self, server):
        """SPR 只抑制正响应，负响应仍正常返回"""
        resp = server.handle_request(bytes([UDSSID.SECURITY_ACCESS, 0x82, 0x00, 0x00]))
        assert resp[0] == NEGATIVE_RESPONSE_SID
        assert resp[2] == NRC.CONDITIONS_NOT_CORRECT
