# -*- coding: utf-8 -*-
"""Tests for the independent virtual ECU core."""

from vehicle_dynamics_toolkit import CoreECU
from vehicle_dynamics_toolkit.can_demo import CAN_MESSAGES, generate_frame, parse_can_frame
from vehicle_dynamics_toolkit.uds import UDSSID, NRC


def test_core_ecu_exposes_shared_state_and_diagnostics():
    ecu = CoreECU(seed=1)

    assert ecu.snapshot()["rpm"] == 800
    assert ecu.handle_request(bytes([UDSSID.TESTER_PRESENT])) == b"~"

    ecu.update(0.01)

    assert ecu.snapshot()["speed"] > 0
    response = ecu.handle_request(bytes([UDSSID.READ_DATA_BY_IDENTIFIER, 0x00, 0x0D]))
    assert response[:3] == bytes([0x62, 0x00, 0x0D])
    assert int.from_bytes(response[3:], "big") >= 0


def test_core_ecu_allows_adapters_to_update_custom_did():
    ecu = CoreECU()
    ecu.update_did(0x1234, 12.5)

    response = ecu.handle_request(bytes([UDSSID.READ_DATA_BY_IDENTIFIER, 0x12, 0x34]))

    assert response == bytes([0x62, 0x12, 0x34, 0x00, 0x0C])


def test_vehicle_state_is_consistent_through_can_and_uds():
    ecu = CoreECU()
    ecu.update(1.0, throttle=100, brake=0)

    can_data = generate_frame("EngineData", CAN_MESSAGES["EngineData"], ecu, 1.0)
    can_state = parse_can_frame(can_data, CAN_MESSAGES["EngineData"])
    uds_response = ecu.handle_request(bytes([UDSSID.READ_DATA_BY_IDENTIFIER, 0x00, 0x0D]))
    uds_speed = int.from_bytes(uds_response[3:], "big") * 0.01

    assert can_state["车速"] == uds_speed
    assert can_state["车速"] == ecu.speed


def test_simulation_clock_controls_s3_timeout():
    ecu = CoreECU()
    ecu.handle_request(bytes([UDSSID.DIAGNOSTIC_SESSION_CONTROL, 0x03]))
    ecu.update(5.1, throttle=0, brake=0)

    response = ecu.handle_request(bytes([UDSSID.ECU_RESET, 0x01]))

    assert response[0] == 0x7F


def test_fault_injection_read_and_clear_dtc_through_uds():
    ecu = CoreECU()
    ecu.inject_fault("P0301")

    read_response = ecu.handle_request(bytes([UDSSID.READ_DTC_INFORMATION, 0x02, 0xFF]))
    assert b"\x00\x03\x01" in read_response

    ecu.handle_request(bytes([UDSSID.DIAGNOSTIC_SESSION_CONTROL, 0x03]))
    clear_response = ecu.handle_request(bytes([UDSSID.CLEAR_DIAGNOSTIC_INFORMATION, 0xFF, 0xFF, 0xFF]))
    assert clear_response == bytes([0x54])

    cleared = ecu.handle_request(bytes([UDSSID.READ_DTC_INFORMATION, 0x02, 0xFF]))
    assert cleared == bytes([0x59, 0x02, 0x01, 0xFF])


def test_fault_injection_rejects_wrong_ecu_dtc():
    ecu = CoreECU("EMS")
    try:
        ecu.inject_fault("C0035")
    except ValueError:
        pass
    else:
        raise AssertionError("wrong ECU DTC should be rejected")
