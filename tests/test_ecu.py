# -*- coding: utf-8 -*-
"""Tests for the independent virtual ECU core."""

from vehicle_dynamics_toolkit import CoreECU
from vehicle_dynamics_toolkit.uds import UDSSID


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
