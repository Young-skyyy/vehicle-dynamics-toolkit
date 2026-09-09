from vehicle_dynamics_toolkit import CANBus, CANFrame, CoreECU, DiagnosticTester


def test_can_bus_routes_frames_by_identifier() -> None:
    bus = CANBus()
    received: list[CANFrame] = []
    bus.subscribe(0x100, received.append)

    bus.send(CANFrame(0x100, b"\x01"))
    bus.send(CANFrame(0x200, b"\x02"))

    assert received == [CANFrame(0x100, b"\x01")]
    assert len(bus.history()) == 2


def test_diagnostic_tester_keeps_protocol_boundary() -> None:
    ecu = CoreECU()
    tester = DiagnosticTester(ecu)

    response = tester.request(bytes([0x3E]))

    assert response == bytes([0x7E])
    assert tester.last_exchange() == (bytes([0x3E]), response)
