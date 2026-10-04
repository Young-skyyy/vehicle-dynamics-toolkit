from vehicle_dynamics_toolkit import CANBus, CANDiagnosticTester, CoreECU, VirtualECUNode
from vehicle_dynamics_toolkit.can_bus import CANFrame


def make_link() -> CANDiagnosticTester:
    bus = CANBus()
    VirtualECUNode(CoreECU(), bus)
    return CANDiagnosticTester(bus)


def test_uds_request_uses_can_bus_and_iso_tp_single_frame() -> None:
    tester = make_link()

    assert tester.request(bytes([0x3E])) == bytes([0x7E])


def test_uds_response_is_reassembled_from_can_frames() -> None:
    bus = CANBus()
    VirtualECUNode(CoreECU(), bus)
    tester = CANDiagnosticTester(bus)

    response = tester.request(bytes([0x22, 0xF1, 0x90]))

    assert response[:3] == bytes([0x62, 0xF1, 0x90])
    assert len(response) == 20
    assert [frame.can_id for frame in bus.history()] == [
        0x7E0,
        0x7E8,
        0x7E0,
        0x7E8,
        0x7E8,
    ]


def test_uds_request_can_use_multi_frame_iso_tp() -> None:
    bus = CANBus()
    VirtualECUNode(CoreECU(), bus)
    tester = CANDiagnosticTester(bus)

    response = tester.request(bytes([0x22, 0xF1, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00]))

    assert response[:3] == bytes([0x62, 0xF1, 0x90])
    assert len(response) == 20
    assert [frame.can_id for frame in bus.history()[:3]] == [0x7E0, 0x7E8, 0x7E0]


def test_uds_negative_response_travels_through_can() -> None:
    tester = make_link()

    assert tester.request(bytes([0x22, 0x12, 0x34])) == bytes([0x7F, 0x22, 0x31])


def test_can_error_frame_is_not_delivered_as_iso_tp_data() -> None:
    bus = CANBus()
    received: list[CANFrame] = []
    errors: list[CANFrame] = []
    bus.subscribe(0x7E8, received.append)
    bus.subscribe_errors(errors.append)

    bus.inject_error("bit_error", timestamp_s=1.0, can_id=0x7E8)

    assert received == []
    assert errors == [CANFrame(0x7E8, b"", 1.0, True, "bit_error")]
    assert bus.history()[-1].is_error_frame


def test_tester_records_can_error_frame() -> None:
    bus = CANBus()
    tester = CANDiagnosticTester(bus)

    bus.inject_error("bus_off", timestamp_s=2.0)

    assert tester.last_error() == CANFrame(0, b"", 2.0, True, "bus_off")
