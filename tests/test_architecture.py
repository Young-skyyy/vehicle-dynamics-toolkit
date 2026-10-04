"""Integration regressions for state, time, transport and fault boundaries."""
import math
import pytest
from vehicle_dynamics_toolkit import CANBus, CANFrame, CoreECU, CANDiagnosticTester, VirtualECUNode
from vehicle_dynamics_toolkit.can_demo import CAN_MESSAGES, generate_frame, simulate_can_bus_advanced
from vehicle_dynamics_toolkit.iso_tp import segment_payload, parse_frame, FrameType, IsoTPReceiver
from vehicle_dynamics_toolkit.uds_can import IsoTPChannel
from vehicle_dynamics_toolkit.uds import DIDDefinition, ECUDiagnosticServer


def test_observation_and_diagnostics_do_not_change_signal_evolution():
    observed, reference = CoreECU(seed=7), CoreECU(seed=7)
    for i in range(100):
        generate_frame("EngineData", CAN_MESSAGES["EngineData"], observed, i / 100)
        observed.handle_request(b"\x10\x03")
        observed.handle_request(b"\x27\x01")
        observed.update(0.01)
        reference.update(0.01)
        assert observed.snapshot() == reference.snapshot()


def test_explicit_brake_is_not_replaced_with_random_noise():
    ecu = CoreECU()
    ecu.speed = 50
    ecu.update(1, throttle=0, brake=75)
    assert ecu.brake_pressure == 75
    assert ecu.speed == 45.5


@pytest.mark.parametrize("dt,throttle,brake", [(math.nan, 0, 0), (-1, 0, 0), (1, math.inf, 0), (1, 0, -1)])
def test_invalid_update_is_atomic(dt, throttle, brake):
    ecu = CoreECU()
    before = ecu.snapshot()
    with pytest.raises(ValueError):
        ecu.update(dt, throttle, brake)
    assert ecu.snapshot() == before
    assert ecu.clock.now == 0


def test_shared_can_clock_expires_session_before_late_tester_present():
    bus = CANBus()
    ecu = CoreECU()
    VirtualECUNode(ecu, bus)
    tester = CANDiagnosticTester(bus)
    tester.request(b"\x10\x03")
    tester.request(b"\x27\x01")
    assert tester.request(b"\x3e", timestamp_s=6) == b"\x7e"
    assert ecu.diagnostic.session.session_type == "default"
    assert not hasattr(ecu.diagnostic, "_pending_seed")
    assert ecu.clock.now == bus.clock.now == 6


def test_late_consecutive_frame_never_reaches_uds():
    bus = CANBus()
    ecu = CoreECU()
    VirtualECUNode(ecu, bus)
    frames = segment_payload(b"\x10\x03" + b"x" * 8)
    bus.send(CANFrame(0x7E0, frames[0], 0))
    bus.send(CANFrame(0x7E0, frames[1], 2))
    assert ecu.diagnostic.session.session_type == "default"
    assert bus.history()[-1].data[0] == 0x32
    # A fresh transaction after a timeout is accepted.
    tester = CANDiagnosticTester(bus)
    assert tester.request(b"\x3e") == b"\x7e"


def test_legacy_transport_preserves_timestamp_and_suppression():
    ecu = CoreECU()
    frames = segment_payload(b"\x10\x03" + b"x" * 8)
    ecu.handle_iso_tp_frame(frames[0], 0)
    assert ecu.handle_iso_tp_frame(frames[1], 2)[0] == 0x32
    ecu.handle_request(b"\x10\x03")
    assert ecu.handle_iso_tp_frame(segment_payload(b"\x27\x81")[0], 2) is None


def test_bidirectional_flow_control_spacing_and_blocks():
    bus = CANBus()
    VirtualECUNode(CoreECU(), bus, request_block_size=1, request_st_min_ms=7)
    tester = CANDiagnosticTester(bus, response_block_size=1, response_st_min_ms=5)
    response = tester.request(b"\x22\xf1\x90" + bytes(40))
    assert len(response) == 20
    for can_id, spacing in ((0x7E0, .007), (0x7E8, .005)):
        data = [f for f in bus.history() if f.can_id == can_id and parse_frame(f.data).frame_type != FrameType.FLOW_CONTROL]
        assert all(b.timestamp_s - a.timestamp_s >= spacing - 1e-12 for a, b in zip(data, data[1:]))
    assert tester.request(b"\x3e") == b"\x7e"


def test_max_length_transfer_does_not_recurse_through_callbacks():
    bus = CANBus()
    received = []
    server = IsoTPChannel(bus, 2, 1, lambda data, ts: server.send_payload(data, ts), block_size=1)
    client = IsoTPChannel(bus, 1, 2, lambda data, ts: received.append(data), block_size=1)
    payload = bytes(i % 256 for i in range(4095))
    client.send_payload(payload, 0)
    assert received == [payload]


def test_suppressed_positive_response_sends_no_zero_length_frame():
    bus = CANBus()
    VirtualECUNode(CoreECU(), bus)
    tester = CANDiagnosticTester(bus)
    tester.request(b"\x10\x03")
    bus.clear_history()
    assert tester.request(b"\x27\x81", expect_response=False) == b""
    assert len(bus.history()) == 1


def test_bus_fault_filter_affects_diagnostic_transport_and_recovery():
    def drop_cf(frame):
        if frame.can_id == 0x7E8 and frame.data[0] >> 4 == 2:
            return CANFrame(frame.can_id, b"", frame.timestamp_s, True, "lost_cf")
        return frame
    bus = CANBus(frame_filter=drop_cf)
    VirtualECUNode(CoreECU(), bus)
    tester = CANDiagnosticTester(bus)
    with pytest.raises(TimeoutError):
        tester.request(b"\x22\xf1\x90")
    assert tester.last_error().error_code == "lost_cf"
    bus.frame_filter = None
    assert len(tester.request(b"\x22\xf1\x90")) == 20


def test_new_single_frame_not_timed_out_by_previous_completed_transfer():
    receiver = IsoTPReceiver()
    sf = segment_payload(b"ok")[0]
    assert receiver.feed(sf, 0)[0] == b"ok"
    assert receiver.feed(sf, 10)[0] == b"ok"


def test_queued_bus_delivers_in_timestamp_order_and_rejects_past():
    bus = CANBus()
    bus.schedule(CANFrame(1, b"later", 2))
    bus.schedule(CANFrame(1, b"first", 1))
    bus.run_until(1)
    assert [f.data for f in bus.history()] == [b"first"]
    bus.run_until()
    assert [f.data for f in bus.history()] == [b"first", b"later"]
    with pytest.raises(ValueError):
        bus.send(CANFrame(1, b"past", 0))


def test_advanced_scenario_uses_bus_and_separate_fault_rng(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    healthy, faulty = CoreECU(seed=7), CoreECU(seed=7)
    normal_bus, fault_bus = CANBus(), CANBus()
    errors = []
    fault_bus.subscribe_errors(errors.append)
    common = dict(duration_s=1, asc_log=None, dbc_path=None)
    result = simulate_can_bus_advanced(**common, ecu=healthy, bus=normal_bus, error_rate=0)
    failed = simulate_can_bus_advanced(**common, ecu=faulty, bus=fault_bus, error_rate=1)
    assert healthy.snapshot() == faulty.snapshot()
    assert result["total_frames"] == len(normal_bus.history())
    assert failed["error_frames"] == len(errors) > 0
    assert not list(tmp_path.iterdir())


def test_dtc_instances_are_isolated():
    a, b = CoreECU(), CoreECU()
    before = b.handle_request(b"\x19\x02\xff")
    a.clear_faults()
    assert b.handle_request(b"\x19\x02\xff") == before


@pytest.mark.parametrize("definition", [dict(scale=0), dict(scale=math.nan), dict(byte_length=0), dict(data_type="float")])
def test_invalid_did_metadata_fails_at_configuration(definition):
    with pytest.raises(ValueError):
        DIDDefinition(0x1234, "invalid", **definition)


def test_nonfinite_did_returns_negative_response():
    server = ECUDiagnosticServer("EMS", {0x000C: math.nan})
    assert server.handle_request(b"\x22\x00\x0c") == b"\x7f\x22\x31"
