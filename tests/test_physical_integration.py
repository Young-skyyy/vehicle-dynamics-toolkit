"""Independent physics and physical CAN/UDS integration regressions."""
import math
import pytest
from vehicle_dynamics_toolkit import (
    DynamicsModel, DynamicsECU, DriverInput, CANBus, CANDiagnosticTester, VirtualECUNode,
)
from vehicle_dynamics_toolkit.can_demo import CAN_MESSAGES, generate_frame, parse_can_frame
from vehicle_dynamics_toolkit.physical_validation import physics_report
from vehicle_dynamics_toolkit.validation import validate_acceleration, validate_braking, validate_lateral
from vehicle_dynamics_toolkit.simulation import reference_vehicle


def test_analytical_physical_baselines():
    report = physics_report()
    assert report["checks"] == 34
    assert report["passed"], [r for r in report["results"] if not r["passed"]]
    assert report["real_vehicle_accuracy"] == "NOT_VALIDATED"


def test_baseline_detects_wrong_resistance_sign(monkeypatch):
    import vehicle_dynamics_toolkit.simulation as simulation
    acceleration = simulation.calc_acceleration
    monkeypatch.setattr(simulation, "calc_acceleration", lambda *a, **kw: abs(acceleration(*a, **kw)))
    report = physics_report()
    assert not report["passed"]
    assert any(not r["passed"] and r["case"] == "rolling_stop" for r in report["results"])


@pytest.mark.parametrize("command", [(-.1, 0, 0), (50, 0, 0), (0, 1.1, 0),
                                     (0, 0, .71), (math.nan, 0, 0), (0, math.inf, 0)])
def test_physical_command_rejects_invalid_units_and_ranges(command):
    with pytest.raises(ValueError):
        DriverInput(*command)


def test_physical_can_and_uds_share_plant_outputs_and_leave_state_unchanged():
    ecu = DynamicsECU()
    model = DynamicsModel()
    bus = CANBus(clock=ecu.clock)
    VirtualECUNode(ecu, bus)
    tester = CANDiagnosticTester(bus)
    for _ in range(200):
        state = ecu.step(.01, DriverInput(.5, 0, .02))
        expected = model.step(.01, .5, 0, .02)
        assert state["vx"] == pytest.approx(expected["vx"], abs=1e-10)
        data = generate_frame("EngineData", CAN_MESSAGES["EngineData"], ecu, ecu.clock.now)
        decoded = parse_can_frame(data, CAN_MESSAGES["EngineData"])
        assert abs(decoded["车速"] - expected["vx"] * 3.6) <= .01
        response = tester.request(b"\x22\x00\x0d")
        assert response[:3] == b"\x62\x00\x0d"
        assert abs(int.from_bytes(response[3:], "big") * .01 - expected["vx"] * 3.6) <= .01
    before = ecu.snapshot()
    for _ in range(10):
        generate_frame("EngineData", CAN_MESSAGES["EngineData"], ecu, ecu.clock.now)
        tester.request(b"\x22\x00\x0c")
    assert ecu.snapshot() == before


def test_protocol_time_gap_holds_last_physical_control_then_applies_next():
    ecu = DynamicsECU()
    reference = DynamicsModel()
    ecu.step(.1, DriverInput(.5))
    reference.step(.1, .5, 0, 0)
    bus = CANBus(clock=ecu.clock)
    VirtualECUNode(ecu, bus)
    tester = CANDiagnosticTester(bus, response_st_min_ms=10)
    # VIN has two consecutive frames; the CAN clock advances by 20 ms.
    tester.request(b"\x22\xf1\x90")
    assert ecu.clock.now == pytest.approx(.12)
    assert ecu.model.state.t == pytest.approx(.1)  # catch-up occurs on next observation/step
    reference.step(.02, .5, 0, 0)
    ecu.step(.01, DriverInput(brake=.5))
    expected = reference.step(.01, 0, .5, 0)
    for field in expected:
        assert ecu.snapshot()[field] == pytest.approx(expected[field], abs=1e-9)
    assert ecu.model.state.t == ecu.clock.now


def test_can_generation_catches_up_to_protocol_clock():
    ecu = DynamicsECU(DynamicsModel(initial_vx=20))
    ecu.clock.advance(.05)
    data = generate_frame("EngineData", CAN_MESSAGES["EngineData"], ecu, ecu.clock.now)
    assert ecu.model.state.t == .05
    assert parse_can_frame(data, CAN_MESSAGES["EngineData"])["车速"] < 72


def test_invalid_ecu_step_is_atomic_and_percent_bridge_is_explicit():
    ecu = DynamicsECU()
    before = ecu.snapshot()
    with pytest.raises(ValueError):
        ecu.step(.2, DriverInput(.5))
    assert ecu.snapshot() == before
    ecu.update(.01, throttle=50, brake=0)
    other = DynamicsECU()
    other.step(.01, DriverInput(.5))
    assert ecu.snapshot() == other.snapshot()


def test_legacy_benchmarks_cannot_claim_real_vehicle_pass():
    car = reference_vehicle()
    car.name = "Honda Civic 1.5T (2023)"
    assert validate_acceleration(car)["verdict"].startswith("UNVERIFIED")
    assert validate_acceleration(car, 50)["benchmark_time_s"] is None
    assert validate_braking(50)["benchmark_dist_m"] is None
    assert validate_lateral(car)["verdict"].startswith("UNVERIFIED")
