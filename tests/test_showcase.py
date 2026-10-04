"""End-to-end showcase: evidence integrity, repeatability and visible failures."""
import csv
import json
from vehicle_dynamics_toolkit.can_bus import CANFrame
from vehicle_dynamics_toolkit.showcase import run_showcase


def test_showcase_records_physical_inputs_fault_detection_and_recovery(tmp_path):
    report = run_showcase(tmp_path, plot=False)
    assert report["passed"], report["first_failures"]
    assert report["samples"] == 2000
    assert report["fmu"] == "NOT_RUN"
    assert report["received_periodic_frames"] == 1999
    assert report["detected_can_gaps"] == 1
    assert report["peak_speed_kmh"] > 20 and report["final_speed_kmh"] == 0
    assert report["real_vehicle_accuracy"] == "NOT_VALIDATED"
    with (tmp_path / "states.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert {r["phase"] for r in rows} == {"launch", "turn", "coast", "brake_to_stop"}
    assert sum(r["can_speed_kmh"] == "" for r in rows) == 1
    assert all(r["fmu_vx_m_s"] == "" for r in rows)
    assert float(rows[-1]["t"]) == report["duration_s"]
    events = json.loads((tmp_path / "events.json").read_text())
    names = {e["event"] for e in events}
    assert {"read_vin_over_isotp", "can_gap_detected", "read_injected_dtc", "clear_dtc", "read_cleared_dtc"} <= names
    assert (tmp_path / "signals.dbc").is_file()
    assert "7E0 Rx" in (tmp_path / "can.asc").read_text()
    assert "7E8 Rx" in (tmp_path / "can.asc").read_text()


def test_showcase_is_reproducible(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    assert run_showcase(a, plot=False) == run_showcase(b, plot=False)
    for file in ("states.csv", "events.json", "can.asc", "report.json"):
        assert (a / file).read_bytes() == (b / file).read_bytes()


def test_corrupted_physical_can_signal_cannot_pass(tmp_path):
    def corrupt(frame):
        if frame.can_id == 0x0C9:
            data = bytearray(frame.data)
            data[4] ^= 255  # speed's least significant byte in the teaching DBC
            return CANFrame(frame.can_id, bytes(data), frame.timestamp_s)
        return frame
    report = run_showcase(tmp_path, plot=False, frame_filter=corrupt)
    assert not report["passed"]
    assert any(f["check"] == "can_observation" for f in report["first_failures"])
    assert not json.loads((tmp_path / "report.json").read_text())["passed"]


def test_missing_diagnostic_fault_is_a_failure(tmp_path, monkeypatch):
    from vehicle_dynamics_toolkit.ecu import DynamicsECU
    monkeypatch.setattr(DynamicsECU, "inject_fault", lambda *args: None)
    report = run_showcase(tmp_path, plot=False)
    assert not report["passed"]
    assert any(f["check"] == "fault_detected" for f in report["first_failures"])
