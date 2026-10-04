"""Synthetic fixtures exercise the measured-data interface; no real telemetry here."""
import copy
import pytest
from vehicle_dynamics_toolkit.trace_validation import compare_measured_trace


@pytest.fixture
def traces(tmp_path):
    reference, actual = tmp_path / "reference.csv", tmp_path / "actual.csv"
    reference.write_text("t,vx\n0,10\n1,9\n", encoding="utf-8")
    actual.write_text("t,vx\n0,10.1\n1,9.2\n", encoding="utf-8")
    # Caller-declared provenance only. Source explicitly identifies the test fixture.
    metadata = dict(kind="measured", source="synthetic unit-test fixture (not real data)",
                    vehicle="fixture", configuration={"mass_kg": 1500}, conditions="test",
                    units={"vx": "m/s"}, limits={"vx": {"max_abs_error": .3, "rmse": .2}})
    return reference, actual, metadata


def test_trace_report_has_errors_and_content_hashes(traces):
    report = compare_measured_trace(*traces)
    assert report["passed"]
    assert report["results"]["vx"]["max_abs_error"] == pytest.approx(.2)
    assert report["results"]["vx"]["rmse"] == pytest.approx((.01 / 2 + .04 / 2)**.5)
    assert len(report["reference_sha256"]) == 64
    assert report["reference_sha256"] != report["actual_sha256"]


def test_trace_exceeding_declared_limits_fails(traces):
    reference, actual, metadata = traces
    actual.write_text("t,vx\n0,10\n1,11\n", encoding="utf-8")
    assert not compare_measured_trace(reference, actual, metadata)["passed"]


@pytest.mark.parametrize("content", ["t,vx\n", "t,vx\n0,10\n", "t,vx\n0,10\n2,9\n",
                                     "t,vx\n0,10\n0,9\n", "t,vx\n0,10\n1,nan\n", "t,ax\n0,0\n1,0\n"])
def test_invalid_or_unaligned_trace_is_rejected(traces, content):
    reference, actual, metadata = traces
    actual.write_text(content, encoding="utf-8")
    with pytest.raises(ValueError):
        compare_measured_trace(reference, actual, metadata)


@pytest.mark.parametrize("change", [dict(kind="synthetic"), dict(source=""), dict(configuration={}),
                                    dict(units={"vx": "km/h"}), dict(limits={}),
                                    dict(limits={"vx": {"max_abs_error": float("nan"), "rmse": .2}})])
def test_missing_provenance_wrong_units_and_bad_limits_are_rejected(traces, change):
    reference, actual, metadata = traces
    metadata = {**copy.deepcopy(metadata), **change}
    with pytest.raises(ValueError):
        compare_measured_trace(reference, actual, metadata)
