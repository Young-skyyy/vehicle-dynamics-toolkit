"""The comparison gate must reject bad output, not only accept equal output."""
import csv
import importlib.util
import json
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location("parity", Path(__file__).parents[1] / "scripts/compare_py_cpp.py")
parity = importlib.util.module_from_spec(spec)
spec.loader.exec_module(parity)


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, ["case", "step"] + parity.FIELDS)
        writer.writeheader()
        writer.writerows(rows)


@pytest.fixture
def records(tmp_path):
    rows = [{"case": "launch", "step": i, **dict.fromkeys(parity.FIELDS, 0.0)} for i in (1, 2)]
    for i, row in enumerate(rows):
        row.update(t=(i+1)*.01, vx=.1*(i+1), gear=1)
    ref, actual = tmp_path / "ref.csv", tmp_path / "actual.csv"
    write_csv(ref, rows)
    write_csv(actual, rows)
    return ref, actual, rows


def test_equal_records_pass(records):
    ref, actual, _ = records
    report = parity.compare_outputs(ref, actual)
    assert report["passed"] and report["samples"] == 2


@pytest.mark.parametrize("field,value", [("vx", 1.0), ("gear", 2), ("t", .011), ("ay", .01)])
def test_bad_numeric_output_fails(records, field, value):
    ref, actual, rows = records
    rows[0][field] = value
    write_csv(actual, rows)
    assert not parity.compare_outputs(ref, actual)["passed"]


@pytest.mark.parametrize("mutation", ["missing", "duplicate", "reorder", "nan", "inf", "empty"])
def test_invalid_sample_sets_are_rejected(records, mutation):
    ref, actual, rows = records
    if mutation == "missing":
        rows.pop()
    elif mutation == "duplicate":
        rows[1] = rows[0]
    elif mutation == "reorder":
        rows.reverse()
    elif mutation == "empty":
        rows.clear()
    else:
        rows[0]["vx"] = float(mutation)
    write_csv(actual, rows)
    with pytest.raises(ValueError):
        parity.compare_outputs(ref, actual)


def test_cli_returns_failure_and_writes_report(records, tmp_path, monkeypatch):
    ref, actual, rows = records
    rows[0]["vx"] += 1
    write_csv(actual, rows)
    monkeypatch.setattr("sys.argv", ["compare_py_cpp.py", "--compare", str(actual),
                                     "--ref", str(ref), "--output", str(tmp_path)])
    assert parity.main() == 1
    assert not json.loads((tmp_path / "report.json").read_text())["passed"]


def test_reference_contains_real_launch_and_both_turns(tmp_path):
    _, ref = parity.generate_reference(tmp_path)
    data = parity.read_records(ref)
    assert len({row["case"] for row in data}) == 12
    assert max(row["vx"] for row in data if row["case"].startswith("launch")) > 5
    assert max(row["yaw_rate"] for row in data if row["case"].startswith("left")) > 0
    assert min(row["yaw_rate"] for row in data if row["case"].startswith("right")) < 0
