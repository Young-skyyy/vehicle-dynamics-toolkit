"""Compare an aligned physical trace to externally supplied measurements.

Provenance is declared by the caller, not independently authenticated here.
There is no built-in measured dataset and no automatic interpolation.
"""
from __future__ import annotations
import csv
import hashlib
import math
from pathlib import Path

SI_UNITS = {"vx": "m/s", "vy": "m/s", "ax": "m/s2", "ay": "m/s2",
            "yaw_rate": "rad/s", "heading": "rad", "position_x": "m", "position_y": "m"}


def _records(path: Path, fields: list[str]) -> list[dict[str, float]]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not {"t", *fields}.issubset(reader.fieldnames):
            raise ValueError(f"{path}: missing trace columns")
        records = []
        previous = -math.inf
        for raw in reader:
            row = {field: float(raw[field]) for field in ("t", *fields)}
            if not all(math.isfinite(value) for value in row.values()):
                raise ValueError("non-finite trace value")
            if row["t"] < 0 or row["t"] <= previous:
                raise ValueError("timestamps must be non-negative and strictly increasing")
            previous = row["t"]
            records.append(row)
    if not records:
        raise ValueError("empty trace")
    return records


def compare_measured_trace(reference: str | Path, actual: str | Path, metadata: dict) -> dict:
    """Absolute max-error and RMSE limits in SI, specified before the comparison.

metadata requires kind, source, vehicle, configuration, conditions, units and
limits; each field's limits contain max_abs_error and rmse. Caller must use the
same vehicle, conditions and input timeline for simulation and measurement.
"""
    for key in ("source", "vehicle", "configuration", "conditions"):
        if not metadata.get(key):
            raise ValueError(f"missing provenance: {key}")
    if metadata.get("kind") != "measured":
        raise ValueError("real-vehicle comparison requires kind=measured, not synthetic/specification")
    limits = metadata.get("limits", {})
    if not limits or not isinstance(limits, dict):
        raise ValueError("explicit limits required")
    for field, bounds in limits.items():
        if field not in SI_UNITS or metadata.get("units", {}).get(field) != SI_UNITS[field]:
            raise ValueError(f"wrong or missing SI unit for {field}")
        for name in ("max_abs_error", "rmse"):
            value = bounds.get(name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError(f"invalid limit {field}.{name}")
    ref_path, actual_path = Path(reference), Path(actual)
    expected, observed = _records(ref_path, list(limits)), _records(actual_path, list(limits))
    if len(expected) != len(observed):
        raise ValueError("sample count mismatch; no samples may be silently dropped")
    for a, b in zip(expected, observed):
        if abs(a["t"] - b["t"]) > 1e-9:
            raise ValueError("unaligned timestamps; resampling must be explicit upstream")
    results = {}
    for field, bounds in limits.items():
        errors = [b[field] - a[field] for a, b in zip(expected, observed)]
        maximum = max(abs(error) for error in errors)
        rmse = math.sqrt(sum(error**2 for error in errors) / len(errors))
        results[field] = {"max_abs_error": maximum, "rmse": rmse, "limits": bounds,
                          "passed": maximum <= bounds["max_abs_error"] and rmse <= bounds["rmse"]}
    return {"passed": all(result["passed"] for result in results.values()),
            "evidence": "caller_supplied_measured_trace", "samples": len(expected),
            "provenance": metadata, "results": results,
            "reference_sha256": hashlib.sha256(ref_path.read_bytes()).hexdigest(),
            "actual_sha256": hashlib.sha256(actual_path.read_bytes()).hexdigest()}
