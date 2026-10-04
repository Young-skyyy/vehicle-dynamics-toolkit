#!/usr/bin/env python3
"""Replay the ROS2 C++ core and compare every sample with Python.

--check compiles/replays/compares, --generate writes inputs and reference,
--compare ACTUAL.csv compares an existing replay. No ROS installation needed.
This is core parity, not a ROS bag, transport latency or deadline test.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from vehicle_dynamics_toolkit.simulation import DynamicsModel  # noqa: E402

FIELDS = ["t", "vx", "ax", "vy", "ay", "yaw_rate", "heading",
          "position_x", "position_y", "gear", "engine_rpm", "engine_torque"]
INPUT_FIELDS = ["case", "step", "dt", "initial_vx", "throttle", "brake", "steer"]


from vehicle_dynamics_toolkit.scenarios import scenario_rows


def generate_reference(directory: Path) -> tuple[Path, Path]:
    directory.mkdir(parents=True, exist_ok=True)
    inputs, reference = directory / "inputs.csv", directory / "python.csv"
    with inputs.open("w", newline="", encoding="utf-8") as f_in, reference.open("w", newline="", encoding="utf-8") as f_ref:
        writer = csv.DictWriter(f_in, INPUT_FIELDS)
        ref_writer = csv.DictWriter(f_ref, ["case", "step"] + FIELDS)
        writer.writeheader()
        ref_writer.writeheader()
        model = DynamicsModel()
        active_case = None
        for row in scenario_rows():
            writer.writerow(row)
            if row["case"] != active_case:
                model = DynamicsModel(initial_vx=row["initial_vx"])
                active_case = row["case"]
            state = model.step(row["dt"], row["throttle"], row["brake"], row["steer"])
            ref_writer.writerow({"case": row["case"], "step": row["step"], **state})
    return inputs, reference


def read_records(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None or not {"case", "step", *FIELDS}.issubset(reader.fieldnames):
            raise ValueError(f"{path}: missing required columns")
        records = []
        seen = set()
        for row in reader:
            key = row["case"], int(row["step"])
            if not key[0] or key in seen:
                raise ValueError(f"{path}: missing case name or duplicate sample {key}")
            seen.add(key)
            values = {field: float(row[field]) for field in FIELDS}
            if not all(math.isfinite(value) for value in values.values()):
                raise ValueError(f"{path}: non-finite sample {key}")
            records.append({"case": key[0], "step": key[1], **values})
        if not records:
            raise ValueError(f"{path}: empty output")
        return records


def compare_outputs(ref_path: Path, actual_path: Path, rtol: float = 0.001, atol: float = 1e-6) -> dict:
    """|actual-reference| <= atol + rtol*|reference|; gear exact, time <=1e-9s.

    SI units. No samples are interpolated or silently dropped.
    """
    if not all(math.isfinite(x) and x >= 0 for x in (rtol, atol)):
        raise ValueError("tolerances must be finite and non-negative")
    reference, actual = read_records(ref_path), read_records(actual_path)
    if len(reference) != len(actual):
        raise ValueError(f"sample count mismatch: {len(reference)} vs {len(actual)}")
    failures = []
    failed_values = 0
    maxima = dict.fromkeys(FIELDS, 0.0)
    for expected, observed in zip(reference, actual):
        key = expected["case"], expected["step"]
        if key != (observed["case"], observed["step"]):
            raise ValueError(f"sample ordering mismatch at {key}")
        for field in FIELDS:
            error = abs(observed[field] - expected[field])
            maxima[field] = max(maxima[field], error)
            limit = 0.0 if field == "gear" else 1e-9 if field == "t" else atol + rtol * abs(expected[field])
            if error > limit:
                failed_values += 1
                if len(failures) < 20:
                    failures.append(dict(case=key[0], step=key[1], field=field,
                                         expected=expected[field], actual=observed[field], limit=limit))
    return dict(passed=failed_values == 0, samples=len(reference),
                cases=len({row["case"] for row in reference}), rtol=rtol, atol=atol,
                max_abs_error=maxima, failed_values=failed_values, first_failures=failures)


def compile_runner(directory: Path, compiler: str | None) -> Path:
    compiler = compiler or os.environ.get("CXX") or shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
    if not compiler:
        raise RuntimeError("C++17 compiler not found; pass --compiler or --runner")
    command = [compiler]
    if Path(compiler).stem == "zig":
        command.append("c++")
    runner = (directory / ("dynamics_runner.exe" if os.name == "nt" else "dynamics_runner")).resolve()
    command += ["-std=c++17", "-O2", "-Wall", "-Wextra", "-I",
                str(ROOT / "ros2_ws/src/vehicle_dynamics_node/include"), "-I",
                str(ROOT / "src/vehicle_dynamics_toolkit/native"),
                str(ROOT / "ros2_ws/src/vehicle_dynamics_node/src/dynamics_runner.cpp"), "-o", str(runner)]
    log = directory / "compile.log"
    with log.open("w", encoding="utf-8") as output:
        completed = subprocess.run(command, stdout=output, stderr=subprocess.STDOUT)
    if completed.returncode:
        raise RuntimeError(f"C++ compilation failed; see {log}")
    return runner


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--generate", action="store_true")
    mode.add_argument("--compare", type=Path, help="actual C++ output CSV")
    parser.add_argument("--ref", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / "build/parity")
    parser.add_argument("--runner", type=Path, help="prebuilt C++ replay executable")
    parser.add_argument("--compiler", help="compiler executable path (also supports zig)")
    parser.add_argument("--rtol", type=float, default=.001)
    parser.add_argument("--atol", type=float, default=1e-6)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    try:
        if args.generate or args.check:
            inputs, reference = generate_reference(args.output)
        else:
            reference = args.ref or args.output / "python.csv"
        if args.generate:
            print(f"Generated {inputs} and {reference}")
            return 0
        actual = args.compare
        if args.check:
            runner = args.runner.resolve() if args.runner else compile_runner(args.output, args.compiler)
            actual = args.output / "cpp.csv"
            subprocess.run([str(runner), str(inputs), str(actual)], check=True)
        report = compare_outputs(reference, actual, args.rtol, args.atol)
    except (ValueError, TypeError, OverflowError, csv.Error, OSError, RuntimeError, subprocess.CalledProcessError) as error:
        report = {"passed": False, "error": str(error)}
    (args.output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
