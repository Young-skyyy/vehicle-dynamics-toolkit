"""Compare matching measured and simulated SI traces; emit a reproducible report."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vehicle_dynamics_toolkit.trace_validation import compare_measured_trace

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--actual", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("build/measured/report.json"))
    args = parser.parse_args()
    try:
        report = compare_measured_trace(args.reference, args.actual,
                                        json.loads(args.metadata.read_text(encoding="utf-8")))
    except (ValueError, TypeError, KeyError, OverflowError, OSError) as error:
        report = {"passed": False, "error": str(error)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 1)
