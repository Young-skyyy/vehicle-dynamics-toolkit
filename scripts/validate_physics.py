"""Check physical equations against independent solutions; fail on discrepancies."""
from pathlib import Path
import argparse
import json
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vehicle_dynamics_toolkit.physical_validation import physics_report

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/physics/report.json"))
    args = parser.parse_args()
    report = physics_report()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"{'PASS' if report['passed'] else 'FAIL'}: {report['checks']} analytical checks; real-car accuracy NOT_VALIDATED")
    for result in report["results"]:
        if not result["passed"]:
            print(result)
    raise SystemExit(0 if report["passed"] else 1)
