"""Compile the installed FMU; compare physical samples and CAN/UDS observations.

No checkout import path is inserted. This is not FMI conformance certification.
"""
from __future__ import annotations
import argparse
import ctypes
import json
import math
from pathlib import Path
import tempfile
import zipfile
import vehicle_dynamics_toolkit.fmu as fmu
from vehicle_dynamics_toolkit import DynamicsECU, CANBus, CANFrame, CANDiagnosticTester, VirtualECUNode
from vehicle_dynamics_toolkit.can_demo import CAN_MESSAGES, generate_frame, parse_can_frame
from vehicle_dynamics_toolkit.fmu_driver import FMUPlant
from vehicle_dynamics_toolkit.scenarios import scenario_rows
from vehicle_dynamics_toolkit.simulation import DynamicsModel, DriverInput


def lifecycle_checks(plant: FMUPlant) -> None:
    lib, component = plant.lib, plant.component
    assert lib.fmi2GetVersion() == b"2.0"
    assert lib.fmi2GetTypesPlatform() == b"default"
    assert lib.fmi2DoStep(component, 0, .01, 1) == 3
    assert not lib.fmi2Instantiate(b"bad", 1, b"wrong-guid", None, None, 0, 0)
    plant.initialize(20, 2)
    assert plant.state()["vx"] == 20 and plant.state()["t"] == 2
    assert lib.fmi2DoStep(component, 0, .01, 1) == 3
    for dt in (0, -.01, .1001, float("nan"), float("inf")):
        assert lib.fmi2DoStep(component, 2, dt, 1) == 3
    before = plant.state()
    refs = (ctypes.c_uint * 2)(1, 999)
    values = (ctypes.c_double * 2)(.5, 0)
    assert lib.fmi2SetReal(component, refs, 2, values) == 3
    out = (ctypes.c_double * 1)()
    assert lib.fmi2GetReal(component, (ctypes.c_uint * 1)(1), 1, out) == 0
    assert out[0] == 0  # invalid batch cannot partially update
    for ref, value in ((1, 50), (2, -1), (8, .8), (19, 5), (1, float("nan"))):
        assert lib.fmi2SetReal(component, (ctypes.c_uint * 1)(ref), 1,
                              (ctypes.c_double * 1)(value)) == 3
    assert plant.state() == before
    assert lib.fmi2GetReal(component, (ctypes.c_uint * 1)(6), 1, out) == 3
    assert lib.fmi2Terminate(component) == 0
    assert lib.fmi2DoStep(component, 2, .01, 1) == 3
    plant.initialize()
    assert plant.state()["vx"] == 0 and plant.state()["t"] == 0


def check(compiler: str | None = None, require_installed: bool = False,
          output: Path | None = None) -> dict:
    module = Path(fmu.__file__).resolve()
    source_root = Path(__file__).resolve().parents[1] / "src"
    if require_installed and source_root in module.parents:
        raise RuntimeError("Test imported checkout instead of installed wheel")
    maxima = {field: 0. for field in (*fmu.STATE_REFS, "gear")}
    failed = 0
    first_failures = []
    samples = observations = 0
    cases = set()
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        archive = fmu.build_fmu(root / "vehicle.fmu", compiler=compiler)
        with zipfile.ZipFile(archive) as packed:
            packed.extractall(root)
        binary, = (root / "binaries").rglob("ecu_fmu.*")
        with FMUPlant(binary) as plant:
            lifecycle_checks(plant)
            active = None
            for row in scenario_rows():
                if row["case"] != active:
                    active = row["case"]
                    cases.add(active)
                    model = DynamicsModel(initial_vx=row["initial_vx"])
                    ecu = DynamicsECU(DynamicsModel(initial_vx=row["initial_vx"]))
                    bus = CANBus(clock=ecu.clock)
                    VirtualECUNode(ecu, bus)
                    tester = CANDiagnosticTester(bus)
                    plant.initialize(row["initial_vx"])
                command = DriverInput(row["throttle"], row["brake"], row["steer"])
                expected = model.step(row["dt"], command.throttle, command.brake, command.steer)
                actual = plant.step(row["dt"], command)
                physical_ecu = ecu.step(row["dt"], command)
                for adapter, state in (("FMU", actual), ("DynamicsECU", physical_ecu)):
                    for field in maxima:
                        if not math.isfinite(state[field]) or not math.isfinite(expected[field]):
                            raise AssertionError(f"non-finite physical output: {adapter}/{field}")
                        error = abs(state[field] - expected[field])
                        maxima[field] = max(maxima[field], error)
                        limit = 0 if field == "gear" else 1e-9 if field == "t" else 1e-6 + .001 * abs(expected[field])
                        if error > limit:
                            failed += 1
                            if len(first_failures) < 20:
                                first_failures.append(dict(adapter=adapter, case=active, step=row["step"],
                                                           field=field, error=error, limit=limit))
                samples += 1
                if row["step"] == 1 or row["step"] % 100 == 0:
                    data = generate_frame("EngineData", CAN_MESSAGES["EngineData"], ecu, ecu.clock.now)
                    bus.send(CANFrame(CAN_MESSAGES["EngineData"]["id"], data, ecu.clock.now))
                    decoded = parse_can_frame(bus.history()[-1].data, CAN_MESSAGES["EngineData"])
                    assert abs(decoded["车速"] - expected["vx"] * 3.6) <= .01 + 1e-9
                    assert abs(decoded["发动机转速"] - expected["engine_rpm"]) <= .25 + 1e-9
                    for did, target, scale in ((0x000D, expected["vx"] * 3.6, .01),
                                               (0x000C, expected["engine_rpm"], 1.)):
                        response = tester.request(bytes([0x22, did >> 8, did & 255]))
                        assert response[:3] == bytes([0x62, did >> 8, did & 255])
                        assert abs(int.from_bytes(response[3:], "big") * scale - target) <= scale + 1e-9
                    observations += 1
    report = dict(passed=failed == 0, cases=len(cases), samples=samples,
                  can_uds_observations=observations, failed_values=failed,
                  max_abs_error=maxima, first_failures=first_failures, imported=str(module),
                  evidence="adapter_parity", real_vehicle_accuracy="NOT_VALIDATED")
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise AssertionError("FMU / physical ECU parity failed")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler")
    parser.add_argument("--require-installed", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    check(args.compiler, args.require_installed, args.output)
