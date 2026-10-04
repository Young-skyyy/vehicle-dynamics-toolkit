"""Reproducible physical vehicle / CAN / UDS showcase, with optional native FMU.

The 20-second control timeline is fixed, uses the reference vehicle, and writes
simulation evidence. Faults affect diagnostic status or transport, not mechanics.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import subprocess
from typing import cast
import zipfile

from .can_bus import CANBus, CANFrame, FrameFilter
from .can_demo import CAN_MESSAGES, generate_dbc, generate_frame, parse_can_frame
from .ecu import DynamicsECU
from .fmu import build_fmu
from .fmu_driver import FMUPlant
from .simulation import DriverInput, DynamicsModel, DynamicsState
from .uds_can import CANDiagnosticTester, VirtualECUNode

DT = .01
SAMPLES = 2000
STATE_FIELDS = list(asdict(DynamicsState()))


def _control(step: int) -> tuple[str, DriverInput]:
    if step <= 800:
        return "launch", DriverInput(.65)
    if step <= 1200:
        return "turn", DriverInput(.25, steer=.02)
    if step <= 1500:
        return "coast", DriverInput()
    return "brake_to_stop", DriverInput(brake=.7)


def _plot(rows: list[dict], output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    time = [r["t"] for r in rows]
    axes[0, 0].plot(time, [r["vx"] * 3.6 for r in rows], label="Physical speed")
    axes[0, 0].plot(time, [r["can_speed_kmh"] if r["can_speed_kmh"] != "" else math.nan for r in rows],
                    "--", label="CAN decoded")
    axes[0, 0].set(ylabel="Speed (km/h)", xlabel="Time (s)")
    axes[0, 0].legend()
    axes[0, 1].plot(time, [r["throttle"] for r in rows], label="Throttle")
    axes[0, 1].plot(time, [r["brake"] for r in rows], label="Brake")
    axes[0, 1].set(ylabel="Normalized command", xlabel="Time (s)")
    axes[0, 1].legend()
    axes[1, 0].plot([r["position_x"] for r in rows], [r["position_y"] for r in rows])
    axes[1, 0].set(xlabel="World x (m)", ylabel="World y (m)")
    axes[1, 0].set_aspect("equal", adjustable="datalim")
    axes[1, 1].plot(time, [r["yaw_rate"] for r in rows])
    axes[1, 1].set(xlabel="Time (s)", ylabel="Yaw rate (rad/s)")
    for ax in (axes[0, 0], axes[0, 1], axes[1, 1]):
        for point in (8, 12, 15):
            ax.axvline(point, color="grey", linewidth=.6, alpha=.5)
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.suptitle("Reference vehicle: launch / turn / coast / brake\n"
                 "CAN gap at 9 s | Diagnostic fault at 10 s | DTC cleared at 14 s\n"
                 "Simulation evidence; not measured vehicle data", fontsize=12)
    fig.savefig(output, dpi=150)
    plt.close(fig)


def run_showcase(output: str | Path, *, with_fmu: bool = False,
                 compiler: str | None = None, plot: bool = True,
                 frame_filter: FrameFilter | None = None) -> dict:
    """Run one deterministic control timeline, verify interfaces and write evidence.

    A missing periodic frame at 9 s and a P0301 diagnostic status at 10 s are
    intentional. CAN reception detects the gap; UDS reads and clears the DTC.
    An optional extra filter allows repeatable transport corruption experiments.
    """
    directory = Path(output).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    ecu = DynamicsECU()
    ecu.clear_faults()  # Clean initial diagnostic state, unlike the legacy fixture.
    reference = DynamicsModel()
    events: list[dict] = []
    rows: list[dict] = []
    failures: list[dict] = []
    failed_checks = 0
    maxima = {field: 0. for field in STATE_FIELDS}
    can_errors = {"speed_kmh": 0., "rpm": 0.}
    uds_errors = {"speed_kmh": 0., "rpm": 0.}
    received: list[CANFrame] = []
    last_can_time: float | None = None
    gaps = 0
    engine_id = cast(int, CAN_MESSAGES["EngineData"]["id"])

    def check(condition: bool, name: str, **details) -> None:
        nonlocal failed_checks
        if not condition:
            failed_checks += 1
            if len(failures) < 20:
                failures.append({"check": name, "t": ecu.clock.now, **details})

    def transport_filter(frame: CANFrame) -> CANFrame | None:
        if frame.can_id == engine_id and abs(frame.timestamp_s - 9) < 1e-8:
            events.append({"t": frame.timestamp_s, "event": "periodic_frame_dropped", "can_id": frame.can_id})
            return None
        return frame if frame_filter is None else frame_filter(frame)

    bus = CANBus(clock=ecu.clock, frame_filter=transport_filter)
    bus.subscribe(engine_id, received.append)
    VirtualECUNode(ecu, bus)
    tester = CANDiagnosticTester(bus)

    def request(name: str, payload: bytes) -> bytes:
        response = tester.request(payload)
        events.append({"t": ecu.clock.now, "event": name,
                       "request_hex": payload.hex(), "response_hex": response.hex()})
        return response

    vin = request("read_vin_over_isotp", b"\x22\xf1\x90")
    check(vin[:3] == b"\x62\xf1\x90" and len(vin) == 20, "segmented_vin_read")
    clean_dtc = b"\x59\x02\x01\xff"
    check(request("initial_dtc_read", b"\x19\x02\xff") == clean_dtc, "initial_dtc_clear")
    generate_dbc(str(directory / "signals.dbc"))

    with ExitStack() as stack:
        native: FMUPlant | None = None
        if with_fmu:
            archive = build_fmu(directory / "reference_vehicle.fmu", compiler=compiler)
            import tempfile
            extracted = Path(stack.enter_context(tempfile.TemporaryDirectory()))
            with zipfile.ZipFile(archive) as packed:
                packed.extractall(extracted)
            binary, = (extracted / "binaries").rglob("ecu_fmu.*")
            native = stack.enter_context(FMUPlant(binary))
            native.initialize()

        for step in range(1, SAMPLES + 1):
            phase, command = _control(step)
            expected = reference.step(DT, command.throttle, command.brake, command.steer)
            state = ecu.step(DT, command)
            adapters = [("physical_ecu", state)]
            native_state = native.step(DT, command) if native is not None else None
            if native_state is not None:
                adapters.append(("fmu", native_state))
            for adapter, actual in adapters:
                for field in STATE_FIELDS:
                    error = abs(actual[field] - expected[field])
                    maxima[field] = max(maxima[field], error)
                    limit = 0 if field == "gear" else 1e-9 if field == "t" else 1e-6 + .001 * abs(expected[field])
                    check(math.isfinite(actual[field]) and error <= limit, "physical_parity",
                          adapter=adapter, field=field, error=error, limit=limit)

            count_before = len(received)
            data = generate_frame("EngineData", CAN_MESSAGES["EngineData"], ecu, ecu.clock.now)
            bus.send(CANFrame(engine_id, bytes(data), ecu.clock.now))
            observed_can: dict[str, float] = {}
            if len(received) > count_before:
                frame = received[-1]
                if last_can_time is not None and frame.timestamp_s - last_can_time > DT * 1.5:
                    gaps += 1
                    events.append({"t": frame.timestamp_s, "event": "can_gap_detected",
                                   "gap_s": frame.timestamp_s - last_can_time})
                last_can_time = frame.timestamp_s
                observed_can = parse_can_frame(list(frame.data), CAN_MESSAGES["EngineData"])
                for label, signal, target, quantum in (
                    ("speed_kmh", "车速", state["vx"] * 3.6, .01),
                    ("rpm", "发动机转速", state["engine_rpm"], .25),
                ):
                    error = abs(observed_can[signal] - target)
                    can_errors[label] = max(can_errors[label], error)
                    check(error <= quantum + 1e-9, "can_observation", field=label, error=error)

            uds_speed: float | str = ""
            uds_rpm: float | str = ""
            if step % 100 == 0:
                for label, did, target, quantum in (
                    ("speed_kmh", 0x000D, state["vx"] * 3.6, .01),
                    ("rpm", 0x000C, state["engine_rpm"], 1.),
                ):
                    response = request(f"read_{label}", bytes([0x22, did >> 8, did & 255]))
                    valid = response[:3] == bytes([0x62, did >> 8, did & 255]) and len(response) == 5
                    check(valid, "uds_response", field=label)
                    if valid:
                        value = int.from_bytes(response[3:], "big") * quantum
                        error = abs(value - target)
                        uds_errors[label] = max(uds_errors[label], error)
                        check(error <= quantum + 1e-9, "uds_observation", field=label, error=error)
                        if label == "speed_kmh":
                            uds_speed = value
                        else:
                            uds_rpm = value

            if step == 1000:
                ecu.inject_fault("P0301")
                fault_response = request("read_injected_dtc", b"\x19\x02\xff")
                check(fault_response[:4] == clean_dtc and b"\x00\x03\x01" in fault_response[4:], "fault_detected")
            if step == 1400:
                check(request("enter_extended_session", b"\x10\x03")[:2] == b"\x50\x03", "extended_session")
                check(request("clear_dtc", b"\x14\xff\xff\xff") == b"\x54", "fault_clear")
                check(request("read_cleared_dtc", b"\x19\x02\xff") == clean_dtc, "fault_cleared")

            row = {"phase": phase, "throttle": command.throttle, "brake": command.brake,
                   "steer_rad": command.steer, **{f: state[f] for f in STATE_FIELDS},
                   "can_speed_kmh": observed_can.get("车速", ""),
                   "can_rpm": observed_can.get("发动机转速", ""),
                   "uds_speed_kmh": uds_speed, "uds_rpm": uds_rpm,
                   "fmu_vx_m_s": native_state["vx"] if native_state is not None else ""}
            rows.append(row)

    check(len(received) == SAMPLES - 1, "can_received_count", actual=len(received), expected=SAMPLES - 1)
    check(gaps == 1, "can_gap_count", actual=gaps, expected=1)
    check(rows[-1]["vx"] == 0, "vehicle_stopped", actual=rows[-1]["vx"])
    with (directory / "states.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    asc = ["date Thu Jan 01 00:00:00 1970", "base hex  timestamps absolute", "internal events logged",
           "// Deterministic simulation; channel 1 includes periodic and diagnostic CAN traffic.", "Begin Triggerblock"]
    for frame in bus.history():
        payload = " ".join(f"{value:02X}" for value in frame.data)
        asc.append(f"{frame.timestamp_s:11.6f} 1 {frame.can_id:03X} Rx d {len(frame.data)} {payload}")
    asc.append("End Triggerblock")
    (directory / "can.asc").write_text("\n".join(asc) + "\n", encoding="utf-8")
    (directory / "events.json").write_text(json.dumps(events, indent=2), encoding="utf-8")
    report = {"passed": failed_checks == 0, "samples": len(rows), "duration_s": rows[-1]["t"],
              "dt_s": DT, "seed": 42,
              "scenario": "launch_turn_coast_brake", "vehicle": "reference_vehicle",
              "fmu": "CHECKED" if with_fmu else "NOT_RUN", "real_vehicle_accuracy": "NOT_VALIDATED",
              "fault_scope": "CAN transport drop and diagnostic DTC status only; mechanics unchanged",
              "received_periodic_frames": len(received), "detected_can_gaps": gaps,
              "peak_speed_kmh": max(r["vx"] for r in rows) * 3.6,
              "final_speed_kmh": rows[-1]["vx"] * 3.6,
              "max_physical_abs_error": maxima, "max_can_abs_error": can_errors,
              "max_uds_abs_error": uds_errors, "failed_checks": failed_checks, "first_failures": failures}
    if plot:
        _plot(rows, directory / "overview.png")
    report["artifacts"] = ["states.csv", "can.asc", "signals.dbc", "events.json"]
    if plot:
        report["artifacts"].append("overview.png")
    if with_fmu:
        report["artifacts"].append("reference_vehicle.fmu")
    (directory / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/showcase"))
    parser.add_argument("--with-fmu", action="store_true")
    parser.add_argument("--compiler", help="C++17 compiler executable (including Zig)")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    try:
        report = run_showcase(args.output, with_fmu=args.with_fmu, compiler=args.compiler, plot=not args.no_plot)
    except (OSError, RuntimeError, ValueError, ImportError, subprocess.CalledProcessError) as error:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "report.json").write_text(json.dumps({"passed": False, "error": str(error)}, indent=2),
                                                 encoding="utf-8")
        print(f"FAIL: {error}")
        return 1
    print(f"{'PASS' if report['passed'] else 'FAIL'}: {report['samples']} samples, "
          f"{report['received_periodic_frames']} CAN frames, "
          f"{report['detected_can_gaps']} detected gap; FMU {report['fmu']}")
    print(f"Evidence: {args.output.resolve()}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
