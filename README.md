# Vehicle Dynamics Toolkit — CAN · UDS · Lateral Dynamics Simulation & Testing

[![pytest](https://github.com/Young-skyyy/vehicle-dynamics-toolkit/actions/workflows/test.yml/badge.svg)](https://github.com/Young-skyyy/vehicle-dynamics-toolkit/actions/workflows/test.yml)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue)](https://en.cppreference.com/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-orange)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![Star](https://img.shields.io/github/stars/Young-skyyy/vehicle-dynamics-toolkit?style=social)](https://github.com/Young-skyyy/vehicle-dynamics-toolkit)

**A Python library + C++ ROS2 node for automotive software testing: CAN bus simulation with DBC export, ISO 14229 UDS diagnostic stack, and 2-DOF bicycle model with Pacejka Magic Formula tire model.**

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  Layer 1 — Python Analysis & Simulation                         │
│  vehicle.py · lateral_dynamics.py · can_demo.py · uds.py        │
│  243+ pytest cases · GitHub Actions CI (3.10/3.11/3.12 + mypy)  │
├─────────────────────────────────────────────────────────────────┤
│  Layer 2 — C++ ROS2 Real-Time Nodes                             │
│  vehicle_dynamics_node (rclcpp, 100 Hz)                         │
│  uds_server (ISO 14229 Service, 5 ECUs)                         │
│  vehicle_msgs (ROS2 IDL: State / Control / UDS)                 │
└─────────────────────────────────────────────────────────────────┘
```

Layer 2 C++ nodes port algorithms from Layer 1 Python models. Validation between layers uses [`scripts/compare_py_cpp.py`](scripts/compare_py_cpp.py): Python generates per-second reference JSON with the same default parameters as C++, then C++ output from recorded ROS2 bags is compared field-by-field against the reference. The C++ node measures real-time jitter to quantify timing deviations.

---
## Engineering Depth — Three Implementation Details Worth Noticing

While the module list above covers what this project does, these three specific implementation choices show real engineering consideration beyond a simple feature checklist.

### 1. UDS SecurityAccess SPR Bit (ISO 14229-1 §9.5)

Most student UDS projects stop at `ReadDataByIdentifier`. SecurityAccess (0x27) with `requestSeed` / `sendKey` is uncommon. Handling the **Suppress Positive Response (SPR) bit** — the most significant bit of the sub-function byte — is rare even in industrial code:

```python
suppress = (sub & 0x80) != 0   # bit7 = SPR
actual_sub = sub & 0x7F        # strip SPR to get real sub-function

if actual_sub == 0x01:         # requestSeed (0x01 or 0x81)
    self._pending_seed = random.randint(0, 0xFFFF)
    if suppress:
        return b""              # positive response suppressed
    return bytes([0x67, 0x01]) + seed_bytes

elif actual_sub == 0x02:       # sendKey (0x02 or 0x82)
    # ...key validation...
    if suppress:
        return b""              # unlock succeeded, but response suppressed
```

SPR changes the protocol semantics: `0x81` ≠ `0x01`, even though both request a seed. The seed is still generated and stored so the subsequent `sendKey` can validate against it — but the ECU stays silent. Negative responses are never suppressed regardless of SPR. This behavior is documented with 3 dedicated test cases in `test_uds.py`.

### 2. CAN Motorola/Intel Byte Order — Bit-Position-Aware Encoding

CAN DBC files encode signal layout with a `start_bit | length @ byte_order` triplet like `24|16@0+`. The `@0` vs `@1` flag determines whether bits are laid out MSB-first (Motorola, bytes count down) or LSB-first (Intel, bytes count up). When a 16-bit signal starts at bit 24 in Motorola order, it spans byte 3 (bits 0–7) down to byte 2 (bits 0–7) — the most significant bits live in byte 3, the least significant in byte 2.

The `_signal_bit_positions()` function decomposes this into an explicit position list:

```python
# Motorola 16-bit signal at start_bit=24:
# Result: [(3, 0, 15), (3, 1, 14), ..., (3, 7, 8), (2, 0, 7), ..., (2, 7, 0)]
#          byte 3, MSB (shift 15) → byte 2, LSB (shift 0)

positions = _signal_bit_positions(start_bit=24, length=16, byte_order="motorola")
assert positions[0]  == (3, 0, 15)   # MSB
assert positions[-1] == (2, 7, 0)    # LSB
```

This is verified by test cases that round-trip encode → decode for mixed Motorola+Intel signals within a single CAN frame, and by explicit bit-position assertions matching the DBC standard.

### 3. Model Validation — Self-Awareness Over Self-Promotion

The 6-benchmark validation table in the section above is not just a green checkmark gallery. Each discrepancy is explained:

| Discrepancy | Actual | Model | Why |
|---|---|---|---|
| Civic 100–0 braking | 37.0 m | 43.7 m (+18%) | Braking uses `v²/(2μg)` with fixed μ=0.90. Real braking involves weight transfer (adds load to front axle → higher peak μ), brake force distribution, tire nonlinearity at the friction ellipse limit, and thermal fade resistance — none of which are modeled here. |
| Tiguan 0–100 acceleration | 9.0 s | 9.5 s (+5.6%) | Turbo torque plateau flattens the mid-range but drops faster at high RPM than the NA curve. The model captures this qualitatively but the exact falloff rate depends on turbo sizing (A/R ratio, boost curve) — which varies across the EA888 engine family. |

Every FAIL-to-PASS fix (Tiguan engine type, braking friction coefficient) is documented in the [CHANGELOG](CHANGELOG.md). The validation module exists to expose model limitations, not to pretend they don't exist.

---

## What Problems This Solves

### 1. CAN Bus Testing

Simulating and debugging multi-ECU CAN networks without hardware.

The Python module builds a virtual CAN bus with 5 ECUs (Engine, Transmission, ABS, BCM, Gateway), each sending periodic messages with realistic payloads. It encodes signals in both Motorola and Intel byte order, generates complete DBC files for tools like Vector CANoe or SavvyCAN, and injects configurable faults — CRC errors, missing frames, bit flips — to test error-handling paths. Bus load is calculated from frame bit counts and baud rate, with overflow detection and frame-drop strategies for high-load scenarios.

Built on top of the CAN layer, the UDS stack uses standard CAN identifiers (0x7E0/0x7E8 for physical addressing, 0x7DF for functional) as the transport — every diagnostic request and response is packed into CAN frames with ISO-TP (ISO 15765-2) multi-frame flow control.

### 2. UDS Diagnostics (ISO 14229)

A diagnostic protocol stack that models the full ECU session lifecycle.

The Python reference implementation implements the core ISO 14229 services: `DiagnosticSessionControl` (0x10) with session state transitions, `ECUReset` (0x11), `ReadDataByIdentifier` (0x22), `SecurityAccess` (0x27) with seed/key challenge-response, `ReadDTCInformation` (0x19) with DTC status byte decoding per ISO 14229-1 Annex D, and `TesterPresent` (0x3E) with S3 timer handling. A session state machine enforces service permissions — e.g., SecurityAccess is rejected in DefaultSession, and TesterPresent keeps an extended session alive. Multi-frame responses (e.g., 17-byte VIN via DID 0xF190) are handled by an **ISO 15765-2 (ISO-TP) transport layer** (`iso_tp.py`) that segments payloads into CAN frames with `FirstFrame` → `FlowControl` → `ConsecutiveFrame` flow, including sequence number validation and overflow detection. The C++ ROS2 `uds_server` node runs the same logic for up to 5 ECUs simultaneously, exposing diagnostic endpoints as ROS2 services so test scripts can automate UDS test sequences without real hardware.

### 3. Vehicle Lateral Dynamics

Predicting vehicle handling behaviour from first principles.

A 2-DOF (degrees of freedom) bicycle model computes slip angles at front and rear axles, maps them to lateral cornering forces through the Pacejka Magic Formula (`Fy = D·sin(C·arctan(Bα − E(Bα − arctan(Bα))))`), and then solves the yaw moment equilibrium to obtain yaw rate and lateral acceleration. From the steady-state cornering equations the model derives the understeer gradient — distinguishing understeer, neutral, and oversteer vehicles — and calculates characteristic/critical speeds. Transient response analysis via step-steer simulation shows how yaw rate converges to steady state, making it possible to assess handling stability without instrumented track testing.

---

## Validation Against Published Vehicle Specifications

The longitudinal model was tested against published 0–100 km/h acceleration and 100–0 km/h braking data for three production vehicles. All 6 benchmarks pass within the specified tolerances (±15% for acceleration, ±20% for braking).

| Vehicle | Metric | Model Value | Benchmark | Error | Verdict |
|--------:|--------|:----------:|:---------:|:-----:|:-------:|
| Toyota Camry 2.0L | 0–100 km/h | 10.6 s | 9.5 s | +11.6% | PASS |
| Honda Civic 1.5T | 0–100 km/h | 7.1 s | 8.0 s | −11.3% | PASS |
| VW Tiguan 2.0T | 0–100 km/h | 9.5 s | 9.0 s | +5.6% | PASS |
| Toyota Camry 2.0L | 100–0 km/h | 43.7 m | 39.0 m | +12.1% | PASS |
| Honda Civic 1.5T | 100–0 km/h | 43.7 m | 37.0 m | +18.1% | PASS |
| VW Tiguan 2.0T | 100–0 km/h | 43.7 m | 39.0 m | +12.1% | PASS |

The braking model uses a simplified kinematics formula (`v²/(2μg)`, μ=0.90 for dry asphalt with ABS) that does not account for weight transfer, brake fade, or tire nonlinearity, so braking distance is systematically slightly longer than published figures. Acceleration uses a normalized wide-open-throttle torque curve with fixed shift points (92% of redline); real-world launch control, turbo lag, and traction variations account for the remaining error.

Validation code and the full report are in [`src/vehicle_dynamics_toolkit/validation.py`](src/vehicle_dynamics_toolkit/validation.py).

---

## Quick Start

```bash
# Clone and install
git clone https://github.com/Young-skyyy/vehicle-dynamics-toolkit.git
cd vehicle-dynamics-toolkit
pip install -e ".[test]"

# Run the main demo (longitudinal + lateral dynamics)
python -m vehicle_dynamics_toolkit

# CAN bus simulation with DBC export and UDS diagnostics
python -m vehicle_dynamics_toolkit.can_demo

# Generate validation report
python -c "from vehicle_dynamics_toolkit.validation import print_validation_report; print_validation_report()"
```

### ROS2 (requires Ubuntu 22.04 + ROS2 Humble)

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Vehicle dynamics simulation (100 Hz)
ros2 launch vehicle_dynamics_node vehicle_sim.launch.py

# UDS diagnostic server (5 ECUs)
ros2 run uds_server uds_server_node

# UDS automated test client (separate terminal)
python3 src/uds_server/scripts/uds_test_client.py EMS
```

---

## Tests

**243 pytest cases** across 5 test modules, covering:

- **Vehicle dynamics** — engine torque, wheel force, acceleration (0–100 km/h), braking distance, resistance (SAE J2263 dynamic rolling), power breakdown by source, understeer gradient, characteristic/critical speed, steady-state cornering, step-steer transient response, Pacejka tire model (longitudinal and combined slip)
- **CAN bus** — signal encode/decode (Motorola + Intel byte order), frame build/parse, multi-ECU simulation, DBC file generation, bus load calculation, overflow detection, edge cases
- **UDS diagnostics** — DTC status byte, session state machine, ReadDataByIdentifier, ReadDTCInformation, ECUReset, SecurityAccess seed/key, negative response codes (NRC), service permission enforcement, ISO-TP multi-frame VIN read
- **Real-world benchmarks** — validation against Camry, Civic, Tiguan published data

CI runs on GitHub Actions with a Python version matrix (3.10, 3.11, 3.12) plus `mypy` static type checking.

```bash
# Run all tests
python -m pytest tests/ -v

# Run a specific test module
python -m pytest tests/test_uds.py -v
```

The ROS2 C++ build is also verified in CI — a separate job on `ubuntu-22.04` compiles `vehicle_dynamics_node` and `uds_server` with `colcon build`.

---

## Project Structure

```
├── src/vehicle_dynamics_toolkit/
│   ├── __init__.py                # Public API surface
│   ├── __main__.py                # CLI entry: python -m vehicle_dynamics_toolkit
│   ├── vehicle.py                 # Vehicle physics, powertrain, longitudinal dynamics, IDM
│   ├── lateral_dynamics.py        # 2-DOF bicycle model, understeer, Pacejka Magic Formula
│   ├── can_demo.py                # CAN bus multi-ECU simulation, DBC export
│   ├── can_bus_load_demo.py       # Bus load analysis, overflow detection
│   ├── uds.py                     # UDS (ISO 14229) diagnostic stack (Python reference)
│   ├── iso_tp.py                   # ISO 15765-2 multi-frame transport protocol
│   ├── validation.py              # Real-vehicle benchmark validation + report
│   ├── plot_dashboard.py          # Multi-panel dashboard
│   ├── plotting.py                # Visualization utilities
│   └── _plot_utils.py             # matplotlib helpers
│
├── tests/
│   ├── test_vehicle_dynamics.py   # 109 tests — longitudinal + lateral + benchmarks
│   ├── test_can_demo.py           # 42 tests — CAN encode/decode, DBC, ECU simulation
│   ├── test_uds.py                # 31 tests — UDS session, SecurityAccess, DTC
│   ├── test_iso_tp.py              # 32 tests — ISO-TP frames, receiver, UDS VIN
│   └── test_can_bus_load.py       # 28 tests — bus load, baud rate, edge cases
│
├── ros2_ws/                       # ROS2 workspace (C++ + Python)
│   └── src/
│       ├── vehicle_msgs/              # Custom ROS2 messages + services (IDL)
│       │   ├── msg/VehicleState.msg
│       │   ├── msg/VehicleControl.msg
│       │   └── srv/UdsRequest.srv
│       ├── vehicle_dynamics_node/     # C++ dynamics node (rclcpp, 100 Hz, jitter monitor)
│       │   ├── src/vehicle_dynamics_node.cpp
│       │   ├── scripts/control_pub.py
│       │   └── launch/vehicle_sim.launch.py
│       └── uds_server/                # C++ UDS diagnostic server (5 ECUs)
│           ├── src/uds_server_node.cpp
│           └── scripts/uds_test_client.py
│
├── scripts/
│   ├── compare_py_cpp.py          # Cross-layer validation (Python ref vs. C++ output)
│   └── compare_integrators.py     # Integrator method comparison
│
├── pyproject.toml
├── .github/workflows/test.yml     # CI: pytest matrix + mypy + ROS2 build
└── .pre-commit-config.yaml
```

---

## ROS2 Real-Time Simulation

The C++ `vehicle_dynamics_node` runs a 100 Hz closed-loop simulation using `rclcpp` timers. It publishes `VehicleState` (14 fields: position, velocity, acceleration, yaw rate, gear, engine RPM, slip angles, etc.) and subscribes to `VehicleControl` (throttle, brake, steering angle). A built-in jitter monitor tracks cycle-to-cycle timing and reports P50/P95/P99 latencies, so real-time deviation can be measured directly from log output.

The `uds_server` node exposes ISO 14229 services (0x10, 0x11, 0x22, 0x27, 0x19, 0x3E) for 5 simulated ECUs via ROS2 service calls, with S3 session timeout enforcement. Every diagnostic response can be validated against the Python reference to confirm algorithm fidelity.

---

## Key Technical Details

- **SAE J2263 coastdown model**: dynamic rolling resistance coefficient μ(v) = f₀ + f₁v + f₄v⁴, with temperature-adjusted parameter ranges from coastdown test data
- **2-DOF bicycle model**: slip angles → lateral forces → yaw moment → Euler integration for yaw rate and lateral acceleration transient response
- **Pacejka Magic Formula**: pure lateral slip formulation with tunable stiffness (B), shape (C), peak (D), and curvature (E) parameters
- **CAN frame encoding**: 11-bit arbitration ID, up to 8-byte payload, Motorola (big-endian) and Intel (little-endian) byte order with bit-position-aware packing
- **UDS session state machine**: enforces ISO 14229-1 service permissions — DefaultSession (0x01), ExtendedDiagnosticSession (0x03), ProgrammingSession (0x02) — with automatic fallback on S3 timeout
- **SecurityAccess (0x27)**: challenge-response via seed generation + key computation; services behind security gate are rejected with NRC 0x33 until unlocked
- **ISO 15765-2 (ISO-TP)**: multi-frame transport — SingleFrame (<8 bytes), FirstFrame + ConsecutiveFrame with FlowControl handshake for payloads up to 4095 bytes; sequence number validation with Overflow on mismatch

---

## License

MIT
