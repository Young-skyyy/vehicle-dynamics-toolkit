# Module boundaries and simulation contract

## State and units

`DynamicsModel` is the deterministic vehicle plant: `DynamicsState` uses seconds,
metres, m/s, m/s² and radians; engine speed is rpm and torque is N·m. `step(dt,
throttle, brake, steer)` accepts throttle/brake in 0..1 and front-wheel steer in
radians. `reference_vehicle()` supplies the configuration shared with the C++ runner.
The step returns state at the end of the interval.

`SignalModel` is a protocol test fixture, independent of UDS and CAN. Its speed is
km/h, throttle and SOC are percent, coolant temperature is °C, and rpm is rev/min.
The historical `brake_pressure` field is a percent brake-command proxy, **not** bar
or Pa. `update(dt, throttle=..., brake=...)` uses percentages 0..100; omitted commands
select the existing synthetic acceleration/deceleration scenario. Explicit commands
are finite and range-checked. A zero step is a no-op.

`CoreECU` is the compatibility facade around that signal model and diagnostic
endpoint. Existing `VehicleECU`, snapshot and raw-request APIs remain available.
CAN and UDS encode the same CoreECU values; they never run their own vehicle update.
DID identifiers/scales in this project are a teaching profile, not OEM definitions.
`DIDDefinition` declares byte length, signedness, scale, offset, unit and access rules.

These model families are intentionally named separately. CoreECU/FMU outputs must
not be treated as a validated substitute for DynamicsModel outputs. The standalone
C++ UDS server is also a separate demo, not an adapter around Python CoreECU.

## Time ownership

`SimulationClock` is monotonic, measured in seconds, and rejects NaN, infinity and
backward movement. `CoreECU.update(dt)` advances its state and clock once. A
`VirtualECUNode` binds its ECU and UDS session to the bus clock, provided they start
at the same time. Supply `CANBus(clock=ecu.clock)` when attaching an already-stepped
ECU. CAN scheduling advances protocol time; it does not silently integrate the plant.
Scenarios own plant stepping explicitly.

`CANBus.schedule(frame)` enqueues without executing; `run_until(t)` delivers events
through t and advances the clock. `send(frame)` drains the queue for legacy synchronous
callers. Nested responses are queued, avoiding recursive callback stacks. Equal-time
events preserve insertion order. This is a logical event bus: it does not model bitwise
arbitration, bus-off recovery or transmission duration.

UDS S3 expiration is checked before any service, including TesterPresent, and after
CoreECU updates. Expiration also invalidates an outstanding security seed.
Standalone `ECUDiagnosticServer` retains wall-clock behavior unless a SimulationClock
is supplied. ISO-TP uses delivered CAN timestamps, including the legacy pull adapter.

## Protocol and transport

`ECUDiagnosticServer.handle_request` consumes and produces complete UDS payloads.
`IsoTPChannel` owns receive buffers, sequence numbers, response segmentation and
transmit flow-control state. Both VirtualECUNode and CANDiagnosticTester use it.
This supports classic CAN, CTS/WAIT/OVERFLOW, block sizes, receive timeouts and
millisecond STmin values 0..127. Reserved/microsecond STmin encodings are rejected;
CAN FD, extended addressing and full ISO conformance are outside this implementation.
WAIT is bounded to three frames and does not extend the original FC deadline.

The tester is synchronous: a request drains scheduled events and raises TimeoutError
when a response cannot complete. `expect_response=False` permits intentional UDS SPR
silence. It is not an acknowledgement that a silent request arrived. The old
`handle_iso_tp_frame` / `get_next_response_frame` pull API uses LegacyIsoTPAdapter;
it is retained for examples, not for testing flow control.

## Randomness and faults

The model RNG, CAN signal-noise RNG, diagnostic seed RNG and scenario fault RNG are
separate. Reading extra signals or asking for a security seed cannot alter the next
model state. Repeating the same scenario with the same seeds remains deterministic.
Changing observation order may change observation noise, not vehicle state.

Simple and advanced scenarios deliver through CANBus. `frame_filter` can drop a
frame (return None), corrupt it, or replace it with an error frame at the same time.
It applies to diagnostic traffic too. Error subscriptions and history observe what
was delivered; error frames are not fed to UDS. Advanced scenario error_rate controls
periodic traffic; use the bus filter to affect diagnostic frames as well.

## Native FMU and packaging

`native/ecu_fmu.c` ships as package data. `build_fmu` locates it inside the installed
package, not by walking back to the source checkout. Windows/Linux x86-64 are the
supported build targets. Integer gear output is discrete; outputs specify exact
initial values. fmi2GetTypesPlatform returns `default`, as specified in the
[FMI 2.0 headers](https://github.com/modelica/fmi-standard/blob/v2.0.5/headers/fmi2TypesPlatform.h).

The wrapper remains experimental: its signal equations differ from both Python
models and it implements a subset of FMI entry points. The packaging test validates
native load, initialization, stepping and reset via ctypes. It does not certify
compatibility with every FMI importer or full FMI lifecycle conformance.

## Verification

- `python -m pytest tests -q`: existing behavior plus state/time/fault/transport regressions.
- `python -m mypy --ignore-missing-imports src/vehicle_dynamics_toolkit/`: package type checks.
- `python scripts/compare_py_cpp.py --check`: compile and compare 12 scenarios, 18,600 samples.
- Build a wheel, install it outside this checkout, then run
  `python /path/to/repository/scripts/check_fmu_install.py --require-installed --compiler cc`.
  This exercises the installed native source resource rather than an editable checkout.
- CI runs Python tests, mypy, parity, ROS2 builds and wheel/FMU smoke validation
  (the job is named `wheel-fmu`). Local Windows validation does not replace the ROS2 job.
