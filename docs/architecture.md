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

The physical path is `DriverInput -> DynamicsModel -> DynamicsECU -> CAN/UDS`.
Construct `DynamicsECU(model=DynamicsModel(...))`, attach it with
`VirtualECUNode(ecu, CANBus(clock=ecu.clock))`, and call `ecu.step(dt, DriverInput(...))`.
CAN speed is converted from m/s to km/h only at the adapter boundary; RPM/gear
come from the same plant state. Legacy `update(..., throttle=50)` explicitly
converts percentages to normalized commands. Coolant and SOC stay constant because
there is no thermal/energy plant. Synthetic auxiliary/noise signals stay labeled
as protocol fixtures. The standalone C++ UDS server remains an independent demo.

## Time ownership

`SimulationClock` is monotonic, measured in seconds, and rejects NaN, infinity and
backward movement. `CoreECU.update(dt)` advances its state and clock once. A
`VirtualECUNode` binds its ECU and UDS session to the bus clock, provided they start
at the same time. Supply `CANBus(clock=ecu.clock)` when attaching an already-stepped
ECU. CAN scheduling advances protocol time. DynamicsECU catches up to the clock before
the next step, snapshot, CAN encoding or UDS request, holding its last command
across the gap in equal substeps no larger than 0.1 s. The new command then applies
to the requested interval; communication delay cannot freeze vehicle time.
Reads at the same timestamp do not step the plant. A scenario must use the same
time grid/control timeline when comparing adapters: adding delayed protocol events
can change numerical substep boundaries.

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

`native/ecu_fmu.cpp` and `native/dynamics_core.hpp` ship in the wheel. The C++17
header is canonical for ROS2, the headless runner and FMU; the ROS2 include is a
compatibility wrapper. The FMU builder loads resources from the installed package.
Windows/Linux x86-64 are supported; pass `--compiler c++` or a Zig executable.

The v2 GUID is `{vehicle-dynamics-toolkit-vehicle-plant-v2}`. This is an intentional
interface break: regenerate old v1 FMUs. Throttle/brake inputs (refs 1/2) now use
0..1; steering (8) uses front-wheel radians. Speed (3) remains km/h; SI outputs
(9..18) expose velocity, acceleration, yaw, heading, position, time and torque.
RPM (4), gear (6), coolant (5) and SOC (7) retain their names, but coolant/SOC are
constant auxiliary placeholders. Initial speed (19) is a fixed initialization
parameter. The FMU supports the reference vehicle only; custom Python vehicles
are not automatically exported. The physical step accepts 0 < dt <= 0.1 s.

FMI initialization/termination/reset and time consistency are checked. Invalid
input batches/unknown references return error without partially applying controls.
Units and ranges are declared in XML; state outputs have calculated initial values
because initial speed/start time can differ from zero. The wrapper remains
experimental: no serialized FMU state, derivative interpolation or asynchronous
step support, and no claim of universal importer/FMI conformance.

## Verification

`python -m vehicle_dynamics_toolkit.showcase` is the reproducible demonstration:
a fixed 20-second control timeline, physical ECU observations, reception-gap
detection and UDS fault recovery. `--with-fmu --compiler c++` adds native replay.
Evidence lives in `build/showcase`; it is simulation output, not measured telemetry.

- `python -m pytest tests -q`: behavior, independent baselines and integration regressions.
- `python -m mypy --ignore-missing-imports src/vehicle_dynamics_toolkit/`: type checks.
- `python scripts/validate_physics.py`: independent analytical verification.
- `python scripts/compare_py_cpp.py --check`: 12 cases, 18,600 physical samples.
- Build/install a wheel outside the checkout, then run
  `python /path/to/repository/scripts/check_fmu_install.py --require-installed --compiler c++`.
  Every sample goes through Python, native FMU and DynamicsECU; 198 observations
  exercise actual CAN delivery and UDS-over-ISO-TP speed/RPM reads within quantization.
- CI runs these checks plus ROS2 builds. Local Windows core checks do not replace
  the ROS2 job or a measured real-vehicle validation.
