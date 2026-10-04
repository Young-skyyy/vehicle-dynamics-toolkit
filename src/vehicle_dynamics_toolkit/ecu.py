"""Compatibility facade composing a signal model, simulation clock and UDS."""
from __future__ import annotations
import random
from .signal_model import SignalModel
from .timing import SimulationClock
from .uds import ECUDiagnosticServer
from .simulation import DynamicsModel, DriverInput

class CoreECU(SignalModel):
    """Synthetic protocol fixture. Use DynamicsModel for vehicle physics.

    Existing signal attributes and percentage controls remain compatible.
    CAN observation and diagnostic randomness cannot change model evolution.
    """
    def __init__(self, ecu_name: str = "EMS", seed: int | None = 42,
                 did_values: dict[int, float] | None = None,
                 clock: SimulationClock | None = None):
        super().__init__(seed)
        self.ecu_name = ecu_name
        self.clock = clock if clock is not None else SimulationClock()
        self.signal_rng = random.Random(None if seed is None else f"{seed}:signals")
        self.diagnostic = ECUDiagnosticServer(
            ecu_name, did_values, clock=self.clock,
            rng=random.Random(None if seed is None else f"{seed}:diagnostics"))
        self._publish_dids()

    def bind_clock(self, clock: SimulationClock) -> None:
        if abs(clock.now - self.clock.now) > 1e-12:
            raise ValueError("ECU and CAN must start at the same simulation time")
        self.clock = clock
        self.diagnostic.session.clock = clock

    def _publish_dids(self) -> None:
        for did, value in ((0x000C, self.rpm), (0x000D, self.speed),
                           (0x0005, self.coolant_temp)):
            self.diagnostic.update_did(did, float(value))

    def update(self, dt_s: float, throttle: float | None = None,
               brake: float | None = None) -> None:
        super().update(dt_s, throttle, brake)
        self.clock.advance(dt_s)
        self._publish_dids()
        self.diagnostic.expire_session()

    def update_did(self, did: int, value: float) -> None:
        self.diagnostic.update_did(did, value)

    def inject_fault(self, code: str, status: int | None = None) -> None:
        self.diagnostic.set_dtc(code, status)

    def clear_faults(self) -> None:
        self.diagnostic.clear_dtcs()

    def handle_request(self, request: bytes) -> bytes:
        return self.diagnostic.handle_request(request)

    def handle_iso_tp_frame(self, can_data: bytes,
                            timestamp_s: float | None = None) -> bytes | None:
        timestamp = self.clock.now if timestamp_s is None else timestamp_s
        self.clock.advance_to(timestamp)
        self.prepare_observation()
        return self.diagnostic.handle_iso_tp_frame(can_data, timestamp)

    def prepare_observation(self) -> None:
        """Hook for physical adapters before encoding an observation."""

    def get_next_response_frame(self) -> bytes | None:
        return self.diagnostic.get_next_response_frame()


class DynamicsECU(CoreECU):
    """CAN/UDS facade of the physical plant. No synthetic speed/RPM update.

    CAN/UDS speed is km/h; the underlying model state remains SI. Thermal/SOC
    signals are held auxiliary values, since the plant has no thermal/energy model.
    step() accepts normalized controls; legacy update() explicitly converts percent.
    """
    def __init__(self, model: DynamicsModel | None = None, ecu_name: str = "EMS",
                 seed: int | None = 42, did_values: dict[int, float] | None = None,
                 clock: SimulationClock | None = None):
        super().__init__(ecu_name, seed, did_values, clock)
        self.model = model if model is not None else DynamicsModel()
        self._command = DriverInput()
        if self.model.state.t != self.clock.now:
            raise ValueError("physical model and simulation clock must start together")
        self._sync_signals()

    def _sync_signals(self) -> None:
        self.speed = self.model.state.vx * 3.6
        self.rpm = self.model.state.engine_rpm
        self.gear = self.model.state.gear
        self.throttle = self._command.throttle * 100
        self.brake_pressure = self._command.brake * 100
        self.accelerating = self.model.state.ax > 0
        self._publish_dids()

    def _integrate_to(self, timestamp: float) -> None:
        import math
        remaining = timestamp - self.model.state.t
        if remaining < -1e-10:
            raise ValueError("physical time cannot move backwards")
        # Equal substeps avoid a final floating-point sliver altering engine/gear state.
        if remaining > 1e-12:
            count = max(1, math.ceil(remaining / .1 - 1e-12))
            dt = min(.1, remaining / count)
            for _ in range(count):
                self.model.step(dt, self._command.throttle, self._command.brake, self._command.steer)
            self.model.state.t = timestamp
        self._sync_signals()

    def step(self, dt_s: float, command: DriverInput | None = None) -> dict:
        import math
        from .simulation import DriverInput
        if not math.isfinite(dt_s) or not 0 < dt_s <= .1:
            raise ValueError("physical adapter requires 0 < dt_s <= 0.1")
        if command is not None and not isinstance(command, DriverInput):
            raise TypeError("command must be DriverInput")
        self._integrate_to(self.clock.now)  # Hold previous controls across protocol-time gaps.
        if command is not None:
            self._command = command
        target = self.clock.now + dt_s
        self._integrate_to(target)
        self.clock.advance_to(target)
        self.diagnostic.expire_session()
        return self.snapshot()

    def update(self, dt_s: float, throttle: float | None = None,
               brake: float | None = None) -> None:
        from .simulation import DriverInput
        command = (self._command if throttle is None and brake is None else
                   DriverInput((throttle or 0) / 100, (brake or 0) / 100, self._command.steer))
        self.step(dt_s, command)

    def handle_request(self, request: bytes) -> bytes:
        self._integrate_to(self.clock.now)
        return super().handle_request(request)

    def prepare_observation(self) -> None:
        self._integrate_to(self.clock.now)

    def snapshot(self) -> dict:
        from dataclasses import asdict
        self.prepare_observation()
        return {**super().snapshot(), **asdict(self.model.state)}
