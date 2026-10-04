"""Compatibility facade composing a signal model, simulation clock and UDS."""
from __future__ import annotations
import random
from .signal_model import SignalModel
from .timing import SimulationClock
from .uds import ECUDiagnosticServer

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
        return self.diagnostic.handle_iso_tp_frame(can_data, timestamp)

    def get_next_response_frame(self) -> bytes | None:
        return self.diagnostic.get_next_response_frame()
