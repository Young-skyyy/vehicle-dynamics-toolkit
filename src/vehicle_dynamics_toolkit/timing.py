"""Monotonic simulation time in seconds; never sleeps or reads wall time."""
from __future__ import annotations
import math

class SimulationClock:
    def __init__(self, now: float = 0.0):
        self.now = 0.0
        self.advance_to(now)

    def advance_to(self, timestamp_s: float) -> None:
        if not math.isfinite(timestamp_s) or timestamp_s < self.now - 1e-12:
            raise ValueError("simulation time must be finite and cannot move backwards")
        self.now = max(self.now, timestamp_s)

    def advance(self, dt_s: float) -> None:
        if not math.isfinite(dt_s) or dt_s < 0:
            raise ValueError("dt_s must be finite and non-negative")
        self.advance_to(self.now + dt_s)
