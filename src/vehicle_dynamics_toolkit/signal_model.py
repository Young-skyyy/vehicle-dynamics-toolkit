"""Synthetic ECU signals for protocol tests, not a calibrated vehicle plant.

State units: speed km/h, rpm rev/min, throttle/soc/brake_pressure percent,
coolant_temp degC, gear integer. The legacy brake_pressure name is a brake
command proxy, NOT pressure in Pa or bar. DynamicsModel provides SI physics.
"""
from __future__ import annotations
import math
import random

class SignalModel:
    def __init__(self, seed: int | None = 42):
        self._rng = random.Random(seed)
        self.rpm = 800.0
        self.throttle = 0.0
        self.speed = 0.0
        self.coolant_temp = 25.0
        self.gear = 0
        self.soc = 80.0
        self.brake_pressure = 0.0
        self.accelerating = False

    def update(self, dt_s: float, throttle: float | None = None,
               brake: float | None = None) -> None:
        """Advance a test signal source; throttle/brake are percentages (0..100)."""
        if not math.isfinite(dt_s) or dt_s < 0:
            raise ValueError("dt_s must be non-negative")

        if any(x is not None and (not math.isfinite(x) or not 0 <= x <= 100)
               for x in (throttle, brake)):
            raise ValueError("driver commands must be finite percentages in [0, 100]")
        if dt_s == 0:
            return
        if throttle is None and brake is None:
            if not self.accelerating and self.speed <= 0:
                self.accelerating = True
                self.gear = 1
            if self.speed >= 80:
                self.accelerating = False
            if self.accelerating:
                self.throttle = min(80, self.throttle + self._rng.uniform(0, 10) * dt_s)
                self.rpm += int(500 * dt_s)
                self.speed += 3 * dt_s
            else:
                self.throttle = max(0, self.throttle - self._rng.uniform(5, 15) * dt_s)
                self.rpm -= int(300 * dt_s)
                self.speed = max(0, self.speed - 2 * dt_s)
            self.brake_pressure = self._rng.uniform(0, 5) if not self.accelerating else 0
        else:
            self.throttle = max(0.0, min(100.0, throttle or 0.0))
            brake_input = max(0.0, min(100.0, brake or 0.0))
            self.accelerating = self.throttle > 0 and brake_input == 0
            self.rpm += int((800 * self.throttle / 100 - 500 * brake_input / 100) * dt_s)
            self.speed += (4 * self.throttle / 100 - 6 * brake_input / 100) * dt_s
            self.brake_pressure = brake_input

        self.rpm = max(800, min(6000, self.rpm))
        self.speed = max(0, min(120, self.speed))
        self.coolant_temp = min(95, self.coolant_temp + 0.5 * dt_s)
        self.soc = max(0.0, self.soc - 0.001 * dt_s)

        if self.speed > 60:
            self.gear = 5
        elif self.speed > 40:
            self.gear = 4
        elif self.speed > 25:
            self.gear = 3
        elif self.speed > 10:
            self.gear = 2
        elif self.speed > 0:
            self.gear = 1
        else:
            self.gear = 0

    def snapshot(self) -> dict[str, float | int | bool]:
        """Return a serializable state snapshot for adapters and assertions."""
        return {
            "rpm": self.rpm,
            "throttle": self.throttle,
            "speed": self.speed,
            "coolant_temp": self.coolant_temp,
            "gear": self.gear,
            "soc": self.soc,
            "brake_pressure": self.brake_pressure,
            "accelerating": self.accelerating,
        }
