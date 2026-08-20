# -*- coding: utf-8 -*-
"""Independent virtual ECU core shared by Python, CAN and UDS adapters."""

from __future__ import annotations

import random

from .uds import ECUDiagnosticServer


class CoreECU:
    """可独立运行的虚拟 ECU 核心，统一维护状态和诊断端点。"""

    def __init__(self, ecu_name: str = "EMS", seed: int | None = 42,
                 did_values: dict[int, float] | None = None):
        self.ecu_name = ecu_name
        self._rng = random.Random(seed)
        self.rpm = 800
        self.throttle = 0
        self.speed = 0
        self.coolant_temp = 25
        self.gear = 0
        self.soc = 80.0
        self.brake_pressure = 0
        self.accelerating = False
        self.diagnostic = ECUDiagnosticServer(ecu_name, did_values or {
            0x000C: float(self.rpm),
            0x000D: float(self.speed),
            0x0005: float(self.coolant_temp),
        })

    def update(self, dt_s: float) -> None:
        """推进 ECU 状态，并同步动态 DID。"""
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

        self.rpm = max(800, min(6000, self.rpm))
        self.speed = max(0, min(120, self.speed))
        self.coolant_temp = min(95, self.coolant_temp + 0.5 * dt_s)
        self.soc -= 0.001 * dt_s

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

        self.brake_pressure = self._rng.uniform(0, 5) if not self.accelerating else 0
        self.diagnostic.update_did(0x000C, float(self.rpm))
        self.diagnostic.update_did(0x000D, float(self.speed))
        self.diagnostic.update_did(0x0005, float(self.coolant_temp))

    def snapshot(self) -> dict[str, float | int | bool]:
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

    def update_did(self, did: int, value: float) -> None:
        self.diagnostic.update_did(did, value)

    def handle_request(self, request: bytes) -> bytes:
        return self.diagnostic.handle_request(request)

    def handle_iso_tp_frame(self, can_data: bytes) -> bytes | None:
        return self.diagnostic.handle_iso_tp_frame(can_data)

    def get_next_response_frame(self) -> bytes | None:
        return self.diagnostic.get_next_response_frame()
