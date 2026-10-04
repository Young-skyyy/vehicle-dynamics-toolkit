"""Diagnostic tester boundary for black-box virtual ECU scenarios."""

from __future__ import annotations

from .ecu import CoreECU


class DiagnosticTester:
    """Send raw UDS payloads to a virtual ECU and retain the exchange."""

    def __init__(self, ecu: CoreECU) -> None:
        self.ecu = ecu
        self.exchanges: list[tuple[bytes, bytes]] = []

    def request(self, payload: bytes) -> bytes:
        """Send one UDS request and return the ECU response."""
        response = self.ecu.handle_request(payload)
        self.exchanges.append((payload, response))
        return response

    def last_exchange(self) -> tuple[bytes, bytes] | None:
        """Return the latest request/response pair, if one exists."""
        return self.exchanges[-1] if self.exchanges else None
