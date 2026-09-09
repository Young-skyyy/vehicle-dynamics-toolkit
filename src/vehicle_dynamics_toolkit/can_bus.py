"""Small in-memory CAN bus used by virtual ECU integrations and tests."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class CANFrame:
    """A classic CAN data frame or an observed CAN error frame."""

    can_id: int
    data: bytes
    timestamp_s: float = 0.0
    is_error_frame: bool = False
    error_code: str | None = None


CANReceiver = Callable[[CANFrame], None]


class CANBus:
    """Broadcast CAN bus with ID-based subscriptions."""

    def __init__(self) -> None:
        self._receivers: dict[int, list[CANReceiver]] = {}
        self._error_receivers: list[CANReceiver] = []
        self._frames: list[CANFrame] = []

    def subscribe(self, can_id: int, receiver: CANReceiver) -> None:
        """Register a receiver for one CAN identifier."""
        self._receivers.setdefault(can_id, []).append(receiver)

    def subscribe_errors(self, receiver: CANReceiver) -> None:
        """Register a receiver for observed CAN error frames."""
        self._error_receivers.append(receiver)

    def send(self, frame: CANFrame) -> None:
        """Broadcast a data frame, or publish an error frame separately."""
        self._frames.append(frame)
        if frame.is_error_frame:
            for receiver in self._error_receivers:
                receiver(frame)
            return
        for receiver in self._receivers.get(frame.can_id, []):
            receiver(frame)

    def inject_error(self, error_code: str, timestamp_s: float = 0.0,
                     can_id: int = 0) -> None:
        """Inject a bus error without delivering it as application data."""
        self.send(CANFrame(can_id, b"", timestamp_s, True, error_code))

    def history(self) -> tuple[CANFrame, ...]:
        """Return frames sent on this bus in transmission order."""
        return tuple(self._frames)

    def clear_history(self) -> None:
        self._frames.clear()
