"""Deterministic timestamp-ordered CAN event delivery (no arbitration model)."""
from __future__ import annotations
from dataclasses import dataclass
import heapq
import itertools
import math
from typing import Callable
from .timing import SimulationClock

@dataclass(frozen=True)
class CANFrame:
    can_id: int
    data: bytes
    timestamp_s: float = 0.0
    is_error_frame: bool = False
    error_code: str | None = None

CANReceiver = Callable[[CANFrame], None]
FrameFilter = Callable[[CANFrame], CANFrame | None]

class CANBus:
    """Queue callbacks instead of recursive delivery; FIFO at equal timestamps.

    send() drains queued events for compatibility. schedule()/run_until() allow
    scenarios to enqueue events before advancing. A filter can drop/corrupt data
    or replace it with an error frame, including diagnostic traffic.
    """
    def __init__(self, clock: SimulationClock | None = None,
                 frame_filter: FrameFilter | None = None) -> None:
        self.clock = clock if clock is not None else SimulationClock()
        self.frame_filter = frame_filter
        self._receivers: dict[int, list[CANReceiver]] = {}
        self._error_receivers: list[CANReceiver] = []
        self._frames: list[CANFrame] = []
        self._queue: list[tuple[float, int, CANFrame]] = []
        self._order = itertools.count()
        self._dispatching = False

    def subscribe(self, can_id: int, receiver: CANReceiver) -> None:
        self._receivers.setdefault(can_id, []).append(receiver)

    def subscribe_errors(self, receiver: CANReceiver) -> None:
        self._error_receivers.append(receiver)

    def schedule(self, frame: CANFrame) -> None:
        if not math.isfinite(frame.timestamp_s) or frame.timestamp_s < self.clock.now - 1e-12:
            raise ValueError("CAN frame time cannot precede the simulation clock")
        heapq.heappush(self._queue, (frame.timestamp_s, next(self._order), frame))

    def send(self, frame: CANFrame) -> None:
        self.schedule(frame)
        if not self._dispatching:
            self.run_until()

    def run_until(self, timestamp_s: float | None = None) -> None:
        if self._dispatching:
            return
        if timestamp_s is not None and (not math.isfinite(timestamp_s) or timestamp_s < self.clock.now):
            raise ValueError("run_until requires a finite future time")
        self._dispatching = True
        try:
            while self._queue and (timestamp_s is None or self._queue[0][0] <= timestamp_s):
                _, _, frame = heapq.heappop(self._queue)
                self.clock.advance_to(frame.timestamp_s)
                if self.frame_filter is not None and not frame.is_error_frame:
                    filtered = self.frame_filter(frame)
                    if filtered is None:
                        continue
                    if filtered.timestamp_s != frame.timestamp_s:
                        raise ValueError("a frame filter cannot change event time")
                    frame = filtered
                self._frames.append(frame)
                receivers = (self._error_receivers if frame.is_error_frame
                             else self._receivers.get(frame.can_id, []))
                for receiver in tuple(receivers):
                    receiver(frame)
            if timestamp_s is not None:
                self.clock.advance_to(timestamp_s)
        finally:
            self._dispatching = False

    def inject_error(self, error_code: str, timestamp_s: float | None = None,
                     can_id: int = 0) -> None:
        self.send(CANFrame(can_id, b"", self.clock.now if timestamp_s is None else timestamp_s,
                           True, error_code))

    def history(self) -> tuple[CANFrame, ...]:
        return tuple(self._frames)

    def clear_history(self) -> None:
        self._frames.clear()
