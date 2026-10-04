"""ISO-TP transport ownership is independent of the UDS service endpoint."""
from __future__ import annotations
from collections import deque
from typing import Callable
from .can_bus import CANBus, CANFrame
from .ecu import CoreECU
from .iso_tp import FlowControlFlag, FrameType, IsoTPReceiver, parse_frame, segment_payload

class IsoTPChannel:
    """One duplex transport, shared by the ECU and tester adapters.

    Supports classic CAN, CTS/WAIT/OVERFLOW, block size, millisecond STmin
    (0..127), RX timeouts and bounded FC waits. No CAN FD/extended addressing.
    """
    def __init__(self, bus: CANBus, tx_id: int, rx_id: int,
                 on_payload: Callable[[bytes, float], None], block_size: int = 0,
                 st_min_ms: int = 0, timeout_s: float = 1.0):
        self.bus, self.tx_id = bus, tx_id
        self.on_payload = on_payload
        self.receiver = IsoTPReceiver(block_size, st_min_ms, timeout_s)
        self.timeout_s = timeout_s
        self.pending: deque[bytes] = deque()
        self.last_tx = bus.clock.now
        self.error: str | None = None
        self.wait_count = 0
        bus.subscribe(rx_id, self._receive)

    def reset(self) -> None:
        self.pending.clear()
        self.receiver.reset()
        self.error = None
        self.wait_count = 0

    def _send(self, data: bytes, timestamp_s: float) -> None:
        self.bus.send(CANFrame(self.tx_id, data, timestamp_s))

    def send_payload(self, payload: bytes, timestamp_s: float) -> None:
        self.pending.clear()
        self.error = None
        self.wait_count = 0
        if not payload:  # UDS suppress-positive-response means no CAN frame.
            return
        frames = segment_payload(payload)
        self.pending.extend(frames[1:])
        self.last_tx = timestamp_s
        self._send(frames[0], timestamp_s)

    def _receive(self, frame: CANFrame) -> None:
        try:
            parsed = parse_frame(frame.data)
            if parsed.frame_type == FrameType.FLOW_CONTROL:
                if not self.pending:
                    return
                if frame.timestamp_s - self.last_tx > self.timeout_s:
                    self.error = "flow-control timeout"
                    self.pending.clear()
                    return
                if parsed.fc_flag == FlowControlFlag.WAIT:
                    self.wait_count += 1
                    if self.wait_count > 3:
                        self.pending.clear()
                        self.error = "flow-control WAIT limit exceeded"
                    return
                if parsed.fc_flag != FlowControlFlag.CTS or parsed.st_min_ms > 127:
                    self.error = "flow control rejected or unsupported STmin"
                    self.pending.clear()
                    return
                count = parsed.block_size or len(self.pending)
                for _ in range(min(count, len(self.pending))):
                    data = self.pending.popleft()
                    self.last_tx = max(frame.timestamp_s, self.last_tx) + parsed.st_min_ms / 1000
                    self._send(data, self.last_tx)
                return
            assembled, fc = self.receiver.feed(frame.data, frame.timestamp_s)
            if fc is not None:
                self._send(fc, frame.timestamp_s)
            if assembled is not None:
                self.on_payload(assembled, frame.timestamp_s)
        except ValueError as exc:
            self.reset()
            self.error = str(exc)

class VirtualECUNode:
    def __init__(self, ecu: CoreECU, bus: CANBus, request_id: int = 0x7E0,
                 response_id: int = 0x7E8, request_block_size: int = 0,
                 request_st_min_ms: int = 0, timeout_s: float = 1.0) -> None:
        self.ecu, self.bus = ecu, bus
        self.request_id, self.response_id = request_id, response_id
        ecu.bind_clock(bus.clock)
        self.transport = IsoTPChannel(bus, response_id, request_id, self._request,
                                      request_block_size, request_st_min_ms, timeout_s)

    def _request(self, payload: bytes, timestamp_s: float) -> None:
        self.transport.send_payload(self.ecu.handle_request(payload), timestamp_s)

class CANDiagnosticTester:
    def __init__(self, bus: CANBus, request_id: int = 0x7E0,
                 response_id: int = 0x7E8, response_block_size: int = 0,
                 response_st_min_ms: int = 0, timeout_s: float = 1.0) -> None:
        self.bus = bus
        self.request_id, self.response_id = request_id, response_id
        self._responses: list[bytes] = []
        self._error: CANFrame | None = None
        self.transport = IsoTPChannel(bus, request_id, response_id, self._response,
                                      response_block_size, response_st_min_ms, timeout_s)
        bus.subscribe_errors(self._on_error_frame)

    def _response(self, payload: bytes, timestamp_s: float) -> None:
        self._responses.append(payload)

    def _on_error_frame(self, frame: CANFrame) -> None:
        self._error = frame

    def last_error(self) -> CANFrame | None:
        return self._error

    def request(self, payload: bytes, timestamp_s: float | None = None,
                expect_response: bool = True) -> bytes:
        if not payload:
            raise ValueError("UDS request must not be empty")
        self._responses.clear()
        self._error = None
        self.transport.reset()
        timestamp = self.bus.clock.now if timestamp_s is None else timestamp_s
        self.transport.send_payload(payload, timestamp)
        if self._responses:
            return self._responses[-1]
        if not expect_response and not self.transport.pending and self.transport.error is None:
            return b""
        self.bus.run_until(self.bus.clock.now + self.transport.timeout_s)
        error = self.transport.error or "ECU did not return a UDS response before timeout"
        self.transport.reset()
        raise TimeoutError(error)
