"""UDS over CAN adapters connecting the tester, CAN bus and virtual ECU."""

from __future__ import annotations

from .can_bus import CANBus, CANFrame
from .ecu import CoreECU
from .iso_tp import (
    FlowControlFlag,
    FrameType,
    IsoTPReceiver,
    parse_frame,
    segment_payload,
)


class VirtualECUNode:
    """Expose one CoreECU diagnostic endpoint on two CAN identifiers."""

    def __init__(self, ecu: CoreECU, bus: CANBus, request_id: int = 0x7E0,
                 response_id: int = 0x7E8) -> None:
        self.ecu = ecu
        self.bus = bus
        self.request_id = request_id
        self.response_id = response_id
        self._response_block_size = 0
        self._response_st_min_ms = 0
        self._response_frames_in_block = 0
        self._next_response_timestamp_s = 0.0
        bus.subscribe(request_id, self._on_request_frame)

    def _send(self, data: bytes, timestamp_s: float) -> None:
        self.bus.send(CANFrame(self.response_id, data, timestamp_s))

    def _on_request_frame(self, frame: CANFrame) -> None:
        parsed = parse_frame(frame.data)
        if parsed.frame_type == FrameType.FLOW_CONTROL:
            if parsed.fc_flag != FlowControlFlag.CTS:
                return
            self._response_block_size = parsed.block_size
            self._response_st_min_ms = parsed.st_min_ms
            self._response_frames_in_block = 0
            next_frame = self.ecu.get_next_response_frame()
            while next_frame is not None:
                self._next_response_timestamp_s = max(
                    frame.timestamp_s,
                    self._next_response_timestamp_s,
                ) + self._response_st_min_ms / 1000
                self._send(next_frame, self._next_response_timestamp_s)
                self._response_frames_in_block += 1
                if (self._response_block_size
                        and self._response_frames_in_block >= self._response_block_size):
                    return
                next_frame = self.ecu.get_next_response_frame()
            return
        response = self.ecu.handle_iso_tp_frame(frame.data)
        if response is not None:
            self._send(response, frame.timestamp_s)


class CANDiagnosticTester:
    """Send UDS payloads through CANBus instead of calling the ECU directly."""

    def __init__(self, bus: CANBus, request_id: int = 0x7E0,
                 response_id: int = 0x7E8, response_block_size: int = 0,
                 response_st_min_ms: int = 0, timeout_s: float = 1.0) -> None:
        self.bus = bus
        self.request_id = request_id
        self.response_id = response_id
        self._receiver = IsoTPReceiver(
            block_size=response_block_size,
            st_min_ms=response_st_min_ms,
            timeout_s=timeout_s,
        )
        self._responses: list[bytes] = []
        self._request_flow_control: bytes | None = None
        self._error: CANFrame | None = None
        bus.subscribe(response_id, self._on_response_frame)
        bus.subscribe_errors(self._on_error_frame)

    def _send(self, data: bytes, timestamp_s: float = 0.0) -> None:
        self.bus.send(CANFrame(self.request_id, data, timestamp_s))

    def _on_error_frame(self, frame: CANFrame) -> None:
        self._error = frame

    def _on_response_frame(self, frame: CANFrame) -> None:
        parsed = parse_frame(frame.data)
        if parsed.frame_type == FrameType.FLOW_CONTROL:
            self._request_flow_control = frame.data
            return
        assembled, flow_control = self._receiver.feed(frame.data, frame.timestamp_s)
        if flow_control is not None:
            self._send(flow_control, frame.timestamp_s)
        if assembled is not None:
            self._responses.append(assembled)

    def last_error(self) -> CANFrame | None:
        """Return the latest observed CAN error frame, if any."""
        return self._error

    def request(self, payload: bytes, timestamp_s: float = 0.0) -> bytes:
        """Send one complete UDS payload and return its reassembled response."""
        self._responses.clear()
        self._receiver.reset()
        self._request_flow_control = None
        self._error = None
        frames = segment_payload(payload)
        self._send(frames[0], timestamp_s)
        if len(frames) > 1:
            if self._request_flow_control is None:
                raise RuntimeError("ECU did not return ISO-TP flow control")
            flow_control = parse_frame(self._request_flow_control)
            if flow_control.fc_flag != FlowControlFlag.CTS:
                raise RuntimeError("ECU did not accept ISO-TP request")
            for frame_data in frames[1:]:
                self._send(frame_data, timestamp_s)
        if not self._responses:
            raise RuntimeError("ECU did not return a UDS response")
        return self._responses[-1]
