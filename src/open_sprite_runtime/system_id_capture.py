"""Receive-only CAN-FD capture for real-robot system identification."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import selectors
import time
from typing import Any, Callable, Mapping

import numpy as np

from .socketcan import ReceivedCanFrame, SocketCanTimestampError


@dataclass
class ActiveCanTraceBuffer:
    interface_names: tuple[str, ...]

    def __post_init__(self) -> None:
        if not self.interface_names or len(set(self.interface_names)) != len(self.interface_names):
            raise ValueError("interface_names must be non-empty and unique")
        self._interface_index = {
            name: index for index, name in enumerate(self.interface_names)
        }
        self.monotonic_receive_ns: list[int] = []
        self.software_timestamp_ns: list[int] = []
        self.hardware_timestamp_ns: list[int] = []
        self.userspace_realtime_ns: list[int] = []
        self.interface_index: list[int] = []
        self.can_id: list[int] = []
        self.payload_length: list[int] = []
        self.flags: list[int] = []
        self.payload: list[bytes] = []

    def append(self, frame: ReceivedCanFrame, monotonic_receive_ns: int) -> None:
        if frame.interface not in self._interface_index:
            raise ValueError(f"unexpected CAN interface: {frame.interface}")
        if monotonic_receive_ns <= 0:
            raise ValueError("monotonic_receive_ns must be positive")
        if len(frame.data) > 64:
            raise ValueError("CAN payload exceeds 64 bytes")
        flags = (
            int(frame.is_extended)
            | (int(frame.is_remote) << 1)
            | (int(frame.is_error) << 2)
            | (int(frame.is_fd) << 3)
            | (int(frame.bit_rate_switch) << 4)
            | (int(frame.error_state_indicator) << 5)
        )
        self.monotonic_receive_ns.append(int(monotonic_receive_ns))
        self.software_timestamp_ns.append(int(frame.software_timestamp_ns))
        self.hardware_timestamp_ns.append(int(frame.hardware_timestamp_ns))
        self.userspace_realtime_ns.append(int(frame.userspace_receive_timestamp_ns))
        self.interface_index.append(self._interface_index[frame.interface])
        self.can_id.append(int(frame.can_id))
        self.payload_length.append(len(frame.data))
        self.flags.append(flags)
        self.payload.append(frame.data.ljust(64, b"\0"))

    def arrays(self) -> dict[str, np.ndarray]:
        payload = (
            np.frombuffer(b"".join(self.payload), dtype=np.uint8).reshape(-1, 64)
            if self.payload
            else np.empty((0, 64), dtype=np.uint8)
        )
        return {
            "interface_names": np.asarray(self.interface_names),
            "monotonic_receive_ns": np.asarray(self.monotonic_receive_ns, dtype=np.int64),
            "software_timestamp_ns": np.asarray(self.software_timestamp_ns, dtype=np.int64),
            "hardware_timestamp_ns": np.asarray(self.hardware_timestamp_ns, dtype=np.int64),
            "userspace_realtime_ns": np.asarray(self.userspace_realtime_ns, dtype=np.int64),
            "interface_index": np.asarray(self.interface_index, dtype=np.uint8),
            "can_id": np.asarray(self.can_id, dtype=np.uint32),
            "payload_length": np.asarray(self.payload_length, dtype=np.uint8),
            "flags": np.asarray(self.flags, dtype=np.uint8),
            "payload": payload,
        }


def capture_active_can_trace(
    receivers: Mapping[str, Any],
    duration_s: float,
    *,
    selector_factory: Callable[[], Any] = selectors.DefaultSelector,
    monotonic: Callable[[], float] = time.monotonic,
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
) -> tuple[ActiveCanTraceBuffer, dict[str, Any]]:
    """Capture all visible frames without owning a socket that can transmit."""
    if not 0.0 < duration_s <= 3600.0:
        raise ValueError("duration_s must be in (0, 3600]")
    names = tuple(receivers)
    if not names or len(set(names)) != len(names):
        raise ValueError("receivers must have unique interface names")
    trace = ActiveCanTraceBuffer(names)
    counts = {name: 0 for name in names}
    timestamp_rejections = {name: 0 for name in names}
    started = monotonic()
    deadline = started + duration_s
    with selector_factory() as selector:
        for name, receiver in receivers.items():
            if hasattr(receiver, "send"):
                raise RuntimeError("capture receiver unexpectedly exposes a send method")
            selector.register(receiver, selectors.EVENT_READ, name)
        while monotonic() < deadline:
            for key, _mask in selector.select(timeout=min(0.05, deadline - monotonic())):
                name = str(key.data)
                receiver = receivers[name]
                while True:
                    try:
                        frame = receiver.receive()
                    except BlockingIOError:
                        break
                    except SocketCanTimestampError:
                        timestamp_rejections[name] += 1
                        continue
                    trace.append(frame, monotonic_ns())
                    counts[name] += 1
    elapsed = monotonic() - started
    errors = [f"{name}: captured no frames" for name, count in counts.items() if count == 0]
    return trace, {
        "mode": "receive_only_active_socketcan_system_id_capture",
        "requested_duration_s": duration_s,
        "elapsed_s": elapsed,
        "frame_count": sum(counts.values()),
        "frame_count_by_interface": counts,
        "timestamp_rejection_count_by_interface": timestamp_rejections,
        "hardware_tx_attempts": 0,
        "errors": errors,
        "passed": not errors,
    }


def write_active_can_trace(
    trace: ActiveCanTraceBuffer,
    report: Mapping[str, Any],
    trace_path: str | Path,
    report_path: str | Path,
) -> None:
    trace_path = Path(trace_path)
    report_path = Path(report_path)
    trace_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(trace_path, **trace.arrays())
    report_path.write_text(json.dumps(dict(report), indent=2) + "\n", encoding="utf-8")
