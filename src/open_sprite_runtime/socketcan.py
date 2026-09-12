"""Offline, fail-closed validation of SocketCAN receive-only state."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import socket
import struct
import time
from typing import Any, Iterable


SO_TIMESTAMPING_LINUX_64 = 37
SOF_TIMESTAMPING_RX_HARDWARE = 1 << 2
SOF_TIMESTAMPING_RX_SOFTWARE = 1 << 3
SOF_TIMESTAMPING_SOFTWARE = 1 << 4
SOF_TIMESTAMPING_RAW_HARDWARE = 1 << 6
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_ID_MASK = 0x1FFFFFFF
CANFD_FRAME = struct.Struct("=IBBBB64s")
CAN_FRAME = struct.Struct("=IB3x8s")


@dataclass(frozen=True)
class SocketCanRxPreflightReport:
    expected_interfaces: tuple[str, ...]
    observed_interfaces: tuple[str, ...]
    listen_only_interfaces: tuple[str, ...]
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.errors

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "passed": self.passed, "hardware_tx_attempts": 0}


@dataclass(frozen=True)
class ReceivedCanFrame:
    interface: str
    can_id: int
    data: bytes
    is_extended: bool
    is_remote: bool
    is_error: bool
    is_fd: bool
    bit_rate_switch: bool
    error_state_indicator: bool
    software_timestamp_ns: int
    hardware_timestamp_ns: int
    userspace_receive_timestamp_ns: int

    @property
    def userspace_queue_age_ns(self) -> int:
        return self.userspace_receive_timestamp_ns - self.software_timestamp_ns


def _ctrlmodes(entry: dict[str, Any]) -> set[str]:
    linkinfo = entry.get("linkinfo")
    info_data = linkinfo.get("info_data") if isinstance(linkinfo, dict) else None
    raw = info_data.get("ctrlmode") if isinstance(info_data, dict) else None
    if isinstance(raw, str):
        return {raw.upper()}
    if isinstance(raw, list):
        return {str(value).upper() for value in raw}
    if isinstance(raw, dict):
        return {str(key).upper() for key, value in raw.items() if value is True}
    return set()


def audit_socketcan_rx_snapshot(
    snapshot: Any, expected_interfaces: Iterable[str]
) -> SocketCanRxPreflightReport:
    """Audit structured output from ``ip -j -d link show`` without opening CAN."""
    expected = tuple(expected_interfaces)
    errors: list[str] = []
    if len(expected) != 4 or len(set(expected)) != 4:
        errors.append("exactly four unique expected interfaces are required")
    if not isinstance(snapshot, list):
        snapshot = []
        errors.append("SocketCAN snapshot must be a JSON array")

    entries = {
        entry.get("ifname"): entry
        for entry in snapshot
        if isinstance(entry, dict) and isinstance(entry.get("ifname"), str)
    }
    listen_only: list[str] = []
    for name in expected:
        entry = entries.get(name)
        if entry is None:
            errors.append(f"{name}: interface is missing")
            continue
        linkinfo = entry.get("linkinfo")
        info_kind = linkinfo.get("info_kind") if isinstance(linkinfo, dict) else None
        if entry.get("link_type") != "can" and info_kind != "can":
            errors.append(f"{name}: interface is not CAN")
        flags = entry.get("flags")
        if not isinstance(flags, list) or "UP" not in {str(flag).upper() for flag in flags}:
            errors.append(f"{name}: interface is not UP")
        if "LISTEN-ONLY" not in _ctrlmodes(entry):
            errors.append(f"{name}: kernel LISTEN-ONLY is not enabled")
        else:
            listen_only.append(name)

    return SocketCanRxPreflightReport(
        expected_interfaces=expected,
        observed_interfaces=tuple(sorted(entries)),
        listen_only_interfaces=tuple(listen_only),
        errors=tuple(errors),
    )


def _socket_timestamps_ns(
    ancillary: Iterable[tuple[int, int, bytes]],
) -> tuple[int, int]:
    for level, kind, payload in ancillary:
        if level != socket.SOL_SOCKET or kind != SO_TIMESTAMPING_LINUX_64:
            continue
        if len(payload) < 6 * 8:
            raise RuntimeError("SCM_TIMESTAMPING payload is shorter than three timespec values")
        values = struct.unpack_from("=6q", payload)
        software_ns = values[0] * 1_000_000_000 + values[1]
        raw_hardware_ns = values[4] * 1_000_000_000 + values[5]
        if software_ns <= 0:
            raise RuntimeError("kernel software RX timestamp is missing")
        if raw_hardware_ns <= 0:
            raise RuntimeError(
                "raw hardware RX timestamp is missing; software fallback is forbidden"
            )
        return software_ns, raw_hardware_ns
    raise RuntimeError("SCM_TIMESTAMPING evidence is missing")


class SocketCanReceiver:
    """Receive-only SocketCAN wrapper with no transmit API."""

    def __init__(self, interface: str, raw_socket: Any, *, clock_ns: Any = time.time_ns):
        self.interface = interface
        self._socket = raw_socket
        self._clock_ns = clock_ns

    @classmethod
    def open(
        cls,
        interface: str,
        preflight: SocketCanRxPreflightReport,
        *,
        socket_factory: Any = socket.socket,
    ) -> "SocketCanReceiver":
        if not preflight.passed:
            raise RuntimeError("SocketCAN RX preflight did not pass")
        if interface not in preflight.listen_only_interfaces:
            raise RuntimeError(f"{interface}: no listen-only preflight evidence")
        raw_socket = socket_factory(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            raw_socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            timestamp_flags = (
                SOF_TIMESTAMPING_RX_HARDWARE
                | SOF_TIMESTAMPING_RX_SOFTWARE
                | SOF_TIMESTAMPING_SOFTWARE
                | SOF_TIMESTAMPING_RAW_HARDWARE
            )
            raw_socket.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPING_LINUX_64, timestamp_flags)
            raw_socket.bind((interface,))
        except BaseException:
            raw_socket.close()
            raise
        return cls(interface, raw_socket)

    def receive(self) -> ReceivedCanFrame:
        payload, ancillary, _flags, _address = self._socket.recvmsg(
            CANFD_FRAME.size, socket.CMSG_SPACE(6 * 8)
        )
        userspace_receive_ns = int(self._clock_ns())
        software_timestamp_ns, hardware_timestamp_ns = _socket_timestamps_ns(ancillary)
        if userspace_receive_ns < software_timestamp_ns:
            raise RuntimeError("kernel RX timestamp is in the future relative to userspace")
        if len(payload) == CANFD_FRAME.size:
            raw_id, length, flags, _reserved0, _reserved1, data = CANFD_FRAME.unpack(payload)
            is_fd = True
            bit_rate_switch = bool(flags & 0x01)
            error_state_indicator = bool(flags & 0x02)
        elif len(payload) == CAN_FRAME.size:
            raw_id, length, data = CAN_FRAME.unpack(payload)
            is_fd = False
            bit_rate_switch = False
            error_state_indicator = False
        else:
            raise RuntimeError(f"unexpected SocketCAN frame size: {len(payload)}")
        if length > len(data):
            raise RuntimeError(f"invalid CAN payload length: {length}")
        return ReceivedCanFrame(
            interface=self.interface,
            can_id=raw_id & CAN_ID_MASK,
            data=data[:length],
            is_extended=bool(raw_id & CAN_EFF_FLAG),
            is_remote=bool(raw_id & CAN_RTR_FLAG),
            is_error=bool(raw_id & CAN_ERR_FLAG),
            is_fd=is_fd,
            bit_rate_switch=bit_rate_switch,
            error_state_indicator=error_state_indicator,
            software_timestamp_ns=software_timestamp_ns,
            hardware_timestamp_ns=hardware_timestamp_ns,
            userspace_receive_timestamp_ns=userspace_receive_ns,
        )

    def close(self) -> None:
        self._socket.close()

    def fileno(self) -> int:
        return self._socket.fileno()

    def __enter__(self) -> "SocketCanReceiver":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()
