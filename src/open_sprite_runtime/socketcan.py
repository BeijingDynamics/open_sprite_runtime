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


class SocketCanTimestampError(RuntimeError):
    """A received frame lacked the timestamp evidence required by the runtime."""


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
class SocketCanActiveFdPreflightReport:
    interface: str
    arbitration_bitrate: int
    data_bitrate: int
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.errors

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "passed": self.passed}


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


def audit_socketcan_active_fd_snapshot(
    snapshot: Any,
    interface: str,
    *,
    arbitration_bitrate: int = 1_000_000,
    data_bitrate: int = 5_000_000,
) -> SocketCanActiveFdPreflightReport:
    """Require one active CAN-FD+BRS interface with exact measured bitrates."""
    errors: list[str] = []
    entries = {
        entry.get("ifname"): entry
        for entry in snapshot
        if isinstance(entry, dict) and isinstance(entry.get("ifname"), str)
    } if isinstance(snapshot, list) else {}
    entry = entries.get(interface)
    if entry is None:
        errors.append(f"{interface}: interface is missing")
    else:
        flags = {str(value).upper() for value in entry.get("flags", [])}
        if "UP" not in flags:
            errors.append(f"{interface}: interface is not UP")
        modes = _ctrlmodes(entry)
        if "LISTEN-ONLY" in modes:
            errors.append(f"{interface}: LISTEN-ONLY must be disabled for the active probe")
        if "FD" not in modes:
            errors.append(f"{interface}: CAN FD is not enabled")
        linkinfo = entry.get("linkinfo")
        info = linkinfo.get("info_data") if isinstance(linkinfo, dict) else None
        info = info if isinstance(info, dict) else {}
        nominal = info.get("bittiming")
        data = info.get("data_bittiming")
        observed_nominal = nominal.get("bitrate") if isinstance(nominal, dict) else None
        observed_data = data.get("bitrate") if isinstance(data, dict) else None
        if observed_nominal != arbitration_bitrate:
            errors.append(
                f"{interface}: arbitration bitrate must be {arbitration_bitrate}, "
                f"observed {observed_nominal}"
            )
        if observed_data != data_bitrate:
            errors.append(
                f"{interface}: data bitrate must be {data_bitrate}, observed {observed_data}"
            )
        state = str(info.get("state", "")).upper()
        if state and state != "ERROR-ACTIVE":
            errors.append(f"{interface}: CAN state must be ERROR-ACTIVE, observed {state}")
    return SocketCanActiveFdPreflightReport(
        interface=interface,
        arbitration_bitrate=arbitration_bitrate,
        data_bitrate=data_bitrate,
        errors=tuple(errors),
    )


def _socket_timestamps_ns(
    ancillary: Iterable[tuple[int, int, bytes]],
) -> tuple[int, int]:
    for level, kind, payload in ancillary:
        if level != socket.SOL_SOCKET or kind != SO_TIMESTAMPING_LINUX_64:
            continue
        if len(payload) < 6 * 8:
            raise SocketCanTimestampError(
                "SCM_TIMESTAMPING payload is shorter than three timespec values"
            )
        values = struct.unpack_from("=6q", payload)
        software_ns = values[0] * 1_000_000_000 + values[1]
        raw_hardware_ns = values[4] * 1_000_000_000 + values[5]
        if software_ns <= 0:
            raise SocketCanTimestampError("kernel software RX timestamp is missing")
        if raw_hardware_ns <= 0:
            raise SocketCanTimestampError(
                "raw hardware RX timestamp is missing; software fallback is forbidden"
            )
        return software_ns, raw_hardware_ns
    raise SocketCanTimestampError("SCM_TIMESTAMPING evidence is missing")


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


def _validate_zero_gain_mit_payload(data: bytes) -> None:
    if len(data) != 8:
        raise ValueError("zero-gain MIT poll must contain exactly eight bytes")
    velocity_raw = (data[2] << 4) | (data[3] >> 4)
    kp_raw = ((data[3] & 0x0F) << 8) | data[4]
    kd_raw = (data[5] << 4) | (data[6] >> 4)
    torque_raw = ((data[6] & 0x0F) << 8) | data[7]
    if kp_raw != 0 or kd_raw != 0:
        raise ValueError("zero-gain MIT poll requires raw Kp=Kd=0")
    if velocity_raw not in (2047, 2048) or torque_raw not in (2047, 2048):
        raise ValueError("zero-gain MIT poll requires encoded velocity=torque=0")


class SocketCanZeroGainPoller:
    """CAN-FD+BRS transceiver that can emit only validated zero-gain MIT polls."""

    def __init__(self, interface: str, raw_socket: Any, *, clock_ns: Any = time.time_ns):
        self.interface = interface
        self._socket = raw_socket
        self._receiver = SocketCanReceiver(interface, raw_socket, clock_ns=clock_ns)
        self.hardware_tx_attempts = 0

    @classmethod
    def open(
        cls,
        interface: str,
        preflight: SocketCanActiveFdPreflightReport,
        *,
        socket_factory: Any = socket.socket,
    ) -> "SocketCanZeroGainPoller":
        if not preflight.passed or preflight.interface != interface:
            raise RuntimeError(f"{interface}: active CAN-FD preflight did not pass")
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
            raw_socket.setblocking(False)
        except BaseException:
            raw_socket.close()
            raise
        return cls(interface, raw_socket)

    def send_zero_gain_poll(self, can_id: int, data: bytes) -> None:
        if not isinstance(can_id, int) or not 0 <= can_id <= 0x7FF:
            raise ValueError("zero-gain poll requires an 11-bit standard CAN ID")
        _validate_zero_gain_mit_payload(data)
        frame = CANFD_FRAME.pack(can_id, 8, 0x01, 0, 0, data.ljust(64, b"\0"))
        sent = self._socket.send(frame)
        self.hardware_tx_attempts += 1
        if sent != len(frame):
            raise RuntimeError(f"short CAN-FD write: {sent}/{len(frame)} bytes")

    def receive(self) -> ReceivedCanFrame:
        return self._receiver.receive()

    def fileno(self) -> int:
        return self._socket.fileno()

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> "SocketCanZeroGainPoller":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


class SocketCanDamiaoRegisterReader:
    """CAN-FD+BRS writer restricted to Damiao PMAX/VMAX/TMAX read requests."""

    PARAMETER_CAN_ID = 0x7FF
    READ_OPCODE = 0x33
    ALLOWED_REGISTER_IDS = frozenset((21, 22, 23))

    def __init__(self, interface: str, raw_socket: Any, allowed_motor_can_ids: Iterable[int]):
        self.interface = interface
        self._socket = raw_socket
        self._receiver = SocketCanReceiver(interface, raw_socket)
        self._allowed_motor_can_ids = frozenset(allowed_motor_can_ids)
        if not self._allowed_motor_can_ids:
            raise ValueError("at least one motor CAN ID must be allowed")
        if any(not isinstance(can_id, int) or not 0 <= can_id <= 0x7FF for can_id in self._allowed_motor_can_ids):
            raise ValueError("motor allowlist requires 11-bit standard CAN IDs")
        self.hardware_tx_attempts = 0

    @classmethod
    def open(
        cls,
        interface: str,
        preflight: SocketCanActiveFdPreflightReport,
        allowed_motor_can_ids: Iterable[int],
        *,
        socket_factory: Any = socket.socket,
    ) -> "SocketCanDamiaoRegisterReader":
        if not preflight.passed or preflight.interface != interface:
            raise RuntimeError(f"{interface}: active CAN-FD preflight did not pass")
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
            raw_socket.setblocking(False)
        except BaseException:
            raw_socket.close()
            raise
        return cls(interface, raw_socket, allowed_motor_can_ids)

    def send_read_request(self, motor_can_id: int, register_id: int) -> None:
        if motor_can_id not in self._allowed_motor_can_ids:
            raise ValueError("motor CAN ID is outside the register-read allowlist")
        if register_id not in self.ALLOWED_REGISTER_IDS:
            raise ValueError("only PMAX/VMAX/TMAX registers 21/22/23 may be read")
        data = bytes(
            (
                motor_can_id & 0xFF,
                (motor_can_id >> 8) & 0xFF,
                self.READ_OPCODE,
                register_id,
                0,
                0,
                0,
                0,
            )
        )
        frame = CANFD_FRAME.pack(
            self.PARAMETER_CAN_ID, 8, 0x01, 0, 0, data.ljust(64, b"\0")
        )
        sent = self._socket.send(frame)
        self.hardware_tx_attempts += 1
        if sent != len(frame):
            raise RuntimeError(f"short CAN-FD write: {sent}/{len(frame)} bytes")

    def receive(self) -> ReceivedCanFrame:
        return self._receiver.receive()

    def fileno(self) -> int:
        return self._socket.fileno()

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> "SocketCanDamiaoRegisterReader":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


class SocketCanDamiaoCommissioningRegisterReader(SocketCanDamiaoRegisterReader):
    """CAN-FD+BRS writer restricted to six read-only commissioning registers."""

    ALLOWED_REGISTER_IDS = frozenset((2, 3, 6, 13, 14, 36))

    def send_read_request(self, motor_can_id: int, register_id: int) -> None:
        if motor_can_id not in self._allowed_motor_can_ids:
            raise ValueError("motor CAN ID is outside the register-read allowlist")
        if register_id not in self.ALLOWED_REGISTER_IDS:
            raise ValueError("only OT/OC/MAX_SPD/version commissioning registers may be read")
        data = bytes(
            (
                motor_can_id & 0xFF,
                (motor_can_id >> 8) & 0xFF,
                self.READ_OPCODE,
                register_id,
                0,
                0,
                0,
                0,
            )
        )
        frame = CANFD_FRAME.pack(
            self.PARAMETER_CAN_ID, 8, 0x01, 0, 0, data.ljust(64, b"\0")
        )
        sent = self._socket.send(frame)
        self.hardware_tx_attempts += 1
        if sent != len(frame):
            raise RuntimeError(f"short CAN-FD write: {sent}/{len(frame)} bytes")


class SocketCanSetZeroWriter:
    """Capability-restricted CAN-FD writer for the exact Damiao set-zero frame."""

    SET_ZERO_PAYLOAD = bytes((0xFF,) * 7 + (0xFE,))

    def __init__(self, interface: str, raw_socket: Any, allowed_can_ids: Iterable[int]):
        self.interface = interface
        self._socket = raw_socket
        self._allowed_can_ids = frozenset(allowed_can_ids)
        if not self._allowed_can_ids:
            raise ValueError("at least one set-zero CAN ID must be allowed")
        if any(not isinstance(can_id, int) or not 0 <= can_id <= 0x7FF for can_id in self._allowed_can_ids):
            raise ValueError("set-zero allowlist requires 11-bit standard CAN IDs")
        self.hardware_tx_attempts = 0

    @classmethod
    def open(
        cls,
        interface: str,
        preflight: SocketCanActiveFdPreflightReport,
        allowed_can_ids: Iterable[int],
        *,
        socket_factory: Any = socket.socket,
    ) -> "SocketCanSetZeroWriter":
        if not preflight.passed or preflight.interface != interface:
            raise RuntimeError(f"{interface}: active CAN-FD preflight did not pass")
        raw_socket = socket_factory(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            raw_socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            raw_socket.bind((interface,))
            return cls(interface, raw_socket, allowed_can_ids)
        except BaseException:
            raw_socket.close()
            raise

    def send_set_zero(self, can_id: int) -> None:
        if can_id not in self._allowed_can_ids:
            raise ValueError(f"CAN ID {can_id:#x} is not in the set-zero allowlist")
        frame = CANFD_FRAME.pack(
            can_id,
            8,
            0x01,
            0,
            0,
            self.SET_ZERO_PAYLOAD.ljust(64, b"\0"),
        )
        sent = self._socket.send(frame)
        self.hardware_tx_attempts += 1
        if sent != len(frame):
            raise RuntimeError(f"short CAN-FD write: {sent}/{len(frame)} bytes")

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> "SocketCanSetZeroWriter":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


class SocketCanSingleMotorMitWriter:
    """Writer restricted to one reviewed motor's MIT, enable, and disable frames."""

    ENABLE_PAYLOAD = bytes((0xFF,) * 7 + (0xFC,))
    DISABLE_PAYLOAD = bytes((0xFF,) * 7 + (0xFD,))

    def __init__(self, interface: str, raw_socket: Any, motor_name: str, can_id: int):
        if not motor_name or not isinstance(can_id, int) or not 0 <= can_id <= 0x7FF:
            raise ValueError("single-motor writer requires one named 11-bit CAN endpoint")
        self.interface = interface
        self.motor_name = motor_name
        self.can_id = can_id
        self._socket = raw_socket
        self._receiver = SocketCanReceiver(interface, raw_socket)
        self.command_tx_attempts = 0
        self.enable_attempts = 0
        self.disable_attempts = 0

    @classmethod
    def open(
        cls,
        interface: str,
        preflight: SocketCanActiveFdPreflightReport,
        motor_name: str,
        can_id: int,
        *,
        socket_factory: Any = socket.socket,
    ) -> "SocketCanSingleMotorMitWriter":
        if not preflight.passed or preflight.interface != interface:
            raise RuntimeError(f"{interface}: active CAN-FD preflight did not pass")
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
            raw_socket.setblocking(False)
            return cls(interface, raw_socket, motor_name, can_id)
        except BaseException:
            raw_socket.close()
            raise

    def _send_payload(self, data: bytes) -> None:
        if len(data) != 8:
            raise ValueError("single-motor MIT payload must contain exactly eight bytes")
        frame = CANFD_FRAME.pack(
            self.can_id, 8, 0x01, 0, 0, data.ljust(64, b"\0")
        )
        sent = self._socket.send(frame)
        if sent != len(frame):
            raise RuntimeError(f"short CAN-FD write: {sent}/{len(frame)} bytes")

    def send_command(self, command: Any) -> None:
        if (
            command.motor_name != self.motor_name
            or command.interface != self.interface
            or command.can_id != self.can_id
        ):
            raise ValueError("encoded MIT command is outside the single-motor allowlist")
        self._send_payload(command.data)
        self.command_tx_attempts += 1

    def send_enable(self) -> None:
        self._send_payload(self.ENABLE_PAYLOAD)
        self.enable_attempts += 1

    def send_disable(self) -> None:
        self._send_payload(self.DISABLE_PAYLOAD)
        self.disable_attempts += 1

    def receive(self) -> ReceivedCanFrame:
        return self._receiver.receive()

    def fileno(self) -> int:
        return self._socket.fileno()

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> "SocketCanSingleMotorMitWriter":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


class SocketCanMotorGroupMitWriter:
    """Writer restricted to one fixed 2..8 motor group on one CAN-FD bus."""

    ENABLE_PAYLOAD = SocketCanSingleMotorMitWriter.ENABLE_PAYLOAD
    DISABLE_PAYLOAD = SocketCanSingleMotorMitWriter.DISABLE_PAYLOAD

    def __init__(
        self,
        interface: str,
        raw_socket: Any,
        motor_endpoints: Iterable[tuple[str, int]],
    ):
        endpoints = tuple(motor_endpoints)
        if not 2 <= len(endpoints) <= 8:
            raise ValueError("motor-group writer requires 2..8 endpoints")
        self._can_id_by_name = dict(endpoints)
        if len(self._can_id_by_name) != len(endpoints):
            raise ValueError("motor-group writer endpoint names must be unique")
        if len(set(self._can_id_by_name.values())) != len(endpoints):
            raise ValueError("motor-group writer CAN IDs must be unique")
        if any(
            not name or not isinstance(can_id, int) or not 0 <= can_id <= 0x7FF
            for name, can_id in endpoints
        ):
            raise ValueError("motor-group writer requires named 11-bit CAN endpoints")
        self.interface = interface
        self._socket = raw_socket
        self._receiver = SocketCanReceiver(interface, raw_socket)
        self.command_tx_attempts = {name: 0 for name in self._can_id_by_name}
        self.enable_attempts = {name: 0 for name in self._can_id_by_name}
        self.disable_attempts = {name: 0 for name in self._can_id_by_name}

    @classmethod
    def open(
        cls,
        interface: str,
        preflight: SocketCanActiveFdPreflightReport,
        motor_endpoints: Iterable[tuple[str, int]],
        *,
        socket_factory: Any = socket.socket,
    ) -> "SocketCanMotorGroupMitWriter":
        if not preflight.passed or preflight.interface != interface:
            raise RuntimeError(f"{interface}: active CAN-FD preflight did not pass")
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
            raw_socket.setblocking(False)
            return cls(interface, raw_socket, motor_endpoints)
        except BaseException:
            raw_socket.close()
            raise

    def _send_payload(self, motor_name: str, data: bytes) -> None:
        if motor_name not in self._can_id_by_name:
            raise ValueError("motor is outside the fixed motor-group allowlist")
        if len(data) != 8:
            raise ValueError("motor-group MIT payload must contain exactly eight bytes")
        can_id = self._can_id_by_name[motor_name]
        frame = CANFD_FRAME.pack(can_id, 8, 0x01, 0, 0, data.ljust(64, b"\0"))
        sent = self._socket.send(frame)
        if sent != len(frame):
            raise RuntimeError(f"short CAN-FD write: {sent}/{len(frame)} bytes")

    def send_command(self, command: Any) -> None:
        expected_can_id = self._can_id_by_name.get(command.motor_name)
        if (
            expected_can_id is None
            or command.interface != self.interface
            or command.can_id != expected_can_id
        ):
            raise ValueError("encoded MIT command is outside the motor-group allowlist")
        self._send_payload(command.motor_name, command.data)
        self.command_tx_attempts[command.motor_name] += 1

    def send_enable(self, motor_name: str) -> None:
        self._send_payload(motor_name, self.ENABLE_PAYLOAD)
        self.enable_attempts[motor_name] += 1

    def send_disable(self, motor_name: str) -> None:
        self._send_payload(motor_name, self.DISABLE_PAYLOAD)
        self.disable_attempts[motor_name] += 1

    def receive(self) -> ReceivedCanFrame:
        return self._receiver.receive()

    def fileno(self) -> int:
        return self._socket.fileno()

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> "SocketCanMotorGroupMitWriter":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()
