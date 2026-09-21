"""Capability-limited decoding for Damiao read-only register transactions."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
import time
from typing import Any, Iterable, Mapping

from .damiao import DamiaoFeedbackEndpoint
from .socketcan import ReceivedCanFrame, SocketCanTimestampError


DAMIAO_PARAMETER_CAN_ID = 0x7FF
READ_OPCODE = 0x33
MIT_RANGE_REGISTERS = {21: "PMAX", 22: "VMAX", 23: "TMAX"}


def build_read_request(command_can_id: int, register_id: int) -> bytes:
    if not isinstance(command_can_id, int) or not 0 <= command_can_id <= 0x7FF:
        raise ValueError("Damiao command CAN ID must be an 11-bit standard ID")
    if register_id not in MIT_RANGE_REGISTERS:
        raise ValueError("only PMAX/VMAX/TMAX registers 21/22/23 may be read")
    return bytes(
        (
            command_can_id & 0xFF,
            (command_can_id >> 8) & 0xFF,
            READ_OPCODE,
            register_id,
            0,
            0,
            0,
            0,
        )
    )


@dataclass(frozen=True)
class DamiaoRegisterValue:
    master_id: int
    register_id: int
    register_name: str
    value: float


def decode_read_response(
    frame: ReceivedCanFrame,
    *,
    expected_master_id: int,
    expected_register_id: int,
) -> DamiaoRegisterValue:
    if frame.can_id != expected_master_id:
        raise ValueError("register response arbitration ID does not match Master ID")
    if not frame.is_fd or not frame.bit_rate_switch:
        raise ValueError("register response must be CAN-FD with bit-rate switching")
    if len(frame.data) != 8:
        raise ValueError("register response must contain exactly eight bytes")
    payload_master_id = frame.data[0] | (frame.data[1] << 8)
    if payload_master_id != expected_master_id:
        raise ValueError("register response payload Master ID mismatch")
    if frame.data[2] != READ_OPCODE:
        raise ValueError("register response is not a read response")
    if frame.data[3] != expected_register_id:
        raise ValueError("register response RID mismatch")
    if expected_register_id not in MIT_RANGE_REGISTERS:
        raise ValueError("unexpected register ID")
    value = struct.unpack("<f", frame.data[4:8])[0]
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("register value must be finite and positive")
    return DamiaoRegisterValue(
        master_id=expected_master_id,
        register_id=expected_register_id,
        register_name=MIT_RANGE_REGISTERS[expected_register_id],
        value=float(value),
    )


def collect_mit_range_registers(
    readers: Mapping[str, Any],
    endpoints: Iterable[DamiaoFeedbackEndpoint],
    *,
    timeout_s: float = 0.1,
    retries: int = 3,
    monotonic: Any = time.monotonic,
    sleep: Any = time.sleep,
) -> dict:
    """Read only PMAX/VMAX/TMAX from each endpoint, one transaction at a time."""
    if not math.isfinite(timeout_s) or not 0.01 <= timeout_s <= 1.0:
        raise ValueError("timeout_s must be finite and in [0.01, 1.0]")
    if not isinstance(retries, int) or not 1 <= retries <= 10:
        raise ValueError("retries must be an integer in [1, 10]")
    selected = tuple(endpoints)
    if not selected:
        raise ValueError("at least one endpoint is required")
    if len({endpoint.motor_name for endpoint in selected}) != len(selected):
        raise ValueError("motor names must be unique")
    if any(endpoint.interface not in readers for endpoint in selected):
        raise ValueError("every endpoint must have a register reader")

    values: dict[str, dict[str, float]] = {}
    attempts = 0
    discarded_timestamp_frames = 0
    for endpoint in selected:
        motor_values: dict[str, float] = {}
        reader = readers[endpoint.interface]
        for register_id, register_name in MIT_RANGE_REGISTERS.items():
            decoded = None
            for _attempt in range(retries):
                reader.send_read_request(endpoint.can_id, register_id)
                attempts += 1
                deadline = monotonic() + timeout_s
                while monotonic() < deadline:
                    try:
                        frame = reader.receive()
                    except BlockingIOError:
                        sleep(0.001)
                        continue
                    except SocketCanTimestampError:
                        discarded_timestamp_frames += 1
                        continue
                    if frame.can_id == DAMIAO_PARAMETER_CAN_ID:
                        continue
                    if frame.can_id != endpoint.master_id:
                        continue
                    decoded = decode_read_response(
                        frame,
                        expected_master_id=endpoint.master_id,
                        expected_register_id=register_id,
                    )
                    break
                if decoded is not None:
                    break
            if decoded is None:
                raise RuntimeError(
                    f"no valid {register_name} response from {endpoint.motor_name} "
                    f"after {retries} attempts"
                )
            motor_values[register_name] = decoded.value
        values[endpoint.motor_name] = motor_values

    tx_count = sum(reader.hardware_tx_attempts for reader in readers.values())
    if tx_count != attempts:
        raise RuntimeError("register reader TX accounting mismatch")
    return {
        "mode": "read_only_damiao_mit_range_register_audit",
        "allowed_register_ids": sorted(MIT_RANGE_REGISTERS),
        "write_register_attempts": 0,
        "enable_attempts": 0,
        "mode_switch_attempts": 0,
        "tx_count": tx_count,
        "discarded_timestamp_frames": discarded_timestamp_frames,
        "motors": values,
        "passed": len(values) == len(selected),
    }
