"""Capability-limited decoding for Damiao read-only register transactions."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import struct
import time
from typing import Any, Iterable, Mapping

from .damiao import DamiaoFeedbackEndpoint
from .socketcan import ReceivedCanFrame, SocketCanTimestampError


DAMIAO_PARAMETER_CAN_ID = 0x7FF
READ_OPCODE = 0x33
MIT_RANGE_REGISTERS = {21: "PMAX", 22: "VMAX", 23: "TMAX"}
COMMISSIONING_REGISTERS = {
    2: ("OT_Value", "float"),
    3: ("OC_Value", "float"),
    6: ("MAX_SPD", "float"),
    13: ("hw_ver", "uint32"),
    14: ("sw_ver", "uint32"),
    36: ("sub_ver", "uint32"),
}


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
    value: float | int


def decode_commissioning_read_response(
    frame: ReceivedCanFrame,
    *,
    expected_command_can_id: int,
    expected_master_id: int,
    expected_register_id: int,
) -> DamiaoRegisterValue:
    """Decode one response from the fixed commissioning-register allowlist."""
    if expected_register_id not in COMMISSIONING_REGISTERS:
        raise ValueError("unexpected commissioning register ID")
    if frame.can_id != expected_master_id:
        raise ValueError("register response arbitration ID does not match Master ID")
    if not frame.is_fd or not frame.bit_rate_switch:
        raise ValueError("register response must be CAN-FD with bit-rate switching")
    if len(frame.data) != 8:
        raise ValueError("register response must contain exactly eight bytes")
    payload_command_id = frame.data[0] | (frame.data[1] << 8)
    if payload_command_id != expected_command_can_id:
        raise ValueError("register response payload command CAN ID mismatch")
    if frame.data[2] != READ_OPCODE:
        raise ValueError("register response is not a read response")
    if frame.data[3] != expected_register_id:
        raise ValueError("register response RID mismatch")
    register_name, value_type = COMMISSIONING_REGISTERS[expected_register_id]
    if value_type == "uint32":
        value: float | int = struct.unpack("<I", frame.data[4:8])[0]
    else:
        value = struct.unpack("<f", frame.data[4:8])[0]
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("register value must be finite and positive")
    return DamiaoRegisterValue(
        master_id=expected_master_id,
        register_id=expected_register_id,
        register_name=register_name,
        value=value,
    )


def collect_commissioning_registers(
    readers: Mapping[str, Any],
    endpoints: Iterable[DamiaoFeedbackEndpoint],
    *,
    timeout_s: float = 0.1,
    retries: int = 3,
    monotonic: Any = time.monotonic,
    sleep: Any = time.sleep,
) -> dict:
    """Read only protection, speed, and version registers from each endpoint."""
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

    values: dict[str, dict[str, float | int]] = {}
    attempts = 0
    discarded_timestamp_frames = 0
    for endpoint in selected:
        motor_values: dict[str, float | int] = {}
        reader = readers[endpoint.interface]
        for register_id, (register_name, _value_type) in COMMISSIONING_REGISTERS.items():
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
                    if frame.can_id == DAMIAO_PARAMETER_CAN_ID or frame.can_id != endpoint.master_id:
                        continue
                    decoded = decode_commissioning_read_response(
                        frame,
                        expected_command_can_id=endpoint.can_id,
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
        "mode": "read_only_damiao_commissioning_register_audit",
        "allowed_register_ids": sorted(COMMISSIONING_REGISTERS),
        "write_register_attempts": 0,
        "enable_attempts": 0,
        "mode_switch_attempts": 0,
        "mit_control_attempts": 0,
        "tx_count": tx_count,
        "discarded_timestamp_frames": discarded_timestamp_frames,
        "motors": values,
        "passed": len(values) == len(selected),
    }


def decode_read_response(
    frame: ReceivedCanFrame,
    *,
    expected_command_can_id: int,
    expected_master_id: int,
    expected_register_id: int,
) -> DamiaoRegisterValue:
    if frame.can_id != expected_master_id:
        raise ValueError("register response arbitration ID does not match Master ID")
    if not frame.is_fd or not frame.bit_rate_switch:
        raise ValueError("register response must be CAN-FD with bit-rate switching")
    if len(frame.data) != 8:
        raise ValueError("register response must contain exactly eight bytes")
    payload_command_id = frame.data[0] | (frame.data[1] << 8)
    if payload_command_id != expected_command_can_id:
        raise ValueError(
            "register response payload command CAN ID mismatch: "
            f"arbitration={frame.can_id:#x} expected_master={expected_master_id:#x} "
            f"expected_command={expected_command_can_id:#x} "
            f"payload_id={payload_command_id:#x} data={frame.data.hex()}"
        )
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
                        expected_command_can_id=endpoint.can_id,
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


def apply_mit_range_readback(hardware: dict, report: dict, evidence_report: str) -> int:
    """Install an exact 31-motor PMAX/VMAX/TMAX readback into the contract."""
    if report.get("mode") != "read_only_damiao_mit_range_register_audit":
        raise ValueError("unexpected MIT range report mode")
    if report.get("passed") is not True:
        raise ValueError("MIT range report did not pass")
    if any(report.get(key) != 0 for key in (
        "write_register_attempts", "enable_attempts", "mode_switch_attempts"
    )):
        raise ValueError("MIT range report contains prohibited TX attempts")
    motors = report.get("motors")
    motor_map = hardware.get("motor_map")
    if not isinstance(motors, dict) or not isinstance(motor_map, dict):
        raise ValueError("report motors and hardware motor_map must be objects")
    if set(motors) != set(motor_map):
        raise ValueError("MIT range report must exactly cover the hardware motor map")
    digest = hashlib.sha256(
        json.dumps(report, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    updated = 0
    for motor_name, record in motor_map.items():
        values = motors[motor_name]
        pmax, vmax, tmax = (float(values[name]) for name in ("PMAX", "VMAX", "TMAX"))
        if not all(math.isfinite(value) and value > 0.0 for value in (pmax, vmax, tmax)):
            raise ValueError(f"invalid MIT range readback for {motor_name}")
        record["mit_ranges"] = {
            "position_rad": [-pmax, pmax],
            "velocity_rad_s": [-vmax, vmax],
            "kp": [0.0, 500.0],
            "kd": [0.0, 5.0],
            "torque_nm": [-tmax, tmax],
            "source": "motor_register_readback",
            "evidence_report": evidence_report,
            "report_content_sha256": digest,
        }
        updated += 1
    return updated
