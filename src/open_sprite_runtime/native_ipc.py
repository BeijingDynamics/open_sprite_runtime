"""Versioned binary IPC between the 50 Hz actor and native state/safety loop."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Iterable


MAGIC = b"SPRT"
VERSION = 1
STATE_KIND = 1
TARGET_KIND = 2
JOINT_COUNT = 31

_STATE = struct.Struct("<4sHHQqQ62d")
_TARGET = struct.Struct("<4sHHQqQQ155d")


def ordered_name_hash(names: Iterable[str]) -> int:
    """Return the cross-language FNV-1a hash of an exact ordered name list."""
    result = 0xCBF29CE484222325
    selected = tuple(names)
    if len(selected) != JOINT_COUNT or len(set(selected)) != JOINT_COUNT:
        raise ValueError("ordered IPC name list must contain 31 unique names")
    for name in selected:
        if not name or "\0" in name:
            raise ValueError("IPC names must be non-empty and contain no NUL")
        for byte in name.encode("utf-8") + b"\0":
            result ^= byte
            result = (result * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return result


def _vector(values: Iterable[float], label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != JOINT_COUNT or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain 31 finite values")
    return result


@dataclass(frozen=True)
class NativeStatePacket:
    sequence: int
    monotonic_ns: int
    motor_order_hash: int
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]

    def pack(self) -> bytes:
        position = _vector(self.position_rad, "position_rad")
        velocity = _vector(self.velocity_rad_s, "velocity_rad_s")
        return _STATE.pack(
            MAGIC,
            VERSION,
            STATE_KIND,
            int(self.sequence),
            int(self.monotonic_ns),
            int(self.motor_order_hash),
            *position,
            *velocity,
        )

    @classmethod
    def unpack(cls, payload: bytes) -> "NativeStatePacket":
        if len(payload) != _STATE.size:
            raise ValueError(f"native state packet must be {_STATE.size} bytes")
        magic, version, kind, sequence, timestamp, name_hash, *values = _STATE.unpack(payload)
        if (magic, version, kind) != (MAGIC, VERSION, STATE_KIND):
            raise ValueError("native state packet header mismatch")
        position = _vector(values[:JOINT_COUNT], "position_rad")
        velocity = _vector(values[JOINT_COUNT:], "velocity_rad_s")
        return cls(
            sequence,
            timestamp,
            name_hash,
            position,
            velocity,
        )


@dataclass(frozen=True)
class PolicyTargetPacket:
    sequence: int
    monotonic_ns: int
    joint_order_hash: int
    source_state_sequence: int
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    feedforward_torque_nm: tuple[float, ...]

    def pack(self) -> bytes:
        vectors = (
            _vector(self.position_rad, "position_rad"),
            _vector(self.velocity_rad_s, "velocity_rad_s"),
            _vector(self.kp, "kp"),
            _vector(self.kd, "kd"),
            _vector(self.feedforward_torque_nm, "feedforward_torque_nm"),
        )
        if any(value < 0.0 for value in vectors[2]) or any(value < 0.0 for value in vectors[3]):
            raise ValueError("policy target Kp/Kd must be non-negative")
        return _TARGET.pack(
            MAGIC,
            VERSION,
            TARGET_KIND,
            int(self.sequence),
            int(self.monotonic_ns),
            int(self.joint_order_hash),
            int(self.source_state_sequence),
            *(value for vector in vectors for value in vector),
        )

    @classmethod
    def unpack(cls, payload: bytes) -> "PolicyTargetPacket":
        if len(payload) != _TARGET.size:
            raise ValueError(f"policy target packet must be {_TARGET.size} bytes")
        magic, version, kind, sequence, timestamp, name_hash, source_sequence, *values = (
            _TARGET.unpack(payload)
        )
        if (magic, version, kind) != (MAGIC, VERSION, TARGET_KIND):
            raise ValueError("policy target packet header mismatch")
        labels = ("position_rad", "velocity_rad_s", "kp", "kd", "feedforward_torque_nm")
        vectors = tuple(
            _vector(values[index * JOINT_COUNT : (index + 1) * JOINT_COUNT], labels[index])
            for index in range(5)
        )
        if any(value < 0.0 for value in vectors[2]) or any(value < 0.0 for value in vectors[3]):
            raise ValueError("policy target Kp/Kd must be non-negative")
        return cls(sequence, timestamp, name_hash, source_sequence, *vectors)


STATE_PACKET_SIZE = _STATE.size
TARGET_PACKET_SIZE = _TARGET.size
