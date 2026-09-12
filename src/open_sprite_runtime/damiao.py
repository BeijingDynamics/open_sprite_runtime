"""Pure, receive-only decoding for Damiao MIT feedback frames."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Iterable

from .socketcan import ReceivedCanFrame


STATUS_NAMES = {
    0x0: "disabled",
    0x1: "enabled",
    0x8: "over_voltage",
    0x9: "under_voltage",
    0xA: "over_current",
    0xB: "mos_over_temperature",
    0xC: "motor_coil_over_temperature",
    0xD: "communication_lost",
    0xE: "overload",
}


def _validate_range(name: str, value: tuple[float, float]) -> tuple[float, float]:
    if len(value) != 2:
        raise ValueError(f"{name} must contain exactly two values")
    low, high = float(value[0]), float(value[1])
    if not math.isfinite(low) or not math.isfinite(high) or low >= high:
        raise ValueError(f"{name} must have finite lower < upper")
    return low, high


@dataclass(frozen=True)
class DamiaoMitRanges:
    """The PMAX/VMAX/TMAX mapping currently stored in one motor."""

    position_rad: tuple[float, float]
    velocity_rad_s: tuple[float, float]
    torque_nm: tuple[float, float]
    source: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "position_rad", _validate_range("position_rad", self.position_rad))
        object.__setattr__(
            self, "velocity_rad_s", _validate_range("velocity_rad_s", self.velocity_rad_s)
        )
        object.__setattr__(self, "torque_nm", _validate_range("torque_nm", self.torque_nm))
        if self.source != "motor_register_readback":
            raise ValueError("MIT ranges must come from a motor_register_readback snapshot")


@dataclass(frozen=True)
class DamiaoFeedbackEndpoint:
    motor_name: str
    interface: str
    can_id: int
    master_id: int
    ranges: DamiaoMitRanges

    def __post_init__(self) -> None:
        if not self.motor_name or not self.interface:
            raise ValueError("motor_name and interface are required")
        for name, value in (("can_id", self.can_id), ("master_id", self.master_id)):
            if not isinstance(value, int) or not 0 <= value <= 0x7FF:
                raise ValueError(f"{name} must be an 11-bit CAN identifier")
        # The official V1.4 frame stores the controller ID in D[0]'s low nibble.
        if self.can_id > 0xF:
            raise ValueError("can_id must fit the feedback frame's four-bit controller ID field")


@dataclass(frozen=True)
class DamiaoFeedback:
    motor_name: str
    interface: str
    can_id: int
    master_id: int
    status_code: int
    status_name: str
    position_rad: float
    velocity_rad_s: float
    estimated_output_torque_nm: float
    mos_temperature_c: int
    rotor_temperature_c: int
    hardware_timestamp_ns: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _uint_to_float(value: int, limits: tuple[float, float], bits: int) -> float:
    low, high = limits
    return float(value) * (high - low) / float((1 << bits) - 1) + low


def decode_damiao_feedback(
    frame: ReceivedCanFrame, endpoint: DamiaoFeedbackEndpoint
) -> DamiaoFeedback:
    """Decode one official eight-byte feedback frame without any hardware I/O."""
    if frame.interface != endpoint.interface:
        raise ValueError("frame interface does not match the configured motor endpoint")
    if frame.can_id != endpoint.master_id:
        raise ValueError("frame CAN ID does not match the configured Damiao Master ID")
    if frame.is_extended or frame.is_remote or frame.is_error:
        raise ValueError("Damiao feedback must be a standard CAN data frame")
    if len(frame.data) != 8:
        raise ValueError("Damiao feedback must contain exactly eight data bytes")

    data = frame.data
    controller_id = data[0] & 0x0F
    if controller_id != endpoint.can_id:
        raise ValueError("feedback payload controller ID does not match configured CAN ID")
    status = (data[0] >> 4) & 0x0F
    position_raw = (data[1] << 8) | data[2]
    velocity_raw = (data[3] << 4) | (data[4] >> 4)
    torque_raw = ((data[4] & 0x0F) << 8) | data[5]
    return DamiaoFeedback(
        motor_name=endpoint.motor_name,
        interface=frame.interface,
        can_id=endpoint.can_id,
        master_id=frame.can_id,
        status_code=status,
        status_name=STATUS_NAMES.get(status, "reserved_or_unknown"),
        position_rad=_uint_to_float(position_raw, endpoint.ranges.position_rad, 16),
        velocity_rad_s=_uint_to_float(velocity_raw, endpoint.ranges.velocity_rad_s, 12),
        estimated_output_torque_nm=_uint_to_float(
            torque_raw, endpoint.ranges.torque_nm, 12
        ),
        mos_temperature_c=data[6],
        rotor_temperature_c=data[7],
        hardware_timestamp_ns=frame.hardware_timestamp_ns,
    )


class DamiaoFeedbackDecoder:
    """Map feedback by (interface, Master ID); CAN enumeration order is never used."""

    def __init__(self, endpoints: Iterable[DamiaoFeedbackEndpoint]):
        self._endpoints: dict[tuple[str, int], DamiaoFeedbackEndpoint] = {}
        for endpoint in endpoints:
            key = (endpoint.interface, endpoint.master_id)
            if key in self._endpoints:
                raise ValueError(f"duplicate Damiao feedback endpoint: {key[0]}:{key[1]}")
            self._endpoints[key] = endpoint
        if not self._endpoints:
            raise ValueError("at least one Damiao feedback endpoint is required")

    def decode(self, frame: ReceivedCanFrame) -> DamiaoFeedback:
        endpoint = self._endpoints.get((frame.interface, frame.can_id))
        if endpoint is None:
            raise ValueError(
                f"unconfigured Damiao feedback endpoint: {frame.interface}:{frame.can_id}"
            )
        return decode_damiao_feedback(frame, endpoint)

