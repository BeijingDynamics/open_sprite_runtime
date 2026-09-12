"""Pure, receive-only decoding for Damiao MIT feedback frames."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Iterable, Mapping

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
    software_timestamp_ns: int
    hardware_timestamp_ns: int
    userspace_receive_timestamp_ns: int

    @property
    def userspace_queue_age_ns(self) -> int:
        return self.userspace_receive_timestamp_ns - self.software_timestamp_ns

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
        software_timestamp_ns=frame.software_timestamp_ns,
        hardware_timestamp_ns=frame.hardware_timestamp_ns,
        userspace_receive_timestamp_ns=frame.userspace_receive_timestamp_ns,
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


def endpoints_from_hardware_config(
    hardware: Mapping[str, Any],
) -> tuple[DamiaoFeedbackEndpoint, ...]:
    """Build explicit decoder endpoints from a measured hardware contract."""
    adapter = hardware.get("can_adapter")
    if not isinstance(adapter, Mapping):
        raise ValueError("can_adapter configuration is missing")
    interfaces = adapter.get("interfaces")
    if not isinstance(interfaces, list) or len(interfaces) != 4:
        raise ValueError("can_adapter.interfaces must contain four entries")
    if not all(isinstance(name, str) and name for name in interfaces):
        raise ValueError("all SocketCAN interface names must be measured")

    motor_map = hardware.get("motor_map")
    if not isinstance(motor_map, Mapping) or not motor_map:
        raise ValueError("motor_map is missing or empty")
    endpoints: list[DamiaoFeedbackEndpoint] = []
    for motor_name, raw in motor_map.items():
        if not isinstance(raw, Mapping):
            raise ValueError(f"motor_map.{motor_name} must be an object")
        try:
            channel = int(raw["can_channel"])
            can_id = raw["can_id"]
            master_id = raw["master_id"]
            mit = raw["mit_ranges"]
            ranges = DamiaoMitRanges(
                position_rad=tuple(mit["position_rad"]),
                velocity_rad_s=tuple(mit["velocity_rad_s"]),
                torque_nm=tuple(mit["torque_nm"]),
                source=mit["source"],
            )
            interface = interfaces[channel]
        except (KeyError, IndexError, TypeError, ValueError) as exc:
            raise ValueError(f"motor_map.{motor_name} feedback mapping is incomplete") from exc
        endpoints.append(
            DamiaoFeedbackEndpoint(
                motor_name=str(motor_name),
                interface=interface,
                can_id=can_id,
                master_id=master_id,
                ranges=ranges,
            )
        )
    DamiaoFeedbackDecoder(endpoints)
    return tuple(endpoints)


def _percentile_99(values: list[float]) -> float:
    if not values:
        return math.inf
    ordered = sorted(values)
    return ordered[max(0, math.ceil(0.99 * len(ordered)) - 1)]


@dataclass(frozen=True)
class DamiaoMotorRxStats:
    samples: int
    feedback_hz: float
    maximum_hardware_gap_ms: float
    userspace_queue_age_p99_ms: float
    maximum_mos_temperature_c: int | None
    maximum_rotor_temperature_c: int | None


@dataclass(frozen=True)
class DamiaoRxAuditReport:
    expected_motor_count: int
    seen_motor_count: int
    missing_motors: tuple[str, ...]
    motor_stats: dict[str, DamiaoMotorRxStats]
    errors: tuple[str, ...]
    hardware_tx_attempts: int = 0

    @property
    def passed(self) -> bool:
        return not self.errors

    def to_dict(self) -> dict[str, Any]:
        return {
            "expected_motor_count": self.expected_motor_count,
            "seen_motor_count": self.seen_motor_count,
            "missing_motors": self.missing_motors,
            "motor_stats": {
                name: asdict(stats) for name, stats in self.motor_stats.items()
            },
            "errors": self.errors,
            "hardware_tx_attempts": self.hardware_tx_attempts,
            "passed": self.passed,
        }


class DamiaoRxAudit:
    """Aggregate a finite receive-only trace into fail-closed evidence."""

    def __init__(
        self,
        endpoints: Iterable[DamiaoFeedbackEndpoint],
        *,
        minimum_samples_per_motor: int,
        minimum_feedback_hz: float = 475.0,
        maximum_hardware_gap_ms: float = 6.0,
        maximum_userspace_queue_age_p99_ms: float = 6.0,
    ):
        endpoint_list = tuple(endpoints)
        self._decoder = DamiaoFeedbackDecoder(endpoint_list)
        self._expected = {endpoint.motor_name for endpoint in endpoint_list}
        if minimum_samples_per_motor <= 0:
            raise ValueError("minimum_samples_per_motor must be positive")
        for name, value in (
            ("minimum_feedback_hz", minimum_feedback_hz),
            ("maximum_hardware_gap_ms", maximum_hardware_gap_ms),
            ("maximum_userspace_queue_age_p99_ms", maximum_userspace_queue_age_p99_ms),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        self.minimum_samples_per_motor = minimum_samples_per_motor
        self.minimum_feedback_hz = minimum_feedback_hz
        self.maximum_hardware_gap_ms = maximum_hardware_gap_ms
        self.maximum_userspace_queue_age_p99_ms = maximum_userspace_queue_age_p99_ms
        self._samples: dict[str, list[DamiaoFeedback]] = {
            name: [] for name in self._expected
        }
        self._errors: list[str] = []

    def ingest(self, frame: ReceivedCanFrame) -> DamiaoFeedback:
        feedback = self._decoder.decode(frame)
        samples = self._samples[feedback.motor_name]
        if samples and feedback.hardware_timestamp_ns <= samples[-1].hardware_timestamp_ns:
            self._errors.append(f"{feedback.motor_name}: raw hardware timestamp regressed")
        if feedback.status_code not in (0x0, 0x1):
            self._errors.append(
                f"{feedback.motor_name}: motor fault {feedback.status_name}"
            )
        samples.append(feedback)
        return feedback

    def report(self) -> DamiaoRxAuditReport:
        errors = list(self._errors)
        stats: dict[str, DamiaoMotorRxStats] = {}
        missing = tuple(sorted(name for name, values in self._samples.items() if not values))
        for name in sorted(self._samples):
            samples = self._samples[name]
            if not samples:
                continue
            if len(samples) < self.minimum_samples_per_motor:
                errors.append(
                    f"{name}: received {len(samples)} samples; "
                    f"minimum is {self.minimum_samples_per_motor}"
                )
            gaps_ns = [
                current.hardware_timestamp_ns - previous.hardware_timestamp_ns
                for previous, current in zip(samples, samples[1:])
                if current.hardware_timestamp_ns > previous.hardware_timestamp_ns
            ]
            elapsed_ns = samples[-1].hardware_timestamp_ns - samples[0].hardware_timestamp_ns
            feedback_hz = (
                (len(samples) - 1) * 1_000_000_000.0 / elapsed_ns
                if len(samples) > 1 and elapsed_ns > 0
                else 0.0
            )
            maximum_gap_ms = max(gaps_ns, default=0) / 1_000_000.0
            queue_ages_ms = [sample.userspace_queue_age_ns / 1_000_000.0 for sample in samples]
            queue_p99_ms = _percentile_99(queue_ages_ms)
            if feedback_hz < self.minimum_feedback_hz:
                errors.append(f"{name}: feedback rate {feedback_hz:.3f} Hz is too low")
            if maximum_gap_ms > self.maximum_hardware_gap_ms:
                errors.append(f"{name}: hardware timestamp gap {maximum_gap_ms:.3f} ms is too large")
            if queue_p99_ms > self.maximum_userspace_queue_age_p99_ms:
                errors.append(f"{name}: userspace queue age P99 {queue_p99_ms:.3f} ms is too large")
            stats[name] = DamiaoMotorRxStats(
                samples=len(samples),
                feedback_hz=feedback_hz,
                maximum_hardware_gap_ms=maximum_gap_ms,
                userspace_queue_age_p99_ms=queue_p99_ms,
                maximum_mos_temperature_c=max(sample.mos_temperature_c for sample in samples),
                maximum_rotor_temperature_c=max(
                    sample.rotor_temperature_c for sample in samples
                ),
            )
        if missing:
            errors.append("missing motor feedback: " + ", ".join(missing))
        return DamiaoRxAuditReport(
            expected_motor_count=len(self._expected),
            seen_motor_count=len(self._expected) - len(missing),
            missing_motors=missing,
            motor_stats=stats,
            errors=tuple(errors),
        )
