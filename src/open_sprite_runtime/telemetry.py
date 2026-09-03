"""Fail-closed checks for measured motor state before command transmission."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Iterable

import numpy as np


@dataclass(frozen=True)
class MotorTelemetryLimits:
    hard_position_rad: tuple[float, float]
    maximum_speed_rad_s: float
    peak_torque_nm: float
    peak_current_a: float
    maximum_temperature_c: float
    torque_speed_envelope: tuple[tuple[float, float], ...]

    def validate(self) -> None:
        low, high = self.hard_position_rad
        scalars = (
            low,
            high,
            self.maximum_speed_rad_s,
            self.peak_torque_nm,
            self.peak_current_a,
            self.maximum_temperature_c,
        )
        if not np.isfinite(scalars).all() or low >= high:
            raise ValueError("motor telemetry limits must be finite with lower < upper")
        if min(scalars[2:]) <= 0.0:
            raise ValueError("speed, torque, current, and temperature limits must be positive")
        points = np.asarray(self.torque_speed_envelope, dtype=float)
        if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] != 2:
            raise ValueError("torque-speed envelope requires at least two [speed, torque] points")
        if not np.isfinite(points).all() or np.any(points < 0.0):
            raise ValueError("torque-speed envelope points must be finite and non-negative")
        if np.any(np.diff(points[:, 0]) <= 0.0):
            raise ValueError("torque-speed envelope speeds must be strictly increasing")
        if np.any(np.diff(points[:, 1]) > 0.0):
            raise ValueError("torque-speed envelope torque must be non-increasing")
        if points[-1, 0] > self.maximum_speed_rad_s:
            raise ValueError("torque-speed envelope exceeds maximum speed")
        if points[0, 1] > self.peak_torque_nm:
            raise ValueError("torque-speed envelope exceeds peak torque")

    def torque_limit_at_speed(self, speed_rad_s: float) -> float:
        self.validate()
        speed = abs(float(speed_rad_s))
        points = np.asarray(self.torque_speed_envelope, dtype=float)
        if speed > points[-1, 0]:
            return 0.0
        return float(np.interp(speed, points[:, 0], points[:, 1]))


@dataclass(frozen=True)
class MotorTelemetry:
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    current_a: float
    temperature_c: float


@dataclass(frozen=True)
class MotorTelemetryDecision:
    healthy: bool
    violations: tuple[str, ...]
    margins: dict[str, float]

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def limits_from_hardware_record(record: dict[str, Any]) -> MotorTelemetryLimits:
    """Build limits from one validated hardware record.

    A measured envelope may be supplied explicitly. Otherwise use the
    conservative nameplate polyline peak-at-zero -> rated point -> zero-at-max.
    """
    envelope = record.get("torque_speed_envelope")
    if envelope is None:
        envelope = (
            (0.0, float(record["peak_torque_nm"])),
            (float(record["rated_speed_rad_s"]), float(record["rated_torque_nm"])),
            (float(record["max_speed_rad_s"]), 0.0),
        )
    limits = MotorTelemetryLimits(
        hard_position_rad=tuple(float(value) for value in record["hard_limit_rad"]),
        maximum_speed_rad_s=float(record["max_speed_rad_s"]),
        peak_torque_nm=float(record["peak_torque_nm"]),
        peak_current_a=float(record["peak_current_a"]),
        maximum_temperature_c=float(record["temperature_limit_c"]),
        torque_speed_envelope=tuple(
            (float(point[0]), float(point[1])) for point in envelope
        ),
    )
    limits.validate()
    return limits


def evaluate_motor_telemetry(
    sample: MotorTelemetry, limits: MotorTelemetryLimits
) -> MotorTelemetryDecision:
    """Check one physical motor sample using absolute SI-unit limits."""
    limits.validate()
    values = np.asarray(
        [
            sample.position_rad,
            sample.velocity_rad_s,
            sample.torque_nm,
            sample.current_a,
            sample.temperature_c,
        ],
        dtype=float,
    )
    if not np.isfinite(values).all():
        return MotorTelemetryDecision(False, ("nonfinite",), {})

    low, high = limits.hard_position_rad
    speed = abs(sample.velocity_rad_s)
    torque = abs(sample.torque_nm)
    current = abs(sample.current_a)
    envelope_torque = limits.torque_limit_at_speed(speed)
    margins = {
        "position_lower_rad": sample.position_rad - low,
        "position_upper_rad": high - sample.position_rad,
        "speed_rad_s": limits.maximum_speed_rad_s - speed,
        "peak_torque_nm": limits.peak_torque_nm - torque,
        "torque_speed_nm": envelope_torque - torque,
        "peak_current_a": limits.peak_current_a - current,
        "temperature_c": limits.maximum_temperature_c - sample.temperature_c,
    }
    violations = tuple(name for name, margin in margins.items() if margin < 0.0)
    return MotorTelemetryDecision(not violations, violations, margins)


def evaluate_motor_bank(
    samples: dict[str, MotorTelemetry],
    limits: dict[str, MotorTelemetryLimits],
    expected_motor_names: Iterable[str],
) -> dict[str, object]:
    """Evaluate an exact physical-motor set; missing or extra frames fail closed."""
    expected = tuple(expected_motor_names)
    missing = sorted(set(expected) - set(samples))
    extra = sorted(set(samples) - set(expected))
    missing_limits = sorted(set(expected) - set(limits))
    decisions: dict[str, MotorTelemetryDecision] = {}
    for name in expected:
        if name in samples and name in limits:
            decisions[name] = evaluate_motor_telemetry(samples[name], limits[name])
    unhealthy = sorted(name for name, decision in decisions.items() if not decision.healthy)
    healthy = not (missing or extra or missing_limits or unhealthy)
    return {
        "healthy": healthy,
        "expected_motor_count": len(expected),
        "received_motor_count": len(samples),
        "missing_motors": missing,
        "extra_motors": extra,
        "missing_limits": missing_limits,
        "unhealthy_motors": unhealthy,
        "decisions": {name: decision.to_dict() for name, decision in decisions.items()},
    }
