"""Pelvis IMU frame conversion and policy-facing state estimation."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

import numpy as np


Vector3 = tuple[float, float, float]
QuaternionWxyz = tuple[float, float, float, float]


def _vector3(value: Vector3, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain three finite values")
    return result


def normalize_quaternion_wxyz(value: QuaternionWxyz) -> np.ndarray:
    quaternion = np.asarray(value, dtype=np.float64)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-9:
        raise ValueError("quaternion norm is zero")
    return quaternion / norm


def quaternion_wxyz_to_matrix(value: QuaternionWxyz) -> np.ndarray:
    w, x, y, z = normalize_quaternion_wxyz(value)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion_wxyz(matrix: np.ndarray) -> np.ndarray:
    rotation = np.asarray(matrix, dtype=np.float64)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("rotation matrix must be finite and 3x3")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        result = np.array(
            [0.25 * scale, (rotation[2, 1] - rotation[1, 2]) / scale,
             (rotation[0, 2] - rotation[2, 0]) / scale,
             (rotation[1, 0] - rotation[0, 1]) / scale]
        )
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            result = np.array(
                [(rotation[2, 1] - rotation[1, 2]) / scale, 0.25 * scale,
                 (rotation[0, 1] + rotation[1, 0]) / scale,
                 (rotation[0, 2] + rotation[2, 0]) / scale]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            result = np.array(
                [(rotation[0, 2] - rotation[2, 0]) / scale,
                 (rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale,
                 (rotation[1, 2] + rotation[2, 1]) / scale]
            )
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            result = np.array(
                [(rotation[1, 0] - rotation[0, 1]) / scale,
                 (rotation[0, 2] + rotation[2, 0]) / scale,
                 (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale]
            )
    result = normalize_quaternion_wxyz(tuple(result))
    return result if result[0] >= 0.0 else -result


@dataclass(frozen=True)
class ImuMount:
    """Fixed rotation from sensor coordinates to the policy root-link frame."""

    sensor_to_body_matrix: np.ndarray

    def __post_init__(self) -> None:
        matrix = np.asarray(self.sensor_to_body_matrix, dtype=np.float64)
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            raise ValueError("sensor_to_body_matrix must be finite and 3x3")
        if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1.0e-9):
            raise ValueError("sensor_to_body_matrix must be orthonormal")
        if not math.isclose(float(np.linalg.det(matrix)), 1.0, abs_tol=1.0e-9):
            raise ValueError("sensor_to_body_matrix must be a right-handed rotation")
        object.__setattr__(self, "sensor_to_body_matrix", matrix)

    @classmethod
    def sprite0825_rear_pelvis(cls) -> "ImuMount":
        # Sensor +Z points backward and sensor +X points down. Right-handedness
        # therefore fixes sensor +Y to point right.
        return cls(
            np.array(
                [
                    [0.0, 0.0, -1.0],
                    [0.0, -1.0, 0.0],
                    [-1.0, 0.0, 0.0],
                ],
                dtype=np.float64,
            )
        )

    @property
    def body_to_sensor_quaternion_wxyz(self) -> np.ndarray:
        return matrix_to_quaternion_wxyz(self.sensor_to_body_matrix.T)

    def vector_sensor_to_body(self, value: Vector3) -> np.ndarray:
        return self.sensor_to_body_matrix @ _vector3(value, "sensor vector")


@dataclass(frozen=True)
class RawImuSample:
    monotonic_ns: int
    angular_velocity_sensor_rad_s: Vector3
    linear_acceleration_sensor_m_s2: Vector3
    orientation_sensor_to_world_wxyz: QuaternionWxyz
    magnetic_field_sensor: Vector3 | None = None


@dataclass(frozen=True)
class PelvisImuSample:
    monotonic_ns: int
    angular_velocity_body_rad_s: np.ndarray
    linear_acceleration_body_m_s2: np.ndarray
    orientation_body_to_world_wxyz: np.ndarray
    projected_gravity_body: np.ndarray
    magnetic_field_body: np.ndarray | None


def transform_pelvis_sample(raw: RawImuSample, mount: ImuMount) -> PelvisImuSample:
    rotation_world_from_sensor = quaternion_wxyz_to_matrix(
        raw.orientation_sensor_to_world_wxyz
    )
    rotation_sensor_from_body = mount.sensor_to_body_matrix.T
    rotation_world_from_body = rotation_world_from_sensor @ rotation_sensor_from_body
    projected_gravity = rotation_world_from_body.T @ np.array([0.0, 0.0, -1.0])
    magnetic = (
        None
        if raw.magnetic_field_sensor is None
        else mount.vector_sensor_to_body(raw.magnetic_field_sensor)
    )
    return PelvisImuSample(
        monotonic_ns=int(raw.monotonic_ns),
        angular_velocity_body_rad_s=mount.vector_sensor_to_body(
            raw.angular_velocity_sensor_rad_s
        ),
        linear_acceleration_body_m_s2=mount.vector_sensor_to_body(
            raw.linear_acceleration_sensor_m_s2
        ),
        orientation_body_to_world_wxyz=matrix_to_quaternion_wxyz(rotation_world_from_body),
        projected_gravity_body=projected_gravity,
        magnetic_field_body=magnetic,
    )


@dataclass(frozen=True)
class ImuValidityLimits:
    expected_update_hz: float = 100.0
    maximum_age_ms: float = 30.0
    maximum_gap_ms: float = 30.0
    quaternion_norm_tolerance: float = 0.05
    maximum_angular_speed_rad_s: float = math.radians(2000.0)
    maximum_acceleration_m_s2: float = 16.0 * 9.80665

    def validate(self) -> None:
        values = (
            self.expected_update_hz,
            self.maximum_age_ms,
            self.maximum_gap_ms,
            self.quaternion_norm_tolerance,
            self.maximum_angular_speed_rad_s,
            self.maximum_acceleration_m_s2,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("IMU validity limits must be finite and positive")


class ImuValidityGate:
    def __init__(self, limits: ImuValidityLimits | None = None):
        self.limits = limits or ImuValidityLimits()
        self.limits.validate()
        self._previous_timestamp_ns: int | None = None

    def check(self, sample: RawImuSample, now_ns: int | None = None) -> tuple[bool, tuple[str, ...]]:
        now_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        errors: list[str] = []
        if sample.monotonic_ns <= 0 or sample.monotonic_ns > now_ns:
            errors.append("invalid_timestamp")
        age_ms = (now_ns - sample.monotonic_ns) / 1.0e6
        if age_ms > self.limits.maximum_age_ms:
            errors.append("stale_sample")
        if self._previous_timestamp_ns is not None:
            gap_ms = (sample.monotonic_ns - self._previous_timestamp_ns) / 1.0e6
            if gap_ms <= 0.0:
                errors.append("non_monotonic_timestamp")
            elif gap_ms > self.limits.maximum_gap_ms:
                errors.append("sample_gap")
        quaternion = np.asarray(sample.orientation_sensor_to_world_wxyz, dtype=np.float64)
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
            errors.append("invalid_quaternion")
        elif abs(float(np.linalg.norm(quaternion)) - 1.0) > self.limits.quaternion_norm_tolerance:
            errors.append("quaternion_norm")
        try:
            angular_velocity = _vector3(
                sample.angular_velocity_sensor_rad_s, "angular velocity"
            )
            acceleration = _vector3(
                sample.linear_acceleration_sensor_m_s2, "linear acceleration"
            )
            if float(np.linalg.norm(angular_velocity)) > self.limits.maximum_angular_speed_rad_s:
                errors.append("angular_speed_range")
            if float(np.linalg.norm(acceleration)) > self.limits.maximum_acceleration_m_s2:
                errors.append("acceleration_range")
        except ValueError:
            errors.append("invalid_vector")
        if sample.monotonic_ns > 0:
            self._previous_timestamp_ns = sample.monotonic_ns
        return not errors, tuple(errors)


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class IntegratedYawEstimator:
    """Integrate body-frame yaw rate outside the actor and reset while standing."""

    def __init__(self, gyro_bias_rad_s: float = 0.0, maximum_gap_ms: float = 30.0):
        if not math.isfinite(gyro_bias_rad_s):
            raise ValueError("gyro bias must be finite")
        if not math.isfinite(maximum_gap_ms) or maximum_gap_ms <= 0.0:
            raise ValueError("maximum gap must be finite and positive")
        self.gyro_bias_rad_s = gyro_bias_rad_s
        self.maximum_gap_ns = int(maximum_gap_ms * 1.0e6)
        self._yaw_rad = 0.0
        self._previous_rate_rad_s: float | None = None
        self._previous_timestamp_ns: int | None = None

    @property
    def yaw_rad(self) -> float:
        return self._yaw_rad

    def reset(self, timestamp_ns: int | None = None) -> None:
        self._yaw_rad = 0.0
        self._previous_rate_rad_s = None
        self._previous_timestamp_ns = timestamp_ns

    def update(self, body_yaw_rate_rad_s: float, timestamp_ns: int, *, standing: bool) -> float:
        rate = float(body_yaw_rate_rad_s) - self.gyro_bias_rad_s
        if not math.isfinite(rate) or timestamp_ns <= 0:
            raise ValueError("yaw update must be finite and positively timestamped")
        if standing:
            self.reset(timestamp_ns)
            self._previous_rate_rad_s = rate
            return 0.0
        if self._previous_timestamp_ns is None or self._previous_rate_rad_s is None:
            self._previous_timestamp_ns = timestamp_ns
            self._previous_rate_rad_s = rate
            return self._yaw_rad
        elapsed_ns = timestamp_ns - self._previous_timestamp_ns
        if elapsed_ns <= 0 or elapsed_ns > self.maximum_gap_ns:
            raise ValueError("yaw integration timestamp gap is invalid")
        elapsed_s = elapsed_ns / 1.0e9
        self._yaw_rad = wrap_to_pi(
            self._yaw_rad + 0.5 * (self._previous_rate_rad_s + rate) * elapsed_s
        )
        self._previous_timestamp_ns = timestamp_ns
        self._previous_rate_rad_s = rate
        return self._yaw_rad
