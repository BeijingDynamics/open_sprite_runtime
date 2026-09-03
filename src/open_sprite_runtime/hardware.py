"""Fail-closed validation for the physical Sprite0825 hardware inventory."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Iterable

import numpy as np


ANKLE_PAIRS = {
    "left": ("left_ankle_pitch_joint", "left_ankle_roll_joint"),
    "right": ("right_ankle_pitch_joint", "right_ankle_roll_joint"),
}
ANKLE_JOINTS = frozenset(joint for pair in ANKLE_PAIRS.values() for joint in pair)

MOTOR_REQUIRED_FIELDS = (
    "model",
    "firmware",
    "can_channel",
    "can_id",
    "motor_zero_rad",
    "encoder_sign",
    "reduction_ratio",
    "linkage_ratio",
    "soft_limit_rad",
    "hard_limit_rad",
    "rated_torque_nm",
    "peak_torque_nm",
    "rated_speed_rad_s",
    "max_speed_rad_s",
    "rated_current_a",
    "peak_current_a",
    "temperature_limit_c",
    "mit_ranges",
)


@dataclass(frozen=True)
class HardwareInventoryReport:
    physical_motor_count: int
    expected_motor_count: int
    direct_policy_joints: tuple[str, ...]
    missing_policy_joints: tuple[str, ...]
    duplicate_policy_joints: tuple[str, ...]
    can_endpoints: tuple[str, ...]
    errors: tuple[str, ...]
    warnings: tuple[str, ...]

    @property
    def valid(self) -> bool:
        return not self.errors

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "valid": self.valid}


def _nonempty(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip()) and "TODO" not in value.upper()


def _pair(value: Any, name: str, errors: list[str]) -> tuple[float, float] | None:
    if not isinstance(value, list) or len(value) != 2:
        errors.append(f"{name} must contain [lower, upper]")
        return None
    try:
        low, high = float(value[0]), float(value[1])
    except (TypeError, ValueError):
        errors.append(f"{name} must contain finite numeric values")
        return None
    if not np.isfinite((low, high)).all() or low >= high:
        errors.append(f"{name} must have finite lower < upper")
        return None
    return low, high


def _positive(record: dict[str, Any], field: str, label: str, errors: list[str]) -> None:
    try:
        value = float(record[field])
    except (KeyError, TypeError, ValueError):
        errors.append(f"{label}.{field} must be numeric")
        return
    if not np.isfinite(value) or value <= 0.0:
        errors.append(f"{label}.{field} must be finite and positive")


def _validate_motor_record(
    label: str,
    record: Any,
    policy_joints: set[str],
    errors: list[str],
) -> tuple[str | None, tuple[str, ...] | None, str | None]:
    if not isinstance(record, dict):
        errors.append(f"motor_map.{label} must be an object")
        return None, None, None
    missing = [field for field in MOTOR_REQUIRED_FIELDS if field not in record]
    if missing:
        errors.append(f"motor_map.{label} missing fields: {', '.join(missing)}")

    direct = record.get("policy_joint")
    coupled_raw = record.get("coupled_joints")
    coupled = tuple(coupled_raw) if isinstance(coupled_raw, list) else None
    if (direct is None) == (coupled is None):
        errors.append(f"motor_map.{label} must define exactly one of policy_joint/coupled_joints")
    if direct is not None and direct not in policy_joints:
        errors.append(f"motor_map.{label}.policy_joint is not in the frozen policy order")
    if coupled is not None and coupled not in ANKLE_PAIRS.values():
        errors.append(f"motor_map.{label}.coupled_joints is not a physical ankle pair")

    for field in ("model", "firmware"):
        if not _nonempty(record.get(field)):
            errors.append(f"motor_map.{label}.{field} is empty or TODO")
    channel = record.get("can_channel")
    can_id = record.get("can_id")
    endpoint = None
    if not isinstance(channel, int) or not 0 <= channel < 4:
        errors.append(f"motor_map.{label}.can_channel must be an integer in [0, 3]")
    if not isinstance(can_id, int) or not 0 <= can_id <= 0x7FF:
        errors.append(f"motor_map.{label}.can_id must be an 11-bit integer")
    if isinstance(channel, int) and isinstance(can_id, int):
        endpoint = f"{channel}:{can_id}"

    try:
        zero = float(record["motor_zero_rad"])
        if not np.isfinite(zero):
            raise ValueError
    except (KeyError, TypeError, ValueError):
        errors.append(f"motor_map.{label}.motor_zero_rad must be finite")
    if record.get("encoder_sign") not in (-1, 1):
        errors.append(f"motor_map.{label}.encoder_sign must be -1 or 1")
    if direct is not None and record.get("policy_to_motor_sign") not in (-1, 1):
        errors.append(f"motor_map.{label}.policy_to_motor_sign must be -1 or 1")
    for field in (
        "reduction_ratio",
        "linkage_ratio",
        "rated_torque_nm",
        "peak_torque_nm",
        "rated_speed_rad_s",
        "max_speed_rad_s",
        "rated_current_a",
        "peak_current_a",
        "temperature_limit_c",
    ):
        _positive(record, field, f"motor_map.{label}", errors)
    soft = _pair(record.get("soft_limit_rad"), f"motor_map.{label}.soft_limit_rad", errors)
    hard = _pair(record.get("hard_limit_rad"), f"motor_map.{label}.hard_limit_rad", errors)
    if soft is not None and hard is not None and not (hard[0] <= soft[0] < soft[1] <= hard[1]):
        errors.append(f"motor_map.{label} soft limit must lie inside hard limit")
    mit = record.get("mit_ranges")
    if not isinstance(mit, dict):
        errors.append(f"motor_map.{label}.mit_ranges must be an object")
    else:
        for field in ("position_rad", "velocity_rad_s", "kp", "kd", "torque_nm"):
            _pair(mit.get(field), f"motor_map.{label}.mit_ranges.{field}", errors)
    return direct if isinstance(direct, str) else None, coupled, endpoint


def _close(value: Any, expected: float, tolerance: float = 1.0e-6) -> bool:
    try:
        return abs(float(value) - expected) <= tolerance
    except (TypeError, ValueError):
        return False


def _validate_known_leg_motor(
    label: str,
    record: dict[str, Any],
    direct: str | None,
    coupled: tuple[str, ...] | None,
    errors: list[str],
) -> None:
    model = str(record.get("model", "")).upper().replace("-", "")
    direct_leg = direct is not None and (
        "_hip_" in direct or direct.endswith("_knee_joint")
    )
    if direct_leg:
        if "4340P" not in model:
            errors.append(f"motor_map.{label} leg motor must be DM-J4340P-2EC")
        if not _close(record.get("rated_torque_nm"), 14.0):
            errors.append(f"motor_map.{label} J4340P rated torque must be 14 Nm")
        if not _close(record.get("peak_torque_nm"), 40.0):
            errors.append(f"motor_map.{label} J4340P peak torque must be 40 Nm")
        try:
            rated_speed = float(record["rated_speed_rad_s"])
            max_speed = float(record["max_speed_rad_s"])
            if not 3.6 <= rated_speed <= 4.0:
                errors.append(f"motor_map.{label} J4340P rated speed must be about 3.8 rad/s")
            if not 9.0 <= max_speed <= 10.5:
                errors.append(f"motor_map.{label} J4340P 38 V max speed must be 9-10 rad/s")
        except (KeyError, TypeError, ValueError):
            pass
    if coupled is not None:
        if "4310P" not in model:
            errors.append(f"motor_map.{label} ankle motor must be DM-J4310P-2EC")
        for field, expected in (
            ("rated_torque_nm", 3.5),
            ("peak_torque_nm", 12.5),
            ("rated_speed_rad_s", 12.56),
            ("max_speed_rad_s", 47.1),
        ):
            if not _close(record.get(field), expected, tolerance=0.02):
                errors.append(f"motor_map.{label} J4310P {field} must be {expected}")


def validate_hardware_inventory(
    hardware: dict[str, Any], policy_joint_names: Iterable[str]
) -> HardwareInventoryReport:
    """Validate inventory completeness without opening or transmitting on CAN."""
    errors: list[str] = []
    warnings: list[str] = []
    policy_order = tuple(policy_joint_names)
    policy_set = set(policy_order)
    if len(policy_order) != 31 or len(policy_set) != 31:
        errors.append("frozen policy contract must contain 31 unique joints")

    motor_map = hardware.get("motor_map")
    if not isinstance(motor_map, dict):
        errors.append("motor_map must be an object")
        motor_map = {}
    direct_counts: dict[str, int] = {}
    coupled_counts = {pair: 0 for pair in ANKLE_PAIRS.values()}
    endpoints: list[str] = []
    for label, record in motor_map.items():
        direct, coupled, endpoint = _validate_motor_record(
            str(label), record, policy_set, errors
        )
        if isinstance(record, dict):
            _validate_known_leg_motor(str(label), record, direct, coupled, errors)
        if direct is not None:
            direct_counts[direct] = direct_counts.get(direct, 0) + 1
        if coupled in coupled_counts:
            coupled_counts[coupled] += 1
        if endpoint is not None:
            endpoints.append(endpoint)

    expected_direct = policy_set - ANKLE_JOINTS
    missing = sorted(joint for joint in expected_direct if direct_counts.get(joint, 0) == 0)
    duplicate = sorted(joint for joint, count in direct_counts.items() if count > 1)
    unexpected_direct = sorted(set(direct_counts) & ANKLE_JOINTS)
    if missing:
        errors.append(f"missing direct motor mappings for: {', '.join(missing)}")
    if duplicate:
        errors.append(f"duplicate direct motor mappings for: {', '.join(duplicate)}")
    if unexpected_direct:
        errors.append(
            "ankle policy joints must use coupled_joints, not direct mappings: "
            + ", ".join(unexpected_direct)
        )
    for side, pair in ANKLE_PAIRS.items():
        if coupled_counts[pair] != 2:
            errors.append(f"{side} ankle requires exactly two coupled motor records")
    duplicate_endpoints = sorted({item for item in endpoints if endpoints.count(item) > 1})
    if duplicate_endpoints:
        errors.append(f"duplicate CAN channel/ID endpoints: {', '.join(duplicate_endpoints)}")
    if len(motor_map) != 31:
        errors.append(f"motor_map has {len(motor_map)} physical motors; expected 31")

    imu = hardware.get("imu", {})
    if not isinstance(imu, dict) or imu.get("configured") is not True:
        errors.append("pelvis IMU is not configured")
    else:
        for field in (
            "mount_link",
            "body_to_sensor_quaternion_wxyz",
            "gyro_units",
            "quaternion_order",
            "update_hz",
            "timestamp_source",
            "measured_yaw_drift_deg_per_min",
        ):
            if field not in imu or imu[field] is None:
                errors.append(f"imu.{field} is missing")
    estop = hardware.get("estop", {})
    if not isinstance(estop, dict) or estop.get("configured") is not True:
        errors.append("independent hardware e-stop is not configured")
    elif not _nonempty(estop.get("hardware_chain")):
        errors.append("estop.hardware_chain is empty or TODO")

    ankles = hardware.get("ankles", {})
    for side in ANKLE_PAIRS:
        config = ankles.get(side, {}) if isinstance(ankles, dict) else {}
        if not isinstance(config, dict) or config.get("calibrated") is not True:
            errors.append(f"{side} differential ankle is not calibrated")
            continue
        try:
            matrix = np.asarray(config["joint_to_motor_matrix"], dtype=float)
            zeros = np.asarray(config["motor_zero_rad"], dtype=float)
            if matrix.shape != (2, 2) or zeros.shape != (2,):
                raise ValueError
            if not np.isfinite(matrix).all() or not np.isfinite(zeros).all():
                raise ValueError
            if abs(float(np.linalg.det(matrix))) < 1.0e-6:
                errors.append(f"{side} ankle matrix is singular")
        except (KeyError, TypeError, ValueError):
            errors.append(f"{side} ankle matrix/zeros are invalid")
        if not _nonempty(config.get("source")) or "PLACEHOLDER" in str(config.get("source")):
            errors.append(f"{side} ankle calibration source is missing or placeholder")

    if hardware.get("configured") is True and errors:
        errors.append("configured=true is inconsistent with an incomplete hardware inventory")
    if hardware.get("configured") is not True:
        warnings.append("hardware remains intentionally unconfigured and must not arm")
    return HardwareInventoryReport(
        physical_motor_count=len(motor_map),
        expected_motor_count=31,
        direct_policy_joints=tuple(sorted(direct_counts)),
        missing_policy_joints=tuple(missing),
        duplicate_policy_joints=tuple(duplicate),
        can_endpoints=tuple(sorted(endpoints)),
        errors=tuple(errors),
        warnings=tuple(warnings),
    )
