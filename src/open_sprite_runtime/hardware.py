"""Fail-closed validation for the physical Sprite0825 hardware inventory."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import re
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
    "master_id",
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

SOCKETCAN_INTERFACE_RE = re.compile(r"^[A-Za-z0-9_.-]+$")


def _validate_can_adapter(hardware: dict[str, Any], errors: list[str]) -> None:
    adapter = hardware.get("can_adapter")
    if not isinstance(adapter, dict):
        errors.append("can_adapter configuration is missing")
        return
    if adapter.get("backend") != "socketcan":
        errors.append("can_adapter.backend must be socketcan")
    if not _nonempty(adapter.get("vendor")):
        errors.append("can_adapter.vendor is empty or TODO")
    if not _nonempty(adapter.get("sdk_version")):
        errors.append("can_adapter.sdk_version is empty or TODO")

    interfaces = adapter.get("interfaces")
    if not isinstance(interfaces, list) or len(interfaces) != 4:
        errors.append("can_adapter.interfaces must contain four SocketCAN interface names")
    elif any(
        not isinstance(name, str) or not SOCKETCAN_INTERFACE_RE.fullmatch(name)
        for name in interfaces
    ):
        errors.append("can_adapter.interfaces contains an invalid interface name")
    elif len(set(interfaces)) != 4:
        errors.append("can_adapter.interfaces must be unique")

    shadow = adapter.get("rx_only_shadow")
    if not isinstance(shadow, dict):
        errors.append("can_adapter.rx_only_shadow configuration is missing")
        return
    required_true = (
        "required_before_arm",
        "kernel_listen_only_required",
        "hardware_timestamp_required",
        "completed",
    )
    for field in required_true:
        if shadow.get(field) is not True:
            errors.append(f"can_adapter.rx_only_shadow.{field} must be true")
    if shadow.get("kernel_ctrlmode") != "CAN_CTRLMODE_LISTENONLY":
        errors.append(
            "can_adapter.rx_only_shadow.kernel_ctrlmode must be CAN_CTRLMODE_LISTENONLY"
        )
    if not _nonempty(shadow.get("evidence_report")):
        errors.append("can_adapter.rx_only_shadow.evidence_report is empty or TODO")
    if shadow.get("timestamp_source") not in ("device", "hardware"):
        errors.append("can_adapter.rx_only_shadow.timestamp_source must be device or hardware")
    try:
        p99 = float(shadow["measured_rx_age_p99_ms"])
        if not np.isfinite(p99) or p99 <= 0.0 or p99 > 6.0:
            raise ValueError
    except (KeyError, TypeError, ValueError):
        errors.append(
            "can_adapter.rx_only_shadow.measured_rx_age_p99_ms must be in (0, 6]"
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
    direct = record.get("policy_joint")
    coupled_raw = record.get("coupled_joints")
    coupled = tuple(coupled_raw) if isinstance(coupled_raw, list) else None
    if (direct is None) == (coupled is None):
        errors.append(f"motor_map.{label} must define exactly one of policy_joint/coupled_joints")
    if direct is not None and direct not in policy_joints:
        errors.append(f"motor_map.{label}.policy_joint is not in the frozen policy order")
    if coupled is not None and coupled not in ANKLE_PAIRS.values():
        errors.append(f"motor_map.{label}.coupled_joints is not a physical ankle pair")

    required = list(MOTOR_REQUIRED_FIELDS)
    if direct is not None:
        required.append("policy_to_motor_sign")
    unset = [field for field in required if record.get(field) is None]
    if unset:
        errors.append(f"motor_map.{label} unset fields: {', '.join(unset)}")

    for field in ("model", "firmware"):
        if record.get(field) is not None and not _nonempty(record.get(field)):
            errors.append(f"motor_map.{label}.{field} is empty or TODO")
    channel = record.get("can_channel")
    can_id = record.get("can_id")
    master_id = record.get("master_id")
    endpoint = None
    if channel is not None and (not isinstance(channel, int) or not 0 <= channel < 4):
        errors.append(f"motor_map.{label}.can_channel must be an integer in [0, 3]")
    if can_id is not None and (not isinstance(can_id, int) or not 0 <= can_id <= 0x7FF):
        errors.append(f"motor_map.{label}.can_id must be an 11-bit integer")
    elif isinstance(can_id, int) and can_id > 0xF:
        errors.append(
            f"motor_map.{label}.can_id must fit the Damiao feedback controller-ID nibble"
        )
    if master_id is not None and (
        not isinstance(master_id, int) or not 0 <= master_id <= 0x7FF
    ):
        errors.append(f"motor_map.{label}.master_id must be an 11-bit integer")
    if isinstance(channel, int) and isinstance(can_id, int):
        endpoint = f"{channel}:{can_id}"

    if record.get("motor_zero_rad") is not None:
        try:
            zero = float(record["motor_zero_rad"])
            if not np.isfinite(zero):
                raise ValueError
        except (TypeError, ValueError):
            errors.append(f"motor_map.{label}.motor_zero_rad must be finite")
    if record.get("encoder_sign") is not None and record.get("encoder_sign") not in (-1, 1):
        errors.append(f"motor_map.{label}.encoder_sign must be -1 or 1")
    if (
        direct is not None
        and record.get("policy_to_motor_sign") is not None
        and record.get("policy_to_motor_sign") not in (-1, 1)
    ):
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
        if record.get(field) is not None:
            _positive(record, field, f"motor_map.{label}", errors)
    soft = (
        _pair(record["soft_limit_rad"], f"motor_map.{label}.soft_limit_rad", errors)
        if record.get("soft_limit_rad") is not None
        else None
    )
    hard = (
        _pair(record["hard_limit_rad"], f"motor_map.{label}.hard_limit_rad", errors)
        if record.get("hard_limit_rad") is not None
        else None
    )
    if soft is not None and hard is not None and not (hard[0] <= soft[0] < soft[1] <= hard[1]):
        errors.append(f"motor_map.{label} soft limit must lie inside hard limit")
    mit = record.get("mit_ranges")
    if mit is not None and not isinstance(mit, dict):
        errors.append(f"motor_map.{label}.mit_ranges must be an object")
    elif isinstance(mit, dict):
        unset_mit = [
            field
            for field in (
                "position_rad",
                "velocity_rad_s",
                "kp",
                "kd",
                "torque_nm",
                "source",
            )
            if mit.get(field) is None
        ]
        if unset_mit:
            errors.append(
                f"motor_map.{label}.mit_ranges unset fields: {', '.join(unset_mit)}"
            )
        for field in ("position_rad", "velocity_rad_s", "kp", "kd", "torque_nm"):
            if mit.get(field) is not None:
                _pair(mit[field], f"motor_map.{label}.mit_ranges.{field}", errors)
        if mit.get("source") is not None and mit.get("source") != "motor_register_readback":
            errors.append(
                f"motor_map.{label}.mit_ranges.source must be motor_register_readback"
            )
    return direct if isinstance(direct, str) else None, coupled, endpoint


def _close(value: Any, expected: float, tolerance: float = 1.0e-6) -> bool:
    try:
        return abs(float(value) - expected) <= tolerance
    except (TypeError, ValueError):
        return False


def _validate_known_motor_profile(
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
    direct_upgraded_shoulder = direct is not None and direct in {
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
    }
    if direct_leg or direct_upgraded_shoulder:
        if "4340P" not in model:
            role = "leg" if direct_leg else "shoulder pitch/roll"
            errors.append(f"motor_map.{label} {role} motor must be DM-J4340P-2EC")
        if not _close(record.get("rated_torque_nm"), 14.0):
            errors.append(f"motor_map.{label} J4340P rated torque must be 14 Nm")
        if not _close(record.get("peak_torque_nm"), 40.0):
            errors.append(f"motor_map.{label} J4340P peak torque must be 40 Nm")
        try:
            rated_speed = float(record["rated_speed_rad_s"])
            max_speed = float(record["max_speed_rad_s"])
            if not 3.6 <= rated_speed <= 4.0:
                errors.append(f"motor_map.{label} J4340P rated speed must be about 3.8 rad/s")
            if not _close(max_speed, 9.3, tolerance=0.02):
                errors.append(
                    f"motor_map.{label} J4340P 38 V max speed must match the frozen 9.3 rad/s envelope"
                )
        except (KeyError, TypeError, ValueError):
            pass
    if coupled is not None:
        if "4310P" not in model:
            errors.append(f"motor_map.{label} ankle motor must be DM-J4310P-2EC")
        for field, expected in (
            ("rated_torque_nm", 3.5),
            ("peak_torque_nm", 12.5),
            ("rated_speed_rad_s", 12.56),
            ("max_speed_rad_s", 36.2),
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

    _validate_can_adapter(hardware, errors)

    controller = hardware.get("controller")
    if not isinstance(controller, dict):
        errors.append("controller configuration is missing")
    else:
        voltage = controller.get("nominal_bus_voltage_v")
        try:
            voltage = float(voltage)
            if not np.isfinite(voltage) or not 36.0 <= voltage <= 40.0:
                raise ValueError
        except (TypeError, ValueError):
            errors.append(
                "controller.nominal_bus_voltage_v must be within the qualified 36-40 V range"
            )

    motor_map = hardware.get("motor_map")
    if not isinstance(motor_map, dict):
        errors.append("motor_map must be an object")
        motor_map = {}
    direct_counts: dict[str, int] = {}
    coupled_counts = {pair: 0 for pair in ANKLE_PAIRS.values()}
    endpoints: list[str] = []
    feedback_endpoints: list[str] = []
    for label, record in motor_map.items():
        direct, coupled, endpoint = _validate_motor_record(
            str(label), record, policy_set, errors
        )
        if isinstance(record, dict):
            _validate_known_motor_profile(str(label), record, direct, coupled, errors)
        if direct is not None:
            direct_counts[direct] = direct_counts.get(direct, 0) + 1
        if coupled in coupled_counts:
            coupled_counts[coupled] += 1
        if endpoint is not None:
            endpoints.append(endpoint)
        if isinstance(record, dict):
            channel = record.get("can_channel")
            master_id = record.get("master_id")
            if isinstance(channel, int) and isinstance(master_id, int):
                feedback_endpoints.append(f"{channel}:{master_id}")

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
    duplicate_feedback = sorted(
        {item for item in feedback_endpoints if feedback_endpoints.count(item) > 1}
    )
    if duplicate_feedback:
        errors.append(
            "duplicate CAN channel/Master-ID feedback endpoints: "
            + ", ".join(duplicate_feedback)
        )
    feedback_command_overlap = sorted(set(endpoints) & set(feedback_endpoints))
    if feedback_command_overlap:
        errors.append(
            "CAN command and feedback endpoints overlap: "
            + ", ".join(feedback_command_overlap)
        )
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


def make_hardware_template(policy_joint_names: Iterable[str]) -> dict[str, Any]:
    """Build a complete, non-armable 31-motor measurement worksheet."""
    policy_order = tuple(policy_joint_names)
    if len(policy_order) != 31 or len(set(policy_order)) != 31:
        raise ValueError("frozen policy contract must contain 31 unique joints")

    def base_record() -> dict[str, Any]:
        return {
            "model": None,
            "firmware": None,
            "can_channel": None,
            "can_id": None,
            "master_id": None,
            "motor_zero_rad": None,
            "encoder_sign": None,
            "reduction_ratio": None,
            "linkage_ratio": None,
            "soft_limit_rad": None,
            "hard_limit_rad": None,
            "rated_torque_nm": None,
            "peak_torque_nm": None,
            "rated_speed_rad_s": None,
            "max_speed_rad_s": None,
            "rated_current_a": None,
            "peak_current_a": None,
            "temperature_limit_c": None,
            "mit_ranges": {
                "position_rad": None,
                "velocity_rad_s": None,
                "kp": None,
                "kd": None,
                "torque_nm": None,
                "source": None,
            },
        }

    motor_map: dict[str, dict[str, Any]] = {}
    for joint in policy_order:
        if joint in ANKLE_JOINTS:
            continue
        record = base_record()
        record["policy_joint"] = joint
        record["policy_to_motor_sign"] = None
        if (
            "_hip_" in joint
            or joint.endswith("_knee_joint")
            or joint
            in {
                "left_shoulder_pitch_joint",
                "left_shoulder_roll_joint",
                "right_shoulder_pitch_joint",
                "right_shoulder_roll_joint",
            }
        ):
            record.update(
                {
                    "model": "DM-J4340P-2EC",
                    "rated_torque_nm": 14.0,
                    "peak_torque_nm": 40.0,
                    "rated_speed_rad_s": 3.77,
                    "max_speed_rad_s": 9.3,
                }
            )
        motor_map[joint.removesuffix("_joint") + "_motor"] = record

    for side, pair in ANKLE_PAIRS.items():
        for suffix in ("a", "b"):
            record = base_record()
            record.update(
                {
                    "model": "DM-J4310P-2EC",
                    "rated_torque_nm": 3.5,
                    "peak_torque_nm": 12.5,
                    "rated_speed_rad_s": 12.56,
                    "max_speed_rad_s": 36.2,
                    "coupled_joints": list(pair),
                }
            )
            motor_map[f"{side}_ankle_motor_{suffix}"] = record

    if len(motor_map) != 31:
        raise AssertionError("generated hardware template must contain 31 physical motors")
    return {
        "schema": "sprite0825_hardware_contract_v1",
        "configured": False,
        "controller": {
            "candidate": "jetson_orin_nano_or_raspberry_pi_5",
            "usb_canfd_channels": 4,
            "measured_round_trip_latency_required": True,
            "nominal_bus_voltage_v": 38.0,
        },
        "can_adapter": {
            "backend": "socketcan",
            "vendor": "KunHong",
            "sdk_version": "1.3.1",
            "interfaces": [None, None, None, None],
            "rx_only_shadow": {
                "required_before_arm": True,
                "kernel_listen_only_required": True,
                "kernel_ctrlmode": "CAN_CTRLMODE_LISTENONLY",
                "hardware_timestamp_required": True,
                "completed": False,
                "timestamp_source": None,
                "measured_rx_age_p99_ms": None,
                "evidence_report": None,
            },
        },
        "imu": {
            "configured": False,
            "mount_link": None,
            "body_to_sensor_quaternion_wxyz": None,
            "gyro_units": "rad_s",
            "quaternion_order": None,
            "update_hz": None,
            "timestamp_source": None,
            "measured_yaw_drift_deg_per_min": None,
        },
        "estop": {
            "configured": False,
            "hardware_chain": None,
            "watchdog_timeout_ms": 20,
        },
        "motor_map": motor_map,
        "ankles": {
            side: {
                "calibrated": False,
                "joint_order": list(pair),
                "motor_order": [f"{side}_ankle_motor_a", f"{side}_ankle_motor_b"],
                "joint_to_motor_matrix": [[1.0, 1.0], [1.0, -1.0]],
                "motor_zero_rad": [0.0, 0.0],
                "source": "IDEAL_PLACEHOLDER_DO_NOT_ARM",
            }
            for side, pair in ANKLE_PAIRS.items()
        },
    }
