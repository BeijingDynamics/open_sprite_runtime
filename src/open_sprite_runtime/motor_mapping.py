"""Auditable transforms between frozen policy joints and physical motor coordinates."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Mapping

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .ankle import DifferentialAnkle
from .hardware import ANKLE_PAIRS, validate_hardware_inventory


Vector = NDArray[np.float64]


def _finite_scalar(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _sign(value: int, name: str) -> int:
    if value not in (-1, 1):
        raise ValueError(f"{name} must be -1 or 1")
    return int(value)


@dataclass(frozen=True)
class DirectMotorMap:
    motor_name: str
    policy_joint: str
    drive_zero_rad: float
    encoder_sign: int
    policy_to_motor_sign: int
    total_ratio: float

    def __post_init__(self) -> None:
        if not self.motor_name or not self.policy_joint:
            raise ValueError("motor_name and policy_joint are required")
        object.__setattr__(
            self, "drive_zero_rad", _finite_scalar(self.drive_zero_rad, "drive_zero_rad")
        )
        object.__setattr__(self, "encoder_sign", _sign(self.encoder_sign, "encoder_sign"))
        object.__setattr__(
            self,
            "policy_to_motor_sign",
            _sign(self.policy_to_motor_sign, "policy_to_motor_sign"),
        )
        ratio = _finite_scalar(self.total_ratio, "total_ratio")
        if ratio <= 0.0:
            raise ValueError("total_ratio must be positive")
        object.__setattr__(self, "total_ratio", ratio)

    @property
    def signed_ratio(self) -> float:
        return self.encoder_sign * self.policy_to_motor_sign * self.total_ratio

    def joint_to_drive_position(self, position_rad: float) -> float:
        return self.drive_zero_rad + self.signed_ratio * _finite_scalar(
            position_rad, "joint position"
        )

    def drive_to_joint_position(self, position_rad: float) -> float:
        return (
            _finite_scalar(position_rad, "drive position") - self.drive_zero_rad
        ) / self.signed_ratio

    def joint_to_drive_velocity(self, velocity_rad_s: float) -> float:
        return self.signed_ratio * _finite_scalar(velocity_rad_s, "joint velocity")

    def drive_to_joint_velocity(self, velocity_rad_s: float) -> float:
        return _finite_scalar(velocity_rad_s, "drive velocity") / self.signed_ratio

    def joint_to_drive_torque(self, torque_nm: float) -> float:
        return _finite_scalar(torque_nm, "joint torque") / self.signed_ratio

    def drive_to_joint_torque(self, torque_nm: float) -> float:
        return self.signed_ratio * _finite_scalar(torque_nm, "drive torque")

    def joint_to_drive_gains(self, kp: float, kd: float) -> tuple[float, float]:
        scale = self.total_ratio * self.total_ratio
        return (
            _finite_scalar(kp, "joint kp") / scale,
            _finite_scalar(kd, "joint kd") / scale,
        )


@dataclass(frozen=True)
class DifferentialAnkleDriveMap:
    side: str
    joint_names: tuple[str, str]
    motor_names: tuple[str, str]
    drive_zero_rad: Vector
    encoder_sign: Vector
    ankle: DifferentialAnkle

    def __post_init__(self) -> None:
        if self.side not in ANKLE_PAIRS:
            raise ValueError("ankle side must be left or right")
        if self.joint_names != ANKLE_PAIRS[self.side]:
            raise ValueError("ankle joint order must be [pitch, roll]")
        if len(self.motor_names) != 2 or len(set(self.motor_names)) != 2:
            raise ValueError("ankle motor_names must contain two unique labels")
        zeros = np.asarray(self.drive_zero_rad, dtype=np.float64)
        signs = np.asarray(self.encoder_sign, dtype=np.float64)
        if zeros.shape != (2,) or not np.isfinite(zeros).all():
            raise ValueError("ankle drive_zero_rad must be a finite two-vector")
        if signs.shape != (2,) or not np.isin(signs, (-1.0, 1.0)).all():
            raise ValueError("ankle encoder_sign must be a two-vector of -1 or 1")
        object.__setattr__(self, "drive_zero_rad", zeros)
        object.__setattr__(self, "encoder_sign", signs)

    def joint_to_drive_position(self, joint_position: ArrayLike) -> Vector:
        canonical = self.ankle.joint_to_motor_position(joint_position)
        return self.drive_zero_rad + self.encoder_sign * canonical

    def drive_to_joint_position(self, drive_position: ArrayLike) -> Vector:
        drive = np.asarray(drive_position, dtype=np.float64)
        canonical = self.encoder_sign * (drive - self.drive_zero_rad)
        return self.ankle.motor_to_joint_position(canonical)

    def joint_to_drive_velocity(self, joint_velocity: ArrayLike) -> Vector:
        return self.encoder_sign * self.ankle.joint_to_motor_velocity(joint_velocity)

    def drive_to_joint_velocity(self, drive_velocity: ArrayLike) -> Vector:
        drive = np.asarray(drive_velocity, dtype=np.float64)
        return self.ankle.motor_to_joint_velocity(self.encoder_sign * drive)

    def joint_to_drive_torque(self, joint_torque: ArrayLike) -> Vector:
        return self.encoder_sign * self.ankle.joint_to_motor_torque(joint_torque)

    def drive_to_joint_torque(self, drive_torque: ArrayLike) -> Vector:
        drive = np.asarray(drive_torque, dtype=np.float64)
        return self.ankle.motor_to_joint_torque(self.encoder_sign * drive)


@dataclass(frozen=True)
class SpriteMotorMap:
    policy_joint_names: tuple[str, ...]
    direct: Mapping[str, DirectMotorMap]
    ankles: Mapping[str, DifferentialAnkleDriveMap]

    def __post_init__(self) -> None:
        if len(self.policy_joint_names) != 31 or len(set(self.policy_joint_names)) != 31:
            raise ValueError("policy_joint_names must contain 31 unique joints")
        expected_direct = set(self.policy_joint_names) - {
            joint for pair in ANKLE_PAIRS.values() for joint in pair
        }
        if set(self.direct) != expected_direct:
            raise ValueError("direct motor map does not exactly cover non-ankle policy joints")
        if set(self.ankles) != set(ANKLE_PAIRS):
            raise ValueError("left and right differential ankle maps are required")

    @property
    def physical_motor_names(self) -> tuple[str, ...]:
        direct_names = tuple(item.motor_name for item in self.direct.values())
        ankle_names = tuple(
            name for side in ("left", "right") for name in self.ankles[side].motor_names
        )
        return direct_names + ankle_names

    def _joint_vector(self, values: ArrayLike, name: str) -> Vector:
        vector = np.asarray(values, dtype=np.float64)
        if vector.shape != (31,) or not np.isfinite(vector).all():
            raise ValueError(f"{name} must be a finite 31-vector")
        return vector

    def _motor_values(self, values: Mapping[str, float], name: str) -> dict[str, float]:
        if set(values) != set(self.physical_motor_names):
            raise ValueError(f"{name} must exactly cover all 31 physical motors")
        return {key: _finite_scalar(value, name) for key, value in values.items()}

    def joint_to_motor_positions(self, values: ArrayLike) -> dict[str, float]:
        return self._joint_to_motor(values, "position")

    def joint_to_motor_velocities(self, values: ArrayLike) -> dict[str, float]:
        return self._joint_to_motor(values, "velocity")

    def joint_to_motor_torques(self, values: ArrayLike) -> dict[str, float]:
        return self._joint_to_motor(values, "torque")

    def _joint_to_motor(self, values: ArrayLike, kind: str) -> dict[str, float]:
        vector = self._joint_vector(values, f"joint {kind}")
        joint = dict(zip(self.policy_joint_names, vector, strict=True))
        result: dict[str, float] = {}
        for joint_name, mapping in self.direct.items():
            transform = getattr(mapping, f"joint_to_drive_{kind}")
            result[mapping.motor_name] = float(transform(joint[joint_name]))
        for mapping in self.ankles.values():
            transform = getattr(mapping, f"joint_to_drive_{kind}")
            motor_values = transform([joint[name] for name in mapping.joint_names])
            result.update(zip(mapping.motor_names, map(float, motor_values), strict=True))
        if len(result) != 31:
            raise RuntimeError("policy-to-motor mapping did not produce 31 physical motors")
        return result

    def motor_to_joint_positions(self, values: Mapping[str, float]) -> Vector:
        return self._motor_to_joint(values, "position")

    def motor_to_joint_velocities(self, values: Mapping[str, float]) -> Vector:
        return self._motor_to_joint(values, "velocity")

    def motor_to_joint_torques(self, values: Mapping[str, float]) -> Vector:
        return self._motor_to_joint(values, "torque")

    def _motor_to_joint(self, values: Mapping[str, float], kind: str) -> Vector:
        motor = self._motor_values(values, f"motor {kind}")
        joint: dict[str, float] = {}
        for joint_name, mapping in self.direct.items():
            transform = getattr(mapping, f"drive_to_joint_{kind}")
            joint[joint_name] = float(transform(motor[mapping.motor_name]))
        for mapping in self.ankles.values():
            transform = getattr(mapping, f"drive_to_joint_{kind}")
            joint_values = transform([motor[name] for name in mapping.motor_names])
            joint.update(zip(mapping.joint_names, map(float, joint_values), strict=True))
        return np.asarray([joint[name] for name in self.policy_joint_names], dtype=np.float64)


def motor_map_from_hardware_config(
    hardware: Mapping[str, Any], policy_joint_names: Iterable[str]
) -> SpriteMotorMap:
    """Build an exact coordinate map only from a complete measured inventory."""
    hardware_dict = dict(hardware)
    policy_order = tuple(policy_joint_names)
    if hardware_dict.get("configured") is not True:
        raise ValueError("hardware configured=true is required for a motor map")
    report = validate_hardware_inventory(hardware_dict, policy_order)
    if not report.valid:
        raise ValueError("hardware inventory is invalid: " + "; ".join(report.errors))
    records = hardware_dict["motor_map"]
    direct: dict[str, DirectMotorMap] = {}
    for motor_name, record in records.items():
        joint_name = record.get("policy_joint")
        if joint_name is None:
            continue
        direct[joint_name] = DirectMotorMap(
            motor_name=motor_name,
            policy_joint=joint_name,
            drive_zero_rad=record["motor_zero_rad"],
            encoder_sign=record["encoder_sign"],
            policy_to_motor_sign=record["policy_to_motor_sign"],
            total_ratio=float(record["reduction_ratio"]) * float(record["linkage_ratio"]),
        )
    ankles: dict[str, DifferentialAnkleDriveMap] = {}
    for side, joint_names in ANKLE_PAIRS.items():
        ankle_record = hardware_dict["ankles"][side]
        motor_names = tuple(ankle_record["motor_names"])
        ankles[side] = DifferentialAnkleDriveMap(
            side=side,
            joint_names=joint_names,
            motor_names=motor_names,
            drive_zero_rad=np.asarray(
                [records[name]["motor_zero_rad"] for name in motor_names], dtype=np.float64
            ),
            encoder_sign=np.asarray(
                [records[name]["encoder_sign"] for name in motor_names], dtype=np.float64
            ),
            ankle=DifferentialAnkle(
                joint_to_motor_matrix=np.asarray(
                    ankle_record["joint_to_motor_matrix"], dtype=np.float64
                ),
                motor_zero_rad=np.asarray(ankle_record["motor_zero_rad"], dtype=np.float64),
            ),
        )
    result = SpriteMotorMap(policy_order, direct, ankles)
    if set(result.physical_motor_names) != set(records):
        raise ValueError("coordinate map does not exactly cover hardware motor_map")
    return result
