"""Auditable transforms between frozen policy joints and physical motor coordinates."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Mapping

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .ankle import DifferentialPair
from .damiao import DamiaoMitCommand
from .hardware import DIFFERENTIAL_PAIRS, validate_hardware_inventory


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

    def joint_impedance_to_drive_command(
        self,
        desired_position_rad: float,
        desired_velocity_rad_s: float,
        kp: float,
        kd: float,
        feedforward_torque_nm: float,
    ) -> DamiaoMitCommand:
        drive_kp, drive_kd = self.joint_to_drive_gains(kp, kd)
        return DamiaoMitCommand(
            position_rad=self.joint_to_drive_position(desired_position_rad),
            velocity_rad_s=self.joint_to_drive_velocity(desired_velocity_rad_s),
            kp=drive_kp,
            kd=drive_kd,
            feedforward_torque_nm=self.joint_to_drive_torque(feedforward_torque_nm),
        )


@dataclass(frozen=True)
class DifferentialPairDriveMap:
    name: str
    joint_names: tuple[str, str]
    motor_names: tuple[str, str]
    drive_zero_rad: Vector
    encoder_sign: Vector
    coupling: DifferentialPair

    def __post_init__(self) -> None:
        if self.name not in DIFFERENTIAL_PAIRS:
            raise ValueError("unknown differential pair")
        if self.joint_names != DIFFERENTIAL_PAIRS[self.name]:
            raise ValueError("differential joint order must match the hardware contract")
        if len(self.motor_names) != 2 or len(set(self.motor_names)) != 2:
            raise ValueError("differential motor_names must contain two unique labels")
        zeros = np.asarray(self.drive_zero_rad, dtype=np.float64)
        signs = np.asarray(self.encoder_sign, dtype=np.float64)
        if zeros.shape != (2,) or not np.isfinite(zeros).all():
            raise ValueError("differential drive_zero_rad must be a finite two-vector")
        if signs.shape != (2,) or not np.isin(signs, (-1.0, 1.0)).all():
            raise ValueError("differential encoder_sign must be a two-vector of -1 or 1")
        object.__setattr__(self, "drive_zero_rad", zeros)
        object.__setattr__(self, "encoder_sign", signs)

    def joint_to_drive_position(self, joint_position: ArrayLike) -> Vector:
        canonical = self.coupling.joint_to_motor_position(joint_position)
        return self.drive_zero_rad + self.encoder_sign * canonical

    def drive_to_joint_position(self, drive_position: ArrayLike) -> Vector:
        drive = np.asarray(drive_position, dtype=np.float64)
        canonical = self.encoder_sign * (drive - self.drive_zero_rad)
        return self.coupling.motor_to_joint_position(canonical)

    def joint_to_drive_velocity(self, joint_velocity: ArrayLike) -> Vector:
        return self.encoder_sign * self.coupling.joint_to_motor_velocity(joint_velocity)

    def drive_to_joint_velocity(self, drive_velocity: ArrayLike) -> Vector:
        drive = np.asarray(drive_velocity, dtype=np.float64)
        return self.coupling.motor_to_joint_velocity(self.encoder_sign * drive)

    def joint_to_drive_torque(self, joint_torque: ArrayLike) -> Vector:
        return self.encoder_sign * self.coupling.joint_to_motor_torque(joint_torque)

    def drive_to_joint_torque(self, drive_torque: ArrayLike) -> Vector:
        drive = np.asarray(drive_torque, dtype=np.float64)
        return self.coupling.motor_to_joint_torque(self.encoder_sign * drive)

    @property
    def joint_to_drive_matrix(self) -> NDArray[np.float64]:
        return np.diag(self.encoder_sign) @ self.coupling.joint_to_motor_matrix

    def joint_impedance_to_drive_commands(
        self,
        desired_position_rad: ArrayLike,
        desired_velocity_rad_s: ArrayLike,
        measured_position_rad: ArrayLike,
        measured_velocity_rad_s: ArrayLike,
        kp: ArrayLike,
        kd: ArrayLike,
        feedforward_torque_nm: ArrayLike,
    ) -> tuple[DamiaoMitCommand, DamiaoMitCommand]:
        """Split exact joint impedance into embedded diagonal PD and host coupling."""
        desired_position = np.asarray(desired_position_rad, dtype=np.float64)
        desired_velocity = np.asarray(desired_velocity_rad_s, dtype=np.float64)
        measured_position = np.asarray(measured_position_rad, dtype=np.float64)
        measured_velocity = np.asarray(measured_velocity_rad_s, dtype=np.float64)
        joint_kp = np.asarray(kp, dtype=np.float64)
        joint_kd = np.asarray(kd, dtype=np.float64)
        joint_ff = np.asarray(feedforward_torque_nm, dtype=np.float64)
        vectors = (
            desired_position,
            desired_velocity,
            measured_position,
            measured_velocity,
            joint_kp,
            joint_kd,
            joint_ff,
        )
        if any(value.shape != (2,) or not np.isfinite(value).all() for value in vectors):
            raise ValueError("differential impedance inputs must be finite two-vectors")
        if np.any(joint_kp < 0.0) or np.any(joint_kd < 0.0):
            raise ValueError("differential impedance gains must be non-negative")

        desired_drive_position = self.joint_to_drive_position(desired_position)
        desired_drive_velocity = self.joint_to_drive_velocity(desired_velocity)
        measured_drive_position = self.joint_to_drive_position(measured_position)
        measured_drive_velocity = self.joint_to_drive_velocity(measured_velocity)
        position_error = desired_drive_position - measured_drive_position
        velocity_error = desired_drive_velocity - measured_drive_velocity

        inverse = np.linalg.inv(self.joint_to_drive_matrix)
        drive_kp = inverse.T @ np.diag(joint_kp) @ inverse
        drive_kd = inverse.T @ np.diag(joint_kd) @ inverse
        embedded_kp = np.diag(drive_kp)
        embedded_kd = np.diag(drive_kd)
        coupled_torque = (
            (drive_kp - np.diag(embedded_kp)) @ position_error
            + (drive_kd - np.diag(embedded_kd)) @ velocity_error
            + inverse.T @ joint_ff
        )
        return tuple(
            DamiaoMitCommand(
                position_rad=float(desired_drive_position[index]),
                velocity_rad_s=float(desired_drive_velocity[index]),
                kp=float(embedded_kp[index]),
                kd=float(embedded_kd[index]),
                feedforward_torque_nm=float(coupled_torque[index]),
            )
            for index in range(2)
        )


@dataclass(frozen=True)
class SpriteMotorMap:
    policy_joint_names: tuple[str, ...]
    direct: Mapping[str, DirectMotorMap]
    differentials: Mapping[str, DifferentialPairDriveMap]

    def __post_init__(self) -> None:
        if len(self.policy_joint_names) != 31 or len(set(self.policy_joint_names)) != 31:
            raise ValueError("policy_joint_names must contain 31 unique joints")
        expected_direct = set(self.policy_joint_names) - {
            joint for pair in DIFFERENTIAL_PAIRS.values() for joint in pair
        }
        if set(self.direct) != expected_direct:
            raise ValueError("direct motor map does not exactly cover non-differential joints")
        if set(self.differentials) != set(DIFFERENTIAL_PAIRS):
            raise ValueError("left/right ankle and head differential maps are required")

    @property
    def ankles(self) -> Mapping[str, DifferentialPairDriveMap]:
        """Backward-compatible left/right ankle view."""
        return {
            "left": self.differentials["left_ankle"],
            "right": self.differentials["right_ankle"],
        }

    @property
    def physical_motor_names(self) -> tuple[str, ...]:
        direct_names = tuple(item.motor_name for item in self.direct.values())
        differential_names = tuple(
            name
            for pair_name in DIFFERENTIAL_PAIRS
            for name in self.differentials[pair_name].motor_names
        )
        return direct_names + differential_names

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
        for mapping in self.differentials.values():
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

    def joint_impedance_to_motor_commands(
        self,
        desired_position_rad: ArrayLike,
        desired_velocity_rad_s: ArrayLike,
        measured_position_rad: ArrayLike,
        measured_velocity_rad_s: ArrayLike,
        kp: ArrayLike,
        kd: ArrayLike,
        feedforward_torque_nm: ArrayLike,
    ) -> dict[str, DamiaoMitCommand]:
        vectors = {
            "desired_position": self._joint_vector(desired_position_rad, "desired position"),
            "desired_velocity": self._joint_vector(desired_velocity_rad_s, "desired velocity"),
            "measured_position": self._joint_vector(measured_position_rad, "measured position"),
            "measured_velocity": self._joint_vector(measured_velocity_rad_s, "measured velocity"),
            "kp": self._joint_vector(kp, "joint kp"),
            "kd": self._joint_vector(kd, "joint kd"),
            "feedforward": self._joint_vector(feedforward_torque_nm, "joint feedforward"),
        }
        if np.any(vectors["kp"] < 0.0) or np.any(vectors["kd"] < 0.0):
            raise ValueError("joint impedance gains must be non-negative")
        indices = {name: index for index, name in enumerate(self.policy_joint_names)}
        commands: dict[str, DamiaoMitCommand] = {}
        for joint_name, mapping in self.direct.items():
            index = indices[joint_name]
            commands[mapping.motor_name] = mapping.joint_impedance_to_drive_command(
                vectors["desired_position"][index],
                vectors["desired_velocity"][index],
                vectors["kp"][index],
                vectors["kd"][index],
                vectors["feedforward"][index],
            )
        for mapping in self.differentials.values():
            pair_indices = [indices[name] for name in mapping.joint_names]
            pair_commands = mapping.joint_impedance_to_drive_commands(
                vectors["desired_position"][pair_indices],
                vectors["desired_velocity"][pair_indices],
                vectors["measured_position"][pair_indices],
                vectors["measured_velocity"][pair_indices],
                vectors["kp"][pair_indices],
                vectors["kd"][pair_indices],
                vectors["feedforward"][pair_indices],
            )
            commands.update(zip(mapping.motor_names, pair_commands, strict=True))
        if set(commands) != set(self.physical_motor_names):
            raise RuntimeError("impedance mapping did not produce the exact physical motor set")
        return commands

    def _motor_to_joint(self, values: Mapping[str, float], kind: str) -> Vector:
        motor = self._motor_values(values, f"motor {kind}")
        joint: dict[str, float] = {}
        for joint_name, mapping in self.direct.items():
            transform = getattr(mapping, f"drive_to_joint_{kind}")
            joint[joint_name] = float(transform(motor[mapping.motor_name]))
        for mapping in self.differentials.values():
            transform = getattr(mapping, f"drive_to_joint_{kind}")
            joint_values = transform([motor[name] for name in mapping.motor_names])
            joint.update(zip(mapping.joint_names, map(float, joint_values), strict=True))
        return np.asarray([joint[name] for name in self.policy_joint_names], dtype=np.float64)


def motor_map_from_hardware_config(
    hardware: Mapping[str, Any],
    policy_joint_names: Iterable[str],
    *,
    require_armable: bool = True,
) -> SpriteMotorMap:
    """Build the coordinate map, optionally for receive-only commissioning."""
    hardware_dict = dict(hardware)
    policy_order = tuple(policy_joint_names)
    if require_armable:
        if hardware_dict.get("configured") is not True:
            raise ValueError("hardware configured=true is required for a motor map")
        report = validate_hardware_inventory(hardware_dict, policy_order)
        if not report.valid:
            raise ValueError("hardware inventory is invalid: " + "; ".join(report.errors))
    elif len(hardware_dict.get("motor_map", {})) != 31:
        raise ValueError("receive-only motor map requires exactly 31 physical motors")
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
    differentials: dict[str, DifferentialPairDriveMap] = {}
    for name, joint_names in DIFFERENTIAL_PAIRS.items():
        pair_record = hardware_dict["differentials"][name]
        motor_names = tuple(pair_record["motor_names"])
        differentials[name] = DifferentialPairDriveMap(
            name=name,
            joint_names=joint_names,
            motor_names=motor_names,
            drive_zero_rad=np.asarray(
                [records[name]["motor_zero_rad"] for name in motor_names], dtype=np.float64
            ),
            encoder_sign=np.asarray(
                [records[name]["encoder_sign"] for name in motor_names], dtype=np.float64
            ),
            coupling=DifferentialPair(
                joint_to_motor_matrix=np.asarray(
                    pair_record["joint_to_motor_matrix"], dtype=np.float64
                ),
                motor_zero_rad=np.asarray(pair_record["motor_zero_rad"], dtype=np.float64),
            ),
        )
    result = SpriteMotorMap(policy_order, direct, differentials)
    if set(result.physical_motor_names) != set(records):
        raise ValueError("coordinate map does not exactly cover hardware motor_map")
    return result


# Backward-compatible import name; the implementation now also serves the head.
DifferentialAnkleDriveMap = DifferentialPairDriveMap
