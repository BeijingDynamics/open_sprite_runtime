"""Differential-ankle calibration and power-consistent transforms."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray


Vector2 = NDArray[np.float64]
Matrix2 = NDArray[np.float64]


def fit_differential_pair(
    joint_positions_rad: ArrayLike,
    motor_positions_rad: ArrayLike,
    *,
    maximum_rms_residual_rad: float = 0.01,
    maximum_condition_number: float = 100.0,
) -> dict[str, object]:
    """Fit ``q_motor = A @ q_joint + zero`` for a two-joint differential."""
    joints = np.asarray(joint_positions_rad, dtype=np.float64)
    motors = np.asarray(motor_positions_rad, dtype=np.float64)
    if joints.ndim != 2 or joints.shape[1] != 2 or motors.shape != joints.shape:
        raise ValueError("joint and motor samples must both have shape [N, 2]")
    if joints.shape[0] < 6:
        raise ValueError("at least six ankle calibration samples are required")
    if not np.isfinite(joints).all() or not np.isfinite(motors).all():
        raise ValueError("differential calibration samples must be finite")
    design = np.column_stack((joints, np.ones(joints.shape[0])))
    if int(np.linalg.matrix_rank(design)) != 3:
        raise ValueError("differential samples do not independently excite both joints")
    excitation_condition = float(np.linalg.cond(design))
    coefficients, _, _, _ = np.linalg.lstsq(design, motors, rcond=None)
    matrix = coefficients[:2, :].T
    zero = coefficients[2, :]
    determinant = float(np.linalg.det(matrix))
    if abs(determinant) < 1.0e-6:
        raise ValueError("fitted differential matrix is singular")
    predicted = joints @ matrix.T + zero
    residual = motors - predicted
    rms = np.sqrt(np.mean(np.square(residual), axis=0))
    maximum = np.max(np.abs(residual), axis=0)
    fitted_condition = float(np.linalg.cond(matrix))
    passed = bool(
        excitation_condition <= maximum_condition_number
        and fitted_condition <= maximum_condition_number
        and float(np.max(rms)) <= maximum_rms_residual_rad
    )
    return {
        "joint_to_motor_matrix": matrix.tolist(),
        "motor_zero_rad": zero.tolist(),
        "determinant": determinant,
        "excitation_condition_number": excitation_condition,
        "matrix_condition_number": fitted_condition,
        "rms_residual_rad": rms.tolist(),
        "max_abs_residual_rad": maximum.tolist(),
        "sample_count": joints.shape[0],
        "maximum_rms_residual_rad": maximum_rms_residual_rad,
        "maximum_condition_number": maximum_condition_number,
        "passed": passed,
    }


def _vector2(value: ArrayLike, name: str) -> Vector2:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (2,):
        raise ValueError(f"{name} must have shape (2,), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} contains a non-finite value")
    return result


@dataclass(frozen=True)
class DifferentialPair:
    """Linearized two-joint map ``q_motor = A @ q_joint + zero``.

    Joint order is the configured ``[pitch, roll]`` pair. Mechanism row signs and scale belong in
    ``A``; raw drive encoder polarity is applied by the outer motor-coordinate
    map. Both layers must be identified independently for left and right.
    """

    joint_to_motor_matrix: Matrix2
    motor_zero_rad: Vector2

    def __post_init__(self) -> None:
        matrix = np.asarray(self.joint_to_motor_matrix, dtype=np.float64)
        zero = _vector2(self.motor_zero_rad, "motor_zero_rad")
        if matrix.shape != (2, 2):
            raise ValueError(f"joint_to_motor_matrix must be 2x2, got {matrix.shape}")
        if not np.all(np.isfinite(matrix)):
            raise ValueError("joint_to_motor_matrix contains a non-finite value")
        if abs(float(np.linalg.det(matrix))) < 1.0e-8:
            raise ValueError("joint_to_motor_matrix is singular")
        object.__setattr__(self, "joint_to_motor_matrix", matrix)
        object.__setattr__(self, "motor_zero_rad", zero)

    @classmethod
    def ideal_symmetric(cls) -> "DifferentialPair":
        return cls(
            joint_to_motor_matrix=np.array([[1.0, 1.0], [1.0, -1.0]]),
            motor_zero_rad=np.zeros(2),
        )

    def joint_to_motor_position(self, joint_position: ArrayLike) -> Vector2:
        position = _vector2(joint_position, "joint_position")
        return self.joint_to_motor_matrix @ position + self.motor_zero_rad

    def motor_to_joint_position(self, motor_position: ArrayLike) -> Vector2:
        centered = _vector2(motor_position, "motor_position") - self.motor_zero_rad
        return np.linalg.solve(self.joint_to_motor_matrix, centered)

    def joint_to_motor_velocity(self, joint_velocity: ArrayLike) -> Vector2:
        return self.joint_to_motor_matrix @ _vector2(joint_velocity, "joint_velocity")

    def motor_to_joint_velocity(self, motor_velocity: ArrayLike) -> Vector2:
        return np.linalg.solve(
            self.joint_to_motor_matrix, _vector2(motor_velocity, "motor_velocity")
        )

    def joint_to_motor_torque(self, joint_torque: ArrayLike) -> Vector2:
        # A^-T preserves instantaneous power: tau_j^T qd_j == tau_m^T qd_m.
        return np.linalg.solve(
            self.joint_to_motor_matrix.T, _vector2(joint_torque, "joint_torque")
        )

    def motor_to_joint_torque(self, motor_torque: ArrayLike) -> Vector2:
        return self.joint_to_motor_matrix.T @ _vector2(motor_torque, "motor_torque")

    def motor_impedance_matrix(self, joint_gains: ArrayLike) -> Matrix2:
        gains = _vector2(joint_gains, "joint_gains")
        inverse = np.linalg.inv(self.joint_to_motor_matrix)
        return inverse.T @ np.diag(gains) @ inverse

    def diagonal_motor_gains(
        self, joint_gains: ArrayLike, tolerance: float = 1.0e-9
    ) -> Vector2:
        matrix = self.motor_impedance_matrix(joint_gains)
        off_diagonal = matrix - np.diag(np.diag(matrix))
        if float(np.max(np.abs(off_diagonal))) > tolerance:
            raise ValueError(
                "joint gains require cross-coupled motor impedance; embedded diagonal PD is insufficient"
            )
        return np.diag(matrix).copy()


# Backward-compatible names for downstream code using the original ankle-only API.
DifferentialAnkle = DifferentialPair
fit_differential_ankle = fit_differential_pair
