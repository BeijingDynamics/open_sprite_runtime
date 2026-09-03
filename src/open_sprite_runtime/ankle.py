"""Differential-ankle kinematics and power-consistent torque transforms."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray


Vector2 = NDArray[np.float64]
Matrix2 = NDArray[np.float64]


def _vector2(value: ArrayLike, name: str) -> Vector2:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (2,):
        raise ValueError(f"{name} must have shape (2,), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} contains a non-finite value")
    return result


@dataclass(frozen=True)
class DifferentialAnkle:
    """Linearized map ``q_motor = A @ q_joint + zero``.

    Joint order is ``[pitch, roll]``. Motor signs and scale belong in ``A``;
    they must be identified independently for the left and right mechanisms.
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
    def ideal_symmetric(cls) -> "DifferentialAnkle":
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
