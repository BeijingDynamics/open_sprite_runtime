"""Physical pose capture and gain ramp for protected commissioning."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .multirate_control import JointImpedanceTarget


Vector = NDArray[np.float64]


def _vector(value: ArrayLike, name: str) -> Vector:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (31,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must be a finite 31-vector")
    return result.copy()


@dataclass
class PhysicalStartupRamp:
    """Hold a captured pose, then smoothly admit a protected policy target."""

    policy_hz: float
    hold_seconds: float
    ramp_seconds: float

    def __post_init__(self) -> None:
        for name in ("policy_hz", "hold_seconds", "ramp_seconds"):
            value = float(getattr(self, name))
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            setattr(self, name, value)
        if self.policy_hz <= 0.0:
            raise ValueError("policy_hz must be positive")
        if self.hold_seconds < 0.0 or self.ramp_seconds <= 0.0:
            raise ValueError("hold_seconds must be non-negative and ramp_seconds positive")
        self.hold_steps = int(math.ceil(self.hold_seconds * self.policy_hz))
        self.ramp_steps = max(1, int(math.ceil(self.ramp_seconds * self.policy_hz)))
        self.completed_steps = 0
        self.initial_position_rad: Vector | None = None
        self.last_alpha = 0.0
        self.maximum_output_position_delta_rad = 0.0
        self._previous_output: Vector | None = None

    def _alpha(self) -> float:
        if self.completed_steps < self.hold_steps:
            return 0.0
        ramp_step = self.completed_steps - self.hold_steps + 1
        linear = min(1.0, ramp_step / self.ramp_steps)
        return linear * linear * (3.0 - 2.0 * linear)

    @property
    def complete(self) -> bool:
        return self.completed_steps >= self.hold_steps + self.ramp_steps

    def apply(
        self,
        target: JointImpedanceTarget,
        measured_position_rad: ArrayLike,
    ) -> JointImpedanceTarget:
        measured = _vector(measured_position_rad, "measured_position_rad")
        if self.initial_position_rad is None:
            self.initial_position_rad = measured
        alpha = self._alpha()
        position = (1.0 - alpha) * self.initial_position_rad + alpha * target.position_rad
        if self._previous_output is not None:
            self.maximum_output_position_delta_rad = max(
                self.maximum_output_position_delta_rad,
                float(np.max(np.abs(position - self._previous_output))),
            )
        self._previous_output = position.copy()
        self.last_alpha = alpha
        self.completed_steps += 1
        return JointImpedanceTarget(
            position_rad=position,
            velocity_rad_s=alpha * target.velocity_rad_s,
            kp=alpha * target.kp,
            kd=alpha * target.kd,
            feedforward_torque_nm=alpha * target.feedforward_torque_nm,
        )

    def report(self) -> dict[str, Any]:
        return {
            "enabled": True,
            "policy_hz": self.policy_hz,
            "hold_seconds": self.hold_seconds,
            "ramp_seconds": self.ramp_seconds,
            "hold_steps": self.hold_steps,
            "ramp_steps": self.ramp_steps,
            "completed_steps": self.completed_steps,
            "complete": self.complete,
            "last_alpha": self.last_alpha,
            "maximum_output_position_delta_rad": self.maximum_output_position_delta_rad,
        }
