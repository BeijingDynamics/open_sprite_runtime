"""Measured-pose startup handoff for the frozen Sprite policy contract."""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .contracts import PolicyContract


Vector = NDArray[np.float64]


def _joint_vector(value: ArrayLike, name: str) -> Vector:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (31,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 31-vector")
    return result.copy()


class MeasuredPoseActionHandoff:
    """Blend from the measured-pose-equivalent action to policy actions.

    This preserves the startup behavior qualified in the MuJoCo deployment:
    the initial measured joint pose is converted through the frozen action
    affine transform, then smoothstep blended over complete policy ticks.
    """

    _SUPPORTED_MODE = "smoothstep_from_pose_equivalent_action"

    def __init__(self, contract: PolicyContract):
        mode = contract.data.get("deployment_handoff_mode")
        if mode != self._SUPPORTED_MODE:
            raise ValueError(f"unsupported deployment handoff mode: {mode!r}")
        if contract.data.get("deployment_initial_velocity_mode") != "zero":
            raise ValueError("deployment initial velocity mode must be zero")
        seconds = float(contract.data.get("deployment_handoff_seconds", math.nan))
        if not math.isfinite(seconds) or seconds < 0.0:
            raise ValueError("deployment_handoff_seconds must be finite and non-negative")
        self._offset = _joint_vector(contract.data["action_offset"], "action_offset")
        self._scale = _joint_vector(contract.data["action_scale"], "action_scale")
        if np.any(np.abs(self._scale) <= 1.0e-12):
            raise ValueError("action_scale values must be non-zero for measured-pose handoff")
        self.total_steps = max(0, int(round(seconds * contract.policy_hz)))
        self.completed_steps = 0
        self.initial_action: Vector | None = None

    @property
    def complete(self) -> bool:
        return self.completed_steps >= self.total_steps

    @property
    def progress(self) -> float:
        if self.total_steps == 0:
            return 1.0
        linear = min(1.0, self.completed_steps / self.total_steps)
        return linear * linear * (3.0 - 2.0 * linear)

    def blend(self, policy_action: ArrayLike, measured_joint_position: ArrayLike) -> Vector:
        action = _joint_vector(policy_action, "policy_action")
        measured = _joint_vector(measured_joint_position, "measured_joint_position")
        if self.initial_action is None:
            self.initial_action = (measured - self._offset) / self._scale
        if self.total_steps == 0 or self.complete:
            return action
        self.completed_steps += 1
        blend = self.progress
        return (1.0 - blend) * self.initial_action + blend * action

