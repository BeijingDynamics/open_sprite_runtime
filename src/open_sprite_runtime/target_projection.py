"""Fail-closed joint target projection for protected hardware commissioning."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .multirate_control import JointImpedanceTarget


@dataclass
class ProtectedTargetProjector:
    """Project policy targets into reviewed joint limits and a startup gain tier."""

    joint_names: tuple[str, ...]
    lower_rad: np.ndarray
    upper_rad: np.ndarray
    gain_scale: float
    maximum_embedded_kd: float = 3.0

    def __post_init__(self) -> None:
        if len(self.joint_names) != 31 or len(set(self.joint_names)) != 31:
            raise ValueError("joint_names must contain 31 unique joints")
        self.lower_rad = np.asarray(self.lower_rad, dtype=np.float64)
        self.upper_rad = np.asarray(self.upper_rad, dtype=np.float64)
        if (
            self.lower_rad.shape != (31,)
            or self.upper_rad.shape != (31,)
            or not np.isfinite(self.lower_rad).all()
            or not np.isfinite(self.upper_rad).all()
            or np.any(self.lower_rad >= self.upper_rad)
        ):
            raise ValueError("joint soft limits must be finite ordered 31-vectors")
        if not math.isfinite(self.gain_scale) or not 0.0 < self.gain_scale <= 1.0:
            raise ValueError("gain_scale must be finite and in (0, 1]")
        if not math.isfinite(self.maximum_embedded_kd) or self.maximum_embedded_kd <= 0.0:
            raise ValueError("maximum_embedded_kd must be finite and positive")
        self.clamp_count_by_joint = {name: 0 for name in self.joint_names}
        self.maximum_raw_position_overshoot_rad = 0.0

    @classmethod
    def from_limit_report(
        cls,
        joint_names: tuple[str, ...],
        report: str | Path | Mapping[str, Any],
        *,
        gain_scale: float,
        maximum_embedded_kd: float = 3.0,
    ) -> "ProtectedTargetProjector":
        if isinstance(report, (str, Path)):
            data = json.loads(Path(report).read_text(encoding="utf-8"))
        else:
            data = dict(report)
        limits = data.get("joint_limits")
        if not isinstance(limits, Mapping) or set(limits) != set(joint_names):
            raise ValueError("limit report must exactly cover the policy joint order")
        lower: list[float] = []
        upper: list[float] = []
        for name in joint_names:
            values = limits[name].get("soft_limit_rad_candidate")
            if not isinstance(values, list) or len(values) != 2:
                raise ValueError(f"missing soft-limit candidate for {name}")
            lower.append(float(values[0]))
            upper.append(float(values[1]))
        return cls(
            joint_names,
            np.asarray(lower),
            np.asarray(upper),
            gain_scale,
            maximum_embedded_kd,
        )

    def project(self, target: JointImpedanceTarget) -> JointImpedanceTarget:
        position = np.asarray(target.position_rad, dtype=np.float64)
        velocity = np.asarray(target.velocity_rad_s, dtype=np.float64)
        kp = np.asarray(target.kp, dtype=np.float64)
        kd = np.asarray(target.kd, dtype=np.float64)
        feedforward = np.asarray(target.feedforward_torque_nm, dtype=np.float64)
        vectors = (position, velocity, kp, kd, feedforward)
        if any(value.shape != (31,) or not np.isfinite(value).all() for value in vectors):
            raise ValueError("protected target vectors must be finite 31-vectors")
        if np.any(kp < 0.0) or np.any(kd < 0.0):
            raise ValueError("protected target Kp/Kd must be non-negative")

        bounded = np.clip(position, self.lower_rad, self.upper_rad)
        overshoot = np.maximum(self.lower_rad - position, position - self.upper_rad)
        self.maximum_raw_position_overshoot_rad = max(
            self.maximum_raw_position_overshoot_rad,
            float(np.max(np.maximum(overshoot, 0.0))),
        )
        for index in np.flatnonzero(bounded != position):
            self.clamp_count_by_joint[self.joint_names[int(index)]] += 1

        projected_kp = self.gain_scale * kp
        projected_kd = self.gain_scale * kd
        projected_feedforward = self.gain_scale * feedforward
        if np.any(projected_kd > self.maximum_embedded_kd):
            raise ValueError("projected Kd exceeds the qualified Damiao limit")
        return JointImpedanceTarget(
            position_rad=bounded,
            velocity_rad_s=velocity.copy(),
            kp=projected_kp,
            kd=projected_kd,
            feedforward_torque_nm=projected_feedforward,
        )

    def report(self) -> dict[str, Any]:
        return {
            "enabled": True,
            "gain_scale": self.gain_scale,
            "maximum_embedded_kd": self.maximum_embedded_kd,
            "maximum_raw_position_overshoot_rad": self.maximum_raw_position_overshoot_rad,
            "clamp_count_by_joint": self.clamp_count_by_joint,
        }
