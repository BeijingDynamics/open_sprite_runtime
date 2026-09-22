"""Fail-closed joint target projection for protected hardware commissioning."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .multirate_control import JointImpedanceTarget


def parse_joint_gain_multiplier_overrides(
    joint_names: tuple[str, ...], specifications: list[str] | tuple[str, ...]
) -> np.ndarray:
    """Parse unique ``joint=value`` commissioning gain multipliers."""
    result = np.ones(len(joint_names), dtype=np.float64)
    indices = {name: index for index, name in enumerate(joint_names)}
    seen: set[str] = set()
    for specification in specifications:
        name, separator, raw_value = specification.partition("=")
        if not separator or name not in indices:
            raise ValueError(f"invalid joint gain multiplier: {specification}")
        if name in seen:
            raise ValueError(f"duplicate joint gain multiplier: {name}")
        value = float(raw_value)
        if not math.isfinite(value) or not 0.0 < value <= 1.0:
            raise ValueError(f"joint gain multiplier for {name} must be in (0, 1]")
        result[indices[name]] = value
        seen.add(name)
    return result


@dataclass
class ConsecutiveClampWatchdog:
    """Abort when selected raw targets remain materially beyond soft limits."""

    joint_names: tuple[str, ...]
    watched_joint_names: tuple[str, ...]
    minimum_overshoot_rad: float
    maximum_consecutive_ticks: int

    def __post_init__(self) -> None:
        indices = {name: index for index, name in enumerate(self.joint_names)}
        if not self.watched_joint_names or len(set(self.watched_joint_names)) != len(
            self.watched_joint_names
        ):
            raise ValueError("clamp watchdog joints must be nonempty and unique")
        unknown = sorted(set(self.watched_joint_names) - set(indices))
        if unknown:
            raise ValueError(f"unknown clamp watchdog joints: {', '.join(unknown)}")
        if not math.isfinite(self.minimum_overshoot_rad) or not (
            0.0 < self.minimum_overshoot_rad <= 0.2
        ):
            raise ValueError("clamp watchdog overshoot must be in (0, 0.2] rad")
        if not 1 <= self.maximum_consecutive_ticks <= 100:
            raise ValueError("clamp watchdog consecutive ticks must be in [1, 100]")
        self._indices = {name: indices[name] for name in self.watched_joint_names}
        self._consecutive = {name: 0 for name in self.watched_joint_names}
        self.maximum_observed_consecutive_ticks = {
            name: 0 for name in self.watched_joint_names
        }

    def update(self, raw_position_rad: Any, projected_position_rad: Any) -> None:
        raw = np.asarray(raw_position_rad, dtype=np.float64)
        projected = np.asarray(projected_position_rad, dtype=np.float64)
        if (
            raw.shape != (len(self.joint_names),)
            or projected.shape != raw.shape
            or not np.isfinite(raw).all()
            or not np.isfinite(projected).all()
        ):
            raise ValueError("clamp watchdog positions must be finite joint vectors")
        for name, index in self._indices.items():
            overshoot = abs(float(raw[index] - projected[index]))
            self._consecutive[name] = (
                self._consecutive[name] + 1
                if overshoot > self.minimum_overshoot_rad
                else 0
            )
            self.maximum_observed_consecutive_ticks[name] = max(
                self.maximum_observed_consecutive_ticks[name], self._consecutive[name]
            )
            if self._consecutive[name] >= self.maximum_consecutive_ticks:
                raise RuntimeError(
                    f"clamp watchdog tripped for {name}: overshoot={overshoot:.6f} rad "
                    f"for {self._consecutive[name]} consecutive ticks"
                )

    def report(self) -> dict[str, Any]:
        return {
            "enabled": True,
            "watched_joint_names": list(self.watched_joint_names),
            "minimum_overshoot_rad": self.minimum_overshoot_rad,
            "maximum_consecutive_ticks": self.maximum_consecutive_ticks,
            "maximum_observed_consecutive_ticks_by_joint": dict(
                self.maximum_observed_consecutive_ticks
            ),
        }


@dataclass
class ProtectedTargetProjector:
    """Project policy targets into reviewed joint limits and a startup gain tier."""

    joint_names: tuple[str, ...]
    lower_rad: np.ndarray
    upper_rad: np.ndarray
    gain_scale: float
    maximum_embedded_kd: float = 3.0
    joint_gain_multipliers: np.ndarray | None = None

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
        if self.joint_gain_multipliers is None:
            self.joint_gain_multipliers = np.ones(31, dtype=np.float64)
        else:
            self.joint_gain_multipliers = np.asarray(
                self.joint_gain_multipliers, dtype=np.float64
            )
        if (
            self.joint_gain_multipliers.shape != (31,)
            or not np.isfinite(self.joint_gain_multipliers).all()
            or np.any(self.joint_gain_multipliers <= 0.0)
            or np.any(self.joint_gain_multipliers > 1.0)
        ):
            raise ValueError("joint_gain_multipliers must be a finite (0, 1] 31-vector")
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
        joint_gain_multipliers: np.ndarray | None = None,
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
            joint_gain_multipliers,
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

        scale = self.gain_scale * self.joint_gain_multipliers
        projected_kp = scale * kp
        projected_kd = scale * kd
        projected_feedforward = scale * feedforward
        if np.any(projected_kd > self.maximum_embedded_kd):
            raise ValueError("projected Kd exceeds the qualified Damiao limit")
        return JointImpedanceTarget(
            position_rad=bounded,
            velocity_rad_s=velocity.copy(),
            kp=projected_kp,
            kd=projected_kd,
            feedforward_torque_nm=projected_feedforward,
        )

    def project_position(self, position_rad: Any) -> np.ndarray:
        """Clamp a measured or commanded joint pose without changing counters."""
        position = np.asarray(position_rad, dtype=np.float64)
        if position.shape != (31,) or not np.isfinite(position).all():
            raise ValueError("protected position must be a finite 31-vector")
        return np.clip(position, self.lower_rad, self.upper_rad)

    def require_position_within_limits(
        self, position_rad: Any, *, label: str = "position"
    ) -> np.ndarray:
        """Return a validated pose or identify every joint outside soft limits."""
        position = np.asarray(position_rad, dtype=np.float64)
        if position.shape != (31,) or not np.isfinite(position).all():
            raise ValueError(f"{label} must be a finite 31-vector")
        outside = np.flatnonzero(
            (position < self.lower_rad) | (position > self.upper_rad)
        )
        if outside.size:
            details = ", ".join(
                f"{self.joint_names[int(index)]}={position[index]:+.6f} "
                f"not in [{self.lower_rad[index]:+.6f}, {self.upper_rad[index]:+.6f}]"
                for index in outside
            )
            raise ValueError(f"{label} is outside reviewed soft limits: {details}")
        return position.copy()

    def report(self) -> dict[str, Any]:
        return {
            "enabled": True,
            "gain_scale": self.gain_scale,
            "maximum_embedded_kd": self.maximum_embedded_kd,
            "joint_gain_multiplier_overrides": {
                name: float(self.joint_gain_multipliers[index])
                for index, name in enumerate(self.joint_names)
                if self.joint_gain_multipliers[index] != 1.0
            },
            "maximum_raw_position_overshoot_rad": self.maximum_raw_position_overshoot_rad,
            "clamp_count_by_joint": self.clamp_count_by_joint,
        }
