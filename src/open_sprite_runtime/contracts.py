"""Policy and multi-rate runtime contract validation."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any


EXPECTED_OBSERVATION_TERMS = (
    ("joint_pos", 31, 8),
    ("joint_vel", 31, 8),
    ("actions", 31, 8),
    ("base_ang_vel", 3, 8),
    ("projected_gravity", 3, 8),
    ("velocity_commands", 3, 0),
)
JOINT_VECTOR_FIELDS = (
    "default_joint_pos",
    "action_offset",
    "action_scale",
    "stiffness",
    "damping",
    "effort_limit",
    "velocity_limit",
)


def _observation_term_dim(term: dict[str, Any]) -> int:
    if "dim" in term:
        return int(term["dim"])
    shape = term.get("shape")
    if not isinstance(shape, list) or not shape:
        return -1
    result = 1
    for extent in shape:
        result *= int(extent)
    return result


@dataclass(frozen=True)
class RuntimeTiming:
    policy_hz: int
    state_hz: int
    motor_internal_hz: int
    policy_target_semantics: str = "zero_order_hold_for_20ms"

    def validate(self) -> None:
        for name, value in (
            ("policy_hz", self.policy_hz),
            ("state_hz", self.state_hz),
            ("motor_internal_hz", self.motor_internal_hz),
        ):
            if value <= 0:
                raise ValueError(f"{name} must be positive")
        if self.state_hz % self.policy_hz:
            raise ValueError("state_hz must be an integer multiple of policy_hz")
        if self.motor_internal_hz % self.state_hz:
            raise ValueError("motor_internal_hz must be an integer multiple of state_hz")
        if self.policy_target_semantics != "zero_order_hold_for_20ms":
            raise ValueError("policy target must use the qualified 20 ms zero-order hold")

    @property
    def state_updates_per_policy(self) -> int:
        return self.state_hz // self.policy_hz

    @property
    def interpolation_steps(self) -> int:
        """Backward-compatible alias; policy targets are not interpolated."""
        return self.state_updates_per_policy


@dataclass(frozen=True)
class PolicyContract:
    path: Path
    data: dict[str, Any]

    @classmethod
    def load(cls, path: str | Path) -> "PolicyContract":
        resolved = Path(path).expanduser().resolve()
        return cls(resolved, json.loads(resolved.read_text(encoding="utf-8")))

    @property
    def policy_hz(self) -> int:
        dt = float(self.data["policy_dt"])
        hz = round(1.0 / dt)
        if not math.isclose(hz * dt, 1.0, rel_tol=0.0, abs_tol=1.0e-9):
            raise ValueError(f"policy_dt does not represent an integer frequency: {dt}")
        return hz

    @property
    def maximum_history_length(self) -> int:
        return max(int(term["history_length"]) for term in self.data["observation_terms"])

    def _joint_vector(self, name: str, errors: list[str]) -> list[float]:
        value = self.data.get(name)
        if not isinstance(value, list) or len(value) != 31:
            errors.append(f"{name} must contain exactly 31 values")
            return []
        try:
            result = [float(item) for item in value]
        except (TypeError, ValueError):
            errors.append(f"{name} must contain only numeric values")
            return []
        if not all(math.isfinite(item) for item in result):
            errors.append(f"{name} contains a non-finite value")
            return []
        return result

    def validate_for_hardware(self, timing: RuntimeTiming) -> None:
        timing.validate()
        errors: list[str] = []
        if self.policy_hz != timing.policy_hz:
            errors.append(
                f"policy was trained at {self.policy_hz} Hz but runtime requests {timing.policy_hz} Hz"
            )
        if self.data.get("observation_has_horizontal_base_velocity") is not False:
            errors.append("actor observation must not contain horizontal base velocity")
        if self.data.get("observation_has_global_position") is not False:
            errors.append("actor observation must not contain global position")
        if self.data.get("observation_has_global_yaw") is not False:
            errors.append("actor observation must not contain global yaw")
        if int(self.data.get("action_dim", -1)) != 31:
            errors.append("expected 31 policy actions")
        if not math.isclose(float(self.data.get("physics_dt", -1.0)), 0.002, abs_tol=1.0e-12):
            errors.append("physics_dt must be 0.002 s (500 Hz)")
        decimation = int(self.data.get("decimation", -1))
        if decimation != timing.interpolation_steps:
            errors.append("decimation does not match state_hz / policy_hz")
        if not math.isclose(
            float(self.data.get("physics_dt", -1.0)) * decimation,
            float(self.data.get("policy_dt", -2.0)),
            abs_tol=1.0e-12,
        ):
            errors.append("policy_dt must equal physics_dt * decimation")

        joint_names = self.data.get("joint_names")
        if not isinstance(joint_names, list) or len(joint_names) != 31:
            errors.append("joint_names must contain exactly 31 entries")
        elif len(set(joint_names)) != 31 or not all(isinstance(name, str) for name in joint_names):
            errors.append("joint_names must be 31 unique strings")
        vectors = {name: self._joint_vector(name, errors) for name in JOINT_VECTOR_FIELDS}
        for name in ("action_scale", "effort_limit", "velocity_limit"):
            if vectors[name] and not all(value > 0.0 for value in vectors[name]):
                errors.append(f"{name} values must be positive")
        for name in ("stiffness", "damping"):
            if vectors[name] and not all(value >= 0.0 for value in vectors[name]):
                errors.append(f"{name} values must be non-negative")
        if self.data.get("action_formula") != (
            "target_joint_pos = action_offset + action_scale * policy_action"
        ):
            errors.append("unsupported action_formula")

        terms = self.data.get("observation_terms")
        if not isinstance(terms, list) or len(terms) != len(EXPECTED_OBSERVATION_TERMS):
            errors.append("unexpected observation term count")
        else:
            cursor = 0
            for term, (name, frame_dim, history_length) in zip(
                terms, EXPECTED_OBSERVATION_TERMS, strict=True
            ):
                expected_dim = frame_dim * max(history_length, 1)
                if term.get("name") != name:
                    errors.append(f"observation term order mismatch at {name}")
                if int(term.get("frame_dim", -1)) != frame_dim:
                    errors.append(f"{name} frame_dim mismatch")
                if int(term.get("history_length", -1)) != history_length:
                    errors.append(f"{name} history_length mismatch")
                if int(term.get("start", -1)) != cursor:
                    errors.append(f"{name} observation slice is not contiguous")
                cursor += expected_dim
                if int(term.get("end", -1)) != cursor or _observation_term_dim(term) != expected_dim:
                    errors.append(f"{name} observation dimension mismatch")
            if int(self.data.get("actor_observation_dim", -1)) != cursor:
                errors.append("actor_observation_dim does not match observation slices")

        if self.data.get("base_ang_vel_frame") != "root_link_local":
            errors.append("base angular velocity must use the root-link local frame")
        if self.data.get("projected_gravity_definition") != "R_world_to_base @ [0, 0, -1]":
            errors.append("unsupported projected-gravity definition")
        if self.data.get("quaternion_order") != "wxyz":
            errors.append("quaternion_order must be wxyz")
        if self.data.get("command_layout") != ["vx", "vy", "yaw_rate"]:
            errors.append("command_layout must be [vx, vy, yaw_rate]")

        ankle = self.data.get("physical_ankle_differential")
        if not isinstance(ankle, dict) or ankle.get("enabled") is not True:
            errors.append("physical differential-ankle contract is missing")
        else:
            expected = {
                "rated_torque_nm": 3.5,
                "peak_torque_nm": 12.5,
                "rated_speed_rad_s": 12.56,
                "no_load_speed_rad_s": 36.2,
                "joint_to_motor_ratio": 1.0,
            }
            for name, target in expected.items():
                try:
                    actual = float(ankle[name])
                except (KeyError, TypeError, ValueError):
                    errors.append(f"physical_ankle_differential.{name} is missing")
                    continue
                if not math.isclose(actual, target, rel_tol=0.0, abs_tol=1.0e-6):
                    errors.append(f"physical_ankle_differential.{name} mismatch")
        if errors:
            raise ValueError("; ".join(errors))

    def summary(self) -> dict[str, Any]:
        return {
            "policy_hz": self.policy_hz,
            "history_frames": self.maximum_history_length,
            "history_window_ms": 1000.0 * self.maximum_history_length / self.policy_hz,
            "observation_dim": int(self.data["actor_observation_dim"]),
            "action_dim": int(self.data["action_dim"]),
            "horizontal_base_velocity": bool(
                self.data["observation_has_horizontal_base_velocity"]
            ),
        }
