"""Multi-rate joint control for the Sprite0825 physical motor bank."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .damiao import (
    DamiaoMitCommand,
    DamiaoMitCommandProfile,
    DamiaoMitState,
    EncodedDamiaoMitCommand,
)
from .motor_mapping import SpriteMotorMap


Vector = NDArray[np.float64]
ANKLE_PAIR_NAMES = ("left_ankle", "right_ankle")


def _joint_vector(value: ArrayLike, name: str) -> Vector:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (31,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 31-vector")
    return result.copy()


@dataclass(frozen=True)
class JointImpedanceTarget:
    position_rad: Vector
    velocity_rad_s: Vector
    kp: Vector
    kd: Vector
    feedforward_torque_nm: Vector

    def __post_init__(self) -> None:
        for field in (
            "position_rad",
            "velocity_rad_s",
            "kp",
            "kd",
            "feedforward_torque_nm",
        ):
            object.__setattr__(self, field, _joint_vector(getattr(self, field), field))
        if np.any(self.kp < 0.0) or np.any(self.kd < 0.0):
            raise ValueError("joint impedance gains must be non-negative")


@dataclass(frozen=True)
class MultiRateCommandFrame:
    state_tick: int
    ankle_commands: Mapping[str, DamiaoMitCommand]
    other_motor_commands: Mapping[str, DamiaoMitCommand]
    ankle_joint_torque_nm: Mapping[str, tuple[float, float]]
    saturated_ankle_joints: tuple[str, ...]

    @property
    def updates_other_motors(self) -> bool:
        return bool(self.other_motor_commands)


@dataclass(frozen=True)
class EncodedMultiRateCommandFrame:
    state_tick: int
    ankle_commands: tuple[EncodedDamiaoMitCommand, ...]
    other_motor_commands: tuple[EncodedDamiaoMitCommand, ...]


def encode_multirate_command_frame(
    mapping: SpriteMotorMap,
    profiles: tuple[DamiaoMitCommandProfile, ...],
    frame: MultiRateCommandFrame,
    measured_motor_states: Mapping[str, DamiaoMitState],
) -> EncodedMultiRateCommandFrame:
    """Apply all dynamic command envelopes before any frame reaches transport."""
    profile_map = {profile.endpoint.motor_name: profile for profile in profiles}
    expected_all = set(mapping.physical_motor_names)
    if set(profile_map) != expected_all or len(profile_map) != len(profiles):
        raise ValueError("command profiles must exactly cover all 31 physical motors")
    if set(measured_motor_states) != expected_all:
        raise ValueError("measured motor states must exactly cover all 31 physical motors")
    expected_ankles = {
        motor
        for pair_name in ANKLE_PAIR_NAMES
        for motor in mapping.differentials[pair_name].motor_names
    }
    if set(frame.ankle_commands) != expected_ankles:
        raise ValueError("frame must contain exactly the four physical ankle motors")
    expected_other = expected_all - expected_ankles if frame.updates_other_motors else set()
    if set(frame.other_motor_commands) != expected_other:
        raise ValueError("frame contains an invalid 50 Hz other-motor command set")

    ankle = tuple(
        profile_map[name].encode(frame.ankle_commands[name], measured_motor_states[name])
        for name in sorted(expected_ankles)
    )
    other = tuple(
        profile_map[name].encode(
            frame.other_motor_commands[name], measured_motor_states[name]
        )
        for name in sorted(expected_other)
    )
    return EncodedMultiRateCommandFrame(frame.state_tick, ankle, other)


class SpriteMultiRateController:
    """Run host ankle torque PD at 500 Hz and all other motor PD at 50 Hz.

    The policy target is held between policy ticks. Ankle pitch/roll generalized
    torque is computed in the serial URDF joint coordinates and transformed to
    the two physical motors with the calibrated linear transmission Jacobian.
    The ankle MIT commands use Kp=Kd=0 and feed-forward torque only.
    """

    def __init__(
        self,
        mapping: SpriteMotorMap,
        joint_effort_limit_nm: ArrayLike,
        *,
        policy_hz: int = 50,
        state_hz: int = 500,
    ):
        if policy_hz <= 0 or state_hz <= 0 or state_hz % policy_hz:
            raise ValueError("state_hz must be a positive integer multiple of policy_hz")
        if policy_hz != 50 or state_hz != 500:
            raise ValueError("Sprite0825 deployment requires policy_hz=50 and state_hz=500")
        limits = _joint_vector(joint_effort_limit_nm, "joint_effort_limit_nm")
        if np.any(limits <= 0.0):
            raise ValueError("joint effort limits must be positive")
        self.mapping = mapping
        self.joint_effort_limit_nm = limits
        self.policy_hz = policy_hz
        self.state_hz = state_hz
        self.state_ticks_per_policy = state_hz // policy_hz
        self._indices = {
            name: index for index, name in enumerate(mapping.policy_joint_names)
        }
        self._ankle_motor_names = frozenset(
            motor
            for pair_name in ANKLE_PAIR_NAMES
            for motor in mapping.differentials[pair_name].motor_names
        )
        self._target: JointImpedanceTarget | None = None
        self._state_tick = 0

    def latch_policy_target(self, target: JointImpedanceTarget) -> None:
        self._target = target

    def _measured_joint_state(
        self, measured_motor_states: Mapping[str, DamiaoMitState]
    ) -> tuple[Vector, Vector]:
        expected = set(self.mapping.physical_motor_names)
        if set(measured_motor_states) != expected:
            raise ValueError("measured motor states must exactly cover all 31 motors")
        positions = {
            name: state.position_rad for name, state in measured_motor_states.items()
        }
        velocities = {
            name: state.velocity_rad_s for name, state in measured_motor_states.items()
        }
        return (
            self.mapping.motor_to_joint_positions(positions),
            self.mapping.motor_to_joint_velocities(velocities),
        )

    def _ankle_commands(
        self,
        measured_motor_states: Mapping[str, DamiaoMitState],
        measured_joint_position: Vector,
        measured_joint_velocity: Vector,
    ) -> tuple[
        dict[str, DamiaoMitCommand],
        dict[str, tuple[float, float]],
        tuple[str, ...],
    ]:
        if self._target is None:
            raise RuntimeError("a policy target must be latched before stepping")
        commands: dict[str, DamiaoMitCommand] = {}
        joint_torques: dict[str, tuple[float, float]] = {}
        saturated: list[str] = []
        for pair_name in ANKLE_PAIR_NAMES:
            pair = self.mapping.differentials[pair_name]
            indices = np.asarray([self._indices[name] for name in pair.joint_names])
            raw_torque = (
                self._target.kp[indices]
                * (self._target.position_rad[indices] - measured_joint_position[indices])
                + self._target.kd[indices]
                * (self._target.velocity_rad_s[indices] - measured_joint_velocity[indices])
                + self._target.feedforward_torque_nm[indices]
            )
            limits = self.joint_effort_limit_nm[indices]
            limited_torque = np.clip(raw_torque, -limits, limits)
            for offset, joint_name in enumerate(pair.joint_names):
                if not np.isclose(raw_torque[offset], limited_torque[offset]):
                    saturated.append(joint_name)
            motor_torque = pair.joint_to_drive_torque(limited_torque)
            joint_torques[pair_name] = tuple(map(float, limited_torque))
            for index, motor_name in enumerate(pair.motor_names):
                state = measured_motor_states[motor_name]
                commands[motor_name] = DamiaoMitCommand(
                    position_rad=state.position_rad,
                    velocity_rad_s=0.0,
                    kp=0.0,
                    kd=0.0,
                    feedforward_torque_nm=float(motor_torque[index]),
                )
        return commands, joint_torques, tuple(saturated)

    def step(
        self, measured_motor_states: Mapping[str, DamiaoMitState]
    ) -> MultiRateCommandFrame:
        if self._target is None:
            raise RuntimeError("a policy target must be latched before stepping")
        measured_position, measured_velocity = self._measured_joint_state(
            measured_motor_states
        )
        ankle_commands, ankle_torque, saturated = self._ankle_commands(
            measured_motor_states, measured_position, measured_velocity
        )
        other_commands: dict[str, DamiaoMitCommand] = {}
        if self._state_tick % self.state_ticks_per_policy == 0:
            all_commands = self.mapping.joint_impedance_to_motor_commands(
                self._target.position_rad,
                self._target.velocity_rad_s,
                measured_position,
                measured_velocity,
                self._target.kp,
                self._target.kd,
                self._target.feedforward_torque_nm,
            )
            other_commands = {
                name: command
                for name, command in all_commands.items()
                if name not in self._ankle_motor_names
            }
        frame = MultiRateCommandFrame(
            state_tick=self._state_tick,
            ankle_commands=ankle_commands,
            other_motor_commands=other_commands,
            ankle_joint_torque_nm=ankle_torque,
            saturated_ankle_joints=saturated,
        )
        self._state_tick += 1
        return frame
