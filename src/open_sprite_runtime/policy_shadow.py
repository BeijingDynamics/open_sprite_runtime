"""Live policy inference and command calculation without actuator transmission."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import math
import time
from typing import Any, Mapping

import numpy as np

from .contracts import PolicyContract
from .damiao import DamiaoFeedback, DamiaoMitState
from .handoff import MeasuredPoseActionHandoff
from .imu import ImuMount, RawImuSample, transform_pelvis_sample
from .motor_mapping import SpriteMotorMap
from .multirate_control import JointImpedanceTarget, SpriteMultiRateController
from .shadow import load_actor
from .yahboom_imu import YahboomPacket, YahboomQuaternion, YahboomRawImu


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)]


class PolicyObservationHistory:
    """Build the frozen Isaac Lab actor observation in contract order."""

    _HISTORY_TERMS = (
        "joint_pos",
        "joint_vel",
        "actions",
        "base_ang_vel",
        "projected_gravity",
    )

    def __init__(self, contract: PolicyContract):
        self.contract = contract
        self.default_joint_pos = np.asarray(
            contract.data["default_joint_pos"], dtype=np.float32
        )
        self._terms = {term["name"]: term for term in contract.data["observation_terms"]}
        self._history: dict[str, deque[np.ndarray]] = {
            name: deque(maxlen=int(self._terms[name]["history_length"]))
            for name in self._HISTORY_TERMS
        }

    def _frame_values(
        self,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        previous_action: np.ndarray,
        base_ang_vel: np.ndarray,
        projected_gravity: np.ndarray,
    ) -> dict[str, np.ndarray]:
        values = {
            "joint_pos": np.asarray(joint_pos, dtype=np.float32) - self.default_joint_pos,
            "joint_vel": np.asarray(joint_vel, dtype=np.float32),
            "actions": np.asarray(previous_action, dtype=np.float32),
            "base_ang_vel": np.asarray(base_ang_vel, dtype=np.float32),
            "projected_gravity": np.asarray(projected_gravity, dtype=np.float32),
        }
        expected = {"joint_pos": 31, "joint_vel": 31, "actions": 31,
                    "base_ang_vel": 3, "projected_gravity": 3}
        for name, value in values.items():
            if value.shape != (expected[name],) or not np.isfinite(value).all():
                raise ValueError(f"{name} must be a finite {expected[name]}-vector")
        return values

    def append(
        self,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        previous_action: np.ndarray,
        base_ang_vel: np.ndarray,
        projected_gravity: np.ndarray,
    ) -> None:
        values = self._frame_values(
            joint_pos, joint_vel, previous_action, base_ang_vel, projected_gravity
        )
        for name, value in values.items():
            history = self._history[name]
            if not history:
                history.extend(value.copy() for _ in range(history.maxlen or 0))
            else:
                history.append(value.copy())

    def observation(self, command: np.ndarray) -> np.ndarray:
        command = np.asarray(command, dtype=np.float32)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError("velocity command must be a finite [vx, vy, yaw_rate] vector")
        if any(not history for history in self._history.values()):
            raise RuntimeError("observation history is not initialized")
        pieces = [
            np.concatenate(tuple(self._history[name]), dtype=np.float32)
            for name in self._HISTORY_TERMS
        ]
        pieces.append(command)
        result = np.concatenate(pieces, dtype=np.float32)
        expected = int(self.contract.data["actor_observation_dim"])
        if result.shape != (expected,):
            raise RuntimeError(
                f"assembled observation has shape {result.shape}, expected {expected}"
            )
        return result


@dataclass(frozen=True)
class PolicyShadowReport:
    inference_backend: str
    command_vx_vy_yaw_rate: tuple[float, float, float]
    policy_ticks: int
    state_ticks: int
    state_tick_deadline_misses: int
    policy_deadline_misses: int
    inference_mean_ms: float | None
    inference_p99_ms: float | None
    inference_max_ms: float | None
    state_tick_lateness_p99_ms: float | None
    state_tick_lateness_max_ms: float | None
    maximum_abs_action: float
    maximum_abs_action_delta: float
    maximum_abs_target_delta_rad: float
    maximum_abs_ankle_joint_torque_nm: dict[str, float]
    maximum_abs_ankle_motor_torque_nm: dict[str, float]
    ankle_joint_saturation_count: int
    maximum_abs_direct_motor_estimated_torque_nm: dict[str, float]
    maximum_direct_motor_torque_limit_ratio_by_motor: dict[str, float]
    maximum_direct_motor_peak_torque_ratio: float
    damiao_embedded_kd_limit: float
    maximum_embedded_kd_requested: float
    horizontal_base_velocity_present: bool
    serial_write_count: int
    nonzero_motor_command_tx_attempts: int
    automatic_enable_attempts: int
    automatic_mode_switch_attempts: int
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return (
            not self.errors
            and self.policy_ticks > 0
            and self.state_ticks > 0
            and self.policy_deadline_misses == 0
            and self.state_tick_lateness_p99_ms is not None
            and self.state_tick_lateness_p99_ms <= 2.0
            and self.state_tick_lateness_max_ms is not None
            and self.state_tick_lateness_max_ms <= 20.0
            and self.maximum_direct_motor_peak_torque_ratio <= 1.0
            and not self.horizontal_base_velocity_present
            and self.serial_write_count == 0
            and self.nonzero_motor_command_tx_attempts == 0
            and self.automatic_enable_attempts == 0
            and self.automatic_mode_switch_attempts == 0
        )

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "passed": self.passed}


class LivePolicyShadow:
    """Consume live feedback and execute the deployment math without command TX."""

    def __init__(
        self,
        contract: PolicyContract,
        hardware: Mapping[str, Any],
        mapping: SpriteMotorMap,
        command: tuple[float, float, float],
        *,
        clock_ns=time.perf_counter_ns,
    ):
        self.contract = contract
        self.hardware = hardware
        self.mapping = mapping
        self.command = np.asarray(command, dtype=np.float32)
        if self.command.shape != (3,) or not np.isfinite(self.command).all():
            raise ValueError("command must be finite [vx, vy, yaw_rate]")
        self.actor = load_actor(contract)
        self.history = PolicyObservationHistory(contract)
        self.handoff = MeasuredPoseActionHandoff(contract)
        self.mount = ImuMount.sprite0825_rear_pelvis()
        self.clock_ns = clock_ns
        self.motor_states: dict[str, DamiaoMitState] = {}
        self.latest_raw_imu: YahboomRawImu | None = None
        self.latest_quaternion: YahboomQuaternion | None = None
        self.previous_action = np.zeros(31, dtype=np.float32)
        self.previous_target: np.ndarray | None = None
        self.next_state_tick_ns: int | None = None
        self._policy_tick_divisor = 10
        self._inference_ms: list[float] = []
        self._state_lateness_ms: list[float] = []
        self._policy_ticks = 0
        self._state_ticks = 0
        self._state_deadline_misses = 0
        self._deadline_misses = 0
        self._max_action = 0.0
        self._max_action_delta = 0.0
        self._max_target_delta = 0.0
        self._ankle_joint_max = {"left_ankle": 0.0, "right_ankle": 0.0}
        self._ankle_motor_max = {
            name: 0.0
            for pair in mapping.ankles.values()
            for name in pair.motor_names
        }
        self._ankle_saturations = 0
        ankle_motors = set(self._ankle_motor_max)
        self._direct_torque_max = {
            name: 0.0 for name in mapping.physical_motor_names if name not in ankle_motors
        }
        self._direct_torque_ratio = {name: 0.0 for name in self._direct_torque_max}
        self._max_direct_peak_ratio = 0.0
        self._max_embedded_kd = 0.0
        self._errors: list[str] = []

        kd_limit = float(hardware["controller"]["damiao_embedded_kd_max"])
        contract_kd = np.asarray(contract.data["damping"], dtype=np.float64)
        # The physical Damiao drive limit is authoritative. Ankle Kd is host-side.
        self.kd = np.minimum(contract_kd, kd_limit)
        self.kp = np.asarray(contract.data["stiffness"], dtype=np.float64)
        self.effort_limit = np.asarray(contract.data["effort_limit"], dtype=np.float64)
        self.action_offset = np.asarray(contract.data["action_offset"], dtype=np.float64)
        self.action_scale = np.asarray(contract.data["action_scale"], dtype=np.float64)
        self.controller = SpriteMultiRateController(mapping, self.effort_limit)

    def update_feedback(self, feedback: DamiaoFeedback) -> None:
        self.motor_states[feedback.motor_name] = DamiaoMitState(
            feedback.position_rad, feedback.velocity_rad_s
        )

    def update_imu(self, packet: YahboomPacket) -> None:
        if isinstance(packet, YahboomRawImu):
            self.latest_raw_imu = packet
        elif isinstance(packet, YahboomQuaternion):
            self.latest_quaternion = packet

    @property
    def ready(self) -> bool:
        return (
            set(self.motor_states) == set(self.mapping.physical_motor_names)
            and self.latest_raw_imu is not None
            and self.latest_quaternion is not None
        )

    def _joint_and_imu_state(self, now_ns: int):
        positions = {name: state.position_rad for name, state in self.motor_states.items()}
        velocities = {name: state.velocity_rad_s for name, state in self.motor_states.items()}
        joint_pos = self.mapping.motor_to_joint_positions(positions)
        joint_vel = self.mapping.motor_to_joint_velocities(velocities)
        raw = self.latest_raw_imu
        quaternion = self.latest_quaternion
        assert raw is not None and quaternion is not None
        pelvis = transform_pelvis_sample(
            RawImuSample(
                monotonic_ns=now_ns,
                angular_velocity_sensor_rad_s=raw.angular_velocity_rad_s,
                linear_acceleration_sensor_m_s2=raw.acceleration_m_s2,
                orientation_sensor_to_world_wxyz=quaternion.wxyz,
                magnetic_field_sensor=raw.magnetic_field_ut,
            ),
            self.mount,
        )
        return joint_pos, joint_vel, pelvis

    def _run_policy(self, now_ns: int) -> None:
        joint_pos, joint_vel, pelvis = self._joint_and_imu_state(now_ns)
        self.history.append(
            joint_pos,
            joint_vel,
            self.previous_action,
            pelvis.angular_velocity_body_rad_s,
            pelvis.projected_gravity_body,
        )
        observation = self.history.observation(self.command)
        started = self.clock_ns()
        raw_action = self.actor.run(observation)
        elapsed_ms = (self.clock_ns() - started) / 1.0e6
        if raw_action.shape != (31,):
            raise ValueError(f"policy action has shape {raw_action.shape}, expected (31,)")
        action = self.handoff.blend(raw_action, joint_pos)
        self._inference_ms.append(elapsed_ms)
        self._deadline_misses += int(elapsed_ms > 20.0)
        self._max_action = max(self._max_action, float(np.max(np.abs(action))))
        self._max_action_delta = max(
            self._max_action_delta, float(np.max(np.abs(action - self.previous_action)))
        )
        target = self.action_offset + self.action_scale * action
        if self.previous_target is not None:
            self._max_target_delta = max(
                self._max_target_delta,
                float(np.max(np.abs(target - self.previous_target))),
            )
        self.controller.latch_policy_target(
            JointImpedanceTarget(
                position_rad=target,
                velocity_rad_s=np.zeros(31),
                kp=self.kp,
                kd=self.kd,
                feedforward_torque_nm=np.zeros(31),
            )
        )
        self.previous_action = action.copy()
        self.previous_target = target.copy()
        self._policy_ticks += 1

    def infer_policy_target(self, now_ns: int) -> JointImpedanceTarget:
        """Run exactly one actor tick from the latest complete hardware state."""
        if not self.ready:
            raise RuntimeError("complete motor and IMU state is required for policy inference")
        self._run_policy(int(now_ns))
        assert self.previous_target is not None
        return JointImpedanceTarget(
            position_rad=self.previous_target.copy(),
            velocity_rad_s=np.zeros(31),
            kp=self.kp.copy(),
            kd=self.kd.copy(),
            feedforward_torque_nm=np.zeros(31),
        )

    def _audit_command_frame(self, frame) -> None:
        for pair_name, torques in frame.ankle_joint_torque_nm.items():
            self._ankle_joint_max[pair_name] = max(
                self._ankle_joint_max[pair_name], max(map(abs, torques))
            )
        for motor_name, command in frame.ankle_commands.items():
            self._ankle_motor_max[motor_name] = max(
                self._ankle_motor_max[motor_name], abs(command.feedforward_torque_nm)
            )
        self._ankle_saturations += len(frame.saturated_ankle_joints)
        records = self.hardware["motor_map"]
        for motor_name, command in frame.other_motor_commands.items():
            state = self.motor_states[motor_name]
            self._max_embedded_kd = max(self._max_embedded_kd, float(command.kd))
            torque = (
                command.kp * (command.position_rad - state.position_rad)
                + command.kd * (command.velocity_rad_s - state.velocity_rad_s)
                + command.feedforward_torque_nm
            )
            absolute = abs(float(torque))
            self._direct_torque_max[motor_name] = max(
                self._direct_torque_max[motor_name], absolute
            )
            peak = records[motor_name].get("peak_torque_nm")
            if peak is None:
                torque_range = records[motor_name]["mit_ranges"]["torque_nm"]
                peak = min(abs(float(value)) for value in torque_range)
            if float(peak) > 0.0:
                ratio = absolute / float(peak)
                self._direct_torque_ratio[motor_name] = max(
                    self._direct_torque_ratio[motor_name], ratio
                )
                self._max_direct_peak_ratio = max(
                    self._max_direct_peak_ratio, ratio
                )

    def tick(self, now_ns: int | None = None) -> None:
        if not self.ready:
            return
        now_ns = self.clock_ns() if now_ns is None else int(now_ns)
        if self.next_state_tick_ns is None:
            self.next_state_tick_ns = now_ns
        catch_up = 0
        while now_ns >= self.next_state_tick_ns and catch_up < 20:
            lateness_ms = (now_ns - self.next_state_tick_ns) / 1.0e6
            self._state_lateness_ms.append(lateness_ms)
            self._state_deadline_misses += int(lateness_ms > 2.0)
            try:
                if self._state_ticks % self._policy_tick_divisor == 0:
                    self._run_policy(self.next_state_tick_ns)
                frame = self.controller.step(self.motor_states)
                self._audit_command_frame(frame)
            except (RuntimeError, ValueError) as exc:
                self._errors.append(str(exc))
                return
            self._state_ticks += 1
            self.next_state_tick_ns += 2_000_000
            catch_up += 1
        if catch_up == 20 and now_ns >= self.next_state_tick_ns:
            self._errors.append("500 Hz shadow calculation fell more than 20 ticks behind")

    def report(self) -> PolicyShadowReport:
        kd_limit = float(self.hardware["controller"]["damiao_embedded_kd_max"])
        errors = list(self._errors)
        if self._policy_ticks == 0:
            errors.append("policy never received one complete 31-motor plus IMU state")
        return PolicyShadowReport(
            inference_backend=self.actor.backend,
            command_vx_vy_yaw_rate=tuple(map(float, self.command)),
            policy_ticks=self._policy_ticks,
            state_ticks=self._state_ticks,
            state_tick_deadline_misses=self._state_deadline_misses,
            policy_deadline_misses=self._deadline_misses,
            inference_mean_ms=(
                float(np.mean(self._inference_ms)) if self._inference_ms else None
            ),
            inference_p99_ms=(
                _percentile(self._inference_ms, 0.99) if self._inference_ms else None
            ),
            inference_max_ms=max(self._inference_ms) if self._inference_ms else None,
            state_tick_lateness_p99_ms=(
                _percentile(self._state_lateness_ms, 0.99)
                if self._state_lateness_ms
                else None
            ),
            state_tick_lateness_max_ms=(
                max(self._state_lateness_ms) if self._state_lateness_ms else None
            ),
            maximum_abs_action=self._max_action,
            maximum_abs_action_delta=self._max_action_delta,
            maximum_abs_target_delta_rad=self._max_target_delta,
            maximum_abs_ankle_joint_torque_nm=self._ankle_joint_max,
            maximum_abs_ankle_motor_torque_nm=self._ankle_motor_max,
            ankle_joint_saturation_count=self._ankle_saturations,
            maximum_abs_direct_motor_estimated_torque_nm=self._direct_torque_max,
            maximum_direct_motor_torque_limit_ratio_by_motor=self._direct_torque_ratio,
            maximum_direct_motor_peak_torque_ratio=self._max_direct_peak_ratio,
            damiao_embedded_kd_limit=kd_limit,
            maximum_embedded_kd_requested=self._max_embedded_kd,
            horizontal_base_velocity_present=bool(
                self.contract.data["observation_has_horizontal_base_velocity"]
            ),
            serial_write_count=0,
            nonzero_motor_command_tx_attempts=0,
            automatic_enable_attempts=0,
            automatic_mode_switch_attempts=0,
            errors=tuple(dict.fromkeys(errors)),
        )
