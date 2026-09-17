import unittest

import numpy as np

from open_sprite_runtime.damiao import (
    DamiaoMitState,
    command_profiles_from_hardware_config,
)
from open_sprite_runtime.motor_mapping import motor_map_from_hardware_config
from open_sprite_runtime.multirate_control import (
    JointImpedanceTarget,
    SpriteMultiRateController,
    encode_multirate_command_frame,
)
from tests.test_hardware import JOINTS, complete_hardware


class MultiRateControllerTests(unittest.TestCase):
    def setUp(self):
        self.mapping = motor_map_from_hardware_config(complete_hardware(), JOINTS)
        self.controller = SpriteMultiRateController(
            self.mapping, np.full(31, 20.0)
        )
        zeros = np.zeros(31)
        motor_position = self.mapping.joint_to_motor_positions(zeros)
        motor_velocity = self.mapping.joint_to_motor_velocities(zeros)
        self.states = {
            name: DamiaoMitState(motor_position[name], motor_velocity[name])
            for name in self.mapping.physical_motor_names
        }

    def target(self, position=None, kp=None, feedforward=None):
        return JointImpedanceTarget(
            position_rad=np.zeros(31) if position is None else position,
            velocity_rad_s=np.zeros(31),
            kp=np.full(31, 10.0) if kp is None else kp,
            kd=np.full(31, 1.0),
            feedforward_torque_nm=(
                np.zeros(31) if feedforward is None else feedforward
            ),
        )

    def test_requires_latched_target_and_complete_feedback(self):
        with self.assertRaisesRegex(RuntimeError, "latched"):
            self.controller.step(self.states)
        self.controller.latch_policy_target(self.target())
        incomplete = dict(self.states)
        incomplete.pop(next(iter(incomplete)))
        with self.assertRaisesRegex(ValueError, "exactly cover"):
            self.controller.step(incomplete)

    def test_ankles_are_pure_torque_commands_every_state_tick(self):
        position = np.zeros(31)
        position[JOINTS.index("left_ankle_pitch_joint")] = 0.1
        position[JOINTS.index("left_ankle_roll_joint")] = -0.2
        self.controller.latch_policy_target(self.target(position=position))
        for _ in range(13):
            frame = self.controller.step(self.states)
            self.assertEqual(len(frame.ankle_commands), 4)
            for command in frame.ankle_commands.values():
                self.assertEqual(command.kp, 0.0)
                self.assertEqual(command.kd, 0.0)
                self.assertEqual(command.velocity_rad_s, 0.0)

        pair = self.mapping.differentials["left_ankle"]
        expected = pair.joint_to_drive_torque(np.array([1.0, -2.0]))
        np.testing.assert_allclose(
            [
                frame.ankle_commands[name].feedforward_torque_nm
                for name in pair.motor_names
            ],
            expected,
        )

    def test_other_motors_update_once_per_ten_state_ticks(self):
        self.controller.latch_policy_target(self.target())
        ankle_updates = 0
        other_updates = 0
        for _ in range(500):
            frame = self.controller.step(self.states)
            ankle_updates += bool(frame.ankle_commands)
            other_updates += frame.updates_other_motors
            self.assertFalse(
                set(frame.ankle_commands) & set(frame.other_motor_commands)
            )
            if frame.other_motor_commands:
                self.assertEqual(len(frame.other_motor_commands), 27)
                self.assertTrue(any(command.kp > 0.0 for command in frame.other_motor_commands.values()))
        self.assertEqual(ankle_updates, 500)
        self.assertEqual(other_updates, 50)

    def test_ankle_joint_torque_is_limited_before_transmission_map(self):
        position = np.zeros(31)
        position[JOINTS.index("right_ankle_pitch_joint")] = 10.0
        self.controller.latch_policy_target(
            self.target(position=position, kp=np.full(31, 100.0))
        )
        frame = self.controller.step(self.states)
        self.assertEqual(frame.ankle_joint_torque_nm["right_ankle"], (20.0, 0.0))
        self.assertEqual(
            frame.saturated_ankle_joints, ("right_ankle_pitch_joint",)
        )

    def test_frame_encoding_gates_ankles_and_other_motors_before_transport(self):
        hardware = complete_hardware()
        profiles = command_profiles_from_hardware_config(hardware, JOINTS)
        self.controller.latch_policy_target(self.target())
        policy_frame = self.controller.step(self.states)
        encoded = encode_multirate_command_frame(
            self.mapping, profiles, policy_frame, self.states
        )
        self.assertEqual(len(encoded.ankle_commands), 4)
        self.assertEqual(len(encoded.other_motor_commands), 27)

        state_frame = self.controller.step(self.states)
        encoded = encode_multirate_command_frame(
            self.mapping, profiles, state_frame, self.states
        )
        self.assertEqual(len(encoded.ankle_commands), 4)
        self.assertFalse(encoded.other_motor_commands)

    def test_motor_torque_envelope_rejects_coupled_ankle_overload(self):
        hardware = complete_hardware()
        profiles = command_profiles_from_hardware_config(hardware, JOINTS)
        feedforward = np.zeros(31)
        feedforward[JOINTS.index("left_ankle_pitch_joint")] = 20.0
        feedforward[JOINTS.index("left_ankle_roll_joint")] = 20.0
        self.controller.latch_policy_target(self.target(feedforward=feedforward))
        frame = self.controller.step(self.states)
        with self.assertRaisesRegex(ValueError, "feedforward_torque_nm"):
            encode_multirate_command_frame(
                self.mapping, profiles, frame, self.states
            )


if __name__ == "__main__":
    unittest.main()
