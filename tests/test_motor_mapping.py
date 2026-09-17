import unittest

import numpy as np

from open_sprite_runtime.damiao import (
    DamiaoMitState,
    command_profiles_from_hardware_config,
    encode_damiao_mit_command_bank,
)
from open_sprite_runtime.motor_mapping import DirectMotorMap, motor_map_from_hardware_config
from tests.test_hardware import JOINTS, complete_hardware


class MotorMappingTests(unittest.TestCase):
    def test_direct_map_round_trip_gains_and_power(self) -> None:
        mapping = DirectMotorMap("motor", "joint", 0.3, -1, 1, 2.0)
        self.assertAlmostEqual(mapping.joint_to_drive_position(0.2), -0.1)
        self.assertAlmostEqual(mapping.drive_to_joint_position(-0.1), 0.2)
        self.assertEqual(mapping.joint_to_drive_gains(20.0, 2.0), (5.0, 0.5))
        joint_velocity = 0.7
        joint_torque = 3.2
        self.assertAlmostEqual(
            joint_velocity * joint_torque,
            mapping.joint_to_drive_velocity(joint_velocity)
            * mapping.joint_to_drive_torque(joint_torque),
        )

    def test_full_map_round_trips_position_velocity_and_torque(self) -> None:
        mapping = motor_map_from_hardware_config(complete_hardware(), JOINTS)
        rng = np.random.default_rng(7)
        for kind in ("positions", "velocities", "torques"):
            joint = rng.normal(0.0, 0.2, 31)
            to_motor = getattr(mapping, f"joint_to_motor_{kind}")
            to_joint = getattr(mapping, f"motor_to_joint_{kind}")
            motor = to_motor(joint)
            self.assertEqual(len(motor), 31)
            np.testing.assert_allclose(to_joint(motor), joint, atol=1.0e-12)

    def test_full_map_preserves_instantaneous_power(self) -> None:
        hardware = complete_hardware()
        hardware["motor_map"]["motor_00"]["encoder_sign"] = -1
        hardware["motor_map"]["motor_00"]["policy_to_motor_sign"] = -1
        hardware["motor_map"]["motor_00"]["reduction_ratio"] = 2.0
        hardware["motor_map"]["left_ankle_motor_b"]["encoder_sign"] = -1
        mapping = motor_map_from_hardware_config(hardware, JOINTS)
        rng = np.random.default_rng(19)
        joint_velocity = rng.normal(0.0, 0.5, 31)
        joint_torque = rng.normal(0.0, 2.0, 31)
        motor_velocity = mapping.joint_to_motor_velocities(joint_velocity)
        motor_torque = mapping.joint_to_motor_torques(joint_torque)
        motor_power = sum(
            motor_velocity[name] * motor_torque[name] for name in motor_velocity
        )
        self.assertAlmostEqual(float(joint_velocity @ joint_torque), motor_power)

    def test_ankle_motor_name_order_defines_matrix_rows(self) -> None:
        hardware = complete_hardware()
        normal = motor_map_from_hardware_config(hardware, JOINTS)
        joint = np.zeros(31)
        joint[JOINTS.index("left_ankle_roll_joint")] = 0.2
        normal_motor = normal.joint_to_motor_positions(joint)

        hardware = complete_hardware()
        hardware["differentials"]["left_ankle"]["motor_names"].reverse()
        swapped = motor_map_from_hardware_config(hardware, JOINTS)
        swapped_motor = swapped.joint_to_motor_positions(joint)
        self.assertAlmostEqual(
            normal_motor["left_ankle_motor_a"], swapped_motor["left_ankle_motor_b"]
        )
        self.assertAlmostEqual(
            normal_motor["left_ankle_motor_b"], swapped_motor["left_ankle_motor_a"]
        )

    def test_incomplete_hardware_cannot_build_mapping(self) -> None:
        hardware = complete_hardware()
        hardware["configured"] = False
        with self.assertRaisesRegex(ValueError, "configured=true"):
            motor_map_from_hardware_config(hardware, JOINTS)

    def test_receive_only_mapping_does_not_require_armable_inventory(self) -> None:
        hardware = complete_hardware()
        hardware["configured"] = False
        mapping = motor_map_from_hardware_config(
            hardware, JOINTS, require_armable=False
        )
        zeros = np.zeros(31)
        np.testing.assert_allclose(
            mapping.motor_to_joint_positions(mapping.joint_to_motor_positions(zeros)),
            zeros,
        )

    def test_impedance_commands_reproduce_exact_joint_space_torque(self) -> None:
        hardware = complete_hardware()
        hardware["motor_map"]["motor_00"]["encoder_sign"] = -1
        hardware["motor_map"]["motor_00"]["policy_to_motor_sign"] = -1
        hardware["motor_map"]["motor_00"]["reduction_ratio"] = 1.7
        hardware["motor_map"]["left_ankle_motor_b"]["encoder_sign"] = -1
        hardware["differentials"]["left_ankle"]["joint_to_motor_matrix"] = [
            [1.02, 0.96],
            [0.98, -1.04],
        ]
        mapping = motor_map_from_hardware_config(hardware, JOINTS)
        rng = np.random.default_rng(31)
        desired_position = rng.normal(0.0, 0.1, 31)
        desired_velocity = rng.normal(0.0, 0.2, 31)
        measured_position = rng.normal(0.0, 0.1, 31)
        measured_velocity = rng.normal(0.0, 0.2, 31)
        kp = np.linspace(8.0, 38.0, 31)
        kd = np.linspace(0.4, 1.6, 31)
        joint_ff = rng.normal(0.0, 0.3, 31)
        commands = mapping.joint_impedance_to_motor_commands(
            desired_position,
            desired_velocity,
            measured_position,
            measured_velocity,
            kp,
            kd,
            joint_ff,
        )
        measured_drive_position = mapping.joint_to_motor_positions(measured_position)
        measured_drive_velocity = mapping.joint_to_motor_velocities(measured_velocity)
        drive_torque = {
            name: command.kp * (command.position_rad - measured_drive_position[name])
            + command.kd * (command.velocity_rad_s - measured_drive_velocity[name])
            + command.feedforward_torque_nm
            for name, command in commands.items()
        }
        expected_joint_torque = (
            kp * (desired_position - measured_position)
            + kd * (desired_velocity - measured_velocity)
            + joint_ff
        )
        np.testing.assert_allclose(
            mapping.motor_to_joint_torques(drive_torque),
            expected_joint_torque,
            atol=1.0e-11,
        )

    def test_unequal_ankle_gains_create_host_coupling_feedforward(self) -> None:
        mapping = motor_map_from_hardware_config(complete_hardware(), JOINTS)
        desired = np.zeros(31)
        desired[JOINTS.index("left_ankle_pitch_joint")] = 0.1
        commands = mapping.joint_impedance_to_motor_commands(
            desired,
            np.zeros(31),
            np.zeros(31),
            np.zeros(31),
            np.linspace(10.0, 40.0, 31),
            np.ones(31),
            np.zeros(31),
        )
        self.assertNotEqual(
            commands["left_ankle_motor_a"].feedforward_torque_nm,
            0.0,
        )

    def test_head_pitch_roll_use_two_motor_differential(self) -> None:
        hardware = complete_hardware()
        hardware["differentials"]["head"]["motor_zero_rad"] = [0.0, 0.0]
        mapping = motor_map_from_hardware_config(hardware, JOINTS)
        joint = np.zeros(31)
        joint[JOINTS.index("head_pitch_joint")] = 0.2
        joint[JOINTS.index("head_roll_joint")] = -0.1
        motor = mapping.joint_to_motor_positions(joint)
        self.assertAlmostEqual(motor["head_motor_a"], 0.1)
        self.assertAlmostEqual(motor["head_motor_b"], 0.3)
        np.testing.assert_allclose(mapping.motor_to_joint_positions(motor), joint)

    def test_confirmed_negative_direct_sign_round_trips_feedback(self) -> None:
        hardware = complete_hardware()
        record = next(
            row
            for row in hardware["motor_map"].values()
            if row.get("policy_joint") == "waist_yaw_joint"
        )
        record["policy_to_motor_sign"] = -1
        mapping = motor_map_from_hardware_config(hardware, JOINTS)
        joint = np.zeros(31)
        joint[JOINTS.index("waist_yaw_joint")] = 0.2
        motor = mapping.joint_to_motor_positions(joint)
        mapped_motor_name = next(
            name
            for name, row in hardware["motor_map"].items()
            if row.get("policy_joint") == "waist_yaw_joint"
        )
        self.assertAlmostEqual(motor[mapped_motor_name], -0.2)
        np.testing.assert_allclose(mapping.motor_to_joint_positions(motor), joint)

    def test_mapping_profiles_and_encoder_form_exact_31_motor_bank(self) -> None:
        hardware = complete_hardware()
        mapping = motor_map_from_hardware_config(hardware, JOINTS)
        profiles = command_profiles_from_hardware_config(hardware, JOINTS)
        zeros = np.zeros(31)
        commands = mapping.joint_impedance_to_motor_commands(
            zeros,
            zeros,
            zeros,
            zeros,
            np.full(31, 10.0),
            np.full(31, 1.0),
            zeros,
        )
        motor_position = mapping.joint_to_motor_positions(zeros)
        motor_velocity = mapping.joint_to_motor_velocities(zeros)
        states = {
            name: DamiaoMitState(motor_position[name], motor_velocity[name])
            for name in mapping.physical_motor_names
        }
        encoded = encode_damiao_mit_command_bank(profiles, commands, states)
        self.assertEqual(len(encoded), 31)
        self.assertEqual({item.motor_name for item in encoded}, set(commands))

        missing = dict(commands)
        missing.pop(next(iter(missing)))
        with self.assertRaisesRegex(ValueError, "exactly cover"):
            encode_damiao_mit_command_bank(profiles, missing, states)


if __name__ == "__main__":
    unittest.main()
