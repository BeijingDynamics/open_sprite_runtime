import unittest

import numpy as np

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
        hardware["ankles"]["left"]["motor_names"].reverse()
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


if __name__ == "__main__":
    unittest.main()
