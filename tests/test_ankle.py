import unittest

import numpy as np

from open_sprite_runtime.ankle import DifferentialAnkle


class DifferentialAnkleTests(unittest.TestCase):
    def setUp(self) -> None:
        self.ankle = DifferentialAnkle.ideal_symmetric()

    def test_position_round_trip(self) -> None:
        joint = np.array([0.2, -0.1])
        np.testing.assert_allclose(
            self.ankle.motor_to_joint_position(self.ankle.joint_to_motor_position(joint)),
            joint,
        )

    def test_torque_round_trip(self) -> None:
        joint_torque = np.array([4.0, -2.0])
        motor_torque = self.ankle.joint_to_motor_torque(joint_torque)
        np.testing.assert_allclose(motor_torque, [1.0, 3.0])
        np.testing.assert_allclose(
            self.ankle.motor_to_joint_torque(motor_torque), joint_torque
        )

    def test_transform_preserves_power(self) -> None:
        joint_velocity = np.array([0.7, -0.2])
        joint_torque = np.array([3.2, 1.1])
        motor_velocity = self.ankle.joint_to_motor_velocity(joint_velocity)
        motor_torque = self.ankle.joint_to_motor_torque(joint_torque)
        self.assertAlmostEqual(
            float(joint_velocity @ joint_torque),
            float(motor_velocity @ motor_torque),
        )

    def test_equal_joint_gains_are_diagonal_in_ideal_motor_space(self) -> None:
        np.testing.assert_allclose(
            self.ankle.diagonal_motor_gains([14.0, 14.0]), [7.0, 7.0]
        )

    def test_unequal_joint_gains_require_cross_coupling(self) -> None:
        with self.assertRaisesRegex(ValueError, "cross-coupled"):
            self.ankle.diagonal_motor_gains([14.0, 8.0])


if __name__ == "__main__":
    unittest.main()
