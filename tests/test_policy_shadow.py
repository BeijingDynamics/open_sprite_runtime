import unittest
from unittest.mock import patch

import numpy as np

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.damiao import DamiaoMitState
from open_sprite_runtime.imu import ImuMount, matrix_to_quaternion_wxyz
from open_sprite_runtime.motor_mapping import motor_map_from_hardware_config
from open_sprite_runtime.policy_shadow import LivePolicyShadow, PolicyObservationHistory
from open_sprite_runtime.yahboom_imu import YahboomQuaternion, YahboomRawImu
from tests.test_contracts import valid_policy_data
from tests.test_hardware import JOINTS, complete_hardware


class PolicyObservationHistoryTests(unittest.TestCase):
    def setUp(self) -> None:
        data = valid_policy_data()
        data["default_joint_pos"] = list(np.linspace(-0.3, 0.3, 31))
        self.contract = PolicyContract(path=None, data=data)  # type: ignore[arg-type]

    def test_assembles_relative_joint_position_and_replicated_history(self) -> None:
        history = PolicyObservationHistory(self.contract)
        default = np.asarray(self.contract.data["default_joint_pos"])
        history.append(
            default + 0.25,
            np.full(31, 0.5),
            np.full(31, -0.2),
            np.array([1.0, 2.0, 3.0]),
            np.array([0.0, 0.0, -1.0]),
        )
        observation = history.observation(np.array([0.3, 0.0, -0.2]))
        self.assertEqual(observation.shape, (795,))
        np.testing.assert_allclose(observation[:248], 0.25, atol=1.0e-6)
        np.testing.assert_allclose(observation[248:496], 0.5)
        np.testing.assert_allclose(observation[496:744], -0.2)
        np.testing.assert_allclose(observation[-3:], [0.3, 0.0, -0.2])

    def test_history_is_oldest_to_newest(self) -> None:
        history = PolicyObservationHistory(self.contract)
        default = np.asarray(self.contract.data["default_joint_pos"])
        zeros31 = np.zeros(31)
        zeros3 = np.zeros(3)
        history.append(default, zeros31, zeros31, zeros3, np.array([0.0, 0.0, -1.0]))
        history.append(default + 1.0, zeros31, zeros31, zeros3, np.array([0.0, 0.0, -1.0]))
        observation = history.observation(zeros3)
        joint_history = observation[:248].reshape(8, 31)
        np.testing.assert_allclose(joint_history[:-1], 0.0, atol=1.0e-7)
        np.testing.assert_allclose(joint_history[-1], 1.0, atol=1.0e-7)


class LivePolicyShadowTests(unittest.TestCase):
    def test_runs_50hz_policy_and_500hz_control_without_tx(self) -> None:
        data = valid_policy_data()
        data["joint_names"] = list(JOINTS)
        contract = PolicyContract(path=None, data=data)  # type: ignore[arg-type]
        hardware = complete_hardware()
        mapping = motor_map_from_hardware_config(hardware, JOINTS)

        class Actor:
            backend = "fake_test_actor"

            @staticmethod
            def run(observation):
                self.assertEqual(observation.shape, (795,))
                return np.zeros(31, dtype=np.float32)

        class Clock:
            value = 0

            def __call__(self):
                self.value += 100_000
                return self.value

        with patch("open_sprite_runtime.policy_shadow.load_actor", return_value=Actor()):
            shadow = LivePolicyShadow(
                contract, hardware, mapping, (0.0, 0.0, 0.0), clock_ns=Clock()
            )

        default = np.asarray(data["default_joint_pos"])
        motor_position = mapping.joint_to_motor_positions(default)
        motor_velocity = mapping.joint_to_motor_velocities(np.zeros(31))
        shadow.motor_states = {
            name: DamiaoMitState(motor_position[name], motor_velocity[name])
            for name in mapping.physical_motor_names
        }
        mount = ImuMount.sprite0825_rear_pelvis()
        upright = matrix_to_quaternion_wxyz(mount.sensor_to_body_matrix)
        shadow.update_imu(YahboomRawImu((0.0, 0.0, 9.80665), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
        shadow.update_imu(YahboomQuaternion(tuple(upright)))

        for tick in range(20):
            shadow.tick(1_000_000_000 + tick * 2_000_000)
        report = shadow.report()
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(report.policy_ticks, 2)
        self.assertEqual(report.state_ticks, 20)
        self.assertEqual(report.nonzero_motor_command_tx_attempts, 0)
        self.assertLess(report.maximum_embedded_kd_requested, 3.01)


if __name__ == "__main__":
    unittest.main()
