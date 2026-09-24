import unittest

from tools.export_native_motor_config import _default_feedback_cap


class NativeMotorConfigExportTests(unittest.TestCase):
    def test_conservative_feedback_cap_uses_ten_percent_rated(self) -> None:
        self.assertAlmostEqual(
            _default_feedback_cap(
                rated_torque_nm=14.0,
                mechanical_peak_torque_nm=40.0,
                protocol_torque_cap_nm=28.0,
                use_hardware_limit=False,
            ),
            1.4,
        )

    def test_hardware_feedback_cap_uses_protocol_limit_when_lower(self) -> None:
        self.assertEqual(
            _default_feedback_cap(
                rated_torque_nm=14.0,
                mechanical_peak_torque_nm=40.0,
                protocol_torque_cap_nm=28.0,
                use_hardware_limit=True,
            ),
            28.0,
        )

    def test_hardware_feedback_cap_uses_mechanical_peak_when_lower(self) -> None:
        self.assertEqual(
            _default_feedback_cap(
                rated_torque_nm=0.8,
                mechanical_peak_torque_nm=3.0,
                protocol_torque_cap_nm=5.0,
                use_hardware_limit=True,
            ),
            3.0,
        )


if __name__ == "__main__":
    unittest.main()
