import unittest

from tools.apply_known_motor_nameplates import apply_nameplates


class KnownMotorNameplateTests(unittest.TestCase):
    def test_applies_only_confirmed_models(self) -> None:
        hardware = {
            "motor_map": {
                "known": {
                    "model": "DM-J4310P-2EC (48V)",
                    "rated_torque_nm": None,
                },
                "unknown": {"model": "DM-J3507-2EC (48V)", "rated_torque_nm": None},
            }
        }
        self.assertEqual(apply_nameplates(hardware), 4)
        self.assertEqual(hardware["motor_map"]["known"]["peak_torque_nm"], 12.5)
        self.assertIsNone(hardware["motor_map"]["unknown"]["rated_torque_nm"])


if __name__ == "__main__":
    unittest.main()
