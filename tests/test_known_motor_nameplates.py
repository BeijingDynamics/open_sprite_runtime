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
                "unknown": {"model": "unconfirmed-model", "rated_torque_nm": None},
            }
        }
        self.assertEqual(apply_nameplates(hardware), 4)
        self.assertEqual(hardware["motor_map"]["known"]["peak_torque_nm"], 12.5)
        self.assertIsNone(hardware["motor_map"]["unknown"]["rated_torque_nm"])

    def test_applies_confirmed_torque_only_to_3507_and_6248(self) -> None:
        hardware = {
            "motor_map": {
                "head": {"model": "DM-J3507-2EC (48V)"},
                "waist": {"model": "DM-J6248P-2EC"},
            }
        }
        self.assertEqual(apply_nameplates(hardware), 4)
        self.assertEqual(hardware["motor_map"]["head"]["peak_torque_nm"], 3.0)
        self.assertEqual(hardware["motor_map"]["waist"]["rated_torque_nm"], 30.0)


if __name__ == "__main__":
    unittest.main()
