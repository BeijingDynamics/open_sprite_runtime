import json
import unittest
from pathlib import Path

from open_sprite_runtime.hardware import make_hardware_template, validate_hardware_inventory


ROOT = Path(__file__).resolve().parents[1]
HARDWARE = ROOT / "config" / "hardware.sprite0825.measurement.json"


class Sprite0825ShoulderMotorContractTest(unittest.TestCase):
    @staticmethod
    def policy_joint_names() -> list[str]:
        motor_map = json.loads(HARDWARE.read_text(encoding="utf-8"))["motor_map"]
        names: list[str] = []
        for motor in motor_map.values():
            candidates = [motor["policy_joint"]] if "policy_joint" in motor else motor["coupled_joints"]
            for name in candidates:
                if name not in names:
                    names.append(name)
        return names

    def test_proximal_shoulders_are_j4340p(self) -> None:
        motor_map = json.loads(HARDWARE.read_text(encoding="utf-8"))["motor_map"]
        expected = {
            "left_shoulder_pitch_motor",
            "right_shoulder_pitch_motor",
            "left_shoulder_roll_motor",
            "right_shoulder_roll_motor",
        }
        for name in expected:
            motor = motor_map[name]
            self.assertEqual(motor["model"], "DM-J4340P-2EC")
            self.assertEqual(motor["rated_torque_nm"], 14.0)
            self.assertEqual(motor["peak_torque_nm"], 40.0)
            self.assertEqual(motor["rated_speed_rad_s"], 3.77)
            self.assertEqual(motor["max_speed_rad_s"], 9.3)

    def test_distal_arm_motors_are_not_reclassified(self) -> None:
        motor_map = json.loads(HARDWARE.read_text(encoding="utf-8"))["motor_map"]
        distal = {
            "left_shoulder_yaw_motor",
            "right_shoulder_yaw_motor",
            "left_elbow_motor",
            "right_elbow_motor",
            "left_wrist_yaw_motor",
            "right_wrist_yaw_motor",
        }
        for name in distal:
            self.assertNotEqual(motor_map[name]["model"], "DM-J4340P-2EC")

    def test_generated_template_preserves_upgraded_shoulders(self) -> None:
        generated = make_hardware_template(self.policy_joint_names())
        for side in ("left", "right"):
            for axis in ("pitch", "roll"):
                motor = generated["motor_map"][f"{side}_shoulder_{axis}_motor"]
                self.assertEqual(motor["model"], "DM-J4340P-2EC")
                self.assertEqual(motor["rated_torque_nm"], 14.0)
                self.assertEqual(motor["peak_torque_nm"], 40.0)
                self.assertEqual(motor["max_speed_rad_s"], 9.3)

    def test_inventory_rejects_legacy_motor_on_upgraded_shoulder(self) -> None:
        joints = self.policy_joint_names()
        generated = make_hardware_template(joints)
        generated["motor_map"]["left_shoulder_pitch_motor"]["model"] = "DM-J4310P-2EC"
        report = validate_hardware_inventory(generated, joints)
        self.assertTrue(
            any("left_shoulder_pitch_motor shoulder pitch/roll motor must be DM-J4340P-2EC" in error for error in report.errors),
            report.errors,
        )


if __name__ == "__main__":
    unittest.main()
