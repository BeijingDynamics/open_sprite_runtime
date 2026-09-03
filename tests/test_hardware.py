import copy
import unittest

from open_sprite_runtime.hardware import (
    ANKLE_JOINTS,
    ANKLE_PAIRS,
    make_hardware_template,
    validate_hardware_inventory,
)


JOINTS = (
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_roll_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "waist_yaw_joint",
    "left_hip_yaw_joint", "right_hip_yaw_joint", "head_pitch_joint",
    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint", "left_knee_joint",
    "right_knee_joint", "head_roll_joint", "left_shoulder_roll_joint",
    "right_shoulder_roll_joint", "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "head_yaw_joint", "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint", "left_elbow_joint",
    "right_elbow_joint", "left_wrist_yaw_joint", "right_wrist_yaw_joint",
    "left_wrist_pitch_joint", "right_wrist_pitch_joint", "left_wrist_roll_joint",
    "right_wrist_roll_joint",
)


def motor(channel: int, can_id: int) -> dict:
    return {
        "model": "DM-J4340P-2EC",
        "firmware": "measured-version",
        "can_channel": channel,
        "can_id": can_id,
        "motor_zero_rad": 0.0,
        "encoder_sign": 1,
        "policy_to_motor_sign": 1,
        "reduction_ratio": 1.0,
        "linkage_ratio": 1.0,
        "soft_limit_rad": [-1.0, 1.0],
        "hard_limit_rad": [-1.1, 1.1],
        "rated_torque_nm": 14.0,
        "peak_torque_nm": 40.0,
        "rated_speed_rad_s": 3.8,
        "max_speed_rad_s": 10.0,
        "rated_current_a": 4.0,
        "peak_current_a": 12.0,
        "temperature_limit_c": 70.0,
        "mit_ranges": {
            "position_rad": [-12.5, 12.5],
            "velocity_rad_s": [-30.0, 30.0],
            "kp": [0.0, 500.0],
            "kd": [0.0, 5.0],
            "torque_nm": [-40.0, 40.0],
        },
    }


def complete_hardware() -> dict:
    records = {}
    index = 0
    for joint in JOINTS:
        if joint in ANKLE_JOINTS:
            continue
        row = motor(index // 8, index % 8 + 1)
        row["policy_joint"] = joint
        records[f"motor_{index:02d}"] = row
        index += 1
    for side, pair in ANKLE_PAIRS.items():
        for suffix in ("a", "b"):
            row = motor(index // 8, index % 8 + 1)
            row["model"] = "DM-J4310P-2EC"
            row["rated_torque_nm"] = 3.5
            row["peak_torque_nm"] = 12.5
            row["rated_speed_rad_s"] = 12.56
            row["max_speed_rad_s"] = 47.1
            row["coupled_joints"] = list(pair)
            row.pop("policy_to_motor_sign")
            records[f"{side}_ankle_motor_{suffix}"] = row
            index += 1
    return {
        "configured": True,
        "imu": {
            "configured": True,
            "mount_link": "pelvis",
            "body_to_sensor_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            "gyro_units": "rad_s",
            "quaternion_order": "wxyz",
            "update_hz": 500,
            "timestamp_source": "device",
            "measured_yaw_drift_deg_per_min": 0.2,
        },
        "estop": {"configured": True, "hardware_chain": "physical normally-closed loop"},
        "motor_map": records,
        "ankles": {
            side: {
                "calibrated": True,
                "joint_to_motor_matrix": [[1.0, 1.0], [1.0, -1.0]],
                "motor_zero_rad": [0.1, -0.1],
                "source": "fixture measurement 2026-09-03",
            }
            for side in ANKLE_PAIRS
        },
    }


class HardwareInventoryTest(unittest.TestCase):
    def test_generated_template_has_physical_topology_and_known_leg_ratings(self) -> None:
        template = make_hardware_template(JOINTS)
        records = template["motor_map"]
        self.assertEqual(len(records), 31)
        self.assertFalse(template["configured"])
        self.assertEqual(records["left_hip_pitch_motor"]["peak_torque_nm"], 40.0)
        self.assertEqual(records["left_ankle_motor_a"]["peak_torque_nm"], 12.5)
        self.assertNotIn("policy_to_motor_sign", records["left_ankle_motor_a"])
        report = validate_hardware_inventory(template, JOINTS)
        self.assertFalse(report.valid)
        self.assertFalse(report.missing_policy_joints)
        self.assertLess(len(report.errors), 80)

    def test_complete_inventory_passes(self) -> None:
        report = validate_hardware_inventory(complete_hardware(), JOINTS)
        self.assertTrue(report.valid, report.errors)
        self.assertEqual(report.physical_motor_count, 31)

    def test_empty_example_fails_closed(self) -> None:
        report = validate_hardware_inventory({"configured": False, "motor_map": {}}, JOINTS)
        self.assertFalse(report.valid)
        self.assertEqual(len(report.missing_policy_joints), 27)

    def test_duplicate_can_and_missing_mapping_are_rejected(self) -> None:
        hardware = complete_hardware()
        records = hardware["motor_map"]
        labels = list(records)
        records[labels[1]]["can_channel"] = records[labels[0]]["can_channel"]
        records[labels[1]]["can_id"] = records[labels[0]]["can_id"]
        records[labels[1]]["policy_joint"] = records[labels[0]]["policy_joint"]
        report = validate_hardware_inventory(hardware, JOINTS)
        self.assertFalse(report.valid)
        self.assertTrue(any("duplicate CAN" in error for error in report.errors))
        self.assertTrue(report.duplicate_policy_joints)

    def test_ankle_must_use_two_coupled_motors_per_side(self) -> None:
        hardware = complete_hardware()
        hardware["motor_map"].pop("left_ankle_motor_b")
        report = validate_hardware_inventory(hardware, JOINTS)
        self.assertFalse(report.valid)
        self.assertTrue(any("left ankle requires exactly two" in error for error in report.errors))


if __name__ == "__main__":
    unittest.main()
