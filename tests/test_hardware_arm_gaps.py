import unittest

from tests.test_hardware import JOINTS, complete_hardware
from tools.report_hardware_arm_gaps import build_report


class HardwareArmGapReportTests(unittest.TestCase):
    def test_complete_hardware_has_no_grouped_or_sensor_gaps(self) -> None:
        hardware = complete_hardware()
        hardware["imu"]["device_path"] = "/dev/sprite-pelvis-imu"
        report = build_report(hardware, list(JOINTS))
        self.assertTrue(report["inventory_valid"])
        self.assertFalse(report["imu_blockers"])
        self.assertFalse(report["estop_blockers"])
        for group in report["motor_groups"].values():
            self.assertFalse(group["mit_range_register_readback_required"])
            self.assertTrue(all(not fields for fields in group["missing_fields_by_motor"].values()))

    def test_groups_repeated_missing_fields_by_motor_model(self) -> None:
        hardware = complete_hardware()
        names = list(hardware["motor_map"])
        for name in names[:2]:
            hardware["motor_map"][name]["firmware"] = None
            hardware["motor_map"][name]["mit_ranges"]["source"] = (
                "operator_confirmed_drive_configuration"
            )
        report = build_report(hardware, list(JOINTS))
        group = report["motor_groups"][hardware["motor_map"][names[0]]["model"]]
        self.assertIn("firmware", group["missing_fields_by_motor"][names[0]])
        self.assertIn(names[0], group["mit_range_register_readback_required"])
        self.assertFalse(report["inventory_valid"])


if __name__ == "__main__":
    unittest.main()
