import unittest

from open_sprite_runtime.damiao import DamiaoFeedbackEndpoint, DamiaoMitRanges
from open_sprite_runtime.motor_group_hold import (
    LEFT_ARM_GROUP,
    RIGHT_ARM_GROUP,
    run_left_arm_group_low_gain_hold,
    run_right_arm_group_low_gain_hold,
    run_right_wrist_group_low_gain_hold,
)
from open_sprite_runtime.socketcan import ReceivedCanFrame


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        self.value += 0.001
        return self.value

    def sleep(self, seconds):
        self.value += seconds


def endpoints():
    ranges = DamiaoMitRanges(
        (-12.566, 12.566), (-50.0, 50.0), (-5.0, 5.0), "motor_register_readback"
    )
    return (
        DamiaoFeedbackEndpoint("right_wrist_yaw_motor", "kcan4", 5, 0x15, ranges),
        DamiaoFeedbackEndpoint("right_wrist_pitch_motor", "kcan4", 6, 0x16, ranges),
        DamiaoFeedbackEndpoint("right_wrist_roll_motor", "kcan4", 7, 0x17, ranges),
    )


def right_arm_endpoints():
    ranges = DamiaoMitRanges(
        (-12.566, 12.566), (-50.0, 50.0), (-5.0, 5.0), "motor_register_readback"
    )
    return tuple(
        DamiaoFeedbackEndpoint(name, interface, can_id, master_id, ranges)
        for name, interface, can_id, master_id in RIGHT_ARM_GROUP
    )


def left_arm_endpoints():
    ranges = DamiaoMitRanges(
        (-12.566, 12.566), (-50.0, 50.0), (-5.0, 5.0), "motor_register_readback"
    )
    return tuple(
        DamiaoFeedbackEndpoint(name, interface, can_id, master_id, ranges)
        for name, interface, can_id, master_id in LEFT_ARM_GROUP
    )


class FakeGroupWriter:
    def __init__(self, selected, *, bad_torque_motor=None):
        self.selected = {item.motor_name: item for item in selected}
        self.interface = next(iter(self.selected.values())).interface
        self.status = {name: 0 for name in self.selected}
        self.last_name = next(iter(self.selected))
        self.bad_torque_motor = bad_torque_motor
        self.command_tx_attempts = {name: 0 for name in self.selected}
        self.enable_attempts = {name: 0 for name in self.selected}
        self.disable_attempts = {name: 0 for name in self.selected}

    def send_command(self, command):
        self.last_name = command.motor_name
        self.command_tx_attempts[command.motor_name] += 1

    def send_enable(self, name):
        self.status[name] = 1
        self.enable_attempts[name] += 1

    def send_disable(self, name):
        self.status[name] = 0
        self.disable_attempts[name] += 1

    def receive(self):
        endpoint = self.selected[self.last_name]
        torque_raw = 2200 if self.last_name == self.bad_torque_motor else 2048
        data = bytes((
            (self.status[self.last_name] << 4) | endpoint.can_id,
            0x80, 0x00, 0x80, torque_raw >> 8, torque_raw & 0xFF, 30, 29,
        ))
        return ReceivedCanFrame(
            interface=self.interface, can_id=endpoint.master_id, data=data,
            is_extended=False, is_remote=False, is_error=False,
            is_fd=True, bit_rate_switch=True, error_state_indicator=False,
            software_timestamp_ns=1, hardware_timestamp_ns=1,
            userspace_receive_timestamp_ns=2,
        )


class MotorGroupHoldTests(unittest.TestCase):
    def test_right_wrist_group_hold_passes_and_disables_every_motor(self):
        selected = endpoints()
        writer = FakeGroupWriter(selected)
        clock = Clock()
        report = run_right_wrist_group_low_gain_hold(
            writer,
            selected,
            soft_position_rad={item.motor_name: (-0.7, 0.7) for item in selected},
            monotonic=clock,
            sleep=clock.sleep,
        )
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(set(report.initial_position_rad), {item.motor_name for item in selected})
        self.assertTrue(all(value == 1 for value in report.enable_attempts.values()))
        self.assertTrue(all(value >= 3 for value in report.disable_attempts.values()))
        self.assertTrue(all(value == "disabled" for value in report.final_status.values()))

    def test_one_motor_fault_disables_whole_group(self):
        selected = endpoints()
        writer = FakeGroupWriter(selected, bad_torque_motor="right_wrist_pitch_motor")
        clock = Clock()
        report = run_right_wrist_group_low_gain_hold(
            writer,
            selected,
            soft_position_rad={item.motor_name: (-0.7, 0.7) for item in selected},
            monotonic=clock,
            sleep=clock.sleep,
        )
        self.assertFalse(report.passed)
        self.assertTrue(any("torque guard" in value for value in report.errors))
        self.assertTrue(all(value >= 3 for value in report.disable_attempts.values()))
        self.assertTrue(all(value == "disabled" for value in report.final_status.values()))

    def test_rejects_reordered_or_incomplete_group(self):
        selected = endpoints()
        with self.assertRaisesRegex(ValueError, "frozen ordered group"):
            run_right_wrist_group_low_gain_hold(
                FakeGroupWriter(selected[:2]),
                selected[:2],
                soft_position_rad={item.motor_name: (-0.7, 0.7) for item in selected[:2]},
            )

    def test_soft_limit_failure_preserves_initial_position_without_enable(self):
        selected = endpoints()
        writer = FakeGroupWriter(selected)
        clock = Clock()
        limits = {item.motor_name: (-0.7, 0.7) for item in selected}
        limits[selected[0].motor_name] = (0.1, 0.7)
        report = run_right_wrist_group_low_gain_hold(
            writer,
            selected,
            soft_position_rad=limits,
            monotonic=clock,
            sleep=clock.sleep,
        )
        self.assertFalse(report.passed)
        self.assertAlmostEqual(
            report.initial_position_rad[selected[0].motor_name], 0.0, delta=0.001
        )
        self.assertTrue(any("soft-limit margin" in value for value in report.errors))
        self.assertTrue(all(value == 0 for value in report.enable_attempts.values()))

    def test_right_arm_group_hold_uses_all_seven_fixed_endpoints(self):
        selected = right_arm_endpoints()
        writer = FakeGroupWriter(selected)
        clock = Clock()
        report = run_right_arm_group_low_gain_hold(
            writer,
            selected,
            soft_position_rad={item.motor_name: (-0.7, 0.7) for item in selected},
            monotonic=clock,
            sleep=clock.sleep,
        )
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(len(report.motor_names), 7)
        self.assertTrue(all(value == 1 for value in report.enable_attempts.values()))
        self.assertTrue(all(value == "disabled" for value in report.final_status.values()))

    def test_left_arm_group_excludes_head_yaw_and_uses_kcan3(self):
        selected = left_arm_endpoints()
        writer = FakeGroupWriter(selected)
        clock = Clock()
        report = run_left_arm_group_low_gain_hold(
            writer,
            selected,
            soft_position_rad={item.motor_name: (-0.7, 0.7) for item in selected},
            monotonic=clock,
            sleep=clock.sleep,
        )
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(report.interface, "kcan3")
        self.assertNotIn("head_yaw_motor", report.motor_names)
        self.assertTrue(all(value == "disabled" for value in report.final_status.values()))


if __name__ == "__main__":
    unittest.main()
