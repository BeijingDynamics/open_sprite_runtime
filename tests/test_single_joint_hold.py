import unittest

from open_sprite_runtime.damiao import DamiaoFeedbackEndpoint, DamiaoMitRanges
from open_sprite_runtime.single_joint_hold import run_head_yaw_low_gain_hold
from open_sprite_runtime.socketcan import ReceivedCanFrame


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        self.value += 0.001
        return self.value

    def sleep(self, seconds):
        self.value += seconds


class FakeWriter:
    interface = "kcan3"

    def __init__(self, endpoint, *, torque_raw=2048):
        self.endpoint = endpoint
        self.status = 0
        self.torque_raw = torque_raw
        self.command_tx_attempts = 0
        self.enable_attempts = 0
        self.disable_attempts = 0

    def send_command(self, command):
        self.command_tx_attempts += 1

    def send_enable(self):
        self.enable_attempts += 1
        self.status = 1

    def send_disable(self):
        self.disable_attempts += 1
        self.status = 0

    def receive(self):
        position_raw = 32768
        velocity_raw = 2048
        data = bytes((
            (self.status << 4) | self.endpoint.can_id,
            position_raw >> 8,
            position_raw & 0xFF,
            velocity_raw >> 4,
            ((velocity_raw & 0xF) << 4) | (self.torque_raw >> 8),
            self.torque_raw & 0xFF,
            30,
            29,
        ))
        return ReceivedCanFrame(
            interface="kcan3", can_id=0x18, data=data,
            is_extended=False, is_remote=False, is_error=False,
            is_fd=True, bit_rate_switch=True, error_state_indicator=False,
            software_timestamp_ns=1, hardware_timestamp_ns=1,
            userspace_receive_timestamp_ns=2,
        )


def endpoint():
    return DamiaoFeedbackEndpoint(
        "head_yaw_motor", "kcan3", 8, 0x18,
        DamiaoMitRanges((-12.566, 12.566), (-50.0, 50.0), (-5.0, 5.0), "motor_register_readback"),
    )


class SingleJointHoldTests(unittest.TestCase):
    def test_current_position_hold_disables_and_passes(self):
        clock = Clock()
        writer = FakeWriter(endpoint())
        report = run_head_yaw_low_gain_hold(
            writer, endpoint(), soft_position_rad=(-1.0, 1.0),
            monotonic=clock, sleep=clock.sleep,
        )
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(report.enable_attempts, 1)
        self.assertGreaterEqual(report.disable_attempts, 3)
        self.assertEqual(report.final_status, "disabled")

    def test_torque_guard_fails_closed_and_disables(self):
        clock = Clock()
        writer = FakeWriter(endpoint(), torque_raw=2200)
        report = run_head_yaw_low_gain_hold(
            writer, endpoint(), soft_position_rad=(-1.0, 1.0),
            monotonic=clock, sleep=clock.sleep,
        )
        self.assertFalse(report.passed)
        self.assertTrue(any("torque guard" in value for value in report.errors))
        self.assertGreaterEqual(report.disable_attempts, 3)
        self.assertEqual(report.final_status, "disabled")

    def test_rejects_every_other_motor(self):
        wrong = DamiaoFeedbackEndpoint(
            "left_wrist_yaw_motor", "kcan3", 5, 0x15,
            endpoint().ranges,
        )
        with self.assertRaisesRegex(ValueError, "restricted"):
            run_head_yaw_low_gain_hold(
                FakeWriter(wrong), wrong, soft_position_rad=(-1.0, 1.0)
            )


if __name__ == "__main__":
    unittest.main()
