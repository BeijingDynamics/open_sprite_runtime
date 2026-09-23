import math
import unittest

from open_sprite_runtime.damiao import DamiaoFeedbackEndpoint, DamiaoMitRanges
from open_sprite_runtime.single_joint_hold import (
    _quintic_smoothstep,
    run_head_yaw_low_gain_hold,
    run_head_yaw_low_gain_motion,
    run_right_hip_yaw_low_gain_motion,
    run_right_wrist_roll_low_gain_hold,
    run_right_wrist_roll_low_gain_motion,
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


class FakeWriter:
    def __init__(
        self, endpoint, *, torque_raw=2048, stale_enabled_after_disable=0,
        track_command=False,
    ):
        self.endpoint = endpoint
        self.interface = endpoint.interface
        self.status = 0
        self.torque_raw = torque_raw
        self.command_tx_attempts = 0
        self.enable_attempts = 0
        self.disable_attempts = 0
        self.stale_enabled_after_disable = stale_enabled_after_disable
        self.commands = []
        self.track_command = track_command
        self.position_raw = 32768

    def send_command(self, command):
        self.command_tx_attempts += 1
        self.commands.append(command)
        if self.track_command and self.status == 1:
            self.position_raw = (command.data[0] << 8) | command.data[1]

    def send_enable(self):
        self.enable_attempts += 1
        self.status = 1

    def send_disable(self):
        self.disable_attempts += 1
        self.status = 0

    def receive(self):
        position_raw = self.position_raw
        velocity_raw = 2048
        reported_status = self.status
        if self.status == 0 and self.enable_attempts and self.stale_enabled_after_disable:
            reported_status = 1
            self.stale_enabled_after_disable -= 1
        data = bytes((
            (reported_status << 4) | self.endpoint.can_id,
            position_raw >> 8,
            position_raw & 0xFF,
            velocity_raw >> 4,
            ((velocity_raw & 0xF) << 4) | (self.torque_raw >> 8),
            self.torque_raw & 0xFF,
            30,
            29,
        ))
        return ReceivedCanFrame(
            interface=self.endpoint.interface, can_id=self.endpoint.master_id, data=data,
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

    def test_shutdown_drains_stale_enabled_feedback_until_disabled(self):
        clock = Clock()
        writer = FakeWriter(endpoint(), stale_enabled_after_disable=3)
        report = run_head_yaw_low_gain_hold(
            writer, endpoint(), soft_position_rad=(-1.0, 1.0),
            monotonic=clock, sleep=clock.sleep,
        )
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(report.disable_attempts, 4)
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

    def test_right_wrist_roll_loaded_hold_is_separately_allowlisted(self):
        wrist = DamiaoFeedbackEndpoint(
            "right_wrist_roll_motor", "kcan4", 7, 0x17, endpoint().ranges
        )
        clock = Clock()
        report = run_right_wrist_roll_low_gain_hold(
            FakeWriter(wrist), wrist, soft_position_rad=(-0.7, 0.7),
            monotonic=clock, sleep=clock.sleep,
        )
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(report.motor_name, "right_wrist_roll_motor")
        self.assertEqual(report.final_status, "disabled")

    def test_right_wrist_roll_wrapper_rejects_neighbor_motor(self):
        wrist_pitch = DamiaoFeedbackEndpoint(
            "right_wrist_pitch_motor", "kcan4", 6, 0x16, endpoint().ranges
        )
        with self.assertRaisesRegex(ValueError, "right_wrist_roll_motor"):
            run_right_wrist_roll_low_gain_hold(
                FakeWriter(wrist_pitch), wrist_pitch, soft_position_rad=(-0.7, 0.7)
            )


class SingleJointMotionTests(unittest.TestCase):
    def test_quintic_smoothstep_has_zero_endpoint_velocity(self):
        self.assertEqual(_quintic_smoothstep(0.0), (0.0, 0.0))
        self.assertEqual(_quintic_smoothstep(1.0), (1.0, 0.0))
        position, derivative = _quintic_smoothstep(0.5)
        self.assertAlmostEqual(position, 0.5)
        self.assertAlmostEqual(derivative, 1.875)

    def test_frozen_motion_disables_and_passes(self):
        clock = Clock()
        writer = FakeWriter(endpoint())
        report = run_head_yaw_low_gain_motion(
            writer, endpoint(), soft_position_rad=(-1.0, 1.0),
            monotonic=clock, sleep=clock.sleep,
        )
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(report.duration_s, 4.5)
        self.assertEqual(report.enable_attempts, 1)
        self.assertGreaterEqual(report.disable_attempts, 3)
        self.assertEqual(report.final_status, "disabled")
        self.assertGreater(report.command_count, 200)
        positions = []
        low, high = endpoint().ranges.position_rad
        for command in writer.commands:
            raw = (command.data[0] << 8) | command.data[1]
            positions.append(low + raw / 65535.0 * (high - low))
        self.assertGreater(max(positions) - report.initial_position_rad, 0.019)
        self.assertLess(min(positions) - report.initial_position_rad, -0.019)

    def test_motion_torque_guard_fails_closed_and_disables(self):
        clock = Clock()
        writer = FakeWriter(endpoint(), torque_raw=2200)
        report = run_head_yaw_low_gain_motion(
            writer, endpoint(), soft_position_rad=(-1.0, 1.0),
            monotonic=clock, sleep=clock.sleep,
        )
        self.assertFalse(report.passed)
        self.assertTrue(any("torque guard" in value for value in report.errors))
        self.assertGreaterEqual(report.disable_attempts, 3)
        self.assertEqual(report.final_status, "disabled")

    def test_motion_shutdown_drains_stale_enabled_feedback(self):
        clock = Clock()
        writer = FakeWriter(endpoint(), stale_enabled_after_disable=3)
        report = run_head_yaw_low_gain_motion(
            writer, endpoint(), soft_position_rad=(-1.0, 1.0),
            monotonic=clock, sleep=clock.sleep,
        )
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(report.disable_attempts, 4)

    def test_motion_rejects_gain_or_excursion_changes(self):
        with self.assertRaisesRegex(ValueError, "frozen"):
            run_head_yaw_low_gain_motion(
                FakeWriter(endpoint()), endpoint(), soft_position_rad=(-1.0, 1.0), kp=1.1
            )
        with self.assertRaisesRegex(ValueError, "frozen"):
            run_head_yaw_low_gain_motion(
                FakeWriter(endpoint()), endpoint(), soft_position_rad=(-1.0, 1.0),
                excursion_rad=0.03,
            )

    def test_motion_rejects_insufficient_soft_limit_margin(self):
        clock = Clock()
        writer = FakeWriter(endpoint())
        report = run_head_yaw_low_gain_motion(
            writer, endpoint(), soft_position_rad=(-0.01, 0.01),
            monotonic=clock, sleep=clock.sleep,
        )
        self.assertFalse(report.passed)
        self.assertTrue(any("soft-limit margin" in value for value in report.errors))
        self.assertEqual(report.enable_attempts, 0)
        self.assertGreaterEqual(report.disable_attempts, 3)

    def test_visible_ten_degree_profile_tracks_both_sides_and_disables(self):
        clock = Clock()
        writer = FakeWriter(endpoint(), track_command=True)
        report = run_head_yaw_low_gain_motion(
            writer,
            endpoint(),
            soft_position_rad=(-1.0, 1.0),
            excursion_rad=math.radians(10.0),
            transition_s=4.0,
            dwell_s=1.0,
            kp=2.0,
            maximum_position_error_rad=0.08,
            maximum_velocity_rad_s=0.8,
            maximum_torque_nm=0.25,
            monotonic=clock,
            sleep=clock.sleep,
        )
        self.assertTrue(report.passed, report.errors)
        self.assertGreater(
            report.measured_maximum_position_rad - report.initial_position_rad,
            math.radians(9.9),
        )
        self.assertLess(
            report.measured_minimum_position_rad - report.initial_position_rad,
            -math.radians(9.9),
        )
        self.assertAlmostEqual(
            report.final_measured_position_rad, report.initial_position_rad, delta=0.001
        )
        self.assertEqual(report.final_status, "disabled")

    def test_right_wrist_roll_motion_applies_negative_motor_sign(self):
        wrist = DamiaoFeedbackEndpoint(
            "right_wrist_roll_motor", "kcan4", 7, 0x17, endpoint().ranges
        )
        clock = Clock()
        writer = FakeWriter(wrist, track_command=True)
        report = run_right_wrist_roll_low_gain_motion(
            writer, wrist, soft_position_rad=(-0.7, 0.7),
            monotonic=clock, sleep=clock.sleep,
        )
        self.assertTrue(report.passed, report.errors)
        enabled_positions = []
        low, high = wrist.ranges.position_rad
        for command in writer.commands[2:]:
            raw = (command.data[0] << 8) | command.data[1]
            enabled_positions.append(low + raw / 65535.0 * (high - low))
        first_extreme = min(enabled_positions[:200])
        self.assertLess(first_extreme - report.initial_position_rad, -math.radians(4.9))
        self.assertGreater(
            report.measured_maximum_position_rad - report.initial_position_rad,
            math.radians(4.9),
        )
        self.assertEqual(report.final_status, "disabled")

    def test_right_hip_yaw_motion_tracks_both_sides_and_disables(self):
        hip = DamiaoFeedbackEndpoint(
            "right_hip_yaw_motor", "kcan2", 3, 0x13, endpoint().ranges
        )
        clock = Clock()
        writer = FakeWriter(hip, track_command=True)
        report = run_right_hip_yaw_low_gain_motion(
            writer, hip, soft_position_rad=(-2.69, 2.69),
            monotonic=clock, sleep=clock.sleep,
        )
        self.assertTrue(report.passed, report.errors)
        self.assertGreater(
            report.measured_maximum_position_rad - report.initial_position_rad,
            math.radians(4.9),
        )
        self.assertLess(
            report.measured_minimum_position_rad - report.initial_position_rad,
            -math.radians(4.9),
        )
        self.assertAlmostEqual(
            report.final_measured_position_rad, report.initial_position_rad, delta=0.001
        )
        self.assertEqual(report.final_status, "disabled")


if __name__ == "__main__":
    unittest.main()
