import unittest

from open_sprite_runtime.telemetry import (
    MotorTelemetry,
    MotorTelemetryLimits,
    evaluate_motor_bank,
    evaluate_motor_telemetry,
    limits_from_hardware_record,
)


class MotorTelemetryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.limits = MotorTelemetryLimits(
            hard_position_rad=(-1.0, 1.0),
            maximum_speed_rad_s=10.0,
            peak_torque_nm=40.0,
            peak_current_a=12.0,
            maximum_temperature_c=80.0,
            torque_speed_envelope=((0.0, 40.0), (4.0, 14.0), (10.0, 0.0)),
        )
        self.sample = MotorTelemetry(0.0, 2.0, 10.0, 3.0, 40.0)

    def test_healthy_sample_has_positive_margins(self) -> None:
        decision = evaluate_motor_telemetry(self.sample, self.limits)
        self.assertTrue(decision.healthy)
        self.assertTrue(all(value >= 0.0 for value in decision.margins.values()))

    def test_torque_speed_envelope_is_enforced(self) -> None:
        decision = evaluate_motor_telemetry(
            MotorTelemetry(0.0, 7.0, 8.0, 3.0, 40.0), self.limits
        )
        self.assertFalse(decision.healthy)
        self.assertIn("torque_speed_nm", decision.violations)

    def test_nonfinite_fails_closed(self) -> None:
        decision = evaluate_motor_telemetry(
            MotorTelemetry(float("nan"), 0.0, 0.0, 0.0, 20.0), self.limits
        )
        self.assertEqual(decision.violations, ("nonfinite",))

    def test_motor_bank_requires_exact_inventory(self) -> None:
        report = evaluate_motor_bank(
            {"motor_a": self.sample}, {"motor_a": self.limits}, ("motor_a", "motor_b")
        )
        self.assertFalse(report["healthy"])
        self.assertEqual(report["missing_motors"], ["motor_b"])

    def test_nameplate_record_builds_conservative_envelope(self) -> None:
        limits = limits_from_hardware_record(
            {
                "hard_limit_rad": [-1.0, 1.0],
                "rated_torque_nm": 14.0,
                "peak_torque_nm": 40.0,
                "rated_speed_rad_s": 3.8,
                "max_speed_rad_s": 10.0,
                "peak_current_a": 12.0,
                "temperature_limit_c": 80.0,
            }
        )
        self.assertEqual(limits.torque_limit_at_speed(0.0), 40.0)
        self.assertEqual(limits.torque_limit_at_speed(3.8), 14.0)
        self.assertEqual(limits.torque_limit_at_speed(10.0), 0.0)


if __name__ == "__main__":
    unittest.main()
