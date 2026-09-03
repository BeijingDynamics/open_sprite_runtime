import unittest

from open_sprite_runtime.safety import (
    RuntimeMode,
    SafetyInputs,
    SafetyLimits,
    SafetySupervisor,
)


class SafetySupervisorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.now = 1_000_000_000
        self.limits = SafetyLimits(6.0, 100.0, 2.0)

    def inputs(self, **overrides: object) -> SafetyInputs:
        values: dict[str, object] = {
            "now_ns": self.now,
            "state_timestamp_ns": self.now - 2_000_000,
            "command_timestamp_ns": self.now - 20_000_000,
            "policy_overrun_ms": 0.2,
            "allow_hardware_tx": True,
            "hardware_configured": True,
            "left_ankle_calibrated": True,
            "right_ankle_calibrated": True,
            "imu_valid": True,
            "estop_healthy": True,
            "motor_telemetry_healthy": True,
        }
        values.update(overrides)
        return SafetyInputs(**values)  # type: ignore[arg-type]

    def test_healthy_armed_state_permits_tx(self) -> None:
        decision = SafetySupervisor(RuntimeMode.ARMED, self.limits).evaluate(self.inputs())
        self.assertTrue(decision.hardware_tx_permitted)
        self.assertFalse(decision.safe_hold_required)

    def test_shadow_mode_never_permits_tx(self) -> None:
        decision = SafetySupervisor(RuntimeMode.SHADOW, self.limits).evaluate(self.inputs())
        self.assertFalse(decision.hardware_tx_permitted)
        self.assertIn("mode_shadow_no_tx", decision.blockers)

    def test_stale_state_latches_and_requires_explicit_clear(self) -> None:
        supervisor = SafetySupervisor(RuntimeMode.ARMED, self.limits)
        stale = supervisor.evaluate(
            self.inputs(state_timestamp_ns=self.now - 7_000_000)
        )
        self.assertIn("state_stale", stale.latched_faults)
        still_blocked = supervisor.evaluate(self.inputs())
        self.assertIn("state_stale", still_blocked.blockers)
        supervisor.clear_latched_faults(("state_stale",))
        self.assertTrue(supervisor.evaluate(self.inputs()).hardware_tx_permitted)

    def test_command_timeout_forces_hold_without_latching(self) -> None:
        supervisor = SafetySupervisor(RuntimeMode.ARMED, self.limits)
        stale = supervisor.evaluate(
            self.inputs(command_timestamp_ns=self.now - 101_000_000)
        )
        self.assertTrue(stale.safe_hold_required)
        self.assertIn("command_stale", stale.blockers)
        self.assertNotIn("command_stale", stale.latched_faults)
        self.assertTrue(supervisor.evaluate(self.inputs()).hardware_tx_permitted)

    def test_estop_and_policy_overrun_latch(self) -> None:
        supervisor = SafetySupervisor(RuntimeMode.ARMED, self.limits)
        decision = supervisor.evaluate(
            self.inputs(estop_healthy=False, policy_overrun_ms=2.1)
        )
        self.assertEqual(
            set(decision.latched_faults), {"estop_unhealthy", "policy_overrun"}
        )
        self.assertFalse(supervisor.evaluate(self.inputs()).hardware_tx_permitted)

    def test_motor_limit_latches(self) -> None:
        supervisor = SafetySupervisor(RuntimeMode.ARMED, self.limits)
        decision = supervisor.evaluate(self.inputs(motor_telemetry_healthy=False))
        self.assertIn("motor_limit", decision.latched_faults)
        self.assertFalse(supervisor.evaluate(self.inputs()).hardware_tx_permitted)

    def test_future_timestamp_is_rejected(self) -> None:
        supervisor = SafetySupervisor(RuntimeMode.SHADOW, self.limits)
        with self.assertRaisesRegex(ValueError, "future"):
            supervisor.evaluate(self.inputs(state_timestamp_ns=self.now + 1))


if __name__ == "__main__":
    unittest.main()
