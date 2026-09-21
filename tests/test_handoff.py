import unittest

import numpy as np

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.handoff import MeasuredPoseActionHandoff
from tests.test_contracts import valid_policy_data


def contract_with_handoff(seconds: float = 0.04) -> PolicyContract:
    data = valid_policy_data()
    data["deployment_handoff_seconds"] = seconds
    data["deployment_handoff_mode"] = "smoothstep_from_pose_equivalent_action"
    data["deployment_initial_velocity_mode"] = "zero"
    data["action_offset"] = list(np.linspace(-0.2, 0.2, 31))
    data["action_scale"] = list(np.linspace(0.1, 0.4, 31))
    return PolicyContract(path=None, data=data)  # type: ignore[arg-type]


class MeasuredPoseActionHandoffTests(unittest.TestCase):
    def test_matches_two_tick_mujoco_smoothstep_handoff(self) -> None:
        contract = contract_with_handoff()
        handoff = MeasuredPoseActionHandoff(contract)
        offset = np.asarray(contract.data["action_offset"])
        scale = np.asarray(contract.data["action_scale"])
        measured = offset + scale * 0.25
        policy = np.full(31, 1.25)

        first = handoff.blend(policy, measured)
        second = handoff.blend(policy, measured + 10.0)

        self.assertEqual(handoff.total_steps, 2)
        np.testing.assert_allclose(first, 0.75)
        np.testing.assert_allclose(second, policy)
        self.assertTrue(handoff.complete)

    def test_zero_duration_uses_policy_action_immediately(self) -> None:
        handoff = MeasuredPoseActionHandoff(contract_with_handoff(0.0))
        policy = np.linspace(-1.0, 1.0, 31)
        np.testing.assert_allclose(handoff.blend(policy, np.zeros(31)), policy)
        self.assertTrue(handoff.complete)

    def test_rejects_unsupported_contract_semantics(self) -> None:
        contract = contract_with_handoff()
        contract.data["deployment_handoff_mode"] = "linear"
        with self.assertRaisesRegex(ValueError, "unsupported deployment handoff mode"):
            MeasuredPoseActionHandoff(contract)

    def test_hardware_contract_validation_rejects_missing_handoff(self) -> None:
        contract = contract_with_handoff()
        del contract.data["deployment_handoff_seconds"]
        from open_sprite_runtime.contracts import RuntimeTiming

        with self.assertRaisesRegex(ValueError, "deployment_handoff_seconds"):
            contract.validate_for_hardware(RuntimeTiming(50, 500, 1000))

    def test_rejects_nonfinite_input(self) -> None:
        handoff = MeasuredPoseActionHandoff(contract_with_handoff())
        action = np.zeros(31)
        action[4] = np.nan
        with self.assertRaisesRegex(ValueError, "policy_action"):
            handoff.blend(action, np.zeros(31))


if __name__ == "__main__":
    unittest.main()
