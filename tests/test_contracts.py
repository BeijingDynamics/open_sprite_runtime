import copy
import unittest

from open_sprite_runtime.contracts import PolicyContract, RuntimeTiming
from open_sprite_runtime.safety import SafetyState


class RuntimeTimingTests(unittest.TestCase):
    def test_qualified_multirate_plan(self) -> None:
        timing = RuntimeTiming(policy_hz=50, state_hz=500, motor_internal_hz=1000)
        timing.validate()
        self.assertEqual(timing.interpolation_steps, 10)

    def test_rejects_non_integer_rate_relationship(self) -> None:
        with self.assertRaises(ValueError):
            RuntimeTiming(policy_hz=50, state_hz=400, motor_internal_hz=1000).validate()

    def test_default_safety_state_cannot_arm(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "hardware arm blocked"):
            SafetyState().require_armed()


def valid_policy_data() -> dict:
    terms = []
    cursor = 0
    for name, frame_dim, history in (
        ("joint_pos", 31, 8),
        ("joint_vel", 31, 8),
        ("actions", 31, 8),
        ("base_ang_vel", 3, 8),
        ("projected_gravity", 3, 8),
        ("velocity_commands", 3, 0),
    ):
        dim = frame_dim * max(history, 1)
        terms.append(
            {
                "name": name,
                "start": cursor,
                "end": cursor + dim,
                "shape": [dim],
                "frame_dim": frame_dim,
                "history_length": history,
            }
        )
        cursor += dim
    data = {
        "policy_dt": 0.02,
        "physics_dt": 0.002,
        "decimation": 10,
        "action_dim": 31,
        "actor_observation_dim": 795,
        "observation_has_horizontal_base_velocity": False,
        "observation_has_global_position": False,
        "observation_has_global_yaw": False,
        "observation_terms": terms,
        "joint_names": [f"joint_{index}" for index in range(31)],
        "action_formula": "target_joint_pos = action_offset + action_scale * policy_action",
        "base_ang_vel_frame": "root_link_local",
        "projected_gravity_definition": "R_world_to_base @ [0, 0, -1]",
        "quaternion_order": "wxyz",
        "command_layout": ["vx", "vy", "yaw_rate"],
        "physical_ankle_differential": {
            "enabled": True,
            "rated_torque_nm": 3.5,
            "peak_torque_nm": 12.5,
            "rated_speed_rad_s": 12.56,
            "no_load_speed_rad_s": 36.2,
            "joint_to_motor_ratio": 1.0,
        },
    }
    for name, value in (
        ("default_joint_pos", 0.0),
        ("action_offset", 0.0),
        ("action_scale", 0.2),
        ("stiffness", 10.0),
        ("damping", 1.0),
        ("effort_limit", 12.5),
        ("velocity_limit", 9.3),
    ):
        data[name] = [value] * 31
    return data


class PolicyContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.timing = RuntimeTiming(policy_hz=50, state_hz=500, motor_internal_hz=1000)

    def contract(self, data: dict) -> PolicyContract:
        return PolicyContract(path=None, data=data)  # type: ignore[arg-type]

    def test_accepts_complete_50hz_hardware_contract(self) -> None:
        self.contract(valid_policy_data()).validate_for_hardware(self.timing)

    def test_rejects_global_or_horizontal_state(self) -> None:
        for field in (
            "observation_has_horizontal_base_velocity",
            "observation_has_global_position",
            "observation_has_global_yaw",
        ):
            with self.subTest(field=field):
                data = valid_policy_data()
                data[field] = True
                with self.assertRaises(ValueError):
                    self.contract(data).validate_for_hardware(self.timing)

    def test_rejects_observation_slice_or_order_drift(self) -> None:
        data = valid_policy_data()
        data["observation_terms"] = copy.deepcopy(data["observation_terms"])
        data["observation_terms"][1]["start"] += 1
        data["observation_terms"][3]["name"] = "base_lin_vel"
        with self.assertRaisesRegex(ValueError, "not contiguous"):
            self.contract(data).validate_for_hardware(self.timing)

    def test_rejects_joint_vector_or_ankle_contract_drift(self) -> None:
        data = valid_policy_data()
        data["action_scale"] = data["action_scale"][:-1]
        data["physical_ankle_differential"]["peak_torque_nm"] = 25.0
        with self.assertRaisesRegex(ValueError, "action_scale"):
            self.contract(data).validate_for_hardware(self.timing)


if __name__ == "__main__":
    unittest.main()
