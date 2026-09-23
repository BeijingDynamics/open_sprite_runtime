import hashlib
import tempfile
import unittest
from pathlib import Path

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.limit_derivation import apply_limit_candidates, derive_limit_candidates
from tests.test_contracts import valid_policy_data
from tests.test_hardware import JOINTS, complete_hardware


class LimitDerivationTests(unittest.TestCase):
    def _fixture(self, directory: Path) -> tuple[PolicyContract, Path]:
        urdf = directory / "robot.urdf"
        joints = "".join(
            f'<joint name="{name}" type="revolute"><parent link="a"/><child link="b"/>'
            f'<limit lower="-1" upper="2" effort="1" velocity="1"/></joint>'
            for name in JOINTS
        )
        urdf.write_text(f'<robot name="test">{joints}</robot>', encoding="utf-8")
        data = valid_policy_data()
        data["joint_names"] = list(JOINTS)
        data["asset_urdf_sha256"] = hashlib.sha256(urdf.read_bytes()).hexdigest()
        return PolicyContract(path=None, data=data), urdf  # type: ignore[arg-type]

    def test_derives_all_direct_and_differential_motor_boxes(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            contract, urdf = self._fixture(Path(raw))
            report = derive_limit_candidates(complete_hardware(), contract, urdf)
        self.assertEqual(len(report["joint_limits"]), 31)
        self.assertEqual(len(report["motor_limits"]), 31)
        self.assertFalse(report["confirmed_physical_hard_limits"])
        self.assertEqual(
            report["differential_pairs"]["head"]["joint_order"],
            ["head_pitch_joint", "head_roll_joint"],
        )
        self.assertEqual(report["joint_limits"][JOINTS[0]]["soft_limit_rad_candidate"], [-0.95, 1.95])
        self.assertEqual(
            report["motor_limits"]["left_ankle_motor_a"]["topology"], "differential"
        )

    def test_rejects_asset_hash_drift(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            contract, urdf = self._fixture(Path(raw))
            contract.data["asset_urdf_sha256"] = "0" * 64
            with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
                derive_limit_candidates(complete_hardware(), contract, urdf)

    def test_joint_soft_override_updates_differential_motor_boxes(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            contract, urdf = self._fixture(Path(raw))
            baseline = derive_limit_candidates(complete_hardware(), contract, urdf)
            report = derive_limit_candidates(
                complete_hardware(),
                contract,
                urdf,
                joint_soft_limit_overrides={"left_ankle_pitch_joint": (-0.9, 1.99)},
            )
        self.assertEqual(
            report["joint_limits"]["left_ankle_pitch_joint"]["soft_limit_rad_candidate"],
            [-0.9, 1.99],
        )
        self.assertEqual(
            report["joint_soft_limit_overrides"],
            {"left_ankle_pitch_joint": [-0.9, 1.99]},
        )
        self.assertNotEqual(
            report["motor_limits"]["left_ankle_motor_a"]["soft_limit_rad_candidate"],
            baseline["motor_limits"]["left_ankle_motor_a"]["soft_limit_rad_candidate"],
        )

    def test_joint_soft_override_must_remain_inside_urdf_hard_limit(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            contract, urdf = self._fixture(Path(raw))
            with self.assertRaisesRegex(ValueError, "strictly inside URDF hard limits"):
                derive_limit_candidates(
                    complete_hardware(),
                    contract,
                    urdf,
                    joint_soft_limit_overrides={"left_ankle_pitch_joint": (-1.0, 1.9)},
                )

    def test_applies_strictly_nested_candidates_with_provenance(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            contract, urdf = self._fixture(Path(raw))
            hardware = complete_hardware()
            report = derive_limit_candidates(hardware, contract, urdf)
            self.assertEqual(apply_limit_candidates(hardware, report), 62)
        for record in hardware["motor_map"].values():
            hard = record["hard_limit_rad"]
            soft = record["soft_limit_rad"]
            self.assertLess(hard[0], soft[0])
            self.assertLess(soft[1], hard[1])
            self.assertFalse(record["limit_provenance"]["physical_hard_stop_measured"])

    def test_apply_rejects_incomplete_motor_set(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            contract, urdf = self._fixture(Path(raw))
            hardware = complete_hardware()
            report = derive_limit_candidates(hardware, contract, urdf)
            report["motor_limits"].pop(next(iter(report["motor_limits"])))
            with self.assertRaisesRegex(ValueError, "motor set mismatch"):
                apply_limit_candidates(hardware, report)


if __name__ == "__main__":
    unittest.main()
