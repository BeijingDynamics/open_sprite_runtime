import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from xbox_shadow_completion import completed_rejected_shadow, can_diagnostics


class CompletionTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name) / 'can.json'
        self.report = dict(mode='native_xbox_500hz_zero_gain_can_shadow', passed=False,
                           elapsed_s=120.001, state_ticks=60000,
                           nonzero_gain_or_torque_tx_attempts=0, automatic_enable_attempts=0,
                           automatic_mode_switch_attempts=0, transport_slot_overruns=2)

    def save(self):
        self.path.write_text(json.dumps(self.report))

    def test_completed_rejection_preserves_evidence_not_pass(self):
        self.save()
        self.assertTrue(completed_rejected_shadow(self.path, 2, 120))
        self.assertFalse(json.loads(self.path.read_text())['passed'])
        self.assertEqual(can_diagnostics(self.path)['can_qualification_reasons'],
                         ['transport_slot_overruns'])

    def test_early_or_unsafe_exit_is_not_deferred(self):
        for key, value in [('elapsed_s', 1), ('state_ticks', 59999),
                           ('nonzero_gain_or_torque_tx_attempts', 1),
                           ('automatic_enable_attempts', 1),
                           ('automatic_mode_switch_attempts', 1), ('mode', 'powered')]:
            with self.subTest(key=key):
                old = self.report[key]
                self.report[key] = value
                self.save()
                self.assertFalse(completed_rejected_shadow(self.path, 2, 120))
                self.report[key] = old

    def test_fault_and_missing_reports_are_not_deferred(self):
        self.assertFalse(completed_rejected_shadow(self.path, 2, 120))
        self.path.write_text('{')
        self.assertFalse(completed_rejected_shadow(self.path, 2, 120))
        self.save()
        for rc in (1, -15, None, 0):
            self.assertFalse(completed_rejected_shadow(self.path, rc, 120))


if __name__ == '__main__':
    unittest.main()
