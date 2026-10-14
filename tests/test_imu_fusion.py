import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from open_sprite_runtime.imu_fusion import build_fusion_command, configure


class FusionTests(unittest.TestCase):
    def test_vendor_frames(self):
        self.assertEqual(build_fusion_command(6), bytes.fromhex('7e 23 07 61 06 5f 6e'))
        self.assertEqual(build_fusion_command(9), bytes.fromhex('7e 23 07 61 09 5f 71'))

    def test_invalid_modes(self):
        for mode in (0, 7, True, 6.0, '6'):
            with self.assertRaises(ValueError):
                build_fusion_command(mode)

    def test_unarmed_has_no_serial_access(self):
        with self.assertRaises(ValueError):
            configure('/dev/test', 115200, 6, '', 'unused.json')

    def test_write_failure_is_saved_not_reported_as_success(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch('serial.Serial', side_effect=OSError('busy')):
                r = configure('/dev/test', 115200, 6, 'WRITE_IMU_CONFIGURATION',
                              Path(directory)/'result.json')
            self.assertFalse(r['write_completed'])
            self.assertFalse(r['mode_readback_verified'])
            self.assertEqual(r['errors'], ['busy'])


if __name__ == '__main__':
    unittest.main()
