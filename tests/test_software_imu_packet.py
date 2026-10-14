import struct
import unittest
from open_sprite_runtime.native_imu_packet import decode_packet, decode_software_packet


def packet():
    p=bytearray(192);p[:8]=b'SPIMU002'
    struct.pack_into('<QQQ',p,8,1,1_000_000_000,1_001_000_000)
    struct.pack_into('<3h',p,32,0,0,981)
    struct.pack_into('<3h',p,44,90,-180,270)
    struct.pack_into('<3d',p,56,.01,-.02,.03)
    struct.pack_into('<13d',p,80,0,0,9.81,.09,-.18,.27,1,0,0,0,0,0,9.80665)
    return p


class SoftwarePacketTests(unittest.TestCase):
    def test_valid(self):
        value=decode_software_packet(packet(),1_010_000_000)
        self.assertAlmostEqual(value.sample.gyro_p0_rad_s[0],.09)
        self.assertEqual(value.sample.read_start_ns,1_000_000_000)
    def test_rejects(self):
        for offset in (0,50,56,80,128,160,184):
            p=packet()
            if offset in (56,80,128,160):struct.pack_into('<d',p,offset,2)
            else:p[offset]=255
            with self.assertRaises(ValueError):decode_software_packet(p,1_010_000_000)
        with self.assertRaises(ValueError):decode_packet(packet(),1_010_000_000)
        with self.assertRaises(ValueError):decode_software_packet(packet(),1_010_000_000,2)
        for now in (999_000_000,1_031_000_000):
            with self.assertRaises(RuntimeError):decode_software_packet(packet(),now)


if __name__=='__main__':unittest.main()
