import struct
import unittest
from open_sprite_runtime.native_imu_packet import decode_packet


def packet():
    p=bytearray(192);p[:8]=b'SPIMU001'
    struct.pack_into('<QQQ',p,8,1,1_000_000_000,1_001_000_000)
    struct.pack_into('<h',p,32+24,16384)
    struct.pack_into('<13d',p,80,0,0,0,0,0,0,1,0,0,0,0,0,0)
    return p


class PacketTests(unittest.TestCase):
    def test_valid_and_timestamp_preserved(self):
        value=decode_packet(packet(),1_010_000_000)
        self.assertEqual(value.sample.read_start_ns,1_000_000_000)
    def test_expired_and_future(self):
        for now in (1_031_000_000,999_000_000):
            with self.assertRaises(RuntimeError):decode_packet(packet(),now)
    def test_regression(self):
        with self.assertRaises(ValueError):decode_packet(packet(),1_010_000_000,2)
    def test_corrupt(self):
        for index in (0,78,80,184):
            p=packet();p[index]=255
            # A low mantissa-byte perturbation can be below the numeric tolerance.
            if index==80:struct.pack_into('<d',p,80,1.0)
            with self.assertRaises(ValueError):decode_packet(p,1_010_000_000)
    def test_truncated(self):
        with self.assertRaises(ValueError):decode_packet(packet()[:-1],1_010_000_000)
    def test_bad_quaternion(self):
        p=packet();struct.pack_into('<h',p,56,0)
        with self.assertRaises(RuntimeError):decode_packet(p,1_010_000_000)


if __name__=='__main__':unittest.main()
