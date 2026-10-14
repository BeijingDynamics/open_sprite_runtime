import struct
import unittest
import numpy as np
from open_sprite_runtime.bno055 import BNO055, CONFIRM, EXPECTED, P0_FROM_CHIP, decode_sample
from open_sprite_runtime.imu import ImuMount, matrix_to_quaternion_wxyz


class Registers:
    def __init__(self):
        self.regs={**EXPECTED,0:0xA0,0x39:5,0x3A:0,0x35:0xFC}
        self.writes=[]
    def read(self,reg,count=1):
        return bytes([self.regs.get(reg,0)])
    def write(self,reg,value):
        self.writes.append((reg,value))
        self.regs[reg]=value


def data(q=(1,0,0,0)):
    b=bytearray(46)
    struct.pack_into('<3h',b,0,100,-200,981)
    struct.pack_into('<3h',b,12,900,-1800,450)
    struct.pack_into('<4h',b,24,*(round(x*16384) for x in q))
    struct.pack_into('<3h',b,38,0,0,981)
    struct.pack_into('<b',b,44,-2)
    b[45]=0xFC
    return bytes(b)


class BNOTests(unittest.TestCase):
    def test_si_and_quaternion_decode(self):
        s=decode_sample(data(),100,200)
        self.assertEqual(s.gyro_p0_rad_s,(1,-2,.5))
        self.assertEqual(s.acceleration_p0_m_s2,(1,-2,9.81))
        self.assertEqual(s.temperature_c,-2)
        self.assertEqual(s.calibration,dict(system=3,gyro=3,accel=3,mag=0))

    def test_bad_sample(self):
        for b,start,end in [(b'bad',0,1),(data((0,0,0,0)),0,1),(data(),0,30_000_001),(data(),2,1)]:
            with self.assertRaises((ValueError,RuntimeError)):
                decode_sample(b,start,end)

    def test_probe_never_configures(self):
        io=Registers()
        self.assertTrue(BNO055(io).inspect()['matches'])
        self.assertEqual(io.writes,[])

    def test_configure_requires_confirmation_and_chip_id(self):
        io=Registers()
        with self.assertRaises(ValueError): BNO055(io).configure('')
        io.regs[0]=0
        with self.assertRaises(RuntimeError): BNO055(io).configure(CONFIRM)
        self.assertEqual(io.writes,[])

    def test_configure_sequence_readback(self):
        io=Registers(); sleeps=[]
        result=BNO055(io,sleep=sleeps.append).configure(CONFIRM)
        self.assertTrue(result['matches'])
        self.assertEqual(io.writes,[(0x3D,0),(7,0),(0x3E,0),(0x3B,2),(0x41,0x21),(0x42,4),(0x3D,8)])
        self.assertGreaterEqual(sleeps[0],.019)
        self.assertGreaterEqual(sleeps[-1],.007)

    def test_bad_readback(self):
        class Broken(Registers):
            def write(self,reg,value): pass
        io=Broken(); io.regs[0x41]=0x24
        with self.assertRaises(RuntimeError): BNO055(io,sleep=lambda _:None).configure(CONFIRM)

    def test_p0_mount_upright_and_vector(self):
        mount=ImuMount.sprite0825_rear_pelvis().sensor_to_body_matrix
        q=matrix_to_quaternion_wxyz(mount@P0_FROM_CHIP.T)
        state=decode_sample(data(q),0,1).body_state('chip-axes')
        np.testing.assert_allclose(state['projected_gravity'],[0,0,-1],atol=1e-4)
        np.testing.assert_allclose(state['angular_velocity_body_rad_s'],mount@P0_FROM_CHIP.T@np.array([1,-2,.5]))

    def test_p0_output_mount_upright(self):
        q=matrix_to_quaternion_wxyz(ImuMount.sprite0825_rear_pelvis().sensor_to_body_matrix)
        state=decode_sample(data(q),0,1).body_state('p0-output-axes')
        np.testing.assert_allclose(state['projected_gravity'],[0,0,-1],atol=1e-4)

    def test_wrong_page(self):
        io=Registers(); io.regs[7]=1
        with self.assertRaises(RuntimeError): BNO055(io).inspect()
        self.assertEqual(io.writes,[])


if __name__=='__main__': unittest.main()
