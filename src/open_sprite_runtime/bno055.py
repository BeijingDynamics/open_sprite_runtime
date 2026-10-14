"""BNO055 IMUPLUS/P0 commissioning only. No CAN or live-policy connection.

Bosch BST-BNO055-DS000-18, sections 3.3, 3.4, 3.6 and register map.
"""
from dataclasses import asdict, dataclass
import argparse
import json
import math
from pathlib import Path
import struct
import time

import numpy as np

from .imu import ImuMount, matrix_to_quaternion_wxyz, quaternion_wxyz_to_matrix

CONFIRM = 'CONFIGURE_BNO055_IMUPLUS_P0'
# v_p0 = P0_FROM_CHIP @ v_chip; P0 config=0x21, sign=0x04.
P0_FROM_CHIP = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
EXPECTED = {0x07: 0, 0x3B: 0x02, 0x3D: 0x08, 0x3E: 0, 0x41: 0x21, 0x42: 0x04}


class I2CRegisters:
    """Combined I2C_RDWR transactions, not SMBus's 32-byte block read."""
    def __init__(self, bus=7, address=0x28):
        if address not in (0x28, 0x29) or bus < 0:
            raise ValueError('Explicit BNO055 address 0x28/0x29 and nonnegative bus required')
        from smbus2 import SMBus, i2c_msg
        self.bus = SMBus(bus)
        self.address = address
        self.msg = i2c_msg

    def read(self, register, count=1):
        request = self.msg.write(self.address, [register])
        response = self.msg.read(self.address, count)
        self.bus.i2c_rdwr(request, response)
        return bytes(response)

    def write(self, register, value):
        self.bus.i2c_rdwr(self.msg.write(self.address, [register, value]))

    def close(self):
        self.bus.close()


@dataclass(frozen=True)
class Sample:
    read_start_ns: int
    read_end_ns: int
    acceleration_p0_m_s2: tuple
    gyro_p0_rad_s: tuple
    quaternion_p0_to_world_wxyz: tuple
    gravity_p0_m_s2: tuple
    temperature_c: int
    calibration: dict
    raw_hex: str

    def body_state(self, mount_basis):
        if mount_basis not in ('chip-axes', 'p0-output-axes'):
            raise ValueError('Select the axes to which physical mounting refers')
        body_from_chip = ImuMount.sprite0825_rear_pelvis().sensor_to_body_matrix
        body_from_output = (body_from_chip @ P0_FROM_CHIP.T
                            if mount_basis == 'chip-axes' else body_from_chip)
        rotation = quaternion_wxyz_to_matrix(self.quaternion_p0_to_world_wxyz) @ body_from_output.T
        return dict(angular_velocity_body_rad_s=(body_from_output @ self.gyro_p0_rad_s).tolist(),
                    acceleration_body_m_s2=(body_from_output @ self.acceleration_p0_m_s2).tolist(),
                    orientation_body_to_world_wxyz=matrix_to_quaternion_wxyz(rotation).tolist(),
                    projected_gravity=(rotation.T @ np.array([0., 0., -1.])).tolist(),
                    mounting_verified=False)


def decode_sample(data, start_ns, end_ns):
    # Page0 0x08..0x35: accel, mag, gyro, Euler, quaternion, linear accel,
    # gravity, temperature, calibration. One burst reduces inter-read skew;
    # it does NOT establish a device sampling timestamp or atomic snapshot.
    if len(data) != 46:
        raise ValueError('Expected 46-byte register burst')
    if end_ns < start_ns or end_ns-start_ns > 30_000_000:
        raise RuntimeError('I2C sample read invalid or older than 30ms')
    def vector(offset, divisor, count=3):
        return tuple(x/divisor for x in struct.unpack_from('<'+'h'*count, data, offset))
    quat = vector(24, 16384., 4)
    if abs(math.sqrt(sum(x*x for x in quat))-1.) > 0.05:
        raise RuntimeError('Invalid quaternion norm; fusion not ready or bad sample')
    cal = data[45]
    return Sample(start_ns, end_ns, vector(0,100.), vector(12,900.), quat,
                  vector(38,100.), struct.unpack_from('<b',data,44)[0],
                  {name: (cal >> shift) & 3 for name,shift in
                   [('system',6),('gyro',4),('accel',2),('mag',0)]}, data.hex())


class BNO055:
    def __init__(self, registers, sleep=time.sleep, clock_ns=time.monotonic_ns):
        self.io, self.sleep, self.clock_ns = registers, sleep, clock_ns

    def inspect(self):
        if self.io.read(0x07)[0] != 0:
            raise RuntimeError('BNO055 page is not0; refusing ambiguous register access')
        chip = self.io.read(0)[0]
        if chip != 0xA0:
            raise RuntimeError(f'BNO055 CHIP_ID mismatch: 0x{chip:02x}')
        config = {f'0x{reg:02x}': self.io.read(reg)[0] for reg in EXPECTED}
        return dict(chip_id=chip,registers=config,system_status=self.io.read(0x39)[0],
                    system_error=self.io.read(0x3A)[0],
                    calibration=self.io.read(0x35)[0],
                    matches=all(config[f'0x{r:02x}']==v for r,v in EXPECTED.items()))

    def configure(self, confirmation):
        if confirmation != CONFIRM:
            raise ValueError('Explicit BNO055 configuration confirmation required')
        self.inspect()  # Identify before ANY configuration write.
        self.io.write(0x3D,0)
        self.sleep(0.025)
        for reg,value in [(0x07,0),(0x3E,0),(0x3B,0x02),(0x41,0x21),(0x42,0x04)]:
            self.io.write(reg,value)
        self.io.write(0x3D,0x08)
        self.sleep(0.1)
        result = self.inspect()
        if not result['matches'] or result['system_error']:
            raise RuntimeError(f'Configuration readback failed: {result}')
        return result

    def read_sample(self):
        start = self.clock_ns()
        data = self.io.read(0x08,46)
        return decode_sample(data,start,self.clock_ns())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('probe','configure','record'))
    parser.add_argument('--bus',type=int,default=7,help='Linux adapter; Orin header I2C8 is usually bus7')
    parser.add_argument('--address',type=lambda x:int(x,0),default=0x28)
    parser.add_argument('--confirm',default='')
    parser.add_argument('--duration',type=float,default=60)
    parser.add_argument('--mount-basis',choices=('chip-axes','p0-output-axes'))
    parser.add_argument('--output')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 120:
        parser.error('duration must be in (0,120]')
    if args.action=='record' and (not args.mount_basis or not args.output):
        parser.error('record requires --mount-basis and --output')
    if args.action=='configure' and args.confirm != CONFIRM:
        parser.error(f'configure requires --confirm {CONFIRM}')
    output = None
    if args.output:
        path=Path(args.output)
        path.parent.mkdir(parents=True,exist_ok=True)
        output=path.open('x')
    io=None
    report=dict(mode='bno055_commissioning_no_can',action=args.action,bus=args.bus,
                address=args.address,mount_basis=args.mount_basis,errors=[],samples=[])
    try:
        io=I2CRegisters(args.bus,args.address)
        sensor=BNO055(io)
        report['configuration']=sensor.configure(args.confirm) if args.action=='configure' else sensor.inspect()
        if args.action=='record':
            cfg=report['configuration']
            if not cfg['matches'] or cfg['system_error'] or cfg['system_status'] != 5:
                raise RuntimeError('IMUPLUS/P0 configuration or fusion status not ready')
            start=time.monotonic_ns()
            deadline=start
            missed=0
            next_status=start
            while time.monotonic_ns()-start < args.duration*1e9:
                sample=sensor.read_sample()
                row=asdict(sample)
                row['body']=sample.body_state(args.mount_basis)
                report['samples'].append(row)
                if sample.read_end_ns>=next_status:
                    cfg=sensor.inspect()
                    if not cfg['matches'] or cfg['system_error'] or cfg['system_status']!=5:
                        raise RuntimeError(f'Sensor reset/configuration/status changed: {cfg}')
                    next_status=sample.read_end_ns+1_000_000_000
                deadline+=10_000_000
                now=time.monotonic_ns()
                if deadline<now:
                    skipped=(now-deadline)//10_000_000+1
                    missed+=skipped
                    deadline+=skipped*10_000_000
                time.sleep(max(0,(deadline-time.monotonic_ns())/1e9))
            report.update(elapsed_s=(time.monotonic_ns()-start)/1e9,missed_poll_slots=int(missed))
            report['timing_note']='100Hz host polling is not proof of new100Hz sensor samples; no capture timestamp/sequence available'
    except Exception as exc:
        report['errors'].append(f'{type(exc).__name__}: {exc}')
    finally:
        if io is not None:
            io.close()
        if output is not None:
            json.dump(report,output,indent=2)
            output.close()
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
    print('SAMPLES',len(report['samples']))
    raise SystemExit(1 if report['errors'] else 0)


if __name__=='__main__':
    main()
