"""Version1 native IMU packets; host read timestamps, never arrival timestamps."""
from dataclasses import dataclass
import math
import struct
from .bno055 import decode_sample, Sample


def decode_software_packet(payload, now_ns, previous_sequence=0):
    if len(payload)!=192 or payload[:8]!=b'SPIMU002':
        raise ValueError('software IMU packet size/version mismatch')
    if payload[50:56]!=bytes(6) or payload[184:]!=bytes(8):
        raise ValueError('software IMU reserved bytes nonzero')
    sequence,start,end=struct.unpack_from('<QQQ',payload,8)
    if not sequence or sequence<previous_sequence:
        raise ValueError('software IMU sequence regression')
    if not 0<start<=end<=now_ns or now_ns-start>30_000_000:
        raise RuntimeError('software IMU stale/future timestamp')
    bias=struct.unpack_from('<3d',payload,56)
    v=struct.unpack_from('<13d',payload,80)
    if not all(math.isfinite(x) for x in bias+v) or sum(x*x for x in bias)>.01:
        raise ValueError('invalid software IMU values/bias')
    accel=tuple(x/100 for x in struct.unpack_from('<3h',payload,32))
    gyro=tuple(x/900-b for x,b in zip(struct.unpack_from('<3h',payload,44),bias))
    if any(abs(a-b)>1e-12 for a,b in zip(v[:6],accel+gyro)):
        raise ValueError('software IMU raw/bias mismatch')
    w,x,y,z=v[6:10]
    if abs(w*w+x*x+y*y+z*z-1)>1e-6:
        raise ValueError('software IMU quaternion norm')
    up=(2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y))
    if any(abs(a-9.80665*b)>1e-9 for a,b in zip(v[10:],up)):
        raise ValueError('software IMU gravity mismatch')
    sample=Sample(start,end,accel,gyro,tuple(v[6:10]),tuple(v[10:]),None,{},payload[32:50].hex())
    return NativeImuPacket(sequence,sample)


@dataclass(frozen=True)
class NativeImuPacket:
    sequence: int
    sample: object


def decode_packet(payload, now_ns, previous_sequence=0):
    if len(payload)!=192 or payload[:8]!=b'SPIMU001':
        raise ValueError('native IMU packet size/version mismatch')
    if payload[78:80]!=b'\0\0' or payload[184:]!=b'\0'*8:
        raise ValueError('native IMU reserved bytes nonzero')
    sequence,start,end=struct.unpack_from('<QQQ',payload,8)
    if not sequence or sequence<previous_sequence:
        raise ValueError('native IMU sequence regression')
    if start<=0 or end>now_ns or now_ns-start>30_000_000:
        raise RuntimeError('native IMU stale/future timestamp')
    sample=decode_sample(payload[32:78],start,end)
    values=struct.unpack_from('<13d',payload,80)
    expected=(*sample.acceleration_p0_m_s2,*sample.gyro_p0_rad_s,
              *sample.quaternion_p0_to_world_wxyz,*sample.gravity_p0_m_s2)
    if any(not math.isfinite(a) or abs(a-b)>1e-12 for a,b in zip(values,expected)):
        raise ValueError('native IMU decoded values disagree with raw registers')
    return NativeImuPacket(sequence,sample)
