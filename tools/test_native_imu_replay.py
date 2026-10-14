"""Validate native decode on captured bytes, without opening any device."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
from open_sprite_runtime.bno055 import decode_sample
from open_sprite_runtime.native_imu_packet import decode_packet


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',required=True)
    parser.add_argument('--capture',required=True)
    args=parser.parse_args()
    samples=json.loads(Path(args.capture).read_text())['samples']
    with tempfile.TemporaryDirectory() as tmp:
        raw=Path(tmp)/'raw';out=Path(tmp)/'packets'
        raw.write_bytes(b''.join(bytes.fromhex(s['raw_hex']) for s in samples))
        subprocess.run([args.binary,'--replay',str(raw),str(out)],check=True)
        data=out.read_bytes()
        if len(data)!=192*len(samples):raise AssertionError('record count mismatch')
        for i,s in enumerate(samples):
            packet=decode_packet(data[i*192:(i+1)*192],(i+1)*10_000_000+2000,i)
            reference=decode_sample(bytes.fromhex(s['raw_hex']),1,2)
            for key in ('acceleration_p0_m_s2','gyro_p0_rad_s','quaternion_p0_to_world_wxyz','gravity_p0_m_s2'):
                if getattr(packet.sample,key)!=getattr(reference,key):raise AssertionError(key)
        raw.write_bytes(b'\0'*46)
        result=subprocess.run([args.binary,'--replay',str(raw),str(out)],capture_output=True)
        if result.returncode==0:raise AssertionError('invalid quaternion accepted')
        raw.write_bytes(b'\0'*45)
        result=subprocess.run([args.binary,'--replay',str(raw),str(out)],capture_output=True)
        if result.returncode==0:raise AssertionError('truncated frame accepted')
    print(json.dumps(dict(passed=True,samples=len(samples),decode_equivalence='exact',can_tx_count=0)))


if __name__=='__main__':main()
