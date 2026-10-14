"""Compare native motor/IMU conversion with Python and original live traces."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import numpy as np
from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.motor_mapping import motor_map_from_hardware_config
from open_sprite_runtime.imu import ImuMount, RawImuSample, transform_pelvis_sample


def main():
    p=argparse.ArgumentParser()
    for name in ('hardware','contract','trace','binary','output-dir'):
        p.add_argument('--'+name,required=True)
    args=p.parse_args();out=Path(args.output_dir);out.mkdir(parents=True,exist_ok=False)
    contract=PolicyContract.load(args.contract)
    hardware=json.loads(Path(args.hardware).read_text())
    mapping=motor_map_from_hardware_config(hardware,tuple(contract.data['joint_names']),require_armable=False)
    trace=np.load(args.trace,allow_pickle=False)
    names=list(trace['motor_names']);joints=list(trace['joint_names'])
    if len(names)!=31 or len(set(names))!=31 or set(names)!=set(mapping.physical_motor_names):
        raise ValueError('motor order/set mismatch')
    if joints!=contract.data['joint_names']:raise ValueError('joint order mismatch')
    zero=np.zeros(31);offset_map=mapping.joint_to_motor_positions(zero)
    offset=np.array([offset_map[n] for n in names]);matrix=np.zeros((31,31))
    for j in range(31):
        basis=zero.copy();basis[j]=1
        values=mapping.joint_to_motor_positions(basis)
        matrix[:,j]=np.array([values[n] for n in names])-offset
    inverse=np.linalg.inv(matrix)
    actual=np.concatenate([trace[k] for k in ('motor_position_rad','motor_velocity_rad_s','imu_angular_velocity_sensor_rad_s','imu_quaternion_wxyz')],axis=1)
    if not np.isfinite(actual).all():raise ValueError('nonfinite trace')
    rng=np.random.default_rng(20261013)
    synthetic=rng.normal(size=(256,69));synthetic[:,65:]/=np.linalg.norm(synthetic[:,65:],axis=1)[:,None]
    frames=np.concatenate((actual,synthetic))
    data=b'SPFDB001'+struct.pack('<I',len(frames))+offset.astype('<f8').tobytes()+inverse.astype('<f8').tobytes()+frames.astype('<f8').tobytes()
    (out/'input.bin').write_bytes(data)
    process=subprocess.run([args.binary,str(out/'input.bin'),str(out/'output.bin')],capture_output=True,text=True)
    (out/'native.log').write_text(process.stdout+process.stderr)
    if process.returncode:raise RuntimeError(process.stderr)
    native=np.fromfile(out/'output.bin',dtype='<f8').reshape((-1,68))
    if native.shape!=(len(frames),68):raise ValueError('output size mismatch')
    expected=[];mount=ImuMount.sprite0825_rear_pelvis()
    for row in frames:
        pos=mapping.motor_to_joint_positions(dict(zip(names,row[:31])))
        vel=mapping.motor_to_joint_velocities(dict(zip(names,row[31:62])))
        imu=transform_pelvis_sample(RawImuSample(1,tuple(row[62:65]),(0,0,0),tuple(row[65:69])),mount)
        expected.append(np.concatenate((pos,vel,imu.angular_velocity_body_rad_s,imu.projected_gravity_body)))
    expected=np.array(expected)
    recorded=np.concatenate([trace[k] for k in ('joint_position_rad','joint_velocity_rad_s','base_angular_velocity_rad_s','projected_gravity')],axis=1)
    errors={}
    for name,sl in [('joint_pos',slice(0,31)),('joint_vel',slice(31,62)),('angular_velocity',slice(62,65)),('gravity',slice(65,68))]:
        errors[name]=float(np.max(np.abs(native[:,sl]-expected[:,sl])))
    recorded_error=float(np.max(np.abs(native[:len(actual)]-recorded)))
    # Reject malformed inputs without hardware or permissive fallback.
    bad_cases={'truncated':data[:-1],'trailing':data+b'X','bad_magic':b'BADMAGIC'+data[8:]}
    nan=bytearray(data);struct.pack_into('<d',nan,12+8*(31+961),float('nan'));bad_cases['nan']=nan
    quat=bytearray(data);struct.pack_into('<4d',quat,12+8*(31+961+65),0,0,0,0);bad_cases['zero_quaternion']=quat
    rejected=[]
    for name,blob in bad_cases.items():
        path=out/(name+'.bin');path.write_bytes(blob)
        result=subprocess.run([args.binary,str(path),str(out/(name+'.out'))],capture_output=True)
        if result.returncode==0:raise RuntimeError('invalid input accepted: '+name)
        rejected.append(name)
    report=dict(mode='offline_feedback_equivalence_no_hardware',recorded_frames=len(actual),synthetic_frames=len(synthetic),
        maximum_abs_errors=errors,maximum_vs_recorded=recorded_error,rejected_cases=rejected,
        passed=all(np.isfinite(v) and v<1e-10 for v in [*errors.values(),recorded_error]),hardware_tx=0,
        hardware_sha256=hashlib.sha256(Path(args.hardware).read_bytes()).hexdigest(),
        trace_sha256=hashlib.sha256(Path(args.trace).read_bytes()).hexdigest())
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if not report['passed']:raise SystemExit('feedback equivalence failed')


if __name__=='__main__':main()
