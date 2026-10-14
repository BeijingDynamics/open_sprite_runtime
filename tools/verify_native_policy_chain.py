"""Recorded sensor-to-protected-target replay; no live transport or hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import numpy as np
from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.policy_shadow import PolicyObservationHistory
from open_sprite_runtime.handoff import MeasuredPoseActionHandoff
from open_sprite_runtime.shadow import load_actor
from open_sprite_runtime.motor_mapping import motor_map_from_hardware_config
from open_sprite_runtime.imu import ImuMount, RawImuSample, transform_pelvis_sample
from open_sprite_runtime.target_projection import ProtectedTargetProjector
from open_sprite_runtime.physical_startup import PhysicalStartupRamp
from open_sprite_runtime.multirate_control import JointImpedanceTarget


def main():
    p=argparse.ArgumentParser()
    for name in ('contract','hardware','limits','actor-report','trace','binary','output-dir'):
        p.add_argument('--'+name,required=True)
    args=p.parse_args();out=Path(args.output_dir);out.mkdir(parents=True,exist_ok=False)
    contract=PolicyContract.load(args.contract);c=contract.data
    if c['actor_observation_dim']!=795 or contract.policy_hz!=50:raise ValueError('contract shape/rate')
    expected=[('joint_pos',8),('joint_vel',8),('actions',8),('base_ang_vel',8),('projected_gravity',8),('velocity_commands',0)]
    if [(t['name'],t['history_length']) for t in c['observation_terms']]!=expected:raise ValueError('observation contract')
    hardware=json.loads(Path(args.hardware).read_text())
    report=json.loads(Path(args.actor_report).read_text())
    if report['consecutive_clamp_watchdog']['enabled']:raise ValueError('watchdog not supported in this replay')
    mapping=motor_map_from_hardware_config(hardware,tuple(c['joint_names']),require_armable=False)
    trace=np.load(args.trace,allow_pickle=False);names=list(trace['motor_names'])
    if list(trace['joint_names'])!=c['joint_names'] or len(names)!=31 or set(names)!=set(mapping.physical_motor_names):
        raise ValueError('recorded joint/motor order mismatch')
    cfg=report['protected_target_projection'];ramp_cfg=report['physical_startup_ramp']
    if not cfg['enabled'] or not ramp_cfg['enabled']:raise ValueError('projection/ramp required')
    gain=np.array([cfg['joint_gain_multiplier_overrides'].get(n,1.) for n in c['joint_names']])
    damping=np.array([cfg['joint_damping_multiplier_overrides'].get(n,1.) for n in c['joint_names']])
    projector=ProtectedTargetProjector.from_limit_report(tuple(c['joint_names']),args.limits,
        gain_scale=cfg['gain_scale'],maximum_embedded_kd=cfg['maximum_embedded_kd'],
        joint_gain_multipliers=gain,joint_damping_multipliers=damping)
    startup=PhysicalStartupRamp(ramp_cfg['policy_hz'],ramp_cfg['hold_seconds'],ramp_cfg['ramp_seconds'])
    offset_map=mapping.joint_to_motor_positions(np.zeros(31));offset=np.array([offset_map[n] for n in names])
    matrix=np.zeros((31,31))
    for j in range(31):
        v=np.zeros(31);v[j]=1;values=mapping.joint_to_motor_positions(v)
        matrix[:,j]=np.array([values[n] for n in names])-offset
    inverse=np.linalg.inv(matrix)
    inputs=np.concatenate([trace[k] for k in ('motor_position_rad','motor_velocity_rad_s',
        'imu_angular_velocity_sensor_rad_s','imu_quaternion_wxyz','command_vx_vy_yaw_rate')],axis=1)
    if not np.isfinite(inputs).all():raise ValueError('nonfinite input')
    kp=np.asarray(c['stiffness']);kd=np.minimum(c['damping'],hardware['controller']['damiao_embedded_kd_max'])
    handoff=MeasuredPoseActionHandoff(contract)
    with (out/'input.bin').open('wb') as f:
        f.write(b'SPCHN001'+struct.pack('<II',len(inputs),handoff.total_steps))
        arrays=[c['default_joint_pos'],c['action_offset'],c['action_scale'],offset,inverse,
                projector.lower_rad,projector.upper_rad,gain,damping,
                [cfg['gain_scale'],cfg['maximum_embedded_kd'],ramp_cfg['policy_hz'],ramp_cfg['hold_seconds'],ramp_cfg['ramp_seconds']],
                kp,kd,inputs]
        for a in arrays:f.write(np.asarray(a,dtype='<f8').tobytes())
    model=contract.path.parent/c['policy_onnx']
    proc=subprocess.run([args.binary,str(model),str(out/'input.bin'),str(out/'output.bin')],capture_output=True,text=True)
    (out/'native.log').write_text(proc.stdout+proc.stderr)
    if proc.returncode:raise RuntimeError(proc.stderr)
    dtype=np.dtype([('obs','<f4',(795,)),('raw','<f4',(31,)),('handoff','<f8',(31,)),('target','<f8',(31,)),('protected','<f8',(5,31))])
    native=np.fromfile(out/'output.bin',dtype=dtype)
    if len(native)!=len(inputs):raise ValueError('frame count mismatch')
    history=PolicyObservationHistory(contract);actor=load_actor(contract);mount=ImuMount.sprite0825_rear_pelvis()
    previous=np.zeros(31);maximum={n:0. for n in dtype.names};recorded_obs_error=0.
    for i,row in enumerate(inputs):
        q=mapping.motor_to_joint_positions(dict(zip(names,row[:31])))
        v=mapping.motor_to_joint_velocities(dict(zip(names,row[31:62])))
        imu=transform_pelvis_sample(RawImuSample(1,tuple(row[62:65]),(0,0,0),tuple(row[65:69])),mount)
        history.append(q,v,previous,imu.angular_velocity_body_rad_s,imu.projected_gravity_body)
        obs=history.observation(row[69:]);raw=actor.run(obs);previous=handoff.blend(raw,q)
        target=np.asarray(c['action_offset'])+np.asarray(c['action_scale'])*previous
        protected=projector.project(JointImpedanceTarget(target,np.zeros(31),kp,kd,np.zeros(31)))
        measured=projector.require_position_within_limits(q) if startup.initial_position_rad is None else q
        protected=startup.apply(protected,measured)
        values=[obs,raw,previous,target,np.array([protected.position_rad,protected.velocity_rad_s,
                                                 protected.kp,protected.kd,protected.feedforward_torque_nm])]
        for name,value in zip(dtype.names,values):
            delta=float(np.max(np.abs(native[name][i]-value)))
            if not np.isfinite(delta):raise ValueError('nonfinite comparison')
            maximum[name]=max(maximum[name],delta)
        recorded_obs_error=max(recorded_obs_error,float(np.max(np.abs(obs-trace['observation'][i]))))
    recorded_target=np.stack([trace[k] for k in ('startup_target_position_rad','startup_target_velocity_rad_s',
                            'startup_kp','startup_kd','startup_feedforward_torque_nm')],axis=1)
    recorded_target_error=float(np.max(np.abs(native['protected']-recorded_target)))
    hashes={n:hashlib.sha256(Path(getattr(args,n.replace('-','_'))).read_bytes()).hexdigest()
            for n in ('contract','hardware','limits','actor-report','trace')}
    result=dict(mode='offline_sensor_to_protected_target_chain',hardware_tx=0,frames=len(inputs),
        maximum_abs_errors=maximum,python_vs_recorded_observation_max=recorded_obs_error,
        native_vs_recorded_final_target_max=recorded_target_error,
        passed=all(x<=1e-4 for x in maximum.values()) and recorded_obs_error<=1e-4 and recorded_target_error<=1e-4,
        tolerance=1e-4,source_sha256=hashes,model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        command_source='recorded commands; no live command profile or IPC exercised')
    (out/'report.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit('chain equivalence failed')


if __name__=='__main__':main()
