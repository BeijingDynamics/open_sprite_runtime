"""Offline recurrence equivalence against Python; never opens hardware devices."""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import hashlib
import numpy as np
from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.policy_shadow import PolicyObservationHistory
from open_sprite_runtime.handoff import MeasuredPoseActionHandoff
from open_sprite_runtime.shadow import load_actor


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--contract',required=True)
    p.add_argument('--trace',required=True)
    p.add_argument('--binary',required=True)
    p.add_argument('--output-dir',required=True)
    args=p.parse_args()
    contract=PolicyContract.load(args.contract)
    terms=contract.data['observation_terms']
    expected=[('joint_pos',8),('joint_vel',8),('actions',8),('base_ang_vel',8),('projected_gravity',8),('velocity_commands',0)]
    assert [(t['name'],t['history_length']) for t in terms]==expected
    assert contract.data['actor_observation_dim']==795 and contract.policy_hz==50
    trace=np.load(args.trace,allow_pickle=False)
    assert list(trace['joint_names'])==contract.data['joint_names']
    inputs=np.concatenate([trace[k] for k in ('joint_position_rad','joint_velocity_rad_s','base_angular_velocity_rad_s','projected_gravity','command_vx_vy_yaw_rate')],axis=1).astype('<f8')
    assert inputs.shape[1]==71 and np.isfinite(inputs).all()
    output=Path(args.output_dir);output.mkdir(parents=True,exist_ok=False)
    handoff=MeasuredPoseActionHandoff(contract)
    with (output/'input.bin').open('wb') as f:
        f.write(b'SPPOL001'+struct.pack('<II',len(inputs),handoff.total_steps))
        for field in ('default_joint_pos','action_offset','action_scale'):
            f.write(np.asarray(contract.data[field],dtype='<f8').tobytes())
        f.write(inputs.tobytes())
    model=contract.path.parent/contract.data['policy_onnx']
    result=subprocess.run([args.binary,str(model),str(output/'input.bin'),str(output/'output.bin')],capture_output=True,text=True)
    (output/'native.log').write_text(result.stdout+result.stderr)
    if result.returncode:raise RuntimeError(result.stderr)
    print(result.stdout,end='')
    (output/'native.log').write_text(result.stdout+result.stderr)
    dtype=np.dtype([('obs','<f4',(795,)),('raw','<f4',(31,)),('handoff','<f8',(31,)),('target','<f8',(31,))])
    native=np.fromfile(output/'output.bin',dtype=dtype)
    assert len(native)==len(inputs)
    history=PolicyObservationHistory(contract);actor=load_actor(contract)
    previous=np.zeros(31,dtype=np.float32)
    offset=np.asarray(contract.data['action_offset']);scale=np.asarray(contract.data['action_scale'])
    maximum={k:0.0 for k in ('obs','raw','handoff','target')}
    recorded_obs_max=0.0
    for i,row in enumerate(inputs):
        history.append(row[:31],row[31:62],previous,row[62:65],row[65:68])
        obs=history.observation(row[68:71]);raw=actor.run(obs)
        previous=handoff.blend(raw,row[:31]);target=offset+scale*previous
        for name,value in zip(maximum,(obs,raw,previous,target)):
            delta=float(np.max(np.abs(native[name][i]-value)))
            if not np.isfinite(delta):raise RuntimeError('nonfinite comparison')
            maximum[name]=max(maximum[name],delta)
        recorded_obs_max=max(recorded_obs_max,float(np.max(np.abs(obs-trace['observation'][i]))))
    # Cold-start replay is compared separately to original live observations.
    passed=all(v<=1e-4 for v in maximum.values())
    report=dict(mode='offline_native_policy_recurrence_no_hardware',frames=len(inputs),
        max_abs_error=maximum,python_vs_recorded_observation_max=recorded_obs_max,
        tolerance=1e-4,passed=passed,contract_sha256=hashlib.sha256(contract.path.read_bytes()).hexdigest(),
        model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        trace_sha256=hashlib.sha256(Path(args.trace).read_bytes()).hexdigest(),hardware_tx=0)
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not passed:raise SystemExit('native equivalence failed')


if __name__=='__main__':main()
