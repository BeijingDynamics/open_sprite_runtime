"""Offline-only candidate preparation. Never opens CAN or launches a controller.

Use a completed baseline's effective joint table, not just its contract: the
baseline applies additional head/wrist/waist multipliers at runtime.
"""
import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(contract_path, baseline, output):
    contract = json.loads(contract_path.read_text())
    def rows(name):
        with (baseline / name).open() as stream:
            return list(csv.DictReader(stream, delimiter='\t'))
    joints, motors = rows('joint_safety.tsv'), rows('motors.tsv')
    if [r['joint_name'] for r in joints] != contract['joint_names']:
        raise ValueError('baseline/contract joint order mismatch')
    if len(joints) != 31 or len(motors) != 31:
        raise ValueError('expected 31 joints and motors')
    original_joints, original_motors = copy.deepcopy(joints), copy.deepcopy(motors)
    gains = {'waist_yaw_joint': (130., 5.)}
    caps = {'waist_yaw_motor': 28.}
    for side in ('left', 'right'):
        for axis in ('pitch', 'roll'):
            gains[f'{side}_shoulder_{axis}_joint'] = (130., 5.)
            caps[f'{side}_shoulder_{axis}_motor'] = 28.
            gains[f'{side}_wrist_{axis}_joint'] = (7., .4)
        gains[f'{side}_wrist_yaw_joint'] = (7., 1.)
    if not set(gains) <= {r['joint_name'] for r in joints}:
        raise ValueError('missing selected joint')
    if not set(caps) <= {r['motor_name'] for r in motors}:
        raise ValueError('missing selected motor')
    if float(next(r for r in motors if r['motor_name']=='waist_roll_motor')['commissioning_torque_cap_nm']) != 30.:
        raise ValueError('expected baseline waist roll cap 30Nm')
    changes = []
    for row in joints:
        if row['joint_name'] in gains:
            kp, kd = gains[row['joint_name']]
            changes.append({'joint': row['joint_name'], 'old': [float(row['kp_max']), float(row['kd_max'])], 'new': [kp,kd]})
            row['kp_max'], row['kd_max'] = str(kp), str(kd)
    for row in motors:
        if row['motor_name'] in caps:
            cap = caps[row['motor_name']]
            bound = min(float(row['torque_max_nm']), -float(row['torque_min_nm']), float(row['mechanical_peak_torque_nm']), float(row['feedback_torque_cap_nm']))
            if cap > bound:
                raise ValueError('requested cap exceeds protocol/hardware/feedback bound')
            row['commissioning_torque_cap_nm'] = str(cap)
    # Assert exact change scope: feedback, temperatures, signs and limits stay put.
    for old, new in zip(original_joints,joints):
        allowed = {'kp_max','kd_max'} if old['joint_name'] in gains else set()
        assert all(old[k]==new[k] for k in old if k not in allowed)
    for old, new in zip(original_motors,motors):
        allowed = {'commissioning_torque_cap_nm'} if old['motor_name'] in caps else set()
        assert all(old[k]==new[k] for k in old if k not in allowed)
    output.mkdir(parents=True, exist_ok=False)
    for name, table in [('joint_safety.tsv',joints),('motors.tsv',motors)]:
        with (output/name).open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(table[0]),delimiter='\t',lineterminator='\n')
            writer.writeheader();writer.writerows(table)
    report={'mode':'offline_upper_body_pd_candidate_no_hardware',
            'powered_ready':False,
            'reason':'Requires native input-fixture regeneration and parity validation; do not substitute TSVs in the old frozen actor bundle.',
            'joint_changes':changes,'motor_cap_overrides_nm':caps,
            'waist_roll_cap_nm':30.,'remaining_caps_unchanged':True,
            'effective_gain_tables':True,'additional_gain_multipliers_allowed':False,
            'source_sha256':{str(p):digest(p) for p in [contract_path,baseline/'joint_safety.tsv',baseline/'motors.tsv']},
            'output_sha256':{name:digest(output/name) for name in ['joint_safety.tsv','motors.tsv']}}
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--contract',type=Path,required=True)
    p.add_argument('--baseline-bundle',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();prepare(a.contract,a.baseline_bundle,a.output)
