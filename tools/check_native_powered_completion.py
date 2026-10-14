"""Fail-closed report gate. Reading reports never issues hardware commands."""
import argparse
import json
from pathlib import Path


def validate_completion(report, expected_motors, actor_exit, controller_exit):
    errors = []
    if actor_exit != 0 or controller_exit != 0:
        errors.append('actor/controller exit failed')
    if report.get('mode') != 'native_suspended_protected_policy_actuation':
        errors.append('wrong controller report mode')
    if report.get('passed') is not True or report.get('errors') != []:
        errors.append('controller did not pass')
    if report.get('duration_s') != 40 or report.get('deadline_misses') != 0:
        errors.append('duration/deadline mismatch')
    if len(expected_motors) != 31 or len(set(expected_motors)) != 31:
        errors.append('invalid expected motor set')
    motors = report.get('motors', {})
    if not isinstance(motors, dict) or set(motors) != set(expected_motors):
        errors.append('motor set mismatch')
    elif any(not isinstance(motors[n], dict) or motors[n].get('final_disabled') is not True
             for n in expected_motors):
        errors.append('not all 31 motors verified disabled')
    for key in ('automatic_mode_switch_attempts', 'automatic_zero_reset_attempts'):
        if report.get(key) != 0:
            errors.append(key+' missing or nonzero')
    if errors:
        raise ValueError('; '.join(errors))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--report', required=True)
    parser.add_argument('--names', required=True)
    parser.add_argument('--actor-exit', type=int, required=True)
    parser.add_argument('--controller-exit', type=int, required=True)
    args = parser.parse_args()
    names = Path(args.names).read_text().split()
    if len(names) != 63 or names[0] != 'SPRITE_POWERED_WALK40_V1':
        raise ValueError('invalid powered names manifest')
    validate_completion(json.loads(Path(args.report).read_text()), names[1:32],
                        args.actor_exit, args.controller_exit)
    print('POWERED_COMPLETION_VERIFIED all_31_disabled=true')


if __name__ == '__main__':
    main()
