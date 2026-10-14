"""Offline parity fixtures for target projection and physical startup ramp."""
import argparse
import json
from pathlib import Path
import subprocess
import numpy as np
from open_sprite_runtime.target_projection import ProtectedTargetProjector
from open_sprite_runtime.physical_startup import PhysicalStartupRamp
from open_sprite_runtime.multirate_control import JointImpedanceTarget


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--binary', required=True)
    p.add_argument('--output-dir', required=True)
    args = p.parse_args()
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=False)
    rng = np.random.default_rng(20261013)
    names = tuple(f'joint_{i}' for i in range(31))
    results = []
    rejected = []

    def run(name, config, frames, reject=False, suffix=''):
        path = out / (name + '.txt')
        tokens = ['SPPROT01', str(len(frames))]
        tokens += [format(float(x), '.17g') for x in np.concatenate(config)]
        tokens += [format(float(x), '.17g') for x in frames.ravel()]
        path.write_text(' '.join(tokens) + suffix)
        output = out / (name + '.out')
        proc = subprocess.run([args.binary, str(path), str(output)], capture_output=True, text=True)
        (out / (name + '.log')).write_text(proc.stdout + proc.stderr)
        if reject:
            if proc.returncode == 0:
                raise AssertionError('invalid input accepted: ' + name)
            rejected.append(name)
            return
        if proc.returncode:
            raise RuntimeError(proc.stderr)
        return np.loadtxt(output, ndmin=2)

    for case, (hold, ramp_seconds) in enumerate([(0, .001), (.04, .06), (2, 6), (.031, .071)]):
        lower = np.full(31, -.5)
        upper = np.full(31, .6)
        gains = rng.uniform(.05, 1, 31)
        damping = rng.uniform(.1, 4, 31)
        config = [lower, upper, gains, damping, np.array([.8, 5, 50, hold, ramp_seconds])]
        frames = rng.normal(size=(501, 6, 31))
        frames[:, 2] = np.abs(frames[:, 2]) * 130
        frames[:, 3] = rng.uniform(0, 1, (501, 31))
        frames[0, 5] = rng.uniform(-.5, .6, 31)
        projector = ProtectedTargetProjector(names, lower, upper, .8, 5, gains, damping)
        ramp = PhysicalStartupRamp(50, hold, ramp_seconds)
        expected = []
        for row in frames:
            t = projector.project(JointImpedanceTarget(*row[:5]))
            measured = row[5]
            if ramp.initial_position_rad is None:
                measured = projector.require_position_within_limits(measured)
            t = ramp.apply(t, measured)
            expected.append(np.concatenate((t.position_rad, t.velocity_rad_s, t.kp,
                                           t.kd, t.feedforward_torque_nm, [ramp.last_alpha])))
        native = run(f'case_{case}', config, frames)
        expected = np.asarray(expected)
        if native.shape != expected.shape or not np.isfinite(native).all():
            raise AssertionError('invalid native output')
        error = float(np.max(np.abs(native - expected)))
        if error > 1e-12:
            raise AssertionError(f'parity error {error}')
        results.append(dict(hold_seconds=hold, ramp_seconds=ramp_seconds,
                            frames=len(frames), maximum_abs_error=error))

    for name, index, value in [('negative_kp', (0,2,0), -1),
                               ('negative_kd', (0,3,0), -1),
                               ('excess_kd', (0,3,0), 10000),
                               ('bad_start_pose', (0,5,0), 2),
                               ('nan_target', (0,0,0), float('nan'))]:
        invalid = frames.copy()
        invalid[index] = value
        run(name, config, invalid, reject=True)
    run('trailing', config, frames, reject=True, suffix=' EXTRA')
    for name, idx, value in [('zero_hz', 2, 0), ('negative_hold', 3, -1),
                              ('zero_ramp', 4, 0), ('bad_scale', 0, 2), ('bad_kd_max', 1, 0)]:
        invalid = [v.copy() for v in config]
        invalid[-1][idx] = value
        run(name, invalid, frames, reject=True)
    invalid = [v.copy() for v in config]
    invalid[0][0] = invalid[1][0]
    run('unordered_limits', invalid, frames, reject=True)
    report = dict(mode='offline_protection_equivalence_no_hardware', hardware_tx=0,
                  cases=results, rejected_cases=rejected, passed=True)
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
