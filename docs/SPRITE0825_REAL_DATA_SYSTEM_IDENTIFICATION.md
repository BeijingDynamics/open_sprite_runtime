# Sprite0825 real-data system identification

This workflow records evidence for narrowing the simulation-to-real gap. It does
not change the policy, URDF, gains, or motor limits automatically.

## Clock and rate contract

- Native CAN transport: 2 kHz scheduling loop.
- Ankle feedback/control: 500 Hz per motor.
- Other motor feedback/control: 50 Hz per motor.
- Policy: 50 Hz.
- IMU: 100 Hz nominal.
- Alignment clock: Jetson `CLOCK_MONOTONIC`, in nanoseconds.

The policy trace records the observation, action, projected joint target, final
startup-ramped target, and raw IMU samples. The independent CAN observer records
the exact frames visible on all four active SocketCAN interfaces. Its class has
no transmit method.

## Capture

Use two terminals. Start the recorder first:

```bash
cd /home/tony/open_sprite_runtime
./capture_sprite0825_system_id_can_on_253.sh 70 first_walk_001
```

Within a few seconds, run the already-qualified policy command in the other
terminal. The CAN capture may be longer than the policy run; overlap is checked
during the merge.

The recorder writes under `captures/system_id/<session>/`:

- `can_raw.npz`: raw CAN payloads and kernel/hardware/userspace timestamps.
- `can_capture_report.json`: frame counts and timestamp rejection counts.
- `SHA256SUMS`: immutable session hashes.

Live captures are intentionally ignored by Git. Copy the complete session to
the project backup before changing any model parameter.

## Merge

Pass explicit paths; never select the newest file implicitly:

```bash
./merge_sprite0825_system_id_session_on_253.sh \
  captures/system_id/first_walk_001/can_raw.npz \
  reports/native_protected_policy_trace_<timestamp>.npz \
  captures/system_id/first_walk_001/aligned
```

The merged NPZ contains, at every policy tick and for every physical motor:

- measured motor position and velocity;
- estimated output torque and both temperatures;
- most recently observed exact MIT position, velocity, Kp, Kd, and feedforward
  torque command;
- feedback and command sample age.

The merge fails its quality gate when any motor has less than 98% feedback
coverage, p99 feedback age exceeds 60 ms, or observed command coverage is below
90%. Command coverage also verifies that SocketCAN loopback is available on the
installed KH driver.

## Identification order

1. Suspended single-joint low-amplitude tests: delay, Kp, Kd, friction, deadband.
2. Supported standing perturbations: pelvis/leg inertia and damping residuals.
3. Slow weight transfer and stepping: contact timing and ankle coupling.
4. Walking: held-out validation first, parameter fitting second.

Keep CAD mass and inertia as priors. Do not freely fit every link tensor from a
single walk. Fit actuator/contact effects separately, validate on an unused run,
then update simulation parameters and domain-randomization ranges.

Horizontal base velocity may be estimated offline from a fixed camera for
identification, but it must not be added to the deployed policy observation.

## Measurements outside CAN feedback

Damiao MIT feedback does not contain battery-bus voltage or pack current. Log
those channels independently from the BMS or an isolated power monitor when
available. Without them, a battery voltage sag or current limit can be mistaken
for actuator delay, weak gains, or an incorrect inertia model.

The first battery-powered records should keep the same policy and gains and
progress from quiet standing, to small manual perturbations, to slow weight
transfer, and only then to short straight walking. Change one physical or model
parameter at a time and retain a held-out session for validation.
