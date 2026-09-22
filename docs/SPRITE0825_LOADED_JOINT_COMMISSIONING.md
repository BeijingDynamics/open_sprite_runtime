# Sprite0825 loaded-joint commissioning

The first mechanism-bearing powered test uses only
`right_wrist_roll_motor` (`kcan4`, command ID `0x07`, feedback ID `0x17`). It
is a DM-J3507 with a 0.8 Nm rated torque and a frozen
`policy_to_motor_sign=-1`. No other motor is enabled by either test.

## Current-position hold

The first gate latches the disabled feedback position and holds that same motor
position for two seconds at 50 Hz with `Kp=0.2`, `Kd=0.05`, and zero
feedforward. Its fixed guards are 0.05 rad position error, 0.2 rad/s measured
speed, and 0.1 Nm estimated torque.

```bash
cd /home/tony/open_sprite_runtime
./hold_sprite0825_right_wrist_roll_low_gain_on_253.sh \
  ENABLE_RIGHT_WRIST_ROLL_LOW_GAIN_HOLD
```

The 2026-09-22 run passed 100/100 command/feedback cycles, observed at most
0.000384 rad error and 0.003664 Nm, and verified disabled feedback after three
disable frames. The Jetson report is
`reports/right_wrist_roll_low_gain_hold_20260922_103133.json`.

## Joint-space motion

The second gate commands relative joint-space `+5 degrees`, then `-5 degrees`,
then the measured start. The motor command applies the frozen negative mapping,
so its first motor-space excursion is negative. The final profile uses
four-second quintic transitions, one-second dwells, `Kp=2.0`, `Kd=0.2`, zero
feedforward, and 0.08 rad/0.6 rad/s/0.2 Nm guards.

```bash
cd /home/tony/open_sprite_runtime
./move_sprite0825_right_wrist_roll_5deg_on_253.sh \
  ENABLE_RIGHT_WRIST_ROLL_5DEG_MOTION
```

The first conservative `Kp=1.0` attempt failed closed at its position-error
guard after moving in the correct negative motor direction. The final
2026-09-22 run completed 750/750 cycles, measured approximately `-2.50/+2.79`
degrees at low gain, stayed below 0.526 rad/s and 0.092 Nm, and verified
disabled feedback. Its Jetson report is
`reports/right_wrist_roll_5deg_20260922_103819.json`.

These tests qualify the direct sign mapping and guarded single-motor transport.
They do not authorize a whole-arm or whole-body command. The next gate is a
separately reviewed fixed right-arm current-position hold.

## Fixed right-wrist group gate

The first fixed group contains only the right wrist yaw, pitch, and roll motors
on `kcan4` (command IDs `0x05/0x06/0x07`). The group writer requires all three
initial positions to retain 0.05 rad of soft-limit margin before it sends any
enable frame. Any runtime fault disables all three motors, then polls each motor
individually until disabled feedback is observed.

```bash
cd /home/tony/open_sprite_runtime
./hold_sprite0825_right_wrist_group_low_gain_on_253.sh \
  ENABLE_RIGHT_WRIST_GROUP_LOW_GAIN_HOLD
```

The first 2026-09-22 gate attempt was safely rejected before enable. A follow-up
zero-gain read measured right wrist yaw at `-1.736667 rad`, while its configured
soft and hard lower limits are `-1.695 rad` and `-1.745 rad`. It was therefore
only `0.00833 rad` (about 0.48 degrees) from the negative hard boundary. Wrist
pitch and roll were `+0.139782 rad` and `+0.081492 rad`. Do not widen the limit:
with all motors disabled, return wrist yaw toward its neutral pose and repeat
the zero-gain read before rerunning this gate.

After the wrist yaw was manually returned to `+0.01545 rad`, the repeated
three-motor gate passed 100/100 cycles per motor. Peak error was 0.000384 rad,
peak speed was 0.0611 rad/s, peak estimated torque was 0.0171 Nm, and all three
motors were verified disabled at exit. The Jetson report is
`reports/right_wrist_group_low_gain_hold_20260922_110740.json`.

## Fixed arm groups

The fixed right-arm group contains only `kcan4` IDs `0x01..0x07`. The fixed
left-arm group contains only `kcan3` IDs `0x01..0x07`; `head_yaw_motor` at ID
`0x08` is deliberately outside its writer allowlist. Both groups hold the
measured pose for two seconds at 50 Hz with `Kp=0.2`, `Kd=0.05`, and zero
feedforward. Position and speed guards remain 0.05 rad and 0.2 rad/s. The two
DM-J4340 shoulder motors use a 0.5 Nm estimated-torque guard; all DM-J4310 and
DM-J3507 arm motors retain the 0.1 Nm guard.

```bash
cd /home/tony/open_sprite_runtime
./hold_sprite0825_right_arm_group_low_gain_on_253.sh \
  ENABLE_RIGHT_ARM_GROUP_LOW_GAIN_HOLD
./hold_sprite0825_left_arm_group_low_gain_on_253.sh \
  ENABLE_LEFT_ARM_GROUP_LOW_GAIN_HOLD
```

The right-arm run passed 100/100 cycles on all seven motors with peak estimated
torque 0.1026 Nm and verified all motors disabled. The left-arm run passed
100/100 cycles on all seven motors with peak estimated torque 0.3077 Nm on the
loaded left shoulder pitch and also verified all motors disabled. Reports:

- `reports/right_arm_group_low_gain_hold_20260922_111440.json`
- `reports/left_arm_group_low_gain_hold_20260922_112119.json`

These arm results do not authorize directly applying the same group hold to a
leg. Ankle commands require differential kinematics and the dedicated 500 Hz
ankle layer.

## Proximal-leg preparation

Two higher-risk fixed groups are prepared but not yet physically executed. Each
contains only one side's DM-J4340 hip pitch/roll/yaw and knee motors at command
IDs `0x01..0x04`. The left writer uses `kcan1`; the right writer uses `kcan2`.
Both exclude the ankle differential motors and all waist/head motors by exact
allowlist. Their measured-pose hold remains two seconds at 50 Hz with `Kp=0.2`,
`Kd=0.05`, zero feedforward, and a 0.5 Nm estimated-torque guard per motor.

Do not run either script until the lifting frame is reconfirmed, the selected
leg and cables have free motion, and the safety operator is at the independent
power cut-off:

```bash
./hold_sprite0825_left_proximal_leg_low_gain_on_253.sh \
  ENABLE_LEFT_PROXIMAL_LEG_LOW_GAIN_HOLD
./hold_sprite0825_right_proximal_leg_low_gain_on_253.sh \
  ENABLE_RIGHT_PROXIMAL_LEG_LOW_GAIN_HOLD
```

Both groups were then executed with renewed physical confirmation. The left
group completed 100/100 cycles per motor with peak estimated torque 0.2257 Nm;
the right group completed 100/100 cycles per motor with peak estimated torque
0.1847 Nm. Position error remained zero at feedback resolution, peak speed was
0.00489 rad/s, and all eight tested motors were verified disabled at exit.
Neither ankle pair nor any waist/head motor was enabled. Reports:

- `reports/left_proximal_leg_low_gain_hold_20260922_113414.json`
- `reports/right_proximal_leg_low_gain_hold_20260922_113548.json`

## Zero-position writes

Motor zero-position reset/write commands are forbidden unless the robot owner
gives explicit approval for that exact operation. Commissioning scripts in this
document neither reset zero positions nor switch motor modes.

## Differential-ankle transport gate

The first powered ankle gate is prepared and offline-tested but not yet
physically executed. It is intentionally not a position hold: one calibrated
pair is enabled for two seconds and receives 500 Hz MIT frames per motor with
embedded `Kp=0`, `Kd=0`, and feedforward torque `0`. Before enable, the tool
uses the measured differential map to reconstruct joint pitch/roll and requires
both motor positions to retain 0.05 rad of soft-limit margin. It then guards
motor drift, velocity, and estimated torque and verifies both motors disabled
on every exit path.

This gate validates only differential feedback, paired enable/disable, and the
500 Hz transport. It does not qualify ankle joint-space PD torque.

Both zero-torque gates passed physically while the feet were suspended. Each
motor completed 1000/1000 command/feedback cycles and both pairs were verified
disabled at exit. The left reconstruction was pitch `0.54425 rad`, roll
`0.01767 rad`; its peak drift, speed, and estimated motor torque were
0.000382 rad, 0.03664 rad/s, and 0.05129 Nm. The right reconstruction was pitch
`0.49735 rad`, roll `-0.03604 rad`; no position drift was observed and its peak
speed and estimated motor torque were 0.01222 rad/s and 0.03175 Nm. Reports:

- `reports/left_ankle_zero_torque_500hz_20260922_114646.json`
- `reports/right_ankle_zero_torque_500hz_20260922_114712.json`

The low-PD gates were separately approved and physically executed on both
sides. Each motor completed 1000/1000 command/feedback cycles at 500 Hz, and
both pairs were verified disabled at exit. They held the measured joint pose
with joint-space `Kp=0.5 Nm/rad`, `Kd=0.05 Nm s/rad`, capped each joint torque
at 0.15 Nm, mapped through `A^-T`, and left embedded motor gains at zero. Peak
estimated motor torque was 0.0464 Nm on the left and 0.0269 Nm on the right.
Reports:

- `reports/left_ankle_low_joint_pd_500hz_20260922_115643.json`
- `reports/right_ankle_low_joint_pd_500hz_20260922_authorized.json`

Because the target was the measured suspended pose, these runs qualify the
500 Hz feedback/torque/disable path, not ankle load capacity or walking gains.

## Qualified subsystem gates before whole-body hold

Both isolated subsystem gates are implemented, covered by unit tests, and now
physically qualified while the robot is suspended:

- Waist yaw/roll measured-pose hold on `kcan1` IDs `0x07/0x08`: 2 seconds at
  50 Hz, embedded `Kp=0.2`, `Kd=0.05`, and zero feedforward. Its first physical
  run failed closed after eight cycles because the DM-J6248 feedback reached
  0.967 Nm against the initial 0.5 Nm guard. A subsequent 500-sample disabled,
  zero-gain baseline held exactly constant position while torque feedback ranged
  from -1.495 to +1.670 Nm. The revised gate keeps the command-torque estimate
  capped at 0.5 Nm and independently caps J6248 feedback at 2.5 Nm (8.3% of its
  30 Nm rated torque). The authorized rerun completed 100/100 command and
  feedback cycles per motor with zero position error. Maximum command-torque
  estimates were 0.000244 Nm yaw and 0.000733 Nm roll; maximum feedback torque
  magnitudes were 0.1983 Nm yaw and 1.3187 Nm roll. Both motors were verified
  disabled at exit. Report:
  `reports/waist_group_low_gain_hold_20260922_124056.json`.
- Head pitch/roll differential measured-pose hold on `kcan2` IDs `0x07/0x08`:
  2 seconds at 500 Hz, host `Kp=0.2`, `Kd=0.03`, 0.05 Nm joint torque cap,
  0.10 Nm motor cap, and embedded motor gains zero. The authorized run completed
  1000/1000 command and feedback cycles per motor. Maximum position errors were
  0.000191 rad pitch and 0.000312 rad roll; maximum joint torque commands were
  0.001132 Nm pitch and 0.003637 Nm roll; maximum motor command magnitude was
  0.003121 Nm. Both motors were verified disabled at exit. Report:
  `reports/head_pair_low_joint_pd_500hz_20260922_130434.json`.

Both launchers refuse to run without exact acknowledgement tokens:

```bash
./hold_sprite0825_waist_group_low_gain_on_253.sh \
  ENABLE_WAIST_GROUP_LOW_GAIN_HOLD
./run_sprite0825_head_pair_low_joint_pd_500hz_on_253.sh \
  ENABLE_HEAD_PAIR_LOW_JOINT_PD_500HZ
```

These runs qualify the suspended measured-pose transport, feedback, torque-map,
and final-disable paths. They do not qualify loaded head motion or walking gains.
The project may now prepare a separately reviewed 31-motor measured-pose hold.
Neither script resets motor zero positions or switches mode.

## Whole-body measured-pose hold

The first Python implementation was deliberately rejected as a physical-runtime
path. In two 0.5 second suspended attempts it preserved the safety envelope and
disabled all motors, but `kcan1/kcan2` reached only about 280-306 Hz for the
ankle endpoints and about 28-32 Hz for their direct endpoints. Lowering the
coverage threshold would have hidden a real scheduling limitation, so the
active gate was moved into the already qualified native C++ transport.

The native gate captures all 31 motor positions while disabled, reconstructs
the 31-joint measured pose, and then uses the frozen mixed-rate architecture:

- ankle pitch/roll: 500 Hz host joint PD, `Kp=0.5`, `Kd=0.05`, 0.15 Nm joint
  cap, `A^-T` differential mapping, 0.10 Nm motor cap, embedded gains zero;
- head pitch/roll: 50 Hz host joint PD, `Kp=0.2`, `Kd=0.03`, 0.05 Nm joint cap,
  `A^-T` mapping, 0.10 Nm motor cap, embedded gains zero;
- all 25 direct joints: 50 Hz embedded MIT hold at the captured motor position,
  `Kp=0.2`, `Kd=0.05`, zero feedforward, and a 0.50 Nm command estimate cap;
- waist roll retains its separately qualified 2.50 Nm feedback-noise guard;
- every failure path sends three whole-body disable passes and polls until the
  final observed state of every endpoint is disabled.

The first native 0.5 second run on 2026-09-22 passed. Each ankle motor completed
250 active command cycles; every other motor completed 25. There were zero
deadline misses, maximum lateness was 0.059544 ms, maximum measured drift was
0.000383 rad, and all 31 endpoints were finally verified disabled. The largest
reported feedback torque was 1.142857 Nm on waist roll, inside its independently
qualified guard. The runtime made zero mode-switch and zero-position-reset
attempts. Evidence:
`reports/native_full_body_measured_pose_hold_20260922_135007.json`.

After separate approval, the same gains, limits, rates, and guards were extended
unchanged to 2.0 seconds. Each ankle motor completed 1000 active command cycles
and every other motor completed 100. There were zero deadline misses, maximum
lateness was 0.060251 ms, maximum measured drift remained 0.000383 rad, and all
31 endpoints were finally verified disabled. Maximum feedback torque was
1.084249 Nm on waist roll; maximum commanded torque estimate across all motors
was 0.003129 Nm. The run made zero mode-switch and zero-position-reset attempts.
Evidence: `reports/native_full_body_measured_pose_hold_20260922_140134.json`.

The qualified launcher is:

```bash
cd /home/tony/open_sprite_runtime
./hold_sprite0825_native_full_body_measured_pose_on_253.sh 0.5
```

The 2.0 second suspended measured-pose gate is now qualified. This result does
not yet authorize policy targets, loaded standing, or walking.

## Next gate: protected policy targets

The first protected-policy tier is implemented and has passed disabled-motor
shadow, deterministic replay, and exact physical-command margin audit. It is
fixed to 2.0 seconds with `gain_scale=0.02`, a 1.0 second measured-pose hold,
and a 4.0 second ramp. The native writer limits each motor to 10% of rated
torque after MIT encoding and independently monitors returned torque, speed,
position, status, temperature, policy freshness, CAN coverage, and timing.

This preparation does not authorize actuation. The suspended physical run must
be reviewed and approved separately using the exact token printed by
`run_sprite0825_native_protected_policy_admission_on_253.sh`. No motor-zero or
mode-switch command exists in this path.

The first explicitly authorized attempt on 2026-09-22 failed closed before
nonzero policy commands were admitted. The measured suspended pose had left
ankle pitch `0.544104 rad` and right ankle pitch `0.497345 rad`, both above the
reviewed `0.386 rad` soft maximum (and the `0.436 rad` hard candidate). The
Python startup projection therefore produced boundary targets, while the native
writer correctly rejected their `-0.158104 rad` and `-0.111345 rad` jumps from
the measured pose. All other joints matched exactly; target velocity, Kp, Kd,
and feedforward were zero. The runtime sent no nonzero policy command, attempted
no mode switch or zero reset, and verified all 31 motors disabled at exit.
Evidence:

- `reports/native_protected_policy_admission_20260922_143734.json`
- `reports/native_protected_policy_actor_20260922_143734.json`
- `reports/native_protected_policy_trace_20260922_143734.npz`

The software now reports measured-pose soft-limit violations by joint and value
instead of silently clamping the startup pose. The safety limits were not
widened. Before another authorized attempt, both disabled ankles must be moved
inside the reviewed soft range and a fresh zero-gain shadow must pass.

The owner repositioned both ankles and authorized a second 2.0 second attempt.
It passed with zero deadline misses, 0.026848 ms maximum lateness, and final
disabled confirmation for all 31 motors. The maximum feedback torque was
1.025641 Nm on waist roll, below its 3.0 Nm cap. Evidence:
`reports/native_protected_policy_admission_20260922_145834.json`.

## Complete-ramp suspended policy tier

A separate 6.0 second tier was qualified in disabled shadow before actuation.
It uses global `gain_scale=0.015`, a `0.1` multiplier on the seven DM-J3507
head/wrist joints, a 1.0 second measured-pose hold, and a 4.0 second ramp. Two
separately authorized physical runs reached `alpha=1.0`, had zero deadline
misses, stayed within every 10%-of-rated-torque commissioning cap, attempted no
mode switch or zero reset, and verified all 31 endpoints disabled. Evidence:

- `reports/native_protected_policy_admission_20260922_145947.json`
- `reports/native_protected_policy_admission_20260922_150121.json`

These passes qualify protected suspended transport through a complete ramp;
they do not qualify the observed startup pose or ground contact. Both traces
showed repeated ankle soft-limit projection. Exact actor replay and observation
counterfactuals established that the behavior existed before actuation and was
caused mainly by the live joint pose plus a roughly 15-degree pelvis pitch tilt.
The frozen G74 training configuration starts with zero root roll/pitch and joint
positions scaled only `0.8–1.2` around their defaults.

The active launcher therefore performs a new 1.0 second disabled zero-gain
readiness shadow before starting its requested tier. The gate requires all four
ankle raw targets to remain inside reviewed soft limits and horizontal projected
gravity norm to remain at or below `0.10`. A regression using the 15:01 trace
failed for exactly the expected reasons while all motors remained disabled. The
next physical task is to align the suspended robot with the G74 reset pose and
obtain a clean readiness report; loaded standing is still blocked.
