# Sim2Real plan

## Gate 0: frozen software contract

- [x] Pin `model1050` as the qualified 100 Hz teacher/baseline.
- [x] Retain G59 `model2999` as the pre-shoulder-upgrade rollback baseline.
- [x] Train, dual-seed qualify, and freeze G74 `model3000`, a native 50 Hz
  deployment actor with 160 ms history and four J4340P proximal shoulder
  pitch/roll motors.
- [x] Reproduce 299 sampled MuJoCo policy actions from recorded observations
  through the packaged ONNX actor with zero numerical difference.
- [x] Replay all 2,998 policy ticks through 29,980 synthetic 500 Hz safety
  ticks with zero action error, zero target-hold error, zero policy overruns,
  and zero permitted hardware transmissions.
- [x] Exclude horizontal base velocity, global position, and global yaw from actor observations.
- [x] Implement and unit-test the PM01-style integrated-IMU-yaw outer command
  controller without adding global yaw to the actor.
- [x] Qualify the outer heading controller and G74 policy under independent
  Isaac seeds 303/404 and the unchanged seven-case MuJoCo matrix. Model3000
  passed all 48 downstream gates.
- [ ] Obtain Tony's final visual sign-off using the frozen model3000 MuJoCo
  review; visual review is the only remaining software-candidate gate.

## Gate 1: hardware inventory

- Complete all 31 motor mappings, limits, and MIT ranges.
- [x] Derive review-only direct and differential motor limit candidates from
  the contract-pinned URDF; physical hard-stop confirmation remains open.
- [x] Read PMAX/VMAX/TMAX registers from all 31 disabled motors using 93
  capability-limited read-only transactions with no write, enable, or mode
  switch attempts, then pin the evidence in the hardware contract.
- [x] Read OT/OC/MAX_SPD and three version registers from all 31 disabled
  motors using 186 capability-limited read-only transactions. Preserve the
  evidence without mislabelling register values as SI nameplate limits.
- Measure four-channel USB-CAN FD latency and sustained bus utilization.
- [x] Establish a hardware emergency-stop chain independent of the SBC process:
  a safety operator controls the regulated supply's independent motor-bus switch.
- Record the power-cut behavior and stop time before releasing ground walking.
- Measure and record pelvis IMU mounting transform, axes, timestamps, update
  rate, gyro bias, and short-term integrated-yaw drift.
- [x] Record a 120-second read-only stationary IMU audit at 99.94 Hz with zero
  rejected frames and -0.0101 deg/min measured yaw drift.
- [x] Install and verify the topology-pinned `/dev/sprite0825-imu` udev link.

## Gate 2: ankle calibration

- [x] Identify left/right position matrices, signs, and motor zeros with legs unloaded.
- [x] Verify position and velocity round trips.
- [x] Verify power-consistent `A^-T` torque mapping at low commanded torque on
  both suspended ankle pairs at 500 Hz. This qualifies transport and mapping,
  not load capacity or walking gains.

## Gate 3: shadow mode

- Read motors and IMU; never transmit policy commands.
- Log raw frames, normalized observations, actor output, proposed motor targets,
  safety margins, loop jitter, and state age.
- Replay every log deterministically offline.
- [x] Replay every recorded 50 Hz policy tick through the packaged ONNX actor and
  expand each output through ten 500 Hz zero-order-held safety ticks. This is a
  no-transmit software check; live state frames are still required for Gate 3.
- Demonstrate that stale state, stale command, policy overrun, and an open
  emergency-stop input all force a no-transmit safe hold. State, overrun, and
  emergency-stop faults remain latched until explicitly cleared after a
  separate healthy check.
- [x] Implement exact 31-motor telemetry checks for finite state, hard position,
  speed, torque, current, temperature, and torque-speed envelope. Any violation
  latches the no-transmit `motor_limit` fault. Live frames are still required.
- Measure 500 Hz loop jitter on the selected SBC under representative USB-CAN,
  logging, and inference load. A desktop timing probe is diagnostic only and
  cannot certify the Raspberry Pi 5 or Jetson Orin Nano.

The deterministic safety scenarios and a no-load 500 Hz probe pass on the 234
development PC. The final G74 60-second MuJoCo trace also passed 2,998 ONNX
policy ticks and 29,980 synthetic 500 Hz safety ticks with zero action/hold
error and zero permitted hardware transmissions. This does not complete Gate 3: live
motor/IMU frames, USB-CAN load, full logging, and the selected SBC are still
required.

### Native transport result (2026-09-21)

- [x] Pin a native C++ absolute-time transport loop to Jetson CPU 5.
- [x] Schedule ankle motors at 500 Hz and all other motors at 50 Hz without
  issuing more than one frame per bus per 0.5 ms slot.
- [x] Complete a 120-second, 31-motor, four-bus zero-gain shadow run with zero
  deadline misses, complete expected feedback, and no new CAN errors or drops.
- [x] Connect the 50 Hz Python/ONNX actor to the native layer through a versioned,
  sequence-numbered, stale-target-failing shadow interface and qualify it for
  120 seconds under full CAN and IMU load. Repeat against the stable IMU udev
  path with 60,000 native ticks, 100% feedback coverage on all 31 motors, zero
  deadline misses, and zero nonzero command attempts.
- [x] Implement and qualify the frozen two-tick measured-pose-equivalent-action
  smoothstep handoff in the live policy path, matching the MuJoCo deployment.
- [x] Fail closed on invalid target ABI/order, future source sequence, stale
  timestamp, embedded `Kd > 3`, ordinary target timeout, and missing first
  target; verify all injected faults against the native process.
- [x] Export the frozen 31-joint/31-motor affine kinematics into C++, reconstruct
  joint state there, and preview the exact final motor command at the real
  500 Hz/50 Hz schedule. Enforce motor soft/hard/protocol position, deployment
  speed, embedded gain, protocol torque, mechanical peak torque, status, and
  temperature limits before any future writer boundary.
- [x] Record and deterministically replay a per-policy-tick live trace containing
  measured joint state, IMU-derived actor inputs, observation history, raw
  actor action, handoff action, proposed joint targets, and command margins.
  The 120-second/6,000-tick trace replayed with exactly zero actor error. Its
  unmodified physical-command audit intentionally fails closed: the disabled,
  suspended robot cannot follow the proposed targets, the two-tick handoff
  applies 50% policy blend on its first tick, and four policy joint targets
  exceed URDF-derived soft limits.
- [x] Demonstrate an offline protected-actuation candidate against the same
  live trace. Joint-space soft-limit projection plus `gain_scale=0.1` produced
  zero position/velocity/Kd/torque violations and zero ankle torque saturation.
  This is an initial suspended-test setting, not a walking gain qualification.
- [x] Implement joint-space soft-limit projection and startup gain scaling in
  the Python policy client, plus an independently exported, order-hashed joint
  envelope in the C++ native IPC receiver. Qualify `gain_scale=0.1` for 10
  seconds under the full four-bus/IMU load and verify fail-closed rejection of
  an out-of-limit position and excessive Kp.
- [x] Separate physical startup from the frozen two-tick MuJoCo handoff. Capture
  the measured pose, hold it for one second, then smoothstep both position and
  the protected gain tier over four seconds. The zero-gain live shadow completed
  this sequence with a maximum per-tick target change of 0.01881 rad.
- [ ] Add an approved conservative 38 V dynamic torque-speed/current envelope;
  the manuals do not provide enough 38 V curve data to infer one safely.
- [ ] Carry only validated ramped targets into a separately reviewed nonzero
  native writer. The current writer remains physically incapable of emitting
  nonzero Kp, Kd, velocity, or feedforward torque.

## Gate 4: protected actuation

- [x] Prepare a single-purpose first-powered hold restricted to the independent
  DM-J3507 head-yaw motor. It captures measured position, uses frozen
  `Kp=0.2`, `Kd=0.05`, zero feedforward, applies strict 0.1 Nm/0.2 rad/s
  guards, and always sends and verifies disable on exit.
- [x] Execute and review the two-second head-yaw current-position hold with the
  robot supported and the safety operator on the independent power cut-off.
  The 2026-09-21 test completed 100/100 control/feedback cycles, held within
  0.000384 rad and 0.003664 Nm, and verified disabled feedback after exit.
- [x] Prepare a separately gated, fixed second test for the unloaded head-yaw
  shaft: quintic `+0.02 -> -0.02 -> start` motion at 50 Hz with `Kp=1.0`,
  `Kd=0.2`, zero feedforward, and the same fail-closed guards.
- [x] Execute and safety-review the unloaded head-yaw smooth-motion test. The
  2026-09-21 run completed 225/225 command/feedback cycles, stayed below
  0.0611 rad/s and 0.0306 Nm, reached at most 40/35 C MOS/rotor temperature,
  reported no guard errors, and verified `disabled` after three disable frames.
- [x] Execute a visible unloaded head-yaw test with frozen relative `+10/-10`
  degree commands. Two conservative commissioning attempts failed closed on
  the position-error and speed guards. The final 15-second run completed
  750/750 cycles, measured approximately `+8.59/-8.79` degrees at low gain,
  stayed below 0.526 rad/s and 0.080 Nm, and verified `disabled` on exit.
- [x] Validate the first joint with a real mechanism on `kcan4`: the right-wrist
  roll current-position hold completed 100/100 cycles below 0.004 Nm, then the
  joint-space `+/-5 degree` test applied `policy_to_motor_sign=-1`, completed
  750/750 cycles below 0.092 Nm, and verified `disabled` on exit.
- [x] Extend the fail-closed writer from one endpoint to the fixed right-wrist
  yaw/pitch/roll group on `kcan4`, including all-group disable and per-motor
  disabled verification on every exit path.
- [x] Commission the fixed right-wrist current-position hold before any larger
  arm or whole-body enable. Its first attempt correctly sent zero enable frames
  because wrist yaw was 0.00833 rad from its negative hard boundary. After the
  disabled joint was manually returned toward neutral, the gate completed
  100/100 cycles per motor below 0.0171 Nm and disabled all motors at exit.
- [x] Commission both fixed seven-motor arm groups at measured pose. Both sides
  completed 100/100 cycles per motor and verified every endpoint disabled;
  head yaw remained excluded from the left-arm writer allowlist.
- [x] Build the leg/waist commissioning path around the calibrated differential
  ankle mapping and dedicated 500 Hz ankle layer. The direct arm-group hold is
  not used for either ankle motor pair.
- [x] Prepare and offline-test the first powered differential-ankle transport
  gate. It reconstructs pitch/roll through the measured matrix, then enables
  only one ankle pair for two seconds at 500 Hz per motor with embedded gains
  and feedforward torque all exactly zero.
- [x] Physically qualify both zero-torque ankle transport gates while suspended:
  1000/1000 cycles per motor at 500 Hz, calibrated pitch/roll reconstruction,
  negligible drift, and verified disabled feedback on both pairs.
- [x] Implement and offline-test the next low joint-space ankle-PD gate with
  `Kp=0.5`, `Kd=0.05`, 0.15 Nm joint torque caps, `A^-T` motor mapping, zero
  embedded motor gains, and fail-closed pair disable.
- [x] Execute both low ankle-PD gates after separate explicit owner approvals.
  Both completed 1000/1000 cycles per motor, stayed below 0.047 Nm estimated
  motor torque, and verified both endpoints disabled at exit.
- [x] Prepare, offline-test, and physically commission fixed four-motor
  proximal-leg groups for each side. Their allowlists contain only hip
  pitch/roll/yaw and knee; ankle, waist, and head endpoints are excluded. Both
  sides completed 100/100 cycles per motor and verified disabled on exit.
- [x] Prepare and unit-test isolated waist yaw/roll and head pitch/roll
  differential measured-pose gates.
- [x] Physically commission the waist yaw/roll gate with separate approval. Its
  first run failed closed on the J6248 feedback guard; disabled zero-gain data
  proved a -1.495 to +1.670 Nm feedback-noise span at fixed position. The revised
  gate separates a 0.5 Nm command estimate from a 2.5 Nm J6248 feedback guard.
  The authorized rerun completed 100/100 cycles per motor with zero position
  error, command estimates below 0.000733 Nm, and verified both motors disabled.
  Evidence: `reports/waist_group_low_gain_hold_20260922_124056.json`.
- [x] Physically commission the lower-torque head differential gate with
  separate approval. It completed 1000/1000 cycles per motor at 500 Hz, stayed
  below 0.00313 Nm commanded motor torque, and verified both motors disabled.
  Evidence: `reports/head_pair_low_joint_pd_500hz_20260922_130434.json`.
- [x] Implement and physically qualify the capability-limited native 31-motor
  measured-pose hold before connecting policy targets. The 0.5 second suspended
  run delivered exact 500 Hz ankle and 50 Hz remaining-joint coverage, zero
  deadline misses, 0.059544 ms maximum lateness, at most 0.000383 rad measured
  drift, and final disabled confirmation for all 31 motors. The earlier Python
  path was rejected after measuring only 280-306 Hz ankle coverage rather than
  weakening the gate. Evidence:
  `reports/native_full_body_measured_pose_hold_20260922_135007.json`.
- [x] With renewed physical approval, extend the same unchanged native
  measured-pose hold from 0.5 seconds to 2.0 seconds. The run delivered 1000
  active cycles per ankle motor and 100 per remaining motor, zero deadline
  misses, 0.060251 ms maximum lateness, 0.000383 rad maximum drift, and final
  disabled confirmation for all 31 motors. Evidence:
  `reports/native_full_body_measured_pose_hold_20260922_140134.json`.
- [x] Prepare and offline-qualify the native protected-policy target path after
  the 2.0 second measured-pose hold passed. It preserves the measured-pose
  startup hold/ramp, 50 Hz policy, 500 Hz ankle controller, watchdogs, and
  independent power cut-off. Physical nonzero policy actuation remains a
  separate, explicitly authorized gate.
- [x] Execute the first 2.0 second suspended protected-policy admission after
  operator review and exact authorization. After correcting the ankle startup
  pose, two authorized runs passed with zero deadline misses and all 31 motors
  verified disabled at exit.
- [x] Execute the separately authorized 6.0 second complete-ramp suspended
  policy tier twice. Both runs reached `alpha=1.0`, stayed inside the fixed
  10%-of-rated-torque commissioning caps, and verified every endpoint disabled.
- [x] Requalify the upright startup pose with the zero-gain readiness preflight,
  then complete suspended 1.0 Nm and per-leg-motor 2.0 Nm policy tiers. The
  20-second disturbance run confirmed live IMU-to-policy reaction, zero native
  deadline misses, and final-disabled feedback from all 31 motors.
- [ ] Execute the first partial-contact standing gate. The lifting frame must
  continue carrying most of the robot weight, both soles may only touch a flat
  floor, leg motor command caps remain 2.0 Nm, and free standing remains
  unauthorized until this gate and its trace review pass.
- [ ] Treat every motor zero-position reset/write as a separately authorized
  maintenance operation. Never emit one without explicit owner approval for
  that exact operation.
- Single-joint tests, then fixed arm groups, then whole-body measured pose in a
  lifting frame.
- Low Kp/Kd and strict current limits first.
- Stand, weight shift, one step, 0.15 m/s walk, stop, and restart.

## Protected-policy admission preparation

The native runtime now has a separate protected-policy writer. It is not a
general CAN command path: it requires the exact acknowledgement token, live
policy IPC, matching joint-order hash, exported kinematics and joint safety,
and `CAP_SYS_NICE` for the reviewed SCHED_FIFO loop. A native rebuild removes
that file capability, so it must be restored explicitly with:

```bash
./install_sprite0825_native_realtime_capability_on_253.sh
```

The first physical tier is fixed at 2.0 seconds, policy gain scale `0.02`, a
1.0 second measured-pose hold, and a 4.0 second smooth ramp. Each motor is
limited to 10% of its hardware rated torque, independently of both protocol
`TMAX` and mechanical peak torque. The runtime checks the torque again after
MIT quantization, watches policy age, hard position, speed, feedback torque,
temperature, CAN status, coverage, and real-time deadlines, and performs a
three-pass whole-body disable plus disabled-feedback verification on every exit
path. Signals request the same orderly fail-closed shutdown. The path contains
no mode-switch or zero-position-reset operation.

The exact first-tier parameters were exercised for 2.0 seconds in disabled,
zero-gain shadow on 2026-09-22. All 31 motors had complete scheduled coverage,
the native 2 kHz loop had zero deadline misses (maximum lateness 0.026370 ms),
and CAN transmitted no nonzero gain or torque. Deterministic replay of all 100
policy ticks had zero action error. The physical-command audit used the exact
final startup fields and found no position, velocity, Kd, protocol torque,
mechanical peak, or 10%-of-rated commissioning torque violations. The largest
commissioning-cap ratio was head yaw at 0.13955; the largest estimated motor
torque was 0.03315 Nm. Evidence:

- `reports/native_policy_ipc_transport_20260922_142831.json`
- `reports/native_policy_ipc_actor_20260922_142831.json`
- `reports/native_policy_ipc_trace_20260922_142831.npz`
- `reports/native_policy_ipc_replay_20260922_142831.json`
- `reports/native_policy_ipc_command_margin_20260922_142831.json`

The active launcher intentionally refuses to run without the exact token
`ENABLE_NATIVE_PROTECTED_POLICY_ACTUATION`. Its physical execution is still
pending a fresh operator approval after startup-pose correction. The first
authorized attempt on 2026-09-22 failed closed before nonzero policy admission:
both suspended ankle-pitch measurements were outside the reviewed soft range.
All 31 motors were verified disabled, with zero mode-switch and zero-reset
attempts. The limits were not widened; both ankles must first be moved inside
the reviewed range and requalified in zero-gain shadow.

The owner corrected that pose and authorized a rerun. The 2.0 second admission
passed at 14:58:34 with zero deadline misses, 0.026848 ms maximum lateness, and
all 31 endpoints disabled at exit. The largest feedback torque was 1.025641 Nm
on waist roll, below its 3.0 Nm commissioning cap. Evidence:
`reports/native_protected_policy_admission_20260922_145834.json`.

The next fixed tier used 6.0 seconds, `gain_scale=0.015`, the same 1.0 second
hold and 4.0 second ramp, and a separate `0.1` gain multiplier for the seven
DM-J3507 head/wrist joints. It was first qualified in zero-gain shadow with
deterministic replay and exact command-margin audit, then executed twice with
separate owner authorization. Both active runs completed the full ramp with
zero deadline misses and final-disabled confirmation for all motors. Evidence:

- `reports/native_protected_policy_admission_20260922_145947.json`
- `reports/native_protected_policy_admission_20260922_150121.json`

Post-run trace analysis found that the safety layer was repeatedly clamping
left/right ankle pitch and right ankle roll. This was not a reason to widen the
limits. Counterfactual ONNX replay reproduced the first actor tick exactly and
showed that the abnormal actions were already present during the measured-pose
hold. They were driven primarily by the live joint pose and projected gravity,
not by velocity, action history, angular velocity, or actuation feedback. The
live projected gravity `[0.2558, 0.0301, -0.9663]` represents roughly 15 degrees
of pitch tilt. The frozen G74 source snapshot resets roll/pitch to zero and only
scales joint positions by `0.8–1.2` around the default pose, so the suspended
startup was outside the trained reset distribution.

The active launcher now runs a 1.0 second disabled zero-gain readiness shadow
before any nonzero policy tier. It fails closed if any ankle raw target leaves
its reviewed soft range or if horizontal projected gravity exceeds `0.10`.
Offline regression against the 15:01 trace correctly rejected 157 left ankle
pitch, 85 right ankle pitch, and 149 right ankle roll violations plus a `0.2621`
horizontal-gravity norm. No further active or ground-contact test is permitted
until a fresh live preflight passes.

The startup pose was subsequently corrected and repeatedly passed that live
preflight. Suspended policy admission progressed through 1.0 Nm global caps and
then 2.0 Nm per-leg-motor caps. The 20-second disturbance run at 17:34 completed
with zero real-time deadline misses, no mode switch, no zero reset, and all 31
motors verified disabled. The largest command/feedback values were 1.527/1.361
Nm on the right knee; the externally loaded waist-roll feedback reached 2.667
Nm under its separate 3.0 Nm feedback guard. Evidence:
`reports/native_protected_policy_admission_20260922_173434.json` and
`reports/native_protected_policy_trace_20260922_173434.npz`.

The disturbance trace and exact ONNX counterfactual replay establish that the
principal attitude response has the sign of negative feedback. Removing only
the live IMU history while preserving joint state and action history changed
the ankle, knee, hip, and waist targets; mirroring the IMU disturbance mirrored
the target response with cosine similarity 0.9826. With both ankle pitch axes
equal to `-Y` and both ankle roll axes equal to `-X`, the summed ankle response
opposed positive pitch/roll rate and positive pitch/roll tilt. Measured angular
speed peaked at 0.470 rad/s and fell to approximately 0.033 rad/s by shutdown.
This validates reaction direction while suspended, not ground-contact balance.

The same disturbance produced 43 left and 135 right ankle-pitch target clamps.
This is not a zero or differential-mapping error: measured ankle pitch remained
inside the hard range, while the raw right target reached -0.540 rad, beyond
the -0.436 rad URDF hard limit. The runtime correctly projected it to the
-0.386 rad reviewed soft limit. Do not widen either limit to hide this result.
The next partial-contact gate must determine whether foot contact keeps the
policy inside its trained regime and reduces this saturation before torque caps
or supported weight are increased.

The first 8-second partial-contact run at 18:00 passed all startup and active
guards. The lifting frame carried most weight while both soles touched the
floor. All four ankle clamp counters remained zero, the native loop had zero
deadline misses, and all 31 motors were verified disabled at exit. The operator
applied front/back and left/right disturbances near the end of the run. The
maximum horizontal projected-gravity norm was 0.0616 (about 3.53 degrees), and
maximum pitch angular speed was 0.153 rad/s. Because the trace ended while the
robot was still recovering, this run validates safe contact response but not
return to steady state. Evidence:
`reports/native_protected_policy_admission_20260922_180051.json` and
`reports/native_protected_policy_trace_20260922_180051.npz`.

A separate 20-second partial-contact tier preserves the same gains, 2.0 Nm leg
command caps, 2.2 Nm leg feedback guards, lower non-leg limits, and the four
ankle 100 ms consecutive-clamp watchdog. It exists only to capture separated
front/back and left/right disturbances plus complete recovery; it requires a
fresh exact operator acknowledgement and the native extended-duration token.

The first 20-second attempt stopped safely at approximately 17.2 seconds when
the right ankle-pitch raw target exceeded its reviewed soft limit by 0.0557 rad
for five consecutive policy ticks. All motors were verified disabled. The robot
was supported by two flat straps attached at the shoulders. This fixture strongly
constrains roll but permits substantially more pelvis pitch, so the front/back
and left/right pushes are not comparable independent balance tests. The ankle
pitch event may be delayed recovery from the earlier front/back disturbance.
Do not use this fixture to qualify roll balance, and do not widen the ankle
limit. Repeat with a pitch-only disturbance and preserved partial-fault trace
before changing gains, torque caps, or supported weight.

The forward-to-rear pitch-only repeat at 18:22 completed the full 20 seconds.
The disturbance peaked at 5.93 seconds with 0.108 rad/s pitch rate and 1.84
degrees total tilt. Pitch rate and attitude returned to the final baseline for
at least one continuous second by 8.03 seconds, 2.10 seconds after the peak.
No ankle target was projected or clamped. The right ankle-pitch raw target
reached -0.210 rad, retaining about 0.176 rad margin to its -0.386 rad reviewed
soft limit. All 31 motors were verified disabled at exit. This qualifies light
pitch recovery in one direction under the current harness, but not the opposite
pitch direction, roll recovery, or unsupported standing. Evidence:
`reports/native_protected_policy_admission_20260922_182203.json`,
`reports/native_protected_policy_trace_20260922_182203.npz`, and
`reports/partial_contact_pitch_only_analysis_20260922_182203.json`.

The opposite rear-to-front pitch test at 18:25 also completed all 20 seconds.
It produced the expected opposite pitch-rate sign (-0.123 rad/s versus +0.108
rad/s in the first direction), reached 3.07 degrees maximum tilt, and returned
to the final pitch baseline for at least one continuous second 1.36 seconds
after the rate peak. The more demanding right ankle-pitch raw target reached
-0.304 rad, retaining about 0.082 rad margin to the -0.386 rad soft limit; no
ankle target was projected or clamped. All motors were verified disabled at
exit. Bidirectional light pitch recovery is therefore qualified only under the
current mostly unloaded two-shoulder-strap fixture. Evidence:
`reports/native_protected_policy_admission_20260922_182549.json`,
`reports/native_protected_policy_trace_20260922_182549.npz`, and
`reports/partial_contact_reverse_pitch_analysis_20260922_182549.json`.

After the shoulder straps were lowered slightly, a six-second zero-gain
preflight and an eight-second increased-load static contact run both passed.
No disturbance was applied during the active run. The native loop had zero
deadline misses, no joint target was projected or clamped, and all 31 motors
were verified disabled at exit. Attitude stayed nearly constant: total tilt
started at 1.76 degrees, ended at 1.70 degrees, and peaked at 1.81 degrees.
The highest measured leg feedback torque was 0.786 Nm at the right knee; waist
roll reached 1.026 Nm. During the startup ramp, the right ankle-pitch raw
target reached -0.3518 rad, leaving about 0.0342 rad to the reviewed -0.386 rad
soft limit. Its full-ramp target then remained between -0.349 and -0.339 rad. This
qualifies one additional static supported-load tier, not push recovery,
unsupported standing, or another reduction in harness support. Do not increase
load or add a disturbance until the right ankle-pitch target margin has been
reviewed. Evidence:
`reports/native_protected_policy_admission_20260922_183124.json` and
`reports/native_protected_policy_trace_20260922_183124.npz`.

The next static contact tier separates proximal support authority from distal
safety. Its policy gain scale is 0.06; only the left/right hip pitch and roll
motors receive 4.5 Nm command and 5.0 Nm feedback caps. Hip yaw, knees, and
ankles remain at the qualified 2.0 Nm command and 2.2 Nm feedback caps, and all
non-leg limits remain unchanged. The harness stays at the previously qualified
height and no disturbance is allowed. A zero-gain shadow preview must pass
before this tier may be actively authorized.

That 0.06-gain tier completed all eight seconds with zero deadline misses, no
target projection or ankle clamp, and all 31 motors verified disabled. Total
tilt started at 1.74 degrees, ended at 1.70 degrees, and peaked at 1.80 degrees.
The largest measured leg feedback was 1.265 Nm at the right knee; the largest
proximal-hip feedback was 0.732 Nm. Right ankle pitch reached a -0.3604 rad raw
target during the ramp and stayed between -0.3572 and -0.3391 rad after the
ramp, while measured ankle position stayed near +0.297 rad. An exact actor
counterfactual showed the target changes sign with measured ankle position and
therefore does not indicate another feedback-sign error. A global 0.08 preview
was rejected before actuation because predicted right-shoulder-pitch torque was
1.125 Nm, above its unchanged 1.0 Nm command guard. The revised candidate keeps
the four hip pitch/roll joints at global gain scale 0.08, applies a 0.75
multiplier to every other non-DM3507 joint, and applies 0.075 to DM3507 joints.
This preserves effective gains of 0.06 and 0.006 outside the four proximal hip
joints without weakening the safety layer's multiplier range of (0, 1]. Every
torque and position guard remains unchanged. It requires a matching zero-gain
preview and explicit active-test approval.
Evidence: `reports/native_protected_policy_admission_20260922_184812.json` and
`reports/native_protected_policy_trace_20260922_184812.npz`.

The corrected hip-only 0.08-gain tier also completed all eight seconds with
zero deadline misses, no target projection or ankle clamp, and all 31 motors
verified disabled. Total tilt ended at 1.68 degrees and averaged 1.67 degrees
after the startup ramp. Right-knee movement after the ramp fell from 0.0134 rad
in the 0.06 tier to 0.0053 rad, consistent with stronger proximal support. The
largest hip feedback was 0.937 Nm and right-knee feedback was 1.032 Nm. Right
ankle pitch reached -0.3642 rad, retaining about 0.0218 rad to its reviewed
soft limit. Before changing support height or applying a disturbance, repeat
this exact tier for 20 seconds with no disturbance to detect delayed drift or
clamp accumulation. Evidence:
`reports/native_protected_policy_admission_20260922_185914.json` and
`reports/native_protected_policy_trace_20260922_185914.npz`.

The exact hip-only 0.08-gain configuration then completed a 20-second static
contact run at the same harness height with zero deadline misses, no target
projection or ankle clamp, and all 31 motors verified disabled at exit. Total
tilt started at 1.69 degrees, ended at 1.66 degrees, peaked at 1.80 degrees,
and averaged 1.68 degrees during the final five seconds; no delayed attitude
drift was observed. The largest measured hip feedback was 1.032 Nm at right
hip pitch, and right-knee feedback peaked at 1.238 Nm. Right ankle-pitch target
stayed between -0.3622 and -0.3530 rad during the final five seconds, retaining
at least about 0.024 rad to the reviewed -0.386 rad soft limit. This qualifies
the sustained static tier only. It does not qualify a disturbance, unsupported
standing, or a simultaneous reduction in harness support. Evidence:
`reports/native_protected_policy_admission_20260922_190230.json` and
`reports/native_protected_policy_trace_20260922_190230.npz`.

After the harness was lowered slightly, the next zero-gain readiness preview
was rejected before motor enable. The static configuration changed materially:
right ankle-pitch feedback moved from about +0.307 rad to +0.460 rad, left
ankle-pitch moved from about +0.274 rad to +0.360 rad, and total body tilt rose
from about 1.68 degrees to 2.33 degrees. The actor requested right ankle-pitch
below its -0.386 rad reviewed soft limit on 258 of 300 policy ticks, with a
maximum raw overshoot of 0.0061 rad and a 201-tick consecutive run. Do not
relax the soft limit or actuate from this configuration. Raise the harness
partway toward the qualified height, leave both feet naturally flat, and repeat
the same zero-gain readiness gate. Evidence:
`reports/native_policy_ipc_actor_20260922_191024.json` and
`reports/native_policy_ipc_trace_20260922_191024.npz`.

After the harness was raised partway back, the hip-only 0.08-gain tier passed
its zero-gain preview but was stopped fail-closed about 5.9 seconds into active
control by the 1.0 Nm right-shoulder-pitch command guard. All 31 motors were
verified disabled. The trace showed the actual right ankle pitch moving from
+0.129 rad to +0.327 rad while its projected target reached the -0.386 rad soft
limit. The right shoulder command was a downstream whole-body compensation,
not the support root cause, so its guard was not relaxed. Evidence:
`reports/native_protected_policy_admission_20260922_191449.json` and
`reports/native_protected_policy_trace_20260922_191449.npz`.

The corrected candidate keeps arms and waist at effective gain 0.06 and DM3507
joints at 0.006, but raises hip yaw, knees, and ankles to effective gain 0.08 so
all leg joints match the already-qualified hip pitch/roll gain. Motor torque
caps remain unchanged. This all-leg 0.08 tier completed eight seconds at the
adjusted intermediate harness height with zero deadline misses, no target
projection or ankle clamp, and all 31 motors verified disabled. Total tilt
started at 1.49 degrees, ended at 1.46 degrees, and averaged 1.46 degrees after
the startup ramp. Right ankle-pitch feedback stayed nearly fixed at +0.3441 to
+0.3432 rad; its post-ramp target stayed between -0.2534 and -0.2292 rad,
retaining at least about 0.133 rad to the -0.386 rad soft limit. Maximum measured
feedback was 0.814 Nm at right hip pitch, 0.595 Nm at right knee, and 0.354 Nm
among the right ankle motors. This qualifies only static contact at this harness
height, with no disturbance. Evidence:
`reports/native_protected_policy_admission_20260922_192258.json` and
`reports/native_protected_policy_trace_20260922_192258.npz`.

The same all-leg 0.08 configuration then completed a 20-second static contact
run after the Jetson reboot at the same adjusted intermediate harness height.
It had zero deadline misses, no target projection or ankle clamp, and all 31
motors were verified disabled at exit. Total tilt started at 1.509 degrees,
ended at 1.502 degrees, and stayed between 1.485 and 1.531 degrees. Mean tilt
for the four successive five-second windows was 1.513, 1.509, 1.510, and
1.511 degrees, so there was no delayed attitude drift. Right ankle-pitch
feedback stayed nearly fixed at +0.3442 to +0.3434 rad; its final-five-second
target stayed between -0.2519 and -0.2465 rad, retaining at least about 0.134
rad to the reviewed -0.386 rad soft limit. Maximum measured feedback was
0.786 Nm at right hip pitch, 0.663 Nm at right hip roll, 0.581 Nm at right
knee, and 0.359 Nm among the right ankle motors. This qualifies sustained
static contact only at this harness height. It does not qualify a disturbance,
unsupported standing, walking, or a simultaneous support-height change.
Evidence: `reports/native_protected_policy_admission_20260923_104734.json` and
`reports/native_protected_policy_trace_20260923_104734.npz`.

Lowering the harness slightly from that qualified height materially changed
the static ankle geometry. The next zero-gain readiness preflight rejected the
pose on its first state, before any nonzero command or motor enable: measured
left and right ankle pitch were +0.4374 and +0.4803 rad, respectively, beyond
the reviewed +0.386 rad soft limit. Relative to the preceding qualified pose,
both ankles moved by about +0.13 rad in the same direction, consistent with a
support-height/load change rather than a unilateral feedback fault. Do not
relax the soft limit or run active control from this pose. Raise the harness
slightly or reposition the feet so both measured ankle-pitch joints are below
+0.35 rad, preserving useful margin, and repeat the identical zero-gain
readiness gate. Evidence:
`reports/native_policy_ipc_actor_20260923_105255.json`.

After manually correcting both ankles, a repeated six-second zero-gain shadow
completed with full feedback coverage, zero deadline misses, and no target
projection. Both ankle-pitch joints returned inside the reviewed envelope
(left settled near +0.348 rad and right near +0.292 rad). Active admission was
still withheld because the whole-leg pose had changed substantially from the
qualified baseline: left hip pitch was about -0.35 rad and left knee about
+0.64 rad, compared with -0.05 and +0.33 rad in the qualified 20-second run.
The resulting preview estimated 3.54 Nm at the left knee, above its unchanged
2.0 Nm commissioning command cap. Do not raise the knee torque cap. With all
motors disabled, reduce the asymmetric left-leg crouch while keeping both
ankle-pitch joints below +0.35 rad, then repeat this zero-gain gate. Evidence:
`reports/native_policy_ipc_actor_20260923_110251.json`,
`reports/native_policy_ipc_transport_20260923_110251.json`, and
`reports/native_policy_ipc_trace_20260923_110251.npz`.

The owner subsequently confirmed that the adjusted static pose is mechanically
safe and requested a commissioning-envelope review. The right ankle-pitch
measured pose was +0.4142 rad: outside the symmetric +0.386 rad soft candidate
but still inside the +0.436 rad URDF hard candidate. For this commissioning
stage only, the right ankle-pitch positive soft boundary is raised minimally to
+0.425 rad. Its negative boundary remains -0.386 rad, the left ankle remains
[-0.386, +0.386] rad, and all torque, speed, clamp-watchdog, and motor hard
guards remain unchanged. This leaves about 0.011 rad on either side of the
current pose between the revised soft boundary and the hard candidate. The
native safety exporter now rejects any soft candidate that lies outside its
corresponding hard candidate. This change authorizes only a fresh zero-gain
readiness evaluation; active control still requires all torque gates and an
explicit approval.

The zero-gain evaluation with that reviewed positive boundary accepted the
measured pose and completed with full CAN coverage and zero deadline misses,
but it did not qualify policy actuation. Body tilt was about 3.64 degrees,
compared with about 1.51 degrees in the qualified 20-second baseline. Measured
right ankle pitch was +0.4142 rad while the actor requested as low as -0.4458
rad. Projection clamped that target to the unchanged -0.386 rad negative soft
boundary on 293 of 300 policy ticks, leaving a roughly 0.80 rad projected
measured-to-target separation. The preview also estimated 2.315 Nm at the
right knee, above its unchanged 2.0 Nm commissioning command cap. Do not widen
the negative ankle boundary or raise the knee cap to force admission: those
would hide a policy/start-pose incompatibility. This pose may be used for an
explicitly approved measured-pose hold, but policy actuation requires a closer
standing start pose and a fresh zero-gain gate. Evidence:
`reports/native_policy_ipc_actor_20260923_111343.json`,
`reports/native_policy_ipc_transport_20260923_111343.json`, and
`reports/native_policy_ipc_trace_20260923_111343.npz`.

After the right ankle was manually repositioned again, the zero-gain readiness
gate completed with no projection or clamp and estimated at most 1.84 Nm, so
the owner's available 3.5 Nm leg ceiling was not needed. The explicitly
authorized eight-second active test therefore retained the more conservative
2.0 Nm non-hip leg command cap, 2.2 Nm feedback cap, and 4.5/5.0 Nm hip
pitch/roll caps. It completed with zero deadline misses, no ankle clamp, and all
31 motors verified disabled. Total tilt decreased from 3.29 to 2.59 degrees;
pitch improved from about -2.00 to -0.80 degrees. Maximum measured feedback was
1.374 Nm at left hip pitch, 1.320 Nm at right hip pitch, 0.745 Nm at the right
knee, and 0.295 Nm among the ankle motors. The joint trajectories moved
smoothly toward their policy targets without approaching a reviewed position
boundary. This qualifies the adjusted pose for a 20-second repeat at identical
support height and limits, without disturbance; it does not qualify reduced
support or walking. Evidence:
`reports/native_protected_policy_admission_20260923_112015.json` and
`reports/native_protected_policy_trace_20260923_112015.npz`.

The explicitly authorized 20-second repeat at the same support height and
limits also passed with zero deadline misses, no target clamp, and all 31
motors verified disabled. Mean tilt over successive five-second windows was
2.662, 2.595, 2.608, and 2.611 degrees; final-window drift was only -0.001
degrees. The remaining attitude offset was predominantly a stable roll near
-2.46 degrees rather than a growing instability. Maximum measured feedback was
1.210 Nm at left hip pitch, 1.128 Nm at right hip pitch, 0.650 Nm at right hip
yaw, 0.622 Nm at right knee, and 0.266 Nm among the ankle motors. The 2.0 Nm
non-hip leg cap remains sufficient; do not raise it to the available 3.5 Nm
ceiling without evidence. This qualifies sustained static contact at the
current support height only. The next progression is a small reduction in
harness support while preserving the settled leg and foot pose, followed by a
fresh zero-gain readiness gate before any actuation. Evidence:
`reports/native_protected_policy_admission_20260923_112251.json` and
`reports/native_protected_policy_trace_20260923_112251.npz`.

After the harness was lowered again, the first zero-gain preview passed with no
clamp and estimated at most 1.27 Nm. A separately authorized eight-second
active attempt then repeated its mandatory immediate preflight, which estimated
2.0067 Nm at the right knee and rejected the run before motor enable because
the fixed non-hip leg command cap was 2.0 Nm. No active policy command was sent.
The owner had already confirmed that up to 3.5 Nm is acceptable for the leg
motors, but the next tier increases only as much as needed: non-hip leg command
and feedback caps become 2.2 and 2.5 Nm. Hip pitch/roll remain at 4.5/5.0 Nm,
all gains and all other motor caps remain unchanged, and the ankle clamp
watchdog remains active. This new tier requires its own explicit approval.
Evidence: `reports/native_protected_policy_preflight_20260923_112720.json` and
`reports/native_policy_ipc_trace_20260923_112720.npz`.

The separately approved eight-second lower-support test at the conservative
2.2/2.5 Nm non-hip leg command/feedback tier passed its repeated immediate
zero-gain gate and the active run. The preflight estimated 2.0047 Nm at the
right knee, below the new command cap. Active control completed with zero
deadline misses, zero target clamp, no ankle clamp-watchdog event, no rejected
IMU frame, and all 31 motors verified disabled. Maximum measured feedback was
1.566 Nm at right hip pitch, 1.361 Nm at right knee, 1.251 Nm at left hip
pitch, and 0.286 Nm among the ankle motors. Total tilt changed from 3.148 to
3.061 degrees; its final three-second mean was 3.071 degrees. The remaining
attitude offset stayed predominantly a steady roll near -3.064 degrees, while
the final three-second mean pitch was -0.183 degrees. This qualifies this
lower-support height for a sustained static repeat at the same gains and caps;
it does not yet qualify another support reduction or a walking command.
Evidence: `reports/native_protected_policy_preflight_20260923_113227.json`,
`reports/native_policy_ipc_trace_20260923_113227.npz`,
`reports/native_protected_policy_admission_20260923_113227.json`, and
`reports/native_protected_policy_trace_20260923_113227.npz`.

The explicitly approved 20-second repeat at the same lower-support height and
the same 0.08 gain and 2.2/2.5 Nm non-hip leg tier also passed. Its repeated
immediate preflight estimated at most 1.993 Nm. Active control had zero deadline
misses, zero target clamp, no ankle clamp-watchdog event, no rejected IMU frame,
and all 31 motors were verified disabled. Mean tilt over successive five-second
windows was 3.145, 3.138, 3.136, and 3.138 degrees; the last window remained
within 3.124 to 3.154 degrees. Maximum measured leg feedback was 1.361 Nm at
right hip pitch, 1.224 Nm at left hip pitch, 0.882 Nm at right knee, and 0.305
Nm among the ankle motors. Waist roll measured a transient 1.612 Nm while its
maximum policy command was only 0.149 Nm, but this was below its independent
3.0 Nm feedback gate and had no corresponding attitude drift. Treat that as a
contact/harness-load observation to monitor rather than a reason to change a
gain or limit. This qualifies sustained static contact at this support height;
the next support reduction remains a separately gated physical change.
Evidence: `reports/native_protected_policy_preflight_20260923_114033.json`,
`reports/native_policy_ipc_trace_20260923_114033.npz`,
`reports/native_protected_policy_admission_20260923_114033.json`, and
`reports/native_protected_policy_trace_20260923_114033.npz`.

After the harness was lowered again, a zero-gain shadow showed that the new pose
requires an explicit recovery stage rather than another static-hold tier. Body
roll was about -4.25 degrees. Measured right knee and ankle pitch were -0.571
and +0.276 rad, while the policy requested right ankle pitch near -0.426 rad.
The raw target exceeded the unchanged -0.386 rad soft boundary by at most
0.0583 rad and was safely projected to that boundary; the preview estimated
2.405 Nm at the right knee. No motor was enabled. The recovery tier therefore
retains gain 0.08 and the one-second hold plus four-second smooth ramp, permits
3.0 Nm command and 3.5 Nm feedback only for knee and hip yaw, retains 4.5/5.0
Nm for hip pitch/roll, and retains 2.2/2.5 Nm for the ankle motors. Target
projection remains active. The first explicitly approved recovery attempt did
not enable any motor: its repeated preflight found that the unloaded pose had
sagged further, increasing the stable right-ankle raw-target overshoot from
0.0583 to about 0.125 rad. Two zero-gain samples reproduced that value while
the right-knee preview remained about 2.35 Nm. The recovery-only preflight
ceiling is therefore 0.15 rad, while projection continues to prevent that raw
target from reaching hardware. The ankle clamp watchdog records but does not
trip during the first 350 ticks (seven seconds), leaving the final second under
the original 0.05 rad/five-tick gate. If the policy has not moved out of the
infeasible-target regime by then, control disables automatically. The
horizontal-gravity ceiling remains 0.10. This revised active recovery requires
a new explicit approval. Evidence:
`reports/native_policy_ipc_transport_20260923_114557.json`,
`reports/native_policy_ipc_trace_20260923_114557.npz`,
`reports/native_recovery_preflight_review_20260923_114557.json`,
`reports/native_policy_ipc_trace_20260923_115919.npz`, and
`reports/native_policy_ipc_trace_20260923_120058.npz`.

## First milestone

In a lifting frame, Sprite0825 starts from stand, walks at 0.15 m/s, stops, and
restarts while every motor remains inside torque-speed-current-temperature
limits and the watchdog and physical emergency stop are demonstrated.
