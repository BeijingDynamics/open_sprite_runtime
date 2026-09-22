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

- Identify left/right position matrices, signs, and motor zeros with legs unloaded.
- Verify position and velocity round trips.
- Verify power-consistent torque mapping at low commanded torque.

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
- [ ] Build the next leg/waist gate around the calibrated differential ankle
  mapping and dedicated 500 Hz ankle layer. Do not reuse the direct arm-group
  hold for either ankle motor pair.
- [x] Prepare and offline-test fixed four-motor proximal-leg groups for each
  side. Their allowlists contain only hip pitch/roll/yaw and knee; ankle,
  waist, and head endpoints are excluded. Hardware execution remains pending
  renewed confirmation of the lifting frame, leg clearance, and safety operator.
- [ ] Treat every motor zero-position reset/write as a separately authorized
  maintenance operation. Never emit one without explicit owner approval for
  that exact operation.
- Single-joint tests, then fixed arm groups, then whole-body measured pose in a
  lifting frame.
- Low Kp/Kd and strict current limits first.
- Stand, weight shift, one step, 0.15 m/s walk, stop, and restart.

## First milestone

In a lifting frame, Sprite0825 starts from stand, walks at 0.15 m/s, stops, and
restarts while every motor remains inside torque-speed-current-temperature
limits and the watchdog and physical emergency stop are demonstrated.
