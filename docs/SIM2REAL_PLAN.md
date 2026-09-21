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
- Measure four-channel USB-CAN FD latency and sustained bus utilization.
- [x] Establish a hardware emergency-stop chain independent of the SBC process:
  a safety operator controls the regulated supply's independent motor-bus switch.
- Record the power-cut behavior and stop time before releasing ground walking.
- Measure and record pelvis IMU mounting transform, axes, timestamps, update
  rate, gyro bias, and short-term integrated-yaw drift.
- [x] Record a 120-second read-only stationary IMU audit at 99.94 Hz with zero
  rejected frames and -0.0101 deg/min measured yaw drift.
- Install and verify the topology-pinned `/dev/sprite0825-imu` udev link.

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
  120 seconds under full CAN and IMU load.
- [x] Implement and qualify the frozen two-tick measured-pose-equivalent-action
  smoothstep handoff in the live policy path, matching the MuJoCo deployment.
- [x] Fail closed on invalid target ABI/order, future source sequence, stale
  timestamp, embedded `Kd > 3`, ordinary target timeout, and missing first
  target; verify all injected faults against the native process.
- [ ] Qualify complete command envelopes before any nonzero motor command is
  permitted.

## Gate 4: protected actuation

- Single-joint unloaded tests, then whole-body zero pose in a lifting frame.
- Low Kp/Kd and strict current limits first.
- Stand, weight shift, one step, 0.15 m/s walk, stop, and restart.

## First milestone

In a lifting frame, Sprite0825 starts from stand, walks at 0.15 m/s, stops, and
restarts while every motor remains inside torque-speed-current-temperature
limits and the watchdog and physical emergency stop are demonstrated.
