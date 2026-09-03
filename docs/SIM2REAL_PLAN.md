# Sim2Real plan

## Gate 0: frozen software contract

- [x] Pin `model1050` as the qualified 100 Hz teacher/baseline.
- [x] Train and pin G60 `model3450`, a native 50 Hz deployment actor with 160 ms history.
- [x] Reproduce 299 sampled MuJoCo policy actions from recorded observations
  through the packaged ONNX actor with zero numerical difference.
- [x] Exclude horizontal base velocity, global position, and global yaw from actor observations.
- [x] Implement and unit-test the PM01-style integrated-IMU-yaw outer command
  controller without adding global yaw to the actor.
- [x] Qualify the outer heading controller under the same Isaac robust pushes
  and randomization used for the open-loop G60 stress matrix.

## Gate 1: hardware inventory

- Complete all 31 motor mappings, limits, and MIT ranges.
- Measure four-channel USB-CAN FD latency and sustained bus utilization.
- Establish a hardware emergency-stop chain independent of the SBC process.
- Measure and record pelvis IMU mounting transform, axes, timestamps, update
  rate, gyro bias, and short-term integrated-yaw drift.

## Gate 2: ankle calibration

- Identify left/right position matrices, signs, and motor zeros with legs unloaded.
- Verify position and velocity round trips.
- Verify power-consistent torque mapping at low commanded torque.

## Gate 3: shadow mode

- Read motors and IMU; never transmit policy commands.
- Log raw frames, normalized observations, actor output, proposed motor targets,
  safety margins, loop jitter, and state age.
- Replay every log deterministically offline.
- Demonstrate that stale state, stale command, policy overrun, and an open
  emergency-stop input all force a no-transmit safe hold. State, overrun, and
  emergency-stop faults remain latched until explicitly cleared after a
  separate healthy check.
- Measure 500 Hz loop jitter on the selected SBC under representative USB-CAN,
  logging, and inference load. A desktop timing probe is diagnostic only and
  cannot certify the Raspberry Pi 5 or Jetson Orin Nano.

The deterministic safety scenarios and a no-load 500 Hz probe pass on the 234
development PC. This does not complete Gate 3: live motor/IMU frames, USB-CAN
load, full logging, and the selected SBC are still required.

## Gate 4: protected actuation

- Single-joint unloaded tests, then whole-body zero pose in a lifting frame.
- Low Kp/Kd and strict current limits first.
- Stand, weight shift, one step, 0.15 m/s walk, stop, and restart.

## First milestone

In a lifting frame, Sprite0825 starts from stand, walks at 0.15 m/s, stops, and
restarts while every motor remains inside torque-speed-current-temperature
limits and the watchdog and physical emergency stop are demonstrated.
