# Sim2Real plan

## Gate 0: frozen software contract

- Pin `model1050` as the qualified 100 Hz teacher/baseline.
- Train and pin a native 50 Hz deployment actor with approximately 160 ms history.
- Reproduce deployment observations from recorded MuJoCo traces.
- No horizontal base velocity in actor observations.

## Gate 1: hardware inventory

- Complete all 31 motor mappings, limits, and MIT ranges.
- Measure four-channel USB-CAN FD latency and sustained bus utilization.
- Establish a hardware emergency-stop chain independent of the SBC process.

## Gate 2: ankle calibration

- Identify left/right position matrices, signs, and motor zeros with legs unloaded.
- Verify position and velocity round trips.
- Verify power-consistent torque mapping at low commanded torque.

## Gate 3: shadow mode

- Read motors and IMU; never transmit policy commands.
- Log raw frames, normalized observations, actor output, proposed motor targets,
  safety margins, loop jitter, and state age.
- Replay every log deterministically offline.

## Gate 4: protected actuation

- Single-joint unloaded tests, then whole-body zero pose in a lifting frame.
- Low Kp/Kd and strict current limits first.
- Stand, weight shift, one step, 0.15 m/s walk, stop, and restart.

## First milestone

In a lifting frame, Sprite0825 starts from stand, walks at 0.15 m/s, stops, and
restarts while every motor remains inside torque-speed-current-temperature
limits and the watchdog and physical emergency stop are demonstrated.
