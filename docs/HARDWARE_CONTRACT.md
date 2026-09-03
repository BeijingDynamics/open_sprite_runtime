# Hardware contract

Before shadow mode, fill every policy joint entry in `motor_map` with:

- Damiao model and firmware version
- USB-CAN FD channel and CAN ID
- mechanical zero and encoder zero
- positive policy direction versus positive motor direction
- reduction ratio and any linkage ratio
- soft limit and independently measured hard limit
- rated/peak torque, speed, current, and temperature limits
- MIT protocol ranges and quantization for position, velocity, Kp, Kd, torque

The runtime must reorder feedback into the exact 31-joint policy order from the
qualified contract. It must never use CAN enumeration order as policy order.

## IMU and heading

The minimum deployment configuration uses one timestamped IMU rigidly mounted
to the pelvis/root assembly. Record its exact body-to-sensor rotation, axis
signs, quaternion order, gyro units, update rate, and timestamp source in the
hardware contract. Validate projected gravity and local angular velocity
against known robot orientations before enabling actuation.

Integrate the measured yaw rate outside the actor. While standing, reset the
heading reference to the current integrated yaw. While walking with zero manual
yaw input, use the same PM01-style command law as the qualified Isaac test:

```text
wz_command = clip(0.5 * wrap(target_yaw - imu_yaw), -0.2, 0.2)
```

During an operator turn, pass through the bounded requested yaw rate and follow
the measured yaw with the hold target. On release, hold the new heading. The
actor receives only `[vx, vy, wz_command]`; it never receives integrated/global
yaw, horizontal base velocity, or global position.

A second torso IMU may be logged later for structural-flex diagnostics, but it
must not alter the frozen actor observation. Invalid, stale, discontinuous, or
non-finite IMU data blocks hardware transmission through the safety supervisor.

## Differential ankles

The ideal linear map is:

```text
[q_motor_a]   [1  1] [q_pitch]
[q_motor_b] = [1 -1] [q_roll ] + motor_zero
```

The real left and right matrices must be measured separately. The torque map is
the inverse transpose, not the position map:

```text
tau_motor = A^-T tau_joint
```

This preserves instantaneous mechanical power. For the ideal matrix and equal
pitch/roll gains, embedded motor gains are half the joint gains. With unequal
joint gains or a non-ideal measured Jacobian, the exact motor impedance contains
cross terms. Independent motor PD alone then cannot reproduce the trained joint
impedance; the 500 Hz layer must supply the cross-coupled correction through
MIT feed-forward torque, or the policy must be requalified with the realizable
impedance.
