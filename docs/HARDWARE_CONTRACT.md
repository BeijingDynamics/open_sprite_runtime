# Hardware contract

Before shadow mode, fill every policy joint entry in `motor_map` with:

- Damiao model and firmware version
- USB-CAN FD channel and CAN ID
- mechanical zero and encoder zero
- raw encoder sign; for direct joints, positive policy direction versus positive motor direction
- reduction ratio and any linkage ratio
- soft limit and independently measured hard limit
- rated/peak torque, speed, current, and temperature limits
- MIT protocol ranges and quantization for position, velocity, Kp, Kd, torque

At every 500 Hz state tick, the runtime must reject missing/extra motor frames,
non-finite values, hard-position-limit violations, excessive speed, peak torque,
peak current, temperature, or torque-speed-envelope violations. Any such event
locks `motor_limit` and requires a separate healthy check plus explicit clear.
Until a measured `torque_speed_envelope` is supplied for a motor, the runtime
uses a conservative nameplate polyline from peak torque at zero speed, through
rated torque at rated speed, to zero torque at maximum speed. This is a safety
bound, not a motor thermal model; sustained-load qualification remains required.

Generate the measurement worksheet from the frozen policy order instead of
typing joint names manually:

```bash
open-sprite-runtime hardware-template \
  --contract artifacts/g60_model3450/deploy/contract.json \
  --output config/hardware.sprite0825.local.json
```

The generated file is deliberately non-armable. It pre-fills only the known
leg motor nameplate values and physical ankle topology; every machine-specific
CAN endpoint, firmware version, zero, sign, limit, current/temperature bound,
MIT range, IMU transform, e-stop description, and measured ankle matrix remains
unset until measured.

The runtime must reorder feedback into the exact 31-joint policy order from the
qualified contract. It must never use CAN enumeration order as policy order.

Run `open-sprite-runtime inspect` after filling the hardware file. The
`hardware_inventory` report is fail-closed: it requires 27 one-to-one joint
motors plus two coupled motors for each differential ankle, exactly 31 physical
motors in total, unique `(can_channel, can_id)` endpoints, complete MIT ranges,
finite zeros, nested soft/hard limits, a measured IMU configuration, an
independent e-stop chain, and separate left/right ankle calibrations. A passing
inventory check validates configuration consistency only; it does not enable
CAN transmission.

For direct joints, record `encoder_sign` and `policy_to_motor_sign` separately.
For each coupled ankle motor, record only `encoder_sign`; the relationship from
the two calibrated motor coordinates to pitch/roll belongs in that side's
measured 2x2 `joint_to_motor_matrix`. Assigning a single ankle motor sign to one
policy joint would be physically incorrect.

The inventory validator also pins the known leg nameplate data: hip/knee motors
are DM-J4340P-2EC with 14/40 Nm rated/peak torque, about 3.8 rad/s rated speed,
and the frozen 9.3 rad/s operating maximum at approximately 38 V. Raising this
limit toward 10 rad/s requires a new motor-envelope qualification. The four ankle motors
are DM-J4310P-2EC with 3.5/12.5 Nm rated/peak torque and 12.56 rad/s rated speed.
The 48 V nameplate no-load speed is 47.1 rad/s, but the qualified approximately
38 V deployment contract uses 36.2 rad/s as its operating no-load safety bound.
Record `controller.nominal_bus_voltage_v`; this candidate rejects values outside
36-40 V. Current and temperature limits remain hardware/firmware measurements
and must be entered before qualification.

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

Collect at least six unloaded poses per side spanning independent positive and
negative pitch and roll. Record a CSV with
`pitch_rad,roll_rad,motor_a_rad,motor_b_rad`, then fit each side separately:

```bash
open-sprite-runtime ankle-calibrate --side left \
  --samples calibration/left_ankle.csv --output calibration/left_ankle_fit.json
```

The fitter reports the 2x2 matrix, both motor zeros, excitation and matrix
condition numbers, and per-motor residuals. Its default gate requires RMS
residual at most 0.01 rad and both condition numbers at most 100. A failed fit
must not be copied into the hardware contract.
