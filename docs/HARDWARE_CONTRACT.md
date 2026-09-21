# Hardware contract

Before protected actuation, fill every physical motor entry in `motor_map` with:

- Damiao model and firmware version
- USB-CAN FD channel, command CAN ID, and feedback Master ID
- mechanical zero and encoder zero
- raw encoder sign; for direct joints, positive policy direction versus positive motor direction
- reduction ratio and any linkage ratio
- soft limit and independently measured hard limit
- rated/peak torque, speed, current, and temperature limits
- MIT protocol ranges and quantization for position, velocity, Kp, Kd, torque;
  PMAX/VMAX/TMAX must be captured from each drive's register readback

At every 500 Hz state tick, the runtime must reject missing/extra motor frames,
non-finite values, hard-position-limit violations, excessive speed, peak torque,
peak current, temperature, or torque-speed-envelope violations. Any such event
locks `motor_limit` and requires a separate healthy check plus explicit clear.
Until a measured `torque_speed_envelope` is supplied for a motor, the runtime
uses a conservative nameplate polyline from peak torque at zero speed, through
rated torque at rated speed, to zero torque at maximum speed. This is a safety
bound, not a motor thermal model; sustained-load qualification remains required.

For the KH SocketCAN driver, the raw hardware timestamp is in the adapter's
device clock domain. Retain it for bus-event intervals and jitter, but never
subtract it directly from host time. The p99 receive-age gate uses the kernel
software RX timestamp through userspace receipt and is specifically a host
queue/scheduling bound. USB bus-to-kernel latency requires a separate
synchronized-clock or physical-loopback measurement.

Generate the measurement worksheet from the frozen policy order instead of
typing joint names manually:

```bash
sprite-runtime hardware-template \
  --contract <g74-model3000-candidate>/deploy/contract.json \
  --output config/hardware.sprite0825.local.json
```

The generated file is deliberately non-armable. It pre-fills the known leg and
all J4340P/J4310P motor nameplate values plus the physical ankle and head
differential topology; every machine-specific
CAN endpoint, firmware version, zero, sign, limit, current/temperature bound,
MIT range, IMU transform, e-stop description, and measured ankle matrix remains
unset until measured.

MIT command profiles are built only from a complete `configured=true` hardware
inventory. They use the physical motor map rather than policy-joint index order,
which is essential for every two-motor differential. Each command is checked
against motor soft position limits, motor-register-readback PMAX/VMAX/TMAX, and
the current-speed torque envelope. The complete impedance request
`kp*(q_des-q)+kd*(dq_des-dq)+tau_ff` must pass, not only `tau_ff`.

Both `motor_telemetry_healthy` and `command_envelope_healthy` default false in the
runtime safety input. Either violation creates a latched fault, so omission of a
health signal cannot silently authorize transmission.

The runtime must reorder feedback into the exact 31-joint policy order from the
qualified contract. It must never use CAN enumeration order as policy order.
Feedback identity is the pair `(SocketCAN interface, Master ID)` and D0's
controller-ID nibble must agree with the configured command CAN ID.

See `SPRITE0825_DAMIAO_PROTOCOL_SOURCE_AUDIT.md` for the pinned official
protocol sources, exact receive-frame layout, status values, and the reason the
decoded torque is an estimate rather than an independent torque measurement.

Run `sprite-runtime inspect` after filling the hardware file. The
`hardware_inventory` report is fail-closed: it requires 25 one-to-one joint
motors plus two coupled motors for each left ankle, right ankle, and head
pitch/roll differential, exactly 31 physical
motors in total, unique `(can_channel, can_id)` endpoints, complete MIT ranges,
finite zeros, nested soft/hard limits, a measured IMU configuration, an
independent e-stop chain, and three separate differential calibrations. A passing
inventory check validates configuration consistency only; it does not enable
CAN transmission.

The CAN commissioning evidence is recorded under
`can_adapter.commissioning_shadow`. For the installed Damiao drives this must be
`active_zero_gain_position_echo`, because a disabled drive returns its state in
response to a request rather than broadcasting the complete required stream.
The gate requires at least 120 seconds, all motors disabled, at least 99% sample
coverage for every motor, and zero deadline misses, CAN errors/drops, nonzero
gain/torque writes, automatic enables, or automatic mode switches. A pure
listen-only capture must not be presented as this evidence.

During suspended commissioning, a dedicated safety operator controls the main
motor-power switch and the robot remains mechanically supported. This is a
required operating procedure for the current tests, but it is not the
independent hardware emergency-stop chain required before walking tests.

Generate a concise, model-grouped list of remaining physical inputs without
opening CAN or serial devices:

```bash
PYTHONPATH=src .venv/bin/python tools/report_hardware_arm_gaps.py \
  --hardware config/hardware.sprite0825.measurement.json \
  --contract <g74-model3000-candidate>/deploy/contract.json \
  --json-output reports/sprite0825_arm_gaps.json \
  --markdown-output reports/sprite0825_arm_gaps.md
```

## Frozen CAN FD assignment

Runtime channels are zero-based: channel 0 through 3 correspond to physical
`CANFD1` through `CANFD4`. Linux SocketCAN interface names are configured
separately in `can_adapter.interfaces`; their order must follow this logical
channel order. Every Damiao `master_id` is frozen as `can_id + 0x10`.

| Bus | Channel | ID 1-8 assignment |
|---|---:|---|
| CANFD1 | 0 | left hip pitch, left hip roll, left hip yaw, left knee, left ankle motor A, left ankle motor B, waist yaw, waist roll |
| CANFD2 | 1 | right hip pitch, right hip roll, right hip yaw, right knee, right ankle motor A, right ankle motor B, head motor A, head motor B |
| CANFD3 | 2 | left shoulder pitch, left shoulder roll, left shoulder yaw, left elbow, left wrist yaw, left wrist pitch, left wrist roll, head yaw |
| CANFD4 | 3 | right shoulder pitch, right shoulder roll, right shoulder yaw, right elbow, right wrist yaw, right wrist pitch, right wrist roll; ID 8 unused |

## Frozen installed motor models

The model variant is part of the endpoint identity and is validated together
with the bus and CAN ID. `V1.1 (48V)` is retained where it was explicitly
confirmed on the installed hardware; it must not be shortened to a generic SDK
enum in the physical inventory.

| Bus | ID 1-8 motor models |
|---|---|
| CANFD1 | J4340P V1.1 48V, J4340P V1.1 48V, J4340P V1.1 48V, J4340P V1.1 48V, J4310P 48V, J4310P 48V, J4340P V1.1 48V, J6248P |
| CANFD2 | J4340P V1.1 48V, J4340P V1.1 48V, J4340P V1.1 48V, J4340P V1.1 48V, J4310P 48V, J4310P 48V, J3507 48V, J3507 48V |
| CANFD3 | J4340P V1.1 48V, J4340P V1.1 48V, J4310P 48V, J4310P 48V, J4310P 48V, J3507 48V, J3507 48V, J3507 48V |
| CANFD4 | J4340P V1.1 48V, J4340P V1.1 48V, J4310P 48V, J4310P 48V, J4310P 48V, J3507 48V, J3507 48V; ID 8 unused |

This table identifies hardware only. MIT `PMAX/VMAX/TMAX` still comes from
per-drive register readback, while rated/peak torque, current, speed, and
temperature limits come from the matching nameplate/manual plus qualification.
The protocol can encode `Kd` through 5, but Sprite0825 deliberately qualifies
only `0 <= Kd <= 3` for every Damiao motor. The hardware inventory records this
as `controller.damiao_embedded_kd_max = 3.0`, and command profiles reject larger
values before frame encoding. This is an engineering limit based on bench
behavior, not a change to the Damiao wire format.

Training/deployment contracts retain the simulation stiffness and damping that
produced a policy. Those values are provenance and must not be silently clipped
into hardware commands. A hardware-realizable gain profile must be selected and
requalified in MuJoCo/Isaac; until then, a simulation damping such as `8.042`
causes a fail-closed preflight error rather than CAN transmission.

In runtime naming, hardware motor `1`/`2` for each differential corresponds to
motor `a`/`b`. The inventory validator rejects changes to any confirmed channel,
command ID, or master ID.

For direct joints, record `encoder_sign` and `policy_to_motor_sign` separately.
For each coupled motor, record only `encoder_sign`; the relationship from
the two calibrated motor coordinates to pitch/roll belongs in that mechanism's
measured 2x2 `joint_to_motor_matrix`. Assigning a scalar policy sign from one
coupled motor to one policy joint would be physically incorrect.

The five direct-joint sign discrepancies confirmed against the Sprite0825 v5
policy asset are frozen as `policy_to_motor_sign = -1` for `waist_yaw_joint`,
`left_wrist_pitch_joint`, `left_wrist_roll_joint`, `right_wrist_roll_joint`, and
`head_yaw_joint`. The same signed ratio is used for outgoing position, velocity,
and torque commands and for incoming feedback. Damiao firmware direction is not
changed.

The coordinate layers are fixed as follows. `q_drive` is the raw coordinate used
by the Damiao MIT frame, while `q_motor` is the signed physical motor coordinate:

```text
q_motor = encoder_sign * (q_drive - motor_zero_rad)
```

For a direct joint:

```text
q_motor = policy_to_motor_sign * reduction_ratio * linkage_ratio * q_joint
```

For any differential, `differentials.<name>.motor_names` gives the exact row
order of the calibrated matrix and must match that mechanism's two coupled motor
records. The configured names are `left_ankle`, `right_ankle`, and `head`:

```text
q_motor = joint_to_motor_matrix * [q_pitch, q_roll] + ankle_motor_zero
```

The head topology is frozen as a differential over
`[head_pitch_joint, head_roll_joint]`; `head_yaw_joint` is an independent direct
joint on CANFD3 ID 8. Do not include yaw in the head differential matrix.

Velocity uses the same linear map without offsets. Torque uses the inverse
transpose so instantaneous power is preserved. These transforms are tested in
both directions over the complete 31-joint/31-motor map.

For the ankle differential pairs, the 500 Hz host layer computes complete
joint-space PD torque from fresh state and maps it through `A^-T`. The ankle
motors receive `Kp=Kd=0` plus feed-forward motor torque, so the embedded-Damiao
`Kd <= 3` limit does not alter ankle impedance. The head differential remains
non-actuating until its explicit control path is qualified.

The inventory validator also pins the known J4340P nameplate data: hip/knee and
the four shoulder pitch/roll motors are DM-J4340P-2EC with 14/40 Nm rated/peak
torque, about 3.8 rad/s rated speed, and the frozen 9.3 rad/s operating maximum
at approximately 38 V. Raising this limit toward 10 rad/s requires a new
motor-envelope qualification. The four ankle motors
are DM-J4310P-2EC with 3.5/12.5 Nm rated/peak torque and 12.56 rad/s rated speed.
The 48 V nameplate no-load speed is 47.1 rad/s, but the qualified approximately
38 V deployment contract uses 36.2 rad/s as its operating no-load safety bound.
Record `controller.nominal_bus_voltage_v`; this candidate rejects values outside
36-40 V. Current and temperature limits remain hardware/firmware measurements
and must be entered before qualification.

The confirmed nameplate torque values for the remaining installed models are
0.8/3.0 Nm rated/peak for DM-J3507-2EC and 30/97 Nm rated/peak for
DM-J6248P-2EC. These mechanical ratings remain distinct from each drive's MIT
protocol `TMAX`, which must come from register readback.

Generate review-only joint and motor limit candidates from the exact URDF pinned
by the policy contract:

```bash
PYTHONPATH=src .venv/bin/python tools/derive_urdf_motor_limits.py \
  --hardware config/hardware.sprite0825.measurement.json \
  --contract <g74-model3000-candidate>/deploy/contract.json \
  --urdf <g74-model3000-candidate>/<contract-asset-urdf> \
  --soft-margin-rad 0.05 \
  --output reports/sprite0825_urdf_limit_candidates.json
```

The tool verifies the URDF SHA256 before deriving anything. For direct joints it
applies sign, zero, and ratio. For each ankle and the head pitch/roll
differential it maps all four corners of the two-joint limit rectangle through
the calibrated matrix. The resulting motor boxes are secondary guards only:
the runtime must first enforce the pitch/roll joint-space limits. The 0.05 rad
inset and all URDF limits remain candidates until the physical hard stops are
measured and signed off.

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

## Differential ankles and head

The ideal linear map is:

```text
[q_motor_a]   [1  1] [q_pitch]
[q_motor_b] = [1 -1] [q_roll ] + motor_zero
```

The real left ankle, right ankle, and head matrices must be verified or measured
separately. The torque map is
the inverse transpose, not the position map:

```text
tau_motor = A^-T tau_joint
```

This preserves instantaneous mechanical power. The active ankle design does
not approximate the coupled impedance with embedded diagonal gains: the 500 Hz
host controller computes the complete pitch/roll PD torque, applies `A^-T`, and
sends the resulting motor torques with MIT `Kp=Kd=0`. The head uses the same
calibrated position/feedback transform but remains non-actuating until a head
control path is separately qualified.

Collect at least six unloaded poses per mechanism spanning independent positive
and negative pitch and roll. Record a CSV with
`pitch_rad,roll_rad,motor_a_rad,motor_b_rad`, then fit each side separately:

```bash
sprite-runtime differential-calibrate --pair left_ankle \
  --samples calibration/left_ankle.csv --output calibration/left_ankle_fit.json
```

For the head use the same tool with `--pair head`. The legacy
`sprite-runtime ankle-calibrate --side left|right` command remains available.

The fitter reports the 2x2 matrix, both motor zeros, excitation and matrix
condition numbers, and per-motor residuals. Its default gate requires RMS
residual at most 0.01 rad and both condition numbers at most 100. A failed fit
must not be copied into the hardware contract.
