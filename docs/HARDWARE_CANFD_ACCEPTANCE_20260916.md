# Sprite0825 CAN-FD Hardware Acceptance - 2026-09-16

## Result

All four CAN-FD buses and all connected Sprite0825 motors passed the physical
feedback/direction audit. Each complete bus group also passed the 500 Hz per
motor zero-gain position-echo stress test.

The active probe payload was always:

- position command equal to the latest reported position
- velocity command = 0
- Kp = 0
- Kd = 0
- feedforward torque = 0
- CAN-FD BRS enabled, 1 Mbps arbitration and 5 Mbps data phase

The test tools did not automatically enable/disable motors or switch modes.

## Bus acceptance

| Interface | Physical group | Motor count | Direction and limits | 500 Hz stress |
|---|---|---:|---|---|
| `kcan1` | Left leg + waist | 8 | Passed | Passed |
| `kcan2` | Right leg + head pitch/roll differential | 8 | Passed | Passed |
| `kcan3` | Left arm + head yaw | 8 | Passed | Passed |
| `kcan4` | Right arm | 7 | Passed | Passed |

## Differential mechanisms

- Left ankle pitch/roll: physically checked in MuJoCo; final calibrated mapping accepted.
- Right ankle pitch/roll: physically checked in MuJoCo; pitch convention corrected on
  2026-09-16 and final calibrated mapping accepted.
- Head pitch/roll: physically checked in MuJoCo; final calibrated mapping accepted.

Do not change individual motor `encoder_sign` to compensate for differential joint
sign errors. Differential conventions belong in the calibrated
`joint_to_motor_matrix`. Direct-joint command/feedback sign corrections belong in
`policy_to_motor_sign` and must be applied symmetrically to commands and feedback.

## Accepted runtime configuration

The accepted hardware configuration is:

`config/hardware.sprite0825.measurement.json`

At acceptance time, its SHA-256 was:

`ee193b3f1fc7ceb28e2bd267290b055cb81f38550f034614ceae36fc8e860a43`

The file was verified byte-identical on Jetson 253 and workstation 234.

## Scope

This acceptance establishes CAN transport, motor identity, feedback decoding,
joint direction, mechanical-limit correspondence, and differential kinematics.
It does not authorize powered whole-body policy execution. The next hardware gate
must separately validate IMU orientation, safety state machine, command clamps,
watchdog/estop behavior, low-gain supported pose control, and only then staged
policy activation.
