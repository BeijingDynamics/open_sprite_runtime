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
