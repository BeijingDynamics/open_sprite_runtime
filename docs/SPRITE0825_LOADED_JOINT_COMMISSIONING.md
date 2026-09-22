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
