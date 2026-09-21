# Sprite0825 first powered hold

The first closed-loop hardware test is intentionally restricted to the independent
`head_yaw_motor` (`kcan3`, command ID `0x08`, feedback ID `0x18`, DM-J3507).
It does not run the policy and cannot select another motor or raise its gains from
the command line.

The DM-J3507 V1.1 manual warns that position control must start from the measured
initial position to avoid impact. The tool therefore reads disabled feedback,
uses that exact position as the target, preloads a zero-gain position frame, and
only then sends the documented enable frame. The frozen test is:

- 2.0 seconds at 50 Hz;
- `Kp=0.2`, `Kd=0.05`, velocity target zero, feedforward torque zero;
- position error at most 0.05 rad;
- measured speed at most 0.2 rad/s;
- estimated motor torque at most 0.1 Nm, versus 0.8 Nm rated;
- MOS below 100 C and rotor below 80 C.

Any timeout, status change, limit violation, or exception enters `finally`, sends
the documented disable frame three times, and verifies disabled feedback. This is
not a mode switch or zero-setting operation.

## Physical prerequisites

1. Robot remains supported in the lifting frame.
2. Head yaw is mechanically free and clear of cables and hard stops.
3. A safety operator holds the independent power cut-off.
4. `kcan3` is ERROR-ACTIVE at 1 Mbit/s arbitration and 5 Mbit/s data rate.
5. Every other motor remains disabled.

Run only after reading the printed command summary:

```bash
cd /home/tony/open_sprite_runtime
./hold_sprite0825_head_yaw_low_gain_on_253.sh ENABLE_HEAD_YAW_LOW_GAIN_HOLD
```

The JSON report is written under `reports/head_yaw_low_gain_hold_*.json`.
Passing this hold does not authorize motion commands. The separately gated second
test uses the same one-motor allowlist and runs a frozen quintic trajectory:

- measured position to `+0.02 rad` in 1.0 second, then dwell 0.5 second;
- `+0.02 rad` to `-0.02 rad` in 1.0 second, then dwell 0.5 second;
- return to the measured start in 1.0 second, then dwell 0.5 second;
- `Kp=1.0`, `Kd=0.2`, feedforward torque zero, at 50 Hz;
- the same 0.05 rad error, 0.2 rad/s speed, and 0.1 Nm torque guards;
- verified disable on every return path.

Run it only with the bare head-yaw shaft clear and the safety operator ready:

```bash
cd /home/tony/open_sprite_runtime
./move_sprite0825_head_yaw_low_gain_on_253.sh ENABLE_HEAD_YAW_LOW_GAIN_MOTION
```

The motion report is written under `reports/head_yaw_low_gain_motion_*.json`.

The first physical run passed on 2026-09-21. It completed 225 command and 225
feedback cycles, observed at most 0.0611 rad/s and 0.0306 Nm, and verified the
motor was disabled after the trajectory. The preserved report is
`reports/head_yaw_low_gain_motion_20260921_192055.json` on the Jetson.

## Visible ten-degree test

The next frozen gate commands the unloaded shaft from its measured start to
relative `+10 degrees`, then `-10 degrees`, and back to the start using quintic
four-second transitions and one-second dwells. It runs at 50 Hz with `Kp=2.0`,
`Kd=0.2`, zero feedforward, and 0.08 rad/0.8 rad/s/0.25 Nm guards:

```bash
cd /home/tony/open_sprite_runtime
./move_sprite0825_head_yaw_visible_10deg_on_253.sh \
  ENABLE_HEAD_YAW_VISIBLE_10DEG_MOTION
```

The final 2026-09-21 physical run completed all 750 command/feedback cycles and
verified disabled feedback. Low-gain measured travel was approximately
`+8.59/-8.79 degrees`; maximum measured speed was 0.526 rad/s and maximum
estimated torque was 0.080 Nm. The preserved report is
`reports/head_yaw_visible_10deg_20260921_192909.json` on the Jetson.
