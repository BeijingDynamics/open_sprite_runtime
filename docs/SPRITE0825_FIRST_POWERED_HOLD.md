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
Passing this hold does not authorize motion commands; the next test would be a
separately reviewed smooth `+/-0.02 rad` head-yaw excursion.
