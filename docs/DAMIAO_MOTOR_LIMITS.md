# Damiao motor limits used by Sprite0825

This document separates three different limit classes that must not be
interchanged:

1. Manufacturer mechanical/electrical specifications from the motor manuals.
2. Deployment limits for the approximately 38 V Sprite0825 DC bus.
3. PMAX/VMAX/TMAX protocol mapping ranges read back from each motor.

The protocol mapping range controls encoding and feedback scaling. It is not a
mechanical rating and must never replace the rated or peak motor limit.

## Manufacturer specifications

| Installed model | Rated torque | Peak torque | Rated speed | Maximum no-load speed | Rated phase/supply current | Peak phase/supply current |
|---|---:|---:|---:|---:|---:|---:|
| DM-J4340P-2EC V1.1 (48V) | 14 Nm | 40 Nm | 36 rpm / 3.770 rad/s | 112 rpm / 11.729 rad/s at 48 V | 4.11 / 1.8 A | 19.85 / 11.2 A |
| DM-J4310P-2EC (48V) | 3.5 Nm | 12.5 Nm | 120 rpm / 12.566 rad/s | 450 rpm / 47.124 rad/s at 48 V | 4.8 / 1.6 A | 20.0 / 12.8 A |
| DM-J3507-2EC (48V) | 0.8 Nm | 3 Nm | 150 rpm / 15.708 rad/s | 910 rpm / 95.295 rad/s at 48 V | 3.0 / 0.6 A | 8.3 / 2.0 A |
| DM-J6248P-2EC | 30 Nm | 97 Nm | 40 rpm / 4.189 rad/s | 60 rpm / 6.283 rad/s | 11.3 / 3.7 A at 48 V | 38.7 / 8.0 A at 48 V |

All four manuals specify drive shutdown at 120 C and a configurable motor
winding limit with 100 C recommended. These are manufacturer protection values,
not yet the lower commissioning warning and stop thresholds.

Sources:

- `DM-J4340P-2EC V1.1 User Manual V1.1 2026-04-09`
- `DM-J4310P-2EC User Manual V1.1`
- `DM-J3507-2EC Gear Motor User Manual V1.1 2026-04-16`
- `DM-J6248P-2EC Gear Motor User Manual V1.1 2026-04-13`

The source PDFs are not redistributed by this repository. Their transcribed
values are frozen in `hardware.DAMIAO_MODEL_SPECS` and duplicated in the
hardware contract so changes are reviewable.

## Dynamic envelope status

The current per-motor `max_speed_rad_s` values for J4340P and J4310P are the
existing conservative 38 V deployment limits, not the 48 V no-load values in
the manuals. The MIT `velocity_rad_s` and `torque_nm` ranges are register
readbacks used for protocol encoding.

A final torque-speed guard must not infer a curve merely by drawing a line from
peak torque to no-load speed. The manual curves have different stated test
conditions, mostly at 24 V, while Sprite0825 operates near 38 V. Until a reviewed
38 V curve or a conservative commissioning envelope is approved, the native
writer must enforce position, protocol velocity, embedded Kd, feedforward
torque, estimated peak torque, motor status, and temperature limits, but keep
dynamic torque-speed qualification explicitly incomplete.
