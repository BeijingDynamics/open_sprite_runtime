# Sprite0825 full-body shadow commissioning

This is the first integrated wiring test for all 31 motors, four CAN-FD buses,
the pelvis IMU, and the MuJoCo kinematic display. It is not a policy-control or
motor-enable command.

## Safety boundary

- Mechanically support the complete robot.
- Confirm every drive is already in MIT mode and reports `disabled`.
- The command never enables, disables, or changes a drive mode.
- Each transmitted MIT frame echoes the latest measured position with velocity,
  `Kp`, `Kd`, and feedforward torque all zero.
- The first run uses 50 Hz per motor. Do not substitute the 500 Hz per-bus stress
  rate for the first integrated wiring test.
- Close the MuJoCo window to stop early.

## Run on Jetson 253

Configure all four KH SocketCAN interfaces after connecting or rebooting the
adapter:

```bash
cd /home/tony
./setup_sprite0825_kcanfd_on_253.sh
```

The setup script first requires all four interfaces to exist, then configures
1 Mbps arbitration, 5 Mbps data, CAN FD, and a 1000-frame transmit queue.

The first argument is duration in seconds and the second is the per-motor polling
rate. Start with ten seconds:

```bash
cd /home/tony
./probe_sprite0825_full_body_shadow_mujoco_on_253.sh 10 50
```

After the ten-second run passes and the displayed joint directions are correct,
run 120 seconds:

```bash
cd /home/tony
./probe_sprite0825_full_body_shadow_mujoco_on_253.sh 120 50
```

Reports are written under `/home/tony/open_sprite_runtime/reports` as
`full_body_shadow_<timestamp>.json`.

## Automatic gates

The report passes only when:

- all 31 configured motors return CAN-FD+BRS feedback;
- every motor remains in status `disabled`;
- every motor reaches at least 90 percent sample coverage;
- both raw IMU and quaternion streams reach at least 80 Hz;
- no IMU frame is rejected;
- no motor or IMU stream is stale for more than 200 ms;
- serial writes, automatic enable attempts, and automatic mode-switch attempts
  remain zero.

The MuJoCo root attitude is driven from the absolute pelvis IMU mounting transform.
The three measured differential maps convert left ankle, right ankle, and head
motor feedback into their logical pitch/roll joints.
