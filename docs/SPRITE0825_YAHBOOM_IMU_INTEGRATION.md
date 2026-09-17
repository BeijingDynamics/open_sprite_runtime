# Sprite0825 Yahboom 9-axis IMU integration

## Fixed physical mount

The IMU is mounted on the rear of the pelvis. Sprite body axes are +X forward,
+Y left, and +Z up. The installed sensor axes are +X down, +Y right, and +Z
backward. Sensor vectors are converted to the pelvis/root-link frame by:

```text
[x_body, y_body, z_body] = [-z_sensor, -y_sensor, -x_sensor]

R_body_from_sensor =
  0  0 -1
  0 -1  0
 -1  0  0
```

The corresponding body-to-sensor quaternion in `wxyz` order is
`[0, 0.7071067811865476, 0, -0.7071067811865476]`.

The policy receives body-frame angular velocity and projected gravity. It does
not receive horizontal base velocity or global yaw. Heading is maintained
outside the actor by integrating body gyro Z and resetting the heading target
while standing. Magnetometer yaw is monitoring evidence, not the sole safety
or heading source near motors and power wiring.

## Jetson preparation

The Jetson already has the high-performance CH340 driver. Do not install,
replace, blacklist, or reconfigure a serial kernel driver for this work.

Before the sensor arrives, validate the fixed transform:

```bash
cd /home/tony/open_sprite_runtime
PYTHONPATH=src python3 -m open_sprite_runtime.cli imu-mount-self-test
```

## Hardware arrival procedure

1. Keep motor power disabled. Connect only the IMU and list stable device names:
   `ls -l /dev/serial/by-id/`.
2. Confirm the exact serial baud and packet format from the supplied Yahboom
   example or `YbImuLib`. Do not guess either value.
3. Capture raw bytes without transmitting protocol data:

```bash
PYTHONPATH=src python3 -m open_sprite_runtime.cli imu-serial-capture \
  --device /dev/serial/by-id/<device> \
  --baud <documented-baud> \
  --duration 10 \
  --raw-output reports/yahboom_imu_raw.bin \
  --report reports/yahboom_imu_raw.json
```

4. Add a parser adapter against captured bytes and verify the vendor's units,
   quaternion order, and orientation convention with six static face tests.
5. Measure output rate, sample gaps, host receive age, stationary gyro bias,
   and yaw drift. Only then fill the remaining IMU fields and set
   `imu.configured` true.
6. Re-run the fail-closed hardware inventory and policy observation tests before
   any motor-enable experiment.

The raw serial command never calls `write()`. Opening a USB UART may change DTR
or RTS, so both are held inactive. The current code deliberately contains no
guessed Yahboom packet decoder.
