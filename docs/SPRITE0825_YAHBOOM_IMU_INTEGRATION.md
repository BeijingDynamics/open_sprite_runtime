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

1. Keep motor power disabled. Connect only the IMU. The installed high-performance
   CH341 driver exposes `/dev/ttyCH341USB0`. This unit has no unique USB serial
   number and therefore no `/dev/serial/by-id` entry; the production alias must
   match its physical USB path and use a dedicated `/dev/sprite-imu` symlink.
2. The measured serial configuration is 115200 8N1. At the factory-default 25 Hz
   output rate, a 10-second read-only capture produced exactly 250 complete update
   groups with 750/750 valid frame checksums. The module supports adjustment up to
   100 Hz; Sprite deployment uses 100 Hz so each 50 Hz policy interval receives
   two fresh IMU samples.
3. The verified serial frame is `7E 23 LENGTH FUNCTION PAYLOAD CHECKSUM`, where
   `LENGTH` includes the entire frame and `CHECKSUM` is the low byte of the sum of
   all preceding bytes. The observed functions are raw IMU `0x04`, quaternion
   `0x16`, and Euler angles `0x26`. Vendor examples define acceleration as
   `raw * 16/32767 g`, angular velocity as `raw * 2000/32767 deg/s`, magnetic
   field as `raw * 800/32767 uT`, quaternion order as `wxyz`, and Euler units as
   radians.

### Output-rate configuration

The vendor serial protocol defines function `0x60` for setting the automatic
report rate. Parameter 1 is the integer rate in Hz (10--100), parameter 2 is the
fixed guard byte `0x5F`, and this command has no protocol response. The 100 Hz
command is:

```text
7E 23 07 60 64 5F CB
```

`CB` is the low byte of the additive sum of all preceding bytes. Neither the
protocol nor the vendor Python library states whether the setting survives a
power cycle, so persistence remains an explicit commissioning test.

The following command requires an exact hardware-TX confirmation, measures the
raw-IMU frame rate before and after the one-time write, and fails closed unless
the measured result is within 5 percent (minimum tolerance 2 Hz):

```bash
PYTHONPATH=src .venv/bin/python -m open_sprite_runtime.cli imu-set-report-rate \
  --device /dev/ttyCH341USB0 \
  --baud 115200 \
  --rate 100 \
  --verification-duration 3 \
  --report reports/yahboom_imu_set_100hz.json \
  --confirm-hardware-tx WRITE_IMU_CONFIGURATION
```

After a full USB power cycle, repeat the read-only capture and measure 100 Hz
again before setting `imu.configured` to true.

4. Capture raw bytes without transmitting protocol data:

```bash
PYTHONPATH=src python3 -m open_sprite_runtime.cli imu-serial-capture \
  --device /dev/ttyCH341USB0 \
  --baud 115200 \
  --duration 10 \
  --raw-output reports/yahboom_imu_raw.bin \
  --report reports/yahboom_imu_raw.json
```

5. Use `open_sprite_runtime.yahboom_imu.YahboomStreamDecoder` against captured
   bytes, then verify orientation convention with six static face tests.
6. Configure and verify the sensor's native 100 Hz output rate. Never upsample a
   25 Hz stream and describe it as 50 Hz or 100 Hz feedback.
7. Measure sample gaps, host receive age, stationary gyro bias,
   and yaw drift. Only then fill the remaining IMU fields and set
   `imu.configured` true.
8. Re-run the fail-closed hardware inventory and policy observation tests before
   any motor-enable experiment.

Run a stationary, read-only commissioning audit with:

```bash
PYTHONPATH=src .venv/bin/python -m open_sprite_runtime.cli imu-static-audit \
  --device /dev/ttyCH341USB0 --baud 115200 --duration 20 \
  --report reports/yahboom_imu_static.json
```

To validate the physical mounting and live axis direction on the Jetson display,
run the absolute-attitude MuJoCo viewer. Correct mounting displays the robot
upright. A sensor lying flat with sensor +Z upward must not display upright. The
viewer keeps all joints at the model neutral pose, does not step physics, and
performs no serial or CAN transmission:

```bash
cd /home/tony/open_sprite_runtime
./review_sprite0825_imu_mujoco_on_253.sh 120
```

Startup-relative display remains available only as an explicit diagnostic mode
using `imu-mujoco-view --orientation-mode relative`; it must not be used to
qualify the physical installation.

The raw serial capture command never calls `write()`. Opening a USB UART may
change DTR or RTS, so both are held inactive. The decoder accepts only the three
documented frame types and rejects bad lengths or checksums.
