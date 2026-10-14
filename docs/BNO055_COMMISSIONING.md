# BNO055 preparation on Jetson253

Status: commissioning driver only; NOT selected as the live policy IMU.
No CAN, motor commands, resets, calibration-offset writes, or automatic yaw/tilt zeroing.
Old Yahboom scripts remain unchanged. Do not run a live controller while configuring an IMU.

## Bus and wiring

The hardware name I2C8 is not Linux adapter number8. The inspected253 has no
/dev/i2c-8; /dev/i2c-7 resolves to controller c250000.i2c. Orin Nano devkit
header pins3(SDA)/5(SCL) correspond to this I2C8 controller. Confirm carrier
board and actual wiring before use; do not scan unrelated buses.
Use common ground (e.g. pin6), 3.3V I2C logic/pull-ups, and a supply appropriate
to the specific breakout. Do not pull SDA/SCL up to5V. Wire with power off.
Default address0x28; set BNO_ADDRESS=0x29 if the breakout's address strap requires it.
No broad i2cdetect scan is used. Probe reads only the selected device's registers
(I2C register-address pointer writes are inherent to a read transaction).

## Modes and coordinates

IMUPLUS0x08 uses accelerometer+gyro, not magnetometer heading. Bosch specifies
100Hz fusion output. Configure writes CONFIGMODE, normal power, UNIT_SEL0x02
(m/s2, rad/s, Celsius, Windows Euler convention), P0 remap0x21/sign0x04,
then IMUPLUS, with settling waits and readback. Configuration must be reapplied
and checked after power-on. No external crystal is assumed.

P0 is not the identity mapping: output=(-chipY, chipX, chipZ).
User confirmed on2026-10-12: the physically aligned axes are the axes defined
by P0, matching the previous Yahboom X/Y/Z. Therefore use `p0-output-axes`:
X down, Z rear, Y right. This is now the launcher default. Do not undo P0 again.
Two explicit modes remain available for other mountings:

- `chip-axes`: that description refers to the native chip/board axes. The P0
  remap is included in the conversion back to the existing pelvis frame.
- `p0-output-axes`: that description refers to the axes AFTER the P0 remap.

Verify which axes the breakout silkscreen actually labels. Raw P0 quaternion,
gyro and acceleration remain in every sample. Converted body orientation is
marked mounting_verified=false until upright and single-axis tests confirm it.
No software startup leveling is performed; a tilted robot must remain tilted.

## Commands

From `/home/tony/open_sprite_runtime`:

```bash
bash ./probe_sprite0825_bno055_on_253.sh probe
bash ./probe_sprite0825_bno055_on_253.sh configure CONFIGURE_BNO055_IMUPLUS_P0
bash ./probe_sprite0825_bno055_on_253.sh record 60
```

The record command defaults to the user-confirmed `p0-output-axes` mounting.
For address0x29, prepend `BNO_ADDRESS=0x29` to each command. `BNO_BUS` explicitly
overrides the Linux adapter after verification. No configuration is written by
probe or record. If current register page is not0, the tool refuses ambiguous
access rather than silently writing a different device/page.

Record refuses an unconfigured or unhealthy fusion mode. It does NOT require
magnetometer calibration in six-axis mode; gyro/accel/system calibration values
are recorded separately. A successful recording is not deployment qualification.
Keep the supported robot still initially, then make small controlled pitch/roll
motions with motors disabled. Inspect axes, gravity, calibration, bias and lag.

## Timing and limitations

- Combined I2C_RDWR reads0x08..0x35 in a46-byte burst, avoiding SMBus32-byte truncation.
- Host100Hz polling records start/end times, raw bytes and missed polling slots.
- Burst reads reduce skew but do not establish atomic sensor capture or device timestamps.
- Repeated data can be valid at rest. Poll count is not a measurement of new sensor samples.
- Reject malformed quaternion or reads taking over30ms; recheck configuration/status each second.
- Synchronous I2C driver blocking is still possible; this diagnostic tool is NOT the policy reader.
- Live integration must use an independently sampled latest-state buffer and freshness watchdogs;
  it must not wait for I2C inside the50Hz policy inference tick.
- Before actuation: verify mounting, gyro sign, gravity sign, stationary bias,
  calibration behavior, dynamic response, and timed failures under CAN/policy load.

## Sources

- Bosch BNO055 datasheet, sections3.3/3.4/3.6 and page0 register map:
  https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
- NVIDIA carrier board specification (header GP16_I2C8_DAT / GP15_I2C8_CLK):
  https://developer.nvidia.com/downloads/assets/embedded/secure/jetson/orin_nano/docs/jetson_orin_nano_devkit_carrier_board_specification_sp.pdf
- NVIDIA clarification of header versus Linux adapter numbering:
  https://forums.developer.nvidia.com/t/i2c-bus-7-not-working-on-jetson-orin-nano-developer-kit/287217/15
