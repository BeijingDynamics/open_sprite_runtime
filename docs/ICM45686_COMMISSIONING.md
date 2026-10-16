# ICM45686 commissioning candidate

Independent Linux C++ executable; no CAN, policy execution or motor enable.
The working BNO055 reader and walking scripts are unchanged. As of 2026-10-16,
ICM has a distinct SPIMU003 native IPC adapter and three successful supervised
walks. See [walking evidence and scope](ICM45686_WALKING_20261016.md).
The capture executable itself never transmits CAN or enables motors.

## Wiring prerequisites

Confirm the module's supply/VDDIO requirements before power-on; Jetson GPIO/SPI
signals are 3.3 V, not 5 V tolerant. Use a common ground. For the selected SPI
header group, MOSI=physical19, MISO=21, SCLK=23, CS0=24. Default spidev0.0 is a
candidate mapping, to be confirmed by identity read on the actual board.
INT1=physical22, INT2 unconnected. NVIDIA Orin Nano mapping identifies pin22 as
tegra234-gpio PY.01 (gpiochip0 offset123 on the inspected253). The program checks
chip label/line name and exclusive availability rather than trusting offset alone.
Do not confuse BOARD22 with BCM22 or the AGX Orin mapping.

The sensor axes must be verified physically: X down, Z back, Y right. The body
transform is (-sensorZ, -sensorY, -sensorX). No BNO P0 register remapping applies
to this chip. Verify all six static orientations and rotation signs before reuse.

## Commands on253

```bash
cd /home/tony/open_sprite_runtime
bash build_sprite0825_icm45686_on_253.sh
bash probe_sprite0825_icm45686_on_253.sh probe
bash probe_sprite0825_icm45686_on_253.sh 400 30 CONFIGURE_ICM45686_RAW_NO_CAN
bash probe_sprite0825_icm45686_on_253.sh 800 30 CONFIGURE_ICM45686_RAW_NO_CAN
```

Probe reads WHO_AM_I only (expected0xE9); it does not configure sensor registers.
Capture resets/configures only the selected IMU, including INT1. Stop other
readers first. Do not run this commissioning capture during powered walking.
Use `ICM_SPI_DEVICE=/dev/spidev0.1` only if wiring intentionally uses that CS.
The first15 seconds of the total30 seconds require stillness. BIAS_READY marks
the beginning of attitude output. Ctrl+C stops capture; data is written on exit.
The selected rates are400 and800Hz; unsupported rates are rejected before access.

## Data and timing

- Official pinned TDK basic driver, SPI mode3/MSB/1MHz, one14-byte burst for
  six axes plus temperature. Full scales4g and2000deg/s; LN mode and DIV4 LPF.
- INT1 rising edges use kernel monotonic timestamps. Old queued edges are
  coalesced; missing sequence numbers and late reads are counted. Register reads
  are not FIFO history: discarded samples are never replayed as fresh data.
- Existing StaticBias and SixAxis algorithms are reused unmodified, in body
  coordinates, with actual timestamp differences. No magnetometer. Gyro output
  after bias readiness is raw gyro minus fixed startup bias, not fusion correction.
- CSV preserves raw counts, body SI acceleration/gyro, gravity unit direction,
  body-to-world quaternion(wxyz), sequence and edge/read timestamps. Before
  bias_ready, gyro is uncorrected and attitude/gravity columns are placeholders.
- JSON records maximum SPI duration, edge-to-read age, sample gap and drops.
  `completed` only means capture completed, not that timing or physical accuracy
  passed. `hardware_qualified` remains false. CSV supports percentile analysis.
- Edge timestamps do not measure the sensor's internal filter group delay.
  Higher ODR is not by itself proof of lower end-to-end attitude latency.

## Commissioning gates

Identity/configuration readback, interrupt wiring, observed ODR, unit/sign and
90degree rotation checks, stationary noise/bias, 400vs800 timing/drop comparison,
then MuJoCo-only display and full-load zero-gain shadow form the commissioning
sequence. See the dated milestone for completed checks and measured results;
repeat physical unit/sign checks for a new installation. Never fake
BNO raw-count packets using ICM data. Existing ankle500Hz/CAN scheduling unchanged.
