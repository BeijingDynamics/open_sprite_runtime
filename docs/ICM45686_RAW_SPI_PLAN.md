# ICM45686 raw SPI candidate

Historical design plan. For the current implementation and October 16 supervised
walking results, see [the milestone](ICM45686_WALKING_20261016.md).

Status (2026-10-14): independent native commissioning candidate implemented;
offline build/test on253, real ICM hardware qualification pending. See
ICM45686_COMMISSIONING.md for commands and scope. Preserve the working BNO055
software-estimator/native-policy walking entry unchanged. Policy IPC integration
and live viewer remain subsequent hardware-gated work.

## Confirmed requirements

- Jetson Orin Nano 40-pin header; SPI enabled and spidev nodes present.
- Module INT1 planned for physical pin 22; INT2 not connected.
- Confirm module supply/VDDIO, physical axis markings, actual SPI controller/CS
  mapping and pin22 GPIO input/edge capability before hardware access.
- Raw accelerometer and gyroscope only. No on-chip attitude fusion or APEX.
- Reuse StaticBias and SixAxis in native/src/imu_six_axis.hpp. Keep policy at
  50 Hz and ankle control at 500 Hz; IMU acquisition is an independent process.

## Official source findings

Reference: https://github.com/tdk-invn-oss/motion.arduino.ICM45686
Reviewed commit: 9abfe2e112f0d219c293e8960e76ece882bbaebf

- src/ICM45686.cpp accel_freq_to_param/gyro_freq_to_param and
  src/imu/inv_imu_defs.h expose ordinary ODRs 200, 400, 800, 1600 Hz and others,
  not 500/1000 Hz. The Arduino wrapper silently defaults unsupported values
  to 100 Hz. Our candidate must reject unsupported requested rates.
- User-approved test rates: 400 Hz first, then 800 Hz sensor ODR/estimator
  updates; retain the BNO 200 Hz baseline for comparison. Defer 1600 Hz to
  limit I/O load; do not include it in the current test sweep.
  Do not label 1000 Hz host polling of 800 Hz
  hardware as 1000 independent samples per second.
- inv_imu_get_register_data reads accel, gyro and temperature together and
  converts according to configured register endianness. Decode signed values
  and configured full scales explicitly, yielding m/s^2 and rad/s.
- Prefer the official platform-independent C driver with Linux SPI transport
  callbacks over importing Arduino runtime or hand-writing initialization.
  Preserve the upstream license notices for any vendored files.

## Implementation sequence

1. Map spidev and pin22 GPIO on the actual carrier. Probe WHO_AM_I at a low
   SPI clock before any sensor configuration. No CAN access in this tool.
2. Add an independent native raw capture candidate: configure equal accel/gyro
   ODR, documented low-noise mode and explicit bandwidth, read settings back.
   Use INT1 data-ready and monotonic kernel edge timestamps. Keep SPI transfer
   start/end and publication times separately; a GPIO edge is not automatically
   an exact physical sensing timestamp.
3. Detect stale/coalesced interrupts and acquisition overruns. Do not associate
   old queued edges with current register contents or integrate duplicate data.
   Evaluate small FIFO/watermark capture if direct-register consistency or
   missed samples require it; avoid accumulating batches that add latency.
4. Convert raw data into the existing estimator's physical units and frame.
   Verify all three signs and 90-degree rotations. BNO P0 is a BNO register
   remap, not an ICM setting: reuse the body-frame convention, not its registers.
5. Reuse 15-second stationary bias calibration, frozen gyro bias, actual-dt
   integration and gravity estimator. Policy gyro remains bias-corrected raw
   gyro rotated to body frame, without attitude-correction feedback mixed in.
6. Preserve the observation contract. The current SPIMU002 packet includes
   BNO-specific raw bytes: do not forge BNO counts from ICM data. Implement an
   explicit sensor-neutral/versioned adapter and replay-equivalence tests
   before wiring the candidate to policy IPC.
7. Validate stationary noise/bias, orientation, known-angle integration and
   MuJoCo display without motors. Then measure full-load zero-gain timing;
   powered trials require separate onsite authorization.

## Acceptance evidence

Record requested and read-back ODR, scales/bandwidth, unique samples, interrupt
gaps, missed edges/overruns, SPI duration, edge-to-publication latency and IMU
age consumed at each 50 Hz policy tick (p50/p95/p99/max). Also compare CPU load,
noise and bias stability against the BNO baseline. Higher ODR alone does not
prove lower end-to-end latency: sensor filtering and Linux scheduling matter.
