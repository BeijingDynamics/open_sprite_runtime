# BNO055 runtime input, 2026-10-13

User visually confirmed absolute attitude and hand-motion directions.
Linux I2C bus 7, address 0x28; IMUPLUS six-axis, P0 output axes physically
aligned with the prior Yahboom mounting. No additional P0 inversion.

The native policy client accepts:

```text
--imu-source bno055 --bno-bus 7 --bno-address 0x28 --bno-initialize
```

Default remains Yahboom. Existing powered launchers are unchanged. These
arguments alone do not constitute authorization for a powered hardware test.
Policy weights, gains, command schedule, motor limits and native CAN binary
are unchanged.

Initialization/readback occurs before IPC connection. Configuration writes
require the explicit startup option. During operation a configuration/status
change fails closed; the reader never automatically reinitializes mid-run.
An independent 100 Hz worker publishes only its newest sample. Policy reads
do not perform I2C transactions. Read-start monotonic timestamps are preserved;
duplicate consumption does not refresh freshness. Existing 30 ms checks before
inference and target transmission remain active. This age is NOT total sensor
fusion latency. No device sample counter is available.

Existing Yahboom-named SI packet containers are reused internally, not its
wire protocol. Magnetic field values are placeholders (zero) and explicitly
marked unavailable in the source report. They must not be used for heading.
The existing mounting transform is applied once by the policy observation path.

Validation:
- 13 BNO driver/source tests, 3 native-client tests, 15 serial-receive tests passed.
- Jetson 10 s sensor-only source check: 1002 polls, zero skipped slots/errors,
  maximum I2C read 8.621 ms, maximum consumed host sample age 17.464 ms.
- Report: reports/bno055_source_check_20261013_115707.json on Jetson.
- No CAN traffic, enable commands or policy actuation were executed.

Remaining gate: full native IPC/ONNX shadow using measured motor feedback,
including compute-load freshness and disconnect behavior, before any powered
comparison. Sensor-only success is not full policy qualification.

Sensor-only check (may initialize IMU; never accesses CAN):
```bash
cd /home/tony/open_sprite_runtime
PYTHONPATH=src .venv/bin/python tools/check_bno055_source.py
```

Remote rollback copy of the previously verified client:
`src/open_sprite_runtime/native_policy_client.py.before_bno055_20261013`.
