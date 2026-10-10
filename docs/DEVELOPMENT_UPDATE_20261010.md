# October 10 runtime development update

## Walking milestone

The supervised 40-second battery-powered trial on 2026-10-10 completed:
0-10 s standing command, 10-13 s ramp to 0.10 m/s, 13-27 s forward command,
27-30 s ramp to zero, and 30-40 s standing command before disable.
All 31 motors were verified disabled at exit. The operator observed actual
stepping and a final stop; the left lower leg contacted the support frame
during the trial. This is a bounded milestone, not unrestricted walking,
turning, reverse, gamepad control, or unsupervised-operation qualification.

The 2001-frame local trace showed median forward-phase pelvis tilt 2.65 degrees,
95th percentile 7.54 degrees, and maximum 16.51 degrees. Final-standing tilt
had median 0.57 degrees and maximum 2.68 degrees. The log has no external
position, support-load, or foot-contact measurement; measured forward speed
and collision timing cannot be established from the velocity command alone.
Raw physical-trial files remain in the local backup, not in this update.

## IMU changes after the trial

The native Python client drains available serial packets before inference,
rejects excessive backlog, tracks partial-frame age, and verifies raw and
quaternion freshness before inference and target transmission (30 ms host-age
budget). It never waits for a new packet inside a policy tick. Host receipt
age is not the sensor's complete physical latency.

Relative gyro heading resets immediately before the first inference of each
session and is recorded with a validity flag. Integration uses policy-rate
samples and tilt correction. It does not modify actor observations, gyro bias,
motor zeros, commands, or controller gains. There is no heading-hold feedback
in this native path. Quaternion and gyro yaw disagreement from the walking
trace remains an open investigation; do not assume the heading estimate is
ground truth or that resetting it fixes left drift.

A five-second receive-only sensor check passed after the freshness fix.
The new IMU and heading code has not yet completed a powered policy trial;
full zero-gain shadow validation is the next gate. Do not conflate the earlier
walking success with validation of these later changes.

## Xbox controller

Bluetooth pairing and Linux input registration succeeded on Jetson 253. The
read-only probe saw initialization events without disconnects; live axis and
button coverage still needs operator verification. The agreed interface is
LT permission, AXIS 1 forward/reverse, AXIS 2 turning, and controlled stopping
on release/disconnect. This remains a design, not active hardware teleoperation.
See [the pairing and control guide](XBOX_BLUETOOTH_CONTROL.md).

## Release boundaries

This update preserves the previously published model artifacts, hardware
configuration, and historical records. It adds the bounded command schedule,
IMU receive protection, heading telemetry, gamepad probe, tests and documentation.
Private cloud addresses, credentials, raw commissioning logs, experiments and
build products are excluded. Published active-control scripts retain their
explicit acknowledgement requirements; documentation is not an authorization
to run a physical trial.
