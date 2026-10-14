# Dual IMU acquisition pause diagnosis

## Confirmed cause and correction

The diagnostic recorder retains many Python dictionaries for both sensors
in one process. Instrumented run dual_imu_guided_20261013_134901 recorded a
generation-2 cyclic garbage collection starting at27.730s and taking107.593ms.
BNO then failed its existing30ms read guard. This confirms a shared Python
GC pause in that run, not an I2C/USB hardware outage. Earlier uninstrumented
80/140ms pauses have a matching pattern but are not individually proven GC.

The recorder now defaults to deferring cyclic GC only during its bounded
capture (maximum120s stationary or55s guided). Initial collection runs before
readers start; prior GC enabled state is restored after readers join, including
error paths. Reference counting remains active. No sensor configuration,
motor commands, policy parameters, or freshness thresholds were changed.
Use --allow-cyclic-gc only for diagnostic reproduction.

Validated60s run dual_imu_guided_20261013_135011:

| Sensor | Median gap ms | P99 gap ms | Maximum gap ms | Gaps above30ms |
| --- | ---: | ---: | ---: | ---: |
| BNO | 10.001 | 10.109 | 10.562 | 0 |
| Yahboom | 9.999 | 10.878 | 13.869 | 0 |

No unlimited global GC suppression was added to production control.

## Policy relevance and verification limits

Native policy client checks IMU age before inference, after inference, and
before sending targets; maximum age remains30ms. Bno055Source preserves
read-start timestamps and rejects stale cached data and worker faults.
The source and freshness test suites passed19tests.

benchmark_bno_policy_no_can.py runs the deployed ONNX model using actual
recorded observation vectors while concurrently running the production BNO
source. It never opens CAN, enables motors, or writes sensor configuration.
It does not replace recorded observations with live IMU data and does not
exercise the complete native IPC, projection, full trace, or CAN controller.

Report bno_policy_no_can_20261013_135306.json,60s:
- 3000/3000 inference cycles; automaticGC enabled.
- BNO5999polls,0skipped slots, maximumread7.150ms.
- Inference maximum2.382ms.
- IMU age after inference P997.824ms, maximum17.349ms.
- No errors. GC callbacks only saw brief generation0 collections.

This qualifies the reduced concurrent sensor/inference load, not end-to-end
hardware control nor arbitrarily long sessions. Full zero-gain shadow with
nativeIPC/CAN and normal trace logging is the next gate before powered walking.
If heavy retention causes GC pauses there, use bounded/preallocated telemetry
or isolate sensor acquisition rather than relax freshness limits.

Ruler-based gyro scale/fusion assessment must be repeated with corrected
capture. No bias or scale corrections were adopted from earlier captures.
