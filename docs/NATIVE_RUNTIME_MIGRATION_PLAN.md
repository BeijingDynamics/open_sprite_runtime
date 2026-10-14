# Native runtime migration

Status: in progress, 2026-10-13. Owner: open_sprite_runtime.

## Objective and invariants

Remove Python cyclic-GC coupling from IMU acquisition and then the policy
execution path. Keep the current ONNX weights, observation contract, 50Hz
policy period, 100Hz BNO polling, 500Hz ankle PD, joint order, units, gains,
mapping and motor protection unchanged. No zero-setting commands, implicit
IMU configuration writes, or motor enables during development validation.

Three independent processes: sensor producer; policy consumer; existing
native CAN controller. Sensor and CAN must never wait for ONNX completion.
Python remains useful for configuration, comparison, reports and viewers.
This is not a hard-real-time certification: kernel/driver/scheduling latency
must still be measured. Host read timestamps are not sensor capture times.

## Stage 1: native BNO acquisition

- Read-only Linux I2C_RDWR, existing bus7/address0x28 configuration.
- Verify IMUPLUS/P0, units, chip identity and health before publishing and
  periodically thereafter. No automatic configuration or recovery writes.
- Fixed-size versioned packets containing sequence, original monotonic read
  timestamps, raw46bytes and decoded values; single bounded latest snapshot.
- Nonblocking Unix IPC, no consumer-dependent queue growth. Slow/disconnected
  consumers must not delay sensor polling. Expired timestamps never refreshed.
- Preserve30ms validity gate and explicit errors; no stale-data fallback.
- Test decode against Python on recorded samples, malformed payloads,
  invalid quaternion, stale data, producer exit, consumer pause and restart.
- Test at least120s locally on253 without CAN and with CPU inference load;
  record p50/p99/max gap, read duration, data age, sequence skips, memory.
- Do not replace powered entrypoints until full shadow passes.

## Stage 2: C++ observation and ONNX inference

- Freeze model/contract/config hashes and a representative real trace.
- Implement native mapping, mount transform, projected gravity, observation
  scaling, clipping, joint/history order, startup warmup and action conversion.
- ONNX Runtime CPU first, one inference thread initially; no MNN migration.
- Preallocate observation/history/action/telemetry buffers. Avoid synchronous
  disk output and unbounded containers in control loops.
- Compare Python and C++ observation per element (target1e-6 float tolerance),
  action and joint targets (initial target1e-5; document any justified change).
  Quaternion sign equivalence must not mask frame/order errors.
- Replay normal walking, stand/start/stop and boundary/stale/fault cases.
- Migrate startup projection/ramp and all safety semantics explicitly; do not
  bypass them merely because the model output matches.

## Stage 3: complete zero-gain shadow

- Only reviewed zero-gain CAN polling; no motor enable or position-zero writes.
- Use actual motor feedback and live IMU through new IPC with ordinary logs.
- Short smoke first, then10min concurrent CAN/IMU/50Hz policy test.
- Preserve existing native timeouts, guards and500Hz ankle schedule. Inspect
  native500Hz jitter, policy20ms deadline misses, data age, packet backlog,
  CPU usage and memory. No stale sample may be accepted above30ms.
- Exercise no-CAN simulated fault injection for producer death, malformed
  packets, stale timestamp, sequence regression and consumer stalls.
- Progress only with documented evidence; exceeding thresholds is failure,
  not a reason to relax freshness limits.

## Stage 4: powered comparison

- Requires fresh user authorization and physical safety readiness.
- Same model, gains and command schedule as accepted baseline; begin short
  supported stand, then40s stand/walk/stop after previous gate passes.
- User visual review plus numerical timing, symmetry and motor-load review.

## Rollback and documentation

Keep existing Python source and all current powered scripts unchanged by
default. New binaries/scripts use separate names/build directory; never
rebuild the active CAN binary or disturb its capabilities for sensor work.
Record SHA256, build/compiler, test reports and remaining gaps. Do not push
private credentials or cloud addresses. Update this file at each gate.

## Progress

- [x] Existing GC failure captured; bounded diagnostic collector corrected.
- [x] Reduced ONNX plus BNO60s no-CAN benchmark completed (not full shadow).
- [ ] Native BNO implementation and offline/physical verification.
- [ ] Native observation/ONNX equivalence.
- [ ] Full shadow qualification.
- [ ] Authorized powered validation.

### Stage1 implementation checkpoint

- Independent sprite_imu_bno C++17 binary implemented; read-only I2C_RDWR,
 100Hz polling, config health checks,30ms read/gap/freshness rejection.
- Versioned192-byte little-endian packets, fixed-size latest sample and
 nonblocking Unix datagram request/reply; no consumer wait in polling path.
- Only diagnostic duration1..600s currently supported. Timing vectors reserve
 storage before capture and report after capture. This is not yet an unlimited
 service or the production Python policy's input source.
-5489 recorded frames decoded identically to Python; native invalid quaternion
 and truncated-file rejection verified; six Python packet tests passed.
- ARM64 build uses isolated build/imu_native_stage1 directory; CAN binaries
 and capabilities untouched. No powered launchers changed.
- First live qualification refused BNO config: UNIT_SEL0x80, OPR_MODE0x10,
 AXIS_MAP_CONFIG0x24, AXIS_MAP_SIGN0, SYS_STATUS0. Python independent probe
 agreed. User subsequently approved restoring IMUPLUS/P0. Existing Python
 commissioning tool restored configuration; readback matches and fusion status5.
 The native reader itself performs no configuration writes.

### Stage1 physical smoke completed, 2026-10-13 14:52

- Reports: native_bno_stage1_qRbntIVI/producer.json and
 native_imu_consumer_20261013_145224.json, backed up locally.
- Producer130s,12999samples,1skipped slot, P99gap10.0156ms,
 maxgap20.0071ms; maxread7.81893ms. No interval above30ms.
- Consumer120s,6000/6000ticks, no errors; post-inference age P999.031ms,
 maximum18.098ms; maximum inference1.380ms.
- Deliberate consumer150.22ms pause: sequence advances15 without producer
 stall.6000IPC replies,0send drops. No CAN transmission or enables.
- Uses actual ONNX with recorded real observations plus concurrent live
 native IMU. It is NOT full live-observation policy or native CAN/IPC shadow.
- Remaining stage1 gates: producer death/restart and stalled-client faults,
 production input adapter and full shadow integration. Investigate the single
 skipped slot (periodic health reads are a possible cause, not yet established).
 Existing walking entrypoints remain unchanged. No powered test authorized.

### Native input adapter checkpoint, 2026-10-13

- Added opt-in native-bno055 source and --native-imu-socket to the Python
 policy client. Defaults and powered launchers remain unchanged.
- Adapter preserves producer timestamps, rejects stale or malformed packets
 and sequence regression, latches failures, and does not reconnect silently.
- Local Windows: six packet tests, three adapter unit tests, and fifteen
 IMU receive regression tests passed. Six Unix-socket tests were explicitly
 skipped because this Windows Python has no AF_UNIX.
- Added Linux fault tests for missing producer cleanup, timeout, malformed
 packet, stale packet, sequence reset, and producer exit/error latching.
- Deployment and Linux execution are pending. The SSH tool request did not
 execute: automatic approval review reported model capacity failure. This is
 not evidence of a Jetson or network fault. No hardware TX occurred.
- Next: deploy adapter/tests, run Linux suite, then create an opt-in full
 zero-gain shadow entrypoint with the native producer. Do not configure BNO
 concurrently with sampling, or enable motors as part of this validation.

### Stage1 integrated 10s shadow passed, 2026-10-13 15:18

- SSH restored. Adapter deployed after diffing against the remote client;
 only opt-in source selection changed. Nine adapter/fault tests and six packet
 tests passed on Jetson; existing remote IMU receive suite passed twelve tests.
- New entrypoint: probe_sprite0825_native_bno_full_shadow_on_253.sh
 ZERO_GAIN_ALL_DISABLED_SUPPORTED. Fixed 10s CAN diagnostic, 30s native IMU
 producer. Requires supported robot/all motors disabled. No IMU config writes.
 Existing BNO shadow supports an explicit SPRITE_NATIVE_IMU_SOCKET override;
 default Python behavior and powered walking launchers remain unchanged.
- User explicitly confirmed support, all motors disabled, power on and agreed
 to this 10s test. All 31 motor TX/RX coverages were 100%; ankle polling500Hz,
 other motors50Hz. Native CAN deadline misses0, maximum lateness0.024384ms.
- Policy500/500 computations; native received499/500 targets (99.8%, existing
 gate passed). Final shutdown-boundary target accounting needs verification;
 do not label this 100% native target reception.
- Policy reported inference mean2.106ms, P992.711ms, max6.485ms. This field
 includes policy work beyond just the ONNX actor; last actor stage0.355ms.
- Trace IMU age median13.864ms, P9915.094ms, max19.163ms. Host read-start
 age only, NOT internal sensor fusion/physical latency. No rejected frames.
- Native IMU2999samples/30s, one skipped slot, maxgap20.0054ms,
 maxread8.28842ms. The skipped slot remains an investigation item.
- All enable/mode-switch/nonzero CAN gain-or-torque attempt counts were0.
 Reports and trace backed up to local work/walk40: native_policy_ipc_*_20261013_151808
 and native_bno_full_shadow_I3JjBm5i. Both policy and native reports passed.
- Still pending: longer soak, sampling skip diagnosis, native observation/ONNX
 migration and equivalence. Short shadow success is not powered qualification.

### Segmented long shadow passed, 2026-10-13 15:24-15:34

- User authorized long stability testing after supported/disabled confirmation.
 Native CAN diagnostic retains its120s duration ceiling. Ran5x110s sessions
 with a single uninterrupted600s IMU producer, not one600s policy session.
- Wrapper now accepts duration and rounds after its acknowledgement token;
 validates duration1..120, rounds1..5 and producer duration<=600. Stops further
 rounds on failure; no automatic retries. Defaults remain10s/one round.
- All five policy and CAN gates and the producer gate passed.27500policy ticks.
 CAN deadline misses0; maximum scheduling lateness0.199858ms.
- CAN TX1842500/RX1842498. Last round left_ankle_motor_a and
 right_ankle_motor_a each recorded54999/55000responses. Do not claim zero loss;
 ending-window accounting versus actual missing response is not yet resolved.
- Each native session received5499/5500policy targets, while Python sent5500.
 Consistent with a possible shutdown boundary, not proven; retain for follow-up.
- IMU59996samples,4skipped slots, gapP9910.043ms/max24.1328ms;
 readP996.28867ms/max12.7862ms. No80-140ms stalls.107214replies/0send drops.
- Combined trace host read-start age median10.211ms/P9917.896ms/max24.759ms.
 Per-round P99/max:12.368/24.078,18.095/23.957,10.651/20.887,
 10.447/20.629,13.567/24.759ms. No monotonic deterioration demonstrated.
 These are trace target-time ages, not sensor-internal fusion latency or a
 complete recording of every post-send timing stage.
- Maximum reported policy work duration16.638ms. IMU rejection counts0.
 No enable, mode switch, or nonzero-gain/torque transmission occurred.
- Local backup work/walk40/native_bno_full_shadow_azj67e2Z plus reports/traces
 with stamps20261013_152429,152623,152817,153011,153205.
- Remaining: investigate skipped slots and end-of-session accounting; add
 uninterrupted longer policy qualification when diagnostic support is ready;
 proceed with native observation/ONNX equivalence. No powered entrypoint changed.

### Powered comparison prepared (NOT executed), 2026-10-13

- User requested scripts only, notification before testing. Added independent
 run_sprite0825_native_bno_waist60kd3_walk40_on_253.sh wrapper with explicit
 ENABLE_NATIVE_BNO_WAIST60_KD3_WALK40 acknowledgement; --check is static only.
- Baseline run_sprite0825_bno055_waist60kd3_walk40_on_253.sh was downloaded and
 diffed: only optional SPRITE_NATIVE_IMU_SOCKET selection added. All policy,
 gains, torque limits, admission checks and40s command profile unchanged.
 Without this variable the original Python BNO input remains the default.
- Wrapper starts finite120s read-only native BNO producer, requires a fresh
 packet, then reuses baseline admission and powered procedure. Same producer
 spans preflight and powered stages. No auto IMU configuration writes.
- Cleanup waits for baseline controller disable handling before stopping IMU.
 Producer report may say capture interrupted on intentional wrapper cleanup;
 that flag alone is not a sensor-failure diagnosis in this wrapper.
- Refuses an existing native IMU/policy/CAN process. Native producer lock also
 rejects a second native reader; unrelated Python I2C readers must stay stopped.
- Deployed to253. Bash syntax, module import, executable availability and
 --check passed; native CAN capability cap_sys_nice=ep verified. No CAN/I2C
 access or actuation performed during these static checks. Signal handling
 and powered behavior still require the controlled comparison; not yet qualified.
- Next test requires fresh onsite support/clearance/power-safety confirmation.

### First native-IMU powered comparison completed, 2026-10-13 15:55

- Initial attempt refused BNO configuration before CAN or enable. Readback
 UNIT_SEL128/OPR_MODE16/MAP36/SIGN0/status0. Cause remains unknown.
 User authorized IMUPLUS/P0 restoration; readback matched, status5/error0.
 Five-second native read-only check500samples,0skips, maxgap10.2902ms passed.
- After renewed onsite authorization, executed40s baseline comparison with
 native IMU only changed. Preflight passed; stand/ramp/0.10m/s/stop completed.
- Reports native_protected_policy_*_20261013_155525 and native_bno_walk40_OvIDnRrx
 backed up locally under work/walk40. Native passed/errors empty, all31drives
 final_disabled=true. No motor zero reset or mode switch. This WAS powered:
 Python actor's legacy shadow/no_actuation label describes its lack of direct
 CAN ownership, not absence of native actuation.
- CAN deadline misses0, maxlateness0.043924ms.2001states/2000native targets.
 Trace IMU read-start age median14.652ms/P9915.443ms/max17.145ms.
 Actor processing max6.164ms; no IMU rejected frames.
- Torque saturation events: left knee23, right knee33, waist yaw2. Not yet
 localized to startup versus walking. Relative gyro yaw final0.8654rad (~49.6deg)
 requires visual context; not evidence of sensor drift by itself.
- User visual gait feedback pending. Do not equate software gates with gait
 quality or automatically repeat/change gains. Producer intentional cleanup
 interruption is expected after native controller exit.

### Same-parameter repeat, 2026-10-13 15:58

- User rated155525 as best actual walking so far and confirmed persistent
 physical left yaw, then requested one same-parameter repeat. No gains,
 policy, compensation or protection thresholds changed.
- Repeat155850 completed40s, passed/errors empty, all31motors disabled,
 CAN deadline misses0. Trace IMU age median8.823ms/P9919.404ms/max20.308ms.
 Relative gyro yaw final60.58deg versus49.59deg previously (not absolute truth).
- Saturation counts increased: left/right knee204/122 versus23/33; waist yaw517
 versus2. Also hip pitch41/26, hip roll44/43, shoulder pitch112/113,
 shoulder roll11/9, waist roll2, head motor_b2. Counts are events, not durations
 or proof of causation; initial posture, external support and walking phases
 need separation before diagnosing. User visual feedback for repeat pending.
- All native_protected_policy_*_20261013_155850 files and
 native_bno_walk40_xq1dUs7c backed up in local work/walk40. No further test run.

### Native policy pure-compute milestone, 2026-10-13

- User reports performance broadly unchanged but walking demonstrated; requests
 native policy migration. No new powered run authorized or performed here.
- Added policy_core.hpp: float32 term-major8-frame histories, replicated first
 frame, previous blended action, smoothstep measured-pose handoff and affine
 target. Frozen31joints/795observations/50Hz contract checked by fixture exporter.
- Added sprite_policy_replay.cpp with CPU ONNX Runtime1.23.2, one inference
 thread. Offline file IO only: no CAN, I2C, sockets, enable or zero commands.
- Built separately in build/policy_native_stage2 on253. Reused installed
 libonnxruntime.so.1.23.2 with stage-local soname symlink; official Microsoft
 v1.23.2 headers. Existing live binaries, capabilities and launchers untouched.
- Two independent recursive C++/Python replays,2001frames each:
 reports/native_policy_equivalence_155525_v2 and native_policy_equivalence_155850_v1.
 Max errors observation/raw action/blended action/target ALL0. Python rebuilt
 observation versus recorded live observation also0 on both traces.
- Native offline compute mean/max0.175331/1.01527ms and0.172930/0.563557ms.
 Not a live scheduling benchmark and excludes motor/IMU transforms, IPC,
 safety projection, physical-startup ramp and CAN. Timing cannot be compared
 directly to full Python live cycle measurements.
- C++ core unit tests passed (history ordering, handoff, NaN/Inf and zero scale
 rejection). Initial test gravity index typo corrected; implementation unchanged.
- Still incomplete: native motor mapping and IMU transform, protected target
 projection/ramp, live50Hz IPC orchestration, end-to-end shadow equivalence,
 long soak and authorized powered comparison. Do not deploy replay as controller.

### Native feedback conversion milestone, 2026-10-13

- Added policy_feedback.hpp and sprite_feedback_replay.cpp. Offline file IO
  only; no CAN, I2C, sockets or motor commands. Existing live entrypoints unchanged.
- Exported affine mapping from the current Python hardware configuration,
  including direct motor signs, zeros and both ankles/head differential pairs.
  Native inverse mapping computes joint positions and velocities. IMU conversion
  preserves the verified P0 sensor-output-to-pelvis mounting transform once.
- Built on253 with C++17, warnings-as-errors and disabled FP contraction.
- Verified both recorded walking traces (2001 frames each), plus256 synthetic
  samples per run (same seed). Maximum native/Python errors:8.88e-16 and1.78e-15;
  comparison against original recorded joint/IMU values has the same maxima.
  Angular velocity transform error0; projected gravity error6.66e-16.
- Reports: reports/native_feedback_equivalence_155525_v1 and
  reports/native_feedback_equivalence_155850_v1. Both passed; hardware_tx=0.
  Truncated/trailing input, invalid magic, NaN and zero quaternion rejected.
- These tests validate conversion mathematics, not sensor freshness, motor
  calibration accuracy, live scheduling or actuator safety. No powered test run.
- Next: protected target projection and startup ramp parity, then live IPC
  orchestration with stale-data/sequence guards, offline and shadow validation.
  Full native policy control remains incomplete; do not replace the live launcher.

### Native protected-target and startup math milestone, 2026-10-13

- Added policy_protection.hpp and offline sprite_protection_replay.cpp.
  Preserves projection-before-startup order, per-joint gain/damping multipliers,
  qualified Kd rejection, initial measured-pose bounds check, ceil-based hold/ramp
  tick counts, smoothstep alpha and all five impedance target vectors.
- First pose is captured once. Subsequent measured poses are finite-checked but
  not recaptured or incorrectly checked against startup limits every frame.
  No gains, limits, live launchers, motor firmware or calibration changed.
- Jetson warnings-as-errors build passed. verify_native_protection_replay.py
  exercised4 timing configurations,501 frames each (2004 total), including
  zero hold, sub-tick ramp, fractional durations and2s hold/6s ramp.
  All output position/velocity/Kp/Kd/feedforward/alpha errors versus Python are0.
- Twelve rejection cases passed: negative Kp/Kd, excessive Kd, startup pose
  outside bounds, NaN, trailing data, zero Hz, negative hold, zero ramp,
  invalid scale/Kd ceiling and unordered limits.
- Evidence: reports/native_protection_equivalence_v1; hardware_tx=0.
  These are deterministic synthetic fixtures, NOT a live or recorded full-chain
  replay. The projector/ramp telemetry counters are not yet compared separately.
- Reproducible standalone build: bash tools/build_native_policy_math_replays_on_253.sh.
- Remaining before live replacement: clamp watchdog, command-profile timing,
  integrated config/order/hash validation, state/IMU IPC, freshness/sequence and
  deadline guards, complete recorded-chain parity and no-actuation soak.
  Powered acceptance still requires a fresh onsite safety confirmation.

### Native policy IMU IPC client, 2026-10-13

- Added policy_imu_client.hpp: dedicated200Hz Unix-datagram receiving worker,
  connected peer, private temporary socket directory,25ms receive timeout,
 30ms original host-read-start freshness, packet version/reserved/raw-decoded
  consistency checks. Errors latch; no automatic reconnect or timestamp refresh.
- Snapshot API does not wait for a packet or perform I2C. Rechecks freshness
  on each access. Future integration must check before inference AND before
  target send; snapshot freshness alone is not a complete control-loop guard.
- Added sprite_imu_ipc_probe and fake-server test harness. No CAN, I2C, actor,
  targets, enable, calibration or sensor configuration writes in these tests.
- Initial compile found missing imu_packet.hpp on253; synced existing local
  protocol header, without replacing deployed sensor/CAN binaries.
- Fourteen fake-server scenarios passed: normal and repeated identical samples;
  silence, stale/future/reversed timestamps, regressing sequence, oversized packet,
  bad magic/reserved bytes, raw/decoded mismatch, NaN, bad quaternion and freeze.
- Healthy cases deliberately stall the consumer120ms and confirm producer
  sequence progresses in the independent receiving thread. This is NOT proof
  of timing under ONNX/CAN/system contention. No realtime priority changes made.
- Evidence: reports/native_policy_imu_ipc_v2.json, backed up locally.
- Remaining: motor-state/target IPC and config validation, complete policy loop,
  integrated parity, fault injection and longer no-actuation load tests. Current
  powered walking entrypoint continues to use the previously qualified path.

### Native motor-state/target IPC milestone, 2026-10-13

- Added policy_state_ipc.hpp: existing SPRT v1 little-endian ABI,528-byte
  motor state and1280-byte target, ordered FNV-1a name hashes, finite numeric
  validation, increasing state sequence and source-state reference on targets.
- SOCK_SEQPACKET client uses nonblocking sends and bounded100ms receive poll.
  Transport/validation errors latch; no automatic reconnect. State age is checked
  at receive and before send. Maximum age is an explicit constructor parameter;
  fake tests use100ms. This is not a newly qualified hardware timeout value.
- Added fake-only sprite_state_ipc_probe with deliberately non-robot name hashes;
  it sends fixture targets only to test sockets, never opens CAN. All five target
  fields, including nonzero Kp/Kd/feedforward, checked against Python packing.
- Thirteen fake-server cases passed: normal, oversized/truncated/header/hash
  errors, repeated sequence, NaN, stale/future/zero timestamp, disconnect,
  silence and150ms simulated processing delay. No target emitted for invalid
  input (repeat case emits only the first valid target). Report:
  reports/native_policy_state_ipc_v2.json, backed up to Windows work/walk40.
- Tested source/header code compiled with warnings-as-errors on253, independent
  of deployed CAN binaries. No motor enable, CAN TX or sensor configuration.
- Not complete: full-loop integration with ONNX/history, protection/ramp,
  profile/watchdog and pre-send IMU freshness; config/hash provenance checks;
  backpressure and latched-fault recovery tests; end-to-end recorded parity,
  no-actuation live shadow and soak. Existing live walking launcher unchanged.

### Integrated offline sensor-to-target chain, 2026-10-13

- Extended sprite_policy_replay with explicit SPCHN001 offline fixture mode;
  retains original SPPOL001 mode. Integrates motor/IMU conversion, term-major
  observation history, ONNX CPU actor, previous blended action, affine targets,
  projection and physical startup ramp in one C++ process.
- verify_native_policy_chain.py rebuilds configuration from contract, hardware,
  limit report and saved actor report. Validates observation layout, joint/motor
  order, and refuses enabled clamp watchdogs (not yet implemented natively).
- Two actual walking recordings,2001 frames each, passed both Python recurrence
  parity and direct comparison with original final target traces. Native actor
  raw action error0 on both; final protected target error1.38778e-17 and
  5.55112e-17. Python rebuilt observations match recorded observations exactly.
- Reports native_policy_chain_155525_v2 and native_policy_chain_155850_v2
  include contract/hardware/limits/report/trace/model hashes and are backed up
  to local work/walk40. No new hardware samples, CAN TX or enable performed.
- Reproduction: bash tools/verify_native_policy_chain_recordings_on_253.sh
  after offline build. Output directories must not already exist (preserve evidence).
- Scope: command values come from recordings. This is NOT live IPC validation,
  command-profile timing validation, a closed-loop simulation or a runtime soak.
  Offline timing includes fixture IO and is not a control-deadline guarantee.
- Next integrate live state/IMU IPC with this qualified compute path, profile,
  metadata provenance and before/after inference freshness checks. Fake-peer
  end-to-end faults must pass before hardware zero-gain shadow; powered runtime
  replacement still requires fresh onsite authorization. Live scripts unchanged.

### Paced native full compute/IPC loop tests, 2026-10-13

- User authorized real-time loop testing. Implemented separate SPRITE_LOOP_TEST
  build of the qualified chain, linked with native IMU worker and state/target
  IPC. Test name hashes intentionally differ from real motor/joint names.
  No CAN, I2C, enable, sensor writes or deployed launcher changes.
- Fake state producer sends recorded motor feedback at50Hz. Fake IMU producer
  publishes quantized BNO-compatible samples near100Hz; independent native
  receiver polls200Hz. Commands remain recorded, not live joystick/profile.
- Native loop consumes latest IMU, runs complete ONNX/history/protection/ramp,
  rechecks BOTH consumed IMU age and current receiver health before send, and
  checks source-state age. Used inputs/output/timings are saved for replay.
- Healthy155525 fixture:2001frames/40.0894s, roundtrip median1.0664ms,
  P991.9128ms/max2.4528ms. Report native_policy_realtime_v1.
- Healthy155850 fixture:2001frames/40.0892s, roundtrip median1.1407ms,
  P992.0347ms/max2.8769ms; zero roundtrips>=20ms. Final IPC five target vectors
  exactly match computed output. Report native_policy_realtime_none_v2.
- Both live-paced outputs are byte-identical to independent offline replay
  using the exact consumed motor/IMU/command inputs. These are not comparisons
  against original walk targets because asynchronous IMU sampling is different.
- Injected IMU freeze exits in43.6ms after injection, state disconnect in25.9ms;
  expected nonzero exit, no continued loop. Reports native_policy_realtime_
  imu_freeze_v2 and state_disconnect_v2. Exit delays include cleanup; not actuator
  shutdown latency. Native controller's existing motor watchdog is not exercised.
- All reports/fixtures backed up in local work/walk40. Build with
  tools/build_native_policy_loop_test_on_253.sh; suite script
  tools/test_native_policy_realtime_suite_on_253.sh refuses reused output dirs.
- LIMITATION: these are two40s paced fake-peer CPU tests, not full hardware
  shadow, hard real-time proof, concurrent ankle500Hz CAN load, or long soak.
  Next prepare qualified real-name config export/live shadow integration and
  command-profile semantics. Do not substitute this test binary for powered use.

### Real hardware zero-gain baseline gate, 2026-10-13 17:18

- User requested zero-gain shadow. Read-only preflight found no active control
  processes, four kcan interfaces UP/FD1M/5M with zero error counters and native
  CAN scheduling capability present. No reset/reconfiguration performed.
- Ran existing probe_sprite0825_native_bno_full_shadow_on_253.sh for10s/one
  round. IMPORTANT: native BNO and native CAN, but Python policy. This is the
  hardware baseline, NOT acceptance of the new native policy on real hardware.
- Native transport report native_policy_ipc_transport_20261013_171843.json:
  passed,31motor feedback coverage100%,5000state ticks,0deadline misses,
  lateness P990.016875ms/max0.022552ms. Nonzero gain/torque TX attempts0,
  automatic enable0. Mixed-rate zero-gain polling only.
- Python actor500states/500targets; native receiver499targets/500states.
  Existing end-boundary discrepancy remains unresolved; do not call this
  end-to-end zero packet loss. Native target age max0.790166ms.
- Actor processing mean2.12546/P993.11913/max7.84080ms. BNO producer2800
  samples,0skips, gap max11.8412ms/read max9.81876ms. No configuration writes.
- Static pose produces497 right_ankle_pitch target clamps out of500, maximum
  overshoot0.198579rad. No limit changes; shadow computed targets were NOT
  applied to motors. Preserve observation for later live-pose interpretation.
- Backed up native_bno_full_shadow_SFEF3ca3 and actor/transport/trace171843 to
  Windows work/walk40. Test processes finished normally.
- Still required: qualified real-name configuration export and native policy
  hardware-shadow launcher. Fake-name test binary cannot be used directly.
  No powered native-policy test or live walking entrypoint replacement yet.

### First all-native real-feedback shadow accepted, 2026-10-13 17:31

- Added isolated SPRITE_SHADOW_LOOP actor build and
  tools/run_native_policy_hardware_shadow.py. Launcher checks active hardware
  processes, verified config/model hashes, recorded mapping versus native motor
  order, and valid read-only BNO packets before CAN start. Stand command only.
- Actor computes complete targets but shadow build explicitly zeros IPC
  velocity/Kp/Kd/feedforward. Existing native CAN runs ZERO_GAIN_NATIVE_SHADOW,
  no enable. Full computed outputs are saved separately and replayed offline.
  This does NOT validate nonzero native target safety in powered operation.
- First attempt1hjh4taz: first-state100ms poll raced CAN100ms initial delay.
  Separated first-state wait1000ms from unchanged running100ms receive timeout,
  source-age100ms and consumed-IMU30ms guards. No stale samples admitted.
- Second attempt2xtith3n: actor completed500frames, closed before CAN's last
  slot, causing explicit disconnect failure. Added bounded200ms peer-close
  wait only after the fixed500-frame shadow run. Mid-run disconnect remains
  an error; unexpected further state data at completion also fails.
- Third attempt native_actor_hardware_shadow_raf25b3d passed: actor/CAN/IMU
  exit0;500computed targets and500received targets;10.001543s transport;
  no deadline misses, maxlateness0.071140ms.31motor RX/TX counts match,
  ankle motors5000each at500Hz, remaining motors500each at50Hz.
- Native actor maxcompute1.46633ms, max source age1.47129ms, max consumed IMU
  age9.37799ms. Exact used-input offline replay byte-matches native outputs.
  CAN nonzero gain/torque TX0, enable0, mode switch0, previews also zero gain.
- All three attempts backed up to Windows work/walk40, including failed logs.
  No powered launcher, CAN binary/capability, gains or motor zeros changed.
- Scope limitation: native joint safety envelope in CAN is absent for this
  all-zero-IPC diagnostic; computed actor projection remains active. Restore
  full independent safety validation before any powered integration. Also
  pending long soak, command-profile/heading telemetry migration and normal
  production shutdown semantics beyond fixed-count diagnostic completion.

### Fixed BNO startup and bounded soak (2026-10-13)

- User confirmed IMUPLUS/P0 as fixed startup configuration, not a per-boot
  approval question. Hardware shadow launcher now configures Linux I2C7/0x28
  before starting IMU/CAN/actor processes, verifies register readback and
  requires 20 consecutive valid samples with fusion status5 within5s.
- Initialization does not level the measured attitude or change motor zeros.
  The native reader remains read-only; runtime configuration changes still
  cause a fault, never an automatic reconfiguration during control.
- This change currently applies to run_native_policy_hardware_shadow.py;
  existing powered launchers have not been modified.
- Launcher accepts integer duration1..120s (default10), retaining native CAN
  duration bounds. Each110s round computes5500frames and replays exactly the
  consumed real inputs offline for byte-exact comparison.
- Pre-change reboot startup failure5m0bt_67 was backed up locally. No CAN
  started in that failed attempt. Long soak results are recorded below.
- All five110s rounds passed existing gates: 1swlilkh,30jj5cot,z9tmw_qu,
  6sngo3yp,4_jx9tod. Total550s/27500frames, all offline byte-exact; all
  actor/CAN/IMU exits0, enable0/nonzero gain-or-torque TX0. These are repeated
  bounded rounds, not one uninterrupted550s process.
- CAN deadline misses0, target coverage100% throughout; maximum scheduling
  lateness0.157301ms. Policy compute maximum7.86032ms, consumed motor-state
  age maximum16.7206ms, IMU age maximum25.6724ms (30ms guard unchanged).
- IMU skipped16 polling slots across five125s producer recordings; maximum
  gap20.0699ms. Host timestamps do not measure internal sensor fusion latency.
- First three rounds RX/TX exact. Round4 ankle RX shortfall8/6/8/5 out of
  55000each; round5 shortfall11/14/11/13. Other27motors exact. Minimum ankle
  coverage99.9745%. Do not claim lossless transport; aggregate counters alone
  cannot distinguish end-boundary outstanding replies from runtime loss.
- All evidence backed up under Windows work/walk40. No matching native
  hardware processes remained after completion. This is zero-gain stability
  evidence only, not authorization/qualification for powered walking.

### BNO sampling-gap investigation (2026-10-13)

- Added per-sample host timestamps, wake lateness, I2C read duration, health
  check duration, skipped slots, thread CPU time and context-switch counters
  to native reader diagnostics. CSV is buffered in memory and written only
  after capture. Production binary was NOT replaced; diagnostic binary is
  sprite_imu_bno_diagnostic. No CAN, motor enables or IMU writes in these tests.
- Linux device-tree c250000.i2c clock-frequency is400000Hz; runtime PM is auto.
  These facts do not prove actual wire timing or that runtime PM causes gaps.
- First60s standalone:6000samples,0skips,maxgap10.3245ms,maxread6.74756ms.
- IPC180s trace bno_timing_ipc_20261013_181036:17999samples,1skip.
  At sequence110 gap20.126394ms followed previous I2C read14.916181ms,
  previous wake lateness0.061538ms, health check0ms, skipped_slots1.
  Thus the hypothesis that this particular event was caused by nine health
  reads is disproven. The scheduler intentionally advances to the next future
  10ms slot after the slow read, explaining the approximately20ms start gap.
- Second IPC180s bno_timing_ipc_20261013_181634:18000samples,0skips,
  maxread7.179029ms with thread CPU0.202112ms,2voluntary/1involuntary switches.
  This establishes substantial waiting for that7ms read, not the cause of
  the earlier15ms event (which predates CPU instrumentation).
- Bosch datasheet explicitly documents I2C clock stretching:
  https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
  Stretching is a candidate, NOT a confirmed diagnosis of this device.
  Separating driver/device waiting from runnable scheduling delay requires
  kernel scheduler/I2C trace or an SCL/SDA logic-analyzer capture. Kernel
  sched_schedstats was0; no sysctl, power or priority tuning was applied.
- Tools: diagnose_bno_timing_no_can.sh and analyze_bno_timing.py. CSVs/reports
  backed up to Windows work/walk40. Consumer deliberately pauses150ms once;
  sequence jump15 during that pause is not a producer gap. No ONNX model was
  passed in these runs, so the consumer's inference timing label is NOT an
  inference benchmark. No claim of a complete fix or sensor-fusion latency.

### Kernel runqueue accounting (2026-10-13)

- User enabled kernel.sched_schedstats=1 explicitly. Native diagnostic now
  reads /proc/thread-self/schedstat around each sample read, with counters
  recorded alongside CPU time. Report marks whether schedstats was enabled;
  analysis must not treat disabled counters as zero scheduling delay.
- bno_timing_ipc_20261013_182607:180s,18000samples,0skips. Slowest read
  7.108656ms, thread CPU0.213504ms, runqueue wait0.018304ms. Ten slowest reads
  had runqueue delay0.017536..0.034144ms. Wake lateness maximum0.483212ms.
- These captured slow reads are dominated by blocking/waiting, not this
  thread waiting runnable for CPU. This does NOT identify I2C device clock
  stretching versus driver/IRQ/runtime-PM delays; also the prior14.916181ms
  extreme was not reproduced in this run. Do not claim it ruled out all
  kernel scheduling causes or that raising priority fixes the issue.
- Tracefs requires root and SSH sudo -n is unavailable. Added read-only
  tools/inspect_bno_kernel_trace.sh to discover sched/I2C tracepoint formats
  before selecting narrow filters. No tracing configuration changed yet.
- Production IMU binary/control gains unchanged; only diagnostic binary
  rebuilt. User must restore kernel.sched_schedstats=0 after investigations
  (no sudo credentials available to this session). Evidence backed locally.

### First kernel-event capture analysis (2026-10-13)

- bno_kernel_gw6orbur:180s/18000samples,0skips,420501trace events,
  all per-CPU overrun/dropped counters0. IMU/consumer exited0, no CAN TX.
- Worst read sequence8603:7.248827ms total,CPU0.229760ms,runqueue0.029056ms.
  Relative to read start: I2C read event0.005ms; blocked(D)0.028ms;
  first IRQ0.092ms; reader wake0.572ms/run0.592ms; blocked again0.653ms;
  next IRQ6.756ms; reader wake7.092ms/run7.112ms; result7.221ms.
  The longest observed interval is6.103ms blocked before the second IRQ.
- I2C threaded IRQ PID150 (irq/168-c250000), CPU0, SCHED_FIFO priority50.
  Reader wake-to-run is short; raising reader priority is not supported as
  the primary fix by this capture. The14.916ms rare event was not reproduced.
- No selected device_pm_callback events appeared; this does not rule out all
  runtime-PM cost or callbacks elsewhere. Device/driver/IRQ delivery remain
  candidates. First trace omitted IRQ-thread scheduler events; capture script
  now discovers its PID and includes it in sched_switch/sched_wakeup filters.
- Added analyze_bno_kernel_trace.py; event windows in local evidence directory
  kernel_analysis.json. Production binaries/gains unchanged. Next refined
  trace requires user sudo invocation; not yet run.

### Refined IRQ scheduler capture (2026-10-13)

- User completed bno_kernel_fsaepmmd:180s,18000samples,0skips,
  539644events, no trace overruns/drops. PID4348 reader and PID150 IRQ thread
  both included. Evidence and kernel_analysis.json backed up on Windows.
- 39439 matched IRQ wake-to-run intervals: p99=0.015ms,max=0.105ms.
  Worst read sequence3290 total7.033984ms,CPU0.145952ms,reader runqueue0.014624ms.
  First IRQ thread wake/run0.099294/0.104294ms, second6.715294/6.723294ms.
  Reader blocked at0.678294ms; second IRQ entry6.711294ms:6.033ms interval.
  IRQ thread dispatch latency does not explain the observed6ms interval.
- For captured slow reads, neither reader nor IRQ thread CPU queueing is
  dominant. Still cannot identify device stretching versus controller/driver
  or hard-IRQ delivery latency from these events alone. Earlier14.916ms
  extreme not reproduced. No priority change or claim of complete repair.
- Next useful no-CAN experiment: compare46-byte current burst with shorter
  register reads, preserving production semantics until validated. Inspect
  driver transfer path if length dependence emerges. No repeated identical
  scheduler capture requested. sched_schedstats may now be restored to0.

### Read-length comparison (2026-10-13)

- Independent native bno_read_length_benchmark compiled on Jetson with
  -O2 -Wall -Wextra -Werror. No CAN, no IPC targets, no configuration writes.
  Shared native BNO lock prevents another native reader. Health checked at
  block boundaries; this is a controlled transfer test, not full-runtime soak.
- Eight15s blocks:46,32,26,23+23,23+23,26,32,46bytes. All quaternion norms
  valid,12008samples, no reads over10ms or skipped slots. Source data saved
  as reports/bno_lengths_round1.csv and backed up to Windows work/walk40.
- Combined median/P99/max ms:46bytes2.799/6.080/6.685;
  32bytes2.006/5.644/6.227;26bytes1.809/5.454/6.242;
  split23+23bytes2.969/6.184/7.229. Forward/reverse blocks broadly agree.
- 26bytes at0x14 includes gyro and quaternion but omits parts of the current
  full packet.32bytes starts0x08; split reads cover the full46bytes but can
  straddle sensor updates. Neither is a validated drop-in production change.
- Shortening improves typical transfer time but does not remove the5..6ms
  tail. No evidence here that a sharp length threshold or splitting solves
  the rare14.916ms call/20ms sample gap. Production read/30ms guards unchanged.
- Local /usr/src has kernel headers but no Tegra I2C driver source found.
  No speculative priority/DMA/power tuning applied. Further extreme-event
  attribution needs capture under full native workload, rather than repeated
  unloaded tests being treated as proof of a fix.

### Retraining observation decision (2026-10-14)

- Before any new training design, read
  [IMU_OBSERVATION_RETRAINING_DECISION.md](IMU_OBSERVATION_RETRAINING_DECISION.md).
- Confirmed: our actor and inspected PM01/G1 public examples use gravity
  projection, with real deployment deriving it from estimated IMU attitude.
- Compare the preserved baseline against matched six-axis estimation in
  simulation/deployment, then raw specific-force plus gyro/history training.
  Do not replace observations in existing weights or assume this is the
  current asymmetric-gait root cause. No hardware behavior changed.
- Keep sensor acquisition tail-latency diagnosis independent of that proposal.

### Current-policy software IMU candidate (2026-10-14)

- User chose to keep existing policy and estimate gravity from BNO raw accel/
  gyro ourselves; actor gyro must be raw minus bias, then mounted-body rotation.
- See [BNO055_SOFTWARE_FUSION.md](BNO055_SOFTWARE_FUSION.md) for reference,
  fixed raw-mode settings, bias gates, protocol and isolated253 probe commands.
- No production walking script/weights/gains switched. Hardware qualification
  and full native zero-gain validation remain prerequisites to powered use.
