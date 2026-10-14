# Native walk40 readiness, 2026-10-14

## Preserved baseline

The three visually accepted walks use native software IMU at 200 Hz, Python
ONNX inference at 50 Hz, and the existing native CAN controller. Preserve
their model, gains, limit projection, calibration and motor disable behavior.
No powered test was performed during this migration step.

## Implemented and verified

- Added optional native monotonic state-timestamp command profiles identical
  to command_schedule.py: stand 10 s, ramp 3 s, walk 14 s, stop ramp 3 s,
  final stand 10 s. The initial state sets time zero after IMU calibration.
- Compared both speed profiles at every millisecond: 80,002 samples, zero
  velocity difference and identical stage labels. Invalid/backward times fail.
- Added final five-second countdown, retaining the existing Python semantics.
- Fake-peer 40 s run: 2,001 frames; output byte-identical to offline replay,
  IPC targets equal computed targets. Round trip median 1.082 ms,
  P99 2.110 ms, maximum 2.581 ms, no 20 ms overruns.
- Fake IMU freeze and state disconnect cause actor exit (approximately
  61 ms and 28 ms respectively in these runs). These are process-exit delays,
  not measured motor-disable latencies.
- Evidence: reports/native_walk40_fake_OPjtfQ42 on Jetson; Windows backup
  under work/walk40. Fake IMU uses the existing V1 test packet; this does not
  replace the earlier V2 software-IMU full-load zero-gain qualification.

## Remaining before powered native inference

- Native actor still uses a finite replay fixture and shadow completion
  handshake. It is NOT a powered runtime entry point. Real-name IPC targets
  remain forced to zero gains; fake-peer targets use non-robot name hashes.
- Integrate controller-owned completion, preserving the final standing period
  and verified disable. Native CAN currently publishes a preparation state,
  then periodic states while active; unexpected disconnection must remain a
  fault. A fixed frame count alone must not authorize successful shutdown.
- Trace writes now use preallocated bounded buffers, saved after the loop.
  Faults close IPC before saving partial evidence; overflow and save failure
  are tested. Latest IMU freshness checks and actual dt remain unchanged.
- V2 software-IMU zero-gain qualification with the command schedule passed
  (details below). Powered launcher integration still requires separate
  qualification and fresh onsite approval for powered A/B.
- Do not modify motor zero, torque limits, gains, model or observation layout
  as part of this migration. Do not advertise powered readiness yet.

## Completed follow-up

- Controller completion wait increased from 200 ms to 1000 ms. The powered
  CAN controller's disable/verification path takes about 240 ms before report
  generation; the former timeout was insufficient for reuse. This is only
  transport completion, not proof of successful motor disable. Supervisor
  must check controller exit and all final_disabled fields in the report.
- Fake completion tests: delayed EOF at 350 ms passes; extra data and silence
  fail. Final fake walk/fault regression: native_walk40_fake_ed49UNVe,
  2001 frames, byte-exact parity, max roundtrip 3.480185 ms.
- User confirmed supported robot, powered but disabled motors and CAN ready.
  Ran 40-second zero-gain with native software IMU and native policy:
  reports/native_actor_hardware_shadow_2ymynwb6, passed=true.
- 2000 frames, byte-exact offline parity; compute max 1.17012 ms,
  consumed state max age 1.18583 ms, consumed IMU max age 13.0392 ms.
  These are host timestamps, not physical sensor latency measurements.
- CAN deadline misses 0, maximum lateness 0.180208 ms, target coverage 1.0.
  Enable attempts 0, nonzero CAN gain/torque attempts 0. Actor/CAN/IMU exits 0.
- Reports backed up to Windows work/walk40. Production powered Python
  launcher and CAN executable were not replaced; no powered walking done.

## Protected IPC candidate

- Added a separately compiled SPRITE_POWERED_LOOP actor candidate. It requires
  ENABLE_NATIVE_ACTOR_WALK40, the fixed 0.10 m/s profile, a distinct manifest
  tag and exactly 2001 states (preparation plus 2000 active states). The
  zero-gain build remains separate and still forces gains/torque to zero.
- The candidate was exercised ONLY against fake UNIX peers with fake motor
  names and V2 IMU packets. No CAN access or motor enable occurred.
- reports/native_walk40_fake_FUoLPorz: complete 40 s plus simulated 350 ms
  shutdown confirmation, 2001 frames, byte-exact offline parity, maximum IPC
  roundtrip 2.287474 ms, no 20 ms overruns. IMU freeze and disconnection fail.
- check_native_powered_completion.py checks process exits, controller mode,
  duration, deadlines, error list, exact motor set and all 31 final_disabled
  fields. Eleven positive/negative report-gate cases pass. It is not yet
  integrated into a real powered launcher, and is not an execution permit.
- Build initially found the main remote source still had the V1-only IMU
  client header. Synced the previously validated V2-capable header and rebuilt;
  the active walking binary and scripts were not replaced.
- Remaining work: prepare and validate the immutable configuration bundle,
  connect this actor to the existing preflight/launcher and wire the completion
  gate there. No powered entry script is offered until those checks pass.

## Isolated admission entry prepared

- Added native_walk40_adapter.py: validates current model/contract/hardware/
  limits and all shell gain/ramp arguments against the accepted 131201 run
  and its parity evidence. Unknown options fail instead of silently dropping
  unsupported watchdogs. Inference runs entirely in the C++ child; Python
  only prepares, launches and collects its exit status.
- Per-run bundle freezes actor input header, name order and the three native
  CAN configuration TSVs. Hashes also cover the policy and executables.
  Preparation occurs before starting the powered CAN controller. Hashes are
  rechecked before opening the actor IPC connection.
- prepare_native_walk40_entry.py derives separate candidate scripts from
  the existing baseline without modifying it. The final report gate is wired
  after both processes exit. Existing preflight and native safety flags stay.
- Entry on253: run_sprite0825_full_native_walk40_candidate_on_253.sh.
  --check is static-only; ENABLE_NATIVE_ACTOR_WALK40 permits the powered path
  and must only be used after fresh onsite safety confirmation.
- Tests passed: bundle creation, changed gain rejection, changed bundle
  rejection, generated Bash syntax, --check, and evaluation of actual shell
  parameters against the reference settings. Tests use fake sockets and stop
  before hardware preflight. Native powered-candidate fake-peer tests above
  cover inference and IPC shutdown, not actual motor-disable effectiveness.
- Ready for a first supervised powered A/B, NOT yet physically qualified.
  No powered test has been run with this entry. Binary actor trace is stored
  in <actor-report>.native/actor.bin; it is not the old Python NPZ format.

## First authorized powered run: 20261014_134512

- User explicitly confirmed fall arrest, safety operator, clear area and power.
  Executed the candidate once. Static IMU calibration and zero-gain preflight
  passed; all command stages through final stand and countdown completed.
- CAN report passed, exit 0, 40 seconds, 31/31 final_disabled=true,
  deadline_misses=0, maximum lateness 0.254202 ms. Maximum reported MOS/rotor
  temperatures 37/32 C. Actor recorded 2001 frames, compute max 1.67444 ms,
  consumed IMU max host-read age 13.8696 ms.
- Overall launch FAILED: actor exit 1 at controller completion. Preserve the
  failed result; do not relabel this as fully qualified. Native CAN consumed
  2000 targets and sent 2001 states (including preparation/final state).
- Fake socket reproduction: a peer that closes with the final target unread
  produces recv=-1, errno=104 (ECONNRESET). This matches the observed count
  mismatch and is the likely cause; original generic error did not capture
  errno, so direct errno evidence for the powered run is unavailable.
- Candidate now permits terminal ECONNRESET only after completing all expected
  powered frames. Active-loop disconnects, extra bytes, silence still fail;
  supervisor still requires the successful CAN report and all31 disabled.
  Five close-boundary tests passed and candidate rebuilt. No second powered
  test executed; awaiting visual feedback and fresh authorization to retest.
- All actor binary evidence, CAN report and IMU report saved on Windows in
  work/walk40. Policy/gains/torque caps and CAN executable unchanged.

## Subsequent operator validation and startup diagnostics

- After the completion fix, the operator reported many successful walking trials
  on October 14. This is repeated supervised visual validation of the current
  candidate, not a claim that every subsequent report has been audited. Preserve
  the first run's failed overall exit result above as development history.
- Three later preflight refusals (135221, 135303, 135359) were due to maximum
  horizontal projected-gravity norm 0.110174, 0.109943, 0.121326 exceeding 0.1.
  Joint target projection was advisory-only and did not cause those refusals.
- The analyzer now prints failure reasons and relevant joint measured/target/
  projected ranges and limits to stderr, even when shell callers hide JSON
  stdout. An offline replay of 135359 verified this behavior. Safety limits,
  policy weights and motor gains were not changed by this diagnostic fix.
- Keep the current candidate as a repeatable supervised walking baseline.
  Gamepad-controlled repeated starts/stops and turning remain separate validation
  steps; this release does not automatically enable either.
