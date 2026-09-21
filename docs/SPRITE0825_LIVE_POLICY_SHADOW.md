# Sprite0825 live policy shadow

This gate runs the frozen G74 actor from the real 31-motor feedback and pelvis
IMU while the robot remains mechanically supported and every motor remains
disabled. It is the bridge between the qualified sensor shadow and any low-gain
actuation test.

The actor runs at 50 Hz. The exact trained observation is reconstructed as
eight oldest-to-newest frames of relative joint position, joint velocity,
previous action, body angular velocity, and projected gravity, followed by the
current `[vx, vy, yaw_rate]` command. Horizontal base velocity, global position,
and global yaw are absent.

At 500 Hz the runtime computes the held-target ankle joint PD torque and maps it
through the measured differential matrices. At 50 Hz it computes the other
motor position/velocity/gain request. These nonzero requests are audited in
memory only. The transport object used by this command can transmit only the
previously qualified position-echo frame with `velocity=Kp=Kd=tau=0`.

## First run on Jetson 253

Keep the robot supported and confirm every motor is disabled:

```bash
cd /home/tony
./setup_sprite0825_kcanfd_on_253.sh
./run_sprite0825_live_policy_shadow_on_253.sh 10 headless
```

After the headless timing gate passes, the actual measured pose can also be
displayed. The viewer is not a policy target animation.

```bash
cd /home/tony
./run_sprite0825_live_policy_shadow_on_253.sh 120 viewer
```

The initial command is deliberately stand (`0,0,0`). Forward-command shadow is
not meaningful while disabled hardware cannot follow the target and should only
be added as a separate diagnostic after this gate is understood.

## 2026-09-21 commissioning result

The first full-hardware run exposed and fixed an I/O scheduling defect. The
corrected retry received 496-500 samples per motor over ten seconds (minimum
coverage 0.990), received both IMU packet types at approximately 100 Hz, and
ran the 50 Hz ONNX actor with 0.424 ms mean and 1.43 ms P99 inference time.
All 31 drives remained disabled and the restricted writer made zero nonzero
control, enable, disable, or mode-switch attempts.

The Python process is not yet qualified as the final 500 Hz ankle execution
layer. Its best integrated result had 3.64 ms P99 and 9.24 ms maximum state-tick
lateness. A Python worker-thread experiment was rejected because GIL contention
made both CAN coverage and timing worse. Keep the 50 Hz ONNX policy in Python,
but implement the 500 Hz ankle/safety/transport loop in a native real-time
component before actuation.

The audit also rejects direct transition from the measured supported pose to
the policy target. Several distal joints exceeded their configured MIT torque
range in the calculated command, led by head yaw at 3.86 times its protocol
limit. The first actuation gate therefore requires a bounded measured-pose
handoff/ramp and command-envelope validation; direct policy enable is forbidden.

## Native mixed-rate transport qualification

The native C++ transport uses a 2 kHz absolute-time slot schedule pinned to
CPU 5. Each CAN bus emits at most one frame per 0.5 ms slot. The two ankle
motors on each leg bus occupy separate slots and each run at 500 Hz; every
other motor runs at 50 Hz. The restricted writer can encode only position echo
with velocity, Kp, Kd, and feedforward torque set to zero. It has no motor
enable, disable, or mode-switch path.

On 2026-09-21 the powered, mechanically supported robot completed both a
10-second and a 120-second four-bus native shadow test. The long run completed
60,000 state ticks with zero deadline misses, 0.060 ms P99 lateness, and
0.525 ms maximum lateness. Each ankle motor returned 60,000 samples and every
other motor returned 6,000 samples. All four SocketCAN interfaces remained
ERROR-ACTIVE with zero current TX/RX error counters, and their cumulative bus
error, drop, and bus-off counters did not change during the run.

This qualifies the native timing and zero-gain transport layer only. Nonzero
actuation remains forbidden until the measured-pose handoff, stale-target
watchdog, command envelope, and Python-policy-to-native target interface pass
their own shadow tests.

## Native policy IPC qualification

The integrated architecture keeps all four CAN sockets and the 500 Hz timing
loop in C++. It publishes a versioned 50 Hz motor-state packet to a Python
client. Python combines that state with the 100 Hz pelvis IMU, reconstructs the
exact 795-value observation history, runs the frozen ONNX actor, and returns a
versioned 31-joint target packet. Packets carry monotonic timestamps, increasing
sequence numbers, source-state sequence numbers, and hashes of the ordered
motor/joint names. The native side rejects stale, future, reordered, nonfinite,
or excessive-Kd targets. During this gate returned targets are audited only and
cannot reach the restricted zero-gain CAN writer.

The integrated load exposed rare 2--5 ms scheduling spikes under Linux
`SCHED_OTHER`. CPU affinity alone did not remove them. The qualified setup pins
the native thread to CPU 5 with `SCHED_FIFO` priority 50, pins Python/ONNX to
CPU 4, and gives only `CAP_SYS_NICE` to the native binary:

```bash
cd /home/tony/open_sprite_runtime
./build_sprite0825_native_runtime_on_253.sh
./install_sprite0825_native_runtime_capabilities_on_253.sh
./probe_sprite0825_native_policy_ipc_shadow_on_253.sh 120
```

Capabilities must be reinstalled after rebuilding the binary. The launcher
fails closed if `SCHED_FIFO` cannot be applied.

On 2026-09-21 the integrated runtime passed a 120-second live-hardware shadow:

- 60,000 native 500 Hz ticks, zero deadline misses;
- 0.0123 ms P99 and 0.227 ms maximum native lateness;
- 6,000 Python state packets and 6,000 inferred targets;
- 5,999 targets accepted by native (99.983% including the final shutdown race);
- 0.901 ms maximum accepted target age;
- 2.01 ms ONNX inference P99 and 5.40 ms maximum;
- approximately 100 Hz raw IMU and quaternion packets with zero rejected frames;
- exact expected CAN TX/RX counts and zero new errors or drops on all four buses.

After installing the stable `/dev/sprite0825-imu` udev path and completing the
drive register audits, the full qualification was repeated. The second
120-second run produced 60,000 native ticks with zero deadline misses,
0.0119 ms P99 and 0.424 ms maximum lateness; all 31 motors again had 100%
feedback coverage. Python completed 6,000 policy targets from 6,000 states,
accepted 12,005 IMU raw packets and 12,005 quaternions with zero rejected
frames, and measured 1.74 ms mean, 2.06 ms P99, and 7.68 ms maximum ONNX
inference time. The native side accepted 5,999 targets (99.983%, with only the
final shutdown boundary outstanding), and the maximum accepted target age was
0.861 ms. Nonzero gain/torque, automatic-enable, and mode-switch attempts all
remained zero.

This still does not authorize actuation. The measured-pose handoff and native
IPC fault injection are now qualified below; complete target-envelope audit and
replayable per-tick live traces remain required before any nonzero command path
is implemented.

## Measured-pose handoff qualification

The live policy path now implements the frozen contract fields
`deployment_handoff_seconds=0.04`,
`deployment_handoff_mode=smoothstep_from_pose_equivalent_action`, and
`deployment_initial_velocity_mode=zero`. The first measured 31-joint pose is
converted through the frozen action offset/scale, then smoothstep blended to
the policy output over exactly two 50 Hz ticks. The blended action is also fed
back into action history, matching the qualified MuJoCo runtime.

On 2026-09-21 this path passed all 145 offline tests and a 20-second powered,
mechanically-supported, all-motors-disabled live shadow run. Every motor had
100% feedback coverage; the native loop completed 10,000 500 Hz ticks with
zero deadline misses; IMU decoding accepted 2,010 raw and 2,009 quaternion
frames with zero rejected frames; and nonzero gain/torque TX, automatic enable,
and automatic mode-switch counts all remained zero.

This completes the measured-pose handoff gate only. The hardware inventory is
still intentionally non-armable: physical soft/hard limits, drive register
readback provenance, current/temperature thresholds, and the independent
hardware e-stop must be completed before any nonzero actuation test.

## Native IPC fault injection

Run the process-level fail-closed cases only while the robot is mechanically
supported and all motors report disabled:

```bash
cd /home/tony/open_sprite_runtime
./probe_sprite0825_native_ipc_faults_on_253.sh
```

The harness can only exercise the restricted zero-gain position-echo CAN
writer. It deliberately injects a wrong joint-order hash, a future source-state
sequence, a stale target timestamp, embedded `Kd=3.01`, and a client that never
sends its first target. On 2026-09-21 all five cases were rejected by the native
process with their exact expected invariant failure. A new initial-target
watchdog now stops the native loop after five unanswered 50 Hz state packets;
the existing 100 ms watchdog still covers loss after the first accepted target.

After the fault tests, a normal 10-second `SCHED_FIFO` live shadow regression
passed with 5,000 native ticks, zero deadline misses, 100% motor feedback
coverage, zero rejected IMU frames, and zero nonzero-command, enable, or mode
switch attempts.

## Replayable live trace

The policy client can now write a compressed NPZ trace alongside its summary.
Every 50 Hz tick records the native motor position/velocity packet, reconstructed
joint state, pelvis angular velocity and projected gravity, the exact 795-value
actor observation, raw ONNX action, measured-pose handoff action, and final
joint position target. The trace contains no hardware command frames.

The first 10-second capture on 2026-09-21 contained 500 complete ticks and was
replayed through the frozen ONNX actor with a maximum absolute action error of
exactly zero. The trace SHA256 is recorded both in the actor report and the
artifact manifest. A 120-second trace and per-motor command-margin analysis are
still required to close the command-envelope gate.
