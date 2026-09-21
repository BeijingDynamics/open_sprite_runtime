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
