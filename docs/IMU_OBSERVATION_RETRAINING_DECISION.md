# IMU observations: decision for the next retraining

Recorded: 2026-10-14 (Asia/Shanghai).
Status: research conclusion and proposed controlled experiments, NOT a
production observation change or authorization to train/enable hardware.

## Decision

Treat the difference between simulation gravity truth and real estimated
gravity as an explicit sim-to-real concern in the next training design.
Do not assume it explains the present left drift or asymmetric gait.
Keep the qualified policy and runtime as a baseline. Do not feed acceleration
into existing weights in place of projected gravity, even if dimensions match.

The user proposes feeding accelerometer specific force directly to a newly
trained policy. This is a valid experiment, not an established improvement.
First compare it against a matched simulation/deployment six-axis estimator.
Do not change the motion data, reward design, and observation design together.

## Verified implementation and public references

- Our saved training configuration uses mdp.projected_gravity, obtained from
  simulated orientation, with additive observation noise. See local workspace
  work/stage2_src/sprite0615_amp_locomotion/config/sprite0615/flat_env_cfg.py.
  This is not a simulated accelerometer. That saved configuration has older
  history settings; it must not override the active deployment contract.
- Current native policy_core.hpp uses 8 history frames and 795 inputs:
  31 joint positions, 31 velocities, 31 previous actions, 3 angular velocities,
  3 projected-gravity components per history frame, plus 3 current commands.
  policy_feedback.hpp derives gravity from the mounted IMU quaternion.
  Neither quaternion nor acceleration is directly an actor input.
- EngineAI PM01 AMP public training uses projected gravity with uniform
  noise [-0.05, 0.05] and observation history. Its public SDK walking runner
  reads IMU quaternion, corrects installation orientation and computes
  -R_world_body.transpose() * UnitZ for the policy.
- Unitree's public G1-compatible RL deployment reads IMU quaternion and gyro,
  then calls get_gravity_orientation(quat) for its observation.
- These public examples do not establish the manufacturers' complete deployed
  estimator algorithms, sensor latency, or all commercial policy designs.
  Independent additive noise on gravity truth is not a faithful model of
  correlated estimator error, dynamic tilt error or latency.

Sources inspected on 2026-10-14 (moving branches; pin commits for experiments):

- https://github.com/engineai-robotics/engineai_rl_lab/blob/main/source/engineai_rl_lab/engineai_rl_lab/tasks/locomotion/amp/amp_env_cfg.py
- https://github.com/engineai-robotics/engineai_robotics_native_sdk/blob/main/src/runner/rl_walking_example/src/rl_walking_example_runner.cc
- https://github.com/engineai-robotics/engineai_robotics_native_sdk/blob/main/src/runner/rl_lab/src/rl_lab_runner.cc
- https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/deploy_real.py
- https://isaac-sim.github.io/IsaacLab/develop/source/concepts/sensors/imu.html

## Physical contract

With R_world_body mapping body vectors into world coordinates:

    projected_gravity_b = R_world_body.T * [0, 0, -1]
    specific_force_b = R_world_body.T * (a_sensor_world - gravity_world)

The second expression assumes the sensor measurement has already been
transformed into body axes. Apply calibrated mounting rotation, bias and scale
consistently. A supported upright stationary sensor gives approximately
[0, 0, +9.81] m/s^2, whereas projected gravity is [0, 0, -1]. Free-fall
specific force is approximately zero. Do not confuse specific force with a
fusion module's gravity-removed linear-acceleration output.

A fixed sensor-to-body mounting transform needs only a constant rotation
matrix. Estimating time-varying gravity is a separate problem. One acceleration
sample cannot uniquely distinguish tilt from translation; temporal and joint
information help but do not guarantee complete observability.

Pure yaw error does not change projected gravity. Do not infer that leftward
walking proves a gravity-estimation fault. Roll/pitch bias, gyro errors,
mechanical asymmetry and other causes still need independent evidence.

## Proposed comparison

1. A: preserve the current gravity-input baseline and its exact artifact.
2. B (preferred conservative improvement): simulate accelerometer and gyro,
   run the SAME causal six-axis estimator intended for deployment, and feed
   its estimated gravity and gyro to the actor. Match startup and timing too.
3. C (user proposal): train an actor using specific force, gyro, joint states,
   previous actions, commands and history, without true gravity as an input.
   Evaluate history length or recurrent estimation rather than assuming the
   present 8-frame history is sufficient. Retain acceleration magnitude;
   any scaling/clipping must be explicit and identical at deployment.

Keep actor horizontal base velocity excluded. Privileged simulation state
may be used by a critic or training-only teacher, never leaked to the deployed
actor. Teacher/student distillation is optional, not a requirement to add
another training stage. Preserve the established locomotion method.

## Before retraining

- [ ] Pin baseline model, source revisions, robot asset, joint order, PD and
      parallel-linkage mapping, observation contract and evaluation commands.
- [ ] Use the actual pelvis IMU location/orientation. Include angular and
      centripetal acceleration at the offset sensor, not only COM acceleration.
- [ ] Validate simulated specific force in static tilt, free fall and known
      motion; verify signs/units and reset derivative history correctly.
- [ ] Model measured sensor bias, scale/misalignment, bandwidth, vibration,
      sampling/hold, latency and occasional missing samples. Distinguish host
      receive timestamps from physical measurement time. Do not double-count
      gravity or apply a second P0/mount remap.
- [ ] Preserve the 50 Hz deployment policy contract unless separately changed;
      model sensor updates independently. Validate the installed Isaac Lab
      version rather than copying develop-branch APIs blindly.
- [ ] Compare under matched training budgets and multiple seeds. Evaluate
      stand/start/stop/restart, turns, perturbations, foot clearance, left/right
      symmetry, command tracking and actuator demand, not just reward.
- [ ] Test sensitivity to tilt bias and observation delay in simulation.
      Pass sim2sim and disabled-motor shadow checks before a separately
      authorized, protected physical comparison.

## Separate unresolved acquisition issue

Do not equate BNO fusion's 100 Hz output rate with a mandatory 10 ms I2C read
wait. Host polling rate is not proof that every result is a new sensor sample.
Earlier diagnostics observed a 14.916 ms read and approximately 20 ms sample
gap; shorter reads improved typical time but did not eliminate the tail.
The captured slow reads were not dominated by reader/IRQ-thread CPU queueing.
Device/controller/driver attribution of the rare event remains unresolved.

Bypassing internal fusion may reduce one source of latency, but does not
prove the transport delay is fixed. Learned estimation also does not remove
all filtering/temporal dependence. Keep latency investigation and retraining
evaluation separate. See NATIVE_RUNTIME_MIGRATION_PLAN.md for evidence.
