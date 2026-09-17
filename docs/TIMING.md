# Timing contract

## Initial deployment rates

| Layer | Rate | Responsibility |
|---|---:|---|
| Damiao internal loop | 1 kHz | Motor current/MIT impedance loop |
| SBC state and ankle-PD layer | 500 Hz | CAN RX, safety, ankle joint PD, `A^-T` motor torque, ankle MIT `tau_ff` |
| Deployment policy and other motors | 50 Hz | ONNX actor and position/velocity/Kp/Kd updates for the other 27 motors |

Each 50 Hz policy target is held for ten 500 Hz state-layer ticks. This
zero-order hold matches Isaac Lab and the qualified MuJoCo runtime. The 500 Hz
layer must not linearly interpolate between policy targets. For each ankle it
computes pitch/roll joint PD torque from fresh feedback, applies the calibrated
linear transmission Jacobian `tau_motor = A^-T tau_joint`, and sends both ankle
motors with MIT `Kp=Kd=0` and feed-forward torque. The other 27 motors receive
position/velocity/Kp/Kd updates only at 50 Hz and use the motor's internal PD.
The motor's 1 kHz loop executes two internal cycles per ankle-control tick.

The current ankle transform is deliberately the measured constant linear map.
Replacing it with a configuration-dependent linkage Jacobian is a later
refinement and must preserve the same virtual-work and limit tests.

The qualified `model1050` baseline uses 15 frames at 100 Hz, covering 150 ms.
Calling that actor at 50 Hz would change the window to 300 ms and is forbidden.
The deployment actor is trained natively at 50 Hz with 8 frames, covering
160 ms. It must be qualified independently in Isaac Lab, MuJoCo, and shadow
logs before hardware use.

## SBC choice

Jetson Orin Nano is the lower-risk first bring-up controller because it gives
more inference and logging margin. Raspberry Pi 5 may be sufficient for this
MLP with ONNX Runtime, but it must first demonstrate worst-case inference time,
500 Hz state-loop jitter, and USB-CAN FD round-trip latency under full logging
and thermal load. Neither board should be assumed real-time without measurement.

The motor's internal 1 kHz loop does not make a USB host loop deterministic.
If the four-channel USB-CAN FD bridge cannot meet the measured state-age and
watchdog requirements, add a dedicated real-time CAN gateway rather than
changing the qualified policy timing after training.

The 500 Hz state layer timestamps and validates gyro samples and maintains the
integrated yaw estimate. The PM01-style heading controller runs once per 50 Hz
policy tick and writes only the existing yaw-rate command observation. Standing
may reset the heading reference, but must not reset policy history or fabricate
joint/IMU samples.
