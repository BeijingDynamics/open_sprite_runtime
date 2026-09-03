# Timing contract

## Initial deployment rates

| Layer | Rate | Responsibility |
|---|---:|---|
| Damiao internal loop | 1 kHz | Motor current/MIT impedance loop |
| SBC state and setpoint layer | 500 Hz | CAN RX aggregation, safety, ankle transform, held-target refresh |
| Deployment policy | 50 Hz | Build deployment observation and run ONNX actor |

Each 50 Hz policy target is held for ten 500 Hz state-layer ticks. This
zero-order hold matches Isaac Lab and the qualified MuJoCo runtime. The 500 Hz
layer must not linearly interpolate between policy targets. It may repeatedly
send the held position/velocity/Kp/Kd target and calculate the differential
ankle's cross-coupled feed-forward torque. The motor's 1 kHz loop executes two
internal cycles per SBC tick.

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
