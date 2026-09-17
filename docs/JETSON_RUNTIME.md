# Jetson Runtime

The Sprite0825 runtime on the Jetson Orin Nano uses the system-provided
PyTorch/TensorRT stack and the contract-pinned TorchScript actor when ONNX
Runtime is unavailable. ONNX Runtime remains the preferred desktop backend.
The replay report records the selected backend.

Run commands through the project-local launcher:

```bash
./run_sprite_runtime_jetson.sh inspect \
  --contract <candidate>/deploy/contract.json \
  --runtime-config config/runtime.example.json \
  --hardware-config config/hardware.sprite0825.measurement.json
```

The launcher does not open CAN devices by itself. CAN interfaces must remain
down until the receive-only preflight and all hardware calibration gates pass.

Jetson systems without a battery-backed RTC may boot with an invalid clock.
Hardware operation must not proceed until `date` is credible. NTP should be
enabled, and the operator must verify synchronization after each cold boot.
The Jetson launcher rejects a system year earlier than 2025.

The validated Jetson environment on 2026-09-15 used Python 3.10.12, the
NVIDIA-provided PyTorch 2.3.0 build, TensorRT 10.3, NumPy 1.21.5, and
python-can 4.5.0. A full 60-second G74 shadow replay ran 2,998 policy ticks
with TorchScript action error below `1.2e-6`; no CAN interface was opened.
