# Runtime artifacts

Do not commit an arbitrary policy here. `model1050` is the qualified 100 Hz
teacher/baseline, not the 50 Hz hardware actor. The runtime must consume a
separately qualified 50 Hz ONNX policy and matching contract, verify both
SHA-256 values, and record those values in every hardware log.
