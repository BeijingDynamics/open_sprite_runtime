#!/usr/bin/env bash
set -euo pipefail

ACK=ENABLE_RIGHT_ANKLE_ZERO_TORQUE_500HZ
if [[ "${1:-}" != "$ACK" ]]; then
  echo "FIRST POWERED DIFFERENTIAL-ANKLE TRANSPORT GATE: right pair only" >&2
  echo "Robot must be securely suspended, foot/linkages clear, safety operator ready." >&2
  echo "Both motors: enabled, 500Hz, Kp=0, Kd=0, torque=0, duration 2s." >&2
  echo "This command never resets motor zero positions." >&2
  echo "Run only after explicit physical confirmation: $0 $ACK" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kcan2_right_ankle_zero_torque_${STAMP}.json"
OUTPUT="$ROOT/reports/right_ankle_zero_torque_500hz_${STAMP}.json"
mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"
echo "ACTIVE ZERO-TORQUE GATE: right ankle motors kcan2 IDs 0x05/0x06 only"
echo "2.0s; 500Hz per motor; Kp=0 Kd=0 tau=0; NO set-zero or mode switch"
cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/run_ankle_pair_zero_torque_gate.py" \
  --side right --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --contract "$ROOT/artifacts/g60_model3450/deploy/contract.json" \
  --socketcan-snapshot "$SNAPSHOT" --output "$OUTPUT" \
  --robot-supported --safety-operator-ready --ankle-clear \
  --confirm-hardware-tx "$ACK"
