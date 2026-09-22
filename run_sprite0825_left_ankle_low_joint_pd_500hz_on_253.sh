#!/usr/bin/env bash
set -euo pipefail

ACK=ENABLE_LEFT_ANKLE_LOW_JOINT_PD_500HZ
if [[ "${1:-}" != "$ACK" ]]; then
  echo "FIRST HOST-PD DIFFERENTIAL-ANKLE GATE: left pair only" >&2
  echo "Robot must be securely suspended, foot/linkages clear, safety operator ready." >&2
  echo "500Hz joint PD: Kp=0.5, Kd=0.05, |joint torque|<=0.15Nm, duration 2s." >&2
  echo "Motor embedded Kp=Kd=0; torque is mapped by the calibrated inverse transpose." >&2
  echo "This command never resets motor zero positions." >&2
  echo "Run only after explicit physical confirmation: $0 $ACK" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kcan1_left_ankle_low_joint_pd_${STAMP}.json"
OUTPUT="$ROOT/reports/left_ankle_low_joint_pd_500hz_${STAMP}.json"
mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"
echo "ACTIVE HOST-PD GATE: left ankle motors kcan1 IDs 0x05/0x06 only"
echo "2.0s; 500Hz; joint Kp=0.5 Kd=0.05; |tau_joint|<=0.15Nm"
echo "embedded motor Kp=Kd=0; NO set-zero or mode switch"
cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/run_ankle_pair_zero_torque_gate.py" \
  --side left --mode joint-pd \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --contract "$ROOT/artifacts/g60_model3450/deploy/contract.json" \
  --socketcan-snapshot "$SNAPSHOT" --output "$OUTPUT" \
  --robot-supported --safety-operator-ready --ankle-clear \
  --confirm-hardware-tx "$ACK"
