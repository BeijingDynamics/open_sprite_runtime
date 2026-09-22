#!/usr/bin/env bash
set -euo pipefail

ACK=ENABLE_HEAD_PAIR_LOW_JOINT_PD_500HZ
if [[ "${1:-}" != "$ACK" ]]; then
  echo "HOST-PD DIFFERENTIAL GATE: head pitch/roll pair only" >&2
  echo "Robot must be suspended; head/linkages clear; safety operator ready." >&2
  echo "500Hz joint PD: Kp=0.2 Kd=0.03, |joint torque|<=0.05Nm, duration 2s." >&2
  echo "Motor embedded Kp=Kd=0; calibrated inverse-transpose torque mapping." >&2
  echo "NO set-zero; NO mode switch. Run after explicit approval: $0 $ACK" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kcan2_head_pair_low_joint_pd_${STAMP}.json"
OUTPUT="$ROOT/reports/head_pair_low_joint_pd_500hz_${STAMP}.json"
mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"
echo "ACTIVE HOST-PD GATE: head pitch/roll kcan2 IDs 0x07/0x08 only"
echo "2.0s; 500Hz; joint Kp=0.2 Kd=0.03; |tau_joint|<=0.05Nm"
echo "embedded motor Kp=Kd=0; NO set-zero or mode switch"
cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/run_head_pair_joint_pd_gate.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --contract "$ROOT/artifacts/g60_model3450/deploy/contract.json" \
  --socketcan-snapshot "$SNAPSHOT" --output "$OUTPUT" \
  --robot-supported --safety-operator-ready --head-clear \
  --confirm-hardware-tx "$ACK"
