#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
CANDIDATE=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate
MODES=(wrong_hash future_source stale_timestamp excessive_kd protected_position protected_kp silence)
JOINT_LIMITS="$ROOT/artifacts/g60_model3450/reports/sprite0825_urdf_limit_candidates.json"
[[ -f "$JOINT_LIMITS" ]] || JOINT_LIMITS="$ROOT/reports/sprite0825_urdf_limit_candidates.json"

PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_motor_config.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --output "$ROOT/build/native/motors.tsv"
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_joint_safety_config.py" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --joint-limit-candidates "$JOINT_LIMITS" \
  --gain-scale 0.1 \
  --maximum-embedded-kd 3.0 \
  --output "$ROOT/build/native/joint_safety.tsv"
JOINT_HASH="$(PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" -c \
  'import json,sys; from open_sprite_runtime.native_ipc import ordered_name_hash; d=json.load(open(sys.argv[1])); print(hex(ordered_name_hash(d["joint_names"])))' \
  "$CANDIDATE/deploy/contract.json")"

echo "NATIVE IPC FAIL-CLOSED FAULT INJECTION"
echo "All CAN frames remain restricted zero-gain position echo; all motors must be disabled"
for mode in "${MODES[@]}"; do
  socket="/tmp/open_sprite_fault_${mode}_${$}.sock"
  log="$ROOT/reports/native_ipc_fault_${mode}_$(date +%Y%m%d_%H%M%S).log"
  rm -f "$socket"
  set +e
  "$ROOT/build/native/sprite_can_shadow" \
    --config "$ROOT/build/native/motors.tsv" --duration 2 --cpu 5 \
    --output /tmp/unused_native_fault_report.json \
    --ipc-socket "$socket" --policy-joint-hash "$JOINT_HASH" \
    --joint-safety-config "$ROOT/build/native/joint_safety.tsv" \
    --acknowledge-hardware-tx ZERO_GAIN_NATIVE_SHADOW \
    --all-motors-disabled-confirmed >"$log" 2>&1 &
  native_pid=$!
  for _ in $(seq 1 100); do
    [[ -S "$socket" ]] && break
    kill -0 "$native_pid" 2>/dev/null || break
    sleep 0.02
  done
  PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
    -m open_sprite_runtime.native_ipc_fault_client \
    --socket "$socket" --joint-hash "$JOINT_HASH" --mode "$mode"
  client_status=$?
  wait "$native_pid"
  native_status=$?
  set -e
  if (( native_status == 0 )); then
    echo "FAIL mode=$mode native accepted injected fault" >&2
    cat "$log" >&2
    exit 1
  fi
  case "$mode" in
    wrong_hash) expected="policy IPC target ABI/order invariant failed" ;;
    future_source) expected="policy IPC target sequence invariant failed" ;;
    stale_timestamp) expected="policy IPC target timestamp is future or stale" ;;
    excessive_kd) expected="policy IPC target numeric/gain invariant failed" ;;
    protected_position|protected_kp) expected="policy IPC protected target envelope failed" ;;
    silence) expected="policy IPC initial target watchdog expired" ;;
  esac
  if ! grep -Fq "$expected" "$log"; then
    echo "FAIL mode=$mode did not produce expected rejection" >&2
    cat "$log" >&2
    exit 1
  fi
  echo "PASS mode=$mode native_status=$native_status client_status=$client_status rejection='$expected'"
done
echo "PASS all native IPC faults failed closed"
