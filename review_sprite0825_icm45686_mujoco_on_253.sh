#!/usr/bin/env bash
set -euo pipefail
[[ $EUID == 0 && ${SUDO_USER:-} == tony ]] || { echo 'Run with sudo as tony'; exit 1; }
cd "$(dirname "${BASH_SOURCE[0]}")"
duration=${1:-120}
[[ $duration == 120 ]] || { echo 'This commissioning viewer uses 120 seconds'; exit 2; }
exec 9>/run/lock/sprite_icm_spi_pm.lock
flock -n 9 || { echo 'SPI PM test already active'; exit 1; }
power=/sys/bus/platform/devices/3210000.spi/power/control
old=$(cat "$power")
[[ $old == auto || $old == on ]] || exit 1
restore() { printf '%s\n' "$old" > "$power"; echo "RESTORED SPI PM=$old"; }
trap restore EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf 'on\n' > "$power"
echo 'ICM45686 ONLY: 1MHz request,800Hz native estimator,~50Hz display. NO CAN.'
runuser -u tony -- env DISPLAY=:1 python3 tools/view_icm45686.py --duration "$duration" \
  --mjcf /home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate/assets/mujoco/sprite0825_v5_4340_shoulders/sprite0825_v5_external_pd.xml
