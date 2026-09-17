#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
ROOT="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"

YEAR="$(date +%Y)"
if (( YEAR < 2025 )); then
  echo "ERROR: system clock is not credible (year=$YEAR); synchronize time before runtime use" >&2
  exit 2
fi

PYTHON="$ROOT/.venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
  echo "ERROR: runtime virtual environment is missing: $PYTHON" >&2
  echo "Install the project into .venv before running the Jetson launcher." >&2
  exit 2
fi

exec "$PYTHON" -m open_sprite_runtime.cli "$@"
