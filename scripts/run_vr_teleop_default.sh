#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONPATH="$PWD:${HOME}/Iliyas/questVR_ws/src/oculus_reader/scripts:${PYTHONPATH:-}"
python3 -m piper_vr.vr_teleop "$@"
