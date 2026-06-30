#!/usr/bin/env bash
set -euo pipefail

python3 -m piper_vr.vr_teleop --config configs/single_piper.yaml --control-mode joint_mimic --can can0 --speed-percent 5 --verbose
