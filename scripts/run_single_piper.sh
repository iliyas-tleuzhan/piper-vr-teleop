#!/usr/bin/env bash
set -e

python3 -m piper_vr.movep_teleop --config configs/single_piper.yaml "$@"
