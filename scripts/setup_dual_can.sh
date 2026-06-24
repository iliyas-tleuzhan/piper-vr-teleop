#!/usr/bin/env bash
set -e

LEFT_CAN="${1:-can0}"
RIGHT_CAN="${2:-can1}"
BITRATE="${3:-1000000}"

"$(dirname "$0")/setup_can.sh" "${LEFT_CAN}" "${BITRATE}"
"$(dirname "$0")/setup_can.sh" "${RIGHT_CAN}" "${BITRATE}"
