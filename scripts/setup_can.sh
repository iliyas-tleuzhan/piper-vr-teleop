#!/usr/bin/env bash
set -e

CAN_IF="${1:-can0}"
BITRATE="${2:-1000000}"

echo "[CAN] Bringing down ${CAN_IF} if needed..."
sudo ip link set "${CAN_IF}" down 2>/dev/null || true

echo "[CAN] Bringing up ${CAN_IF} at ${BITRATE} bps..."
sudo ip link set "${CAN_IF}" up type can bitrate "${BITRATE}"

echo "[CAN] Done."
ip -details link show "${CAN_IF}"
