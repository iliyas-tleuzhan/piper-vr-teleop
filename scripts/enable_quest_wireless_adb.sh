#!/usr/bin/env bash
set -e

QUEST_IP="${1:-}"

echo "[Quest] Enabling TCP ADB on port 5555 over the current USB connection..."
adb tcpip 5555

if [ -z "${QUEST_IP}" ]; then
  echo "[Quest] Find the headset IP address in the Quest Wi-Fi settings or run:"
  echo "adb shell ip route"
  echo "[Quest] Then run:"
  echo "$0 <Quest_IP>"
  exit 0
fi

echo "[Quest] Connecting to ${QUEST_IP}:5555..."
adb connect "${QUEST_IP}:5555"
adb devices
