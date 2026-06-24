#!/usr/bin/env bash
set -e

APK_PATH="${1:-third_party/APK/teleop-debug.apk}"

echo "[Quest] Checking ADB devices..."
adb devices

if [ ! -f "${APK_PATH}" ]; then
  echo "[Quest] APK not found at ${APK_PATH}."
  echo "[Quest] Place the Quest teleop APK at third_party/APK/teleop-debug.apk or pass its path as the first argument."
  echo "[Quest] Enable developer mode and accept USB debugging inside the headset before installing."
  exit 1
fi

echo "[Quest] Installing ${APK_PATH}..."
adb install -r -t "${APK_PATH}"
echo "[Quest] Done. If the device is unauthorized, put on the headset and accept USB debugging."
