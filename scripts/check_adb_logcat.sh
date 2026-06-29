#!/usr/bin/env bash
set -euo pipefail

TAG="${1:-wE9ryARX}"

echo "ADB devices:"
adb devices
echo
echo "If the Quest appears as unauthorized, put on the headset and accept USB debugging."
echo "Quest teleop package: com.rail.oculus.teleop"
echo "Filtering logcat for tag: ${TAG}"
echo "Press Ctrl+C to stop."
adb logcat -T 0 | grep --line-buffered "${TAG}" || true
