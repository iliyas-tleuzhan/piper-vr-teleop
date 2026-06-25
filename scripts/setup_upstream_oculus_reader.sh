#!/usr/bin/env bash
set -e

UPSTREAM_DIR="${1:-$HOME/questVR_ws}"
OCULUS_SCRIPTS="${UPSTREAM_DIR}/src/oculus_reader/scripts"

if [ ! -d "${UPSTREAM_DIR}" ]; then
  echo "[oculus_reader] Upstream questVR_ws not found at ${UPSTREAM_DIR}"
  echo "Clone it with:"
  echo "  git clone https://github.com/agilexrobotics/questVR_ws.git ${UPSTREAM_DIR}"
  exit 1
fi

if [ ! -d "${OCULUS_SCRIPTS}" ]; then
  echo "[oculus_reader] Could not find ${OCULUS_SCRIPTS}"
  echo "Search manually:"
  echo "  find ${UPSTREAM_DIR} -type f | grep -i oculus"
  exit 1
fi

echo "[oculus_reader] Add this to your shell:"
echo "export PYTHONPATH=${OCULUS_SCRIPTS}:\$PYTHONPATH"
