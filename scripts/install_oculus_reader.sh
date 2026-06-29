#!/usr/bin/env bash
set -euo pipefail

SOURCE_URL="${OCULUS_READER_URL:-https://github.com/jborbik/oculus_reader.git}"

echo "Installing Quest-3-capable oculus_reader from: ${SOURCE_URL}"
if python3 -m pip install "git+${SOURCE_URL}"; then
  python3 - <<'PY'
import importlib
for name in ("oculus_reader.reader", "oculus_reader"):
    try:
        module = importlib.import_module(name)
        print(f"oculus_reader import OK: {getattr(module, '__file__', name)}")
        break
    except ImportError:
        pass
else:
    raise SystemExit("Install finished but oculus_reader could not be imported.")
PY
  exit 0
fi

cat <<'EOF'
Direct install failed.

Legacy fallback:
  cd ~
  git clone https://github.com/agilexrobotics/questVR_ws.git
  export PYTHONPATH=~/questVR_ws/src/oculus_reader/scripts:$PYTHONPATH
  python3 -c "import oculus_reader; print('oculus_reader ok')"
EOF
exit 1
