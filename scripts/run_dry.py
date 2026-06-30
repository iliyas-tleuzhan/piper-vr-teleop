#!/usr/bin/env python3
"""Run joint mimic teleop in dry-run mode using config defaults."""

from __future__ import annotations

import sys

from piper_vr.vr_teleop import main


if __name__ == "__main__":
    if "--dry-run" not in sys.argv:
        sys.argv.append("--dry-run")
    raise SystemExit(main())
