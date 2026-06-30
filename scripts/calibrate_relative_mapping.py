#!/usr/bin/env python3
"""Compatibility wrapper for guided relative controller axis calibration."""

from __future__ import annotations

import sys

from scripts.record_quest_axis_movements import main as guided_main


def main() -> int:
    filtered = [arg for arg in sys.argv[1:] if arg not in ("--write-config", "--robot")]
    if "--seconds" in filtered:
        index = filtered.index("--seconds")
        del filtered[index : index + 2]
    sys.argv = [sys.argv[0], *filtered]
    print("calibrate_relative_mapping.py now runs the guided axis recorder.")
    return guided_main()


if __name__ == "__main__":
    raise SystemExit(main())
