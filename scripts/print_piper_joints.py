#!/usr/bin/env python3
"""Print current Piper joint feedback."""

from __future__ import annotations

import argparse

from piper_vr.piper_driver import PiperDriver


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--can", default="can0")
    parser.add_argument("--speed-percent", type=int, default=5)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--debug-feedback", action="store_true")
    args = parser.parse_args()

    driver = PiperDriver(can=args.can, speed_percent=args.speed_percent, dry_run=args.dry_run)
    driver.connect(initial_mode="joint")
    pose = driver.read_joint_pose(debug_feedback=args.debug_feedback)
    if pose is None:
        raise RuntimeError("No Piper joint feedback was available. Check piper-sdk getter compatibility.")
    print(f"Piper joints deg: {pose.joints_deg.round(3).tolist()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
