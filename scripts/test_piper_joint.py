#!/usr/bin/env python3
"""Safely test a small Piper JointCtrl move."""

from __future__ import annotations

import argparse
import time

import numpy as np

from piper_vr.joint_limits import clamp_joints_deg
from piper_vr.piper_driver import PiperDriver


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--can", default="can0")
    parser.add_argument("--speed-percent", type=int, default=5)
    parser.add_argument("--joint", type=int, default=2, choices=range(1, 7))
    parser.add_argument("--delta-deg", type=float, default=3.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    driver = PiperDriver(can=args.can, speed_percent=args.speed_percent, dry_run=args.dry_run)
    driver.connect()
    driver.set_move_j_mode()
    start = driver.read_joint_pose()
    if start is None:
        raise RuntimeError("No Piper joint feedback. Refusing to move without a measured start pose.")
    target = start.joints_deg.copy()
    target[args.joint - 1] += float(args.delta_deg)
    target = clamp_joints_deg(target)
    print(f"START deg:  {start.joints_deg.round(3).tolist()}")
    print(f"TARGET deg: {target.round(3).tolist()}")
    driver.send_joint_pose(target)
    time.sleep(1.0)
    after = driver.read_joint_pose()
    print(f"AFTER deg:  {None if after is None else after.joints_deg.round(3).tolist()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
