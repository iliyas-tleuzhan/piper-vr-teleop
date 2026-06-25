#!/usr/bin/env python3
"""Move Piper endpoint Z by a small amount through the PiperDriver wrapper."""

from __future__ import annotations

import argparse
import time

import numpy as np

from piper_vr.piper_driver import PiperDriver


def main() -> int:
    parser = argparse.ArgumentParser(description="Test Piper endpoint movement through PiperDriver")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--speed-percent", type=int, default=5)
    parser.add_argument("--dz", type=float, default=0.02, help="Z offset in meters")
    args = parser.parse_args()

    driver = PiperDriver(can=args.can, speed_percent=args.speed_percent, dry_run=False)
    try:
        driver.connect()
        start_pose = driver.read_end_pose()
        if start_pose is None:
            raise RuntimeError("Could not read Piper endpoint pose after connecting.")

        print(f"START xyz_m={start_pose.xyz_m.round(4).tolist()} rpy_deg={start_pose.rpy_deg.round(2).tolist()}")

        target_xyz = np.array(start_pose.xyz_m, dtype=float)
        target_xyz[2] += float(args.dz)
        target_rpy = np.array(start_pose.rpy_deg, dtype=float)

        period_s = 1.0 / 50.0
        end_s = time.monotonic() + 3.0
        while time.monotonic() < end_s:
            loop_start = time.monotonic()
            driver.send_end_pose(target_xyz, target_rpy)
            time.sleep(max(0.0, period_s - (time.monotonic() - loop_start)))

        after_pose = driver.read_end_pose()
        if after_pose is None:
            print("AFTER pose unavailable")
        else:
            print(f"AFTER xyz_m={after_pose.xyz_m.round(4).tolist()} rpy_deg={after_pose.rpy_deg.round(2).tolist()}")

        driver.hold()
        return 0
    except KeyboardInterrupt:
        print("\nCtrl+C received. Holding the last endpoint command.")
        driver.hold()
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
