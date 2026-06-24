#!/usr/bin/env python3
"""Print Piper endpoint feedback."""

from __future__ import annotations

import argparse
import time

from piper_vr.piper_driver import PiperDriver


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--can", default="can0")
    parser.add_argument("--hz", type=float, default=5.0)
    args = parser.parse_args()
    driver = PiperDriver(can=args.can, dry_run=False)
    driver.connect()
    period = 1.0 / args.hz
    while True:
        pose = driver.read_end_pose()
        print(f"xyz_m={pose.xyz_m.round(4).tolist()} rpy_deg={pose.rpy_deg.round(2).tolist()}")
        time.sleep(period)


if __name__ == "__main__":
    raise SystemExit(main())
