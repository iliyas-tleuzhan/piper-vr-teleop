#!/usr/bin/env python3
"""Safely test a small Piper JointCtrl move."""

from __future__ import annotations

import argparse
import time

import numpy as np

from piper_vr.joint_limits import clamp_joints_deg
from piper_vr.piper_driver import PiperDriver


def run_joint_step_test(driver, joint: int, delta_deg: float, duration_s: float, rate_hz: float) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    start = driver.read_joint_pose()
    if start is None:
        raise RuntimeError("No Piper joint feedback. Refusing to move without a measured start pose.")
    target = start.joints_deg.copy()
    target[joint - 1] += float(delta_deg)
    target = clamp_joints_deg(target)

    period_s = 1.0 / float(rate_hz)
    end_s = time.monotonic() + float(duration_s)
    while time.monotonic() < end_s:
        loop_start = time.monotonic()
        driver.send_joint_pose(target)
        time.sleep(max(0.0, period_s - (time.monotonic() - loop_start)))

    after = driver.read_joint_pose()
    return start.joints_deg.copy(), target, None if after is None else after.joints_deg.copy()


def hold_for_exit(driver, seconds: float = 1.0, rate_hz: float = 50.0) -> None:
    period_s = 1.0 / float(rate_hz)
    end_s = time.monotonic() + float(seconds)
    while time.monotonic() < end_s:
        loop_start = time.monotonic()
        driver.hold_joints(allow_last_command_fallback=True)
        time.sleep(max(0.0, period_s - (time.monotonic() - loop_start)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--can", default="can0")
    parser.add_argument("--speed-percent", type=int, default=5)
    parser.add_argument("--joint", type=int, default=2, choices=range(1, 7))
    parser.add_argument("--delta-deg", type=float, default=3.0)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    driver = PiperDriver(can=args.can, speed_percent=args.speed_percent, dry_run=args.dry_run)
    try:
        driver.connect(initial_mode="joint")
        start, target, after = run_joint_step_test(driver, args.joint, args.delta_deg, args.duration, args.rate)
        print(f"START deg:  {start.round(3).tolist()}")
        print(f"TARGET deg: {target.round(3).tolist()}")
        print(f"AFTER deg:  {None if after is None else after.round(3).tolist()}")
        if after is not None:
            print(f"DELTA deg:  {(after - start).round(3).tolist()}")
        hold_for_exit(driver, 1.0, args.rate)
        return 0
    except KeyboardInterrupt:
        print("\nCtrl+C received. Holding current or last commanded joint pose.")
        hold_for_exit(driver, 1.0, args.rate)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
