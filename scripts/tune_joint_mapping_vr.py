#!/usr/bin/env python3
"""Conservative VR joystick joint tuning through JointCtrl."""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import numpy as np

from piper_vr.buttons import analog_value, is_pressed
from piper_vr.joint_limits import clamp_joints_deg, rate_limit_joints_deg
from piper_vr.piper_driver import PiperDriver
from piper_vr.quest_reader import QuestReader


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--can", default="can0")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--deadman-button", default="rightGrip")
    parser.add_argument("--speed-percent", type=int, default=5)
    parser.add_argument("--joint", type=int, default=1, choices=range(1, 7))
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--max-speed", type=float, default=5.0)
    parser.add_argument("--log-path", default="logs/joint_mimic/manual_joint_tuning.csv")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    quest = QuestReader(simulate_on_missing=args.dry_run)
    driver = PiperDriver(can=args.can, speed_percent=args.speed_percent, dry_run=args.dry_run)
    driver.connect(initial_mode="joint")
    measured = driver.read_joint_pose()
    if measured is None:
        raise RuntimeError("Joint feedback is required before manual joint tuning.")
    command = measured.joints_deg.copy()
    log_path = Path(args.log_path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["timestamp", "joint", "joystick", "command_deg"])
        try:
            while True:
                sample = quest.get_sample()
                buttons = {} if sample is None else sample.buttons
                joystick = analog_value(buttons, "rightJS")
                target = command.copy()
                if is_pressed(buttons, args.deadman_button):
                    target[args.joint - 1] += joystick * args.max_speed / args.rate
                    target = clamp_joints_deg(target)
                    command = rate_limit_joints_deg(target, command, [args.max_speed] * 6, 1.0 / args.rate)
                    driver.send_joint_pose(command)
                    writer.writerow([time.time(), args.joint, joystick, command.round(4).tolist()])
                    handle.flush()
                time.sleep(1.0 / args.rate)
        except KeyboardInterrupt:
            print("\nCtrl+C received. Holding tuned joint pose.")
            driver.hold_joints(allow_last_command_fallback=True)
            quest.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
