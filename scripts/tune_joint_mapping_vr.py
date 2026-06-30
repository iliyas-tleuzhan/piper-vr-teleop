#!/usr/bin/env python3
"""Conservative VR joystick joint tuning through JointCtrl."""

from __future__ import annotations

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from piper_vr.buttons import analog_value, is_pressed
from piper_vr.joint_limits import clamp_joints_deg, rate_limit_joints_deg
from piper_vr.piper_driver import PiperDriver
from piper_vr.quest_reader import QuestReader


def _axis(buttons: dict[str, Any], key: str, index: int) -> float:
    value = buttons.get(key, (0.0, 0.0))
    if isinstance(value, (tuple, list)) and len(value) > index:
        return float(value[index])
    return 0.0


def main() -> int:
    parser = argparse.ArgumentParser(description="Tune Piper joint signs/gains safely with Quest controls")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--deadman-button", default="rightGrip")
    parser.add_argument("--speed-percent", type=int, default=5)
    parser.add_argument("--joint", type=int, default=1, choices=range(1, 7))
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--max-speed-deg-s", type=float, default=5.0)
    parser.add_argument("--delta-scale", type=float, default=1.0)
    parser.add_argument("--log-dir", default="logs/joint_tuning")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    quest = QuestReader(simulate_on_missing=args.dry_run)
    driver = PiperDriver(can=args.can, speed_percent=args.speed_percent, dry_run=args.dry_run)
    driver.connect(initial_mode="joint")
    measured = driver.read_joint_pose()
    if measured is None:
        raise RuntimeError("Joint feedback is required before manual joint tuning.")

    selected = int(args.joint) - 1
    command = measured.joints_deg.copy()
    period_s = 1.0 / float(args.rate)
    max_speeds = np.full(6, float(args.max_speed_deg_s))
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"joint_tuning_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    next_print_s = 0.0
    last_select_step_s = 0.0

    print("Manual joint tuning")
    print("Hold deadman to command. rightJS_y moves selected joint. A/B or rightJS_x selects joint.")
    print(f"Logging to {log_path}")

    with log_path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["timestamp", "selected_joint", "deadman", "input", "target_deg", "command_deg", "measured_deg"])
        try:
            while True:
                loop_start = time.monotonic()
                sample = quest.get_sample()
                buttons = {} if sample is None else sample.buttons
                now_s = time.monotonic()

                js_x = _axis(buttons, "rightJS", 0)
                if is_pressed(buttons, "A") or js_x > 0.75 and now_s - last_select_step_s > 0.35:
                    selected = min(5, selected + 1)
                    last_select_step_s = now_s
                if is_pressed(buttons, "B") or js_x < -0.75 and now_s - last_select_step_s > 0.35:
                    selected = max(0, selected - 1)
                    last_select_step_s = now_s

                move_input = _axis(buttons, "rightJS", 1)
                trigger_input = analog_value(buttons, "rightTrig") - analog_value(buttons, "leftTrig")
                if abs(move_input) < 0.05:
                    move_input = trigger_input

                deadman = is_pressed(buttons, args.deadman_button)
                measured = driver.read_joint_pose()
                measured_deg = None if measured is None else measured.joints_deg.copy()
                target = command.copy()
                if deadman:
                    target[selected] += move_input * float(args.delta_scale) * float(args.max_speed_deg_s) * period_s
                    target = clamp_joints_deg(target)
                    command = rate_limit_joints_deg(target, command, max_speeds, period_s)
                    driver.send_joint_pose(command)

                if now_s >= next_print_s:
                    next_print_s = now_s + 0.2
                    print(
                        f"joint={selected + 1} deadman={deadman} input={move_input:.2f} "
                        f"command={command.round(2).tolist()} "
                        f"measured={None if measured_deg is None else measured_deg.round(2).tolist()}"
                    )

                writer.writerow([
                    time.time(),
                    selected + 1,
                    deadman,
                    round(move_input, 4),
                    target.round(4).tolist(),
                    command.round(4).tolist(),
                    None if measured_deg is None else measured_deg.round(4).tolist(),
                ])
                handle.flush()
                time.sleep(max(0.0, period_s - (time.monotonic() - loop_start)))
        except KeyboardInterrupt:
            print("\nCtrl+C received. Holding tuned joint pose.")
            try:
                driver.hold_joints(allow_last_command_fallback=True)
            except RuntimeError as exc:
                print(f"WARNING: could not command joint hold on exit: {exc}")
            quest.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
