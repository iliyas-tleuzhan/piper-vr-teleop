#!/usr/bin/env python3
"""Verify endpoint IK direction mapping with Quest-only guided motions."""

from __future__ import annotations

import argparse
import time

import numpy as np

from piper_vr.buttons import is_pressed
from piper_vr.config import deep_merge, load_config
from piper_vr.frame_calibration import ControlFrameConfig, get_control_frame
from piper_vr.quest_endpoint_ik import QuestEndpointIKConfig, endpoint_target_from_controller
from piper_vr.quest_reader import QuestReader
from piper_vr.relative_calibration import dominant_channel


EXPECTED = {
    "right": ("robot_y", -1.0),
    "left": ("robot_y", 1.0),
    "up": ("robot_z", 1.0),
    "down": ("robot_z", -1.0),
    "forward": ("robot_x", 1.0),
    "backward": ("robot_x", -1.0),
}
AXIS_INDEX = {"robot_x": 0, "robot_y": 1, "robot_z": 2}
QUEST_AXIS = ("quest_x", "quest_y", "quest_z")


def _fmt(values: np.ndarray, digits: int = 4) -> str:
    return np.asarray(values, dtype=float).round(digits).tolist().__repr__()


def _load_with_base(path: str) -> dict:
    return deep_merge(load_config("configs/single_piper.yaml"), load_config(path))


def _wait_button(quest: QuestReader, side: str, button: str, prompt: str):
    print(prompt)
    was_pressed = True
    while True:
        sample = quest.get_sample()
        transform = None if sample is None else sample.transforms_openxr.get(side)
        if transform is None:
            print("No controller sample")
            time.sleep(0.2)
            continue
        pressed = is_pressed(sample.buttons, button)
        if pressed and not was_pressed:
            return sample, np.asarray(transform, dtype=float).copy()
        was_pressed = pressed
        time.sleep(0.02)


def _suggest_rule(robot_axis: str, controller_delta_xyz: np.ndarray, expected_sign: float) -> str:
    index, _, _, value = dominant_channel(controller_delta_xyz)
    sign = expected_sign / (1.0 if value >= 0 else -1.0)
    return ("+" if sign >= 0 else "-") + QUEST_AXIS[index]


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify endpoint IK direction signs")
    parser.add_argument("--config", default="configs/generated_endpoint_ik_mapping.yaml")
    parser.add_argument("--side", choices=("left", "right"), default=None)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    config = _load_with_base(args.config)
    side = args.side or config.get("side", "right")
    button = config.get("calibrate_button", "A")
    quest_config = config.get("quest", {})
    ik_config = QuestEndpointIKConfig.from_config(config.get("quest_endpoint_ik"))
    quest = QuestReader(
        transport=quest_config.get("transport", "adb_logcat"),
        connection=quest_config.get("connection", "usb"),
        ip_address=quest_config.get("ip_address"),
        simulate_on_missing=args.dry_run,
    )

    home_sample, home = _wait_button(quest, side, button, f"Hold controller at HOME, then press {button}.")
    control_frame = get_control_frame(home_sample, side, ControlFrameConfig(source=ik_config.control_frame), home)
    robot_home_xyz = np.array([0.35, 0.0, 0.25], dtype=float)
    robot_home_rpy = np.zeros(3)
    try:
        for movement, (robot_axis, expected_sign) in EXPECTED.items():
            _, final = _wait_button(quest, side, button, f"Move {movement.upper()}, hold, then press {button}.")
            _, _, debug = endpoint_target_from_controller(home, final, robot_home_xyz, robot_home_rpy, ik_config, control_frame=control_frame)
            axis_index = AXIS_INDEX[robot_axis]
            value = float(debug["mapped_robot_delta_xyz"][axis_index])
            passed = np.sign(value) == np.sign(expected_sign) and abs(value) > 1e-5
            print(f"{movement.upper()} physical movement:")
            print(f"  controller_delta_xyz = {_fmt(debug['controller_delta_xyz'])}")
            print(f"  mapped_robot_delta_xyz = {_fmt(debug['mapped_robot_delta_xyz'])}")
            print(f"  scaled_robot_delta_xyz = {_fmt(debug['scaled_robot_delta_xyz'])}")
            print(f"  expected robot axis: {'+' if expected_sign > 0 else '-'}{robot_axis[-1].upper()}")
            print(f"  {'PASS' if passed else 'FAIL'}")
            if not passed:
                suggestion = _suggest_rule(robot_axis, debug["controller_delta_xyz"], expected_sign)
                print(f"  {movement.upper()} is inverted or weak. Suggested fix: {robot_axis}: \"{suggestion}\"")
    finally:
        quest.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
