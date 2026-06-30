#!/usr/bin/env python3
"""Generate endpoint IK axis and rotation mapping from guided Quest motions."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import numpy as np
import yaml

from piper_vr.buttons import is_pressed
from piper_vr.config import load_config
from piper_vr.frame_calibration import controller_delta_in_control_frame
from piper_vr.quest_reader import QuestReader
from piper_vr.relative_calibration import dominant_channel, dominant_rotation_channel


TRANSLATION_STEPS = (("right", "robot_y", -1.0), ("up", "robot_z", 1.0), ("forward", "robot_x", -1.0))
ROTATION_STEPS = (("roll_clockwise", "robot_roll", 1.0), ("pitch_up", "robot_pitch", 1.0), ("yaw_right", "robot_yaw", 1.0))
QUEST_AXES = ("quest_x", "quest_y", "quest_z")
QUEST_ROT_AXES = ("quest_roll", "quest_pitch", "quest_yaw")


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


def _rule(sign: float, axis_value: float, axis_name: str) -> str:
    signed = sign * (1.0 if axis_value >= 0.0 else -1.0)
    return ("+" if signed >= 0.0 else "-") + axis_name


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate Quest endpoint IK mapping")
    parser.add_argument("--config", default="configs/single_piper.yaml")
    parser.add_argument("--side", choices=("left", "right"), default=None)
    parser.add_argument("--output", default="configs/generated_endpoint_ik_mapping.yaml")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config)
    side = args.side or config.get("side", "right")
    button = config.get("calibrate_button", "A")
    quest_config = config.get("quest", {})
    quest = QuestReader(
        transport=quest_config.get("transport", "adb_logcat"),
        connection=quest_config.get("connection", "usb"),
        ip_address=quest_config.get("ip_address"),
        simulate_on_missing=args.dry_run,
    )

    home_sample, home = _wait_button(quest, side, button, f"Hold controller at endpoint IK HOME, then press {button}.")
    axis_mapping = {}
    rotation_mapping = {}
    try:
        for movement, robot_axis, desired_sign in TRANSLATION_STEPS:
            _, final = _wait_button(quest, side, button, f"Move controller {movement.upper()}, hold, then press {button}.")
            delta_xyz, _ = controller_delta_in_control_frame(home, final, np.eye(3))
            index, channel, sign, value = dominant_channel(delta_xyz)
            axis_mapping[robot_axis] = _rule(desired_sign, value, QUEST_AXES[index])
            print(f"{movement}: delta_xyz={delta_xyz.round(4).tolist()} dominant={channel} {sign} -> {robot_axis}: {axis_mapping[robot_axis]}")
        for movement, robot_axis, desired_sign in ROTATION_STEPS:
            _, final = _wait_button(quest, side, button, f"Rotate controller {movement.upper().replace('_', ' ')}, hold, then press {button}.")
            _, delta_rot = controller_delta_in_control_frame(home, final, np.eye(3))
            index, channel, sign, value = dominant_rotation_channel(delta_rot)
            rotation_mapping[robot_axis] = _rule(desired_sign, value, QUEST_ROT_AXES[index])
            print(f"{movement}: delta_rot={delta_rot.round(3).tolist()} dominant={channel} {sign} -> {robot_axis}: {rotation_mapping[robot_axis]}")
    finally:
        quest.stop()

    payload = {"quest_endpoint_ik": {"axis_mapping": axis_mapping, "rotation_mapping": rotation_mapping}}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    print(f"Wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
