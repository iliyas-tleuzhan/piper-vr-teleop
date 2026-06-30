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


TRANSLATION_STEPS = (
    ("right", "left", "robot_y", -1.0),
    ("up", "down", "robot_z", 1.0),
    ("forward", "backward", "robot_x", -1.0),
)
ROTATION_STEPS = (
    ("roll_clockwise", "roll_counterclockwise", "robot_roll", 1.0),
    ("pitch_up", "pitch_down", "robot_pitch", 1.0),
    ("yaw_right", "yaw_left", "robot_yaw", 1.0),
)
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


def _pair_warnings(name_a: str, value_a: float, index_a: int, name_b: str, value_b: float, index_b: int, *, min_amplitude: float, unit: str) -> list[str]:
    warnings = []
    amplitude = (abs(value_a) + abs(value_b)) / 2.0
    if amplitude < min_amplitude:
        warnings.append(f"{name_a}/{name_b} amplitude too small: {amplitude:.4f} {unit}")
    if index_a != index_b:
        warnings.append(f"{name_a}/{name_b} dominant axes differ")
    if np.sign(value_a) == np.sign(value_b) and abs(value_a) > 0 and abs(value_b) > 0:
        warnings.append(f"{name_a}/{name_b} signs are not opposite")
    return warnings


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
    records = []
    warnings = []
    try:
        for movement, opposite, robot_axis, desired_sign in TRANSLATION_STEPS:
            _, final = _wait_button(quest, side, button, f"Move controller {movement.upper()}, hold, then press {button}.")
            delta_xyz, _ = controller_delta_in_control_frame(home, final, np.eye(3))
            index, channel, sign, value = dominant_channel(delta_xyz)
            axis_mapping[robot_axis] = _rule(desired_sign, value, QUEST_AXES[index])
            records.append({"movement": movement, "delta_xyz": delta_xyz.round(6).tolist(), "dominant_channel": channel, "value": round(value, 6)})
            print(f"{movement}: delta_xyz={delta_xyz.round(4).tolist()} dominant={channel} {sign} -> {robot_axis}: {axis_mapping[robot_axis]}")
            _, final = _wait_button(quest, side, button, f"Move controller {opposite.upper()}, hold, then press {button}.")
            opposite_delta, _ = controller_delta_in_control_frame(home, final, np.eye(3))
            opposite_index, opposite_channel, opposite_sign, opposite_value = dominant_channel(opposite_delta)
            records.append({"movement": opposite, "delta_xyz": opposite_delta.round(6).tolist(), "dominant_channel": opposite_channel, "value": round(opposite_value, 6)})
            warnings.extend(_pair_warnings(movement, value, index, opposite, opposite_value, opposite_index, min_amplitude=0.03, unit="m"))
        for movement, opposite, robot_axis, desired_sign in ROTATION_STEPS:
            _, final = _wait_button(quest, side, button, f"Rotate controller {movement.upper().replace('_', ' ')}, hold, then press {button}.")
            _, delta_rot = controller_delta_in_control_frame(home, final, np.eye(3))
            index, channel, sign, value = dominant_rotation_channel(delta_rot)
            rotation_mapping[robot_axis] = _rule(desired_sign, value, QUEST_ROT_AXES[index])
            records.append({"movement": movement, "delta_rot_deg": delta_rot.round(6).tolist(), "dominant_channel": channel, "value": round(value, 6)})
            print(f"{movement}: delta_rot={delta_rot.round(3).tolist()} dominant={channel} {sign} -> {robot_axis}: {rotation_mapping[robot_axis]}")
            _, final = _wait_button(quest, side, button, f"Rotate controller {opposite.upper().replace('_', ' ')}, hold, then press {button}.")
            _, opposite_rot = controller_delta_in_control_frame(home, final, np.eye(3))
            opposite_index, opposite_channel, _, opposite_value = dominant_rotation_channel(opposite_rot)
            records.append({"movement": opposite, "delta_rot_deg": opposite_rot.round(6).tolist(), "dominant_channel": opposite_channel, "value": round(opposite_value, 6)})
            warnings.extend(_pair_warnings(movement, value, index, opposite, opposite_value, opposite_index, min_amplitude=5.0, unit="deg"))
    finally:
        quest.stop()

    payload = {
        "quest_endpoint_ik": {
            "axis_mapping": axis_mapping,
            "rotation_mapping": rotation_mapping,
            "mapping_metadata": {"records": records, "warnings": warnings, "ok": not warnings},
        }
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    print(f"Wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
