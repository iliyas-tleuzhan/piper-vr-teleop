#!/usr/bin/env python3
"""Guided Quest controller axis calibration for relative joint mimic."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from piper_vr.buttons import is_pressed
from piper_vr.config import load_config
from piper_vr.frame_calibration import ControlFrameConfig, controller_delta_in_control_frame, get_control_frame
from piper_vr.joint_mimic import JointMimicConfig
from piper_vr.quest_reader import QuestReader
from piper_vr.relative_calibration import MOVEMENTS, ROTATION_MOVEMENTS, build_observation


def _fmt(values: np.ndarray) -> str:
    return np.asarray(values, dtype=float).round(4).tolist().__repr__()


def _wait_for_button_sample(quest: QuestReader, side: str, button: str, prompt: str):
    print(prompt)
    was_pressed = True
    while True:
        sample = quest.get_sample()
        transform = None if sample is None else sample.transforms_openxr.get(side)
        if sample is None or transform is None:
            print("No controller sample")
            time.sleep(0.2)
            continue
        pressed = is_pressed(sample.buttons, button)
        if pressed and not was_pressed:
            return sample, np.asarray(transform, dtype=float).copy()
        was_pressed = pressed
        time.sleep(0.02)


def main() -> int:
    parser = argparse.ArgumentParser(description="Record physical Quest controller movement directions")
    parser.add_argument("--config", default="configs/single_piper.yaml")
    parser.add_argument("--side", choices=("left", "right"), default=None)
    parser.add_argument("--calibrate-button", default=None)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config)
    side = args.side or config.get("side", "right")
    button = args.calibrate_button or config.get("calibrate_button", "A")
    mimic = JointMimicConfig.from_config(config.get("joint_mimic"))
    quest_config = config.get("quest", {})
    quest = QuestReader(
        transport=quest_config.get("transport", "adb_logcat"),
        connection=quest_config.get("connection", "usb"),
        ip_address=quest_config.get("ip_address"),
        simulate_on_missing=args.dry_run,
    )

    log_dir = Path("logs/quest_axis_calibration")
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    latest_path = log_dir / "latest_axis_calibration.json"
    stamped_path = log_dir / f"axis_calibration_{stamp}.json"

    print("Quest axis calibration. This does not connect to or move Piper.")
    print(f"Controller: {side}, set/record button: {button}, control frame: {mimic.control_frame}")
    home_sample, home = _wait_for_button_sample(quest, side, button, f"Hold the controller at HOME, then press {button}.")
    control_frame = get_control_frame(home_sample, side, ControlFrameConfig(source=mimic.control_frame), home)
    print(f"Home raw controller xyz: {_fmt(home[:3, 3])}")

    rows = []
    try:
        for movement in (*MOVEMENTS, *ROTATION_MOVEMENTS):
            _, final = _wait_for_button_sample(
                quest,
                side,
                button,
                f"\nMove/rotate controller {movement.upper().replace('_', ' ')}, hold it there, then press {button}.",
            )
            delta_xyz, delta_rot_deg = controller_delta_in_control_frame(home, final, control_frame)
            row = build_observation(movement, delta_xyz, delta_rot_deg)
            row["raw_controller_xyz"] = final[:3, 3].round(6).tolist()
            rows.append(row)
            print(f"{movement.upper().replace('_', ' ')} movement:")
            print(f"  raw controller xyz = {_fmt(final[:3, 3])}")
            print(f"  delta_xyz = {_fmt(delta_xyz)}")
            print(f"  delta_rot_deg = {_fmt(delta_rot_deg)}")
            print(f"  dominant channel = {row['dominant_channel']}")
            print(f"  sign = {row['sign']}")
            print(f"  dominant rotation channel = {row['dominant_rotation_channel']}")
            print(f"  rotation sign = {row['rotation_sign']}")
    finally:
        quest.stop()

    payload = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "config": args.config,
        "side": side,
        "calibrate_button": button,
        "control_frame": mimic.control_frame,
        "home_raw_controller_xyz": home[:3, 3].round(6).tolist(),
        "movements": rows,
    }
    text = json.dumps(payload, indent=2)
    latest_path.write_text(text + "\n", encoding="utf-8")
    stamped_path.write_text(text + "\n", encoding="utf-8")
    print(f"\nWrote {latest_path}")
    print(f"Wrote {stamped_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
