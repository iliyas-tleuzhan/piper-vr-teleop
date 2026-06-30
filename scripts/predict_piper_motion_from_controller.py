#!/usr/bin/env python3
"""Live no-robot predictor for Quest relative controller motion."""

from __future__ import annotations

import argparse
import time

import numpy as np

from piper_vr.buttons import is_pressed
from piper_vr.config import load_config
from piper_vr.frame_calibration import ControlFrameConfig, controller_delta_in_control_frame, get_control_frame
from piper_vr.joint_mimic import JointMimicConfig
from piper_vr.quest_reader import QuestReader
from piper_vr.relative_calibration import dominant_channel


def _fmt(values: np.ndarray | None, digits: int = 3) -> str:
    if values is None:
        return "None"
    return np.asarray(values, dtype=float).round(digits).tolist().__repr__()


def main() -> int:
    parser = argparse.ArgumentParser(description="Predict Piper joint deltas from Quest controller movement without robot")
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

    print(f"Press {button} to set home. No Piper connection will be opened.")
    home = None
    control_frame = None
    previous_transform = None
    previous_pressed = False
    try:
        while True:
            sample = quest.get_sample()
            transform = None if sample is None else sample.transforms_openxr.get(side)
            if transform is None:
                print("No controller sample")
                time.sleep(0.2)
                continue
            pressed = is_pressed(sample.buttons, button)
            if pressed and not previous_pressed:
                home = np.asarray(transform, dtype=float).copy()
                previous_transform = home.copy()
                control_frame = get_control_frame(sample, side, ControlFrameConfig(source=mimic.control_frame), home)
                print(f"HOME set at raw xyz {_fmt(home[:3, 3])}")
            previous_pressed = pressed
            if home is None or previous_transform is None or control_frame is None:
                time.sleep(0.05)
                continue
            delta_xyz, delta_rot = controller_delta_in_control_frame(previous_transform, transform, control_frame)
            previous_transform = np.asarray(transform, dtype=float).copy()
            if float(np.linalg.norm(delta_xyz)) < mimic.translation_deadband_m:
                delta_xyz = np.zeros(3)
            if float(np.linalg.norm(delta_rot)) < mimic.rotation_deadband_deg:
                delta_rot = np.zeros(3)
            if not mimic.wrist_rotation_enabled:
                delta_rot = np.zeros(3)
            u = np.concatenate((delta_xyz, delta_rot))
            dq = mimic.relative_gain_matrix @ u
            _, channel, sign, value = dominant_channel(delta_xyz)
            print(
                f"raw_xyz={_fmt(np.asarray(transform)[:3, 3])} "
                f"delta_xyz={_fmt(delta_xyz, 4)} dominant={channel} {sign} {value:.4f} "
                f"u={_fmt(u, 4)} predicted_joint_delta_deg={_fmt(dq, 2)}"
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        return 0
    finally:
        quest.stop()


if __name__ == "__main__":
    raise SystemExit(main())
