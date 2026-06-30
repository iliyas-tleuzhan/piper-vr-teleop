#!/usr/bin/env python3
"""Inspect relative controller directions and generated joint deltas."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import yaml

from piper_vr.buttons import is_pressed
from piper_vr.frame_calibration import ControlFrameConfig, controller_delta_in_control_frame, get_control_frame
from piper_vr.joint_mimic import JointMimicConfig
from piper_vr.quest_reader import QuestReader


def _fmt(values: np.ndarray) -> str:
    return np.asarray(values, dtype=float).round(4).tolist().__repr__()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/single_piper.yaml")
    parser.add_argument("--side", default="right")
    parser.add_argument("--seconds", type=float, default=45.0)
    parser.add_argument("--calibrate-button", default="A")
    parser.add_argument("--write-config", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--robot", action="store_true", help="reserved for future robot-assisted calibration")
    args = parser.parse_args()

    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    mimic = JointMimicConfig.from_config(config.get("joint_mimic"))
    quest_config = config.get("quest", {})
    quest = QuestReader(
        transport=quest_config.get("transport", "adb_logcat"),
        connection=quest_config.get("connection", "usb"),
        ip_address=quest_config.get("ip_address"),
        simulate_on_missing=args.dry_run,
    )

    print(f"Press {args.calibrate_button} to set home, then move controller up, right, and forward.")
    home = None
    control_frame = None
    previous_buttons_pressed = False
    end_s = time.monotonic() + args.seconds
    while time.monotonic() < end_s:
        sample = quest.get_sample()
        transform = None if sample is None else sample.transforms_openxr.get(args.side)
        if transform is None:
            print("No controller sample")
            time.sleep(0.2)
            continue
        pressed = is_pressed(sample.buttons, args.calibrate_button)
        if home is None:
            if pressed and not previous_buttons_pressed:
                home = transform.copy()
                control_frame = get_control_frame(sample, args.side, ControlFrameConfig(source=mimic.control_frame), home)
                print("CALIBRATED. Move one direction at a time.")
            previous_buttons_pressed = pressed
            time.sleep(0.05)
            continue
        delta_xyz, delta_rot = controller_delta_in_control_frame(home, transform, control_frame)
        u = np.concatenate((delta_xyz, np.zeros(3) if not mimic.wrist_rotation_enabled else delta_rot))
        dq = mimic.relative_gain_matrix @ u
        print(f"delta_xyz={_fmt(delta_xyz)} delta_rot_deg={_fmt(delta_rot)} joint_delta_deg={_fmt(dq)}")
        for channel, name in enumerate(("dx", "dy", "dz")):
            if abs(delta_xyz[channel]) > 0.05:
                sign = "positive" if delta_xyz[channel] > 0 else "negative"
                print(f"  detected {name} {sign}; check relative_gain_matrix column {channel}")
        time.sleep(0.2)
    quest.stop()

    if args.write_config:
        path = Path("configs/local_relative_mapping.yaml")
        path.write_text(
            yaml.safe_dump({"joint_mimic": {"relative_gain_matrix": mimic.relative_gain_matrix.tolist()}}, sort_keys=False),
            encoding="utf-8",
        )
        print(f"Wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
