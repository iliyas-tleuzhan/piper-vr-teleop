#!/usr/bin/env python3
"""Create a small, ready-to-run single-arm teleoperation configuration."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


DEFAULT = {
    "can": "can0",
    "hz": 30.0,
    "speed_percent": 20,
    "side": "right",
    "deadman_button": "rightGrip",
    "calibrate_button": "A",
    "gripper_enabled": False,
    "scale": 0.80,
    "max_speed_m_s": 0.15,
    "position_filter_enabled": True,
    "position_deadband_m": 0.003,
    "position_filter_alpha": 0.35,
    "workspace_min_m": [0.18, -0.32, 0.08],
    "workspace_max_m": [0.62, 0.32, 0.55],
    "axis_mapping": {
        "translation_frame": "controller_home",
        "piper_x": "-vr_z",
        "piper_y": "-vr_x",
        "piper_z": "+vr_y",
    },
    "hold_orientation": True,
    "orientation_enabled": False,
    "orientation_scale": 1.0,
    "max_angular_speed_deg_s": 60.0,
    "max_orientation_delta_deg": [45.0, 45.0, 60.0],
    "orientation_filter_enabled": True,
    "orientation_deadband_deg": 2.0,
    "orientation_filter_alpha": 0.25,
    "default_rpy_deg": None,
    "urdf_guard_enabled": True,
    "urdf_path": "third_party/agx_arm_urdf/piper/urdf/piper_description.urdf",
    "quest": {"connection": "usb", "ip_address": None},
}


def main() -> int:
    parser = argparse.ArgumentParser(description="Create a Piper Quest teleop YAML configuration")
    parser.add_argument("--output", default="configs/local_piper.yaml")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--quest-ip")
    parser.add_argument("--scale", type=float, default=0.80)
    parser.add_argument("--max-speed", type=float, default=0.15)
    args = parser.parse_args()

    config = DEFAULT | {
        "can": args.can,
        "side": args.side,
        "deadman_button": f"{args.side}Grip",
        "calibrate_button": "A" if args.side == "right" else "X",
        "scale": args.scale,
        "max_speed_m_s": args.max_speed,
    }
    if args.quest_ip:
        config["quest"] = {"connection": "wireless", "ip_address": args.quest_ip}

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(config, handle, sort_keys=False)
    print(f"Wrote {output}")
    print("Controls: press the calibration button, release the grip, then press and hold the grip to move.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
