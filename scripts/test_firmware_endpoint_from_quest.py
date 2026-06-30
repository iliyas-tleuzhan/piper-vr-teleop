#!/usr/bin/env python3
"""Test Quest relative endpoint targets through Piper firmware MOVE P commands."""

from __future__ import annotations

import argparse
import time

import numpy as np

from piper_vr.buttons import is_pressed
from piper_vr.config import deep_merge, load_config
from piper_vr.piper_driver import PiperDriver
from piper_vr.quest_endpoint_ik import QuestEndpointIKConfig, endpoint_target_from_controller
from piper_vr.quest_reader import QuestReader
from piper_vr.units import degrees_to_piper_rpy, meters_to_piper_xyz


def _fmt(values: np.ndarray | None, digits: int = 3) -> str:
    if values is None:
        return "None"
    return np.asarray(values, dtype=float).round(digits).tolist().__repr__()


def _load_with_base(path: str) -> dict:
    config = load_config(path)
    if "quest" not in config or "quest_endpoint_ik" not in config:
        return deep_merge(load_config("configs/single_piper.yaml"), config)
    return config


def main() -> int:
    parser = argparse.ArgumentParser(description="Print or send firmware endpoint targets from Quest motion")
    parser.add_argument("--config", default="configs/single_piper.yaml")
    parser.add_argument("--side", choices=("left", "right"), default=None)
    parser.add_argument("--robot", action="store_true")
    parser.add_argument("--send", action="store_true")
    parser.add_argument("--can", default=None)
    parser.add_argument("--scale", type=float, default=None)
    parser.add_argument("--speed-percent", type=int, default=10)
    args = parser.parse_args()

    config = _load_with_base(args.config)
    side = args.side or config.get("side", "right")
    button = config.get("calibrate_button", "A")
    quest_config = config.get("quest", {})
    ik_patch = dict(config.get("quest_endpoint_ik", {}))
    ik_patch["backend"] = "firmware_endpoint"
    ik_patch["orientation_enabled"] = bool(ik_patch.get("orientation_enabled", False))
    if args.scale is not None:
        ik_patch["scale"] = float(args.scale)
    ik_config = QuestEndpointIKConfig.from_config(ik_patch)

    quest = QuestReader(
        transport=quest_config.get("transport", "adb_logcat"),
        connection=quest_config.get("connection", "usb"),
        ip_address=quest_config.get("ip_address"),
        simulate_on_missing=not args.robot,
    )
    driver = None
    robot_home_xyz = np.array([0.35, 0.0, 0.25], dtype=float)
    robot_home_rpy = np.zeros(3)
    if args.robot:
        driver = PiperDriver(can=args.can or config.get("can", "can0"), speed_percent=args.speed_percent, dry_run=False)
        driver.connect(initial_mode="endpoint")
        pose = driver.read_end_pose()
        if pose is None:
            raise RuntimeError("Piper endpoint feedback is required with --robot")
        robot_home_xyz = pose.xyz_m.copy()
        robot_home_rpy = pose.rpy_deg.copy()

    print(f"Press {button} to set controller home. position-only={not ik_config.orientation_enabled} scale={ik_config.scale}")
    home = None
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
                if driver is not None:
                    pose = driver.read_end_pose()
                    if pose is not None:
                        robot_home_xyz = pose.xyz_m.copy()
                        robot_home_rpy = pose.rpy_deg.copy()
                print(f"HOME set: robot_xyz={_fmt(robot_home_xyz, 4)} robot_rpy={_fmt(robot_home_rpy, 2)}")
            previous_pressed = pressed
            if home is None:
                time.sleep(0.05)
                continue
            target_xyz, target_rpy, debug = endpoint_target_from_controller(home, transform, robot_home_xyz, robot_home_rpy, ik_config)
            xyz_mm = target_xyz * 1000.0
            xyz_raw = [meters_to_piper_xyz(float(v)) for v in target_xyz]
            rpy_raw = [degrees_to_piper_rpy(float(v)) for v in target_rpy]
            print(
                f"delta={_fmt(debug['mapped_robot_delta_xyz'], 4)} "
                f"XYZ_mm={_fmt(xyz_mm, 2)} XYZ_raw_0.001mm={xyz_raw} "
                f"RPY_deg={_fmt(target_rpy, 2)} RPY_raw_0.001deg={rpy_raw}"
            )
            if args.robot and args.send and driver is not None:
                driver.send_end_pose(target_xyz, target_rpy)
            time.sleep(0.1)
    except KeyboardInterrupt:
        if driver is not None:
            driver.hold()
        return 0
    finally:
        quest.stop()


if __name__ == "__main__":
    raise SystemExit(main())
