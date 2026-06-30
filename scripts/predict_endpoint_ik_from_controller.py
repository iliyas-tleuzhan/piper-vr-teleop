#!/usr/bin/env python3
"""Predict Quest endpoint IK targets from controller motion without sending robot commands."""

from __future__ import annotations

import argparse
import time

import numpy as np

from piper_vr.buttons import is_pressed
from piper_vr.config import deep_merge, load_config
from piper_vr.frame_calibration import ControlFrameConfig, get_control_frame
from piper_vr.piper_driver import PiperDriver
from piper_vr.piper_official_kinematics import create_official_fk
from piper_vr.piper_kinematics import PiperKinematics
from piper_vr.quest_endpoint_ik import QuestEndpointIKConfig, endpoint_target_from_controller, solve_endpoint_ik
from piper_vr.quest_reader import QuestReader
from piper_vr.vr_mapping import _matrix_to_rpy_deg


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
    parser = argparse.ArgumentParser(description="Predict endpoint IK targets from Quest controller motion")
    parser.add_argument("--config", default="configs/single_piper.yaml")
    parser.add_argument("--side", choices=("left", "right"), default=None)
    parser.add_argument("--robot", action="store_true", help="read real Piper FK/IK seed")
    parser.add_argument("--send", action="store_true", help="with --robot, send solved joint commands")
    parser.add_argument("--can", default=None)
    args = parser.parse_args()

    config = _load_with_base(args.config)
    side = args.side or config.get("side", "right")
    button = config.get("calibrate_button", "A")
    quest_config = config.get("quest", {})
    ik_config = QuestEndpointIKConfig.from_config(config.get("quest_endpoint_ik"))
    if ik_config.backend == "host_ik_urdf":
        kinematics = PiperKinematics(ik_config.urdf_path, tip_link=ik_config.ee_frame)
    elif ik_config.backend == "host_ik_sdk_fk":
        kinematics = create_official_fk(prefer_sdk=True)
    else:
        kinematics = None
    quest = QuestReader(
        transport=quest_config.get("transport", "adb_logcat"),
        connection=quest_config.get("connection", "usb"),
        ip_address=quest_config.get("ip_address"),
        simulate_on_missing=not args.robot,
    )

    driver = None
    robot_home_joints = np.array([0.0, 90.0, -90.0, 0.0, 0.0, 0.0], dtype=float)
    if args.robot:
        driver = PiperDriver(can=args.can or config.get("can", "can0"), speed_percent=int(config.get("speed_percent", 10)), dry_run=False)
        driver.connect(initial_mode="joint")
        measured = driver.read_joint_pose()
        if measured is None:
            raise RuntimeError("Piper joint feedback is required with --robot")
        robot_home_joints = measured.joints_deg.copy()
    if args.robot and driver is not None and ik_config.backend == "firmware_endpoint":
        pose = driver.read_end_pose()
        robot_home_xyz = pose.xyz_m.copy()
        robot_home_rpy = pose.rpy_deg.copy()
    elif kinematics is not None:
        robot_home_xyz, robot_home_rotation = kinematics.forward(np.radians(robot_home_joints))
        robot_home_rpy = _matrix_to_rpy_deg(robot_home_rotation)
    else:
        robot_home_xyz = np.array([0.35, 0.0, 0.25], dtype=float)
        robot_home_rpy = np.zeros(3)

    print(f"Press {button} to set controller home. Robot home xyz={_fmt(robot_home_xyz)} rpy={_fmt(robot_home_rpy, 2)}")
    home = None
    control_frame = None
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
                control_frame = get_control_frame(sample, side, ControlFrameConfig(source=ik_config.control_frame), home)
                print(f"HOME set at controller xyz={_fmt(home[:3, 3])}")
            previous_pressed = pressed
            if home is None:
                time.sleep(0.05)
                continue
            target_xyz, target_rpy, debug = endpoint_target_from_controller(home, transform, robot_home_xyz, robot_home_rpy, ik_config, control_frame=control_frame)
            print(
                f"control_frame={ik_config.control_frame} "
                f"controller_delta_xyz={_fmt(debug['controller_delta_xyz'], 4)} "
                f"mapped_robot_delta_xyz={_fmt(debug['mapped_robot_delta_xyz'], 4)} "
                f"scaled_robot_delta_xyz={_fmt(debug['scaled_robot_delta_xyz'], 4)} "
                f"target_before_home_clamp={_fmt(debug['target_before_home_clamp'], 4)} "
                f"target_after_home_clamp={_fmt(debug['target_after_home_clamp'], 4)} "
                f"clamped_axes={debug['clamped_axes']} "
                f"target_xyz={_fmt(target_xyz, 4)} "
                f"controller_delta_rpy={_fmt(debug['controller_delta_rpy_deg'], 2)} "
                f"target_rpy={_fmt(target_rpy, 2)}"
            )
            if args.robot and ik_config.backend == "firmware_endpoint":
                print("  firmware_endpoint target only; Piper firmware performs IK internally")
                if args.send and driver is not None:
                    driver.send_end_pose(target_xyz, target_rpy)
            elif args.robot and kinematics is not None:
                result = solve_endpoint_ik(kinematics, target_xyz, target_rpy, robot_home_joints, robot_home_joints, ik_config)
                print(f"  ik success={result.success} q_deg={_fmt(result.joints_deg, 2)} error=({result.position_error_m:.4f},{result.orientation_error_deg:.2f})")
                if args.send and driver is not None and result.success:
                    driver.send_joint_pose(result.joints_deg)
            time.sleep(0.1)
    except KeyboardInterrupt:
        return 0
    finally:
        quest.stop()


if __name__ == "__main__":
    raise SystemExit(main())
