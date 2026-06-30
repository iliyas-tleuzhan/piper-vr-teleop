#!/usr/bin/env python3
"""Test Quest controller rotation to Piper wrist joints without right trigger."""

from __future__ import annotations

import argparse
import time

import numpy as np

from piper_vr.buttons import is_pressed
from piper_vr.config import load_config
from piper_vr.frame_calibration import ControlFrameConfig, controller_delta_in_control_frame, get_control_frame
from piper_vr.joint_limits import clamp_joints_deg, rate_limit_joints_deg
from piper_vr.joint_mimic import JointMimicConfig
from piper_vr.piper_driver import PiperDriver
from piper_vr.quest_reader import QuestReader


def _fmt(values: np.ndarray | None, digits: int = 3) -> str:
    if values is None:
        return "None"
    return np.asarray(values, dtype=float).round(digits).tolist().__repr__()


def main() -> int:
    parser = argparse.ArgumentParser(description="Predict or command wrist joints from controller rotation")
    parser.add_argument("--config", default="configs/single_piper.yaml")
    parser.add_argument("--side", choices=("left", "right"), default=None)
    parser.add_argument("--can", default=None)
    parser.add_argument("--robot", action="store_true", help="connect Piper and command joints 4-6 only")
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--speed-percent", type=int, default=None)
    parser.add_argument("--dry-run", action="store_true", default=True)
    args = parser.parse_args()

    config = load_config(args.config)
    side = args.side or config.get("side", "right")
    button = config.get("calibrate_button", "A")
    deadman = config.get("deadman_button", "rightGrip")
    mimic = JointMimicConfig.from_config(config.get("joint_mimic"))
    quest_config = config.get("quest", {})
    quest = QuestReader(
        transport=quest_config.get("transport", "adb_logcat"),
        connection=quest_config.get("connection", "usb"),
        ip_address=quest_config.get("ip_address"),
        simulate_on_missing=not args.robot,
    )

    driver = None
    fixed_joints = mimic.neutral_deg.copy()
    last_command = fixed_joints.copy()
    if args.robot:
        driver = PiperDriver(
            can=args.can or config.get("can", "can0"),
            speed_percent=args.speed_percent or int(config.get("speed_percent", 100)),
            dry_run=False,
        )
        driver.connect(initial_mode="joint")
        measured = driver.read_joint_pose()
        if measured is None:
            raise RuntimeError("Piper joint feedback is required for --robot")
        fixed_joints = measured.joints_deg.copy()
        last_command = fixed_joints.copy()

    print(f"Press {button} to calibrate orientation. Hold {deadman} and rotate the controller. rightTrig is not required.")
    home = None
    control_frame = None
    previous_transform = None
    previous_pressed = False
    filtered_delta_rot = np.zeros(3)
    period_s = 1.0 / args.hz
    last_s = time.monotonic()
    try:
        while True:
            loop_start = time.monotonic()
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
                filtered_delta_rot = np.zeros(3)
                print("Orientation calibrated.")
            previous_pressed = pressed
            if home is None or previous_transform is None or control_frame is None:
                time.sleep(0.05)
                continue
            if not is_pressed(sample.buttons, deadman):
                previous_transform = np.asarray(transform, dtype=float).copy()
                filtered_delta_rot = np.zeros(3)
                print("Grip released: holding/stopped.")
                time.sleep(0.2)
                continue

            _, delta_rot = controller_delta_in_control_frame(previous_transform, transform, control_frame)
            previous_transform = np.asarray(transform, dtype=float).copy()
            rotation_deadband = mimic.rotation_deadband_deg if mimic.wrist_rotation_deadband_deg is None else mimic.wrist_rotation_deadband_deg
            if float(np.linalg.norm(delta_rot)) < rotation_deadband:
                delta_rot = np.zeros(3)
            if np.allclose(delta_rot, 0.0):
                filtered_delta_rot = np.zeros(3)
            else:
                filtered_delta_rot = filtered_delta_rot + mimic.wrist_rotation_filter_alpha * (delta_rot - filtered_delta_rot)
                delta_rot = filtered_delta_rot.copy()

            u = np.concatenate((np.zeros(3), delta_rot))
            dq = mimic.relative_gain_matrix @ u
            target = last_command.copy()
            target[3:6] = fixed_joints[3:6] + dq[3:6]
            target[:3] = fixed_joints[:3]
            now_s = time.monotonic()
            dt = max(now_s - last_s, 1e-3)
            last_s = now_s
            safe = rate_limit_joints_deg(clamp_joints_deg(target), last_command, mimic.max_joint_speed_deg_s, dt)
            safe[:3] = fixed_joints[:3]
            if driver is not None:
                driver.send_joint_pose(safe)
            last_command = safe.copy()
            print(f"delta_rot_deg={_fmt(delta_rot, 3)} predicted_wrist_dq={_fmt(dq[3:6], 2)} target_wrist={_fmt(safe[3:6], 2)}")
            time.sleep(max(0.0, period_s - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        if driver is not None:
            driver.hold_joints(allow_last_command_fallback=True)
        return 0
    finally:
        quest.stop()


if __name__ == "__main__":
    raise SystemExit(main())
