#!/usr/bin/env python3
"""Debug calibration-relative joint mimic mapping without robot connection."""

from __future__ import annotations

import argparse
import time

import numpy as np
import yaml

from piper_vr.human_arm_model import HumanArmConfig, build_human_arm_state
from piper_vr.joint_mimic import JointMimicConfig, human_arm_to_mimic_vector_deg, mimic_vector_to_piper_joints
from piper_vr.quest_reader import QuestReader


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/single_piper.yaml")
    parser.add_argument("--side", choices=("left", "right"), default=None)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--calibrate", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    side = args.side or config.get("side", "right")
    quest_config = config.get("quest", {})
    quest = QuestReader(
        transport=quest_config.get("transport", "adb_logcat"),
        connection=quest_config.get("connection", "usb"),
        ip_address=quest_config.get("ip_address"),
        simulate_on_missing=args.dry_run,
    )
    human_config = HumanArmConfig.from_config({**config.get("human_arm", {}), "side": side})
    mimic_config = JointMimicConfig.from_config(config.get("joint_mimic"))
    robot_home = mimic_config.neutral_deg.copy()
    shoulder = None
    human_home = None
    previous_elbow = None
    end_s = time.monotonic() + args.seconds

    while time.monotonic() < end_s:
        sample = quest.get_sample()
        transform = None if sample is None else sample.transforms_openxr.get(side)
        if transform is None:
            print("No controller sample")
            time.sleep(0.2)
            continue
        if shoulder is None or args.calibrate and human_home is None:
            shoulder = transform[:3, 3] + human_config.fixed_shoulder_from_hand_home_m
        human = build_human_arm_state(shoulder, transform, human_config.elbow_swivel_default_rad, human_config, previous_elbow)
        previous_elbow = human.elbow_xyz_m
        vector = human_arm_to_mimic_vector_deg(human)
        if human_home is None:
            human_home = vector.copy()
        delta = vector - human_home
        target = mimic_vector_to_piper_joints(vector, human_home, robot_home, mimic_config)
        print(
            f"human_vector={vector.round(2).tolist()} "
            f"human_home={human_home.round(2).tolist()} "
            f"human_delta={delta.round(2).tolist()} "
            f"robot_home={robot_home.round(2).tolist()} "
            f"target_piper={target.round(2).tolist()} "
            f"shoulder={human.shoulder_xyz_m.round(3).tolist()} "
            f"elbow={human.elbow_xyz_m.round(3).tolist()} "
            f"wrist={human.wrist_xyz_m.round(3).tolist()}"
        )
        time.sleep(0.2)
    quest.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
