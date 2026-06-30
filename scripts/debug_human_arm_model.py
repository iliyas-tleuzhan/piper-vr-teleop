#!/usr/bin/env python3
"""Print inferred human arm posture and mapped Piper joints without robot connection."""

from __future__ import annotations

import argparse
import time

import yaml

from piper_vr.human_arm_model import HumanArmConfig, build_human_arm_state
from piper_vr.joint_mimic import JointMimicConfig, human_arm_to_piper_joints
from piper_vr.quest_reader import QuestReader


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/single_piper.yaml")
    parser.add_argument("--side", choices=("left", "right"), default=None)
    parser.add_argument("--seconds", type=float, default=30.0)
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

    shoulder = None
    previous_elbow = None
    end_s = time.monotonic() + args.seconds
    while time.monotonic() < end_s:
        sample = quest.get_sample()
        transform = None if sample is None else sample.transforms_openxr.get(side)
        if transform is None:
            print("No controller sample")
            time.sleep(0.2)
            continue
        if shoulder is None:
            shoulder = transform[:3, 3] + human_config.fixed_shoulder_from_hand_home_m
        human = build_human_arm_state(shoulder, transform, human_config.elbow_swivel_default_rad, human_config, previous_elbow)
        previous_elbow = human.elbow_xyz_m
        joints = human_arm_to_piper_joints(human, mimic_config)
        print(
            f"shoulder={human.shoulder_xyz_m.round(3).tolist()} "
            f"elbow={human.elbow_xyz_m.round(3).tolist()} "
            f"wrist={human.wrist_xyz_m.round(3).tolist()} "
            f"shoulder_deg={human.shoulder_angles_deg.round(2).tolist()} "
            f"elbow_flex={human.elbow_flex_deg:.2f} "
            f"wrist_deg={human.wrist_angles_deg.round(2).tolist()} "
            f"piper_deg={joints.round(2).tolist()}"
        )
        time.sleep(0.2)
    quest.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
