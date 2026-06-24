"""Single Piper Quest controller endpoint teleoperation."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import yaml

from .buttons import analog_value, is_pressed
from .piper_driver import PiperDriver
from .quest_reader import QuestReader
from .safety import SafetyLimiter, tracking_is_stale
from .vr_mapping import AxisMapping, target_from_home


def load_config(path: str | Path) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def apply_overrides(config: dict, args: argparse.Namespace) -> dict:
    for attr, key in (
        ("can", "can"),
        ("hz", "hz"),
        ("speed_percent", "speed_percent"),
        ("scale", "scale"),
        ("max_speed", "max_speed_m_s"),
        ("side", "side"),
        ("deadman_button", "deadman_button"),
    ):
        value = getattr(args, attr)
        if value is not None:
            config[key] = value
    if args.gripper:
        config["gripper_enabled"] = True
    if args.quest_ip:
        config.setdefault("quest", {})["ip_address"] = args.quest_ip
        config["quest"]["connection"] = "wireless"
    return config


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Quest 3 to AgileX Piper endpoint teleoperation")
    parser.add_argument("--config", default="configs/single_piper.yaml")
    parser.add_argument("--can")
    parser.add_argument("--hz", type=float)
    parser.add_argument("--speed-percent", type=int)
    parser.add_argument("--scale", type=float)
    parser.add_argument("--max-speed", type=float)
    parser.add_argument("--side", choices=("left", "right"))
    parser.add_argument("--deadman-button")
    parser.add_argument("--gripper", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--quest-ip")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    config = apply_overrides(load_config(args.config), args)
    hz = float(config.get("hz", 20.0))
    period_s = 1.0 / hz
    side = config.get("side", "right")
    deadman_button = config.get("deadman_button", "B")
    calibrate_button = config.get("calibrate_button", "A")
    quest_ip = config.get("quest", {}).get("ip_address")

    print("Piper Quest VR teleoperation")
    print(f"Mode: {'dry-run' if args.dry_run else 'REAL ROBOT'}")
    print(f"Controller: {side}, deadman: {deadman_button}, calibrate: {calibrate_button}")
    if not args.dry_run:
        print("WARNING: real robot mode can move the arm. Keep clear and hold the deadman only when ready.")
        time.sleep(2.0)

    quest = QuestReader(ip_address=quest_ip, simulate_on_missing=args.dry_run)
    driver = PiperDriver(
        can=config.get("can", "can0"),
        speed_percent=int(config.get("speed_percent", 10)),
        dry_run=args.dry_run,
    )
    driver.connect()
    safety = SafetyLimiter.from_config(config)
    mapping = AxisMapping.from_config(config.get("axis_mapping"))
    scale = float(config.get("scale", 0.35))

    calibrated = False
    vr_home = None
    piper_home = None
    rpy_home = None
    was_calibrate_pressed = False

    try:
        while True:
            loop_start = time.monotonic()
            buttons = quest.get_buttons()
            deadman = is_pressed(buttons, deadman_button)
            calibrate_pressed = is_pressed(buttons, calibrate_button)

            if calibrate_pressed and not was_calibrate_pressed:
                vr_home = quest.get_controller_pose(side)
                current_pose = driver.read_end_pose()
                piper_home = current_pose.xyz_m.copy()
                if config.get("hold_orientation", True):
                    rpy_home = current_pose.rpy_deg.copy()
                else:
                    rpy_home = np.asarray(config.get("default_rpy_deg") or current_pose.rpy_deg, dtype=float)
                safety.last_command_m = piper_home.copy()
                safety.last_time_s = time.monotonic()
                calibrated = True
                print(f"Calibrated: piper_home_m={piper_home.round(4).tolist()} rpy_deg={rpy_home.round(2).tolist()}")
            was_calibrate_pressed = calibrate_pressed

            if not calibrated:
                print("Waiting for calibration. Press the calibrate button while the robot is in a safe pose.", end="\r")
                time.sleep(period_s)
                continue

            if tracking_is_stale(quest.last_update_s, safety.stale_timeout_s):
                print("Quest tracking is stale or missing; holding position.", end="\r")
                time.sleep(period_s)
                continue

            if not deadman:
                time.sleep(period_s)
                continue

            current_vr = quest.get_controller_pose(side)
            target = target_from_home(vr_home, current_vr, piper_home, mapping, scale)
            safe_target = safety.limit_step(target)
            driver.send_end_pose(safe_target, rpy_home)

            if config.get("gripper_enabled", False):
                trigger_key = "rightTrig" if side == "right" else "leftTrig"
                driver.send_gripper(analog_value(buttons, trigger_key) * 0.08)

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, period_s - elapsed))
    except KeyboardInterrupt:
        print("\nCtrl+C received. Holding the last endpoint command.")
        driver.hold()
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
