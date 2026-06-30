"""Mode-aware Quest 3 to AgileX Piper teleoperation entrypoint."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

from .human_arm_model import HumanArmConfig
from .joint_mimic import JointMimicConfig
from .movep_teleop import main as endpoint_main
from .piper_driver import PiperDriver
from .quest_reader import QuestReader
from .session import JointMimicSession


def load_config(path: str | Path) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Quest 3 to AgileX Piper teleoperation")
    parser.add_argument("--config", default="configs/single_piper.yaml")
    parser.add_argument("--control-mode", choices=("joint_mimic", "endpoint_firmware", "external_ik"))
    parser.add_argument("--can")
    parser.add_argument("--hz", type=float)
    parser.add_argument("--speed-percent", type=int)
    parser.add_argument("--scale", type=float)
    parser.add_argument("--max-speed", type=float)
    parser.add_argument("--max-joint-speed", type=float)
    parser.add_argument("--deadman-button")
    parser.add_argument("--calibrate-button")
    parser.add_argument("--side", choices=("left", "right"))
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--no-urdf-guard", action="store_true")
    parser.add_argument("--orientation-enabled", action="store_true")
    parser.add_argument("--joint-mimic-only", action="store_true")
    parser.add_argument("--endpoint-fallback", action="store_true")
    parser.add_argument("--log", action="store_true")
    parser.add_argument("--log-dir", default="logs/joint_mimic")
    return parser


def _apply_common_overrides(config: dict, args: argparse.Namespace) -> dict:
    for attr, key in (
        ("control_mode", "control_mode"),
        ("can", "can"),
        ("hz", "hz"),
        ("speed_percent", "speed_percent"),
        ("side", "side"),
        ("deadman_button", "deadman_button"),
        ("calibrate_button", "calibrate_button"),
    ):
        value = getattr(args, attr)
        if value is not None:
            config[key] = value
    if args.max_joint_speed is not None:
        mimic = config.setdefault("joint_mimic", {})
        mimic["max_joint_speed_deg_s"] = [float(args.max_joint_speed)] * 6
    return config


def _format_array(value: np.ndarray | None, digits: int = 3) -> str:
    if value is None:
        return "None"
    return np.asarray(value, dtype=float).round(digits).tolist().__repr__()


def _print_joint_verbose(result, driver: PiperDriver, transport_name: str) -> None:
    human = result.human_arm
    measured = result.measured_joints or driver.read_joint_pose()
    print(
        "[verbose] "
        f"state={result.state.value} "
        f"calibrated={result.calibrated} "
        f"controller_xyz={_format_array(result.controller_xyz)} "
        f"shoulder_xyz={_format_array(None if human is None else human.shoulder_xyz_m)} "
        f"elbow_xyz={_format_array(None if human is None else human.elbow_xyz_m)} "
        f"wrist_xyz={_format_array(None if human is None else human.wrist_xyz_m)} "
        f"human_shoulder_deg={_format_array(None if human is None else human.shoulder_angles_deg, 2)} "
        f"elbow_flex_deg={None if human is None else round(human.elbow_flex_deg, 2)} "
        f"wrist_deg={_format_array(None if human is None else human.wrist_angles_deg, 2)} "
        f"human_vector_deg={_format_array(result.human_vector_deg, 2)} "
        f"human_delta_deg={_format_array(result.human_delta_deg, 2)} "
        f"robot_home_joints_deg={_format_array(result.robot_home_joints_deg, 2)} "
        f"target_joints_deg={_format_array(result.raw_joint_target_deg, 2)} "
        f"safe_joints_deg={_format_array(result.safe_joint_target_deg, 2)} "
        f"action={'JointCtrl sent' if result.action == 'sent' else 'skipped:' + result.reason} "
        f"measured_joints_deg={_format_array(None if measured is None else measured.joints_deg, 2)} "
        f"quest_age_s={None if result.sample_age_s is None else round(result.sample_age_s, 3)} "
        f"transport={transport_name}"
    )


def command_joint_hold_on_exit(driver: PiperDriver) -> bool:
    try:
        driver.hold_joints(allow_last_command_fallback=True)
    except RuntimeError as exc:
        print(f"WARNING: could not command joint hold on exit: {exc}")
        return False
    return True


class JointMimicJsonlLogger:
    def __init__(self, log_dir: str | Path) -> None:
        directory = Path(log_dir)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = directory / f"joint_mimic_{stamp}.jsonl"
        self._handle = self.path.open("a", encoding="utf-8")

    def write(self, result) -> None:
        human = result.human_arm

        def arr(value):
            return None if value is None else np.asarray(value, dtype=float).round(6).tolist()

        row = {
            "timestamp": time.time(),
            "state": result.state.value,
            "controller_xyz": arr(result.controller_xyz),
            "shoulder_xyz": arr(None if human is None else human.shoulder_xyz_m),
            "elbow_xyz": arr(None if human is None else human.elbow_xyz_m),
            "wrist_xyz": arr(None if human is None else human.wrist_xyz_m),
            "human_vector_deg": arr(result.human_vector_deg),
            "human_home_vector_deg": arr(result.human_home_vector_deg),
            "human_delta_deg": arr(result.human_delta_deg),
            "robot_home_joints_deg": arr(result.robot_home_joints_deg),
            "target_joints_deg": arr(result.raw_joint_target_deg),
            "safe_joints_deg": arr(result.safe_joint_target_deg),
            "measured_joints_deg": arr(None if result.measured_joints is None else result.measured_joints.joints_deg),
            "action": result.action,
            "reason": result.reason,
        }
        self._handle.write(json.dumps(row, separators=(",", ":")) + "\n")
        self._handle.flush()

    def close(self) -> None:
        self._handle.close()


def _run_joint_mimic(args: argparse.Namespace, config: dict) -> int:
    hz = float(config.get("hz", 30.0))
    period_s = 1.0 / hz
    side = config.get("side", "right")
    deadman_button = config.get("deadman_button", "rightGrip")
    calibrate_button = config.get("calibrate_button", "A")
    quest_config = config.get("quest", {})
    transport_name = quest_config.get("transport", "adb_logcat")

    print("Piper Quest VR teleoperation")
    print(f"Control mode: joint_mimic, robot: {'dry-run' if args.dry_run else 'REAL ROBOT'}")
    print(f"Transport: {transport_name}, controller: {side}, deadman: {deadman_button}, calibrate: {calibrate_button}")
    if not args.dry_run:
        print("WARNING: real robot joint mode can move all six joints. Keep clear and tune at low speed.")
        time.sleep(2.0)

    quest = QuestReader(
        transport=transport_name,
        connection=quest_config.get("connection", "usb"),
        ip_address=quest_config.get("ip_address"),
        simulate_on_missing=args.dry_run,
    )
    driver = PiperDriver(can=config.get("can", "can0"), speed_percent=int(config.get("speed_percent", 5)), dry_run=args.dry_run)
    driver.connect(initial_mode="joint")
    measured = driver.read_joint_pose()
    if measured is not None:
        print(f"Measured Piper joints: {measured.joints_deg.round(2).tolist()}")
    else:
        print("WARNING: Piper joint feedback unavailable. Real joint mimic calibration will be refused until feedback works.")

    human_config = HumanArmConfig.from_config({**config.get("human_arm", {}), "side": side})
    mimic_config = JointMimicConfig.from_config(config.get("joint_mimic"))
    session = JointMimicSession(
        side=side,
        deadman_button=deadman_button,
        calibrate_button=calibrate_button,
        human_config=human_config,
        mimic_config=mimic_config,
        stale_timeout_s=float(config.get("stale_timeout_s", 0.25)),
        elbow_swivel_joystick=config.get("human_arm", {}).get("elbow_swivel_joystick", "rightJS_x"),
        shoulder_lift_joystick=config.get("human_arm", {}).get("shoulder_lift_joystick"),
    )

    next_verbose_s = 0.0
    logger = JointMimicJsonlLogger(args.log_dir) if args.log else None
    if logger is not None:
        print(f"Logging joint mimic JSONL to {logger.path}")
    try:
        while True:
            loop_start = time.monotonic()
            result = session.step(quest.get_sample(), driver)
            if logger is not None:
                logger.write(result)
            if args.verbose and time.monotonic() >= next_verbose_s:
                next_verbose_s = time.monotonic() + 0.2
                _print_joint_verbose(result, driver, transport_name)
            time.sleep(max(0.0, period_s - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        print("\nCtrl+C received. Holding at the measured joint pose.")
        command_joint_hold_on_exit(driver)
        return 0
    finally:
        quest.stop()
        if logger is not None:
            logger.close()


def main() -> int:
    args = build_parser().parse_args()
    config = _apply_common_overrides(load_config(args.config), args)
    mode = config.get("control_mode", "joint_mimic")
    if mode == "joint_mimic":
        return _run_joint_mimic(args, config)
    if mode in ("endpoint_firmware", "external_ik"):
        if mode == "external_ik":
            print("external_ik is available as a library path; endpoint_firmware is used as the runtime fallback until tuned.")
        return endpoint_main()
    raise ValueError(f"Unsupported control_mode: {mode!r}")


if __name__ == "__main__":
    raise SystemExit(main())
