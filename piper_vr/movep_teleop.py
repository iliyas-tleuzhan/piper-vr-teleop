"""Single Piper Quest controller endpoint teleoperation."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import yaml

from .buttons import analog_value
from .piper_driver import PiperDriver
from .piper_kinematics import PiperKinematics
from .quest_reader import QuestReader
from .safety import OrientationLimiter, SafetyLimiter, SignalFilter
from .session import TeleopSession
from .vr_mapping import AxisMapping


def load_config(path: str | Path) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def apply_overrides(config: dict, args: argparse.Namespace) -> dict:
    endpoint = dict(config.get("endpoint_firmware", {}))
    for key in (
        "scale",
        "max_speed_m_s",
        "max_position_jump_m",
        "workspace_min_m",
        "workspace_max_m",
        "axis_mapping",
    ):
        if key in endpoint:
            config[key] = endpoint[key]
    for attr, key in (
        ("can", "can"),
        ("hz", "hz"),
        ("speed_percent", "speed_percent"),
        ("control_mode", "control_mode"),
        ("scale", "scale"),
        ("max_speed", "max_speed_m_s"),
        ("side", "side"),
        ("deadman_button", "deadman_button"),
        ("calibrate_button", "calibrate_button"),
        ("stale_timeout", "stale_timeout_s"),
        ("position_deadband", "position_deadband_m"),
        ("position_filter_alpha", "position_filter_alpha"),
    ):
        value = getattr(args, attr)
        if value is not None:
            config[key] = value
    if args.gripper:
        config["gripper_enabled"] = True
    if args.orientation_enabled:
        config["orientation_enabled"] = True
    if args.urdf_guard:
        config["urdf_guard_enabled"] = True
    if args.no_urdf_guard:
        config["urdf_guard_enabled"] = False
    quest = config.setdefault("quest", {})
    if args.transport:
        quest["transport"] = args.transport
    if args.quest_ip:
        quest["ip_address"] = args.quest_ip
        quest["connection"] = "wireless"
    return config


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Quest 3 to AgileX Piper endpoint teleoperation")
    parser.add_argument("--config", default="configs/single_piper.yaml")
    parser.add_argument("--can")
    parser.add_argument("--hz", type=float)
    parser.add_argument("--speed-percent", type=int)
    parser.add_argument("--control-mode", choices=("joint_mimic", "endpoint_firmware", "external_ik"), default=None)
    parser.add_argument("--scale", type=float)
    parser.add_argument("--max-speed", type=float)
    parser.add_argument("--side", choices=("left", "right"))
    parser.add_argument("--deadman-button")
    parser.add_argument("--calibrate-button")
    parser.add_argument("--gripper", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--transport", default=None)
    parser.add_argument("--quest-ip")
    parser.add_argument("--stale-timeout", type=float)
    parser.add_argument("--position-deadband", type=float)
    parser.add_argument("--position-filter-alpha", type=float)
    parser.add_argument("--orientation-enabled", action="store_true")
    parser.add_argument("--urdf-guard", action="store_true", help="enable optional constrained URDF IK feasibility check")
    parser.add_argument("--no-urdf-guard", action="store_true", help="disable optional URDF IK feasibility check")
    return parser


def _format_xyz(value: np.ndarray | None) -> str:
    if value is None:
        return "None"
    return np.asarray(value, dtype=float).round(4).tolist().__repr__()


def _print_verbose(result, driver: PiperDriver, transport_name: str) -> None:
    piper_pose = driver.read_end_pose()
    piper_xyz = None if piper_pose is None else piper_pose.xyz_m
    action = result.action if result.action == "sent" else f"skipped:{result.reason}"
    print(
        "[verbose] "
        f"state={result.state.value} "
        f"calibrated={result.calibrated} "
        f"deadman={result.deadman} "
        f"calibrate={result.calibrate} "
        f"controller_xyz={_format_xyz(result.controller_xyz)} "
        f"raw_target_xyz={_format_xyz(result.raw_target_xyz)} "
        f"safe_target_xyz={_format_xyz(result.safe_target_xyz)} "
        f"action={action} "
        f"piper_xyz={_format_xyz(piper_xyz)} "
        f"quest_age_s={None if result.sample_age_s is None else round(result.sample_age_s, 3)} "
        f"transport={transport_name}"
    )


def main() -> int:
    args = build_parser().parse_args()
    config = apply_overrides(load_config(args.config), args)
    if config.get("control_mode", "endpoint_firmware") not in ("endpoint_firmware", "external_ik"):
        print("movep_teleop.py is endpoint-oriented. Use `python -m piper_vr.vr_teleop --control-mode joint_mimic` for joint mimic.")
        return 2
    hz = float(config.get("hz", 30.0))
    period_s = 1.0 / hz
    side = config.get("side", "right")
    deadman_button = config.get("deadman_button", "rightGrip")
    calibrate_button = config.get("calibrate_button", "A")
    quest_config = config.get("quest", {})
    transport_name = quest_config.get("transport", "adb_logcat")

    print("Piper Quest VR teleoperation")
    print(f"Mode: {'dry-run' if args.dry_run else 'REAL ROBOT'}")
    print(f"Transport: {transport_name}, controller: {side}, deadman: {deadman_button}, calibrate: {calibrate_button}")
    if not args.dry_run:
        print("WARNING: real robot mode can move the arm. Keep clear and hold the deadman only when ready.")
        time.sleep(2.0)

    quest = QuestReader(
        transport=transport_name,
        connection=quest_config.get("connection", "usb"),
        ip_address=quest_config.get("ip_address"),
        simulate_on_missing=args.dry_run,
    )
    diagnostics = quest.diagnostics()
    if diagnostics is not None:
        print(f"Quest transport module: {diagnostics.module_path}")
        print(f"Quest connection: {diagnostics.connection}, ip={diagnostics.ip_address}")

    driver = PiperDriver(
        can=config.get("can", "can0"),
        speed_percent=int(config.get("speed_percent", 5)),
        dry_run=args.dry_run,
    )
    driver.connect(initial_mode="endpoint")
    measured = driver.read_end_pose()
    if measured is not None:
        print(f"Measured Piper endpoint: xyz_m={measured.xyz_m.round(4).tolist()} rpy_deg={measured.rpy_deg.round(2).tolist()}")

    safety = SafetyLimiter.from_config(config)
    session = TeleopSession(
        side=side,
        deadman_button=deadman_button,
        calibrate_button=calibrate_button,
        scale=float(config.get("scale", 0.40)),
        mapping=AxisMapping.from_config(config.get("axis_mapping")),
        safety=safety,
        position_filter=SignalFilter(
            deadband=float(config.get("position_deadband_m", 0.003)),
            alpha=float(config.get("position_filter_alpha", 0.35)),
            enabled=bool(config.get("position_filter_enabled", True)),
        ),
        orientation_safety=OrientationLimiter(float(config.get("max_angular_speed_deg_s", 60.0))),
        orientation_filter=SignalFilter(
            deadband=float(config.get("orientation_deadband_deg", 2.0)),
            alpha=float(config.get("orientation_filter_alpha", 0.25)),
            enabled=bool(config.get("orientation_filter_enabled", True)),
        ),
        stale_timeout_s=float(config.get("stale_timeout_s", 0.25)),
        orientation_enabled=bool(config.get("orientation_enabled", False)),
        orientation_scale=float(config.get("orientation_scale", 1.0)),
        max_orientation_delta_deg=np.asarray(config.get("max_orientation_delta_deg", [45.0, 45.0, 60.0]), dtype=float),
        hold_orientation=bool(config.get("hold_orientation", True)),
        default_rpy_deg=None if config.get("default_rpy_deg") is None else np.asarray(config.get("default_rpy_deg"), dtype=float),
    )

    kinematics = None
    ik_seed = None
    if bool(config.get("urdf_guard_enabled", False)):
        urdf_path = Path(config.get("urdf_path", "third_party/agx_arm_urdf/piper/urdf/piper_description.urdf"))
        kinematics = PiperKinematics(urdf_path)
        print(f"Optional URDF guard enabled: {urdf_path}")

        def urdf_guard(xyz_m: np.ndarray, rpy_deg: np.ndarray) -> tuple[bool, str]:
            nonlocal ik_seed
            ik_result = kinematics.solve(xyz_m, rpy_deg, ik_seed)
            if not ik_result.success:
                return (
                    False,
                    f"urdf_ik_unreachable:{ik_result.position_error_m:.3f}m:{ik_result.orientation_error_deg:.1f}deg",
                )
            ik_seed = ik_result.joints_rad
            return True, "ok"

        session.command_guard = urdf_guard

    next_verbose_s = 0.0
    try:
        while True:
            loop_start = time.monotonic()
            sample = quest.get_sample()
            result = session.step(sample, driver)

            if result.action == "sent" and config.get("gripper_enabled", False) and sample is not None:
                trigger_key = "rightTrig" if side == "right" else "leftTrig"
                driver.send_gripper(analog_value(sample.buttons, trigger_key) * 0.08)

            if args.verbose and time.monotonic() >= next_verbose_s:
                next_verbose_s = time.monotonic() + 0.2
                _print_verbose(result, driver, transport_name)

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, period_s - elapsed))
    except KeyboardInterrupt:
        print("\nCtrl+C received. Holding at the measured endpoint pose.")
        driver.hold()
        quest.stop()
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
