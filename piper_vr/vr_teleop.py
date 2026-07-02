"""Mode-aware Quest 3 to AgileX Piper teleoperation entrypoint."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from .config import apply_profile, deep_merge, load_config
from .human_arm_model import HumanArmConfig
from .joint_mimic import JointMimicConfig
from .movep_teleop import main as endpoint_main
from .piper_driver import PiperDriver
from .quest_endpoint_ik import QuestEndpointIKConfig, QuestEndpointIKSession
from .quest_reader import QuestReader
from .session import JointMimicSession
from .relative_calibration import dominant_channel
from .units import degrees_to_piper_rpy, meters_to_piper_xyz
from .viz_broadcaster import QuestVizBroadcaster


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Quest 3 to AgileX Piper teleoperation")
    parser.add_argument("--config", default="configs/single_piper.yaml")
    parser.add_argument("--mapping-config", help="YAML patch merged into --config, normally configs/generated_endpoint_ik_mapping.yaml")
    parser.add_argument("--profile", choices=("safe", "normal", "fast"), help="Apply a speed profile from the config")
    parser.add_argument("--control-mode", choices=("joint_mimic", "endpoint_firmware", "external_ik", "quest_endpoint_ik"))
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
    parser.add_argument("--no-log", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--log-dir")
    parser.add_argument("--viz", action="store_true", help="broadcast passive UDP state for the Quest visualization app")
    parser.add_argument("--viz-host", help="Quest visualization UDP host/IP")
    parser.add_argument("--viz-port", type=int, help="Quest visualization UDP port")
    parser.add_argument("--debug-motion", action="store_true", help="print relative motion internals at 10 Hz")
    parser.add_argument("--disable-wrist", action="store_true", help="force wrist rotation channels and joints 4-6 to zero")
    parser.add_argument("--wrist-gain", type=float, help="scale wrist rotation gain columns live")
    parser.add_argument("--translation-only", action="store_true", help="zero rotation columns so only joints 1-3 move")
    parser.add_argument("--rotation-only", "--wrist-only", dest="rotation_only", action="store_true", help="zero translation columns so only joints 4-6 move")
    parser.add_argument("--endpoint-ik", action="store_true", help="shortcut for --control-mode quest_endpoint_ik")
    parser.add_argument("--ik-backend", choices=("firmware_endpoint", "host_ik_sdk_fk", "host_ik_urdf"), help="Quest endpoint IK backend")
    parser.add_argument("--ik-scale", type=float, help="Quest endpoint IK translation scale")
    parser.add_argument("--endpoint-speed-percent", type=int, help="override Piper firmware endpoint MOVE P speed")
    parser.add_argument("--ik-orientation-scale", type=float, help="Quest endpoint IK orientation scale")
    parser.add_argument("--position-only", action="store_true", help="disable endpoint IK orientation control")
    parser.add_argument("--disable-orientation", action="store_true", help="disable endpoint IK orientation control")
    parser.add_argument("--full-workspace", action="store_true", help="disable endpoint IK home-relative and absolute workspace target clamps")
    parser.add_argument("--no-home-delta-clamp", action="store_true", help="disable endpoint IK target clamp around calibration home")
    parser.add_argument("--no-workspace-clamp", action="store_true", help="disable endpoint IK absolute workspace target clamp")
    parser.add_argument("--home-delta-clamp", action="store_true", help="enable endpoint IK target clamp around calibration home")
    parser.add_argument("--workspace-clamp", action="store_true", help="enable endpoint IK absolute workspace target clamp")
    parser.add_argument("--max-delta-from-home", nargs=3, type=float, metavar=("X", "Y", "Z"), help="endpoint IK home-relative clamp half extents in meters")
    parser.add_argument("--workspace-min", nargs=3, type=float, metavar=("X", "Y", "Z"), help="endpoint IK absolute workspace minimum in meters")
    parser.add_argument("--workspace-max", nargs=3, type=float, metavar=("X", "Y", "Z"), help="endpoint IK absolute workspace maximum in meters")
    parser.add_argument("--max-position-step-xyz", nargs=3, type=float, metavar=("X", "Y", "Z"), help="endpoint IK per-cycle XYZ step limit in meters")
    parser.add_argument("--debug-ik", action="store_true", help="print endpoint IK internals at 10 Hz")
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
    if getattr(args, "endpoint_ik", False):
        config["control_mode"] = "quest_endpoint_ik"
    endpoint_ik = config.setdefault("quest_endpoint_ik", {})
    if getattr(args, "ik_backend", None) is not None:
        endpoint_ik["backend"] = args.ik_backend
    if getattr(args, "ik_scale", None) is not None:
        endpoint_ik["scale"] = float(args.ik_scale)
    if getattr(args, "ik_orientation_scale", None) is not None:
        endpoint_ik["orientation_scale"] = float(args.ik_orientation_scale)
    if getattr(args, "disable_orientation", False) or getattr(args, "position_only", False) or getattr(args, "translation_only", False):
        endpoint_ik["orientation_enabled"] = False
        endpoint_ik["position_only_default"] = True
    if getattr(args, "orientation_enabled", False):
        endpoint_ik["orientation_enabled"] = True
        endpoint_ik["position_only_default"] = False
    if getattr(args, "rotation_only", False):
        endpoint_ik["translation_enabled"] = False
        endpoint_ik["orientation_enabled"] = True
    if getattr(args, "full_workspace", False):
        endpoint_ik["home_delta_clamp_enabled"] = False
        endpoint_ik["workspace_clamp_enabled"] = False
    if getattr(args, "no_home_delta_clamp", False):
        endpoint_ik["home_delta_clamp_enabled"] = False
    if getattr(args, "no_workspace_clamp", False):
        endpoint_ik["workspace_clamp_enabled"] = False
    if getattr(args, "home_delta_clamp", False):
        endpoint_ik["home_delta_clamp_enabled"] = True
    if getattr(args, "workspace_clamp", False):
        endpoint_ik["workspace_clamp_enabled"] = True
    if getattr(args, "max_delta_from_home", None) is not None:
        endpoint_ik["max_delta_from_home_m"] = [float(value) for value in args.max_delta_from_home]
    if getattr(args, "workspace_min", None) is not None:
        endpoint_ik["workspace_min_m"] = [float(value) for value in args.workspace_min]
    if getattr(args, "workspace_max", None) is not None:
        endpoint_ik["workspace_max_m"] = [float(value) for value in args.workspace_max]
    if getattr(args, "max_position_step_xyz", None) is not None:
        endpoint_ik["max_position_step_m_xyz"] = [float(value) for value in args.max_position_step_xyz]
    if args.max_joint_speed is not None:
        mimic = config.setdefault("joint_mimic", {})
        mimic["max_joint_speed_deg_s"] = [float(args.max_joint_speed)] * 6
    mimic = config.setdefault("joint_mimic", {})
    if args.disable_wrist:
        mimic["wrist_rotation_enabled"] = False
    if args.wrist_gain is not None:
        matrix = np.asarray(mimic.get("relative_gain_matrix", JointMimicConfig.from_config(mimic).relative_gain_matrix), dtype=float).copy()
        mask = np.abs(matrix[:, 3:6]) > 1e-12
        matrix[:, 3:6] = np.where(mask, np.sign(matrix[:, 3:6]) * float(args.wrist_gain), matrix[:, 3:6])
        mimic["wrist_rotation_gain"] = float(args.wrist_gain)
        mimic["relative_gain_matrix"] = matrix.tolist()
    if args.translation_only or args.disable_wrist:
        matrix = np.asarray(mimic.get("relative_gain_matrix", JointMimicConfig.from_config(mimic).relative_gain_matrix), dtype=float).copy()
        matrix[:, 3:6] = 0.0
        matrix[3:6, :] = 0.0
        mimic["relative_gain_matrix"] = matrix.tolist()
    if args.rotation_only:
        matrix = np.asarray(mimic.get("relative_gain_matrix", JointMimicConfig.from_config(mimic).relative_gain_matrix), dtype=float).copy()
        matrix[:, 0:3] = 0.0
        matrix[0:3, :] = 0.0
        mimic["relative_gain_matrix"] = matrix.tolist()
        mimic["wrist_rotation_enabled"] = True
    viz = config.setdefault("viz", {})
    if args.viz:
        viz["enabled"] = True
    if args.viz_host is not None:
        viz["host"] = args.viz_host
    if args.viz_port is not None:
        viz["port"] = args.viz_port
    return config


def _load_runtime_config(args: argparse.Namespace) -> dict:
    config = load_config(args.config)
    if args.mapping_config:
        return deep_merge(config, load_config(args.mapping_config))

    runtime = config.get("runtime", {})
    mapping_config = runtime.get("auto_mapping_config", "configs/generated_endpoint_ik_mapping.yaml")
    mapping_path = Path(mapping_config)
    if mapping_path.exists():
        print(f"Auto-loaded mapping config: {mapping_config}")
        return deep_merge(config, load_config(mapping_config))

    print(f"No generated endpoint mapping found at {mapping_config}; using base config mapping.")
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


def _print_motion_debug(result, driver: PiperDriver) -> None:
    measured = result.measured_joints or driver.read_joint_pose()
    if result.delta_xyz is None:
        dominant = "None"
    else:
        _, channel, sign, value = dominant_channel(result.delta_xyz)
        dominant = f"{channel} {sign} {value:.4f}"
    tracking_error = None
    if measured is not None and result.safe_joint_target_deg is not None:
        tracking_error = float(np.linalg.norm(measured.joints_deg - result.safe_joint_target_deg))
    print(
        "[debug-motion] "
        f"raw_controller_xyz={_format_array(result.controller_xyz, 4)} "
        f"delta_xyz={_format_array(result.delta_xyz, 4)} "
        f"delta_rot_raw={_format_array(result.delta_rot_raw_deg, 3)} "
        f"delta_rot_used={_format_array(result.delta_rot_deg, 3)} "
        f"dominant={dominant} "
        f"translation_dq={_format_array(result.translation_dq_deg, 2)} "
        f"wrist_dq={_format_array(result.wrist_dq_deg, 2)} "
        f"full_dq={_format_array(result.relative_dq_deg, 2)} "
        f"raw_target={_format_array(result.raw_joint_target_deg, 2)} "
        f"safe_target={_format_array(result.safe_joint_target_deg, 2)} "
        f"measured_joints={_format_array(None if measured is None else measured.joints_deg, 2)} "
        f"tracking_error={None if tracking_error is None else round(tracking_error, 2)}"
    )


def _print_ik_debug(result, driver: PiperDriver) -> None:
    measured = result.measured_joints or driver.read_joint_pose()
    print(
        "[debug-ik] "
        f"control_frame={getattr(result, 'control_frame', None)} "
        f"fk_backend={getattr(result, 'fk_backend', None)} "
        f"state={result.state.value} "
        f"calibrated={result.calibrated} "
        f"controller_xyz={_format_array(result.controller_xyz, 4)} "
        f"controller_delta_xyz={_format_array(result.controller_delta_xyz, 4)} "
        f"axis_mapping={result.axis_mapping} "
        f"mapped_robot_delta_xyz={_format_array(result.mapped_robot_delta_xyz, 4)} "
        f"scale={getattr(result, 'scale', None)} "
        f"scale_xyz={_format_array(getattr(result, 'scale_xyz', None), 3)} "
        f"scaled_robot_delta_xyz={_format_array(result.scaled_robot_delta_xyz, 4)} "
        f"home_delta_clamp_enabled={result.home_delta_clamp_enabled} "
        f"workspace_clamp_enabled={result.workspace_clamp_enabled} "
        f"target_before_home_clamp={_format_array(result.target_before_home_clamp, 4)} "
        f"target_after_home_clamp={_format_array(result.target_after_home_clamp, 4)} "
        f"target_after_workspace_clamp={_format_array(result.target_after_workspace_clamp, 4)} "
        f"clamped_axes={result.clamped_axes} "
        f"target_xyz={_format_array(result.target_xyz, 4)} "
        f"controller_delta_rpy={_format_array(result.controller_delta_rpy_deg, 2)} "
        f"target_rpy={_format_array(result.target_rpy_deg, 2)} "
        f"raw_joints={_format_array(result.raw_joint_target_deg, 2)} "
        f"safe_joints={_format_array(result.safe_joint_target_deg, 2)} "
        f"measured_joints={_format_array(None if measured is None else measured.joints_deg, 2)} "
        f"ik_error=({None if result.position_error_m is None else round(result.position_error_m, 4)},"
        f"{None if result.orientation_error_deg is None else round(result.orientation_error_deg, 2)}) "
        f"firmware_xyz_mm={None if result.target_xyz is None else (np.asarray(result.target_xyz) * 1000).round(2).tolist()} "
        f"firmware_xyz_raw={None if result.target_xyz is None else [meters_to_piper_xyz(float(v)) for v in result.target_xyz]} "
        f"firmware_rpy_raw={None if result.target_rpy_deg is None else [degrees_to_piper_rpy(float(v)) for v in result.target_rpy_deg]} "
        f"action={result.action}:{result.reason}"
    )
    if result.home_delta_clamped:
        print(f"LIMIT ACTIVE: target_clamped_home_delta axes={result.clamped_axes}")
    if result.workspace_clamped:
        print(f"LIMIT ACTIVE: target_clamped_workspace axes={result.clamped_axes}")


def _debug_ik_enabled(args: argparse.Namespace, config: dict) -> bool:
    return bool(args.debug_ik or config.get("runtime", {}).get("debug_ik", False))


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
            "delta_xyz": arr(result.delta_xyz),
            "delta_rot_raw_deg": arr(result.delta_rot_raw_deg),
            "delta_rot_used_deg": arr(result.delta_rot_deg),
            "dominant_movement_channel": None if result.delta_xyz is None else dominant_channel(result.delta_xyz)[1],
            "relative_u": arr(result.relative_u),
            "relative_dq_deg": arr(result.relative_dq_deg),
            "translation_dq_deg": arr(result.translation_dq_deg),
            "wrist_dq_deg": arr(result.wrist_dq_deg),
            "action": result.action,
            "reason": result.reason,
        }
        self._handle.write(json.dumps(row, separators=(",", ":")) + "\n")
        self._handle.flush()

    def close(self) -> None:
        self._handle.close()


def _run_joint_mimic(args: argparse.Namespace, config: dict) -> int:
    runtime = config.get("runtime", {})
    verbose = bool(args.verbose or runtime.get("verbose", False))
    if args.quiet:
        verbose = False
    log_enabled = bool(args.log or runtime.get("log", False))
    if args.no_log:
        log_enabled = False
    log_dir = args.log_dir or runtime.get("log_dir", "logs/joint_mimic")
    viz_config = config.get("viz", {})
    viz = QuestVizBroadcaster(
        host=viz_config.get("host", "127.0.0.1"),
        port=int(viz_config.get("port", 5055)),
        enabled=bool(viz_config.get("enabled", False)),
    )

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
    speed_percent = int(args.endpoint_speed_percent if getattr(args, "endpoint_speed_percent", None) is not None and ik_config.backend == "firmware_endpoint" else config.get("speed_percent", 5))
    driver = PiperDriver(can=config.get("can", "can0"), speed_percent=speed_percent, dry_run=args.dry_run)
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
    next_debug_s = 0.0
    logger = JointMimicJsonlLogger(log_dir) if log_enabled else None
    if logger is not None:
        print(f"Logging joint mimic JSONL to {logger.path}")
    if viz.enabled:
        print(f"Quest visualization UDP enabled: {viz.host}:{viz.port}")
    try:
        while True:
            loop_start = time.monotonic()
            result = session.step(quest.get_sample(), driver)
            viz.send(
                result,
                driver,
                mode="joint_mimic",
                mapping_mode=session.mimic_config.mapping_mode,
            )
            if logger is not None:
                logger.write(result)
            if verbose and time.monotonic() >= next_verbose_s:
                next_verbose_s = time.monotonic() + 0.2
                _print_joint_verbose(result, driver, transport_name)
            if args.debug_motion and time.monotonic() >= next_debug_s:
                next_debug_s = time.monotonic() + 0.1
                _print_motion_debug(result, driver)
            time.sleep(max(0.0, period_s - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        print("\nCtrl+C received. Holding at the measured joint pose.")
        command_joint_hold_on_exit(driver)
        return 0
    finally:
        quest.stop()
        viz.close()
        if logger is not None:
            logger.close()


def _run_quest_endpoint_ik(args: argparse.Namespace, config: dict) -> int:
    debug_ik = _debug_ik_enabled(args, config)
    hz = float(config.get("hz", 30.0))
    period_s = 1.0 / hz
    side = config.get("side", "right")
    deadman_button = config.get("deadman_button", "rightGrip")
    calibrate_button = config.get("calibrate_button", "A")
    quest_config = config.get("quest", {})
    transport_name = quest_config.get("transport", "adb_logcat")

    print("Piper Quest endpoint IK teleoperation")
    print(f"Control mode: quest_endpoint_ik, robot: {'dry-run' if args.dry_run else 'REAL ROBOT'}")
    print(f"Transport: {transport_name}, controller: {side}, deadman: {deadman_button}, calibrate: {calibrate_button}")
    if not args.dry_run:
        print("WARNING: endpoint IK sends six-joint JointCtrl commands. Keep clear and start with --profile safe.")
        time.sleep(2.0)

    quest = QuestReader(
        transport=transport_name,
        connection=quest_config.get("connection", "usb"),
        ip_address=quest_config.get("ip_address"),
        simulate_on_missing=args.dry_run,
    )
    ik_config = QuestEndpointIKConfig.from_config(config.get("quest_endpoint_ik"))
    driver = PiperDriver(can=config.get("can", "can0"), speed_percent=int(config.get("speed_percent", 5)), dry_run=args.dry_run)
    driver.connect(initial_mode="endpoint" if ik_config.backend == "firmware_endpoint" else "joint")
    session = QuestEndpointIKSession(
        side=side,
        deadman_button=deadman_button,
        calibrate_button=calibrate_button,
        config=ik_config,
        stale_timeout_s=float(config.get("stale_timeout_s", 0.25)),
    )

    next_debug_s = 0.0
    try:
        while True:
            loop_start = time.monotonic()
            result = session.step(quest.get_sample(), driver)
            if debug_ik and time.monotonic() >= next_debug_s:
                next_debug_s = time.monotonic() + 0.1
                _print_ik_debug(result, driver)
            time.sleep(max(0.0, period_s - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        print("\nCtrl+C received. Holding at the measured joint pose.")
        command_joint_hold_on_exit(driver)
        return 0
    finally:
        quest.stop()


def main() -> int:
    args = build_parser().parse_args()
    config = _load_runtime_config(args)
    config = apply_profile(config, args.profile)
    config = _apply_common_overrides(config, args)
    mode = config.get("control_mode", "joint_mimic")
    if mode == "joint_mimic":
        return _run_joint_mimic(args, config)
    if mode == "quest_endpoint_ik":
        return _run_quest_endpoint_ik(args, config)
    if mode in ("endpoint_firmware", "external_ik"):
        if mode == "external_ik":
            print("external_ik is available as a library path; endpoint_firmware is used as the runtime fallback until tuned.")
        return endpoint_main()
    raise ValueError(f"Unsupported control_mode: {mode!r}")


if __name__ == "__main__":
    raise SystemExit(main())
