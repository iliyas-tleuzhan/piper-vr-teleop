"""Single Piper Quest controller endpoint teleoperation."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import yaml

from .buttons import analog_value, is_pressed
from .piper_driver import PiperDriver
from .piper_kinematics import PiperKinematics
from .quest_reader import QuestReader
from .safety import OrientationLimiter, SafetyLimiter, tracking_is_stale
from .vr_mapping import AxisMapping, orientation_target_from_home, target_from_home


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
    if args.no_urdf_guard:
        config["urdf_guard_enabled"] = False
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
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--quest-ip")
    parser.add_argument("--no-urdf-guard", action="store_true", help="disable constrained URDF IK feasibility checks")
    return parser


def _format_xyz(value: np.ndarray | None) -> str:
    if value is None:
        return "None"
    return np.asarray(value, dtype=float).round(4).tolist().__repr__()


def main() -> int:
    args = build_parser().parse_args()
    config = apply_overrides(load_config(args.config), args)
    hz = float(config.get("hz", 20.0))
    period_s = 1.0 / hz
    side = config.get("side", "right")
    deadman_button = config.get("deadman_button", "rightGrip")
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
    orientation_safety = OrientationLimiter(float(config.get("max_angular_speed_deg_s", 60.0)))
    mapping = AxisMapping.from_config(config.get("axis_mapping"))
    scale = float(config.get("scale", 1.0))
    orientation_enabled = bool(config.get("orientation_enabled", True))
    orientation_scale = float(config.get("orientation_scale", 1.0))
    max_orientation_delta_deg = np.asarray(config.get("max_orientation_delta_deg", [45.0, 45.0, 60.0]), dtype=float)
    urdf_guard_enabled = bool(config.get("urdf_guard_enabled", True))
    kinematics = None
    ik_seed = None
    if urdf_guard_enabled:
        urdf_path = Path(config.get("urdf_path", "third_party/agx_arm_urdf/piper/urdf/piper_description.urdf"))
        kinematics = PiperKinematics(urdf_path)
        print(f"URDF constrained IK guard: {urdf_path} ({', '.join(joint.name for joint in kinematics.joints)})")

    calibrated = False
    vr_home = None
    piper_home = None
    rpy_home = None
    was_calibrate_pressed = False
    was_deadman_pressed = False
    motion_active = False
    holding = False
    motion_blocked = False
    next_verbose_s = 0.0

    def freeze_at_current_pose(reason: str) -> EndPose:
        """Stop the arm at feedback position and reset the rate limiter there."""
        nonlocal holding
        driver.hold()
        pose = driver.read_end_pose() or driver.last_pose
        safety.reset(pose.xyz_m)
        orientation_safety.reset(pose.rpy_deg)
        holding = True
        if args.verbose:
            print(f"[teleop] holding at measured pose ({reason})")
        return pose

    def anchor_motion(controller_pose: np.ndarray, reason: str) -> None:
        """Create a zero-error clutch point from the current controller and arm poses."""
        nonlocal vr_home, piper_home, rpy_home, motion_active, holding, ik_seed
        pose = driver.read_end_pose() or driver.last_pose
        vr_home = controller_pose.copy()
        piper_home = pose.xyz_m.copy()
        rpy_home = pose.rpy_deg.copy() if config.get("hold_orientation", True) else np.asarray(
            config.get("default_rpy_deg") or pose.rpy_deg, dtype=float
        )
        safety.reset(piper_home)
        orientation_safety.reset(rpy_home)
        if kinematics is not None:
            result = kinematics.solve(piper_home, rpy_home, ik_seed)
            if result.success:
                ik_seed = result.joints_rad
            else:
                print("WARNING: URDF model cannot match the measured endpoint at this clutch point.")
        motion_active = True
        holding = False
        print(f"Teleop armed ({reason}): endpoint_m={piper_home.round(4).tolist()}")

    def maybe_print_verbose(
        *,
        buttons: dict,
        calibrated_state: bool,
        deadman_state: bool,
        calibrate_state: bool,
        controller_xyz: np.ndarray | None = None,
        raw_target_xyz: np.ndarray | None = None,
        safe_target_xyz: np.ndarray | None = None,
        command_sent: bool = False,
        skipped_reason: str | None = None,
    ) -> None:
        nonlocal next_verbose_s
        if not args.verbose:
            return
        now_s = time.monotonic()
        if now_s < next_verbose_s:
            return
        next_verbose_s = now_s + 0.2
        piper_pose = driver.read_end_pose()
        piper_xyz = None if piper_pose is None else piper_pose.xyz_m
        status = "calibrated" if calibrated_state else "waiting_for_calibration"
        action = "sent" if command_sent else f"skipped:{skipped_reason or 'none'}"
        print(
            "[verbose] "
            f"status={status} "
            f"deadman={deadman_state} "
            f"calibrate={calibrate_state} "
            f"controller_xyz={_format_xyz(controller_xyz)} "
            f"raw_target_xyz={_format_xyz(raw_target_xyz)} "
            f"safe_target_xyz={_format_xyz(safe_target_xyz)} "
            f"action={action} "
            f"piper_xyz={_format_xyz(piper_xyz)}"
        )

    try:
        while True:
            loop_start = time.monotonic()
            buttons = quest.get_buttons()
            deadman = is_pressed(buttons, deadman_button)
            calibrate_pressed = is_pressed(buttons, calibrate_button)
            current_vr = None
            controller_xyz = None

            if calibrate_pressed and not was_calibrate_pressed:
                current_vr = quest.get_controller_pose(side)
                controller_xyz = current_vr[:3, 3]
                # Calibration is deliberately disarmed.  The grip must be released
                # and pressed again, preventing an already-held grip from moving the
                # arm with a stale controller offset.
                current_pose = freeze_at_current_pose("calibration")
                vr_home = current_vr.copy()
                piper_home = current_pose.xyz_m.copy()
                rpy_home = current_pose.rpy_deg.copy() if config.get("hold_orientation", True) else np.asarray(
                    config.get("default_rpy_deg") or current_pose.rpy_deg, dtype=float
                )
                calibrated = True
                motion_active = False
                motion_blocked = False
                was_deadman_pressed = deadman
                print(
                    f"Calibrated: piper_home_m={piper_home.round(4).tolist()} rpy_deg={rpy_home.round(2).tolist()}. "
                    "Release then press the deadman to arm motion."
                )
            was_calibrate_pressed = calibrate_pressed

            # Do not process the calibrate press as a motion command in this cycle.
            if calibrate_pressed:
                time.sleep(period_s)
                continue

            if not calibrated:
                maybe_print_verbose(
                    buttons=buttons,
                    calibrated_state=calibrated,
                    deadman_state=deadman,
                    calibrate_state=calibrate_pressed,
                    skipped_reason="not_calibrated",
                )
                print("Waiting for calibration. Press the calibrate button while the robot is in a safe pose.", end="\r")
                time.sleep(period_s)
                continue

            if tracking_is_stale(quest.last_update_s, safety.stale_timeout_s):
                if motion_active or not holding:
                    freeze_at_current_pose("tracking stale")
                motion_active = False
                # A controller held through a tracking outage must be released and
                # pressed again before it can command motion.
                was_deadman_pressed = deadman
                maybe_print_verbose(
                    buttons=buttons,
                    calibrated_state=calibrated,
                    deadman_state=deadman,
                    calibrate_state=calibrate_pressed,
                    skipped_reason="tracking_stale",
                )
                print("Quest tracking is stale or missing; holding position.", end="\r")
                time.sleep(period_s)
                continue

            if not deadman:
                if motion_active or not holding:
                    freeze_at_current_pose("deadman released")
                motion_active = False
                motion_blocked = False
                was_deadman_pressed = False
                current_vr = quest.get_controller_pose(side)
                controller_xyz = current_vr[:3, 3]
                maybe_print_verbose(
                    buttons=buttons,
                    calibrated_state=calibrated,
                    deadman_state=deadman,
                    calibrate_state=calibrate_pressed,
                    controller_xyz=controller_xyz,
                    skipped_reason="deadman_released",
                )
                time.sleep(period_s)
                continue

            current_vr = quest.get_controller_pose(side)
            controller_xyz = current_vr[:3, 3]
            if motion_blocked:
                maybe_print_verbose(
                    buttons=buttons,
                    calibrated_state=calibrated,
                    deadman_state=deadman,
                    calibrate_state=calibrate_pressed,
                    controller_xyz=controller_xyz,
                    skipped_reason="urdf_ik_unreachable_release_deadman",
                )
                time.sleep(period_s)
                continue
            if not was_deadman_pressed:
                # Clutch semantics: every new grip press starts from the arm's
                # actual current pose, so previous controller motion is never replayed.
                anchor_motion(current_vr, "deadman pressed")
                was_deadman_pressed = True
                maybe_print_verbose(
                    buttons=buttons,
                    calibrated_state=calibrated,
                    deadman_state=deadman,
                    calibrate_state=calibrate_pressed,
                    controller_xyz=controller_xyz,
                    skipped_reason="armed_this_cycle",
                )
                time.sleep(period_s)
                continue

            was_deadman_pressed = True
            target = target_from_home(vr_home, current_vr, piper_home, mapping, scale)
            safe_target = safety.limit_step(target)
            target_rpy = (
                orientation_target_from_home(
                    vr_home, current_vr, rpy_home, mapping, orientation_scale, max_orientation_delta_deg
                )
                if orientation_enabled
                else rpy_home
            )
            safe_rpy = orientation_safety.limit_step(target_rpy)
            if kinematics is not None:
                result = kinematics.solve(safe_target, safe_rpy, ik_seed)
                if not result.success:
                    freeze_at_current_pose(
                        f"URDF IK unreachable (position error {result.position_error_m:.3f} m, "
                        f"orientation error {result.orientation_error_deg:.1f} deg)"
                    )
                    motion_active = False
                    motion_blocked = True
                    continue
                ik_seed = result.joints_rad
            driver.send_end_pose(safe_target, safe_rpy)
            maybe_print_verbose(
                buttons=buttons,
                calibrated_state=calibrated,
                deadman_state=deadman,
                calibrate_state=calibrate_pressed,
                controller_xyz=controller_xyz,
                raw_target_xyz=target,
                safe_target_xyz=safe_target,
                command_sent=True,
            )

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
