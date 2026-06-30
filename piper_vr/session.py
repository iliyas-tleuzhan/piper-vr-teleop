"""Teleoperation session state machine."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any
from collections.abc import Callable

import numpy as np

from .buttons import analog_value, is_pressed
from .human_arm_model import HumanArmConfig, HumanArmState, build_human_arm_state, estimate_shoulder_from_hmd
from .joint_limits import clamp_joints_deg, rate_limit_joints_deg
from .joint_mimic import JointMimicConfig, human_arm_to_mimic_vector_deg, mimic_vector_to_piper_joints
from .piper_driver import EndPose, JointPose
from .safety import OrientationLimiter, SafetyLimiter, SignalFilter
from .types import QuestSample, TeleopState
from .vr_mapping import AxisMapping, orientation_target_from_home, target_from_home


@dataclass
class SessionResult:
    state: TeleopState
    calibrated: bool
    deadman: bool = False
    calibrate: bool = False
    controller_xyz: np.ndarray | None = None
    raw_target_xyz: np.ndarray | None = None
    safe_target_xyz: np.ndarray | None = None
    target_rpy_deg: np.ndarray | None = None
    action: str = "skipped"
    reason: str = "none"
    piper_pose: EndPose | None = None
    human_arm: HumanArmState | None = None
    human_vector_deg: np.ndarray | None = None
    human_home_vector_deg: np.ndarray | None = None
    human_delta_deg: np.ndarray | None = None
    robot_home_joints_deg: np.ndarray | None = None
    raw_joint_target_deg: np.ndarray | None = None
    safe_joint_target_deg: np.ndarray | None = None
    measured_joints: JointPose | None = None
    sample_age_s: float | None = None


class TeleopSession:
    """Stateful Quest-to-Piper endpoint teleop policy."""

    def __init__(
        self,
        *,
        side: str,
        deadman_button: str,
        calibrate_button: str,
        scale: float,
        mapping: AxisMapping,
        safety: SafetyLimiter,
        position_filter: SignalFilter,
        orientation_safety: OrientationLimiter,
        orientation_filter: SignalFilter,
        stale_timeout_s: float,
        orientation_enabled: bool = False,
        orientation_scale: float = 1.0,
        max_orientation_delta_deg: np.ndarray | None = None,
        hold_orientation: bool = True,
        default_rpy_deg: np.ndarray | None = None,
    ) -> None:
        self.side = side
        self.deadman_button = deadman_button
        self.calibrate_button = calibrate_button
        self.scale = float(scale)
        self.mapping = mapping
        self.safety = safety
        self.position_filter = position_filter
        self.orientation_safety = orientation_safety
        self.orientation_filter = orientation_filter
        self.stale_timeout_s = float(stale_timeout_s)
        self.orientation_enabled = orientation_enabled
        self.orientation_scale = float(orientation_scale)
        self.max_orientation_delta_deg = np.asarray(
            [45.0, 45.0, 60.0] if max_orientation_delta_deg is None else max_orientation_delta_deg,
            dtype=float,
        )
        self.hold_orientation = hold_orientation
        self.default_rpy_deg = None if default_rpy_deg is None else np.asarray(default_rpy_deg, dtype=float)
        self.state = TeleopState.WAITING_FOR_DEVICE
        self.command_guard: Callable[[np.ndarray, np.ndarray], tuple[bool, str]] | None = None
        self.vr_home: np.ndarray | None = None
        self.piper_home: np.ndarray | None = None
        self.rpy_home: np.ndarray | None = None
        self.was_deadman_pressed = False
        self.was_calibrate_pressed = False
        self.require_deadman_repress = False

    @property
    def calibrated(self) -> bool:
        return self.vr_home is not None and self.piper_home is not None and self.rpy_home is not None

    def step(self, sample: QuestSample | None, driver: Any) -> SessionResult:
        if sample is None:
            self.state = TeleopState.WAITING_FOR_DEVICE
            return SessionResult(self.state, self.calibrated, reason="no_quest_sample")

        buttons = sample.buttons
        deadman = is_pressed(buttons, self.deadman_button)
        calibrate = is_pressed(buttons, self.calibrate_button)
        current_vr = sample.transforms_openxr.get(self.side)
        controller_xyz = None if current_vr is None else np.asarray(current_vr, dtype=float)[:3, 3].copy()
        result = SessionResult(
            state=self.state,
            calibrated=self.calibrated,
            deadman=deadman,
            calibrate=calibrate,
            controller_xyz=controller_xyz,
            sample_age_s=sample.age_s,
        )

        if current_vr is None:
            self.state = TeleopState.WAITING_FOR_DEVICE
            result.state = self.state
            result.reason = f"missing_{self.side}_controller"
            return result

        if sample.age_s > self.stale_timeout_s:
            self._hold(driver)
            self.state = TeleopState.HOLDING
            self.require_deadman_repress = True
            self.was_deadman_pressed = deadman
            result.state = self.state
            result.reason = "tracking_stale"
            result.piper_pose = driver.read_end_pose()
            return result

        if calibrate and not self.was_calibrate_pressed:
            pose = self._hold(driver)
            self.vr_home = np.asarray(current_vr, dtype=float).copy()
            self.piper_home = pose.xyz_m.copy()
            self.rpy_home = pose.rpy_deg.copy() if self.hold_orientation else (
                self.default_rpy_deg.copy() if self.default_rpy_deg is not None else pose.rpy_deg.copy()
            )
            self._reset_filters(self.piper_home, self.rpy_home)
            self.require_deadman_repress = True
            self.was_deadman_pressed = deadman
            self.state = TeleopState.READY_IDLE
            result.state = self.state
            result.calibrated = True
            result.action = "skipped"
            result.reason = "calibrated_release_deadman"
            result.piper_pose = pose
            self.was_calibrate_pressed = calibrate
            return result
        self.was_calibrate_pressed = calibrate

        if not self.calibrated:
            self.state = TeleopState.WAITING_FOR_CALIBRATION
            result.state = self.state
            result.reason = "not_calibrated"
            return result

        if not deadman:
            pose = self._hold(driver)
            self.state = TeleopState.READY_IDLE
            self.was_deadman_pressed = False
            self.require_deadman_repress = False
            result.state = self.state
            result.reason = "deadman_released"
            result.piper_pose = pose
            return result

        if self.require_deadman_repress:
            self.state = TeleopState.HOLDING
            result.state = self.state
            result.reason = "release_deadman_required"
            return result

        if not self.was_deadman_pressed:
            pose = driver.read_end_pose() or driver.last_pose
            self.vr_home = np.asarray(current_vr, dtype=float).copy()
            self.piper_home = pose.xyz_m.copy()
            self.rpy_home = pose.rpy_deg.copy() if self.hold_orientation else (
                self.default_rpy_deg.copy() if self.default_rpy_deg is not None else pose.rpy_deg.copy()
            )
            self._reset_filters(self.piper_home, self.rpy_home)
            self.was_deadman_pressed = True
            self.state = TeleopState.ACTIVE
            result.state = self.state
            result.reason = "armed_this_cycle"
            result.piper_pose = pose
            return result

        raw_target = target_from_home(self.vr_home, current_vr, self.piper_home, self.mapping, self.scale)
        filtered_target = self.position_filter.apply(raw_target)
        safe_target, safety_reason = self.safety.limit_step_with_reason(filtered_target)
        target_rpy = (
            orientation_target_from_home(
                self.vr_home,
                current_vr,
                self.rpy_home,
                self.mapping,
                self.orientation_scale,
                self.max_orientation_delta_deg,
            )
            if self.orientation_enabled
            else self.rpy_home
        )
        safe_rpy = self.orientation_safety.limit_step(self.orientation_filter.apply(target_rpy))
        if self.command_guard is not None:
            allowed, guard_reason = self.command_guard(safe_target, safe_rpy)
            if not allowed:
                pose = self._hold(driver)
                self.state = TeleopState.HOLDING
                self.require_deadman_repress = True
                result.state = self.state
                result.raw_target_xyz = raw_target
                result.safe_target_xyz = safe_target
                result.target_rpy_deg = safe_rpy
                result.action = "skipped"
                result.reason = guard_reason
                result.piper_pose = pose
                return result
        driver.send_end_pose(safe_target, safe_rpy)
        self.state = TeleopState.ACTIVE
        result.state = self.state
        result.raw_target_xyz = raw_target
        result.safe_target_xyz = safe_target
        result.target_rpy_deg = safe_rpy
        result.action = "sent"
        result.reason = safety_reason
        return result

    def safety_reject(self, driver: Any, reason: str) -> SessionResult:
        pose = self._hold(driver)
        self.state = TeleopState.HOLDING
        self.require_deadman_repress = True
        return SessionResult(self.state, self.calibrated, action="skipped", reason=reason, piper_pose=pose)

    def _hold(self, driver: Any) -> EndPose:
        driver.hold()
        pose = driver.read_end_pose() or driver.last_pose
        self._reset_filters(pose.xyz_m, pose.rpy_deg)
        return pose

    def _reset_filters(self, xyz_m: np.ndarray, rpy_deg: np.ndarray) -> None:
        now_s = time.monotonic()
        self.safety.reset(xyz_m, now_s)
        self.position_filter.reset(xyz_m)
        self.orientation_safety.reset(rpy_deg, now_s)
        self.orientation_filter.reset(rpy_deg)


class JointMimicSession:
    """Stateful Quest-to-Piper joint mimic policy."""

    def __init__(
        self,
        *,
        side: str,
        deadman_button: str,
        calibrate_button: str,
        human_config: HumanArmConfig,
        mimic_config: JointMimicConfig,
        stale_timeout_s: float,
        elbow_swivel_joystick: str | None = "rightJS_x",
        shoulder_lift_joystick: str | None = None,
    ) -> None:
        self.side = side
        self.deadman_button = deadman_button
        self.calibrate_button = calibrate_button
        self.human_config = human_config
        self.mimic_config = mimic_config
        self.stale_timeout_s = float(stale_timeout_s)
        self.elbow_swivel_joystick = elbow_swivel_joystick
        self.shoulder_lift_joystick = shoulder_lift_joystick
        self.state = TeleopState.WAITING_FOR_DEVICE
        self.controller_home: np.ndarray | None = None
        self.fixed_shoulder_xyz_m: np.ndarray | None = None
        self.robot_home_joints_deg: np.ndarray | None = None
        self.human_home_vector_deg: np.ndarray | None = None
        self.last_command_deg: np.ndarray | None = None
        self.previous_elbow: np.ndarray | None = None
        self.elbow_swivel_rad = float(human_config.elbow_swivel_default_rad)
        self.last_step_s: float | None = None
        self.was_deadman_pressed = False
        self.was_calibrate_pressed = False
        self.require_deadman_repress = False

    @property
    def calibrated(self) -> bool:
        return (
            self.controller_home is not None
            and self.fixed_shoulder_xyz_m is not None
            and self.robot_home_joints_deg is not None
            and self.human_home_vector_deg is not None
        )

    def step(self, sample: QuestSample | None, driver: Any) -> SessionResult:
        now_s = time.monotonic()
        dt = 1e-3 if self.last_step_s is None else max(now_s - self.last_step_s, 1e-3)
        self.last_step_s = now_s
        if sample is None:
            self.state = TeleopState.WAITING_FOR_DEVICE
            return SessionResult(self.state, self.calibrated, reason="no_quest_sample")

        buttons = sample.buttons
        deadman = is_pressed(buttons, self.deadman_button)
        calibrate = is_pressed(buttons, self.calibrate_button)
        current_vr = sample.transforms_openxr.get(self.side)
        controller_xyz = None if current_vr is None else np.asarray(current_vr, dtype=float)[:3, 3].copy()
        result = SessionResult(
            state=self.state,
            calibrated=self.calibrated,
            deadman=deadman,
            calibrate=calibrate,
            controller_xyz=controller_xyz,
            sample_age_s=sample.age_s,
        )

        if current_vr is None:
            self.state = TeleopState.WAITING_FOR_DEVICE
            result.state = self.state
            result.reason = f"missing_{self.side}_controller"
            return result

        if sample.age_s > self.stale_timeout_s:
            measured, action, reason = self._hold(driver)
            self.state = TeleopState.HOLDING
            self.require_deadman_repress = True
            self.was_deadman_pressed = deadman
            result.state = self.state
            result.reason = "tracking_stale"
            result.action = action
            if reason != "ok":
                result.reason = reason
                self.state = TeleopState.FAULT
                result.state = self.state
            result.measured_joints = measured
            return result

        if calibrate and not self.was_calibrate_pressed:
            measured = driver.read_joint_pose()
            if measured is None:
                if not getattr(driver, "dry_run", False):
                    self.state = TeleopState.WAITING_FOR_CALIBRATION
                    result.state = self.state
                    result.reason = "joint_feedback_required_for_calibration"
                    self.was_calibrate_pressed = calibrate
                    return result
                measured = JointPose(self.mimic_config.neutral_deg.copy())
            self.controller_home = np.asarray(current_vr, dtype=float).copy()
            self.fixed_shoulder_xyz_m = self._estimate_calibrated_shoulder(current_vr, sample)
            human_home = build_human_arm_state(
                self.fixed_shoulder_xyz_m,
                current_vr,
                self.elbow_swivel_rad,
                self.human_config,
                None,
            )
            self.robot_home_joints_deg = measured.joints_deg.copy()
            self.human_home_vector_deg = human_arm_to_mimic_vector_deg(human_home)
            self.last_command_deg = self.robot_home_joints_deg.copy()
            self.previous_elbow = None
            self.elbow_swivel_rad = float(self.human_config.elbow_swivel_default_rad)
            self.require_deadman_repress = True
            self.was_deadman_pressed = deadman
            self.state = TeleopState.READY_IDLE
            result.state = self.state
            result.calibrated = True
            result.reason = "calibrated_release_deadman"
            result.measured_joints = measured
            result.human_arm = human_home
            result.human_vector_deg = self.human_home_vector_deg.copy()
            result.human_home_vector_deg = self.human_home_vector_deg.copy()
            result.human_delta_deg = np.zeros(6)
            result.robot_home_joints_deg = self.robot_home_joints_deg.copy()
            self.was_calibrate_pressed = calibrate
            return result
        self.was_calibrate_pressed = calibrate

        if not self.calibrated:
            self.state = TeleopState.WAITING_FOR_CALIBRATION
            result.state = self.state
            result.reason = "not_calibrated"
            return result

        if not deadman:
            measured, action, reason = self._hold(driver)
            self.state = TeleopState.READY_IDLE if reason == "ok" else TeleopState.FAULT
            self.was_deadman_pressed = False
            self.require_deadman_repress = False
            result.state = self.state
            result.action = action
            result.reason = "deadman_released" if reason == "ok" else reason
            result.measured_joints = measured
            return result

        if self.require_deadman_repress:
            self.state = TeleopState.HOLDING
            result.state = self.state
            result.reason = "release_deadman_required"
            return result

        if not self.was_deadman_pressed:
            measured = driver.read_joint_pose()
            self.controller_home = np.asarray(current_vr, dtype=float).copy()
            self.fixed_shoulder_xyz_m = self._estimate_calibrated_shoulder(current_vr, sample)
            if measured is not None:
                self.last_command_deg = measured.joints_deg.copy()
            elif self.last_command_deg is None:
                self.last_command_deg = self.mimic_config.neutral_deg.copy()
            self.was_deadman_pressed = True
            self.state = TeleopState.ACTIVE
            result.state = self.state
            result.reason = "armed_this_cycle"
            result.measured_joints = measured
            return result

        self._update_elbow_swivel(buttons, dt)
        shoulder = self._shoulder_for_sample(sample)
        human = build_human_arm_state(shoulder, current_vr, self.elbow_swivel_rad, self.human_config, self.previous_elbow)
        self.previous_elbow = human.elbow_xyz_m.copy()
        human_vector = human_arm_to_mimic_vector_deg(human)
        if self.human_home_vector_deg is None or self.robot_home_joints_deg is None:
            self.state = TeleopState.FAULT
            result.state = self.state
            result.reason = "missing_joint_mimic_calibration"
            return result
        raw_target = mimic_vector_to_piper_joints(human_vector, self.human_home_vector_deg, self.robot_home_joints_deg, self.mimic_config)
        if self.shoulder_lift_joystick:
            raw_target[1] = raw_target[1] + analog_axis(buttons, self.shoulder_lift_joystick) * 10.0
            raw_target = clamp_joints_deg(raw_target)
        previous = self.mimic_config.neutral_deg if self.last_command_deg is None else self.last_command_deg
        smoothed = previous + self.mimic_config.smoothing_alpha * (raw_target - previous)
        safe_target = rate_limit_joints_deg(smoothed, previous, self.mimic_config.max_joint_speed_deg_s, dt)
        driver.send_joint_pose(safe_target)
        self.last_command_deg = safe_target.copy()
        self.state = TeleopState.ACTIVE
        result.state = self.state
        result.human_arm = human
        result.human_vector_deg = human_vector
        result.human_home_vector_deg = self.human_home_vector_deg.copy()
        result.human_delta_deg = human_vector - self.human_home_vector_deg
        result.robot_home_joints_deg = self.robot_home_joints_deg.copy()
        result.raw_joint_target_deg = raw_target
        result.safe_joint_target_deg = safe_target
        result.action = "sent"
        result.reason = "ok"
        result.measured_joints = driver.read_joint_pose()
        return result

    def _estimate_calibrated_shoulder(self, current_vr: np.ndarray, sample: QuestSample) -> np.ndarray:
        hmd_pose = sample.transforms_openxr.get("hmd")
        if hmd_pose is None:
            hmd_pose = sample.transforms_openxr.get("head")
        if self.human_config.shoulder_source == "hmd" and hmd_pose is not None:
            return estimate_shoulder_from_hmd(hmd_pose, self.human_config)
        return np.asarray(current_vr, dtype=float)[:3, 3] + np.asarray(self.human_config.fixed_shoulder_from_hand_home_m, dtype=float)

    def _shoulder_for_sample(self, sample: QuestSample) -> np.ndarray:
        hmd_pose = sample.transforms_openxr.get("hmd")
        if hmd_pose is None:
            hmd_pose = sample.transforms_openxr.get("head")
        if self.human_config.shoulder_source == "hmd" and hmd_pose is not None:
            return estimate_shoulder_from_hmd(hmd_pose, self.human_config)
        if self.fixed_shoulder_xyz_m is None:
            raise RuntimeError("Joint mimic session is not calibrated")
        return self.fixed_shoulder_xyz_m.copy()

    def _update_elbow_swivel(self, buttons: dict[str, Any], dt: float) -> None:
        if self.elbow_swivel_joystick:
            self.elbow_swivel_rad += analog_axis(buttons, self.elbow_swivel_joystick) * self.human_config.elbow_swivel_speed_rad_s * dt
            self.elbow_swivel_rad = float(np.clip(self.elbow_swivel_rad, -np.pi, np.pi))

    def _hold(self, driver: Any) -> tuple[JointPose | None, str, str]:
        measured = driver.read_joint_pose()
        if measured is not None:
            self.last_command_deg = measured.joints_deg.copy()
            driver.send_joint_pose(measured.joints_deg)
            return measured, "sent", "ok"
        if getattr(driver, "has_sent_joint_command", False) and self.last_command_deg is not None:
            driver.hold_joints(allow_last_command_fallback=True)
            return None, "sent", "ok"
        return None, "skipped", "joint_feedback_required_for_hold"


def analog_axis(buttons: dict[str, Any], name: str) -> float:
    if name.endswith("_x") or name.endswith("_y"):
        base, axis = name.rsplit("_", 1)
        value = buttons.get(base, (0.0, 0.0))
        if isinstance(value, (tuple, list)) and len(value) >= 2:
            return float(value[0 if axis == "x" else 1])
        return 0.0
    return analog_value(buttons, name)
