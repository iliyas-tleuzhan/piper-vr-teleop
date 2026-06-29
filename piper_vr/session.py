"""Teleoperation session state machine."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any
from collections.abc import Callable

import numpy as np

from .buttons import is_pressed
from .piper_driver import EndPose
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
