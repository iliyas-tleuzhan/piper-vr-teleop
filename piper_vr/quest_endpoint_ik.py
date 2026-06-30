"""Quest-controller relative endpoint IK teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

import numpy as np

from .buttons import is_pressed
from .joint_limits import clamp_joints_deg, rate_limit_joints_deg
from .piper_driver import JointPose
from .piper_kinematics import PiperKinematics
from .types import QuestSample, TeleopState
from .vr_mapping import _matrix_to_rpy_deg


QUEST_AXIS_INDEX = {"quest_x": 0, "quest_y": 1, "quest_z": 2, "quest_roll": 0, "quest_pitch": 1, "quest_yaw": 2}


@dataclass(frozen=True)
class EndpointAxisMapping:
    robot_x: str = "-quest_z"
    robot_y: str = "-quest_x"
    robot_z: str = "+quest_y"

    @classmethod
    def from_config(cls, config: dict | None) -> "EndpointAxisMapping":
        config = config or {}
        return cls(
            robot_x=str(config.get("robot_x", "-quest_z")),
            robot_y=str(config.get("robot_y", "-quest_x")),
            robot_z=str(config.get("robot_z", "+quest_y")),
        )

    def apply(self, values: np.ndarray) -> np.ndarray:
        return np.array([_map_rule(self.robot_x, values), _map_rule(self.robot_y, values), _map_rule(self.robot_z, values)], dtype=float)


@dataclass(frozen=True)
class EndpointRotationMapping:
    robot_roll: str = "+quest_roll"
    robot_pitch: str = "+quest_pitch"
    robot_yaw: str = "+quest_yaw"

    @classmethod
    def from_config(cls, config: dict | None) -> "EndpointRotationMapping":
        config = config or {}
        return cls(
            robot_roll=str(config.get("robot_roll", "+quest_roll")),
            robot_pitch=str(config.get("robot_pitch", "+quest_pitch")),
            robot_yaw=str(config.get("robot_yaw", "+quest_yaw")),
        )

    def apply(self, values: np.ndarray) -> np.ndarray:
        return np.array([_map_rule(self.robot_roll, values), _map_rule(self.robot_pitch, values), _map_rule(self.robot_yaw, values)], dtype=float)


@dataclass
class QuestEndpointIKConfig:
    urdf_path: str = "third_party/agx_arm_urdf/piper/urdf/piper_description.urdf"
    ee_frame: str = "link6"
    scale: float = 1.0
    orientation_scale: float = 1.0
    position_deadband_m: float = 0.002
    orientation_deadband_deg: float = 1.5
    position_filter_alpha: float = 0.35
    orientation_filter_alpha: float = 0.35
    max_position_step_m: float = 0.02
    max_orientation_step_deg: float = 5.0
    workspace_min_m: np.ndarray = None
    workspace_max_m: np.ndarray = None
    max_joint_speed_deg_s: np.ndarray = None
    max_orientation_delta_deg: np.ndarray = None
    axis_mapping: EndpointAxisMapping = None
    rotation_mapping: EndpointRotationMapping = None
    orientation_enabled: bool = True
    translation_enabled: bool = True
    max_tracking_error_deg: float = 12.0
    tracking_error_fault_frames: int = 10

    @classmethod
    def from_config(cls, config: dict | None) -> "QuestEndpointIKConfig":
        config = config or {}
        return cls(
            urdf_path=str(config.get("urdf_path", "third_party/agx_arm_urdf/piper/urdf/piper_description.urdf")),
            ee_frame=str(config.get("ee_frame", "link6")),
            scale=float(config.get("scale", 1.0)),
            orientation_scale=float(config.get("orientation_scale", 1.0)),
            position_deadband_m=float(config.get("position_deadband_m", 0.002)),
            orientation_deadband_deg=float(config.get("orientation_deadband_deg", 1.5)),
            position_filter_alpha=float(config.get("position_filter_alpha", 0.35)),
            orientation_filter_alpha=float(config.get("orientation_filter_alpha", 0.35)),
            max_position_step_m=float(config.get("max_position_step_m", 0.02)),
            max_orientation_step_deg=float(config.get("max_orientation_step_deg", 5.0)),
            workspace_min_m=np.asarray(config.get("workspace_min_m", [0.18, -0.35, 0.08]), dtype=float),
            workspace_max_m=np.asarray(config.get("workspace_max_m", [0.65, 0.35, 0.60]), dtype=float),
            max_joint_speed_deg_s=np.asarray(config.get("max_joint_speed_deg_s", [45, 45, 45, 60, 60, 90]), dtype=float),
            max_orientation_delta_deg=np.asarray(config.get("max_orientation_delta_deg", [60, 60, 90]), dtype=float),
            axis_mapping=EndpointAxisMapping.from_config(config.get("axis_mapping")),
            rotation_mapping=EndpointRotationMapping.from_config(config.get("rotation_mapping")),
            orientation_enabled=bool(config.get("orientation_enabled", True)),
            translation_enabled=bool(config.get("translation_enabled", True)),
            max_tracking_error_deg=float(config.get("max_tracking_error_deg", 12.0)),
            tracking_error_fault_frames=int(config.get("tracking_error_fault_frames", 10)),
        )

    def __post_init__(self) -> None:
        for name in ("workspace_min_m", "workspace_max_m", "max_joint_speed_deg_s", "max_orientation_delta_deg"):
            value = np.asarray(getattr(self, name), dtype=float)
            if value.shape != (3,) and name != "max_joint_speed_deg_s":
                raise ValueError(f"{name} must contain three values")
            if name == "max_joint_speed_deg_s" and value.shape != (6,):
                raise ValueError("max_joint_speed_deg_s must contain six values")
            setattr(self, name, value)
        if not 0.0 < self.position_filter_alpha <= 1.0 or not 0.0 < self.orientation_filter_alpha <= 1.0:
            raise ValueError("filter alphas must be in the range (0, 1]")


@dataclass
class EndpointIKResult:
    state: TeleopState
    calibrated: bool
    deadman: bool = False
    calibrate: bool = False
    controller_xyz: np.ndarray | None = None
    controller_delta_xyz: np.ndarray | None = None
    mapped_robot_delta_xyz: np.ndarray | None = None
    controller_delta_rpy_deg: np.ndarray | None = None
    mapped_robot_delta_rpy_deg: np.ndarray | None = None
    target_xyz: np.ndarray | None = None
    target_rpy_deg: np.ndarray | None = None
    raw_joint_target_deg: np.ndarray | None = None
    safe_joint_target_deg: np.ndarray | None = None
    measured_joints: JointPose | None = None
    action: str = "skipped"
    reason: str = "none"
    position_error_m: float | None = None
    orientation_error_deg: float | None = None
    sample_age_s: float | None = None


class QuestEndpointIKSession:
    def __init__(
        self,
        *,
        side: str,
        deadman_button: str,
        calibrate_button: str,
        config: QuestEndpointIKConfig,
        stale_timeout_s: float,
    ) -> None:
        self.side = side
        self.deadman_button = deadman_button
        self.calibrate_button = calibrate_button
        self.config = config
        self.stale_timeout_s = float(stale_timeout_s)
        try:
            self.kinematics = PiperKinematics(config.urdf_path, tip_link=config.ee_frame)
        except Exception as exc:
            raise RuntimeError(f"Quest endpoint IK solver unavailable for URDF {config.urdf_path!r}: {exc}") from exc
        self.state = TeleopState.WAITING_FOR_DEVICE
        self.controller_home_transform: np.ndarray | None = None
        self.clutch_controller_home_transform: np.ndarray | None = None
        self.robot_home_joint_pose: np.ndarray | None = None
        self.robot_home_xyz: np.ndarray | None = None
        self.robot_home_rpy_deg: np.ndarray | None = None
        self.last_command_deg: np.ndarray | None = None
        self.filtered_xyz: np.ndarray | None = None
        self.filtered_rpy: np.ndarray | None = None
        self.last_step_s: float | None = None
        self.was_deadman_pressed = False
        self.was_calibrate_pressed = False
        self.require_deadman_repress = False
        self.tracking_error_frames = 0

    @property
    def calibrated(self) -> bool:
        return self.controller_home_transform is not None and self.robot_home_joint_pose is not None

    def step(self, sample: QuestSample | None, driver: Any) -> EndpointIKResult:
        now_s = time.monotonic()
        dt = 1e-3 if self.last_step_s is None else max(now_s - self.last_step_s, 1e-3)
        self.last_step_s = now_s
        if sample is None:
            self.state = TeleopState.WAITING_FOR_DEVICE
            return EndpointIKResult(self.state, self.calibrated, reason="no_quest_sample")

        buttons = sample.buttons
        deadman = is_pressed(buttons, self.deadman_button)
        calibrate = is_pressed(buttons, self.calibrate_button)
        current = sample.transforms_openxr.get(self.side)
        controller_xyz = None if current is None else np.asarray(current, dtype=float)[:3, 3].copy()
        result = EndpointIKResult(
            self.state,
            self.calibrated,
            deadman=deadman,
            calibrate=calibrate,
            controller_xyz=controller_xyz,
            sample_age_s=sample.age_s,
        )
        if current is None:
            self.state = TeleopState.WAITING_FOR_DEVICE
            result.state = self.state
            result.reason = f"missing_{self.side}_controller"
            return result
        if sample.age_s > self.stale_timeout_s:
            measured, action, reason = self._hold(driver)
            self.state = TeleopState.HOLDING if reason == "ok" else TeleopState.FAULT
            self.require_deadman_repress = True
            result.state = self.state
            result.action = action
            result.reason = "tracking_stale" if reason == "ok" else reason
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
                measured = JointPose(np.array([0.0, 90.0, -90.0, 0.0, 0.0, 0.0], dtype=float))
            self.controller_home_transform = np.asarray(current, dtype=float).copy()
            self.clutch_controller_home_transform = self.controller_home_transform.copy()
            self.robot_home_joint_pose = measured.joints_deg.copy()
            self.last_command_deg = measured.joints_deg.copy()
            xyz, rotation = self.kinematics.forward(np.radians(measured.joints_deg))
            self.robot_home_xyz = xyz.copy()
            self.robot_home_rpy_deg = _matrix_to_rpy_deg(rotation)
            self.filtered_xyz = self.robot_home_xyz.copy()
            self.filtered_rpy = self.robot_home_rpy_deg.copy()
            self.require_deadman_repress = True
            self.was_deadman_pressed = deadman
            self.state = TeleopState.READY_IDLE
            result.state = self.state
            result.calibrated = True
            result.measured_joints = measured
            result.target_xyz = self.robot_home_xyz.copy()
            result.target_rpy_deg = self.robot_home_rpy_deg.copy()
            result.reason = "calibrated_release_deadman"
            self.was_calibrate_pressed = calibrate
            return result
        self.was_calibrate_pressed = calibrate

        if not self.calibrated:
            self.state = TeleopState.WAITING_FOR_CALIBRATION
            result.state = self.state
            result.reason = "not_calibrated"
            return result
        if not deadman:
            measured, action, reason = self._hold(driver) if self.was_deadman_pressed else (None, "skipped", "deadman_released")
            self.state = TeleopState.READY_IDLE if reason in ("ok", "deadman_released") else TeleopState.FAULT
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
            if measured is None:
                if not getattr(driver, "dry_run", False):
                    self.state = TeleopState.FAULT
                    result.state = self.state
                    result.reason = "joint_feedback_required_for_clutch"
                    return result
                measured = JointPose(self.last_command_deg.copy())
            self.clutch_controller_home_transform = np.asarray(current, dtype=float).copy()
            self.robot_home_joint_pose = measured.joints_deg.copy()
            self.last_command_deg = measured.joints_deg.copy()
            xyz, rotation = self.kinematics.forward(np.radians(measured.joints_deg))
            self.robot_home_xyz = xyz.copy()
            self.robot_home_rpy_deg = _matrix_to_rpy_deg(rotation)
            self.filtered_xyz = self.robot_home_xyz.copy()
            self.filtered_rpy = self.robot_home_rpy_deg.copy()
            self.was_deadman_pressed = True
            self.state = TeleopState.ACTIVE
            result.state = self.state
            result.reason = "armed_this_cycle"
            result.measured_joints = measured
            return result

        tracking_result = self._tracking_error_guard(driver)
        if tracking_result is not None:
            tracking_result.deadman = deadman
            tracking_result.calibrate = calibrate
            tracking_result.controller_xyz = controller_xyz
            return tracking_result

        target_xyz, target_rpy, debug = endpoint_target_from_controller(
            self.clutch_controller_home_transform,
            current,
            self.robot_home_xyz,
            self.robot_home_rpy_deg,
            self.config,
        )
        if self.filtered_xyz is None or self.filtered_rpy is None:
            self.filtered_xyz = target_xyz.copy()
            self.filtered_rpy = target_rpy.copy()
        xyz_step = np.clip(target_xyz - self.filtered_xyz, -self.config.max_position_step_m, self.config.max_position_step_m)
        target_xyz = self.filtered_xyz + xyz_step
        rpy_step = np.clip(target_rpy - self.filtered_rpy, -self.config.max_orientation_step_deg, self.config.max_orientation_step_deg)
        target_rpy = self.filtered_rpy + rpy_step
        if np.linalg.norm(target_xyz - self.filtered_xyz) < self.config.position_deadband_m:
            target_xyz = self.filtered_xyz.copy()
        self.filtered_xyz = self.filtered_xyz + self.config.position_filter_alpha * (target_xyz - self.filtered_xyz)
        rpy_delta = target_rpy - self.filtered_rpy
        if np.linalg.norm(rpy_delta) < self.config.orientation_deadband_deg:
            target_rpy = self.filtered_rpy.copy()
        self.filtered_rpy = self.filtered_rpy + self.config.orientation_filter_alpha * (target_rpy - self.filtered_rpy)
        seed = np.radians(self.last_command_deg if self.last_command_deg is not None else self.robot_home_joint_pose)
        ik = self.kinematics.solve(self.filtered_xyz, self.filtered_rpy, seed)
        result.controller_delta_xyz = debug["controller_delta_xyz"]
        result.mapped_robot_delta_xyz = debug["mapped_robot_delta_xyz"]
        result.controller_delta_rpy_deg = debug["controller_delta_rpy_deg"]
        result.mapped_robot_delta_rpy_deg = debug["mapped_robot_delta_rpy_deg"]
        result.target_xyz = self.filtered_xyz.copy()
        result.target_rpy_deg = self.filtered_rpy.copy()
        result.position_error_m = ik.position_error_m
        result.orientation_error_deg = ik.orientation_error_deg
        if not ik.success:
            measured, action, _ = self._hold(driver)
            self.state = TeleopState.HOLDING
            result.state = self.state
            result.action = action
            result.reason = f"ik_failed:{ik.position_error_m:.3f}:{ik.orientation_error_deg:.1f}"
            result.measured_joints = measured
            return result
        raw_target = clamp_joints_deg(np.degrees(ik.joints_rad))
        previous = self.last_command_deg if self.last_command_deg is not None else raw_target
        safe_target = rate_limit_joints_deg(raw_target, previous, self.config.max_joint_speed_deg_s, dt)
        driver.send_joint_pose(safe_target)
        self.last_command_deg = safe_target.copy()
        self.state = TeleopState.ACTIVE
        result.state = self.state
        result.raw_joint_target_deg = raw_target
        result.safe_joint_target_deg = safe_target
        result.action = "sent"
        result.reason = "ok"
        result.measured_joints = driver.read_joint_pose()
        return result

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

    def _tracking_error_guard(self, driver: Any) -> EndpointIKResult | None:
        if self.last_command_deg is None or self.config.max_tracking_error_deg <= 0.0:
            return None
        measured = driver.read_joint_pose()
        if measured is None:
            return None
        error = float(np.linalg.norm(measured.joints_deg - self.last_command_deg))
        if error <= self.config.max_tracking_error_deg:
            self.tracking_error_frames = 0
            return None
        self.tracking_error_frames += 1
        if self.tracking_error_frames < self.config.tracking_error_fault_frames:
            return None
        self.state = TeleopState.FAULT
        self.last_command_deg = measured.joints_deg.copy()
        try:
            driver.send_joint_pose(measured.joints_deg)
            action = "sent"
        except RuntimeError:
            action = "skipped"
        return EndpointIKResult(self.state, self.calibrated, action=action, reason="joint_tracking_error_too_large", measured_joints=measured)


def _map_rule(rule: str, values: np.ndarray) -> float:
    if not isinstance(rule, str) or len(rule) < 3:
        raise ValueError(f"Invalid mapping rule: {rule!r}")
    sign_char = rule[0]
    axis = rule[1:]
    if sign_char not in ("+", "-") or axis not in QUEST_AXIS_INDEX:
        raise ValueError(f"Invalid mapping rule: {rule!r}")
    return (1.0 if sign_char == "+" else -1.0) * float(np.asarray(values, dtype=float)[QUEST_AXIS_INDEX[axis]])


def clamp_workspace(xyz: np.ndarray, config: QuestEndpointIKConfig) -> np.ndarray:
    return np.minimum(np.maximum(np.asarray(xyz, dtype=float), config.workspace_min_m), config.workspace_max_m)


def clamp_orientation_delta(delta_rpy_deg: np.ndarray, config: QuestEndpointIKConfig) -> np.ndarray:
    return np.clip(np.asarray(delta_rpy_deg, dtype=float), -config.max_orientation_delta_deg, config.max_orientation_delta_deg)


def endpoint_target_from_controller(
    controller_home_transform: np.ndarray,
    current_controller_transform: np.ndarray,
    robot_home_xyz: np.ndarray,
    robot_home_rpy_deg: np.ndarray,
    config: QuestEndpointIKConfig,
) -> tuple[np.ndarray, np.ndarray, dict[str, np.ndarray]]:
    home = np.asarray(controller_home_transform, dtype=float)
    current = np.asarray(current_controller_transform, dtype=float)
    controller_delta = np.linalg.inv(home) @ current
    controller_delta_xyz = controller_delta[:3, 3].copy()
    controller_delta_rpy = _matrix_to_rpy_deg(controller_delta[:3, :3])
    mapped_xyz = config.axis_mapping.apply(controller_delta_xyz) if config.translation_enabled else np.zeros(3)
    mapped_rpy = config.rotation_mapping.apply(controller_delta_rpy) if config.orientation_enabled else np.zeros(3)
    raw_target_xyz = np.asarray(robot_home_xyz, dtype=float) + mapped_xyz * config.scale
    target_xyz = clamp_workspace(raw_target_xyz, config)
    mapped_rpy = clamp_orientation_delta(mapped_rpy * config.orientation_scale, config)
    target_rpy = np.asarray(robot_home_rpy_deg, dtype=float) + mapped_rpy
    target_rpy = np.asarray(robot_home_rpy_deg, dtype=float) + np.clip(target_rpy - robot_home_rpy_deg, -config.max_orientation_delta_deg, config.max_orientation_delta_deg)
    return target_xyz, target_rpy, {
        "controller_delta_xyz": controller_delta_xyz,
        "mapped_robot_delta_xyz": mapped_xyz,
        "controller_delta_rpy_deg": controller_delta_rpy,
        "mapped_robot_delta_rpy_deg": mapped_rpy,
    }
