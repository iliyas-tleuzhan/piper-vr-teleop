"""Quest-controller relative endpoint IK teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

import numpy as np

from .buttons import is_pressed
from .frame_calibration import ControlFrameConfig, controller_delta_in_control_frame, get_control_frame
from .joint_limits import clamp_joints_deg, rate_limit_joints_deg
from .piper_driver import JointPose
from .piper_official_kinematics import create_official_fk
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
    backend: str = "firmware_endpoint"
    urdf_path: str = "third_party/agx_arm_urdf/piper/urdf/piper_description.urdf"
    ee_frame: str = "link6"
    scale: float = 1.0
    scale_xyz: np.ndarray = None
    orientation_scale: float = 1.0
    position_only_default: bool = True
    position_deadband_m: float = 0.002
    orientation_deadband_deg: float = 1.5
    position_filter_alpha: float = 0.35
    orientation_filter_alpha: float = 0.35
    max_position_step_m: float = 0.02
    max_position_step_m_xyz: np.ndarray = None
    max_orientation_step_deg: float = 5.0
    max_delta_from_home_m: np.ndarray = None
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
    ik_position_tolerance_m: float = 0.008
    ik_orientation_tolerance_deg: float = 10.0
    control_frame: str = "hmd_yaw"

    @classmethod
    def from_config(cls, config: dict | None) -> "QuestEndpointIKConfig":
        config = config or {}
        return cls(
            backend=str(config.get("backend", "firmware_endpoint")),
            urdf_path=str(config.get("urdf_path", "third_party/agx_arm_urdf/piper/urdf/piper_description.urdf")),
            ee_frame=str(config.get("ee_frame", "link6")),
            scale=float(config.get("scale", 1.0)),
            scale_xyz=np.asarray(config.get("scale_xyz", [1.0, 1.0, 1.0]), dtype=float),
            orientation_scale=float(config.get("orientation_scale", 1.0)),
            position_only_default=bool(config.get("position_only_default", True)),
            position_deadband_m=float(config.get("position_deadband_m", 0.002)),
            orientation_deadband_deg=float(config.get("orientation_deadband_deg", 1.5)),
            position_filter_alpha=float(config.get("position_filter_alpha", 0.35)),
            orientation_filter_alpha=float(config.get("orientation_filter_alpha", 0.35)),
            max_position_step_m=float(config.get("max_position_step_m", 0.02)),
            max_position_step_m_xyz=np.asarray(config.get("max_position_step_m_xyz", [config.get("max_position_step_m", 0.02)] * 3), dtype=float),
            max_orientation_step_deg=float(config.get("max_orientation_step_deg", 5.0)),
            max_delta_from_home_m=np.asarray(config.get("max_delta_from_home_m", [0.20, 0.20, 0.18]), dtype=float),
            workspace_min_m=np.asarray(config.get("workspace_min_m", [0.18, -0.35, 0.08]), dtype=float),
            workspace_max_m=np.asarray(config.get("workspace_max_m", [0.65, 0.35, 0.60]), dtype=float),
            max_joint_speed_deg_s=np.asarray(config.get("max_joint_speed_deg_s", [45, 45, 45, 60, 60, 90]), dtype=float),
            max_orientation_delta_deg=np.asarray(config.get("max_orientation_delta_deg", [60, 60, 90]), dtype=float),
            axis_mapping=EndpointAxisMapping.from_config(config.get("axis_mapping")),
            rotation_mapping=EndpointRotationMapping.from_config(config.get("rotation_mapping")),
            orientation_enabled=bool(config.get("orientation_enabled", not bool(config.get("position_only_default", True)))),
            translation_enabled=bool(config.get("translation_enabled", True)),
            max_tracking_error_deg=float(config.get("max_tracking_error_deg", 12.0)),
            tracking_error_fault_frames=int(config.get("tracking_error_fault_frames", 10)),
            ik_position_tolerance_m=float(config.get("ik_position_tolerance_m", 0.008)),
            ik_orientation_tolerance_deg=float(config.get("ik_orientation_tolerance_deg", 10.0)),
            control_frame=str(config.get("control_frame", "hmd_yaw")),
        )

    def __post_init__(self) -> None:
        if self.backend not in ("firmware_endpoint", "host_ik_sdk_fk", "host_ik_urdf"):
            raise ValueError("quest_endpoint_ik.backend must be firmware_endpoint, host_ik_sdk_fk, or host_ik_urdf")
        if self.control_frame not in ("hmd_yaw", "controller_home", "world"):
            raise ValueError("quest_endpoint_ik.control_frame must be hmd_yaw, controller_home, or world")
        for name in ("workspace_min_m", "workspace_max_m", "max_joint_speed_deg_s", "max_orientation_delta_deg", "max_delta_from_home_m", "scale_xyz", "max_position_step_m_xyz"):
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
    scaled_robot_delta_xyz: np.ndarray | None = None
    target_before_home_clamp: np.ndarray | None = None
    target_after_home_clamp: np.ndarray | None = None
    target_after_workspace_clamp: np.ndarray | None = None
    clamped_axes: list[str] | None = None
    control_frame: str | None = None
    scale: float | None = None
    scale_xyz: np.ndarray | None = None
    axis_mapping: dict[str, str] | None = None
    fk_backend: str | None = None
    target_rpy_deg: np.ndarray | None = None
    raw_joint_target_deg: np.ndarray | None = None
    safe_joint_target_deg: np.ndarray | None = None
    measured_joints: JointPose | None = None
    action: str = "skipped"
    reason: str = "none"
    position_error_m: float | None = None
    orientation_error_deg: float | None = None
    sample_age_s: float | None = None


@dataclass
class EndpointIKSolveResult:
    success: bool
    joints_deg: np.ndarray
    position_error_m: float
    orientation_error_deg: float
    reason: str = "ok"
    condition_number: float | None = None


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
        self.kinematics = self._build_kinematics(config)
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

    def _build_kinematics(self, config: QuestEndpointIKConfig) -> Any | None:
        if config.backend == "firmware_endpoint":
            return None
        if config.backend == "host_ik_sdk_fk":
            return create_official_fk(prefer_sdk=True)
        try:
            return PiperKinematics(config.urdf_path, tip_link=config.ee_frame)
        except Exception as exc:
            raise RuntimeError(
                f"urdf_missing: Quest endpoint IK URDF backend could not load {config.urdf_path!r}. "
                "Run `git submodule update --init --recursive`."
            ) from exc

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
            xyz, rotation = self._home_endpoint_pose(driver, measured)
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
            xyz, rotation = self._home_endpoint_pose(driver, measured)
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
            control_frame=get_control_frame(sample, self.side, ControlFrameConfig(source=self.config.control_frame), self.clutch_controller_home_transform),
        )
        if self.filtered_xyz is None or self.filtered_rpy is None:
            self.filtered_xyz = target_xyz.copy()
            self.filtered_rpy = target_rpy.copy()
        xyz_step = np.clip(target_xyz - self.filtered_xyz, -self.config.max_position_step_m_xyz, self.config.max_position_step_m_xyz)
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
        result.controller_delta_xyz = debug["controller_delta_xyz"]
        result.mapped_robot_delta_xyz = debug["mapped_robot_delta_xyz"]
        result.scaled_robot_delta_xyz = debug["scaled_robot_delta_xyz"]
        result.target_before_home_clamp = debug["target_before_home_clamp"]
        result.target_after_home_clamp = debug["target_after_home_clamp"]
        result.target_after_workspace_clamp = debug["target_after_workspace_clamp"]
        result.clamped_axes = debug["clamped_axes"]
        result.control_frame = self.config.control_frame
        result.scale = self.config.scale
        result.scale_xyz = self.config.scale_xyz.copy()
        result.axis_mapping = {
            "robot_x": self.config.axis_mapping.robot_x,
            "robot_y": self.config.axis_mapping.robot_y,
            "robot_z": self.config.axis_mapping.robot_z,
        }
        result.fk_backend = None if self.kinematics is None else getattr(self.kinematics, "backend_name", type(self.kinematics).__name__)
        result.controller_delta_rpy_deg = debug["controller_delta_rpy_deg"]
        result.mapped_robot_delta_rpy_deg = debug["mapped_robot_delta_rpy_deg"]
        result.target_xyz = self.filtered_xyz.copy()
        result.target_rpy_deg = self.filtered_rpy.copy()
        if debug.get("workspace_clamped", False):
            result.reason = "target_clamped_workspace"
        if debug.get("home_delta_clamped", False):
            result.reason = "target_clamped_home_delta"
        if self.config.backend == "firmware_endpoint":
            driver.send_end_pose(self.filtered_xyz, self.filtered_rpy)
            self.state = TeleopState.ACTIVE
            result.state = self.state
            result.action = "sent"
            result.reason = result.reason if result.reason != "none" else "ok"
            result.measured_joints = driver.read_joint_pose()
            return result

        ik = solve_endpoint_ik(
            self.kinematics,
            self.filtered_xyz,
            self.filtered_rpy,
            self.last_command_deg,
            self.robot_home_joint_pose,
            self.config,
        )
        result.position_error_m = ik.position_error_m
        result.orientation_error_deg = ik.orientation_error_deg
        if not ik.success:
            measured, action, _ = self._hold(driver)
            self.state = TeleopState.HOLDING
            result.state = self.state
            result.action = action
            result.reason = ik.reason
            result.measured_joints = measured
            return result
        raw_target = clamp_joints_deg(ik.joints_deg)
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
        if self.config.backend == "firmware_endpoint":
            try:
                driver.hold()
                return driver.read_joint_pose(), "sent", "ok"
            except RuntimeError as exc:
                return None, "skipped", f"firmware_endpoint_hold_failed:{exc}"
        measured = driver.read_joint_pose()
        if measured is not None:
            self.last_command_deg = measured.joints_deg.copy()
            driver.send_joint_pose(measured.joints_deg)
            return measured, "sent", "ok"
        if getattr(driver, "has_sent_joint_command", False) and self.last_command_deg is not None:
            driver.hold_joints(allow_last_command_fallback=True)
            return None, "sent", "ok"
        return None, "skipped", "joint_feedback_required_for_hold"

    def _home_endpoint_pose(self, driver: Any, measured: JointPose) -> tuple[np.ndarray, np.ndarray]:
        if self.config.backend == "firmware_endpoint":
            pose = driver.read_end_pose()
            if pose is not None:
                return pose.xyz_m.copy(), _rpy_matrix(np.radians(pose.rpy_deg))
        if self.kinematics is None:
            raise RuntimeError("fk_unavailable: endpoint FK is unavailable")
        return self.kinematics.forward(np.radians(measured.joints_deg))

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


def clamp_to_home_box(xyz: np.ndarray, home_xyz: np.ndarray, config: QuestEndpointIKConfig) -> np.ndarray:
    home = np.asarray(home_xyz, dtype=float)
    return np.minimum(np.maximum(np.asarray(xyz, dtype=float), home - config.max_delta_from_home_m), home + config.max_delta_from_home_m)


def clamp_orientation_delta(delta_rpy_deg: np.ndarray, config: QuestEndpointIKConfig) -> np.ndarray:
    return np.clip(np.asarray(delta_rpy_deg, dtype=float), -config.max_orientation_delta_deg, config.max_orientation_delta_deg)


def endpoint_target_from_controller(
    controller_home_transform: np.ndarray,
    current_controller_transform: np.ndarray,
    robot_home_xyz: np.ndarray,
    robot_home_rpy_deg: np.ndarray,
    config: QuestEndpointIKConfig,
    control_frame: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, dict[str, np.ndarray]]:
    home = np.asarray(controller_home_transform, dtype=float)
    current = np.asarray(current_controller_transform, dtype=float)
    frame = np.eye(3) if control_frame is None else np.asarray(control_frame, dtype=float)
    controller_delta_xyz, controller_delta_rpy = controller_delta_in_control_frame(home, current, frame)
    mapped_xyz = config.axis_mapping.apply(controller_delta_xyz) if config.translation_enabled else np.zeros(3)
    mapped_rpy = config.rotation_mapping.apply(controller_delta_rpy) if config.orientation_enabled else np.zeros(3)
    scaled_xyz = mapped_xyz * config.scale * config.scale_xyz
    raw_target_xyz = np.asarray(robot_home_xyz, dtype=float) + scaled_xyz
    home_clamped_xyz = clamp_to_home_box(raw_target_xyz, robot_home_xyz, config)
    target_xyz = clamp_workspace(home_clamped_xyz, config)
    mapped_rpy = clamp_orientation_delta(mapped_rpy * config.orientation_scale, config)
    target_rpy = np.asarray(robot_home_rpy_deg, dtype=float) + mapped_rpy
    target_rpy = np.asarray(robot_home_rpy_deg, dtype=float) + np.clip(target_rpy - robot_home_rpy_deg, -config.max_orientation_delta_deg, config.max_orientation_delta_deg)
    return target_xyz, target_rpy, {
        "controller_delta_xyz": controller_delta_xyz,
        "mapped_robot_delta_xyz": mapped_xyz,
        "scaled_robot_delta_xyz": scaled_xyz,
        "controller_delta_rpy_deg": controller_delta_rpy,
        "mapped_robot_delta_rpy_deg": mapped_rpy,
        "target_before_home_clamp": raw_target_xyz,
        "target_after_home_clamp": home_clamped_xyz,
        "target_after_workspace_clamp": target_xyz,
        "clamped_axes": [axis for axis, before, after in zip(("x", "y", "z"), raw_target_xyz, target_xyz, strict=True) if not np.isclose(before, after)],
        "home_delta_clamped": not np.allclose(raw_target_xyz, home_clamped_xyz),
        "workspace_clamped": not np.allclose(home_clamped_xyz, target_xyz),
    }


def solve_endpoint_ik(
    kinematics: Any,
    target_xyz: np.ndarray,
    target_rpy_deg: np.ndarray,
    last_command_deg: np.ndarray | None,
    home_joints_deg: np.ndarray,
    config: QuestEndpointIKConfig,
) -> EndpointIKSolveResult:
    if isinstance(kinematics, PiperKinematics):
        seeds = _ik_seeds(last_command_deg, home_joints_deg)
        best = None
        for seed_deg in seeds:
            result = kinematics.solve(
                target_xyz,
                target_rpy_deg,
                np.radians(seed_deg),
                position_tolerance_m=config.ik_position_tolerance_m,
                orientation_tolerance_deg=180.0 if not config.orientation_enabled else config.ik_orientation_tolerance_deg,
            )
            candidate = EndpointIKSolveResult(
                result.success,
                clamp_joints_deg(np.degrees(result.joints_rad)),
                result.position_error_m,
                0.0 if not config.orientation_enabled else result.orientation_error_deg,
                "ok" if result.success else _ik_failure_reason(result.position_error_m, result.orientation_error_deg, config),
            )
            if best is None or candidate.position_error_m < best.position_error_m:
                best = candidate
            if candidate.success:
                return candidate
        return best

    return _solve_generic_fk_ik(kinematics, target_xyz, target_rpy_deg, last_command_deg, home_joints_deg, config)


def _ik_seeds(last_command_deg: np.ndarray | None, home_joints_deg: np.ndarray) -> list[np.ndarray]:
    neutral = np.array([0.0, 90.0, -90.0, 0.0, 0.0, 0.0], dtype=float)
    seeds = [np.asarray(home_joints_deg, dtype=float), neutral]
    if last_command_deg is not None:
        seeds.insert(0, np.asarray(last_command_deg, dtype=float))
    for delta in (np.array([8, 0, 0, 0, 0, 0]), np.array([-8, 0, 0, 0, 0, 0]), np.array([0, 8, -8, 0, 0, 0]), np.array([0, -8, 8, 0, 0, 0])):
        seeds.append(clamp_joints_deg(np.asarray(home_joints_deg, dtype=float) + delta))
    return seeds


def _solve_generic_fk_ik(
    fk: Any,
    target_xyz: np.ndarray,
    target_rpy_deg: np.ndarray,
    last_command_deg: np.ndarray | None,
    home_joints_deg: np.ndarray,
    config: QuestEndpointIKConfig,
) -> EndpointIKSolveResult:
    best: EndpointIKSolveResult | None = None
    for seed_deg in _ik_seeds(last_command_deg, home_joints_deg):
        q = np.radians(seed_deg).astype(float)
        home_rad = np.radians(home_joints_deg)
        condition_number = None
        for _ in range(90):
            err = _fk_error(fk, q, target_xyz, target_rpy_deg, config)
            pos_err = float(np.linalg.norm(err[:3]))
            ori_err = float(np.degrees(np.linalg.norm(err[3:])))
            if pos_err <= config.ik_position_tolerance_m and (not config.orientation_enabled or ori_err <= config.ik_orientation_tolerance_deg):
                break
            jacobian = _numeric_jacobian(fk, q, target_xyz, target_rpy_deg, config)
            try:
                condition_number = float(np.linalg.cond(jacobian @ jacobian.T))
            except np.linalg.LinAlgError:
                condition_number = None
            posture = 0.015 * (home_rad - q)
            damping = 0.10
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping * damping * np.eye(6), err) + posture
            q = np.radians(clamp_joints_deg(np.degrees(q + np.clip(step, -0.10, 0.10))))
        err = _fk_error(fk, q, target_xyz, target_rpy_deg, config)
        pos_err = float(np.linalg.norm(err[:3]))
        ori_err = float(np.degrees(np.linalg.norm(err[3:])))
        success = pos_err <= config.ik_position_tolerance_m and (not config.orientation_enabled or ori_err <= config.ik_orientation_tolerance_deg)
        candidate = EndpointIKSolveResult(
            success,
            clamp_joints_deg(np.degrees(q)),
            pos_err,
            0.0 if not config.orientation_enabled else ori_err,
            "ok" if success else _ik_failure_reason(pos_err, ori_err, config),
            condition_number,
        )
        if best is None or candidate.position_error_m < best.position_error_m:
            best = candidate
        if success:
            return candidate
    return best


def _fk_error(fk: Any, q_rad: np.ndarray, target_xyz: np.ndarray, target_rpy_deg: np.ndarray, config: QuestEndpointIKConfig) -> np.ndarray:
    xyz, rotation = fk.forward(q_rad)
    pos = np.asarray(target_xyz, dtype=float) - xyz
    if not config.orientation_enabled:
        return np.concatenate((pos, np.zeros(3)))
    target_rot = _rpy_matrix(np.radians(target_rpy_deg))
    return np.concatenate((pos, _rotation_vector(rotation.T @ target_rot)))


def _numeric_jacobian(fk: Any, q_rad: np.ndarray, target_xyz: np.ndarray, target_rpy_deg: np.ndarray, config: QuestEndpointIKConfig) -> np.ndarray:
    epsilon = 1e-5
    jacobian = np.empty((6, 6))
    for index in range(6):
        plus = q_rad.copy()
        minus = q_rad.copy()
        plus[index] += epsilon
        minus[index] -= epsilon
        jacobian[:, index] = (_fk_error(fk, minus, target_xyz, target_rpy_deg, config) - _fk_error(fk, plus, target_xyz, target_rpy_deg, config)) / (2 * epsilon)
    return jacobian


def _ik_failure_reason(position_error: float, orientation_error: float, config: QuestEndpointIKConfig) -> str:
    if position_error > config.ik_position_tolerance_m:
        return "ik_failed_position_error"
    if config.orientation_enabled and orientation_error > config.ik_orientation_tolerance_deg:
        return "ik_failed_orientation_error"
    return "ik_failed"


def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    if angle < 1e-8:
        return np.zeros(3)
    vector = np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]])
    return vector * (angle / (2.0 * np.sin(angle)))


def _rpy_matrix(rpy_rad: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = np.asarray(rpy_rad, dtype=float)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )
