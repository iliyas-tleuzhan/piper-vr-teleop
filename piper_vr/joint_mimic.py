"""Map inferred human arm posture to Piper joint targets."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .human_arm_model import HumanArmState
from .joint_limits import clamp_joints_deg


@dataclass
class JointMimicConfig:
    signs: np.ndarray
    offsets_deg: np.ndarray
    gains: np.ndarray
    neutral_deg: np.ndarray
    max_joint_speed_deg_s: np.ndarray
    smoothing_alpha: float
    idle_hold_hz: float = 0.0
    mapping_mode: str = "pose_delta"
    control_frame: str = "hmd_yaw"
    translation_deadband_m: float = 0.003
    rotation_deadband_deg: float = 2.0
    settle_frames_on_stop: int = 3
    cancel_backlog_on_stop: bool = True
    relative_gain_matrix: np.ndarray | None = None
    wrist_rotation_enabled: bool = False
    wrist_rotation_deadman: str = "rightTrig"
    max_tracking_error_deg: float = 12.0
    tracking_error_fault_frames: int = 10

    @classmethod
    def from_config(cls, config: dict | None) -> "JointMimicConfig":
        config = config or {}
        return cls(
            neutral_deg=np.asarray(config.get("neutral_deg", [0.0, 90.0, -90.0, 0.0, 0.0, 0.0]), dtype=float),
            offsets_deg=np.asarray(config.get("offsets_deg", [0.0] * 6), dtype=float),
            signs=np.asarray(config.get("signs", [1.0] * 6), dtype=float),
            gains=np.asarray(config.get("gains", [1.0] * 6), dtype=float),
            max_joint_speed_deg_s=np.asarray(config.get("max_joint_speed_deg_s", [25, 25, 25, 45, 45, 60]), dtype=float),
            smoothing_alpha=float(config.get("smoothing_alpha", 0.25)),
            idle_hold_hz=float(config.get("idle_hold_hz", 0.0)),
            mapping_mode=str(config.get("mapping_mode", "pose_delta")),
            control_frame=str(config.get("control_frame", "hmd_yaw")),
            translation_deadband_m=float(config.get("translation_deadband_m", 0.003)),
            rotation_deadband_deg=float(config.get("rotation_deadband_deg", 2.0)),
            settle_frames_on_stop=int(config.get("settle_frames_on_stop", 3)),
            cancel_backlog_on_stop=bool(config.get("cancel_backlog_on_stop", True)),
            relative_gain_matrix=np.asarray(config.get(
                "relative_gain_matrix",
                [
                    [30.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 30.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, -30.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                ],
            ), dtype=float),
            wrist_rotation_enabled=bool(config.get("wrist_rotation_enabled", False)),
            wrist_rotation_deadman=str(config.get("wrist_rotation_deadman", "rightTrig")),
            max_tracking_error_deg=float(config.get("max_tracking_error_deg", 12.0)),
            tracking_error_fault_frames=int(config.get("tracking_error_fault_frames", 10)),
        )

    def __post_init__(self) -> None:
        for name in ("signs", "offsets_deg", "gains", "neutral_deg", "max_joint_speed_deg_s"):
            value = np.asarray(getattr(self, name), dtype=float)
            if value.shape != (6,):
                raise ValueError(f"{name} must contain six values")
            setattr(self, name, value)
        if not 0.0 < self.smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in the range (0, 1]")
        if self.idle_hold_hz < 0.0:
            raise ValueError("idle_hold_hz must be non-negative")
        if self.mapping_mode not in ("pose_delta", "relative_delta", "relative_ik_posture"):
            raise ValueError("mapping_mode must be pose_delta, relative_delta, or relative_ik_posture")
        if self.translation_deadband_m < 0.0 or self.rotation_deadband_deg < 0.0:
            raise ValueError("deadbands must be non-negative")
        if self.settle_frames_on_stop < 1:
            raise ValueError("settle_frames_on_stop must be >= 1")
        if self.tracking_error_fault_frames < 1:
            raise ValueError("tracking_error_fault_frames must be >= 1")
        if self.max_tracking_error_deg < 0.0:
            raise ValueError("max_tracking_error_deg must be non-negative")
        self.relative_gain_matrix = np.asarray(self.relative_gain_matrix, dtype=float)
        if self.relative_gain_matrix.shape != (6, 6):
            raise ValueError("relative_gain_matrix must have shape (6, 6)")


def human_arm_to_mimic_vector_deg(human: HumanArmState) -> np.ndarray:
    """Return six human-derived posture channels in degrees."""
    shoulder_yaw, shoulder_pitch, shoulder_roll = np.asarray(human.shoulder_angles_deg, dtype=float)
    wrist_roll, wrist_pitch, wrist_yaw = np.asarray(human.wrist_angles_deg, dtype=float)
    return np.array(
        [
            shoulder_yaw,
            shoulder_pitch,
            human.elbow_flex_deg,
            shoulder_roll + wrist_roll,
            wrist_pitch,
            wrist_yaw,
        ],
        dtype=float,
    )


def mimic_vector_to_piper_joints(
    human_vector_deg: np.ndarray,
    human_home_vector_deg: np.ndarray,
    robot_home_joints_deg: np.ndarray,
    config: JointMimicConfig,
) -> np.ndarray:
    """Map a calibration-relative human posture delta to Piper joint targets."""
    human_vector_deg = np.asarray(human_vector_deg, dtype=float)
    human_home_vector_deg = np.asarray(human_home_vector_deg, dtype=float)
    robot_home_joints_deg = np.asarray(robot_home_joints_deg, dtype=float)
    for name, value in (
        ("human_vector_deg", human_vector_deg),
        ("human_home_vector_deg", human_home_vector_deg),
        ("robot_home_joints_deg", robot_home_joints_deg),
    ):
        if value.shape != (6,):
            raise ValueError(f"{name} must contain six values")
    delta = human_vector_deg - human_home_vector_deg
    target = robot_home_joints_deg + config.offsets_deg + config.signs * config.gains * delta
    return clamp_joints_deg(target)


def human_arm_to_piper_joints(human: HumanArmState, config: JointMimicConfig) -> np.ndarray:
    """Legacy absolute helper for dry-run/debug shortcuts.

    Runtime joint mimic teleop uses calibration-relative
    `mimic_vector_to_piper_joints` instead.
    """
    human_vector = human_arm_to_mimic_vector_deg(human)
    target = config.neutral_deg + config.offsets_deg + config.signs * config.gains * human_vector
    return clamp_joints_deg(target)
