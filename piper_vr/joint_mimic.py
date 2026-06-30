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
        )

    def __post_init__(self) -> None:
        for name in ("signs", "offsets_deg", "gains", "neutral_deg", "max_joint_speed_deg_s"):
            value = np.asarray(getattr(self, name), dtype=float)
            if value.shape != (6,):
                raise ValueError(f"{name} must contain six values")
            setattr(self, name, value)
        if not 0.0 < self.smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in the range (0, 1]")


def human_arm_to_piper_joints(human: HumanArmState, config: JointMimicConfig) -> np.ndarray:
    """Convert approximate human arm angles to six Piper joint angles in degrees.

    The axis correspondence is intentionally simple and fully tuneable through
    signs, gains, and offsets. Real hardware should be tuned at low speed.
    """
    shoulder_yaw, shoulder_pitch, shoulder_roll = np.asarray(human.shoulder_angles_deg, dtype=float)
    wrist_roll, wrist_pitch, wrist_yaw = np.asarray(human.wrist_angles_deg, dtype=float)
    human_vector = np.array(
        [
            shoulder_yaw,
            shoulder_pitch,
            human.elbow_flex_deg - 90.0,
            shoulder_roll + wrist_roll,
            wrist_pitch,
            wrist_yaw,
        ],
        dtype=float,
    )
    target = config.neutral_deg + config.offsets_deg + config.signs * config.gains * human_vector
    return clamp_joints_deg(target)
