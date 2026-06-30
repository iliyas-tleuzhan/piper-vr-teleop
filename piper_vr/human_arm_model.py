"""Infer a practical human arm posture from Quest controller tracking."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .vr_mapping import _matrix_to_rpy_deg


@dataclass
class HumanArmConfig:
    shoulder_offset_from_hmd_m: np.ndarray
    upper_arm_length_m: float
    forearm_length_m: float
    elbow_swivel_default_rad: float
    elbow_swivel_speed_rad_s: float
    handedness: str = "right"
    shoulder_source: str = "fixed_from_calibration"
    fixed_shoulder_from_hand_home_m: np.ndarray | None = None

    @classmethod
    def from_config(cls, config: dict | None) -> "HumanArmConfig":
        config = config or {}
        return cls(
            shoulder_offset_from_hmd_m=np.asarray(config.get("shoulder_offset_from_hmd_m", [0.05, -0.18, -0.22]), dtype=float),
            fixed_shoulder_from_hand_home_m=np.asarray(config.get("fixed_shoulder_from_hand_home_m", [-0.25, 0.18, -0.15]), dtype=float),
            upper_arm_length_m=float(config.get("upper_arm_length_m", 0.30)),
            forearm_length_m=float(config.get("forearm_length_m", 0.27)),
            elbow_swivel_default_rad=np.radians(float(config.get("elbow_swivel_default_deg", -35.0))),
            elbow_swivel_speed_rad_s=np.radians(float(config.get("elbow_swivel_speed_deg_s", 60.0))),
            handedness=str(config.get("handedness", config.get("side", "right"))),
            shoulder_source=str(config.get("shoulder_source", "fixed_from_calibration")),
        )


@dataclass
class HumanArmState:
    shoulder_xyz_m: np.ndarray
    elbow_xyz_m: np.ndarray
    wrist_xyz_m: np.ndarray
    hand_rotation: np.ndarray
    shoulder_angles_deg: np.ndarray
    elbow_flex_deg: float
    wrist_angles_deg: np.ndarray


def estimate_shoulder_from_hmd(hmd_pose: np.ndarray | None, config: HumanArmConfig) -> np.ndarray:
    """Estimate shoulder in the Quest tracking frame from HMD pose.

    If no HMD pose is available, callers should use the fixed calibration
    fallback. That fallback is explicit because a controller alone cannot
    observe torso position.
    """
    if hmd_pose is None:
        raise ValueError("HMD pose is unavailable; use fixed shoulder calibration fallback")
    pose = np.asarray(hmd_pose, dtype=float)
    if pose.shape != (4, 4):
        raise ValueError(f"hmd_pose must be a 4x4 transform, got {pose.shape}")
    return pose[:3, 3] + pose[:3, :3] @ np.asarray(config.shoulder_offset_from_hmd_m, dtype=float)


def solve_elbow_position(
    shoulder: np.ndarray,
    wrist: np.ndarray,
    upper_len: float,
    forearm_len: float,
    swivel_angle: float,
    previous_elbow: np.ndarray | None = None,
) -> np.ndarray:
    """Solve elbow position using two-sphere geometry and a swivel angle."""
    shoulder = np.asarray(shoulder, dtype=float)
    wrist = np.asarray(wrist, dtype=float)
    line = wrist - shoulder
    distance = float(np.linalg.norm(line))
    if distance < 1e-6:
        axis = np.array([1.0, 0.0, 0.0])
    else:
        axis = line / distance

    min_reach = abs(float(upper_len) - float(forearm_len)) + 1e-4
    max_reach = float(upper_len) + float(forearm_len) - 1e-4
    reach = float(np.clip(distance, min_reach, max_reach))
    along = (upper_len * upper_len - forearm_len * forearm_len + reach * reach) / (2.0 * reach)
    radius = float(np.sqrt(max(upper_len * upper_len - along * along, 0.0)))
    center = shoulder + axis * along

    reference = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(reference, axis))) > 0.92:
        reference = np.array([0.0, 1.0, 0.0])
    basis_a = reference - axis * float(np.dot(reference, axis))
    basis_a /= max(float(np.linalg.norm(basis_a)), 1e-9)
    basis_b = np.cross(axis, basis_a)
    circle_dir = np.cos(swivel_angle) * basis_a + np.sin(swivel_angle) * basis_b
    elbow = center + radius * circle_dir
    if previous_elbow is not None:
        elbow = 0.75 * elbow + 0.25 * np.asarray(previous_elbow, dtype=float)
    return elbow


def _shoulder_angles(shoulder: np.ndarray, elbow: np.ndarray, wrist: np.ndarray) -> np.ndarray:
    upper = np.asarray(elbow, dtype=float) - np.asarray(shoulder, dtype=float)
    forearm = np.asarray(wrist, dtype=float) - np.asarray(elbow, dtype=float)
    upper_norm = max(float(np.linalg.norm(upper)), 1e-9)
    yaw = np.degrees(np.arctan2(upper[1], upper[0]))
    pitch = np.degrees(np.arctan2(upper[2], np.linalg.norm(upper[:2])))
    plane_normal = np.cross(upper, forearm)
    roll = np.degrees(np.arctan2(float(np.dot(plane_normal, [0.0, 0.0, 1.0])), upper_norm * max(float(np.linalg.norm(forearm)), 1e-9)))
    return np.array([yaw, pitch, roll], dtype=float)


def build_human_arm_state(shoulder: np.ndarray, wrist_transform: np.ndarray, swivel_angle: float, config: HumanArmConfig, previous_elbow: np.ndarray | None = None) -> HumanArmState:
    wrist_transform = np.asarray(wrist_transform, dtype=float)
    wrist = wrist_transform[:3, 3].copy()
    rotation = wrist_transform[:3, :3].copy()
    elbow = solve_elbow_position(shoulder, wrist, config.upper_arm_length_m, config.forearm_length_m, swivel_angle, previous_elbow)
    upper = elbow - shoulder
    forearm = wrist - elbow
    elbow_cos = float(np.dot(upper, forearm) / max(np.linalg.norm(upper) * np.linalg.norm(forearm), 1e-9))
    elbow_flex = 180.0 - np.degrees(np.arccos(np.clip(elbow_cos, -1.0, 1.0)))
    return HumanArmState(
        shoulder_xyz_m=np.asarray(shoulder, dtype=float).copy(),
        elbow_xyz_m=elbow,
        wrist_xyz_m=wrist,
        hand_rotation=rotation,
        shoulder_angles_deg=_shoulder_angles(shoulder, elbow, wrist),
        elbow_flex_deg=float(elbow_flex),
        wrist_angles_deg=_matrix_to_rpy_deg(rotation),
    )
