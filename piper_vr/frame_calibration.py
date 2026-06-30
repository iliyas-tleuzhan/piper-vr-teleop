"""Calibrated Quest control-frame helpers."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .types import QuestSample


@dataclass
class ControlFrameConfig:
    source: str = "hmd_yaw"
    vr_to_robot_axis: dict | np.ndarray | None = None
    translation_deadband_m: float = 0.003
    rotation_deadband_deg: float = 2.0


def yaw_only_rotation_from_hmd(hmd_transform: np.ndarray) -> np.ndarray:
    matrix = np.asarray(hmd_transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"hmd_transform must be 4x4, got {matrix.shape}")
    forward = matrix[:3, :3] @ np.array([0.0, 0.0, -1.0])
    yaw = np.arctan2(forward[0], -forward[2])
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def get_control_frame(
    sample: QuestSample,
    side: str,
    config: ControlFrameConfig,
    controller_home: np.ndarray | None = None,
) -> np.ndarray:
    if config.source == "hmd_yaw":
        hmd = sample.transforms_openxr.get("hmd")
        if hmd is not None:
            return yaw_only_rotation_from_hmd(hmd)
    if config.source in ("hmd_yaw", "controller_home") and controller_home is not None:
        return np.asarray(controller_home, dtype=float)[:3, :3].copy()
    if config.source not in ("hmd_yaw", "controller_home", "world"):
        raise ValueError(f"Unsupported control frame source: {config.source!r}")
    return np.eye(3)


def _rotation_vector_deg(rotation: np.ndarray) -> np.ndarray:
    rotation = np.asarray(rotation, dtype=float)
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    if angle < 1e-8:
        return np.zeros(3)
    vector = np.array(
        [rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]],
        dtype=float,
    )
    return np.degrees(vector * (angle / (2.0 * np.sin(angle))))


def controller_delta_in_control_frame(
    prev_controller: np.ndarray,
    current_controller: np.ndarray,
    control_frame: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    previous = np.asarray(prev_controller, dtype=float)
    current = np.asarray(current_controller, dtype=float)
    frame = np.asarray(control_frame, dtype=float)
    if previous.shape != (4, 4) or current.shape != (4, 4) or frame.shape != (3, 3):
        raise ValueError("controller transforms must be 4x4 and control_frame must be 3x3")
    translation_delta = frame.T @ (current[:3, 3] - previous[:3, 3])
    rotation_delta_world = current[:3, :3] @ previous[:3, :3].T
    rotation_delta = frame.T @ _rotation_vector_deg(rotation_delta_world)
    return translation_delta, rotation_delta
