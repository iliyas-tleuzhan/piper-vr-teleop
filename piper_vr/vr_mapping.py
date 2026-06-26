"""Map Quest controller motion into Piper Cartesian target motion."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


AXIS_INDEX = {"vr_x": 0, "vr_y": 1, "vr_z": 2}


@dataclass(frozen=True)
class AxisMapping:
    # OpenXR controller-local coordinates: +X right, +Y up, -Z forward.
    # Piper base coordinates conventionally use +X forward, +Y left, +Z up.
    piper_x: str = "-vr_z"
    piper_y: str = "-vr_x"
    piper_z: str = "+vr_y"
    translation_frame: str = "controller_home"

    @classmethod
    def from_config(cls, config: dict | None) -> "AxisMapping":
        config = config or {}
        return cls(
            piper_x=config.get("piper_x", "-vr_z"),
            piper_y=config.get("piper_y", "-vr_x"),
            piper_z=config.get("piper_z", "+vr_y"),
            translation_frame=config.get("translation_frame", "controller_home"),
        )

    def apply(self, delta_vr_m: np.ndarray) -> np.ndarray:
        return np.array(
            [
                _map_one(self.piper_x, delta_vr_m),
                _map_one(self.piper_y, delta_vr_m),
                _map_one(self.piper_z, delta_vr_m),
            ],
            dtype=float,
        )

    def apply_rotation(self, delta_vr_rpy_deg: np.ndarray) -> np.ndarray:
        """Map controller-local roll/pitch/yaw changes to Piper RPY changes."""
        return self.apply(delta_vr_rpy_deg)


def _map_one(rule: str, delta_vr_m: np.ndarray) -> float:
    if not isinstance(rule, str) or len(rule) < 5:
        raise ValueError(f"Invalid axis mapping rule: {rule!r}")
    sign_char = rule[0]
    axis_name = rule[1:]
    if sign_char not in ("+", "-") or axis_name not in AXIS_INDEX:
        raise ValueError(f"Invalid axis mapping rule: {rule!r}")
    sign = 1.0 if sign_char == "+" else -1.0
    return sign * float(delta_vr_m[AXIS_INDEX[axis_name]])


def controller_translation(transform: np.ndarray) -> np.ndarray:
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"Expected a 4x4 transform, got shape {matrix.shape}")
    return matrix[:3, 3].copy()


def _matrix_to_rpy_deg(matrix: np.ndarray) -> np.ndarray:
    """Return intrinsic roll/pitch/yaw degrees for a proper 3x3 rotation matrix."""
    rotation = np.asarray(matrix, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError(f"Expected a 3x3 rotation, got shape {rotation.shape}")
    pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
    if abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:  # Gimbal lock: retain a stable zero yaw solution.
        roll = np.arctan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return np.degrees([roll, pitch, yaw])


def target_from_home(
    vr_home_transform: np.ndarray,
    vr_current_transform: np.ndarray,
    piper_home_xyz_m: np.ndarray,
    mapping: AxisMapping,
    scale: float,
) -> np.ndarray:
    vr_delta_m = controller_translation(vr_current_transform) - controller_translation(vr_home_transform)
    if mapping.translation_frame == "controller_home":
        # Quest positions are in its room frame.  Expressing translation in the
        # controller orientation captured at calibration makes forward/up/right
        # remain intuitive even when the headset faces a different direction.
        vr_delta_m = np.asarray(vr_home_transform, dtype=float)[:3, :3].T @ vr_delta_m
    elif mapping.translation_frame != "quest_world":
        raise ValueError(f"Invalid translation frame: {mapping.translation_frame!r}")
    piper_delta_m = mapping.apply(vr_delta_m) * float(scale)
    return np.asarray(piper_home_xyz_m, dtype=float) + piper_delta_m


def orientation_target_from_home(
    vr_home_transform: np.ndarray,
    vr_current_transform: np.ndarray,
    piper_home_rpy_deg: np.ndarray,
    mapping: AxisMapping,
    scale: float,
    max_delta_deg: np.ndarray,
) -> np.ndarray:
    """Map controller rotation relative to the clutch point into endpoint RPY."""
    home_rotation = np.asarray(vr_home_transform, dtype=float)[:3, :3]
    current_rotation = np.asarray(vr_current_transform, dtype=float)[:3, :3]
    controller_delta_rpy_deg = _matrix_to_rpy_deg(home_rotation.T @ current_rotation)
    piper_delta_deg = mapping.apply_rotation(controller_delta_rpy_deg) * float(scale)
    max_delta_deg = np.asarray(max_delta_deg, dtype=float)
    if max_delta_deg.shape != (3,) or np.any(max_delta_deg < 0):
        raise ValueError("max_orientation_delta_deg must contain three non-negative values")
    piper_delta_deg = np.clip(piper_delta_deg, -max_delta_deg, max_delta_deg)
    return np.asarray(piper_home_rpy_deg, dtype=float) + piper_delta_deg
