"""Piper joint limits and joint command unit conversions."""

from __future__ import annotations

import numpy as np


PIPER_JOINT_LIMITS_DEG = {
    "joint_1": (-150.0, 150.0),
    "joint_2": (0.0, 180.0),
    "joint_3": (-170.0, 0.0),
    "joint_4": (-100.0, 100.0),
    "joint_5": (-70.0, 70.0),
    "joint_6": (-120.0, 120.0),
}

PIPER_JOINT_MIN_DEG = np.array([limits[0] for limits in PIPER_JOINT_LIMITS_DEG.values()], dtype=float)
PIPER_JOINT_MAX_DEG = np.array([limits[1] for limits in PIPER_JOINT_LIMITS_DEG.values()], dtype=float)


def _six(values: np.ndarray | list[float] | tuple[float, ...], name: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (6,):
        raise ValueError(f"{name} must contain six joint values, got shape {array.shape}")
    return array


def clamp_joints_deg(q_deg: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    """Clamp six joint angles in degrees to documented Piper limits."""
    return np.minimum(np.maximum(_six(q_deg, "q_deg"), PIPER_JOINT_MIN_DEG), PIPER_JOINT_MAX_DEG)


def rate_limit_joints_deg(
    q_target: np.ndarray | list[float] | tuple[float, ...],
    q_prev: np.ndarray | list[float] | tuple[float, ...],
    max_speed_deg_s: np.ndarray | list[float] | tuple[float, ...] | float,
    dt: float,
) -> np.ndarray:
    """Limit each joint step independently by max speed and elapsed time."""
    target = _six(q_target, "q_target")
    previous = _six(q_prev, "q_prev")
    speeds = np.full(6, float(max_speed_deg_s), dtype=float) if np.isscalar(max_speed_deg_s) else _six(max_speed_deg_s, "max_speed_deg_s")
    max_step = np.maximum(speeds, 0.0) * max(float(dt), 1e-3)
    return clamp_joints_deg(previous + np.clip(target - previous, -max_step, max_step))


def degrees_to_piper_joint_units(deg: float) -> int:
    """Convert degrees to Piper JointCtrl units, where 1 unit is 0.001 degrees."""
    return int(round(float(deg) * 1_000.0))


def piper_joint_units_to_degrees(raw: float) -> float:
    """Convert Piper JointCtrl feedback units to degrees."""
    return float(raw) / 1_000.0
