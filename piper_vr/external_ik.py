"""Host-side Piper IK with a posture objective for joint-space commanding."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .joint_limits import clamp_joints_deg
from .piper_kinematics import PiperKinematics


@dataclass
class IKJointResult:
    success: bool
    q_deg: np.ndarray
    position_error_m: float
    orientation_error_deg: float
    reason: str = "ok"


def solve_piper_ik_with_posture(
    target_xyz_m: np.ndarray,
    target_rpy_deg: np.ndarray,
    posture_seed_deg: np.ndarray,
    previous_q_deg: np.ndarray | None,
    weights: dict[str, Any] | None,
) -> IKJointResult:
    """Solve Piper joint angles from an endpoint target and posture seed.

    This keeps external IK optional. It uses the existing URDF solver and blends
    the previous joint command toward the human-posture seed before solving.
    """
    weights = weights or {}
    urdf_path = Path(weights.get("urdf_path", "third_party/agx_arm_urdf/piper/urdf/piper_description.urdf"))
    posture_weight = float(weights.get("posture_weight", 0.35))
    posture_seed_deg = clamp_joints_deg(posture_seed_deg)
    previous = posture_seed_deg if previous_q_deg is None else clamp_joints_deg(previous_q_deg)
    seed_deg = (1.0 - posture_weight) * previous + posture_weight * posture_seed_deg
    solver = PiperKinematics(urdf_path)
    result = solver.solve(target_xyz_m, target_rpy_deg, np.radians(seed_deg))
    q_deg = clamp_joints_deg(np.degrees(result.joints_rad))
    return IKJointResult(result.success, q_deg, result.position_error_m, result.orientation_error_deg, "ok" if result.success else "ik_unreachable")
