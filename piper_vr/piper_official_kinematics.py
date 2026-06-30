"""Official-SDK-compatible Piper forward kinematics helpers.

The real AgileX SDK exposes ``piper_sdk.kinematics.C_PiperForwardKinematics``.
This module prefers that implementation when installed and provides a small
local DH-compatible fallback so endpoint IK tests and dry-run tools do not
depend on the URDF submodule.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from .vr_mapping import _matrix_to_rpy_deg


def _dh(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


@dataclass
class PiperOfficialDHForwardKinematics:
    """Local FK fallback using Piper-sized DH geometry in meters.

    The constants intentionally live in meters internally. They are close to the
    public Piper URDF geometry and provide stable host-side FK/IK when the SDK FK
    object is not importable.
    """

    # a, alpha, d, theta_offset
    dh_parameters: tuple[tuple[float, float, float, float], ...] = (
        (0.0, math.pi / 2.0, 0.123, 0.0),
        (0.285, 0.0, 0.0, math.pi / 2.0),
        (0.021, -math.pi / 2.0, 0.0, -math.pi / 2.0),
        (0.0, math.pi / 2.0, 0.250, 0.0),
        (0.0, -math.pi / 2.0, 0.0, 0.0),
        (0.0, 0.0, 0.091, 0.0),
    )

    def transform(self, joints_rad: np.ndarray) -> np.ndarray:
        joints = np.asarray(joints_rad, dtype=float)
        if joints.shape != (6,):
            raise ValueError("joints_rad must contain six values")
        transform = np.eye(4)
        for joint, (a, alpha, d, offset) in zip(joints, self.dh_parameters, strict=True):
            transform = transform @ _dh(a, alpha, d, float(joint) + offset)
        return transform

    def forward(self, joints_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        transform = self.transform(joints_rad)
        return transform[:3, 3].copy(), transform[:3, :3].copy()


class PiperSDKForwardKinematics:
    """Adapter around ``C_PiperForwardKinematics`` with local fallback shape."""

    def __init__(self) -> None:
        try:
            from piper_sdk.kinematics import C_PiperForwardKinematics
        except Exception as exc:
            raise RuntimeError("piper_sdk.kinematics.C_PiperForwardKinematics is not available") from exc
        self._sdk_fk: Any = C_PiperForwardKinematics()

    def forward(self, joints_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        joints_deg = np.degrees(np.asarray(joints_rad, dtype=float)).tolist()
        for name in ("CalFK", "cal_fk", "GetFK", "get_fk"):
            method = getattr(self._sdk_fk, name, None)
            if method is None:
                continue
            result = method(joints_deg)
            parsed = _parse_sdk_fk_result(result)
            if parsed is not None:
                return parsed
        raise RuntimeError("C_PiperForwardKinematics did not expose a recognized FK method")


def _parse_sdk_fk_result(result: Any) -> tuple[np.ndarray, np.ndarray] | None:
    if result is None:
        return None
    if isinstance(result, np.ndarray) and result.shape == (4, 4):
        return result[:3, 3].astype(float) / _unit_scale(result[:3, 3]), result[:3, :3].astype(float)
    if isinstance(result, (tuple, list)):
        if len(result) >= 6 and all(np.isscalar(v) for v in result[:6]):
            xyz = np.asarray(result[:3], dtype=float)
            rpy = np.asarray(result[3:6], dtype=float)
            xyz_m = xyz / _unit_scale(xyz)
            rotation = _rpy_matrix(np.radians(rpy / (1000.0 if np.max(np.abs(rpy)) > 360.0 else 1.0)))
            return xyz_m, rotation
        if len(result) > 0:
            return _parse_sdk_fk_result(result[-1])
    for attrs in (("X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis"), ("x", "y", "z", "rx", "ry", "rz")):
        if all(hasattr(result, attr) for attr in attrs):
            return _parse_sdk_fk_result([getattr(result, attr) for attr in attrs])
    return None


def _unit_scale(xyz: np.ndarray) -> float:
    values = np.asarray(xyz, dtype=float)
    # SDK pose values are commonly in 0.001 mm. Accept mm too.
    if np.max(np.abs(values)) > 1000.0:
        return 1_000_000.0
    if np.max(np.abs(values)) > 10.0:
        return 1000.0
    return 1.0


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


def create_official_fk(prefer_sdk: bool = True) -> Any:
    if prefer_sdk:
        try:
            return PiperSDKForwardKinematics()
        except RuntimeError:
            pass
    return PiperOfficialDHForwardKinematics()


def fk_pose_deg(fk: Any, joints_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    xyz, rotation = fk.forward(np.radians(np.asarray(joints_deg, dtype=float)))
    return xyz, _matrix_to_rpy_deg(rotation)
