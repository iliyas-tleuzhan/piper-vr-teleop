"""Piper forward kinematics helpers.

The real AgileX SDK exposes ``piper_sdk.kinematics.C_PiperForwardKinematics``.
This module prefers that implementation when installed and provides an
approximate local fallback so endpoint IK tests and dry-run tools do not depend
on the URDF submodule. The fallback is intentionally not advertised as
official-SDK equivalent.
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
class PiperApproxForwardKinematics:
    """Approximate local FK fallback using Piper-sized DH geometry in meters.

    The constants intentionally live in meters internally. They are close to the
    public Piper geometry and provide stable host-side FK/IK when the SDK FK
    object is not importable. They are not guaranteed to match official SDK FK.
    """

    backend_name: str = "approx_fk"

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
        self.backend_name = "sdk_fk"

    def forward(self, joints_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        joints_deg = np.degrees(np.asarray(joints_rad, dtype=float)).tolist()
        for name in ("CalFK", "cal_fk", "GetFK", "get_fk"):
            method = getattr(self._sdk_fk, name, None)
            if method is None:
                continue
            for args, kwargs in (((joints_deg,), {}), (tuple(joints_deg), {})):
                try:
                    result = method(*args, **kwargs)
                except TypeError:
                    continue
                parsed = _parse_sdk_fk_result(result)
                if parsed is not None:
                    return parsed
        raise RuntimeError("C_PiperForwardKinematics did not expose a recognized FK method")


def _parse_sdk_fk_result(result: Any) -> tuple[np.ndarray, np.ndarray] | None:
    if result is None:
        return None
    if isinstance(result, np.ndarray) and result.shape == (4, 4):
        return parse_xyz_to_meters(result[:3, 3]), result[:3, :3].astype(float)
    if isinstance(result, (tuple, list)):
        if len(result) >= 6 and all(np.isscalar(v) for v in result[:6]):
            xyz_m = parse_xyz_to_meters(result[:3])
            rotation = _rpy_matrix(np.radians(parse_rpy_to_degrees(result[3:6])))
            return xyz_m, rotation
        if len(result) > 0:
            return _parse_sdk_fk_result(result[-1])
    for attr in ("end_pose", "arm_end_pose", "pose", "fk", "forward_kinematics"):
        nested = getattr(result, attr, None)
        if nested is not None:
            parsed = _parse_sdk_fk_result(nested)
            if parsed is not None:
                return parsed
    for attrs in (("X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis"), ("x", "y", "z", "rx", "ry", "rz")):
        if all(hasattr(result, attr) for attr in attrs):
            return _parse_sdk_fk_result([getattr(result, attr) for attr in attrs])
    return None


def parse_xyz_to_meters(xyz: Any) -> np.ndarray:
    """Parse SDK FK XYZ values into meters.

    Piper endpoint APIs commonly expose 0.001 mm integer units. Some helpers
    return millimeters and some wrappers return meters. The thresholds below
    mirror those unit families explicitly.
    """
    values = np.asarray(xyz, dtype=float)
    return values / _xyz_unit_scale(values)


def parse_rpy_to_degrees(rpy: Any) -> np.ndarray:
    """Parse SDK FK RPY values into degrees.

    Values above 360 are treated as 0.001 degree units, matching Piper endpoint
    command/feedback conventions. Otherwise values are already degrees.
    """
    values = np.asarray(rpy, dtype=float)
    if values.size and np.max(np.abs(values)) > 360.0:
        return values / 1000.0
    return values


def _xyz_unit_scale(xyz: np.ndarray) -> float:
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
    print("WARNING: using approximate Piper FK fallback; install piper_sdk official FK or use firmware_endpoint for real robot.")
    return PiperApproxForwardKinematics()


def fk_pose_deg(fk: Any, joints_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    xyz, rotation = fk.forward(np.radians(np.asarray(joints_deg, dtype=float)))
    return xyz, _matrix_to_rpy_deg(rotation)


# Backward-compatible alias for older imports. New code should use
# PiperApproxForwardKinematics to avoid implying SDK equivalence.
PiperOfficialDHForwardKinematics = PiperApproxForwardKinematics
