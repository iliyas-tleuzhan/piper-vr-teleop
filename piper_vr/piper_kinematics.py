"""Small dependency-free constrained IK guard built from Piper's URDF."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


@dataclass(frozen=True)
class Joint:
    name: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


@dataclass(frozen=True)
class IKResult:
    success: bool
    joints_rad: np.ndarray
    position_error_m: float
    orientation_error_deg: float


def _rpy_matrix(rpy_rad: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = np.asarray(rpy_rad, dtype=float)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]],
        dtype=float,
    )


def _axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    return np.array(
        [[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
        [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
        [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]],
        dtype=float,
    )


def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    if angle < 1e-8:
        return np.zeros(3)
    vector = np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]])
    return vector * (angle / (2.0 * np.sin(angle)))


class PiperKinematics:
    """Parse the six-axis Piper chain and solve bounded damped-least-squares IK."""

    def __init__(self, urdf_path: str | Path, base_link: str = "base_link", tip_link: str = "link6") -> None:
        root = ET.parse(urdf_path).getroot()
        joints = []
        for element in root.findall("joint"):
            if element.get("type") not in {"revolute", "continuous"}:
                continue
            parent = element.find("parent")
            child = element.find("child")
            origin = element.find("origin")
            axis = element.find("axis")
            limit = element.find("limit")
            if None in (parent, child, origin, axis, limit):
                continue
            joints.append(Joint(
                name=element.get("name", ""),
                parent=parent.get("link", ""), child=child.get("link", ""),
                origin_xyz=np.fromstring(origin.get("xyz", "0 0 0"), sep=" "),
                origin_rpy=np.fromstring(origin.get("rpy", "0 0 0"), sep=" "),
                axis=np.fromstring(axis.get("xyz", "0 0 1"), sep=" "),
                lower=float(limit.get("lower", "-3.14159265359")),
                upper=float(limit.get("upper", "3.14159265359")),
            ))
        by_parent = {joint.parent: joint for joint in joints}
        chain = []
        link = base_link
        while link != tip_link:
            if link not in by_parent:
                raise ValueError(f"No actuated URDF chain from {base_link!r} to {tip_link!r}")
            joint = by_parent[link]
            chain.append(joint)
            link = joint.child
        if len(chain) != 6:
            raise ValueError(f"Expected six Piper joints, found {len(chain)}")
        self.joints = tuple(chain)
        self.lower = np.array([joint.lower for joint in chain])
        self.upper = np.array([joint.upper for joint in chain])

    def forward(self, joints_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        transform = self.link_transforms(joints_rad)[self.joints[-1].child]
        return transform[:3, 3], transform[:3, :3]

    def link_transforms(self, joints_rad: np.ndarray) -> dict[str, np.ndarray]:
        """Return base-to-link transforms for rendering the URDF visual meshes."""
        transform = np.eye(4)
        transforms = {self.joints[0].parent: transform.copy()}
        for joint, angle in zip(self.joints, np.asarray(joints_rad, dtype=float), strict=True):
            origin = np.eye(4)
            origin[:3, :3] = _rpy_matrix(joint.origin_rpy)
            origin[:3, 3] = joint.origin_xyz
            rotation = np.eye(4)
            rotation[:3, :3] = _axis_angle_matrix(joint.axis, float(angle))
            transform = transform @ origin @ rotation
            transforms[joint.child] = transform.copy()
        return transforms

    def solve(
        self,
        target_xyz_m: np.ndarray,
        target_rpy_deg: np.ndarray,
        seed_rad: np.ndarray | None = None,
        *,
        max_iterations: int = 80,
        damping: float = 0.08,
        position_tolerance_m: float = 0.004,
        orientation_tolerance_deg: float = 6.0,
    ) -> IKResult:
        target_xyz_m = np.asarray(target_xyz_m, dtype=float)
        target_rotation = _rpy_matrix(np.radians(target_rpy_deg))
        q = np.clip((self.lower + self.upper) / 2 if seed_rad is None else seed_rad, self.lower, self.upper).astype(float)
        epsilon = 1e-5

        def error(values: np.ndarray) -> np.ndarray:
            position, rotation = self.forward(values)
            # Position is in base coordinates; angular error is in the current tool frame.
            return np.concatenate((target_xyz_m - position, _rotation_vector(rotation.T @ target_rotation)))

        for _ in range(max_iterations):
            value = error(q)
            if np.linalg.norm(value[:3]) <= position_tolerance_m and np.degrees(np.linalg.norm(value[3:])) <= orientation_tolerance_deg:
                break
            jacobian = np.empty((6, 6))
            for index in range(6):
                plus = q.copy(); plus[index] = min(self.upper[index], plus[index] + epsilon)
                minus = q.copy(); minus[index] = max(self.lower[index], minus[index] - epsilon)
                difference = plus[index] - minus[index]
                jacobian[:, index] = (error(minus) - error(plus)) / difference
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping * damping * np.eye(6), value)
            q = np.clip(q + np.clip(step, -0.12, 0.12), self.lower, self.upper)

        final_error = error(q)
        position_error = float(np.linalg.norm(final_error[:3]))
        orientation_error = float(np.degrees(np.linalg.norm(final_error[3:])))
        return IKResult(position_error <= position_tolerance_m and orientation_error <= orientation_tolerance_deg, q, position_error, orientation_error)
