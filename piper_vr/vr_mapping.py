"""Map Quest controller motion into Piper Cartesian target motion."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


AXIS_INDEX = {"vr_x": 0, "vr_y": 1, "vr_z": 2}


@dataclass(frozen=True)
class AxisMapping:
    piper_x: str = "+vr_x"
    piper_y: str = "+vr_y"
    piper_z: str = "+vr_z"

    @classmethod
    def from_config(cls, config: dict | None) -> "AxisMapping":
        config = config or {}
        return cls(
            piper_x=config.get("piper_x", "+vr_x"),
            piper_y=config.get("piper_y", "+vr_y"),
            piper_z=config.get("piper_z", "+vr_z"),
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


def target_from_home(
    vr_home_transform: np.ndarray,
    vr_current_transform: np.ndarray,
    piper_home_xyz_m: np.ndarray,
    mapping: AxisMapping,
    scale: float,
) -> np.ndarray:
    vr_delta_m = controller_translation(vr_current_transform) - controller_translation(vr_home_transform)
    piper_delta_m = mapping.apply(vr_delta_m) * float(scale)
    return np.asarray(piper_home_xyz_m, dtype=float) + piper_delta_m
