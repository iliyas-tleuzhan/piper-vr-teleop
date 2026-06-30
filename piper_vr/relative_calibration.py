"""Helpers for measuring Quest controller axes and generating relative gains."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

CHANNEL_NAMES = ("dx", "dy", "dz")
MOVEMENTS = ("up", "down", "left", "right", "forward", "backward")


@dataclass(frozen=True)
class AxisObservation:
    movement: str
    delta_xyz: np.ndarray

    @property
    def dominant_index(self) -> int:
        return dominant_channel(self.delta_xyz)[0]

    @property
    def dominant_channel(self) -> str:
        return dominant_channel(self.delta_xyz)[1]

    @property
    def sign(self) -> str:
        return dominant_channel(self.delta_xyz)[2]


def dominant_channel(delta_xyz: np.ndarray) -> tuple[int, str, str, float]:
    values = np.asarray(delta_xyz, dtype=float)
    if values.shape != (3,):
        raise ValueError("delta_xyz must have shape (3,)")
    index = int(np.argmax(np.abs(values)))
    value = float(values[index])
    sign = "positive" if value >= 0.0 else "negative"
    return index, CHANNEL_NAMES[index], sign, value


def build_observation(movement: str, delta_xyz: list[float] | np.ndarray) -> dict[str, Any]:
    index, channel, sign, value = dominant_channel(np.asarray(delta_xyz, dtype=float))
    return {
        "movement": movement.lower(),
        "delta_xyz": np.asarray(delta_xyz, dtype=float).round(6).tolist(),
        "dominant_channel": channel,
        "dominant_index": index,
        "dominant_value": round(value, 6),
        "sign": sign,
    }


def validate_axis_calibration(data: dict[str, Any]) -> dict[str, dict[str, Any]]:
    records = data.get("movements", [])
    by_name = {str(row.get("movement", "")).lower(): row for row in records}
    missing = [name for name in MOVEMENTS if name not in by_name]
    if missing:
        raise ValueError(f"Axis calibration is missing movements: {', '.join(missing)}")
    return by_name


def _dominant_index_and_sign(record: dict[str, Any]) -> tuple[int, float]:
    if "dominant_index" in record:
        index = int(record["dominant_index"])
    else:
        index = dominant_channel(np.asarray(record["delta_xyz"], dtype=float))[0]
    value = float(np.asarray(record["delta_xyz"], dtype=float)[index])
    return index, 1.0 if value >= 0.0 else -1.0


def generate_relative_gain_matrix(data: dict[str, Any]) -> list[list[float]]:
    """Generate a 6x6 matrix from physical movement observations.

    Desired signs:
    - physical RIGHT increases joint 1 yaw
    - physical UP increases joint 2 and decreases joint 3, flipping the prior bad vertical behavior
    - physical FORWARD increases both joint 2 and joint 3 for reach
    """
    records = validate_axis_calibration(data)
    matrix = np.zeros((6, 6), dtype=float)

    right_col, right_sign = _dominant_index_and_sign(records["right"])
    matrix[0, right_col] = 300.0 / right_sign

    up_col, up_sign = _dominant_index_and_sign(records["up"])
    matrix[1, up_col] = 275.0 / up_sign
    matrix[2, up_col] = -275.0 / up_sign

    forward_col, forward_sign = _dominant_index_and_sign(records["forward"])
    matrix[1, forward_col] += 250.0 / forward_sign
    matrix[2, forward_col] += 250.0 / forward_sign

    return matrix.round(6).tolist()


def generated_mapping_config(data: dict[str, Any]) -> dict[str, Any]:
    return {
        "joint_mimic": {
            "relative_gain_matrix": generate_relative_gain_matrix(data),
            "max_joint_speed_deg_s": [90, 90, 90, 90, 90, 90],
            "translation_deadband_m": 0.0015,
            "rotation_deadband_deg": 2.0,
            "smoothing_alpha": 0.65,
        }
    }
