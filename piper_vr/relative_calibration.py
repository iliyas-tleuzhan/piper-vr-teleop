"""Helpers for measuring Quest controller axes and generating relative gains."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

CHANNEL_NAMES = ("dx", "dy", "dz")
ROTATION_CHANNEL_NAMES = ("droll", "dpitch", "dyaw")
MOVEMENTS = ("up", "down", "left", "right", "forward", "backward")
ROTATION_MOVEMENTS = ("roll_clockwise", "roll_counterclockwise", "pitch_up", "pitch_down", "yaw_left", "yaw_right")


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


def build_observation(
    movement: str,
    delta_xyz: list[float] | np.ndarray,
    delta_rot_deg: list[float] | np.ndarray | None = None,
) -> dict[str, Any]:
    index, channel, sign, value = dominant_channel(np.asarray(delta_xyz, dtype=float))
    row = {
        "movement": movement.lower(),
        "delta_xyz": np.asarray(delta_xyz, dtype=float).round(6).tolist(),
        "dominant_channel": channel,
        "dominant_index": index,
        "dominant_value": round(value, 6),
        "sign": sign,
    }
    if delta_rot_deg is not None:
        rot_index, rot_channel, rot_sign, rot_value = dominant_rotation_channel(np.asarray(delta_rot_deg, dtype=float))
        row.update(
            {
                "delta_rot_deg": np.asarray(delta_rot_deg, dtype=float).round(6).tolist(),
                "dominant_rotation_channel": rot_channel,
                "dominant_rotation_index": rot_index,
                "dominant_rotation_value": round(rot_value, 6),
                "rotation_sign": rot_sign,
            }
        )
    return row


def dominant_rotation_channel(delta_rot_deg: np.ndarray) -> tuple[int, str, str, float]:
    values = np.asarray(delta_rot_deg, dtype=float)
    if values.shape != (3,):
        raise ValueError("delta_rot_deg must have shape (3,)")
    index = int(np.argmax(np.abs(values)))
    value = float(values[index])
    sign = "positive" if value >= 0.0 else "negative"
    return index, ROTATION_CHANNEL_NAMES[index], sign, value


def validate_axis_calibration(data: dict[str, Any]) -> dict[str, dict[str, Any]]:
    records = data.get("movements", [])
    by_name = {str(row.get("movement", "")).lower(): row for row in records}
    missing = [name for name in MOVEMENTS if name not in by_name]
    if missing:
        raise ValueError(f"Axis calibration is missing movements: {', '.join(missing)}")
    return by_name


def validate_rotation_calibration(data: dict[str, Any]) -> dict[str, dict[str, Any]]:
    records = data.get("movements", [])
    by_name = {str(row.get("movement", "")).lower(): row for row in records}
    missing = [name for name in ROTATION_MOVEMENTS if name not in by_name]
    if missing:
        raise ValueError(f"Axis calibration is missing rotation movements: {', '.join(missing)}")
    return by_name


def _dominant_index_and_sign(record: dict[str, Any]) -> tuple[int, float]:
    if "dominant_index" in record:
        index = int(record["dominant_index"])
    else:
        index = dominant_channel(np.asarray(record["delta_xyz"], dtype=float))[0]
    value = float(np.asarray(record["delta_xyz"], dtype=float)[index])
    return index, 1.0 if value >= 0.0 else -1.0


def _dominant_rotation_index_and_sign(record: dict[str, Any]) -> tuple[int, float]:
    if "dominant_rotation_index" in record:
        index = int(record["dominant_rotation_index"])
    else:
        index = dominant_rotation_channel(np.asarray(record["delta_rot_deg"], dtype=float))[0]
    value = float(np.asarray(record["delta_rot_deg"], dtype=float)[index])
    return index, 1.0 if value >= 0.0 else -1.0


def generate_relative_gain_matrix(
    data: dict[str, Any],
    *,
    translation_gain: float = 300.0,
    reach_gain: float = 250.0,
    wrist_gain: float = 0.6,
) -> list[list[float]]:
    """Generate a 6x6 matrix from physical movement observations.

    Desired signs:
    - physical RIGHT increases joint 1 yaw
    - physical UP increases joint 2 and decreases joint 3, flipping the prior bad vertical behavior
    - physical FORWARD increases both joint 2 and joint 3 for reach
    """
    records = validate_axis_calibration(data)
    matrix = np.zeros((6, 6), dtype=float)

    right_col, right_sign = _dominant_index_and_sign(records["right"])
    matrix[0, right_col] = float(translation_gain) / right_sign

    up_col, up_sign = _dominant_index_and_sign(records["up"])
    matrix[1, up_col] = float(reach_gain) / up_sign
    matrix[2, up_col] = -float(reach_gain) / up_sign

    forward_col, forward_sign = _dominant_index_and_sign(records["forward"])
    matrix[1, forward_col] += float(reach_gain) / forward_sign
    matrix[2, forward_col] += float(reach_gain) / forward_sign

    rotation_records = validate_rotation_calibration(data)
    for movement, row_index in (("roll_clockwise", 3), ("pitch_up", 4), ("yaw_right", 5)):
        rot_col, rot_sign = _dominant_rotation_index_and_sign(rotation_records[movement])
        matrix[row_index, 3 + rot_col] = float(wrist_gain) / rot_sign

    return matrix.round(6).tolist()


def generated_mapping_config(
    data: dict[str, Any],
    *,
    translation_gain: float = 300.0,
    reach_gain: float = 250.0,
    wrist_gain: float = 0.6,
) -> dict[str, Any]:
    return {
        "joint_mimic": {
            "relative_gain_matrix": generate_relative_gain_matrix(
                data,
                translation_gain=translation_gain,
                reach_gain=reach_gain,
                wrist_gain=wrist_gain,
            ),
            "max_joint_speed_deg_s": [90, 90, 90, 90, 90, 90],
            "translation_deadband_m": 0.0015,
            "rotation_deadband_deg": 2.0,
            "smoothing_alpha": 0.65,
            "wrist_rotation_enabled": True,
            "wrist_rotation_deadman": None,
            "wrist_rotation_gain": float(wrist_gain),
            "wrist_rotation_deadband_deg": 1.5,
            "max_wrist_speed_deg_s": [45, 45, 45],
            "wrist_rotation_filter_alpha": 0.5,
        }
    }
