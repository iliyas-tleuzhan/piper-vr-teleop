"""Typed data structures shared by Quest transport and teleop control."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np


@dataclass
class ControllerState:
    """Normalized controller pose and button state for one Quest controller."""

    side: str
    transform_openxr: np.ndarray
    buttons: dict[str, Any] = field(default_factory=dict)

    @property
    def xyz(self) -> np.ndarray:
        return np.asarray(self.transform_openxr, dtype=float)[:3, 3].copy()


@dataclass
class QuestSample:
    """Latest source-native Quest sample.

    Raw transforms are OpenXR-like/source-native matrices. Axis conversion into
    Piper base coordinates is intentionally handled later by vr_mapping.
    """

    timestamp_s: float
    source: str
    transforms_openxr: dict[str, np.ndarray]
    buttons: dict[str, Any]
    age_s: float


class TeleopState(str, Enum):
    WAITING_FOR_DEVICE = "WAITING_FOR_DEVICE"
    WAITING_FOR_CALIBRATION = "WAITING_FOR_CALIBRATION"
    READY_IDLE = "READY_IDLE"
    ACTIVE = "ACTIVE"
    HOLDING = "HOLDING"
    FAULT = "FAULT"


@dataclass
class TeleopConfig:
    can: str = "can0"
    hz: float = 30.0
    speed_percent: int = 5
    side: str = "right"
    deadman_button: str = "rightGrip"
    calibrate_button: str = "A"
    gripper_enabled: bool = False
    scale: float = 0.40
    max_speed_m_s: float = 0.05
    max_position_jump_m: float = 0.03
    stale_timeout_s: float = 0.25
    orientation_enabled: bool = False
    hold_orientation: bool = True
    urdf_guard_enabled: bool = False
    transport: str = "adb_logcat"
    quest_ip: str | None = None
