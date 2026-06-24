"""Quest button helpers."""

from __future__ import annotations

from typing import Any


DEFAULT_BUTTONS = {
    "A": False,
    "B": False,
    "X": False,
    "Y": False,
    "RG": False,
    "LG": False,
    "RTr": False,
    "LTr": False,
    "RThU": False,
    "LThU": False,
    "RJ": False,
    "LJ": False,
    "rightGrip": (0.0,),
    "leftGrip": (0.0,),
    "rightTrig": (0.0,),
    "leftTrig": (0.0,),
    "rightJS": (0.0, 0.0),
    "leftJS": (0.0, 0.0),
}


def normalize_buttons(buttons: dict[str, Any] | None) -> dict[str, Any]:
    normalized = dict(DEFAULT_BUTTONS)
    if buttons:
        normalized.update(buttons)
    return normalized


def analog_value(buttons: dict[str, Any], key: str) -> float:
    value = buttons.get(key, (0.0,))
    if isinstance(value, (tuple, list)) and value:
        return float(value[0])
    if isinstance(value, (int, float, bool)):
        return float(value)
    return 0.0


def is_pressed(buttons: dict[str, Any], key: str, threshold: float = 0.5) -> bool:
    if key in ("rightGrip", "leftGrip", "rightTrig", "leftTrig"):
        return analog_value(buttons, key) >= threshold
    return bool(buttons.get(key, False))
