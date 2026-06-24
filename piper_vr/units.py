"""Unit conversions for Piper endpoint commands."""

from __future__ import annotations


def meters_to_piper_xyz(value_m: float) -> int:
    """Convert meters to Piper XYZ command units, where 1 unit is 0.001 mm."""
    return int(round(value_m * 1_000_000.0))


def piper_xyz_to_meters(value: float) -> float:
    """Convert Piper XYZ command units to meters."""
    return float(value) / 1_000_000.0


def degrees_to_piper_rpy(value_deg: float) -> int:
    """Convert degrees to Piper RPY command units, where 1 unit is 0.001 degrees."""
    return int(round(value_deg * 1_000.0))


def piper_rpy_to_degrees(value: float) -> float:
    """Convert Piper RPY command units to degrees."""
    return float(value) / 1_000.0


def meters_to_gripper_units(value_m: float) -> int:
    """Convert a gripper opening in meters to Piper gripper units."""
    return int(round(value_m * 1_000_000.0))
