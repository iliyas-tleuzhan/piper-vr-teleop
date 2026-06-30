"""Configuration loading, deep-merge, and runtime profile helpers."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


def load_config(path: str | Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def deep_merge(base: dict[str, Any], patch: dict[str, Any]) -> dict[str, Any]:
    """Return a recursive merge where patch values override base values."""
    merged = deepcopy(base)
    for key, value in (patch or {}).items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = deepcopy(value)
    return merged


def apply_profile(config: dict[str, Any], profile: str | None) -> dict[str, Any]:
    if not profile:
        return config
    profiles = config.get("profiles", {})
    if profile not in profiles:
        available = ", ".join(sorted(profiles)) or "none"
        raise ValueError(f"Unknown profile {profile!r}; available profiles: {available}")
    selected = profiles[profile] or {}
    updated = deep_merge(config, selected)
    if "max_joint_speed_deg_s" in selected:
        updated.setdefault("joint_mimic", {})["max_joint_speed_deg_s"] = selected["max_joint_speed_deg_s"]
    updated["active_profile"] = profile
    return updated
