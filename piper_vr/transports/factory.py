"""Factory for Quest input transports."""

from __future__ import annotations

from typing import Any

from .adb_logcat import AdbLogcatTransport
from .base import QuestTransport
from .ros_topics import RosTopicsTransport


def create_transport(name: str = "adb_logcat", **kwargs: Any) -> QuestTransport:
    normalized = (name or "adb_logcat").lower().replace("-", "_")
    if normalized in ("adb", "adb_logcat", "oculus_reader"):
        return AdbLogcatTransport(**kwargs)
    if normalized in ("ros", "ros_topics", "ros2"):
        return RosTopicsTransport(**kwargs)
    raise ValueError(f"Unknown Quest transport: {name!r}")
