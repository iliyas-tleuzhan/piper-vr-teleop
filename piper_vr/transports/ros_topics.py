"""Optional ROS topic Quest transport placeholder.

ADB/logcat is the maintained first-working transport. This module exists so
ROS-based Quest apps can be added later without changing teleop control code.
"""

from __future__ import annotations

from piper_vr.types import QuestSample


class RosTopicsTransport:
    def __init__(self, *_: object, **__: object) -> None:
        self.error = "ROS topic transport is not implemented in this first-working path."

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def is_ready(self) -> bool:
        return False

    def get_latest(self) -> QuestSample | None:
        return None
