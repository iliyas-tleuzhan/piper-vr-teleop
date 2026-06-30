"""Best-effort UDP broadcaster for the Quest Piper visualization app."""

from __future__ import annotations

import json
import socket
import time
from typing import Any

import numpy as np


def _six_or_none(value: Any) -> list[float] | None:
    if value is None:
        return None
    try:
        array = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    if array.size != 6 or not np.all(np.isfinite(array)):
        return None
    return [float(x) for x in array]


def _vector_or_none(value: Any, length: int) -> list[float] | None:
    if value is None:
        return None
    try:
        array = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    if array.size != length or not np.all(np.isfinite(array)):
        return None
    return [float(x) for x in array]


def _state_value(value: Any) -> str:
    return str(getattr(value, "value", value))


class QuestVizBroadcaster:
    """Send teleop loop state to a passive Quest visualization client.

    The broadcaster intentionally never raises during normal sends. Visualization
    must not become part of the robot safety or control path.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 5055, enabled: bool = False) -> None:
        self.host = str(host)
        self.port = int(port)
        self.enabled = bool(enabled)
        self._socket: socket.socket | None = None
        if self.enabled:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def build_packet(self, result: Any, driver: Any = None, *, mode: str | None = None, mapping_mode: str | None = None) -> dict[str, Any]:
        measured = getattr(result, "measured_joints", None)
        measured_deg = getattr(measured, "joints_deg", None)
        if measured_deg is None and driver is not None:
            last_joint_command = getattr(driver, "last_joint_command", None)
            measured_deg = getattr(last_joint_command, "joints_deg", None)

        commanded = getattr(result, "safe_joint_target_deg", None)
        if commanded is None:
            commanded = getattr(result, "raw_joint_target_deg", None)
        if commanded is None and driver is not None:
            last_joint_command = getattr(driver, "last_joint_command", None)
            commanded = getattr(last_joint_command, "joints_deg", None)

        return {
            "type": "piper_joint_state",
            "timestamp": time.time(),
            "commanded_joints_deg": _six_or_none(commanded),
            "measured_joints_deg": _six_or_none(measured_deg),
            "controller_xyz": _vector_or_none(getattr(result, "controller_xyz", None), 3),
            "state": _state_value(getattr(result, "state", "UNKNOWN")),
            "reason": str(getattr(result, "reason", "")),
            "mode": mode,
            "mapping_mode": mapping_mode,
            "sample_age_s": None if getattr(result, "sample_age_s", None) is None else float(result.sample_age_s),
            "action": str(getattr(result, "action", "")),
            "calibrated": bool(getattr(result, "calibrated", False)),
            "deadman": bool(getattr(result, "deadman", False)),
        }

    def send(self, result: Any, driver: Any = None, *, mode: str | None = None, mapping_mode: str | None = None) -> bool:
        if not self.enabled:
            return False
        if self._socket is None:
            return False
        try:
            packet = self.build_packet(result, driver, mode=mode, mapping_mode=mapping_mode)
            payload = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode("utf-8")
            self._socket.sendto(payload, (self.host, self.port))
            return True
        except OSError:
            return False
        except (TypeError, ValueError):
            return False

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None
