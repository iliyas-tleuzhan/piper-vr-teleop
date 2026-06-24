"""Safety filters for Cartesian endpoint commands."""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np


@dataclass
class SafetyLimiter:
    workspace_min_m: np.ndarray
    workspace_max_m: np.ndarray
    max_speed_m_s: float
    stale_timeout_s: float = 0.5
    last_command_m: np.ndarray | None = None
    last_time_s: float | None = None

    @classmethod
    def from_config(cls, config: dict) -> "SafetyLimiter":
        return cls(
            workspace_min_m=np.asarray(config["workspace_min_m"], dtype=float),
            workspace_max_m=np.asarray(config["workspace_max_m"], dtype=float),
            max_speed_m_s=float(config["max_speed_m_s"]),
            stale_timeout_s=float(config.get("stale_timeout_s", 0.5)),
        )

    def clamp_workspace(self, xyz_m: np.ndarray) -> np.ndarray:
        return np.minimum(np.maximum(np.asarray(xyz_m, dtype=float), self.workspace_min_m), self.workspace_max_m)

    def limit_step(self, target_m: np.ndarray, now_s: float | None = None) -> np.ndarray:
        now_s = time.monotonic() if now_s is None else now_s
        target_m = self.clamp_workspace(target_m)
        if self.last_command_m is None:
            self.last_command_m = target_m
            self.last_time_s = now_s
            return target_m

        dt = max(now_s - (self.last_time_s or now_s), 1e-3)
        max_step = max(self.max_speed_m_s * dt, 0.0)
        delta = target_m - self.last_command_m
        distance = float(np.linalg.norm(delta))
        if distance > max_step > 0.0:
            target_m = self.last_command_m + delta / distance * max_step

        self.last_command_m = self.clamp_workspace(target_m)
        self.last_time_s = now_s
        return self.last_command_m

    def hold(self) -> np.ndarray | None:
        return None if self.last_command_m is None else self.last_command_m.copy()


def tracking_is_stale(last_update_s: float | None, timeout_s: float) -> bool:
    if last_update_s is None:
        return True
    return (time.monotonic() - last_update_s) > timeout_s
