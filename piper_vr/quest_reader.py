"""Quest reader wrapper compatible with the upstream oculus_reader approach."""

from __future__ import annotations

import importlib
import time
from typing import Any

import numpy as np

from .buttons import normalize_buttons


class QuestReader:
    """Small wrapper around an object exposing get_transformations_and_buttons()."""

    SIDE_KEYS = {"right": ("r", "right"), "left": ("l", "left")}

    def __init__(
        self,
        ip_address: str | None = None,
        reader: Any | None = None,
        start: bool = True,
        install_apk: bool = False,
        simulate_on_missing: bool = False,
    ) -> None:
        self._reader = reader or self._create_oculus_reader(ip_address, start, install_apk, simulate_on_missing)
        self._last_update_s: float | None = None
        self._last_transforms: dict[str, np.ndarray] = {}
        self._last_buttons: dict[str, Any] = normalize_buttons(None)

    @property
    def last_update_s(self) -> float | None:
        return self._last_update_s

    def _create_oculus_reader(
        self,
        ip_address: str | None,
        start: bool,
        install_apk: bool,
        simulate_on_missing: bool,
    ) -> Any:
        try:
            module = importlib.import_module("oculus_reader")
        except ImportError:
            if simulate_on_missing:
                print("[DRY-RUN] oculus_reader is not installed; using simulated Quest data.")
                return SimulatedQuestReader()
            raise
        reader_class = getattr(module, "OculusReader")
        reader = reader_class(ip_address=ip_address, run=start)
        if install_apk and hasattr(reader, "install"):
            reader.install()
        return reader

    def poll(self) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        transformations, buttons = self._reader.get_transformations_and_buttons()
        if transformations:
            self._last_transforms = dict(transformations)
            self._last_buttons = normalize_buttons(buttons)
            self._last_update_s = time.monotonic()
        return self._last_transforms, self._last_buttons

    def get_controller_pose(self, side: str) -> np.ndarray:
        transformations, _ = self.poll()
        for key in self.SIDE_KEYS[side.lower()]:
            if key in transformations:
                return np.asarray(transformations[key], dtype=float)
        raise RuntimeError(f"No {side} controller transform is available")

    def get_buttons(self) -> dict[str, Any]:
        _, buttons = self.poll()
        return normalize_buttons(buttons)

    def is_ready(self) -> bool:
        transformations, _ = self.poll()
        return bool(transformations)


class SimulatedQuestReader:
    """Small deterministic reader used only for no-hardware dry-run checks."""

    def __init__(self) -> None:
        self.started_s = time.monotonic()

    def get_transformations_and_buttons(self) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        t = time.monotonic() - self.started_s
        right = np.eye(4)
        right[:3, 3] = [0.02 * np.sin(t), 0.02 * np.cos(t), 0.01 * np.sin(t * 0.5)]
        left = np.eye(4)
        left[:3, 3] = [-0.02 * np.sin(t), 0.02 * np.cos(t), 0.01 * np.sin(t * 0.5)]
        buttons = normalize_buttons({"A": t < 1.0, "B": True, "rightGrip": (1.0,)})
        return {"r": right, "l": left}, buttons
