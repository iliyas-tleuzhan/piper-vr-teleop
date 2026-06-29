"""High-level Quest reader facade over pluggable transports."""

from __future__ import annotations

from typing import Any

import numpy as np

from .buttons import normalize_buttons
from .transports.adb_logcat import SimulatedQuestReader
from .transports.factory import create_transport
from .types import QuestSample


class QuestReader:
    """Small compatibility wrapper used by scripts and teleop loops."""

    def __init__(
        self,
        transport: str = "adb_logcat",
        connection: str = "usb",
        ip_address: str | None = None,
        reader: Any | None = None,
        start: bool = True,
        install_apk: bool = False,
        simulate_on_missing: bool = False,
    ) -> None:
        self.transport_name = transport
        self.transport = create_transport(
            transport,
            connection=connection,
            ip_address=ip_address,
            reader=reader,
            start=False,
            install_apk=install_apk,
            simulate_on_missing=simulate_on_missing,
        )
        if start:
            self.start()

    @property
    def last_update_s(self) -> float | None:
        return getattr(self.transport, "last_update_s", None)

    def start(self) -> None:
        self.transport.start()

    def stop(self) -> None:
        self.transport.stop()

    def is_ready(self) -> bool:
        return self.transport.is_ready()

    def get_sample(self) -> QuestSample | None:
        return self.transport.get_latest()

    def poll(self) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        sample = self.get_sample()
        if sample is None:
            return {}, normalize_buttons(None)
        return sample.transforms_openxr, normalize_buttons(sample.buttons)

    def get_controller_pose(self, side: str) -> np.ndarray:
        sample = self.get_sample()
        if sample is None:
            raise RuntimeError("No Quest sample is available")
        side = side.lower()
        if side not in sample.transforms_openxr:
            raise RuntimeError(f"No {side} controller transform is available")
        return np.asarray(sample.transforms_openxr[side], dtype=float)

    def get_buttons(self) -> dict[str, Any]:
        sample = self.get_sample()
        return normalize_buttons(None if sample is None else sample.buttons)

    def diagnostics(self) -> Any:
        method = getattr(self.transport, "diagnostics", None)
        return None if method is None else method()


__all__ = ["QuestReader", "SimulatedQuestReader"]
