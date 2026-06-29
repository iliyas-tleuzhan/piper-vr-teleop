"""ADB/logcat Quest transport backed by the oculus_reader API."""

from __future__ import annotations

import importlib
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from piper_vr.buttons import normalize_buttons
from piper_vr.types import QuestSample

OCULUS_LOG_TAG = "wE9ryARX"
OCULUS_PACKAGE = "com.rail.oculus.teleop"


def normalize_side_keys(transforms: dict[str, Any] | None) -> dict[str, np.ndarray]:
    output: dict[str, np.ndarray] = {}
    for key, value in (transforms or {}).items():
        side = {"r": "right", "right": "right", "l": "left", "left": "left"}.get(str(key).lower())
        if side is None:
            continue
        matrix = np.asarray(value, dtype=float)
        if matrix.shape == (4, 4):
            output[side] = matrix
    return output


def parse_oculus_payload(payload: str) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Parse the classic oculus_reader log payload.

    The upstream APK logs a transform section and button section separated by
    ``&``. This parser is intentionally permissive because it is used only for
    diagnostics and tests; the live path calls oculus_reader directly.
    """

    transforms: dict[str, np.ndarray] = {}
    buttons: dict[str, Any] = {}
    transform_part, _, button_part = payload.partition("&")
    for entry in transform_part.replace("\n", "|").split("|"):
        entry = entry.strip()
        if not entry or ":" not in entry:
            continue
        key, raw_values = entry.split(":", 1)
        values = [float(v) for v in raw_values.replace(",", " ").split()]
        if len(values) == 16:
            transforms[key.strip()] = np.asarray(values, dtype=float).reshape(4, 4)
    for entry in button_part.replace("\n", "|").split("|"):
        entry = entry.strip()
        if not entry or ":" not in entry:
            continue
        key, raw_value = entry.split(":", 1)
        key = key.strip()
        items = raw_value.replace(",", " ").split()
        if not items:
            continue
        if len(items) == 1:
            token = items[0]
            if token in ("True", "False"):
                buttons[key] = token == "True"
            else:
                try:
                    buttons[key] = float(token)
                except ValueError:
                    buttons[key] = token
        else:
            buttons[key] = tuple(float(item) for item in items)
    return normalize_side_keys(transforms), normalize_buttons(buttons)


@dataclass
class TransportDiagnostics:
    transport: str
    module_path: str | None = None
    connection: str = "usb"
    ip_address: str | None = None
    has_sample: bool = False
    last_sample_age_s: float | None = None
    package: str = OCULUS_PACKAGE
    log_tag: str = OCULUS_LOG_TAG


class AdbLogcatTransport:
    """Primary Quest transport using oculus_reader over USB or wireless ADB."""

    source = "adb_logcat"

    def __init__(
        self,
        connection: str = "usb",
        ip_address: str | None = None,
        reader: Any | None = None,
        start: bool = False,
        install_apk: bool = False,
        simulate_on_missing: bool = False,
    ) -> None:
        self.connection = connection or ("wireless" if ip_address else "usb")
        self.ip_address = ip_address
        self.install_apk = install_apk
        self.simulate_on_missing = simulate_on_missing
        self.reader = reader
        self.module_path: str | None = None
        self.last_update_s: float | None = None
        self.last_sample: QuestSample | None = None
        if self.reader is not None:
            self.module_path = type(self.reader).__module__
        if start:
            self.start()

    def start(self) -> None:
        if self.reader is None:
            self.reader = self._create_oculus_reader()
        if self.install_apk and hasattr(self.reader, "install"):
            self.reader.install()

    def stop(self) -> None:
        for method_name in ("stop", "shutdown", "close"):
            method = getattr(self.reader, method_name, None)
            if method is not None:
                method()
                return

    def is_ready(self) -> bool:
        return self.get_latest() is not None

    def get_latest(self) -> QuestSample | None:
        if self.reader is None:
            self.start()
        transformations, buttons = self.reader.get_transformations_and_buttons()
        transforms = normalize_side_keys(transformations)
        if transforms:
            now_s = time.monotonic()
            self.last_update_s = now_s
            self.last_sample = QuestSample(
                timestamp_s=now_s,
                source=self.source,
                transforms_openxr=transforms,
                buttons=normalize_buttons(buttons),
                age_s=0.0,
            )
        if self.last_sample is None:
            return None
        age = time.monotonic() - self.last_sample.timestamp_s
        self.last_sample.age_s = age
        return self.last_sample

    def diagnostics(self) -> TransportDiagnostics:
        sample = self.get_latest()
        return TransportDiagnostics(
            transport=self.source,
            module_path=self.module_path,
            connection=self.connection,
            ip_address=self.ip_address,
            has_sample=sample is not None,
            last_sample_age_s=None if sample is None else sample.age_s,
        )

    def _create_oculus_reader(self) -> Any:
        try:
            reader_class, module_path = _import_oculus_reader()
        except ImportError:
            if self.simulate_on_missing:
                self.module_path = "piper_vr.transports.adb_logcat.SimulatedQuestReader"
                print("[DRY-RUN] oculus_reader is not installed; using simulated Quest data.")
                return SimulatedQuestReader()
            raise
        self.module_path = module_path
        kwargs: dict[str, Any] = {"run": True}
        if self.ip_address:
            kwargs["ip_address"] = self.ip_address
        try:
            return reader_class(**kwargs)
        except TypeError:
            kwargs.pop("run", None)
            return reader_class(**kwargs)


def _import_oculus_reader() -> tuple[type[Any], str]:
    errors: list[str] = []
    for module_name in ("oculus_reader.reader", "oculus_reader"):
        try:
            module = importlib.import_module(module_name)
        except ImportError as exc:
            errors.append(f"{module_name}: {exc}")
            continue
        reader_class = getattr(module, "OculusReader", None)
        if reader_class is not None:
            return reader_class, getattr(module, "__file__", module_name) or module_name
        errors.append(f"{module_name}: OculusReader not found")
    raise ImportError(
        "Could not import OculusReader. Install oculus_reader directly or add the "
        "legacy questVR_ws oculus_reader/scripts directory to PYTHONPATH. "
        + " | ".join(errors)
    )


class SimulatedQuestReader:
    """Deterministic no-hardware reader for dry-run and tests."""

    def __init__(self) -> None:
        self.started_s = time.monotonic()

    def get_transformations_and_buttons(self) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        t = time.monotonic() - self.started_s
        right = np.eye(4)
        right[:3, 3] = [0.02 * np.sin(t), 0.02 * np.cos(t), 0.01 * np.sin(t * 0.5)]
        left = np.eye(4)
        left[:3, 3] = [-0.02 * np.sin(t), 0.02 * np.cos(t), 0.01 * np.sin(t * 0.5)]
        buttons = {"A": t < 0.4, "rightGrip": (1.0 if t > 0.8 else 0.0,), "rightTrig": (0.0,)}
        return {"r": right, "l": left}, normalize_buttons(buttons)
