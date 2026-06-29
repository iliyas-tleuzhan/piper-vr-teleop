"""Transport protocol for Quest controller input."""

from __future__ import annotations

from typing import Protocol

from piper_vr.types import QuestSample


class QuestTransport(Protocol):
    def start(self) -> None: ...

    def stop(self) -> None: ...

    def is_ready(self) -> bool: ...

    def get_latest(self) -> QuestSample | None: ...
