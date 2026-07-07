from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class ParsedCommand:
    pick: str
    place: str | None = None


class LlmBackend(Protocol):
    def parse(self, text: str) -> ParsedCommand | None: ...

    def preload(self) -> None: ...
