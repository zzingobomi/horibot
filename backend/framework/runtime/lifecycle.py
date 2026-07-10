from __future__ import annotations

from typing import Any, Awaitable, Protocol, runtime_checkable


@runtime_checkable
class Lifecycle(Protocol):
    def start(self) -> None | Awaitable[None]:
        ...

    def stop(self) -> None | Awaitable[None]:
        ...


def has_start(mod: Any) -> bool:
    return callable(getattr(mod, "start", None))


def has_stop(mod: Any) -> bool:
    return callable(getattr(mod, "stop", None))
