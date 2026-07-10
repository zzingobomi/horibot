from __future__ import annotations

from typing import Protocol, TypeVar

T = TypeVar("T")


class Repository(Protocol[T]):
    def get(self, id: int) -> T | None: ...

    def save(self, entity: T) -> None: ...

    def delete(self, id: int) -> None: ...
