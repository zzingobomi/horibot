from __future__ import annotations

from typing import Protocol, runtime_checkable


@runtime_checkable
class ObjectStore(Protocol):
    def put(self, key: str, data: bytes) -> None:
        ...

    def get(self, key: str) -> bytes:
        ...

    def delete(self, key: str) -> None:
        ...

    def list(self, prefix: str) -> list[str]:
        ...
