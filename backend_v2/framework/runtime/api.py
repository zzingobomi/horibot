from __future__ import annotations

from typing import Awaitable, Protocol, TypeVar

from pydantic import BaseModel

TRes = TypeVar("TRes", bound=BaseModel)


class ModuleRuntime(Protocol):
    def publish(self, wire_key: str, event: BaseModel) -> None:
        ...

    def call(
        self,
        key: str,
        req: BaseModel,
        res_cls: type[TRes],
        *,
        robot_id: str | None = None,
        timeout: float = 5.0,
    ) -> Awaitable[TRes]:
        ...
