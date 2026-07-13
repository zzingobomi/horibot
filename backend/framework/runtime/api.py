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
        timeout: float | None = None,
    ) -> Awaitable[TRes]:
        """timeout=None → contract 선언 기본값 (declare_service_timeouts),
        선언도 없으면 DEFAULT_SERVICE_TIMEOUT_S."""
        ...
