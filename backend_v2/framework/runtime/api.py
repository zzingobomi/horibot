"""ModuleRuntime Protocol — Module 이 Framework 에 요청하는 통신 surface (§3.7).

두 원칙 정합 (§3.0):
- explicit at every use site — `publish(wire_key, event)` / `call(target, req)` 자세 wire_key 직접
- typed — wire_key 자세 StrEnum value / method reference

Module 자세 import boundary — `import zenoh` 자세 안 박힘. Transport object 본 적 없음.
constructor 자세 `runtime: ModuleRuntime` 인자 자세 받음 (DIP).
"""

from __future__ import annotations

from typing import Awaitable, Callable, Protocol, TypeVar

from pydantic import BaseModel

TRes = TypeVar("TRes", bound=BaseModel)


class ModuleRuntime(Protocol):
    """Module 이 Framework 에 요청하는 통신 surface (§3.7)."""

    def publish(self, wire_key: str, event: BaseModel) -> None:
        """event publish 자세.

        wire_key 자세 첫 인자 (explicit + typed, StrEnum value 추천).
        event 자세 Pydantic instance (msgpack encode 자세 transport 보냄).

        `{robot_id}` placeholder 자세 event payload 의 `robot_id` field 자세
        framework 가 substitute (§3.7 의 placeholder 자세).
        """
        ...

    def call(
        self,
        target: Callable[..., TRes],
        req: BaseModel,
        *,
        robot_id: str | None = None,
        timeout: float = 5.0,
    ) -> Awaitable[TRes]:
        """service 호출 자세 — generic on target 의 return type.

        target 자세 method reference — `ModuleClass.method` (typed, @service 박힌 method).
        framework 자세 method 의 @service spec 자세 wire_key + req_cls + res_cls 자세 lookup.
        return type 자세 target 의 return annotation 자세 (caller 자세 narrow).

        robot-scoped target 자세 = wire_key 의 `{robot_id}` placeholder 자세 — caller 자세
        `robot_id=` 인자 자세 substitute. robot-agnostic 자세 robot_id 자세 None OK.
        """
        ...
