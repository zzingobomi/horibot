"""publisher / event encoding 자세.

두 원칙 (spec §3.0):
- explicit at every use site — publisher (`runtime.publish(wire_key, event)`) /
  subscriber (`@subscriber(wire_key)`) / Mirror (`change_event_topic=`) 모두 wire_key 직접 박음
- typed — `StrEnum` value 박음 (raw str / class attribute lookup 자세 X)

event class 자세 *pure Pydantic data* — `__wire_topic__` 자세 박지 X.
wire_key 정의 자리 = Module 별 `wire_keys.py` 의 `StrEnum` (유일).

wire encoding 자세 (spec §3.4) — Pydantic + msgpack layered:
- Module 자세 Pydantic schema 만 알음 (`event.model_dump()` 자세 dict)
- Transport boundary 자세 msgpack 박음 (`msgspec.msgpack` 자세 native bytes)
- `bytes` field 자세 base64 overhead 0 (JPEG / depth zstd / pointcloud 자리 영향 큼)

`@publishes((wire_key, event_cls), ...)` 자세 — Module 자세 publish 박는 pair 자세 self-declare:
- self-doc — Module 자세 어떤 wire_key + event 자세 publish 박는지 명시
- contract.ts 자동 generate 자세 활용 (frontend type emit)
- 실 publish 강제 X (declare 안 된 자세도 publish 가능)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, TypeVar

import msgspec
from pydantic import BaseModel

C = TypeVar("C", bound=type)
T = TypeVar("T", bound=BaseModel)


@dataclass(frozen=True)
class PublishesSpec:
    """Module 자세 publish 박는 (wire_key, event_cls) pair 자세 박힘."""
    pairs: tuple[tuple[str, type[BaseModel]], ...]


_PUBLISHES_ATTR = "_publishes_spec"


def publishes(
    *pairs: tuple[str, type[BaseModel]],
) -> Callable[[C], C]:
    """class-level 데코 — Module 이 publish 할 (wire_key, event_cls) pair 자세 self-declare.

    각 pair = `(WireKey.X, EventCls)` tuple. StrEnum value + Pydantic BaseModel subclass.

    실제 publish 강제 X — declare 안 된 pair 자세 publish 박아도 동작.
    self-doc / contract.ts 자세 활용.

    사용:
        @publishes(
            (CalibrationEventTopic.ACTIVATED, CalibrationActivated),
            (CalibrationEventTopic.COMMITTED, CalibrationCommitted),
        )
        class CalibrationModule: ...
    """
    # validate
    normalized: list[tuple[str, type[BaseModel]]] = []
    for i, pair in enumerate(pairs):
        if not isinstance(pair, tuple) or len(pair) != 2:
            raise TypeError(
                f"@publishes pair {i}: (wire_key, event_cls) tuple 박혀있어야 함 "
                f"(got {pair!r})"
            )
        wire_key, event_cls = pair
        if not isinstance(event_cls, type) or not issubclass(event_cls, BaseModel):
            raise TypeError(
                f"@publishes pair {i}: event_cls 자세 Pydantic BaseModel subclass "
                f"여야 함 (got {event_cls})"
            )
        normalized.append((str(wire_key), event_cls))

    def decorator(cls: C) -> C:
        spec = PublishesSpec(pairs=tuple(normalized))
        setattr(cls, _PUBLISHES_ATTR, spec)
        return cls

    return decorator


def get_publishes_spec(cls: type) -> PublishesSpec | None:
    return getattr(cls, _PUBLISHES_ATTR, None)


# ─── wire encoding (Pydantic + msgpack layered) ───────────────────────


def encode_event(event: BaseModel) -> bytes:
    """event instance → wire bytes (msgpack).

    Pydantic `model_dump()` 자세 dict 로 schema validation 자세 거친 후,
    msgspec.msgpack 자세 native bytes pass-through 자세 wire 보냄.
    `bytes` field 자세 base64 overhead 0.
    """
    return msgspec.msgpack.encode(event.model_dump())


def decode_event(event_cls: type[T], payload: bytes) -> T:
    """wire bytes → event instance (msgpack → Pydantic validate). 자세 generic."""
    return event_cls.model_validate(msgspec.msgpack.decode(payload))
