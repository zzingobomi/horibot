from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, TypeVar

import msgspec
from pydantic import BaseModel

C = TypeVar("C", bound=type)
T = TypeVar("T", bound=BaseModel)


@dataclass(frozen=True)
class PublishesSpec:
    pairs: tuple[tuple[str, type[BaseModel]], ...]


_PUBLISHES_ATTR = "_publishes_spec"


def publishes(
    *pairs: tuple[str, type[BaseModel]],
) -> Callable[[C], C]:
    normalized: list[tuple[str, type[BaseModel]]] = []
    for i, pair in enumerate(pairs):
        if not isinstance(pair, tuple) or len(pair) != 2:
            raise TypeError(
                f"@publishes pair {i}: (wire_key, event_cls) tuple 필요 "
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


# ─── wire encoding ───────────────────────


def encode_event(event: BaseModel) -> bytes:
    return msgspec.msgpack.encode(event.model_dump())


def decode_event(event_cls: type[T], payload: bytes) -> T:
    return event_cls.model_validate(msgspec.msgpack.decode(payload))
