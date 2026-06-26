"""@subscriber 데코 — event subscriber 박힌 method 의 contract spec 추출.

두 원칙 (spec §3.0):
- explicit at every use site — wire_key 자세 `@subscriber` 인자 자세 직접 박힘
- typed — `StrEnum` value (raw str 도 받음 — 단 사용자 코드 자세 StrEnum 추천)

사용 (spec §3.2):
    class AuditModule:
        @subscriber(CalibrationEventTopic.ACTIVATED)             # wire_key 명시
        def on_calibration_activated(self, event: CalibrationActivated):
            self.log_audit(event)

framework 자세:
- wire_key 자세 = `@subscriber` 인자 — transport subscribe key.
- event class 자세 = method 의 event parameter type hint — payload decode 자세 type.
- 둘 다 typed (raw string / `__wire_topic__` class attribute lookup 자세 X).
"""

from __future__ import annotations

import inspect
from dataclasses import dataclass
from typing import Any, Callable, get_type_hints

from pydantic import BaseModel


@dataclass(frozen=True)
class SubscriberSpec:
    method_name: str
    wire_key: str                         # @subscriber 인자 = explicit key (§3.0)
    event_cls: type[BaseModel]            # type hint 자세 추출 (decode 자세)
    handler: Callable[..., Any]


_SUBSCRIBER_ATTR = "_subscriber_spec"


def subscriber(
    wire_key: str,
) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    """factory — wire_key 인자 명시 + type hint 에서 event_cls 추출.

    wire_key 자세 = `StrEnum` value (Module 별 `wire_keys.py` 에 정의) 자세 추천.
    raw str 도 받음 — 단 사용자 코드 자세 StrEnum 박는 자세 (두 원칙 §3.0).
    """
    key_str = str(wire_key)               # StrEnum value → str

    def decorator(method: Callable[..., Any]) -> Callable[..., Any]:
        hints = get_type_hints(method)
        sig = inspect.signature(method)
        params = [p for p in sig.parameters.values() if p.name != "self"]

        if len(params) != 1:
            raise TypeError(
                f"@subscriber '{method.__name__}': self + event 한 parameter 필요 "
                f"(got {len(params)} parameter)"
            )

        event_name = params[0].name
        event_cls = hints.get(event_name)

        if (
            event_cls is None
            or not isinstance(event_cls, type)
            or not issubclass(event_cls, BaseModel)
        ):
            raise TypeError(
                f"@subscriber '{method.__name__}': event parameter '{event_name}' 의 "
                f"type hint 가 BaseModel subclass 여야 함 (got {event_cls})"
            )

        spec = SubscriberSpec(
            method_name=method.__name__,
            wire_key=key_str,
            event_cls=event_cls,
            handler=method,
        )
        setattr(method, _SUBSCRIBER_ATTR, spec)
        return method

    return decorator


def is_subscriber(method: Any) -> bool:
    return hasattr(method, _SUBSCRIBER_ATTR)


def get_subscriber_spec(method: Any) -> SubscriberSpec | None:
    return getattr(method, _SUBSCRIBER_ATTR, None)
