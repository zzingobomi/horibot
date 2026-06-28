from __future__ import annotations

import inspect
from dataclasses import dataclass
from typing import Any, Callable, get_type_hints

from pydantic import BaseModel


@dataclass(frozen=True)
class SubscriberSpec:
    method_name: str
    wire_key: str
    event_cls: type[BaseModel]
    handler: Callable[..., Any]


_SUBSCRIBER_ATTR = "_subscriber_spec"


def subscriber(
    wire_key: str,
) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    key_str = str(wire_key)

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
