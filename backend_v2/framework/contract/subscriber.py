"""@subscriber 데코 — event subscriber 박힌 method 의 contract spec 추출.

사용 (spec §3.2):
    class AuditModule:
        @subscriber
        def on_calibration_activated(self, event: CalibrationActivated):
            self.log_audit(event)

event class 자체가 topic 식별 — topic string 박지 않음. framework 가 event_cls 의
type hint 에서 event class 추출 후, publisher.event_to_topic 으로 wire key 변환.
"""

from __future__ import annotations

import inspect
from dataclasses import dataclass
from typing import Any, Callable, get_type_hints

from pydantic import BaseModel


@dataclass(frozen=True)
class SubscriberSpec:
    """@subscriber 데코가 박힌 method 의 contract spec."""

    method_name: str
    event_cls: type[BaseModel]
    handler: Callable[..., Any]


_SUBSCRIBER_ATTR = "_subscriber_spec"


def subscriber(method: Callable[..., Any]) -> Callable[..., Any]:
    """method 박힌 event parameter 의 type hint 에서 event_cls 추출.

    제약:
    - method 시그니처 = `(self, event: EventCls) -> None`
    - EventCls = Pydantic BaseModel subclass
    """
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
        event_cls=event_cls,
        handler=method,
    )
    setattr(method, _SUBSCRIBER_ATTR, spec)
    return method


def is_subscriber(method: Any) -> bool:
    return hasattr(method, _SUBSCRIBER_ATTR)


def get_subscriber_spec(method: Any) -> SubscriberSpec | None:
    return getattr(method, _SUBSCRIBER_ATTR, None)
