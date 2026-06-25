"""@publishes 데코 — class-level event publish self-declaration + topic 변환.

사용 (spec §3.2):
    @publishes(CalibrationActivated, CalibrationCommitted)
    class CalibrationModule:
        @service
        def activate(self, req: ActivateRequest) -> ActivateResponse:
            ...
            self.publish(CalibrationActivated(...))

self-doc + future contract.ts auto-emit 용. self.publish 의 impl + binding 은
Runtime (Step 3) 의 책임 — 본 모듈은 declaration spec 만.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Callable, TypeVar

from pydantic import BaseModel

C = TypeVar("C", bound=type)


@dataclass(frozen=True)
class PublishesSpec:
    """Module class 박힌 declaration — 어떤 event 들을 publish 하나."""

    event_classes: tuple[type[BaseModel], ...]


_PUBLISHES_ATTR = "_publishes_spec"


def publishes(*event_classes: type[BaseModel]) -> Callable[[C], C]:
    """class-level 데코 — Module 이 publish 할 event class 들 self-declare.

    실제 publish 강제 X — declare 안 된 event 도 self.publish 가능 (단 contract
    surface 누락 = self-doc 약화). 운영 strict 모드는 Runtime 옵션 자리.
    """

    def decorator(cls: C) -> C:
        spec = PublishesSpec(event_classes=tuple(event_classes))
        setattr(cls, _PUBLISHES_ATTR, spec)
        return cls

    return decorator


def get_publishes_spec(cls: type) -> PublishesSpec | None:
    return getattr(cls, _PUBLISHES_ATTR, None)


# ─── event class → topic key 변환 ─────────────────────────

# CamelCase / acronym → snake_case 변환 — 두 단계 regex 정석:
#   1) `XYZw` 형태 (acronym + 새 단어) → `XYZ_w` (마지막 대문자 앞 분리)
#   2) `xY` / `9Y` 형태 (소문자/숫자 + 대문자) → `x_Y`
# 결과 lower → `httpresponse` X, `http_response` O.
_ACRONYM_BOUNDARY = re.compile(r"([A-Z]+)([A-Z][a-z])")
_WORD_BOUNDARY = re.compile(r"([a-z\d])([A-Z])")


def event_to_topic(event_cls: type[BaseModel]) -> str:
    """event class → topic string.

    형식: `event/{snake_case_of_class_name}`
    예:
        `CalibrationActivated` → `event/calibration_activated`
        `HTTPResponse`         → `event/http_response`

    naming collision 회피는 class name unique 성에 의존. 같은 base name 박지 말 것
    (예: `calibration.Activated` 와 `motion.Activated` 둘 다 `event/activated` 충돌).
    """
    name = event_cls.__name__
    s1 = _ACRONYM_BOUNDARY.sub(r"\1_\2", name)
    s2 = _WORD_BOUNDARY.sub(r"\1_\2", s1)
    return f"event/{s2.lower()}"


def encode_event(event: BaseModel) -> bytes:
    """event 인스턴스 → wire bytes (Pydantic JSON)."""
    return event.model_dump_json(by_alias=True).encode()


def decode_event(event_cls: type[BaseModel], payload: bytes) -> BaseModel:
    """wire bytes → event 인스턴스 (Pydantic JSON parse + validation)."""
    return event_cls.model_validate_json(payload)
