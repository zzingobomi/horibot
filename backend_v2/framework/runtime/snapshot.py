"""ContractSnapshot — 런타임에 로드된 Module 들이 노출하는 계약(wire_key → payload).

frontend_contract_gen.md §6.1. gen_contract (frontend TS) 가 module.py (numpy/torch/
open3d 등 heavy dep) 를 import 하지 않고도 key↔payload 매핑을 얻게, *이미 import 를
끝낸 running runtime* 이 자기 registry 를 직렬화해 내준다.

- services: wire_key(template, {robot_id} 유지) → (req_cls, res_cls)
- topics:   wire_key(template) → payload_cls (stream + event, publish/subscribe 무관)

category(service/stream/event) 는 wire_key prefix 로 유도 (srv/ stream/ event/).
const 이름(GET_TOPOLOGY 등) 은 여기 없음 — contract.py enum 에만 있고, contract.py 는
import-light (StrEnum + BaseModel 뿐) 라 serializer 가 직접 읽는다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from pydantic import BaseModel

from framework.contract.publisher import get_publishes_spec
from framework.runtime.discovery import discover_services, discover_subscribers


@dataclass(frozen=True)
class ContractSnapshot:
    services: dict[str, tuple[type[BaseModel], type[BaseModel]]] = field(
        default_factory=dict
    )
    topics: dict[str, type[BaseModel]] = field(default_factory=dict)


def build_snapshot(modules: list[Any]) -> ContractSnapshot:
    """로드된 Module 인스턴스들의 @service / @subscriber / @publishes spec 열거.

    같은 wire_key 가 여러 Module (per-robot 인스턴스 등) 에서 나와도 template 이
    동일하니 dedup — 다른 payload 로 충돌하면 fail-fast (spec invariant 위반)."""
    services: dict[str, tuple[type[BaseModel], type[BaseModel]]] = {}
    topics: dict[str, type[BaseModel]] = {}

    for module in modules:
        for _bound, spec in discover_services(module):
            _put_service(services, spec.wire_key, spec.req_cls, spec.res_cls)
        for _bound, spec in discover_subscribers(module):
            _put_topic(topics, spec.wire_key, spec.event_cls)
        pub = get_publishes_spec(type(module))
        if pub is not None:
            for wire_key, event_cls in pub.pairs:
                _put_topic(topics, wire_key, event_cls)

    return ContractSnapshot(services=services, topics=topics)


def _put_service(
    dst: dict[str, tuple[type[BaseModel], type[BaseModel]]],
    key: str,
    req_cls: type[BaseModel],
    res_cls: type[BaseModel],
) -> None:
    existing = dst.get(key)
    if existing is None:
        dst[key] = (req_cls, res_cls)
    elif existing != (req_cls, res_cls):
        raise RuntimeError(
            f"service {key!r} 가 서로 다른 payload 로 등록됨: "
            f"{existing[0].__name__}/{existing[1].__name__} vs "
            f"{req_cls.__name__}/{res_cls.__name__}"
        )


def _put_topic(
    dst: dict[str, type[BaseModel]], key: str, payload_cls: type[BaseModel]
) -> None:
    existing = dst.get(key)
    if existing is None:
        dst[key] = payload_cls
    elif existing is not payload_cls:
        raise RuntimeError(
            f"topic {key!r} 가 서로 다른 payload 로 등록됨: "
            f"{existing.__name__} vs {payload_cls.__name__}"
        )
