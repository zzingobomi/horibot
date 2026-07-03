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


# ─── module attribution (contract graph viewer — contract_graph_viewer.md §5.1) ──
#
# build_snapshot 은 module attribution + publish/subscribe 방향을 버리고 flat dict
# 로 collapse 한다 (frontend gen 은 "wire_key → payload" 만 필요). 그래프 뷰어는
# 반대로 그 attribution + 방향이 본질 — 어느 module 이 무엇을 serve/publish/subscribe
# 하는지. 그래서 같은 discovery source 를 attribution 을 살려 다시 열거한다.


@dataclass(frozen=True)
class ModuleContract:
    """한 Module class 가 노출하는 계약 (attribution + 방향 보존).

    services / publishes / subscribes 는 wire_key template ({robot_id} 유지).
    per-robot 인스턴스 (같은 class) 는 template 이 동일하니 module_id 로 dedup.
    """

    module_id: str  # type(m).__name__
    robot_scoped: bool  # **service** wire_key 에 {robot_id} → per-robot 인스턴스 필요
    services: tuple[str, ...]  # @service wire_keys (owner=server)
    publishes: tuple[str, ...]  # @publishes wire_keys (output stream/event)
    subscribes: tuple[str, ...]  # @subscriber wire_keys (input stream/event)


def build_module_contracts(modules: list[Any]) -> list[ModuleContract]:
    """로드된 Module 인스턴스 → class 별 ModuleContract.

    같은 class 의 여러 인스턴스 (per-robot) 는 wire_key template 이 동일하므로
    module_id 기준 dedup (첫 인스턴스 wins). contract 이 하나도 없는 Module
    (Bridge 같은 relay) 도 여기선 그대로 열거 — 그래프 노드로 넣을지 (empty node
    제외) 는 apps 빌더의 관심사 (framework 는 editorialize X)."""
    seen: set[str] = set()
    out: list[ModuleContract] = []
    for module in modules:
        module_id = type(module).__name__
        if module_id in seen:
            continue
        seen.add(module_id)

        services = sorted(spec.wire_key for _bound, spec in discover_services(module))
        subscribes = sorted(
            spec.wire_key for _bound, spec in discover_subscribers(module)
        )
        pub = get_publishes_spec(type(module))
        publishes = sorted(wire_key for wire_key, _cls in pub.pairs) if pub else []

        # robot_scoped = **service 키** 의 {robot_id} — framework 가 self.robot_id
        # 를 요구하는 유일한 자리 (per-robot 인스턴스 필요의 SSOT). stream/event 는
        # payload robot_id 라우팅 / wildcard 구독이라 host-level module 도 robot-
        # scoped 키를 publish/subscribe 함 (예: 호스트 1개 calibration 의 preview).
        robot_scoped = any("{robot_id}" in k for k in services)
        out.append(
            ModuleContract(
                module_id=module_id,
                robot_scoped=robot_scoped,
                services=tuple(services),
                publishes=tuple(publishes),
                subscribes=tuple(subscribes),
            )
        )
    out.sort(key=lambda mc: mc.module_id)
    return out


# ─── class-only variant — 분산 배치 자리 contract graph viewer 를 위한 자리 ─
#
# build_module_contracts(instances) 는 이 프로세스에 로드된 Module 인스턴스만 봄.
# 분산 배치 (예: PC 는 camera_decoded + bridge, pi_motor 는 motor + motion) 자리
# bridge 의 `/contract/graph` 는 fleet 전체 아키텍처를 보여야 개발자 뷰어로서
# 의미 있음. class 자체는 decorator spec (_service_spec / _subscriber_spec /
# _publishes_spec) 을 attribute 로 들고 있어 instantiate 없이 introspect 가능.


def build_module_contracts_from_classes(classes: list[type]) -> list[ModuleContract]:
    """Module class 리스트 → ModuleContract. 인스턴스 생성 없이 decorator spec 만
    introspect. `discover_services(cls)` / `discover_subscribers(cls)` 는 dir/getattr
    라 class 도 동작 (attribute 는 unbound function 에 붙어있음)."""
    seen: set[str] = set()
    out: list[ModuleContract] = []
    for cls in classes:
        module_id = cls.__name__
        if module_id in seen:
            continue
        seen.add(module_id)

        services = sorted(spec.wire_key for _bound, spec in discover_services(cls))
        subscribes = sorted(
            spec.wire_key for _bound, spec in discover_subscribers(cls)
        )
        pub = get_publishes_spec(cls)
        publishes = sorted(wire_key for wire_key, _cls in pub.pairs) if pub else []

        # service 키 기준 — build_module_contracts 와 동일 규칙 (위 주석 참조)
        robot_scoped = any("{robot_id}" in k for k in services)
        out.append(
            ModuleContract(
                module_id=module_id,
                robot_scoped=robot_scoped,
                services=tuple(services),
                publishes=tuple(publishes),
                subscribes=tuple(subscribes),
            )
        )
    out.sort(key=lambda mc: mc.module_id)
    return out


def build_snapshot_from_classes(classes: list[type]) -> ContractSnapshot:
    """Module class 리스트 → ContractSnapshot (payload 매핑). class introspection —
    build_snapshot(instances) 의 class-only 변형. 같은 wire_key 를 여러 class 가
    출현하면 (없어야 정상) `_put_service` / `_put_topic` 이 충돌 fail-fast."""
    services: dict[str, tuple[type[BaseModel], type[BaseModel]]] = {}
    topics: dict[str, type[BaseModel]] = {}

    for cls in classes:
        for _bound, spec in discover_services(cls):
            _put_service(services, spec.wire_key, spec.req_cls, spec.res_cls)
        for _bound, spec in discover_subscribers(cls):
            _put_topic(topics, spec.wire_key, spec.event_cls)
        pub = get_publishes_spec(cls)
        if pub is not None:
            for wire_key, event_cls in pub.pairs:
                _put_topic(topics, wire_key, event_cls)

    return ContractSnapshot(services=services, topics=topics)
