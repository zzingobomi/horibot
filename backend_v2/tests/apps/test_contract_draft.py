"""DraftModel 마커 판정 검증 — introspection 계층의 draft 인지 로직.

draft 는 엔드포인트가 아니라 *payload 타입* 에 붙는 마커다. 그래서:
  - 서비스는 req·res 중 하나라도 DraftModel 이면 draft
  - 토픽은 payload(req_cls) 가 DraftModel 이면 draft
  - StrictModel / plain BaseModel 은 draft 아님
build_contract_json / build_contract_graph 가 이 판정으로 draft 메타를 emit 한다.
"""

from __future__ import annotations

from pydantic import BaseModel

from apps.contract_export import KeyEntry, _is_draft_model, _key_is_draft
from framework.contract.model import DraftModel, StrictModel


class _Draft(DraftModel):
    pass


class _Strict(StrictModel):
    foo: int = 0


class _Plain(BaseModel):
    foo: int = 0


def _svc(req: type[BaseModel] | None, res: type[BaseModel] | None) -> KeyEntry:
    return KeyEntry(
        const_name="X", key="srv/x/y", category="service", req_cls=req, res_cls=res
    )


def _topic(payload: type[BaseModel] | None) -> KeyEntry:
    return KeyEntry(
        const_name="X", key="stream/x/y", category="stream", req_cls=payload, res_cls=None
    )


def test_is_draft_model_only_true_for_draftmodel_subclass():
    assert _is_draft_model(_Draft) is True
    assert _is_draft_model(_Strict) is False
    assert _is_draft_model(_Plain) is False
    assert _is_draft_model(None) is False


def test_service_draft_iff_req_or_res_is_draft():
    assert _key_is_draft(_svc(_Draft, _Strict)) is True  # req only
    assert _key_is_draft(_svc(_Strict, _Draft)) is True  # res only
    assert _key_is_draft(_svc(_Draft, _Draft)) is True
    assert _key_is_draft(_svc(_Strict, _Strict)) is False
    assert _key_is_draft(_svc(_Plain, None)) is False


def test_topic_draft_follows_payload():
    assert _key_is_draft(_topic(_Draft)) is True
    assert _key_is_draft(_topic(_Strict)) is False
    assert _key_is_draft(_topic(_Plain)) is False


def test_draftmodel_allows_extra_fields():
    # extra="allow" — 선언 안 한 필드도 통과 (탐색 중 payload 자유롭게)
    m = _Draft.model_validate({"anything": 1, "angle_rad": 0.3})
    assert m.model_dump()["angle_rad"] == 0.3
