"""plc wire 계약 — 산업용 PLC 태그의 read model + write 표면.

설계 정본 = docs/plc_conveyor.md §9 (2026-07-23 확정, §10 잠금).
Task/frontend 는 **의미 이름(태그)** 만 안다 — 주소·프로토콜은 드라이버 뒤
(태그↔주소 바인딩 = plc/plcs.yaml 인스턴스 config).

- host-scoped 1 인스턴스 (robot-scoped 아님 — backend.md §2.7). 대상 PLC 는
  req 필드 `plc_id` 로 파생 (여러 PLC 를 한 모듈이 스캔).
- 태그 상태 = Mirror (SNAPSHOT_TAGS + TAGS_CHANGED invalidate) — 폴링(Modbus)
  이냐 구독(OPC-UA)이냐를 드라이버 뒤로 숨기는 seam.
- **quality 는 산업통신 본질** (§10): "PLC 죽음"과 "값 false" 를 구분한다.
  connected=False 시 태그는 마지막 값 유지 + STALE 강등 — 침묵 fallback 금지.
"""

from __future__ import annotations

from datetime import datetime
from enum import StrEnum

from framework.contract.model import StrictModel

# 태그 값 스칼라 — config dtype 에서 런타임 디코드 (Generic[T] 금지, §10).
TagScalar = bool | int | float | str


class Quality(StrEnum):
    """태그 품질 (OPC-UA StatusCode / Ignition quality 대응 — 초기 3개, §9.2)."""

    GOOD = "good"  # 이번 스캔에서 정상 읽음
    STALE = "stale"  # 연결 끊김 — 마지막으로 읽은 값 유지 중
    BAD = "bad"  # 장비가 에러 응답 (주소 없음 / 범위 밖 등)


class TagReading(StrictModel):
    """태그 1개의 현재값 — 값이 아니라 값+품질+시각 (§9.2 TagValue 의 wire 투영)."""

    value: TagScalar
    quality: Quality
    ts: datetime  # 드라이버가 읽은 시각 (UTC). STALE 은 마지막 GOOD 시각 유지


class PlcSnapshot(StrictModel):
    """PLC 1대의 현재 상태 — 연결 여부 + 전 태그."""

    connected: bool
    tags: dict[str, TagReading]


class PlcBundle(StrictModel):
    """전 PLC snapshot — Mirror value. key = plc_id (plcs.yaml)."""

    plcs: dict[str, PlcSnapshot]


class SnapshotTagsRequest(StrictModel):
    pass


class WriteTagRequest(StrictModel):
    plc_id: str
    tag: str  # 의미 이름 (plcs.yaml tags 키) — 주소 아님
    value: TagScalar


class WriteTagResponse(StrictModel):
    """드라이버로 나간 확정값. 태그 read model 반영은 **다음 스캔**부터 —
    PLC 자체 스캔(수십 ms)이 있어 즉시 read-back 은 반영 전 값을 잡는다
    (plc/probe.py 실측 교훈)."""

    tag: str
    value: TagScalar


class TagsChanged(StrictModel):
    """Mirror invalidate 이벤트 — 소비자는 이걸 계기로 snapshot 재당김.

    changed = 이번 스캔에서 값/품질이 바뀐 태그만 (연결 단절 시 = STALE 강등된
    전체). 연결 전이 자체도 발행된다 (changed 가 비어도)."""

    plc_id: str
    seq: int
    timestamp_unix: float
    connected: bool
    changed: dict[str, TagReading]


class Plc:
    class Service(StrEnum):
        SNAPSHOT_TAGS = "srv/plc/snapshot_tags"
        WRITE_TAG = "srv/plc/write_tag"

    class Event(StrEnum):
        TAGS_CHANGED = "event/plc/tags_changed"
