"""
계약 모델 공통 베이스 (StrictModel)

StrictModel은 모든 wire contract에서 `extra="forbid"`를 강제하는 pydantic 기본 클래스다.

목적:
- 정의되지 않은 필드 차단 (오타 / 스키마 드리프트 숨김 방지)
- API/통신 경계에서 fail-fast 보장

제약:
- import-light 유지 (pydantic만 사용)
- domain / framework 의존성 없음
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class DraftModel(BaseModel):
    """탐색 단계의 미완성 계약 마커 — TS 의 `any` 에 대응하는 명시적 표식.

    아직 필드가 확정되지 않은 wire payload(요청/응답/이벤트)에 상속시킨다.
    StrictModel(확정 계약)의 반대쪽 끝 — `extra="allow"` 로 선언 안 한 필드도 통과시켜,
    타입을 미리 못박지 않고도 서비스/토픽을 띄우고 payload 를 바꿔가며 탐색할 수 있다.

    이건 계약 경계를 느슨하게 만드는 게 아니라 "아직 안 굳음" 이라는 *상태* 에
    이름을 주는 마커다. framework introspection(contract graph / export)이
    `issubclass(cls, DraftModel)` 로 인지해 일관되게 굴린다:
      - graph viewer 에 [DRAFT] 배지 (숨기지 않음 — 미완성이지 부재가 아님)
      - contract export / 생성 타입에 draft 메타 보존
      - CI/스크립트(scripts/list_draft_contracts.py)로 "안 굳은 계약" 목록화
    런타임 동작은 바꾸지 않는다 (그냥 BaseModel — validate/encode 동일).

    모양이 굳으면 base 를 StrictModel 로 교체한다 — `extra="forbid"` 로 바뀌며
    빠뜨린 필드가 fail-fast 로 걸리는 게 자연스러운 "다 잡았나" 체크가 된다.
    """

    model_config = ConfigDict(extra="allow")
