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
