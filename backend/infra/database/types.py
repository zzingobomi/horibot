from __future__ import annotations

from datetime import UTC, datetime
from typing import Any

from sqlalchemy import DateTime, Dialect
from sqlalchemy.types import TypeDecorator


class UtcDateTime(TypeDecorator[datetime]):
    """UTC-aware datetime 컬럼 — DB 별 timezone 처리 차이를 숨긴다.

    SQLite 는 PostgreSQL 의 TIMESTAMPTZ 같은 timezone-aware 타입이 없다. 이 타입은
    저장 시 UTC 로 정규화, 조회 시 UTC-aware datetime 을 항상 돌려준다.

    프로젝트 전역 컨벤션 — persisted timestamp = UTC-aware (CLAUDE.md). 옛 backend
    `modules/storage/rdb/base.py::UtcDateTime` 에서 infra 로 승격 (Database-per-Module
    이라 각 모듈 ORM 이 공유해야 하는 generic 타입 — 도메인 무관).
    """

    impl = DateTime(timezone=True)
    cache_ok = True

    def process_bind_param(
        self, value: datetime | None, dialect: Dialect
    ) -> datetime | None:
        if value is None:
            return None
        if value.tzinfo is None:
            return value.replace(tzinfo=UTC)
        return value.astimezone(UTC)

    def process_result_value(self, value: Any, dialect: Dialect) -> datetime | None:
        if value is None:
            return None
        if value.tzinfo is None:
            return value.replace(tzinfo=UTC)
        return value
