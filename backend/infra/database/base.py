"""공유 ORM Base — 하나의 물리 DB 를 여러 모듈이 공유 (모듈은 자기 테이블 소유).

소유권 vs 마이그레이션 분리:
- **테이블/ORM 소유 = 모듈별** — calibration_* 는 calibration, scan_* 는 scan 모듈이
  자기 orm.py 에서 정의 (Storage Module RPC 중개자 폐기는 그대로).
- **마이그레이션 권위 = 루트 하나** (`backend_v2/alembic/`) — 같은 프로세스/공유 DB 라
  Database-per-Service 가 아님. 단일 metadata + 단일 history 가 version_table 충돌 /
  cross-module FK 순서 문제를 자연 회피.

모든 DB 모듈 ORM 이 이 Base 를 상속 → `Base.metadata` 에 전 모듈 테이블 등록 (루트
alembic 이 target). 특정 모듈만 import 하면 그 모듈 테이블만 등록되므로 test 의
`create_all` 은 여전히 모듈 단위로 동작.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from sqlalchemy.orm import DeclarativeBase, Mapper

if TYPE_CHECKING:
    from sqlalchemy.sql import FromClause


class Base(DeclarativeBase):
    if TYPE_CHECKING:
        # SQLAlchemy 런타임 제공 속성 — pyright 정적 검사용 선언.
        __table__: FromClause  # pyright: ignore[reportIncompatibleVariableOverride]
        __mapper__: Mapper[Any]  # pyright: ignore[reportIncompatibleVariableOverride]
