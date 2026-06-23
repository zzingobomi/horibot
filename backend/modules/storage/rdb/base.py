from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from datetime import UTC, datetime
from typing import TYPE_CHECKING, Any

from sqlalchemy import DateTime, Dialect, Engine, create_engine, event
from sqlalchemy.orm import DeclarativeBase, Mapper, Session
from sqlalchemy.pool import StaticPool
from sqlalchemy.types import TypeDecorator

if TYPE_CHECKING:
    from sqlalchemy.sql import FromClause


class UtcDateTime(TypeDecorator[datetime]):
    """UTC 기준으로 저장·조회되는 datetime 컬럼.

    SQLite 는 PostgreSQL 의 TIMESTAMPTZ 와 같은 timezone-aware datetime 타입을 제공하지 않는다.
    이 타입은 DB별 datetime 처리 차이를 숨기고 항상 UTC-aware datetime 을 반환한다.
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


class Base(DeclarativeBase):
    """모든 ORM 모델의 공통 base.

    SQLAlchemy 는 `__table__`, `__mapper__` 를 런타임에 동적으로 생성한다.
    pyright 는 이를 인식하지 못하므로 TYPE_CHECKING 용 선언을 추가한다.
    """

    if TYPE_CHECKING:
        # SQLAlchemy 가 런타임에 제공하는 속성.
        # 정적 타입 검사 시 ModelProtocol 만족을 위해 선언한다.
        __table__: FromClause  # pyright: ignore[reportIncompatibleVariableOverride]
        __mapper__: Mapper[Any]  # pyright: ignore[reportIncompatibleVariableOverride]


def make_engine(uri: str) -> Engine:
    if uri.startswith("sqlite:///:memory:") or uri == "sqlite://":
        engine = create_engine(
            uri,
            # SQLite in-memory DB 를 multi-thread에서 공유하려면 `check_same_thread=False` + StaticPool 필요.
            connect_args={"check_same_thread": False},
            poolclass=StaticPool,
            future=True,
        )
    elif uri.startswith("sqlite://"):
        engine = create_engine(
            uri,
            connect_args={"check_same_thread": False},
            future=True,
        )
    else:
        engine = create_engine(uri, future=True)

    if engine.dialect.name == "sqlite":

        @event.listens_for(engine, "connect")
        def _enable_fk_cascade(dbapi_conn, _conn_record):  # noqa: ANN001
            cursor = dbapi_conn.cursor()
            # SQLite는 외래키 제약조건이 기본적으로 비활성화되어 있으므로, 연결 시마다 활성
            cursor.execute("PRAGMA foreign_keys=ON")
            cursor.execute("PRAGMA journal_mode=WAL")
            cursor.close()

    return engine


# NOTE:
# 이 Storage Layer는 AsyncSession이 아닌 동기 Session을 사용한다.
#
# 이유:
# - Zenoh handler 는 호출 측이 sync (zenoh-python binding 의 queryable callback
#   contract 가 sync callable). storage 호출은 그 안에서 도는 짧은 transaction.
#   호출 측이 sync 인데 DB만 async 박으면 sync handler 안에서 event loop
#   juggling 하게 되어 오히려 복잡해진다.
# - 현재 storage 사용 패턴은 짧은 CRUD + 메타데이터 조회 위주이며 DB I/O 가
#   전체 pipeline 의 병목이 아니다. 동시 요청은 storage_node 의 multi-thread
#   handler + SQLAlchemy connection pool 로 이미 처리된다.
# - sync Session 은 명시적 commit/rollback/close 가 try/except/finally 와 자연
#   fit — service boundary 내부에서 단순하고 안정적이다.
#
# 추후 storage_node 자체가 async layer (예: aiohttp gateway) 로 전환되거나,
# 호출 측이 async event loop 위에서 도는 패턴이 등장하면, 이 경계 (RdbStore
# Protocol + session_scope) 는 유지한 채 AsyncSession 으로 교체 가능.
@contextmanager
def session_scope(engine: Engine) -> Iterator[Session]:
    session = Session(engine, future=True)
    try:
        yield session
        session.commit()
    except Exception:
        session.rollback()
        raise
    finally:
        session.close()
