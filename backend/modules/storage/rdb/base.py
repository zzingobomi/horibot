from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager

from sqlalchemy import Engine, create_engine, event
from sqlalchemy.orm import DeclarativeBase, Session
from sqlalchemy.pool import StaticPool


class Base(DeclarativeBase):
    """모든 ORM 모델의 공통 base."""


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
