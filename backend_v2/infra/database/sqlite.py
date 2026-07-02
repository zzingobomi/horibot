from __future__ import annotations

from pathlib import Path

from sqlalchemy import Engine, create_engine, event
from sqlalchemy.orm import Session, sessionmaker
from sqlalchemy.pool import StaticPool


def open_sqlite(
    path: str | Path = ":memory:",
    *,
    echo: bool = False,
) -> tuple[Engine, sessionmaker[Session]]:
    if isinstance(path, Path):
        url = f"sqlite:///{path.resolve()}"
    elif path == ":memory:":
        url = "sqlite:///:memory:"
    else:
        url = f"sqlite:///{path}"

    is_memory = url.endswith(":memory:")
    if is_memory:
        # in-memory 는 connection 마다 별 DB — 프로세스에서 공유하려면 StaticPool +
        # check_same_thread=False (single shared connection). shared engine 싱글톤이
        # 여러 session/thread 에서 같은 in-memory DB 를 봐야 하므로 필수.
        engine = create_engine(
            url,
            echo=echo,
            future=True,
            connect_args={"check_same_thread": False},
            poolclass=StaticPool,
        )
    else:
        engine = create_engine(url, echo=echo, future=True)

    # SQLite 는 외래키 제약이 connection 마다 기본 OFF — 켜지 않으면 ORM 의
    # ondelete="CASCADE" 가 **조용히 no-op** 된다. 첫 FK-CASCADE 테이블(calibration)
    # 부터 필요. 파일 DB 는 WAL 로 동시 read/write latency 완화 (:memory: 는 무시됨).
    @event.listens_for(engine, "connect")
    def _sqlite_pragmas(dbapi_conn, _record):  # noqa: ANN001
        cur = dbapi_conn.cursor()
        cur.execute("PRAGMA foreign_keys=ON")
        if not is_memory:
            cur.execute("PRAGMA journal_mode=WAL")
        cur.close()

    factory = sessionmaker(bind=engine, future=True, expire_on_commit=False)
    return engine, factory
