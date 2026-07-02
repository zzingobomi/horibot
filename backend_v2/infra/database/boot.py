"""DB boot 헬퍼 — process infra 싱글톤 생성 + 마이그레이션.

boot(main/build_runtime)가 프로세스당 1번 호출: engine + session_factory 생성
(ZenohSession 과 동형 process infra) → `run_migrations` (root alembic upgrade head)
→ 모든 DB 모듈 Repository 에 같은 session_factory 주입.
"""

from __future__ import annotations

from pathlib import Path

from alembic import command
from alembic.config import Config
from sqlalchemy import Engine
from sqlalchemy.orm import Session, sessionmaker

from .postgres import open_postgres
from .sqlite import open_sqlite

# backend_v2/alembic.ini (루트 단일 마이그레이션)
_ALEMBIC_INI = Path(__file__).resolve().parents[2] / "alembic.ini"


def open_database(uri: str) -> tuple[Engine, sessionmaker[Session]]:
    """rdb_uri scheme 으로 sqlite / postgres dispatch."""
    if uri.startswith("sqlite://"):
        # sqlite:///path 또는 sqlite:///:memory:
        rest = uri[len("sqlite:///") :] if uri.startswith("sqlite:///") else ":memory:"
        return open_sqlite(rest or ":memory:")
    if uri.startswith(("postgresql://", "postgresql+psycopg://")):
        return open_postgres(uri)
    raise ValueError(f"지원 안 하는 rdb_uri scheme: {uri!r}")


def run_migrations(engine: Engine) -> None:
    """root alembic upgrade head — engine 의 connection 재사용 (:memory: 도 같은 DB)."""
    cfg = Config(str(_ALEMBIC_INI))
    with engine.connect() as conn:
        cfg.attributes["connection"] = conn
        command.upgrade(cfg, "head")
