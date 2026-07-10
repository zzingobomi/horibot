from __future__ import annotations

from pathlib import Path

from alembic import command
from alembic.config import Config
from sqlalchemy import Engine
from sqlalchemy.orm import Session, sessionmaker

from .postgres import open_postgres
from .sqlite import open_sqlite


_ALEMBIC_INI = Path(__file__).resolve().parents[2] / "alembic.ini"


def open_database(uri: str) -> tuple[Engine, sessionmaker[Session]]:
    if uri.startswith("sqlite://"):
        # sqlite:///path 또는 sqlite:///:memory:
        rest = uri[len("sqlite:///") :] if uri.startswith("sqlite:///") else ":memory:"
        return open_sqlite(rest or ":memory:")
    if uri.startswith(("postgresql://", "postgresql+psycopg://")):
        return open_postgres(uri)
    raise ValueError(f"지원 안 하는 rdb_uri scheme: {uri!r}")


def run_migrations(engine: Engine) -> None:
    cfg = Config(str(_ALEMBIC_INI))
    with engine.connect() as conn:
        cfg.attributes["connection"] = conn
        command.upgrade(cfg, "head")
