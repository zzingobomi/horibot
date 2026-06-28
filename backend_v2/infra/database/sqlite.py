from __future__ import annotations

from pathlib import Path

from sqlalchemy import Engine, create_engine
from sqlalchemy.orm import Session, sessionmaker


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

    engine = create_engine(url, echo=echo, future=True)
    factory = sessionmaker(bind=engine, future=True, expire_on_commit=False)
    return engine, factory
