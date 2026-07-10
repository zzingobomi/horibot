from __future__ import annotations

from sqlalchemy import Engine, create_engine
from sqlalchemy.orm import Session, sessionmaker


def open_postgres(
    url: str,
    *,
    echo: bool = False,
    pool_size: int = 5,
    pool_pre_ping: bool = True,
) -> tuple[Engine, sessionmaker[Session]]:
    if not url.startswith(("postgresql://", "postgresql+psycopg://")):
        raise ValueError(
            f"PostgreSQL URL 'postgresql://' 또는 'postgresql+psycopg://' "
            f"시작 박힘 (got {url!r})"
        )

    engine = create_engine(
        url,
        echo=echo,
        future=True,
        pool_size=pool_size,
        pool_pre_ping=pool_pre_ping,
    )
    factory = sessionmaker(bind=engine, future=True, expire_on_commit=False)
    return engine, factory
