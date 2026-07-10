"""Project-wide Alembic env — single DB, single migration history.

모듈은 자기 테이블/ORM 만 소유 (modules/<name>/orm.py). 이 루트 env 가 **모든 DB 모듈
ORM 을 import** 해서 공유 `Base.metadata` 에 등록 → autogenerate 가 전 스키마를 한
history 로 관리. 새 DB 모듈 추가 시 아래 REGISTER 블록에 import 한 줄.

URL 우선순위: runtime connection (config.attributes["connection"]) > CLI `-x db_url=`
> env HORIBOT_DB_URL > alembic.ini sqlalchemy.url.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from alembic import context
from sqlalchemy import create_engine, pool

# backend_v2 root 를 path 에.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from infra.database.base import Base  # noqa: E402
from infra.database.types import UtcDateTime  # noqa: E402

# ── REGISTER: DB 모듈 ORM import (테이블을 Base.metadata 에 등록) ──
# 새 DB 모듈 추가 시 여기 한 줄. import 만으로 mapper 등록됨.
import modules.calibration.persistence.orm  # noqa: E402,F401
import modules.scan.persistence.orm  # noqa: E402,F401
import modules.waypoint.persistence.orm  # noqa: E402,F401

# ─────────────────────────────────────────────────────────────────

config = context.config
target_metadata = Base.metadata


def _resolve_url() -> str:
    x = context.get_x_argument(as_dictionary=True)
    return (
        x.get("db_url")
        or os.environ.get("HORIBOT_DB_URL")
        or config.get_main_option("sqlalchemy.url")
        or "sqlite:///horibot.db"
    )


def _render_item(type_, obj, autogen_context):  # noqa: ANN001, ANN202
    """migration 을 app TypeDecorator 에 결합 X — UtcDateTime 은 DDL 상 그냥
    timezone-aware DATETIME 이므로 `sa.DateTime(timezone=True)` 로 스냅샷.
    (기본 autogen 은 `infra.database.types.UtcDateTime(...)` 로 렌더하는데 import 를
    안 넣어 NameError.)"""
    # script.py.mako 가 이미 `import sqlalchemy as sa` 를 넣으므로 imports 추가 X (중복 방지).
    if type_ == "type" and isinstance(obj, UtcDateTime):
        return "sa.DateTime(timezone=True)"
    return False


def run_migrations_offline() -> None:
    context.configure(
        url=_resolve_url(),
        target_metadata=target_metadata,
        literal_binds=True,
        render_as_batch=True,
        render_item=_render_item,
        dialect_opts={"paramstyle": "named"},
    )
    with context.begin_transaction():
        context.run_migrations()


def run_migrations_online() -> None:
    connectable = config.attributes.get("connection", None)
    if connectable is not None:
        context.configure(
            connection=connectable,
            target_metadata=target_metadata,
            render_as_batch=True,
            render_item=_render_item,
        )
        with context.begin_transaction():
            context.run_migrations()
        return

    engine = create_engine(_resolve_url(), poolclass=pool.NullPool, future=True)
    with engine.connect() as connection:
        context.configure(
            connection=connection,
            target_metadata=target_metadata,
            render_as_batch=True,
            render_item=_render_item,
        )
        with context.begin_transaction():
            context.run_migrations()


if context.is_offline_mode():
    run_migrations_offline()
else:
    run_migrations_online()
