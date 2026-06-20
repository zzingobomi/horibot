"""Alembic env — Base.metadata 위에서 동작.

dialect-portable — env.py 자체는 SQLite / Postgres 무관. `engine_from_config`
가 `sqlalchemy.url` 의 dialect prefix 자리 보고 알맞은 engine 자리 자동 생성.
NAS Postgres 진입 자리 = host yaml 의 rdb_uri 만 `postgresql://...` 로 교체,
본 파일 변경 X.

두 entry 지원 (Alembic 정석):

1) Programmatic — StorageRegistry 가 부팅 시 `config.attributes['connection']`
   으로 live SQLAlchemy connection 주입. host yaml 의 rdb_uri 가 engine 생성
   SSOT. 실 운영 (`upgrade head`) 자리.

2) CLI standalone — `alembic revision --autogenerate -m "..."` /
   `alembic upgrade head` 등 CLI 실행 자리. connection 자리 주입자 없으면
   scratch SQLite (`.alembic_autogen.db`, gitignored) 자리 fallback — ORM
   metadata 와 schema diff 떠서 새 revision emit 용. *운영 DB 아님*.

   ※ Postgres 특화 기능 (JSONB / UUID / GIN index / partial expression /
   ENUM type) 자리 쓰기 시작하면 scratch SQLite 자리는 metadata 자리 부족 자리
   부정확한 diff 자리 emit 할 수 있음. 그때는 CLI 자리도 실 Postgres 에 붙는
   방식 자리 (예: `ALEMBIC_DB_URL=postgresql://... alembic revision ...`)
   필요. 지금은 ORM 이 dialect-portable types 만 쓰므로 scratch SQLite 자리
   충분.

context 옵션:
- `render_as_batch` — SQLite ALTER TABLE 제약 회피 (table 재생성 패턴).
  *dialect 자리 SQLite 인 경우만* True. Postgres 자리는 native ALTER 자리
  지원 → batch wrapping 자체 자리 verbose 자리 (한 column 추가 자리도 `with
  batch_alter_table` 블록 자리 emit). dialect-aware 자리 분기로 SQLite=batch,
  Postgres=native.
- `compare_type` / `compare_server_default` — autogenerate 가 column type /
  default 변경도 잡게 함 (기본 OFF, 정석 자리는 True). 양쪽 dialect 자리 모두
  portable.
"""

from __future__ import annotations

import sys
from pathlib import Path

from alembic import context
from sqlalchemy import engine_from_config, pool
from sqlalchemy.engine import Connection

# backend/ 가 sys.path 에 들어와야 modules.* 가 import 가능. 보통 main.py /
# pytest 가 이미 처리하지만, alembic CLI 단독 실행 자리 안전망.
BACKEND_ROOT = Path(__file__).resolve().parents[1]
if str(BACKEND_ROOT) not in sys.path:
    sys.path.insert(0, str(BACKEND_ROOT))

from modules.storage.rdb.base import Base  # noqa: E402

# ORM 모델 import — Base.metadata 가 모든 테이블 수집하게 트리거.
# entity 추가 시 본 파일에 import 한 줄 추가.
import modules.calibration.orm  # noqa: E402, F401
import modules.scan_workflow.orm  # noqa: E402, F401


config = context.config

# Logging 은 host 가 소유 — alembic.ini 의 [loggers] 섹션 자체를 비웠음.
# 정석 (Alembic embedding pattern): logging.basicConfig 는 main.py (programmatic
# 자리) 또는 사용자 환경 (CLI 자리) SSOT, env.py 는 손 X.

target_metadata = Base.metadata


def _run_with_connection(connection: Connection) -> None:
    # SQLite 만 batch (ALTER TABLE 제약 회피). Postgres 등은 native ALTER.
    is_sqlite = connection.dialect.name == "sqlite"
    context.configure(
        connection=connection,
        target_metadata=target_metadata,
        render_as_batch=is_sqlite,
        compare_type=True,
        compare_server_default=True,
    )
    with context.begin_transaction():
        context.run_migrations()


def run_migrations_online() -> None:
    """Programmatic (StorageRegistry) 우선, fallback 으로 CLI standalone."""
    connection = config.attributes.get("connection", None)
    if connection is not None:
        _run_with_connection(connection)
        return

    # CLI standalone — scratch SQLite 자리. cwd 무관하게 BACKEND_ROOT 기준
    # 절대 경로 override (alembic.ini::sqlalchemy.url 의 placeholder 덮어쓰기).
    cfg_section = config.get_section(config.config_ini_section, {}) or {}
    scratch_db = BACKEND_ROOT / ".alembic_autogen.db"
    cfg_section["sqlalchemy.url"] = f"sqlite:///{scratch_db.as_posix()}"
    engine = engine_from_config(
        cfg_section,
        prefix="sqlalchemy.",
        poolclass=pool.NullPool,
    )
    with engine.connect() as conn:
        _run_with_connection(conn)


run_migrations_online()
