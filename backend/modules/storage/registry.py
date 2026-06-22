from __future__ import annotations

import logging
from pathlib import Path

from alembic import command
from alembic.config import Config
from sqlalchemy import Engine

from modules.storage.factory import make_object_store, make_rdb_store
from modules.storage.object_store.store import ObjectStore
from modules.storage.rdb.store import RdbStore

logger = logging.getLogger(__name__)

_BACKEND_ROOT = Path(__file__).resolve().parents[2]
_ALEMBIC_INI = _BACKEND_ROOT / "alembic.ini"


def _ensure_schema(engine: Engine) -> None:
    config = Config(str(_ALEMBIC_INI))
    with engine.begin() as connection:
        config.attributes["connection"] = connection
        command.upgrade(config, "head")


class StorageRegistry:
    """RDB + ObjectStore 핸들 + Alembic schema migration 한 묶음.

    Process Infrastructure (외부 자원 보유 — DB connection + filesystem/S3).
    main.py 가 application_nodes 에 'storage' 있을 때 host yaml 의 URI 로
    `StorageRegistry.init()` 호출 — Alembic `upgrade head` 도 안에서 자동.
    다른 코드는 `StorageRegistry.get()` 으로 받아 사용 — 미초기화 시
    RuntimeError 로 fail-fast. Process-wide Memory State (`Foo()` 패턴) 와 대비됨.
    """

    _instance: "StorageRegistry | None" = None

    def __init__(self, rdb: RdbStore, objects: ObjectStore) -> None:
        self.rdb = rdb
        self.objects = objects

    @classmethod
    def init(cls, rdb_uri: str, object_uri: str) -> "StorageRegistry":
        if cls._instance is not None:
            logger.warning("StorageRegistry 이미 초기화됨 — 재호출 무시")
            return cls._instance
        rdb = make_rdb_store(rdb_uri)
        # Migration hook용으로 RDB store는 내부 SQLAlchemy Engine을 노출한다.
        # Alembic은 Store 종류(SQLite/Postgres)가 아닌 Engine 기준으로 동작한다.
        engine = getattr(rdb, "_engine", None)
        if engine is not None:
            _ensure_schema(engine)
        objects = make_object_store(object_uri)
        cls._instance = cls(rdb, objects)
        logger.info("StorageRegistry 초기화: rdb=%s, object=%s", rdb_uri, object_uri)
        return cls._instance

    @classmethod
    def get(cls) -> "StorageRegistry":
        if cls._instance is None:
            raise RuntimeError(
                "StorageRegistry 미초기화. main.py 가 application_nodes 에 "
                "'storage' 있을 때 init() 호출함."
            )
        return cls._instance

    @classmethod
    def _reset_for_test(cls) -> None:
        cls._instance = None
