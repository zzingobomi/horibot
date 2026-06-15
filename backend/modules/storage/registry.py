"""StorageRegistry — host config 의 storage URI 로 store 인스턴스 1회 빌드.

main.py 가 부팅 시 `StorageRegistry.init(rdb_uri, object_uri)` 호출 → 노드
들은 `StorageRegistry.get().rdb` / `.objects` 로 접근.

ZenohSession / RobotRegistry 와 같은 모듈 singleton 패턴. storage_node 가
PC 에만 뜨므로 분산 모드의 모터 Pi / 카메라 Pi 에선 init() 호출 안 됨 — 본
registry 도 미초기화.
"""

from __future__ import annotations

import logging

from modules.storage.factory import make_object_store, make_rdb_store
from modules.storage.object_store.store import ObjectStore
from modules.storage.rdb.store import RdbStore

logger = logging.getLogger(__name__)


class StorageRegistry:
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
        objects = make_object_store(object_uri)
        cls._instance = cls(rdb, objects)
        logger.info(
            "StorageRegistry 초기화: rdb=%s, object=%s", rdb_uri, object_uri
        )
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
        """unit test 용. production code 에서 호출 X."""
        cls._instance = None
