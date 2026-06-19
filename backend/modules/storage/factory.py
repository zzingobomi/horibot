from __future__ import annotations

import logging
from pathlib import Path

from modules.storage.object_store.store import ObjectStore
from modules.storage.rdb.base import make_engine
from modules.storage.rdb.store import RdbStore

logger = logging.getLogger(__name__)


def make_rdb_store(uri: str) -> RdbStore:
    if uri.startswith("sqlite:///") and uri != "sqlite:///:memory:":
        Path(uri[len("sqlite:///") :]).parent.mkdir(parents=True, exist_ok=True)
    return RdbStore(make_engine(uri))


def make_object_store(uri: str) -> ObjectStore:
    if uri == "memory://":
        from modules.storage.object_store.adapters.memory import MemoryObjectStore

        return MemoryObjectStore()
    if uri.startswith("file:///"):
        from modules.storage.object_store.adapters.filesystem import (
            FilesystemObjectStore,
        )

        root = Path(uri[len("file:///") :])
        root.mkdir(parents=True, exist_ok=True)
        return FilesystemObjectStore(root)
    raise ValueError(
        f"unknown object URI scheme: {uri!r}. 지원: memory:// | file:///<path>"
    )
