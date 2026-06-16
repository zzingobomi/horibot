"""URI → RdbStore / ObjectStore 인스턴스 (MLflow 식 factory).

backend swap (sqlite → postgres, file → s3) = adapter 파일 추가 + host yaml URI
교체. caller 코드 변경 X.

지원 scheme (Phase 1):
  RdbStore:    memory://  /  sqlite:///<path>
  ObjectStore: memory://  /  file:///<path>

Path placeholder 지원 — URI 안에 `${PROJECT_ROOT}` 박으면 project root 로 치환.
git tracked DB / 공유 자원이 user home 대신 repo 안에 살게 하는 자리.

Phase 3 추가 자리:
  RdbStore:    postgresql://user@host:port/db
  ObjectStore: s3://endpoint/bucket
"""

from __future__ import annotations

from pathlib import Path

from modules.storage.object_store.store import ObjectStore
from modules.storage.rdb.store import RdbStore

# project root — backend/modules/storage/factory.py 기준 3 parents 위. `${PROJECT_ROOT}`
# placeholder 가 yaml URI 안에서 본 경로로 치환됨.
PROJECT_ROOT = Path(__file__).resolve().parents[3]


def _expand(uri_path: str) -> Path:
    """URI path 부분의 placeholder + `~` expand.

    - `${PROJECT_ROOT}` → 절대 경로 (repo root)
    - `~`              → user home (Windows = `C:\\Users\\<user>`, Linux = `/home/<user>`)
    """
    expanded = uri_path.replace("${PROJECT_ROOT}", str(PROJECT_ROOT))
    return Path(expanded).expanduser()


def make_rdb_store(uri: str) -> RdbStore:
    if uri == "memory://":
        from modules.storage.rdb.adapters.memory import MemoryRdbStore
        return MemoryRdbStore()
    if uri.startswith("sqlite:///"):
        from modules.storage.rdb.adapters.sqlite import SqliteStore
        # sqlite:///~/.local/horibot/storage.db → ~/.local/horibot/storage.db
        path = _expand(uri[len("sqlite:///") :])
        path.parent.mkdir(parents=True, exist_ok=True)
        return SqliteStore(path)
    raise ValueError(
        f"unknown rdb URI scheme: {uri!r}. 지원: memory:// | sqlite:///<path>"
    )


def make_object_store(uri: str) -> ObjectStore:
    if uri == "memory://":
        from modules.storage.object_store.adapters.memory import MemoryObjectStore
        return MemoryObjectStore()
    if uri.startswith("file:///"):
        from modules.storage.object_store.adapters.filesystem import (
            FilesystemObjectStore,
        )
        root = _expand(uri[len("file:///") :])
        root.mkdir(parents=True, exist_ok=True)
        return FilesystemObjectStore(root)
    raise ValueError(
        f"unknown object URI scheme: {uri!r}. 지원: memory:// | file:///<path>"
    )
