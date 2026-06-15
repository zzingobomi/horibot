"""FilesystemObjectStore — Phase 2 entity (scans/meshes) 의 blob backend.

Phase 1 에선 사용처 없음 — Phase 2 진입 시 ObjectStore Protocol method 가
처음 실 호출됨. 미리 만들어두는 이유: storage_node 의 host yaml 에 `object_uri`
가 항상 있어야 정합 (memory:// vs file:///), Phase 2 진입 시 adapter 추가 X.
"""

from __future__ import annotations

from pathlib import Path


class FilesystemObjectStore:
    def __init__(self, root: Path):
        self._root = root
        self._root.mkdir(parents=True, exist_ok=True)

    def _resolve(self, key: str) -> Path:
        # 안전: key 가 절대경로 / `..` 포함 시 root 밖으로 나가는 것 차단.
        p = (self._root / key).resolve()
        if self._root.resolve() not in p.parents and p != self._root.resolve():
            raise ValueError(f"key 가 root 밖: {key!r}")
        return p

    def put(self, key: str, data: bytes) -> None:
        path = self._resolve(key)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)

    def get(self, key: str) -> bytes:
        return self._resolve(key).read_bytes()

    def delete(self, key: str) -> None:
        path = self._resolve(key)
        if path.exists():
            path.unlink()

    def list(self, prefix: str) -> list[str]:
        prefix_path = self._resolve(prefix) if prefix else self._root
        if not prefix_path.exists():
            return []
        keys: list[str] = []
        for p in prefix_path.rglob("*"):
            if p.is_file():
                keys.append(str(p.relative_to(self._root)).replace("\\", "/"))
        return sorted(keys)
