from __future__ import annotations

import os
from pathlib import Path


class FilesystemObjectStore:
    def __init__(self, base_dir: str | Path):
        self._base = Path(base_dir).resolve()
        self._base.mkdir(parents=True, exist_ok=True)

    def put(self, key: str, data: bytes) -> None:
        path = self._resolve(key)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_bytes(data)
        os.replace(tmp, path)

    def get(self, key: str) -> bytes:
        path = self._resolve(key)
        if not path.is_file():
            raise KeyError(key)
        return path.read_bytes()

    def delete(self, key: str) -> None:
        path = self._resolve(key)
        if not path.is_file():
            raise KeyError(key)
        path.unlink()

    def list(self, prefix: str) -> list[str]:
        root = self._resolve(prefix) if prefix else self._base
        keys: list[str] = []
        if root.is_dir():
            for p in root.rglob("*"):
                if p.is_file() and p.suffix != ".tmp":
                    keys.append(p.relative_to(self._base).as_posix())
        elif prefix:
            # prefix  dir  아닌  — file  직접 prefix  박은
            parent = root.parent
            stem = root.name
            if parent.is_dir():
                for p in parent.iterdir():
                    if p.is_file() and p.name.startswith(stem) and p.suffix != ".tmp":
                        keys.append(p.relative_to(self._base).as_posix())
        return sorted(keys)

    # ─── internal ────────────────────────────────────────

    def _resolve(self, key: str) -> Path:
        """key  base_dir 안 path  resolve — escape (`..`) 차단."""
        # Path  join  normalize 박은  base  escape  차단.
        # `key`  leading `/` 박은  박지 X — relative  강제.
        clean = key.lstrip("/").replace("\\", "/")
        resolved = (self._base / clean).resolve()
        try:
            resolved.relative_to(self._base)
        except ValueError as e:
            raise ValueError(f"key {key!r}  base_dir 밖  escape 박음") from e
        return resolved
