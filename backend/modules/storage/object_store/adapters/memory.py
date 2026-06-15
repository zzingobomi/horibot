"""MemoryObjectStore — host_mock + 테스트. 프로세스 종료 시 사라짐."""

from __future__ import annotations

import threading


class MemoryObjectStore:
    def __init__(self) -> None:
        self._data: dict[str, bytes] = {}
        self._lock = threading.Lock()

    def put(self, key: str, data: bytes) -> None:
        with self._lock:
            self._data[key] = data

    def get(self, key: str) -> bytes:
        with self._lock:
            try:
                return self._data[key]
            except KeyError:
                raise KeyError(f"key 없음: {key!r}") from None

    def delete(self, key: str) -> None:
        with self._lock:
            self._data.pop(key, None)

    def list(self, prefix: str) -> list[str]:
        with self._lock:
            return sorted(k for k in self._data if k.startswith(prefix))
