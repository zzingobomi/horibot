"""Object store — universal 4 method Protocol (Phase 2 entity 자리).

S3 / MinIO / GCS / Azure / local fs 다 fit 하는 작은 surface. Phase 1 에서는
사용처 없음 — Phase 2 의 scans/meshes/task_runs 진입 시 실 사용 시작.

docs/storage_layer.md §8 — streaming / multipart 등은 Phase 2 진입 시 *추가*
(additive, 기존 4 method 변경 X).
"""

from __future__ import annotations

from typing import Protocol


class ObjectStore(Protocol):
    """작고 보수적인 universal interface."""

    def put(self, key: str, data: bytes) -> None: ...
    def get(self, key: str) -> bytes: ...
    def delete(self, key: str) -> None: ...
    def list(self, prefix: str) -> list[str]: ...
