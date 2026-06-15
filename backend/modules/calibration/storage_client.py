"""CalibrationStorageClient — 캘 entity 의 storage service wrapping.

`modules/storage/transport.py` 의 generic `StorageTransport` 위에 캘 4 service +
invalidation subscribe 를 typed 으로 노출.

미래 entity (Scan / TSDF / TaskRun) 들도 같은 transport 위에 own `*StorageClient`.
storage 모듈은 entity 어휘 0 — 본 파일이 캘 ↔ storage 다리.

docs/storage_layer.md §7 노드 측 패턴 — 부팅 시 fetch + spill fallback + invalidation
구독. 본 파일이 그 client side.
"""

from __future__ import annotations

import logging
import time
from typing import Callable

import zenoh

from core.transport.messages.storage import (
    CalibrationInvalidated,
    StorageActivateReq,
    StorageActivateRes,
    StorageCommitReq,
    StorageCommitRes,
    StorageGetActiveReq,
    StorageGetActiveRes,
    StorageListReq,
    StorageListRes,
)
from core.transport.topic_map import Service, Topic
from modules.calibration.persistence_models import (
    CalibrationCaptureRecord,
    CalibrationKind,
    CalibrationResultRecord,
    CalibrationRunRecord,
)
from modules.storage.transport import StorageTransport, StorageUnavailable

logger = logging.getLogger(__name__)


class CalibrationStorageClient:
    def __init__(self, transport: StorageTransport | None = None):
        self._t = transport or StorageTransport()

    # ─── read ────────────────────────────────────────────────

    def get_active(
        self, robot_id: str, kind: CalibrationKind
    ) -> CalibrationResultRecord | None:
        res = self._t.call(
            Service.STORAGE_GET_ACTIVE_CALIBRATION,
            StorageGetActiveReq(robot_id=robot_id, kind=kind),
            StorageGetActiveRes,
        )
        return res.result if res.found else None

    def list(
        self, robot_id: str, kind: CalibrationKind, limit: int = 100
    ) -> list[CalibrationResultRecord]:
        res = self._t.call(
            Service.STORAGE_LIST_CALIBRATIONS,
            StorageListReq(robot_id=robot_id, kind=kind, limit=limit),
            StorageListRes,
        )
        return res.results

    # ─── write ───────────────────────────────────────────────

    def commit(
        self,
        run: CalibrationRunRecord,
        results: list[CalibrationResultRecord],
        captures: list[CalibrationCaptureRecord] | None = None,
    ) -> tuple[int, list[int]]:
        res = self._t.call(
            Service.STORAGE_COMMIT_CALIBRATION,
            StorageCommitReq(run=run, results=results, captures=captures or []),
            StorageCommitRes,
        )
        return res.run_id, res.result_ids

    def activate(self, result_id: int) -> CalibrationResultRecord:
        res = self._t.call(
            Service.STORAGE_ACTIVATE_CALIBRATION,
            StorageActivateReq(result_id=result_id),
            StorageActivateRes,
        )
        return res.result

    # ─── invalidation subscribe ──────────────────────────────

    def subscribe_invalidations(
        self, callback: Callable[[CalibrationInvalidated], None]
    ) -> zenoh.Subscriber:
        """ACTIVATE 마다 1회 publish 되는 topic 구독. caller 가 callback 안에서
        자기 robot_id 만 filter + cache refetch. Subscriber undeclare 책임 = caller.
        """
        return self._t.subscribe_topic(
            Topic.STORAGE_CALIBRATION_INVALIDATED,
            CalibrationInvalidated,
            callback,
        )


# ─── convenience — 서비스 대기 패턴 ──────────────────────


def load_active_blocking(
    robot_id: str,
    kind: CalibrationKind,
    retry_interval: float = 1.0,
    max_wait: float | None = None,
) -> CalibrationResultRecord | None:
    """Storage 응답 받을 때까지 대기. docs/storage_layer.md §7 — Storage 필수 가정.

    - storage 응답 OK + found=True   → record
    - storage 응답 OK + found=False  → None (첫 부팅 robot — caller 는 default 캘)
    - storage timeout / unreachable  → retry_interval 후 재시도 (무한, max_wait 지정 시 cap)

    cache-first 가 아니라 SSOT-first. spill / version / conflict 자리 없음.
    """
    start = time.monotonic()
    while True:
        try:
            return CalibrationStorageClient().get_active(robot_id, kind)
        except StorageUnavailable as e:
            elapsed = time.monotonic() - start
            if max_wait is not None and elapsed >= max_wait:
                raise
            logger.info(
                "storage 대기 중 (%s[%s], elapsed=%.0fs): %s",
                kind,
                robot_id,
                elapsed,
                e,
            )
            time.sleep(retry_interval)
