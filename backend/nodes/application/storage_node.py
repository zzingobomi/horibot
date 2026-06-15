"""StorageNode — Zenoh service gateway (DB/blob store 격리).

bridge_node 가 브라우저 ↔ Zenoh 통로듯, storage_node 는 다른 노드 ↔ DB/blob
store 통로. 다른 노드는 SQL 도 S3 도 모름. 호스트당 1 인스턴스 (PC만).

docs/storage_layer.md §2 architecture / §7 노드 측 패턴 / §11 책임 경계.

Phase 1 — 캘 4 service + 1 invalidation topic:
- STORAGE_GET_ACTIVE_CALIBRATION / LIST / COMMIT / ACTIVATE
- STORAGE_CALIBRATION_INVALIDATED (ACTIVATE 후 1회)
"""

from __future__ import annotations

import logging
import time

from core.transport.application_node import ApplicationNode
from core.transport.messages.base import ServiceRequest, ServiceResponse
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
from modules.storage.registry import StorageRegistry

logger = logging.getLogger(__name__)


class StorageNode(ApplicationNode):
    def __init__(self) -> None:
        super().__init__("storage_node")
        # init() 은 main.py 가 host yaml 의 storage URI 로 이미 호출. 미초기화면
        # 여기서 RuntimeError 라 즉시 fail-fast.
        self._reg = StorageRegistry.get()

    def start(self) -> None:
        self.create_service(
            Service.STORAGE_GET_ACTIVE_CALIBRATION,
            StorageGetActiveReq,
            StorageGetActiveRes,
            self._srv_get_active,
        )
        self.create_service(
            Service.STORAGE_LIST_CALIBRATIONS,
            StorageListReq,
            StorageListRes,
            self._srv_list,
        )
        self.create_service(
            Service.STORAGE_COMMIT_CALIBRATION,
            StorageCommitReq,
            StorageCommitRes,
            self._srv_commit,
        )
        self.create_service(
            Service.STORAGE_ACTIVATE_CALIBRATION,
            StorageActivateReq,
            StorageActivateRes,
            self._srv_activate,
        )
        super().start()
        logger.info("StorageNode 시작 — 캘 4 service 등록 (Phase 1)")

    def stop(self) -> None:
        super().stop()
        # SqliteStore 의 connection 명시 close. MemoryRdbStore 등은 close() 없을
        # 수 있어 안전 가드.
        close_fn = getattr(self._reg.rdb, "close", None)
        if callable(close_fn):
            close_fn()

    # ─── service handlers ─────────────────────────────────────

    def _srv_get_active(
        self, req: ServiceRequest[StorageGetActiveReq]
    ) -> ServiceResponse[StorageGetActiveRes]:
        record = self._reg.rdb.get_active_result(req.data.robot_id, req.data.kind)
        return ServiceResponse(
            success=True,
            data=StorageGetActiveRes(
                found=record is not None, result=record
            ),
        )

    def _srv_list(
        self, req: ServiceRequest[StorageListReq]
    ) -> ServiceResponse[StorageListRes]:
        records = self._reg.rdb.list_results(
            req.data.robot_id, req.data.kind, req.data.limit
        )
        return ServiceResponse(
            success=True, data=StorageListRes(results=records)
        )

    def _srv_commit(
        self, req: ServiceRequest[StorageCommitReq]
    ) -> ServiceResponse[StorageCommitRes]:
        run_id, result_ids = self._reg.rdb.commit_calibration(
            req.data.run, req.data.results, req.data.captures
        )
        logger.info(
            "COMMIT: run_id=%d, result_ids=%s (robot=%s, results=%d, captures=%d)",
            run_id,
            result_ids,
            req.data.run.robot_id,
            len(req.data.results),
            len(req.data.captures),
        )
        return ServiceResponse(
            success=True,
            data=StorageCommitRes(run_id=run_id, result_ids=result_ids),
        )

    def _srv_activate(
        self, req: ServiceRequest[StorageActivateReq]
    ) -> ServiceResponse[StorageActivateRes]:
        try:
            activated = self._reg.rdb.activate_result(req.data.result_id)
        except KeyError as e:
            return ServiceResponse(success=False, message=str(e))

        # transaction commit 직후 invalidation publish — caller (calibration_node)
        # service 응답 받기 전에도 노드들이 refetch 시작 가능.
        assert activated.id is not None  # activate_result 반환은 id 항상 있음
        self.publish(
            Topic.STORAGE_CALIBRATION_INVALIDATED,
            CalibrationInvalidated(
                robot_id=activated.robot_id,
                kind=activated.kind,
                result_id=activated.id,
                timestamp=time.time(),
            ),
        )
        logger.info(
            "ACTIVATE: result_id=%d (robot=%s, kind=%s) — invalidation publish",
            activated.id,
            activated.robot_id,
            activated.kind,
        )
        return ServiceResponse(
            success=True, data=StorageActivateRes(result=activated)
        )
