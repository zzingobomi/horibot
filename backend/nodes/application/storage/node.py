"""StorageNode — Zenoh service gateway (DB/blob store 격리).

bridge_node 가 브라우저 ↔ Zenoh 통로듯, storage_node 는 다른 노드 ↔ DB/blob
store 통로. 다른 노드는 SQL 도 S3 도 모름. 호스트당 1 인스턴스 (PC만).

본 파일 = lifecycle + composition root. 실제 service handler 들은 도메인별로
[handlers/calibration.py](handlers/calibration.py) (캘) + [handlers/scan_workflow.py](handlers/scan_workflow.py)
(스캔) 에 분리. 새 도메인 (task_runs / pose_library / cross_calibration) 추가
자리 = handlers/ 에 새 파일 + 본 파일 `__init__` / `start()` 두 줄 추가.

docs/storage_layer.md §2 architecture / §7 노드 측 패턴 / §11 책임 경계.
"""

from __future__ import annotations

import logging

from core.transport.application_node import ApplicationNode
from modules.storage.registry import StorageRegistry
from nodes.application.storage.handlers.calibration import CalibrationHandlers
from nodes.application.storage.handlers.scan_workflow import ScanWorkflowHandlers

logger = logging.getLogger(__name__)


class StorageNode(ApplicationNode):
    def __init__(self) -> None:
        super().__init__("storage_node")
        # init() 은 main.py 가 host yaml 의 storage URI 로 이미 호출. 미초기화면
        # 여기서 RuntimeError 라 즉시 fail-fast.
        self._reg = StorageRegistry.get()
        self._calibration = CalibrationHandlers(self._reg, self.publish)
        self._scan_workflow = ScanWorkflowHandlers(self._reg, self.publish)

    def start(self) -> None:
        self._calibration.register(self)
        self._scan_workflow.register(self)
        super().start()
        logger.info(
            "StorageNode 시작 — 캘 11 service + scan workflow 10 service"
        )

    def stop(self) -> None:
        super().stop()
        # RdbStore.close() = engine.dispose() — connection pool 비움. close() 없는
        # 구현체 (테스트 mock 등) 안전 가드.
        close_fn = getattr(self._reg.rdb, "close", None)
        if callable(close_fn):
            close_fn()
