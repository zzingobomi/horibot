"""CalibrationStorageClient — 캘 entity 의 storage service wrapping.

`modules/storage/transport.py` 의 generic `StorageTransport` 위에 캘 4 service +
invalidation subscribe 를 typed 으로 노출.

미래 entity (Scan / TSDF / TaskRun) 들도 같은 transport 위에 own `*StorageClient`.
storage 모듈은 entity 어휘 0 — 본 파일이 캘 ↔ storage 다리.

docs/storage_layer.md §7 노드 측 패턴 — 부팅 시 fetch + spill fallback + invalidation
구독. 본 파일이 그 client side.
"""

from __future__ import annotations

import base64
import logging
import time
from typing import Callable

import zenoh

from core.transport.messages.storage import (
    ActivateCalibrationReq,
    ActivateCalibrationRes,
    AppendCalibrationCaptureReq,
    AppendCalibrationCaptureRes,
    CalibrationInvalidated,
    CalibrationRunSummary,
    CommitCalibrationReq,
    CommitCalibrationRes,
    CreateCalibrationRunReq,
    CreateCalibrationRunRes,
    DeleteCalibrationRunReq,
    DeleteLastCalibrationCaptureReq,
    DeleteLastCalibrationCaptureRes,
    FinalizeCalibrationRunReq,
    FinalizeCalibrationRunRes,
    GetActiveCalibrationReq,
    GetActiveCalibrationRes,
    GetInProgressCalibrationRunReq,
    GetInProgressCalibrationRunRes,
    ListCalibrationRunsReq,
    ListCalibrationRunsRes,
    ListCalibrationsReq,
    ListCalibrationsRes,
    MarkCalibrationRunReadyReq,
    MarkCalibrationRunReadyRes,
)
from core.transport.messages.base import EmptyData
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
            GetActiveCalibrationReq(robot_id=robot_id, kind=kind),
            GetActiveCalibrationRes,
        )
        return res.result if res.found else None

    def list(
        self, robot_id: str, kind: CalibrationKind, limit: int = 100
    ) -> list[CalibrationResultRecord]:
        res = self._t.call(
            Service.STORAGE_LIST_CALIBRATIONS,
            ListCalibrationsReq(robot_id=robot_id, kind=kind, limit=limit),
            ListCalibrationsRes,
        )
        return res.results

    def list_runs(
        self, robot_id: str, limit: int = 50
    ) -> list[CalibrationRunSummary]:
        """Run 단위 history — frontend list/ACTIVATE 패널이 사용. 한 Run 의
        모든 kind Result 가 묶여 옴 (storage_layer.md Stage 4 design A)."""
        res = self._t.call(
            Service.STORAGE_LIST_CALIBRATION_RUNS,
            ListCalibrationRunsReq(robot_id=robot_id, limit=limit),
            ListCalibrationRunsRes,
        )
        return res.runs

    # ─── write ───────────────────────────────────────────────

    def commit(
        self,
        run: CalibrationRunRecord,
        results: list[CalibrationResultRecord],
        captures: list[CalibrationCaptureRecord] | None = None,
    ) -> tuple[int, list[int]]:
        res = self._t.call(
            Service.STORAGE_COMMIT_CALIBRATION,
            CommitCalibrationReq(run=run, results=results, captures=captures or []),
            CommitCalibrationRes,
        )
        return res.run_id, res.result_ids

    def activate(self, result_id: int) -> CalibrationResultRecord:
        res = self._t.call(
            Service.STORAGE_ACTIVATE_CALIBRATION,
            ActivateCalibrationReq(result_id=result_id),
            ActivateCalibrationRes,
        )
        return res.result

    # ─── Draft run / capture-as-you-go (사용자 flow) ─────────

    def new_run(self, run: CalibrationRunRecord) -> int:
        """[캘 시작] — in_progress run 생성. run.kind 채워야 함."""
        res = self._t.call(
            Service.STORAGE_NEW_CAL_RUN,
            CreateCalibrationRunReq(run=run),
            CreateCalibrationRunRes,
        )
        return res.run_id

    def append_capture(
        self,
        capture: CalibrationCaptureRecord,
        *,
        robot_id: str,
        blob_bytes: bytes = b"",
    ) -> tuple[int, str | None]:
        """[캡처] — draft run 에 capture 1장 append + ObjectStore blob (color+depth).

        `blob_bytes` 가 비어있으면 ObjectStore.put 안 함 (intrinsic 캡처 등). hand_eye
        는 `depth_frame.py` 의 encode 결과 (color JPEG + zstd Z16 depth) 를 그대로
        넘기면 server 가 blob_key 부여 + put. caller 의 capture.blob_key 는 무시.

        반환 (capture_id, blob_key) — blob_key 는 저장된 경우만 채워짐.

        주의: Pydantic `Base64Bytes` 자리 raw bytes input 을 base64-DECODE 시도해서
        조용히 손상시킨다 (binary 자리 base64 char 자리 자리 자리 자리 drop 자리).
        caller 자리 미리 base64-encode 한 str 자리 자리 넘겨야 round-trip 됨.
        """
        # Pydantic Base64Bytes 자리 input 자리 base64-encoded string 자리 자리. raw bytes
        # 자리 넘기면 silent corruption (~99% byte 손실).
        blob_b64 = (
            base64.b64encode(blob_bytes).decode("ascii") if blob_bytes else ""
        )
        res = self._t.call(
            Service.STORAGE_APPEND_CAPTURE,
            AppendCalibrationCaptureReq(
                capture=capture, blob_bytes=blob_b64, robot_id=robot_id  # type: ignore[arg-type]
            ),
            AppendCalibrationCaptureRes,
        )
        return res.capture_id, res.blob_key

    def delete_last_capture(self, run_id: int) -> int | None:
        """[되돌리기] — 마지막 capture 1장 삭제. 삭제된 pose_index, 없으면 None."""
        res = self._t.call(
            Service.STORAGE_DELETE_LAST_CAPTURE,
            DeleteLastCalibrationCaptureReq(run_id=run_id),
            DeleteLastCalibrationCaptureRes,
        )
        return res.deleted_pose_index

    def get_in_progress_run(
        self, robot_id: str, kind: CalibrationKind
    ) -> tuple[CalibrationRunRecord, list[CalibrationCaptureRecord]] | None:
        """부팅 시 복원 — 진행 중이던 세션. 없으면 None."""
        res = self._t.call(
            Service.STORAGE_GET_IN_PROGRESS_RUN,
            GetInProgressCalibrationRunReq(robot_id=robot_id, kind=kind),
            GetInProgressCalibrationRunRes,
        )
        if not res.found or res.run is None:
            return None
        return res.run, res.captures

    def delete_run(self, run_id: int) -> None:
        """[리셋] — run + captures + results cascade delete + blob 도 같이."""
        self._t.call(
            Service.STORAGE_DELETE_CAL_RUN,
            DeleteCalibrationRunReq(run_id=run_id),
            EmptyData,
        )

    def mark_run_ready(self, run_id: int) -> CalibrationRunRecord:
        """[세션 종료] — in_progress → ready_for_analysis. 캡처 immutable 화."""
        res = self._t.call(
            Service.STORAGE_MARK_CAL_RUN_READY,
            MarkCalibrationRunReadyReq(run_id=run_id),
            MarkCalibrationRunReadyRes,
        )
        return res.run

    def finalize_run(
        self,
        run_id: int,
        results: list[CalibrationResultRecord],
        capture_residuals: dict[int, tuple[float | None, float | None, float | None]]
        | None = None,
    ) -> list[int]:
        """[커밋] — in_progress → success, result rows INSERT, captures residual UPDATE."""
        res = self._t.call(
            Service.STORAGE_FINALIZE_CAL_RUN,
            FinalizeCalibrationRunReq(
                run_id=run_id,
                results=results,
                capture_residuals=capture_residuals,
            ),
            FinalizeCalibrationRunRes,
        )
        return res.result_ids

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
