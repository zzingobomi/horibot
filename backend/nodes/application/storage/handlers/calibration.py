"""Calibration 도메인 Zenoh service handler group.

storage_node 의 lifecycle + composition root 안 register(node) 자리 호출되어
service handler 들을 node 에 등록. 본 group 의 책임 = 캘리브레이션 도메인
(run / result / capture / activate / draft flow / finalize) 의 Zenoh 노출.

11 service:
- GET_ACTIVE / LIST / LIST_RUNS / COMMIT / ACTIVATE
- NEW_CAL_RUN / APPEND_CAPTURE / DELETE_LAST_CAPTURE
- GET_IN_PROGRESS_RUN / DELETE_CAL_RUN / FINALIZE_CAL_RUN
+ STORAGE_CALIBRATION_INVALIDATED topic publish (ACTIVATE 후 1회)
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from typing import Any

from core.transport.application_node import ApplicationNode
from core.transport.messages.base import EmptyData, ServiceRequest, ServiceResponse
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
)
from core.transport.topic_map import Service, Topic
from modules.storage.registry import StorageRegistry

logger = logging.getLogger(__name__)


class CalibrationHandlers:
    """캘리브레이션 도메인 service handler 묶음."""

    def __init__(self, reg: StorageRegistry, publish: Callable[[str, Any], None]) -> None:
        self._reg = reg
        self._publish = publish

    def register(self, node: ApplicationNode) -> None:
        node.create_service(
            Service.STORAGE_GET_ACTIVE_CALIBRATION,
            GetActiveCalibrationReq,
            GetActiveCalibrationRes,
            self._srv_get_active,
        )
        node.create_service(
            Service.STORAGE_LIST_CALIBRATIONS,
            ListCalibrationsReq,
            ListCalibrationsRes,
            self._srv_list,
        )
        node.create_service(
            Service.STORAGE_LIST_CALIBRATION_RUNS,
            ListCalibrationRunsReq,
            ListCalibrationRunsRes,
            self._srv_list_runs,
        )
        node.create_service(
            Service.STORAGE_COMMIT_CALIBRATION,
            CommitCalibrationReq,
            CommitCalibrationRes,
            self._srv_commit,
        )
        node.create_service(
            Service.STORAGE_ACTIVATE_CALIBRATION,
            ActivateCalibrationReq,
            ActivateCalibrationRes,
            self._srv_activate,
        )
        node.create_service(
            Service.STORAGE_NEW_CAL_RUN,
            CreateCalibrationRunReq,
            CreateCalibrationRunRes,
            self._srv_new_cal_run,
        )
        node.create_service(
            Service.STORAGE_APPEND_CAPTURE,
            AppendCalibrationCaptureReq,
            AppendCalibrationCaptureRes,
            self._srv_append_capture,
        )
        node.create_service(
            Service.STORAGE_DELETE_LAST_CAPTURE,
            DeleteLastCalibrationCaptureReq,
            DeleteLastCalibrationCaptureRes,
            self._srv_delete_last_capture,
        )
        node.create_service(
            Service.STORAGE_GET_IN_PROGRESS_RUN,
            GetInProgressCalibrationRunReq,
            GetInProgressCalibrationRunRes,
            self._srv_get_in_progress_run,
        )
        node.create_service(
            Service.STORAGE_DELETE_CAL_RUN,
            DeleteCalibrationRunReq,
            EmptyData,
            self._srv_delete_cal_run,
        )
        node.create_service(
            Service.STORAGE_FINALIZE_CAL_RUN,
            FinalizeCalibrationRunReq,
            FinalizeCalibrationRunRes,
            self._srv_finalize_cal_run,
        )

    # ─── service handlers ─────────────────────────────────────

    def _srv_get_active(
        self, req: ServiceRequest[GetActiveCalibrationReq]
    ) -> ServiceResponse[GetActiveCalibrationRes]:
        with self._reg.rdb.session() as repos:
            record = repos.calibration.get_active_result(
                req.data.robot_id, req.data.kind
            )
        return ServiceResponse(
            success=True,
            data=GetActiveCalibrationRes(
                found=record is not None, result=record
            ),
        )

    def _srv_list(
        self, req: ServiceRequest[ListCalibrationsReq]
    ) -> ServiceResponse[ListCalibrationsRes]:
        with self._reg.rdb.session() as repos:
            records = repos.calibration.list_results(
                req.data.robot_id, req.data.kind, req.data.limit
            )
        return ServiceResponse(success=True, data=ListCalibrationsRes(results=records))

    def _srv_list_runs(
        self, req: ServiceRequest[ListCalibrationRunsReq]
    ) -> ServiceResponse[ListCalibrationRunsRes]:
        with self._reg.rdb.session() as repos:
            rows = repos.calibration.list_runs(req.data.robot_id, req.data.limit)
        summaries = [
            CalibrationRunSummary(run=run, results=results) for run, results in rows
        ]
        return ServiceResponse(
            success=True, data=ListCalibrationRunsRes(runs=summaries)
        )

    def _srv_commit(
        self, req: ServiceRequest[CommitCalibrationReq]
    ) -> ServiceResponse[CommitCalibrationRes]:
        with self._reg.rdb.session() as repos:
            run_id, result_ids = repos.calibration.commit(
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
            data=CommitCalibrationRes(run_id=run_id, result_ids=result_ids),
        )

    def _srv_activate(
        self, req: ServiceRequest[ActivateCalibrationReq]
    ) -> ServiceResponse[ActivateCalibrationRes]:
        try:
            with self._reg.rdb.session() as repos:
                activated = repos.calibration.activate_result(req.data.result_id)
        except KeyError as e:
            return ServiceResponse(success=False, message=str(e))

        # transaction commit 직후 invalidation publish — caller (calibration_node)
        # service 응답 받기 전에도 노드들이 refetch 시작 가능.
        assert activated.id is not None  # activate_result 반환은 id 항상 있음
        self._publish(
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
            success=True, data=ActivateCalibrationRes(result=activated)
        )

    # ─── Draft run handlers (사용자 [캘 시작] flow) ───────────

    def _srv_new_cal_run(
        self, req: ServiceRequest[CreateCalibrationRunReq]
    ) -> ServiceResponse[CreateCalibrationRunRes]:
        run = req.data.run
        if run.kind is None:
            return ServiceResponse(
                success=False, message="run.kind 필수 (intrinsic / hand_eye)"
            )
        with self._reg.rdb.session() as repos:
            # 같은 (robot_id, kind) 의 기존 in_progress 가 있으면 거부 — frontend 는
            # 먼저 GET_IN_PROGRESS 로 확인해야 함.
            existing = repos.calibration.get_in_progress_run(run.robot_id, run.kind)
            if existing is not None:
                existing_run, _ = existing
                return ServiceResponse(
                    success=False,
                    message=(
                        f"이미 in_progress run 있음 (robot={run.robot_id}, "
                        f"kind={run.kind}, run_id={existing_run.id})"
                    ),
                )
            run_id = repos.calibration.new_run(run)
        logger.info(
            "NEW_CAL_RUN: run_id=%d (robot=%s, kind=%s, algorithm=%s)",
            run_id, run.robot_id, run.kind, run.algorithm,
        )
        return ServiceResponse(success=True, data=CreateCalibrationRunRes(run_id=run_id))

    def _srv_append_capture(
        self, req: ServiceRequest[AppendCalibrationCaptureReq]
    ) -> ServiceResponse[AppendCalibrationCaptureRes]:
        capture = req.data.capture
        with self._reg.rdb.session() as repos:
            capture_id = repos.calibration.append_capture(capture)
        return ServiceResponse(
            success=True,
            data=AppendCalibrationCaptureRes(capture_id=capture_id),
        )

    def _srv_delete_last_capture(
        self, req: ServiceRequest[DeleteLastCalibrationCaptureReq]
    ) -> ServiceResponse[DeleteLastCalibrationCaptureRes]:
        with self._reg.rdb.session() as repos:
            deleted = repos.calibration.delete_last_capture(req.data.run_id)
        return ServiceResponse(
            success=True,
            data=DeleteLastCalibrationCaptureRes(deleted_pose_index=deleted),
        )

    def _srv_get_in_progress_run(
        self, req: ServiceRequest[GetInProgressCalibrationRunReq]
    ) -> ServiceResponse[GetInProgressCalibrationRunRes]:
        with self._reg.rdb.session() as repos:
            result = repos.calibration.get_in_progress_run(
                req.data.robot_id, req.data.kind
            )
        if result is None:
            return ServiceResponse(
                success=True, data=GetInProgressCalibrationRunRes(found=False)
            )
        run, captures = result
        return ServiceResponse(
            success=True,
            data=GetInProgressCalibrationRunRes(found=True, run=run, captures=captures),
        )

    def _srv_delete_cal_run(
        self, req: ServiceRequest[DeleteCalibrationRunReq]
    ) -> ServiceResponse[EmptyData]:
        with self._reg.rdb.session() as repos:
            repos.calibration.delete_run(req.data.run_id)
        logger.info("DELETE_CAL_RUN: run_id=%d", req.data.run_id)
        return ServiceResponse(success=True, data=EmptyData())

    def _srv_finalize_cal_run(
        self, req: ServiceRequest[FinalizeCalibrationRunReq]
    ) -> ServiceResponse[FinalizeCalibrationRunRes]:
        try:
            with self._reg.rdb.session() as repos:
                result_ids = repos.calibration.finalize_run(
                    req.data.run_id, req.data.results, req.data.capture_residuals
                )
        except KeyError as e:
            return ServiceResponse(success=False, message=str(e))
        logger.info(
            "FINALIZE_CAL_RUN: run_id=%d, result_ids=%s",
            req.data.run_id, result_ids,
        )
        return ServiceResponse(
            success=True, data=FinalizeCalibrationRunRes(result_ids=result_ids),
        )
