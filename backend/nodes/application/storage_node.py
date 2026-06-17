"""StorageNode — Zenoh service gateway (DB/blob store 격리).

bridge_node 가 브라우저 ↔ Zenoh 통로듯, storage_node 는 다른 노드 ↔ DB/blob
store 통로. 다른 노드는 SQL 도 S3 도 모름. 호스트당 1 인스턴스 (PC만).

docs/storage_layer.md §2 architecture / §7 노드 측 패턴 / §11 책임 경계.

Phase 1 — 캘 4 service + 1 invalidation topic:
- STORAGE_GET_ACTIVE_CALIBRATION / LIST / COMMIT / ACTIVATE
- STORAGE_CALIBRATION_INVALIDATED (ACTIVATE 후 1회)

Phase 2 — scan workflow (append-only, is_active X):
- STORAGE_NEW_SCAN_SESSION / LIST_SCAN_SESSIONS / DELETE_SCAN_SESSION
- STORAGE_PUT_SCAN / LIST_SCANS / DELETE_SCAN
- STORAGE_GET_BLOB (generic — scan / reconstruction 공통)
- STORAGE_PUT_RECONSTRUCTION / LIST_RECONSTRUCTIONS / DELETE_RECONSTRUCTION
"""

from __future__ import annotations

import logging
import time

from core.transport.application_node import ApplicationNode
from core.transport.messages.base import EmptyData, ServiceRequest, ServiceResponse
from core.transport.messages.storage import (
    CalibrationInvalidated,
    CalibrationRunSummary,
    StorageActivateReq,
    StorageActivateRes,
    StorageCommitReq,
    StorageCommitRes,
    StorageDeleteReconstructionReq,
    StorageDeleteScanReq,
    StorageDeleteScanSessionReq,
    StorageGetActiveReq,
    StorageGetActiveRes,
    StorageGetBlobReq,
    StorageGetBlobRes,
    StorageListReconstructionsReq,
    StorageListReconstructionsRes,
    StorageListReq,
    StorageListRes,
    StorageListRunsReq,
    StorageListRunsRes,
    StorageListScanSessionsReq,
    StorageListScanSessionsRes,
    StorageListScansReq,
    StorageListScansRes,
    StorageNewScanSessionReq,
    StorageNewScanSessionRes,
    StoragePutReconstructionReq,
    StoragePutReconstructionRes,
    StoragePutScanReq,
    StoragePutScanRes,
)
from core.transport.topic_map import Service, Topic
from modules.scan_workflow.persistence_models import (
    ReconstructionRecord,
    ScanRecord,
    ScanSessionRecord,
)
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
            Service.STORAGE_LIST_CALIBRATION_RUNS,
            StorageListRunsReq,
            StorageListRunsRes,
            self._srv_list_runs,
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
        # ─── Phase 2 — scan workflow ────────────────────────────
        self.create_service(
            Service.STORAGE_NEW_SCAN_SESSION,
            StorageNewScanSessionReq,
            StorageNewScanSessionRes,
            self._srv_new_scan_session,
        )
        self.create_service(
            Service.STORAGE_LIST_SCAN_SESSIONS,
            StorageListScanSessionsReq,
            StorageListScanSessionsRes,
            self._srv_list_scan_sessions,
        )
        self.create_service(
            Service.STORAGE_DELETE_SCAN_SESSION,
            StorageDeleteScanSessionReq,
            EmptyData,
            self._srv_delete_scan_session,
        )
        self.create_service(
            Service.STORAGE_PUT_SCAN,
            StoragePutScanReq,
            StoragePutScanRes,
            self._srv_put_scan,
        )
        self.create_service(
            Service.STORAGE_LIST_SCANS,
            StorageListScansReq,
            StorageListScansRes,
            self._srv_list_scans,
        )
        self.create_service(
            Service.STORAGE_DELETE_SCAN,
            StorageDeleteScanReq,
            EmptyData,
            self._srv_delete_scan,
        )
        self.create_service(
            Service.STORAGE_GET_BLOB,
            StorageGetBlobReq,
            StorageGetBlobRes,
            self._srv_get_blob,
        )
        self.create_service(
            Service.STORAGE_PUT_RECONSTRUCTION,
            StoragePutReconstructionReq,
            StoragePutReconstructionRes,
            self._srv_put_reconstruction,
        )
        self.create_service(
            Service.STORAGE_LIST_RECONSTRUCTIONS,
            StorageListReconstructionsReq,
            StorageListReconstructionsRes,
            self._srv_list_reconstructions,
        )
        self.create_service(
            Service.STORAGE_DELETE_RECONSTRUCTION,
            StorageDeleteReconstructionReq,
            EmptyData,
            self._srv_delete_reconstruction,
        )
        super().start()
        logger.info(
            "StorageNode 시작 — 캘 4 service (Phase 1) + scan workflow 10 service (Phase 2)"
        )

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

    def _srv_list_runs(
        self, req: ServiceRequest[StorageListRunsReq]
    ) -> ServiceResponse[StorageListRunsRes]:
        rows = self._reg.rdb.list_runs(req.data.robot_id, req.data.limit)
        summaries = [
            CalibrationRunSummary(run=run, results=results) for run, results in rows
        ]
        return ServiceResponse(
            success=True, data=StorageListRunsRes(runs=summaries)
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

    # ─── Phase 2 — scan workflow handlers ─────────────────────

    # ── scan_sessions
    def _srv_new_scan_session(
        self, req: ServiceRequest[StorageNewScanSessionReq]
    ) -> ServiceResponse[StorageNewScanSessionRes]:
        data = req.data
        sid = (data.session_id or "").strip() or time.strftime(
            "session_%Y%m%d_%H%M%S"
        )
        # idempotent — 이미 있으면 그것 반환 (CAPTURE 자리 재진입 자리 robust).
        existing = self._reg.rdb.find_scan_session_by_id(data.robot_id, sid)
        if existing is not None:
            return ServiceResponse(
                success=True,
                message="이미 존재하는 session",
                data=StorageNewScanSessionRes(session=existing),
            )
        record = ScanSessionRecord(
            robot_id=data.robot_id,
            session_id=sid,
            created_at=time.time(),
            label=data.label,
            note=data.note,
        )
        row_id = self._reg.rdb.insert_scan_session(record)
        out = self._reg.rdb.get_scan_session(row_id)
        assert out is not None
        logger.info(
            "NEW_SCAN_SESSION: row_id=%d, robot=%s, session_id=%s",
            row_id, out.robot_id, out.session_id,
        )
        return ServiceResponse(
            success=True, data=StorageNewScanSessionRes(session=out)
        )

    def _srv_list_scan_sessions(
        self, req: ServiceRequest[StorageListScanSessionsReq]
    ) -> ServiceResponse[StorageListScanSessionsRes]:
        sessions = self._reg.rdb.list_scan_sessions(
            req.data.robot_id, req.data.limit
        )
        return ServiceResponse(
            success=True, data=StorageListScanSessionsRes(sessions=sessions)
        )

    def _srv_delete_scan_session(
        self, req: ServiceRequest[StorageDeleteScanSessionReq]
    ) -> ServiceResponse[EmptyData]:
        sid = req.data.session_row_id
        # 자식 blob_key 자리 먼저 모아 ObjectStore delete (CASCADE 전에).
        scans = self._reg.rdb.list_scans(sid)
        recons = self._reg.rdb.list_reconstructions(sid)
        for s in scans:
            try:
                self._reg.objects.delete(s.blob_key)
            except Exception as e:
                logger.warning("scan blob delete 실패 (%s): %s", s.blob_key, e)
        for r in recons:
            try:
                self._reg.objects.delete(r.blob_key)
            except Exception as e:
                logger.warning(
                    "reconstruction blob delete 실패 (%s): %s", r.blob_key, e
                )
        self._reg.rdb.delete_scan_session(sid)
        logger.info(
            "DELETE_SCAN_SESSION: row_id=%d (scans=%d, recons=%d)",
            sid, len(scans), len(recons),
        )
        return ServiceResponse(success=True, data=EmptyData())

    # ── scans
    def _srv_put_scan(
        self, req: ServiceRequest[StoragePutScanReq]
    ) -> ServiceResponse[StoragePutScanRes]:
        data = req.data
        session = self._reg.rdb.get_scan_session(data.session_row_id)
        if session is None:
            return ServiceResponse(
                success=False,
                message=f"session_row_id={data.session_row_id} 없음",
            )
        scan_id = self._reg.rdb.allocate_scan_id(data.session_row_id)
        blob_key = (
            f"scans/{session.robot_id}/{session.session_id}/{scan_id:03d}.bin"
        )
        # blob put 먼저 — 실패 자리 RDB row 안 들어감.
        self._reg.objects.put(blob_key, data.blob_bytes)
        record = ScanRecord(
            session_row_id=data.session_row_id,
            robot_id=session.robot_id,
            scan_id=scan_id,
            created_at=time.time(),
            blob_key=blob_key,
            num_frames=data.num_frames,
            width=data.width,
            height=data.height,
            fx=data.fx,
            fy=data.fy,
            cx=data.cx,
            cy=data.cy,
            depth_scale=data.depth_scale,
            motor_positions=data.motor_positions,
            arm_motor_ids=data.arm_motor_ids,
        )
        row_id = self._reg.rdb.insert_scan(record)
        out = self._reg.rdb.get_scan(row_id)
        assert out is not None
        logger.info(
            "PUT_SCAN: row_id=%d, session=%d, scan_id=%d (blob=%d bytes)",
            row_id, data.session_row_id, scan_id, len(data.blob_bytes),
        )
        return ServiceResponse(success=True, data=StoragePutScanRes(scan=out))

    def _srv_list_scans(
        self, req: ServiceRequest[StorageListScansReq]
    ) -> ServiceResponse[StorageListScansRes]:
        scans = self._reg.rdb.list_scans(req.data.session_row_id)
        return ServiceResponse(success=True, data=StorageListScansRes(scans=scans))

    def _srv_delete_scan(
        self, req: ServiceRequest[StorageDeleteScanReq]
    ) -> ServiceResponse[EmptyData]:
        scan = self._reg.rdb.get_scan(req.data.scan_row_id)
        if scan is None:
            return ServiceResponse(
                success=False, message=f"scan_row_id={req.data.scan_row_id} 없음"
            )
        try:
            self._reg.objects.delete(scan.blob_key)
        except Exception as e:
            logger.warning("scan blob delete 실패 (%s): %s", scan.blob_key, e)
        self._reg.rdb.delete_scan(req.data.scan_row_id)
        return ServiceResponse(success=True, data=EmptyData())

    # ── blob (generic)
    def _srv_get_blob(
        self, req: ServiceRequest[StorageGetBlobReq]
    ) -> ServiceResponse[StorageGetBlobRes]:
        try:
            data = self._reg.objects.get(req.data.blob_key)
        except Exception as e:
            return ServiceResponse(
                success=False, message=f"blob get 실패 ({req.data.blob_key}): {e}"
            )
        return ServiceResponse(
            success=True, data=StorageGetBlobRes(blob_bytes=data)
        )

    # ── reconstructions
    def _srv_put_reconstruction(
        self, req: ServiceRequest[StoragePutReconstructionReq]
    ) -> ServiceResponse[StoragePutReconstructionRes]:
        data = req.data
        session = self._reg.rdb.get_scan_session(data.session_row_id)
        if session is None:
            return ServiceResponse(
                success=False,
                message=f"session_row_id={data.session_row_id} 없음",
            )
        # blob_key 자리 RDB lastrowid 모름 — INSERT 후 UPDATE 자리 패턴 X.
        # session_id + created_at 기반 자리 — uniqueness 자리 session 안 created_at 으로.
        created_at = time.time()
        blob_key = (
            f"reconstructions/{session.robot_id}/{session.session_id}/"
            f"recon_{int(created_at * 1000)}.ply"
        )
        self._reg.objects.put(blob_key, data.blob_bytes)
        record = ReconstructionRecord(
            session_row_id=data.session_row_id,
            robot_id=session.robot_id,
            created_at=created_at,
            blob_key=blob_key,
            voxel_size=data.voxel_size,
            sdf_trunc=data.sdf_trunc,
            depth_trunc=data.depth_trunc,
            icp_max_dist=data.icp_max_dist,
            n_scans=data.n_scans,
            n_edges=data.n_edges,
            vertex_count=data.vertex_count,
            triangle_count=data.triangle_count,
            elapsed=data.elapsed,
        )
        row_id = self._reg.rdb.insert_reconstruction(record)
        out = self._reg.rdb.get_reconstruction(row_id)
        assert out is not None
        logger.info(
            "PUT_RECONSTRUCTION: row_id=%d, session=%d, blob=%d bytes",
            row_id, data.session_row_id, len(data.blob_bytes),
        )
        return ServiceResponse(
            success=True, data=StoragePutReconstructionRes(reconstruction=out)
        )

    def _srv_list_reconstructions(
        self, req: ServiceRequest[StorageListReconstructionsReq]
    ) -> ServiceResponse[StorageListReconstructionsRes]:
        recons = self._reg.rdb.list_reconstructions(req.data.session_row_id)
        return ServiceResponse(
            success=True,
            data=StorageListReconstructionsRes(reconstructions=recons),
        )

    def _srv_delete_reconstruction(
        self, req: ServiceRequest[StorageDeleteReconstructionReq]
    ) -> ServiceResponse[EmptyData]:
        recon = self._reg.rdb.get_reconstruction(req.data.recon_row_id)
        if recon is None:
            return ServiceResponse(
                success=False,
                message=f"recon_row_id={req.data.recon_row_id} 없음",
            )
        try:
            self._reg.objects.delete(recon.blob_key)
        except Exception as e:
            logger.warning(
                "reconstruction blob delete 실패 (%s): %s", recon.blob_key, e
            )
        self._reg.rdb.delete_reconstruction(req.data.recon_row_id)
        return ServiceResponse(success=True, data=EmptyData())
