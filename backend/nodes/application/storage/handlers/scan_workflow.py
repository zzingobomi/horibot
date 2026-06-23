"""Scan workflow 도메인 Zenoh service handler group.

storage_node 의 lifecycle + composition root 안 register(node) 자리 호출되어
service handler 들을 node 에 등록. 본 group 의 책임 = scan workflow 도메인
(sessions / scans / reconstructions + ObjectStore blob CRUD) 의 Zenoh 노출.

10 service:
- NEW_SCAN_SESSION / LIST_SCAN_SESSIONS / DELETE_SCAN_SESSION
- PUT_SCAN / LIST_SCANS / DELETE_SCAN
- GET_BLOB (generic — scan / reconstruction 공통)
- PUT_RECONSTRUCTION / LIST_RECONSTRUCTIONS / DELETE_RECONSTRUCTION
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from datetime import UTC, datetime
from typing import Any

from core.transport.application_node import ApplicationNode
from core.transport.messages.base import EmptyData, ServiceRequest, ServiceResponse
from core.transport.messages.storage import (
    CreateScanSessionReq,
    CreateScanSessionRes,
    DeleteReconstructionReq,
    DeleteScanReq,
    DeleteScanSessionReq,
    GetBlobReq,
    GetBlobRes,
    ListReconstructionsReq,
    ListReconstructionsRes,
    ListScanSessionsReq,
    ListScanSessionsRes,
    ListScansReq,
    ListScansRes,
    PutReconstructionReq,
    PutReconstructionRes,
    PutScanReq,
    PutScanRes,
)
from core.transport.topic_map import Service
from modules.scan_workflow.persistence_models import (
    ReconstructionRecord,
    ScanRecord,
    ScanSessionRecord,
)
from modules.storage.registry import StorageRegistry

logger = logging.getLogger(__name__)


class ScanWorkflowHandlers:
    """Scan workflow 도메인 service handler 묶음."""

    def __init__(self, reg: StorageRegistry, publish: Callable[[str, Any], None]) -> None:
        self._reg = reg
        self._publish = publish

    def register(self, node: ApplicationNode) -> None:
        node.create_service(
            Service.STORAGE_NEW_SCAN_SESSION,
            CreateScanSessionReq,
            CreateScanSessionRes,
            self._srv_new_scan_session,
        )
        node.create_service(
            Service.STORAGE_LIST_SCAN_SESSIONS,
            ListScanSessionsReq,
            ListScanSessionsRes,
            self._srv_list_scan_sessions,
        )
        node.create_service(
            Service.STORAGE_DELETE_SCAN_SESSION,
            DeleteScanSessionReq,
            EmptyData,
            self._srv_delete_scan_session,
        )
        node.create_service(
            Service.STORAGE_PUT_SCAN,
            PutScanReq,
            PutScanRes,
            self._srv_put_scan,
        )
        node.create_service(
            Service.STORAGE_LIST_SCANS,
            ListScansReq,
            ListScansRes,
            self._srv_list_scans,
        )
        node.create_service(
            Service.STORAGE_DELETE_SCAN,
            DeleteScanReq,
            EmptyData,
            self._srv_delete_scan,
        )
        node.create_service(
            Service.STORAGE_GET_BLOB,
            GetBlobReq,
            GetBlobRes,
            self._srv_get_blob,
        )
        node.create_service(
            Service.STORAGE_PUT_RECONSTRUCTION,
            PutReconstructionReq,
            PutReconstructionRes,
            self._srv_put_reconstruction,
        )
        node.create_service(
            Service.STORAGE_LIST_RECONSTRUCTIONS,
            ListReconstructionsReq,
            ListReconstructionsRes,
            self._srv_list_reconstructions,
        )
        node.create_service(
            Service.STORAGE_DELETE_RECONSTRUCTION,
            DeleteReconstructionReq,
            EmptyData,
            self._srv_delete_reconstruction,
        )

    # ─── scan_sessions ────────────────────────────────────────

    def _srv_new_scan_session(
        self, req: ServiceRequest[CreateScanSessionReq]
    ) -> ServiceResponse[CreateScanSessionRes]:
        data = req.data
        sid = (data.session_id or "").strip() or datetime.now(UTC).strftime(
            "session_%Y%m%d_%H%M%S"
        )
        with self._reg.rdb.session() as repos:
            # idempotent — 이미 있으면 그것 반환 (CAPTURE 자리 재진입 robust).
            existing = repos.scan_workflow.find_session_by_id(data.robot_id, sid)
            if existing is not None:
                return ServiceResponse(
                    success=True,
                    message="이미 존재하는 session",
                    data=CreateScanSessionRes(session=existing),
                )
            record = ScanSessionRecord(
                robot_id=data.robot_id,
                session_id=sid,
                created_at=datetime.now(UTC),
                label=data.label,
                note=data.note,
            )
            row_id = repos.scan_workflow.insert_session(record)
            out = repos.scan_workflow.get_session(row_id)
        assert out is not None
        logger.info(
            "NEW_SCAN_SESSION: row_id=%d, robot=%s, session_id=%s",
            row_id, out.robot_id, out.session_id,
        )
        return ServiceResponse(
            success=True, data=CreateScanSessionRes(session=out)
        )

    def _srv_list_scan_sessions(
        self, req: ServiceRequest[ListScanSessionsReq]
    ) -> ServiceResponse[ListScanSessionsRes]:
        with self._reg.rdb.session() as repos:
            sessions = repos.scan_workflow.list_sessions(
                req.data.robot_id, req.data.limit
            )
        return ServiceResponse(
            success=True, data=ListScanSessionsRes(sessions=sessions)
        )

    def _srv_delete_scan_session(
        self, req: ServiceRequest[DeleteScanSessionReq]
    ) -> ServiceResponse[EmptyData]:
        sid = req.data.session_row_id
        # 자식 blob_key 먼저 모아 ObjectStore delete (CASCADE 전에).
        with self._reg.rdb.session() as repos:
            scans = repos.scan_workflow.list_scans(sid)
            recons = repos.scan_workflow.list_reconstructions(sid)
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
        with self._reg.rdb.session() as repos:
            repos.scan_workflow.delete_session(sid)
        logger.info(
            "DELETE_SCAN_SESSION: row_id=%d (scans=%d, recons=%d)",
            sid, len(scans), len(recons),
        )
        return ServiceResponse(success=True, data=EmptyData())

    # ─── scans ────────────────────────────────────────────────

    def _srv_put_scan(
        self, req: ServiceRequest[PutScanReq]
    ) -> ServiceResponse[PutScanRes]:
        data = req.data
        with self._reg.rdb.session() as repos:
            session = repos.scan_workflow.get_session(data.session_row_id)
            if session is None:
                return ServiceResponse(
                    success=False,
                    message=f"session_row_id={data.session_row_id} 없음",
                )
            scan_id = repos.scan_workflow.allocate_scan_id(data.session_row_id)
            blob_key = (
                f"scans/{session.robot_id}/{session.session_id}/{scan_id:03d}.bin"
            )
            # blob put 먼저 — 실패 시 RDB transaction 도 rollback (session.flush 안 함).
            self._reg.objects.put(blob_key, data.blob_bytes)
            record = ScanRecord(
                session_row_id=data.session_row_id,
                robot_id=session.robot_id,
                scan_id=scan_id,
                created_at=datetime.now(UTC),
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
            row_id = repos.scan_workflow.insert_scan(record)
            out = repos.scan_workflow.get_scan(row_id)
        assert out is not None
        logger.info(
            "PUT_SCAN: row_id=%d, session=%d, scan_id=%d (blob=%d bytes)",
            row_id, data.session_row_id, scan_id, len(data.blob_bytes),
        )
        return ServiceResponse(success=True, data=PutScanRes(scan=out))

    def _srv_list_scans(
        self, req: ServiceRequest[ListScansReq]
    ) -> ServiceResponse[ListScansRes]:
        with self._reg.rdb.session() as repos:
            scans = repos.scan_workflow.list_scans(req.data.session_row_id)
        return ServiceResponse(success=True, data=ListScansRes(scans=scans))

    def _srv_delete_scan(
        self, req: ServiceRequest[DeleteScanReq]
    ) -> ServiceResponse[EmptyData]:
        with self._reg.rdb.session() as repos:
            scan = repos.scan_workflow.get_scan(req.data.scan_row_id)
        if scan is None:
            return ServiceResponse(
                success=False, message=f"scan_row_id={req.data.scan_row_id} 없음"
            )
        try:
            self._reg.objects.delete(scan.blob_key)
        except Exception as e:
            logger.warning("scan blob delete 실패 (%s): %s", scan.blob_key, e)
        with self._reg.rdb.session() as repos:
            repos.scan_workflow.delete_scan(req.data.scan_row_id)
        return ServiceResponse(success=True, data=EmptyData())

    # ─── blob (generic) ───────────────────────────────────────

    def _srv_get_blob(
        self, req: ServiceRequest[GetBlobReq]
    ) -> ServiceResponse[GetBlobRes]:
        try:
            data = self._reg.objects.get(req.data.blob_key)
        except Exception as e:
            return ServiceResponse(
                success=False, message=f"blob get 실패 ({req.data.blob_key}): {e}"
            )
        # Pydantic Base64Bytes 가 raw bytes input 을 base64-DECODE 시도해 손상.
        # 미리 base64-encode 후 전달.
        import base64
        b64 = base64.b64encode(data).decode("ascii")
        return ServiceResponse(
            success=True, data=GetBlobRes(blob_bytes=b64)  # type: ignore[arg-type]
        )

    # ─── reconstructions ──────────────────────────────────────

    def _srv_put_reconstruction(
        self, req: ServiceRequest[PutReconstructionReq]
    ) -> ServiceResponse[PutReconstructionRes]:
        data = req.data
        with self._reg.rdb.session() as repos:
            session = repos.scan_workflow.get_session(data.session_row_id)
            if session is None:
                return ServiceResponse(
                    success=False,
                    message=f"session_row_id={data.session_row_id} 없음",
                )
            # blob_key 자리 RDB lastrowid 모름 — INSERT 후 UPDATE 패턴 X.
            # session_id + created_at 기반 자리 — uniqueness 자리 created_at 으로.
            created_at = datetime.now(UTC)
            blob_key = (
                f"reconstructions/{session.robot_id}/{session.session_id}/"
                f"recon_{int(created_at.timestamp() * 1000)}.ply"
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
            row_id = repos.scan_workflow.insert_reconstruction(record)
            out = repos.scan_workflow.get_reconstruction(row_id)
        assert out is not None
        logger.info(
            "PUT_RECONSTRUCTION: row_id=%d, session=%d, blob=%d bytes",
            row_id, data.session_row_id, len(data.blob_bytes),
        )
        return ServiceResponse(
            success=True, data=PutReconstructionRes(reconstruction=out)
        )

    def _srv_list_reconstructions(
        self, req: ServiceRequest[ListReconstructionsReq]
    ) -> ServiceResponse[ListReconstructionsRes]:
        with self._reg.rdb.session() as repos:
            recons = repos.scan_workflow.list_reconstructions(
                req.data.session_row_id
            )
        return ServiceResponse(
            success=True,
            data=ListReconstructionsRes(reconstructions=recons),
        )

    def _srv_delete_reconstruction(
        self, req: ServiceRequest[DeleteReconstructionReq]
    ) -> ServiceResponse[EmptyData]:
        with self._reg.rdb.session() as repos:
            recon = repos.scan_workflow.get_reconstruction(req.data.recon_row_id)
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
        with self._reg.rdb.session() as repos:
            repos.scan_workflow.delete_reconstruction(req.data.recon_row_id)
        return ServiceResponse(success=True, data=EmptyData())
