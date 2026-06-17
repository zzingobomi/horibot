"""ReconstructionNode — multi-view reconstruction heavy compute.

Storage 의 ScanRecord list + blob 자리 fetch → ICP + PoseGraph + TSDF + mesh
extract → Storage 의 ReconstructionRecord + .ply blob put. ScanTask 의
BuildReconstruction step 자리 caller.

PC host-level 1개 (robot-scoped X). 미래 multi-robot scan fusion 자리도 같은
자리 (input 자리 multi-robot scan list 자리).

progress 자리는 RECONSTRUCTION_PROGRESS topic publish — 5 stage (loading_scans
/ pairwise_registration / pose_graph_optimization / tsdf_integration /
mesh_extraction). frontend BuildReconstruction step 자리 progress bar 자리.
"""

from __future__ import annotations

import logging
import threading

from core.transport.application_node import ApplicationNode
from core.transport.messages.base import ServiceRequest, ServiceResponse
from core.transport.messages.reconstruction import (
    ReconstructionBuildReq,
    ReconstructionBuildRes,
    ReconstructionProgress,
)
from core.transport.messages.storage import (
    StorageGetBlobReq,
    StorageGetBlobRes,
    StorageListScansReq,
    StorageListScansRes,
    StoragePutReconstructionReq,
    StoragePutReconstructionRes,
)
from core.transport.topic_map import Service, Topic
from modules.motor.motor_config import load_motor_layout
from modules.reconstruction import build as recon_build
from modules.scan_workflow import blob as scan_blob

logger = logging.getLogger(__name__)


class ReconstructionNode(ApplicationNode):
    def __init__(self) -> None:
        super().__init__("reconstruction_node")
        # 한 번에 build 하나만 (heavy compute) — 자원 경합 차단.
        self._build_lock = threading.Lock()

    def start(self) -> None:
        self.create_service(
            Service.RECONSTRUCTION_BUILD,
            ReconstructionBuildReq,
            ReconstructionBuildRes,
            self._srv_build,
        )
        super().start()
        logger.info("ReconstructionNode 시작")

    def _srv_build(
        self, req: ServiceRequest[ReconstructionBuildReq]
    ) -> ServiceResponse[ReconstructionBuildRes]:
        if not self._build_lock.acquire(blocking=False):
            return ServiceResponse(
                success=False, message="다른 build 진행 중"
            )
        try:
            return self._build(req.data)
        except Exception as e:
            logger.exception("reconstruction build 실패")
            return ServiceResponse(success=False, message=str(e))
        finally:
            self._build_lock.release()

    def _build(
        self, req: ReconstructionBuildReq
    ) -> ServiceResponse[ReconstructionBuildRes]:
        sid = req.session_row_id

        # 1. scan list fetch
        list_res = self.call_service(
            Service.STORAGE_LIST_SCANS,
            StorageListScansReq(session_row_id=sid),
            StorageListScansRes,
        )
        if not list_res.success or list_res.data is None:
            return ServiceResponse(
                success=False,
                message=f"STORAGE_LIST_SCANS 실패: {list_res.message}",
            )
        scan_records = list_res.data.scans
        if len(scan_records) < recon_build.MIN_SCANS:
            return ServiceResponse(
                success=False,
                message=(
                    f"scan {recon_build.MIN_SCANS}개 이상 필요 "
                    f"(현재 {len(scan_records)})"
                ),
            )

        robot_id = scan_records[0].robot_id
        arm_cfgs = load_motor_layout(robot_id).arm

        # 2. 각 scan blob 자리 fetch + decode
        build_inputs: list[recon_build.BuildScanInput] = []
        for idx, record in enumerate(scan_records):
            self._publish_progress(
                sid,
                recon_build.STAGE_LOADING,
                idx / max(len(scan_records), 1),
                f"blob fetch {idx + 1}/{len(scan_records)}",
            )
            blob_res = self.call_service(
                Service.STORAGE_GET_BLOB,
                StorageGetBlobReq(blob_key=record.blob_key),
                StorageGetBlobRes,
                timeout=30.0,
            )
            if not blob_res.success or blob_res.data is None:
                return ServiceResponse(
                    success=False,
                    message=(
                        f"blob fetch 실패 ({record.blob_key}): {blob_res.message}"
                    ),
                )
            color_bgr, depth_z16 = scan_blob.decode(
                blob_res.data.blob_bytes, record.width, record.height
            )
            build_inputs.append(
                recon_build.BuildScanInput(
                    color_bgr=color_bgr,
                    depth_z16=depth_z16,
                    width=record.width,
                    height=record.height,
                    fx=record.fx,
                    fy=record.fy,
                    cx=record.cx,
                    cy=record.cy,
                    depth_scale=record.depth_scale,
                    motor_positions=record.motor_positions,
                    arm_motor_ids=record.arm_motor_ids,
                )
            )

        # 3. build
        def _on_progress(stage: str, percent: float, message: str) -> None:
            self._publish_progress(sid, stage, percent, message)

        voxel_size = (
            req.voxel_size if req.voxel_size is not None
            else recon_build.DEFAULT_VOXEL_SIZE
        )
        sdf_trunc = (
            req.sdf_trunc if req.sdf_trunc is not None
            else recon_build.DEFAULT_SDF_TRUNC
        )
        depth_trunc = (
            req.depth_trunc if req.depth_trunc is not None
            else recon_build.DEFAULT_DEPTH_TRUNC
        )
        icp_max_dist = (
            req.icp_max_dist if req.icp_max_dist is not None
            else recon_build.DEFAULT_ICP_MAX_DIST
        )

        result = recon_build.build_mesh(
            build_inputs,
            arm_cfgs,
            robot_id=robot_id,
            voxel_size=voxel_size,
            sdf_trunc=sdf_trunc,
            depth_trunc=depth_trunc,
            icp_max_dist=icp_max_dist,
            progress=_on_progress,
        )

        # 4. storage put
        put_res = self.call_service(
            Service.STORAGE_PUT_RECONSTRUCTION,
            StoragePutReconstructionReq(
                session_row_id=sid,
                blob_bytes=result.mesh_bytes,
                voxel_size=voxel_size,
                sdf_trunc=sdf_trunc,
                depth_trunc=depth_trunc,
                icp_max_dist=icp_max_dist,
                n_scans=result.n_scans,
                n_edges=result.n_edges,
                vertex_count=result.vertex_count,
                triangle_count=result.triangle_count,
                elapsed=result.elapsed,
            ),
            StoragePutReconstructionRes,
            timeout=60.0,
        )
        if not put_res.success or put_res.data is None:
            return ServiceResponse(
                success=False,
                message=f"STORAGE_PUT_RECONSTRUCTION 실패: {put_res.message}",
            )

        logger.info(
            "RECONSTRUCTION_BUILD 완료: session=%d, verts=%d, tris=%d, elapsed=%.1fs",
            sid, result.vertex_count, result.triangle_count, result.elapsed,
        )
        return ServiceResponse(
            success=True,
            data=ReconstructionBuildRes(reconstruction=put_res.data.reconstruction),
        )

    def _publish_progress(
        self, session_row_id: int, stage: str, percent: float, message: str
    ) -> None:
        try:
            self.publish(
                Topic.RECONSTRUCTION_PROGRESS,
                ReconstructionProgress(
                    session_row_id=session_row_id,
                    stage=stage,  # type: ignore[arg-type]
                    percent=max(0.0, min(1.0, percent)),
                    message=message,
                ),
            )
        except Exception as e:
            logger.warning("reconstruction progress publish 실패: %s", e)
