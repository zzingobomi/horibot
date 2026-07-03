"""ScanModule — robot-agnostic scan workflow + persistence + reconstruction.

옛 StorageNode(scan) + ReconstructionNode + ScanTask orchestration 통합. Task DSL
없이 frontend 가 서비스 직접 호출 (실용 슬라이스). PC 배치 (Open3D heavy + DB owner).

**robot-agnostic** — host 당 1 인스턴스 (backend_v2.md §2.7).
대상 robot 은 새 세션(new_session/list_sessions)
은 req.robot_id, 진행 중 자원은 session row 에서 파생. per-robot config(kinematics/
arm_specs)는 resolve 가 주입한 robots 투영으로 조회, runtime state 는 robot_id 키 dict.

capture flow: scene3d SNAPSHOT(consensus) + latest raw motor → blob 저장.
build flow: scans 로드 → raw→rad→FK→hand_eye 로 camera pose → TSDF (to_thread) →
progress stream 발행 → .ply 저장.

다른 모듈 호출은 `async def` 핸들러 + `await self.runtime.call(...)` 하나로 통일
(framework_async_call_contract.md). sync→async bridge 는 framework 가 흡수 —
모듈은 run_coroutine_threadsafe 를 모른다. heavy build_mesh 는 `await
asyncio.to_thread(...)` 로 event loop 를 안 막음.
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass
from datetime import UTC, datetime

import numpy as np

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.contract.subscriber import subscriber
from framework.runtime.api import ModuleRuntime
from framework.storage.protocol import ObjectStore
from modules.calibration.contract import (
    Calibration,
    CalibrationBundle,
    SnapshotBundleRequest,
)
from modules.motion import units
from modules.motion.kinematics import Kinematics
from modules.motor.contract import JointState, Motor
from modules.motor.layout import MotorSpec
from modules.scene3d.contract import Scene3d, SnapshotRequest, SnapshotResponse

from . import blob as scan_blob
from . import build as recon
from .contract import (
    BuildProgress,
    BuildRequest,
    BuildResponse,
    CaptureRequest,
    CaptureResponse,
    DeleteScanRequest,
    DeleteScanResponse,
    DeleteSessionRequest,
    DeleteSessionResponse,
    GetMeshRequest,
    GetMeshResponse,
    ListReconstructionsRequest,
    ListReconstructionsResponse,
    ListScansRequest,
    ListScansResponse,
    ListSessionsRequest,
    ListSessionsResponse,
    NewSessionRequest,
    NewSessionResponse,
    ReconstructionRecord,
    Scan,
    ScanRecord,
    ScanSessionRecord,
)
from .persistence.repository import ScanRepository

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class ScanRobotSpec:
    """robot 별 정적 config — resolve 가 robots.yaml/URDF 에서 투영해 주입.

    wire contract 아님 (constructor dep). kinematics 인스턴스를 들고 있어
    pydantic 아니라 dataclass (calibration 의 CalibrationRobotSpec 과 같은 role).
    """

    kinematics: Kinematics
    arm_specs: list[MotorSpec]  # motors.yaml 순 (gripper 제외)


@publishes((Scan.Stream.BUILD_PROGRESS, BuildProgress))
class ScanModule:
    def __init__(
        self,
        runtime: ModuleRuntime,
        repository: ScanRepository,
        object_store: ObjectStore,
        robots: dict[str, ScanRobotSpec],
    ) -> None:
        self.runtime = runtime
        self._repo = repository
        self._blob = object_store
        self._robots = robots
        # runtime state — robot_id 키 dict (실행 중에만 존재)
        self._latest_raw: dict[str, list[int]] = {}
        self._progress_seq: dict[str, int] = {}

    # ── lifecycle ─────────────────────────────────────────────
    async def start(self) -> None:
        logger.info("ScanModule start (host-level, robots=%s)", sorted(self._robots))
        # PyBullet init 는 순차 (per-robot 독립 client 지만 thread 동시 init 회피)
        for spec in self._robots.values():
            await asyncio.to_thread(spec.kinematics.initialize)

    async def stop(self) -> None:
        for spec in self._robots.values():
            await asyncio.to_thread(spec.kinematics.close)
        logger.info("ScanModule stop (host-level)")

    @subscriber(Motor.Stream.RAW_STATE)
    def on_motor_raw(self, state: JointState) -> None:
        spec = self._robots.get(state.robot_id)
        if spec is None:
            return
        dof = len(spec.arm_specs)
        if len(state.positions_raw) < dof:
            return
        self._latest_raw[state.robot_id] = list(state.positions_raw[:dof])

    # ── sessions ──────────────────────────────────────────────
    @service(Scan.Service.NEW_SESSION)
    def new_session(self, req: NewSessionRequest) -> NewSessionResponse:
        now = datetime.now(UTC)
        session_id = "session_" + now.strftime("%Y%m%d_%H%M%S")
        rec = self._repo.insert_session(
            ScanSessionRecord(
                robot_id=req.robot_id,
                session_id=session_id,
                created_at=now,
                label=req.label,
            )
        )
        return NewSessionResponse(session=rec)

    @service(Scan.Service.LIST_SESSIONS)
    def list_sessions(self, req: ListSessionsRequest) -> ListSessionsResponse:
        return ListSessionsResponse(sessions=self._repo.list_sessions(req.robot_id))

    @service(Scan.Service.DELETE_SESSION)
    def delete_session(self, req: DeleteSessionRequest) -> DeleteSessionResponse:
        self._repo.delete_session(req.session_row_id)
        return DeleteSessionResponse(ok=True)

    # ── capture ───────────────────────────────────────────────
    @service(Scan.Service.CAPTURE)
    async def capture(self, req: CaptureRequest) -> CaptureResponse:
        session = self._repo.get_session(req.session_row_id)
        if session is None:
            return CaptureResponse(accepted=False, message="scan 세션 없음")
        robot_id = session.robot_id  # 대상 robot = 세션 소유자 (req 중복 채널 X)
        spec = self._robots.get(robot_id)
        if spec is None:
            return CaptureResponse(
                accepted=False, message=f"robot {robot_id!r} 이 이 host fleet 에 없음"
            )
        raw = self._latest_raw.get(robot_id)
        if raw is None:
            return CaptureResponse(accepted=False, message="motor state 아직 없음")

        try:
            snap = await self.runtime.call(
                Scene3d.Service.SNAPSHOT,
                SnapshotRequest(robot_id=robot_id, num_frames=req.num_frames),
                SnapshotResponse,
                timeout=8.0,
            )
        except Exception as e:
            return CaptureResponse(
                accepted=False, message=f"scene3d snapshot 실패: {e}"
            )

        blob = scan_blob.encode(snap.color_jpeg, snap.depth_zstd)
        scan_id = self._repo.allocate_scan_id(req.session_row_id)
        key = f"scans/{robot_id}/{session.session_id}/{scan_id:03d}.bin"
        self._blob.put(key, blob)
        intr = snap.intrinsic
        saved = self._repo.insert_scan(
            ScanRecord(
                session_row_id=req.session_row_id,
                robot_id=robot_id,
                scan_id=scan_id,
                created_at=datetime.now(UTC),
                blob_key=key,
                num_frames=snap.num_frames,
                width=intr.width,
                height=intr.height,
                fx=intr.fx,
                fy=intr.fy,
                cx=intr.cx,
                cy=intr.cy,
                depth_scale=intr.depth_scale,
                motor_positions=list(raw),
                arm_motor_ids=[s.id for s in spec.arm_specs],
            )
        )
        count = len(self._repo.list_scans(req.session_row_id))
        return CaptureResponse(accepted=True, scan=saved, scan_count=count)

    @service(Scan.Service.LIST_SCANS)
    def list_scans(self, req: ListScansRequest) -> ListScansResponse:
        return ListScansResponse(scans=self._repo.list_scans(req.session_row_id))

    @service(Scan.Service.DELETE_SCAN)
    def delete_scan(self, req: DeleteScanRequest) -> DeleteScanResponse:
        scan = self._repo.get_scan(req.scan_row_id)
        self._repo.delete_scan(req.scan_row_id)
        if scan is not None:
            try:
                self._blob.delete(scan.blob_key)
            except KeyError:
                pass
        return DeleteScanResponse(ok=True)

    # ── build (reconstruction) ────────────────────────────────
    @service(Scan.Service.BUILD)
    async def build(self, req: BuildRequest) -> BuildResponse:
        scans = self._repo.list_scans(req.session_row_id)
        if len(scans) < recon.MIN_SCANS:
            return BuildResponse(
                accepted=False,
                message=f"scan {len(scans)}개 < 최소 {recon.MIN_SCANS} — build 불가",
            )
        session = self._repo.get_session(req.session_row_id)
        if session is None:
            return BuildResponse(accepted=False, message="scan 세션 없음")
        robot_id = session.robot_id  # 대상 robot = 세션 소유자
        spec = self._robots.get(robot_id)
        if spec is None:
            return BuildResponse(
                accepted=False, message=f"robot {robot_id!r} 이 이 host fleet 에 없음"
            )

        try:
            # calibration 은 robot-agnostic — 대상 robot 은 req 필드.
            bundle = await self.runtime.call(
                Calibration.Service.SNAPSHOT_BUNDLE,
                SnapshotBundleRequest(robot_id=robot_id),
                CalibrationBundle,
                timeout=5.0,
            )
        except Exception as e:
            return BuildResponse(
                accepted=False, message=f"calibration bundle 실패: {e}"
            )
        if bundle.hand_eye is None:
            return BuildResponse(
                accepted=False, message="hand_eye 캘 없음 — build 불가 (캘 먼저)"
            )

        t_ee_cam = np.eye(4)
        t_ee_cam[:3, :3] = np.array(
            bundle.hand_eye.result_data.R_cam2gripper, dtype=float
        )
        t_ee_cam[:3, 3] = np.array(
            bundle.hand_eye.result_data.t_cam2gripper, dtype=float
        ).reshape(3)

        inputs: list[recon.BuildScanInput] = []
        for i, s in enumerate(scans):
            self._publish_progress(
                robot_id,
                req.session_row_id,
                "loading_scans",
                (i + 1) / len(scans),
                f"scan {i + 1}/{len(scans)} 로딩",
            )
            color, depth = scan_blob.decode(
                self._blob.get(s.blob_key), s.width, s.height
            )
            arm_rad = self._arm_rad(spec, s.motor_positions, s.arm_motor_ids)
            rot, pos = spec.kinematics.fk_to_matrix(arm_rad)
            t_base_ee = np.eye(4)
            t_base_ee[:3, :3] = np.array(rot, dtype=float)
            t_base_ee[:3, 3] = np.array(pos, dtype=float)
            inputs.append(
                recon.BuildScanInput(
                    color_bgr=color,
                    depth_z16=depth,
                    width=s.width,
                    height=s.height,
                    fx=s.fx,
                    fy=s.fy,
                    cx=s.cx,
                    cy=s.cy,
                    depth_scale=s.depth_scale,
                    t_base_cam_init=t_base_ee @ t_ee_cam,
                )
            )

        kwargs: dict[str, float] = {}
        if req.voxel_size is not None:
            kwargs["voxel_size"] = req.voxel_size
        if req.sdf_trunc is not None:
            kwargs["sdf_trunc"] = req.sdf_trunc
        if req.depth_trunc is not None:
            kwargs["depth_trunc"] = req.depth_trunc
        if req.icp_max_dist is not None:
            kwargs["icp_max_dist"] = req.icp_max_dist

        def _progress(stage: str, percent: float, message: str) -> None:
            self._publish_progress(robot_id, req.session_row_id, stage, percent, message)

        try:
            # heavy Open3D TSDF/ICP — event loop 를 안 막게 thread 로 offload.
            # progress 콜백은 그 thread 에서 runtime.publish (sync, thread-safe) 호출.
            result = await asyncio.to_thread(
                recon.build_mesh, inputs, progress=_progress, **kwargs
            )
        except Exception as e:
            logger.exception("build_mesh 실패 robot=%s", robot_id)
            self._publish_progress(robot_id, req.session_row_id, "failed", 1.0, str(e))
            return BuildResponse(accepted=False, message=f"build 실패: {e}")

        now = datetime.now(UTC)
        key = (
            f"reconstructions/{robot_id}/{session.session_id}/"
            f"recon_{int(now.timestamp() * 1000)}.ply"
        )
        self._blob.put(key, result.mesh_bytes)
        saved = self._repo.insert_reconstruction(
            ReconstructionRecord(
                session_row_id=req.session_row_id,
                robot_id=robot_id,
                created_at=now,
                blob_key=key,
                voxel_size=kwargs.get("voxel_size", recon.DEFAULT_VOXEL_SIZE),
                sdf_trunc=kwargs.get("sdf_trunc", recon.DEFAULT_SDF_TRUNC),
                depth_trunc=kwargs.get("depth_trunc", recon.DEFAULT_DEPTH_TRUNC),
                icp_max_dist=kwargs.get("icp_max_dist", recon.DEFAULT_ICP_MAX_DIST),
                n_scans=result.n_scans,
                n_edges=result.n_edges,
                vertex_count=result.vertex_count,
                triangle_count=result.triangle_count,
                elapsed=result.elapsed,
            )
        )
        self._publish_progress(
            robot_id,
            req.session_row_id,
            "done",
            1.0,
            f"완료 ({result.vertex_count} verts)",
            recon_id=saved.id,
        )
        return BuildResponse(accepted=True, reconstruction=saved)

    @service(Scan.Service.LIST_RECONSTRUCTIONS)
    def list_reconstructions(
        self, req: ListReconstructionsRequest
    ) -> ListReconstructionsResponse:
        return ListReconstructionsResponse(
            reconstructions=self._repo.list_reconstructions(req.session_row_id)
        )

    @service(Scan.Service.GET_MESH)
    def get_mesh(self, req: GetMeshRequest) -> GetMeshResponse:
        rec = self._repo.get_reconstruction(req.reconstruction_row_id)
        if rec is None:
            raise RuntimeError(f"reconstruction {req.reconstruction_row_id} 없음")
        ply = self._blob.get(rec.blob_key)
        return GetMeshResponse(
            ply_bytes=ply,
            vertex_count=rec.vertex_count,
            triangle_count=rec.triangle_count,
        )

    # ── internal ──────────────────────────────────────────────
    @staticmethod
    def _arm_rad(
        spec: ScanRobotSpec, motor_positions: list[int], arm_motor_ids: list[int]
    ) -> list[float]:
        """raw motor → arm rad. 저장된 motor id 로 매핑 (order 무관 robust)."""
        by_id = dict(zip(arm_motor_ids, motor_positions))
        return [units.raw_to_rad(by_id[s.id], s) for s in spec.arm_specs]

    def _publish_progress(
        self,
        robot_id: str,
        session_row_id: int,
        stage: str,
        percent: float,
        message: str,
        recon_id: int | None = None,
    ) -> None:
        seq = self._progress_seq.get(robot_id, 0)
        self.runtime.publish(
            Scan.Stream.BUILD_PROGRESS,
            BuildProgress(
                robot_id=robot_id,
                seq=seq,
                timestamp_unix=time.time(),
                session_row_id=session_row_id,
                stage=stage,
                percent=percent,
                message=message,
                reconstruction_row_id=recon_id,
            ),
        )
        self._progress_seq[robot_id] = seq + 1
