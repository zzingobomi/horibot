import logging
import struct
import threading
import time

import numpy as np
import open3d as o3d

from core.transport.base_node import BaseNode
from core.common import GRIPPER_ID
from core.cache.joint_state_cache import JointStateCache
from core.transport.messages.base import EmptyData, ServiceRequest, ServiceResponse
from core.transport.messages.camera import CameraSetDepthStreamReq, CameraSetDepthStreamRes
from core.transport.messages.pointcloud import (
    MeshMeta,
    PointcloudBuildMeshReq,
    PointcloudBuildMeshRes,
    PointcloudCaptureReq,
    PointcloudCaptureRes,
    PointcloudConfigureReq,
    PointcloudConfigureRes,
    PointcloudDeleteScanReq,
    PointcloudDeleteScanRes,
    PointcloudListMeshesRes,
    PointcloudListScansReq,
    PointcloudListScansRes,
    PointcloudListSessionsRes,
    PointcloudNewSessionReq,
    PointcloudNewSessionRes,
    PointcloudState,
    ScanMeta,
)
from core.transport.topic_map import Service, Topic
from modules.camera.depth_frame import DepthFrame, decode as decode_depth_frame
from modules.motor.motor_config import load_motor_config
from modules.pointcloud import scan_capture, scan_io, tsdf_builder

logger = logging.getLogger(__name__)

DEFAULT_VOXEL_SIZE = 0.005  # 5mm — 라이브 스트림용 (TSDF는 별도 voxel)
TARGET_FPS = 8.0
IDLE_SLEEP = 0.1
DEPTH_TRUNC = 1.0  # m


class PointCloudNode(BaseNode):
    def __init__(self) -> None:
        super().__init__("pointcloud_node")
        self._cfg_lock = threading.Lock()
        self._enabled = False
        self._voxel_size = DEFAULT_VOXEL_SIZE

        self._frame_lock = threading.Lock()
        self._latest_frame: DepthFrame | None = None

        self._stream_thread: threading.Thread | None = None

        # ─── capture/build 공용 락 (동시 capture/build 직렬화) ───
        self._capture_lock = threading.Lock()

        # ─── arm config + motor state cache ───
        _, motor_cfgs = load_motor_config()
        self._arm_cfgs = [m for m in motor_cfgs if m.id != GRIPPER_ID]
        self._cache = JointStateCache()

    def start(self) -> None:
        # configure
        self.create_service(
            Service.POINTCLOUD_CONFIGURE,
            PointcloudConfigureReq,
            PointcloudConfigureRes,
            self._srv_configure,
        )
        # capture
        self.create_service(
            Service.POINTCLOUD_NEW_SESSION,
            PointcloudNewSessionReq,
            PointcloudNewSessionRes,
            self._srv_new_session,
        )
        self.create_service(
            Service.POINTCLOUD_CAPTURE,
            PointcloudCaptureReq,
            PointcloudCaptureRes,
            self._srv_capture,
        )
        self.create_service(
            Service.POINTCLOUD_LIST_SESSIONS,
            EmptyData,
            PointcloudListSessionsRes,
            self._srv_list_sessions,
        )
        self.create_service(
            Service.POINTCLOUD_LIST_SCANS,
            PointcloudListScansReq,
            PointcloudListScansRes,
            self._srv_list_scans,
        )
        self.create_service(
            Service.POINTCLOUD_DELETE_SCAN,
            PointcloudDeleteScanReq,
            PointcloudDeleteScanRes,
            self._srv_delete_scan,
        )
        # TSDF
        self.create_service(
            Service.POINTCLOUD_BUILD_MESH,
            PointcloudBuildMeshReq,
            PointcloudBuildMeshRes,
            self._srv_build_mesh,
        )
        self.create_service(
            Service.POINTCLOUD_LIST_MESHES,
            EmptyData,
            PointcloudListMeshesRes,
            self._srv_list_meshes,
        )
        # depth frame subscriber
        self.create_raw_subscriber(
            Topic.CAMERA_DEPTH_FRAME, self._on_depth_frame)

        super().start()
        self._cache.subscribe(self)

        self._stream_thread = threading.Thread(
            target=self._stream_loop,
            name="pointcloud-stream",
            daemon=True,
        )
        self._stream_thread.start()
        self._publish_state()

    # ─── Subscriber ──────────────────────────────────────────

    def _on_depth_frame(self, payload: bytes) -> None:
        try:
            frame = decode_depth_frame(payload)
        except Exception as e:
            logger.warning(f"depth_frame 디코드 실패: {e}")
            return
        with self._frame_lock:
            self._latest_frame = frame

    # ─── Service: configure ──────────────────────────────────

    def _srv_configure(
        self, req: ServiceRequest[PointcloudConfigureReq]
    ) -> ServiceResponse[PointcloudConfigureRes]:
        data = req.data

        if data.voxel_size is not None:
            if data.voxel_size <= 0:
                return ServiceResponse(
                    success=False, message="voxel_size > 0 필요", data=None
                )
            with self._cfg_lock:
                self._voxel_size = data.voxel_size

        if data.enabled is not None:
            target = data.enabled
            res = self.call_service(
                Service.CAMERA_SET_DEPTH_STREAM,
                CameraSetDepthStreamReq(enabled=target),
                CameraSetDepthStreamRes,
            )
            if not res.success:
                return ServiceResponse(
                    success=False,
                    message=f"카메라 depth 스트림 전환 실패: {res.message}",
                    data=None,
                )
            with self._cfg_lock:
                self._enabled = target
            if not target:
                with self._frame_lock:
                    self._latest_frame = None

        with self._cfg_lock:
            state = PointcloudConfigureRes(
                enabled=self._enabled,
                voxel_size=self._voxel_size,
            )
        self._publish_state()
        return ServiceResponse(success=True, message="ok", data=state)

    def _publish_state(self) -> None:
        with self._cfg_lock:
            self.publish(
                Topic.POINTCLOUD_STATE,
                PointcloudState(
                    timestamp=time.time(),
                    enabled=self._enabled,
                    voxel_size=self._voxel_size,
                ),
            )

    # ─── Service: capture ────────────────────────────────────

    def _srv_new_session(
        self, req: ServiceRequest[PointcloudNewSessionReq]
    ) -> ServiceResponse[PointcloudNewSessionRes]:
        sid_raw = req.data.session_id.strip()
        try:
            sid = (
                scan_io.validate_session_id(sid_raw)
                if sid_raw
                else scan_io.make_default_session_id()
            )
        except ValueError as e:
            return ServiceResponse(success=False, message=str(e), data=None)

        sdir = scan_io.session_dir(sid)
        sdir.mkdir(parents=True, exist_ok=True)
        return ServiceResponse(
            success=True,
            message=f"세션 생성: {sid}",
            data=PointcloudNewSessionRes(session_id=sid),
        )

    def _srv_capture(
        self, req: ServiceRequest[PointcloudCaptureReq]
    ) -> ServiceResponse[PointcloudCaptureRes]:
        if not self._capture_lock.acquire(blocking=False):
            return ServiceResponse(
                success=False,
                message="다른 capture/build 진행 중",
                data=None,
            )
        try:
            try:
                sid = scan_io.validate_session_id(req.data.session_id)
            except ValueError as e:
                return ServiceResponse(success=False, message=str(e), data=None)
            num_frames = (
                req.data.num_frames
                if req.data.num_frames is not None
                else scan_capture.N_FRAMES_DEFAULT
            )

            with self._cfg_lock:
                enabled = self._enabled
            if not enabled:
                return ServiceResponse(
                    success=False,
                    message="depth 스트림 OFF — 먼저 enable",
                    data=None,
                )

            def _get_frame() -> DepthFrame | None:
                with self._frame_lock:
                    return self._latest_frame

            try:
                frames = scan_capture.gather_frames(_get_frame, n=num_frames)
            except TimeoutError as e:
                return ServiceResponse(success=False, message=str(e), data=None)

            depth_z16 = scan_capture.consensus_depth(frames)
            color_bgr = scan_capture.consensus_color(frames)

            raw_dict = self._cache.get_raw_motor_positions(self._arm_cfgs)
            if raw_dict is None:
                return ServiceResponse(
                    success=False,
                    message="motor state 없음 — motor 노드 확인",
                    data=None,
                )
            arm_motor_ids = [cfg.id for cfg in self._arm_cfgs]
            raw_positions = [raw_dict[mid] for mid in arm_motor_ids]

            sdir = scan_io.session_dir(sid)
            sdir.mkdir(parents=True, exist_ok=True)
            scan_id = scan_io.allocate_scan_id(sdir)
            scan_path = scan_io.scan_path_for_id(sdir, scan_id)

            f0 = frames[0]
            scan_io.save_scan(
                scan_path,
                scan_id=scan_id,
                color_bgr=color_bgr,
                depth_z16=depth_z16,
                fx=f0.fx,
                fy=f0.fy,
                cx=f0.cx,
                cy=f0.cy,
                width=f0.width,
                height=f0.height,
                depth_scale=f0.depth_scale,
                raw_motor_positions=raw_positions,
                arm_motor_ids=arm_motor_ids,
                num_frames=len(frames),
            )
            return ServiceResponse(
                success=True,
                message=f"scan_{scan_id:03d}.npz 저장",
                data=PointcloudCaptureRes(
                    session_id=sid,
                    scan_id=scan_id,
                    path=scan_path.relative_to(scan_io.robot_root()).as_posix(),
                    num_frames=len(frames),
                ),
            )
        except Exception as e:
            logger.exception("capture 실패")
            return ServiceResponse(success=False, message=str(e), data=None)
        finally:
            self._capture_lock.release()

    def _srv_list_sessions(
        self, _req: ServiceRequest[EmptyData]
    ) -> ServiceResponse[PointcloudListSessionsRes]:
        return ServiceResponse(
            success=True,
            message="ok",
            data=PointcloudListSessionsRes(sessions=scan_io.list_session_ids()),
        )

    def _srv_list_scans(
        self, req: ServiceRequest[PointcloudListScansReq]
    ) -> ServiceResponse[PointcloudListScansRes]:
        try:
            sid = scan_io.validate_session_id(req.data.session_id)
        except ValueError as e:
            return ServiceResponse(success=False, message=str(e), data=None)
        sdir = scan_io.session_dir(sid)
        scans: list[ScanMeta] = []
        for p in scan_io.list_scans(sdir):
            try:
                scans.append(ScanMeta.model_validate(scan_io.scan_meta(p)))
            except Exception as e:
                logger.warning("scan meta 실패 (%s): %s", p, e)
        return ServiceResponse(
            success=True,
            message="ok",
            data=PointcloudListScansRes(session_id=sid, scans=scans),
        )

    def _srv_delete_scan(
        self, req: ServiceRequest[PointcloudDeleteScanReq]
    ) -> ServiceResponse[PointcloudDeleteScanRes]:
        try:
            sid = scan_io.validate_session_id(req.data.session_id)
        except ValueError as e:
            return ServiceResponse(success=False, message=str(e), data=None)
        scan_id = req.data.scan_id
        if scan_id < 0:
            return ServiceResponse(
                success=False, message="scan_id 필요", data=None
            )
        sdir = scan_io.session_dir(sid)
        ok = scan_io.delete_scan(sdir, scan_id)
        if not ok:
            return ServiceResponse(
                success=False,
                message=f"scan_{scan_id:03d}.npz 없음",
                data=None,
            )
        return ServiceResponse(
            success=True,
            message=f"scan_{scan_id:03d} 삭제",
            data=PointcloudDeleteScanRes(session_id=sid, scan_id=scan_id),
        )

    # ─── Service: TSDF build ─────────────────────────────────

    def _srv_build_mesh(
        self, req: ServiceRequest[PointcloudBuildMeshReq]
    ) -> ServiceResponse[PointcloudBuildMeshRes]:
        if not self._capture_lock.acquire(blocking=False):
            return ServiceResponse(
                success=False,
                message="다른 capture/build 진행 중",
                data=None,
            )
        try:
            try:
                sid = scan_io.validate_session_id(req.data.session_id)
            except ValueError as e:
                return ServiceResponse(success=False, message=str(e), data=None)

            sdir = scan_io.session_dir(sid)
            npz_paths = scan_io.list_scans(sdir)
            if len(npz_paths) < tsdf_builder.MIN_SCANS:
                return ServiceResponse(
                    success=False,
                    message=(
                        f"scan {tsdf_builder.MIN_SCANS}개 이상 필요 "
                        f"(현재 {len(npz_paths)})"
                    ),
                    data=None,
                )

            scans = [scan_io.load_scan(p) for p in npz_paths]
            out_path = scan_io.meshes_dir() / f"mesh_{sid}.ply"

            t0 = time.time()
            result = tsdf_builder.build_mesh(
                scans,
                self._arm_cfgs,
                out_path,
                voxel_size=(
                    req.data.voxel_size
                    if req.data.voxel_size is not None
                    else tsdf_builder.DEFAULT_VOXEL_SIZE
                ),
                sdf_trunc=(
                    req.data.sdf_trunc
                    if req.data.sdf_trunc is not None
                    else tsdf_builder.DEFAULT_SDF_TRUNC
                ),
                depth_trunc=(
                    req.data.depth_trunc
                    if req.data.depth_trunc is not None
                    else tsdf_builder.DEFAULT_DEPTH_TRUNC
                ),
                icp_max_dist=(
                    req.data.icp_max_dist
                    if req.data.icp_max_dist is not None
                    else tsdf_builder.DEFAULT_ICP_MAX_DIST
                ),
            )
            elapsed = time.time() - t0

            return ServiceResponse(
                success=True,
                message=(
                    f"mesh: {result.vertex_count} vertices, "
                    f"{result.triangle_count} triangles ({elapsed:.1f}s)"
                ),
                data=PointcloudBuildMeshRes(
                    session_id=sid,
                    path=out_path.relative_to(scan_io.robot_root()).as_posix(),
                    vertex_count=result.vertex_count,
                    triangle_count=result.triangle_count,
                    n_scans=result.n_scans,
                    n_edges=result.n_edges,
                    elapsed=elapsed,
                ),
            )
        except Exception as e:
            logger.exception("build_mesh 실패")
            return ServiceResponse(success=False, message=str(e), data=None)
        finally:
            self._capture_lock.release()

    def _srv_list_meshes(
        self, _req: ServiceRequest[EmptyData]
    ) -> ServiceResponse[PointcloudListMeshesRes]:
        scan_io.meshes_dir().mkdir(parents=True, exist_ok=True)
        meshes: list[MeshMeta] = []
        for p in sorted(scan_io.meshes_dir().glob("mesh_*.ply")):
            try:
                stat = p.stat()
                sid = p.stem[len("mesh_"):]
                meshes.append(MeshMeta(
                    session_id=sid,
                    path=p.relative_to(scan_io.robot_root()).as_posix(),
                    size=int(stat.st_size),
                    mtime=float(stat.st_mtime),
                ))
            except Exception as e:
                logger.warning("mesh meta 실패 (%s): %s", p, e)
        return ServiceResponse(
            success=True,
            message="ok",
            data=PointcloudListMeshesRes(meshes=meshes),
        )

    # ─── Stream Loop (라이브 PC) ─────────────────────────────

    def _stream_loop(self) -> None:
        period = 1.0 / TARGET_FPS
        last_processed_ts = 0.0

        while self._running:
            with self._cfg_lock:
                enabled = self._enabled
                voxel = self._voxel_size

            if not enabled:
                time.sleep(IDLE_SLEEP)
                continue

            with self._frame_lock:
                frame = self._latest_frame

            if frame is None or frame.timestamp <= last_processed_ts:
                time.sleep(IDLE_SLEEP)
                continue

            t0 = time.time()
            try:
                payload = self._build_payload(frame, voxel)
            except Exception as e:
                logger.warning(f"포인트클라우드 생성 실패: {e}")
                time.sleep(IDLE_SLEEP)
                continue

            try:
                self.session.put(Topic.POINTCLOUD_STREAM, payload)
                last_processed_ts = frame.timestamp
            except Exception as e:
                logger.warning(f"포인트클라우드 발행 실패: {e}")

            elapsed = time.time() - t0
            time.sleep(max(0.0, period - elapsed))

    # ─── Cloud build (라이브) ────────────────────────────────

    def _build_payload(self, frame: DepthFrame, voxel_size: float) -> bytes:
        pcd = _build_pcd(frame)
        if voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size)

        xyz = np.asarray(pcd.points, dtype=np.float32)
        rgb_f = np.asarray(pcd.colors, dtype=np.float32)
        rgb_u8 = (np.clip(rgb_f, 0.0, 1.0) * 255.0).astype(np.uint8)

        n = xyz.shape[0]
        return struct.pack("<I", n) + xyz.tobytes() + rgb_u8.tobytes()


def _build_pcd(frame: DepthFrame) -> "o3d.geometry.PointCloud":
    rgb = np.ascontiguousarray(frame.color_bgr[:, :, ::-1])
    color_o3d = o3d.geometry.Image(rgb)
    depth_o3d = o3d.geometry.Image(frame.depth_z16)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d,
        depth_o3d,
        depth_scale=1.0 / frame.depth_scale,
        depth_trunc=DEPTH_TRUNC,
        convert_rgb_to_intensity=False,
    )
    pinhole = o3d.camera.PinholeCameraIntrinsic(
        width=frame.width,
        height=frame.height,
        fx=frame.fx,
        fy=frame.fy,
        cx=frame.cx,
        cy=frame.cy,
    )
    return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole)
