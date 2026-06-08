import logging
import struct
import threading
import time

import numpy as np
import open3d as o3d

from core.transport.base_node import BaseNode
from core.common import GRIPPER_ID
from core.cache.joint_state_cache import JointStateCache
from core.robot.robot_registry import RobotRegistry
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
from core.transport.topic_map import Service, Topic, topic_for
from modules.camera.depth_frame import DepthFrame, decode as decode_depth_frame
from modules.motor.motor_config import MotorConfig, load_motor_config
from modules.pointcloud import scan_capture, scan_io, tsdf_builder

logger = logging.getLogger(__name__)

DEFAULT_VOXEL_SIZE = 0.005  # 5mm — 라이브 스트림용 (TSDF는 별도 voxel)
TARGET_FPS = 8.0
IDLE_SLEEP = 0.1
DEPTH_TRUNC = 1.0  # m


class _RobotState:
    """robot 별 PointCloud 상태."""

    def __init__(self, arm_cfgs: list[MotorConfig]) -> None:
        self.arm_cfgs = arm_cfgs
        self.cfg_lock = threading.Lock()
        self.frame_lock = threading.Lock()
        self.capture_lock = threading.Lock()
        self.enabled = False
        self.voxel_size = DEFAULT_VOXEL_SIZE
        self.latest_frame: DepthFrame | None = None


class PointCloudNode(BaseNode):
    """SYSTEM 노드 — robot 무관 한 인스턴스. robot 별 dict[robot_id] state."""

    def __init__(self) -> None:
        super().__init__("pointcloud_node", robot_id=None)
        self._registry = RobotRegistry()
        self._enabled_robot_ids: list[str] = [
            c.robot_id for c in self._registry.enabled_robots()
        ]
        self._states: dict[str, _RobotState] = {}
        for rid in self._enabled_robot_ids:
            _, motor_cfgs = load_motor_config(rid)
            arm_cfgs = [m for m in motor_cfgs if m.id != GRIPPER_ID]
            self._states[rid] = _RobotState(arm_cfgs)

        self._cache = JointStateCache()

        self._stream_thread: threading.Thread | None = None

    def start(self) -> None:
        for rid in self._enabled_robot_ids:
            # configure
            self.create_service(
                topic_for(Service.POINTCLOUD_CONFIGURE, rid),
                PointcloudConfigureReq,
                PointcloudConfigureRes,
                lambda req, _rid=rid: self._srv_configure(req, _rid),
            )
            # capture
            self.create_service(
                topic_for(Service.POINTCLOUD_NEW_SESSION, rid),
                PointcloudNewSessionReq,
                PointcloudNewSessionRes,
                lambda req, _rid=rid: self._srv_new_session(req, _rid),
            )
            self.create_service(
                topic_for(Service.POINTCLOUD_CAPTURE, rid),
                PointcloudCaptureReq,
                PointcloudCaptureRes,
                lambda req, _rid=rid: self._srv_capture(req, _rid),
            )
            self.create_service(
                topic_for(Service.POINTCLOUD_LIST_SESSIONS, rid),
                EmptyData,
                PointcloudListSessionsRes,
                lambda req, _rid=rid: self._srv_list_sessions(req, _rid),
            )
            self.create_service(
                topic_for(Service.POINTCLOUD_LIST_SCANS, rid),
                PointcloudListScansReq,
                PointcloudListScansRes,
                lambda req, _rid=rid: self._srv_list_scans(req, _rid),
            )
            self.create_service(
                topic_for(Service.POINTCLOUD_DELETE_SCAN, rid),
                PointcloudDeleteScanReq,
                PointcloudDeleteScanRes,
                lambda req, _rid=rid: self._srv_delete_scan(req, _rid),
            )
            # TSDF
            self.create_service(
                topic_for(Service.POINTCLOUD_BUILD_MESH, rid),
                PointcloudBuildMeshReq,
                PointcloudBuildMeshRes,
                lambda req, _rid=rid: self._srv_build_mesh(req, _rid),
            )
            self.create_service(
                topic_for(Service.POINTCLOUD_LIST_MESHES, rid),
                EmptyData,
                PointcloudListMeshesRes,
                lambda req, _rid=rid: self._srv_list_meshes(req, _rid),
            )
            # depth frame subscriber
            self.create_raw_subscriber(
                topic_for(Topic.CAMERA_DEPTH_FRAME, rid),
                lambda payload, _rid=rid: self._on_depth_frame(_rid, payload),
            )

        super().start()
        self._cache.subscribe(self)

        self._stream_thread = threading.Thread(
            target=self._stream_loop,
            name="pointcloud-stream",
            daemon=True,
        )
        self._stream_thread.start()
        for rid in self._enabled_robot_ids:
            self._publish_state(rid)

        logger.info(
            "PointCloudNode 시작 (robots=%s)", self._enabled_robot_ids
        )

    # ─── Subscriber ──────────────────────────────────────────

    def _on_depth_frame(self, robot_id: str, payload: bytes) -> None:
        try:
            frame = decode_depth_frame(payload)
        except Exception as e:
            logger.warning(f"depth_frame[{robot_id}] 디코드 실패: {e}")
            return
        st = self._states[robot_id]
        with st.frame_lock:
            st.latest_frame = frame

    # ─── Service: configure ──────────────────────────────────

    def _srv_configure(
        self, req: ServiceRequest[PointcloudConfigureReq], robot_id: str
    ) -> ServiceResponse[PointcloudConfigureRes]:
        st = self._states[robot_id]
        data = req.data

        if data.voxel_size is not None:
            if data.voxel_size <= 0:
                return ServiceResponse(
                    success=False, message="voxel_size > 0 필요", data=None
                )
            with st.cfg_lock:
                st.voxel_size = data.voxel_size

        if data.enabled is not None:
            target = data.enabled
            res = self.call_service(
                topic_for(Service.CAMERA_SET_DEPTH_STREAM, robot_id),
                CameraSetDepthStreamReq(enabled=target),
                CameraSetDepthStreamRes,
            )
            if not res.success:
                return ServiceResponse(
                    success=False,
                    message=f"카메라 depth 스트림 전환 실패: {res.message}",
                    data=None,
                )
            with st.cfg_lock:
                st.enabled = target
            if not target:
                with st.frame_lock:
                    st.latest_frame = None

        with st.cfg_lock:
            state = PointcloudConfigureRes(
                enabled=st.enabled,
                voxel_size=st.voxel_size,
            )
        self._publish_state(robot_id)
        return ServiceResponse(success=True, message="ok", data=state)

    def _publish_state(self, robot_id: str) -> None:
        st = self._states[robot_id]
        with st.cfg_lock:
            self.publish(
                topic_for(Topic.POINTCLOUD_STATE, robot_id),
                PointcloudState(
                    timestamp=time.time(),
                    enabled=st.enabled,
                    voxel_size=st.voxel_size,
                ),
            )

    # ─── Service: capture ────────────────────────────────────

    def _srv_new_session(
        self, req: ServiceRequest[PointcloudNewSessionReq], robot_id: str
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

        sdir = scan_io.session_dir(robot_id, sid)
        sdir.mkdir(parents=True, exist_ok=True)
        return ServiceResponse(
            success=True,
            message=f"세션 생성: {sid}",
            data=PointcloudNewSessionRes(session_id=sid),
        )

    def _srv_capture(
        self, req: ServiceRequest[PointcloudCaptureReq], robot_id: str
    ) -> ServiceResponse[PointcloudCaptureRes]:
        st = self._states[robot_id]
        if not st.capture_lock.acquire(blocking=False):
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

            with st.cfg_lock:
                enabled = st.enabled
            if not enabled:
                return ServiceResponse(
                    success=False,
                    message="depth 스트림 OFF — 먼저 enable",
                    data=None,
                )

            def _get_frame() -> DepthFrame | None:
                with st.frame_lock:
                    return st.latest_frame

            try:
                frames = scan_capture.gather_frames(_get_frame, n=num_frames)
            except TimeoutError as e:
                return ServiceResponse(success=False, message=str(e), data=None)

            depth_z16 = scan_capture.consensus_depth(frames)
            color_bgr = scan_capture.consensus_color(frames)

            raw_dict = self._cache.get_raw_motor_positions(
                st.arm_cfgs, robot_id=robot_id
            )
            if raw_dict is None:
                return ServiceResponse(
                    success=False,
                    message="motor state 없음 — motor 노드 확인",
                    data=None,
                )
            arm_motor_ids = [cfg.id for cfg in st.arm_cfgs]
            raw_positions = [raw_dict[mid] for mid in arm_motor_ids]

            sdir = scan_io.session_dir(robot_id, sid)
            sdir.mkdir(parents=True, exist_ok=True)
            scan_id = scan_io.allocate_scan_id(sdir)
            scan_path = scan_io.scan_path_for_id(sdir, scan_id)

            f0 = frames[0]
            scan_io.save_scan(
                scan_path,
                robot_id=robot_id,
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
            logger.exception("[%s] capture 실패", robot_id)
            return ServiceResponse(success=False, message=str(e), data=None)
        finally:
            st.capture_lock.release()

    def _srv_list_sessions(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[PointcloudListSessionsRes]:
        return ServiceResponse(
            success=True,
            message="ok",
            data=PointcloudListSessionsRes(
                sessions=scan_io.list_session_ids(robot_id)
            ),
        )

    def _srv_list_scans(
        self, req: ServiceRequest[PointcloudListScansReq], robot_id: str
    ) -> ServiceResponse[PointcloudListScansRes]:
        try:
            sid = scan_io.validate_session_id(req.data.session_id)
        except ValueError as e:
            return ServiceResponse(success=False, message=str(e), data=None)
        sdir = scan_io.session_dir(robot_id, sid)
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
        self, req: ServiceRequest[PointcloudDeleteScanReq], robot_id: str
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
        sdir = scan_io.session_dir(robot_id, sid)
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
        self, req: ServiceRequest[PointcloudBuildMeshReq], robot_id: str
    ) -> ServiceResponse[PointcloudBuildMeshRes]:
        st = self._states[robot_id]
        if not st.capture_lock.acquire(blocking=False):
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

            sdir = scan_io.session_dir(robot_id, sid)
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
            out_path = scan_io.meshes_dir(robot_id) / f"mesh_{sid}.ply"

            t0 = time.time()
            result = tsdf_builder.build_mesh(
                scans,
                st.arm_cfgs,
                out_path,
                robot_id=robot_id,
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
            logger.exception("[%s] build_mesh 실패", robot_id)
            return ServiceResponse(success=False, message=str(e), data=None)
        finally:
            st.capture_lock.release()

    def _srv_list_meshes(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[PointcloudListMeshesRes]:
        meshes_dir = scan_io.meshes_dir(robot_id)
        meshes_dir.mkdir(parents=True, exist_ok=True)
        meshes: list[MeshMeta] = []
        for p in sorted(meshes_dir.glob("mesh_*.ply")):
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

    # ─── Stream Loop (라이브 PC) — 모든 enabled robot ────────

    def _stream_loop(self) -> None:
        period = 1.0 / TARGET_FPS
        last_processed_ts: dict[str, float] = {
            rid: 0.0 for rid in self._enabled_robot_ids
        }

        while self._running:
            any_processed = False
            for rid in self._enabled_robot_ids:
                st = self._states[rid]
                with st.cfg_lock:
                    enabled = st.enabled
                    voxel = st.voxel_size

                if not enabled:
                    continue

                with st.frame_lock:
                    frame = st.latest_frame

                if frame is None or frame.timestamp <= last_processed_ts[rid]:
                    continue

                t0 = time.time()
                try:
                    payload = self._build_payload(frame, voxel)
                except Exception as e:
                    logger.warning(f"[{rid}] 포인트클라우드 생성 실패: {e}")
                    continue

                try:
                    self.session.put(
                        topic_for(Topic.POINTCLOUD_STREAM, rid), payload
                    )
                    last_processed_ts[rid] = frame.timestamp
                    any_processed = True
                except Exception as e:
                    logger.warning(f"[{rid}] 포인트클라우드 발행 실패: {e}")

                # robot 간 fairness 위해 한 robot 처리 후 다음으로
                elapsed = time.time() - t0
                if elapsed > period:
                    break

            if not any_processed:
                time.sleep(IDLE_SLEEP)
            else:
                time.sleep(max(0.0, period / max(1, len(self._enabled_robot_ids))))

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
