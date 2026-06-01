import logging
import struct
import threading
import time

import numpy as np
import open3d as o3d

from core.base_node import BaseNode
from core.common import GRIPPER_ID
from core.joint_state_cache import JointStateCache
from core.topic_map import Service, Topic
from modules.camera.depth_frame import DepthFrame, decode as decode_depth_frame
from modules.dynamixel.motor_config import load_motor_config
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
        self.create_service(Service.POINTCLOUD_CONFIGURE, self._srv_configure)
        # capture
        self.create_service(
            Service.POINTCLOUD_NEW_SESSION, self._srv_new_session)
        self.create_service(Service.POINTCLOUD_CAPTURE, self._srv_capture)
        self.create_service(
            Service.POINTCLOUD_LIST_SESSIONS, self._srv_list_sessions)
        self.create_service(
            Service.POINTCLOUD_LIST_SCANS, self._srv_list_scans)
        self.create_service(
            Service.POINTCLOUD_DELETE_SCAN, self._srv_delete_scan)
        # TSDF
        self.create_service(
            Service.POINTCLOUD_BUILD_MESH, self._srv_build_mesh)
        self.create_service(
            Service.POINTCLOUD_LIST_MESHES, self._srv_list_meshes)
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

    def _srv_configure(self, req: dict) -> dict:
        data = req.get("data", {}) or {}

        if "voxel_size" in data:
            v = float(data["voxel_size"])
            if v <= 0:
                return {
                    "success": False,
                    "message": "voxel_size > 0 필요",
                    "data": {},
                }
            with self._cfg_lock:
                self._voxel_size = v

        if "enabled" in data:
            target = bool(data["enabled"])
            res = self.call_service(
                Service.CAMERA_SET_DEPTH_STREAM,
                {"enabled": target},
            )
            if not res.get("success"):
                return {
                    "success": False,
                    "message": f"카메라 depth 스트림 전환 실패: {res.get('message')}",
                    "data": {},
                }
            with self._cfg_lock:
                self._enabled = target
            if not target:
                with self._frame_lock:
                    self._latest_frame = None

        with self._cfg_lock:
            state = {
                "enabled": self._enabled,
                "voxel_size": self._voxel_size,
            }
        self._publish_state()
        return {"success": True, "message": "ok", "data": state}

    def _publish_state(self) -> None:
        with self._cfg_lock:
            self.publish(
                Topic.POINTCLOUD_STATE,
                {
                    "timestamp": time.time(),
                    "enabled": self._enabled,
                    "voxel_size": self._voxel_size,
                },
            )

    # ─── Service: capture ────────────────────────────────────

    def _srv_new_session(self, req: dict) -> dict:
        data = req.get("data", {}) or {}
        sid_raw = str(data.get("session_id", "")).strip()
        try:
            sid = (
                scan_io.validate_session_id(sid_raw)
                if sid_raw
                else scan_io.make_default_session_id()
            )
        except ValueError as e:
            return {"success": False, "message": str(e), "data": {}}

        sdir = scan_io.session_dir(sid)
        sdir.mkdir(parents=True, exist_ok=True)
        return {
            "success": True,
            "message": f"세션 생성: {sid}",
            "data": {"session_id": sid},
        }

    def _srv_capture(self, req: dict) -> dict:
        """{ session_id, num_frames? }"""
        if not self._capture_lock.acquire(blocking=False):
            return {
                "success": False,
                "message": "다른 capture/build 진행 중",
                "data": {},
            }
        try:
            data = req.get("data", {}) or {}
            try:
                sid = scan_io.validate_session_id(
                    str(data.get("session_id", "")))
            except ValueError as e:
                return {"success": False, "message": str(e), "data": {}}
            num_frames = int(
                data.get("num_frames", scan_capture.N_FRAMES_DEFAULT))

            with self._cfg_lock:
                enabled = self._enabled
            if not enabled:
                return {
                    "success": False,
                    "message": "depth 스트림 OFF — 먼저 enable",
                    "data": {},
                }

            def _get_frame() -> DepthFrame | None:
                with self._frame_lock:
                    return self._latest_frame

            try:
                frames = scan_capture.gather_frames(_get_frame, n=num_frames)
            except TimeoutError as e:
                return {"success": False, "message": str(e), "data": {}}

            depth_z16 = scan_capture.consensus_depth(frames)
            color_bgr = scan_capture.consensus_color(frames)

            raw_dict = self._cache.get_raw_motor_positions(self._arm_cfgs)
            if raw_dict is None:
                return {
                    "success": False,
                    "message": "motor state 없음 — motor 노드 확인",
                    "data": {},
                }
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
            return {
                "success": True,
                "message": f"scan_{scan_id:03d}.npz 저장",
                "data": {
                    "session_id": sid,
                    "scan_id": scan_id,
                    "path": scan_path.relative_to(
                        scan_io.robot_root()).as_posix(),
                    "num_frames": len(frames),
                },
            }
        except Exception as e:
            logger.exception("capture 실패")
            return {"success": False, "message": str(e), "data": {}}
        finally:
            self._capture_lock.release()

    def _srv_list_sessions(self, _req: dict) -> dict:
        sessions = scan_io.list_session_ids()
        return {
            "success": True,
            "message": "ok",
            "data": {"sessions": sessions},
        }

    def _srv_list_scans(self, req: dict) -> dict:
        data = req.get("data", {}) or {}
        try:
            sid = scan_io.validate_session_id(
                str(data.get("session_id", "")))
        except ValueError as e:
            return {"success": False, "message": str(e), "data": {}}
        sdir = scan_io.session_dir(sid)
        scans = []
        for p in scan_io.list_scans(sdir):
            try:
                scans.append(scan_io.scan_meta(p))
            except Exception as e:
                logger.warning("scan meta 실패 (%s): %s", p, e)
        return {
            "success": True,
            "message": "ok",
            "data": {"session_id": sid, "scans": scans},
        }

    def _srv_delete_scan(self, req: dict) -> dict:
        data = req.get("data", {}) or {}
        try:
            sid = scan_io.validate_session_id(
                str(data.get("session_id", "")))
        except ValueError as e:
            return {"success": False, "message": str(e), "data": {}}
        scan_id = int(data.get("scan_id", -1))
        if scan_id < 0:
            return {
                "success": False,
                "message": "scan_id 필요",
                "data": {},
            }
        sdir = scan_io.session_dir(sid)
        ok = scan_io.delete_scan(sdir, scan_id)
        if not ok:
            return {
                "success": False,
                "message": f"scan_{scan_id:03d}.npz 없음",
                "data": {},
            }
        return {
            "success": True,
            "message": f"scan_{scan_id:03d} 삭제",
            "data": {"session_id": sid, "scan_id": scan_id},
        }

    # ─── Service: TSDF build ─────────────────────────────────

    def _srv_build_mesh(self, req: dict) -> dict:
        """{ session_id, voxel_size?, sdf_trunc?, depth_trunc?, icp_max_dist? }"""
        if not self._capture_lock.acquire(blocking=False):
            return {
                "success": False,
                "message": "다른 capture/build 진행 중",
                "data": {},
            }
        try:
            data = req.get("data", {}) or {}
            try:
                sid = scan_io.validate_session_id(
                    str(data.get("session_id", "")))
            except ValueError as e:
                return {"success": False, "message": str(e), "data": {}}

            sdir = scan_io.session_dir(sid)
            npz_paths = scan_io.list_scans(sdir)
            if len(npz_paths) < tsdf_builder.MIN_SCANS:
                return {
                    "success": False,
                    "message": (
                        f"scan {tsdf_builder.MIN_SCANS}개 이상 필요 "
                        f"(현재 {len(npz_paths)})"
                    ),
                    "data": {},
                }

            scans = [scan_io.load_scan(p) for p in npz_paths]
            out_path = scan_io.meshes_dir() / f"mesh_{sid}.ply"

            t0 = time.time()
            result = tsdf_builder.build_mesh(
                scans,
                self._arm_cfgs,
                out_path,
                voxel_size=float(
                    data.get("voxel_size", tsdf_builder.DEFAULT_VOXEL_SIZE)),
                sdf_trunc=float(
                    data.get("sdf_trunc", tsdf_builder.DEFAULT_SDF_TRUNC)),
                depth_trunc=float(
                    data.get("depth_trunc", tsdf_builder.DEFAULT_DEPTH_TRUNC)),
                icp_max_dist=float(
                    data.get("icp_max_dist",
                             tsdf_builder.DEFAULT_ICP_MAX_DIST)),
            )
            elapsed = time.time() - t0

            return {
                "success": True,
                "message": (
                    f"mesh: {result.vertex_count} vertices, "
                    f"{result.triangle_count} triangles ({elapsed:.1f}s)"
                ),
                "data": {
                    "session_id": sid,
                    "path": out_path.relative_to(
                        scan_io.robot_root()).as_posix(),
                    "vertex_count": result.vertex_count,
                    "triangle_count": result.triangle_count,
                    "n_scans": result.n_scans,
                    "n_edges": result.n_edges,
                    "elapsed": elapsed,
                },
            }
        except Exception as e:
            logger.exception("build_mesh 실패")
            return {"success": False, "message": str(e), "data": {}}
        finally:
            self._capture_lock.release()

    def _srv_list_meshes(self, _req: dict) -> dict:
        scan_io.meshes_dir().mkdir(parents=True, exist_ok=True)
        meshes = []
        for p in sorted(scan_io.meshes_dir().glob("mesh_*.ply")):
            try:
                stat = p.stat()
                sid = p.stem[len("mesh_"):]
                meshes.append({
                    "session_id": sid,
                    "path": p.relative_to(scan_io.robot_root()).as_posix(),
                    "size": int(stat.st_size),
                    "mtime": float(stat.st_mtime),
                })
            except Exception as e:
                logger.warning("mesh meta 실패 (%s): %s", p, e)
        return {
            "success": True,
            "message": "ok",
            "data": {"meshes": meshes},
        }

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
