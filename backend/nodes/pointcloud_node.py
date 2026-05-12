import logging
import struct
import threading
import time

import numpy as np
import open3d as o3d

from core.base_node import BaseNode
from core.topic_map import Service, Topic
from modules.camera.depth_frame import DepthFrame, decode as decode_depth_frame

logger = logging.getLogger(__name__)

DEFAULT_VOXEL_SIZE = 0.005  # 5mm
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

    def start(self) -> None:
        self.create_service(Service.POINTCLOUD_CONFIGURE, self._srv_configure)
        self.create_raw_subscriber(
            Topic.CAMERA_DEPTH_FRAME, self._on_depth_frame)
        super().start()
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

    # ─── Service ─────────────────────────────────────────────

    def _srv_configure(self, req: dict) -> dict:
        data = req.get("data", {}) or {}

        # voxel_size — 로컬 상태
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

        # enabled — 카메라 호스트로 forward
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
            # 비활성이면 캐시된 프레임도 비움 (오래된 프레임으로 cloud 만드는 거 방지)
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

    # ─── Stream Loop ─────────────────────────────────────────

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

    # ─── Cloud build ─────────────────────────────────────────

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
