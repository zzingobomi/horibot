import logging
import struct
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d

from core.base_node import BaseNode
from core.realsense_capture import RealsenseCapture
from core.topic_map import Service, Topic
from modules.calibration.loader import load_calibration

logger = logging.getLogger(__name__)

DEFAULT_VOXEL_SIZE = 0.008  # 8mm
TARGET_FPS = 8.0
IDLE_SLEEP = 0.1
DEPTH_TRUNC = 1.5  # m


class PointCloudNode(BaseNode):
    def __init__(self) -> None:
        super().__init__("pointcloud_node")
        self._rs = RealsenseCapture()
        self._cfg_lock = threading.Lock()
        self._enabled = False
        self._voxel_size = DEFAULT_VOXEL_SIZE
        self._stream_thread: threading.Thread | None = None

    def start(self) -> None:
        self.create_service(Service.POINTCLOUD_CONFIGURE, self._srv_configure)
        super().start()
        self._stream_thread = threading.Thread(
            target=self._stream_loop,
            name="pointcloud-stream",
            daemon=True,
        )
        self._stream_thread.start()
        self._publish_state()

    # ─── Service ─────────────────────────────────────────────

    def _srv_configure(self, req: dict) -> dict:
        data = req.get("data", {}) or {}
        with self._cfg_lock:
            if "enabled" in data:
                self._enabled = bool(data["enabled"])
                self._rs.set_cloud_enabled(self._enabled)
            if "voxel_size" in data:
                v = float(data["voxel_size"])
                if v <= 0:
                    return {
                        "success": False,
                        "message": "voxel_size > 0 필요",
                        "data": {},
                    }
                self._voxel_size = v
            state = {
                "enabled": self._enabled,
                "voxel_size": self._voxel_size,
            }
        self._publish_state()
        return {"success": True, "message": "ok", "data": state}

    def _publish_state(self) -> None:
        with self._cfg_lock:
            self.publish(Topic.POINTCLOUD_STATE, {
                "timestamp": time.time(),
                "enabled": self._enabled,
                "voxel_size": self._voxel_size,
            })

    # ─── Stream Loop ─────────────────────────────────────────

    def _stream_loop(self) -> None:
        period = 1.0 / TARGET_FPS
        while self._running:
            with self._cfg_lock:
                enabled = self._enabled
                voxel = self._voxel_size

            if not enabled or not self._rs.is_opened:
                time.sleep(IDLE_SLEEP)
                continue

            color, depth = self._rs.read_aligned_color_depth()
            intr = self._rs.depth_intrinsics
            if color is None or depth is None or intr is None:
                time.sleep(IDLE_SLEEP)
                continue

            t0 = time.time()
            try:
                payload = self._build_payload(
                    color, depth, intr, self._rs.depth_scale, voxel
                )
            except Exception as e:
                logger.warning(f"포인트클라우드 생성 실패: {e}")
                time.sleep(IDLE_SLEEP)
                continue

            try:
                self.session.put(Topic.POINTCLOUD_STREAM, payload)
            except Exception as e:
                logger.warning(f"포인트클라우드 발행 실패: {e}")

            elapsed = time.time() - t0
            sleep_for = max(0.0, period - elapsed)
            time.sleep(sleep_for)

    # ─── Cloud build ─────────────────────────────────────────

    def _build_payload(
        self,
        color_bgr: np.ndarray,
        depth_z16: np.ndarray,
        intr: Any,
        depth_scale: float,
        voxel_size: float,
    ) -> bytes:
        pcd = _build_pcd(color_bgr, depth_z16, intr, depth_scale)
        if voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size)

        xyz = np.asarray(pcd.points, dtype=np.float32)
        rgb_f = np.asarray(pcd.colors, dtype=np.float32)
        rgb_u8 = (np.clip(rgb_f, 0.0, 1.0) * 255.0).astype(np.uint8)

        n = xyz.shape[0]
        return struct.pack("<I", n) + xyz.tobytes() + rgb_u8.tobytes()


def _build_pcd(
    color_bgr: np.ndarray,
    depth_z16: np.ndarray,
    intr: Any,
    depth_scale: float,
) -> "o3d.geometry.PointCloud":
    rgb = np.ascontiguousarray(color_bgr[:, :, ::-1])
    color_o3d = o3d.geometry.Image(rgb)
    depth_o3d = o3d.geometry.Image(depth_z16)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d, depth_o3d,
        depth_scale=1.0 / depth_scale,
        depth_trunc=DEPTH_TRUNC,
        convert_rgb_to_intensity=False,
    )
    pinhole = o3d.camera.PinholeCameraIntrinsic(
        width=intr.width, height=intr.height,
        fx=intr.fx, fy=intr.fy, cx=intr.ppx, cy=intr.ppy,
    )
    return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole)
