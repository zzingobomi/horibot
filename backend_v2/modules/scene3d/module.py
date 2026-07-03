"""Scene3DModule — robot-scoped RGBD primitive (라이브 PC + consensus snapshot).

옛 backend/nodes/application/scene3d_node.py 의 primitive 부분 이월. v2 적응:
- camera depth on/off refcount 제거 (v2 camera 상시 depth stream).
- 라이브 PC 는 camera-frame 발행 → frontend 가 tcp·hand_eye transform (옛 패턴 유지).
- intrinsic 은 active calibration(우선) → camera factory(fallback) 에서 boot 시 pull.

subscriber(sync) 로 depth/color 캐시 → snapshot(sync service) 이 읽음. 라이브 loop
(async) 의 open3d build 는 to_thread (event loop non-block, async 계약).
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from collections import deque

import cv2
import numpy as np
import zstandard as zstd

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.contract.subscriber import subscriber
from framework.runtime.api import ModuleRuntime
from modules.calibration.contract import Calibration, CalibrationBundle, SnapshotBundleRequest
from modules.camera.contract import (
    Camera,
    CameraDecodedFrame,
    CameraDepthDecodedFrame,
    FactoryIntrinsic,
    GetFactoryIntrinsicRequest,
)

from . import pointcloud as pc
from .consensus import consensus_depth
from .contract import (
    Scene3d,
    Scene3dCloud,
    Scene3dIntrinsic,
    SetStreamRequest,
    SetStreamResponse,
    SnapshotRequest,
    SnapshotResponse,
)

logger = logging.getLogger(__name__)

_LIVE_HZ = 8.0
_DEFAULT_VOXEL = 0.005  # 5mm (라이브)
_DEPTH_BUFFER = 16  # consensus 용 최근 depth ring
_JPEG_QUALITY = 90
_ZSTD_LEVEL = 3


@publishes((Scene3d.Stream.CLOUD, Scene3dCloud))
class Scene3DModule:
    def __init__(self, runtime: ModuleRuntime, robot_id: str) -> None:
        self.runtime = runtime
        self.robot_id = robot_id

        self._lock = threading.Lock()
        self._depths: deque[np.ndarray] = deque(maxlen=_DEPTH_BUFFER)
        self._depth_scale = 0.0
        self._depth_wh: tuple[int, int] | None = None
        self._latest_color: np.ndarray | None = None
        # base intrinsic (fx/fy/cx/cy @ image_size). depth 해상도로 scale 해서 사용.
        self._base_intrinsic: Scene3dIntrinsic | None = None

        self._enabled = False
        self._voxel = _DEFAULT_VOXEL
        self._seq = 0
        self._stop = False
        self._live_task: asyncio.Task[None] | None = None
        self._zstd = zstd.ZstdCompressor(level=_ZSTD_LEVEL)

    # ── lifecycle ─────────────────────────────────────────────
    async def start(self) -> None:
        logger.info("Scene3DModule start robot=%s", self.robot_id)
        await self._fetch_intrinsic()
        self._stop = False
        self._live_task = asyncio.create_task(self._live_loop())

    async def stop(self) -> None:
        self._stop = True
        task = self._live_task
        if task is not None:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            self._live_task = None
        logger.info("Scene3DModule stop robot=%s", self.robot_id)

    async def _fetch_intrinsic(self) -> None:
        """active calibration intrinsic 우선 → camera factory fallback. cache."""
        # 1) active calibration intrinsic
        try:
            bundle = await self.runtime.call(
                Calibration.Service.SNAPSHOT_BUNDLE,
                SnapshotBundleRequest(),
                CalibrationBundle,
                robot_id=self.robot_id,
                timeout=3.0,
            )
            if bundle.intrinsic is not None:
                cm = bundle.intrinsic.result_data.camera_matrix
                size = bundle.intrinsic.result_data.image_size
                if size and len(size) == 2:
                    self._base_intrinsic = Scene3dIntrinsic(
                        width=int(size[0]),
                        height=int(size[1]),
                        fx=float(cm[0][0]),
                        fy=float(cm[1][1]),
                        cx=float(cm[0][2]),
                        cy=float(cm[1][2]),
                        depth_scale=0.0,
                    )
                    logger.info("scene3d intrinsic ← calibration robot=%s", self.robot_id)
                    return
        except Exception:
            logger.info("calibration intrinsic pull 실패 robot=%s — factory fallback", self.robot_id)

        # 2) camera factory
        try:
            fi = await self.runtime.call(
                Camera.Service.GET_FACTORY_INTRINSIC,
                GetFactoryIntrinsicRequest(),
                FactoryIntrinsic,
                robot_id=self.robot_id,
                timeout=3.0,
            )
            if fi.available:
                self._base_intrinsic = Scene3dIntrinsic(
                    width=fi.width,
                    height=fi.height,
                    fx=fi.fx,
                    fy=fi.fy,
                    cx=fi.cx,
                    cy=fi.cy,
                    depth_scale=0.0,
                )
                logger.info("scene3d intrinsic ← camera factory robot=%s", self.robot_id)
        except Exception:
            logger.warning("scene3d intrinsic 미확보 robot=%s — snapshot/live 불가", self.robot_id)

    # ── camera 캐시 (sync subscriber) ─────────────────────────
    @subscriber(Camera.Stream.DEPTH_DECODED)
    def on_depth(self, frame: CameraDepthDecodedFrame) -> None:
        if frame.robot_id != self.robot_id:
            return
        arr = np.frombuffer(frame.depth_bytes, dtype=np.uint16)
        if arr.size != frame.height * frame.width:
            return
        depth = arr.reshape(frame.height, frame.width)
        with self._lock:
            self._depths.append(depth)
            self._depth_scale = frame.depth_scale
            self._depth_wh = (frame.width, frame.height)

    @subscriber(Camera.Stream.DECODED)
    def on_color(self, frame: CameraDecodedFrame) -> None:
        if frame.robot_id != self.robot_id:
            return
        arr = np.frombuffer(frame.ndarray_bytes, dtype=np.uint8)
        if arr.size != frame.height * frame.width * 3:
            return
        with self._lock:
            self._latest_color = arr.reshape(frame.height, frame.width, 3)

    # ── services ──────────────────────────────────────────────
    @service(Scene3d.Service.SET_STREAM)
    def set_stream(self, req: SetStreamRequest) -> SetStreamResponse:
        self._enabled = req.enabled
        if req.voxel_size is not None and req.voxel_size > 0:
            self._voxel = req.voxel_size
        return SetStreamResponse(ok=True, enabled=self._enabled, voxel_size=self._voxel)

    @service(Scene3d.Service.SNAPSHOT)
    def snapshot(self, req: SnapshotRequest) -> SnapshotResponse:
        """최근 N depth 의 consensus median + latest color → jpeg/zstd + intrinsic."""
        with self._lock:
            depths = list(self._depths)[-max(1, req.num_frames):]
            color = None if self._latest_color is None else self._latest_color.copy()
            depth_scale = self._depth_scale
            depth_wh = self._depth_wh
        if not depths or color is None or depth_wh is None:
            raise RuntimeError("scene3d snapshot: depth/color frame 아직 없음")
        if self._base_intrinsic is None:
            raise RuntimeError("scene3d snapshot: intrinsic 미확보 (calibration/camera 확인)")

        depth = consensus_depth(depths)
        intr = pc.scale_intrinsic(
            self._base_intrinsic, depth_wh[0], depth_wh[1], depth_scale
        )

        ok, buf = cv2.imencode(
            ".jpg", color, [int(cv2.IMWRITE_JPEG_QUALITY), _JPEG_QUALITY]
        )
        if not ok:
            raise RuntimeError("scene3d snapshot: color JPEG 인코딩 실패")
        depth_zstd = self._zstd.compress(np.ascontiguousarray(depth).tobytes())

        return SnapshotResponse(
            color_jpeg=bytes(buf.tobytes()),
            depth_zstd=depth_zstd,
            intrinsic=intr,
            num_frames=len(depths),
            timestamp_unix=time.time(),
        )

    # ── live loop (8Hz) ───────────────────────────────────────
    async def _live_loop(self) -> None:
        interval = 1.0 / _LIVE_HZ
        try:
            while not self._stop:
                if self._enabled:
                    try:
                        await self._publish_cloud()
                    except Exception:
                        logger.exception("scene3d live publish 실패 %s", self.robot_id)
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            pass

    async def _publish_cloud(self) -> None:
        if self._base_intrinsic is None:
            await self._fetch_intrinsic()
            if self._base_intrinsic is None:
                return
        with self._lock:
            depth = None if not self._depths else self._depths[-1].copy()
            color = None if self._latest_color is None else self._latest_color.copy()
            depth_scale = self._depth_scale
            depth_wh = self._depth_wh
        if depth is None or color is None or depth_wh is None:
            return
        intr = pc.scale_intrinsic(
            self._base_intrinsic, depth_wh[0], depth_wh[1], depth_scale
        )
        voxel = self._voxel
        # open3d build 는 CPU — event loop non-block (async 계약)
        count, xyz_b, rgb_b = await asyncio.to_thread(
            self._build_and_encode, color, depth, intr, voxel
        )
        if count == 0:
            return
        self.runtime.publish(
            Scene3d.Stream.CLOUD,
            Scene3dCloud(
                robot_id=self.robot_id,
                seq=self._seq,
                timestamp_unix=time.time(),
                point_count=count,
                xyz_bytes=xyz_b,
                rgb_bytes=rgb_b,
            ),
        )
        self._seq += 1

    @staticmethod
    def _build_and_encode(
        color: np.ndarray, depth: np.ndarray, intr: Scene3dIntrinsic, voxel: float
    ) -> tuple[int, bytes, bytes]:
        pcd = pc.build_pcd(color, depth, intr)
        if voxel > 0:
            pcd = pcd.voxel_down_sample(voxel)
        return pc.encode_cloud(pcd)
