"""Scene3DModule — robot-agnostic RGBD primitive (라이브 PC + consensus snapshot).

옛 backend/nodes/application/scene3d_node.py 의 primitive 부분 이월. v2 적응:
- camera depth on/off refcount 제거 (v2 camera 상시 depth stream).
- 라이브 PC 는 camera-frame 발행 → frontend 가 tcp·hand_eye transform (옛 패턴 유지).
- intrinsic 은 active calibration(우선) → camera factory(fallback) 에서 boot 시 pull.

**robot-agnostic** — host 당 1 인스턴스 (backend_v2.md §2.7).
대상 robot 은 req.robot_id. per-robot config 는
멤버십(rgbd robot 목록)뿐 — resolve 가 robot_ids 주입, runtime state(depth ring /
color / intrinsic / stream on-off)는 robot_id 키 dict.

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


class _RobotBuffers:
    """robot 1개의 RGBD runtime state — lock 은 module 전역 (짧은 임계영역)."""

    def __init__(self) -> None:
        self.depths: deque[np.ndarray] = deque(maxlen=_DEPTH_BUFFER)
        self.depth_scale = 0.0
        self.depth_wh: tuple[int, int] | None = None
        self.latest_color: np.ndarray | None = None
        # base intrinsic (fx/fy/cx/cy @ image_size). depth 해상도로 scale 해서 사용.
        self.base_intrinsic: Scene3dIntrinsic | None = None
        self.enabled = False
        self.voxel = _DEFAULT_VOXEL
        self.seq = 0


@publishes((Scene3d.Stream.CLOUD, Scene3dCloud))
class Scene3DModule:
    def __init__(self, runtime: ModuleRuntime, robot_ids: list[str]) -> None:
        self.runtime = runtime
        self._lock = threading.Lock()
        # rgbd robot 별 buffers — 멤버십 = resolve 투영 (robots.yaml rgbd capability)
        self._buf: dict[str, _RobotBuffers] = {rid: _RobotBuffers() for rid in robot_ids}

        self._stop = False
        self._live_task: asyncio.Task[None] | None = None
        self._zstd = zstd.ZstdCompressor(level=_ZSTD_LEVEL)

    # ── lifecycle ─────────────────────────────────────────────
    async def start(self) -> None:
        logger.info("Scene3DModule start (host-level, robots=%s)", sorted(self._buf))
        # 전 robot intrinsic pull — gather (미배치 timeout 직렬 누적 방지)
        if self._buf:
            await asyncio.gather(*(self._fetch_intrinsic(rid) for rid in self._buf))
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
        logger.info("Scene3DModule stop (host-level)")

    async def _fetch_intrinsic(self, robot_id: str) -> None:
        """active calibration intrinsic 우선 → camera factory fallback. cache."""
        buf = self._buf[robot_id]
        # 1) active calibration intrinsic (calibration 은 robot-agnostic — req 필드)
        try:
            bundle = await self.runtime.call(
                Calibration.Service.SNAPSHOT_BUNDLE,
                SnapshotBundleRequest(robot_id=robot_id),
                CalibrationBundle,
                timeout=3.0,
            )
            if bundle.intrinsic is not None:
                cm = bundle.intrinsic.result_data.camera_matrix
                size = bundle.intrinsic.result_data.image_size
                if size and len(size) == 2:
                    buf.base_intrinsic = Scene3dIntrinsic(
                        width=int(size[0]),
                        height=int(size[1]),
                        fx=float(cm[0][0]),
                        fy=float(cm[1][1]),
                        cx=float(cm[0][2]),
                        cy=float(cm[1][2]),
                        depth_scale=0.0,
                    )
                    logger.info("scene3d intrinsic ← calibration robot=%s", robot_id)
                    return
        except Exception:
            logger.info("calibration intrinsic pull 실패 robot=%s — factory fallback", robot_id)

        # 2) camera factory
        try:
            fi = await self.runtime.call(
                Camera.Service.GET_FACTORY_INTRINSIC,
                GetFactoryIntrinsicRequest(),
                FactoryIntrinsic,
                robot_id=robot_id,
                timeout=3.0,
            )
            if fi.available:
                buf.base_intrinsic = Scene3dIntrinsic(
                    width=fi.width,
                    height=fi.height,
                    fx=fi.fx,
                    fy=fi.fy,
                    cx=fi.cx,
                    cy=fi.cy,
                    depth_scale=0.0,
                )
                logger.info("scene3d intrinsic ← camera factory robot=%s", robot_id)
        except Exception:
            logger.warning("scene3d intrinsic 미확보 robot=%s — snapshot/live 불가", robot_id)

    # ── camera 캐시 (sync subscriber, 전 robot wildcard) ──────
    @subscriber(Camera.Stream.DEPTH_DECODED)
    def on_depth(self, frame: CameraDepthDecodedFrame) -> None:
        buf = self._buf.get(frame.robot_id)
        if buf is None:
            return
        arr = np.frombuffer(frame.depth_bytes, dtype=np.uint16)
        if arr.size != frame.height * frame.width:
            return
        depth = arr.reshape(frame.height, frame.width)
        with self._lock:
            buf.depths.append(depth)
            buf.depth_scale = frame.depth_scale
            buf.depth_wh = (frame.width, frame.height)

    @subscriber(Camera.Stream.DECODED)
    def on_color(self, frame: CameraDecodedFrame) -> None:
        buf = self._buf.get(frame.robot_id)
        if buf is None:
            return
        arr = np.frombuffer(frame.ndarray_bytes, dtype=np.uint8)
        if arr.size != frame.height * frame.width * 3:
            return
        with self._lock:
            buf.latest_color = arr.reshape(frame.height, frame.width, 3)

    # ── services ──────────────────────────────────────────────
    @service(Scene3d.Service.SET_STREAM)
    def set_stream(self, req: SetStreamRequest) -> SetStreamResponse:
        buf = self._buf.get(req.robot_id)
        if buf is None:
            raise KeyError(f"robot {req.robot_id!r} 이 이 host 의 rgbd fleet 에 없음")
        buf.enabled = req.enabled
        if req.voxel_size is not None and req.voxel_size > 0:
            buf.voxel = req.voxel_size
        return SetStreamResponse(ok=True, enabled=buf.enabled, voxel_size=buf.voxel)

    @service(Scene3d.Service.SNAPSHOT)
    def snapshot(self, req: SnapshotRequest) -> SnapshotResponse:
        """최근 N depth 의 consensus median + latest color → jpeg/zstd + intrinsic."""
        buf = self._buf.get(req.robot_id)
        if buf is None:
            raise KeyError(f"robot {req.robot_id!r} 이 이 host 의 rgbd fleet 에 없음")
        with self._lock:
            depths = list(buf.depths)[-max(1, req.num_frames):]
            color = None if buf.latest_color is None else buf.latest_color.copy()
            depth_scale = buf.depth_scale
            depth_wh = buf.depth_wh
        if not depths or color is None or depth_wh is None:
            raise RuntimeError("scene3d snapshot: depth/color frame 아직 없음")
        if buf.base_intrinsic is None:
            raise RuntimeError("scene3d snapshot: intrinsic 미확보 (calibration/camera 확인)")

        depth = consensus_depth(depths)
        intr = pc.scale_intrinsic(
            buf.base_intrinsic, depth_wh[0], depth_wh[1], depth_scale
        )

        ok, jpg = cv2.imencode(
            ".jpg", color, [int(cv2.IMWRITE_JPEG_QUALITY), _JPEG_QUALITY]
        )
        if not ok:
            raise RuntimeError("scene3d snapshot: color JPEG 인코딩 실패")
        depth_zstd = self._zstd.compress(np.ascontiguousarray(depth).tobytes())

        return SnapshotResponse(
            color_jpeg=bytes(jpg.tobytes()),
            depth_zstd=depth_zstd,
            intrinsic=intr,
            num_frames=len(depths),
            timestamp_unix=time.time(),
        )

    # ── live loop (8Hz, robot 별) ─────────────────────────────
    async def _live_loop(self) -> None:
        interval = 1.0 / _LIVE_HZ
        try:
            while not self._stop:
                for rid, buf in self._buf.items():
                    if not buf.enabled:
                        continue
                    try:
                        await self._publish_cloud(rid, buf)
                    except Exception:
                        logger.exception("scene3d live publish 실패 %s", rid)
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            pass

    async def _publish_cloud(self, robot_id: str, buf: _RobotBuffers) -> None:
        if buf.base_intrinsic is None:
            await self._fetch_intrinsic(robot_id)
            if buf.base_intrinsic is None:
                return
        with self._lock:
            depth = None if not buf.depths else buf.depths[-1].copy()
            color = None if buf.latest_color is None else buf.latest_color.copy()
            depth_scale = buf.depth_scale
            depth_wh = buf.depth_wh
        if depth is None or color is None or depth_wh is None:
            return
        intr = pc.scale_intrinsic(
            buf.base_intrinsic, depth_wh[0], depth_wh[1], depth_scale
        )
        voxel = buf.voxel
        # open3d build 는 CPU — event loop non-block (async 계약)
        count, xyz_b, rgb_b = await asyncio.to_thread(
            self._build_and_encode, color, depth, intr, voxel
        )
        if count == 0:
            return
        self.runtime.publish(
            Scene3d.Stream.CLOUD,
            Scene3dCloud(
                robot_id=robot_id,
                seq=buf.seq,
                timestamp_unix=time.time(),
                point_count=count,
                xyz_bytes=xyz_b,
                rgb_bytes=rgb_b,
            ),
        )
        buf.seq += 1

    @staticmethod
    def _build_and_encode(
        color: np.ndarray, depth: np.ndarray, intr: Scene3dIntrinsic, voxel: float
    ) -> tuple[int, bytes, bytes]:
        pcd = pc.build_pcd(color, depth, intr)
        if voxel > 0:
            pcd = pcd.voxel_down_sample(voxel)
        return pc.encode_cloud(pcd)
