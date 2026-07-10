"""CameraDriverModule — robot-scoped Hardware Layer Module.

backend.md §16.1 #2 (CameraDriver) + §11 Build order Step B.

책임:
- driver.capture_color() → cv2.imencode JPEG → Camera.Stream.JPEG publish
- driver.capture_depth() → zstd compress → Camera.Stream.DEPTH_RAW publish
- driver self-declare capability relay

driver 종류 모름 — RealSense / USB UVC / Basler / mock 다 Protocol 위 동작.
"""

from __future__ import annotations

import asyncio
import logging
import time

import cv2
import numpy as np
import zstandard as zstd

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.runtime.api import ModuleRuntime

from .contract import (
    Camera,
    CameraCapabilities,
    CameraDepthRawFrame,
    CameraJpegFrame,
    CapabilitiesRequest,
    FactoryIntrinsic,
    GetFactoryIntrinsicRequest,
)
from .drivers.protocol import CameraDriver

logger = logging.getLogger(__name__)

# 30Hz capture (RealSense D405 자리). backend/ 의 camera_node 와 동일
_CAPTURE_HZ = 30.0
_JPEG_QUALITY = 85
_ZSTD_LEVEL = 3


@publishes(
    (Camera.Stream.JPEG, CameraJpegFrame),
    (Camera.Stream.DEPTH_RAW, CameraDepthRawFrame),
)
class CameraDriverModule:
    """robot-scoped Module — robot 의 camera hardware adapter relay."""

    def __init__(
        self,
        runtime: ModuleRuntime,
        robot_id: str,
        driver: CameraDriver,
    ) -> None:
        self.runtime = runtime
        self.robot_id = robot_id
        self._driver = driver

        # boot 1회 cache — capability static (§7.3)
        self._capabilities: CameraCapabilities | None = None

        # stream seq counter (§8.5 — per-stream)
        self._jpeg_seq = 0
        self._depth_seq = 0

        self._capture_task: asyncio.Task[None] | None = None
        self._stop_requested = False

        # zstd compressor 인스턴스 — boot 1회 (compress 재사용)
        self._zstd_compressor = zstd.ZstdCompressor(level=_ZSTD_LEVEL)

    # ── lifecycle ─────────────────────────────────────────────

    async def start(self) -> None:
        self._driver.open()
        self._capabilities = self._driver.capabilities()

        self._stop_requested = False
        self._capture_task = asyncio.create_task(self._capture_loop())

    async def stop(self) -> None:
        self._stop_requested = True
        task = self._capture_task
        if task is not None:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            self._capture_task = None
        self._driver.close()

    # ── service handlers ──────────────────────────────────────

    @service(Camera.Service.CAPABILITIES)
    def get_capabilities(self, req: CapabilitiesRequest) -> CameraCapabilities:
        assert self._capabilities is not None, "start() 박힌 후 호출"
        return self._capabilities

    @service(Camera.Service.GET_FACTORY_INTRINSIC)
    def get_factory_intrinsic(
        self, req: GetFactoryIntrinsicRequest
    ) -> FactoryIntrinsic:
        # internal — Calibration seed only (§7.6). driver 가 없으면 available=False.
        fi = self._driver.get_factory_intrinsics()
        if fi is None:
            return FactoryIntrinsic(available=False)
        return FactoryIntrinsic(
            available=True,
            fx=fi.fx,
            fy=fi.fy,
            cx=fi.cx,
            cy=fi.cy,
            width=fi.width,
            height=fi.height,
        )

    # ── capture loop (30Hz) ───────────────────────────────────

    async def _capture_loop(self) -> None:
        interval = 1.0 / _CAPTURE_HZ
        has_depth = (
            self._capabilities is not None
            and "depth" in {c.value for c in self._capabilities.flags}
        )
        try:
            while not self._stop_requested:
                try:
                    self._capture_color_and_publish()
                    if has_depth:
                        self._capture_depth_and_publish()
                except Exception:
                    logger.exception(
                        "CameraDriver capture 실패 robot_id=%s",
                        self.robot_id,
                    )
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            pass

    def _capture_color_and_publish(self) -> None:
        raw = self._driver.capture_color()
        arr = np.frombuffer(raw.ndarray_bytes, dtype=np.uint8).reshape(
            raw.height, raw.width, 3,
        )
        success, encoded = cv2.imencode(
            ".jpg", arr, [int(cv2.IMWRITE_JPEG_QUALITY), _JPEG_QUALITY],
        )
        if not success:
            logger.warning("JPEG encode failed robot_id=%s", self.robot_id)
            return
        event = CameraJpegFrame(
            robot_id=self.robot_id,
            seq=self._jpeg_seq,
            timestamp_unix=time.time(),
            jpeg_bytes=bytes(encoded.tobytes()),
            width=raw.width,
            height=raw.height,
        )
        self._jpeg_seq += 1
        self.runtime.publish(Camera.Stream.JPEG, event)

    def _capture_depth_and_publish(self) -> None:
        raw = self._driver.capture_depth()
        if raw is None:
            return
        compressed = self._zstd_compressor.compress(raw.depth_bytes)
        event = CameraDepthRawFrame(
            robot_id=self.robot_id,
            seq=self._depth_seq,
            timestamp_unix=time.time(),
            depth_zstd=compressed,
            width=raw.width,
            height=raw.height,
            depth_scale=raw.depth_scale,
        )
        self._depth_seq += 1
        self.runtime.publish(Camera.Stream.DEPTH_RAW, event)
