"""RealSenseD405Driver — `pyrealsense2` 기반 실 CameraDriver (D405).

옛 backend/modules/camera/adapters/{realsense_driver,realsense_capture}.py 의
faithful port. pipeline / align(depth→color) / producer thread + latest frame
cache 그대로, v2 CameraDriver Protocol 형태(capture_color / capture_depth /
get_factory_intrinsics / capabilities)로 재구성.

depth 는 color 프레임에 align — color 는 그대로(raw color = aligned color, depth 만
warp)라 v2 의 분리된 color(JPEG)+depth(zstd) stream 이 픽셀 대응 유지.

검증: 작성/import/type 은 회사, 실 D405 캡처는 집.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

import numpy as np
import pyrealsense2  # type: ignore[import-untyped]

from ..contract import CameraCapabilities, CameraCapability
from .protocol import FactoryIntrinsics, RawColorFrame, RawDepthFrame

# pyrealsense2 stub 이 일부 attribute 만 노출 — Any rebind 으로 동적 접근 허용.
rs: Any = pyrealsense2

logger = logging.getLogger(__name__)


class RealSenseD405Driver:
    """RealSense D405 wrap — v2 CameraDriver Protocol 만족. process 당 1 pipeline."""

    def __init__(self, width: int = 1280, height: int = 720, fps: int = 30) -> None:
        self._width = width
        self._height = height
        self._fps = fps

        self._pipeline: Any = None
        self._opened = False
        self._open_lock = threading.Lock()
        self._running = False
        self._producer: threading.Thread | None = None

        self._depth_scale = 1.0
        self._factory_intrinsics: FactoryIntrinsics | None = None

        self._latest_color: np.ndarray | None = None  # BGR
        self._latest_depth: np.ndarray | None = None  # uint16, color 에 align
        self._frame_lock = threading.Lock()

    # ── self-declare (§7.3) ──

    def capabilities(self) -> CameraCapabilities:
        # max_resolution / supported_fps 는 §7.3 미래 metadata (현 contract = flags only)
        return CameraCapabilities(
            flags={
                CameraCapability.RGB,
                CameraCapability.DEPTH,
                CameraCapability.POINTCLOUD,
            },
        )

    # ── lifecycle ──

    def open(self) -> None:
        with self._open_lock:
            if self._opened:
                return
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(
                rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps
            )
            config.enable_stream(
                rs.stream.depth, self._width, self._height, rs.format.z16, self._fps
            )
            try:
                profile = pipeline.start(config)
            except RuntimeError as e:
                raise RuntimeError(f"RealSense pipeline 시작 실패: {e}") from e

            try:
                self._depth_scale = float(
                    profile.get_device().first_depth_sensor().get_depth_scale()
                )
            except RuntimeError as e:
                logger.warning("depth scale 조회 실패: %s", e)

            self._factory_intrinsics = self._read_color_intrinsics(profile)

            self._pipeline = pipeline
            self._opened = True
            self._running = True
            self._producer = threading.Thread(
                target=self._producer_loop, name="rs-d405-producer", daemon=True
            )
            self._producer.start()

            # 첫 color frame 까지 대기 — capture_color 가 곧장 호출돼도 race 없게
            deadline = time.time() + 5.0
            while time.time() < deadline:
                with self._frame_lock:
                    if self._latest_color is not None:
                        break
                time.sleep(0.02)
            logger.info(
                "RealSense D405: %dx%d@%d depth_scale=%.5f",
                self._width, self._height, self._fps, self._depth_scale,
            )

    def close(self) -> None:
        with self._open_lock:
            if not self._opened:
                return
            self._running = False
            if self._producer is not None:
                self._producer.join(timeout=2.0)
                self._producer = None
            # pipeline.stop() 은 USB 상태 따라 blocking 영구 stuck 가능 —
            # daemon thread + timeout 으로 process exit 막지 않음 (옛 driver 패턴).
            pipeline = self._pipeline
            if pipeline is not None:
                def _stop_safe() -> None:
                    try:
                        pipeline.stop()
                    except RuntimeError as e:
                        logger.warning("RealSense stop 오류: %s", e)

                stopper = threading.Thread(target=_stop_safe, daemon=True)
                stopper.start()
                stopper.join(timeout=3.0)
            self._pipeline = None
            self._opened = False
            with self._frame_lock:
                self._latest_color = None
                self._latest_depth = None
            logger.info("RealSense D405 종료")

    # ── producer ──

    def _producer_loop(self) -> None:
        align = rs.align(rs.stream.color)
        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
            except RuntimeError:
                continue
            try:
                aligned = align.process(frames)
                color = aligned.get_color_frame()
                depth = aligned.get_depth_frame()
            except RuntimeError:
                continue
            color_np = np.asanyarray(color.get_data()).copy() if color else None
            depth_np = np.asanyarray(depth.get_data()).copy() if depth else None
            with self._frame_lock:
                if color_np is not None:
                    self._latest_color = color_np
                if depth_np is not None:
                    self._latest_depth = depth_np

    # ── capture (v2 Protocol) ──

    def capture_color(self) -> RawColorFrame:
        with self._frame_lock:
            color = self._latest_color
        if color is None:
            raise RuntimeError("RealSense color frame 아직 없음")
        return RawColorFrame(
            ndarray_bytes=color.tobytes(),
            width=int(color.shape[1]),
            height=int(color.shape[0]),
        )

    def capture_depth(self) -> RawDepthFrame | None:
        with self._frame_lock:
            depth = self._latest_depth
        if depth is None:
            return None
        return RawDepthFrame(
            depth_bytes=depth.tobytes(),
            width=int(depth.shape[1]),
            height=int(depth.shape[0]),
            depth_scale=self._depth_scale,
        )

    def get_factory_intrinsics(self) -> FactoryIntrinsics | None:
        return self._factory_intrinsics

    # ── util ──

    @staticmethod
    def _read_color_intrinsics(profile: Any) -> FactoryIntrinsics | None:
        try:
            intr = (
                profile.get_stream(rs.stream.color)
                .as_video_stream_profile()
                .get_intrinsics()
            )
        except RuntimeError as e:
            logger.warning("color intrinsics 조회 실패: %s", e)
            return None
        return FactoryIntrinsics(
            fx=float(intr.fx),
            fy=float(intr.fy),
            cx=float(intr.ppx),
            cy=float(intr.ppy),
            width=int(intr.width),
            height=int(intr.height),
        )
