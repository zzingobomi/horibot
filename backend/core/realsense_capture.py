import logging
import threading
import time
from typing import Any

import numpy as np
import pyrealsense2 as rs

logger = logging.getLogger(__name__)


class RealsenseCapture:
    _instance: "RealsenseCapture | None" = None
    _new_lock = threading.Lock()

    def __new__(cls) -> "RealsenseCapture":
        if cls._instance is None:
            with cls._new_lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(
        self,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
    ):
        if self._initialized:
            return
        self._initialized = True
        self._width = width
        self._height = height
        self._fps = fps

        self._pipeline: Any = None
        self._opened = False
        self._open_lock = threading.Lock()

        self._running = False
        self._producer_thread: threading.Thread | None = None

        self._cloud_enabled = False
        self._depth_scale: float = 1.0
        self._depth_intrinsics: Any = None

        self._latest_color: np.ndarray | None = None
        self._latest_aligned_color: np.ndarray | None = None
        self._latest_depth: np.ndarray | None = None
        self._aligned_frame_seq: int = 0  # 새 aligned 쌍이 쓰일 때마다 +1
        self._frame_lock = threading.Lock()

    # ─── Lifecycle ───────────────────────────────────────────

    def open(self) -> bool:
        with self._open_lock:
            if self._opened:
                return True
            try:
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(
                    rs.stream.color,
                    self._width, self._height, rs.format.bgr8, self._fps,
                )
                config.enable_stream(
                    rs.stream.depth,
                    self._width, self._height, rs.format.z16, self._fps,
                )
                profile = pipeline.start(config)
            except RuntimeError as e:
                logger.error(f"RealSense 파이프라인 시작 실패: {e}")
                return False

            try:
                depth_sensor = profile.get_device().first_depth_sensor()
                self._depth_scale = float(depth_sensor.get_depth_scale())
            except RuntimeError as e:
                logger.warning(f"depth scale 조회 실패: {e}")

            self._pipeline = pipeline
            self._opened = True
            self._running = True
            self._producer_thread = threading.Thread(
                target=self._producer_loop,
                name="rs-producer",
                daemon=True,
            )
            self._producer_thread.start()
            logger.info(
                f"RealSense 연결: {self._width}x{self._height}@{self._fps}, "
                f"depth_scale={self._depth_scale:.5f}"
            )
            return True

    def close(self) -> None:
        with self._open_lock:
            if not self._opened:
                return
            self._running = False
            if self._producer_thread:
                self._producer_thread.join(timeout=2.0)
                self._producer_thread = None
            try:
                if self._pipeline is not None:
                    self._pipeline.stop()
            except RuntimeError as e:
                logger.warning(f"RealSense 종료 중 오류: {e}")
            self._pipeline = None
            self._opened = False
            with self._frame_lock:
                self._latest_color = None
                self._latest_aligned_color = None
                self._latest_depth = None
            logger.info("RealSense 연결 종료")

    # ─── Producer ────────────────────────────────────────────

    def _producer_loop(self) -> None:
        align = rs.align(rs.stream.color)
        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
            except RuntimeError as e:
                logger.debug(f"RS frame timeout/err: {e}")
                continue

            color = frames.get_color_frame()
            color_np = (
                np.asanyarray(color.get_data()).copy() if color else None
            )

            depth_np: np.ndarray | None = None
            aligned_color_np: np.ndarray | None = None
            depth_intr = None
            if self._cloud_enabled:
                raw_depth = frames.get_depth_frame()
                if color and raw_depth:
                    try:
                        aligned = align.process(frames)
                        aligned_color = aligned.get_color_frame()
                        depth = aligned.get_depth_frame()
                    except RuntimeError as e:
                        logger.debug(f"RS align err: {e}")
                        aligned_color = None
                        depth = None
                    if aligned_color and depth:
                        aligned_color_np = np.asanyarray(
                            aligned_color.get_data()
                        ).copy()
                        depth_np = np.asanyarray(depth.get_data()).copy()
                        if self._depth_intrinsics is None:
                            depth_intr = (
                                depth.profile.as_video_stream_profile()
                                .get_intrinsics()
                            )

            with self._frame_lock:
                if color_np is not None:
                    self._latest_color = color_np
                if depth_np is not None and aligned_color_np is not None:
                    self._latest_aligned_color = aligned_color_np
                    self._latest_depth = depth_np
                    self._aligned_frame_seq += 1
                    if depth_intr is not None and self._depth_intrinsics is None:
                        self._depth_intrinsics = depth_intr

    # ─── Consumer API ────────────────────────────────────────

    def set_cloud_enabled(self, enabled: bool) -> None:
        self._cloud_enabled = enabled
        if not enabled:
            with self._frame_lock:
                self._latest_aligned_color = None
                self._latest_depth = None

    def read_color(self) -> tuple[bool, np.ndarray | None]:
        with self._frame_lock:
            if self._latest_color is None:
                return False, None
            return True, self._latest_color

    def read_aligned_color_depth(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        with self._frame_lock:
            if self._latest_aligned_color is None or self._latest_depth is None:
                return None, None
            return self._latest_aligned_color, self._latest_depth

    def grab_aligned_blocking(
        self, timeout: float = 1.5
    ) -> tuple[np.ndarray | None, np.ndarray | None, Any]:
        if not self._opened:
            return None, None, None

        prev_enabled = self._cloud_enabled
        if not prev_enabled:
            with self._frame_lock:
                self._latest_aligned_color = None
                self._latest_depth = None
            self._cloud_enabled = True

        deadline = time.time() + timeout
        try:
            while time.time() < deadline:
                with self._frame_lock:
                    color = self._latest_aligned_color
                    depth = self._latest_depth
                    intr = self._depth_intrinsics
                if color is not None and depth is not None and intr is not None:
                    return color, depth, intr
                time.sleep(0.03)
            return None, None, None
        finally:
            if not prev_enabled:
                self._cloud_enabled = False
                with self._frame_lock:
                    self._latest_aligned_color = None
                    self._latest_depth = None

    def grab_n_aligned_blocking(
        self,
        n: int,
        timeout: float = 2.0,
    ) -> list[tuple[np.ndarray, np.ndarray, Any]]:
        """정지한 자세에서 N개의 서로 다른 aligned (color, depth, intr) 프레임을 수집.

        producer가 새 aligned 쌍을 쓸 때마다 _aligned_frame_seq가 증가하는 걸 이용.
        seq가 바뀔 때마다 한 장씩 채취 → N장 모일 때까지 반복.
        시간 안에 못 모으면 빈 리스트 반환.
        """
        if not self._opened or n <= 0:
            return []

        prev_enabled = self._cloud_enabled
        if not prev_enabled:
            with self._frame_lock:
                self._latest_aligned_color = None
                self._latest_depth = None
            self._cloud_enabled = True

        deadline = time.time() + timeout
        collected: list[tuple[np.ndarray, np.ndarray, Any]] = []
        last_seq = -1
        try:
            while len(collected) < n and time.time() < deadline:
                with self._frame_lock:
                    seq = self._aligned_frame_seq
                    color = self._latest_aligned_color
                    depth = self._latest_depth
                    intr = self._depth_intrinsics

                if (
                    seq != last_seq
                    and color is not None
                    and depth is not None
                    and intr is not None
                ):
                    collected.append((color.copy(), depth.copy(), intr))
                    last_seq = seq
                else:
                    time.sleep(0.005)

            if len(collected) < n:
                logger.warning(
                    f"grab_n_aligned_blocking 타임아웃: {len(collected)}/{n}"
                )
                return []
            return collected
        finally:
            if not prev_enabled:
                self._cloud_enabled = False
                with self._frame_lock:
                    self._latest_aligned_color = None
                    self._latest_depth = None

    # ─── Properties ──────────────────────────────────────────

    @property
    def is_opened(self) -> bool:
        return self._opened

    @property
    def width(self) -> int:
        return self._width if self._opened else 0

    @property
    def height(self) -> int:
        return self._height if self._opened else 0

    @property
    def fps(self) -> float:
        return float(self._fps) if self._opened else 0.0

    @property
    def depth_scale(self) -> float:
        return self._depth_scale

    @property
    def depth_intrinsics(self) -> Any:
        return self._depth_intrinsics
