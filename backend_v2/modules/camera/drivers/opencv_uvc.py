"""OpenCVUvcDriver — `cv2.VideoCapture` 기반 실 CameraDriver (USB UVC).

OMX 의 720P USB UVC (DFOV 120°) 자리 (docs/distributed_topology.md §1 — D405 는
so101 양도). color-only:
- capabilities = {RGB} — depth 미지원. Scene3D/Scan 은 rgbd capability gate 로
  자동 제외 (resolve.py), camera 모듈의 depth publish 도 skip (mock has_depth=False
  와 동일 경로).
- factory intrinsic 없음 — get_factory_intrinsics() → None. contract 의
  `FactoryIntrinsic.available=False` 가 정확히 이 자리 (사용자 intrinsic 캘 필요).

realsense_d405.py 와 동형 producer thread + latest frame cache — VideoCapture 의
내부 버퍼가 쌓이면 오래된 frame 이 나오므로 (지연 누적), 전용 thread 가 계속
read 해서 최신만 유지.

검증: 작성/import/type 은 회사, 실 UVC 캡처는 집 (hori3 + OMX 마운트).
"""

from __future__ import annotations

import logging
import threading
import time

import cv2
import numpy as np

from ..contract import CameraCapabilities, CameraCapability
from .protocol import FactoryIntrinsics, RawColorFrame, RawDepthFrame

logger = logging.getLogger(__name__)


class OpenCVUvcDriver:
    """USB UVC wrap — v2 CameraDriver Protocol 만족. color-only."""

    def __init__(
        self,
        device_index: int = 0,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
    ) -> None:
        self._device_index = device_index
        self._width = width
        self._height = height
        self._fps = fps

        self._cap: cv2.VideoCapture | None = None
        self._opened = False
        self._open_lock = threading.Lock()
        self._running = False
        self._producer: threading.Thread | None = None

        self._latest_color: np.ndarray | None = None  # BGR
        self._frame_lock = threading.Lock()

    # ── self-declare (§7.3) ──

    def capabilities(self) -> CameraCapabilities:
        return CameraCapabilities(flags={CameraCapability.RGB})

    # ── lifecycle ──

    def open(self) -> None:
        with self._open_lock:
            if self._opened:
                return
            cap = cv2.VideoCapture(self._device_index)
            if not cap.isOpened():
                raise RuntimeError(
                    f"UVC 카메라 open 실패: device_index={self._device_index}"
                )
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            cap.set(cv2.CAP_PROP_FPS, self._fps)
            # 내부 버퍼 최소화 — producer 가 최신 유지하지만 드라이버 단 지연도 줄임.
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            # 실 적용값 (카메라가 요청 해상도 미지원 시 다른 값 반환)
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if (actual_w, actual_h) != (self._width, self._height):
                logger.warning(
                    "UVC 해상도 %dx%d 미지원 — %dx%d 로 동작",
                    self._width, self._height, actual_w, actual_h,
                )
                self._width, self._height = actual_w, actual_h

            self._cap = cap
            self._opened = True
            self._running = True
            self._producer = threading.Thread(
                target=self._producer_loop, name="uvc-producer", daemon=True
            )
            self._producer.start()

            # 첫 frame 까지 대기 — capture_color 가 곧장 호출돼도 race 없게
            deadline = time.time() + 5.0
            while time.time() < deadline:
                with self._frame_lock:
                    if self._latest_color is not None:
                        break
                time.sleep(0.02)
            logger.info(
                "UVC 카메라: device=%d %dx%d@%d",
                self._device_index, self._width, self._height, self._fps,
            )

    def close(self) -> None:
        with self._open_lock:
            if not self._opened:
                return
            self._running = False
            if self._producer is not None:
                self._producer.join(timeout=2.0)
                self._producer = None
            if self._cap is not None:
                self._cap.release()
                self._cap = None
            self._opened = False
            with self._frame_lock:
                self._latest_color = None
            logger.info("UVC 카메라 종료")

    # ── producer ──

    def _producer_loop(self) -> None:
        while self._running:
            cap = self._cap
            if cap is None:
                return
            ok, frame = cap.read()
            if not ok:
                # 일시 실패 (USB glitch) — 직전 frame 유지, 잠깐 쉬고 재시도
                time.sleep(0.05)
                continue
            with self._frame_lock:
                self._latest_color = frame

    # ── capture (v2 Protocol) ──

    def capture_color(self) -> RawColorFrame:
        with self._frame_lock:
            color = self._latest_color
        if color is None:
            raise RuntimeError("UVC color frame 아직 없음")
        return RawColorFrame(
            ndarray_bytes=color.tobytes(),
            width=int(color.shape[1]),
            height=int(color.shape[0]),
        )

    def capture_depth(self) -> RawDepthFrame | None:
        return None  # UVC = color-only

    def get_factory_intrinsics(self) -> FactoryIntrinsics | None:
        # UVC 는 factory intrinsic 제공 안 함 — 사용자 intrinsic 캘 필요
        # (contract FactoryIntrinsic.available=False 경로).
        return None
