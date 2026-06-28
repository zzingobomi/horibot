"""Mock CameraDriver — 합성 color + depth frame. hardware-less 자리."""

from __future__ import annotations

import numpy as np

from ..contract import CameraCapabilities, CameraCapability
from .protocol import FactoryIntrinsics, RawColorFrame, RawDepthFrame


class MockCameraDriver:
    """In-process mock — 합성 BGR + uint16 depth. 매 capture 마다 frame counter
    인 gradient pattern 박음 (test 자리 deterministic 검증)."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        has_depth: bool = True,
        depth_scale: float = 0.0001,
    ) -> None:
        self._width = width
        self._height = height
        self._has_depth = has_depth
        self._depth_scale = depth_scale
        self._counter = 0

    # ── self-declare ──

    def capabilities(self) -> CameraCapabilities:
        flags = {CameraCapability.RGB}
        if self._has_depth:
            flags.add(CameraCapability.DEPTH)
            flags.add(CameraCapability.POINTCLOUD)
        return CameraCapabilities(flags=flags)

    # ── lifecycle ──

    def open(self) -> None:
        self._counter = 0

    def close(self) -> None:
        pass

    # ── capture ──

    def capture_color(self) -> RawColorFrame:
        # gradient pattern + frame counter — deterministic
        c = self._counter % 256
        arr = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        arr[:, :, 0] = c  # B channel = counter
        # G channel = x gradient (uint8 wrap)
        arr[:, :, 1] = (np.arange(self._width) % 256).astype(np.uint8)
        # R channel = y gradient
        arr[:, :, 2] = (
            (np.arange(self._height) % 256).astype(np.uint8)[:, None]
        )
        self._counter += 1
        return RawColorFrame(
            ndarray_bytes=arr.tobytes(),
            width=self._width,
            height=self._height,
        )

    def capture_depth(self) -> RawDepthFrame | None:
        if not self._has_depth:
            return None
        # 합성 uint16 depth — 매 row 1000 + y*10 (mm scale 대신 임의 raw)
        arr = (
            (np.arange(self._height, dtype=np.uint16) * 10 + 1000)[:, None]
            .repeat(self._width, axis=1)
        )
        return RawDepthFrame(
            depth_bytes=arr.tobytes(),
            width=self._width,
            height=self._height,
            depth_scale=self._depth_scale,
        )

    # ── factory intrinsic (Module internal — Calibration seed only, §7.6) ──

    def get_factory_intrinsics(self) -> FactoryIntrinsics | None:
        # 합성 — D405 자리 가까운 값
        return FactoryIntrinsics(
            fx=float(self._width),
            fy=float(self._width),
            cx=self._width / 2.0,
            cy=self._height / 2.0,
            width=self._width,
            height=self._height,
        )
