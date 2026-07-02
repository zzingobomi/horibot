"""Mock CameraDriver — 합성 color + depth frame. hardware-less 자리.

CALIB_SIM_BOARD=1 env 면 color 를 gradient 대신 ChArUco 보드(cycling pose)로 렌더 —
브라우저/실 카메라 없이 calibration capture/preview e2e (옛 backend CALIB_SIM_BOARD 패턴).
factory intrinsic 과 동일 camera_matrix 로 렌더 → PnP reproj 낮음.
"""

from __future__ import annotations

import os

import numpy as np

from ..contract import CameraCapabilities, CameraCapability
from .protocol import FactoryIntrinsics, RawColorFrame, RawDepthFrame

# sim board 자세 (rvec rad, tvec m) — tilt ~20-32°, 시야 안. capture diversity 위해 cycle.
_SIM_POSES: list[tuple[list[float], list[float]]] = [
    ([0.35, 0.15, 0.0], [0.0, 0.0, 0.32]),
    ([0.45, -0.2, 0.1], [0.03, 0.0, 0.38]),
    ([-0.3, 0.35, 0.05], [-0.03, 0.02, 0.30]),
    ([0.2, 0.4, -0.1], [0.0, -0.02, 0.34]),
]


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
        # sim board 모드 (calibration e2e) — env-gated. 켜지면 gradient 대신 ChArUco.
        self._sim_board = os.environ.get("CALIB_SIM_BOARD") == "1"
        self._sim_frames: list[np.ndarray] | None = None

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

    def _sim_camera_matrix(self) -> np.ndarray:
        # factory intrinsic 과 동일 (fx=fy=width, cx/cy 중심) → capture PnP reproj 낮음.
        return np.array(
            [
                [self._width, 0.0, self._width / 2.0],
                [0.0, self._width, self._height / 2.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def _ensure_sim_frames(self) -> list[np.ndarray]:
        if self._sim_frames is not None:
            return self._sim_frames
        import cv2  # lazy — sim board 모드에서만 (mock 은 PC/dev)

        from modules.calibration.vision.se3 import make_T
        from modules.calibration.vision.sim_board import render_charuco_at_pose

        cm = self._sim_camera_matrix()
        dist = np.zeros((1, 5))
        frames: list[np.ndarray] = []
        for rvec, tvec in _SIM_POSES:
            bic = make_T(cv2.Rodrigues(np.array(rvec))[0], np.array(tvec))
            frames.append(
                render_charuco_at_pose(
                    width=self._width,
                    height=self._height,
                    camera_matrix=cm,
                    dist_coeffs=dist,
                    board_in_cam=bic,
                )
            )
        self._sim_frames = frames
        return frames

    def capture_color(self) -> RawColorFrame:
        if self._sim_board:
            frames = self._ensure_sim_frames()
            # ~0.5s 마다 pose 전환 (30Hz / 15) — detect 안정 + capture diversity
            arr = frames[(self._counter // 15) % len(frames)]
            self._counter += 1
            return RawColorFrame(
                ndarray_bytes=arr.tobytes(), width=self._width, height=self._height
            )
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
