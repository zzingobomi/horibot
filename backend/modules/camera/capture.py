"""CameraCapture Protocol + RealSense adapter.

multi_robot_architecture.md §3.4 참조.

Protocol 정의 + 현재 facade class `CameraCapture` (RealSense wrap) 가 그것을 만족.
미래 다른 카메라 (MuJoCo / USB / etc.) 추가 시 같은 Protocol 만 만족하면 됨.

데이터 클래스 (`CameraIntrinsic` / `ColorFrame` / `DepthFrame`) 는 frozen dataclass
— process-internal 사용. 토픽 페이로드로 publish 시는 별도 Pydantic model
(`backend/core/messages/camera.py` 의 `DepthFrameHeader` 등).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np

from core.realsense_capture import RealsenseCapture


# ─── Protocol 타입 ────────────────────────────────────────────


class CameraCaptureError(Exception):
    """카메라 capture 관련 예외 base."""


@dataclass(frozen=True)
class CameraIntrinsic:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    depth_scale: float | None = None  # depth 픽셀값 → m 환산 계수 (None = depth 없음)


@dataclass(frozen=True)
class ColorFrame:
    rgb: np.ndarray  # (H, W, 3) uint8 BGR
    timestamp: float


@dataclass(frozen=True)
class DepthFrame:
    color_aligned: np.ndarray  # (H, W, 3) uint8 BGR
    depth: np.ndarray  # (H, W) uint16 Z16
    timestamp: float
    intrinsic: CameraIntrinsic


class CameraCaptureProtocol(Protocol):
    """RGBD 카메라의 통합 인터페이스.

    `multi_robot_architecture.md` §3.4 의 design API. 기존 facade 가 이 Protocol 만족
    + legacy method 도 함께 제공 (caller migration 점진).
    """

    def open(self) -> bool: ...
    def close(self) -> None: ...

    @property
    def is_opened(self) -> bool: ...

    def set_depth_enabled(self, enabled: bool) -> None: ...

    def read_color(self) -> ColorFrame | None: ...

    def read_depth_frame(self) -> DepthFrame | None: ...


# ─── 구현체: RealSense (현재 default) ─────────────────────────


class CameraCapture:
    """RealSense D405 wrap. CameraCaptureProtocol 만족 + legacy method (read /
    read_aligned / set_cloud_enabled / depth_scale property) 제공.

    Singleton — 내부 `RealsenseCapture()` 도 singleton 이라 process 당 1 인스턴스.
    미래 robot 별 인스턴스 분리는 multi-robot 진행 시 RobotRegistry 와 결합.
    """

    def __init__(self):
        self._rs = RealsenseCapture()

    # ─── Lifecycle ───────────────────────────────────────────

    def open(self) -> bool:
        return self._rs.open()

    def close(self) -> None:
        self._rs.close()

    @property
    def is_opened(self) -> bool:
        return self._rs.is_opened

    # ─── Protocol API (clean names) ──────────────────────────

    def set_depth_enabled(self, enabled: bool) -> None:
        self._rs.set_cloud_enabled(enabled)

    def read_color(self) -> ColorFrame | None:
        ok, rgb = self._rs.read_color()
        if not ok or rgb is None:
            return None
        # RealsenseCapture 가 timestamp 노출 안 함 — frame-by-frame 가능시 추후
        # _producer_loop 갱신 시점 시각 추가. 지금은 0 (caller 가 자기 시각 부여).
        return ColorFrame(rgb=rgb, timestamp=0.0)

    def read_depth_frame(self) -> DepthFrame | None:
        color, depth = self._rs.read_aligned_color_depth()
        intr = self._rs.depth_intrinsics
        if color is None or depth is None or intr is None:
            return None
        # RealSense intrinsics → CameraIntrinsic 변환
        ci = CameraIntrinsic(
            fx=float(intr.fx),
            fy=float(intr.fy),
            cx=float(intr.ppx),
            cy=float(intr.ppy),
            width=int(intr.width),
            height=int(intr.height),
            depth_scale=float(self._rs.depth_scale),
        )
        return DepthFrame(
            color_aligned=color,
            depth=depth,
            timestamp=0.0,
            intrinsic=ci,
        )

    # ─── Legacy aliases (기존 camera_node 사용) ──────────────

    def read(self) -> tuple[bool, np.ndarray | None]:
        return self._rs.read_color()

    def set_cloud_enabled(self, enabled: bool) -> None:
        self.set_depth_enabled(enabled)

    def read_aligned(
        self,
    ) -> tuple[np.ndarray | None, np.ndarray | None, Any]:
        color, depth = self._rs.read_aligned_color_depth()
        intr = self._rs.depth_intrinsics
        return color, depth, intr

    # ─── Properties (legacy) ─────────────────────────────────

    @property
    def width(self) -> int:
        return self._rs.width

    @property
    def height(self) -> int:
        return self._rs.height

    @property
    def fps(self) -> float:
        return self._rs.fps

    @property
    def depth_scale(self) -> float:
        return self._rs.depth_scale
