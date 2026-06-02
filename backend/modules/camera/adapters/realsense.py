"""RealSenseCapture — `pyrealsense2` 기반 CameraCapture adapter.

multi_robot_architecture.md §3.4 / distributed_topology.md §6 참조.

내부 [`RealsenseCapture`](../../../core/realsense_capture.py) singleton 의 raw SDK
wrap 을 그대로 활용. 이 adapter 는 `CameraCapture` Protocol 의 method (clean
names) + 기존 camera_node 가 사용하는 legacy method (read / read_aligned /
set_cloud_enabled / depth_scale property) 양쪽 다 제공.

caller migration 은 점진 — 새 코드는 Protocol method (`read_color` /
`read_depth_frame` / `set_depth_enabled`) 사용, 기존 camera_node 는 legacy.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from core.realsense_capture import RealsenseCapture
from modules.camera.capture import CameraIntrinsic, ColorFrame, DepthFrame


class RealSenseCapture:
    """RealSense D405 wrap. `CameraCapture` Protocol 만족.

    Singleton — 내부 `RealsenseCapture()` 도 singleton 이라 process 당 1 인스턴스.
    multi-robot 진행 시 robot 별 인스턴스 분리는 `RobotRegistry.get_camera_capture()`
    factory 와 결합 (Phase 2+).
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
