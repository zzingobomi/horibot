from typing import Any

import numpy as np

from core.realsense_capture import RealsenseCapture


class CameraCapture:
    def __init__(self):
        self._rs = RealsenseCapture()

    # ─── Lifecycle ───────────────────────────────────────────

    def open(self) -> bool:
        return self._rs.open()

    def close(self) -> None:
        self._rs.close()

    # ─── Color (라이브 JPEG 스트림용) ────────────────────────

    def read(self) -> tuple[bool, np.ndarray | None]:
        return self._rs.read_color()

    # ─── Aligned color + depth (포인트클라우드 스트림용) ────

    def set_cloud_enabled(self, enabled: bool) -> None:
        self._rs.set_cloud_enabled(enabled)

    def read_aligned(
        self,
    ) -> tuple[np.ndarray | None, np.ndarray | None, Any]:
        color, depth = self._rs.read_aligned_color_depth()
        intr = self._rs.depth_intrinsics
        return color, depth, intr

    def grab_n_aligned_blocking(
        self,
        n: int,
        timeout: float = 2.0,
    ) -> list[tuple[np.ndarray, np.ndarray, Any]]:
        return self._rs.grab_n_aligned_blocking(n, timeout=timeout)

    # ─── Properties ──────────────────────────────────────────

    @property
    def is_opened(self) -> bool:
        return self._rs.is_opened

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
