"""CameraDriver Protocol — Camera 도메인의 hardware adapter 계약.

backend_v2_modules.md §6.2 (driver Protocol) + §7.3 (driver self-declare)
+ §7.6 (factory intrinsic — driver internal only, Calibration 의 public X).
"""

from __future__ import annotations

from typing import Protocol

from ..contract import CameraCapabilities


class RawColorFrame:
    """driver capture 결과 — BGR ndarray + width / height. encode 는 Module."""

    __slots__ = ("ndarray_bytes", "width", "height")

    def __init__(self, ndarray_bytes: bytes, width: int, height: int) -> None:
        self.ndarray_bytes = ndarray_bytes
        self.width = width
        self.height = height


class RawDepthFrame:
    """driver capture 결과 — uint16 depth ndarray bytes + scale."""

    __slots__ = ("depth_bytes", "width", "height", "depth_scale")

    def __init__(
        self,
        depth_bytes: bytes,
        width: int,
        height: int,
        depth_scale: float,
    ) -> None:
        self.depth_bytes = depth_bytes
        self.width = width
        self.height = height
        self.depth_scale = depth_scale


class FactoryIntrinsics:
    """driver-reported factory intrinsic — Calibration seed only (§7.6).

    public service 박지 X — Calibration Module 안에서만 호출.
    """

    __slots__ = ("fx", "fy", "cx", "cy", "width", "height")

    def __init__(
        self,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        width: int,
        height: int,
    ) -> None:
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.width = width
        self.height = height


class CameraDriver(Protocol):
    """Hardware adapter — RealSense / USB UVC / Basler / mock 의 공통 계약."""

    # ── self-declare ──
    def capabilities(self) -> CameraCapabilities: ...

    # ── lifecycle ──
    def open(self) -> None: ...
    def close(self) -> None: ...

    # ── capture ──
    def capture_color(self) -> RawColorFrame: ...
    def capture_depth(self) -> RawDepthFrame | None: ...

    # ── factory intrinsic (Module internal — Calibration seed) ──
    def get_factory_intrinsics(self) -> FactoryIntrinsics | None: ...
