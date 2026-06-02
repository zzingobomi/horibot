"""CameraCapture Protocol + domain data classes.

multi_robot_architecture.md §3.4 / distributed_topology.md §6 참조.

Protocol 정의 + 공통 데이터 타입만. 구현체 (RealSense / OpenCV / MuJoCo) 는
[adapters/](adapters/) 하위에서 별도 파일로 분리 — kinematics / motor 모듈과 동일
패턴.

데이터 클래스 (`CameraIntrinsic` / `ColorFrame` / `DepthFrame`) 는 frozen dataclass
— process-internal 사용. 토픽 페이로드로 publish 시는 별도 Pydantic model
(`backend/core/transport/messages/camera.py` 의 `DepthFrameHeader` 등).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


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


class CameraCapture(Protocol):
    """RGBD 카메라의 통합 인터페이스.

    [multi_robot_architecture.md §3.4](../../../docs/multi_robot_architecture.md#34-cameracapture-protocol)
    의 design API. 구현체는 이 Protocol 만 만족하면 plug-and-play
    (예: `RealsenseCapture`, 추후 `OpenCVCapture` / `MujocoCapture`).
    """

    def open(self) -> bool: ...
    def close(self) -> None: ...

    @property
    def is_opened(self) -> bool: ...

    def set_depth_enabled(self, enabled: bool) -> None: ...

    def read_color(self) -> ColorFrame | None: ...

    def read_depth_frame(self) -> DepthFrame | None: ...
