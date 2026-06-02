"""CAMERA_STREAM_RAW / CAMERA_STATE_STATUS 토픽 구독 + JPEG 디코드 캐시.
robot_id 차원 도입 (JointStateCache 와 동일 패턴).

multi_robot_architecture.md §4.5 / distributed_topology.md §8. state: dict[robot_id].

Phase 1 (토픽 namespace 미정정) 에선 단일 CAMERA_STREAM_RAW 토픽 구독 →
default robot_id 로 저장. Phase 2 (todo 7: 토픽 namespace 정정) 에서 robot 별
토픽 (`<robot_id>/camera/stream/raw`) 으로 분리.
"""

from __future__ import annotations

import logging
import threading
from typing import TYPE_CHECKING

import cv2
import numpy as np

from core.transport.messages.camera import CameraStatus
from core.robot.robot_registry import RobotRegistry
from core.transport.topic_map import Topic

if TYPE_CHECKING:
    from core.transport.base_node import BaseNode

logger = logging.getLogger(__name__)


class FrameCache:
    _instance: "FrameCache | None" = None
    _new_lock = threading.Lock()

    def __new__(cls) -> "FrameCache":
        if cls._instance is None:
            with cls._new_lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self) -> None:
        if self._initialized:
            return
        self._initialized = True
        self._lock = threading.Lock()
        self._latest_jpeg_by_robot: dict[str, bytes] = {}
        self._latest_status_by_robot: dict[str, CameraStatus] = {}
        self._subscribed_robots: set[str] = set()

    def _resolve(self, robot_id: str | None) -> str:
        return robot_id if robot_id is not None else RobotRegistry().default_robot_id()

    def subscribe(self, node: "BaseNode", robot_id: str | None = None) -> None:
        """해당 robot 의 camera 토픽 구독. None 이면 default robot.

        TODO (todo 7): robot 별 토픽 namespace 분리 시 Topic.CAMERA_STREAM_RAW 가
        함수형 (`Topic.camera_stream_raw(robot_id)`) 으로 변경되면 호출도 갱신.
        """
        rid = self._resolve(robot_id)
        if rid in self._subscribed_robots:
            return
        self._subscribed_robots.add(rid)
        node.create_raw_subscriber(
            Topic.CAMERA_STREAM_RAW,
            lambda payload, _rid=rid: self._on_frame(_rid, payload),
        )
        node.create_subscriber(
            Topic.CAMERA_STATE_STATUS,
            CameraStatus,
            lambda status, _rid=rid: self._on_status(_rid, status),
        )

    def _on_frame(self, robot_id: str, payload: bytes) -> None:
        with self._lock:
            self._latest_jpeg_by_robot[robot_id] = payload

    def _on_status(self, robot_id: str, status: CameraStatus) -> None:
        with self._lock:
            self._latest_status_by_robot[robot_id] = status

    def get_frame(
        self, robot_id: str | None = None
    ) -> tuple[bool, np.ndarray | None]:
        rid = self._resolve(robot_id)
        with self._lock:
            jpeg = self._latest_jpeg_by_robot.get(rid)
        if jpeg is None:
            return False, None
        try:
            arr = np.frombuffer(jpeg, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                return False, None
            return True, frame
        except Exception as e:
            logger.error("FrameCache JPEG decode 실패 (robot=%s): %s", rid, e)
            return False, None

    def width(self, robot_id: str | None = None) -> int | None:
        rid = self._resolve(robot_id)
        with self._lock:
            status = self._latest_status_by_robot.get(rid)
            return status.width if status is not None else None

    def height(self, robot_id: str | None = None) -> int | None:
        rid = self._resolve(robot_id)
        with self._lock:
            status = self._latest_status_by_robot.get(rid)
            return status.height if status is not None else None

    def is_ready(self, robot_id: str | None = None) -> bool:
        rid = self._resolve(robot_id)
        with self._lock:
            return rid in self._latest_jpeg_by_robot
