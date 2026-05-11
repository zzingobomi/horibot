import logging
import threading
from typing import TYPE_CHECKING

import cv2
import numpy as np

from core.topic_map import Topic

if TYPE_CHECKING:
    from core.base_node import BaseNode

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
        self._latest_jpeg: bytes | None = None
        self._latest_status: dict = {}
        self._subscribed = False

    def subscribe(self, node: "BaseNode") -> None:
        if self._subscribed:
            return
        self._subscribed = True
        node.create_raw_subscriber(Topic.CAMERA_STREAM_RAW, self._on_frame)
        node.create_subscriber(Topic.CAMERA_STATE_STATUS, self._on_status)

    def _on_frame(self, payload: bytes) -> None:
        with self._lock:
            self._latest_jpeg = payload

    def _on_status(self, status: dict) -> None:
        with self._lock:
            self._latest_status = status

    def get_frame(self) -> tuple[bool, np.ndarray | None]:
        with self._lock:
            jpeg = self._latest_jpeg
        if jpeg is None:
            return False, None
        try:
            arr = np.frombuffer(jpeg, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                return False, None
            return True, frame
        except Exception as e:
            logger.error("FrameCache JPEG decode 실패: %s", e)
            return False, None

    @property
    def width(self) -> int | None:
        with self._lock:
            return self._latest_status.get("width")

    @property
    def height(self) -> int | None:
        with self._lock:
            return self._latest_status.get("height")

    @property
    def is_ready(self) -> bool:
        with self._lock:
            return self._latest_jpeg is not None
