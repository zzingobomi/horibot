"""카메라 없이 CAMERA_STREAM_RAW / STATE_STATUS topic 만 충족시키는 mock 노드.

합성 JPEG (Mock label + frame counter + 움직이는 dot) 을 30Hz 로 발행 — frontend
MJPEG bridge route / FrameCache 기반 detector / calibration 도 그대로 돌아감 (실
검출 결과는 의미 없지만 topic path / UI 흐름 검증용).

depth 스트림은 enable 토글만 받고 실 payload 발행 X — PointCloud 노드는 빈
상태로 남음. D405 가 so101_0 으로 양도되면 그때 실 hardware 로 재검증.
"""

import logging
import threading
import time

import cv2
import numpy as np

from core.transport.base_node import BaseNode
from core.transport.messages.base import ServiceRequest, ServiceResponse
from core.transport.messages.camera import (
    CameraSetDepthStreamReq,
    CameraSetDepthStreamRes,
    CameraStatus,
)
from core.transport.topic_map import Service, Topic
from core.transport.zenoh_session import ZenohSession
from modules.camera.stream import frame_to_jpeg_bytes

logger = logging.getLogger(__name__)

STREAM_FPS = 30
MOCK_WIDTH = 640
MOCK_HEIGHT = 480
MOCK_FPS = float(STREAM_FPS)
MOCK_DEPTH_SCALE = 0.001  # D405 실 hardware 와 동일 단위 (m / raw)


class MockCameraNode(BaseNode):
    def __init__(self, robot_id: str | None = None):
        # heartbeat node name = real camera_node 와 동일 — frontend 가 mock/real
        # 무관 동일 lookup 가능 (CLAUDE.md "mock 노드는 contract 만 충족" 정합).
        super().__init__("camera_node", robot_id=robot_id)
        self._depth_enabled = False
        self._stream_thread: threading.Thread | None = None
        self._frame_counter = 0

        self.create_service(
            self.r(Service.CAMERA_SET_DEPTH_STREAM),
            CameraSetDepthStreamReq,
            CameraSetDepthStreamRes,
            self._srv_set_depth_stream,
        )

    def start(self) -> None:
        super().start()
        self._publish_status()
        self.log("info", "mock camera 노드 시작")
        self._stream_thread = threading.Thread(
            target=self._stream_loop,
            name="mock-camera-stream",
            daemon=True,
        )
        self._stream_thread.start()

    # ─── Status ──────────────────────────────────────────────

    def _publish_status(self) -> None:
        self.publish(
            self.r(Topic.CAMERA_STATE_STATUS),
            CameraStatus(
                timestamp=time.time(),
                connected=True,
                width=MOCK_WIDTH,
                height=MOCK_HEIGHT,
                fps=MOCK_FPS,
                depth_scale=MOCK_DEPTH_SCALE,
            ),
        )

    # ─── Stream loop ─────────────────────────────────────────

    def _make_frame(self) -> np.ndarray:
        frame = np.zeros((MOCK_HEIGHT, MOCK_WIDTH, 3), dtype=np.uint8)
        frame[:] = (32, 28, 24)  # dark slate
        cv2.putText(
            frame,
            "MOCK CAMERA",
            (40, MOCK_HEIGHT // 2 - 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.4,
            (220, 220, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"frame #{self._frame_counter}  {time.strftime('%H:%M:%S')}",
            (40, MOCK_HEIGHT // 2 + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (180, 180, 200),
            1,
            cv2.LINE_AA,
        )
        cx = 40 + (self._frame_counter % (MOCK_WIDTH - 80))
        cv2.circle(frame, (cx, MOCK_HEIGHT - 60), 18, (120, 200, 0), -1)
        return frame

    def _stream_loop(self) -> None:
        interval = 1.0 / STREAM_FPS
        session = ZenohSession.get()
        topic = self.r(Topic.CAMERA_STREAM_RAW)
        while self._running:
            try:
                jpeg = frame_to_jpeg_bytes(self._make_frame())
                session.put(topic, jpeg)
                self._frame_counter += 1
            except Exception as e:
                logger.error(f"mock camera 스트림 발행 오류: {e}")
            time.sleep(interval)

    # ─── Services ────────────────────────────────────────────

    def _srv_set_depth_stream(
        self, req: ServiceRequest[CameraSetDepthStreamReq]
    ) -> ServiceResponse[CameraSetDepthStreamRes]:
        # mock: enable 플래그만 echo back. 실 depth payload 발행 X.
        self._depth_enabled = bool(req.data.enabled)
        return ServiceResponse(
            success=True,
            message="mock ok",
            data=CameraSetDepthStreamRes(enabled=self._depth_enabled),
        )
