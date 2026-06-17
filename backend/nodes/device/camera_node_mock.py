"""카메라 없이 CAMERA_STREAM_RAW / STATE_STATUS / CAMERA_DEPTH_FRAME 충족 mock.

합성 JPEG (Mock label + frame counter + 움직이는 dot) 30Hz 발행 — frontend
MJPEG bridge / FrameCache 기반 detector / calibration 그대로 동작.

depth 스트림 — enable 토글 ON 시 합성 depth_frame (gradient uint16 + JPEG
color) 8Hz publish. Scene3DNode 의 snapshot / live stream e2e 가능 — host_mock
ScanTask CaptureScan 자체 자리 self-contained 통과.
"""

import logging
import threading
import time

import cv2
import numpy as np

from core.transport.device_node import DeviceNode
from core.transport.messages.base import ServiceRequest, ServiceResponse
from core.transport.messages.camera import (
    CameraSetDepthStreamReq,
    CameraSetDepthStreamRes,
    CameraStatus,
)
from core.transport.topic_map import Service, Topic
from core.transport.zenoh_session import ZenohSession
from modules.camera.depth_frame import encode as encode_depth_frame
from modules.camera.stream import frame_to_jpeg_bytes

logger = logging.getLogger(__name__)

STREAM_FPS = 30
DEPTH_FPS = 8
MOCK_WIDTH = 640
MOCK_HEIGHT = 480
MOCK_FPS = float(STREAM_FPS)
MOCK_DEPTH_SCALE = 0.001  # D405 실 hardware 와 동일 단위 (m / raw)
# pinhole intrinsic — 640x480 default (mock 자체 자리, 실 hardware 자체 자리는
# factory_intrinsic.npz 자체 자리).
MOCK_FX = 600.0
MOCK_FY = 600.0
MOCK_CX = float(MOCK_WIDTH) / 2.0
MOCK_CY = float(MOCK_HEIGHT) / 2.0


class MockCameraNode(DeviceNode):
    def __init__(self, robot_id: str):
        # heartbeat node name = real camera_node 와 동일 — frontend 가 mock/real
        # 무관 동일 lookup 가능 (CLAUDE.md "mock 노드는 contract 만 충족" 정합).
        super().__init__("camera_node", robot_id=robot_id)
        self._depth_enabled = False
        self._stream_thread: threading.Thread | None = None
        self._depth_thread: threading.Thread | None = None
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
        self._depth_thread = threading.Thread(
            target=self._depth_loop,
            name="mock-camera-depth",
            daemon=True,
        )
        self._depth_thread.start()

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

    # ─── Depth loop — _depth_enabled True 일 때만 8Hz publish ─

    def _make_depth_frame(self) -> np.ndarray:
        """합성 depth uint16 (mm) — 화면 위쪽 멀고 아래쪽 가까운 gradient.

        300mm ~ 800mm 자체 자리 자체 자리 — D405 sweet spot 안. depth_scale=0.001
        과 함께 0.3 ~ 0.8 m 자체 자리 자체 자리 자체 자리 자체 자리.
        """
        depth = np.linspace(800, 300, MOCK_HEIGHT, dtype=np.uint16)[:, None]
        depth = np.broadcast_to(depth, (MOCK_HEIGHT, MOCK_WIDTH)).copy()
        # frame counter 따라 미세하게 변화 — consensus median 자체 자리 자체 자리.
        depth += (self._frame_counter % 5)
        return depth

    def _depth_loop(self) -> None:
        interval = 1.0 / DEPTH_FPS
        session = ZenohSession.get()
        topic = self.r(Topic.CAMERA_DEPTH_FRAME)
        while self._running:
            if not self._depth_enabled:
                time.sleep(interval)
                continue
            try:
                color = self._make_frame()
                depth = self._make_depth_frame()
                payload = encode_depth_frame(
                    timestamp=time.time(),
                    color_bgr=color,
                    depth_z16=depth,
                    depth_scale=MOCK_DEPTH_SCALE,
                    fx=MOCK_FX, fy=MOCK_FY, cx=MOCK_CX, cy=MOCK_CY,
                )
                session.put(topic, payload)
            except Exception as e:
                logger.error(f"mock camera depth_frame 발행 오류: {e}")
            time.sleep(interval)

    # ─── Services ────────────────────────────────────────────

    def _srv_set_depth_stream(
        self, req: ServiceRequest[CameraSetDepthStreamReq]
    ) -> ServiceResponse[CameraSetDepthStreamRes]:
        # mock: enable 플래그 + _depth_loop publish 토글.
        self._depth_enabled = bool(req.data.enabled)
        return ServiceResponse(
            success=True,
            message="mock ok",
            data=CameraSetDepthStreamRes(enabled=self._depth_enabled),
        )
