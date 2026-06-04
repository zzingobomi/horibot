import logging
import threading
import time

from core.transport.base_node import BaseNode
from core.transport.messages.base import ServiceRequest, ServiceResponse
from core.transport.messages.camera import (
    CameraSetDepthStreamReq,
    CameraSetDepthStreamRes,
    CameraStatus,
)
from core.robot.robot_registry import RobotRegistry
from core.transport.topic_map import Service, Topic
from core.transport.zenoh_session import ZenohSession
from modules.camera.depth_frame import encode as encode_depth_frame
from modules.camera.stream import frame_to_jpeg_bytes

logger = logging.getLogger(__name__)

STREAM_FPS = 30
DEPTH_FPS = 8
DEPTH_IDLE_SLEEP = 0.1


class CameraNode(BaseNode):
    def __init__(self, robot_id: str | None = None):
        super().__init__("camera_node", robot_id=robot_id)
        self.camera = RobotRegistry().get_camera_capture(robot_id)
        self._stream_thread: threading.Thread | None = None
        self._depth_thread: threading.Thread | None = None

        self._depth_lock = threading.Lock()
        self._depth_enabled = False

    def start(self) -> None:
        connected = self.camera.open()
        self._publish_status(connected)
        if connected:
            self.log("info", "카메라 노드 시작 (RealSense D405)")
        else:
            self.log("error", "카메라 연결 실패")

        self.create_service(
            self.r(Service.CAMERA_SET_DEPTH_STREAM),
            CameraSetDepthStreamReq,
            CameraSetDepthStreamRes,
            self._srv_set_depth_stream,
        )

        super().start()

        self._stream_thread = threading.Thread(
            target=self._stream_loop,
            name="camera-stream",
            daemon=True,
        )
        self._stream_thread.start()

        self._depth_thread = threading.Thread(
            target=self._depth_loop,
            name="camera-depth-stream",
            daemon=True,
        )
        self._depth_thread.start()

    def stop(self) -> None:
        super().stop()
        self.camera.close()

    # ─── Color 스트림 ────────────────────────────────────────

    def _stream_loop(self) -> None:
        interval = 1.0 / STREAM_FPS
        session = ZenohSession.get()
        last_connected = self.camera.is_opened

        while self._running:
            if not self.camera.is_opened:
                if last_connected:  # 연결 → 끊김
                    self._publish_status(False)
                    last_connected = False
                time.sleep(1.0)
                continue

            if not last_connected:  # 끊김 → 연결
                self._publish_status(True)
                last_connected = True

            ret, frame = self.camera.read()
            if ret and frame is not None:
                try:
                    jpeg_bytes = frame_to_jpeg_bytes(frame)
                    session.put(self.r(Topic.CAMERA_STREAM_RAW), jpeg_bytes)
                except Exception as e:
                    logger.error(f"color 스트림 발행 오류: {e}")

            time.sleep(interval)

    # ─── Depth 스트림 ────────────────────────────────────────

    def _depth_loop(self) -> None:
        period = 1.0 / DEPTH_FPS
        session = ZenohSession.get()

        while self._running:
            with self._depth_lock:
                enabled = self._depth_enabled

            if not enabled or not self.camera.is_opened:
                time.sleep(DEPTH_IDLE_SLEEP)
                continue

            t0 = time.time()
            color, depth, intr = self.camera.read_aligned()
            if color is None or depth is None or intr is None:
                # 첫 프레임은 RealsenseDriver 내부 producer가 cloud_enabled를 받고 다음 사이클에 채움. 잠깐 대기.
                time.sleep(DEPTH_IDLE_SLEEP)
                continue

            try:
                payload = encode_depth_frame(
                    timestamp=time.time(),
                    color_bgr=color,
                    depth_z16=depth,
                    depth_scale=self.camera.depth_scale,
                    fx=intr.fx,
                    fy=intr.fy,
                    cx=intr.ppx,
                    cy=intr.ppy,
                )
                session.put(self.r(Topic.CAMERA_DEPTH_FRAME), payload)
            except Exception as e:
                logger.warning(f"depth_frame 인코드/발행 실패: {e}")

            elapsed = time.time() - t0
            time.sleep(max(0.0, period - elapsed))

    # ─── Services ────────────────────────────────────────────

    def _srv_set_depth_stream(
        self, req: ServiceRequest[CameraSetDepthStreamReq]
    ) -> ServiceResponse[CameraSetDepthStreamRes]:
        enabled = req.data.enabled
        self.camera.set_cloud_enabled(enabled)
        with self._depth_lock:
            self._depth_enabled = enabled
        logger.info("depth 스트림 %s", "ON" if enabled else "OFF")
        return ServiceResponse(
            success=True,
            message="ok",
            data=CameraSetDepthStreamRes(enabled=enabled),
        )

    # ─── Status ──────────────────────────────────────────────

    def _publish_status(self, connected: bool) -> None:
        self.publish(
            self.r(Topic.CAMERA_STATE_STATUS),
            CameraStatus(
                timestamp=time.time(),
                connected=connected,
                width=self.camera.width,
                height=self.camera.height,
                fps=self.camera.fps,
                depth_scale=self.camera.depth_scale,
            ),
        )
