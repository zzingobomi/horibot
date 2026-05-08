import time
import logging
import threading
from core.base_node import BaseNode
from core.topic_map import Topic
from core.zenoh_session import ZenohSession
from modules.camera.capture import CameraCapture
from modules.camera.stream import frame_to_jpeg_bytes

logger = logging.getLogger(__name__)

STREAM_FPS = 30


class CameraNode(BaseNode):
    def __init__(self):
        super().__init__("camera_node")
        self.camera = CameraCapture()
        self._stream_thread: threading.Thread | None = None

    def start(self) -> None:
        connected = self.camera.open()
        self._publish_status(connected)
        if connected:
            self.log("info", "카메라 노드 시작 (RealSense D405)")
        else:
            self.log("error", "카메라 연결 실패")

        super().start()

        self._stream_thread = threading.Thread(
            target=self._stream_loop,
            name="camera-stream",
            daemon=True,
        )
        self._stream_thread.start()

    def stop(self) -> None:
        super().stop()
        self.camera.close()

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
                    session.put(Topic.CAMERA_STREAM_RAW, jpeg_bytes)
                except Exception as e:
                    logger.error(f"스트림 발행 오류: {e}")

            time.sleep(interval)

    def _publish_status(self, connected: bool) -> None:
        self.publish(Topic.CAMERA_STATE_STATUS, {
            "timestamp": time.time(),
            "connected": connected,
            "width":     self.camera.width,
            "height":    self.camera.height,
            "fps":       self.camera.fps,
        })
