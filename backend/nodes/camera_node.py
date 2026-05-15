import base64
import logging
import threading
import time

from core.base_node import BaseNode
from core.topic_map import Service, Topic
from core.zenoh_session import ZenohSession
from modules.camera.capture import CameraCapture
from modules.camera.depth_frame import (
    encode as encode_depth_frame,
    envelope_encode,
)
from modules.camera.stream import frame_to_jpeg_bytes

logger = logging.getLogger(__name__)

STREAM_FPS = 30
DEPTH_FPS = 8
DEPTH_IDLE_SLEEP = 0.1


class CameraNode(BaseNode):
    def __init__(self):
        super().__init__("camera_node")
        self.camera = CameraCapture()
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
            Service.CAMERA_SET_DEPTH_STREAM,
            self._srv_set_depth_stream,
        )
        self.create_service(
            Service.CAMERA_CAPTURE_DEPTH_FRAMES,
            self._srv_capture_depth_frames,
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
                    session.put(Topic.CAMERA_STREAM_RAW, jpeg_bytes)
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
                # 첫 프레임은 RealsenseCapture 내부 producer가 cloud_enabled를 받고 다음 사이클에 채움. 잠깐 대기.
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
                session.put(Topic.CAMERA_DEPTH_FRAME, payload)
            except Exception as e:
                logger.warning(f"depth_frame 인코드/발행 실패: {e}")

            elapsed = time.time() - t0
            time.sleep(max(0.0, period - elapsed))

    # ─── Services ────────────────────────────────────────────

    def _srv_set_depth_stream(self, req: dict) -> dict:
        data = req.get("data", {}) or {}
        if "enabled" not in data:
            return {
                "success": False,
                "message": "'enabled' 필드 필요",
                "data": {},
            }

        enabled = bool(data["enabled"])
        self.camera.set_cloud_enabled(enabled)
        with self._depth_lock:
            self._depth_enabled = enabled
        logger.info("depth 스트림 %s", "ON" if enabled else "OFF")
        return {
            "success": True,
            "message": "ok",
            "data": {"enabled": enabled},
        }

    def _srv_capture_depth_frames(self, req: dict) -> dict:
        """정지 자세에서 N개의 raw depth_frame을 묶어 base64 응답으로 반환.

        스트림 ON이든 OFF든 호출 가능. OFF면 임시로 cloud_enabled 켜고 N장 수집 후 복원.
        """
        if not self.camera.is_opened:
            return {
                "success": False,
                "message": "카메라 연결 안 됨",
                "data": {},
            }

        data = req.get("data", {}) or {}
        try:
            n = int(data.get("num_frames", 5))
        except (TypeError, ValueError):
            return {
                "success": False,
                "message": "num_frames는 정수여야 함",
                "data": {},
            }
        if n <= 0 or n > 30:
            return {
                "success": False,
                "message": "num_frames는 1~30 범위",
                "data": {},
            }
        try:
            timeout = float(data.get("timeout", 2.0))
        except (TypeError, ValueError):
            timeout = 2.0

        frames = self.camera.grab_n_aligned_blocking(n, timeout=timeout)
        if not frames:
            return {
                "success": False,
                "message": f"frame 획득 실패 ({n}장 타임아웃)",
                "data": {},
            }

        depth_scale = self.camera.depth_scale
        encoded: list[bytes] = []
        ts0 = time.time()
        for i, (color, depth, intr) in enumerate(frames):
            try:
                encoded.append(
                    encode_depth_frame(
                        timestamp=ts0 + i * 1e-6,
                        color_bgr=color,
                        depth_z16=depth,
                        depth_scale=depth_scale,
                        fx=intr.fx,
                        fy=intr.fy,
                        cx=intr.ppx,
                        cy=intr.ppy,
                    )
                )
            except Exception as e:
                return {
                    "success": False,
                    "message": f"depth_frame 인코드 실패: {e}",
                    "data": {},
                }

        payload = envelope_encode(encoded)
        payload_b64 = base64.b64encode(payload).decode("ascii")
        return {
            "success": True,
            "message": "ok",
            "data": {
                "num_frames": len(encoded),
                "payload_b64": payload_b64,
            },
        }

    # ─── Status ──────────────────────────────────────────────

    def _publish_status(self, connected: bool) -> None:
        self.publish(
            Topic.CAMERA_STATE_STATUS,
            {
                "timestamp": time.time(),
                "connected": connected,
                "width": self.camera.width,
                "height": self.camera.height,
                "fps": self.camera.fps,
                "depth_scale": self.camera.depth_scale,
            },
        )
