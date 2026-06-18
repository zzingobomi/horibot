"""카메라 없이 CAMERA_STREAM_RAW / STATE_STATUS / CAMERA_DEPTH_FRAME 충족 mock.

합성 JPEG (Mock label + frame counter + 움직이는 dot) 30Hz 발행 — frontend
MJPEG bridge / FrameCache 기반 detector / calibration 그대로 동작.

depth 스트림 — enable 토글 ON 시 합성 depth_frame (gradient uint16 + JPEG
color) 8Hz publish. Scene3DNode 의 snapshot / live stream e2e 가능 — host_mock
ScanTask CaptureScan 자체 자리 self-contained 통과.
"""

import logging
import os
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

        # ChArUco eye-in-hand 시뮬 — CALIB_SIM_BOARD=1 일 때만 (headless 캘 e2e).
        # 로봇 joint 상태로부터 board_in_cam 을 계산해 보드 렌더 → 캡처→BA 전체
        # 파이프라인 검증 (modules/calibration/sim_board.py). 기본 off → 평소
        # host_mock 동작 불변.
        self._calib_sim = os.environ.get("CALIB_SIM_BOARD") == "1"
        self._joint_cache = None
        self._fk_chain = None
        self._arm_cfgs = None
        self._sim_K = np.array(
            [[MOCK_FX, 0.0, MOCK_CX], [0.0, MOCK_FY, MOCK_CY], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        self._sim_dist = np.zeros(5, dtype=np.float64)
        if self._calib_sim:
            from core.cache.joint_state_cache import JointStateCache
            from core.robot.robot_registry import RobotRegistry
            from modules.motor.motor_config import load_motor_layout

            self._joint_cache = JointStateCache()
            self._fk_chain = RobotRegistry().get_fk_chain(robot_id)
            self._arm_cfgs = load_motor_layout(robot_id).arm
            logger.info("[%s] mock camera ChArUco 시뮬 모드 ON", robot_id)

        self.create_service(
            self.r(Service.CAMERA_SET_DEPTH_STREAM),
            CameraSetDepthStreamReq,
            CameraSetDepthStreamRes,
            self._srv_set_depth_stream,
        )

    def start(self) -> None:
        super().start()
        if self._calib_sim and self._joint_cache is not None:
            self._joint_cache.subscribe(self)
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
        if self._calib_sim:
            sim = self._make_sim_charuco_frame()
            if sim is not None:
                return sim
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

    def _make_sim_charuco_frame(self) -> np.ndarray | None:
        """현재 joint 상태 → board_in_cam → ChArUco 렌더 (eye-in-hand 시뮬).

        joint 미수신이면 None (caller 가 기본 mock frame fallback).
        """
        if self._joint_cache is None or self._fk_chain is None:
            return None
        try:
            from modules.calibration import sim_board
            from modules.calibration.se3 import make_T

            angles = self._joint_cache.get_joint_angles_rad(
                self._arm_cfgs, robot_id=self.robot_id
            )
            if angles is None:
                return None
            n = self._fk_chain.n_arm
            Z = np.zeros((n, 3), dtype=np.float64)
            R, t = self._fk_chain.fk(np.asarray(angles[:n], dtype=np.float64), Z, Z)
            T_gb = make_T(np.asarray(R), np.asarray(t).reshape(3))
            board_in_cam = sim_board.board_in_cam_from_fk(T_gb)
            return sim_board.render_charuco_at_pose(
                width=MOCK_WIDTH,
                height=MOCK_HEIGHT,
                camera_matrix=self._sim_K,
                dist_coeffs=self._sim_dist,
                board_in_cam=board_in_cam,
            )
        except Exception as e:
            logger.debug("[%s] sim charuco 렌더 실패: %s", self.robot_id, e)
            return None

    def _stream_loop(self) -> None:
        interval = 1.0 / STREAM_FPS
        session = ZenohSession.get()
        topic = self.r(Topic.CAMERA_STREAM_RAW)
        while self._running:
            try:
                jpeg = frame_to_jpeg_bytes(self._make_frame())
                session.put(topic, jpeg)
                self._frame_counter += 1
                # status 주기적 republish (~1s) — 분산 토폴로지에서 늦게 뜨는
                # 구독자(pc 의 FrameCache)가 1회성 status 를 놓치는 문제 방지.
                if self._frame_counter % STREAM_FPS == 0:
                    self._publish_status()
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
