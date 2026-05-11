import time
import logging
import threading
import cv2
import numpy as np

from core.common import GRIPPER_ID
from core.base_node import BaseNode
from core.frame_cache import FrameCache
from core.joint_state_cache import JointStateCache
from core.topic_map import Service, Topic
from modules.calibration.loader import load_calibration
from modules.detector.yolo_detector import YoloDetector
from modules.dynamixel.motor_config import MotorConfig, load_motor_config


logger = logging.getLogger(__name__)

DETECTION_INTERVAL = 0.2  # 5fps (초)


class DetectorNode(BaseNode):
    def __init__(self) -> None:
        super().__init__("detector_node")

        self._frame_cache = FrameCache()
        _, self._motor_cfgs = load_motor_config()
        self._arm_cfgs: list[MotorConfig] = [
            m for m in self._motor_cfgs if m.id != GRIPPER_ID
        ]
        self._joint_cache = JointStateCache()

        self._calib = load_calibration()
        if not self._calib.is_ready():
            logger.warning(
                "DetectorNode: 캘리브레이션 미완료 (intrinsic=%s, hand_eye=%s)",
                self._calib.intrinsic is not None,
                self._calib.hand_eye is not None,
            )

        self._detector = YoloDetector()
        self._detection_thread: threading.Thread | None = None

    def start(self) -> None:
        self._joint_cache.subscribe(self)
        self._frame_cache.subscribe(self)
        self.create_service(Service.DETECT_SERVICE, self._handle_detect)
        super().start()

        self._detection_thread = threading.Thread(
            target=self._detection_loop,
            daemon=True,
            name="detector-loop",
        )
        self._detection_thread.start()
        logger.info("DetectorNode 시작")

    def _detection_loop(self) -> None:
        while self._running:
            try:
                ret, frame = self._frame_cache.get_frame()
                if ret and frame is not None:
                    results = self._detector.raw_detect(frame)
                    self.publish(
                        Topic.DETECTOR_STATE,
                        {
                            "timestamp": time.time(),
                            "detections": results,
                        },
                    )
            except Exception as e:
                logger.debug("detection loop 오류: %s", e)
            time.sleep(DETECTION_INTERVAL)

    def _handle_detect(self, req: dict) -> dict:
        if not self._calib.is_ready():
            return {"success": False, "message": "캘리브레이션 미완료", "data": {}}

        # ── 카메라 프레임 취득 ────────────────────
        ret, frame = self._frame_cache.get_frame()
        if not ret or frame is None:
            return {"success": False, "message": "카메라 프레임 취득 실패", "data": {}}

        # ── 물체 감지 → image centroid ────────────
        result = self._detector.detect(frame)
        if result is None:
            return {"success": False, "message": "물체 감지 실패", "data": {}}

        cx, cy = result
        logger.info("감지: centroid (%.1f, %.1f)", cx, cy)

        # ── image → 정규화 좌표 ───────────────────
        camera_matrix = self._calib.intrinsic.camera_matrix
        dist_coeffs = self._calib.intrinsic.dist_coeffs

        pt = np.array([[[cx, cy]]], dtype=np.float32)
        pt_undistorted = cv2.undistortPoints(pt, camera_matrix, dist_coeffs)
        xn = float(pt_undistorted[0, 0, 0])
        yn = float(pt_undistorted[0, 0, 1])

        # ── FK: get_tcp → R_be, t_be ──────────────
        res = self.call_service(Service.MOTION_GET_TCP, {})
        if not res.get("success"):
            return {
                "success": False,
                "message": f"get_tcp 실패: {res.get('message')}",
                "data": {},
            }

        tcp_data = res.get("data", {})
        pos = tcp_data.get("position")
        quat = tcp_data.get("quaternion")
        if pos is None or quat is None:
            return {"success": False, "message": "TCP pose 없음", "data": {}}

        R_be = _quat_to_rot(quat)  # end-effector → base
        t_be = np.array(pos)

        # ── hand-eye 행렬 ─────────────────────────
        R_ce = self._calib.hand_eye.R  # camera → end-effector
        t_ce = self._calib.hand_eye.t.flatten()

        # ── base frame Z=0 조건으로 Z_cam 역산 ───
        # obj_in_base = R_be @ (R_ce @ [xn*Z, yn*Z, Z] + t_ce) + t_be
        # obj_in_base[2] = 0 으로 Z에 대해 풀면:
        # Z * (R_total[2,0]*xn + R_total[2,1]*yn + R_total[2,2]) + t_total[2] = 0
        R_total = R_be @ R_ce
        t_total = R_be @ t_ce + t_be

        denom = R_total[2, 0] * xn + R_total[2, 1] * yn + R_total[2, 2]
        if abs(denom) < 1e-6:
            return {"success": False, "message": "Z_cam 역산 실패 (분모 0)", "data": {}}

        Z_cam = -t_total[2] / denom
        if Z_cam <= 0:
            return {
                "success": False,
                "message": f"Z_cam 음수 ({Z_cam:.3f}), 캘리브레이션 확인 필요",
                "data": {},
            }

        logger.info("Z_cam 역산: %.3fm", Z_cam)

        # ── camera frame → base frame ─────────────
        obj_in_cam = np.array([xn * Z_cam, yn * Z_cam, Z_cam])
        obj_in_ee = R_ce @ obj_in_cam + t_ce
        obj_in_base = R_be @ obj_in_ee + t_be

        logger.info("감지 완료: base=(%.3f, %.3f, %.3f)", *obj_in_base)

        return {
            "success": True,
            "message": "ok",
            "data": {"position": obj_in_base.tolist()},
        }


def _quat_to_rot(quat: list[float]) -> np.ndarray:
    """quaternion [x, y, z, w] → 3x3 회전 행렬."""
    x, y, z, w = quat
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
