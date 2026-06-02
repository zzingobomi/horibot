import time
import logging
import threading
import cv2
import numpy as np

from core.common import GRIPPER_ID
from core.base_node import BaseNode
from core.frame_cache import FrameCache
from core.joint_state_cache import JointStateCache
from core.messages.camera import CameraSetDepthStreamReq, CameraSetDepthStreamRes
from core.topic_map import Service, Topic
from modules.calibration.loader import load_calibration
from modules.camera.depth_frame import DepthFrame, decode as decode_depth_frame
from modules.detector.grounded_detector import GroundedDetector
from modules.detector.yolo_detector import YoloDetector
from modules.motor.motor_config import MotorConfig, load_motor_config


logger = logging.getLogger(__name__)

DETECTION_INTERVAL = 0.2  # 5fps (초)

# GroundedDetector 호출 시점의 depth frame 신선도 기준 (초)
DEPTH_FRESH_THRESHOLD = 1.0  # s
# enable 후 새 frame 기다리는 최대 시간
DEPTH_WAIT_TIMEOUT = 5.0  # s


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
        self._grounded = GroundedDetector()
        self._detection_thread: threading.Thread | None = None
        self._grounded_preload_thread: threading.Thread | None = None

        # ─── depth frame 캐시 (grounded_detect 전용) ───
        self._depth_lock = threading.Lock()
        self._latest_depth_frame: DepthFrame | None = None

    def start(self) -> None:
        self._joint_cache.subscribe(self)
        self._frame_cache.subscribe(self)

        self.create_raw_subscriber(
            Topic.CAMERA_DEPTH_FRAME, self._on_depth_frame
        )
        self.create_service(Service.DETECT_SERVICE, self._handle_detect)
        self.create_service(
            Service.PERCEPTION_GROUNDED_DETECT, self._handle_grounded_detect
        )
        super().start()

        self._detection_thread = threading.Thread(
            target=self._detection_loop,
            daemon=True,
            name="detector-loop",
        )
        self._detection_thread.start()

        # Grounding DINO 모델 백그라운드 preload — 첫 detect 호출의 체감 지연 제거.
        # 로드 중에 detect 호출되면 GroundedDetector 내부 lock 이 기다림.
        self._grounded_preload_thread = threading.Thread(
            target=self._preload_grounded,
            daemon=True,
            name="grounded-preload",
        )
        self._grounded_preload_thread.start()

        logger.info("DetectorNode 시작")

    def _preload_grounded(self) -> None:
        try:
            self._grounded.preload()
        except Exception:
            logger.exception("Grounding DINO preload 실패")

    # ─── Subscribers ─────────────────────────────────────────

    def _on_depth_frame(self, payload: bytes) -> None:
        try:
            frame = decode_depth_frame(payload)
        except Exception as e:
            logger.warning("depth_frame 디코드 실패: %s", e)
            return
        with self._depth_lock:
            self._latest_depth_frame = frame

    # ─── Detection loop (YOLO 라이브 5fps) ──────────────────

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

    # ─── Service: YOLO + plane Z=0 (기존) ───────────────────

    def _handle_detect(self, req: dict) -> dict:
        if not self._calib.is_ready():
            return {"success": False, "message": "캘리브레이션 미완료", "data": {}}
        assert self._calib.intrinsic is not None
        assert self._calib.hand_eye is not None

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

    # ─── Service: Grounding DINO + depth median ─────────────

    def _handle_grounded_detect(self, req: dict) -> dict:
        data = req.get("data", {}) or {}
        prompt = str(data.get("prompt", "")).strip()
        if not prompt:
            return {"success": False, "message": "prompt 필요", "data": {}}

        if not self._calib.is_ready():
            return {"success": False, "message": "캘리브레이션 미완료", "data": {}}
        assert self._calib.hand_eye is not None

        # ── depth stream 확보 (on-demand) ────────────────
        need_enable = True
        with self._depth_lock:
            f = self._latest_depth_frame
            if f is not None and (time.time() - f.timestamp) < DEPTH_FRESH_THRESHOLD:
                need_enable = False

        if need_enable:
            res = self.call_service(
                Service.CAMERA_SET_DEPTH_STREAM,
                CameraSetDepthStreamReq(enabled=True),
                CameraSetDepthStreamRes,
            )
            if not res.success:
                return {
                    "success": False,
                    "message": f"depth enable 실패: {res.message}",
                    "data": {},
                }
            # 새 frame 한 장 기다림
            deadline = time.time() + DEPTH_WAIT_TIMEOUT
            while time.time() < deadline:
                with self._depth_lock:
                    f = self._latest_depth_frame
                    if f is not None and (time.time() - f.timestamp) < 0.5:
                        break
                time.sleep(0.05)

        with self._depth_lock:
            depth_frame = self._latest_depth_frame
        if depth_frame is None:
            return {
                "success": False,
                "message": "depth frame 없음 (카메라 노드/스트림 확인)",
                "data": {},
            }

        # ── Grounding DINO inference ─────────────────────
        try:
            det = self._grounded.detect(depth_frame.color_bgr, prompt)
        except Exception as e:
            logger.exception("Grounding DINO inference 실패")
            return {"success": False, "message": f"detection 실패: {e}", "data": {}}

        if det is None:
            return {
                "success": False,
                "message": f"'{prompt}' 감지 실패 (threshold 미달)",
                "data": {},
            }

        (x1, y1, x2, y2), score = det

        # ── bbox 영역 depth median (Z_cam) ──────────────
        h, w = depth_frame.depth_z16.shape
        ix1 = max(0, int(round(x1)))
        iy1 = max(0, int(round(y1)))
        ix2 = min(w, int(round(x2)))
        iy2 = min(h, int(round(y2)))
        if ix2 <= ix1 or iy2 <= iy1:
            return {"success": False, "message": "bbox 무효", "data": {}}

        roi = depth_frame.depth_z16[iy1:iy2, ix1:ix2]
        valid = roi[roi > 0]
        if valid.size == 0:
            return {
                "success": False,
                "message": "bbox 영역에 valid depth 없음",
                "data": {},
            }

        # 객체 윗면 z (카메라에 가장 가까운 percentile 25)
        top_raw = float(np.percentile(valid, 25))
        Z_cam = top_raw * depth_frame.depth_scale  # m, 객체 윗면

        # ── unproject (bbox 중심) ───────────────────────
        u = (x1 + x2) / 2.0
        v = (y1 + y2) / 2.0
        X_cam = (u - depth_frame.cx) / depth_frame.fx * Z_cam
        Y_cam = (v - depth_frame.cy) / depth_frame.fy * Z_cam
        obj_in_cam = np.array([X_cam, Y_cam, Z_cam])

        # ── TCP pose ────────────────────────────────────
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

        R_be = _quat_to_rot(quat)
        t_be = np.array(pos)

        # ── hand_eye → base 좌표 ────────────────────────
        R_ce = self._calib.hand_eye.R
        t_ce = self._calib.hand_eye.t.flatten()

        obj_in_ee = R_ce @ obj_in_cam + t_ce
        obj_in_base = R_be @ obj_in_ee + t_be

        # ── 책상 base_z + height 추정 ────────────────────
        # bbox 외곽 ring 의 모든 valid 픽셀을 base 프레임으로 unproject 후 z 통계.
        # camera 광축 방향 depth 차이가 아닌 base.z 평면을 직접 추정 → 카메라가
        # 책상에 수직이 아니거나 객체가 화면 가장자리에 있어도 편향되지 않음.
        # PAD 는 bbox 크기 비례 — 멀어 bbox 가 작아져도 ring 이 객체 가까운 책상
        # 영역을 잡게 함 (고정 PAD=30 이면 멀리서 책상 영역이 더 멀어져 height 과대 추정).
        bbox_w = ix2 - ix1
        bbox_h = iy2 - iy1
        pad = max(15, min(80, int(min(bbox_w, bbox_h) * 0.5)))

        ex1 = max(0, ix1 - pad)
        ey1 = max(0, iy1 - pad)
        ex2 = min(w, ix2 + pad)
        ey2 = min(h, iy2 + pad)
        ext_roi = depth_frame.depth_z16[ey1:ey2, ex1:ex2].copy()
        # bbox 내부 mask out (객체 픽셀 제거)
        ext_roi[(iy1 - ey1):(iy2 - ey1), (ix1 - ex1):(ix2 - ex1)] = 0

        vs_local, us_local = np.nonzero(ext_roi)
        if us_local.size > 0:
            us_global = us_local.astype(np.float64) + ex1
            vs_global = vs_local.astype(np.float64) + ey1
            raws = ext_roi[vs_local, us_local].astype(np.float64)
            Z_cam_ring = raws * depth_frame.depth_scale
            X_cam_ring = (us_global - depth_frame.cx) / depth_frame.fx * Z_cam_ring
            Y_cam_ring = (vs_global - depth_frame.cy) / depth_frame.fy * Z_cam_ring
            pts_cam_ring = np.stack(
                [X_cam_ring, Y_cam_ring, Z_cam_ring], axis=1
            )  # Nx3
            pts_ee_ring = pts_cam_ring @ R_ce.T + t_ce
            pts_base_ring = pts_ee_ring @ R_be.T + t_be

            # ring 에 객체 가장자리가 섞이면 base z 가 위쪽으로 튐.
            # percentile 25 = 진짜 책상 픽셀 쪽에 편향 (lower z = floor).
            floor_z = float(np.percentile(pts_base_ring[:, 2], 25))
            height = max(0.0, float(obj_in_base[2]) - floor_z)
        else:
            floor_z = float(obj_in_base[2])
            height = 0.0

        base_z = floor_z

        logger.info(
            "grounded_detect '%s' score=%.3f bbox=(%.0f,%.0f,%.0f,%.0f) "
            "Z_cam=%.3fm base=(%.3f, %.3f, %.3f) floor_z=%.3f h=%.3f pad=%d",
            prompt, score, x1, y1, x2, y2, Z_cam,
            *obj_in_base, floor_z, height, pad,
        )

        result_payload = {
            "prompt": prompt,
            "position": obj_in_base.tolist(),  # 객체 윗면 base xyz
            "bbox2d": {
                "x1": float(x1),
                "y1": float(y1),
                "x2": float(x2),
                "y2": float(y2),
            },
            "confidence": score,
            "base_z": base_z,
            "height": height,
            "timestamp": time.time() * 1000.0,  # ms (frontend Date.now() 호환)
        }

        # 호출자 무관하게 카메라 feed / 3D 마커에 자동 표시되도록 토픽 broadcast.
        # (self-play / pick_and_place GroundedDetectStep 등 backend 호출도 시각화됨)
        try:
            self.publish(Topic.PERCEPTION_GROUNDED_STATE, result_payload)
        except Exception as exc:
            logger.warning("grounded_state publish 실패: %s", exc)

        return {"success": True, "message": "ok", "data": result_payload}


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
