import logging
import numpy as np
from pathlib import Path

from core.base_node import BaseNode
from core.topic_map import Service
from core.frame_cache import FrameCache
from core.joint_state_cache import JointStateCache
from core.common import GRIPPER_ID
from modules.dynamixel.motor_config import load_motor_config
from modules.camera.stream import frame_to_base64
from modules.calibration.intrinsic import IntrinsicCalibration
from modules.calibration.hand_eye import HandEyeCalibration, Pose
from modules.calibration.pose_estimator import PoseEstimator
from modules.kinematics.solver import PybulletSolver

logger = logging.getLogger(__name__)

SAVE_DIR = Path(__file__).parents[2] / "robot" / "calibration"


class CalibrationNode(BaseNode):
    def __init__(self) -> None:
        super().__init__("calibration_node")

        self._frame_cache = FrameCache()
        self.intrinsic = IntrinsicCalibration()
        self.hand_eye = HandEyeCalibration()
        self.pose_estimator = PoseEstimator()
        self.solver = PybulletSolver()

        _, motor_cfgs = load_motor_config()
        self._arm_cfgs = [m for m in motor_cfgs if m.id != GRIPPER_ID]
        self._cache = JointStateCache()
        self._cache.subscribe(self)
        self._frame_cache.subscribe(self)

        path = SAVE_DIR / "intrinsic.npz"
        loaded = self.intrinsic.load(path)

        if loaded:
            logger.info(f"Intrinsic 로드 완료: {path}")
        else:
            logger.warning("Intrinsic 파일 없음")

        # 내부 캘리브레이션
        self.create_service(Service.CALIB_CAPTURE, self._srv_capture)
        self.create_service(Service.CALIB_INTRINSIC_START, self._srv_intrinsic_start)
        self.create_service(Service.CALIB_INTRINSIC_SAVE, self._srv_intrinsic_save)

        # Hand-Eye 캘리브레이션
        self.create_service(Service.CALIB_HANDEYE_START, self._srv_handeye_start)
        self.create_service(Service.CALIB_HANDEYE_SAVE, self._srv_handeye_save)

    # ─── 이미지 캡처 ─────────────────────────────────────────

    def _srv_capture(self, req: dict) -> dict:
        mode = req.get("data", {}).get("mode", "intrinsic")

        ret, frame = self._frame_cache.get_frame()
        if not ret or frame is None:
            return {
                "success": False,
                "message": "카메라 프레임을 읽을 수 없습니다",
                "data": {},
            }

        if mode == "intrinsic":
            detected, vis = self.intrinsic.capture(frame)
            b64 = frame_to_base64(vis)
            return {
                "success": True,
                "message": "체커보드 감지됨" if detected else "체커보드 미감지",
                "data": {
                    "detected": detected,
                    "captured_count": len(self.intrinsic.obj_points),
                    "preview": b64,
                },
            }

        return {"success": False, "message": f"알 수 없는 mode: {mode}", "data": {}}

    # ─── 내부 캘리브레이션 ────────────────────────────────────

    def _srv_intrinsic_start(self, req: dict) -> dict:
        self.intrinsic.reset()
        return {"success": True, "message": "내부 캘리브레이션 초기화됨", "data": {}}

    def _srv_intrinsic_save(self, req: dict) -> dict:
        width = self._frame_cache.width
        height = self._frame_cache.height
        if width is None or height is None:
            return {
                "success": False,
                "message": "카메라 status(width/height) 미수신",
                "data": {},
            }
        image_size = (width, height)
        result = self.intrinsic.calibrate(image_size)

        if result is None:
            return {
                "success": False,
                "message": f"캘리브레이션 실패 (캡처 수: {len(self.intrinsic.obj_points)})",
                "data": {},
            }

        path = SAVE_DIR / "intrinsic.npz"
        self.intrinsic.save(path)

        return {
            "success": True,
            "message": f"저장 완료: {path}",
            "data": {
                "rms_error": result.rms_error,
                "camera_matrix": result.camera_matrix.tolist(),
                "dist_coeffs": result.dist_coeffs.tolist(),
                "captured_count": result.captured_count,
            },
        }

    # ─── Hand-Eye 캘리브레이션 ────────────────────────────────

    def _srv_handeye_start(self, req: dict) -> dict:
        if self.intrinsic.result is None:
            return {
                "success": False,
                "message": "내부 캘리브레이션 결과가 필요합니다",
                "data": {},
            }

        # FK로 gripper R, t 계산
        joint_angles = self._cache.get_joint_angles_rad(self._arm_cfgs)
        if joint_angles is None:
            return {
                "success": False,
                "message": "관절 상태 수신 전",
                "data": {},
            }

        R_list, t_list = self.solver.fk_to_matrix(joint_angles)
        gripper_R = np.array(R_list)
        gripper_t = np.array(t_list).reshape(3, 1)

        # 카메라 캡처 + 체커보드 검출
        ret, frame = self._frame_cache.get_frame()
        if not ret or frame is None:
            return {"success": False, "message": "카메라 프레임 읽기 실패", "data": {}}

        detected, _ = self.intrinsic.capture(frame)
        if not detected:
            return {
                "success": False,
                "message": "체커보드 미감지",
                "data": {"detected": False, "pose_count": len(self.hand_eye.poses)},
            }

        pose = self.pose_estimator.estimate(
            obj_points=self.intrinsic.obj_points[-1],
            img_points=self.intrinsic.img_points[-1],
            camera_matrix=self.intrinsic.result.camera_matrix,
            dist_coeffs=self.intrinsic.result.dist_coeffs,
        )
        if pose is None:
            return {"success": False, "message": "포즈 추정 실패", "data": {}}

        self.hand_eye.add_pose(
            Pose(
                R_gripper2base=gripper_R,
                t_gripper2base=gripper_t,
                R_target2cam=pose.R,
                t_target2cam=pose.t,
            )
        )

        return {
            "success": True,
            "message": f"포즈 기록됨 ({len(self.hand_eye.poses)}개)",
            "data": {
                "detected": True,
                "pose_count": len(self.hand_eye.poses),
            },
        }

    def _srv_handeye_save(self, req: dict) -> dict:
        result = self.hand_eye.calibrate()
        if result is None:
            return {
                "success": False,
                "message": f"Hand-Eye 실패 (포즈 수: {len(self.hand_eye.poses)})",
                "data": {},
            }

        path = SAVE_DIR / "hand_eye.npz"
        self.hand_eye.save(path)

        return {
            "success": True,
            "message": f"저장 완료: {path}",
            "data": {
                "R_cam2gripper": result.R_cam2gripper.tolist(),
                "t_cam2gripper": result.t_cam2gripper.tolist(),
                "method": result.method,
            },
        }
