import logging
import threading
import time
import cv2
import numpy as np
from pathlib import Path

from core.base_node import BaseNode
from core.topic_map import Service, Topic
from core.frame_cache import FrameCache
from core.joint_state_cache import JointStateCache
from core.common import GRIPPER_ID
from modules.dynamixel.motor_config import load_motor_config
from modules.camera.stream import frame_to_base64
from modules.calibration.intrinsic import (
    CHECKERBOARD,
    IntrinsicCalibration,
)
from modules.calibration.hand_eye import HandEyeCalibration, Pose
from modules.calibration.pose_estimator import PoseEstimator
from modules.kinematics.solver import PybulletSolver

logger = logging.getLogger(__name__)

SAVE_DIR = Path(__file__).parents[2] / "robot" / "calibration"

PREVIEW_INTERVAL = 0.2  # 5Hz


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
        self.create_service(Service.CALIB_HANDEYE_CAPTURE, self._srv_handeye_capture)
        self.create_service(Service.CALIB_HANDEYE_RESET, self._srv_handeye_reset)
        self.create_service(Service.CALIB_HANDEYE_COMPUTE, self._srv_handeye_compute)
        self.create_service(Service.CALIB_HANDEYE_COMMIT, self._srv_handeye_commit)
        self.create_service(
            Service.CALIB_HANDEYE_REMOVE_POSE, self._srv_handeye_remove_pose
        )
        self.create_service(
            Service.CALIB_HANDEYE_LIST_POSES, self._srv_handeye_list_poses
        )
        self.create_service(
            Service.CALIB_HANDEYE_VALIDATE, self._srv_handeye_validate
        )
        self.create_service(
            Service.CALIB_HANDEYE_PREVIEW_ENABLE, self._srv_handeye_preview_enable
        )

        # 마지막 COMPUTE 결과 (COMMIT 시 사용)
        self._last_compute: dict | None = None

        # 라이브 미리보기 (체커보드 검출 피드백)
        self._preview_enabled = False
        self._preview_thread: threading.Thread | None = None

    def start(self) -> None:
        super().start()
        self._preview_thread = threading.Thread(
            target=self._preview_loop,
            daemon=True,
            name="calib-preview",
        )
        self._preview_thread.start()

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

    def _srv_handeye_capture(self, req: dict) -> dict:
        """한 자세에서 프레임 캡처 + 체커보드 검출 + 포즈 추가."""
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
                joint_angles_rad=list(joint_angles),
            )
        )
        # 새 포즈 추가 시 마지막 COMPUTE 결과는 무효화
        self._last_compute = None

        return {
            "success": True,
            "message": f"포즈 기록됨 ({len(self.hand_eye.poses)}개)",
            "data": {
                "detected": True,
                "pose_count": len(self.hand_eye.poses),
            },
        }

    def _srv_handeye_reset(self, req: dict) -> dict:
        self.hand_eye.reset()
        self._last_compute = None
        return {
            "success": True,
            "message": "Hand-Eye 누적 포즈 초기화됨",
            "data": {"pose_count": 0},
        }

    def _srv_handeye_remove_pose(self, req: dict) -> dict:
        index = int(req.get("data", {}).get("index", -1))
        ok = self.hand_eye.remove_pose(index)
        if not ok:
            return {
                "success": False,
                "message": f"포즈 #{index} 제거 실패 (범위 밖)",
                "data": {"pose_count": len(self.hand_eye.poses)},
            }
        self._last_compute = None
        return {
            "success": True,
            "message": f"포즈 #{index} 제거됨",
            "data": {"pose_count": len(self.hand_eye.poses)},
        }

    def _srv_handeye_list_poses(self, req: dict) -> dict:
        return {
            "success": True,
            "message": "ok",
            "data": {
                "poses": self.hand_eye.list_poses_meta(),
                "pose_count": len(self.hand_eye.poses),
            },
        }

    def _srv_handeye_compute(self, req: dict) -> dict:
        """calibrate() 실행 + method 비교 + 잔차 반환. 파일 저장 X."""
        diag = self.hand_eye.compute_with_diagnostics()
        if diag is None:
            return {
                "success": False,
                "message": f"Hand-Eye 실패 (포즈 수: {len(self.hand_eye.poses)})",
                "data": {},
            }
        self._last_compute = diag
        return {
            "success": True,
            "message": f"compute 완료 (poses={diag['pose_count']})",
            "data": diag,
        }

    def _srv_handeye_validate(self, req: dict) -> dict:
        """누적된 포즈로 주어진 hand-eye의 흩어짐(σ_rot, σ_t) 검증.

        source:
          - "saved": robot/calibration/hand_eye.npz 로드해서 검증
          - "compute": 마지막 COMPUTE 결과(in-memory)로 검증
          - "custom": data.R_cam2gripper / t_cam2gripper 직접 받음 (BA 결과 검증 등)
        """
        data = req.get("data", {})
        source = str(data.get("source", "saved"))

        R: np.ndarray | None = None
        t: np.ndarray | None = None
        source_label = source

        if source == "saved":
            path = SAVE_DIR / "hand_eye.npz"
            if not path.exists():
                return {
                    "success": False,
                    "message": f"저장된 hand_eye.npz 없음: {path}",
                    "data": {},
                }
            loaded = np.load(str(path), allow_pickle=True)
            R = np.asarray(loaded["R_cam2gripper"])
            t = np.asarray(loaded["t_cam2gripper"]).reshape(3)
            source_label = f"saved ({str(loaded.get('method', 'UNKNOWN'))})"
        elif source == "compute":
            if self.hand_eye.result is None:
                return {
                    "success": False,
                    "message": "COMPUTE 결과 없음 — 먼저 COMPUTE 실행",
                    "data": {},
                }
            R = self.hand_eye.result.R_cam2gripper
            t = self.hand_eye.result.t_cam2gripper.reshape(3)
            source_label = f"compute ({self.hand_eye.result.method})"
        elif source == "custom":
            try:
                R = np.asarray(data["R_cam2gripper"], dtype=np.float64)
                t = np.asarray(data["t_cam2gripper"], dtype=np.float64).reshape(3)
            except (KeyError, ValueError) as e:
                return {
                    "success": False,
                    "message": f"custom source는 R_cam2gripper/t_cam2gripper 필요: {e}",
                    "data": {},
                }
        else:
            return {
                "success": False,
                "message": f"알 수 없는 source: {source}",
                "data": {},
            }

        result = self.hand_eye.validate(R, t)
        if result is None:
            return {
                "success": False,
                "message": f"검증 불가 (포즈 수: {len(self.hand_eye.poses)}, 최소 2 필요)",
                "data": {},
            }

        return {
            "success": True,
            "message": f"validate 완료 (source={source_label})",
            "data": {
                **result,
                "source": source_label,
            },
        }

    def _srv_handeye_commit(self, req: dict) -> dict:
        """마지막 COMPUTE 결과를 hand_eye.npz에 저장."""
        if self._last_compute is None or self.hand_eye.result is None:
            return {
                "success": False,
                "message": "먼저 COMPUTE를 실행하세요",
                "data": {},
            }
        path = SAVE_DIR / "hand_eye.npz"
        self.hand_eye.save(path)
        return {
            "success": True,
            "message": f"저장 완료: {path}",
            "data": {
                "path": str(path),
                "method": self.hand_eye.result.method,
            },
        }

    def _srv_handeye_preview_enable(self, req: dict) -> dict:
        enabled = bool(req.get("data", {}).get("enabled", False))
        self._preview_enabled = enabled
        return {
            "success": True,
            "message": f"preview {'enabled' if enabled else 'disabled'}",
            "data": {"enabled": enabled},
        }

    def _preview_loop(self) -> None:
        """5Hz로 체커보드 검출을 시도하고 결과를 발행. enable 시에만 동작."""
        flags = cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_NORMALIZE_IMAGE
        while self._running:
            if not self._preview_enabled:
                time.sleep(PREVIEW_INTERVAL)
                continue

            try:
                ret, frame = self._frame_cache.get_frame()
                if not ret or frame is None:
                    self.publish(
                        Topic.CALIB_HANDEYE_PREVIEW,
                        {
                            "timestamp": time.time(),
                            "detected": False,
                            "reason": "no_frame",
                        },
                    )
                    time.sleep(PREVIEW_INTERVAL)
                    continue

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                h, w = gray.shape[:2]
                found, corners = cv2.findChessboardCorners(
                    gray, CHECKERBOARD, flags=flags
                )

                payload: dict = {
                    "timestamp": time.time(),
                    "detected": bool(found),
                    "image_size": [int(w), int(h)],
                }

                if found and corners is not None:
                    pts = corners.reshape(-1, 2)
                    payload["corners"] = pts.tolist()
                    xs, ys = pts[:, 0], pts[:, 1]
                    bbox_w = float(xs.max() - xs.min())
                    bbox_h = float(ys.max() - ys.min())
                    payload["bbox"] = [
                        float(xs.min()),
                        float(ys.min()),
                        bbox_w,
                        bbox_h,
                    ]
                    payload["coverage_ratio"] = (bbox_w * bbox_h) / float(w * h)

                self.publish(Topic.CALIB_HANDEYE_PREVIEW, payload)
            except Exception as e:
                logger.debug("preview loop 오류: %s", e)

            time.sleep(PREVIEW_INTERVAL)

