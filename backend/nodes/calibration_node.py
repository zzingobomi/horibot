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
from modules.calibration.intrinsic import CHECKERBOARD, IntrinsicCalibration
from modules.calibration.hand_eye import HandEyeCalibration, Pose
from modules.calibration import joint_offsets as joint_offsets_io
from modules.calibration import thresholds as calib_thresholds
from modules.calibration.pose_estimator import PoseEstimator
from modules.kinematics.solver import PybulletSolver

logger = logging.getLogger(__name__)

SAVE_DIR = Path(__file__).parents[2] / "robot" / "calibration"
HANDEYE_POSES_PATH = SAVE_DIR / "handeye_poses.npz"

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

        self._last_compute: dict | None = None
        self._preview_enabled = False
        self._preview_thread: threading.Thread | None = None

        # 내부 캘리브레이션
        self.create_service(Service.CALIB_CAPTURE, self._srv_capture)
        self.create_service(Service.CALIB_INTRINSIC_START,
                            self._srv_intrinsic_start)
        self.create_service(Service.CALIB_INTRINSIC_SAVE,
                            self._srv_intrinsic_save)

        # Hand-Eye 캘리브레이션
        self.create_service(Service.CALIB_HANDEYE_CAPTURE,
                            self._srv_handeye_capture)
        self.create_service(Service.CALIB_HANDEYE_RESET,
                            self._srv_handeye_reset)
        self.create_service(Service.CALIB_HANDEYE_COMPUTE,
                            self._srv_handeye_compute)
        self.create_service(Service.CALIB_HANDEYE_COMMIT,
                            self._srv_handeye_commit)
        self.create_service(
            Service.CALIB_HANDEYE_LIST_POSES, self._srv_handeye_list_poses
        )
        self.create_service(
            Service.CALIB_HANDEYE_PREVIEW_ENABLE, self._srv_handeye_preview_enable
        )
        self.create_service(
            Service.CALIB_HANDEYE_THRESHOLDS, self._srv_handeye_thresholds
        )

    def start(self) -> None:
        super().start()
        self._preview_thread = threading.Thread(
            target=self._preview_loop,
            daemon=True,
            name="calib-preview",
        )
        self._preview_thread.start()
        # 시작 시 현재 적용 중인 joint offsets 1회 발행 (frontend가 mount 시 받게)
        self._publish_joint_offsets()
        # 이전 세션에서 캡처한 포즈가 디스크에 있으면 복원 (thresholds 튜닝 사이클).
        loaded = self.hand_eye.load_poses(HANDEYE_POSES_PATH)
        if loaded > 0:
            logger.info(f"이전 Hand-Eye 포즈 {loaded}개 복원됨")

    def _publish_joint_offsets(self) -> None:
        """현재 디스크에 있는 joint_offsets.npz 상태를 프론트엔드/구독자에게 브로드캐스트."""
        path = SAVE_DIR / "joint_offsets.npz"
        offsets = joint_offsets_io.load(path)
        payload = {
            "timestamp": time.time(),
            "offsets": [
                {"motor_id": int(mid), "offset_rad": float(off)}
                for mid, off in sorted(offsets.items())
            ],
        }
        self.publish(Topic.CALIB_STATE_JOINT_OFFSETS, payload)

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

        self._last_compute = None  # 새 포즈 추가 시 이전 계산 결과 무효화
        # 디스크 영구화 — threshold 튜닝/재시작 후에도 보존.
        try:
            self.hand_eye.save_poses(HANDEYE_POSES_PATH)
        except Exception as e:
            logger.warning("포즈 디스크 저장 실패 (메모리에는 남음): %s", e)

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
        # 디스크 파일도 삭제 — "처음부터 다시" 의도와 일치.
        if HANDEYE_POSES_PATH.exists():
            try:
                HANDEYE_POSES_PATH.unlink()
            except OSError as e:
                logger.warning("포즈 파일 삭제 실패: %s", e)
        return {
            "success": True,
            "message": "Hand-Eye 누적 포즈 초기화됨",
            "data": {"pose_count": 0},
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
        arm_motor_ids = [cfg.id for cfg in self._arm_cfgs]
        diag = self.hand_eye.compute_with_diagnostics(
            fk_fn=self.solver.fk_to_matrix,
            arm_motor_ids=arm_motor_ids,
        )
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

    def _srv_handeye_commit(self, req: dict) -> dict:
        if self._last_compute is None or self.hand_eye.result is None:
            return {
                "success": False,
                "message": "먼저 COMPUTE를 실행하세요",
                "data": {},
            }

        # 1) hand_eye.npz — 카메라↔그리퍼 외부 보정
        hand_eye_path = SAVE_DIR / "hand_eye.npz"
        self.hand_eye.save(hand_eye_path)

        # 2) joint_offsets.npz — BA가 추정한 delta offset을 기존 값에 cumulative 합산.
        offset_msg = ""
        if self._last_compute.get("joint_offset_estimated"):
            delta_list = self._last_compute.get("joint_offset_delta", [])
            delta_by_id = {
                int(e["motor_id"]): float(e["offset_rad"]) for e in delta_list
            }
            offsets_path = SAVE_DIR / "joint_offsets.npz"
            existing = joint_offsets_io.load(offsets_path)
            merged = joint_offsets_io.merge_delta(existing, delta_by_id)
            joint_offsets_io.save(
                offsets_path,
                merged,
                method=self.hand_eye.result.method,
            )
            # 즉시 적용
            applied = self._cache.reload_joint_offsets()
            applied_deg = {i: round(float(np.degrees(v)), 3) for i, v in applied.items()}
            offset_msg = f" + joint_offsets 갱신 (cumulative, deg={applied_deg})"
            logger.info("joint_offsets 즉시 적용: %s", applied_deg)
            # 프론트엔드 URDF가 즉시 보정되도록 브로드캐스트
            self._publish_joint_offsets()

        return {
            "success": True,
            "message": f"저장 완료: {hand_eye_path}{offset_msg}",
            "data": {
                "path": str(hand_eye_path),
                "method": self.hand_eye.result.method,
                "joint_offsets_applied": self._last_compute.get(
                    "joint_offset_estimated", False
                ),
            },
        }

    def _srv_handeye_thresholds(self, req: dict) -> dict:
        """프론트엔드가 mount 시 1회 fetch. 단일 출처 보장."""
        return {
            "success": True,
            "message": "ok",
            "data": calib_thresholds.as_dict(),
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
        # SB는 조명/블러에 강함. preview는 속도 우선이라 EXHAUSTIVE/ACCURACY 미사용.
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE
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
                found, corners = cv2.findChessboardCornersSB(
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
                    payload["coverage_ratio"] = (
                        bbox_w * bbox_h) / float(w * h)

                    # tilt: 보드 평면과 카메라 이미지 평면 사이 각도.
                    # R_target2cam의 board Z축이 카메라 Z축과 얼마나 평행한가로 측정.
                    if self.intrinsic.result is not None:
                        try:
                            ok, rvec, _tvec = cv2.solvePnP(
                                self.intrinsic._objp_template,
                                corners,
                                self.intrinsic.result.camera_matrix,
                                self.intrinsic.result.dist_coeffs,
                                flags=cv2.SOLVEPNP_ITERATIVE,
                            )
                            R, _ = cv2.Rodrigues(rvec)
                            # R[2,2] = board Z축의 카메라 Z성분.
                            # |R[2,2]|=1 → 보드 평면이 이미지 평면과 평행 → tilt 0°
                            cos_v = float(np.clip(abs(R[2, 2]), 0.0, 1.0))
                            payload["tilt_deg"] = float(
                                np.degrees(np.arccos(cos_v)))
                        except cv2.error:
                            pass

                self.publish(Topic.CALIB_HANDEYE_PREVIEW, payload)
            except Exception as e:
                logger.debug("preview loop 오류: %s", e)

            time.sleep(PREVIEW_INTERVAL)
