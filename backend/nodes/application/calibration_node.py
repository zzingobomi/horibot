import logging
import threading
import time
import cv2
import numpy as np
from dataclasses import dataclass, field
from pathlib import Path

from core.transport.application_node import ApplicationNode
from core.coords.joint_coordinates import JointCoordinates
from core.coords.link_coordinates import LinkCoordinates
from core.robot.robot_registry import RobotRegistry
from core.coords.sag_coordinates import SagCoordinates
from core.coords.tool_coordinates import ToolCoordinates
from modules.calibration.sag_offsets import SagOffsets
from core.transport.messages.base import EmptyData, ServiceRequest, ServiceResponse
from core.transport.messages.calibration import (
    BackupEntry,
    BackupListRes,
    BackupRestoreReq,
    BackupRestoreRes,
    CalibCaptureReq,
    CalibCaptureRes,
    HandeyeCaptureRes,
    HandeyeCommitRes,
    HandeyeListPosesRes,
    HandeyePoseMeta,
    HandeyePreviewEnableReq,
    HandeyePreviewEnableRes,
    HandeyeResetRes,
    IntrinsicSaveRes,
    JointOffsetEntry,
    LinkOffsetEntry,
    MultiStartReq,
    MultiStartRes,
    RecommendationFailReq,
    RecommendationFailRes,
    SagOffsetEntry,
)
from core.transport.topic_map import Service, Topic, topic_for
from core.cache.frame_cache import FrameCache
from core.cache.joint_state_cache import JointStateCache
from modules.motor.motor_config import MotorConfig, load_motor_layout
from modules.camera.stream import frame_to_base64
from modules.calibration.intrinsic import IntrinsicCalibration
from modules.calibration.hand_eye import HandEyeCalibration, Pose
from modules.calibration import backup as calib_backup
from modules.calibration import board as calib_board
from modules.calibration import next_pose_planner
from modules.calibration import thresholds as calib_thresholds
from modules.calibration.link_offsets import LinkOffsets
from modules.kinematics.corrected import CorrectedIKSolver

logger = logging.getLogger(__name__)


def _save_dir(robot_id: str) -> Path:
    return RobotRegistry().get(robot_id).calibration_dir


def _handeye_poses_path(robot_id: str) -> Path:
    return _save_dir(robot_id) / "handeye_poses.npz"


PREVIEW_INTERVAL = 0.2  # 5Hz


def _optional_float(v: object) -> float | None:
    return float(v) if isinstance(v, (int, float)) else None


def _optional_int(v: object) -> int | None:
    return int(v) if isinstance(v, (int, float)) else None


def _optional_str(v: object) -> str | None:
    return str(v) if v is not None else None


@dataclass
class _RobotState:
    """robot 별 캘리브레이션 상태."""

    arm_cfgs: list[MotorConfig]
    intrinsic: IntrinsicCalibration
    hand_eye: HandEyeCalibration
    solver: CorrectedIKSolver
    last_compute: dict | None = None
    preview_enabled: bool = False
    # 사용자 명시 신호 ([👎 안 보임] / [👎 빨강] / [👎 도달 실패]) 로 fail 표시한
    # 추천 ID set. 다음 추천 생성 시 제외. [수동 모드 종료] / 라운드 reset 시 초기화.
    recommendation_fail_ids: set[str] = field(default_factory=set)
    # Saturate 인지 — σ 변화율 추적 (최근 N capture 동안 변화 거의 0 → saturate).
    sigma_history: list[float] = field(default_factory=list)


class CalibrationNode(ApplicationNode):
    """Application 노드 — robot 무관 한 인스턴스. robot 별 dict[robot_id] state."""

    def __init__(self) -> None:
        super().__init__("calibration_node")

        self._frame_cache = FrameCache()
        self._cache = JointStateCache()

        # robot 별 상태
        self._states: dict[str, _RobotState] = {}
        for rid in self.enabled_robot_ids:
            arm_cfgs = load_motor_layout(rid).arm
            intrinsic = IntrinsicCalibration()
            hand_eye = HandEyeCalibration()
            solver = self._registry.get_iksolver(rid)
            assert isinstance(solver, CorrectedIKSolver)

            path = _save_dir(rid) / "intrinsic.npz"
            loaded = intrinsic.load(path)
            if loaded:
                logger.info("[%s] Intrinsic 로드 완료: %s", rid, path)
            else:
                logger.warning("[%s] Intrinsic 파일 없음", rid)

            self._states[rid] = _RobotState(
                arm_cfgs=arm_cfgs,
                intrinsic=intrinsic,
                hand_eye=hand_eye,
                solver=solver,
            )

        self._preview_thread: threading.Thread | None = None

    def start(self) -> None:
        for rid in self.enabled_robot_ids:
            self._frame_cache.subscribe(self, robot_id=rid)
            # 내부 캘리브레이션
            self.create_service(
                topic_for(Service.CALIB_CAPTURE, rid),
                CalibCaptureReq,
                CalibCaptureRes,
                lambda req, _rid=rid: self._srv_capture(req, _rid),
            )
            self.create_service(
                topic_for(Service.CALIB_INTRINSIC_START, rid),
                EmptyData,
                EmptyData,
                lambda req, _rid=rid: self._srv_intrinsic_start(req, _rid),
            )
            self.create_service(
                topic_for(Service.CALIB_INTRINSIC_SAVE, rid),
                EmptyData,
                IntrinsicSaveRes,
                lambda req, _rid=rid: self._srv_intrinsic_save(req, _rid),
            )
            # Hand-Eye 캘리브레이션
            self.create_service(
                topic_for(Service.CALIB_HANDEYE_CAPTURE, rid),
                EmptyData,
                HandeyeCaptureRes,
                lambda req, _rid=rid: self._srv_handeye_capture(req, _rid),
            )
            self.create_service(
                topic_for(Service.CALIB_HANDEYE_RESET, rid),
                EmptyData,
                HandeyeResetRes,
                lambda req, _rid=rid: self._srv_handeye_reset(req, _rid),
            )
            # legacy dict — typed 면제 (free-form). robot_id closure.
            self.create_service(
                topic_for(Service.CALIB_HANDEYE_COMPUTE, rid),
                lambda req, _rid=rid: self._srv_handeye_compute(req, _rid),
            )
            self.create_service(
                topic_for(Service.CALIB_HANDEYE_COMMIT, rid),
                EmptyData,
                HandeyeCommitRes,
                lambda req, _rid=rid: self._srv_handeye_commit(req, _rid),
            )
            self.create_service(
                topic_for(Service.CALIB_HANDEYE_LIST_POSES, rid),
                EmptyData,
                HandeyeListPosesRes,
                lambda req, _rid=rid: self._srv_handeye_list_poses(req, _rid),
            )
            self.create_service(
                topic_for(Service.CALIB_HANDEYE_PREVIEW_ENABLE, rid),
                HandeyePreviewEnableReq,
                HandeyePreviewEnableRes,
                lambda req, _rid=rid: self._srv_handeye_preview_enable(req, _rid),
            )
            self.create_service(
                topic_for(Service.CALIB_HANDEYE_THRESHOLDS, rid),
                lambda req, _rid=rid: self._srv_handeye_thresholds(req, _rid),
            )
            self.create_service(
                topic_for(Service.CALIB_HANDEYE_RECOMMENDATION_FAIL, rid),
                RecommendationFailReq,
                RecommendationFailRes,
                lambda req, _rid=rid: self._srv_handeye_recommendation_fail(
                    req, _rid
                ),
            )
            self.create_service(
                topic_for(Service.CALIB_HANDEYE_MULTI_START, rid),
                MultiStartReq,
                MultiStartRes,
                lambda req, _rid=rid: self._srv_handeye_multi_start(req, _rid),
            )
            self.create_service(
                topic_for(Service.CALIB_BACKUP_LIST, rid),
                EmptyData,
                BackupListRes,
                lambda req, _rid=rid: self._srv_backup_list(req, _rid),
            )
            self.create_service(
                topic_for(Service.CALIB_BACKUP_RESTORE, rid),
                BackupRestoreReq,
                BackupRestoreRes,
                lambda req, _rid=rid: self._srv_backup_restore(req, _rid),
            )

        super().start()
        self._cache.subscribe(self)
        self._preview_thread = threading.Thread(
            target=self._preview_loop,
            daemon=True,
            name="calib-preview",
        )
        self._preview_thread.start()

        # 이전 hand-eye poses 복원
        for rid, st in self._states.items():
            loaded = st.hand_eye.load_poses(_handeye_poses_path(rid))
            if loaded > 0:
                logger.info("[%s] 이전 Hand-Eye 포즈 %d개 복원됨", rid, loaded)

        logger.info(
            "CalibrationNode 시작 (robots=%s)", self.enabled_robot_ids
        )

    # ─── 이미지 캡처 ─────────────────────────────────────────

    def _srv_capture(
        self, req: ServiceRequest[CalibCaptureReq], robot_id: str
    ) -> ServiceResponse[CalibCaptureRes]:
        st = self._states[robot_id]
        mode = req.data.mode

        ret, frame = self._frame_cache.get_frame(robot_id=robot_id)
        if not ret or frame is None:
            return ServiceResponse(
                success=False,
                message="카메라 프레임을 읽을 수 없습니다",
                data=None,
            )

        if mode == "intrinsic":
            width = self._frame_cache.width(robot_id=robot_id)
            height = self._frame_cache.height(robot_id=robot_id)
            image_size = (
                (int(width), int(height))
                if width is not None and height is not None
                else None
            )
            detected, vis, hint = st.intrinsic.capture(frame, image_size)
            b64 = frame_to_base64(vis)
            return ServiceResponse(
                success=True,
                message=hint,
                data=CalibCaptureRes(
                    detected=detected,
                    captured_count=len(st.intrinsic.obj_points),
                    preview=b64,
                    hint=hint,
                    coverage_count=len(st.intrinsic.coverage_cells),
                ),
            )

        return ServiceResponse(
            success=False, message=f"알 수 없는 mode: {mode}", data=None
        )

    # ─── 내부 캘리브레이션 ────────────────────────────────────

    def _srv_intrinsic_start(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[EmptyData]:
        self._states[robot_id].intrinsic.reset()
        return ServiceResponse(
            success=True, message="내부 캘리브레이션 초기화됨", data=EmptyData()
        )

    def _srv_intrinsic_save(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[IntrinsicSaveRes]:
        st = self._states[robot_id]
        width = self._frame_cache.width(robot_id=robot_id)
        height = self._frame_cache.height(robot_id=robot_id)
        if width is None or height is None:
            return ServiceResponse(
                success=False,
                message="카메라 status(width/height) 미수신",
                data=None,
            )
        image_size = (width, height)
        result = st.intrinsic.calibrate(image_size)

        if result is None:
            return ServiceResponse(
                success=False,
                message=f"캘리브레이션 실패 (캡처 수: {len(st.intrinsic.obj_points)})",
                data=None,
            )

        path = _save_dir(robot_id) / "intrinsic.npz"
        st.intrinsic.save(path)

        return ServiceResponse(
            success=True,
            message=f"저장 완료: {path}",
            data=IntrinsicSaveRes(
                rms_error=result.rms_error,
                camera_matrix=result.camera_matrix.tolist(),
                dist_coeffs=result.dist_coeffs.tolist(),
                captured_count=result.captured_count,
                coverage_count=len(result.coverage_cells),
                coverage_cells=[[gx, gy] for gx, gy in result.coverage_cells],
            ),
        )

    # ─── Hand-Eye 캘리브레이션 ────────────────────────────────

    # ─── BA 실행 + 상태 stash (수동 COMPUTE / 자동 BA 공통) ───────
    def _run_ba_and_stash(
        self, robot_id: str, mode: str = "physical_sag"
    ) -> dict | None:
        """compute_with_diagnostics + joint absolute reconciliation + last_compute stash.

        수동 COMPUTE (`_srv_handeye_compute`) 와 자동 BA (`_srv_handeye_capture` 끝)
        둘 다 본 함수 호출 → 결과 일관성 + Bug A 같은 logic 분기 X.
        """
        st = self._states[robot_id]
        arm_motor_ids = [cfg.id for cfg in st.arm_cfgs]
        joint_limits = st.solver.joint_limits(len(arm_motor_ids))
        use_physical_sag = mode == "physical_sag"
        use_extended_ba = mode in ("physical_sag", "extended")
        diag = st.hand_eye.compute_with_diagnostics(
            fk_fn=st.solver.fk_to_matrix,
            arm_motor_cfgs=st.arm_cfgs,
            joint_limits_rad=joint_limits,
            use_extended_ba=use_extended_ba,
            use_physical_sag=use_physical_sag,
        )
        if diag is None:
            return None

        # joint absolute reconciliation — Bug A 의 fix 자리. 모든 BA 진입점이 본 함수 통과.
        if diag.get("joint_offset_estimated"):
            current_by_id = JointCoordinates().snapshot(robot_id=robot_id)
            absolute_by_id: dict[int, float] = {}
            for e in diag.get("joint_offset_delta", []):
                mid = int(e["motor_id"])
                delta = float(e["offset_rad"])
                absolute_by_id[mid] = current_by_id.get(mid, 0.0) + delta
            diag["_joint_absolute_by_id"] = absolute_by_id
            diag["joint_offset_absolute"] = [
                {
                    "motor_id": mid,
                    "offset_rad": v,
                    "offset_deg": float(np.degrees(v)),
                }
                for mid, v in sorted(absolute_by_id.items())
            ]

        st.last_compute = diag
        return diag

    def _publish_saturate_state(self, robot_id: str, diag: dict) -> None:
        """σ 변화율 추적 → saturate 인지 publish.

        최근 N capture 동안 σ_rot 변화 < epsilon 이면 saturate. TSDF GOOD 안이면
        "sufficient, COMMIT 권장" 명시. 밖이면 "floor 도달, escape (BA mode 변경 /
        자유 자세) 시도" 명시. 사용자가 외부 도구 자리 진입 자체 자리 막음.
        """
        st = self._states[robot_id]
        sigma_rot = diag.get("sigma_rot_deg")
        sigma_t = diag.get("sigma_t_mm")
        if sigma_rot is None or sigma_t is None:
            return
        st.sigma_history.append(float(sigma_rot))
        window = 5
        if len(st.sigma_history) > window:
            st.sigma_history = st.sigma_history[-window:]

        saturate = False
        in_good = False
        reason = ""
        if len(st.sigma_history) >= window:
            recent = st.sigma_history[-window:]
            sigma_range = max(recent) - min(recent)
            if sigma_range < 0.05:  # 0.05° 임계
                saturate = True
                in_good = (
                    float(sigma_rot) < calib_thresholds.SIGMA_ROT_GOOD_DEG
                    and float(sigma_t) < calib_thresholds.SIGMA_T_GOOD_MM
                )
                reason = (
                    "현재 σ TSDF GOOD 임계 안 — sufficient, COMMIT 권장"
                    if in_good
                    else "saturate (floor 도달) — escape 시도 (BA mode 변경 / 자유 자세 / 외부 도구)"
                )

        self.publish(
            topic_for(Topic.CALIB_HANDEYE_SATURATE, robot_id),
            {
                "timestamp": time.time(),
                "saturate": saturate,
                "in_good": in_good,
                "reason": reason,
                "sigma_history": list(st.sigma_history),
            },
        )

    def _publish_recommendations(self, robot_id: str) -> None:
        """매 capture 후 추천 자세 갱신 + publish.

        backend 자체 자리 자취 자리 = 모든 capture 마다 추천 generated. frontend 자체
        자리 자취 자리 = phase 별 hide/show.
        """
        try:
            result = self._compute_recommendations(robot_id)
        except Exception as e:
            logger.debug("[%s] 추천 계산 실패: %s", robot_id, e)
            return
        self.publish(
            topic_for(Topic.CALIB_HANDEYE_RECOMMENDATIONS, robot_id),
            {
                "timestamp": time.time(),
                "recommendations": result["recommendations"],
                "no_candidates_reason": result["no_candidates_reason"],
            },
        )

    def _publish_sigma_state(self, robot_id: str, diag: dict) -> None:
        """σ live topic. 자동 BA / 수동 COMPUTE 모두 호출 → frontend 가 즉시 표시.

        axis_distributions 같이 보내 frontend 가 자세 다양성 표 / 부족 axis 색깔
        표시. verdict 4 상태 분기와 같이 UI 가 "어느 axis 변주 캡처" 안내 가능.
        """
        self.publish(
            topic_for(Topic.CALIB_HANDEYE_SIGMA, robot_id),
            {
                "timestamp": time.time(),
                "sigma_rot_deg": diag.get("sigma_rot_deg"),
                "sigma_t_mm": diag.get("sigma_t_mm"),
                "pose_count": diag.get("pose_count", 0),
                "ba_mode": diag.get("method"),
                "ba_converged": diag.get("ba_converged", False),
                "coach_verdict": diag.get("coach", {}).get("verdict"),
                "joint_offset_estimated": diag.get(
                    "joint_offset_estimated", False
                ),
                "link_offset_estimated": diag.get(
                    "link_offset_estimated", False
                ),
                "sag_offset_estimated": diag.get(
                    "sag_offset_estimated", False
                ),
                "axis_distributions": diag.get("coach", {}).get(
                    "axis_distributions", []
                ),
            },
        )

    def _estimate_board_base_frame(
        self, robot_id: str
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """모든 capture 의 보드 pose (target2cam) → base-frame 평균.

        T_target2base = T_gripper2base · T_cam2gripper(=hand_eye) · T_target2cam
        각 capture 별 T_target2base 계산 → origin 평균 + R 평균(SVD averaging) →
        보드 4 외곽 코너 (board frame, board.py SSOT) 를 base-frame 으로 변환.

        카메라 평균 위치도 같이 반환 — recommend_geometry 의 outward_hint 결정용
        (보드 normal 부호 = 카메라가 있던 쪽). 보드 수직/기울임 setup 에서 anchor
        가 로봇 반대편 공중에 생성되는 bug 의 fix.

        Returns:
            (corners_base, avg_cam_pos_base) — corners (4, 3) + camera 평균 (3,)
            또는 None (포즈 없음 / hand_eye 없음).
        """
        st = self._states[robot_id]
        if st.hand_eye.result is None or not st.hand_eye.poses:
            return None

        R_c2g = st.hand_eye.result.R_cam2gripper
        t_c2g = np.asarray(st.hand_eye.result.t_cam2gripper).reshape(3)
        T_c2g = np.eye(4)
        T_c2g[:3, :3] = R_c2g
        T_c2g[:3, 3] = t_c2g

        coords = JointCoordinates()
        origins: list[np.ndarray] = []
        Rs: list[np.ndarray] = []
        cam_positions: list[np.ndarray] = []
        for p in st.hand_eye.poses:
            angles: list[float] = []
            for cfg in st.arm_cfgs:
                raw = p.raw_motor_positions.get(cfg.id)
                if raw is None:
                    angles = []
                    break
                angles.append(coords.motor_to_urdf(int(raw), cfg, robot_id))
            if not angles:
                continue
            R_g2b, t_g2b = st.solver.fk_to_matrix(angles)
            T_g2b = np.eye(4)
            T_g2b[:3, :3] = np.asarray(R_g2b)
            T_g2b[:3, 3] = np.asarray(t_g2b).reshape(3)
            T_t2c = np.eye(4)
            T_t2c[:3, :3] = np.asarray(p.R_target2cam)
            T_t2c[:3, 3] = np.asarray(p.t_target2cam).reshape(3)
            T_t2b = T_g2b @ T_c2g @ T_t2c
            origins.append(T_t2b[:3, 3])
            Rs.append(T_t2b[:3, :3])
            # 카메라 base-frame 위치 = T_g2b · T_c2g · 0 = T_g2b · t_c2g + t_g2b
            T_c2b = T_g2b @ T_c2g
            cam_positions.append(T_c2b[:3, 3])

        if not origins:
            return None

        avg_origin = np.mean(np.stack(origins), axis=0)
        Rsum = np.sum(np.stack(Rs), axis=0)
        U, _, Vt = np.linalg.svd(Rsum)
        avg_R = U @ Vt
        if np.linalg.det(avg_R) < 0:
            Vt[-1] *= -1
            avg_R = U @ Vt

        corners_board = calib_board.board_corner_points_3d()  # (4, 3)
        corners_base = (avg_R @ corners_board.T).T + avg_origin
        avg_cam_pos = np.mean(np.stack(cam_positions), axis=0)
        return corners_base, avg_cam_pos

    def _srv_handeye_capture(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[HandeyeCaptureRes]:
        st = self._states[robot_id]
        if st.intrinsic.result is None:
            return ServiceResponse(
                success=False,
                message="내부 캘리브레이션 결과가 필요합니다",
                data=None,
            )

        raw_positions = self._cache.get_raw_motor_positions(
            st.arm_cfgs, robot_id=robot_id
        )
        if raw_positions is None:
            return ServiceResponse(
                success=False, message="관절 상태 수신 전", data=None
            )

        ret, frame = self._frame_cache.get_frame(robot_id=robot_id)
        if not ret or frame is None:
            return ServiceResponse(
                success=False, message="카메라 프레임 읽기 실패", data=None
            )

        # ChArUco 검출 — intrinsic pool 안 건드림 (handeye pool 과 분리).
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ok, ch_corners, ch_ids = calib_board.detect(gray)
        if not ok or ch_corners is None or ch_ids is None:
            return ServiceResponse(
                success=False,
                message="ChArUco 보드 미감지",
                data=HandeyeCaptureRes(
                    detected=False, pose_count=len(st.hand_eye.poses)
                ),
            )
        obj_pts, img_pts = calib_board.match_object_points(ch_corners, ch_ids)

        ok, rvec, tvec = cv2.solvePnP(
            obj_pts,
            img_pts,
            st.intrinsic.result.camera_matrix,
            st.intrinsic.result.dist_coeffs,
        )
        if not ok:
            logger.warning("solvePnP 풀이 실패")
            return ServiceResponse(
                success=False, message="포즈 추정 실패", data=None
            )
        R_t2c, _ = cv2.Rodrigues(rvec)

        st.hand_eye.add_pose(
            Pose(
                raw_motor_positions=raw_positions,
                R_target2cam=R_t2c,
                t_target2cam=tvec,
            )
        )

        st.last_compute = None
        try:
            st.hand_eye.save_poses(_handeye_poses_path(robot_id))
        except Exception as e:
            logger.warning("[%s] 포즈 디스크 저장 실패: %s", robot_id, e)

        # 자동 BA — capture 마다 σ live + 추천 + saturate 인지 갱신. 사용자가
        # [COMPUTE] 별도로 안 눌러도 매 capture 후 즉시 publish (재캘 거부감 0).
        # 최소 capture 수 미만이면 skip. 실패는 warning 로그만.
        if len(st.hand_eye.poses) >= calib_thresholds.MIN_POSES_FOR_COMPUTE:
            try:
                auto_diag = self._run_ba_and_stash(robot_id, mode="physical_sag")
                if auto_diag is not None:
                    self._publish_sigma_state(robot_id, auto_diag)
                    self._publish_saturate_state(robot_id, auto_diag)
                    self._publish_recommendations(robot_id)
            except Exception as e:
                logger.warning("[%s] 자동 BA 실패: %s", robot_id, e)

        return ServiceResponse(
            success=True,
            message=f"포즈 기록됨 ({len(st.hand_eye.poses)}개)",
            data=HandeyeCaptureRes(
                detected=True, pose_count=len(st.hand_eye.poses)
            ),
        )

    def _srv_handeye_reset(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[HandeyeResetRes]:
        st = self._states[robot_id]
        st.hand_eye.reset()
        st.last_compute = None
        st.recommendation_fail_ids.clear()
        st.sigma_history.clear()
        poses_path = _handeye_poses_path(robot_id)
        if poses_path.exists():
            try:
                poses_path.unlink()
            except OSError as e:
                logger.warning("[%s] 포즈 파일 삭제 실패: %s", robot_id, e)
        return ServiceResponse(
            success=True,
            message="Hand-Eye 누적 포즈 초기화됨",
            data=HandeyeResetRes(pose_count=0),
        )

    def _srv_handeye_list_poses(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[HandeyeListPosesRes]:
        st = self._states[robot_id]
        poses = [
            HandeyePoseMeta.model_validate(m)
            for m in st.hand_eye.list_poses_meta(st.arm_cfgs)
        ]
        return ServiceResponse(
            success=True,
            message="ok",
            data=HandeyeListPosesRes(
                poses=poses, pose_count=len(st.hand_eye.poses)
            ),
        )

    def _srv_handeye_compute(self, req: dict, robot_id: str) -> dict:
        mode = str(req.get("mode", "physical_sag")).lower()
        diag = self._run_ba_and_stash(robot_id, mode=mode)
        if diag is None:
            st = self._states[robot_id]
            return {
                "success": False,
                "message": f"Hand-Eye 실패 (포즈 수: {len(st.hand_eye.poses)})",
                "data": {},
            }
        rec_result = self._compute_recommendations(robot_id)
        diag["recommendations"] = rec_result["recommendations"]
        diag["no_candidates_reason"] = rec_result["no_candidates_reason"]
        self._publish_sigma_state(robot_id, diag)
        return {
            "success": True,
            "message": f"compute 완료 (poses={diag['pose_count']})",
            "data": diag,
        }

    def _srv_handeye_commit(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[HandeyeCommitRes]:
        st = self._states[robot_id]
        if st.last_compute is None or st.hand_eye.result is None:
            return ServiceResponse(
                success=False, message="먼저 COMPUTE를 실행하세요", data=None
            )

        # 0) pre-commit snapshot — 현재 live disk 상태 통째로 백업 → rollback picker.
        try:
            calib_backup.snapshot(
                calibration_dir=_save_dir(robot_id),
                tag="pre-commit",
                meta={
                    "sigma_rot_deg": st.last_compute.get("sigma_rot_deg"),
                    "sigma_t_mm": st.last_compute.get("sigma_t_mm"),
                    "capture_count": len(st.hand_eye.poses),
                    "ba_mode": st.last_compute.get("method"),
                },
            )
        except Exception as e:
            logger.warning("[%s] pre-commit snapshot 실패: %s", robot_id, e)

        # 1) hand_eye.npz — BA 출력이 절대값이라 overwrite contract.
        hand_eye_path = _save_dir(robot_id) / "hand_eye.npz"
        st.hand_eye.save(hand_eye_path)

        # 2) joint_offsets — COMPUTE 시점 stash 한 absolute 만 사용.
        # Bug A: 옛 commit_offsets(delta) 가 호출마다 existing + delta 누적해서
        # COMMIT 두 번 누르면 double-add. commit_absolute(absolute) 로 통일 +
        # 끝에서 last_compute invalidate → 두 번 클릭 == idempotent.
        applied: dict[int, float] = {}
        offset_msg = ""
        if st.last_compute.get("joint_offset_estimated"):
            absolute_by_id: dict[int, float] = (
                st.last_compute["_joint_absolute_by_id"]
            )
            applied = JointCoordinates().commit_absolute(
                absolute_by_id,
                method=st.hand_eye.result.method,
                robot_id=robot_id,
            )
            applied_deg = {
                i: round(float(np.degrees(v)), 3) for i, v in applied.items()
            }
            offset_msg = f" + joint_offsets 갱신 (absolute, deg={applied_deg})"
            logger.info("[%s] joint_offsets 즉시 적용: %s", robot_id, applied_deg)

        # 3) link_offsets — BA 출력이 이미 absolute total.
        link_msg = ""
        link_applied_meta: list[dict] = []
        restart_required = False
        if st.last_compute.get("link_offset_estimated"):
            trans_list = st.last_compute.get("link_trans_delta", [])
            rot_list = st.last_compute.get("link_rot_delta", [])
            new_link = LinkOffsets(
                trans={
                    int(e["motor_id"]): np.array(
                        [e["x_m"], e["y_m"], e["z_m"]], dtype=np.float64
                    )
                    for e in trans_list
                },
                rot={
                    int(e["motor_id"]): np.array(
                        [e["rx_rad"], e["ry_rad"], e["rz_rad"]], dtype=np.float64
                    )
                    for e in rot_list
                },
            )
            link_applied = LinkCoordinates().commit_absolute(
                new_link,
                method=st.hand_eye.result.method,
                robot_id=robot_id,
            )
            n_joints = len(link_applied.trans)
            link_msg = (
                f" + link_offsets 갱신 (absolute, n={n_joints}, 백엔드 재시작 후 FK/IK 적용)"
            )
            link_applied_meta = [
                {
                    "motor_id": int(jid),
                    "trans_m": link_applied.get_trans(jid).tolist(),
                    "rot_rad": link_applied.get_rot(jid).tolist(),
                }
                for jid in sorted(link_applied.trans.keys())
            ]
            restart_required = True
            logger.info(
                "[%s] link_offsets 디스크 적용 (absolute, 재시작 필요): n=%d",
                robot_id, n_joints,
            )

        # 4) sag_offsets — BA 출력이 absolute total.
        sag_msg = ""
        sag_applied_meta: list[dict] = []
        if st.last_compute.get("sag_offset_estimated"):
            sag_delta_list = st.last_compute.get("sag_offset_delta", [])
            new_sag = SagOffsets(
                k_rad_per_m={
                    int(e["motor_id"]): float(e["k_rad_per_m"])
                    for e in sag_delta_list
                },
            )
            sag_applied = SagCoordinates().commit_absolute(
                new_sag,
                method=st.hand_eye.result.method,
                robot_id=robot_id,
            )
            st.solver._reload_sag_cache()
            sag_applied_meta = [
                {
                    "motor_id": int(jid),
                    "k_rad_per_m": float(sag_applied.get_k(jid)),
                }
                for jid in sorted(sag_applied.k_rad_per_m.keys())
            ]
            n_sag = len(sag_applied.k_rad_per_m)
            sag_msg = f" + sag_offsets 갱신 (absolute, n={n_sag}, 즉시 적용)"
            logger.info(
                "[%s] sag_offsets 즉시 적용: %s",
                robot_id,
                {m["motor_id"]: round(m["k_rad_per_m"], 5)
                 for m in sag_applied_meta},
            )

        # Response 준비 — last_compute 의 estimated 플래그를 invalidate 전에 캡처.
        joint_estimated = st.last_compute.get("joint_offset_estimated", False)
        link_estimated = st.last_compute.get("link_offset_estimated", False)
        sag_estimated = st.last_compute.get("sag_offset_estimated", False)

        # Bug A fix: COMMIT 끝에 invalidate. 같은 compute 로 다시 누르면
        # "먼저 COMPUTE 를 실행하세요" 응답 → disk 누적 부작용 0.
        st.last_compute = None

        return ServiceResponse(
            success=True,
            message=f"저장 완료: {hand_eye_path}{offset_msg}{link_msg}{sag_msg}",
            data=HandeyeCommitRes(
                path=str(hand_eye_path),
                method=st.hand_eye.result.method,
                joint_offsets_applied=joint_estimated,
                joint_offsets=[
                    JointOffsetEntry(motor_id=int(mid), offset_rad=float(off))
                    for mid, off in sorted(applied.items())
                ],
                link_offsets_applied=link_estimated,
                link_offsets=[
                    LinkOffsetEntry.model_validate(m)
                    for m in link_applied_meta
                ],
                sag_offsets_applied=sag_estimated,
                sag_offsets=[
                    SagOffsetEntry.model_validate(m)
                    for m in sag_applied_meta
                ],
                restart_required=restart_required,
            ),
        )

    # ─── 명시 신호 / Multi-start ──────────────────────────────
    def _srv_handeye_recommendation_fail(
        self, req: ServiceRequest[RecommendationFailReq], robot_id: str
    ) -> ServiceResponse[RecommendationFailRes]:
        """사용자 명시 신호 — 추천 자세 fail 기록. 다음 추천 시 제외.

        카테고리: not_visible / red / motion_fail. 분류 자체 자리 자취 자리 = backend
        log 자체 자리 자취 자리 자체 자리 자취 자리 진단 자체 자리 자취 자리 — 알고리즘 자체 자리 자취 자리 자체 자리 자취 자리 모두
        같이 자체 자리 자취 자리 fail mark.
        """
        st = self._states[robot_id]
        anchor_id = req.data.anchor_id
        category = req.data.category
        st.recommendation_fail_ids.add(anchor_id)
        logger.info(
            "[%s] 추천 fail: anchor_id=%s, category=%s",
            robot_id, anchor_id, category,
        )
        # 즉시 추천 갱신 publish
        self._publish_recommendations(robot_id)
        return ServiceResponse(
            success=True,
            message=f"fail 기록: {anchor_id} ({category})",
            data=RecommendationFailRes(
                excluded_count=len(st.recommendation_fail_ids)
            ),
        )

    def _srv_handeye_multi_start(
        self, _req: ServiceRequest[MultiStartReq], robot_id: str
    ) -> ServiceResponse[MultiStartRes]:
        """Multi-mode BA — standard / extended / physical_sag 시도 → 가장 좋은 σ.

        Local minimum 자리 escape 자체 자리 — 같은 데이터로 다른 모델 가정 자체 자리
        시도. 사용자가 saturate 알림 받고 명시 트리거 자체 자리, 또는 [수동 모드
        종료] 시점 자체 자리 자동.

        진짜 random init multi-start 자체 자리 (각 init 자체 자리 random rotation /
        translation) 자체 자리 자취 자리 = TODO. 현재 = 3 BA mode 시도 자체 자리 자취 자리.
        """
        st = self._states[robot_id]
        if len(st.hand_eye.poses) < calib_thresholds.MIN_POSES_FOR_COMPUTE:
            return ServiceResponse(
                success=False,
                message=(
                    f"포즈 수 부족 ({len(st.hand_eye.poses)} < "
                    f"{calib_thresholds.MIN_POSES_FOR_COMPUTE})"
                ),
                data=None,
            )

        baseline_rot: float | None = None
        baseline_t: float | None = None
        if st.last_compute is not None:
            baseline_rot = st.last_compute.get("sigma_rot_deg")
            baseline_t = st.last_compute.get("sigma_t_mm")

        modes = ["standard", "extended", "physical_sag"]
        n_converged = 0
        best_diag: dict | None = None
        best_sigma = float("inf")

        for mode in modes:
            try:
                diag = self._run_ba_and_stash(robot_id, mode=mode)
                if diag is None:
                    continue
                n_converged += 1
                sigma = diag.get("sigma_rot_deg")
                if sigma is None:
                    continue
                if float(sigma) < best_sigma:
                    best_sigma = float(sigma)
                    best_diag = diag
            except Exception as e:
                logger.warning(
                    "[%s] multi-start mode=%s 실패: %s", robot_id, mode, e
                )

        if best_diag is None:
            return ServiceResponse(
                success=False, message="모든 BA mode 실패", data=None
            )

        # best 결과 last_compute 자체 자리 + topic publish
        st.last_compute = best_diag
        self._publish_sigma_state(robot_id, best_diag)
        self._publish_saturate_state(robot_id, best_diag)
        self._publish_recommendations(robot_id)

        best_rot = best_diag.get("sigma_rot_deg")
        best_t = best_diag.get("sigma_t_mm")
        improvement_rot = (
            float(baseline_rot) - float(best_rot)
            if baseline_rot is not None and best_rot is not None
            else None
        )
        improvement_t = (
            float(baseline_t) - float(best_t)
            if baseline_t is not None and best_t is not None
            else None
        )
        return ServiceResponse(
            success=True,
            message=(
                f"multi-start: n_converged={n_converged}/{len(modes)}, "
                f"best σ_rot={best_rot}°"
            ),
            data=MultiStartRes(
                n_tried=len(modes),
                n_converged=n_converged,
                sigma_rot_deg=float(best_rot) if best_rot is not None else None,
                sigma_t_mm=float(best_t) if best_t is not None else None,
                improvement_rot_deg=improvement_rot,
                improvement_t_mm=improvement_t,
            ),
        )

    # ─── Backup / Rollback ───────────────────────────────────
    def _srv_backup_list(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[BackupListRes]:
        infos = calib_backup.list_snapshots(_save_dir(robot_id))
        entries = [
            BackupEntry(
                timestamp=i.timestamp,
                tag=i.tag,
                sigma_rot_deg=_optional_float(i.meta.get("sigma_rot_deg")),
                sigma_t_mm=_optional_float(i.meta.get("sigma_t_mm")),
                capture_count=_optional_int(i.meta.get("capture_count")),
                ba_mode=_optional_str(i.meta.get("ba_mode")),
            )
            for i in infos
        ]
        return ServiceResponse(
            success=True,
            message=f"snapshots={len(entries)}",
            data=BackupListRes(snapshots=entries),
        )

    def _srv_backup_restore(
        self, req: ServiceRequest[BackupRestoreReq], robot_id: str
    ) -> ServiceResponse[BackupRestoreRes]:
        try:
            info = calib_backup.restore(_save_dir(robot_id), req.data.timestamp)
        except FileNotFoundError as e:
            return ServiceResponse(success=False, message=str(e), data=None)

        # 메모리 reload — joint/sag/tool 은 즉시 반영. link 는 URDF patch 라 재시작 필요.
        JointCoordinates().reload(robot_id)
        LinkCoordinates().reload(robot_id)
        SagCoordinates().reload(robot_id)
        ToolCoordinates().reload(robot_id)
        st = self._states[robot_id]
        st.hand_eye.load(_save_dir(robot_id) / "hand_eye.npz")
        st.intrinsic.load(_save_dir(robot_id) / "intrinsic.npz")
        st.solver._reload_sag_cache()
        # 안전상 compute 결과도 무효화 (옛 absolute 가 현 disk 와 맞지 않음).
        st.last_compute = None

        logger.info(
            "[%s] calibration snapshot 복원 완료: timestamp=%s, restart_required=True",
            robot_id, info.timestamp,
        )

        return ServiceResponse(
            success=True,
            message=f"snapshot {info.timestamp} 복원 완료 (URDF 재적용 위해 백엔드 재시작 필요)",
            data=BackupRestoreRes(
                restored_timestamp=info.timestamp,
                restart_required=True,
            ),
        )

    def _srv_handeye_thresholds(self, req: dict, _robot_id: str) -> dict:
        """legacy dict — thresholds.as_dict() free-form. robot 무관 — 모든 robot 동일 threshold."""
        return {
            "success": True,
            "message": "ok",
            "data": calib_thresholds.as_dict(),
        }

    # ─── 다음 자세 후보 리스트 산출 ────────────────────────────
    def _compute_recommendations(self, robot_id: str) -> dict:
        """{"recommendations": list[dict], "no_candidates_reason": str | None}.

        no_candidates_reason 값:
          - "insufficient_poses"           ─ MIN_POSES_FOR_COMPUTE 미달
          - "no_board_estimate"            ─ hand_eye / intrinsic / 보드 base 추정 X
          - "all_ik_fail"                  ─ planner 의 anchor 다 IK 실패
          - "all_invisible"                ─ 모든 anchor invisible (visibility hard fail 가 hint 만이라 잘 안 발생)
          - "user_marked_fail"             ─ 사용자가 모든 anchor [👎] 마크
          - "sigma_sufficient_and_diverse" ─ σ + 다양성 둘 다 충족 → COMMIT 권장
          - "sigma_sufficient_but_narrow"  ─ σ 좋은데 자세 다양성 부족 → 부족 axis 변주 캡처

        frontend NextPoseCard 가 분기 별 메시지 표시 + COMMIT 가이드.
        """
        st = self._states[robot_id]
        empty = {"recommendations": [], "no_candidates_reason": "insufficient_poses"}

        current = self._cache.get_joint_angles_rad(
            st.arm_cfgs, robot_id=robot_id
        )
        if current is None:
            return empty
        arm_motor_ids = [cfg.id for cfg in st.arm_cfgs]
        joint_limits = st.solver.joint_limits(len(arm_motor_ids))

        # 추천 = sphere shell anchor 기반 (정면 / 좌 / 우 / 위 / 아래 + IK + visibility).
        # 필요 조건: intrinsic + hand_eye + 보드 base 추정 모두 있어야 함.
        if st.intrinsic.result is None or st.hand_eye.result is None:
            return {"recommendations": [], "no_candidates_reason": "no_board_estimate"}
        estimate = self._estimate_board_base_frame(robot_id)
        if estimate is None:
            return {"recommendations": [], "no_candidates_reason": "no_board_estimate"}
        board_corners_base, avg_cam_pos = estimate
        board_center = board_corners_base.mean(axis=0)
        outward_hint = avg_cam_pos - board_center  # planner 의 board normal 부호 결정

        camera_matrix = st.intrinsic.result.camera_matrix
        dist_coeffs = st.intrinsic.result.dist_coeffs
        w, h = st.intrinsic.result.image_size
        R_c2g = st.hand_eye.result.R_cam2gripper
        t_c2g = st.hand_eye.result.t_cam2gripper
        fk_fn = st.solver.fk_to_matrix

        def _check(angles: list[float]) -> tuple[bool, str]:
            return next_pose_planner.is_pose_visible(
                angles,
                fk_fn=fk_fn,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                image_size=(int(w), int(h)),
                hand_eye_R=R_c2g,
                hand_eye_t=t_c2g,
                board_corners_base=board_corners_base,
            )

        # IK 함수 wrapper — IKSolver Protocol 의 ik() 시그니처 그대로.
        def _ik(
            target_position,
            target_quaternion,
            current_joint_angles,
        ):
            return st.solver.ik(
                target_position, target_quaternion, current_joint_angles
            )

        # 사용자가 명시 신호 ([👎]) 로 fail 표시한 추천 ID set — 다음 추천 시 제외.
        excluded_ids = st.recommendation_fail_ids

        result = next_pose_planner.recommend_geometry(
            board_corners_base=board_corners_base,
            ik_fn=_ik,
            hand_eye_R=R_c2g,
            hand_eye_t=t_c2g,
            arm_motor_ids=arm_motor_ids,
            joint_limits_rad=joint_limits,
            current_joint_angles_rad=list(current),
            outward_hint=outward_hint,
            visibility_check=_check,
            excluded_ids=excluded_ids,
        )

        # σ + 다양성 verdict 결합 — planner 단독 reason 을 verdict 기반 reason 으로
        # 덮어쓰기. σ 충분 + 다양성 OK 면 양성 메시지 (COMMIT 권장), σ 충분 + 다양성
        # 부족 이면 "narrow" 메시지. 이게 §8.7 deferred 의 fix 핵심.
        reason = result.no_candidates_reason
        if not result.recommendations:
            verdict = (st.last_compute or {}).get("coach", {}).get("verdict")
            if verdict == "good":
                reason = "sigma_sufficient_and_diverse"
            elif verdict == "narrow_sigma_good":
                reason = "sigma_sufficient_but_narrow"

        return {
            "recommendations": [
                next_pose_planner.to_dict(r) for r in result.recommendations
            ],
            "no_candidates_reason": reason,
        }

    def _srv_handeye_preview_enable(
        self, req: ServiceRequest[HandeyePreviewEnableReq], robot_id: str
    ) -> ServiceResponse[HandeyePreviewEnableRes]:
        enabled = req.data.enabled
        self._states[robot_id].preview_enabled = enabled
        return ServiceResponse(
            success=True,
            message=f"preview {'enabled' if enabled else 'disabled'}",
            data=HandeyePreviewEnableRes(enabled=enabled),
        )

    def _preview_loop(self) -> None:
        while self._running:
            for rid, st in self._states.items():
                if not st.preview_enabled:
                    continue

                try:
                    ret, frame = self._frame_cache.get_frame(robot_id=rid)
                    if not ret or frame is None:
                        self.publish(
                            topic_for(Topic.CALIB_HANDEYE_PREVIEW, rid),
                            {
                                "timestamp": time.time(),
                                "detected": False,
                                "reason": "no_frame",
                            },
                        )
                        continue

                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    h, w = gray.shape[:2]
                    ch_corners, ch_ids, m_corners, m_ids = (
                        calib_board.detect_full(gray)
                    )
                    ok = (
                        ch_corners is not None
                        and ch_ids is not None
                        and len(ch_ids) >= calib_board.MIN_CORNERS
                    )

                    payload: dict = {
                        "timestamp": time.time(),
                        "detected": bool(ok),
                        "image_size": [int(w), int(h)],
                    }

                    # marker outline — ChArUco 답게 검출된 마커 quad + ID 시각화.
                    # ok 와 무관하게 marker 잡히면 publish (사용자에게 "절반은 보이는데
                    # corner 모자란" 상황 피드백).
                    if m_corners is not None and m_ids is not None:
                        markers_payload = []
                        for quad, mid in zip(m_corners, m_ids):
                            pts = quad.reshape(-1, 2).tolist()
                            markers_payload.append(
                                {"corners": pts, "id": int(mid[0])}
                            )
                        if markers_payload:
                            payload["markers"] = markers_payload

                    if ok and ch_corners is not None and ch_ids is not None:
                        pts = ch_corners.reshape(-1, 2)
                        payload["corners"] = pts.tolist()
                        payload["corner_count"] = int(len(ch_ids))
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
                            bbox_w * bbox_h
                        ) / float(w * h)

                        if st.intrinsic.result is not None:
                            try:
                                obj_pts, img_pts = (
                                    calib_board.match_object_points(
                                        ch_corners, ch_ids
                                    )
                                )
                                ok_pnp, rvec, _tvec = cv2.solvePnP(
                                    obj_pts,
                                    img_pts,
                                    st.intrinsic.result.camera_matrix,
                                    st.intrinsic.result.dist_coeffs,
                                    flags=cv2.SOLVEPNP_ITERATIVE,
                                )
                                if ok_pnp:
                                    R, _ = cv2.Rodrigues(rvec)
                                    cos_v = float(
                                        np.clip(abs(R[2, 2]), 0.0, 1.0)
                                    )
                                    payload["tilt_deg"] = float(
                                        np.degrees(np.arccos(cos_v))
                                    )
                            except cv2.error:
                                pass

                    self.publish(
                        topic_for(Topic.CALIB_HANDEYE_PREVIEW, rid), payload
                    )
                except Exception as e:
                    logger.debug("[%s] preview loop 오류: %s", rid, e)

            time.sleep(PREVIEW_INTERVAL)
