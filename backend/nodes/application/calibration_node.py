import logging
import threading
import time
import cv2
import numpy as np
from dataclasses import dataclass
from pathlib import Path

from core.transport.application_node import ApplicationNode
from core.coords.joint_coordinates import JointCoordinates
from core.coords.link_coordinates import LinkCoordinates
from core.robot.robot_registry import RobotRegistry
from core.coords.sag_coordinates import SagCoordinates
from modules.calibration.sag_offsets import SagOffsets
from core.transport.messages.base import EmptyData, ServiceRequest, ServiceResponse
from core.transport.messages.calibration import (
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
    SagOffsetEntry,
)
from core.transport.topic_map import Service, Topic, topic_for
from core.cache.frame_cache import FrameCache
from core.cache.joint_state_cache import JointStateCache
from core.common import GRIPPER_ID
from modules.motor.motor_config import MotorConfig, load_motor_config
from modules.camera.stream import frame_to_base64
from modules.calibration.intrinsic import CHECKERBOARD, IntrinsicCalibration
from modules.calibration.hand_eye import HandEyeCalibration, Pose
from modules.calibration import next_pose_planner
from modules.calibration import thresholds as calib_thresholds
from modules.calibration.link_offsets import LinkOffsets
from modules.calibration.pose_estimator import PoseEstimator
from modules.kinematics.corrected import CorrectedIKSolver

logger = logging.getLogger(__name__)


def _save_dir(robot_id: str) -> Path:
    return RobotRegistry().get(robot_id).calibration_dir


def _handeye_poses_path(robot_id: str) -> Path:
    return _save_dir(robot_id) / "handeye_poses.npz"


PREVIEW_INTERVAL = 0.2  # 5Hz


@dataclass
class _RobotState:
    """robot 별 캘리브레이션 상태."""

    arm_cfgs: list[MotorConfig]
    intrinsic: IntrinsicCalibration
    hand_eye: HandEyeCalibration
    solver: CorrectedIKSolver
    last_compute: dict | None = None
    preview_enabled: bool = False


class CalibrationNode(ApplicationNode):
    """Application 노드 — robot 무관 한 인스턴스. robot 별 dict[robot_id] state."""

    def __init__(self) -> None:
        super().__init__("calibration_node")

        # pose_estimator 는 stateless — robot 무관 한 인스턴스.
        self.pose_estimator = PoseEstimator()

        self._frame_cache = FrameCache()
        self._cache = JointStateCache()

        # robot 별 상태
        self._states: dict[str, _RobotState] = {}
        for rid in self.enabled_robot_ids:
            _, motor_cfgs = load_motor_config(rid)
            arm_cfgs = [m for m in motor_cfgs if m.id != GRIPPER_ID]
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
            detected, vis = st.intrinsic.capture(frame)
            b64 = frame_to_base64(vis)
            return ServiceResponse(
                success=True,
                message="체커보드 감지됨" if detected else "체커보드 미감지",
                data=CalibCaptureRes(
                    detected=detected,
                    captured_count=len(st.intrinsic.obj_points),
                    preview=b64,
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
            ),
        )

    # ─── Hand-Eye 캘리브레이션 ────────────────────────────────

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

        detected, _ = st.intrinsic.capture(frame)
        if not detected:
            return ServiceResponse(
                success=False,
                message="체커보드 미감지",
                data=HandeyeCaptureRes(
                    detected=False, pose_count=len(st.hand_eye.poses)
                ),
            )

        pose = self.pose_estimator.estimate(
            obj_points=st.intrinsic.obj_points[-1],
            img_points=st.intrinsic.img_points[-1],
            camera_matrix=st.intrinsic.result.camera_matrix,
            dist_coeffs=st.intrinsic.result.dist_coeffs,
        )
        if pose is None:
            return ServiceResponse(
                success=False, message="포즈 추정 실패", data=None
            )

        st.hand_eye.add_pose(
            Pose(
                raw_motor_positions=raw_positions,
                R_target2cam=pose.R,
                t_target2cam=pose.t,
            )
        )

        st.last_compute = None
        try:
            st.hand_eye.save_poses(_handeye_poses_path(robot_id))
        except Exception as e:
            logger.warning("[%s] 포즈 디스크 저장 실패: %s", robot_id, e)

        return ServiceResponse(
            success=True,
            message=f"포즈 기록됨 ({len(st.hand_eye.poses)}개) — [계산]을 눌러 진척 확인",
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
        st = self._states[robot_id]
        arm_motor_ids = [cfg.id for cfg in st.arm_cfgs]
        joint_limits = st.solver.joint_limits(len(arm_motor_ids))
        mode = str(req.get("mode", "physical_sag")).lower()
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
            return {
                "success": False,
                "message": f"Hand-Eye 실패 (포즈 수: {len(st.hand_eye.poses)})",
                "data": {},
            }
        st.last_compute = diag
        diag["recommendations"] = self._compute_recommendations(robot_id)
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

        # 1) hand_eye.npz
        hand_eye_path = _save_dir(robot_id) / "hand_eye.npz"
        st.hand_eye.save(hand_eye_path)

        # 2) joint_offsets.npz
        applied: dict[int, float] = {}
        offset_msg = ""
        if st.last_compute.get("joint_offset_estimated"):
            delta_list = st.last_compute.get("joint_offset_delta", [])
            delta_by_id = {
                int(e["motor_id"]): float(e["offset_rad"]) for e in delta_list
            }
            applied = JointCoordinates().commit_offsets(
                delta_by_id,
                method=st.hand_eye.result.method,
                robot_id=robot_id,
            )
            applied_deg = {
                i: round(float(np.degrees(v)), 3) for i, v in applied.items()
            }
            offset_msg = f" + joint_offsets 갱신 (cumulative, deg={applied_deg})"
            logger.info("[%s] joint_offsets 즉시 적용: %s", robot_id, applied_deg)

        # 3) link_offsets.npz
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
            link_applied = LinkCoordinates().commit_offsets(
                new_link,
                method=st.hand_eye.result.method,
                robot_id=robot_id,
            )
            n_joints = len(link_applied.trans)
            link_msg = (
                f" + link_offsets 갱신 (overwrite, n={n_joints}, 백엔드 재시작 후 FK/IK 적용)"
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
                "[%s] link_offsets 디스크 적용 (overwrite, 재시작 필요): n=%d",
                robot_id, n_joints,
            )

        # 4) sag_offsets.npz
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
            sag_applied = SagCoordinates().commit_offsets(
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
            sag_msg = f" + sag_offsets 갱신 (overwrite, n={n_sag}, 즉시 적용)"
            logger.info(
                "[%s] sag_offsets 즉시 적용: %s",
                robot_id,
                {m["motor_id"]: round(m["k_rad_per_m"], 5)
                 for m in sag_applied_meta},
            )

        return ServiceResponse(
            success=True,
            message=f"저장 완료: {hand_eye_path}{offset_msg}{link_msg}{sag_msg}",
            data=HandeyeCommitRes(
                path=str(hand_eye_path),
                method=st.hand_eye.result.method,
                joint_offsets_applied=st.last_compute.get(
                    "joint_offset_estimated", False
                ),
                joint_offsets=[
                    JointOffsetEntry(motor_id=int(mid), offset_rad=float(off))
                    for mid, off in sorted(applied.items())
                ],
                link_offsets_applied=st.last_compute.get(
                    "link_offset_estimated", False
                ),
                link_offsets=[
                    LinkOffsetEntry.model_validate(m)
                    for m in link_applied_meta
                ],
                sag_offsets_applied=st.last_compute.get(
                    "sag_offset_estimated", False
                ),
                sag_offsets=[
                    SagOffsetEntry.model_validate(m)
                    for m in sag_applied_meta
                ],
                restart_required=restart_required,
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
    def _compute_recommendations(self, robot_id: str) -> list[dict]:
        st = self._states[robot_id]
        current = self._cache.get_joint_angles_rad(
            st.arm_cfgs, robot_id=robot_id
        )
        if current is None:
            return []
        arm_motor_ids = [cfg.id for cfg in st.arm_cfgs]
        joint_limits = st.solver.joint_limits(len(arm_motor_ids))
        ja_at_compute = (
            st.last_compute.get("joint_angles_per_pose")
            if st.last_compute
            else None
        )
        recs = next_pose_planner.recommend_many(
            last_compute=st.last_compute,
            joint_angles_per_pose_at_compute=ja_at_compute,
            current_joint_angles_rad=list(current),
            arm_motor_ids=arm_motor_ids,
            joint_limits_rad=joint_limits,
        )
        return [next_pose_planner.to_dict(r) for r in recs]

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
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE
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

                        if st.intrinsic.result is not None:
                            try:
                                ok, rvec, _tvec = cv2.solvePnP(
                                    st.intrinsic._objp_template,
                                    corners,
                                    st.intrinsic.result.camera_matrix,
                                    st.intrinsic.result.dist_coeffs,
                                    flags=cv2.SOLVEPNP_ITERATIVE,
                                )
                                R, _ = cv2.Rodrigues(rvec)
                                cos_v = float(np.clip(abs(R[2, 2]), 0.0, 1.0))
                                payload["tilt_deg"] = float(
                                    np.degrees(np.arccos(cos_v)))
                            except cv2.error:
                                pass

                    self.publish(
                        topic_for(Topic.CALIB_HANDEYE_PREVIEW, rid), payload
                    )
                except Exception as e:
                    logger.debug("[%s] preview loop 오류: %s", rid, e)

            time.sleep(PREVIEW_INTERVAL)
