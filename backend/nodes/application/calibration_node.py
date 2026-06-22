from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np

from core.cache.frame_cache import FrameCache
from core.cache.joint_state_cache import JointStateCache
from core.coords.joint_coordinates import JointCoordinates
from core.coords.link_coordinates import LinkCoordinates
from core.coords.sag_coordinates import SagCoordinates
from core.transport.application_node import ApplicationNode
from core.transport.messages.base import EmptyData, ServiceRequest, ServiceResponse
from core.transport.messages.calibration import (
    HandeyeCaptureRes,
    HandeyeFinalizeRes,
    HandeyeListPosesRes,
    HandeyePoseMeta,
    HandeyePreviewEnableReq,
    HandeyePreviewEnableRes,
    HandeyeResetRes,
    HandeyeStartRes,
    HandeyeUndoLastCaptureRes,
    IntrinsicCaptureRes,
    IntrinsicSaveRes,
)
from core.transport.topic_map import Service, Topic, key_for
from modules.calibration import board as calib_board
from modules.calibration import capture_quality as cq
from modules.calibration import thresholds as calib_thresholds
from modules.calibration.calibration_cache import CalibrationCache
from modules.calibration.intrinsic import IntrinsicCalibration, IntrinsicResult
from modules.calibration.loader import CalibrationData, HandEyeData, IntrinsicData
from modules.calibration.persistence_models import (
    CalibrationCaptureRecord,
    CalibrationRunRecord,
    IntrinsicResultRecord,
)
from modules.calibration.result_models import (
    IntrinsicResultData,
    LinkOffsetResultData,
    SagOffsetResultData,
)
from modules.calibration.storage_client import (
    CalibrationStorageClient,
    load_active_blocking,
)
from core.transport.messages.storage import CalibrationInvalidated
from modules.camera import depth_frame as dframe
from modules.camera.stream import frame_to_base64
from modules.kinematics.adapters.pybullet_kinematics import PybulletKinematics
from modules.kinematics.adapters.sag_corrected import SagCorrectedKinematics
from modules.motor.motor_config import MotorConfig, load_motor_layout

logger = logging.getLogger(__name__)


PREVIEW_INTERVAL = 0.2  # 5Hz
DEPTH_FRAME_FRESH_SEC = 2.0  # 캡처 시 depth_frame 최대 허용 나이


@dataclass
class _CapturePose:
    motor_positions: dict[int, int]
    R_target2cam: np.ndarray  # (3, 3)
    t_target2cam: np.ndarray  # (3,)
    pose_index: int


@dataclass
class _RobotState:
    arm_cfgs: list[MotorConfig]
    intrinsic: IntrinsicCalibration
    hand_eye_run_id: int | None = None
    session_intrinsic: IntrinsicResult | None = None
    captures: list[_CapturePose] = field(default_factory=list)
    preview_enabled: bool = False
    latest_depth_blob: bytes | None = None
    latest_depth_blob_ts: float = 0.0


class CalibrationNode(ApplicationNode):
    def __init__(self) -> None:
        super().__init__("calibration_node")

        self._frame_cache = FrameCache()
        self._joint_cache = JointStateCache()

        self._states: dict[str, _RobotState] = {}
        for rid in self.enabled_robot_ids:
            arm_cfgs = load_motor_layout(rid).arm
            intrinsic = IntrinsicCalibration()
            self._states[rid] = _RobotState(
                arm_cfgs=arm_cfgs, intrinsic=intrinsic
            )

        self._preview_thread: threading.Thread | None = None
        self._setup_thread: threading.Thread | None = None
        self._storage = CalibrationStorageClient()
        # ACTIVATE 마다 publish 되는 invalidation subscribe — Pi 의 factory intrinsic
        # seed 가 PC 부팅 *후* commit + activate 하는 경로 자리, PC 의 CalibrationCache
        # 가 stale 자리 안 되도록 refetch.
        self._invalidation_sub: object | None = None

    # ─── 부팅 — calibration cache push + draft 복원 ──────────

    def _setup_runtime_calibration(self) -> None:
        """부팅 background — Storage 대기 → 5종 fetch → in-memory cache push.

        docs/storage_layer.md §7 — Calibration Service 만 storage 앎. 다른 노드/
        Coordinates/PybulletKinematics 는 본 push 받음.
        """
        for rid in self.enabled_robot_ids:
            try:
                self._push_calibration(rid)
                logger.info("[%s] runtime calibration push 완료", rid)
            except Exception:
                logger.exception("[%s] runtime calibration setup 실패", rid)

    def _push_calibration(self, robot_id: str) -> None:
        """Atomic snapshot — 5종 다 fetch *후* push. partial state 차단."""
        joint_rec = load_active_blocking(robot_id, "joint_offset")
        link_rec = load_active_blocking(robot_id, "link_offset")
        sag_rec = load_active_blocking(robot_id, "sag")
        intrinsic_rec = load_active_blocking(robot_id, "intrinsic")
        hand_eye_rec = load_active_blocking(robot_id, "hand_eye")

        joint_offsets = (
            dict(joint_rec.result_data.offsets)
            if joint_rec is not None and joint_rec.kind == "joint_offset"
            else {}
        )
        if link_rec is not None and link_rec.kind == "link_offset":
            link_offsets = link_rec.result_data
        else:
            link_offsets = LinkOffsetResultData(offsets=[], method="empty")
        sag_offsets = (
            sag_rec.result_data
            if sag_rec is not None and sag_rec.kind == "sag"
            else SagOffsetResultData(k_rad_per_m={}, method="empty")
        )
        intrinsic = (
            IntrinsicData(
                camera_matrix=np.array(
                    intrinsic_rec.result_data.camera_matrix, dtype=np.float64
                ),
                dist_coeffs=np.array(
                    intrinsic_rec.result_data.dist_coeffs, dtype=np.float64
                ),
                image_size=(
                    # type: ignore[arg-type]
                    tuple(intrinsic_rec.result_data.image_size)
                    if intrinsic_rec.result_data.image_size is not None
                    else None
                ),
            )
            if intrinsic_rec is not None and intrinsic_rec.kind == "intrinsic"
            else None
        )
        hand_eye = (
            HandEyeData(
                R=np.array(
                    hand_eye_rec.result_data.R_cam2gripper, dtype=np.float64
                ),
                t=np.array(
                    hand_eye_rec.result_data.t_cam2gripper, dtype=np.float64
                ),
            )
            if hand_eye_rec is not None and hand_eye_rec.kind == "hand_eye"
            else None
        )

        # atomic push — consumer hot path 의 wait_ready 가 partial 노출 차단.
        JointCoordinates().set_offsets(robot_id, joint_offsets)
        LinkCoordinates().set_offsets(robot_id, link_offsets)
        SagCoordinates().set_offsets(robot_id, sag_offsets)

        kinematics = self._registry.get_kinematics(robot_id)
        assert isinstance(kinematics, SagCorrectedKinematics)
        inner = kinematics._inner  # type: ignore[attr-defined]
        assert isinstance(inner, PybulletKinematics)
        if not inner._initialized:  # type: ignore[attr-defined]
            inner.apply_link_offsets(link_offsets)
            inner.initialize()
        kinematics.reload_calibration()

        cache = CalibrationCache()
        cache.set(
            robot_id, CalibrationData(intrinsic=intrinsic, hand_eye=hand_eye)
        )

        # in-memory IntrinsicCalibration.result 도 storage 와 sync — preview/capture
        # 가 st.intrinsic.result 직접 읽음 (intrinsic 미완료 시 capture 거부 안내).
        st = self._states[robot_id]
        if intrinsic is not None:
            st.intrinsic.result = IntrinsicResult(
                camera_matrix=intrinsic.camera_matrix,
                dist_coeffs=intrinsic.dist_coeffs,
                rms_error=0.0,
                image_size=intrinsic.image_size or (0, 0),
                captured_count=0,
                coverage_cells=[],
            )
        else:
            st.intrinsic.result = None

        cache.signal_ready(robot_id)

    def _restore_in_progress_handeye(
        self, robot_id: str, st: _RobotState
    ) -> None:
        """부팅 시 storage 의 in_progress hand_eye run 복원.

        ready_for_analysis run 자리는 복원 안 함 (immutable, frontend session 종료).
        """
        existing = self._storage.get_in_progress_run(robot_id, "hand_eye")
        if existing is None:
            return
        run, captures = existing
        st.hand_eye_run_id = run.id
        # session_intrinsic 복원 — run.algorithm_params["intrinsic_snapshot"].
        snap = run.algorithm_params.get(
            "intrinsic_snapshot") if run.algorithm_params else None
        if isinstance(snap, dict):
            try:
                st.session_intrinsic = IntrinsicResult(
                    camera_matrix=np.array(
                        snap["camera_matrix"], dtype=np.float64),
                    dist_coeffs=np.array(
                        snap["dist_coeffs"], dtype=np.float64),
                    rms_error=float(snap.get("rms_error", 0.0)),
                    # type: ignore[arg-type]
                    image_size=tuple(snap.get("image_size", (0, 0))),
                    captured_count=0,
                    coverage_cells=[],
                )
            except Exception:
                logger.exception(
                    "[%s] session intrinsic 복원 실패 — capture 막힘", robot_id
                )
                st.session_intrinsic = None
        # capture cache 복원.
        st.captures.clear()
        for cap in captures:
            if (
                cap.board_in_cam is None
                or cap.motor_positions is None
            ):
                logger.warning(
                    "[%s] capture row 손상 — skip (pose_index=%d)",
                    robot_id, cap.pose_index,
                )
                continue
            T = np.asarray(cap.board_in_cam, dtype=np.float64)
            st.captures.append(
                _CapturePose(
                    motor_positions=dict(cap.motor_positions),
                    R_target2cam=T[:3, :3].copy(),
                    t_target2cam=T[:3, 3].copy(),
                    pose_index=cap.pose_index,
                )
            )
        logger.info(
            "[%s] in_progress hand_eye run 복원 (run_id=%d, %d장)",
            robot_id, run.id, len(captures),
        )

    # ─── lifecycle ──────────────────────────────────────────────

    def start(self) -> None:
        for rid in self.enabled_robot_ids:
            self._frame_cache.subscribe(self, robot_id=rid)
            # CAMERA_DEPTH_FRAME raw subscribe — latest blob 캐시.
            self.create_raw_subscriber(
                key_for(Topic.CAMERA_DEPTH_FRAME, rid),
                lambda payload, _rid=rid: self._on_depth_frame(_rid, payload),
            )

            # ─── Intrinsic 캘 (별개 flow, 새 시나리오 무관) ─────────
            self.create_service(
                key_for(Service.CALIB_INTRINSIC_CAPTURE, rid),
                EmptyData,
                IntrinsicCaptureRes,
                lambda req, _rid=rid: self._srv_intrinsic_capture(req, _rid),
            )
            self.create_service(
                key_for(Service.CALIB_INTRINSIC_START, rid),
                EmptyData,
                EmptyData,
                lambda req, _rid=rid: self._srv_intrinsic_start(req, _rid),
            )
            self.create_service(
                key_for(Service.CALIB_INTRINSIC_SAVE, rid),
                EmptyData,
                IntrinsicSaveRes,
                lambda req, _rid=rid: self._srv_intrinsic_save(req, _rid),
            )

            # ─── Hand-Eye 캡처 flow ─────────────────────────────────
            self.create_service(
                key_for(Service.CALIB_HANDEYE_START, rid),
                EmptyData,
                HandeyeStartRes,
                lambda req, _rid=rid: self._srv_handeye_start(req, _rid),
            )
            self.create_service(
                key_for(Service.CALIB_HANDEYE_CAPTURE, rid),
                EmptyData,
                HandeyeCaptureRes,
                lambda req, _rid=rid: self._srv_handeye_capture(req, _rid),
            )
            self.create_service(
                key_for(Service.CALIB_HANDEYE_RESET, rid),
                EmptyData,
                HandeyeResetRes,
                lambda req, _rid=rid: self._srv_handeye_reset(req, _rid),
            )
            self.create_service(
                key_for(Service.CALIB_HANDEYE_UNDO_LAST_CAPTURE, rid),
                EmptyData,
                HandeyeUndoLastCaptureRes,
                lambda req, _rid=rid: self._srv_handeye_undo_last_capture(
                    req, _rid
                ),
            )
            self.create_service(
                key_for(Service.CALIB_HANDEYE_FINALIZE, rid),
                EmptyData,
                HandeyeFinalizeRes,
                lambda req, _rid=rid: self._srv_handeye_finalize(req, _rid),
            )
            self.create_service(
                key_for(Service.CALIB_HANDEYE_LIST_POSES, rid),
                EmptyData,
                HandeyeListPosesRes,
                lambda req, _rid=rid: self._srv_handeye_list_poses(req, _rid),
            )
            self.create_service(
                key_for(Service.CALIB_HANDEYE_PREVIEW_ENABLE, rid),
                HandeyePreviewEnableReq,
                HandeyePreviewEnableRes,
                lambda req, _rid=rid: self._srv_handeye_preview_enable(
                    req, _rid
                ),
            )
            self.create_service(
                key_for(Service.CALIB_HANDEYE_THRESHOLDS, rid),
                lambda req, _rid=rid: self._srv_handeye_thresholds(req, _rid),
            )

        super().start()
        self._joint_cache.subscribe(self)
        self._preview_thread = threading.Thread(
            target=self._preview_loop, daemon=True, name="calib-preview"
        )
        self._preview_thread.start()

        # 부팅 시 in_progress hand_eye run 복원 — DB SSOT. browser reload / backend
        # restart 자리 자리 세션 잃지 않음.
        for rid, st in self._states.items():
            try:
                self._restore_in_progress_handeye(rid, st)
            except Exception:
                logger.exception("[%s] in_progress 복원 실패", rid)

        # 부팅 시 storage 에서 5종 fetch + push (background — main start 안 막힘).
        self._setup_thread = threading.Thread(
            target=self._setup_runtime_calibration,
            daemon=True,
            name="calib-setup",
        )
        self._setup_thread.start()

        # ACTIVATE 이벤트 구독 — Pi 가 PC 부팅 후 intrinsic seed 하는 시나리오 자리,
        # 또는 offline 스크립트가 새 hand_eye result 활성화 자리 PC CalibrationCache 갱신.
        self._invalidation_sub = self._storage.subscribe_invalidations(
            self._on_calibration_invalidated
        )

        logger.info("CalibrationNode 시작 (robots=%s)", self.enabled_robot_ids)

    def _on_calibration_invalidated(self, msg: CalibrationInvalidated) -> None:
        """ACTIVATE 마다 publish. 본 process 의 enabled robot 자리만 _push_calibration."""
        if msg.robot_id not in self._states:
            return
        try:
            self._push_calibration(msg.robot_id)
            logger.info(
                "[%s] invalidation 받음 (kind=%s) — calibration cache 갱신",
                msg.robot_id, msg.kind,
            )
        except Exception:
            logger.exception(
                "[%s] invalidation 처리 실패", msg.robot_id
            )

    # ─── depth frame raw subscribe ──────────────────────────────

    def _on_depth_frame(self, robot_id: str, payload: bytes) -> None:
        st = self._states.get(robot_id)
        if st is None:
            return
        st.latest_depth_blob = payload
        st.latest_depth_blob_ts = time.monotonic()

    def _latest_depth_frame(
        self, robot_id: str
    ) -> tuple[bytes, dframe.DepthFrame] | None:
        """fresh depth_frame raw blob + decoded. 없으면 None."""
        st = self._states.get(robot_id)
        if st is None or st.latest_depth_blob is None:
            return None
        age = time.monotonic() - st.latest_depth_blob_ts
        if age > DEPTH_FRAME_FRESH_SEC:
            return None
        try:
            decoded = dframe.decode(st.latest_depth_blob)
        except Exception:
            logger.exception("[%s] depth_frame decode 실패", robot_id)
            return None
        return st.latest_depth_blob, decoded

    # ─── Intrinsic flow (불변) ──────────────────────────────────

    def _srv_intrinsic_capture(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[IntrinsicCaptureRes]:
        st = self._states[robot_id]
        ret, frame = self._frame_cache.get_frame(robot_id=robot_id)
        if not ret or frame is None:
            return ServiceResponse(
                success=False, message="카메라 프레임을 읽을 수 없습니다", data=None
            )
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
            data=IntrinsicCaptureRes(
                detected=detected,
                captured_count=len(st.intrinsic.obj_points),
                preview=b64,
                hint=hint,
                coverage_count=len(st.intrinsic.coverage_cells),
            ),
        )

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

        now = time.time()
        run = CalibrationRunRecord(
            robot_id=robot_id,
            started_at=now,
            ended_at=now,
            algorithm="intrinsic_chessboard",
            algorithm_params={"image_size": list(image_size)},
        )
        record = IntrinsicResultRecord(  # type: ignore[arg-type]
            run_id=0,
            robot_id=robot_id,
            created_at=now,
            sigma_rot=None,
            sigma_t=None,
            result_data=IntrinsicResultData(
                camera_matrix=result.camera_matrix.tolist(),
                dist_coeffs=result.dist_coeffs.tolist(),
                image_size=[int(width), int(height)],
            ),
        )
        try:
            _, result_ids = self._storage.commit(run, [record], [])
            self._storage.activate(result_ids[0])
        except Exception as e:
            logger.exception("[%s] intrinsic storage commit 실패", robot_id)
            return ServiceResponse(
                success=False, message=f"storage commit 실패: {e}", data=None
            )

        self._push_calibration(robot_id)
        logger.info(
            "[%s] intrinsic COMMIT: rms=%.4f, captured=%d",
            robot_id, result.rms_error, len(st.intrinsic.obj_points),
        )
        return ServiceResponse(
            success=True,
            message="내부 캘리브레이션 저장",
            data=IntrinsicSaveRes(
                rms_error=result.rms_error,
                camera_matrix=result.camera_matrix.tolist(),
                dist_coeffs=result.dist_coeffs.tolist(),
                captured_count=len(st.intrinsic.obj_points),
                coverage_count=len(st.intrinsic.coverage_cells),
                coverage_cells=[list(c) for c in st.intrinsic.coverage_cells],
            ),
        )

    # ─── Hand-Eye capture flow ──────────────────────────────────

    def _srv_handeye_start(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[HandeyeStartRes]:
        st = self._states[robot_id]
        if st.intrinsic.result is None:
            return ServiceResponse(
                success=False,
                message="내부 캘리브레이션(intrinsic) 필요 — 먼저 완료해주세요",
                data=None,
            )
        if st.hand_eye_run_id is not None:
            return ServiceResponse(
                success=False,
                message=(
                    "이미 진행 중인 세션 — 리셋 또는 세션 종료 후 다시 시작"
                ),
                data=None,
            )

        # 세션 freeze — 현재 active intrinsic + board_spec 자리 algorithm_params 박음.
        # offline 스크립트가 이 snapshot 으로 일관된 BA 입력 확보.
        intrinsic_snapshot: dict[str, Any] = {
            "camera_matrix": st.intrinsic.result.camera_matrix.tolist(),
            "dist_coeffs": st.intrinsic.result.dist_coeffs.tolist(),
            "image_size": list(st.intrinsic.result.image_size),
            "rms_error": st.intrinsic.result.rms_error,
        }
        board_spec = calib_board.spec_as_dict()
        now = time.time()
        run = CalibrationRunRecord(
            robot_id=robot_id,
            started_at=now,
            algorithm="hand_eye_capture_only",
            algorithm_params={
                "intrinsic_snapshot": intrinsic_snapshot,
                "board_spec": board_spec,
            },
            status="in_progress",
            kind="hand_eye",
        )
        try:
            run_id = self._storage.new_run(run)
        except Exception as e:
            logger.exception("[%s] hand_eye new_run 실패", robot_id)
            return ServiceResponse(
                success=False, message=f"세션 생성 실패: {e}", data=None
            )

        st.hand_eye_run_id = run_id
        st.session_intrinsic = st.intrinsic.result
        st.captures.clear()
        logger.info("[%s] hand_eye 세션 시작 run_id=%d", robot_id, run_id)
        return ServiceResponse(
            success=True,
            message="hand-eye 세션 시작",
            data=HandeyeStartRes(run_id=run_id, pose_count=0),
        )

    def _srv_handeye_capture(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[HandeyeCaptureRes]:
        st = self._states[robot_id]
        if st.hand_eye_run_id is None or st.session_intrinsic is None:
            return ServiceResponse(
                success=False,
                message="세션 없음 — [캘 시작] 부터 눌러주세요",
                data=HandeyeCaptureRes(
                    detected=False, pose_count=len(st.captures)),
            )

        # 1. 최신 depth_frame raw + decoded — blob 그대로 storage 로 넘김 (재인코딩 X).
        latest = self._latest_depth_frame(robot_id)
        if latest is None:
            return ServiceResponse(
                success=False,
                message=(
                    "depth 스트림 OFF — Scene Controls 의 Point Cloud 를 켜주세요"
                ),
                data=HandeyeCaptureRes(
                    detected=False, pose_count=len(st.captures)),
            )
        blob_bytes, df = latest

        # 2. raw motor positions — drift-free SSOT.
        raw_positions = self._joint_cache.get_raw_motor_positions(
            st.arm_cfgs, robot_id=robot_id
        )
        if raw_positions is None:
            return ServiceResponse(
                success=False,
                message="모터 상태 미수신",
                data=HandeyeCaptureRes(
                    detected=False, pose_count=len(st.captures)),
            )

        # 3. ChArUco 검출 — depth_frame 의 (aligned) color 위에서.
        gray = cv2.cvtColor(df.color_bgr, cv2.COLOR_BGR2GRAY)
        ok, ch_corners, ch_ids = calib_board.detect(gray)
        if not ok or ch_corners is None or ch_ids is None:
            return ServiceResponse(
                success=False,
                message="ChArUco 보드 미감지",
                data=HandeyeCaptureRes(
                    detected=False, pose_count=len(st.captures)),
            )
        obj_pts, img_pts = calib_board.match_object_points(ch_corners, ch_ids)

        # 4. PnP — 세션 intrinsic 으로 (live intrinsic 변할 가능성 회피).
        cam_mtx = st.session_intrinsic.camera_matrix
        dist = st.session_intrinsic.dist_coeffs
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, cam_mtx, dist)
        if not ok:
            return ServiceResponse(
                success=False, message="자세 추정 실패 (solvePnP)", data=None
            )

        # 5. reprojection RMS — 품질 gate.
        projected, _ = cv2.projectPoints(obj_pts, rvec, tvec, cam_mtx, dist)
        err = np.linalg.norm(
            projected.reshape(-1, 2) - img_pts.reshape(-1, 2), axis=1
        )
        reproj_rms = float(np.sqrt(np.mean(err**2)))
        if reproj_rms > calib_thresholds.HANDEYE_PNP_RMS_REJECT_PX:
            return ServiceResponse(
                success=False,
                message=(
                    "이미지 품질이 부족해 자세 추정이 부정확합니다. "
                    "보드가 또렷이 보이는 자세에서 다시 시도해주세요."
                ),
                data=HandeyeCaptureRes(
                    detected=False, pose_count=len(st.captures)
                ),
            )

        # 6. tilt 계산 — 보드 normal 과 카메라 광축 각.
        R_t2c, _ = cv2.Rodrigues(rvec)
        # 보드 +Z normal — camera 자리 정면 자리 자리 정렬 자리 자리 dot product.
        board_normal_cam = R_t2c[:, 2]
        tilt_rad = float(
            np.arccos(np.clip(abs(board_normal_cam[2]), -1.0, 1.0)))
        tilt_deg = float(np.degrees(tilt_rad))

        # 7. board_in_cam 4x4.
        board_in_cam = np.eye(4)
        board_in_cam[:3, :3] = R_t2c
        board_in_cam[:3, 3] = np.asarray(tvec).reshape(3)

        pose_index = len(st.captures)
        record = CalibrationCaptureRecord(  # type: ignore[call-arg]
            run_id=st.hand_eye_run_id,
            pose_index=pose_index,
            motor_positions=dict(raw_positions),
            board_in_cam=board_in_cam.tolist(),
            corners_2d=[[float(c[0]), float(c[1])]
                        for c in img_pts.reshape(-1, 2)],
            corner_ids=[int(i) for i in ch_ids.reshape(-1)],
            reproj_rms_px=reproj_rms,
            tilt_deg=tilt_deg,
        )

        # 8. RDB row + ObjectStore blob 한 transaction (server 측).
        try:
            capture_id, blob_key = self._storage.append_capture(
                record, robot_id=robot_id, blob_bytes=blob_bytes
            )
        except Exception as e:
            logger.exception("[%s] capture append 실패", robot_id)
            return ServiceResponse(success=False, message=str(e), data=None)

        # 9. in-memory cache — capture_quality 다음 비교 + restore 자리.
        st.captures.append(
            _CapturePose(
                motor_positions=dict(raw_positions),
                R_target2cam=R_t2c.copy(),
                t_target2cam=np.asarray(tvec).reshape(3).copy(),
                pose_index=pose_index,
            )
        )
        logger.info(
            "[%s] capture #%d (rms=%.2fpx, tilt=%.1f°, capture_id=%d, blob=%s)",
            robot_id, pose_index, reproj_rms, tilt_deg, capture_id, blob_key,
        )
        return ServiceResponse(
            success=True,
            message=f"capture #{pose_index+1} 저장",
            data=HandeyeCaptureRes(detected=True, pose_count=len(st.captures)),
        )

    def _srv_handeye_reset(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[HandeyeResetRes]:
        st = self._states[robot_id]
        if st.hand_eye_run_id is None:
            return ServiceResponse(
                success=True, message="진행 중 세션 없음",
                data=HandeyeResetRes(pose_count=0),
            )
        run_id = st.hand_eye_run_id
        try:
            self._storage.delete_run(run_id)
        except Exception as e:
            logger.exception("[%s] hand_eye delete_run 실패", robot_id)
            return ServiceResponse(success=False, message=str(e), data=None)
        st.hand_eye_run_id = None
        st.session_intrinsic = None
        st.captures.clear()
        logger.info("[%s] hand_eye 세션 리셋 (run_id=%d 삭제)", robot_id, run_id)
        return ServiceResponse(
            success=True, message="세션 리셋",
            data=HandeyeResetRes(pose_count=0),
        )

    def _srv_handeye_undo_last_capture(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[HandeyeUndoLastCaptureRes]:
        st = self._states[robot_id]
        if st.hand_eye_run_id is None:
            return ServiceResponse(
                success=False, message="진행 중 세션 없음", data=None
            )
        if not st.captures:
            return ServiceResponse(
                success=True, message="삭제할 capture 없음",
                data=HandeyeUndoLastCaptureRes(deleted=False, pose_count=0),
            )
        try:
            deleted = self._storage.delete_last_capture(st.hand_eye_run_id)
        except Exception as e:
            logger.exception("[%s] undo_last_capture 실패", robot_id)
            return ServiceResponse(success=False, message=str(e), data=None)
        if deleted is not None:
            st.captures.pop()
            logger.info("[%s] undo capture #%d", robot_id, deleted)
        return ServiceResponse(
            success=True, message="마지막 capture 삭제",
            data=HandeyeUndoLastCaptureRes(
                deleted=deleted is not None, pose_count=len(st.captures)
            ),
        )

    def _srv_handeye_finalize(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[HandeyeFinalizeRes]:
        st = self._states[robot_id]
        if st.hand_eye_run_id is None:
            return ServiceResponse(
                success=False, message="진행 중 세션 없음", data=None
            )
        run_id = st.hand_eye_run_id
        try:
            self._storage.mark_run_ready(run_id)
        except Exception as e:
            logger.exception("[%s] mark_run_ready 실패", robot_id)
            return ServiceResponse(success=False, message=str(e), data=None)
        pose_count = len(st.captures)
        st.hand_eye_run_id = None
        st.session_intrinsic = None
        st.captures.clear()
        logger.info(
            "[%s] hand_eye 세션 종료 (run_id=%d, %d장 → ready_for_analysis)",
            robot_id, run_id, pose_count,
        )
        return ServiceResponse(
            success=True,
            message=(
                f"세션 종료 — {pose_count}장 저장. offline 분석 스크립트 실행 자리."
            ),
            data=HandeyeFinalizeRes(run_id=run_id, pose_count=pose_count),
        )

    def _srv_handeye_list_poses(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[HandeyeListPosesRes]:
        st = self._states[robot_id]
        # frontend list 자체 자체 보여주는 자체 자체 — pose_index + tilt 정도.
        # capture cache 의 R/t 만으로 tilt 재계산 자리.
        poses: list[HandeyePoseMeta] = []
        for cap in st.captures:
            normal = cap.R_target2cam[:, 2]
            tilt = float(
                np.degrees(np.arccos(np.clip(abs(normal[2]), -1.0, 1.0)))
            )
            poses.append(
                # type: ignore[call-arg]
                HandeyePoseMeta(pose_index=cap.pose_index, tilt_deg=tilt)
            )
        return ServiceResponse(
            success=True,
            data=HandeyeListPosesRes(
                poses=poses,
                pose_count=len(poses),
                run_id=st.hand_eye_run_id,
            ),
        )

    def _srv_handeye_preview_enable(
        self,
        req: ServiceRequest[HandeyePreviewEnableReq],
        robot_id: str,
    ) -> ServiceResponse[HandeyePreviewEnableRes]:
        st = self._states[robot_id]
        st.preview_enabled = bool(req.data.enabled)
        return ServiceResponse(
            success=True,
            data=HandeyePreviewEnableRes(enabled=st.preview_enabled),
        )

    def _srv_handeye_thresholds(self, _req: dict, robot_id: str) -> dict:
        # legacy form (free-form dict). mount 시 1회 fetch — thresholds.as_dict().
        # wire envelope `{success, message, data}` 직접 구성 (typed 자리 아님).
        return {"success": True, "message": "ok", "data": calib_thresholds.as_dict()}

    # ─── Preview loop — 5Hz traffic light ────────────────────────

    def _preview_loop(self) -> None:
        while self._running:
            time.sleep(PREVIEW_INTERVAL)
            for rid, st in self._states.items():
                if not st.preview_enabled:
                    continue
                try:
                    self._publish_preview(rid, st)
                except Exception:
                    logger.exception("[%s] preview 실패", rid)

    def _publish_preview(self, robot_id: str, st: _RobotState) -> None:
        # color frame (FrameCache — 30Hz, capture 자리는 depth_frame 의 aligned color
        # 사용하지만 preview 자리는 fresh 가 더 중요).
        ret, frame = self._frame_cache.get_frame(robot_id=robot_id)
        if not ret or frame is None:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ch_corners, ch_ids, marker_corners, _marker_ids = calib_board.detect_full(
            gray
        )
        detected = (
            ch_corners is not None
            and ch_ids is not None
            and len(ch_ids) >= calib_board.MIN_CORNERS
        )

        # 세션 진행 중엔 freeze 자리, 평시엔 active intrinsic 자리 PnP.
        intrinsic_for_pnp = st.session_intrinsic or st.intrinsic.result

        tilt_deg: float | None = None
        current_R: np.ndarray | None = None
        current_t: np.ndarray | None = None
        if (
            detected
            and ch_corners is not None
            and ch_ids is not None
            and intrinsic_for_pnp is not None
        ):
            try:
                obj_pts, img_pts = calib_board.match_object_points(
                    ch_corners, ch_ids
                )
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts,
                    img_pts,
                    intrinsic_for_pnp.camera_matrix,
                    intrinsic_for_pnp.dist_coeffs,
                )
                if ok:
                    R, _ = cv2.Rodrigues(rvec)
                    normal = R[:, 2]
                    tilt_deg = float(
                        np.degrees(
                            np.arccos(np.clip(abs(normal[2]), -1.0, 1.0)))
                    )
                    current_R = R
                    current_t = np.asarray(tvec).reshape(3)
            except Exception:
                logger.debug("[%s] preview PnP 예외", robot_id, exc_info=True)

        # capture_quality verdict — 세션 + intrinsic 있을 때만 (없으면 단순 detect).
        verdict: str = "red"
        reasons: list[str] = []
        if st.hand_eye_run_id is not None:
            current_joints_rad: list[float] | None = None
            raw = self._joint_cache.get_raw_motor_positions(
                st.arm_cfgs, robot_id=robot_id
            )
            if raw is not None:
                jc = JointCoordinates()
                current_joints_rad = [
                    jc.motor_to_urdf(raw[cfg.id], cfg, robot_id=robot_id)
                    for cfg in st.arm_cfgs
                ]
            existing_R = [c.R_target2cam for c in st.captures]
            existing_t = [c.t_target2cam for c in st.captures]
            existing_joints: list[list[float]] = []
            if current_joints_rad is not None:
                jc = JointCoordinates()
                for cap in st.captures:
                    try:
                        existing_joints.append(
                            [
                                jc.motor_to_urdf(
                                    cap.motor_positions[cfg.id],
                                    cfg,
                                    robot_id=robot_id,
                                )
                                for cfg in st.arm_cfgs
                            ]
                        )
                    except KeyError:
                        continue
            quality = cq.evaluate_capture_quality(
                detected=detected,
                tilt_deg=tilt_deg,
                current_joints_rad=current_joints_rad,
                current_R_t2c=current_R,
                current_t_t2c=current_t,
                existing_joints_rad=existing_joints,
                existing_R_t2c=existing_R,
                existing_t_t2c=existing_t,
            )
            verdict = quality.verdict
            reasons = list(quality.reasons)
        elif detected:
            verdict = "green"
            reasons = ["세션 시작 전 — [캘 시작] 후 캡처 가능"]
        else:
            verdict = "red"
            reasons = ["보드 미감지"]

        # ChArUco corner pixel + marker outlines (frontend overlay).
        corners_payload: list[list[float]] = []
        if detected and ch_corners is not None:
            corners_payload = [
                [float(c[0]), float(c[1])]
                for c in ch_corners.reshape(-1, 2)
            ]

        marker_outline_payload: list[list[list[float]]] = []
        if marker_corners is not None and len(marker_corners) > 0:
            for m in marker_corners:
                marker_outline_payload.append(
                    [[float(p[0]), float(p[1])]
                     for p in np.asarray(m).reshape(-1, 2)]
                )

        self.publish(
            key_for(Topic.CALIB_HANDEYE_PREVIEW, robot_id),
            {
                "timestamp": time.time(),
                "detected": detected,
                "tilt_deg": tilt_deg,
                "pose_count": len(st.captures),
                "session_active": st.hand_eye_run_id is not None,
                "capture_verdict": verdict,
                "capture_reasons": reasons,
                "corners_2d": corners_payload,
                "marker_outlines": marker_outline_payload,
            },
        )
