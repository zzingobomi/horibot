import logging
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
from dataclasses import dataclass, field

from core.transport.application_node import ApplicationNode
from core.coords.joint_coordinates import JointCoordinates
from core.coords.link_coordinates import LinkCoordinates
from core.robot.robot_registry import RobotRegistry
from core.coords.sag_coordinates import SagCoordinates
from modules.calibration.sag_offsets import SagOffsets
from core.transport.messages.base import EmptyData, ServiceRequest, ServiceResponse
from core.transport.messages.calibration import (
    HandeyeCaptureRes,
    HandeyeCommitRes,
    HandeyeListPosesRes,
    HandeyePoseMeta,
    HandeyePreviewEnableReq,
    HandeyePreviewEnableRes,
    HandeyeResetRes,
    HandeyeStartRes,
    HandeyeUndoLastCaptureRes,
    IntrinsicCaptureRes,
    IntrinsicSaveRes,
    JointOffsetEntry,
    LinkOffsetEntry,
    BeginRefinementReq,
    BeginRefinementRes,
    SagOffsetEntry,
)
from core.transport.topic_map import Service, Topic, topic_for
from core.cache.frame_cache import FrameCache
from core.cache.joint_state_cache import JointStateCache
from modules.motor.motor_config import MotorConfig, load_motor_layout
from modules.camera.stream import frame_to_base64
from modules.calibration.intrinsic import IntrinsicCalibration, IntrinsicResult
from modules.calibration.hand_eye import HandEyeCalibration, HandEyeResult, Pose
from modules.calibration import board as calib_board
from modules.calibration import next_pose_planner
from modules.calibration import thresholds as calib_thresholds
from modules.calibration.calibration_cache import CalibrationCache
from modules.calibration.link_offsets import LinkOffsets
from modules.calibration.loader import CalibrationData, HandEyeData, IntrinsicData
from modules.calibration.persistence_models import (
    CalibrationCaptureRecord,
    CalibrationResultRecord,
    CalibrationRunRecord,
    HandEyeResultRecord,
    IntrinsicResultRecord,
    JointOffsetResultRecord,
    LinkOffsetResultRecord,
    SagOffsetResultRecord,
)
from modules.calibration.result_models import (
    HandEyeResultData,
    IntrinsicResultData,
    JointOffsetResultData,
    LinkOffsetEntry as LinkOffsetResultEntry,
    LinkOffsetResultData,
    SagOffsetResultData,
)
from modules.calibration.storage_client import (
    CalibrationStorageClient,
    load_active_blocking,
)
from modules.kinematics.adapters.pybullet_kinematics import PybulletKinematics
from modules.kinematics.adapters.sag_corrected import SagCorrectedKinematics

logger = logging.getLogger(__name__)


PREVIEW_INTERVAL = 0.2  # 5Hz


def _optional_float(v: object) -> float | None:
    return float(v) if isinstance(v, (int, float)) else None


def _optional_str(v: object) -> str | None:
    return str(v) if v is not None else None


@dataclass
class _RobotState:
    """robot 별 캘리브레이션 상태."""

    arm_cfgs: list[MotorConfig]
    intrinsic: IntrinsicCalibration
    hand_eye: HandEyeCalibration
    kinematics: SagCorrectedKinematics
    last_compute: dict | None = None
    preview_enabled: bool = False
    sigma_history: list[float] = field(default_factory=list)
    # draft run id — [캘 시작] 자리 시 setting, [리셋]/[커밋] 자리 시 None.
    # capture 가 None 이면 fail (사용자가 START 안 한 자리). storage_layer.md §13.
    hand_eye_run_id: int | None = None
    # Phase (handeye_ux_solver_v3_plan.md §2): "collection" = Phase 1 (geometry only,
    # BA 안 돔 — 캡처마다 BA 큐 쌓이는 backlog 방지 + 스펙 "Phase1 RMS/BA 금지").
    # "refinement" = Phase 2 (초기 solve 이후 — auto-BA + σ + observability + gating).
    # START/리셋 시 collection, begin_refinement/compute(초기 solve) 시 refinement.
    phase: str = "collection"


class CalibrationNode(ApplicationNode):
    def __init__(self) -> None:
        super().__init__("calibration_node")

        self._frame_cache = FrameCache()
        self._joint_cache = JointStateCache()

        # robot 별 상태 — intrinsic/hand_eye result 는 부팅 시 _push_calibration 가
        # storage 에서 fetch 후 채움 (file load X). storage 가 SSOT.
        self._states: dict[str, _RobotState] = {}
        for rid in self.enabled_robot_ids:
            arm_cfgs = load_motor_layout(rid).arm
            intrinsic = IntrinsicCalibration()
            # sag joint = robots.yaml::sag_joint_motor_ids (1-based) → arm idx (0-based).
            # 같은 코드로 5축/6축 — sag joint 만 robot config 로 분기 (SSOT).
            sag_arm_indices = [
                m - 1 for m in self._registry.get(rid).sag_joint_motor_ids
            ]
            hand_eye = HandEyeCalibration(
                self._registry.get_fk_chain(rid), sag_arm_indices=sag_arm_indices
            )
            kinematics = self._registry.get_kinematics(rid)
            assert isinstance(kinematics, SagCorrectedKinematics)

            self._states[rid] = _RobotState(
                arm_cfgs=arm_cfgs,
                intrinsic=intrinsic,
                hand_eye=hand_eye,
                kinematics=kinematics,
            )

        self._preview_thread: threading.Thread | None = None
        self._setup_thread: threading.Thread | None = None
        self._storage = CalibrationStorageClient()

        # per-robot single-thread executor — capture service fast path 자체 자체,
        # 자동 BA / observability 자체 자체 자체 background 자체 자체. max_workers=1 자체
        # 자체 자체 자체 BA 자체 자체 자체 직렬화 (latest pose 자체 사용).
        self._ba_executors: dict[str, ThreadPoolExecutor] = {
            rid: ThreadPoolExecutor(
                max_workers=1, thread_name_prefix=f"calib-ba-{rid}"
            )
            for rid in self.enabled_robot_ids
        }

    def _restore_in_progress_handeye(
        self, robot_id: str, st: _RobotState
    ) -> None:
        """부팅 시 storage 의 in_progress hand_eye run 자체 자체 복원.

        있으면 hand_eye_run_id setting + capture rows 자체 in-memory Pose 자체 자체.
        없으면 no-op (사용자가 [캘 시작] 누를 때까지 빈 상태).
        """
        existing = self._storage.get_in_progress_run(robot_id, "hand_eye")
        if existing is None:
            return
        run, captures = existing
        st.hand_eye_run_id = run.id
        self._restore_poses_from_captures(robot_id, st, captures)
        logger.info(
            "[%s] in_progress hand_eye run 복원 (run_id=%d, %d장)",
            robot_id, run.id, len(captures),
        )

    def _restore_poses_from_captures(
        self,
        robot_id: str,
        st: _RobotState,
        captures: list[CalibrationCaptureRecord],
    ) -> None:
        """capture rows → in-memory Pose 객체. id 는 pose_index 그대로 사용."""
        joints = JointCoordinates()
        st.hand_eye.poses.clear()
        for cap in captures:
            if cap.board_in_cam is None or len(cap.joint_angles) != len(st.arm_cfgs):
                logger.warning(
                    "[%s] capture row 손상 — skip (pose_index=%d)",
                    robot_id, cap.pose_index,
                )
                continue
            raw_dict = {
                cfg.id: joints.urdf_to_motor(rad, cfg, robot_id=robot_id)
                for cfg, rad in zip(st.arm_cfgs, cap.joint_angles)
            }
            board_T = np.asarray(cap.board_in_cam, dtype=np.float64)
            R = board_T[:3, :3]
            t = board_T[:3, 3].reshape(3, 1)
            st.hand_eye.poses.append(
                Pose(
                    raw_motor_positions=raw_dict,
                    R_target2cam=R,
                    t_target2cam=t,
                    id=cap.pose_index,
                )
            )
        st.hand_eye._next_id = (
            max((p.id for p in st.hand_eye.poses), default=-1) + 1
        )

    def _setup_runtime_calibration(self) -> None:
        """부팅 시 background thread. Storage 대기 → 5종 fetch → 소비자에 push.

        docs/storage_layer.md §7 ownership layer — Calibration Service (본 노드)
        만 storage 앎. 다른 노드 / Coordinates / PybulletKinematics 는 본 push 받음.
        """
        for rid in self.enabled_robot_ids:
            try:
                self._push_calibration(rid)
                logger.info("[%s] runtime calibration push 완료", rid)
            except Exception:
                logger.exception("[%s] runtime calibration setup 실패", rid)

    def _push_calibration(self, robot_id: str) -> None:
        """Atomic snapshot — 5종 다 fetch *후* push. partial state 차단
        (docs/storage_layer.md §7). 한 kind fetch fail 시 전체 보류.
        """
        # ─── Phase 1: fetch all (storage 대기, partial 시점 X) ─────
        joint_rec = load_active_blocking(robot_id, "joint_offset")
        link_rec = load_active_blocking(robot_id, "link_offset")
        sag_rec = load_active_blocking(robot_id, "sag")
        intrinsic_rec = load_active_blocking(robot_id, "intrinsic")
        hand_eye_rec = load_active_blocking(robot_id, "hand_eye")

        # ─── Phase 2: snapshot 객체 만들기 (storage 의존 X) ─────────
        joint_offsets = (
            dict(joint_rec.result_data.offsets)
            if joint_rec is not None and joint_rec.kind == "joint_offset"
            else {}
        )
        if link_rec is not None and link_rec.kind == "link_offset":
            link_offsets = LinkOffsets(
                trans={
                    e.joint_id: np.array(e.trans_m, dtype=np.float64)
                    for e in link_rec.result_data.offsets
                },
                rot={
                    e.joint_id: np.array(e.rot_rad, dtype=np.float64)
                    for e in link_rec.result_data.offsets
                },
            )
        else:
            link_offsets = LinkOffsets()
        sag_offsets = (
            SagOffsets(k_rad_per_m=dict(sag_rec.result_data.k_rad_per_m))
            if sag_rec is not None and sag_rec.kind == "sag"
            else SagOffsets()
        )
        intrinsic = (
            IntrinsicData(
                camera_matrix=np.array(intrinsic_rec.result_data.camera_matrix, dtype=np.float64),
                dist_coeffs=np.array(intrinsic_rec.result_data.dist_coeffs, dtype=np.float64),
                image_size=(
                    tuple(intrinsic_rec.result_data.image_size)  # type: ignore[arg-type]
                    if intrinsic_rec.result_data.image_size is not None
                    else None
                ),
            )
            if intrinsic_rec is not None and intrinsic_rec.kind == "intrinsic"
            else None
        )
        hand_eye = (
            HandEyeData(
                R=np.array(hand_eye_rec.result_data.R_cam2gripper, dtype=np.float64),
                t=np.array(hand_eye_rec.result_data.t_cam2gripper, dtype=np.float64),
            )
            if hand_eye_rec is not None and hand_eye_rec.kind == "hand_eye"
            else None
        )

        # ─── Phase 3: atomic push (consumer hot path 가 wait_ready 라 partial 노출 X) ─
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
        cache.set(robot_id, CalibrationData(intrinsic=intrinsic, hand_eye=hand_eye))

        # _states[rid] 의 in-memory result 도 storage record 로 sync — preview loop
        # / recommendation 가 st.intrinsic.result / st.hand_eye.result 직접 읽음.
        # CalibrationCache 만 채우면 부팅 시 _states 가 None 으로 남음 (storage refactor
        # 의 빈 자리 — file load 폐기 후 한 번 동기화 path 필요).
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
        if (
            hand_eye is not None
            and hand_eye_rec is not None
            and isinstance(hand_eye_rec.result_data, HandEyeResultData)
        ):
            st.hand_eye.result = HandEyeResult(
                R_cam2gripper=hand_eye.R,
                t_cam2gripper=hand_eye.t,
                method=hand_eye_rec.result_data.method,
            )
        else:
            st.hand_eye.result = None

        # ─── Phase 4: ready signal — consumer hot path 의 wait_ready 풀림 ──
        cache.signal_ready(robot_id)

    def start(self) -> None:
        for rid in self.enabled_robot_ids:
            self._frame_cache.subscribe(self, robot_id=rid)
            # 내부 캘리브레이션
            self.create_service(
                topic_for(Service.CALIB_INTRINSIC_CAPTURE, rid),
                EmptyData,
                IntrinsicCaptureRes,
                lambda req, _rid=rid: self._srv_intrinsic_capture(req, _rid),
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
                topic_for(Service.CALIB_HANDEYE_BEGIN_REFINEMENT, rid),
                BeginRefinementReq,
                BeginRefinementRes,
                lambda req, _rid=rid: self._srv_handeye_begin_refinement(req, _rid),
            )
            # Draft run flow — 사용자 [캘 시작] / [되돌리기].
            self.create_service(
                topic_for(Service.CALIB_HANDEYE_START, rid),
                EmptyData,
                HandeyeStartRes,
                lambda req, _rid=rid: self._srv_handeye_start(req, _rid),
            )
            self.create_service(
                topic_for(Service.CALIB_HANDEYE_UNDO_LAST_CAPTURE, rid),
                EmptyData,
                HandeyeUndoLastCaptureRes,
                lambda req, _rid=rid: self._srv_handeye_undo_last_capture(
                    req, _rid
                ),
            )

        super().start()
        self._joint_cache.subscribe(self)
        self._preview_thread = threading.Thread(
            target=self._preview_loop,
            daemon=True,
            name="calib-preview",
        )
        self._preview_thread.start()

        # 부팅 시 in_progress hand_eye run 복원 — DB SSOT. 사용자가 캘 진행
        # 중에 backend 재시작 / browser reload 한 자리 자리 자리 자리 자리 자리 복원.
        for rid, st in self._states.items():
            try:
                self._restore_in_progress_handeye(rid, st)
            except Exception:
                logger.exception("[%s] in_progress 복원 실패", rid)

        # 부팅 시 storage 에서 5종 fetch + 소비자에 push (background thread —
        # main start 안 막힘. storage 늦게 떠도 retry).
        self._setup_thread = threading.Thread(
            target=self._setup_runtime_calibration,
            daemon=True,
            name="calib-setup",
        )
        self._setup_thread.start()

        logger.info("CalibrationNode 시작 (robots=%s)", self.enabled_robot_ids)

    def stop(self) -> None:
        # 진행 중 BA 자체 자체 자체 자체 wait — process 종료 자체 자체 자체 자체.
        for rid, ex in self._ba_executors.items():
            try:
                ex.shutdown(wait=False, cancel_futures=True)
            except Exception:
                logger.exception("[%s] BA executor shutdown 오류", rid)
        super().stop()

    # ─── 이미지 캡처 ─────────────────────────────────────────

    def _srv_intrinsic_capture(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[IntrinsicCaptureRes]:
        st = self._states[robot_id]

        ret, frame = self._frame_cache.get_frame(robot_id=robot_id)
        if not ret or frame is None:
            return ServiceResponse(
                success=False,
                message="카메라 프레임을 읽을 수 없습니다",
                data=None,
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

        # Storage commit + activate. docs/storage_layer.md §7 SSOT.
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
            run_id, result_ids = self._storage.commit(run, [record], [])
            self._storage.activate(result_ids[0])
        except Exception as e:
            logger.exception("[%s] intrinsic storage commit 실패", robot_id)
            return ServiceResponse(
                success=False, message=f"storage commit 실패: {e}", data=None
            )

        # in-memory push — CalibrationCache 의 IntrinsicData 갱신
        self._push_calibration(robot_id)
        logger.info(
            "[%s] intrinsic COMMIT: run_id=%d, rms=%.4f, captured=%d",
            robot_id,
            run_id,
            result.rms_error,
            result.captured_count,
        )

        return ServiceResponse(
            success=True,
            message=f"storage commit (run_id={run_id}, rms={result.rms_error:.4f})",
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
        joint_limits = st.kinematics.joint_limits(len(arm_motor_ids))
        use_physical_sag = mode == "physical_sag"
        use_extended_ba = mode in ("physical_sag", "extended")
        diag = st.hand_eye.compute_with_diagnostics(
            fk_fn=st.kinematics.fk_to_matrix,
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

    def _publish_observability_state(self, robot_id: str, st) -> None:
        """매 capture 후 자세 분포 진단 publish.

        verdict (A / B / mid) 만 frontend 안내. 4 metric 숫자는 backend 진단용.
        - A: 다양성 충분 → 사용자 안내 "캘 가능, 추가 자세는 σ 개선 가능"
        - B: 구조적 부족 → "보드 위치 / 거리 변경 권고"
        - mid: 중립
        """
        from modules.calibration import observability as _obs

        poses = st.hand_eye.poses
        if len(poses) < 3:
            return
        R_arr = np.array([p.R_target2cam for p in poses], dtype=np.float64)
        # raw_motor_positions 는 dict{motor_id: raw} — arm 순서 array 로 변환
        # (이전 np.array(dict) → TypeError 로 geometry observability 가 죽던 버그 fix).
        raw_arr = np.array(
            [[p.raw_motor_positions[c.id] for c in st.arm_cfgs] for p in poses],
            dtype=np.float64,
        )
        cfg = self._registry.get(robot_id)
        # motor_id (1-based, yaml SSOT) → array index (0-based, raw[:, axis])
        rep = _obs.analyze_pose_data(
            R_arr, raw_arr, wrist_roll_axis=cfg.wrist_roll_motor_id - 1
        )
        v = rep.verdict()
        # verdict 첫 글자만 ('A'/'B'/'mid')
        v_short = "A" if v.startswith("A") else ("B" if v.startswith("B") else "mid")

        self.publish(
            topic_for(Topic.CALIB_HANDEYE_OBSERVABILITY, robot_id),
            {
                "timestamp": time.time(),
                "pose_count": rep.n_poses,
                "axis_spread_deg": rep.axis_spread_deg,
                "tilt_min_deg": rep.tilt_min_deg,
                "tilt_max_deg": rep.tilt_max_deg,
                "tilt_std_deg": rep.tilt_std_deg,
                "tilt_in_range_count": rep.tilt_in_range_count,
                "rotation_axis_ratio": rep.rotation_axis_ratio,
                "wrist_roll_range_raw": rep.wrist_roll_range_raw,
                "verdict": v_short,
            },
        )

    def _publish_param_observability(self, robot_id: str, diag: dict) -> None:
        """per-parameter observability + staged gating 결과 publish (physical_sag 만).

        diag["param_observability"] = {n_poses, scores, verdicts, unlocked} 또는 None.
        frontend 가 블록별 색 dot 으로 "어느 보정값이 잘 잡혔나" 표시.
        """
        po = diag.get("param_observability")
        if not po:
            return
        self.publish(
            topic_for(Topic.CALIB_HANDEYE_PARAM_OBSERVABILITY, robot_id),
            {
                "timestamp": time.time(),
                "pose_count": int(po.get("n_poses", 0)),
                "scores": po.get("scores", {}),
                "verdicts": po.get("verdicts", {}),
                "unlocked": po.get("unlocked", []),
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
                "joint_offset_estimated": diag.get("joint_offset_estimated", False),
                "link_offset_estimated": diag.get("link_offset_estimated", False),
                "sag_offset_estimated": diag.get("sag_offset_estimated", False),
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
            R_g2b, t_g2b = st.kinematics.fk_to_matrix(angles)
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

        raw_positions = self._joint_cache.get_raw_motor_positions(
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
            return ServiceResponse(success=False, message="포즈 추정 실패", data=None)

        # PnP 품질 gate — trauma 차단 (코너 가림 / blur / 광량 / board 미세 움직임이 만든
        # 안 좋은 자세 자체를 거부). thresholds.HANDEYE_PNP_RMS_REJECT_PX 초과 시 capture
        # 받지 않음. 사용자는 "다시 시도" 안내만 봄 (숫자 노출 X).
        projected, _ = cv2.projectPoints(
            obj_pts,
            rvec,
            tvec,
            st.intrinsic.result.camera_matrix,
            st.intrinsic.result.dist_coeffs,
        )
        reproj_err = np.linalg.norm(
            projected.reshape(-1, 2) - img_pts.reshape(-1, 2), axis=1
        )
        reproj_rms_px = float(np.sqrt(np.mean(reproj_err**2)))
        if reproj_rms_px > calib_thresholds.HANDEYE_PNP_RMS_REJECT_PX:
            logger.info(
                "[%s] PnP 품질 부족 (RMS=%.2fpx > %.2fpx) — capture 거부",
                robot_id,
                reproj_rms_px,
                calib_thresholds.HANDEYE_PNP_RMS_REJECT_PX,
            )
            return ServiceResponse(
                success=False,
                message=(
                    "이미지 품질이 부족해 자세 추정이 부정확합니다. "
                    "보드가 또렷이 보이는 자세에서 다시 시도해주세요."
                ),
                data=HandeyeCaptureRes(
                    detected=False, pose_count=len(st.hand_eye.poses)
                ),
            )

        # draft run 필수 — 사용자가 [캘 시작] 안 누른 자리 자리 자체 자체 자체.
        if st.hand_eye_run_id is None:
            return ServiceResponse(
                success=False,
                message=(
                    "캘 세션이 시작 안 됨 — 먼저 [캘 시작] 을 눌러주세요"
                ),
                data=HandeyeCaptureRes(
                    detected=False, pose_count=len(st.hand_eye.poses)
                ),
            )

        R_t2c, _ = cv2.Rodrigues(rvec)
        pose_index = len(st.hand_eye.poses)

        # capture record 자체 자체 storage 자체 박기 — in-memory 자체 자체 박기 *전*.
        # 자체 자체 storage fail 자체 자체 자체 자체 in-memory 자체 자체 자체 일관 (consistent fail).
        joints = JointCoordinates()
        joint_angles_rad = [
            joints.motor_to_urdf(raw_positions[cfg.id], cfg, robot_id=robot_id)
            for cfg in st.arm_cfgs
        ]
        board_in_cam = np.eye(4)
        board_in_cam[:3, :3] = R_t2c
        board_in_cam[:3, 3] = np.asarray(tvec).reshape(3)
        try:
            self._storage.append_capture(
                CalibrationCaptureRecord(  # type: ignore[call-arg]
                    run_id=st.hand_eye_run_id,
                    pose_index=pose_index,
                    joint_angles=joint_angles_rad,
                    board_in_cam=board_in_cam.tolist(),
                )
            )
        except Exception as e:
            logger.exception("[%s] capture storage append 실패", robot_id)
            return ServiceResponse(
                success=False,
                message=f"storage append 실패: {e}",
                data=HandeyeCaptureRes(
                    detected=False, pose_count=len(st.hand_eye.poses)
                ),
            )

        st.hand_eye.add_pose(
            Pose(
                raw_motor_positions=raw_positions,
                R_target2cam=R_t2c,
                t_target2cam=tvec,
            )
        )

        st.last_compute = None

        # Phase 1 (collection): geometry observability 만 publish — BA 안 돔.
        #   캡처마다 BA 큐 쌓여 backlog 되는 문제 방지 + 스펙 "Phase1 RMS/BA 금지".
        # Phase 2 (refinement): 자동 BA + σ + observability + gating background.
        #   service response 는 fast (~100ms), frontend 가 topic 으로 async 갱신.
        if (
            st.phase == "refinement"
            and len(st.hand_eye.poses) >= calib_thresholds.MIN_POSES_FOR_COMPUTE
        ):
            self._ba_executors[robot_id].submit(
                self._auto_ba_and_publish, robot_id
            )
        else:
            # Phase 1 — solver-free geometry 진단만 (가볍다, inline).
            try:
                self._publish_observability_state(robot_id, st)
            except Exception:
                logger.exception("[%s] Phase1 geometry observability 실패", robot_id)

        return ServiceResponse(
            success=True,
            message=f"포즈 기록됨 ({len(st.hand_eye.poses)}개)",
            data=HandeyeCaptureRes(detected=True, pose_count=len(st.hand_eye.poses)),
        )

    def _auto_ba_and_publish(self, robot_id: str) -> None:
        """capture 마다 background 자체 자체 BA + observability + 추천. capture service
        자체 자체 fast response 자체. 같은 robot 자체 capture 자체 자체 자체 자체 queue 자체
        ThreadPoolExecutor(max_workers=1) 자체 자체 직렬화."""
        st = self._states[robot_id]
        if len(st.hand_eye.poses) < calib_thresholds.MIN_POSES_FOR_COMPUTE:
            return
        try:
            auto_diag = self._run_ba_and_stash(robot_id, mode="physical_sag")
            if auto_diag is not None:
                self._publish_sigma_state(robot_id, auto_diag)
                self._publish_param_observability(robot_id, auto_diag)
                self._publish_saturate_state(robot_id, auto_diag)
                self._publish_recommendations(robot_id)
        except Exception:
            logger.exception("[%s] 자동 BA 실패 (background)", robot_id)
        try:
            self._publish_observability_state(robot_id, st)
        except Exception:
            logger.exception("[%s] observability 실패 (background)", robot_id)

    def _srv_handeye_reset(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[HandeyeResetRes]:
        """draft run + captures cascade 삭제 + in-memory 비우기. 사용자 [리셋].

        이후 사용자는 [캘 시작] 재호출 필요. storage_layer.md §13.
        """
        st = self._states[robot_id]
        if st.hand_eye_run_id is not None:
            try:
                self._storage.delete_run(st.hand_eye_run_id)
            except Exception:
                logger.exception(
                    "[%s] draft run delete 실패 (run_id=%d)",
                    robot_id, st.hand_eye_run_id,
                )
        st.hand_eye.reset()
        st.hand_eye_run_id = None
        st.last_compute = None
        st.sigma_history.clear()
        st.phase = "collection"
        return ServiceResponse(
            success=True,
            message="Hand-Eye 세션 초기화됨",
            data=HandeyeResetRes(pose_count=0),
        )

    def _srv_handeye_start(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[HandeyeStartRes]:
        """[캘 시작] — draft run 생성. 기존 in_progress 가 있으면 reject — frontend
        는 부팅 시 GET_IN_PROGRESS 로 확인 후 호출하거나 [리셋] 먼저 누름."""
        st = self._states[robot_id]
        if st.hand_eye_run_id is not None:
            return ServiceResponse(
                success=False,
                message=(
                    f"이미 진행 중 세션 있음 (run_id={st.hand_eye_run_id}). "
                    "이어하기 또는 [리셋] 후 다시 시작."
                ),
                data=None,
            )
        # 안전 가드 — DB 자체 자체 확인. in-memory 상태 자체 자체 자체 자체 disagreement 자체 자체
        # 자체 자체 자체 (예: 다른 process 자체 자체 만든 in_progress).
        existing = self._storage.get_in_progress_run(robot_id, "hand_eye")
        if existing is not None:
            run, captures = existing
            st.hand_eye_run_id = run.id
            self._restore_poses_from_captures(robot_id, st, captures)
            return ServiceResponse(
                success=False,
                message=(
                    f"DB 에 기존 진행 중 세션 있음 (run_id={run.id}, "
                    f"{len(captures)}장 복원). 이어하기 또는 [리셋] 후 다시 시작."
                ),
                data=None,
            )

        now = time.time()
        run = CalibrationRunRecord(
            robot_id=robot_id,
            started_at=now,
            algorithm="hand_eye",
            kind="hand_eye",
        )
        try:
            run_id = self._storage.new_run(run)
        except Exception as e:
            logger.exception("[%s] new_run 실패", robot_id)
            return ServiceResponse(
                success=False, message=f"storage new_run 실패: {e}", data=None
            )

        # 새 세션이라 in-memory 자체 자체 비우기 (이전 session 잔재 제거).
        st.hand_eye.reset()
        st.hand_eye_run_id = run_id
        st.last_compute = None
        st.sigma_history.clear()
        st.phase = "collection"  # Phase 1 시작 — 초기 solve 전엔 geometry only

        logger.info(
            "[%s] Hand-Eye 세션 시작 (run_id=%d)", robot_id, run_id
        )
        return ServiceResponse(
            success=True,
            message=f"Hand-Eye 세션 시작 (run_id={run_id})",
            data=HandeyeStartRes(run_id=run_id, pose_count=0),
        )

    def _srv_handeye_undo_last_capture(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[HandeyeUndoLastCaptureRes]:
        """[되돌리기] — 마지막 capture 1장 삭제 + in-memory pop."""
        st = self._states[robot_id]
        if st.hand_eye_run_id is None or not st.hand_eye.poses:
            return ServiceResponse(
                success=True,
                message="삭제할 capture 없음",
                data=HandeyeUndoLastCaptureRes(
                    deleted=False, pose_count=len(st.hand_eye.poses)
                ),
            )

        try:
            deleted_idx = self._storage.delete_last_capture(st.hand_eye_run_id)
        except Exception as e:
            logger.exception("[%s] storage delete_last_capture 실패", robot_id)
            return ServiceResponse(
                success=False, message=f"storage delete 실패: {e}", data=None
            )

        if deleted_idx is None:
            # DB 와 in-memory 가 어긋남 — 안전 동기화: in-memory 도 비움.
            st.hand_eye.reset()
            return ServiceResponse(
                success=True,
                message="DB capture 없음 — in-memory 도 동기화 reset",
                data=HandeyeUndoLastCaptureRes(deleted=False, pose_count=0),
            )

        # in-memory 마지막 pop + next_id 보정.
        if st.hand_eye.poses:
            st.hand_eye.poses.pop()
            st.hand_eye._next_id = (
                max((p.id for p in st.hand_eye.poses), default=-1) + 1
            )
        st.last_compute = None
        return ServiceResponse(
            success=True,
            message=f"capture #{deleted_idx} 삭제됨",
            data=HandeyeUndoLastCaptureRes(
                deleted=True, pose_count=len(st.hand_eye.poses)
            ),
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
                poses=poses,
                pose_count=len(st.hand_eye.poses),
                run_id=st.hand_eye_run_id,
            ),
        )

    def _srv_handeye_compute(self, req: dict, robot_id: str) -> dict:
        # 명시 COMPUTE = 초기 solve → Phase 2 (refinement). 이후 capture 는 auto-BA.
        self._states[robot_id].phase = "refinement"
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
        self._publish_param_observability(robot_id, diag)
        return {
            "success": True,
            "message": f"compute 완료 (poses={diag['pose_count']})",
            "data": diag,
        }

    def _srv_handeye_commit(
        self, _req: ServiceRequest[EmptyData], robot_id: str
    ) -> ServiceResponse[HandeyeCommitRes]:
        """BA 결과 → finalize_run (in_progress→success + Result rows INSERT) → activate.

        draft run + captures 가 capture 단계 동안 storage 에 누적됨. commit 은 그 run
        finalize: status flip + Result rows INSERT + 새 result activate + invalidation publish.
        끝나면 session 종료 — hand_eye_run_id None, in-memory 비움, 다음 [캘 시작]
        대기.
        """
        st = self._states[robot_id]
        if st.last_compute is None or st.hand_eye.result is None:
            return ServiceResponse(
                success=False, message="먼저 COMPUTE를 실행하세요", data=None
            )
        if st.hand_eye_run_id is None:
            return ServiceResponse(
                success=False,
                message="진행 중 세션 없음 — [캘 시작] 후 capture/compute 필요",
                data=None,
            )

        method = st.hand_eye.result.method
        now = time.time()
        joint_estimated = bool(st.last_compute.get("joint_offset_estimated"))
        link_estimated = bool(st.last_compute.get("link_offset_estimated"))
        sag_estimated = bool(st.last_compute.get("sag_offset_estimated"))

        # ─── ResultRecord 빌드 (hand_eye 항상, 다른 kind 는 estimated 시) ──
        results: list[CalibrationResultRecord] = []

        # hand_eye — 항상 포함
        R = st.hand_eye.result.R_cam2gripper
        t = st.hand_eye.result.t_cam2gripper
        results.append(
            HandEyeResultRecord(  # type: ignore[arg-type]
                run_id=0,  # storage 가 채움
                robot_id=robot_id,
                created_at=now,
                sigma_rot=_optional_float(st.last_compute.get("sigma_rot_deg")),
                sigma_t=_optional_float(st.last_compute.get("sigma_t_mm")),
                result_data=HandEyeResultData(
                    R_cam2gripper=R.tolist(),
                    t_cam2gripper=np.asarray(t).reshape(3, 1).tolist(),
                    method=method,
                ),
            )
        )

        applied_joint: dict[int, float] = {}
        if joint_estimated:
            applied_joint = dict(st.last_compute["_joint_absolute_by_id"])
            results.append(
                JointOffsetResultRecord(  # type: ignore[arg-type]
                    run_id=0,
                    robot_id=robot_id,
                    created_at=now,
                    result_data=JointOffsetResultData(
                        offsets=dict(applied_joint),
                        method=method,
                    ),
                )
            )

        link_entries: list[LinkOffsetResultEntry] = []
        if link_estimated:
            trans_list = st.last_compute.get("link_trans_delta", [])
            rot_list = st.last_compute.get("link_rot_delta", [])
            rot_by_id = {int(e["motor_id"]): e for e in rot_list}
            for tr in trans_list:
                jid = int(tr["motor_id"])
                rt = rot_by_id.get(jid, {"rx_rad": 0.0, "ry_rad": 0.0, "rz_rad": 0.0})
                link_entries.append(
                    LinkOffsetResultEntry(
                        joint_id=jid,
                        trans_m=[float(tr["x_m"]), float(tr["y_m"]), float(tr["z_m"])],
                        rot_rad=[
                            float(rt["rx_rad"]),
                            float(rt["ry_rad"]),
                            float(rt["rz_rad"]),
                        ],
                    )
                )
            results.append(
                LinkOffsetResultRecord(  # type: ignore[arg-type]
                    run_id=0,
                    robot_id=robot_id,
                    created_at=now,
                    result_data=LinkOffsetResultData(
                        offsets=link_entries,
                        method=method,
                    ),
                )
            )

        sag_dict: dict[int, float] = {}
        if sag_estimated:
            sag_delta_list = st.last_compute.get("sag_offset_delta", [])
            sag_dict = {
                int(e["motor_id"]): float(e["k_rad_per_m"]) for e in sag_delta_list
            }
            results.append(
                SagOffsetResultRecord(  # type: ignore[arg-type]
                    run_id=0,
                    robot_id=robot_id,
                    created_at=now,
                    result_data=SagOffsetResultData(
                        k_rad_per_m=dict(sag_dict),
                        method=method,
                    ),
                )
            )

        # ─── draft run finalize — status flip + result rows INSERT (atomic) ──
        try:
            result_ids = self._storage.finalize_run(
                st.hand_eye_run_id,
                results,
                capture_residuals=None,
            )
        except Exception as e:
            logger.exception("[%s] storage finalize 실패", robot_id)
            return ServiceResponse(
                success=False, message=f"storage finalize 실패: {e}", data=None
            )
        run_id = st.hand_eye_run_id

        # ─── ACTIVATE 모든 새 result — storage_node 가 invalidation publish ──
        for rid_ in result_ids:
            try:
                self._storage.activate(rid_)
            except Exception:
                logger.exception("[%s] activate 실패 (result_id=%d)", robot_id, rid_)

        # ─── in-memory push — 방금 활성화된 새 record 가 fresh fetch ─────
        self._push_calibration(robot_id)

        # log + 응답 위해 sigma 값 자체 자체 미리 추출 (세션 종료 자체 자체 자체 자체 자체 None 되기 전에).
        sigma_rot_log = st.last_compute.get("sigma_rot_deg")
        sigma_t_log = st.last_compute.get("sigma_t_mm")

        # ─── 세션 종료 — 다음 [캘 시작] 까지 빈 상태 ────────────────
        st.hand_eye_run_id = None
        st.hand_eye.reset()
        st.last_compute = None
        st.sigma_history.clear()

        # link offset 적용은 PybulletKinematics URDF patch 자리 — 부팅 시 1회.
        # 따라서 link estimated 면 재시작 필요 (sag 는 runtime cache 갱신만으로 OK).
        restart_required = link_estimated

        logger.info(
            "[%s] COMMIT: run_id=%d, result_ids=%s, sigma_rot=%s, sigma_t=%s",
            robot_id,
            run_id,
            result_ids,
            sigma_rot_log,
            sigma_t_log,
        )

        msg_parts = [f"storage commit (run_id={run_id})"]
        if joint_estimated:
            applied_deg = {i: round(float(np.degrees(v)), 3) for i, v in applied_joint.items()}
            msg_parts.append(f"joint_offsets (deg={applied_deg})")
        if link_estimated:
            msg_parts.append(f"link_offsets (n={len(link_entries)}, 재시작 필요)")
        if sag_estimated:
            msg_parts.append(f"sag_offsets (n={len(sag_dict)}, 즉시 적용)")

        return ServiceResponse(
            success=True,
            message=" + ".join(msg_parts),
            data=HandeyeCommitRes(
                method=method,
                joint_offsets_applied=joint_estimated,
                joint_offsets=[
                    JointOffsetEntry(motor_id=int(mid), offset_rad=float(off))
                    for mid, off in sorted(applied_joint.items())
                ],
                link_offsets_applied=link_estimated,
                link_offsets=[
                    LinkOffsetEntry(
                        motor_id=e.joint_id,
                        trans_m=list(e.trans_m),
                        rot_rad=list(e.rot_rad),
                    )
                    for e in link_entries
                ],
                sag_offsets_applied=sag_estimated,
                sag_offsets=[
                    SagOffsetEntry(motor_id=int(mid), k_rad_per_m=float(k))
                    for mid, k in sorted(sag_dict.items())
                ],
                restart_required=restart_required,
            ),
        )

    # ─── 초기 solve (Phase 1 → 2) ─────────────────────────────
    def _srv_handeye_begin_refinement(
        self, _req: ServiceRequest[BeginRefinementReq], robot_id: str
    ) -> ServiceResponse[BeginRefinementRes]:
        """초기 hand-eye solve = Phase 1(수집) → Phase 2(정밀화) 전이.

        여러 BA mode (standard / extended / physical_sag) 를 시도해 가장 좋은 σ 채택
        — local minimum escape (같은 데이터, 다른 모델 가정). 사용자가 [자동 추천
        시작] 누르면 (충분히 수집 후) 호출 → Phase 2 진입 (refinement).
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
                    "[%s] begin_refinement mode=%s 실패: %s", robot_id, mode, e
                )

        if best_diag is None:
            return ServiceResponse(
                success=False, message="모든 BA mode 실패", data=None
            )

        # 초기 solve → Phase 2 (refinement). 이후 capture 는 auto-BA.
        st.phase = "refinement"
        # best 결과 last_compute 자체 자리 + topic publish
        st.last_compute = best_diag
        self._publish_sigma_state(robot_id, best_diag)
        self._publish_param_observability(robot_id, best_diag)
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
                f"초기 solve 완료: n_converged={n_converged}/{len(modes)}, "
                f"best σ_rot={best_rot}°"
            ),
            data=BeginRefinementRes(
                n_tried=len(modes),
                n_converged=n_converged,
                sigma_rot_deg=float(best_rot) if best_rot is not None else None,
                sigma_t_mm=float(best_t) if best_t is not None else None,
                improvement_rot_deg=improvement_rot,
                improvement_t_mm=improvement_t,
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
          - "sigma_sufficient_and_diverse" ─ σ + 다양성 둘 다 충족 → COMMIT 권장
          - "sigma_sufficient_but_narrow"  ─ σ 좋은데 자세 다양성 부족 → 부족 axis 변주 캡처

        frontend NextPoseCard 가 분기 별 메시지 표시 + COMMIT 가이드.
        """
        st = self._states[robot_id]
        empty = {"recommendations": [], "no_candidates_reason": "insufficient_poses"}

        current = self._joint_cache.get_joint_angles_rad(st.arm_cfgs, robot_id=robot_id)
        if current is None:
            return empty
        arm_motor_ids = [cfg.id for cfg in st.arm_cfgs]
        joint_limits = st.kinematics.joint_limits(len(arm_motor_ids))

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
        fk_fn = st.kinematics.fk_to_matrix

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

        # IK 함수 wrapper — Kinematics Protocol 의 ik() 시그니처 그대로.
        def _ik(
            target_position,
            target_quaternion,
            current_joint_angles,
        ):
            return st.kinematics.ik(
                target_position, target_quaternion, current_joint_angles
            )

        # 사용자가 명시 신호 ([👎]) 로 fail 표시한 추천 ID set — 다음 추천 시 제외.
        excluded_ids: set[str] = set()  # [👎] 제거됨 — 항상 빈 set

        # 기존 캡처 자세 list (다양성 score 용 — joint_perturbation strategy 가 사용).
        # raw → rad 단순 변환 (joint_offset 적용은 다양성 score 에 영향 X).
        from core import units as _units

        # raw_motor_positions 키는 1-based motor id (cfg.id) — range(len) 0-based
        # 인덱싱은 KeyError(0). cfg.id 로 조회 (이전 버그: auto-BA 에선 try/except 로
        # 삼켜졌지만 explicit COMPUTE 에선 크래시).
        existing_ja = [
            [_units.raw_to_rad(int(p.raw_motor_positions[cfg.id]))
             for cfg in st.arm_cfgs]
            for p in st.hand_eye.poses
        ]

        # Strategy 선택 — robots.yaml::pose_recommend_strategy SSOT.
        # OMX-F (5DOF) = joint_perturbation, SO-101 (6DOF) = geometry.
        robot_cfg = RobotRegistry().get(robot_id)
        strategy = next_pose_planner.make_strategy(
            robot_cfg.pose_recommend_strategy
        )

        # FK wrapper — joint_sample 의 visibility forward 용.
        def _fk(angles):
            R, t = st.kinematics.fk_to_matrix(list(angles))
            return np.asarray(R), np.asarray(t).reshape(3)

        ctx = next_pose_planner.RecommendContext(
            current_joint_angles_rad=list(current),
            arm_motor_ids=arm_motor_ids,
            joint_limits_rad=joint_limits,
            fk_fn=_fk,
            ik_fn=_ik,
            board_corners_base=board_corners_base,
            hand_eye_R=R_c2g,
            hand_eye_t=t_c2g,
            outward_hint=outward_hint,
            visibility_check=_check,
            existing_joint_angles=existing_ja,
            excluded_ids=excluded_ids,
        )
        result = strategy.recommend(ctx)

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

    def _add_capture_quality(
        self,
        payload: dict,
        robot_id: str,
        st: "_RobotState",
        detected: bool,
        tilt_deg: float | None,
        cur_R: np.ndarray | None,
        cur_t: np.ndarray | None,
    ) -> None:
        """Phase 1 Traffic Light — 현재 pose vs 기존 캡처 diversity → G/Y/R.

        verdict/reasons 를 preview payload 에 추가. 토크오프 이동 중 실시간 안내.
        """
        from core.units import raw_to_rad
        from modules.calibration.capture_quality import evaluate_capture_quality

        cur_joints = self._joint_cache.get_joint_angles_rad_uncorrected(
            st.arm_cfgs, robot_id=robot_id
        )
        existing_joints: list[list[float]] = []
        existing_R: list[np.ndarray] = []
        existing_t: list[np.ndarray] = []
        for p in st.hand_eye.poses:
            try:
                ej = [
                    raw_to_rad(int(p.raw_motor_positions[c.id]), reverse=c.reverse)
                    for c in st.arm_cfgs
                ]
            except KeyError:
                continue
            existing_joints.append(ej)
            existing_R.append(np.asarray(p.R_target2cam, dtype=np.float64))
            existing_t.append(np.asarray(p.t_target2cam, dtype=np.float64).reshape(3))

        q = evaluate_capture_quality(
            detected=detected,
            tilt_deg=tilt_deg,
            current_joints_rad=cur_joints,
            current_R_t2c=cur_R,
            current_t_t2c=cur_t,
            existing_joints_rad=existing_joints,
            existing_R_t2c=existing_R,
            existing_t_t2c=existing_t,
        )
        payload["capture_verdict"] = q.verdict
        payload["capture_reasons"] = q.reasons

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
                    ch_corners, ch_ids, m_corners, m_ids = calib_board.detect_full(gray)
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
                            markers_payload.append({"corners": pts, "id": int(mid[0])})
                        if markers_payload:
                            payload["markers"] = markers_payload

                    cur_R: np.ndarray | None = None
                    cur_t: np.ndarray | None = None
                    cur_tilt: float | None = None
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
                        payload["coverage_ratio"] = (bbox_w * bbox_h) / float(w * h)

                        if st.intrinsic.result is not None:
                            try:
                                obj_pts, img_pts = calib_board.match_object_points(
                                    ch_corners, ch_ids
                                )
                                ok_pnp, rvec, tvec = cv2.solvePnP(
                                    obj_pts,
                                    img_pts,
                                    st.intrinsic.result.camera_matrix,
                                    st.intrinsic.result.dist_coeffs,
                                    flags=cv2.SOLVEPNP_ITERATIVE,
                                )
                                if ok_pnp:
                                    cur_R, _ = cv2.Rodrigues(rvec)
                                    cur_t = np.asarray(tvec).reshape(3)
                                    cos_v = float(np.clip(abs(cur_R[2, 2]), 0.0, 1.0))
                                    cur_tilt = float(np.degrees(np.arccos(cos_v)))
                                    payload["tilt_deg"] = cur_tilt
                            except cv2.error:
                                pass

                    # Phase 1 Traffic Light — 현재 pose 를 기존 캡처와 비교해 G/Y/R
                    # 실시간 판정 (검출+tilt+diversity). 토크오프 이동 중 "지금 찍어도
                    # 좋은 데이터셋이 되나" 안내 (handeye_ux_solver_v3_plan.md §5).
                    self._add_capture_quality(payload, rid, st, bool(ok), cur_tilt,
                                              cur_R, cur_t)

                    self.publish(topic_for(Topic.CALIB_HANDEYE_PREVIEW, rid), payload)
                except Exception as e:
                    logger.debug("[%s] preview loop 오류: %s", rid, e)

            time.sleep(PREVIEW_INTERVAL)
