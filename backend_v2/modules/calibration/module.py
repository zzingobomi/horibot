"""CalibrationModule — robot-scoped Domain Module (5 종 산출물 owner).

boundary spec = [docs/calibration_module_boundary.md]. **robot-scoped** — service key
`srv/calibration/{robot_id}/...` 가 framework 에서 self.robot_id 로 확장. PC 배치.

service 핸들러는 sync → 옛 FrameCache/JointStateCache 패턴대로 camera decoded frame +
motor raw 를 @subscriber 로 캐시하고 capture 가 sync 로 읽음 (runtime.call async 회피).

미구현(다음): preview 5Hz stream / factory-intrinsic pull(§10.1, camera-coupled).
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from datetime import UTC, datetime

import cv2
import numpy as np

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.contract.subscriber import subscriber
from framework.runtime.api import ModuleRuntime
from framework.storage.protocol import ObjectStore
from modules.camera.contract import (
    Camera,
    CameraDecodedFrame,
    FactoryIntrinsic,
    GetFactoryIntrinsicRequest,
)
from modules.motor.contract import JointState, Motor

from .vision import board as calib_board
from .vision import capture_quality, processing
from .vision import thresholds as thr
from .contract import (
    ActivateResultRequest,
    ActivateResultResponse,
    Calibration,
    CalibrationActivated,
    CalibrationBundle,
    CalibrationCaptureArtifactRecord,
    CalibrationCaptureRecord,
    CalibrationCommitted,
    CalibrationPreview,
    CaptureQualityPayload,
    CaptureRequest,
    CaptureResponse,
    FinalizeRunRequest,
    FinalizeRunResponse,
    GetThresholdsRequest,
    GetThresholdsResponse,
    IntrinsicResultData,
    IntrinsicResultRecord,
    ListResultsRequest,
    ListResultsResponse,
    ListRunsRequest,
    ListRunsResponse,
    PreviewEnableRequest,
    PreviewEnableResponse,
    SnapshotBundleRequest,
    StartRunRequest,
    StartRunResponse,
    UndoLastCaptureRequest,
    UndoLastCaptureResponse,
)
from .persistence.repository import CalibrationRepository

logger = logging.getLogger(__name__)


def _raw_to_rad(raw: int) -> float:
    """rough raw→rad (0..4095 중심 2048, 4096=2π) — quality diversity 용 근사."""
    return (raw - 2048) / 4096.0 * 2.0 * math.pi


_PREVIEW_HZ = 5.0


@publishes(
    (Calibration.Event.ACTIVATED, CalibrationActivated),
    (Calibration.Event.COMMITTED, CalibrationCommitted),
    (Calibration.Stream.PREVIEW, CalibrationPreview),
)
class CalibrationModule:
    def __init__(
        self,
        runtime: ModuleRuntime,
        robot_id: str,
        repository: CalibrationRepository,
        object_store: ObjectStore,
        motor_ids: list[int],
    ) -> None:
        self.runtime = runtime
        self.robot_id = robot_id
        self._repo = repository
        self._blob = object_store
        self._motor_ids = motor_ids  # arm joint motor ids (motors.yaml 순)
        # 캐시 (subscriber 갱신 → capture 가 sync read)
        self._latest_frame: np.ndarray | None = None
        self._latest_raw: dict[int, int] | None = None
        # preview
        self._preview_on = False
        self._preview_seq = 0
        self._stop = False
        self._preview_task: asyncio.Task[None] | None = None

    # ── lifecycle ─────────────────────────────────────────────
    async def start(self) -> None:
        logger.info("CalibrationModule start robot=%s", self.robot_id)
        await self._seed_factory_intrinsic()
        self._stop = False
        self._preview_task = asyncio.create_task(self._preview_loop())

    async def _seed_factory_intrinsic(self) -> None:
        """§10.1 A — Camera 에서 factory intrinsic pull → idempotent seed.

        이미 active intrinsic 있으면 skip (사용자 chessboard 캘 결과 덮지 않음). Camera
        미배치/실패면 skip (USB UVC 는 available=False → 사용자 캘 자리)."""
        if self._repo.get_active(self.robot_id, "intrinsic") is not None:
            return
        try:
            fi = await self.runtime.call(
                Camera.Service.GET_FACTORY_INTRINSIC,
                GetFactoryIntrinsicRequest(),
                FactoryIntrinsic,
                robot_id=self.robot_id,
                timeout=3.0,
            )
        except Exception:
            logger.info("factory intrinsic pull 실패/미배치 — skip robot=%s", self.robot_id)
            return
        if not fi.available:
            return
        cm = [[fi.fx, 0.0, fi.cx], [0.0, fi.fy, fi.cy], [0.0, 0.0, 1.0]]
        run = self._repo.create_run(self.robot_id, "intrinsic", "factory")
        assert run.id is not None
        rid = self._repo.save_result(
            run.id,
            IntrinsicResultRecord(
                run_id=run.id,
                robot_id=self.robot_id,
                created_at=datetime.now(UTC),
                result_data=IntrinsicResultData(
                    camera_matrix=cm,
                    dist_coeffs=[[0.0, 0.0, 0.0, 0.0, 0.0]],
                    image_size=[fi.width, fi.height],
                ),
            ),
        )
        self._repo.finalize_run(run.id, "success")
        self._repo.activate_result(rid)
        logger.info("factory intrinsic seeded robot=%s", self.robot_id)

    async def stop(self) -> None:
        self._stop = True
        task = self._preview_task
        if task is not None:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            self._preview_task = None
        logger.info("CalibrationModule stop robot=%s", self.robot_id)

    # ── camera / motor 캐시 (sync callback) ───────────────────
    @subscriber(Camera.Stream.DECODED)
    def on_decoded_frame(self, frame: CameraDecodedFrame) -> None:
        if frame.robot_id != self.robot_id:
            return
        arr = np.frombuffer(frame.ndarray_bytes, dtype=np.uint8)
        expected = frame.height * frame.width * 3
        if arr.size != expected:
            return
        self._latest_frame = arr.reshape(frame.height, frame.width, 3)

    @subscriber(Motor.Stream.RAW_STATE)
    def on_motor_raw(self, state: JointState) -> None:
        if state.robot_id != self.robot_id:
            return
        n = min(len(self._motor_ids), len(state.positions_raw))
        self._latest_raw = {
            self._motor_ids[i]: int(state.positions_raw[i]) for i in range(n)
        }

    # ── commands (write) ──────────────────────────────────────
    @service(Calibration.Service.START_RUN)
    def start_run(self, req: StartRunRequest) -> StartRunResponse:
        run = self._repo.create_run(self.robot_id, req.kind, req.algorithm)
        assert run.id is not None
        return StartRunResponse(run_id=run.id)

    @service(Calibration.Service.CAPTURE)
    def capture(self, req: CaptureRequest) -> CaptureResponse:
        """현재 캐시된 frame + raw 로 ChArUco PnP → gate → DB row + color blob.

        gate: 미검출 / reproj RMS > reject 임계 시 accepted=False (capture 안 들임 —
        trauma source 입구 차단, thresholds §HANDEYE_PNP).
        """
        frame = self._latest_frame
        if frame is None:
            return CaptureResponse(accepted=False, message="카메라 프레임 없음")

        intrinsic = self._repo.get_active(self.robot_id, "intrinsic")
        if not isinstance(intrinsic, IntrinsicResultRecord):
            return CaptureResponse(
                accepted=False,
                message="active intrinsic 없음 (D405 factory seed / USB 캘 필요)",
            )
        cm = np.array(intrinsic.result_data.camera_matrix, dtype=np.float64)
        dist = np.array(intrinsic.result_data.dist_coeffs, dtype=np.float64)

        det = processing.detect_and_pnp(frame, cm, dist)
        if det is None:
            return CaptureResponse(
                accepted=False,
                quality=CaptureQualityPayload(verdict="red", reasons=["보드 미검출"]),
                message="ChArUco 보드 미검출",
            )
        if det.reproj_rms_px > thr.HANDEYE_PNP_RMS_REJECT_PX:
            return CaptureResponse(
                accepted=False,
                reproj_rms_px=det.reproj_rms_px,
                tilt_deg=det.tilt_deg,
                quality=CaptureQualityPayload(
                    verdict="red", reasons=["이미지 품질 부족 (또렷하게 다시)"]
                ),
                message=f"reproj RMS {det.reproj_rms_px:.2f}px > "
                f"{thr.HANDEYE_PNP_RMS_REJECT_PX}px",
            )

        quality = self._evaluate_quality(req.run_id, det)

        capture = CalibrationCaptureRecord(
            run_id=req.run_id,
            pose_index=req.pose_index,
            motor_positions=self._latest_raw,
            board_in_cam=det.board_in_cam,
            corners_2d=det.corners_2d,
            corner_ids=det.corner_ids,
            reproj_rms_px=det.reproj_rms_px,
            tilt_deg=det.tilt_deg,
        )
        capture_id = self._repo.append_capture(req.run_id, capture)

        # color blob (JPEG). depth/primary 는 후속 (mock color-only).
        ok, buf = cv2.imencode(".jpg", frame)
        if ok:
            key = (
                f"calib_captures/{self.robot_id}/{req.run_id}/"
                f"{req.pose_index:03d}_color.jpg"
            )
            self._blob.put(key, buf.tobytes())
            self._repo.save_artifact(
                capture_id,
                CalibrationCaptureArtifactRecord(
                    capture_id=capture_id,
                    kind="color",
                    blob_key=key,
                    size_bytes=len(buf.tobytes()),
                    content_type="image/jpeg",
                    created_at=datetime.now(UTC),
                ),
            )

        return CaptureResponse(
            accepted=True,
            capture_id=capture_id,
            reproj_rms_px=det.reproj_rms_px,
            tilt_deg=det.tilt_deg,
            quality=CaptureQualityPayload(
                verdict=quality.verdict, reasons=quality.reasons
            ),
        )

    @service(Calibration.Service.FINALIZE_RUN)
    def finalize_run(self, req: FinalizeRunRequest) -> FinalizeRunResponse:
        self._repo.finalize_run(req.run_id, "ready_for_analysis")
        self.runtime.publish(
            Calibration.Event.COMMITTED,
            CalibrationCommitted(robot_id=self.robot_id, run_id=req.run_id),
        )
        return FinalizeRunResponse(ok=True)

    @service(Calibration.Service.ACTIVATE_RESULT)
    def activate_result(self, req: ActivateResultRequest) -> ActivateResultResponse:
        rec = self._repo.activate_result(req.result_id)
        self.runtime.publish(
            Calibration.Event.ACTIVATED,
            CalibrationActivated(
                robot_id=self.robot_id, result_id=req.result_id, kind=rec.kind
            ),
        )
        return ActivateResultResponse(ok=True)

    @service(Calibration.Service.UNDO_LAST_CAPTURE)
    def undo_last_capture(
        self, req: UndoLastCaptureRequest
    ) -> UndoLastCaptureResponse:
        self._repo.undo_last_capture(req.run_id)
        return UndoLastCaptureResponse(ok=True)

    @service(Calibration.Service.PREVIEW_ENABLE)
    def preview_enable(self, req: PreviewEnableRequest) -> PreviewEnableResponse:
        self._preview_on = req.enabled
        return PreviewEnableResponse(ok=True)

    # ── queries (read) ────────────────────────────────────────
    @service(Calibration.Service.SNAPSHOT_BUNDLE)
    def snapshot_bundle(self, req: SnapshotBundleRequest) -> CalibrationBundle:
        return self._repo.get_active_bundle(self.robot_id)

    @service(Calibration.Service.LIST_RUNS)
    def list_runs(self, req: ListRunsRequest) -> ListRunsResponse:
        return ListRunsResponse(runs=self._repo.list_runs(self.robot_id, req.kind))

    @service(Calibration.Service.LIST_RESULTS)
    def list_results(self, req: ListResultsRequest) -> ListResultsResponse:
        return ListResultsResponse(
            results=self._repo.list_results(self.robot_id, req.kind)
        )

    @service(Calibration.Service.GET_THRESHOLDS)
    def get_thresholds(self, req: GetThresholdsRequest) -> GetThresholdsResponse:
        return GetThresholdsResponse(thresholds=thr.as_dict())

    # ── preview loop (5Hz) ────────────────────────────────────
    async def _preview_loop(self) -> None:
        interval = 1.0 / _PREVIEW_HZ
        try:
            while not self._stop:
                if self._preview_on and self._latest_frame is not None:
                    try:
                        self._publish_preview()
                    except Exception:
                        logger.exception("preview publish 실패 %s", self.robot_id)
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            pass

    def _publish_preview(self) -> None:
        frame = self._latest_frame
        if frame is None:
            return
        intrinsic = self._repo.get_active(self.robot_id, "intrinsic")
        det = None
        detected = False
        corner_count = 0
        if isinstance(intrinsic, IntrinsicResultRecord):
            cm = np.array(intrinsic.result_data.camera_matrix, dtype=np.float64)
            dist = np.array(intrinsic.result_data.dist_coeffs, dtype=np.float64)
            det = processing.detect_and_pnp(frame, cm, dist)
        if det is not None:
            detected = True
            corner_count = len(det.corner_ids)
            run = self._repo.get_in_progress_run(self.robot_id, "hand_eye")
            quality = (
                self._evaluate_quality(run.id, det)
                if run is not None and run.id is not None
                else capture_quality.CaptureQuality("green", ["첫 자세 — 캡처 권장"])
            )
            verdict, reasons, tilt = quality.verdict, quality.reasons, det.tilt_deg
        else:
            # intrinsic 없거나 미검출 — detect-only (tilt 없음)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ok, _c, ids = calib_board.detect(gray)
            detected = ok
            corner_count = 0 if ids is None else len(ids)
            verdict = "red"
            reasons = ["보드 미검출"] if not ok else ["자세 추정 불가 (intrinsic 확인)"]
            tilt = None
        self.runtime.publish(
            Calibration.Stream.PREVIEW,
            CalibrationPreview(
                robot_id=self.robot_id,
                seq=self._preview_seq,
                timestamp_unix=time.time(),
                detected=detected,
                corner_count=corner_count,
                tilt_deg=tilt,
                verdict=verdict,
                reasons=reasons,
            ),
        )
        self._preview_seq += 1

    # ── internal ──────────────────────────────────────────────
    def _evaluate_quality(
        self, run_id: int, det: processing.CaptureDetection
    ) -> capture_quality.CaptureQuality:
        """현재 자세 vs run 의 기존 capture 들 → traffic light."""
        existing = self._repo.list_captures(run_id)
        ex_joints: list[list[float]] = []
        ex_R: list[np.ndarray] = []
        ex_t: list[np.ndarray] = []
        for c in existing:
            if c.board_in_cam is not None:
                T = np.array(c.board_in_cam, dtype=np.float64)
                ex_R.append(T[:3, :3])
                ex_t.append(T[:3, 3])
            if c.motor_positions:
                ex_joints.append(
                    [_raw_to_rad(c.motor_positions[m]) for m in sorted(c.motor_positions)]
                )
        cur_T = np.array(det.board_in_cam, dtype=np.float64)
        cur_joints = (
            [_raw_to_rad(self._latest_raw[m]) for m in sorted(self._latest_raw)]
            if self._latest_raw
            else None
        )
        return capture_quality.evaluate_capture_quality(
            detected=True,
            tilt_deg=det.tilt_deg,
            current_joints_rad=cur_joints,
            current_R_t2c=cur_T[:3, :3],
            current_t_t2c=cur_T[:3, 3],
            existing_joints_rad=ex_joints,
            existing_R_t2c=ex_R,
            existing_t_t2c=ex_t,
        )
