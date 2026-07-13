"""CalibrationModule — robot-agnostic Domain Module (5 종 산출물 owner).

boundary spec = [docs/calibration.md]. **robot-agnostic** — host 당
1 인스턴스 (backend.md §2.7). 서비스
키에 {robot_id} 없음; 대상 robot 은 req.robot_id 또는 run_id/result_id 의 DB row 에서
파생. runtime state 는 전부 robot_id 키 dict, robot config(모터 id 등)는 resolve 가
주입한 robots 투영으로 요청 시점 조회 (모듈이 복사·재보유 X). PC 배치.

service 핸들러는 sync → 옛 FrameCache/JointStateCache 패턴대로 camera decoded frame +
motor raw 를 @subscriber 로 캐시하고 capture 가 sync 로 읽음 (runtime.call async 회피).
@subscriber 는 framework 가 {robot_id}→* wildcard 구독 — 전 robot frame 이 들어오고
payload 의 robot_id 로 dict 캐시.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections import Counter, deque
from datetime import UTC, datetime

import cv2
import numpy as np
from pydantic import BaseModel

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
    CalibrationKind,
    CalibrationPreview,
    CaptureQualityPayload,
    CaptureRequest,
    AbortRunRequest,
    AbortRunResponse,
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

# 캡처 세션을 여는 kind (start_run 허용 + stale cleanup + preview 라우팅 공용).
# intrinsic = detect-only, hand_eye/cross = PnP 경로 (동일 캡처·판정, 소비만 다름
# — hand_eye 는 offline BA, cross 는 cross_calibrate.py 합성 → robots.yaml).
_CAPTURE_SESSION_KINDS: tuple[CalibrationKind, ...] = (
    "intrinsic", "hand_eye", "cross",
)
# PnP 캡처 세션 (preview 가 hand-eye 다양성 판정기를 공유하는 kind)
_PNP_SESSION_KINDS: tuple[CalibrationKind, ...] = ("hand_eye", "cross")


class CalibrationRobotSpec(BaseModel):
    """robot 별 정적 config — resolve 가 robots.yaml 에서 투영해 주입.

    wire contract 아님 (constructor dep). 모듈은 robots SSOT 를 재보유하지 않고
    이 lean 투영으로 요청 시점 조회 (bridge 의 RobotInfo 변환과 동형).
    """

    motor_ids: list[int]  # arm joint motor ids (motors.yaml 순)
    has_camera: bool  # factory intrinsic seed 시도 대상 (camera_backend 있음)


@publishes(
    (Calibration.Event.ACTIVATED, CalibrationActivated),
    (Calibration.Event.COMMITTED, CalibrationCommitted),
    (Calibration.Stream.PREVIEW, CalibrationPreview),
)
class CalibrationModule:
    def __init__(
        self,
        runtime: ModuleRuntime,
        repository: CalibrationRepository,
        object_store: ObjectStore,
        robots: dict[str, CalibrationRobotSpec],
    ) -> None:
        self.runtime = runtime
        self._repo = repository
        self._blob = object_store
        self._robots = robots
        # runtime state — 전부 robot_id 키 dict (실행 중에만 존재, 대부분 0~1 sparse)
        self._latest_frame: dict[str, np.ndarray] = {}
        self._latest_raw: dict[str, dict[int, int]] = {}
        self._preview_on: dict[str, bool] = {}
        self._preview_seq: dict[str, int] = {}
        # preview 신호등 flap 억제 — 최근 raw 판정 window + 현재 표시 판정
        self._preview_recent: dict[str, deque[tuple[str, tuple[str, ...]]]] = {}
        self._preview_shown: dict[str, tuple[str, list[str]]] = {}
        self._stop = False
        self._preview_task: asyncio.Task[None] | None = None

    # ── lifecycle ─────────────────────────────────────────────
    async def start(self) -> None:
        logger.info(
            "CalibrationModule start (host-level, robots=%s)", sorted(self._robots)
        )
        # 카메라 보유 robot 전체 seed — gather 로 동시 (미배치 robot 의 timeout 이
        # 직렬로 안 쌓이게). 각 seed 는 내부 try/except → 하나 실패해도 나머지 진행.
        seeds = [
            self._seed_factory_intrinsic(rid)
            for rid, spec in self._robots.items()
            if spec.has_camera
        ]
        if seeds:
            await asyncio.gather(*seeds)
        self._stop = False
        self._preview_task = asyncio.create_task(self._preview_loop())

    async def _seed_factory_intrinsic(self, robot_id: str) -> None:
        """§10.1 A — Camera 에서 factory intrinsic pull → idempotent seed.

        이미 active intrinsic 있으면 skip (사용자 chessboard 캘 결과 덮지 않음). Camera
        미배치/실패면 skip (USB UVC 는 available=False → 사용자 캘 자리)."""
        if self._repo.get_active(robot_id, "intrinsic") is not None:
            return
        try:
            fi = await self.runtime.call(
                Camera.Service.GET_FACTORY_INTRINSIC,
                GetFactoryIntrinsicRequest(),
                FactoryIntrinsic,
                robot_id=robot_id,
                timeout=3.0,
            )
        except Exception:
            logger.info("factory intrinsic pull 실패/미배치 — skip robot=%s", robot_id)
            return
        if not fi.available:
            return
        cm = [[fi.fx, 0.0, fi.cx], [0.0, fi.fy, fi.cy], [0.0, 0.0, 1.0]]
        run = self._repo.create_run(robot_id, "intrinsic", "factory")
        assert run.id is not None
        rid = self._repo.save_result(
            run.id,
            IntrinsicResultRecord(
                run_id=run.id,
                robot_id=robot_id,
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
        logger.info("factory intrinsic seeded robot=%s", robot_id)

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
        logger.info("CalibrationModule stop (host-level)")

    # ── camera / motor 캐시 (sync callback, 전 robot wildcard) ──
    @subscriber(Camera.Stream.DECODED)
    def on_decoded_frame(self, frame: CameraDecodedFrame) -> None:
        if frame.robot_id not in self._robots:
            return
        arr = np.frombuffer(frame.ndarray_bytes, dtype=np.uint8)
        expected = frame.height * frame.width * 3
        if arr.size != expected:
            return
        self._latest_frame[frame.robot_id] = arr.reshape(
            frame.height, frame.width, 3
        )

    @subscriber(Motor.Stream.RAW_STATE)
    def on_motor_raw(self, state: JointState) -> None:
        spec = self._robots.get(state.robot_id)
        if spec is None:
            return
        n = min(len(spec.motor_ids), len(state.positions_raw))
        self._latest_raw[state.robot_id] = {
            spec.motor_ids[i]: int(state.positions_raw[i]) for i in range(n)
        }

    # ── commands (write) ──────────────────────────────────────
    @service(Calibration.Service.START_RUN)
    def start_run(self, req: StartRunRequest) -> StartRunResponse:
        # 캡처 세션은 이 3종만 — joint/link/sag 는 offline BA 산출물이라 세션이 없다.
        if req.kind not in _CAPTURE_SESSION_KINDS:
            raise ValueError(
                f"kind {req.kind!r} 는 캡처 세션 대상 아님 "
                f"(가능: {', '.join(_CAPTURE_SESSION_KINDS)})"
            )
        # 도메인 규칙: robot 당 활성 캡처 세션 1개 — preview 판정이
        # get_in_progress_run 으로 세션을 찾으므로 stale run 이 남으면 오판.
        # 새 시작 = 이전 미완 세션 폐기 (browser reload 로 UI 가 runId 를 잃어도
        # orphan run 이 영구 in_progress 로 쌓이지 않음).
        for kind in _CAPTURE_SESSION_KINDS:
            stale = self._repo.get_in_progress_run(req.robot_id, kind)
            while stale is not None and stale.id is not None:
                self._repo.finalize_run(stale.id, "failed")
                logger.info(
                    "stale in_progress run 자동 abort: robot=%s run=%d (%s)",
                    req.robot_id,
                    stale.id,
                    kind,
                )
                stale = self._repo.get_in_progress_run(req.robot_id, kind)
        run = self._repo.create_run(req.robot_id, req.kind, req.algorithm)
        assert run.id is not None
        return StartRunResponse(run_id=run.id)

    @service(Calibration.Service.ABORT_RUN)
    def abort_run(self, req: AbortRunRequest) -> AbortRunResponse:
        """세션 중도 포기 — run status → failed. 캡처 row 는 보존 (append-only).

        finalize 와 별개 경로: intrinsic finalize 는 캡처 부족 시 run 을 살려두는데
        (ok=False, 더 캡처 유도), 사용자가 그 상태에서 그냥 나가고 싶을 때의
        탈출구가 이 서비스 (0장 세션에서 종료 거부 + undo 비활성으로 갇히던 결함).
        """
        run = self._repo.get_run(req.run_id)
        if run is None:
            raise KeyError(f"run {req.run_id} 없음")
        if run.status != "in_progress":
            return AbortRunResponse(
                ok=False, message=f"run {req.run_id} 은 이미 {run.status}"
            )
        self._repo.finalize_run(req.run_id, "failed")
        return AbortRunResponse(ok=True)

    @service(Calibration.Service.CAPTURE)
    def capture(self, req: CaptureRequest) -> CaptureResponse:
        """run 의 robot frame + raw 로 ChArUco PnP → gate → DB row + color blob.

        대상 robot = run 소유자 (run_id 에서 파생 — req robot_id 중복 채널 X).
        gate: 미검출 / reproj RMS > reject 임계 시 accepted=False (capture 안 들임 —
        trauma source 입구 차단, thresholds §HANDEYE_PNP).

        kind 분기: intrinsic run 은 detect-only (PnP 불가 — intrinsic 이 아직 없음,
        닭-달걀). USB UVC 처럼 factory intrinsic 없는 카메라의 사용자 캘 경로.
        """
        run = self._repo.get_run(req.run_id)
        if run is None:
            raise KeyError(f"run {req.run_id} 없음")
        robot_id = run.robot_id
        # 캡처는 coverage/diversity 기준 자체를 바꾸므로 신호등 smoothing 리셋 —
        # 다음 preview 판정이 hysteresis 지연 없이 즉시 표시
        self._preview_recent.pop(robot_id, None)
        self._preview_shown.pop(robot_id, None)

        frame = self._latest_frame.get(robot_id)
        if frame is None:
            return CaptureResponse(accepted=False, message="카메라 프레임 없음")

        if run.kind == "intrinsic":
            return self._capture_intrinsic(req, robot_id, frame)

        intrinsic = self._repo.get_active(robot_id, "intrinsic")
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

        quality = self._evaluate_quality(req.run_id, det, robot_id)

        capture = CalibrationCaptureRecord(
            run_id=req.run_id,
            pose_index=req.pose_index,
            motor_positions=self._latest_raw.get(robot_id),
            board_in_cam=det.board_in_cam,
            corners_2d=det.corners_2d,
            corner_ids=det.corner_ids,
            reproj_rms_px=det.reproj_rms_px,
            tilt_deg=det.tilt_deg,
        )
        capture_id = self._repo.append_capture(req.run_id, capture)
        self._save_color_artifact(robot_id, req, capture_id, frame)

        return CaptureResponse(
            accepted=True,
            capture_id=capture_id,
            reproj_rms_px=det.reproj_rms_px,
            tilt_deg=det.tilt_deg,
            quality=CaptureQualityPayload(
                verdict=quality.verdict, reasons=quality.reasons
            ),
        )

    def _save_color_artifact(
        self,
        robot_id: str,
        req: CaptureRequest,
        capture_id: int,
        frame: np.ndarray,
    ) -> None:
        """capture color blob (JPEG) 저장. depth/primary 는 후속 (mock color-only)."""
        ok, buf = cv2.imencode(".jpg", frame)
        if not ok:
            return
        key = (
            f"calib_captures/{robot_id}/{req.run_id}/"
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

    def _capture_intrinsic(
        self, req: CaptureRequest, robot_id: str, frame: np.ndarray
    ) -> CaptureResponse:
        """intrinsic detect-only capture — corners 저장, PnP 없음 (닭-달걀 회피).

        gate = ChArUco MIN_CORNERS (board.detect 내장). 품질 판정 = 3×3 grid
        coverage (distortion 모델의 image plane 전 영역 generalize — 옛 backend
        intrinsic.py 정공법). compute 는 finalize_run 에서 (calibrateCamera).
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ok, ch_corners, ch_ids = calib_board.detect(gray)
        if not ok or ch_corners is None or ch_ids is None:
            return CaptureResponse(
                accepted=False,
                quality=CaptureQualityPayload(
                    verdict="red", reasons=["보드 미검출 (또는 코너 부족)"]
                ),
                message="ChArUco 보드 미검출/코너 부족",
            )

        h, w = int(frame.shape[0]), int(frame.shape[1])
        existing = self._repo.list_captures(req.run_id)
        covered = {
            capture_quality.intrinsic_coverage_cell(
                np.array(c.corners_2d, dtype=np.float64), w, h
            )
            for c in existing
            if c.corners_2d
        }
        cell = capture_quality.intrinsic_coverage_cell(ch_corners, w, h)
        quality = capture_quality.evaluate_intrinsic_capture(
            cell=cell, covered_cells=covered, n_existing=len(existing)
        )

        capture = CalibrationCaptureRecord(
            run_id=req.run_id,
            pose_index=req.pose_index,
            corners_2d=ch_corners.reshape(-1, 2).astype(float).tolist(),
            corner_ids=ch_ids.reshape(-1).astype(int).tolist(),
        )
        capture_id = self._repo.append_capture(req.run_id, capture)
        self._save_color_artifact(robot_id, req, capture_id, frame)
        return CaptureResponse(
            accepted=True,
            capture_id=capture_id,
            quality=CaptureQualityPayload(
                verdict=quality.verdict, reasons=quality.reasons
            ),
        )

    @service(Calibration.Service.FINALIZE_RUN)
    def finalize_run(self, req: FinalizeRunRequest) -> FinalizeRunResponse:
        run = self._repo.get_run(req.run_id)
        if run is None:
            raise KeyError(f"run {req.run_id} 없음")
        if run.kind == "intrinsic":
            # intrinsic 은 finalize 에서 compute 까지 — calibrateCamera 는 빠르고
            # 결정적 (offline 분리는 hand_eye BA trauma 이유였음, v1 save 동형).
            return self._finalize_intrinsic(req.run_id, run.robot_id)
        self._repo.finalize_run(req.run_id, "ready_for_analysis")
        self.runtime.publish(
            Calibration.Event.COMMITTED,
            CalibrationCommitted(robot_id=run.robot_id, run_id=req.run_id),
        )
        return FinalizeRunResponse(ok=True)

    def _finalize_intrinsic(self, run_id: int, robot_id: str) -> FinalizeRunResponse:
        """intrinsic run finalize = cv2.calibrateCamera + result 저장 + activate.

        캡처 부족이면 run 을 살려둔 채 ok=False (사용자가 더 캡처 후 재시도).
        """
        captures = [
            c
            for c in self._repo.list_captures(run_id)
            if c.corners_2d and c.corner_ids
        ]
        if len(captures) < thr.INTRINSIC_MIN_CAPTURES:
            return FinalizeRunResponse(
                ok=False,
                message=(
                    f"캡처 부족 ({len(captures)}장 < 최소 "
                    f"{thr.INTRINSIC_MIN_CAPTURES}장) — 더 캡처 후 재시도"
                ),
            )

        # image size — 저장된 color blob 에서 (capture record 에 크기 필드 없음)
        image_size = self._intrinsic_image_size(captures)
        if image_size is None:
            return FinalizeRunResponse(
                ok=False, message="color blob 에서 image size 복원 실패"
            )

        obj_list: list[np.ndarray] = []
        img_list: list[np.ndarray] = []
        for c in captures:
            assert c.corners_2d is not None and c.corner_ids is not None
            corners = np.array(c.corners_2d, dtype=np.float32).reshape(-1, 1, 2)
            ids = np.array(c.corner_ids, dtype=np.int32).reshape(-1, 1)
            obj_pts, img_pts = calib_board.match_object_points(corners, ids)
            if obj_pts is None or img_pts is None or len(obj_pts) < 4:
                continue
            obj_list.append(obj_pts)
            img_list.append(img_pts)
        if len(obj_list) < thr.INTRINSIC_MIN_CAPTURES:
            return FinalizeRunResponse(
                ok=False,
                message=f"유효 캡처 부족 ({len(obj_list)}장) — 더 캡처 후 재시도",
            )

        # cv2.calibrateCamera — 가변 길이 obj/img list 그대로 받음 (v1 동형).
        rms, cm, dist, _rvecs, _tvecs = cv2.calibrateCamera(
            obj_list, img_list, image_size, None, None  # type: ignore[arg-type,call-overload]
        )

        result_id = self._repo.save_result(
            run_id,
            IntrinsicResultRecord(
                run_id=run_id,
                robot_id=robot_id,
                created_at=datetime.now(UTC),
                result_data=IntrinsicResultData(
                    camera_matrix=np.asarray(cm, dtype=float).tolist(),
                    dist_coeffs=np.asarray(dist, dtype=float)
                    .reshape(1, -1)
                    .tolist(),
                    image_size=[image_size[0], image_size[1]],
                    rms_px=float(rms),
                ),
            ),
        )
        self._repo.finalize_run(run_id, "success")
        rec = self._repo.activate_result(result_id)
        self.runtime.publish(
            Calibration.Event.COMMITTED,
            CalibrationCommitted(robot_id=robot_id, run_id=run_id),
        )
        self.runtime.publish(
            Calibration.Event.ACTIVATED,
            CalibrationActivated(
                robot_id=robot_id, result_id=result_id, kind=rec.kind
            ),
        )
        logger.info(
            "intrinsic 캘 완료 robot=%s run=%d: RMS=%.4fpx (%d장, %dx%d)",
            robot_id, run_id, rms, len(obj_list), image_size[0], image_size[1],
        )
        return FinalizeRunResponse(
            ok=True,
            message=f"RMS {rms:.3f}px ({len(obj_list)}장) — 저장 + 활성화 완료",
        )

    def _intrinsic_image_size(
        self, captures: list[CalibrationCaptureRecord]
    ) -> tuple[int, int] | None:
        """(w, h) — 첫 캡처의 color blob JPEG 디코드로 복원."""
        for c in captures:
            art = c.find_artifact("color")
            if art is None:
                continue
            try:
                data = self._blob.get(art.blob_key)
            except Exception:
                continue
            arr = cv2.imdecode(
                np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if arr is not None:
                return int(arr.shape[1]), int(arr.shape[0])
        return None

    @service(Calibration.Service.ACTIVATE_RESULT)
    def activate_result(self, req: ActivateResultRequest) -> ActivateResultResponse:
        rec = self._repo.activate_result(req.result_id)
        self.runtime.publish(
            Calibration.Event.ACTIVATED,
            CalibrationActivated(
                robot_id=rec.robot_id, result_id=req.result_id, kind=rec.kind
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
        if req.robot_id not in self._robots:
            raise KeyError(f"robot {req.robot_id!r} 이 이 host fleet 에 없음")
        self._preview_on[req.robot_id] = req.enabled
        return PreviewEnableResponse(ok=True)

    # ── queries (read) ────────────────────────────────────────
    @service(Calibration.Service.SNAPSHOT_BUNDLE)
    def snapshot_bundle(self, req: SnapshotBundleRequest) -> CalibrationBundle:
        return self._repo.get_active_bundle(req.robot_id)

    @service(Calibration.Service.LIST_RUNS)
    def list_runs(self, req: ListRunsRequest) -> ListRunsResponse:
        return ListRunsResponse(runs=self._repo.list_runs(req.robot_id, req.kind))

    @service(Calibration.Service.LIST_RESULTS)
    def list_results(self, req: ListResultsRequest) -> ListResultsResponse:
        return ListResultsResponse(
            results=self._repo.list_results(req.robot_id, req.kind)
        )

    @service(Calibration.Service.GET_THRESHOLDS)
    def get_thresholds(self, req: GetThresholdsRequest) -> GetThresholdsResponse:
        return GetThresholdsResponse(thresholds=thr.as_dict())

    # ── preview loop (5Hz, robot 별) ──────────────────────────
    async def _preview_loop(self) -> None:
        interval = 1.0 / _PREVIEW_HZ
        try:
            while not self._stop:
                # snapshot 순회 — preview_enable(워커 스레드)의 dict 변경과 격리
                for rid, on in list(self._preview_on.items()):
                    if not on or self._latest_frame.get(rid) is None:
                        continue
                    try:
                        # detect_and_pnp 는 CPU-bound — loop 블로킹 방지
                        await asyncio.to_thread(self._publish_preview, rid)
                    except Exception:
                        logger.exception("preview publish 실패 %s", rid)
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            pass

    def _publish_preview(self, robot_id: str) -> None:
        frame = self._latest_frame.get(robot_id)
        if frame is None:
            return
        img_h, img_w = int(frame.shape[0]), int(frame.shape[1])
        det = None
        detected = False
        corner_count = 0
        corners_2d: list[list[float]] = []
        # 진행 중 세션의 kind 가 판정 경로를 결정한다. active intrinsic 존재 여부로
        # 먼저 분기하면 재캘(intrinsic 세션 + active intrinsic 공존)이 hand-eye
        # 판정으로 새서 상시 green (2026-07-11 실사고).
        intr_run = self._repo.get_in_progress_run(robot_id, "intrinsic")
        if intr_run is not None and intr_run.id is not None:
            # intrinsic 세션 — detect-only + 3×3 coverage 판정 (PnP 불필요)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ok, ch_corners, ids = calib_board.detect(gray)
            detected = ok
            corner_count = 0 if ids is None else len(ids)
            tilt = None
            if ok and ch_corners is not None:
                corners_2d = ch_corners.reshape(-1, 2).astype(float).tolist()
                existing = self._repo.list_captures(intr_run.id)
                covered = {
                    capture_quality.intrinsic_coverage_cell(
                        np.array(c.corners_2d, dtype=np.float64), img_w, img_h
                    )
                    for c in existing
                    if c.corners_2d
                }
                cell = capture_quality.intrinsic_coverage_cell(
                    ch_corners, img_w, img_h
                )
                q = capture_quality.evaluate_intrinsic_capture(
                    cell=cell,
                    covered_cells=covered,
                    n_existing=len(existing),
                )
                verdict, reasons = q.verdict, q.reasons
            else:
                verdict, reasons = "red", ["보드 미검출"]
        else:
            intrinsic = self._repo.get_active(robot_id, "intrinsic")
            if isinstance(intrinsic, IntrinsicResultRecord):
                cm = np.array(intrinsic.result_data.camera_matrix, dtype=np.float64)
                dist = np.array(intrinsic.result_data.dist_coeffs, dtype=np.float64)
                det = processing.detect_and_pnp(frame, cm, dist)
            if det is not None:
                detected = True
                corner_count = len(det.corner_ids)
                corners_2d = det.corners_2d
                # PnP 세션(hand_eye/cross)이 열려 있으면 그 run 의 기존 캡처 대비
                # 다양성 판정 — kind 하드코드 금지 (intrinsic 재캘 상시-green 사고
                # 와 같은 클래스)
                run = next(
                    (
                        r
                        for k in _PNP_SESSION_KINDS
                        if (r := self._repo.get_in_progress_run(robot_id, k))
                        is not None
                    ),
                    None,
                )
                quality = (
                    self._evaluate_quality(run.id, det, robot_id)
                    if run is not None and run.id is not None
                    else capture_quality.CaptureQuality(
                        "green", ["첫 자세 — 캡처 권장"]
                    )
                )
                verdict, reasons, tilt = (
                    quality.verdict,
                    quality.reasons,
                    det.tilt_deg,
                )
            else:
                # intrinsic 없거나 미검출 — detect-only (tilt 없음). corners_2d 는
                # overlay 를 위해 intrinsic 없이도 raw ChArUco 픽셀로 채움.
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                ok, ch_corners, ids = calib_board.detect(gray)
                detected = ok
                corner_count = 0 if ids is None else len(ids)
                if ok and ch_corners is not None:
                    corners_2d = ch_corners.reshape(-1, 2).astype(float).tolist()
                tilt = None
                verdict = "red"
                reasons = (
                    ["보드 미검출"] if not ok else ["자세 추정 불가 (intrinsic 확인)"]
                )
        verdict, reasons = self._smooth_preview_verdict(robot_id, verdict, reasons)
        seq = self._preview_seq.get(robot_id, 0)
        self.runtime.publish(
            Calibration.Stream.PREVIEW,
            CalibrationPreview(
                robot_id=robot_id,
                seq=seq,
                timestamp_unix=time.time(),
                detected=detected,
                corner_count=corner_count,
                tilt_deg=tilt,
                verdict=verdict,
                reasons=reasons,
                corners_2d=corners_2d,
                image_width=img_w,
                image_height=img_h,
                board_in_cam=det.board_in_cam if det is not None else None,
            ),
        )
        self._preview_seq[robot_id] = seq + 1

    def _smooth_preview_verdict(
        self, robot_id: str, verdict: str, reasons: list[str]
    ) -> tuple[str, list[str]]:
        """신호등 flap 억제 — 다수결 hysteresis (2026-07-11 실사고: 보드 고정인데
        경계 검출 때문에 프레임마다 red/green 널뜀).

        최근 5프레임 window 에서 같은 판정이 4번 이상 지속돼야 표시를 전환하고,
        그 전까지는 직전 표시 유지 (단일 프레임 미검출/cell 경계 jitter 무시).
        같은 판정 유지 중엔 reasons 만 최신 프레임 것으로 갱신 (cell 카운트 등).
        detected/corners_2d 는 raw 유지 — overlay 는 프레임 그대로가 정직하다.
        """
        dq = self._preview_recent.setdefault(robot_id, deque(maxlen=5))
        dq.append((verdict, tuple(reasons)))
        top, n = Counter(v for v, _ in dq).most_common(1)[0]
        shown = self._preview_shown.get(robot_id)
        if shown is None or top == shown[0] or n >= 4:
            latest = next(r for v, r in reversed(dq) if v == top)
            shown = (top, list(latest))
            self._preview_shown[robot_id] = shown
        return shown

    # ── internal ──────────────────────────────────────────────
    def _evaluate_quality(
        self, run_id: int, det: processing.CaptureDetection, robot_id: str
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
        latest_raw = self._latest_raw.get(robot_id)
        cur_joints = (
            [_raw_to_rad(latest_raw[m]) for m in sorted(latest_raw)]
            if latest_raw
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
