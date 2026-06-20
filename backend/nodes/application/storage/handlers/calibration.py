"""Calibration 도메인 Zenoh service handler group.

storage_node 의 lifecycle + composition root 안 register(node) 자리 호출되어
service handler 들을 node 에 등록. 본 group 의 책임 = 캘리브레이션 도메인
(run / result / capture / activate / draft flow / finalize) 의 Zenoh 노출.

13 service:
- GET_ACTIVE / LIST / LIST_RUNS / COMMIT / ACTIVATE
- NEW_CAL_RUN / APPEND_CAPTURE / DELETE_LAST_CAPTURE
- GET_IN_PROGRESS_RUN / DELETE_CAL_RUN / MARK_CAL_RUN_READY / FINALIZE_CAL_RUN
- LIST_RUN_CAPTURES
+ STORAGE_CALIBRATION_INVALIDATED topic publish (ACTIVATE 후 1회)

APPEND_CAPTURE + DELETE_* 자리는 ObjectStore blob 자리 (color JPEG + zstd depth)
같이 다룸 — RDB row + ObjectStore put 한 transaction 의미. RDB rollback 시 orphan
blob 자리 cleanup 자리도 본 handler 가 담당.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from typing import Any

import cv2
import numpy as np

from core.transport.application_node import ApplicationNode
from modules.camera import depth_frame as dframe
from core.transport.messages.base import EmptyData, ServiceRequest, ServiceResponse
from modules.calibration.persistence_models import (
    CalibrationCaptureArtifactRecord,
)
from core.transport.messages.storage import (
    ActivateCalibrationReq,
    ActivateCalibrationRes,
    AppendCalibrationCaptureReq,
    AppendCalibrationCaptureRes,
    CalibrationInvalidated,
    CalibrationRunSummary,
    CommitCalibrationReq,
    CommitCalibrationRes,
    CreateCalibrationRunReq,
    CreateCalibrationRunRes,
    DeleteCalibrationRunReq,
    DeleteLastCalibrationCaptureReq,
    DeleteLastCalibrationCaptureRes,
    FinalizeCalibrationRunReq,
    FinalizeCalibrationRunRes,
    GetActiveCalibrationReq,
    GetActiveCalibrationRes,
    GetInProgressCalibrationRunReq,
    GetInProgressCalibrationRunRes,
    ListCalibrationRunsReq,
    ListCalibrationRunsRes,
    ListCalibrationsReq,
    ListCalibrationsRes,
    ListRunCapturesReq,
    ListRunCapturesRes,
    MarkCalibrationRunReadyReq,
    MarkCalibrationRunReadyRes,
)
from core.transport.topic_map import Service, Topic
from modules.storage.registry import StorageRegistry

logger = logging.getLogger(__name__)


class CalibrationHandlers:
    """캘리브레이션 도메인 service handler 묶음."""

    def __init__(self, reg: StorageRegistry, publish: Callable[[str, Any], None]) -> None:
        self._reg = reg
        self._publish = publish

    def register(self, node: ApplicationNode) -> None:
        node.create_service(
            Service.STORAGE_GET_ACTIVE_CALIBRATION,
            GetActiveCalibrationReq,
            GetActiveCalibrationRes,
            self._srv_get_active,
        )
        node.create_service(
            Service.STORAGE_LIST_CALIBRATIONS,
            ListCalibrationsReq,
            ListCalibrationsRes,
            self._srv_list,
        )
        node.create_service(
            Service.STORAGE_LIST_CALIBRATION_RUNS,
            ListCalibrationRunsReq,
            ListCalibrationRunsRes,
            self._srv_list_runs,
        )
        node.create_service(
            Service.STORAGE_COMMIT_CALIBRATION,
            CommitCalibrationReq,
            CommitCalibrationRes,
            self._srv_commit,
        )
        node.create_service(
            Service.STORAGE_ACTIVATE_CALIBRATION,
            ActivateCalibrationReq,
            ActivateCalibrationRes,
            self._srv_activate,
        )
        node.create_service(
            Service.STORAGE_NEW_CAL_RUN,
            CreateCalibrationRunReq,
            CreateCalibrationRunRes,
            self._srv_new_cal_run,
        )
        node.create_service(
            Service.STORAGE_APPEND_CAPTURE,
            AppendCalibrationCaptureReq,
            AppendCalibrationCaptureRes,
            self._srv_append_capture,
        )
        node.create_service(
            Service.STORAGE_DELETE_LAST_CAPTURE,
            DeleteLastCalibrationCaptureReq,
            DeleteLastCalibrationCaptureRes,
            self._srv_delete_last_capture,
        )
        node.create_service(
            Service.STORAGE_GET_IN_PROGRESS_RUN,
            GetInProgressCalibrationRunReq,
            GetInProgressCalibrationRunRes,
            self._srv_get_in_progress_run,
        )
        node.create_service(
            Service.STORAGE_DELETE_CAL_RUN,
            DeleteCalibrationRunReq,
            EmptyData,
            self._srv_delete_cal_run,
        )
        node.create_service(
            Service.STORAGE_MARK_CAL_RUN_READY,
            MarkCalibrationRunReadyReq,
            MarkCalibrationRunReadyRes,
            self._srv_mark_cal_run_ready,
        )
        node.create_service(
            Service.STORAGE_FINALIZE_CAL_RUN,
            FinalizeCalibrationRunReq,
            FinalizeCalibrationRunRes,
            self._srv_finalize_cal_run,
        )
        node.create_service(
            Service.STORAGE_LIST_RUN_CAPTURES,
            ListRunCapturesReq,
            ListRunCapturesRes,
            self._srv_list_run_captures,
        )

    # ─── service handlers ─────────────────────────────────────

    def _srv_get_active(
        self, req: ServiceRequest[GetActiveCalibrationReq]
    ) -> ServiceResponse[GetActiveCalibrationRes]:
        with self._reg.rdb.session() as repos:
            record = repos.calibration.get_active_result(
                req.data.robot_id, req.data.kind
            )
        return ServiceResponse(
            success=True,
            data=GetActiveCalibrationRes(
                found=record is not None, result=record
            ),
        )

    def _srv_list(
        self, req: ServiceRequest[ListCalibrationsReq]
    ) -> ServiceResponse[ListCalibrationsRes]:
        with self._reg.rdb.session() as repos:
            records = repos.calibration.list_results(
                req.data.robot_id, req.data.kind, req.data.limit
            )
        return ServiceResponse(success=True, data=ListCalibrationsRes(results=records))

    def _srv_list_runs(
        self, req: ServiceRequest[ListCalibrationRunsReq]
    ) -> ServiceResponse[ListCalibrationRunsRes]:
        with self._reg.rdb.session() as repos:
            rows = repos.calibration.list_runs(req.data.robot_id, req.data.limit)
        summaries = [
            CalibrationRunSummary(run=run, results=results) for run, results in rows
        ]
        return ServiceResponse(
            success=True, data=ListCalibrationRunsRes(runs=summaries)
        )

    def _srv_commit(
        self, req: ServiceRequest[CommitCalibrationReq]
    ) -> ServiceResponse[CommitCalibrationRes]:
        with self._reg.rdb.session() as repos:
            run_id, result_ids = repos.calibration.commit(
                req.data.run, req.data.results, req.data.captures
            )
        logger.info(
            "COMMIT: run_id=%d, result_ids=%s (robot=%s, results=%d, captures=%d)",
            run_id,
            result_ids,
            req.data.run.robot_id,
            len(req.data.results),
            len(req.data.captures),
        )
        return ServiceResponse(
            success=True,
            data=CommitCalibrationRes(run_id=run_id, result_ids=result_ids),
        )

    def _srv_activate(
        self, req: ServiceRequest[ActivateCalibrationReq]
    ) -> ServiceResponse[ActivateCalibrationRes]:
        try:
            with self._reg.rdb.session() as repos:
                activated = repos.calibration.activate_result(req.data.result_id)
        except KeyError as e:
            return ServiceResponse(success=False, message=str(e))

        # transaction commit 직후 invalidation publish — caller (calibration_node)
        # service 응답 받기 전에도 노드들이 refetch 시작 가능.
        assert activated.id is not None  # activate_result 반환은 id 항상 있음
        self._publish(
            Topic.STORAGE_CALIBRATION_INVALIDATED,
            CalibrationInvalidated(
                robot_id=activated.robot_id,
                kind=activated.kind,
                result_id=activated.id,
                timestamp=time.time(),
            ),
        )
        logger.info(
            "ACTIVATE: result_id=%d (robot=%s, kind=%s) — invalidation publish",
            activated.id,
            activated.robot_id,
            activated.kind,
        )
        return ServiceResponse(
            success=True, data=ActivateCalibrationRes(result=activated)
        )

    # ─── Draft run handlers (사용자 [캘 시작] flow) ───────────

    def _srv_new_cal_run(
        self, req: ServiceRequest[CreateCalibrationRunReq]
    ) -> ServiceResponse[CreateCalibrationRunRes]:
        run = req.data.run
        if run.kind is None:
            return ServiceResponse(
                success=False, message="run.kind 필수 (intrinsic / hand_eye)"
            )
        with self._reg.rdb.session() as repos:
            # 같은 (robot_id, kind) 의 기존 in_progress 가 있으면 거부 — frontend 는
            # 먼저 GET_IN_PROGRESS 로 확인해야 함.
            existing = repos.calibration.get_in_progress_run(run.robot_id, run.kind)
            if existing is not None:
                existing_run, _ = existing
                return ServiceResponse(
                    success=False,
                    message=(
                        f"이미 in_progress run 있음 (robot={run.robot_id}, "
                        f"kind={run.kind}, run_id={existing_run.id})"
                    ),
                )
            run_id = repos.calibration.new_run(run)
        logger.info(
            "NEW_CAL_RUN: run_id=%d (robot=%s, kind=%s, algorithm=%s)",
            run_id, run.robot_id, run.kind, run.algorithm,
        )
        return ServiceResponse(success=True, data=CreateCalibrationRunRes(run_id=run_id))

    def _srv_append_capture(
        self, req: ServiceRequest[AppendCalibrationCaptureReq]
    ) -> ServiceResponse[AppendCalibrationCaptureRes]:
        capture = req.data.capture
        blob_bytes = bytes(req.data.blob_bytes)
        robot_id = req.data.robot_id

        # 1. ObjectStore 자리 primary + debug artifacts 먼저 put. RDB 실패 시 orphan
        # blob 자리 cleanup 자리 (반대 case = capture row 있고 blob 없음 = broken).
        artifacts: list[CalibrationCaptureArtifactRecord] = []
        primary_blob_key: str | None = None
        if blob_bytes:
            primary_blob_key = (
                f"calib_captures/{robot_id}/{capture.run_id}/"
                f"{capture.pose_index:03d}.bin"
            )
            try:
                self._reg.objects.put(primary_blob_key, blob_bytes)
            except Exception as e:
                logger.exception("ObjectStore.put 실패 (key=%s)", primary_blob_key)
                return ServiceResponse(
                    success=False, message=f"blob put 실패: {e}"
                )
            now = time.time()
            artifacts.append(
                CalibrationCaptureArtifactRecord(  # type: ignore[call-arg]
                    capture_id=0,  # repo 가 INSERT 시 채움
                    kind="primary",
                    blob_key=primary_blob_key,
                    size_bytes=len(blob_bytes),
                    content_type="application/octet-stream",
                    created_at=now,
                )
            )
            # 디버깅 artifacts — primary 옆에 color.jpg / depth.png / depth_vis.png /
            # .ply. 실패 시 그 artifact 만 누락 (capture 자체는 성공).
            artifacts.extend(
                self._save_debug_artifacts(primary_blob_key, blob_bytes, now)
            )

        # 2. RDB row + 자식 artifact rows atomic INSERT.
        try:
            with self._reg.rdb.session() as repos:
                capture_id = repos.calibration.append_capture(capture, artifacts)
        except (KeyError, ValueError) as e:
            # RDB 실패 시 직전 put 한 blob 자리 cleanup — orphan 차단.
            for a in artifacts:
                try:
                    self._reg.objects.delete(a.blob_key)
                except Exception:
                    logger.exception(
                        "RDB rollback 후 orphan cleanup 실패: %s", a.blob_key,
                    )
            return ServiceResponse(success=False, message=str(e))

        return ServiceResponse(
            success=True,
            data=AppendCalibrationCaptureRes(
                capture_id=capture_id, blob_key=primary_blob_key
            ),
        )

    def _srv_delete_last_capture(
        self, req: ServiceRequest[DeleteLastCalibrationCaptureReq]
    ) -> ServiceResponse[DeleteLastCalibrationCaptureRes]:
        with self._reg.rdb.session() as repos:
            result = repos.calibration.delete_last_capture(req.data.run_id)
        if result is None:
            return ServiceResponse(
                success=True,
                data=DeleteLastCalibrationCaptureRes(deleted_pose_index=None),
            )
        pose_index, artifacts = result
        for a in artifacts:
            try:
                self._reg.objects.delete(a.blob_key)
            except Exception:
                logger.debug("artifact %s delete skip (%s)", a.kind, a.blob_key)

        return ServiceResponse(
            success=True,
            data=DeleteLastCalibrationCaptureRes(deleted_pose_index=pose_index),
        )

    def _srv_get_in_progress_run(
        self, req: ServiceRequest[GetInProgressCalibrationRunReq]
    ) -> ServiceResponse[GetInProgressCalibrationRunRes]:
        with self._reg.rdb.session() as repos:
            result = repos.calibration.get_in_progress_run(
                req.data.robot_id, req.data.kind
            )
        if result is None:
            return ServiceResponse(
                success=True, data=GetInProgressCalibrationRunRes(found=False)
            )
        run, captures = result
        return ServiceResponse(
            success=True,
            data=GetInProgressCalibrationRunRes(found=True, run=run, captures=captures),
        )

    def _srv_delete_cal_run(
        self, req: ServiceRequest[DeleteCalibrationRunReq]
    ) -> ServiceResponse[EmptyData]:
        # ObjectStore blob 자리 RDB cascade 안 따라옴 → 별도 cleanup.
        with self._reg.rdb.session() as repos:
            artifacts = repos.calibration.list_run_artifacts(req.data.run_id)
            repos.calibration.delete_run(req.data.run_id)
        for a in artifacts:
            try:
                self._reg.objects.delete(a.blob_key)
            except Exception:
                logger.debug(
                    "artifact %s delete skip (%s)", a.kind, a.blob_key,
                )
        logger.info(
            "DELETE_CAL_RUN: run_id=%d (artifacts=%d)",
            req.data.run_id, len(artifacts),
        )
        return ServiceResponse(success=True, data=EmptyData())

    def _srv_mark_cal_run_ready(
        self, req: ServiceRequest[MarkCalibrationRunReadyReq]
    ) -> ServiceResponse[MarkCalibrationRunReadyRes]:
        try:
            with self._reg.rdb.session() as repos:
                run = repos.calibration.mark_run_ready(req.data.run_id)
        except (KeyError, ValueError) as e:
            return ServiceResponse(success=False, message=str(e))
        logger.info(
            "MARK_CAL_RUN_READY: run_id=%d (robot=%s, kind=%s)",
            req.data.run_id, run.robot_id, run.kind,
        )
        return ServiceResponse(
            success=True, data=MarkCalibrationRunReadyRes(run=run)
        )

    def _srv_list_run_captures(
        self, req: ServiceRequest[ListRunCapturesReq]
    ) -> ServiceResponse[ListRunCapturesRes]:
        with self._reg.rdb.session() as repos:
            captures = repos.calibration.list_captures(req.data.run_id)
        return ServiceResponse(
            success=True, data=ListRunCapturesRes(captures=captures)
        )

    def _srv_finalize_cal_run(
        self, req: ServiceRequest[FinalizeCalibrationRunReq]
    ) -> ServiceResponse[FinalizeCalibrationRunRes]:
        try:
            with self._reg.rdb.session() as repos:
                result_ids = repos.calibration.finalize_run(
                    req.data.run_id, req.data.results, req.data.capture_residuals
                )
        except KeyError as e:
            return ServiceResponse(success=False, message=str(e))
        logger.info(
            "FINALIZE_CAL_RUN: run_id=%d, result_ids=%s",
            req.data.run_id, result_ids,
        )
        return ServiceResponse(
            success=True, data=FinalizeCalibrationRunRes(result_ids=result_ids),
        )

    # ─── Debug artifact helpers ──────────────────────────────────
    # primary .bin (transport / offline script 용) 외에 같은 폴더에 color.jpg /
    # depth.png / depth_vis.png / .ply 저장. 사람이 탐색기에서 capture 품질 바로 확인.

    def _save_debug_artifacts(
        self, primary_blob_key: str, blob_bytes: bytes, now: float
    ) -> list[CalibrationCaptureArtifactRecord]:
        """primary .bin 옆에 color.jpg + depth.png + depth_vis.png + .ply 저장.

        Decode 실패 / 인코딩 실패 자리 그 artifact 만 누락 — 다른 자리 계속. 반환:
        성공한 artifact 들의 record list (capture_id=0 임시, repo 가 INSERT 시 채움).
        """
        if not primary_blob_key.endswith(".bin"):
            return []
        stem = primary_blob_key[:-4]
        try:
            df = dframe.decode(blob_bytes)
        except Exception:
            logger.warning(
                "debug artifacts decode 실패 (%s) — primary 만 저장", primary_blob_key,
            )
            return []

        results: list[CalibrationCaptureArtifactRecord] = []

        def _put(kind: str, suffix: str, content_type: str, data: bytes) -> None:
            k = stem + suffix
            try:
                self._reg.objects.put(k, data)
                results.append(
                    CalibrationCaptureArtifactRecord(  # type: ignore[call-arg]
                        capture_id=0,
                        kind=kind,  # type: ignore[arg-type]
                        blob_key=k,
                        size_bytes=len(data),
                        content_type=content_type,
                        created_at=now,
                    )
                )
            except Exception:
                logger.exception("artifact %s put 실패 (%s)", kind, k)

        # color.jpg.
        try:
            ok, jpg = cv2.imencode(
                ".jpg", df.color_bgr, [cv2.IMWRITE_JPEG_QUALITY, 90]
            )
            if ok:
                _put("color", ".color.jpg", "image/jpeg", jpg.tobytes())
        except Exception:
            logger.exception("artifact color encode 실패")

        # depth.png — 16-bit raw.
        try:
            ok, png = cv2.imencode(".png", df.depth_z16)
            if ok:
                _put("depth", ".depth.png", "image/png", png.tobytes())
        except Exception:
            logger.exception("artifact depth encode 실패")

        # depth_vis.png — 8-bit colorized.
        try:
            valid = df.depth_z16[df.depth_z16 > 0]
            if valid.size > 0:
                z_min = max(int(np.percentile(valid, 2)), 1)
                z_max = max(int(np.percentile(valid, 98)), z_min + 1)
                norm = np.clip(
                    (df.depth_z16.astype(np.float32) - z_min)
                    / (z_max - z_min) * 255,
                    0, 255,
                ).astype(np.uint8)
                norm[df.depth_z16 == 0] = 0
                vis = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
                ok, vis_png = cv2.imencode(".png", vis)
                if ok:
                    _put("depth_vis", ".depth_vis.png", "image/png", vis_png.tobytes())
        except Exception:
            logger.exception("artifact depth_vis encode 실패")

        # .ply — binary color point cloud.
        try:
            ply_bytes = _make_ply_binary_rgb(df)
            if ply_bytes:
                _put("ply", ".ply", "application/octet-stream", ply_bytes)
        except Exception:
            logger.exception("artifact ply encode 실패")

        return results


def _make_ply_binary_rgb(df) -> bytes | None:
    """depth + color (BGR) → binary little-endian PLY (xyz + rgb).

    invalid depth (z=0) 픽셀 제외. ~300-500K points / 1280×720 frame.
    """
    if df.depth_z16.size == 0:
        return None
    v, u = np.where(df.depth_z16 > 0)
    if v.size == 0:
        return None
    z = df.depth_z16[v, u].astype(np.float32) * float(df.depth_scale)
    x = (u.astype(np.float32) - float(df.cx)) * z / float(df.fx)
    y = (v.astype(np.float32) - float(df.cy)) * z / float(df.fy)
    # BGR → RGB (PLY convention).
    bgr = df.color_bgr[v, u]
    if bgr.ndim != 2 or bgr.shape[1] != 3:
        return None

    n = v.size
    header = (
        f"ply\nformat binary_little_endian 1.0\nelement vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    ).encode("ascii")

    dtype = np.dtype([
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
        ("r", "u1"), ("g", "u1"), ("b", "u1"),
    ])
    arr = np.empty(n, dtype=dtype)
    arr["x"] = x
    arr["y"] = y
    arr["z"] = z
    arr["r"] = bgr[:, 2]
    arr["g"] = bgr[:, 1]
    arr["b"] = bgr[:, 0]
    return header + arr.tobytes()
