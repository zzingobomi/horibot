"""Calibration 노드 토픽 / 서비스 payload schema — capture-only 시나리오.

handeye_ux_solver_v3 의 online BA / 추천 / observability 자리 전부 폐기. 새 시나리오 =
[캘 시작] → 자세 자유 + traffic light → [캡처] N장 (raw blob + record) → [세션 종료]
(in_progress → ready_for_analysis) → offline Python 스크립트가 BA + commit.

토픽:
- CALIB_HANDEYE_PREVIEW (publish) — typed 면제 (free-form dict: detected / tilt /
  pose_count / session_active / capture_verdict (G/Y/R) / capture_reasons /
  corners_2d / marker_outlines)

서비스 (request data / response data):
- CALIB_INTRINSIC_START         — EmptyData / EmptyData
- CALIB_INTRINSIC_SAVE          — EmptyData / IntrinsicSaveRes
- CALIB_INTRINSIC_CAPTURE       — EmptyData / IntrinsicCaptureRes
- CALIB_HANDEYE_START           — EmptyData / HandeyeStartRes
- CALIB_HANDEYE_CAPTURE         — EmptyData / HandeyeCaptureRes
- CALIB_HANDEYE_RESET           — EmptyData / HandeyeResetRes
- CALIB_HANDEYE_UNDO_LAST_CAPTURE — EmptyData / HandeyeUndoLastCaptureRes
- CALIB_HANDEYE_FINALIZE        — EmptyData / HandeyeFinalizeRes (세션 종료)
- CALIB_HANDEYE_LIST_POSES      — EmptyData / HandeyeListPosesRes
- CALIB_HANDEYE_PREVIEW_ENABLE  — HandeyePreviewEnableReq / HandeyePreviewEnableRes
- CALIB_HANDEYE_THRESHOLDS      — typed 면제 (legacy dict, thresholds.as_dict())
"""

from __future__ import annotations

from typing import Any

from pydantic import ConfigDict

from core.transport.messages.base import StrictModel


# ─── Service: CALIB_INTRINSIC_SAVE ───────────────────────────────────


class IntrinsicSaveRes(StrictModel):
    rms_error: float
    camera_matrix: list[list[float]]
    dist_coeffs: list[list[float]]  # shape (1, 5)
    captured_count: int
    coverage_count: int = 0  # 3×3 grid 중 채운 cell 수 (0..9)
    coverage_cells: list[list[int]] = []  # 채운 cell 좌표 [[gx, gy], ...]


# ─── Service: CALIB_INTRINSIC_CAPTURE ────────────────────────────────


class IntrinsicCaptureRes(StrictModel):
    detected: bool
    captured_count: int
    preview: str  # base64 JPEG
    hint: str = ""  # 사용자 안내 — 성공/실패 분기별 사유
    coverage_count: int = 0  # 누적 3×3 grid coverage


# ─── Service: CALIB_HANDEYE_CAPTURE ──────────────────────────────────


class HandeyeCaptureRes(StrictModel):
    """detected=False 인 경우도 success=True (가이드 의미). pose_count 는 누적."""

    detected: bool
    pose_count: int


# ─── Service: CALIB_HANDEYE_RESET ────────────────────────────────────


class HandeyeResetRes(StrictModel):
    pose_count: int


# ─── Service: CALIB_HANDEYE_START ────────────────────────────────────


class HandeyeStartRes(StrictModel):
    """[캘 시작] — draft run 생성. 기존 in_progress 있으면 reject."""

    run_id: int
    pose_count: int  # 항상 0


# ─── Service: CALIB_HANDEYE_UNDO_LAST_CAPTURE ────────────────────────


class HandeyeUndoLastCaptureRes(StrictModel):
    """[되돌리기] — 마지막 capture 1장 + blob 삭제. deleted=False 면 삭제할 거 없음."""

    deleted: bool
    pose_count: int


# ─── Service: CALIB_HANDEYE_FINALIZE ─────────────────────────────────


class HandeyeFinalizeRes(StrictModel):
    """[세션 종료] — in_progress → ready_for_analysis. immutable 전이.

    이후 capture 불가. offline Python 스크립트가 captures + blobs read → BA →
    finalize_run + activate. frontend 무관.
    """

    run_id: int
    pose_count: int


# ─── Service: CALIB_HANDEYE_LIST_POSES ───────────────────────────────


class HandeyePoseMeta(StrictModel):
    """capture 1장 요약 — frontend PoseList 표시용.

    `extra="allow"` 로 향후 진단 필드 추가에 유연 (e.g. reproj_rms_px).
    """

    model_config = ConfigDict(extra="allow")
    pose_index: int
    tilt_deg: float | None = None


class HandeyeListPosesRes(StrictModel):
    """`run_id` 가 None 이면 in_progress draft 없음."""

    poses: list[HandeyePoseMeta]
    pose_count: int
    run_id: int | None = None


# ─── Service: CALIB_HANDEYE_PREVIEW_ENABLE ───────────────────────────


class HandeyePreviewEnableReq(StrictModel):
    enabled: bool = False


class HandeyePreviewEnableRes(StrictModel):
    enabled: bool


# ─── Not typed (legacy dict handler 유지) ────────────────────────────
# CALIB_HANDEYE_THRESHOLDS — thresholds.as_dict() 가 free-form, mount 1회 fetch.
# CALIB_HANDEYE_PREVIEW topic — preview 페이로드 free-form (capture_verdict, corners,
#   marker_outlines 등 동적).

HandeyeThresholdsData = dict[str, Any]
HandeyePreviewPayload = dict[str, Any]
