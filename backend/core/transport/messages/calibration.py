"""Calibration 노드 토픽 / 서비스 payload schema.

토픽:
- CALIB_HANDEYE_PREVIEW (publish) — typed 면제 (CHESSBOARD 검출 메타 dict, 동적)

서비스 (request data / response data):
- CALIB_INTRINSIC_START         — EmptyData / EmptyData
- CALIB_INTRINSIC_SAVE          — EmptyData / IntrinsicSaveRes
- CALIB_CAPTURE                 — CalibCaptureReq / CalibCaptureRes
- CALIB_HANDEYE_CAPTURE         — EmptyData / HandeyeCaptureRes
- CALIB_HANDEYE_RESET           — EmptyData / HandeyeResetRes
- CALIB_HANDEYE_COMPUTE         — typed 면제 (legacy dict — 응답 ~25 필드 동적)
- CALIB_HANDEYE_COMMIT          — EmptyData / HandeyeCommitRes
- CALIB_HANDEYE_LIST_POSES      — EmptyData / HandeyeListPosesRes
- CALIB_HANDEYE_PREVIEW_ENABLE  — HandeyePreviewEnableReq / HandeyePreviewEnableRes
- CALIB_HANDEYE_THRESHOLDS      — typed 면제 (legacy dict — thresholds.as_dict() free-form)
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


# ─── Service: CALIB_CAPTURE ──────────────────────────────────────────


class CalibCaptureReq(StrictModel):
    """mode: intrinsic 만 현재 지원. 추후 handeye 모드 추가 시 Literal 확장."""

    mode: str = "intrinsic"


class CalibCaptureRes(StrictModel):
    detected: bool
    captured_count: int
    preview: str  # base64 JPEG


# ─── Service: CALIB_HANDEYE_CAPTURE ──────────────────────────────────


class HandeyeCaptureRes(StrictModel):
    """detected=False 인 경우도 success=True (가이드 의미). pose_count 는 누적."""

    detected: bool
    pose_count: int


# ─── Service: CALIB_HANDEYE_RESET ────────────────────────────────────


class HandeyeResetRes(StrictModel):
    pose_count: int


# ─── Service: CALIB_HANDEYE_LIST_POSES ───────────────────────────────


class HandeyePoseMeta(StrictModel):
    """`HandEyeCalibration.list_poses_meta()` 의 element. 표시용.

    실제 필드는 hand_eye.py 의 list_poses_meta 출력에 따름. 동적 필드가 추가될
    수 있어 extra="allow".
    """

    model_config = ConfigDict(extra="allow")


class HandeyeListPosesRes(StrictModel):
    poses: list[HandeyePoseMeta]
    pose_count: int


# ─── Service: CALIB_HANDEYE_COMMIT ───────────────────────────────────


class JointOffsetEntry(StrictModel):
    motor_id: int
    offset_rad: float


class LinkOffsetEntry(StrictModel):
    motor_id: int
    trans_m: list[float]  # (3,)
    rot_rad: list[float]  # (3,)


class SagOffsetEntry(StrictModel):
    motor_id: int
    k_rad_per_m: float


class HandeyeCommitRes(StrictModel):
    path: str
    method: str
    joint_offsets_applied: bool
    joint_offsets: list[JointOffsetEntry]
    link_offsets_applied: bool
    link_offsets: list[LinkOffsetEntry]
    sag_offsets_applied: bool
    sag_offsets: list[SagOffsetEntry]
    restart_required: bool


# ─── Service: CALIB_HANDEYE_PREVIEW_ENABLE ───────────────────────────


class HandeyePreviewEnableReq(StrictModel):
    enabled: bool = False


class HandeyePreviewEnableRes(StrictModel):
    enabled: bool


# ─── Not typed (legacy dict handler 유지) ────────────────────────────
# CALIB_HANDEYE_COMPUTE — 응답 ~25 동적 필드 (per_pose_residual, method_compare,
#   coach, recommendations, joint_offset_delta, link_trans_delta, sag_offset_delta
#   …) + BA mode 분기에 따라 필드 가/감. typed_messaging.md §마이그레이션 사유 인용.
# CALIB_HANDEYE_THRESHOLDS — thresholds.as_dict() 가 free-form, 프론트 mount 1회 fetch.
# CALIB_HANDEYE_PREVIEW topic — CHESSBOARD 검출 메타 dict (corners / tilt 등 optional).

# legacy form 으로 두려면 free-form Any 타입 hint 만 export — 실제 모델 없음.
HandeyeComputeData = dict[str, Any]
HandeyeThresholdsData = dict[str, Any]
