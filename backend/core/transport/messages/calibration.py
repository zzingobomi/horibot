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
    coverage_count: int = 0  # 3×3 grid 중 채운 cell 수 (0..9)
    coverage_cells: list[list[int]] = []  # 채운 cell 좌표 [[gx, gy], ...]


# ─── Service: CALIB_CAPTURE ──────────────────────────────────────────


class CalibCaptureReq(StrictModel):
    """mode: intrinsic 만 현재 지원. 추후 handeye 모드 추가 시 Literal 확장."""

    mode: str = "intrinsic"


class CalibCaptureRes(StrictModel):
    detected: bool
    captured_count: int
    preview: str  # base64 JPEG
    hint: str = ""  # 사용자 안내 — 성공/실패 분기별 사유
    coverage_count: int = 0  # 누적 3×3 grid coverage (intrinsic 만 의미)


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


# ─── Service: CALIB_BACKUP_LIST / RESTORE ────────────────────────────


class BackupEntry(StrictModel):
    """`.history/` 안 한 snapshot 의 picker 표시용 메타.

    sigma_*/capture_count/ba_mode 는 commit 시점에 박힘. tag 는 "pre-commit" /
    "pre-restore" 등 origin 구분.
    """

    timestamp: str
    tag: str
    sigma_rot_deg: float | None = None
    sigma_t_mm: float | None = None
    capture_count: int | None = None
    ba_mode: str | None = None


class BackupListRes(StrictModel):
    snapshots: list[BackupEntry]


class BackupRestoreReq(StrictModel):
    timestamp: str


class BackupRestoreRes(StrictModel):
    restored_timestamp: str
    restart_required: bool  # link_offsets 복원 시 URDF patch 재적용 필요


# ─── Topic: CALIB_HANDEYE_SIGMA ──────────────────────────────────────


class AxisDistributionEntry(StrictModel):
    """coach.axis_distributions 원소. UI 의 자세 다양성 표 + low_diversity 색 분기.

    motor_id ─ J{motor_id} 로 표시. std_deg ─ 캡처 자세의 표준편차.
    threshold_deg ─ thresholds.JOINT_DIVERSITY_THRESHOLD_DEG 의 해당 axis 값.
    is_low_diversity ─ std < threshold. suggested_deg ─ planner 가 권장하는 다음
    각도 (있으면 NextPoseCard 와 별개로 status 패널에서 안내 가능).
    """

    motor_id: int
    name_ko: str
    std_deg: float
    min_deg: float
    max_deg: float
    threshold_deg: float
    is_low_diversity: bool
    motor_limit_min_deg: float
    motor_limit_max_deg: float
    suggested_deg: float | None
    suggestion_text: str


class HandeyeSigmaState(StrictModel):
    """capture 후 자동 BA / 수동 COMPUTE 마다 publish. frontend σ live 표시.

    BA 실패 / 포즈 부족 시에는 publish 안 함 (직전 σ 유지 또는 frontend 가 unknown).

    `axis_distributions` 는 4-상태 verdict (good / narrow_sigma_good / needs_work /
    bad) 와 묶여 UI 의 자세 다양성 표시 → 사용자가 *어느 axis 가 부족한지* 매 capture
    후 즉시 확인. trauma source 의 root cause fix.
    """

    timestamp: float
    sigma_rot_deg: float | None
    sigma_t_mm: float | None
    pose_count: int
    ba_mode: str | None
    ba_converged: bool
    coach_verdict: str | None
    joint_offset_estimated: bool
    link_offset_estimated: bool
    sag_offset_estimated: bool
    axis_distributions: list[AxisDistributionEntry] = []


# ─── Service: CALIB_HANDEYE_PREVIEW_ENABLE ───────────────────────────


class HandeyePreviewEnableReq(StrictModel):
    enabled: bool = False


class HandeyePreviewEnableRes(StrictModel):
    enabled: bool


# ─── Service: CALIB_HANDEYE_RECOMMENDATION_FAIL ──────────────────────


class RecommendationFailReq(StrictModel):
    """사용자 명시 신호 — 추천 자세 fail 기록. 다음 추천 생성 시 제외.

    카테고리:
      - "not_visible": [이동] 후 보드 화면 밖
      - "red": 도달했고 보이지만 한 장 단위 hint 빨강 (tilt extreme / 코너 부족)
      - "motion_fail": 도달 실패 (IK 통과했지만 motion 자체 fail)
    """

    anchor_id: str
    category: str  # "not_visible" | "red" | "motion_fail"


class RecommendationFailRes(StrictModel):
    excluded_count: int


# ─── Service: CALIB_HANDEYE_MULTI_START ──────────────────────────────


class MultiStartReq(StrictModel):
    """Multi-start BA 명시 트리거 — random init 다중 시도 → 가장 좋은 σ 선택.

    Local minimum 자리 escape. 사용자가 saturate 알림 받고 시도, 또는
    [수동 모드 종료] 시점 자동 트리거.
    """

    n_starts: int = 10
    mode: str = "physical_sag"


class MultiStartRes(StrictModel):
    n_tried: int
    n_converged: int
    sigma_rot_deg: float | None
    sigma_t_mm: float | None
    improvement_rot_deg: float | None
    improvement_t_mm: float | None


# ─── Not typed (legacy dict handler 유지) ────────────────────────────
# CALIB_HANDEYE_COMPUTE — 응답 ~25 동적 필드 (per_pose_residual, method_compare,
#   coach, recommendations, joint_offset_delta, link_trans_delta, sag_offset_delta
#   …) + BA mode 분기에 따라 필드 가/감. typed_messaging.md §마이그레이션 사유 인용.
# CALIB_HANDEYE_THRESHOLDS — thresholds.as_dict() 가 free-form, 프론트 mount 1회 fetch.
# CALIB_HANDEYE_PREVIEW topic — CHESSBOARD 검출 메타 dict (corners / tilt 등 optional).

# legacy form 으로 두려면 free-form Any 타입 hint 만 export — 실제 모델 없음.
HandeyeComputeData = dict[str, Any]
HandeyeThresholdsData = dict[str, Any]
