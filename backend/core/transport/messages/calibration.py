"""Calibration 노드 토픽 / 서비스 payload schema.

토픽:
- CALIB_HANDEYE_PREVIEW (publish) — typed 면제 (CHESSBOARD 검출 메타 dict, 동적)

서비스 (request data / response data):
- CALIB_INTRINSIC_START         — EmptyData / EmptyData
- CALIB_INTRINSIC_SAVE          — EmptyData / IntrinsicSaveRes
- CALIB_INTRINSIC_CAPTURE       — EmptyData / IntrinsicCaptureRes
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
    """[캘 시작] — draft run 생성. 기존 in_progress 있으면 reject (frontend 가
    먼저 GET_IN_PROGRESS 로 확인 후 호출)."""

    run_id: int
    pose_count: int  # 항상 0 (방금 시작했으니). frontend state 자체 자체 자체 reset.


# ─── Service: CALIB_HANDEYE_UNDO_LAST_CAPTURE ────────────────────────


class HandeyeUndoLastCaptureRes(StrictModel):
    """[되돌리기] — 마지막 capture 1장 삭제. deleted=False 면 삭제할 거 없음."""

    deleted: bool
    pose_count: int


# ─── Service: CALIB_HANDEYE_LIST_POSES ───────────────────────────────


class HandeyePoseMeta(StrictModel):
    """`HandEyeCalibration.list_poses_meta()` 의 element. 표시용.

    실제 필드는 hand_eye.py 의 list_poses_meta 출력에 따름. 동적 필드가 추가될
    수 있어 extra="allow".
    """

    model_config = ConfigDict(extra="allow")


class HandeyeListPosesRes(StrictModel):
    """`run_id` 가 None 이면 in_progress draft 없음 (사용자 [캘 시작] 안 누름).
    아니면 draft run id — frontend 가 in_progress 여부 / 이어하기 UI 결정 자료."""

    poses: list[HandeyePoseMeta]
    pose_count: int
    run_id: int | None = None


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
    method: str
    joint_offsets_applied: bool
    joint_offsets: list[JointOffsetEntry]
    link_offsets_applied: bool
    link_offsets: list[LinkOffsetEntry]
    sag_offsets_applied: bool
    sag_offsets: list[SagOffsetEntry]
    restart_required: bool


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


class HandeyeObservabilityState(StrictModel):
    """매 capture 후 자동 발행. 캡처된 자세들의 *기하학적 observability* 진단.

    개발자 진단용 metric 이지만 verdict ('A'/'B'/'mid') 만 frontend 표시 (4 metric
    숫자 X). 사용자는:
      - verdict='A' (다양성 충분) → 추가 자세 의미 있음 안내
      - verdict='B' (구조적 부족) → 보드 위치 / 거리 변경 안내
      - verdict='mid' → 중립

    metric 4가지는 docs/observability.md 참조 — 광축 펼침 / tilt / 회전축 spanning
    / wrist roll.
    """

    timestamp: float
    pose_count: int
    axis_spread_deg: float
    tilt_min_deg: float
    tilt_max_deg: float
    tilt_std_deg: float
    tilt_in_range_count: int
    rotation_axis_ratio: float
    wrist_roll_range_raw: int
    verdict: str  # 'A' | 'B' | 'mid'


class HandeyeParamObservabilityState(StrictModel):
    """매 compute(physical_sag) 후 발행 — *parameter별* 식별성 + staged gating 결과.

    위 HandeyeObservabilityState(geometry A/B/mid)와 별개. BA 정보행렬(Fisher)에서
    블록(handeye_rot / handeye_trans / joint_offset / link / sag)별 식별성 score
    ∈[0,1] + verdict(OK/WEAK/INSUFFICIENT) 산출. unlocked = gate 통과해 BA 가 실제
    추정한 블록 (나머지는 freeze=정보부족). docs/handeye_ux_solver_v3_plan.md §3.

    frontend 는 블록별 색 dot (수치 노출 X) — "어느 보정값이 잘 잡혔나 / 자세 보강 필요".
    """

    timestamp: float
    pose_count: int
    scores: dict[str, float]  # block → score ∈ [0,1]
    verdicts: dict[str, str]  # block → "OK" | "WEAK" | "INSUFFICIENT"
    unlocked: list[str]  # gate 통과 블록 (joint_offset / link / sag 중)


class HandeyeBaStatus(StrictModel):
    """BA 진행 상태 — frontend spinner 용.

    state: "running" (BA 시작), "done" (성공, σ 결과는 SIGMA topic 자세 자리),
           "failed" (예외/수렴 실패).
    mode: "standard" | "extended" | "physical_sag" — 어떤 BA 인지.
    """

    timestamp: float
    state: str  # "running" | "done" | "failed"
    mode: str


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


# ─── Service: CALIB_HANDEYE_BEGIN_REFINEMENT ──────────────────────────────


class BeginRefinementReq(StrictModel):
    """Multi-start BA 명시 트리거 — random init 다중 시도 → 가장 좋은 σ 선택.

    Local minimum 자리 escape. 사용자가 saturate 알림 받고 시도, 또는
    [수동 모드 종료] 시점 자동 트리거.
    """

    n_starts: int = 10
    mode: str = "physical_sag"


class BeginRefinementRes(StrictModel):
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
