"""Calibration 의 persistence 모델 — DB / object store 에 저장되는 row 의 Pydantic shape.

본 파일은 calibration 도메인의 일부 — "캘이 무엇을 저장하는지" 의 소유권은 캘에
있음. storage 모듈은 "어떻게 저장하는지" (SQL connection / blob put-get) 만 다룸.
캘 schema 변경 (컬럼 추가 / kind 추가 등) 은 본 파일만 건드림 — storage/ 안 건드림.

`result_models.py` 와의 구분:
- `result_models.py`     — 계산 결과 shape (HandEyeResultData ...)
- `persistence_models.py` — DB row shape (CalibrationResultRecord ...)

3 테이블 (docs/storage_layer.md §6):
- calibration_runs       — 한 번의 캘 실행 (immutable, run_id)
- calibration_results    — Run 의 산출물 (kind 별, is_active 토글). Discriminated
                           union — kind 가 result_data 의 shape 을 결정 (Pydantic
                           이 자동 validate). drift 차단.
- calibration_captures   — Evidence (per-pose 자세 + residual + IRLS weight)
"""

from __future__ import annotations

from typing import Annotated, Any, Literal

from pydantic import Field, TypeAdapter

from core.transport.messages.base import StrictModel
from modules.calibration.result_models import (
    HandEyeResultData,
    IntrinsicResultData,
    JointOffsetResultData,
    LinkOffsetResultData,
    SagOffsetResultData,
)


# 캘 5종. SSOT — RDB row 의 kind 컬럼 값과 일치.
CalibrationKind = Literal[
    "intrinsic",
    "hand_eye",
    "joint_offset",
    "link_offset",
    "sag",
]


# Run status — capture flow 의 4-stage:
#   in_progress       — 사용자가 [캘 시작] 누른 직후. 캡처 가능. [세션 종료] 누르면 다음 단계로.
#   ready_for_analysis — 캡처 끝. immutable (더 이상 캡처 X). offline 스크립트 처리 대기.
#   success           — offline 스크립트 BA 끝 + Result row 들 INSERT. 완료.
#   failed            — 사용자 reset 또는 분석 실패 (현재는 거의 안 씀, future-proof).
CalibrationRunStatus = Literal[
    "in_progress", "ready_for_analysis", "success", "failed"
]


# ─── Run ──────────────────────────────────────────────────────


class CalibrationRunRecord(StrictModel):
    """한 번의 캘 실행 이력 (immutable).

    한 Run 이 여러 Result 만들 수 있음 (예: 확장 BA → hand_eye + joint + link +
    sag 동시 산출). algorithm 은 'extended_ba_irls' 등 식별자.

    `kind` 는 run 의 *목적* (사용자가 누른 버튼). 예: hand_eye run 은 4 kind result
    만들지만 kind='hand_eye'. in_progress run 의 robot/kind lookup 용. None=legacy.
    """

    # id 는 storage 가 부여 (INSERT 후 채워짐). insert 호출 시 None.
    id: int | None = None
    robot_id: str
    started_at: float  # epoch seconds
    ended_at: float | None = None
    operator: str | None = None
    note: str | None = None
    algorithm: str
    algorithm_params: dict[str, Any] = {}
    status: CalibrationRunStatus = "success"
    kind: CalibrationKind | None = None  # run 의 목적 (intrinsic / hand_eye). draft lookup 용.


# ─── Result — discriminated union ─────────────────────────────


class _ResultRecordBase(StrictModel):
    """공통 컬럼 — kind / result_data 만 sub-class 가 차이."""

    id: int | None = None
    run_id: int  # FK → calibration_runs.id
    robot_id: str
    created_at: float  # epoch seconds
    is_active: bool = False
    # σ 두 종류 ([[project-calibration-sigma-dual-metric]]):
    #   sigma_*           = BA Jacobian σ (parameter confidence, (JᵀJ)⁻¹·σ²)
    #   effective_sigma_* = effective σ (accuracy, board_in_base std) — commit 결정 metric
    # joint_offset / link_offset / sag 등 σ 무관 kind 자리는 둘 다 None.
    sigma_rot: float | None = None
    sigma_t: float | None = None
    effective_sigma_rot: float | None = None
    effective_sigma_t: float | None = None


class HandEyeResultRecord(_ResultRecordBase):
    kind: Literal["hand_eye"] = "hand_eye"
    result_data: HandEyeResultData


class IntrinsicResultRecord(_ResultRecordBase):
    kind: Literal["intrinsic"] = "intrinsic"
    result_data: IntrinsicResultData


class JointOffsetResultRecord(_ResultRecordBase):
    kind: Literal["joint_offset"] = "joint_offset"
    result_data: JointOffsetResultData


class LinkOffsetResultRecord(_ResultRecordBase):
    kind: Literal["link_offset"] = "link_offset"
    result_data: LinkOffsetResultData


class SagOffsetResultRecord(_ResultRecordBase):
    kind: Literal["sag"] = "sag"
    result_data: SagOffsetResultData


# Pydantic 의 `Field(discriminator="kind")` — `kind` 컬럼 값 보고 자동으로 알맞은
# arm 골라 validate. 잘못된 (kind, result_data) 조합은 ValidationError. drift 차단.
CalibrationResultRecord = Annotated[
    HandEyeResultRecord
    | IntrinsicResultRecord
    | JointOffsetResultRecord
    | LinkOffsetResultRecord
    | SagOffsetResultRecord,
    Field(discriminator="kind"),
]

# TypeAdapter — 외부 dict (SQL row → JSON parse 결과 등) 를 union 으로 validate.
# CalibrationRepo 의 row → record 변환에서 사용.
CalibrationResultRecordAdapter: TypeAdapter[CalibrationResultRecord] = TypeAdapter(
    CalibrationResultRecord
)


# ─── Capture (Evidence) ───────────────────────────────────────


CalibrationArtifactKind = Literal["primary", "color", "depth", "depth_vis", "ply"]


class CalibrationCaptureArtifactRecord(StrictModel):
    """Capture 1장의 ObjectStore blob 1개 — primary .bin 또는 디버깅 artifact.

    별도 정규화 테이블 `calibration_capture_artifacts`. `kind` 자리:
      - "primary"  — `depth_frame.py` encode 결과 (color JPEG + zstd Z16 depth +
                     header). offline 분석 자리 입력.
      - "color"    — color.jpg. 탐색기 viewable.
      - "depth"    — depth.png 16-bit raw. ImageJ 자리.
      - "depth_vis"— 8-bit colorized depth.png. 사람용.
      - "ply"      — binary color point cloud. CloudCompare/MeshLab 자리.
    """

    id: int | None = None
    capture_id: int  # FK → calibration_captures.id
    kind: CalibrationArtifactKind
    blob_key: str  # ObjectStore key — handler 가 결정
    size_bytes: int | None = None
    content_type: str | None = None  # e.g. "image/jpeg" / "application/octet-stream"
    created_at: float  # epoch seconds


class CalibrationCaptureRecord(StrictModel):
    """Evidence — per-pose 자세 정보 + raw sensor 데이터 캐시.

    drift-free 저장 — `motor_positions` 만이 robot 측 SSOT. joint_angles (rad) 는
    캡처 시점 캘에 잠겨버리니까 저장 안 함 (offline 분석 시 raw → rad 재계산).

    Blob (primary .bin + 디버깅 artifact) 는 정규화 별도 테이블
    `calibration_capture_artifacts` 에 — `artifacts` 자리 list 로 동봉.

    BA output (residual_rot/trans/weight) 는 offline 스크립트가 finalize 시 UPDATE.
    """

    id: int | None = None
    run_id: int  # FK → calibration_runs.id
    pose_index: int
    # raw motor positions — `motor_id → raw int (0..4095)`. drift-free SSOT.
    # intrinsic 캡처는 dict 무관 (joint 무용) — empty dict 또는 None 허용.
    motor_positions: dict[int, int] | None = None
    # PnP 결과 4x4 matrix (board_in_cam) — 재현용 캐시. offline 스크립트가 신뢰 안
    # 하면 corners_2d 로 재PnP 가능. intrinsic 캡처는 None.
    board_in_cam: list[list[float]] | None = None
    # ChArUco 검출 결과 캐시 — corners_2d (N,2) sub-pixel + corner_ids (N,).
    corners_2d: list[list[float]] | None = None
    corner_ids: list[int] | None = None
    # 캡처 시점 PnP 품질 (reprojection RMS, px) + 보드 tilt (deg). 진단 / outlier
    # 필터링용.
    reproj_rms_px: float | None = None
    tilt_deg: float | None = None
    # BA output — offline 스크립트가 finalize 시 채움.
    residual_rot: float | None = None
    residual_trans: float | None = None
    weight: float | None = None  # IRLS Huber weight (1.0 = 정상)
    # Artifacts — repo 가 selectinload 자리 채움. 비어있으면 capture 만 fetch 한 자리.
    artifacts: list[CalibrationCaptureArtifactRecord] = []

    def find_artifact(
        self, kind: CalibrationArtifactKind
    ) -> CalibrationCaptureArtifactRecord | None:
        """편의 — kind 의 artifact 자리 찾음. 없으면 None."""
        for a in self.artifacts:
            if a.kind == kind:
                return a
        return None

    @property
    def primary_blob_key(self) -> str | None:
        """backward-compat 편의 — primary artifact 의 blob_key."""
        a = self.find_artifact("primary")
        return a.blob_key if a is not None else None
